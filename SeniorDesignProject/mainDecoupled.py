#!/usr/bin/env python3
# main_decoupled.py
#
# Drop-in main that preserves your UI/UX (camera, overlays, switches, UPS HUD,
# exposure profiles, hotkeys) but uses *decoupled lighting control*:
#   - Loads decoupler_M.csv (10x6)  -> M
#   - Loads pid_gains.json (6 sets) -> per-sector PID
#   - Each loop: measure y (6 %dark), e = y - r0, Δv = PID(e), Δu = M·Δv
#   - Apply Δu to the 10 DAC channels with per-step slew limiting and flood shaping
#
# Files expected in the working directory (or adjust the constants below):
#   ./decoupler_M.csv
#   ./pid_gains.json
#
# Keeps your existing interfaces and modules:
#   Config, LightingAnalysis, ModeSwitches, OverlayRenderer, HDRView,
#   EmergencyController, ExposureControl; DAC writes go through PIDPolicy._push_all
#   so flood shaping remains active. (Same analyzer/setpoint/overlay inputs.)
#
# Hotkeys (unchanged):
#   q : quit
#   [ / ] : adjust analyzer threshold (brightness) in GLOBAL mode
#   - / = : adjust global darkness setpoint (target % dark)
#   \ : toggle GLOBAL <-> local_otsu thresholding
#   e : toggle exposure auto/manual for current profile
#   p : cycle exposure profile

import os
import csv
import json
import cv2
import time
import math
import numpy as np

from modules import Config as C                                 # settings (camera index, names, loop timing, etc.)
from modules.LightingAnalysis import CameraManager, LightingAnalyzer  # camera + 3x2 %dark analyzer
from modules.DualPicam import PicamFeed                         # HDR CSI feeds (allocated lazily)
from modules.UPS import get_ups_data                            # UPS HUD
from modules.ModeSwitches import ModeSwitches                   # AUTO/MANUAL + Day/Night switches
from modules.OverlayRenderer import draw_analysis               # DAC overlay on analysis window
from modules.HDRView import ensure_active as hdr_ensure_active, capture_frame as hdr_capture_frame, show as hdr_show
from modules.EmergencyController import EmergencyController     # UPS edge logic -> EMERGENCY mode
from modules.ExposureControl import apply_profile, cycle_profile, set_exposure_auto, set_exposure_manual, PROFILES, PROFILE_ORDER
# We still reuse PIDPolicy for: init_state() (builds DAC dict), and its *hardware push* function (flood shaping + DAC write).
from modules.PIDPolicy import init_state                        # returns {light_name: dac}
import modules.PIDPolicy as PIDPolicy                           # access CHANNEL_MAP and _push_all (hardware write with FloodMap shaping)

# ------------------------------- User paths (edit if needed) -------------------------------
DECOUPLER_CSV = os.environ.get("DECOUPLER_CSV", "decoupler_M.csv")
PID_GAINS_JSON = os.environ.get("PID_GAINS_JSON", "pid_gains.json")

# ------------------------------- Helpers: load M and gains -------------------------------
def _canonical_sectors():
    # Analyzer returns [S11,S12,S21,S22,S31,S32] in that order.
    return ["S11","S12","S21","S22","S31","S32"]

def load_decoupler(csv_path: str):
    """
    Robust CSV reader for decoupler_M.csv written by build_decoupler.m.
    Expects a header row with sector names (S11..S32) and the first column as light_name row labels.
    Returns:
      M           : (10x6) numpy array (float64)
      light_names : list[str] length 10 (row order of M)
      sector_names: list[str] length 6  (column order of M)
    """
    if not os.path.exists(csv_path):
        raise FileNotFoundError(f"decoupler CSV not found: {csv_path}")

    with open(csv_path, "r", newline="") as f:
        rdr = csv.reader(f)
        rows = list(rdr)

    if not rows or len(rows) < 2:
        raise ValueError("decoupler CSV appears empty or malformed")

    header = [h.strip() for h in rows[0]]
    # If first header cell is a label (e.g., '' or 'RowNames'), sectors start at col 2; otherwise they start at col 1.
    # We prefer canonical sector names if present; otherwise fall back to canonical order.
    hdr_first = (header[0] or "").upper()
    if hdr_first in ("", "ROWNAMES", "LIGHT", "NAME", "LIGHT_NAME"):
        sector_names = [h.strip() for h in header[1:]]
        col0_is_label = True
    else:
        # Many MATLAB exports put row labels in the first column anyway; try header[1:] first
        maybe = [h.strip() for h in header[1:]]
        if all(len(s)>=2 for s in maybe):
            sector_names = maybe
            col0_is_label = True
        else:
            sector_names = header  # header contains sector names directly
            col0_is_label = False

    # Normalize sector names to canonical if they look like Var1..Var6 or have been sanitized
    canon = _canonical_sectors()
    def _as_canon(s):
        s = s.upper().strip()
        for c in canon:
            if s == c or s.endswith(c) or c in s:
                return c
        return s
    sector_names = [ _as_canon(s) for s in sector_names ]
    # If the columns are not six canonical sectors, force canonical order and hope numeric columns are in that order
    if set(sector_names) != set(canon):
        sector_names = canon[:]  # fallback

    light_names = []
    data = []
    for r in rows[1:]:
        if not r or len(r) < 2:
            continue
        if col0_is_label:
            light_names.append(r[0].strip().upper())
            nums = r[1:]
        else:
            # No explicit row label; synthesize light names as Var#
            light_names.append(f"Var{len(light_names)+1}")
            nums = r
        # keep only first 6 numeric entries (extra columns may come from export indices)
        vals = []
        for x in nums[:6]:
            try:
                vals.append(float(x))
            except Exception:
                vals.append(0.0)
        data.append(vals)

    M = np.asarray(data, dtype=float)
    if M.shape[1] != 6:
        raise ValueError(f"Decoupler columns != 6 (got {M.shape})")
    # If there are extra rows, we keep the first 10; if fewer, that's a fatal mismatch.
    if M.shape[0] < 6:
        raise ValueError(f"Decoupler rows too few: {M.shape[0]}")
    return M, light_names, sector_names

def load_pid_gains(json_path: str, sector_order: list[str]):
    """
    Flexible reader for pid_gains.json produced by prepare_controller_artifacts.m.
    Accepts several shapes:
      - {"S11":{"Kp":..,"Ki":..,"Kd":..}, ...}
      - {"gains":{"S11":{...}, ...}, "Ts":..., "delay":...}
      - [{"sector":"S11","Kp":..,"Ki":..,"Kd":..}, ...]
    Returns an ordered list of (Kp,Ki,Kd) in sector_order.
    """
    if not os.path.exists(json_path):
        raise FileNotFoundError(f"PID gains JSON not found: {json_path}")

    with open(json_path, "r") as f:
        obj = json.load(f)

    # Normalize to dict[str]->dict with Kp/Ki/Kd
    by_name = {}
    if isinstance(obj, dict):
        if "gains" in obj and isinstance(obj["gains"], dict):
            by_name = obj["gains"]
        else:
            # if keys look like sector names, assume this is the map
            keys = [k.upper() for k in obj.keys()]
            if any(k in keys for k in ["S11","S12","S21","S22","S31","S32"]):
                by_name = obj
            elif "sectors" in obj and isinstance(obj["sectors"], list):
                for entry in obj["sectors"]:
                    s = str(entry.get("sector","")).upper()
                    if s:
                        by_name[s] = entry
    elif isinstance(obj, list):
        for entry in obj:
            s = str(entry.get("sector","")).upper()
            if s:
                by_name[s] = entry

    gains = []
    for s in sector_order:
        entry = by_name.get(s, {})
        # be robust to case and key variants
        def _g(k):
            for kk in (k, k.lower(), k.upper(), "K"+k, "k"+k):
                if kk in entry:
                    return float(entry[kk])
            return None
        Kp = _g("p") or _g("Kp") or 0.6
        Ki = _g("i") or _g("Ki") or 0.1
        Kd = _g("d") or _g("Kd") or 0.0
        gains.append((float(Kp), float(Ki), float(Kd)))
    return gains

# ------------------------------- Simple per-sector PID (velocity form) -------------------------------
class SectorPID:
    """
    Discrete PID in *velocity* form: returns Δv per call.
    We integrate on the plant input (DAC via M) rather than on the measurement.
    Anti-windup: when Δu or u saturates, bleed the I term.
    """
    def __init__(self, Kp, Ki, Kd, out_step_limit=8.0, deadband=0.5):
        self.Kp = float(Kp)
        self.Ki = float(Ki)
        self.Kd = float(Kd)
        self.db = float(deadband)
        self.step_limit = float(out_step_limit)
        self.i = 0.0
        self.prev_e = 0.0
        self.initialized = False

    def step(self, e: float, dt: float) -> float:
        # deadband around setpoint
        if abs(e) < self.db:
            e_eff = 0.0
        else:
            e_eff = e

        if not self.initialized:
            self.prev_e = e_eff
            self.initialized = True

        # velocity-form PID (Δv); dt guards
        p = self.Kp * (e_eff - self.prev_e)
        self.i += self.Ki * e_eff * dt
        d = self.Kd * ( (e_eff - self.prev_e) / max(dt, 1e-3) )

        dv = p + self.i + d
        # instantaneous step clamp
        if dv > self.step_limit:
            dv = self.step_limit
            self.i *= 0.8
        elif dv < -self.step_limit:
            dv = -self.step_limit
            self.i *= 0.8

        self.prev_e = e_eff
        return float(dv)

    def reset(self):
        self.i = 0.0
        self.prev_e = 0.0
        self.initialized = False

# ------------------------------- Utilities: DAC dict <-> vector -------------------------------
def dict_to_vec(d: dict, names: list[str]) -> np.ndarray:
    return np.array([int(d.get(k, 0)) for k in names], dtype=float)

def vec_to_dict(v: np.ndarray, names: list[str]) -> dict:
    return {names[i]: int(max(0, min(255, int(round(v[i]))))) for i in range(len(names))}

# ------------------------------- Main program -------------------------------
def clamp(val, lo, hi): return max(lo, min(hi, val))

def main():
    # --- Load decoupler (M) and PID gains ---
    try:
        M, M_light_names, M_sector_names = load_decoupler(DECOUPLER_CSV)
        print(f"[decoupler] M loaded: {M.shape[0]} lights x {M.shape[1]} sectors")
        print(f"[decoupler] rows (lights): {M_light_names}")
        print(f"[decoupler] cols (sectors): {M_sector_names}")
    except Exception as e:
        print(f"FATAL: failed to load decoupler_M.csv: {e}")
        return

    canon_sectors = _canonical_sectors()
    # Reorder columns of M to canonical sector order if needed
    if set(M_sector_names) == set(canon_sectors) and M_sector_names != canon_sectors:
        col_idx = [M_sector_names.index(s) for s in canon_sectors]
        M = M[:, col_idx]
        M_sector_names = canon_sectors[:]

    try:
        gains = load_pid_gains(PID_GAINS_JSON, canon_sectors)
        print("[pid] Loaded sector gains (Kp,Ki,Kd) in S11..S32 order:")
        for s, g in zip(canon_sectors, gains):
            print(f"  {s}: {g}")
    except Exception as e:
        print(f"[pid] WARNING: could not load pid_gains.json ({e}); using conservative defaults.")
        gains = [(0.6, 0.1, 0.0)] * 6

    # Build six per-sector velocity-form PIDs
    sector_pid = [SectorPID(Kp, Ki, Kd, out_step_limit=8.0, deadband=0.5) for (Kp,Ki,Kd) in gains]

    # --- Switches and analysis camera (same as your main) ---
    switches = ModeSwitches(auto_pin=C.AUTO_PIN, dn_pin=C.DN_PIN, dn_debounce_ms=C.DN_DEBOUNCE_MS)
    analysis_cam = CameraManager(camera_index=C.ANALYSIS_CAMERA_INDEX)

    # HDR feeds lazy-init & ensure the right one based on DN switch
    active_name = C.HDR_DAY_NAME
    hdr_cam_primary = None
    hdr_cam_alt = None

    analyzer = LightingAnalyzer(threshold_value=C.THRESHOLD_DARK, rows=3, cols=2)

    # --- DAC state dict from your policy module (same mapping & flood shaping path) ---
    dac = init_state()  # e.g., {"HS11":0, ..., "LF2":0}
    # Align M row order to the DAC dict if possible (prefer M's canonical tokens)
    # If any M_light_names are not present in the DAC dict, we still keep their rows but they will be ignored on push.
    light_order = []
    for name in M_light_names:
        light_order.append(name if name in dac else name)  # keep as-is; vec_to_dict will set only existing keys
    # Current DAC vector in M row order
    u_vec = dict_to_vec(dac, light_order)

    # Global setpoint (shared across sectors) and loop timing
    dark_cutoff = int(clamp(C.THRESHOLD_DARK, 0, 100))
    last_time = time.monotonic()

    # Exposure profile handling (same UX)
    current_profile = PROFILE_ORDER[0]
    apply_profile(current_profile, analyzer)
    exposure_locked = (PROFILES[current_profile]["exposure"] == "manual")

    # Day/Night HDR selection (same logic)
    dn_state_initial, _ = switches.read_dn_debounced()
    last_dn_high = dn_state_initial
    desired_active = C.HDR_DAY_NAME if last_dn_high else C.HDR_NIGHT_NAME
    active_name, hdr_cam_primary, hdr_cam_alt = hdr_ensure_active(
        desired_active, active_name, hdr_cam_primary, hdr_cam_alt, settle_s=C.SWITCH_SETTLE_S
    )
    switch_in_progress = False

    emer = EmergencyController()
    prev_auto_mode = None

    print("Starting decoupled lighting control (PID-on-decoupled channels).")
    print("Hotkeys: q=quit  [ ]=brightness  -=,+=darkness  \\=Otsu  e=exposure  p=profile")

    try:
        while True:
            now = time.monotonic()
            dt = max(1e-3, now - last_time)
            last_time = now

            # Read switches
            states = switches.read_states()
            auto_mode  = states["auto_mode"]
            dn_state   = states["dn_state"]
            dn_changed = states["dn_changed"]

            # Acquire analysis frame
            frame = analysis_cam.capture_frame()
            if frame is not None:
                overall_dark, cell_dark, img = analyzer.analyze(frame)  # cell_dark is [S11..S32] floats (0..100)

                if auto_mode:
                    # ---------- EMERGENCY OVERRIDE ----------
                    if emer.emergency:
                        # All off except LF2=255 (hardware path through PIDPolicy keeps flood shaping)
                        for k in dac.keys():
                            dac[k] = 0
                        dac["LF2"] = 255
                        PIDPolicy._push_all(dac, PIDPolicy.CHANNEL_MAP)
                    else:
                        # ---------- NORMAL (DECOUPLED) CONTROL ----------
                        # 1) Per-sector PID -> Δv (six decoupled channels)
                        #    error = measured - setpoint (positive => too dark -> brighten)
                        e = np.array([float(cell_dark[i]) - float(dark_cutoff) for i in range(6)], dtype=float)
                        dv = np.zeros(6, dtype=float)
                        for i in range(6):
                            dv[i] = sector_pid[i].step(e[i], dt)  # Δv_i

                        # 2) Map Δv to actuator space: Δu = M · Δv
                        du = M.dot(dv)  # shape (10,)

                        # 3) Slew limit per actuator (behaves like your per-step limiter)
                        #    Keep instantaneous change small to avoid flicker / saturation swings.
                        MAX_DU_PER_STEP = 12.0
                        du = np.clip(du, -MAX_DU_PER_STEP, +MAX_DU_PER_STEP)

                        # 4) Integrate and clamp to DAC range
                        u_vec = u_vec + du
                        u_vec = np.clip(u_vec, 0.0, 255.0)

                        # 5) Push to hardware (FloodMap shaping happens inside PIDPolicy._push_all)
                        dac.update(vec_to_dict(u_vec, light_order))
                        PIDPolicy._push_all(dac, PIDPolicy.CHANNEL_MAP)

                else:
                    # MANUAL: zero outputs and reset integrators once on entry
                    if prev_auto_mode in (True, None):
                        for ctl in sector_pid: ctl.reset()
                        for k in dac.keys(): dac[k] = 0
                        PIDPolicy._push_all(dac, PIDPolicy.CHANNEL_MAP)
                        # Reset the "state" vector to reflect what's on hardware
                        u_vec = dict_to_vec(dac, light_order)

                # Overlay: show DACs + MODE (same as your current overlay)
                draw_analysis(img, dac, auto_mode)
                # HUD: analyzer threshold, mode, DARK setpoint, profile, policy label
                try:
                    t_val = analyzer.get_threshold()
                except Exception:
                    t_val = getattr(analyzer, "threshold_value", 0)
                mode_str = getattr(analyzer, "threshold_mode", "global")
                cv2.putText(
                    img,
                    f"T={t_val}  MODE={mode_str}  DARK={dark_cutoff}%  PROFILE={current_profile}  POLICY=DEC+PID",
                    (10, img.shape[0] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (255, 255, 255), 2
                )
                cv2.imshow(C.WINDOW_ANALYSIS, img)

            # Day/Night HDR switching (same behavior)
            if not switch_in_progress and dn_changed:
                switch_in_progress = True
                desired_active = C.HDR_DAY_NAME if dn_state else C.HDR_NIGHT_NAME
                active_name, hdr_cam_primary, hdr_cam_alt = hdr_ensure_active(
                    desired_active, active_name, hdr_cam_primary, hdr_cam_alt, settle_s=C.SWITCH_SETTLE_S
                )
                last_dn_high = dn_state
                time.sleep(C.SWITCH_SETTLE_S)
                switch_in_progress = False

            # HDR HUD + Emergency state machine (same logic)
            hdr_src = hdr_cam_primary if active_name == C.HDR_DAY_NAME else hdr_cam_alt
            hdr_frame = hdr_capture_frame(hdr_src)
            if hdr_frame is not None:
                ups = get_ups_data()
                current_status = ups.get("ups.status", "Unknown")
                just_switched_to_auto = (prev_auto_mode is False and auto_mode)
                emergency_mode, event = emer.eval_transition(
                    current_status,
                    auto_mode=auto_mode,
                    just_switched_to_auto=just_switched_to_auto
                )
                if event == "enter_returned_to_auto":
                    print("Returned to AUTO while UPS already On Battery. ENTERING EMERGENCY MODE.")
                    for ctl in sector_pid: ctl.reset()
                    for k in dac.keys(): dac[k] = 0
                    PIDPolicy._push_all(dac, PIDPolicy.CHANNEL_MAP)
                    u_vec = dict_to_vec(dac, light_order)
                elif event == "enter_online_to_onbatt":
                    print("UPS transitioned Online -> On Battery. ENTERING EMERGENCY MODE.")
                    for ctl in sector_pid: ctl.reset()
                    for k in dac.keys(): dac[k] = 0
                    PIDPolicy._push_all(dac, PIDPolicy.CHANNEL_MAP)
                    u_vec = dict_to_vec(dac, light_order)
                elif event == "exit_online":
                    print("UPS back Online. EXITING EMERGENCY MODE.")

                hdr_show(
                    hdr_frame,
                    active_name=active_name,
                    auto_mode=auto_mode,
                    ups_data=ups,
                    emergency_mode=emergency_mode
                )

            prev_auto_mode = auto_mode

            # Hotkeys (same set, minus MPC toggle)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 13, 10):
                print("Exiting...")
                # zero hardware on exit
                for k in dac.keys(): dac[k] = 0
                PIDPolicy._push_all(dac, PIDPolicy.CHANNEL_MAP)
                break

            elif key == ord('['):
                analyzer.adjust_threshold(-5)
            elif key == ord(']'):
                analyzer.adjust_threshold(+5)

            elif key == ord('-'):
                dark_cutoff = int(clamp(dark_cutoff - 2, 0, 100))
                print(f"[Setpoint] DARK → {dark_cutoff}%")
            elif key in (ord('='), ord('+')):
                dark_cutoff = int(clamp(dark_cutoff + 2, 0, 100))
                print(f"[Setpoint] DARK → {dark_cutoff}%")

            elif key == ord('\\'):
                cur = getattr(analyzer, "threshold_mode", "global")
                new = "local_otsu" if cur == "global" else "global"
                analyzer.set_threshold_mode(new)

            elif key == ord('e'):
                if not exposure_locked:
                    if set_exposure_manual(PROFILES[current_profile]["exposure_ms"],
                                           PROFILES[current_profile]["gain"]):
                        exposure_locked = True
                else:
                    if set_exposure_auto():
                        exposure_locked = False

            elif key == ord('p'):
                current_profile = cycle_profile(current_profile, analyzer)
                exposure_locked = (PROFILES[current_profile]["exposure"] == "manual")

            time.sleep(C.LOOP_SLEEP_S)

    finally:
        # Cleanup (same pattern)
        try:
            for k in dac.keys(): dac[k] = 0
            PIDPolicy._push_all(dac, PIDPolicy.CHANNEL_MAP)
        except Exception:
            pass
        try:
            analysis_cam.release()
        except Exception:
            pass
        try:
            if hdr_cam_primary: hdr_cam_primary.release()
        except Exception:
            pass
        try:
            if hdr_cam_alt: hdr_cam_alt.release()
        except Exception:
            pass
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        switches.cleanup()

if __name__ == "__main__":
    main()
