#!/usr/bin/env python3
# SeniorDesignProject/mpc_main.py
# Same UX as the old main: camera display, HDR switching, overlays, UPS/emergency,
# threshold hotkeys, exposure profiles... but with MPC in place of PID.

import cv2
import time
import numpy as np
from pathlib import Path

from modules import Config as C
from modules.LightingAnalysis import CameraManager, LightingAnalyzer
from modules.DualPicam import PicamFeed  # (kept for compatibility; used via HDRView helpers)
from modules.UPS import get_ups_data
from modules.ModeSwitches import ModeSwitches
from modules.OverlayRenderer import draw_analysis
from modules.HDRView import ensure_active as hdr_ensure_active, capture_frame as hdr_capture_frame, show as hdr_show
from modules.EmergencyController import EmergencyController
from modules.ExposureControl import apply_profile, cycle_profile, set_exposure_auto, set_exposure_manual, PROFILES, PROFILE_ORDER
from modules.mpc_controller import MPCController

# # add temporarily near top (after imports):
# from time import sleep

#def _blink_mapping_once():
 #   names = ["HF1","HF2","HS11","HS12","HS21","HS22","LF1","LF2","LS1","LS2"]
  #  vec = np.zeros(10)
   # for i, n in enumerate(names):
    #    vec[:] = 0
     #   vec[i] = 60  # small visible step
      #  dac_apply_vec(vec)
       # print("Blink:", n)
        #sleep(0.4)
    #dac_zero_all()

# then call it once at startup (before the main loop), verify visually, then remove:
# _blink_mapping_once()



# --- NEW: MPC controller + DAC I/O ---
from spi_dac import apply_vec as dac_apply_vec, vec_to_dict as dac_vec_to_dict, zero_all as dac_zero_all

# === IMPORTANT ===
# Set this list to the EXACT order of the MPC inputs (columns of B).
# If your artifact differs, edit this to match your mpc_model.npz.
MODEL_INPUT_NAMES = ["HF1","HF2","HS11","HS12","HS21","HS22","LF1","LF2","LS1","LS2"]

def clamp(val, lo, hi):
    return max(lo, min(hi, val))

# AFTER
def _build_reference(y_set: float, ny: int, N: int) -> np.ndarray:
    r1 = np.full((ny,), float(y_set))   # (ny,)
    return np.tile(r1.reshape(ny, 1), (1, N))  # (ny, N)
    
def main():
    # ---------- Hardware switches & cameras (unchanged behavior) ----------
    switches = ModeSwitches(auto_pin=C.AUTO_PIN, dn_pin=C.DN_PIN, dn_debounce_ms=C.DN_DEBOUNCE_MS)
    analysis_cam = CameraManager(camera_index=C.ANALYSIS_CAMERA_INDEX)

    active_name = C.HDR_DAY_NAME
    hdr_cam_primary = None
    hdr_cam_alt = None

    analyzer = LightingAnalyzer(threshold_value=C.THRESHOLD_DARK, rows=3, cols=2)

    emer = EmergencyController()
    prev_auto_mode = None  # track for just_switched_to_auto

    dark_cutoff = int(clamp(C.THRESHOLD_DARK, 0, 100))  # percent 0..100

    current_profile = PROFILE_ORDER[0]
    apply_profile(current_profile, analyzer)
    exposure_locked = (PROFILES[current_profile]["exposure"] == "manual")

    dn_state_initial, _ = switches.read_dn_debounced()
    last_dn_high = dn_state_initial
    desired_active = C.HDR_DAY_NAME if last_dn_high else C.HDR_NIGHT_NAME
    active_name, hdr_cam_primary, hdr_cam_alt = hdr_ensure_active(
        desired_active, active_name, hdr_cam_primary, hdr_cam_alt, settle_s=C.SWITCH_SETTLE_S
    )
    switch_in_progress = False

    # ---------- NEW: MPC setup ----------
    model_path = Path(__file__).parent / "build" / "mpc_model.npz"
    if not model_path.exists():
        raise FileNotFoundError(f"[MPC] Model file missing: {model_path}")
    data = np.load(model_path, allow_pickle=True)
    A, B = data["A"], data["B"]
    Ts = float(data.get("Ts", C.LOOP_SLEEP_S if C.LOOP_SLEEP_S > 0 else 0.05))
    ny, nu = A.shape[0], B.shape[1]
    assert ny == 6, f"[MPC] Expected 6 outputs (S11..S32); got ny={ny}"
    assert nu == len(MODEL_INPUT_NAMES), f"[MPC] B has {nu} cols but MODEL_INPUT_NAMES has {len(MODEL_INPUT_NAMES)}"

    # Tunings (start conservative; adjust after first live test)
    N  = 5                       # horizon
    Qd = np.ones(ny) * 5.0       # output tracking weights
    Rd = np.ones(nu) * 0.1       # control effort weights
    u_min = np.zeros(nu)
    u_max = np.ones(nu) * 255.0
    du_max = 8.0                 # DAC code slew per step
    mpc = MPCController(
        A, B,
        N=N, Q_diag=Qd, R_diag=Rd,
        u_min=u_min, u_max=u_max,
        du_max=du_max,
        lam=0.0,
        p_coeffs=None, P_max=None,
        slack_penalty=1e6
    )

    # Reference built from the analyzer threshold (same semantics as old main)
    y_set = float(dark_cutoff)
    r_traj = _build_reference(y_set, ny, N)
    u_prev = np.zeros(nu)  # last MPC command vector (nu=10)

    # We'll maintain a 'dac' dict for overlay compatibility (same as old main)
    dac_dict = {name: 0 for name in MODEL_INPUT_NAMES}

    print("Starting multi-light control (MPC).")
    print("Hotkeys: q=quit  [ ]=brightness threshold  -=/+= darkness cutoff  \\=global/local-Otsu  E=exposure  P=profile")

    try:
        while True:
            states = switches.read_states()
            auto_mode = states["auto_mode"]
            dn_state = states["dn_state"]
            dn_changed = states["dn_changed"]

            # ---- Analysis frame & darkness measurement (unchanged) ----
            frame = analysis_cam.capture_frame()
            if frame is not None:
                overall_dark, cell_dark, img = analyzer.analyze(frame)  # cell_dark order: [S11,S12,S21,S22,S31,S32]
                y_k = np.asarray(cell_dark, dtype=float)                # <- MPC output vector (ny=6)
            else:
                img = None
                y_k = np.zeros(ny, dtype=float)

            # ---- Control law: MPC (replaces PID) ----
            if auto_mode and not emer.emergency:
                # Solve MPC for new command vector
                u_cmd, status, solve_ms, cost_val = mpc.compute(y0=y_k, u_prev=u_prev, r_traj=r_traj)

                # Clamp and write to DACs in the physical/driver order expected by spi_dac
                # Here we assume spi_dac uses the same order as MODEL_INPUT_NAMES; if not,
                # re-order u_cmd into spi order before apply_vec.
                u_cmd = np.clip(u_cmd, 0, 255)

                # Apply to hardware
                try:
                    dac_apply_vec(u_cmd)  # writes all 10 channels
                except Exception as e:
                    print(f"[SPI] write error: {e}")

                # For overlay: convert to dict keyed by light names
                dac_dict = {name: int(round(val)) for name, val in zip(MODEL_INPUT_NAMES, u_cmd.tolist())}
                u_prev = u_cmd.copy()

            else:
                # Manual or emergency: zero outputs (same UX as old main)
                if prev_auto_mode in (True, None) or emer.emergency:
                    u_prev[:] = 0.0
                try:
                    dac_zero_all()
                except Exception:
                    pass
                for k in dac_dict.keys():
                    dac_dict[k] = 0

            # ---- Draw overlay (unchanged) ----
            if img is not None:
                draw_analysis(img, dac_dict, auto_mode)
                t_val = analyzer.get_threshold()
                mode_str = getattr(analyzer, "threshold_mode", "global")
                cv2.putText(
                    img,
                    f"T={t_val}  MODE={mode_str}  DARK={dark_cutoff}%  PROFILE={current_profile}",
                    (10, img.shape[0] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (255, 255, 255), 2
                )
                cv2.imshow(C.WINDOW_ANALYSIS, img)

            # ---- HDR switching (unchanged) ----
            if not switch_in_progress and dn_changed:
                switch_in_progress = True
                desired_active = C.HDR_DAY_NAME if dn_state else C.HDR_NIGHT_NAME
                active_name, hdr_cam_primary, hdr_cam_alt = hdr_ensure_active(
                    desired_active, active_name, hdr_cam_primary, hdr_cam_alt, settle_s=C.SWITCH_SETTLE_S
                )
                last_dn_high = dn_state
                time.sleep(C.SWITCH_SETTLE_S)
                switch_in_progress = False

            hdr_src = hdr_cam_primary if active_name == C.HDR_DAY_NAME else hdr_cam_alt
            hdr_frame = hdr_capture_frame(hdr_src)
            if hdr_frame is not None:
                ups = get_ups_data()
                current_status = ups.get("ups.status", "Unknown")

                # Determine whether AUTO was just switched on (handles startup case)
                just_switched_to_auto = ((prev_auto_mode is False or prev_auto_mode is None) and auto_mode)

                # Evaluate emergency state transition (unchanged)
                emergency_mode, event = emer.eval_transition(
                    current_status,
                    auto_mode=auto_mode,
                    just_switched_to_auto=just_switched_to_auto
                )

                # Handle emergency events (unchanged messaging/behavior)
                if event == "enter_returned_to_auto":
                    print("Returned to AUTO while UPS is already On Battery. ENTERING EMERGENCY MODE.")
                    u_prev[:] = 0.0
                elif event == "enter_online_to_onbatt":
                    print("UPS transitioned Online -> On Battery. ENTERING EMERGENCY MODE.")
                    u_prev[:] = 0.0
                elif event == "enter_boot_on_battery":
                    print("System booted in AUTO while UPS is On Battery. ENTERING EMERGENCY MODE.")
                    u_prev[:] = 0.0
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

            # ---- Hotkeys (unchanged semantics) ----
            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 13, 10):
                print("Exiting...")
                try:
                    dac_zero_all()
                except Exception:
                    pass
                break
            elif key == ord('['):
                analyzer.adjust_threshold(-5)
                # Keep reference aligned with visual threshold semantics
                y_set = float(analyzer.get_threshold())
                r_traj = _build_reference(y_set, ny, N)
            elif key == ord(']'):
                analyzer.adjust_threshold(+5)
                y_set = float(analyzer.get_threshold())
                r_traj = _build_reference(y_set, ny, N)
            elif key == ord('-'):
                dark_cutoff = clamp(dark_cutoff - 2, 0, 100)
                print(f"[Policy] darkness cutoff → {dark_cutoff}%")
                y_set = float(dark_cutoff)
                r_traj = _build_reference(y_set, ny, N)
            elif key in (ord('='), ord('+')):
                dark_cutoff = clamp(dark_cutoff + 2, 0, 100)
                print(f"[Policy] darkness cutoff → {dark_cutoff}%")
                y_set = float(dark_cutoff)
                r_traj = _build_reference(y_set, ny, N)
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
        # ---- Safe shutdown (same UX) ----
        try:
            dac_zero_all()
        except Exception:
            pass
        try:
            analysis_cam.release()
        except Exception:
            pass
        try:
            if hdr_cam_primary:
                hdr_cam_primary.release()
        except Exception:
            pass
        try:
            if hdr_cam_alt:
                hdr_cam_alt.release()
        except Exception:
            pass
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        switches.cleanup()

if __name__ == "__main__":
    main()
