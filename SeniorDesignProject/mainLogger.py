#!/usr/bin/env python3
# mainLogger.py
import cv2
import time
import argparse
from pathlib import Path
from datetime import datetime
import csv

from modules import Config as C
from modules.LightingAnalysis import CameraManager, LightingAnalyzer
from modules.DualPicam import PicamFeed
from modules.UPS import get_ups_data
from modules.ModeSwitches import ModeSwitches
from modules.PIDPolicy import init_state, reset_all, pid_step_and_apply, PIDManager
from modules.OverlayRenderer import draw_analysis
from modules.HDRView import ensure_active as hdr_ensure_active, capture_frame as hdr_capture_frame, show as hdr_show
from modules.EmergencyController import EmergencyController
from modules.ExposureControl import apply_profile, cycle_profile, set_exposure_auto, set_exposure_manual, PROFILES, PROFILE_ORDER


def clamp(val, lo, hi):
    return max(lo, min(hi, val))


# ---------------------------
# CSV Logger with timestamps + exposure + PID gains
# ---------------------------
class RunLogger:
    """
    Logs darkness, DACs, timestamps, exposure, and PID coefficients (Kp, Ki, Kd).
    Robust to cell_dark being scalar/1-D/2-D and numpy/Python lists.
    """
    def __init__(self, analyzer, dac_keys, pid_gains=None, pid_obj=None, log_dir="./logs", filename_prefix="PID_LOG"):
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.path = Path(log_dir) / f"{filename_prefix}_{ts}.csv"
        self.path.parent.mkdir(parents=True, exist_ok=True)

        self.rows = getattr(analyzer, "rows", 3)
        self.cols = getattr(analyzer, "cols", 2)
        self.dac_keys = sorted(list(dac_keys)) if dac_keys else []

        # Determine PID gains (prefer passed-in values; else peek at first internal PID)
        kp = ki = kd = None
        if pid_gains and len(pid_gains) == 3:
            kp, ki, kd = pid_gains
        elif pid_obj is not None:
            try:
                first = next(iter(pid_obj.pids.values()))
                kp, ki, kd = getattr(first, "kp", None), getattr(first, "ki", None), getattr(first, "kd", None)
            except Exception:
                pass
        self.kp, self.ki, self.kd = kp, ki, kd

        # Open file + writer
        self.fh = open(self.path, "w", newline="")
        self.writer = csv.writer(self.fh)

        # Metadata (MATLAB readmatrix ignores # lines)
        self._meta(f"started_utc={datetime.utcnow().isoformat()}Z")
        self._meta(f"rows={self.rows}")
        self._meta(f"cols={self.cols}")
        self._meta(f"PID_Kp={self.kp}")
        self._meta(f"PID_Ki={self.ki}")
        self._meta(f"PID_Kd={self.kd}")
        self._meta("time_notes=t_iso_utc is UTC; t_unix_s is epoch seconds; monotonic_ns is high-res monotonic clock")

        # Header
        base = [
            # high-quality timing
            "t_iso_utc", "t_unix_s", "monotonic_ns", "loop_dt_ms", "frame_dt_ms",
            # run state
            "loop_idx", "auto_mode", "dn_state", "profile",
            "exposure_locked", "exposure_ms", "exposure_gain",
            # PID coefficients
            "PID_Kp", "PID_Ki", "PID_Kd",
            # analyzer/policy
            "threshold_mode", "threshold_value", "dark_cutoff", "overall_dark"
        ]
        sectors = [f"S{r}{c}_dark" for r in range(self.rows) for c in range(self.cols)]
        dacs = [f"DAC_{k}" for k in self.dac_keys]
        self.header = base + sectors + dacs
        self.writer.writerow(self.header)
        self.fh.flush()
        self.loop_idx = 0

    def _meta(self, text):
        self.fh.write(f"# {text}\n")

    def _as_int_pct(self, x):
        try:
            return int(round(float(x)))
        except Exception:
            return "NA"

    def _flatten_cells(self, cell_dark):
        """
        Return a row-major list of length rows*cols.
        Accepts scalar/1-D/2-D lists or numpy arrays. Pads/trims to length.
        """
        try:
            import numpy as np
            if isinstance(cell_dark, np.ndarray):
                flat = cell_dark.reshape(-1).tolist()
                flat = [self._as_int_pct(v) for v in flat]
            else:
                raise TypeError
        except Exception:
            # Not numpy ndarray — handle Python sequences/scalars
            if isinstance(cell_dark, (int, float)):
                return [self._as_int_pct(cell_dark)] * (self.rows * self.cols)
            try:
                # 2-D?
                if len(cell_dark) > 0 and hasattr(cell_dark[0], "__len__") and not isinstance(cell_dark[0], (int, float)):
                    flat = []
                    for row in cell_dark:
                        try:
                            for v in row:
                                flat.append(self._as_int_pct(v))
                        except TypeError:
                            flat.append(self._as_int_pct(row))
                else:
                    # 1-D
                    flat = [self._as_int_pct(v) for v in cell_dark]
            except Exception:
                return [self._as_int_pct(cell_dark)] * (self.rows * self.cols)

        need = self.rows * self.cols
        if len(flat) >= need:
            return flat[:need]
        return flat + (["NA"] * (need - len(flat)))

    def log(self, *, t_iso_utc, t_unix_s, monotonic_ns, loop_dt_ms, frame_dt_ms,
            auto_mode, dn_state, profile, exposure_locked, exposure_ms, exposure_gain,
            threshold_mode, threshold_value, dark_cutoff, overall_dark,
            cell_dark, dac_dict):
        self.loop_idx += 1

        # Cells
        flat_cells = self._flatten_cells(cell_dark)

        # DACs aligned to fixed order
        dac_vals = []
        for k in self.dac_keys:
            try:
                dac_vals.append(int(dac_dict.get(k, 0)))
            except Exception:
                dac_vals.append("NA")

        row = [
            t_iso_utc,
            f"{float(t_unix_s):.6f}",
            int(monotonic_ns) if monotonic_ns is not None else "",
            (None if loop_dt_ms is None else float(loop_dt_ms)),
            (None if frame_dt_ms is None else float(frame_dt_ms)),
            self.loop_idx,
            int(bool(auto_mode)),
            int(bool(dn_state)),
            profile,
            int(bool(exposure_locked)),
            (None if exposure_ms is None else float(exposure_ms)),
            (None if exposure_gain is None else float(exposure_gain)),
            self.kp, self.ki, self.kd,
            threshold_mode,
            int(threshold_value) if threshold_value is not None else "NA",
            int(dark_cutoff) if dark_cutoff is not None else "NA",
            int(round(float(overall_dark))) if overall_dark is not None else "NA",
        ] + flat_cells + dac_vals

        self.writer.writerow(row)
        self.fh.flush()

    def close(self):
        try:
            self.fh.close()
        except Exception:
            pass


# ---------------------------
# Main
# ---------------------------
def main():
    ap = argparse.ArgumentParser(description="ICLS mainLogger with thresholds, timestamps, exposure, and PID gains")
    ap.add_argument("--exp-ms", type=float, default=8.0, help="Initial manual exposure time in ms (default: 8.0)")
    ap.add_argument("--gain", type=float, default=None, help="Initial manual gain (optional)")
    ap.add_argument("--kp", type=float, default=1.2, help="PID Kp")
    ap.add_argument("--ki", type=float, default=2.4, help="PID Ki")
    ap.add_argument("--kd", type=float, default=0.045, help="PID Kd")
    args = ap.parse_args()

    # IO and analysis
    switches = ModeSwitches(auto_pin=C.AUTO_PIN, dn_pin=C.DN_PIN, dn_debounce_ms=C.DN_DEBOUNCE_MS)
    analysis_cam = CameraManager(camera_index=C.ANALYSIS_CAMERA_INDEX)
    analyzer = LightingAnalyzer(threshold_value=C.THRESHOLD_DARK, rows=3, cols=2)

    # PID manager (gains also passed to logger)
    pid = PIDManager(
        kp=args.kp, ki=args.ki, kd=args.kd,
        sample_time=C.LOOP_SLEEP_S / 10,
        slew_per_step=3.0,
        deadband=2.0
    )
    try:
        print("PID Sample_time =", next(iter(pid.pids.values())).sample_time)
    except Exception:
        pass

    # DAC state
    dac = init_state()

    emer = EmergencyController()
    prev_auto_mode = None

    # Policy: darkness cutoff (%) and threshold mode/value
    dark_cutoff = int(clamp(C.THRESHOLD_DARK, 0, 100))  # percent 0..100
    current_profile = PROFILE_ORDER[0]
    apply_profile(current_profile, analyzer)

    # ---- Force MANUAL exposure at startup (changeable via --exp-ms / --gain) ----
    exposure_locked = True
    current_exposure_ms = float(args.exp_ms)
    current_gain = args.gain
    ok = set_exposure_manual(current_exposure_ms, current_gain)
    if ok:
        print(f"[OV9281] Manual exposure: {current_exposure_ms:.2f} ms, gain={current_gain}")
    else:
        print("[OV9281] WARNING: set_exposure_manual failed; falling back to AUTO")
        if set_exposure_auto():
            exposure_locked = False
            current_exposure_ms = None
            current_gain = None

    # HDR setup
    active_name = C.HDR_DAY_NAME
    hdr_cam_primary, hdr_cam_alt = None, None
    dn_state_initial, _ = switches.read_dn_debounced()
    desired_active = C.HDR_DAY_NAME if dn_state_initial else C.HDR_NIGHT_NAME
    active_name, hdr_cam_primary, hdr_cam_alt = hdr_ensure_active(
        desired_active, active_name, hdr_cam_primary, hdr_cam_alt, settle_s=C.SWITCH_SETTLE_S
    )

    print("Hotkeys:")
    print("  q: quit   [ / ]: T -/+5    - / = or +: darkness cutoff -/+2%    \\: toggle global/local-otsu")
    print("  e: toggle exposure AUTO/MAN   , / .: manual exposure -/+0.5 ms")
    run_logger = RunLogger(
        analyzer=analyzer,
        dac_keys=list(dac.keys()),
        pid_gains=(args.kp, args.ki, args.kd),
        pid_obj=pid
    )
    print(f"[LOG] Logging to {run_logger.path}")

    # Timing refs
    last_loop_ns = time.perf_counter_ns()
    last_frame_ns = None
    switch_in_progress = False

    try:
        while True:
            now_ns = time.perf_counter_ns()
            loop_dt_ms = (now_ns - last_loop_ns) / 1e6 if last_loop_ns is not None else None
            last_loop_ns = now_ns

            states = switches.read_states()
            auto_mode = states["auto_mode"]
            dn_state = states["dn_state"]
            dn_changed = states["dn_changed"]

            frame = analysis_cam.capture_frame()
            if frame is not None:
                overall_dark, cell_dark, img = analyzer.analyze(frame)

                frame_now = time.perf_counter_ns()
                frame_dt_ms = (frame_now - last_frame_ns) / 1e6 if last_frame_ns is not None else None
                last_frame_ns = frame_now

                # Apply lighting control
                if auto_mode:
                    dac = pid_step_and_apply(
                        cell_darkness=cell_dark,
                        state=dac,
                        manager=pid,
                        threshold_dark=dark_cutoff,
                        emergency_mode=emer.emergency
                    )
                else:
                    if prev_auto_mode in (True, None):
                        dac = reset_all(dac, pid)
                    for k in dac.keys():
                        dac[k] = 0

                # Log row
                t_iso = datetime.utcnow().isoformat() + "Z"
                t_unix = time.time()
                try:
                    run_logger.log(
                        t_iso_utc=t_iso,
                        t_unix_s=t_unix,
                        monotonic_ns=now_ns,
                        loop_dt_ms=loop_dt_ms,
                        frame_dt_ms=frame_dt_ms,
                        auto_mode=auto_mode,
                        dn_state=dn_state,
                        profile=current_profile,
                        exposure_locked=exposure_locked,
                        exposure_ms=current_exposure_ms,
                        exposure_gain=current_gain,
                        threshold_mode=getattr(analyzer, "threshold_mode", "global"),
                        threshold_value=analyzer.get_threshold(),
                        dark_cutoff=dark_cutoff,
                        overall_dark=overall_dark,
                        cell_dark=cell_dark,
                        dac_dict=dac
                    )
                except Exception as e:
                    print(f"[LOG] Warning: failed to log row: {e}")

                # Overlay (show T, mode, dark cutoff, exposure, PID gains)
                overlay_text = (
                    f"T={analyzer.get_threshold()} MODE={getattr(analyzer,'threshold_mode','global')} "
                    f"DARK={dark_cutoff}% PID=({args.kp},{args.ki},{args.kd}) "
                    f"EXP={'AUTO' if not exposure_locked else f'{current_exposure_ms:.2f}ms'}"
                )
                draw_analysis(img, dac, auto_mode)
                cv2.putText(img, overlay_text, (10, img.shape[0] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
                cv2.imshow(C.WINDOW_ANALYSIS, img)

            # DN switch handling
            if dn_changed and not switch_in_progress:
                switch_in_progress = True
                desired_active = C.HDR_DAY_NAME if dn_state else C.HDR_NIGHT_NAME
                active_name, hdr_cam_primary, hdr_cam_alt = hdr_ensure_active(
                    desired_active, active_name, hdr_cam_primary, hdr_cam_alt, settle_s=C.SWITCH_SETTLE_S
                )
                time.sleep(C.SWITCH_SETTLE_S)
                switch_in_progress = False

            # Keyboard
            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 13, 10):
                print("Exiting...")
                try:
                    dac = reset_all(dac, pid)
                except Exception:
                    pass
                break

            # --- Threshold (T) controls like other main ---
            elif key == ord('['):
                analyzer.adjust_threshold(-5)
                print(f"[Analyzer] T → {analyzer.get_threshold()} (mode={getattr(analyzer,'threshold_mode','global')})")
            elif key == ord(']'):
                analyzer.adjust_threshold(+5)
                print(f"[Analyzer] T → {analyzer.get_threshold()} (mode={getattr(analyzer,'threshold_mode','global')})")
            elif key == ord('\\'):
                cur = getattr(analyzer, "threshold_mode", "global")
                new = "local_otsu" if cur == "global" else "global"
                analyzer.set_threshold_mode(new)
                print(f"[Analyzer] threshold_mode → {new}")

            # --- Percent-dark cutoff controls like other main ---
            elif key == ord('-'):
                dark_cutoff = clamp(dark_cutoff - 2, 0, 100)
                print(f"[Policy] darkness cutoff → {dark_cutoff}%")
            elif key in (ord('='), ord('+')):
                dark_cutoff = clamp(dark_cutoff + 2, 0, 100)
                print(f"[Policy] darkness cutoff → {dark_cutoff}%")

            # --- Exposure toggles and nudge ---
            elif key == ord('e'):
                # Toggle AUTO/MANUAL exposure
                if not exposure_locked:
                    target_ms = current_exposure_ms if current_exposure_ms is not None else 8.0
                    if set_exposure_manual(target_ms, current_gain):
                        exposure_locked = True
                        current_exposure_ms = target_ms
                        print(f"[OV9281] Manual exposure: {current_exposure_ms:.2f} ms, gain={current_gain}")
                else:
                    if set_exposure_auto():
                        exposure_locked = False
                        print("[OV9281] Auto exposure ON")
            elif key == ord(','):
                if exposure_locked:
                    new_ms = max(0.1, (current_exposure_ms or 8.0) - 0.5)
                    if set_exposure_manual(new_ms, current_gain):
                        current_exposure_ms = new_ms
                        print(f"[Exposure] → {new_ms:.2f} ms")
            elif key == ord('.'):
                if exposure_locked:
                    new_ms = (current_exposure_ms or 8.0) + 0.5
                    if set_exposure_manual(new_ms, current_gain):
                        current_exposure_ms = new_ms
                        print(f"[Exposure] → {new_ms:.2f} ms")

            prev_auto_mode = auto_mode
            time.sleep(C.LOOP_SLEEP_S)

    finally:
        try:
            run_logger.close()
            print(f"[LOG] Closed {run_logger.path}")
        except Exception:
            pass
        try:
            analysis_cam.release()
        except Exception:
            pass
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        switches.cleanup()


if __name__ == "__main__":
    main()
