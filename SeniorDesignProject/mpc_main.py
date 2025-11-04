
#!/usr/bin/env python3
# mpc_main.py
# Glue that wires the MPC into your Pi loop using your existing camera analysis and overlay,
# and drives the real DACs over SPI. Drop alongside your current code and run on the Pi.
#
# Windows:
#   • "Analysis (C270)" – your annotated analysis view with per-sector darkness and DAC overlays
#
# Controls:
#   • GPIO 22 (AUTO)     – HIGH => AUTO; LOW => MANUAL (holds last command)
#   • GPIO 23 (DAY/NIGHT) – Debounced level forwarded to your HDR path if you hook it
#
# Requirements:
#   pip install cvxpy osqp spidev RPi.GPIO opencv-python numpy
#
from __future__ import annotations
import os, time, csv
import numpy as np
import cv2

# Local modules (shipped with this file set)
import mpc_config as CFG
from mpc_controller import MPCController
import spi_dac as HW

# Your existing helpers
from LightingAnalysis import CameraManager, LightingAnalyzer
from ModeSwitches import ModeSwitches
from OverlayRenderer import draw_analysis

# -------------- Helpers --------------
def _load_model(paths):
    last_err = None
    for p in paths:
        try:
            data = np.load(p)
            A = data["A"]; B = data["B"]
            return A, B, p
        except Exception as e:
            last_err = e
    raise FileNotFoundError(f"Couldn't load model from {paths}. Last error: {last_err}")

def _stack_ref(target: np.ndarray, N: int) -> np.ndarray:
    # (ny,) -> (ny,N) constant trajectory
    return np.tile(target.reshape(-1,1), (1, N))

def _open_csv(path, header):
    need_header = not os.path.exists(path)
    f = open(path, "a", newline="")
    w = csv.writer(f)
    if need_header:
        w.writerow(header)
    return f, w

# -------------- Main --------------
def main():
    # --- Load model ---
    A, B, model_path = _load_model(CFG.MODEL_PATHS)
    ny, nu = B.shape
    if ny != 6 or nu != 10:
        raise ValueError(f"Model must be 6x10 (ny,nu) = (6,10), got {B.shape}")
    print(f"[MPC] Loaded model from {model_path} (A{A.shape}, B{B.shape})")

    # --- Build controller ---
    mpc = MPCController(
        A, B,
        N=CFG.N_HORIZON,
        Q_diag=CFG.Q_TRACK,
        R_diag=CFG.R_USE,
        u_min=CFG.U_MIN,
        u_max=CFG.U_MAX,
        du_max=CFG.DU_MAX,
        lam=CFG.LAMBDA_LINEAR,
        p_coeffs=CFG.P_COEFFS,
        P_max=CFG.P_MAX,
        slack_penalty=1e6,
    )
    last_u = np.zeros(nu, dtype=float)

    # --- Camera + analysis ---
    cam = CameraManager(camera_index=0)  # uses V4L2 UVC (OV9281) in your project
    analyzer = LightingAnalyzer(threshold_value=int(CFG.TARGET_PER_SECTOR.mean()), rows=3, cols=2)

    # --- GPIO mode switches ---
    switches = ModeSwitches()

    # --- Logging (heartbeat) ---
    hb_file = None; hb_writer = None
    if CFG.HEARTBEAT_CSV:
        hb_file, hb_writer = _open_csv(CFG.HEARTBEAT_CSV,
            ["t_s","status","solve_ms","cost"] +
            [f"y_{s}" for s in ["S11","S12","S21","S22","S31","S32"]] +
            HW.NAMES
        )

    # --- Windows ---
    if CFG.SHOW_WINDOWS:
        cv2.namedWindow("Analysis (C270)", cv2.WINDOW_NORMAL)

    print("[MPC] Starting control loop. Press 'q' to quit.")
    try:
        while True:
            ret, frame = cam.read()
            if not ret:
                time.sleep(0.01)
                continue

            # Per-sector darkness and annotated image
            overall_dark, cell_dark, annotated = analyzer.analyze(frame)
            y = np.array(cell_dark, dtype=float).reshape(6)

            # Mode switches
            states = switches.read_states()
            auto_mode = bool(states["auto_mode"])  # HIGH => AUTO

            # Prepare MPC parameters
            r_traj = _stack_ref(CFG.TARGET_PER_SECTOR, CFG.N_HORIZON)

            # Solve / hold depending on AUTO/MANUAL
            if auto_mode:
                u_cmd, status, solve_ms, cost = mpc.compute(y0=y, r_traj=r_traj, u_prev=last_u)
            else:
                u_cmd, status, solve_ms, cost = last_u.copy(), "manual_hold", 0.0, None

            # Apply to hardware
            HW.apply_vec(u_cmd)
            last_u = u_cmd

            # Overlay: DACs and mode
            dac_dict = HW.vec_to_dict(u_cmd)
            annotated = draw_analysis(annotated, dac_dict, auto_mode)

            if CFG.SHOW_WINDOWS:
                cv2.imshow("Analysis (C270)", annotated)
                k = cv2.waitKey(1) & 0xFF
                if k in (ord('q'), ord('Q')):
                    break

            # Heartbeat log
            if hb_writer:
                t_s = f"{time.time():.3f}"
                row = [t_s, status, f"{solve_ms:.2f}", "" if cost is None else f"{cost:.3f}"]                       + [f"{v:.2f}" for v in y.tolist()]                       + [str(int(dac_dict[n])) for n in HW.NAMES]
                hb_writer.writerow(row)
                hb_file.flush()

            time.sleep(CFG.SLEEP_S)

    finally:
        try:
            HW.zero_all()
        except Exception:
            pass
        try:
            cam.release()
        except Exception:
            pass
        try:
            if CFG.SHOW_WINDOWS:
                cv2.destroyAllWindows()
        except Exception:
            pass
        try:
            switches.cleanup()
        except Exception:
            pass
        if hb_file:
            hb_file.close()

if __name__ == "__main__":
    main()
