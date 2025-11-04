# main.py
import os
import sys
import cv2
import time
import numpy as np

# Ensure local modules are importable when launched via a script
HERE = os.path.dirname(__file__)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "modules"))

from modules import Config as C
from modules.LightingAnalysis import CameraManager, LightingAnalyzer
from modules.DualPicam import PicamFeed
from modules.UPS import get_ups_data
from modules.ModeSwitches import ModeSwitches

# --- PID POLICY IMPORTS ---
from modules.PIDPolicy import init_state, reset_all, pid_step_and_apply, PIDManager
# --- END PID POLICY IMPORTS ---

from modules.OverlayRenderer import draw_analysis
from modules.HDRView import ensure_active as hdr_ensure_active, capture_frame as hdr_capture_frame, show as hdr_show
from modules.EmergencyController import EmergencyController
from modules.ExposureControl import apply_profile, cycle_profile, set_exposure_auto, set_exposure_manual, PROFILES, PROFILE_ORDER

# --- MPC IMPORTS (as a package) ---
# mpc_config.py and mpc_controller_fast.py live in modules/
# spi_dac.py lives alongside this file (SeniorDesignProject/)
try:
    from modules import mpc_config as CFG
    from modules import mpc_controller_fast as mpc_fast
    import spi_dac as HW
except ImportError as e:
    print(f"FATAL: Could not import MPC modules. {e}")
    print("Make sure 'modules/mpc_config.py', 'modules/mpc_controller_fast.py', and 'spi_dac.py' are accessible.")
    sys.exit(1)
# --- END MPC IMPORTS ---


def clamp(val, lo, hi):
    return max(lo, min(hi, val))


def main():
    switches = ModeSwitches(auto_pin=C.AUTO_PIN, dn_pin=C.DN_PIN, dn_debounce_ms=C.DN_DEBOUNCE_MS)
    analysis_cam = CameraManager(camera_index=C.ANALYSIS_CAMERA_INDEX)

    # LAZY INIT: don't construct either HDR cam yet
    active_name = C.HDR_DAY_NAME
    hdr_cam_primary = None
    hdr_cam_alt = None

    analyzer = LightingAnalyzer(threshold_value=C.THRESHOLD_DARK, rows=3, cols=2)

    # --- PID POLICY ---
    dac = init_state()
    pid = PIDManager(
        kp=1.3, ki=2.4, kd=0.05,
        sample_time=C.LOOP_SLEEP_S/10,
        slew_per_step=3.0,
        deadband=2.0
    )
    print("PID Sample_time =", next(iter(pid.pids.values())).sample_time)

    # --- MPC POLICY INIT ---
    try:
        # Prefer configured path; fall back to ./build/mpc_model.npz
        model_path = CFG.MODEL_PATHS[0] if getattr(CFG, "MODEL_PATHS", None) else os.path.join(HERE, "build", "mpc_model.npz")
        if not os.path.isabs(model_path):
            model_path = os.path.join(HERE, model_path)
        if not os.path.exists(model_path):
            raise FileNotFoundError(model_path)

        # Load once for logging shapes
        model_npz = np.load(model_path)
        A, B = model_npz["A"], model_npz["B"]
        print(f"Loaded MPC model from {model_path}: A={A.shape}, B={B.shape}")
    except Exception as e:
        print(f"FATAL: Could not load MPC model. Tried '{model_path}'. {e}")
        return

    mpc_cfg = mpc_fast.MPCConfig(
        horizon=CFG.N_HORIZON,
        du_max=CFG.DU_MAX,
        u_min=CFG.U_MIN,
        u_max=CFG.U_MAX,
        Q_diag=CFG.Q_TRACK,
        R_diag=CFG.R_USE,
        lam=CFG.LAMBDA_LINEAR,
        P_max=CFG.P_MAX,
        p_coeffs=CFG.P_COEFFS
    )

    # IMPORTANT: FastMPC expects model_path, NOT (A, B)
    mpc = mpc_fast.FastMPC(model_path, mpc_cfg)

    # Start MPC's previous command vector from current DAC dict (safe: no dict_to_vec in driver)
    mpc_last_u = np.array([dac.get(n, 0) for n in HW.NAMES], dtype=float)

    # --- Policy switching state (default PID; press 'm' to toggle) ---
    control_policy = "PID"

    emer = EmergencyController()
    prev_auto_mode = None  # track for just_switched_to_auto

    # Runtime-adjustable darkness cutoff (percent)
    dark_cutoff = int(clamp(C.THRESHOLD_DARK, 0, 100))  # 0..100 %

    # Start in Classroom profile
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

    print("Starting multi-light control.")
    print("Hotkeys: q=quit  m=mode (PID/MPC)  [ ]=brightness  -=,+=darkness  \\=Otsu  E=exposure  P=profile")

    try:
        while True:
            states = switches.read_states()
            auto_mode  = states["auto_mode"]
            dn_state   = states["dn_state"]
            dn_changed = states["dn_changed"]

            frame = analysis_cam.capture_frame()
            if frame is not None:
                overall_dark, cell_dark, img = analyzer.analyze(frame)

                if auto_mode:
                    # --- Policy Switcher ---
                    if control_policy == "PID":
                        # Run PID policy
                        dac = pid_step_and_apply(
                            cell_darkness=cell_dark,
                            state=dac,
                            manager=pid,
                            threshold_dark=dark_cutoff,
                            emergency_mode=emer.emergency,
                        )
                        # Keep MPC state in sync with current outputs
                        mpc_last_u = np.array([dac.get(n, 0) for n in HW.NAMES], dtype=float)

                    else:  # MPC
                        y_k = np.array(cell_dark, dtype=float)  # 6-vector

                        # Choose target: use your shared per-sector darkness target.
                        # If you want the cutoff to be the setpoint, uncomment below.
                        # r_target = np.full(mpc.ny, dark_cutoff, dtype=float)
                        r_target = np.zeros(mpc.ny, dtype=float)  # 0% darkness (fully lit)

                        # Stack across horizon: shape (ny, N)
                        r_traj = np.tile(r_target, (mpc.N, 1)).T

                        # Solve MPC (FastMPC.compute)
                        u_cmd_vec, mpc_hb = mpc.compute(
                            y0=y_k,
                            r_traj=r_traj,
                            u_prev=mpc_last_u
                        )

                        # Apply to hardware
                        HW.apply_vec(u_cmd_vec)

                        # Update loop state
                        mpc_last_u = u_cmd_vec
                        dac = HW.vec_to_dict(u_cmd_vec)
                    # --- End Policy Switcher ---

                else:
                    # Manual Mode: if we just left AUTO, reset everything
                    if prev_auto_mode in (True, None):
                        dac = reset_all(dac, pid)
                        try:
                            mpc_last_u = np.zeros(mpc.nu, dtype=float)
                        except Exception:
                            mpc_last_u = np.zeros(len(HW.NAMES), dtype=float)
                    # Hold zeros in manual
                    for k in list(dac.keys()):
                        dac[k] = 0

                draw_analysis(img, dac, auto_mode)

                # HUD: show analyzer threshold (T), mode, active profile, darkness cutoff, policy
                t_val = analyzer.get_threshold()
                mode_str = getattr(analyzer, "threshold_mode", "global")
                cv2.putText(
                    img,
                    f"T={t_val}  MODE={mode_str}  DARK={dark_cutoff}%  PROFILE={current_profile}  POLICY={control_policy}",
                    (10, img.shape[0] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (255, 255, 255), 2
                )
                cv2.imshow(C.WINDOW_ANALYSIS, img)

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

                # Emergency controller needs just_switched_to_auto
                just_switched_to_auto = (prev_auto_mode is False and auto_mode)

                emergency_mode, event = emer.eval_transition(
                    current_status,
                    auto_mode=auto_mode,
                    just_switched_to_auto=just_switched_to_auto
                )

                if event == "enter_returned_to_auto":
                    print("Returned to AUTO while UPS is already On Battery. ENTERING EMERGENCY MODE.")
                    dac = reset_all(dac, pid)
                elif event == "enter_online_to_onbatt":
                    print("UPS transitioned Online -> On Battery. ENTERING EMERGENCY MODE.")
                    dac = reset_all(dac, pid)
                elif event == "exit_online":
                    print("UPS back Online. EXITING EMERGENCY MODE.")

                hdr_show(
                    hdr_frame,
                    active_name=active_name,
                    auto_mode=auto_mode,
                    ups_data=ups,
                    emergency_mode=emergency_mode
                )

            # remember for next loop
            prev_auto_mode = auto_mode

            # --- Key handling ---
            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 13, 10):
                print("Exiting...")
                try:
                    dac = reset_all(dac, pid)
                except Exception:
                    pass
                break

            # Toggle PID/MPC
            elif key == ord('m'):
                if control_policy == "PID":
                    control_policy = "MPC"
                    # Sync MPC state to current PID outputs
                    mpc_last_u = np.array([dac.get(n, 0) for n in HW.NAMES], dtype=float)
                else:
                    control_policy = "PID"
                    pid.reset_all()  # Reset PID integrators
                print(f"Switched to {control_policy} control")

            # Analyzer threshold (brightness) — global mode only
            elif key == ord('['):
                analyzer.adjust_threshold(-5)
            elif key == ord(']'):
                analyzer.adjust_threshold(+5)

            # Darkness cutoff adjustment (percent)
            elif key == ord('-'):
                dark_cutoff = clamp(dark_cutoff - 2, 0, 100)
                print(f"[Policy] darkness cutoff → {dark_cutoff}%")
            elif key in (ord('='), ord('+')):
                dark_cutoff = clamp(dark_cutoff + 2, 0, 100)
                print(f"[Policy] darkness cutoff → {dark_cutoff}%")

            # Toggle global fixed threshold <-> local Otsu per cell
            elif key == ord('\\'):
                cur = getattr(analyzer, "threshold_mode", "global")
                new = "local_otsu" if cur == "global" else "global"
                analyzer.set_threshold_mode(new)

            # Exposure + Profiles
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
        try:
            dac = reset_all(dac, pid)
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
