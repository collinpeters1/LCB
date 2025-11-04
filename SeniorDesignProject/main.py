# main.py
import cv2
import time
import numpy as np

from modules import Config as C
from modules.LightingAnalysis import CameraManager, LightingAnalyzer
from modules.DualPicam import PicamFeed
from modules.UPS import get_ups_data
from modules.ModeSwitches import ModeSwitches
#from modules.LightPolicy import init_state, reset_all, step_and_apply

# --- MODIFIED: We only need the functions and manager from PIDPolicy ---
# init_state and reset_all are assumed to talk to hardware
from modules.PIDPolicy import init_state, reset_all, pid_step_and_apply, PIDManager
# --- END MODIFIED ---

from modules.OverlayRenderer import draw_analysis
from modules.HDRView import ensure_active as hdr_ensure_active, capture_frame as hdr_capture_frame, show as hdr_show
from modules.EmergencyController import EmergencyController
from modules.ExposureControl import apply_profile, cycle_profile, set_exposure_auto, set_exposure_manual, PROFILES, PROFILE_ORDER

# --- NEW MPC IMPORTS ---
# Make sure mpc_config.py, mpc_controller_fast.py, and spi_dac.py
# are in the same directory or in your Python path
try:
    import mpc_config as CFG
    import mpc_controller_fast as mpc_fast
    import spi_dac as HW
except ImportError as e:
    print(f"FATAL: Could not import MPC modules. {e}")
    print("Make sure mpc_config.py, mpc_controller_fast.py, and spi_dac.py are accessible.")
    exit()
# --- END NEW MPC IMPORTS ---


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
    # This init_state() is from your PIDPolicy module.
    # It is assumed to init hardware and return the state dict.
    dac = init_state()
    pid = PIDManager(
        kp=1.3, ki = 2.4, kd = 0.05,
        sample_time = C.LOOP_SLEEP_S/10,
        slew_per_step=3.0,
        deadband=2.0
    )
    print("PID Sample_time =", next(iter(pid.pids.values())).sample_time)
    
    # --- NEW: MPC POLICY INIT ---
    try:
        # Use the loader from mpc_main.py, but simplified to one path
        model_path = CFG.MODEL_PATHS[0] 
        model = np.load(model_path)
        A, B = model['A'], model['B']
        print(f"Loaded MPC model from {model_path}: A={A.shape}, B={B.shape}")
    except Exception as e:
        print(f"FATAL: Could not load MPC model from {CFG.MODEL_PATHS}. {e}")
        return # Can't run without a model

    # Load config from mpc_config.py
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
    mpc = mpc_fast.FastMPC(A, B, mpc_cfg)
    mpc_last_u = HW.dict_to_vec(dac) # Start MPC state from the zeroed DAC dict
    
    # --- NEW: Policy switching state ---
    control_policy = "PID"  # Start with PID as default
    # --- END NEW MPC INIT ---

    emer = EmergencyController()
    prev_auto_mode = None  # track for just_switched_to_auto

    # --- NEW: runtime-adjustable darkness cutoff (starts from config) ---
    dark_cutoff = int(clamp(C.THRESHOLD_DARK, 0, 100))  # percent 0..100

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
                    
                    # --- NEW: Policy Switcher ---
                    if control_policy == "PID":
                        # --- Run PID Policy ---
                        dac = pid_step_and_apply(
                            cell_darkness=cell_dark,
                            state=dac,
                            manager=pid,
                            threshold_dark=dark_cutoff,
                            emergency_mode=emer.emergency,
                        )
                        # Sync MPC state for next loop
                        mpc_last_u = HW.dict_to_vec(dac)
                    
                    else: # control_policy == "MPC"
                        # --- Run MPC Policy ---
                        y_k = np.array(cell_dark) # (6,) vector
                        
                        # Set reference: 0% darkness (or use dark_cutoff)
                        # Using 0% as the target (fully lit)
                        r_target = np.zeros(mpc.ny)
                        
                        # Use dark_cutoff as target instead (e.g., 10%)
                        # r_target = np.full(mpc.ny, dark_cutoff) 
                        
                        r_traj = np.tile(r_target, (mpc.N, 1)).T # Shape (6, N)
                        
                        # Solve MPC
                        u_cmd_vec, mpc_hb = mpc.update(
                            y0=y_k,
                            r_traj=r_traj,
                            u_prev=mpc_last_u
                        )
                        
                        # Apply to hardware using spi_dac
                        HW.apply_vec(u_cmd_vec)
                        
                        # Store state for next loop
                        mpc_last_u = u_cmd_vec
                        dac = HW.vec_to_dict(u_cmd_vec) # Update dict for UI
                    # --- END Policy Switcher ---

                else: # Manual Mode
                    if prev_auto_mode in (True, None):
                        # Just switched to manual, reset everything
                        dac = reset_all(dac, pid) # This zeros HW and resets PID
                        mpc_last_u = np.zeros(mpc.nu) # Reset MPC state
                    
                    # This loop is from your original code
                    for k in dac.keys():
                        dac[k] = 0

                draw_analysis(img, dac, auto_mode)

                # HUD: show analyzer threshold (T), mode, active profile, and NEW darkness cutoff (DARK)
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
                    dac = reset_all(dac,pid)
                elif event == "enter_online_to_onbatt":
                    print("UPS transitioned Online -> On Battery. ENTERING EMERGENCY MODE.")
                    dac = reset_all(dac,pid)
                elif event == "exit_online":
                    print("UPS back Online. EXITING EMERGENCY MODE.")

                # hdr_show expects keyword args
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
                    dac = reset_all(dac,pid)
                except Exception:
                    pass
                break
            
            # --- NEW: Policy Toggle ---
            elif key == ord('m'):
                if control_policy == "PID":
                    control_policy = "MPC"
                    # Sync MPC state to current PID output
                    mpc_last_u = HW.dict_to_vec(dac)
                else:
                    control_policy = "PID"
                    pid.reset_all() # Reset PID integrators
                print(f"Switched to {control_policy} control")

            # Analyzer threshold (brightness) — global mode only
            elif key == ord('['):
                analyzer.adjust_threshold(-5)
            elif key == ord(']'):
                analyzer.adjust_threshold(+5)

            # NEW: Darkness cutoff adjustment (percentage of dark pixels to trigger)
            elif key == ord('-'):
                dark_cutoff = clamp(dark_cutoff - 2, 0, 100)
                print(f"[Policy] darkness cutoff → {dark_cutoff}%")
            elif key in (ord('='), ord('+')):  # plus key reports '=' on most layouts
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
            # This is your original reset function
            dac = reset_all(dac,pid) 
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