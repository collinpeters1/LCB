# main.py
import cv2
import time

from modules import Config as C
from modules.LightingAnalysis import CameraManager, LightingAnalyzer
from modules.DualPicam import PicamFeed
from modules.UPS import get_ups_data
from modules.ModeSwitches import ModeSwitches
# ⬇️ replaced LightPolicy with PIDPolicy
from modules.PIDPolicy import (
    PIDManager,
    init_state as pid_init_state,
    reset_all as pid_reset_all,
    pid_step_and_apply,
)
from modules.OverlayRenderer import draw_analysis
from modules.HDRView import ensure_active as hdr_ensure_active, capture_frame as hdr_capture_frame, show as hdr_show
from modules.EmergencyController import EmergencyController
from modules.ExposureControl import apply_profile, cycle_profile, set_exposure_auto, set_exposure_manual, PROFILES, PROFILE_ORDER


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

    # ⬇️ PID: state + manager
    dac = pid_init_state()
    pid_mgr = PIDManager(kp=7.5, ki=1.2, kd=0.8, out_slew_per_step=12.0, deadband=0.5)

    emer = EmergencyController()
    prev_auto_mode = None  # track for just_switched_to_auto

    # --- runtime-adjustable darkness cutoff (starts from config) ---
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

    print("Starting multi-light control (PID).")
    print("Hotkeys: q=quit  [ ]=brightness threshold  -=/+= darkness cutoff  \\=global/local-Otsu  E=exposure  P=profile")

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
                    # ⬇️ PID step instead of step_and_apply from LightPolicy
                    dac = pid_step_and_apply(
                        cell_darkness=cell_dark,
                        state=dac,
                        manager=pid_mgr,
                        threshold_dark=dark_cutoff,
                        emergency_mode=emer.emergency
                    )
                else:
                    # On entry to MANUAL, zero hardware and reset integrators
                    if prev_auto_mode in (True, None):
                        dac = pid_reset_all(dac, pid_mgr)
                    # Keep manual behavior: lights stay 0 unless manual knobs are added
                    for k in dac.keys():
                        dac[k] = 0

                draw_analysis(img, dac, auto_mode)

                # HUD: show analyzer threshold (T), mode, active profile, and darkness cutoff (DARK)
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
                    dac = pid_reset_all(dac, pid_mgr)
                elif event == "enter_online_to_onbatt":
                    print("UPS transitioned Online -> On Battery. ENTERING EMERGENCY MODE.")
                    dac = pid_reset_all(dac, pid_mgr)
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
                    dac = pid_reset_all(dac, pid_mgr)
                except Exception:
                    pass
                break

            # Analyzer threshold (brightness) — global mode only
            elif key == ord('['):
                analyzer.adjust_threshold(-5)
            elif key == ord(']'):
                analyzer.adjust_threshold(+5)

            # Darkness cutoff adjustment (percentage of dark pixels to target)
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
            dac = pid_reset_all(dac, pid_mgr)
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
            cv2.destroyAllWindows()
        except Exception:
            pass
        switches.cleanup()


if __name__ == "__main__":
    main()
