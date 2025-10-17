# main.py
import cv2
import time

from modules import Config as C
from modules.LightingAnalysis import CameraManager, LightingAnalyzer
from modules.DualPicam import PicamFeed
from modules.UPS import get_ups_data
from modules.ModeSwitches import ModeSwitches
from modules.LightPolicy import init_state, reset_all, step_and_apply
from modules.OverlayRenderer import draw_analysis
from modules.HDRView import ensure_active as hdr_ensure_active, capture_frame as hdr_capture_frame, show as hdr_show
from modules.EmergencyController import EmergencyController

def main():
    # -------- Mode switches (debounced) --------
    switches = ModeSwitches(auto_pin=C.AUTO_PIN, dn_pin=C.DN_PIN, dn_debounce_ms=C.DN_DEBOUNCE_MS)

    # -------- Cameras --------
    analysis_cam = CameraManager(camera_index=C.ANALYSIS_CAMERA_INDEX)

    active_name = C.HDR_DAY_NAME
    hdr_cam_primary = PicamFeed(camera_num=0, af=True)  # IMX708
    hdr_cam_alt = None                                  # Lazy-init IMX327 when needed

    analyzer = LightingAnalyzer(threshold_value=C.THRESHOLD_DARK, rows=3, cols=2)

    # -------- DAC state (LightPolicy) --------
    dac = init_state()  # keys: HS11, HS21, LS1, HF1, LF1, HS12, HS22, LS2, HF2, LF2

    emer = EmergencyController()
    prev_auto_mode = None  # Unknown at start

    # --- Initialize Day/Night based on debounced module state at startup ---
    dn_state_initial, _ = switches.read_dn_debounced()
    last_dn_high = dn_state_initial
    desired_active = C.HDR_DAY_NAME if last_dn_high else C.HDR_NIGHT_NAME
    active_name, hdr_cam_primary, hdr_cam_alt = hdr_ensure_active(
        desired_active, active_name, hdr_cam_primary, hdr_cam_alt
    )
    switch_in_progress = False  # guard to avoid capturing while switching

    print("Starting multi-light control (two columns + MANUAL override + EMERGENCY MODE + GPIO23 Day/Night).")
    print("Press 'q' or Enter to exit.")

    try:
        while True:
            # -------- Determine mode & DN from module (debounced) --------
            states     = switches.read_states()
            auto_mode  = states["auto_mode"]
            dn_state   = states["dn_state"]
            dn_changed = states["dn_changed"]

            # -------- Lighting analysis (C270) --------
            analysis_frame = analysis_cam.capture_frame()
            if analysis_frame is not None:
                overall_dark, cell_darkness, analysis_img = analyzer.analyze(analysis_frame)

                if auto_mode:
                    # AUTO: normal ramping OR emergency override
                    dac = step_and_apply(
                        cell_darkness,
                        dac,
                        step=C.STEP,
                        threshold_dark=C.THRESHOLD_DARK,
                        emergency_mode=emer.emergency
                    )
                else:
                    # MANUAL: on entry, zero hardware + state once
                    if prev_auto_mode is True or prev_auto_mode is None:
                        dac = reset_all(dac)
                    # Ensure values are zero for overlays (hardware already zeroed above)
                    for k in dac.keys():
                        dac[k] = 0

                # ---------- Overlay (module) ----------
                draw_analysis(analysis_img, dac, auto_mode)
                cv2.imshow(C.WINDOW_ANALYSIS, analysis_img)

            # -------- Debounced Day/Night hardware switch --------
            if not switch_in_progress and dn_changed:
                switch_in_progress = True
                desired_active = C.HDR_DAY_NAME if dn_state else C.HDR_NIGHT_NAME
                active_name, hdr_cam_primary, hdr_cam_alt = hdr_ensure_active(
                    desired_active, active_name, hdr_cam_primary, hdr_cam_alt
                )
                last_dn_high = dn_state
                time.sleep(C.SWITCH_SETTLE_S)  # short settle to avoid immediate capture on a starting/stopping cam
                switch_in_progress = False

            # -------- HDR feed (source based on active_name) --------
            hdr_src = hdr_cam_primary if active_name == C.HDR_DAY_NAME else hdr_cam_alt
            hdr_frame = hdr_capture_frame(hdr_src)

            if hdr_frame is not None:
                ups_data = get_ups_data()
                current_status  = ups_data.get("ups.status", "Unknown")

                just_switched_to_auto = (prev_auto_mode is False and auto_mode)
                emergency_mode, event = emer.eval_transition(
                    current_status,
                    auto_mode=auto_mode,
                    just_switched_to_auto=just_switched_to_auto
                )

                if event == "enter_returned_to_auto":
                    print("Returned to AUTO while UPS is already On Battery. ENTERING EMERGENCY MODE.")
                    dac = reset_all(dac)
                elif event == "enter_online_to_onbatt":
                    print("UPS transitioned Online -> On Battery. ENTERING EMERGENCY MODE.")
                    dac = reset_all(dac)
                elif event == "exit_online":
                    print("UPS back Online. EXITING EMERGENCY MODE.")

                hdr_show(hdr_frame, active_name=active_name, auto_mode=auto_mode,
                         ups_data=ups_data, emergency_mode=emergency_mode)

            prev_auto_mode = auto_mode

            # Exit handling
            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 13, 10):  # q or Enter
                print("Exiting… turning all lights OFF.")
                try:
                    dac = reset_all(dac)  # ensure hardware is zeroed on exit
                except Exception:
                    pass
                break

            time.sleep(C.LOOP_SLEEP_S)

    finally:
        # Extra safety: ensure lights are off even if we crashed out above
        try:
            dac = reset_all(dac)
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
