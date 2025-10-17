# main.py
import cv2
import time

from modules.LightingAnalysis import CameraManager, LightingAnalyzer
from modules.DualPicam import PicamFeed
from modules.UPS import get_ups_data
from modules.ModeSwitches import ModeSwitches
from modules.LightPolicy import init_state, reset_all, step_and_apply  # <-- NEW

def _has_token(status_str: str, token: str) -> bool:
    if not status_str:
        return False
    toks = status_str.replace(",", " ").lower().split()
    return token.lower() in toks

def _is_online(status_str: str) -> bool:
    return _has_token(status_str, "ol")   # matches "OL", "OL CHRG"

def _is_on_battery(status_str: str) -> bool:
    return _has_token(status_str, "ob")   # matches "OB", "OB DISCHRG"

def _switch_hdr_camera(desired_name, active_name, hdr_cam_primary, hdr_cam_alt):
    """
    Ensure HDR feed matches desired_name: "IMX708" (Day) or "IMX327" (Night).
    Starts/stops the appropriate PicamFeed. Returns (new_active_name, hdr_cam_primary, hdr_cam_alt).
    """
    if desired_name == active_name:
        return active_name, hdr_cam_primary, hdr_cam_alt

    if desired_name == "IMX708":
        # Switch to IMX708 on camera_num=0
        if hdr_cam_alt is not None:
            hdr_cam_alt.stop()
        hdr_cam_primary.start()
        print("[Cam] Switched to IMX708 (Day).")
        return "IMX708", hdr_cam_primary, hdr_cam_alt
    else:
        # Switch to IMX327 on camera_num=1
        hdr_cam_primary.stop()
        if hdr_cam_alt is None:
            hdr_cam_alt = PicamFeed(camera_num=1, af=False)
        else:
            hdr_cam_alt.start()
        print("[Cam] Switched to IMX327 (Night).")
        return "IMX327", hdr_cam_primary, hdr_cam_alt

def main():
    # -------- Mode switches (debounced) --------
    switches = ModeSwitches(auto_pin=22, dn_pin=23, dn_debounce_ms=75)

    # -------- Cameras --------
    analysis_cam = CameraManager(camera_index=0)

    active_name = "IMX708"
    hdr_cam_primary = PicamFeed(camera_num=0, af=True)  # IMX708
    hdr_cam_alt = None                                  # Lazy-init IMX327 when needed

    analyzer = LightingAnalyzer(threshold_value=10, rows=3, cols=2)

    # -------- DAC state (moved to LightPolicy) --------
    dac = init_state()  # keys: HS11, HS21, LS1, HF1, LF1, HS12, HS22, LS2, HF2, LF2

    STEP = 3
    THRESHOLD_DARK = 10

    # --- Emergency mode state ---
    emergency_mode = False
    prev_ups_status = None

    # --- Track AUTO/MANUAL transitions to avoid redundant SPI spam ---
    prev_auto_mode = None  # Unknown at start

    # --- Initialize Day/Night based on debounced module state at startup ---
    dn_state_initial, _ = switches.read_dn_debounced()
    last_dn_high = dn_state_initial
    desired_active = "IMX708" if last_dn_high else "IMX327"
    active_name, hdr_cam_primary, hdr_cam_alt = _switch_hdr_camera(
        desired_active, active_name, hdr_cam_primary, hdr_cam_alt
    )
    switch_in_progress = False  # guard to avoid capturing while switching

    print("Starting multi-light control (two columns + MANUAL override + EMERGENCY MODE + GPIO23 Day/Night).")
    print("Press 'q' or Enter to exit.")

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
                    step=STEP,
                    threshold_dark=THRESHOLD_DARK,
                    emergency_mode=emergency_mode
                )
            else:
                # MANUAL: on entry, zero hardware + state once
                if prev_auto_mode is True or prev_auto_mode is None:
                    dac = reset_all(dac)
                # Ensure values are zero for overlays (hardware already zeroed above)
                for k in dac.keys():
                    dac[k] = 0

            # ---------- Overlay (Column 1 on left, Column 2 on right) ----------
            H, W = analysis_img.shape[:2]
            font = cv2.FONT_HERSHEY_SIMPLEX
            scale = 0.7
            thick = 2
            left_x  = 10
            top_y   = 30
            line_h  = 30
            color   = (0, 255, 0)

            def put_right(text, y):
                (tw, th), _ = cv2.getTextSize(text, font, scale, thick)
                x = W - 10 - tw
                cv2.putText(analysis_img, text, (x, y), font, scale, color, thick)

            mode_text = "MODE: AUTO" if auto_mode else "MODE: MANUAL"
            (tw, th), _ = cv2.getTextSize(mode_text, font, 0.9, 2)
            cv2.putText(analysis_img, mode_text, ((W - tw)//2, 25), font, 0.9, (255,255,255), 2)

            # Column 1 (left)
            cv2.putText(analysis_img, f"HS11: {dac['HS11']}", (left_x, top_y + 0*line_h), font, scale, color, thick)
            cv2.putText(analysis_img, f"HS21: {dac['HS21']}", (left_x, top_y + 1*line_h), font, scale, color, thick)
            cv2.putText(analysis_img, f"HF1:  {dac['HF1']}",  (left_x, top_y + 2*line_h), font, scale, color, thick)
            cv2.putText(analysis_img, f"LS1:  {dac['LS1']}",  (left_x, top_y + 3*line_h), font, scale, color, thick)
            cv2.putText(analysis_img, f"LF1:  {dac['LF1']}",  (left_x, top_y + 4*line_h), font, scale, color, thick)

            # Column 2 (right)
            put_right(f"HS12: {dac['HS12']}", top_y + 0*line_h)
            put_right(f"HS22: {dac['HS22']}", top_y + 1*line_h)
            put_right(f"HF2:  {dac['HF2']}",  top_y + 2*line_h)
            put_right(f"LS2:  {dac['LS2']}",  top_y + 3*line_h)
            put_right(f"LF2:  {dac['LF2']}",  top_y + 4*line_h)

            cv2.imshow("Analysis (C270)", analysis_img)

        # -------- Debounced Day/Night hardware switch --------
        if not switch_in_progress:
            changed = dn_changed
            new_high = dn_state
            if changed:
                switch_in_progress = True
                desired_active = "IMX708" if new_high else "IMX327"
                active_name, hdr_cam_primary, hdr_cam_alt = _switch_hdr_camera(
                    desired_active, active_name, hdr_cam_primary, hdr_cam_alt
                )
                last_dn_high = new_high
                time.sleep(0.05)  # short settle to avoid immediate capture on a starting/stopping cam
                switch_in_progress = False

        # -------- HDR feed (source based on active_name) --------
        hdr_src = hdr_cam_primary if active_name == "IMX708" else (
            hdr_cam_alt if hdr_cam_alt is not None else PicamFeed(camera_num=1, af=False)
        )
        if active_name != "IMX708" and hdr_cam_alt is None:
            hdr_cam_alt = hdr_src  # record creation

        hdr_frame = None
        try:
            hdr_frame = hdr_src.capture_frame()
        except Exception as e:
            # If capture failed during a race with a switch, just skip this frame
            print(f"[Cam] capture_frame exception: {e}")

        if hdr_frame is not None:
            ups_data = get_ups_data()
            current_status  = ups_data.get("ups.status", "Unknown")
            current_charge  = ups_data.get("battery.charge", "Unknown")
            current_runtime = ups_data.get("battery.runtime", "0")

            now_online  = _is_online(current_status)
            now_on_batt = _is_on_battery(current_status)
            was_online  = _is_online(prev_ups_status) if prev_ups_status is not None else False

            # If we just switched into AUTO and the UPS is already on battery, enter EMERGENCY
            if prev_auto_mode is False and auto_mode:
                if now_on_batt and not emergency_mode:
                    print("Returned to AUTO while UPS is already On Battery. ENTERING EMERGENCY MODE.")
                    emergency_mode = True
                    # Zero all except LF2; state will be driven by step_and_apply on next loop
                    dac = reset_all(dac)

            # Normal edge-based EMERGENCY transitions (AUTO only)
            if auto_mode:
                if (not emergency_mode) and was_online and now_on_batt:
                    print("UPS transitioned Online -> On Battery. ENTERING EMERGENCY MODE.")
                    emergency_mode = True
                    dac = reset_all(dac)
                if emergency_mode and now_online:
                    print("UPS back Online. EXITING EMERGENCY MODE.")
                    emergency_mode = False

            prev_ups_status = current_status

            # Format runtime (sec->HH:MM:SS)
            runtime_sec = int(current_runtime) if str(current_runtime).isdigit() else 0
            h, m, s = runtime_sec // 3600, (runtime_sec % 3600) // 60, runtime_sec % 60
            formatted_runtime = f"{h:02}:{m:02}:{s:02}"

            # Overlay UPS + active camera label + mode
            cv2.putText(hdr_frame, f"Active: {active_name}",         (10, 30),  cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,255), 2)
            cv2.putText(hdr_frame, f"UPS Status: {current_status}",  (10, 60),  cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
            cv2.putText(hdr_frame, f"Charge: {current_charge}%",     (10, 90),  cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
            cv2.putText(hdr_frame, f"Runtime: {formatted_runtime}",  (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)

            mode_text_hdr = "MODE: AUTO" if auto_mode else "MODE: MANUAL"
            cv2.putText(hdr_frame, mode_text_hdr, (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (200,255,200), 2)

            if auto_mode and emergency_mode:
                cv2.putText(hdr_frame, "EMERGENCY MODE", (10, 185),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,0,255), 3)
            elif (not auto_mode) and now_on_batt:
                cv2.putText(hdr_frame, "EMERGENCY OVERRIDDEN BY MANUAL", (10, 185),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0,165,255), 3)

            cv2.imshow("HDR Feed (CSI)", hdr_frame)

        # Remember the mode for edge-detect behavior
        prev_auto_mode = auto_mode

        # Exit handling
        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), 13, 10):  # q or Enter
            print("Exiting…")
            break

        time.sleep(0.05)

    # -------- Cleanup --------
    try:
        analysis_cam.release()
        if hdr_cam_primary: hdr_cam_primary.release()
        if hdr_cam_alt:     hdr_cam_alt.release()
        cv2.destroyAllWindows()
    finally:
        switches.cleanup()

if __name__ == "__main__":
    main()
