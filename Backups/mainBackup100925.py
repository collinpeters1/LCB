# main.py — AUTO/MANUAL via GPIO22, EMERGENCY MODE, Day/Night via GPIO23 (debounced)
import cv2
import time
import spidev
import RPi.GPIO as GPIO

from modules.LightingAnalysis import CameraManager, LightingAnalyzer
from modules.DualPicam import PicamFeed
from modules.UPS import get_ups_data

# --- GPIO (Pi5) ---
AUTO_PIN = 22  # DPDT board center bus -> GPIO22 (pin 15)
DN_PIN   = 23  # Day/Night switch      -> GPIO23 (pin 16)

GPIO.setmode(GPIO.BCM)
GPIO.setup(AUTO_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)   # your file used PUD_UP for AUTO
GPIO.setup(DN_PIN,   GPIO.IN, pull_up_down=GPIO.PUD_OFF)  # external pull network on your board

# --- SPI/DAC helpers (two-column control on SPI1 CE0/CE1) ---
SPI_BUS = 1
# Column 1 -> CE1 (device=1), Column 2 -> CE0 (device=0)
COL_TO_DEVICE = {1: 1, 2: 0}

def _write_dac(value: int, channel: int, column: int):
    """
    Send an 8-bit value (0..255) to LTC1665 channel (1..8) on the given column (1 or 2).
    Column 1 is on SPI1 CE1, Column 2 is on SPI1 CE0.
    """
    if not (1 <= channel <= 8): raise ValueError("Channel must be 1..8")
    if not (0 <= value <= 255): raise ValueError("Value must be 0..255")
    if column not in (1, 2):    raise ValueError("Column must be 1 or 2")

    cmd = (channel << 12) | (value << 4)  # LTC1665: CCCC DDDDDDDD 0000
    hi = (cmd >> 8) & 0xFF
    lo = cmd & 0xFF

    dev = COL_TO_DEVICE[column]
    spi = spidev.SpiDev()
    spi.open(SPI_BUS, dev)
    spi.max_speed_hz = 2_000_000
    spi.mode = 0
    spi.writebytes([hi, lo])
    spi.close()

def _zero_all_dacs():
    """Force all mapped channels on both columns to 0."""
    for col in (1, 2):
        for ch in (1, 2, 3, 4, 5):
            _write_dac(0, ch, column=col)

def _has_token(status_str: str, token: str) -> bool:
    if not status_str:
        return False
    toks = status_str.replace(",", " ").lower().split()
    return token.lower() in toks

def _is_online(status_str: str) -> bool:
    return _has_token(status_str, "ol")   # matches "OL", "OL CHRG"

def _is_on_battery(status_str: str) -> bool:
    return _has_token(status_str, "ob")   # matches "OB", "OB DISCHRG"

# --- Day/Night helpers (with debounce) ---
def _is_day_raw() -> bool:
    return GPIO.input(DN_PIN) == GPIO.HIGH

def _debounced_dn_change(last_state: bool, samples: int = 5, interval_s: float = 0.01):
    """
    Return (changed, new_state). We first check if the *raw* state differs from last_state.
    If so, confirm it by reading 'samples' times over 'interval_s' to debounce.
    """
    raw = _is_day_raw()
    if raw == last_state:
        return (False, last_state)

    # confirm new level is stable
    for _ in range(samples - 1):
        time.sleep(interval_s)
        if _is_day_raw() != raw:
            return (False, last_state)  # bounced back; no change
    return (True, raw)

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
    # -------- Cameras --------
    analysis_cam = CameraManager(camera_index=0)

    active_name = "IMX708"
    hdr_cam_primary = PicamFeed(camera_num=0, af=True)  # IMX708
    hdr_cam_alt = None                                  # Lazy-init IMX327 when needed

    analyzer = LightingAnalyzer(threshold_value=10, rows=3, cols=2)

    # -------- Column 1 (left) DAC state --------
    dac_HS11 = 0  # High Spot Row1 Col1 -> S11 (ch1)
    dac_HS21 = 0  # High Spot Row2 Col1 -> S21 (ch2)
    dac_LS1  = 0  # Low  Spot Col1 -> S11+S21 (ch3)
    dac_HF1  = 0  # High Flood Col1 -> S31+S32 (ch4)
    dac_LF1  = 0  # Low  Flood Col1 -> S21+S31 (ch5)

    # -------- Column 2 (right) DAC state --------
    dac_HS12 = 0  # High Spot Row1 Col2 -> S12 (ch1)
    dac_HS22 = 0  # High Spot Row2 Col2 -> S22 (ch2)
    dac_LS2  = 0  # Low  Spot Col2 -> S12+S22 (ch3)
    dac_HF2  = 0  # High Flood Col2 -> S32+S31 (ch4)
    dac_LF2  = 0  # Low  Flood Col2 -> S22+S32 (ch5)

    STEP = 3
    THRESHOLD_DARK = 10

    # --- Emergency mode state ---
    emergency_mode = False
    prev_ups_status = None

    # --- Track AUTO/MANUAL transitions to avoid redundant SPI spam ---
    prev_auto_mode = None  # Unknown at start

    # --- Initialize Day/Night based on GPIO23 at startup (debounced) ---
    last_dn_high = _is_day_raw()
    desired_active = "IMX708" if last_dn_high else "IMX327"
    active_name, hdr_cam_primary, hdr_cam_alt = _switch_hdr_camera(
        desired_active, active_name, hdr_cam_primary, hdr_cam_alt
    )
    switch_in_progress = False  # guard to avoid capturing while switching

    print("Starting multi-light control (two columns + MANUAL override + EMERGENCY MODE + GPIO23 Day/Night).")
    print("Press 'q' or Enter to exit.")

    while True:
        # -------- Determine mode from GPIO22 --------
        auto_mode = GPIO.input(AUTO_PIN) == GPIO.HIGH

        # -------- Lighting analysis (C270) --------
        analysis_frame = analysis_cam.capture_frame()
        if analysis_frame is not None:
            overall_dark, cell_darkness, analysis_img = analyzer.analyze(analysis_frame)

            # Index mapping for 3x2 grid in row-major:
            # 0:S11  1:S12
            # 2:S21  3:S22
            # 4:S31  5:S32
            dark_S11 = cell_darkness[0]; dark_S12 = cell_darkness[1]
            dark_S21 = cell_darkness[2]; dark_S22 = cell_darkness[3]
            dark_S31 = cell_darkness[4]; dark_S32 = cell_darkness[5]

            # Booleans for each sector exceeding threshold
            s11_dark = dark_S11 >= THRESHOLD_DARK
            s21_dark = dark_S21 >= THRESHOLD_DARK
            s31_dark = dark_S31 >= THRESHOLD_DARK
            s12_dark = dark_S12 >= THRESHOLD_DARK
            s22_dark = dark_S22 >= THRESHOLD_DARK
            s32_dark = dark_S32 >= THRESHOLD_DARK

            # Coupled lights per your definitions:
            # Column 1
            LS1_active = s11_dark and s21_dark       # Low spot covers S11+S21
            LF1_active = s21_dark and s31_dark       # Low flood covers S21+S31
            # Column 2
            LS2_active = s12_dark and s22_dark       # Low spot covers S12+S22
            LF2_active = s22_dark and s32_dark       # Low flood covers S22+S32

            if auto_mode:
                if not emergency_mode:
                    # ----- Normal AUTO control -----
                    # Column 1
                    dac_HS11 = min(dac_HS11 + STEP, 255) if (s11_dark and not LS1_active) else max(dac_HS11 - STEP, 0)
                    dac_HS21 = min(dac_HS21 + STEP, 255) if (s21_dark and not LS1_active and not LF1_active) else max(dac_HS21 - STEP, 0)
                    dac_HF1  = min(dac_HF1  + STEP, 255) if (s31_dark and not LF1_active) else max(dac_HF1  - STEP, 0)
                    dac_LS1  = min(dac_LS1  + STEP, 255) if LS1_active else max(dac_LS1  - STEP, 0)
                    dac_LF1  = min(dac_LF1  + STEP, 255) if LF1_active else max(dac_LF1  - STEP, 0)

                    # Column 2
                    dac_HS12 = min(dac_HS12 + STEP, 255) if (s12_dark and not LS2_active) else max(dac_HS12 - STEP, 0)
                    dac_HS22 = min(dac_HS22 + STEP, 255) if (s22_dark and not LS2_active and not LF2_active) else max(dac_HS22 - STEP, 0)
                    dac_HF2  = min(dac_HF2  + STEP, 255) if (s32_dark and not LF2_active) else max(dac_HF2  - STEP, 0)
                    dac_LS2  = min(dac_LS2  + STEP, 255) if LS2_active else max(dac_LS2  - STEP, 0)
                    dac_LF2  = min(dac_LF2  + STEP, 255) if LF2_active else max(dac_LF2  - STEP, 0)
                else:
                    # ----- Emergency override in AUTO -----
                    dac_HS11 = dac_HS21 = dac_LS1 = dac_HF1 = dac_LF1 = 0
                    dac_HS12 = dac_HS22 = dac_LS2 = dac_HF2 = 0
                    dac_LF2  = min(dac_LF2 + 15, 255)

                # -------- Push to DACs (column-aware) --------
                _write_dac(dac_HS11, 1, column=1)
                _write_dac(dac_HS21, 2, column=1)
                _write_dac(dac_LS1,  3, column=1)
                _write_dac(dac_HF1,  4, column=1)
                _write_dac(dac_LF1,  5, column=1)

                _write_dac(dac_HS12, 1, column=2)
                _write_dac(dac_HS22, 2, column=2)
                _write_dac(dac_LS2,  3, column=2)
                _write_dac(dac_HF2,  4, column=2)
                _write_dac(dac_LF2,  5, column=2)
            else:
                # ----- MANUAL mode -----
                if prev_auto_mode is True or prev_auto_mode is None:
                    _zero_all_dacs()
                dac_HS11 = dac_HS21 = dac_LS1 = dac_HF1 = dac_LF1 = 0
                dac_HS12 = dac_HS22 = dac_LS2 = dac_HF2 = dac_LF2 = 0

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

            cv2.putText(analysis_img, f"HS11: {dac_HS11}", (left_x, top_y + 0*line_h), font, scale, color, thick)
            cv2.putText(analysis_img, f"HS21: {dac_HS21}", (left_x, top_y + 1*line_h), font, scale, color, thick)
            cv2.putText(analysis_img, f"HF1:  {dac_HF1}",  (left_x, top_y + 2*line_h), font, scale, color, thick)
            cv2.putText(analysis_img, f"LS1:  {dac_LS1}",  (left_x, top_y + 3*line_h), font, scale, color, thick)
            cv2.putText(analysis_img, f"LF1:  {dac_LF1}",  (left_x, top_y + 4*line_h), font, scale, color, thick)

            put_right(f"HS12: {dac_HS12}", top_y + 0*line_h)
            put_right(f"HS22: {dac_HS22}", top_y + 1*line_h)
            put_right(f"HF2:  {dac_HF2}",  top_y + 2*line_h)
            put_right(f"LS2:  {dac_LS2}",  top_y + 3*line_h)
            put_right(f"LF2:  {dac_LF2}",  top_y + 4*line_h)

            cv2.imshow("Analysis (C270)", analysis_img)

        # -------- Debounced Day/Night hardware switch (GPIO23) --------
        # Only attempt a switch if not in the middle of one
        if not switch_in_progress:
            changed, new_high = _debounced_dn_change(last_dn_high)
            if changed:
                switch_in_progress = True
                desired_active = "IMX708" if new_high else "IMX327"
                # Stop capturing for this iteration while we switch cameras
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
                    for ch in (1, 2, 3, 4, 5): _write_dac(0, ch, column=1)
                    for ch in (1, 2, 3, 4):     _write_dac(0, ch, column=2)
                    dac_LF2 = 0

            # Normal edge-based EMERGENCY transitions
            if auto_mode:
                if (not emergency_mode) and was_online and now_on_batt:
                    print("UPS transitioned Online -> On Battery. ENTERING EMERGENCY MODE.")
                    emergency_mode = True
                    for ch in (1, 2, 3, 4, 5): _write_dac(0, ch, column=1)
                    for ch in (1, 2, 3, 4):     _write_dac(0, ch, column=2)
                    dac_LF2 = 0
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
        GPIO.cleanup()

if __name__ == "__main__":
    main()
