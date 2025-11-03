# main.py
import cv2
import time
import spidev
from modules.LightingAnalysis import CameraManager, LightingAnalyzer
from modules.DualPicam import PicamFeed
from modules.UPS import get_ups_data

# --- SPI/DAC helpers (two-column control on SPI1 CE0/CE1) ---
SPI_BUS = 1
# Column 1 -> CE1 (device=1), Column 2 -> CE0 (device=0)
COL_TO_DEVICE = {1: 1, 2: 0}

def _write_dac(value: int, channel: int, column: int):
    """
    Send an 8-bit value (0..255) to LTC1665 channel (1..8) on the given column (1 or 2).
    Column 1 is on SPI1 CE1, Column 2 is on SPI1 CE0.
    """
    if not (1 <= channel <= 8):
        raise ValueError("Channel must be 1..8")
    if not (0 <= value <= 255):
        raise ValueError("Value must be 0..255")
    if column not in (1, 2):
        raise ValueError("Column must be 1 or 2")

    cmd = (channel << 12) | (value << 4)  # LTC1665 format
    hi = (cmd >> 8) & 0xFF
    lo = cmd & 0xFF

    dev = COL_TO_DEVICE[column]
    spi = spidev.SpiDev()
    spi.open(SPI_BUS, dev)
    spi.max_speed_hz = 2_000_000
    spi.mode = 0  # Mode 0 for LTC1665
    spi.writebytes([hi, lo])
    spi.close()

def _has_token(status_str: str, token: str) -> bool:
    """Return True if 'token' appears as a separate, case-insensitive token in the UPS status string."""
    if not status_str:
        return False
    toks = status_str.replace(",", " ").lower().split()
    return token.lower() in toks

def _is_online(status_str: str) -> bool:
    # Matches 'OL', including 'OL CHRG'
    return _has_token(status_str, "ol")

def _is_on_battery(status_str: str) -> bool:
    # Matches 'OB', including 'OB DISCHRG'
    return _has_token(status_str, "ob")

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
    dac_HF1  = 0  # High Flood Col1 -> S31+S32 (ch4)  [you can remap later]
    dac_LF1  = 0  # Low  Flood Col1 -> S21+S31 (ch5)

    # -------- Column 2 (right) DAC state --------
    dac_HS12 = 0  # High Spot Row1 Col2 -> S12 (ch1)
    dac_HS22 = 0  # High Spot Row2 Col2 -> S22 (ch2)
    dac_LS2  = 0  # Low  Spot Col2 -> S12+S22 (ch3)
    dac_HF2  = 0  # High Flood Col2 -> S32+S31 (ch4)
    dac_LF2  = 0  # Low  Flood Col2 -> S22+S32 (ch5)
    
    STEP = 5            # Normal ramp step
    THRESHOLD_DARK = 10 # Darkness threshold (%)

    # --- Emergency mode state ---
    emergency_mode = False
    EM_STEP = 15
    prev_ups_status = None

    print("Starting multi-light control (two columns + EMERGENCY MODE).")
    print("Press 's' to switch the HDR camera feed (IMX708 <-> IMX327). Press 'q' to exit.")
    
    while True:
        # -------- Lighting analysis (C270) --------
        analysis_frame = analysis_cam.capture_frame()
        if analysis_frame is not None:
            overall_dark, cell_darkness, analysis_img = analyzer.analyze(analysis_frame)

            # Index mapping for 3x2 grid in row-major:
            # 0:S11  1:S12
            # 2:S21  3:S22
            # 4:S31  5:S32
            dark_S11 = cell_darkness[0]
            dark_S12 = cell_darkness[1]
            dark_S21 = cell_darkness[2]
            dark_S22 = cell_darkness[3]
            dark_S31 = cell_darkness[4]
            dark_S32 = cell_darkness[5]

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

            if not emergency_mode:
                # ----- Normal control: Column 1 mirrors Column 2 logic for its own sectors -----
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
                # ----- Emergency override -----
                # All lights off except LF2 ramps to max
                dac_HS11 = dac_HS21 = dac_LS1 = dac_HF1 = dac_LF1 = 0
                dac_HS12 = dac_HS22 = dac_LS2 = dac_HF2 = 0
                dac_LF2  = min(dac_LF2 + EM_STEP, 255)

            # -------- Push to DACs (column-aware) --------
            # Column 1 on CE1
            _write_dac(dac_HS11, 1, column=1)
            _write_dac(dac_HS21, 2, column=1)
            _write_dac(dac_LS1,  3, column=1)
            _write_dac(dac_HF1,  4, column=1)
            _write_dac(dac_LF1,  5, column=1)

            # Column 2 on CE0
            _write_dac(dac_HS12, 1, column=2)
            _write_dac(dac_HS22, 2, column=2)
            _write_dac(dac_LS2,  3, column=2)
            _write_dac(dac_HF2,  4, column=2)
            _write_dac(dac_LF2,  5, column=2)
            
            # ---------- Overlay (Column 1 on left, Column 2 on right) ----------
            H, W = analysis_img.shape[:2]
            font = cv2.FONT_HERSHEY_SIMPLEX
            scale = 0.7
            thick = 2
            left_x  = 10
            top_y   = 30
            line_h  = 30  # vertical spacing per line
            color   = (0, 255, 0)

            def put_right(text, y):
                (tw, th), _ = cv2.getTextSize(text, font, scale, thick)
                x = W - 10 - tw
                cv2.putText(analysis_img, text, (x, y), font, scale, color, thick)

            # Left: Column 1 labels/values
            cv2.putText(analysis_img, f"HS11: {dac_HS11}", (left_x, top_y + 0*line_h), font, scale, color, thick)
            cv2.putText(analysis_img, f"HS21: {dac_HS21}", (left_x, top_y + 1*line_h), font, scale, color, thick)
            cv2.putText(analysis_img, f"HF1:  {dac_HF1}",  (left_x, top_y + 2*line_h), font, scale, color, thick)
            cv2.putText(analysis_img, f"LS1:  {dac_LS1}",  (left_x, top_y + 3*line_h), font, scale, color, thick)
            cv2.putText(analysis_img, f"LF1:  {dac_LF1}",  (left_x, top_y + 4*line_h), font, scale, color, thick)

            # Right: Column 2 labels/values
            put_right(f"HS12: {dac_HS12}", top_y + 0*line_h)
            put_right(f"HS22: {dac_HS22}", top_y + 1*line_h)
            put_right(f"HF2:  {dac_HF2}",  top_y + 2*line_h)
            put_right(f"LS2:  {dac_LS2}",  top_y + 3*line_h)
            put_right(f"LF2:  {dac_LF2}",  top_y + 4*line_h)

            if emergency_mode:
                cv2.putText(analysis_img, "EMERGENCY MODE", (left_x, top_y + 6*line_h),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,0,255), 3)

            cv2.imshow("Analysis (C270)", analysis_img)

        # -------- HDR feed (toggle IMX708 <-> IMX327) --------
        hdr_src = hdr_cam_primary if active_name == "IMX708" else (
            hdr_cam_alt if hdr_cam_alt is not None else PicamFeed(camera_num=1, af=False)
        )
        if active_name != "IMX708" and hdr_cam_alt is None:
            hdr_cam_alt = hdr_src  # record we created it

        hdr_frame = hdr_src.capture_frame()
        if hdr_frame is not None:
            ups_data = get_ups_data()
            current_status  = ups_data.get("ups.status", "Unknown")
            current_charge  = ups_data.get("battery.charge", "Unknown")
            current_runtime = ups_data.get("battery.runtime", "0")

            # --- EMERGENCY MODE transition detection ---
            now_online = _is_online(current_status)
            now_on_batt = _is_on_battery(current_status)
            was_online = _is_online(prev_ups_status) if prev_ups_status is not None else False

            # Enter EMERGENCY MODE only on Online -> On Battery edge
            if (not emergency_mode) and was_online and now_on_batt:
                print("UPS transitioned Online -> On Battery. ENTERING EMERGENCY MODE.")
                emergency_mode = True
                # Immediately force off non-LF2 channels (both columns)
                for ch in (1,2,3,4,5):
                    _write_dac(0, ch, column=1)  # Col1 off
                for ch in (1,2,3,4):            # LF2 handled by loop ramp
                    _write_dac(0, ch, column=2)

            # Exit EMERGENCY MODE when power returns
            if emergency_mode and now_online:
                print("UPS back Online. EXITING EMERGENCY MODE.")
                emergency_mode = False
                # Optionally reset LF2 to let normal logic take over smoothly:
                # dac_LF2 = 0

            prev_ups_status = current_status

            # Format runtime (sec->HH:MM:SS)
            runtime_sec = int(current_runtime) if str(current_runtime).isdigit() else 0
            h, m, s = runtime_sec // 3600, (runtime_sec % 3600) // 60, runtime_sec % 60
            formatted_runtime = f"{h:02}:{m:02}:{s:02}"

            # Overlay UPS + active camera label
            cv2.putText(hdr_frame, f"Active: {active_name}",         (10, 30),  cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,255), 2)
            cv2.putText(hdr_frame, f"UPS Status: {current_status}",  (10, 60),  cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
            cv2.putText(hdr_frame, f"Charge: {current_charge}%",     (10, 90),  cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
            cv2.putText(hdr_frame, f"Runtime: {formatted_runtime}",  (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)

            if emergency_mode:
                cv2.putText(hdr_frame, "EMERGENCY MODE", (10, 160),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0,0,255), 3)

            cv2.imshow("HDR Feed (CSI)", hdr_frame)

        # -------- Key handling --------
        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), 13, 10):  # 'q' or Enter
            print("Exiting…")
            break
        elif key == ord('s'):
            # Switch which CSI cam is active
            if active_name == "IMX708":
                hdr_cam_primary.stop()
                if hdr_cam_alt is None:
                    hdr_cam_alt = PicamFeed(camera_num=1, af=False)
                else:
                    hdr_cam_alt.start()
                active_name = "IMX327"
                print("Switched HDR feed to IMX327 (camera 1).")
            else:
                if hdr_cam_alt is not None:
                    hdr_cam_alt.stop()
                hdr_cam_primary.start()
                active_name = "IMX708"
                print("Switched HDR feed to IMX708 (camera 0).")

        time.sleep(0.05)
    
    # -------- Cleanup --------
    analysis_cam.release()
    if hdr_cam_primary: hdr_cam_primary.release()
    if hdr_cam_alt:     hdr_cam_alt.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
