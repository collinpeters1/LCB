import cv2
import time
from modules.LightingAnalysis import CameraManager, LightingAnalyzer
from modules.IMX708 import CameraManagerPicamera
from modules.DACControl import update_dac
from modules.UPS import get_ups_data

def main():
    # Initialize cameras: C270 for lighting analysis and IMX708 for the HDR feed.
    analysis_cam = CameraManager(camera_index=0)
    hdr_cam = CameraManagerPicamera()
    analyzer = LightingAnalyzer(threshold_value=15, rows=3, cols=2)
    
    # Initialize DAC outputs for each light.
    dac_HS11 = 0  # High angle Spot for S11 (channel 1)
    dac_HS21 = 0  # High angle Spot for S21 (channel 2)
    dac_HF1  = 0  # High Flood for S31 (channel 3)
    dac_LS1  = 0  # Low angle Spot for S11 and S21 (channel 4)
    dac_LF1  = 0  # Low angle Flood for S21 and S31 (channel 5)
    
    STEP = 5                  # Step size for ramping the DAC value.
    THRESHOLD_DARK = 10       # Darkness threshold (in %)
    
    print("Starting multi-light control simulation (one column). Press 'q' to exit.")
    
    while True:
        # Capture frame from the C270 for lighting analysis.
        analysis_frame = analysis_cam.capture_frame()
        if analysis_frame is not None:
            # analyzer.analyze returns overall darkness, an array of cell darkness values, and an annotated image.
            overall_dark, cell_darkness, analysis_img = analyzer.analyze(analysis_frame)
            # Extract the left column sectors: S11, S21, S31 (indices 0, 2, 4).
            dark_S11 = cell_darkness[0]
            dark_S21 = cell_darkness[2]
            dark_S31 = cell_darkness[4]
            
            # Determine if each sector is dark.
            s11_dark = dark_S11 >= THRESHOLD_DARK
            s21_dark = dark_S21 >= THRESHOLD_DARK
            s31_dark = dark_S31 >= THRESHOLD_DARK
            
            # Define multi-sector conditions.
            LS1_active = s11_dark and s21_dark   # Low angle spot covers S11 and S21.
            LF1_active = s21_dark and s31_dark   # Low angle flood covers S21 and S31.
            
            # Update individual DAC values:
            # HS11: Increase if S11 is dark and LS1 is not active.
            if s11_dark and not LS1_active:
                dac_HS11 = min(dac_HS11 + STEP, 255)
            else:
                dac_HS11 = max(dac_HS11 - STEP, 0)
            
            # HS21: Increase if S21 is dark and neither LS1 nor LF1 are active.
            if s21_dark and not LS1_active and not LF1_active:
                dac_HS21 = min(dac_HS21 + STEP, 255)
            else:
                dac_HS21 = max(dac_HS21 - STEP, 0)
            
            # HF1: Increase if S31 is dark and LF1 is not active.
            if s31_dark and not LF1_active:
                dac_HF1 = min(dac_HF1 + STEP, 255)
            else:
                dac_HF1 = max(dac_HF1 - STEP, 0)
            
            # LS1: Increase if both S11 and S21 are dark.
            if LS1_active:
                dac_LS1 = min(dac_LS1 + STEP, 255)
            else:
                dac_LS1 = max(dac_LS1 - STEP, 0)
            
            # LF1: Increase if both S21 and S31 are dark.
            if LF1_active:
                dac_LF1 = min(dac_LF1 + STEP, 255)
            else:
                dac_LF1 = max(dac_LF1 - STEP, 0)
            
            # Update DAC outputs with the new values.
            update_dac(dac_HS11, 1)  # HS11 on channel 1.
            update_dac(dac_HS21, 2)  # HS21 on channel 2.
            update_dac(dac_HF1, 3)   # HF1 on channel 3.
            update_dac(dac_LS1, 4)   # LS1 on channel 4.
            update_dac(dac_LF1, 5)   # LF1 on channel 5.
            
            # Overlay measured darkness and current DAC values on the analysis image.
            cv2.putText(analysis_img, f"S11: {dark_S11:.1f}%", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
            cv2.putText(analysis_img, f"S21: {dark_S21:.1f}%", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
            cv2.putText(analysis_img, f"S31: {dark_S31:.1f}%", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
            cv2.putText(analysis_img, f"HS11: {dac_HS11}", (200, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
            cv2.putText(analysis_img, f"HS21: {dac_HS21}", (200, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
            cv2.putText(analysis_img, f"HF1:  {dac_HF1}", (200, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
            cv2.putText(analysis_img, f"LS1:  {dac_LS1}", (200, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
            cv2.putText(analysis_img, f"LF1:  {dac_LF1}", (200, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
            
            cv2.imshow("Analysis (C270)", analysis_img)
            
            print(f"S11: {dark_S11:.1f}%, S21: {dark_S21:.1f}%, S31: {dark_S31:.1f}%")
            print(f"HS11: {dac_HS11}, HS21: {dac_HS21}, HF1: {dac_HF1}, LS1: {dac_LS1}, LF1: {dac_LF1}")
        
        # Capture HDR feed from the IMX708 and overlay UPS data.
        hdr_frame = hdr_cam.capture_frame()
        if hdr_frame is not None:
            ups_data = get_ups_data()
            current_status = ups_data.get("ups.status", "Unknown")
            current_charge = ups_data.get("battery.charge", "Unknown")
            current_runtime = ups_data.get("battery.runtime", "Unknown")
            overlay_text = f"UPS: {current_status}  Charge: {current_charge}%  Runtime: {current_runtime}s"
            cv2.putText(hdr_frame, overlay_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2, cv2.LINE_AA)
            cv2.imshow("HDR Feed (IMX708)", hdr_frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        
        time.sleep(0.1)
    
    analysis_cam.release()
    hdr_cam.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
