import cv2
import time
from modules.LightingAnalysis import CameraManager, LightingAnalyzer
from modules.IMX708 import CameraManagerPicamera
from modules.DACControl import update_dac
from modules.UPS import get_ups_data

def main():
    # Initialize cameras and modules
    analysis_cam = CameraManager(camera_index=0)
    hdr_cam = CameraManagerPicamera()
    analyzer = LightingAnalyzer(threshold_value=15, rows=3, cols=2)
    
    dac_value = 128
    step = 5
    active_channel = 1

    print("Starting dual camera display. Press 'q' to exit.")

    while True:
        analysis_frame = analysis_cam.capture_frame()
        if analysis_frame is not None:
            overall_dark, cell_darkness, analysis_img = analyzer.analyze(analysis_frame)
            top_left_darkness = cell_darkness[0]
            print(f"C270 - Top left cell darkness: {top_left_darkness:.2f}%")
            if top_left_darkness > 10:
                dac_value = min(dac_value + step, 255)
            else:
                dac_value = max(dac_value - step, 0)
            update_dac(dac_value, active_channel)
            cv2.imshow("Analysis (C270)", analysis_img)
        
        hdr_frame = hdr_cam.capture_frame()
        if hdr_frame is not None:
            ups_data = get_ups_data()
            current_status = ups_data.get("ups.status", "Unknown")
            current_charge = ups_data.get("battery.charge", "Unknown")
            current_runtime = ups_data.get("battery.runtime", "Unknown")
            overlay_text = f"UPS Status: {current_status}  Charge: {current_charge}%  Runtime: {current_runtime}s"
            cv2.putText(hdr_frame, overlay_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                        0.7, (255, 255, 255), 2, cv2.LINE_AA)
            cv2.imshow("HDR Feed with UPS Overlay (IMX708)", hdr_frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        # Optionally reduce delay or remove entirely for higher FPS.
        time.sleep(0.1)

    analysis_cam.release()
    hdr_cam.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
