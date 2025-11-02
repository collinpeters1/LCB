#!/usr/bin/env python3
import cv2
import time
from modules.LightingAnalysis import CameraManager, LightingAnalyzer

def main():
    # Our OV9281/analysis camera on /dev/video0 (Config.ANALYSIS_CAMERA_INDEX = 0)
    cam = CameraManager(camera_index=0)

    # LightingAnalyzer is our new logic:
    # - 3x2 grid => [S11,S12,S21,S22,S31,S32]
    # - threshold_value is the global cutoff for "bright vs dark" when in global mode
    analyzer = LightingAnalyzer(threshold_value=40, rows=3, cols=2)

    print("Starting lighting analysis preview.")
    print("Controls:")
    print("  q          : quit")
    print("  [ / ]      : lower / raise global threshold")
    print("  \\          : toggle global ↔ local_otsu")
    print()

    try:
        while True:
            frame = cam.capture_frame()
            if frame is None:
                # No frame this loop, just try again next tick
                time.sleep(0.01)
                continue

            # Run the real analysis pipeline:
            # overall_dark  -> single % for whole frame
            # cell_darkness -> [S11,S12,S21,S22,S31,S32] darkness %
            # annotated_img -> grayscale+overlay (grid lines + % text)
            overall_dark, cell_darkness, annotated_img = analyzer.analyze(frame)

            # Dump numbers to console so you can correlate to overlay
            # (this prints every loop; you can comment it if it's too spammy)
            print(
                f"Overall: {overall_dark:.2f}% | "
                f"S11={cell_darkness[0]:.1f}% "
                f"S12={cell_darkness[1]:.1f}% "
                f"S21={cell_darkness[2]:.1f}% "
                f"S22={cell_darkness[3]:.1f}% "
                f"S31={cell_darkness[4]:.1f}% "
                f"S32={cell_darkness[5]:.1f}%"
            )

            #
            # On-screen HUD for debugging:
            #  - current threshold mode (global vs local_otsu)
            #  - current global threshold value (even if we're in Otsu mode,
            #    it's still good to see what the global setting is)
            #
            hud_text = (
                f"MODE={analyzer.threshold_mode}  "
                f"T={analyzer.get_threshold()}"
            )
            cv2.putText(
                annotated_img,
                hud_text,
                (10, annotated_img.shape[0] - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2
            )

            # Show both the raw camera feed and the analyzed/annotated view
            cv2.imshow("Analysis Camera (raw)", frame)
            cv2.imshow("Lighting Analysis Overlay", annotated_img)

            # --- key handling ---
            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 13, 10):  # q or Enter
                print("Exiting preview...")
                break

            elif key == ord('['):
                analyzer.adjust_threshold(-5)

            elif key == ord(']'):
                analyzer.adjust_threshold(+5)

            elif key == ord('\\'):
                # flip between "global" and "local_otsu"
                new_mode = "local_otsu" if analyzer.threshold_mode == "global" else "global"
                analyzer.set_threshold_mode(new_mode)

            # match our main loop timing-ish
            time.sleep(0.01)

    finally:
        try:
            cam.release()
        except Exception:
            pass
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass

if __name__ == "__main__":
    main()
