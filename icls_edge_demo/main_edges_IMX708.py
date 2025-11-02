import cv2
from modules.IMX708_ed import CameraManagerPicamera   # your existing helper
from modules.EdgeDetector import EdgeDetector      # new module you added

WIN = "IMX708_Edge_View"

def main():
    cam = CameraManagerPicamera()          # Starts Pi Camera 3 (IMX708)
    detector = EdgeDetector(low_thresh=50) # Adjustable via trackbar

    cv2.namedWindow(WIN)
    cv2.createTrackbar(
        "Min Threshold",   # label
        WIN,               # parent window
        detector.low_thresh,
        100,               # max value
        lambda v: detector.set_threshold(v)
    )

    try:
        while True:
            frame = cam.capture_frame()    # BGR frame from IMX708
            edged = detector.apply(frame)  # burnt‑orange edges
            cv2.imshow(WIN, edged)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 27):      # q or Esc quits
                break
    finally:
        cam.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
