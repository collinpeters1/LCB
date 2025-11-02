import cv2
import numpy as np

class EdgeDetector:
    """
    Real‑time Canny edge detector that paints edges burnt‑orange.
    """
    BURNT_ORANGE_BGR = (0, 87, 191)      # B, G, R

    def __init__(self, low_thresh=50, ratio=3, kernel_size=3):
        self.low_thresh = int(low_thresh)
        self.ratio = ratio
        self.kernel_size = kernel_size

    def set_threshold(self, val: int):
        self.low_thresh = max(1, int(val))

    def apply(self, frame):
        gray   = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur   = cv2.blur(gray, (3, 3))
        edges  = cv2.Canny(blur,
                           self.low_thresh,
                           self.low_thresh * self.ratio,
                           self.kernel_size)
        # Overlay burnt‑orange edges on top of original frame
        colour = frame.copy()
        mask   = edges.astype(bool)
        colour[mask] = self.BURNT_ORANGE_BGR
        return colour
