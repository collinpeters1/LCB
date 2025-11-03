import cv2
import time
import numpy as np

class CameraManager:
    """
    Manages camera initialization, frame capture, and cleanup for the C270 camera.
    """
    def __init__(self, camera_index=0):
        self.camera_index = camera_index
        self.cap = cv2.VideoCapture(self.camera_index)
        if not self.cap.isOpened():
            raise Exception("Cannot open camera")
        print("Camera initialized for index", self.camera_index)

    def capture_frame(self):
        ret, frame = self.cap.read()
        if ret:
            return frame
        return None

    def release(self):
        self.cap.release()
        cv2.destroyAllWindows()
        print("Camera released.")

class LightingAnalyzer:
    """
    Processes frames to analyze lighting conditions.
    Converts images to HSV, thresholds the V channel,
    computes overall darkness and per-cell darkness for a 3x2 grid,
    and draws grid lines with annotations.
    """
    def __init__(self, threshold_value=15, rows=3, cols=2):
        self.threshold_value = threshold_value
        self.rows = rows
        self.cols = cols

    def analyze(self, frame):
        hsv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        v_channel = hsv_image[:, :, 2]
        ret, thresh_img = cv2.threshold(v_channel, self.threshold_value, 255, cv2.THRESH_BINARY)
        annotated_img = cv2.cvtColor(thresh_img, cv2.COLOR_GRAY2BGR)
        total_pixels = v_channel.size
        dark_pixels = np.count_nonzero(thresh_img)
        overall_darkness = 100 - ((dark_pixels / total_pixels) * 100)
        height, width = thresh_img.shape
        cell_height = height // self.rows
        cell_width = width // self.cols
        cell_darkness = []
        for row in range(self.rows):
            for col in range(self.cols):
                start_y = row * cell_height
                end_y = (row + 1) * cell_height if row < self.rows - 1 else height
                start_x = col * cell_width
                end_x = (col + 1) * cell_width if col < self.cols - 1 else width
                cell = thresh_img[start_y:end_y, start_x:end_x]
                cell_total = cell.size
                cell_dark = np.count_nonzero(cell)
                darkness = 100 - ((cell_dark / cell_total) * 100)
                cell_darkness.append(darkness)
                text = f'{darkness:.1f}%'
                text_size, _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.9, 3)
                text_width, text_height = text_size
                center_x = start_x + (cell_width - text_width) // 2
                center_y = start_y + (cell_height + text_height) // 2
                cv2.putText(annotated_img, text, (center_x, center_y), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 155, 255), 3)
        for r in range(1, self.rows):
            y = r * cell_height
            cv2.line(annotated_img, (0, y), (width, y), (0, 155, 255), 3)
        for c in range(1, self.cols):
            x = c * cell_width
            cv2.line(annotated_img, (x, 0), (x, height), (0, 155, 255), 3)
        return overall_darkness, cell_darkness, annotated_img
