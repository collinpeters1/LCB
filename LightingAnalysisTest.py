import cv2
import time
import numpy as np

class CameraManager:
    """
    Manages camera initialization, frame capture, and cleanup.
    """
    def __init__(self, camera_index=0):
        self.camera_index = camera_index
        self.cap = cv2.VideoCapture(self.camera_index)
        if not self.cap.isOpened():
            raise Exception("Cannot open camera")
        print("Camera initialized.")

    def capture_frame(self):
        ret, frame = self.cap.read()
        if ret:
            return frame
        return None

    def release(self):
        self.cap.release()
        cv2.destroyAllWindows()
        print("Camera released and windows destroyed.")


class LightingAnalyzer:
    """
    Processes frames to analyze lighting conditions.
    Converts images to HSV, thresholds the V channel,
    computes overall darkness and darkness for each cell in a 3x2 grid,
    and draws red grid lines and red text to annotate each sector.
    """
    def __init__(self, threshold_value=15, rows=3, cols=2):
        self.threshold_value = threshold_value
        self.rows = rows
        self.cols = cols

    def analyze(self, frame):
        """
        Converts the frame to HSV, thresholds the V channel,
        calculates the overall darkness, and divides the image into a grid
        (3 rows x 2 columns) to compute and annotate darkness for each cell.
        Draws grid lines and annotations in red.
        Returns overall darkness, a list of cell darkness values, and the annotated threshold image.
        """
        # Convert image to HSV and extract the V channel
        hsv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        v_channel = hsv_image[:, :, 2]

        # Threshold the V channel to separate bright and dark areas
        ret, thresh_img = cv2.threshold(v_channel, self.threshold_value, 255, cv2.THRESH_BINARY)

        # Convert the threshold image to BGR for colored annotations
        annotated_img = cv2.cvtColor(thresh_img, cv2.COLOR_GRAY2BGR)

        # Calculate overall darkness percentage
        total_pixels = v_channel.size
        dark_pixels = np.count_nonzero(thresh_img)
        overall_darkness = 100 - ((dark_pixels / total_pixels) * 100)

        # Get image dimensions and calculate cell size
        height, width = thresh_img.shape
        cell_height = height // self.rows
        cell_width = width // self.cols

        cell_darkness = []  # Will hold darkness percentage for each cell
        
        # Iterate over the grid cells to calculate darkness and annotate
        for row in range(self.rows):
            for col in range(self.cols):
                # Compute cell boundaries
                start_y = row * cell_height
                end_y = (row + 1) * cell_height if row < self.rows - 1 else height
                start_x = col * cell_width
                end_x = (col + 1) * cell_width if col < self.cols - 1 else width

                cell = thresh_img[start_y:end_y, start_x:end_x]
                cell_total = cell.size
                cell_dark = np.count_nonzero(cell)
                darkness = 100 - ((cell_dark / cell_total) * 100)
                cell_darkness.append(darkness)

                # Annotate the cell with the darkness percentage in red (BGR: 0,0,255)
                text = f'{darkness:.1f}%'
                text_size, _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
                text_width, text_height = text_size
                center_x = start_x + (cell_width - text_width) // 2
                center_y = start_y + (cell_height + text_height) // 2
                cv2.putText(annotated_img, text, (center_x, center_y), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        # Draw grid lines in red
        # Draw horizontal lines
        for r in range(1, self.rows):
            y = r * cell_height
            cv2.line(annotated_img, (0, y), (width, y), (0, 0, 255), 2)
        # Draw vertical lines
        for c in range(1, self.cols):
            x = c * cell_width
            cv2.line(annotated_img, (x, 0), (x, height), (0, 0, 255), 2)

        return overall_darkness, cell_darkness, annotated_img


def main():
    # Initialize the camera and lighting analyzer
    cam = CameraManager(camera_index=0)
    analyzer = LightingAnalyzer(threshold_value=15, rows=3, cols=2)

    print("Starting lighting analysis. Press 'q' to exit.")
    while True:
        frame = cam.capture_frame()
        if frame is not None:
            overall_dark, cell_darkness, annotated_img = analyzer.analyze(frame)
            print(f"Overall darkness: {overall_dark:.2f}%")
            print(f"Cell darkness values: {cell_darkness}")

            # Display the original frame and the annotated threshold image
            cv2.imshow("Camera Frame", frame)
            cv2.imshow("Annotated Threshold Image", annotated_img)

        # Exit if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        time.sleep(1)

    # Cleanup resources
    cam.release()


if __name__ == "__main__":
    main()
