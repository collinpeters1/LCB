from picamera2 import Picamera2
import cv2
import numpy as np

class CameraManagerPicamera:
    """
    Manages the Picamera2 for capturing frames from the IMX708.
    Uses the preview configuration to mimic the built‑in preview.
    Applies a blue-channel correction to reduce the blue tint.
    """
    def __init__(self):
        self.picam2 = Picamera2()
        # Use the preview configuration (as used in IMX708Preview.py)
        config = self.picam2.create_preview_configuration()
        self.picam2.configure(config)
        self.picam2.start()
        from libcamera import controls
        # Enable continuous autofocus (and leave AWB on for now)
        self.picam2.set_controls({"AfMode": controls.AfModeEnum.Continuous})
        print("Picamera2 started with preview configuration.")

    def capture_frame(self):
        """
        Captures a frame, converts from RGB to BGR, then applies a correction
        to reduce the intensity of the blue channel.
        """
        frame = self.picam2.capture_array()
        # Convert from RGB (Picamera2 default) to BGR (OpenCV default)
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        # Apply a blue channel correction: in OpenCV, channel 0 is blue.
        correction_factor = 0.8  # Lower this factor to reduce blue intensity further.
        # Convert to float for scaling then back to original type.
        frame[:,:,0] = (frame[:,:,0].astype(np.float32) * correction_factor).astype(frame.dtype)
        return frame

    def release(self):
        self.picam2.stop()
        print("Picamera2 stopped.")
