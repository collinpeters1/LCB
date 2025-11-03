# modules/DualPicam.py
from picamera2 import Picamera2
from libcamera import controls
import cv2

class PicamFeed:
    """
    Minimal wrapper for a Picamera2 CSI camera.
    - camera_num: 0 or 1
    - af: enable Continuous AF if True
    """
    def __init__(self, camera_num=0, af=True):
        self.camera_num = camera_num
        self.picam2 = Picamera2(camera_num=self.camera_num)
        cfg = self.picam2.create_preview_configuration()
        self.picam2.configure(cfg)
        self.started = False
        self.af = af
        self.start()  # start immediately by default

    def start(self):
        if not self.started:
            self.picam2.start()
            if self.af:
                self.picam2.set_controls({"AfMode": controls.AfModeEnum.Continuous})
            self.started = True

    def stop(self):
        if self.started:
            self.picam2.stop()
            self.started = False

    def capture_frame(self):
        """
        Returns a BGR frame (OpenCV-ready).
        """
        frame_rgb = self.picam2.capture_array()
        return cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)

    def release(self):
        self.stop()
