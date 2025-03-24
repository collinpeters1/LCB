#from picamera2 documentation
from picamera2 import Picamera2
from libcamera import controls
import os
os.system("v4l2-ctl --set-ctrl wide_dynamic_range=1 -d /dev/v4l-subdev0")
picam2 = Picamera2()
picam2.start(show_preview=True)
picam2.set_controls({"AfMode": controls.AfModeEnum.Continuous})