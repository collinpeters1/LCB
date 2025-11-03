from picamera2 import Picamera2
from libcamera import controls
import time

picam2 = Picamera2()
picam2.start(show_preview=True)
picam2.set_controls({"AfMode": controls.AfModeEnum.Continuous})
while True:
	time.sleep(1) 

picam2.stop()  

