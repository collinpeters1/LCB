from picamera2 import Picamera2
import time

# Initialize the second camera (IMX327 on disp1)
picam2 = Picamera2(camera_num=1)
picam2.start(show_preview=True)

try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("Exiting preview.")

picam2.stop()
