import cv2, time

# Try /dev/video0 first (your listing shows OV9281 there)
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)

# FORCE MJPG first, then size/fps (order matters for many UVC cams)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH,  1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
cap.set(cv2.CAP_PROP_FPS, 60)

ok, frame = cap.read()
print("Opened:", cap.isOpened(), "Read:", ok)
if ok:
    print("Shape:", frame.shape, "dtype:", frame.dtype)
    cv2.imshow("OV9281 MJPG 1280x720@60", frame)
    cv2.waitKey(250)

# quick fps pulse
t0 = time.time(); n=0
while time.time()-t0 < 1.0:
    ok, frame = cap.read()
    if not ok: break
    n += 1
print("Frames in ~1s:", n)

cap.release()
cv2.destroyAllWindows()
