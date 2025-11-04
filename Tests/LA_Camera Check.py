import cv2, numpy as np

cap = cv2.VideoCapture(0)  # change index if needed
ret, frame = cap.read()
print("ret:", ret)
print("shape:", frame.shape)   # (H,W) means mono; (H,W,3) means BGR
print("dtype:", frame.dtype)   # usually uint8
fourcc = int(cap.get(cv2.CAP_PROP_FOURCC))
print("fourcc:", "".join([chr((fourcc >> (8*i)) & 0xFF) for i in range(4)]))
cap.release()
