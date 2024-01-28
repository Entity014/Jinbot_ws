import cv2
import numpy as np


cap = cv2.VideoCapture("/dev/video4")
# result = cv2.VideoWriter("table4.avi", cv2.VideoWriter_fourcc(*"MJPG"), 10, (720, 680))

while cap.isOpened():
    ref, frame = cap.read()
    # frame = cv2.flip(frame, 0)
    # frame = cv2.flip(frame, 1)
    # frame = cv2.resize(frame, (720, 680))
    # result.write(frame)
    cv2.imshow("webcam", frame)
    if not ref:
        break

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
# result.release()
cv2.destroyAllWindows()
