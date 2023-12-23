import cv2
import numpy as np


cap = cv2.VideoCapture(2)

while cap.isOpened():
    ref, frame = cap.read()
    frame = cv2.flip(frame, 0)
    frame = cv2.resize(frame, (720, 680))

    cv2.imshow("webcam", frame)
    if not ref:
        break

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
