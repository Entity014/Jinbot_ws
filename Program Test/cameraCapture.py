import cv2
import numpy as np


def on_button_click(event, x, y, flags, param):
    global button_pressed
    if event == cv2.EVENT_LBUTTONDOWN:
        button_pressed = True
    elif event == cv2.EVENT_LBUTTONUP:
        button_pressed = False


cap = cv2.VideoCapture(7)
cv2.namedWindow("webcam")
cv2.setMouseCallback("webcam", on_button_click)
button_pressed = False
index = 0

while cap.isOpened():
    ref, frame = cap.read()
    # frame = cv2.flip(frame, 1)
    frame = cv2.resize(frame, (720, 680))
    if button_pressed:
        cv2.imwrite(f"/home/entity014/Jinbot_ws/Program Test/image/{index}.png", frame)
        print(f"Image saved successfully. {index}")
        index += 1

    cv2.imshow("webcam", frame)
    if not ref:
        break

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
