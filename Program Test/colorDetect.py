import cv2
import numpy as np


def constrain(value, min_val, max_val):
    return min(max_val, max(min_val, value))


def search_contours(mask, pre_color, color, i):
    contours, hierarchy = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    for contour in contours:
        area = cv2.contourArea(contour)
        x, y, w, h = cv2.boundingRect(contour)
        epsilon = 0.01 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)
        num_vertices = len(approx)
        if area > 1e4:
            color = f"Red {i}"
            if pre_color != color:
                i += 1
                pre_color = color
            cv2.putText(
                frame,
                f"{color}",
                (x, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (255, 0, 255),
                2,
                cv2.LINE_AA,
            )
            cv2.drawContours(frame, [approx], -1, (0, 255, 0), 2)
        print(color)


def dilate_frame(hue, diff=10, sl=0, vl=0, vh=255):
    lower_hsv = np.array([constrain(hue - diff, 0, 179), sl, vl])
    upper_hsv = np.array([constrain(hue + diff, 0, 179), 255, vh])
    mask = cv2.inRange(hsv_frame, lower_hsv, upper_hsv)
    return mask


cap = cv2.VideoCapture("My Movie 1.mov")

i = 0
color = "None"
pre_color = "None"
while True:
    ref, frame = cap.read()
    frame = cv2.resize(frame, (760, 600))

    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    dil_frame_red = dilate_frame(19, 10, 110, 85)
    search_contours(dil_frame_red, pre_color, color, i)

    cv2.imshow("frame", frame)

    if not ref:
        break

    if cv2.waitKey(10) & 0xFF == ord("q"):
        break

    if cap.get(cv2.CAP_PROP_POS_FRAMES) == cap.get(cv2.CAP_PROP_FRAME_COUNT):
        cap.set(cv2.CAP_PROP_POS_FRAMES, 0)

cap.release()
cv2.destroyAllWindows()
