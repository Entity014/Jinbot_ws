import cv2
import numpy as np


def constrain(value, min_val, max_val):
    return min(max_val, max(min_val, value))


def search_contours(mask):
    global cX, cY
    contours, hierarchy = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    for contour in contours:
        area = cv2.contourArea(contour)
        x, y, w, h = cv2.boundingRect(contour)
        epsilon = 0.05 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)
        num_vertices = len(approx)
        if area > 1e3 and num_vertices == 4 and (0.75 <= w / h <= 1.25):
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
            else:
                cX, cY = 0, 0

            cv2.drawContours(frame, [approx], -1, (0, 255, 0), 2)
            cv2.circle(frame, (cX, cY), 7, (0, 0, 255), -1)


cap = cv2.VideoCapture(2)

color_search = np.zeros((200, 200, 3), np.uint8)

cX, cY = 0, 0
text = "None"
index = 0
pre_index = -1
hue = [175, 70, 115]
storage = np.zeros(3)
storage_full = False
position_shelf = np.zeros([3, 2])

while cap.isOpened():
    ref, frame = cap.read()
    frame = cv2.resize(frame, (760, 600))
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # B = frame[round(600 / 2), round(760 / 2)][0]
    # G = frame[round(600 / 2), round(760 / 2)][1]
    # R = frame[round(600 / 2), round(760 / 2)][2]
    # color_search[:] = (B, G, R)
    # hue = hsv_frame[round(600 / 2), round(760 / 2)][0]

    lower_hue = constrain(hue[index] - 10, 0, 255)
    upper_hue = constrain(hue[index] + 10, 0, 179)

    lower_hsv = np.array([lower_hue, 69, 72])
    upper_hsv = np.array([upper_hue, 255, 255])

    mask = cv2.inRange(hsv_frame, lower_hsv, upper_hsv)
    res_frame = cv2.bitwise_and(frame, frame, mask=mask)
    gray_frame = cv2.cvtColor(res_frame, cv2.COLOR_BGR2GRAY)
    canny_frame = cv2.Canny(gray_frame, 50, 200)
    kernel = np.ones((5, 5))
    dil_frame = cv2.dilate(canny_frame, kernel, iterations=1)
    search_contours(dil_frame)

    if not storage_full:
        if cX > round(760 / 2) + 20:
            text = "Go Right"
        elif cX < round(760 / 2) - 20:
            text = "Go Left"
        elif cY > round(600 / 2) + 20:
            text = "Go Down"
        elif cY < round(600 / 2) - 20:
            text = "Go Up"
        else:
            text = "Stable"
            position_shelf[index] = [cX, cY]
            cX, cY = 0, 0
            storage[index] = 1
            index += 1
            index = constrain(index, 0, 2)

    if np.count_nonzero(storage == 1) == 3:
        text = "Storage Full"
        storage_full = True
        min_y = np.max(position_shelf[:, 1])
        position_shelf[:, 1] = min_y
    print(position_shelf)

    cv2.putText(
        frame,
        f"{text}",
        (20, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        1,
        (255, 0, 255),
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        frame,
        f"{hue}",
        (round(760 / 2), round(600 / 2) + 50),
        cv2.FONT_HERSHEY_SIMPLEX,
        1,
        (255, 0, 255),
        2,
        cv2.LINE_AA,
    )

    cv2.circle(frame, (round(760 / 2), round(600 / 2)), 10, (255, 0, 0), -1)

    cv2.imshow("frame", frame)
    # cv2.imshow("color_search", color_search)
    cv2.imshow("mask", dil_frame)

    if not ref:
        break

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
