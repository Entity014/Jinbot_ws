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
        epsilon = 0.01 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)
        num_vertices = len(approx)
        if area > 1000:
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
            else:
                cX, cY = 0, 0
            cv2.drawContours(frame, [approx], -1, (0, 255, 0), 2)
            cv2.circle(frame, (cX, cY), 7, (0, 0, 255), -1)
            cv2.circle(frame, (380, 300), 7, (255, 0, 0), -1)


def dilate_frame(hue=0, diff=0, sl=0, vl=0):
    lower_hsv = np.array([constrain(hue - diff, 0, 179), sl, vl])
    upper_hsv = np.array([179, 255, 255])
    mask = cv2.inRange(hsv_frame, lower_hsv, upper_hsv)
    # res_frame = cv2.bitwise_or(frame, frame, mask=mask)
    # gray_frame = cv2.cvtColor(res_frame, cv2.COLOR_BGR2GRAY)
    # threshold = cv2.adaptiveThreshold(
    #     mask, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 11, 2.5
    # )
    # canny_frame = cv2.Canny(gray_frame, 0, 255)
    # kernel = np.ones((5, 5))
    # cv2.imshow("color_search", canny_frame)
    # return cv2.dilate(canny_frame, kernel, iterations=1)
    return mask


cap = cv2.VideoCapture("/dev/video2")

color_search = np.zeros((200, 200, 3), np.uint8)

while cap.isOpened():
    ref, frame = cap.read()
    frame = cv2.resize(frame, (760, 600))
    frame = cv2.rotate(frame, 1)
    # frame_copy = frame.copy()
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    dil_frame_mask = dilate_frame()
    search_contours(dil_frame_mask)
    # print(np.interp(cX, [0, 760], [0, 180]))

    cv2.imshow("frame1", dil_frame_mask)
    cv2.imshow("frame", frame)
    if not ref:
        break

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
