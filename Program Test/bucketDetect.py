import cv2
import numpy as np


def constrain(value, min_val, max_val):
    return min(max_val, max(min_val, value))


def search_contours(mask, num_color):
    global cX, cY
    contours, hierarchy = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    for contour in contours:
        area = cv2.contourArea(contour)
        x, y, w, h = cv2.boundingRect(contour)
        epsilon = 0.04 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)
        num_vertices = len(approx)
        if area > 5e3 and (4 <= num_vertices <= 5) and (0.75 <= h / w <= 1.25):
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
            else:
                cX, cY = 0, 0

            color_arr.append(num_color)
            deltha_dis.append(abs(round(760 / 2) - cX))
            position_bucket.append([cX, cY])
            cv2.putText(
                frame,
                f"{deltha_dis.index(abs(round(760 / 2) - cX))} {round(760 / 2) - cX}",
                (x, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (255, 0, 255),
                2,
                cv2.LINE_AA,
            )
            cv2.drawContours(frame, [approx], -1, (0, 255, 0), 2)
            cv2.circle(frame, (cX, cY), 7, (0, 0, 255), -1)


def dilate_frame(hue, diff=10, sl=0, vl=0, vh=255):
    lower_hsv = np.array([constrain(hue - diff, 0, 179), sl, vl])
    upper_hsv = np.array([constrain(hue + diff, 0, 179), 255, vh])
    mask = cv2.inRange(hsv_frame, lower_hsv, upper_hsv)
    res_frame = cv2.bitwise_or(frame, frame, mask=mask)
    gray_frame = cv2.cvtColor(res_frame, cv2.COLOR_BGR2GRAY)
    # threshold = cv2.adaptiveThreshold(
    #     mask, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 11, 2.5
    # )
    canny_frame = cv2.Canny(gray_frame, 0, 255)
    kernel = np.ones((5, 5))
    # cv2.imshow("color_search", canny_frame)
    return cv2.dilate(canny_frame, kernel, iterations=1)


cap = cv2.VideoCapture(3)

color_search = np.zeros((200, 200, 3), np.uint8)

text = "None"
hue = [178, 55, 112]
deltha_dis = []
position_bucket = []
color_arr = []
index = 0
pre_index = -1
pre_sent = -1
storage = np.zeros(3)
storage_full = False

while cap.isOpened():
    ref, frame = cap.read()
    frame = cv2.resize(frame, (760, 600))
    frame = cv2.rotate(frame, 1)

    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # B = frame[round(600 / 2), round(760 / 2)][0]
    # G = frame[round(600 / 2), round(760 / 2)][1]
    # R = frame[round(600 / 2), round(760 / 2)][2]
    # color_search[:] = (B, G, R)
    # hue = hsv_frame[round(600 / 2), round(760 / 2)][0]
    # print(hsv_frame[round(600 / 2), round(760 / 2)][0])

    # lower_hsv = np.array([constrain(hue - 10, 0, 255), 81, 55])
    # upper_hsv = np.array([constrain(hue + 10, 0, 179), 255, 255])
    # mask = cv2.inRange(hsv_frame, lower_hsv, upper_hsv)
    # res_frame2 = cv2.bitwise_or(frame, frame, mask=mask)

    dil_frame_red = dilate_frame(hue[0], 10, 110, 85)
    dil_frame_green = dilate_frame(hue[1], 10, 115, 65)
    dil_frame_blue = dilate_frame(hue[2], 10, 110, 85)
    search_contours(dil_frame_red, 1)
    search_contours(dil_frame_green, 2)
    search_contours(dil_frame_blue, 3)

    if len(deltha_dis) != 0:
        if not storage_full:
            if (
                position_bucket[deltha_dis.index(min(deltha_dis))][0]
                > round(760 / 2) + 20
            ):
                text = "Go Right"
            elif (
                position_bucket[deltha_dis.index(min(deltha_dis))][0]
                < round(760 / 2) - 20
            ):
                text = "Go Left"
            elif (
                position_bucket[deltha_dis.index(min(deltha_dis))][1]
                > round(600 / 2) + 20
            ):
                text = "Go Down"
            elif (
                position_bucket[deltha_dis.index(min(deltha_dis))][1]
                < round(600 / 2) - 20
            ):
                text = "Go Up"
            else:
                text = "Stable"
                if pre_index != deltha_dis.index(min(deltha_dis)):
                    pre_index = deltha_dis.index(min(deltha_dis))
                    storage[index] = color_arr[deltha_dis.index(min(deltha_dis))]
                    index += 1
                    index = constrain(index, 0, 2)
        else:
            print(storage)
            if storage[np.argmax(storage == 1)] == 1:
                if (
                    round(760 * 0.25) - 20
                    <= position_bucket[deltha_dis.index(min(deltha_dis))][0]
                    <= round(760 * 0.25) + 20
                ) and (
                    round(600 * 0.75) - 20
                    <= position_bucket[deltha_dis.index(min(deltha_dis))][1]
                    <= round(600 * 0.75) + 20
                ):
                    if pre_sent != storage[np.argmax(storage == 1)]:
                        pre_sent = storage[np.argmax(storage == 1)]
                        print("sent Red")
                        storage[np.argmax(storage == 1)] = 99
                cv2.circle(
                    frame, (round(760 * 0.25), round(600 * 0.75)), 10, (255, 255, 0), -1
                )
            elif storage[np.argmax(storage == 2)] == 2:
                if (
                    round(760 * 0.5) - 20
                    <= position_bucket[deltha_dis.index(min(deltha_dis))][0]
                    <= round(760 * 0.5) + 20
                ) and (
                    round(600 * 0.75) - 20
                    <= position_bucket[deltha_dis.index(min(deltha_dis))][1]
                    <= round(600 * 0.75) + 20
                ):
                    if pre_sent != storage[np.argmax(storage == 2)]:
                        pre_sent = storage[np.argmax(storage == 2)]
                        print("sent Green")
                        storage[np.argmax(storage == 2)] = 99
                cv2.circle(
                    frame, (round(760 * 0.5), round(600 * 0.75)), 10, (255, 255, 0), -1
                )
            elif storage[np.argmax(storage == 3)] == 3:
                if (
                    round(760 * 0.75) - 20
                    <= position_bucket[deltha_dis.index(min(deltha_dis))][0]
                    <= round(760 * 0.75) + 20
                ) and (
                    round(600 * 0.75) - 20
                    <= position_bucket[deltha_dis.index(min(deltha_dis))][1]
                    <= round(600 * 0.75) + 20
                ):
                    if pre_sent != storage[np.argmax(storage == 3)]:
                        pre_sent = storage[np.argmax(storage == 3)]
                        print("sent Blue")
                        storage[np.argmax(storage == 3)] = 99
                cv2.circle(
                    frame, (round(760 * 0.75), round(600 * 0.75)), 10, (255, 255, 0), -1
                )
    else:
        text = "None"
        pre_index = -1
        pre_sent = -1

    # print(storage)
    if (
        np.count_nonzero(storage == 1)
        + np.count_nonzero(storage == 2)
        + np.count_nonzero(storage == 3)
    ) == 3:
        # text = "Storage Full"
        storage_full = True

    cv2.circle(frame, (round(760 / 2), round(600 / 2)), 10, (255, 0, 0), -1)
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
    deltha_dis.clear()
    position_bucket.clear()
    color_arr.clear()
    cv2.imshow("frame", frame)
    # cv2.imshow("mask", dil_frame_green)

    if not ref:
        break

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
