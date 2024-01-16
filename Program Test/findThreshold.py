import cv2
import numpy as np

cap = cv2.VideoCapture("/dev/video2")


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
        epsilon = 0.04 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)
        num_vertices = len(approx)
        if area > 1000:
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
            else:
                cX, cY = 0, 0
            cv2.circle(frame, (cX, cY), 7, (0, 0, 255), -1)
            cv2.drawContours(frame, [approx], -1, (0, 255, 0), 2)
            cv2.putText(
                frame,
                f"{num_vertices}",
                (x, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (255, 0, 255),
                2,
                cv2.LINE_AA,
            )


def crop_vertical_half(frame):
    # Get the width and height of the frame
    height, width, _ = frame.shape

    # Crop the left half of the frame
    cropped_frame = frame[: height // 2, :, :]

    return cropped_frame


while True:
    # Read a frame from the webcam
    ret, frame = cap.read()
    frame = crop_vertical_half(frame)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (7, 7), 1)
    _, threshold = cv2.threshold(blurred, 40, 255, cv2.THRESH_BINARY_INV)
    search_contours(threshold)

    cv2.imshow("Original", frame)
    # cv2.imshow("Original2", threshold)

    # Break the loop if the 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

# Release the webcam and close all windows
cap.release()
cv2.destroyAllWindows()
