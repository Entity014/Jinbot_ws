import cv2
import numpy as np


def search_contours(frame, mask):
    global cX, cY
    height, width, _ = frame.shape
    contours, hierarchy = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    for contour in contours:
        area = cv2.contourArea(contour)
        x, y, w, h = cv2.boundingRect(contour)
        epsilon = 0.001 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)
        num_vertices = len(approx)
        (center_x, center_y), radius = cv2.minEnclosingCircle(contour)
        radius = int(radius)

        if (
            area >= 10000
            and radius <= 170
            and num_vertices >= 40
            and num_vertices <= 70
        ):
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
            else:
                cX, cY = 0, 0

            print(num_vertices)
            if cX > (width // 2) - 20 and cX < (width // 2) + 20:
                print(f"no hold")
            cv2.circle(frame, (int(center_x), int(center_y)), radius, (0, 255, 0), 2)
            cv2.circle(frame, (cX, cY), 7, (0, 0, 255), -1)
            cv2.drawContours(frame, [approx], -1, (0, 255, 0), 2)
            cv2.putText(
                frame,
                f"( {num_vertices} )",
                (x, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (255, 0, 0),
                2,
            )


def detect_black_color(frame):
    # Convert the frame to the HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define a lower and upper threshold for the black color in HSV
    lower_black = np.array([0, 0, 0])
    upper_black = np.array([179, 255, 30])
    # Create a mask using the inRange function to threshold the image
    mask = cv2.inRange(hsv, lower_black, upper_black)
    blurred = cv2.GaussianBlur(mask, (7, 7), 1.5)
    kernel = np.ones((5, 5))
    opening = cv2.morphologyEx(blurred, cv2.MORPH_OPEN, kernel, iterations=2)
    # canny_frame = cv2.Canny(opening, 0, 255)
    # imgDil = cv2.dilate(canny_frame, kernel, iterations=1)

    return opening


def crop_vertical_half(frame):
    # Get the width and height of the frame
    height, width, _ = frame.shape

    # Crop the left half of the frame
    cropped_frame = frame[height // 2 :, 160:600, :]

    return cropped_frame


# Open a connection to the webcam (0 represents the default camera)
cap = cv2.VideoCapture("My Movie.mov")

while True:
    # Read a frame from the webcam
    ret, frame = cap.read()
    frame = cv2.resize(frame, (760, 600))
    cropped_frame = crop_vertical_half(frame)
    result = detect_black_color(cropped_frame)
    search_contours(cropped_frame, result)
    cv2.line(frame, (340, 0), (340, 600), (255, 0, 0), 2)
    cv2.line(frame, (380, 0), (380, 600), (255, 0, 0), 2)
    cv2.imshow("Original", cropped_frame)
    cv2.imshow("Black Color Detection", frame)
    if cv2.waitKey(10) & 0xFF == ord("q"):
        break

    if cap.get(cv2.CAP_PROP_POS_FRAMES) == cap.get(cv2.CAP_PROP_FRAME_COUNT):
        cap.set(cv2.CAP_PROP_POS_FRAMES, 0)

# Release the webcam and close all windows
cap.release()
cv2.destroyAllWindows()
