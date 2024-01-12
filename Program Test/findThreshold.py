import cv2
import numpy as np


def search_contours(frame, mask):
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
        if area >= 10000:
            cv2.drawContours(frame, [approx], -1, (0, 255, 0), 2)
            cv2.putText(
                frame,
                f"{num_vertices} {area}",
                (x, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (255, 0, 0),
                2,
            )
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
            else:
                cX, cY = 0, 0
            cv2.circle(frame, (cX, cY), 7, (0, 0, 255), -1)


def detect_black_color(frame):
    # Convert the frame to the HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define a lower and upper threshold for the black color in HSV
    lower_black = np.array([0, 0, 0])
    upper_black = np.array([179, 255, 50])

    # Create a mask using the inRange function to threshold the image
    mask = cv2.inRange(hsv, lower_black, upper_black)

    # Apply the mask to the original frame
    result = cv2.bitwise_and(frame, frame, mask=mask)

    return mask


def crop_vertical_half(frame):
    # Get the width and height of the frame
    height, width, _ = frame.shape

    # Crop the left half of the frame
    cropped_frame = frame[height // 2 :, :, :]

    return cropped_frame


# Open a connection to the webcam (0 represents the default camera)
cap = cv2.VideoCapture("filename4.avi")

while True:
    # Read a frame from the webcam
    ret, frame = cap.read()

    # Call the function to detect black color
    cropped_frame = crop_vertical_half(frame)
    result = detect_black_color(cropped_frame)
    search_contours(cropped_frame, result)

    # Display the original frame and the result
    cv2.imshow("Original", cropped_frame)
    cv2.imshow("Black Color Detection", result)

    # Break the loop if the 'q' key is pressed
    if cv2.waitKey(30) & 0xFF == ord("q"):
        break

# Release the webcam and close all windows
cap.release()
cv2.destroyAllWindows()
