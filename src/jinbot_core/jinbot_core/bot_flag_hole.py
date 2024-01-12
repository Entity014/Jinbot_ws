import rclpy
import numpy as np
import cv2
import os

from rclpy.node import Node
from std_msgs.msg import Int8, Float32, Bool
from geometry_msgs.msg import Twist
from rclpy import qos
from cv_bridge import CvBridge


class BotFlagHole(Node):
    def __init__(self):
        super().__init__("flag_grip_hole_node")
        self.sent_hole_gripper = self.create_publisher(
            Bool, "gripper/flag/hole", qos_profile=qos.qos_profile_system_default
        )
        self.sent_timer = self.create_timer(0.05, self.timer_callback)
        self.sub_state = self.create_subscription(
            Int8,
            "state/main_ros",
            self.sub_state_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.sub_state
        self.sub_step_motor = self.create_subscription(
            Twist,
            "gripper/flag/state",
            self.sub_step_motor_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.sub_step_motor

        self.cap = cv2.VideoCapture("/dev/video2")
        self.frame = np.zeros((760, 600, 3), dtype=np.uint8)
        self.mainros_state = 0

        self.current_step_motor1 = 0.0
        self.current_step_motor2 = 0.0
        self.current_step_motor3 = 0.0
        self.distance_step_motor1 = 0.0
        self.distance_step_motor2 = 0.0
        self.distance_step_motor3 = 0.0

        self.hole = False

    def timer_callback(self):
        msg_tuning = Float32()
        if self.mainros_state == 6 and self.current_step_motor1 >= int(
            (415000 / 0.455) * 0.45
        ):
            cap = cv2.VideoCapture("/dev/video2")
            ref, self.frame = cap.read()
            if ref:
                self.frame = cv2.resize(self.frame, (760, 600))
                self.frame = cv2.flip(self.frame, 0)
                self.frame = cv2.flip(self.frame, 1)
                cropped_frame = crop_vertical_half(self.frame)
                result = detect_black_color(cropped_frame)
                self.search_contours(cropped_frame, result)
                cv2.line(cropped_frame, (340, 0), (340, 600), (255, 0, 0), 2)
                cv2.line(cropped_frame, (380, 0), (380, 600), (255, 0, 0), 2)
                if cX >= 340 and cX <= 380:
                    self.hole = True
                msg_tuning.data = self.hole
                self.sent_hole_gripper.publish(msg_tuning)
                cv2.imshow("frame", cropped_frame)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                cv2.destroyAllWindows()
                exit()

    def sub_state_callback(self, msg_in):
        self.mainros_state = msg_in.data

    def sub_step_motor_callback(self, msg_in):
        self.current_step_motor1 = msg_in.linear.x
        self.current_step_motor2 = msg_in.linear.y
        self.current_step_motor3 = msg_in.linear.z
        self.distance_step_motor1 = msg_in.angular.x
        self.distance_step_motor2 = msg_in.angular.y
        self.distance_step_motor3 = msg_in.angular.z


def search_contours(frame, mask):
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
        # if area <= 5000 and area >= 3000 and num_vertices <= 17:
        if area >= 30000:
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
    # canny_frame = cv2.Canny(mask, 0, 255)
    # kernel = np.ones((5, 5))
    # cv2.imshow("color_search", canny_frame)

    return mask


def crop_vertical_half(frame):
    # Get the width and height of the frame
    height, width, _ = frame.shape

    # Crop the left half of the frame
    cropped_frame = frame[height // 2 :, :, :]

    return cropped_frame


def main():
    rclpy.init()

    sub = BotFlagHole()
    rclpy.spin(sub)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
