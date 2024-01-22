import rclpy
import numpy as np
import cv2
import os

from rclpy.node import Node
from std_msgs.msg import Int8, Bool, String
from geometry_msgs.msg import Twist
from rclpy import qos
from cv_bridge import CvBridge


class BotFlagHole(Node):
    def __init__(self):
        super().__init__("flag_grip_hole_node")
        self.sent_hole_gripper = self.create_publisher(
            Bool, "gripper/flag/hole", qos_profile=qos.qos_profile_system_default
        )
        self.sub_state = self.create_subscription(
            Int8,
            "state/main_ros",
            self.sub_state_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.sub_state
        self.sub_main = self.create_subscription(
            String,
            "state/main",
            self.sub_main_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.sub_main
        self.sub_step_motor = self.create_subscription(
            Twist,
            "gripper/flag/state",
            self.sub_step_motor_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.sub_step_motor

        self.sent_timer = self.create_timer(0.05, self.timer_callback)
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
        self.state_main = "Idle"

    def timer_callback(self):
        msg_hole = Bool()
        if self.mainros_state == 6 and self.current_step_motor3 >= int(
            (415000 / 0.455) * 0.45
        ):
            ref, self.frame = self.cap.read()
            if ref:
                self.frame = cv2.resize(self.frame, (760, 600))
                cropped_frame = crop_vertical_half(self.frame)
                result = detect_black_color(cropped_frame)
                self.search_contours(cropped_frame, result)
                cv2.line(cropped_frame, (340, 0), (340, 600), (255, 0, 0), 2)
                cv2.line(cropped_frame, (380, 0), (380, 600), (255, 0, 0), 2)
                if self.state_main == "Start":
                    if cX >= 340 and cX <= 380:
                        self.hole = True
                else:
                    self.hole = False
                msg_hole.data = self.hole
                self.sent_hole_gripper.publish(msg_hole)
                cv2.imshow("frame", cropped_frame)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                cv2.destroyAllWindows()
                exit()

    def sub_state_callback(self, msg_in):
        self.mainros_state = msg_in.data

    def sub_main_callback(self, msg_in):
        self.state_main = msg_in.data

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

            cv2.circle(frame, (int(center_x), int(center_y)), radius, (0, 255, 0), 2)
            cv2.circle(frame, (cX, cY), 7, (0, 0, 255), -1)
            cv2.drawContours(frame, [approx], -1, (0, 255, 0), 2)
            cv2.putText(
                frame,
                f"( {1} )",
                (x, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (255, 0, 0),
                2,
            )


def detect_black_color(frame):
    # Convert the frame to the HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_black = np.array([0, 0, 0])
    upper_black = np.array([179, 255, 30])
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
    cropped_frame = frame[height // 2 :, :, :]

    return cropped_frame


def main():
    rclpy.init()

    sub = BotFlagHole()
    rclpy.spin(sub)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
