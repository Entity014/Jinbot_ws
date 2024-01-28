import rclpy
import numpy as np
import torch
import cv2
import os

from ultralytics import YOLO
from supervision import Detections, BoxAnnotator
from rclpy.node import Node
from std_msgs.msg import Int8, Float32, String, Bool
from geometry_msgs.msg import Twist
from rclpy import qos, Parameter
from cv_bridge import CvBridge


class BotFlagModel(Node):
    def __init__(self):
        super().__init__("flag_grip_model_node")
        self.sent_tune_gripper = self.create_publisher(
            Float32, "gripper/flag/tune", qos_profile=qos.qos_profile_system_default
        )
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
        self.sub_goal = self.create_subscription(
            Bool,
            "drive/goal",
            self.sub_goal_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.sub_goal
        self.sent_timer = self.create_timer(0.05, self.timer_callback)

        self.declare_parameters(
            "",
            [
                ("max_range", Parameter.Type.DOUBLE),
                ("min_range", Parameter.Type.DOUBLE),
            ],
        )
        self.tuning = 90.0
        self.diff_tuning = 8.0
        self.frame = np.zeros((760, 600, 3), dtype=np.uint8)
        self.mainros_state = 0

        self.max_range = (
            self.get_parameter("max_range").get_parameter_value().double_value
        )
        self.min_range = (
            self.get_parameter("min_range").get_parameter_value().double_value
        )

        self.cX1 = 0
        self.cY1 = 0
        self.cX2 = 0
        self.cY2 = 0
        self.state_main = "Idle"
        self.hole = False

        self.current_step_motor1 = 0.0
        self.current_step_motor2 = 0.0
        self.current_step_motor3 = 0.0
        self.distance_step_motor1 = 0.0
        self.distance_step_motor2 = 0.0
        self.distance_step_motor3 = 0.0

        self.goal = False
        self.cap = cv2.VideoCapture("/dev/video2")

    def timer_callback(self):
        msg_tuning = Float32()
        msg_hole = Bool()

        ref, self.frame = self.cap.read()
        if ref:
            self.frame = cv2.resize(self.frame, (760, 600))
            self.frame = crop_vertical_half(self.frame)
            cropped_frame = self.frame.copy()
            if self.mainros_state == 4 and self.goal:
                gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
                blurred = cv2.GaussianBlur(gray, (7, 7), 1)
                _, threshold = cv2.threshold(blurred, 40, 255, cv2.THRESH_BINARY_INV)
                self.search_contours_1(threshold)
                if self.state_main == "Start":
                    if self.cX1 > 400:
                        self.tuning += self.diff_tuning
                    elif self.cX1 < 350:
                        self.tuning -= self.diff_tuning
                msg_tuning.data = self.tuning
                self.sent_tune_gripper.publish(msg_tuning)
                # cv2.imshow("frame", self.frame)
                # cv2.imshow("frame2", threshold)
            elif self.mainros_state == 6 and self.current_step_motor3 >= int(
                (415000 / 0.455) * 0.45
            ):
                result = detect_black_color(cropped_frame)
                self.search_contours_2(cropped_frame, result)
                cv2.line(cropped_frame, (340, 0), (340, 600), (255, 0, 0), 2)
                cv2.line(cropped_frame, (380, 0), (380, 600), (255, 0, 0), 2)
                if self.state_main == "Start":
                    if self.cX2 >= 340 and self.cX2 <= 380:
                        self.hole = True
                msg_hole.data = self.hole
                self.sent_hole_gripper.publish(msg_hole)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                cv2.destroyAllWindows()
                exit()
        if self.state_main == "Reset":
            self.hole = False
            self.tuning = 90.0
            self.goal = False

    def sub_state_callback(self, msg_in):
        self.mainros_state = msg_in.data

    def sub_main_callback(self, msg_in):
        self.state_main = msg_in.data

    def sub_goal_callback(self, msg_in):
        self.goal = msg_in.data

    def sub_step_motor_callback(self, msg_in):
        self.current_step_motor1 = msg_in.linear.x
        self.current_step_motor2 = msg_in.linear.y
        self.current_step_motor3 = msg_in.linear.z
        self.distance_step_motor1 = msg_in.angular.x
        self.distance_step_motor2 = msg_in.angular.y
        self.distance_step_motor3 = msg_in.angular.z

    def search_contours_1(self, mask):
        contours, hierarchy = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        for contour in contours:
            area = cv2.contourArea(contour)
            x, y, w, h = cv2.boundingRect(contour)
            epsilon = 0.01 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            num_vertices = len(approx)
            if area > 10000:
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    self.cX1 = int(M["m10"] / M["m00"])
                    self.cY1 = int(M["m01"] / M["m00"])
                else:
                    self.cX1, self.cY1 = 0, 0
                cv2.drawContours(self.frame, [approx], -1, (0, 255, 0), 2)
                cv2.circle(self.frame, (self.cX1, self.cY1), 7, (0, 0, 255), -1)
                cv2.circle(self.frame, (380, 300), 7, (255, 0, 0), -1)

    def search_contours_2(self, frame, mask):
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
                    self.cX2 = int(M["m10"] / M["m00"])
                    self.cY2 = int(M["m01"] / M["m00"])
                else:
                    self.cX2, self.cY2 = 0, 0

                cv2.circle(
                    frame, (int(center_x), int(center_y)), radius, (0, 255, 0), 2
                )
                cv2.circle(frame, (self.cX2, self.cY2), 7, (0, 0, 255), -1)
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


def constrain(value, min_val, max_val):
    return min(max_val, max(min_val, value))


def crop_vertical_half(frame):
    # Get the width and height of the frame
    height, width, _ = frame.shape

    # Crop the left half of the frame
    cropped_frame = frame[: height // 2, :, :]

    return cropped_frame


def main():
    rclpy.init()

    sub = BotFlagModel()
    rclpy.spin(sub)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
