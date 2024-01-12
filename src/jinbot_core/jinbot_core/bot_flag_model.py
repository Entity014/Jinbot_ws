import rclpy
import numpy as np
import torch
import cv2
import os

from ultralytics import YOLO
from supervision import Detections, BoxAnnotator
from rclpy.node import Node
from std_msgs.msg import Int8, Float32
from geometry_msgs.msg import Twist
from rclpy import qos, Parameter
from cv_bridge import CvBridge


class BotFlagModel(Node):
    def __init__(self):
        super().__init__("flag_grip_model_node")
        self.sent_tune_gripper = self.create_publisher(
            Float32, "gripper/flag/tune", qos_profile=qos.qos_profile_system_default
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

        self.cX = 0
        self.cY = 0

        self.current_step_motor1 = 0.0
        self.current_step_motor2 = 0.0
        self.current_step_motor3 = 0.0
        self.distance_step_motor1 = 0.0
        self.distance_step_motor2 = 0.0
        self.distance_step_motor3 = 0.0

    def timer_callback(self):
        msg_tuning = Float32()
        if self.mainros_state == 4:
            cap = cv2.VideoCapture("/dev/video2")
            ref, self.frame = cap.read()
            if ref:
                self.frame = cv2.resize(self.frame, (760, 600))
                self.frame = cv2.flip(self.frame, 0)
                self.frame = cv2.flip(self.frame, 1)
                hsv_frame = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
                # dil_frame_mask = dilate_frame(hsv_frame, 50, 0, 69, 43)
                dil_frame_mask = dilate_frame(hsv_frame)
                self.search_contours(dil_frame_mask)
                if self.cX > 400:
                    self.tuning += self.diff_tuning
                elif self.cX < 350:
                    self.tuning -= self.diff_tuning
                # self.tuning = np.interp(self.cX, [0, 760], [50, 130])
                msg_tuning.data = self.tuning
                self.sent_tune_gripper.publish(msg_tuning)
                cv2.imshow("frame", self.frame)

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

    def search_contours(self, mask):
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
                    self.cX = int(M["m10"] / M["m00"])
                    self.cY = int(M["m01"] / M["m00"])
                else:
                    self.cX, self.cY = 0, 0
                cv2.drawContours(self.frame, [approx], -1, (0, 255, 0), 2)
                cv2.circle(self.frame, (self.cX, self.cY), 7, (0, 0, 255), -1)
                cv2.circle(self.frame, (380, 300), 7, (255, 0, 0), -1)


def dilate_frame(frame, diff=0, sl=0, vl=0):
    lower_hsv = np.array([constrain(0 - diff, 0, 179), sl, vl])
    upper_hsv = np.array([179, 255, 120])
    mask = cv2.inRange(frame, lower_hsv, upper_hsv)
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


def constrain(value, min_val, max_val):
    return min(max_val, max(min_val, value))


def main():
    rclpy.init()

    sub = BotFlagModel()
    rclpy.spin(sub)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
