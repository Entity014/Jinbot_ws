import rclpy
import numpy as np
import torch
import cv2
import os

from ultralytics import YOLO
from supervision import Detections, BoxAnnotator
from rclpy.node import Node
from std_msgs.msg import Int8, Float32, Bool
from geometry_msgs.msg import Twist
from rclpy import qos
from cv_bridge import CvBridge


class BotFlagModel(Node):
    def __init__(self):
        super().__init__("flag_grip_hole_node")
        self.sent_tune_gripper = self.create_publisher(
            Float32, "gripper/flag/hole", qos_profile=qos.qos_profile_system_default
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
        self.frame = np.zeros((480, 640, 3), dtype=np.uint8)
        self.mainros_state = 0

        self.current_step_motor1 = 0.0
        self.current_step_motor2 = 0.0
        self.current_step_motor3 = 0.0
        self.distance_step_motor1 = 0.0
        self.distance_step_motor2 = 0.0
        self.distance_step_motor3 = 0.0

    def timer_callback(self):
        pass

    def sub_state_callback(self, msg_in):
        self.mainros_state = msg_in.data

    def sub_step_motor_callback(self, msg_in):
        self.current_step_motor1 = msg_in.linear.x
        self.current_step_motor2 = msg_in.linear.y
        self.current_step_motor3 = msg_in.linear.z
        self.distance_step_motor1 = msg_in.angular.x
        self.distance_step_motor2 = msg_in.angular.y
        self.distance_step_motor3 = msg_in.angular.z


def main():
    rclpy.init()

    sub = BotFlagModel()
    rclpy.spin(sub)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
