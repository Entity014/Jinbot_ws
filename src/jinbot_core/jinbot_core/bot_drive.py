import rclpy
import numpy as np

import time
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from rclpy import qos, Parameter


class BotDrive(Node):
    def __init__(self):
        super().__init__("bot_drive_node")
        self.sent_drive = self.create_publisher(
            Twist, "cmd_vel", qos_profile=qos.qos_profile_system_default
        )
        self.sub_slope = self.create_subscription(
            Bool,
            "drive/slope",
            self.sub_slope_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.sub_slope
        self.sent_drive_timer = self.create_timer(0.05, self.drive_callback)

        self.declare_parameters(
            "",
            [
                ("speed.x", Parameter.Type.DOUBLE),
                ("speed.y", Parameter.Type.DOUBLE),
                ("speed.z", Parameter.Type.DOUBLE),
                ("rotation.x", Parameter.Type.DOUBLE),
                ("rotation.y", Parameter.Type.DOUBLE),
                ("rotation.z", Parameter.Type.DOUBLE),
            ],
        )
        self.speed_x = 0.0
        self.speed_y = 0.0
        self.speed_z = 0.0
        self.rotation_x = 0.0
        self.rotation_y = 0.0
        self.rotation_z = 0.0
        self.state = 0
        self.pre_time = 0

        self.slope = False
        self.start_slope = False
        self.self_drive_state = 0

    def drive_callback(self):
        msg = Twist()
        # self.speed_x = self.get_parameter("speed.x").get_parameter_value().double_value
        # self.speed_y = self.get_parameter("speed.y").get_parameter_value().double_value
        # self.speed_z = self.get_parameter("speed.z").get_parameter_value().double_value
        # self.rotation_x = (
        #     self.get_parameter("rotation.x").get_parameter_value().double_value
        # )
        # self.rotation_y = (
        #     self.get_parameter("rotation.y").get_parameter_value().double_value
        # )
        # self.rotation_z = (
        #     self.get_parameter("rotation.z").get_parameter_value().double_value
        # )
        if self.self_drive_state == 0:
            msg.linear.x = self.speed_x
            self.sent_drive.publish(msg)
        elif self.self_drive_state == 1:
            msg.linear.x = 0.0
            self.sent_drive.publish(msg)
            self.self_drive_state = 2
        if not self.start_slope:
            self.speed_x = 0.5
        if self.slope:
            self.speed_x = 0.5
            self.start_slope = True
        elif not self.slope and self.start_slope:
            self.self_drive_state = 1
        self.get_logger().info(
            f"{self.self_drive_state} {self.start_slope} {self.slope}"
        )
        # msg.linear.x = self.speed_x
        msg.linear.y = self.speed_y
        msg.linear.z = self.speed_z
        msg.angular.x = self.rotation_x
        msg.angular.y = self.rotation_y
        msg.angular.z = self.rotation_z
        self.sent_drive.publish(msg)

    def sub_slope_callback(self, msg_in):
        self.slope = msg_in.data


def main():
    rclpy.init()

    sub = BotDrive()
    rclpy.spin(sub)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
