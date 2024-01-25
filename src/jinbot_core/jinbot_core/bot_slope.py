import rclpy
import numpy as np

import math
from rclpy.node import Node
from std_msgs.msg import Bool, Int8

from tf_transformations import euler_from_quaternion
from sensor_msgs.msg import Imu
from rclpy import qos, Parameter


class BotSlope(Node):
    def __init__(self):
        super().__init__("bot_slope_node")
        self.sent_slope = self.create_publisher(
            Bool, "drive/slope", qos_profile=qos.qos_profile_system_default
        )
        self.sub_imu = self.create_subscription(
            Imu,
            "imu/data",
            self.sub_imu_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.sub_imu
        self.sent_slope_timer = self.create_timer(0.05, self.slope_callback)
        self.isSlope = False
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

    def slope_callback(self):
        msg = Bool()
        roll = -math.degrees(self.roll)
        if roll >= 8:  # 8
            self.isSlope = True
        else:
            self.isSlope = False
        msg.data = self.isSlope
        self.get_logger().info(
            f"{math.degrees(-self.roll)} {math.degrees(-self.pitch)} {math.degrees(-self.yaw)}"
        )
        self.sent_slope.publish(msg)

    def sub_imu_callback(self, msg_in):
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(
            [
                msg_in.orientation.x,
                msg_in.orientation.y,
                msg_in.orientation.z,
                msg_in.orientation.w,
            ]
        )


def main():
    rclpy.init()

    sub = BotSlope()
    rclpy.spin(sub)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
