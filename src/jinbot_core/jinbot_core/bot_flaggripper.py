import rclpy
import numpy as np
from time import time

from rclpy.node import Node
from rclpy import qos, Parameter
from geometry_msgs.msg import Point, Vector3


class BotFlag(Node):
    def __init__(self):
        super().__init__("flag_grip_node")
        self.sent_flag_pos = self.create_publisher(
            Point, "gripper/flag/pos", qos_profile=qos.qos_profile_system_default
        )
        self.sent_flag_theta = self.create_publisher(
            Vector3, "gripper/flag/hand", qos_profile=qos.qos_profile_system_default
        )

        self.sent_timer = self.create_timer(0.05, self.timer_callback)
        self.declare_parameters(
            "",
            [
                ("position.x", Parameter.Type.DOUBLE),
                ("position.y", Parameter.Type.DOUBLE),
                ("position.z", Parameter.Type.DOUBLE),
                ("theta.x", Parameter.Type.DOUBLE),
                ("theta.y", Parameter.Type.DOUBLE),
                ("theta.z", Parameter.Type.DOUBLE),
            ],
        )
        self.position_x = 0.0
        self.position_y = 0.0
        self.position_z = 0.0
        self.theta1 = 0.0
        self.theta2 = 0.0
        self.theta3 = 0.0

    def timer_callback(self):
        msg_pos = Point()
        msg_theta = Vector3()
        self.position_x = (
            self.get_parameter("position.x").get_parameter_value().double_value
        )
        self.position_y = (
            self.get_parameter("position.y").get_parameter_value().double_value
        )
        self.position_z = (
            self.get_parameter("position.z").get_parameter_value().double_value
        )
        self.theta1 = self.get_parameter("theta.x").get_parameter_value().double_value
        self.theta2 = self.get_parameter("theta.y").get_parameter_value().double_value
        self.theta3 = self.get_parameter("theta.z").get_parameter_value().double_value

        msg_pos.x = self.position_x
        msg_pos.y = self.position_y
        msg_pos.z = self.position_z

        msg_theta.x = self.theta1
        msg_theta.y = self.theta2
        msg_theta.z = self.theta3

        # self.get_logger().info(f"{self.speed1}")

        self.sent_flag_pos.publish(msg_pos)
        self.sent_flag_theta.publish(msg_theta)


def main():
    rclpy.init()

    sub = BotFlag()
    rclpy.spin(sub)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
