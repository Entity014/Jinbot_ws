import rclpy
import numpy as np

from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from rclpy import qos, Parameter


class BotFlag(Node):
    def __init__(self):
        super().__init__("flag_grip_node")
        self.sent_flag = self.create_publisher(
            Pose2D, "gripper/flag", qos_profile=qos.qos_profile_system_default
        )
        self.sent_flag_timer = self.create_timer(0.05, self.flag_callback)

        self.declare_parameters(
            "",
            [
                ("position.x", Parameter.Type.DOUBLE),
                ("position.y", Parameter.Type.DOUBLE),
                ("theta", Parameter.Type.DOUBLE),
            ],
        )
        self.position_x = 0.0
        self.position_y = 0.0
        self.theta = 0.0

    def flag_callback(self):
        self.position_x = (
            self.get_parameter("position.x").get_parameter_value().double_value
        )
        self.position_y = (
            self.get_parameter("position.y").get_parameter_value().double_value
        )
        self.theta = self.get_parameter("theta").get_parameter_value().double_value
        msg = Pose2D()
        msg.x = self.position_x
        msg.y = self.position_y
        msg.theta = self.theta

        # self.get_logger().info(f"{self.speed_x}")

        self.sent_flag.publish(msg)


def main():
    rclpy.init()

    sub = BotFlag()
    rclpy.spin(sub)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
