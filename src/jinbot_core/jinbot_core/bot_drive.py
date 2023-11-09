import rclpy
import numpy as np

from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy import qos, Parameter


class BotDrive(Node):
    def __init__(self):
        super().__init__("bot_drive_node")
        self.sent_drive = self.create_publisher(
            Twist, "cmd_vel", qos_profile=qos.qos_profile_system_default
        )
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

    def drive_callback(self):
        self.speed_x = self.get_parameter("speed.x").get_parameter_value().double_value
        self.speed_y = self.get_parameter("speed.y").get_parameter_value().double_value
        self.speed_z = self.get_parameter("speed.z").get_parameter_value().double_value
        self.rotation_x = (
            self.get_parameter("rotation.x").get_parameter_value().double_value
        )
        self.rotation_y = (
            self.get_parameter("rotation.y").get_parameter_value().double_value
        )
        self.rotation_z = (
            self.get_parameter("rotation.z").get_parameter_value().double_value
        )
        msg = Twist()

        msg.linear.x = self.speed_x
        msg.linear.y = self.speed_y
        msg.linear.z = self.speed_z
        msg.angular.x = self.rotation_x
        msg.angular.y = self.rotation_y
        msg.angular.z = self.rotation_z

        # self.get_logger().info(f"{self.speed_x}")

        self.sent_drive.publish(msg)


def main():
    rclpy.init()

    sub = BotDrive()
    rclpy.spin(sub)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
