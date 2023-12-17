import rclpy
import numpy as np

from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy import qos, Parameter


class BotState(Node):
    def __init__(self):
        super().__init__("bot_state_node")
        pass

    def state_callback(self):
        pass


def main():
    rclpy.init()

    sub = BotState()
    rclpy.spin(sub)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
