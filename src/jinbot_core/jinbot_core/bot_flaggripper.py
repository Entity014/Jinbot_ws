import rclpy
import numpy as np

from rclpy.node import Node
from rclpy import qos, Parameter
from geometry_msgs.msg import Pose2D, Vector3

from jinbot_interfaces.srv import Pose3step


class BotFlag(Node):
    def __init__(self):
        super().__init__("flag_grip_node")
        self.sent_flag_pos = self.create_publisher(
            Pose2D, "gripper/flag/pos", qos_profile=qos.qos_profile_system_default
        )
        self.sent_flag_speed = self.create_publisher(
            Vector3, "gripper/flag/speed", qos_profile=qos.qos_profile_system_default
        )

        # self.server = self.create_service(Pose3step, "flag/pos", self.server_callback)

        self.sent_timer = self.create_timer(0.05, self.timer_callback)
        self.declare_parameters(
            "",
            [
                ("position.x", Parameter.Type.DOUBLE),
                ("position.y", Parameter.Type.DOUBLE),
                ("theta", Parameter.Type.DOUBLE),
                ("speed.x", Parameter.Type.DOUBLE),
                ("speed.y", Parameter.Type.DOUBLE),
                ("speed.z", Parameter.Type.DOUBLE),
            ],
        )
        self.position_x = 0.0
        self.position_y = 0.0
        self.theta = 0.0
        self.speed1 = 0.0
        self.speed2 = 0.0
        self.speed3 = 0.0

    def timer_callback(self):
        msg_pos = Pose2D()
        msg_speed = Vector3()
        self.position_x = (
            self.get_parameter("position.x").get_parameter_value().double_value
        )
        self.position_y = (
            self.get_parameter("position.y").get_parameter_value().double_value
        )
        self.theta = self.get_parameter("theta").get_parameter_value().double_value
        self.speed1 = self.get_parameter("speed.x").get_parameter_value().double_value
        self.speed2 = self.get_parameter("speed.y").get_parameter_value().double_value
        self.speed3 = self.get_parameter("speed.z").get_parameter_value().double_value

        msg_pos.x = self.position_x
        msg_pos.y = self.position_y
        msg_pos.theta = self.theta

        msg_speed.x = self.speed1
        msg_speed.y = self.speed2
        msg_speed.z = self.speed3

        # self.get_logger().info(f"{self.speed1}")

        self.sent_flag_pos.publish(msg_pos)
        self.sent_flag_speed.publish(msg_speed)

    # def server_callback(self, request, response):
    #     self.position_x = (
    #         self.get_parameter("position.x").get_parameter_value().double_value
    #     )
    #     self.position_y = (
    #         self.get_parameter("position.y").get_parameter_value().double_value
    #     )
    #     self.theta = self.get_parameter("theta").get_parameter_value().double_value
    #     if request:
    #         response.step1 = self.position_x
    #         response.step2 = self.position_y
    #         response.step3 = self.theta
    #     else:
    #         response.step1 = 0.0
    #         response.step2 = 0.0
    #         response.step3 = 0.0
    #     return response


def main():
    rclpy.init()

    sub = BotFlag()
    rclpy.spin(sub)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
