import rclpy
import numpy as np

from rclpy.node import Node
from rclpy import qos, Parameter
from std_msgs.msg import Int8, Bool, Float32
from geometry_msgs.msg import Point, Vector3, Twist


class BotFlag(Node):
    def __init__(self):
        super().__init__("flag_grip_node")
        self.sent_flag_pos = self.create_publisher(
            Point, "gripper/flag/pos", qos_profile=qos.qos_profile_system_default
        )
        self.sent_flag_theta = self.create_publisher(
            Vector3, "gripper/flag/hand", qos_profile=qos.qos_profile_system_default
        )
        self.sub_state = self.create_subscription(
            Int8,
            "state/main_ros",
            self.sub_state_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.sub_state
        self.sub_step_motor = self.create_subscription(
            Twist,
            "step_motor/state",
            self.sub_step_motor_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.sub_step_motor
        self.sub_flag_tuning = self.create_subscription(
            Float32,
            "gripper/flag/tune",
            self.sub_flag_tuning_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.sub_flag_tuning
        self.sub_flag_detect = self.create_subscription(
            Bool,
            "gripper/flag/detect",
            self.sub_flag_detect_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.sub_flag_detect

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

        self.mainros_state = 0
        self.node_state = 0
        self.current_step_motor1 = 0.0
        self.current_step_motor2 = 0.0
        self.current_step_motor3 = 0.0
        self.distance_step_motor1 = 0.0
        self.distance_step_motor2 = 0.0
        self.distance_step_motor3 = 0.0

        self.tuning = 0.0
        self.detect = False

    def timer_callback(self):
        msg_pos = Point()
        msg_theta = Vector3()
        # self.position_x = (
        #     self.get_parameter("position.x").get_parameter_value().double_value
        # )
        # self.position_y = (
        #     self.get_parameter("position.y").get_parameter_value().double_value
        # )
        # self.position_z = (
        #     self.get_parameter("position.z").get_parameter_value().double_value
        # )
        self.theta1 = self.get_parameter("theta.x").get_parameter_value().double_value
        self.theta2 = self.get_parameter("theta.y").get_parameter_value().double_value
        self.theta3 = self.get_parameter("theta.z").get_parameter_value().double_value

        msg_pos.x = self.position_x
        msg_pos.y = self.position_y
        msg_pos.z = self.position_z

        msg_theta.x = self.theta1
        msg_theta.y = self.theta2
        msg_theta.z = self.theta3

        self.sent_flag_pos.publish(msg_pos)
        self.sent_flag_theta.publish(msg_theta)

    def sub_state_callback(self, msg_in):
        self.mainros_state = msg_in.data
        self.get_logger().info(f"{self.node_state}")
        if self.mainros_state == 4 and self.node_state == 0:
            self.node_state = 1

        if self.node_state == 1 and self.current_step_motor3 == 5000.0:
            self.position_z = 0.4
            self.node_state = 2
        elif (
            self.node_state == 2
            and self.current_step_motor1 == 0.0
            and self.current_step_motor2 == 0.0
            and self.distance_step_motor3 == 1.0
            and self.detect
        ):
            self.position_x = 0.1 * self.tuning
            self.position_y = 0.43
            self.position_z = 0.08
            self.node_state = 3
        elif (
            self.node_state == 3
            and self.distance_step_motor1 == 1.0
            and self.distance_step_motor2 == 1.0
            and self.distance_step_motor3 == 1.0
        ):
            self.theta1 = 160.0
            self.position_z = 0.40
            self.node_state = 4
        elif self.node_state == 4 and self.distance_step_motor3 == 1.0:
            self.position_x = 0.01
            self.position_y = 0.11
            self.position_z = 0.05
            self.node_state = 5

    def sub_step_motor_callback(self, msg_in):
        self.current_step_motor1 = msg_in.linear.x
        self.current_step_motor2 = msg_in.linear.y
        self.current_step_motor3 = msg_in.linear.z
        self.distance_step_motor1 = msg_in.angular.x
        self.distance_step_motor2 = msg_in.angular.y
        self.distance_step_motor3 = msg_in.angular.z

    def sub_flag_tuning_callback(self, msg_in):
        self.tuning = msg_in.data

    def sub_flag_detect_callback(self, msg_in):
        self.detect = msg_in.data


def main():
    rclpy.init()

    sub = BotFlag()
    rclpy.spin(sub)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
