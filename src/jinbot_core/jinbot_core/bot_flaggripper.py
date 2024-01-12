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
        self.sent_flag_hold = self.create_publisher(
            Bool, "gripper/flag/hold", qos_profile=qos.qos_profile_system_default
        )
        self.sub_step_motor = self.create_subscription(
            Twist,
            "gripper/flag/state",
            self.sub_step_motor_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.sub_step_motor
        self.sub_state = self.create_subscription(
            Int8,
            "state/main_ros",
            self.sub_state_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.sub_state
        self.sub_flag_tuning = self.create_subscription(
            Float32,
            "gripper/flag/tune",
            self.sub_flag_tuning_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.sub_flag_tuning

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
        self.pre_node_state = 0
        self.current_step_motor1 = 0.0
        self.current_step_motor2 = 0.0
        self.current_step_motor3 = 0.0
        self.distance_step_motor1 = 0.0
        self.distance_step_motor2 = 0.0
        self.distance_step_motor3 = 0.0

        self.start_step_motor1 = False
        self.start_step_motor2 = False
        self.start_step_motor3 = False
        self.ready_step_motor1 = False
        self.ready_step_motor2 = False
        self.ready_step_motor3 = False

        self.tuning = 90.0
        self.hold = False

    def timer_callback(self):
        msg_pos = Point()
        msg_theta = Vector3()
        msg_hold = Bool()
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
        # self.theta3 = self.get_parameter("theta.z").get_parameter_value().double_value

        msg_pos.x = self.position_x
        msg_pos.y = self.position_y
        msg_pos.z = self.position_z

        msg_theta.x = self.theta1
        msg_theta.y = self.theta2
        msg_theta.z = self.theta3

        msg_hold.data = self.hold

        self.sent_flag_pos.publish(msg_pos)
        self.sent_flag_theta.publish(msg_theta)
        self.sent_flag_hold.publish(msg_hold)

    def sub_state_callback(self, msg_in):
        self.mainros_state = msg_in.data
        # self.get_logger().info(f"{self.node_state} {self.tuning}")
        if self.pre_node_state != self.node_state:
            if self.pre_node_state != 0:
                self.ready_step_motor1 = False
                self.ready_step_motor2 = False
                self.ready_step_motor3 = False
            self.pre_node_state = self.node_state

        if self.mainros_state == 4:
            if self.node_state <= 3:
                self.theta2 = self.tuning

            if self.node_state == 0:
                self.node_state = 1
            elif self.node_state == 1 and self.ready_step_motor3:
                self.position_z = 0.10
                self.node_state = 2
            elif self.node_state == 2 and self.ready_step_motor3:
                self.position_y = 0.4
                self.node_state = 3
            elif (
                self.node_state == 3
                and self.ready_step_motor1
                and self.ready_step_motor2
            ):
                self.theta1 = 150.0
                self.position_z = 0.25
                self.node_state = 4
            elif self.node_state == 4 and self.ready_step_motor3:
                self.position_y = 999.0
                self.node_state = 5
            elif (
                self.node_state == 5
                and self.ready_step_motor1
                and self.ready_step_motor2
            ):
                self.theta2 = 180.0
                self.position_z = 0.2
                self.node_state = 5

            # elif (
            #     self.node_state == 2
            #     and self.ready_step_motor1
            #     and self.ready_step_motor2
            #     and self.ready_step_motor3
            # ):
            #     self.theta1 = 140.0
            #     self.position_z = 0.40
            #     self.node_state = 3
        #     elif self.node_state == 3 and self.ready_step_motor3:
        #         self.position_x = 0.001
        #         self.position_y = 0.06
        #         self.position_z = 0.05
        #         self.node_state = 4
        #     elif (
        #         self.node_state == 4
        #         and self.ready_step_motor1
        #         and self.ready_step_motor2
        #         and self.ready_step_motor3
        #     ):
        #         self.hold = True

    def sub_step_motor_callback(self, msg_in):
        self.current_step_motor1 = msg_in.linear.x
        self.current_step_motor2 = msg_in.linear.y
        self.current_step_motor3 = msg_in.linear.z
        self.distance_step_motor1 = msg_in.angular.x
        self.distance_step_motor2 = msg_in.angular.y
        self.distance_step_motor3 = msg_in.angular.z

        if abs(self.distance_step_motor1) > 0.0 and not self.ready_step_motor1:
            self.start_step_motor1 = True
            self.ready_step_motor1 = False
        elif self.distance_step_motor1 == 0.0 and self.start_step_motor1:
            self.start_step_motor1 = False
            self.ready_step_motor1 = True

        if abs(self.distance_step_motor2) > 0.0 and not self.ready_step_motor2:
            self.start_step_motor2 = True
            self.ready_step_motor2 = False
        elif self.distance_step_motor2 == 0.0 and self.start_step_motor2:
            self.start_step_motor2 = False
            self.ready_step_motor2 = True

        if abs(self.distance_step_motor3) > 0.0 and not self.ready_step_motor3:
            self.start_step_motor3 = True
            self.ready_step_motor3 = False
        elif self.distance_step_motor3 == 0.0 and self.start_step_motor3:
            self.start_step_motor3 = False
            self.ready_step_motor3 = True

    def sub_flag_tuning_callback(self, msg_in):
        self.tuning = msg_in.data


def main():
    rclpy.init()

    sub = BotFlag()
    rclpy.spin(sub)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
