import rclpy
import numpy as np
import os
import math

from rclpy.node import Node
from rclpy import qos, Parameter
from rclpy.duration import Duration
from std_msgs.msg import String, Int8, Bool
from geometry_msgs.msg import PoseStamped, Twist
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from ament_index_python.packages import get_package_share_directory
from tf_transformations import quaternion_from_euler


class BotState(Node):
    def __init__(self):
        super().__init__("bot_state_node")
        self.sent_state_main_ros = self.create_publisher(
            Int8, "state/main_ros", qos_profile=qos.qos_profile_system_default
        )
        self.sent_state_main = self.create_publisher(
            String, "state/main", qos_profile=qos.qos_profile_system_default
        )
        self.sent_state_team = self.create_publisher(
            String, "state/team", qos_profile=qos.qos_profile_system_default
        )
        self.sent_state_retry = self.create_publisher(
            String, "state/retry", qos_profile=qos.qos_profile_system_default
        )
        self.sent_timer = self.create_timer(0.05, self.timer_callback)

        self.sub_start_button = self.create_subscription(
            Int8,
            "button/start",
            self.sub_start_button_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.sub_start_button
        self.sub_team_button = self.create_subscription(
            Int8,
            "button/team",
            self.sub_team_button_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.sub_team_button
        self.sub_retry_button = self.create_subscription(
            Int8,
            "button/retry",
            self.sub_retry_button_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.sub_retry_button
        self.sub_flag_hold = self.create_subscription(
            Bool,
            "gripper/flag/hold",
            self.sub_flag_hold_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.sub_flag_hold
        self.sub_goal = self.create_subscription(
            Bool,
            "drive/goal",
            self.sub_goal_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.sub_goal

        self.declare_parameters(
            "",
            [
                ("main_ros", Parameter.Type.INTEGER),
                ("main", Parameter.Type.STRING),
                ("team", Parameter.Type.STRING),
                ("retry", Parameter.Type.STRING),
            ],
        )

        self.state_main_ros = 1
        self.state_main = "Idle"
        self.state_team = "Blue"
        self.state_retry = "None"

        self.main_order = 0
        self.team_order = 0
        self.retry_order = 0
        self.pre_start_button = 0
        self.pre_team_button = 0
        self.pre_retry_button = 0
        self.flag_hold = False
        self.goal = False
        self.pre_goal = False

    def timer_callback(self):
        msg_state_main_ros = Int8()
        msg_state_main = String()
        msg_state_team = String()
        msg_state_retry = String()

        if self.state_main == "Idle":
            self.state_main_ros = 1
        elif self.state_main == "Reset":
            if self.retry_order == "First":
                self.state_main_ros = 1
            elif self.retry_order == "Second":
                self.state_main_ros = 7
            else:
                self.state_main_ros = 1
        elif self.state_main == "Start":
            if self.state_main_ros == 1 and self.goal:
                self.state_main_ros = 2
                self.goal = False
            elif self.state_main_ros == 2 and self.goal:
                self.state_main_ros = 3
                self.goal = False
            elif self.state_main_ros == 3 and self.goal:
                self.state_main_ros = 4
                self.goal = False
            elif self.state_main_ros == 4 and self.goal:
                self.state_main_ros = 5
                self.goal = False
            elif self.state_main_ros == 5 and self.goal:
                self.state_main_ros = 6
                self.goal = False
            elif self.state_main_ros == 7 and self.goal:
                self.state_main_ros = 8
                self.goal = False
            elif self.state_main_ros == 8 and self.goal:
                self.state_main_ros = 9
                self.goal = False
        # self.state_main_ros = (
        #     self.get_parameter("main_ros").get_parameter_value().integer_value
        # )
        self.state_retry = (
            self.get_parameter("retry").get_parameter_value().string_value
        )
        self.state_main = self.get_parameter("main").get_parameter_value().string_value
        self.state_team = self.get_parameter("team").get_parameter_value().string_value

        msg_state_main_ros.data = self.state_main_ros
        msg_state_main.data = self.state_main
        msg_state_team.data = self.state_team
        msg_state_retry.data = self.state_retry

        self.sent_state_main_ros.publish(msg_state_main_ros)
        self.sent_state_main.publish(msg_state_main)
        self.sent_state_team.publish(msg_state_team)
        self.sent_state_retry.publish(msg_state_retry)

    def sub_start_button_callback(self, msg):
        if self.pre_start_button != msg.data:
            if self.pre_start_button == 1:
                self.main_order += 1
            self.pre_start_button = msg.data

        if self.main_order == 0:
            self.state_main = "Idle"
        elif self.main_order == 1:
            self.state_main = "Start"
        elif self.main_order == 2:
            self.state_main = "Reset"
        else:
            self.main_order = 0

    def sub_team_button_callback(self, msg):
        if self.pre_team_button != msg.data:
            if self.pre_team_button == 1:
                self.team_order += 1
            self.pre_team_button = msg.data

        if self.team_order == 0:
            self.state_team = "Blue"
        elif self.team_order == 1:
            self.state_team = "Red"
        else:
            self.team_order = 0

    def sub_retry_button_callback(self, msg):
        if self.pre_retry_button != msg.data:
            if self.pre_retry_button == 1:
                self.retry_order += 1
            self.pre_retry_button = msg.data

        if self.retry_order == 0:
            self.state_retry = "None"
        elif self.retry_order == 1:
            self.state_retry = "First"
        elif self.retry_order == 2:
            self.state_retry = "Second"
        else:
            self.retry_order = 0

    def sub_flag_hold_callback(self, msg_in):
        self.flag_hold = msg_in.data

    def sub_goal_callback(self, msg_in):
        self.goal = msg_in.data


def main():
    rclpy.init()

    sub = BotState()
    rclpy.spin(sub)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
