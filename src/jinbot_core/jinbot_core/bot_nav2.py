import rclpy
import numpy as np
import os
import math
import time

from rclpy.node import Node
from rclpy import qos, Parameter
from rclpy.duration import Duration
from std_msgs.msg import String, Int8, Bool
from geometry_msgs.msg import PoseStamped, Twist
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from ament_index_python.packages import get_package_share_directory
from tf_transformations import quaternion_from_euler


class BotNav2(Node):
    def __init__(self):
        super().__init__("bot_nav2_node")
        self.sent_velocity = self.create_publisher(
            Twist, "cmd_vel", qos_profile=qos.qos_profile_system_default
        )
        self.sent_goal = self.create_publisher(
            Bool, "drive/goal", qos_profile=qos.qos_profile_system_default
        )

        self.sub_slope = self.create_subscription(
            Bool,
            "drive/slope",
            self.sub_slope_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.sub_slope
        self.sub_state = self.create_subscription(
            Int8,
            "state/main_ros",
            self.sub_state_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.sub_state
        self.sub_main = self.create_subscription(
            String,
            "state/main",
            self.sub_main_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.sub_main
        self.sub_team = self.create_subscription(
            String,
            "state/team",
            self.sub_team_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.sub_team
        self.sub_retry = self.create_subscription(
            String,
            "state/retry",
            self.sub_retry_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.sub_retry
        self.sent_timer = self.create_timer(0.05, self.timer_callback)

        self.state_main_ros = 1
        self.pre_state_main_ros = 0
        self.state_main = "Idle"
        self.state_team = "Blue"
        self.state_retry = "None"

        self.main_order = 0
        self.team_order = 0
        self.retry_order = 0
        self.pre_start_button = 0
        self.pre_team_button = 0
        self.pre_retry_button = 0

        self.path_blue_map = os.path.join(
            get_package_share_directory("jinbot_core"), "maps", "jinpao_blue2.yaml"
        )
        self.path_green_map = os.path.join(
            get_package_share_directory("jinbot_core"), "maps", "jinpao_green2.yaml"
        )
        self.slope = False
        self.start_slope = False
        self.self_drive_state = 0
        self.speed_x = 0.0
        self.navigator = BasicNavigator()
        self.goal = False
        self.once = True

    def timer_callback(self):
        msg_velocity = Twist()
        # self.get_logger().info(
        #     f"{self.state_main} {self.state_main_ros} {self.self_drive_state} {self.pre_state_main_ros}"
        # )
        if self.state_main == "Start":
            if self.state_team == "Blue":  # ? BLUE
                if self.pre_state_main_ros != self.state_main_ros:
                    if self.state_main_ros == 1:
                        self.navigator.changeMap(self.path_blue_map)
                        self.setip(-0.31, 0.44, 185)
                        time.sleep(2.0)
                        self.goto(-1.95, 0.7, 185)
                    elif self.state_main_ros == 2:
                        self.goto(-2.15, 3.65, 185)
                    elif self.state_main_ros == 3:
                        self.goto(-1.85, 1.4, -25)
                    elif self.state_main_ros == 4:
                        self.goto(-0.32, 1.0, 102)
                    elif self.state_main_ros == 5:
                        if self.self_drive_state == 2:
                            self.setip(-0.8, 5.4, 100)
                            self.waypoint([[-4.0, 5.5, -10], [-3.5, 1.0, -10]])
                    elif self.state_main_ros == 7:
                        self.setip(-3.3, 2.5, 100)
                        self.goto(-4.3, 5.8, 100)
                    elif self.state_main_ros == 8:
                        self.waypoint([-3.6, 5.5, -10], [-3.5, 1.0, -10])
                    self.pre_state_main_ros = self.state_main_ros

                if self.state_main_ros != 5:
                    self.check_goal()
                else:
                    if self.self_drive_state == 0:
                        msg_velocity.linear.x = self.speed_x
                        self.sent_velocity.publish(msg_velocity)
                    elif self.self_drive_state == 1:
                        msg_velocity.linear.x = 0.0
                        self.sent_velocity.publish(msg_velocity)
                        self.self_drive_state = 2
                        self.pre_state_main_ros = 0
                    if not self.start_slope:
                        self.speed_x = 0.5
                    if self.slope:
                        self.speed_x = 0.5
                        self.start_slope = True
                    elif not self.slope and self.start_slope:
                        self.self_drive_state = 1
                if self.self_drive_state == 2:
                    self.check_goal()

            else:  # ! RED
                if self.pre_state_main_ros != self.state_main_ros:
                    if self.state_main_ros == 1:
                        self.navigator.changeMap(self.path_green_map)
                        self.setip(-5.1, 3.0, -10)
                        time.sleep(2.0)
                        self.goto(-3.5, 3.15, -18)
                    elif self.state_main_ros == 2:
                        self.goto(-3.05, 6.05, -18)
                    elif self.state_main_ros == 3:
                        self.goto(-3.6, 3.8, -170)
                    elif self.state_main_ros == 4:
                        self.goto(-5.2, 3.7, 70)
                    elif self.state_main_ros == 5:
                        if self.self_drive_state == 2:
                            self.setip(-4.2, 7.7, 80)
                            self.waypoint([[-0.9, 7.8, 180], [-2.0, 3.3, 180]])
                    elif self.state_main_ros == 7:
                        self.setip(-1.9, 4.7, 80)
                        self.goto(-0.5, 8.0, 80)
                    elif self.state_main_ros == 8:
                        self.waypoint([[-0.9, 7.8, 180], [-2.0, 3.3, 180]])
                    self.pre_state_main_ros = self.state_main_ros

                if self.state_main_ros != 5:
                    self.check_goal()
                else:
                    if self.self_drive_state == 0:
                        msg_velocity.linear.x = self.speed_x
                        self.sent_velocity.publish(msg_velocity)
                    elif self.self_drive_state == 1:
                        msg_velocity.linear.x = 0.0
                        self.sent_velocity.publish(msg_velocity)
                        self.self_drive_state = 2
                        self.pre_state_main_ros = 0
                    if not self.start_slope:
                        self.speed_x = 0.5
                    if self.slope:
                        self.speed_x = 0.5
                        self.start_slope = True
                    elif not self.slope and self.start_slope:
                        self.self_drive_state = 1
                if self.self_drive_state == 2:
                    self.check_goal()

        elif self.state_main == "Reset":
            self.once = False
            self.pre_state_main_ros = 0
            self.state_main_ros = 1
            if self.retry_order == "First":
                self.slope = False
                self.start_slope = False
                self.self_drive_state = 0

    def sub_main_callback(self, msg_in):
        self.state_main = msg_in.data

    def sub_team_callback(self, msg_in):
        self.state_team = msg_in.data

    def sub_retry_callback(self, msg_in):
        self.state_retry = msg_in.data

    def sub_state_callback(self, msg_in):
        self.state_main_ros = msg_in.data

    def sub_slope_callback(self, msg_in):
        self.slope = msg_in.data

    def set_initial_pose(self, x, y, yaw):
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = "map"
        initial_pose.header.stamp = self.get_clock().now().to_msg()

        initial_pose.pose.position.x = x
        initial_pose.pose.position.y = y
        initial_pose.pose.position.z = 0.0
        (
            initial_pose.pose.orientation.x,
            initial_pose.pose.orientation.y,
            initial_pose.pose.orientation.z,
            initial_pose.pose.orientation.w,
        ) = quaternion_from_euler(0, 0, math.radians(yaw))

        return initial_pose

    def set_point(self, x, y, yaw):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = 0.0
        (
            goal_pose.pose.orientation.x,
            goal_pose.pose.orientation.y,
            goal_pose.pose.orientation.z,
            goal_pose.pose.orientation.w,
        ) = quaternion_from_euler(0, 0, math.radians(yaw))
        return goal_pose

    def goto(self, x, y, yaw):
        self.once = True
        target = self.set_point(x, y, yaw)
        self.navigator.goToPose(target)

    def waypoint(self, pos_waypoint):
        self.once = True
        target_waypoint = [
            self.set_point(point[0], point[1], point[2]) for point in pos_waypoint
        ]
        self.navigator.followWaypoints(target_waypoint)

    def setip(self, x, y, yaw):
        ip = self.set_initial_pose(x, y, yaw)
        self.navigator.setInitialPose(ip)

    def check_goal(self):
        if self.navigator.isTaskComplete() == True and self.once == True:
            msg_goal = Bool()
            self.goal = True
            self.once = False
            msg_goal.data = self.goal
            self.sent_goal.publish(msg_goal)


def main():
    rclpy.init()

    sub = BotNav2()
    rclpy.spin(sub)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
