from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from ament_index_python.packages import get_package_share_directory
from tf_transformations import quaternion_from_euler
import rclpy
import numpy as np
import math
import time

from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String, Int8MultiArray, Int8, Bool
from rclpy import qos, Parameter


class motor(Node):
    def __init__(self):
        super().__init__("motor_node")
        self.sent_bucket_pos = self.create_publisher(
            Twist, "gripper/bucket/pos", qos_profile=qos.qos_profile_system_default
        )
        self.sent_bucket_lock = self.create_publisher(
            Bool, "gripper/bucket/lock", qos_profile=qos.qos_profile_system_default
        )
        self.sent_state_mainbucket = self.create_publisher(
            Int8, "state/main_bucket", qos_profile=qos.qos_profile_system_default
        )
        self.sent_bucket_get = self.create_publisher(
            Bool, "gripper/bucket/get", qos_profile=qos.qos_profile_system_default
        )
        self.sent_bucket_sent = self.create_publisher(
            Bool, "gripper/bucket/sent", qos_profile=qos.qos_profile_system_default
        )
        self.sent_bucket_storage = self.create_publisher(
            Int8MultiArray, "bucket/storage", qos_profile=qos.qos_profile_system_default
        )
        self.sub_goal = self.create_subscription(
            Bool,
            "drive/goal",
            self.sub_goal_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.sub_goal

        self.sub_main = self.create_subscription(
            String,
            "state/main",
            self.sub_state_main_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.sub_main

        self.sub_main_ros = self.create_subscription(
            Int8,
            "state/main_ros",
            self.sub_state_mainros_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.sub_main_ros

        self.sub_retry = self.create_subscription(
            String,
            "state/retry",
            self.sub_state_retry_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.sub_retry

        self.sub_team = self.create_subscription(
            String,
            "state/team",
            self.sub_team_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.sub_team

        self.sub_state = self.create_subscription(
            Twist,
            "step/motor",
            self.sub_step_motor_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.sub_state

        self.sub_bucket_detect = self.create_subscription(
            Int8MultiArray,
            "bucket/detect",
            self.sub_bucket_detect_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.sub_bucket_detect

        self.sent_timer = self.create_timer(0.05, self.timer_callback)

        self.declare_parameters(
            "",
            [
                ("position.y", Parameter.Type.DOUBLE),
                ("position.z1", Parameter.Type.DOUBLE),
                ("position.z2", Parameter.Type.DOUBLE),
                ("position.z3", Parameter.Type.DOUBLE),
                ("lock", Parameter.Type.BOOL),
                ("main", Parameter.Type.BOOL),
            ],
        )

        self.mission_get = False
        self.mission_sent = False
        self.position_y = 0.0
        self.position_z1 = 0.0
        self.position_z2 = 0.0
        self.position_z3 = 0.0
        self.current_position_y1 = 0.0
        self.current_position_y2 = 0.0
        self.current_position_z1 = 0.0
        self.current_position_z2 = 0.0
        self.current_position_z3 = 0.0
        self.main_state = "Idle"
        self.mainros = -1
        self.main_retry = "Idle"
        self.main_team = "Idle"
        self.lock = False
        self.mainbucket_state = -1
        self.detect_array = [0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.storage = [0, 0, 0]
        self.re_mission = np.zeros(3)
        self.aimR = np.zeros(3)
        self.aimG = np.zeros(3)
        self.aimB = np.zeros(3)
        self.dis_goal = 0.0
        self.basemove = 0.0
        self.process = False
        self.i = 0

        self.navigator = BasicNavigator()
        self.goal = False
        self.once = True

    def timer_callback(self):
        msg_mission_get = Bool()
        msg_mission_sent = Bool()
        msg_pos = Twist()
        msg_lock = Bool()
        msg_statemain = Int8()
        msg_storage = Int8MultiArray()

        # --------------------------Para-----------------------------------#
        # self.position_y = self.get_parameter("position.y").get_parameter_value().double_value
        # self.position_z1 = self.get_parameter("position.z1").get_parameter_value().double_value
        # self.position_z2 = self.get_parameter("position.z2").get_parameter_value().double_value
        # self.position_z3 = self.get_parameter("position.z3").get_parameter_value().double_value
        # self.lock = self.get_parameter("lock").get_parameter_value().bool_value
        # self.signal = self.get_parameter("main").get_parameter_value().bool_value

        # --------------------------message--------------------------------#
        msg_mission_get.data = self.mission_get
        msg_mission_sent.data = self.mission_sent
        msg_pos.linear.x = self.position_y
        msg_pos.linear.y = self.position_z1
        msg_pos.linear.z = self.position_z2
        msg_pos.angular.x = self.position_z3
        msg_lock.data = self.lock
        msg_storage.data = self.storage
        msg_statemain.data = self.mainbucket_state

        # -----------------------------picking------------------------------#
        if self.main_state == "Start" and self.mainros == 1 and self.goal:
            if self.mainbucket_state == 1:
                self.detect_array = np.reshape(self.detect_array, (3, 3))
                self.selectTarget("R")
                self.selectTarget("G")
                self.selectTarget("B")
                self.mainbucket_state = 2
            if self.mainbucket_state == 2 and self.main_retry != "First":
                if self.storage[self.i] == 0:
                    if self.i == 0 and not self.process:  # Pick up Red
                        if np.count_nonzero(self.aimR) == 0:
                            self.re_mission[self.i] = 3
                            self.i = 1
                        else:
                            self.dis_to_go(self.aimR)
                            self.position_y = self.dis_goal
                            self.preaimR = self.aimR
                    if self.i == 1 and not self.process:  # Pick up Green
                        if np.count_nonzero(self.aimG) == 0:
                            self.re_mission[self.i] = 2
                            self.i = 2
                        else:
                            self.dis_to_go(self.aimG)
                            self.position_y = self.dis_goal
                            self.preaimG = self.aimG
                    if self.i == 2 and not self.process:  # Pick up Blue
                        if np.count_nonzero(self.aimB) == 0:
                            self.re_mission[self.i] = 1
                            self.i = 3
                        else:
                            self.dis_to_go(self.aimB)
                            self.position_y = self.dis_goal
                            self.preaimB = self.aimB
                    if (
                        self.current_position_y1 == self.position_y
                        and self.current_position_y2 == self.position_y
                        and not self.process
                    ):
                        self.lock = True
                        if self.i == 0:
                            self.position_z1 = -25500.0
                            self.process = True
                        if self.i == 1:
                            self.position_z2 = -31000.0
                            self.process = True
                        if self.i == 2:
                            self.position_z3 = -25500.0
                            self.process = True
                    if self.process:
                        if self.current_position_z1 == -25500.0 and self.i == 0:
                            self.position_z1 = -1000.0
                            self.storage[0] = 3
                            self.aimR = np.zeros(3)
                            self.i = 1
                            self.process = False
                        if self.current_position_z2 == -31000.0 and self.i == 1:
                            self.position_z2 = -1000.0
                            self.storage[1] = 2
                            self.aimG = np.zeros(3)
                            self.i = 2
                            self.process = False
                        if self.current_position_z3 == -25500.0 and self.i == 2:
                            self.position_z3 = -1000.0
                            self.storage[2] = 1
                            self.aimB = np.zeros(3)
                            self.i = 3
                            self.process = False

                    if self.i == 3:
                        self.i = 0
                        self.mainbucket_state = 3

            if self.mainbucket_state == 3:
                if self.current_position_z1 != -500.0:
                    self.position_z1 = -500.0
                if self.current_position_z2 != -500.0:
                    self.position_z2 = -500.0
                if self.current_position_z3 != -500.0:
                    self.position_z3 = -500.0
                if (
                    self.current_position_z1 == -500.0
                    and self.current_position_z2 == -500.0
                    and self.current_position_z3 == -500.0
                ):
                    self.goal = False
                    if np.count_nonzero(self.storage) != 0:
                        self.mission_get = True
                        self.mainbucket_state = 4
                    if np.count_nonzero(self.storage) == 0:
                        # self.re_pick()
                        self.mainbucket_state = 5

            if self.mainbucket_state == 5:
                self.position_y = 18000.0
                if not self.mission_get:
                    self.lock = True
                    self.position_z1 = -25500.0
                    self.position_z2 = -29000.0
                    self.position_z3 = -25500.0
                    self.mission_get = True
                if (
                    self.current_position_z1 == -25500.0
                    and self.current_position_z2 == -29000.0
                    and self.current_position_z3 == -25500.0
                ):
                    self.position_z1 = -500.0
                    self.position_z2 = -500.0
                    self.position_z3 = -500.0
                    self.mainbucket_state = 4

        # -----------------------------deli--------------------------------#
        if (
            self.mainbucket_state == 4
            and not self.mission_sent
            and self.mainros == 2
            and self.goal
        ):
            self.position_y = 28000.0
            if (
                self.current_position_y1 == self.position_y
                and self.current_position_y2 == self.position_y
            ):
                self.position_z1 = -25500.0
                self.position_z2 = -32000.0
                self.position_z3 = -25500.0
            if (
                self.current_position_z1 == -25500.0
                and self.current_position_z2 == -32000.0
                and self.current_position_z3 == -25500.0
            ):
                self.lock = False
                self.position_z1 = -1000.0
                self.position_z2 = -1000.0
                self.position_z3 = -1000.0
                self.mission_sent = True

        # -----------------------------retry--------------------------------#
        if self.mainbucket_state == 2 and self.main_retry == "First":
            if self.preaimR != self.aimR:
                self.storage[0] = 3
            if self.preaimG != self.aimG:
                self.storage[1] = 2
            if self.preaimB != self.aimB:
                self.storage[2] = 1
            self.main_retry = "Idle"

        # ----------------------------publish-------------------------------#
        self.sent_bucket_pos.publish(msg_pos)
        self.sent_bucket_lock.publish(msg_lock)
        self.sent_bucket_storage.publish(msg_storage)
        self.sent_state_mainbucket.publish(msg_statemain)
        self.sent_bucket_get.publish(msg_mission_get)
        self.sent_bucket_sent.publish(msg_mission_sent)

    def selectTarget(self, color):
        match color:
            case "R":  # z1->class_id:3
                if self.detect_array[0][0] == 3:
                    self.aimR[0] = 18000
                if self.detect_array[0][1] == 3:
                    self.aimR[1] = 93000
                if self.detect_array[0][2] == 3:
                    self.aimR[2] = 163000
            case "G":  # z2->class_id:2
                if self.detect_array[1][0] == 2:
                    self.aimG[0] = 18000
                if self.detect_array[1][1] == 2:
                    self.aimG[1] = 93000
                if self.detect_array[1][2] == 2:
                    self.aimG[2] = 163000
            case "B":  # z3->class_id:1
                if self.detect_array[2][0] == 1:
                    self.aimB[0] = 18000
                if self.detect_array[2][1] == 1:
                    self.aimB[1] = 93000
                if self.detect_array[2][2] == 1:
                    self.aimB[2] = 163000

    def dis_to_go(self, aim):
        check = 1000000.0
        for i in range(3):
            if aim[i] != 0:
                if abs(self.current_position_y1 - aim[i]) < check:
                    check = abs(self.current_position_y1 - aim[i])
                    self.dis_goal = aim[i]

    def re_pick(self):
        for j in self.re_mission:
            match j:
                case 3:  # R
                    for i in range(3):
                        for j in range(3):
                            if self.detect_array[i][j] == 3:
                                self.remake(i)
                                break
                case 2:  # G
                    for i in range(3):
                        for j in range(3):
                            if self.detect_array[i][j] == 2:
                                self.remake(i)
                                break
                case 1:  # B
                    for i in range(3):
                        for j in range(3):
                            if self.detect_array[i][j] == 1:
                                self.remake(i)
                                break

    def remake(self, basemove):
        movebase = False
        ##################sent msg to pwm#####################
        # Blue ? Red

        ######################################################
        if movebase:
            self.mainbucket_state = 0
            movebase = False

    def sub_step_motor_callback(self, msg_in):
        self.current_position_y1 = msg_in.linear.x  # y1 axis stepper
        self.current_position_y2 = msg_in.linear.y  # y2 axis stepper
        self.current_position_z1 = msg_in.linear.z  # z(R) axis stepper
        self.current_position_z2 = msg_in.angular.x  # z(G) axis stepper
        self.current_position_z3 = msg_in.angular.y  # z(B) axis stepper

        if (
            msg_in.linear.x == 18000.0
            and msg_in.linear.y == 18000.0
            and msg_in.linear.z == -15000.0
            and msg_in.angular.x == -15000.0
            and msg_in.angular.y == -15000.0
            and msg_in.angular.z == 1
            and self.mainbucket_state == -1
        ):
            self.mainbucket_state = 0

    ################################ mission condition stages ####################################

    def sub_bucket_detect_callback(self, msg_in):
        if self.mainbucket_state == 0 and self.detect_array != list(msg_in.data):
            self.detect_array = list(msg_in.data)
            self.mainbucket_state = 1
        # else:
        #     self.mainbucket_state = 5

    def sub_state_main_callback(self, msg_in):
        self.main_state = msg_in.data

        # if msg_in.data == "Reset":
        #     self.mission_get = False
        #     self.mission_sent = False
        #     self.position_y = 0.0
        #     self.position_z1 = 0.0
        #     self.position_z2 = 0.0
        #     self.position_z3 = 0.0
        #     self.lock = False
        #     self.mainbucket_state = -1
        #     self.detect_array = [0, 0, 0, 0, 0, 0, 0, 0, 0]
        #     self.storage = [0, 0, 0]
        #     self.re_mission = np.zeros(3)
        #     self.aimR = np.zeros(3)
        #     self.aimG = np.zeros(3)
        #     self.aimB = np.zeros(3)
        #     self.dis_goal = 0.0
        #     self.basemove = 0.0

    def sub_state_retry_callback(self, msg_in):
        self.main_retry = msg_in.data
        # if self.main_retry == "First":
        #     self.storage = [0, 0, 0]
        #     self.mainbucket_state = -1

    def sub_state_mainros_callback(self, msg_in):
        self.mainros = msg_in.data

    def sub_team_callback(self, msg_in):
        self.main_team = msg_in.data

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

    def setip(self, x, y, yaw):
        ip = self.set_initial_pose(x, y, yaw)
        self.navigator.setInitialPose(ip)

    def sub_goal_callback(self, msg_in):
        self.goal = msg_in.data


def main():
    rclpy.init()
    sub = motor()
    rclpy.spin(sub)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
