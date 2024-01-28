import rclpy
import numpy as np
import cv2
import os

from rclpy.node import Node
from std_msgs.msg import Int8, Float32, Bool, String
from geometry_msgs.msg import Twist
from rclpy import qos
from cv_bridge import CvBridge


class BotFlagColor(Node):
    def __init__(self):
        super().__init__("flag_grip_color_node")
        self.sent_color_gripper = self.create_publisher(
            String, "gripper/flag/color", qos_profile=qos.qos_profile_system_default
        )
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
        self.sub_step_motor = self.create_subscription(
            Twist,
            "gripper/flag/state",
            self.sub_step_motor_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.sub_step_motor

        self.sent_timer = self.create_timer(0.05, self.timer_callback)
        self.frame = np.zeros((760, 600, 3), dtype=np.uint8)
        self.mainros_state = 0

        self.current_step_motor1 = 0.0
        self.current_step_motor2 = 0.0
        self.current_step_motor3 = 0.0
        self.distance_step_motor1 = 0.0
        self.distance_step_motor2 = 0.0
        self.distance_step_motor3 = 0.0

        self.color = ""
        self.pre_color = ""
        self.sent_color = ""
        self.isRed = False
        self.pre_time = self.get_clock().now().nanoseconds * 1e-9
        self.time_start = self.get_clock().now().nanoseconds * 1e-9
        self.state_color = 0

        self.state_main = "Idle"

    def timer_callback(self):
        msg_color = String()
        if self.mainros_state == 6 and self.current_step_motor3 >= int(
            (415000 / 0.455) * 0.45
        ):
            self.cap = cv2.VideoCapture("/dev/video3")
            ref, self.frame = self.cap.read()
            if ref:
                self.frame = cv2.resize(self.frame, (760, 600))
                hsv_frame = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
                dil_frame = dilate_frame(hsv_frame, 19, 10, 110, 85)
                self.search_contours(dil_frame)
                if self.state_main == "Start":
                    if (
                        ((self.get_clock().now().nanoseconds * 1e-9) - self.pre_time)
                        >= 8
                    ) and self.isRed:
                        self.state_color += 1
                        self.pre_time = self.get_clock().now().nanoseconds * 1e-9
                    self.check_color()
                else:
                    self.sent_color = ""
                    self.isRed = False
                    self.state_color = 0
                    self.pre_time = self.get_clock().now().nanoseconds * 1e-9
                msg_color.data = self.sent_color
                self.sent_color_gripper.publish(msg_color)
                cv2.putText(
                    self.frame,
                    f"{self.state_color} {self.sent_color, self.get_clock().now().nanoseconds * 1e-9 - self.time_start}",
                    (100, 100),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,
                    (255, 0, 255),
                    2,
                    cv2.LINE_AA,
                )
                cv2.imshow("frame", self.frame)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            cv2.destroyAllWindows()
            exit()

        # if self.cap.get(cv2.CAP_PROP_POS_FRAMES) == self.cap.get(
        #     cv2.CAP_PROP_FRAME_COUNT
        # ):
        #     self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)

    def sub_state_callback(self, msg_in):
        self.mainros_state = msg_in.data

    def sub_main_callback(self, msg_in):
        self.state_main = msg_in.data

    def sub_step_motor_callback(self, msg_in):
        self.current_step_motor1 = msg_in.linear.x
        self.current_step_motor2 = msg_in.linear.y
        self.current_step_motor3 = msg_in.linear.z
        self.distance_step_motor1 = msg_in.angular.x
        self.distance_step_motor2 = msg_in.angular.y
        self.distance_step_motor3 = msg_in.angular.z

    def search_contours(self, mask):
        contours, hierarchy = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        for contour in contours:
            area = cv2.contourArea(contour)
            x, y, w, h = cv2.boundingRect(contour)
            epsilon = 0.01 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            num_vertices = len(approx)
            if area > 1e4:
                self.color = f"Red"
                if self.pre_color != self.color:
                    self.isRed = True
                    self.time_start = self.get_clock().now().nanoseconds * 1e-9
                    self.pre_color = self.color
                cv2.drawContours(self.frame, [approx], -1, (0, 255, 0), 2)

    def check_color(self):
        if self.state_color == 1:
            self.sent_color = "Red"
        elif self.state_color == 2:
            self.sent_color = "Blue"
        elif self.state_color == 3:
            self.sent_color = "Blue"
        elif self.state_color == 4:
            self.sent_color = "Red"
        elif self.state_color == 5:
            self.sent_color = "Green"
        elif self.state_color == 6:
            self.sent_color = "Green"
        elif self.state_color >= 7:
            self.state_color = 1


def constrain(value, min_val, max_val):
    return min(max_val, max(min_val, value))


def dilate_frame(hsv_frame, hue, diff=10, sl=0, vl=0, vh=255):
    lower_hsv = np.array([constrain(hue - diff, 0, 179), sl, vl])
    upper_hsv = np.array([constrain(hue + diff, 0, 179), 255, vh])
    mask = cv2.inRange(hsv_frame, lower_hsv, upper_hsv)
    return mask


def main():
    rclpy.init()

    sub = BotFlagColor()
    rclpy.spin(sub)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
