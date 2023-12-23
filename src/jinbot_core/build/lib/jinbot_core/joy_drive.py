import rclpy
import math
import numpy as np

from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, Vector3
from rclpy import qos


class Ps4(Node):
    def __init__(self):
        super().__init__("xbox_control_node")
        self.dat = self.create_subscription(
            Joy, "joy", self.sub_callback, qos_profile=qos.qos_profile_sensor_data
        )
        self.dat

        self.sent_drive = self.create_publisher(
            Twist, "drive/pwm", qos_profile=qos.qos_profile_system_default
        )
        # self.sent_flag_theta = self.create_publisher(
        #     Vector3, "gripper/flag/hand", qos_profile=qos.qos_profile_system_default
        # )
        self.sent_drive_timer = self.create_timer(0.05, self.sent_drive_callback)

        self.all = [
            "X",
            "O",
            "Dummy2",
            "S",
            "T",
            "Dummy5",
            "L1",
            "R1",
            "Dummy8",
            "Dummy9",
            "L",
            "R",
            "PS",
            "LS",
            "RS",
            "XBOX",
        ]  # ? XBOX
        self.all2 = ["LX", "LY", "RX", "RY", "LT", "RT", "AX", "AY"]  # ? XBOX
        self.button = {element: 0 for element in self.all}
        self.axes = {element: 0 for element in self.all2}

        self.button["L2"] = 0
        self.button["R2"] = 0
        self.theta1 = 0.0
        self.theta2 = 0.0

    def sub_callback(self, msg_in):  # subscription topic
        self.new_dat = msg_in
        # ? XBOX
        if msg_in.axes[5] < 0:
            self.button["L2"] = 1
        else:
            self.button["L2"] = 0
        if msg_in.axes[4] < 0:
            self.button["R2"] = 1
        else:
            self.button["R2"] = 0

        for index, element in enumerate(self.all):
            self.button[element] = msg_in.buttons[index]
        #     print(f"{self.all[index]}  :  {self.button[element]}")

        for index, element in enumerate(self.all2):
            if msg_in.axes[index] <= 0.1 and msg_in.axes[index] >= -0.1:
                self.axes[element] = 0
            else:
                self.axes[element] = msg_in.axes[index]

    def sent_drive_callback(self):  # publisher drive topic
        msg = Twist()
        msg_theta = Vector3()

        x = 0.0
        y = 0.0
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0

        if (self.axes["AX"] != 0) or (self.axes["AY"] != 0):
            y = -1 * self.axes["AY"]
            x = -1 * self.axes["AX"]

        else:
            y = -1 * self.axes["LY"]
            x = -1 * self.axes["LX"]

        y = y * -1
        turn = np.interp(self.axes["RX"], [-1, 1], [-1, 1])
        theta = math.atan2(y, x)
        power = math.hypot(x, y)
        sin = math.sin(theta - math.pi / 4)
        cos = math.cos(theta - math.pi / 4)
        Max = max(abs(sin), abs(cos))
        leftFront = power * cos / Max - turn
        rightFront = power * sin / Max + turn
        leftBack = power * sin / Max - turn
        rightBack = power * cos / Max + turn

        if (power + abs(turn)) > 1:
            leftFront /= power + abs(turn)
            rightFront /= power + abs(turn)
            leftBack /= power + abs(turn)
            rightBack /= power + abs(turn)

        msg.linear.x = float(round(leftFront * 1023))
        msg.linear.y = float(round(rightFront * 1023))
        msg.linear.z = float(round(leftBack * 1023))
        msg.angular.x = float(round(rightBack * 1023))

        if self.button["X"] == 1:
            msg_theta.x = 90.0

        self.get_logger().info(
            f"{msg.linear.x} {msg.linear.y} {msg.linear.z} {msg.angular.x}"
        )
        self.sent_drive.publish(msg)
        # self.sent_flag_theta.publish(msg_theta)


def main():
    rclpy.init()

    sub = Ps4()
    rclpy.spin(sub)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
