import rclpy
import numpy as np
import torch
import cv2
import os

from ultralytics import YOLO
from supervision import Detections, BoxAnnotator
from rclpy.node import Node
from std_msgs.msg import Int8, Float32, Bool
from geometry_msgs.msg import Twist
from rclpy import qos
from cv_bridge import CvBridge


class BotFlagModel(Node):
    def __init__(self):
        super().__init__("flag_grip_model_node")
        self.sent_tune_gripper = self.create_publisher(
            Float32, "gripper/flag/tune", qos_profile=qos.qos_profile_system_default
        )
        self.sent_detect_gripper = self.create_publisher(
            Bool, "gripper/flag/detect", qos_profile=qos.qos_profile_system_default
        )
        self.sent_timer = self.create_timer(0.05, self.timer_callback)
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

        self.cap = cv2.VideoCapture("/dev/video2")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.path = os.path.join(
            os.path.expanduser("~"),
            "Jinbot_ws",
            "install",
            "jinbot_core",
            "share",
            "jinbot_core",
            "weights",
        )
        self.model = self.load_model("model1_FN.pt")
        self.CLASS_NAMES_DICT = self.model.model.names
        self.box_annotator = BoxAnnotator(thickness=3, text_thickness=1, text_scale=0.5)
        self.is_found = False
        self.result_model = []
        self.tuning = 0.0
        self.frame = np.zeros((480, 640, 3), dtype=np.uint8)
        self.mainros_state = 0

        self.current_step_motor1 = 0.0
        self.current_step_motor2 = 0.0
        self.current_step_motor3 = 0.0
        self.distance_step_motor1 = 0.0
        self.distance_step_motor2 = 0.0
        self.distance_step_motor3 = 0.0

    def load_model(self, file):
        model = YOLO(f"{self.path}/{file}")
        model.fuse()
        return model

    def predict(self, frame):
        results = self.model(frame)
        return results

    def plot_bboxes(self, results, frame):
        for result in results[0]:
            if result.boxes.conf.cpu().numpy() >= 0.4:
                detections = Detections(
                    xyxy=result.boxes.xyxy.cpu().numpy(),
                    confidence=result.boxes.conf.cpu().numpy(),
                    class_id=result.boxes.cls.cpu().numpy().astype(int),
                )

                self.labels = [
                    f"{self.CLASS_NAMES_DICT[class_id]} {confidence:0.2f}"
                    for confidence, class_id in zip(
                        detections.confidence, detections.class_id
                    )
                ]

                frame = self.box_annotator.annotate(
                    scene=frame, detections=detections, labels=self.labels
                )

                for xyxys in detections.xyxy:
                    cv2.circle(
                        frame,
                        (
                            round((xyxys[2] + xyxys[0]) / 2),
                            round((xyxys[3] + xyxys[1]) / 2),
                        ),
                        5,
                        (255, 0, 0),
                        -1,
                    )

                if detections.confidence[0] > 0.6:
                    self.is_found = True
                    self.result_model = detections

        return frame

    def timer_callback(self):
        msg_tuning = Float32()
        msg_detect = Bool()
        if self.mainros_state == 4 and self.current_step_motor3 >= round(
            (179000 / 0.46) * 0.4
        ):
            ref, frame = self.cap.read()
            frame = cv2.flip(frame, 0)
            frame = cv2.flip(frame, 1)
            if not self.is_found:
                results = self.predict(frame)
                self.frame = self.plot_bboxes(results, frame)
                cv2.circle(
                    frame,
                    (
                        round(640 / 2),
                        round(480 / 2),
                    ),
                    5,
                    (255, 0, 0),
                    -1,
                )
            if len(self.result_model) != 0:
                self.tuning = (self.result_model.xyxy[0][0] - 320) * 1e-3
                msg_tuning.data = self.tuning
                msg_detect.data = self.is_found
                self.sent_tune_gripper.publish(msg_tuning)
                self.sent_detect_gripper.publish(msg_detect)
                # cv2.destroyAllWindows()
                # exit()
            cv2.imshow("frame", self.frame)

            if not ref:
                cv2.destroyAllWindows()
                exit()

            if cv2.waitKey(1) & 0xFF == ord("q"):
                cv2.destroyAllWindows()
                exit()

    def sub_state_callback(self, msg_in):
        self.mainros_state = msg_in.data

    def sub_step_motor_callback(self, msg_in):
        self.current_step_motor1 = msg_in.linear.x
        self.current_step_motor2 = msg_in.linear.y
        self.current_step_motor3 = msg_in.linear.z
        self.distance_step_motor1 = msg_in.angular.x
        self.distance_step_motor2 = msg_in.angular.y
        self.distance_step_motor3 = msg_in.angular.z


def main():
    rclpy.init()

    sub = BotFlagModel()
    rclpy.spin(sub)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
