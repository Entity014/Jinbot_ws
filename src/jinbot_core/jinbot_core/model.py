import rclpy
import torch
import numpy as np
import cv2
import os

from ultralytics import YOLO
from supervision import Detections, BoxAnnotator
from rclpy.node import Node
from std_msgs.msg import Int8MultiArray, Int8, String, Bool
from rclpy import qos


class bucket_model(Node):
    def __init__(self):
        super().__init__("bucket_model_node")
        self.sent_bucket_detect = self.create_publisher(
            Int8MultiArray, "bucket/detect", qos_profile=qos.qos_profile_system_default
        )
        self.sent_timer = self.create_timer(0.05, self.timer_callback)

        self.sub_state = self.create_subscription(
            Int8,
            "state/main_bucket",
            self.sub_state_mainbucket_callback,
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
        self.sub_goal = self.create_subscription(
            Bool,
            "drive/goal",
            self.sub_goal_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.sub_goal
        self.state_main = "Idle"

        self.color_array = [0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.round = 0
        self.check = False

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
        self.frame = np.zeros((480, 640, 3), dtype=np.uint8)
        self.model = self.load_model("best.pt")
        self.CLASS_NAMES_DICT = self.model.model.names
        self.box_annotator = BoxAnnotator(thickness=3, text_thickness=1, text_scale=0.5)

        self.mainbucket_state = -1
        self.cap = cv2.VideoCapture("/dev/video4")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        self.goal = False

    def load_model(self, file):
        model = YOLO(f"{self.path}/{file}")
        model.fuse()
        return model

    def predict(self, frame):
        results = self.model(frame)
        return results

    def plot_bboxes(self, results, frame):
        for result in results[0]:
            if result.boxes.conf.cpu().numpy() >= 0.2:
                detections = Detections(
                    xyxy=result.boxes.xyxy.cpu().numpy(),
                    confidence=result.boxes.conf.cpu().numpy(),
                    class_id=result.boxes.cls.cpu().numpy().astype(int),
                )
                self.labels = [
                    f"{self.CLASS_NAMES_DICT[class_id]}"
                    for confidence, class_id in zip(
                        detections.confidence, detections.class_id
                    )
                ]
                frame = self.box_annotator.annotate(
                    scene=frame, detections=detections, labels=self.labels
                )

                for xyxys, classid in zip(detections.xyxy, detections.class_id):
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
                    if detections.confidence[0] >= 0.3:
                        if (round((xyxys[2] + xyxys[0]) / 2)) <= 420:  # 1st x
                            if round((xyxys[3] + xyxys[1]) / 2) <= (200):
                                self.color_array[2] = classid + 1
                            if round((xyxys[3] + xyxys[1]) / 2) > (200) and round(
                                (xyxys[3] + xyxys[1]) / 2
                            ) <= (440):
                                self.color_array[1] = classid + 1
                            if round((xyxys[3] + xyxys[1]) / 2) > (440):
                                self.color_array[0] = classid + 1
                        if (round((xyxys[2] + xyxys[0]) / 2)) > 420 and (
                            round((xyxys[2] + xyxys[0]) / 2)
                        ) <= 850:  # 2nd x
                            if round((xyxys[3] + xyxys[1]) / 2) <= (200):
                                self.color_array[5] = classid + 1
                            if round((xyxys[3] + xyxys[1]) / 2) > (200) and round(
                                (xyxys[3] + xyxys[1]) / 2
                            ) <= (440):
                                self.color_array[4] = classid + 1
                            if round((xyxys[3] + xyxys[1]) / 2) > (440):
                                self.color_array[3] = classid + 1
                        if (round((xyxys[2] + xyxys[0]) / 2)) > 850:  # 3rd x
                            if round((xyxys[3] + xyxys[1]) / 2) <= (200):
                                self.color_array[8] = classid + 1
                            if round((xyxys[3] + xyxys[1]) / 2) > (200) and round(
                                (xyxys[3] + xyxys[1]) / 2
                            ) <= (440):
                                self.color_array[7] = classid + 1
                            if round((xyxys[3] + xyxys[1]) / 2) > (440):
                                self.color_array[6] = classid + 1
                    self.round += 1

        if self.round == 9:
            self.check = True

        return frame

    def timer_callback(self):
        self.msg_detect = Int8MultiArray()
        ref, frame = self.cap.read()
        frame = cv2.flip(frame, 0)
        frame = cv2.flip(frame, 1)
        if self.mainbucket_state == 0:
            if not self.check and self.goal and self.state_main == "Start":
                results = self.predict(frame)
                self.frame = self.plot_bboxes(results, frame)
                cv2.line(frame, (0, 200), (1280, 200), (0, 0, 0), 4)
                cv2.line(frame, (0, 440), (1280, 440), (0, 0, 0), 4)
                cv2.line(frame, (420, 0), (420, 720), (0, 0, 0), 4)
                cv2.line(frame, (850, 0), (850, 860), (0, 0, 0), 4)
                cv2.imshow("frame", self.frame)
                for i in range(9):
                    self.msg_detect.data.append(self.color_array[i])

                self.sent_bucket_detect.publish(self.msg_detect)
                cv2.destroyAllWindows()
                exit()
            if self.check:
                self.check = False

        if not ref:
            cv2.destroyAllWindows()
            exit()

        if cv2.waitKey(1) & 0xFF == ord("q"):
            cv2.destroyAllWindows()
            exit()

    def sub_state_mainbucket_callback(self, msg_in):
        self.mainbucket_state = msg_in.data

    def sub_main_callback(self, msg_in):
        self.state_main = msg_in.data

    def sub_goal_callback(self, msg_in):
        self.goal = msg_in.data


def main():
    rclpy.init()
    sub = bucket_model()
    rclpy.spin(sub)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
