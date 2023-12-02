from typing import Any
import torch
import numpy as np
import cv2
import os
from pathlib import Path
from ultralytics import YOLO

from supervision import Detections, BoxAnnotator


class ObjectDetection:
    def __init__(self, capture_index):
        self.captur_index = capture_index

        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        print("Using Device: ", self.device)

        self.path = os.path.join(
            os.path.expanduser("~"), "Jinbot_ws/Program Test/weights/"
        )

        self.model = self.load_model("best.pt")
        self.CLASS_NAMES_DICT = self.model.model.names
        self.box_annotator = BoxAnnotator(thickness=3, text_thickness=1, text_scale=0.5)

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

        return frame

    def __call__(self):
        cap = cv2.VideoCapture(self.captur_index)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        while cap.isOpened():
            ref, frame = cap.read()
            results = self.predict(frame)
            frame = self.plot_bboxes(results, frame)
            cv2.imshow("frame", frame)

            if not ref:
                break

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

        cap.release()
        cv2.destroyAllWindows()


detector = ObjectDetection(capture_index=2)
detector()
