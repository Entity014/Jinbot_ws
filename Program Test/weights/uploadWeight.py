from roboflow import Roboflow
import os

home = os.path.expanduser("~")
path = os.path.join(home, "Jinbot_ws/Program Test")

rf = Roboflow(api_key="kGh3yLlFVUGswU8bw8sm")
project = rf.workspace().project("project-reai/flag-1")
project.version(1).deploy(model_type="yolov8", model_path=f"{path}")
