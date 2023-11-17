from roboflow import Roboflow
import os

path = os.path.join(os.path.expanduser("~"), "Jinbot_ws/Program Test")

rf = Roboflow(api_key="qxD2opSG6qzE1kByZ042")
project = rf.workspace().project("flag-new-1")
project.version(1).deploy(model_type="yolov8", model_path=f"{path}")
