from roboflow import Roboflow
import os

path = os.path.join(os.path.expanduser("~"), "Jinbot_ws/Program Test")

rf = Roboflow(api_key="5rFUseoGx1H3AEw6iDMF")
project = rf.workspace().project("flag-2")
project.version(1).deploy(model_type="yolov8", model_path=f"{path}")
