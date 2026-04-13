import os

from ultralytics import YOLO

# 1. Load the trained model
base_path = os.path.dirname(os.path.abspath(__file__))
dataset = "dataset_robot_1"

final_model_path = os.path.join(base_path, f"{dataset}/final_results_single/Robot_Object_Finder_Final/weights/best.pt")
model = YOLO(final_model_path)


data_yaml_path = os.path.join(base_path, dataset, "data_single.yaml")
# 2. Run Validation to get predictions on the test set
# we use save_json=True or simply iterate through results
results = model.val(data=data_yaml_path, split="test", plots=True)
