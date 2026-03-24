import os

import yaml
from ultralytics import YOLO

# 1. Setup Absolute Paths
try:
    # This works if running as a .py file
    base_path = os.path.dirname(os.path.abspath(__file__))
except NameError:
    # This works if running in a Google Colab / Jupyter cell
    base_path = "/content/dataset_robot_1/dataset_robot_1"

yaml_path = os.path.join(base_path, "data.yaml")

# 2. DYNAMICALLY UPDATE YAML PATH
# This ensures YOLO looks for Training_Data/Validation_Data relative to THIS folder
if os.path.exists(yaml_path):
    with open(yaml_path, "r") as f:
        data_config = yaml.safe_load(f)

    data_config["path"] = base_path  # Force the root path to this script's folder

    with open(yaml_path, "w") as f:
        yaml.dump(data_config, f)
else:
    print(f"Error: {yaml_path} not found!")

# 3. Load the YOLOv8 Nano model
model = YOLO("yolov8n.pt")

# 4. Train the model
# Using os.path.join for the project folder keeps it clean
model.tune(
    data=yaml_path,
    epochs=30,  # Each 'trial' runs for 30 epochs
    iterations=50,  # Try 50 different combinations of settings
    optimizer="AdamW",
    plots=True,  # Generates charts showing which settings worked
    save=True,
    val=True,  # CRITICAL: Uses your validation set to score the trial
    project=os.path.join(base_path, "tuning_results"),
)
