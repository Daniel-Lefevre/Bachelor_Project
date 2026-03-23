import os

from ultralytics import YOLO

# 1. Load the YOLOv8 Nano model (smallest/fastest for small datasets)
model = YOLO("yolov8n.pt")

# Get the full path to the current folder
base_path = os.path.dirname(os.path.abspath(__file__))
# Combine it with your dataset path
yaml_path = os.path.join(base_path, "dataset_robot_0", "data.yaml")

# 2. Train the model
# We set 'freeze=10' to freeze the first 10 layers (the backbone)
# to preserve general visual knowledge.
results = model.train(
    data=yaml_path,
    epochs=5,
    imgsz=640,
    batch=16,
    freeze=10,
    augment=True,  # Enables automatic data augmentation
    project=base_path + "/results",  # This creates a main folder
    name="Object_Finder_v1",
)
