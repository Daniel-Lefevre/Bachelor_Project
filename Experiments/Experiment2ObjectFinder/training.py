from ultralytics import YOLO

# 1. Load the YOLOv8 Nano model (smallest/fastest for small datasets)
model = YOLO('yolov8n.pt')

# 2. Train the model
# We set 'freeze=10' to freeze the first 10 layers (the backbone)
# to preserve general visual knowledge.
results = model.train(
    data='dataset_robot_0/data.yaml',
    epochs=100,
    imgsz=640,
    batch=16,
    freeze=10,
    augment=True # Enables automatic data augmentation
)
