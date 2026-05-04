import os

import torch
import yaml
from ultralytics import YOLO

device = 0 if torch.cuda.is_available() else "cpu"


# define the dataset to use it on
dataset = "dataset_robot_0"
single_or_multi = "multi"


# 1. Setup Absolute Paths (Works for both Colab and Local .py files)
try:
    # This works if running as a .py file
    base_path = os.path.dirname(os.path.abspath(__file__))
except NameError:
    # This works if running in a Google Colab / Jupyter cell
    # Ensure this matches your folder name in the Colab sidebar

    base_path = f"/content/{dataset}/{dataset}"

yaml_path = os.path.join(base_path, dataset, f"data_{single_or_multi}.yaml")

# 2. DYNAMICALLY UPDATE YAML PATH
# Forces YOLO to look for images relative to the current base_path
if os.path.exists(yaml_path):
    with open(yaml_path, "r") as f:
        data_config = yaml.safe_load(f)

    data_config["path"] = os.path.join(base_path, dataset)

    with open(yaml_path, "w") as f:
        yaml.dump(data_config, f)
    print(f"✅ YAML 'path' updated to: {base_path}")
else:
    print(f"❌ Error: {yaml_path} not found!")

# 3. Load the fresh YOLOv8 Nano model
model = YOLO("yolov8n.pt")

# 4. Final Training Run
# We use freeze=0 (default) to allow full specialization of the model
print("Device:", torch.cuda.get_device_name(0) if torch.cuda.is_available() else "CPU")
print("--- Starting Final Production Training ---")
results = model.train(
    data=yaml_path,
    epochs=100,  # High limit; patience will stop it when optimal
    imgsz=640,  # Standard high-res for YOLOv8
    batch=16,  # Good balance for T4 GPU memory
    patience=15,  # Stop if no improvement for 5 epochs (prevents overfitting)
    project=os.path.join(base_path, f"{dataset}/final_results_{single_or_multi}"),
    name="Robot_Object_Finder_Final",
    exist_ok=True,  # Overwrite if folder exists
    save=True,  # Ensures the final and best models are saved
    save_period=-1,  # Disables intermediate epoch checkpoints (only keeps best.pt and last.pt)
    plots=True,
    device=device,
)

# 5. Final Evaluation on UNSEEN Test Data
print("\n--- FINAL EVALUATION ON TEST DATA ---")
# Load the best weights produced by this specific run
final_model_path = os.path.join(base_path, f"{dataset}/final_results_{single_or_multi}/Robot_Object_Finder_Final/weights/best.pt")
final_model = YOLO(final_model_path)

metrics = final_model.val(split="test")

print("\n" + "=" * 30)
print(f"mAP@50:    {metrics.box.map50:.4f}")
print(f"mAP@50-95: {metrics.box.map:.4f}")
print(f"Precision: {metrics.box.mp:.4f}")
print(f"Recall:    {metrics.box.mr:.4f}")
print("=" * 30)
