import os

import yaml
from ultralytics import YOLO

# 1. Setup Absolute Paths (Works for both Colab and Local .py files)
try:
    # This works if running as a .py file
    base_path = os.path.dirname(os.path.abspath(__file__))
except NameError:
    # This works if running in a Google Colab / Jupyter cell
    # Ensure this matches your folder name in the Colab sidebar
    base_path = "/content/dataset_robot_0/dataset_robot_0"

yaml_path = os.path.join(base_path, "data.yaml")
# Path to the best hyperparameters found during your tuning session
best_hyp_path = os.path.join(base_path, "best_hyperparameters.yaml")

# 2. DYNAMICALLY UPDATE YAML PATH
# Forces YOLO to look for images relative to the current base_path
if os.path.exists(yaml_path):
    with open(yaml_path, "r") as f:
        data_config = yaml.safe_load(f)

    data_config["path"] = base_path

    with open(yaml_path, "w") as f:
        yaml.dump(data_config, f)
    print(f"✅ YAML 'path' updated to: {base_path}")
else:
    print(f"❌ Error: {yaml_path} not found!")

# 3. Load the fresh YOLOv8 Nano model
model = YOLO("yolov8n.pt")

# 4. Final Training Run
# We use freeze=0 (default) to allow full specialization of the model
print("--- Starting Final Production Training ---")
results = model.train(
    data=yaml_path,
    epochs=150,  # High limit; patience will stop it when optimal
    imgsz=640,  # Standard high-res for YOLOv8
    batch=16,  # Good balance for T4 GPU memory
    cfg=best_hyp_path,  # Inject the 'winning' settings from tuning
    patience=5,  # Stop if no improvement for 30 epochs (prevents overfitting)
    project=os.path.join(base_path, "final_results"),
    name="Robot_Object_Finder_Final",
    exist_ok=True,  # Overwrite if folder exists
)

# 5. Final Evaluation on UNSEEN Test Data
print("\n--- FINAL EVALUATION ON TEST DATA ---")
# Load the best weights produced by this specific run
final_model_path = os.path.join(base_path, "final_results/Robot_Object_Finder_Final/weights/best.pt")
final_model = YOLO(final_model_path)

metrics = final_model.val(split="test")

print("\n" + "=" * 30)
print("OFFICIAL THESIS RESULTS:")
print(f"mAP@50:    {metrics.box.map50:.4f}")
print(f"Precision: {metrics.box.mp:.4f}")
print(f"Recall:    {metrics.box.mr:.4f}")
print("=" * 30)
