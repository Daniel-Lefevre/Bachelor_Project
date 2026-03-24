import os

import yaml
from ultralytics import YOLO

# 1. Setup Absolute Paths
base_path = os.path.dirname(os.path.abspath(__file__))
model_path = os.path.join(base_path, "best.pt")
yaml_path = os.path.join(base_path, "data.yaml")

# 2. DYNAMICALLY UPDATE YAML PATH
# This reads your yaml and forces the 'path' to be your current folder
with open(yaml_path, "r") as f:
    data_config = yaml.safe_load(f)

data_config["path"] = base_path  # Set the path to your current absolute directory

with open(yaml_path, "w") as f:
    yaml.dump(data_config, f)

# 3. Load your trained model
model = YOLO(model_path)

# 4. Run validation
print(f"--- Starting Validation on Test Set in {base_path} ---")
metrics = model.val(data=yaml_path, split="test", imgsz=640, save_json=True)

# 5. Print the results
print("\n" + "=" * 30)
print("RESULTS FOR TEST DATA:")
print(f"mAP@50:    {metrics.box.map50:.4f}")
print(f"Precision: {metrics.box.mp:.4f}")
print(f"Recall:    {metrics.box.mr:.4f}")
print("=" * 30)
