import os
import time

from ultralytics import YOLO

# 1. Load the trained model
base_path = os.path.dirname(os.path.abspath(__file__))
dataset = "dataset_robot_1"

final_model_path = os.path.join(base_path, f"{dataset}/final_results_single/Robot_Object_Finder_Final/weights/best.pt")
model = YOLO(final_model_path)


data_yaml_path = os.path.join(base_path, dataset, "data_single.yaml")

start_time = time.perf_counter()
# 2. Run Validation to get predictions on the test set
# we use save_json=True or simply iterate through results
results = model.val(data=data_yaml_path, split="test", plots=True)

end_time = time.perf_counter()

total_duration = end_time - start_time
num_images = 30  # Number of images processed
avg_time_per_image = total_duration / num_images

# 3. Access the Confusion Matrix correctly
# We use the validator's internal confusion matrix which is populated during val
confusion_matrix = results.confusion_matrix
matrix = confusion_matrix.matrix

# In YOLOv8/v11, index -1 is 'background'
# We slice to exclude the background row and column
class_matrix = matrix[:-1, :-1]

correct_classifications = class_matrix.diagonal().sum()
total_objects_found = class_matrix.sum()

# In Ultralytics ConfusionMatrix:
# - Rows are Ground Truth
# - Columns are Predictions
# - The last row/col is often "Background" (FP/FN)
# We want only the object-to-object classification part.

# Remove the background category (usually the last index) to focus on classification
class_matrix = matrix[:-1, :-1]

correct_classifications = class_matrix.diagonal().sum()
total_objects_found = class_matrix.sum()

if total_objects_found > 0:
    accuracy = correct_classifications / total_objects_found
else:
    accuracy = 0

map50_95 = results.results_dict["metrics/mAP50-95(B)"]
map50 = results.results_dict["metrics/mAP50(B)"]


print("\n" + "=" * 30)
print(f"Total Objects Detected:  {int(total_objects_found)}")
print(f"Correctly Classified:   {int(correct_classifications)}")
print(f"Classification Accuracy: {accuracy:.4f}")
print(f"Avg Time per Image:      {avg_time_per_image:.4f} seconds")
print(f"mAP50-95:                {map50_95:.4f}")
print(f"mAP50:                   {map50:.4f}")
print("=" * 30)
