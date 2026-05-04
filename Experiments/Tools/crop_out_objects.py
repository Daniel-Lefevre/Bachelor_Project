import os

import cv2
import pandas as pd

labeled_data = [
    r"C:\Users\danie\OneDrive\Skrivebord\Rep\Bachelor_Project\Experiments\Experiment2ObjectFinder\dataset_robot_0\Training_Data\labels_my-project-name_2026-03-20-02-31-19.csv",
    r"C:\Users\danie\OneDrive\Skrivebord\Rep\Bachelor_Project\Experiments\Experiment2ObjectFinder\dataset_robot_1\Training_Data\labels_my-project-name_2026-03-20-02-55-07.csv",
]

object_counter = {"Green_Circle": 1, "Green_Square": 1, "Blue_Circle": 1, "Blue_Square": 1, "Red_Circle": 1, "Red_Square": 1, "Unknown": 1}

buffer = 10
max_x = 639
max_y = 479

for robot_index in range(2):
    df = pd.read_csv(labeled_data[robot_index])

    for index, row in df.iterrows():
        # Read the image
        img = cv2.imread(f"C:/Users/danie/OneDrive/Skrivebord/Rep/Bachelor_Project/Experiments/Experiment2ObjectFinder/dataset_robot_{robot_index}/Training_Data/images/{row['image_name']}")

        if img is None:
            print(f"Could not find image: {row['image_name']}")
            continue

        # Define the coordinates
        x, y, w, h = row["bbox_x"], row["bbox_y"], row["bbox_width"], row["bbox_height"]

        # Crop the image using NumPy slicing: img[y1:y2, x1:x2]
        y_lower = y - buffer
        if y_lower < 0:
            y_lower = 0

        y_higher = y + h + buffer
        if y_higher > max_y:
            y_higher = max_y

        x_lower = x - buffer
        if x_lower < 0:
            x_lower = 0

        x_higher = x + w + buffer
        if x_higher > max_x:
            x_higher = max_x

        crop = img[y_lower:y_higher, x_lower:x_higher]

        output_folder = f"C:/Users/danie/OneDrive/Skrivebord/Rep/Bachelor_Project/Experiments/Experiment4OpenSetRecognition/Training_Data/{row['label_name']}"
        filename = f"{object_counter[row['label_name']]}.jpg"
        object_counter[row["label_name"]] += 1
        full_path = os.path.join(output_folder, filename)
        cv2.imwrite(full_path, crop)
