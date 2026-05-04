import os
import shutil  # Added to help delete old folders

import pandas as pd


def convert_csv_to_yolo(csv_file, output_folder="single_class_labels"):
    # CLEANUP: Delete the folder if it exists so old files like '1.txt' disappear
    if os.path.exists(output_folder):
        shutil.rmtree(output_folder)
    os.makedirs(output_folder)

    # Load the data
    df = pd.read_csv(csv_file)

    # Process each image
    for image_name, group in df.groupby("image_name"):
        txt_filename = os.path.splitext(str(image_name))[0] + ".txt"
        txt_path = os.path.join(output_folder, txt_filename)

        yolo_lines = []
        for _, row in group.iterrows():
            class_id = 0

            x_center = (row["bbox_x"] + row["bbox_width"] / 2) / row["image_width"]
            y_center = (row["bbox_y"] + row["bbox_height"] / 2) / row["image_height"]
            w = row["bbox_width"] / row["image_width"]
            h = row["bbox_height"] / row["image_height"]

            yolo_lines.append(f"{class_id} {x_center:.6f} {y_center:.6f} {w:.6f} {h:.6f}")

        with open(txt_path, "w") as f:
            f.write("\n".join(yolo_lines))

    print(f"Success! Fresh files are in the folder: {os.path.abspath(output_folder)}")


# 2. Run the script
script_dir = os.path.dirname(os.path.abspath(__file__))
target_file = os.path.join(script_dir, "labels_my-project-name_2026-03-20-01-59-49.csv")

convert_csv_to_yolo(target_file)
