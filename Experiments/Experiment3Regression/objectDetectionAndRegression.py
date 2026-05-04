import csv
from pathlib import Path

import numpy as np
from pysr import PySRRegressor
from ultralytics import YOLO

# FIX: Make BASE_DIR a Path object so .parent works properly
BASE_DIR = Path(__file__).resolve().parent


def get_objects(path, robot_index, data_set_type):
    # --- CONFIGURATION ---
    # path is already a Path object, so we can chain it smoothly
    MODEL_PATH = BASE_DIR.parent / "Experiment2ObjectFinder" / f"dataset_robot_{robot_index}" / "final_results_single" / "Robot_Object_Finder_Final" / "weights" / "best.pt"
    IMAGE_FOLDER = path
    OUTPUT_CSV = IMAGE_FOLDER / "object_centers.csv"

    # 1. Load the model (Cast to string as YOLO sometimes prefers strings for paths)
    model = YOLO(str(MODEL_PATH))

    # 2. Prepare the CSV file
    with open(OUTPUT_CSV, mode="w", newline="") as f:
        writer = csv.writer(f)
        print(f"--- Processing images in {IMAGE_FOLDER} ---")

        # 3. Loop through images
        if data_set_type == "Training_Data":
            number_of_images = 60
        else:
            number_of_images = 10

        for i in range(1, number_of_images + 1):
            img_name = f"{i}.jpg"
            img_path = IMAGE_FOLDER / img_name

            if not img_path.exists():
                print(f"Warning: {img_name} not found. Skipping.")
                continue

            # Run inference
            results = model.predict(source=str(img_path), conf=0.1, save=False, verbose=False)

            # 4. Extract data and Visualize
            for r in results:
                if len(r.boxes) > 0:
                    center_x = r.boxes.xywh[0][0].item()
                    center_y = r.boxes.xywh[0][1].item()

                    writer.writerow([center_x, center_y])
                else:
                    writer.writerow([0, 0])
                    print(f"Processed {img_name}: No object detected.")


def regression(data_path):
    csv_path = data_path / "object_centers.csv"
    data_array = np.loadtxt(str(csv_path), delimiter=",")

    n_samples = data_array.shape[0]

    Y = []
    for i in range(n_samples):
        Y.append(i // 3 * 0.025 + 0.09)  # in meters

    X = data_array
    y = np.array(Y)

    mask = np.any(X != 0, axis=1)

    X_clean = X[mask]
    y_clean = y[mask]

    # --- NEW: Isolate the second column (center_y) ---
    # Using 1:2 keeps the shape as (N, 1) rather than a 1D array of (N,)
    X_only_x1 = X_clean[:, 1:2]

    results_dir = data_path / "regression_results"
    results_dir.mkdir(parents=True, exist_ok=True)

    model = PySRRegressor(
        niterations=40,
        binary_operators=["+", "*", "-", "/"],
        model_selection="best",
        loss="loss(prediction, target) = (prediction - target)^2",
        output_directory=str(results_dir),  # Minor fix: point directly to the results_dir variable
    )

    # --- NEW: Pass variable_names during fit ---
    model.fit(X_only_x1, y_clean, variable_names=["x1"])

    print(model.equations_)


if __name__ == "__main__":
    CURRENT_DIR = Path(__file__).resolve().parent

    for dataset in ["Training_Data", "Test_Data"]:
        for conveyor_id in range(2):
            for robot_id in range(2):
                images_path = CURRENT_DIR / dataset / f"Conveyor_{conveyor_id}" / f"Robot_{robot_id}"

                # FIX: Passed 'dataset' into the function call
                # Start by detecting all objects
                get_objects(images_path, robot_id, dataset)

                if dataset == "Training_Data":
                    # Do regression on the objects
                    regression(images_path)
