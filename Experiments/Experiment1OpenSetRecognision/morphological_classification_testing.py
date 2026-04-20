import os
import time

import cv2
import matplotlib.pyplot as plt
import numpy as np
import optuna
import optuna.visualization.matplotlib as vis
from noML_testing import ImageProcessing
from sklearn.metrics import confusion_matrix


# 1. Pre-load all images into memory once
def load_all_data(script_dir):
    print("Loading all images into RAM... This takes a moment but saves hours.")
    dataset = []
    labels = ["Blue_Circle", "Blue_Square", "Red_Circle", "Red_Square", "Green_Circle", "Green_Square", "No_Object", "Unidentified_Object"]

    for label in labels:
        number_of_image = 206 if label == "Unidentified_Object" else 166
        for i in range(number_of_image):
            image_path = os.path.join(script_dir, "Training_Data", label, f"{i + 1}.jpg")
            if os.path.exists(image_path):
                img = cv2.imread(image_path)
                if img is not None:
                    dataset.append((img, label))
    print(f"Loaded {len(dataset)} images into memory.")
    return dataset


# 2. Pass the pre-loaded dataset to the objective function
def objective(trial, dataset):
    disk_size = trial.suggest_int("disk_size", 1, 6)
    color_p = trial.suggest_float("color_p", 0.55, 0.85)
    pp_circle = trial.suggest_float("pp_circle", 0.75, 0.88)
    pp_square = trial.suggest_float("pp_square", 0.55, 0.74)
    min_threshold = trial.suggest_float("min_threshold", 50, 120)
    radius_min = trial.suggest_float("radius_min", 30, 43)
    radius_max = trial.suggest_float("radius_max", 38, 55)
    length_min = trial.suggest_float("length_min", 85, 99)
    length_max = trial.suggest_float("length_max", 100, 115)
    no_object_threshold = trial.suggest_float("no_object_threshold", 10, 1000)

    image_processor = ImageProcessing()  # Add your crop points here

    correctly_labeled_images = 0
    total_images = len(dataset)

    # 3. Iterate entirely in memory (Lightning fast)
    for img, actual_label in dataset:
        # Assuming you update ImageProcessing to accept the raw array directly:
        image_processor.image = img

        return_label = image_processor.classify(disk_size, color_p, pp_circle, pp_square, min_threshold, radius_min, radius_max, length_min, length_max, no_object_threshold)

        if return_label == actual_label:
            correctly_labeled_images += 1

    return correctly_labeled_images / total_images if total_images > 0 else 0


if __name__ == "__main__":
    image_processor = ImageProcessing()
    script_dir = os.path.dirname(os.path.abspath(__file__))

    # Load data ONCE before trials start
    preloaded_dataset = load_all_data(script_dir)

    optuna.logging.set_verbosity(optuna.logging.WARNING)
    print("Starting Bayesian Optimization with expanded parameters...")

    study = optuna.create_study(direction="maximize")

    # Use a lambda to pass the dataset into the objective function
    study.optimize(lambda trial: objective(trial, preloaded_dataset), n_trials=100, show_progress_bar=True, n_jobs=-1)

    print("\n--- OPTIMIZATION FINISHED ---")
    print(f"Best Accuracy Achieved: {round(study.best_value * 100, 2)}%")
    print("Best Parameters Found:")
    for key, value in study.best_params.items():
        print(f"  {key}: {value}")

    print("\n--- RUNNING ON TEST DATA ---")
    labels = ["Blue_Circle", "Blue_Square", "Red_Circle", "Red_Square", "Green_Circle", "Green_Square", "No_Object", "Unidentified_Object"]
    # labels = ["No_Object"]

    y_true = []
    y_pred = []

    correctly_labeled_images = 0
    falsy_labeled_images = 0
    total_processing_time = 0

    for label in labels:
        print(label)
        number_of_image = 30 if label == "Unidentified_Object" else 20
        for i in range(number_of_image):
            image_path = os.path.join(script_dir, "Test_Data", label, f"{i + 1}.jpg")

            image_processor.set_image(image_path)
            # Pass the optimized parameters here!
            start_time = time.perf_counter()
            return_label = image_processor.classify(**study.best_params)
            end_time = time.perf_counter()
            total_processing_time += end_time - start_time

            y_true.append(label)
            y_pred.append(return_label)

            if return_label == label:
                correctly_labeled_images += 1
            else:
                falsy_labeled_images += 1
                print(f"  [ERROR] {label} classified as {return_label} - Image {i + 1}")

    test_point_sum = correctly_labeled_images + falsy_labeled_images
    if test_point_sum > 0:
        avg_time = total_processing_time / test_point_sum
        print(f"\n{test_point_sum} test points were labeled.")
        print(f"{correctly_labeled_images} ({round((correctly_labeled_images / test_point_sum) * 100, 2)}%) were labeled correctly.")
        print(f"{falsy_labeled_images} ({round((falsy_labeled_images / test_point_sum) * 100, 2)}%) were labeled incorrectly.")
        print(f"Average classification time: {round(avg_time * 1000, 2)} ms per image")  # Converted to milliseconds
    else:
        print("No test images were found.")

    # Create the directory if it doesn't exist

    folder_path = os.path.join(script_dir, "morphological_classification_figures")
    if not os.path.exists(folder_path):
        os.makedirs(folder_path)
        print(f"\nCreated folder: {folder_path}")

    print("Generating high-quality thesis figures using Matplotlib...")

    # 1. Optimization History
    vis.plot_optimization_history(study)
    plt.tight_layout()
    plt.savefig(os.path.join(folder_path, "optimization_history.png"), dpi=300)
    plt.close()

    # 2. Parameter Importances
    vis.plot_param_importances(study)
    plt.tight_layout()
    plt.savefig(os.path.join(folder_path, "param_importances.png"), dpi=300)
    plt.close()

    # 3. Parallel Coordinate Plot
    vis.plot_parallel_coordinate(study)
    plt.tight_layout()
    plt.savefig(os.path.join(folder_path, "parallel_coordinates.png"), dpi=300)
    plt.close()

    # 4. Slice Plot
    vis.plot_slice(study)
    plt.tight_layout()
    plt.savefig(os.path.join(folder_path, "parameter_slices.png"), dpi=300)
    plt.close()

    # 5. Export the 'Grid-like' data to CSV for your tables
    df = study.trials_dataframe()
    df.to_csv(os.path.join(folder_path, "morph_optuna_results_table.csv"), index=False)

    # Confusion Matrix
    display_labels = ["Unknown", "Green_Circle", "Green_Square", "Blue_Circle", "Blue_Square", "Red_Circle", "Red_Square", "background"]
    label_map = {
        "Unidentified_Object": "Unknown",
        "No_Object": "background",
        "Blue_Circle": "Blue_Circle",
        "Blue_Square": "Blue_Square",
        "Red_Circle": "Red_Circle",
        "Red_Square": "Red_Square",
        "Green_Circle": "Green_Circle",
        "Green_Square": "Green_Square",
    }

    # Map the results
    y_true_mapped = [label_map.get(lbl, lbl) for lbl in y_true]
    y_pred_mapped = [label_map.get(lbl, lbl) for lbl in y_pred]

    # 2. Calculate matrix
    cm = confusion_matrix(y_true_mapped, y_pred_mapped, labels=display_labels)

    # 3. Plotting
    fig, ax = plt.subplots(figsize=(10, 8), dpi=300)
    im = ax.imshow(cm, interpolation="nearest", cmap=plt.cm.Blues)
    ax.figure.colorbar(im, ax=ax)

    # Styles
    ax.set(xticks=np.arange(cm.shape[1]), yticks=np.arange(cm.shape[0]), xticklabels=display_labels, yticklabels=display_labels, title="Confusion Matrix", ylabel="Predicted", xlabel="True")
    ax.grid(False)

    plt.setp(ax.get_xticklabels(), rotation=90, ha="right", rotation_mode="anchor")

    # Add text annotations
    thresh = cm.max() / 2.0
    for i in range(cm.shape[0]):
        for j in range(cm.shape[1]):
            if cm[i, j] > 0:
                ax.text(j, i, format(cm[i, j], "d"), ha="center", va="center", color="white" if cm[i, j] > thresh else "black")

    fig.tight_layout(pad=2.0)
    plt.savefig(os.path.join(folder_path, "confusion_matrix.png"))
    plt.close()

    print(f"Done! High-res figures and results table are in /{folder_path}")
