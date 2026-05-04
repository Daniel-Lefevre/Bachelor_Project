import cv2
import numpy as np
from skimage.filters import threshold_otsu
from skimage.morphology import dilation, disk, erosion


class ImageProcessing:
    def __init__(
        self,
    ):
        self.image = None

    def set_image(self, path: str) -> None:
        self.image = cv2.imread(path)

    def _create_hsv_image(self, image: np.ndarray) -> np.ndarray:
        return cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Remove the 'threshold: int' parameter, we don't need it anymore
    def _create_binary_image(self, gray_scale_image: np.ndarray, threshold: float) -> np.ndarray:
        # 1. Threshold to 255 (standard OpenCV 8-bit white)
        _, binary = cv2.threshold(gray_scale_image, threshold, 255, cv2.THRESH_BINARY)

        # 2. Find contours
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return np.zeros_like(binary)

        # 3. Identify the largest object
        largest_contour = max(contours, key=cv2.contourArea)

        # 4. Create a black mask and draw the largest object in WHITE (255)
        mask = np.zeros_like(binary)
        cv2.drawContours(mask, [largest_contour], -1, 255, thickness=cv2.FILLED)

        # 5. Optional: Convert back to 0/1 scale if your other functions expect it
        return (mask / 255).astype(np.uint8)

    def _opening_on_image(self, image: np.ndarray, disk_size: int) -> np.ndarray:
        SE = disk(disk_size)

        # opening
        return dilation(erosion(image, SE), SE)

    def _get_perimeter_and_area_of_biggest_object(self, image: np.ndarray) -> tuple[float, float] | None:

        # Get pixel size of the perimeter of the object and the area
        contours, _ = cv2.findContours(image * 255, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Check if there is no object
        if len(contours) == 0:
            return None

        # Pick largest contour
        cnt = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(cnt)
        perimeter_size = cv2.arcLength(cnt, True)

        # print(area, perimeter_size)

        return (area, perimeter_size)

    def _polsby_popper(self, area: float, perimeter_size: float) -> float:
        # print(f"AREA: {area}, perimeter: {perimeter_size}")
        if perimeter_size == 0:
            return 0
        else:
            return (4 * np.pi * area) / perimeter_size**2

    def _characterize_geometry(self, polsby_popper: float, area: float, pp_circle: float, pp_square: float, radius_min, radius_max, length_min, length_max) -> tuple[str, float] | None:
        if polsby_popper > pp_circle:
            radius = np.sqrt(area / np.pi)
            if not (radius >= radius_min and radius <= radius_max):
                return None
            return ("Circle", radius)

        elif polsby_popper >= pp_square and polsby_popper <= pp_circle:
            side_length = np.sqrt(area)
            if not (side_length >= length_min and side_length <= length_max):
                return None
            return ("Square", side_length)
        else:
            return None

    def _get_color_of_object(self, hue: np.ndarray, binary: np.ndarray, color_p: float) -> str | None:
        extracted_object = hue[binary != 0]
        pixel_sum = len(extracted_object)

        # Prevent division by zero if object is totally empty
        if pixel_sum == 0:
            return None

        red_sum, blue_sum, green_sum = 0, 0, 0

        for hue_value in extracted_object:
            if hue_value >= 105 and hue_value <= 120:
                blue_sum += 1
            elif hue_value >= 170 or hue_value <= 5:
                red_sum += 1
            elif hue_value >= 75 and hue_value <= 85:
                green_sum += 1

        if red_sum / pixel_sum >= color_p:
            return "Red"
        elif blue_sum / pixel_sum >= color_p:
            return "Blue"
        elif green_sum / pixel_sum >= color_p:
            return "Green"
        else:
            return None

    def classify(
        self,
        disk_size: int = 3,
        color_p: float = 0.7,
        pp_circle: float = 0.80,
        pp_square: float = 0.65,
        min_threshold: float = 120,
        radius_min: int = 37,
        radius_max: int = 45,
        length_min: int = 92,
        length_max: int = 108,
    ) -> str:

        # cropped_image = self._crop_and_warp(self.image)
        # cv2.imwrite("Experiments/Experiment1OpenSetRecognision/Processed_Images/cropped_image.jpg", cropped_image)
        hsv_image = self._create_hsv_image(self.image)
        hue, _, value = cv2.split(hsv_image)

        valid_pixels = value[value > 25]

        # Calculate Otsu specifically on the remaining pixels (peaks 2 and 3)
        if len(valid_pixels) > 0:
            threshold = threshold_otsu(valid_pixels)
        else:
            threshold = min_threshold  # Fallback

        # print(threshold)
        threshold = threshold if threshold > min_threshold else min_threshold

        binary_image = self._create_binary_image(value, threshold)

        # ERROR FIX 1: Removed the hardcoded 'disk_size = 3' here so it uses the Optuna parameter!
        closed_image = self._opening_on_image(binary_image, disk_size)

        stats = self._get_perimeter_and_area_of_biggest_object(closed_image)

        if stats is None:
            return "Unidentified_Object"

        area, perimeter_size = stats

        polsby_popper = self._polsby_popper(area, perimeter_size)

        # ERROR FIX 2 & 3: Actually pass the parameters into the sub-functions!
        geometry = self._characterize_geometry(polsby_popper, area, pp_circle, pp_square, radius_min, radius_max, length_min, length_max)
        object_color = self._get_color_of_object(hue, binary_image, color_p)

        # If the object does not resemble a circle or a square
        if geometry is None or object_color is None:
            return "Unidentified_Object"

        shape, length = geometry
        return object_color + "_" + shape


import os
import time

import matplotlib.pyplot as plt
import numpy as np
import optuna
import optuna.visualization.matplotlib as vis
from sklearn.metrics import confusion_matrix

test_label_count = {"Blue_Circle": 12, "Blue_Square": 12, "Red_Circle": 24, "Red_Square": 24, "Green_Circle": 30, "Green_Square": 22, "Unidentified_Object": 34}
validation_label_count = {"Blue_Circle": 28, "Blue_Square": 18, "Red_Circle": 32, "Red_Square": 21, "Green_Circle": 26, "Green_Square": 28, "Unidentified_Object": 36}
training_label_count = {"Blue_Circle": 130, "Blue_Square": 111, "Red_Circle": 137, "Red_Square": 92, "Green_Circle": 121, "Green_Square": 134, "Unidentified_Object": 232}


# 1. Pre-load all images into memory once
def load_all_data(script_dir):
    print("Loading all images into RAM... This takes a moment but saves hours.")
    dataset = []
    labels = ["Blue_Circle", "Blue_Square", "Red_Circle", "Red_Square", "Green_Circle", "Green_Square", "Unidentified_Object"]

    for label in labels:
        number_of_image = training_label_count[label]
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
    # disk_size = trial.suggest_int("disk_size", 0, 6)
    # color_p = trial.suggest_float("color_p", 0.50, 0.85)
    # pp_circle = trial.suggest_float("pp_circle", 0.6, 0.88)
    # pp_square = trial.suggest_float("pp_square", 0.40, 0.59)
    # min_threshold = trial.suggest_float("min_threshold", 20, 150)
    # radius_min = trial.suggest_float("radius_min", 0, 100)
    # radius_max = trial.suggest_float("radius_max", 0, 100)
    # length_min = trial.suggest_float("length_min", 0, 300)
    # length_max = trial.suggest_float("length_max", 0, 300)

    disk_size = trial.suggest_int("disk_size", 0, 6)
    color_p = trial.suggest_float("color_p", 0.5152833705387988 - 0.1, 0.5152833705387988 + 0.1)
    pp_circle = trial.suggest_float("pp_circle", 0.8134042006978266 - 0.1, 0.8134042006978266 + 0.1)
    pp_square = trial.suggest_float("pp_square", 0.5104332087819804 - 0.1, 0.5104332087819804 + 0.1)
    min_threshold = trial.suggest_float("min_threshold", 63.95668927164181 - 10, 63.95668927164181 + 10)
    radius_min = trial.suggest_float("radius_min", 0.7650100715719592 - 0.2, 0.7650100715719592 + 3)
    radius_max = trial.suggest_float("radius_max", 91.43501815318375 - 10, 91.43501815318375 + 10)
    length_min = trial.suggest_float("length_min", 3.8789964982240477 - 1, 3.8789964982240477 + 10)
    length_max = trial.suggest_float("length_max", 103.62946170488297 - 15, 103.62946170488297 + 15)

    image_processor = ImageProcessing()  # Add your crop points here

    correctly_labeled_images = 0
    total_images = len(dataset)

    # 3. Iterate entirely in memory (Lightning fast)
    for img, actual_label in dataset:
        # Assuming you update ImageProcessing to accept the raw array directly:
        image_processor.image = img

        return_label = image_processor.classify(disk_size, color_p, pp_circle, pp_square, min_threshold, radius_min, radius_max, length_min, length_max)

        if return_label == actual_label:
            correctly_labeled_images += 1

        if length_max < length_min + 5:
            correctly_labeled_images -= 1

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
    study.optimize(lambda trial: objective(trial, preloaded_dataset), n_trials=10, show_progress_bar=True, n_jobs=-1)

    print("\n--- OPTIMIZATION FINISHED ---")
    print(f"Best Accuracy Achieved: {round(study.best_value * 100, 2)}%")
    print("Best Parameters Found:")
    for key, value in study.best_params.items():
        print(f"  {key}: {value}")

    print("\n--- RUNNING ON TEST DATA ---")
    labels = ["Blue_Circle", "Blue_Square", "Red_Circle", "Red_Square", "Green_Circle", "Green_Square", "Unidentified_Object"]

    y_true = []
    y_pred = []

    correctly_labeled_images = 0
    falsy_labeled_images = 0
    total_processing_time = 0

    for label in labels:
        print(label)
        number_of_image = test_label_count[label]
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
