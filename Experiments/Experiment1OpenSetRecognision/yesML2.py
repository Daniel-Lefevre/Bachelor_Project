import os
import random
import time

import cv2
import matplotlib.colors as mcolors
import matplotlib.pyplot as plt
import numpy as np
import torch
import torch.nn as nn
from PIL import Image
from sklearn.decomposition import PCA
from sklearn.discriminant_analysis import LinearDiscriminantAnalysis as LDA
from sklearn.pipeline import Pipeline
from sklearn.preprocessing import Normalizer
from torchvision import transforms
from torchvision.models import ResNet18_Weights, resnet18

script_dir = os.path.dirname(os.path.abspath(__file__))


# Set a fixed seed
SEED = 42


def make_deterministic(seed):
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    torch.cuda.manual_seed_all(seed)
    # This forces the GPU to use slower, but deterministic algorithms
    torch.backends.cudnn.deterministic = True
    torch.backends.cudnn.benchmark = False
    # If using newer versions of torch
    os.environ["CUBLAS_WORKSPACE_CONFIG"] = ":4096:8"


make_deterministic(SEED)


class ImageProcessingML:
    def __init__(self):
        self.dimension = 6
        self.augment_factor = 4
        self.euclidean_threshold_multiplication = 1.75
        self.images_per_label = 20
        self.euclidean_minimum_distances = []

        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        print(f"Using device: {self.device}")

        self.weights = ResNet18_Weights.IMAGENET1K_V1
        self.model = resnet18(weights=self.weights)
        self.model.eval()
        self.src_points_robot_0 = np.array([[154, 114], [424, 83], [468, 361], [197, 401]], dtype=np.float32)
        self.src_points_robot_1 = np.array([[112, 100], [356, 49], [423, 327], [175, 371]], dtype=np.float32)
        self.preprocess = transforms.Compose(
            [
                transforms.Resize((224, 224)),  # resize image directly to 224x224
                transforms.ToTensor(),
                transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),  # Standard for NetImage
            ]
        )
        self.augment_transform = transforms.Compose(
            [
                transforms.RandomRotation(degrees=15),
                transforms.RandomHorizontalFlip(p=0.5),
                transforms.RandomVerticalFlip(p=0.2),
                transforms.ColorJitter(brightness=0.2, contrast=0.2, saturation=0.2, hue=0.05),
                transforms.Resize((224, 224)),
                transforms.ToTensor(),
                transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
            ]
        )
        self.feature_extractor = nn.Sequential(*list(self.model.children())[:-1]).to(self.device)

        # --- THE FIX: L2 Normalization + PCA + LDA ---
        self.pipeline = Pipeline([("scaler", Normalizer(norm="l2")), ("pca", PCA(n_components=50)), ("lda", LDA(n_components=self.dimension, solver="svd"))])

        self.labels = ["Blue_Circle", "Blue_Square", "Red_Circle", "Red_Square", "Green_Circle", "Green_Square", "No_Object"]
        self._train_model()

    def format_image(self, image_bgr, crop_id):
        cropped_bgr = self._crop_and_warp(image_bgr, crop_id)
        image_rgb = cv2.cvtColor(cropped_bgr, cv2.COLOR_BGR2RGB)  # RBB
        return Image.fromarray(image_rgb)  # Convert to PIL image

    def _crop_and_warp(self, image: np.ndarray, crop_id) -> np.ndarray:
        # Define the box to crop to
        if crop_id == 0:
            crop_points = self.src_points_robot_0
        else:
            crop_points = self.src_points_robot_1

        top_width = np.linalg.norm(crop_points[1] - crop_points[0])
        bottom_width = np.linalg.norm(crop_points[2] - crop_points[3])
        left_height = np.linalg.norm(crop_points[3] - crop_points[0])
        right_height = np.linalg.norm(crop_points[2] - crop_points[1])

        # Define height and width of the square to crop
        width_square = int(max(top_width, bottom_width))
        height_square = int(max(left_height, right_height))

        # Determine dimensions of the output rectangle
        data_points = np.float32(
            [
                [0, 0],  # top-left
                [width_square, 0],  # top-right
                [width_square, height_square],  # bottom-right
                [0, height_square],  # bottom-left
            ]
        )

        # Calculate the Transformation Matrix
        M = cv2.getPerspectiveTransform(crop_points, data_points)

        # Apply and return the warp
        return cv2.warpPerspective(image, M, (width_square, height_square))

    def _train_model(self):
        self.training_features = []
        self.training_labels = []

        # get features from images and insert in high dimensional space
        for label_index, label in enumerate(self.labels):
            print(f"Training - {label}")
            for i in range(self.images_per_label):
                path = os.path.join(script_dir, "Training_Data", label, f"{i + 1}.jpg")
                image_bgr = cv2.imread(path)
                image = self.format_image(image_bgr, 0 if i < 10 else 1)

                normal_tensor = self.preprocess(image).unsqueeze(0).to(self.device)
                with torch.no_grad():
                    normal_features = self.feature_extractor(normal_tensor).squeeze().numpy()
                self.training_features.append(normal_features)
                self.training_labels.append(label_index)

                # 2. Process Slightly Rotated Image
                for j in range(self.augment_factor - 1):
                    rotated_tensor = self.augment_transform(image).unsqueeze(0).to(self.device)
                    with torch.no_grad():
                        rotated_features = self.feature_extractor(rotated_tensor).squeeze().numpy()
                    self.training_features.append(rotated_features)
                    self.training_labels.append(label_index)

        self.lda_data = self.pipeline.fit_transform(self.training_features, self.training_labels)
        print(self.lda_data.shape)

        samples_per_class = 20 * self.augment_factor
        self.means = []
        self.distances = []

        for label_index in range(len(self.labels)):
            start_index = samples_per_class * label_index
            end_index = samples_per_class * (label_index + 1)
            label_points = self.lda_data[start_index:end_index, :]

            mean = np.mean(label_points, axis=0)
            self.means.append(mean)

            euclidean_distances = []
            for x in label_points:
                euclidean_distance = np.linalg.norm(x - mean)
                euclidean_distances.append(euclidean_distance)

            best_index = np.argmax(euclidean_distances)
            max_euclidean = euclidean_distances[best_index] * self.euclidean_threshold_multiplication

            self.euclidean_minimum_distances.append(max_euclidean)
            self.distances.append(euclidean_distances)

    def classify_image(self, image_bgr, crop_id):
        # get features form image
        image = self.format_image(image_bgr, crop_id)
        features = self._get_features(image)
        features_2d = features.reshape(1, -1)

        # Do LDA
        lda = self.pipeline.transform(features_2d)
        lda_vector = lda.squeeze()
        euclidean_distance_threshold_percentage = []

        # Calculate eucledian
        for label_index, label in enumerate(self.labels):
            mean = self.means[label_index]
            euclidean_distance = np.linalg.norm(mean - lda_vector)
            euclidean_distance_threshold_percentage.append(euclidean_distance / self.euclidean_minimum_distances[label_index])

        best_index = np.argmin(euclidean_distance_threshold_percentage)
        min_euclidean_percentage = euclidean_distance_threshold_percentage[best_index]

        if min_euclidean_percentage <= 1:
            return self.labels[best_index]

        return "Unidentified_Object"

    def _get_features(self, image):
        input_tensor = self.preprocess(image).unsqueeze(0).to(self.device)

        with torch.no_grad():
            features = self.feature_extractor(input_tensor)

        features = features.squeeze().cpu().numpy()
        return features

    # Fucntions for plots
    def plot_3d(self):
        print("Plotting")
        label_list = []
        for label in self.labels:
            for i in range(self.images_per_label * self.augment_factor):
                label_list.append(label)

        unique_labels = self.labels
        colors = list(mcolors.TABLEAU_COLORS.keys())  # or any color palette
        label_color_map = {label: colors[i % len(colors)] for i, label in enumerate(unique_labels)}

        sample_colors = [label_color_map[lbl] for lbl in label_list]

        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection="3d")

        ax.scatter(
            self.lda_data[:, 0],  # PC1
            self.lda_data[:, 1],  # PC2
            self.lda_data[:, 2],  # PC3
            c=sample_colors,
            s=50,  # marker size
            alpha=0.8,
        )

        # Optional: add legend
        for label in unique_labels:
            ax.scatter([], [], [], c=label_color_map[label], label=label.replace("_", " "))
        ax.legend()

        ax.set_xlabel("LD: 1")
        ax.set_ylabel("LD: 2")
        ax.set_zlabel("LD: 3")
        ax.set_title("3D LDA of Image Features")
        plt.show()

    def plot_lda_variance(self):
        print("Calculating Full LDA for Variance Plot...")

        # Update to use the new L2 Normalizer and PCA to match your pipeline
        scaler = Normalizer(norm="l2")
        pca = PCA(n_components=50)

        scaled_features = scaler.fit_transform(self.training_features)
        pca_features = pca.fit_transform(scaled_features)

        # Fit a full LDA (no n_components restriction) to see all variance
        lda_full = LDA(solver="svd")
        lda_full.fit(pca_features, self.training_labels)

        # Calculate cumulative explained variance
        cumulative_variance = np.cumsum(lda_full.explained_variance_ratio_)

        # Plot the Scree Plot
        plt.figure(figsize=(8, 5))
        plt.plot(range(1, len(cumulative_variance) + 1), cumulative_variance, marker="o", linestyle="--")
        plt.xlabel("Number of Components")
        plt.ylabel("Cumulative Explained Variance")

        # Adjusted the x-axis limit to match the maximum possible LDA components (Classes - 1)
        plt.xlim(0, len(self.labels))
        plt.xticks(range(1, len(self.labels)))

        plt.title("LDA Explained Variance (Scree Plot)")
        plt.grid(True)
        plt.show()

    def plot_euclidean_distances(self):
        print("Plotting eucledian Distances...")
        plt.figure(figsize=(10, 6))

        # We iterate over the labels and their corresponding distances
        for i, label in enumerate(self.labels):
            # Get the list of distances for the current label
            label_distances = self.distances[i]

            # Create a list of the label string to serve as our y-coordinates
            y_values = [label] * len(label_distances)

            # Plot the distances on the x-axis and labels on the y-axis
            plt.scatter(label_distances, y_values, alpha=0.7, edgecolors="w", s=60)

        plt.xlabel("Eucledian Distance")
        plt.ylabel("Classes (Labels)")
        plt.title("Eucledian Distances per Class")
        plt.grid(True, axis="x", linestyle="--", alpha=0.7)

        # Reverses the y-axis so the first label is at the top
        plt.gca().invert_yaxis()

        plt.tight_layout()
        plt.show()

    def plot_train_test_1d_distances(self):
        print("Plotting 1D Train/Test Distances...")
        fig, ax = plt.subplots(figsize=(10, 6))

        # Iterate over known classes only
        for i, label in enumerate(self.labels):
            # --- Format the label to remove underscores ---
            display_label = label.replace("_", " ")

            # --- 1. Plot Training Distances (Red) ---
            train_dists = self.distances[i]
            y_values_train = [display_label] * len(train_dists)

            ax.scatter(train_dists, y_values_train, color="#d32f2f", alpha=0.3, s=25, edgecolors="none", zorder=3, label="Training" if i == 0 else "")

            # --- 2. Calculate & Plot Testing Distances (Blue) ---
            test_dists = []
            number_of_images = 20

            for j in range(number_of_images):
                path = os.path.join(script_dir, "Test_Data", label, f"{j + 1}.jpg")
                if not os.path.exists(path):
                    continue

                image_bgr = cv2.imread(path)
                image = self.format_image(image_bgr, 0 if j < number_of_images / 2 else 1)

                features = self._get_features(image).reshape(1, -1)
                lda_vector = self.pipeline.transform(features).squeeze()

                mean = self.means[i]
                euclidean_dist = np.linalg.norm(mean - lda_vector)
                test_dists.append(euclidean_dist)

            y_values_test = [display_label] * len(test_dists)

            ax.scatter(test_dists, y_values_test, color="#0288d1", alpha=0.6, s=30, edgecolors="none", zorder=4, label="Testing" if i == 0 else "")

        # --- 3. Aesthetics & Formatting ---
        ax.set_xlabel("Euclidean Distance to Centroid", fontsize=11, fontweight="bold")

        # --- Force the X-axis to start at 0 ---
        ax.set_xlim(left=0)

        # Style the Y-axis labels
        ax.tick_params(axis="y", labelsize=11, length=0)
        for label in ax.get_yticklabels():
            label.set_fontweight("bold")

        # Add subtle horizontal lines to visually separate the classes
        ax.grid(True, axis="y", linestyle="-", color="#e0e0e0", linewidth=1, zorder=0)

        # Clean up the bounding box
        ax.spines["top"].set_visible(False)
        ax.spines["right"].set_visible(False)
        ax.spines["left"].set_visible(False)
        ax.spines["bottom"].set_color("#cccccc")

        # --- Legend in a solid white box ---
        legend = ax.legend(frameon=True, facecolor="white", framealpha=1.0, edgecolor="#cccccc", loc="upper right", fontsize=11)
        for handle in legend.legend_handles:
            handle.set_alpha(1.0)
            handle.set_sizes([60])

        ax.invert_yaxis()  # Put the first class at the top
        plt.tight_layout()
        plt.show()


if __name__ == "__main__":
    image_processor = ImageProcessingML()
    # image_processor.plot_euclidean_distances()
    # image_processor.plot_3d()
    # image_processor.plot_lda_variance()
    image_processor.plot_train_test_1d_distances()
    labels = ["Blue_Circle", "Blue_Square", "Red_Circle", "Red_Square", "Green_Circle", "Green_Square", "No_Object", "Unidentified_Object"]

    correctly_labeled_images = 0
    falsy_labeled_images = 0
    total_validation_time = 0

    for label in labels:
        number_of_images = 36 if label == "Unidentified_Object" else 20
        for i in range(number_of_images):
            path = os.path.join(script_dir, "Test_Data", label, f"{i + 1}.jpg")
            if not os.path.exists(path):
                continue
            image_bgr = cv2.imread(path)

            start_time = time.perf_counter_ns()
            return_label = image_processor.classify_image(image_bgr, 0 if i < number_of_images / 2 else 1)
            total_validation_time += time.perf_counter_ns() - start_time

            if return_label == label:
                correctly_labeled_images += 1
            else:
                falsy_labeled_images += 1
                print(label + " classified as " + return_label + " - " + str(i + 1))

    test_point_sum = correctly_labeled_images + falsy_labeled_images
    print(f"{test_point_sum} test points were labeled.")
    print(f"{correctly_labeled_images} ({round(correctly_labeled_images / test_point_sum, 2)}%) were labeled corretly.")
    print(f"{falsy_labeled_images} ({round(falsy_labeled_images / test_point_sum, 2)}%) were labeled incorretly.")
    if test_point_sum > 0:
        print(f"Time per image classification in validation: {(total_validation_time / test_point_sum) * 10 ** (-6)} ms")
