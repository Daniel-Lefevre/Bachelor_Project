import os

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
from torchvision import transforms
from torchvision.models import resnet18
from ultralytics import YOLO


class VisionModule:
    def __init__(self):
        self.current_dir = os.path.dirname(os.path.abspath(__file__))
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        # Weights
        robot_0_detection_weights = os.path.abspath(os.path.join(self.current_dir, "..", "..", "..", "..", "resources", "best_param_detection_modular_robot_0.pt"))
        robot_1_detection_weights = os.path.abspath(os.path.join(self.current_dir, "..", "..", "..", "..", "resources", "best_param_detection_modular_robot_1.pt"))
        classification_weights = os.path.abspath(os.path.join(self.current_dir, "..", "..", "..", "..", "resources", "best_param_classification_modular.pth"))

        # Models
        self.robot_0_detection_module = YOLO(robot_0_detection_weights)
        self.robot_1_detection_module = YOLO(robot_1_detection_weights)
        self.classification_module = self._load_resnet(classification_weights)

    #
    # Private functions
    #

    def _load_resnet(self, classification_weights: str) -> nn.Module:
        model = resnet18()
        num_features = model.fc.in_features

        model.fc = nn.Sequential(nn.Dropout(0.5), nn.Linear(num_features, 7))

        model.load_state_dict(torch.load(classification_weights, map_location=self.device))

        model = model.to(self.device)
        model.eval()
        return model

    def _detect_object(self, image: np.ndarray, robot_id: int) -> list[tuple[np.ndarray, tuple[float, float]]]:

        detector = self.robot_0_detection_module if robot_id == 0 else self.robot_1_detection_module

        results = detector(image, verbose=False)

        cropped_images_and_centers = []

        for result in results:
            boxes = result.boxes
            for box in boxes:
                x1, y1, x2, y2 = box.xyxy[0].int().tolist()
                cropped_img = image[y1:y2, x1:x2]
                cropped_images_and_centers.append((cropped_img, ((x2 - x1) / 2 + x1), ((y2 - y1) / 2 + y1)))

        return cropped_images_and_centers

    def _classify_object(self, cropped_image: np.ndarray) -> str:
        class_names = ["Blue_Circle", "Blue_Square", "Green_Circle", "Green_Square", "Red_Circle", "Red_Square", "Unknown"]

        normalize = transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
        transform = transforms.Compose(
            [
                transforms.ToPILImage(),
                transforms.Resize(256),
                transforms.CenterCrop(224),
                transforms.ToTensor(),
                normalize,
            ]
        )

        input_tensor = transform(cropped_image).unsqueeze(0).to(self.device)

        # Make prediction
        with torch.no_grad():
            output = self.classification_module(input_tensor)

        probabilities = F.softmax(output[0], dim=0)
        _, predicted_idx = torch.max(probabilities, 0)

        predicted_class = class_names[predicted_idx.item()]

        return predicted_class

    def _object_regression(self, center: tuple[float, float], robot_id: int) -> float:
        robot_0_threshold = 292
        robot_1_threshold = 241

        x_pos, y_pos = center

        object_position_on_conveyor_cm = 0

        if robot_id == 0:
            conveyor_id = 1 if x_pos < robot_0_threshold else 0
            if conveyor_id == 0:
                object_position_on_conveyor_cm = (((y_pos * -1.0953979e-6) + 0.00166411) * y_pos) - -0.06900252
            elif conveyor_id == 1:
                object_position_on_conveyor_cm = (y_pos * ((y_pos * 9.2536857e-7) + -0.0015947099)) + 0.6181552
        elif robot_id == 1:
            conveyor_id = 0 if x_pos < robot_1_threshold else 1
            if conveyor_id == 0:
                object_position_on_conveyor_cm = 0.64444643 - (y_pos * (0.0017557096 - (y_pos * 1.1290276e-6)))
            elif conveyor_id == 1:
                object_position_on_conveyor_cm = (((y_pos * -1.5215395e-6) + 0.0018650172) * y_pos) - -0.07937143

        progress = (object_position_on_conveyor_cm - 9) / 48
        progress = 0 if progress < 0 else progress
        progress = 1 if progress > 1 else progress
        return progress

    #
    # Public functions
    #

    def process_image(self, image: np.ndarray, robot_id: int) -> list[str, float]:
        cropped_images_and_centers = self._detect_object(image, robot_id)

        classified_objects_and_progress = []

        for cropped_object, center in cropped_images_and_centers:
            predicted_class_name = self._classify_object(cropped_object)
            progress = self._object_regression(center, robot_id)
            classified_objects_and_progress.append((predicted_class_name, progress))

        return classified_objects_and_progress
