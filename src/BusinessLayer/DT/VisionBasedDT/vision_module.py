from __future__ import annotations

import os

import numpy as np
import torch
from typing import TYPE_CHECKING
import torch.nn as nn
import torch.nn.functional as F
from pyniryo import ObjectColor, ObjectShape
from torchvision import transforms
from torchvision.models import resnet18
from ultralytics import YOLO
import cv2
from types import SimpleNamespace

class VisionModule:
    def __init__(self):
        self.current_dir = os.path.dirname(os.path.abspath(__file__))
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.vision_progress_buffer = 0.1

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
        buffer = 10
        max_x = 639
        max_y = 479

        detector = self.robot_0_detection_module if robot_id == 0 else self.robot_1_detection_module

        results = detector(image, verbose=False)

        cropped_images_and_centers = []

        for result in results:
            boxes = result.boxes
            for box in boxes:
                x1, y1, x2, y2 = box.xyxy[0].int().tolist()
                h = abs(y2 - y1)
                w = abs(x2 - x1)
                y = y1
                x = x1

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

                cropped_img = image[y_lower:y_higher, x_lower:x_higher]
                cropped_images_and_centers.append((cropped_img, ((x2 - x1) / 2 + x1, (y2 - y1) / 2 + y1)))

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

    def _object_regression(self, center: tuple[float, float], robot_id: int) -> tuple[int, float]:
        robot_0_threshold = 281
        robot_1_threshold = 264

        x_pos, y_pos = center

        object_position_on_conveyor_cm = 0

        conveyor_id = None
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

        progress = (object_position_on_conveyor_cm - 0.09) / 0.48
        # progress = 0 if progress < 0 else progress
        progress = 1 if progress > 1 else progress
        return (conveyor_id, progress)

    def _convert_string_to_object(self, predicted_class_name) -> tuple[ObjectShape, ObjectColor] | str:
        if predicted_class_name == "Unknown":
            return predicted_class_name
        parts = predicted_class_name.split("_")
        shape = ObjectShape.SQUARE if parts[1] == "Square" else ObjectShape.CIRCLE
        color = ObjectColor.RED if parts[0] == "Red" else ObjectColor.BLUE if parts[0] == "Blue" else ObjectColor.GREEN
        return (shape, color)
    

    def _crop_out_conveyor(self, image, robot_id) -> np.ndarray:
        # Robot 0
        # if (robot_id == 0):
        #     points1 = np.array([[0, 0], [165, 0], [92, 216], [62, 479], [0, 479]], np.int32)
        #     points2 = np.array([[257, 0], [212, 202], [179, 479], [330, 479], [344, 0]], np.int32)
        #     points3 = np.array([[433, 0], [446, 310], [433, 479], [639, 479], [639, 0]], np.int32)
        # elif (robot_id == 1):
        #     points1 = np.array([[0, 0], [133, 0], [69, 227], [66, 479], [0, 479]], np.int32)
        #     points2 = np.array([[294, 0], [302, 479], [167, 479], [189, 52], [206, 0]], np.int32)
        #     points3 = np.array([[380, 0], [425, 335], [425, 479], [639, 479], [639, 0]], np.int32)

        if (robot_id == 0):
            points1 = np.array([[0, 0], [167, 0], [100, 203], [52, 479], [0, 479]], np.int32)
            points2 = np.array([[250, 0], [332, 0], [324, 374], [310, 479], [175, 479], [195, 239]], np.int32)
            points3 = np.array([[418, 0], [639, 0], [639, 479], [426, 479], [434, 252]], np.int32)

        elif (robot_id == 1):
            points1 = np.array([[0, 0], [149, 0], [100, 206], [80, 479], [0, 479]], np.int32)
            points2 = np.array([[231, 0], [320, 0], [314, 479], [189, 479], [211, 48]], np.int32)
            points3 = np.array([[407, 0], [639, 0], [639, 479], [443, 479], [435, 172]], np.int32)

        

        # 2. Reshape the points for OpenCV
        # fillPoly expects an array of "polygons", so we wrap our points in a list
        pts1 = points1.reshape((-1, 1, 2))
        pts2 = points2.reshape((-1, 1, 2))
        pts3 = points3.reshape((-1, 1, 2))

        # 3. Draw the polygon
        # (0, 0, 0) is BGR for Pitch Black
        processed_image = cv2.fillPoly(image, [pts1], color=(0, 0, 0))
        processed_image = cv2.fillPoly(processed_image, [pts2], color=(0, 0, 0))
        processed_image = cv2.fillPoly(processed_image, [pts3], color=(0, 0, 0))

        cv2.imwrite("Experiments/ExtraImages/testing.jpg", cv2.cvtColor(processed_image, cv2.COLOR_RGB2BGR))

        return processed_image

    #
    # Public functions
    #

    def process_image(self, image: np.ndarray, robot_id: int, opposite_robot_state_key: str) -> list[tuple[ObjectShape, ObjectColor | str] | int, int]:

        cropped_images_and_centers = self._detect_object(self._crop_out_conveyor(image, robot_id), robot_id)

        classified_objects_and_progress = []

        for cropped_object, center in cropped_images_and_centers:
            predicted_class_name = self._classify_object(cropped_object)
            if robot_id == 1:
                cv2.imwrite("cropped_circle.jpg", cropped_object)
                cv2.imwrite("full_image.jpg", image)
            conveyor_id, progress = self._object_regression(center, robot_id)
            # If objects on this robots conveyor belt, and an object is currently beeing placed
            if (conveyor_id == robot_id and opposite_robot_state_key in ["Standby_to_Place_Conveyor", "Place_Conveyor_to_Observation"]):
                # Check object isn't in the 'danger zone'
                progress_threshold = {0: 0.14324481705333333, 1: 0.10813231460442708}
                if progress < progress_threshold[robot_id]: continue
            # If object is on opposite conveyor belt, and an object i currently beeing picked up
            elif (conveyor_id != robot_id and opposite_robot_state_key in ["Pickup_Conveyor_to_Observation", "Workspace_Observation_to_Pickup_Conveyor"]):
                # Check object isn't in the 'danger zone'
                progress_threshold = {0: 0.7756544934440002, 1: 0.8160349564741668}
                if progress > progress_threshold[robot_id]: continue
                
            object_appearance = self._convert_string_to_object(predicted_class_name)
            classified_objects_and_progress.append((object_appearance, conveyor_id, progress))

        return classified_objects_and_progress

    def compare_image_with_DT(self, image: np.ndarray, robot_id: int, state_snapshot: SimpleNamespace) -> tuple[list[SimpleNamespace], list[SimpleNamespace]]:
        virtual_objects = state_snapshot.objects
        virtual_robots = state_snapshot.robots

        opposite_id = 0 if robot_id == 1 else 1
        opposite_robot_state_key = virtual_robots[opposite_id].state.key
        classified_objects_and_progress = self.process_image(image, robot_id, opposite_robot_state_key)
        
        # Check for unknows
        for object_appearance, conveyor_id, progress in classified_objects_and_progress:
            if object_appearance == "Unknown":
                if ((conveyor_id == 0 and robot_id == 1 and (progress < 0 or progress > 0.94))
                    or (conveyor_id == 1 and robot_id == 1 and progress > 0.94)
                    or progress < 0):
                    continue
                print(f"Robot {robot_id}: Unknown Object on conveyor {conveyor_id} with progress {progress}")

        classified_objects_and_progress = [(o,c,p) for o,c,p in classified_objects_and_progress if o != "Unknown"]        
        print(f"Robot {robot_id} {classified_objects_and_progress}")

        return_objects = []
        conveyor_cache_entries = []

        # Loop through the objects in the image and compare with the state of the DT
        for image_object in classified_objects_and_progress:
            (image_object_shape, image_object_color), image_object_conveyor_id, image_object_progress = image_object

            for virtual_object in virtual_objects:
                if (virtual_object.shape == image_object_shape and virtual_object.color == image_object_color):
                    # Check if object is on the correct conveyor
                    if virtual_object.state.id == image_object_conveyor_id:
                        # Push the progress to the conveyor cache
                        status = "Normal"
                        error = virtual_object.progress - image_object_progress
                        if (error < -self.vision_progress_buffer):
                            status = "Early"
                            return_object = SimpleNamespace(
                                color=virtual_object.color, 
                                shape=virtual_object.shape, 
                                error_correction = -error)
                            return_objects.append(return_object)
                        elif (error > self.vision_progress_buffer):
                            status = "Late"
                            return_object = SimpleNamespace(
                                color=virtual_object.color, 
                                shape=virtual_object.shape, 
                                error_correction = -error)
                            return_objects.append(return_object)

                        conveyor_cache_entry = SimpleNamespace(conveyor_id=image_object_conveyor_id, args = [virtual_object.shape, virtual_object.color, image_object_progress, status])
                        conveyor_cache_entries.append(conveyor_cache_entry)

                    # Objects was not on the correct conveyor
                    else:
                        pass

                # Object was not in image
                else:
                    pass

            # Object was not expected in the image
            else:
                pass
        
        return (return_objects, conveyor_cache_entries)
    
    

# Progress cutoff values

# if __name__ == "__main__":
#     vision_module = VisionModule()
#     current_dir = os.path.dirname(os.path.abspath(__file__))

#     for image_id in range(21, 25):
#         robot_id = 1 if image_id in [21, 22] else 0
#         print(f"Image {image_id}")
#         image_path = os.path.abspath(os.path.join(current_dir, "..", "..", "..", "..", "Experiments", "ExtraImages", f"{image_id}.jpg"))

#         # Load the image as BGR and convert to RGB
#         image_bgr = cv2.imread(image_path)
#         image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)

#         predictions = vision_module.process_image(image_rgb, robot_id)


#         print(predictions)

# Image 21
# {(<ObjectShape.CIRCLE: 'CIRCLE'>, <ObjectColor.RED: 'RED'>): (1, 0.8160349564741668)}
# Image 22
# {(<ObjectShape.CIRCLE: 'CIRCLE'>, <ObjectColor.RED: 'RED'>): (0, 0.10813231460442708)}
# Image 23
# {(<ObjectShape.CIRCLE: 'CIRCLE'>, <ObjectColor.RED: 'RED'>): (0, 0.7756544934440002)}
# Image 24
# {(<ObjectShape.CIRCLE: 'CIRCLE'>, <ObjectColor.RED: 'RED'>): (1, 0.14324481705333333)}


# if __name__ == "__main__":
#     vision_module = VisionModule()
#     current_dir = os.path.dirname(os.path.abspath(__file__))
#     TEST_POINTS_CONVEYOR_DISTANCE = (np.array([[26, 16.5, 43.5, 27, 22.5, 27, 51.5, 26, 40.5, 47.5], [21, 14.5, 37, 9.5, 52, 28.5, 45, 11, 33.5, 58]]) - 9) / 48
#     OBEJCTS = [
#         ["Red_Square", "Red_Circle", "Blue_Square", "Green_Square", "Red_Circle", "Blue_Circle", "Green_Square", "Green_Circle", "Blue_Square", "Green_Square"],
#         ["Blue_Circle", "Green_Circle", "Red_Square", "Green_Square", "Red_Circle", "Blue_Square", "Green_Square", "Red_Square", "Green_Circle", "Blue_Circle"],
#     ]

#     for robot_id in [0, 1]:
#         for conveyor_id in [0, 1]:
#             for image_id in range(1, 11):
#                 print(f"Image {image_id}")
#                 image_path = os.path.abspath(os.path.join(current_dir, "..", "..", "..", "..", "Experiments", "Experiment3Regression", "Test_Data", f"Conveyor_{conveyor_id}", f"Robot_{robot_id}", f"{image_id}.jpg"))

#                 # Load the image as BGR and convert to RGB
#                 image_bgr = cv2.imread(image_path)
#                 image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)

#                 correct_distance = TEST_POINTS_CONVEYOR_DISTANCE[conveyor_id][image_id - 1]
#                 correct_classifiaction = OBEJCTS[conveyor_id][image_id - 1]
#                 predictions = vision_module.process_image(image_rgb, robot_id)


#                 print(predictions)
#                 print(f"{correct_classifiaction} {correct_distance}")

