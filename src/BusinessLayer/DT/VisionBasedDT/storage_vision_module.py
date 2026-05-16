from __future__ import annotations

import os
from types import SimpleNamespace

import cv2
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
from pyniryo import ObjectColor, ObjectShape
from torchvision import transforms
from torchvision.models import resnet18
from ultralytics import YOLO

from resources.environment import configuration


class StorageVisionModule:
    def __init__(self):
        self.current_dir = os.path.dirname(os.path.abspath(__file__))
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        # Weights
        detection_weights = os.path.abspath(os.path.join(self.current_dir, "..", "..", "..", "..", "resources", "best_param_detection_storage.pt"))
        classification_weights = os.path.abspath(os.path.join(self.current_dir, "..", "..", "..", "..", "resources", "best_param_classification_storage.pth"))

        # Models
        self.detection_module = YOLO(detection_weights)
        self.classification_module = self._load_resnet(classification_weights)

        self.unknown_object = None
        self.robot_0_crop_points = [(186, 82), (421, 92), (416, 332), (172, 322)]
        self.robot_1_crop_points = [(214, 52), (447, 106), (398, 344), (152, 288)]

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

    def _detect_object(self, image: np.ndarray) -> list[tuple[np.ndarray, tuple[float, float]]]:
        buffer = 10
        max_x = 639
        max_y = 479

        results = self.detection_module(image, verbose=False)

        cropped_images = []

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

                cropped_images.append(cropped_img)

        return cropped_images

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

    def _convert_string_to_object(self, predicted_class_name: str) -> tuple[ObjectShape, ObjectColor] | str:
        if predicted_class_name == "Unknown":
            return predicted_class_name
        parts = predicted_class_name.split("_")
        shape = ObjectShape.SQUARE if parts[1] == "Square" else ObjectShape.CIRCLE
        color = ObjectColor.RED if parts[0] == "Red" else ObjectColor.BLUE if parts[0] == "Blue" else ObjectColor.GREEN
        return (shape, color)

    def _crop_out_storage(self, image: np.ndarray, robot_id: int) -> np.ndarray:
        crop_points = None
        if robot_id == 0:
            crop_points = self.robot_0_crop_points
        elif robot_id == 1:
            crop_points = self.robot_1_crop_points

        # Convert the 4 coordinates into a NumPy array
        pts = np.array(crop_points, dtype=np.int32)

        # 2. Get the exact bounding box of those 4 coordinates
        # This automatically calculates the max width (w) and max height (h) needed!
        x, y, w, h = cv2.boundingRect(pts)

        # 3. Crop the exact bounding box out of the original image (no padding)
        cropped_box = image[y : y + h, x : x + w]

        # 4. Create a black mask the exact same size as our new cropped box
        mask = np.zeros(cropped_box.shape[:2], dtype=np.uint8)

        # 5. Shift our original 4 coordinates so they fit inside the new cropped box
        # We subtract the exact top-left coordinate (x, y) from our original points
        shifted_pts = pts - np.array([x, y])

        # 6. Draw the 4 coordinates onto the black mask and fill it with white (255)
        cv2.fillPoly(mask, [shifted_pts], 255)

        # 7. Apply the mask to the cropped box
        final_result = cv2.bitwise_and(cropped_box, cropped_box, mask=mask)

        return final_result

    #
    # Public functions
    #

    def process_image(self, image: np.ndarray, robot_id: int) -> list[tuple[ObjectShape, ObjectColor | str] | int, int]:

        cropped_images = self._detect_object(self._crop_out_storage(image, robot_id))

        classified_objects = []

        for cropped_object in cropped_images:
            predicted_class_name = self._classify_object(cropped_object)

            object_appearance = self._convert_string_to_object(predicted_class_name)
            classified_objects.append(object_appearance)

        return classified_objects

    def compare_image_with_DT(self, image: np.ndarray, robot_id: int, state_snapshot: SimpleNamespace) -> tuple[list[SimpleNamespace], list[SimpleNamespace]]:
        virtual_objects = state_snapshot.objects
        virtual_robots = state_snapshot.robots

        classified_objects = self.process_image(image, robot_id)
        print("Classified object:", classified_objects)

        for object in classified_objects:
            # Check for unknows
            if object == "Unknown":
                self.unknown_object = {"Location": "Storage", "Storage": robot_id}

                print(f"Robot {robot_id}: Unknown Object in storage {robot_id}")

        classified_objects_filtered = [object for object in classified_objects if object != "Unknown"]

        return_objects = []
        anomalies = []

        storage_pickup_confirmation = "Waiting"

        for virtual_object in virtual_objects:
            if virtual_object.state.id == robot_id and virtual_object.state.origin == "Robot":
                storage_pickup_confirmation = "Success"

            object_in_storage = False
            for image_object in classified_objects_filtered:
                (image_object_shape, image_object_color) = image_object
                if virtual_object.shape == image_object_shape and virtual_object.color == image_object_color:
                    object_in_storage = True

            if not object_in_storage and virtual_object.state.origin == "Storage" and virtual_object.state.id == robot_id:
                anomalies.append((configuration["Anomalies"][9], robot_id))

        # Loop through the objects in the image and compare with the state of the DT
        for image_object in classified_objects_filtered:
            (image_object_shape, image_object_color) = image_object

            for virtual_object in virtual_objects:
                if virtual_object.shape == image_object_shape and virtual_object.color == image_object_color:
                    update_object_dict = {}

                    # Check if the object should be in a storage
                    if virtual_object.state.origin == "Storage":
                        # Check if the object is in the correct storage
                        if virtual_object.state.id == robot_id:
                            continue

                        # Object is in the wrong storage
                        else:
                            update_object_dict["Updated_State"] = SimpleNamespace(origin="Storage", id=robot_id)

                    # Object is in storage but in the DT is elsewhere
                    elif virtual_object.state.origin == "Robot" and virtual_object.state.id == robot_id:
                        storage_pickup_confirmation = "Failure"
                        update_object_dict["Updated_State"] = SimpleNamespace(origin="Storage", id=robot_id)

                    if update_object_dict:
                        return_object = SimpleNamespace(color=virtual_object.color, shape=virtual_object.shape, updates=update_object_dict)
                        return_objects.append(return_object)

        print(f"storage_pickup_confirmation: {storage_pickup_confirmation}")
        return (self.unknown_object, return_objects, storage_pickup_confirmation, anomalies)


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
