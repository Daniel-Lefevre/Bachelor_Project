import os

import cv2
import numpy as np
from noML import ImageProcessing

# ROBOT_0_SRC_Training = np.array([[174, 11], [417, 82], [463, 357], [220, 387]], dtype=np.float32)
ROBOT_0_SRC_TEST = np.array([[170, 131], [421, 102], [463, 376], [207, 417]], dtype=np.float32)
# ROBOT_1_SRC_TRAINING = np.array([[103, 134], [316, 42], [393, 326], [138, 376]], dtype=np.float32)
ROBOT_1_SRC_TEST = np.array([[116, 105], [340, 48], [412, 325], [171, 373]], dtype=np.float32)


def inspect_image(path, robot_id):

    if robot_id == 0:
        path_0 = os.path.join(script_dir, "Training_Data/No_Object/1.jpg")
        background_image_0 = cv2.imread(path_0)
        src_points_robot = ROBOT_0_SRC_TEST
        image_processor = ImageProcessing(src_points_robot, background_image_0)

    elif robot_id == 1:
        path_1 = os.path.join(script_dir, "Training_Data/No_Object/11.jpg")
        background_image_1 = cv2.imread(path_1)
        src_points_robot = ROBOT_1_SRC_TEST
        image_processor = ImageProcessing(src_points_robot, background_image_1)

    image = cv2.imread(path)

    hsv_image = image_processor._create_hsv_image(image)
    hue, saturation, value = cv2.split(hsv_image)
    threshold = 30
    binary_image = image_processor._create_binary_image(hsv_image, threshold)

    # plt.hist(combi.ravel(), bins=256, range=[0, 256])
    # plt.show()

    # cv2.imwrite("Experiments/Experiment1OpenSetRecognision/Processed_Images/v.jpg", value)
    # cv2.imwrite("Experiments/Experiment1OpenSetRecognision/Processed_Images/saturation.jpg", saturation)
    # cv2.imwrite("Experiments/Experiment1OpenSetRecognision/Processed_Images/binary.jpg", binary_image * 255)
    # cv2.imwrite("Experiments/Experiment1OpenSetRecognision/Processed_Images/hue.jpg", hue)


if __name__ == "__main__":
    script_dir = os.path.dirname(os.path.abspath(__file__))
    src_points_robot_0 = ROBOT_0_SRC_TEST
    src_points_robot_1 = ROBOT_1_SRC_TEST
    path_0 = os.path.join(script_dir, "Training_Data/No_Object/1.jpg")
    path_1 = os.path.join(script_dir, "Training_Data/No_Object/11.jpg")
    background_image_0 = cv2.imread(path_0)
    background_image_1 = cv2.imread(path_1)
    image_processor_0 = ImageProcessing(src_points_robot_0)
    image_processor_1 = ImageProcessing(src_points_robot_1)

    manual_image_path = os.path.join(script_dir, "Test_Data/No_Object/11.jpg")
    # inspect_image(manual_image_path, 1)

    labels = ["Blue_Circle", "Blue_Square", "Red_Circle", "Red_Square", "Green_Circle", "Green_Square", "No_Object", "Unidentified_Object"]
    labels = ["Green_Square"]

    correctly_labeled_images = 0
    falsy_labeled_images = 0

    for label in labels:
        print(label)
        number_of_image = 36 if label == "Unidentified_Object" else 20
        for i in range(number_of_image):
            if i == 1:
                image_path = os.path.join(script_dir, "Test_Data", label, f"{i + 1}.jpg")
                if i < number_of_image / 2:
                    image_processor_0.set_image(image_path)
                    return_label = image_processor_0.classify()
                else:
                    image_processor_1.set_image(image_path)
                    return_label = image_processor_1.classify()
                if return_label == label:
                    correctly_labeled_images += 1
                else:
                    falsy_labeled_images += 1
                    print(label + " classified as " + return_label + " - " + str(i + 1))

    test_point_sum = correctly_labeled_images + falsy_labeled_images
    print(f"{test_point_sum} test points were labeled.")
    print(f"{correctly_labeled_images} ({round(correctly_labeled_images / test_point_sum, 2)}%) were labeled corretly.")
    print(f"{falsy_labeled_images} ({round(falsy_labeled_images / test_point_sum, 2)}%) were labeled incorretly.")
