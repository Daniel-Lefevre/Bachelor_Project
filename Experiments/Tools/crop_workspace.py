import os

import cv2
import numpy as np


def crop_and_mask_polygon(image_path, points):
    # 1. Load the image
    img = cv2.imread(image_path)
    if img is None:
        raise ValueError(f"Could not load image at {image_path}")

    # Convert the 4 coordinates into a NumPy array
    pts = np.array(points, dtype=np.int32)

    # 2. Get the exact bounding box of those 4 coordinates
    # This automatically calculates the max width (w) and max height (h) needed!
    x, y, w, h = cv2.boundingRect(pts)

    # 3. Crop the exact bounding box out of the original image (no padding)
    cropped_box = img[y : y + h, x : x + w]

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


if __name__ == "__main__":
    # Define your 4 coordinates.
    # Note: Make sure to fill in robot_0_points before running, empty tuples will cause an error!
    robot_0_points = [(159, 154), (428, 118), (456, 336), (187, 369)]
    robot_1_points = [(91, 132), (348, 61), (404, 276), (146, 345)]

    # Define your input and output directories
    # Blue_Circle, Blue_Square, Green_Circle, Green_Square, No_Object, Red_Circle, Red_Square, Unidentified_Object
    for label in ["Unidentified_Object"]:
        input_folder = os.path.join(r"C:\Users\danie\OneDrive\Skrivebord\Rep\Bachelor_Project\Experiments\Experiment1OpenSetRecognision\Test_Data", label)
        output_folder = os.path.join(r"C:\Users\danie\OneDrive\Skrivebord\Rep\Bachelor_Project\Experiments\Experiment1OpenSetRecognision\Test_Data", label)

        # Create the output directory if it doesn't already exist
        os.makedirs(output_folder, exist_ok=True)

        # Loop over image numbers 1 through 200
        for i in range(1, 21):
            filename = f"{i}.jpg"

            # Safely construct the full paths
            input_path = os.path.join(input_folder, filename)
            output_path = os.path.join(output_folder, filename)

            # Process the image (Notice we removed padding=30 here)
            result_image = crop_and_mask_polygon(input_path, robot_1_points)

            # Save the new image to the output folder
            success = cv2.imwrite(output_path, result_image)
