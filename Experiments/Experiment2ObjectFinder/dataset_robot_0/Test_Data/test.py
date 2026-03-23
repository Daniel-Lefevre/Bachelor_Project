import os

import cv2
import numpy as np

base_path = os.path.dirname(os.path.abspath(__file__))
input_folder = os.path.join(base_path, "original_images")
output_folder = os.path.join(base_path, "images")


def process_image(image):
    # Robot 0
    # points1 = np.array([[0, 0], [165, 0], [92, 216], [62, 479], [0, 479]], np.int32)
    # points2 = np.array([[257, 0], [212, 202], [179, 479], [330, 479], [344, 0]], np.int32)
    # points3 = np.array([[433, 0], [446, 310], [433, 479], [639, 479], [639, 0]], np.int32)

    # Robot 1
    points1 = np.array([[0,0], [133,0], [69,227], [66,479], [0,479]], np.int32)
    points2 = np.array([[294,0], [302,479], [167,479], [189,52], [206,0]], np.int32)
    points3 = np.array([[380,0], [425,335], [425,479], [639,479], [639,0]], np.int32)

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

    return processed_image


# Create the output directory if it doesn't already exist.
if not os.path.exists(output_folder):
    os.makedirs(output_folder)
    print(f"Created output directory: '{output_folder}'")


# --- 3. THE MAIN LOOP ---
# This part goes through the folder and uses your function.

# A list of valid image extensions to process.
valid_extensions = (".jpg", ".jpeg", ".png", ".bmp", ".tiff")

print(f"Starting to process images from '{input_folder}'...")

image_count = 0
for filename in os.listdir(input_folder):
    # Check if the file is an image based on its extension
    if filename.lower().endswith(valid_extensions):
        # Construct the full path to the input image
        input_path = os.path.join(input_folder, filename)

        # 1. Read the image
        # This loads the image into a format (NumPy array) Python can work with.
        image = cv2.imread(input_path)

        # Check if the image was read successfully
        if image is None:
            print(f"Error: Could not read image at {input_path}")
            continue

        # 2. Process the image using your custom function
        processed_image = process_image(image)

        # 3. Create the full path to save the new image
        # We save it in 'output_folder' but keep the same filename.
        output_path = os.path.join(output_folder, filename)

        # 4. Write the processed image to the new folder
        # 'cv2.imwrite' saves the image data back to a file.
        # It handles converting the NumPy array back into a standard file format.
        cv2.imwrite(output_path, processed_image)

        image_count += 1
        print(f"Processed and saved: {filename}")

print(f"\nFinished processing. Total images processed: {image_count}")
print(f"You can find them in the '{output_folder}' folder.")
