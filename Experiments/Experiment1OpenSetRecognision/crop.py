import cv2
import numpy as np
import os

def crop_and_mask_polygon(image_path, points, padding=20):
    # 1. Load the image
    img = cv2.imread(image_path)
    if img is None:
        raise ValueError(f"Could not load image at {image_path}")

    img_height, img_width = img.shape[:2]

    # Convert the 4 coordinates into a NumPy array
    pts = np.array(points, dtype=np.int32)

    # 2. Get the standard bounding box of those 4 coordinates
    # x, y is the top-left corner; w, h are the width and height
    x, y, w, h = cv2.boundingRect(pts)

    # 3. Calculate the "little bit bigger" padded box
    # We use max() and min() to ensure the padding doesn't go off the edge of the image
    x_pad = max(0, x - padding)
    y_pad = max(0, y - padding)
    x_end_pad = min(img_width, x + w + padding)
    y_end_pad = min(img_height, y + h + padding)

    # 4. Crop the padded box out of the original image
    cropped_padded_box = img[y_pad:y_end_pad, x_pad:x_end_pad]

    # 5. Create a black mask the exact same size as our new cropped box
    mask = np.zeros(cropped_padded_box.shape[:2], dtype=np.uint8)

    # 6. Shift our original 4 coordinates so they fit inside the new cropped box
    # We subtract the top-left padded coordinates from our original points
    shifted_pts = pts - np.array([x_pad, y_pad])

    # 7. Draw the 4 coordinates onto the black mask and fill it with white (255)
    cv2.fillPoly(mask, [shifted_pts], 255)

    # 8. Apply the mask to the cropped box
    # This keeps the white area (your 4 points) and turns the rest of the box black
    final_result = cv2.bitwise_and(cropped_padded_box, cropped_padded_box, mask=mask)

    return final_result

# ==========================================
# Example Usage
# ==========================================
if __name__ == "__main__":
    # Define your 4 coordinates.
    # Note: This assumes the same 4 coordinates apply to EVERY image in the folder.
    my_4_points = [(150, 100), (350, 120), (330, 280), (120, 250)]

    # Define your input and output directories
    input_folder = os.path.join(r"C:\Users\danie\OneDrive\Skrivebord\Rep\Bachelor_Project\Experiments\Experiment1OpenSetRecognision\Training_Data", "Blue_Circle")
    output_folder = os.path.join(r"C:\Users\danie\OneDrive\Skrivebord\Rep\Bachelor_Project\Experiments\Experiment1OpenSetRecognision\Training_Data_Cropped", "Blue_Circle")

    # Create the output directory if it doesn't already exist
    # exist_ok=True prevents an error if the folder is already there
    os.makedirs(output_folder, exist_ok=True)

    # Loop over image numbers 1 through 200
    for i in range(1, 201):
        filename = f"{i}.jpg"

        # Safely construct the full paths
        input_path = os.path.join(input_folder, filename)
        output_path = os.path.join(output_folder, filename)

        # Check if the file actually exists before trying to process it
        if not os.path.exists(input_path):
            print(f"Skipping {filename} - File not found in {input_folder}")
            continue

        try:
            # Process the image
            result_image = crop_and_mask_polygon(input_path, my_4_points, padding=30)

            # Save the new image to the output folder
            success = cv2.imwrite(output_path, result_image)

            if success:
                print(f"Successfully saved: {output_path}")
            else:
                print(f"Failed to save: {output_path} (Check folder permissions)")

        except Exception as e:
            print(f"Error processing {filename}: {e}")

    print("Batch processing complete!")
