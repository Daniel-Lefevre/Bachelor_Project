import cv2
import numpy as np
from skimage.morphology import dilation, disk, erosion
from skimage.filters import threshold_otsu

class ImageProcessing:
    def __init__(self, ):
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
        return (4 * np.pi * area) / perimeter_size**2

    def _characterize_geometry(self, polsby_popper: float, area: float, pp_circle: float, pp_square: float, radius_min,radius_max,length_min,length_max) -> tuple[str, float] | None:
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
            no_object_threshold: float = 500) -> str:

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

        if stats is None or stats[0] <= no_object_threshold:
            return "No_Object"

        area, perimeter_size = stats

        polsby_popper = self._polsby_popper(area, perimeter_size)

        # ERROR FIX 2 & 3: Actually pass the parameters into the sub-functions!
        geometry = self._characterize_geometry(polsby_popper, area, pp_circle, pp_square,radius_min,radius_max,length_min,length_max)
        object_color = self._get_color_of_object(hue, binary_image, color_p)

        # If the object does not resemble a circle or a square
        if geometry is None or object_color is None:
            return "Unidentified_Object"

        shape, length = geometry
        return object_color + "_" + shape
