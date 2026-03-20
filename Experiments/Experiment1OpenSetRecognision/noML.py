import cv2
import numpy as np
from skimage.morphology import dilation, disk, erosion


class ImageProcessing:
    def __init__(self, crop_points: np.float32):
        self.image = None
        self.crop_points = crop_points

    def set_image(self, path: str) -> None:
        self.image = cv2.imread(path)

    def _crop_and_warp(self, image: np.ndarray) -> np.ndarray:
        # Define the box to crop to
        top_width = np.linalg.norm(self.crop_points[1] - self.crop_points[0])
        bottom_width = np.linalg.norm(self.crop_points[2] - self.crop_points[3])
        left_height = np.linalg.norm(self.crop_points[3] - self.crop_points[0])
        right_height = np.linalg.norm(self.crop_points[2] - self.crop_points[1])

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
        M = cv2.getPerspectiveTransform(self.crop_points, data_points)

        # Apply and return the warp
        return cv2.warpPerspective(image, M, (width_square, height_square))

    def _create_hsv_image(self, image: np.ndarray) -> np.ndarray:
        return cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    def _create_binary_image(self, gray_scale_image: np.ndarray, threshold: int) -> np.ndarray:
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

    def _characterize_geometry(self, polsby_popper: float, area: float) -> tuple[str, float] | None:
        if polsby_popper > 0.80:
            radius = np.sqrt(area / np.pi)
            # print(f"Radius: {radius}")

            if not (radius >= 37 and radius <= 45):
                return None

            shape = "Circle"
            return (shape, radius)

        elif polsby_popper >= 0.65 and polsby_popper <= 0.8:
            side_length = np.sqrt(area)
            # print(f"Side Length: {side_length}")

            if not (side_length >= 92 and side_length <= 108):
                return None

            shape = "Square"
            return (shape, side_length)
        else:
            return None

    def _get_color_of_object(self, hue: np.ndarray, binary: np.ndarray) -> str | None:
        extracted_object = hue[binary != 0]

        pixel_sum = len(extracted_object)
        red_sum = 0
        blue_sum = 0
        green_sum = 0

        for hue_value in extracted_object:
            if hue_value >= 105 and hue_value <= 120:
                blue_sum += 1
            elif hue_value >= 170 or hue_value <= 5:
                red_sum += 1
            elif hue_value >= 75 and hue_value <= 85:
                green_sum += 1

        # print(f"RGB: {red_sum / pixel_sum}, {green_sum / pixel_sum}, {blue_sum / pixel_sum}")

        if red_sum / pixel_sum >= 0.7:
            return "Red"
        elif blue_sum / pixel_sum >= 0.7:
            return "Blue"
        elif green_sum / pixel_sum >= 0.7:
            return "Green"
        else:
            return None

    def classify(self) -> str:
        cv2.imwrite("Experiments/Experiment1OpenSetRecognision/Processed_Images/original.jpg", self.image)

        cropped_image = self._crop_and_warp(self.image)
        cv2.imwrite("Experiments/Experiment1OpenSetRecognision/Processed_Images/cropped_image.jpg", cropped_image)
        hsv_image = self._create_hsv_image(cropped_image)
        hue, _, value = cv2.split(hsv_image)
        cv2.imwrite("Experiments/Experiment1OpenSetRecognision/Processed_Images/hue.jpg", hue)
        cv2.imwrite("Experiments/Experiment1OpenSetRecognision/Processed_Images/value.jpg", value)

        # plt.hist(value.ravel(), bins=256, range=[0, 256])
        # plt.title("Histogram of Red Circle")
        # plt.xlabel("Brightness")
        # plt.ylabel("Pixel Count")
        # plt.axvline(x=130, color="r", linestyle="--", linewidth=2, label="Threshold (130)")
        # plt.legend()
        # plt.show()

        threshold = 120
        binary_image = self._create_binary_image(value, threshold)
        cv2.imwrite("Experiments/Experiment1OpenSetRecognision/Processed_Images/binary_image.jpg", binary_image * 255)

        disk_size = 3
        closed_image = self._opening_on_image(binary_image, disk_size)
        cv2.imwrite("Experiments/Experiment1OpenSetRecognision/Processed_Images/closed_image.jpg", closed_image * 255)

        stats = self._get_perimeter_and_area_of_biggest_object(closed_image)

        if stats is None or stats[0] <= 800:
            return "No_Object"

        area, perimeter_size = stats

        polsby_popper = self._polsby_popper(area, perimeter_size)
        # print(f"Polsby_popper {polsby_popper}")

        # Get geometry and color of object
        geometry = self._characterize_geometry(polsby_popper, area)
        object_color = self._get_color_of_object(hue, binary_image)

        # If the object does not resemble a circle or a square
        if geometry is None or object_color is None:
            return "Unidentified_Object"

        shape, length = geometry
        # print(f"LENGTH: {length}")
        return object_color + "_" + shape
