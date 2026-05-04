import cv2
import numpy as np
from skimage.morphology import dilation, disk, erosion

PRINTING = False


class ImageProcessing:
    def __init__(self, crop_points: np.float32, background_image):
        self.image = None
        self.crop_points = crop_points
        self.background_image = self._create_hsv_image(background_image)
        self.background_hue, _, _ = cv2.split(self.background_image)

    def set_image(self, path: str) -> None:
        self.image = cv2.imread(path)

    def _crop_and_warp(self, image: np.ndarray, noise: bool) -> np.ndarray:
        # 1. Create a completely black mask with the same dimensions as the original image
        mask = np.zeros(image.shape[:2], dtype=np.uint8)

        # 2. Convert crop_points to integers (required by cv2.fillPoly)
        pts = np.int32([self.crop_points])

        # 3. Draw the polygon defined by your points in solid white (255) on the mask
        cv2.fillPoly(mask, pts, 255)

        h, w = image.shape[:2]

        # 4. Get the actual image inside the polygon (everything outside becomes black/0)
        # We do this FIRST because it serves as our base image in both scenarios!
        foreground = cv2.bitwise_and(image, image, mask=mask)

        if noise:
            # 1. Create a tiny noise grid (e.g., 1/5th the size of your image)
            tiny_noise = np.random.randint(0, 20, size=(h // 5, w // 5), dtype=np.uint8)

            # 2. Blow it up to full size using Cubic interpolation.
            noise_bg = cv2.resize(tiny_noise, (w, h), interpolation=cv2.INTER_CUBIC)

            # 3. Add a tiny blur just to soften the edges of the blobs
            noise_bg = cv2.GaussianBlur(noise_bg, (3, 3), 0)

            if len(image.shape) == 3:  # Check if original is color
                noise_bg = cv2.cvtColor(noise_bg, cv2.COLOR_GRAY2BGR)

            # 4. Get the noise outside the polygon
            inverse_mask = cv2.bitwise_not(mask)
            background = cv2.bitwise_and(noise_bg, noise_bg, mask=inverse_mask)

            # 5. Add them together.
            final_full_image = cv2.add(foreground, background)
        else:
            # If noise is False, just use the foreground (which has the black background)
            final_full_image = foreground

        # 8. Get the straight bounding rectangle of your points
        x, y, w, h = cv2.boundingRect(pts)

        # 9. Crop the final image to just that bounding box
        final_cropped_image = final_full_image[y : y + h, x : x + w]

        cv2.imwrite("Experiments/Experiment1OpenSetRecognision/Processed_Images/final_cropped_image.png", final_cropped_image)

        return final_cropped_image

    def _create_hsv_image(self, image: np.ndarray) -> np.ndarray:
        return cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    def _create_binary_image(self, hsv_image: np.ndarray, threshold: int) -> np.ndarray:
        _, s1, v1 = cv2.split(hsv_image)
        _, s2, v2 = cv2.split(self.background_image)
        saturation = cv2.absdiff(s1, s2).astype(np.uint8)
        value = cv2.absdiff(v1, v2).astype(np.uint8)
        combi = cv2.addWeighted(value, 0.5, saturation, 0.5, 0)

        cropped_combi = self._crop_and_warp(combi, noise=True)

        _, binary_threshold_combi = cv2.threshold(self._crop_and_warp(combi, noise=False), 50, 255, cv2.THRESH_BINARY)
        binary_threshold_combi_closed = self._closing_on_image(binary_threshold_combi, (6, 6))
        # cv2.imwrite("Experiments/Experiment1OpenSetRecognision/Processed_Images/binary_threshold_combi.png", binary_threshold_combi_closed)
        cv2.imwrite("Experiments/Experiment1OpenSetRecognision/Processed_Images/cropped_combi.png", cropped_combi)
        # cv2.imwrite("Experiments/Experiment1OpenSetRecognision/Processed_Images/value.png", value)
        # cv2.imwrite("Experiments/Experiment1OpenSetRecognision/Processed_Images/saturation.png", saturation)

        # 2. Apply a Gaussian Blur to smooth out the background texture
        # This prevents Canny from picking up the grain of the surface as edges
        blurred = cv2.GaussianBlur(cropped_combi, (5, 5), 0)
        cv2.imwrite("Experiments/Experiment1OpenSetRecognision/Processed_Images/blurred.jpg", blurred)

        # 3. Apply Canny Edge Detection
        # Canny uses two thresholds. A common practice is making the upper one 2x or 3x the lower.
        edges = cv2.Canny(blurred, threshold, threshold * 3)
        cv2.imwrite("Experiments/Experiment1OpenSetRecognision/Processed_Images/edges.jpg", edges)

        # cv2.imwrite("Experiments/Experiment1OpenSetRecognision/Processed_Images/canny_edges.jpg", edges)

        # 4. Close any small gaps in the edge outline
        disk_size = (6, 2)
        closed_edges = self._closing_on_image(edges, disk_size)
        cv2.imwrite("Experiments/Experiment1OpenSetRecognision/Processed_Images/closed_edges.jpg", closed_edges)

        # cv2.imwrite("Experiments/Experiment1OpenSetRecognision/Processed_Images/closed_edges.jpg", closed_edges)

        # 5. Find contours on the closed edges
        contours, _ = cv2.findContours(closed_edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return np.zeros_like(closed_edges)

        # 6. Identify the largest outline

        largest_contour = max(contours, key=cv2.contourArea)

        # 7. Create a black mask and fill the inside of the edge outline with WHITE (255)

        mask = np.zeros_like(closed_edges)

        cv2.drawContours(mask, [largest_contour], -1, 255, thickness=cv2.FILLED)

        multiplied_binary = binary_threshold_combi_closed & mask
        cv2.imwrite("Experiments/Experiment1OpenSetRecognision/Processed_Images/multiplied_binary.jpg", multiplied_binary)

        SE = disk(2)

        # Closing
        cleaned_up_image = dilation(erosion((multiplied_binary / 255).astype(np.uint8), SE), SE)

        # cleaned_up_image = (mask / 255).astype(np.uint8)

        cv2.imwrite("Experiments/Experiment1OpenSetRecognision/Processed_Images/binary.jpg", cleaned_up_image * 255)

        return cleaned_up_image

    def _closing_on_image(self, image: np.ndarray, disk_size: int) -> np.ndarray:
        SE_0 = disk(disk_size[0])
        SE_1 = disk(disk_size[1])

        # Closing
        return erosion(dilation(image, SE_0), SE_1)

    def _get_perimeter_and_area_of_biggest_object(self, image: np.ndarray) -> tuple[float, float] | None:

        contours, _ = cv2.findContours(image * 255, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) == 0:
            return None

        # Pick largest contour
        cnt = max(contours, key=cv2.contourArea)

        # 1. Get the raw perimeter to use as a baseline
        raw_perimeter = cv2.arcLength(cnt, True)

        # 2. Set the "smoothing factor" (epsilon).
        # 0.02 means we tolerate a 2% deviation from the original edge.
        # Increase this to 0.05 for more smoothing, decrease to 0.005 for stricter edges.
        epsilon = 0.02 * raw_perimeter

        # 3. Create the smoothed contour!
        smoothed_contour = cv2.approxPolyDP(cnt, epsilon, True)

        # 4. Calculate your stats based on the smoothed contour
        area = cv2.contourArea(smoothed_contour)
        perimeter_size = cv2.arcLength(smoothed_contour, True)

        return (area, perimeter_size)

    def _polsby_popper(self, area: float, perimeter_size: float) -> float:
        if PRINTING:
            print(f"AREA: {area}, perimeter: {perimeter_size}")

        return (4 * np.pi * area) / perimeter_size**2

    def _characterize_geometry(self, polsby_popper: float, area: float) -> tuple[str, float] | None:
        if polsby_popper > 0.85:
            radius = np.sqrt(area / np.pi)
            if PRINTING:
                print(f"Radius: {radius}")

            if not (radius >= 35 and radius <= 45):
                return None

            shape = "Circle"
            return (shape, radius)

        elif polsby_popper >= 0.70 and polsby_popper <= 0.85:
            side_length = np.sqrt(area)
            if PRINTING:
                print(f"Side Length: {side_length}")

            if not (side_length >= 80 and side_length <= 110):
                return None

            shape = "Square"
            return (shape, side_length)
        else:
            return None

    def _get_color_of_object(self, hue: np.ndarray, binary: np.ndarray) -> str | None:
        extracted_object = hue[binary != 0]

        # plt.hist(extracted_object.ravel(), bins=256, range=[0, 256])
        # plt.show()

        pixel_sum = len(extracted_object)
        red_sum = 0
        blue_sum = 0
        green_sum = 0

        for hue_value in extracted_object:
            if hue_value >= 100 and hue_value <= 125:
                blue_sum += 1
            elif hue_value >= 165 or hue_value <= 15:
                red_sum += 1
            elif hue_value >= 70 and hue_value <= 95:
                green_sum += 1

        if PRINTING:
            print(f"RGB: {red_sum / pixel_sum}, {green_sum / pixel_sum}, {blue_sum / pixel_sum}")

        if red_sum / pixel_sum >= 0.7:
            return "Red"
        elif blue_sum / pixel_sum >= 0.7:
            return "Blue"
        elif green_sum / pixel_sum >= 0.7:
            return "Green"
        else:
            return None

    def classify(self) -> str:
        # cropped_image = self._crop_and_warp()
        hsv_image = self._create_hsv_image(self.image)
        hue, saturation, value = cv2.split(self._crop_and_warp(hsv_image, noise=False))

        cv2.imwrite("Experiments/Experiment1OpenSetRecognision/Processed_Images/hue.png", hue)
        cv2.imwrite("Experiments/Experiment1OpenSetRecognision/Processed_Images/saturation.png", saturation)
        cv2.imwrite("Experiments/Experiment1OpenSetRecognision/Processed_Images/value.png", value)

        threshold = 30
        binary_image_closed = self._create_binary_image(hsv_image, threshold)

        stats = self._get_perimeter_and_area_of_biggest_object(binary_image_closed)

        if stats is None or stats[0] <= 1000:
            return "No_Object"

        area, perimeter_size = stats

        polsby_popper = self._polsby_popper(area, perimeter_size)
        if PRINTING:
            print(f"Polsby_popper {polsby_popper}")

        # Get geometry and color of object
        geometry = self._characterize_geometry(polsby_popper, area)
        object_color = self._get_color_of_object(hue, binary_image_closed)

        # If the object does not resemble a circle or a square
        if geometry is None or object_color is None:
            return "Unidentified_Object"

        shape, length = geometry
        # print(f"LENGTH: {length}")
        return object_color + "_" + shape
