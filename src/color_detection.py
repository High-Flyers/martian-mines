import numpy as np
import cv2 as cv
import os
import json
import argparse

from itertools import combinations


def find_closest_divisor(n, m):
    """Find the divisor of n closest to m"""

    divisors = np.array([i for i in range(1, int(np.sqrt(n) + 1)) if n % i == 0])
    divisions = n / divisors

    return int(divisions[np.argmin(np.abs(m - divisions))])


class ColorDetection:
    def __init__(self, path: str = None):
        self.load(path)

    def set_min(self, array: np.ndarray):
        self.min = array

        if self.color:
            min_list = [int(x) for x in array]
            self.config[self.color]["range"]["min"] = list(min_list)

    def set_max(self, array: np.ndarray):
        self.max = array

        if self.color:
            max_list = [int(x) for x in array]
            self.config[self.color]["range"]["max"] = list(max_list)

    def save(self, path: str = None):
        path = path if path else "config/color_detection.json"

        if os.path.exists(path):
            os.remove(path)

        with open(path, "w") as file:
            json.dump(self.config, file)

    def load(self, path: str = None):
        path = path if path else "config/color_detection.json"

        if os.path.exists(path):
            with open(path, "r") as file:
                self.config = json.load(file)
                self.colors = list(self.config.keys())
                self.set_color(self.colors[0])
        else:
            raise Exception(f"Config path {path} for color detection not exists")

    def set_color(self, color: str = None):
        if not color:
            self.min = np.array([0, 0, 0], np.uint8)
            self.max = np.array([255, 255, 255], np.uint8)
        else:
            if color not in self.colors:
                raise Exception(f"Color {color} is none of this {self.colors}")

            range_config = self.config[color]["range"]

            self.min = np.array(range_config["min"], np.uint8)
            self.max = np.array(range_config["max"], np.uint8)

        self.color = color

    def get_mask(self, img: np.ndarray, cv_code: int = None) -> np.ndarray:
        if cv_code:
            img = cv.cvtColor(img, cv_code)

        return cv.inRange(img, self.min, self.max)

    def apply_mask(self, img: np.ndarray):
        img_lab = cv.cvtColor(img, cv.COLOR_BGR2LAB)
        mask = self.get_mask(img_lab)
        img_masked = cv.bitwise_and(img_lab, img_lab, mask=mask)
        img = cv.cvtColor(img_masked, cv.COLOR_LAB2BGR)

        return img

    def get_color(self, img: np.ndarray, mask: np.ndarray = None) -> str:
        img_lab = cv.cvtColor(img, cv.COLOR_BGR2LAB)
        mean_col = np.array(cv.mean(img_lab, mask=mask)[:3], np.uint8)

        for color in self.colors:
            self.set_color(color)

            if (mean_col > self.min).all() and (mean_col < self.max).all():
                return color

        return None

    def get_kmeans_thresh(self, img: np.ndarray, k: int = 1, show=False) -> np.ndarray:
        # Get contours using kmeans and binary thresh
        kmeans = self.__get_kmeans(img, k)
        if show:
            cv.imshow("kmeans", kmeans)
        shape = np.array(kmeans.shape[:2])
        center = (shape / 2).astype(np.int32)

        # Get most common color in range
        range = int((shape * 0.2).min())
        colors = kmeans[
            (center[0] - range): (center[0] + range),
            (center[1] - range): (center[1] + range),
        ].reshape(-1, 3)
        colors_unq, counts = np.unique(colors, return_counts=True, axis=0)
        color = colors_unq[counts.argmax()]
        thresh = cv.inRange(kmeans, color, color)

        return thresh

    def get_dominant_colors(self, img: np.ndarray, k: int = 2, show=False):
        pixels = img.reshape((-1, 3))
        pixels = np.float32(pixels)
        criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 10, 1.0)
        _, labels, centers = cv.kmeans(pixels, k, None, criteria, 10, cv.KMEANS_RANDOM_CENTERS)
        centers = np.uint8(centers)
        dominant_colors = centers[labels.flatten()]

        clustered_img = dominant_colors.reshape(img.shape)
        if show:
            cv.imshow("clustered_img", clustered_img)
        return centers
    
    def get_matching_color(self, dominant_colors: np.ndarray) -> str:
        def rgb_to_lab(bgr_color):
            lab_color = cv.cvtColor(np.array([[bgr_color]]), cv.COLOR_BGR2LAB)
            return lab_color[0][0]
        lab_colors = [rgb_to_lab(c) for c in dominant_colors]
        white_lab = (255, 128, 128)
        distances_to_white = [np.linalg.norm(np.array(lab_color) - np.array(white_lab)) for lab_color in lab_colors]

        max_distance_index = np.argmax(distances_to_white)
        most_non_white_color = lab_colors[max_distance_index]
        # print(f"Most non white color: {most_non_white_color}")

        # Check ranges
        for color in self.colors:
            self.set_color(color)
            if (most_non_white_color > self.min).all() and (most_non_white_color < self.max).all():
                return color

        return "none"

    def __get_kmeans(self, img: np.ndarray, k: int) -> np.ndarray:
        # blur = cv.bilateralFilter(img, 10, 120, 120)
        z = img.reshape((-1, 3))
        # Convert to np.float32
        z = np.float32(z)
        # Define criteria, number of clusters (K) and apply kmeans function
        criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 10, 1.0)
        ret, label, center = cv.kmeans(
            z, k, None, criteria, 2, cv.KMEANS_RANDOM_CENTERS
        )
        # Now convert back into uint8, and make original image
        center = np.uint8(center)
        res = center[label.flatten()]

        return res.reshape(img.shape)


class ColorDetectionUI:
    def __init__(self, color_detection: ColorDetection):
        self.color_detection = color_detection
        self.wnd = "Color detection config"

        self.trackbar_names = {
            "L_min": "Lightness min",
            "L_max": "Lightness max",
            "a_min": "Red/Green Value min",
            "a_max": "Red/Green Value max",
            "b_min": "Blue/Yellow Value min",
            "b_max": "Blue/Yellow Value max",
        }

        self.color_index = {"L": 0, "a": 1, "b": 2}

        self.cv_gradient = cv.imread("utils/all_rgb.jpg")
        self.cv_gradient_masked = np.zeros((), dtype=np.uint8)
        self.cv_test_image = np.zeros((), dtype=np.uint8)
        self.cv_test_image_masked = np.zeros((), dtype=np.uint8)
        self.clicked_button = None

    def __del__(self):
        cv.destroyWindow(self.wnd)

    def run(self):
        # self.load_config()
        cv.namedWindow(self.wnd, flags=cv.WINDOW_AUTOSIZE)
        self.create_trackbars()

    def create_trackbar_callback(self, key_name: str):
        if "_" in key_name:
            name_list = key_name.split("_")
            index = self.color_index[name_list[0]]
            type = name_list[1]

            def callback(x):
                if type == "min":
                    min = self.color_detection.min
                    min[index] = x
                    self.color_detection.set_min(min)
                else:
                    max = self.color_detection.max
                    max[index] = x
                    self.color_detection.set_max(max)

                self.color_detection.save()
                self.update_cv_gradient_masked()
                self.update_cv_test_image_masked()
                self.show_all()

        else:
            raise Exception(f"Wrong trackbar key name, get {key_name}")

        return callback

    def create_button_callback(self, color: str):
        def callback(x, y):
            self.color_detection.set_color(color)
            self.clicked_button = color
            self.update_all()
            self.show_all()

        return callback

    def button_common_callback(self, x, y):
        self.color_detection.set_color(None)
        self.clicked_button = "common"
        self.update_all()
        self.show_all()

    def create_trackbars(self):
        cv.createTrackbar(
            self.trackbar_names["L_min"],
            self.wnd,
            0,
            255,
            self.create_trackbar_callback("L_min"),
        )
        cv.createTrackbar(
            self.trackbar_names["L_max"],
            self.wnd,
            0,
            255,
            self.create_trackbar_callback("L_max"),
        )
        cv.createTrackbar(
            self.trackbar_names["a_min"],
            self.wnd,
            0,
            255,
            self.create_trackbar_callback("a_min"),
        )
        cv.createTrackbar(
            self.trackbar_names["a_max"],
            self.wnd,
            0,
            255,
            self.create_trackbar_callback("a_max"),
        )
        cv.createTrackbar(
            self.trackbar_names["b_min"],
            self.wnd,
            0,
            255,
            self.create_trackbar_callback("b_min"),
        )
        cv.createTrackbar(
            self.trackbar_names["b_max"],
            self.wnd,
            0,
            255,
            self.create_trackbar_callback("b_max"),
        )

        self.update_trackbars_pos()

        for color in self.color_detection.colors:
            cv.createButton(color, self.create_button_callback(color))

        cv.createButton("test", self.create_button_callback(None))
        cv.createButton("common", self.button_common_callback)

    def update_trackbars_pos(self):
        cv.setTrackbarPos(
            self.trackbar_names["L_min"],
            self.wnd,
            self.color_detection.min[self.color_index["L"]],
        )
        cv.setTrackbarPos(
            self.trackbar_names["L_max"],
            self.wnd,
            self.color_detection.max[self.color_index["L"]],
        )
        cv.setTrackbarPos(
            self.trackbar_names["a_min"],
            self.wnd,
            self.color_detection.min[self.color_index["a"]],
        )
        cv.setTrackbarPos(
            self.trackbar_names["a_max"],
            self.wnd,
            self.color_detection.max[self.color_index["a"]],
        )
        cv.setTrackbarPos(
            self.trackbar_names["b_min"],
            self.wnd,
            self.color_detection.min[self.color_index["b"]],
        )
        cv.setTrackbarPos(
            self.trackbar_names["b_max"],
            self.wnd,
            self.color_detection.max[self.color_index["b"]],
        )

    def update_cv_gradient_masked(self):
        if self.clicked_button == "common":
            mask = np.zeros(self.cv_gradient.shape[:2], dtype=np.uint8)

            for color_comb in combinations(self.color_detection.colors, 2):
                self.color_detection.set_color(color_comb[0])
                color_mask1 = self.color_detection.get_mask(
                    self.cv_gradient, cv.COLOR_BGR2LAB
                )
                self.color_detection.set_color(color_comb[1])
                color_mask2 = self.color_detection.get_mask(
                    self.cv_gradient, cv.COLOR_BGR2LAB
                )

                color_mask_comb = color_mask1 & color_mask2
                mask = mask | color_mask_comb

            self.cv_gradient_masked = mask
            self.color_detection.set_color(None)
            self.cv_gradient_masked = cv.bitwise_and(
                self.cv_gradient, self.cv_gradient, mask=mask
            )
        else:
            self.cv_gradient_masked = self.color_detection.apply_mask(self.cv_gradient)

    def update_cv_test_image_masked(self):
        if self.clicked_button == "common":
            mask = np.zeros(self.cv_test_image.shape[:2], dtype=np.uint8)

            for color in self.color_detection.colors:
                self.color_detection.set_color(color)
                color_mask = self.color_detection.get_mask(
                    self.cv_test_image, cv.COLOR_BGR2LAB
                )
                mask = mask | color_mask

            self.color_detection.set_color(None)
            self.cv_test_image_masked = cv.bitwise_and(
                self.cv_test_image, self.cv_test_image, mask=mask
            )
        else:
            self.cv_test_image_masked = color_detection.apply_mask(self.cv_test_image)

    def update_all(self):
        self.update_cv_gradient_masked()
        self.update_cv_test_image_masked()
        self.update_trackbars_pos()

    def show_all(self):
        height = 600
        width = int(
            self.cv_gradient_masked.shape[1] * height / self.cv_gradient_masked.shape[0]
        )
        self.cv_gradient_masked = cv.resize(
            self.cv_gradient_masked, (width, height), interpolation=cv.INTER_AREA
        )
        self.cv_gradient = cv.resize(
            self.cv_gradient, (width, height), interpolation=cv.INTER_AREA
        )

        width = int(
            self.cv_test_image_masked.shape[1] * height / self.cv_test_image_masked.shape[0]
        )
        self.cv_test_image_masked = cv.resize(
            self.cv_test_image_masked, (width, height), interpolation=cv.INTER_AREA
        )
        self.cv_test_image = cv.resize(
            self.cv_test_image, (width, height), interpolation=cv.INTER_AREA
        )

        img = np.concatenate(
            (self.cv_gradient_masked, self.cv_test_image_masked), axis=1
        )

        cv.imshow(self.wnd, img)

    def load_config(self, path: str = None):
        self.color_detection.load(path)

    def load_test_image(self, path: str = None):
        path = path if path else "utils/test_color_detection.png"

        if not os.path.exists(path):
            raise Exception(f"Path {path} to test image does not exists!")

        self.cv_test_image = cv.imread(path)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--video-source", "-v", help="video source file path")
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()

    color_detection = ColorDetection()

    # Run GUI
    ui = ColorDetectionUI(color_detection)
    ui.run()

    vid = cv.VideoCapture(args.video_source)

    while True:
        ret, frame = vid.read()
        ui.cv_test_image = frame
        ui.update_cv_test_image_masked()
        ui.show_all()
        frame = cv.resize(frame, (640, 360))
        cv.imshow("video", frame)
        key = cv.waitKey(10)
        if key == ord("p"):
            while (key := cv.waitKey(-1)):
                if key == ord("p"):
                    break
        if key == ord("q"):
            break
