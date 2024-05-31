import cv2
import numpy as np

from vision_msgs.msg import BoundingBox2D
from typing import List, Tuple


class ArucoDetector:
    def __init__(self, aruco_dict=cv2.aruco.DICT_ARUCO_ORIGINAL):
        self.aruco_dict = aruco_dict
        aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict)
        aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
        self.corners = []
        self.ids = []

    def detect(self, frame: np.ndarray) -> Tuple[List[BoundingBox2D], List[str]]:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        self.corners, self.ids, _ = self.aruco_detector.detectMarkers(gray)

        if self.ids is None:
            self.corners = np.array([])
            self.ids = np.array([])

        bboxes = [self.__to_bounding_box(corner) for corner in self.corners]
        labels = [str(id) for id in self.ids]

        return bboxes, labels

    def __to_bounding_box(self, corner: np.ndarray) -> BoundingBox2D:
        x_min = int(corner[0][:, 0].min())
        y_min = int(corner[0][:, 1].min())
        x_max = int(corner[0][:, 0].max())
        y_max = int(corner[0][:, 1].max())

        bbox = BoundingBox2D()
        bbox.center.x = (x_min + x_max) / 2
        bbox.center.y = (y_min + y_max) / 2
        bbox.size_x = x_max - x_min
        bbox.size_y = y_max - y_min

        return bbox

    def draw_markers(self, frame: np.ndarray):
        cv2.aruco.drawDetectedMarkers(frame, self.corners, self.ids)
