import cv2
import numpy as np
from typing import List, Tuple
from vision_msgs.msg import BoundingBox2D
from martian_mines.msg import BoundingBoxLabeled
from detectors.abstract_detector import AbstractDetector


class ArucoDetector(AbstractDetector):
    def __init__(self, aruco_dict=cv2.aruco.DICT_ARUCO_ORIGINAL):
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        self.corners = []
        self.ids = []

    def detect(self, frame: np.ndarray) -> Tuple[List[BoundingBox2D], List[str]]:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        self.corners, self.ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        if self.ids is None:
            self.corners = np.array([])
            self.ids = np.array([])

        bboxes = [self.__to_bounding_box(corner) for corner in self.corners]
        labels = [str(id) for id in self.ids]
        labeled_bboxes = [self.__to_labeled_bbox(bbox, label) for bbox, label in zip(bboxes, labels)]
        return labeled_bboxes

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
    
    def __to_labeled_bbox(self, bbox, label) -> BoundingBoxLabeled:
        labeled_bbox = BoundingBoxLabeled()
        labeled_bbox.bbox = bbox
        labeled_bbox.label = label
        return labeled_bbox

    def draw_markers(self, frame: np.ndarray) -> np.ndarray:
        return cv2.aruco.drawDetectedMarkers(frame, self.corners, self.ids)
