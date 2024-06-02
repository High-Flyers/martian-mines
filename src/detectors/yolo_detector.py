import numpy as np
from ultralytics import YOLO
from typing import List
from abstract_detector import AbstractDetector
from vision_msgs.msg import BoundingBox2D
from martian_mines.msg import BoundingBoxLabeled


class YoloDetector(AbstractDetector):
    def __init__(self, nn_model_path: str):
        self.yolo_model = YOLO(nn_model_path, verbose=True)
        self.last_results = None

    def detect(self, frame: np.ndarray) -> List[BoundingBoxLabeled]:
        self.last_results = self.yolo_model.predict(frame, verbose=False, imgsz=640)
        bboxes = self.__from_ultralytics(self.last_results[0].boxes, self.last_results[0].names)
        return bboxes

    def __from_ultralytics(data, label_names) -> List[BoundingBoxLabeled]:
        boxes = []
        for i in range(data.data.shape[0]):
            x_min, y_min, x_max, y_max = map(int, data.xyxy[i].tolist())
            label_id = int(data.cls[i].item())
            label = label_names.get(label_id, 'Unknown')
            confidence = data.conf[i].item()
            bbox = BoundingBox2D()
            bbox.center.x = (x_min + x_max) / 2
            bbox.center.y = (y_min + y_max) / 2
            bbox.size_x = x_max - x_min
            bbox.size_y = y_max - y_min
            bbox_labeled = BoundingBoxLabeled()
            bbox_labeled.bbox = bbox
            bbox_labeled.label = label
            bbox_labeled.confidence = confidence
            boxes.append(bbox_labeled)

    def draw_markers(self, frame: np.ndarray) -> np.ndarray:
        return self.last_results[0].plot()
