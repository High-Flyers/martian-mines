import numpy as np

from typing import Tuple


class BoundingBox:
    """Base class for all detectors"""

    def __init__(self, coords: Tuple[int, int, int, int] = (0, 0, 0, 0), label: str = '', confidence: float = 0):
        """
        coords list[ left_top_x, left_top_y, right_bot_x, right_bot_y]
        label - string
        confidence - 0.xx 
        """
        self.coords = coords
        self.label = label
        self.confidence = confidence

        self.__check_coords()

    @classmethod
    def from_ultralytics(cls, data, label_names: dict):
        """Constructs a BoundingBox object from Ultralytics data"""
        boxes = []
        for i in range(data.data.shape[0]):
            x1, y1, x2, y2 = map(int, data.xyxy[i].tolist())
            label_id = int(data.cls[i].item())
            label = label_names.get(label_id, 'Unknown')
            confidence = data.conf[i].item()
            boxes.append(cls((x1, y1, x2, y2), label, confidence))
        return boxes

    def __str__(self) -> str:
        return "{\n" + f"\tcoords: {self.coords},\n \tlabel: {self.label},\n \tconf: {self.confidence}" + "\n}"

    def __repr__(self) -> str:
        return self.__str__()

    def __check_coords(self):
        coords = np.array(self.coords)

        if (coords < 0).any():
            raise Exception(f"coords cannot be lower than 0, got {self.coords}")

        if (coords[:2] > coords[2:]).any():
            raise Exception(f"First two coords cannot be greater than other two, got {self.coords}")

    @property
    def width(self):
        return self.coords[2] - self.coords[0]

    @property
    def height(self):
        return self.coords[3] - self.coords[1]

    @property
    def shape(self):
        return (self.height, self.width)

    def to_point(self):
        x = (self.coords[0] + self.coords[2]) / 2
        y = (self.coords[1] + self.coords[3]) / 2

        return (x, y)

    def shrink_by_offset(self, offset_percent: float = 0.0):

        # Calculate offset in pixels
        offset_x = int(offset_percent * (self.coords[2] - self.coords[0]))
        offset_y = int(offset_percent * (self.coords[3] - self.coords[1]))

        # Apply offset
        self.coords[0] = self.coords[0] + offset_x
        self.coords[1] = self.coords[1] + offset_y
        self.coords[2] = self.coords[2] - offset_x
        self.coords[3] = self.coords[3] - offset_y

    def get_img_piece(self, img: np.ndarray) -> np.ndarray:
        if img.shape[0] < self.height or img.shape[1] < self.width:
            raise Exception(f"coords out of the image bounds {img.shape}, got {self.coords}")

        piece_img = img[self.coords[1]:self.coords[3], self.coords[0]:self.coords[2]]

        return piece_img
