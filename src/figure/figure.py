import numpy as np

from figure.bounding_box import BoundingBox
from figure.rejection_type import RejectionType
from typing import Tuple


class Figure:
    def __init__(self, fig_type: str = '', bbox: BoundingBox = BoundingBox(), color: str = '',
                 coords: Tuple[float, float] = (0, 0), local_frame_coords: Tuple[float, float, float] = (0, 0, 0), area: float = 0, figure_img: np.ndarray = None,
                 is_verified: bool = True, rejection_type: RejectionType = RejectionType.NONE):
        self.fig_type = fig_type
        self.color = color
        self.bbox = bbox
        self.coords = coords
        self.local_frame_coords = local_frame_coords
        self.area = area
        self.figure_img = figure_img
        self.is_verified = is_verified
        self.rejection_type = rejection_type

    def __str__(self) -> str:
        return ("{\n" + f"\tfigure: {self.fig_type},\n \tcolor: {self.color},\n \tbbox: \n\t" +
                '\t'.join(str(self.bbox).splitlines(True)) +
                f",\n \tcoords: {self.coords},\n \tlocal_frame_coords: {self.local_frame_coords}, \tarea: {self.area},\n "
                f"\tis_verified: {self.is_verified},\n \trejection_type: {self.rejection_type.name}" + "\n}")

    def __repr__(self) -> str:
        return self.__str__()
