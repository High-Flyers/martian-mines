import numpy as np

from martian_mines.msg import BoundingBoxLabeled
from figure.rejection_type import RejectionType
from martian_mines.msg import FigureMsg
from typing import Tuple


class Figure:
    def __init__(self, nn_label: str = '', bbox: BoundingBoxLabeled = BoundingBoxLabeled(), color: str = '',
                 coords: Tuple[float, float] = (0, 0), local_frame_coords: Tuple[float, float, float] = (0, 0, 0), area: float = 0, figure_img: np.ndarray = None,
                 is_verified: bool = True, rejection_type: RejectionType = RejectionType.NONE, determined_type: str = '', group_id: int = None, status: str = ""):
        self.nn_label = nn_label
        self.determined_type = determined_type
        self.color = color
        self.bbox = bbox
        self.coords = coords
        self.local_frame_coords = local_frame_coords
        self.area = area
        self.figure_img = figure_img
        self.is_verified = is_verified
        self.rejection_type = rejection_type
        self.group_id = group_id
        self.status = status  # values: on_ground, picked_up, delivered

    def __str__(self) -> str:
        return ("{\n" + f"\tfigure nn labels: {self.nn_label},\n \tcolor: {self.color},\n \tbbox: \n\t" +
                '\t'.join(str(self.bbox).splitlines(True)) +
                f",\n \tcoords: {self.coords},\n \tlocal_frame_coords: {self.local_frame_coords}, \tarea: {self.area},\n "
                f"\tis_verified: {self.is_verified},\n \trejection_type: {self.rejection_type.name} \t determined_type: {self.determined_type} \t group_id: {self.group_id}" + "\n}")

    def __repr__(self) -> str:
        return self.__str__()

    def to_msg(self, status: str = ""):
        fig_msg = FigureMsg()
        fig_msg.id = self.group_id
        fig_msg.local_x = self.local_frame_coords[0]
        fig_msg.local_y = self.local_frame_coords[1]
        fig_msg.type = self.determined_type
        fig_msg.status = self.status
        if status != "":
            fig_msg.status = status
        return fig_msg
