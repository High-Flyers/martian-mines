import numpy as np
import os
import json

from typing import List
from figure.bounding_box import BoundingBox
from .color_detection import ColorDetection
from utils.positioner import Positioner
from figure.figure import Figure
from figure.rejection_type import RejectionType


class FigureManager():
    def __init__(self, positioner: Positioner, load_config: bool = True):
        if load_config:
            self.load()

        self.positioner = positioner
        # self.color_detection = ColorDetection()

    def load(self, path: str = None):
        path = path if path else "config/figure_manager.json"

        if os.path.exists(path):
            with open(path, "r") as file:
                self.config = json.load(file)
        else:
            raise Exception(
                f"Config path {path} for figure manager not exists")

    def save(self, path: str = None):
        path = path if path else "config/figure_manager.json"

        if os.path.exists(path):
            os.remove(path)

        with open(path, "w") as file:
            json.dump(self.config, file)

    def create_figures(self, img: np.ndarray, bounding_boxes: List[BoundingBox], telem) -> List[Figure]:
        figures = []

        for bbox in bounding_boxes:
            try:  # temp fix for get figure thresh, probably caused by to small area to calculate color/thresh
                # bbox.shrink_by_offset(0.1)
                figure_img = bbox.get_img_piece(img)
                fig_type = bbox.label

                figure = Figure(fig_type, bbox, figure_img=figure_img)

                # if not self.verify_by_type(fig_type):
                #     figure.is_verified = False
                #     figure.rejection_type = RejectionType.TYPE
                #     figures.append(figure)
                #     continue

                # thresh = self.get_figure_thresh(figure_img, fig_type)
                # color = self.color_detection.get_color(figure_img, thresh)
                # figure.color = color

                # if not self.verify_by_color(fig_type, color):
                #     figure.is_verified = False
                #     figure.rejection_type = RejectionType.COLOR
                #     figures.append(figure)
                #     continue

                # area = self.positioner.get_real_area(thresh, telem['altitude'])
                # figure.area = area

                # if not self.verify_by_area(fig_type, area):
                #     figure.is_verified = False
                #     figure.rejection_type = RejectionType.AREA
                #     figures.append(figure)

                #     continue

                xyz_from_camera = self.positioner.get_pos_in_camera_frame(bbox.to_point(), telem["altitude"])
                figure.local_frame_coords = xyz_from_camera
                figure.coords = self.positioner.get_real_coords(bbox.to_point(), telem)
            except Exception as e:
                print(f"Figure creation exception: {e}")

            figures.append(figure)

        return figures

    def get_figure_thresh(self, img: np.ndarray, fig_type: str, show=False) -> np.ndarray:
        k_colors = self.config['figures'][fig_type]['k_colors']
        thresh = self.color_detection.get_kmeans_thresh(img, k=k_colors + 1, show=show)

        return thresh

    def verify_by_type(self, fig_type: str) -> bool:
        return fig_type in self.config['figures'].keys()

    def verify_by_color(self, fig_type: str, color: str) -> bool:
        fig_config = self.config['figures'][fig_type]
        colors = fig_config['colors']

        if "exclude-colors" in fig_config and color in fig_config["exclude-colors"]:
            return False

        if len(colors) == 0:
            return True

        if not color:
            return False

        return color in colors

    def verify_by_area(self, fig_type: str, area: float) -> bool:
        area_bounds = self.config['figures'][fig_type]['area']

        return area >= area_bounds['min'] and area <= area_bounds['max']
