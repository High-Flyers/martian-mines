import numpy as np
import math
from figure.bounding_box import BoundingBox
from utils.positioner import get_coords_distance  # TO refactor
from figure.figure import Figure
from figure.rejection_type import RejectionType
from typing import List
from collections import Counter

EARTH_RADIUS = 6365.821


class FigureGroup:
    """Class for grouped figures as a list and its basic stats"""

    def __init__(self, figure: Figure, use_local_coords: bool = False):
        self.figure_list = [figure]
        if use_local_coords:
            self.mean_coords = figure.local_frame_coords
        else:
            self.mean_coords = figure.coords
        self.confirmed = False
        self.use_local_coords = use_local_coords

    def add_figure(self, figure: Figure):
        """Adds figure to group and calculate mean_coords"""
        self.figure_list.append(figure)
        if self.use_local_coords:
            cords_list = [f.local_frame_coords for f in self.figure_list]
        else:
            cords_list = [f.coords for f in self.figure_list]
        self.mean_coords = tuple(np.mean(cords_list, axis=0))

    def get_best_image(self, num_thresh):
        return self.figure_list[num_thresh // 2].figure_img

    def most_common(self, lst):
        data = Counter(lst)
        return data.most_common(1)[0][0]

    def get_most_common_determined_type(self):
        """Gets mosts common type within figures in group"""
        types_lst = [fig.determined_type for fig in self.figure_list]
        return self.most_common(types_lst)

    def get_most_common_color(self):
        """Gets mosts common color within figures in group"""
        colors_lst = [fig.color for fig in self.figure_list]
        return self.most_common(colors_lst)

    def get_mean_area(self):
        area_list = [f.area for f in self.figure_list]
        return np.mean(area_list)


def find_proper_figure_in_distance(fig_to_find: Figure, detected_figures: List[Figure], min_distance):
    closest_fig = None
    closest_dist = min_distance
    for fig in detected_figures:
        dist = get_coords_distance(fig.coords, fig_to_find.coords)
        if dist < closest_dist:
            if fig.color == fig_to_find.color and fig.nn_label == fig_to_find.nn_label:
                closest_dist = dist
                closest_fig = fig

    return closest_fig


class FigureCollector:

    def __init__(self, config):
        self.num_thresh = config["group_num_instances"]
        self.dist_thresh = config["group_dist_thresh"]
        self.use_local_coords = config["use_local_coords"]
        self.fig_groups: List[FigureGroup] = []

    def append_new_fig_group(self, new_figure):
        """Adds new figure group"""
        self.fig_groups.append(FigureGroup(new_figure, self.use_local_coords))

    def update(self, new_figures: List[Figure]):
        """Update groups with new figures"""
        for new_fig in new_figures:
            if new_fig.rejection_type != RejectionType.NONE:
                continue

            closest_group = None
            min_distance = self.dist_thresh
            for fig_group in self.fig_groups:
                if self.use_local_coords:
                    dist = math.sqrt(pow(fig_group.mean_coords[0] - new_fig.local_frame_coords[0], 2) + pow(fig_group.mean_coords[1] - new_fig.local_frame_coords[1], 2))
                else:
                    dist = get_coords_distance(fig_group.mean_coords, new_fig.coords)
                if dist < min_distance:  # TODO consider adding to many groups when dist < thresh
                    closest_group = fig_group
                    min_distance = dist

            if closest_group is None:
                self.append_new_fig_group(new_fig)
            else:
                if closest_group.confirmed:
                    new_fig.figure_img = None  # HACK We do not update picture for now so it is useless after confirmation
                closest_group.add_figure(new_fig)

    def confirm_figures(self):
        """Confirm and get newly confirmed figures"""
        confirmed_figures = []
        for group_id, fig_group in enumerate(self.fig_groups):
            if len(fig_group.figure_list) >= self.num_thresh and fig_group.confirmed == False:
                fig_group.confirmed = True
                fig_image = fig_group.get_best_image(self.num_thresh)
                fig_determined_type = fig_group.get_most_common_determined_type()
                fig_color = fig_group.get_most_common_color()
                fig_area = fig_group.get_mean_area()
                confirmed_fig = Figure(bbox=BoundingBox(), color=fig_color, determined_type=fig_determined_type, local_frame_coords=fig_group.mean_coords, area=fig_area, figure_img=fig_image, group_id=group_id)
                confirmed_figures.append(confirmed_fig)

        return confirmed_figures
