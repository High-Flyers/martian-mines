import numpy as np
import matplotlib.pyplot as plt
import math

from shapely.geometry import Polygon, LineString


class ScanTrajectory:
    def __init__(self, polygon_coords: list, fov_x: float, fov_y: float):
        self.altitude = 1.0
        self.overlap = 0.0
        self.start_point = (0, 0)
        self.fov_x = fov_x
        self.fov_y = fov_y
        polygon_coords = [(-y, x) for x, y in polygon_coords]

        self.polygon = Polygon(polygon_coords)
        self.polygon_offseted = self.polygon.buffer(0)

    def get_photo_size_on_ground(self):
        image_width_on_ground = self.altitude * math.tan(self.fov_x / 2) * 2
        image_height_on_ground = self.altitude * math.tan(self.fov_y / 2) * 2

        return image_width_on_ground, image_height_on_ground

    def get_distance_between_photos(self):
        image_width_on_ground = self.altitude * math.tan(self.fov_x / 2) * 2

        return image_width_on_ground * (1 - self.overlap)

    def set_offset(self, offset: float):
        self.polygon_offseted = self.polygon.buffer(offset)

    def set_altitude(self, altitude: float):
        self.altitude = altitude

    def set_overlap(self, overlap: float):
        self.overlap = overlap

    def set_start_point(self, start_point):
        self.start_point = start_point

    def generate_optimized_trajectory(self):
        waypoints = [(*self.start_point, self.altitude)]
        minx, miny, maxx, maxy = self.polygon_offseted.bounds
        polygon_width = maxx - minx
        distance_between_photos = self.get_distance_between_photos()
        num_turns = math.ceil(polygon_width / distance_between_photos)

        x = min([minx, maxx], key=lambda x: abs(x - self.start_point[0]))
        sgn = 1 if maxx - x > 0 else -1

        for _ in range(num_turns):
            line = LineString([(x, miny), (x, maxy)])
            intersection = self.polygon_offseted.intersection(line)

            if not intersection.is_empty:
                coords_2d = list(intersection.coords)
                coords_2d = sorted(coords_2d, key=lambda x: abs(x[1] - waypoints[-1][1]))
                coords_3d = [(x, y, self.altitude) for x, y in coords_2d]
                waypoints.extend(coords_3d)

            x += distance_between_photos * sgn

        waypoints = np.array(waypoints)
        waypoints[:, 0], waypoints[:, 1] = waypoints[:, 1], -waypoints[:, 0]

        return waypoints

    def plot(self, waypoints):
        fig, ax = plt.subplots()

        x, y = self.polygon_offseted.exterior.xy
        y, x = np.array(y), -np.array(x)
        ax.plot(x, y, 'b-', label='Polygon')

        waypoints_y = [point[0] for point in waypoints]
        waypoints_x = [point[1] for point in waypoints]
        ax.plot(waypoints_x, waypoints_y, 'r-', label='Trajectory')

        for waypoint in waypoints:
            self.plot_camera_footprint(ax, waypoint[1], waypoint[0])

        ax.legend()
        ax.set_xlabel('Y (meters)')
        ax.set_ylabel('X (meters)')
        ax.invert_xaxis()
        ax.set_title('UAV Trajectory and Camera Coverage')
        plt.gca().set_aspect('equal', adjustable='box')
        plt.show()

    def plot_camera_footprint(self, ax, x, y):
        image_width_on_ground, image_height_on_ground = self.get_photo_size_on_ground()
        half_width = image_width_on_ground / 2
        half_height = image_height_on_ground / 2

        footprint = Polygon([
            (x - half_width, y - half_height),
            (x + half_width, y - half_height),
            (x + half_width, y + half_height),
            (x - half_width, y + half_height)
        ])

        f_x, f_y = footprint.exterior.xy
        ax.plot(f_x, f_y, 'g--', alpha=0.5)
