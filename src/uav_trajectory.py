import numpy as np
import matplotlib.pyplot as plt
import matplotlib
import math

from shapely.geometry import Polygon, LineString

matplotlib.use('TkAgg')


def ecualiden_distance(p1, p2):
    return np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)


def get_closest_point(point, points):
    return min(points, key=lambda x: ecualiden_distance(point, x))


class UAVTrajectory:
    def __init__(self):
        # UAV and Camera parameters
        self.altitude = 20.0  # UAV altitude in meters
        self.sensor_width = 0.017  # Sensor width in meters (17mm)
        self.sensor_height = 0.013  # Sensor height in meters (13mm)
        self.focal_length = 0.024  # Focal length in meters (24mm)
        self.image_width_pixels = 4000  # Image width in pixels
        self.image_height_pixels = 3000  # Image height in pixels
        self.overlap = 0.10
        self.start_point = (0, 0)

        # Calculate GSD
        self.gsd_width = self.calculate_gsd(self.sensor_width, self.image_width_pixels)
        self.gsd_height = self.calculate_gsd(self.sensor_height, self.image_height_pixels)
        self.image_width_on_ground = self.gsd_width * self.image_width_pixels
        self.image_height_on_ground = self.gsd_height * self.image_height_pixels

        self.distance_between_photos = self.image_width_on_ground * (1 - self.overlap)
        self.polygon = Polygon([(0, -50), (100, 0), (100, 100), (0, 100)]).buffer(10)

    def calculate_gsd(self, sensor_size, image_dimension):
        return (self.altitude * sensor_size) / (self.focal_length * image_dimension)

    def set_start_point(self, start_point):
        self.start_point = start_point

    def generate_optimized_trajectory(self):
        waypoints = [self.start_point]
        minx, miny, maxx, maxy = self.polygon.bounds
        polygon_width = maxx - minx
        num_turns = math.ceil(polygon_width / self.distance_between_photos)

        x = min([minx, maxx], key=lambda x: abs(x - self.start_point[0]))
        sgn = 1 if maxx - x > 0 else -1

        for _ in range(num_turns):
            line = LineString([(x, miny), (x, maxy)])
            intersection = self.polygon.intersection(line)

            if not intersection.is_empty:
                coords = list(intersection.coords)
                coords = sorted(coords, key=lambda x: abs(x[1] - waypoints[-1][1]))
                waypoints.extend(coords)

            x += self.distance_between_photos * sgn

        return waypoints

    def visualize(self, waypoints):
        fig, ax = plt.subplots()

        # Plot polygon
        x, y = self.polygon.exterior.xy
        ax.plot(x, y, 'b-', label='Polygon')

        # Plot trajectory
        waypoints_x = [point[0] for point in waypoints]
        waypoints_y = [point[1] for point in waypoints]
        ax.plot(waypoints_x, waypoints_y, 'r-', label='Trajectory')

        # Plot camera footprints
        for waypoint in waypoints:
            self.plot_camera_footprint(ax, waypoint[0], waypoint[1])

        # Add legend and labels
        ax.legend()
        ax.set_xlabel('X (meters)')
        ax.set_ylabel('Y (meters)')
        ax.set_title('UAV Trajectory and Camera Coverage')
        plt.gca().set_aspect('equal', adjustable='box')
        plt.show()

    def plot_camera_footprint(self, ax, x, y):
        half_width = self.image_width_on_ground / 2
        half_height = self.image_height_on_ground / 2

        footprint = Polygon([
            (x - half_width, y - half_height),
            (x + half_width, y - half_height),
            (x + half_width, y + half_height),
            (x - half_width, y + half_height)
        ])

        f_x, f_y = footprint.exterior.xy
        ax.plot(f_x, f_y, 'g--', alpha=0.5)


if __name__ == '__main__':
    uav_trajectory = UAVTrajectory()
    uav_trajectory.set_start_point((100, 130))
    waypoints = uav_trajectory.generate_optimized_trajectory()
    uav_trajectory.visualize(waypoints)
