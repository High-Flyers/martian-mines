import math
import numpy as np

EARTH_RADIUS = 6365.821


def get_coords_distance(coords1, coords2):
    """Returns distance between two global coords (lat, lon) in meters"""
    lat1, lon1 = coords1
    lat2, lon2 = coords2
    fi1 = lat1 * math.pi / 180.0
    fi2 = lat2 * math.pi / 180.0
    delta_fi = (lat2 - lat1) * math.pi / 180.0
    delta_lambda = (lon2 - lon1) * math.pi / 180.0
    a = math.sin(delta_fi / 2) ** 2 + math.cos(fi1) * \
        math.cos(fi2) * (math.sin(delta_lambda / 2) ** 2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    distance = EARTH_RADIUS * c

    return distance * 1000


class Positioner:
    def __init__(self, config):
        self.camera_fov = config["fov"]
        self.camera_res = config["res"]
        self.center = (self.camera_res[0] // 2, self.camera_res[1] // 2)

    def get_real_area(self, thresh, current_altitude):
        # contours, hierarchies = cv.findContours(thresh, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
        # max_area = SurfaceAnalyzer.__get_max_area(contours)
        max_area = np.sum(thresh) / 255
        scale = self.__get_scale(current_altitude)
        figure_real_area = max_area * scale ** 2

        return round(figure_real_area, 3)

    def __get_scale(self, altitude: float):
        """Returns scale in meters per pixel"""
        diagonal_length = 2 * altitude * \
            math.tan(math.radians(self.camera_fov / 2))
        camera_diaognal = math.sqrt(
            self.camera_res[0] ** 2 + self.camera_res[1] ** 2)

        return diagonal_length / camera_diaognal

    def __rotate(self, vector: tuple, angle):
        angle = math.radians(angle)
        x = math.cos(angle) * vector[0] - math.sin(angle) * vector[1]
        y = math.sin(angle) * vector[0] + math.cos(angle) * vector[1]

        return (x, y)

    def get_real_coords(self, point, drone_telem):
        """Returns real coords of figure center"""
        scale = self.__get_scale(drone_telem['altitude'])
        # X and Y distance [in metres] between centre points of frame and centre of figure
        dist_x = (point[0] - self.center[0]) * scale
        # minus in Y axis because of inverted Y coordinate in OpenCV
        dist_y = -(point[1] - self.center[1]) * scale

        diff_angle = 360 - drone_telem['heading']

        lat = math.radians(drone_telem['latitude'])
        lon = math.radians(drone_telem['longitude'])
        # print('Before: ', dist_x, dist_y)
        dist_x, dist_y = self.__rotate((dist_x, dist_y), diff_angle)
        # print('After: ', dist_x, dist_y)
        r_dist_x = dist_x / 1000 / EARTH_RADIUS
        r_dist_y = dist_y / 1000 / EARTH_RADIUS
        r_lat = lat + r_dist_y
        r_lon = lon + r_dist_x / math.cos(lat)
        latitude = math.degrees(r_lat)
        longitude = math.degrees(r_lon)

        return (latitude, longitude)

    def get_pos_in_camera_frame(self, point, drone_telem):
        scale = self.__get_scale(drone_telem['altitude'])
        # X and Y distance [in metres] between centre points of frame and centre of figure
        dist_x = (point[0] - self.center[0]) * scale
        # minus in Y axis because of inverted Y coordinate in OpenCV
        dist_y = -(point[1] - self.center[1]) * scale
        dist_z = drone_telem['altitude']

        return (dist_x, dist_y, -dist_z)
