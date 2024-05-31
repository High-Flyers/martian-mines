"""
Reference materials:
- https://stackoverflow.com/questions/7477003/calculating-new-longitude-latitude-from-old-n-meters
- https://en.wikipedia.org/wiki/Geographic_coordinate_system
- https://en.wikipedia.org/wiki/Rotation_of_axes_in_two_dimensions
"""

import math
from typing import Tuple


class CoordinateScaler:
    EARTH_RADIUS = 6_378_137

    def __init__(self, lat: float, lon: float, heading: float) -> None:
        """
        params:
        lat: float - offboard position latitude in degrees
        lon: float - offboard postiion longitude in degrees
        heading: float - offboard position compass heading in degrees
        """
        self.base_latitude = lat
        self.base_longitude = lon
        self.base_heading = math.radians(heading)

    def scale(self, x: float, y: float) -> Tuple[float, float]:
        """Scales the xy local coordinates to global values based on the offboard position and heading."""
        east_offset, north_offset = self.__on_rotated_axes(x, y)
        lat, lon = self.__get_latitude(north_offset), self.__get_longitude(east_offset)
        return lat, lon

    def __on_rotated_axes(self, x: float, y: float) -> Tuple[float, float]:
        """Get the coordinates in the frame of reference rotated by heading angle."""
        return (x * math.cos(self.base_heading) + y * math.sin(self.base_heading),
                - x * math.sin(self.base_heading) + y * math.cos(self.base_heading))

    def __get_latitude(self, offset: float) -> float:
        return self.base_latitude + (offset / self.EARTH_RADIUS) * (180 / math.pi)

    def __get_longitude(self, offset: float) -> float:
        return self.base_longitude + (offset / self.EARTH_RADIUS) * (180 / math.pi) / math.cos(self.base_latitude * (math.pi / 180))
