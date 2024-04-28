import math
import numpy as np

EARTH_RADIUS = 6365.821


def get_coords_distance(
    coords1: tuple[float, float], coords2: tuple[float, float]
) -> float:
    """Returns the distance between two coordinates (lat, lon) in meters."""
    lat1, lon1 = coords1
    lat2, lon2 = coords2

    fi1 = lat1 * math.pi / 180.0
    fi2 = lat2 * math.pi / 180.0

    delta_fi = (lat2 - lat1) * math.pi / 180.0
    delta_lambda = (lon2 - lon1) * math.pi / 180.0

    a = math.sin(delta_fi / 2) ** 2 + math.cos(fi1) * math.cos(fi2) * (
        math.sin(delta_lambda / 2) ** 2
    )
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    distance = EARTH_RADIUS * c
    return distance * 1000
