import pytest
from typing import Tuple

from src.utils.coords_scaler import CoordinateScaler

"""
Test data comes from online distance calculator: https://www.nhc.noaa.gov/gccalc.shtml
It is rounded to the nearest whole unit (kilometers), so the results are not not fully accurate.
"""


@pytest.mark.parametrize("base_lat,base_lon,heading,x,y,expected", [
    (52.0, 18.0, 0.0, 68_000, 0, (52.0, 19.0)),
    (52.0, 18.0, 0.0, 0, 111_000, (53.0, 18.0)),
    (52.0, 18.0, 0.0, 68_000, 111_000, (53.0, 19.0)),
    (52.0, 18.0, 45.0, 68_000, 0, (51.56681746738121, 18.707106781186546)),
    (52.0, 18.0, 45.0, 0, 111_000, (52.707106781186546, 19.154247833995687)),
    (52.0, 18.0, 45.0, 68_000, 111_000, (52.27392424856776, 19.861354615182233)),
])
def test_scale(base_lat: float, base_lon: float, heading: float, x: float, y: float, expected: Tuple[float, float]):
    coordinate_scaler = CoordinateScaler(base_lat, base_lon, heading)
    assert coordinate_scaler.scale(x, y) == pytest.approx(expected, abs=1.5e-2)
