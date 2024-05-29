import pytest
from typing import Tuple

from src.utils.coords_scaler import CoordinateScaler


@pytest.fixture(params=[{"offboard latitude": None, "offboard longitude": None, "offboard heading": None},])
def coordinate_scaler(request):
    return CoordinateScaler(
        request.param["offboard latitude"],
        request.param["offboard longitude"],
        request.param["offboard heading"]
    )


@pytest.mark.parametrize("x,y,expected", [
    (None, None, (None, None))
])
def test_scale(coordinate_scaler, x: float, y: float, expected: Tuple[float, float]):
    assert coordinate_scaler.scale(x, y) == expected
