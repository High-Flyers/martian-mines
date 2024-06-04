import pyrr
import numpy as np
from pyrr.geometric_tests import point_closest_point_on_line_segment, ray_intersect_sphere

from typing import List


def pt_to_pt_distance(point1: np.ndarray, point2: np.ndarray) -> np.float32:
    return np.linalg.norm(point1 - point2)


class Segment:
    def __init__(self, start: np.ndarray, end: np.ndarray) -> None:
        self.start = start
        self.end = end
        self._line = pyrr.line.create_from_points(start, end)
        self._length = float(np.linalg.norm(end - start))

    def is_between(self, point: np.ndarray, epsilon=1e-6) -> bool:
        closest_point = point_closest_point_on_line_segment(point, self._line)
        distance = np.linalg.norm(closest_point - point)

        return bool(distance < epsilon)

    def get_closest_point(self, point: np.ndarray) -> np.ndarray:
        return point_closest_point_on_line_segment(point, self._line)

    @property
    def line(self) -> np.ndarray:
        return self._line

    @property
    def length(self) -> float:
        return self._length

    def __repr__(self) -> str:
        return f'<Segment({self.start}, {self.end})>'


class Sphere:
    def __init__(self, center: np.ndarray, radius: float) -> None:
        self.center = center
        self.radius = radius

    def intersect_segment(self, segment: Segment) -> np.ndarray:
        sphere = pyrr.sphere.create(self.center, self.radius)
        ray = pyrr.ray.create_from_line(segment.line)
        intersection_points = ray_intersect_sphere(ray, sphere)
        intersection_points = [point for point in intersection_points if segment.is_between(point)]
        return np.array(intersection_points)


class Waypoint:
    def __init__(self, point: np.ndarray, segment: Segment) -> None:
        self.point = point
        self.segment = segment

        if not segment.is_between(point):
            raise ValueError(f'Point {point} is not on {segment}')

    def segment_distance(self) -> float:
        return float(pt_to_pt_distance(self.point, self.segment.start))


class Trajectory:
    def __init__(self)-> None:
        self._points = np.array([])
        self._segments = []
        self._waypoints = []
        self._index = 0
        self._segment: Segment = None
    
    @classmethod
    def from_points(cls, points: np.ndarray):
        if len(points) < 2:
            raise ValueError('Trajectory must have at least 2 points')

        trajectory: Trajectory = cls()
        trajectory._points = points
        trajectory._segments = [Segment(start, end) for start, end in zip(points[:-1], points[1:])]
        trajectory._waypoints = [Waypoint(point, segment) for point, segment in zip(points, trajectory._segments)]

        if len(trajectory.points) > 0:
            trajectory._waypoints.append(Waypoint(points[-1], trajectory._segments[-1]))

        trajectory._index = 0
        trajectory._segment = trajectory._segments[0]

        return trajectory

    def __len__(self) -> int:
        return len(self._waypoints)

    def __getitem__(self, index: int) -> Waypoint:
        return self._waypoints[index]

    @property
    def points(self) -> np.ndarray:
        return self._points

    @property
    def segments(self) -> List[Segment]:
        return self._segments

    @property
    def index(self) -> int:
        return self._index

    @property
    def segment(self) -> Segment:
        return self._segment

    def is_last(self) -> bool:
        return self._index >= len(self._segments) - 1

    def update_index(self, index: int) -> None:
        if index < 0 or index >= len(self._waypoints):
            raise ValueError(f'Index must be greater or equal 0 and smaller than {len(self._waypoints)}! Got index: {index}')
        self._index = index
        self._segment = self._segments[min(index, len(self._segments) - 1)]

    def get_distance(self, point1: Waypoint, point2: Waypoint) -> float:
        segment1_idx = self._segments.index(point1.segment)
        segment2_idx = self._segments.index(point2.segment)
        segments_between = self._segments[segment1_idx:segment2_idx]

        distance = sum(segment.length for segment in segments_between)
        distance -= point1.segment_distance()
        distance += point2.segment_distance()

        return distance
