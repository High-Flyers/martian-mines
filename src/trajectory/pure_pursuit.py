import numpy as np

from trajectory.trajectory import Trajectory, Waypoint, Sphere, pt_to_pt_distance


def sgn(num: float) -> int:
    return 1 if num >= 0 else -1


class PurePursuit:
    def __init__(self, lookahead_distance: float = 1.0) -> None:
        self.lookahead_distance = lookahead_distance
        self.trajectory: Trajectory = None
        self.target_point: np.ndarray = None

    def set_trajectory(self, trajectory: Trajectory) -> None:
        self.trajectory = trajectory
        self.target_point = trajectory[0].point

    def step(self, current_pose: np.ndarray) -> None:
        if self.trajectory is None:
            raise ValueError('Trajectory is not set!')

        target_point = self.target_point
        index = self.trajectory.index

        if self.is_last(current_pose):
            self.trajectory.update_index(len(self.trajectory) - 1)
            self.target_point = self.trajectory[-1].point
            return

        for i in range(self.trajectory.index, len(self.trajectory.segments)):
            segment = self.trajectory.segments[i]
            sphere = Sphere(current_pose, self.lookahead_distance)
            intersection_points = sphere.intersect_segment(segment)

            if len(intersection_points) == 0:
                self.target_point = self.trajectory[index].point
                continue
            elif len(intersection_points) == 1:
                target_point = intersection_points[0]
            elif len(intersection_points) == 2:
                target_point = self._get_closest_point_to_target(intersection_points, segment.end)

            if pt_to_pt_distance(target_point, segment.end) < pt_to_pt_distance(current_pose, segment.end):
                index = i
                self.target_point = target_point
                break
            else:
                index += 1

        self.trajectory.update_index(index)

    def get_velocities(self, current_pose: np.ndarray, velocity: float) -> np.ndarray:
        target_vector = self.target_point - current_pose
        distance = np.linalg.norm(target_vector)
        velocity_factor = -2 ** (-(distance / self.lookahead_distance) + 1) + 2

        if self.is_last(current_pose):
            return target_vector * velocity_factor / distance
        else:
            return target_vector * velocity * velocity_factor / distance

    def is_last(self, current_pose: np.ndarray) -> bool:
        last_waypoint = self.trajectory[-1]
        closest_point = self.trajectory.segment.get_closest_point(last_waypoint.point)
        closest_waypoint = Waypoint(closest_point, self.trajectory.segment)
        trajectory_distance = self.trajectory.get_distance(closest_waypoint, last_waypoint)
        euclidean_distance = np.linalg.norm(last_waypoint.point - current_pose)

        return bool(trajectory_distance < self.lookahead_distance) \
            and bool(euclidean_distance < self.lookahead_distance)
    
    def finished(self, current_pose: np.ndarray, epsilon=0.1) -> bool:
        euclidean_distance = np.linalg.norm(self.target_point - current_pose)

        return self.is_last(current_pose) and bool(euclidean_distance < epsilon)

    def _get_closest_point_to_target(self, points: np.ndarray, target: np.ndarray) -> np.ndarray:
        min_idx = np.argmin(np.sum((points - target) ** 2, axis=1))
        return points[min_idx]
