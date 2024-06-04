import rospy
import numpy as np

from nav_msgs.msg import Path
from std_msgs.msg import Empty
from std_srvs.srv import Trigger
from drone.offboard import Offboard
from geometry_msgs.msg import Point
from trajectory.trajectory import Trajectory
from trajectory.pure_pursuit import PurePursuit


def point_to_np(point: Point):
    return np.array([point.x, point.y, point.z])


def path_to_trajectory(path: Path) -> Trajectory:
    points = np.array([point_to_np(pose_stamped.pose.position) for pose_stamped in path.poses])

    return Trajectory.from_points(points)


class TrajectoryTracker:
    def __init__(self):
        self.radius = float(rospy.get_param('~radius', 1))
        self.velocity = float(rospy.get_param('~velocity', 1))

        self.offboard = Offboard()
        self.trajectory = Trajectory()
        self.pure_pursiut = PurePursuit(lookahead_distance=self.radius)

        self.pub_finished = rospy.Publisher('trajectory_tracker/finished', Empty, queue_size=1)
        self.sub_trajectory = rospy.Subscriber('trajectory_tracker/path', Path, self.callback_trajectory)

    def callback_trajectory(self, path: Path):
        self.trajectory = path_to_trajectory(path)
        self.timer = rospy.Timer(rospy.Duration(0.02), self.callback_timer)
        rospy.sleep(0.2)
        self.offboard.set_offboard_mode()

    def callback_timer(self, _):
        current_pose = point_to_np(self.offboard.local_pos.pose.position)
        self.pure_pursiut.set_trajectory(self.trajectory)
        self.pure_pursiut.step(current_pose)

        velocities = self.pure_pursiut.get_velocities(current_pose, self.velocity)
        self.offboard.fly_velocity(*velocities)

        if self.pure_pursiut.is_last(current_pose):
            self.offboard.set_hold_mode()
            self.pub_finished.publish()
            self.timer.shutdown()


if __name__ == "__main__":
    rospy.init_node('trajectory_tracker')

    try:
        trajectory_tracker = TrajectoryTracker()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
