import rospy

from nav_msgs.msg import Path
from std_srvs.srv import Trigger
from drone.offboard import Offboard
from geometry_msgs.msg import Point


class TrajectoryTracker:
    def __init__(self):
        self.path = Path()
        self.waypoint_idx = 0
        self.offboard = Offboard()

        self.sub_trajectory = rospy.Subscriber('trajectory_generator/path', Path, self.callback_trajectory)

        rospy.wait_for_service('trajectory_generator/generate', timeout=10)
        self.client_trajectory_generate = rospy.ServiceProxy('trajectory_generator/generate', Trigger)
        self.client_trajectory_generate()

    def callback_trajectory(self, path: Path):
        self.path = path
        self.waypoint_idx = 0
        self.timer = rospy.Timer(rospy.Duration(0.01), self.callback_timer)
        rospy.sleep(1)
        self.offboard.set_offboard_mode()

    def callback_timer(self, _):
        if self.waypoint_idx < len(self.path.poses):
            position: Point = self.path.poses[self.waypoint_idx].pose.position

            self.offboard.fly_point(position.x, position.y, position.z)

            if self.offboard.is_point_reached(position.x, position.y, position.z, 0.2):
                self.waypoint_idx += 1
        else:
            self.timer.shutdown()


if __name__ == "__main__":
    rospy.init_node('trajectory_tracker')

    try:
        trajectory_tracker = TrajectoryTracker()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
