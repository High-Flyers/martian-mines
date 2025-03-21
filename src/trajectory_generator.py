import rospy
import matplotlib

from sensor_msgs.msg import CameraInfo
from image_geometry import PinholeCameraModel
from nav_msgs.msg import Path
from std_srvs.srv import Trigger, TriggerResponse
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import PoseStamped

from drone.scan_trajectory import ScanTrajectory
from utils.environment import Environemnt


class TrajectoryGenerator():
    def __init__(self):
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)

        self.trajectory_link = rospy.get_param('~trajectory_link', 'start_pose')

        camera_info = rospy.wait_for_message('camera/camera_info', CameraInfo, timeout=15)

        self.camera_model = PinholeCameraModel()
        self.camera_model.fromCameraInfo(camera_info)
        self.scan_trajectory = self.create_scan_trajectory()

        self.wait_for_transform()
        self.service_generate = rospy.Service('trajectory_generator/generate', Trigger, self.generate_trajectory)
        self.pub_trajectory = rospy.Publisher('trajectory_generator/path', Path, queue_size=1)

    def wait_for_transform(self):
        while not rospy.is_shutdown():
            if self.tf_buffer.can_transform(self.trajectory_link, "map", rospy.Time(0), rospy.Duration(1.0)):
                break

    def create_scan_trajectory(self) -> ScanTrajectory:
        altitude = float(rospy.get_param('~altitude', 4))
        overlap = float(rospy.get_param('~overlap', 0.1))
        offset = float(rospy.get_param('~offset', 1))

        environment = Environemnt(0, 0)
        polygon_coords = [environment.left_lower_ball, environment.left_upper_ball, environment.right_upper_ball, environment.right_lower_ball]
        trajectory = ScanTrajectory(polygon_coords, self.camera_model.fovX(), self.camera_model.fovY())
        trajectory.set_altitude(altitude)
        trajectory.set_overlap(overlap)
        trajectory.set_offset(offset)

        return trajectory

    def generate_trajectory(self, _) -> TriggerResponse:
        waypoints = self.scan_trajectory.generate_optimized_trajectory()

        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = 'map'
        path.poses = []

        for waypoint in waypoints:
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = self.trajectory_link
            pose.pose.position.x = waypoint[0]
            pose.pose.position.y = waypoint[1]
            pose.pose.position.z = waypoint[2]
            pose_map_link = self.tf_buffer.transform(pose, "map")

            path.poses.append(pose_map_link)

        self.pub_trajectory.publish(path)

        return TriggerResponse(success=True)

    def plot(self):
        waypoints = self.scan_trajectory.generate_optimized_trajectory()
        self.scan_trajectory.plot(waypoints)


if __name__ == "__main__":
    rospy.init_node("trajectory_generator")

    try:
        trajectory_generator = TrajectoryGenerator()
        plot = rospy.get_param('~plot', False)

        if plot:
            matplotlib.use('TkAgg')
            trajectory_generator.plot()

        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo(f"Shutting down {rospy.get_name()}")
