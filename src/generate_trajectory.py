import rospy
import matplotlib

from drone.scan_trajectory import ScanTrajectory
from sensor_msgs.msg import CameraInfo
from image_geometry import PinholeCameraModel


def get_elements_positions(x, y):
    return {"left_lower_ball": (x + 3, y + 3),
            "middle_lower_ball": (x + 7, y + 3),
            "right_lower_ball": (x + 11, y + 3),
            "left_central_ball": (x + 3, y + 7),
            "middle_central_ball": (x + 7, y + 7),
            "right_central_ball": (x + 11, y + 7),
            "left_upper_ball": (x + 3, y + 11),
            "middle_upper_ball": (x + 7, x + 11),
            "right_upper_ball": (x + 11, x + 11),
            "barrel": (x + 7, y + 27)}


if __name__ == "__main__":
    rospy.init_node("scan_trajectory")

    matplotlib.use('TkAgg')

    elements = get_elements_positions(0, 0)
    polygon_coords = [elements['left_lower_ball'], elements['left_upper_ball'], elements['right_upper_ball'], elements['right_lower_ball']]
    camera_info: CameraInfo = rospy.wait_for_message('/uav0/camera/camera_info', CameraInfo)
    camera_model = PinholeCameraModel()
    camera_model.fromCameraInfo(camera_info)

    trajectory = ScanTrajectory(polygon_coords, camera_model.fovX(), camera_model.fovY())
    trajectory.set_start_point((0, 0))
    trajectory.set_altitude(4)
    trajectory.set_overlap(0.1)
    trajectory.set_offset(1)
    waypoints = trajectory.generate_optimized_trajectory()

    trajectory.visualize(waypoints)
