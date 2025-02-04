import rospy
import numpy as np

from mavros_msgs.msg import LandingTarget
from sensor_msgs.msg import CameraInfo
from drone.offboard import Offboard
from tf2_ros import Buffer, TransformListener
from image_geometry import PinholeCameraModel
from tf2_geometry_msgs import PoseStamped
from geometry_msgs.msg import PointStamped, Point, Pose
from vision_msgs.msg import BoundingBox2D
from std_msgs.msg import Empty
from std_srvs.srv import Trigger, TriggerResponse


def point_to_point_stamped(point: Point, frame_id='map') -> PointStamped:
    point_stamped = PointStamped()
    point_stamped.header.stamp = rospy.Time.now()
    point_stamped.header.frame_id = frame_id
    point_stamped.point.x = point.x
    point_stamped.point.y = point.y
    point_stamped.point.z = point.z

    return point_stamped


class PrecisionLanding():
    def __init__(self) -> None:
        rospy.init_node('precision_landing_node')

        self.camera_link = rospy.get_param('~camera_link', 'camera_link')
        self.offboard = Offboard()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)
        self.camera_model = PinholeCameraModel()
        self.landing_target = self.__init_landing_target()
        self.camera_info = rospy.wait_for_message('camera/camera_info', CameraInfo)
        self.timer_check_landing_status = None

        self.__init_precision_landing_params()
        self.wait_for_transform()

        self.sub_target_object_point = rospy.Subscriber("precision_landing/landing_target/bbox", BoundingBox2D, self.__callback_landing_target_bbox)
        self.pub_landing_target = rospy.Publisher("mavros/landing_target/raw", LandingTarget, queue_size=10)
        self.pub_estimated_target_object_point = rospy.Publisher("precision_landing/landing_target/estimated/pose", PointStamped, queue_size=1)
        self.pub_landing_finished = rospy.Publisher("precision_landing/finished", Empty, queue_size=1)
        self.service_start = rospy.Service("precision_landing/start", Trigger, self.__callback_service_start)

    def wait_for_transform(self):
        while not rospy.is_shutdown():
            if self.tf_buffer.can_transform(self.camera_link, "map", rospy.Time(0), rospy.Duration(1.0)):
                break

    def __init_precision_landing_params(self):
        landing_target_timeout = rospy.get_param('~landing_target_timeout', 2.0)
        search_altitude = rospy.get_param('~search_altitude', 2.0)
        final_approach_altitude = rospy.get_param('~final_approach_altitude', 0.1)
        horizontal_acceptence_radius = rospy.get_param('~horizontal_acceptence_radius', 0.1)
        max_search_attempts = rospy.get_param('~max_search_attempts', 3)
        search_timeout = rospy.get_param('~search_timeout', 10.0)

        # self.offboard.set_param('PLD_BTOUT', landing_target_timeout)
        # self.offboard.set_param('PLD_SRCH_ALT', search_altitude)
        # self.offboard.set_param('PLD_FAPPR_ALT', final_approach_altitude)
        # self.offboard.set_param('PLD_HACC_RAD', horizontal_acceptence_radius)
        # self.offboard.set_param('PLD_MAX_SRCH', max_search_attempts)
        # self.offboard.set_param('PLD_SRCH_TOUT', search_timeout)

    def __init_landing_target(self) -> LandingTarget:
        landing_target = LandingTarget()
        landing_target.header.stamp = rospy.Time.now()
        landing_target.header.frame_id = 'map'
        landing_target.frame = 1  # MAV_FRAME_LOCAL_NED
        landing_target.type = LandingTarget.VISION_OTHER

        return landing_target

    def __callback_landing_target_bbox(self, bbox: BoundingBox2D):
        self.landing_target.header.stamp = rospy.Time.now()

        self.landing_target.pose = self.bbox_to_target_pose(bbox)
        self.landing_target.size = [bbox.size_x, bbox.size_y]

        self.pub_estimated_target_object_point.publish(point_to_point_stamped(self.landing_target.pose.position))

        self.pub_landing_target.publish(self.landing_target)

    def __callback_service_start(self, req):
        response = self.offboard.set_precision_landing_mode()
        self.timer_check_landing_status = rospy.Timer(rospy.Duration(0.1), self.__callback_check_landing_status)

        return TriggerResponse(response.mode_sent, "")

    def __callback_check_landing_status(self, _):
        if self.offboard.is_landed():
            self.pub_landing_finished.publish()
            self.timer_check_landing_status.shutdown()

    def bbox_to_target_pose(self, bbox: BoundingBox2D) -> Pose:
        distance = self.offboard.rel_alt
        target_pose_camera_link = self.pixel_to_3d(bbox.center.x, bbox.center.y, distance)
        target_pose_map_link = self.tf_buffer.transform(target_pose_camera_link, "map")

        return target_pose_map_link.pose

    def pixel_to_3d(self, x, y, distance) -> PoseStamped:
        self.camera_model.fromCameraInfo(self.camera_info)
        point = self.camera_model.projectPixelTo3dRay((x, y))
        position = np.array(point) * distance

        pose = PoseStamped()
        pose.header.stamp = rospy.Time(0)
        pose.header.frame_id = self.camera_link
        pose.pose.position.x = position[0]
        pose.pose.position.y = position[1]
        pose.pose.position.z = position[2]

        return pose


if __name__ == '__main__':
    try:
        precision_landing = PrecisionLanding()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
