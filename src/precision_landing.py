import rospy
import numpy as np

from gazebo_msgs.msg import ModelStates
from mavros_msgs.msg import LandingTarget
from sensor_msgs.msg import CameraInfo
from drone.offboard import Offboard
from tf2_ros import Buffer, TransformListener
from image_geometry import PinholeCameraModel
from tf2_geometry_msgs import PoseStamped
from geometry_msgs.msg import PointStamped, Point
from vision_msgs.msg import BoundingBox2D
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

        self.camera_objects = ModelStates()
        self.camera_link = rospy.get_param('~camera_link', 'cgo3_camera_link')
        self.offboard = Offboard()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)
        self.camera_model = PinholeCameraModel()
        self.landing_target = self.__init_landing_target()
        self.camera_info = rospy.wait_for_message('camera/camera_info', CameraInfo)

        self.wait_for_transform()

        self.sub_target_object_point = rospy.Subscriber("precision_landing/landing_target/bbox", BoundingBox2D, self.__callback_landing_target_bbox)
        self.pub_landing_target = rospy.Publisher("mavros/landing_target/raw", LandingTarget, queue_size=10)
        self.pub_estimated_target_object_point = rospy.Publisher("precision_landing/landing_target/estimated/pose", PointStamped, queue_size=1)
        self.service_start = rospy.Service("precision_landing/start", Trigger, self.__callback_service_start)


    def wait_for_transform(self):
        while not rospy.is_shutdown():
            if self.tf_buffer.can_transform(self.camera_link, "map", rospy.Time(0), rospy.Duration(1.0)):
                break
    
    def __init_landing_target(self) -> LandingTarget:
        landing_target = LandingTarget()
        landing_target.header.stamp = rospy.Time.now()
        landing_target.header.frame_id = 'map'
        landing_target.frame = 1  # MAV_FRAME_LOCAL_NED
        landing_target.type = LandingTarget.VISION_OTHER

        return landing_target

    def __callback_landing_target_bbox(self, bbox: BoundingBox2D):
        self.landing_target.header.stamp = rospy.Time.now()

        distance = self.offboard.local_pos.pose.position.z
        target_pose_camera_link = self.pixel_to_3d(bbox.center.x, bbox.center.y, distance)
        target_pose_map_link = self.tf_buffer.transform(target_pose_camera_link, "map")

        self.landing_target.pose = target_pose_map_link.pose
        self.landing_target.size = [bbox.size_x, bbox.size_y]

        self.pub_estimated_target_object_point.publish(point_to_point_stamped(self.landing_target.pose.position))

        self.pub_landing_target.publish(self.landing_target)

    def __callback_service_start(self, req):
        self.offboard.set_precision_landing_mode()

        return TriggerResponse(True, "Precision landing started!")

    def pixel_to_3d(self, x, y, distance) -> PoseStamped:
        self.camera_model.fromCameraInfo(self.camera_info)
        point = self.camera_model.projectPixelTo3dRay((x, y))
        position = np.array(point) * distance

        pose = PoseStamped()
        pose.header.stamp = rospy.Time(0)
        pose.header.frame_id = self.camera_link
        pose.pose.position.x = position[2]
        pose.pose.position.y = -position[0]
        pose.pose.position.z = -position[1]

        return pose


if __name__ == '__main__':
    try:
        precision_landing = PrecisionLanding()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
