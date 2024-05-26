import rospy
import math
import numpy as np

from gazebo_msgs.msg import ModelStates
from mavros_msgs.msg import LandingTarget
from sensor_msgs.msg import CameraInfo
from drone.offboard import Offboard
from tf2_ros import Buffer, TransformListener
from image_geometry import PinholeCameraModel
from tf2_geometry_msgs import PoseStamped
from geometry_msgs.msg import PointStamped, Point


class TargetObject():
    def __init__(self, x, y, distance, name) -> None:
        self.x = x
        self.y = y
        self.distance = distance
        self.width = 0.1
        self.height = 0.1
        self.name = name


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
        self.camera_objects = ModelStates()
        self.camera_info_received = False
        self.target_object = None
        self.target_object_name = "yellow_ball_0"
        self.target_object_point = self.__get_target_object_position()
        self.offboard = Offboard()
        self.rate = rospy.Rate(10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)
        self.camera_model = PinholeCameraModel()

        self.sub_camera_objects = rospy.Subscriber('camera/objects', ModelStates, self.callback_camera_objects)
        self.pub_landing_target = rospy.Publisher("mavros/landing_target/raw", LandingTarget, queue_size=10)
        self.pub_target_object_point = rospy.Publisher("target_object_point", PointStamped, queue_size=1, latch=True)
        self.pub_estimated_target_object_point = rospy.Publisher("estimated_target_object_point", PointStamped, queue_size=1)

        self.pub_target_object_point.publish(point_to_point_stamped(self.target_object_point))
        self.camera_info = rospy.wait_for_message('camera/camera_info', CameraInfo)

    def wait_for_transform(self):
        while not rospy.is_shutdown():
            if self.tf_buffer.can_transform(self.camera_info.header.frame_id, "map", rospy.Time(0), rospy.Duration(1.0)):
                break

            self.rate.sleep()

    def callback_camera_objects(self, msg):
        self.camera_objects = msg

    def callback_camera_info(self, msg):
        self.camera_info = msg

    def __get_target_object_position(self) -> np.ndarray:
        model_states = rospy.wait_for_message('/gazebo/model_states', ModelStates)

        index = model_states.name.index(self.target_object_name)
        point = model_states.pose[index].position

        return point

    def __find_target_object(self):
        try:
            index = self.camera_objects.name.index(self.target_object_name)

            object_position = self.camera_objects.pose[index].position
            self.target_object = TargetObject(object_position.x,
                                              object_position.y,
                                              object_position.z,
                                              self.target_object_name)
        except ValueError:
            self.target_object = None

    def takeoff(self, height):
        offboard_count = 0
        rospy.loginfo("Taking off!")

        while not rospy.is_shutdown():
            offboard_count += 1

            if offboard_count > 10:
                self.offboard.start()

            self.offboard.takeoff(height)

            if self.offboard.is_takeoff_finished(height):
                break

            self.rate.sleep()

    def fly_to_target(self, height):
        rospy.loginfo("Flying to target!")
        while not rospy.is_shutdown():
            if self.target_object_point:
                target_point = np.array([self.target_object_point.x, self.target_object_point.y, height])
                target_point[0] -= 1
                target_point[1] -= 2
                self.offboard.fly_point(*target_point)

                if self.offboard.is_point_reached(*target_point):
                    break

            self.rate.sleep()

    def land_on_target(self):
        rospy.loginfo("Landing on target!")
        self.offboard.set_precision_landing_mode()
        while not rospy.is_shutdown():
            self.__find_target_object()

            if self.target_object is None:
                continue

            landing_target = LandingTarget()
            landing_target.header.stamp = rospy.Time.now()
            landing_target.header.frame_id = 'map'
            landing_target.frame = 1  # MAV_FRAME_LOCAL_NED

            target_pose_camera_link = self.pixel_to_3d(self.target_object.x, self.target_object.y, self.target_object.distance)
            target_pose_map_link = self.tf_buffer.transform(target_pose_camera_link, "map")

            landing_target.pose = target_pose_map_link.pose
            landing_target.distance = self.target_object.distance
            landing_target.size = [self.target_object.width, self.target_object.height]
            landing_target.type = LandingTarget.VISION_OTHER

            self.pub_estimated_target_object_point.publish(point_to_point_stamped(landing_target.pose.position))

            self.pub_landing_target.publish(landing_target)

    def pixel_to_3d(self, x, y, distance) -> PoseStamped:
        self.camera_model.fromCameraInfo(self.camera_info)
        point = self.camera_model.projectPixelTo3dRay((x, y))
        position = np.array(point) * distance

        pose = PoseStamped()
        pose.header.stamp = rospy.Time(0)
        pose.header.frame_id = self.camera_info.header.frame_id
        pose.pose.position.x = position[2]
        pose.pose.position.y = -position[0]
        pose.pose.position.z = -position[1]

        return pose

    def run(self):
        self.takeoff(5)
        self.fly_to_target(5)
        self.wait_for_transform()
        self.land_on_target()


if __name__ == '__main__':
    try:
        rospy.init_node('precision_landing_node')
        precision_landing = PrecisionLanding()
        precision_landing.run()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
