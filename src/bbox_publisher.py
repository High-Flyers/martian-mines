import rospy

from vision_msgs.msg import BoundingBox2D
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PointStamped, Point
from std_srvs.srv import Trigger


def point_to_point_stamped(point: Point, frame_id='map') -> PointStamped:
    point_stamped = PointStamped()
    point_stamped.header.stamp = rospy.Time.now()
    point_stamped.header.frame_id = frame_id
    point_stamped.point.x = point.x
    point_stamped.point.y = point.y
    point_stamped.point.z = point.z

    return point_stamped


class BBoxPublisher():
    def __init__(self) -> None:
        rospy.init_node('bbox_publisher_node')

        self.object_name = self.__find_object_name()
        self.pub_real_object_position = rospy.Publisher('precision_landing/landing_target/real/pose', PointStamped, queue_size=1, latch=True)
        self.pub_bbox = rospy.Publisher("precision_landing/landing_target/bbox", BoundingBox2D, queue_size=1)
        self.sub_camera_objects = rospy.Subscriber('camera/objects', ModelStates, self.__callback_camera_objects)

        rospy.wait_for_service('precision_landing/start')
        self.client_precision_landing_start = rospy.ServiceProxy('precision_landing/start', Trigger)
        rospy.loginfo(self.client_precision_landing_start())

        self.__visualize_real_object_position(self.object_name)

    def __find_object_name(self) -> str:
        camera_objects = rospy.wait_for_message('camera/objects', ModelStates)

        if len(camera_objects.name) == 0:
            raise rospy.ROSException('No objects detected in the camera!')

        return camera_objects.name[0]

    def __visualize_real_object_position(self, name: str):
        camera_objects: ModelStates = rospy.wait_for_message('/gazebo/model_states', ModelStates)
        object_index = camera_objects.name.index(name)
        object_point = camera_objects.pose[object_index].position

        object_point_stamped = point_to_point_stamped(object_point)
        self.pub_real_object_position.publish(object_point_stamped)

    def __callback_camera_objects(self, camera_objects: ModelStates):
        if self.object_name in camera_objects.name:
            object_index = camera_objects.name.index(self.object_name)
            object_point = camera_objects.pose[object_index].position
            bbox = BoundingBox2D()
            bbox.center.x = object_point.x
            bbox.center.y = object_point.y
            bbox.size_x = 0.01
            bbox.size_y = 0.01

            self.pub_bbox.publish(bbox)


if __name__ == '__main__':
    try:
        bbox_publihser = BBoxPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
