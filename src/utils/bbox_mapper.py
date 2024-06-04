import tf2_ros
import rospy
import pyrr
from image_geometry import PinholeCameraModel
from tf2_geometry_msgs import do_transform_vector3
from geometry_msgs.msg import Vector3Stamped
from martian_mines.msg import BoundingBoxLabeled


class BBoxMapper:
    "Map bbox to a plane which contains startpose and its normal vector is equal (0,0,1)"

    def __init__(self, camera_info_msg, base_frame="start_pose", camera_frame="camera_link"):
        self.camera_model = PinholeCameraModel()
        self.camera_model.fromCameraInfo(camera_info_msg)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.base_frame = base_frame
        self.camera_frame = camera_frame

    def set_base_frame(self, base_frame):
        self.base_frame = base_frame

    def set_camera_frame(self, camera_frame):
        self.camera_frame = camera_frame

    def get_transform(self, transform_time):
        try:
            transform = self.tf_buffer.lookup_transform(self.base_frame, self.camera_frame, transform_time)
            return transform
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"Failed to get transform: {e}")
            return None

    def bbox_to_ground_position(self, bbox: BoundingBoxLabeled, transform_time=rospy.Time()):
        ray_camera_frame = self.camera_model.projectPixelTo3dRay((bbox.bbox.center.x, bbox.bbox.center.y))
        transform = self.get_transform(transform_time)
        if transform:
            vector = Vector3Stamped()
            vector.vector.x = ray_camera_frame[0]
            vector.vector.y = ray_camera_frame[1]
            vector.vector.z = ray_camera_frame[2]
            transformed_ray = do_transform_vector3(vector, transform)

            camera_position = (transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z)
            ray_direction = (transformed_ray.vector.x, transformed_ray.vector.y, transformed_ray.vector.z)
            ray_local_frame = pyrr.ray.create(camera_position, ray_direction)
            plane = pyrr.plane.create()
            intersection_point = pyrr.geometric_tests.ray_intersect_plane(ray_local_frame, plane)
            if intersection_point is not None:
                return intersection_point
            rospy.logwarn("The line is parallel to the plane, cannot get intersection.")

        return (0, 0, 0)
