#!/usr/bin/env python

import rospy
import cv2
import time
import tf2_ros
import yaml
import pyrr
from typing import List
from ultralytics import YOLO
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, NavSatFix, CameraInfo
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3Stamped
from tf2_geometry_msgs import do_transform_vector3
from image_geometry import PinholeCameraModel
from figure.bounding_box import BoundingBox
from figure.figure import Figure


class Detector:
    def __init__(self):
        rospy.init_node("video_subscriber", anonymous=True)
        self.real_world = rospy.get_param("~real_world")
        config_file_path = rospy.get_param("~config_file_path")
        model_path = str(rospy.get_param("~nn_model_path"))
        self.config = yaml.safe_load(open(config_file_path))

        self.last_telem = {}
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.bridge = CvBridge()
        self.yolo_model = YOLO(model_path, verbose=True)

        self.image_sub = rospy.Subscriber(
            "/uav0/camera/image_raw", Image, self.image_callback
        )
        self.global_pos_sub = rospy.Subscriber(
            "/uav0/mavros/global_position/global", NavSatFix, self.global_pos_callback
        )
        self.rel_alt_sub = rospy.Subscriber(
            "/uav0/mavros/global_position/rel_alt", Float64, self.rel_alt_callback
        )
        self.compass_subscriber = rospy.Subscriber('/uav0/mavros/global_position/compass_hdg', Float64, self.compass_callback)
        camera_info_msg = rospy.wait_for_message('/uav0/camera/camera_info', CameraInfo)
        rospy.loginfo("Camera info received")

        self.camera_model = PinholeCameraModel()
        self.camera_model.fromCameraInfo(camera_info_msg)

    def get_transform(self, base_frame="map", to_frame="camera_link_custom"):
        try:
            transform = self.tf_buffer.lookup_transform(base_frame, to_frame, rospy.Time())
            return transform
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"Failed to get transform: {e}")
            return None

    def bbox_to_ground_position(self, bbox: BoundingBox):
        ray_camera_frame = self.camera_model.projectPixelTo3dRay(bbox.to_point())
        transform = self.get_transform("map", "camera_link_custom")
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

    def create_figures(self, frame, bboxes: List[BoundingBox]):
        figures = []

        for bbox in bboxes:
            try:
                figure_img = bbox.get_img_piece(frame)
                fig_type = bbox.label
                local_position = self.bbox_to_ground_position(bbox)
                figure = Figure(fig_type, bbox, figure_img=figure_img, local_frame_coords=local_position)
            except Exception as e:
                rospy.logwarn(f"Figure creation exception: {e}")

            figures.append(figure)

        return figures

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        start_time = time.time()
        results = self.yolo_model.predict(frame, verbose=False)
        end_time = time.time()
        inference_time = end_time - start_time
        rospy.loginfo_throttle(3, f"NN inference total time {round(inference_time * 1000, 1)} ms")
        bboxes = BoundingBox.from_ultralytics(results[0].boxes, results[0].names)
        figures = self.create_figures(frame, bboxes)
        if figures:
            print("Figures: ", figures)

        annotated_frame = results[0].plot()

        if not self.real_world:
            cv2.imshow("Adnotated Stream", annotated_frame)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                rospy.signal_shutdown("User pressed 'q'")

    def global_pos_callback(self, msg):
        self.last_telem["latitude"] = msg.latitude
        self.last_telem["longitude"] = msg.longitude
        self.last_telem["altitude_amsl"] = msg.altitude

        rospy.loginfo_throttle(
            10,
            f"Received global position: Latitude: {msg.latitude}, Longitude: {msg.longitude}, Altitude(AMSL): {msg.altitude}",
        )

    def compass_callback(self, msg):
        self.last_telem["heading"] = msg.data

    def rel_alt_callback(self, msg):
        self.last_alt_agl = msg.data
        self.last_telem["altitude"] = msg.data
        rospy.loginfo_throttle(10, f"Received relative Altitude(AGL): {msg.data}")

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    video_subscriber = Detector()
    video_subscriber.run()
    cv2.destroyAllWindows()
