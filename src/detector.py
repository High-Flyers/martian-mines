#!/usr/bin/env python

import rospy
import cv2
import time
import tf2_ros
import yaml
from typing import List
from ultralytics import YOLO
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, NavSatFix, CameraInfo
from std_msgs.msg import Float64
from geometry_msgs.msg import PointStamped, Vector3Stamped
from tf2_geometry_msgs import do_transform_vector3
from image_geometry import PinholeCameraModel
from figure.bounding_box import BoundingBox
from figure_detection.figure_manager import FigureManager
from figure_detection.figure_collector import FigureCollector
from utils.positioner import Positioner, plane_line_intersection
from figure.figure import Figure


class Detector:
    def __init__(self):
        rospy.init_node("video_subscriber", anonymous=True)
        self.real_world = rospy.get_param("~real_world")
        config_file_path = rospy.get_param("~config_file_path")
        self.config = yaml.safe_load(open(config_file_path))

        self.positioner = Positioner(self.config["camera"])
        self.figure_manager = FigureManager(self.positioner, False)
        self.figure_collector = FigureCollector(self.config["figure_collector"])
        self.last_telem = {}
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.bridge = CvBridge()

        model_path = str(rospy.get_param("~nn_model_path"))
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
        print("Camera info received")

        self.camera_model = PinholeCameraModel()
        self.camera_model.fromCameraInfo(camera_info_msg)
        self.figure_pose_pub = rospy.Publisher('figure_pose', PointStamped, queue_size=10)

    def get_transform(self, base_frame="map", to_frame="camera_link_custom"):
        try:
            transform = self.tf_buffer.lookup_transform(base_frame, to_frame, rospy.Time())
            return transform
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("Failed to get transform: {}".format(e))
            return None

    def create_figures(self, frame, bboxes: List[BoundingBox]):
        figures = []

        for bbox in bboxes:
            try:
                figure_img = bbox.get_img_piece(frame)
                fig_type = bbox.label

                figure = Figure(fig_type, bbox, figure_img=figure_img)
                ray = self.camera_model.projectPixelTo3dRay(bbox.to_point())
                transform = self.get_transform("map", "camera_link_custom")
                if transform:
                    vector = Vector3Stamped()
                    vector.vector.x = ray[0]
                    vector.vector.y = -ray[1]
                    vector.vector.z = -ray[2]
                    transformed_ray = do_transform_vector3(vector, transform)

                    camera_pose = (transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z)
                    plane_normal = (0, 0, 1)  # Normal vector of the plane
                    plane_point = (0, 0, 0)    # A point on the plane (any point where z=0)
                    ray_direction = (transformed_ray.vector.x, transformed_ray.vector.y, transformed_ray.vector.z)
                    intersection = plane_line_intersection(plane_normal, plane_point, ray_direction, camera_pose)
                    if not intersection:
                        rospy.logwarn("The line is parallel to the plane, cannot get intersection.")

                    figure.local_frame_coords = intersection
            except Exception as e:
                print(f"Figure creation exception: {e}")

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
