import rospy
import rospkg
import cv2
import tf2_ros
import yaml
import pyrr
import os
import message_filters
from typing import List
from ultralytics import YOLO
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, NavSatFix, CameraInfo
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3Stamped
from tf2_geometry_msgs import do_transform_vector3
from image_geometry import PinholeCameraModel
from figure.figure import Figure
from figure_managment.figure_collector import FigureCollector
from color_detection import ColorDetection
from martian_mines.msg import BoundingBoxLabeledList, BoundingBoxLabeled


class FigureFinder:
    def __init__(self):
        rospy.init_node("video_subscriber", anonymous=True)
        rospack = rospkg.RosPack()

        package_path = rospack.get_path('martian_mines')
        print("Package path: ", package_path)
        config_file_path = rospy.get_param("~config_file_path")
        print("Config file path: ", config_file_path)
        self.config = yaml.safe_load(open(config_file_path))
        model_path = os.path.join(package_path, self.config["nn_model_path"])
        color_detection_config = os.path.join(package_path, self.config["color_detection_config_file"])
        figure_operations_path = os.path.join(package_path, self.config["figure_operations"])
        self.figure_operations_config = yaml.safe_load(open(figure_operations_path))

        self.last_telem = {}
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.bridge = CvBridge()
        self.yolo_model = YOLO(model_path, verbose=True)
        self.figure_colletor = FigureCollector(self.config["figure_collector"])

        self.camera_model = PinholeCameraModel()
        camera_info_msg = rospy.wait_for_message('/uav0/camera/camera_info', CameraInfo)
        rospy.loginfo("Camera info received")
        self.camera_model.fromCameraInfo(camera_info_msg)
        self.color_detection = ColorDetection(color_detection_config)

        image_sub = message_filters.Subscriber("/uav0/camera/image_raw", Image)
        bboxes_sub = message_filters.Subscriber('/uav0/detection/bboxes', BoundingBoxLabeledList)

        ts_image_bboxes = message_filters.TimeSynchronizer([image_sub, bboxes_sub], 10)
        ts_image_bboxes.registerCallback(self.detection_callback)

        self.global_pos_sub = rospy.Subscriber(
            "/uav0/mavros/global_position/global", NavSatFix, self.global_pos_callback
        )
        self.rel_alt_sub = rospy.Subscriber(
            "/uav0/mavros/global_position/rel_alt", Float64, self.rel_alt_callback
        )
        self.compass_subscriber = rospy.Subscriber('/uav0/mavros/global_position/compass_hdg', Float64, self.compass_callback)

    def get_transform(self, base_frame="map", to_frame="camera_link"):
        try:
            transform = self.tf_buffer.lookup_transform(base_frame, to_frame, rospy.Time())
            return transform
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"Failed to get transform: {e}")
            return None

    def bbox_to_ground_position(self, bbox: BoundingBoxLabeled):
        ray_camera_frame = self.camera_model.projectPixelTo3dRay((bbox.bbox.center.x, bbox.bbox.center.y))
        transform = self.get_transform("start_pose", "camera_link")
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

    def create_figures(self, frame, bboxes: List[BoundingBoxLabeled], config):
        def shrink_bbox(bbox_labeled: BoundingBoxLabeled, offset_percent: float = 0.0):
            bbox_labeled.bbox.size_x = int(offset_percent * (bbox_labeled.bbox.size_x))
            bbox_labeled.bbox.size_y = int(offset_percent * (bbox_labeled.bbox.size_y))

        def get_img_piece(frame, bbox_labeled: BoundingBoxLabeled):
            x1 = int(bbox_labeled.bbox.center.x - bbox_labeled.bbox.size_x / 2)
            x2 = int(bbox_labeled.bbox.center.x + bbox_labeled.bbox.size_x / 2)
            y1 = int(bbox_labeled.bbox.center.y - bbox_labeled.bbox.size_y / 2)
            y2 = int(bbox_labeled.bbox.center.y + bbox_labeled.bbox.size_y / 2)
            return frame[y1:y2, x1:x2]

        figures = []

        for bbox_labeled in bboxes:
            try:
                # Filter by bbox width height ratio
                if bbox_labeled.label in config["ratio_verification"]["labels"]:
                    ratio = bbox_labeled.bbox.size_x / bbox_labeled.bbox.size_x
                    if ratio < 0.8 or ratio > 1.25:
                        print(f"Ratio excedeed: {ratio}")
                        continue

                # Filter by bbox area to frame ratio
                if bbox_labeled.label in config["area_verification"]["labels"]:
                    frame_area = frame.shape[0] * frame.shape[1]
                    bbox_to_image_ratio = bbox_labeled.bbox.size_x * bbox_labeled.bbox.size_y / frame_area
                    if bbox_to_image_ratio < config["area_verification"]["min_bbox_to_image_ratio"]:
                        print(f"Bbox area to frame ratio too small: {bbox_to_image_ratio}")
                        continue

                color = None
                determined_type = None

                if bbox_labeled.label in config["bbox_shrink"]["labels"]:
                    shrink_bbox(bbox_labeled, 0.5)

                if bbox_labeled.label in config["ball_color_verification"]["labels"]:
                    figure_img = get_img_piece(frame, bbox_labeled)
                    dominant_colors = self.color_detection.get_dominant_colors(figure_img, 2, show=False)
                    color = self.color_detection.get_matching_color(dominant_colors)
                    determined_type = config["ball_color_verification"]["color_mapping"].get(color, "unknown")

                if bbox_labeled.label in config["label_mapping"]["labels"]:
                    determined_type = config["label_mapping"]["mapping"].get(bbox_labeled.label, "unknown")

                local_position = self.bbox_to_ground_position(bbox_labeled)
                figure = Figure(bbox_labeled.label, bbox_labeled, local_frame_coords=local_position, color=color, determined_type=determined_type)
                # print(f"Figure: {figure}")
                figures.append(figure)
            except Exception as e:
                rospy.logwarn(f"Figure creation exception: {e}")

        return figures

    def detection_callback(self, image, bboxes_msg: BoundingBoxLabeledList):
        frame = self.bridge.imgmsg_to_cv2(image, "bgr8")
        figures = self.create_figures(frame, bboxes_msg.boxes, self.figure_operations_config)

        self.figure_colletor.update(figures)
        confirmed_figures = self.figure_colletor.confirm_figures()
        if confirmed_figures:
            print("Confirmed figures: ", confirmed_figures)

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
    figure_finder = FigureFinder()
    figure_finder.run()
    cv2.destroyAllWindows()
