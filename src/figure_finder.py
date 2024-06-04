import rospy
import rospkg
import cv2
import yaml
import os
import message_filters
from typing import List
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, NavSatFix, CameraInfo
from std_msgs.msg import Float64
from geometry_msgs.msg import PointStamped
from std_srvs.srv import Trigger, TriggerResponse
from figure.figure import Figure
from figure_managment.figure_collector import FigureCollector
from color_detection import ColorDetection
from utils.bbox_mapper import BBoxMapper
from martian_mines.msg import BoundingBoxLabeledList, BoundingBoxLabeled, FigureMsgList


class FigureFinder:
    def __init__(self):
        rospy.init_node("figure_finder", anonymous=True)

        rospack = rospkg.RosPack()
        package_path = rospack.get_path('martian_mines')
        color_detection_config = os.path.join(package_path, rospy.get_param("~color_detection_config_file"))
        self.color_detection = ColorDetection(color_detection_config)
        figure_operations_path = os.path.join(package_path, rospy.get_param("~figure_operations_config_file"))
        self.figure_operations_config = yaml.safe_load(open(figure_operations_path))
        self.figure_colletor = FigureCollector(rospy.get_param("~figure_collector"))
        self.processing = False

        self.last_telem = {}
        self.bridge = CvBridge()

        camera_info_msg = rospy.wait_for_message('camera/camera_info', CameraInfo)
        rospy.loginfo("Camera info received")
        self.bbox_mapper = BBoxMapper(camera_info_msg)

        self.confirmed_figures_pub = rospy.Publisher(
            "detection/confirmed_figures", FigureMsgList, queue_size=10)
        self.debug_figure_pos_pub = rospy.Publisher('detection/debug_figure_pos', PointStamped, queue_size=10)

        image_sub = message_filters.Subscriber("camera/image_raw", Image)
        bboxes_sub = message_filters.Subscriber('detection/bboxes', BoundingBoxLabeledList)
        ts_image_bboxes = message_filters.TimeSynchronizer([image_sub, bboxes_sub], 10)
        ts_image_bboxes.registerCallback(self.detection_callback)

        self.global_pos_sub = rospy.Subscriber(
            "mavros/global_position/global", NavSatFix, self.global_pos_callback
        )
        self.rel_alt_sub = rospy.Subscriber(
            "mavros/global_position/rel_alt", Float64, self.rel_alt_callback
        )
        self.compass_subscriber = rospy.Subscriber('mavros/global_position/compass_hdg', Float64, self.compass_callback)
        self.service_start = rospy.Service("figure_finder/start", Trigger, self.callback_service_start)
        self.service_finish = rospy.Service("figure_finder/finish", Trigger, self.callback_service_finish)

    def callback_service_start(self, req: Trigger):
        self.processing = True
        return TriggerResponse(success=True, message="Figure finder started")

    def callback_service_finish(self, req: Trigger):
        self.processing = False
        confirmed_figures = self.figure_colletor.confirm_figures()
        rospy.loginfo(f"Confirmed figures: {confirmed_figures}")
        self.publish_confirmed_figures(confirmed_figures)

        return TriggerResponse(success=True, message="Figure finder stopped")

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

                figure = Figure(bbox_labeled.label, bbox_labeled, color=color, determined_type=determined_type)
                # print(f"Figure: {figure}")
                figures.append(figure)
            except Exception as e:
                rospy.logwarn(f"Figure creation exception: {e}")

        return figures

    def publish_confirmed_figures(self, confirmed_figures: List[Figure]):
        confirmed_msg_list = [f.to_msg(status="on_ground") for f in confirmed_figures]
        figure_msg_list = FigureMsgList()
        figure_msg_list.figures = confirmed_msg_list
        self.confirmed_figures_pub.publish(figure_msg_list)

    def publish_debug_figure_pos(self, figure: Figure):
        point = PointStamped()
        point.header.frame_id = 'start_pose'
        point.point.x = figure.local_frame_coords[0]
        point.point.y = figure.local_frame_coords[1]
        point.point.z = figure.local_frame_coords[2]
        self.debug_figure_pos_pub.publish(point)

    def map_figures_to_ground(self, figures: List[Figure], transform_time):
        mapped_figures = []
        for f in figures:
            figure_ground_position = self.bbox_mapper.bbox_to_ground_position(f.bbox, transform_time)
            if figure_ground_position is not None:
                f.local_frame_coords = figure_ground_position
                mapped_figures.append(f)

        return mapped_figures

    def detection_callback(self, image, bboxes_msg: BoundingBoxLabeledList):
        if not self.processing:
            return
        rospy.loginfo_throttle(10, "Figure finder processing...")
        frame = self.bridge.imgmsg_to_cv2(image, "bgr8")
        figures = self.create_figures(frame, bboxes_msg.boxes, self.figure_operations_config)
        figures = self.map_figures_to_ground(figures, bboxes_msg.header.stamp)

        if figures:
            self.publish_debug_figure_pos(figures[0])

        self.figure_colletor.update(figures)

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
