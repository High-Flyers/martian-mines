#!/usr/bin/env python

import rospy
import cv2
import time
import tf2_ros
import yaml
from ultralytics import YOLO
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, NavSatFix
from std_msgs.msg import Float64
from geometry_msgs.msg import PointStamped, Point
from tf2_geometry_msgs import do_transform_point
from figure.bounding_box import BoundingBox
from figure_detection.figure_manager import FigureManager
from figure_detection.figure_collector import FigureCollector
from utils.positioner import Positioner


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
        self.figure_pose_pub = rospy.Publisher('figure_pose', PointStamped, queue_size=10)


    def get_transform(self):
        try:
            transform = self.tf_buffer.lookup_transform("map", "camera_link_custom", rospy.Time())
            return transform
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("Failed to get transform: {}".format(e))
            return None

    def image_callback(self, msg):
        # try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            start_time = time.time()
            results = self.yolo_model.predict(frame, verbose=False)
            end_time = time.time()
            inference_time = end_time - start_time
            rospy.loginfo_throttle(3, f"NN inference total time {round(inference_time * 1000, 1)} ms")
            bboxes = BoundingBox.from_ultralytics(results[0].boxes, results[0].names)
            figures = self.figure_manager.create_figures(frame, bboxes, self.last_telem)
            transform = self.get_transform()
            if transform:
                for fig in figures:
                    p = PointStamped()
                    p.point = Point(*fig.local_frame_coords)
                    point_in_local = do_transform_point(p, transform)
                    fig.local_frame_coords = (point_in_local.point.x, point_in_local.point.y, point_in_local.point.z)
                    self.figure_pose_pub.publish(point_in_local)


            # print(figures)
            # self.figure_collector.update(figures)
            # confirmed_figures = self.figure_collector.confirm_figures()
            # if confirmed_figures:
            #     print("CONFIRMED FIGURES:", confirmed_figures)

            annotated_frame = results[0].plot()

            if not self.real_world:
                cv2.imshow("Adnotated Stream", annotated_frame)

                if cv2.waitKey(1) & 0xFF == ord("q"):
                    rospy.signal_shutdown("User pressed 'q'")

        # except Exception as e:
        #     rospy.logerr(f"Error in ROS Image to OpenCV image callback: {e}")

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
