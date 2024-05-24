#!/usr/bin/env python

import rospy
import cv2
import time
from ultralytics import YOLO
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, NavSatFix
from std_msgs.msg import Float64
from figure.bounding_box import BoundingBox
from figure_detection.figure_manager import FigureManager
from utils.positioner import Positioner


class Detector:
    def __init__(self):
        rospy.init_node("video_subscriber", anonymous=True)
        self.real_world = rospy.get_param("~real_world")
        positioner_config = {
            "fov": 80,
            "res": [1920, 1080]
        }
        self.positioner = Positioner(positioner_config)
        self.figure_manager = FigureManager(self.positioner, False)
        self.last_frame = None
        self.last_alt_agl = None

        self.bridge = CvBridge()

        # Load the YOLOv8 model
        model_path = str(rospy.get_param("~nn_model_path"))
        self.yolo_model = YOLO(model_path, verbose=True)

        # Subscribe to the video topic
        self.image_sub = rospy.Subscriber(
            "/uav0/camera/image_raw", Image, self.image_callback
        )
        self.global_pos_sub = rospy.Subscriber(
            "/uav0/mavros/global_position/global", NavSatFix, self.global_pos_callback
        )
        self.rel_alt_sub = rospy.Subscriber(
            "/uav0/mavros/global_position/rel_alt", Float64, self.rel_alt_callback
        )

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            start_time = time.time()
            results = self.yolo_model.predict(frame, verbose=False)
            end_time = time.time()
            inference_time = end_time - start_time
            rospy.loginfo_throttle(3, f"NN inference total time {round(inference_time * 1000, 1)} ms")
            bboxes = BoundingBox.from_ultralytics(results[0].boxes, results[0].names)
            figures = self.figure_manager.create_figures(frame, bboxes, self.last_alt_agl)
            # print(figures)

            annotated_frame = results[0].plot()

            if not self.real_world:
                cv2.imshow("Adnotated Stream", annotated_frame)

                if cv2.waitKey(1) & 0xFF == ord("q"):
                    rospy.signal_shutdown("User pressed 'q'")

        except Exception as e:
            rospy.logerr(f"Error in ROS Image to OpenCV image callback: {e}")

    def global_pos_callback(self, msg):
        latitude = msg.latitude
        longitude = msg.longitude
        altitude_amsl = msg.altitude

        rospy.loginfo_throttle(
            10,
            f"Received global position: Latitude: {latitude}, Longitude: {longitude}, Altitude(AMSL): {altitude_amsl}",
        )

    def rel_alt_callback(self, msg):
        self.last_alt_agl = msg.data
        rospy.loginfo_throttle(10, f"Received relative Altitude(AGL): {msg.data}")

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    video_subscriber = Detector()
    video_subscriber.run()
    cv2.destroyAllWindows()
