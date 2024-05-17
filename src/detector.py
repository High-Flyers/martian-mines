#!/usr/bin/env python

import rospy
import cv2
from ultralytics import YOLO
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, NavSatFix
from std_msgs.msg import Float64


class Detector:
    def __init__(self):
        rospy.init_node("video_subscriber", anonymous=True)
        self.real_world = str(rospy.get_param("~real_world"))


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
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            results = self.yolo_model.predict(cv_image, verbose=False)

            annotated_frame = results[0].plot()

            if not self.real_world:
                cv2.imshow("Og Stream", cv_image)
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
            1,
            f"Received global position: Latitude: {latitude}, Longitude: {longitude}, Altitude(AMSL): {altitude_amsl}",
        )

    def rel_alt_callback(self, msg):
        rospy.loginfo_throttle(1, f"Received relative Altitude(AGL): {msg.data}")

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    video_subscriber = Detector()
    video_subscriber.run()
    cv2.destroyAllWindows()
