import rospy
import rospkg
import os
import numpy as np
import cv2
from sensor_msgs.msg import Image, CompressedImage
from martian_mines.msg import BoundingBoxLabeledList
from cv_bridge import CvBridge, CvBridgeError
from detectors.aruco_detector import ArucoDetector
from detectors.yolo_detector import YoloDetector


class Detection:
    def __init__(self):
        self.bridge = CvBridge()

        self.pub_bboxes = rospy.Publisher("detection/bboxes", BoundingBoxLabeledList, queue_size=10)
        self.pub_visualization = rospy.Publisher("detection/image/compressed", CompressedImage, queue_size=2)

        rospack = rospkg.RosPack()
        package_path = rospack.get_path('martian_mines')
        detector = rospy.get_param('~detector')
        if detector == "aruco":
            self.detector = ArucoDetector()
        elif detector == "yolo":
            model_path = os.path.join(package_path, rospy.get_param('~nn_model_path'))
            self.detector = YoloDetector(model_path)
        else:
            rospy.logerr(f"Unknown detector: {detector}")
            raise ValueError(f"Unknown detector: {detector}")
        rospy.loginfo(f"Used detector: {detector}")
            
        self.sub_image = rospy.Subscriber("camera/image_raw", Image, self.image_callback)

    def publish_compressed_visualization(self, image_np: np.array):
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', image_np)[1]).tostring()
        self.pub_visualization.publish(msg)

    def image_callback(self, data: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            stamp = data.header.stamp
            bboxes = self.detector.detect(cv_image)
            bboxes_list_msg = self.__to_bboxes_msg_array(bboxes)
            bboxes_list_msg.header.stamp = stamp
            self.pub_bboxes.publish(bboxes_list_msg)
            cv_image = self.detector.draw_markers(cv_image)
            self.publish_compressed_visualization(cv_image)
            
        except CvBridgeError as e:
            rospy.logerr(e)
            return

    def __to_bboxes_msg_array(self, bboxes) -> BoundingBoxLabeledList:
        bboxes_msg = BoundingBoxLabeledList()
        bboxes_msg.boxes = bboxes
        return bboxes_msg


if __name__ == '__main__':
    rospy.init_node('detection', anonymous=True)

    try:
        detection = Detection()
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
