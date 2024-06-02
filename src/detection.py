import rospy

from sensor_msgs.msg import Image
from martian_mines.msg import BoundingBoxLabeledList
from cv_bridge import CvBridge, CvBridgeError
from detectors.aruco_detector import ArucoDetector


class Detection:
    def __init__(self):
        self.bridge = CvBridge()

        self.sub_image = rospy.Subscriber("camera/image_raw", Image, self.image_callback)
        self.pub_bboxes = rospy.Publisher("detection/bboxes", BoundingBoxLabeledList, queue_size=10)
        self.pub_visualization = rospy.Publisher("detection/image", Image, queue_size=10)

        self.detector = ArucoDetector()

    def image_callback(self, data: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            bboxes = self.detector.detect(cv_image)
            bboxes_list_msg = self.__to_bboxes_msg_array(bboxes)
            self.pub_bboxes.publish(bboxes_list_msg)
            self.detector.draw_markers(cv_image)
            self.pub_visualization.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            rospy.logerr(e)
            return

    def __to_bboxes_msg_array(self, bboxes) -> BoundingBoxLabeledList:
        bboxes_msg = BoundingBoxLabeledList()
        bboxes_msg.boxes = bboxes
        return bboxes_msg


if __name__ == '__main__':
    rospy.init_node('aruco_detector', anonymous=True)

    try:
        detection = Detection()
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
