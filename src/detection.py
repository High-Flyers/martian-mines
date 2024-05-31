import rospy

from sensor_msgs.msg import Image
from vision_msgs.msg import BoundingBox2DArray
from cv_bridge import CvBridge, CvBridgeError
from detectors.aruco_detector import ArucoDetector


class Detection:
    def __init__(self):
        self.bridge = CvBridge()

        self.sub_image = rospy.Subscriber("camera/image_raw", Image, self.image_callback)
        self.pub_bboxes = rospy.Publisher("detection/bboxes", BoundingBox2DArray, queue_size=10)
        self.pub_visualization = rospy.Publisher("detection/image", Image, queue_size=10)

        self.detector = ArucoDetector()

    def image_callback(self, data: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            bboxes, labels = self.detector.detect(cv_image)
            bboxes_array = self.__to_bboxes_array(bboxes)
            self.pub_bboxes.publish(bboxes_array)
            self.detector.draw_markers(cv_image)
            self.pub_visualization.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            rospy.logerr(e)
            return

    def __to_bboxes_array(self, bboxes) -> BoundingBox2DArray:
        bboxes_msg = BoundingBox2DArray()
        bboxes_msg.header.stamp = rospy.Time.now()
        bboxes_msg.boxes = bboxes

        return bboxes_msg

if __name__ == '__main__':
    rospy.init_node('aruco_detector', anonymous=True)

    try:
        detection = Detection()
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
