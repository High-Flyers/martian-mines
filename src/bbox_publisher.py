import rospy

from std_srvs.srv import Trigger, Empty
from vision_msgs.msg import BoundingBox2DArray, BoundingBox2D


class BBoxPublisher:
    def __init__(self):
        self.precision_landing_started = False

        rospy.wait_for_service("precision_landing/start")
        self.client_precision_landing_start = rospy.ServiceProxy("precision_landing/start", Trigger)
        self.service_precision_landing_end = rospy.Service("precision_landing/end", Empty, self.__callback_end)

        self.sub_bboxes = rospy.Subscriber("detection/bboxes", BoundingBox2DArray, self.__callback_bboxes)
        self.pub_landing_target = rospy.Publisher("precision_landing/landing_target/bbox", BoundingBox2D, queue_size=1)

    def __callback_bboxes(self, bboxes: BoundingBox2DArray):
        if len(bboxes.boxes) > 0:
            self.pub_landing_target.publish(bboxes.boxes[0])

            if not self.precision_landing_started:
                response = self.client_precision_landing_start()
                rospy.loginfo(f"Precision Landing response: {response}")
                self.precision_landing_started = True

    def __callback_end(self, _):
        self.precision_landing_started = False
        rospy.loginfo("Precision Landing ended!")

        return {}


if __name__ == '__main__':
    rospy.init_node('bbox_publisher', anonymous=True)

    try:
        detection = BBoxPublisher()
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
