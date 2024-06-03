import rospy

from std_srvs.srv import Trigger, Empty
from vision_msgs.msg import BoundingBox2D
from martian_mines.msg import BoundingBoxLabeledList


class BBoxPublisher:
    def __init__(self):
        self.precision_landing_started = False

        rospy.wait_for_service("precision_landing/start")
        self.client_precision_landing_start = rospy.ServiceProxy("precision_landing/start", Trigger)
        self.sub_precision_landing_finished = rospy.Service("precision_landing/finished", Empty,
                                                            self.__callback_precision_landing_finished)

        self.sub_bboxes = rospy.Subscriber("detection/bboxes", BoundingBoxLabeledList, self.__callback_bboxes)
        self.pub_landing_target = rospy.Publisher("precision_landing/landing_target/bbox", BoundingBox2D, queue_size=1)

    def __callback_bboxes(self, bboxes: BoundingBoxLabeledList):
        if len(bboxes.boxes) > 0:
            self.pub_landing_target.publish(bboxes.boxes[0].bbox)

            if not self.precision_landing_started:
                response = self.client_precision_landing_start()
                rospy.loginfo(f"Precision Landing response: {response}")
                self.precision_landing_started = True

    def __callback_precision_landing_finished(self, _):
        self.precision_landing_started = False
        rospy.loginfo("Precision Landing finished!")


if __name__ == '__main__':
    rospy.init_node('bbox_publisher', anonymous=True)

    try:
        detection = BBoxPublisher()
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
