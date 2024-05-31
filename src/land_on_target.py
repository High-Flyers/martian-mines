import rospy

from std_srvs.srv import Trigger
from vision_msgs.msg import BoundingBox2DArray, BoundingBox2D


class Detection:
    def __init__(self):
        self.sub_bboxes = rospy.Subscriber("detection/bboxes", BoundingBox2DArray, self.__callback_bboxes)
        self.pub_landing_target = rospy.Publisher("precision_landing/landing_target/bbox", BoundingBox2D, queue_size=1)
        rospy.wait_for_service("precision_landing/start")
        self.client_precision_landing_start = rospy.ServiceProxy("precision_landing/start", Trigger)
        self.precision_landing_started = False

    def __callback_bboxes(self, bboxes: BoundingBox2DArray):
        if len(bboxes.boxes) > 0:
            self.pub_landing_target.publish(bboxes.boxes[0])

            if not self.precision_landing_started:
                self.client_precision_landing_start()
                self.precision_landing_started = True


if __name__ == '__main__':
    rospy.init_node('detection', anonymous=True)

    try:
        detection = Detection()
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
