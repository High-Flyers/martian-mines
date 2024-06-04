import rospy
import numpy as np

from std_srvs.srv import Trigger
from std_msgs.msg import Empty
from sensor_msgs.msg import CameraInfo

from martian_mines.msg import BoundingBoxLabeledList

from drone.offboard import Offboard
from utils.environment import Environemnt


class BarrelTracker:
    def __init__(self):
        rospy.set_param("~detector", "aruco")

        self.service_generate = rospy.Service("barrel_tracker/activate", Trigger, callback=self.activate)
        self.finished_pub = rospy.Publisher("barrel_tracker/finished", Empty, queue_size=1)
        self.bboxes_sub = rospy.Subscriber("detection/bboxes", BoundingBoxLabeledList, callback=self.detection_callback)

        self.offboard = Offboard()
        self.camera_info = rospy.wait_for_message("camera/camera_info", CameraInfo)

        self.barrel_waypoint = *Environemnt(0, 0).barrel, 1.5

    def activate(self, _):
        self.point_timer = rospy.Timer(rospy.Duration(0.02), callback=self.point_timer_callback)
        rospy.sleep(0.2)
        self.offboard.set_offboard_mode()

    def point_timer_callback(self, _):
        self.offboard.fly_point(*self.barrel_waypoint)

        if self.offboard.is_point_reached(*self.barrel_waypoint):
            self.offboard.set_hold_mode()
            self.point_timer.shutdown()

    def detection_callback(self, data):
        for bbox in data.boxes:
            if bbox.label != 0:
                continue
            self.diff_vector = np.array([self.camera_info.width, self.camera_info.height]) - bbox.center
            self.velocity_timer = rospy.Timer(rospy.Duration(0.02), callback=self.velocity_timer_callback)
            rospy.sleep(0.2)
            self.offboard.set_offboard_mode()

    def velocity_timer_callback(self, _):
        self.offboard.fly_velocity(*self.diff_vector, vz=0)

        if np.allclose(self.diff_vector, [0, 0]):
            self.offboard.set_hold_mode()
            self.point_timer.shutdown()
            self.finished_pub.publish()
            
