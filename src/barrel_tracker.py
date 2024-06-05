import rospy
import numpy as np

from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import Empty
from sensor_msgs.msg import CameraInfo

from martian_mines.msg import BoundingBoxLabeledList

from drone.offboard import Offboard
from utils.environment import Environment


class BarrelTracker:
    def __init__(self, alt=4):
        rospy.set_param("~detector", "yolo")

        self.offboard = Offboard()
        camera_info = rospy.wait_for_message("camera/camera_info", CameraInfo)
        self.camera_center = np.array([camera_info.width, camera_info.height]) * 0.5

        self.barrel_waypoint = (*Environment(0, 0).barrel, alt)

        self.service_generate = rospy.Service("barrel_tracker/activate", Trigger, handler=self.activate)
        self.finished_pub = rospy.Publisher("barrel_tracker/finished", Empty, queue_size=1)
        self.bboxes_sub = rospy.Subscriber("detection/bboxes", BoundingBoxLabeledList, callback=self.detection_callback)

        rospy.wait_for_service("figure_finder/finish")

    def activate(self, _):
        self.point_timer = rospy.Timer(rospy.Duration(0.02), callback=self.point_timer_callback)
        rospy.sleep(0.2)
        self.offboard.set_offboard_mode()
        return TriggerResponse(success=True)

    def point_timer_callback(self, _):
        self.offboard.fly_point(*self.barrel_waypoint)

        if self.offboard.is_point_reached(*self.barrel_waypoint):
            self.offboard.set_hold_mode()
            self.point_timer.shutdown()
            rospy.wait_for_service("figure_finder/start")

    def detection_callback(self, data):
        for bbox in data.boxes:
            if bbox.label != "Barrel":
                continue
            bbox = bbox.bbox
            self.diff_vector = self.camera_center - np.array([bbox.center.x, bbox.center.y])
            self.velocity_timer = rospy.Timer(rospy.Duration(0.02), callback=self.velocity_timer_callback)
            rospy.sleep(0.2)
            self.offboard.set_offboard_mode()

    def velocity_timer_callback(self, _):
        velocities = self.get_velocities(self.diff_vector)
        self.offboard.fly_velocity(*velocities, vz=0)

        if np.allclose(self.diff_vector, [0, 0]):
            self.offboard.set_hold_mode()
            self.point_timer.shutdown()
            self.finished_pub.publish()
            rospy.wait_for_service("figure_finder/finish")

    def get_velocities(self, target_vector):
        distance = np.linalg.norm(target_vector)
        velocity_factor = -2 ** (-(distance) + 1) + 2
        return target_vector * velocity_factor / distance


if __name__ == "__main__":
    rospy.init_node("barrel_tracker_py")
    try:
        barrel_tracker = BarrelTracker()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
