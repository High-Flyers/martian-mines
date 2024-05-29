import rospy
import numpy as np

from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, SetMode


class Offboard():
    def __init__(self) -> None:
        self.pub_setpoint_local = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
        self.sub_local_pos = rospy.Subscriber("mavros/local_position/pose", PoseStamped, self.callback_local_pos)
        self.local_pos = PoseStamped()

        rospy.wait_for_service("mavros/cmd/arming")
        self.client_arming = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

        rospy.wait_for_service("mavros/set_mode")
        self.client_set_mode = rospy.ServiceProxy("mavros/set_mode", SetMode)

    def start(self):
        self.arm()
        self.set_offboard_mode()

    def takeoff(self, height):
        curr_position = self.local_pos.pose.position
        self.fly_point(curr_position.x, curr_position.y, height)

    def arm(self):
        self.client_arming(True)

    def disarm(self):
        self.client_arming(False)

    def set_offboard_mode(self):
        self.client_set_mode(custom_mode="OFFBOARD")

    def set_precision_landing_mode(self):
        self.client_set_mode(custom_mode="AUTO.PRECLAND")
    
    def is_takeoff_finished(self, height, epsilon=0.1):
        curr_position = self.local_pos.pose.position
        return self.is_point_reached(curr_position.x, curr_position.y, height, epsilon)

    def is_point_reached(self, x, y, z, epsilon=0.1):
        diff_vector = [x - self.local_pos.pose.position.x, y - self.local_pos.pose.position.y, z - self.local_pos.pose.position.z]
        distance = np.linalg.norm(diff_vector)
        
        return distance < epsilon

    def fly_point(self, x, y, z):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z

        self.pub_setpoint_local.publish(pose)

    def callback_local_pos(self, msg: PoseStamped):
        self.local_pos = msg
