import rospy
import numpy as np

from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.srv import CommandBool, SetMode, SetModeResponse, ParamSet, ParamSetRequest, ParamSetResponse
from mavros_msgs.msg import ExtendedState


class Offboard():
    def __init__(self) -> None:
        self.pub_setpoint_local = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=1)
        self.pub_setpoint_velocity = rospy.Publisher('mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=1)
        self.sub_local_pos = rospy.Subscriber("mavros/local_position/pose", PoseStamped, self.callback_local_pos)
        self.sub_extended_state = rospy.Subscriber("mavros/extended_state", ExtendedState, self.callback_extended_state)
        self.local_pos = PoseStamped()
        self.extended_state = ExtendedState()

        rospy.wait_for_service("mavros/cmd/arming")
        self.client_arming = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

        rospy.wait_for_service("mavros/set_mode")
        self.client_set_mode = rospy.ServiceProxy("mavros/set_mode", SetMode)

        rospy.wait_for_service("mavros/param/set")
        self.client_param_set = rospy.ServiceProxy("mavros/param/set", ParamSet)

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
    
    def set_mission_mode(self):
        self.client_set_mode(custom_mode="AUTO.MISSION")

    def set_offboard_mode(self):
        self.client_set_mode(custom_mode="OFFBOARD")
    
    def set_precision_landing_mode(self) -> SetModeResponse:
        return self.client_set_mode(custom_mode="AUTO.PRECLAND")

    def is_takeoff_finished(self, height, epsilon=0.1):
        curr_position = self.local_pos.pose.position
        return self.is_point_reached(curr_position.x, curr_position.y, height, epsilon)

    def is_point_reached(self, x, y, z, epsilon=0.1):
        diff_vector = [x - self.local_pos.pose.position.x, y - self.local_pos.pose.position.y, z - self.local_pos.pose.position.z]
        distance = np.linalg.norm(diff_vector)

        return distance < epsilon

    def is_landed(self):
        return self.extended_state.landed_state == ExtendedState.LANDED_STATE_ON_GROUND

    def fly_point(self, x, y, z):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z

        self.pub_setpoint_local.publish(pose)
    
    def fly_velocity(self, vx, vy, vz):
        twist = TwistStamped()
        twist.header.stamp = rospy.Time.now()
        twist.header.frame_id = "map"
        twist.twist.linear.x = vx
        twist.twist.linear.y = vy
        twist.twist.linear.z = vz

        self.pub_setpoint_velocity.publish(twist)
        
    def callback_local_pos(self, msg: PoseStamped):
        self.local_pos = msg

    def callback_extended_state(self, msg: ExtendedState):
        self.extended_state = msg

    def set_param(self, name: str, value) -> ParamSetResponse:
        request = ParamSetRequest()
        request.param_id = name

        if isinstance(value, float):
            request.value.real = value
        elif isinstance(value, int):
            request.value.integer = value
        else:
            raise ValueError(f"Parameter {name} value must be int or float! Got {type(value)}")

        return self.client_param_set(request)
