import random

from typing import List

import rospy
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Pose, Vector3
from std_msgs.msg import Header, ColorRGBA

from utils.environment import Environemnt


class RvizEnvironment:
    def __init__(self, marker_pub_address: str = "/visualization_marker_array") -> None:
        self.marker_pub = rospy.Publisher(marker_pub_address, MarkerArray, queue_size=100, latch=True)
        self.env = Environemnt(0, 0)
        self.poses = (
            self.env.left_central_ball, self.env.left_lower_ball, self.env.left_upper_ball,
            self.env.middle_central_ball, self.env.middle_lower_ball, self.env.middle_upper_ball,
            self.env.right_central_ball, self.env.right_lower_ball, self.env.right_upper_ball
        )
        self.marker_id = 0
        self.markers = MarkerArray([*self.__get_balls(), *self.__get_squares(), self.__get_barrel()])

    def generate_map(self) -> None:
        self.marker_pub.publish(self.markers)

    def __get_balls(self):
        return [
            Marker(
                type=Marker.SPHERE, action=Marker.ADD, id=self.__get_marker_id(), pose=self.__get_pose(x, y),
                lifetime=rospy.Duration(0), header=Header(frame_id="map"), color=color, scale=Vector3(0.06, 0.06, 0.06)
            ) for color, (x, y) in zip(self.__get_colors(), self.poses)
        ]

    def __get_colors(self) -> List[ColorRGBA]:
        color_list = [ColorRGBA(1.0, .0, .0, .8), ColorRGBA(0.52, .0, 0.52, .8), ColorRGBA(.0, .0, 1.0, .8)] + [ColorRGBA(1.0, 1.0, .0, .8)] * 6
        random.shuffle(color_list)
        return color_list

    def __get_squares(self):
        return [
            Marker(
                type=Marker.CUBE, action=Marker.ADD, id=self.__get_marker_id(), pose=self.__get_pose(x, y),
                lifetime=rospy.Duration(0), header=Header(frame_id="map"), color=ColorRGBA(1.0, 1.0, 1.0, 1.0),
                scale=Vector3(1.0, 1.0, 0.01)
            ) for x, y in self.poses
        ]

    def __get_barrel(self):
        return Marker(
            type=Marker.CYLINDER, action=Marker.ADD, id=self.__get_marker_id(), pose=self.__get_pose(*self.env.barrel),
            lifetime=rospy.Duration(0), header=Header(frame_id="map"), color=ColorRGBA(.0, .0, 1.0, .8), scale=Vector3(.58, .58, 1.2)
        )

    def __get_marker_id(self) -> int:
        self.marker_id += 1
        return self.marker_id

    def __get_pose(self, x, y) -> Pose:
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = 0
        return pose


if __name__ == "__main__":
    rospy.init_node("rviz_environment_py")
    rviz_environment = RvizEnvironment()
    while True:
        rviz_environment.generate_map()
