from dataclasses import dataclass

import rospy

from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64

from martian_mines.msg import FigureMsgList, FigureMsg
from utils.coords_scaler import CoordinateScaler
from utils.uploader import Uploader


@dataclass
class DroneData:
    global_pose: NavSatFix = None
    heading: Float64 = None
    rel_altitude: Float64 = None

    def is_complete(self) -> bool:
        return all(param is not None for param in (self.global_pose, self.heading, self.rel_altitude))

    def reset(self) -> None:
        self.global_pose = None
        self.heading = None
        self.rel_altitude = None


class UploadNode:
    def __init__(self):
        self.global_pose_sub = rospy.Subscriber("uav0/mavros/global_position/global", NavSatFix, self.global_pose_callback)
        self.heading_sub = rospy.Subscriber("uav0/mavros/global_position/compass_hdg", Float64, self.heading_callback)
        self.alt_sub = rospy.Subscriber("uav0/mavros/global_position/rel_alt", Float64, self.altitude_callback)
        self.figure_sub = rospy.Subscriber("uav0/detection/confirmed_figures", FigureMsgList, self.figure_callback)
        self.start_pose = DroneData()
        self.drone_uploader = Uploader("http://91.227.41.24:3001/drones")
        self.figure_uploader = Uploader("http://91.227.41.24:3001/samples")
        self.drone_data = DroneData()
        self.rate = rospy.Rate(10)
        self.ready = False
        self.figures = None

    def run(self):
        while True:
            if self.ready:
                if self.drone_data.is_complete():
                    data = self.get_drone_request_data()
                    self.drone_uploader.add(data)
                    self.drone_data.reset()
                if self.figures is not None:
                    for fig in self.figures:
                        data = self.get_figure_request_data(fig)
                        self.figure_uploader.add(data)
                    self.figures = None
                self.rate.sleep()
                continue

            if self.start_pose.global_pose is None or self.start_pose.heading is None:
                rospy.loginfo("Waiting for start_pose")
                continue

            self.coordinate_scaler = CoordinateScaler(
                self.start_pose.global_pose.latitude,
                self.start_pose.global_pose.longitude,
                self.start_pose.heading.data
            )

            self.ready = True

    def global_pose_callback(self, data):
        if self.start_pose.global_pose is None:
            self.start_pose.global_pose = data
            return
        self.drone_data.global_pose = data

    def heading_callback(self, data):
        if self.start_pose.heading is None:
            self.start_pose.heading = data
            return
        self.drone_data.heading = data

    def altitude_callback(self, data):
        self.drone_data.rel_altitude = data

    def figure_callback(self, data):
        self.figures = data.figures

    def get_drone_request_data(self):
        return {
            "x": self.drone_data.global_pose.latitude,
            "y": self.drone_data.global_pose.longitude,
            "z": self.drone_data.rel_altitude.data
        }

    def get_figure_request_data(self, figure: FigureMsg):
        x, y = self.coordinate_scaler.scale(figure.local_x, figure.local_y)
        return {
            "x": x,
            "y": y,
            "color": self.get_figure_color(figure),
            "sample_status": figure.status
        }

    def get_figure_color(self, figure):
        if figure.type == "yellowBall":
            return "y"
        if figure.type == "redBall":
            return "r"
        if figure.type == "purpleBall":
            return "p"
        if figure.type == "blueBall":
            return "b"


if __name__ == "__main__":
    rospy.init_node("upload_node_py")
    upload_node = UploadNode()
    upload_node.run()
