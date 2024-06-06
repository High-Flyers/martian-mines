import rospy
import message_filters

from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64

from martian_mines.msg import FigureMsgList, FigureMsg
from utils.coords_scaler import CoordinateScaler
from utils.uploader import Uploader


class UploadNode:
    def __init__(self):
        self.drone_uploader = Uploader("http://91.227.41.24:3001/drones")
        self.figure_uploader = Uploader("http://91.227.41.24:3001/samples")

        global_pose_sub = message_filters.Subscriber("mavros/global_position/global", NavSatFix)
        alt_sub = message_filters.Subscriber("mavros/global_position/rel_alt", Float64)
        self.drone_data_ts = message_filters.ApproximateTimeSynchronizer([global_pose_sub, alt_sub], queue_size=10, slop=0.1, allow_headerless=True)
        self.drone_data_ts.registerCallback(self.drone_data_callback)

        self.figure_sub = rospy.Subscriber("figure_finder/confirmed_figures", FigureMsgList, self.figure_callback)

        start_global_pose = rospy.wait_for_message("mavros/global_position/global", NavSatFix)
        start_heading = rospy.wait_for_message("mavros/global_position/compass_hdg", Float64)
        self.coordinate_scaler = CoordinateScaler(
            start_global_pose.latitude,
            start_global_pose.longitude,
            start_heading.data
        )

    def drone_data_callback(self, global_pose, altitide):
        data = self.get_drone_request_data(global_pose, altitide)
        self.drone_uploader.add(data)

    def figure_callback(self, data):
        for fig in data.figures:
            data = self.get_figure_request_data(fig)
            self.figure_uploader.add(data)

    def get_drone_request_data(self, global_pose, rel_altitude):
        return {
            "x": global_pose.latitude,
            "y": global_pose.longitude,
            "z": rel_altitude.data
        }

    def get_figure_request_data(self, figure: FigureMsg):
        x, y = self.coordinate_scaler.scale(figure.local_x, figure.local_y)
        return {
            "x": x,
            "y": y,
            "color": self.get_figure_color(figure),
            "sample_status": figure.status,
            "sample_id": figure.id
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
    rospy.init_node("report_uploader_py")
    try:
        upload_node = UploadNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
