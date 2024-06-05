import rospy
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Pose, Vector3
from std_msgs.msg import Header, ColorRGBA
from martian_mines.msg import FigureMsgList, FigureMsg


class DetectionsVisualization:
    def __init__(self) -> None:
        self.markers_pub = rospy.Publisher("detections_visualization/markers", MarkerArray, queue_size=5)
        self.marker_id = 0
        self.figures_sub = rospy.Subscriber("figure_finder/detected_figures", FigureMsgList, self.figure_callback)
        self.color_mapping = {
            "redBall": ColorRGBA(1.0, .0, .0, .8),
            "purpleBall": ColorRGBA(0.52, .0, 0.52, .8),
            "blueBall": ColorRGBA(.0, .0, 1.0, .8),
            "yellowBall": ColorRGBA(1.0, 1.0, .0, .8)}
        self.marker_id = 0

    def figure_callback(self, data: FigureMsgList):
        marker_array = MarkerArray()
        marker_array.markers = [self.__create_marker(fig) for fig in data.figures]
        self.markers_pub.publish(marker_array)

    def __create_marker(self, fig: FigureMsg):
        return Marker(
            type=Marker.SPHERE, action=Marker.ADD, id=self.__get_marker_id(), pose=self.__get_pose(fig),
            lifetime=rospy.Duration(0), header=Header(stamp=rospy.Time.now(), frame_id="start_pose"), color=self.color_mapping[fig.type],
            scale=Vector3(0.1, 0.1, 0.1)
        )

    def __get_marker_id(self) -> int:
        self.marker_id += 1
        return self.marker_id

    def __get_pose(self, f: FigureMsg) -> Pose:
        pose = Pose()
        pose.position.x = f.local_x
        pose.position.y = f.local_y
        pose.position.z = 0
        return pose


if __name__ == "__main__":
    rospy.init_node("detections_visualization")
    try:
        detections_visualization = DetectionsVisualization()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
