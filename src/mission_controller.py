import rospy

from transitions import Machine
from enum import Enum, auto
from std_msgs.msg import Empty
from std_srvs.srv import Trigger
from typing import List
from vision_msgs.msg import BoundingBox2D

from drone.offboard import Offboard
from martian_mines.msg import FigureMsgList, FigureMsg, BoundingBoxLabeledList


class States(Enum):
    IDLE = auto()
    TAKEOFF = auto()
    SCANNING = auto()
    COLLECT_FIGURES = auto()
    TARGET_FIGURE = auto()
    FIGURE_LANDING = auto()


class MissionController(Machine):
    def __init__(self):
        self.__init_state_machine()

        self.offboard = Offboard()
        self.target_figures: List[FigureMsg] = None
        self.bboxes = BoundingBoxLabeledList()

        self.takeoff_height = rospy.get_param('~takeoff_height', 4)
        self.target_figure_types = rospy.get_param('~target_figure_types', ['blueBall', 'redBall', 'purpleBall'])
        self.target_figure_approach_height = rospy.get_param('~target_figure_approach_height', 3)

        self.pub_precision_landing_bbox = rospy.Publisher('precision_landing/landing_target/bbox', BoundingBox2D, queue_size=1)

        rospy.loginfo('Waiting for services...')
        rospy.wait_for_service('trajectory_generator/generate', timeout=5)
        self.client_generate_trajectory = rospy.ServiceProxy('trajectory_generator/generate', Trigger)
        rospy.wait_for_service('figure_finder/start', timeout=5)
        self.client_figure_finder_start = rospy.ServiceProxy('figure_finder/start', Trigger)
        rospy.wait_for_service('figure_finder/finish', timeout=5)
        self.client_figure_finder_finish = rospy.ServiceProxy('figure_finder/finish', Trigger)
        rospy.wait_for_service('precision_landing/start', timeout=5)
        self.client_precision_landing_start = rospy.ServiceProxy('precision_landing/start', Trigger)
        rospy.loginfo('Services are ready!')

        self.sub_confirmed_figures = rospy.Subscriber('figure_finder/confirmed_figures', FigureMsgList, self.cb_confirmed_figures)
        self.sub_trajectory_tracker_finished = rospy.Subscriber('trajectory_tracker/finished', Empty, self.cb_trajectory_tracker_finished)
        self.sub_precision_landing_finished = rospy.Subscriber('precision_landing/finished', Empty, self.cb_precision_landing_finished)
        self.sub_detection_bboxes = rospy.Subscriber('detection/bboxes', BoundingBoxLabeledList, self.cb_detection_bboxes)

    def __init_state_machine(self):
        transitions = [
            ['takeoff', [States.IDLE, States.FIGURE_LANDING], States.TAKEOFF],
            {'trigger': 'takeoff_finished', 'source': States.TAKEOFF, 'dest': States.SCANNING, 'conditions': 'is_need_scanning'},
            ['collect_figures', States.SCANNING, States.COLLECT_FIGURES],
            ['target_figure', [States.SCANNING, States.TAKEOFF], States.TARGET_FIGURE],
            ['land_figure', States.TARGET_FIGURE, States.FIGURE_LANDING],
        ]

        Machine.__init__(self, states=States, transitions=transitions, initial=States.IDLE)

    def is_need_scanning(self):
        return self.target_figures is None

    def on_enter_TAKEOFF(self):
        self.timer_takeoff = rospy.Timer(rospy.Duration(0.02), self.cb_timer_takeoff)
        self.offboard.takeoff(self.takeoff_height)
        rospy.sleep(0.2)
        self.offboard.start()

    def cb_timer_takeoff(self, _):
        self.offboard.takeoff(self.takeoff_height)

        if self.offboard.is_takeoff_finished(self.takeoff_height):
            self.offboard.set_hold_mode()
            self.takeoff_finished()
            self.timer_takeoff.shutdown()

    def on_enter_SCANNING(self):
        self.client_generate_trajectory()
        self.client_figure_finder_start()

    def on_enter_TARGET_FIGURE(self, target_figures: List[FigureMsg]):
        def cb_timer_fly_to_figure(figure: FigureMsg):
            figure_point = [figure.local_x, figure.local_y, self.target_figure_approach_height]
            self.offboard.fly_point(*figure_point)

            if self.offboard.is_point_reached(*figure_point):
                self.offboard.set_hold_mode()
                self.land_figure(target_figure)
                self.cb_timer_fly_to_figure.shutdown()

        target_figure = target_figures.pop()
        self.cb_timer_fly_to_figure = rospy.Timer(rospy.Duration(0.02), lambda _: cb_timer_fly_to_figure(target_figure))
        rospy.sleep(0.2)
        self.offboard.set_offboard_mode()

    def on_enter_FIGURE_LANDING(self, target_figure: FigureMsg):
        def cb_figure_landing(_, target_figure: FigureMsg):
            bboxes = self.bboxes.boxes

            if len(bboxes) == 0:
                return

            bbox = bboxes[0]
            self.pub_precision_landing_bbox.publish(bbox.bbox)

        self.timer_figure_landing = rospy.Timer(rospy.Duration(0.02), lambda _: cb_figure_landing(target_figure))
        self.client_precision_landing_start()

    def cb_precision_landing_finished(self, _):
        self.timer_figure_landing.shutdown()
        self.takeoff()

    def cb_trajectory_tracker_finished(self, _):
        self.client_figure_finder_finish()

    def cb_confirmed_figures(self, msg: FigureMsgList):
        target_figures = [fig for fig in msg.figures if fig.type in self.target_figure_types]
        target_figures.sort(key=lambda fig: self.target_figure_types.index(fig.type))
        self.target_figures = target_figures[::-1]
        self.target_figure(self.target_figures)

    def cb_detection_bboxes(self, msg: BoundingBoxLabeledList):
        self.bboxes = msg


if __name__ == "__main__":
    rospy.init_node('mission_controller')

    try:
        mission_controller = MissionController()
        mission_controller.takeoff()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
