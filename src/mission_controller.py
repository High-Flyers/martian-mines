import rospy
import logging

from transitions import Machine
from enum import Enum, auto
from std_srvs.srv import Trigger

from drone.offboard import Offboard


class States(Enum):
    IDLE = auto()
    TAKEOFF = auto()
    SCANNING = auto()


class MissionController(Machine):
    def __init__(self):
        self.__init_state_machine()

        self.offboard = Offboard()

        self.takeoff_height = rospy.get_param('~takeoff_height', 4)

        rospy.wait_for_service('trajectory_generator/generate')
        self.client_generate_trajectory = rospy.ServiceProxy('trajectory_generator/generate', Trigger)
        rospy.wait_for_service('figure_finder/start')
        self.client_figure_finder_start = rospy.ServiceProxy('figure_finder/start', Trigger)

    def __init_state_machine(self):
        transitions = [
            ['takeoff', States.IDLE, States.TAKEOFF],
            ['scan', States.TAKEOFF, States.SCANNING],
        ]

        Machine.__init__(self, states=States, transitions=transitions, initial=States.IDLE)

    def on_enter_TAKEOFF(self):
        self.timer_takeoff = rospy.Timer(rospy.Duration(0.02), self.cb_timer_takeoff)
        self.offboard.takeoff(self.takeoff_height)
        rospy.sleep(0.2)
        self.offboard.start()

    def cb_timer_takeoff(self, _):
        self.offboard.takeoff(self.takeoff_height)

        if self.offboard.is_takeoff_finished(self.takeoff_height):
            self.offboard.set_hold_mode()
            self.scan()
            self.timer_takeoff.shutdown()

    def on_enter_SCANNING(self):
        self.client_generate_trajectory()
        self.client_figure_finder_start()


if __name__ == "__main__":
    rospy.init_node('mission_controller')

    try:
        mission_controller = MissionController()
        mission_controller.takeoff()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
