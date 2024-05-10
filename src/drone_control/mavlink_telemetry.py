import threading

from time import sleep
from pymavlink import mavutil

MESSAGES_SLEEP_TIME = 0.02


class MavlinkTelemetry:
    def __init__(self):
        self.telem_data = self.create_telem_data()
        self.fixed_heading_deg = None

    def connect(self, connection_string: str):
        self.master = mavutil.mavlink_connection(connection_string)
        print("Connecting mavlink...")
        self.master.wait_heartbeat()
        print("Mavlink connected!")

    def set_fixed_heading(self, heading_deg):
        self.fixed_heading_deg = heading_deg

    def start(self):
        position_thread = threading.Thread(target=self.collect_position_data)
        euler_angle_thread = threading.Thread(target=self.collect_euler_angle_data)
        battery_thread = threading.Thread(target=self.collect_battery_data)

        position_thread.start()
        euler_angle_thread.start()
        battery_thread.start()

    def create_telem_data(self) -> dict:
        return {
            'longitude': 0,
            'latitude': 0,
            'altitude': 0,
            'heading': 0,
            'pitch': 0,
            'roll': 0,
            'yaw': 0,
            'batt': 0,
        }

    def get_telem_data(self):
        return self.telem_data

    def collect_position_data(self):
        while True:
            msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
            self.telem_data['latitude'] = msg.lat / 1e7
            self.telem_data['longitude'] = msg.lon / 1e7
            self.telem_data['altitude'] = msg.relative_alt / 1e3
            if not self.fixed_heading_deg:
                heading_deg = msg.hdg / 100
                if heading_deg < 0:
                    heading_deg = 360 + heading_deg
                self.telem_data['heading'] = heading_deg
            else:
                self.telem_data['heading'] = self.fixed_heading_deg
            sleep(MESSAGES_SLEEP_TIME)

    def collect_euler_angle_data(self):
        while True:
            msg = self.master.recv_match(type='ATTITUDE', blocking=True)
            self.telem_data['pitch'] = msg.pitch * 180.0 / 3.14159
            self.telem_data['roll'] = msg.roll * 180.0 / 3.14159
            self.telem_data['yaw'] = msg.yaw * 180.0 / 3.14159
            sleep(MESSAGES_SLEEP_TIME)

    def collect_battery_data(self):
        while True:
            msg = self.master.recv_match(type='SYS_STATUS', blocking=True)
            self.telem_data['batt'] = msg.voltage_battery / 1000.0
            sleep(MESSAGES_SLEEP_TIME)
