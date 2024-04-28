import asyncio

from mavsdk import System
from .drone_base import DroneBase
from .mavsdk_drone_control import MavsdkDroneControl
from .mavlink_telemetry import MavlinkTelemetry

class Drone(DroneBase):
    def __init__(self):
        self.system = System()
        self.telemetry = MavlinkTelemetry()
        self.control = MavsdkDroneControl(self.system)

    def set_fixed_heading(self, fixed_heading):
        self.telemetry.set_fixed_heading(fixed_heading)

    async def connect(self, mavsdk_port, mavlink_port):
        self.telemetry.connect(f"udpin:0.0.0.0:{mavlink_port}")
        await self.system.connect(f"udp://:{mavsdk_port}")
        await self.__wait_for_connection()

    async def start(self):
        await self.__wait_for_global_position()
        self.telemetry.start()

    def get_telem(self):
        return self.telemetry.get_telem_data()
    
    async def __wait_for_connection(self):
        print("Waiting for drone to connect...")
        async for state in self.system.core.connection_state():
            if state.is_connected:
                print(f"-- Connected to drone!")
                break
            await asyncio.sleep(0.1)

    async def __wait_for_global_position(self):
        print("Waiting for drone to have a global position estimate...")
        async for health in self.system.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                print("-- Global position estimate OK")
                break
            await asyncio.sleep(0.1)