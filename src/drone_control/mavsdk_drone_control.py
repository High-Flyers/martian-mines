import asyncio
import numpy as np

from mavsdk import System
from mavsdk.offboard import PositionGlobalYaw, PositionNedYaw, VelocityBodyYawspeed, VelocityNedYaw
from utils.positioner import get_coords_distance

OFFSET_M = 0.2
GLOBAL_ALT_OFFSET_M = 0.8

class MavsdkDroneControl:
    def __init__(self, system: System):
        self.drone = system

    async def arm(self):
        await self.drone.action.arm()

    async def takeoff(self, altitude):
        await self.drone.action.set_takeoff_altitude(altitude)
        await self.drone.action.takeoff()
        await self.__wait_for_takeoff(altitude)

    async def offboard_start(self):
        await self.__offboard_init()
        await self.drone.offboard.start()
    
    async def offboard_stop(self):
        await self.drone.offboard.stop()

    async def set_param(self, name, value):
        await self.drone.param.set_param_int(name, value)
    
    async def goto_global(self, lat, lon, alt_m, yaw_deg=0):
        position = PositionGlobalYaw(lat, lon, alt_m, yaw_deg, PositionGlobalYaw.AltitudeType.REL_HOME)
        await self.drone.offboard.set_position_global(position)
    
    async def goto_global_wait(self, lat, lon, alt_m, yaw_deg=0):
        await self.goto_global(lat, lon, alt_m, yaw_deg)
        await self.__wait_for_goto_global(lat, lon, alt_m)
    
    async def goto_local(self, north_m, east_m, down_m, yaw_deg=0):
        position = PositionNedYaw(north_m, east_m, down_m, yaw_deg)
        await self.drone.offboard.set_position_ned(position)
    
    async def velocity_body(self, forward_m_s, right_m_s, down_m_s, yaw_deg_s=0):
        velocity = VelocityBodyYawspeed(forward_m_s, right_m_s, down_m_s, yaw_deg_s)
        await self.drone.offboard.set_velocity_body(velocity)

    async def velocity_local(self, north_m_s, east_m_s, down_m_s, yaw_deg=0):
        velocity = VelocityNedYaw(north_m_s, east_m_s, down_m_s, yaw_deg)
        await self.drone.offboard.set_velocity_ned(velocity)
    
    async def return_to_launch(self):
        await self.drone.action.return_to_launch()

    async def land(self):
        await self.drone.action.land()

    async def __offboard_init(self):
        position = await self.drone.telemetry.position().__anext__()
        heading = await self.drone.telemetry.heading().__anext__()
        await self.goto_global(position.latitude_deg, position.longitude_deg, position.relative_altitude_m, heading.heading_deg)

    async def __wait_for_takeoff(self, target_alt):
        print("Waiting for drone to takeoff...")
        async for position in self.drone.telemetry.position():
            curr_altitude = position.relative_altitude_m
            if abs(target_alt - curr_altitude) <= OFFSET_M:
                print("-- Takeoff finished!")
                break

    async def __wait_for_goto_global(self, lat, lon, alt_m):
        print(f"Waiting for drone to go to {lat}, {lon}...")
        async for position in self.drone.telemetry.position():
            curr_altitude = position.relative_altitude_m
            coords_distance = get_coords_distance((lat, lon), (position.latitude_deg, position.longitude_deg))
            altitude_ditance = abs(alt_m - curr_altitude)
            if altitude_ditance <= GLOBAL_ALT_OFFSET_M and coords_distance <= OFFSET_M:
                print("-- Go to finished!")
                break
