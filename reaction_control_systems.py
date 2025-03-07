# -*- coding:utf8 -*-
# !/usr/bin/env python3
# Copyright 2025 Octadero.
# Author Volodymyr Pavliukevych

"""This is a Reaction Control Systems
"""

from time import time
from pymavlink import mavutil
from pymavlink.mavutil import mavudp, mavtcp
from pymavlink.dialects.v20.ardupilotmega import MAVLink_set_position_target_local_ned_message
# from pymavlink.dialects.v20.ardupilotmega import set_position_target_local_ned_send
from pymavlink.dialects.v20.ardupilotmega import MAVLink

from pymavlink.dialects.v20.ardupilotmega import COPTER_MODE_GUIDED_NOGPS, MAV_TYPE_QUADROTOR

from utilities.connect_to_sysid import connect_to_sysid
from utilities.wait_for_position_aiding import wait_until_position_aiding
from utilities.get_autopilot_info import get_autopilot_info

from time import sleep
import math


from environment import FlyState, FlyTask, FlyControlException, FlyControlExceptionCode

class ReactionControlSystems():
    def __init__(self, the_connection: mavudp | mavtcp):
        self.the_connection = the_connection

    def send_motors_off(self):
        self.send_attitude_target(thrust=0)
        self.the_connection.arducopter_disarm()
        self.the_connection.motors_disarmed_wait()

    def send_motors_on(self, force_thrust: bool = False):
        self.the_connection.arducopter_arm()
        if self.the_connection.motors_armed() == False:
            print("Motors are not ready")
            
            while self.the_connection.motors_armed() == False:
                self.the_connection.arducopter_arm()
                print("arming...")
                sleep(1.0)
#        if force_thrust:
#            self.send_attitude_target(thrust=0.6)


    # Function to send attitude target
    def send_attitude_target(self, roll=0, pitch=0, yaw=0, thrust=0.5):
        # Arm the drone
        self.send_motors_on()

        while True:
            sysid = self.the_connection.sysid
            if self.the_connection.sysid_state[sysid].armed:
                break
            else:
                print("Drone not armed.")
                sleep(0.1)


        #print("Drone armed.")
        """
        Sends a MAVLink SET_ATTITUDE_TARGET message.
        roll, pitch, yaw are in radians.
        thrust is from 0.0 (no thrust) to 1.0 (full thrust).
        """
        q = [
            math.cos(yaw / 2),  # w
            roll,               # x (roll)
            pitch,              # y (pitch)
            math.sin(yaw / 2)   # z (yaw)
        ]
        
        self.the_connection.mav.set_attitude_target_send(
            int(time()),  # Timestamp in microseconds
            1,  # Target system (drone ID)
            1,  # Target component
            0b00000000,  # Type mask: No masking (control roll, pitch, yaw, and thrust)
            q,  # Quaternion (attitude)
            0,  # Body roll rate
            0,  # Body pitch rate
            0,  # Body yaw rate
            thrust  # Thrust (0 to 1)
        )

    def guided_no_gps_exec_taking_off(self):
        # Takeoff sequence
        takeoff_thrust = 0.6  # Adjust thrust value if needed

        for _ in range(150):  # Send multiple commands to ensure takeoff
            self.send_attitude_target(thrust=takeoff_thrust)
            print(f"send_attitude_target: {takeoff_thrust}")
            sleep(0.1)        

    def guided_no_gps_exec_altitude(self):
        hover_thrust = 0.5

        for _ in range(100):  # Maintain altitude
            self.send_attitude_target(thrust=hover_thrust)
            sleep(0.1)

    def guided_exec_position_target_local_ned_message(self, xyz: tuple, vxvyvz: tuple, yaw_params: tuple):
        type_mask = 0b0000111111000111  # Ignore acceleration & force
        
        x, y, z = xyz
        vx, vy, vz = vxvyvz
        yaw, yaw_rate = yaw_params

        self.the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
            0, 
            self.the_connection.target_system, 
            self.the_connection.target_component, 
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, 
            int(type_mask), 
            x, y, z, # x, y, z positions
            vx, vy, vz, # x, y, z velocity
            0, 0, 0, # x, y, z acceleration
            yaw, yaw_rate)) #yaw, yaw_rate

    def guided_exec_position_target_local_ned_send(self, xyz: tuple, vxvyvz: tuple, yaw_params: tuple):
        type_mask = 0b0000111111000111  # Ignore acceleration & force
        
        x, y, z = xyz
        vx, vy, vz = vxvyvz
        yaw, yaw_rate = yaw_params
        
        self.the_connection.mav.set_position_target_local_ned_send(
            int(time()),        #Timestamp in milliseconds
            self.the_connection.target_system,      # Target system ID
            self.the_connection.target_component,   # Target component ID
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,    # Use NED frame
            int(type_mask),     # Ignore some fields
            x, y, z,            # Position (meters)
            vx, vy, vz,         # Velocity (m/s)
            0, 0, 0,            # Acceleration (not used)
            yaw, yaw_rate       # Yaw and yaw rate
        )

    def guided_exec_land(self, tgt_sys_id: int = 1, tgt_comp_id=1):
        autopilot_info = get_autopilot_info(self.the_connection, tgt_sys_id)
        if autopilot_info["autopilot"] == "ardupilotmega":
            mode = self.the_connection.mode_mapping()
            mode_id = mode.get("GUIDED")

        elif autopilot_info["autopilot"] == "px4":
            mode_id = self.the_connection.mode_mapping()["TAKEOFF"][1]
            msg = self.the_connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        else:
            raise FlyControlException(code=FlyControlExceptionCode.AUTO_PILOT_NOT_SUPPORTED)

        args = [tgt_sys_id, tgt_comp_id, mavutil.mavlink.MAV_CMD_NAV_LAND, 0, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id, 0, 0, 0, 0, 0]
        self.the_connection.mav.command_long_send(*args)

    def guided_exec_takeoff(self, takeoff_altitude: float, tgt_sys_id: int = 1, tgt_comp_id=1):
        print(f"execut take off: {takeoff_altitude}m")
        wait_until_position_aiding(self.the_connection)

        autopilot_info = get_autopilot_info(self.the_connection, tgt_sys_id)

        if autopilot_info["autopilot"] == "ardupilotmega":
            mode = self.the_connection.mode_mapping()
            mode_id = mode.get("GUIDED")
            takeoff_params = [0, 0, 0, 0, 0, 0, takeoff_altitude]

        elif autopilot_info["autopilot"] == "px4":
            mode_id = self.the_connection.mode_mapping()["TAKEOFF"][1]
            msg = self.the_connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
            starting_alt = msg.alt / 1000
            takeoff_params = [0, 0, 0, 0, float("NAN"), float("NAN"), starting_alt + takeoff_altitude]
        else:
            raise FlyControlException(code=FlyControlExceptionCode.AUTO_PILOT_NOT_SUPPORTED)

        # Change mode to guided (Ardupilot) or takeoff (PX4)
        args = [tgt_sys_id, tgt_comp_id, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id, 0, 0, 0, 0, 0]
        self.the_connection.mav.command_long_send(*args)
        
        ack_msg = self.the_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
        print(f"Change Mode ACK:  {ack_msg}")

        # Arm the UAS
        args = [tgt_sys_id, tgt_comp_id, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0]
        self.the_connection.mav.command_long_send(*args)

        arm_msg = self.the_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
        print(f"Arm ACK:  {arm_msg}")

        # Command Takeoff
        args = [tgt_sys_id, tgt_comp_id, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0] + takeoff_params
        self.the_connection.mav.command_long_send(*args)

        takeoff_msg = self.the_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
        print(f"Takeoff ACK:  {takeoff_msg}")
