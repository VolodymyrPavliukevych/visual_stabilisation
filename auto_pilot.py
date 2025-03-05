# -*- coding:utf8 -*-
# !/usr/bin/env python3
# Copyright 2025 Octadero.
# Author Volodymyr Pavliukevych

"""This is a MAVLink interactor
"""

from pymavlink import mavutil

from pymavlink.dialects.v20.ardupilotmega import MAVLink_optical_flow_message
from pymavlink.dialects.v20.ardupilotmega import MAVLink_set_position_target_local_ned_message
# from pymavlink.dialects.v20.ardupilotmega import set_position_target_local_ned_send
from pymavlink.dialects.v20.ardupilotmega import MAVLink

from pymavlink.dialects.v20.ardupilotmega import COPTER_MODE_GUIDED_NOGPS, MAV_TYPE_QUADROTOR

from utilities.connect_to_sysid import connect_to_sysid
from utilities.wait_for_position_aiding import wait_until_position_aiding
from utilities.get_autopilot_info import get_autopilot_info


import threading
from enum import IntEnum, StrEnum
from time import sleep, time
import math
from optical_flow_interactor import OpticalFlowInteractor

from message_dispatcher import MessageDispatcher

class FlyTask(IntEnum):
    IDLE = 0
    TEST_FLY_CONTROLS = 1
    DONE = 2
    
class FlyState(StrEnum):
    UNKNOWN = "UNKNOWN"
    ON_THE_GROUND = "ON_THE_GROUND"
    UNCONTROLLED_IN_THE_AIR = "UNCONTROLLED_IN_THE_AIR"
    TAKING_OFF = "TAKING_OFF"
    ALTITUDE_CLIMBING = "ALTITUDE_CLIMBING"
    LAND = "LAND"
    IN_THE_AIR = "IN_THE_AIR"
    MOVE = "MOVE"
    

class FlyControlExceptionCode(IntEnum):
    NOT_SUITABLE_STATE = 0
    NOT_SUITABLE_TASK = 1
    AUTO_PILOT_NOT_SUPPORTED = 2

class FlyControlException(Exception):
    def __init__(self, code, message=None, traceback=None):
        self.code = code
        self.traceback = traceback		
        if message is not None:
            self.message = message

        if isinstance(code, FlyControlExceptionCode):
            if message is None and code.value == FlyControlExceptionCode.NOT_SUITABLE_STATE.value:
                self.message = "You can't change state while corrent state is ongoing"
            elif message is None and code.value == FlyControlExceptionCode.NOT_SUITABLE_TASK.value:
                self.message = "You can't change task while corrent task is ongoing"
            elif message is None and code.value == FlyControlExceptionCode.AUTO_PILOT_NOT_SUPPORTED.value:
                self.message = "Auto pilot not supported"

        if isinstance(code, FlyControlExceptionCode) == False:
            self.message = 'Intent error heppend code: ' + str(code)

class AutoPilot():
    def __init__(self, connection_string: str):
        # Start a connection listening to a UDP port
        self.the_connection = mavutil.mavlink_connection(connection_string)
        self.is_listening = False
        self.initialized = False
        self.optical_flow_interactor = OpticalFlowInteractor()
        self.__fly_task = FlyTask.IDLE
        self.__fly_state = FlyState.UNKNOWN
        self.message_dispatcher = MessageDispatcher(controller=self)
        self.connection_listener_thread = threading.Thread(target=self.connection_listener, name="CLT", daemon=True)
        self.connection_listener_thread.start()

        self.message_subscriber_thread = None

        self.altitute = None

    def start_subscibers(self):
        self.message_subscriber_thread = threading.Thread(target=self.message_subscriber_listener, name="SubscriberListenerThread", daemon=True)
        self.message_subscriber_thread.start()       

    def change_fly_task(self, to_task:FlyTask):
        assert isinstance(to_task, FlyTask) == True, "Wrong task type"
        match self.__fly_task:
            case FlyTask.IDLE:
                  self.__fly_task = to_task
            case FlyTask.DONE:
                  self.__fly_task = to_task
            case FlyTask.TEST_FLY_CONTROLS:
                  self.__fly_task = to_task
            case _:
                raise FlyControlException(code=FlyControlExceptionCode.NOT_SUITABLE_TASK)

    def change_fly_state(self, to_state:  FlyState):
        assert isinstance(to_state, FlyState) == True, "Wrong state type"
        print(f"{self.__fly_state} -> {to_state}")
        self.__fly_state = to_state

        # match to_state:
        #  case FlyState.ON_THE_GROUND:  
        #     self.__fly_state = to_state
        #  case FlyState.TAKING_OFF:
        #     if self.__fly_state != FlyState.ON_THE_GROUND:
        #          raise FlyControlException(code=FlyControlExceptionCode.NOT_SUITABLE_STATE)
        #     self.__fly_state = to_state
        #  case FlyState.ALTITUDE_CLIMBING:  
        #     self.__fly_state = to_state
        #  case FlyState.LAND:  
        #     self.__fly_state = to_state
        #  case  FlyState.UNCONTROLLED_IN_THE_AIR:
        #     self.__fly_state = to_state
        #  case _:
        #     raise FlyControlException(code=FlyControlExceptionCode.NOT_SUITABLE_STATE)

    @property
    def is_ready(self) -> bool:
         return self.is_listening and self.altitute is not None


    def fly(self):
        while True: #self.is_ready == False:
            print(f"Whaiting for connection...")
            sleep(1)
        
        try:
            # pass

            print("Taking off...")
            self.guided_no_gps_exec_taking_off()

            print("Holding altitude...")
            self.guided_no_gps_exec_altitude()
            # self.change_fly_task(to_task=FlyTask.TEST_FLY_CONTROLS)
            # self.follow_test_fly_controls_task()
        except Exception as error:
             print(error)
        
        while self.__fly_task == FlyTask.IDLE:
             # Waiting for task will be resolved
             pass


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

    def follow_test_fly_controls_task(self):
        print(f"Follow test fly control: {self.__fly_state}")
        try:
            if self.__fly_state == FlyState.ON_THE_GROUND:
                if self.__fly_task == FlyTask.TEST_FLY_CONTROLS:
                    self.change_fly_state(to_state=FlyState.TAKING_OFF)
                    self.takeoff(takeoff_altitude=5)
                elif self.__fly_task == FlyTask.DONE:
                    return

            elif self.__fly_state == FlyState.UNCONTROLLED_IN_THE_AIR:
                 self.change_fly_state(to_state=FlyState.LAND)
                 self.land()
            
            elif self.__fly_state == FlyState.IN_THE_AIR:
                
                self.change_fly_state(to_state=FlyState.MOVE)
                self.compute_velocity_command(0,0,0)
                #  self.change_fly_task(to_task=FlyTask.DONE)
                #  self.change_fly_state(to_state=FlyState.LAND)
                #  self.land()
                             
            elif self.__fly_state == FlyState.TAKING_OFF:
                 self.change_fly_state(to_state=FlyState.ALTITUDE_CLIMBING)
            
            elif self.__fly_state == FlyState.LAND:
                while self.altitute > 0.5:
                     print(f"Waiting for land: {self.altitute:0.2f}")
                     sleep(1)
                self.change_fly_state(to_state=FlyState.ON_THE_GROUND)

            elif self.__fly_state == FlyState.ALTITUDE_CLIMBING:
                while self.altitute < 4.5:
                    print(f"Waiting for climbing: {self.altitute:0.2f}")
                    sleep(1)
                self.change_fly_state(to_state=FlyState.IN_THE_AIR)
            
            sleep(1)
            self.follow_test_fly_controls_task()        



        except Exception as error:
             print(error)
                
    def connection_established(self):
        self.the_connection.set_mode(COPTER_MODE_GUIDED_NOGPS)
        self.is_listening = True
        self.start_subscibers()

    def connection_lost(self):
         self.is_listening = False

    def current_mode(self, timeout=3) -> tuple:
        start_time = time()
        while time() - start_time < timeout:
            message = self.the_connection.wait_heartbeat(timeout=timeout)
            if message is not None:
                if message.type != MAV_TYPE_QUADROTOR:
                    continue
                mode_id = message.custom_mode
                mode_mapping = self.the_connection.mode_mapping()
                current_mode = [mode for mode, id in mode_mapping.items() if id == mode_id]
                print(f"Mode: {current_mode[0]}")
                return (mode_id, current_mode)
        return None


    def connection_listener(self, timeout=3):
        self.the_connection.wait_heartbeat()
        print(f"Heartbeat from system (system {self.the_connection.target_system} component {self.the_connection.target_component}) for connection listener")
        self.connection_established()

        timestamp = 0
        start_time = time()
        while True:
            try:
                loop_time = time() - start_time
                if loop_time > timeout:
                    self.connection_lost()

                message = self.the_connection.wait_heartbeat(blocking=False, timeout=timeout)
                if message is None:
                    continue
                if message.type != MAV_TYPE_QUADROTOR:
                    continue
                
                dlt = message._timestamp - timestamp
                mode_name = mavutil.mode_string_acm(message.custom_mode)
                print(f"Heartbeat dlt: {dlt:0.2f} sec timeout:{loop_time:0.2f} sec. Mode: '{mode_name}'")
                
                timestamp = message._timestamp
                start_time = time() 
                
                if self.is_listening == False:
                    self.connection_established()

            except Exception as error:
                print(f"Error: {error}")
                
    def message_subscriber_listener(self):
        print("Run message_subscriber_listener")
        subscriber_list_types = ["GLOBAL_POSITION_INT", "OPTICAL_FLOW"]
        
        while self.is_listening:
            message = self.the_connection.recv_msg()
            if message is None:
                continue
            if message.get_type() in subscriber_list_types:
                self.message_dispatcher.did_receive_message(message)

            
        print("Stop message_subscriber_listener")


    # It should be subscriber base message buss
    def did_receive_global_position(self, message):
         altitute = message.relative_alt / 1000
         self.altitute = altitute
         if self.initialized == False:
              self.initialized = True
              if self.altitute < 0.3:
                   self.__fly_state = FlyState.ON_THE_GROUND
              else:
                   self.__fly_state = FlyState.UNCONTROLLED_IN_THE_AIR


    def did_receive_optical_flow(self, message:MAVLink_optical_flow_message):
        compensation = self.optical_flow_interactor.compute_correction_optical_flow(message)
        if compensation is not None:
            timestamp, velocity_x, velocity_y = compensation
            self.compute_velocity_command(timestamp, velocity_x, velocity_y)
            pass

    # Function to send attitude target
    def send_attitude_target(self, roll=0, pitch=0, yaw=0, thrust=0.5):
        # Arm the drone
        self.the_connection.arducopter_arm()
        self.the_connection.motors_armed_wait()
        print("Drone armed.")
        """
        Sends a MAVLink SET_ATTITUDE_TARGET message.
        roll, pitch, yaw are in radians.
        thrust is from 0.0 (no thrust) to 1.0 (full thrust).
        """
        q = [
            math.cos(yaw / 2),  # w
            0,                  # x (roll)
            0,                  # y (pitch)
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

    # Compute velocity command to the drone
    def compute_velocity_command(self, timestamp: int, velocity_x: float, velocity_y: float, velocity_z: float = 0):
        pass

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


if __name__ == "__main__":
    pilot = AutoPilot(connection_string='udpin:localhost:14551')
    pilot.fly()