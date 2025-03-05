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

from utilities.connect_to_sysid import connect_to_sysid
from utilities.wait_for_position_aiding import wait_until_position_aiding
from utilities.get_autopilot_info import get_autopilot_info


import threading
from enum import IntEnum, StrEnum
from time import sleep

from optical_flow_interactor import OpticalFlowInteractor

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
        

        self.connection_listener_thread = threading.Thread(target=self.connection_listener, name="CLT", daemon=True)
        self.connection_listener_thread.start()

        self.global_position_subscriber_thread = None
        self.optical_flow_subscriber_thread = None

        self.altitute = None

    def start_subscibers(self):
        self.global_position_subscriber_thread = threading.Thread(target=self.global_position_listener, name="GPSubscriberThread", daemon=True)
        self.global_position_subscriber_thread.start()

        # self.optical_flow_subscriber_thread = threading.Thread(target=self.optical_flow_listener, name="OFSubscriberThread", daemon=True)
        # self.optical_flow_subscriber_thread.start()        

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
        while self.is_ready == False:
            print(f"Whaiting for connection...")
            sleep(1)
        
        try:
            # pass
            self.change_fly_task(to_task=FlyTask.TEST_FLY_CONTROLS)
            self.follow_test_fly_controls_task()
        except Exception as error:
             print(error)
        
        while self.__fly_task == FlyTask.IDLE:
             # Waiting for task will be resolved
             pass

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
        self.is_listening = True
        # Set to GUIDED mode
        #self.the_connection.set_mode_apm("GUIDED_NOGPS")

        # Arm the drone
        # self.the_connection.arducopter_arm()
        self.start_subscibers()
        # self.compute_velocity_command(0,0,0)


    def connection_lost(self):
         self.is_listening = False

    def connection_listener(self):
        self.the_connection.wait_heartbeat()
        print(f"Heartbeat from system (system {self.the_connection.target_system} component {self.the_connection.target_component}) for connection listener")
        self.connection_established()

        timestamp = 0
        while True:
            try:
                message = self.the_connection.wait_heartbeat(timeout=5, blocking=True)
                if message:
                    dlt = message._timestamp - timestamp - 1
                    print(f"Heartbeat dlt: {dlt:0.2f} sec")
                    timestamp = message._timestamp
                    if self.is_listening == False:
                        self.connection_established()
                        continue
                else:
                    self.connection_lost()
            except Exception as error:
                print(f"Error: {error}")
                
            sleep(1)

    def global_position_listener(self):
        self.the_connection.wait_heartbeat()
        print(f"Heartbeat from system (system {self.the_connection.target_system} component {self.the_connection.target_component}) for global position subscriber")

        while self.is_listening:
            message = self.the_connection.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
            if message is None:
                continue
            self.did_receive_global_position(message)
        print("Stop global_position_listener")

    def optical_flow_listener(self):
        self.the_connection.wait_heartbeat()
        print(f"Heartbeat from system (system {self.the_connection.target_system} component {self.the_connection.target_component}) for optical flow subscriber")

        while self.is_listening:
            message = self.the_connection.recv_match(type='OPTICAL_FLOW', blocking=False)
            if message is None:
                continue            
            self.did_receive_optical_flow(message)

            message = self.the_connection.recv_match(type='POSITION_TARGET_LOCAL_NED', blocking=False)
            if message:
                print(f"Received: {message}")

            message = self.the_connection.recv_match(type='LOCAL_POSITION_NED', blocking=False)
            if message:
                print(f"Received: {message}")

        print("Stop optical_flow_listener")


    # Listen to optical flow messages
    def mavlink_callback(self, name, msg):
        if name == 'OPTICAL_FLOW':
            self.did_receive_optical_flow(msg)


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
            #self.compute_velocity_command(timestamp, velocity_x, velocity_y)
            pass

    # Compute velocity command to the drone
    def compute_velocity_command(self, timestamp, velocity_x, velocity_y, velocity_z=0):
        self.the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
            10, 
            self.the_connection.target_system, 
            self.the_connection.target_component, 
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, 
            int(0b010111111000), 
            20, 0, -10, # x, y, z positions
            2, 0, 1, # x, y, z velocity
            0, 0, 0, # x, y, z acceleration
            0.0, 0.0)) #yaw, yaw_rate

        # message = MAVLink_set_position_target_local_ned_message(
        #     0,  # time_boot_ms (not used)
        #     1, 1,  # target_system, target_component (usually 1 for both)
        #     mavutil.mavlink.MAV_FRAME_BODY_NED,  # Coordinate frame
        #     0b0000111111000111,  # Type mask (only velocity control)
        #     0, 0, 0,  # x, y, z positions (not used)
        #     velocity_x, velocity_y, velocity_z,  # x, y, z velocity (move forward at 1 m/s)
        #     0, 0, 0,  # x, y, z acceleration (not used)
        #     0, 0  # yaw, yaw_rate (not used)
        # )
        # time_boot_ms = 0
        # coordinate_frame = mavutil.mavlink.MAV_FRAME_BODY_NED
        # force_mavlink1 = True
        # target_system = 1
        # target_component = 1
        # type_mask = 0b0000111111000111
        # self.the_connection.mav.send(
        #     self.the_connection.mav.set_position_target_local_ned_encode(
        #         time_boot_ms, 
        #         target_system, 
        #         target_component, 
        #         coordinate_frame, 
        #         type_mask, 
        #         1, 
        #         1, 
        #         1, 
        #         1, 
        #         1, 
        #         1, 
        #         1, 
        #         1, 
        #         1, 
        #         1, 
        #         1), 
        #     force_mavlink1=force_mavlink1)


        # type_mask = 0b0000111111000111  # Ignore acceleration & force

        # import time
        # x, y, z = 1, 1, 0
        # vx, vy, vz = 1, 1, 0
        # yaw, yaw_rate = 0, 0

        # self.the_connection.mav.set_position_target_local_ned_send(
        #     0,  #int(time.time())  Timestamp in milliseconds
        #     target_system,    # Target system ID
        #     target_component, # Target component ID
        #     mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # Use NED frame
        #     type_mask,               # Ignore some fields
        #     x, y, z,                 # Position (meters)
        #     vx, vy, vz,               # Velocity (m/s)
        #     0, 0, 0,                 # Acceleration (not used)
        #     yaw, yaw_rate             # Yaw and yaw rate
        # )        
        
    def update_attitude(self, message):
         pass

    def land(self, tgt_sys_id: int = 1, tgt_comp_id=1):

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

    def takeoff(self, takeoff_altitude: float, tgt_sys_id: int = 1, tgt_comp_id=1):
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
        #print(f"Change Mode ACK:  {ack_msg}")

        # Arm the UAS
        args = [tgt_sys_id, tgt_comp_id, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0]
        self.the_connection.mav.command_long_send(*args)

        arm_msg = self.the_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
        #print(f"Arm ACK:  {arm_msg}")

        # Command Takeoff
        args = [tgt_sys_id, tgt_comp_id, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0] + takeoff_params
        self.the_connection.mav.command_long_send(*args)

        takeoff_msg = self.the_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
        #print(f"Takeoff ACK:  {takeoff_msg}")


if __name__ == "__main__":
    pilot = AutoPilot(connection_string='udpin:localhost:14551')
    pilot.fly()