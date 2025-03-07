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

from message_dispatcher import MessageDispatcher
from fly_controller import FlyController
from environment import FlyState, FlyTask, FlyControlException, FlyControlExceptionCode


class AutoPilot():
    def __init__(self, connection_string: str):
        # Start a connection listening to a UDP port
        self.the_connection = mavutil.mavlink_connection(connection_string)
        self.fly_controller = FlyController(self.the_connection, delegate=self)

        self.__is_listening = False
        self.__latest_received_message = None
        self.message_subscriber_thread = None

        self.message_dispatcher = MessageDispatcher(controller=self.fly_controller)
        self.connection_listener_thread = threading.Thread(target=self.connection_listener, name="CLT", daemon=True)
        self.connection_listener_thread.start()
    
        self.__flight_palan_is_acomplished = False

    def flight_palan_is_acomplished(self):
        self.__flight_palan_is_acomplished = True

    @property
    def is_listening(self):
        return self.__is_listening
    
    def start_subscibers(self):
        self.message_subscriber_thread = threading.Thread(target=self.message_subscriber_listener, name="SubscriberListenerThread", daemon=True)
        self.message_subscriber_thread.start()       

    def fly(self):
        while self.is_listening == False:
            print("Waiting for connection...")
            sleep(1)        
        
        self.fly_controller.fly()
        
        while self.__flight_palan_is_acomplished == False:
            self.the_connection.close()
            sleep(1)        

    @property
    def is_ready(self) -> bool:
         return self.is_listening
                
    def connection_established(self):
        self.the_connection.set_mode(COPTER_MODE_GUIDED_NOGPS)
        self.__is_listening = True

    def connection_lost(self):
         self.__is_listening = False

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
        self.__latest_received_message = self.the_connection.wait_heartbeat()
        print(f"Heartbeat from system (system {self.the_connection.target_system} component {self.the_connection.target_component}) for connection listener")
        self.connection_established()
        self.start_subscibers()

        timestamp = self.__latest_received_message._timestamp
        while True:
            try:
                if self.__latest_received_message is None:
                    continue
                dlt = self.__latest_received_message._timestamp - timestamp
                if dlt > timeout:
                    self.connection_lost()
                
                #if DEBUG:    
                #print(f"Heartbeat dlt: {dlt:0.2f}sec.")
                
                timestamp = self.__latest_received_message._timestamp
                
                if self.is_listening == False:
                    self.connection_established()

            except Exception as error:
                print(f"Error: {error}")
            
            sleep(1)

                
    def message_subscriber_listener(self):
        print("Run message_subscriber_listener")
        subscriber_list_types = ["GLOBAL_POSITION_INT", "OPTICAL_FLOW"]
        
        while True:
            message = self.the_connection.recv_match(blocking=False, type=subscriber_list_types, timeout=None)
            if message is None:
                continue
            kind = message.get_type()
            if kind in subscriber_list_types:
                # It is a way how dispatch diffrent messages by different processing queues
                # and different controllers
                # if kind == "HEARTBEAT" and message.type == MAV_TYPE_QUADROTOR:
                    # self.__latest_received_message = message
                    # continue
                self.__latest_received_message = message
                self.message_dispatcher.did_receive_message(message)



if __name__ == "__main__":
    pilot = AutoPilot(connection_string='udpin:localhost:14551')
    pilot.fly()