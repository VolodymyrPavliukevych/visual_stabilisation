# -*- coding:utf8 -*-
# !/usr/bin/env python3
# Copyright 2025 Octadero.
# Author Volodymyr Pavliukevych

"""This is a Fly controller
"""

from pymavlink import mavutil
from pymavlink.mavutil import mavudp, mavtcp
from pymavlink.dialects.v20.ardupilotmega import MAVLink_optical_flow_message

from optical_flow_interactor import OpticalFlowInteractor
from environment import FlyState, FlyTask, FlyControlException, FlyControlExceptionCode
from reaction_control_systems import ReactionControlSystems
from time import sleep, time


class FlightPlan():
    def __init__(self):
        self.__altitude_approximation = 0.5
        self.target_altitude = None

    def is_target_altitude_been_achieved(self, at_altitude: float):
        if self.target_altitude is None:
            return False
        
        altitude_dlt = at_altitude - self.target_altitude
        return abs(altitude_dlt) < self.__altitude_approximation
    
    def __str__(self):
        return f"Flight plan: {self.target_altitude}"


class FlyController():
    FLIGHT_PLAN_LOOP_DALEY_SEC = 0.5

    def __init__(self, the_connection: mavudp | mavtcp):
        self.__fly_task = FlyTask.IDLE
        self.__fly_state = FlyState.UNKNOWN
        self.__initialized = False
        self.__flight_plan = FlightPlan()

        self.the_connection = the_connection
        self.optical_flow_interactor = OpticalFlowInteractor()
        self.rcs = ReactionControlSystems(the_connection=the_connection)
    
    @property
    def initialized(self):
        return self.__initialized
    
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
        match to_state:
         case FlyState.ON_THE_GROUND:  
            self.__fly_state = to_state
         case FlyState.TAKING_OFF:
            if self.__fly_state != FlyState.ON_THE_GROUND:
                 raise FlyControlException(code=FlyControlExceptionCode.NOT_SUITABLE_STATE)
            self.__fly_state = to_state
         case FlyState.ALTITUDE_CLIMBING:  
            self.__fly_state = to_state
         case FlyState.LAND:  
            self.__fly_state = to_state
         case  FlyState.UNCONTROLLED_IN_THE_AIR:
            self.__fly_state = to_state
         case _:
            raise FlyControlException(code=FlyControlExceptionCode.NOT_SUITABLE_STATE)

    def fly(self):
        try:
            self.change_fly_task(to_task=FlyTask.TEST_FLY_CONTROLS)
            self.follow_flight_plan_loop()
        except Exception as error:
             print(error)
        
        while self.__fly_task == FlyTask.IDLE:
             # Waiting for task will be resolved
             pass


    def follow_flight_plan_loop(self):
        print(f"Follow test fly control: {self.__fly_state}")
        try:
            if self.__fly_state == FlyState.ON_THE_GROUND:
                if self.__fly_task == FlyTask.TEST_FLY_CONTROLS:
                    self.change_fly_state(to_state=FlyState.TAKING_OFF)
                    self.__flight_plan.target_altitude = 5.0
                    
                    self.takeoff(takeoff_altitude=self.__flight_plan.target_altitude)
                elif self.__fly_task == FlyTask.DONE:
                    return

            elif self.__fly_state == FlyState.UNCONTROLLED_IN_THE_AIR:
                 self.change_fly_state(to_state=FlyState.LAND)
                 self.land()
            
            elif self.__fly_state == FlyState.IN_THE_AIR:
                 start_time = time()
                 while time() - start_time < 5:
                    self.althold(altitude=5.0)


                 self.change_fly_task(to_task=FlyTask.DONE)
                 self.change_fly_state(to_state=FlyState.LAND)
                 self.land()
                             
            elif self.__fly_state == FlyState.TAKING_OFF:
                 self.change_fly_state(to_state=FlyState.ALTITUDE_CLIMBING)
            
            elif self.__fly_state == FlyState.LAND:
                while self.__flight_plan.is_target_altitude_been_achieved(at_altitude=self.altitude) == False:
                     print(f"Waiting for land: {self.altitude:0.2f}")
                     sleep(1)
                self.change_fly_state(to_state=FlyState.ON_THE_GROUND)

            elif self.__fly_state == FlyState.ALTITUDE_CLIMBING:
                while self.altitude < 4.5:
                    print(f"Waiting for climbing: {self.altitude:0.2f}")
                    sleep(1)
                    
                self.change_fly_state(to_state=FlyState.IN_THE_AIR)
            
            sleep(FlyController.FLIGHT_PLAN_LOOP_DALEY_SEC)
            self.follow_flight_plan_loop()        

        except Exception as error:
             self.change_fly_task(to_task=FlyTask.ABORT)
             self.change_fly_state(to_state=FlyState.LAND)
             print(error)


    def althold(self, altitude: float=5.0):
        altitude_dlt = self.altitude - altitude
        if altitude_dlt > 0.5:
            self.rcs.send_attitude_target(thrust=0.4)
        elif altitude_dlt < -0.5:
            self.rcs.send_attitude_target(thrust=0.6)
        else:
            self.rcs.send_attitude_target(thrust=0.5)

    def land(self):
        self.rcs.send_attitude_target(thrust=0.4)

    def takeoff(self, takeoff_altitude: float):
        self.rcs.send_attitude_target(thrust=0.6)

# --------------------------------------
# MARK: - Income sensors information processiong
# --------------------------------------
    # It should be subscriber base message buss
    def did_receive_global_position(self, message):
         altitude = message.relative_alt / 1000
         self.altitude = altitude
         if self.initialized == False:
              self.__initialized = True
              if self.altitude < 0.3:
                   self.__fly_state = FlyState.ON_THE_GROUND
              else:
                   self.__fly_state = FlyState.UNCONTROLLED_IN_THE_AIR


    def did_receive_optical_flow(self, message:MAVLink_optical_flow_message):
        compensation = self.optical_flow_interactor.compute_correction_optical_flow(message)
        if compensation is not None:
            timestamp, velocity_x, velocity_y = compensation
            self.compute_velocity_command(timestamp, velocity_x, velocity_y)
            
    # Compute velocity command to the drone
    def compute_velocity_command(self, timestamp: int, velocity_x: float, velocity_y: float, velocity_z: float = 0):
        pass