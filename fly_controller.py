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
from environment import FlightPlanExceptionCode, FlightPlanException
from reaction_control_systems import ReactionControlSystems
from enum import IntEnum
from time import sleep, time
import threading
import queue
import math

class FlightPlan():
    def __init__(self, ground_altitude: float = 0.9):
        self.__altitude_approximation = FlyController.ALTITUDE_APPROXIMATION
        self.ground_altitude = ground_altitude
        self.target_altitude = 0.0
        self.target_latitude = None
        self.target_longitude = None        

        self.corrent_altitude = None
        self.corrent_latitude = None
        self.corrent_longitude = None
    
    @property
    def is_target_altitude_been_achieved(self):
        if self.target_altitude is None or self.corrent_altitude is None:
            return False
        
        altitude_dlt = self.corrent_altitude - self.target_altitude
        return abs(altitude_dlt) < self.__altitude_approximation
    
    @property
    def dlt_altitude(self):
        if self.target_altitude is None:
            raise FlightPlanException(code=FlightPlanExceptionCode.TARGET_ALTITUDE_IN_NONE)
        if self.corrent_altitude is None:
            raise FlightPlanException(code=FlightPlanExceptionCode.CURRENT_ALTITUDE_IN_NONE)
        return self.target_altitude - self.corrent_altitude
    
    def __str__(self):
        return f"Flight plan: {self.target_altitude}"

class FlyCommandKind(IntEnum):
    TAKE_OFF = 0
    CLIMBING = 1
    ALTITUDE_HOLD = 2
    LANDING = 3
    MOTORS_OFF = 4
    
    # TODO: FIX IT
    VELOCITY_CORRECTION = 5


class FlyCommand():
    def __init__(self, kind: FlyCommandKind, value: float | tuple = None):
        self.kind = kind
        self.value = value

class FlyController():
    FLIGHT_PLAN_LOOP_DALEY_SEC = 0.5
    CLAIMBING_THRUST = 0.55
    TAKE_OFF_THRUST = 0.7
    LAND_THRUST = 0.3
    ALTITUDE_HOLD_THRUST = 0.1
    ALTITUDE_APPROXIMATION = 0.1
    MAIN_LOOP_STEP_SEC = 0.1 # 10 Hz
    MAX_VELOCITY_CORRECTION_THRUST = 0.9

    def __init__(self, the_connection: mavudp | mavtcp, delegate=None):
        self.__delegate = delegate
        self.__fly_task = FlyTask.IDLE
        self.__fly_state = FlyState.UNKNOWN
        self.__initialized = False
        self.__flight_plan = FlightPlan()
        self.__queue = queue.Queue(maxsize=0)
        self.__target_thrust = 0.0
        self.__target_roll = math.radians(0)
        self.__target_pitch = math.radians(0)
        self.__target_yaw = math.radians(0)

        self.the_connection = the_connection
        self.optical_flow_interactor = OpticalFlowInteractor()
        self.rcs = ReactionControlSystems(the_connection=the_connection)
        
        self.main_reaction_thread = threading.Thread(target=self.main_reaction_control_loop, name="MainReactionThread", daemon=True)
        self.main_reaction_thread.start()        
    
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
         case FlyState.IN_THE_AIR:
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
            self.__delegate.flight_palan_is_acomplished()
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

                elif self.__fly_task == FlyTask.DONE:
                    self.add_command(FlyCommand(kind=FlyCommandKind.MOTORS_OFF))
                    sleep(1)
                    return

            elif self.__fly_state == FlyState.UNCONTROLLED_IN_THE_AIR:
                 self.change_fly_state(to_state=FlyState.LAND)
            
            elif self.__fly_state == FlyState.IN_THE_AIR:
                start_time = time()
                while time() - start_time < 5:
                    self.altitude_hold(altitude=5.0)
                    print(f"In the air {time() - start_time:0.2f}s")
                    sleep(1)

                print("The Task is acoplished")

                self.change_fly_task(to_task=FlyTask.DONE)
                self.change_fly_state(to_state=FlyState.LAND)

                             
            elif self.__fly_state == FlyState.TAKING_OFF:
                self.takeoff()
                self.change_fly_state(to_state=FlyState.ALTITUDE_CLIMBING)
            
            elif self.__fly_state == FlyState.LAND:
                self.__flight_plan.target_altitude = 0.0
                while self.__flight_plan.is_target_altitude_been_achieved == False:
                     print(f"Waiting for land: {self.__flight_plan.corrent_altitude:0.2f} -> {self.__flight_plan.target_altitude:0.2f}")
                     self.land()
                     sleep(1)

                self.change_fly_state(to_state=FlyState.ON_THE_GROUND)

            elif self.__fly_state == FlyState.ALTITUDE_CLIMBING:
                self.climbing(altitude=5.0)
                while self.__flight_plan.is_target_altitude_been_achieved == False:
                    print(f"Waiting for climbing: {self.__flight_plan.corrent_altitude:0.2f} -> {self.__flight_plan.target_altitude:0.2f}")
                    self.climbing(altitude=5.0)
                    if self.__flight_plan.corrent_altitude >= self.__flight_plan.target_altitude:
                        break
                    sleep(1)

                self.change_fly_state(to_state=FlyState.IN_THE_AIR)
            
            sleep(FlyController.FLIGHT_PLAN_LOOP_DALEY_SEC)
            self.follow_flight_plan_loop()        

        except Exception as error:
             self.change_fly_task(to_task=FlyTask.ABORT)
             self.change_fly_state(to_state=FlyState.LAND)
             print(error)




    def land(self):
        self.add_command(FlyCommand(kind=FlyCommandKind.LANDING, value=0.0))

    def takeoff(self):        
        self.__flight_plan.target_altitude = FlyController.ALTITUDE_APPROXIMATION

        # for _ in range(2):  # Send multiple commands to ensure takeoff
        self.rcs.send_motors_on(force_thrust=True)
        print("Motors on")
        sleep(1.0)
        self.add_command(FlyCommand(kind=FlyCommandKind.TAKE_OFF, value=self.__flight_plan.target_altitude))
        sleep(1.0)

    def climbing(self, altitude: float):
        self.__flight_plan.target_altitude = altitude
        self.add_command(FlyCommand(kind=FlyCommandKind.CLIMBING, value=altitude))

    def altitude_hold(self, altitude: float):
        self.__flight_plan.target_altitude = altitude
        self.add_command(FlyCommand(kind=FlyCommandKind.ALTITUDE_HOLD, value=altitude))

    def send_attitude_target(self, roll=0, pitch=0, yaw=0, thrust=0.0):
        self.rcs.send_attitude_target(roll=roll, pitch=pitch, yaw=yaw, thrust=thrust)

    def add_command(self, command: FlyCommand, block=False):
        self.__queue.put(command, block=block)

    @property
    def fly_state_is_suitable_for_correction(self):
        return self.__fly_state != FlyState.UNKNOWN and self.__fly_state != FlyState.ON_THE_GROUND and self.__fly_state != FlyState.TAKING_OFF
    
    @property
    def target_altitude(self):
        return self.__flight_plan.target_altitude
    
    @property
    def corrent_altitude(self):
        return self.__flight_plan.corrent_altitude

    @property
    def is_ziro_target_altitude(self):
        return self.target_altitude == 0.0
    
    @property
    def is_ziro_corrent_altitude(self):
        return self.corrent_altitude == 0.0

    def compute_thrust_adjustment(self, target_thrust: float, dlt_altitude: float, movement_thrust: float = 0.0):
        #TODO: compute thrust adjustment in respact to the dlt_altitude
        return target_thrust - self.__target_thrust + movement_thrust

    # Main loop transform target altitude, latitude, longtitudes in to the drone reaction: throst, pitch, roll, yaw
    def main_reaction_control_loop(self):

        thrust_adjustment_for_maintain_movement = 0.3
        while True:
            
            thrust_adjustment_for_maintain_altitude = 0.0
            pitch_adjustment = 0.0
            roll_adjustment = 0.0
            yaw_adjustment = 0.0
            command = self.__queue.get()

            # 1) Compute thrust adjustment based on target and current altitude
            if (self.target_altitude != 0.0 or self.corrent_altitude != 0.0) and (self.target_altitude is not None and self.corrent_altitude is not None):
                print(f"dlt_altitude: {self.__flight_plan.dlt_altitude:0.2f}")
                if self.__flight_plan.dlt_altitude > FlyController.ALTITUDE_APPROXIMATION:
                    thrust_adjustment_for_maintain_altitude += self.compute_thrust_adjustment(target_thrust=FlyController.CLAIMBING_THRUST, movement_thrust=0.0)

                elif self.__flight_plan.dlt_altitude < -FlyController.ALTITUDE_APPROXIMATION:
                    thrust_adjustment_for_maintain_altitude += self.compute_thrust_adjustment(target_thrust=FlyController.LAND_THRUST, movement_thrust=0.0)
                    
                else:
                    thrust_adjustment_for_maintain_altitude = 0.0 #FlyController.ALTITUDE_HOLD_THRUST
                
            elif command is None:
                continue
                
            # 2) Compute pitch, roll, yaw adjustments
            if command is not None:
                if command.kind == FlyCommandKind.TAKE_OFF:
                    thrust_adjustment_for_maintain_altitude = FlyController.TAKE_OFF_THRUST

                if command.kind == FlyCommandKind.CLIMBING:
                    pass
                    
                if command.kind == FlyCommandKind.ALTITUDE_HOLD:
                    pass
                
                if command.kind == FlyCommandKind.LANDING:
                    pass

                if command.kind == FlyCommandKind.MOTORS_OFF:
                    if self.corrent_altitude < FlyController.ALTITUDE_APPROXIMATION:
                        self.__target_thrust = 0.0
                        self.__target_pitch = math.radians(0)
                        self.__target_roll = 0.0
                        self.__target_yaw = 0.0
                        self.rcs.send_motors_off()
                        print("Motors off")
                    else:
                        print("ERROR: You can't disarm motors while in the air")

                if command.kind == FlyCommandKind.VELOCITY_CORRECTION and self.fly_state_is_suitable_for_correction:
                    vx, vy, vz, yaw_rate = command.value
                    pitch_adjustment = math.radians(-45)
                    self.__target_pitch = pitch_adjustment
                    # thrust_adjustment += 0.5
                    #max(self.target_thrust * 1.25, FlyController.MAX_VELOCITY_CORRECTION_THRUST)


            if thrust_adjustment_for_maintain_altitude != 0.0:
                print(f"target thrust: {self.__target_thrust:0.2f} thrust adjustment: {thrust_adjustment_for_maintain_altitude:0.2f} target pitch: {self.__target_pitch:0.2f}")
                self.__target_thrust += thrust_adjustment_for_maintain_altitude
                self.__target_thrust = min(self.__target_thrust, 1.0)

            
            # 3) Send the command to the drone
            self.send_attitude_target(pitch=0, thrust=self.__target_thrust)
            
            # Throtle the message sending
            sleep(FlyController.MAIN_LOOP_STEP_SEC)

            self.__queue.task_done()


# --------------------------------------
# MARK: - Income sensors information processiong
# --------------------------------------
    # It should be subscriber base message buss
    def did_receive_global_position(self, message):
         altitude = message.relative_alt / 1000
         self.__flight_plan.corrent_altitude = altitude
         
         if self.initialized == False:
              self.__initialized = True
              if self.__flight_plan.corrent_altitude < self.__flight_plan.ground_altitude:
                   self.__fly_state = FlyState.ON_THE_GROUND
              else:
                   self.__fly_state = FlyState.UNCONTROLLED_IN_THE_AIR


    def did_receive_optical_flow(self, message:MAVLink_optical_flow_message):
        compensation = self.optical_flow_interactor.compute_correction_optical_flow(message)
        if compensation is not None:
            dt, velocity_x, velocity_y = compensation
            self.compute_compensation_command(dt, velocity_x, velocity_y)
            
    # Compute compensation command to the drone
    def compute_compensation_command(self, dt: float, velocity_x: float, velocity_y: float, velocity_z: float = 0):
        # Compute velocity command
        vx = velocity_x
        vy = velocity_y
        vz = velocity_z
        yaw_rate = 0
        self.add_command(FlyCommand(kind=FlyCommandKind.VELOCITY_CORRECTION, value=(vx, vy, vz, yaw_rate)))
