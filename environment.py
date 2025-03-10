# -*- coding:utf8 -*-
# !/usr/bin/env python3
# Copyright 2025 Octadero.
# Author Volodymyr Pavliukevych

"""This is a Fly environment
"""
from enum import IntEnum, StrEnum

class FlyCommandKind(IntEnum):
    TAKE_OFF = 0
    CLIMBING = 1
    ALTITUDE_HOLD = 2
    LANDING = 3
    MOTORS_OFF = 4
    MOVE = 5
    
    # TODO: FIX IT
    ALTITUDE_HOLD_CORRECTION = 6

class FlyTask(IntEnum):
    IDLE = 0
    TEST_FLY_CONTROLS = 1
    DONE = 2
    ABORT = 3
    
class FlyState(StrEnum):
    UNKNOWN = "UNKNOWN"
    UNCONTROLLED_IN_THE_AIR = "UNCONTROLLED_IN_THE_AIR"
    ON_THE_GROUND = "ON_THE_GROUND"
    TAKING_OFF = "TAKING_OFF"
    ALTITUDE_CLIMBING = "ALTITUDE_CLIMBING"
    IN_THE_AIR = "IN_THE_AIR"
    ALTITUDE_HOLD = "ALTITUDE_HOLD"
    LANDING = "LANDING"
    MOVE = "MOVE"
    DONE = "DONE"
    

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


class FlightPlanExceptionCode(IntEnum):
    TARGET_ALTITUDE_IN_NONE = 0
    CURRENT_ALTITUDE_IN_NONE = 1

class FlightPlanException(Exception):
    def __init__(self, code, message=None, traceback=None):
        self.code = code
        self.traceback = traceback		
        if message is not None:
            self.message = message

        if isinstance(code, FlightPlanExceptionCode):
            if message is None and code.value == FlightPlanExceptionCode.TARGET_ALTITUDE_IN_NONE.value:
                self.message = "Target altitude is not set"
            elif message is None and code.value == FlightPlanExceptionCode.CURRENT_ALTITUDE_IN_NONE.value:
                self.message = "Current altitude is not set"

        if isinstance(code, FlyControlExceptionCode) == False:
            self.message = 'Intent error heppend code: ' + str(code)



class RCSExceptionCode(IntEnum):
    CAN_NOT_ARM_UAV = 0
    CAN_NOT_DISARM_UAV = 1

class RCSException(Exception):
    def __init__(self, code, message=None, traceback=None):
        self.code = code
        self.traceback = traceback		
        if message is not None:
            self.message = message

        if isinstance(code, FlightPlanExceptionCode):
            if message is None and code.value == RCSExceptionCode.CAN_NOT_ARM_UAV.value:
                self.message = "Can not arm UAV"
            elif message is None and code.value == RCSExceptionCode.CAN_NOT_DISARM_UAV.value:
                self.message = "Can not disarm UAV"

        if isinstance(code, FlyControlExceptionCode) == False:
            self.message = 'Intent error heppend code: ' + str(code)            