# -*- coding:utf8 -*-
# !/usr/bin/env python3
# Copyright 2025 Octadero.
# Author Volodymyr Pavliukevych

"""This is a Optical Flow Interactor
"""
import numpy as np
from pymavlink.dialects.v20.ardupilotmega import MAVLink_optical_flow_message


class ProportionalIntegralDerivative():
    def __init__(self, kp, ki, kd, max_output=1.0, min_output=-1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        self.min_output = min_output
        self.prev_error = 0
        self.integral = 0

    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        self.prev_error = error
        return np.clip(output, self.min_output, self.max_output)


class OpticalFlowInteractor():
    def __init__(self):
        # Initialize PID controllers for x and y stabilization
        self.pid_x = ProportionalIntegralDerivative(kp=0.1, ki=0.01, kd=0.05)
        self.pid_y = ProportionalIntegralDerivative(kp=0.1, ki=0.01, kd=0.05)
        self.latest_timestamp = 0
    
    # Processing optical flow based correction signal
    def compute_correction_optical_flow(self, message):
        _timestamp = message._timestamp

        # Extract optical flow data
        flow_x = message.flow_comp_m_x
        flow_y = message.flow_comp_m_x
        flow_quality = message.quality  # 0-255 quality of the flow
        # print(f"flow_x: {flow_x}, flow_y: {flow_y}, flow_quality: {flow_quality}")
        
        if flow_quality < 50:  # Ignore low-quality measurements
            return

        dt = _timestamp - self.latest_timestamp
        self.latest_timestamp = _timestamp

        # Calculate velocity based on optical flow and distance (ground_distance in meters)
        ground_distance = message.ground_distance
        if ground_distance > 0 and dt < 5.0:
            velocity_x = flow_x * ground_distance
            velocity_y = flow_y * ground_distance

            # Stabilize the drone using PID
            correction_x = self.pid_x.compute(-velocity_x, dt)
            correction_y = self.pid_y.compute(-velocity_y, dt)

            #print(f"dt: {dt:0.2f}s ground_distance: {ground_distance:0.2f}m, correction_x: {correction_x:0.4f}m, correction_y: {correction_y:0.4f}m")
            return (dt, correction_x, correction_y)
        return None
