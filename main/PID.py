"""
Simple PID controller implementation for process control, including options for proportional, integral, and derivative gains.
Allows for output normalization, windup guarding, and sample time adjustment to control the rate of computation.
"""

import time

class PID:
    def __init__(self, P=1.0, I=0.0, D=0.0, current_time=None, max_output=1.0):
        self.Kp = P
        self.Ki = I
        self.Kd = D

        self.sample_time = 0.00
        self.current_time = current_time if current_time is not None else time.time()
        self.last_time = self.current_time

        self.max_output = max_output  # Maximum value for normalization
        self.clear()

    def clear(self):
        """Clears PID computations and coefficients"""
        self.SetPoint = 0.0

        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0

        # Windup Guard
        self.int_error = 0.0
        self.windup_guard = 1.0

        self.output = 0.0

    def update(self, feedback_value, current_time=None):
        """Calculates PID value for given reference feedback and normalizes the output"""
        error = self.SetPoint - feedback_value

        self.current_time = current_time if current_time is not None else time.time()
        delta_time = self.current_time - self.last_time
        delta_error = error - self.last_error

        if (delta_time >= self.sample_time):
            self.PTerm = self.Kp * error
            self.ITerm += error * delta_time

            if (self.ITerm < -self.windup_guard):
                self.ITerm = -self.windup_guard
            elif (self.ITerm > self.windup_guard):
                self.ITerm = self.windup_guard

            self.DTerm = 0.0
            if delta_time > 0:
                self.DTerm = delta_error / delta_time

            # Remember last time and last error for next calculation
            self.last_time = self.current_time
            self.last_error = error

            # Compute raw output
            raw_output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)

            # Debugging
            #print(f"PTerm: {self.PTerm}, ITerm: {self.ITerm}, DTerm: {self.DTerm}, raw_output: {raw_output}")

            # Normalize output
            if self.max_output != 0:
                self.output = max(-self.max_output, min(self.max_output, raw_output))
                #print(f"[PID] Output: {self.output}")
            else:
                self.output = raw_output

    def setKp(self, proportional_gain):
        """Sets Proportional Gain"""
        self.Kp = proportional_gain

    def setKi(self, integral_gain):
        """Sets Integral Gain"""
        self.Ki = integral_gain

    def setKd(self, derivative_gain):
        """Sets Derivative Gain"""
        self.Kd = derivative_gain

    def setWindup(self, windup):
        """Sets windup guard"""
        self.windup_guard = windup

    def setSampleTime(self, sample_time):
        """Sets sample time"""
        self.sample_time = sample_time

    def setMaxOutput(self, max_output):
        """Sets the maximum absolute value for normalized output"""
        self.max_output = max_output
