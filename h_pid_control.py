"""
Author: Flaviano Nzambi
Program Description: 
    This Python program demonstrates a simple PID Controller feedback for the MiniTHOR system 
Date: January 19, 2024
"""

class PID_Controller:
    
    def __init__(self, kp=1 , ki=0, kd=0):
        self.kp = kp    # proportional_gain
        self.ki = ki    # integral_gain
        self.kd = kd    # derivative_gain 

        self.time_prev = 0
        self.error_prev = 0

        self.P = 0
        self.I = 0
        self.D = 0

    def calculate_roll(self,target_x=0 ,current_x=0,time=0,kp=1 , ki=0, kd=0,new=False):
        if new is True:
            self.kp = kp    # proportional_gain
            self.ki = ki    # integral_gain
            self.kd = kd    # derivative_gain 

        error = target_x - current_x

        self.P = error * self.kp
        self.I = self.I + self.ki*error*(time-self.time_prev)
        self.D = self.kd*(error-self.error_prev)/(time-self.time_prev)
        a = (time-self.time_prev)
        print("P:",int(self.P),"  I:",int(self.I),"  D:",int(self.D),"Time:",a)
        roll = self.P + self.I + self.D

        self.time_prev = time
        self.error_prev = error

        return roll

