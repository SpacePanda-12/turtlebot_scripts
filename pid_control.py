# class for implementing PID control on the turtlebot
# evaluates a PID controller given current state

import math
import time


class pid_controller(object):

    def __init__(self, p_gain, d_gain, i_gain, i_min, i_max):
        self._set_gains(p_gain, d_gain, i_gain, i_min, i_max)
        self._reset()
        self.current_time = time.time()

    def _reset(self):
        self._p_error_last = 0
        self._p_error = 0
        self._d_error = 0
        self._i_error = 0
        self._cmd = 0
        self._last_time = None
    
    
    def _set_gains(self, p_gain, d_gain, i_gain, i_min, i_max):
        # setter for proportional, derivative, and integral components
        # of a the controller with a built-in check for integral gain bounds
        self._p_gain = p_gain
        self._d_gain = d_gain

        if i_gain > i_max:
            i_gain = i_max
        elif i_gain < i_min:
            i_gain = i_min

        self._i_gain = i_gain
        self._i_min = i_min
        self._i_max = i_max

    
    def _get_gains(self):
        return (self._p_gain, self._d_gain, self._i_gain)
        
    def _set_errors(self, current_state, goal, dt):
        self._p_error = goal - current_state

        if dt > 0:
            self._d_error = (self._p_error - self._p_error_last)/dt
        else:
            self._d_error = 0
        
        self._i_error += (goal - current_state)*dt
    
    
    def _get_errors(self):
        return (self._p_error, self._d_error, self._i_error)

    
    def _previous_time(self):
        return self._last_time

    def _evaluate_controller(self, p_gain, d_gain, i_gain, p_error, d_error, i_error):
        # output PID controller based on state and gains
        return p_gain * p_error + d_gain * d_error + i_gain * i_error

    #this is the only function that needs to be called from outside this class
    def get_command(self, current_state, goal):
        if self._last_time is None:
            self._last_time = time.time()
            self.current_time = self._last_time
        
        else:
            self.current_time = time.time()
        
        dt = self.current_time - self._last_time
        self._set_errors(current_state, goal, dt)
        p_error, d_error, i_error = self._get_errors()
        p_gain, d_gain, i_gain = self._get_gains()

        self.command = self._evaluate_controller(p_gain, d_gain, i_gain, p_error, d_error, i_error)


        self._last_time = self.current_time

        return self.command

        

        
        
    

    

    
    

