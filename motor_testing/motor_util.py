# motor_util.py

import time

class motor():
    
    def __init__(self, step, dir):
        self.dir_pin = dir
        self.step_pin = step
        self.current_dir = 0
        self.current_step = 0
        
    def set_dir(self, dir):
        self.current_dir = dir
        self.dir_pin.value(dir)
    
    def step_once(self):
        self.step_pin.value(1)
        self.step_pin.value(0)
        if (self.current_dir == 1):
            self.current_step += 1
        elif (self.current_dir == 0):
            self.current_step -= 1
    
class transducer():
    
    def __init__(self, pin):
        self.pin = pin
        
    def read(self):
        return ((self.pin.read_u16() * 3.3 / 65536.0) -0.4389 + 0.13 ) / 0.0175 # in PSI