# motor_util.py

import time

class motor_driver():
    
    def __init__(self, dir_pin, step_pin):
        self.dir_pin = dir_pin
        self.step_pin = step_pin
        self.current_step = 0
        self.wanted_step = 0
    
    def set_step(self, step):
        self.wanted_step = step
    
    def update(self):
        if (self.current_step != self.wanted_step):
            dir_value = (self.wanted_step - self.current_step) > 0 # TODO : check if right
            self.dir_pin.value(dir_value)
            
            self.step_pin.value(1)
            self.step_pin.value(0)
            self.current_step += 1
            time.sleep(1/400)