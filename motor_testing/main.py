# motor_test.py

# start at 0, get the relationship between
# the current step and the pressure output
# from the pressure transducer

# imports
from motor_util import*
from machine import Pin, ADC
import time

# user defines
MOTOR_TO_TEST = "TOP" # use TOP, BOTTOM, or BOTH
REV_LIMIT = 6 # how many turns
STEPS_PER_REV = 200 # how many steps per turn
INCREMENT_STEP = 3 # test every every 5.4 degrees

# pressure transducers
transducer_top = transducer(ADC(27))
transducer_bot = transducer(ADC(26))

# motors
motor_top = motor(step = Pin(2, mode=Pin.OUT), dir = Pin(3, mode=Pin.OUT))
motor_bot = motor(step = Pin(4, mode=Pin.OUT), dir = Pin(5, mode=Pin.OUT))

# log file
log = open("log.csv", "w")

if (MOTOR_TO_TEST == "TOP"):
    log.write("current_step_top, pressure_read_top\n")
    
    motor_top.set_dir(1) # set direction to opening
    for i in range(0, RANGE_LIMIT * STEPS_PER_REV):
        
        # step the motor once
        motor_top.step_once()
        time.sleep(0.1)
        
        # read the pressure
        pressure_top = transducer_top.read()
        
        # every increment step record data
        if (motor_top.current_step % INCREMENT_STEP == 0):
            # record the data
            log.write(str(motor_top.current_step) + ", " + str(pressure_top) + "\n")
            time.sleep(1)
        
        log.flush() # immediently write to file
      
    motor_top.set_dir(0) # set direction to closing
    for i in range(0, RANGE_LIMIT * STEPS_PER_REV):
        
        # step the motor once
        motor_top.step_once()
        time.sleep(0.1)
        
        # read the pressure
        pressure_top = transducer_top.read()
        
        if (motor_top.current_step % INCREMENT_STEP == 0):
            # record the data
            log.write(str(motor_top.current_step) + ", " + str(pressure_top) + "\n")
            time.sleep(1)
            
        log.flush() # immediently write to file
    
elif (MOTOR_TO_TEST == "BOTTOM"):
    log.write("current_step_bottom, pressure_read_bottom\n")
    
    motor_bot.set_dir(1) # set direction to opening
    for i in range(0, RANGE_LIMIT * STEPS_PER_REV):
        
        # step the motor once
        motor_bot.step_once()
        time.sleep(0.1)
        
        # read the pressure
        pressure_bot = transducer_bot.read()
        
        # every increment step record data
        if (motor_bot.current_step % INCREMENT_STEP == 0):
            # record the data
            log.write(str(motor_bot.current_step) + ", " + str(pressure_bot) + "\n")
            time.sleep(1)
            
        log.flush() # immediently write to file
        
    motor_bot.set_dir(0) # set direction to closing
    for i in range(0, RANGE_LIMIT * STEPS_PER_REV):
        
        # step the motor once
        motor_bot.step_once()
        time.sleep(0.1)
        
        # read the pressure
        pressure_bot = transducer_bot.read()
        
        if (motor_bot.current_step % INCREMENT_STEP == 0):
            # record the data
            log.write(str(motor_bot.current_step) + ", " + str(pressure_bot) + "\n")
            time.sleep(1)
        
        log.flush() # immediently write to file

log.close()


