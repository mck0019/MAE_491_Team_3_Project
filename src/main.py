# main.py
# Last Updated: April 11th, 2023
# Author: Michael Key

# This program implements a PID controller for our MAE 491 
# Control Application Project : Active Single Axis Rocket 
# Attitude Controller.
#
# It relies on the following modules: 
#  - control_util.py
#  - sensor_util.py
#  - web_util.py
#

# imports 
from control_util import *
from sensor_util import *
from web_util import *

from machine import Pin, ADC
import _thread as thread
import utime as time

# set up controller
controller = pid_controller(1.1, 0.2, 15.5) # PID controller

# set up sensors
imu = imu(scl_pin=Pin(1, mode=Pin.OUT), sda_pin=Pin(0, mode=Pin.OUT), freq=400000) # set up the imu
transducer_top = transducer(ADC(26)) # set up the top pressure transducer
transducer_bot = transducer(ADC(27)) # set up the bottom pressure transducer

# open a wireless access point
network = start_network("MAE 491 Project Interface", "123456789")

# global variables - these are variables that are shared across threads
g_state = "None" # defines the current state of the system (ex: "Start", "Stop", etc.)
g_test = "None"
g_controller_pressure_top = 0 # the pressure output from the controller for the top nozzle. 
g_controller_pressure_bot = 0 # the pressure output from the controller for the bottom nozzle. 

# the stepper motor thread
def motors_thread(lock):
    
    # define the stepper motors
    motor_top = stepper_motor(step=Pin(2, mode=Pin.OUT), dir=Pin(3, mode=Pin.OUT))
    motor_bot = stepper_motor(step=Pin(5, mode=Pin.OUT), dir=Pin(4, mode=Pin.OUT))
    
    # global variables
    global g_state
    global g_test
    global g_controller_pressure_top
    global g_controller_pressure_bot
    
    
    count = 0
    while True:
        
        with lock:
            state = g_state
            test = g_test
            controller_pressure_top = g_controller_pressure_top
            controller_pressure_bot = g_controller_pressure_bot
            
        if (state == "End"):
            thread.exit()
        
        if (state == "Start"):
            
            # if full controller
            if (test not in {"1_1_2", "1_2_1", "1_2_2"}):
                motor_top.set_target_pressure(controller_pressure_top)
                motor_bot.set_target_pressure(controller_pressure_bot)
                
                motor_top.update()
                motor_bot.update()
                
                time.sleep_ms(7)
                
            # if leak test
            elif (test == "1_2_2"):
                motor_top.set_target_step(MOTOR_MAX_REV * 200)
                motor_bot.set_target_step(MOTOR_MAX_REV * 200)
                motor_top.update()
                motor_bot.update()
                time.sleep_ms(7)
            
            # safety test
            elif (test == "1_2_1"):
                motor_top.set_target_pressure(53)
                motor_top.update()
                time.sleep_ms(7)

            
        elif (state == "Stop"):
            motor_top.set_target_step(0)
            motor_bot.set_target_step(0)
            motor_top.update()
            motor_bot.update()
            time.sleep_ms(7)
        

# main loop
while True:
    
    # initial startup
    serve_webpage() # wait to serve webpage
    socket = web_socket(("0.0.0.0", 80)) # create socket
    socket.upgrade() # upgrade to web socket
    
    # local variables
    state = "None"
    test = "none"
    start_time = 0
    controller_pressure_top = 0
    controller_pressure_bot = 0
    record_pressure = False

    # start the motor_thread
    lock = thread.allocate_lock() # thread lock
    thread.start_new_thread(motors_thread, (lock,))
    
    # controller loop
    while True:
        
        # get data
        data = socket.receive()
        if data==None: break
        
        # parse message
        message = str(data.decode())
        arguments = {}
        split_str = message.split("; ")
        for item in split_str:
            key, value = item.split(": ")
            arguments[key.strip()] = value.strip()
        
        # these are the test that require the controller
        controller_test = {"1_1_3", "1_2_3", "1_3_1", "1_3_2", "1_3_3", "1_1_2_3", "1_3_3_1"}
        controller_test_without_pressure = {"1_1_2_3", "1_3_3_1"}

        # determine which test we are running
        if (arguments["cmd"] == "Start"):
            print("Start")
            state = "Start"
            test = arguments["test"]
            imu.reset() # reset the imu
            
            # test requires full controller
            if (test in controller_test):
                controller.set_gains(float(arguments["Kp"]), float(arguments["Ki"]), float(arguments["Kd"])) # set the gains
                controller.set_target_angle(float(arguments["angle"])) # set the target angle

                if (test in controller_test_without_pressure):
                    record_pressure = False
                    log_file = logger("time [ms], angle [degrees]\n") # start the logger
                else:
                    record_pressure = True
                    log_file = logger("time [ms], angle [degrees], pressure_top [psig], pressure_bot [psig]\n") # start the logger

            # imu test or safety test
            if (test in {"1_1_2", "1_2_1"}):
                log_file = logger("time [ms], angle [degrees]\n") # start the logger
                
            # if leak test
            if (test == "1_2_2"):
                log_file = logger("time [ms], pressure_top [psig], pressure_bot [psig]\n") # start the logger

            start_time = time.ticks_ms() # start the timer

        elif (arguments["cmd"] == "Stop"):
            print("Stop")
            state = "Stop"
            log_file.close()

        elif (arguments["cmd"] == "Download"):
            print("Download")
            state = "Download"
            f = open("log_file.csv", 'r')
            log_data = f.read()
            download_data = "cmd: Download; data: " + str(log_data)
            socket.sendall(download_data.encode())
            f.close()
        
        with lock:
            g_state = state
            g_test = test
    
        # if running test, run controller
        if state == "Start":
            
            # read sensor data
            pressure_read_top = transducer_top.read() # top pressure transducer [PSI]
            pressure_read_bot = transducer_bot.read() # bottom pressure transducer [PSI]
            angle = -imu.euler()[0] # IMU angle [degrees]
            
            # wrap the angle value to -180:180 range
            if angle < -180:
                angle += 360
            
            # controller calculation
            if (test in controller_test):
                controller_pressure_top, controller_pressure_bot = controller.update(deg_to_rad(angle))
            
            with lock:
                g_controller_pressure_top = controller_pressure_top
                g_controller_pressure_bot = controller_pressure_bot
            
            # calculate timestep
            current_time = time.ticks_ms()
            elapsed_time = time.ticks_diff(current_time, start_time)
            
            # write to log file

            # if full controller
            if (test in controller_test):
                if (record_pressure):
                    log_file.write(elapsed_time, angle, pressure_read_top, pressure_read_bot)
                else:
                    log_file.write(elapsed_time, angle)

            # if imu test or safety test
            if (test in {"1_1_2", "1_2_1"}):
                log_file.write(elapsed_time, angle)

            # if leak test
            if (test == "1_2_2"):
                # every 5 seconds
                if (elapsed_time > 100 and elapsed_time % 2000 < 100):
                    log_file.write(elapsed_time, pressure_read_top, pressure_read_bot)

            
            # send data to interface
            update_data = "cmd: Update; "
            update_data += "time: " + str(elapsed_time/1000) + "; "
            update_data += "angle: " + str(angle) + "; "
            update_data += "pressure_top: " + str(pressure_read_top) + "; "
            update_data += "pressure_bot: " + str(pressure_read_bot) + "; "
            socket.send(update_data.encode())
    
    # close the thread
    with lock:
        g_state = "End"
    
    socket.close() # close when disconnected
