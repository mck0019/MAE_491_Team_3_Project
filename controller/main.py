# main.py
# Last Updated: April 8th, 2023
# Author: Michael Key

# This program implements a full state space controller for out 
# MAE 491 Control Application Project : Active Single Axis Rocket 
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
import time

# global variables - these are variables that are shared across threads
g_lock = thread.allocate_lock() # thread lock
g_state = "None" # defines the current state of the system (ex: "Start", "Stop", etc.)
g_controller_pressure_top = 0 # the pressure output from the controller for the top nozzle. 
g_controller_pressure_bot = 0 # the pressure output from the controller for the bottom nozzle. 

# calculate K_matrix

I = 0.044 # moment of inertia [kg*m^2]
K = 0.12; # spring constant, [N*m/rad]
C = 0.2; # rotational damping friction [N*m*s/rad]
df = 0.180975 # moment arm [m]

# control matrices
A = matrix([[0, 1], [-K/I, 0]]) # (2x2)
B = matrix([[0], [1/I]]) # (2x1)
Q = matrix([[1.75, 0],[0, 1.08]]) # (2x2)
R = matrix([[1.5, 0], [0, 1.5]]) # (2x2)

K_matrix = lqr(A,B,Q,R)
print(K_matrix)

# set up controller
#controller = fss_controller(K_matrix = matrix([0.9668, 0.8814]), target_state = matrix([deg_to_rad(30), 0.0])) # full state space controller
#controller = pid_controller(3.0, 5.0, 2.0, deg_to_rad(45), 100) # PID controller
controller = fss_controller_real(matrix([0.9668, 0.8814]), 0.825, 100) # full state space controller with an integrator

imu = imu(scl_pin=Pin(0, mode=Pin.OUT), sda_pin=Pin(0, mode=Pin.OUT), freq=400000) # set up the imu
transducer_top = transducer(ADC(26)) # set up the top pressure transducer
transducer_bot = transducer(ADC(27)) # set up the bottom pressure transducer

network = start_network("MAE 491 Project Interface", "123456789") # open a wireless access point

# the stepper motor thread
def motors_thread():
    
    # define the stepper motors
    motor_top = stepper_motor(step=Pin(2, mode=Pin.OUT), dir=Pin(3, mode=Pin.OUT))
    motor_bot = stepper_motor(step=Pin(5, mode=Pin.OUT), dir=Pin(4, mode=Pin.OUT))
    
    # global variables
    global g_lock
    global g_state
    global g_controller_pressure_top
    global g_controller_pressure_bot
    
    while True:

        with g_lock:
            g_state
        
        if (state == "Start"):
            
            with g_lock:
                motor_top.set_target_pressure(g_controller_pressure_top)
                motor_bot.set_target_pressure(g_controller_pressure_bot)
            
            motor_top.update()
            motor_bot.update()
            
            #time.sleep_ms(round(new_time))
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
    
    g_state = "None"
    state = g_state
    start_time = 0
    pressure_needed_top = 0
    pressure_needed_bot = 0
    log_file = logger("time, angle, angular_vel, pressure_read_top, pressure_read_bot, pressure_needed_top, pressure_needed_bot\n")
    
    motor_thread = thread.start_new_thread(motors_thread, ()) # start the motor_thread
    
    # control loop
    while True:
        
        # get data
        data = socket.receive()
        if data==None: break;
        
        # decode message
        message = str(data.decode())

        result = {}
        split_str = message.split(", ")
        for item in split_str:
            key, value = item.split(": ")
            result[key.strip()] = value.strip()
        
        if (result["cmd"] == "Start"):
            state = "Start"
            controller.set_desired_angle(float(result["arg"]))
        elif (result["cmd"] == "Stop"):
            state = "Stop"
        
        # if running test, run controller
        if state == "Start":
            
            # read transducer data
            pressure_read_top = transducer_top.read()
            pressure_read_bot = transducer_bot.read()
            
            # read imu data
            angle = imu.euler()[0] # angle in degrees
            angular_vel = imu.gyroscope()[2] # angular velocity in radians per second
            
            # wrap the value
            if angle > 180:
                angle -= 360
            
            # convert to radians
            theta = deg_to_rad(angle)
            theta_dot = angular_vel
            
            # controller calculation
            pressure_needed_top, pressure_needed_bot = controller.update(theta, theta_dot)
            #pressure_needed_top, pressure_needed_bot = controller.update(theta)
            
            # write to log file
            current_time = time.ticks_ms()
            elapsed_time = time.ticks_diff(start_time, current_time)
            
            log_file.write(elapsed_time, angle, angular_vel, pressure_read_top, pressure_read_bot, g_pressure_needed_top, g_pressure_needed_bot)
            
            # send data
            update_data = "angle, " + str(-angle) + ", velocity, " + str(rad_to_deg(-angular_vel))
            socket.send(update_data.encode())
            
        if state == "Stop":
            log_file.close()
        
        
        lock.acquire()
        g_state = state
        g_pressure_needed_top = pressure_needed_top
        g_pressure_needed_bot = pressure_needed_bot
        lock.release()
        
    thread.exit() # stop the motor thread
    socket.close() # close when disconnected
