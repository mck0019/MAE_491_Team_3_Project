# main.py
# Last Updated: April 9th, 2023
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
import utime as time

# calculate K_matrix
I = 0.044 # moment of inertia [kg*m^2]
K = 0.12; # spring constant, [N*m/rad]
C = 0.2; # rotational damping friction [N*m*s/rad]
df = 0.180975 # moment arm [m]

# control matrices
A = matrix([[0, 1], [-K/I, 0]]) # (2x2)
B = matrix([[0], [1/I]]) # (2x1)
Q = matrix([[1.75, 0],[0, 1.08]]) # (2x2)
R = matrix([1.5]) # (1x1)

K_matrix = lqr(A,B,Q,R)

# set up controller
#controller = fss_controller(K_matrix = matrix([0.9668, 0.8814]), target_state = matrix([deg_to_rad(30), 0.0])) # full state space controller
#controller = pid_controller(3.0, 5.0, 2.0, deg_to_rad(45), 100) # PID controller
controller = fss_controller_w_int(K=matrix([0.9847, 0.8981]), K_i=0.5) # full state space controller with an integrator

# set up sensors
imu = imu(scl_pin=Pin(1, mode=Pin.OUT), sda_pin=Pin(0, mode=Pin.OUT), freq=400000) # set up the imu
transducer_top = transducer(ADC(26)) # set up the top pressure transducer
transducer_bot = transducer(ADC(27)) # set up the bottom pressure transducer

# open a wireless access point
network = start_network("MAE 491 Project Interface", "123456789")

# global variables - these are variables that are shared across threads
g_state = "None" # defines the current state of the system (ex: "Start", "Stop", etc.)
g_controller_pressure_top = 0 # the pressure output from the controller for the top nozzle. 
g_controller_pressure_bot = 0 # the pressure output from the controller for the bottom nozzle. 

# the stepper motor thread
def motors_thread(lock):
    
    # define the stepper motors
    motor_top = stepper_motor(step=Pin(2, mode=Pin.OUT), dir=Pin(3, mode=Pin.OUT))
    motor_bot = stepper_motor(step=Pin(5, mode=Pin.OUT), dir=Pin(4, mode=Pin.OUT))
    
    # global variables
    global g_state
    global g_controller_pressure_top
    global g_controller_pressure_bot
    
    while True:
        
        with lock:
            state = g_state
            controller_pressure_top = g_controller_pressure_top
            controller_pressure_bot = g_controller_pressure_bot
            
        if (state == "End"):
            thread.exit()
        
        if (state == "Start"):
            motor_top.set_target_pressure(controller_pressure_top)
            motor_bot.set_target_pressure(controller_pressure_bot)
            
            motor_top.update()
            motor_bot.update()
            
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
    controller_pressure_top = 0
    controller_pressure_bot = 0
    start_time = 0
    
    # start the motor_thread
    lock = thread.allocate_lock() # thread lock
    thread.start_new_thread(motors_thread, (lock,))
    
    # controller loop
    while True:
        
        # get data
        data = socket.receive()
        if data==None: break;
        
        # parse message
        message = str(data.decode())
        result = {}
        split_str = message.split(", ")
        for item in split_str:
            key, value = item.split(": ")
            result[key.strip()] = value.strip()
        
        # get command
        if (result["cmd"] == "Start"):
            state = "Start"
            imu.reset()
            controller.set_target_angle(float(result["arg"]))
            log_file = logger("time, angle, angular_vel, pressure_read_top, pressure_read_bot, pressure_needed_top, pressure_needed_bot, err, cumul_err\n")
            start_time = time.ticks_ms()
        elif (result["cmd"] == "Stop"):
            state = "Stop"
            log_file.close()
        elif (result["cmd"] == "Download"):
            state = "Download"
            print("Download")
            with open("log_file.csv", 'r') as f:
                log_data = f.read()
                
            download_data = "cmd: Download; data: " + str(log_data)
            socket.send(download_data.encode())
        
        with lock:
            g_state = state
    
        # if running test, run controller
        if state == "Start":
            
            # read sensor data
            pressure_read_top = transducer_top.read() # top pressure transducer [PSI]
            pressure_read_bot = transducer_bot.read() # bottom pressure transducer [PSI]
            angle = -imu.euler()[0] # IMU angle [degrees]
            angular_vel = imu.gyroscope()[2] # IMU angular velocity [rad/s]
            
            # wrap the angle value to -180:180 range
            if angle < -180:
                angle += 360
            
            
            # convert to radians
            theta = deg_to_rad(angle)
            theta_dot = angular_vel
            
            # controller calculation
            controller_pressure_top, controller_pressure_bot = controller.update(theta, theta_dot)
            
            with lock:
                g_controller_pressure_top = controller_pressure_top
                g_controller_pressure_bot = controller_pressure_bot
            
            # calculate timestep
            current_time = time.ticks_ms()
            elapsed_time = time.ticks_diff(current_time, start_time)
            
            # write to log file
            log_file.write(elapsed_time, angle, angular_vel, pressure_read_top, pressure_read_bot, controller_pressure_top, controller_pressure_bot, controller.err, controller.cumul_err)
            
            # send data to interface
            update_data = "cmd: Update; angle: " + str(angle) + "; velocity: " + str(rad_to_deg(angular_vel))
            socket.send(update_data.encode())
            
    
    # close the thread
    with lock:
        g_state = "End"
    
    socket.close() # close when disconnected
