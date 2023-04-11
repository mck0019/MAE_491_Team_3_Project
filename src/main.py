# main.py
# Last Updated: April 10th, 2023
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
controller = pid_controller(1.1, 0.065, 2.5) # PID controller

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

time.sleep(1)

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
        
        if (state == "Start_Test"):
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
        arguments = {}
        split_str = message.split("; ")
        for item in split_str:
            key, value = item.split(": ")
            arguments[key.strip()] = value.strip()
        
        if (arguments["cmd"] == "Start_Test"):
            state = "Start_Test"
            imu.reset() # reset the imu
            controller.set_gains(float(arguments["Kp"]), float(arguments["Ki"]), float(arguments["Kd"])) # set the gains
            controller.set_target_angle(float(arguments["angle"])) # set the target angle
            log_file = logger("time [ms], angle [degrees], angular_vel [degress/s], pressure_top [psi], pressure_bot [psi]\n") # start the logger
            start_time = time.ticks_ms() # start the timer
            print("Start_Test")
        elif (arguments["cmd"] == "Start_Sensor"):
            state = "Start_Sensor"
            imu.reset() # reset the imu
            log_file = logger("time [ms], angle [degrees], angular_vel [degress/s], pressure_top [psi], pressure_bot [psi]\n") # start the logger
            start_time = time.ticks_ms() # start the timer
            print("Start_Sensor")
        elif (arguments["cmd"] == "Start_Safety"):
            state = "Start_Safety"
            print("Start Safety")
        elif (arguments["cmd"] == "Stop"):
            state = "Stop"
            log_file.close()
            print("Stop")
        elif (arguments["cmd"] == "Download"):
            state = "Download"
            print("Download")
            f = open("log_file.csv", 'r')
            log_data = f.read()
            download_data = "cmd: Download; data: " + str(log_data)
            socket.sendall(download_data.encode())
            f.close()
        
        with lock:
            g_state = state
    
        # if running test, run controller
        if state == "Start_Test":
            
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
            controller_pressure_top, controller_pressure_bot = controller.update(theta)
            
            with lock:
                g_controller_pressure_top = controller_pressure_top
                g_controller_pressure_bot = controller_pressure_bot
            
            # calculate timestep
            current_time = time.ticks_ms()
            elapsed_time = time.ticks_diff(current_time, start_time)
            
            # write to log file
            log_file.write(elapsed_time, angle, angular_vel, pressure_read_top, pressure_read_bot)
            
            # send data to interface
            update_data = "cmd: Update;"
            update_data += "angle: " + str(angle) + ";"
            update_data += "velocity: " + str(rad_to_deg(angular_vel)) + ";"
            update_data += "pressure_top: " + str(controller_pressure_top) + ";"
            update_data += "pressure_bot: " + str(controller_pressure_bot) + ";"
            socket.send(update_data.encode())
    
    # close the thread
    with lock:
        g_state = "End"
    
    socket.close() # close when disconnected
