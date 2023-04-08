# main.py
# April 4th, 2023

# imports
from control_util import *
from sensor_util import *
from web_util import *
import machine
import _thread
import time

# pin defines
PIN_IMU_SDA = machine.Pin(0, mode=machine.Pin.IN)
PIN_IMU_SCL = machine.Pin(1, mode=machine.Pin.IN)

PIN_MOTOR_TOP_STEP = machine.Pin(2, mode=machine.Pin.OUT)
PIN_MOTOR_TOP_DIR = machine.Pin(3, mode=machine.Pin.OUT)
PIN_MOTOR_BOT_STEP = machine.Pin(5, mode=machine.Pin.OUT)
PIN_MOTOR_BOT_DIR = machine.Pin(4, mode=machine.Pin.OUT)

PIN_TRANSDUCER_TOP = machine.ADC(26)
PIN_TRANSDUCER_BOT = machine.ADC(27)

PIN_LED = machine.Pin("LED")

# global variables
g_state = "None"
g_pressure_needed_top = 0
g_pressure_needed_bot = 0

# set up imu
imu = bno055(machine.I2C(0, scl=PIN_IMU_SCL, sda=PIN_IMU_SDA, freq=400000))

# set up motors
motor_top = stepper_motor(PIN_MOTOR_TOP_STEP, PIN_MOTOR_TOP_DIR)
motor_bot = stepper_motor(PIN_MOTOR_BOT_STEP, PIN_MOTOR_BOT_DIR)

# set up state space model
K = matrix([0.9668, 0.8814]) # k matrix (1x2)
x0 = matrix([0.785398, 0.0]) # target state (1x2)
df = 0.180975 # 
F_avg = (1.246+4.3503)/2

# start network
network = start_network("Interface", "123456789")

# turn LED on
PIN_LED.on()

# stepper motor thread
def motors_thread():
    while True:
        if (g_state == "Start"):
            
            motor_top.set_wanted_psi(g_pressure_needed_top)
            motor_bot.set_wanted_psi(g_pressure_needed_bot)
            
            motor_top.update()
            motor_bot.update()
            
            time.sleep(0.0075)
            
        
# start motors thread
_thread.start_new_thread(motors_thread, ()) 

# main loop
while True:
    
    # initial startup
    serve_webpage() # wait to serve webpage
    socket = web_socket(("0.0.0.0", 80)) # create socket
    socket.upgrade() # upgrade to web socket
    
    # control loop
    while True:
        
        # get data
        data = socket.receive()
        if data==None: break;
        
        # decode message
        message = data.decode()
        
        if (message == "Start"):
            print("starting test")
            g_state = "Start"
            
        if (message == "Stop"):
            print("stopping test")
            g_state = "Stop"
        
        # if running test, run controller
        if g_state == "Start":
            
            angle = imu.euler()[0] # read imu angle
            angular_vel = imu.gyro()[0] # read imu angular velocity
            
            theta = -deg_to_rad(angle)
            theta_dot = -deg_to_rad(angular_vel)
            
            # controller calculation
            x = matrix([ [0.0, theta_dot], [theta, 0.0] ]) # state (2x2)
            x0 = matrix([desired_angle, 0.0]) # target state (1x2)
            cmd = x0 - K*x # target - k_matrix * state (1x2)
            T = cmd[0][0]
            F = matrix([[0.5, 0.5, F_avg], [df, -df, T]])
            results = F.rref()
            
            # set pressures
            g_pressure_needed_top = max(1.246, min(4.3503, results[0][2])) * 22.5537 - 3.1155
            g_pressure_needed_bot = max(1.246, min(4.3503, results[1][2])) * 22.5537 - 3.1155
        
    socket.close() # close when disconnected


