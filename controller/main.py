# main.py
# April 5th, 2023

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
#i2c = machine.I2C(0, scl=PIN_IMU_SCL, sda=PIN_IMU_SDA, freq=400000)
#imu = bno055(i2c)

# set up motors
motor_top = stepper_motor(PIN_MOTOR_TOP_STEP, PIN_MOTOR_TOP_DIR)
motor_bot = stepper_motor(PIN_MOTOR_BOT_STEP, PIN_MOTOR_BOT_DIR)

# set up full state space model
controller = fss_controller(K = matrix([0.9668, 0.8814]), x0 = matrix([0.785398, 0.0]))

# start network
network = start_network("Interface", "123456789")

# turn LED on
PIN_LED.on()

# stepper motor thread
def motors_thread():
    while True:
        if (g_state == "Start"):
            
            motor_top.set_target_pressure(g_pressure_needed_top)
            motor_bot.set_target_pressure(g_pressure_needed_bot)
            
            motor_top.update()
            motor_bot.update()
            
            time.sleep_ms(MOTOR_STEP_TIME)
        
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
        
        # TODO: add in a parser to parse
        # our incoming messages so that
        # we can have more complex data
        
        if (message == "Start"):
            g_state = "Start"
        elif (message == "Stop"):
            g_state = "Stop"
        #elif (message =="Download"):
            # TODO: add download option
        
        # if running test, run controller
        if g_state == "Start":
            
            #angle = imu.euler()[0] # read imu angle
            #angular_vel = imu.gyro()[0] # read imu angular velocity
            angle = 45.0
            angular_vel = 1.0
            
            theta = -deg_to_rad(angle)
            theta_dot = -deg_to_rad(angular_vel)
            
            # controller calculation
            g_pressure_needed_top, g_pressure_needed_bot = controller.update(theta, theta_dot)
            
            # TODO: add logging back in
        
    socket.close() # close when disconnected


