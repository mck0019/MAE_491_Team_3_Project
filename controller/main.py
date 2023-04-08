# main.py
# April 5th, 2023

#TODO
#
# add in a way to reset the imu angle via the interface (the start button)


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
i2c = machine.I2C(0, scl=PIN_IMU_SCL, sda=PIN_IMU_SDA, freq=400000)
imu = bno055(i2c)

# set up pressure transduces
transducer_top = transducer(PIN_TRANSDUCER_TOP)
transducer_bot = transducer(PIN_TRANSDUCER_BOT)

# set up full state space model
#controller = fss_controller(K_matrix = matrix([0.9668, 0.8814]), target_state = matrix([deg_to_rad(30), 0.0]))
#controller = pid_controller(3.0, 5.0, 2.0, deg_to_rad(45), 100)
controller = fss_controller_real(matrix([0.9668, 0.8814]), 0.825, 100)

# start network
network = start_network("MAE 491 Project Interface", "123456789")

# turn LED on
PIN_LED.on()

lock = _thread.allocate_lock()

# stepper motor thread
def motors_thread():
    
    motor_top = stepper_motor(PIN_MOTOR_TOP_STEP, PIN_MOTOR_TOP_DIR)
    motor_bot = stepper_motor(PIN_MOTOR_BOT_STEP, PIN_MOTOR_BOT_DIR)
    
    global g_state
    global g_pressure_needed_top
    global g_pressure_needed_bot
    
    while True:

        lock.acquire()
        state = g_state
        pressure_needed_top = g_pressure_needed_top
        pressure_needed_bot = g_pressure_needed_bot
        lock.release()
        
        if (state == "Start"):
            
            motor_top.set_target_pressure(pressure_needed_top)
            motor_bot.set_target_pressure(pressure_needed_bot)
            
            #motor_top_error = abs(motor_top.target_step - motor_top.current_step)
            #motor_bot_error = abs(motor_bot.target_step - motor_bot.current_step)
            
            #new_time_top = ((motor_top_error) / (1600)) * (50 - 7) + 7
            #new_time_bot = ((motor_bot_error) / (1600)) * (50 - 7) + 7
            
            #new_time = max(new_time_top, new_time_bot)
            #new_time = min(7, max(50, new_time))
            
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
    
    motor_thread = _thread.start_new_thread(motors_thread, ()) # start the motor_thread
    
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
        
    _thread.exit() # stop the motor thread
    socket.close() # close when disconnected

