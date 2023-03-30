# main.py 

# imports
from math_util import *
from web_util import *
from motor_util import *
from bno055 import *
from machine import Pin, I2C, ADC
import time

# pin defines
pin_imu_sda = Pin(0, mode=Pin.IN)
pin_imu_scl = Pin(1, mode=Pin.IN)

pin_motor_l_step = Pin(2, mode=Pin.OUT)
pin_motor_l_dir = Pin(3, mode=Pin.OUT)
pin_motor_r_step = Pin(4, mode=Pin.OUT)
pin_motor_r_dir = Pin(5, mode=Pin.OUT)

pin_pressure_transducer_l = ADC(26)
pin_pressure_transducer_r = ADC(27)

pin_LED = Pin("LED")

# global variables
g_desired_angle = 0
g_system_ready = False
g_is_running = False
g_log_filename = "log.csv"
g_current_task = "Idle"
g_start_time = 0
g_end_time = 0
g_time_step = 0

# set up imu
i2c = I2C(0, scl=pin_imu_scl, sda=pin_imu_sda, freq=400000)
imu = BNO055(i2c) # TODO : Error checking and resolution if imu doesnt connect
imu_calibrated = False
print("[log] IMU Connected!")

# set up motors
motor_l = motor_driver(pin_motor_l_dir, pin_motor_l_step)
motor_r = motor_driver(pin_motor_r_dir, pin_motor_r_step)

# set up state space model
K = matrix([1.0646, 0.8914]) # k matrix (1x2)
x0 = matrix([0.785398, 0.0]) # target state (1x2)
df = 0.1841
F_avg = (1.246+4.3503)/2

# create web server
server = web_server("MAE491 Interface", "123456789")
server.create_socket()

pin_LED.on()
log_file = open(g_log_filename,"w")

try:
    while True:
        
        # retreive data from connection
        conn, data = server.receive()
        
        if (conn):
            if (data):
                
                # parse data
                p = parser(data[0:64])
                
                angle = p.get_element("angle", "0")
                state = p.get_element("state", "None")
                
                print("[log] request recieved (angle = " + angle + ", state = " + state + ")")
                
                # check if system is ready
                if (g_system_ready == False and state == "Start"):
                    state = "None"
                    g_current_task = "Idle"
                    print("[log] System Not Ready!")
                if (state == "Start" and g_is_running == False):
                    g_is_running = True
                    g_start_time = time.ticks_us()
                    g_current_task = "Running Test..."
                    print("[log] Starting Test...")
                    state = "None"
                if (state == "Stop" and g_is_running == True):
                    g_is_running = False
                    g_time_step = 0
                    g_current_task = "Stopped Test"
                    print("[log] Stopping Test")
                    state = "None"
                if (state == "Download" and g_is_running == True):
                    g_current_task = "Log file sent"
                    print("[log] Can not download log while the system is running!")
                    state = "None"
                if (state == "Reset" and g_is_running == False):
                    print("[log] Can not reset while the system is running!")\
                    state = "None"
                    
                # set the desired angle
                g_desired_angle = deg_to_rad(int(angle))
                x0 = matrix([g_desired_angle, 0.0])
                
                # response
                if (state == "Download"):
                    response = get_csv(g_log_filename)
                    conn.send("HTTP/1.0 200 OK\r\nContent-type: text/csv\r\n\r\n")
                    conn.send(response)
                else:
                    response = get_html("index.html")
                    response = response.replace("{slider_value}", angle)
                    response = response.replace("{state}", g_current_task)
                    conn.send("HTTP/1.0 200 OK\r\nContent-type: text/html\r\n\r\n")
                    conn.send(response)
                g_system_ready = True
            else:
                server.sockets.remove(conn)
                print("[log] removing connection.")
            conn.close()
            
        time.sleep(0.05) # sleep 50 ms

        # if running test
        if (g_is_running):
            
            # read pressure tranducers
            pressure_in_l = (pin_pressure_transducer_l.read_u16() * 3.3 / 65536.0) # in Volts
            pressure_in_r = (pin_pressure_transducer_r.read_u16() * 3.3 / 65536.0) # in Volts
            
            # read imu data
            if not imu_calibrated:
                imu_calibrated = imu.calibrated()
                angle = imu.euler()[1]
                angular_vel = imu.gyro()[1]
                theta = -deg_to_rad(angle)
                theta_dot = -deg_to_rad(angular_vel)
            
            # controller calculation
            x = matrix([ [0.0, theta_dot], [theta, 0.0] ]) # state (2x2)
            cmd = x0 - K*x # target - k_matrix * state 
            T = cmd[0][0]
            F = matrix([[0.5, 0.5, F_avg], [df, -df, T]])
            results = F.rref()
            pressure_needed_l = (max(1.246, min(4.3503, results[0][2])) * 22.5537 + 11.5845) - 14.7
            pressure_needed_r = (max(1.246, min(4.3503, results[1][2])) 22.5537 + 11.5845) - 14.7
            
            # TODO : calculate what angle is needs for the
            # required pressure using pressure_in
            
            # set angle
            motor_l.set_step(SET_ANGLE_HERE)
            motor.r.set_step(SET_ANGLE_HERE)
            
            # update motors
            motor_l.update() # TODO : Check if this update is fast enough with the controller calculation,
            motor_r.update() # if not we can run the update multiple times per "frame"
            
            # get time step
            g_end_time = time.ticks_us()
            g_time_step = time.ticks_diff(end, start)
            
            # output results
            log.write('{:9.2f}'.format(g_time_step) + ", " + '{:11.5f}'.format(angle) + ", " + '{:19.5f}'.format(angular_vel) + ", ")
            log.write('{:11.3f}'.format(pressure_needed_l) + ", " + '{:12.3f}'.format(pressure_needed_r) + "\n")
            
except OSError:
except KeyboardInterrupt:
    log_file.close()
    pin_LED.off()
    server.stop()
    
