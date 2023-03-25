# main.py 

# imports
from math_util import *
from web_util import *
from bno055 import *
from machine import Pin, I2C, ADC

# global variables
g_desired_angle = 0
g_system_ready = False
g_is_running = False
g_elapsed_time = 0.0
g_log_filename = "log.csv"
g_LED = Pin("LED")

g_analog = ADC(26)

# state space model
K = matrix([1.0646, 0.8914]) # K Matrix (1x2)
x0 = matrix([0.785398, 0.0]) # Target State (1x2)

# connect imu
#i2c = I2C(0, scl=Pin(17), sda=Pin(16), freq=400000)
#imu = BNO055(i2c) # TODO : Error checking and resolution if imu doesnt connect
#imu_calibrated = False
#print("[log] IMU Connected!")

# create web server
server = web_server("MAE491 Interface", "123456789")
server.create_socket()

g_LED.on()
counter = 0
log_file = open(g_log_filename,"w")

try:

    while True:
        
        try:
            # retreive data from connection
            conn, data = server.receive()
            
            if (conn):
                if (data):
                    #print(data)
                    
                    # parse data
                    p = parser(data[0:64])
                    
                    angle = p.get_element("angle", "0")
                    state = p.get_element("state", "None")
                    
                    print("[log] request recieved (angle = " + angle + ", state = " + state + ")")
                    
                    # check if system is ready
                    if (g_system_ready == False and state == "Start"):
                        state = "None"
                        print("[log] System Not Ready!")
                    
                    if (state == "Start" and g_is_running == False):
                        g_is_running = True
                        print("[log] Starting Test...")
                        state == "None"
                    if (state == "Stop" and g_is_running == True):
                        g_is_running = False
                        print("[log] Stopping Test")
                        state == "None"
                    if (state == "Download" and g_is_running == True):
                        print("[log] Can not download log while the system is running!")
                        state == "None"
                        
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
                        conn.send("HTTP/1.0 200 OK\r\nContent-type: text/html\r\n\r\n")
                        conn.send(response)
                    g_system_ready = True
                else:
                    server.sockets.remove(conn)
                    print("[log] removing connection.")
                conn.close()
                
            counter += 1
            if (counter > 50):
                counter = 0
                if (g_is_running):
                    
                    
                    reading = (g_analog.read_u16() * 3.3 / 65536.0)
                    print(reading)
                    log_file.write(str(reading) + "\n")
                    
                    # get imu data
                    #if not imu_calibrated:
                    #    imu_calibrated = imu.calibrated()
                    #    angle = imu.euler()[1]
                    #    angular_vel = imu.gyro()[1]
                    #    theta = -deg_to_rad(angle)
                    #    theta_dot = -deg_to_rad(angular_vel)
                    
                    # controller calculation
                    #x = matrix([[theta, theta_dot], [0.0, 0.0]]) # state (2x2)
                    #cmd = x0 - K*x 
                    #df = 0.1715
                    #F_avg = 2.5
                    #T = cmd[0][0]
                    #F = matrix([[0.5, 0.5, F_avg], [df, -df, T]])
                    #results = F.rref() # more epic calculations
                    
                    # output results
                    #print("[log] left = " + str(results[0][2]))
                    #print("[log] right = " + str(results[1][2]))
            
        except OSError as e:
            # TODO : Emergency Shutdown procedure
            log_file.close()
            server.close()
except KeyboardInterrupt:
    log_file.close()
    g_LED.off()
    server.stop()
    
