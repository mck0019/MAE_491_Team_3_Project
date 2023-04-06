# sensor_util.py
# April 5th, 2023

# imports
import time
import math
import struct

# imu defines

CONFIG_MODE = 0x00
ACCONLY_MODE = 0x01
MAGONLY_MODE = 0x02
GYRONLY_MODE = 0x03
ACCMAG_MODE = 0x04
ACCGYRO_MODE = 0x05
MAGGYRO_MODE = 0x06
AMG_MODE = 0x07
IMUPLUS_MODE = 0x08
COMPASS_MODE = 0x09
M4G_MODE = 0x0A
NDOF_FMC_OFF_MODE = 0x0B
NDOF_MODE = 0x0C

AXIS_P0 = bytes([0x21, 0x04])
AXIS_P1 = bytes([0x24, 0x00])
AXIS_P2 = bytes([0x24, 0x06])
AXIS_P3 = bytes([0x21, 0x02])
AXIS_P4 = bytes([0x24, 0x03])
AXIS_P5 = bytes([0x21, 0x01])
AXIS_P6 = bytes([0x21, 0x07])
AXIS_P7 = bytes([0x24, 0x05])

_MODE_REGISTER = 0x3D
_POWER_REGISTER = 0x3E
_AXIS_MAP_CONFIG = 0x41

# bno055 imu class
class bno055:
    def __init__(self, i2c, address=0x28, mode=NDOF_MODE, axis=AXIS_P4):
        self.i2c = i2c
        self.address = address
        if self.read_id() != bytes([0xA0, 0xFB, 0x32, 0x0F]):
            raise RuntimeError("Failed to find expected ID register values. Check wiring!")
        self.operation_mode(CONFIG_MODE)
        self.system_trigger(0x20)  # reset
        time.sleep_ms(700)
        self.power_mode(0x00)  # POWER_NORMAL
        self.axis(axis)
        self.page(0)
        time.sleep_ms(10)
        self.operation_mode(mode)
        self.system_trigger(0x80)  # external oscillator
        time.sleep(200)

    def read_registers(self, register, size=1):
        return self.i2c.readfrom_mem(self.address, register, size)

    def write_registers(self, register, data):
        self.i2c.writeto_mem(self.address, register, data)

    def operation_mode(self, mode=None):
        if mode:
            self.write_registers(_MODE_REGISTER, bytes([mode]))
        else:
            return self.read_registers(_MODE_REGISTER, 1)[0]

    def system_trigger(self, data):
        self.write_registers(0x3F, bytes([data]))

    def power_mode(self, mode=None):
        if mode:
            self.write_registers(_POWER_REGISTER, bytes([mode]))
        else:
            return self.read_registers(_POWER_REGISTER, 1)

    def page(self, num=None):
        if num:
            self.write_registers(0x3F, bytes([num]))
        else:
            self.read_registers(0x3F)

    def temperature(self):
        return self.read_registers(0x34, 1)[0]

    def read_id(self):
        return self.read_registers(0x00, 4)

    def axis(self, placement=None):
        if placement:
            self.write_registers(_AXIS_MAP_CONFIG, placement)
        else:
            return self.read_registers(_AXIS_MAP_CONFIG, 2)

    def quaternion(self):
        data = struct.unpack("<hhhh", self.read_registers(0x20, 8))
        return [d / (1 << 14) for d in data]  # [w, x, y, z]

    def euler(self):
        data = struct.unpack("<hhh", self.read_registers(0x1A, 6))
        return [d / 16 for d in data]  # [yaw, roll, pitch]

    def accelerometer(self):
        data = struct.unpack("<hhh", self.read_registers(0x08, 6))
        return [d / 100 for d in data]  # [x, y, z]

    def magnetometer(self):
        data = struct.unpack("<hhh", self.read_registers(0x0E, 6))
        return [d / 16 for d in data]  # [x, y, z]

    def gyroscope(self):
        data = struct.unpack("<hhh", self.read_registers(0x14, 6))
        return [d / 900 for d in data]  # [x, y, z]

    def linear_acceleration(self):
        data = struct.unpack("<hhh", self.read_registers(0x28, 6))
        return [d / 100 for d in data]  # [x, y, z]

    def gravity(self):
        data = struct.unpack("<hhh", self.read_registers(0x2E, 6))
        return [d / 100 for d in data]  # [x, y, z]


# motor defines
MOTOR_DIR_OPEN = 0
MOTOR_DIR_CLOSE = 1
MOTOR_STEP_TIME = 7

# motor class
class stepper_motor():
    
    def __init__(self, step, dir):
        self.dir_pin = dir
        self.step_pin = step
        self.current_dir = 0
        self.current_step = 0
        self.target_step = 0
    
    # set the current direction of the motor
    def set_dir(self, dir):
        self.current_dir = dir
        self.dir_pin.value(dir)
    
    # step the motor one time in its current direction
    def step_once(self):
        self.step_pin.value(1)
        self.step_pin.value(0)
        if (self.current_dir == 1):
            self.current_step += 1
        elif (self.current_dir == 0):
            self.current_step -= 1
            
    # set the target step
    def set_target_pressure(self, psi):
        self.target_step = round(((psi+41.531) / 0.0571)) # convert to step
    
    # update step
    def update(self):
        if (self.current_step != self.target_step):
            self.current_dir = (self.current_step - self.target_step) > 0  # calculate direction
            self.dir_pin.value(self.current_dir) # set the direction pin
            self.step_once() # step once
        
    # returns the motor to its original step position
    def return_to_zero(self):
        self.current_dir = MOTOR_DIR_CLOSE # set direction to close
        while self.current_step != 0:
            self.step_once()
            time.sleep_ms(MOTOR_STEP_TIME)
        
# transducer class
class transducer():
    
    def __init__(self, pin):
        self.pin = pin
        
    def read(self):
        return (0.00287737 * self.pin.read_u16() - 17.65142857) # convert to psi
