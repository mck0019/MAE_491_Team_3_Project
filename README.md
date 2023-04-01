# MAE 491 Team 3 Project

Controller software for our Senior Design Project: Active Single Axis Attitude Rocket Controller.

* Creates and runs a wireless access point labeled ```MAE491 Interface```
* Serves a webpage that interfaces the user with the system
* Runs a full state space controller
* Returns the test data as a .CSV  

## Set up
It is recommended to use the [Thonny IDE](https://thonny.org/) to upload files to the Raspberry Pi Pico W.

To configure Thonny go to ```Tools > Options... > Interpreter``` and select ```MicroPython(Raspberry Pi Pico)``` as your interpreter.

Then the Raspberry Pi Pico W can be connected via USB and any files open in Thonny can be saved on board the Pico.

If any file is named ```main.py``` and saved to the Pico, it will start running automatically when Pico has power.

## Usage

Each program is organized in its own folder. In order to use that program on the Pico device, you need to upload all of the files within that folder to the Pico. You then can disconnect the Raspberry Pi Pico W and insert it into the system. The Pico will automatically start running when the system is powered.

The current programs are:

* ```controller``` - This is the main controller program.
* ```sensor_data``` - This program reports the on board IMU and pressure transducer data.
* ```motor_control``` - This program remote controls just the stepper motors.
* ```pressure_testing``` - This program is for our testing procedures that checks for air leaks.
* ```safety_testing``` - This program is for our testing procedures that checks our safety stop block.

Each of these programs will create a wireless access point labeled ```MAE491 Interface``` .
Once connected the ip ```192.168.4.1``` can be typed in a browser to connect to the Pico to controller/view the data.
