# MAE 491 Team 3 Project

Controller software for our Senior Design Project: Active Single Axis Attitude Rocket Controller.

* Creates and runs a wireless access point labeled ```MAE491 Interface```
* Serves a webpage that interfaces the user with the system
* Runs a full state space controller
* Returns the test data as a .CSV  

## Set up
It is recommended to use the [Thonny IDE](https://thonny.org/) to upload files to the Raspberry Pi Pico W.

To configure Thonny go to ```Tools > Options... > Interpreter``` and select ```MicroPython(Raspberry Pi Pico)``` as your interpreter.

## Usage
In order to use that program on the Pico device, you need to upload all of the files within the src folder to the Pico. You then can disconnect the Raspberry Pi Pico W and power on the system. The Pico will automatically start running when the system is powered.