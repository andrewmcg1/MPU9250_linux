# MPU9250_linux

This repository is a library that is able to get raw and calibrated values from an MPU9250 accelerameter, gyroscope, magnetometer, and thermometer. 
It also includes calibration methods for both the magnetometer and accelerameter, but not yet the gyroscope or thermometer.

Inside the IMUlib.h file you can find every function prototype for every sensor. 

In order to use the library functions, you first have to run IMU_START() to allocate memory and initialize variables. There is not yet a function to deallocate memory so be very careful.

This library compiles using gcc, and has been tested on a beaglebone blue.
