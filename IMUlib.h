#ifndef IMULIB_H
#define IMULIB_H

#include "IMUlib.c"


struct addresses;


void IMU_START();

int IMU_TEMP_RAW();

int IMU_ACCEL_X_RAW();
int IMU_ACCEL_Y_RAW();
int IMU_ACCEL_Z_RAW();

int IMU_GYRO_X_RAW();
int IMU_GYRO_Y_RAW();
int IMU_GYRO_Z_RAW();

int IMU_MAGN_X_RAW();
int IMU_MAGN_Y_RAW();
int IMU_MAGN_Z_RAW();


double IMU_TEMP_C();
double IMU_TEMP_F();

double IMU_ACCEL_X();
double IMU_ACCEL_Y();
double IMU_ACCEL_Z();

double IMU_GYRO_X();
double IMU_GYRO_Y();
double IMU_GYRO_Z();

double IMU_MAGN_X();
double IMU_MAGN_Y();
double IMU_MAGN_Z();
#endif