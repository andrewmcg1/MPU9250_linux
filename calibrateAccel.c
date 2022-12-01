#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "IMUlib.h"

int main()
{
    char* ACCEL_CALIBRATION_VALUES = "CalibrationValues/Accelerameter.txt";

    FILE *calibration_values;
    char wait;

    double x_calibration[3];
    double y_calibration[3];
    double z_calibration[3];

    double accel_multipliers[3];
    double accel_offsets[3];


    IMU_START();


    calibration_values = fopen(ACCEL_CALIBRATION_VALUES, "w");

    if(calibration_values == NULL)
    {
        printf("file cannot be opened\n");
        return 0;
    }


    printf("When the x-axis is pointing up, press enter\n");
    while(wait != '\n')
    {
        scanf("%c", &wait);
    }
    wait = 0;
    x_calibration[0] = IMU_ACCEL_X_RAW();
    y_calibration[1] = IMU_ACCEL_Y_RAW();
    z_calibration[1] = IMU_ACCEL_Z_RAW();

    printf("When the x-axis is pointing down, press enter\n");
    while(wait != '\n')
    {
        scanf("%c", &wait);
    }
    wait = 0;
    x_calibration[2] = IMU_ACCEL_X_RAW();
    y_calibration[1] += IMU_ACCEL_Y_RAW();
    z_calibration[1] += IMU_ACCEL_Z_RAW();




    printf("When the y-axis is pointing up, press enter\n");
    while(wait != '\n')
    {
        scanf("%c", &wait);
    }
    wait = 0;
    x_calibration[1] += IMU_ACCEL_X_RAW();
    y_calibration[0] = IMU_ACCEL_Y_RAW();
    z_calibration[1] += IMU_ACCEL_Z_RAW();

    printf("When the y-axis is pointing down, press enter\n");
    while(wait != '\n')
    {
        scanf("%c", &wait);
    }
    wait = 0;
    x_calibration[1] += IMU_ACCEL_X_RAW();
    y_calibration[2] = IMU_ACCEL_Y_RAW();
    z_calibration[1] += IMU_ACCEL_Z_RAW();




    printf("When the z-axis is pointing up, press enter\n");
    while(wait != '\n')
    {
        scanf("%c", &wait);
    }
    wait = 0;
    x_calibration[1] += IMU_ACCEL_X_RAW();
    y_calibration[1] += IMU_ACCEL_Y_RAW();
    z_calibration[0] = IMU_ACCEL_Z_RAW();

    printf("When the z-axis is pointing down, press enter\n");
    while(wait != '\n')
    {
        scanf("%c", &wait);
    }
    wait = 0;
    x_calibration[1] += IMU_ACCEL_X_RAW();
    y_calibration[1] += IMU_ACCEL_Y_RAW();
    z_calibration[2] = IMU_ACCEL_Z_RAW();

    x_calibration[1] /= 4;
    y_calibration[1] /= 4;
    z_calibration[1] /= 4;


    accel_multipliers[0] = 2. / (x_calibration[2] - x_calibration[0]);
    accel_multipliers[1] = 2. / (y_calibration[2] - y_calibration[0]);
    accel_multipliers[2] = 2. / (z_calibration[2] - z_calibration[0]);

    accel_offsets[0] = x_calibration[1];
    accel_offsets[1] = y_calibration[1];
    accel_offsets[2] = z_calibration[1];

    fprintf(calibration_values, "%lf\t%lf\t%lf\n", accel_multipliers[0], accel_multipliers[1], accel_multipliers[2]);
    fprintf(calibration_values, "%lf\t%lf\t%lf\n", accel_offsets[0], accel_offsets[1], accel_offsets[2]);

    printf("%lf\t%lf\t%lf\n", accel_multipliers[0], accel_multipliers[1], accel_multipliers[2]);
    printf("%lf\t%lf\t%lf\n", accel_offsets[0], accel_offsets[1], accel_offsets[2]);

    fclose(calibration_values);

    return 0;
}