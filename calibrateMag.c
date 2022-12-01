#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "IMUlib.h"

int main()
{
    clock_t start, deltaTime;
    FILE* calibration_values;

    double offsets[3];
    double scale[3];
    double maxValues[3];
    double minValues[3];
    double deltas[3];
    double avgDelta;

    double magX, magY, magZ;

    maxValues[0] = 0;
    maxValues[1] = 0;
    maxValues[2] = 0;

    minValues[0] = 50000;
    minValues[1] = 50000;
    minValues[2] = 50000;

    calibration_values = fopen("CalibrationValues/Magnetometer.txt", "w");


    printf("Move Magnetometer in a figure-eight motion\n");

    IMU_START();
    for(int i = 0; i < 3; i++)
    {
        start = clock();
        do {
            magX = IMU_MAGN_X_RAW();
            magY = IMU_MAGN_Y_RAW();
            magZ = IMU_MAGN_Z_RAW();

            if(magX > maxValues[0])
            {
                maxValues[0] = magX;
            }
            if(magX < minValues[0])
            {
                minValues[0] = magX;
            }

            if(magY > maxValues[1])
            {
                maxValues[1] = magY;
            }
            if(magY < minValues[1])
            {
                minValues[1] = magY;
            }

            if(magZ > maxValues[2])
            {
                maxValues[2] = magZ;
            }
            if(magZ < minValues[2])
            {
                minValues[2] = magZ;
            }
            

            deltaTime = clock() - start;
        } while(deltaTime < 1000000);
        printf("Keep Going\n");
    }
    printf("Done Calibrating\n");

    offsets[0] = (maxValues[0] + minValues[0]) / 2.;
    offsets[1] = (maxValues[1] + minValues[1]) / 2.;
    offsets[2] = (maxValues[2] + minValues[2]) / 2.;

    deltas[0] = (maxValues[0] - minValues[0]) / 2.;
    deltas[1] = (maxValues[1] - minValues[1]) / 2.;
    deltas[2] = (maxValues[2] - minValues[2]) / 2.;

    avgDelta = (deltas[0] + deltas[1] + deltas[2]) / 3.;

    scale[0] = (avgDelta / deltas[0]) * .6;
    scale[1] = (avgDelta / deltas[1]) * .6;
    scale[2] = (avgDelta / deltas[2]) * .6;


    fprintf(calibration_values, "%lf\t%lf\t%lf\n", scale[0], scale[1], scale[2]);
    fprintf(calibration_values, "%lf\t%lf\t%lf\n", offsets[0], offsets[1], offsets[2]);

    fclose(calibration_values);

    return 0;
}
