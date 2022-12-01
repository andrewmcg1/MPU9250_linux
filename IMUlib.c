#include <stdio.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <stdint.h>
#include <sys/ioctl.h> 
#include <unistd.h>
#include <time.h>

#include "IMUlib.h"
   
struct ACCEL_ADDRESSES {
    int fd, mg;
    char I2C_ADDR;
    char MAG_ADDR;

    char *TEMP_OUT_H;
    char *TEMP_OUT_L;

    char *ACCEL_OUT_H;
    char *ACCEL_OUT_L;

    char *GYRO_OUT_H;
    char *GYRO_OUT_L;

    char *MAGN_OUT_H;
    char *MAGN_OUT_L;

    char *MAG_START;
    char *MAG_STATUS2;
    char *MAG_STATUS_OUT;

    FILE *TEMP_CALIBRATION;
    FILE *ACCEL_CALIBRATION;
    FILE *GYRO_CALIBRATION;
    FILE *MAGN_CALIBRATION;

    double *temp_multipliers;
    double *temp_offsets;

    double *accel_multipliers;
    double *accel_offsets;

    double *gyro_multipliers;
    double *gyro_offsets;

    double *magn_multipliers;
    double *magn_offsets;
};
struct ACCEL_ADDRESSES *IMU;



void IMU_START()
{
    IMU = (struct ACCEL_ADDDRESSES*)malloc(sizeof(struct ACCEL_ADDRESSES));

    IMU->I2C_ADDR = 0x68;
    IMU->MAG_ADDR = 0x0c;

    IMU->TEMP_OUT_H = (char*)malloc(sizeof(char));
    IMU->TEMP_OUT_L = (char*)malloc(sizeof(char));

    IMU->ACCEL_OUT_H = (char*)malloc(sizeof(char));
    IMU->ACCEL_OUT_L = (char*)malloc(sizeof(char));

    IMU->GYRO_OUT_H = (char*)malloc(sizeof(char));
    IMU->GYRO_OUT_L = (char*)malloc(sizeof(char));

    IMU->MAGN_OUT_H = (char*)malloc(sizeof(char));
    IMU->MAGN_OUT_L = (char*)malloc(sizeof(char));

    IMU->MAG_START = (char*)malloc(sizeof(char) * 2);
    IMU->MAG_STATUS2 = (char*)malloc(sizeof(char));
    IMU->MAG_STATUS_OUT = (char*)malloc(sizeof(char));

    IMU->TEMP_CALIBRATION = fopen("CalibrationValues/Temperature.txt", "r");
    IMU->ACCEL_CALIBRATION = fopen("CalibrationValues/Accelerameter.txt", "r");
    IMU->GYRO_CALIBRATION = fopen("CalibrationValues/Gyroscope.txt", "r");
    IMU->MAGN_CALIBRATION = fopen("CalibrationValues/Magnetometer.txt", "r");

    IMU->temp_multipliers = (double*)malloc(sizeof(double));
    IMU->temp_offsets = (double*)malloc(sizeof(double));

    IMU->accel_multipliers = (double*)malloc(sizeof(double) * 3);
    IMU->accel_offsets = (double*)malloc(sizeof(double) * 3);

    IMU->gyro_multipliers = (double*)malloc(sizeof(double) * 3);
    IMU->gyro_offsets = (double*)malloc(sizeof(double) * 3);

    IMU->magn_multipliers = (double*)malloc(sizeof(double) * 3);
    IMU->magn_offsets = (double*)malloc(sizeof(double) * 3);


    fscanf(IMU->TEMP_CALIBRATION, "%lf", IMU->temp_multipliers);
    fscanf(IMU->TEMP_CALIBRATION, "%lf", IMU->temp_offsets);

    fscanf(IMU->ACCEL_CALIBRATION, "%lf %lf %lf", &IMU->accel_multipliers[0], &IMU->accel_multipliers[1], &IMU->accel_multipliers[2]);
    fscanf(IMU->ACCEL_CALIBRATION, "%lf %lf %lf", &IMU->accel_offsets[0], &IMU->accel_offsets[1], &IMU->accel_offsets[2]);

    fscanf(IMU->GYRO_CALIBRATION, "%lf %lf %lf", &IMU->gyro_multipliers[0], &IMU->gyro_multipliers[1], &IMU->gyro_multipliers[2]);
    fscanf(IMU->GYRO_CALIBRATION, "%lf %lf %lf", &IMU->gyro_offsets[0], &IMU->gyro_offsets[1], &IMU->gyro_offsets[2]);

    fscanf(IMU->MAGN_CALIBRATION, "%lf %lf %lf", &IMU->magn_multipliers[0], &IMU->magn_multipliers[1], &IMU->magn_multipliers[2]);
    fscanf(IMU->MAGN_CALIBRATION, "%lf %lf %lf", &IMU->magn_offsets[0], &IMU->magn_offsets[1], &IMU->magn_offsets[2]);


    *(IMU->MAG_STATUS2) = 0x09;
    IMU->MAG_START[0] = 0x37;
    IMU->MAG_START[1] = 0x22;

    IMU->fd = open("/dev/i2c-2", O_RDWR);
    if (IMU->fd < 0) {
        printf("Error opening IMU\n");
        return;
    }

    if (ioctl(IMU->fd, I2C_SLAVE, IMU->I2C_ADDR) < 0) {
        printf("IMU ioctl error\n");
        return;
    }

    write(IMU->fd, IMU->MAG_START, 2);
    IMU->mg = open("/dev/i2c-2", O_RDWR);
    if (IMU->mg < 0) {
        printf("Error opening Magnetometer\n");
        return;
    }

    if (ioctl(IMU->mg, I2C_SLAVE, IMU->MAG_ADDR) < 0) {
        printf("Magnetometer ioctl error\n");
        return;
    }

    IMU->MAG_START[0] = 0x0A;
    IMU->MAG_START[1] = 0b10110;
    write(IMU->mg, IMU->MAG_START, 2);

    fclose(IMU->ACCEL_CALIBRATION);
    fclose(IMU->GYRO_CALIBRATION);
    fclose(IMU->MAGN_CALIBRATION);

}

int IMU_TEMP_RAW()
{
    int16_t final;

    *(IMU->TEMP_OUT_H) = 0x41;
    *(IMU->TEMP_OUT_L) = 0x42;

    write(IMU->fd, IMU->TEMP_OUT_H, 1);
    read(IMU->fd, IMU->TEMP_OUT_H, 1);

    write(IMU->fd, IMU->TEMP_OUT_L, 1);
    read(IMU->fd, IMU->TEMP_OUT_L, 1);

    final = (*(IMU->TEMP_OUT_H) << 8) | *(IMU->TEMP_OUT_L);

    return final;
}

int IMU_ACCEL_X_RAW()
{
    int16_t final;

    *(IMU->ACCEL_OUT_H) = 0x3B;
    *(IMU->ACCEL_OUT_L) = 0x3C;

    write(IMU->fd, IMU->ACCEL_OUT_H, 1);
    read(IMU->fd, IMU->ACCEL_OUT_H, 1);

    write(IMU->fd, IMU->ACCEL_OUT_L, 1);
    read(IMU->fd, IMU->ACCEL_OUT_L, 1);

    final = (*(IMU->ACCEL_OUT_H) << 8) | *(IMU->ACCEL_OUT_L);

    return final;
}

int IMU_ACCEL_Y_RAW()
{
    int16_t final;

    *(IMU->ACCEL_OUT_H) = 0x3D;
    *(IMU->ACCEL_OUT_L) = 0x3E;

    write(IMU->fd, IMU->ACCEL_OUT_H, 1);
    read(IMU->fd, IMU->ACCEL_OUT_H, 1);

    write(IMU->fd, IMU->ACCEL_OUT_L, 1);
    read(IMU->fd, IMU->ACCEL_OUT_L, 1);

    final = (*(IMU->ACCEL_OUT_H) << 8) | *(IMU->ACCEL_OUT_L);

    return final;
}

int IMU_ACCEL_Z_RAW()
{
    int16_t final;

    *(IMU->ACCEL_OUT_H) = 0x3F;
    *(IMU->ACCEL_OUT_L) = 0x40;

    write(IMU->fd, IMU->ACCEL_OUT_H, 1);
    read(IMU->fd, IMU->ACCEL_OUT_H, 1);

    write(IMU->fd, IMU->ACCEL_OUT_L, 1);
    read(IMU->fd, IMU->ACCEL_OUT_L, 1);

    final = (*(IMU->ACCEL_OUT_H) << 8) | *(IMU->ACCEL_OUT_L);

    return final;
}


int IMU_GYRO_X_RAW()
{
    int16_t final;

    *(IMU->GYRO_OUT_H) = 0x43;
    *(IMU->GYRO_OUT_L) = 0x44;

    write(IMU->fd, IMU->GYRO_OUT_H, 1);
    read(IMU->fd, IMU->GYRO_OUT_H, 1);

    write(IMU->fd, IMU->GYRO_OUT_L, 1);
    read(IMU->fd, IMU->GYRO_OUT_L, 1);

    final = (*(IMU->GYRO_OUT_H) << 8) | *(IMU->GYRO_OUT_L);

    return final;
}

int IMU_GYRO_Y_RAW()
{
    int16_t final;

    *(IMU->GYRO_OUT_H) = 0x45;
    *(IMU->GYRO_OUT_L) = 0x46;

    write(IMU->fd, IMU->GYRO_OUT_H, 1);
    read(IMU->fd, IMU->GYRO_OUT_H, 1);

    write(IMU->fd, IMU->GYRO_OUT_L, 1);
    read(IMU->fd, IMU->GYRO_OUT_L, 1);

    final = (*(IMU->GYRO_OUT_H) << 8) | *(IMU->GYRO_OUT_L);

    return final;
}

int IMU_GYRO_Z_RAW()
{
    int16_t final;

    *(IMU->GYRO_OUT_H) = 0x47;
    *(IMU->GYRO_OUT_L) = 0x48;

    write(IMU->fd, IMU->GYRO_OUT_H, 1);
    read(IMU->fd, IMU->GYRO_OUT_H, 1);

    write(IMU->fd, IMU->GYRO_OUT_L, 1);
    read(IMU->fd, IMU->GYRO_OUT_L, 1);

    final = (*(IMU->GYRO_OUT_H) << 8) | *(IMU->GYRO_OUT_L);

    return final;
}

int IMU_MAGN_X_RAW()
{
    int16_t final;

    write(IMU->mg, IMU->MAG_STATUS2, 1);
    read(IMU->mg, IMU->MAG_STATUS_OUT, 1);

    *(IMU->MAGN_OUT_H) = 0x06;
    *(IMU->MAGN_OUT_L) = 0x05;

    write(IMU->mg, IMU->MAGN_OUT_H, 1);
    read(IMU->mg, IMU->MAGN_OUT_H, 1);

    write(IMU->mg, IMU->MAG_STATUS2, 1);
    read(IMU->mg, IMU->MAG_STATUS_OUT, 1);

    write(IMU->mg, IMU->MAGN_OUT_L, 1);
    read(IMU->mg, IMU->MAGN_OUT_L, 1);

    final = (*(IMU->MAGN_OUT_H) << 8) | *(IMU->MAGN_OUT_L);

    write(IMU->mg, IMU->MAG_STATUS2, 1);
    read(IMU->mg, IMU->MAG_STATUS_OUT, 1);

    return final;
}

int IMU_MAGN_Y_RAW()
{
    int16_t final;

    write(IMU->mg, IMU->MAG_STATUS2, 1);
    read(IMU->mg, IMU->MAG_STATUS_OUT, 1);

    *(IMU->MAGN_OUT_H) = 0x04;
    *(IMU->MAGN_OUT_L) = 0x03;

    write(IMU->mg, IMU->MAGN_OUT_H, 1);
    read(IMU->mg, IMU->MAGN_OUT_H, 1);

    write(IMU->mg, IMU->MAG_STATUS2, 1);
    read(IMU->mg, IMU->MAG_STATUS_OUT, 1);

    write(IMU->mg, IMU->MAGN_OUT_L, 1);
    read(IMU->mg, IMU->MAGN_OUT_L, 1);

    final = (*(IMU->MAGN_OUT_H) << 8) | *(IMU->MAGN_OUT_L);

    write(IMU->mg, IMU->MAG_STATUS2, 1);
    read(IMU->mg, IMU->MAG_STATUS_OUT, 1);

    return final;
}

int IMU_MAGN_Z_RAW()
{
    int16_t final;

    write(IMU->mg, IMU->MAG_STATUS2, 1);
    read(IMU->mg, IMU->MAG_STATUS_OUT, 1);

    *(IMU->MAGN_OUT_H) = 0x08;
    *(IMU->MAGN_OUT_L) = 0x07;

    write(IMU->mg, IMU->MAGN_OUT_H, 1);
    read(IMU->mg, IMU->MAGN_OUT_H, 1);

    write(IMU->mg, IMU->MAG_STATUS2, 1);
    read(IMU->mg, IMU->MAG_STATUS_OUT, 1);

    write(IMU->mg, IMU->MAGN_OUT_L, 1);
    read(IMU->mg, IMU->MAGN_OUT_L, 1);

    final = ((*(IMU->MAGN_OUT_H) << 8) | *(IMU->MAGN_OUT_L)) * -1;

    write(IMU->mg, IMU->MAG_STATUS2, 1);
    read(IMU->mg, IMU->MAG_STATUS_OUT, 1);

    return final;
}



double IMU_TEMP_C()
{
    int16_t final;
    double C;

    *(IMU->TEMP_OUT_H) = 0x41;
    *(IMU->TEMP_OUT_L) = 0x42;

    write(IMU->fd, IMU->TEMP_OUT_H, 1);
    read(IMU->fd, IMU->TEMP_OUT_H, 1);

    write(IMU->fd, IMU->TEMP_OUT_L, 1);
    read(IMU->fd, IMU->TEMP_OUT_L, 1);

    final = (*(IMU->TEMP_OUT_H) << 8) | *(IMU->TEMP_OUT_L);

    C = final * *(IMU->temp_multipliers) - *(IMU->temp_offsets);

    return C;
}

double IMU_TEMP_F()
{
    double C = IMU_TEMP_C();
    double F = C * (9./5) + 32;
    return F;
}

double IMU_ACCEL_X()
{
    int16_t final;
    double Gs;

    *(IMU->ACCEL_OUT_H) = 0x3B;
    *(IMU->ACCEL_OUT_L) = 0x3C;

    write(IMU->fd, IMU->ACCEL_OUT_H, 1);
    read(IMU->fd, IMU->ACCEL_OUT_H, 1);

    write(IMU->fd, IMU->ACCEL_OUT_L, 1);
    read(IMU->fd, IMU->ACCEL_OUT_L, 1);

    final = (*(IMU->ACCEL_OUT_H) << 8) | *(IMU->ACCEL_OUT_L);

    Gs = final - IMU->accel_offsets[0];
    Gs *= IMU->accel_multipliers[0];

    return Gs;
}

double IMU_ACCEL_Y()
{
    int16_t final;
    double Gs;

    *(IMU->ACCEL_OUT_H) = 0x3D;
    *(IMU->ACCEL_OUT_L) = 0x3E;

    write(IMU->fd, IMU->ACCEL_OUT_H, 1);
    read(IMU->fd, IMU->ACCEL_OUT_H, 1);

    write(IMU->fd, IMU->ACCEL_OUT_L, 1);
    read(IMU->fd, IMU->ACCEL_OUT_L, 1);

    final = (*(IMU->ACCEL_OUT_H) << 8) | *(IMU->ACCEL_OUT_L);

    Gs = final - IMU->accel_offsets[1];
    Gs *= IMU->accel_multipliers[1];

    return Gs;
}

double IMU_ACCEL_Z()
{
    int16_t final;
    double Gs;

    *(IMU->ACCEL_OUT_H) = 0x3F;
    *(IMU->ACCEL_OUT_L) = 0x40;

    write(IMU->fd, IMU->ACCEL_OUT_H, 1);
    read(IMU->fd, IMU->ACCEL_OUT_H, 1);

    write(IMU->fd, IMU->ACCEL_OUT_L, 1);
    read(IMU->fd, IMU->ACCEL_OUT_L, 1);

    final = (*(IMU->ACCEL_OUT_H) << 8) | *(IMU->ACCEL_OUT_L);

    Gs = final - IMU->accel_offsets[2];
    Gs *= IMU->accel_multipliers[2];

    return Gs;
}


double IMU_GYRO_X()
{
    int16_t final;
    double dps;

    *(IMU->GYRO_OUT_H) = 0x43;
    *(IMU->GYRO_OUT_L) = 0x44;

    write(IMU->fd, IMU->GYRO_OUT_H, 1);
    read(IMU->fd, IMU->GYRO_OUT_H, 1);

    write(IMU->fd, IMU->GYRO_OUT_L, 1);
    read(IMU->fd, IMU->GYRO_OUT_L, 1);

    final = (*(IMU->GYRO_OUT_H) << 8) | *(IMU->GYRO_OUT_L);

    dps = final - IMU->gyro_offsets[0];
    dps *= IMU->gyro_multipliers[0];

    return dps;
}

double IMU_GYRO_Y()
{
    int16_t final;
    double dps;

    *(IMU->GYRO_OUT_H) = 0x45;
    *(IMU->GYRO_OUT_L) = 0x46;

    write(IMU->fd, IMU->GYRO_OUT_H, 1);
    read(IMU->fd, IMU->GYRO_OUT_H, 1);

    write(IMU->fd, IMU->GYRO_OUT_L, 1);
    read(IMU->fd, IMU->GYRO_OUT_L, 1);

    final = (*(IMU->GYRO_OUT_H) << 8) | *(IMU->GYRO_OUT_L);

    dps = final - IMU->gyro_offsets[1];
    dps *= IMU->gyro_multipliers[1];

    return dps;
}

double IMU_GYRO_Z()
{
    int16_t final;
    double dps;

    *(IMU->GYRO_OUT_H) = 0x47;
    *(IMU->GYRO_OUT_L) = 0x48;

    write(IMU->fd, IMU->GYRO_OUT_H, 1);
    read(IMU->fd, IMU->GYRO_OUT_H, 1);

    write(IMU->fd, IMU->GYRO_OUT_L, 1);
    read(IMU->fd, IMU->GYRO_OUT_L, 1);

    final = (*(IMU->GYRO_OUT_H) << 8) | *(IMU->GYRO_OUT_L);

    dps = final - IMU->gyro_offsets[2];
    dps *= IMU->gyro_multipliers[2];

    return dps;
}

double IMU_MAGN_X()
{
    int16_t final;
    double uT;

    write(IMU->mg, IMU->MAG_STATUS2, 1);
    read(IMU->mg, IMU->MAG_STATUS_OUT, 1);

    *(IMU->MAGN_OUT_H) = 0x06;
    *(IMU->MAGN_OUT_L) = 0x05;

    write(IMU->mg, IMU->MAGN_OUT_H, 1);
    read(IMU->mg, IMU->MAGN_OUT_H, 1);

    write(IMU->mg, IMU->MAG_STATUS2, 1);
    read(IMU->mg, IMU->MAG_STATUS_OUT, 1);

    write(IMU->mg, IMU->MAGN_OUT_L, 1);
    read(IMU->mg, IMU->MAGN_OUT_L, 1);

    final = ((*(IMU->MAGN_OUT_H) << 8) | *(IMU->MAGN_OUT_L));

    write(IMU->mg, IMU->MAG_STATUS2, 1);
    read(IMU->mg, IMU->MAG_STATUS_OUT, 1);

    uT = final - IMU->magn_offsets[0];
    uT *= IMU->magn_multipliers[0];

    return uT;
}

double IMU_MAGN_Y()
{
    int16_t final;
    double uT;

    write(IMU->mg, IMU->MAG_STATUS2, 1);
    read(IMU->mg, IMU->MAG_STATUS_OUT, 1);

    *(IMU->MAGN_OUT_H) = 0x04;
    *(IMU->MAGN_OUT_L) = 0x03;

    write(IMU->mg, IMU->MAGN_OUT_H, 1);
    read(IMU->mg, IMU->MAGN_OUT_H, 1);

    write(IMU->mg, IMU->MAG_STATUS2, 1);
    read(IMU->mg, IMU->MAG_STATUS_OUT, 1);

    write(IMU->mg, IMU->MAGN_OUT_L, 1);
    read(IMU->mg, IMU->MAGN_OUT_L, 1);

    final = ((*(IMU->MAGN_OUT_H) << 8) | *(IMU->MAGN_OUT_L));

    write(IMU->mg, IMU->MAG_STATUS2, 1);
    read(IMU->mg, IMU->MAG_STATUS_OUT, 1);

    uT = final - IMU->magn_offsets[1];
    uT *= IMU->magn_multipliers[1];

    return uT;
}

double IMU_MAGN_Z()
{
    int16_t final;
    double uT;

    write(IMU->mg, IMU->MAG_STATUS2, 1);
    read(IMU->mg, IMU->MAG_STATUS_OUT, 1);

    *(IMU->MAGN_OUT_H) = 0x08;
    *(IMU->MAGN_OUT_L) = 0x07;

    write(IMU->mg, IMU->MAGN_OUT_H, 1);
    read(IMU->mg, IMU->MAGN_OUT_H, 1);

    write(IMU->mg, IMU->MAG_STATUS2, 1);
    read(IMU->mg, IMU->MAG_STATUS_OUT, 1);

    write(IMU->mg, IMU->MAGN_OUT_L, 1);
    read(IMU->mg, IMU->MAGN_OUT_L, 1);

    final = ((*(IMU->MAGN_OUT_H) << 8) | *(IMU->MAGN_OUT_L));

    write(IMU->mg, IMU->MAG_STATUS2, 1);
    read(IMU->mg, IMU->MAG_STATUS_OUT, 1);

    uT = final - IMU->magn_offsets[2];
    uT *= IMU->magn_multipliers[2] * -1;

    return uT;
}
