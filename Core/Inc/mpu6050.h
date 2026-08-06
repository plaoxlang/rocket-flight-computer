#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include "main.h"

#define MPU6050_ADDR (0x68 << 1)
#define MPU6050_WHO_AM_I 0x75
#define MPU6050_WHO_AM_I_VALUE 0x68
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_DATA_LENGTH 14
#define MPU6050_CALIB_READ 1000;

#define MPU6050_ACCEL_SCALE 16384.0f
#define MPU6050_GYRO_SCALE 131.0f
#define MPU6050_TEMP_SCALE 340.0f
#define MPU6050_TEMP_OFFSET 36.53f

typedef struct
{
    float ax;
    float ay;
    float az;

    float gx;
    float gy;
    float gz;

    float temperature;

} MPU6050_Data;

HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *hi2c);

HAL_StatusTypeDef MPU6050_Read(MPU6050_Data *imu);

uint8_t MPU6050_IsConnected(void);

#endif
