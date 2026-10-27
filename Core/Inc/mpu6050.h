#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include "main.h"

#define MPU6050_ADDR (0x68 << 1)
#define MPU6050_WHO_AM_I 0x75
#define MPU6050_WHO_AM_I_VALUE 0x68
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_DATA_LENGTH 14
#define MPU6050_CALIB_READ 1000
#define MPU6050_CALIB_MOTION_THRESHOLD 3.0f
#define MPU6050_I2C_TIMEOUT_MS 50

#define MPU6050_ACCEL_SCALE 16384.0f
#define MPU6050_GYRO_SCALE 131.0f
#define MPU6050_TEMP_SCALE 340.0f
#define MPU6050_TEMP_OFFSET 36.53f

typedef struct
{
    float accel_x_g;
    float accel_y_g;
    float accel_z_g;

    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;

    float temperature_c;
} MPU6050_Data;

typedef struct
{
	float gx_bias;
	float gy_bias;
	float gz_bias;
} MPU6050_Biases;

HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *hi2c);

HAL_StatusTypeDef MPU6050_Read(MPU6050_Data *imu);

HAL_StatusTypeDef MPU6050_CalibrateGyro(void);

HAL_StatusTypeDef MPU6050_Check(void);

#endif
