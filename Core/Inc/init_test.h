#ifndef INIT_TEST_H_
#define INIT_TEST_H_

#define INIT_ITERATIONS 100

#include "main.h"
#include "mpu6050.h"

HAL_StatusTypeDef MPU6050_ReadRegister(I2C_HandleTypeDef *mpu_i2c, 
    uint8_t i2c_address, 
    uint8_t reg, 
    uint8_t *value
);

HAL_StatusTypeDef MPU6050_Reset(MPU6050 *imu);
HAL_StatusTypeDef checkInitialization(MPU6050 *imu, uint8_t reg, uint8_t expected_value);

#endif
