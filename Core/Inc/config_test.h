#ifndef CONFIG_TEST_H_
#define CONFIG_TEST_H_

#define INIT_ITERATIONS 100
#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_ACCEL_CONFIG 0x1C

#define MPU6050_WHO_AM_I_MASK 0XFF
#define MPU6050_PWR_MGMT_1_MASK 0x40
#define MPU6050_GYRO_CONFIG_MASK 0x18
#define MPU6050_ACCEL_CONFIG_MASK 0x18

#define EXPECTED_WHO_AM_I 0x68
#define EXPECTED_PWR_MGMT_1 0x00
#define EXPECTED_GYRO_CONFIG 0x00
#define EXPECTED_ACCEL_CONFIG 0x00

#include "main.h"
#include "mpu6050.h"

typedef struct {
    uint16_t successful_reads;

    uint16_t communication_errors;
    uint16_t configuration_errors;
    uint16_t reset_errors;
    uint16_t failed_initializations;
} MPU6050_TestResults;

typedef struct {
    uint8_t reg;
    uint8_t expected_value;
    uint8_t mask;
} Register_Check;

HAL_StatusTypeDef MPU6050_Reset(MPU6050 *imu);
void MPU6050_RunInitTest(MPU6050 *imu, MPU6050_TestResults *test_results);
void MPU6050_CheckInitialization(MPU6050 *imu);

#endif
