#include "config_test.h"
#include <stdio.h>

#define REGISTER_COUNT sizeof(registers) / sizeof(registers[0])

static const Register_Check registers[] = {
    {MPU6050_WHO_AM_I, EXPECTED_WHO_AM_I, MPU6050_WHO_AM_I_MASK},
    {MPU6050_PWR_MGMT_1, EXPECTED_PWR_MGMT_1, MPU6050_PWR_MGMT_1_MASK},
    {MPU6050_GYRO_CONFIG, EXPECTED_GYRO_CONFIG, MPU6050_GYRO_CONFIG_MASK},
    {MPU6050_ACCEL_CONFIG, EXPECTED_ACCEL_CONFIG, MPU6050_ACCEL_CONFIG_MASK}
};

/** 
 * @brief Reset the MPU6050
 * @param imu : MPU6050 imu struct
 * @retval HAL Status, whether the reset was completed without troubles
 */
HAL_StatusTypeDef MPU6050_Reset(MPU6050 *imu) {
    I2C_HandleTypeDef *mpu_i2c = imu->i2c;
    uint8_t i2c_address = imu->info.i2c_address;

    HAL_StatusTypeDef status;
    uint8_t reset = 0x80;
    uint8_t reset_result = 0x00;

    status = HAL_I2C_Mem_Write(mpu_i2c, i2c_address << 1,
                     MPU6050_PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT,
                     &reset, 1, MPU6050_I2C_TIMEOUT_MS);

    if(status != HAL_OK) {
        return status;
    }

    HAL_Delay(100);

    status = HAL_I2C_Mem_Read(mpu_i2c, i2c_address << 1,
                         MPU6050_PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT,
                        &reset_result, 1, MPU6050_I2C_TIMEOUT_MS);

    if(status != HAL_OK) { 
        return status;
    }
    if(!(reset_result & (1 << 6))) {
        return HAL_ERROR;
    }

    status = MPU6050_Init(imu);
    if(status != HAL_OK) {
        return status;
    }

    return HAL_OK;     
}

void MPU6050_RunInitTest(MPU6050 *imu, MPU6050_TestResults *test_results) {
    HAL_StatusTypeDef status;
    uint8_t read_value;

    int i = 0;
    int j;
    while(i < INIT_ITERATIONS) {
        status = MPU6050_Reset(imu);
        if(status != HAL_OK) {
            test_results->reset_errors++;
            test_results->failed_initializations++;
            continue;
        }
    
        j = 0;
        for(; j < REGISTER_COUNT; j++) {
            status = HAL_I2C_Mem_Read(imu->i2c, imu->info.i2c_address << 1,
                        registers[j].reg, I2C_MEMADD_SIZE_8BIT,
                        &read_value, 1, MPU6050_I2C_TIMEOUT_MS);

            if(status != HAL_OK) {
                test_results->communication_errors++;
                continue;
            }

            if((read_value & registers[j].mask) != 
                        (registers[j].expected_value & registers[j].mask)) {
                test_results->configuration_errors++;
                continue;
            }

            test_results->successful_reads++;
        }
        i++;
    }
}

void MPU6050_CheckInitialization(MPU6050 *imu) {
    MPU6050_TestResults test_results = {0};
    MPU6050_RunInitTest(imu, &test_results);

    printf("\r\n----------- IMU INITIALIZATION RESULT -----------\r\n");
    printf("Total iterations: %d\r\n", INIT_ITERATIONS);
    printf("Successful initializations: %lu\r\n",
        (unsigned long)test_results.successful_reads);
    printf("Failed initializations: %lu\r\n",
        (unsigned long)test_results.reset_errors);
    printf("Communication errors: %lu\r\n",
        (unsigned long)test_results.communication_errors);
    printf("Configuration errors: %lu\r\n",
        (unsigned long)test_results.configuration_errors);
}
