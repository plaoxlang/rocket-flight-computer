#include "init_test.h"
// Checking the correct configuration of the MPU6050 registers

/**
 * @brief Read a specific register from the MPU
 * @param mpu_i2c : I2C handler
 * @param i2c_address : Device(MPU6050) identificator
 * @param reg : Register number
 * @param value : The value stored in the register
 * @retval HAL Status of the operation
 */
HAL_StatusTypeDef MPU6050_ReadRegister(I2C_HandleTypeDef *mpu_i2c, 
    uint8_t i2c_address, 
    uint8_t reg, 
    uint8_t *value
) {
    return HAL_I2C_Mem_Read(mpu_i2c, i2c_address << 1,
                     reg, I2C_MEMADD_SIZE_8BIT,
                     value, 1, MPU6050_I2C_TIMEOUT_MS);
}

/** 
 * @brief Reset the MPU6050
 * @param mpu_i2c : I2C handler for the MPU
 * @param i2c_address : MPU6050 WHO_AM_I value
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

/**
 @brief Compares the expected value and the read one from the MPU register
 @param imu MPU6050 imu struct
 @param reg : register to read
 @param expected_value : (of the register)
 @retval HAL Status 
 */
HAL_StatusTypeDef checkInitialization(MPU6050 *imu, uint8_t reg, uint8_t expected_value) {
    HAL_StatusTypeDef status;
    uint8_t read_value = 0x00;
    uint16_t successful_reads = 0;
    uint16_t failed_reads = 0;
    uint16_t communication_errors = 0;
    I2C_HandleTypeDef *mpu_i2c = imu->i2c;

    int i = 0;
    while(i < INIT_ITERATIONS) {
        status = MPU6050_Reset(imu);
        if(status != HAL_OK) {
            communication_errors++;
        }

        status = MPU6050_ReadRegister(mpu_i2c, imu->info.i2c_address, reg, &read_value);
        if(status != HAL_OK) {
            communication_errors++;
        }

        if(expected_value != read_value) {
            failed_reads++;
        } else {
            successful_reads++;
        }
        i++;
    }

    return HAL_OK;
}
