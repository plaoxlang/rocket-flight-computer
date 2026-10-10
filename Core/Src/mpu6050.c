#include "mpu6050.h"
#include "i2c_recover.h"
#include <stdio.h>
#include <math.h>

static I2C_HandleTypeDef *mpu_i2c;
static float gx_bias = 0, gy_bias = 0, gz_bias = 0;

// wake the MPU6050 by clearing the sleep bit
HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *hi2c) {
	mpu_i2c = hi2c;

	HAL_StatusTypeDef statusMPU;

	uint8_t data = 0x00;
	statusMPU = HAL_I2C_Mem_Write(mpu_i2c, MPU6050_ADDR, MPU6050_PWR_MGMT_1,
						I2C_MEMADD_SIZE_8BIT, &data,
						1, HAL_MAX_DELAY);
					

	if(statusMPU != HAL_OK) {
		printf("MPU initialization failed, %d\r\n", statusMPU);
		I2C_BusClear(mpu_i2c);
	}

	return statusMPU;
}

// read one complete measurement frame from the MPU6050
HAL_StatusTypeDef MPU6050_ReadRaw(MPU6050_Data *imu) {
	HAL_StatusTypeDef status;
	uint8_t sensor_data[MPU6050_DATA_LENGTH];

	status = HAL_I2C_Mem_Read(mpu_i2c, MPU6050_ADDR, MPU6050_ACCEL_XOUT_H,
								I2C_MEMADD_SIZE_8BIT, sensor_data, MPU6050_DATA_LENGTH,
								HAL_MAX_DELAY);

	if(status == HAL_OK) {
		int16_t accel_x = (sensor_data[0] << 8) | sensor_data[1];
		int16_t accel_y = (sensor_data[2] << 8) | sensor_data[3];
		int16_t accel_z = (sensor_data[4] << 8) | sensor_data[5];

		int16_t gyro_x = (sensor_data[8] << 8) | sensor_data[9];
		int16_t gyro_y = (sensor_data[10] << 8) | sensor_data[11];
		int16_t gyro_z = (sensor_data[12] << 8) | sensor_data[13];

		int16_t temperature = (sensor_data[6] << 8) | sensor_data[7];

		imu->ax = accel_x / MPU6050_ACCEL_SCALE;
		imu->ay = accel_y / MPU6050_ACCEL_SCALE;
		imu->az = accel_z / MPU6050_ACCEL_SCALE;

		imu->gx = gyro_x / MPU6050_GYRO_SCALE;
		imu->gy = gyro_y / MPU6050_GYRO_SCALE;
		imu->gz = gyro_z / MPU6050_GYRO_SCALE;

		imu->temperature = temperature / MPU6050_TEMP_SCALE + MPU6050_TEMP_OFFSET;
	}

	return status;
}

// apply bias to the gyroscope values
HAL_StatusTypeDef MPU6050_Read(MPU6050_Data *imu) {
	HAL_StatusTypeDef status;

	status = MPU6050_ReadRaw(imu);

	if(status == HAL_OK) {
		imu->gx -= gx_bias;
		imu->gy -= gy_bias;
		imu->gz -= gz_bias;
	}

	return status;
}

HAL_StatusTypeDef MPU6050_CalibrateGyro(void) {
	MPU6050_Data imu;
	HAL_StatusTypeDef status;

	float valuesXSum = 0;
	float valuesYSum = 0;
	float valuesZSum = 0;
	int read_errors = 0;
	int motion_errors = 0;

	int i = 0;
	for(; i < MPU6050_CALIB_READ; i++) {
		status = MPU6050_ReadRaw(&imu);

		if(status != HAL_OK) {
			read_errors++;
			if(read_errors > 9) {
				return HAL_ERROR;
			}
			i--;
		} else if(fabsf(imu.gx) > 3 || fabsf(imu.gy) > 3 || fabsf(imu.gz) > 3) {
			motion_errors++;

			valuesXSum = 0;
			valuesYSum = 0;
			valuesZSum = 0;

			i = -1;

			if(motion_errors > 9) {
				return HAL_ERROR;
			}
		} else {
			motion_errors = 0;
			read_errors = 0;

			valuesXSum += imu.gx;
			valuesYSum += imu.gy;
			valuesZSum += imu.gz;
		}
	}

	gx_bias = valuesXSum / MPU6050_CALIB_READ;
	gy_bias = valuesYSum / MPU6050_CALIB_READ;
	gz_bias = valuesZSum / MPU6050_CALIB_READ;
	return HAL_OK;
}

HAL_StatusTypeDef MPU6050_Check(void) {
	uint8_t isConnected = 0;
	HAL_StatusTypeDef MPUIsON;

	MPUIsON = HAL_I2C_Mem_Read(mpu_i2c, MPU6050_ADDR, MPU6050_WHO_AM_I,
			I2C_MEMADD_SIZE_8BIT, &isConnected, 1,
			50);

	if(MPUIsON != HAL_OK) return MPUIsON;
	return (isConnected == 0x68) ? HAL_OK : HAL_ERROR;
}
