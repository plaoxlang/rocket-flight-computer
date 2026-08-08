#include "mpu6050.h"
#include "math.h"

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

	return statusMPU;
}

// read one complete measurement frame from the MPU6050
HAL_StatusTypeDef MPU6050_Read(MPU6050_Data *imu) {
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

		imu->gx = gyro_x / MPU6050_GYRO_SCALE - gx_bias;
		imu->gy = gyro_y / MPU6050_GYRO_SCALE - gy_bias;
		imu->gz = gyro_z / MPU6050_GYRO_SCALE - gz_bias;

		imu->temperature = temperature / MPU6050_TEMP_SCALE + MPU6050_TEMP_OFFSET;
	}

	return status;
}

HAL_StatusTypeDef MPU6050_CalibrateGyro(void) {
	MPU6050_Data imu;
	HAL_StatusTypeDef status;

	float valuesXSum = 0;
	float valuesYSum = 0;
	float valuesZSum = 0;
	int i = 0;
	int error_counter = 0;

	for(; i < MPU6050_CALIB_READ; i++) {
		status = MPU6050_Read(&imu);

		if(error_counter > 9) {
			return HAL_ERROR;
		}

		if(status != HAL_OK) {
			i--;
			error_counter++;
		} else if(fabsf(imu.gx) > 3 || fabsf(imu.gy) > 3 || fabsf(imu.gz) > 3) {
			valuesXSum = 0;
			valuesYSum = 0;
			valuesZSum = 0;
			i = -1;
			error_counter++;
		} else {
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

uint8_t MPU6050_IsConnected(void) {
	uint8_t isConnected;
	HAL_StatusTypeDef MPUIsON;

	MPUIsON = HAL_I2C_Mem_Read(mpu_i2c, MPU6050_ADDR, MPU6050_WHO_AM_I,
			I2C_MEMADD_SIZE_8BIT, &isConnected, 1,
			HAL_MAX_DELAY);

	if(MPUIsON != HAL_OK) {
		return 0;
	}
	return (isConnected == MPU6050_WHO_AM_I_VALUE);
}
