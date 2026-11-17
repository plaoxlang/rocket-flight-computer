#include "mpu6050.h"
#include <math.h>

// wake the MPU6050 by clearing the sleep bit
HAL_StatusTypeDef MPU6050_Init(MPU6050 *imu) {
	I2C_HandleTypeDef *mpu_i2c = imu->i2c;
	uint8_t mpu_address = imu->info.i2c_address;

	HAL_StatusTypeDef statusMPU;

	uint8_t data = 0x00;
	statusMPU = HAL_I2C_Mem_Write(mpu_i2c, (uint16_t)(mpu_address << 1), MPU6050_PWR_MGMT_1,
						I2C_MEMADD_SIZE_8BIT, &data,
						1, MPU6050_I2C_TIMEOUT_MS);

	return statusMPU;
}

static int16_t combineBytes(uint8_t high, uint8_t low) {
	return (int16_t)((high << 8) | low);
}

// read one complete measurement frame from the MPU6050
HAL_StatusTypeDef MPU6050_ReadRaw(MPU6050 *imu) {
	//MPU6050_Data *imu
	HAL_StatusTypeDef status;
	uint8_t sensor_data[MPU6050_DATA_LENGTH];

	I2C_HandleTypeDef *mpu_i2c = imu->i2c;
	uint8_t mpu_address = imu->info.i2c_address;

	status = HAL_I2C_Mem_Read(mpu_i2c, (uint16_t)(mpu_address << 1), MPU6050_ACCEL_XOUT_H,
								I2C_MEMADD_SIZE_8BIT, sensor_data, MPU6050_DATA_LENGTH,
								MPU6050_I2C_TIMEOUT_MS);

	if(status == HAL_OK) {
		int16_t accel_x = combineBytes(sensor_data[0], sensor_data[1]);
		int16_t accel_y = combineBytes(sensor_data[2], sensor_data[3]);
		int16_t accel_z = combineBytes(sensor_data[4], sensor_data[5]);

		int16_t gyro_x = combineBytes(sensor_data[8], sensor_data[9]);
		int16_t gyro_y = combineBytes(sensor_data[10], sensor_data[11]);
		int16_t gyro_z = combineBytes(sensor_data[12], sensor_data[13]);

		int16_t temperature = combineBytes(sensor_data[6], sensor_data[7]);

		imu->data.accel_x_g = accel_x / MPU6050_ACCEL_SCALE;
		imu->data.accel_y_g = accel_y / MPU6050_ACCEL_SCALE;
		imu->data.accel_z_g = accel_z / MPU6050_ACCEL_SCALE;

		imu->data.gyro_x_dps = gyro_x / MPU6050_GYRO_SCALE;
		imu->data.gyro_y_dps = gyro_y / MPU6050_GYRO_SCALE;
		imu->data.gyro_z_dps = gyro_z / MPU6050_GYRO_SCALE;

		imu->data.temperature_c = temperature / MPU6050_TEMP_SCALE + MPU6050_TEMP_OFFSET;
	}

	return status;
}

// apply bias to the gyroscope values
HAL_StatusTypeDef MPU6050_Read(MPU6050 *imu) {
	HAL_StatusTypeDef status;

	status = MPU6050_ReadRaw(imu);

	if(status == HAL_OK) {
		imu->data.gyro_x_dps -= imu->biases.gx_bias;
		imu->data.gyro_y_dps -= imu->biases.gy_bias;
		imu->data.gyro_z_dps -= imu->biases.gz_bias;
	}

	return status;
}

static HAL_StatusTypeDef MPU6050_IsMoving(const MPU6050 *imu) {
	uint8_t isMoving = 0;
	isMoving = fabsf(imu->data.gyro_x_dps) > MPU6050_CALIB_MOTION_THRESHOLD ||
			fabsf(imu->data.gyro_y_dps) > MPU6050_CALIB_MOTION_THRESHOLD ||
			fabsf(imu->data.gyro_z_dps) > MPU6050_CALIB_MOTION_THRESHOLD;

	return isMoving ? HAL_ERROR : HAL_OK;
}

HAL_StatusTypeDef MPU6050_CalibrateGyro(MPU6050 *imu) {
	HAL_StatusTypeDef status;

	float valuesXSum = 0;
	float valuesYSum = 0;
	float valuesZSum = 0;
	int read_errors = 0;
	int motion_errors = 0;

	int valid_samples = 0;
	while(valid_samples < MPU6050_CALIB_READ) {
		status = MPU6050_Read(imu);

		if(status != HAL_OK) {
			if(++read_errors > 9)
				return HAL_ERROR;
		}

		if(MPU6050_IsMoving(imu)) {
			if(++motion_errors > 9)
				return HAL_ERROR;

			valuesXSum = 0;
			valuesYSum = 0;
			valuesZSum = 0;
			valid_samples = 0;
		}

		motion_errors = 0;
		read_errors = 0;

		valuesXSum += imu->data.gyro_x_dps;
		valuesYSum += imu->data.gyro_y_dps;
		valuesZSum += imu->data.gyro_z_dps;

		valid_samples++;
	}

	imu->biases.gx_bias = valuesXSum / MPU6050_CALIB_READ;
	imu->biases.gy_bias = valuesYSum / MPU6050_CALIB_READ;
	imu->biases.gz_bias = valuesZSum / MPU6050_CALIB_READ;
	return HAL_OK;
}

HAL_StatusTypeDef MPU6050_ReadWhoAmI(MPU6050 *imu) {
	I2C_HandleTypeDef *mpu_i2c = imu->i2c;
	uint8_t mpu_address = imu->info.i2c_address;

	uint8_t isConnected = 0;
	HAL_StatusTypeDef MPUIsON;

	MPUIsON = HAL_I2C_Mem_Read(mpu_i2c, (int16_t)(mpu_address << 1), MPU6050_WHO_AM_I,
			I2C_MEMADD_SIZE_8BIT, &isConnected, 1,
			MPU6050_I2C_TIMEOUT_MS);

	if(MPUIsON != HAL_OK) return MPUIsON;
	return (isConnected == imu->info.i2c_address) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef MPU6050_Check(MPU6050 *imu) {
	I2C_HandleTypeDef *mpu_i2c = imu->i2c;
	uint8_t mpu_address = imu->info.i2c_address;

    return HAL_I2C_IsDeviceReady(mpu_i2c, mpu_address, 3, 50);
}
