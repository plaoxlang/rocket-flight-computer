#include "sensor_fusion.h"
#include <math.h>

void sensorFusion_Init(const MPU6050_Data *imu, Attitude *attitude) {
	attitude->roll_deg = atan2(imu->accel_y_g, imu->accel_z_g) * RAD_TO_DEG;
	attitude->pitch_deg = atan2(imu->accel_x_g, imu->accel_z_g) * RAD_TO_DEG;
}

void sensorFusion_Update(const MPU6050_Data *imu,
		uint32_t dt, Attitude *attitude)
{
	float dt_sec = (float)dt / SEC_IN_MILISEC;
	float gyroRoll = attitude->roll_deg + imu->gyro_x_dps * dt_sec;
	float gyroPitch = attitude->pitch_deg + imu->gyro_y_dps * dt_sec;

	float accelRoll = atan2(imu->accel_y_g, imu->accel_z_g) * RAD_TO_DEG;
	float accelPitch = atan2(-imu->accel_x_g, sqrt(imu->accel_y_g * imu->accel_y_g + imu->accel_z_g * imu->accel_z_g)) * RAD_TO_DEG;

	attitude->roll_deg = gyroRoll * SENSOR_FUSION_ALPHA + accelRoll * (1 - SENSOR_FUSION_ALPHA);
	attitude->pitch_deg = gyroPitch * SENSOR_FUSION_ALPHA + accelPitch * (1 - SENSOR_FUSION_ALPHA);
}
