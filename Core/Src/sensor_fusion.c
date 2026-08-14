#include "sensor_fusion.h"
#include <math.h>

void sensorFusion_Init(const MPU6050_Data *imu, Attitude *attitude) {
	attitude->roll = atan2(imu->ay, imu->az) * RAD_TO_DEG;
	attitude->pitch = atan2(imu->ax, imu->az) * RAD_TO_DEG;
}

void sensorFusion_Update(const MPU6050_Data *imu,
		uint32_t dt, Attitude *attitude)
{
	float dt_sec = (float)dt / SEC_IN_MILISEC;
	float gyroRoll = attitude->roll + imu->gx * dt_sec;
	float gyroPitch = attitude->pitch + imu->gy * dt_sec;

	float accelRoll = atan2(imu->ay, imu->az) * RAD_TO_DEG;
	float accelPitch = atan2(-imu->ax, sqrt(imu->ay * imu->ay + imu->az * imu->az)) * RAD_TO_DEG;

	attitude->roll = gyroRoll * ALPHA + accelRoll * (1 - ALPHA);
	attitude->pitch = gyroPitch * ALPHA + accelPitch * (1 - ALPHA);
}
