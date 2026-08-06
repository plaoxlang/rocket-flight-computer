#include "sensor_fusion.h"
#include <math.h>

void sensorFusion_Init(const MPU6050_Data *imu, Attitude *attitude) {
	attitude->roll = atan2(imu->ay, imu->az) * RAD_TO_DEG;
	attitude->pitch = atan2(imu->ax, imu->az) * RAD_TO_DEG;
}

void sensorFusion_Update(const MPU6050_Data *imu,
		float dt, Attitude *attitude)
{
	float gyroRoll = attitude->roll + imu->gx * dt;
	float gyroPitch = attitude->pitch + imu->gy * dt;

	float accelRoll = atan2(imu->ay, imu->az) * RAD_TO_DEG;
	float accelPitch = atan2(imu->ax, imu->az) * RAD_TO_DEG;

	attitude->roll = gyroRoll * ALPHA + accelRoll * (1 - ALPHA);
	attitude->pitch = gyroPitch * ALPHA + accelPitch * (1 - ALPHA);
}
