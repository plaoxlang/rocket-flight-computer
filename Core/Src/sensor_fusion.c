#include "sensor_fusion.h"
#include <math.h>

void sensorFusion_Init(void) {

}

void sensorFusion_Update(const MPU6050_Data *imu,
		float dt, Attitude *attitude)
{
	attitude->roll=atan2(imu->ax, imu->az) * RAD_TO_DEG;
	attitude->pitch=atan2(imu->ay, imu->az) * RAD_TO_DEG;
}
