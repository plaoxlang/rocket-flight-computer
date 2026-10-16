#ifndef INC_SENSOR_FUSION_H_
#define INC_SENSOR_FUSION_H_

#include "mpu6050.h"
#define SEC_IN_MILISEC 1000.0f
#define RAD_TO_DEG (180.0f / 3.14159265f)
#define SENSOR_FUSION_ALPHA 0.97f

typedef struct
{
	float roll_deg;
	float pitch_deg;
} Attitude;

void sensorFusion_Init(const MPU6050_Data *imu, Attitude *attitude);

void sensorFusion_Update(const MPU6050_Data *imu,
		uint32_t dt, Attitude *attitude);

#endif
