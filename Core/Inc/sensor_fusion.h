#ifndef INC_SENSOR_FUSION_H_
#define INC_SENSOR_FUSION_H_

#include "mpu6050.h"
#define SEC_IN_MILISEC 1000.0f
#define RAD_TO_DEG (180.0f / 3.14159265f)

typedef struct
{
	float roll;
	float pitch;
} Attitude;

void sensorFusion_Init(void);

void sensorFusion_Update(const MPU6050_Data *imu,
		float dt, Attitude *attitude);

#endif
