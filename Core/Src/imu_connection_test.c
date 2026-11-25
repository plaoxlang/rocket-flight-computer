#include "imu_connection_test.h"
#include <stdio.h>

#define TEST_ITERATIONS 100

/**
 * @brief checks if the connected device is really an MPU
 * @param imu : MPU6050 measurements
 * @retval None
 */
void test_IMU_Identification(MPU6050 *imu) {
	uint32_t successful_reads = 0;
	uint32_t failed_reads = 0;

	for(int i = 0; i < TEST_ITERATIONS; i++) {

		if(MPU6050_ReadWhoAmI(imu) == HAL_OK) {
			successful_reads++;
		} else {
			failed_reads++;
		}

		HAL_Delay(10);
	}

	printf("\r\n-----IMU IDENTIFICATION-----\r\n");

	printf("Device: %s\r\n", imu->info.instance);

	printf("Total attempts: %d\r\n", TEST_ITERATIONS);
	printf("Successful: %lu\r\n", successful_reads);
	printf("Failed: %lu\r\n", failed_reads);

	if(failed_reads == 0)
		printf("TEST PASSED");
	else
		printf("TEST FAILED");

	HAL_Delay(10000);
}


