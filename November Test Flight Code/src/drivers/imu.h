/*
 * imu.h
 *
 * Created: 5/23/2017 5:35:14 PM
 *  Author: trb0023
 */ 

#include <math.h>
#include <asf.h>

#ifndef IMU_H_
#define IMU_H_

#include "drivers/mpu9250.h"
#define MPU_UPDATE_DIV 3

#define MAG_CALIBRATION_SAMPLES 7500

typedef struct  
{
	double pitch;
	double roll;
	double yaw;
}imu_data_t;

typedef struct
{
	int16_t x_off;
	int16_t y_off;
	int16_t z_off;
}imu_mag_cal_t;

mpu9250_data_t data;

void init_imu(void);

bool imu_is_data_ready(void);

imu_data_t imu_update(uint16_t cycles);

void imu_calibrate_mag(void);

imu_mag_cal_t imu_get_mag_calibration(void);
void imu_set_mag_calibration(imu_mag_cal_t calibration);

#endif /* IMU_H_ */