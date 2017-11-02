/*
 * imu.c
 *
 * Created: 5/23/2017 5:35:00 PM
 *  Author: trb0023
 */ 

#include "drivers/imu.h"
#include "drivers/mpu9250.h"
#include "drivers/MahonyAHRS.h"

static double mapdouble(double x, double in_min, double in_max, double out_min, double out_max);
static void update_max_min_mag(void);
static void calculate_offset_mag(void);

static int16_t mag_x_max =-32768;
static int16_t mag_x_min = 32767;
static int16_t mag_y_max =-32768;
static int16_t mag_y_min = 32767;
static int16_t mag_z_max =-32768;
static int16_t mag_z_min = 32767;

// int16_t MAG_X_OFF = 123;
// int16_t MAG_Y_OFF = 123;
// int16_t MAG_Z_OFF = 123;

int16_t MAG_X_OFF = 136;
int16_t MAG_Y_OFF = -206;
int16_t MAG_Z_OFF = 3538;

int16_t ACC_X_OFF = -14;
int16_t ACC_Y_OFF = -8;
int16_t ACC_Z_OFF = -41;

int16_t GYR_X_OFF = 6;
int16_t GYR_Y_OFF = 0;
int16_t GYR_Z_OFF = 0;

mpu9250_t imu =
{
	.twi = &TWID,
	.int_pin = MPU9250_INT_PIN_NUM
};

void init_imu(void)
{
	init_mpu9250(imu, 100);
}

bool imu_is_data_ready(void)
{
	return (bool)is_data_rdy();
}

imu_data_t imu_update(uint16_t cycles)
{
	if(cycles % MPU_UPDATE_DIV == 0)
	{
		data = read_mpu9250(imu);
	}
	
	
	MahonyAHRSupdate(	mapdouble((double)data.gyro_x+(GYR_X_OFF), -32768.0, 32767.0, -2000, 2000),
						mapdouble((double)data.gyro_y+(GYR_Y_OFF), -32768.0, 32767.0, -2000.0, 2000.0),
						mapdouble((double)data.gyro_z+(GYR_Z_OFF), -32768.0, 32767.0, -2000.0, 2000.0),
						(double)data.acc_x+(ACC_X_OFF), (double)data.acc_y+(ACC_Y_OFF), (double)data.acc_z+(ACC_Z_OFF),
						(double)data.mag_y+(MAG_Y_OFF), (double)data.mag_x+(MAG_X_OFF), -((double)data.mag_z+(MAG_Z_OFF)));

	
	
	imu_data_t imudata;					
	imudata.yaw   = atan2(2.0 * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3);
	imudata.pitch = -asin(2.0 * (q1 * q3 - q0 * q2));
	imudata.roll  = atan2(2.0 * (q0 * q1 + q2 * q3), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3);
	imudata.pitch *= 180.0 / PI;
	imudata.yaw   *= 180.0 / PI;	// yaw = compass heading
	imudata.roll  *= 180.0 / PI;
	
	return imudata;
}

static double mapdouble(double x, double in_min, double in_max, double out_min, double out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


static void update_max_min_mag(void)
{
	if(data.mag_x > mag_x_max)
	{
		mag_x_max = data.mag_x;
	}
	if (data.mag_x < mag_x_min)
	{
		mag_x_min = data.mag_x;
	}
	if(data.mag_y > mag_y_max)
	{
		mag_y_max = data.mag_y;
	}
	if (data.mag_y < mag_y_min)
	{
		mag_y_min = data.mag_y;
	}
	if(data.mag_z > mag_z_max)
	{
		mag_z_max = data.mag_z;
	}
	if (data.mag_z < mag_z_min)
	{
		mag_z_min = data.mag_z;
	}
}

static void calculate_offset_mag(void)
{
	MAG_X_OFF = -((mag_x_max + mag_x_min)/2);
	MAG_Y_OFF = -((mag_y_max + mag_y_min)/2);
	MAG_Z_OFF = -((mag_z_max + mag_z_min)/2);
}

void imu_calibrate_mag(void)
{
	printf("\nCalibrating magnetometer! Swing the electronics around randomly!");

	delay_ms(2000);

	mag_x_max =-32768;
	mag_x_min = 32767;
	mag_y_max =-32768;
	mag_y_min = 32767;
	mag_z_max =-32768;
	mag_z_min = 32767;

	for (uint16_t i = 0; i < MAG_CALIBRATION_SAMPLES;)
	{
		
		if(is_data_rdy())
		{
			i++;

			data = read_mpu9250(imu);

			update_max_min_mag();

			if(i%30 == 0)
			{
				
				printf("\n%.1f%%\n	X %i %i\n	Y %i %i\n	Z %i %i", ((double)i*100.0/(double)MAG_CALIBRATION_SAMPLES), mag_x_min, mag_x_max, mag_y_min, mag_y_max, mag_z_min, mag_z_max);
			}

		}

		
	}

	calculate_offset_mag();

	printf("\nCalibration Complete!\nNew Offsets:\nX: %i  Y: %i  Z %i\n", MAG_X_OFF, MAG_Y_OFF, MAG_Z_OFF);

	imu_set_mag_calibration(imu_get_mag_calibration());

	delay_ms(5000);
}

imu_mag_cal_t imu_get_mag_calibration(void)
{
	imu_mag_cal_t cal;
	cal.x_off = MAG_X_OFF;
	cal.y_off = MAG_Y_OFF;
	cal.z_off = MAG_Z_OFF;

	return cal;
}

void imu_set_mag_calibration(imu_mag_cal_t calibration)
{
	MAG_X_OFF = calibration.x_off;
	MAG_Y_OFF = calibration.y_off;
	MAG_Z_OFF = calibration.z_off;
}