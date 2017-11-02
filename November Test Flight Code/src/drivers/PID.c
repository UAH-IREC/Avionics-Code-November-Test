/*
 * PID.c
 *
 * Created: 8/19/2017 10:40:46 PM
 *  Author: dc0069
 */ 

#include "drivers/PID.h"

//Position
#define Kp	0.5
#define Ki	0.2 //0.2
#define Kd	0.5 //0.4

double target;

#define NUM_KEPT_DATA_POINTS	3
double errors[NUM_KEPT_DATA_POINTS];
double tstep;

//Rates
#define Kp_rate		0.3
#define Ki_rate		1.5 //0.2
#define Kd_rate		0.0 //0.4

double rate_errors[NUM_KEPT_DATA_POINTS];

double deg_dist(double t1, double t2)
{
	if (t1 >= 0)
	{
		
	}
}

double d_dt(double x0, double x1, double x2)
{
	double smallest = min(min(x0, x1), x2);
	x0 -= smallest;
	x1 -= smallest;
	x2 -= smallest;
	return (3 * x0 - 4 * x1 + x2) / (2 * tstep);
}

void init_PID(double time_step, double target)
{
	uint8_t i;
	for (i = 0; i < NUM_KEPT_DATA_POINTS; i++)
	{
		errors[i] = 0.0;
		rate_errors[i] = 0.0;
	}
	tstep = time_step;
}

double array_abs_min(double* arr, uint16_t len)
{
	uint16_t i;
	double m = arr[0];
	for (i = 0; i < len; i++)
	{
		if (abs(arr[i]) < m)
		{
			m = abs(arr[i]);
		}
	}
	return m;
}

int8_t update_and_output(double error)
{
	//Shift back old data
	uint8_t i;
	for (i = NUM_KEPT_DATA_POINTS - 1; i > 0; i--)
	{
		errors[i] = errors[i - 1];
	}
	
	if (error > 160 && errors[1] < -160)
	{
		error = -180 - (180 - error);
	}
	else if (error < -160 && errors[1] > 160)
	{
		error = 180 + (180 - abs(error));
	}
	errors[0] = error;
	
	if (array_abs_min(errors, NUM_KEPT_DATA_POINTS) > 180)
	{
		for (i = 0; i < NUM_KEPT_DATA_POINTS; i++)
		{
			errors[i] -= 180 * abs(errors[i]) / errors[i];
		}
	}
	
	//Trapezoidal integration
	double int_error = 0;
	for (i = 0; i < NUM_KEPT_DATA_POINTS; i++)
	{
		int_error += (errors[i] - errors[i + 1]) * tstep;
	}
	
	//2-point backwards difference
	double d_error = (3 * errors[0] - 4 * errors[1] + errors[2]) / (2 * tstep);
	
	return (int8_t)(Kp * error + Ki * int_error + Kd * d_error);
}


int8_t angular_rate_pid(int16_t rate)
{
	//Shift back old data
	uint8_t i;
	for (i = NUM_KEPT_DATA_POINTS - 1; i > 0; i--)
	{
		rate_errors[i] = rate_errors[i - 1];
	}
		
	rate_errors[0] = rate;
		
	//Trapezoidal integration
	double int_error = 0;
	for (i = 0; i < NUM_KEPT_DATA_POINTS; i++)
	{
		int_error += (rate_errors[i] - rate_errors[i + 1]) * tstep;
	}
		
	//2-point backwards difference
	double d_error = (3 * rate_errors[0] - 4 * rate_errors[1] + rate_errors[2]) / (2 * tstep);
		
	return (int8_t)(Kp_rate * rate + Ki_rate * int_error + Kd_rate * d_error);
}