/*
 * altimeter_utils.c
 *
 * Created: 5/26/2017 6:00:09 PM
 *  Author: trb0023
 */ 

#include "drivers/altimeter_utils.h"

static void add_sample(double sample);
static void calc_instability(void);
static void calc_smoothed_altitude(void);
static double calc_average_altitude(uint8_t numSamples);
static void calc_velocity(void);

double previousSamples[NUM_PREVIOUS_SAMPLES];

double smoothed_altitude = 0;
double instability = 0;
double velocity = 0;

void update_altimeter_utils(double altitude)
{
	add_sample(altitude);
	calc_smoothed_altitude();
	//calc_instability();
	calc_velocity();
}


static void add_sample(double sample)
{
	for (uint16_t i = 0; i < NUM_PREVIOUS_SAMPLES - 1; i++)
	{
		previousSamples[i] = previousSamples[i+1];
	}

	previousSamples[NUM_PREVIOUS_SAMPLES - 1] = sample;
}

static void calc_smoothed_altitude(void)
{
	smoothed_altitude = (smoothed_altitude * (1.0 - SMOOTHING_CONSTANT)) + (previousSamples[NUM_PREVIOUS_SAMPLES-1]);
}

static double calc_average_altitude(uint8_t numSamples)
{
	numSamples = min(NUM_PREVIOUS_SAMPLES, numSamples);
	double sum = 0;
	for (uint16_t i = 0; i < numSamples; i++)
	{
		sum += previousSamples[i];
	}
	return sum/(double)numSamples;
}

static void calc_instability(void)
{
	double sum = 0;
	for (uint16_t i = 0; i < NUM_PREVIOUS_SAMPLES - 1; i++)
	{
		sum += fabs(previousSamples[i] - previousSamples[i+1]);
	}
	instability = sum/(double)(NUM_PREVIOUS_SAMPLES-1.0);
}

static void calc_velocity(void)
{
	double sum = 0;
	for (uint16_t i = 0; i < NUM_PREVIOUS_SAMPLES - 1; i++)
	{
		sum +=previousSamples[i+1] -previousSamples[i];
	}
	velocity = (sum/(double)(NUM_PREVIOUS_SAMPLES-1.0))/0.1;
}

double alt_util_get_smoothed_altitude(void)
{
	return smoothed_altitude;
}

double alt_util_get_average_altitude(uint8_t numSamples)
{
	return calc_average_altitude(numSamples);
}

double alt_util_get_instability(void)
{
	return instability;
}

double alt_util_get_velocity()
{
	return velocity;
}