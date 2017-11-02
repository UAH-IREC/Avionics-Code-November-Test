/*
 * Altitude.c
 *
 * Created: 2/16/2016 9:15:01 PM
 *  Author: ashah, epradhan
 */ 

#include <math.h>
#include <stdio.h>
#include <inttypes.h>
int32_t Measure_altitude (int32_t pressure_ground, int32_t pressure_above)
{
	//Exact height of parking garage is 27.5 ft, measured from top of the handrail at the corner closest to the CU to the top of the sidewalk leading to the CU (specifically, the edge of the sidewalk away from CV)
	//TODO: This function should be calibrated against that value (or a higher and/or more precise one)
	
	 int32_t height = (int32_t)(100.0/(double)0.0000225577)*(double)(1.0 - pow(((double)pressure_above) / ((double)pressure_ground), (double)(1.0 / 5.25588)));
	 return height;
}