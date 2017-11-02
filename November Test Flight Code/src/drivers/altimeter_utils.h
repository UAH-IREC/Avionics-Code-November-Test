/*
 * altimeter_utils.h
 *
 * Created: 5/26/2017 6:00:26 PM
 *  Author: trb0023
 */ 

#include <asf.h>
#include <math.h>

#ifndef ALTIMETER_UTILS_H_
#define ALTIMETER_UTILS_H_

#define NUM_PREVIOUS_SAMPLES 20
#define SMOOTHING_CONSTANT 0.7

void update_altimeter_utils(double altitude);
double alt_util_get_smoothed_altitude(void);
double alt_util_get_average_altitude(uint8_t numSamples);
double alt_util_get_instability(void);				//The lower the stability is, the more stable it is. It sounds weird but just go with it.
double alt_util_get_velocity(void);				//The lower the stability is, the more stable it is. It sounds weird but just go with it.




#endif /* ALTIMETER_UTILS_H_ */