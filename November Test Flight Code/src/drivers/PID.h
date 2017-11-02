/*
 * PID.h
 *
 * Created: 8/19/2017 11:04:38 PM
 *  Author: dc0069
 */ 


#ifndef PID_H_
#define PID_H_

#include <asf.h>

void init_PID(double time_step, double target);
int8_t update_and_output(double error);
int8_t angular_rate_pid(int16_t rate);

#endif /* PID_H_ */