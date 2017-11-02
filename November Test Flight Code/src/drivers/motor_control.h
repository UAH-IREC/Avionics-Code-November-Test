/*
 * motor_control.h
 *
 * Created: 8/19/2017 12:23:21 AM
 *  Author: dc0069
 */ 


#ifndef MOTOR_CONTROL_H_
#define MOTOR_CONTROL_H_

#include <asf.h>
//#include <ASF/common/services/ioport/ioport.h>

void init_motor_control(void);
void motor_counter_clockwise(void);
void motor_clockwise(void);
void motor_speed(uint8_t speed_percent);
int16_t get_motor_percent(void);

#endif /* MOTOR_CONTROL_H_ */