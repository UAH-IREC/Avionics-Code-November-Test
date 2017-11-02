/*
 * motor_control.c
 *
 * Created: 8/19/2017 12:23:07 AM
 *  Author: dc0069
 */ 



#include "motor_control.h"

#define MOTOR_CCW_PIN	IOPORT_CREATE_PIN(PORTF, 0)
#define MOTOR_CW_PIN	IOPORT_CREATE_PIN(PORTF, 1)

#define MOTOR_CCW_TC	TCF0_CCA
#define MOTOR_CW_TC		TCF0_CCB
#define MOTOR_TC		TCF0

#define MOTOR_DIR_CCW	0
#define MOTOR_DIR_CW	1

uint16_t motor_speed_percent;

uint8_t dir;

void init_motor_control(void)
{
	ioport_set_pin_dir(MOTOR_CW_PIN, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(MOTOR_CCW_PIN, IOPORT_DIR_OUTPUT);
	
	ioport_set_pin_low(MOTOR_CCW_PIN);
	ioport_set_pin_low(MOTOR_CW_PIN);
	
	dir = MOTOR_DIR_CCW;
	motor_speed_percent = 0;
	
	sysclk_enable_peripheral_clock(&TCF0);
	sysclk_enable_module(SYSCLK_PORT_F, SYSCLK_HIRES);
	
	MOTOR_TC.CTRLA = 0b00000101;
	MOTOR_TC.CTRLB = 0b11110011;
	MOTOR_TC.PER = 125; //100 Hz
}

void motor_counter_clockwise(void)
{
	MOTOR_CCW_TC = motor_speed_percent;
	MOTOR_CW_TC = 0;
	dir = MOTOR_DIR_CCW;
}

void motor_clockwise(void)
{
	MOTOR_CW_TC = motor_speed_percent;
	MOTOR_CCW_TC = 0;
	dir = MOTOR_DIR_CW;
}

void motor_speed(uint8_t speed_percent)
{
	if (dir == MOTOR_DIR_CW)
	{
		MOTOR_CW_TC = speed_percent * MOTOR_TC.PER / 100;
	}
	else
	{
		MOTOR_CCW_TC = speed_percent * MOTOR_TC.PER / 100;
	}
	motor_speed_percent = speed_percent * MOTOR_TC.PER / 100;
}

int16_t get_motor_percent(void)
{
	if (dir == MOTOR_DIR_CCW)
		return motor_speed_percent;
	else
		return -1 * (int16_t)motor_speed_percent;
}