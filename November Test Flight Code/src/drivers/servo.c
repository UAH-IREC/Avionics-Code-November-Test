/*
 * servo.c
 *
 * Created: 3/30/2016 8:57:27 PM
 *  Author: whankins
 */ 

#include "drivers/servo.h"
#include <inttypes.h>
#include <asf.h>

void initializeServo(void)
//Start the PWM control for the servo, but keep it switched off
{
 	ioport_set_pin_dir(SERVO1_CTRL_PIN, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(SERVO2_CTRL_PIN, IOPORT_DIR_OUTPUT);
	//servoOff();
	
	//Set up clock
	sysclk_enable_peripheral_clock(&SERVO_TC);
	sysclk_enable_module(SYSCLK_SERVO_PORT, SYSCLK_HIRES);
	
	//PWM settings
	SERVO_TC.CTRLA = 0b00000110;
	SERVO_TC.CTRLB = 0b11110011;
	SERVO_TC.PER = 2499;// This is the whole PWM(50hz, 20ms). The servo only uses a max of 2 ms of it (norm 1.5ms)
	
	turnServo(1, 0);
	turnServo(2, 0);
	
}
	

	
void turnServo(uint8_t servoNum, int8_t servoAngle)
{
	if(servoNum == 1)	
	{
		SERVO1_TC_CC =(int16_t) (((int32_t)25380+(int32_t)14*(int32_t)(90+servoAngle))* (int32_t) SERVO_TC.PER/(int32_t)28800);
	}
	else if(servoNum == 2)
	{
		SERVO2_TC_CC =(int16_t) (((int32_t)25380+(int32_t)14*(int32_t)(90+servoAngle))* (int32_t) SERVO_TC.PER/(int32_t)28800);
	}
}