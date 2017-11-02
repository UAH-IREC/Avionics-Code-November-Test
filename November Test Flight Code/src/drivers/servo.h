/*
 * servo.h
 *
 * Created: 3/30/2016 8:57:52 PM
 *  Author: whankins
 */ 


#ifndef SERVO_H_
#define SERVO_H_

#define SERVO_PORT							PORTF
#define SERVO_TC							TCF0
#define SERVO_TC_CC							TCF0_CCA
#define SYSCLK_SERVO_PORT					SYSCLK_PORT_F

#define SERVO1_CTRL_PIN						IOPORT_CREATE_PIN(PORTF, 0)
#define SERVO2_CTRL_PIN						IOPORT_CREATE_PIN(PORTF, 1)
#define SERVO2_TC_CC						TCF0_CCA
#define SERVO1_TC_CC						TCF0_CCB

#define SERVO_YAW 1
#define SERVO_PITCH 2

#include <asf.h>

#define SERVO_CLOSED_POSITION			-20
#define SERVO_OPEN_POSITION				90

void initializeServo(void);

void turnServo(uint8_t servoNum, int8_t servoAngle);

int8_t servo_angle;

#endif /* SERVO_H_ */