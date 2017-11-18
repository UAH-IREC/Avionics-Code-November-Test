/*
 * MS5607.h
 */
 //Thanks for the code Daniel!

#ifndef MS5607_H_
#define MS5607_H_

#define PRESSURE_SENSOR_SPI SPIC

#include <asf.h>
#include "SPI.h"

#define MS_SENS_STATE_ASKING_PRESS	1
#define MS_SENS_STATE_ASKING_TEMP	2
#define MS_SENS_STATE_READY			3
#define MS_SENS_LENGTH_VALUE_READ	10 //ms

typedef struct MS5607
{
	ioport_pin_t select_pin;
	uint16_t SENSt1; //C1
	uint16_t OFFt1; //C2
	uint16_t TCS; //You can guess
	uint16_t TCO;
	uint16_t Tref;
	uint16_t TEMPSENS;
	uint8_t STATE;
} MS5607_t;

void calibratePressureSensor(MS5607_t* sensor);
void readMS5607(MS5607_t* sensor, int32_t* pressure_dest, int32_t* temperature_dest);


void askForPressureMS5607(MS5607_t* sensor);
void readRawPressureMS5607(MS5607_t* sensor);
void askForTemperatureMS5607(MS5607_t* sensor);
void readRawTemperatureMS5607(MS5607_t* sensor);
void calculateValuesMS5607(MS5607_t* sensor, int32_t* pressure_dest, int32_t* temperature_dest);

#endif /* MS5607_H_ */