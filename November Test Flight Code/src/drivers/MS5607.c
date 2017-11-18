/*
 * MS5607.c
 *
 * Created: 9/25/2015 8:25:16 PM
 *  Author: dcorey
 */


 /*
 *	MS5611 info
 *
 *	Five commands: 1) Reset, 2) Read 128 bit PROM, 3) D1 conversion, 4) D2 conversion, 5) Read ADC result (24 bit temp/pressure)
 *	6 coefficients in PROM (all uint16_t)
 *		C1. Pressure sensitivity (SENSt1)
 *		C2. Pressure offset (OFFt1)
 *		C3. Temperature coefficient of pressure sensitivity (TCS)
 *		C4. Temperature coefficient of pressure offset (TCO)
 *		C5. Reference temperature (Tref)
 *		C6. Temperature coefficient of the temperature (TEMPSENS)
 *	D1 - pressure (digital)
 *	D2 - temperature (digital)
 *
 *	int32_t dt = D2 - C5 * 2^8
 *	int32_t TEMP = 2000 + dT * C6 / 2^23
 *
 *	int64_t OFF = C2 * 2^16 + (C4 * dT) / 2^7
 *	int64_t SENS = C1 * 2^15 + (C3 * dT) / 2^8
 *	int32_t PRESSURE = (D1 * SENS / 2^21 - OFF) / 2^15
 *	P (in millibars) = PRESSURE / 100.0
 *	PRESSURE is in Pascals.
 *
 *	If TEMP is less than 2000 (20 degrees Celsius), then
 *		uint32_t T2 = dT^2 / 2^31
 *		uint32_t OFF2 = 5 * (TEMP - 2000)^2 / 2
 *		uint32_t SENS2 = 5 * (TEMP - 2000)^2 / 4
 *		TEMP -= T2
 *		OFF -= OFF2
 *		SENS -= SENS2
 *
 */

 //Thanks for the code Daniel!

#include "MS5607.h"
#include <inttypes.h>
#include <asf.h>

int32_t rawPressure = 0;
int32_t rawTemp = 0;

void pressureSensorReset(MS5607_t* sensor);
uint16_t read16(SPI_t* targetspi);
uint32_t read24(SPI_t* targetspi);



uint16_t read16(SPI_t* targetspi)
{
	//Read in 16 bits, MSB first
	uint16_t ret = ((uint16_t)spiread(targetspi)) << 8;
	ret |= (uint16_t)spiread(targetspi);
	return ret;
}

uint32_t read24(SPI_t* targetspi)
{
	uint32_t ret = ((uint32_t)spiread(targetspi)) << 16;
	ret |= ((uint32_t)spiread(targetspi)) << 8;
	ret |= (uint32_t)spiread(targetspi);
	return ret;
}



void pressureSensorReset(MS5607_t* sensor)
{
	spiselect(sensor->select_pin);
	spiwrite(&PRESSURE_SENSOR_SPI, 0b00011110);
	delay_ms(5);
	spideselect(sensor->select_pin);
	delay_ms(1);
}

void calibratePressureSensor(MS5607_t* sensor)
//Set up the appropriate SPI before calling this
{
	pressureSensorReset(sensor);
	
	//printf("reset\n");
	//Get all the lovely little calibration constants
	
	spiselect(sensor->select_pin);
	spiwrite(&PRESSURE_SENSOR_SPI, 0b10100010); //Bits 1 - 3 are 001, for C1
	sensor->SENSt1 = read16(&PRESSURE_SENSOR_SPI);
	spideselect(sensor->select_pin);

	spiselect(sensor->select_pin);
	spiwrite(&PRESSURE_SENSOR_SPI, 0b10100100); //010 = 2, for C2
	sensor->OFFt1 = read16(&PRESSURE_SENSOR_SPI);
	spideselect(sensor->select_pin);

	spiselect(sensor->select_pin);
	spiwrite(&PRESSURE_SENSOR_SPI, 0b10100110); // 011 = 3, for C3
	sensor->TCS = read16(&PRESSURE_SENSOR_SPI);
	spideselect(sensor->select_pin);

	spiselect(sensor->select_pin);
	spiwrite(&PRESSURE_SENSOR_SPI, 0b10101000); // 100 = 4
	sensor->TCO = read16(&PRESSURE_SENSOR_SPI);
	spideselect(sensor->select_pin);

	spiselect(sensor->select_pin);
	spiwrite(&PRESSURE_SENSOR_SPI, 0b10101010); // 101 = 5
	sensor->Tref = read16(&PRESSURE_SENSOR_SPI);
	spideselect(sensor->select_pin);


	spiselect(sensor->select_pin);
	spiwrite(&PRESSURE_SENSOR_SPI, 0b10101100); // 110 = 6
	sensor->TEMPSENS = read16(&PRESSURE_SENSOR_SPI);
	spideselect(sensor->select_pin);
	
	
// 	printf("C1: %u\n", sensor->SENSt1);
// 	printf("C2: %u\n", sensor->OFFt1);
// 	printf("C3: %u\n", sensor->TCS);
// 	printf("C4: %u\n", sensor->TCO);
// 	printf("C5: %u\n", sensor->Tref);
// 	printf("C6: %u\n", sensor->TEMPSENS);
}

void askForPressureMS5607(MS5607_t* sensor)
{
	//Ask for raw pressure, 4096 OSR
	spiselect(sensor->select_pin);
	spiwrite(&PRESSURE_SENSOR_SPI, 0x48); //OSR = 4096
	spideselect(sensor->select_pin);
}

void readRawPressureMS5607(MS5607_t* sensor)
{
	//Read off raw pressure (D1)
	spiselect(sensor->select_pin);
	spiwrite(&PRESSURE_SENSOR_SPI, 0x0);
	rawPressure = read24(&PRESSURE_SENSOR_SPI);
/*	printf("rawPressure is %" PRIi32 "\n", rawPressure);*/
	spideselect(sensor->select_pin);
}

void askForTemperatureMS5607(MS5607_t* sensor)
{
	//Ask for raw temperature, 4096 OSR
	spiselect(sensor->select_pin);
	spiwrite(&PRESSURE_SENSOR_SPI, 0x58); //OSR = 4096
	spideselect(sensor->select_pin);
}

void readRawTemperatureMS5607(MS5607_t* sensor)
{
	//Read off raw temperature (D2)
	spiselect(sensor->select_pin);
	spiwrite(&PRESSURE_SENSOR_SPI, 0x0);
	rawTemp = read24(&PRESSURE_SENSOR_SPI);
/*	printf("rawTemp is %" PRIi32 "\n", rawTemp);*/
	spideselect(sensor->select_pin);
}

void calculateValuesMS5607(MS5607_t* sensor, int32_t* pressure_dest, int32_t* temperature_dest)
{
	int32_t dT = rawTemp - (int32_t)(((int64_t)sensor->Tref) * ((int64_t)256));
	int32_t TEMP = (int32_t)(((int32_t)2000) + ((int32_t)(((int64_t)dT) * ((int64_t)sensor->TEMPSENS) / ((int64_t)8388608))));
	
	int32_t T2 = 0;
	int64_t OFF2 = 0;
	int64_t SENS2 = 0;
	
	if (TEMP < 2000)
	{
		T2 = ((int64_t)dT) * ((int64_t)dT) / ((int64_t)2147483648);
		OFF2 = ((int64_t)61) * ((int64_t)(TEMP - 2000)) * ((int64_t)(TEMP - 2000)) / ((int64_t)16);
		SENS2 = ((int64_t)2) * ((int64_t)(TEMP - 2000)) * ((int64_t)(TEMP - 2000));
	}
	else
	{
	    T2 = 0;
		OFF2 = 0;
		SENS2 = 0;	
	} 
	
	if (TEMP<-1500)
	{
		OFF2 += ((int64_t)15) * (((int64_t) TEMP) + ((int64_t) 1500))^2; 
		SENS2 += ((int64_t) 8) * (((int64_t) TEMP) + ((int64_t) 1500) )^2;
	}
	int64_t OFF = ((int64_t)sensor->OFFt1) * ((int64_t)131072) +
	(((int64_t)sensor->TCO) * ((int64_t)dT)) / ((int64_t)64);
	
	int64_t SENS = ((int64_t)sensor->SENSt1) * ((int64_t)65536) +
		(
			((int64_t)sensor->TCS) * ((int64_t)dT)
		) / ((int64_t)128);
		
	TEMP -= T2;
	OFF -= OFF2;
	SENS -= SENS2;
	
	int64_t PRESSURE = (((int64_t)rawPressure) * SENS / ((int64_t)2097152) - OFF) 
			/ ((int64_t)32768);
			
// 	printf("C1: %u\nC2: %u\nC3: %u\nC4: %u\nC5: %u\nC6: %u\n", 
// 	sensor->SENSt1,
// 	sensor->OFFt1,
// 	sensor->TCS,
// 	sensor->TCO,
// 	sensor->Tref,
// 	sensor->TEMPSENS);
				
// 	printf("TEMP: %" PRIi32 "\n", TEMP);
// 	printf("Pressure: %" PRIi32 "\n", (int32_t)PRESSURE);
	
	*pressure_dest = (int32_t) PRESSURE; //In pascals
	*temperature_dest = TEMP; //In hundredths of degree celsius
}



void readMS5607(MS5607_t* sensor, int32_t* pressure_dest, int32_t* temperature_dest)
 {
	askForPressureMS5607(sensor);

	delay_ms(10);

	readRawPressureMS5607(sensor);
	askForTemperatureMS5607(sensor);
	
	delay_ms(10);

	readRawTemperatureMS5607(sensor);
	calculateValuesMS5607(sensor,pressure_dest,temperature_dest);
 }
 
 