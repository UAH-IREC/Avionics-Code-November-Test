/*
 * apbmand.c
 *
 * Created: 5/17/2017 2:10:41 PM
 *  Author: trb0023
 */ 

#include "drivers/apbmand.h"
#include <math.h>

static uint8_t APBMAND_twi_data[2];
//transaction settings
static twi_package_t readAPBMAND = {
	.addr[0]	  = 0x00,
	.addr_length = 1,
	.chip        = APBMAND_TWI_SLAVE_ADD,
	.buffer      = APBMAND_twi_data,
	.length      = 1,
	.no_wait     = false
};
 
void APBMAND_twi_read_multi(apbmand_t chip, uint8_t regi, uint8_t length);
uint16_t read_data_apbmand(apbmand_t chip);

void APBMAND_twi_read_multi(apbmand_t chip, uint8_t regi, uint8_t length)
{
	readAPBMAND.chip=APBMAND_TWI_SLAVE_ADD;
	readAPBMAND.addr[0]	  = regi;
	readAPBMAND.length=length;

	if( twi_master_read(chip.twi, &readAPBMAND)==TWI_SUCCESS)
	{
		//printf("\nRead Successful");
	}
	else
	{
		printf("\nRead Failed");
	}
}

uint16_t read_data_apbmand(apbmand_t chip)
{
	APBMAND_twi_read_multi(chip, 0x00, 2);

	uint16_t data = APBMAND_twi_data[0] & 0b00111111;
	data <<= 8;
	data |= APBMAND_twi_data[1];

	return data;
}

apbmand_data_t read_apbmand(apbmand_t chip)
{
	uint32_t outMax = 0x3999;
	uint32_t outMin = 0x0666;
	double pMin = 0.0;
	double pMax = 1.0;
	uint32_t out = read_data_apbmand(chip);

	out = min(outMax, max(outMin, out));

	//printf("\n%u",out);

	double pressure = ((((double)(out-outMin))*(pMax-pMin))/((double)(outMax-outMin))) + pMin;
	pressure *= 6894.76;// convert to pascals
	//pressure -= 18.5;
	pressure = max(0, pressure);

	double airspeed = sqrt((2.0*pressure)/1.225);

	apbmand_data_t data;
	data.pressure = pressure;
	data.airspeed = airspeed;

	data.stale = ((APBMAND_twi_data[0] & 0b11000000) == 0b10000000);
	return data;
}

