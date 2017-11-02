/*
 * mpl3115a2.c
 *
 * Created: 5/11/2017 11:54:55 AM
 *  Author: trb0023
 */ 

#include "drivers/mpl3115a2.h"


void MPL3115_twi_write_reg(mpl3115_t chip, uint8_t regi, uint8_t val);
void MPL3115_twi_read_multi(mpl3115_t chip, uint8_t regi, uint8_t length);
void print_binary8(uint8_t n);
void print_binary32(uint32_t n);

double getStandardPressure(double altitude );
double getStandardTemperature(double geopot_height);
double getGeopotential(double altitude_km);

static uint8_t MPL3115_twi_data[256];
//transaction settings
static twi_package_t readMPL3115 = {
	.addr[0]	  = 0x00,
	.addr_length = 1,
	.chip        = MPL3115_TWI_SLAVE_ADD,
	.buffer      = MPL3115_twi_data,
	.length      = 1,
	.no_wait     = false
};

void MPL3115_twi_write_reg(mpl3115_t chip, uint8_t regi,uint8_t val)	
{
	readMPL3115.chip=MPL3115_TWI_SLAVE_ADD;
	readMPL3115.addr[0]	  = regi;
	readMPL3115.length=1;
	MPL3115_twi_data[0]=val;
	if( twi_master_write(chip.twi, &readMPL3115)==TWI_SUCCESS)
	{
		//printf("\nWrite Successful");
	}
	else
	{
		printf("\nWrite Failed");
	}
}

void MPL3115_twi_read_multi(mpl3115_t chip, uint8_t regi, uint8_t length)
{
	readMPL3115.chip=MPL3115_TWI_SLAVE_ADD;
	readMPL3115.addr[0]	  = regi;
	readMPL3115.length=length;

	if( twi_master_read(chip.twi, &readMPL3115)==TWI_SUCCESS)
	{
		//printf("\nRead Successful");
	}
	else
	{
		printf("\nRead Failed");
	}
}

void start_read_mpl3115(mpl3115_t chip)
{
	MPL3115_twi_write_reg(chip, 0x26, 0b10101011);
}

bool is_data_ready_mpl3115(mpl3115_t chip)
{
	MPL3115_twi_read_multi(chip, 0x00, 1);
	return (MPL3115_twi_data[0] & 0b00001110) == 0b00001110;
}

mpl3115_data_t get_data_mpl3115(mpl3115_t chip)
{
	MPL3115_twi_write_reg(chip, 0x26, 0b10101001);

	MPL3115_twi_read_multi(chip, 0x01, 5);

	int32_t alt = MPL3115_twi_data[0];
	alt <<= 8;
	alt |= MPL3115_twi_data[1];
	alt <<= 8;
	alt |= MPL3115_twi_data[2];
	alt >>= 4;

	double altitude = alt;
	altitude /= 16.0;

	mpl3115_data_t data;
	data.altitude = altitude;
	data.pressure = getStandardPressure(altitude);

	return data;
}

mpl3115_data_t read_mpl3115(mpl3115_t chip)			//read data from chip
{
	start_read_mpl3115(chip);
	
	while(!is_data_ready_mpl3115(chip)) {delay_ms(10);}

	return get_data_mpl3115(chip);
}

void init_mpl3115(mpl3115_t chip)
{
	MPL3115_twi_write_reg(chip, 0x26, 0b10101000);
	MPL3115_twi_write_reg(chip, 0x13, 0x07);
	MPL3115_twi_write_reg(chip, 0x26, 0b10101001);
}

// void print_binary8(uint8_t n)
// {
// 	int numbits = sizeof(uint8_t) * 8;
// 	while(--numbits >= 0) 
// 		printf("%c", (n & ((uint8_t)1<<numbits)) ? '1' : '0');
// }
// 
// void print_binary32(uint32_t n)
// {
// 	int numbits = sizeof(uint32_t) * 8;
// 	while(--numbits >= 0)
// 	printf("%c", (n & ((uint32_t)1<<numbits)) ? '1' : '0');
// }

//Copied from https://en.wikipedia.org/wiki/Barometric_formula with modification

double getStandardPressure(double altitude /* meters */)   // Returns result in Pascals
{
	altitude = altitude / 1000.0;  // Convert m to km
	double geopot_height = EARTH_RADIUS * altitude / (EARTH_RADIUS + altitude);

	double t = 288.15 - (6.5 * geopot_height);

	return  101325 * pow(288.15 / t, -5.255877);
}