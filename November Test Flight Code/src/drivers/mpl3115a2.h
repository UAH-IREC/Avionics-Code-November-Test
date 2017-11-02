/*
 * mpl3115a2.h
 *
 * Created: 5/11/2017 11:55:52 AM
 *  Author: trb0023

 Put the following code in the initialization portion of the main file before using this sensor:

	twi_options_t m_options = {
		.speed = 100000,
		.speed_reg = TWI_BAUD(32000000, 100000),
	};
	
	sysclk_enable_peripheral_clock(&TWIC);
	twi_master_init(&TWIC, &m_options);
	twi_master_enable(&TWIC);

	mpl3115_t altimeter =
	{
		.twi = &TWIC
	};

	uint8_t altimeterState=MPL3115_STATE_UNITITIALIZED;
	init_mpl3115(altimeter);
	altimeterState = MPL3115_STATE_STANDBY;

Here is an example of how to use the sensor without blocking other sensors:

	double altitude = 0.0;

	while(true)
	{
		if (altimeterState == MPL3115_STATE_STANDBY)
		{
			start_read_mpl3115(altimeter);
			altimeterState = MPL3115_STATE_DATA_NOT_READY;
		}
		if (altimeterState == MPL3115_STATE_DATA_NOT_READY)
		{
			altimeterState = is_data_ready_mpl3115(altimeter) ? MPL3115_STATE_DATA_READY : MPL3115_STATE_DATA_NOT_READY;
		}
		if(altimeterState == MPL3115_STATE_DATA_READY)
		{
			altitude = get_data_mpl3115(altimeter).altitude;
			altimeterState = MPL3115_STATE_STANDBY;
		}

		printf("\n%f",altitude);
		// Rest of program

		delay_ms(100);
	}

 */ 


#ifndef MPL3115A2_H_
#define MPL3115A2_H_

#include <asf.h>
#include <util/delay.h>

//slave address of this chip
#define MPL3115_TWI_SLAVE_ADD 0x60

#define EARTH_RADIUS 6356.766

#define MPL3115_STATE_UNITITIALIZED 0
#define MPL3115_STATE_STANDBY 1
#define MPL3115_STATE_DATA_NOT_READY 2
#define MPL3115_STATE_DATA_READY 3

typedef struct
{
	TWI_t *twi;			//the TWI port that this sensor is connected to
} mpl3115_t;

typedef struct
{
	double pressure;				//Pressure in pascals
	double altitude;				//Altitude in meters
} mpl3115_data_t;


void init_mpl3115(mpl3115_t chip);					//Initializes the Altimeter sensor
void start_read_mpl3115(mpl3115_t chip);			//Start the data read process
bool is_data_ready_mpl3115(mpl3115_t chip);			//Checks to see if the data is ready
mpl3115_data_t get_data_mpl3115(mpl3115_t chip);	//Reads the altitude from the chip and calculates the pressure
mpl3115_data_t read_mpl3115(mpl3115_t chip);		//Does the full read process, but it's inefficient. Use other 3 functions if possible


#endif /* MPL3115A2_H_ */