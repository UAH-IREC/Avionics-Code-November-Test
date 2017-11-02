/*
 * apbmand.h
 *
 * Created: 5/17/2017 2:10:53 PM
 *  Author: trb0023
 */ 

#include <asf.h>

#ifndef APBMAND_H_
#define APBMAND_H_

#define APBMAND_TWI_SLAVE_ADD 0x28

typedef struct
{
	TWI_t *twi;			//the TWI port that this sensor is connected to
} apbmand_t;

typedef struct
{
	double pressure;
	double airspeed;
	bool stale;
} apbmand_data_t;


apbmand_data_t read_apbmand(apbmand_t chip);


#endif /* APBMAND_H_ */