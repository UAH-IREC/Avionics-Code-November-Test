/*
 * state_save.h
 *
 * Created: 5/26/2017 8:42:10 PM
 *  Author: trb0023
 */ 
#include <asf.h>
#include "drivers/mcp7940.h"
#include "drivers/imu.h"

#define ADDR_FLIGHT_STATE 1
#define ADDR_START_TIME 2
#define ADDR_PACKET_COUNT 6
#define ADDR_GROUND_ALT 10
#define ADDR_MAG_CAL 14
#define ADDR_IS_THIS_FOR_REAL 20	
#define ADDR_WHAT_AM_I 21
#define ADDR_PICTURES_TAKEN 22

#ifndef STATE_SAVE_H_
#define STATE_SAVE_H_



void	 save_flight_state(mcp7940_t chip, uint8_t state);					//1 byte
uint8_t  load_flight_state(mcp7940_t chip);

void	 save_start_time(mcp7940_t chip, uint32_t start_time);				//4 bytes
uint32_t load_start_time(mcp7940_t chip);

void	 save_packet_count(mcp7940_t chip, uint32_t packet_count);			//4 bytes
uint32_t load_packet_count(mcp7940_t chip);

void	 save_ground_alt(mcp7940_t chip, double ground_alt);				//4 bytes? (double check)
double	 load_ground_alt(mcp7940_t chip);

void	 save_mag_cal(mcp7940_t chip);										//6 bytes
void	 load_mag_cal(mcp7940_t chip);

void	 save_is_this_for_real(mcp7940_t chip, uint8_t is_this_for_real);	//1 byte
uint8_t	 load_is_this_for_real(mcp7940_t chip);

void	 save_what_am_i(mcp7940_t chip, uint8_t what_am_i);					//1 byte
uint8_t	 load_what_am_i(mcp7940_t chip);

void	 save_pictures_taken(mcp7940_t chip, uint32_t pictures_taken);		//4 bytes
uint32_t load_pictures_taken(mcp7940_t chip);

#endif /* STATE_SAVE_H_ */