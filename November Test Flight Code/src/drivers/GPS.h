/*
 * GPS.h
 *
 * Created: 5/3/2016 9:12:05 PM
 *  Author: dcorey
 */ 


#ifndef GPS_H_
#define GPS_H_

#include <asf.h>
//#include <CommandParse.h>
#include "tools/RingBuffer.h"

#define GPS_PORT							PORTD
#define GPS_USART							USARTD0
#define GPS_RECEIVE_INTERRUPT_VECTOR		USARTD0_RXC_vect
#define GPS_SEND_INTERRUPT_VECTOR			USARTD0_TXC_vect
#define GPS_SEND_PIN						IOPORT_CREATE_PIN(PORTD, 3)
#define GPS_RECIEVE_PIN						IOPORT_CREATE_PIN(PORTD, 2)

#define GPS_MODE_NMEA		0
#define GPS_MODE_UBX		1

#define UBX_CLASS_NAV			0x01
#define UBX_CLASS_RXM			0x02
#define UBX_CLASS_INF			0x04
#define UBX_CLASS_ACK			0x05
#define UBX_CLASS_CFG			0x06
#define UBX_CLASS_MON			0x0a
#define UBX_CLASS_AID			0x0b
#define UBX_CLASS_TIM			0x0d

#define UBX_ID_CFG_NAV5			0x24
#define	UBX_ID_CFG_POLL_MSG		0x01

typedef struct GPS_data
{
	uint16_t latdegrees;
	uint32_t latminutes; // actually minutes * 10000
	char latdirection;
	double latdecimal;
	uint16_t londegrees;
	uint32_t lonminutes; // actually minutes * 10000
	char londirection;
	double londecimal;
	float altitude;
	uint8_t hour;
	uint8_t minutes;
	uint8_t seconds;
	uint8_t fix_status;
	
} GPS_data_t;


void init_GPS_pins_and_usart(void);
void sendUBXMessage(USART_t* dest, uint8_t class, uint8_t id, uint16_t length, uint8_t* payload);
uint8_t get_dynamics_config(void);
void configure_gps(void);
uint8_t takeoldestNMEAmessage(RingBufferu8_t* gpsdata, uint8_t* buff);
uint8_t findnext(uint8_t* searchstr, uint8_t target, uint8_t length);
GPS_data_t getGPSData(void);
void switch_to_UBX(void);
GPS_data_t parse_NMEA(uint8_t* bufferdata, uint8_t length);
uint8_t find_the_beginning (uint8_t* some_array, uint8_t length);
uint8_t find_the_end(uint8_t* some_array, uint8_t length);
GPS_data_t getGPSDatafromNMEA(uint8_t* NMEA_sentence, uint8_t sentencelength);
#define NORTH	'N'
#define SOUTH	'S'
#define WEST	'W'
#define EAST	'E'


uint8_t gpgga_index, gpgll_index, gpvtg_index, gpgsv_index;
uint8_t gpgga_buff[85];
uint8_t gpgll_buff[85];
uint8_t gpvtg_buff[85];
uint8_t gpgsv_buff[85];
uint8_t dollar_counter;
uint8_t current_sentence, last_finished;

#define SENTENCE_NONE	0
#define SENTENCE_GPGGA	1
#define SENTENCE_GPGLL	2
#define SENTENCE_GPVTG	3
#define SENTENCE_GPGSV	4


#endif /* GPS_H_ */