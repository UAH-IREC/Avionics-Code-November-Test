/*
 * mcp7940.h
 *
 * Created: 5/11/2017 6:24:16 PM
 *  Author: trb0023
 */ 
 #include <asf.h>

#ifndef MCP7940_H_
#define MCP7940_H_

#define MCP7940_TWI_SLAVE_ADD 0x6F

typedef struct
{
	TWI_t *twi;			//the TWI port that this sensor is connected to
} mcp7940_t;

typedef struct
{
	uint8_t seconds;				//
	uint8_t minutes;
	uint8_t hours;
	uint8_t weekday;
	uint8_t date;
	uint8_t month;
	uint8_t year;
} mcp7940_data_t;



#define REG_RTCSEC		0x00
#define REG_RTCMIN		0x01
#define REG_RTCHOUR		0x02
#define REG_RTCWKDAY	0x03
#define REG_RTCDATE		0x04
#define REG_RTCMTH		0x05
#define REG_RTCYEAR		0x06
#define REG_RTCCONTROL	0x07
#define REG_RTCOSCTRIM	0x08


uint8_t convert_to_BCD(uint8_t byteDecimal);
uint8_t convert_from_BCD(uint8_t byteBCD);


void init_mcp7940(mcp7940_t chip);
void set_time_mcp7940(mcp7940_t chip, mcp7940_data_t date);
mcp7940_data_t read_mcp7940(mcp7940_t chip);
void print_date_mcp7940(mcp7940_t chip);
uint32_t get_timestamp_mcp7940(mcp7940_t chip);

void write_user_byte_mcp7940(mcp7940_t chip, uint8_t address, uint8_t data);
uint8_t read_user_byte_mcp7940(mcp7940_t chip, uint8_t address);

#endif /* MCP7940_H_ */