/*
* mcp7940.c
*
* Created: 5/11/2017 6:24:03 PM
*  Author: trb0023
*/ 

#include "drivers/mcp7940.h"

const char *weekdays[7] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
const char *months[12] = {"January", "February", "March", "April", "May", "June", "July", "August", "September", "October", "November", "December"};
const char *endings[10] = {"th", "st", "nd", "rd", "th", "th", "th", "th", "th", "th"};

void MCP7940_twi_write_reg(mcp7940_t chip, uint8_t regi, uint8_t val);
void MCP7940_twi_read_multi(mcp7940_t chip, uint8_t regi, uint8_t length);

uint32_t calculate_timestamp(mcp7940_data_t data);
void configure_register(mcp7940_t chip, uint8_t registerAddress, uint8_t value, uint8_t position);
static uint8_t bit_set(uint8_t value, uint8_t bit);
static uint8_t bit_clear(uint8_t value, uint8_t bit);
uint8_t convert_user_address(uint8_t address);

static uint8_t MCP7940_twi_data[256];
//transaction settings
static twi_package_t readMCP7940 = {
	.addr[0]	  = 0x00,
	.addr_length = 1,
	.chip        = MCP7940_TWI_SLAVE_ADD,
	.buffer      = MCP7940_twi_data,
	.length      = 1,
	.no_wait     = false
};

void MCP7940_twi_write_reg(mcp7940_t chip, uint8_t regi,uint8_t val)
{
	readMCP7940.chip=MCP7940_TWI_SLAVE_ADD;
	readMCP7940.addr[0]	  = regi;
	readMCP7940.length=1;
	MCP7940_twi_data[0]=val;
	if( twi_master_write(chip.twi, &readMCP7940)==TWI_SUCCESS)
	{
		//printf("\nWrite Successful");
	}
	else
	{
		printf("\nWrite Failed");
	}
}

void MCP7940_twi_read_multi(mcp7940_t chip, uint8_t regi, uint8_t length)
{
	readMCP7940.chip=MCP7940_TWI_SLAVE_ADD;
	readMCP7940.addr[0]	  = regi;
	readMCP7940.length=length;

	if( twi_master_read(chip.twi, &readMCP7940)==TWI_SUCCESS)
	{
		//printf("\nRead Successful");
	}
	else
	{
		printf("\nRead Failed");
	}
}

void init_mcp7940(mcp7940_t chip)
{
	uint8_t registerValue = 0x00;
	uint8_t twelveHour = 0x00;
	uint8_t batEnable = 0x01;
	uint8_t startClock = 0x01;
	uint8_t coarseTrim = 0x00;
	uint8_t trimVal = 0x00;

	MCP7940_twi_write_reg(chip, REG_RTCOSCTRIM, trimVal);

	configure_register(chip, REG_RTCHOUR, twelveHour, 6);
	configure_register(chip, REG_RTCWKDAY, batEnable, 3);
	configure_register(chip, REG_RTCSEC, startClock, 7);
	configure_register(chip, REG_RTCCONTROL, coarseTrim, 7);


}

void set_time_mcp7940(mcp7940_t chip, mcp7940_data_t date)
{
	MCP7940_twi_write_reg(chip, REG_RTCSEC, convert_to_BCD(date.seconds));
	MCP7940_twi_write_reg(chip, REG_RTCMIN, convert_to_BCD(date.minutes));
	MCP7940_twi_write_reg(chip, REG_RTCHOUR, convert_to_BCD(date.hours));
	MCP7940_twi_write_reg(chip, REG_RTCWKDAY, convert_to_BCD(date.weekday));
	MCP7940_twi_write_reg(chip, REG_RTCDATE, convert_to_BCD(date.date));
	MCP7940_twi_write_reg(chip, REG_RTCMTH, convert_to_BCD(date.month));
	MCP7940_twi_write_reg(chip, REG_RTCYEAR, convert_to_BCD(date.year));
}

mcp7940_data_t read_mcp7940(mcp7940_t chip)
{
	MCP7940_twi_read_multi(chip, REG_RTCSEC, 7);

	mcp7940_data_t data =
	{
		.seconds	= convert_from_BCD(MCP7940_twi_data[0] & 0b01111111),
		.minutes	= convert_from_BCD(MCP7940_twi_data[1] & 0b01111111),
		.hours		= convert_from_BCD(MCP7940_twi_data[2] & 0b00111111),
		.weekday	= convert_from_BCD(MCP7940_twi_data[3] & 0b00000111),
		.date		= convert_from_BCD(MCP7940_twi_data[4] & 0b00111111),
		.month		= convert_from_BCD(MCP7940_twi_data[5] & 0b00011111),
		.year		= convert_from_BCD(MCP7940_twi_data[6] & 0b11111111)

	};	
	
	return data;
}

void print_date_mcp7940(mcp7940_t chip)
{
	mcp7940_data_t date = read_mcp7940(chip);

	printf	("\nCurrent Time: %i:%02i:%02i on %s the %i%s of %s, 20%i", date.hours, date.minutes,
			 date.seconds, weekdays[date.weekday-1], date.date, endings[date.date%10], 
			 months[date.month-1], date.year);
}

uint32_t get_timestamp_mcp7940(mcp7940_t chip)
{
	return calculate_timestamp(read_mcp7940(chip));
}

//Address goes from 0 to 63
void write_user_byte_mcp7940(mcp7940_t chip, uint8_t address, uint8_t data)
{
	MCP7940_twi_write_reg(chip, convert_user_address(address), data);
}

//Address goes from 0 to 63
uint8_t read_user_byte_mcp7940(mcp7940_t chip, uint8_t address)
{
	MCP7940_twi_read_multi(chip, convert_user_address(address), 1);
	return MCP7940_twi_data[0];
}

uint8_t convert_user_address(uint8_t address)
{
	uint8_t convertedAddr = address + 0x20;
	convertedAddr = min(convertedAddr,0x5F);
	return convertedAddr;
}


uint32_t calculate_timestamp(mcp7940_data_t data)
{
	//Fails if used at midnight on the last day of a month
	return (uint32_t)data.seconds + (uint32_t)data.minutes*60 + (uint32_t)data.hours*60*60 + (uint32_t)data.date*24*60*60;
}


uint8_t convert_to_BCD(uint8_t byteDecimal)
{
	return ((byteDecimal / 10) << 4) | (byteDecimal % 10);
}

uint8_t convert_from_BCD(uint8_t byteBCD)
{
	uint8_t byteMSB = 0;
	uint8_t byteLSB = 0;
	byteMSB = (byteBCD & 0b11110000) >> 4;
	byteLSB = (byteBCD & 0b00001111);
	return (byteMSB*10)+byteLSB;
}

void configure_register(mcp7940_t chip, uint8_t registerAddress, uint8_t value, uint8_t position)
{
		MCP7940_twi_read_multi(chip, registerAddress,1);
		uint8_t registerValue = MCP7940_twi_data[0];
		if( value == 0x00 )
		{
			MCP7940_twi_write_reg(chip, registerAddress, bit_clear(registerValue, position));
		}
		else if ( value == 0x01 )
		{
			MCP7940_twi_write_reg(chip, registerAddress, bit_set(registerValue, position));
		}
}

static uint8_t bit_set(uint8_t value, uint8_t bit)
{
	return value | (1 << bit);
}

static uint8_t bit_clear(uint8_t value, uint8_t bit)
{
	return value & (~(1 << bit));
}