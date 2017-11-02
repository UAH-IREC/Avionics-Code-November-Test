/*
* state_save.c
*
* Created: 5/26/2017 8:41:54 PM
*  Author: trb0023
*/

#include "drivers/state_save.h"

union double_and_uint32 {
	double double_value;
	uint32_t uint_value;
};

union int16_and_uint16 {
	uint16_t uint_value;
	int16_t int_value;
};

void	 save_flight_state(mcp7940_t chip, uint8_t state)
{
	write_user_byte_mcp7940(chip,ADDR_FLIGHT_STATE,state);
}
uint8_t  load_flight_state(mcp7940_t chip)
{
	return read_user_byte_mcp7940(chip,ADDR_FLIGHT_STATE);
}

void	 save_start_time(mcp7940_t chip, uint32_t start_time)
{
	uint32_t data = start_time;

	uint8_t data1 = data;
	uint8_t data2 = data>>8;
	uint8_t data3 = data>>16;
	uint8_t data4 = data>>24;

	write_user_byte_mcp7940(chip, ADDR_START_TIME, data1);
	write_user_byte_mcp7940(chip, ADDR_START_TIME+1, data2);
	write_user_byte_mcp7940(chip, ADDR_START_TIME+2, data3);
	write_user_byte_mcp7940(chip, ADDR_START_TIME+3, data4);
}
uint32_t load_start_time(mcp7940_t chip)
{
	uint32_t combined = read_user_byte_mcp7940(chip, ADDR_START_TIME+3);
	combined <<= 8;
	combined |= read_user_byte_mcp7940(chip, ADDR_START_TIME+2);
	combined <<= 8;
	combined |= read_user_byte_mcp7940(chip, ADDR_START_TIME+1);
	combined <<= 8;
	combined |= read_user_byte_mcp7940(chip, ADDR_START_TIME);
	
	return combined;
}

void	 save_packet_count(mcp7940_t chip, uint32_t packet_count)
{
	uint32_t data = packet_count;

	uint8_t data1 = data;
	uint8_t data2 = data>>8;
	uint8_t data3 = data>>16;
	uint8_t data4 = data>>24;

	write_user_byte_mcp7940(chip, ADDR_PACKET_COUNT, data1);
	write_user_byte_mcp7940(chip, ADDR_PACKET_COUNT+1, data2);
	write_user_byte_mcp7940(chip, ADDR_PACKET_COUNT+2, data3);
	write_user_byte_mcp7940(chip, ADDR_PACKET_COUNT+3, data4);
}
uint32_t load_packet_count(mcp7940_t chip)
{
	uint32_t combined = read_user_byte_mcp7940(chip, ADDR_PACKET_COUNT+3);
	combined <<= 8;
	combined |= read_user_byte_mcp7940(chip, ADDR_PACKET_COUNT+2);
	combined <<= 8;
	combined |= read_user_byte_mcp7940(chip, ADDR_PACKET_COUNT+1);
	combined <<= 8;
	combined |= read_user_byte_mcp7940(chip, ADDR_PACKET_COUNT);
	
	return combined;
}

void	 save_ground_alt(mcp7940_t chip, double ground_alt)
{
	union double_and_uint32 double_to_uint;
	double_to_uint.double_value = ground_alt;

	uint32_t data = double_to_uint.uint_value;

	uint8_t data1 = data;
	uint8_t data2 = data>>8;
	uint8_t data3 = data>>16;
	uint8_t data4 = data>>24;

	write_user_byte_mcp7940(chip, ADDR_GROUND_ALT, data1);
	write_user_byte_mcp7940(chip, ADDR_GROUND_ALT+1, data2);
	write_user_byte_mcp7940(chip, ADDR_GROUND_ALT+2, data3);
	write_user_byte_mcp7940(chip, ADDR_GROUND_ALT+3, data4);
}
double	 load_ground_alt(mcp7940_t chip)
{
	uint32_t combined = read_user_byte_mcp7940(chip, ADDR_GROUND_ALT+3);
	combined <<= 8;
	combined |= read_user_byte_mcp7940(chip, ADDR_GROUND_ALT+2);
	combined <<= 8;
	combined |= read_user_byte_mcp7940(chip, ADDR_GROUND_ALT+1);
	combined <<= 8;
	combined |= read_user_byte_mcp7940(chip, ADDR_GROUND_ALT);

	union double_and_uint32 uint_to_double;
	uint_to_double.uint_value = combined;
	return uint_to_double.double_value;
}

void	 save_mag_cal(mcp7940_t chip)
{
	union int16_and_uint16 int_to_uint;
	imu_mag_cal_t cal = imu_get_mag_calibration();


	int_to_uint.int_value = cal.x_off;
	uint16_t xdata = int_to_uint.uint_value;

	uint8_t xdata1 = xdata;
	uint8_t xdata2 = xdata>>8;

	write_user_byte_mcp7940(chip, ADDR_MAG_CAL, xdata1);
	write_user_byte_mcp7940(chip, ADDR_MAG_CAL+1, xdata2);


	int_to_uint.int_value = cal.y_off;
	uint16_t ydata = int_to_uint.uint_value;

	uint8_t ydata1 = ydata;
	uint8_t ydata2 = ydata>>8;

	write_user_byte_mcp7940(chip, ADDR_MAG_CAL+2, ydata1);
	write_user_byte_mcp7940(chip, ADDR_MAG_CAL+1+2, ydata2);


	int_to_uint.int_value = cal.z_off;
	uint16_t zdata = int_to_uint.uint_value;

	uint8_t zdata1 = zdata;
	uint8_t zdata2 = zdata>>8;

	write_user_byte_mcp7940(chip, ADDR_MAG_CAL+4, zdata1);
	write_user_byte_mcp7940(chip, ADDR_MAG_CAL+1+4, zdata2);

}
void	 load_mag_cal(mcp7940_t chip)
{
	imu_mag_cal_t cal;
	union int16_and_uint16 uint_to_int;


	uint16_t combinedx = read_user_byte_mcp7940(chip, ADDR_MAG_CAL+1);
	combinedx <<= 8;
	combinedx |= read_user_byte_mcp7940(chip, ADDR_MAG_CAL);

	uint_to_int.uint_value = combinedx;
	cal.x_off = uint_to_int.int_value;


	uint16_t combinedy = read_user_byte_mcp7940(chip, ADDR_MAG_CAL+1+2);
	combinedy <<= 8;
	combinedy |= read_user_byte_mcp7940(chip, ADDR_MAG_CAL+2);

	uint_to_int.uint_value = combinedy;
	cal.y_off = uint_to_int.int_value;


	uint16_t combinedz = read_user_byte_mcp7940(chip, ADDR_MAG_CAL+1+4);
	combinedz <<= 8;
	combinedz |= read_user_byte_mcp7940(chip, ADDR_MAG_CAL+4);

	uint_to_int.uint_value = combinedz;
	cal.z_off = uint_to_int.int_value;


	imu_set_mag_calibration(cal);
}

void	 save_is_this_for_real(mcp7940_t chip, uint8_t is_this_for_real)
{
	write_user_byte_mcp7940(chip,ADDR_IS_THIS_FOR_REAL,is_this_for_real);
}
uint8_t	 load_is_this_for_real(mcp7940_t chip)
{
	read_user_byte_mcp7940(chip,ADDR_IS_THIS_FOR_REAL);
}

void	 save_what_am_i(mcp7940_t chip, uint8_t what_am_i)
{
	write_user_byte_mcp7940(chip,ADDR_WHAT_AM_I,what_am_i);
}
uint8_t	 load_what_am_i(mcp7940_t chip)
{
	read_user_byte_mcp7940(chip,ADDR_WHAT_AM_I);
}

void	 save_pictures_taken(mcp7940_t chip, uint32_t pictures_taken)
{
	uint32_t data = pictures_taken;

	uint8_t data1 = data;
	uint8_t data2 = data>>8;
	uint8_t data3 = data>>16;
	uint8_t data4 = data>>24;

	write_user_byte_mcp7940(chip, ADDR_PICTURES_TAKEN, data1);
	write_user_byte_mcp7940(chip, ADDR_PICTURES_TAKEN+1, data2);
	write_user_byte_mcp7940(chip, ADDR_PICTURES_TAKEN+2, data3);
	write_user_byte_mcp7940(chip, ADDR_PICTURES_TAKEN+3, data4);
}
uint32_t load_pictures_taken(mcp7940_t chip)
{
	uint32_t combined = read_user_byte_mcp7940(chip, ADDR_PICTURES_TAKEN+3);
	combined <<= 8;
	combined |= read_user_byte_mcp7940(chip, ADDR_PICTURES_TAKEN+2);
	combined <<= 8;
	combined |= read_user_byte_mcp7940(chip, ADDR_PICTURES_TAKEN+1);
	combined <<= 8;
	combined |= read_user_byte_mcp7940(chip, ADDR_PICTURES_TAKEN);
	
	return combined;
}