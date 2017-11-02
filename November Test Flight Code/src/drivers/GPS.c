/*+
 * GPS.c
 *
 * Created: 5/3/2016 9:11:56 PM
 *  Author: dcorey
 */ 

#include <string.h>
#include "drivers/GPS.h"
#include "interrupts/GPS_Interrupts.h"
//#include "config/ports.h"
//#include "drivers/uart_tools.h"
#include "tools/RingBuffer.h"
//#include "CommandParse.h"
//#include "tools/satellite_vectors.h"


void init_GPS_pins_and_usart(void)
{
	GPS_PORT.DIR |= 0b00001000;
	GPS_PORT.DIR &= 0b11111011;
	ioport_set_pin_high(GPS_SEND_PIN);
	
	//Config pins
	//PORTA.DIR |= 0b00001100;
// 	if (mode == GPS_MODE_NMEA)
// 	{
// 		ioport_set_pin_high(GPS_CONFIG_PIN_8);
// 		ioport_set_pin_high(GPS_CONFIG_PIN_16);
// 	}
// 	else //Assume UBX
// 	{
// 		ioport_set_pin_low(GPS_CONFIG_PIN_8);
// 		ioport_set_pin_low(GPS_CONFIG_PIN_16);
// 	}
	
	
	//Configure USART
	sysclk_enable_peripheral_clock(&GPS_USART);
	
	static usart_serial_options_t options = {
		.baudrate = 9600,
		.charlength = USART_CHSIZE_8BIT_gc,
		.paritytype = USART_PMODE_DISABLED_gc,
		.stopbits = 1
	};
	
	usart_serial_init(&GPS_USART, &options);
}

uint8_t startswith(uint8_t* data, const char* beginning)
{
	uint8_t i = 0;
	while (1)
	{
		if (beginning[i] == '\0')
		return 1;
		if (beginning[i] != data[i])
		return 0;
		i++;
	}
	return 1;
}

uint8_t find_the_beginning (uint8_t* some_array, uint8_t length)
{
uint8_t i;
	for ( i=0; i< (length); i++ )
	{
		if(some_array[i]=='$')
		{
			return i;
		}
	}
	return i+1;
}
uint8_t find_the_end(uint8_t* some_array, uint8_t length)
{
	uint8_t b=1;
	uint8_t i;
	for ( i=0; i< (length); i++ )
	{
		if(some_array[i]=='\r' && some_array[i+1]=='\n')
		{
			return i;
		}
	}
	return i+1;
}

GPS_data_t getGPSDatafromNMEA(uint8_t* NMEA_sentence, uint8_t sentencelength)
{
	int8_t countcommas=0;
	GPS_data_t GPSData;
	uint8_t first_comma_position=0;
	uint8_t second_comma_position=0;
	uint8_t third_comma_position=0;
	uint8_t fourth_comma_position=0;
	uint8_t fifth_comma_position=0;
	uint8_t comma7=0;
	uint8_t comma8=0;
	uint8_t comma9=0;
	uint8_t comma10=0;
	uint8_t commab=0;
	uint8_t commac=0;
	uint8_t speedwritten=0;
	uint8_t heightwritten=0;
	uint8_t no_of_sateliiteswritten=0;
	uint8_t comma_position=0;
	int a=0;
	if (startswith(NMEA_sentence, "$GPGGA"))
	{
		//printf("Parsing GPGGA with len %u\n", sentencelength);
		for (a=0;a<sentencelength; a++)
		{
			//printf("hate6");
			if (NMEA_sentence[a]==',')
			{
				//printf("gfhghhf");
				countcommas++;
				if (countcommas == 1)
				{
					//printf("Begin %u, end %u\n", NMEA_sentence[a + 1] - '0', NMEA_sentence[a + 2] - '0');
					GPSData.hour = 10 * (NMEA_sentence[a + 1] - (uint8_t)'0') + (NMEA_sentence[a + 2] - (uint8_t)'0');
					GPSData.minutes = 10 * (NMEA_sentence[a + 3] - '0') + NMEA_sentence[a + 4] - '0';
					GPSData.seconds = 10 * (NMEA_sentence[a + 5] - '0') + NMEA_sentence[a + 6] - '0';
				}
				else if (countcommas == 2 && NMEA_sentence[a+1] != ',')
				{
					GPSData.latdegrees = 10*(NMEA_sentence[a+1] - '0') + (NMEA_sentence[a+2] - '0');
					//printf("%u\n",GPSData.latdegrees);
					GPSData.latminutes = 100000l*(NMEA_sentence[a+3] - '0') + 10000l*(NMEA_sentence[a+4] - '0') + 1000l*(NMEA_sentence[a+6] - '0') + 100l*(NMEA_sentence[a+7] - '0') + 10l*(NMEA_sentence[a+8] - '0') + (NMEA_sentence[a+9] - '0');
					//printf("%lu\n",GPSData.latminutes);

					GPSData.latdecimal = (NMEA_sentence[a+11] == 'N' ? 1.0 : -1.0) * ((double)GPSData.latdegrees + ((double)GPSData.latminutes/(60.0*10000.0)));

					//printf("%f\n",GPSData.latdecimal);
				}
				else if (countcommas == 2)
				{
					GPSData.latdecimal = 0.0;
				}
				else if (countcommas == 4 && NMEA_sentence[a+1] != ',')
				{
					GPSData.londegrees = 100*(NMEA_sentence[a+1] - '0') +10*(NMEA_sentence[a+2] - '0') + (NMEA_sentence[a+3] - '0');
					//printf("%u\n",GPSData.londegrees);
					GPSData.lonminutes = 100000l*(NMEA_sentence[a+4] - '0')
									    + 10000l*(NMEA_sentence[a+5] - '0')
										 + 1000l*(NMEA_sentence[a+7] - '0')
										  + 100l*(NMEA_sentence[a+8] - '0')
										   + 10l*(NMEA_sentence[a+9] - '0')
										       + (NMEA_sentence[a+10] - '0');
					//printf("%lu\n",GPSData.lonminutes);
					GPSData.londecimal = (NMEA_sentence[a+12] == 'E' ? 1.0 : -1.0) * ((double)GPSData.londegrees + ((double)GPSData.lonminutes/(60.0*10000.0)));
					//printf("%f\n",GPSData.londecimal);
				}
				else if(countcommas == 4)
				{
					GPSData.londecimal = 0.0;
				}
				else if (countcommas == 6)
				{
					GPSData.fix_status = NMEA_sentence[a + 1] - '0';
				}
				else if (countcommas == 9)
				{
					uint8_t period_reached = 0;
					uint8_t period_counter;
					float altitude = 0;
					uint8_t inc = 0;
					while (NMEA_sentence[a + inc + 1] != ',')
					{
						if (NMEA_sentence[a + inc + 1] == '.')
						{
							period_reached = 1;
							period_counter = 1;
						}
						else if (!period_reached)
						{
							altitude *= 10;
							altitude += (float)(NMEA_sentence[a + inc + 1] - '0');
						}
						else if (period_reached && period_counter == 1)
						{
							altitude += ((float)(NMEA_sentence[a + inc + 1] - '0')) / 10.0;
							break;
						}
						
						inc++;
					}
					GPSData.altitude = altitude;
					//printf("Altitudes!\n");
				}
			}
			
		}
	}
	else
	{
		//printf("hate2");
		//GPSData.latdegrees[0]='\0';
		//GPSData.latminutes[0]='\0';
		//GPSData.latdirection=' ';
		//GPSData.longdegrees[0]='\0';
		//GPSData.longminutes[0]='\0';
		//GPSData.longdirection=' ';
		//GPSData.noofsatellites[0]='\0';
		//GPSData.noofsatellites[0]='\0';
		//GPSData.speed[0]='\0';
	}
			
	//printf("NMEA sentence %s\n", NMEA_sentence);
		

   //printf("hate6");
	
/*printf("GPS lat degrees %s\n", GPSData.latdegrees);
printf("GPS lat minutes %s\n",GPSData.latminutes);
printf("GPS lat direction %c\n", GPSData.latdirection);
printf("GPS long degrees %s\n", GPSData.longdegrees);
printf("GPS long minutes %s\n",GPSData.longminutes);
printf("GPS long direction %c \n", GPSData.longdirection);*/
	
	
	
	
	
	
//printf("GPSDATA altitude test %s \n ", GPSData.altitude);
//printf("GPS no of satellites test= %s \n", GPSData.noofsatellites);
return GPSData;


}


uint8_t findnext(uint8_t* searchstr, uint8_t target, uint8_t length)
{
	uint8_t i;
	for (i = 0; i < length; i++)
	{
		if (searchstr[i] == target)
		{
			return i;
		}
	}
}
