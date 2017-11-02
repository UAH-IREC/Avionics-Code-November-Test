/*
 * GPS_Interrupts.h
 *
 * Created: 5/3/2016 9:02:31 PM
 *  Author: dcorey
 */ 


#ifndef GPS_INTERRUPTS_H_
#define GPS_INTERRUPTS_H_

#define GPS_RECEIVE_BUFFER_SIZE 256
#define GPS_TRANSMIT_BUFFER_SIZE 64

#include "tools/RingBuffer.h"

void init_gps_interrupts(void);
void init_gps_buffers(void);

// These two are just used by the RingBuffer objects below, don't access directly
volatile uint8_t RAW_gps_receive_buffer[GPS_RECEIVE_BUFFER_SIZE];
volatile uint8_t RAW_gps_send_buffer[GPS_TRANSMIT_BUFFER_SIZE];

// The XBee receive interrupt should add data to this
// The cansat should read off of it to receive commands
volatile RingBufferu8_t gps_receive_buffer;

// The CanSat should write telemetry and image packets to this
// The XBee interrupt should read off of it to send those packets to the ground station
volatile RingBufferu8_t gps_send_buffer;

#endif /* GPS_INTERRUPTS_H_ */