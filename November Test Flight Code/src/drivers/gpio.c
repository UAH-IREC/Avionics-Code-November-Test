/*
 * gpio.c
 *
 * Created: 5/18/2017 7:53:22 PM
 *  Author: trb0023
 */ 
#include "gpio.h"

static uint8_t bit_set(uint8_t value, uint8_t bit);
static uint8_t bit_clear(uint8_t value, uint8_t bit);
static uint8_t bit_toggle(uint8_t value, uint8_t bit);

void init_gpio(void)
{
	PORTA.DIR |= 0b10000000;

	PORTC.DIR |= 0b01110000;
	
// 	PORTF.DIR |= 0b00000111;
// 	PORTF.OUT |= 0b00000111;
}

void led_r_disable(void)
{
	PORTF.OUT = bit_set(PORTF.OUT, 0);
}

void led_r_enable(void)
{
	PORTF.OUT = bit_clear(PORTF.OUT, 0);
}

void led_r_toggle(void)
{
	PORTF.OUT = bit_toggle(PORTF.OUT, 0);
}

void led_g_disable(void)
{
	PORTF.OUT = bit_set(PORTF.OUT, 1);
}

void led_g_enable(void)
{
	PORTF.OUT = bit_clear(PORTF.OUT, 1);
}

void led_g_toggle(void)
{
	PORTF.OUT = bit_toggle(PORTF.OUT, 1);
}

void led_b_disable(void)
{
	PORTF.OUT = bit_set(PORTF.OUT, 2);
}

void led_b_enable(void)
{
	PORTF.OUT = bit_clear(PORTF.OUT, 2);
}

void led_b_toggle(void)
{
	PORTF.OUT = bit_toggle(PORTF.OUT, 2);
}

void cam_power_enable(void)
{
	PORTC.OUT = bit_set(PORTC.OUT, 6);
}

void cam_power_disable(void)
{
	PORTC.OUT = bit_clear(PORTC.OUT, 6);
}

void cam_power_toggle(void)
{
	PORTC.OUT = bit_toggle(PORTC.OUT, 6);
}

void cam_trigger_enable(void)
{
	PORTC.OUT = bit_set(PORTC.OUT, 5);
}

void cam_trigger_disable(void)
{
	PORTC.OUT = bit_clear(PORTC.OUT, 5);
}

void cam_trigger_toggle(void)
{
	PORTC.OUT = bit_toggle(PORTC.OUT, 5);
}

void buzzer_enable(void)
{
	PORTC.OUT = bit_toggle(PORTC.OUT, 4);
}

void buzzer_disable(void)
{
	PORTC.OUT = bit_clear(PORTC.OUT, 4);
}

void buzzer_toggle(void)
{
	PORTC.OUT = bit_toggle(PORTC.OUT, 4);
}

void cutdown_enable(void)
{
	PORTA.OUT = bit_toggle(PORTA.OUT, 7);
}

void cutdown_disable(void)
{
	PORTA.OUT = bit_clear(PORTA.OUT, 7);
}

void cutdown_toggle(void)
{
	PORTA.OUT = bit_toggle(PORTA.OUT, 7);
}



static uint8_t bit_set(uint8_t value, uint8_t bit)
{
	return value | (1 << bit);
}

static uint8_t bit_clear(uint8_t value, uint8_t bit)
{
	return value & (~(1 << bit));
}

static uint8_t bit_toggle(uint8_t value, uint8_t bit)
{
	return value ^ (1 << bit);
}