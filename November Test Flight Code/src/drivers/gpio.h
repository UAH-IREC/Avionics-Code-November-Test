/*
 * gpio.h
 *
 * Created: 5/18/2017 7:53:34 PM
 *  Author: trb0023
 */ 

#include <asf.h>

#ifndef GPIO_H_
#define GPIO_H_

void init_gpio(void);

void led_r_enable(void);
void led_r_disable(void);
void led_r_toggle(void);

void led_g_enable(void);
void led_g_disable(void);
void led_g_toggle(void);

void led_b_enable(void);
void led_b_disable(void);
void led_b_toggle(void);

void cam_power_enable(void);
void cam_power_disable(void);
void cam_power_toggle(void);

void cam_trigger_enable(void);
void cam_trigger_disable(void);
void cam_trigger_toggle(void);

void buzzer_enable(void);
void buzzer_disable(void);
void buzzer_toggle(void);

void cutdown_enable(void);
void cutdown_disable(void);
void cutdown_toggle(void); 

#endif /* GPIO_H_ */