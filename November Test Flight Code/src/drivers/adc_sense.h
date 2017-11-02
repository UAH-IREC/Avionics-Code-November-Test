/*
 * adc_sense.h
 *
 * Created: 5/18/2017 5:35:16 PM
 *  Author: trb0023
 */ 

#include <asf.h>
#include <math.h>

#ifndef ADC_SENSE_H_
#define ADC_SENSE_H_

#define VOLT_SENS    ADCB
#define CAP_SENS ADC_CH0
#define SOL_SENS ADC_CH1
#define THERM_SENS ADC_CH2

#define COUNT_MIN 0.0
#define COUNT_MAX 2047.0
#define VREF_MIN 0.0
#define VREF_MAX 2.0625

void adc_init(void);

double get_voltage_solar(uint8_t OSR);
double get_voltage_capacitor(uint8_t OSR);
double get_temperature_thermistor(uint8_t OSR);

#endif /*ADC_SENSE_H_*/