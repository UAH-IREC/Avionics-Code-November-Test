/*
* adc_sense.c
*
* Created: 5/18/2017 5:34:47 PM
*  Author: trb0023
*/ 
#include "drivers/adc_sense.h"

static double mapdouble(double x, double in_min, double in_max, double out_min, double out_max);

void adc_init(void)
{
	struct adc_config adc_conf;
	struct adc_channel_config adcch_conf_cap;
	struct adc_channel_config adcch_conf_sol;
	struct adc_channel_config adcch_conf_therm;

	adc_read_configuration(&VOLT_SENS, &adc_conf);
	adcch_read_configuration(&VOLT_SENS, CAP_SENS, &adcch_conf_cap);
	adcch_read_configuration(&VOLT_SENS, SOL_SENS, &adcch_conf_sol);
	adcch_read_configuration(&VOLT_SENS, THERM_SENS, &adcch_conf_therm);

	adc_set_conversion_parameters(&adc_conf, ADC_SIGN_ON, ADC_RES_12, ADC_REF_VCC);
	adc_set_conversion_trigger(&adc_conf, ADC_TRIG_MANUAL, 1, 0);
	adc_set_clock_rate(&adc_conf, 200000UL);

	adcch_set_input(&adcch_conf_cap, ADCCH_POS_PIN4, ADCCH_NEG_NONE, 1);
	adcch_set_input(&adcch_conf_sol, ADCCH_POS_PIN3, ADCCH_NEG_NONE, 1);
	adcch_set_input(&adcch_conf_therm, ADCCH_POS_PIN5, ADCCH_NEG_NONE, 1);

	adc_write_configuration(&VOLT_SENS, &adc_conf);
	adcch_write_configuration(&VOLT_SENS, CAP_SENS, &adcch_conf_cap);
	adcch_write_configuration(&VOLT_SENS, SOL_SENS, &adcch_conf_sol);
	adcch_write_configuration(&VOLT_SENS, THERM_SENS, &adcch_conf_therm);

	adc_enable(&VOLT_SENS);
}


static double mapdouble(double x, double in_min, double in_max, double out_min, double out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


double get_voltage_solar(uint8_t OSR)
{
	uint32_t sum = 0;

	for(uint8_t i = 0; i < OSR; i++)
	{
		adc_start_conversion(&VOLT_SENS, SOL_SENS);
		adc_wait_for_interrupt_flag(&VOLT_SENS, SOL_SENS);
		sum += adc_get_result(&VOLT_SENS, SOL_SENS);
	}
	
	double result = (double)sum/(double)OSR;
	
	double voltage = mapdouble((double)result, COUNT_MIN, COUNT_MAX, VREF_MIN, VREF_MAX);
	
	voltage = voltage/((2000.0)/(59000.0+2000.0));
	
	return voltage;
}

double get_voltage_capacitor(uint8_t OSR)
{
	uint32_t sum = 0;

	for(uint8_t i = 0; i < OSR; i++)
	{
		adc_start_conversion(&VOLT_SENS, CAP_SENS);
		adc_wait_for_interrupt_flag(&VOLT_SENS, CAP_SENS);
		sum += adc_get_result(&VOLT_SENS, CAP_SENS);
	}
	
	double result = (double)sum/(double)OSR;
	
	double voltage = mapdouble((double)result, COUNT_MIN, COUNT_MAX, VREF_MIN, VREF_MAX);

	voltage = voltage/((2000.0)/(10000.0+2000.0));

	return voltage;
}

double get_temperature_thermistor(uint8_t OSR)
{
	uint32_t sum = 0;

	for(uint8_t i = 0; i < OSR; i++)
	{
		adc_start_conversion(&VOLT_SENS, THERM_SENS);
		adc_wait_for_interrupt_flag(&VOLT_SENS, THERM_SENS);
		sum += adc_get_result(&VOLT_SENS, THERM_SENS);
	}
		
	double result = sum/OSR;
		
	double voltage = mapdouble((double)result, COUNT_MIN, COUNT_MAX, VREF_MIN, VREF_MAX);
	
	double resistance = 6490.0 * ((3.3/voltage)-1.0);

	double temperature = (4250.0/log(resistance/0.0645))-273.15;

	return temperature;
}

