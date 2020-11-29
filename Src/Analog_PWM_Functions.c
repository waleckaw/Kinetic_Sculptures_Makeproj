/*
 * Analog_PWM_Functions.c
 *
 *  Created on: Mar 9, 2020
 *      Author: SupremeOverlord
 */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "CustomUserfunctions.h"
#include "httpclient.h"
#include "http_lib.h"
#include "es_wifi.h"
#include <stdint.h>

//module defines
#define ANALOG_MIN 700
#define ANALOG_MAX 2500

//module declarations
extern TIM_HandleTypeDef htim15;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim16;


//public functions
void setDuty(int duty);
int analogToDegrees(int analogVal);
int degreesToDC(int deg);
int degToPulse(int deg);

//private module functions
static int map(int x, int in_min, int in_max, int out_min, int out_max);

//DISCONTINUED - LOWER CONTROL RESOLUTION THAN SETTING PULSE WITH
void setDuty(int duty)
{
	int MAX_PERIOD = __HAL_TIM_GET_AUTORELOAD(&htim15);
	if (duty <= 0) {
		TIM15->CCR1 = 0;
		duty = 0;
	} else if (duty >= 100) {
		TIM15->CCR1 = __HAL_TIM_GET_AUTORELOAD(&htim15);
		duty = 100;
	} else {
		TIM15->CCR1 = (MAX_PERIOD*duty)/100;
	}
	TIM15->CCMR1 |= TIM_CCMR1_OC1PE;
}

//set pulse to base servo, or turn servo off
void setPulseBase(int pulse, bool ON)
{
	if (pulse <= BASE_PULSE_MIN) {
		TIM15->CCR1 = BASE_PULSE_MIN;
	} else if (pulse >= BASE_PULSE_MAX) {
		TIM15->CCR1 = BASE_PULSE_MAX;
	} else {
		TIM15->CCR1 = pulse;
	}
	if (!ON) {
		TIM15->CCR1 = 0;
	}

	TIM15->CCMR1 |= TIM_CCMR1_OC1PE;
}

//set pulse to middle tier servo, or turn servo off
void setPulseMid(int pulse, bool ON)
{
	if (pulse <= MID_PULSE_MIN) {
		TIM4->CCR3 = MID_PULSE_MIN;
	} else if (pulse >= MID_PULSE_MAX) {
		TIM4->CCR3 = MID_PULSE_MAX;
	} else {
		TIM4->CCR3 = pulse;
	}
	if (!ON) {
		TIM4->CCR3 = 0;
	}

	TIM4->CCMR3 |= TIM_CCMR1_OC1PE;
}

//set pulse to top servo, or turn servo off
void setPulseTop(int pulse, bool ON)
{
	if (pulse <= TOP_PULSE_MIN) {
		TIM16->CCR1 = TOP_PULSE_MIN;
		//pulse = 200;
	} else if (pulse >= TOP_PULSE_MAX) {
		TIM16->CCR1 = TOP_PULSE_MAX;
	} else {
		TIM16->CCR1 = pulse;
	}
	if (!ON) {
		TIM16->CCR1 = 0;
	}

	TIM16->CCMR1 |= TIM_CCMR1_OC1PE;
}

void setPulse(int pulse, bool ON, uint8_t link)
{

	if (link == 0) {
		setPulseBase(pulse, ON);
	} else if (link == 1) {
		setPulseMid(pulse, ON);
	} else if (link == 2) {
		setPulseTop(pulse, ON);
	}
}

//get analog value from desired ADC (base = &hadc1, mid = &hadc2, top = &hadc3)
uint16_t getAnalogVal(ADC_HandleTypeDef *ADC)
{
	HAL_ADC_Start(ADC);
	HAL_ADC_PollForConversion(ADC, 5);
	return HAL_ADC_GetValue(ADC);
}

//basic scaling function given input + input & output bounds
static int map(int x, int in_min, int in_max, int out_min, int out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//return mapped pulse width for desired PWM channel
//total period is 5 ms, max and min pulse widths vary between different servos
int analogToPulse(int analogVal, TIM_HandleTypeDef *channel)
{
	if (channel == &htim15) {
		if (analogVal < BASE_ANALOG_MIN) {
			analogVal = BASE_ANALOG_MIN;
		} else if (analogVal > BASE_ANALOG_MAX) {
			analogVal = BASE_ANALOG_MAX;
		}
		return map(analogVal, BASE_ANALOG_MIN, BASE_ANALOG_MAX, BASE_PULSE_MIN, BASE_PULSE_MAX);
	} else if (channel == &htim4) {
		if (analogVal < MID_ANALOG_MIN) {
			analogVal = MID_ANALOG_MIN;
		} else if (analogVal > MID_ANALOG_MAX) {
			analogVal = MID_ANALOG_MAX;
		}
		return map(analogVal, MID_ANALOG_MIN, MID_ANALOG_MAX, MID_PULSE_MIN, MID_PULSE_MAX);
	} else if (channel == &htim16) {
		if (analogVal < TOP_ANALOG_MIN) {
			analogVal = TOP_ANALOG_MIN;
		} else if (analogVal > TOP_ANALOG_MAX) {
			analogVal = TOP_ANALOG_MAX;
		}
		return map(analogVal, TOP_ANALOG_MIN, TOP_ANALOG_MAX, TOP_PULSE_MIN, TOP_PULSE_MAX);
	}
	else {
		return 0;
	}
}

//DISCONTINUED - FOR USE IN DEGREES MODE (TOO LOW RES)
int analogToDegrees(int analogVal)
{
	return map(analogVal, ANALOG_MIN, ANALOG_MAX, 0, 180);
}

//DISCONTINUED - FOR USE IN DEGREES MODE (TOO LOW RES)

int degreesToDC(int deg)
{
	return map(deg, 0, 180, 0, 98);
}

//DISCONTINUED - FOR USE IN DEGREES MODE (TOO LOW RES)
int degToPulse(int deg)
{
	int MAX_PERIOD = __HAL_TIM_GET_AUTORELOAD(&htim15);
	return map(deg, 0, 180, 50, MAX_PERIOD);
}
