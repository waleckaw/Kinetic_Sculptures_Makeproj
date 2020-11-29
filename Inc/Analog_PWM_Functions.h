/*
 * Analog_PWM_Functions.h
 *
 *  Created on: Mar 9, 2020
 *      Author: SupremeOverlord
 */

#ifndef ANALOG_PWM_FUNCTIONS_H_
#define ANALOG_PWM_FUNCTIONS_H_

#include <stdint.h>
#include "main.h"

//public functions
//uint16_t getAnalogVal2(void);
//uint16_t getAnalogVal3(void);
void setDuty(int duty);

void setPulseBase(int pulse, bool ON);
void setPulseMid(int pulse, bool ON);
void setPulseTop(int pulse, bool ON);
void setPulse(int pulse, bool ON, uint8_t link);
uint16_t getAnalogVal(ADC_HandleTypeDef *ADC);
int analogToPulse(int analogVal, TIM_HandleTypeDef *channel);
int analogToDegrees(int analogVal);
int degreesToDC(int deg);
int degToPulse(int deg);

#endif /* ANALOG_PWM_FUNCTIONS_H_ */
