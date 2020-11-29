/*
 * Hardware_Init.h
 *
 *  Created on: Feb 24, 2020
 *      Author: SupremeOverlord
 */

#ifndef HARDWARE_INIT_H_
#define HARDWARE_INIT_H_

void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_ADC1_Init(void);
void MX_TIM3_Init(void);
void MX_TIM15_Init(void);
void Console_UART_Init(void);

//******* EXTRA PERIPHS ADDED *********
void MX_ADC2_Init(void);
void MX_ADC3_Init(void);
void MX_TIM4_Init(void);
void MX_TIM16_Init(void);
//*******END EXTRA PERIPHS ADDED *********

#endif /* HARDWARE_INIT_H_ */
