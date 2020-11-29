/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "stm32l4xx_hal_uart.h"
#include "Hardware_Init.h"
#include "Analog_PWM_Functions.h"

//#include "../Stolen_Header_Files/httpclient.h"
//#include "../Stolen_Header_Files/CustomUserFunctions.h"
//#include "httpclient.h"
#include "CustomUserFunctions.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//#define B1_Pin GPIO_PIN_13
//#define B1_GPIO_Port GPIOC

/* USER CODE END PD */

//GLOBAL VARIABLES

//RTC_HandleTypeDef hrtc;
//RNG_HandleTypeDef hrng;
net_hnd_t         hnet; /* Is initialized by cloud_main(). */

#define USE_WIFI
#define HTTPCLIENT


//**************************MAIL/MSG QUEUE DEFS****************************

//MEMORY POOLS!!!
typedef struct {
	uint8_t Buf[32];
	uint8_t Idx;
} MEM_BLOCK_t;

osMessageQueueId_t wifiMsgQID;

osThreadId_t wifiMsgHandleHandle;
osThreadId_t wifiMsgGetHandle;
osThreadId_t servoDestHandleHandle;
osThreadId_t servoPosHandleHandle;
osMutexId_t useSPI3MutexHandle;
/* USER CODE BEGIN PV */

osThreadId_t defaultTask;
TIM_HandleTypeDef htim15;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim16;

//osPoolDef(MemPool, 8, MEM_BLOCK);

/* Private function prototypes -----------------------------------------------*/

void parseWifiMsg(void *argument);
void getWifiMsg(void *argument);
void setServoDestinations(void *argument);
void updateServoPositions(void *argument);
void startDefaultTask(void *argument);

//static void AllocMemoryPoolBlock (void);

//void SPI_WIFI_ISR(void);

/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	SystemClock_Config();

	/* USER CODE END Init */

	/* Configure the system clock */
	Periph_Config();
	BSP_LED_Init(LED_GREEN);
	BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);
	/* Initialize all configured peripherals */
	Console_UART_Init();
	printf("initialized UART\r\n");
	//connect to wifi network, begin connection

#ifdef WIFI_TIME
	connect_nodes();
#ifdef NODE_A
	int cnxnStatus;
	do
	{
	cnxnStatus = beginClientServerConnection();
	HAL_Delay(5000);
	} while ( cnxnStatus != 0);

#endif //NODE A

#ifndef NODE_A
	beginClientServerConnection();
#endif
#endif //WIFI TIME

	clearPosnHistory();

	MX_GPIO_Init();
	MX_ADC1_Init();
	MX_TIM3_Init();
	MX_TIM15_Init();

	//******* EXTRA PERIPHS ADDED *********
	  MX_ADC2_Init();
	  MX_ADC3_Init();
	  MX_TIM4_Init();
	  MX_TIM16_Init();
	//*******END EXTRA PERIPHS ADDED *********

	HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1); //
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3); //
	HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1); //

	setPulseBase(BASE_PULSE_MIN, 1);
	setPulseMid(MID_PULSE_MIN, 1);
	setPulseTop(TOP_PULSE_MIN, 1);

	/* USER CODE END 2 */

	osKernelInitialize();

	/* Create the mutex(es) */
	/* definition and creation of useSPI3Mutex */
	const osMutexAttr_t useSPI3Mutex_attributes = {
			.name = "useSPI3Mutex"
	};
	useSPI3MutexHandle = osMutexNew(&useSPI3Mutex_attributes);

	const osMessageQueueAttr_t wifiMsgQ_attributes = {
			.name = "wifiMsgQ"
	};
	wifiMsgQID = osMessageQueueNew (8, sizeof(ANALOG_CMD_t), &wifiMsgQ_attributes);

	/* definition and creation of wifiMsgHandle */
	const osThreadAttr_t wifiMsgHandle_attributes = {
			.name = "wifiMsgHandle",
			.priority = (osPriority_t) osPriorityNormal,
			.stack_size = 2000 //1000
	};
	wifiMsgHandleHandle = osThreadNew(parseWifiMsg, NULL, &wifiMsgHandle_attributes);

	/* definition and creation of wifiMsgHandle */
	const osThreadAttr_t wifiMsgGet_attributes = {
			.name = "wifiMsgGet",
			.priority = (osPriority_t) osPriorityNormal,
			.stack_size = 2000 //1000
	};
	wifiMsgGetHandle = osThreadNew(getWifiMsg, NULL, &wifiMsgGet_attributes);

	/* definition and creation of servoPosHandle */
	const osThreadAttr_t servoPosHandle_attributes = {
			.name = "servoPosHandle",
			.priority = (osPriority_t) osPriorityAboveNormal,
			.stack_size = 1200 //600
	};
	servoPosHandleHandle = osThreadNew(updateServoPositions, NULL, &servoPosHandle_attributes);

	/* definition and creation of defaultTask */
	const osThreadAttr_t defaultTask_attributes = {
			.name = "defaultTask",
			.priority = (osPriority_t) osPriorityBelowNormal,
			.stack_size = 128
	};
	defaultTask = osThreadNew(startDefaultTask, NULL, &defaultTask_attributes);


	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_3);


	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */
	while (1)
	{
		//:D
	}

}


/* USER CODE BEGIN 4 */
/*
static void AllocMemoryPoolBlock (void) {
	osPoolId MemPool_Id;
	MEM_BLOCK *addr;

	MemPool_Id = osPoolCreate(osPool(MemPool));
	if (MemPool_Id != NULL) {
		// allocate a memory block
		addr = (MEM_BLOCK *)osPoolAlloc(MemPool_Id);

		if (addr != NULL) {
			// memory block was allocated
		}
	}
}
*/






void SPI3_IRQHandler(void)
{
	HAL_SPI_IRQHandler(&hspi);
}


/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	while(1)
	{
		BSP_LED_Toggle(LED_GREEN);
		HAL_Delay(200);
	}
}


#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(char *file, uint32_t line)
{ 
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
