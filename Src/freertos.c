/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : tasks and auxilary functions required for device operation
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

#include "Analog_PWM_Functions.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "CustomUserfunctions.h"
#include "httpclient.h"
#include "http_lib.h"
#include "es_wifi.h"
#include <stdlib.h>
#include "ring_buf.h"

/* Private typedef -----------------------------------------------------------*/

// typedef struct {
// 	uint16_t servoVal1;
// 	uint16_t servoVal2;
// 	uint16_t servoVal3;
// } avgValArray;


/* Private defines ------------------------------------------------------------*/

#define ES_WIFI_DATA_SIZE 1400

//SERVO CONTROL GAINS
#define K_GAIN_BASE 0.2
#define D_GAIN_BASE 0.04

#define K_GAIN_MID 0.15
#define D_GAIN_MID 0.04

#define K_GAIN_TOP 0.2
#define D_GAIN_TOP 0.04

//TASK TIMING VALUES
#define TIME_MAG 10
#define GET_MSG_TIME TIME_MAG*20
#define PARSE_MSG_TIME TIME_MAG*20
#define UPDATE_TIME (TIME_MAG*3)/2

//STATE VALUES

typedef enum {
	MASTER,
	NEUTRAL,
	SLAVE
} sculpture_status;

#define MASTER 1
#define NEUTRAL 2
#define SLAVE 3

/* Private extern typdefs ------------------------------------------------------------*/

//External Peripherals and Functions
extern osMessageQId wifiMsgQID;
extern osMutexId_t useSPI3MutexHandle;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;
extern TIM_HandleTypeDef htim15;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim16;
extern void SPI_WIFI_ISR();
static ADC_HandleTypeDef *ADC_Array[] = {&hadc1, &hadc2, &hadc3};
static TIM_HandleTypeDef *PWM_Array[] = {&htim15, &htim4, &htim16};

/* Private variable declarations ------------------------------------------------------------*/

//structs containing analog position servo values for both sculptures
static analogPosnArray posnAvgForUpdate;
static analogPosnArray analogDestPosns;
static analogPosnArray currentAnalogPosns;

// static analogPosnArray posnHistory[8];
static ring_buffer posnHistory = {.numFilled = 0, .size = 8, .head = 0, .tail = 0};
// static uint8_t valuesAdded = 0;

//ARRAYS READ FROM AND WRITTEN TO BY WIFI FUNCTIONS
uint8_t msgToSend[8];
static uint8_t retData[10];
//values manipulated by wifi sending and receiving function
static uint16_t msgSentLen = 0;
static uint16_t msgReadLen = 0;

//ISR VARIABLES FOR DERIVATIVE TERM
static int isrLastPosErr[NUM_SERVOS];
static int lastPosErr[NUM_SERVOS];
static int posErr[NUM_SERVOS];
static int totalErr[NUM_SERVOS];
static int ctrlOut[NUM_SERVOS];

//STATE VARIABLES
//1 = master, 2 = neutral, 3 = slave
static sculpture_status masterStatus = NEUTRAL;
static bool wifiConnected = false;
static bool lastMessage = false;

static volatile uint8_t button_flags = 0;

/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/

// static void shiftAndAdd(void);
static void calcCurrentPos(analogPosnArray *ptr);
static void Button_ISR(void);
static void updatePosnArray(analogPosnArray* ptr, ANALOG_CMD_t message);
static void lock_servos(void);
static analogPosnArray getCurrAnalogPosns(void); 

/* Private application code --------------------------------------------------*/

//default low priority task that runs while all other other tasks are inactive
void startDefaultTask(void *argument)
{
	for (;;)
	{
		osDelay(1);
	}
}

//parses messages in wifiMsgQ and responds according to current state. If master,
//also responsible for sending messages
void parseWifiMsg(void *argument) 	//AND SEND!!!
{
	ANALOG_CMD_t msg;
	osStatus_t status;
	osStatus_t mutexStatus;
	uint16_t msgSize = sizeof(msgToSend);
	analogPosnArray posnAvgForUpdate;

	for(;;)
	{
		//if there's a message in the queue, process it
		status = osMessageQueueGet(wifiMsgQID, &msg, 0, 0);
		sculpture_status parseTaskMasterStatus = masterStatus;
		bool parseTaskLastMessage = lastMessage;

		//read the message in the queue and if (master), fill the commands
		if (status == osOK) {
			//other sculpture has taken over as master and you are still neutral
			if ((msg.master == MASTER) && (parseTaskMasterStatus == NEUTRAL)) {
				//enter SLAVE state
				masterStatus = SLAVE;

				//turn yellow LED off
				HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_3);
				//turn RED LED on
				HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);

				//update servo destination positions based on data bytes from message
				updatePosnArray(&analogDestPosns, msg);

			} else if (parseTaskMasterStatus == SLAVE) {
				//last message from the master
				if (msg.master == LAST_MESSAGE) {
					//turn yellow LED on
					HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_3);
					//turn RED LED off
					HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);

					//set state to NEUTRAL
					masterStatus = NEUTRAL;

					//lock all servos in current positions
					lock_servos();
				} else {
					//SLAVE state, read message and update servo destination positions
					updatePosnArray(&analogDestPosns, msg);
				}
			}
		}

		//every time you enter task, after checking message queue...
		if (parseTaskMasterStatus == MASTER) {

			if (parseTaskLastMessage) {
				//set first byte with special LAST MESSAGE denotation
				msgToSend[0] = LAST_MESSAGE;
				//clear lastMessage state variable
				lastMessage = false;
				//re-enter NEUTRAL state
				masterStatus = NEUTRAL;
				//lock all servos to current positions
				lock_servos();
			} else { //if you are set to master, calc and send your positions
				//get averaged current position to eliminate noise
				calcCurrentPos(&posnAvgForUpdate);
				//set msgToSend state byte to MASTER
				msgToSend[0] = MASTER;
				//set msgToSend data bytes to current average analog servo position values
				int k=1;
				for (int i=0; i<NUM_SERVOS; i++) {
					msgToSend[k++] = posnAvgForUpdate.servoVals[i] >> 8;
					msgToSend[k++] = posnAvgForUpdate.servoVals[i] & 0xff;
				}
			}

			//toggle ORANGE/BLUE LED
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9);

			//acquire SPI mutex and send message to SLAVE sculpture
			mutexStatus = osMutexAcquire(useSPI3MutexHandle, osWaitForever);
			if (mutexStatus == osOK)
			{
				sendTCPData(msgToSend, msgSize, &msgSentLen);
				osMutexRelease(useSPI3MutexHandle);
			}
		}
				
		osDelay(PARSE_MSG_TIME);
	}
}

//intermittently polls for incoming message from other node. If this message is a sculpture
//command, it adds it into  wifiMsgQ
void getWifiMsg(void *argument)
{
	ANALOG_CMD_t msg;
	uint16_t msgSize = sizeof(retData);
	osStatus_t mutexStatus;
	static uint8_t recStatus;
	uint32_t queueTimeout;

	for(;;)
	{
		//grab current master status
		uint8_t getTaskMasterStatus = masterStatus;

		//if SLAVE or NEUTRAL state
		if (getTaskMasterStatus != MASTER)
		{
			//toggle BLUE/ORANGE LED
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9);
			//get SPI mutex... should return almost instantly
			mutexStatus = osMutexAcquire(useSPI3MutexHandle, osWaitForever);
			if (mutexStatus == osOK)
			{
				//poll for TCP message
				recStatus = receiveTCPData(msgSize, &msgReadLen, retData);
				osMutexRelease(useSPI3MutexHandle);
				//create wifiMsgQ message
				if (recStatus == 0)
				{
					msg.master = retData[1];
					msg.analogCmdVal1a = retData[2];
					msg.analogCmdVal1b = retData[3];
					msg.analogCmdVal2a = retData[4];
					msg.analogCmdVal2b = retData[5];
					msg.analogCmdVal3a = retData[6];
					msg.analogCmdVal3b = retData[7];

					//if message signifies a state change, make sure to process it
					//otherwise, put destinations in if there's room in the queue
					if (msg.master == LAST_MESSAGE || msg.master == MASTER) {
						queueTimeout = osWaitForever;
					} else {
						queueTimeout = 0;
					}

					//enqueue message
					osMessageQueuePut(wifiMsgQID, &msg, 0, queueTimeout); //attempt to put message in
				}
			}
		}
		osDelay(GET_MSG_TIME);
	}
}

//reads analog value of each servo and updates currentAnalogPosns accordingly. highest
//priority task
void updateServoPositions(void *argument)
{
	//after entering this task, wifiConnected set so that pushbutton use changes state
	wifiConnected = true;
	for(;;)
	{
		//analog read values from each servo
		currentAnalogPosns = getCurrAnalogPosns();
		if (posnHistory.numFilled == 0) {
			posnHistory.vals[0] = currentAnalogPosns;
			posnHistory.numFilled = 1;
		} else {
			addToRingBuf(&posnHistory, currentAnalogPosns);
		}

		// shiftAndAdd();
		osDelay(UPDATE_TIME);
	}
	/* USER CODE END updateServoPositions */
}

//function called by buttonISR. if MASTER sends to NEUTRAL and vice versa. If causing
//NEUTRAL -> MASTER transition, unlocks servos on MASTER sculpture
void toggleMasterStatus()
{
	if (masterStatus == MASTER)
	{
		lastMessage = true;
		//turn yellow on
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_3);
		//turn GREEN off
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
	}
	else if (masterStatus == NEUTRAL)
	{
		masterStatus = MASTER;
		//turn yellow off
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_3);
		//turn green on
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);

		//unlock servos (set pulse to zero)
		for (int i=0; i<NUM_SERVOS; i++) {
			setPulse(0, false, i);
		}
	}
}

//function that takes the average of the most recently added analog position values for each servo
//and uses this value to update posnAvgForUpdate
static void calcCurrentPos(analogPosnArray *ptr)
{
	//account for case where values added is zero
	if (posnHistory.numFilled == 0) {
		(*ptr) = getCurrAnalogPosns();
	} else {
		for (int j=0; j<NUM_SERVOS; j++) {
			ptr->servoVals[j] = 0;
		}

		for (int i = 0; i<posnHistory.numFilled; i++) {
			for (int j=0; j<NUM_SERVOS; j++) {
				ptr->servoVals[j] += ((analogPosnArray)getRingBufXRecent(&posnHistory, i)).servoVals[j];
			}
		}
		for (int j=0; j<NUM_SERVOS; j++) {
			ptr->servoVals[j] /= posnHistory.numFilled;
		}
	}
}

static uint16_t calcSinglePos(uint8_t link) {

	uint16_t servoPosn = 0;
		//account for case where values added is zero
	if (posnHistory.numFilled == 0) {
		servoPosn = getAnalogVal(ADC_Array[link]);
	} else {
		for (int i = 0; i<posnHistory.numFilled; i++) {
			//add all of the values accumulated
			servoPosn += posnHistory.vals[i].servoVals[link];
		}
		//take the average for each servo
		servoPosn /= posnHistory.numFilled;
	}
	//set posnAvgForUpdate (since we went to the trouble of calculating one link)
	posnAvgForUpdate.servoVals[link] = servoPosn;
	return servoPosn;
}

//********************************TIMER OVERFLOW ISR************************************

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	//increase global variable uwTick - Hardware Abstraction Layer runs on timer 2
	if (htim->Instance == TIM2) {
		HAL_IncTick();
	}
	//control law/servo output pulse setISR
	if (htim->Instance == TIM3)
	{
		if (masterStatus == SLAVE) //SHOULD BE SLAVE HERE
		{

			//if the user desires, the code below can be used to simply set the SLAVE's servos to
			//exactly mimic those of the master, with no controls
#ifndef USE_CONTROL
			int pulseVal;

			for (int i=0; i<NUM_SERVOS; i++) {
				pulseVal = analogToPulse(analogDestPosns.servoVals[i], PWM_Array[i]);
				setPulse(pulseVal, true, i);
			}

#endif //NOT USE_CONTROL

			//the default mode, which makes for much smoother tracking, is to use PD control
			//to track from the current position to the destination position (for each servo)
			//analog position values are used to calculate error, which is later mapped to
			//pulse before being output
#ifdef USE_CONTROL

			for (int i=0; i<NUM_SERVOS; i++) {
				uint16_t posn = calcSinglePos(i);
				// isrLastPosErr[i] = lastPosErr[i];
				posErr[i] = analogDestPosns.servoVals[i] - posn;
				totalErr[i] = posn + (K_GAIN_BASE*posErr[i]) + D_GAIN_BASE*(posErr[i] - lastPosErr[i]);
				setPulse(analogToPulse(totalErr[i], PWM_Array[i]), true, i);
				lastPosErr[i] = posErr[i];

			}

#endif //USE_CONTROL

		}
	}
}


//external interrupt line detection ISR. Services SPI and pushbutton
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch (GPIO_Pin)
	{
	case (GPIO_PIN_13):
    	{
		Button_ISR();
		break;
    	}
	case (GPIO_PIN_1):
		{
		SPI_WIFI_ISR();
		break;
		}

	default:
	{
		break;
	}
	}
}

//during wifi initialization/connection, increments button_flags
//after connection, is used to enter and exit master state
static void Button_ISR(void)
{
	button_flags++;
	if (wifiConnected)
	{
		toggleMasterStatus();
	}
}

//waits for button to be pushed in order to determine whether user
//wants to change wifi settings
uint8_t Button_WaitForPush(uint32_t delay)
{
	uint32_t time_out = HAL_GetTick()+delay;
	do
	{
		if (button_flags > 1)
		{
			button_flags = 0;
			return BP_MULTIPLE_PUSH;
		}

		if (button_flags == 1)
		{
			button_flags = 0;
			return BP_SINGLE_PUSH;
		}
	}
	while( HAL_GetTick() < time_out);
	return BP_NOT_PUSHED;
}

static void updatePosnArray(analogPosnArray* ptr, ANALOG_CMD_t message) {

	ptr->servoVals[0] = (message.analogCmdVal1a << 8) | message.analogCmdVal1b;
	ptr->servoVals[1] = (message.analogCmdVal2a << 8) | message.analogCmdVal2b;
	ptr->servoVals[2] = (message.analogCmdVal3a << 8) | message.analogCmdVal3b;

}

void clearPosnHistory(void) {
	resetRingBuf(&posnHistory);
}

static void lock_servos(void) {
	int mypulse;
	for (int i=0; i<NUM_SERVOS; i++) {
		mypulse = analogToPulse(currentAnalogPosns.servoVals[i], PWM_Array[i]);
		setPulse(mypulse, true, i);
	}

}

static analogPosnArray getCurrAnalogPosns(void) {

	analogPosnArray arr;

	for (int i=0; i<NUM_SERVOS; i++) {
		arr.servoVals[i] = getAnalogVal(ADC_Array[i]);
	}

	return arr;

}



/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
