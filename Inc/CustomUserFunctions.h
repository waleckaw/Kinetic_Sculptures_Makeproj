/*
 * CustomUserFunctions.h
 *
 *  Created on: Feb 16, 2020
 *      Author: SupremeOverlord
 */

#ifndef APPLICATION_USER_CUSTOMUSERFUNCTIONS_H_
#define APPLICATION_USER_CUSTOMUSERFUNCTIONS_H_

#define NODE_A
#define WIFI_TIME

typedef struct {
	uint8_t master;
	uint8_t analogCmdVal1a;
	uint8_t analogCmdVal1b;
	uint8_t analogCmdVal2a;
	uint8_t analogCmdVal2b;
	uint8_t analogCmdVal3a;
	uint8_t analogCmdVal3b;
} ANALOG_CMD_t;

typedef struct {
	uint16_t servoVals[3];
} analogPosnArray;

//#define TEST_MODE

#define DUPLEX

#define SHORT_TIMEOUT 200

#define USE_CONTROL

#ifdef NODE_A

#define BASE_PULSE_MIN 110
//#define BASE_PULSE_MAX 500
#define BASE_PULSE_MAX 480
#define BASE_ANALOG_MIN 640
#define BASE_ANALOG_MAX 2800

#define MID_PULSE_MIN 90
//#define MID_PULSE_MAX 440
#define MID_PULSE_MAX 420 //NEW MONDAY GAINS
#define MID_ANALOG_MIN 590
#define MID_ANALOG_MAX 2870

#define TOP_PULSE_MIN 90
#define TOP_PULSE_MAX 470
#define TOP_ANALOG_MIN 540
#define TOP_ANALOG_MAX 2835

#endif //NODE_A
#ifndef NODE_A

#define BASE_PULSE_MIN 110
//#define BASE_PULSE_MAX 510 //worked saturday nigth
#define BASE_PULSE_MAX 460
#define BASE_ANALOG_MIN 630
#define BASE_ANALOG_MAX 2825

//#define MID_PULSE_MIN 100
#define MID_PULSE_MIN 95
//#define MID_PULSE_MAX 470
#define MID_PULSE_MAX 440
#define MID_ANALOG_MIN 620
#define MID_ANALOG_MAX 2825

#define TOP_PULSE_MIN 100
#define TOP_PULSE_MAX 450
#define TOP_ANALOG_MIN 590
#define TOP_ANALOG_MAX 2650

#endif //NOT NODE_A

#define NUM_SERVOS 3
#define LAST_MESSAGE 0xff

static uint8_t nodeA_IPAddr[4] = {192, 168, 4, 10};
static uint8_t nodeB_IPAddr[4] = {192, 168, 4, 11};

int beginClientServerConnection();
//void connect_nodes(void const *arg);
void connect_nodes(void);
int sendTCPData(uint8_t *dataToSend, uint16_t Reqlen, uint16_t *SentLen);
int receiveTCPData(uint16_t Reqlen, uint16_t *ReceivedLen, uint8_t *msg_arr);
void clearPosnHistory(void);

#endif /* APPLICATION_USER_CUSTOMUSERFUNCTIONS_H_ */
