/*
 * CustomUserFunctions.c
 *
 *  Created on: Feb 16, 2020
 *      Author: Will Walecka and Kristine Chen
 */
//functions used for wifi connection and inter-device/communication
//carried out using commands to ISM chip

#include "rootca.h"
#include "httpclient.h"
#include "es_wifi_conf.h"
#include "es_wifi_io.h"
#include "es_wifi.h"
#include "http_lib.h"
#include "main.h"
#include "CustomUserFunctions.h"

#define NET_READ_TIMEOUT  "2000"
#define ES_WIFI_DATA_SIZE 1400
#define AT_OK_STRING "\r\nOK\r\n> "
#define AT_OK_STRING_LEN (sizeof(AT_OK_STRING) - 1)

#define AT_ERROR_STRING "\r\nERROR"

#define AT_DELIMETER_STRING "\r\n> "
#define AT_DELIMETER_LEN        4
#define NO_DATA 22

#define CHARISHEXNUM(x)                 (((x) >= '0' && (x) <= '9') || \
                                         ((x) >= 'a' && (x) <= 'f') || \
                                         ((x) >= 'A' && (x) <= 'F'))

#define CHARISNUM(x)                    ((x) >= '0' && (x) <= '9')
#define CHAR2NUM(x)                     ((x) - '0')




//Declarations
static uint8_t ComBuf[ES_WIFI_DATA_SIZE];
static uint8_t retData[ES_WIFI_DATA_SIZE];
long unsigned int UNIV_TIMEOUT = 10000;
static uint8_t TCPSocket = 0;
static uint16_t localPort = 8002;
#ifdef NODE_A //defined in header file
static uint16_t remotePort = 8002;
char *comCheck = "P?";
#endif
#ifndef NODE_A
char *comCheck = "C?";
#endif


//public user functions
void connect_nodes();
int sendTCPData(uint8_t *dataToSend, uint16_t Reqlen, uint16_t *SentLen);
static int sendATCommand();
static int sendATData(uint8_t *dataptr, uint16_t len);
static int receiveATData(uint16_t Reqlen, uint16_t *ReadData, uint8_t *arr_to_fill);
int beginClientServerConnection();
int receiveTCPData(uint16_t Reqlen, uint16_t *ReceivedLen, uint8_t *msg_arr);


//connect each node to wifi network (independent of A OR B) - taken from http library
void connect_nodes()
{
	httpclient_init();
}

//modified http library function - sends AT commands to ISM 43362 M3G to set (local) socket,
//set send timeout, set set data length, and read data to send from address in memory
//denoted by *dataToSend
int sendTCPData(uint8_t *dataToSend, uint16_t Reqlen, uint16_t *SentLen)
{
	ES_WIFI_Status_t ret = ES_WIFI_STATUS_ERROR;
	if(Reqlen >= ES_WIFI_PAYLOAD_SIZE )
		{
		Reqlen= ES_WIFI_PAYLOAD_SIZE;
		}

	*SentLen = Reqlen;
	sprintf((char*)ComBuf,"P0=%d\r", TCPSocket);
	ret = sendATCommand();
	if(ret == ES_WIFI_STATUS_OK)
	{
		sprintf((char*)ComBuf,"S2=%lu\r",UNIV_TIMEOUT);
		ret = sendATCommand();

		if(ret == ES_WIFI_STATUS_OK)
		{
			sprintf((char *)ComBuf,"S3=%d\r\r",Reqlen);
			ret = sendATData(dataToSend, Reqlen);

			if(ret == ES_WIFI_STATUS_OK)
			{
				if(strstr((char *)(&retData),"-1\r\n"))
				{
					ret = ES_WIFI_STATUS_ERROR;
				}
			}
		}
	}

	if (ret == ES_WIFI_STATUS_ERROR)
	{
		*SentLen = 0;
	}
	return ret;
}

int receiveTCPData(uint16_t Reqlen, uint16_t *ReceivedLen, uint8_t *msg_arr)
{
	  ES_WIFI_Status_t ret = ES_WIFI_STATUS_ERROR;

	  if(Reqlen <= ES_WIFI_PAYLOAD_SIZE )
	  {
		  sprintf((char*)ComBuf,"P0=%d\r", TCPSocket);
		  ret = sendATCommand();

		  if(ret == ES_WIFI_STATUS_OK)
		  {
			  sprintf((char*)ComBuf,"R1=%d\r", Reqlen);
			  ret = sendATCommand();
			  if(ret == ES_WIFI_STATUS_OK)
			  {
				  //poll for messages
				  sprintf((char*)ComBuf,"R2=%i\r", SHORT_TIMEOUT);
				  ret = sendATCommand();
				  if(ret == ES_WIFI_STATUS_OK)
				  {
					  sprintf((char*)ComBuf,"R0\r");
					  ret = receiveATData(Reqlen, ReceivedLen, msg_arr);
					  if (ret != ES_WIFI_STATUS_OK)
					  {
						  if (ret == NO_DATA)
						  {
							  return NO_DATA;
						  }
					  }
				  }
			  }
			  else
			  {
				  *ReceivedLen = 0;
			  }
		  }
	  }
	  return ret;
}


//flexible function that initializes board as either client or server, dependent
//on #define in CustomUserFunctions.h. significance of each AT command in comments below
int beginClientServerConnection()
{
	printf("opening client (A) /server (B) connection\r\n");
	ES_WIFI_Status_t ret = ES_WIFI_STATUS_OK;

	//set socket to 0
	sprintf((char*)(ComBuf),"P0=%d\r", TCPSocket);
	ret = sendATCommand();

	//set TCP protocol
	if (ret == ES_WIFI_STATUS_OK)
	{
		printf("set socket, setting TCP protocol\r\n");
		sprintf((char*)ComBuf,"P1=%d\r", 0);
		ret = sendATCommand();
	}

#ifdef NODE_A
	//set destination IP address (server address)
	if ((ret == ES_WIFI_STATUS_OK))
	{
		printf("set TCP protocol, setting destination IP address to %d.%d.%d.%d\r\n", nodeB_IPAddr[0],nodeB_IPAddr[1],
				nodeB_IPAddr[2],nodeB_IPAddr[3]);
		sprintf((char*)ComBuf,"P3=%d.%d.%d.%d\r", nodeB_IPAddr[0],nodeB_IPAddr[1],
				nodeB_IPAddr[2],nodeB_IPAddr[3]);
		ret = sendATCommand();
	}
	else
	{
		return 1;
	}

	//set remote port
	if ((ret == ES_WIFI_STATUS_OK))
	{
		printf("set dest IP addr, setting remote port to %d\r\n", remotePort);
		sprintf((char*)ComBuf,"P4=%d\r", remotePort);
		ret = sendATCommand();
	}
	else
	{
		return 1;
	}

	//show settings before connecting
	if (ret == ES_WIFI_STATUS_OK)
	{
		printf("showing transport settings before trying to connect\r\n");
		sprintf((char*)ComBuf,"%s\r", comCheck);
		ret = sendATCommand();
	}
	else
	{
		return 1;
	}

	//try to connect to server as client
	if (ret == ES_WIFI_STATUS_OK)
	{
		printf("set dest IP address, setting client mode\r\n");
		sprintf((char*)ComBuf,"P6=1\r");
		ret = sendATCommand();
	}
	else
	{
		return 1;
	}
#endif
#ifndef NODE_A

	//set local port
	if ((ret == ES_WIFI_STATUS_OK))
	{
		printf("set TCP protocol, setting local port to %d\r\n", localPort);
		sprintf((char*)ComBuf,"P2=%d\r", localPort);
		ret = sendATCommand();
	}

	//show settings before connecting
	if (ret == ES_WIFI_STATUS_OK)
	{
		printf("showing transport settings before trying to connect\r\n");
		sprintf((char*)ComBuf,"%s\r", comCheck);
		ret = sendATCommand();
	}

	//set server mode
	if (ret == ES_WIFI_STATUS_OK)
	{
		printf("set local port, setting server mode\r\n");
		sprintf((char*)ComBuf,"P5=1\r");
		ret = sendATCommand();
	}



#endif

	if (ret == ES_WIFI_STATUS_OK)
	{
		printf("server/client connection opened successfully\r\n");
	}

	return ret;
}

//modified httpclient code - sends AT command to ISM43362 M3G, assumes that
//proper command been placed in ComBuf before sending, writes wifi chip response into
//retData, which is checked for ACK
static int sendATCommand()
{
  int ret = 0;
  int16_t recv_len = 0;

  ret = SPI_WIFI_SendData(&ComBuf[0],  strlen((char*)(&ComBuf)), UNIV_TIMEOUT);

  if( ret > 0)
  {
	  recv_len = SPI_WIFI_ReceiveData(&retData[0], ES_WIFI_DATA_SIZE, UNIV_TIMEOUT);
    if((recv_len > 0) && (recv_len < ES_WIFI_DATA_SIZE))
    {
      *((&retData[0]) + recv_len) = 0;
      //receiving ACK from wifi module breaks out of function
      if(strstr((char *)(&retData), AT_OK_STRING))
      {
        return ES_WIFI_STATUS_OK;
      }
      else if(strstr((char *)(&retData), AT_ERROR_STRING))
      {
        return ES_WIFI_STATUS_UNEXPECTED_CLOSED_SOCKET;
      }
    }
    if (recv_len == ES_WIFI_ERROR_STUFFING_FOREVER )
    {
      return ES_WIFI_STATUS_MODULE_CRASH;
    }
  }
  return ES_WIFI_STATUS_IO_ERROR;
}

//modified function to send data to wifi chip via SPI3 for TCP message send
static int sendATData(uint8_t *dataptr, uint16_t len)
{
	int16_t send_len = 0;
	int16_t recv_len = 0;
	uint16_t cmd_len = 0;
	uint16_t n ;

	cmd_len = strlen((char*)(&ComBuf));

	//can send only even number of byte on first send
	//reasons for this are unclear
	if (cmd_len & 1) return ES_WIFI_STATUS_ERROR;
	n=SPI_WIFI_SendData(&ComBuf[0], cmd_len, UNIV_TIMEOUT);
	if (n == cmd_len)
	{
		send_len = SPI_WIFI_SendData(dataptr, len, UNIV_TIMEOUT);
		if (send_len == len)
		{
			recv_len = SPI_WIFI_ReceiveData(&retData[0], 0, UNIV_TIMEOUT);
			if (recv_len > 0)
			{
				*((&retData[0])+recv_len) = 0;
				//if ACK found in message returned, exit function, otherwise continue error checking
				if(strstr((char *)(&retData), AT_OK_STRING))
				{
					return ES_WIFI_STATUS_OK;
				}
				else if(strstr((char *)(&retData), AT_ERROR_STRING))
				{
					return ES_WIFI_STATUS_UNEXPECTED_CLOSED_SOCKET;
				}
				else
				{
					return ES_WIFI_STATUS_ERROR;
				}
			}
			if (recv_len == ES_WIFI_ERROR_STUFFING_FOREVER )
			{
				return ES_WIFI_STATUS_MODULE_CRASH;
			}
			return ES_WIFI_STATUS_ERROR;
		}
		else
		{
			return ES_WIFI_STATUS_ERROR;
		}
	}
	return ES_WIFI_STATUS_IO_ERROR;
}

//modified function to recieve data from ISM wifi chip via SPI3 after receive socket/length/timeout set
static int receiveATData(uint16_t Reqlen, uint16_t *ReadData, uint8_t *arr_to_fill)
{
	int len;
	int ret = ES_WIFI_STATUS_OK;
	uint8_t *p = &ComBuf;

	//sends AT command to get data and then checks for data received
	if(SPI_WIFI_SendData(&ComBuf[0], strlen((char*)(&ComBuf[0])), SHORT_TIMEOUT) > 0)
	{
		len = SPI_WIFI_ReceiveData(p, 0 , SHORT_TIMEOUT);
		if (len > 0)
		{
			if ((p[0]!='\r') || (p[1]!='\n'))
			{
				return  ES_WIFI_STATUS_IO_ERROR;
			}
			//read past newline and CR
			len-=2;
			p+=2;
			if (len >= AT_OK_STRING_LEN)
			{
				while(len && (p[len-1]==0x15)) len--;
				p[len] = '\0';
				if(strstr( (char*) p + len - AT_OK_STRING_LEN, AT_OK_STRING))
				{
					*ReadData = len - AT_OK_STRING_LEN;
					//special case where only data returned is ACK,
					if (*ReadData == 0)
					{
						ret = NO_DATA;
					}
					if (*ReadData > Reqlen)
					{
						*ReadData = Reqlen;
					}
					memcpy(arr_to_fill, p, *ReadData);
					return ret;
				}
				else if(memcmp((char *)p + len - AT_DELIMETER_LEN , AT_DELIMETER_STRING, AT_DELIMETER_LEN) == 0)
				{
					*ReadData = 0;
					return ES_WIFI_STATUS_UNEXPECTED_CLOSED_SOCKET;
				}

				*ReadData = 0;
				return ES_WIFI_STATUS_UNEXPECTED_CLOSED_SOCKET;
			}
			if (len == ES_WIFI_ERROR_STUFFING_FOREVER )
			{
				return ES_WIFI_STATUS_MODULE_CRASH;
			}
		}
		else
		{
			return NO_DATA; //TIMEOUT
		}
	}
	return ret;

}



