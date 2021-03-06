/******************** (C) COPYRIGHT 2015 Atom Corporation *********************
* File Name          : ATOM_PROTOCOL.h
* Author             : Stephen
* Date First Issued  : 2016.10.20
* Description        : 
********************************************************************************
* History:

* 2016.10.20: V0.1

********************************************************************************
* THE PRESENT SOFTWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/
#include "com.h"
#include "stdio.h"
#include "string.h"
#include "atom_macro.h"
#include "atomprotocol.h"
#include "main.h"
#include <unistd.h> 


#define COMPORT_COMMUNICATION
#define DMP_OUTPUT

#define THREADCOUNT 1

SaberData_HandleType saberDataHandle;


unsigned short usLength = 0, usRxLength = 0;
long long gValidCount = 0;

int resendCounter;
unsigned char * pBufferStart = NULL, *pCustomer = NULL;
unsigned char *pProducer = NULL, *pCurrent = NULL, *pBufferEnd = NULL;
unsigned char * p;
unsigned char chr[RING_BUFFER_SIZE * 2];
unsigned char chrBuffer[RING_BUFFER_SIZE];
u8 ringBuf[RING_BUFFER_SIZE];
u8 tempbuf[RING_BUFFER_SIZE] = { '\0' };
extern unsigned short CollectUARTData(char chrUARTBufferOutput[]);
unsigned char sum = 0;
char ucComNo[2] = { 0, 0 };


u32 unProcessBytes, previous_bytes, processedBytes;

int count_L;
int points = 0;
u8 receiveAck;

extern int dataReady;
extern int Receive_count;
extern void Process();
extern void DataPacketParser(u8 *pBuffer, u16 dataLen);
int timeout_count = 0;
u8 reTx_flag = 0;
u8 errorCode_ack;
u8 rcid;
u8 rmid;


char waitAck(void (*pFunction)(char x),u8 cid,u8 mid,char parameter)
{

	usleep(100);
	ReceiveData(cid, mid);
	while (1)
	{

		if((cid == rcid) && (mid == rmid)&& (receiveAck == 1) )
			break;
#if 1
		reTx_flag++;
		/*------------------Timeout processing--------------*/
		if (reTx_flag == 20)	;										//Retransmission after 2s
	
		{

			reTx_flag = 0;
			pFunction(parameter);
			timeout_count++;
			if (timeout_count == 4)										//Unable to receive after 6 retransmissions
			{
				return 0;
			}

		}
#endif	
		usleep(100);
		ReceiveData(cid, mid);

	
		
	}	
	if (errorCode_ack != 0)
	{
		/*---------User Code Begin-------------*/
		//errorCode processing
		//return 0 or 1?
		/*---------User Code End-------------*/
	}

	receiveAck = 0;
	timeout_count = 0;

	return 1;
}
int main()
{
	FILE *fp;
	u8 ret = 0;
	pBufferStart = (unsigned char *)ringBuf;
	pBufferEnd = pBufferStart + RING_BUFFER_SIZE;
	pCustomer = pBufferStart;
	pProducer = pBufferStart;


	// Create COMthread
	//OpenCom();
	ATR_Linux_OpenCom("/dev/ttyUSB0",460800);
	/****************************** Switch to config mode ********************************************/
	Atom_switchModeReq(CONFIG_MODE);

	ret = waitAck(Atom_switchModeReq, CLASS_ID_OPERATION_CMD, CMD_ID_SWITCH_TO_CFG_MODE, CONFIG_MODE);
	if (ret == 0)
	{
		return 1;
	}

	/****************************** Switch to config mode ********************************************/

	
	/****************************** Select packet***************************************************/
	// Enable packets,  use SET_BIT to enable packet and use CLEAR_BIT to disable packet.
	int bitEnable = 0;
	/*-------------------Uesr Code Begin------------------*/
	//Users can set the packet according to their own needs
	SET_BIT(bitEnable, PACKET_CAL_ACC);
	//SET_BIT(bitEnable, PACKET_CAL_ACC);
	SET_BIT(bitEnable, PACKET_CAL_GYRO);
	//SET_BIT(bitEnable, PACKET_CAL_MAG);	
	SET_BIT(bitEnable, PACKET_EULER_DATA);
	//SET_BIT(bitEnable, PACKET_EULER_DATA);
	//CLEAR_BIT(bitEnable, PACKET_QUAT_DATA);
	//SET_BIT(bitEnable, PACKET_QUAT_DATA);

	/*-------------------Uesr Code End-------------------*/

	SelectPackets(bitEnable);

	ret = waitAck(SelectPackets, CLASS_ID_HOSTCONTROL_CMD, CMD_ID_SET_DATA_PAC_CFG, bitEnable);
	if (ret == 0)
	{
		return 1;
	}
	/****************************** Select packet***************************************************************/

	/****************************** Set host update rate***************************************************************/
	SetHostUpdateRate(RATE_400);

	ret = waitAck(SetHostUpdateRate, CLASS_ID_ALGORITHMENGINE, CMD_ID_SET_PACKET_UPDATE_RATE, RATE_400);
	if (ret == 0)
	{
		return 1;
	}
	/****************************** Set host update rate***************************************************************/

	/****************************** Switch To measure mode***************************************************************/
	Atom_switchModeReq(MEASURE_MODE);

	ret = waitAck(Atom_switchModeReq, CLASS_ID_OPERATION_CMD, CMD_ID_SWITCH_TO_MEASURE_MODE, MEASURE_MODE);
	if (ret == 0)
	{
		return 1;
	}
	/****************************** Switch To measure mode***************************************************************/

	ReceiveData(0x06,0x81);
	
	getchar();
	return 0;
}


int print_flag = 0;


void Process()
{
	u8 * pCurrentHead = NULL;
	u8 * pCurrent = NULL;
	u8 CompletePackage = 0;
	u8 * pEndRing = NULL;
	u8 * pStartRing = NULL;
	int ret = 0;
	pEndRing = (u8*)(&ringBuf) + RING_BUFFER_SIZE;
	pStartRing = (u8*)&ringBuf;
	u32 needProcessedLen = 0;
	if (print_flag)
	{
		printf("\n======================================================================================================================\n");
		//printf("%s(),pStartRing = %p, pEndRing  = %p bufferSize = %d \n", __FUNCTION__, pStartRing, pEndRing, pEndRing - pStartRing);
	}
	//copy ringbuff to tempbuf
	if (pProducer >= pCustomer)
	{

		unProcessBytes = (u32)(pProducer - pCustomer);
		if (print_flag)
			printf("%s(), pCustomer = %p, pProducer = %p, unProcessBytes = %d \n", __FUNCTION__, pCustomer, pProducer, unProcessBytes);
		if (unProcessBytes <= sizeof(tempbuf))
			memcpy((void*)&tempbuf, pCustomer, unProcessBytes);

	}
	else
	{

		unProcessBytes = (u32)(RING_BUFFER_SIZE - (pCustomer - pProducer));
		if (print_flag)
			printf("%s(),pProducer  = %p, pCustomer = %p, unProcessBytes = %d \n", __FUNCTION__, pProducer, pCustomer, unProcessBytes);
		if ((u32)(pEndRing - pCustomer) <= sizeof(tempbuf))
			memcpy((void*)&tempbuf, (void*)pCustomer, pEndRing - pCustomer);

		if ((u32)(pProducer - pStartRing) <= (sizeof(tempbuf) - (pEndRing - pCustomer)))
			memcpy((void*)((u8*)&tempbuf + (pEndRing - pCustomer)), (void*)pStartRing, pProducer - pStartRing);

	}

	
	if (unProcessBytes < FRAME_MIN_SIZE)
	{
		return;
	}

	pCurrentHead = (u8*)&tempbuf;

	while ((unProcessBytes >= FRAME_MIN_SIZE) && (ret != ERROR_NOT_ENOUGH_LENGTH))
	{
		previous_bytes = unProcessBytes;
		if (unProcessBytes >= ATOM_HEADER_LEN)
		{
			pCurrent = pCurrentHead;
			ret = AtomCmd_SearchingFrame((u8**)&pCurrentHead, RING_BUFFER_SIZE, (u32*)&unProcessBytes, &needProcessedLen);
		}

		if (ret == FRAME_COMPLETE)
		{
			CompletePackage = 1;
			printf("RX: ");
			for (int i = 0; i < (needProcessedLen); i++)
				printf("%02X ", pCurrentHead[i]);
			printf("\n");

		}

		else if (ret == FRAME_ERROR)
			CompletePackage = 0;
		else
			CompletePackage = 0;

		//Modify consumer if unProcessBytes had been changed.
		pCustomer += (pCurrentHead - pCurrent);
		if (pCustomer > pEndRing)
			pCustomer = pStartRing + (pCustomer - pEndRing);

		if (print_flag)
		{
			printf("%s(),CompletePackage = %d, needProcessedLen = %d, unProcessBytes = %d\n", __FUNCTION__, CompletePackage, needProcessedLen, unProcessBytes);
			printf("%s(),AtomCmd_SearchingFrame() pProducer  = %p, pCustomer = %p, \n", __FUNCTION__, pProducer, pCustomer);
		}
		if (CompletePackage)
		{
			CompletePackage = 0;

			//Save unprocessBytes before processer.
			previous_bytes = unProcessBytes;
			processedBytes = needProcessedLen;
			ret = AtomCmd_Processer(pCurrentHead, (u8**)&pCustomer, ringBuf, RING_BUFFER_SIZE, (u32*)&needProcessedLen);

			//processedBytes = previous_bytes - unProcessBytes;
			pCurrentHead += processedBytes;
			unProcessBytes -= processedBytes;
			if (print_flag)
				printf("after: pProducer  = %p, pCustomer = %p, pCustomerMove: %d,unProcessBytes= %d \n",
				pProducer, pCustomer, processedBytes, unProcessBytes);

		}
		else
		{
			continue;
		}
	}

	if (print_flag)
		printf("after: pProducer  = %p, pCustomer = %p, pCustomerMove: %d,unProcessBytes= %d \n",
		pProducer, pCustomer, processedBytes, unProcessBytes);
}

void ReceiveData(u8 cid,u8 mid)
{
	int index = 0;
	int frameStart = 0;
	int ind = 0;
	unsigned char * pStart = NULL;
	int remainBytes = 0;

	signed char cResult[2] = { 0 };

	while (1)
	{
		usLength = CollectUARTData((char*)chrBuffer);
		int chrIndex = 0;

		if (usLength > 0)
		{
			//Start_time = GetTickCount();
			usRxLength += usLength;

			//Save data from comport to ringbuffer then update producer
			if (usLength < (pBufferEnd - pProducer))
			{
				memcpy((void*)pProducer, chrBuffer, usLength);
			}
			else //part of chrBuffer should be copied to begin of ringbuffer.
			{
				memcpy((void*)pProducer, chrBuffer, pBufferEnd - pProducer);
				chrIndex += (int)(pBufferEnd - pProducer);
				memcpy(pBufferStart, (char*)&chrBuffer + chrIndex, usLength - (pBufferEnd - pProducer));
			}
			//update producer 
			pProducer += usLength;

			//reach the end and rewind to begin
			if (pProducer > pBufferEnd)
				pProducer = pBufferStart + (pProducer - pBufferEnd);

			//Processing data
			if (usRxLength >= FRAME_MIN_SIZE)
			{

				if ((cid == CLASS_ID_HOSTCONTROL_CMD) && (mid == (CMD_ID_SABER_DATA_PACKET | 0x80)))
				{
					Process();
				}
				else
				{
					commandAckHandle(cid, mid);
					return;
				}
				
			}

			
		}
		else
			usleep(100);
	}
	return;
}

/*--------------Process received ACk------------------------*/

void commandAckHandle(unsigned char classID, unsigned char mID)
{
	u8 errorCode = ((classID & 0xF0) >> 4);;
	int index = 2 ;
	u8 *p= chrBuffer;
	
	while (1)
	{
		//usleep(100);
		if (index == RING_BUFFER_SIZE)
			return;
		if (((chrBuffer[index] & 0x0F) == classID) && ((chrBuffer[index + 1] ) == (mID |0x80)))
		{
			if (chrBuffer[index + 2 +chrBuffer[index + 2] + 2] == 0x6D)
			{
				rcid = classID;
				rmid = mID;
				p = (u8*)(chrBuffer + index - 3);
				break;
			}
		}					
		index++;
	}
	usleep(100);
	errorCode_ack = ((chrBuffer[index] & 0xF0) >> 4);
	
	switch (classID)
	{
		/*---------User Code Begin-------------*/
		/*----------Users can add other command parsing--------------------------*/
	case CLASS_ID_OPERATION_CMD:
		if (mID == (CMD_ID_SWITCH_TO_CFG_MODE))
		{
			receiveAck = 1;
		}
		if (mID == (CMD_ID_SWITCH_TO_MEASURE_MODE))
		{
			receiveAck = 1;
		}
		break;
	case CLASS_ID_HOSTCONTROL_CMD:
		if (mID == (CMD_ID_SET_DATA_PAC_CFG))
		{
			receiveAck = 1;
		}
		break;
	case CLASS_ID_ALGORITHMENGINE:
		if (mID == (CMD_ID_SET_PACKET_UPDATE_RATE))
		{
			receiveAck = 1;
		}
		break;
	/*---------User Code End-------------*/
	default:
		break;
	}

	printf("RX: ");
	for (int i = 0; i < (p[ATOM_PAYLOAD_LEN_INDEX] + 8); i++)
		printf("%02X ", p[i]);
	printf("\n");
}

/*-------------------Packet parsing---------------------------------*/
void DataPacketParser(u8 *pBuffer, u16 dataLen)
{
	u16 PID = 0;
	u8 *pData = pBuffer;
	u8 index = 0;
	u8 pl = 0;

	//reset saberDataHandle
	memset(&saberDataHandle, 0, sizeof(saberDataHandle));
	printf("\n");
	while (index < dataLen)
	{
		PID = ((*((u16*)(pData + index))) & 0x7fff);
		pl = *(pData + index + 2);
		if (PID == (SESSION_NAME_TEMPERATURE))
		{
			//Ignore pid and pl
			index += 3;

			memcpy(&saberDataHandle.temperature.data, pData + index, PL_TEMPERTURE);
			saberDataHandle.temperature.dataID = PID;
			saberDataHandle.temperature.dataLen = pl;
			printf(" *** temperature:\t%11.4f *** \n", saberDataHandle.temperature.data);

			index += PL_TEMPERTURE;

		}
		else if (PID == (SESSION_NAME_RAW_ACC))
		{
			//Ignore pid and pl
			index += 3;

			memcpy(&saberDataHandle.accRawData.accX, pData + index, PL_RAW_DATA);
			saberDataHandle.accRawData.dataID = PID;
			saberDataHandle.accRawData.dataLen = pl;

			index += PL_RAW_DATA;

		}
		else if (PID == SESSION_NAME_RAW_GYRO)
		{
			//Ignore pid and pl
			index += 3;

			memcpy(&saberDataHandle.gyroRawData.gyroX, pData + index, PL_RAW_DATA);
			saberDataHandle.gyroRawData.dataID = PID;
			saberDataHandle.gyroRawData.dataLen = pl;
			index += PL_RAW_DATA;
		}
		else if (PID == SESSION_NAME_RAW_MAG)
		{
			//Ignore pid and pl
			index += 3;

			memcpy(&saberDataHandle.magRawData.magX, pData + index, PL_RAW_DATA);
			saberDataHandle.magRawData.dataID = PID;
			saberDataHandle.magRawData.dataLen = pl;
			index += PL_RAW_DATA;
		}
		else if (PID == SESSION_NAME_CAL_ACC)
		{
			//Ignore pid and pl
			index += 3;

			memcpy(&saberDataHandle.accCal.accX, pData + index, PL_CAL_DATA);
			saberDataHandle.accCal.dataID = PID;
			saberDataHandle.accCal.dataLen = pl;
			index += PL_CAL_DATA;

			printf(" *** accCal:     \t%11.4f, %11.4f, %11.4f *** \n", saberDataHandle.accCal.accX, saberDataHandle.accCal.accY, saberDataHandle.accCal.accZ);
		}
		else if (PID == SESSION_NAME_CAL_GYRO)
		{
			//Ignore pid and pl
			index += 3;

			memcpy(&saberDataHandle.gyroCal.gyroX, pData + index, PL_CAL_DATA);

			saberDataHandle.gyroCal.dataID = PID;
			saberDataHandle.gyroCal.dataLen = pl;
			index += PL_CAL_DATA;

			printf(" *** gyroCal:    \t%11.4f, %11.4f, %11.4f *** \n", saberDataHandle.gyroCal.gyroX, saberDataHandle.gyroCal.gyroY, saberDataHandle.gyroCal.gyroZ);
		}
		else if (PID == SESSION_NAME_CAL_MAG)
		{
			//Ignore pid and pl
			index += 3;

			memcpy(&saberDataHandle.magCal.magX, pData + index, PL_CAL_DATA);
			saberDataHandle.magCal.dataID = PID;
			saberDataHandle.magCal.dataLen = pl;
			index += PL_CAL_DATA;

			printf(" *** magCal:     \t%11.4f, %11.4f, %11.4f *** \n", saberDataHandle.magCal.magX, saberDataHandle.magCal.magY, saberDataHandle.magCal.magZ);
		}
		else if (PID == SESSION_NAME_KAL_ACC)
		{
			//Ignore pid and pl
			index += 3;

			memcpy(&saberDataHandle.accKal.accX, pData + index, PL_KAL_DATA);
			saberDataHandle.accKal.dataID = PID;
			saberDataHandle.accKal.dataLen = pl;
			index += PL_KAL_DATA;

		}
		else if (PID == SESSION_NAME_KAL_GYRO)
		{
			//Ignore pid and pl
			index += 3;

			memcpy(&saberDataHandle.gyroKal.gyroX, pData + index, PL_KAL_DATA);
			saberDataHandle.gyroKal.dataID = PID;
			saberDataHandle.gyroKal.dataLen = pl;
			index += PL_KAL_DATA;
		}
		else if (PID == SESSION_NAME_KAL_MAG)
		{
			//Ignore pid and pl
			index += 3;

			memcpy(&saberDataHandle.magKal.magX, pData + index, PL_KAL_DATA);
			saberDataHandle.magKal.dataID = PID;
			saberDataHandle.magKal.dataLen = pl;
			index += PL_KAL_DATA;
		}
		//////////////////////////
		else if (PID == SESSION_NAME_QUAT)
		{
			//Ignore pid and pl
			index += 3;

			memcpy(&saberDataHandle.quat.Q0.uint_x, pData + index, PL_QUAT_EULER);
			saberDataHandle.quat.dataID = PID;
			saberDataHandle.quat.dataLen = pl;
			index += PL_QUAT_EULER;
			printf(" *** quat :      \t%11.4f, %11.4f, %11.4f, %11.4f *** \n", saberDataHandle.quat.Q0.float_x, saberDataHandle.quat.Q1.float_x, saberDataHandle.quat.Q2.float_x, saberDataHandle.quat.Q3.float_x);

		}
		else if (PID == SESSION_NAME_EULER)
		{
			//Ignore pid and pl
			index += 3;

			memcpy(&saberDataHandle.euler.roll, pData + index, PL_QUAT_EULER);
			saberDataHandle.euler.dataID = PID;
			saberDataHandle.euler.dataLen = pl;
			index += PL_QUAT_EULER;
			//temperature:
			printf(" *** euler:      \t%11.4f, %11.4f, %11.4f *** \n", saberDataHandle.euler.roll, saberDataHandle.euler.pitch, saberDataHandle.euler.yaw);
		}

		else if (PID == SESSION_NAME_ROTATION_M)
		{
			//Ignore pid and pl
			index += 3;

			memcpy(&saberDataHandle.romatix.a, pData + index, PL_MATERIX);
			saberDataHandle.romatix.dataID = PID;
			saberDataHandle.romatix.dataLen = pl;
			index += PL_MATERIX;

		}

		else if (PID == SESSION_NAME_LINEAR_ACC)
		{
			//Ignore pid and pl
			index += 3;

			memcpy(&saberDataHandle.accLinear.accX, pData + index, PL_LINEAR_ACC_DATA);
			saberDataHandle.accLinear.dataID = PID;
			saberDataHandle.accLinear.dataLen = pl;
			index += PL_LINEAR_ACC_DATA;

		}
		else if (PID == SESSION_NAME_PACKET_COUNTER)
		{
			//Ignore pid and pl
			index += 3;

			memcpy(&saberDataHandle.packetCounter.packerCounter, pData + index, PL_PACKET_NUMBER);
			saberDataHandle.packetCounter.dataID = PID;
			saberDataHandle.packetCounter.dataLen = pl;
			index += PL_PACKET_NUMBER;
		}
		else if (PID == SESSION_NAME_DELTA_T)
		{
			//Ignore pid and pl
			index += 3;
			memcpy(&saberDataHandle.dt.DT, pData + index, PL_DT_DATA);

			saberDataHandle.dt.dataID = PID;
			saberDataHandle.dt.dataLen = pl;
			index += PL_DT_DATA;
		}

		else if (PID == SESSION_NAME_OS_TIME)
		{
			//Ignore pid and pl
			index += 3;

			memcpy(&saberDataHandle.tick.OS_Time_ms, pData+index, PL_OS_REFERENCE_TIME-2); //first 4 bytes are miliseconds
			saberDataHandle.tick.OS_Time_ms = *((u32*)(pData + index));
			saberDataHandle.tick.OS_Time_us = *((u16*)(pData + index + 4));

			saberDataHandle.tick.dataID = PID;
			saberDataHandle.tick.dataLen = pl;
			index += PL_OS_REFERENCE_TIME;
		}
		else if (PID == SESSION_NAME_STATUS_WORD)
		{
			//Ignore pid and pl
			index += 3;

			memcpy(&saberDataHandle.status.status, pData + index, PL_STATUS);
			saberDataHandle.status.dataID = PID;
			saberDataHandle.status.dataLen = pl;
			index += PL_STATUS;
		}else {
		index +=3;
		index +=pl;
		}
	}
	//	printf("\n\n");
}
/*-----------------------Packet selection---------------------------*/
void SelectPackets(char enable)
{
	u16 pid = 0;
	ConfigSingleDataPacket_HandleType Packet[6];
	u8 index = 0;
	for (int i = 0; i < 6; i++)
	{
		switch (i)
			{
			case CAL_ACC:
				if (((enable >> i) & 0x1) == 0x1)
					pid = SESSION_NAME_CAL_ACC | 0x8000;
				else
					pid = SESSION_NAME_CAL_ACC;
				break;
			case CAL_GYRO:
				if (((enable >> i) & 0x1) == 0x1)
					pid = SESSION_NAME_CAL_GYRO | 0x8000;
				else
					pid = SESSION_NAME_CAL_GYRO;
				break;
			case CAL_MAG:
				if (((enable >> i) & 0x1) == 0x1)
					pid = SESSION_NAME_CAL_MAG | 0x8000;
				else
					pid = SESSION_NAME_CAL_MAG;
				break;
			case EULER_DATA:
				if (((enable >> i) & 0x1) == 0x1)
					pid = SESSION_NAME_EULER | 0x8000;
				else
					pid = SESSION_NAME_EULER;
				break;
			case QUAT_DATA:
				if (((enable >> i) & 0x1) == 0x1)
					pid = SESSION_NAME_QUAT | 0x8000;
				else
					pid = SESSION_NAME_QUAT;
				break;
			case TEMPERATURE:
				if (((enable >> i) & 0x1) == 0x1)
					pid = SESSION_NAME_TEMPERATURE | 0x8000;
				else
					pid = SESSION_NAME_TEMPERATURE;
				break;
			default:
				break;

			}
			Packet[index].reserve0 = 0xff;
			Packet[index].reserve1 = 0xff;
			Packet[index].packetID = pid;
			index++;
	
	}
	Atom_setDataPacketConfigReq((u8*)&Packet, index * 4);

}
/*------------------------Set Output Data Rate----------------------*/
void SetHostUpdateRate(char rate)
{
	u8 data[2];
	data[0] = rate & 0xff;
	data[1] = (rate & 0xff00) >> 8;
	Atom_setPacktUpdateRateReq((u8*)&data, 2);

}
