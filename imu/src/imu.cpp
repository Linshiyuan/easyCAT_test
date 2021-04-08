#include "imu.h"


//main.c

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

//main.c

  
  
    // Member Functions()
    void atomImu::printname()
    {
       std::cout << "Geekname is: " << geekname;
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