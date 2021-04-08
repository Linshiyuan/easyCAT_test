/*---------------------------------------------------------------------------------------------
 *  Copyright (c) Atom-Robotics Corporation. All rights reserved.
 *  Author: niuyunzhu
 *  Last Modify: 2019.10.20
 *  Description: Saber Demo Project on ROS
 *  Licensed under the Apache License Version 2.0
 *  See https://www.apache.org/licenses/LICENSE-2.0.html for license information.
 *--------------------------------------------------------------------------------------------*/

#define SET_BIT(REG, BIT)     ((REG) |= (BIT))
#define CLEAR_BIT(REG, BIT)   ((REG) &= ~(BIT))
//std include
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdbool.h>
#include <iostream>
#include <iomanip>
#include 	<pthread.h>

//usr include
#include "saber_ros_inc/saber_serial.h"
#include "saber_ros_inc/saber_macro.h"
#include "saber_ros_inc/saber_protocol.h"
#include "saber_ros_inc/saber_config.h"
#include "saber_ros_inc/saber_tool.h"

// #define CUSTOM

#include "EasyCAT.h"	
EasyCAT EASYCAT(DC_SYNC);   
pthread_t catThread;
void *runEasyCAT(void* arg);
using namespace std;

//global variant
SaberData saberDataHandle;

//Warning:Don't use malloc to get buffer,Just use a global array;
unsigned char frmBuf[256] = {0};

//main
int main(int argc, char **argv)
{
         if (EASYCAT.Init() == true)						 // initialization
     {
       printf("inizialized\n");							 // succesfully completed
     }
     else											 	 // initialization failed   
     {							
       printf("inizialization failed\n");				 // the EasyCAT board was not recognized
	   return -1;
     }	



    unsigned char nFD = 0;
    int packLengthFW = 0;
    int pkgLen = 0;
    int pubCnt = 0;
    FILE *fpLog = NULL;
    u8 ret = 0;
    int seq = 0;
    bool met;
    int cycleCnt = 0;
    int errCnt = 0;

    unsigned char *dataBuf = NULL;
    dataBuf = &frmBuf[0];

#ifndef MCU_ON
    printf("Hello,Saber on ROS!\n");
#endif



    if (dataBuf == NULL)
    {
#ifndef MCU_ON
        printf("Data buffer %d bytes valid fail \n", pkgLen);
#endif
        return -2;
    }

    //step 1: read config,open serialport,get serialport file desriptor
    //be careful to choose file path
    nFD = SaberInitConfig("/home/pi/pi8gcode/easyCAT/cmakeTest/saber_cfg.json");
    //option: a log file
    //fpLog = fopen("imu.log", "w");

    //step 2: align Saber data frame from the serial stream
    packLengthFW = SaberAlign(nFD);
    pkgLen = packLengthFW + SABER_EMPTY_LEN;
    SaberFillFrameHead(dataBuf);

    //step 3:get a whole frame and valid the frame
    SaberGetFrame(nFD, dataBuf + SABER_HEAD_LEN, packLengthFW + SABER_TAIL_LEN);
    while (!SaberValidFrame(dataBuf, pkgLen))
    {
        packLengthFW = SaberAlign(nFD);
        SaberFillFrameHead(dataBuf);
        SaberGetFrame(nFD, dataBuf + SABER_HEAD_LEN, packLengthFW + SABER_TAIL_LEN);
    }
	u32 counter=0;
    
    if(pthread_create(&catThread, NULL, &runEasyCAT, NULL)!=0)
		printf("Create receive com data thread failed!\n");
	else



    while (1)
    {
        
        //SaberFillFrameHead(dataBuf);
        SaberGetFrame(nFD, dataBuf, packLengthFW + SABER_EMPTY_LEN);

        while (!SaberValidFrame(dataBuf, pkgLen))
        {
           
            packLengthFW = SaberAlign(nFD);
            SaberFillFrameHead(dataBuf);
            SaberGetFrame(nFD, dataBuf + SABER_HEAD_LEN, packLengthFW + SABER_TAIL_LEN);
            errCnt++;
        }
        //step 4:parser a whole frame to generate ros publish data
        SaberParserDataPacket(&saberDataHandle, &dataBuf[SABER_HEAD_LEN], packLengthFW, fpLog);


        counter++;


        std::cout << "roll: " << std::fixed << std::setprecision(4) << saberDataHandle.euler.roll << 
                    " pitch: " << std::fixed << std::setprecision(4) << saberDataHandle.euler.pitch << 
                    " yaw: " << std::fixed << std::setprecision(4) << saberDataHandle.euler.yaw << std::endl;

        // std::cout <<"raw: "<<std::hex<<(int32_t)rawEuler[0] <<"pitch: "<<std::hex<<rawEuler[1] <<"yaw: "<<std::hex<<rawEuler[2]<< std::endl;
        EASYCAT.BufferIn.Cust.roll= rawEuler[0];
        EASYCAT.BufferIn.Cust.pitch= rawEuler[1];
        EASYCAT.BufferIn.Cust.yaw= rawEuler[2];
         EASYCAT.BufferIn.Cust.counter=counter;

        //    std::cout <<" set is : "<<EASYCAT.BufferOut.Cust.set<< std::endl;
           printf("set = %d counter is %d\n\n ", EASYCAT.BufferOut.Cust.set,counter);	
          usleep(10);		
        //4000 frame to exit, user should customize the value or bypass for a infinite loop
        // if ((++cycleCnt) > 40000)
        //     break;
    }
    if (nFD > 0)
    {
        Saber_CloseSerialPort(nFD);
    }

#ifndef MCU_ON
    printf("%d Frames record,%d  Interrupt Frames\n", cycleCnt - 1, errCnt);
#endif
    return 0;
}

void *runEasyCAT(void* arg){
    while (1)
    {
         unsigned char Status = EASYCAT.MainTask();	
         usleep(1);
    }
    

}


