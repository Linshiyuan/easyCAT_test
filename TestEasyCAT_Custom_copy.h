#ifndef CUSTOM_PDO_NAME_H
#define CUSTOM_PDO_NAME_H

//-------------------------------------------------------------------//
//                                                                   //
//     This file has been created by the Easy Configurator tool      //
//                                                                   //
//     Easy Configurator project TestEasyCAT_Custom.prj
//                                                                   //
//-------------------------------------------------------------------//


#define CUST_BYTE_NUM_OUT	1
#define CUST_BYTE_NUM_IN	16
#define TOT_BYTE_NUM_ROUND_OUT	4
#define TOT_BYTE_NUM_ROUND_IN	16


typedef union												//---- output buffer ----
{
	uint8_t  Byte [TOT_BYTE_NUM_ROUND_OUT];
	struct
	{
		uint8_t     set;
	}Cust;
} PROCBUFFER_OUT;


typedef union												//---- input buffer ----
{
	uint8_t  Byte [TOT_BYTE_NUM_ROUND_IN];
	struct
	{
		uint32_t    roll;
		uint32_t    pitch;
		uint32_t    yaw;
		uint32_t    counter;
	}Cust;
} PROCBUFFER_IN;

#endif