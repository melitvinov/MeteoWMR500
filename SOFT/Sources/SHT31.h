/*****************************************************************************************
Заголовочный файл для SHT31 

*****************************************************************************************/

#ifndef __SHT31
#define __SHT31

#include "GPIO_STM32F10x.h"
#include "stm32f10x.h"

#include "Board.h"

#include "FreeRTOS.h"
#include "task.h"

// адрес микросхемы
#define SHT_ADDRESS	0x44		

#define CMD_RH_MEASUREMENT_HOLD	(0xE5)
#define CMD_T_MEASUREMENT_HOLD	(0xE3)

#define OPERATION_READ 			(1)
#define OPERATION_WRITE 		(0)

#pragma pack (push,1)
typedef struct __data_struct
{
	uint16_t uiTemperature;
	uint8_t crcT;
	uint16_t uiHR;
	uint8_t crcHR;
} TDataStruct;
#pragma pack (pop)

void SHT31_Init( void );
bool SHT31_Send_ART_Command( void );
bool SHT31_Read_SensorValues( float * pRH, float * pTemp );

#endif
