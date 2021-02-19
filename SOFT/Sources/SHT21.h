/*****************************************************************************************
Заголовочный файл для FM24V02 

*****************************************************************************************/

#ifndef __FM24V02
#define __FM24V02

#include "GPIO_STM32F10x.h"
#include "stm32f10x.h"

#include "Board.h"

#include "FreeRTOS.h"
#include "task.h"

// адрес микросхемы
#define SHT_ADDRESS	0x80		

#define CMD_RH_MEASUREMENT_HOLD	(0xE5)
#define CMD_T_MEASUREMENT_HOLD	(0xE3)

#define OPERATION_READ 			(1)
#define OPERATION_WRITE 		(0)

void SHT21_Init( void );
bool SHT21_Read_RH( float * pValue );
bool SHT21_Read_T( float * pValue );

#endif
