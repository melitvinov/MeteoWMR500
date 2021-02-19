/***********************************************************************************
* Заголовочный файл для	: Переключатель адреса для шины RS-485
*
************************************************************************************/

#ifndef __ADDRSWITCH_H
#define __ADDRSWITCH_H

//--- INCLUDES -------------------

#include "GPIO_STM32F10x.h"
#include "stm32f10x.h"
#include "Board.h"

#define ADDR_SHIFT_VALUE  (0); // какая то добавка к адресу устройства Meteo


//--- FUNCTIONS ------------------

void ADRSW_init(void);
uint8_t ADRSW_GetAdr(void);


#endif
