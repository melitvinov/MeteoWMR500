/***********************************************************************************
* Заголовочный файл для	: Модуль аналоговых входов
*
************************************************************************************/

#ifndef __ANA_INPUTS_H
#define __ANA_INPUTS_H

//--- INCLUDES -------------------

#include "GPIO_STM32F10x.h"
#include "stm32f10x.h"
#include "Board.h"

// --- DEFINES -------------------

#define AVERAGE_COUNT		10		// кол-во значений для усреднения по аналоговым входам
#define ADC_POWER			3300	// напряжение питания на АЦП (mV)
#define ADC_STEP_CNT		4095	// кол-во интервалов АЦП

// --- TYPES ---------------------

//--- FUNCTIONS ------------------
void Thread_AnalogSensors( void *pvParameters );
int read_SUN_AdcValue( uint16_t idx );

#endif
