/***********************************************************************************
* ������������ ���� ���	: ������ ���������� ������
*
************************************************************************************/

#ifndef __ANA_INPUTS_H
#define __ANA_INPUTS_H

//--- INCLUDES -------------------

#include "GPIO_STM32F10x.h"
#include "stm32f10x.h"
#include "Board.h"

// --- DEFINES -------------------

#define AVERAGE_COUNT		10		// ���-�� �������� ��� ���������� �� ���������� ������
#define ADC_POWER			3300	// ���������� ������� �� ��� (mV)
#define ADC_STEP_CNT		4095	// ���-�� ���������� ���

// --- TYPES ---------------------

//--- FUNCTIONS ------------------
void Thread_AnalogSensors( void *pvParameters );
int read_SUN_AdcValue( uint16_t idx );

#endif
