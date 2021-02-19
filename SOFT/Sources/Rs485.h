/***********************************************************************************
* ������������ ���� ���	: ������ ����� �� ���� RS-485
*
************************************************************************************/

#ifndef __RS485_H
#define __RS485_H

//--- INCLUDES -------------------

#include "GPIO_STM32F10x.h"
#include "stm32f10x.h"
#include "Board.h"
#include "Modbus.h"

// --- DEFINES -------------------
#define DELAY_BEFORE_SEND	1	// �������� �������� ����� ������������ � ����� ��������
#define DELAY_AFTER_SEND	2	// �������� ������������ � ����� ������ ����� ��������

#define	USART_BRR_9600  0x341;		// Baudrate for 9600 on 8Mhz (0x341)

// --- TYPES ---------------------

//--- FUNCTIONS ------------------

void Rs485_Init(void);
void RS485_SendBuf( TPacketBuffer * buffer );

#endif
