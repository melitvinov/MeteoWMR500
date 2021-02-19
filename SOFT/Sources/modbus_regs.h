/***********************************************************************************
* ������������ ���� ���	: 
*
************************************************************************************/

#ifndef __MODBUS_REGS_H
#define __MODBUS_REGS_H

//--- INCLUDES -------------------

#include "GPIO_STM32F10x.h"
#include "stm32f10x.h"
#include "Board.h"

// --- DEFINES -------------------
#define START_REG_NUM 			0 			
#define START_REG_VALUES		1000						// ����� ������� �������� ��������

// --- TYPES ---------------------

#pragma pack(push,1)

typedef struct __reg_entry
{
    uint16_t addr;											// ������������� ����� �������� (�������� ������������ �������� ������)
    uint16_t idx;											// ������ ����� (0-9)
	int (*read)(uint16_t idx);								// ����� ������� ������ �������� ��������
    bool (*write)(uint16_t idx, uint16_t val);				// ����� ������� ������ �������� ��������
} TRegEntry;

#pragma pack(pop)

//--- FUNCTIONS ------------------

bool REG_isWriteEnable( uint16_t regAddr );
bool REG_Write( uint16_t RegAddr, uint16_t value );
bool REG_Read( uint16_t RegAddr, uint16_t * pValue );

#endif
