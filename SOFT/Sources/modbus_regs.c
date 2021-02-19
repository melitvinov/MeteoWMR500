/***********************************************************************************
* Наименование модуля	: Оисание регистров MODBUS, функции чтения и записи платы РЕЛЕ
*-----------------------------------------------------------------------------------
* Версия				: 1.0
* Автор					: Тимофеев
* Дата создания			: 21.08.2020
************************************************************************************/

//--- INCLUDES -------------------
#include "FreeRTOS.h"
#include "task.h"
#include "modbus_regs.h"

#include "AnaInputs.h"

// --- DEFINES -------------------

// --- TYPES ---------------------

//--- CONSTANTS ------------------

// описательные регистры
const uint16_t gDescRegs[] = {
	0xAAAA,	// vendor_id
	0x1111, // product_id
	0x0203,	// firmware_version
	0x0000,	// firmware_git_hash_0
	0x0000,	// firmware_git_hash_1
	0x0000,	// firmware_git_hash_2
	0x0000,	// firmware_git_hash_3
	0x0000,	// firmware_build_timestamp_0
	0x0000	// firmware_build_timestamp_1
};

//--- FORWARD --------------------
TRegEntry * get_regentry_by_regaddr( uint16_t regaddr );

//--- GLOBAL VARIABLES -----------


uint16_t regs_addin[15] = {
	0, 0, 0, 0xBEB0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 
};

//--- EXTERN ---------------------
extern uint8_t GetDeviceAddress(void);
extern uint16_t g_Status;
extern uint16_t g_WIND_value;
extern uint16_t g_RAIN_value;
extern uint16_t g_WIND_ANGLE_value;
extern uint16_t g_RH_flag;
extern uint16_t g_RH_Value;
extern uint16_t g_T_flag;
extern uint16_t g_T_Value;

//--- FUNCTIONS ------------------

int read_DescReg( uint16_t idx )
{
	return gDescRegs[idx];
}

int read_AddIn( uint16_t idx )
{
	return gDescRegs[idx];
}

bool write_AddIn( uint16_t idx, uint16_t val )
{
	if( idx > 14 ) return false;
	regs_addin[idx] = val;
	return true;
}

int read_Status( uint16_t idx )
{
	return g_Status;
}

bool write_Status( uint16_t idx, uint16_t val )
{
	g_Status = val;
	return true;
}
int read_RainValue( uint16_t idx )
{
	return g_RAIN_value;
}
int read_WindValue( uint16_t idx )
{
	return g_WIND_value;
}
int read_WindAngleValue( uint16_t idx )
{
	return g_WIND_ANGLE_value;
}

int read_RH_T( uint16_t idx )
{
	switch( idx )
	{
		case 0:
			return g_RH_flag;
		case 1:
			return g_RH_Value;
		case 2:
			return g_T_flag;
		case 3:
			return g_T_Value;
		default:
			break;
	}
	return -1;
}

/*******************************************************
Функция		: Пытается прочитать значение указанного регистра
Параметр 1	: индекс регистра
Параметр 2	: следующий байт
Возвр. знач.: флаг - значение прочитано
********************************************************/
bool REG_Read( uint16_t RegAddr, uint16_t * pValue )
{
	if( !pValue )
		return false;
	
	TRegEntry * pReg = get_regentry_by_regaddr( RegAddr );
	if( pReg )
	{
		if( pReg->read )
		{
			int result = pReg->read( pReg->idx );
			if( result >= 0 )
			{
				*pValue = (uint16_t) result;
				return true;
			}
		}
	}

	return false;
}	

/*******************************************************
Функция		: Пытается прочитать значение указанного регистра
Параметр 1	: индекс регистра
Параметр 2	: следующий байт
Возвр. знач.: флаг - значение прочитано
********************************************************/
bool REG_Write( uint16_t RegAddr, uint16_t value )
{
	TRegEntry * pReg = get_regentry_by_regaddr( RegAddr );
	if( pReg )
	{
		if( pReg->write )
		{
			bool result = pReg->write( pReg->idx, value );
			return result;
		}
	}
	return false;
}	

/*******************************************************
Функция		: Проверяет регистр на возможность записи в него
Параметр 1	: номер регистра
Возвр. знач.: флаг - запись разрешена 
********************************************************/
bool REG_isWriteEnable( uint16_t regAddr )
{
	TRegEntry * pReg = get_regentry_by_regaddr( regAddr );
	if( pReg )
	{
		if( pReg->write )
		{
			return true;
		}
	}

	return false;
}

// Регистры платы универсальных входов
TRegEntry RegEntries[] = 
{
	{.addr=0, .idx = 0, .read = read_DescReg, .write=0, },
	{.addr=1, .idx = 1, .read = read_DescReg, .write=0, },
	{.addr=2, .idx = 2, .read = read_DescReg, .write=0, },
	{.addr=3, .idx = 3, .read = read_DescReg, .write=0, },
	{.addr=4, .idx = 4, .read = read_DescReg, .write=0, },
	{.addr=5, .idx = 5, .read = read_DescReg, .write=0, },
	{.addr=6, .idx = 6, .read = read_DescReg, .write=0, },
	{.addr=7, .idx = 7, .read = read_DescReg, .write=0, },
	{.addr=8, .idx = 8, .read = read_DescReg, .write=0, },
	
	{.addr=30, .idx = 0, .read = read_AddIn, .write=write_AddIn, },
	{.addr=31, .idx = 1, .read = read_AddIn, .write=0, },
	{.addr=32, .idx = 2, .read = read_AddIn, .write=0, },
	
	{.addr=50, .idx = 3, .read = read_AddIn, .write=write_AddIn, },
	{.addr=51, .idx = 4, .read = read_AddIn, .write=write_AddIn, },
	{.addr=52, .idx = 5, .read = read_AddIn, .write=write_AddIn, },
	{.addr=53, .idx = 6, .read = read_AddIn, .write=write_AddIn, },
	{.addr=54, .idx = 7, .read = read_AddIn, .write=write_AddIn, },
	{.addr=55, .idx = 8, .read = read_AddIn, .write=write_AddIn, },
	{.addr=56, .idx = 9, .read = read_AddIn, .write=write_AddIn, },
	{.addr=57, .idx = 10, .read = read_AddIn, .write=write_AddIn, },
	{.addr=58, .idx = 11, .read = read_AddIn, .write=write_AddIn, },
	{.addr=59, .idx = 12, .read = read_AddIn, .write=write_AddIn, },

	// Регистр статуса
	{.addr=START_REG_VALUES-1, .idx = 0, .read = read_Status, .write=write_Status, },

	// Значение adc солнца
	{.addr=START_REG_VALUES + 0, .idx = 0, .read = read_SUN_AdcValue, .write=0, },
	// Значение датчика Дождь
	{.addr=START_REG_VALUES + 1, .idx = 0, .read = read_RainValue, .write=0, },
	// Значение датчика Скорость ветра
	{.addr=START_REG_VALUES + 2, .idx = 0, .read = read_WindValue, .write=0, },
	// Значение датчика Направление ветра
	{.addr=START_REG_VALUES + 3, .idx = 0, .read = read_WindAngleValue, .write=0, },
	
	// Флаг ошибки датчика влажности 
	{.addr=START_REG_VALUES + 4, .idx = 0, .read = read_RH_T, .write=0, },
	// Значение датчика влажности 
	{.addr=START_REG_VALUES + 5, .idx = 1, .read = read_RH_T, .write=0, },
	// Флаг ошибки датчика температуры 
	{.addr=START_REG_VALUES + 6, .idx = 2, .read = read_RH_T, .write=0, },
	// Значение датчика температуры
	{.addr=START_REG_VALUES + 7, .idx = 3, .read = read_RH_T, .write=0, },
};

/*******************************************************
Функция		: Ищет регистр по его адресу
Параметр 1	: адрес регистра
Возвр. знач.: указатель на структуру регистра
********************************************************/
TRegEntry * get_regentry_by_regaddr( uint16_t regaddr )
{
	TRegEntry * regEntry = 0;

	uint16_t total = sizeof(RegEntries);
	uint16_t one = sizeof(TRegEntry);
	uint16_t count = total / one;	// Количество регистров
	
	for( int i=0; i<count; i++ )
	{
		if( RegEntries[i].addr == regaddr )
		{
			regEntry = &(RegEntries[i]);
			break;
		}
	}
	return regEntry;
}
