/***********************************************************************************
* Наименование модуля	: Модуль датчиков EC
*-----------------------------------------------------------------------------------
* Версия				: 1.0
* Автор					: Тимофеев 
* Дата создания			: 30.05.2018
************************************************************************************/

//--- INCLUDES -------------------
#include "FreeRTOS.h"
#include "task.h"

#include "AnaInputs.h"

// --- DEFINES -------------------

// --- TYPES ---------------------

//--- CONSTANTS ------------------

//--- GLOBAL VARIABLES -----------
uint16_t g_SUN_AdcValue;

//--- EXTERN ---------------------

//--- IRQ ------------------------

//--- FUNCTIONS ------------------

/*******************************************************
Функция		: Получение значения adc солнца
Параметр 1	: idx - не используется
Возвр. знач.: значение adc солнца
********************************************************/
int read_SUN_AdcValue( uint16_t idx )
{
	return g_SUN_AdcValue;
}

/*******************************************************
Функция		: Инициализация приложения
Параметр 1	: нет
Возвр. знач.: нет
********************************************************/
void adc_Initialize (void)
{
	// настройка аналоговых входов
	// пропускаем, по умолчанию выводы запрограммированы как входы

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	// настройки ADC
	ADC_InitTypeDef ADC_InitStructure;
	ADC_StructInit(&ADC_InitStructure);
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent; // режим работы - одиночный, независимый
	ADC_InitStructure.ADC_ScanConvMode = DISABLE; // не сканировать каналы, просто измерить один канал
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE; // однократное измерение
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; // без внешнего триггера
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; //выравнивание битов результат - прижать вправо
	ADC_InitStructure.ADC_NbrOfChannel = 1; //количество каналов - одна штука
	ADC_Init(ADC1, &ADC_InitStructure);
	ADC_Cmd(ADC1, ENABLE);

	// настройка канала
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_55Cycles5);

	// калибровка АЦП
	ADC_ResetCalibration(ADC1);
	while (ADC_GetResetCalibrationStatus(ADC1));
	ADC_StartCalibration(ADC1);
	while (ADC_GetCalibrationStatus(ADC1)); 
}

/*******************************************************
Функция		: Получение значения АЦП по заданному каналу
Параметр 1	: канал
Возвр. знач.: нет
********************************************************/
uint16_t adc_get_value(uint8_t channel)
{
	uint32_t tmp_adc = 0;
	uint16_t value;
	
	// настройка канала
	ADC_RegularChannelConfig(ADC1, channel, 1, ADC_SampleTime_55Cycles5);

	for( int i=0; i<AVERAGE_COUNT; i++ )
	{
		ADC_SoftwareStartConvCmd(ADC1, ENABLE);
		while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
		tmp_adc += ADC_GetConversionValue(ADC1);
		vTaskDelay( 5 );
	}
	
	value = tmp_adc / AVERAGE_COUNT;
	
	return value;
}

/*******************************************************
Поток		: Получение и обработка значений аналоговых входов
Параметр 1	: не используется
Возвр. знач.: бесконечный цикл
********************************************************/
void Thread_AnalogSensors( void *pvParameters )
{
	adc_Initialize();
	
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 100;
	xLastWakeTime = xTaskGetTickCount();
	
	for(;;)
	{
         // Wait for the next cycle.
		vTaskDelayUntil( &xLastWakeTime, xFrequency );
		
		g_SUN_AdcValue = adc_get_value( SUN_CHANNEL );
	}
}
