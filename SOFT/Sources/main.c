/***********************************************************************************
* Наименование : Проект "Метеостанция WMR500"
*-----------------------------------------------------------------------------------
* Версия				: 1.0
* Автор					: Тимофеев
* Дата создания			: 10.02.2021
************************************************************************************/

//--- INCLUDES -------------------
#include "stdio.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "Leds.h"
#include "RS485.h"
#include "modbus.h"
#include "AnaInputs.h"
#include "AddrSwitch.h"
#include "SHT31.h"

//--- CONSTANTS ------------------
const GPIO_PIN_ID ILeds[] = {
	{PORT_InfLed_1, PIN_InfLed_1},
	{PORT_InfLed_2, PIN_InfLed_2},
	{PORT_InfLed_3, PIN_InfLed_3},
	{PORT_InfLed_4, PIN_InfLed_4},
};

const int ILEDS_COUNT = sizeof( ILeds ) / sizeof( GPIO_PIN_ID );
	
//--- GLOBAL VARIABLES -----------
uint8_t g_DeviceAddr = 0;	// текущий адрес устройства на шине RS-485
bool g_isAddressChanged = true;

uint16_t g_Status = 0;
uint16_t g_WIND_value = 0;
uint16_t g_RAIN_value = 0;
uint16_t g_WIND_ANGLE_value = 0;
uint16_t g_RH_flag = 1;
uint16_t g_RH_Value = 0;
uint16_t g_T_flag = 1;
uint16_t g_T_Value = 0;

int g_rain_count = 0; 
int g_wind_count = 0; 

//--- FUNCTIONS ------------------
void Thread_DigitalSensors( void *pvParameters );

/*----------------------------------------------------------------------------
  EXTI15..10 Interrupt Handler for INP_RAIN and INP_WIND connected to GPIOC.13 GPIOC.14
 *----------------------------------------------------------------------------*/
void EXTI15_10_IRQHandler(void)
{
	if( EXTI->PR & RAIN_EXTI ) 			// RAIN_EXTI interrupt pending?
	{	
		EXTI->PR = RAIN_EXTI;  			// clear pending interrupt
		g_rain_count++;
	}
	if( EXTI->PR & WIND_EXTI ) 			// WIND_EXTI interrupt pending?
	{	
		EXTI->PR = WIND_EXTI;			// clear pending interrupt
		g_wind_count++;
	}
}

/*******************************************************
Поток		: Мониторинг изменения адреса на шине RS-485
Параметр 1	: не используется
Возвр. знач.: бесконечный цикл
********************************************************/
void ThreadCheckAddrChange( void *pvParameters )
{
	for(int i=0; i<5; i++ )
	{
		Leds_OnAll();
		vTaskDelay(100);
		Leds_OffAll();
		vTaskDelay(100);
	}
	
	xTaskCreate( Thread_AnalogSensors, (const char*)"Analog", configMINIMAL_STACK_SIZE,	( void * ) NULL, ( tskIDLE_PRIORITY + 1 ), NULL);
	
	xTaskCreate( Thread_DigitalSensors, (const char*)"Digital", configMINIMAL_STACK_SIZE,	( void * ) NULL, ( tskIDLE_PRIORITY + 2 ), NULL);
	
	xTaskCreate( MBUS_Thread, (const char*)"MODBUS", 2048, ( void * ) NULL, ( tskIDLE_PRIORITY + 1 ), NULL);
	
	
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 250;
	
	 // Initialise the xLastWakeTime variable with the current time.
	 xLastWakeTime = xTaskGetTickCount();

	for(;;)
	{
         // Wait for the next cycle.
		vTaskDelayUntil( &xLastWakeTime, xFrequency );

		uint16_t addr = ADRSW_GetAdr();
		if( addr != g_DeviceAddr )
		{
			vTaskDelay(2000);
			if( addr == ADRSW_GetAdr() )
			{
				// адрес устройства изменился, меняем и показываем сообщение на LCD
				g_DeviceAddr = addr;
				g_isAddressChanged = true;
				uint8_t nums = g_DeviceAddr - ADDR_SHIFT_VALUE;
				
				for( int i=0; i < nums; i++ )
				{
					Led_On( LED_BLUE );
					vTaskDelay( 200 );
					Led_Off( LED_BLUE );
					vTaskDelay( 200 );
				}
			}
		}
	}
}

/*******************************************************
Функция		: Инициализация выводов для датчика направления ветра
Параметр 1	: нет
Возвр. знач.: нет
********************************************************/
void Wind_Angle_Init(void)
{
	for( int i=0; i < ILEDS_COUNT; i++ )
	{
		// Перевод вывода светодиода на выход с открытым коллектором
		GPIO_PinConfigure( ILeds[i].port, ILeds[i].num, GPIO_OUT_OPENDRAIN, GPIO_MODE_OUT2MHZ );
		GPIO_PinWrite( ILeds[i].port, ILeds[i].num, 1 );
	}

	// Перевод вывода фотодиода на вход
	GPIO_PinConfigure( PORT_InfPhoto, PIN_InfPhoto, GPIO_IN_FLOATING, GPIO_MODE_INPUT );
}

/*******************************************************
Функция		: Инициализация и настройка прерываний для датчиков дождя и ветра
Параметр 1	: нет
Возвр. знач.: нет
********************************************************/
void DigInputs_Int_Init(void)
{
	// Инициализация выводов входов датчиков

	// Тактирование портов
	GPIO_PortClock( INP_WIND_PORT, ENABLE );
	GPIO_PortClock( INP_RAIN_PORT, ENABLE );
	
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN; //Тактирование AFIO
	
	// Вход скорости ветра
	GPIO_PinConfigure( INP_WIND_PORT, INP_WIND_PIN, GPIO_IN_PULL_UP, GPIO_MODE_INPUT );
	// Вход датчика дождя
	GPIO_PinConfigure( INP_RAIN_PORT, INP_RAIN_PIN, GPIO_IN_PULL_UP, GPIO_MODE_INPUT );

	/* Add IRQ vector to NVIC */
	NVIC_InitTypeDef NVIC_InitStruct;
	
	NVIC_InitStruct.NVIC_IRQChannel = EXTI4_IRQn;
	/* Set priority */
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x00;
	/* Set sub priority */
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x00;
	/* Enable interrupt */
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	/* Add to NVIC */
	NVIC_Init(&NVIC_InitStruct);	

	/* Tell system that you will use PC13 for EXTI_Line0 */
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource13);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource14);
	
	// Настраиваем прерывания для датчиков
	EXTI_InitTypeDef exti_InitStruct;
	
	exti_InitStruct.EXTI_Line = WIND_EXTI | RAIN_EXTI;
	exti_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	exti_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;
	exti_InitStruct.EXTI_LineCmd = ENABLE;
	
	EXTI_Init( &exti_InitStruct );
	
	NVIC_EnableIRQ(EXTI15_10_IRQn);  //Разрешаем прерывание в контроллере прерываний
}

/*******************************************************
Функция		: Инициализация приложения
Параметр 1	: нет
Возвр. знач.: нет
********************************************************/
void Initialize()
{
	Leds_init();
	ADRSW_init();
	DigInputs_Int_Init();
	Wind_Angle_Init();
	
	__enable_irq ();
}

/*******************************************************
Функция		: Получения адреса устройства
Параметр 1	: нет
Возвр. знач.: Адрес устройства
********************************************************/
uint8_t GetDeviceAddress(void)
{
	return g_DeviceAddr;
}

/*******************************************************
Функция		: Точка старта приложения
Параметр 1	: нет
Возвр. знач.: бесконечный цикл
********************************************************/
int main(void)
{
	Initialize();
	
	xTaskCreate( ThreadCheckAddrChange, (const char*)"ADDR", configMINIMAL_STACK_SIZE,	( void * ) NULL, ( tskIDLE_PRIORITY + 1 ), NULL);

	/* Start the scheduler. */
	vTaskStartScheduler();

}

/*******************************************************
Функция		: Вычисляет направление ветра
Параметр 1	: нет
Возвр. знач.: направление ветра в градусах
********************************************************/
uint16_t get_wind_angle_value(void)
{
	uint16_t angle = 0;

	// Выключаем все инфракрасные светодиоды
//	for( int i=0; i < ILEDS_COUNT; i++ )
//	{
//		// отключаются лог. единицей
//		GPIO_PinWrite( ILeds[i].port, ILeds[i].num, 1 );
//	}

	// Включаем светодиоды по очереди и проверям состояние входа фотодиода
	for( int i=0; i < ILEDS_COUNT; i++ )
	{
		// Включаем светодиод
		GPIO_PinWrite( ILeds[i].port, ILeds[i].num, 0 );
		// Небольшая пауза
		vTaskDelay(10);
		// Проверка фотодиода
		if( (bool) GPIO_PinRead( PORT_InfPhoto, PIN_InfPhoto ) )
		{
			// Если фотодиод сработал, то:
			angle |= 1 << i;
		}
		
		// Отключаем светодиод
		GPIO_PinWrite( ILeds[i].port, ILeds[i].num, 1 );
	}
	
	switch( angle )
	{
		case 0:
			break;
		default:
			break;
	}
	
	return angle;
}

/*******************************************************
Поток		: Обрабатывает сигналы от датчика дождя
			: и датчика скорости ветра
Параметр 1	: не используется
Возвр. знач.: бесконечный цикл
********************************************************/
void Thread_DigitalSensors( void *pvParameters )
{
	const TickType_t xFrequency = 500;
	const TickType_t WIND_TIME_CALC = 2000; 	// время измерения скорости ветра в мс.
	const TickType_t NO_RAIN_TIME = 60;			// время измерения отсуствия дождя в сек.
	
	TickType_t meashure_wind_counter = 0;
	TickType_t meashure_norain_counter = 0;
	float flValueRH, flValueTemp;
	
	// Initialise the xLastWakeTime variable with the current time.
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	
	bool isShtArtCommandOk = false;
	
	for(;;)
	{
         // Wait for the next cycle.
		vTaskDelayUntil( &xLastWakeTime, xFrequency );

		Led_On( LED_RED );
		
		// Скорость ветра
		meashure_wind_counter += xFrequency;
		if( meashure_wind_counter >= WIND_TIME_CALC )
		{
			meashure_wind_counter = 0;
			
			// считаем текущую скорость ветра
			float imp_hour = (3600.0 / (WIND_TIME_CALC / 1000.0) ) * g_wind_count;
			g_wind_count = 0;
			float mile_hour = imp_hour / 1600.0;
			float meters_sec = mile_hour * 1609 / 3600;
			
			g_WIND_value = meters_sec * 100;
		}

		// Осадки
		if( g_rain_count )
		{
			g_rain_count = 0;
			g_RAIN_value = 1;
			meashure_norain_counter = 0;
		}
		else
		{
			meashure_norain_counter += xFrequency;
			if( meashure_norain_counter >= (NO_RAIN_TIME * 1000) )
			{
				meashure_norain_counter = 0;
				g_RAIN_value = 0;
			}
		}
		
		// Направление ветра
		g_WIND_ANGLE_value = get_wind_angle_value();
		
		// Температура и влажность
		bool isShtDataOk = false;
		
		if( !isShtArtCommandOk )
		{
			isShtArtCommandOk = SHT31_Send_ART_Command();
		}
		else if( SHT31_Read_SensorValues( &flValueRH, &flValueTemp ) )
		{
			g_RH_flag = 0;
			g_T_flag = 0;
			
			int16_t rh, t;
			rh = flValueRH * 10.0;
			t = flValueTemp * 10.0;
			
			g_RH_Value = rh;
			g_T_Value = t;
			isShtDataOk = true;
		}
		
		if( !isShtDataOk )
		{
			g_RH_flag = 1;
			g_T_flag = 1;
			g_RH_Value = 0;
			g_T_Value = 0;
		}
		
		Led_Off( LED_RED );
	}
}
