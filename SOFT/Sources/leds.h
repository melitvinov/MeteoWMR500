/***********************************************************************************
* Заголовочный файл для	: модуля системных светодиодов
*
************************************************************************************/

#ifndef __LEDS_H_
#define __LEDS_H_

//--- INCLUDES -------------------

#include "GPIO_STM32F10x.h"
#include "stm32f10x.h"
#include "Board.h"

// --- DEFINES -------------------
#define LED_COUNT	3

// --- TYPES ---------------------

//-- Номера светодиодов
enum 
{
	LED_RED        = 1,
	LED_GREEN      = 2,
	LED_BLUE	   = 3
};

typedef struct __leddesc
{
	GPIO_TypeDef *GPIOx;
	uint32_t num;
} TLedDesc;

//--- FUNCTIONS ------------------

void Leds_init(void);
bool Led_IsOn(uint8_t led_num);
void Led_On(uint8_t led_num);
void Led_Off(uint8_t led_num);
void Led_Switch(uint8_t led_num);
void Leds_OnAll(void);
void Leds_OffAll(void);

#endif
