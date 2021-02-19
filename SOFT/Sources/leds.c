/***********************************************************************************
* ������������ ������	: ���������� ���������� ������������
*-----------------------------------------------------------------------------------
* ������				: 1.0
* �����					: ��������
* ���� ��������			: 23.05.2018
************************************************************************************/

//--- INCLUDES -------------------
#include "leds.h"

//--- CONSTANTS ------------------
const TLedDesc Led[] = {
	{PORT_LED_R, OUT_LED_R},
	{PORT_LED_G, OUT_LED_G},
	{PORT_LED_B, OUT_LED_B},
};

//--- GLOBAL VARIABLES -----------

//--- FUNCTIONS ------------------

/*******************************************************
�������		: ������������� ������
�������� 1	: ���
�����. ����.: ���
********************************************************/
void Leds_init(void)
{
	for( int i=0; i<LED_COUNT; i++ )
	{
		// ������� ������ ���������� �� ����� � �������� �����������
		GPIO_PinConfigure( Led[i].GPIOx, Led[i].num, GPIO_OUT_PUSH_PULL, GPIO_MODE_OUT2MHZ );
		// ���������� ���������� (����������� ���. 1)
		GPIO_PinWrite( Led[i].GPIOx, Led[i].num, 1 );
	}
}

/*******************************************************
�������		: ��������� ����������
�������� 1	: ����� ����������
�����. ����.: ���
********************************************************/
void Led_On(uint8_t led_num)
{
	if(( led_num < LED_RED ) || ( led_num > LED_BLUE ))
		return;
	
	// ���������� ���. �����
	GPIO_PinWrite( Led[led_num-1].GPIOx, Led[led_num-1].num, 0 );
}

/*******************************************************
�������		: ���������� ����������
�������� 1	: ����� ����������
�����. ����.: ���
********************************************************/
void Led_Off(uint8_t led_num)
{
	if(( led_num < LED_RED ) || ( led_num > LED_BLUE ))
		return;
	
	GPIO_PinWrite( Led[led_num-1].GPIOx, Led[led_num-1].num, 1 );
}

/*******************************************************
�������		: ���������� ��������� ����������
�������� 1	: ����� ����������
�����. ����.: ��������� ����������
********************************************************/
bool Led_IsOn(uint8_t led_num)
{
    bool isHigh;
	
	if(( led_num < LED_RED ) || ( led_num > LED_GREEN ))
		return false;
	
    isHigh = (bool) GPIO_PinRead(Led[led_num-1].GPIOx, Led[led_num-1].num);

    return !isHigh;
}

/*******************************************************
�������		: ������������ ����������
�������� 1	: ����� ����������
�����. ����.: ���
********************************************************/
void Led_Switch(uint8_t led_num)
{
    if( Led_IsOn(led_num) ) 
        Led_Off(led_num);
    else
        Led_On(led_num);
}

/*******************************************************
�������		: ��������� ���� �����������
�������� 1	: ���
�����. ����.: ���
********************************************************/
void Leds_OnAll(void)
{
	for( int num = 1; num <= LED_COUNT; num++ ) 
	{
		Led_On( num );
	}
}

/*******************************************************
�������		: ���������� ���� �����������
�������� 1	: ���
�����. ����.: ���
********************************************************/
void Leds_OffAll(void)
{
	for( int num = 1; num <= LED_COUNT; num++ ) 
	{
		Led_Off( num );
	}
}
