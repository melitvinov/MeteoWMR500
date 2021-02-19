/*****************************************************************************************
Функции для датчика температуры и влажности SHT21 

Начало -      11.02.2021

*****************************************************************************************/

#include "SHT21.h"

static bool SHT21_initialized;

// ------------- Функции I2C ------------------------
void i2c_init()
{
	// Установки для выводов I2C
	GPIO_PinConfigure( PORT_I2C, PIN_SCL, GPIO_OUT_OPENDRAIN, GPIO_MODE_OUT2MHZ );
	GPIO_PinConfigure( PORT_I2C, PIN_SDA, GPIO_IN_FLOATING, GPIO_MODE_INPUT );
	
	GPIO_PinWrite( PORT_I2C, PIN_SCL, 1 );
	GPIO_PinWrite( PORT_I2C, PIN_SDA, 1 );
}

void i2c_start()
{
	GPIO_PinConfigure( PORT_I2C, PIN_SDA, GPIO_OUT_OPENDRAIN, GPIO_MODE_OUT2MHZ );
	GPIO_PinWrite( PORT_I2C, PIN_SDA, 1 );
	GPIO_PinWrite( PORT_I2C, PIN_SCL, 1 );
	GPIO_PinWrite( PORT_I2C, PIN_SDA, 0 );
	GPIO_PinWrite( PORT_I2C, PIN_SCL, 0 );
}

void i2c_stop()
{
	GPIO_PinWrite( PORT_I2C, PIN_SDA, 0 );
	GPIO_PinWrite( PORT_I2C, PIN_SCL, 0 );
	GPIO_PinConfigure( PORT_I2C, PIN_SDA, GPIO_OUT_OPENDRAIN, GPIO_MODE_OUT2MHZ );
	GPIO_PinWrite( PORT_I2C, PIN_SCL, 1 );
	GPIO_PinWrite( PORT_I2C, PIN_SDA, 1 );
	GPIO_PinConfigure( PORT_I2C, PIN_SDA, GPIO_IN_FLOATING, GPIO_MODE_INPUT );
}

void i2c_ask()
{
	GPIO_PinConfigure( PORT_I2C, PIN_SDA, GPIO_OUT_OPENDRAIN, GPIO_MODE_OUT2MHZ );
	GPIO_PinWrite( PORT_I2C, PIN_SDA, 0 );
	GPIO_PinWrite( PORT_I2C, PIN_SCL, 1 );
	GPIO_PinWrite( PORT_I2C, PIN_SCL, 0 );
}

void i2c_sendBit( uint8_t bit )
{
	if( bit == 0 )	GPIO_PinWrite( PORT_I2C, PIN_SDA, 0 );
	else			GPIO_PinWrite( PORT_I2C, PIN_SDA, 1 );
	GPIO_PinWrite( PORT_I2C, PIN_SCL, 1 );
	GPIO_PinWrite( PORT_I2C, PIN_SCL, 0 );
}

bool i2c_sendByte( uint8_t byte )
{
	bool ask = false;
	GPIO_PinConfigure( PORT_I2C, PIN_SDA, GPIO_OUT_OPENDRAIN, GPIO_MODE_OUT2MHZ );

	for( int i=7; i>=0; i-- )
	{
		i2c_sendBit( GETBIT(byte,i) );
	}
	// ask
	GPIO_PinConfigure( PORT_I2C, PIN_SDA, GPIO_IN_FLOATING, GPIO_MODE_INPUT );
	//GPIO_PinWrite( PORT_I2C, PIN_SCL, 1 ); // здесь SCL вроде не нужен судя по описанию
	if( 0 == GPIO_PinRead( PORT_I2C, PIN_SDA ) )
		ask = true;
	//GPIO_PinWrite( PORT_I2C, PIN_SCL, 0 );
	return ask;
}

uint8_t i2c_readBit()
{
	GPIO_PinWrite( PORT_I2C, PIN_SCL, 1 );
	uint8_t bit = GPIO_PinRead( PORT_I2C, PIN_SDA );
	GPIO_PinWrite( PORT_I2C, PIN_SCL, 0 );
	return bit;
}

uint8_t i2c_readByte(void)
{
	uint8_t bit, byte = 0;
	
	GPIO_PinConfigure( PORT_I2C, PIN_SDA, GPIO_IN_FLOATING, GPIO_MODE_INPUT );

	for( int i=7; i>=0; i-- )
	{
		bit = i2c_readBit();
		byte |= bit << i;
	}
	
	return byte;
}

// ------------- Конец функций I2C ------------------

/*****************************************************************************************
InitSHT21 - инициализация SHT21
******************************************************************************************/
void SHT21_Init( void )
{
	if( !SHT21_initialized )
	{
		i2c_init();
		SHT21_initialized = true;
	}
} 
//<-- Конец функции SHT21_Init -----------------------------------------------------------

bool SHT21_sendAddress( uint8_t operation )
{
	uint8_t byte_to_send = SHT_ADDRESS | operation;
	
	bool ask = i2c_sendByte( byte_to_send );

	return ask;
}

bool sht21_wait_complete( void )
{
	bool result = false;
	
	// Здесь ждем когда на линии данных появится единица
	int restDelay = 100;	// ждем максимум 100 мс.
	const int time_step = 10;	// интервал проверок в мс.
	while( restDelay > 0 || result )
	{
		vTaskDelay( time_step );
		if( GPIO_PinRead( PORT_I2C, PIN_SDA ) > 0 )
			result = true;
		restDelay -= time_step;
	}		
	
	return result;
}

void sht21_read_data( uint8_t * data, int datalen )
{
	if( !data || datalen < 3 )
		return;
	
	for( int i=0; i<3; i++ )
	{
		data[i] = i2c_readByte();
		i2c_ask();
	}
}

bool SHT21_Read_RH( float * pValue )
{
	bool result = false;
	uint8_t data[3];
	float flValue;
	
	if( pValue )
	{
		if( !SHT21_initialized ) 
			SHT21_Init();
		
		i2c_start();
		
		if( SHT21_sendAddress( OPERATION_WRITE ) )
		{
			if( i2c_sendByte( CMD_RH_MEASUREMENT_HOLD ) )
			{
				if( sht21_wait_complete() )
				{
					sht21_read_data( data, 3 );
					uint16_t uiValue = *( (uint16_t*) &data[0] );
					
					flValue = (uiValue & 0xFFFC); 
					flValue /= 65536;
					flValue *= 125;
					flValue -= 6;
					*pValue = flValue;
					result = true;
				}
			}
		}
	}
	
	return result;
}

bool SHT21_Read_T( float * pValue )
{
	bool result = false;
	uint8_t data[3];
	float flValue;
	
	if( pValue )
	{
		if( !SHT21_initialized ) 
			SHT21_Init();
		
		i2c_start();
		
		if( SHT21_sendAddress( OPERATION_WRITE ) )
		{
			if( i2c_sendByte( CMD_T_MEASUREMENT_HOLD ) )
			{
				if( sht21_wait_complete() )
				{
					sht21_read_data( data, 3 );
					uint16_t uiValue = *( (uint16_t*) &data[0] );
					
					flValue = (uiValue & 0xFFFC); 
					flValue /= 65536;
					flValue *= 175.72;
					flValue -= 46.85;
					*pValue = flValue;
					result = true;
				}
			}
		}
	}
	
	return result;
}

