/*****************************************************************************************
Функции для датчика температуры и влажности SHT21 

Начало -      18.02.2021

*****************************************************************************************/

#include "SHT31.h"

static bool SHT_initialized;

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

uint8_t Crc8(uint8_t * data, uint16_t length )
{
    uint8_t crc = 0xff;
    for( int i=0; i<length; i++ )
    {
        crc ^= data[i];
        for (int j = 0; j < 8; j++)
        {
            if ((crc & 0x80) != 0)
                crc = (crc << 1) ^ 0x31;
            else
                crc <<= 1;
        }
    }

    return crc;
}

/*****************************************************************************************
InitSHT31 - инициализация SHT31
******************************************************************************************/
void SHT31_Init( void )
{
	if( !SHT_initialized )
	{
		i2c_init();
		
		SHT_initialized = SHT31_Send_ART_Command();
	}
} 

bool sht_sendAddress( uint8_t operation )
{
	uint8_t byte_to_send = SHT_ADDRESS | operation;
	
	bool ask = i2c_sendByte( byte_to_send );

	return ask;
}

void sht_read_data( uint8_t * data, int datalen )
{
	if( !data || datalen < 1 )
		return;
	
	for( int i=0; i<datalen; i++ )
	{
		data[i] = i2c_readByte();
		i2c_ask();
	}
}

/*****************************************************************************************
Command for a periodic data acquisition with the ART feature
******************************************************************************************/
bool SHT31_Send_ART_Command( void )
{
	bool isOk = false;

	if( sht_sendAddress( OPERATION_WRITE ) )
	{
		if( i2c_sendByte( 0x2B ) )
		{
			if( i2c_sendByte( 0x32 ) )
			{
				isOk = true;
			}
		}
	}
	
	return isOk;
}

/*****************************************************************************************
Считывает значения датчиков температуры и влажности с SHT31 в режиме ART
******************************************************************************************/
bool SHT31_Read_SensorValues( float * pRH, float * pTemp )
{
	bool isOk = false;
	
	float rh, t;
	uint8_t data[6];
	uint8_t crc;
	
	TDataStruct * pDataStruct;
	
	if( sht_sendAddress( OPERATION_READ ) )
	{
		sht_read_data( data, 6 );
		
		crc = Crc8( data, 2 );
		
		if( crc == data[2] )
		{
			crc = Crc8( &(data[3]), 2 );
			
			if( crc == data[5] )
			{
				pDataStruct = (TDataStruct*) &(data[0]);
				
				rh = SWAP_W( pDataStruct->uiHR );
				rh /= 65535;
				rh *= 100.0;
				*pRH = rh;
				
				t = SWAP_W( pDataStruct->uiTemperature );
				t /= 65535;
				t *= 175.0;
				t -= 45.0;
				*pTemp = t;
				
				isOk = true;
			}
		}
	}
	
	return isOk;
}


