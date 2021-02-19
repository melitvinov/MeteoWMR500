/***********************************************************************************
* ������������ ������	: MODBUS Protocol
*-----------------------------------------------------------------------------------
* ������				: 1.0
* �����					: ��������
* ���� ��������			: 25.05.2018
************************************************************************************/

//--- INCLUDES -------------------
#include "FreeRTOS.h"
#include "task.h"

#include "modbus.h"

#include "leds.h"
#include "modbus_regs.h"

// --- DEFINES -------------------

// --- TYPES ---------------------

//--- CONSTANTS ------------------

//--- FORWARD --------------------
bool Parser( TPacketBuffer * pMsg );

void timerInit(void);
void startTimer(void);
void stopTimer(void);
void restartWaitTimer(void);

void startReceiveMessages(void);
void resetInputPacket(void);
uint16_t calcCRC( uint8_t * data, uint8_t len );
bool isCrcOk( TPacketBuffer * pMsg );

void On_Usart_ReceiveChar( char chr );
void On_Timer( void );

bool makeErrorAnswer( uint8_t cmd, uint8_t err );
void makeModbusAnswer( uint8_t slaveId, uint8_t funcId, uint8_t * data, uint8_t datalen );

//--- GLOBAL VARIABLES -----------
//EMBusState MbusState;
TPacketBuffer Mbus_RecvMsg;
TPacketBuffer Mbus_Answer;
TIM_TimeBaseInitTypeDef timer;
bool IsPacketReady;

extern uint16_t g_Status;

//--- IRQ ------------------------
// ������
void TIM4_IRQHandler()
{
    TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
	On_Timer();
}

//--- EXTERN ---------------------
extern uint8_t GetDeviceAddress(void);
extern void Rs485_Init(void);
extern void RS485_SendBuf( TPacketBuffer * buffer );

//--- FUNCTIONS ------------------

/*******************************************************
�������		: ������������ �������� ����
�������� 1	: ���� ������
�����. ����.: ���
********************************************************/
void On_Usart_ReceiveChar( char chr )
{
	if( !IsPacketReady )
	{
		
		TIM_Cmd(TIM4, DISABLE);
		TIM_SetCounter(TIM4, 0);
		TIM_Cmd(TIM4, ENABLE);
		
		if( Mbus_RecvMsg.msgLen >= MBUS_MAX_MSGLEN )
		{
			// ������ ������! ������������ ��������� ������
			// ������� �����
			resetInputPacket();
		}

		Mbus_RecvMsg.msgData[Mbus_RecvMsg.msgLen++] = chr;
	}
}

/*******************************************************
�������		: ������������� �������
�������� 1	: ���
�����. ����.: ���
********************************************************/
void timerInit()
{
	//�������� ������������ ������� TIM4
    //������ 4 � ��� ����� �� ���� APB1
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	//� ��� ��������� �������
    //��������� ���� ��������� ���������� ����������
    TIM_TimeBaseStructInit(&timer);
    //���������� ������������
    timer.TIM_Prescaler = TIMER_PRESCALER;
    //��� ��������, �������� �� �������� ������ ����������� ����������
    //������ ��� �������� �� ����� ������ � ����� ����������
    timer.TIM_Period = MBUS_PAKETS_TIMEOUT;
    //�������������� TIM4 ������ ����������
    TIM_TimeBaseInit(TIM4, &timer);	
	//����������� ������ ��� ��������� ���������� �� ���������� (������������)
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
    //��������� ��������������� ����������
    NVIC_EnableIRQ(TIM4_IRQn);
}

/*******************************************************
�������		: ������������� ����� �� modbus
�������� 1	: ���
�����. ����.: ���
********************************************************/
void On_Timer()
{
	// ������������� ������ 
	stopTimer();

	IsPacketReady = true;
}

/*******************************************************
�������		: �������� ��������� ��������� ��������� modbus
�������� 1	: ���
�����. ����.: ���
********************************************************/
void startReceiveMessages(void)
{
	// ������������� ������
	stopTimer();
	
	// ������� ����� �������� ������
	resetInputPacket();

	// ���������� ���� ������ ������
	IsPacketReady = false;
}

/*******************************************************
�������		: �������������� ����� modbus �� ������� ������
			  ��������� ��������� � ����. ���������� Mbus_Answer
�������� 1	: ����� ����������
�������� 2	: ��� ������� (���� ������� ������ ���� ���� 0x80)
�������� 3	: ������ ������
�������� 4	: ����� ������
�����. ����.: ���
********************************************************/
void makeModbusAnswer( uint8_t slaveId, uint8_t funcId, uint8_t * data, uint8_t datalen )
{
	// �������� ������� ����������
	if( datalen > 0 && !data )
		return;
	
	// �������� ����� ������
	if( datalen > MBUS_MAX_MSGLEN - sizeof(TMB_Header) - sizeof(uint16_t)/*CRC*/ )
		return;
	
	// ������ ����� ������
	memset( &Mbus_Answer, 0, sizeof(TPacketBuffer) );

	// ��������� ���������
	PMB_Header pHeader = (PMB_Header)  &(Mbus_Answer.msgData[0]);
	pHeader->slaveId = slaveId;
	pHeader->funcId = funcId;
	
	// ��������� ���� ������
	if( datalen )
	for(int i=0; i<datalen; i++)
	{
		memcpy( &(pHeader->firstdataByte), data, datalen );
	}
	
	// �������� ������� ����� ������
	Mbus_Answer.msgLen = sizeof(TMB_Header) - 1 + datalen;
	
	// ������� ����������� �����
	uint16_t crc = calcCRC( Mbus_Answer.msgData, Mbus_Answer.msgLen );
	
	// ���������� CRC � ����� ������
	*((uint16_t*) &(Mbus_Answer.msgData[Mbus_Answer.msgLen])) = crc;
	
	// ����������� ����� ������ �� ������ CRC
	Mbus_Answer.msgLen += sizeof(uint16_t);
	
	// ������! �������������� ����� ����� � Mbus_Answer, ����� ���������.
}

/*******************************************************
�����		: ���������������� modbus
�������� 1	: �� ������������
�����. ����.: ����������� ����
********************************************************/
void MBUS_Thread( void *pvParameters )
{
	// ������������� modbus
	
	// ������
	timerInit();
	stopTimer();
	
	// ������������� RS-485, ���������� ���������� �� USART
	Rs485_Init();
	
	g_Status = 0;
	
	Led_Off(LED_GREEN);

	// �������� ����� �������
	startReceiveMessages();
	// ����������� ���� ������ modbus
	for(;;)
	{
		vTaskDelay(1);

		if( IsPacketReady )
		{
			IsPacketReady = false;
			
			Led_On(LED_GREEN);
			
			if( Parser( &Mbus_RecvMsg ) )
			{
				Led_On(LED_BLUE);
				
				RS485_SendBuf( &Mbus_Answer );
			}
			
			startReceiveMessages();

			Led_Off(LED_GREEN);
			Led_Off(LED_BLUE);
		}
	}
}


/*******************************************************
�������		: ������������� �������� CRC ��� ���������� �����
�������� 1	: ���������� �������� CRC
�������� 2	: ��������� ����
�����. ����.: CRC
********************************************************/
uint16_t CRC16(uint16_t crc, uint16_t data)
{
	const unsigned int Poly16=0xA001;
	unsigned int LSB, i;
	crc = ((crc^data) | 0xFF00) & (crc | 0x00FF);
	for (i=0; i<8; i++) {
		LSB=(crc & 0x0001);
		crc=crc/2;
		if (LSB)
		crc=crc^Poly16;
	}
	return (crc);
}

/*******************************************************
�������		: ��������� CRC ������ modbus
�������� 1	: ��������� �� ������ ������ ������
�������� 2	: ����� ��������� ������ � ������ (��� CRC)
�����. ����.: CRC
********************************************************/
uint16_t calcCRC( uint8_t * data, uint8_t len )
{
	unsigned int crc = 0xFFFF;
	
	for (uint8_t i = 0; i < len; i++) 
	{
		crc = CRC16 (crc, data[i] );
	}
	
	return crc;
}
 
/*******************************************************
�������		: ������ �������� Modbus �����, ���������� ��������� �� �����
�������� 1	: ��������� �� ������ ������ ��������� ������
�������� 2	: ����� ��������� ������ � ������
�����. ����.: ��������� �� �����
********************************************************/
bool isCrcOk( TPacketBuffer * pMsg )
{
	uint16_t crcInPacket = * ((uint16_t*) &(pMsg->msgData[pMsg->msgLen - 2]));
	uint16_t crcCalculated = calcCRC( (uint8_t*) pMsg->msgData, pMsg->msgLen - 2 );
	
	return ( crcInPacket == crcCalculated );
}

/*******************************************************
�������		: �������������� ����� � ��������� ������
�������� 1	: ������� �� �������
�������� 2	: ����� ������
�����. ����.: ���������� ���� - ����� �����������
********************************************************/
bool makeErrorAnswer( uint8_t cmd, uint8_t err )
{
	TErrAnswer errAnswer;
	
	errAnswer.devAddr = GetDeviceAddress();
	errAnswer.errCmd = (cmd & 0x3F) | 0x80;
	errAnswer.errCode = err;
	errAnswer.crc = calcCRC( (uint8_t *) &errAnswer, sizeof(TErrAnswer) - sizeof(uint16_t) );
	
	memset( &Mbus_Answer, 0, sizeof(Mbus_Answer) );
	memcpy( &(Mbus_Answer.msgData[0]), &errAnswer, sizeof(TErrAnswer) );
	Mbus_Answer.msgLen = sizeof(TErrAnswer);
	
	return true;
}

/*******************************************************
�������		: ������ ��������e �������(��������)
�������� 1	: ��������� �� ������ ������ �������
�������� 2	: ����� ������� � ������
�����. ����.: ����� ����������� 
********************************************************/
bool Read_HoldingRegs( uint8_t * data, uint8_t datalen )
{
	if( !data || datalen != 4 )
		return false;
	
	// ������������� ������
	TReqReadHoldRegsHeader * pReadReq = (TReqReadHoldRegsHeader *) data;
	// ������ ������� ����
	pReadReq->countRegs = SWAP_W(pReadReq->countRegs);
	pReadReq->startAddr = SWAP_W(pReadReq->startAddr);
	
	if( pReadReq->countRegs > MBUS_MAX_REQREGS )
	{
		// ����� �� ���������� � �����
		// �������� ������� ������� ���������� �������
		makeErrorAnswer( READ_HOLDING_REGS, err_Value );
		return true;
	}
	
	// ����� ��� ���������� �������� ����������� ���������
	uint8_t buffer[MBUS_MAX_REQREGS*2 + 1];
	uint16_t * pRegs = ( uint16_t* ) &(buffer[1]);
	uint8_t indxReg = 0;
	
	// ������ ����������� �������� � �����
	for( uint16_t regAdr = pReadReq->startAddr; regAdr < pReadReq->startAddr + pReadReq->countRegs; regAdr++, indxReg++ )
	{
		
		if( !REG_Read( regAdr, &(pRegs[indxReg]) ) )
		{
			// ���� �� ������� ��������� ����������� ������� 
			// ���������� ������ ������ ��������
			makeErrorAnswer( READ_HOLDING_REGS, err_DataAddr );
			return true;
		}
		// ������ ������� ���� � ���������� ��������
		pRegs[indxReg] = SWAP_W(pRegs[indxReg]);
	}

	// ��� �������� ���������, �������� �������� ���-�� ����
	buffer[0] = pReadReq->countRegs * 2;
	
	// ��������� ����� modbus
	
	makeModbusAnswer( GetDeviceAddress(), READ_HOLDING_REGS, buffer, pReadReq->countRegs * 2 + 1 );
	
	return true;
}

/*******************************************************
�������		: ���������� ��������e �������(��������)
�������� 1	: ��������� �� ������ ������ �������
�������� 2	: ����� ������� � ������
�����. ����.: ����� ����������� 
********************************************************/
bool Write_MultipleRegs( uint8_t * data, uint8_t datalen )
{
	if( !data || datalen < sizeof(TReqWriteRegsHeader) + sizeof(uint16_t) )
		return false;
	
	// ������������� ������
	TReqWriteRegsHeader * pWriteReq = (TReqWriteRegsHeader *) data;
	// ������ ������� ����
	pWriteReq->countRegs = SWAP_W(pWriteReq->countRegs);
	pWriteReq->startAddr = SWAP_W(pWriteReq->startAddr);
	
	if(	( pWriteReq->countRegs > MBUS_MAX_REQREGS ) || 
		( pWriteReq->countRegs != pWriteReq->bytesCount / 2 ) )
	{
		// ���� ����� �� ���������� � ����� ��� � ������� ����������� ������� ���-�� ���� ������
		// �������� ������� ������� ���������� �������
		makeErrorAnswer( WRITE_MULTIPLE_REGS, err_Value );
		return true;
	}

	// ��������� ����������� �������� �� ������� � ����������� ������
	for( int i=0; i<pWriteReq->countRegs; i++ )
	{

		if( !REG_isWriteEnable( pWriteReq->startAddr + i ) )
		{
			// ���� ����� �� ��������� ����������� ��� ������ � ���� ��������� 
			// �� ���������� ������ ���������� �������
			makeErrorAnswer( WRITE_MULTIPLE_REGS, err_Value );
			return true;
		}
	}
	
	// ��� �������� � ������� � ������ � ��� ���������, �����
	uint16_t * pValues = (uint16_t*) ( &(pWriteReq->bytesCount) + 1);
	for( int i=0; i<pWriteReq->countRegs; i++ )
	{
		REG_Write( pWriteReq->startAddr + i, SWAP_W( pValues[i] ) );
	}
	
	// ��������� ����� modbus
	uint16_t answer[2];
	answer[0] = SWAP_W( pWriteReq->startAddr );
	answer[1] = SWAP_W( pWriteReq->countRegs );
	
	makeModbusAnswer( GetDeviceAddress(), WRITE_MULTIPLE_REGS, (uint8_t*) answer, sizeof(answer) );
	
	return true;
}

/*******************************************************
�������		: ������ �������� Modbus �����, ���������� ��������� �� �����
�������� 1	: ��������� �� ������ ������ ��������� ������
�������� 2	: ����� ��������� ������ � ������
�����. ����.: ��������� �� �����
********************************************************/
bool Parser( TPacketBuffer * pMsg )
{
	// �������� ������� ������
	if( !pMsg )
		return 0;
	
	// �������� CRC ������ �������
	if( !isCrcOk(pMsg) )
		return 0;
	
	// �������� ���������� ������ � ������� � ������� ����������
	PMB_Header pHeader = (PMB_Header) pMsg->msgData;
	if( pHeader->slaveId != GetDeviceAddress() )
		return 0;

	bool isNeedSendAnswer = false;
	switch( pHeader->funcId )
	{
		case READ_HOLDING_REGS:
			isNeedSendAnswer = Read_HoldingRegs( &(pHeader->firstdataByte), pMsg->msgLen - PACK_HEADER_CRC_LEN );
			break;
		case WRITE_MULTIPLE_REGS:
			isNeedSendAnswer = Write_MultipleRegs( &(pHeader->firstdataByte), pMsg->msgLen - PACK_HEADER_CRC_LEN );
			break;
		default:
			isNeedSendAnswer = makeErrorAnswer( pHeader->funcId, err_FuncId );
			break;
	}
	
	return isNeedSendAnswer;
}

/*******************************************************
�������		: ������������� ������ 
�������� 1	: ���
�����. ����.: ���
********************************************************/
void stopTimer()
{
    TIM_Cmd(TIM4, DISABLE);
	TIM_SetCounter(TIM4,0);
}

/*******************************************************
�������		: ��������� ������ �� ����������� �����
				����� ����� �������� modbus 
�������� 1	: ���
�����. ����.: ���
********************************************************/
void startTimer()
{
    //��������� ������ 
	TIM_SetCounter(TIM4,0);
    TIM_Cmd(TIM4, ENABLE);
}

void restartWaitTimer(void)
{
	TIM_Cmd(TIM4, DISABLE);
	TIM_SetCounter(TIM4, 0);
    TIM_Cmd(TIM4, ENABLE);
}

/*******************************************************
�������		: ������� ����� ���������� ������ modbus
�������� 1	: ���
�����. ����.: ���
********************************************************/
void resetInputPacket()
{
	//memset( Mbus_RecvMsg.msgData, 0, MBUS_MAX_MSGLEN );
	Mbus_RecvMsg.msgLen = 0;
	Mbus_Answer.msgLen = 0;
}

