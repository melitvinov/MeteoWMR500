/***********************************************************************************
* ������������ ���� ���	: 
*
************************************************************************************/

#ifndef __MODBUS_H
#define __MODBUS_H

//--- INCLUDES -------------------

#include "GPIO_STM32F10x.h"
#include "stm32f10x.h"
#include "Board.h"

// --- DEFINES -------------------

#define PACK_HEADER_CRC_LEN			4							// ����� ��������� ������ + ����� ���� CRC
#define MIN_MB_PACK_LEN				(PACK_HEADER_CRC_LEN+1)		// ����������� ����� ������ ��������� modbus
#define MBUS_MAX_MSGLEN				250							// ������������ ����� ������� � ������ �� ���� modbus
#define MBUS_MAX_REQREGS			100							// ����������� ����� ������ - ����������� ��������� ���-�� ��������� � ������

#define MBUS_PAKETS_TIMEOUT			3							// ����� ���� ���� ����� �������� modbus (��.)
#define TIMER_PRESCALER				8000

// --- TYPES ---------------------

#pragma pack(push,1)

// ��������� ������ � ������� �� ������ ���������
typedef struct __req_read_holding_regs
{
	uint16_t startAddr;
	uint16_t countRegs;
} TReqReadHoldRegsHeader;

// ��������� ������ � ������� �� ������ ���������
typedef struct __req_write_regs
{
	uint16_t startAddr;
	uint16_t countRegs;
	uint8_t bytesCount;
} TReqWriteRegsHeader;

// ����� ��� �������� ������ modbus
typedef struct __mb_packet_buffer
{
	uint8_t msgLen;
	uint8_t msgData[MBUS_MAX_MSGLEN];
} TPacketBuffer;

// ��������� ������ �� modbus
typedef enum __modbus_states
{
	STATE_STOP = 0,		// ����� ����������
	STATE_RECEIVE		// ����� ������
} EMBusState;

// ����������� ��������� ������
typedef struct __mb_header
{
	uint8_t  	slaveId;
	uint8_t		funcId;
	uint8_t		firstdataByte;
} TMB_Header, *PMB_Header;

// ��������� ������ ��������� �� ������
typedef struct __mb_err_answer
{
	uint8_t		devAddr;
	uint8_t		errCmd;
	uint8_t		errCode;
	uint16_t	crc;
} TErrAnswer;

// ������ ������ modbus
typedef enum __cmd_exec_err
{
	err_FuncId		= 1,	// �������� ��� ������� �� ����� ���� ���������
	err_DataAddr	= 2,	// ����� ������, ��������� � �������, ����������
	err_Value		= 3,	// ��������, ������������ � ���� ������ �������, �������� ������������ ���������
	err_Error		= 4,	// ������������������� ������ ����� �����, ���� ������� ���������� �������� ��������� ������������� ��������
	err_Wait		= 5,	// ������� ���������� ������� ������ � ������������ ���, �� ��� ������� ����� �������. ���� ����� ������������ ������� ���������� �� ��������� ������ ����-����
	err_Busy		= 6,	// ������� ���������� ������ ���������� �������. ������� ���������� ������ ��������� ��������� �����, ����� ������� �����������
	err_ExecFunc	= 7,	// ������� ���������� �� ����� ��������� ����������� �������, �������� � �������. ���� ��� ������������ ��� ����������� ������������ �������, ������������� ������� � �������� 13 ��� 14. ������� ���������� ������ ��������� ��������������� ���������� ��� ���������� �� ������� �� ��������.
	err_Mem			= 8		// ������� ���������� ��� ������ ����������� ������ ���������� ������ ��������. ������� ���������� ����� ��������� ������, �� ������ � ����� ������� ��������� ������.	
} EFuncResult;

// �������������� ������� � ��������
typedef enum __valid_commands
{
	READ_HOLDING_REGS 	= 0x03,		// Read Holding Registers
	WRITE_MULTIPLE_REGS	= 0x10		// Preset Multiple Registers
} EValidCmd;


#pragma pack(pop)

//--- FUNCTIONS ------------------
void MBUS_Thread( void *pvParameters );

bool Write_MultipleRegs( uint8_t * data, uint8_t datalen );
bool Read_HoldingRegs( uint8_t * data, uint8_t datalen );
#endif
