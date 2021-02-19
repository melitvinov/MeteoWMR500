/*****************************************************************************************
  Описание типов
*****************************************************************************************/

#ifndef __mytypes
#define __mytypes

#define     TRUE   1
#define     FALSE  0

typedef unsigned char   BOOL;

typedef unsigned char   BYTE;
typedef unsigned short  WORD;
typedef unsigned long   DWORD;

typedef unsigned char*  PBYTE;
typedef unsigned short* PWORD;
typedef unsigned long*  PDWORD;

typedef struct __STRING  {
  WORD  wLen;           // 1..2 byte    // note: длина строки (буфера)
  WORD  wMaxLen;        // 3..4 byte    // note: MAX длина строки (буфера)
  PBYTE pBuf;           // 2 byte       // note: указатель на данные
} String;

//--> работа с байтами
  #define  LOW(a)                 ((a)&0xFF)
  #define  HIGH(a)                ((a)>>8)
  #define  MAX(a, b)              ((a)>(b)? (a):(b))
  #define  MIN(a, b)              ((a)>(b)? (b):(a))

#endif
