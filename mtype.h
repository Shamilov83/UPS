/**
* описание типов для проекта 
*/

#ifndef _MTYPE_H
#define _MTYPE_H

#ifndef __STDINT_H_
typedef  unsigned char uint8_t;
typedef  uint8_t BYTE;

typedef  unsigned short uint16_t;
typedef  uint16_t WORD;

typedef  unsigned long uint32_t;
typedef  uint32_t DWORD;

#endif


typedef struct _CONF
{
	uint16_t brate;		// скорость передачи
	uint8_t adr;		// сетевой адрес
	uint16_t Uz;			//код напряжения окончания заряда
	uint16_t Um;			//код напряжения начала заряда
}conf;

typedef struct _CONF	conf;

#endif // end _MTYPE_H