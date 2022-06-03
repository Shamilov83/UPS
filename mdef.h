
#ifndef _MDEF_H
#define _MDEF_H

#define F_CPU 16000000UL //

#include <avr/io.h>

#include <util/delay.h>

#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>
#include <stdlib.h>
#include "mtype.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

//#define TYPE_DEV 5 // ��� ���������� (��� ���������)  ��������� ����

#define DEBUG 0			// ������� �������
#if DEBUG
#define M_SIZE 100		// ������ ������� ���� ��� ������ ���������� ����������
extern char deb_log[];
extern char *x,*y;
#endif




#define SetBit(x,y) 	(x|=(1<<y))
#define ClrBit(x,y) 	(x&=~(1<<y))
#define ToggleBit(x,y)	(x^=(1<<y))
#define FlipBit(x,y)	(x^=(1<<y))
#define TestBit(x,y)	(x&(1<<y))

#define Uout			PORTC1				//���������� �� ������
#define Uin				PORTC3				//���������� �� �����
#define Ubat			PORTC2				//���������� �������

#define ZAR				PORTD4				//���� ������ ������
#define preobr			PORTD3				//���� �������� ���������� � ��������������� ��� �� ���������������
#define OUT_BAT			PORTD5				//���� � ������� � ���������������
#define RAZR			PORTD6				//���� �������
#define Uvt9			PORTD7				//���� ��������



// ���������� �������
#define ZAR_ON()		SetBit(PORTD, ZAR);	// �������� �������
#define ZAR_OFF()		ClrBit(PORTD, ZAR);	// ��������� �������
#define RAZR_ON()		SetBit(PORTD,RAZR);	// �������� ��������
#define RAZR_OFF()		ClrBit(PORTD,RAZR);	// ��������� ��������
#define OUT_BAT_ON()	SetBit(PORTD,OUT_BAT);	//���������� ������� � ���������������
#define OUT_BAT_OFF()	ClrBit(PORTD,OUT_BAT);	//��������� ������� � ���������������
#define PREOBR_ON()		SetBit(PORTD,preobr);	//���������� ��������������� � �������� ���������
#define PREOBR_OFF()	ClrBit(PORTD,preobr);	//��������� ��������������� �� �������� ���������
#define NAGR_ON()		SetBit(PORTD,Uvt9);		//�������� ���� ��������
#define NAGR_OFF()		ClrBit(PORTD,Uvt9);		//��������� ���� ��������

#define DRAWE			PORTD2					// ���������� ��������� 485

#define KN				PORTB1					// ������ ���������
#define led_pwr			PORTB2					// ��������� ������ ������ �� ����
#define led_err			PORTB3					// ��������� ������
#define led_zar			PORTB4					// ��������� ������ ������
#define led_razr		PORTB5					// ��������� ������ �������
#define led_avton		PORTB0					// ��������� ����������� ������

#define time_perep0  0.001						//����� ������������ �������-��������0 (1��)
#define time_perep1  4							//����� ������������ �������-��������1 (4�)//			�� ����� 8�. ����� �����������

#define ust_tc0 256-((time_perep0*F_CPU)/1024)			//������������� �������-��������0,  ������������ ������ 10��
#define ust_tc1 65536 -((time_perep1*F_CPU)/1024)		//������������� �������-��������1,  ������������ ������ 4�


#define Time_min(x) (time_perep0*1000*60*x)				//			
#define Time_chas(x) (time_perep1*15*60*x)				


const uint32_t Ust_period_test= 21600000;//6�//15552000 ;	//��� � 72 ����//       Time_chas(72);	//������ ���������� �����(����)
const uint16_t Ust_time_test  = 30000;	//30 ���//                  Time_min(1);	//������������ ����� (���)
const uint16_t Ust_count2 = 10000;		//10 ���//					Time_min(1);		//������ ������ ���������� ��� ������ �� ������������(���)
const uint8_t T = 1;								//������ ������ ���������� ������� (���)
const uint32_t UST_time_zar = 60000;//60���//310000;				//��������� ������� �������� 10�. (����)


/*

U1------| 
		R1
		|
		|---U2
		|
		R2
--------|

U2=(U1*R2)/(R1+R2)
*/

//��������� ��������� ����������
//��� �������� ����������:R1=180k, R2=10k.
#define R1 180
#define R2 10

#define U_OP 1.1								//���������� �������� �������� ���������� 1.1�
#define DC_COD(U1)		(uint32_t)(1024* ((U1*R2)/(R2+R1)) /U_OP) //������ ��������� ���� ���������� ��� c ������ ��������

//#define UST_Uout		DC_COD(11)				//�������� ��������� ���������� 
//#define UST_Uin		DC_COD(15)				//������� ��������� ���������� 
//#define UST_Ubat_min	DC_COD(11)				//���������� ������� ��� ������� ���������� �����
//#define UST_Ubat_zar	DC_COD(14)				//���������� ������� ��� ������� ������� ��������
//#define Umin			DC_COD(8)				//���������� ��� ������� ���������� ���������� �������� 
//#define Pad_napr_test	(DC_COD(12)-DC_COD(10))//�������� ���������� ��� �����


#define UST_Ubat_min	603					//���������� ������� ��� ������� ���������� ����� 12,3
#define UST_Ubat_zar	698					//���������� ������� ��� ������� ������� �������� 14,25

#define UST_Uout		530					//11
#define UST_Uin			392					//8
#define Umin			342                 //7 //           DC_COD(6)			//����������� ����� ���������� ������������

#define Pad_napr_test	100					//�������� ��� ������� ����������(2�) ����� �����((0.11*1024)/1.1)

//�������� � ����������
//��������� ����
#define LED_PWR_ON()		SetBit(PORTB, led_pwr);	// ��� 
#define LED_PWR_OFF()		ClrBit(PORTB,led_pwr);	// ����
//��������� ������
#define LED_ERR_ON()		SetBit(PORTB,led_err);		// ���
#define LED_ERR_OFF()		ClrBit(PORTB,led_err);		// ����
//��������� ������
#define LED_ZAR_ON()		SetBit(PORTB,led_zar);		// ���
#define LED_ZAR_OFF()		ClrBit(PORTB,led_zar);		// ����
//��������� �������
#define LED_RAZR_ON()		SetBit(PORTB,led_razr);		// ��� 
#define LED_RAZR_OFF()		ClrBit(PORTB,led_razr);		// ����
//��������� ����������� ������
#define LED_AKK_ON()		SetBit(PORTB,led_avton);		// ���
#define LED_AKK_OFF()		ClrBit(PORTB,led_avton);		// ����

/////////////////////////////////////////////////////////////////////////

#define  ADC_REF_VCC		(0<<REFS0)					// ��� ������� -  ���������� �������
#define  ADC_REF_INTERNAL	(1<<REFS0)|(1<<REFS1)		// ��� ������� -  ���������� �������� 1.1�
#define  ADC_RIGHT_ADJ		(0<<ADLAR)					// �������� ������
#define  ADC_LEFT_ADJ		(1<<ADLAR)					// �������� �����


#define  ADC_VREF_TYPE ( ADC_REF_INTERNAL | ADC_RIGHT_ADJ) // ���������� �������� �������� ���������� � �������� ������ � ��������  ADLAR

#define  ADC_CH0			(0)		// ������������ ADC0 (PB5)
#define  ADC_CH1			(1)		// ������������ ADC1 (PB2)
#define  ADC_CH2			(2)		// ������������ ADC2 (PB4)
#define  ADC_CH3			(3)		// ������������ ADC3 (PB3)


/**********************************************************
 * ����������� ���������
 *********************************************************/
// �������� ����� ����������. ���������������� �������
#define SERIAL_NUMBER		1804041333 // ��.��.��.��.��

//������ �� 2.4
#define VERSION_X			2			// ������ �������� ��, ����� ����� ������
#define VERSION_Y			4			// ������ �������� ��, ����� ����� �����.
// �������������. 
#define TYPE_DEF			5			//��� ����������

#define TIME_BETWEEN_BYTE	2			// ����� ����� ����� �������.
#define SIZE_BUF 			20			// ������ ������ USART

// ������� ����� ���������� �� ��������� 2 ��������� ����� ��������� ������
#define NET_NUMBER			((unsigned char)(SERIAL_NUMBER%100))

/**********************************************************
 * ����������� ��� ��������� �������� ��������
 *********************************************************/
#define formul(spid) (( (F_CPU/16)/spid)-1) 
// ������������
#define BR_2400		formul(2400)		//�� ���
#define BR_4800		formul(4800)		//�� ���
#define BR_9600		formul(9600)
#define BR_19200	formul(19200)
#define BR_38400	formul(38400)
#define BR_57600	formul(57600)		//�� ���
#define BR_115200	formul(115200)		//�� ���

// ������������� � ������ ������
#define ABR_2400	24
#define ABR_4800	48
#define ABR_9600	96
#define ABR_19200	19
#define ABR_38400	38
#define ABR_57600	57
#define ABR_115200	115

#define RXE 		( (1<<RXEN0)|(1<<RXCIE0) ) // �������� ��������

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* ��������� � ����������*/

//�����
extern bool Flag_test;					//���� ���������� �����
extern bool flag_z;
extern bool flag_d;
extern uint8_t flagi;					//����� ��������� ��� ���������


extern int Tp;						//����� ������ �� ������������
extern uint32_t count_period_test;		//���������� ������� ������������
extern uint16_t count_time_test;		//���������� �������� ������������ �����
extern uint16_t count_time_test_T2;		//���������� �������� ������� ������ ���������� ��� � ���������� ������
extern int U1;						//��������� ����������
extern uint32_t count_time_zar;			//����� ��������



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*********************************************************
  * EEPROM. ���������� � ����������������� ������
  * ������ �� ���������
  ********************************************************/
extern uint8_t EEMEM ee_empty[];						//������ ����� ������ ���� ������
extern conf EEMEM ee_cfg;
extern conf cfg;


extern uint8_t usart_buf[SIZE_BUF];						// ������ �������� ����
extern uint8_t received_bytes ;							// ���������� �������� ���� � ������
extern uint8_t timer1ms;								// ������� ����� ����� �������
extern uint32_t test_buf[5];

//���������� �������
void transmit(unsigned char *data, uint8_t sz);
void USART_Init( unsigned int baud );
uint16_t crc16(char *in, uint8_t size);
uint8_t razb_pac (void);
void parsing_package(void);
unsigned int read_adc(unsigned char adc_input);
void init_device(void);
void regim_test(void);
void time_work(void);
void zaryd(void);

#if DEBUG
void massage(char mass[],unsigned char AD);
#endif


#endif // end _MDEF_H

