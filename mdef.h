
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

//#define TYPE_DEV 5 // тип устройства (для протокола)  определен ниже

#define DEBUG 0			// признак отладки
#if DEBUG
#define M_SIZE 100		// размер массива байт для вывода отладочной информации
extern char deb_log[];
extern char *x,*y;
#endif




#define SetBit(x,y) 	(x|=(1<<y))
#define ClrBit(x,y) 	(x&=~(1<<y))
#define ToggleBit(x,y)	(x^=(1<<y))
#define FlipBit(x,y)	(x^=(1<<y))
#define TestBit(x,y)	(x&(1<<y))

#define Uout			PORTC1				//напряжение на выходе
#define Uin				PORTC3				//напряжение на входе
#define Ubat			PORTC2				//напряжение батареи

#define ZAR				PORTD4				//ключ цепипи заряда
#define preobr			PORTD3				//ключ входного напряжения и преобразователя что за преобразователь
#define OUT_BAT			PORTD5				//ключ с батарии и преобразователя
#define RAZR			PORTD6				//ключ разряда
#define Uvt9			PORTD7				//ключ нагрузки



// управление ключами
#define ZAR_ON()		SetBit(PORTD, ZAR);	// включить зарядку
#define ZAR_OFF()		ClrBit(PORTD, ZAR);	// отключить зарядку
#define RAZR_ON()		SetBit(PORTD,RAZR);	// включить разрядку
#define RAZR_OFF()		ClrBit(PORTD,RAZR);	// отключить разрядку
#define OUT_BAT_ON()	SetBit(PORTD,OUT_BAT);	//подключить батарею к преобразователю
#define OUT_BAT_OFF()	ClrBit(PORTD,OUT_BAT);	//отключить батарею к преобразователю
#define PREOBR_ON()		SetBit(PORTD,preobr);	//подключить преобразователь к входному источнику
#define PREOBR_OFF()	ClrBit(PORTD,preobr);	//отключить преобразователь от входного источника
#define NAGR_ON()		SetBit(PORTD,Uvt9);		//включить ключ нагрузки
#define NAGR_OFF()		ClrBit(PORTD,Uvt9);		//отключить ключ нагрузки

#define DRAWE			PORTD2					// управление драйвером 485

#define KN				PORTB1					// кнопка включения
#define led_pwr			PORTB2					// индикатор режима работы от сети
#define led_err			PORTB3					// индикатор ошибки
#define led_zar			PORTB4					// индикатор режима заряда
#define led_razr		PORTB5					// индикатор режима разряда
#define led_avton		PORTB0					// индикация автономного режима

#define time_perep0  0.001						//время переполнения таймера-счетчика0 (1мс)
#define time_perep1  4							//время переполнения таймера-счетчика1 (4с)//			ПО ФАКТУ 8с. НУЖНО ПЕРЕСЧИТАТЬ

#define ust_tc0 256-((time_perep0*F_CPU)/1024)			//предустановка таймера-счетчика0,  переполнение каждые 10мс
#define ust_tc1 65536 -((time_perep1*F_CPU)/1024)		//предустановка таймера-счетчика1,  переполнение каждые 4с


#define Time_min(x) (time_perep0*1000*60*x)				//			
#define Time_chas(x) (time_perep1*15*60*x)				


const uint32_t Ust_period_test= 21600000;//6ч//15552000 ;	//раз в 72 часа//       Time_chas(72);	//период проведения теста(часы)
const uint16_t Ust_time_test  = 30000;	//30 сек//                  Time_min(1);	//длительность теста (мин)
const uint16_t Ust_count2 = 10000;		//10 сек//					Time_min(1);		//период замера напряжения при работе от аккумулятора(мин)
const uint8_t T = 1;								//период замера напряжения батареи (мин)
const uint32_t UST_time_zar = 60000;//60сек//310000;				//установка времени дозаряда 10ч. (мсек)


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

//установка пороговых напряжений
//при делителе напряжения:R1=180k, R2=10k.
#define R1 180
#define R2 10

#define U_OP 1.1								//Внутренний источник опорного напряжения 1.1в
#define DC_COD(U1)		(uint32_t)(1024* ((U1*R2)/(R2+R1)) /U_OP) //расчет цифрового кода напряжения АЦП c учетом делителя

//#define UST_Uout		DC_COD(11)				//выходное пороговое напряжение 
//#define UST_Uin		DC_COD(15)				//входное пороговое напряжение 
//#define UST_Ubat_min	DC_COD(11)				//напряжение батареи при котором начинается заряд
//#define UST_Ubat_zar	DC_COD(14)				//напряжение батарей при котором батарея заряжена
//#define Umin			DC_COD(8)				//напряжение при котором происходит отключение нагрузки 
//#define Pad_napr_test	(DC_COD(12)-DC_COD(10))//разность напряжений при тесте


#define UST_Ubat_min	603					//напряжение батареи при котором начинается заряд 12,3
#define UST_Ubat_zar	698					//напряжение батарей при котором батарея заряжена 14,25

#define UST_Uout		530					//11
#define UST_Uin			392					//8
#define Umin			342                 //7 //           DC_COD(6)			//минимальный порог напряжения аккумулятора

#define Pad_napr_test	100					//цифровой код падения напряжения(2В) после теста((0.11*1024)/1.1)

//действия с лампочками
//индикация сети
#define LED_PWR_ON()		SetBit(PORTB, led_pwr);	// Вкл 
#define LED_PWR_OFF()		ClrBit(PORTB,led_pwr);	// Выкл
//индикация ошибки
#define LED_ERR_ON()		SetBit(PORTB,led_err);		// Вкл
#define LED_ERR_OFF()		ClrBit(PORTB,led_err);		// Выкл
//индикация заряда
#define LED_ZAR_ON()		SetBit(PORTB,led_zar);		// Вкл
#define LED_ZAR_OFF()		ClrBit(PORTB,led_zar);		// Выкл
//индикация разряда
#define LED_RAZR_ON()		SetBit(PORTB,led_razr);		// Вкл 
#define LED_RAZR_OFF()		ClrBit(PORTB,led_razr);		// Выкл
//индикация автономного режима
#define LED_AKK_ON()		SetBit(PORTB,led_avton);		// Вкл
#define LED_AKK_OFF()		ClrBit(PORTB,led_avton);		// Выкл

/////////////////////////////////////////////////////////////////////////

#define  ADC_REF_VCC		(0<<REFS0)					// АЦП опорное -  напряжение питание
#define  ADC_REF_INTERNAL	(1<<REFS0)|(1<<REFS1)		// АЦП опорное -  внутренний источник 1.1в
#define  ADC_RIGHT_ADJ		(0<<ADLAR)					// смещение вправо
#define  ADC_LEFT_ADJ		(1<<ADLAR)					// смещение влево


#define  ADC_VREF_TYPE ( ADC_REF_INTERNAL | ADC_RIGHT_ADJ) // внутренний источник опорного напряжения и смещение вправо в регистре  ADLAR

#define  ADC_CH0			(0)		// использовать ADC0 (PB5)
#define  ADC_CH1			(1)		// использовать ADC1 (PB2)
#define  ADC_CH2			(2)		// использовать ADC2 (PB4)
#define  ADC_CH3			(3)		// использовать ADC3 (PB3)


/**********************************************************
 * Определения консатант
 *********************************************************/
// Серийный номер устройства. Инкрементировать вручную
#define SERIAL_NUMBER		1804041333 // гг.мм.дд.чч.мм

//версия ПО 2.4
#define VERSION_X			2			// Версия текущего ПО, число перед точкой
#define VERSION_Y			4			// Версия текущего ПО, число после точки.
// Идентификатор. 
#define TYPE_DEF			5			//тип устройства

#define TIME_BETWEEN_BYTE	2			// Время между двумя байтами.
#define SIZE_BUF 			20			// Размер буфера USART

// Сетевой номер устройства по умолчанию 2 последние цифры серийного номера
#define NET_NUMBER			((unsigned char)(SERIAL_NUMBER%100))

/**********************************************************
 * Определения для настройки скорости передачи
 *********************************************************/
#define formul(spid) (( (F_CPU/16)/spid)-1) 
// предделитель
#define BR_2400		formul(2400)		//НЕ РАБ
#define BR_4800		formul(4800)		//НЕ РАБ
#define BR_9600		formul(9600)
#define BR_19200	formul(19200)
#define BR_38400	formul(38400)
#define BR_57600	formul(57600)		//НЕ РАБ
#define BR_115200	formul(115200)		//НЕ РАБ

// представление в пакете данных
#define ABR_2400	24
#define ABR_4800	48
#define ABR_9600	96
#define ABR_19200	19
#define ABR_38400	38
#define ABR_57600	57
#define ABR_115200	115

#define RXE 		( (1<<RXEN0)|(1<<RXCIE0) ) // включить приемник

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* константы и переменные*/

//флаги
extern bool Flag_test;					//флаг проведения теста
extern bool flag_z;
extern bool flag_d;
extern uint8_t flagi;					//флаги состояния для протокола


extern int Tp;						//время работы от аккумулятора
extern uint32_t count_period_test;		//переменные посчета переполнений
extern uint16_t count_time_test;		//переменная счетчика длительности теста
extern uint16_t count_time_test_T2;		//переменная счетчика периода опроса напряжения АКБ в автономном режиме
extern int U1;						//начальное напряжение
extern uint32_t count_time_zar;			//время дозаряда



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*********************************************************
  * EEPROM. Переменные в энергонезависимой памяти
  * Данные по умолчанию
  ********************************************************/
extern uint8_t EEMEM ee_empty[];						//первый адрес должен быть пустым
extern conf EEMEM ee_cfg;
extern conf cfg;


extern uint8_t usart_buf[SIZE_BUF];						// массив принятых байт
extern uint8_t received_bytes ;							// количество принятых байт в массив
extern uint8_t timer1ms;								// считает время между байтами
extern uint32_t test_buf[5];

//Объявление функций
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

