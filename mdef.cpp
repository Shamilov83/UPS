// #include <avr/io.h>
// #include <util/delay.h>
// #include <avr/eeprom.h>
#include "mdef.h"

#if DEBUG
char deb_log[M_SIZE];
 char *x,*y;
#endif

/************************************************************************/
/* глобальные переменные                                                */
/************************************************************************/
conf cfg;

uint32_t test_buf[5];						//массив для  теста
uint8_t usart_buf[SIZE_BUF];				// массив принятых байт
uint8_t received_bytes ;					// количество принятых байт в массив
uint8_t timer1ms;							// считает время между байтами


bool Flag_test = 0;								//флаг проведения теста
bool flag_z = 0;
bool flag_d = 0;

int Tp;									//расчитанное время работы от аккумулятора до его разрядки.
uint32_t count_period_test = 0;					//переменные подсчета переполнений
uint16_t count_time_test = 0;					//переменная счетчика длительности теста
uint16_t count_time_test_T2 = 0;				//переменная счетчика приода опроса напряжения АКБ в автономном режиме
int U1;									//начальное напряжение (последнее измеренное напряжение АКБ)
uint32_t count_time_zar;
/**
биты переменной flagi: 
0-
1 - 
2 - проведение заряда
3 - низкий заряд аккумулятора
4 - АВАРИЯ
5 - статус ключа нагрузки(1-вкл, 0-выкл)
6 - состояние аккумулятора (1-хор, 0-плохо) 
7 - режим работы (1-аккум, 0-сеть)
*/
uint8_t flagi;									//байт флагов состояния (для протокола)

/*********************************************************
  * EEPROM. Переменные в энергонезависимой памяти
  * Данные по умолчанию
  ********************************************************/
uint8_t EEMEM  ee_empty[] = "EEPROM";			//первый адрес должен быть пустым

//conf EEMEM ee_cfg = {
	//BR_9600,		 							//предделитель для скорости обмена
	//NET_NUMBER, 								//сетевой адрес устройства
	//0,											//Uz
	//0											//Um
//};
conf EEMEM ee_cfg = {
	BR_9600,		 							//предделитель для скорости обмена
	NET_NUMBER, 								//сетевой адрес устройства
	UST_Ubat_zar,								//Uz
	UST_Ubat_min								//Um
};

/**
 * Инициализация USART
 *@param unsigned int [IN] baud скорость
 */
void USART_Init( unsigned int baud )
{

	asm("cli");			//запретить прерывания
	 //USART initialization
	 //Параметры соединения: 8 Data, 1 Stop, No Parity
	 //USART Приемник: On
	 //USART Передатчик: On
	 //USART Режим: Asynchronous
	// USART Скорость: 9600
	UCSR0A=	(0<<RXC0) |		//устанавливается когда есть непрочитанные данные
			(0<<TXC0) |		//устанавливается когда есть непереданные данные
			(0<<UDRE0)|		//флаг готовности буфера
			(0<<FE0)	|	//ошибка кадрирования в приемном буфере
			(0<<DOR0) |		//переполнение приемного буфера
			(0<<UPE0) |		//ошибка бита четности
			(0<<U2X0) |		//делитель синхронной передачи
			(0<<MPCM0);
	UCSR0B=	(1<<RXCIE0) |	//Запись этого бита в один включает прерывание на флаге RXCn. Прерывание завершения получения USART будет сгенерировано, только если бит RXCIEn записан в единицу, флаг глобального прерывания в SREG записан в единицу и установлен бит RXCn в UCSRnA.
			(0<<TXCIE0) |	//Запись этого бита в один включает прерывание на флаге TXCn. Прерывание USART Transmit Complete будет сгенерировано, только если бит TXCIEn записан в единицу, глобальный флаг прерывания в SREG записан в единицу, и бит TXCn в UCSRnA установлен.
			(0<<UDRIE0) |	//Запись этого бита в один включает прерывание на флаге UDREn. Пустое прерывание регистра данных будет сгенерировано, только если бит UDRIEn записан в единицу, глобальный флаг прерывания в SREG записан в единицу, и бит UDREn в UCSRnA установлен.
			(1<<RXEN0) |	//Запись этого бита в один включает приемник USART.
			(0<<TXEN0) |	//Запись этого бита в один включает передатчик USART.
			(0<<UCSZ02) |	//Биты UCSZn2, объединенные с битом UCSZn1: 0 в UCSRnC, устанавливают количество битов данных (символ SiZe) в кадре, используемом приемником и передатчиком. установленно 8
			(0<<RXB80) | 
			(0<<TXB80);
	UCSR0C=	(0<<UMSEL01) |	//ассинхронный режим
			(0<<UMSEL00) |	//
			(0<<UPM01) |	//Эти биты включают и устанавливают тип генерации и проверки четности.
			(0<<UPM00) | 
			(0<<USBS0) |	//1 стоп-бит*/
			(1<<UCSZ01) |	
			(1<<UCSZ00) | 
			(0<<UCPOL0);
	 	//UBRR0H=0x00;//19200
	 	//UBRR0L=0x33;
	UBRR0H = (unsigned char)(baud>>8);
	UBRR0L = (unsigned char)baud;
	asm("sei");								//разрешить прерывания
	
}


/***********************************************************
 * Инициализация устройства
 **********************************************************/
void init_device(void)
{
	asm("cli");
	eeprom_read_block(&cfg,&ee_cfg,sizeof(conf));   //
	
	/*Бит CLKPCE (7) управляет изменением значения предделителя. 
	Для изменения значения предделителя сначала в него необходимо записать 1, 
	при этом стоит учесть, что в остальные биты данного регистра должно быть записано значение 0. 
	После этого в течении четырех машинных циклов необходимо изменить значение предделителя записью соответствующих битов. 
	По прошествии четырех машинных циклов бит CLKPCE сбрасывается в 0.*/
	//CLKPR=(1<<CLKPCE);
	CLKPR=(0<<CLKPCE) | (0<<CLKPS3) | (0<<CLKPS2) | (0<<CLKPS1) | (0<<CLKPS0);
	
	// ------------------------------------------
	// Настройка портов
	// ------------------------------------------
	// Input/Output Ports initialization 0|1
	// Настрока порта B 
	// -------------------
	// Режим
	DDRB=	(0<<DDB7) | // Вход 	
			(0<<DDB6) | // Вход
			(1<<DDB5) | // Выход	LED_RAZR 
			(1<<DDB4) | // Выход	LED_ZAR
			(1<<DDB3) | // Выход	LED_ERR
			(1<<DDB2) | // Выход	LED_PWR
			(0<<DDB1) | // Вход		KN
			(1<<DDB0);  // Выход	индикация автономного режима
	// Подтяжка		
	PORTB=	(0<<PORTB7) | 
			(0<<PORTB6) | 
			(0<<PORTB5) | 
			(0<<PORTB4) | 
			(0<<PORTB3) | 
			(0<<PORTB2) | 
			(1<<PORTB1) | 	
			(0<<PORTB0);	// Вкл
	// Нстройка порта C 
	// ------------------
	// Режим
	DDRC=	(0<<DDC6) | 	// Вход
			(0<<DDC5) | 	// Вход
			(0<<DDC4) | 	// Вход
			(0<<DDC3) | 	// Вход		U_IN
			(0<<DDC2) | 	// Вход		U_BAT
			(0<<DDC1) | 	// Вход		U_OUT
			(0<<DDC0);		// Вход
	// Подтяжка		
	PORTC=	(0<<PORTC6) | 	
			(0<<PORTC5) | 
			(0<<PORTC4) | 
			(0<<PORTC3) | 
			(0<<PORTC2) | 
			(0<<PORTC1) | 
			(0<<PORTC0);
	// Настройка порта D
	// -------------------
	// Режим
	DDRD=	(1<<DDD7) | 	// Выход	UVT9
			(1<<DDD6) | 	// Выход	RAZR
			(1<<DDD5) | 	// Выход	OUT_BAT
			(1<<DDD4) | 	// Выход	ZAR 		
			(1<<DDD3) | 	// Выход	PREOBR
			(1<<DDD2) | 	// Выход
			(0<<DDD1) | 	// Вход	tx
			(0<<DDD0);		// Вход		rx
	// Подтяжка		
	PORTD=	(0<<PORTD7) | 	// 
			(0<<PORTD6) | 	//
			(0<<PORTD5) | 	//
			(0<<PORTD4) | 	//
			(0<<PORTD3) | 	//
			(0<<PORTD2) | 	//
			(0<<PORTD1) | 	//
			(0<<PORTD0);	//

	// -------------------------------------------------------------------
	// Инициализация таймеров / Счетчиков (Timer/Counter) 0 
	// -------------------------------------------------------------------
	//Биты COM0A1 (7) и COM0A0 (6) влияют на то, какой сигнал появится на выводе OC0A (12 ножка)
	// при совпадении с A (совпадение значения счетного регистра TCNT0 со значением регистра сравнения OCR0A):
	//00 - вывод OC0A не функционирует
	TCCR0A=	(0<<COM0A1) | 
			(0<<COM0A0) | 
			(0<<COM0B1) |	// Режим ШИМ 00 : вывод OC0A не функционирует
			(0<<COM0B0) | 
			(0<<WGM01) |	
			(0<<WGM00);
	//	WGM00..WGM02 = 0 :нормальный режим работы	
	TCCR0B=	(0<<WGM02) | 
			(1<<CS02) | 
			(0<<CS01) | 
			(1<<CS00);			//деление на 1024
	//		
	//TCNT0=0x00;
	//
	OCR0A=0x00;					//регистр сравнения А
	//
	OCR0B=0x00;					//регистр сравнения В

	// -----------------------------------------------------------
	// Инициализация Таймеров / Счетчиков (Timer/Counter) 1 
	// -----------------------------------------------------------
	//
	TCCR1A=	(0<<COM1A1) |		//Режим ШИМ 00 : вывод не функционирует
			(0<<COM1A0) |
			(0<<COM1B1) |
			(0<<COM1B0) |
			(0<<WGM11) |
			(0<<WGM10);
//	WGM10..WGM12 = 0 :нормальный режим работы	
	TCCR1B=	(0<<ICNC1) | 
			(0<<ICES1) | 
			(0<<WGM13) | 
			(0<<WGM12) | 
			(1<<CS12) |			////деление на 1024
			(0<<CS11) | 
			(1<<CS10);
	//		
	TCNT1H=0x00;	//
	TCNT1L=0x00;	//
	ICR1H=0x00;		//
	ICR1L=0x00;		//
	OCR1AH=0x00;	//
	OCR1AL=0x00;	//
	OCR1BH=0x00;	//
	OCR1BL=0x00;	//

	// ------------------------------------------------------------
	// Инициализация Таймеров / Счетчиков (Timer/Counter 2)
	// ------------------------------------------------------------
	//
	ASSR=	(0<<EXCLK) | ////устанавливается в 0 для тактирования от кварца
			(1<<AS2);//синхронизация по кварцу
	//		
	TCCR2A=	(0<<COM2A1) | 
			(0<<COM2A0) | 
			(0<<COM2B1) | 
			(0<<COM2B0) | 
			(0<<WGM21) | 
			(0<<WGM20);
	//		
	TCCR2B=	(0<<WGM22) | 
			(1<<CS22) | 
			(0<<CS21) | 
			(1<<CS20);
	//		
	TCNT2=0x00;
	OCR2A=0x00;
	OCR2B=0x00;

	//у всех счетчиков вызов прерывания по переполнению
	// Timer/Counter 0 Interrupt(s) initialization
	TIMSK0=	(0<<OCIE0B) | 
			(0<<OCIE0A) | 
			(1<<TOIE0);		//разрешить прерывание по переполнению
			//(0<<TOIE0);
	// Timer/Counter 1 Interrupt(s) initialization
	TIMSK1=	(0<<ICIE1) | 
			(0<<OCIE1B) | 
			(0<<OCIE1A) | 
			(1<<TOIE1);		//разрешить прерывание по переполнению
			//(0<<TOIE1);
	// Timer/Counter 2 Interrupt(s) initialization
	TIMSK2=	(0<<OCIE2B) | 
			(0<<OCIE2A) | 
			//(1<<TOIE2);		//разрешить прерывание по переполнению
			(0<<TOIE2);
	// -------------------------------------------------------
	// Инициализация внешнего прерывания
	// -------------------------------------------------------
	// INT0: Off
	// INT1: Off
	// Прерывание при любом изменении  на pins PCINT0-7: Off
	// Прерывание при любом изменении  на  PCINT8-14: Off
	// Прерывание при любом изменении  на  PCINT16-23: Off
	EICRA=	(0<<ISC11) | 
			(0<<ISC10) | 
			(0<<ISC01) | 
			(0<<ISC00);
	EIMSK=	(0<<INT1) | 	// Выкл
			(0<<INT0);		// Выкл
	PCICR=	(0<<PCIE2) | 
			(0<<PCIE1) | 
			(0<<PCIE0);

	// -----------------------------------------------
	// Инициализация аналогового компаратора
	// Аналогвый компаратор: Off
	// Положительный вход аналогового компаратора подключен к выводу AIN0
	// Отрицательный вход аналогового компаратора подключен к выводу AIN1
	ACSR=	(1<<ACD) | //компаратор выключен
			(0<<ACBG) | 
			(0<<ACO) | 
			(0<<ACI) | 
			(0<<ACIE) | 
			(0<<ACIC) | 
			(0<<ACIS1) | 
			(0<<ACIS0);
	// Цифровой входной буфер на AIN0: выключен(1)
	// Цифровой входной буфер на AIN1: выключен(1)
	DIDR1=	(0<<AIN0D) | 
			(0<<AIN1D);

	// Инициализация АЦП
	// АЦП Тактовая частота: 125,000 kHz
	// АЦП опорного напряжения: AREF 
	// Источник автоматического запуска АЦП: ADC Stopped
	
	//Биты ADC5D - ADC0D (5 - 0) регистра DIDR0 запрещают использование входов ADC5 - ADC0 (28 - 23 ножки) если в них установить 1.
	DIDR0=	(1<<ADC5D) | 
			(1<<ADC4D) | 
			(0<<ADC3D) | 
			(0<<ADC2D) | 
			(0<<ADC1D) | 
			(0<<ADC0D);
	//		
	ADMUX=ADC_VREF_TYPE;
	//
	ADCSRA=	(1<<ADEN) | //включает или выключает АЦП (1-включен).
			(1<<ADSC) | //запускает преобразование если в него записать 1 (для многоразового режима запуск первого преобразования).
			(0<<ADATE) | //позволяет запускать преобразование по прерыванию от переферийных устройств микроконтроллера если установить в 1.
			(0<<ADIF) | //флаг прерывания от АЦП
			(0<<ADIE) | //разрешает прерывания от АЦП если установлен в 1
			(1<<ADPS2) | //Биты ADPS2 - ADPS0 (2 - 0) регистра ADCSRA выбирают режим работы предделителя тактовой частоты:110 - CLK/128
			(1<<ADPS1) | 
			(1<<ADPS0);
	//	Биты ADTS2 - ADTS0 (2 - 0) регистра ADCSRB выбирают источник сигнала по которому будет начинаться преобразование АЦП:000 - непрерывное преобразование	
	ADCSRB=	(0<<ADTS2) | 
			(0<<ADTS1) | 
			(0<<ADTS0);

	// Инициализация SPI 
	// SPI отключен
	SPCR=	(0<<SPIE) | 
			(0<<SPE) | 
			(0<<DORD) | 
			(0<<MSTR) | 
			(0<<CPOL) | 
			(0<<CPHA) | 
			(0<<SPR1) | 
			(0<<SPR0);

	// Инициализация TWI 
	// TWI отключен
	TWCR=	(0<<TWEA) | 
			(0<<TWSTA) | 
			(0<<TWSTO) | 
			(0<<TWEN) | 
			(0<<TWIE);

	// Инициализация сторожевого таймера (Watchdog Timer)
	// Предделитель Watchdog Timer (Prescaler): OSC/32k
	// Действие Watchdog timeout: Reset (Перезагрузка)
	
	asm("wdr");	//
	//WDTCSR|=(1<<WDCE) | (1<<WDE);
	//
	WDTCSR|=(0<<WDIF) | //Этот бит устанавливается, когда в сторожевом таймере возникает тайм-аут и сторожевой таймер настроен на прерывание. 
			(0<<WDIE) | //			 
			(0<<WDCE) | // сторожевой таймер выключен
			(0<<WDE) |	//Сброс сторожевого таймера включен
			// Watchdog Timer 2.0 s
			(1<<WDP3) | 
			(0<<WDP2) | 
			(0<<WDP1) | 
			(1<<WDP0);
			
	MCUSR = (1<<WDRF);
/*	*/
	
	//настройка USART
	USART_Init(cfg.brate);
	//USART_Init(BR_9600);
	asm("sei"); //
}

/**
* Передача данных
* @param char* data ссылка данные
* @param unsigned char sz размер данных
*/


void transmit(unsigned char *data, uint8_t sz)
{

	unsigned char i;
	SetBit(PORTD, DRAWE);					// Переключить Max485 на передачу
	SetBit(UCSR0B, TXEN0);					// Разрешить передачу с МК
	for(i=0; i<sz; ++i)
	{	
								// Поместить данные в UDR, немедленная передача
	
		while ( !( UCSR0A & (1<<UDRE0) ) );	// Ждать очистки буфера(установка флага) для новой передачи.устанавливвается 1 после очистки буфера
		UDR0 = data[i];
	}
	UDR0='\n';
	timer1ms = 0;							// включаем задержку 1 мс//   
	//while(!timer1ms);						// ждем	//  ПРОГРАММА ВИСНЕТ НА ЭТОМ МЕСТЕ!!!!!!!!!
	ClrBit(UCSR0B, TXEN0);					// Запретить передачу с МК
	ClrBit(PORTD,DRAWE);				// Переключить на приём, т.е. вывести "0"
}

/**********************************************************
 * Расчет crc16 кода
 *********************************************************/
uint16_t crc16(char *in, uint8_t size)
{
	unsigned int crc = 0xffff;
	unsigned char sz;
	unsigned char s;
	
	for(sz=0; sz<size; sz++) 
	{
		crc ^=	 in[sz];
		for(s=0; s<8 ;++s) 
		{
			if(crc & 0x0001) 
			{
				crc = (crc>>1) ^ 0xA001;
				continue;
			}
			crc >>=1;
		}
	}
	return(crc);
}

/**
* обработка входных данных
*/
uint8_t razb_pac (void)
{
	uint8_t pr= 0;	// признак изменения конфигурации
	uint8_t len=0;	// размер принятого пакета
	// принятые данные хранятся в буфере usart_buf
	// считываем сетевой номер прибора (в 0 байте)
	// по этому же адресу и будем отправлять ответ
	usart_buf[0] = cfg.adr;	
	// тип устройства фиксирован в протоколе
	usart_buf[3] = TYPE_DEF;	
	// проверить номер функции в 1 байте
	// номер функции в ответе тот же
	switch(usart_buf[1]) 
	{
		case 0x03:	//чтение конфигурации устройства
			// формируем ответ:
			usart_buf[2] = 0x12;		// длина пакета ответа (18 байт) фиксирована в протоколе			
			usart_buf[4] = cfg.adr;		//сетевой адрес из конфигурации 
			// скорость из конфигурации(5-й байт)
			switch(cfg.brate)
			{
				case BR_2400: usart_buf[5] = ABR_2400; break;		//скорость 2400
				case BR_4800: usart_buf[5] = ABR_4800; break;		//скорость 4800
				case BR_9600: usart_buf[5] = ABR_9600; break;		//скорость 9600
				case BR_19200: usart_buf[5] = ABR_19200; break;		//скорость 19200 
				case BR_38400: usart_buf[5] = ABR_38400; break;		//скорость 38400
				case BR_57600: usart_buf[5] = ABR_57600; break;		//скорость 57600
				case BR_115200: usart_buf[5] = ABR_115200; break;	//скорость 115200
				 
			}
			*((unsigned short int*)(usart_buf+6)) = cfg.Uz;									// напряжение окончания заряда 
			*((unsigned short int*)(usart_buf+8)) = cfg.Um;									// напряжение начала заряда			
			*((unsigned long*)(usart_buf+10)) = SERIAL_NUMBER;		//серийный номер (8,9,10,11 байт)
			usart_buf[14] = VERSION_Y;								//версия ПО мажор (12 байт)
			usart_buf[15] = VERSION_X;								//версия ПО минор(13 байт)
			// пакет ответа сформирован
			break;
		case 0x10: 	//изменение конфигурации устройства
				//запись предделителя (4 байт в пакете)
				switch(usart_buf[4])
				{
					case ABR_2400:cfg.brate = BR_2400;
					break;
					case ABR_4800:cfg.brate = BR_4800;
					break;
					case ABR_9600:cfg.brate = BR_9600;
					break;
					case ABR_19200:cfg.brate = BR_19200;
					break;
					case ABR_38400:cfg.brate = BR_38400;
					break;
					case ABR_57600:cfg.brate = BR_57600;
					break;
					case ABR_115200:cfg.brate = BR_115200;
					break;						
				}
				// записываем сетевой адрес (5-й байт в пакете)
				cfg.adr = usart_buf[5];					
				// напряжение окончания заряда (6-й байт в пакете)	
				cfg.Uz = *((unsigned short int*)(usart_buf+6));					
				// напряжение начала заряда (7-й байт в пакете)
				cfg.Um = *((unsigned short int*)(usart_buf+8));							
				// сохранить новую конфигурацию
				eeprom_write_block(&cfg,&ee_cfg, sizeof(conf));		
				// устанавливаем признак изменения конфигурации	
				pr = 1;													
				// формируем ответ				
				usart_buf[2] = 0x06;		//длина пакета (2-й байт) 
				break;
		case 0x04: //чтение состояния
				// формируем ответ
				
				usart_buf[2] = 0x11;		//длина пакета из протокола
				usart_buf[4] = flagi;		// байт флагов состояния	
				*((unsigned long*)(usart_buf+5)) = Tp;	// оставшееся время работы	байты 5,6,7,8	
				*((unsigned short int*)(usart_buf+9)) = read_adc(Uin);
				*((unsigned short int*)(usart_buf+11)) = read_adc(Uout);
				*((unsigned short int*)(usart_buf+13)) = read_adc(Ubat);
					
				break;
		default: 	//неизвестная функция						
			usart_buf[1] |= 0x80; // 1 в старшем байте кода команды(признак ошибки)
			usart_buf[2] = 0x06;     // длина пакета (2 байт)
		}
		//присвоить len значение длины буфера без crc			
		len  = usart_buf[2]-2;
		// поместить по аресу  crc (uint16_t 2 последних байта)
		*( (uint16_t*)(usart_buf+len) ) = crc16 ((char*)usart_buf, len);	
		//отправить сформированный пакет данных
		//transmit(usart_buf, usart_buf[2]);
		transmit((unsigned char*)usart_buf, usart_buf[2]);							
		// возвращаем признак изменения конфигурации
		return pr;
} 

/**
* Проверка входных даннных и формирвание пакета
*/ 
void parsing_package(void)			//анализ посылки
{	
	uint8_t x = 0;	// признак изменения конфигурации
	uint8_t len=0;	// длина пакета	
		
	// 1. Проверка кол-ва принятых байт (2 байт пакета)
	// --------------------------------------------
	// если кол-во принятых байт меньше размера указанного в пакете (байт 2)	
	if(received_bytes < usart_buf[2])
	{
		// выход из функции
		return;
	}
	// Запоминаем размер пакета
	len = usart_buf[2];			
	// Запрещаем прерывание по приёму
	UCSR0B &= ~RXE;					
	// 2. Проверка сетевого адреса
	// --------------------------
	// Адрес считываем из 0 байта. 
	//  Если адрес совпадает или 0 (широковещательный)
	if( (cfg.adr == usart_buf[0]) || (0 == usart_buf[0]) )		
	{
		// 3. проверка crc кода
		// ---------------------
		uint16_t crc2 = crc16((char*)usart_buf, len-2);			// рассчитать CRC код
		uint16_t crc3 = *((uint16_t*)(usart_buf+len-2));		// получить CRC код из пакета
		// если расчитанное CRC совпадает с пришедшим
		if( crc2 == crc3 )										
		{
			// 4. Проверяем тип устройства
			//если тип устройства совпадает
			if(usart_buf[3]== TYPE_DEF)				
			{
				//transmit((char*)usart_buf, usart_buf[2]);
				// обрабатываем пакет
				x = razb_pac();								
			}
		}
	}

	//Переход на приём, сброс всех флагов
	received_bytes = 0;	//Сбросить количество принятых байт
	UCSR0B |= RXE;		//Разрешаем прерывание по приёму
	// Если конфигурация изменилась
	if (x)
	{
		// Применить изменения
		USART_Init(cfg.brate);	
	}
}
/**
*
*@param unsigned char adc_input 
*/
//функция чтения с аналогового входа
//возвращает цифровой код 0..1024
unsigned int read_adc(unsigned char adc_input)
{
	ADMUX = ADC_VREF_TYPE | adc_input;
	//Задержка необходима для стабилизации входного напряжения АЦП
	_delay_us(10);
	//начало преобразования
	ADCSRA|=(1<<ADSC);
	// Дождитесь завершения конвертации AD
	while ((ADCSRA & (1<<ADIF))==0);
	ADCSRA|=(1<<ADIF);
	return (uint16_t)ADC;
}

/**
* режим тестирования
*/
 void regim_test(void)							//
 {
 	 uint16_t U2 = 0;							// конечное напряжение

	 if((flagi&(1<<4))==0)							//если нет аварии, то провести тест
	 {

		 if(count_period_test>Ust_period_test)		//если наступило время теста
		 {					 
			 if(!Flag_test)							//если тест не начат
			 {
				 count_time_test = 0;				//обнулить счетчик длительности теста
				 Flag_test = 1;						//установить флаг начала теста
				 U1=read_adc(Ubat);					//записать начальное напряжение аккумулятора
				 // запускать можно только если не заряжается, а не прерывать заряд
				 ZAR_OFF();							//установить соответствующие ключи
				 RAZR_ON();							// включаем разрядку
				 OUT_BAT_OFF();						//выключить питание от батареи
				 LED_RAZR_ON();						//индикация идет ТЕСТ
				 
				 #if DEBUG
				 x="nachalo_testa\r\n";
				 transmit((uint8_t*)x,strlen(x));
				 x="\r\n";
				 transmit((uint8_t*)x,strlen(x));
				 
				  //sprintf(x,"nach_testa U=%.2f ",read_adc(Ubat));
				  //transmit((uint8_t*)x,strlen(x));
				 #endif
			 }
			 else //если тест запущен
			 {
				 if(count_time_test>Ust_time_test)	//проверить прошедшее время
				 {
					 #if DEBUG
					 x="M5 \r\n";
					 transmit((uint8_t*)x,strlen(x));
					 x="\r\n";
					 transmit((uint8_t*)x,strlen(x));
					 #endif
					 
					 U2=read_adc(Ubat);				//если время теста вышло считать значение напряжения АКБ
					 count_period_test = 0;			//обнулить счетчик периода проведения теста
					 Flag_test = 0;					//обнулить флаг теста
					 RAZR_OFF();					//отключить разряд
					 OUT_BAT_OFF();					//установить ключи на заряд и работу от сети
					 ZAR_ON();
					 LED_RAZR_OFF();				//отключить индикацию теста
					 
					 if((U1-U2)> Pad_napr_test)		//если падение напряжения на аккумуляторе при тесте больше установленного,
					 {//то акб плохой
						 //status_akkum = 0;			//УСТАНОВИТЬ ФЛАГ СТАТУСА АККУМУЛЯТОРА
						 ClrBit(flagi,6);			// установить статус "плохой"(сбросить бит 6)
						 #if DEBUG
						x= "_________AKB_bad___________\r\n";
						transmit((uint8_t*)x,strlen(x));
						#endif
						 
					 }
					 else
					 {
						 //status_akkum = 1;			//
						 SetBit(flagi,6);			//иначе установить бит 6
						 #if DEBUG
						  x="___________AKB_good__________\r\n" ;
						  transmit((uint8_t*)x,strlen(x));
						#endif
						
					 }
					 
					  #if DEBUG
					  x="okonchanie_testa\r\n ";
					  transmit((uint8_t*)x,strlen(x));
					  x="\r\n";
					  transmit((uint8_t*)x,strlen(x));
					  
						x="U1= ";
						transmit((uint8_t*)x,strlen(x));
					
						ltoa((long)U1,x,10);						
						transmit((uint8_t*)x,strlen(x));
						x="\r\n";
						transmit((uint8_t*)x,strlen(x));
						
						x="U2= ";
						transmit((uint8_t*)x,strlen(x));
				
					   ltoa((long)U2,x,10);				
					   transmit((uint8_t*)x,strlen(x));
					   x="\r\n";
					   transmit((uint8_t*)x,strlen(x));
					   
					  #endif
					  
				 }
				 
			 }
		 } 
	}
 }

 //функция расчета времени работы от аккумулятора
 //для исключения деления на ноль при U1=U2, Tp = 0xFFFFFFFF;
 void time_work(void)
 {	 
	int U2 = 0;			//конечное напряжение
	 
	 if(U1!=U2)
	 {		 
	 U2 = read_adc(Ubat);
	 Tp = ((T)*(Umin-U1))/(U2-U1);		//расчет остаточного времени работы до напряжения отключения Umin (.мин)
	 
	 
	 
	 #if DEBUG
	 x="____U1= ";
	 transmit((uint8_t*)x,strlen(x));
	 
	 ltoa((long)U1,x,10);
	 transmit((uint8_t*)x,strlen(x));
	 x="___\r\n";
	 transmit((uint8_t*)x,strlen(x));
	 
	 x="___U2= ";
	 transmit((uint8_t*)x,strlen(x));
	 
	 ltoa((long)U2,x,10);
	 transmit((uint8_t*)x,strlen(x));
	 x="____\r\n";
	 transmit((uint8_t*)x,strlen(x));
	 #endif
	 
	 
	 U1 = U2;
	 }
	 else
	 {
		 Tp = 0xFFFFFFFF;
	 }
	 
	 
	 #if DEBUG 	 
 	 x="____Time= ";
 	 transmit((uint8_t*)x,strlen(x));
	  
	ltoa((long)Tp ,x,10);
	transmit((uint8_t*)x,strlen(x));
	
	x="____\r\n";
	transmit((uint8_t*)x,strlen(x));
	 #endif
 }
 
void zaryd (void)
{
		
		
		if(flagi&(1<<2))						
		{
				
					if(read_adc(Ubat)>cfg.Uz)		//если напряжение достигло уровня заряженного аккумулятора					
					{
  					
									/*			 		 
									if(flag_d)				//если установлен флан дозаряда
									{
									#if DEBUG
									x=" dozar ";
									transmit((uint8_t*)x,strlen(x));
									ltoa((long)count_time_zar,x,10);
									transmit((uint8_t*)x,strlen(x));
									#endif
																												
												if(count_time_zar>UST_time_zar)
												{
												flag_z = 0;				//снять флаг заряда
												flag_d = 0;				//снять флаг дозаряда
												ZAR_OFF();						//отключить заряд
												LED_ZAR_OFF();					//выключить инндикацию заряда
	
														#if DEBUG																																																																									
														x = "                 dozar_zak Ubat=";
														transmit((uint8_t*)x,strlen(x));
														x="\r\n";	
														transmit((uint8_t*)x,strlen(x));
																				
														ltoa((long)read_adc(Ubat),x,10);
														transmit((uint8_t*)x,strlen(x));
														x="\r\n";
														transmit((uint8_t*)x,strlen(x));													  
														#endif
												}
												
									}
									else
									{
									flag_d = 1;				//если флаг дозаряда не установлен, установить
									ZAR_ON();
									LED_ZAR_ON();											
									count_time_zar = 0;						//************************		
										
									#if DEBUG															
 									x="                fl_dozar \r\n";									
									transmit((uint8_t*)x,strlen(x));
									x="\r\n";
									transmit((uint8_t*)x,strlen(x));
									#endif	
									}
										
							*/
							ZAR_OFF();						//отключить заряд
							LED_ZAR_OFF();					//выключить инндикацию заряда		
							//flag_z = 0;						//обнулить флаг заряда
							ClrBit(flagi,2);
							
							#if DEBUG
							x="zaryd zakonchen flag_z = ";		
							transmit((uint8_t*)x,strlen(x));
							ltoa(flag_z,x,10);
							transmit((uint8_t*)x,strlen(x));
							x="______________\r\n";
							transmit((uint8_t*)x,strlen(x));
							#endif
											
					}

		}
		else
		{	
																
						if(read_adc(Ubat)<cfg.Um)		//если напряжение аккумулятора ниже установки, то зарядить аккумулятор
						
						{
							SetBit(flagi,2);				//установить флаг заряда
							ZAR_ON();						//зарядить аккумулятор
							LED_ZAR_ON();					//включить индикацию заряда
							
							#if DEBUG																																									
							//transmit((uint8_t*)x,strlen(x)); 																										
 							x="______________Ubat_min = ";
 							transmit((uint8_t*)x,strlen(x));													
 							ltoa((long)read_adc(Ubat),(x),10);
 							transmit((uint8_t*)x,strlen(x));
 							x="______________\r\n";
 							transmit((uint8_t*)x,strlen(x));
 							x="______________ Nachalo_zar_flag_z = ";
 							transmit((uint8_t*)x,strlen(x));
 							ltoa(flag_z,x,10);
 							transmit((uint8_t*)x,strlen(x));
 							x="______________\r\n";
 							transmit((uint8_t*)x,strlen(x));
							//massage("__nachalo zaryda Ubat =",Ubat);
							#endif
																							
							//flag_z = 1;				//установить флаг заряда
							
						}
						else
						{													
							regim_test();					//функция теста.
							ZAR_OFF();										
						}																		
		}	
}
 
#if DEBUG
void massage(char mass[],unsigned char AD)
{
	ltoa((long)read_adc(AD),y,10);
	strcat(mass,y);
	strcat(x,"\0\r\n");
	transmit((uint8_t*)x,strlen(x));
	
}

#endif