// #include <avr/io.h>
// #include <util/delay.h>
// #include <avr/eeprom.h>
#include "mdef.h"

#if DEBUG
char deb_log[M_SIZE];
 char *x,*y;
#endif

/************************************************************************/
/* ���������� ����������                                                */
/************************************************************************/
conf cfg;

uint32_t test_buf[5];						//������ ���  �����
uint8_t usart_buf[SIZE_BUF];				// ������ �������� ����
uint8_t received_bytes ;					// ���������� �������� ���� � ������
uint8_t timer1ms;							// ������� ����� ����� �������


bool Flag_test = 0;								//���� ���������� �����
bool flag_z = 0;
bool flag_d = 0;

int Tp;									//����������� ����� ������ �� ������������ �� ��� ��������.
uint32_t count_period_test = 0;					//���������� �������� ������������
uint16_t count_time_test = 0;					//���������� �������� ������������ �����
uint16_t count_time_test_T2 = 0;				//���������� �������� ������ ������ ���������� ��� � ���������� ������
int U1;									//��������� ���������� (��������� ���������� ���������� ���)
uint32_t count_time_zar;
/**
���� ���������� flagi: 
0-
1 - 
2 - ���������� ������
3 - ������ ����� ������������
4 - ������
5 - ������ ����� ��������(1-���, 0-����)
6 - ��������� ������������ (1-���, 0-�����) 
7 - ����� ������ (1-�����, 0-����)
*/
uint8_t flagi;									//���� ������ ��������� (��� ���������)

/*********************************************************
  * EEPROM. ���������� � ����������������� ������
  * ������ �� ���������
  ********************************************************/
uint8_t EEMEM  ee_empty[] = "EEPROM";			//������ ����� ������ ���� ������

//conf EEMEM ee_cfg = {
	//BR_9600,		 							//������������ ��� �������� ������
	//NET_NUMBER, 								//������� ����� ����������
	//0,											//Uz
	//0											//Um
//};
conf EEMEM ee_cfg = {
	BR_9600,		 							//������������ ��� �������� ������
	NET_NUMBER, 								//������� ����� ����������
	UST_Ubat_zar,								//Uz
	UST_Ubat_min								//Um
};

/**
 * ������������� USART
 *@param unsigned int [IN] baud ��������
 */
void USART_Init( unsigned int baud )
{

	asm("cli");			//��������� ����������
	 //USART initialization
	 //��������� ����������: 8 Data, 1 Stop, No Parity
	 //USART ��������: On
	 //USART ����������: On
	 //USART �����: Asynchronous
	// USART ��������: 9600
	UCSR0A=	(0<<RXC0) |		//��������������� ����� ���� ������������� ������
			(0<<TXC0) |		//��������������� ����� ���� ������������ ������
			(0<<UDRE0)|		//���� ���������� ������
			(0<<FE0)	|	//������ ������������ � �������� ������
			(0<<DOR0) |		//������������ ��������� ������
			(0<<UPE0) |		//������ ���� ��������
			(0<<U2X0) |		//�������� ���������� ��������
			(0<<MPCM0);
	UCSR0B=	(1<<RXCIE0) |	//������ ����� ���� � ���� �������� ���������� �� ����� RXCn. ���������� ���������� ��������� USART ����� �������������, ������ ���� ��� RXCIEn ������� � �������, ���� ����������� ���������� � SREG ������� � ������� � ���������� ��� RXCn � UCSRnA.
			(0<<TXCIE0) |	//������ ����� ���� � ���� �������� ���������� �� ����� TXCn. ���������� USART Transmit Complete ����� �������������, ������ ���� ��� TXCIEn ������� � �������, ���������� ���� ���������� � SREG ������� � �������, � ��� TXCn � UCSRnA ����������.
			(0<<UDRIE0) |	//������ ����� ���� � ���� �������� ���������� �� ����� UDREn. ������ ���������� �������� ������ ����� �������������, ������ ���� ��� UDRIEn ������� � �������, ���������� ���� ���������� � SREG ������� � �������, � ��� UDREn � UCSRnA ����������.
			(1<<RXEN0) |	//������ ����� ���� � ���� �������� �������� USART.
			(0<<TXEN0) |	//������ ����� ���� � ���� �������� ���������� USART.
			(0<<UCSZ02) |	//���� UCSZn2, ������������ � ����� UCSZn1: 0 � UCSRnC, ������������� ���������� ����� ������ (������ SiZe) � �����, ������������ ���������� � ������������. ������������ 8
			(0<<RXB80) | 
			(0<<TXB80);
	UCSR0C=	(0<<UMSEL01) |	//������������ �����
			(0<<UMSEL00) |	//
			(0<<UPM01) |	//��� ���� �������� � ������������� ��� ��������� � �������� ��������.
			(0<<UPM00) | 
			(0<<USBS0) |	//1 ����-���*/
			(1<<UCSZ01) |	
			(1<<UCSZ00) | 
			(0<<UCPOL0);
	 	//UBRR0H=0x00;//19200
	 	//UBRR0L=0x33;
	UBRR0H = (unsigned char)(baud>>8);
	UBRR0L = (unsigned char)baud;
	asm("sei");								//��������� ����������
	
}


/***********************************************************
 * ������������� ����������
 **********************************************************/
void init_device(void)
{
	asm("cli");
	eeprom_read_block(&cfg,&ee_cfg,sizeof(conf));   //
	
	/*��� CLKPCE (7) ��������� ���������� �������� ������������. 
	��� ��������� �������� ������������ ������� � ���� ���������� �������� 1, 
	��� ���� ����� ������, ��� � ��������� ���� ������� �������� ������ ���� �������� �������� 0. 
	����� ����� � ������� ������� �������� ������ ���������� �������� �������� ������������ ������� ��������������� �����. 
	�� ���������� ������� �������� ������ ��� CLKPCE ������������ � 0.*/
	//CLKPR=(1<<CLKPCE);
	CLKPR=(0<<CLKPCE) | (0<<CLKPS3) | (0<<CLKPS2) | (0<<CLKPS1) | (0<<CLKPS0);
	
	// ------------------------------------------
	// ��������� ������
	// ------------------------------------------
	// Input/Output Ports initialization 0|1
	// �������� ����� B 
	// -------------------
	// �����
	DDRB=	(0<<DDB7) | // ���� 	
			(0<<DDB6) | // ����
			(1<<DDB5) | // �����	LED_RAZR 
			(1<<DDB4) | // �����	LED_ZAR
			(1<<DDB3) | // �����	LED_ERR
			(1<<DDB2) | // �����	LED_PWR
			(0<<DDB1) | // ����		KN
			(1<<DDB0);  // �����	��������� ����������� ������
	// ��������		
	PORTB=	(0<<PORTB7) | 
			(0<<PORTB6) | 
			(0<<PORTB5) | 
			(0<<PORTB4) | 
			(0<<PORTB3) | 
			(0<<PORTB2) | 
			(1<<PORTB1) | 	
			(0<<PORTB0);	// ���
	// �������� ����� C 
	// ------------------
	// �����
	DDRC=	(0<<DDC6) | 	// ����
			(0<<DDC5) | 	// ����
			(0<<DDC4) | 	// ����
			(0<<DDC3) | 	// ����		U_IN
			(0<<DDC2) | 	// ����		U_BAT
			(0<<DDC1) | 	// ����		U_OUT
			(0<<DDC0);		// ����
	// ��������		
	PORTC=	(0<<PORTC6) | 	
			(0<<PORTC5) | 
			(0<<PORTC4) | 
			(0<<PORTC3) | 
			(0<<PORTC2) | 
			(0<<PORTC1) | 
			(0<<PORTC0);
	// ��������� ����� D
	// -------------------
	// �����
	DDRD=	(1<<DDD7) | 	// �����	UVT9
			(1<<DDD6) | 	// �����	RAZR
			(1<<DDD5) | 	// �����	OUT_BAT
			(1<<DDD4) | 	// �����	ZAR 		
			(1<<DDD3) | 	// �����	PREOBR
			(1<<DDD2) | 	// �����
			(0<<DDD1) | 	// ����	tx
			(0<<DDD0);		// ����		rx
	// ��������		
	PORTD=	(0<<PORTD7) | 	// 
			(0<<PORTD6) | 	//
			(0<<PORTD5) | 	//
			(0<<PORTD4) | 	//
			(0<<PORTD3) | 	//
			(0<<PORTD2) | 	//
			(0<<PORTD1) | 	//
			(0<<PORTD0);	//

	// -------------------------------------------------------------------
	// ������������� �������� / ��������� (Timer/Counter) 0 
	// -------------------------------------------------------------------
	//���� COM0A1 (7) � COM0A0 (6) ������ �� ��, ����� ������ �������� �� ������ OC0A (12 �����)
	// ��� ���������� � A (���������� �������� �������� �������� TCNT0 �� ��������� �������� ��������� OCR0A):
	//00 - ����� OC0A �� �������������
	TCCR0A=	(0<<COM0A1) | 
			(0<<COM0A0) | 
			(0<<COM0B1) |	// ����� ��� 00 : ����� OC0A �� �������������
			(0<<COM0B0) | 
			(0<<WGM01) |	
			(0<<WGM00);
	//	WGM00..WGM02 = 0 :���������� ����� ������	
	TCCR0B=	(0<<WGM02) | 
			(1<<CS02) | 
			(0<<CS01) | 
			(1<<CS00);			//������� �� 1024
	//		
	//TCNT0=0x00;
	//
	OCR0A=0x00;					//������� ��������� �
	//
	OCR0B=0x00;					//������� ��������� �

	// -----------------------------------------------------------
	// ������������� �������� / ��������� (Timer/Counter) 1 
	// -----------------------------------------------------------
	//
	TCCR1A=	(0<<COM1A1) |		//����� ��� 00 : ����� �� �������������
			(0<<COM1A0) |
			(0<<COM1B1) |
			(0<<COM1B0) |
			(0<<WGM11) |
			(0<<WGM10);
//	WGM10..WGM12 = 0 :���������� ����� ������	
	TCCR1B=	(0<<ICNC1) | 
			(0<<ICES1) | 
			(0<<WGM13) | 
			(0<<WGM12) | 
			(1<<CS12) |			////������� �� 1024
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
	// ������������� �������� / ��������� (Timer/Counter 2)
	// ------------------------------------------------------------
	//
	ASSR=	(0<<EXCLK) | ////��������������� � 0 ��� ������������ �� ������
			(1<<AS2);//������������� �� ������
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

	//� ���� ��������� ����� ���������� �� ������������
	// Timer/Counter 0 Interrupt(s) initialization
	TIMSK0=	(0<<OCIE0B) | 
			(0<<OCIE0A) | 
			(1<<TOIE0);		//��������� ���������� �� ������������
			//(0<<TOIE0);
	// Timer/Counter 1 Interrupt(s) initialization
	TIMSK1=	(0<<ICIE1) | 
			(0<<OCIE1B) | 
			(0<<OCIE1A) | 
			(1<<TOIE1);		//��������� ���������� �� ������������
			//(0<<TOIE1);
	// Timer/Counter 2 Interrupt(s) initialization
	TIMSK2=	(0<<OCIE2B) | 
			(0<<OCIE2A) | 
			//(1<<TOIE2);		//��������� ���������� �� ������������
			(0<<TOIE2);
	// -------------------------------------------------------
	// ������������� �������� ����������
	// -------------------------------------------------------
	// INT0: Off
	// INT1: Off
	// ���������� ��� ����� ���������  �� pins PCINT0-7: Off
	// ���������� ��� ����� ���������  ��  PCINT8-14: Off
	// ���������� ��� ����� ���������  ��  PCINT16-23: Off
	EICRA=	(0<<ISC11) | 
			(0<<ISC10) | 
			(0<<ISC01) | 
			(0<<ISC00);
	EIMSK=	(0<<INT1) | 	// ����
			(0<<INT0);		// ����
	PCICR=	(0<<PCIE2) | 
			(0<<PCIE1) | 
			(0<<PCIE0);

	// -----------------------------------------------
	// ������������� ����������� �����������
	// ��������� ����������: Off
	// ������������� ���� ����������� ����������� ��������� � ������ AIN0
	// ������������� ���� ����������� ����������� ��������� � ������ AIN1
	ACSR=	(1<<ACD) | //���������� ��������
			(0<<ACBG) | 
			(0<<ACO) | 
			(0<<ACI) | 
			(0<<ACIE) | 
			(0<<ACIC) | 
			(0<<ACIS1) | 
			(0<<ACIS0);
	// �������� ������� ����� �� AIN0: ��������(1)
	// �������� ������� ����� �� AIN1: ��������(1)
	DIDR1=	(0<<AIN0D) | 
			(0<<AIN1D);

	// ������������� ���
	// ��� �������� �������: 125,000 kHz
	// ��� �������� ����������: AREF 
	// �������� ��������������� ������� ���: ADC Stopped
	
	//���� ADC5D - ADC0D (5 - 0) �������� DIDR0 ��������� ������������� ������ ADC5 - ADC0 (28 - 23 �����) ���� � ��� ���������� 1.
	DIDR0=	(1<<ADC5D) | 
			(1<<ADC4D) | 
			(0<<ADC3D) | 
			(0<<ADC2D) | 
			(0<<ADC1D) | 
			(0<<ADC0D);
	//		
	ADMUX=ADC_VREF_TYPE;
	//
	ADCSRA=	(1<<ADEN) | //�������� ��� ��������� ��� (1-�������).
			(1<<ADSC) | //��������� �������������� ���� � ���� �������� 1 (��� ������������� ������ ������ ������� ��������������).
			(0<<ADATE) | //��������� ��������� �������������� �� ���������� �� ������������ ��������� ���������������� ���� ���������� � 1.
			(0<<ADIF) | //���� ���������� �� ���
			(0<<ADIE) | //��������� ���������� �� ��� ���� ���������� � 1
			(1<<ADPS2) | //���� ADPS2 - ADPS0 (2 - 0) �������� ADCSRA �������� ����� ������ ������������ �������� �������:110 - CLK/128
			(1<<ADPS1) | 
			(1<<ADPS0);
	//	���� ADTS2 - ADTS0 (2 - 0) �������� ADCSRB �������� �������� ������� �� �������� ����� ���������� �������������� ���:000 - ����������� ��������������	
	ADCSRB=	(0<<ADTS2) | 
			(0<<ADTS1) | 
			(0<<ADTS0);

	// ������������� SPI 
	// SPI ��������
	SPCR=	(0<<SPIE) | 
			(0<<SPE) | 
			(0<<DORD) | 
			(0<<MSTR) | 
			(0<<CPOL) | 
			(0<<CPHA) | 
			(0<<SPR1) | 
			(0<<SPR0);

	// ������������� TWI 
	// TWI ��������
	TWCR=	(0<<TWEA) | 
			(0<<TWSTA) | 
			(0<<TWSTO) | 
			(0<<TWEN) | 
			(0<<TWIE);

	// ������������� ����������� ������� (Watchdog Timer)
	// ������������ Watchdog Timer (Prescaler): OSC/32k
	// �������� Watchdog timeout: Reset (������������)
	
	asm("wdr");	//
	//WDTCSR|=(1<<WDCE) | (1<<WDE);
	//
	WDTCSR|=(0<<WDIF) | //���� ��� ���������������, ����� � ���������� ������� ��������� ����-��� � ���������� ������ �������� �� ����������. 
			(0<<WDIE) | //			 
			(0<<WDCE) | // ���������� ������ ��������
			(0<<WDE) |	//����� ����������� ������� �������
			// Watchdog Timer 2.0 s
			(1<<WDP3) | 
			(0<<WDP2) | 
			(0<<WDP1) | 
			(1<<WDP0);
			
	MCUSR = (1<<WDRF);
/*	*/
	
	//��������� USART
	USART_Init(cfg.brate);
	//USART_Init(BR_9600);
	asm("sei"); //
}

/**
* �������� ������
* @param char* data ������ ������
* @param unsigned char sz ������ ������
*/


void transmit(unsigned char *data, uint8_t sz)
{

	unsigned char i;
	SetBit(PORTD, DRAWE);					// ����������� Max485 �� ��������
	SetBit(UCSR0B, TXEN0);					// ��������� �������� � ��
	for(i=0; i<sz; ++i)
	{	
								// ��������� ������ � UDR, ����������� ��������
	
		while ( !( UCSR0A & (1<<UDRE0) ) );	// ����� ������� ������(��������� �����) ��� ����� ��������.���������������� 1 ����� ������� ������
		UDR0 = data[i];
	}
	UDR0='\n';
	timer1ms = 0;							// �������� �������� 1 ��//   
	//while(!timer1ms);						// ����	//  ��������� ������ �� ���� �����!!!!!!!!!
	ClrBit(UCSR0B, TXEN0);					// ��������� �������� � ��
	ClrBit(PORTD,DRAWE);				// ����������� �� ����, �.�. ������� "0"
}

/**********************************************************
 * ������ crc16 ����
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
* ��������� ������� ������
*/
uint8_t razb_pac (void)
{
	uint8_t pr= 0;	// ������� ��������� ������������
	uint8_t len=0;	// ������ ��������� ������
	// �������� ������ �������� � ������ usart_buf
	// ��������� ������� ����� ������� (� 0 �����)
	// �� ����� �� ������ � ����� ���������� �����
	usart_buf[0] = cfg.adr;	
	// ��� ���������� ���������� � ���������
	usart_buf[3] = TYPE_DEF;	
	// ��������� ����� ������� � 1 �����
	// ����� ������� � ������ ��� ��
	switch(usart_buf[1]) 
	{
		case 0x03:	//������ ������������ ����������
			// ��������� �����:
			usart_buf[2] = 0x12;		// ����� ������ ������ (18 ����) ����������� � ���������			
			usart_buf[4] = cfg.adr;		//������� ����� �� ������������ 
			// �������� �� ������������(5-� ����)
			switch(cfg.brate)
			{
				case BR_2400: usart_buf[5] = ABR_2400; break;		//�������� 2400
				case BR_4800: usart_buf[5] = ABR_4800; break;		//�������� 4800
				case BR_9600: usart_buf[5] = ABR_9600; break;		//�������� 9600
				case BR_19200: usart_buf[5] = ABR_19200; break;		//�������� 19200 
				case BR_38400: usart_buf[5] = ABR_38400; break;		//�������� 38400
				case BR_57600: usart_buf[5] = ABR_57600; break;		//�������� 57600
				case BR_115200: usart_buf[5] = ABR_115200; break;	//�������� 115200
				 
			}
			*((unsigned short int*)(usart_buf+6)) = cfg.Uz;									// ���������� ��������� ������ 
			*((unsigned short int*)(usart_buf+8)) = cfg.Um;									// ���������� ������ ������			
			*((unsigned long*)(usart_buf+10)) = SERIAL_NUMBER;		//�������� ����� (8,9,10,11 ����)
			usart_buf[14] = VERSION_Y;								//������ �� ����� (12 ����)
			usart_buf[15] = VERSION_X;								//������ �� �����(13 ����)
			// ����� ������ �����������
			break;
		case 0x10: 	//��������� ������������ ����������
				//������ ������������ (4 ���� � ������)
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
				// ���������� ������� ����� (5-� ���� � ������)
				cfg.adr = usart_buf[5];					
				// ���������� ��������� ������ (6-� ���� � ������)	
				cfg.Uz = *((unsigned short int*)(usart_buf+6));					
				// ���������� ������ ������ (7-� ���� � ������)
				cfg.Um = *((unsigned short int*)(usart_buf+8));							
				// ��������� ����� ������������
				eeprom_write_block(&cfg,&ee_cfg, sizeof(conf));		
				// ������������� ������� ��������� ������������	
				pr = 1;													
				// ��������� �����				
				usart_buf[2] = 0x06;		//����� ������ (2-� ����) 
				break;
		case 0x04: //������ ���������
				// ��������� �����
				
				usart_buf[2] = 0x11;		//����� ������ �� ���������
				usart_buf[4] = flagi;		// ���� ������ ���������	
				*((unsigned long*)(usart_buf+5)) = Tp;	// ���������� ����� ������	����� 5,6,7,8	
				*((unsigned short int*)(usart_buf+9)) = read_adc(Uin);
				*((unsigned short int*)(usart_buf+11)) = read_adc(Uout);
				*((unsigned short int*)(usart_buf+13)) = read_adc(Ubat);
					
				break;
		default: 	//����������� �������						
			usart_buf[1] |= 0x80; // 1 � ������� ����� ���� �������(������� ������)
			usart_buf[2] = 0x06;     // ����� ������ (2 ����)
		}
		//��������� len �������� ����� ������ ��� crc			
		len  = usart_buf[2]-2;
		// ��������� �� �����  crc (uint16_t 2 ��������� �����)
		*( (uint16_t*)(usart_buf+len) ) = crc16 ((char*)usart_buf, len);	
		//��������� �������������� ����� ������
		//transmit(usart_buf, usart_buf[2]);
		transmit((unsigned char*)usart_buf, usart_buf[2]);							
		// ���������� ������� ��������� ������������
		return pr;
} 

/**
* �������� ������� ������� � ����������� ������
*/ 
void parsing_package(void)			//������ �������
{	
	uint8_t x = 0;	// ������� ��������� ������������
	uint8_t len=0;	// ����� ������	
		
	// 1. �������� ���-�� �������� ���� (2 ���� ������)
	// --------------------------------------------
	// ���� ���-�� �������� ���� ������ ������� ���������� � ������ (���� 2)	
	if(received_bytes < usart_buf[2])
	{
		// ����� �� �������
		return;
	}
	// ���������� ������ ������
	len = usart_buf[2];			
	// ��������� ���������� �� �����
	UCSR0B &= ~RXE;					
	// 2. �������� �������� ������
	// --------------------------
	// ����� ��������� �� 0 �����. 
	//  ���� ����� ��������� ��� 0 (�����������������)
	if( (cfg.adr == usart_buf[0]) || (0 == usart_buf[0]) )		
	{
		// 3. �������� crc ����
		// ---------------------
		uint16_t crc2 = crc16((char*)usart_buf, len-2);			// ���������� CRC ���
		uint16_t crc3 = *((uint16_t*)(usart_buf+len-2));		// �������� CRC ��� �� ������
		// ���� ����������� CRC ��������� � ���������
		if( crc2 == crc3 )										
		{
			// 4. ��������� ��� ����������
			//���� ��� ���������� ���������
			if(usart_buf[3]== TYPE_DEF)				
			{
				//transmit((char*)usart_buf, usart_buf[2]);
				// ������������ �����
				x = razb_pac();								
			}
		}
	}

	//������� �� ����, ����� ���� ������
	received_bytes = 0;	//�������� ���������� �������� ����
	UCSR0B |= RXE;		//��������� ���������� �� �����
	// ���� ������������ ����������
	if (x)
	{
		// ��������� ���������
		USART_Init(cfg.brate);	
	}
}
/**
*
*@param unsigned char adc_input 
*/
//������� ������ � ����������� �����
//���������� �������� ��� 0..1024
unsigned int read_adc(unsigned char adc_input)
{
	ADMUX = ADC_VREF_TYPE | adc_input;
	//�������� ���������� ��� ������������ �������� ���������� ���
	_delay_us(10);
	//������ ��������������
	ADCSRA|=(1<<ADSC);
	// ��������� ���������� ����������� AD
	while ((ADCSRA & (1<<ADIF))==0);
	ADCSRA|=(1<<ADIF);
	return (uint16_t)ADC;
}

/**
* ����� ������������
*/
 void regim_test(void)							//
 {
 	 uint16_t U2 = 0;							// �������� ����������

	 if((flagi&(1<<4))==0)							//���� ��� ������, �� �������� ����
	 {

		 if(count_period_test>Ust_period_test)		//���� ��������� ����� �����
		 {					 
			 if(!Flag_test)							//���� ���� �� �����
			 {
				 count_time_test = 0;				//�������� ������� ������������ �����
				 Flag_test = 1;						//���������� ���� ������ �����
				 U1=read_adc(Ubat);					//�������� ��������� ���������� ������������
				 // ��������� ����� ������ ���� �� ����������, � �� ��������� �����
				 ZAR_OFF();							//���������� ��������������� �����
				 RAZR_ON();							// �������� ��������
				 OUT_BAT_OFF();						//��������� ������� �� �������
				 LED_RAZR_ON();						//��������� ���� ����
				 
				 #if DEBUG
				 x="nachalo_testa\r\n";
				 transmit((uint8_t*)x,strlen(x));
				 x="\r\n";
				 transmit((uint8_t*)x,strlen(x));
				 
				  //sprintf(x,"nach_testa U=%.2f ",read_adc(Ubat));
				  //transmit((uint8_t*)x,strlen(x));
				 #endif
			 }
			 else //���� ���� �������
			 {
				 if(count_time_test>Ust_time_test)	//��������� ��������� �����
				 {
					 #if DEBUG
					 x="M5 \r\n";
					 transmit((uint8_t*)x,strlen(x));
					 x="\r\n";
					 transmit((uint8_t*)x,strlen(x));
					 #endif
					 
					 U2=read_adc(Ubat);				//���� ����� ����� ����� ������� �������� ���������� ���
					 count_period_test = 0;			//�������� ������� ������� ���������� �����
					 Flag_test = 0;					//�������� ���� �����
					 RAZR_OFF();					//��������� ������
					 OUT_BAT_OFF();					//���������� ����� �� ����� � ������ �� ����
					 ZAR_ON();
					 LED_RAZR_OFF();				//��������� ��������� �����
					 
					 if((U1-U2)> Pad_napr_test)		//���� ������� ���������� �� ������������ ��� ����� ������ ��������������,
					 {//�� ��� ������
						 //status_akkum = 0;			//���������� ���� ������� ������������
						 ClrBit(flagi,6);			// ���������� ������ "������"(�������� ��� 6)
						 #if DEBUG
						x= "_________AKB_bad___________\r\n";
						transmit((uint8_t*)x,strlen(x));
						#endif
						 
					 }
					 else
					 {
						 //status_akkum = 1;			//
						 SetBit(flagi,6);			//����� ���������� ��� 6
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

 //������� ������� ������� ������ �� ������������
 //��� ���������� ������� �� ���� ��� U1=U2, Tp = 0xFFFFFFFF;
 void time_work(void)
 {	 
	int U2 = 0;			//�������� ����������
	 
	 if(U1!=U2)
	 {		 
	 U2 = read_adc(Ubat);
	 Tp = ((T)*(Umin-U1))/(U2-U1);		//������ ����������� ������� ������ �� ���������� ���������� Umin (.���)
	 
	 
	 
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
				
					if(read_adc(Ubat)>cfg.Uz)		//���� ���������� �������� ������ ����������� ������������					
					{
  					
									/*			 		 
									if(flag_d)				//���� ���������� ���� ��������
									{
									#if DEBUG
									x=" dozar ";
									transmit((uint8_t*)x,strlen(x));
									ltoa((long)count_time_zar,x,10);
									transmit((uint8_t*)x,strlen(x));
									#endif
																												
												if(count_time_zar>UST_time_zar)
												{
												flag_z = 0;				//����� ���� ������
												flag_d = 0;				//����� ���� ��������
												ZAR_OFF();						//��������� �����
												LED_ZAR_OFF();					//��������� ���������� ������
	
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
									flag_d = 1;				//���� ���� �������� �� ����������, ����������
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
							ZAR_OFF();						//��������� �����
							LED_ZAR_OFF();					//��������� ���������� ������		
							//flag_z = 0;						//�������� ���� ������
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
																
						if(read_adc(Ubat)<cfg.Um)		//���� ���������� ������������ ���� ���������, �� �������� �����������
						
						{
							SetBit(flagi,2);				//���������� ���� ������
							ZAR_ON();						//�������� �����������
							LED_ZAR_ON();					//�������� ��������� ������
							
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
																							
							//flag_z = 1;				//���������� ���� ������
							
						}
						else
						{													
							regim_test();					//������� �����.
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