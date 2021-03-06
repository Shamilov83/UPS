/**
????????? ?????????? ?????????????? ????????? ??????? ?? ???? ??????? ???? ???. 
???????????? ???????? ???????? ? ????????? ??????????, ?????????? ????????????.
??? ?????? ?? ???? ???????? ??????? ?????????? ??????? ???????????? ?? ???? ??? ???????,
? ????? ???????????? ???????????? ???????????? (???? ??? ? ???? ?????) ????? ??????????? ?? 1 ???. ???????????? ?????????.
???? ?????????? ???????????? ??? ????? ??????? ????? 2 ????? ??????????????? ?????? ? ?????????? ????????????.
??? ?????????? ?????????? ???? ???????? ??????? ????????? ?? ?????? ?? ???????????? ? ??????????????? ?????????????? ????. 
?????? 30 ?  ???????????? ????? ?????????? ???????????? ? ?? ????????? ???? ?????? ???????????? ?????? ??????????? 
??????? ?????? ?????????? ?? ?????????? ????????, ???????? ??????? ???????????? ? ??????. 
 ??? ??????? ?????????? ???????????? ?? ?????????? ??????, ??????? 7 ?????,  ??????????? ????????. 
*/

#include "mdef.h"

volatile uint16_t counter_test =0;
/**********************************************************
 * ????????? ??????????
 */
ISR(TIMER1_OVF_vect)			//????????? ???????? ??????? ??????? ????? T1. ?????? ?????? ?????????? ??? ??????? 16??? 238??
{
	TCNT1 = ust_tc1;			//????????????? ???????-????????1,  ???????????? ?????? 8?		
	//counter_test++;				//???????? ???????
}
/**********************************************************
 *   ???????????? ?? ???????
 * 	 ?????? 10 ????????????
 *********************************************************/

ISR(TIMER0_OVF_vect)			
{
	TCNT0 = ust_tc0;			//????????????? ???????-????????0,  ???????????? ?????? 1??
	timer1ms++;					//?????????? ?????????? ?????????
	count_time_zar++;			//??????? ??????? ??????
	count_period_test++;		//????????? ???????? ??????? ??????? ????? T1		
	count_time_test_T2++;		//??????? ??????? ?????? ?????????? ???????
	count_time_test++;			//????????? ???????? ???????????? ?????
	counter_test++;				//???????? ???????
}


/**********************************************************
 *  USART, ????? ????? ????????
 *********************************************************/ 
ISR(USART_RX_vect)
{
	unsigned char temp;
	timer1ms  = 0;						//???????? ??????? ????? ???????
	temp = UDR0;						//???????? ? ?????? ?? UDR ?????? ??????
	usart_buf[received_bytes] = temp;	//????????? ? ????? ??????
	received_bytes++;					//? ????????? ??? ?????????? ? ????.??????
	if(received_bytes > SIZE_BUF) 		//???? ????? ??????????
	{
		received_bytes = 0;				//?? ???????? ?????? ???-?? ???? ? ??????
	}
	
}

	
int main(void)
{
	flagi = 0;
	flag_d = 0;
	flag_z = 0;
	
	
	received_bytes = 0;						// ?????????? ???????? ???? ? ??????
	timer1ms=0;								// ??????? ????? ????? ???????
	init_device();							// ????????????? ???????????


	while (1) 
	{
		
	_delay_ms(10);
	
		//????????? ???? ?? ?????? ??? ??????
		if (received_bytes && (timer1ms >= TIME_BETWEEN_BYTE))
	
		{
			parsing_package();		//????????? ???????						
		} 	
	
		
	#if DEBUG
	if(count_time_zar>1000)
	{
		x="_____________flag_z = ";
		transmit((uint8_t*)x,strlen(x));
		itoa(flag_z,x,10);
		transmit((uint8_t*)x,strlen(x));
		
		x="______________\r\n";
		transmit((uint8_t*)x,strlen(x));
		
		x="_______________Ubat = ";
		transmit((uint8_t*)x,strlen(x));
		
		itoa(read_adc(Ubat),x,10);
		transmit((uint8_t*)x,strlen(x));
		
		x="______________\r\n";
		transmit((uint8_t*)x,strlen(x));
		count_time_zar=0;
	}
	#endif

	
		asm("wdr");		//????????? ??????????? ???????		 
			//???? ??????? ?????????? ???? ?????????, ?? 
			//?????? ?? ??? 
			if(read_adc(Uin)<UST_Uin)
			//if(0)						
			{				

							OUT_BAT_ON();				//?????????? ??????????????? ? ????????????		
							NAGR_ON();					// ???????? ????????		
							SetBit(flagi,5);			//?????????? ???? ????????? ????????				
							PREOBR_OFF();				// ????????? ????				
							ZAR_OFF();					// ????????? ???????  				
							SetBit(flagi,7);			//?????????? ???? ?????? ?? ???????????? (??? 7<<1)				
							Flag_test = 0;				//???????? ???? ?????????? ????? ?? ?????? ???? ?????????? ???? ?????????? ? ?????? ?????							
							LED_AKK_ON();				// ????????? ?????? ?? ???????				
							LED_PWR_OFF();				//????????? ????????? ?????? ?? ????
							RAZR_OFF();					//?????????? ????, ???? ???????
							LED_RAZR_OFF();				//????????? ????????? ?????					
							LED_ZAR_OFF();				//????????? ????????? ??????				
							count_period_test = 0;		//???????? ??????? ??????? ?????.
								
									if(count_time_test_T2>=Ust_count2)		
									{  																		
										//????????? ? ???????? ????? ??????
										time_work();
										//???????? ??????? ??????? ?????? ?????????? 
										// ???????????? ??? ?????????? ??????
										count_time_test_T2 = 0;																		
									}
			}
			//???? ????, ?? 
			//?????? ?? ????
			else
			{	
				Tp = 0xFFFFFFFF;
				LED_AKK_OFF();				//????????? ????????? ?????? ?? ????????????
				LED_PWR_ON();				//???????? ????????? ?????? ?? ????
				PREOBR_ON();				//?????????? ??????????????? ? ????
				NAGR_ON();					//?????????? ????????
				SetBit(flagi,5);			//?????????? ???? ????????? ????????
				OUT_BAT_OFF();				//????????? ??????????????? ?? ????????????			
				ClrBit(flagi,7);			//??? ?????? ?? ???? ???????? ??? 7



						//????????? ???? ?????????? ?????. ???? ???? ?? ??????????? ????????? ????? ??????? ? ????????????? ?????
						//????? ??????? ??????? ?????, ??????? ???????? ???????????? ????? ? ????????? ???? ??????????? ????????????
						if(!Flag_test)
						{
							
							//????????? ???? ??????? ??????, ???? ??????? ????????? ?????? ?? ????? ??????
							//???? ?? ??????????, ?? ????????? ?????????? ?? ???????? 
							
							zaryd();	
						}
						else
						{							
							regim_test();			//??????? ?????.	
						}
						
						
			}	
					
				//???????? ????????? ??????????
				if(read_adc(Uout)<UST_Uout)
				//if(0)	
			{
				ClrBit(flagi,5);				//?????????? ???? ?????????? ????????
				//ZAR_ON();						//???????? ???? ??????
				PREOBR_ON();					//?????????? ???????????????
				LED_ERR_ON();					//???????? ????????? "??????"
				SetBit(flagi,4);				//?????????? ???? "??????"
				RAZR_OFF();						//?????????? ????
				LED_RAZR_OFF();
				
				#if DEBUG

				ltoa((long)read_adc(Ubat),x,10);
				x="___________________Uout_OFF";
				transmit((uint8_t*)x,strlen(x));
				x="____________________\r\n";
				transmit((uint8_t*)x,strlen(x));
				#endif
			}
			else
			{
				LED_ERR_OFF();					//????????? ????????? "??????"
				ClrBit(flagi,4);				//????? ???? "??????"	
				NAGR_ON();
											
			}

	}
}