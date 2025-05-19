/*
 * UART_interface.c
 *
 * Created: 10/13/2023 3:40:33 PM
 * Author : Ibrahim
 */ 

#include "MATH.h"
#include "STD_TYPE.h"
#include "UART_private.h"
#include "UART_interface.h"

/* Note: CLR_BIT instructions are optional if the bits are zero by default */

void UART_Vid_Init(void){
	
	
	u8 x = 0;
	
	/* Select UCSRC register */
	SET_BIT(x,URSEL_BIT);
	
	/* Select Asynchronous mode */
	CLR_BIT(x,UMSEL_BIT);
	
	/* Select 1-bit stop bit */
	CLR_BIT(x,USBS_BIT);
	
	/* Clock Polarity >> Asynchronous mode: write 0 */
	CLR_BIT(x,UCPOL_BIT);
	
	/* Select character size: 8-bit */
	SET_BIT(x,UCSZ1_BIT);
	SET_BIT(x,UCSZ0_BIT);
	UCSRC_REG = x;
	
	/* Set baud rate: 9600 bps */
	/* F = 16MHz , U2X = 0 */
	UBRRL_REG = 103;
	
	/* Enable receiver and transmitter */
	SET_BIT(UCSRB_REG,RXEN_BIT);
	SET_BIT(UCSRB_REG,TXEN_BIT);
}

void UART_Vid_Transmitter(u8 copy_u8_message){
	
	UDR_REG = copy_u8_message;
	/* Wait until the message is sent (USART Transmit Complete / Data Register Empty) */
	
	while(GET_BIT(UCSRA_REG,UDRE_BIT) == 0);
	
	/* The Same :) */
	//while(GET_BIT(UCSRC_REG,TXC_BIT) == 0);	
}

u8 UART_Vid_Receiver(void){
	
	/* Wait until the message is received */
	while(GET_BIT(UCSRA_REG,RXC_BIT) == 0);
	return UDR_REG;
}
void UART_Vid_SendString(ch8* str) {
	u8 i = 0;
	while (str[i] != '\0') {
		UART_Vid_Transmitter(str[i]);
		i++;
	}
}