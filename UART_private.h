/*
 * UART_private.h
 *
 * Created: 10/13/2023 3:41:32 PM
 * Author : Ibrahim
 */ 


#ifndef UART_PRIVATE_H_
#define UART_PRIVATE_H_

/* UBRRH & UCSRC registers share the same I/O address */

#define UBRRH_REG	*((volatile u8*)0x40)
	#define URSEL_BIT	7

#define UCSRC_REG	*((volatile u8*)0x40)
	#define URSEL_BIT	7
	#define UMSEL_BIT	6
	#define UPM1_BIT	5
	#define UPM0_BIT	4
	#define USBS_BIT	3
	#define UCSZ1_BIT	2
	#define UCSZ0_BIT	1
	#define UCPOL_BIT	0

#define UDR_REG		*((volatile u8*)0x2C)

#define UCSRA_REG	*((volatile u8*)0x2B)
	#define RXC_BIT		7
	#define TXC_BIT		6
	#define UDRE_BIT	5
	#define FE_BIT		4
	#define DOR_BIT		3
	#define PE_BIT		2
	#define U2X_BIT		1
	#define MPCM_BIT	0

#define UCSRB_REG	*((volatile u8*)0x2A)
	#define RXCIE_BIT	7
	#define TXCIE_BIT	6
	#define UDRIE_BIT	5
	#define RXEN_BIT	4
	#define TXEN_BIT	3
	#define UCSZ2_BIT	2
	#define RXB8_BIT	1
	#define TXB8_BIT	0
	
#define UBRRL_REG	*((volatile u8*)0x29)

#endif /* UART_PRIVATE_H_ */