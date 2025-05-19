/*
 * UART_interface.h
 *
 * Created: 10/13/2023 3:41:03 PM
 * Author : Ibrahim
 */ 


#ifndef UART_INTERFACE_H_
#define UART_INTERFACE_H_

void UART_Vid_Init(void);

void UART_Vid_Transmitter(u8 copy_u8_message);

u8 UART_u8_Receiver(void);

void UART_Vid_SendString(ch8* str);
#endif /* UART_INTERFACE_H_ */