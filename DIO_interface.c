/*
 * DIO_interface.c
 *
 * Created: 8/27/2023 3:45:21 PM
 *  Author: youssef
 */ 

#include "MATH.h"
#include "STD_TYPE.h"
#include "DIO_private.h"
#include "DIO_interface.h"

void DIO_Vid_Set_Pin_Dir(u8 Copy_u8_port, u8 Copy_u8_pin, u8 Copy_u8_dir) {
	if(Copy_u8_dir==OUTPUT || Copy_u8_dir==INPUT){
		if (Copy_u8_dir==OUTPUT){
			
			switch(Copy_u8_port){
				
				case PORTA : SET_BIT(DDRA_REG,Copy_u8_pin); break;
				case PORTB : SET_BIT(DDRB_REG,Copy_u8_pin); break;
				case PORTC : SET_BIT(DDRC_REG,Copy_u8_pin); break;
				case PORTD : SET_BIT(DDRD_REG,Copy_u8_pin); break;
			}
		}
		else {
			
			switch(Copy_u8_port){
				
				case PORTA : CLR_BIT(DDRA_REG,Copy_u8_pin); break;
				case PORTB : CLR_BIT(DDRB_REG,Copy_u8_pin); break;
				case PORTC : CLR_BIT(DDRC_REG,Copy_u8_pin); break;
				case PORTD : CLR_BIT(DDRD_REG,Copy_u8_pin); break;
			}
		}
	}
}

void DIO_Vid_Set_Pin_Val(u8 Copy_u8_port, u8 Copy_u8_pin, u8 Copy_u8_val) {
	if(Copy_u8_val==HIGH || Copy_u8_val==LOW){
		if (Copy_u8_val==HIGH){
			
			switch(Copy_u8_port){
				
				case PORTA : SET_BIT(PORTA_REG,Copy_u8_pin); break;
				case PORTB : SET_BIT(PORTB_REG,Copy_u8_pin); break;
				case PORTC : SET_BIT(PORTC_REG,Copy_u8_pin); break;
				case PORTD : SET_BIT(PORTD_REG,Copy_u8_pin); break;
			}
		}
		else {
			
			switch(Copy_u8_port){
				
				case PORTA : CLR_BIT(PORTA_REG,Copy_u8_pin); break;
				case PORTB : CLR_BIT(PORTB_REG,Copy_u8_pin); break;
				case PORTC : CLR_BIT(PORTC_REG,Copy_u8_pin); break;
				case PORTD : CLR_BIT(PORTD_REG,Copy_u8_pin); break;
			}
		}
	}
}

u8   DIO_u8_Get_Pin_Val(u8 Copy_u8_port, u8 Copy_u8_pin) {
	u8 LOC_u8_VAL = 0;
	switch(Copy_u8_port){
		
		case PORTA : LOC_u8_VAL =  GET_BIT(PINA_REG,Copy_u8_pin); break;
		case PORTB : LOC_u8_VAL =  GET_BIT(PINB_REG,Copy_u8_pin); break;
		case PORTC : LOC_u8_VAL =  GET_BIT(PINC_REG,Copy_u8_pin); break;
		case PORTD : LOC_u8_VAL =  GET_BIT(PIND_REG,Copy_u8_pin); break;
	}
	return LOC_u8_VAL;
}

void DIO_u8_Toogle_Pin_Val(u8 Copy_u8_port, u8 Copy_u8_pin) {
	switch(Copy_u8_port){
		case PORTA : TOOGLE_BIT(PORTA_REG,Copy_u8_pin); break;
		case PORTB : TOOGLE_BIT(PORTB_REG,Copy_u8_pin); break;
		case PORTC : TOOGLE_BIT(PORTC_REG,Copy_u8_pin); break;
		case PORTD : TOOGLE_BIT(PORTD_REG,Copy_u8_pin); break;
	}
}

void DIO_Vid_Set_Port_Dir (u8 Copy_u8_port, u8 Copy_u8_dir) {
	
	if (Copy_u8_dir==PORT_OUTPUT){
			
		switch(Copy_u8_port){
				
			case PORTA : DDRA_REG = PORT_OUTPUT; break;
			case PORTB : DDRB_REG = PORT_OUTPUT; break;
			case PORTC : DDRC_REG = PORT_OUTPUT; break;
			case PORTD : DDRD_REG = PORT_OUTPUT; break;
		}
	}
	else {
			
		switch(Copy_u8_port){
				
			case PORTA : DDRA_REG = PORT_INPUT; break;
			case PORTB : DDRB_REG = PORT_INPUT; break;
			case PORTC : DDRC_REG = PORT_INPUT; break;
			case PORTD : DDRD_REG = PORT_INPUT; break;
		}
	}
}

void  DIO_Vid_Set_Port_Val (u8 Copy_u8_port, u8 Copy_u8_val) {
		
	switch(Copy_u8_port){
			
		case PORTA : PORTA_REG =  Copy_u8_val; break;
		case PORTB : PORTB_REG =  Copy_u8_val; break;
		case PORTC : PORTC_REG =  Copy_u8_val; break;
		case PORTD : PORTD_REG =  Copy_u8_val; break;
	}
}