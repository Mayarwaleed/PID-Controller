/*
 * LCD_config.h
 *
 * Created: 8/29/2023 4:39:11 PM
 *  Author: youssef
 */ 


#ifndef LCD_CONFIG_H_
#define LCD_CONFIG_H_

#if LCD_MODE == EIGHT_BIT_MODE
	#define LCD_DPORT   PORTA 
#elif LCD_MODE == FOUR_BIT_MODE
	#define LCD_DPORT   PORTC
	#define LCD_D4_PIN  PIN3
	#define LCD_D5_PIN  PIN4
	#define LCD_D6_PIN  PIN5
	#define LCD_D7_PIN  PIN6
#endif

#define LCD_CPORT  PORTC

#define LCD_RS_PIN PIN0
#define LCD_RW_PIN PIN1
#define LCD_EN_PIN PIN2


#endif /* LCD_CONFIG_H_ */