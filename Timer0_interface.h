/*
 * Timer0_interface.h
 *
 * Created: 9/26/2023 3:39:28 PM
 *  Author: user
 */ 


#ifndef TIMER0_INTERFACE_H_
#define TIMER0_INTERFACE_H_
void Tim0_Vid_Init(u8 copy_u8_mode);
void Tim0_Vid_DelayMs(u32 copy_u32_delay);
void Tim0_Vid_DelayUs(u32 copy_u32_delay);
void TIM0_Vid_SetCallBack(u8 copy_u8_mode,void(*ptr_fun)(void));
void Tim0_Vid_Fast_PWM(u8 copy_u8_DutyCycle);

#define     TIMER_STOP		     0X00
#define		NO_PRE_SCALLER		 0X01
#define		_8_PRESCALER		 0X02
#define		_64_PRESCALER		 0X03
#define		_256_PRESCALER	     0X04
#define		_1024_PRESCALER	     0X05

#define		TIMER0_PRESCALLER	 _64_PRESCALER


#define PRE_SC_MODE CLKTWO
/*MODES*/
#define CTC_MODE 1
#define	OVR_MODE 2
/*OCR VALUE*/
#define OCR0_VAL 250

#endif /* TIMER0_INTERFACE_H_ */