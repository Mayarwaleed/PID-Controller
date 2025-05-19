/*
 * Timer0_interface.c
 *
 * Created: 9/26/2023 3:40:26 PM
 *  Author: user
 */ 
#include "DIO_interface.h"
#include "DIO_private.h"
#include "MATH.h"
#include "STD_TYPE.h"
#include "Timer0_private.h"
#include "Timer0_interface.h"
#include "intrupt_vector.h"
#include "GIE_interface.h"


void(*OVR_MODE_ptr)(void)=null;//ptr to overflow function
void(*CTC_MODE_ptr)(void)=null;//ptr to ctc function

void TIM0_Vid_SetCallBack(u8 copy_u8_mode,void(*ptr_fun)(void))
{
	if (copy_u8_mode==OVR_MODE)
	{
		OVR_MODE_ptr = ptr_fun;
	}
	else if (copy_u8_mode==CTC_MODE)
	{
		CTC_MODE_ptr = ptr_fun;
	}
}


void Tim0_Vid_Init(u8 copy_u8_mode)
{
	/*Select prescaler*/
	TCCR0_REG&=0b11111000;
	TCCR0_REG|=TIMER0_PRESCALLER;
	/*enable global intrrupt*/
	GIE_Vid_Interrupt_Enable();
	
	if(copy_u8_mode==OVR_MODE)
	{
		/*Enable overflow intrrupt*/
		SET_BIT(TIMSK_REG,0);
	}
	else if(copy_u8_mode==CTC_MODE)
	{
		CLR_BIT(TCCR0_REG,6);
		SET_BIT(TCCR0_REG,3);
		
		OCR0_REG=OCR0_VAL;
		
		SET_BIT(TIMSK_REG,1);
	}
}

void Tim0_Vid_DelayMs(u32 copy_u32_delay)
{
	u32 count = 0;
	copy_u32_delay = (f32)copy_u32_delay/1.024;
	while(count < copy_u32_delay)
	{
		TCCR0_REG &= 0b11111000; 
		TCCR0_REG |= TIMER0_PRESCALLER; //Start 64 PRESCALER
		while(DIO_u8_Get_Pin_Val(TIFR_REG,PIN0) == 0);
		count++;
		
	}
	TCCR0_REG = TIMER_STOP;
}
void tim0_vid_delay_us(u32 copy_u32_delay)
{
	
	
}
void Tim0_Vid_Fast_PWM(u8 copy_u8_DutyCycle)
{
	/*Select prescaler*/
	TCCR0_REG&=0b11111000;
	TCCR0_REG|=TIMER0_PRESCALLER;
	
	/*FastPwmMODE*/
	SET_BIT(TCCR0_REG,3);
	SET_BIT(TCCR0_REG,6);
	
	/*non inverting mode*/
	SET_BIT(TCCR0_REG,5);
	CLR_BIT(TCCR0_REG,4);
	
	/*SET OCR VALUE*/
	OCR0_REG = copy_u8_DutyCycle * 2.56;
	/*Wait till ctc happen*/
	while (GET_BIT(TIFR_REG,1)==0);
	/*Clear flag bit*/
	SET_BIT(TIFR_REG,1);
	
}


ISR(TIMER0_OVF)
{
	OVR_MODE_ptr();
}
ISR(TIMER0_CTC)
{

	CTC_MODE_ptr();
}