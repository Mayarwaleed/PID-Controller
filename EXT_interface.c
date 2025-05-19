/*
 * EXT_interface.c
 *
 * Created: 10/09/2023 16:27:23
 *  Author: AMIT
 */ 
#include "MATH.h" 
#include "STD_TYPE.h" 
#include "EXT_private.h" 
#include "EXT_interface.h"

void(*ptr_set)(void)=0;
void EXT_Vid_Interrupt_Init(){
	
	#if INT_SENSE_CONTROL == LOW_LEVEL 
	
				/* INT1 */ 
				CLR_BIT(MCUCR_REG,ISC11) ; 
				CLR_BIT(MCUCR_REG,ISC10) ;
	            /* INT0 */ 
				CLR_BIT(MCUCR_REG,ISC01) ;
				CLR_BIT(MCUCR_REG,ISC00) ;
	#elif INT_SENSE_CONTROL== ANY_LOGIC_CHANGE 
					/* INT1 */
					CLR_BIT(MCUCR_REG,ISC11) ;
					SET_BIT(MCUCR_REG,ISC10) ;
					/* INT0 */
					CLR_BIT(MCUCR_REG,ISC01) ;
					SET_BIT(MCUCR_REG,ISC00) ;
	#elif   INT_SENSE_CONTROL == FALLING_EDGE 
					/* INT1 */
					SET_BIT(MCUCR_REG,ISC11) ;
					CLR_BIT(MCUCR_REG,ISC10) ;
					/* INT0 */
					SET_BIT(MCUCR_REG,ISC01) ;
					CLR_BIT(MCUCR_REG,ISC00) ;
					/* INT2*/
                    CLR_BIT(MCUCSR_REG,ISC2) ; 
	#elif    INT_SENSE_CONTROL==RISING_EDGE
						/* INT1 */
						SET_BIT(MCUCR_REG,ISC11) ;
						SET_BIT(MCUCR_REG,ISC10) ;
						/* INT0 */
						SET_BIT(MCUCR_REG,ISC01) ;
						SET_BIT(MCUCR_REG,ISC00) ;
						/* INT2*/
						SET_BIT(MCUCSR_REG,ISC2) ;
						
	#endif					
				   	
}

void EXT_Vid_Enable_Interrupt0(){
	
	/* enable interrupt (PIE)*/
	SET_BIT(GICR_REG,INT_0) ; 
}

void EXT_Vid_Enable_Interrupt1(){
	/* enable interrupt (PIE)*/
	SET_BIT(GICR_REG,INT_1) ; 
}
void EXT_Vid_Enable_Interrupt2(){
	/* enable interrupt (PIE)*/
	SET_BIT(GICR_REG,INT_2) ; 
	
}

void Set_fun(void(*ptr)(void)){
	
	ptr_set=ptr;
}

void __vector_2(void)__attribute__((signal)) ;
void __vector_2(void){

	ptr_set();
	
	
	
}
