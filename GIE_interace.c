

#include"MATH.h" 
#include"STD_TYPE.h" 
#include"GIE_privte.h" 


void GIE_Vid_Interrupt_Enable(){
	
	//SET_BIT(SREG_REG,7) ;
	
	   __asm volatile("SEI");
	
}

void GIE_Vid_Interrupt_Disable(){
	
	//CLR_BIT(SREG_REG,7) ;
	  __asm volatile("CLI");
} 