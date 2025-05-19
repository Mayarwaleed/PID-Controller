/*
 * intrupt_vector.h
 *
 * Created: 10/1/2023 2:02:09 PM
 *  Author: user
 */ 


#ifndef INTRUPT_VECTOR_H_
#define INTRUPT_VECTOR_H_

#define ISR(__vector_NUM) void __vector_NUM(void)__attribute__((signal)) ;\
						  void __vector_NUM(void)

#define INT_0_ __vector_1
#define INT_1_  __vector_2
#define INT_2_ __vector_3

#define TIMER0_OVF __vector_11
#define TIMER0_CTC __vector_10


#endif /* INTRUPT_VECTOR_H_ */