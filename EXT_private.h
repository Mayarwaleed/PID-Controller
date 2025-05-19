/*
 * EXT_private.h
 *
 * Created: 10/09/2023 16:27:09
 *  Author: AMIT
 */ 


#ifndef EXT_PRIVATE_H_
#define EXT_PRIVATE_H_


#define MCUCR_REG      *((volatile u8*)0x55)

#define  ISC11    3 
#define  ISC10    2 
#define  ISC01    1
#define  ISC00    0

#define MCUCSR_REG      *((volatile u8*)0x54)

#define  ISC2    6

#define GICR_REG      *((volatile u8*)0x5B)

#define  INT_1    7 
#define  INT_0    6
#define  INT_2    5 
#endif /* EXT_PRIVATE_H_ */