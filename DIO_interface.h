/*
 * DIO_INTERFACE.h
 *
 * Created: 8/27/2023 3:43:16 PM
 *  Author: youssef
 */ 


#ifndef DIO_interface_H_
#define DIO_interface_H_

/*****************************************************************************
* Function Name: DIO_Vid_Set_Pin_Dir
* Purpose      : set pin dir (OUTPUT , INPUT)
* Parameters   : u8 Copy_u8_port,u8 Copy_u8_pin,u8 Copy_u8_dir
* Return value : void
*****************************************************************************/
void DIO_Vid_Set_Pin_Dir(u8 Copy_u8_port, u8 Copy_u8_pin, u8 Copy_u8_dir) ;
/*****************************************************************************
* Function Name: DIO_Vid_Set_Pin_Val
* Purpose      : set pin val (HIGH , LOW)
* Parameters   : u8 Copy_u8_port,u8 Copy_u8_pin,u8 Copy_u8_val
* Return value : void
*****************************************************************************/
void DIO_Vid_Set_Pin_Val(u8 Copy_u8_port, u8 Copy_u8_pin, u8 Copy_u8_val) ;
/*****************************************************************************
* Function Name: DIO_u8_Get_Pin_Val
* Purpose      : GET PIN  (HIGH , LOW)
* Parameters   : u8 Copy_u8_port,u8 Copy_u8_pin
* Return value : u8
*****************************************************************************/
u8   DIO_u8_Get_Pin_Val(u8 Copy_u8_port, u8 Copy_u8_pin) ;
/*****************************************************************************
* Function Name: DIO_u8_Toogle_Pin_Val
* Purpose      : toogle pin val (HIGH , LOW)
* Parameters   : u8 Copy_u8_port,u8 Copy_u8_pin
* Return value : void
*****************************************************************************/
void DIO_u8_Toogle_Pin_Val(u8 Copy_u8_port, u8 Copy_u8_pin) ;
/*****************************************************************************
* Function Name: DIO_Vid_Set_Port_Dir
* Purpose      : set port dir (HIGH , LOW)
* Parameters   : u8 Copy_u8_port,u8 Copy_u8_pin
* Return value : void
*****************************************************************************/
void DIO_Vid_Set_Port_Dir (u8 Copy_u8_port, u8 Copy_u8_dir) ;
/*****************************************************************************
* Function Name: DIO_Vid_Set_Port_Val 
* Purpose      : set port value (HIGH , LOW)
* Parameters   : u8 Copy_u8_port,u8 Copy_u8_pin
* Return value : void
*****************************************************************************/
void  DIO_Vid_Set_Port_Val (u8 Copy_u8_port, u8 Copy_u8_val) ;
/***********************************************************************************************/


/*  Status --> Dir  */
#define OUTPUT 1
#define INPUT  0

/*  Status --> Val  */
#define HIGH 1
#define LOW  0

/*  port dir  */
#define PORT_OUTPUT 0xff
#define PORT_INPUT  0x00

/*  port val  */
#define PORT_HIGH 0xff
#define PORT_LOW  0x00

/*  ports  */
#define PORTA 0
#define PORTB 1
#define PORTC 2
#define PORTD 3

/* pins */
#define PIN0 0
#define PIN1 1
#define PIN2 2
#define PIN3 3
#define PIN4 4
#define PIN5 5
#define PIN6 6
#define PIN7 7



#endif /* DIO_INTERFACE_H_ */