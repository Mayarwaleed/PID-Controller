/*
 * LCD_interface.h
 *
 * Created: 8/29/2023 4:38:45 PM
 *  Author: youssef
 */ 

#ifndef LCD_INTERFACE_H_
#define LCD_INTERFACE_H_
#include "STD_TYPE.h"
#define FOUR_BIT_MODE   0
#define EIGHT_BIT_MODE  1
#define LCD_MODE        FOUR_BIT_MODE

 /* COMMAND */
#define LCD_CLR         0x01
#define LCD_RETURN_HOME 0x02

/* ROWS */
#define ROW1 0
#define ROW2 1

/* COLUMNS */
#define COL1  0
#define COL2  1
#define COL3  2
#define COL4  3
#define COL5  4
#define COL6  5
#define COL7  6
#define COL8  7
#define COL9  8
#define COL10 9
#define COL11 10
#define COL12 11
#define COL13 12
#define COL14 13
#define COL15 14
#define COL16 15

/*****************************************************************************
* Function Name: LCD_Vid_Send_Command
* Purpose      : send command to lcd
* Parameters   : u8 copy_u8_command
* Return value : void
*****************************************************************************/
void LCD_Vid_Send_Command(u8 copy_u8_command);
/*****************************************************************************
* Function Name: LCD_Vid_Send_Data
* Purpose      : send data to lcd
* Parameters   : u8 copy_u8_data
* Return value : void
*****************************************************************************/
void LCD_Vid_Send_Data(u8 copy_u8_data);
/*****************************************************************************
* Function Name: LCD_Vid_Init
* Purpose      : initialize lcd
* Parameters   : void
* Return value : void
*****************************************************************************/
void LCD_Vid_Init(void);
/*****************************************************************************
* Function Name: void LCD_Vid_Send_String_Data
* Purpose      : Send String Data to LCD
* Parameters   : u8 *add_u32_str
* Return value : void
*****************************************************************************/
void LCD_Vid_Send_String(ch8 *add_u32_str);
/*****************************************************************************
* Function Name: void LCD_Vid_Send_String_Data
* Purpose      : Send Number Data to LCD
* Parameters   : u32 copy_u32_data
* Return value : void
*****************************************************************************/
void LCD_Vid_Send_Number(s32 copy_u32_data);
/*****************************************************************************
* Function Name: void LCD_Vid_GotoXY
* Purpose      : Set the cursor 
* Parameters   :u8 copy_u8_x, u8 copy_u8_y
* Return value : void
*****************************************************************************/
void LCD_Vid_GotoXY(u8 copy_u8_x, u8 copy_u8_y);
/*****************************************************************************
* Function Name: void LCD_Vid_Set_Custom_Char
* Purpose      : create your custom char in CGRAM
* Parameters   : u8 copy_u8_add, u8 copy_u8_charData[8]
* Return value : void
*****************************************************************************/
void LCD_Vid_Set_Custom_Char(u8 copy_u8_add, u8 copy_u8_charData[8]);
void LCD_Vid_Send_Custom_Char(u8 copy_u8_x, u8 copy_u8_y, u8 copy_u8_add, u8 copy_u8_charData[8]);
/*******************************************************************************************/

#endif /* LCD_INTERFACE_H_ */