/*
 * LCD_interface.c
 *
 * Created: 8/29/2023 4:39:38 PM
 *  Author: youssef
 */ 

#include "MATH.h"
#include "STD_TYPE.h"
#include "LCD_interface.h"
#include "LCD_config.h"

#include "DIO_interface.h"
#define F_CPU 16000000UL
#include "util/delay.h"

static void SHDATA(u8 copy_u8_data){
	u8 loc_u8_in = 0;
	u8 loc_u8_pin[4] = {LCD_D4_PIN, LCD_D5_PIN, LCD_D6_PIN, LCD_D7_PIN};
	for(loc_u8_in = 0; loc_u8_in < 4; loc_u8_in++){
		DIO_Vid_Set_Pin_Val(LCD_DPORT, loc_u8_pin[loc_u8_in], GET_BIT(copy_u8_data,loc_u8_in));
	}
}

static void Send_Enable(void){
	DIO_Vid_Set_Pin_Val(LCD_CPORT, LCD_EN_PIN, HIGH);
	_delay_ms(2);
	DIO_Vid_Set_Pin_Val(LCD_CPORT, LCD_EN_PIN, LOW);
}

void LCD_Vid_Send_Command(u8 copy_u8_command){
	
	/* SET RS PIN IN LCD TO SEND COMMAND */
	DIO_Vid_Set_Pin_Val(LCD_CPORT, LCD_RS_PIN, LOW);
	
	/* SET RW PIN IN LCD TO WRITE COMMAND */
	DIO_Vid_Set_Pin_Val(LCD_CPORT, LCD_RW_PIN, LOW);
	
	#if LCD_MODE == FOUR_BIT_MODE
		/* SEND HSB COMMAND */
		SHDATA(copy_u8_command>>4);
		Send_Enable();
		/* LSB */
		SHDATA(copy_u8_command);
		Send_Enable();
	#elif LCD_MODE == EIGHT_BIT_MODE
		/* SEND COMMAND */
		DIO_Vid_Set_Port_Val(LCD_DPORT, copy_u8_command);
		/* SEND ENABLE PULSE */
		Send_Enable();
	#endif
}

void LCD_Vid_Send_Data(u8 copy_u8_data){
	
	/* SET RS PIN IN LCD TO SEND DATA */
	DIO_Vid_Set_Pin_Val(LCD_CPORT, LCD_RS_PIN, HIGH);
	
	/* SET RW PIN IN LCD TO WRITE DATA */
	DIO_Vid_Set_Pin_Val(LCD_CPORT, LCD_RW_PIN, LOW);
	
	#if LCD_MODE == FOUR_BIT_MODE
		/* SEND HSB COMMAND */
		SHDATA(copy_u8_data>>4);
		Send_Enable();
		/* LSB */
		SHDATA(copy_u8_data);
		Send_Enable();
	#elif LCD_MODE == EIGHT_BIT_MODE
		/* SEND COMMAND */
		DIO_Vid_Set_Port_Val(LCD_DPORT, copy_u8_data);
		/* SEND ENABLE PULSE */
		Send_Enable();
	#endif
}

void LCD_Vid_Init(void){
	
	_delay_ms(40);
	#if LCD_MODE == FOUR_BIT_MODE
		/* function set */
		SHDATA(0b0010);
		Send_Enable();
		SHDATA(0b0010);
		Send_Enable();
		SHDATA(0b1000);
		Send_Enable();
	#elif LCD_MODE == EIGHT_BIT_MODE
		/* function set */
		LCD_Vid_Send_Command(0x3C);
	#endif
	/* send on/off command */
	LCD_Vid_Send_Command(0x0C);
	/* send CLR command */
	LCD_Vid_Send_Command(0x01);
}

void LCD_Vid_Send_String(ch8 *add_u8_str){

	u8 loc_u8_in=0;
	while(add_u8_str[loc_u8_in] != '\0'){
		if(loc_u8_in==16){
			LCD_Vid_Send_Command(0xC0);
		}
		LCD_Vid_Send_Data(add_u8_str[loc_u8_in]);
		loc_u8_in++;
	}
}

void LCD_Vid_Send_Number(s32 copy_u32_data){
	u8 arr[10];
	u8 i=0;
	u8 j;
	if(copy_u32_data >= 0){
		do{
			arr[i] = copy_u32_data%10;
			copy_u32_data /= 10;
			i++;
		}while (copy_u32_data>0);
		i--;
		for(j=0; j<=i; j++){
			LCD_Vid_Send_Data( (arr[i-j] + 48) );
		}
	}
	else if(copy_u32_data < 0){
		LCD_Vid_Send_Data('-');
		copy_u32_data = 0 - copy_u32_data;
		do{
			arr[i] = copy_u32_data%10;
			copy_u32_data /= 10;
			i++;
		}while (copy_u32_data>0);
		i--;
		for(j=0; j<=i; j++){
			LCD_Vid_Send_Data( (arr[i-j] + 48) );
		}
	}
}

void LCD_Vid_GotoXY(u8 copy_u8_x, u8 copy_u8_y){
	u8 loc_u8_DDRAMaddress = 0;
	
	if(copy_u8_y == 0){
		loc_u8_DDRAMaddress = copy_u8_x;
	}
	else if (copy_u8_y == 1){
		loc_u8_DDRAMaddress = copy_u8_x + 0x40;
	}
	/* SET DDRAM */
	SET_BIT(loc_u8_DDRAMaddress,7);
	LCD_Vid_Send_Command(loc_u8_DDRAMaddress);
}

void LCD_Vid_Set_Custom_Char(u8 copy_u8_add, u8 copy_u8_charData[8]) {
	u8 i;
	/* SET THE ADDRESS OF THE CGRAM */
	LCD_Vid_Send_Command(0x40 + (copy_u8_add * 8));  
	
	/* SEND THE DATA OF THE CHAR */
	for (i = 0; i < 8; i++) {
		LCD_Vid_Send_Data(copy_u8_charData[i]);  
	}
}

void LCD_Vid_Send_Custom_Char(u8 copy_u8_x, u8 copy_u8_y, u8 copy_u8_add, u8 copy_u8_charData[8]) {
	u8 Loc_u8_in;
	u8 Loc_u8_CGRAMadd = copy_u8_add * 8;
	
	SET_BIT(Loc_u8_CGRAMadd, 6);
	CLR_BIT(Loc_u8_CGRAMadd, 7);
	
	/* SET THE ADDRESS OF THE CGRAM */
	LCD_Vid_Send_Command(Loc_u8_CGRAMadd);
	
	/* SEND THE DATA OF THE CHAR TO CGRAM */
	for (Loc_u8_in = 0; Loc_u8_in < 8; Loc_u8_in++) {
		LCD_Vid_Send_Data(copy_u8_charData[Loc_u8_in]);
	}
	
	/* BACK TO DDRAM TO DISPLAY DATA */
	LCD_Vid_GotoXY(copy_u8_x, copy_u8_y);
	LCD_Vid_Send_Data(copy_u8_add);
}



