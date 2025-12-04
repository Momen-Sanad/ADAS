#ifndef LCD_INT_H_
#define LCD_INT_H_


#include "../MCAL/STD_TYPES.h"
#include "../MCAL/DIO_INT.h"

//config
#define LCD_D4  DIO_PINA4
#define LCD_D5  DIO_PINA5
#define LCD_D6  DIO_PINA6
#define LCD_D7  DIO_PINA7

#define LCD_RS  DIO_PINB1
#define LCD_RW  DIO_PINB2
#define LCD_E	DIO_PINB3

void LCD_init();
void LCD_sendData(u8 data);
void LCD_writeChar(u8 data);
void LCD_writeCmd(u8 cmd);
void LCD_writeStr(u8* str);
void LCD_goto(u8 row, u8 col);
void LCD_clearDisplay();
void LCD_writeNum(s32 num);
void LCD_writeFloatNum(f32 num);



#endif /* LCD_INT_H_ */