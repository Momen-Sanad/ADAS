# define F_CPU 16000000UL
#include <util/delay.h>

#include "../../include/MCAL/BIT_MATH.h"
#include "../../include/HAL/LCD_INT.h"


void LCD_init()
{
    // Set all control and data pins to output
    DIO_setPinDir(LCD_D4, DIO_OUTPUT);
    DIO_setPinDir(LCD_D5, DIO_OUTPUT);
    DIO_setPinDir(LCD_D6, DIO_OUTPUT);
    DIO_setPinDir(LCD_D7, DIO_OUTPUT);
    DIO_setPinDir(LCD_RS, DIO_OUTPUT);
    DIO_setPinDir(LCD_RW, DIO_OUTPUT);
    DIO_setPinDir(LCD_E, DIO_OUTPUT);
    
    _delay_ms(40);  // A delay for LCD to power up
    LCD_writeCmd(0b00000010);  // Set cursor position
    LCD_writeCmd(0b00101000);  // 2-line, 4-bit mode
    _delay_ms(1);
    LCD_writeCmd(0b00001100);  // Display ON, Cursor OFF
    _delay_ms(1);
    LCD_writeCmd(0b00000001);  // Clear display
    _delay_ms(2);
    LCD_writeCmd(0b00000110);  // Auto-increment cursor
}




void LCD_sendData(u8 data)
{
    // Set RW pin to write mode (0)
    DIO_setPinValue(LCD_RW, DIO_LOW);
    
    // Send the higher nibble (4 bits)
    DIO_setPinValue(LCD_D4, GET_BIT(data, 4));
    DIO_setPinValue(LCD_D5, GET_BIT(data, 5));
    DIO_setPinValue(LCD_D6, GET_BIT(data, 6));
    DIO_setPinValue(LCD_D7, GET_BIT(data, 7));
    
    // Pulse the enable pin to latch data into the LCD (high -> low)
    DIO_setPinValue(LCD_E, DIO_HIGH);
    _delay_us(1);  // Short delay to allow LCD to register the data
    DIO_setPinValue(LCD_E, DIO_LOW);
    _delay_us(1);  // Short delay after E is low
    
    // Send the lower nibble (4 bits)
    DIO_setPinValue(LCD_D4, GET_BIT(data, 0));
    DIO_setPinValue(LCD_D5, GET_BIT(data, 1));
    DIO_setPinValue(LCD_D6, GET_BIT(data, 2));
    DIO_setPinValue(LCD_D7, GET_BIT(data, 3));
    
    // Pulse the enable pin again to latch the lower nibble
    DIO_setPinValue(LCD_E, DIO_HIGH);
    _delay_us(1);  // Short delay to allow LCD to register the data
    DIO_setPinValue(LCD_E, DIO_LOW);
    _delay_us(1);  // Short delay after E is low
}




void LCD_writeCmd(u8 cmd)
{
    // Set RS to 0 for command mode
    DIO_setPinValue(LCD_RS, DIO_LOW);
    // Send the command (using 4-bit mode)
    LCD_sendData(cmd);
    // Add a small delay after sending command
    _delay_ms(2);  // Increase if needed for stability
}



void LCD_writeStr(u8* str)
{
	u8 i =0;
	while(str[i] != '\0')
	{
		LCD_writeChar(str[i]);
		i++;
	}
}

void LCD_clearDisplay()
{
	LCD_writeCmd(0b00000001);
	_delay_ms(10);
}

void LCD_goto(u8 row, u8 col)
{
	if(col <16)
	{
		if(row == 1)
		{
			LCD_writeCmd(0b10000000 + col);
		}
		else if(row == 2)
		{
			LCD_writeCmd(0b10000000 + col + 0x40);
		}
	}
	
	
}


void LCD_writeNum(s32 num)
{
	if(num < 0)
	{
		LCD_writeChar('-');	
		num = num*-1;
	}
	u8 arr[10];
	s8 i=0;
	while(num > 0)
	{
		u8 rem = num%10;
		//LCD_writeChar(rem+48);
		arr[i] = (rem+48);
		i++;
		num = num/10;
	}
	
	for(i=i-1; i>=0; i--)
	{
		LCD_writeChar(arr[i]);
	}
}



void LCD_writeChar(u8 ch)
{
    // Set RS to 1 for data mode
    DIO_setPinValue(LCD_RS, DIO_HIGH);
    // Send the character (using 4-bit mode)
    LCD_sendData(ch);
    // Add a small delay after sending character
    _delay_ms(2);  // Increase if needed for stability
}

void LCD_writeFloatNum(f32 num)
{
	s32 intPart = (s32)num;
	LCD_writeNum(intPart);
	LCD_writeChar('.');
	if(num < 0)
	{
		num = num *-1;
		intPart = intPart *-1;
	}
	u32 floatPart = (num - intPart)*1000;
	LCD_writeNum(floatPart);
}