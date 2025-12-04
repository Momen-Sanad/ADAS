#include "../../include/MCAL/DIO_INT.h"
#include "../../include/MCAL/BIT_MATH.h"

void DIO_setPinDir(u8 pinNum, u8 state)
{
	if(pinNum>=0 && pinNum<=7)
	{
		if(state == DIO_OUTPUT)
		{
			SET_BIT(DDRA, pinNum);//High
		}
		else if(state == DIO_INPUT)
		{
			CLEAR_BIT(DDRA, pinNum);//Low
		}
	}
	else if(pinNum>=8 && pinNum<=15)
	{
		pinNum = pinNum-8;
		if(state == DIO_OUTPUT)
		{
			SET_BIT(DDRB, pinNum);//High
		}
		else if(state == DIO_INPUT)
		{
			CLEAR_BIT(DDRB, pinNum);//Low
		}
	}
	else if(pinNum>=16 && pinNum<=23)
	{
		pinNum = pinNum-16;
		if(state == DIO_OUTPUT)
		{
			SET_BIT(DDRC, pinNum);//High
		}
		else if(state == DIO_LOW)
		{
			CLEAR_BIT(DDRC, pinNum);//Low
		}
	}
	else if(pinNum>=24 && pinNum<=31)
	{
		pinNum = pinNum-24;
		if(state == DIO_OUTPUT)
		{
			SET_BIT(DDRD, pinNum);//High
		}
		else if(state == DIO_INPUT)
		{
			CLEAR_BIT(DDRD, pinNum);//Low
		}
	}
}

void DIO_setPinValue(u8 pinNum, u8 level)
{
	if(pinNum>=0 && pinNum<=7)
	{
		if(level == DIO_HIGH)
		{
			SET_BIT(PORTA, pinNum);//High
		}
		else if(level == DIO_LOW)
		{
			CLEAR_BIT(PORTA, pinNum);//Low
		}
	}
	else if(pinNum>=8 && pinNum<=15)
	{
		pinNum = pinNum-8;
		if(level == DIO_HIGH)
		{
			SET_BIT(PORTB, pinNum);//High
		}
		else if(level == DIO_LOW)
		{
			CLEAR_BIT(PORTB, pinNum);//Low
		}
	}
	else if(pinNum>=16 && pinNum<=23)
	{
		pinNum = pinNum-16;
		if(level == DIO_HIGH)
		{
			SET_BIT(PORTC, pinNum);//High
		}
		else if(level == DIO_LOW)
		{
			CLEAR_BIT(PORTC, pinNum);//Low
		}
	}
	else if(pinNum>=24 && pinNum<=31)
	{
		pinNum = pinNum-24;
		if(level == DIO_HIGH)
		{
			SET_BIT(PORTD, pinNum);//High
		}
		else if(level == DIO_LOW)
		{
			CLEAR_BIT(PORTD, pinNum);//Low
		}
	}
	
}


u8 DIO_readPinValue(u8 pinNum)
{
	if(pinNum>=0 && pinNum<=7)
	{
		return GET_BIT(PINA, pinNum);
	}
	else if(pinNum>=8 && pinNum<=15)
	{
		return GET_BIT(PINB, pinNum-8);
	}
	else if(pinNum>=16 && pinNum<=23)
	{
		return GET_BIT(PINC, pinNum-16);
	}
	else if(pinNum>=24 && pinNum<=31)
	{
		return GET_BIT(PIND, pinNum-24);
	}
}


void DIO_togglePinValue(u8 pinNum)
{
	//u8 level = DIO_readPinValue(pinNum);
	//level = !level;
	//DIO_setPinValue(pinNum, level);
	
	DIO_setPinValue(pinNum, !DIO_readPinValue(pinNum));
}


void DIO_setPullUp(u8 pinNum)
{
	DIO_setPinValue(pinNum, DIO_HIGH);
}