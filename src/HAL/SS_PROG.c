# define F_CPU 16000000UL
#include <util/delay.h>

#include "../../include/MCAL/BIT_MATH.h"
#include "../../include/HAL/SS_INT.h"

void SS_init()
{
	DIO_setPinDir(SS_S0, DIO_OUTPUT);
	DIO_setPinDir(SS_S1, DIO_OUTPUT);
	DIO_setPinDir(SS_S2, DIO_OUTPUT);
	DIO_setPinDir(SS_S3, DIO_OUTPUT);
	
	DIO_setPinDir(SS_E1, DIO_OUTPUT);
	DIO_setPinDir(SS_E2, DIO_OUTPUT);
}

void SS_setEnable1()
{
	DIO_setPinValue(SS_E1, DIO_HIGH);
}
void SS_setEnable2()
{
	DIO_setPinValue(SS_E2, DIO_HIGH);
}

void SS_setDisable1()
{
	DIO_setPinValue(SS_E1, DIO_LOW);
}
void SS_setDisable2()
{
	DIO_setPinValue(SS_E2, DIO_LOW);
}


void SS_writeNum(u8 num)
{
	SS_setDisable1();
	SS_setDisable2();
	//write num
	//num/10
	u8 x =num/10;
	DIO_setPinValue(SS_S0, GET_BIT(x,0));
	DIO_setPinValue(SS_S1, GET_BIT(x,1));
	DIO_setPinValue(SS_S2, GET_BIT(x,2));
	DIO_setPinValue(SS_S3, GET_BIT(x,3));
	//enable1, disable2
	SS_setEnable1();
	SS_setDisable2();
	
	//delay 1 msec
	_delay_ms(1);
	
	SS_setDisable1();
	SS_setDisable2();
	
	//write num
	//num%10
	 x =num%10;
	 DIO_setPinValue(SS_S0, GET_BIT(x,0));
	 DIO_setPinValue(SS_S1, GET_BIT(x,1));
	 DIO_setPinValue(SS_S2, GET_BIT(x,2));
	 DIO_setPinValue(SS_S3, GET_BIT(x,3));
	//enable2, disable1
	SS_setEnable2();
	SS_setDisable1();
	
	//delay 1 msec
	_delay_ms(1);
	
}
