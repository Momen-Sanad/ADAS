#include "../../include/MCAL/BIT_MATH.h"
#include "../../include/MCAL/EXT_INT.h"

void EXT_initInt0(u8 state)
{
	if(state == EXT_RISING)
	{
		SET_BIT(MCUCR,ISC00);
		SET_BIT(MCUCR,ISC01);
	}
	else if(state == EXT_FALLING)
	{
		CLEAR_BIT(MCUCR,ISC00);
		SET_BIT(MCUCR,ISC01);
	}
	else if(state == EXT_ANY_LOGICAL)
	{
		SET_BIT(MCUCR,ISC00);
		CLEAR_BIT(MCUCR,ISC01);
	}
	
	//enable
	SET_BIT(GICR,INT0);
}

void (*ptrFuncInt0)() ; 

void EXT_callbackInt0(void (*func)())
{
	ptrFuncInt0 = func;
}


//ISR
void __vector_1() __attribute__((signal));
void __vector_1()
{
	//DIO_togglePinValue(DIO_PINC2);
	ptrFuncInt0();
}