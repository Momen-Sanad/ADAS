#include "../../include/MCAL/BIT_MATH.h"
#include "../../include/MCAL/TIMER0_INT.h"

void TIMER0_initNormal()
{
	//select normal mode
	CLEAR_BIT(TCCR0,WGM00);
	CLEAR_BIT(TCCR0,WGM01);
	
	//enable interrupt
	SET_BIT(TIMSK,TOIE0);
}


void TIMER0_start(u8 prescaler)
{
	if(prescaler == TIMER0_DIV_1)
	{
		SET_BIT(TCCR0,CS00);
		CLEAR_BIT(TCCR0,CS01);
		CLEAR_BIT(TCCR0,CS02);
	}
	else if(prescaler == TIMER0_DIV_8)
	{
		CLEAR_BIT(TCCR0,CS00);
		SET_BIT(TCCR0,CS01);
		CLEAR_BIT(TCCR0,CS02);
	}
	else if(prescaler == TIMER0_DIV_64)
	{
		SET_BIT(TCCR0,CS00);
		SET_BIT(TCCR0,CS01);
		CLEAR_BIT(TCCR0,CS02);
	}
	
}

void TIMER0_stop()
{
	CLEAR_BIT(TCCR0,CS00);
	CLEAR_BIT(TCCR0,CS01);
	CLEAR_BIT(TCCR0,CS02);
}

u8 TIMER0_GetTicks()
{
	return TCNT0;
}

void TIMER0_setPreload(u8 preload)
{
	TCNT0 = preload;
}



void (*ptrFuncOv)() ;

void TIMER0_callbackOv(void (*func)())
{
	ptrFuncOv = func;
}


//ISR
void __vector_11() __attribute__((signal));
void __vector_11()
{
	//toggle();	
	ptrFuncOv();
}