#include "../../include/HAL/KPD_INT.h"

void KPD_init()
{
	DIO_setPinDir(KPD_ROW0 ,DIO_INPUT);
	DIO_setPinDir(KPD_ROW1 ,DIO_INPUT);
	DIO_setPinDir(KPD_ROW2 ,DIO_INPUT);
	DIO_setPinDir(KPD_ROW3 ,DIO_INPUT);
	
	DIO_setPullUp(KPD_ROW0);
	DIO_setPullUp(KPD_ROW1);
	DIO_setPullUp(KPD_ROW2);
	DIO_setPullUp(KPD_ROW3);
	
	DIO_setPinDir(KPD_COL0, DIO_OUTPUT);
	DIO_setPinDir(KPD_COL1, DIO_OUTPUT);
	DIO_setPinDir(KPD_COL2, DIO_OUTPUT);
	DIO_setPinDir(KPD_COL3, DIO_OUTPUT);
	
}



u8 KPD_read()
{
	DIO_setPinValue(KPD_COL0,DIO_LOW);
	DIO_setPinValue(KPD_COL1,DIO_HIGH);
	DIO_setPinValue(KPD_COL2,DIO_HIGH);
	DIO_setPinValue(KPD_COL3,DIO_HIGH);
	
	if(DIO_readPinValue(KPD_ROW0) == DIO_LOW)
	{
			return KPD_ROW0COL0;
	}
	else if(DIO_readPinValue(KPD_ROW1) == DIO_LOW)
	{
		return KPD_ROW1COL0;
	}
	else if(DIO_readPinValue(KPD_ROW2) == DIO_LOW)
	{
		return KPD_ROW2COL0;
	}
	else if(DIO_readPinValue(KPD_ROW3) == DIO_LOW)
	{
		return KPD_ROW3COL0;
	}
	
	
	///////////////////////////////////////////
	DIO_setPinValue(KPD_COL0, DIO_HIGH);
	DIO_setPinValue(KPD_COL1, DIO_LOW);
	DIO_setPinValue(KPD_COL2, DIO_HIGH);
	DIO_setPinValue(KPD_COL3, DIO_HIGH);
	
	if(DIO_readPinValue(KPD_ROW0) == DIO_LOW)
	{
		return KPD_ROW0COL1;
	}
	else if(DIO_readPinValue(KPD_ROW1) == DIO_LOW)
	{
		return KPD_ROW1COL1;
	}
	else if(DIO_readPinValue(KPD_ROW2) == DIO_LOW)
	{
		return KPD_ROW2COL1;
	}
	else if(DIO_readPinValue(KPD_ROW3) == DIO_LOW)
	{
		return KPD_ROW3COL1;
	}
	//////////////////////////////////////////////////
	
	DIO_setPinValue(KPD_COL0, DIO_HIGH);
	DIO_setPinValue(KPD_COL1, DIO_HIGH);
	DIO_setPinValue(KPD_COL2, DIO_LOW);
	DIO_setPinValue(KPD_COL3, DIO_HIGH);
	
	if(DIO_readPinValue(KPD_ROW0) == DIO_LOW)
	{
		return KPD_ROW0COL2;
	}
	else if(DIO_readPinValue(KPD_ROW1) == DIO_LOW)
	{
		return KPD_ROW1COL2;
	}
	else if(DIO_readPinValue(KPD_ROW2) == DIO_LOW)
	{
		return KPD_ROW2COL2;
	}
	else if(DIO_readPinValue(KPD_ROW3) == DIO_LOW)
	{
		return KPD_ROW3COL2;
	}
	///////////////////////////////////////////////////////////
	
	DIO_setPinValue(KPD_COL0, DIO_HIGH);
	DIO_setPinValue(KPD_COL1, DIO_HIGH);
	DIO_setPinValue(KPD_COL2, DIO_HIGH);
	DIO_setPinValue(KPD_COL3, DIO_LOW);
	
	if(DIO_readPinValue(KPD_ROW0) == DIO_LOW)
	{
		return KPD_ROW0COL3;
	}
	else if(DIO_readPinValue(KPD_ROW1) == DIO_LOW)
	{
		return KPD_ROW1COL3;
	}
	else if(DIO_readPinValue(KPD_ROW2) == DIO_LOW)
	{
		return KPD_ROW2COL3;
	}
	else if(DIO_readPinValue(KPD_ROW3) == DIO_LOW)
	{
		return KPD_ROW3COL3;
	}
	
	return KPD_NOT_PRESSED;
}