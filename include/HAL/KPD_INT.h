#ifndef KPD_INT_H_
#define KPD_INT_H_

#include "../MCAL/STD_TYPES.h"
#include "../MCAL/DIO_INT.h"


#define  KPD_ROW0  DIO_PINB4
#define  KPD_ROW1  DIO_PINB5
#define  KPD_ROW2  DIO_PINB6
#define  KPD_ROW3  DIO_PINB7

#define  KPD_COL0  DIO_PINC3
#define  KPD_COL1  DIO_PINC4
#define  KPD_COL2  DIO_PINC5
#define  KPD_COL3  DIO_PINC6


#define KPD_ROW0COL0    '7'
#define KPD_ROW1COL0    '4'
#define KPD_ROW2COL0    '1'
#define KPD_ROW3COL0    'c'


#define KPD_ROW0COL1    '8'
#define KPD_ROW1COL1    '5'
#define KPD_ROW2COL1    '2'
#define KPD_ROW3COL1    '0'


#define KPD_ROW0COL2    '9'
#define KPD_ROW1COL2    '6'
#define KPD_ROW2COL2    '3'
#define KPD_ROW3COL2    '='

#define KPD_ROW0COL3    '/'
#define KPD_ROW1COL3    '*'
#define KPD_ROW2COL3    '-'
#define KPD_ROW3COL3    '+'



#define KPD_NOT_PRESSED	  'x'

void KPD_init();
u8 KPD_read();




#endif /* KPD_INT_H_ */