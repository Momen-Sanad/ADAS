#ifndef EXT_INT_H_
#define EXT_INT_H_

#include "STD_TYPES.h"


#define MCUCR (*(volatile u8*)0x55)
#define ISC00	0
#define ISC01   1
#define ISC10   2
#define ISC11   3


#define MCUCSR (*(volatile u8*)0x54)
#define ISC2  6


#define GICR (*(volatile u8*)0x5B)
#define INT2   5
#define INT0   6
#define INT1   7

#define EXT_FALLING  0
#define EXT_RISING	 1
#define EXT_ANY_LOGICAL 2



void EXT_initInt0(u8 state);
void EXT_callbackInt0(void (*func)());


#endif /* EXT_INT_H_ */