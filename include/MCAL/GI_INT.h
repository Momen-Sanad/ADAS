#ifndef GI_INT_H_
#define GI_INT_H_

#include "STD_TYPES.h"

#define SREG  (*(volatile u8*)0x5F)
#define I	7

void GI_enable();
void GI_disable();



#endif /* GI_INT_H_ */