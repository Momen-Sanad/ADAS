#include "../../include/MCAL/BIT_MATH.h"
#include "../../include/MCAL/GI_INT.h"

void GI_enable()
{
	SET_BIT(SREG, I);
}

void GI_disable()
{
	CLEAR_BIT(SREG, I);
}