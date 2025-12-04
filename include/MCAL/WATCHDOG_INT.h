#ifndef WATCHDOG_INT_H_
#define WATCHDOG_INT_H_

#include "STD_TYPES.h"

/* Watchdog Timer Control Register */
#define WDTCR         *((volatile u8*)0x41)
#define WDTOE         4    /* Watchdog Timeout Enable */
#define WDE           3    /* Watchdog Enable */
#define WDP2          2    /* Watchdog Prescaler bits */
#define WDP1          1
#define WDP0          0

/* Watchdog Timeout Prescaler Options */
#define WDT_PRESCALER_16MS   0x00
#define WDT_PRESCALER_32MS   0x01
#define WDT_PRESCALER_64MS   0x02
#define WDT_PRESCALER_128MS  0x03
#define WDT_PRESCALER_256MS  0x04
#define WDT_PRESCALER_512MS  0x05
#define WDT_PRESCALER_1024MS 0x06

void WATCHDOG_init(u8 prescaler);
void WATCHDOG_start();
void WATCHDOG_reset();
void WATCHDOG_disable();

#endif /* WATCHDOG_INT_H_ */
