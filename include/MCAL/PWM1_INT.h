#ifndef PWM1_INT_H_
#define PWM1_INT_H_

/* Bit definitions for Timer1 registers */
#define TMR1_WGM10   0   // WGM10 bit in TCCR1A
#define TMR1_WGM11   1   // WGM11 bit in TCCR1A
#define TMR1_COM1B0  4   // COM1B0 bit in TCCR1A
#define TMR1_COM1B1  5   // COM1B1 bit in TCCR1A
#define TMR1_COM1A0  6   // COM1A0 bit in TCCR1A
#define TMR1_COM1A1  7   // COM1A1 bit in TCCR1A

#define TMR1_WGM12   3   // WGM12 bit in TCCR1B
#define TMR1_CS10    0   // Clock select bit 0 (No prescaler)
#define TMR1_CS11    1   // Clock select bit 1 (Prescaler 8)

#define TIMER_TCCR1A   (*(volatile u8*)0x4F)
#define TIMER_TCCR1B   (*(volatile u8*)0x4E)
#define TIMER_OCR1A    (*(volatile u16*)0x4A)
#define TIMER_OCR1B    (*(volatile u16*)0x48)
#define TIMER_TCNT1    (*(volatile u16*)0x4C)

#define TIMER1_PRESCALER_8   0x02

void TIMER1_voidInit(void);
void TIMER1_setDutyCycleA(u8 New_DutyCycle);
void TIMER1_setDutyCycleB(u8 New_DutyCycle);
void TIMER1_startPWM(void);
void TIMER1_stopPWM(void);

#endif /* PWM1_INT_H_ */
