#include "../../include/MCAL/PWM1_INT.h"
#include "../../include/MCAL/BIT_MATH.h"
#include "../../include/MCAL/DIO_INT.h"

void TIMER1_voidInit(void)
{
    /* Set PD5 (OC1A) and PD4 (OC1B) as outputs */
    DIO_voidSetPinDirection(PORTD, PIN5, OUTPUT);
    DIO_voidSetPinDirection(PORTD, PIN4, OUTPUT);

    /* Fast PWM 8-bit mode (WGM10 = 1, WGM11 = 0, WGM12 = 1, WGM13 = 0) */
    SET_BIT(TIMER_TCCR1A, TMR1_WGM10);
    CLEAR_BIT(TIMER_TCCR1A, TMR1_WGM11);
    SET_BIT(TIMER_TCCR1B, TMR1_WGM12);

    /* Non-inverted PWM on OC1A and OC1B */
    SET_BIT(TIMER_TCCR1A, TMR1_COM1A1);
    CLEAR_BIT(TIMER_TCCR1A, TMR1_COM1A0);

    SET_BIT(TIMER_TCCR1A, TMR1_COM1B1);
    CLEAR_BIT(TIMER_TCCR1A, TMR1_COM1B0);

    /* Set prescaler to 8 */
    TIMER_TCCR1B &= 0b11111000;
    TIMER_TCCR1B |= TIMER1_PRESCALER_8;

    /* Initialize duty cycles */
    TIMER_OCR1A = 0;   // EN2 (PD5)
    TIMER_OCR1B = 0;   // EN1 (PD4)
}

void TIMER1_setDutyCycleA(u8 New_DutyCycle)
{
    if (New_DutyCycle > 255) New_DutyCycle = 255;
    TIMER_OCR1A = New_DutyCycle;    // 8-bit used from low byte
}

void TIMER1_setDutyCycleB(u8 New_DutyCycle)
{
    if (New_DutyCycle > 255) New_DutyCycle = 255;
    TIMER_OCR1B = New_DutyCycle;
}

void TIMER1_startPWM(void)
{
    /* Start Timer1 with prescaler = 8 */
    TIMER_TCCR1B |= (1 << TMR1_CS11);
}

void TIMER1_stopPWM(void)
{
    /* Stop Timer1 by clearing the clock select bits */
    TIMER_TCCR1B &= ~((1 << TMR1_CS10) | (1 << TMR1_CS11));
}
