#ifndef L298N_MOTOR_H_
#define L298N_MOTOR_H_

#include "../MCAL/STD_TYPES.h"
#include "../MCAL/DIO_INT.h"

// Definitions for your PCB pinout
#define EN1   DIO_PIND4   // PD4 -> H_en1
#define EN2   DIO_PIND5   // PD5 -> H_en2
#define A1    DIO_PINC3   // PC3 -> H_A1
#define A2    DIO_PINC4   // PC4 -> H_A2
#define A3    DIO_PINC5   // PC5 -> H_A3
#define A4    DIO_PINC6   // PC6 -> H_A4

// Function Prototypes
void L298N_Init(void);
void L298N_SetDutyCycle(u8 motor, u8 dutyCycle);
void L298N_StartPWM(void);
void L298N_StopPWM(void);
void L298N_Clockwise(void);
void L298N_CounterClockwise(void);
void L298N_StopMotor(void);

#endif /* L298N_MOTOR_H_ */     