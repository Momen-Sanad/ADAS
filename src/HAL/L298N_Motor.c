#include "../../include/HAL/L298N_Motor.h"
#include "../../include/MCAL/DIO_INT.h"

// Initialize motor control pins
void L298N_Init(void)
{
    // Set the output pins for control and enable
    DIO_setPinDir(EN1, DIO_OUTPUT);  // Enable 1 (H_en1)
    DIO_setPinDir(EN2, DIO_OUTPUT);  // Enable 2 (H_en2)
    DIO_setPinDir(A1,  DIO_OUTPUT);   // Control pin 1 (H_A1)
    DIO_setPinDir(A2,  DIO_OUTPUT);   // Control pin 2 (H_A2)
    DIO_setPinDir(A3,  DIO_OUTPUT);   // Control pin 3 (H_A3)
    DIO_setPinDir(A4,  DIO_OUTPUT);   // Control pin 4 (H_A4)
}

// Set the duty cycle for the motor (0-255)
void L298N_SetDutyCycle(u8 motor, u8 dutyCycle)
{
    if (motor == 1) {
        // Set motor A duty cycle (via EN1)
        DIO_setPinValue(EN1, dutyCycle > 0 ? DIO_HIGH : DIO_LOW);
    }
    else if (motor == 2) {
        // Set motor B duty cycle (via EN2)
        DIO_setPinValue(EN2, dutyCycle > 0 ? DIO_HIGH : DIO_LOW);
    }
}

// Start PWM on the enable pins
void L298N_StartPWM(void)
{
    // Enable PWM signal (set EN1 and EN2 high)
    DIO_setPinValue(EN1, DIO_HIGH);  // Motor A enabled
    DIO_setPinValue(EN2, DIO_HIGH);  // Motor B enabled
}

// Stop PWM by disabling the enable pins
void L298N_StopPWM(void)
{
    // Disable PWM signal (set EN1 and EN2 low)
    DIO_setPinValue(EN1, DIO_LOW);   // Motor A disabled
    DIO_setPinValue(EN2, DIO_LOW);   // Motor B disabled
}

// Rotate Motor A clockwise
void L298N_Clockwise(void)
{
    // Set the control pins for Motor A (Clockwise)
    DIO_setPinValue(A1, DIO_HIGH);  // H_A1 high
    DIO_setPinValue(A2, DIO_LOW);   // H_A2 low
    // Set control pins for Motor B (Clockwise)
    DIO_setPinValue(A3, DIO_HIGH);  // H_A3 high
    DIO_setPinValue(A4, DIO_LOW);   // H_A4 low
}

// Rotate Motor A counter-clockwise
void L298N_CounterClockwise(void)
{
    // Set the control pins for Motor A (Counter-clockwise)
    DIO_setPinValue(A1, DIO_LOW);   // H_A1 low
    DIO_setPinValue(A2, DIO_HIGH);  // H_A2 high
    // Set control pins for Motor B (Counter-clockwise)
    DIO_setPinValue(A3, DIO_LOW);   // H_A3 low
    DIO_setPinValue(A4, DIO_HIGH);  // H_A4 high
}

// Stop both motors
void L298N_StopMotor(void)
{
    // Stop motor A and motor B
    DIO_setPinValue(A1, DIO_LOW);
    DIO_setPinValue(A2, DIO_LOW);
    DIO_setPinValue(A3, DIO_LOW);
    DIO_setPinValue(A4, DIO_LOW);
}
