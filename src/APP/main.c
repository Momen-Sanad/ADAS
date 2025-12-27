#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#define TWO_PI 6.28318530

// Motor pins
#define ENA     PB1  // D9 (OC1A)
#define ENB     PB2  // D10 (OC1B)

#define IN1     PD3  // D3
#define IN2     PD4  // D4
#define IN3     PD5  // D5
#define IN4     PD6  // D6

// Ultrasonic pins
#define TRIG_PIN PB4 // D12
#define ECHO_PIN PB3 // D11

// PWM generator parameters
const unsigned long pwmCycleMs = 2000UL;
const uint8_t pwmMin = 0;
const uint8_t pwmMax = 200;
const unsigned long sampleIntervalMs = 10UL;

/* Runtime-adjustable speed (0..pwmMax). Default to pwmMax */
volatile uint8_t currentPwmMax = pwmMax;

/* MAX_SPEED_M_S: linear speed (m/s) at currentPwmMax == pwmMax
   DECEL_M_S2: braking deceleration magnitude (m/s^2) when braking */
const float MAX_SPEED_M_S = 0.60f;   // adjust to measured top speed (m/s)
const float DECEL_M_S2    = 1.50f;   // adjust to measured braking deceleration (m/s^2)
#define UTURN_DURATION_MS 1100UL     // Duration (ms) needed for ~180° rotation (tune it too)

/* Track current motion: 0=stop, 1=forward, -1=backward, 2=turning */
volatile int8_t currentMotion = 0;

/* LCD Pin Definitions (4-bit) */
// RS = D2 = PD2
// E  = D7 = PD7
#define LCD_RS_PIN    PD2
#define LCD_E_PIN     PD7

// Data pins:
// D4 = D8  = PB0
// D5 = D13 = PB5
// D6 = A2  = PC2
// D7 = A3  = PC3
#define LCD_D4 PB0
#define LCD_D5 PB5
#define LCD_D6 PC2
#define LCD_D7 PC3

/* --- Forward declarations --- */
void motor_init(void);
uint8_t generateSinePWM(unsigned long tMs, unsigned long cycleMs, uint8_t minVal, uint8_t maxVal, float phase);
void runWithGeneratedPWM(unsigned long durationMs, bool Aforward, bool Bforward, float phaseShiftB);

void ultrasonic_init(void);
float measure_distance(void);

void init_usart(void);
void usart_transmit(unsigned char data);
void usart_print_string(const char* str);
void usart_print_float(float value);
int usart_receive_nonblocking(void);
void usart_flush_input(void);
void floatToString(float value, char* str, int precision);

/* --------------------------------------------------------------------- */
/* LCD code (4-bit HD44780)                                              */
/* --------------------------------------------------------------------- */

static void lcd_pulse_enable(void) {
    PORTD |= (1 << LCD_E_PIN);
    _delay_us(1);
    PORTD &= ~(1 << LCD_E_PIN);
    _delay_us(50);
}

static void lcd_write_nibble(uint8_t nibble) {
    // Clear data pins
    PORTB &= ~((1 << LCD_D4) | (1 << LCD_D5));
    PORTC &= ~((1 << LCD_D6) | (1 << LCD_D7));
    
    // Set according to nibble
    if (nibble & 0x01) PORTB |= (1 << LCD_D4);
    if (nibble & 0x02) PORTB |= (1 << LCD_D5);
    if (nibble & 0x04) PORTC |= (1 << LCD_D6);
    if (nibble & 0x08) PORTC |= (1 << LCD_D7);

    lcd_pulse_enable();
}

static void lcd_send_byte(uint8_t value, bool is_data) {
    if (is_data) PORTD |= (1 << LCD_RS_PIN);
    else         PORTD &= ~(1 << LCD_RS_PIN);

    uint8_t upper = (value >> 4) & 0x0F;
    uint8_t lower = value & 0x0F;
    lcd_write_nibble(upper);
    lcd_write_nibble(lower);

    _delay_us(50);
}

void send_command(uint8_t cmd) {
    lcd_send_byte(cmd, false);
    if (cmd == 0x01 || cmd == 0x02) _delay_ms(2);
}

void SendData(char data) {
    lcd_send_byte((uint8_t)data, true);
}

void send_string(const char *str) {
    while (*str) SendData(*str++);
}

void lcd_init(void) {
    // Configure direction registers
    DDRD |= (1 << LCD_RS_PIN) | (1 << LCD_E_PIN);
    DDRB |= (1 << LCD_D4) | (1 << LCD_D5);
    DDRC |= (1 << LCD_D6) | (1 << LCD_D7);

    PORTD &= ~((1 << LCD_RS_PIN) | (1 << LCD_E_PIN));
    PORTB &= ~((1 << LCD_D4) | (1 << LCD_D5));
    PORTC &= ~((1 << LCD_D6) | (1 << LCD_D7));

    _delay_ms(40);

    lcd_write_nibble(0x03);
    _delay_ms(5);
    lcd_write_nibble(0x03);
    _delay_us(150);
    lcd_write_nibble(0x03);
    _delay_us(150);
    lcd_write_nibble(0x02);
    _delay_us(150);

    send_command(0x28);
    send_command(0x0C);
    send_command(0x01);
    _delay_ms(2);
    send_command(0x06);
}

/* --------------------------------------------------------------------- */
/* Motor / PWM code                                                      */
/* --------------------------------------------------------------------- */

void motor_init(void) {
    DDRD |= (1 << IN1) | (1 << IN2) | (1 << IN3) | (1 << IN4);
    DDRB |= (1 << ENA) | (1 << ENB);

    OCR1A = 0;
    OCR1B = 0;

    // 8-bit Fast PWM (WGM10 + WGM12), non-inverting on OC1A/OC1B, prescaler=8 (CS11)
    TCCR1A = (1 << WGM10) | (1 << COM1A1) | (1 << COM1B1);
    TCCR1B = (1 << WGM12) | (1 << CS11);
}

/* generateSinePWM now uses currentPwmMax (runtime) instead of the compile-time pwmMax.
   Signature unchanged to preserve compatibility with calls. */
uint8_t generateSinePWM(unsigned long tMs, unsigned long cycleMs, uint8_t minVal, uint8_t maxVal, float phase) {
    (void)maxVal; // keep parameter for compatibility; we use currentPwmMax instead
    if (cycleMs == 0) return currentPwmMax;
    unsigned long t = tMs % cycleMs;
    float phaseT = (float)t / (float)cycleMs;
    phaseT += phase;
    phaseT = phaseT - floorf(phaseT);
    double angle = TWO_PI * (double)phaseT;
    double s = sin(angle);
    float normalized = (float)((s * 0.5) + 0.5);
    float pwmF = minVal + normalized * (float)(currentPwmMax - minVal);
    if (pwmF < 0.0f) pwmF = 0.0f;
    if (pwmF > 255.0f) pwmF = 255.0f;
    return (uint8_t)(pwmF + 0.5f);
}

void runWithGeneratedPWM(unsigned long durationMs, bool Aforward, bool Bforward, float phaseShiftB) {
    // set motion tracking
    if (Aforward && Bforward) currentMotion = 1;
    else if (!Aforward && !Bforward) currentMotion = -1;
    else currentMotion = 2; // turning

    if (Aforward) {
        PORTD |=  (1 << IN1);
        PORTD &= ~(1 << IN2);
    } else {
        PORTD &= ~(1 << IN1);
        PORTD |=  (1 << IN2);
    }

    if (Bforward) {
        PORTD |=  (1 << IN3);
        PORTD &= ~(1 << IN4);
    } else {
        PORTD &= ~(1 << IN3);
        PORTD |=  (1 << IN4);
    }

    unsigned long elapsed = 0;
    while (elapsed < durationMs) {
        uint8_t pwmA = generateSinePWM(elapsed, pwmCycleMs, pwmMin, pwmMax, 0.0f);
        uint8_t pwmB = generateSinePWM(elapsed, pwmCycleMs, pwmMin, pwmMax, phaseShiftB);

        OCR1A = pwmA;
        OCR1B = pwmB;

        _delay_ms(sampleIntervalMs);
        elapsed += sampleIntervalMs;
    }

    OCR1A = 0;
    OCR1B = 0;

    // after stopping
    currentMotion = 0;
}

/* --------------------------------------------------------------------- */
/* Ultrasonic code (TRIG on D12 / ECHO on D11)                           */
/* --------------------------------------------------------------------- */

void ultrasonic_init(void) {
    DDRB |=  (1 << TRIG_PIN);
    DDRB &= ~(1 << ECHO_PIN);
    PORTB &= ~(1 << TRIG_PIN);
    PORTB &= ~(1 << ECHO_PIN);
}

float measure_distance(void) {
    uint32_t duration = 0;
    uint32_t timeout;

    PORTB &= ~(1 << TRIG_PIN);
    _delay_us(2);
    PORTB |=  (1 << TRIG_PIN);
    _delay_us(10);
    PORTB &= ~(1 << TRIG_PIN);

    timeout = 30000UL;
    while (!(PINB & (1 << ECHO_PIN)) && timeout--) { _delay_us(1); }
    if (timeout == 0) return 100.0f;

    duration = 0;
    timeout = 30000UL;
    while ((PINB & (1 << ECHO_PIN)) && timeout--) { _delay_us(1); duration++; }
    if (timeout == 0) return 100.0f;

    float dist = (duration * 0.0343f) / 2.0f;
    if (dist > 100.0f) dist = 100.0f;
    return dist;
}

/* --------------------------------------------------------------------- */
/* USART functions (TX + RX)                                             */
/* --------------------------------------------------------------------- */

void init_usart(void) {
    // PD1 is TX (we keep it as output). PD0 is RX (input by default).
    DDRD |= (1 << PD1);

    // Baud 9600 (assuming 16MHz): UBRR0 = 103
    UBRR0H = 0;
    UBRR0L = 103;

    // Enable TX and RX
    UCSR0B = (1 << TXEN0) | (1 << RXEN0);

    // Asynchronous, 8-bit, no parity
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void usart_transmit(unsigned char data) {
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = data;
}

void usart_print_string(const char* str) {
    while (*str) usart_transmit(*str++);
}

void usart_print_float(float value) {
    if (value < 0.0f) {
        usart_transmit('-');
        value = -value;
    }
    int32_t int_part = (int32_t)value;
    int32_t frac_part = (int32_t)((value - int_part) * 100.0f + 0.5f);

    if (int_part == 0) {
        usart_transmit('0');
    } else {
        char buf[12];
        int i = 0;
        while (int_part > 0 && i < (int)sizeof(buf)-1) {
            buf[i++] = '0' + (int_part % 10);
            int_part /= 10;
        }
        while (i > 0) usart_transmit(buf[--i]);
    }

    usart_transmit('.');
    if (frac_part < 10) usart_transmit('0');
    usart_transmit('0' + (frac_part / 10));
    usart_transmit('0' + (frac_part % 10));
}

// Non-blocking receive: returns -1 if no data, otherwise returns received byte (0-255)
int usart_receive_nonblocking(void) {
    if (UCSR0A & (1 << RXC0)) {
        return UDR0;
    }
    return -1;
}

void usart_flush_input(void) {
    // Read and discard anything in the receive buffer
    while (UCSR0A & (1 << RXC0)) {
        volatile uint8_t tmp = UDR0;
        (void)tmp;
    }
}

void floatToString(float value, char* str, int precision) {
    snprintf(str, 20, "%.*f", precision, value);
}

/* --------------------------------------------------------------------- */
/* Helper: compute TTC and stopping metrics and print them               */
/* --------------------------------------------------------------------- */
void compute_and_report_ttc(float dist_cm) {
    // Convert cm -> meters
    float dist_m = dist_cm / 100.0f;

    // Estimate linear speed (m/s) based on currentPwmMax (linear scaling)
    float speed_m_s = ((float)currentPwmMax / (float)pwmMax) * MAX_SPEED_M_S;

    if (currentMotion != 1 || speed_m_s <= 0.0001f) {
        // Not driving forward or stopped: report N/A or distance only
        usart_print_string("Dist: ");
        usart_print_float(dist_cm);
        usart_print_string(" cm  TTC: N/A\n");
        return;
    }

    // Time to collision if no braking (s)
    float ttc_no_brake = (speed_m_s > 0.0001f) ? (dist_m / speed_m_s) : -1.0f;

    // Stopping time under assumed deceleration (s): t_stop = v / a
    float t_stop = speed_m_s / DECEL_M_S2;

    // Stopping distance (m): d_stop = v^2 / (2*a)
    float d_stop = (speed_m_s * speed_m_s) / (2.0f * DECEL_M_S2);

    // Determine if we can stop before collision
    bool will_stop_in_time = (d_stop <= dist_m);

    // Print summary to USART
    usart_print_string("Dist: ");
    usart_print_float(dist_cm);
    usart_print_string(" cm  ");

    if (ttc_no_brake > 0.0f) {
        usart_print_string("TTC_no_brake: ");
        usart_print_float(ttc_no_brake);
        usart_print_string(" s  ");
    } 
    else
        usart_print_string("TTC_no_brake: inf  ");
    

    usart_print_string("t_stop: ");
    usart_print_float(t_stop);
    usart_print_string(" s  d_stop: ");
    // convert to cm for printing convenience
    usart_print_float(d_stop * 100.0f);
    usart_print_string(" cm  ");

    if (will_stop_in_time) {
        usart_print_string("SAFE\n");
    } else {
        usart_print_string("CRITICAL\n");
    }
}

/* --------------------------------------------------------------------- */
/* Main program                                                          */
/* --------------------------------------------------------------------- */

int main(void) {
    motor_init();
    ultrasonic_init();
    init_usart();
    lcd_init();
    float dist;

    // Make sure no leftover bytes confuse startup
    usart_flush_input();

    while (1) {
        // Check for Bluetooth command (non-blocking)
        int incoming = usart_receive_nonblocking();
        if (incoming != -1) {
            // handle single-character commands; keep behavior non-invasive
            char cmd = (char)incoming;
            if (cmd >= 'a' && cmd <= 'z') cmd = cmd - 'a' + 'A'; // uppercase

            // Speed control: '0'..'9'
            if (cmd >= '0' && cmd <= '9') {
                uint8_t level = cmd - '0'; // 0..9
                currentPwmMax = (uint8_t)((level * pwmMax) / 9);
                // keep currentMotion unchanged (speed adjusted)
            }

            switch (cmd) {
                case 'F': // Forward
                    currentMotion = 1;
                    runWithGeneratedPWM(2000UL, true, true, 0.5f);
                    break;
                case 'B': // Backward
                    currentMotion = -1;
                    runWithGeneratedPWM(2000UL, false, false, 0.5f);
                    break;
                case 'L': // Left: A backward, B forward
                    currentMotion = 2;
                    runWithGeneratedPWM(2000UL, false, true, 0.5f);
                    break;
                case 'R': // Right: A forward, B backward
                    currentMotion = 2;
                    runWithGeneratedPWM(2000UL, true, false, 0.5f);
                    break;
                case 'S': // Stop
                    OCR1A = 0;
                    OCR1B = 0;
                    currentMotion = 0;
                    break;
                case 'U': // U-Turn (rotate ~180 degrees)
                    // This keeps rotation angle consistent when speed changes.
                    unsigned long scaledUTurn = (UTURN_DURATION_MS * pwmMax) / (currentPwmMax + 1);
                    currentMotion = 2; // turning
                    runWithGeneratedPWM(scaledUTurn, true, false, 0.5f);
                    break;
                default:
                    // ignore unknown
                    break;
            }
            // after executing a bluetooth command, continue loop to perform ultrasonic prints/display
        }
        // periodic distance reporting
        // take a measurement and report TTC info
        float d1 = measure_distance();
        // Print two readings and compute TTC using the last measured distance.
        // First measurement:
        usart_print_float(d1);
        usart_print_string(" cm\n");

        _delay_ms(500);

        float d2 = measure_distance();
        usart_print_float(d2);
        usart_print_string(" cm\n");

        // Final measurement for LCD and TTC computation
        dist = measure_distance();

        // Clear LCD and show distance + basic info
        send_command(1);
        send_string("Dist: ");
        char dist_str[10];
        floatToString(dist, dist_str, 2);
        send_string(dist_str);
        send_string(" cm");

        // compute and report TTC info over serial
        compute_and_report_ttc(dist);

        _delay_ms(1000);
    }

    return 0;
}
