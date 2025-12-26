#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#define TWO_PI 6.28318530

// Motor pins - FIXED FOR YOUR WIRING
#define ENA     PB1  // D9 (OC1A)
#define ENB     PB2  // D10 (OC1B)

#define IN1     PD3  // D3
#define IN2     PD4  // D4
#define IN3     PD5  // D5
#define IN4     PD6  // D6

// Ultrasonic pins - FIXED FOR YOUR WIRING
#define TRIG_PIN PB4 // D12
#define ECHO_PIN PB3 // D11

// PWM generator parameters
const unsigned long pwmCycleMs = 2000UL;
const uint8_t pwmMin = 0;
const uint8_t pwmMax = 200;
const unsigned long sampleIntervalMs = 10UL;

/* LCD Pin Definitions (4-bit) - FIXED FOR YOUR WIRING */
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

    TCCR1A = (1 << WGM10) | (1 << COM1A1) | (1 << COM1B1);
    TCCR1B = (1 << WGM12) | (1 << CS11);
}

uint8_t generateSinePWM(unsigned long tMs, unsigned long cycleMs, uint8_t minVal, uint8_t maxVal, float phase) {
    if (cycleMs == 0) return maxVal;
    unsigned long t = tMs % cycleMs;
    float phaseT = (float)t / (float)cycleMs;
    phaseT += phase;
    phaseT = phaseT - floorf(phaseT);
    double angle = TWO_PI * (double)phaseT;
    double s = sin(angle);
    float normalized = (float)((s * 0.5) + 0.5);
    float pwmF = minVal + normalized * (float)(maxVal - minVal);
    if (pwmF < 0.0f) pwmF = 0.0f;
    if (pwmF > 255.0f) pwmF = 255.0f;
    return (uint8_t)(pwmF + 0.5f);
}

void runWithGeneratedPWM(unsigned long durationMs, bool Aforward, bool Bforward, float phaseShiftB) {
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
/* USART functions (TX only)                                             */
/* --------------------------------------------------------------------- */

void init_usart(void) {
    DDRD |= (1 << PD1);
    UBRR0H = 0;
    UBRR0L = 103;
    UCSR0B = (1 << TXEN0);
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

void floatToString(float value, char* str, int precision) {
    snprintf(str, 20, "%.*f", precision, value);
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

    while (1) {
        runWithGeneratedPWM(2000UL, true, true, 0.5f);
        _delay_ms(100);

        float d1 = measure_distance();
        usart_print_float(d1);
        usart_print_string(" cm (after forward)\n");

        _delay_ms(500);

        runWithGeneratedPWM(2000UL, false, false, 0.5f);
        _delay_ms(100);

        float d2 = measure_distance();
        usart_print_float(d2);
        usart_print_string(" cm (after reverse)\n");

        _delay_ms(1000);

        dist = measure_distance();
        send_command(1);
        send_string("Dist: ");
        char dist_str[10];
        floatToString(dist, dist_str, 2);
        send_string(dist_str);
        send_string(" cm");

        _delay_ms(1000);
    }

    return 0;
}