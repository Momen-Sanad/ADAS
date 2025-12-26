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

#define IN1     PD2  // D2
#define IN2     PD3  // D3
#define IN3     PD4  // D4
#define IN4     PD5  // D5

// Ultrasonic pins
#define TRIG_PIN PD7 // D7
#define ECHO_PIN PD6 // D6

// PWM generator parameters
const unsigned long pwmCycleMs = 2000UL;   // milliseconds per sine cycle
const uint8_t pwmMin = 0;                  // PWM min 0..255
const uint8_t pwmMax = 200;                // PWM max (leave headroom)
const unsigned long sampleIntervalMs = 10UL; // update rate in ms

/* LCD Pin Definitions (4-bit) */
// Control
#define CTRL_Dir  DDRB
#define CTRL_Port PORTB
#define RS_PIN    PB0   // D8
#define E_pin     PB3   // D11

// Data (4-bit) -> A0..A3 (PC0..PC3)
#define LCD_Data_Dir DDRC
#define LCD_Data_Port PORTC
#define LCD_D4 PC0
#define LCD_D5 PC1
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
    CTRL_Port |= (1 << E_pin);
    _delay_us(1);
    CTRL_Port &= ~(1 << E_pin);
    _delay_us(50); // let LCD process
}

static void lcd_write_nibble(uint8_t nibble) {
    // clear PC0..PC3
    LCD_Data_Port &= ~((1 << LCD_D4) | (1 << LCD_D5) | (1 << LCD_D6) | (1 << LCD_D7));
    // set according to nibble (bit0->D4 ... bit3->D7)
    if (nibble & 0x01) LCD_Data_Port |= (1 << LCD_D4);
    if (nibble & 0x02) LCD_Data_Port |= (1 << LCD_D5);
    if (nibble & 0x04) LCD_Data_Port |= (1 << LCD_D6);
    if (nibble & 0x08) LCD_Data_Port |= (1 << LCD_D7);

    lcd_pulse_enable();
}

static void lcd_send_byte(uint8_t value, bool is_data) {
    if (is_data) CTRL_Port |= (1 << RS_PIN);
    else         CTRL_Port &= ~(1 << RS_PIN);

    // RW must be tied to GND physically

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
    // configure direction registers
    CTRL_Dir |= (1 << RS_PIN) | (1 << E_pin); // RS and E outputs
    LCD_Data_Dir |= (1 << LCD_D4) | (1 << LCD_D5) | (1 << LCD_D6) | (1 << LCD_D7);

    CTRL_Port &= ~((1 << RS_PIN) | (1 << E_pin));
    LCD_Data_Port &= ~((1 << LCD_D4) | (1 << LCD_D5) | (1 << LCD_D6) | (1 << LCD_D7));

    _delay_ms(40); // wait for LCD to power up

    // Init sequence for 4-bit mode
    lcd_write_nibble(0x03);
    _delay_ms(5);
    lcd_write_nibble(0x03);
    _delay_us(150);
    lcd_write_nibble(0x03);
    _delay_us(150);
    lcd_write_nibble(0x02); // set 4-bit mode
    _delay_us(150);

    send_command(0x28); // 4-bit, 2 lines, 5x8
    send_command(0x0C); // display ON, cursor OFF, blink OFF
    send_command(0x01); // clear
    _delay_ms(2);
    send_command(0x06); // entry mode: inc, no shift
}

/* --------------------------------------------------------------------- */
/* Motor / PWM code                                                      */
/* --------------------------------------------------------------------- */

void motor_init(void) {
    // Configure direction pins (IN1..IN4) as outputs (PD2..PD5)
    DDRD |= (1 << IN1) | (1 << IN2) | (1 << IN3) | (1 << IN4);

    // Configure EN pins (PB1, PB2) as outputs
    DDRB |= (1 << ENA) | (1 << ENB);

    // Ensure motors stopped initially
    OCR1A = 0;
    OCR1B = 0;

    // Fast PWM 8-bit, non-inverting for OC1A/OC1B, prescaler = 8
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
/* Ultrasonic code (TRIG on PD7 / ECHO on PD6)                           */
/* --------------------------------------------------------------------- */

void ultrasonic_init(void) {
    DDRD |=  (1 << TRIG_PIN);   // TRIG output (PD7)
    DDRD &= ~(1 << ECHO_PIN);   // ECHO input (PD6)
    PORTD &= ~(1 << TRIG_PIN);
    PORTD &= ~(1 << ECHO_PIN); // no pull-up
}

float measure_distance(void) {
    uint32_t duration = 0;
    uint32_t timeout;

    PORTD &= ~(1 << TRIG_PIN);
    _delay_us(2);
    PORTD |=  (1 << TRIG_PIN);
    _delay_us(10);
    PORTD &= ~(1 << TRIG_PIN);

    timeout = 30000UL;
    while (!(PIND & (1 << ECHO_PIN)) && timeout--) { _delay_us(1); }
    if (timeout == 0) return 100.0f;

    duration = 0;
    timeout = 30000UL;
    while ((PIND & (1 << ECHO_PIN)) && timeout--) { _delay_us(1); duration++; }
    if (timeout == 0) return 100.0f;

    float dist = (duration * 0.0343f) / 2.0f;
    if (dist > 100.0f) dist = 100.0f;
    return dist;
}

/* --------------------------------------------------------------------- */
/* USART functions (TX only)                                             */
/* --------------------------------------------------------------------- */

void init_usart(void) {
    DDRD |= (1 << PD1);   // D1 = TX
    UBRR0H = 0;
    UBRR0L = 103; // 9600 @ 16 MHz
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
        send_command(1);  // Clear LCD
        send_string("Dist: ");
        char dist_str[10];
        floatToString(dist, dist_str, 2);
        send_string(dist_str);
        send_string(" cm");

        _delay_ms(1000);
    }

    return 0;
}