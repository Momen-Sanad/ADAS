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

/* Physical model assumptions for TTC (tune these) */
const float MAX_SPEED_M_S = 0.60f;   // adjust to measured top speed (m/s)
const float DECEL_M_S2    = 1.50f;   // adjust to measured braking deceleration (m/s^2)

/* Track current motion: 0=stop, 1=forward, -1=backward, 2=turning */
volatile int8_t currentMotion = 0;

/* lever and photo-LED pins (A0 / A1) on PORTC */
#define LEVERS_PIN    PC0  // A0
#define PHOTO_LED_PIN PC1  // A1

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

/* Init inputs for levers/photo-LED */
void inputs_init(void);

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
/* Init input pins for lever/photo-LED                                   */
/* --------------------------------------------------------------------- */
void inputs_init(void) {
    // LEVERS_PIN (A0 / PC0) as input with internal pull-up.
    // Assumes two toggles are wired in series connecting A0 to GND when both ON.
    DDRC &= ~(1 << LEVERS_PIN); // input
    PORTC |=  (1 << LEVERS_PIN); // enable pull-up

    // PHOTO_LED_PIN (A1 / PC1) as input, no pull-up; expects external circuit to drive it HIGH when LED lit.
    DDRC &= ~(1 << PHOTO_LED_PIN); // input
    PORTC &= ~(1 << PHOTO_LED_PIN); // no pull-up
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
    } else {
        usart_print_string("TTC_no_brake: inf  ");
    }

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
    inputs_init(); // initialize A0 / A1

    float dist;

    // Make sure no leftover bytes confuse startup
    usart_flush_input();

    // Persistent motor state variables
    bool motorA_on = false;
    bool motorB_on = false;
    bool motorA_forward = true; // direction if on
    bool motorB_forward = true;

    while (1) {
        // Read inputs
        bool photoLedOn = (PINC & (1 << PHOTO_LED_PIN)) ? true : false;
        // LEVERS_PIN uses internal pull-up; when both toggles in series are ON, pin is pulled to GND -> LOW
        bool levers_active = ((PINC & (1 << LEVERS_PIN)) == 0);

        // Update LCD second line with light status
        send_command(0xC0); // second line
        if (photoLedOn) {
            send_string("Light: ON  "); // padded to overwrite previous text
        } else {
            send_string("Light: OFF ");
        }

        if (!levers_active) {
            // Not allowed to run: show waiting message and ensure motors stopped
            motorA_on = motorB_on = false;
            OCR1A = 0;
            OCR1B = 0;
            currentMotion = 0;

            send_command(1); // clear and show waiting text
            send_string("Waiting switches");
            _delay_ms(300);
            continue;
        }

        // If we reach here, levers are active -> run the normal program behavior

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
                // If motors already on, immediately update OCR duty
                if (motorA_on) OCR1A = currentPwmMax;
                if (motorB_on) OCR1B = currentPwmMax;
            }

            switch (cmd) {
                case 'F': // Forward persistent
                    motorA_forward = true;
                    motorB_forward = true;
                    motorA_on = true;
                    motorB_on = true;
                    // Set directions for forward
                    PORTD |=  (1 << IN1);
                    PORTD &= ~(1 << IN2);
                    PORTD |=  (1 << IN3);
                    PORTD &= ~(1 << IN4);
                    // Apply PWM duty (persistent)
                    OCR1A = currentPwmMax;
                    OCR1B = currentPwmMax;
                    currentMotion = 1;
                    break;

                case 'B': // Backward persistent
                    motorA_forward = false;
                    motorB_forward = false;
                    motorA_on = true;
                    motorB_on = true;
                    // Set directions for backward
                    PORTD &= ~(1 << IN1);
                    PORTD |=  (1 << IN2);
                    PORTD &= ~(1 << IN3);
                    PORTD |=  (1 << IN4);
                    // Apply PWM duty (persistent)
                    OCR1A = currentPwmMax;
                    OCR1B = currentPwmMax;
                    currentMotion = -1;
                    break;

                case 'L': // Left persistent: A backward, B forward (in-place turn)
                    motorA_forward = false;
                    motorB_forward = true;
                    motorA_on = true;
                    motorB_on = true;
                    // Set directions
                    PORTD &= ~(1 << IN1);
                    PORTD |=  (1 << IN2);
                    PORTD |=  (1 << IN3);
                    PORTD &= ~(1 << IN4);
                    // Apply PWM duty
                    OCR1A = currentPwmMax;
                    OCR1B = currentPwmMax;
                    currentMotion = 2; // turning
                    break;

                case 'R': // Right persistent: A forward, B backward
                    motorA_forward = true;
                    motorB_forward = false;
                    motorA_on = true;
                    motorB_on = true;
                    // Set directions
                    PORTD |=  (1 << IN1);
                    PORTD &= ~(1 << IN2);
                    PORTD &= ~(1 << IN3);
                    PORTD |=  (1 << IN4);
                    // Apply PWM duty
                    OCR1A = currentPwmMax;
                    OCR1B = currentPwmMax;
                    currentMotion = 2; // turning
                    break;

                case 'S': // Stop persistent
                    motorA_on = false;
                    motorB_on = false;
                    OCR1A = 0;
                    OCR1B = 0;
                    currentMotion = 0;
                    break;

                case 'U': // U-turn: keep behavior as before (timed rotate)
                    currentMotion = 2;
                    // runWithGeneratedPWM handles its own OCR/stops at the end
                    runWithGeneratedPWM(1100UL, true, false, 0.5f);
                    // after runWithGeneratedPWM returns, motors are stopped inside that function
                    motorA_on = false;
                    motorB_on = false;
                    currentMotion = 0;
                    break;

                default:
                    // ignore unknown
                    break;
            }
            // after executing a bluetooth command, continue loop to perform ultrasonic prints/display
        }

        // autonomous periodic distance reporting
        // Make sure persistent motors keep running with current duty
        if (motorA_on) 
            OCR1A = currentPwmMax;
        else
            OCR1A = 0;
    
        if (motorB_on) 
            OCR1B = currentPwmMax;

        else 
            OCR1B = 0;
        

        float d1 = measure_distance();
        usart_print_float(d1);
        usart_print_string(" cm\n");

        _delay_ms(500);

        float d2 = measure_distance();
        usart_print_float(d2);
        usart_print_string(" cm\n");

        // Final measurement for LCD and TTC computation
        dist = measure_distance();

        // Show distance on LCD first line
        send_command(1);
        send_string("Dist: ");
        char dist_str[10];
        floatToString(dist, dist_str, 2);
        send_string(dist_str);
        send_string(" cm");

        // Update second line with light status again (since we cleared display)
        send_command(0xC0);
        
        if (photoLedOn) 
            send_string("Light: ON  ");
        
        else
            send_string("Light: OFF ");

        // compute and report TTC info over serial
        compute_and_report_ttc(dist);

        _delay_ms(1000);
    
    }

    return 0;
}