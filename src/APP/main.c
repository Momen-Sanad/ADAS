#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <util/atomic.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

/* Motor pins */
#define ENA     PB1  // D9 (OC1A)
#define ENB     PB2  // D10 (OC1B)

#define IN1     PD3  // D3
#define IN2     PD4  // D4
#define IN3     PD5  // D5
#define IN4     PD6  // D6

/* Ultrasonic pins */
#define TRIG_PIN PB4 // D12
#define ECHO_PIN PB3 // D11

/* PWM generator parameters */
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

/* ----------------- I2C (TWI) SLAVE CONFIG ----------------- */
/* Uses A4 (PC4 SDA) and A5 (PC5 SCL) hardware TWI pins */
#define I2C_SLAVE_ADDRESS  0x08

volatile char i2c_cmd = 0;
volatile uint8_t i2c_cmd_ready = 0;

/* Forward declarations */
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

void inputs_init(void);

/* I2C (TWI) helper initialize as slave */
void i2c_init_slave(uint8_t address);

/* Small helper to write OCR atomically to avoid torn writes */
static inline void setOCR_A(uint8_t val) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        OCR1A = val;
    }
}
static inline void setOCR_B(uint8_t val) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        OCR1B = val;
    }
}
static inline void setOCR_both(uint8_t a, uint8_t b) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        OCR1A = a;
        OCR1B = b;
    }
}

/* ----------------- LCD (4-bit HD44780) ----------------- */

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

/* ----------------- Motor / PWM code ----------------- */

void motor_init(void) {
    DDRD |= (1 << IN1) | (1 << IN2) | (1 << IN3) | (1 << IN4);
    DDRB |= (1 << ENA) | (1 << ENB);

    setOCR_both(0,0);

    // 8-bit Fast PWM (WGM10 + WGM12), non-inverting on OC1A/OC1B, prescaler=8 (CS11)
    TCCR1A = (1 << WGM10) | (1 << COM1A1) | (1 << COM1B1);
    TCCR1B = (1 << WGM12) | (1 << CS11);
}

/* 64-sample sine table scaled 0..255 (midpoint 128) */
const uint8_t sine8_64[64] = {
  128,140,152,164,175,186,196,205,213,220,226,231,235,238,240,241,
  241,240,238,235,231,226,220,213,205,196,186,175,164,152,140,128,
  115,103,91,79,68,57,47,38,30,23,17,12,8,5,3,2,
  2,3,5,8,12,17,23,30,38,47,57,68,79,91,103,115
};

/* generateSinePWM using table; keeps same signature for compatibility */
uint8_t generateSinePWM(unsigned long tMs, unsigned long cycleMs, uint8_t minVal, uint8_t maxVal, float phase) {
    (void)maxVal; // preserved param to keep compatibility
    if (cycleMs == 0) return currentPwmMax;
    unsigned long t = tMs % cycleMs;
    // index from 0..63 using integer math; add phase offset
    uint32_t idx = (uint32_t)((t * 64UL) / cycleMs) & 0x3F;
    int phaseOffset = (int)(phase * 64.0f);
    phaseOffset &= 0x3F;
    idx = (idx + (uint32_t)phaseOffset) & 0x3F;
    uint8_t s = sine8_64[idx]; // 0..255
    uint16_t pwm = (uint16_t)minVal + ((uint16_t)s * (uint16_t)(currentPwmMax - minVal)) / 255U;
    if (pwm > 255U) pwm = 255U;
    return (uint8_t)pwm;
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

        setOCR_both(pwmA, pwmB);

        _delay_ms(sampleIntervalMs);
        elapsed += sampleIntervalMs;
    }

    setOCR_both(0,0);

    // after stopping
    currentMotion = 0;
}

/* ----------------- Ultrasonic code (TRIG on D12 / ECHO on D11) ----------------- */

void ultrasonic_init(void) {
    DDRB |=  (1 << TRIG_PIN);
    DDRB &= ~(1 << ECHO_PIN);
    PORTB &= ~(1 << TRIG_PIN); // TRIG low
    PORTB &= ~(1 << ECHO_PIN); // no pull-up on ECHO
}

float measure_distance(void) {
    uint32_t duration = 0;
    uint32_t timeout;

    // trigger pulse
    PORTB &= ~(1 << TRIG_PIN);
    _delay_us(2);
    PORTB |=  (1 << TRIG_PIN);
    _delay_us(10);
    PORTB &= ~(1 << TRIG_PIN);

    // wait for echo rising edge
    timeout = 60000UL; // microsecond timeout (~60 ms) - adjust based on required max range
    while (!(PINB & (1 << ECHO_PIN)) && timeout--) { _delay_us(1); }
    if (timeout == 0) return 1000.0f; // return big value in cm for no echo

    duration = 0;
    timeout = 60000UL;
    while ((PINB & (1 << ECHO_PIN)) && timeout--) { _delay_us(1); duration++; }
    if (timeout == 0) return 1000.0f;

    // duration is in microseconds; sound speed ~343 m/s -> 0.0343 cm/us
    float dist_cm = (duration * 0.0343f) / 2.0f;
    if (dist_cm > 1000.0f) dist_cm = 1000.0f;
    return dist_cm;
}

/* ----------------- USART functions ----------------- */

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

int usart_receive_nonblocking(void) {
    if (UCSR0A & (1 << RXC0)) {
        return UDR0;
    }
    return -1;
}

void usart_flush_input(void) {
    while (UCSR0A & (1 << RXC0)) {
        volatile uint8_t tmp = UDR0;
        (void)tmp;
    }
}

/* Lightweight float->string with limited precision
   precision 0..3 supported. */
void floatToString(float value, char* str, int precision) {
    if (precision < 0) precision = 0;
    if (precision > 3) precision = 3;
    bool neg = false;
    if (value < 0.0f) { neg = true; value = -value; }

    long int_part = (long)value;
    float fracf = value - (float)int_part;
    long pow10 = 1;
    for (int i = 0; i < precision; ++i) pow10 *= 10;
    long frac = (long)(fracf * (float)pow10 + 0.5f);

    // handle carry from rounding fraction
    if (frac >= pow10) {
        frac -= pow10;
        int_part += 1;
    }

    char tmp[16];
    int idx = 0;
    if (neg) tmp[idx++] = '-';

    // integer part to string
    char rev[12];
    int ri = 0;
    if (int_part == 0) rev[ri++] = '0';
    else {
        long ip = int_part;
        while (ip > 0 && ri < (int)sizeof(rev)-1) {
            rev[ri++] = '0' + (ip % 10);
            ip /= 10;
        }
    }
    for (int i = ri-1; i >= 0; --i) tmp[idx++] = rev[i];

    if (precision > 0) {
        tmp[idx++] = '.';
        // print fractional part with leading zeros
        long div = pow10 / 10;
        for (int p = 0; p < precision; ++p) {
            char digit = '0' + (char)( (frac / div) % 10 );
            tmp[idx++] = digit;
            div /= 10;
        }
    }
    tmp[idx] = '\0';
    // copy out
    for (int i = 0; tmp[i] != '\0'; ++i) str[i] = tmp[i];
    str[idx] = '\0';
}

/* ----------------- Inputs init ----------------- */
void inputs_init(void) {
    // LEVERS_PIN (A0 / PC0) as input with internal pull-up.
    DDRC &= ~(1 << LEVERS_PIN); // input
    PORTC |=  (1 << LEVERS_PIN); // enable pull-up

    // PHOTO_LED_PIN (A1 / PC1) as input, no pull-up; expects external circuit to drive it HIGH when LED lit.
    DDRC &= ~(1 << PHOTO_LED_PIN); // input
    PORTC &= ~(1 << PHOTO_LED_PIN); // no pull-up
}

/* ----------------- I2C (TWI) SLAVE implementation ----------------- */

/* Initialize TWI as slave with given 7-bit address */
void i2c_init_slave(uint8_t address)
{
    // Set own slave address (TWAR holds address << 1)
    TWAR = (address << 1);

    // Enable TWI, enable ACK, enable TWI interrupt
    TWCR = (1 << TWEN) | (1 << TWEA) | (1 << TWIE);

    // Note: PC4/PC5 are automatically used by hardware TWI
}

/* TWI interrupt service routine */
/* We only capture the first data byte sent and set a flag for the main loop */
ISR(TWI_vect)
{
    uint8_t status = TWSR & 0xF8;

    switch (status)
    {
        case 0x60: // Own SLA+W received, ACK returned
        case 0x68: // Own SLA+W received; previously addressed with general call
            // Prepare to receive data, ACK next byte
            TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA) | (1 << TWIE);
            break;

        case 0x80: // Previously addressed with own SLA+W; data received; ACK returned
        case 0x90: // Previously addressed with general call; data received; ACK returned
            // Read the received byte
            i2c_cmd = TWDR;
            i2c_cmd_ready = 1;
            // Keep ACKing to allow further transactions; main loop will consume flag
            TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA) | (1 << TWIE);
            break;

        case 0xA0: // STOP or repeated START received while still addressed as slave
            // Nothing to do, just clear interrupt and stay ready
            TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA) | (1 << TWIE);
            break;

        default:
            // For all other states, just resume TWI and ACK
            TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA) | (1 << TWIE);
            break;
    }
}

/* ----------------- TTC and reporting ----------------- */
void compute_and_report_ttc(float dist_cm) {
    float dist_m = dist_cm / 100.0f;
    float speed_m_s = ((float)currentPwmMax / (float)pwmMax) * MAX_SPEED_M_S;

    if (currentMotion != 1 || speed_m_s <= 0.0001f) {
        usart_print_string("Dist: ");
        usart_print_float(dist_cm);
        usart_print_string(" cm  TTC: N/A\n");
        return;
    }

    float ttc_no_brake = (speed_m_s > 0.0001f) ? (dist_m / speed_m_s) : -1.0f;
    float t_stop = speed_m_s / DECEL_M_S2;
    float d_stop = (speed_m_s * speed_m_s) / (2.0f * DECEL_M_S2);
    bool will_stop_in_time = (d_stop <= dist_m);

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
    usart_print_float(d_stop * 100.0f);
    usart_print_string(" cm  ");

    if (will_stop_in_time) {
        usart_print_string("SAFE\n");
    } else {
        usart_print_string("CRITICAL\n");
    }
}

/* ----------------- Main ----------------- */
int main(void) {
    motor_init();
    ultrasonic_init();
    init_usart();
    lcd_init();
    inputs_init();

    /* Initialize I2C TWI as slave and enable interrupts */
    i2c_init_slave(I2C_SLAVE_ADDRESS);
    sei(); // enable global interrupts so TWI ISR can fire

    float dist;

    usart_flush_input();

    bool motorA_on = false;
    bool motorB_on = false;
    bool motorA_forward = true;
    bool motorB_forward = true;

    // Initial display
    send_command(0x80);
    send_string("Dist: --.-- cm");
    send_command(0xC0);
    send_string("Light: OFF ");

    while (1) {
        // Read inputs
        bool photoLedOn = (PINC & (1 << PHOTO_LED_PIN)) ? true : false;
        bool levers_active = ((PINC & (1 << LEVERS_PIN)) == 0); // pull-up logic

        // Update second line with light status (non-destructive)
        send_command(0xC0);
        if (photoLedOn) send_string("Light: ON  ");
        else            send_string("Light: OFF ");

        if (!levers_active) {
            // Not allowed to run: ensure motors stopped (coast)
            motorA_on = motorB_on = false;
            // set motor inputs low -> coast (check H-bridge datasheet)
            PORTD &= ~((1<<IN1)|(1<<IN2)|(1<<IN3)|(1<<IN4));
            setOCR_both(0,0);
            currentMotion = 0;

            // show waiting message on first line (once)
            send_command(0x01); // clear
            _delay_ms(2);
            send_command(0x80);
            send_string("Waiting switches");
            _delay_ms(300);
            continue;
        }

        // Bluetooth command (non-blocking) BUT give I2C priority if available
        int incoming = -1;

        if (i2c_cmd_ready) {
            cli();
            incoming = (int)i2c_cmd;
            i2c_cmd_ready = 0;
            sei();
        } else {
            incoming = usart_receive_nonblocking();
        }

        if (incoming != -1) {
            char cmd = (char)incoming;
            if (cmd >= 'a' && cmd <= 'z') cmd = cmd - 'a' + 'A';

            // Speed control digits '0'..'9'
            if (cmd >= '0' && cmd <= '9') {
                uint8_t level = cmd - '0'; // 0..9
                currentPwmMax = (uint8_t)((level * pwmMax) / 9);
                // update active channels
                if (motorA_on && motorB_on) setOCR_both(currentPwmMax, currentPwmMax);
                else if (motorA_on) setOCR_A(currentPwmMax);
                else if (motorB_on) setOCR_B(currentPwmMax);
            }

            switch (cmd) {
                case 'F':
                    motorA_forward = true;
                    motorB_forward = true;
                    motorA_on = motorB_on = true;
                    PORTD |=  (1 << IN1); PORTD &= ~(1 << IN2);
                    PORTD |=  (1 << IN3); PORTD &= ~(1 << IN4);
                    setOCR_both(currentPwmMax, currentPwmMax);
                    currentMotion = 1;
                    break;

                case 'B':
                    motorA_forward = false;
                    motorB_forward = false;
                    motorA_on = motorB_on = true;
                    PORTD &= ~(1 << IN1); PORTD |=  (1 << IN2);
                    PORTD &= ~(1 << IN3); PORTD |=  (1 << IN4);
                    setOCR_both(currentPwmMax, currentPwmMax);
                    currentMotion = -1;
                    break;

                case 'L': // left turn in-place
                    motorA_forward = false;
                    motorB_forward = true;
                    motorA_on = motorB_on = true;
                    PORTD &= ~(1 << IN1); PORTD |=  (1 << IN2);
                    PORTD |=  (1 << IN3); PORTD &= ~(1 << IN4);
                    setOCR_both(currentPwmMax, currentPwmMax);
                    currentMotion = 2;
                    break;

                case 'R': // right turn in-place
                    motorA_forward = true;
                    motorB_forward = false;
                    motorA_on = motorB_on = true;
                    PORTD |=  (1 << IN1); PORTD &= ~(1 << IN2);
                    PORTD &= ~(1 << IN3); PORTD |=  (1 << IN4);
                    setOCR_both(currentPwmMax, currentPwmMax);
                    currentMotion = 2;
                    break;

                case 'S': // stop persistent
                    motorA_on = motorB_on = false;
                    setOCR_both(0,0);
                    currentMotion = 0;
                    break;

                case 'U': // timed rotate / U-turn
                    currentMotion = 2;
                    runWithGeneratedPWM(1100UL, true, false, 0.5f);
                    motorA_on = motorB_on = false;
                    currentMotion = 0;
                    break;

                default:
                    break;
            }
        }

        // maintain persistent PWM duty for motors
        if (motorA_on && motorB_on) setOCR_both(currentPwmMax, currentPwmMax);
        else if (motorA_on) setOCR_A(currentPwmMax);
        else if (motorB_on) setOCR_B(currentPwmMax);
        else setOCR_both(0,0);

        // Two quick distance prints (as before)
        float d1 = measure_distance();
        usart_print_float(d1);
        usart_print_string(" cm\n");
        _delay_ms(500);

        float d2 = measure_distance();
        usart_print_float(d2);
        usart_print_string(" cm\n");

        // Final measurement for LCD and TTC computation
        dist = measure_distance();

        // Update LCD first line without clearing whole display to reduce flicker
        send_command(0x80); // position to first line start
        char dist_str[12];
        floatToString(dist, dist_str, 2);
        send_string("Dist: ");
        send_string(dist_str);
        send_string(" cm   "); // pad to overwrite previous content

        // Update second line with light status again (to ensure consistent)
        send_command(0xC0);
        if (photoLedOn) send_string("Light: ON  ");
        else            send_string("Light: OFF ");

        // compute and report TTC info over serial
        compute_and_report_ttc(dist);

        _delay_ms(1000);
    }

    return 0;
}