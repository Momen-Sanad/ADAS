#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>


#define OBSTACLE_THRESHOLD_CM   5       // Stop if distance <= 5cm
#define TURN_90_TIME_MS         400     // Time to turn 90 degrees (adjust!)
#define TURN_180_TIME_MS        800     // Time to turn 180 degrees (adjust!)
#define TURN_SPEED              150     // PWM speed when turning to look
#define DATA_SEND_INTERVAL_MS   500     // Send data every 500ms (2x per second)
#define SEARCH_DELAY_MS         200     // Delay after turn before measuring


// Motor pins (L298N)
#define ENA     PB1  // D9 (OC1A - PWM)
#define ENB     PB2  // D10 (OC1B - PWM)
#define IN1     PD3  // D3
#define IN2     PD4  // D4
#define IN3     PD5  // D5
#define IN4     PD6  // D6

// Ultrasonic pins (HC-SR04)
#define TRIG_PIN PB4  // D12
#define ECHO_PIN PB3  // D11

// LCD pins (4-bit mode)
#define LCD_RS_PIN    PD2   // D2
#define LCD_E_PIN     PD7   // D7
#define LCD_D4        PB0   // D8
#define LCD_D5        PB5   // D13
#define LCD_D6        PC2   // A2
#define LCD_D7        PC3   // A3

// Bluetooth on D0/D1 (Hardware UART)
// HC-05 TXD → D0 (RX)
// HC-05 RXD → D1 (TX)

/* ===================================================================== */
/* GLOBAL VARIABLES                                                      */
/* ===================================================================== */

volatile uint8_t targetSpeed = 0;
volatile uint8_t currentSpeed = 0;
volatile bool brakeActive = false;

// Direction: 0=STOP, 1=FWD, 2=RIGHT, 3=LEFT
#define DIR_STOP    0
#define DIR_FWD     1
#define DIR_RIGHT   2
#define DIR_LEFT    3
volatile uint8_t currentDirection = DIR_STOP;

#define RX_BUFFER_SIZE 32
volatile char rxBuffer[RX_BUFFER_SIZE];
volatile uint8_t rxIndex = 0;
volatile bool commandReady = false;

// Timing
volatile uint32_t millisCounter = 0;
uint32_t lastDataSendTime = 0;

float currentDistance = 999.0f;


void timer0_init(void) {
    TCCR0A = (1 << WGM01);
    TCCR0B = (1 << CS01) | (1 << CS00);
    OCR0A = 249;
    TIMSK0 = (1 << OCIE0A);
}

ISR(TIMER0_COMPA_vect) {
    millisCounter++;
}

uint32_t millis(void) {
    uint32_t ms;
    cli();
    ms = millisCounter;
    sei();
    return ms;
}

void delay_ms(uint16_t ms) {
    uint32_t start = millis();
    while (millis() - start < ms);
}

void uart_init(void) {
    UBRR0H = 0;
    UBRR0L = 103;
    UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void uart_transmit(unsigned char data) {
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = data;
}

void uart_print(const char* str) {
    while (*str) uart_transmit(*str++);
}

void uart_print_int(int16_t value) {
    if (value < 0) {
        uart_transmit('-');
        value = -value;
    }
    if (value == 0) {
        uart_transmit('0');
        return;
    }
    char buf[8];
    int8_t i = 0;
    while (value > 0 && i < 7) {
        buf[i++] = '0' + (value % 10);
        value /= 10;
    }
    while (i > 0) uart_transmit(buf[--i]);
}

void uart_print_float(float value) {
    if (value < 0.0f) {
        uart_transmit('-');
        value = -value;
    }
    int16_t int_part = (int16_t)value;
    int16_t frac_part = (int16_t)((value - int_part) * 10.0f + 0.5f);
    uart_print_int(int_part);
    uart_transmit('.');
    uart_print_int(frac_part);
}

ISR(USART_RX_vect) {
    char c = UDR0;
    if (c == '\n' || c == '\r') {
        if (rxIndex > 0) {
            rxBuffer[rxIndex] = '\0';
            commandReady = true;
        }
        rxIndex = 0;
    } else if (rxIndex < RX_BUFFER_SIZE - 1) {
        rxBuffer[rxIndex++] = c;
    }
}
// Speed Setting + brake as using Bluetooth
void processCommand(void) {
    if (!commandReady) return;
    
    char cmd[RX_BUFFER_SIZE];
    cli();
    strcpy(cmd, (const char*)rxBuffer);
    commandReady = false;
    sei();
    
    if (strncmp(cmd, "SPD:", 4) == 0) {
        int speed = atoi(cmd + 4);
        if (speed < 0) speed = 0;
        if (speed > 255) speed = 255;
        targetSpeed = (uint8_t)speed;
        brakeActive = false;
        uart_print("OK:SPD=");
        uart_print_int(targetSpeed);
        uart_print("\r\n");
    }
    else if (strcmp(cmd, "STOP") == 0) {
        brakeActive = true;
        targetSpeed = 0;
        uart_print("OK:BRAKE\r\n");
    }
    else {
        uart_print("ERR:UNKNOWN\r\n");
    }
}

// LCS drivers
static void lcd_pulse_enable(void) {
    PORTD |= (1 << LCD_E_PIN);
    _delay_us(1);
    PORTD &= ~(1 << LCD_E_PIN);
    _delay_us(50);
}

static void lcd_write_nibble(uint8_t nibble) {
    PORTB &= ~((1 << LCD_D4) | (1 << LCD_D5));
    PORTC &= ~((1 << LCD_D6) | (1 << LCD_D7));
    if (nibble & 0x01) PORTB |= (1 << LCD_D4);
    if (nibble & 0x02) PORTB |= (1 << LCD_D5);
    if (nibble & 0x04) PORTC |= (1 << LCD_D6);
    if (nibble & 0x08) PORTC |= (1 << LCD_D7);
    lcd_pulse_enable();
}

static void lcd_send_byte(uint8_t value, bool is_data) {
    if (is_data) PORTD |= (1 << LCD_RS_PIN);
    else         PORTD &= ~(1 << LCD_RS_PIN);
    lcd_write_nibble((value >> 4) & 0x0F);
    lcd_write_nibble(value & 0x0F);
    _delay_us(50);
}

void lcd_command(uint8_t cmd) {
    lcd_send_byte(cmd, false);
    if (cmd == 0x01 || cmd == 0x02) _delay_ms(2);
}

void lcd_data(char data) {
    lcd_send_byte((uint8_t)data, true);
}

void lcd_print(const char *str) {
    while (*str) lcd_data(*str++);
}

void lcd_set_cursor(uint8_t row, uint8_t col) {
    uint8_t address = (row == 0) ? 0x00 : 0x40;
    lcd_command(0x80 | (address + col));
}

void lcd_clear(void) {
    lcd_command(0x01);
}

void lcd_print_int(int16_t value) {
    if (value < 0) {
        lcd_data('-');
        value = -value;
    }
    if (value == 0) {
        lcd_data('0');
        return;
    }
    char buf[8];
    int8_t i = 0;
    while (value > 0 && i < 7) {
        buf[i++] = '0' + (value % 10);
        value /= 10;
    }
    while (i > 0) lcd_data(buf[--i]);
}
void lcd_init(void) {
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
    lcd_command(0x28);
    lcd_command(0x0C);
    lcd_command(0x01);
    _delay_ms(2);
    lcd_command(0x06);
}


void motor_init(void) {
    DDRD |= (1 << IN1) | (1 << IN2) | (1 << IN3) | (1 << IN4);
    DDRB |= (1 << ENA) | (1 << ENB);
    PORTD &= ~((1 << IN1) | (1 << IN2) | (1 << IN3) | (1 << IN4));
    OCR1A = 0;
    OCR1B = 0;
    TCCR1A = (1 << WGM10) | (1 << COM1A1) | (1 << COM1B1);
    TCCR1B = (1 << WGM12) | (1 << CS11);
}
// Motors Drivers
void motor_stop(void) {
    OCR1A = 0;
    OCR1B = 0;
    PORTD &= ~((1 << IN1) | (1 << IN2) | (1 << IN3) | (1 << IN4));
    currentSpeed = 0;
    currentDirection = DIR_STOP;
}

void motor_forward(uint8_t speed) {
    PORTD |= (1 << IN1);
    PORTD &= ~(1 << IN2);
    PORTD |= (1 << IN3);
    PORTD &= ~(1 << IN4);
    OCR1A = speed;
    OCR1B = speed;
    currentSpeed = speed;
    currentDirection = DIR_FWD;
}

void motor_turn_right(uint8_t speed) {
    PORTD |= (1 << IN1);
    PORTD &= ~(1 << IN2);
    PORTD &= ~(1 << IN3);
    PORTD |= (1 << IN4);
    OCR1A = speed;
    OCR1B = speed;
    currentDirection = DIR_RIGHT;
}

void motor_turn_left(uint8_t speed) {
    PORTD &= ~(1 << IN1);
    PORTD |= (1 << IN2);
    PORTD |= (1 << IN3);
    PORTD &= ~(1 << IN4);
    OCR1A = speed;
    OCR1B = speed;
    currentDirection = DIR_LEFT;
}
// sensor's Shit --should stops at 5 cm
void ultrasonic_init(void) {
    DDRB |= (1 << TRIG_PIN);
    DDRB &= ~(1 << ECHO_PIN);
    PORTB &= ~(1 << TRIG_PIN);
    PORTB &= ~(1 << ECHO_PIN);
}

float measure_distance(void) {
    uint32_t duration = 0;
    uint32_t timeout;

    PORTB &= ~(1 << TRIG_PIN);
    _delay_us(2);
    PORTB |= (1 << TRIG_PIN);
    _delay_us(10);
    PORTB &= ~(1 << TRIG_PIN);

    timeout = 30000UL;
    while (!(PINB & (1 << ECHO_PIN)) && timeout--) { _delay_us(1); }
    if (timeout == 0) return 999.0f;

    duration = 0;
    timeout = 30000UL;
    while ((PINB & (1 << ECHO_PIN)) && timeout--) { _delay_us(1); duration++; }
    if (timeout == 0) return 999.0f;

    float dist = (duration * 0.0343f) / 2.0f;
    if (dist > 400.0f) dist = 400.0f;
    return dist;
}

// motors as logic -- Direction 
void turn_right_90(void) {
    motor_turn_right(TURN_SPEED);
    delay_ms(TURN_90_TIME_MS);
    motor_stop();
    delay_ms(SEARCH_DELAY_MS);
}

void turn_left_90(void) {
    motor_turn_left(TURN_SPEED);
    delay_ms(TURN_90_TIME_MS);
    motor_stop();
    delay_ms(SEARCH_DELAY_MS);
}

void turn_left_180(void) {
    motor_turn_left(TURN_SPEED);
    delay_ms(TURN_180_TIME_MS);
    motor_stop();
    delay_ms(SEARCH_DELAY_MS);
}

void spin_180(void) {
    motor_turn_right(TURN_SPEED);
    delay_ms(TURN_180_TIME_MS);
    motor_stop();
    delay_ms(SEARCH_DELAY_MS);
}

uint8_t searchClearDirection(void) {
    float dist;
    
    turn_right_90();
    dist = measure_distance();
    if (dist > OBSTACLE_THRESHOLD_CM) {
        return DIR_RIGHT;
    }
    
    turn_left_180();
    dist = measure_distance();
    if (dist > OBSTACLE_THRESHOLD_CM) {
        return DIR_LEFT;
    }
    
    spin_180();
    dist = measure_distance();
    if (dist > OBSTACLE_THRESHOLD_CM) {
        return DIR_FWD;
    }
    
    return DIR_STOP;
}

//Data Sending

void sendDataToPC(void) {
    uart_print("DIST:");
    if (currentDistance >= 999.0f) {
        uart_print("CLEAR");
    } else {
        uart_print_float(currentDistance);
    }
    
    uart_print(",SPD:");
    uart_print_int(currentSpeed);
    
    uart_print(",DIR:");
    switch (currentDirection) {
        case DIR_FWD:   uart_print("FWD");   break;
        case DIR_RIGHT: uart_print("RIGHT"); break;
        case DIR_LEFT:  uart_print("LEFT");  break;
        default:        uart_print("STOP");  break;
    }
    
    uart_print(",STATUS:");
    if (brakeActive) {
        uart_print("BRAKE");
    } else if (currentDistance <= OBSTACLE_THRESHOLD_CM) {
        uart_print("OBSTACLE");
    } else {
        uart_print("OK");
    }
    
    uart_print("\r\n");
}

// LCD
void updateLCD(void) {
    lcd_clear();
    
    lcd_set_cursor(0, 0);
    lcd_print("D:");
    if (currentDistance >= 999.0f) {
        lcd_print("CLR");
    } else {
        lcd_print_int((int16_t)currentDistance);
    }
    lcd_print("cm S:");
    lcd_print_int(currentSpeed);
    
    lcd_set_cursor(1, 0);
    switch (currentDirection) {
        case DIR_FWD:   lcd_print("FWD  "); break;
        case DIR_RIGHT: lcd_print("RIGHT"); break;
        case DIR_LEFT:  lcd_print("LEFT "); break;
        default:        lcd_print("STOP "); break;
    }
    
    if (brakeActive) {
        lcd_print(" BRAKE");
    } else if (currentDistance <= OBSTACLE_THRESHOLD_CM) {
        lcd_print(" OBST!");
    } else {
        lcd_print(" OK");
    }
}
// Main Func
int main(void) {
    motor_init();
    ultrasonic_init();
    uart_init();
    lcd_init();
    timer0_init();
    
    sei();
    
    lcd_clear();
    lcd_print("ADAS System");
    lcd_set_cursor(1, 0);
    lcd_print("Ready...");
    
    uart_print("ADAS System Ready\r\n");
    uart_print("CMD: SPD:0-255, STOP\r\n");
    
    delay_ms(1000);
    
    while (1) {
        processCommand();
        
        currentDistance = measure_distance();
        
        if (brakeActive) {
            motor_stop();
        }
        else if (currentDistance <= OBSTACLE_THRESHOLD_CM && targetSpeed > 0) {
            motor_stop();
            uart_print("OBSTACLE! Searching...\r\n");
            
            uint8_t newDir = searchClearDirection();
            
            if (newDir != DIR_STOP && !brakeActive) {
                motor_forward(targetSpeed);
                uart_print("Clear: ");
                switch (newDir) {
                    case DIR_FWD:   uart_print("FWD\r\n");   break;
                    case DIR_RIGHT: uart_print("RIGHT\r\n"); break;
                    case DIR_LEFT:  uart_print("LEFT\r\n");  break;
                }
            } else {
                motor_stop();
                uart_print("All blocked!\r\n");
            }
        }
        else if (targetSpeed > 0 && !brakeActive) {
            motor_forward(targetSpeed);
        }
        else {
            motor_stop();
        }
        
        if (millis() - lastDataSendTime >= DATA_SEND_INTERVAL_MS) {
            lastDataSendTime = millis();
            sendDataToPC();
            updateLCD();
        }
        
        delay_ms(50);
    }

    return 0;
}