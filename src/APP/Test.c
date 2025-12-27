#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

/* ===================================================================== */
/* PARAMETERS                                                            */
/* ===================================================================== */

#define OBSTACLE_THRESHOLD_CM   5
#define TURN_90_TIME_MS         400
#define TURN_180_TIME_MS        800
#define TURN_SPEED              150
#define DATA_SEND_INTERVAL_MS   500
#define SEARCH_DELAY_MS         200

/* ===================================================================== */
/* PINS                                                                  */
/* ===================================================================== */

#define ENA     PB1
#define ENB     PB2
#define IN1     PD3
#define IN2     PD4
#define IN3     PD5
#define IN4     PD6

#define TRIG_PIN PB4
#define ECHO_PIN PB3

#define LCD_RS_PIN    PD2
#define LCD_E_PIN     PD7
#define LCD_D4        PB0
#define LCD_D5        PB5
#define LCD_D6        PC2
#define LCD_D7        PC3

/* ===================================================================== */
/* GLOBALS                                                               */
/* ===================================================================== */

volatile uint8_t targetSpeed = 0;
volatile uint8_t currentSpeed = 0;
volatile bool brakeActive = false;
volatile bool isMoving = false;

#define DIR_STOP    0
#define DIR_FWD     1
#define DIR_RIGHT   2
#define DIR_LEFT    3
volatile uint8_t currentDirection = DIR_STOP;

#define RX_BUFFER_SIZE 32
volatile char rxBuffer[RX_BUFFER_SIZE];
volatile uint8_t rxIndex = 0;
volatile bool commandReady = false;

volatile uint32_t millisCounter = 0;
uint32_t lastDataSendTime = 0;
uint32_t loopCounter = 0;

float currentDistance = 999.0f;

/* ===================================================================== */
/* TIMER                                                                 */
/* ===================================================================== */

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

/* ===================================================================== */
/* UART - SIMPLE VERSION FOR DEBUG                                       */
/* ===================================================================== */

void uart_init(void) {
    // Try 9600 baud
    UBRR0H = 0;
    UBRR0L = 103;
    UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void uart_tx(char c) {
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = c;
}

void uart_print(const char* s) {
    while (*s) uart_tx(*s++);
}

void uart_println(const char* s) {
    uart_print(s);
    uart_tx('\r');
    uart_tx('\n');
}

void uart_print_num(int16_t n) {
    if (n < 0) {
        uart_tx('-');
        n = -n;
    }
    if (n == 0) {
        uart_tx('0');
        return;
    }
    char buf[8];
    int8_t i = 0;
    while (n > 0 && i < 7) {
        buf[i++] = '0' + (n % 10);
        n /= 10;
    }
    while (i > 0) uart_tx(buf[--i]);
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

/* ===================================================================== */
/* COMMAND                                                               */
/* ===================================================================== */

void processCommand(void) {
    if (!commandReady) return;
    
    char cmd[RX_BUFFER_SIZE];
    cli();
    strcpy(cmd, (const char*)rxBuffer);
    commandReady = false;
    sei();
    
    uart_print("CMD: ");
    uart_println(cmd);
    
    if (strncmp(cmd, "SPD:", 4) == 0) {
        int speed = atoi(cmd + 4);
        if (speed < 0) speed = 0;
        if (speed > 255) speed = 255;
        targetSpeed = (uint8_t)speed;
        brakeActive = false;
        isMoving = (targetSpeed > 0);
        
        uart_print("SET SPD=");
        uart_print_num(targetSpeed);
        uart_println("");
    }
    else if (strcmp(cmd, "STOP") == 0) {
        brakeActive = true;
        isMoving = false;
        targetSpeed = 0;
        uart_println("BRAKE ON");
    }
    else if (strcmp(cmd, "GO") == 0) {
        brakeActive = false;
        isMoving = true;
        uart_println("GO");
    }
    else {
        uart_println("UNKNOWN CMD");
    }
}

/* ===================================================================== */
/* LCD                                                                   */
/* ===================================================================== */

void lcd_pulse(void) {
    PORTD |= (1 << LCD_E_PIN);
    _delay_us(2);
    PORTD &= ~(1 << LCD_E_PIN);
    _delay_us(100);
}

void lcd_nibble(uint8_t n) {
    PORTB &= ~((1 << LCD_D4) | (1 << LCD_D5));
    PORTC &= ~((1 << LCD_D6) | (1 << LCD_D7));
    if (n & 0x01) PORTB |= (1 << LCD_D4);
    if (n & 0x02) PORTB |= (1 << LCD_D5);
    if (n & 0x04) PORTC |= (1 << LCD_D6);
    if (n & 0x08) PORTC |= (1 << LCD_D7);
    lcd_pulse();
}

void lcd_byte(uint8_t v, bool data) {
    if (data) PORTD |= (1 << LCD_RS_PIN);
    else      PORTD &= ~(1 << LCD_RS_PIN);
    _delay_us(5);
    lcd_nibble((v >> 4) & 0x0F);
    lcd_nibble(v & 0x0F);
    _delay_us(50);
}

void lcd_cmd(uint8_t c) {
    lcd_byte(c, false);
    if (c == 0x01 || c == 0x02) _delay_ms(2);
}

void lcd_char(char c) {
    lcd_byte((uint8_t)c, true);
}

void lcd_str(const char *s) {
    while (*s) lcd_char(*s++);
}

void lcd_pos(uint8_t row, uint8_t col) {
    lcd_cmd(0x80 | ((row ? 0x40 : 0x00) + col));
}

void lcd_clear(void) {
    lcd_cmd(0x01);
    _delay_ms(2);
}

void lcd_num(int16_t n) {
    if (n < 0) { lcd_char('-'); n = -n; }
    if (n == 0) { lcd_char('0'); return; }
    char buf[8];
    int8_t i = 0;
    while (n > 0 && i < 7) { buf[i++] = '0' + (n % 10); n /= 10; }
    while (i > 0) lcd_char(buf[--i]);
}

void lcd_init(void) {
    DDRD |= (1 << LCD_RS_PIN) | (1 << LCD_E_PIN);
    DDRB |= (1 << LCD_D4) | (1 << LCD_D5);
    DDRC |= (1 << LCD_D6) | (1 << LCD_D7);
    
    PORTD &= ~((1 << LCD_RS_PIN) | (1 << LCD_E_PIN));
    PORTB &= ~((1 << LCD_D4) | (1 << LCD_D5));
    PORTC &= ~((1 << LCD_D6) | (1 << LCD_D7));
    
    _delay_ms(50);
    lcd_nibble(0x03); _delay_ms(5);
    lcd_nibble(0x03); _delay_ms(1);
    lcd_nibble(0x03); _delay_ms(1);
    lcd_nibble(0x02); _delay_ms(1);
    
    lcd_cmd(0x28); _delay_ms(1);
    lcd_cmd(0x0C); _delay_ms(1);
    lcd_cmd(0x06); _delay_ms(1);
    lcd_cmd(0x01); _delay_ms(2);
}

/* ===================================================================== */
/* MOTOR                                                                 */
/* ===================================================================== */

void motor_init(void) {
    DDRD |= (1 << IN1) | (1 << IN2) | (1 << IN3) | (1 << IN4);
    DDRB |= (1 << ENA) | (1 << ENB);
    PORTD &= ~((1 << IN1) | (1 << IN2) | (1 << IN3) | (1 << IN4));
    OCR1A = 0;
    OCR1B = 0;
    TCCR1A = (1 << WGM10) | (1 << COM1A1) | (1 << COM1B1);
    TCCR1B = (1 << WGM12) | (1 << CS11);
}

void motor_stop(void) {
    OCR1A = 0;
    OCR1B = 0;
    PORTD &= ~((1 << IN1) | (1 << IN2) | (1 << IN3) | (1 << IN4));
    currentSpeed = 0;
    currentDirection = DIR_STOP;
}

void motor_fwd(uint8_t spd) {
    PORTD |= (1 << IN1);
    PORTD &= ~(1 << IN2);
    PORTD |= (1 << IN3);
    PORTD &= ~(1 << IN4);
    OCR1A = spd;
    OCR1B = spd;
    currentSpeed = spd;
    currentDirection = DIR_FWD;
}

void motor_right(uint8_t spd) {
    PORTD |= (1 << IN1);
    PORTD &= ~(1 << IN2);
    PORTD &= ~(1 << IN3);
    PORTD |= (1 << IN4);
    OCR1A = spd;
    OCR1B = spd;
    currentSpeed = spd;
    currentDirection = DIR_RIGHT;
}

void motor_left(uint8_t spd) {
    PORTD &= ~(1 << IN1);
    PORTD |= (1 << IN2);
    PORTD |= (1 << IN3);
    PORTD &= ~(1 << IN4);
    OCR1A = spd;
    OCR1B = spd;
    currentSpeed = spd;
    currentDirection = DIR_LEFT;
}

/* ===================================================================== */
/* ULTRASONIC                                                            */
/* ===================================================================== */

void ultrasonic_init(void) {
    DDRB |= (1 << TRIG_PIN);
    DDRB &= ~(1 << ECHO_PIN);
    PORTB &= ~(1 << TRIG_PIN);
    PORTB &= ~(1 << ECHO_PIN);
}

float measure_dist(void) {
    uint32_t dur = 0;
    uint32_t tout;

    PORTB &= ~(1 << TRIG_PIN);
    _delay_us(2);
    PORTB |= (1 << TRIG_PIN);
    _delay_us(10);
    PORTB &= ~(1 << TRIG_PIN);

    tout = 30000UL;
    while (!(PINB & (1 << ECHO_PIN)) && tout--) { _delay_us(1); }
    if (tout == 0) return 999.0f;

    dur = 0;
    tout = 30000UL;
    while ((PINB & (1 << ECHO_PIN)) && tout--) { _delay_us(1); dur++; }
    if (tout == 0) return 999.0f;

    float d = (dur * 0.0343f) / 2.0f;
    if (d > 400.0f) d = 400.0f;
    return d;
}

/* ===================================================================== */
/* OBSTACLE AVOIDANCE                                                    */
/* ===================================================================== */

void turn_r90(void) {
    uart_println("TURN R90");
    motor_right(TURN_SPEED);
    delay_ms(TURN_90_TIME_MS);
    motor_stop();
    delay_ms(SEARCH_DELAY_MS);
}

void turn_l180(void) {
    uart_println("TURN L180");
    motor_left(TURN_SPEED);
    delay_ms(TURN_180_TIME_MS);
    motor_stop();
    delay_ms(SEARCH_DELAY_MS);
}

void spin180(void) {
    uart_println("SPIN 180");
    motor_right(TURN_SPEED);
    delay_ms(TURN_180_TIME_MS);
    motor_stop();
    delay_ms(SEARCH_DELAY_MS);
}

uint8_t searchDir(void) {
    float d;
    
    uart_println("SEARCH: RIGHT");
    turn_r90();
    d = measure_dist();
    uart_print("R=");
    uart_print_num((int16_t)d);
    uart_println("cm");
    if (d > OBSTACLE_THRESHOLD_CM) return DIR_RIGHT;
    
    uart_println("SEARCH: LEFT");
    turn_l180();
    d = measure_dist();
    uart_print("L=");
    uart_print_num((int16_t)d);
    uart_println("cm");
    if (d > OBSTACLE_THRESHOLD_CM) return DIR_LEFT;
    
    uart_println("SEARCH: BACK");
    spin180();
    d = measure_dist();
    uart_print("B=");
    uart_print_num((int16_t)d);
    uart_println("cm");
    if (d > OBSTACLE_THRESHOLD_CM) return DIR_FWD;
    
    return DIR_STOP;
}

void handleObs(void) {
    uart_println("!! OBSTACLE !!");
    motor_stop();
    delay_ms(200);
    
    uint8_t dir = searchDir();
    
    if (brakeActive) {
        uart_println("BRAKE ACTIVE");
        isMoving = false;
        return;
    }
    
    if (dir == DIR_STOP) {
        uart_println("ALL BLOCKED");
        isMoving = false;
    } else {
        uart_println("CLEAR FOUND");
        motor_fwd(targetSpeed);
    }
}

/* ===================================================================== */
/* MAIN                                                                  */
/* ===================================================================== */

int main(void) {
    // Init
    motor_init();
    ultrasonic_init();
    uart_init();
    lcd_init();
    timer0_init();
    
    sei();
    
    // Startup
    lcd_clear();
    lcd_str("DEBUG MODE");
    
    uart_println("");
    uart_println("================");
    uart_println("ADAS DEBUG v1.0");
    uart_println("================");
    uart_println("CMD: SPD:xxx STOP GO");
    uart_println("");
    
    delay_ms(1000);
    
    while (1) {
        loopCounter++;
        
        // 1. Commands
        processCommand();
        
        // 2. Sensor
        currentDistance = measure_dist();
        
        // 3. Debug print every 20 loops (~1 sec)
        if (loopCounter % 20 == 0) {
            uart_print("[");
            uart_print_num(loopCounter);
            uart_print("] D=");
            uart_print_num((int16_t)currentDistance);
            uart_print(" SPD=");
            uart_print_num(currentSpeed);
            uart_print(" TGT=");
            uart_print_num(targetSpeed);
            uart_print(" MOV=");
            uart_print_num(isMoving);
            uart_print(" BRK=");
            uart_print_num(brakeActive);
            uart_println("");
        }
        
        // 4. Control
        if (brakeActive) {
            if (currentSpeed > 0) {
                uart_println("BRAKE->STOP");
                motor_stop();
            }
        }
        else if (isMoving && targetSpeed > 0) {
            if (currentDistance <= OBSTACLE_THRESHOLD_CM) {
                handleObs();
            }
            else if (currentSpeed == 0) {
                uart_println("START FWD");
                motor_fwd(targetSpeed);
            }
        }
        else {
            if (currentSpeed > 0) {
                uart_println("IDLE->STOP");
                motor_stop();
            }
        }
        
        // 5. LCD update every 500ms
        if (millis() - lastDataSendTime >= DATA_SEND_INTERVAL_MS) {
            lastDataSendTime = millis();
            
            lcd_pos(0, 0);
            lcd_str("D:");
            lcd_num((int16_t)currentDistance);
            lcd_str("cm S:");
            lcd_num(currentSpeed);
            lcd_str("  ");
            
            lcd_pos(1, 0);
            if (brakeActive) lcd_str("BRAKE ");
            else if (isMoving) lcd_str("RUN   ");
            else lcd_str("IDLE  ");
            
            if (currentDistance <= OBSTACLE_THRESHOLD_CM) {
                lcd_str("OBST!");
            } else {
                lcd_str("OK   ");
            }
        }
        
        delay_ms(50);
    }

    return 0;
}