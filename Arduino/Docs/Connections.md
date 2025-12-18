
## Connections
---
### Power

| From            | To                               |
| --------------- | -------------------------------- |
| Battery +       | Arduino **VIN** (or barrel jack) |
| Battery −       | Arduino **GND**                  |
| Battery +       | L298N **+12V / Vs**              |
| Battery −       | L298N **GND**                    |
| Arduino **GND** | L298N **GND** (common ground)    |
---
### L298N → Arduino (matches the code)
---
| L298N | Arduino Uno |
| ----- | ----------: |
| ENA   |    D9 (PWM) |
| IN1   |          D3 |
| IN2   |          D4 |
| ENB   |   D10 (PWM) |
| IN3   |          D5 |
| IN4   |          D6 |
| GND   |         GND |
---
Motors:

* **OUT1/OUT2** → left motors (2 motors in parallel)
* **OUT3/OUT4** → right motors (2 motors in parallel)
---
### Ultrasonic HC-SR04

| HC-SR04 | Arduino |
| ------- | ------: |
| VCC     |      5V |
| GND     |     GND |
| TRIG    |     D12 |
| ECHO    |     D11 |

## LCD Control + Data (4-bit Mode)

| LCD Pin | Arduino Connection |
|--------|---------------------|
| RS     | D2                  | 
| RW     | GND                 | 
| E      | D7                  | 
| D0–D3  | Not connected       | 
| D4     | D8                  | 
| D5     | D13                 | 
| D6     | A2                  | 
| D7     | A3                  | 

