# AVR Makefile for ATmega328P

# Project
TARGET = main
SRC = main.c

# MCU Settings
MCU = atmega328p
F_CPU = 16000000UL

# Compiler
CC = avr-gcc
OBJCOPY = avr-objcopy
AVRDUDE = avrdude

# Flags
CFLAGS = -mmcu=$(MCU) -DF_CPU=$(F_CPU) -Os -Wall -std=c99
LDFLAGS = -mmcu=$(MCU)

# Programmer (Arduino Uno uses arduino programmer)
PROGRAMMER = arduino
PORT = /dev/ttyACM0
BAUD = 115200

# Targets
all: $(TARGET).hex

$(TARGET).elf: $(SRC)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $(TARGET).elf $(SRC)

$(TARGET).hex: $(TARGET).elf
	$(OBJCOPY) -O ihex -R .eeprom $(TARGET).elf $(TARGET).hex

flash: $(TARGET).hex
	$(AVRDUDE) -c $(PROGRAMMER) -p $(MCU) -P $(PORT) -b $(BAUD) -U flash:w:$(TARGET).hex

clean:
	rm -f *.elf *.hex

size: $(TARGET).elf
	avr-size --mcu=$(MCU) -C $(TARGET).elf

.PHONY: all flash clean size