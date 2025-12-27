# # Compiler settings
# MCU = atmega328p
# F_CPU = 16000000UL
# CC = avr-gcc
# CFLAGS = -mmcu=$(MCU) -DF_CPU=$(F_CPU) -Os -Wall -std=c99
# OBJCPY = avr-objcopy
# OBJDUMP = avr-objdump
# SIZE = avr-size
# AVRDUDE = avrdude

# PROGRAMMER = arduino
# PORT = COM3
# BAUD = 115200

# SRC_DIR = src
# INC_DIR = include
# OBJ_DIR = obj

# # Output name
# TARGET = adas

# # Files
# SRC = $(wildcard $(SRC_DIR)/APP/*.c) \
#       $(wildcard $(SRC_DIR)/HAL/*.c) \
#       $(wildcard $(SRC_DIR)/MCAL/*.c)

# OBJ = $(SRC:$(SRC_DIR)/%.c=$(OBJ_DIR)/%.o)

# # Targets
# all: $(TARGET).hex size

# # Rule to compile .c to .o files
# $(OBJ_DIR)/%.o: $(SRC_DIR)/%.c
# 	@if not exist "$(OBJ_DIR)\APP" mkdir "$(OBJ_DIR)\APP"
# 	@if not exist "$(OBJ_DIR)\HAL" mkdir "$(OBJ_DIR)\HAL"
# 	@if not exist "$(OBJ_DIR)\MCAL" mkdir "$(OBJ_DIR)\MCAL"
# 	$(CC) $(CFLAGS) -I$(INC_DIR) -c $< -o $@

# # Link object files to create ELF file
# $(TARGET).elf: $(OBJ)
# 	$(CC) $(CFLAGS) -o $@ $^

# # Convert ELF file to HEX file
# $(TARGET).hex: $(TARGET).elf
# 	$(OBJCPY) -O ihex -R .eeprom $< $@

# # Display size information
# size: $(TARGET).elf
# 	$(SIZE) --mcu=$(MCU) -C $<

# # Flash via Arduino bootloader (USB)
# flash: $(TARGET).hex
# 	$(AVRDUDE) -c $(PROGRAMMER) -p m328p -P $(PORT) -b $(BAUD) -U flash:w:$<:i

# # Flash via USBasp programmer
# flash-usbasp: $(TARGET).hex
# 	$(AVRDUDE) -c usbasp -p m328p -U flash:w:$<:i

# # Generate disassembly listing
# disasm: $(TARGET).elf
# 	$(OBJDUMP) -d -S $< > $(TARGET).lst

# # Clean up all generated files
# clean:
# 	@if exist "$(OBJ_DIR)" rmdir /s /q "$(OBJ_DIR)"
# 	@if exist "$(TARGET).elf" del /q "$(TARGET).elf"
# 	@if exist "$(TARGET).hex" del /q "$(TARGET).hex"
# 	@if exist "$(TARGET).lst" del /q "$(TARGET).lst"

# .PHONY: all flash flash-usbasp size disasm clean
