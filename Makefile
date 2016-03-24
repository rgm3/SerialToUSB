MCU        = atmega32u4
BOARD      = ADAFRUITU4
F_CPU      = 16000000
PROGRAMMER = avr109
PORT       = /dev/ttyACM0
BAUD       = 19200
FILENAME   = SerialToUSB
COMPILE    = avr-gcc -Wall -Os -ffunction-sections -fdata-sections -DF_CPU=$(F_CPU) -mmcu=$(MCU)

all: clean build usb upload

usb:
	ls /dev/ttyACM*
	
build:
	$(COMPILE) -c $(FILENAME).c -o $(FILENAME).o
	$(COMPILE) -o $(FILENAME).elf $(FILENAME).o
	avr-objcopy -j .text -j .data -O ihex $(FILENAME).elf $(FILENAME).hex
	avr-size --format=avr --mcu=$(MCU) $(FILENAME).elf

upload:
	avrdude -v -p $(MCU) -c $(PROGRAMMER) -P $(PORT) -U flash:w:$(FILENAME).hex:i 

clean:
	rm -f *.o
	rm -f *.elf
	rm -f *.hex

# Listing of phony targets.
.PHONY : all usb upload clean
