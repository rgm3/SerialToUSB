#
#             LUFA Library
#     Copyright (C) Dean Camera, 2015.
#
#  dean [at] fourwalledcubicle [dot] com
#           www.lufa-lib.org
#
# --------------------------------------
#         LUFA Project Makefile.
# --------------------------------------

# Run "make help" for target help.

MCU          = atmega32u4
ARCH         = AVR8
BOARD        = ADAFRUITU4
F_CPU        = 16000000
F_USB        = $(F_CPU)
OPTIMIZATION = s
TARGET       = SerialToUSB
SRC          = $(TARGET).c Descriptors.c $(LUFA_SRC_USB) $(LUFA_SRC_USBCLASS)
LUFA_PATH    = LUFA
CC_FLAGS     = -DUSE_LUFA_CONFIG_HEADER -IConfig/ -ffunction-sections -fdata-sections
LD_FLAGS     =

AVRDUDE_PROGRAMMER = avr109
AVRDUDE_PORT       = /dev/ttyACM0
BAUD         = 19200
COMPILE      = avr-gcc -Wall -Os -ffunction-sections -fdata-sections -DF_CPU=$(F_CPU) -mmcu=$(MCU)

# Default target
all:

# Include LUFA build script makefiles
include $(LUFA_PATH)/Build/lufa_core.mk
include $(LUFA_PATH)/Build/lufa_sources.mk
include $(LUFA_PATH)/Build/lufa_build.mk
include $(LUFA_PATH)/Build/lufa_cppcheck.mk
include $(LUFA_PATH)/Build/lufa_doxygen.mk
include $(LUFA_PATH)/Build/lufa_dfu.mk
include $(LUFA_PATH)/Build/lufa_hid.mk
include $(LUFA_PATH)/Build/lufa_avrdude.mk
include $(LUFA_PATH)/Build/lufa_atprogram.mk

usb:
	ls /dev/ttyACM*
	
xbuild:
	$(COMPILE) -c $(SRC) -o $(TARGET).o
	$(COMPILE) -o $(TARGET).elf $(TARGET).o
	avr-objcopy -j .text -j .data -O ihex $(TARGET).elf $(TARGET).hex
	avr-size --format=avr --mcu=$(MCU) $(TARGET).elf

upload:
	avrdude -v -p $(MCU) -c $(PROGRAMMER) -P $(PORT) -U flash:w:$(FILENAME).hex:i 

xclean:
	rm -f *.o
	rm -f *.elf
	rm -f *.hex

# Listing of phony targets.
#.PHONY : all usb upload clean
