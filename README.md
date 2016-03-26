# Serial mouse to USB HID

The goal is to resurrect a beloved Microsoft EasyBall trackball ([vid][1]), enabling its use
with anything that understands USB HID without configuration or drivers.  Even works on Android
phones, if you've got a USB OTG cable.

[1]: https://www.dropbox.com/sc/ofrfyzh1wog5gd7/AAAw4_ZpiT6ACZIX_4lHUmnla "EasyBall sending data"


# Programming the Adafruit ATmega32u4 breakout

## Stop Linux from probing the CDC device (bootloader)

Something on Linux Mint 17.3 keeps the device `/dev/ttyACM0` busy for a few seconds when the
bootloader is running.  To stop that, first determine the vendor and product magic numbers
by examining the output of `lsusb` and then create `/etc/udev/rules.d/90-adafruit-32u4.rules`
file containing:

````
SUBSYSTEM=="usb", ATTR{idVendor}=="239a", ATTR{idProduct}=="0001", MODE="0660", GROUP="dialout", ENV{ID_MM_DEVICE_IGNORE}="1"
````

Without this, there's a narrow window between when the serial port is available for programming
and when the code rules.

Your group may differ depending on your distribution.  For more information, refer to
[LUFA Library Support List > Re: VirtualSerial: Device or resource busy](https://groups.google.com/forum/#!topic/lufa-support/CP9cy2bc8yo).

## Dependencies

## LUFA

* git@github.com:abcminiuser/lufa.git
* http://www.github.com/abcminiuser/lufa/archive/LUFA-151115.zip

## Build Tools
````
# on Mac OS X with MacPorts
sudo port install avrdude avr-gcc avr-libc
````

# link dump
* Image credit is CC by-sa 3.0, from lady ada, https://learn.adafruit.com/assets/30449
* http://www.atmel.com/Images/Atmel-7766-8-bit-AVR-ATmega16U4-32U4_Summary.pdf
* http://www.atmel.com/images/atmel-7766-8-bit-avr-atmega16u4-32u4_datasheet.pdf
* ftp://www.zimmers.net/pub/cbm/documents/projects/interfaces/mouse/Mouse.html
* https://github.com/adafruit/Atmega32u4-Breakout-Board
* https://pdfserv.maximintegrated.com/en/ds/MAX3222-MAX3241.pdf
* http://www.fourwalledcubicle.com/AVRArticles.php
* http://paulbourke.net/dataformats/serialmouse/
* https://github.com/jackhumbert/qmk_firmware/blob/master/converter/serialmouse_usb/config.h
* https://github.com/rhaberkorn/tmk_keyboard/tree/master/converter/ascii_usb
* https://www.pjrc.com/teensy/td_uart.html
* http://manpages.ubuntu.com/manpages/dapper/man4/mouse.4.html
