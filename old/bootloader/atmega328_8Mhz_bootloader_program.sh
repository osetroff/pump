#bootloader and fuses
"/tmp/arduino/hardware/tools/avr/bin/avrdude" "-C/tmp/arduino/hardware/tools/avr/etc/avrdude.conf" -v -patmega328p -cstk500v1 "-P/dev/ttyUSB0" -b19200 "-Uflash:w:optibootef_38400_8.hex:i" -Uefuse:w:0xff:m -Ulfuse:w:0xe2:m -Uhfuse:w:0xd2:m 
# -Ulock:w:0x0F:m
#"/tmp/arduino/hardware/tools/avr/bin/avrdude" "-C/tmp/arduino/hardware/tools/avr/etc/avrdude.conf" -v -D -patmega328p -carduino "-P/dev/ttyUSB0" -b38400 "-Uflash:w:/tmp/test.hex:i"