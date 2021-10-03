"/tmp/arduino/hardware/tools/avr/bin/avrdude" "-C/tmp/arduino/hardware/tools/avr/etc/avrdude.conf" -v -patmega328p -cstk500v1 "-P/dev/ttyUSB0" -b19200 -Ulfuse:r:-:i -Uhfuse:r:-:i -Uefuse:r:-:i
