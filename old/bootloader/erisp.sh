./avrdude -v -C./avrdude.conf -patmega328p -cstk500v1 -b19200 -P/dev/ttyUSB0 -Ueeprom:r:e.hex:i
