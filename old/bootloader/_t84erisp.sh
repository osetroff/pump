./avrdude -v -C./avrdude.conf -pt84 -cstk500v1 -b38400 -P/dev/ttyUSB0 -Ueeprom:r:/tmp/er.hex:i
