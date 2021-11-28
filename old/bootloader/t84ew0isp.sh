./avrdude -v -C./avrdude.conf -pt84 -cstk500v1 -b19200 -P/dev/ttyUSB0 -Ueeprom:w:0xFF,0xFF,0xFF,0xFF:m
