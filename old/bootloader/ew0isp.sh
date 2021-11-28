./avrdude -v -C./avrdude.conf -F -pm328p -cstk500v1 -b19200 -P/dev/ttyUSB0 -Ueeprom:w:0xFF,0xFF,0xFF,0xFF:m
