./avrdude -v -C./avrdude.conf -patmega328p -cstk500v1 -b19200 -P/dev/ttyUSB0 -U lfuse:w:0xff:m -U hfuse:w:0xd6:m -U efuse:w:0x05:m
./avrdude -v -C./avrdude.conf -patmega328p -cstk500v1 -b19200 -P/dev/ttyUSB0 -U lfuse:r:-:i
