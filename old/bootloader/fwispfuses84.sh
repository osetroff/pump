./avrdude -v -C./avrdude.conf -pt84 -cstk500v1 -b19200 -P/dev/ttyUSB0 -U lfuse:w:0xc2:m -U hfuse:w:0xd6:m -U efuse:w:0xff:m
./avrdude -v -C./avrdude.conf -pt84 -cstk500v1 -b19200 -P/dev/ttyUSB0 -U lfuse:r:-:i
