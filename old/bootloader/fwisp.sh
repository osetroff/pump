./avrdude -v -C./avrdude.conf -patmega328p -cstk500v1 -b19200 -P/dev/ttyUSB0 -Uflash:w:fw.hex:i
