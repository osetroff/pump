./avrdude -v -C./avrdude.conf -pt84 -cstk500v1 -b19200 -P/dev/ttyUSB0 -Uflash:w:/tmp/fw.hex:i
