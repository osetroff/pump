./avrdude -v -C./avrdude.conf -D -pm328p -carduino -b38400 -P/dev/ttyUSB0 -Uflash:w:fw.hex:i
