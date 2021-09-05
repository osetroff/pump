avrdude -v -C/u/_home/_arduino/kot/r/_/arduino/hardware/tools/avr/bin/avrdude.conf -F -pm328p -carduino -b38400 -P/dev/ttyUSB0 -Ueeprom:w:eeprom_pump_addr.hex:i
