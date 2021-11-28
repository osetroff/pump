./avrdude -v -C./avrdude.conf -pm8 -cstk500v1 -b19200 -P/dev/ttyUSB0 -U hfuse:r:-:i -U lfuse:r:-:i -U efuse:r:-:i

