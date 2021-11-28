./avrdude -v -C./avrdude.conf -F -D -pt85 -cusbasp -U lfuse:w:0xc2:m -U hfuse:w:0xd6:m -U efuse:w:0xff:m
./frfuses84.sh
