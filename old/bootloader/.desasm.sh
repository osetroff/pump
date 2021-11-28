#just change dir inside path /tmp/Stino_build/...

rm /tmp/desasmtxt.txt
/tmp/arduino/hardware/tools/avr/bin/avr-objdump -S /tmp/Stino_build/ddv/*.elf >/tmp/desasmtxt.txt
chmod 666 /tmp/desasmtxt.txt
