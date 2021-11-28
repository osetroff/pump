#fixpath = $(1)
#GCCROOT =
AVRDUDE_CONF = -Cavrdude.conf
#-O1-3 make it bigger
OPTIMIZE = -Os -fno-inline-small-functions -fno-split-wide-types -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -fno-tree-scev-cprop -mrelax -maccumulate-args
#DEFS       =
#LIBS       =
#-v
CC         = ./avr-gcc 
override CFLAGS        = -g -Wall $(OPTIMIZE) -mmcu=$(MCU_TARGET) -DF_CPU=$(AVR_FREQ) $(DEFS)
override LDFLAGS       = $(LDSECTIONS) -Wl,--relax -nostartfiles -nostdlib
#-Wl,--gc-sections
OBJCOPY        = ./avr-objcopy
OBJDUMP        = ./avr-objdump
SIZE           = ./avr-size

# Make command-line Options.
# Permit commands like "make atmega328 LED_START_FLASHES=10" to pass the
# appropriate parameters ("-DLED_START_FLASHES=10") to gcc

ifdef BAUD_RATE
BAUD_RATE_CMD = -DBAUD_RATE=$(BAUD_RATE)
dummy = FORCE
else
BAUD_RATE_CMD = -DBAUD_RATE=38400
endif

# BIG_BOOT: Include extra features, up to 1K.

ifdef BIGBOOT
BIGBOOT_CMD = -DBIGBOOT=0
dummy = FORCE
else
BIGBOOT_CMD = -DBIGBOOT=0
endif

ifdef SOFT_UART
SOFT_UART_CMD = -DSOFT_UART=0
dummy = FORCE
endif

ifdef SINGLESPEED
SSCMD = -DSINGLESPEED=1
endif

ifdef FMHZ
AFREQ=$(FMHZ)
FREQ = $(FMHZ)000000L
else
FREQ = 16000000L
AFREQ=16
endif


SUPPORT_EEPROM_CMD = -DSUPPORT_EEPROM
dummy = FORCE

COMMON_OPTIONS = $(BAUD_RATE_CMD) $(BIGBOOT_CMD)
COMMON_OPTIONS += $(SUPPORT_EEPROM_CMD)
COMMON_OPTIONS += $(SOFT_UART_CMD) $(SSCMD)

atmega328: TARGET = atmega328
atmega328: MCU_TARGET = atmega328p
atmega328: CFLAGS += $(COMMON_OPTIONS)
atmega328: AVR_FREQ ?= $(FREQ)
atmega328: LDSECTIONS  = -Wl,--section-start=.text=0x7e00 -Wl,--section-start=.version=0x7ffe
atmega328: optibootef_$(BAUD_RATE)_$(AFREQ).hex
atmega328: optibootef_$(BAUD_RATE)_$(AFREQ).lst

FORCE:

baudcheck: FORCE
	- @$(CC) $(CFLAGS) -E baudcheck.c -o baudcheck.tmp.sh
	- chmod 777 ./baudcheck.tmp.sh
	- /bin/sh ./baudcheck.tmp.sh


%.elf: optibootmy.o
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LIBS)
	$(SIZE) $@

clean:
	rm -rf *.o *.elf *.lst *.map *.sym *.lss *.eep *.srec *.bin *.hex

%.lst: %.elf
	- $(OBJDUMP) -h -S $< > $@
	- chmod 777 $@

%.hex: %.elf
	- $(OBJCOPY) -j .text -j .data -j .version --set-section-flags .version=alloc,load -O ihex $< $@
	- chmod 777 $@
	- rm /tmp/arduino/hardware/arduino/avr/bootloaders/optiboot/$@
	- cp $@ /tmp/arduino/hardware/arduino/avr/bootloaders/optiboot/

%.srec: %.elf
	$(OBJCOPY) -j .text -j .data -j .version --set-section-flags .version=alloc,load -O srec $< $@

%.bin: %.elf
	$(OBJCOPY) -j .text -j .data -j .version --set-section-flags .version=alloc,load -O binary $< $@

r:
	(rm -f *.o; make)
