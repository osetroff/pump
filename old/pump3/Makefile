TMP = /tmp/
PRJ = pump
MCU = atmega328p
CLK = 8000000L
PRG = -D -carduino -P/dev/ttyUSB0 -b38400
LFU = 0xE2
HFU = 0xD2
EFU = 0x07
SRC = $(PRJ).cpp
# where to look for external libraries (consisting of .c/.cpp files and .h files)
EXT =

# include path
INCLUDE := $(foreach dir, $(EXT), -I$(dir))
CFLAGS    = -Wall -Os -DF_CPU=$(CLK) -mmcu=$(MCU) $(INCLUDE)
CPPFLAGS = -Wno-unused-variable -Wno-unused-function -Wno-unused-value -Wno-return-local-addr

# executables
AVRDUDE = avrdude -c $(PRG) -p $(MCU)
OBJCOPY = avr-objcopy
OBJDUMP = avr-objdump
SIZE    = avr-size --format=avr --mcu=$(MCU)
CC      = avr-gcc

# generate list of objects
CFILES    = $(filter %.c, $(SRC))
EXTC     := $(foreach dir, $(EXT), $(wildcard $(dir)/*.c))
CPPFILES  = $(filter %.cpp, $(SRC))
EXTCPP   := $(foreach dir, $(EXT), $(wildcard $(dir)/*.cpp))
OBJ       = $(CFILES:.c=.o) $(EXTC:.c=.o) $(CPPFILES:.cpp=.o) $(EXTCPP:.cpp=.o)

# user targets
# compile all files
all: $(PRJ).hex

# test programmer connectivity
test:
	$(AVRDUDE) -v

# flash program to mcu
flash: all
	$(AVRDUDE) -Uflash:w:$(TMP)$(PRJ).hex:i
	mi

# write fuses to mcu
fuse:
	$(AVRDUDE) -U lfuse:w:$(LFU):m -U hfuse:w:$(HFU):m -U efuse:w:$(EFU):m

# generate disassembly files for debugging
disasm: $(PRJ).elf
	$(OBJDUMP) -d $(TMP)$(PRJ).elf

# remove compiled files
clean:
	rm -f $(TMP)*.hex $(TMP)*.elf $(TMP)*.o
	$(foreach dir, $(EXT), rm -f $(dir)/*.o;)

# other targets
# objects from c files
.c.o:
	$(CC) $(CFLAGS) -c $< -o $(TMP)$@

# objects from c++ files
.cpp.o:
	$(CC) $(CFLAGS) $(CPPFLAGS) -c $< -o $(TMP)$@

# elf file
$(PRJ).elf: $(OBJ)
#	$(CC) $(CFLAGS) -o $(TMP)$(PRJ).elf $(OBJ)
	$(CC) $(CFLAGS) -o $(TMP)$(PRJ).elf $(TMP)$(PRJ).o

# hex file
$(PRJ).hex: $(PRJ).elf
	rm -f $(TMP)$(PRJ).hex
	$(OBJCOPY) -j .text -j .data -O ihex $(TMP)$(PRJ).elf $(TMP)$(PRJ).hex
	$(SIZE) $(TMP)$(PRJ).elf
