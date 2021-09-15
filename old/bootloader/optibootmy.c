//changed optiboot bootloader (m.sh and makefile you can see in the end of this file)
//1. added read/write to eeprom
//2. added read UBRR0L from eeprom addr 
#define eepromaddrubrr0l 0x0
//3. added read OSCCAL from eeprom addr
#define eepromaddrosccal 0x1
//4. added write to flash from program (c code and example are at the end of this file)

//=================
//how to program:
//(you can merge program.hex and bootloader.hex:
//delete last line ...1F from program.hex and add full bootloader.hex in the end of program.hex)
//1. through isp (GND,VCC,MISO,MOSI,SCK,pin10 to reset) -> 
//select "board",select "arduino as isp", then "burn bootloader" 
//(from "hardware/arduino/avr/bootloaders/optiboot/name.hex)
//2. throwgh bootloader (GND,VCC,RX,TX,DTR) -> 
//select "board", select "avrispmkII", then "upload" (from program.hex)
//
//er.sh 
//./avrdude -v -C./avrdude.conf -D -pm328p -carduino -b38400 -P/dev/ttyUSB0 -Ueeprom:r:e.hex:i
//erisp.sh 
//./avrdude -v -C./avrdude.conf -patmega328p -cstk500v1 -b19200 -P/dev/ttyUSB0 -Ueeprom:r:e.hex:i
//ew.sh
//./avrdude -v -C./avrdude.conf -D -pm328p -carduino -b38400 -P/dev/ttyUSB0 -Ueeprom:w:ew.hex:i
//ew0.sh
//./avrdude -v -C./avrdude.conf -F -pm328p -carduino -b38400 -P/dev/ttyUSB0 -Ueeprom:w:0x30,0x31,0x31:m
//ew0isp.sh
//./avrdude -v -C./avrdude.conf -F -pm328p -cstk500v1 -b19200 -P/dev/ttyUSB0 -Ueeprom:w:0x80,0x31:m
//ewisp.sh
//./avrdude -v -C./avrdude.conf -F -pm328p -cstk500v1 -b19200 -P/dev/ttyUSB0 -Ueeprom:w:ew.hex:i
//fr.sh
//./avrdude -v -C./avrdude.conf -D -pm328p -carduino -b38400 -P/dev/ttyUSB0 -Uflash:r:f.hex:i
//frisp.sh
//./avrdude -v -C./avrdude.conf -patmega328p -cstk500v1 -b19200 -P/dev/ttyUSB0 -Uflash:r:f.hex:i
//fw.sh
//./avrdude -v -C./avrdude.conf -D -pm328p -carduino -b38400 -P/dev/ttyUSB0 -Uflash:w:fw.hex:i
//fwisp.sh
//./avrdude -v -C./avrdude.conf -patmega328p -cstk500v1 -b19200 -P/dev/ttyUSB0 -Uflash:w:fw.hex:i
//lockrisp.sh
//./avrdude -v -C./avrdude.conf -patmega328p -cstk500v1 -b19200 -P/dev/ttyUSB0 -Ulock:r:lock.txt:r
//lockwisp.sh
//./avrdude -v -C./avrdude.conf -patmega328p -cstk500v1 -b19200 -P/dev/ttyUSB0 -Ulock:w:0x3C:m

/* fuses depend on bootloader size
bootsz1 bootsz0    bytes    start addr    hfuse
-----------------------------------------------
1       1          512      7E00          d6
1       0          1024     7C00          d4
0       1          2048     7800          d2
0       0          4096     7000          d0
*/

/* desasm .sh file
#just change dir inside path /tmp/Stino_build/...
rm /tmp/desasmtxt.txt
/tmp/arduino/hardware/tools/avr/bin/avr-objdump -S /tmp/Stino_build/osccal/
*.elf >/tmp/desasmtxt.txt
chmod 777 /tmp/desasmtxt.txt
*/

#define OPTIBOOT_MAJVER 5
#define OPTIBOOT_MINVER 1
#define MAKESTR(a) #a
#define MAKEVER(a, b) MAKESTR(a*256+b)
//2017 uncomment if does not work
/*
asm("  .section .version\n"
    "optiboot_version:  .word " MAKEVER(OPTIBOOT_MAJVER, OPTIBOOT_MINVER) "\n"
    "  .section .text\n");
*/



//================
// from stk500.h
// STK500 constants list, from AVRDUDE
#define STK_OK              0x10
#define STK_FAILED          0x11  // Not used
#define STK_UNKNOWN         0x12  // Not used
#define STK_NODEVICE        0x13  // Not used
#define STK_INSYNC          0x14  // ' '
#define STK_NOSYNC          0x15  // Not used
#define ADC_CHANNEL_ERROR   0x16  // Not used
#define ADC_MEASURE_OK      0x17  // Not used
#define PWM_CHANNEL_ERROR   0x18  // Not used
#define PWM_ADJUST_OK       0x19  // Not used
#define CRC_EOP             0x20  // 'SPACE'
#define STK_GET_SYNC        0x30  // '0'
#define STK_GET_SIGN_ON     0x31  // '1'
#define STK_SET_PARAMETER   0x40  // '@'
#define STK_GET_PARAMETER   0x41  // 'A'
#define STK_SET_DEVICE      0x42  // 'B'
#define STK_SET_DEVICE_EXT  0x45  // 'E'
#define STK_ENTER_PROGMODE  0x50  // 'P'
#define STK_LEAVE_PROGMODE  0x51  // 'Q'
#define STK_CHIP_ERASE      0x52  // 'R'
#define STK_CHECK_AUTOINC   0x53  // 'S'
#define STK_LOAD_ADDRESS    0x55  // 'U'
#define STK_UNIVERSAL       0x56  // 'V'
#define STK_PROG_FLASH      0x60  // '`'
#define STK_PROG_DATA       0x61  // 'a'
#define STK_PROG_FUSE       0x62  // 'b'
#define STK_PROG_LOCK       0x63  // 'c'
#define STK_PROG_PAGE       0x64  // 'd'
#define STK_PROG_FUSE_EXT   0x65  // 'e'
#define STK_READ_FLASH      0x70  // 'p'
#define STK_READ_DATA       0x71  // 'q'
#define STK_READ_FUSE       0x72  // 'r'
#define STK_READ_LOCK       0x73  // 's'
#define STK_READ_PAGE       0x74  // 't'
#define STK_READ_SIGN       0x75  // 'u'
#define STK_READ_OSCCAL     0x76  // 'v'
#define STK_READ_FUSE_EXT   0x77  // 'w'
#define STK_READ_OSCCAL_EXT 0x78  // 'x'





//#include <inttypes.h>
//#include <avr/io.h>
//#include "iom328p.h"
//#include <avr/pgmspace.h>
#include "pgmspace.h"
// We don't use <avr/wdt.h> as those routines have interrupt overhead we don't need.


//=============
// Function Prototypes 
/* The main function is in init9, which removes the interrupt vector table */
/* we don't need. It is also 'naked', which means the compiler does not    */
/* generate any entry or exit code itself. */
void pre_main(void) __attribute__ ((naked)) __attribute__ ((section (".init8")));
int main(void) __attribute__ ((OS_main)) __attribute__ ((section (".init9")));
void putch(char);
uint8_t getch(void);
void getNch(uint8_t); /* "static inline" is a compiler hint to reduce code size */
void verifySpace();
//AV: fix for 0 flashes
uint8_t getLen();
static void __attribute__((noinline)) do_spm(uint16_t address, uint8_t command, uint16_t data);
static void startapp();
#ifdef SOFT_UART
void uartDelay() __attribute__ ((naked));
#endif
//void appStart(uint8_t rstFlags) __attribute__ ((naked));

//=============
// from pin_defs.h
#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__) || defined(__AVR_ATmega88) || defined(__AVR_ATmega8__) || defined(__AVR_ATmega88__)
// Onboard LED is connected to pin PB5 in Arduino NG, Diecimila, and Duemilanove 
#define LED_DDR     DDRB
#define LED_PORT    PORTB
#define LED_PIN     PINB
#define LED         PINB5
// Ports for soft UART 
#ifdef SOFT_UART
#define UART_PORT   PORTD
#define UART_PIN    PIND
#define UART_DDR    DDRD
#define UART_TX_BIT 1
#define UART_RX_BIT 0
#endif
#endif
//
#if defined(__AVR_ATmega8__)
  //Name conversion R.Wiersma
  #define UCSR0A  UCSRA
  #define UDR0    UDR
  #define UDRE0   UDRE
  #define RXC0    RXC
  #define FE0           FE
  #define TIFR1   TIFR
  #define WDTCSR  WDTCR
#endif
// Luminet support 
#if defined(__AVR_ATtiny84__)
// Red LED is connected to pin PA4 
#define LED_DDR     DDRA
#define LED_PORT    PORTA
#define LED_PIN     PINA
#define LED         PINA4
// Ports for soft UART - left port only for now. TX/RX on PA2/PA3 
#ifdef SOFT_UART
#define UART_PORT   PORTA
#define UART_PIN    PINA
#define UART_DDR    DDRA
#define UART_TX_BIT 2
#define UART_RX_BIT 3
#endif
#endif
// Sanguino support 
#if defined(__AVR_ATmega644P__)
// Onboard LED is connected to pin PB0 on Sanguino 
#define LED_DDR     DDRB
#define LED_PORT    PORTB
#define LED_PIN     PINB
#define LED         PINB0
// Ports for soft UART 
#ifdef SOFT_UART
#define UART_PORT   PORTD
#define UART_PIN    PIND
#define UART_DDR    DDRD
#define UART_TX_BIT 1
#define UART_RX_BIT 0
#endif
#endif
// Mega support 
#if defined(__AVR_ATmega1280__)
//Onboard LED is connected to pin PB7 on Arduino Mega
#define LED_DDR     DDRB
#define LED_PORT    PORTB
#define LED_PIN     PINB
#define LED         PINB7
// Ports for soft UART
#ifdef SOFT_UART
#define UART_PORT   PORTE
#define UART_PIN    PINE
#define UART_DDR    DDRE
#define UART_TX_BIT 1
#define UART_RX_BIT 0
#endif
#endif

//=========================
// UART
#ifdef LUDICROUS_SPEED
#define BAUD_RATE 230400L
#endif
// set the UART baud rate defaults
#ifndef BAUD_RATE
#if F_CPU >= 8000000L
 #define BAUD_RATE   115200L // Highest rate Avrdude win32 will support
#else
  #if F_CPU >= 1000000L
   #define BAUD_RATE   9600L   // 19200 also supported, but with significant error
  #else
   #if F_CPU >= 128000L
    #define BAUD_RATE   4800L   // Good for 128kHz internal RC
   #else
    #define BAUD_RATE 1200L     // Good even at 32768Hz
   #endif
  #endif
#endif
#endif
//
#ifndef UART
#define UART 0
#endif
//
#define BAUD_SETTING (( (F_CPU + BAUD_RATE * 4L) / ((BAUD_RATE * 8L))) - 1 )
#define BAUD_ACTUAL (F_CPU/(8 * ((BAUD_SETTING)+1)))
#define BAUD_ERROR (( 100*(BAUD_RATE - BAUD_ACTUAL) ) / BAUD_RATE)
#if BAUD_ERROR >= 5
#error BAUD_RATE error greater than 5%
#elif BAUD_ERROR <= -5
#error BAUD_RATE error greater than -5%
#elif BAUD_ERROR >= 2
#warning BAUD_RATE error greater than 2%
#elif BAUD_ERROR <= -2
#warning BAUD_RATE error greater than -2%
#endif
//
#if 0
// Switch in soft UART for hard baud rates 
//I don't understand what this was supposed to accomplish, where the
//constant "280" came from, or why automatically (and perhaps unexpectedly)
//switching to a soft uart is a good thing, so I'm undoing this in favor
//of a range check using the same calc used to config the BRG...
#if (F_CPU/BAUD_RATE) > 280 // > 57600 for 16MHz
#ifndef SOFT_UART
#define SOFT_UART
#endif
#endif
#else // 0
#if (F_CPU + BAUD_RATE * 4L) / (BAUD_RATE * 8L) - 1 > 250
#error Unachievable baud rate (too slow) BAUD_RATE
#endif // baud rate slow check
#if (F_CPU + BAUD_RATE * 4L) / (BAUD_RATE * 8L) - 1 < 3
#error Unachievable baud rate (too fast) BAUD_RATE
#endif // baud rate fastn check
#endif
//Handle devices with up to 4 uarts (eg m1280.)  Rather inelegantly.
//Note that mega8/m32 still needs special handling, because ubrr is handled
//differently.
#if UART == 0
# define UART_SRA UCSR0A
# define UART_SRB UCSR0B
# define UART_SRC UCSR0C
# define UART_SRL UBRR0L
# define UART_UDR UDR0
#elif UART == 1
#if !defined(UDR1)
#error UART == 1, but no UART1 on device
#endif
# define UART_SRA UCSR1A
# define UART_SRB UCSR1B
# define UART_SRC UCSR1C
# define UART_SRL UBRR1L
# define UART_UDR UDR1
#elif UART == 2
#if !defined(UDR2)
#error UART == 2, but no UART2 on device
#endif
# define UART_SRA UCSR2A
# define UART_SRB UCSR2B
# define UART_SRC UCSR2C
# define UART_SRL UBRR2L
# define UART_UDR UDR2
#elif UART == 3
#if !defined(UDR1)
#error UART == 3, but no UART3 on device
#endif
# define UART_SRA UCSR3A
# define UART_SRB UCSR3B
# define UART_SRC UCSR3C
# define UART_SRL UBRR3L
# define UART_UDR UDR3
#endif

//=========================
// Watchdog settings 
#define watchdogConfig(x) WDTCSR = _BV(WDCE) | _BV(WDE); WDTCSR = x
#define WATCHDOG_OFF    (0)
#define WATCHDOG_16MS   (_BV(WDE))
#define WATCHDOG_32MS   (_BV(WDP0) | _BV(WDE))
#define WATCHDOG_64MS   (_BV(WDP1) | _BV(WDE))
#define WATCHDOG_125MS  (_BV(WDP1) | _BV(WDP0) | _BV(WDE))
#define WATCHDOG_250MS  (_BV(WDP2) | _BV(WDE))
#define WATCHDOG_500MS  (_BV(WDP2) | _BV(WDP0) | _BV(WDE))
#define WATCHDOG_1S     (_BV(WDP2) | _BV(WDP1) | _BV(WDE))
#define WATCHDOG_2S     (_BV(WDP2) | _BV(WDP1) | _BV(WDP0) | _BV(WDE))
#ifndef __AVR_ATmega8__
#define WATCHDOG_4S     (_BV(WDP3) | _BV(WDE))
#define WATCHDOG_8S     (_BV(WDP3) | _BV(WDP0) | _BV(WDE))
#endif



//================
// from boot.h 
// <avr/boot.h> uses sts instructions, but this version uses out instructions
// This saves cycles and program memory.
#ifdef ASRE
#define __COMMON_ASRE   ASRE
#else
#define __COMMON_ASRE   RWWSRE
#endif
#define __BOOT_PAGE_ERASE         (_BV(__SPM_ENABLE) | _BV(PGERS))
#define __BOOT_PAGE_WRITE         (_BV(__SPM_ENABLE) | _BV(PGWRT))
#define __BOOT_PAGE_FILL          _BV(__SPM_ENABLE)
#define __BOOT_RWW_ENABLE         (_BV(__SPM_ENABLE) | _BV(__COMMON_ASRE))
/* Check for SPM Control Register in processor. */
#if defined (SPMCSR)
#  define __SPM_REG    SPMCSR
#elif defined (SPMCR)
#  define __SPM_REG    SPMCR
#else
#  error AVR processor does not provide bootloader support!
#endif
/* Check for SPM Enable bit. */
#if defined(SPMEN)
#  define __SPM_ENABLE  SPMEN
#elif defined(SELFPRGEN)
#  define __SPM_ENABLE  SELFPRGEN
#else
#  error Cannot find SPM Enable bit definition!
#endif
//Check if the SPM instruction is busy
#define boot_spm_busy()               (__SPM_REG & (uint8_t)_BV(__SPM_ENABLE))
//Wait while the SPM instruction is busy. */
#define boot_spm_busy_wait()          do{}while(boot_spm_busy())
#define __boot_rww_enable_short()                      \
(__extension__({                                 \
    __asm__ __volatile__(                         \
        "out %0, %1\n\t"                         \
        "spm\n\t"                                \
        :                                        \
        : "i" (_SFR_IO_ADDR(__SPM_REG)),        \
          "r" ((uint8_t)__BOOT_RWW_ENABLE)       \
    );                                           \
}))
#define __boot_rww_enable()                      \
(__extension__({                                 \
    __asm__ __volatile__(                         \
        "sts %0, %1\n\t"                         \
        "spm\n\t"                                \
        :                                        \
        : "i" (_SFR_MEM_ADDR(__SPM_REG)),        \
          "r" ((uint8_t)__BOOT_RWW_ENABLE)       \
    );                                           \
}))
#define __boot_rww_enable_alternate()            \
(__extension__({                                 \
    __asm__ __volatile__(                         \
        "sts %0, %1\n\t"                         \
        "spm\n\t"                                \
        ".word 0xffff\n\t"                       \
        "nop\n\t"                                \
        :                                        \
        : "i" (_SFR_MEM_ADDR(__SPM_REG)),        \
          "r" ((uint8_t)__BOOT_RWW_ENABLE)       \
    );                                           \
}))
/* Normal versions of the macros use 16-bit addresses.
   Extended versions of the macros use 32-bit addresses.
   Alternate versions of the macros use 16-bit addresses and require special
   instruction sequences after LPM.
   FLASHEND is defined in the ioXXXX.h file.
   USHRT_MAX is defined in <limits.h>. */ 
#if defined(__AVR_ATmega161__) || defined(__AVR_ATmega163__) \
    || defined(__AVR_ATmega323__)
/* Alternate: ATmega161/163/323 and 16 bit address */
#define boot_rww_enable()             __boot_rww_enable_alternate()
#elif (FLASHEND > USHRT_MAX)
/* Extended: >16 bit address */
#define boot_rww_enable()             __boot_rww_enable_short()
#else
/* Normal: 16 bit address */
#define boot_rww_enable()             __boot_rww_enable_short()
#endif



//================================
// NRWW memory
// Addresses below NRWW (Non-Read-While-Write) can be programmed while
// continuing to run code from flash, slightly speeding up programming
// time.  Beware that Atmel data sheets specify this as a WORD address,
// while optiboot will be comparing against a 16-bit byte address.  This
// means that on a part with 128kB of memory, the upper part of the lower
// 64k will get NRWW processing as well, even though it doesn't need it.
// That's OK.  In fact, you can disable the overlapping processing for
// a part entirely by setting NRWWSTART to zero.  This reduces code
// space a bit, at the expense of being slightly slower, overall.
//
// RAMSTART should be self-explanatory.  It's bigger on parts with a
// lot of peripheral registers.
#if defined(__AVR_ATmega168__)
#define RAMSTART (0x100)
#define NRWWSTART (0x3800)
#elif defined(__AVR_ATmega328P__) || defined(__AVR_ATmega32__)
#define RAMSTART (0x100)
#define NRWWSTART (0x7000)
#elif defined (__AVR_ATmega644P__)
#define RAMSTART (0x100)
#define NRWWSTART (0xE000)
// correct for a bug in avr-libc
#undef SIGNATURE_2
#define SIGNATURE_2 0x0A
#elif defined (__AVR_ATmega1284P__)
#define RAMSTART (0x100)
#define NRWWSTART (0xE000)
#elif defined(__AVR_ATtiny84__)
#define RAMSTART (0x100)
#define NRWWSTART (0x0000)
#elif defined(__AVR_ATmega1280__)
#define RAMSTART (0x200)
#define NRWWSTART (0xE000)
#elif defined(__AVR_ATmega8__) || defined(__AVR_ATmega88__)
#define RAMSTART (0x100)
#define NRWWSTART (0x1800)
#endif

//C zero initialises all global variables. However, that requires */
//These definitions are NOT zero initialised, but that doesn't matter */
//This allows us to drop the zero init code, saving us memory */
#define buff ((uint8_t*)(RAMSTART))
//========================
//address in ram to save wbyte if we are inside bootloader
//to know that watchdog event was happend inside bootloader, not in app
#define waddr 0x4f0
#define wbyte 0x55
//exit from bootloader -> go to app
static void startapp(){
    watchdogConfig(WATCHDOG_2S);
    asm volatile ("clr r30 \n"
                  "sts %[ADDR],r30 \n"
                  "clr r31 \n" 
                  "ijmp \n" ::[ADDR]""(waddr));
}
//========================
//everything that needs to run VERY early 
void pre_main(void) {
  // Allow convenient way of calling do_spm function - jump table,
  //   so entry to this function will always be here, indepedent of compilation,
  //   features etc
  //1f
  //"1:\n"
  asm volatile (
    "rjmp	main\n"
    "rjmp	do_spm\n"
   );
}
//===============================
// main program starts here 
int main(void) {
  register uint8_t ch = MCUSR;
  
  //save MCUSR for app:
  //==2 if was external reset
  //==4 if we get started or was low power
  //==8 if was watchdog interrupt inside app (not inside bootloader)
  GPIOR0=ch;
  watchdogConfig(WATCHDOG_1S);
  MCUSR = 0;  
  
  //  No interrupts will execute
  //  SP points to RAMEND
  //  r1 contains zero
  // If not, uncomment the following instructions:
  // cli();
  asm volatile ("clr __zero_reg__");
#if defined(__AVR_ATmega8__) || defined (__AVR_ATmega32__)
  SP=RAMEND;  // This is done by hardware reset
#endif
  //if not EXTRF goto app start
  if (!(ch & (_BV(EXTRF)))) {
    //if (ch & (_BV(WDRF) | _BV(BORF) | _BV(PORF))) {
    //test if we get watchdog reset when stay in bootloader
    register uint8_t tmp;      
    asm volatile ("lds %[TMP],%[ADDR]":[TMP]"=&r"(tmp):[ADDR]""(waddr));
    //we was inside bootloader -> clear WDRF
    if (tmp==wbyte) {GPIOR0=ch&(~_BV(WDRF))|_BV(EXTRF);}
    startapp();
  }
  //set that we inside bootloader
  asm volatile ("ldi r24,%[WBT]\n" 
                "sts %[ADDR],r24"::[ADDR]""(waddr),[WBT]""(wbyte));
  //load UBRR0L
  EEARH=0;
  EEARL=0;
  EECR |= (1<<EERE);
  ch=EEDR;
#ifndef SOFT_UART
#if defined(__AVR_ATmega8__) || defined (__AVR_ATmega32__)
  if (ch==0xFF) {ch=(uint8_t)( (F_CPU + BAUD_RATE * 4L) / (BAUD_RATE * 8L) - 1 );}  
  UBRRL=ch;
  UCSRA = _BV(U2X); //Double speed mode USART
  UCSRB = _BV(RXEN) | _BV(TXEN);  // enable Rx & Tx
  UCSRC = _BV(URSEL) | _BV(UCSZ1) | _BV(UCSZ0);  // config USART; 8N1
#else
  if (ch==0xFF) {ch=(uint8_t)( (F_CPU + BAUD_RATE * 4L) / (BAUD_RATE * 8L) - 1 );}  
  UART_SRL=ch;
  UART_SRA = _BV(U2X0); //Double speed mode USART0
  UART_SRB = _BV(RXEN0) | _BV(TXEN0);
  UART_SRC = _BV(UCSZ00) | _BV(UCSZ01);
#endif
#endif
  //load OSCCAL
  EEARL=1;
  EECR |= (1<<EERE);
  ch=EEDR;
  if (ch!=0xFF) {OSCCAL=ch;}

#ifdef SOFT_UART
  /* Set TX pin as output */
  UART_DDR |= _BV(UART_TX_BIT);
#endif

  // Forever loop 
  register uint16_t address=0;
  register uint8_t  length;
  
  
  for (;;) {
    //asm volatile ("wdr");
    // get character from UART 
    ch = getch();

    if(ch == STK_GET_PARAMETER) {
      //unsigned char which = getch();
      getch();
      verifySpace();
      putch(0x03);
/*
      if (which == 0x82) {
	// Send optiboot version as "minor SW version"
	 
	putch(OPTIBOOT_MINVER);
      } else if (which == 0x81) {
	  putch(OPTIBOOT_MAJVER);
      } else {
	// GET PARAMETER returns a generic 0x03 reply for
        // other parameters - enough to keep Avrdude happy
	 
	putch(0x03);
      }
*/

    }
    else if(ch == STK_SET_DEVICE) {
      // SET DEVICE is ignored
      getNch(20);
    }
//needed

    else if(ch == STK_SET_DEVICE_EXT) {
      // SET DEVICE EXT is ignored
      getNch(5);
    }
    
    else if(ch == STK_LOAD_ADDRESS) {
      // LOAD ADDRESS
      //it's strange, but I have tried and unable optimize it by code
      register uint16_t newAddress;
      newAddress = getch();
      newAddress = (newAddress & 0xff) | (getch() << 8);
      newAddress += newAddress; // Convert from word address to byte address
      address = newAddress;
      verifySpace();
    }
//needed
    else if(ch == STK_UNIVERSAL) {
      // UNIVERSAL command is ignored
      getNch(4);
      putch(0x00);
    }

    // Write memory, length is big endian and is in bytes 
    else if(ch == STK_PROG_PAGE) {
      uint8_t *bufPtr;
      register uint16_t addrPtr;
      getch();			// getlen() 
      length = getch();
      register uint8_t memtype = getch();
#ifdef SUPPORT_EEPROM
        register uint8_t _cnt = length;
#endif
      // If we are in RWW section, immediately start page erase
      // While that is going on, read in page contents
      bufPtr = buff;
      do *bufPtr++ = getch(); while (--length);
      // If we are in NRWW section, page erase has to be delayed until now.
      // Todo: Take RAMPZ into account (not doing so just means that we will
      //  treat the top of both "pages" of flash as NRWW, for a slight speed
      //  decrease, so fixing this is not urgent.)
      //AV: moved to 'F' if...
      if (memtype=='F') {
        //AV: moved:
        //if (address < NRWWSTART) __boot_page_erase_short((uint16_t)(void*)address);
        //if (address >= NRWWSTART) __boot_page_erase_short((uint16_t)(void*)address);

        // Read command terminator, start reply
        verifySpace();
        // If only a partial page is to be programmed, the erase might not be complete.
        // So check that here
        //boot_spm_busy_wait();
        do_spm((uint16_t)(void*)address,__BOOT_PAGE_ERASE,0);

        // Copy buffer into programming buffer
        bufPtr = buff;
        addrPtr = (uint16_t)(void*)address;
        ch = SPM_PAGESIZE / 2;
        do {
          uint16_t a;
          a = *bufPtr++;
          a |= (*bufPtr++) << 8;
          do_spm((uint16_t)(void*)addrPtr,__BOOT_PAGE_FILL,a);
          addrPtr += 2;
        } while (--ch);

        // Write from programming buffer
        //__boot_page_write_short((uint16_t)(void*)address);
        //boot_spm_busy_wait();
        /*
        #if defined(RWWSRE)
          // Reenable read access to flash
          //boot_rww_enable();
        #endif
        */
        do_spm((uint16_t)(void*)address,__BOOT_PAGE_WRITE,0);
      }
#ifdef SUPPORT_EEPROM
      else {//if (memtype=='E') {
        register uint8_t i=0;
        // Read command terminator, start reply
        verifySpace();
        while(i<_cnt) {
            while(EECR & (1<<EEPE));
            EEAR = address++;
            EEDR = buff[i++];
            EECR |= (1<<EEMPE);
            EECR |= (1<<EEPE);
        }
      }
#endif
    }
    
    // Read memory block mode, length is big endian.  
    else if(ch == STK_READ_PAGE) {
      // READ PAGE - we only read flash
      //AV: add EEPROM
      uint8_t memtype;
      getch();			// getlen() 
      length = getch();
      memtype=getch();

        verifySpace();
        do {
          if (memtype=='F') {
        // read a Flash byte and increment the address
        __asm__ ("lpm %0,Z+\n" : "=r" (ch), "=z" (address): "1" (address));
          }
          #if SUPPORT_EEPROM
          if(memtype=='E') {
//            ch=readeeprom(address++);

            while(EECR & (1<<EEPE)) ;
            EEAR = address++;
            EECR |= (1<<EERE);
            ch = EEDR;

          }
          #endif
          putch(ch);
        } while (--length);
    }

    // Get device signature bytes  
    else if(ch == STK_READ_SIGN) {
      // READ SIGN - return what Avrdude wants to hear
      verifySpace();
      putch(SIGNATURE_0);
      putch(SIGNATURE_1);
      putch(SIGNATURE_2);
    }
    else  { 
      // Adaboot no-wait mod
      verifySpace();
    }
   
/*
    else if (ch == STK_LEAVE_PROGMODE) { // 'Q' 
      // Adaboot no-wait mod
      watchdogConfig(WATCHDOG_16MS);
      verifySpace();
    }
    else {
      // This covers the response to commands like STK_ENTER_PROGMODE
      verifySpace();
    }
*/
    putch(STK_OK);
  }
}

void putch(char ch) {
#ifndef SOFT_UART
  while (!(UART_SRA & _BV(UDRE0)));
  UART_UDR = ch;
#else
  __asm__ __volatile__ (
    "   com %[ch]\n" // ones complement, carry set
    "   sec\n"
    "1: brcc 2f\n"
    "   cbi %[uartPort],%[uartBit]\n"
    "   rjmp 3f\n"
    "2: sbi %[uartPort],%[uartBit]\n"
    "   nop\n"
    "3: rcall uartDelay\n"
    "   rcall uartDelay\n"
    "   lsr %[ch]\n"
    "   dec %[bitcnt]\n"
    "   brne 1b\n"
    :
    :
      [bitcnt] "d" (10),
      [ch] "r" (ch),
      [uartPort] "I" (_SFR_IO_ADDR(UART_PORT)),
      [uartBit] "I" (UART_TX_BIT)
    :
      "r25"
  );
#endif
}

uint8_t getch(void) {
  uint8_t ch;
#ifdef SOFT_UART
  __asm__ __volatile__ (
    "1: sbic  %[uartPin],%[uartBit]\n"  // Wait for start edge
    "   rjmp  1b\n"
    "   rcall uartDelay\n"          // Get to middle of start bit
    "2: rcall uartDelay\n"              // Wait 1 bit period
    "   rcall uartDelay\n"              // Wait 1 bit period
    "   clc\n"
    "   sbic  %[uartPin],%[uartBit]\n"
    "   sec\n"
    "   dec   %[bitCnt]\n"
    "   breq  3f\n"
    "   ror   %[ch]\n"
    "   rjmp  2b\n"
    "3:\n"
    :
      [ch] "=r" (ch)
    :
      [bitCnt] "d" (9),
      [uartPin] "I" (_SFR_IO_ADDR(UART_PIN)),
      [uartBit] "I" (UART_RX_BIT)
    :
      "r25"
);
#else
  while(!(UART_SRA & _BV(RXC0))) 
  ;
  //if (!(UART_SRA & _BV(FE0))) {
      // A Framing Error indicates (probably) that something is talking
      // to us at the wrong bit rate.  Assume that this is because it
      // expects to be talking to the application, and DON'T reset the
      // watchdog.  This should cause the bootloader to abort and run
      // the application "soon", if it keeps happening.  (Note that we
      // don't care that an invalid char is returned...)
      __asm__ __volatile__ ("wdr");
  //}
  ch = UART_UDR;
#endif
  return ch;
}

#ifdef SOFT_UART
// AVR305 equation: #define UART_B_VALUE (((F_CPU/BAUD_RATE)-23)/6)
// Adding 3 to numerator simulates nearest rounding for more accurate baud rates
#define UART_B_VALUE (((F_CPU/BAUD_RATE)-20)/6)
#if UART_B_VALUE > 255
#error Baud rate too slow for soft UART
#endif

void uartDelay() {
  __asm__ __volatile__ (
    "ldi r25,%[count]\n"
    "1:dec r25\n"
    "brne 1b\n"
    "ret\n"
    ::[count] "M" (UART_B_VALUE)
  );
}
#endif

void getNch(uint8_t count) {
  do getch(); while (--count);
  verifySpace();
}

void verifySpace() {
  if (getch() != CRC_EOP) { while(1); }
  putch(STK_INSYNC);
}


// Separate function for doing spm stuff
//It's needed for application to do SPM, as SPM instruction works only
//from bootloader.
//How it works:
// - do SPM
// - wait for SPM to complete
// - if chip have RWW/NRWW sections it does additionaly:
// - if command is WRITE or ERASE, AND data=0 then reenable RWW section
// In short:
// If you play erase-fill-write, just set data to 0 in ERASE and WRITE
// If you are brave, you have your code just below bootloader in NRWW section
// you could do fill-erase-write sequence with data!=0 in ERASE and
// data=0 in WRITE
static void do_spm(uint16_t address, uint8_t command, uint16_t data) {
    asm volatile (
	"    movw  r0, %3\n"
        "    out %0, %1\n"
        "    spm\n"
        "    clr  r1\n"
        :
        : "i" (_SFR_IO_ADDR(__SPM_REG)),
          "r" ((uint8_t)command),
          "z" ((uint16_t)address),
          "r" ((uint16_t)data)
        : "r0"
    );
    // wait for spm to complete
    //   it doesn't have much sense for __BOOT_PAGE_FILL,
    //   but it doesn't hurt and saves some bytes on 'if'
    boot_spm_busy_wait();
#if defined(RWWSRE)
    // this 'if' condition should be: (command == __BOOT_PAGE_WRITE || command == __BOOT_PAGE_ERASE)...
    // but it's tweaked a little assuming that in every command we are interested in here, there
    // must be also SELFPRGEN set. If we skip checking this bit, we save here 4B
    if ((command & (_BV(PGWRT)|_BV(PGERS))) && (data == 0) ) {
      // Reenable read access to flash
      boot_rww_enable();
    }
#endif
}
// ========  END =========================





//========================================
//make clean
//make atmega328  BAUD_RATE=38400 FMHZ=16
/* makefile
AVRDUDE_CONF = -Cavrdude.conf
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
*/


//==========================================
/*
//June 2015 by Marek Wodzinski, https://github.com/majekw
//Released to public domain
//This is example how to use optiboot.h together with Optiboot
//bootloader to write to FLASH memory by application code.
//IMPORTANT THINGS:
//buffer must be page aligned (see declaration of flash_buffer)
//interrupts must be disabled during spm
//writing to EEPROM destroys temporary buffer
//you can write only once into one location of temporary buffer
//only safely and always working sequence is erase-fill-write
//if you want to do fill-erase-write, you must put code in NRWW
//and pass data!=0 for erase. It's not easy, but possible.
// WRITE SEQUENCE - option 1 (used in this example)
// 1. Erase page by optiboot_page_erase
// 2. Write contents of page into temporary buffer by optiboot_page_fill
// 3. Write temporary buffer to FLASH by optiboot_page_write
//
// WRITE SEQUENCE - option 2 (works only for code in NRWW)
// 1. Write contents of page into temporary buffer by optiboot_page_fill
// 2. Erase page by optiboot_page_erase (set data to NOT zero)
// 3. Write temporary buffer to FLASH by optiboot_page_write
//
// This gives possibility to use SPM instruction
// from Optiboot bootloader memory.
// There are 3 convenient functions available here:
//  optiboot_page_erase - to erase FLASH page
//  optiboot_page_fill - to put words into temporary buffer
//  optiboot_page_write - to write contents of temporary buffer into FLASH
//For some hardcore users, you could use 'do_spm' as raw entry to
//bootloader spm function.

#include <avr/boot.h>

//Main 'magic' function - enter to bootloader do_spm function
//address - address to write (in bytes) but must be even number
//command - one of __BOOT_PAGE_WRITE, __BOOT_PAGE_ERASE or __BOOT_PAGE_FILL
//data - data to write in __BOOT_PAGE_FILL. In __BOOT_PAGE_ERASE or 
//      __BOOT_PAGE_WRITE it control if boot_rww_enable is run
//      (0 = run, !0 = skip running boot_rww_enable)
//Contents of bootloader's do_spm function, just for reference:
// static void do_spm(uint16_t address, uint8_t command, uint16_t data) {
//     // Do spm stuff
//     asm volatile (
//       "    movw  r0, %3\n"
//       "    out %0, %1\n"
//       "    spm\n"
//       "    clr  r1\n"
//       :
//       : "i" (_SFR_IO_ADDR(__SPM_REG)),
//         "r" ((uint8_t)command),
//         "z" ((uint16_t)address),
//         "r" ((uint16_t)data)
//       : "r0"
//     );
// wait for spm to complete
//   it doesn't have much sense for __BOOT_PAGE_FILL,
//   but it doesn't hurt and saves some bytes on 'if'
//     boot_spm_busy_wait();
// #if defined(RWWSRE)
//     // this 'if' condition should be: (command == __BOOT_PAGE_WRITE || command == __BOOT_PAGE_ERASE)...
//     // but it's tweaked a little assuming that in every command we are interested in here, there
//     // must be also SELFPRGEN set. If we skip checking this bit, we save here 4B
//     if ((command & (_BV(PGWRT)|_BV(PGERS))) && (data == 0) ) {
//       // Reenable read access to flash
//       boot_rww_enable();
//     }
// #endif
// }
// 'typedef' (in following line) and 'const' (few lines below) are a way to define external function at some arbitrary address

typedef void (*do_spm_t)(uint16_t address, uint8_t command, uint16_t data);

//
// Devices with more than 64KB of flash:
// - have larger bootloader area (1KB) (they are BIGBOOT targets)
// - have RAMPZ register :-) 
// - need larger variable to hold address (pgmspace.h uses uint32_t)
// To not do many ifdefs and don't confuse users I declared new 'always valid'

// type to declare address: optiboot_addr_t.
#ifdef RAMPZ
  typedef uint32_t optiboot_addr_t;
  const do_spm_t do_spm = (do_spm_t)((FLASHEND-1023+2)>>1);
#else
  typedef uint16_t optiboot_addr_t;
  const do_spm_t do_spm = (do_spm_t)((FLASHEND-511+2)>>1);
#endif
//The same as do_spm but with disable/restore interrupts state
// required to succesfull SPM execution 
// On devices with more than 64kB flash, 16 bit address is not enough,
// so there is also RAMPZ used in that case.
void do_spm_cli(optiboot_addr_t address, uint8_t command, uint16_t data) {
  uint8_t sreg_save;
  sreg_save = SREG;  // save old SREG value
  asm volatile("cli");  // disable interrupts
  #ifdef RAMPZ
    RAMPZ=(address>>16) & 0xff;  // address bits 23-16 goes to RAMPZ
    do_spm((address & 0xffff),command,data); // do_spm accepts only lower 16 bits of address
  #else
    do_spm(address,command,data);  // 16 bit address - no problems to pass directly
  #endif
  SREG=sreg_save; // restore last interrupts state
}
//Erase page in FLASH
void optiboot_page_erase(optiboot_addr_t address) {
  do_spm_cli(address,__BOOT_PAGE_ERASE,0);}
//Write word into temporary buffer
void optiboot_page_fill(optiboot_addr_t address, uint16_t data) {
  do_spm_cli(address,__BOOT_PAGE_FILL,data);}
//Write temporary buffer into FLASH
void optiboot_page_write(optiboot_addr_t address) {
  do_spm_cli(address,__BOOT_PAGE_WRITE,0);}


//============================
//Example
#include <avr/pgmspace.h>
const char flash_buffer[SPM_PAGESIZE] __attribute__ (( aligned(SPM_PAGESIZE) )) PROGMEM= {
  "0123456789ABCDEFGHIJKLMNOPQRSTUVW"
};
void setup() {
  int i;
  uint8_t c;
  uint16_t w;
  uint8_t ram_buffer[SPM_PAGESIZE];
  // Init serial
  Serial.begin(38400);
  // Print current flash buffer content
  Serial.println("Current flash contents:");
  for (i=0;i<SPM_PAGESIZE;i++) {
    c = pgm_read_byte(&flash_buffer[i]);
    if (c!=0 && c!=255) {
      Serial.write(c);
    } else {
      Serial.print(".");
    }
  }
  // Print prompt to enter some new characters to write to flash
  Serial.println();
  Serial.print("Type ");
  Serial.print(SPM_PAGESIZE);
  Serial.println(" characters to store in flash:");
  // Get characters from serial and put into ram buffer
  i=0;
  while (i<SPM_PAGESIZE) {
    if (Serial.available()>0) {
      c = Serial.read(); // read character from serial
      Serial.write(c); // echo character back
      ram_buffer[i] = c;
      i++;
    }
  }
  Serial.println("\nAll chars received");
  delay(100); // wait for sending all text via serial
  // Erasing FLASH page
  Serial.println("Erasing buffer");
  delay(100); // wait for sending all text via serial
  optiboot_page_erase((optiboot_addr_t)(void*) &flash_buffer[0]);
  // Copy ram buffer to temporary flash buffer
  Serial.println("Writing to temporary flash buffer");
  delay(100); // wait for sending all text via serial
  for (i=0;i<SPM_PAGESIZE;i++) {
    if (i % 2 == 0) { // we must write WORDs
      w = ram_buffer[i];
    } else {
      w += (ram_buffer[i] << 8);
      optiboot_page_fill((optiboot_addr_t)(void*) &flash_buffer[i],w);
    }
  }
  // Writing temporary buffer to FLASH
  Serial.println("Writing buffer to flash");
  delay(100); // wait for sending all text via serial
  optiboot_page_write((optiboot_addr_t)(void*) &flash_buffer[0]);

  Serial.println("Write done, thank you!");
  Serial.println("Now you can reset or power cycle board and check for new contents!");
}
void loop() {;}
*/