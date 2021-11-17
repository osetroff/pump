#ifndef k_atmega328p_8mhz_
#define k_atmega328p_8mhz_
#define F_CPU 8000000L
//to find in disassembled text by avr-objdump -S ...elf > ...txt
#define _debm asm volatile("cli");asm volatile("sei");

/*
    atmega328p dip28

    |1 RST    C5 28|  SCL
RXD |2 D0     C4 27|  SDA
TXD |3 D1     C3 26|
INT0|4 D2     C2 25|
INT1|5 D3     C1 24|
    |6 D4     C0 23|
    |7 VCC   GND 22|
    |8 GND  aref 21|
    |9 B6   AVCC 20|
    |10 B7    B5 19|  SCK
T1  |11 D5    B4 18|  MISO
    |12 D6    B3 17|  MOSI
    |13 D7    B2 16|
    |14 B0    B1 15|

   atmega328 arduino mini


    DTR TXD RXD VCC GND GND
     o   o   o   o   o  o
TXD |  D1            RAW |
RXD |  D0            GND |
    |  RST           RST |
    |  GND           VCC |
INT0|2 PD2        PC3 A3 |
INT1|3 PD3        PC2 A2 |
T0  |4 PD4        PC1 A1 |
T1  |5 PD5        PC0 A0 |
    |6 PD6        PB5 13 |  SCK
    |7 PD7        PB4 12 | MISO
    |8 PB0        PB3 11 | MOSI
    |9 PB1        PB2 10 |
*/


#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/power.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>

//#include "iom328p.h"

#define k_cli() cli()
#define k_sei() sei()

#define u8      uint8_t
#define u16     uint16_t
#define u32     uint32_t
#define i16     int16_t
#define i32     int32_t

#define bit_set0(lvar, lbitmsk) lvar&=~(lbitmsk)
#define bit_set1(lvar, lbitmsk) lvar|=(lbitmsk)
#define bit_is0(lvar, lbitmsk) ((lvar&(lbitmsk))==0)
#define bit_is1(lvar, lbitmsk) ((lvar&(lbitmsk))!=0)


#include <assert.h>

//led debug
inline static void led_deb_high(void);
inline static void led_deb_low(void);
//inline static void led_pin_xor(void);
#define ldh  led_deb_high()
#define ldl  led_deb_low()
//#define lpx  led_pin_xor()

//========================
//
//========================
//4,294,967,295/(year=366*24*3600sec=31,622,400) > 135 years
//65,535/(day=24*3600sec=86400sec)

typedef unsigned char byte;
typedef unsigned short word;
typedef unsigned int dword;

static inline byte k_iswatchdogerror(){return (GPIOR0&(1<<WDRF));}
//========================
//  memory
//========================
word testheap() {
  extern word __heap_start, *__brkval;
  return (word) __brkval;
}
word testsp(){
 byte lsp;
 return (word)&lsp;
}
//get free ram size
//word testram (){return RAMEND - word (__malloc_heap_start);}


//========================
//
//========================

//set freq of atmega
//0 clock_div_1
//1 clock_div_2
//2 clock_div_4
//3 clock_div_8
//4 clock_div_16
//5 clock_div_32
//6 clock_div_64
//7 clock_div_128
//8 clock_div_256
#define freq(lclk_div) cli();CLKPR = 1<<CLKPCE;CLKPR = lclk_div;sei()



//========================
// int
//========================
#define int0en() if ((EIMSK&(1<<INT0))==0) {EIFR|=(1<<INTF0);EIMSK|=(1<<INT0);}
#define int0dis() EIMSK &= ~(1<<INT0)
inline void int0_on_fall() 
{
    u8 lr=EICRA&(~((1<<ISC01)|(1<<ISC00)));
    EICRA=lr|((1<<ISC01)|(0<<ISC00));
}
#define int1en() if ((EIMSK&(1<<INT1))==0) {EIFR|=(1<<INTF1);EIMSK|=(1<<INT1);}
#define int1dis() EIMSK &= ~(1<<INT1)
#define int1clr() EIFR|=(1<<INTF1)
//00 low, 01 any, 10 fall, 11 rise
inline void int1_on_fall() 
{
    u8 lr=EICRA&(~((1<<ISC11)|(1<<ISC10)));
    EICRA=lr|((1<<ISC11)|(0<<ISC10));
}


//========================
//   delay
//========================

//==Delay
static void dus(uint16_t tic_us){
	tic_us <<= 1; //1us = 4 öèêëà
	__asm__ volatile (
			"1: sbiw %0,1" "\n\t"
			"brne 1b"
			: "=w" (tic_us)
			: "0" (tic_us)
		  );
}
void dms(uint16_t tic_ms){
  //correction
  uint16_t tic_us=(tic_ms + (tic_ms>> 2)+(tic_ms>> 11))<<1;
  if (tic_us>0) {
  __asm__ volatile (
    "1: sbiw %0,1" "\n\t"
    "brne 1b"
    : "=w" (tic_us)
    : "0" (tic_us)
   );
  }
  while(tic_ms-->0){
    tic_us=1000 << 1;
    __asm__ volatile (
			"1: sbiw %0,1" "\n\t"
			"brne 1b"
			: "=w" (tic_us)
			: "0" (tic_us)
		 );
  }
}

//========================
//   adc and ac
//========================
#define adc_off() ADCSRA &= ~(1 << ADEN)
#define adc_on() ADCSRA |= (1 << ADEN)
#define ac_off() ACSR |= (1 << ACD)
#define ac_on() ACSR &= ~(1 << ACD)



//========================
//   eeprom
//========================
inline static void eewait(void) {
  while (!bit_is_clear(EECR, EEPE));  
}
inline static byte eerb(void) {
  EECR |= 1 << EERE;  /* Start eeprom read by writing EERE */
  return EEDR;
}

inline static word ee_getaddr(void)
{
    return EEAR;
}

inline static void ee_setaddr(word addr) {
  EEAR = addr;
}

inline static byte eerb_next(void) {
  EEAR += 1;
  return eerb();
}


//!!!! always test for eeprom errors!!!
//ret 1 if error , ret 0 if ok
static byte eewb(byte val, byte lnext) {
  byte lprev;
  if (lnext!=0)
  {
      EEAR+=1;
  }    
  byte lprev=eerb();
  if (lprev!=val) {
    //EEAR = addr;
    EEDR = val;
    EECR |= 1 << EEMPE; /* Write logical one to EEMPE */
    EECR |= 1 << EEPE;  /* Start eeprom write by setting EEPE */
    //test for errors
    eewait();
    if (eerb())!=val) return 1;
  }
  return 0;
}

//========================
//  spi
//========================
#define spi_ddr	DDRB
#define spi_port	PORTB
//pin 11
#define spi_mosi_pin	(1<<3)
//pin12
#define spi_miso_pin	(1<<4)
//pin13
#define spi_sck_pin	(1<<5)
//
#define spi_clock_mask 0x03  // spr1 = bit 1, spr0 = bit 0 on spcr
#define spi_2xclock_mask 0x01  // spi2x = bit 0 on spsr
#define spi_clock_div4 0x00
#define spi_clock_div16 0x01
#define spi_clock_div64 0x02
#define spi_clock_div128 0x03
#define spi_clock_div2 0x04
#define spi_clock_div8 0x05
#define spi_clock_div32 0x06
//
#define spi_mode_mask 0x0c  // cpol = bit 3, cpha = bit 2 on spcr
#define spi_mode0 0x00
#define spi_mode1 0x04
#define spi_mode2 0x08
#define spi_mode3 0x0c
inline static void spi_setclockdiv(byte clockdiv) {
    SPCR = (SPCR & ~spi_clock_mask) | (clockdiv & spi_clock_mask);
    SPSR = (SPSR & ~spi_2xclock_mask) | ((clockdiv >> 2) & spi_2xclock_mask);
}
static void spi_init(byte ldatamode,byte lclockdiv) {
  //SCK & MOSI & SS as outputs, MISO as input
  spi_ddr &= ~spi_miso_pin;
  spi_ddr |= spi_sck_pin | spi_mosi_pin;
  //Enable SPI master, set mode
  SPCR = (1 << SPE) | (1 << MSTR)| (ldatamode & spi_mode_mask);
  //MSB
  //SPCR &= ~(1<<DORD);
  //clock div
  spi_setclockdiv(lclockdiv);
}
#define spi_wait 250
#define spi_waitasm() asm volatile("nop\n");

//!!!spi is bidirectional: every time we send byte too.
//it is important what byte we will send

//read byte
//write lw to spi,read byte to *lb
//ret 0 if ok
inline static byte spi_wbr(byte lw,byte *lb) {
  register byte li=0;
  SPDR=lw;
  while (li++<spi_wait) {
    if (SPSR & (1 << SPIF)) {
      *lb=SPDR;
      return 0;
    }
    spi_waitasm();
  }
  //li=SPDR;
  return 1;
}


//write byte lw to spi,read byte to *lb
//ret 0 if ok
inline static byte spi_wb(byte lw) {
  register byte li=0;
  SPDR=lw;
  while (li++<spi_wait) {
    if (SPSR & (1 << SPIF)) {
         //maibe we need lw=SPDR in the end to reset SPIF
         //li=SPDR;
       return 0;
    }
    spi_waitasm();
  }
  return 1;
}

//write ln bytes to spi
//ret 0 if ok
static byte spi_w(byte ln,byte * lb) {
  while (ln--) {
    SPDR=*lb++;
    register byte li=0;
    while(li++<spi_wait) {
      if (SPSR & (1 << SPIF)) {break;}
      spi_waitasm();
    }
    if (li==spi_wait) {return 1;}
  }
  return 0;
}

//read ln bytes from spi
//ret 0 if ok
static byte spi_r(byte ln,byte * lb,byte lw) {
  while (ln--) {
    SPDR=lw;
    register byte li=0;
    while(li++<spi_wait) {
      if (SPSR & (1 << SPIF)) {*lb++=SPDR;break;}
      spi_waitasm();
    }
    if (li==spi_wait) {return 1;}
  }
  return 0;
}


//==============
//i2c
//==============
/*
Return values.
Looking at a full communication sequence between a
master and slave (transmit data and then readback data) there
a total of 7 points in the sequence where a timeout can occur.
  0 - OK
  1 - Waiting for successful completion of a Start bit
  2 - Waiting for ACK/NACK while addressing slave in transmit mode (MT)
  3 - Waiting for ACK/NACK while sending data to the slave
  4 - Waiting for successful completion of a Repeated Start
  5 - Waiting for ACK/NACK while addressing slave in receiver mode (MR)
  6 - Waiting for ACK/NACK while receiving data from the slave
  7 - Waiting for successful completion of the Stop bit
  8 - 0xFF    See datasheet for exact meaning
*/


//=========================
#define START           0x08
#define REPEATED_START  0x10
#define MT_SLA_ACK  0x18
#define MT_SLA_NACK 0x20
#define MT_DATA_ACK     0x28
#define MT_DATA_NACK    0x30
#define MR_SLA_ACK  0x40
#define MR_SLA_NACK 0x48
#define MR_DATA_ACK     0x50
#define MR_DATA_NACK    0x58
#define LOST_ARBTRTN    0x38
#define TWI_STATUS      (TWSR & 0xF8)
#define SLA_W(address)  (address << 1)
#define SLA_R(address)  ((address << 1) + 0x01)
#define cbi(sfr, bit)   (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit)   (_SFR_BYTE(sfr) |= _BV(bit))

#define MAX_BUFFER_SIZE 32
//=========================
#define i2c_wait() dus(5)
//idle(_15);
#define i2c_waitcount 60
//======================
//releases SDA and SCL lines to high impedance
inline void i2c_disable(){TWCR = 0;}
//==========================
//initialize
inline void i2c_enable(){TWCR = _BV(TWEN) | _BV(TWEA);}
//==========================
inline void i2c_lockup() {
  i2c_disable();
  i2c_enable();
}
//==========================
#define i2c_wait_(ltwi,ltxt) \
  register byte li=i2c_waitcount; \
  while (!(TWCR & (1<<ltwi))) {\
    if (--li==0) {\
      i2c_lockup();\
      return(1);\
    } \
    i2c_wait();\
  } //sp(ltxt);spln(li);

//======================
static void i2c_speed(byte lspeedmul100000){
  cbi(TWSR, TWPS0);cbi(TWSR, TWPS1);
  TWBR = ((F_CPU / (lspeedmul100000*100000L)) - 16) / 2;
}
//======================
inline void i2c_init(byte lspeedmul100000=4){
  //setup ports
  sbi(PORTC, 4);
  sbi(PORTC, 5);
  // default
  i2c_speed(lspeedmul100000);
  i2c_enable();
}
//==========================
inline uint8_t i2c_start(){
  TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
  i2c_wait_(TWINT,"start");
  li=TWI_STATUS;
  if (( li==START) || (li==REPEATED_START)) {return(0);}
  if (li==LOST_ARBTRTN)  {i2c_lockup();}
  return li;
}
//===========================
uint8_t i2c_stop(byte lwait){
  TWCR = (1<<TWINT)|(1<<TWEN)| (1<<TWSTO);
  if (lwait>0) {i2c_wait_(TWSTO,"stop"); }
  return(0);
}
//============================
inline uint8_t i2c_sa(uint8_t i2cAddress){
  TWDR = i2cAddress;
  TWCR = (1<<TWINT) | (1<<TWEN);
  i2c_wait_(TWINT,"sa");
  li=TWI_STATUS;
  if ((li==MT_SLA_ACK) || (li==MR_SLA_ACK)) {return(0);}
  if ((li==MT_SLA_NACK) || (li==MR_SLA_NACK))  {
    i2c_stop(1);
  }  else  {
    i2c_lockup();
  }
  return li;
}
//=============================
// Used to check for a response, while waiting for an EEPROM write.
uint8_t i2c_acknowledgepoll(uint8_t i2cAddress) {
  i2c_start();
  TWDR = SLA_W(i2cAddress);
  TWCR = (1<<TWINT) | (1<<TWEN);
  i2c_wait_(TWINT,"ack");
  li=TWI_STATUS;
  if ((li==MT_SLA_ACK) || (li==MR_SLA_ACK))  { return 1;}
  if ((li==MT_SLA_NACK) || (li==MR_SLA_NACK))  {
    return 0;
  }  else  {
    i2c_lockup();
  }
  return li;
}
//=======================
inline uint8_t i2c_sb(uint8_t i2cData){
  TWDR = i2cData;
  TWCR = (1<<TWINT) | (1<<TWEN);
  i2c_wait_(TWINT,"sb");
  li=TWI_STATUS;
  if (li==MT_DATA_ACK)  {return(0); }
  if (li==MT_DATA_NACK)  {
    i2c_stop(1);
  }  else  {
    i2c_lockup();
  }
  return li;
}
//======================
inline uint8_t i2c_rb(uint8_t ack){
  if(ack){TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWEA);}
  else {TWCR = (1<<TWINT) | (1<<TWEN);}
  i2c_wait_(TWINT,"rb");
  li=TWI_STATUS;
  if (li==LOST_ARBTRTN)  { i2c_lockup();}
  return li;
}
/*
//======================
void i2c_pullup(uint8_t activate)
{
  if(activate)  {
    #if defined(__AVR_ATmega168__) || defined(__AVR_ATmega8__) || defined(__AVR_ATmega328P__)
      // activate internal pull-ups for twi as per note from atmega8 manual pg167
      sbi(PORTC, 4);sbi(PORTC, 5);
    #else
      // activate internal pull-ups for twi as per note from atmega128 manual pg204
      sbi(PORTD, 0);sbi(PORTD, 1);
    #endif
  }  else  {
    #if defined(__AVR_ATmega168__) || defined(__AVR_ATmega8__) || defined(__AVR_ATmega328P__)
      // deactivate internal pull-ups for twi as per note from atmega8 manual pg167
      cbi(PORTC, 4);cbi(PORTC, 5);
    #else
      // deactivate internal pull-ups for twi as per note from atmega128 manual pg204
      cbi(PORTD, 0);cbi(PORTD, 1);
    #endif
  }
}
*/
//=========================
//read
uint8_t i2c_r(uint8_t address,
              uint16_t ln,
              uint8_t *data){
  if(ln == 0){ln++;}

  register byte lr=i2c_start();
  if (lr) {lr=i2c_start();if (lr) return(lr);}

  lr = i2c_sa(SLA_R(address));
  if (lr) {if(lr == 1){return(5);}return(lr);}

  uint8_t * lp=data;
  while (ln-->0)  {
    if( ln == 0 ) {
      lr = i2c_rb(0);
      if(lr == 1){return(6);}
      if(lr != MR_DATA_NACK){return(lr);}
    }  else  {
      lr = i2c_rb(1);
      if(lr == 1){return(6);}
      if(lr != MR_DATA_ACK){return(lr);}
    }
    *lp++ = TWDR;
  }
  lr=i2c_stop(0);
  if (lr==1) lr=7;
  return lr;
}
//=============================
//read
uint8_t i2c_r(uint8_t address,
              uint16_t regaddress,
              uint16_t ln,
              uint8_t *data,
              byte ra16){
  if(ln == 0){ln++;}

  register byte lr=i2c_start();
  if (lr) {lr=i2c_start();if (lr) return(lr);}

  lr = i2c_sa(SLA_W(address));
  if (lr) {if(lr == 1){return(2);}return(lr);}
  if (ra16){
      lr = i2c_sb(regaddress >> 8);
      if (lr) {if(lr == 1){return(3);}return(lr);}
  }
  lr = i2c_sb(regaddress & 0xff);
  if (lr) {if(lr == 1){return(3);}return(lr);}

  lr = i2c_start();
  if (lr) {if(lr == 1){return(4);}return(lr);}

  lr = i2c_sa(SLA_R(address));
  if (lr) {if(lr == 1){return(5);}return(lr);}
  uint8_t * lp=data;
  while (ln-->0)  {
    if( ln == 0 ) {
      lr = i2c_rb(0);
      if(lr == 1){return(6);}
      if(lr != MR_DATA_NACK){return(lr);}
    }  else  {
      lr = i2c_rb(1);
      if(lr == 1){return(6);}
      if(lr != MR_DATA_ACK){return(lr);}
    }
    *lp++ = TWDR;
  }
  lr=i2c_stop(0);
  if (lr==1) lr=7;
  return lr;
}
//=========================
//write
uint8_t i2c_w(uint8_t address,
              uint16_t regaddress,
              uint16_t ln,
              uint8_t *data,
              byte ra16){

  register byte lr=i2c_start();
  if (lr) {lr=i2c_start();if (lr) return(lr);}

  lr = i2c_sa(SLA_W(address));
  if (lr) { if(lr == 1){return(2);}return(lr);}
  if (ra16){
    lr = i2c_sb(regaddress >> 8);
    if (lr) {if(lr == 1){return(3);}return(lr);}
  }
  lr = i2c_sb(regaddress & 0xff);
  if (lr) {if(lr == 1){return(4);}return(lr);}
  uint8_t * lp=data;
  while (ln-->0) {
    lr = i2c_sb(*lp++);
    if (lr) {if(lr == 1){return(5);}return(lr);}
  }
  lr=i2c_stop(0);
  if (lr==1) lr=7;
  return lr;
}


//========================
//  power
//========================
//asm volatile("nop");
enum period_t {_15,_30,_60,_120,_250,_500,_1s,_2s,_4s,_8s,_forever};
enum adc_t{adc0,adc1};
enum bod_t{b0,b1};
enum timer2_t{t20,t21};
enum timer1_t{t10,t11};
enum timer0_t{t00,t01};
enum spi_t{s0,s1};
enum usart0_t{u0,u1};
enum twi_t{t0,t1};
//IDLE 15 mA
//ADC 6.5 mA
//PWR_SAVE 1.62 mA !!! clock on timer 2 will be slower (1ms for 1s)
//EXT_STANDBY 1.62 mA !!!clock on timer 2 will ok
//STANDBY  0.84 mA
//PWR_DOWN  0.36 mA


//Watchdog Timer interrupt service routine.
//This routine is required to allow automatic WDIF and WDIE bit clearance in hardware.
// WDIE & WDIF is cleared in hardware upon entering this ISR
static volatile u8 mc_wdt_will_reset=0;
ISR (WDT_vect)
{
    if (mc_wdt_will_reset==0)
    {
        wdt_disable();
        //mc_wdt_will_reset=1;
    }
//    else
//    {
//        cli();
//        wdt_enable(_1s);
//        while(1);
//    }
}

//reset
void reset(period_t lp) {
    mc_wdt_will_reset=1;
    wdt_enable(lp);
    while (1) {}
}

#ifndef sleep_bod_disable
inline void sleep_bod_disable() {
  //do {
  unsigned char tempreg;
  __asm__ __volatile__("in %[tempreg], %[mcucr]" "\n\t" \
                       "ori %[tempreg], %[bods_bodse]" "\n\t" \
                       "out %[mcucr], %[tempreg]" "\n\t" \
                       "andi %[tempreg], %[not_bodse]" "\n\t" \
                       "out %[mcucr], %[tempreg]" \
                       : [tempreg] "=&d" (tempreg) \
                       : [mcucr] "I" _SFR_IO_ADDR(MCUCR), \
                         [bods_bodse] "i" (_BV(BODS) | _BV(BODSE)), \
                         [not_bodse] "i" (~_BV(BODSE))); \
  //} while (0);
}
#endif




inline void	lowPowerBodOn(byte mode) {
  //do {
    set_sleep_mode(mode);
    cli();
    sleep_enable();
    sei();
    sleep_cpu();
    sleep_disable();
    sei();
  //} while (0);
}




inline void lowPowerBodOff(byte mode) {
  //do {
    //lph;
    set_sleep_mode(mode);
    cli();
    sleep_enable();
    sleep_bod_disable();
    sei();
    //lph;
    sleep_cpu();
    sleep_disable();
    sei();
  //} while (0);
}




inline void wdtsleep(period_t period) {
  if (period != _forever){
    mc_wdt_will_reset=0;
    wdt_enable(period);
    WDTCSR |= (1 << WDIE);
  //!!!!
  } else {wdt_disable();}
}


/*
#define t2begin(timer2,a) \
  unsigned char clockSource = 0;\
  if (timer2 == t20){\
    if (TCCR2B & CS22) clockSource |= (1 << CS22);\
    if (TCCR2B & CS21) clockSource |= (1 << CS21);\
    if (TCCR2B & CS20) clockSource |= (1 << CS20);\
    TCCR2B &= ~(1 << CS22);TCCR2B &= ~(1 << CS21);TCCR2B &= ~(1 << CS20);\
    a;\
  }
#define t2end(timer2,a) \
  if (timer2 == t20){\
    if (clockSource & CS22) TCCR2B |= (1 << CS22);\
    if (clockSource & CS21) TCCR2B |= (1 << CS21);\
    if (clockSource & CS20) TCCR2B |= (1 << CS20);\
    a;\
  }
*/

//==================idle
void idle(period_t period){
  byte adc_ = (ADCSRA & (1 << ADEN));
  adc_off();power_adc_disable();
  power_spi_disable();
  power_usart0_disable();
  wdtsleep(period);
  lowPowerBodOn(SLEEP_MODE_IDLE);
  if (adc_>0) {power_adc_enable();adc_on();}
  power_spi_enable();
  power_usart0_enable();
}


/*
//==================idle
//!!!make errors in elf file
void idle(period_t period,adc_t adc,
                          timer2_t timer2,timer1_t timer1, timer0_t timer0,
                          spi_t spi, usart0_t usart0,twi_t twi){


  byte lt2=TCCR2B;
  if (timer2 == t20)  {
    TCCR2B&=~((1 << CS22)|(1 << CS21)|(1 << CS20));
    power_timer2_disable();
  }
  //t2begin(timer2,power_timer2_disable());
  if (adc == a0){adcoff();power_adc_disable();}

  if (timer1 == t10)	power_timer1_disable();
  if (timer0 == t00)	power_timer0_disable();
  if (spi == s0)		power_spi_disable();
  if (usart0 == u0)	power_usart0_disable();
  if (twi == t0)		power_twi_disable();

  wdtsleep(period);
  lowPowerBodOn(SLEEP_MODE_IDLE);
  if (adc == a0){power_adc_enable();adcon();}

  //t2end(timer2,power_timer2_enable());
  if (timer2 == t20)  {
    TCCR2B=lt2;
    power_timer2_enable();
  }
  if (timer1 == t10)	power_timer1_enable();
  if (timer0 == t00)	power_timer0_enable();
  if (spi == s0)		power_spi_enable();
  if (usart0 == u0)	power_usart0_enable();
  if (twi == t0)		power_twi_enable();

}
*/

//===============adcnoisereduction
inline void adcnr(period_t period,timer2_t timer2) {
  byte lt2=TCCR2B;
  //t2begin(timer2,);
  if (timer2 == t20)  {
    TCCR2B&=~((1 << CS22)|(1 << CS21)|(1 << CS20));
    power_timer2_disable();
  }
  wdtsleep(period);
  lowPowerBodOn(SLEEP_MODE_ADC);
  //t2end(timer2,);
  if (timer2 == t20)  {
    TCCR2B=lt2;
    power_timer2_enable();
  }
}


//================pwrsave
inline void pwrsave(period_t period,bod_t bod,timer2_t timer2) {
  byte lt2=TCCR2B;
  //t2begin(timer2,);
  if (timer2 == t20)  {
    TCCR2B&=~((1 << CS22)|(1 << CS21)|(1 << CS20));
    power_timer2_disable();
  }
  wdtsleep(period);
  if (bod == b0) {
    lowPowerBodOff(SLEEP_MODE_PWR_SAVE);
  } else  {
    lowPowerBodOn(SLEEP_MODE_PWR_SAVE);
  }
  //t2end(timer2,);
  if (timer2 == t20)  {
    TCCR2B=lt2;
    power_timer2_enable();
  }
}


//================extstandby
inline void extstandby(period_t period,bod_t bod,timer2_t timer2) {
  byte lt2;
  //t2begin(timer2,);
  if (timer2 == t20)  {
    lt2==TCCR2B;
    TCCR2B&=~((1 << CS22)|(1 << CS21)|(1 << CS20));
    power_timer2_disable();
  }
  wdtsleep(period);
  if (bod == b0) {
    lowPowerBodOff(SLEEP_MODE_EXT_STANDBY);
  } else  {
    lowPowerBodOn(SLEEP_MODE_EXT_STANDBY);
  }
  //t2end(timer2,);
  if (timer2 == t20)  {
    power_timer2_enable();
    TCCR2B=lt2;
  }
}


//==============standby
inline void standby(period_t period, bod_t bod) {
  wdtsleep(period);
  if (bod == b0) {
    lowPowerBodOff(SLEEP_MODE_STANDBY);
  } else  {
    lowPowerBodOn(SLEEP_MODE_STANDBY);
  }
}


//===============pwrdown
//to wait all chars be sended through uart
//before pwrdown, use:
//while (UCSR0B & (1<<UDRIE0)) {;} dus(120);
static void pwrdown(period_t period,bod_t bod) {
  
  wdtsleep(period);
  if (bod == b0) {
    lowPowerBodOff(SLEEP_MODE_PWR_DOWN);
  } else {
    lowPowerBodOn(SLEEP_MODE_PWR_DOWN);
  }
  
}




#define delay(a) dms(a)
//18
#define dms15 extstandby(_15,b0,t21);
//37
#define dms30 extstandby(_30,b0,t21);
//73
#define dms60 extstandby(_60,b0,t21);
//146
#define dms120 extstandby(_120,b0,t21);
//292
#define dms250 extstandby(_250,b0,t21);
//583
#define dms500 extstandby(_500,b0,t21);
//1.16
#define dms1s extstandby(_1s,b0,t21);
//2.33
#define dms2s extstandby(_2s,b0,t21);
//4.66
#define dms4s extstandby(_4s,b0,t21);
//9.32
#define dms8s extstandby(_8s,b0,t21);



//========================
//   port procs
//========================
//
#define _0 (1<<0)
#define _1 (1<<1)
#define _2 (1<<2)
#define _3 (1<<3)
#define _4 (1<<4)
#define _5 (1<<5)
#define _6 (1<<6)
#define _7 (1<<7)

//output
#define pbo(m) DDRB |=(m)
#define pco(m) DDRC |=(m)
#define pdo(m) DDRD |=(m)
//input
#define pbi(m) DDRB &=~(m)
#define pci(m) DDRC &=~(m)
#define pdi(m) DDRD &=~(m)
//high
#define pbh(m) PORTB |=(m)
#define pch(m) PORTC |=(m)
#define pdh(m) PORTD |=(m)
//input
#define pbl(m) PORTB &=~(m)
#define pcl(m) PORTC &=~(m)
#define pdl(m) PORTD &=~(m)
//xor
#define pbx(m) PORTB ^=(m)
#define pcx(m) PORTC ^=(m)
#define pdx(m) PORTD ^=(m)
//read
#define pbr(m) (PINB&(m))
#define pcr(m) (PINC&(m))
#define pdr(m) (PIND&(m))

//==AnalogRead
#define admux_ref_vcc   (1<<REFS0)
#define aread_refs ((1<<REFS0)|(1<<REFS1))
#define adc_max_points (word)1024
word aread(byte ladmux)
{
    if ((ADMUX&aread_refs)!=(ladmux&aread_refs))
    {
        ADMUX=ladmux;
        dms(20);
    }
    else
    {
        ADMUX=ladmux;
        //dus(10);//dus(10)
    }
    ADCSRA=0xc5;// B11000101-250kHz for 8Mhz
    //ADCSRA |= _BV(ADSC);
    while (ADCSRA & (1 << ADSC));
    byte low = ADCL;
    byte lhigh=ADCH;
    return (word)(lhigh<<8)|low;
}

word vcc_read(void)
{
    return (1.1*adc_max_points*100)/
            aread(admux_ref_vcc|0xE);
}


word aread_to_v(word laread,word lvcc)
{
    //main voltage
    //word lvcc=readvcc()
    //laread*(5000)/1024
    //X/1024
    return 0;
}

//==Analog READ
#define a1r (aread(B01000001))
#define a1r1 (aread(B11000001))
//


//word aread_to_v(word laread,word lvcc)
//{
//    //main voltage
//    //word lvcc_mc=readvcc()*10;
//    //
//    
//    //laread*(5000)/1024
//    //X/1024
//    return 0;
//}


//==Small UART
//get voltage aref=vcc
//word readv(byte lmask,word r1k,word r2k){
//  if (lmask==0) return 0;
//  byte adc_ = (ADCSRA & (1 << ADEN));
//  if (adc_ == 0) {power_adc_enable();adc_on();}
//  byte li=0;
//  lmask>>=1;
//  while (lmask>0) {li++;lmask>>=1;}
//  li+=_6;
//  word la=aread(li);
//  //we need this dms!!!
//  dms(20);
//  la=aread(li);
//  word lmin=aread(li);
//  byte ln=4;
//  while (ln-->0) {la=aread(li);if (la<lmin) {lmin=la;}}
//  la=((dword)lmin*readvcc()/1023);
//  if (adc_ == 0) {adc_off();power_adc_disable();}
//  //dword
//  return ((dword)la*(r1k+r2k))/r2k;
//}






//=======================
// pseudorandom
//=======================
inline byte atmega328p_randbr() {
  ADCSRA |= _BV(ADSC); // Start conversion
  //while (bit_is_set(ADCSRA,ADSC)); // measuring
  byte low  = ADCL&1; // must read ADCL first - it then locks ADCH
  byte high = ADCH; // unlocks both
  //sph(high);sps;sph(low);spn;
  return  low;
}
//
inline byte atmega328p_randbr2() {
  register byte a;
  register byte li=10;//30
  do {
    a=atmega328p_randbr() | (atmega328p_randbr()<<1);
    // 1 to 0 transition: log a zero bit
    if (a==1) {return 0;}
    // 0 to 1 transition: log a one bit
    if (a==2) {return 1;}
    // For other cases, try again.
  } while (--li>0);
  return a&1;
}
//
inline byte atmega328p_randbit() {
  register byte a;
  register byte li=11;//20
  do {
    a=atmega328p_randbr2()|(atmega328p_randbr2()<<1);
    // 1 to 0 transition: log a zero bit
    if (a==1) {return 0;}
    // 0 to 1 transition: log a one bit
    if (a==2) {return 1;}
    // For other cases, try again.
  } while (--li>0);
  return a&2;
}
//laportc = 0-7
byte atmega328p_random(byte laportc){
  register byte ladmux=ADMUX;
  register byte adc_ = (ADCSRA & (1 << ADEN));
  if (adc_ == 0) {power_adc_enable();adc_on();}

  // Read 1.1V reference against Temp
  //ADMUX = _BV(REFS0) | _BV(REFS1) | _BV(MUX3) ;
  //read 1.1V against VCC
  //ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  //ADMUX =_BV(REFS0) |  _BV(MUX3) | _BV(MUX2) | _BV(MUX1)| _BV(MUX0);
  //sph(ADMUX);spn;
  //A0
  ADMUX=_BV(REFS0) | (laportc&0x7);
  //sph(ADCSRA);spn;
  //dms(5);
  register byte lr=0;
  for (byte i=8; i--;i>0) lr+=lr+atmega328p_randbit();

  ADMUX=ladmux;
  if (adc_ == 0) {adc_off();power_adc_disable();}

  return lr;
}
//
inline byte atmega328p_randb() {
  register byte a;
  register byte li=11;//20
  do {
    //vcc
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
    a=atmega328p_randbr2();
    //temp
    ADMUX = _BV(REFS0)|_BV(REFS1)|_BV(MUX3);
    a|=(atmega328p_randbr2()<<1);
    // 1 to 0 transition: log a zero bit
    if (a==1) {return 0;}
    // 0 to 1 transition: log a one bit
    if (a==2) {return 1;}
    // For other cases, try again.
  } while (--li>0);
  return a&2;
}
//rand
byte atmega328p_rand(){
  register byte ladmux=ADMUX;
  register byte adc_ = (ADCSRA & (1 << ADEN));
  if (adc_ == 0) {power_adc_enable();adc_on();}
  register byte lr=0;
  for (byte i=9; i--;) lr+=lr+atmega328p_randb();
  ADMUX=ladmux;
  if (adc_ == 0) {adc_off();power_adc_disable();}
  return lr;
}



//===========================
#define atmega328p_init()  \
  MCUSR=0;wdt_disable(); \
  adc_off();power_adc_disable();ac_off();\
  power_timer0_disable();\
  power_timer1_disable();\
  power_timer2_disable();
  //sei();//sleep_bod_disable();

//MCUSR &= ~(1<<WDRF);
//power_all_disable();
//SPI.setClockDivider(SPI_CLOCK_DIV2)



//---------------
// UART
//cyclic buffer for uart
#define serial_buf_max 20
byte serial_buf[serial_buf_max];
//index
volatile byte serial_buf_i;
//==1 if we have got '/n'
volatile byte serial_has_input;
//save uart incoming bytes to cyclic buffer
ISR(USART_RX_vect){
  register byte lb=UDR0;
  if ((lb=='\n')||(lb=='\r')) {serial_has_input=1;}
  else if (serial_buf_i<serial_buf_max) {serial_buf[serial_buf_i++]=lb;}
}
//clear buf
static void serial_buf_clear(){
  cli();serial_buf_i=0;serial_has_input=0;sei();
}

#define USART_BAUDRATE 38400
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)
static void serial_init(){
  serial_buf_clear();
  // Turn on the transmission and reception circuitry
  UCSR0B = (1 << RXEN0) | (1 << TXEN0);
  // Use 8-bit character sizes
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
  // Load lower 8-bits of the baud rate value into the low byte of the UBRR register
  UBRR0L = BAUD_PRESCALE;
  // Load upper 8-bits of the baud rate value into the high byte of the UBRR register
  UBRR0H = (BAUD_PRESCALE >> 8);
  // Enable the USART Recieve Complete interrupt (USART_RXC)
  UCSR0B |= (1 << RXCIE0);
}
static void serial_write(byte lb){
  while (!(UCSR0A & (1<<UDRE0)));
  UDR0 = lb;
}
static void serial_sendArray(byte *buffer, word bufferSize){
    for(word i=0; i<bufferSize; i++) serial_write(buffer[i]);
}
static void serial_sendString(const char *lp){
    char lc;
    while (1) {
        lc=*lp++;
        if (lc==0) break;
        serial_write(lc);
    }
}

#define _serial_wait 120

//!!does not work
//wait till uart will send all deb info
static void serial_wait(byte lus){
   if ((UCSR0B & (1<<UDRIE0))!=0) {
    //  lph;
    word lwait=60000;//4000
    while (lwait-->0) {
      if ((UCSR0B & (1<<UDRIE0))==0) {dus(lus);break;}
      dus(10);
    }
  }
}

//---------------------
#ifdef _loguart
/*
        //#include <HardwareSerial.h>

        void logb(){serial_init();sb();}
        void logo(){
            if (logi>0) {
                 for(byte li=0;li<logi;li++) {
                    serial_write(loga[li]);
                 }
            }
            serial_write('\r');
            serial_write('\n');
        }
        void loge(){;}
*/
#endif
//-------------------------
//copy from flash to ram
static void cpf(byte * lfp,word ln,byte * lr){
  while (ln-->0) {
    *lr++=pgm_read_byte(lfp++);
  }
}
//--------------
template <typename T> void cp_fromflash(const T * sce, T& dest){
  memcpy_P (&dest, sce, sizeof (T));
}
//--------------
template <typename T> T get_fromflash(const T * sce){
  static T temp;
  memcpy_P (&temp, sce, sizeof (T));
  return temp;
}

#endif
