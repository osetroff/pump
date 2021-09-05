//pump control for dab divertron
//can accept and send power on status to another pump controller
//atmega328p 8mhz int dip28
//20210811
#include "k_atmega328p_8mhz.cpp"

//-----------------
// debug output to serial
#define _deb

#ifdef _deb
#define l(a) a
#else
#define l(a)
#define ldh
#define ldl
#endif

// one common pipe to feed from 2 pumps
// one pump work for 2 pipes


/*
 
atmega328p dip28 VCC=5V
 
relay in 5v out 220V 10a 
 
pressure sensor VCC=5V 10ma out 0.5 - 4.5V 0-12bar

reset vcc to 10K. DTR 0.1u to RST   |01 RST     SCL  C5 28|  
usart rx                            |02 D0 RXD  SDA  C4 27|  
usart tx                            |03 D1 TXD       C3 26|
                                    |04 D2 INT0      C2 25|
RS485 RO                       330R |05 D3 INT1      C1 24| 100R in pressure sensor2 analog 0.4-4.5v
LED1 1K                             |06 D4           C0 23| 100R in pressure sensor1 analog 0.4-4.5v
VCC                                 |07 VCC         GND 22| GND
GND                                 |08 GND        aref 21| 0.1u to gnd
pump num jumper0 330R to vcc or gnd |09 B6         AVCC 20| VCC
pump num jumper1 330R to vcc or gnd |10 B7      SCK  B5 19|
humidity sensor                     |11 D5  T1  MISO B4 18|
LED2 1k                             |12 D6      MOSI B3 17|
RS485 DI   330R                     |13 D7           B2 16|
RS485 notRE/DE . HIGH to send  330R |14 B0           B1 15| pump relay 220V on/off 1/0 out 330R
 
 * we have led om pump relay
 */


//-----------------
// Watchdog
#define wdt_on()    mc_wdt_will_reset=1;wdt_enable(_2s)
#define wdt_off()   wdt_disable()

//-----------------
// random
static volatile u8 mc_rand;
static inline void mc_rand_add(u8 ladd)
{
    mc_rand+=ladd;
}
//not zero random
static volatile u8 mc_randnz=1;
static inline void mc_randnz_add(u8 ladd)
{
    mc_randnz+=ladd;
    if (mc_randnz==0)
    {
        mc_randnz=ladd;
    }
}

//-----------------
// led pins

//PD4
//pin-1k-LED+
//GND-LED-
#define led1_port D
#define led1_pin _4
inline static void led1_pin_high(void){pdh(led1_pin);}
inline static void led1_pin_low(void){pdl(led1_pin);}
#define led1_pin_init() pdo(led1_pin);led1_pin_low();

//PD6
//pin-1k-LED+
//GND-LED-
#define led2_port D
#define led2_pin _6
inline static void led2_pin_high(void){pdh(led2_pin);}
inline static void led2_pin_low(void){pdl(led2_pin);}
#define led2_pin_init() pdo(led2_pin);led2_pin_low();

inline static void initLedPins(void)
{
    led1_pin_init();
    led2_pin_init();
}


//main led 
#define led_main_high() led1_pin_high() 
#define led_main_low() led1_pin_low() 

#ifdef _deb
inline static void led_pin_high(void){ led2_pin_high();}
inline static void led_pin_low(void){led1_pin_low();}
#define led_er_high() 
#define led_er_low() 
#else
//error led 
#define led_er_high() led2_pin_high() 
#define led_er_low() led2_pin_low() 
#endif


//debug led and main led
inline static void led_deb_high(void){led2_pin_high();}
inline static void led_deb_low(void){led2_pin_low();}


//-----------------
// relay pins

//PB1
// pin-330R-DIOD-RELAY_IN
// GND-10K-RELAY_IN 
#define relay_port B
#define relay_pin _1
inline static void relay_pin_high(void){pbh(relay_pin);}
inline static void relay_pin_low(void){pbl(relay_pin);}
#define relay_pin_init() pbo(relay_pin);relay_pin_low()
 
#define relay_is_on() pbr(relay_pin) 

inline static void initRelayPins(void)
{
    relay_pin_init();
}


#define pump_off()      relay_pin_low()
#define pump_on()       relay_pin_high()
#define pump_is_on()    relay_is_on()








//-----------------
// pressure sensor pins

//PC0
// pin-100R-ANALOG_SENSOR_OUT
#define press_port C

#define press1_pin      _0
#define press1_pin_init() pci(press1_pin)
#define press1_admux    admux_ref_vcc|0

#define press2_pin      _1
#define press2_pin_init() pci(press2_pin)
#define press2_admux    admux_ref_vcc|1

#define press_vcc_pin   _2
#define press_vcc_pin_init() pci(press_vcc_pin)
#define press_vcc_admux admux_ref_vcc|2


inline static void initPressPins(void)
{
    press1_pin_init();
    press2_pin_init();
    press_vcc_pin_init();
}






//--------------------
// rs485
//PD3 R0 INT1
//PD7 DI
//PB0 DE high to send
// pin-330R-max485e


#define rs485ro_pin    _3
#define rs485ro_pin_init() pdi(rs485ro_pin)
#define rs485ro_is_on() pdr(rs485ro_pin)

#define rs485di_pin    _7
inline static void rs485di_pin_high(void){pdh(rs485di_pin);}
inline static void rs485di_pin_low(void){pdl(rs485di_pin);}
#define rs485di_pin_init() pdo(rs485di_pin);rs485di_pin_high()

#define rs485de_pin     _0
inline static void rs485de_pin_send(void){pbh(rs485de_pin);}
inline static void rs485de_pin_read(void){pbl(rs485de_pin);}
#define rs485de_pin_init() pbo(rs485de_pin);rs485de_pin_read()


inline static void initRs485Pins(void)
{
    rs485ro_pin_init();
    rs485di_pin_init();
    rs485de_pin_init();
    //ldl;
    
}

//----------------------
// rs485

// msg
// 0xaa start byte
// FFFFTTTT byte has info about msg from (F 4 bits) to (T 4 bits)
// rs485 bufs
#define rs485_bufs_max  12
static u8 rs485_bufs[rs485_bufs_max];

//ciclic buffer for data average
static u8 rs485_bufr_i;//index for new data to save
//rs485_bufri 0-0x3E (0-63, 64 maximum)
//bitmask rs485_bufr is full
#define rs485_bufr_isfullmask 0x80
//bitmask rs485_bufr is stop
#define rs485_bufr_isstopmask 0x40
//! lower than rs485_bufr_isstopmask 
#define rs485_bufr_max  12
//assert(rs485_bufr_max>=rs485_bufr_isstopmask)
static u8 rs485_bufr[rs485_bufr_max];

#define rs485_bufr_datalen(i) i&(~(rs485_bufr_isstopmask|rs485_bufr_isfullmask))
//------
#define rs485_bufr_start(i) i=0
//-------
#define rs485_bufr_stop(i) i|=rs485_bufrisstopmask
//-------------
//if was overflow - so we can use rs485_bufr_avg
#define rs485_bufr_isfull(i) ((i&rs485_bufr_isfullmask)!=0)
//--------------
#define rs485_bufr_isstop(i) ((i&rs485_bufr_isstopmask)!=0)
//--------------
#define rs485_bufr_save(ldata8,buf,i) {\
    if ((i&rs485_bufr_isstopmask)==0) {\
        u8 li=i&(~(rs485_bufr_isstopmask|rs485_bufr_isfullmask));buf[li]=ldata8;\
        if (li==((sizeof(buf))-1)) {i=rs485_bufr_isfullmask;} else {i++;}}}

//-------------
#define sprs485_bufr(buf,i) { u8 llen;\
    if (rs485_bufr_isfull(i)) {llen=sizeof(buf);} \
    else {llen=i&(~(rs485_bufr_isstopmask|rs485_bufr_isfullmask));}\
    if (llen>0) {u8 li=0; do { u8 ls=5; do {\
                sp(buf[li++]); if (li>=llen) {break;} sps;\
            } while (--ls); spn; } while (li<llen); }}



//address of current pump
static u8 rs485_pump_addr;
static void initRs485(void)
{
    rs485_pump_addr=eerb(1023);
    rs485_bufr_start(rs485_bufr_i);
    int1_on_fall();
    int1en();
}




//-------------------
// read byte from rs485
//rs485 9600 1 n halfbit time
#define rs485_halfbit_us    (u8)51
ISR(INT1_vect){
  register byte lb=0;
  register byte l8=8;
  //ldh;
  //int1dis();
  cli();
  
  //skip stop bit
  dus(3*rs485_halfbit_us);
  
  while (1) 
  {
      //ldh;
      //data bit 0
      if (rs485ro_is_on())
      {
          lb|=0x80;
          //ldh;
      }
      else
      {
          //ldl;
      }
      //ldl;
      if (--l8==0) {break;}
      
      //wait next bit
      dus(2*rs485_halfbit_us);
      //>>
      lb>>=1;
  }
  rs485_bufr_save(lb,rs485_bufr,rs485_bufr_i);
  //ldl;
  int1clr();
  sei();
}

//---------------
//send byte through rs485
static void rs485_send(u8 lb)
{
    register byte l8=8;
    mc_rand_add(lb);
    
    cli();
    
    //send mode
    rs485de_pin_send();
    
    //stop bit
    rs485di_pin_low();
    dus(2*rs485_halfbit_us);
  
    while (1) 
    {
      //ldh;
      //data bit 0
      if ((lb&1)==0)
      {
          rs485di_pin_low();
          //ldl;
      }
      else
      {
          rs485di_pin_high();
          //ldh;
      }
      //ldl;
      
      //wait next bit
      dus(2*rs485_halfbit_us);
      
      if (--l8==0) {break;}
      
      //>>
      lb>>=1;
    }
    rs485di_pin_high();
    //ldl;
    
    int1clr();
    sei();
    //read mode
    rs485de_pin_read();
}

static void mc_rand_pause_us(void)
{
    if (mc_rand==0)  {mc_rand=mc_randnz;}
    dus(mc_rand);
}
//--------------------
//send buf by rs485
static void rs485_send_buf( u8 * lp,
                            u8 len)
{
    while (len-->0)
    {
        rs485_send(*lp++);
        //random pause
        if (mc_rand<(2*rs485_halfbit_us))
        {
            dus(2*rs485_halfbit_us);
        }
        else
        {
            mc_rand_pause_us();
        }
    }
}
//--------------------
// serial output
//--------------------
inline static void sp(char lc){serial_write(lc);}
#define spn sp(0xd);sp(0xa)
#define sps sp(' ')
#define sp0 sp('0')
#define spm sp('-')
inline static void s(const char *ls){serial_sendString(ls);}
//sph hex
static inline void sphc(char a){if (a>9) {a+='A'-10;} else {a+='0';} sp(a);}
static void sph(uint8_t a) {sphc(a>>4);sphc(a&0xF);}
static void sph16(uint16_t a) {sph(a>>8);sph(a&0xFF);}

//decimal output in l0 places
//l0=1
static void sp8(uint8_t la,uint8_t l0=0){
 uint8_t li;
 uint8_t lz=0;
 if (la>=100) {
     li=la/100;
     sp(0x30+li);
     la-=(li*100);
     lz=1;
 } else {if (l0>2) {sp0;}}
 if (la>=10) {
     li=la/10;
     sp(0x30+li);
     la-=(li*10);
 } else {if ((l0>1)||(lz>0)) {sp0;}}
 sp(0x30+la);
}

//l0=1
static void sp16(uint16_t la,uint8_t l0=0) {
     uint16_t l10=10000;
     uint8_t l0i=5;
     uint8_t li;
     while (l10>1){
         if (la>=l10) {
             l0=l0i;
             li=la/l10;
             sp(0x30+li);
             la-=(li*l10);
         } else if (l0>=l0i) {sp0;}
         l10/=10;
         l0i--;
     }
     sp(0x30+la);
}


//-------------------
//decimal out, max 2 digit after comma
//ldec=0,1,2,3 decimal digits after comma
static void spdec(int16_t lt,uint8_t ldec,uint8_t l0){
    if (lt<0) {spm;lt=-lt;} else {sps;}
    if (ldec==0) {sp16(lt,l0);}
    else {
        uint16_t ld;
        if (ldec==1) 
        {
            ld=10;
        } else if (ldec==2)
        {
            ld=100;
        } else 
        {
            ld=1000;
        }
        sp16((uint16_t)(lt/ld),l0);
        sp('.');
        sp8((uint8_t)(lt%ld),ldec);
    }
}









//time in seconds pump is on
static u16 time_pump_on;

enum led_t {
    led_no,
    led_low,
    led_high,
    led_empty,
    led_off_on,
    led_error,
};



#define mc_pwrdown(lperiod) pwrdown(lperiod,b1);wdt_on()


static void led_main_blink250(void)
{
    led_main_low();
    mc_pwrdown(_120);
    led_main_high();
    mc_pwrdown(_120);
}



//delay
static void delay_sec_pwrdown(  u16 lsec,
                                led_t lled)
{
    
    
    //wait log output
    dus(400);
    
    if (lled==led_error)
    {
        led_main_low();
    }
    
    
    u8 lodd=0;
    while (lsec--)
    {
        lodd=1-lodd;
        
        
        if ((lodd==0)||(lled==led_no))
        {
            //no blink
            mc_pwrdown(_1s);
        }
        else 
        {
            //blink
            if (lled==led_low)
            {
                mc_pwrdown(_500);
                led_main_blink250();
                led_main_blink250();
            }
            else if (lled==led_high)
            {
                mc_pwrdown(_250);        
                led_main_blink250();
                led_main_blink250();
                led_main_blink250();
            }    
            else if (lled==led_empty)
            {
                mc_pwrdown(_500);
                mc_pwrdown(_250);
                led_main_blink250();
            }    
            else if (lled==led_off_on)
            {
                led_main_blink250();
                led_main_blink250();
                led_main_blink250();
                led_main_blink250();
            }    
            //
            led_main_low();
        }
        
        if (pump_is_on()) 
        {
            time_pump_on++;
            if (time_pump_on==0){time_pump_on--;}
        }
    }
    dus(50);
    
}







//static void testh(){
//    l(
//    sph16(testheap());sps;sph16(testsp());spn;
//    );
//}



void test_blink(void)
{
    // init
    atmega328p_init();
    initLedPins();
    initRs485Pins();
    
    //----------
    initRs485();
    serial_init();
    // main loop
    volatile u8 li=0;
    sph(rs485_pump_addr);spn;
    dus(1000);

    wdt_on();
    while (1) {
        led_main_high();
        led_er_low();
        mc_pwrdown(_250);
        
        led_main_low();
        led_er_high();
        //dms(1200);
        //wdt_reset();
        //dms(1200);
        mc_pwrdown(_250);

        u8 llen;
        cli();
        llen=rs485_bufr_datalen(rs485_bufr_i);
        sei();
        if (llen!=0)
        {
            u8 li=0;
            u8 ln=llen;
            while (llen--)
            {
                sph(rs485_bufr[li++]);sps;
            }
            spn;
            //echo
            rs485_send_buf(rs485_bufr,ln);
            //
        }
            
        dus(1000);
        //wdt_reset();
    }
}

//=========================
//data from adc
static struct adc_s
{
    //water pressure for sensor1
    //for ex. 01 means 0.1 bar 
    volatile u16 press1;
    //water pressure for sensor2
    volatile u16 press2;
    //voltage of sensors power source
    //for ex. 502 means 5.02 V 
    volatile u16 vcc_press;
    //voltage of mc
    //for ex. 502 means 5.02 V 
    volatile u16 vcc_mc;
} adc;

//log adc
void spadc(void)
{
    //spn;
    spdec(adc.vcc_mc,2,0);sp('/');
    spdec(adc.vcc_press,3,0);s(" v");spn;
    spdec(adc.press1,1,0);sp('/');
    //s("press2,bar ");sp16(press2,3);sps;
}

//press in bar
static u16 press_adc_to_bar(u16 lpress)
{
    return (((((long)lpress*adc.vcc_press)/adc_max_points)-
            200)*12)/450;
}

//ret 0 if ok
static u8 adc_read(void)
{
    adc_s ladc;
    // ADC
    power_adc_enable();adc_on();
    //dms(2);
    //to stabilize
    vcc_read();
    vcc_read();
    vcc_read();
    vcc_read();
    //read all adc data
    adc.vcc_mc=vcc_read();
    ladc.vcc_press=aread(press_vcc_admux);
    ladc.press1=aread(press1_admux);
    ladc.press2=aread(press2_admux);
    adc_off();power_adc_disable();
    
    //-----------
    //calculations
    //sp16(ladc.vcc_press,4);sps;
    //press_vcc to volts
    //press_vcc=((long)press_vcc*(vcc*10))/adc_max_points;
    //restore after divider r1=33k r2=100k
    //ladc.vcc_press=((long)ladc.vcc_press*133)/100;
    adc.vcc_press=(((long)ladc.vcc_press*10*133/adc_max_points)*
                    adc.vcc_mc)/100;
    
    //-----------
    //calc press1 in volts
    //sp16(ladc.press1,4);spn;
    adc.press1=press_adc_to_bar(ladc.press1);
    
    sps;sp16(ladc.press1);s(" press1");spn;
    
    return 0;
 }







//===============================
// variants

#define var_state_continue 0
#define var_state_exit     1

//variable to exit from any var_procedure
static u8 var_state;


 

//-------------------
//test variant
static u8 var_test(void)
{
    s("var_test");spn;
    
    while (var_state==var_state_continue) 
    {
    //
    led_main_high();
    led_er_high();
    relay_pin_high();
    dms(1000);
    relay_pin_low();
    led_er_low();
    led_main_low();
    dms(1000);

    adc_read();
    spn;
    }
    return 0;
}























//--------------------
// DAB divertron single pump mode without back valve inside pump
//--------------------




//delay after well was empty in seconds
u16 empty_delay;

struct pump_single_const_s
{
    //all pressure in 0.1bar (1 means 0.1)
    
    u16 press_min;//min pressure
    u16 press_max;//max pressure
    u16 press_dif_error;//difference between two continuous measurements
    u16 press_max_error;//max presure pump cant do
    
    //all time in seconds
    
    u16 empty_time_pump_on_min;
    u16 empty_delay_min;//
    u16 empty_delay_max;//
    u16 empty_delay_step;//
    
    u16 delay_off_on_pause;//between off and on again 
    u16 delay_low_press;//
    u16 delay_high_press;//
    u16 delay_measure;//
    u16 delay_start;//
    
    u16 time_max_pump_on;
    
    u16 delay_error;
};

const pump_single_const_s pump_single={
2,//press_min
30,//press max
10,//press diff error
34,//press max error

15*60,//min*sec pump on min
2*60*60,//hour*min*sec empty delay min
6*60*60,//hour*min*sec empty delay max
30*60,//min*sec empty delay step

20,//delay on off
15,//delay when get low pressure
30,//delay when get high pressure
1,//seconds between measuring pressure
40,//delay_start

2*60*60,//hour*min*sec max pump on time

(u16)12*60*60,//hour*min*sec delay if pressure sensor error
};

void sp_single_log(void)
{
    spadc();
    
    //sph(pump_is_on());spn;
    if (pump_is_on())
    {
        s(" on: ");sp16(time_pump_on);
    }
    
    spn;
    spn;
}


//--------------------------------
// var with single pump without BV
//--------------------------------

//TODO WATCHDOG!!!
static u8 var_single_without_back_valve_press1(void)
{
    led_er_high();
    led_main_high();
    wdt_on();
    s("var_single_without_back_valve_press1");spn;
    
    //
    pump_off();
    
    //set default empty_delay between empty well
    empty_delay=pump_single.empty_delay_min;
    
    //delay start
    delay_sec_pwrdown(pump_single.delay_start,led_no);
    
    
    led_er_low();        
    while (var_state==var_state_continue) 
    {
        // LED is on
        led_main_high();
        
        
        
        
        
        
        
        
        //-------------------
        //check pressure sensor error
        
        //measure pressure
        adc_read();
        //log
        sp_single_log();
        
        //save
        u16 lpress1=adc.press1;
        u8 le=0;
        
        
        //measure pressure
        adc_read();
        //log
        sp_single_log();
        
        
        //check press sensor
        if ((lpress1>=pump_single.press_max_error)||
             (adc.press1>=pump_single.press_max_error))
        {
            le=1;
        }
        else    
        {
            if (lpress1>adc.press1)
            {
                if ((lpress1-adc.press1)>
                    pump_single.press_dif_error) 
                {
                    le=1;
                    //sp16(lpress1-adc.press1);spn;
                }
            }
            else
            {
                if ((adc.press1-lpress1)>
                    pump_single.press_dif_error) 
                {
                    le=1;
                    //sp16(adc.press1-lpress1);spn;
                } 
            }
        }
        

        //if error of pressure sensor
        if (le)
        {
            led_er_high();
            //pressure sensor error
            s(" poff (press err)");spn;
            pump_off();
            
            //delay high_pressure
            delay_sec_pwrdown(
                pump_single.delay_error, led_error);
            led_er_low();
            continue;
        }
//        else
//        {
//            s("ok");spn;
//        }
//        
        
        
        
        
        
        
        //--------------------
        //check max on time
        if ((pump_is_on())&&
                (time_pump_on>pump_single.time_max_pump_on))
        {
            spn;s("! max on time");spn;
        }
        
        
        
        
        
        
        //------------------------
        // main
        if (adc.press1<=pump_single.press_min)
        {
            if (pump_is_on())
            {
                //pump is on but press1 is low -> no water?
                s(" low (on)");spn;
                
                //TODO led
                
                //delay low_pressure
                delay_sec_pwrdown(
                        pump_single.delay_low_press,led_low);
                
                //measure pressure
                adc_read();
                
                //log
                sp_single_log();
                
                
                if (adc.press1<=pump_single.press_min)
                {
                    //press1 still low -> pump off
                    
                    pump_off();
                    
                    //calculate empty_delay
                    if (time_pump_on<=
                            pump_single.empty_time_pump_on_min)
                    {
                        
                        //pump was on short time -> increase delay
                        if ((empty_delay+pump_single.empty_delay_step)<=
                                pump_single.empty_delay_max)
                        {
                            //increase delay
                            empty_delay+=pump_single.empty_delay_step;
                            
                        }
                    }
                    else
                    {
                        
                        //pump was on good amount time -> decrease delay
                        if (empty_delay>=
                               (pump_single.empty_delay_min+
                                pump_single.empty_delay_step))
                        {
                            empty_delay-=pump_single.empty_delay_step;
                        }
                    }
                    
                    s("* poff (empty) ");sp16(empty_delay);spn;
                    
                    //delay empty_delay
                    delay_sec_pwrdown(empty_delay, led_empty);
                }
            }
            else
            {
                //pump is off and press1 is low -> on pump
                s(" low (off)");spn;
                pump_on();
                //start counter
                time_pump_on=0;
                s("* pon");spn;
            }
        }
        else
        {
            if (pump_is_on())
            {
                
                
                //pump is on
                if (adc.press1>=pump_single.press_max)
                {
                    //press1 is max
                    s(" high (on)");spn;
                    
                    //delay high_pressure
                    delay_sec_pwrdown(
                        pump_single.delay_high_press, led_high);

                    //measure pressure
                    adc_read();
                
                    //log
                    sp_single_log();
                
                
                    if (adc.press1>=pump_single.press_max)
                    {
                        //still high pressure
                        s("* poff (high)");spn;
                        pump_off();
                        
                        //delay high_pressure
                        delay_sec_pwrdown(
                            pump_single.delay_off_on_pause, led_off_on);
                    }
                }

            }
            else
            {
                //pump is off but pressure is low max pressure
                s(" normal (off)");spn;
                pump_on();
                //start counter
                time_pump_on=0;
                s("* pon");spn;
            }
        }
        
        
        
        //delay_measure
        delay_sec_pwrdown(pump_single.delay_measure,led_no);
        
    }
    
    //
    //wdt_disable();
    
    pump_off();

    return 0;
}










//=========================
// MAIN
//=========================
int main(void){

    test_blink();
    
    //-------------
    // init
    atmega328p_init();
    initLedPins();
    initRelayPins();
    initPressPins();
    initRs485Pins();
    
    //-------------
    initRs485();
    //---------------
    // serial
    serial_init();

//#ifdef _deb
//    ldh;
//    dms(1000);
//    ldl;
//#endif    


    //=============================
    // main loop
    while (1) 
    {
        var_state=var_state_continue;
        
        var_single_without_back_valve_press1();
        
        //var_test(); 

    }   
}