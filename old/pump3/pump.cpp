//pump control for dab divertron
//can accept and send power on status to another pump controller
//atmega328p 8mhz int dip28
//20210811
//cutecom for rs485
#include "k_atmega328p_8mhz.cpp"

//-----------------
// debug output to serial
//#define _deb

#ifdef _deb
#define l(a) a
#else
#define l(a)
//#define ldh
//#define ldl
#endif

// one common pipe to feed from 2 pumps
// one pump work for 2 pipes


/*
 
atmega328p dip28 VCC=5V
 
relay in 5v out 220V 10a 
 
pressure sensor VCC=5V 10ma out 0.5 - 4.5V 0-12bar

reset vcc to 10K. DTR 0.1u to RST   |01 RST     SCL  C5 28|  
usart rx                            |02 D0 RXD  SDA  C4 27|  
usart tx                            |03 D1 TXD       C3 26| -20k-gnd , -1k-photoresistor to check daylight
Gnd-button+330R-pin,vcc-10k+        |04 D2 INT0      C2 25| -100k-GND, -33k-VCC_sensor to get VCC sensor
RS485 RO                       330R |05 D3 INT1      C1 24| 100R in pressure sensor2 analog 0.4-4.5v
LED1 1K                             |06 D4           C0 23| 100R in pressure sensor1 analog 0.4-4.5v
VCC                                 |07 VCC         GND 22| GND
GND                                 |08 GND        aref 21| 0.1u to gnd
pump num jumper0 330R to vcc or gnd |09 B6         AVCC 20| VCC
pump num jumper1 330R to vcc or gnd |10 B7      SCK  B5 19|
hall flow sensor                    |11 D5  T1  MISO B4 18|
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


//------------------
#define ee_rs485_pump_addr  1023
struct pump_single_const_s
{
    
    u16 press1_offset_adc;
    u16 press2_offset_adc;
    //all pressure in 0.1bar (1 means 0.1)
    
    u16 press_min;//min pressure
    u16 press_max;//max pressure
    u16 press_dif_error;//difference between two continuous measurements
    u16 press_max_error;//max presure pump cant do
    
    //all time in seconds
    
    u16 empty_pump_on_time_min;
    u16 empty_delay_min;//
    u16 empty_delay_max;//
    u16 empty_delay_step;//
    
    //when we have only one pressure sensor
    //after high pressure if we turn pump off - we will loose pressure
    //so we need big delay to escape from on-off loop
    u16 delay_after_high;
    u16 delay_night;//if we can on pump but it is night
    u16 delay_low_press;//
    u16 delay_high_press;//
    u16 delay_measure;//
    u16 delay_start;//
    
    u16 time_max_pump_on;
    
    u16 delay_error;
};

const pump_single_const_s pump_single={

103,//press1_offset_adc
99,//press2_offset_adc

4,//press_min
30,//press max
10,//press diff error
34,//press max error

10*60,//min*sec pump on min
2*60*60,//hour*min*sec empty delay min
4*60*60,//hour*min*sec empty delay max
30*60,//min*sec empty delay step

60*60,//delay after high
60*60,//delay night
10,//delay when get low pressure
10,//delay when get high pressure
1,//seconds between measuring pressure
30,//delay_start

2*60*60,//hour*min*sec max pump on time

(u16)12*60*60,//hour*min*sec delay if pressure sensor error
};


//-----------------
// button pins

//PD2 int0
//pin-330R-button-gnd

#define btn_port D
#define btn_pin _2
inline static void btn_pin_init(void)
{
    pdi(btn_pin);
    //pull up
    //pdh(btn_pin);
}
inline static void initBtnPins(void)
{
    btn_pin_init();
}

//flag for int,
//==1 when int0 issue
volatile static u8 mc_int0_flag;
//btn int
ISR(INT0_vect){
    int0dis();
    mc_int0_flag=1;
}
//check 
static u8 mc_btn_pressed(void)
{
    u8 lf=mc_int0_flag;
    cli();
    mc_int0_flag=0;
    int0en();
    sei();
    return lf;
}

inline static void initBtn(void)
{
    mc_int0_flag=0;
    int0_on_fall();
    int0en();
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

//check
#define pump_is_on()    relay_is_on()

//pump state
static u8 pump_state;
#define pump_state_def              0

#define pump_state_on               _0
#define pump_state_on_too_long      _1
#define pump_state_high_press       _2
#define pump_state_empty            _3
#define pump_state_er_press         _7


//time in seconds pump is on
static u16 pump_on_time_s;
//inc pump_on_time for 1 sec
static void pump_on_time_inc(void)
{
    if (pump_is_on()) 
    {
        pump_on_time_s++;
        if (pump_on_time_s==0){pump_on_time_s--;}
    }
}
//counter how many times pump was on after off
static u16 pump_on_cnt=0;
//on
static void pump_on(void)
{
    relay_pin_high();
    pump_on_cnt++;
    //start counter
    pump_on_time_s=0;
    bit_set1(pump_state,pump_state_on);
    bit_set0(pump_state,pump_state_empty|
                        pump_state_on_too_long|
                        pump_state_high_press);
}
//off
static void pump_off(void)
{
    relay_pin_low();
    bit_set0(pump_state,pump_state_on);
}

enum pump_info_e
{
    pump_first,
    pump_press_er,
    pump_empty,
    pump_on_low,
    pump_press_high,
    pump_norm,
    pump_night,
    pump_last
};

static const char * pump_info_str[pump_last]={
    "init",
    "press sens error",
    "empty",
    "start from low",
    "press high",
    "start from norm",
    "night"
};
//pump info to display
pump_info_e pump_info;

//
static void initPump(void)
{
    pump_state=pump_state_def;
    pump_off();
}

//-----------------
// pressure sensor pins

#define press_port C


// pin-100R-ANALOG_SENSOR_OUT
#define press1_pin      _0
#define press1_pin_init() pci(press1_pin)
#define press1_admux    admux_ref_vcc|0


// pin-100R-ANALOG_SENSOR_OUT
#define press2_pin      _1
#define press2_pin_init() pci(press2_pin)
#define press2_admux    admux_ref_vcc|1


// pin-100k-gnd, pin-33k-vcc_sensor
#define press_vcc_pin   _2
#define press_vcc_pin_init() pci(press_vcc_pin)
#define press_vcc_admux admux_ref_vcc|2

// pin - k-gnd, pin- k photoresistor
#define light_pin       _3
#define light_pin_init() pci(light_pin)
#define light_admux admux_ref_vcc|3



inline static void initAdcPins(void)
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
// rs485 9600 1 n 
//halfbit time pause
#define rs485_halfbit_us    (u8)51

//commands
enum pump_com_e
{
    //get pump state byte
    //aa 41 be 00 00 00
    pump_com_get_state,
    //get pressure data
    pump_com_get_press,
    //try to on
    pump_com_on,
    //off for time X 
    //then on
    pump_com_off_time,
    //
    pump_com_ret_state,
    //
    pump_com_ret_press,
};


//address of current pump
static u8 rs485_pump_addr;
//broadcast addr
#define rs485_broadcast_addr    0

// msg

// 0xaa     - start byte
// FFFFTTTT - byte has info about msg from (F 4 bits) to (T 4 bits)
// fromto_func(FFFFTTTT) - byte something like "ftom to" byte's crc
// IIIILLLL - byte I - info bits, LLLL - length of data bytes (0=1 F=16)

// DDDDDDDD - data bytes
// ...

// CCCCCCCC - crc byte of IL and data bytes (length LLLL)

//max data bytes count
#define rs485_msg_len_max      (0xF+1)

#define rs485_msg_start_byte    0xaa

inline static u8 fromto_func(u8 lb)
{
    return ~lb;
}
// buf for one msg to send (without start byte)
#define rs485_sbuf_max          (rs485_msg_len_max)
static u8 rs485_sbuf[rs485_sbuf_max];


//-------------------
// reading from rs485



//----------------
// rs485 msg data
struct rs485_rmsg_s
{
    u8 fromto;
    u8 len;
    u8 data[rs485_msg_len_max];
    u8 ready;//if msg ready to process
    u8 next;//index of next msg or free space for msg
    
};
//readed msg
#define rs485_rmsg_max  5
static rs485_rmsg_s rs485_rmsg[rs485_rmsg_max];
//no index
#define rs485_rmsg_no   0xff

//index of first readed msg
static u8 rs485_rmsg_first;
//index of last readed msg
static u8 rs485_rmsg_last;
//index of first free line
static u8 rs485_rmsg_first_free;
//----------------
//init
static void rs485_rmsg_init(void)
{
    rs485_rmsg_first_free=0;
    u8 li=0;
    while (li!=(rs485_rmsg_max-1))
    {
        rs485_rmsg[li].next=li+1;
        li++;
    }
    rs485_rmsg[li].next=rs485_rmsg_no;
    
    rs485_rmsg_first=rs485_rmsg_no;
    rs485_rmsg_last=rs485_rmsg_no;

}

//get space for new msg
//!! be careful use cli sei
static u8 rs485_rmsg_get(void)
{
    if (rs485_rmsg_first_free==
            rs485_rmsg_no)
    {
        return rs485_rmsg_no;
    }
    
    //
    //cli();
    //set next free
    u8 lff=rs485_rmsg_first_free;
    rs485_rmsg_first_free=rs485_rmsg[lff].next;
    
    //add msg to msg list
    rs485_rmsg[lff].ready=0;
    rs485_rmsg[lff].next=rs485_rmsg_no;
    if (rs485_rmsg_first==
            rs485_rmsg_no)
    {
        //first
        rs485_rmsg_first=lff;
    }
    else
    {
        //last
        rs485_rmsg[rs485_rmsg_last].next=lff;
    }
    //set last
    rs485_rmsg_last=lff;
    //sei();
    
    return lff;
}
//free line from first processed msg
//!! be careful use cli sei
static void rs485_rmsg_put(void)
{
    if (rs485_rmsg_first==rs485_rmsg_no)
    {
        return;
    }
    
    //cli();
    
    u8 lnext=rs485_rmsg[rs485_rmsg_first].next;
    //return to list
    rs485_rmsg[rs485_rmsg_first].next=rs485_rmsg_first_free;
    rs485_rmsg_first_free=rs485_rmsg_first;
    //remove from msg list
    rs485_rmsg_first=lnext;
    if (lnext==rs485_rmsg_no)
    {
        rs485_rmsg_last=rs485_rmsg_no;
    }
    
    //sei();
}
//reading states
enum rs485_rstate_e
{
    rs485_rstate_start,//wait start byte
    rs485_rstate_fromto1,//wait from to first byte
    rs485_rstate_fromto2,//wait from to first byte
    rs485_rstate_len,//wait len
    rs485_rstate_crc,//wait crc
};
//state of rs485 reading
static rs485_rstate_e rs485_rstate;




volatile static u8 rs485_int_crc;
volatile static u8 rs485_int_readed;
volatile static u8 rs485_int_fromto;
volatile static u8 rs485_int_rmsg_i;
//
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
    //ldh;ldl;
    //we have got one more byte from rs485
    //fsm
    switch (rs485_rstate)
    {
        case rs485_rstate_start:
            //ldh;
            if (lb==rs485_msg_start_byte)
            {
                //ldl;
                //get start byte
                rs485_rstate=rs485_rstate_fromto1;
            }
            
            break;

            
            
            
        case rs485_rstate_fromto1:
            rs485_rstate=rs485_rstate_fromto2;
            rs485_int_fromto=lb;
            break;
            
        case rs485_rstate_fromto2:
            //test for error
            if ((rs485_int_fromto&0xf)!=
                ((rs485_int_fromto>>4)&0xf))
            {
                //good
                if (lb==
                        fromto_func(rs485_int_fromto))
                {
                    //ok we have msg
                    //check if msg for us
                    u8 lto=rs485_int_fromto&0xf;
                    if ((lto==rs485_pump_addr)||
                            (lto==rs485_broadcast_addr))
                    {
                        //msg for us
                        rs485_rstate=rs485_rstate_len;
                    }
                    else
                    {
                        //msg not for us
                        rs485_rstate=rs485_rstate_start;
                    }
                    break;
                }
            }
            //error
            if (rs485_int_fromto!=rs485_msg_start_byte)
            {
                if (lb!=rs485_msg_start_byte)
                {
                    rs485_rstate=rs485_rstate_start;
                }
                else
                {
                    rs485_rstate=rs485_rstate_fromto1;
                }    
            }
            else
            {
                rs485_rstate=rs485_rstate_fromto2;
                rs485_int_fromto=lb;
            }
            break;

        case rs485_rstate_len:
            //test len
            if (lb<rs485_msg_len_max)
            {
                //ok
                //get space for new msg
                rs485_int_rmsg_i=rs485_rmsg_get();
                if (rs485_int_rmsg_i!=rs485_rmsg_no)
                {
                    
                    //ok
                    rs485_rmsg[rs485_int_rmsg_i].fromto=rs485_int_fromto;
                    rs485_rmsg[rs485_int_rmsg_i].len=lb+1;
                    rs485_int_crc=lb;
                    rs485_rstate=rs485_rstate_crc;
                    rs485_int_readed=0;
                }
                break;
            }
            //error
            rs485_rstate=rs485_rstate_start;
            break;
            
        case rs485_rstate_crc:
            //ldh;ldl;
            //test data bytes readed
            if (rs485_int_readed==
                    rs485_rmsg[rs485_int_rmsg_i].len)
            {
                //we have got crc
                if (rs485_int_crc==lb)
                {
                    //ok we have read whole msg
                    rs485_rmsg[rs485_int_rmsg_i].ready=1;
                    
                }
                else
                {
                    //crc error -> free msg space
                    rs485_rmsg_put();
                    //ldh;ldl;
                }
                rs485_rstate=rs485_rstate_start;
                break;
            }
            else
            {
                //save data byte
                rs485_rmsg[rs485_int_rmsg_i].
                    data[rs485_int_readed++]=lb;
                rs485_int_crc=rs485_int_crc^lb;
            }
            break;
            
    }
          
  
  
    //wait for next byte
    int1clr();
    sei();
}

//---------------
//test if we have new msg
//ret index of new msg or msg_no
static u8 rs485_has_new_msg(void)
{
    u8 li;
    cli();
    li=rs485_rmsg_first;
    if (li!=rs485_rmsg_no)
    {
        if (rs485_rmsg[li].ready==0)
        {
            li=rs485_rmsg_no;
        }
    }    
    sei();
    return li;
}
//---------------
// delete msg after processing
static void rs485_del_new_msg(void)
{
    cli();rs485_rmsg_put();sei();
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


//pause
#define rs485_send_pause_min_us (2*rs485_halfbit_us)
static void rs485_rand_pause_us(void)
{
    //random pause
    if (mc_rand<(rs485_send_pause_min_us))
    {
        dus(rs485_send_pause_min_us);
    }
    else
    {
        if (mc_rand==0)  {mc_rand=mc_randnz;}
        dus(mc_rand);
    }
}
//--------------------
//send buf by rs485
static void rs485_send_msg( u8 lfrom,
                            u8 lto,
                            u8 len,
                            u8 * lp)
{
    if ((len==0)||
            (len>rs485_msg_len_max))
    {
        return;
    }
    
    //send start byte
    rs485_send(rs485_msg_start_byte);
    rs485_rand_pause_us();
    
    //from to
    u8 lfromto=((lfrom&0xf)<<4)+(lto&0xf);
    rs485_send(lfromto);
    rs485_rand_pause_us();
    rs485_send(fromto_func(lfromto));
    rs485_rand_pause_us();
    
    
    //length
    rs485_send(len-1);
    rs485_rand_pause_us();
    
    //send data
    u8 lcrc=len-1;
    while (len-->0)
    {
        rs485_send(*lp);
        lcrc=lcrc^(*lp++);
        rs485_rand_pause_us();
    }
    
    //send crc
    rs485_send(lcrc);
    rs485_rand_pause_us();
}

//----------------
static void initRs485(void)
{
    //pump address from eeprom
    rs485_pump_addr=eerb(ee_rs485_pump_addr);
    //init msg space
    rs485_rmsg_init();
    //setup read int and fsm
    rs485_rstate=rs485_rstate_start;
    int1_on_fall();
    int1en();
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

void sptime_s_to_hm(u16 lsec)
{
    if (lsec>=((u16)17*60*60))
    {
        s(">17h");
    }
    else
    {
        u16 lt=lsec;
        if (lsec>=3600)
        {
            lt=lsec/3600;
            sp8(lt,2);sp(':');
            lt=lsec%3600;
        }
        sp8(lt/60,2);sp(':');
        sp8(lt%60,2);
    }
}








//=========================
// adc
//=========================

//data from adc
static struct adc_s
{
    //water pressure for sensor1
    //for ex. 01 means 0.1 bar 
    volatile u16 press1;
    volatile u8 press1_bar;
    //water pressure for sensor2
    volatile u16 press2;
    volatile u8 press2_bar;
    //voltage of sensors power source
    //for ex. 502 means 5.02 V 
    volatile u16 vcc_press;
    //voltage of mc
    //for ex. 502 means 5.02 V 
    volatile u16 vcc_mc;
    //light (day/night)
    volatile u16 light;//night 0< light <  day
} adc;

#define light_day_adc       600
#define light_is_night()    (adc.light<light_day_adc)

//log adc
void spadc(void)
{
    s(" pump n:");sp8(rs485_pump_addr);
    s(" strts:");sp16(pump_on_cnt);spn;
    s(pump_info_str[pump_info]);spn;
    spdec(adc.vcc_mc,2,0);sp('/');
    spdec(adc.vcc_press,2,0);s(" v");spn;
    spdec(adc.press1_bar,1,0);sps;
    sp16(adc.press1);sp('/');
    spdec(adc.press2_bar,1,0);sps;
    sp16(adc.press2);spn;
    s(" l:");sp16(adc.light);sps;
    if (light_is_night())
    {
        s("night");
        
    }
    else
    {
        s("day");
    }
    spn;
}



//press in bar
static u16 press_adc_to_bar(u16 lpress,
                            u16 lpress_offset)
{
    spn;
    if (lpress<=lpress_offset)
    {
        return 0;
    }
    else
    {
        //1.calc ADC=ADC_measured-ADC_zero_offset
        //2.calc adc measured voltage
        // V=ADC*(VCC_press*100)/ADC_max_points
        //3.calc press in 0.1 bar
        // volt per 0.1bar = 4V/12bar=0.333 V
        // P*10=V/v=V/33
        return (((long)(lpress-lpress_offset)//adc
                *adc.vcc_press*10)/adc_max_points)//volt
                /33;// div Y volt per 0.1bar
    }
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
    ladc.vcc_mc=vcc_read();
    ladc.vcc_press=aread(press_vcc_admux);
    ladc.press1=aread(press1_admux);
    ladc.press2=aread(press2_admux);
    ladc.light=aread(light_admux);
    
    adc_off();power_adc_disable();
    
    //-----------
    //calculations
    //sp16(ladc.vcc_press,4);sps;
    //press_vcc to volts
    //press_vcc=((long)press_vcc*(vcc*10))/adc_max_points;
    //restore after divider r1=33k r2=100k
    //ladc.vcc_press=((long)ladc.vcc_press*133)/100;
    
    ladc.vcc_press=(((long)ladc.vcc_press*10*133/
                    adc_max_points)*
                    ladc.vcc_mc)/1000;///100
    
    //-----------
    //calc press1 in volts
    //sp16(ladc.press1,4);spn;
    ladc.press1_bar=press_adc_to_bar(ladc.press1,
                            pump_single.press1_offset_adc);
    ladc.press2_bar=press_adc_to_bar(ladc.press2,
                            pump_single.press2_offset_adc);
    

    cli();
    adc.vcc_mc=ladc.vcc_mc;
    adc.vcc_press=ladc.vcc_press;
    adc.press1=ladc.press1;
    adc.press1_bar=ladc.press1_bar;
    adc.press2=ladc.press2;
    adc.press2_bar=ladc.press2_bar;
    adc.light=ladc.light;
    
    sei();
   
    return 0;
 }







//------------------------
// to exit from delay by rs485 command
static u8 delay_exit;
// to change delay_sec on the fly
static u16 delay_sec;
//send state
static void rs485_send_state(u8 li,
                             u8 lto)
{
    //send pump_state
    rs485_rmsg[li].data[0]=pump_com_ret_state;
    rs485_rmsg[li].data[1]=pump_state;
    rs485_send_msg(rs485_pump_addr,
            lto,
            2,
            rs485_rmsg[li].data);
    //
    s("reply state:");
    sph(pump_state);
    spn;   
    //to wait serial output
    dus(1000);
}

//process rs485 msgs
//always from delay
static void rs485_rmsg_process(void)
{
    u8 li=rs485_has_new_msg();
    if (li!=rs485_rmsg_no)
    {
            
        
        //show new msg
        u8 lto=(rs485_rmsg[li].fromto)>>4;
//            rs485_send_msg((rs485_rmsg[li].fromto)>>4,
//                            (rs485_rmsg[li].fromto)&0xf,
//                            rs485_rmsg[li].len,
//                            rs485_rmsg[li].data);
            
        //new msg to serial
        s("rs485 from ");sp8(lto);sp(':');
        u8 ln=rs485_rmsg[li].len;
        u8 * lp=rs485_rmsg[li].data;
        while (ln--)
        {
          sph(*lp++);sps;   
        }
        spn;



        //do action
        lp=rs485_rmsg[li].data;
        
        if (lto!=rs485_broadcast_addr) switch (*lp++)
        {
            case pump_com_get_state:
                rs485_send_state(li,lto);
                break;
                
            case pump_com_get_press:
                //
                adc_read();
                //send pressure
                rs485_rmsg[li].data[1]=(adc.press1_bar)&0xff;
                rs485_rmsg[li].data[2]=(adc.press2_bar)&0xff;
                
                rs485_send_msg(rs485_pump_addr,
                        lto,
                        3,
                        rs485_rmsg[li].data);
                //send pressure
                s("reply press:");
                spadc();
                
                //to wait serial output
                dus(1000);
                break;
                
            case pump_com_on:
                //test pump_state
                
                if (//bit_is0(pump_state,pump_state_empty)&&
                    bit_is0(pump_state,pump_state_er_press))
                {
                    delay_exit=1;
                    s("*rs485 pump_on");spn;
                    pump_on();
                }
                rs485_send_state(li,lto);
                break;
                
                
            case pump_com_off_time:
                //
                pump_off();
                s("*rs485 pump_off");spn;
                delay_sec=(((u16)rs485_rmsg[li].data[2])<<8)+
                        rs485_rmsg[li].data[1];
                s("delay ");sp16(delay_sec);spn;
                //
                rs485_send_state(li,lto);
                break;


            case pump_com_ret_state:
                s("recieve state:");
                sph(rs485_rmsg[li].data[1]);
                spn;   
                //to wait serial output
                dus(1000);
                break;
            
            case pump_com_ret_press:
                s("recieve press:");
                spdec(adc.press1_bar,1,0);sp('/');
                spdec(adc.press2_bar,1,0);spn;
                //to wait serial output
                dus(1000);
                break;

        }
        //free msg space
        rs485_del_new_msg();
    }
}


//-----------------------------
enum led_t {
    led_no,
    led_low,
    led_high,
    led_empty,
    led_off_on,
    led_error,
};

void sp_single_log(void)
{
    spadc();
    if (pump_is_on())
    {
        s(" on ");sptime_s_to_hm(pump_on_time_s);
        
    }
    else
    {
        s(" delay ");sptime_s_to_hm(delay_sec);
    }
    spn;spn;
}

static void mc_pwrdown(period_t lperiod)
{
    pwrdown(lperiod,b1);
    wdt_on();
    rs485_rmsg_process();
    
    //does not work in pwrdown
//    if (serial_buf_i>0)
//    {
//        serial_buf_clear();
//        //show info
//        sp_single_log();
//        dus(1000);
//    }
    //show pump info
    if (mc_btn_pressed())
    {
        adc_read();
        sp_single_log();
        
        //        dms(400);
//        mc_btn_pressed();
//        
//        //
//        pump_on();
//        led_main_low();
//        
//        //loop
//        
//        u8 lsec=11;
//        while (lsec--)
//        {
//            s("* btn pon for ");
//            sp8(lsec);
////            sps;
////            sp8(serial_has_input);sps;
////            sp8(serial_buf_i);sps;
//            spn;
//            //measure pressure
//            adc_read();
//            //log
//            sp_single_log();
//            //
//            dms(1000);
//            wdt_reset();
//            pump_on_time_inc();
//            //we press btn again
//            if (mc_btn_pressed())
//            {
//                lsec+=10;
//            }
//        }
//        pump_off();
//        s("* btn poff");spn;
        dus(1000);
    }
}    


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
    delay_sec=lsec;
    delay_exit=0;
    //wait log output
    dus(400);
    
    if (lled==led_error)
    {
        led_main_low();
    }
    
    
    u8 lodd=0;
    while (delay_sec--)
    {
        //exit by rs485 command
        if (delay_exit!=0)
        {
            delay_exit=0;
            break;
        }
        
        //for skip one turn to get better blink reading
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
        
        pump_on_time_inc();
        
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
    //-------------
    // init
    atmega328p_init();
    initLedPins();
    initRelayPins();
    initAdcPins();
    initRs485Pins();
    initBtnPins();
    
    //----------
    initPump();

    //-------------
    initRs485();
    
    //-------------
    initBtn();
    
    //-------------
    serial_init();
    
    //----------
    sei();
    
    
    // main loop
    volatile u8 li=0;
    sph(rs485_pump_addr);spn;
    sph(rs485_msg_start_byte);sps;
    sph(fromto_func(rs485_msg_start_byte));spn;
    dus(1000);

    wdt_on();
    while (1) {
        led_main_high();
        //led_er_low();
        relay_pin_high();
        sph(1);spn;
        //dus(200);
        //mc_pwrdown(_1s);
        dms(1000);
        
        
        
        led_main_low();
        //led_er_high();
        relay_pin_low();
        sph(0);spn;
        //dus(200);
        //wdt_reset();
        dms(1000);
        //mc_pwrdown(_1s);
        //dms(250);
        
        
        u8 li=serial_has_input;
        if (li!=0)
        {
            s("send by rs485");spn;
            rs485_send_msg(
                    rs485_pump_addr,
                    rs485_broadcast_addr,
                    1,
                    &serial_buf[0]);
            serial_buf_clear();
        }
        
        //
        li=rs485_has_new_msg();
        if (li!=rs485_rmsg_no)
        {
            s("read by rs485");spn;
            //show new msg
//            rs485_send_msg((rs485_rmsg[li].fromto)>>4,
//                            (rs485_rmsg[li].fromto)&0xf,
//                            rs485_rmsg[li].len,
//                            rs485_rmsg[li].data);
            
            //new msg to serial
            u8 ln=rs485_rmsg[li].len;
            u8 * lp=rs485_rmsg[li].data;
            while (ln--)
            {
              sph(*lp++);sps;   
            }
            spn;
            
            
            
            //do action
            lp=rs485_rmsg[li].data;
            u8 lto=(rs485_rmsg[li].fromto)>>4;
            if (lto!=rs485_broadcast_addr) switch (*lp++)
            {
                case pump_com_get_state:
                    //send pump_state
                    rs485_rmsg[li].data[1]=pump_state;
                    rs485_send_msg(rs485_pump_addr,
                            lto,
                            2,
                            rs485_rmsg[li].data);
                    break;
            }
                    
            
            //free msg space
            rs485_del_new_msg();
        }
            
        dus(1000);
        wdt_reset();
    }
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




//--------------------------------
// var with single pump without BV
//--------------------------------

//check press sensor
static u8 press_check(  u16 lpress_bar,
                        u16 lpress_bar_next)
{
    if ((lpress_bar>=pump_single.press_max_error)||
         (lpress_bar_next>=pump_single.press_max_error))
    {
        return 1;
    }
    else    
    {
        if (lpress_bar>lpress_bar_next)
        {
            if ((lpress_bar-lpress_bar_next)>
                pump_single.press_dif_error) 
            {
                return 1;
            }
        }
        else
        {
            if ((lpress_bar_next-lpress_bar)>
                pump_single.press_dif_error) 
            {
                return 1;
            } 
        }
    }
    
    return 0;
}


//with WATCHDOG!!!
static u8 var_single_without_back_valve_press1(void)
{
    led_er_high();
    led_main_high();
    wdt_on();
    s("var_single_without_back_valve_press1");spn;
    
    pump_info=pump_first;
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
        u16 lpress1_bar=adc.press1_bar;
        u16 lpress2_bar=adc.press2_bar;
        u8 le=0;
        
        
        //measure pressure
        adc_read();
        
        //log
        sp_single_log();
        
        
        //check press sensor
        le+=press_check(lpress1_bar,adc.press1_bar);
        le+=press_check(lpress2_bar,adc.press2_bar);
        
        

        //if error of pressure sensor
        if (le!=0)
        {
            led_er_high();
            pump_off();
            bit_set1(pump_state,pump_state_er_press);
            //pressure sensor error
            s(" poff (press err)");spn;
            pump_info=pump_press_er;
            
            //delay error
            delay_sec_pwrdown(
                pump_single.delay_error, led_error);
            led_er_low();
            //bit_set0(pump_state,pump_state_er_press);
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
                (pump_on_time_s>pump_single.time_max_pump_on))
        {
            spn;s("! max on time");spn;
            bit_set1(pump_state,pump_state_on_too_long);
        }
        
        
        
        
        
        
        //------------------------
        // main
        if (adc.press1_bar<=pump_single.press_min)
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
                
                
                if (adc.press1_bar<=pump_single.press_min)
                {
                    //press1 still low -> pump off
                    
                    bit_set1(pump_state,pump_state_empty);
                    pump_off();
                    
                    //calculate empty_delay
                    if (pump_on_time_s<=
                            pump_single.empty_pump_on_time_min)
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
                    pump_info=pump_empty;
                    //delay empty_delay
                    delay_sec_pwrdown(empty_delay, led_no);
                    //bit_set0(pump_state,pump_state_empty);
                    
                }
            }
            else
            {
                //pump is off and press1 is low -> on pump
                s(" low (off)");spn;
                
                //if was empty
                u8 lon=1;
                if (bit_is1(pump_state,pump_state_empty))
                {
                    if (light_is_night())
                    {
                        lon=0;
                    }
                }
                else
                {
                    lon=1;
                }
                //
                if (lon==0)
                {
                    s("night");
                    pump_info=pump_night;
                    //delay empty_delay
                    delay_sec_pwrdown(pump_single.delay_night, led_no);
                    
                }
                else
                {
                    pump_on();
                    s("* pon");
                    pump_info=pump_on_low;
                }
                spn;
            }
        }
        else
        {
            if (pump_is_on())
            {
                
                
                //pump is on
                if (adc.press1_bar>=pump_single.press_max)
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
                
                
                    if (adc.press1_bar>=pump_single.press_max)
                    {
                        //still high pressure
                        bit_set1(pump_state,pump_state_high_press);
                        pump_off();
                        
                        s("* poff (high)");spn;
                        pump_info=pump_press_high;
                        //delay high_pressure
                        delay_sec_pwrdown(
                            pump_single.delay_after_high, led_off_on);
                    }
                }

            }
            else
            {
                //pump is off but pressure is low max pressure
                s(" normal (off)");spn;
                pump_on();
                s("* pon");spn;
                pump_info=pump_norm;
            }
        }
        
        //delay_measure
        if (pump_is_on())
        {
            //no sleep
            dms(pump_single.delay_measure*1000);
        }
        else
        {
            delay_sec_pwrdown(pump_single.delay_measure,led_no);
        }

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
    initAdcPins();
    initRs485Pins();
    initBtnPins();
    
    //----------
    initPump();

    //-------------
    initRs485();
    
    //-------------
    initBtn();
    
    //---------------
    serial_init();

    
    //--------------
    sei();
    
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