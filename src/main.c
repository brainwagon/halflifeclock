#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>

#include "i2cmaster.h"

////////////////////////////////////////////////////////////////////////

const uint8_t message[16] PROGMEM = {
	0b0000000,		// 
	0b0000000,		// 
	0b1110100,		// h
	0b1011111,		// a
	0b0111000,		// L
	0b1110001,		// F
	0b0000000,		// 
	0b0111000,		// L
	0b0010000,		// i
	0b1110001,		// F
	0b1111001,		// E
	0b0000000,		// 
	0b1011110,		// d
	0b1011111,		// a
	0b1101110,		// Y
	0b0000000,		// 
} ;

const uint8_t message2[] PROGMEM = {
	0b0000000,		// 
	0b0000000,		// 
	0b0000000,		// 
	0b0000000,		// 
        0b1110100,              // h
	0b1011111,		// a
	0b1101101,		// S
	0b0000000,		// 
	0b1110011,		// P
	0b1011111,		// a
	0b1101101,		// S
        0b1111000,              // t
	0b0000000,		// 
	0b0000000,		// 
	0b0000000,		// 
	0b0000000,		// 
        0b0010000,              // i
	0b1101101,		// S
	0b0000000,		// 
        0b1111000,              // t
	0b1011100,		// o
	0b1011110,		// d
	0b1011111,		// a
	0b1101110,		// Y
	0b0000000,		// 
	0b0000000,		// 
	0b0000000,		// 
	0b0000000,		// 
} ;

////////////////////////////////////////////////////////////////////////

uint8_t
bcd2byte(uint8_t b)
{
    uint8_t l = b & 0xF ;
    uint8_t h = b >> 4 ;
    return (h << 3) + (h << 1) + l ;
}

uint8_t
byte2bcd(uint8_t b)
{
    uint8_t r = 0 ;
    for (;;) {
	if (b < 10)
	    return r + b ;
	b -= 10 ;
	r += 0x10 ;
    }
}

uint16_t
int2bcd(uint16_t x)
{
    uint8_t h = 0 ;
    while (x >= 100) {
	h++ ;
	x -= 100 ;
    }
    return (byte2bcd(h) << 8) | byte2bcd(x) ;
}


////////////////////////////////////////////////////////////////////////
// TM1640 driver code
////////////////////////////////////////////////////////////////////////

uint8_t __attribute__ ((section(".noinit"))) tm1640_buf[16] ;

const uint8_t tm1640_font[] PROGMEM = { 
	0b00111111,	
	0b00000110,
	0b01011011,
	0b01001111,
	0b01100110,
	0b01101101,
	0b01111101,
	0b00000111,
	0b01111111,
	0b01101111,
} ;

#define TM1640_DDR	DDRD
#define TM1640_PORT	PORTD
#define	TM1640_DATA	(3)
#define TM1640_CLOCK	(2)

void
tm1640_send(uint8_t data) 
{
    for (uint8_t i=0; i<8; i++) {

	TM1640_PORT &= ~_BV(TM1640_CLOCK) ;

	if (data & 1) 
	    TM1640_PORT |= _BV(TM1640_DATA) ;
	else
	    TM1640_PORT &= ~_BV(TM1640_DATA) ;
	data >>= 1 ;

	TM1640_PORT |= _BV(TM1640_CLOCK) ;
    }
}

void
tm1640_sendbuffer(uint8_t *t)
{
    TM1640_PORT &= ~_BV(TM1640_DATA) ;
    TM1640_PORT |=  _BV(TM1640_DATA) ;
  
    TM1640_PORT &= ~_BV(TM1640_DATA) ;
    tm1640_send(0x40);
    TM1640_PORT |= _BV(TM1640_DATA) ;

    // Another command, with 
    // auto-incrementing address, followed by 
    // 16 bytes of data.
    TM1640_PORT &= ~_BV(TM1640_DATA) ;
    tm1640_send(0xC0) ;
    for (uint8_t i=0; i<16; i++)
	tm1640_send(*t++) ;
    TM1640_PORT |=  _BV(TM1640_DATA) ;

    // extra pulse to clock...
    TM1640_PORT &= ~_BV(TM1640_DATA) ;
    TM1640_PORT &= ~_BV(TM1640_CLOCK) ;
    TM1640_PORT |=  _BV(TM1640_CLOCK) ;
    TM1640_PORT |=  _BV(TM1640_DATA) ;
}


void
tm1640_init()
{
    TM1640_DDR  |= _BV(TM1640_DATA) | _BV(TM1640_CLOCK) ;

    TM1640_PORT |= _BV(TM1640_DATA) ;
    TM1640_PORT |= _BV(TM1640_CLOCK) ;

    // turn on, and set brightness...
    TM1640_PORT &= ~_BV(TM1640_DATA) ;
    tm1640_send(0x89) ;
    TM1640_PORT |= _BV(TM1640_DATA) ;
}

////////////////////////////////////////////////////////////////////////

#include "targetdate.h"

#define CLOCK_ADDRESS (0x68*2)

const uint8_t monthtab[] PROGMEM = {
	31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31
} ;

int16_t
days_since_2000(uint8_t y, uint8_t m, uint8_t d)
{
    int16_t t = 0 ;
 
    for (uint8_t i=0; i<y; i++) {
	t += 365 ;
	if ((i & 3) == 0)
	    t++ ;
    }

    for (uint8_t i=1; i<m; i++) {
	t += pgm_read_byte(monthtab+i-1) ;
	if ((y & 3) == 0 && (i == 2))
	    t++;
    }
    t += d - 1 ;

    return t ;
}

////////////////////////////////////////////////////////////////////////

#ifdef USE_SLEEP

#define CLOCK_PRESCALER		1024
#define OVERFLOW_COUNT		((2*F_CPU/CLOCK_PRESCALER)-1)

ISR(TIMER1_COMPA_vect)
{
    /* interrupt just serves to wake up the processor */
}

void
timer_init() 
{
    TCCR1A = 0 ;
    TCCR1B = _BV(WGM12) | _BV(CS12) | _BV(CS10) ;
    OCR1A = OVERFLOW_COUNT ;
    TIMSK1 |= _BV(OCIE1A) ;

    set_sleep_mode(SLEEP_MODE_IDLE) ;

    sei() ;
}

void __attribute__ ((noinline))
dosleep()
{
    sleep_enable() ; sleep_cpu() ; sleep_disable() ;
}

#else

void
timer_init() 
{
    /* Empty, we are going to delay */
}

void __attribute__ ((noinline))
dosleep()
{
    __builtin_avr_delay_cycles(2*F_CPU) ;
}

#endif


////////////////////////////////////////////////////////////////////////

void
display_flash_message(const uint8_t *t)
{
    for (uint8_t i=0; i<16; i++)
	tm1640_buf[i] = pgm_read_byte(t+i) ;
    tm1640_sendbuffer(tm1640_buf) ;
    dosleep() ;
}

void
display_message()
{
    tm1640_sendbuffer(tm1640_buf) ;
    dosleep() ;
}

////////////////////////////////////////////////////////////////////////
//
// We don't have a lot of space left, so we just turn on the LED on the 
// right day.  I think I'll hook a couple of my 10mm blinking LEDS to
// the output.  Or maybe something more awesome.
//
////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////
// default LED is pin 5 on PORTB
////////////////////////////////////////////////////////////////////////

void
alarm_init()
{
    DDRB |= _BV(5) ;
    PORTB &= ~_BV(5) ;
}

void
alarm(uint8_t on)
{
    if (on)
        PORTB |=  _BV(5) ;
    else
        PORTB &= ~_BV(5) ;
}

////////////////////////////////////////////////////////////////////////


int
main(void)
{
    int16_t days ;
    uint8_t y, m, d ;

    timer_init() ;
    i2c_init() ;
    tm1640_init() ;
    alarm_init() ;

    for (;;) {
       
        // Fetch time from DS3231
        i2c_start_wait(CLOCK_ADDRESS + I2C_WRITE) ;
        i2c_write(4) ;
        i2c_rep_start(CLOCK_ADDRESS + I2C_READ) ;
	d = bcd2byte(i2c_readAck()) ;
	m = bcd2byte(i2c_readAck()) ;
	y = bcd2byte(i2c_readNak()) ;
        i2c_stop() ;

        days = TARGET_DAYS_SINCE_JAN1_2000 - days_since_2000(y, m, d) ;

        alarm(days == 0) ;

        if (days == 0) {

            /* haLF LiFE dAY */
            display_flash_message(message) ;
            /* iS todAY */
            display_flash_message(message2+12) ;

        } else if (days < 0) {

            /* haLF LiFE dAY */
            display_flash_message(message) ;
            /* iS PaSt */
            display_flash_message(message2) ;

        } else {
            days = int2bcd(days) ;
            register uint8_t d0, d1, d2, d3 ;

            d0 = (days >> 12) & 0xF ;
            d1 = (days >>  8) & 0xF ;
            d2 = (days >>  4) & 0xF ;
            d3 = (days >>  0) & 0xF ;

            // Logic to strip leading zeros...
            if (days >= (1<<12))
                d0 = pgm_read_byte(tm1640_font+d0) ;
            if (days >= (1<<8))
                d1 = pgm_read_byte(tm1640_font+d1) ;
            if (days >= (1<<4))
                d2 = pgm_read_byte(tm1640_font+d2) ;

            d3 = pgm_read_byte(tm1640_font+d3) ;
                
            tm1640_buf[0x0] = 0b0000000;		// 
            tm1640_buf[0x1] = 0b0000000;		// 
            tm1640_buf[0x2] = d0 ;		        // <digit0>
            tm1640_buf[0x3] = d1 ;		        // <digit1>
            tm1640_buf[0x4] = d2 ;		        // <digit2>
            tm1640_buf[0x5] = d3 ;		        // <digit3>
            tm1640_buf[0x6] = 0b0000000;		// 
            tm1640_buf[0x7] = 0b1011110;		// d
            tm1640_buf[0x8] = 0b1011111;		// a
            tm1640_buf[0x9] = 0b1101110;		// Y
            tm1640_buf[0xa] = 0b1101101;		// S 
            tm1640_buf[0xb] = 0b0000000;		// 
            tm1640_buf[0xc] = 0b1111000;                // t
            tm1640_buf[0xd] = 0b1011100;		// o
            tm1640_buf[0xe] = 0b0000000;		// 
            tm1640_buf[0xf] = 0b0000000;		// 

            /* 0000 daYS to */
            display_message() ;
            /* haLF LiFE dAY */
            display_flash_message(message) ;
        }
    }
}
