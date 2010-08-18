#include "main.h"
#include "include/global.h"

#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <util/delay.h>

#include "include/timerx8.h"  		// Timer utility library
#include "include/iv4.h"      		// IV4 bitfield defines and functions
#include "include/shift.h"    		// Generic shift register library
#include "include/words.c"    		// Letter-pair data
#include "include/usb/usbconfig.h"	// USB configuration header
#include "include/usb/usbdrv.h"		// USB driver header

const int32_t EEMEM lastseed = 0xDEADBEEF;
uint8_t k, a, b, c, d;
volatile uint8_t up;

uint8_t charMap[4][3];
int32_t stx; 						// PRNG state variable.  We never return this directly.

PROGMEM char usbHidReportDescriptor[22] = {    /* USB report descriptor */
    0x06, 0x00, 0xff,              // USAGE_PAGE (Generic Desktop)
    0x09, 0x01,                    // USAGE (Vendor Usage 1)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0x09, 0x00,                    //   USAGE (Undefined)
    0xb2, 0x02, 0x01,              //   FEATURE (Data,Var,Abs,Buf)
    0xc0                           // END_COLLECTION
};

// general device initialization
void init(void) {
	initHW();

	// NVRAM: get last used random seed, modify, and write back to NVRAM
	eeprom_busy_wait();
	eeprom_read_block((void*) &stx, (const void*) &lastseed, 4);
	stx += 0xF0A110AF;
	eeprom_busy_wait();
	eeprom_write_block((const void*) &stx, (void*) &lastseed, 4);
}

static inline void initHW(void) {
	// INIT: All pins output, all pins low
	PORTB = 0x00;
	PORTC = 0x00;
	DDRB = 0xFF;
	DDRC = 0xFF;

	// INIT: Shift Register
	initSHR();

	// INIT: Button inputs
	DDRB &= ~(_BV(3) | _BV(4) | _BV(5));

	// INIT: USB IOs (configure as inputs, disable pullups!)
	DDRD &= ~(_BV(1) | _BV(2));
	PORTD &= ~(_BV(1) | _BV(2));

	// INIT: ADC input
	DDRC &= ~_BV(1);

	// INIT: Setup timers
	timerInit();
	// INIT: set up timer1 for PWM
	timer1Init();
	timer1PWMInitICR(500);
	timer1SetPrescaler(TIMER_CLK_DIV1);
	timer1PWMASet(40);
	timer1PWMAOn();
	// INIT: set up timer2 for grid muxing
	timer2Init();
	timer2SetPrescaler(TIMERRTC_CLK_DIV256);
	timerAttach(TIMER2OVERFLOW_INT, timer2Overflow);

	// INIT: Set ADC voltage reference to external AVCC
	ADMUX |= _BV(REFS0);
	// INIT: Set ADC multiplexer to ADC1, our feedback channel
	ADMUX |= _BV(MUX0);
	// INIT: Set ADC to left-adjust results
	ADMUX |= _BV(ADLAR);
	// INIT: Enable ADC circuitry
	ADCSRA |= _BV(ADEN);
	// INIT: Enable ADC conversion-completion interrupt
	ADCSRA |= _BV(ADIE);
	// INIT: Set ADC prescaler to 128
	ADCSRA |= (_BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0));
	// INIT: Disable digital input buffer on ADC1
	DIDR0 |= _BV(ADC1D);
}

ISR(ADC_vect) {
	// ADC result of 0x1e1 corresponds to an HV of 50V.  Ignore the two LSBs to create a bit of deadband
	//  and improve noise-immunity (there's gonna be noise)
	if (ADCH > 0x20) {
		// voltage is too high!  back off.
		ICR1 = ICR1--;
	}
	if (ADCH < 0x1c) {
		// voltage is too low!  more power argh argh argh.
		ICR1 = ICR1++;
	}
}

void timer0Overflow(void) {
	// Timer0 is 8-bits, suitable for either very short or coarsely-defined delays.  It supports PWM.
}

void timer1Overflow(void) {
	// Timer1 is 16-bits, suitable for either long or fine-grained delays.  It supports PWM.
	// OCR1A is attached to the input of our boost convertor.
}

void timer2Overflow(void) {
	sei();
	up = 1;

/*	if (PINB & 0x08) {
		if (b1 < OFF)
			b1++;
	} else {
		if (b1 > ON)
			b1--;
	}
	if (PINB & 0x10) {
		if (b2 < OFF)
			b2++;
	} else {
		if (b2 > ON)
			b2--;
	}
	if (PINB & 0x20) {
		if (b3 < OFF)
			b3++;
	} else {
		if (b3 > ON)
			b3--;
	}*/
}

uint8_t prng_8(int32_t* x) {
	/* JUST WHAT'S GOING ON HERE?
	 *   You'll notice that we take a 32-bit int as input, but return a measly unsigned char.  What gives?  Can't we just return 32 bits?
	 *   Or operate on the char instead?  Well, no.  This is a compromise - we can store much more entropy in a 32-bit state variable than
	 *   in a char.  So we keep the 32-bit state variable around in memory to store entropy, and we only return eight of its bits.  Returning
	 *   a 32-bit char would require four (4!) registers to be concatenated on the stack, so really passing by reference is the only option if
	 *   we give a crap about doing this business in any timely manner.  It's still probably pretty slow, what with all that math.
	 *
	 *   shifts4lyfe
	 */
	int32_t temp = *x;

	temp = (temp >> 16) + ((temp << 15) % RAND_MAX) - (temp >> 21) - ((temp << 10) % RAND_MAX);
	if (temp < 0) {
		temp += RAND_MAX;
	}

	*x = temp;

	return (uint8_t) (temp >> 10);
}

void gen_word(void) {
	// TODO: rewrite word-generation algorithm
}

void update_time(void) {
	// TODO: rewrite timekeeping algorithm
}

usbMsgLen_t usbFunctionSetup(uchar data[8]) {
usbRequest_t    *rq = (void *)data;

    if((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_VENDOR){
        if(rq->bRequest == CUSTOM_RQ_SET_STATUS){
            if(rq->wValue.bytes[0] & 1){    /* set LED */
                d = '1';
            }else{                          /* clear LED */
                d = '0';
            }
        }else if(rq->bRequest == CUSTOM_RQ_GET_STATUS){
            static uchar dataBuffer[1];     /* buffer must stay valid when usbFunctionSetup returns */
            dataBuffer[0] = 0;
            usbMsgPtr = dataBuffer;         /* tell the driver which data to return */
            return 1;                       /* tell the driver to send 1 byte */
        }
    } else {
        /* class requests USBRQ_HID_GET_REPORT and USBRQ_HID_SET_REPORT are
         * not implemented since we never call them. The operating system
         * won't call them either because our descriptor defines no meaning.
         */
    }
    return 0;   /* default for not implemented requests: return no data back to host */
}

void do_display(void) {
	k++;
	if (k == 4) {
		k = 0;
	}

	SHRBlank();
	selectGrid(charMap[k], k);
	SHRSendBuffer(charMap[k], k);
	SHRLatch();
	SHRUnblank();
}

/* main function */
int16_t main(void) {
	init();

	uchar   i;

	a = '0';
	b = '0';
	c = '0';
	d = '0';

	cli();
    usbDeviceDisconnect();  /* enforce re-enumeration, do this while interrupts are disabled! */
    i = 0;
    while(--i) {             /* fake USB disconnect for > 250 ms */
        _delay_ms(1);
    }
    usbDeviceConnect();
    sei();

    usbInit();

    for (;;) {              /* main event loop */
        usbPoll();

        if(up == 1) {
        	do_display();
        	up = 0;

        	bufferChar(charMap[0], a);
        	bufferChar(charMap[1], b);
        	bufferChar(charMap[2], c);
        	bufferChar(charMap[3], d);
        }
    }

    return 0;
}
