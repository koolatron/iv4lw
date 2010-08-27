#include "main.h"

#include <stdint.h>
#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <util/delay.h>

#include "include/iv4.h"      		// IV4 bitfield defines and functions
#include "include/shift.h"    		// Generic shift register library
#include "include/words.c"    		// Letter-pair data
#include "include/usb/usbconfig.h"	// USB configuration header
#include "include/usb/usbdrv.h"		// USB driver header

const int32_t EEMEM lastseed = 0xDEADBEEF;
volatile uint8_t up;				// Flag for signaling display updates from timer2 interrupt

uint8_t k;							// grid mux counter
uint8_t bitmap[4][3];				// Buffer for literal bitfields that go to the shift register
uint8_t charmap[4];					// Buffer for logical characters
uint8_t stateReg;					// Store global state
utime_t timeReg;					// Store global time (HH:MM:SS:ticks)
int32_t stx; 						// PRNG state variable.  We never return this directly.

static uchar currentPosition, bytesRemaining;

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
	uchar   i;

	// NVRAM: get last used random seed, modify, and write back to NVRAM
	eeprom_busy_wait();
	eeprom_read_block((void*) &stx, (const void*) &lastseed, 4);
	stx += 0xF0A110AF;
	eeprom_busy_wait();
	eeprom_write_block((const void*) &stx, (void*) &lastseed, 4);

	// INIT: IO ports
	initHW();

	// INIT: all timers
	initTimers();

	// INIT: ADC stuff
	initADC();

	cli();
	usbDeviceDisconnect();  		// enforce re-enumeration, do this while interrupts are disabled!
    i = 0;
    while(--i) {             		// fake USB disconnect for > 250 ms
        _delay_ms(1);
    }
    usbDeviceConnect();
    sei();

    // INIT: USB
    usbInit();
}

static inline void initTimers(void) {
	TCNT1 = 0;										// reset TCNT1
	TIMSK1 |= _BV(TOIE1);							// enable TCNT1 overflow

	TCCR1A &= ~(_BV(WGM10));						// set PWM mode with ICR top-count
	TCCR1A |= _BV(WGM11);
	TCCR1B |= (_BV(WGM12) | _BV(WGM13));

	ICR1 = 500;										// set top count value
	OCR1A = 50;										// set output compare value A
	OCR1B = 0;										// clear output compare value B

	TCCR1A |= _BV(COM1A1);							// turn on channel A PWM output, set OC1A as non-inverted PWM
	TCCR1A &= ~(_BV(COM1A0));

	TCCR1B &= ~(_BV(CS12) | _BV(CS11) | _BV(CS10));	// clear timer1 clock source
	TCCR1B |= _BV(CS10);							// set timer1 clock source to Fclk/1

	// ***** Timer2 *****
	//   Prescaler = Fclk/256 = 62.k kHz; t = 16 us
	//   Topcount = 250; topcount * t = 4 ms

	TCNT2 = 0;										// reset TCNT2
	TIMSK2 |= _BV(TOIE2);							// enable TCNT2 overflow

	//OCR2A = 249;									// set timer2 top count

	//TCCR2A |= _BV(WGM21);						// set timer2 CTC mode
	TCCR2B &= ~(_BV(CS22) | _BV(CS21) | _BV(CS20));	// clear timer2 clock source
	TCCR2B |= (_BV(CS22) | _BV(CS21));				// set timer2 clock source to Fclk/256
}

static inline void initADC(void) {
	ADMUX |= _BV(REFS0);							// set ADC voltage reference to external AVCC
	ADMUX |= _BV(MUX0);								// set ADC multiplexer to ADC1, our feedback channel
	ADMUX |= _BV(ADLAR);							// set ADC to left-adjust results
	ADCSRA |= _BV(ADEN);							// enable ADC circuitry
	ADCSRA |= _BV(ADIE);							// enable ADC conversion-completion interrupt
	ADCSRA |= (_BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0)); 	// set ADC prescaler to 128
	DIDR0 |= _BV(ADC1D);							// disable digital input buffer on ADC1
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
}

ISR(ADC_vect) {
	// ADC result of 0x1e1 corresponds to an HV of 50V.  Ignore the two LSBs to create a bit of deadband
	//  and improve noise-immunity.
	if (ADCH > 0x20) {
		// voltage is too high!  back off.
		ICR1 = ICR1--;
	}
	if (ADCH < 0x1c) {
		// voltage is too low!  more power argh argh argh.
		ICR1 = ICR1++;
	}
}

ISR(TIMER1_OVF_vect) {
	// OCR1A is attached to the input of our boost convertor.
}

ISR(TIMER2_OVF_vect) {
	TCNT2 += 6;
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
	// TODO: maybe obsolete this code in favor of the version in libc
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
	timeReg.ticks += 1;

	if (timeReg.ticks == 250) {	// 250 ticks in a second
		timeReg.seconds++;
		timeReg.ticks = 0;
	}
	if (timeReg.seconds == 60) {	// 60 seconds in a minute
		timeReg.minutes += 1;
		timeReg.seconds = 0;
	}
	if (timeReg.minutes == 60) { 	// 60 minutes in an hour
		timeReg.hours += 1;
		timeReg.minutes = 0;
	}
	if (timeReg.hours == 24) {		// count 24-hours for now
		timeReg.hours = 0;
	}
}

void display_time(void) {
	// secs+mins for now
	uint8_t secondsOnes = (timeReg.seconds) % 10;
	uint8_t secondsTens = (timeReg.seconds) / 10;
	uint8_t minutesOnes = (timeReg.minutes) % 10;
	uint8_t minutesTens = (timeReg.minutes) / 10;

	charmap[3] = secondsOnes + 48;  // using itoa here for some reason clobbers the buffer
	charmap[2] = secondsTens + 48;
	charmap[1] = minutesOnes + 48;
	charmap[0] = minutesTens + 48;

}

usbMsgLen_t usbFunctionSetup(uchar data[8]) {
usbRequest_t    *rq = (void *)data;

    if ((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_VENDOR) {
    	switch (rq->bRequest) {
    	case CUSTOM_RQ_SET_STATE:
    		stateReg = rq->wValue.bytes[0];
    		return 0;
    	case CUSTOM_RQ_GET_STATE:
    		usbMsgPtr = &stateReg;
    		return 1;
    	case CUSTOM_RQ_SET_BUFFER:
    		currentPosition = 0;
    		bytesRemaining = rq->wLength.word;
    		if (bytesRemaining > sizeof(charmap))
    			bytesRemaining = sizeof(charmap);
    		return USB_NO_MSG;
    	case CUSTOM_RQ_GET_BUFFER:
    		usbMsgPtr = charmap;
    		return 4;
    	case CUSTOM_RQ_SET_HOURS:
    		timeReg.hours = rq->wValue.bytes[0];
    		return 0;
    	case CUSTOM_RQ_GET_HOURS:
    		usbMsgPtr = &timeReg.hours;
    		return 1;
    	case CUSTOM_RQ_SET_MINS:
    		timeReg.minutes = rq->wValue.bytes[0];
    		return 0;
    	case CUSTOM_RQ_GET_MINS:
    		usbMsgPtr = &timeReg.minutes;
    		return 1;
    	case CUSTOM_RQ_SET_SECS:
    		timeReg.seconds = rq->wValue.bytes[0];
    		return 0;
    	case CUSTOM_RQ_GET_SECS:
    		usbMsgPtr = &timeReg.seconds;
    		return 1;
    	}
    } else {
        /* class requests USBRQ_HID_GET_REPORT and USBRQ_HID_SET_REPORT are
         * not implemented since we never call them. The operating system
         * won't call them either because our descriptor defines no meaning.
         */
    }
    return 0;   /* default for not implemented requests: return no data back to host */
}

uchar usbFunctionWrite(uchar *data, uchar len) {
	uchar i;
	if (len > bytesRemaining)
		len = bytesRemaining;
	bytesRemaining -= len;
	for(i = 0; i < len; i++)
		charmap[currentPosition++] = data[i];
	return bytesRemaining == 0;
}

void do_display(void) {
	k++;
	if (k >= 4) {
		k = 0;
	}

	bufferChar(bitmap[k], (uint8_t)charmap[k]);

	SHRBlank();
	selectGrid(bitmap[k], k);
	SHRSendBuffer(bitmap[k], k);
	SHRLatch();
	SHRUnblank();
}

/* main function */
int16_t main(void) {
	init();

	charmap[0] = 'A';
	charmap[1] = 'B';
	charmap[2] = 'C';
	charmap[3] = 'D';

    for (;;) {              /* main event loop */
        usbPoll();

        if(up == 1) {
        	update_time();
        	do_display();
        	up = 0;
        }

        if(stateReg == 'T') {
        	display_time();
        } else {
        }
    }

    return 0;
}
