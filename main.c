#include "main.h"

#include <stdint.h>
#include <stdlib.h>
#include <avr/io.h>
#include <string.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <util/delay.h>

#include "include/iv4.h"      		// IV4 bitfield defines and functions
#include "include/pairs.h"			// IV4 letter pair data
#include "include/shift.h"    		// Generic shift register library
#include "include/usb/usbconfig.h"	// USB configuration header
#include "include/usb/usbdrv.h"		// USB driver header

uint32_t EEMEM seed = 0xDEADBEEF;   // Random seed, lives in EEPROM

volatile uint8_t up;                // Flag for signaling display updates from timer2 interrupt

uint8_t k;                          // grid mux counter
uint8_t b1, b2, b3;					// buttons
uint8_t bitmap[4][3];               // Buffer for literal bitfields that go to the shift register
uint8_t charmap[4];                 // Backing buffer for logical characters
uint8_t timemap[4];					// Backing buffer for time values
uint8_t stateReg;                   // Store global state
utime_t timeReg;                    // Store global time (HH:MM:SS:ticks)
uint8_t adcReg;						// Store global ADC data
uint8_t waitTicks;					// global counter to manage waits
uint8_t exitState;					// state to exit waits to
uint8_t *usbWritePtr;               // Pointer to a buffer where we intend to recieve data
uint8_t *displayBuffer;				// Pointer the the buffer we currently want to display

static uchar currentPosition, bytesRemaining;

PROGMEM char usbHidReportDescriptor[22] = {
	0x06, 0x00, 0xff,               // USAGE_PAGE (Generic Desktop)
	0x09, 0x01,                     // USAGE (Vendor Usage 1)
	0xa1, 0x01,                     // COLLECTION (Application)
	0x15, 0x00,                     //   LOGICAL_MINIMUM (0)
	0x26, 0xff, 0x00,               //   LOGICAL_MAXIMUM (255)
	0x75, 0x08,                     //   REPORT_SIZE (8)
	0x95, 0x01,                     //   REPORT_COUNT (1)
	0x09, 0x00,                     //   USAGE (Undefined)
	0xb2, 0x02, 0x01,               //   FEATURE (Data,Var,Abs,Buf)
	0xc0                            // END_COLLECTION
};

// general device initialization
void init(void) {
	uchar i;
    uint32_t seed;

	// INIT: IO ports
	initIO();

	// INIT: all timers
	initTimers();

	// INIT: ADC stuff
	initADC();

    // Generate a new random seed
    seed = eeprom_read_dword( (uint32_t *) 0 );
    seed += 1;
    eeprom_write_dword( (uint32_t *) 0, seed );
    srandom( seed );

    // Fake a USB device disconnect
	cli();
	usbDeviceDisconnect();
	i = 0;
	while (--i) {
		_delay_ms(1);
	}
	usbDeviceConnect();
	sei();

	// INIT: USB
	usbInit();
}

static inline void initTimers(void) {
	TCNT1 = 0;                      // reset TCNT1
	TIMSK1 |= _BV(TOIE1);           // ???: enable TCNT1 overflow

	TCCR1A &= ~(_BV(WGM10));        // set PWM mode with ICR top-count
	TCCR1A |= _BV(WGM11);
	TCCR1B |= (_BV(WGM12) | _BV(WGM13));

	ICR1 = 220;                     // set top count value -- controls frequency
	OCR1A = 20;                     // set output compare value A -- controls duty cycle
	OCR1B = 0;                      // clear output compare value B

	TCCR1A |= _BV(COM1A1);          // turn on channel A PWM output, set OC1A as non-inverted PWM
	TCCR1A &= ~(_BV(COM1A0));

	TCCR1B &= ~(_BV(CS12) | _BV(CS11) | _BV(CS10)); // clear timer1 clock source
	TCCR1B |= _BV(CS10);            // set timer1 clock source to Fclk/1

	TCNT2 = 0;                      // reset TCNT2
	TIMSK2 |= _BV(OCIE2A);          // enable OCR2A CTC interrupt

	TCCR2A |= _BV(WGM21);			// set timer2 to CTC mode

	OCR2A = 249;					// set top count value

	TCCR2B &= ~(_BV(CS22) | _BV(CS21) | _BV(CS20)); // clear timer2 clock source
	TCCR2B |= (_BV(CS22) | _BV(CS21)); // set timer2 clock source to Fclk/256
}

static inline void initADC(void) {
    ADMUX |= _BV(REFS0);    // set reference to external AVCC
	ADMUX |= _BV(ADLAR);	// left-adjust A2D results
	ADMUX |= (_BV(MUX0) | _BV(MUX2));	// select ADC5 as A2D input channel
	ADCSRA |= _BV(ADEN);	// enable A2D circuitry
	ADCSRA |= _BV(ADIE);	// enable A2D conversion-complete interrupt
	ADCSRA |= (_BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0));	// set A2D prescaler to 128
	DIDR0 |= _BV(ADC1D);	// disable digital input buffer on A2D input
}

static inline void initIO(void) {
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
	DDRC &= ~_BV(5);
}

// ADC_vect is called whenever an ADC conversion completes.
ISR(ADC_vect) {
	if (ADCH < 0x5C) {
//	if (ADCH < 0x73) {  // 55V
		OCR1A += 1;
	}
	if (ADCH > 0x68) {
//	if (ADCH > 0x7E) { // 55V
		OCR1A -= 1;
	}
}

// For some insane reason, this needs to be declared or things break
ISR(TIMER1_OVF_vect) {
}

// TIMER2_COMPA_vect is called on a CTC match between TCNT2 and OCR2A.
ISR(TIMER2_COMPA_vect) {
	up = 1;
}


void update_time(void) {
	timeReg.time.ticks += 1;

	if (timeReg.time.ticks == 250) {  // 250 ticks in a second
		timeReg.time.seconds++;
		timeReg.time.ticks = 0;
	}
	if (timeReg.time.seconds == 60) { // 60 seconds in a minute
		timeReg.time.minutes++;
		timeReg.time.seconds = 0;
	}
	if (timeReg.time.minutes == 60) { // 60 minutes in an hour
		timeReg.time.hours++;
		timeReg.time.minutes = 0;
	}
	if (timeReg.time.hours == 24) {   // count 24-hours for now
		timeReg.time.hours = 0;
	}

	timemap[3] = ((timeReg.time.minutes) % 10) + 48;
	timemap[2] = ((timeReg.time.minutes) / 10) + 48;
	timemap[1] = ((timeReg.time.hours) % 10) + 48;
	timemap[0] = ((timeReg.time.hours) / 10) + 48;
}

static inline void set_display_buffer(uint8_t* buffer) {
	displayBuffer = buffer;
}

usbMsgLen_t usbFunctionSetup(uchar data[8]) {
	usbRequest_t *rq = (void *) data;

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
			usbWritePtr = charmap;
			return USB_NO_MSG;
		case CUSTOM_RQ_GET_BUFFER:
			usbMsgPtr = charmap;
			return 4;
		case CUSTOM_RQ_SET_TIME:
			currentPosition = 0;
			bytesRemaining = rq->wLength.word;
			if (bytesRemaining > sizeof(timeReg.raw))
				bytesRemaining = sizeof(timeReg.raw);
			usbWritePtr = timeReg.raw;
			return USB_NO_MSG;
		case CUSTOM_RQ_GET_TIME:
			usbMsgPtr = timeReg.raw;
			return 4;
		case CUSTOM_RQ_SET_RAW:
			displayBuffer[rq->wValue.bytes[0]] = 0xff;
			currentPosition = 0;
			bytesRemaining = rq->wLength.word;
			if (bytesRemaining > sizeof(bitmap[rq->wValue.bytes[0]]))
				bytesRemaining = sizeof(bitmap[rq->wValue.bytes[0]]);
			usbWritePtr = bitmap[rq->wValue.bytes[0]];
			return USB_NO_MSG;
		case CUSTOM_RQ_GET_ADC:
			usbMsgPtr = &adcReg;
			return 1;
		}
	} else {
		/* class requests USBRQ_HID_GET_REPORT and USBRQ_HID_SET_REPORT are
		 * not implemented since we never call them. The operating system
		 * won't call them either because our descriptor defines no meaning.
		 */
	}
	return 0; /* default for not implemented requests: return no data back to host */
}

uchar usbFunctionWrite(uchar *data, uchar len) {
	uchar i;
	if (len > bytesRemaining)
		len = bytesRemaining;
	bytesRemaining -= len;
	for (i = 0; i < len; i++)
		usbWritePtr[currentPosition++] = data[i];
	return bytesRemaining == 0;
}

void do_display(void) {
	k++;
	if (k >= 4)
		k = 0;

	bufferChar(bitmap[k], (uint8_t) displayBuffer[k]);

	SHRBlank();
	selectGrid(bitmap[k], k);
	SHRSendBuffer(bitmap[k], k);
	SHRLatch();
	SHRUnblank();
}

void do_buttons(void) {
	if (bit_is_clear(PINB, 3)) {
		if ( b1 != ON ) {
			b1++;
		}
	} else {
		b1 = OFF;
	}

	if (bit_is_clear(PINB, 4)) {
		if ( b2 != ON ) {
			b2++;
		}
	} else {
		b2 = OFF;
	}

	if (bit_is_clear(PINB, 5)) {
		if ( b3 != ON ) {
			b3++;
		}
	} else {
		b3 = OFF;
	}
}

// All we do here is start a new conversion.  An interrupt vector
// handles actually using this information.
void do_adc(void) {
	adcReg = ADCH;			// stash current ADC value
    if ( bit_is_clear( ADCSRA, ADIF ) ) {
        ADCSRA |= _BV(ADSC);	// start the next conversion
    }
}

// This is an extravagant waste of resources
uint8_t random_8(void) {
    uint32_t random_32;
    uint8_t random_8;

    random_32 = random();
    random_8 = (uint8_t)random_32;

    return random_8;
}

uint8_t* rand_word( uint8_t* buf ) {
	uint8_t ab, bc, cd;
	uint8_t a, b, c, d;

	uint8_t done = 0;

	while ( !done ) {
		ab = random_8() % 216;
		cd = random_8() % 239;

		b = pgm_read_byte( &(first_pairs[ab][1]) );
	    c = pgm_read_byte( &(last_pairs[cd][0]) );

	    for (bc=0; bc<254; bc++) {
	    	if ( (pgm_read_byte( &(middle_pairs[bc][0]) ) == b) && (pgm_read_byte( &(middle_pairs[bc][1]) ) == c) ) {
	    		a = pgm_read_byte( &(first_pairs[ab][0]) );
	    	    d = pgm_read_byte( &(last_pairs[cd][1]) );

	    		done = 1;
	    	}
	    }
	}

	buf[0] = a;
	buf[1] = b;
	buf[2] = c;
	buf[3] = d;

	return buf;
}

/* main function */
int16_t main(void) {
	init();

	DDRD |= _BV(5);
	PORTD |= _BV(5);

	stateReg = 'T';
	exitState = 'T';

	for (;;) { /* main event loop */
		usbPoll();

		if (up == 1) {
			update_time();	// Update time registers
			do_display();	// Update display register
			do_buttons();	// Update button registers
			do_adc();		// Start next ADC conversion

			if (waitTicks > 0) {
				waitTicks--;
			}

			up = 0;			// Get ready for next update
		}

		switch (stateReg) {
		case 'T':
			set_display_buffer(timemap);

			if (b3 == ON) {
				b3 = OFF;
				timeReg.time.hours++;
			}
			if (b2 == ON) {
				b2 = OFF;
				timeReg.time.minutes++;
			}
			if (b1 == ON) {
				b1 = OFF;

                set_display_buffer(charmap);
                strcpy_PF( (char *)charmap, PSTR("WORD") );
                waitTicks = 250;
                exitState = 'U';
				stateReg = 'W';
			}

			break;
		case 'U':
			set_display_buffer(charmap);

            rand_word( charmap );
            waitTicks = 250;
            exitState = 'U';
            stateReg = 'W';

			if (b3 == ON) {
				b3 = OFF;
			}
			if (b2 == ON) {
				b2 = OFF;
			}
			if (b1 == ON) {
				b1 = OFF;

                set_display_buffer(charmap);
                strcpy_PF( (char *)charmap, PSTR("TIME") );
                waitTicks = 250;
                exitState = 'T';
				stateReg = 'W';
			}

			break;
		case 'V':
			set_display_buffer(charmap);
			break;
		case 'W': // this is a wait state -- just chill out.
			if ( b1 == ON ) {
				waitTicks = 0;
			}

			if (waitTicks == 0) {
				stateReg = exitState;
			}
			break;
		default:
			break;
		}
	}

	return 0;
}
