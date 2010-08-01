#include "main.h"
#include "include/global.h"

#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
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
volatile uint8_t b1, b2, b3; 		// asynchronously-updated button state
volatile uint8_t updown;			// asynchronously-updated ADC state

int32_t stx; 						// PRNG state variable.  We never return this directly.

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
	PORTD = 0x00;
	DDRB = 0xFF;
	DDRC = 0xFF;
	DDRD = 0xFF;

	// INIT: Shift Register
	initSHR();

	// INIT: Button inputs
	DDRB &= ~(_BV(3) | _BV(4) | _BV(5));

	// INIT: USB IOs (configure as inputs, disable pullups!)
	DDRD &= ~(_BV(1) | _BV(2));
	usbInit();

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
	if (ADCH > 0x1e) {
		// voltage is too high!  back off.
		// TCNT1 = TCNT1--;
	}
	if (ADCH < 0x1e) {
		// voltage is too low!  more power argh argh argh.
		// TCNT1 = TCNT1++;
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
	// Timer2 is 8-bits, suitable for either very short or coarsely-defined delays.  It supports PWM.  Timer2 is suitable for RTC operation.

	/* crude-but-effective button polling scheme that doesn't cause interrupt contention and doesn't involve busy-waiting.
	 * You can adjust the sensitivity of the buttons by messing around with the OFF define in main.h - higher = less sensitive
	 */
	if (PINB & 0x08) {
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
	}
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
	// TODO: implement USB functionality
	return 0;
}

/* main function */
int16_t main(void) {
	init();

	while (1) {
		_delay_ms(10);
	}

	return 0;
}
