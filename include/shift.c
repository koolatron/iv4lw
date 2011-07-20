#include <stdint.h>
#include <avr/io.h>
#include <avr/sfr_defs.h>

#include "shift.h"

void initSHR(void) {
	SHR_DDR |= (1 << DATA) | (1 << BLANK) | (1 << CLOCK) | (1 << LATCH);

	SHR_PORT &= (unsigned char) ~(1 << DATA);
	SHR_PORT &= (unsigned char) ~(1 << BLANK);
	SHR_PORT &= (unsigned char) ~(1 << CLOCK);
	SHR_PORT &= (unsigned char) ~(1 << LATCH);
}

inline void SHRBlank(void) {
	SHR_PORT |= (unsigned char) (1 << BLANK);
	asm("nop");
}

inline void SHRUnblank(void) {
	SHR_PORT &= (unsigned char) ~(1 << BLANK);
	asm("nop");
}

void SHRSendByte(unsigned char byte) {
	int8_t i;

	for (i = 7; i >= 0; i--) {
		SHR_PORT &= (unsigned char) ~((1 << DATA) | (1 << CLOCK));
		asm("nop");
		SHR_PORT |= (unsigned char) (((byte >> i) & 1) << DATA);
		asm("nop");
		SHR_PORT |= (unsigned char) (1 << CLOCK);
		asm("nop");
	}

	SHR_PORT &= (unsigned char) ~(1 << CLOCK);
	asm("nop");

}

void SHRSendBuffer(unsigned char* buffer, unsigned char n) {
	SHRSendByte(buffer[0]);
	SHRSendByte(buffer[1]);
	SHRSendByte(buffer[2]);
}

inline void SHRLatch(void) {
	SHR_PORT |= (unsigned char) (1 << LATCH);
	asm("nop");
	SHR_PORT &= (unsigned char) ~(1 << LATCH);
	asm("nop");
}

void SHRReset(void) {
}
