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
}

inline void SHRUnblank(void) {
	SHR_PORT &= (unsigned char) ~(1 << BLANK);
}

void SHRSendByte(unsigned char byte) {
	int8_t i;

	for (i = 7; i >= 0; i--) {
		SHR_PORT &= (unsigned char) ~((1 << DATA) | (1 << CLOCK));
		SHR_PORT |= (unsigned char) (((byte >> i) & 1) << DATA);
		SHR_PORT |= (unsigned char) (1 << CLOCK);
	}

	asm("nop");
	SHR_PORT &= (unsigned char) ~(1 << CLOCK);
}

void SHRSendBuffer(unsigned char* buffer, unsigned char n) {
	uint8_t bytes = n / 8;
	int8_t i, j;

	for (i = bytes; i >= 0; i--) {
		uint8_t byte = buffer[i];

		for (j = 7; j >= 0; j--) {
			SHR_PORT &= (unsigned char) ~((1 << DATA) | (1 << CLOCK));
			SHR_PORT |= (unsigned char) (((byte >> j) & 1) << DATA);
			SHR_PORT |= (unsigned char) (1 << CLOCK);
		}
	}

	asm("nop");
	SHR_PORT &= (unsigned char) ~(1 << CLOCK);
}

inline void SHRLatch(void) {
	SHR_PORT |= (unsigned char) (1 << LATCH);
	asm("nop");
	SHR_PORT &= (unsigned char) ~(1 << LATCH);
}

void SHRReset(void) {
}
