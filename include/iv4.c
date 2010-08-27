#include <stdint.h>
#include <avr/pgmspace.h>

#include "iv4.h"

static const uint8_t chartable[44][3] PROGMEM = {
  {blank},    // '/'
  {char_0},
  {char_1},
  {char_2},
  {char_3},
  {char_4},
  {char_5},
  {char_6},
  {char_7},
  {char_8},
  {char_9},
  {char_A},   // ':'
  {char_B},   // ';'
  {char_C},   // '<'
  {char_D},   // '='
  {char_E},   // '>'
  {char_F},   // '?'
  {char_all}, // '@'
  {char_A},
  {char_B},
  {char_C},
  {char_D},
  {char_E},
  {char_F},
  {char_G},
  {char_H},
  {char_I},
  {char_J},
  {char_K},
  {char_L},
  {char_M},
  {char_N},
  {char_O},
  {char_P},
  {char_Q},
  {char_R},
  {char_S},
  {char_T},
  {char_U},
  {char_V},
  {char_W},
  {char_X},
  {char_Y},
  {char_Z}
};

uint8_t* bufferBytes(uint8_t* buffer, uint8_t index) {
  buffer[0] = pgm_read_byte(&chartable[index][0]);
  buffer[1] = pgm_read_byte(&chartable[index][1]);
  buffer[2] = pgm_read_byte(&chartable[index][2]);

  return buffer;
}

uint8_t* bufferChar(uint8_t* buffer, uint8_t index) {
  buffer[0] = pgm_read_byte(&chartable[index-47][0]);
  buffer[1] = pgm_read_byte(&chartable[index-47][1]);
  buffer[2] = pgm_read_byte(&chartable[index-47][2]);

  return buffer;
}

uint8_t* selectGrid(uint8_t* buffer, uint8_t n) {
	if (n == 0) {
		buffer[0] |= 0x08;
	}
	if (n == 1) {
		buffer[0] |= 0x04;
	}
	if (n == 2) {
		buffer[1] |= 0x01;
	}
	if (n == 3) {
		buffer[1] |= 0x02;
	}

	return buffer;
}
