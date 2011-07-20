#include <stdint.h>
#include <avr/pgmspace.h>

#include "iv4.h"

static const uint8_t chartable[76][3] PROGMEM = {
		{blank}, // '/'
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
		{char_col},
		{char_scol},
		{char_lt},
		{char_eq},
		{char_gt},
		{char_qmrk},
		{char_at},
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
		{char_Z},
		{char_all},
		{char_all},
		{char_all},
		{char_all},
		{char_all},
		{char_all},
		{char_a},
		{char_b},
		{char_c},
		{char_d},
		{char_e},
		{char_f},
		{char_g},
		{char_h},
		{char_i},
		{char_j},
		{char_k},
		{char_l},
		{char_m},
		{char_n},
		{char_o},
		{char_p},
		{char_q},
		{char_r},
		{char_s},
		{char_t},
		{char_u},
		{char_v},
		{char_w},
		{char_x},
		{char_y},
		{char_z}
	};

uint8_t* bufferBytes(uint8_t* buffer, uint8_t index) {
	buffer[0] = pgm_read_byte(&chartable[index][0]);
	buffer[1] = pgm_read_byte(&chartable[index][1]);
	buffer[2] = pgm_read_byte(&chartable[index][2]);

	return buffer;
}

uint8_t* bufferChar(uint8_t* buffer, uint8_t index) {
	if (index == 0xff) {  		// 0xff means don't touch this character
		return buffer;
	}

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
		buffer[2] |= 0x40;
	}
	if (n == 3) {
		buffer[2] |= 0x80;
	}

	return buffer;
}
