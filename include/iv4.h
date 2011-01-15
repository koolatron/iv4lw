/*
** iv4.h - header file for iv4.c containing character table defines and
**         function prototypes for the iv4lw
** Requires:    stdint.h
** Required by: iv4.c
*/

#ifndef IV4_H
#define IV4_H

/*  BIT CHAIN:
 *	lowercase letters: segments
 *	uppercase letters: grids
 *	---- ABpo   ahgm nfDC   ledc bijk
 *  ** Bitfield defines must have 0 on all grids! **
 *	---- --po   ahgm nf--   ledc bijk
 */

#define blank     0x00, 0x00, 0x00
#define char_A    0x01, 0xe0, 0x39
#define char_B    0x00, 0x94, 0x7d
#define char_C    0x00, 0xe4, 0x48
#define char_D    0x00, 0x94, 0x7c
#define char_E    0x01, 0xe4, 0x48
#define char_F    0x01, 0xe0, 0x08
#define char_G    0x01, 0xe4, 0x69
#define char_H    0x01, 0x60, 0x31
#define char_I    0x00, 0x94, 0x4c
#define char_J    0x00, 0x24, 0x70
#define char_K    0x01, 0x60, 0x82
#define char_L    0x00, 0x64, 0x40
#define char_M    0x02, 0x60, 0x32
#define char_N    0x02, 0x60, 0xb0
#define char_O    0x00, 0xe4, 0x78
#define char_P    0x01, 0xe0, 0x19
#define char_Q    0x00, 0xe4, 0xf8
#define char_R    0x01, 0xe0, 0x99
#define char_S    0x01, 0xc4, 0x69
#define char_T    0x00, 0x90, 0x0c
#define char_U    0x00, 0x64, 0x70
#define char_V    0x00, 0x68, 0x02
#define char_W    0x00, 0x68, 0xb0
#define char_Y    0x02, 0x10, 0x02
#define char_X    0x02, 0x08, 0x82
#define char_Z    0x00, 0x8c, 0x4a
#define char_0    0x00, 0xec, 0x7a
#define char_1    0x00, 0x00, 0x30
#define char_2    0x01, 0xa4, 0x59
#define char_3    0x00, 0x84, 0x79
#define char_4    0x01, 0x40, 0x31
#define char_5    0x01, 0xc4, 0x69
#define char_6    0x01, 0xe4, 0x69
#define char_7    0x00, 0x80, 0x38
#define char_8    0x01, 0xe4, 0x79
#define char_9    0x01, 0xc0, 0x39
#define char_col  0x00, 0x00, 0x00
#define char_scol 0x00, 0x00, 0x00
#define char_lt   0x00, 0x00, 0x00
#define char_eq   0x00, 0x00, 0x00
#define char_gt   0x00, 0x00, 0x00
#define char_qmrk 0x00, 0x00, 0x00
#define char_at   0x00, 0x00, 0x00
#define char_all  0x03, 0xfc, 0xff

uint8_t* bufferBytes(uint8_t* buffer, uint8_t index);
uint8_t* bufferChar(uint8_t* buffer, uint8_t index);
uint8_t* selectGrid(uint8_t* buffer, uint8_t n);

#endif
