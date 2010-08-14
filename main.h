#include <avr/pgmspace.h>
#include <stdint.h>

#define OFF 5
#define ON 0
#define RAND_MAX 0x7FFFFFFF

#define F_CPU 16000000

#define STATE_WORD 10
#define STATE_TIME 20

#define CUSTOM_RQ_SET_STATUS    1
#define CUSTOM_RQ_GET_STATUS    2

int16_t main(void);
void init(void);
static inline void initHW(void);
void timer0Overflow(void);
void timer1Overflow(void);
void timer2Overflow(void);
uint8_t prng_8(int32_t* x);
void do_display(void);
void gen_word(void);
void update_time(void);
