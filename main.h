#include <avr/pgmspace.h>
#include <stdint.h>

#define OFF 5
#define ON 0
#define RAND_MAX 0x7FFFFFFF

#define F_CPU 16000000

#define STATE_WORD 10
#define STATE_TIME 20

#define CUSTOM_RQ_SET_STATE	    1
#define CUSTOM_RQ_GET_STATE	    2
#define CUSTOM_RQ_SET_BUFFER	3
#define CUSTOM_RQ_GET_BUFFER	4
#define CUSTOM_RQ_SET_HOURS		5
#define CUSTOM_RQ_GET_HOURS		6
#define CUSTOM_RQ_SET_MINS		7
#define CUSTOM_RQ_GET_MINS		8
#define CUSTOM_RQ_SET_SECS		9
#define CUSTOM_RQ_GET_SECS		10

typedef struct {
	uint8_t ticks;
	uint8_t seconds;
	uint8_t minutes;
	uint8_t hours;
} utime_t;

int16_t main(void);
void init(void);
static inline void initHW(void);
static inline void initTimers(void);
static inline void initADC(void);
uint8_t prng_8(int32_t* x);
void do_display(void);
void gen_word(void);
void update_time(void);
