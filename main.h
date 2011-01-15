#include <avr/pgmspace.h>
#include <stdint.h>

#define OFF 5
#define ON 0
#define RAND_MAX 0x7FFFFFFF

#define F_CPU 16000000

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
#define CUSTOM_RQ_SET_TIME		11
#define CUSTOM_RQ_GET_TIME		12
#define CUSTOM_RQ_SET_RAW		13
#define CUSTOM_RQ_GET_RAW		14

typedef struct {
	uint8_t ticks;
	uint8_t seconds;
	uint8_t minutes;
	uint8_t hours;
} time_t;

typedef union {
	uint8_t raw[4];
	time_t time;
} utime_t;

int16_t main(void);
void init(void);
static inline void initHW(void);
static inline void initTimers(void);
static inline void initADC(void);
uint8_t prng_8(int32_t* x);
void update_time(void);
void display_time(void);
void set_buffer(void);
void set_display_buffer(uint8_t* buffer);
void do_display(void);
void gen_word(void);
