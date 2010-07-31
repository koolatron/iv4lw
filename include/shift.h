#define SHR_DDR DDRB
#define SHR_PORT PORTB
#define CLOCK 4
#define LATCH 5
#define BLANK 3
#define DATA  2

// The number of bits in our register, divided by 8, rounded up.
// inputs should be right-justified (so the zeros go in first)
#define SHR_BYTES 3

void initSHR(void);
void SHRSendByte(unsigned char byte);
void SHRSendBuffer(unsigned char* buffer, unsigned char n);
void SHRLatch(void);
void SHRReset(void);
void SHRBlank(void);
void SHRUnblank(void);
