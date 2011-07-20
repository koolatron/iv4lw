#define SHR_DDR DDRC
#define SHR_PORT PORTC
#define CLOCK 1
#define LATCH 2
#define DATA  3
#define BLANK 4

void initSHR(void);
void SHRSendByte(unsigned char byte);
void SHRSendBuffer(unsigned char* buffer, unsigned char n);
void SHRLatch(void);
void SHRReset(void);
void SHRBlank(void);
void SHRUnblank(void);
