#define SHR_DDR DDRC
#define SHR_PORT PORTC
#define CLOCK 4
#define LATCH 5
#define BLANK 3
#define DATA  2

void initSHR(void);
void SHRSendByte(unsigned char byte);
void SHRSendBuffer(unsigned char* buffer, unsigned char n);
void SHRLatch(void);
void SHRReset(void);
void SHRBlank(void);
void SHRUnblank(void);
