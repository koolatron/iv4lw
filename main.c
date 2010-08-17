#include "main.h"

#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

#include "include/usb/usbconfig.h"	// USB configuration header
#include "include/usb/usbdrv.h"		// USB driver header

PROGMEM char usbHidReportDescriptor[22] = {    /* USB report descriptor */
    0x06, 0x00, 0xff,              // USAGE_PAGE (Generic Desktop)
    0x09, 0x01,                    // USAGE (Vendor Usage 1)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0x09, 0x00,                    //   USAGE (Undefined)
    0xb2, 0x02, 0x01,              //   FEATURE (Data,Var,Abs,Buf)
    0xc0                           // END_COLLECTION
};

// general device initialization
void init(void) {
    usbInit();
}

usbMsgLen_t usbFunctionSetup(uchar data[8]) {
usbRequest_t    *rq = (void *)data;

    if((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_VENDOR){
        //DBG1(0x50, &rq->bRequest, 1);   /* debug output: print our request */
        if(rq->bRequest == CUSTOM_RQ_SET_STATUS){
            if(rq->wValue.bytes[0] & 1){    /* set LED */
            }else{                          /* clear LED */
            }
        }else if(rq->bRequest == CUSTOM_RQ_GET_STATUS){
            static uchar dataBuffer[1];     /* buffer must stay valid when usbFunctionSetup returns */
            dataBuffer[0] = 0;
            usbMsgPtr = dataBuffer;         /* tell the driver which data to return */
            return 1;                       /* tell the driver to send 1 byte */
        }
    } else {
    }
    return 0;
}

/* main function */
int16_t main(void) {
	uchar   i;

	cli();

	init();
    usbDeviceConnect();
    _delay_ms(1);
    sei();

    for (;;) {              /* main event loop */
        usbPoll();
    }

    return 0;
}
