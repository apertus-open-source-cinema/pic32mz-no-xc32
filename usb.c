
/*	PIC32MZ USB Demo Code
**
**	Copyright (C) 2019 Herbert Poetzl
**
**	This program is free software; you can redistribute it and/or modify
**	it under the terms of the GNU General Public License 2 as published 
**	by the Free Software Foundation.
*/


/*	Clock Setup
**	
**	24MHz External Oscillator
**	PLL in = (24MHz/2) = 12MHz	[8-16MHz range]
**	PLL freq = 12MHz*32 = 384MHz	[350MHz - 700MHz]
**	PLL out = 384MHz/2 = 192MHz
*/


/*	For USB CDC see CDC120-20101103-track	*/


#define SYS_FREQ (180000000L)
#define GetSystemClock() (180000000ul)	// PIC32MZ System Clock in Hertz.

#define min(a, b) (a > b ? b : a)

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>


// TODO(robin): unsupported for now
/*

	// DEVCFG0
#pragma config BOOTISA = MIPS32
#pragma config ICESEL = ICS_PGx1
#pragma config FECCCON = OFF_UNLOCKED
#pragma config FSLEEP = 0
#pragma config EJTAGBEN = NORMAL
#pragma config DBGPER = PG_ALL

	// DEVCFG1
#pragma	config FDMTEN = OFF
#pragma	config FWDTEN = OFF
#pragma config POSCMOD = EC
#pragma config OSCIOFNC = OFF
#pragma config FSOSCEN = OFF
#pragma config FNOSC = SPLL
#pragma config FCKSM = CSECMD

	// DEVCFG2
#pragma config FPLLICLK = PLL_POSC
#pragma config FPLLIDIV = DIV_2
#pragma config FPLLRNG = RANGE_8_16_MHZ
#pragma config FPLLMULT = MUL_32
#pragma config FPLLODIV = DIV_2
#pragma config UPLLEN = ON
#pragma config UPLLFSEL = FREQ_24MHZ

	// DEVCFG3
#pragma config USERID = 0xC0DE
#pragma config FMIIEN = OFF
#pragma config PGL1WAY = OFF
#pragma config PMDL1WAY = OFF
#pragma config IOL1WAY = OFF
#pragma config FUSBIDIO = ON

	// DEVCP0
#pragma config CP = OFF
*/


#define	OSC_EN_O	LATCbits.LATC15
#define	OSC_EN_T	TRISCbits.TRISC15

enum { FALSE, TRUE };

#define MAXPKT 64

#define EP0TXBUFFSIZE 8 	// Multiples of 8, e.g.- 8 (x8) equals a size of 64 bytes.
#define EP1RXBUFFSIZE 8 	// Also in multiples of 8
#define EP2RXBUFFSIZE 8 	// Also in multiples of 8
#define EP3TXBUFFSIZE 8 	// Also in multiples of 8

#define ASSERT(c) typedef char p__LINE__[(c)?1:-1]

#define	W(w) ((w) & 0xFF), ((w) >> 8)


/********************************************************************
** Device Descriptor						   **
*********************************************************************/

const
unsigned char dev_desc[] = {
	0x12,		// bLength: 18
	0x01,		// bDescriptorType: DEVICE
	0x00, 0x02,	// bcdUSB: (ver 2.0)
	0x02,		// bDeviceClass: Comm
	0x00,		// bDeviceSubClass: unused
	0x00,		// bDeviceProtocol: unused
	0x40,		// bMaxPacketSize0: 64
	0x66, 0x66,	// idVendor: 0x6666
	0x34, 0x12,	// idProduct: 0x1234
	0x00, 0x01,	// bcdDevice: 01.00
	0x01,		// iManufacturer
	0x02,		// iProduct
	0x03,		// iSerialNumber
	0x01		// bNumConfigurations
};

ASSERT(sizeof(dev_desc) == 0x12);

#define	TOTAL_LEN	0x43

/********************************************************************
** Config Descriptor						   **
*********************************************************************/

const
unsigned char conf_desc[] = {
	0x09,		// bLength: 9
	0x02,		// bDescriptorType: CONFIG
	W(TOTAL_LEN),	// wTotalLength: all descriptors 
	0x02,		// bNumInterfaces
	0x01,		// bConfigurationValue
	0x04,		// iConfiguration
	0x80,		// bmAttributes: bus powered
	0xc8,		// bMaxPower: 400mA max

/********************************************************************
** CDC Communication Interface Descriptor			   **
*********************************************************************/

	0x09,		// bLength: 9
	0x04,		// bDescriptorType: INTERFACE
	0x00,		// bInterfaceNumber
	0x00,		// bAlternateSetting
	0x01,		// bNumEndpoints
	0x02,		// bInterfaceClass: Comm
	0x02,		// bInterfaceSubClass: ACM
	0x00,		// bInterfaceProtocol: none
	0x00,		// iInterface

/********************************************************************
** CDC Functional Descriptor					   **
*********************************************************************/

	0x05,		// bLength: 5
	0x24,		// bDescriptorType: CS_INTERFACE
	0x00,		// bDescriptorSubtype: HEADER
	0x10, 0x01,	// bcdCDC

/********************************************************************
** ACM Functional Descriptor					   **
*********************************************************************/

	0x04,		// bLength: 4
	0x24,		// bDescriptorType: CS_INTERFACE
	0x02,		// bDescriptorSubtype: ACM
	0x02,		// bmCapabilities

/********************************************************************
** Union Functional Descriptor					   **
*********************************************************************/

	0x05,		// bLength: 5
	0x24,		// bDescriptorType: CS_INTERFACE
	0x06,		// bDescriptorSubtype: UNION
	0x00,		// bControlInterface: 0
	0x01,		// bSubordinateInterface0: 1

/********************************************************************
** Call Management Functional Descriptor			   **
*********************************************************************/

	0x05,		// bLength: 5
	0x24,		// bDescriptorType: CS_INTERFACE
	0x01,		// bDescriptorsubtype: CM
	0x03,		// bmCapabilities: DIY
	0x01,		// bDataInterface: 1

/********************************************************************
** Notification Endpoint Descriptor				   **
*********************************************************************/

	0x07,		// bLength: 7
	0x05,		// bDescriptorType: ENDPOINT
	0x81,		// bEndpointAddress
	0x03,		// bmAttributes
	0x40, 0x00,	// wMaxPacketSize: 64
	0x08,		// bInterval: 8

/********************************************************************
** CDC Data Interface Descriptor				   **
*********************************************************************/

	0x09,		// bLength: 9
	0x04,		// bDescriptorType: INTERFACE
	0x01,		// bInterfaceNumber
	0x00,		// bAlternateSetting
	0x02,		// bNumEndpoints
	0x0A,		// bInterfaceClass: Data
	0x00,		// bInterfaceSubClass: unused
	0x00,		// bInterfaceProtocol: none
	0x00,		// iInterface

/********************************************************************
** Data OUT Endpoint Descriptor					   **
*********************************************************************/

	0x07,		// bLength: 7
	0x05,		// bDescriptorType: ENDPOINT
	0x02,		// bEndpointAddress
	0x02,		// bmAttributes
	0x00, 0x02,	// wMaxPacketSize: 512
	0x01,		// bInterval: 1

/********************************************************************
** Data IN Endpoint Descriptor					   **
*********************************************************************/

	0x07,		// bLength: 7
	0x05,		// bDescriptorType: ENDPOINT
	0x83,		// bEndpointAddress
	0x02,		// bmAttributes
	0x00, 0x02,	// wMaxPacketSize: 512
	0x01		// bInterval: 1
};

ASSERT(sizeof(conf_desc) == TOTAL_LEN);	// wTotalLength


/********************************************************************
** String Descriptors						   **
*********************************************************************/

#define	STR_DESC(l)	(((((l)+1)<<1) & 0xFF) | 0x0300)

const
unsigned short string0[] = {
	STR_DESC(1),	// bLength + bDescriptorType
	0x0409		// wLANGID: English (US)
};

const
unsigned short string1[] = {
	STR_DESC(11),	// bLength + bDescriptorType
	'A', 'p', 'e', 'r', 't', 'u', 's', '.', 'o', 'r', 'g'
};

const
unsigned short string2[] = {
	STR_DESC(12),	// bLength + bDescriptorType
	'A', 'X', 'I', 'O', 'M', ' ',
	'R', 'e', 'm', 'o', 't', 'e'
};

const
unsigned short string3[] = {
	STR_DESC(8),	// bLength + bDescriptorType
	'A', 'R', '0', '0', '0', '0', '0', '1'
};

const
unsigned short string4[] = {
	STR_DESC(7),	// bLength + bDescriptorType
	'D', 'e', 'f', 'a', 'u', 'l', 't'
};

const
unsigned short string5[] = {
	STR_DESC(3),	// bLength + bDescriptorType
	'C', 'D', 'C'
};





#define	LOOP_SIZE	256
#define	LOOP_MASK	(LOOP_SIZE-1)

volatile char loop[LOOP_SIZE];
volatile unsigned loop_wi = 0;
volatile unsigned loop_ri = 0;

static inline
void	loop_ch(char ch)
{
	loop[loop_wi++ & LOOP_MASK] = ch;
}



#define	BUF_SIZE	4096
#define	BUF_MASK	(BUF_SIZE-1)

volatile char buf[BUF_SIZE];
volatile unsigned buf_wi = 0;
volatile unsigned buf_ri = 0;


static inline
void	buf_ch(char ch)
{
	buf[buf_wi++ & BUF_MASK] = ch;
}

static inline
void	buf_nl(void)
{
	buf_ch('\n');
}

static inline
void	buf_hex(uint8_t hex)
{
	hex &= 0xF;
	if (hex > 9)
	    buf_ch(hex + 'A' - 10);
	else
	    buf_ch(hex + '0');
}

static inline
void	buf_byte(uint8_t val)
{
	buf_hex(val >> 4);
	buf_hex(val);
}

static inline
void	buf_word(uint16_t val)
{
	buf_byte(val >> 8);
	buf_byte(val);
}

static inline
void	buf_long(uint32_t val)
{
	buf_word(val >> 16);
	buf_word(val);
}

static inline
void	buf_str0(const char *str)
{
	while (*str)
	    buf_ch(*str++);
}

static inline
void	uart2_ch(char ch)
{
	while (U2STAbits.UTXBF);
	U2TXREG = ch;
}



enum { S_RESET, S_IDLE, S_TXPKT, S_STATUS, S_ZERO, S_RXPKT };


const char const *sstr[] = {
	[S_RESET] = "RE",
	[S_IDLE] = "ID",
	[S_TXPKT] = "TX",
	[S_STATUS] = "ST",
	[S_ZERO] = "ZE",
	[S_RXPKT] = "RX" };


volatile int ep0s;

volatile int ep0rbc;		// EP0 Received Bytes Count
volatile int ep2rbc;		// EP2 Received Bytes Count

volatile int ep0tbc;		// EP0 Transmit Byte Count
volatile int ep1tbc;		// EP1 Transmit Byte Count
volatile int ep3tbc;		// EP3 Transmit Byte Count

uint8_t volatile *ep0ptr;	// EP0 Transmit Data Pointer
uint8_t volatile *ep1ptr;	// EP1 Transmit Data Pointer
uint8_t volatile *ep3ptr;	// EP3 Transmit Data Pointer

volatile uint8_t ep0data[EP0TXBUFFSIZE * 8];	// USB end point 0 data - USB FIFO size
volatile uint8_t ep1data[EP1RXBUFFSIZE * 8];	// USB end point 1 data - USB FIFO size
volatile uint8_t ep2data[EP2RXBUFFSIZE * 8];	// USB end point 2 data - USB FIFO size
volatile uint8_t ep3data[EP3TXBUFFSIZE * 8];	// USB end point 3 data - USB FIFO size

volatile uint8_t *ptr;

volatile uint16_t bmbRequest;	// bmRequestType, bRequest

volatile uint16_t wValue;
volatile uint16_t wIndex;
volatile uint16_t wLength;

volatile uint8_t dev_addr;
volatile uint8_t usb_conf;





static inline
void usb_ep1_tx(void)
{
	volatile uint8_t *ep1fifo;
	ep1fifo = (uint8_t *)&USBFIFO1;		// for now 8 bits at a time.

	buf_str0("TX1\n");
	buf_ch('[');
	buf_word(ep1tbc);
	buf_ch(']');

	if (!usb_conf)
	    return;

	if (!ep1tbc)
	    return;

	if (USBE1CSR0bits.TXPKTRDY)
	    return;

	buf_ch('|');
	for (int i = 0; i < MAXPKT; i++) {
	    buf_byte(*ep1ptr);
	    *ep1fifo = *ep1ptr++;		// Load data into FIFO
	    if (!--ep1tbc)
		break;
	}
	buf_ch('|');

	USBE1CSR0bits.TXPKTRDY = 1;
}

static inline
void usb_ep2_rx()
{
	ep2rbc = USBE2CSR2bits.RXCNT;		// Endpoint 2 - Received Bytes Count

	buf_ch('|');
	ptr = (uint8_t *)&USBFIFO2;
	for (int i = 0; i < ep2rbc; i++) {
	    ep2data[i] = ptr[i&3];
	    buf_byte(ep2data[i]);
	}
	buf_ch('|');

	buf_str0("RX2\n");
	USBE2CSR1bits.RXPKTRDY = 0;		// Data has been unloaded
}


static inline
void usb_ep3_tx(void)
{
	volatile uint8_t *ep3fifo;
	ep3fifo = (uint8_t *)&USBFIFO3;		// for now 8 bits at a time.

	buf_str0("TX3\n");
	buf_ch('[');
	buf_word(ep3tbc);
	buf_ch(']');

	if (!usb_conf)
	    return;

	if (!ep3tbc)
	    return;

	if (USBE3CSR0bits.TXPKTRDY)
	    return;

	buf_ch('|');
	for (int i = 0; i < MAXPKT; i++) {
	    buf_byte(*ep3ptr);
	    *ep3fifo = *ep3ptr++;		// Load data into FIFO
	    if (!--ep3tbc)
		break;
	}
	buf_ch('|');

	USBE3CSR0bits.TXPKTRDY = 1;
}




#define	USB_EP0_TX(s) do {		\
	ep0ptr = (uint8_t *)(s);	\
	ep0tbc = *ep0ptr;		\
	} while(0)

#define	USB_EP0_TXL(s, l) do {		\
	ep0ptr = (uint8_t *)(s);	\
	ep0tbc = (l);			\
	} while(0)

#define	USB_EP3_TXL(s, l) do {		\
	ep3ptr = (uint8_t *)(s);	\
	ep3tbc = (l);			\
	} while(0)


static inline
int usb_ep0_ctrl_read(void)
{
	int ret = 0;

	buf_str0("\nCR:");
	buf_word(bmbRequest);
	buf_ch(',');
	buf_word(wValue);
	buf_ch(',');
	buf_word(wIndex);
	buf_ch(',');
	buf_word(wLength);

	switch (bmbRequest) {
	case 0x8006:	// Get Descriptor(s)
	    buf_str0("[GET_DESC]");

	    switch (ep0data[3]) {
	    case 0x00:	// Undefined
		break;

	    case 0x01:	// Device Descriptor
		buf_str0("[DEVICE]");

		USB_EP0_TXL(dev_desc, min(wLength, sizeof(dev_desc)));
		ret = 1;
		break;

	    case 0x02:	// Configuration Descriptor
		buf_str0("[CONFIG]");

		USB_EP0_TXL(conf_desc, min(wLength, TOTAL_LEN));
		ret = 1;
		break;

	    case 0x03:	// String Descriptors
		buf_str0("[STRING]");

		switch (ep0data[2]) {
		case 0x00:	// Language ID
		    USB_EP0_TX(string0);
		    ret = 1;
		    break;

		case 0x01:	// Manufacturer
		    USB_EP0_TX(string1);
		    ret = 1;
		    break;

		case 0x02:	// Product
		    USB_EP0_TX(string2);
		    ret = 1;
		    break;

		case 0x03:	// Serial
		    USB_EP0_TX(string3);
		    ret = 1;
		    break;

		case 0x04:	// Configuration
		    USB_EP0_TX(string4);
		    ret = 1;
		    break;

		case 0x05:	// Interface
		    USB_EP0_TX(string5);
		    ret = 1;
		    break;

		default:
		    break;
		}   // End of switch ep0data[2]
		break;

	    default:
		break;

	    }	// End of switch ep0data[3].
	    break;

	default:
	    buf_str0("[UNKNOWN]");
	    break;
	}   // End of switch bmbRequest.

	buf_ch(';');
	buf_word(ep0tbc);
	buf_ch('#');
	buf_hex(ret);
	buf_nl();

	return ret;
}

static inline
int usb_ep0_ctrl_write(void)
{
	int ret = 0;

	buf_str0("\nCW:");
	buf_word(bmbRequest);
	buf_ch(',');
	buf_word(wValue);
	buf_ch(',');
	buf_word(wIndex);
	buf_ch(',');
	buf_word(wLength);
	buf_ch(' ');

	switch (bmbRequest) {
	case 0x0008:	// Get Configuration
	    buf_str0("[GET_CONFIGURATION]");
	    ret = 1;
	    break;

	case 0x2120:	// CDC Set Line Coding
	    buf_str0("[CDC_SET_LINE_CODING]");
	    ret = 1;
	    break;

	default:
	    buf_str0("[UNKNOWN]");
	    break;
	}

	buf_ch('#');
	buf_hex(ret);
	buf_nl();

	return ret;
}

static inline
int usb_ep0_zero(void)
{
	int ret = 0;

	buf_str0("\nCZ:");
	buf_word(bmbRequest);
	buf_ch(',');
	buf_word(wValue);
	buf_ch(',');
	buf_word(wIndex);
	buf_ch(' ');

	switch (bmbRequest) {
	case 0x0001:	// Clear Feature
	    buf_str0("[CLEAR_FEATURE]");
	    ret = 1;
	    break;

	case 0x0003:	// Set Feature
	    buf_str0("[SET_FEATURE]");
	    ret = 1;
	    break;

	case 0x0005:	// Save Address
	    buf_str0("[SAVE_ADDRESS]");
	    dev_addr = wValue & 0xFF;
	    ret = 1;
	    break;

	case 0x0009:	// Set Configuration
	    buf_str0("[SET_CONFIGURATION]");
	    usb_conf = wValue & 0xFF;
	    ret = 1;
	    break;

	case 0x2122:	// CDC Set Control Line State
	    buf_str0("[CDC_SET_CONTROL_LINE_STATE]");
	    ret = 1;
	    break;

	default:
	    buf_str0("[UNKNOWN]");
	    break;
	}

	buf_ch('#');
	buf_hex(ret);
	buf_nl();

	return ret;
}



#define	EP0_STATE(s) do {	\
	int next = (s);		\
	buf_ch('{');		\
	buf_str0(sstr[ep0s]);	\
	buf_str0("->");		\
	buf_str0(sstr[next]);	\
	buf_ch('}');		\
	ep0s = next;		\
	} while (0)


static inline
int usb_ep0_tx(void)
{
	volatile uint8_t *ep0fifo;
	ep0fifo = (uint8_t *)&USBFIFO0;		// for now 8 bits at a time.

	buf_ch('[');
	buf_word(ep0tbc);
	buf_ch(']');

	// buf_ch('|');
	for (int i = 0; i < MAXPKT; i++) {
	    // buf_byte(*ep0ptr);
	    *ep0fifo = *ep0ptr++;		// Load data into FIFO
	    if (!--ep0tbc)
		break;
	}
	// buf_ch('|');

	if (ep0tbc) {			// more to come
	    buf_ch('+');
	    USBE0CSR0bits.TXRDY = 1;
	    return S_TXPKT;
	
	} else {			// last packet
	    buf_ch('/');
	    USBE0CSR0 |= _USBE0CSR0_TXRDY_MASK | _USBE0CSR0_DATAEND_MASK;
		
	    // USBE0CSR0bits.TXRDY = 1;
	    // USBE0CSR0bits.DATAEND = 1;
	    return S_STATUS;
	}
}

static inline
void usb_ep0_addr(void)
{
	if (dev_addr && !USBCSR0bits.FUNC) {
	    buf_str0("ADDR=");
	    buf_word(dev_addr);
	    buf_nl();
	
	    USBCSR0bits.FUNC = dev_addr & 0x7F;
	}
}

static inline
void usb_ep0_stall(void)
{
	buf_str0("STALL");
	USBE0CSR0bits.STALL = 1;
}

static inline
void usb_ep0_flush(void)
{
	buf_str0("FLUSH");
	USBE0CSR0bits.FLUSH = 1;
}

static inline
void usb_ep0(void)
{
	buf_ch('(');
	buf_word(USBE0CSR0 >> 16);
	buf_ch(')');

	if (ep0s == S_RESET)
	    EP0_STATE(S_IDLE);

	if (USBE0CSR0bits.SETEND) {
	    USBE0CSR0bits.SETENDC = 1;
	    buf_str0("SETEND-");
	    if (ep0s != S_ZERO)
		EP0_STATE(S_IDLE);
	}

	if (USBE0CSR0bits.STALLED) {
	    buf_str0("STALLED");
	    USBE0CSR0bits.STALLED = 0;
	    return;
	}

	switch (ep0s) {
	case S_TXPKT:				// Transmit Packet
	    EP0_STATE(usb_ep0_tx());
	    return;

	case S_ZERO:	// Zero Status
	case S_STATUS:	// Status
	    usb_ep0_addr();
	    EP0_STATE(S_IDLE);
	    return;
	}

	if (!USBE0CSR0bits.RXRDY) {
	    buf_ch('!');
	    return;
	}

	ep0rbc = USBE0CSR2bits.RXCNT;		// Received Bytes Count

	if (ep0rbc > MAXPKT) {
	    usb_ep0_stall();
	    EP0_STATE(S_IDLE);
	    return;
	}

	buf_ch('|');
	ptr = (uint8_t *)&USBFIFO0;
	for (int i = 0; i < ep0rbc; i++) {
	    ep0data[i] = ptr[i&3];
	    buf_byte(ep0data[i]);
	}
	buf_ch('|');

	switch (ep0s) {
	case S_IDLE:	// Idle State
	    if (ep0rbc != 8) {
		usb_ep0_flush();
		return;
	    }

	    bmbRequest = (ep0data[0] << 8) | ep0data[1];
	    wValue = (ep0data[3] << 8) | ep0data[2];
	    wIndex = (ep0data[5] << 8) | ep0data[4];
	    wLength = (ep0data[7] << 8) | ep0data[6];

	    if (wLength == 0) {
		if (usb_ep0_zero()) {
		    USBE0CSR0 |= _USBE0CSR0_RXRDYC_MASK | _USBE0CSR0_DATAEND_MASK;

		    // USBE0CSR0bits.RXRDYC = 1;
		    // USBE0CSR0bits.DATAEND = 1;
		    EP0_STATE(S_ZERO);
		    return;

		} else {
		    usb_ep0_stall();
		    return;
		}
	    }

	    if (bmbRequest & 0x8000) {		// control read
		if (usb_ep0_ctrl_read()) {
		    USBE0CSR0bits.RXRDYC = 1;
		    EP0_STATE(usb_ep0_tx());

		} else {
		    usb_ep0_stall();
		}

	    } else {				// control write
		USBE0CSR0bits.RXRDYC = 1;
		EP0_STATE(S_RXPKT);
	    }
	    break;

	case S_RXPKT:				// Receive Packet

	    /* TODO: handle multiple packets */

	    usb_ep0_ctrl_write();
	    USBE0CSR0 |= _USBE0CSR0_RXRDYC_MASK | _USBE0CSR0_DATAEND_MASK;
	
	    // USBE0CSR0bits.RXRDYC = 1;
	    // USBE0CSR0bits.DATAEND = 1;
	    EP0_STATE(S_IDLE);
	    break;
	}
}

static inline
void usb_reset_ep0(void)
{
	USBCSR3bits.ENDPOINT = 0;
	USBE0CSR0 = 0;
	USBFIFOAbits.RXFIFOAD = 0x00;
	USBFIFOAbits.TXFIFOAD = 0x08;
	USBOTGbits.TXFIFOSZ = 3;
	USBOTGbits.RXFIFOSZ = 3;
	USBCSR1bits.EP0IE = 1;
}

static inline
void usb_reset_ep1(void)
{
	USBE3CSR0bits.MODE = 1;	
	USBE3CSR0bits.TXMAXP = EP3TXBUFFSIZE;
	USBE3CSR2bits.PROTOCOL = 3;

	USBCSR3bits.ENDPOINT = 1;
	USBFIFOAbits.TXFIFOAD = 0x010;
	USBCSR3bits.ENDPOINT = 0;
	USBE3CSR3bits.TXFIFOSZ = 0x6;

	USBE3CSR2bits.SPEED = 1;
	USBE3CSR0bits.ISO = 0;
	USBCSR1bits.EP3TXIE = 1;
}

static inline
void usb_reset_ep2(void)
{
	USBE2CSR0bits.MODE = 0;
	USBE2CSR1bits.RXMAXP = EP2RXBUFFSIZE;
	USBE2CSR3bits.PROTOCOL = 2;

	USBCSR3bits.ENDPOINT = 2;
	USBFIFOAbits.RXFIFOAD = 0x020;
	USBCSR3bits.ENDPOINT = 0;
	USBE2CSR3bits.RXFIFOSZ = 0x9;

	USBE2CSR3bits.SPEED = 1;
	USBE2CSR1bits.ISO = 0;
	USBCSR2bits.EP2RXIE = 1;
}

static inline
void usb_reset_ep3(void)
{
	USBE3CSR0bits.MODE = 1;	
	USBE3CSR0bits.TXMAXP = EP3TXBUFFSIZE;
	USBE3CSR2bits.PROTOCOL = 2;

	USBCSR3bits.ENDPOINT = 3;
	USBFIFOAbits.TXFIFOAD = 0x030;
	USBCSR3bits.ENDPOINT = 0;
	USBE3CSR3bits.TXFIFOSZ = 0x9;

	USBE3CSR2bits.SPEED = 1;
	USBE3CSR0bits.ISO = 0;
	USBCSR1bits.EP3TXIE = 1;
}


static inline
void usb_reset(void)
{
	buf_str0("RESET");

	dev_addr = 0;
	usb_conf = 0;

	USBCSR0bits.FUNC = 0;
	usb_reset_ep0();
	usb_reset_ep1();
	usb_reset_ep2();
	usb_reset_ep3();

	EP0_STATE(S_RESET);
}

// void __attribute__((vector(_USB_VECTOR), interrupt(IPL7SRS))) usb_isr(void)
void usb_isr(void)
{
	uint32_t csr0 = USBCSR0;
	uint32_t csr1 = USBCSR1;
	uint32_t csr2 = USBCSR2;
	// IFS4CLR = _IFS4_USBIF_MASK;		// Reset the USB Interrupt flag

	buf_ch('<');
	buf_word(csr0 >> 8);
	buf_ch(':');
	buf_byte(csr1);
	buf_ch(':');
	buf_byte(csr2 >> 16);
	buf_ch('.');

	/* Endpoint 0 Interrupt Handler */
	if (csr0 & _USBCSR0_EP0IF_MASK) {
	    usb_ep0();
	}

	/* Endpoint 1 TX Interrupt Handler */
	if (csr0 & _USBCSR0_EP1TXIF_MASK) {	// Endpoint 1 Transmit.
	    usb_ep1_tx();
	}

	/* Endpoint 2 RX Interrupt Handler */
	if (csr1 & _USBCSR1_EP2RXIF_MASK) {	// Endpoint 2 Receive
	    usb_ep2_rx();
	    buf_ch('*');
	    buf_word(ep2rbc);
	    buf_ch('*');
	    /* for (int i=0; i<ep2rbc; i++)
		loop_ch(ep2data[i]); */
	    loop_ch(ep2data[0]);
	}

	/* Endpoint 3 TX Interrupt Handler */
	if (csr0 & _USBCSR0_EP3TXIF_MASK) {	// Endpoint 3 Transmit
	    usb_ep3_tx();
	}

	if (csr2 & _USBCSR2_RESETIF_MASK) {	// Bus Reset
	    usb_reset();
	}

	buf_ch('>');
	buf_nl();

	IFS4CLR = _IFS4_USBIF_MASK;		// Reset the USB Interrupt flag

	asm volatile("eret");
}


// void __attribute__((vector(_UART2_RX_VECTOR), interrupt(IPL6SRS))) uart2_isr(void)
void uart2_isr(void)
{
	while (U2STAbits.URXDA) {	// process buffer
	    char ch = U2RXREG;

	    uart2_ch(ch);	// echo back
	}

	IFS4CLR = _IFS4_U2RXIF_MASK;	// clear UART2 Rx IRQ

	asm volatile("eret");
}

static inline
void	unlock(void)
{
	SYSKEY = 0xAA996655;
	SYSKEY = 0x556699AA;
}

static inline
void	lock(void)
{
	SYSKEY = 0x33333333;
}

static inline
void	irq_disable(void)
{
	asm volatile("di");
	asm volatile("ehb");
}

static inline
void	irq_enable(void)
{
	asm volatile("ei");
}


void	init_pbus(void)
{
	unlock();
	PB2DIVbits.PBDIV = 0b000000;	// divide by 1
	PB2DIVbits.ON = 1;

	PB7DIVbits.PBDIV = 0b000000;	// divide by 1
	PB7DIVbits.ON = 1;
	lock();
}

void	init_prefetch(void)
{
	PRECONbits.PREFEN = 0b10;	// instruction and data
	PRECONbits.PFMWS = 0b010;	// two wait states

	// cacheable, non-coherent, write-back, alloc
    // __builtin_mtc0(_CP0_CONFIG, 0, 0b011);
    asm volatile("mtc0 %0, $16, 0" :: "r" (0b011));
}

void	init_uart2(void)
{
	U2MODEbits.ON = 0;

	TRISEbits.TRISE8 = 0;		// U2TX out
	TRISEbits.TRISE9 = 1;		// U2RX in
	ANSELEbits.ANSE8 = 0;		// digital
	ANSELEbits.ANSE9 = 0;		// digital

	CFGCONbits.IOLOCK = 0;
	RPE8Rbits.RPE8R = 0b0010;	// U2TX
	U2RXRbits.U2RXR = 0b1101;	// RPE9
	CFGCONbits.IOLOCK = 1;

	IPC36bits.U2TXIP = 6;		// Interrupt priority of 7
	IPC36bits.U2TXIS = 0;		// Interrupt sub-priority of 0
	IPC36bits.U2RXIP = 6;		// Interrupt priority of 7
	IPC36bits.U2RXIS = 0;		// Interrupt sub-priority of 0

	IEC4SET = _IEC4_U2RXIE_MASK;	// Rx INT Enable
	IFS4bits.U2TXIF = 0;		// Clear Tx flag
	IFS4bits.U2RXIF = 0;		// Clear Rx flag

	// U2BRG = 47;			// 1MBaud @ 192MHz
	// U2BRG = 23;			// 2MBaud @ 192MHz
	// U2BRG = 15;			// 3MBaud @ 192MHz
	U2BRG = 11;			// 4MBaud @ 192MHz
	// U2BRG = 7;			// 6MBaud @ 192MHz
	// U2BRG = 5;			// 8MBaud @ 192MHz
	// U2BRG = 3;			// 12MBaud @ 192MHz
	U2STA = 0;
	
	U2MODEbits.BRGH = 1;
	U2MODEbits.PDSEL = 0b00;
	U2MODEbits.STSEL = 0;
	U2MODEbits.UEN = 0b00;
	U2MODEbits.ON = 1;
	U2STASET = 0x9400;		// Enable Transmit and Receive
}

void	init_usb(void)
{
	USBCSR0bits.SOFTCONN = 0;	// USB D+/D- lines are disabled and tristated

	USBCSR2bits.RESETIE = 1;	// ?? When enabled IN packets/Set Address aren't received (unknown reason).
	USBCSR2bits.RESUMEIE = 0;	// ?? When enabled IN packets/Set Address aren't received (unknown reason).

	USBCSR0bits.EP0IF = 0;		// clear interrupt
	USBCSR1bits.EP1RXIF = 0;	// clear interrupt
	USBCSR0bits.EP2TXIF = 0;	// clear interrupt
	USBCSR1bits.EP3RXIF = 0;	// clear interrupt

	IPC33bits.USBIP = 7;		// USB Interrupt Priority.
	IPC33bits.USBIS = 1;		// USB Interrupt Sub-Priority.

	USBCSR0bits.FUNC = 0;		// USB address = 0

	IFS4bits.USBIF = 0;		// Clear the USB interrupt flag.
	IEC4bits.USBIE = 1;		// Enable the USB interrupt.

	USBCSR0bits.HSEN = 1;		// 1 = Enable High Speed (480Mbps) USB mode.
	USBCSR0bits.SOFTCONN = 1;	// USB D+/D- lines are enabled and active
}




static inline
void	delay(unsigned cnt)
{
	unsigned i;
	while (cnt--)
	    for (i=0; i<100; i++);
}

int	main(void)
{
	irq_disable();

	OSC_EN_O = 1;
	OSC_EN_T = 0;

	init_prefetch();
	init_pbus();
	init_uart2();
    init_usb();

	INTCONbits.MVEC = 1;		// Multi Vector Interrupts
	PRISSbits.SS0 = 0;		// Normal Register Set
	PRISSbits.PRI6SS = 6;		// Assign Shadow Register Set
	PRISSbits.PRI7SS = 7;		// Assign Shadow Register Set

	irq_enable();

	buf_str0("\r\n\nAXIOM Remote USB Demo v0.1\n");

	while (1) {
	    while (buf_ri != buf_wi)
		uart2_ch(buf[buf_ri++ & BUF_MASK]);

	    if (!ep3tbc) {
		unsigned cnt = 0;

		while ((loop_ri != loop_wi) && (cnt < 16))
		    ep3data[cnt++] = loop[loop_ri++ & LOOP_MASK];

		if (cnt) {
		    ep3tbc = cnt;
		    ep3ptr = ep3data;
		    usb_ep3_tx();
		}
	    }
	}

	return 0;
}
