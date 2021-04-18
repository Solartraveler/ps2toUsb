/*
PS/2 keyboard to USB
Copyright (C) 2020-2021 Malte Marwedel

Version 1.0.0

What works:
-All but one key on my TATEL-K282 S26381-K257-L120 Siemens Nixdorf keyboard
-Sending the LED state back from the host
-Macro recorder and playback

What does not:
-The BIOS does not detect the keyboard
-The power on/off button on the keyboard does not send a scancode

Electrical connection:
This source is for a USB Prog 3.0 hardware

Connecting the PS/2 keyboard is really simple:
PortB.2: PS2CLOCK
PortB.1: PS2DATA

Note: The 2KiB RAM are mostly used. Adding a 256byte global array will result
in a stack overflow.

This software supports:
Common HID protocol
HID boot subclass (except status LEDs)
The HID protocol uses an report format which simply match the boot subclass

This software misses support:
Keyboard boot protocol

Tested systems (success):
Linux Kernel 5.5
Windows 10 2020H2




This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/


#include <stdlib.h>
#include <avr/io.h>
#include <stdint.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <inttypes.h>
#include <string.h>
#include <stdio.h>
#include <avr/wdt.h>
#include <avr/pgmspace.h>
#include <stdbool.h>

#include "main.h"

//requires main.h
#include <util/delay.h>

#include "usbn2mc.h"
#include "uart.h"
#include "usbn2mc/fifo.h"
#include "ps2kbd.h"


void interrupt_ep_send(void);

//============== DEFINES =====================

#define DBGBUFFERSIZE 512

//the buffer needs 2 bytes for each char + 2 bytes \0 termination
#define USBSTRINGLEN 50

#define STRING_PRODUCT_INDEX 1
#define STRING_MANUFACTURER_INDEX 2

#define USB_CFG_HID_REPORT_DESCRIPTOR1_LENGTH 61
#define USB_CFG_HID_REPORT_DESCRIPTOR2_LENGTH 41

#define USB_CFG_LENGTH 66

#define MAXKEYS 6

#define USBBYTES (MAXKEYS + 2)

//in ms
#define POLLINTERVAL 10

//enough for ~18 chars
#define RECORDSTEPS 40

#define INTERFACEDESCRIPTORS 1

//================ TYPEDEFS ====================

typedef struct {
	uint8_t usb[USBBYTES];
	uint8_t messageLen;
	uint32_t delayMs;
} repStep_t;

typedef struct {
	repStep_t record[RECORDSTEPS];
	uint8_t index; //replay index
	uint8_t maxIndex;
	uint32_t time;
	uint8_t mode; //1 = replay, 0 = stopped, 2 = record
	uint32_t ledTimestamp; //just for user information
	uint8_t ledState;
} macro_t;

//================== GLOBAL VARS ==================

//buffer for RS232 debug prints - stored to be send out by interrupt
char toRS232Buf[DBGBUFFERSIZE];

//manager for the RS232 debug print buffer
fifo_t toRS232FIFO;

//RS232 input buffer, only one char is stored, no FIFO - currently never read
char g_lastDebug;

//the macro record and playback data
macro_t g_Macro;

//toggle bit for USB send endpoint
int togl3=0;

//information send over to the USB host
char g_productString[USBSTRINGLEN];
//information send over to the USB host
char g_manufacturerString[USBSTRINGLEN];

//led state sent by host to the keyboard (in an interrupt)
volatile uint8_t g_LedByHost;
volatile uint8_t g_UpdateLed;

//looks interesting, but mainly a testcase for catching rare communication errors
uint8_t g_BlinkMode;

/* Device Descriptor */

unsigned char usbKeyboard[] =
{
  0x12,       // 18 length of device descriptor
  0x01,       // descriptor type = device descriptor
  0x10,0x01,  // version of usb spec. ( e.g. 1.1)
  0x00,       // device class -> use from interface descriptor
  0x00,       // device subclass
  0x00,       // protocol code
  0x08,       // deep of ep0 fifo in byte (e.g. 8)
  0x81,0x17,  // vendor id
  0x64,0x0c,  // product id
  0x00,0x01,  // revision id (e.g 1.02)
  STRING_MANUFACTURER_INDEX,  // index of manuf. string
  STRING_PRODUCT_INDEX,       // index of product string
  0x00,       // index of ser. number
  0x01        // number of configs
};

/* Configuration descriptor
Get with lsusb -vv -d 1781:
 */
unsigned char usbKeyboardConf[USB_CFG_LENGTH] =
{
  0x09,        // 9 length of this descriptor
  0x02,        // descriptor type = configuration descriptor
  USB_CFG_LENGTH, 0x00,   // total length with interfaces ... (9+(9+9+7+7) + (9+9+7))
  INTERFACEDESCRIPTORS, // number of interfaces //1. for OS, 2. for BIOS
  0x01,        // number if this config. ( arg for setconfig)
  0x00,        // string index for config
  0x80,        // attrib for this configuration ( bus powerded, | 0x20 for remote wakup support)
  100,         // power for this configuration in 2*mA (e.g. 200mA)


  //InterfaceDescriptor 2 for BIOS
  0x09,        // 9 length of this descriptor
  0x04,        // descriptor type = interface descriptor
  0x00,        // interface number
  0x01,        // alternate setting for this interface
  0x01,        // number endpoints without 0
  0x03,        // class code -> HID
  0x01,        // sub-class code 1-> boot interface subclass
  0x01,        // protocoll code 1-> keyboard, 2-> mouse
  0x00,        // string index for interface
  //  HID keyboard descriptor
  0x9,         // length of this descritpor
  0x21,        // Descriptor type
  0x10, 0x01,  // HID class specification
  0x0,         // Country code
  0x1,         // num of hid class descriptors
  0x22,        // report descriptor type
  USB_CFG_HID_REPORT_DESCRIPTOR2_LENGTH, 0x0, // length of report descriptor
  //  Endpoint Descriptor for in packets: keyboard to host
  7,           // sizeof(usbDescrEndpoint)
  5,           // descriptor type = endpoint
  0x81,        // IN endpoint number 1
  0x03,        // attrib: Interrupt endpoint
  USBBYTES, 0, // maximum packet size
  POLLINTERVAL,// in ms

  //InterfaceDescriptor 1 for common OS with endpoint for LEDs
  0x09,        // 9 length of this descriptor
  0x04,        // descriptor type = interface descriptor
  0x00,        // interface number
  0x00,        // alternate setting for this interface
  0x02,        // number endpoints without 0
  0x03,        // class code -> HID
  0x00,        // sub-class code
  0x00,        // protocoll code 1-> keyboard, 2-> mouse
  0x00,        // string index for interface
  //  HID keyboard descriptor
  0x9,         // 9 length of this descriptor
  0x21,        // Descriptor type
  0x10, 0x01,  // HID class specification
  0x0,         // Country code
  0x1,         // num of hid class descriptors
  0x22,        // report descriptor type
  USB_CFG_HID_REPORT_DESCRIPTOR1_LENGTH, 0x0,  // length of report descriptor
  //  Endpoint Descriptor for in packets: keyboard to host
  7,           // sizeof endpoint descriptor
  5,           // descriptor type = endpoint
  0x81,        // IN endpoint number 1
  0x03,        // attrib: Interrupt endpoint
  USBBYTES, 0, // maximum packet size
  POLLINTERVAL,// in ms
  //   Endpoint Descriptor for out packets: host to keyboard LED status
  7,           // sizeof endpoint descriptor
  5,           // descriptor type = endpoint
  0x02,        // OUT endpoint number 2
  0x03,        // attrib: Interrupt endpoint
  1, 0,        // maximum packet size
  POLLINTERVAL,// in ms

};


/* This struct is based on:
 * Project: hid-mouse, a very simple HID example
 * Author: Christian Starkjohann
 * Creation Date: 2008-04-07
 * Copyright: (c) 2008 by OBJECTIVE DEVELOPMENT Software GmbH
 * License: GNU GPL v2 (see License.txt), GNU GPL v3 or proprietary (CommercialLicense.txt)
 * Modified by Malte Marwedel
 * LED code from:
 * https://embeddedguruji.blogspot.com/2019/04/learning-usb-hid-in-linux-part-7.html
 */
uint8_t usbHidReportDescriptor1[USB_CFG_HID_REPORT_DESCRIPTOR1_LENGTH] = { /* USB report descriptor */
    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x09, 0x06,                    // USAGE (Keyboard)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x05, 0x07,                    //   USAGE_PAGE (Keyboard)
//The first byte - the 8 control keys
    0x19, 0xe0,                    //   USAGE_MINIMUM (Keyboard LeftControl)
    0x29, 0xe7,                    //   USAGE_MAXIMUM (Keyboard Right GUI)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //   LOGICAL_MAXIMUM (1)
    0x75, 0x01,                    //   REPORT_SIZE (1)
    0x95, 0x08,                    //   REPORT_COUNT (8)
    0x81, 0x02,                    //   INPUT (Data,Var,Abs)
//the second byte - reserved by OEM
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0x81, 0x03,                    //   INPUT (Const,Var,Abs)
//bytes 2-6 - all the other keys
    0x95, MAXKEYS,                 //   REPORT_COUNT (1) --> increase to 8 to get more than one keypress at the same time
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x25, 0xE7,                    //   LOGICAL_MAXIMUM (231)
    0x19, 0x00,                    //   USAGE_MINIMUM (Reserved (no event indicated))
    0x29, 0xE7,                    //   USAGE_MAXIMUM (Keyboard Application)
    0x81, 0x00,                    //   INPUT (Data,Ary,Abs)
//the leds as one byte input
    0x05, 0x08,                    //   USAGE_PAGE (LEDs)
    0x19, 0x1,                     //   USAGE_MINIMUM (1)
    0x29, 0x3,                     //   USAGE_MAXIMUM (3)
    0x15, 0x1,                     //   logical minimum (0)
    0x25, 0x3,                     //   logical maximum (1)
    0x75, 0x01,                    //   REPORT_SIZE (1)
    0x95, 0x03,                    //   REPORT_COUNT (3)
    0x91, 0x02,                    //   OUTPUT (Data, Var, Abs)
//5 padding bits for LED
    0x95, 0x05,                    //   REPORT_COUNT (5)
    0x91, 0x01,                    //   OUTPUT (Const, Var, Abs)
    0xc0                           // END_COLLECTION
};

uint8_t usbHidReportDescriptor2[USB_CFG_HID_REPORT_DESCRIPTOR2_LENGTH] = { /* USB report descriptor */
    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x09, 0x06,                    // USAGE (Keyboard)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x05, 0x07,                    //   USAGE_PAGE (Keyboard)
//The first byte - the 8 control keys
    0x19, 0xe0,                    //   USAGE_MINIMUM (Keyboard LeftControl)
    0x29, 0xe7,                    //   USAGE_MAXIMUM (Keyboard Right GUI)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //   LOGICAL_MAXIMUM (1)
    0x75, 0x01,                    //   REPORT_SIZE (1)
    0x95, 0x08,                    //   REPORT_COUNT (8)
    0x81, 0x02,                    //   INPUT (Data,Var,Abs)
//the second byte - reserved by OEM
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0x81, 0x03,                    //   INPUT (Const,Var,Abs)
//bytes 2-6 - all the other keys
    0x95, MAXKEYS,                 //   REPORT_COUNT (1) --> increase to 8 to get more than one keypress at the same time
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x25, 0xE7,                    //   LOGICAL_MAXIMUM (231)
    0x19, 0x00,                    //   USAGE_MINIMUM (Reserved (no event indicated))
    0x29, 0xE7,                    //   USAGE_MAXIMUM (Keyboard Application)
    0x81, 0x00,                    //   INPUT (Data,Ary,Abs)
    0xc0                           // END_COLLECTION
};

/* uart interrupt - send complete */
ISR(USART_UDRE_vect)
{
  if(toRS232FIFO.count > 0)
  {
    UDR = fifo_get_nowait(&toRS232FIFO);
  }
  else
    UCSRB &= ~(1 << UDRIE);
}

/* uart interrupt - receive complete */
ISR(USART_RXC_vect)
{
  g_lastDebug = UDR; //Read to clear
}

void KeyboardToUsb(uint8_t * data, size_t len)
{
	cli();
	//char stat1 = USBNRead(TXS1);                        // get transmitter status
	USBNWrite(TXC1,FLUSH);
	int waitcycles = 0;
	while (USBNRead(TXC1) & FLUSH)                        // get transmitter status
	{
		waitcycles++;
	}
	for (uint16_t i = 0; i < len; i++) {
		USBNWrite(TXD1,data[i]);
//		char stat0 = USBNRead(TXS1);                        // get transmitter status
//		printf("%x\r\n", stat0);
	}
	interrupt_ep_send();
	//char stat2 = USBNRead(TXS1);                        // get transmitter status
	sei();
	//printf("%x->%x, waited %u\r\n", stat1, stat2, waitcycles);
	_delay_ms(POLLINTERVAL + 2); //guess: data is ignored when send too fast in a row
}

/* interrupt signal from usb controller */

ISR(INT0_vect)
{
  USBNInterrupt();
}

volatile uint32_t g_timeMs;

ISR(TIMER1_COMPA_vect)
{
	g_timeMs++;
}

uint32_t timestampGet(void)
{
	uint32_t val;
	uint8_t sreg = SREG;
	cli();
	val = g_timeMs;
	SREG = sreg;
	return val;
}


/*************** usb class HID requests  **************/

// reponse for requests on interface
void USBNInterfaceRequests(DeviceRequest *req,EPInfo* ep)
{
	printf_P(PSTR("interface request\r\n"));
	ep->DataPid = 1; //control packets start always with the togl bit set
	/* Linux requests always exactly the size for the descriptor given in the
	   usbKeyboardConf table.
	   Windows requests this size + 64 bytes (seen for three different sizes)
	   However the commercial working keyboard then just answer with the correct
	   size, so we do it too.
	 */
	if ((req->bmRequestType == 0x81) && (req->bRequest == GET_DESCRIPTOR) &&
	    (req->wValue == 0x2200) && (req->wIndex < INTERFACEDESCRIPTORS))
	{
		//we simply guess by the size...
		if ((req->wLength == sizeof(usbHidReportDescriptor2)) || (req->wLength == (sizeof(usbHidReportDescriptor2) + 64)))
		{
			printf_P(PSTR("Alternate HID descr\r\n"));
			ep->Buf = usbHidReportDescriptor2;
			ep->Index = 0;
			ep->Size = USB_CFG_HID_REPORT_DESCRIPTOR2_LENGTH;
		}
		else if (req->wLength >= sizeof(usbHidReportDescriptor1))
		{
			//this is our common default case
			printf_P(PSTR("Default HID descr\r\n"));
			ep->Buf = usbHidReportDescriptor1;
			ep->Index = 0;
			ep->Size = USB_CFG_HID_REPORT_DESCRIPTOR1_LENGTH;
		}
	}
	else if ((req->bmRequestType == 0x1) && (req->bRequest == SET_INTERFACE) &&
		(req->wValue <= 1) && (req->wIndex < INTERFACEDESCRIPTORS))
	{
		//The notebook does this as class request, Windows as standard request -> interface request
		uint8_t reportProtocol = req->wValue;
		uint8_t setInterface = req->wIndex;
		printf_P(PSTR("Set interface(1) %u, reportProt %u\r\n"), setInterface, reportProtocol);
		if (!reportProtocol)
		{
			printf_P(PSTR("Error, boot protocol not supported - just ignoring this\r\n"));
		}
		_USBNTransmitEmtpy(ep);
	}
	else
	{
		printf_P(PSTR("%x %x %x %x %x\r\n"), req->bmRequestType, req->bRequest, req->wValue, req->wIndex, req->wLength);
	}
}

/* id need for live update of firmware */
void USBNDecodeVendorRequest(DeviceRequest *req)
{
	printf_P(PSTR("vreq %x\r\n"), req->bRequest);
}

// class requests
void USBNDecodeClassRequest(DeviceRequest *req,EPInfo* ep)
{
	if ((req->bmRequestType == 0x21) && (req->bRequest == 0xA)) {
		//Set IDLE request
		/*we should answer with a zero byte package, unfortunately, by default the callback
		 does not support this
		*/
		_USBNTransmitEmtpy(ep);
		printf_P(PSTR("Set idle req\r\n"));
	} else if ((req->bmRequestType == 0x21) && (req->bRequest == SET_CONFIGURATION) &&
	    (req->wValue == 0x200) && (req->wLength == 1)) {
		/*The host will *not* use this method for notifiying LEDs, if there is a
		  separate out endpoint. (At least under Linux).
		*/
		_USBNTransmitEmtpy(ep); //confirm the (failed) LED change...
		printf_P(PSTR("LEDs changed by EP0, how do we get byte9?\r\n"));
	} else if ((req->bmRequestType == 0x21) && (req->bRequest == SET_INTERFACE) &&
	    (req->wValue <= 1) && (req->wIndex < INTERFACEDESCRIPTORS)) {
		//The notebook does this as class request, Windows as standard request -> interface request
		uint8_t reportProtocol = req->wValue;
		uint8_t setInterface = req->wIndex;
		printf_P(PSTR("Set interface(2) %u, reportProt %u\r\n"), setInterface, reportProtocol);
		if (!reportProtocol)
		{
			printf_P(PSTR("Error, boot protocol not supported - just ignoring this\r\n"));
		}
		_USBNTransmitEmtpy(ep);
	} else {
		printf_P(PSTR("dec %x %x %x %x %x\r\n"), req->bmRequestType, req->bRequest, req->wValue, req->wIndex, req->wLength);
	}
}

// togl pid for in endpoint
void interrupt_ep_send(void)
{
	//printf("interrupt_ep_send\r\n");
	if(togl3==1) {
		togl3=0;
		USBNWrite(TXC1,TX_LAST+TX_TOGL+TX_EN);
	} else {
		togl3=1;
		USBNWrite(TXC1,TX_LAST+TX_EN); //starting with TX_TOGL, would result in the loss of the first message
	}
}

#define UNUSED(x) (void)(x)

int uart_put(char var, FILE *stream) {
	UNUSED(stream);
#if 1
	fifo_put(&toRS232FIFO, var);
	UCSRB |= (1 << UDRIE);
#else
	while (!(UCSRA & (1<<UDRE)));
	UDR = var;
#endif
	return 0;
}

static FILE mystdout = FDEV_SETUP_STREAM(uart_put, NULL, _FDEV_SETUP_WRITE);

/*************** main function  **************/



/*
See codes for PS/2, Scancode set 2:
https://www.avrfreaks.net/sites/default/files/PS2%20Keyboard.pdf
See codes for USB:
https://www.usb.org/sites/default/files/documents/hut1_12v2.pdf
page 53

returns the USB key. 0 -> ignore 0xFF -> unsupported, give warning on serial port
*/
uint8_t convertTable(uint32_t ps2code, uint8_t modifiers)
{
	switch(ps2code)
	{
		case 0x1C: return 0x4; //A
		case 0x32: return 0x5;
		case 0x21: return 0x6;
		case 0x23: return 0x7;
		case 0x24: return 0x8;
		case 0x2B: return 0x9;
		case 0x34: return 0xA;
		case 0x33: return 0xB;
		case 0x43: return 0xC;
		case 0x3B: return 0xD;
		case 0x42: return 0xE;
		case 0x4B: return 0xF;
		case 0x3A: return 0x10;
		case 0x31: return 0x11;
		case 0x44: return 0x12;
		case 0x4D: return 0x13;
		case 0x15: return 0x14;
		case 0x2D: return 0x15;
		case 0x1B: return 0x16;
		case 0x2C: return 0x17;
		case 0x3C: return 0x18;
		case 0x2A: return 0x19;
		case 0x1D: return 0x1A;
		case 0x22: return 0x1B;
		case 0x35: return 0x1C;
		case 0x1A: return 0x1D; //Z
		case 0x16: return 0x1E; //1
		case 0x1E: return 0x1F;
		case 0x26: return 0x20;
		case 0x25: return 0x21;
		case 0x2E: return 0x22;
		case 0x36: return 0x23;
		case 0x3D: return 0x24;
		case 0x3E: return 0x25;
		case 0x46: return 0x26; //9
		case 0x45: return 0x27; //0
		case 0x5A: return 0x28; //enter
		case 0x76: return 0x29; //escape
		case 0x66: return 0x2A; //backspace
		case 0x0D: return 0x2B; //tab
		case 0x29: return 0x2C; //space
		case 0x4E: return 0x2D; //-
		case 0x55: return 0x2E; //=
		case 0x54: return 0x2F; //[, german ü
		case 0x5B: return 0x30; //], german +
		case 0x5D: return 0x31; //backslash, german #
		//? 0x32 should be non US #, but there is no key on the keyboard...
		case 0x4C: return 0x33; //;, german ö
		case 0x52: return 0x34; //', german ä
		case 0x0E: return 0x35; //`  german ^
		case 0x41: return 0x36; //,, german ,
		case 0x49: return 0x37; //., german .
		case 0x4A: return 0x38; // /, german -
		case 0x58: return 0x39; //caps lock
		case 0x05: return 0x3A; //F1
		case 0x06: return 0x3B; //F2
		case 0x04: return 0x3C; //F3
		case 0x0C: return 0x3D; //F4
		case 0x03: return 0x3E; //F5
		case 0x0B: return 0x3F; //F6
		case 0x83: return 0x40; //F7
		case 0x0A: return 0x41; //F8
		case 0x01: return 0x42; //F9
		case 0x09: return 0x43; //F10
		case 0x78: return 0x44; //F11
		case 0x07: return 0x45; //F12
		case 0xE07C: return 0x46;  //print screen (part2) -> use for release, ignore on press
		case 0x84:
			if (modifiers & ((1<<KB_L_ALT) | (1<<KB_R_ALT))) {
				return 0x46; //print screen when alt or compose is pressed
			} else {
				return 0x7C; //insert word (S26381-K257-L120 only?)-> copy (cant use, same as Alt+Print)
			}
		case 0x7E: return 0x47; //scroll lock
		case 0xE11477: return 0x48; //pause
		case 0xE070: return 0x49; //insert, on S26381-K257-L120: "Zeichen einfügen"
		case 0x48: return 0x49; //SEND (S26381-K257-L120 only?) -> Keyboard insert
		case 0xE06C: return 0x4A; //home
		case 0xE07D: return 0x4B; //page up
		case 0xE071: return 0x4C; //delete
		case 0xE069: return 0x4D; //end
		case 0xE07A: return 0x4E; //page down
		case 0xE074: return 0x4F; //right arrow
		case 0xE06B: return 0x50; //left arrow
		case 0xE072: return 0x51; //down arrow
		case 0xE075: return 0x52; //up arrow
		case 0x77: return 0x53; //num lock
		case 0xE04A: return 0x54; //kp /
		case 0x7C: return 0x55; //kp *
		case 0x7B: return 0x56; //kp -
		case 0x79: return 0x57; //kp +
		case 0xE05A: return 0x58; //kp enter
		case 0x69: return 0x59; //kp 1
		case 0x72: return 0x5A; //kp 2
		case 0x7A: return 0x5B; //kp 3
		case 0x6B: return 0x5C; //kp 4
		case 0x73: return 0x5D; //kp 5
		case 0x74: return 0x5E; //kp 6
		case 0x6C: return 0x5F; //kp 7
		case 0x75: return 0x60; //kp 8
		case 0x7D: return 0x61; //kp 9
		case 0x70: return 0x62; //kp 0
		case 0x71: return 0x63; //kp .
		case 0x61: return 0x64; //german <
		case 0xE02F: return 0x65; //application - right click context menu
		//up to here, all keys are required for keyboard usage page for boot
		//0x66: Keyboard power button
		case 0x27: return 0x67; //kp =
		case 0x37: return 0x68; //F13
		case 0x3F: return 0x69; //F14
		case 0x5E: return 0x6A; //F15
		case 0x56: return 0x6B; //F16
		case 0x2F: return 0x6C; //F17
		case 0x38: return 0x6D; //F18
		case 0x53: return 0x6E; //F19
		case 0x62: return 0x6F; //F20
		case 0x5F: return 0x70; //F21
		case 0x40: return 0x71; //F22 (same code as sidata)
		case 0x28: return 0x72; //end (S26381-K257-L120 only?) -> F23
		case 0x20: return 0x73; //K3 (S26381-K257-L120 only?)-> F24
		//0x74 keyboard execute
		case 0x63: return 0x75; //help (S26381-K257-L120 only?)
		//0x76 keyboard menu
		case 0x8: return 0x77; //markier (S26381-K257-L120 only?) -> select
		//0x78 keyboard stop
		case 0x10: return 0x79; //druck 2 (S26381-K257-L120 only?)-> again
		case 0x18: return 0x7A; //druck 1 (S26381-K257-L120 only?) -> undo
		//0x7B keyboard cut
		//0x7C keyboard copy - see return of 0x46
		case 0x51: return 0x7D; //insert line (S26381-K257-L120 only?) -> paste
		case 0x5C: return 0x7E; //start (S26381-K257-L120 only?) -> find
		case 0x6F: return 0x7F; //delete char (S26381-K257-L120 only?) -> mute
		case 0x64: return 0x80; //delete line (S26381-K257-L120 only?) -> vol up
		case 0x50: return 0x81; //delete word (S26381-K257-L120 only?) -> vol down
		case 0x19: return 0xD9; //(S26381-K257-L120 only?) kp clear entry
		//unique rubberdomes at S26381-K257-L120 (TATEL-K282) without any key on it:
		case 0x17: return 0x9A; //below SIDATA -> move SIDATA cap to this position as SIDATA has the same code as F22 -> return sys request/attention
		//case 0x60: return 0; //above left arrow
		//case 0x57: return 0; //above right arrow
		//case 0x13: return 0; //above context menu key
		//case 0x49: return 0; //2x above context menu key
		//case 0x67: return 0: //between druck 2 and Zeichen
		//case 0xF: return 0: //between markier and druck 1

		default: return 0xFF; //reserved
	}
}


bool ignoreStrangePs2(uint32_t data) {
	/*strange things: We get a release event on a second key press,
	 and then a press event when we release the key 0xE0F059, ending up with a
	 not removed entry in our pressed list.
	*/
	if (data == 0xE012) { //print screen (part1), also added in some cases when left shift is hold
		return true;
	}
	if (data == 0xE059) { //if a right shift is hold and then a pos end, or cursor is pressed
		return true;
	}
	return false;
}

void macroStop(void)
{
	ps2SetLeds(g_LedByHost);
	g_Macro.mode = 0;
}

bool UsbAlternateHook(uint8_t * usbCode, uint8_t * modifiers) {
	bool incept = false;
	if (*usbCode == 0x9A) { //if SIDATA
		printf_P(PSTR("Reset...\r\n"));
		while(1); //quick reboot key
	}
	if (*usbCode == 0x73) //F24 (K3)
	{
		printf_P(PSTR("Blinkmode...\r\n"));
		g_BlinkMode = 1 - g_BlinkMode;
		if (!g_BlinkMode)
		{
			ps2SetLeds(g_LedByHost);
		}
		*usbCode = 0;
		incept = true;
	}
	if (*usbCode == 0x7E) { //if find, used for starting/stopping macro recording
		*usbCode = 0;
		*modifiers = 0;
		incept = true;
		if (g_Macro.mode == 0)
		{
			printf_P(PSTR("Start macro record\r\n"));
			g_Macro.mode = 2;
			g_Macro.index = 0;
			g_Macro.maxIndex = 0;
			g_Macro.ledState = 0;
			g_Macro.ledTimestamp = timestampGet();
		}
		else
		{
			macroStop();
			printf_P(PSTR("Stop macro record\r\n"));
		}
	} else if (*usbCode == 0x72) { //if F23, used for playback a macro
		*usbCode = 0;
		*modifiers = 0;
		incept = true;
		if (g_Macro.mode == 0)
		{
			printf_P(PSTR("Start macro playback\r\n"));
			g_Macro.mode = 1;
			g_Macro.index = 0;
			g_Macro.ledState = 0;
			g_Macro.time = timestampGet();
			g_Macro.ledTimestamp = g_Macro.time;
		}
		else
		{
			printf_P(PSTR("Macro aborted\r\n"));
			macroStop();
		}
	} else if (g_Macro.mode == 1) {
		printf_P(PSTR("Macro playback aborted\r\n"));
		macroStop(); //any key should stop a macro playback
	}
	return incept;
}

void UpdateUsbKeystate(const uint32_t * keycodes, uint8_t modifiers) {
	uint8_t usbData[USBBYTES] = {0};
	uint8_t dataBytes = 2;
	for (uint8_t i = 0; i < MAXKEYS; i++) {
		if (keycodes[i]) {
//			printf_P(PSTR("Converting 0x%x\r\n"), keycodes[i]);
			uint8_t usb = convertTable(keycodes[i], modifiers);
			bool incept = UsbAlternateHook(&usb, &modifiers);
			if ((usb) && (usb != 0xFF)) {
				usbData[dataBytes] = usb;
				dataBytes++;
			} else if ((usb) && (incept == false)) {
				printf_P(PSTR("Keycode %lu(0x%lx) unsupported\r\n"), (unsigned long)keycodes[i], (unsigned long)keycodes[i]);
			}
		}
	}
	usbData[0] = modifiers; //bit positions already proper converted in ps2kbd
	KeyboardToUsb(usbData, USBBYTES); //Linux accepts shorter answers too (dataBytes). Windows not.
#if 1
	printf_P(PSTR("To usb: "));
	for (int i = 0; i < dataBytes; i++) {
		printf_P(PSTR("%x "), usbData[i]);
	}
	printf_P(PSTR("\r\n"));
#endif
	if (g_Macro.mode == 2) { //record...
		uint8_t index = g_Macro.maxIndex;
		if ((index != 0) || (dataBytes > 2) || (usbData[0])) { //filter out the macro start key itself
			memcpy(g_Macro.record[index].usb, usbData, dataBytes);
			g_Macro.record[index].messageLen = dataBytes;
			uint32_t timestamp = timestampGet();
			if (index > 0) {
				g_Macro.record[index].delayMs = timestamp - g_Macro.time;
				g_Macro.record[index].delayMs /= 3; //speedup playback by a factor of 3.
			} else {
				g_Macro.record[index].delayMs = 0;
			}
			g_Macro.time = timestamp;
			index++;
			g_Macro.maxIndex = index;
			if (index >= RECORDSTEPS)
			{
				macroStop();
				printf_P(PSTR("Macro memory full\r\n"));
			}
		}
	}
}

static bool macroExecute(uint32_t timestamp) {

	if (g_Macro.mode) {
		if (timestamp >= g_Macro.ledTimestamp)
		{
			uint8_t ledBits = 0;
			uint8_t ledState = g_Macro.ledState;
			if (g_Macro.mode == 1) {
				switch(ledState) { //flash left to right for playback
					case 0: ledBits = 2; break;
					case 1: ledBits = 4; break;
					case 2: ledBits = 1; break;
					default: ledBits = 0;
				}
			}
			if (g_Macro.mode == 2) {
				switch(ledState) { //flash right to left for recording
					case 0: ledBits = 1; break;
					case 1: ledBits = 4; break;
					case 2: ledBits = 2; break;
					default: ledBits = 0;
				}
			}
			ps2SetLeds(ledBits);
			ledState++;
			if (ledState >= 3) {
				ledState = 0;
			}
			g_Macro.ledState = ledState;
			g_Macro.ledTimestamp += 166;
		}
	}
	if (g_Macro.mode == 1) {
		if (timestamp >= g_Macro.time) {
			uint8_t index = g_Macro.index;
			printf_P(PSTR("Macro replay %u\r\n"), index);
			KeyboardToUsb(g_Macro.record[index].usb, g_Macro.record[index].messageLen);
#if 1
			printf_P(PSTR("To usb %02i: "), index);
			for (int i = 0; i < g_Macro.record[index].messageLen; i++) {
				printf_P(PSTR("%x "), g_Macro.record[index].usb[i]);
			}
			printf_P(PSTR("\r\n"));
#endif
			uint8_t indexNext = index + 1;
			if ((indexNext < RECORDSTEPS) && (indexNext < g_Macro.maxIndex)) {
				g_Macro.index = indexNext;
				g_Macro.time += g_Macro.record[indexNext].delayMs;
			} else {
				uint8_t stopbuffer[2] = {0};
				if ((g_Macro.record[index].messageLen != 2) || g_Macro.record[index].usb[0] != 0) {
					KeyboardToUsb(stopbuffer, 2);
				}
				macroStop();
				printf_P(PSTR("Macro complete\r\n"));
			}
		}
		return true;
	}
	return false;
}

void rx1FifoCallback(char * buf, int len) {
	printf_P(PSTR("Got %u bytes\r\n"), len);
	if (len)
	{
		/* Bit mapping:
		             USB   PS/2
			 NumLock    0  -> 1
			 CapsLock   1  -> 2
			 ScrollLock 2  -> 0
		*/
		uint8_t out = 0;
		if (buf[0] & 1) out |= 2;
		if (buf[0] & 2) out |= 4;
		if (buf[0] & 4) out |= 1;
		g_LedByHost = out;
		g_UpdateLed = 1;
	}
}

int main(void) {
	wdt_enable(WDTO_1S);

	// init fifos for debug output
	fifo_init(&toRS232FIFO, toRS232Buf, DBGBUFFERSIZE);

	//baudrate stopbit parity databits, for debug only
	/*Problem: If we came from the bootloader, we run already with 16MHz,
	  but if the bootloader was not active, we run with 4MHz, so the serial
	  output will be wrong in one case. We cant print safe until the USB chip is
	  properly configured to deliver us a valid clock.*/
	uart_init(19200UL, 2, 0, 8);

	USBNInitMC(); // set up ports and interrupt on AVR side

	USBNWrite(CCONF, 0x02); // clock to 16 MHz, required for serial debug

	sei();

	stdout = &mystdout;

	printf_P(PSTR("PS/2 keyboard to USB\r\n"));

	printf_P(PSTR("(c) 2020-2021 by Malte Marwedel\r\n"));
	printf_P(PSTR("Version 1.0.0\r\n"));

	_delay_ms(10);

	cli();

	// setup usbstack with your descriptors
	USBNInit(usbKeyboard,usbKeyboardConf); //does not initialize anything, just store variable references
	//the EP1 is enabled by the incoming set configuration packet

	USBNSetString(g_manufacturerString, USBSTRINGLEN, "marwedels.de", STRING_MANUFACTURER_INDEX);
	USBNSetString(g_productString, USBSTRINGLEN, "PS/2 keyboard to USB", STRING_PRODUCT_INDEX);

	USBNCallbackFIFORX1(&rx1FifoCallback);

	sei();

	printf_P(PSTR("Start USB...\r\n"));

	USBNStart(); // start device stack, just endpoint 0 is now set up

	for (int i = 0; i < 10; i++) { //The PS/2 keyboard might take 500..750ms
		_delay_ms(100);
		wdt_reset();
	}

	printf_P(PSTR("Init PS/2...\r\n"));

	ps2ReadInit();

	//Start timer
	OCR1A = 250; //1ms tick @ 16MHz and divider 64
	TCNT1 = 0;
	TCCR1A = 0;
	TCCR1B = (1<<WGM12) | (1<<CS11) | (1<<CS10); //divide by 64, overflow at OCR1A value
	TIMSK |= (1<<OCF1A);

	printf_P(PSTR("Entering main loop...\r\n"));

	uint32_t check = 0;
	uint32_t blink = 0;
	uint32_t fallbackTimeout = 0xFFFFFFFF;
	uint8_t toggle = 0;
	uint32_t blinkTimeout = 0;
	uint8_t blinkToggle = 0;

	uint32_t keycodePressed[MAXKEYS] = {0};
	uint8_t modifierOld = 0;

	uint32_t resetEventsLast = 0;

	while(1) {
		uint32_t timestamp = timestampGet();
		uint8_t modifierNew = 0;
		uint32_t keycodeNew = 0;
		uint8_t eventNew = 0;
		bool newState = false;
		bool overflow = ps2ReadPoll(&modifierNew, &keycodeNew, &eventNew);
		if (overflow)
		{
			//emergency abort, to avoid mixig keycodes or ending up with non released keys
			memset(keycodePressed, 0, MAXKEYS);
			eventNew = 0;
			keycodeNew = 0;
			modifierNew = 0;
			newState = true;
		}
		if (eventNew == 1) { //press
			fallbackTimeout = timestamp + 60000; //the keyboard will start repeats after 500ms
			if ((keycodeNew) && (ignoreStrangePs2(keycodeNew) == false)) {
				bool updated = false;
				for (uint8_t i = 0; i < MAXKEYS; i++) {
					if (keycodePressed[i] == keycodeNew) {
						printf_P(PSTR("Update existing 0x%lx\r\n"), (unsigned long)keycodeNew);
						updated = true;
						break;
					}
				}
				if (updated == false) {
					for (uint8_t i = 0; i < MAXKEYS; i++) {
						if (keycodePressed[i] == 0) {
							printf_P(PSTR("Press 0x%x-0x%lx\r\n"), modifierNew, (unsigned long)keycodeNew);
							keycodePressed[i] = keycodeNew;
							newState = true;
							break;
						}
					}
				}
			}
			if (modifierNew != modifierOld) {
				newState = true;
			}
		}
		if (eventNew == 2) { //release
			if ((keycodeNew) && (ignoreStrangePs2(keycodeNew) == false)) {
				bool found = false;
				for (uint8_t i = 0; i < MAXKEYS; i++) {
					if (keycodePressed[i] == keycodeNew) {
						printf_P(PSTR("Release 0x%lx\r\n"), (unsigned long)keycodeNew);
						found = true;
						keycodePressed[i] = 0;
						if (g_Macro.mode != 1) { //dont intercept a replay by releasing the replay key
							newState = true;
						}
						break;
					}
				}
				if (!found) {
					printf_P(PSTR("Release 0x%x not in list!\r\n"), (unsigned long)keycodeNew);
				}
			}
			if (modifierNew != modifierOld) {
				newState = true;
			}
		}
		if (macroExecute(timestamp))
		{
			fallbackTimeout = timestamp + 1000; //the fallback should not interrupt the macro playback
		}
		if (timestamp > fallbackTimeout) {
			/* Key a dont send repeats, once another additional key has
			   been pressed and released. So no safe detection for a key up miss here...
			   This is specially important if someone opens the window overview with
			   ALT+TAB then releases tab to look through the list while holding ALT.
			   So this is limited to one minute safety timeout for decisions...
			*/
			printf_P(PSTR("Clear all keys\r\n"));
			memset(keycodePressed, 0, MAXKEYS);
			modifierNew = 0;
			newState = true;
			fallbackTimeout = 0xFFFFFFFF;
		}
		if (newState) {
			UpdateUsbKeystate(keycodePressed, modifierNew);
			modifierOld = modifierNew;
		}
		if ((g_UpdateLed) && (g_Macro.mode == 0))
		{
			cli();
			uint8_t newLedState = g_LedByHost;
			g_UpdateLed = 0;
			sei();
			ps2SetLeds(newLedState);
		}
		if (timestamp > check) {
			printf_P(PSTR("Ping\r\n"));
			check = timestamp + 3000UL;
			//printf_P(PSTR("%lu\r\n"), timestamp);
		}
		if (g_BlinkMode) {
			if (blinkTimeout < timestamp) {
				if (blinkToggle) {
					ps2SetLeds(0x7 ^ g_LedByHost);
					blinkToggle = 0;
				} else {
					ps2SetLeds(g_LedByHost);
					blinkToggle = 1;
				}
				blinkTimeout = timestamp + 50;
			}
		}
		if (timestamp > blink) {
			toggle = 1 - toggle;
			if (toggle) {
				PORTA |= (1 << PA4);
			} else {
				PORTA &= ~(1 << PA4);
			}
			blink = timestamp + 500UL;
		}
		uint32_t resetEventsNow = USBNGetResetEvents();
		if (resetEventsNow != resetEventsLast) {
			printf_P(PSTR("USB reset events: %lu\r\n"), (unsigned long)resetEventsNow);
			resetEventsLast = resetEventsNow;
		}
		wdt_reset();
	}
}
