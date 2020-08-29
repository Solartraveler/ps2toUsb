/*
PS2KBC, a PS2 Controler implemented on the Atmel ATTINY861.
Copyright (C) 2015 Matt Harlum

Heavily modified by Malte Marwedel (c) 2020 for a Atmega32

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
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <stdbool.h>

#include "main.h"
#include <util/delay.h>

#include "ps2kbd.h"

//Int 0, 1 or 2 can be used
#define PS2INTVECT INT2_vect
#define PS2INTENABLE INT2
#define PS2INTFLAG INTF2

//must have interrupt 0..2
#define PS2CLOCK PB2
#define PS2DATA PB1
#define PS2PORT PORTB
#define PS2DDR  DDRB
#define PS2PIN  PINB

// Volatile, declared here because they're used in and out of the ISR
volatile uint8_t rcv_byte = 0;
volatile uint8_t rcv_bitcount = 0;
volatile uint8_t send_bitcount = 0;
volatile uint8_t ssp = 0; // 0 = Start/ 1 = stop/ 2 = parity
volatile uint8_t send_parity = 0;
volatile uint8_t send_byte = 0;
volatile uint8_t parity_errors = 0; // Currently unused but will provide error info to host computer
volatile uint8_t framing_errors = 0;
volatile uint8_t g_requestResend; //set to 1, if a parity error occured
volatile uint8_t g_NextByteIsAct; //only used within the int routine
volatile enum ps2state mode = KEY;
volatile enum rxtxstate sr = RX;

#define BUFFERENTRIES 32

//by definition, no zeros are filled in the buffer, so zeros mean empty
volatile uint8_t g_rxbuffer[BUFFERENTRIES]; //must be an atomic writeable datatype
uint8_t g_rxbufferRead;
volatile uint8_t g_rxbufferWrite;
volatile uint8_t g_rxOverflow;
volatile uint8_t g_rxAct; //dont put act data into the common control flow

uint8_t ps2RxGet(void)
{
	uint8_t val = g_rxbuffer[g_rxbufferRead];
	if (val)
	{
		g_rxbuffer[g_rxbufferRead] = 0;
		g_rxbufferRead++;
		if (g_rxbufferRead == BUFFERENTRIES)
		{
			g_rxbufferRead = 0;
		}
	}
	return val;
}

void ps2RxPut(uint8_t data)
{
	uint8_t val = g_rxbuffer[g_rxbufferWrite];
	if (val == 0)
	{
		if (data)
		{
			g_rxbuffer[g_rxbufferWrite] = data;
			g_rxbufferWrite++;
			if (g_rxbufferWrite == BUFFERENTRIES)
			{
				g_rxbufferWrite = 0;
			}
		}
	}
	else
	{
		g_rxOverflow = 1;
	}
}

int calc_parity(unsigned parity_x)
{
  // Calculate Odd-Parity of byte needed to send PS/2 Packet
  unsigned parity_y;
  parity_y = parity_x ^ (parity_x >> 1);
  parity_y = parity_y ^ (parity_y >> 2);
  parity_y = parity_y ^ (parity_y >> 4);
  return parity_y & 1;
}

//only called within an interrupt
void framing_error(uint8_t num)
{
  // Deal with PS/2 Protocol Framing errors. delay for the rest of the packet and clear interrupts generated during the delay.
  framing_errors++;
  GICR &= ~(1 << PS2INTENABLE);
  sei();
  _delay_ms(8);
  cli();
  GIFR |= (1 << PS2INTFLAG); // Clear Interrupt flag
  GICR |= (1 << PS2INTENABLE);
}

void sendps2(uint8_t data)
{
/*  Send a PS/2 Packet.
  Begin the request by making both inputs outputs, drag clock low for at least 100us then take data low and release clock.
  the device will soon after start clocking in the data so make clk an input again and pay attention to the interrupt.
  The device will clock in 1 start bit, 8 data bits, 1 parity bit then 1 stop bit. It will then ack by taking data low on the 12th clk (though this is currently ignored) and then it will respond with an 0xFA ACK

  If this is done in the middle of a byte from the device to the hsot, this will be repeated later.
  The catch: If this was a multi byte sequence, the whole sequence is repeated, but our sequence
  interpreter statemachine cant handle this, and would end up in wrong scancodes. So in order to avoid this,
  we wait with sending until the keyboard is "idle", eg did not have sent a clock anything within the last 1ms.
  If we then interrupt the keyboard at the beginning of the first byte, we can count on the repeat logic.
 */
	if (data != 0xFF) //no need to wait if the command is a reset
	{
		uint16_t idleCycles = 0;
		uint16_t timeout = 0;
		while (idleCycles < 500) //1ms, 2us loop
		{
			if (rcv_bitcount == 0)
			{
				idleCycles++;
			}
			else
			{
				idleCycles = 0;
			}
			timeout++;
			if (timeout > 25000) //50ms
			{
				printf_P(PSTR("PS/2: Error, no idle state found\r\n"));
				return;
			}
			_delay_us(2.0);
		}
	}
	uint8_t send_tries = 3;
	do
	{
		send_bitcount = 0;
		send_byte = data;
		send_parity = calc_parity(send_byte);
		GICR &= ~(1 << PS2INTENABLE); // Disable interrupt for CLK
#if 0
		PS2PORT &= ~(1 << PS2DATA); // Set data Low
		PS2DDR &= ~(1 << PS2DATA); // Data is an input signal
		PS2PORT &= ~(1 << PS2CLOCK); // Set Clock low
		PS2DDR |= (1 << PS2CLOCK); // CLK low
		_delay_us(150);
		PS2DDR |= (1 << PS2DATA); // DATA low
		GIFR |= (1 << PS2INTFLAG);
		sr = TX;
		GICR |= (1 << PS2INTENABLE);
		PS2DDR &= ~(1 << PS2CLOCK); // Release clock and set it as an input again, clear interrupt flags and re-enable the interrupts
#else
		PS2PORT &= ~(1 << PS2CLOCK); // Set Clock low
		PS2DDR |= (1 << PS2CLOCK); // CLK low
		_delay_us(150);
		PS2PORT &= ~(1 << PS2DATA); // Set data Low
		PS2DDR |= (1 << PS2DATA); // DATA low
		_delay_us(10);
		GIFR |= (1 << PS2INTFLAG);
		sr = TX;
		GICR |= (1 << PS2INTENABLE);
		PS2DDR &= ~(1 << PS2CLOCK); // Release clock and set it as an input again, clear interrupt flags and re-enable the interrupts
#endif
		uint16_t timeout = 250;
		while (sr == TX) { // All the work for sending the data is handled inside the interrupt
			_delay_us(100); // Wait for ACK packet before proceeding
			timeout--;
		}
		if (sr == TX)
		{
			printf("Ooops\r\n");
		}
		timeout = 250;
		do {
			_delay_us(100); // Wait for ACK packet before proceeding
			timeout--;
		}	while ((g_rxAct == 0) && (timeout));
		send_tries--;
		if (g_rxAct == 1)
		{
			g_rxAct = 0;
			break;
		}
		if (g_rxAct == 0)
		{
			printf_P(PSTR("Retry sending... no act\r\n"));
		}
		if (g_rxAct == 2)
		{
			g_rxAct = 0;
			printf_P(PSTR("Retry sending... bad parity\r\n"));
		}
		wdt_reset();
	} while (send_tries); // If the response is not an ack, resend up to 3 times.
	g_rxAct = 0;
	_delay_us(150);
}

int getresponse(void) {
		mode = COMMAND;
		uint8_t scancode;
		uint16_t timeout = 7000;
		do {
			scancode = ps2RxGet();
			timeout--;
			_delay_us(100);
		} while ((scancode == 0) && (timeout));
		mode = KEY;
		return scancode;
}

void resetKbd(void) {
	printf_P(PSTR("Check PS/2 keyboard...\r\n"));
	sendps2(0xff); // reset kbd
	printf_P(PSTR("Send done\r\n"));
	uint8_t resp = getresponse();
	if (resp != 0xAA) {
		printf_P(PSTR("PS/2 Invalid response 0x%x... resetting\r\n"), resp);
		while (1) {} // Trigger WDT Reset
	}
	sendps2(0xf0); // Set Codeset
	sendps2(0x02); // Codeset 2
}

void parity_error(void)
{
	parity_errors++;
#if 0
  //this cant work in an interrupt...
  //sendps2(0xFE); // Inform the KBD of the Parity error and request a resend.
#else
	//disable sending, and request a resend command by the poll function
	PS2PORT &= ~(1 << PS2DATA); // Set data Low
	PS2DDR &= ~(1 << PS2DATA); // Data is an input signal
	PS2PORT &= ~(1 << PS2CLOCK); // Set Clock low
	PS2DDR |= (1 << PS2CLOCK); // CLK low
	g_requestResend = 1;
#endif
}

ISR(PS2INTVECT)
{
  if (sr == TX) { //Send bytes to device.
    if (send_bitcount <=7) // Data Byte
    {
      if ((send_byte >> send_bitcount) & 1) {
        PS2DDR &= ~(1 << PS2DATA); // DATA High
      }
      else
      {
        PS2DDR |= (1 << PS2DATA); // DATA Low
      }
    }
    else if (send_bitcount == 8) // Parity Bit
    {
      if (send_parity)
      {
        PS2DDR |= (1 << PS2DATA); // DATA Low
      }
      else
      {
        PS2DDR &= ~(1 << PS2DATA); // DATA High
      }
    }
    else if (send_bitcount == 9) // Stop Bit
    {
      PS2DDR &= ~(1 << PS2DATA); // DATA High
    }
    if (send_bitcount < 10)
    {
      send_bitcount++;
    }
    else
    {
      send_bitcount = 0;
      sr = RX;
      PS2DDR &= ~(1 << PS2CLOCK | 1 << PS2DATA); // Clock and Data set back to input
      rcv_bitcount = 0;
      g_NextByteIsAct = 1;
    }
  }

  else { // Receive from device
  uint8_t result = 0;

    if (PS2PIN & (1 << PS2DATA))
    {
      result = 1;
    }
    else {
      result = 0;
    }

    if (rcv_bitcount <=9)
    {
      if (rcv_bitcount >=1 && rcv_bitcount <= 8)
      {
        rcv_byte |= (result << (rcv_bitcount - 1)); //Scancode Byte
      }
      else if (rcv_bitcount == 0)
      {
        ssp = result; // Start Bit
      }
      else if (rcv_bitcount == 9)
      {
        ssp |= (result << 2); // Parity Bit
      }
      rcv_bitcount++;
    }
    else if (rcv_bitcount == 10)
    {
      ssp |= (result << 1); // Stop Bit
      if ((ssp & 0x2) != 0x02) // Check start and stop bits.
      {
        framing_error(ssp);
      }
      else if (calc_parity(rcv_byte) == (ssp >> 2))
      {
        if (g_NextByteIsAct)
        {
          g_rxAct = 2;
        }
        else
        {
          parity_error();
        }
      }
      else
      {
        if (rcv_byte == 0xFA)
        {
          g_rxAct = 1;
        }
        else
        {
          ps2RxPut(rcv_byte);
        }
      }
      rcv_bitcount = 0;
      rcv_byte = 0;
      result = 0;
      g_NextByteIsAct = 0;
    }

  }

}

void ps2ReadInit(void)
{
#if (PS2INTENABLE == INT0)
	MCUCR |= (1 << ISC01); // Interrupt on Falling Edge
#elif (PS2INTENABLE == INT1)
	MCUCR |= (1 << ISC11); // Interrupt on Falling Edge
#else
	MCUCSR &= ~(1 << ISC2); //already falling edge by default
#endif
  SFIOR |= (1 << PUD); // force disable pullups
  PS2DDR &= ~(1 << PS2CLOCK | 1 << PS2DATA); // PINB6 = PS/2 Clock, PINB5 = PS/2 Data both set as input
  GICR |= (1 << PS2INTENABLE); // Enable Interrupt on PINB2 aka INT0
  resetKbd();
}

/*Usually the host feeds back the LED states, making sure they are in sync
  with all other keyboards. But we could use local control here - might get
  strange results with a second keyboard connected.
*/
//#define LOCAL_LED_CONTROL

bool ps2ReadPoll(uint8_t * modifierState, uint32_t * keycode, uint8_t * event)
{
	static uint16_t kb_register = 0;
#ifdef LOCAL_LED_CONTROL
	static uint8_t kb_leds = 0;
#endif
	bool overflow = false;

	if (g_requestResend)
	{
		printf_P(PSTR("Resend due parity error\r\n"));
		g_requestResend = 0;
		sendps2(0xFE);
	}
	if (g_rxOverflow)
	{
		printf_P(PSTR("Warning, PS/2 buffer overflow\r\n"));
		g_rxOverflow = 0;
		overflow = 1;
	}
  uint8_t scancode = ps2RxGet();
  if (scancode)
  {
      //printf("Scancode: %x, mode %x reg %x, perr: %u\r\n", scancode, mode, kb_register, parity_errors);
      if (mode == EXTKEY2) {
        if (kb_register & (1 << KB_KUP)) //This is a keyup event
        {
          //E1 F0 14 F0 77
          if (scancode == 0x77) {
            *event = 2;
            *modifierState = kb_register;
            *keycode = 0xE11477;
            mode = KEY;
            kb_register &= ~(1 << KB_KUP);
          } else if ((scancode != 0x14) && (scancode != 0xF0)) {
            kb_register &= ~(1 << KB_KUP);
            mode = KEY;
          }
        }
        else
        {
          //E1 14 77
          if (scancode == 0x77) {
            *event = 1;
            *modifierState = kb_register;
            *keycode = 0xE11477;
            mode = KEY;
          }
          else if (scancode == 0xF0) {
            kb_register |= (1 << KB_KUP);
          }
          else if (scancode != 0x14) {
            mode = KEY;
          }
        }
      }
      else if (mode == EXTKEY) {
        if (kb_register & (1 << KB_KUP)) //This is a keyup event
        {
          switch(scancode)
          {
            case 0x14:
              kb_register &= ~(1 << KB_R_CTRL);
              break;
            case 0x11:
              kb_register &= ~(1 << KB_R_ALT);
              break;
            case 0x1F: //left GUI
              kb_register &= ~(1 << KB_L_GUI);
              mode = KEY;
              break;
            case 0x27: //right GUI
              kb_register &= ~(1 << KB_R_GUI);
              mode = KEY;
              break;
            default:
              *keycode = (0xE0U << 8U) | scancode;
              break;
          }
          kb_register &= ~(1 << KB_KUP);
          mode = KEY;
          *event = 2;
          *modifierState = kb_register;
        }
        else {
          switch(scancode)
          {
            case 0xF0: //Key up
              kb_register |= (1 << KB_KUP);
              mode = EXTKEY;
              break;
            case 0x14: //ctrl
              kb_register |= (1 << KB_R_CTRL);
              mode = KEY;
              break;
            case 0x11: //alt
              kb_register |= (1 << KB_R_ALT);
              mode = KEY;
              break;
            case 0x1F: //left GUI
              kb_register |= (1 << KB_L_GUI);
              mode = KEY;
              break;
            case 0x27: //right GUI
              kb_register |= (1 << KB_R_GUI);
              mode = KEY;
              break;
            default:
              mode = KEY;
              *keycode = (0xE0U << 8) | scancode;
              break;
          }
          if (scancode != 0xF0)
          {
            *event = 1;
            *modifierState = kb_register;
          }
        }
      }
      else if (mode == KEY)
      {
        if (kb_register & (1 << KB_KUP)) //This is a keyup event
        {
          switch(scancode)
          {
            case 0x12: // Left Shift
              kb_register &= ~(1 << KB_L_SHIFT);
              break;
            case 0x59: // Right Shift
              kb_register &= ~(1 << KB_R_SHIFT);
              break;
            case 0x14:
              kb_register &= ~(1 << KB_L_CTRL);
              break;
            case 0x11:
              kb_register &= ~(1 << KB_L_ALT);
              break;
            default:
              //printf("Up normal\r\n");
              *keycode = scancode;
              break;
          }
          kb_register &= ~(1 << KB_KUP);
          mode = KEY;
          *event = 2; //key up
          *modifierState = kb_register;
        }
        else {
          switch(scancode)
          {
            case 0xF0: //Key up
              kb_register |= (1 << KB_KUP);
              //printf("Key up next\r\n");
              break;
            case 0xE0: //Extended key sequence
              mode = EXTKEY;
              break;
            case 0xE1: //Extended key sequence for pause
              mode = EXTKEY2;
              break;
            case 0x12: // Left Shift
              kb_register |= (1 << KB_L_SHIFT);
              break;
            case 0x59: // Right Shift
              kb_register |= (1 << KB_R_SHIFT);
              break;
            case 0x14: //ctrl
              kb_register |= (1 << KB_L_CTRL);
              break;
            case 0x11: //alt
              kb_register |= (1 << KB_L_ALT);
              break;
            case 0x58: //capslock
#ifdef LOCAL_LED_CONTROL
              kb_leds ^= (1 << KB_CAPSLK);
              sendps2(0xed);
              sendps2(kb_leds & 0x07); // Set KBD Lights
#endif
              *keycode = scancode;
              break;
            case 0x77: //numlock
#ifdef LOCAL_LED_CONTROL
              kb_leds ^= (1 << KB_NUMLK);
              sendps2(0xed);
              sendps2(kb_leds & 0x07); // Set KBD Lights
#endif
              *keycode = scancode;
              break;
            case 0x7E: //scrllock
#ifdef LOCAL_LED_CONTROL
              kb_leds ^= (1 << KB_SCRLK);
              sendps2(0xed);
              sendps2(kb_leds & 0x07); // Set KBD Lights
#endif
             *keycode = scancode;
              break;
            default: // Fall through for all normal usb codes
              *keycode = scancode;
              break;
          }
          if ((scancode != 0xF0) && (scancode != 0xE0) && (scancode != 0xE1))
          {
            *event = 1; //key down
            *modifierState = kb_register;
          }
        }
      }
    }
	return overflow;
}

void ps2SetLeds(uint8_t ledBits) {
	sendps2(0xed);
	sendps2(ledBits);
}

