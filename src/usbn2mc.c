/* usbn960x.c
* Copyright (C) 2005  Benedikt Sauter
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

#include <avr/io.h>
#include "usbn2mc.h"
#include "uart.h"

// ********************************************************************
// This subroutine handles the communication with usbn9604
// ********************************************************************


// Read data from usbn96x register

void USBNInitMC(void)
{
  MCUCR |=  (1 << ISC01); // fallende flanke
   GICR |= (1 << INT0);

  USB_CTRL_DDR = 0xf8;
  //USB_CTRL_DDR = 0xff;
  //USB_CTRL_PORT |= ((PF_RD | PF_WR | PF_CS | PF_RESET) & ~(PF_A0));
  USB_CTRL_PORT |= ((PF_RD | PF_WR | PF_CS) & ~(PF_A0));
}



unsigned char USBNBurstRead(void)
{
  //unsigned char result;

  USB_CTRL_PORT ^= (PF_CS | PF_RD);
  asm("nop");              // pause for data to get to bus
  asm("nop");
  //result = USB_DATA_IN;
  USB_CTRL_PORT ^= (PF_CS | PF_RD);
  asm("nop");              // pause for data to get to bus
  return USB_DATA_IN;
  //return result;
}

unsigned char USBNRead(unsigned char Adr)
{
  USB_DATA_DDR = 0xff;        // set for output
  USB_DATA_OUT = Adr;        // load address

  USB_CTRL_PORT ^= (PF_CS | PF_WR | PF_A0);  // strobe the CS, WR, and A0 pins
  asm("nop");              // pause for data to get to bus
  USB_CTRL_PORT ^= (PF_CS | PF_WR | PF_A0);
  asm("nop");              // pause for data to get to bus
  USB_DATA_DDR = 0x00;       // set PortD for input
  asm("nop");              // pause for data to get to bus
  return (USBNBurstRead());// get data off the bus
}



// Write data to usbn96x register
void USBNWrite(unsigned char Adr, unsigned char Data)
{
  USB_DATA_OUT = Adr;        // put the address on the bus
  USB_DATA_DDR = 0xff;         // set for output
  asm("nop");              // pause for data to get to bus
  USB_CTRL_PORT ^= (PF_CS | PF_WR | PF_A0);
  asm("nop");              // pause for data to get to bus
  USB_CTRL_PORT ^= (PF_CS | PF_WR | PF_A0);
  asm("nop");              // pause for data to get to bus
  USBNBurstWrite(Data);
}


void USBNBurstWrite(unsigned char Data)
{
   USB_DATA_OUT = Data;       // put data on the bus
   asm("nop");              // pause for data to get to bus
   USB_CTRL_PORT ^= (PF_CS | PF_WR);
   asm("nop");              // pause for data to get to bus
   USB_CTRL_PORT ^= (PF_CS | PF_WR);
}



void USBNDebug(char *msg)
{
  //UARTWrite(msg);
}

