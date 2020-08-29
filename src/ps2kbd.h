#pragma once
/*
PS2KBC, a PS2 Controler implemented on the Atmel ATTINY861.
Copyright (C) 2015 Matt Harlum

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

#include <stdbool.h>

#if 0
//original
#define KB_KUP     0
#define KB_L_SHIFT 1
#define KB_L_CTRL  2
#define KB_L_ALT   3
#define KB_R_SHIFT 4
#define KB_R_CTRL  5
#define KB_R_ALT   6
#else
//fit lower 8 bit for USB
#define KB_KUP     8

#define KB_L_SHIFT 1
#define KB_L_CTRL  0
#define KB_L_ALT   2
#define KB_L_GUI   3
#define KB_R_SHIFT 5
#define KB_R_CTRL  4
#define KB_R_ALT   6
#define KB_R_GUI   7



#endif

#define KB_SCRLK 0
#define KB_NUMLK 1
#define KB_CAPSLK 2

enum ps2state {
    KEY,
    EXTKEY,
    EXTKEY2,
    PAUSE,
    COMMAND,
};

enum bufstate {
    FULL,
    EMPTY
};

enum rxtxstate {
    TX,
    RX
};

void ps2ReadInit(void);

bool ps2ReadPoll(uint8_t * modifierState, uint32_t * keycode, uint8_t * event);

void ps2SetLeds(uint8_t ledBits);
