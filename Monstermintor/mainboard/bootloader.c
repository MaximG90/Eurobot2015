/*
 *  Copyright Droids Corporation
 *  Olivier Matz <zer0@droids-corp.org>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <stdio.h>

#include <aversive.h>
#include <aversive/pgmspace.h>

#include "motor.h"

/* 0 means "programmed"
 * ---- with 16 Mhz quartz
 * CKSEL 3-0 : 0111
 * SUT 1-0 : 10
 * CKDIV8 : 1
 * ---- bootloader
 * BOOTZ 1-0 : 01 (4K bootloader)
 * BOOTRST : 0 (reset on bootloader)
 * ---- jtag
 * jtagen : 0
 */

/********* fuses section *************/
//
//__fuse_t __fuse __attribute__((section (".fuse"))) =
//{
//  .low = 0xD0,
//  .high = 0x91,//(  FUSE_EESAVE & FUSE_SPIEN & FUSE_JTAGEN ),
//  .extended = 0xFC, // Brow out 4,3v.
//};

/*************************************/
void bootloader(void)
{
#define BOOTLOADER_ADDR 0x3f000
	if (pgm_read_byte_far(BOOTLOADER_ADDR) == 0xff) {
		printf_P(PSTR("Bootloader is not present\r\n"));
		return;
	}
	cli();
	BRAKE_ON();

	/* ... very specific :( */
	TIMSK0 = 0;
	TIMSK1 = 0;
	TIMSK2 = 0;
	TIMSK3 = 0;
	TIMSK4 = 0;
	TIMSK5 = 0;
	EIMSK = 0;
	UCSR0B = 0;
	UCSR1B = 0;
	UCSR2B = 0;
	UCSR3B = 0;
	SPCR = 0;
	TWCR = 0;
	ACSR = 0;
	ADCSRA = 0;
	EIND = 1;
	__asm__ __volatile__("ldi r31,0xf8\n");
	__asm__ __volatile__("ldi r30,0x00\n");
	__asm__ __volatile__("eijmp\n");

	/* never returns */
}
