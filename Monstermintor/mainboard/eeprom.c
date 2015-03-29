/*
 *  Copyright iRobotique (2011)
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


#include <avr/eeprom.h>
#include <aversive/pgmspace.h>
#include <stdio.h>

#include "eeprom.h"
#include "robot.h"

struct eeprom_config config;

#define EEPROM_ADDR_MAGIC	((void *)0x00)
#define EEPROM_ADDR_CONFIG	((void *)0x10)

#define EEPROM_MAGIC		0xdeadbeef

void eeprom_read_config(void)
{
	uint32_t magic;

	magic = eeprom_read_dword(EEPROM_ADDR_MAGIC);
	 //if(magic != EEPROM_MAGIC) {
		/* If the magic value isn't present, set default parameters. */
		//printf_P(PSTR("eprom: invalid magic value, using defaults\r\n"));
		//printf("eprom: invalid magic value, using defaults\n");
		config.encoders.track_mm = ENC_TRACK_MM;
		config.encoders.pulses_per_mm = ENC_PULSE_PER_MM;
		config.encoders.left_gain = ENC_PULSE_GAIN_LEFT;
		config.encoders.right_gain = ENC_PULSE_GAIN_RIGHT;
		return;
	//}

	eeprom_read_block(&config, EEPROM_ADDR_CONFIG, sizeof config);
}

void eeprom_write_config(void)
{
	eeprom_write_dword(EEPROM_ADDR_MAGIC, EEPROM_MAGIC);
	eeprom_write_block(&config, EEPROM_ADDR_CONFIG, sizeof config);
}
