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

#ifndef __EEPROM_H__
#define __EEPROM_H__

struct eeprom_config {
	struct {
		double track_mm;
		double pulses_per_mm;
		double left_gain;
		double right_gain;
	} encoders;
} __attribute__((__packed__));

extern struct eeprom_config config;

void eeprom_read_config(void);
void eeprom_write_config(void);

#endif /* __EEPROM_H__ */
