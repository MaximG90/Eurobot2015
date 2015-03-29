/*
 *  Copyright Droids Corporation (2009)
 *  Olivier MATZ <zer0@droids-corp.org>
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

#ifndef __SENSOR_H__
#define __SENSOR_H__

#include <aversive.h>

/* synchronize with sensor.c */

#define ADC_BATTERY1		0
#define ADC_BATTERY2		1
#define ADC_BATTERY3		2
#define ADC_CURRENT_MOT1	3
#define ADC_CURRENT_MOT2	4
#define ADC_CURRENT_MOT3	5
#define ADC_CURRENT_MOT4	6
#define ADC_CSENSE1		7
#define ADC_CSENSE2		8
#define ADC_CSENSE3		9
#define ADC_CSENSE4		10
#define ADC_MAX			11

#define S_CAP1			0
#define S_CAP2			1
#define S_CAP3			2
#define S_CAP4			3
#define S_CAP5			4
#define S_CAP6			5
#define S_CAP7			6
#define S_CAP8			7
#define S_CAP9			8
#define S_CAP10			9
#define S_RESERVED3		10
#define S_RESERVED4		11
#define S_RESERVED5		12
#define S_RESERVED6		13
#define S_RESERVED7		14
#define S_RESERVED8		15
#define SENSOR_MAX		10

#define S_START_SWITCH		S_CAP6 // Arduino pin 48
#define S_START_COLOR		S_CAP9 // Arduino pin 45
#define S_AUTO_POS		S_CAP10 // Arduino pin 46 
//#define S_OPPONENT_NEAR	S_CAP3 // Useless
//#define S_OPPONENT_CONTACT	S_CAP4 // Useless
//#define S_OPPONENT_FAR	S_CAP5 // Useless
#define S_LIFT			S_CAP1 // Arduino pin 10
#define S_LIFT_2		S_CAP2 // Arduino pin 11


void sensor_init(void);

/* get filtered values for adc */
uint16_t sensor_get_adc(uint8_t i);
uint16_t sensor_get_adc_mv(uint8_t i);

/* get filtered values of boolean sensors */
uint16_t sensor_get_all(void);
uint8_t sensor_get(uint8_t i);

#endif /* __SENSOR_H__ */
