/*
 *  Copyright Droids Corporation (2009)
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

#ifndef __ACTUATOR_H__
#define __ACTUATOR_H__

#include <aversive.h>
#include <stdbool.h>

#define ARM_POSITION_LEFT(angle)	(1023-1023UL*(angle)/300)
#define ARM_POSITION_RIGHT(angle)	(1023UL*(angle)/300)

#define GUIDE_POSITION_OPEN		ARM_POSITION_RIGHT(73)
#define GUIDE_POSITION_CLOSE		ARM_POSITION_RIGHT(160)

#define HAND_POSITION_OPEN		ARM_POSITION_LEFT(192)
#define HAND_POSITION_CLOSE		ARM_POSITION_LEFT(142)

#define WING_RETRACTED			510
#define WING_PREPARE			750
#define	WING_PUSH			650

void wings_init(void);
void wing_set_position(uint16_t position);
void second_wing_set_position(uint16_t position);

void hand_set_position(uint16_t position);
uint16_t hand_get_position(void);

void guide_set_position(uint16_t position);

void actuators_default_position(void);

void pwm_set_and_save(void *pwm, int32_t val);

#endif /* __ACTUATOR_H__ */
