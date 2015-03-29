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

#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "robot.h"

// PB6 & PB7
#define BRAKE_DDR()		do { DDRB |= 0xC0;  } while(0)
// enable motors
#define BRAKE_OFF()		do { PORTB |= 0xC0; } while(0)
// disable motors
#define BRAKE_ON()		do { PORTB &= 0x3F; } while(0)

#ifndef HOST_VERSION

#define PWM_MOT1		((void *)&gen.pwm1_4A)
#define PWM_MOT2		((void *)&gen.pwm2_4B)
#define PWM_MOT3		((void *)&gen.pwm3_1A)
#define PWM_MOT4		((void *)&gen.pwm4_1B)

#else

#define PWM_MOT1		((void *)0)
#define PWM_MOT2		((void *)1)
#define PWM_MOT3		((void *)2)
#define PWM_MOT4		((void *)3)

#endif

#ifndef  INV_FRONT_REAR_OF_ROBOT
#define LEFT_PWM		PWM_MOT2
#define RIGHT_PWM		PWM_MOT1
#else
#define LEFT_PWM		PWM_MOT1
#define RIGHT_PWM		PWM_MOT2
#endif
#define MOT3_PWM		PWM_MOT3
#define MOT4_PWM		PWM_MOT4

#endif /* __MOTOR_H__ */
