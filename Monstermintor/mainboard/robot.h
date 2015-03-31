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

#ifndef __ROBOT_H__
#define __ROBOT_H__

#include <math.h>

/**** invert easly the front and the rear of the robot ****/
//#define INV_FRONT_REAR_OF_ROBOT

/*
 * Robot dimensions
 *
 *   .^.               .^.  ^
 *   | |               | |  |
 *   | |               | |  | arms length
 *   | |               | |  |
 * 1 o_|__           __|_o  X
 *   |    \_________/    |  |
 *   |                   |  | front length
 *   |_                 _|  |
 *   |_|===--- 2 ---===|_|  X
 *   |                   |  | back length
 *   |___________________|  v
 *    <----------------->
 *           width
 *
 */
#define ROBOT_ARMS_LENGTH	0
#define ROBOT_FRONT_LENGTH	135 
#define ROBOT_BACK_LENGTH	135
#define ROBOT_BODY_LENGTH	(ROBOT_BACK_LENGTH + ROBOT_FRONT_LENGTH)
#define ROBOT_WIDTH		330
#define ROBOT_RADIUS		230 // Distance entre le milieu de chaque roue
#define ROBOT_GLASS_DISTANCE	25 // 25 mm avant que le gobelet soit a fond dans l'ascenseur

/*
 * Encoder wheels parameters
 *
 * The public parameters are ENC_PULSE_PER_MM and ENC_TRACK_MM.
 *
 * The encoders interface counts both edges of the two channels, multiplying the
 * resolution by 4.
 */
#ifndef INV_FRONT_REAR_OF_ROBOT
#define ENC_SIGN_LEFT		-1
#define ENC_SIGN_RIGHT		1
#else
#define ENC_SIGN_LEFT		1
#define ENC_SIGN_RIGHT		-1
#endif

#define ENC_PULSE_COEFF		10
#define ENC_PULSE_GAIN_LEFT	(ENC_PULSE_COEFF*1.000)
#define ENC_PULSE_GAIN_RIGHT	(ENC_PULSE_COEFF*1.00)

#define ENC_PULSE_PER_TURN	1024 
#define ENC_WHEEL_DIAMETER_MM	36.30
#define ENC_PULSE_PER_MM	(((ENC_PULSE_PER_TURN * 4) / \
				  (ENC_WHEEL_DIAMETER_MM * M_PI)) * \
				 ENC_PULSE_COEFF)

#define ENC_TRACK_MM		319 //Distance entre les deux encodeurs

#ifndef INV_FRONT_REAR_OF_ROBOT
#define LEFT_ENCODER		((void *)1)
#define RIGHT_ENCODER		((void *)0)
#else
#define LEFT_ENCODER		((void *)0)
#define RIGHT_ENCODER		((void *)1)
#endif

#define ENCODER_LIFT		((void *)2)

#endif /* __ROBOT_H__ */
