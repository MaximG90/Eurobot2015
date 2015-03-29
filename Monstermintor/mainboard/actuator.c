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

#include <aversive.h>
#include <aversive/wait.h>

#include <encoders_hctl.h>
#include <pwm_ng.h>

#include "actuator.h"
#include "main.h"
#include "motor.h"
#include "robot.h"

#define ID_AX12_GUIDE 			1
#define ID_AX12_HAND 			2
#define ID_AX12_WING_LEFT		4
#define ID_AX12_WING_RIGHT		3

#define SPEED_ARM			800
#define TORQUE_ARM			1000

void pwm_set_and_save(void *pwm, int32_t val)
{
	/* we need to do the saturation here, before saving the
	 * value */
	if (val > 4095)
		val = 4095;
	if (val < -4095)
		val = -4095;

#ifdef  INV_FRONT_REAR_OF_ROBOT
	val = -val;
#endif

	if (pwm == LEFT_PWM)
		mainboard.pwm_l = val;
	else if (pwm == RIGHT_PWM)
		mainboard.pwm_r = val;

	pwm_ng_set(pwm, val);
}

/* -----------------------------------------------------------------------------
 * AX12
 */

void wings_init(void)
{
	struct AX12 *ax12 = &gen.ax12;

	AX12_set_position(ax12, ID_AX12_WING_LEFT, WING_PUSH);
	wait_ms(20);
	AX12_set_position(ax12, ID_AX12_WING_RIGHT, WING_PUSH);
	wait_ms(200);
	AX12_set_position(ax12, ID_AX12_WING_LEFT, WING_RETRACTED);
	wait_ms(20);
	AX12_set_position(ax12, ID_AX12_WING_RIGHT, WING_RETRACTED);
}

void wing_set_position(uint16_t position)
{
	struct AX12 *ax12 = &gen.ax12;
	uint8_t id = ID_AX12_WING_LEFT;

	if (mainboard.our_color == ROBOT_COLOR_ALTERNATE)
		id = ID_AX12_WING_RIGHT;
	else
		id = ID_AX12_WING_LEFT;

	AX12_set_position(ax12, id, position);
}

void second_wing_set_position(uint16_t position)
{
	struct AX12 *ax12 = &gen.ax12;
	uint8_t id = ID_AX12_WING_RIGHT;

	if (mainboard.our_color == ROBOT_COLOR_ALTERNATE)
		id = ID_AX12_WING_LEFT;
	else
		id = ID_AX12_WING_RIGHT;

	AX12_set_position(ax12, id, position);
}

uint16_t hand_get_position(void)
{
	struct AX12 *ax12 = &gen.ax12;
	uint16_t pos;
	
	AX12_get_position(ax12, ID_AX12_HAND, &pos);
	return pos;
}

void hand_set_position(uint16_t position)
{
	struct AX12 *ax12 = &gen.ax12;

	AX12_set_position3(ax12, ID_AX12_HAND, position, SPEED_ARM, TORQUE_ARM);
}

void guide_set_position(uint16_t position)
{
	struct AX12 *ax12 = &gen.ax12;

	AX12_set_position(ax12, ID_AX12_GUIDE, position);
}

void actuators_default_position(void)
{
	/* Put hand, guide and wings in the start position.
	 * FIXME: The AX12s are unreliable without delays between each
	 * action.
	 */
	hand_set_position(HAND_POSITION_CLOSE);
	wait_ms(200);
	hand_set_position(HAND_POSITION_OPEN);
	wait_ms(200);

	guide_set_position(GUIDE_POSITION_CLOSE);
	wait_ms(200);

	wings_init();
}
