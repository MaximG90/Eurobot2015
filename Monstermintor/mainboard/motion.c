/*
 *  Copyright Droids Corporation, Microb Technology (2009)
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
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <aversive/pgmspace.h>
#include <aversive/wait.h>
#include <aversive/error.h>

#include <clock_time.h>
#include <pid.h>
#include <control_system_manager.h>
#include <encoders_hctl.h>
#include <position_manager.h>
#include <trajectory_manager.h>
#include <vect_base.h>
#include <polygon.h>

#include "eeprom.h"
#include "main.h"
#include "motion.h"
#include "obstacle.h"
#include "robot.h"
#include "strat.h"

/* true if we want to interrupt a trajectory */
static uint8_t traj_intr = 0;

/* asked speed */
static volatile uint16_t strat_speed_a = SPEED_DIST_MIDDLE_FAST;
static volatile uint16_t strat_speed_d = SPEED_ANGLE_MIDDLE_FAST;

/* Strings that match the end traj cause */
/* /!\ keep it sync with motion.h */
static const char *err_tab[] = {
	"END_TRAJ",
	"END_BLOCKING",
	"END_NEAR",
	"END_OBSTACLE",
	"END_ERROR",
	"END_INTR",
	"END_TIMER",
	"END_GLASS",
};

/* return string from end traj type num */
const char *motion_get_err(uint8_t err)
{
	uint8_t i;
	if (err == 0)
		return "SUCCESS";
	for (i = 0; i < 8; i++) {
		if (err & (1 << i))
			return err_tab[i];
	}
	return "END_UNKNOWN";
}

/* -----------------------------------------------------------------------------
 * Calibration
 */

static int motion_calibrate_rotation(int nturns, double *gain)
{
	uint8_t why;

	position_set(&mainboard.pos, 0, 0, 0);

	/* Move forward, turn left by nturns * 360° and move backward until we
	 * hit the border.
	 */
	trajectory_d_rel(&mainboard.traj, 300);
	why = motion_wait_traj_end(MOTION_END_INTR | MOTION_END_TRAJ |
				   MOTION_END_BLOCKING);
	if (why != MOTION_END_TRAJ)
		return -1;

	motion_hardstop();
	wait_ms(500);

	trajectory_a_rel(&mainboard.traj, 360 * nturns);
	why = motion_wait_traj_end(MOTION_END_INTR | MOTION_END_TRAJ |
				   MOTION_END_BLOCKING);
	if (why != MOTION_END_TRAJ)
		return -1;

	motion_hardstop();
	wait_ms(500);

	trajectory_d_rel(&mainboard.traj, -600);
	why = motion_wait_traj_end(MOTION_END_INTR | MOTION_END_TRAJ |
				   MOTION_END_BLOCKING);
	if (why != MOTION_END_BLOCKING)
		return -1;

	motion_hardstop();
	wait_ms(500);

	/* Get the position, compute and return the encoders wheels diameter. */
	*gain = (position_get_a_rad_double(&mainboard.pos) + nturns * 2 * M_PI)
	      / (nturns * 2 * M_PI);

	return 0;
}

int motion_calibration(void)
{
	int32_t left, right;
	double gain;
	uint8_t flags;
	uint8_t why;
	int ret;

	/* Stop the robot and lower the translation and rotation speeds. */
	motion_hardstop();
	motion_set_speed(100, 100);
	wait_ms(500);

	/* Move backward until we hit the border and reset encoders and
	 * position.
	 */
	trajectory_d_rel(&mainboard.traj, -1000);
	why = motion_wait_traj_end(MOTION_END_INTR | MOTION_END_TRAJ |
				   MOTION_END_BLOCKING);
	if (why != MOTION_END_BLOCKING)
		return -1;

	motion_hardstop();
	wait_ms(500);

	IRQ_LOCK(flags);
	left = encoders_hctl_get_value(LEFT_ENCODER);
	right = encoders_hctl_get_value(RIGHT_ENCODER);
	IRQ_UNLOCK(flags);

	/* Move forward until we hit the opposite border. */
	trajectory_d_rel(&mainboard.traj, AREA_Y - ROBOT_BODY_LENGTH + 1000);
	why = motion_wait_traj_end(MOTION_END_INTR | MOTION_END_TRAJ |
				   MOTION_END_BLOCKING);
	if (why != MOTION_END_BLOCKING)
		return -1;

	motion_hardstop();
	wait_ms(500);

	/* Get the encoders values, compute and set the gains. */
	left = (encoders_hctl_get_value(LEFT_ENCODER) - left) * ENC_SIGN_LEFT;
	right = (encoders_hctl_get_value(RIGHT_ENCODER) - right)* ENC_SIGN_RIGHT;
	gain = (double)right / (double)left;

	printf_P(PSTR("encoders differential gain: %f\r\n"), gain);
	config.encoders.left_gain *= gain;

	rs_set_left_ext_encoder(&mainboard.rs, encoders_hctl_get_value,
				LEFT_ENCODER, ENC_SIGN_LEFT * config.encoders.left_gain);
	rs_set_right_ext_encoder(&mainboard.rs, encoders_hctl_get_value,
				 RIGHT_ENCODER, ENC_SIGN_RIGHT * config.encoders.right_gain);

	/* Move backward until we hit the first border. */
	position_set(&mainboard.pos, 0, 0, 0);
	wait_ms(500);

	trajectory_d_rel(&mainboard.traj, -(AREA_Y - ROBOT_BODY_LENGTH + 1000));
	why = motion_wait_traj_end(MOTION_END_INTR | MOTION_END_TRAJ |
				   MOTION_END_BLOCKING);
	if (why != MOTION_END_BLOCKING)
		return -1;

	motion_hardstop();
	wait_ms(500);

	/* Get the position, compute and set the encoders wheels diameter. */
	gain = (AREA_Y - ROBOT_BODY_LENGTH)
	     / -position_get_x_double(&mainboard.pos);

	printf_P(PSTR("encoders common gain: %f\r\n"), gain);
	config.encoders.pulses_per_mm *= gain;

	position_set_physical_params(&mainboard.pos, config.encoders.track_mm,
				     config.encoders.pulses_per_mm);

	/* Calibrate rotation one one turn, and refine the calibration on ten
	 * turns.
	 */
	ret = motion_calibrate_rotation(1, &gain);
	if (ret == -1)
		return ret;

	printf_P(PSTR("track distance gain: %f\r\n"), gain);
	config.encoders.track_mm *= gain;

	position_set_physical_params(&mainboard.pos, config.encoders.track_mm,
				     config.encoders.pulses_per_mm);

	ret = motion_calibrate_rotation(10, &gain);
	if (ret == -1)
		return ret;

	printf_P(PSTR("track distance gain verification: %f\r\n"), gain);
	config.encoders.track_mm *= gain;

	position_set_physical_params(&mainboard.pos, config.encoders.track_mm,
				     config.encoders.pulses_per_mm);

	eeprom_write_config();
	return 0;
}

/* -----------------------------------------------------------------------------
 *
 */

void motion_hardstop(void)
{
	trajectory_hardstop(&mainboard.traj);
	pid_reset(&mainboard.angle.pid);
	pid_reset(&mainboard.distance.pid);
	bd_reset(&mainboard.angle.bd);
	bd_reset(&mainboard.distance.bd);

	while ((ABS(mainboard.speed_d) > 200) || (ABS(mainboard.speed_a) > 200))
		trajectory_hardstop(&mainboard.traj);

	pid_reset(&mainboard.angle.pid);
	pid_reset(&mainboard.distance.pid);
	bd_reset(&mainboard.angle.bd);
	bd_reset(&mainboard.distance.bd);
}

/* go to an x,y point without checking for obstacle or blocking. It
 * should be used for very small dist only. Return MOTION_END_TRAJ if we
 * reach destination, or MOTION_END_BLOCKING if the robot blocked more than 3
 * times. */

uint8_t motion_goto_xy_force(int16_t x, int16_t y)
{
	uint8_t i;
	int8_t err;

	for (i = 0; i < 3; i++) {
		trajectory_goto_xy_abs(&mainboard.traj, x, y);
		err = motion_wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
		if (TRAJ_SUCCESS(err))
			return MOTION_END_TRAJ;

		if (err == MOTION_END_BLOCKING) {
			time_wait_ms(500);
			motion_hardstop();
		}
	}
	return MOTION_END_BLOCKING;
}

/* reset position */
void motion_reset_pos(int16_t x, int16_t y, int16_t a)
{
	int16_t posx = position_get_x_s16(&mainboard.pos);
	int16_t posy = position_get_y_s16(&mainboard.pos);
	int16_t posa = position_get_a_deg_s16(&mainboard.pos);

	if (x == DO_NOT_SET_POS)
		x = posx;
	if (y == DO_NOT_SET_POS)
		y = posy;
	if (a == DO_NOT_SET_POS)
		a = posa;

	DEBUG(E_USER_STRAT, "reset pos (%s%s%s)",
	      x == DO_NOT_SET_POS ? "" : "x",
	      y == DO_NOT_SET_POS ? "" : "y", a == DO_NOT_SET_POS ? "" : "a");
	position_set(&mainboard.pos, x, y, a);
	DEBUG(E_USER_STRAT, "pos resetted", __FUNCTION__);
}

void motion_set_speed(uint16_t d, uint16_t a)
{
	uint8_t flags;
	IRQ_LOCK(flags);
	strat_speed_d = d;
	strat_speed_a = a;
	trajectory_set_speed(&mainboard.traj, d, a);
	IRQ_UNLOCK(flags);
}

void motion_get_speed(uint16_t *d, uint16_t *a)
{
	uint8_t flags;
	IRQ_LOCK(flags);
	*d = strat_speed_d;
	*a = strat_speed_a;
	IRQ_UNLOCK(flags);
}

void motion_interrupt_traj(void)
{
	traj_intr = 1;
}

void motion_interrupt_traj_reset(void)
{
	traj_intr = 0;
}

uint8_t motion_test_traj_end(uint8_t why)
{
	uint16_t cur_timer;
	point_t robot_pt;

	robot_pt.x = position_get_x_s16(&mainboard.pos);
	robot_pt.y = position_get_y_s16(&mainboard.pos);

	cur_timer = time_get_s();

	if ((mainboard.flags & DO_TIMER) && (why & MOTION_END_TIMER)) {
		/* end of match */
		if (cur_timer >= MATCH_TIME)
			return MOTION_END_TIMER;
	}

	/* only handled when a character 003 is received on serial link ...
	 */
	if ((why & MOTION_END_INTR) && traj_intr) {
		printf_P(PSTR("Traj interupt !!! \n\r"));
		motion_interrupt_traj_reset();
		return MOTION_END_INTR;
	}

	if ((why & MOTION_END_TRAJ) && trajectory_finished(&mainboard.traj))
		return MOTION_END_TRAJ;

	/* we are near the destination point (depends on current
	 * speed) AND the robot is in the area bounding box. */
	if (why & MOTION_END_NEAR) {
		int16_t d_near = 100;	// 10 cm

		if (mainboard.speed_d > 1000)
			d_near = 150;	// 15 cm

		if (trajectory_in_window(&mainboard.traj, d_near, RAD(5.0)) &&
		    is_in_boundingbox(&robot_pt))
			return MOTION_END_NEAR;
	}

	if (why & MOTION_END_GLASS) {
		if (trajectory_in_window(&mainboard.traj, ROBOT_GLASS_DISTANCE, RAD(5.0)) &&
		    is_in_boundingbox(&robot_pt))
			return MOTION_END_GLASS;
	}

	if ((why & MOTION_END_BLOCKING) && bd_get(&mainboard.angle.bd)) {
		motion_hardstop();
		return MOTION_END_BLOCKING;
	}

	if ((why & MOTION_END_BLOCKING) && bd_get(&mainboard.distance
.bd)) {
		motion_hardstop();
		return MOTION_END_BLOCKING;
	}

	if ((why & MOTION_END_OBSTACLE) && obstacle_will_collide()) {
		motion_hardstop();
		return MOTION_END_OBSTACLE;
	}

	return 0;
}

uint8_t __motion_wait_traj_end_debug(uint8_t why, uint16_t line)
{
	uint8_t ret;

	do {
		ret = motion_test_traj_end(why);
	} while (ret == 0);

	DEBUG(E_USER_STRAT, "Got %s at line %d", motion_get_err(ret), line);

	return ret;
}

void motion_log_position(void)
{
	NOTICE(E_USER_STRAT, "My position : x=% .8d y=% .8d a=% .8d",
	       position_get_x_s16(&mainboard.pos),
	       position_get_y_s16(&mainboard.pos),
	       position_get_a_deg_s16(&mainboard.pos));
}

void motion_print_position(void)
{
	printf_P(PSTR("position is x=%.2f y=%.2f a=%.2f\r\n"),
		 position_get_x_double(&mainboard.pos),
		 position_get_y_double(&mainboard.pos),
		 DEG(position_get_a_rad_double(&mainboard.pos)));
}
