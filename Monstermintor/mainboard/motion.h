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

#ifndef __MOTION_H__
#define __MOTION_H__

#include <aversive.h>

struct xy_point {
	int16_t x;
	int16_t y;
};

#define DEG(x)			(((double)(x)) * (180.0 / M_PI))
#define RAD(x)			(((double)(x)) * (M_PI / 180.0))

/* return values for strats and sub trajs */
#define MOTION_END_TRAJ		(1 << 0)	/* traj successful */
#define MOTION_END_BLOCKING	(1 << 1)	/* blocking during traj */
#define MOTION_END_NEAR		(1 << 2)	/* we are near destination */
#define MOTION_END_OBSTACLE	(1 << 3)	/* There is an obstacle in front of us */
#define MOTION_END_ERROR	(1 << 4)	/* Cannot do the command */
#define MOTION_END_INTR		(1 << 5)	/* interrupted by user */
#define MOTION_END_TIMER	(1 << 6)	/* we don't a lot of time */
#define MOTION_END_GLASS	(1 << 7)	/* we've reached a glass */

/* useful traj flags */
#define TRAJ_SUCCESS(f)			(f & (MOTION_END_TRAJ | MOTION_END_NEAR))
#define TRAJ_FLAGS_STD			(MOTION_END_TRAJ | MOTION_END_BLOCKING | \
					 MOTION_END_NEAR | MOTION_END_OBSTACLE | \
					 MOTION_END_INTR | MOTION_END_TIMER)
#define TRAJ_FLAGS_NO_TIMER		(MOTION_END_TRAJ | MOTION_END_BLOCKING | \
					 MOTION_END_NEAR | MOTION_END_OBSTACLE | \
					 MOTION_END_INTR)
#define TRAJ_FLAGS_NO_NEAR		(MOTION_END_TRAJ | MOTION_END_BLOCKING | \
					 MOTION_END_OBSTACLE | MOTION_END_INTR | \
					 MOTION_END_TIMER)
#define TRAJ_FLAGS_NO_NEAR_NO_TIMER	(MOTION_END_TRAJ | MOTION_END_BLOCKING | \
					 MOTION_END_OBSTACLE | MOTION_END_INTR)
#define TRAJ_FLAGS_SMALL_DIST		(MOTION_END_TRAJ | MOTION_END_BLOCKING | \
					 MOTION_END_INTR)

/* stop as fast as possible, without ramp */
void motion_hardstop(void);

#define DO_NOT_SET_POS -1000
/* Reset position. If arg == DO_NOT_SET_POS, don't update value for
 * it. */
void motion_reset_pos(int16_t x, int16_t y, int16_t a);

/* go to an x,y point without checking for obstacle or blocking. It
 * should be used for very small dist only. Return MOTION_END_TRAJ if we
 * reach destination, or MOTION_END_BLOCKING if the robot blocked more than 3
 * times. */
uint8_t motion_goto_xy_force(int16_t x, int16_t y);

/* set/get user strat speed */
void motion_set_speed(uint16_t d, uint16_t a);
void motion_get_speed(uint16_t * d, uint16_t * a);

/* when user type ctrl-c we can interrupt traj */
void motion_interrupt_traj(void);
void motion_interrupt_traj_reset(void);

/* get name of traj error with its number */
const char *motion_get_err(uint8_t err);

/* test trajectory end condition */
uint8_t motion_test_traj_end(uint8_t why);

/* loop until test_traj_end() is true */
#define motion_wait_traj_end(why) __motion_wait_traj_end_debug(why, __LINE__)
uint8_t __motion_wait_traj_end_debug(uint8_t why, uint16_t line);
void motion_log_position(void);
void motion_print_position(void);

int motion_calibration(void);

#endif /* __MOTION_H__ */
