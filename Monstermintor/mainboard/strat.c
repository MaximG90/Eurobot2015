/*
 *  Copyright Droids, Microb Technology (2009)
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

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <aversive.h>
#include <aversive/pgmspace.h>
#include <aversive/queue.h>
#include <aversive/wait.h>
#include <aversive/error.h>

#include <pwm_ng.h>
#include <clock_time.h>

#include <trajectory_manager.h>
#include <vect_base.h>
#include <polygon.h>

#include "actuator.h"
#include "cmdline.h"
#include "cs.h"
#include "leds.h"
#include "lift.h"
#include "main.h"
#include "motion.h"
#include "motor.h"
#include "obstacle.h"
#include "robot.h"
#include "sensor.h"
#include "strat.h"
#include <encoders_hctl.h>
#include <adc.h>
#include <scheduler.h>

struct strat_info strat_info = {
	.dump_enabled = 1,
	.game_started = false,
	.conf = {
		.mode = STRAT_MODE_DEFAULT,
		.options = STRAT_OPTION_DETECT_OBSTACLE
			 | STRAT_OPTION_USE_ARMS,
	},
};

uint8_t glass_state;
bool stop_trajectory;
uint16_t t_debut;


/*
 * strat_dump_conf - Display strategy configuration
 */
void strat_dump_conf(void)
{
	if (!strat_info.dump_enabled)
		return;

	printf_P(PSTR("-- strat conf --\r\n"));
	printf_P(PSTR("-- /strat conf --\r\n"));
}

/*
 * strat_dump_info - Display strategy information
 */
void strat_dump_info(const char *caller)
{
	if (!strat_info.dump_enabled)
		return;

	printf_P(PSTR("%s() dump strat info:\r\n"), caller);
}

/*
 * strat_reset_info - Reset strategy information
 *
 * Initialize strategy information in preparation for a match.
 */
void strat_reset_info(void)
{
}

/* ----------------------------------------------------------------------------
 * Utilities
 * ----------------------------------------------------------------------------
 */

static void __goto_forward_xy(struct trajectory *traj, double color_x, double color_y, uint8_t flags)
{
	microseconds t;
	uint8_t err;
	int8_t move;
	while (1)
	{
		trajectory_goto_forward_xy_abs(traj, COLOR_X(color_x), COLOR_Y(color_y));
		err = motion_wait_traj_end(flags);
		switch (err)
		{
		case MOTION_END_OBSTACLE:
			motion_hardstop();
			
			do {
				// Attendre 2 s
				t = time_get_us2();
				while (t + 2000000ULL > time_get_us2());
				// Detecter l'angle où se trouve l'adversaire
				move=detect_angle();
				// Tourner 
				if(!obstacle_will_collide()) break;
				trajectory_a_rel(traj, COLOR_A(move));
				
			} while (obstacle_will_collide());
			
			trajectory_d_rel(traj, 150);
			break;

			
		case MOTION_END_BLOCKING:
			if (color_x < 0 || color_x > AREA_X ||
			    color_y < 0 || color_y > AREA_Y)
				return;

			break;

		case MOTION_END_TRAJ:
		case MOTION_END_GLASS:
			return;
		}
	}
}

static void goto_forward_xy(struct trajectory *traj, double color_x, double color_y)
{
	__goto_forward_xy(traj, color_x, color_y, MOTION_END_TRAJ);
}

static void goto_forward_xy_glass(struct trajectory *traj, double color_x, double color_y)
{
	__goto_forward_xy(traj, color_x, OFFSET_Y(color_y), TRAJ_FLAGS_STD | MOTION_END_GLASS);
}

static void goto_backward_xy(struct trajectory *traj, double color_x, double color_y)
{
	uint8_t err;
	
	do
	{
		trajectory_goto_backward_xy_abs(traj, COLOR_X(color_x), COLOR_Y(color_y));
		err = motion_wait_traj_end(TRAJ_FLAGS_STD);
	} while (err != MOTION_END_TRAJ);
}

/* ----------------------------------------------------------------------------
 * Motion strategy
 * ----------------------------------------------------------------------------
 */
struct traj_point {
	uint16_t x;
	uint16_t y;
	unsigned int pause;
};

static struct traj_point all_glasses[] = {
	{ 900, 550, 200 }, 
	{ 1200, 550, 200 }, 
	{ 1800, 550, 600 },
	{ 2100, 550, 500 },
	{ 1950, 800, 200 },
	{ 1650, 800, 200 },
	{ 1350, 800, 200 },
	{ 1050, 800, 200 },
	{ 900, 1050, 200 },
	{ 1200, 1050, 200 },	
	{ 1800, 1050, 200 },
	{ 2100, 1050, 200 }
};

static struct traj_point first_glasses[] = {
	{ 600, 500, 200 },
	{ 1500, 1000, 200 },
	{ 2500, 1250, 200 },
	{ 300, 1250, 300 }
};

static struct traj_point second_glasses[] = {
	{ 1800, 1050, 200 },
	{ 2130, 1050, 200 },
	
	{ 1950, 800, 200 },
	{ 1650, 800, 400 },
	
	{ 1050, 800, 200 },
};

static struct traj_point third_glasses[] = {
	{ 1700, 550, 50 },
	{ 1800, 550, 600 },
	{ 2130, 550, 500 },
	{ 900, 1050, 200 },
};

static struct traj_point candles_blue[] = {
	{ 750, 1510, 300 },
	{ 1110, 1380, 400 },
	{ 1220, 1340, 200 },
	{ 1352, 1315, 200 },
	{ 1490, 1315, 200 },
	{ 1600, 1200, 200}
};

static struct traj_point candles_red[] = {
	{ 750, 1510, 200 },
	{ 1095, 1375, 200 },
	{ 1205, 1345, 200 },
	{ 1352, 1325, 200 },
	{ 1510, 1310, 200 },
	{ 1600, 1200, 200}
};

static struct traj_point candles2[] = {
	{ 1800, 550, 600 },
	{ 2100, 550, 500 },
	{ 2130, 1760, 200},
	{ 2122, 1796, 200},
	{ 2055, 1704, 200},
	{ 1990, 1606, 200},
	{ 1582, 1125, 200},
	{ 1230, 1295, 200},
	{ 1119, 1356, 200},
	{ 992, 1466, 200},
	{ 400, 1000, 200}
};

static struct traj_point home[] = {
	{ 300, 300, 0 },
	{ 300, 600, 0 },
	{ 350, 900, 0 }
};

static uint8_t home_spot;

static void goto_home(struct trajectory * traj)
{
	NOTICE(E_USER_STRAT, "[Motion Strategy] Go to home");
	
	goto_forward_xy(&mainboard.traj, home[home_spot].x, home[home_spot].y);

	lift_drop();
	while (!lift_is_stopped()) {
	}

	trajectory_d_rel(&mainboard.traj, -600);
	motion_wait_traj_end(TRAJ_FLAGS_SMALL_DIST);

	home_spot++;

	lift_start();
}

static unsigned int goto_glasses(struct trajectory * traj, unsigned int n_deb, unsigned int n_fin, struct traj_point glasses[])
{
	unsigned int i;

	for (i = n_deb; i < n_fin; i++) {
		NOTICE(E_USER_STRAT, "[Motion Strategy] Go to glass %i", i+1);

		goto_forward_xy_glass(&mainboard.traj, glasses[i].x, glasses[i].y);
		wait_ms(glasses[i].pause);

		if (lift_is_full()) {			
			return i + 1;
		}
		if (time_get_s()-t_debut > 78) {
			goto_home(&mainboard.traj);
			return i + 1;
		}
	}

	return i;
}

static void goto_candles(struct trajectory * traj)
{
	// Parametrage
	unsigned int i;
	NOTICE(E_USER_STRAT, "[Motion Strategy] Go to candles");
	motion_set_speed(SPEED_DIST_SLOW, SPEED_ANGLE_SLOW);
	
	
	// Choose the trajectory in function of the color
	struct traj_point candles[ARRAY_SIZE(candles_blue)];
	
	if (mainboard.our_color==ROBOT_COLOR_DEFAULT) { //blue
		for (uint8_t i=0; i < ARRAY_SIZE(candles_blue); i++) {
			candles[i] = candles_blue[i];
		}
	} else {
		for (uint8_t i=0; i < ARRAY_SIZE(candles_red); i++) {
			candles[i] = candles_red[i];
		}
	}
		
	
	// Positionnement
	goto_forward_xy(&mainboard.traj, candles[0].x, candles[0].y);
	wing_set_position(WING_PREPARE);
	
	// Boucle d execution
	for (i = 1; i < ARRAY_SIZE(candles)-1; i++) {
		goto_forward_xy(&mainboard.traj, candles[i].x, candles[i].y);
		
		wing_set_position(WING_PUSH);
		wait_ms(300);
		wing_set_position(WING_PREPARE);
		wait_ms(300);
		wing_set_position(WING_PUSH);
		wait_ms(300);
		wing_set_position(WING_PREPARE);
	}
	
	// Shut the wings
	goto_forward_xy(&mainboard.traj, candles[ARRAY_SIZE(candles)-1].x, candles[ARRAY_SIZE(candles)-1].y);
	wing_set_position(WING_RETRACTED);
}

static void goto_candles2(struct trajectory * traj)
{
	// Parametrage
	unsigned int i;
	NOTICE(E_USER_STRAT, "[Motion Strategy] Go to candles BIS");
	motion_set_speed(SPEED_DIST_SLOW, SPEED_ANGLE_SLOW);
	
	// First glasses
	goto_forward_xy_glass(&mainboard.traj, candles2[0].x, candles2[0].y);
	goto_forward_xy_glass(&mainboard.traj, candles2[1].x, candles2[1].y);
	
	// Positionnement
	goto_forward_xy(&mainboard.traj, candles2[2].x, candles2[2].y);
	
	trajectory_turnto_xy(&mainboard.traj, candles2[3].x, 0);
	wait_ms(1000);
	second_wing_set_position(WING_PREPARE);
	
	
	// Boucle d execution
	for (i = 3; i < ARRAY_SIZE(candles2)-1; i++) {
		goto_forward_xy(&mainboard.traj, candles2[i].x, candles2[i].y);
		
		wing_set_position(WING_PUSH);
		wait_ms(300);
		wing_set_position(WING_PREPARE);
		wait_ms(300);
		wing_set_position(WING_PUSH);
		wait_ms(300);
		wing_set_position(WING_PREPARE);
	}
	
	// Shut the wings
	goto_forward_xy(&mainboard.traj, candles2[ARRAY_SIZE(candles2)-1].x, candles2[ARRAY_SIZE(candles2)-1].y);
	second_wing_set_position(WING_RETRACTED);
}

static uint8_t strat_default(void)
{	
	unsigned int point;

	NOTICE(E_USER_STRAT, "[Default Strategy] Strategy launched!\r\n");
	
	// Set Motion speed
	motion_set_speed(SPEED_DIST_MIDDLE_FAST, SPEED_ANGLE_MIDDLE_FAST);
	
	
	// Do the first travel
	obstacle_detection_enable(true);

	for (point = 0; point < ARRAY_SIZE(first_glasses); ) {
		point = goto_glasses(&mainboard.traj, point,
				     ARRAY_SIZE(first_glasses), first_glasses);

		if (!lift_is_empty())
			goto_home(&mainboard.traj);
	}
	
	
	// Pick-up the candles
	goto_candles(&mainboard.traj);
	
	// Do the second travel
	for (point = 0; point < ARRAY_SIZE(second_glasses); ) {
		point = goto_glasses(&mainboard.traj, point,
				     ARRAY_SIZE(second_glasses), second_glasses);

		if (!lift_is_empty())
			goto_home(&mainboard.traj);
	}
	
	// Do  a last travel
	goto_forward_xy_glass(&mainboard.traj, third_glasses[0].x, third_glasses[0].y);
	wait_ms(third_glasses[0].pause);
	for (point = 1; point < ARRAY_SIZE(third_glasses); ) {
		point = goto_glasses(&mainboard.traj, point,
				     ARRAY_SIZE(third_glasses), third_glasses);

		if (!lift_is_empty())
			goto_home(&mainboard.traj);
	}
	
	// End of the game
	goto_forward_xy(&mainboard.traj, 2000, 600);
	
	motion_hardstop();

	return 0;
}

static uint8_t strat_homologation(void)
{
	obstacle_detection_enable(true);

	goto_glasses(&mainboard.traj, 0, 1, first_glasses);
	goto_home(&mainboard.traj);

	return 0;
}

/* -----------------------------------------------------------------------------
 * Auto-position
 * -----------------------------------------------------------------------------
 */

#define AUTOPOS_SPEED_FAST	400
/*
 * auto_position - Position the robot automatically to its start point
 *
 * Reset the robot position and angle by bumping into the table edges.
 */
void strat_auto_position(void)
{
	uint16_t old_spdd, old_spda;
	uint8_t err;

	motion_interrupt_traj_reset();
	motion_get_speed(&old_spdd, &old_spda);
	motion_set_speed(AUTOPOS_SPEED_FAST, AUTOPOS_SPEED_FAST);

	printf_P(PSTR("auto position : reset y axis ()\n\r"));

	/* Reset position along the Y axis. Move backward by 200mm until
	 * blocking is detected.
	 */
	motion_print_position();
	printf_P(PSTR("start calib\r\n"));
	trajectory_d_rel(&mainboard.traj, 800);
	err = motion_wait_traj_end(MOTION_END_INTR | MOTION_END_TRAJ | MOTION_END_BLOCKING);
	printf_P(PSTR("1st wall\r\n"));
	printf_P(PSTR("ok => %s\r\n"), motion_get_err(err));

	if (err == MOTION_END_INTR)
		goto done;

	motion_reset_pos(DO_NOT_SET_POS, COLOR_Y(100 + ROBOT_FRONT_LENGTH), COLOR_A(-90));
	//motion_reset_pos(DO_NOT_SET_POS, COLOR_Y(ROBOT_FRONT_LENGTH), COLOR_A(-90));
	motion_print_position();

	/* Prepare to reset position along the X axis. Move forward, rotate
	* right by 90 degrees and stop.
	*/
	printf_P(PSTR("go back\r\n"));
	trajectory_d_rel(&mainboard.traj, -(START_Y - ROBOT_FRONT_LENGTH - 145));
	//trajectory_d_rel(&mainboard.traj, -(START_Y - ROBOT_FRONT_LENGTH));
	err = motion_wait_traj_end(MOTION_END_INTR | MOTION_END_TRAJ | MOTION_END_BLOCKING);
	printf_P(PSTR("ok => %s\r\n"), motion_get_err(err));
	if (err == MOTION_END_INTR)
		goto done;

	wait_ms(200);
	printf_P(PSTR("rotation\r\n"));
	trajectory_a_rel(&mainboard.traj, COLOR_A(90)); // on se tourne en arrière face au bord
	//trajectory_a_rel(&mainboard.traj, 90);
	err = motion_wait_traj_end(MOTION_END_INTR | MOTION_END_TRAJ);
	printf_P(PSTR("ok => %s\r\n"), motion_get_err(err));
	if (err == MOTION_END_INTR)
		goto done;

	printf_P(PSTR("rotation done\r\n"));
	wait_ms(200);

	motion_hardstop();

	/* Reset position along the X axis. Move backward by 400mm until
	* blocking is detected.
	*/
	printf_P(PSTR("Calib Start calib 2d axe\r\n"));
	trajectory_d_rel(&mainboard.traj, -800); // on recule pour se caler
	//trajectory_goto_xy_rel(&mainboard.traj, -800, 0);
	err = motion_wait_traj_end(MOTION_END_INTR | MOTION_END_TRAJ | MOTION_END_BLOCKING);
	printf_P(PSTR("2d wall\r\n"));
	printf_P(PSTR("ok => %s\r\n"), motion_get_err(err));
	if (err == MOTION_END_INTR)
		goto done;

	wait_ms(200);

	motion_print_position();
	motion_reset_pos(ROBOT_BACK_LENGTH, DO_NOT_SET_POS, COLOR_A(0));
	motion_print_position();

done:
	motion_set_speed(old_spdd, old_spda);
	motion_hardstop();
	printf_P(PSTR("Calib done\r\n"));
}


/* -----------------------------------------------------------------------------
 * Strategy main entry point
 * -----------------------------------------------------------------------------
 */

/*
 * strat_preinit - Pre-initialize the strategy
 *
 * Perform all strategy initialization tasks that don't need to be run right
 * before the start of the match.
 */
static void strat_preinit(void)
{
	printf_P(PSTR("%s\n\r"), __FUNCTION__);

	time_reset();
	motion_interrupt_traj_reset();
	mainboard.flags =
	    DO_ENCODERS | DO_CS | DO_RS | DO_POS | DO_BD | DO_POWER;

	strat_info.game_started = false;

	strat_dump_conf();
	strat_dump_info(__FUNCTION__);
}


/*
 * strat_init - Initialize the strategy
 *
 * Perform all strategy initialization tasks that need to be run right before
 * the start of the match.
 */
static void strat_init(void)
{
	printf_P(PSTR("%s\n\r"), __FUNCTION__);
	strat_reset_info();

	motion_set_speed(SPEED_DIST_SLOW, SPEED_ANGLE_SLOW);
	time_reset();
	motion_interrupt_traj_reset();

	/* DO_TIMER is used in motion for MOTION_END_TIMER */
	mainboard.flags =
	    DO_ENCODERS | DO_CS | DO_RS | DO_POS | DO_BD | DO_TIMER | DO_POWER;	

	lift_start();
}

/*
 * strat_main - Strategy main loop
 *
 * This function implements the robot strategy. Unless an unrecoverable error
 * occurs, it should not return before the end of the game.
 */
static uint8_t strat_main(void)
{
	uint8_t ret;

	strat_info.game_started = true;
	dump_flags();

	switch (strat_info.conf.mode) {
	case STRAT_MODE_HOMOLOGATION:
		ret = strat_homologation();
		break;

	case STRAT_MODE_DEFAULT:
	default:
		ret = strat_default();
		break;
	}

	while (time_get_s() < MATCH_TIME) {
	}

	strat_info.game_started = false;
	return ret;
}

/*
 * strat_exit - Cleanup after running the strategy
 *
 * Stop all motion, disable the control system and set the PWM commands to 0.
 */
static void strat_exit(void)
{
	uint8_t flags;

	mainboard.flags &= ~DO_TIMER;
	motion_hardstop();
	wait_ms(1000);

	IRQ_LOCK(flags);
	mainboard.flags &= ~DO_CS;
	pwm_ng_set(LEFT_PWM, 0);
	pwm_ng_set(RIGHT_PWM, 0);
	IRQ_UNLOCK(flags);
}

/*
 * strat_start - Run the strategy
 *
 * Wait for sensor switch to be unplugged, start, run and stop the strategy.
 *
 * If the sensor switch is not plugged in at startup, wait for the user to plug
 * the switch. Alternatively, strategy can be started by pressing a key on the
 * command line interface.
 */

void strat_start(void)
{
	int8_t i;

	strat_preinit();

	/* Wait for the start switch to be plugged in */
	if (!sensor_get(S_START_SWITCH)) {
		printf_P(PSTR("No start switch, press a key or plug it\r\n"));

		while (!sensor_get(S_START_SWITCH)) {
			if (!cmdline_keypressed()) {
				wait_ms(100);
				LED4_TOGGLE();
				continue;
			}

			for (i = 3; i > 0; i--) {
				printf_P(PSTR("%d\r\n"), i);
				time_wait_ms(100);
			}
			break;
		}

	}

	/* Wait for the start switch to be unplugged */
	if (sensor_get(S_START_SWITCH)) {
		printf_P(PSTR("Ready, unplug start switch to start\r\n"));
		while (sensor_get(S_START_SWITCH)) ;
		wait_ms(250);
		t_debut = time_get_s();
	}

	strat_init();
	strat_main();
	strat_exit();

	NOTICE(E_USER_STRAT, "~~~ That's all, folks! ~~~");
}
