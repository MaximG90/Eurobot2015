/*
 *  Copyright Droids Corporation
 *  Olivier Matz <zer0@droids-corp.org>
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
#include <string.h>

#include <aversive.h>
#include <aversive/error.h>

#include <encoders_hctl.h>
#include <pwm_ng.h>
#include <timer.h>
#include <scheduler.h>
#include <clock_time.h>

#include <pid.h>
#include <quadramp.h>
#include <control_system_manager.h>
#include <trajectory_manager.h>

#include "actuator.h"
#include "eeprom.h"
#include "main.h"
#include "motor.h"
#include "robot.h"
#include "strat.h"

/* called every 5 ms */
static void do_cs(void *dummy)
{
	static uint8_t cpt = 0;
	static int32_t old_a = 0, old_d = 0;



	/* XXX there is an issue which is probably related to avr-libc
	 * 1.6.2 (debian): this code using fixed_point lib does not
	 * work with it */
	/* robot system, conversion to angle,distance */
	if (mainboard.flags & DO_RS) {
		int16_t a, d;
		rs_update(&mainboard.rs);	/* takes about 0.5 ms */
		/* process and store current speed */
		a = rs_get_angle(&mainboard.rs);
		d = rs_get_distance(&mainboard.rs);
		mainboard.speed_a = a - old_a;
		mainboard.speed_d = d - old_d;
		old_a = a;
		old_d = d;
	}

	/* control system */
	if (mainboard.flags & DO_CS) {
		if (mainboard.angle.on)
			cs_manage(&mainboard.angle.cs);
		if (mainboard.distance.on)
			cs_manage(&mainboard.distance.cs);
		if (mainboard.lift.on)
			cs_manage(&mainboard.lift.cs);
	}
	if ((cpt & 1) && (mainboard.flags & DO_POS)) {
		/* about 1.5ms (worst case without centrifugal force
		 * compensation) */
		position_manage(&mainboard.pos);

	}
	if (mainboard.flags & DO_BD) {
		bd_manage_from_cs(&mainboard.angle.bd, &mainboard.angle.cs);
		bd_manage_from_cs(&mainboard.distance.bd,
				  &mainboard.distance.cs);
	}
	if (mainboard.flags & DO_TIMER) {
		uint8_t second;
		/* the robot should stop correctly in the strat, but
		 * in some cases, we must force the stop from an
		 * interrupt */
		second = time_get_s();
		/* HACK: The strategy should stop after the end of the game but
		 * doesn't. It should be fixed, in the meantime hardcode a full
		 * stop here.
		 */
		if (second >= MATCH_TIME) {
			pwm_ng_set(LEFT_PWM, 0);
			pwm_ng_set(RIGHT_PWM, 0);
			pwm_ng_set(MOT3_PWM, 0);
			pwm_ng_set(MOT4_PWM, 0);
			printf_P(PSTR("END OF TIME\r\n"));
			cli();
			while (1) ;
		}
	}

	/* brakes */
	if (mainboard.flags & DO_POWER)
		BRAKE_OFF();
	else
		BRAKE_ON();

	cpt++;
}

void dump_flags(void)
{
	if (mainboard.flags) {
		printf_P(PSTR("mainboard cs_events : "));

		if (mainboard.flags & DO_ENCODERS)
			printf_P(PSTR("DO_ENC "));
		if (mainboard.flags & DO_RS)
			printf_P(PSTR("DO_RS "));
		if (mainboard.flags & DO_CS)
			printf_P(PSTR("DO_CS "));
		if (mainboard.flags & DO_POS)
			printf_P(PSTR("DO_POS "));
		if (mainboard.flags & DO_BD)
			printf_P(PSTR("DO_BD "));
		if (mainboard.flags & DO_TIMER)
			printf_P(PSTR("DO_TIMER "));
		if (mainboard.flags & DO_POWER)
			printf_P(PSTR("DO_POWER "));
	} else
		printf_P(PSTR("mainboard.flags : No Flag Set !"));
	printf_P(PSTR("\r\n"));
}

void dump_cs_debug(const char *name, struct cs *cs)
{
	DEBUG(E_USER_CS, "%s cons=% .5ld fcons=% .5ld err=% .5ld "
	      "in=% .5ld out=% .5ld",
	      name, cs_get_consign(cs), cs_get_filtered_consign(cs),
	      cs_get_error(cs), cs_get_filtered_feedback(cs), cs_get_out(cs));
}

void dump_cs(const char *name, struct cs *cs)
{
	printf_P(PSTR("%s cons=% .5ld fcons=% .5ld err=% .5ld "
		      "in=% .5ld out=% .5ld\r\n"),
		 name, cs_get_consign(cs), cs_get_filtered_consign(cs),
		 cs_get_error(cs), cs_get_filtered_feedback(cs),
		 cs_get_out(cs));
}

void dump_cs_simple(struct cs *cs)
{
	printf_P(PSTR("%ld;%ld;%ld;%ld;%ld;"),
		 cs_get_consign(cs), cs_get_filtered_consign(cs),
		 cs_get_error(cs), cs_get_filtered_feedback(cs),
		 cs_get_out(cs));
}

void dump_pid(const char *name, struct pid_filter *pid)
{
	printf_P(PSTR("%s P=% .8ld I=% .8ld D=% .8ld out=% .8ld\r\n"),
		 name,
		 pid_get_value_in(pid) * pid_get_gain_P(pid),
		 pid_get_value_I(pid) * pid_get_gain_I(pid),
		 pid_get_value_D(pid) * pid_get_gain_D(pid),
		 pid_get_value_out(pid));
}

void monstravr_cs_init(void)
{
	/* ROBOT_SYSTEM */
	rs_init(&mainboard.rs);
	rs_set_left_pwm(&mainboard.rs, pwm_set_and_save, LEFT_PWM);
	rs_set_right_pwm(&mainboard.rs, pwm_set_and_save, RIGHT_PWM);

	rs_set_left_ext_encoder(&mainboard.rs, encoders_hctl_get_value,
				LEFT_ENCODER, ENC_SIGN_LEFT * config.encoders.left_gain);
	rs_set_right_ext_encoder(&mainboard.rs, encoders_hctl_get_value,
				 RIGHT_ENCODER, ENC_SIGN_RIGHT * config.encoders.right_gain);
	/* rs will use external encoders */
	rs_set_flags(&mainboard.rs, RS_USE_EXT);

	/* POSITION MANAGER */
	position_init(&mainboard.pos);
	position_set_physical_params(&mainboard.pos, config.encoders.track_mm,
				     config.encoders.pulses_per_mm);
	position_set_related_robot_system(&mainboard.pos, &mainboard.rs);
	//position_set_centrifugal_coef(&mainboard.pos, 0.000016); // to adjust
	position_use_ext(&mainboard.pos);

	/* TRAJECTORY MANAGER */
	trajectory_init(&mainboard.traj,mainboard.traj.cs_hz);
	trajectory_set_cs(&mainboard.traj, &mainboard.distance.cs,
			  &mainboard.angle.cs);
	trajectory_set_robot_params(&mainboard.traj, &mainboard.rs,
				    &mainboard.pos);
	trajectory_set_speed(&mainboard.traj, SPEED_DIST_MIDDLE_FAST,
			     SPEED_ANGLE_MIDDLE_FAST);	/* d, a */
	/* distance window, angle window, angle start */
	trajectory_set_windows(&mainboard.traj, 200., 5.0, 30.);

	/* ---- CS angle */
	/* PID */
	pid_init(&mainboard.angle.pid);
//	pid_set_gains(&mainboard.angle.pid, 100, 10, 500);
	pid_set_gains(&mainboard.angle.pid, 100, 0, 800);
	pid_set_maximums(&mainboard.angle.pid, 0, 20000, 4095);
	pid_set_out_shift(&mainboard.angle.pid, 10);
	pid_set_derivate_filter(&mainboard.angle.pid, 4);

	/* QUADRAMP */
	quadramp_init(&mainboard.angle.qr);
	quadramp_set_1st_order_vars(&mainboard.angle.qr, 200, 200);	/* set speed */
	quadramp_set_2nd_order_vars(&mainboard.angle.qr, 17, 17);	/* set accel */

	/* CS */
	cs_init(&mainboard.angle.cs);
	cs_set_consign_filter(&mainboard.angle.cs, quadramp_do_filter,
			      &mainboard.angle.qr);
	cs_set_correct_filter(&mainboard.angle.cs, pid_do_filter,
			      &mainboard.angle.pid);
	cs_set_process_in(&mainboard.angle.cs, rs_set_angle, &mainboard.rs);
	cs_set_process_out(&mainboard.angle.cs, rs_get_angle, &mainboard.rs);
	cs_set_consign(&mainboard.angle.cs, 0);

	/* Blocking detection */
	bd_init(&mainboard.angle.bd);
	bd_set_speed_threshold(&mainboard.angle.bd, 80);	// bd enable when speed thr > 80
	bd_set_current_thresholds(&mainboard.angle.bd, 500, 8000, 1000000, 50);	// k1, k2, i_thres, cpt_thres

	/* ---- CS distance */
	/* PID */
	pid_init(&mainboard.distance.pid);
//      pid_set_gains(&mainboard.distance.pid, 500, 10, 7000);
	pid_set_gains(&mainboard.distance.pid, 200, 0, 1200);
	pid_set_maximums(&mainboard.distance.pid, 0, 2000, 4095);
	pid_set_out_shift(&mainboard.distance.pid, 10);	// div by 2^10 = 1024
	pid_set_derivate_filter(&mainboard.distance.pid, 6);

	/* QUADRAMP22 */
	quadramp_init(&mainboard.distance.qr);
	quadramp_set_1st_order_vars(&mainboard.distance.qr, 150, 150);	/* set speed */
	quadramp_set_2nd_order_vars(&mainboard.distance.qr, 6, 6);	/* set accel */

	/* CS */
	cs_init(&mainboard.distance.cs);
	cs_set_consign_filter(&mainboard.distance.cs, quadramp_do_filter,
			      &mainboard.distance.qr);
	cs_set_correct_filter(&mainboard.distance.cs, pid_do_filter,
			      &mainboard.distance.pid);
	cs_set_process_in(&mainboard.distance.cs, rs_set_distance,
			  &mainboard.rs);
	cs_set_process_out(&mainboard.distance.cs, rs_get_distance,
			   &mainboard.rs);
	cs_set_consign(&mainboard.distance.cs, 0);

	/* Blocking detection */
	bd_init(&mainboard.distance.bd);
	bd_set_speed_threshold(&mainboard.distance.bd, 100);
	bd_set_current_thresholds(&mainboard.distance.bd, 500, 8000, 1000000,
				  50);

	/* ---- CS lift */
	/* PID */
	pid_init(&mainboard.lift.pid);
	pid_set_gains(&mainboard.lift.pid, 10000, 0, 0);
	pid_set_maximums(&mainboard.lift.pid, 10000, 0, 3000);
	pid_set_out_shift(&mainboard.lift.pid, 6);

	/* Filter init & params */
	quadramp_init(&mainboard.lift.qr);
	quadramp_set_2nd_order_vars(&mainboard.lift.qr, 2, 1);
	quadramp_set_1st_order_vars(&mainboard.lift.qr, 10, 10);

	/* CS */
	cs_init(&mainboard.lift.cs);
	cs_set_consign_filter(&mainboard.lift.cs, quadramp_do_filter,
			      &mainboard.lift.qr);
	cs_set_correct_filter(&mainboard.lift.cs, pid_do_filter,
			      &mainboard.lift.pid);
	cs_set_process_in(&mainboard.lift.cs, pwm_ng_set, &gen.pwm3_1A);
	cs_set_process_out(&mainboard.lift.cs, encoders_hctl_get_value,
			   ENCODER_LIFT);
	cs_set_consign(&mainboard.lift.cs, 0);

	/* set them on !! */
	mainboard.angle.on = 1;
	mainboard.distance.on = 1;
	mainboard.lift.on = 1;

	scheduler_add_periodical_event_priority(do_cs, NULL,
						5000L / SCHEDULER_UNIT,
						CS_PRIO);
}
