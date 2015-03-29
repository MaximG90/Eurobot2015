/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 */

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <clock_time.h>

#include <aversive.h>
#include <aversive/pgmspace.h>
#include <aversive/queue.h>
#include <aversive/wait.h>
#include <aversive/error.h>

#include <encoders_hctl.h>
#include <scheduler.h>

#include "actuator.h"
#include "cs.h"
#include "main.h"
#include "robot.h"
#include "sensor.h"

#define LIFT_POSITION_UP			475
#define LIFT_POSITION_MEDIUM_UP			55
#define LIFT_POSITION_DOWN			25

#define LIFT_CLOSE_TIME				100000ULL

#define LIFT_MAX_GLASSES			4

enum lift_state {
	LIFT_STATE_STOPPED,
	LIFT_STATE_IDLE,
	LIFT_STATE_GO_DOWN,
	LIFT_STATE_CLOSING,
	LIFT_STATE_GO_UP,
	LIFT_STATE_DROPPING,
};

enum lift_command {
	LIFT_COMMAND_NONE,
	LIFT_COMMAND_START,
	LIFT_COMMAND_STOP,
	LIFT_COMMAND_DROP,
};
 
struct lift {
	enum lift_state state;
	enum lift_command command;
	unsigned int target_pos;
	unsigned int n_glasses;
	microseconds time;
};

static struct lift lift;

static void lift_set_consign(int32_t value)
{
	cs_set_consign(&mainboard.lift.cs, value);
}

static bool lift_motion_done(int32_t consign)
{
	if (fabs(consign - encoders_hctl_get_value(ENCODER_LIFT)) <= 8)
		return true;
	else
		return false;
}

static void lift_task(void *dummy)
{
	enum lift_command command;
	enum lift_state state;
	microseconds time;
	bool presence;

	/* Read inputs */
	presence = sensor_get(S_LIFT);
	command = lift.command;
	state = lift.state;
	time = time_get_us2();

	/* printf_P(PSTR("lift: s %u c %u s %u n %u\r\n"),
		 state, command, presence, lift.n_glasses); */

	/* Evaluate the state */
	if (command == LIFT_COMMAND_STOP) {
		state = LIFT_STATE_STOPPED;
		command = LIFT_COMMAND_NONE;
	}

	switch (state) {
	case LIFT_STATE_STOPPED:
		if (command == LIFT_COMMAND_START) {
			guide_set_position(GUIDE_POSITION_CLOSE);
			state = LIFT_STATE_IDLE;
			command = LIFT_COMMAND_NONE;
		}
		break;

	case LIFT_STATE_IDLE:
		if (command == LIFT_COMMAND_DROP) {
			lift_set_consign(LIFT_POSITION_DOWN);
			state = LIFT_STATE_DROPPING;
			command = LIFT_COMMAND_NONE;
		} else if (presence && lift.n_glasses < LIFT_MAX_GLASSES) {
			hand_set_position(HAND_POSITION_OPEN);
			lift_set_consign(LIFT_POSITION_DOWN);
			state = LIFT_STATE_GO_DOWN;
		}
		break;

	case LIFT_STATE_GO_DOWN:
		if (lift_motion_done(LIFT_POSITION_DOWN)) {
			hand_set_position(HAND_POSITION_CLOSE);
			lift.time = time_get_us2();
			lift.n_glasses++;
			state = LIFT_STATE_CLOSING;
		}
		break;

	case LIFT_STATE_CLOSING:
		if (time - lift.time >= LIFT_CLOSE_TIME) {
			lift.target_pos = lift.n_glasses == LIFT_MAX_GLASSES
					? LIFT_POSITION_MEDIUM_UP
					: LIFT_POSITION_UP;
			lift_set_consign(lift.target_pos);
			state = LIFT_STATE_GO_UP;
		}
		break;

	case LIFT_STATE_GO_UP:
		if (lift_motion_done(lift.target_pos)) {
			state = LIFT_STATE_IDLE;
		}
		break;

	case LIFT_STATE_DROPPING:
		if (lift_motion_done(LIFT_POSITION_DOWN)) {
			hand_set_position(HAND_POSITION_OPEN);
			guide_set_position(GUIDE_POSITION_OPEN);
			state = LIFT_STATE_STOPPED;
			lift.n_glasses = 0;
		}
		break;
	}

	lift.state = state;
	lift.command = command;
}

void lift_start(void)
{
	lift.command = LIFT_COMMAND_START;
}

void lift_stop(void)
{
	lift.command = LIFT_COMMAND_STOP;
}

void lift_drop(void)
{
	lift.command = LIFT_COMMAND_DROP;
}

bool lift_is_stopped(void)
{
	return lift.state == LIFT_STATE_STOPPED;
}

bool lift_is_empty(void)
{
	return lift.n_glasses == 0;
}

bool lift_is_full(void)
{
	return lift.n_glasses >= LIFT_MAX_GLASSES;
}

void lift_init(void)
{
	int32_t pos, prev_pos;
	unsigned int i;

	/* Move the lift down at low speed (PWM) */
	/*pwm_ng_set(&gen.pwm3_1A, -3000);
	pos = encoders_hctl_get_value(ENCODER_LIFT);

	for (i = 0; i < 10; ++i) {
		wait_ms(500);
		prev_pos = pos;
		pos = encoders_hctl_get_value(ENCODER_LIFT);
		
		if (prev_pos == pos)
			break;
	}

	pwm_ng_set(&gen.pwm3_1A, 0);
*/
	encoders_hctl_set_value(ENCODER_LIFT, 0);

	lift.state = LIFT_STATE_STOPPED;
	lift.command = LIFT_COMMAND_NONE;
	lift.n_glasses = 0;

	scheduler_add_periodical_event_priority(lift_task, NULL,
						250000L / SCHEDULER_UNIT,
						LIFT_PRIO);

	printf_P(PSTR("lift init done\r\n"));
}
