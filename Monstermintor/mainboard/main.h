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

#ifndef __MAIN_H__
#define __MAIN_H__

#include <aversive/pgmspace.h>

#include <ax12.h>
#include <blocking_detection_manager.h>
#include <control_system_manager.h>
#include <pid.h>
#include <position_manager.h>
#include <pwm_ng.h>
#include <quadramp.h>
#include <rdline.h>
#include <robot_system.h>
#include <trajectory_manager.h>

/* only 90 seconds, don't forget it :) */
#define MATCH_TIME		88

/* Robot colors. In order to avoid changing the code every time rules use new
 * colors, we define a default and an alternate color. The default color
 * corresponds to the left side of the table when coming from the backstage
 * area, and the alternate color as the right side.
 */
#define ROBOT_COLOR_DEFAULT	0
#define ROBOT_COLOR_ALTERNATE	1

/** ERROR NUMS */
#define E_USER_STRAT		194
#define E_USER_SENSOR		196
#define E_USER_CS		197

// High to low priority
#define LED_PRIO		170
#define TIME_PRIO		160
#define ADC_PRIO		120
#define LIFT_PRIO		110
#define CS_PRIO			100
#define STRAT_PRIO		30
#define ARMS_PRIO		20
#define BEACON_PRIO		5

#define CS_PERIOD		5000L

#define NB_LOGS			4

/* generic to all boards */
struct genboard {
	/* command line interface */
	struct rdline rdl;
	char prompt[RDLINE_PROMPT_SIZE];

	/* motors */
	struct pwm_ng pwm1_4A;
	struct pwm_ng pwm2_4B;
	struct pwm_ng pwm3_1A;
	struct pwm_ng pwm4_1B;

	/* servos */
	struct pwm_ng servo1;
	struct pwm_ng servo2;
	struct pwm_ng servo3;
	struct pwm_ng servo4;
	AX12 ax12;

	/* log */
	uint8_t logs[NB_LOGS + 1];
	uint8_t log_level;
	uint8_t debug;
};

struct cs_block {
	uint8_t on;
	struct cs cs;
	struct pid_filter pid;
	struct quadramp_filter qr;
	struct blocking_detection bd;
};

/* mainboard specific */
struct mainboard {
#define DO_ENCODERS  1
#define DO_CS        2
#define DO_RS        4
#define DO_POS       8
#define DO_BD       16
#define DO_TIMER    32
#define DO_POWER    64
#define DO_LOG      128

	uint8_t flags;		/* misc flags */
	uint8_t ir;		/* ir flags */

	/* control systems */
	struct cs_block angle;
	struct cs_block distance;
	struct cs_block lift;

	/* x,y positionning */
	struct robot_system rs;
	struct robot_position pos;
	struct trajectory traj;

	/* robot status */
	uint8_t our_color;
	volatile int16_t speed_a;	/* current angle speed */
	volatile int16_t speed_d;	/* current dist speed */
	int32_t pwm_l;		/* current left pwm */
	int32_t pwm_r;		/* current right pwm */
};

extern struct genboard gen;
extern struct mainboard mainboard;

#endif /* __MAIN_H__ */
