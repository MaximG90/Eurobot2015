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

#include <stdio.h>
#include <string.h>

#include <aversive/pgmspace.h>
#include <aversive/wait.h>
#include <aversive/error.h>

#include <clock_time.h>
#include <encoders_hctl.h>

#include <trajectory_manager.h>
#include <vect_base.h>
#include <lines.h>
#include <polygon.h>
#include <robot_system.h>
#include <position_manager.h>

#include <parse.h>
#include <parse_string.h>
#include <parse_num.h>

#include "cs.h"
#include "cmdline.h"
#include "main.h"
#include "motion.h"
#include "robot.h"
#include "strat.h"

/**********************************************************/
/* Traj_Speeds for trajectory_manager */

struct cmd_traj_speed_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	uint16_t s;
};

static void cmd_traj_speed_parsed(void *parsed_result, void *data)
{
	struct cmd_traj_speed_result *res = parsed_result;

	if (!strcmp_P(res->arg1, PSTR("angle"))) {
		trajectory_set_speed(&mainboard.traj, mainboard.traj.d_speed,
				     res->s);
	} else if (!strcmp_P(res->arg1, PSTR("distance"))) {
		trajectory_set_speed(&mainboard.traj, res->s,
				     mainboard.traj.a_speed);
	}
	/* else it is a "show" */

	printf_P(PSTR("angle %u, distance %u\r\n"),
		 mainboard.traj.a_speed, mainboard.traj.d_speed);
}

static const prog_char str_traj_speed_arg0[] = "traj_speed";
static const parse_pgm_token_string_t cmd_traj_speed_arg0 =
TOKEN_STRING_INITIALIZER(struct cmd_traj_speed_result, arg0,
			 str_traj_speed_arg0);
static const prog_char str_traj_speed_arg1[] = "angle#distance";
static const parse_pgm_token_string_t cmd_traj_speed_arg1 =
TOKEN_STRING_INITIALIZER(struct cmd_traj_speed_result, arg1,
			 str_traj_speed_arg1);
static const parse_pgm_token_num_t cmd_traj_speed_s =
TOKEN_NUM_INITIALIZER(struct cmd_traj_speed_result, s, UINT16);

static const prog_char help_traj_speed[] = "Set traj_speed values for trajectory manager";
const parse_pgm_inst_t  cmd_traj_speed = {
	.f = cmd_traj_speed_parsed,
	.data = NULL,
	.help_str = help_traj_speed,
	.tokens = {
		(prog_void *)&cmd_traj_speed_arg0,
		(prog_void *)&cmd_traj_speed_arg1,
		(prog_void *)&cmd_traj_speed_s,
		NULL,
	},
};

/* show */

static const prog_char str_traj_speed_show_arg[] = "show";
static const parse_pgm_token_string_t cmd_traj_speed_show_arg =
TOKEN_STRING_INITIALIZER(struct cmd_traj_speed_result, arg1,
			 str_traj_speed_show_arg);

static const prog_char help_traj_speed_show[] =
    "Show traj_speed values for trajectory manager";
const parse_pgm_inst_t  cmd_traj_speed_show = {
	.f = cmd_traj_speed_parsed,
	.data = NULL,
	.help_str = help_traj_speed_show,
	.tokens = {
		(prog_void *)&cmd_traj_speed_arg0,
		(prog_void *)&cmd_traj_speed_show_arg,
		NULL,
	},
};

/**********************************************************/
/* trajectory window configuration */

struct cmd_trajectory_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	float d_win;
	float a_win;
	float a_start;
};

static void cmd_trajectory_parsed(void *parsed_result, void *data)
{
	struct cmd_trajectory_result *res = parsed_result;

	if (!strcmp_P(res->arg1, PSTR("set"))) {
		trajectory_set_windows(&mainboard.traj, res->d_win,
				       res->a_win, res->a_start);
	}

	printf_P(PSTR("trajectory %2.2f %2.2f %2.2f\r\n"), mainboard.traj.d_win,
		 DEG(mainboard.traj.a_win_rad),
		 DEG(mainboard.traj.a_start_rad));
}

static const prog_char str_trajectory_arg0[] = "trajectory";
static const parse_pgm_token_string_t cmd_trajectory_arg0 =
TOKEN_STRING_INITIALIZER(struct cmd_trajectory_result, arg0,
			 str_trajectory_arg0);
static const prog_char str_trajectory_arg1[] = "set";
static const parse_pgm_token_string_t cmd_trajectory_arg1 =
TOKEN_STRING_INITIALIZER(struct cmd_trajectory_result, arg1,
			 str_trajectory_arg1);
static const parse_pgm_token_num_t cmd_trajectory_d =
TOKEN_NUM_INITIALIZER(struct cmd_trajectory_result, d_win, FLOAT);
static const parse_pgm_token_num_t cmd_trajectory_a =
TOKEN_NUM_INITIALIZER(struct cmd_trajectory_result, a_win, FLOAT);
static const parse_pgm_token_num_t cmd_trajectory_as =
TOKEN_NUM_INITIALIZER(struct cmd_trajectory_result, a_start, FLOAT);

static const prog_char help_trajectory[] =
    "Set trajectory windows (distance, angle, angle_start)";
const parse_pgm_inst_t  cmd_trajectory = {
	.f = cmd_trajectory_parsed,
	.data = NULL,
	.help_str = help_trajectory,
	.tokens = {
		(prog_void *)&cmd_trajectory_arg0,
		(prog_void *)&cmd_trajectory_arg1,
		(prog_void *)&cmd_trajectory_d,
		(prog_void *)&cmd_trajectory_a,
		(prog_void *)&cmd_trajectory_as,
		NULL,
	},
};

/* show */

static const prog_char str_trajectory_show_arg[] = "show";
static const parse_pgm_token_string_t cmd_trajectory_show_arg =
TOKEN_STRING_INITIALIZER(struct cmd_trajectory_result, arg1,
			 str_trajectory_show_arg);

static const prog_char help_trajectory_show[] = "Show trajectory window configuration";
const parse_pgm_inst_t  cmd_trajectory_show = {
	.f = cmd_trajectory_parsed,
	.data = NULL,
	.help_str = help_trajectory_show,
	.tokens = {
		(prog_void *)&cmd_trajectory_arg0,
		(prog_void *)&cmd_trajectory_show_arg,
		NULL,
	},
};

/**********************************************************/
/* rs_gains configuration */

struct cmd_rs_gains_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	float left;
	float right;
};

static void cmd_rs_gains_parsed(void *parsed_result, void *data)
{
	struct cmd_rs_gains_result *res = parsed_result;

	if (!strcmp_P(res->arg1, PSTR("set"))) {
		rs_set_left_ext_encoder(&mainboard.rs, encoders_hctl_get_value, LEFT_ENCODER, res->left);	// en augmentant on tourne à gauche
		rs_set_right_ext_encoder(&mainboard.rs, encoders_hctl_get_value, RIGHT_ENCODER, res->right);	//en augmentant on tourne à droite
	}
	printf_P(PSTR("rs_gains set "));
	f64_print(mainboard.rs.left_ext_gain);
	printf_P(PSTR(" "));
	f64_print(mainboard.rs.right_ext_gain);
	printf_P(PSTR("\r\n"));
}

static const prog_char str_rs_gains_arg0[] = "rs_gains";
static const parse_pgm_token_string_t cmd_rs_gains_arg0 =
TOKEN_STRING_INITIALIZER(struct cmd_rs_gains_result, arg0, str_rs_gains_arg0);
static const prog_char str_rs_gains_arg1[] = "set";
static const parse_pgm_token_string_t cmd_rs_gains_arg1 =
TOKEN_STRING_INITIALIZER(struct cmd_rs_gains_result, arg1, str_rs_gains_arg1);
static const parse_pgm_token_num_t cmd_rs_gains_l =
TOKEN_NUM_INITIALIZER(struct cmd_rs_gains_result, left, FLOAT);
static const parse_pgm_token_num_t cmd_rs_gains_r =
TOKEN_NUM_INITIALIZER(struct cmd_rs_gains_result, right, FLOAT);

static const prog_char help_rs_gains[] = "Set rs_gains (left, right)";
const parse_pgm_inst_t  cmd_rs_gains = {
	.f = cmd_rs_gains_parsed,
	.data = NULL,
	.help_str = help_rs_gains,
	.tokens = {
		(prog_void *)&cmd_rs_gains_arg0,
		(prog_void *)&cmd_rs_gains_arg1,
		(prog_void *)&cmd_rs_gains_l,
		(prog_void *)&cmd_rs_gains_r,
		NULL,
	},
};

/* show */

static const prog_char str_rs_gains_show_arg[] = "show";
static const parse_pgm_token_string_t cmd_rs_gains_show_arg =
TOKEN_STRING_INITIALIZER(struct cmd_rs_gains_result, arg1,
			 str_rs_gains_show_arg);

static const prog_char help_rs_gains_show[] = "Show rs_gains";
const parse_pgm_inst_t  cmd_rs_gains_show = {
	.f = cmd_rs_gains_parsed,
	.data = NULL,
	.help_str = help_rs_gains_show,
	.tokens = {
		(prog_void *)&cmd_rs_gains_arg0,
		(prog_void *)&cmd_rs_gains_show_arg,
		NULL,
	},
};

/**********************************************************/
/* track configuration */

struct cmd_track_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	float val;
};

static void cmd_track_parsed(void *parsed_result, void *data)
{
	struct cmd_track_result *res = parsed_result;

	if (!strcmp_P(res->arg1, PSTR("set"))) {
		position_set_physical_params(&mainboard.pos, res->val,
					     ENC_PULSE_PER_MM);
	}
	printf_P(PSTR("track set %f\r\n"), mainboard.pos.phys.track_mm);
}

static const prog_char str_track_arg0[] = "track";
static const parse_pgm_token_string_t cmd_track_arg0 =
TOKEN_STRING_INITIALIZER(struct cmd_track_result, arg0, str_track_arg0);
static const prog_char str_track_arg1[] = "set";
static const parse_pgm_token_string_t cmd_track_arg1 =
TOKEN_STRING_INITIALIZER(struct cmd_track_result, arg1, str_track_arg1);
static const parse_pgm_token_num_t cmd_track_val =
TOKEN_NUM_INITIALIZER(struct cmd_track_result, val, FLOAT);

static const prog_char help_track[] = "Set track in mm";
const parse_pgm_inst_t  cmd_track = {
	.f = cmd_track_parsed,
	.data = NULL,
	.help_str = help_track,
	.tokens = {
		(prog_void *)&cmd_track_arg0,
		(prog_void *)&cmd_track_arg1,
		(prog_void *)&cmd_track_val,
		NULL,
	},
};

/* show */

static const prog_char str_track_show_arg[] = "show";
static const parse_pgm_token_string_t cmd_track_show_arg =
TOKEN_STRING_INITIALIZER(struct cmd_track_result, arg1, str_track_show_arg);

static const prog_char help_track_show[] = "Show track";
const parse_pgm_inst_t  cmd_track_show = {
	.f = cmd_track_parsed,
	.data = NULL,
	.help_str = help_track_show,
	.tokens = {
		(prog_void *)&cmd_track_arg0,
		(prog_void *)&cmd_track_show_arg,
		NULL,
	},
};

/**********************************************************/
/* Pt_Lists for testing traj */

#define PT_LIST_SIZE 10
static struct xy_point pt_list[PT_LIST_SIZE];
static uint16_t pt_list_len = 0;

struct cmd_pt_list_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	uint16_t arg2;
	int16_t arg3;
	int16_t arg4;
};

static void cmd_pt_list_parsed(void *parsed_result, void *data)
{
	struct cmd_pt_list_result *res = parsed_result;
	uint8_t i, why = 0;

	if (!strcmp_P(res->arg1, PSTR("append"))) {
		res->arg2 = pt_list_len;
	}

	if (!strcmp_P(res->arg1, PSTR("insert")) ||
	    !strcmp_P(res->arg1, PSTR("append"))) {
		if (res->arg2 > pt_list_len) {
			printf_P(PSTR("Index too large\r\n"));
			return;
		}
		if (pt_list_len >= PT_LIST_SIZE) {
			printf_P(PSTR("List is too large\r\n"));
			return;
		}
		memmove(&pt_list[res->arg2 + 1], &pt_list[res->arg2],
			PT_LIST_SIZE - 1 - res->arg2);
		pt_list[res->arg2].x = res->arg3;
		pt_list[res->arg2].y = res->arg4;
		pt_list_len++;
	} else if (!strcmp_P(res->arg1, PSTR("del"))) {
		if (pt_list_len <= 0) {
			printf_P(PSTR("Error: list empty\r\n"));
			return;
		}
		if (res->arg2 > pt_list_len) {
			printf_P(PSTR("Index too large\r\n"));
			return;
		}
		memmove(&pt_list[res->arg2], &pt_list[res->arg2 + 1],
			(PT_LIST_SIZE - 1 -
			 res->arg2) * sizeof(struct xy_point));
		pt_list_len--;
	} else if (!strcmp_P(res->arg1, PSTR("reset"))) {
		pt_list_len = 0;
	}

	/* else it is a "show" or a "start" */
	if (pt_list_len == 0) {
		printf_P(PSTR("List empty\r\n"));
		return;
	}
	for (i = 0; i < pt_list_len; i++) {
		printf_P(PSTR("%d: x=%d y=%d\r\n"), i, pt_list[i].x,
			 pt_list[i].y);
		if (!strcmp_P(res->arg1, PSTR("start"))) {
			trajectory_goto_xy_abs(&mainboard.traj, pt_list[i].x,
					       pt_list[i].y);
			why = motion_wait_traj_end(0xFF);	/* all */
		}
		if (why & (~(MOTION_END_TRAJ | MOTION_END_NEAR)))
			trajectory_stop(&mainboard.traj);
	}
}

static const prog_char str_pt_list_arg0[] = "pt_list";
static const parse_pgm_token_string_t cmd_pt_list_arg0 =
TOKEN_STRING_INITIALIZER(struct cmd_pt_list_result, arg0, str_pt_list_arg0);
static const prog_char str_pt_list_arg1[] = "insert";
static const parse_pgm_token_string_t cmd_pt_list_arg1 =
TOKEN_STRING_INITIALIZER(struct cmd_pt_list_result, arg1, str_pt_list_arg1);
static const parse_pgm_token_num_t cmd_pt_list_arg2 =
TOKEN_NUM_INITIALIZER(struct cmd_pt_list_result, arg2, UINT16);
static const parse_pgm_token_num_t cmd_pt_list_arg3 =
TOKEN_NUM_INITIALIZER(struct cmd_pt_list_result, arg3, INT16);
static const parse_pgm_token_num_t cmd_pt_list_arg4 =
TOKEN_NUM_INITIALIZER(struct cmd_pt_list_result, arg4, INT16);

static const prog_char help_pt_list[] = "Insert point in pt_list (idx,x,y)";
const parse_pgm_inst_t  cmd_pt_list = {
	.f = cmd_pt_list_parsed,
	.data = NULL,
	.help_str = help_pt_list,
	.tokens = {
		(prog_void *)&cmd_pt_list_arg0,
		(prog_void *)&cmd_pt_list_arg1,
		(prog_void *)&cmd_pt_list_arg2,
		(prog_void *)&cmd_pt_list_arg3,
		(prog_void *)&cmd_pt_list_arg4,
		NULL,
	},
};

/* append */

static const prog_char str_pt_list_arg1_append[] = "append";
static const parse_pgm_token_string_t cmd_pt_list_arg1_append =
TOKEN_STRING_INITIALIZER(struct cmd_pt_list_result, arg1,
			 str_pt_list_arg1_append);

static const prog_char help_pt_list_append[] = "Append point in pt_list (x,y)";
const parse_pgm_inst_t  cmd_pt_list_append = {
	.f = cmd_pt_list_parsed,
	.data = NULL,
	.help_str = help_pt_list_append,
	.tokens = {
		(prog_void *)&cmd_pt_list_arg0,
		(prog_void *)&cmd_pt_list_arg1_append,
		(prog_void *)&cmd_pt_list_arg3,
		(prog_void *)&cmd_pt_list_arg4,
		NULL,
	},
};

/* del */

static const prog_char str_pt_list_del_arg[] = "del";
static const parse_pgm_token_string_t cmd_pt_list_del_arg =
TOKEN_STRING_INITIALIZER(struct cmd_pt_list_result, arg1, str_pt_list_del_arg);

static const prog_char help_pt_list_del[] = "Del or insert point in pt_list (num)";
const parse_pgm_inst_t  cmd_pt_list_del = {
	.f = cmd_pt_list_parsed,
	.data = NULL,
	.help_str = help_pt_list_del,
	.tokens = {
		(prog_void *)&cmd_pt_list_arg0,
		(prog_void *)&cmd_pt_list_del_arg,
		(prog_void *)&cmd_pt_list_arg2,
		NULL,
	},
};

/* show */

static const prog_char str_pt_list_show_arg[] = "show#reset#start";
static const parse_pgm_token_string_t cmd_pt_list_show_arg =
TOKEN_STRING_INITIALIZER(struct cmd_pt_list_result, arg1, str_pt_list_show_arg);

static const prog_char help_pt_list_show[] = "Show, start or reset pt_list";
const parse_pgm_inst_t  cmd_pt_list_show = {
	.f = cmd_pt_list_parsed,
	.data = NULL,
	.help_str = help_pt_list_show,
	.tokens = {
		(prog_void *)&cmd_pt_list_arg0,
		(prog_void *)&cmd_pt_list_show_arg,
		NULL,
	},
};

/**********************************************************/
/* Goto function */

struct cmd_goto_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	int32_t arg2;
	int32_t arg3;
	int32_t arg4;
};

static void debug_asserv(microseconds time_stamp)
{
	printf_P(PSTR("%d;"), time_stamp / 1000);

	/* for fine tuning pid on angle & distance */

#if 1

//    printf_P(PSTR("%d;"),time_stamp/1000 );

	dump_cs_simple(&mainboard.distance.cs);
	dump_cs_simple(&mainboard.angle.cs);
	printf_P(PSTR("%ld;%ld;"), mainboard.pwm_r, mainboard.pwm_l);
	printf_P(PSTR("%f;%f;"), position_get_x_double(&mainboard.pos),
		 position_get_y_double(&mainboard.pos));
//#else
	/* for tuning blocking detection */

	printf_P(PSTR("%ld;%ld;"), mainboard.distance.bd.speed, mainboard.distance.bd.cpt);

	printf_P(PSTR("%ld;%ld;"), mainboard.angle.bd.speed, mainboard.angle.bd.cpt);
	
	//mainboard.angle.bd.i normalement  mais supprimé
#endif
	printf_P(PSTR("\n\r"));
}

static void cmd_goto_parsed(void *parsed_result, void *data)
{
	struct cmd_goto_result *res = parsed_result;
	uint8_t err, exit;
	microseconds t1, t2, time_stamp, time_start;

	motion_interrupt_traj_reset();
	if (!strcmp_P(res->arg1, PSTR("a_rel"))) {
		trajectory_a_rel(&mainboard.traj, res->arg2);
	} else if (!strcmp_P(res->arg1, PSTR("d_rel"))) {
		trajectory_d_rel(&mainboard.traj, res->arg2);
	} else if (!strcmp_P(res->arg1, PSTR("a_abs"))) {
		trajectory_a_abs(&mainboard.traj, res->arg2);
	} else if (!strcmp_P(res->arg1, PSTR("a_to_xy"))) {
		trajectory_turnto_xy(&mainboard.traj, res->arg2, res->arg3);
	} else if (!strcmp_P(res->arg1, PSTR("a_behind_xy"))) {
		trajectory_turnto_xy_behind(&mainboard.traj, res->arg2,
					    res->arg3);
	} else if (!strcmp_P(res->arg1, PSTR("xy_rel"))) {
		trajectory_goto_xy_rel(&mainboard.traj, res->arg2, res->arg3);
	} else if (!strcmp_P(res->arg1, PSTR("xy_abs"))) {
		trajectory_goto_xy_abs(&mainboard.traj, res->arg2, res->arg3);
	} else if (!strcmp_P(res->arg1, PSTR("xy_abs_fow"))) {
		trajectory_goto_forward_xy_abs(&mainboard.traj, res->arg2,
					       res->arg3);
	} else if (!strcmp_P(res->arg1, PSTR("xy_abs_back"))) {
		trajectory_goto_backward_xy_abs(&mainboard.traj, res->arg2,
						res->arg3);
	} else if (!strcmp_P(res->arg1, PSTR("da_rel"))) {
		trajectory_d_a_rel(&mainboard.traj, res->arg2, res->arg3);
	}
	time_stamp = 0;
	t1 = time_get_us2();
	time_start = time_get_us2();
	debug_asserv(time_stamp);
	exit = 0;
	while (exit < 10) {
		err = motion_test_traj_end(0xFF & (~MOTION_END_NEAR));	// ) != 0)  //  no NEAR
		t2 = time_get_us2();
		time_stamp = t2 - time_start;
		if (t2 - t1 >= 40000)	// 40 ms
		{
			debug_asserv(time_stamp);	// max 140 bytes at 57600 => 24,3 ms
			t1 = t2;
			if (err != 0)
				exit++;
		}
		if (err == MOTION_END_INTR)
			break;
	}

	if (err != MOTION_END_TRAJ && err != MOTION_END_NEAR)
		motion_hardstop();
	printf_P(PSTR("ok => %s\r\n"), motion_get_err(err));
}

static const prog_char str_goto_arg0[] = "goto";
static const parse_pgm_token_string_t cmd_goto_arg0 =
TOKEN_STRING_INITIALIZER(struct cmd_goto_result, arg0, str_goto_arg0);
static const prog_char str_goto_arg1_a[] = "d_rel#a_rel#a_abs";
static const parse_pgm_token_string_t cmd_goto_arg1_a =
TOKEN_STRING_INITIALIZER(struct cmd_goto_result, arg1, str_goto_arg1_a);
static const parse_pgm_token_num_t cmd_goto_arg2 =
TOKEN_NUM_INITIALIZER(struct cmd_goto_result, arg2, INT32);

/* 1 params */
static const prog_char help_goto1[] = "Change orientation of the mainboard";
const parse_pgm_inst_t  cmd_goto1 = {
	.f = cmd_goto_parsed,
	.data = NULL,
	.help_str = help_goto1,
	.tokens = {
		(prog_void *)&cmd_goto_arg0,
		(prog_void *)&cmd_goto_arg1_a,
		(prog_void *)&cmd_goto_arg2,
		NULL,
	},
};

static const prog_char str_goto_arg1_b[] =
    "xy_rel#xy_abs#xy_abs_fow#xy_abs_back#da_rel#a_to_xy#a_behind_xy";
static const parse_pgm_token_string_t cmd_goto_arg1_b =
TOKEN_STRING_INITIALIZER(struct cmd_goto_result, arg1, str_goto_arg1_b);
static const parse_pgm_token_num_t cmd_goto_arg3 =
TOKEN_NUM_INITIALIZER(struct cmd_goto_result, arg3, INT32);

/* 2 params */
static const prog_char help_goto2[] = "Go to a (x,y) or (d,a) position";
const parse_pgm_inst_t  cmd_goto2 = {
	.f = cmd_goto_parsed,
	.data = NULL,
	.help_str = help_goto2,
	.tokens = {
		(prog_void *)&cmd_goto_arg0,
		(prog_void *)&cmd_goto_arg1_b,
		(prog_void *)&cmd_goto_arg2,
		(prog_void *)&cmd_goto_arg3,
		NULL,
	},
};

/**********************************************************/
/* Position tests */

struct cmd_position_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	int32_t arg2;
	int32_t arg3;
	int32_t arg4;
};

static void cmd_position_parsed(void *parsed_result, void *data)
{
	struct cmd_position_result *res = parsed_result;

	/* display raw position values */
	if (!strcmp_P(res->arg1, PSTR("reset"))) {
		position_set(&mainboard.pos, 0, 0, 0);
	} else if (!strcmp_P(res->arg1, PSTR("set"))) {
		position_set(&mainboard.pos, res->arg2, res->arg3, res->arg4);
	} else if (!strcmp_P(res->arg1, PSTR("autoset_" ROBOT_COLOR_DEFAULT_NAME))) {
		mainboard.our_color = ROBOT_COLOR_DEFAULT;
		strat_auto_position();
	} else if (!strcmp_P(res->arg1, PSTR("autoset_" ROBOT_COLOR_ALTERNATE_NAME))) {
		mainboard.our_color = ROBOT_COLOR_ALTERNATE;
		strat_auto_position();
	}

	/* else it's just a "show" */
	printf_P(PSTR("x=%.2f y=%.2f a=%.2f\r\n"),
		 position_get_x_double(&mainboard.pos),
		 position_get_y_double(&mainboard.pos),
		 DEG(position_get_a_rad_double(&mainboard.pos)));
}

static const prog_char str_position_arg0[] = "position";
static const parse_pgm_token_string_t cmd_position_arg0 =
TOKEN_STRING_INITIALIZER(struct cmd_position_result, arg0, str_position_arg0);
static const prog_char str_position_arg1[] = "show#reset#autoset_" ROBOT_COLOR_DEFAULT_NAME \
				       "#autoset_" ROBOT_COLOR_ALTERNATE_NAME;
static const parse_pgm_token_string_t cmd_position_arg1 =
TOKEN_STRING_INITIALIZER(struct cmd_position_result, arg1, str_position_arg1);

static const prog_char help_position[] = "Show/reset (x,y,a) position";
const parse_pgm_inst_t  cmd_position = {
	.f = cmd_position_parsed,
	.data = NULL,
	.help_str = help_position,
	.tokens = {
		(prog_void *)&cmd_position_arg0,
		(prog_void *)&cmd_position_arg1,
		NULL,
	},
};

static const prog_char str_position_arg1_set[] = "set";
static const parse_pgm_token_string_t cmd_position_arg1_set =
TOKEN_STRING_INITIALIZER(struct cmd_position_result, arg1,
			 str_position_arg1_set);
static const parse_pgm_token_num_t cmd_position_arg2 =
TOKEN_NUM_INITIALIZER(struct cmd_position_result, arg2, INT32);
static const parse_pgm_token_num_t cmd_position_arg3 =
TOKEN_NUM_INITIALIZER(struct cmd_position_result, arg3, INT32);
static const parse_pgm_token_num_t cmd_position_arg4 =
TOKEN_NUM_INITIALIZER(struct cmd_position_result, arg4, INT32);

static const prog_char help_position_set[] = "Set (x,y,a) position";
const parse_pgm_inst_t  cmd_position_set = {
	.f = cmd_position_parsed,
	.data = NULL,
	.help_str = help_position_set,
	.tokens = {
		(prog_void *)&cmd_position_arg0,
		(prog_void *)&cmd_position_arg1_set,
		(prog_void *)&cmd_position_arg2,
		(prog_void *)&cmd_position_arg3,
		(prog_void *)&cmd_position_arg4,
		NULL,
	},
};

/* -----------------------------------------------------------------------------
 * motion - Calibrate motion parameters
 *
 * Usage: motion calibrate
 */

struct cmd_motion_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
};

static void cmd_motion_parsed(void *parsed_result, void *data)
{
	struct cmd_motion_result *res = parsed_result;
	int ret;

	if (strcmp_P(res->arg1, PSTR("calibrate")))
		return;

	ret = motion_calibration();
	printf_P(PSTR("motion calibration done (%d)\r\n"), ret);
}

static const prog_char str_motion_arg1[] = "calibrate";
static const parse_pgm_token_string_t cmd_motion_arg1 =
TOKEN_STRING_INITIALIZER(struct cmd_motion_result, arg1, str_motion_arg1);
static const prog_char str_motion_arg0[] = "motion";
static const parse_pgm_token_string_t cmd_motion_arg0 =
TOKEN_STRING_INITIALIZER(struct cmd_motion_result, arg0, str_motion_arg0);

static const prog_char help_motion[] = "Calibrate motion parameters";
const parse_pgm_inst_t  cmd_motion = {
	.f = cmd_motion_parsed,
	.data = NULL,
	.help_str = help_motion,
	.tokens = {
		(prog_void *)&cmd_motion_arg0,
		(prog_void *)&cmd_motion_arg1,
		NULL,
	},
};
