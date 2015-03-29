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

#include <trajectory_manager.h>

#include <parse.h>
#include <parse_string.h>
#include <parse_num.h>

#include "main.h"
#include "cs.h"
#include "cmdline.h"
#include "motion.h"
#include "strat.h"

/**********************************************************/
/* strat configuration */

struct cmd_strat_info_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
};

static void cmd_strat_info_parsed(void *parsed_result, void *data)
{
	struct cmd_strat_info_result *res = parsed_result;

	if (!strcmp_P(res->arg1, PSTR("reset"))) {
		strat_reset_info();
	}
	strat_info.dump_enabled = 1;
	strat_dump_info(__FUNCTION__);
}

static const prog_char str_strat_info_arg0[] = "strat_info";
static parse_token_string_t cmd_strat_info_arg0 =
TOKEN_STRING_INITIALIZER(struct cmd_strat_info_result, arg0,
			 str_strat_info_arg0);
static const prog_char str_strat_info_arg1[] = "show#reset";
static parse_token_string_t cmd_strat_info_arg1 =
TOKEN_STRING_INITIALIZER(struct cmd_strat_info_result, arg1,
			 str_strat_info_arg1);

static const prog_char help_strat_info[] = "reset/show strat_info";
parse_inst_t cmd_strat_info = {
	.f = cmd_strat_info_parsed,
	.data = NULL,
	.help_str = help_strat_info,
	.tokens = {
		(prog_void *)&cmd_strat_info_arg0,
		(prog_void *)&cmd_strat_info_arg1,
		NULL,
	},
};

/**********************************************************/
/* strat configuration */

struct cmd_strat_conf_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
};

static void cmd_strat_conf_parsed(void *parsed_result, void *data)
{
	struct cmd_strat_conf_result *res = parsed_result;

	if (!strcmp_P(res->arg1, PSTR("base"))) {
//              strat_info.conf.flags = 0;
//              strat_info.conf.scan_our_min_time = 90;
//              strat_info.conf.delay_between_our_scan = 90;
//              strat_info.conf.scan_opp_min_time = 90;
//              strat_info.conf.delay_between_opp_scan = 90;
	} else if (!strcmp_P(res->arg1, PSTR("big3"))) {
//              strat_info.conf.flags =
//                      STRAT_CONF_STORE_STATIC2 |
//                      STRAT_CONF_BIG_3_TEMPLE;
//              strat_info.conf.scan_our_min_time = 90;
//              strat_info.conf.delay_between_our_scan = 90;
//              strat_info.conf.scan_opp_min_time = 90;
//              strat_info.conf.delay_between_opp_scan = 90;
	} else if (!strcmp_P(res->arg1, PSTR("base_check"))) {
//              strat_info.conf.flags = 0;
//              strat_info.conf.scan_our_min_time = 35;
//              strat_info.conf.delay_between_our_scan = 90;
//              strat_info.conf.scan_opp_min_time = 90;
//              strat_info.conf.delay_between_opp_scan = 90;
	} else if (!strcmp_P(res->arg1, PSTR("big3_check"))) {
//              strat_info.conf.flags =
//                      STRAT_CONF_STORE_STATIC2 |
//                      STRAT_CONF_BIG_3_TEMPLE;
//              strat_info.conf.scan_our_min_time = 35;
//              strat_info.conf.delay_between_our_scan = 90;
//              strat_info.conf.scan_opp_min_time = 90;
//              strat_info.conf.delay_between_opp_scan = 90;
	} else if (!strcmp_P(res->arg1, PSTR("offensive_early"))) {
//              strat_info.conf.flags =
//                      STRAT_CONF_TAKE_ONE_LINTEL |
//                      STRAT_CONF_STORE_STATIC2 |
//                      STRAT_CONF_EARLY_SCAN |
//                      STRAT_CONF_PUSH_OPP_COLS;
//              strat_info.conf.scan_our_min_time = 50;
//              strat_info.conf.delay_between_our_scan = 90;
//              strat_info.conf.scan_opp_min_time = 15;
//              strat_info.conf.delay_between_opp_scan = 90;
	} else if (!strcmp_P(res->arg1, PSTR("offensive_late"))) {
//              strat_info.conf.flags = STRAT_CONF_TAKE_ONE_LINTEL;
//              strat_info.conf.scan_our_min_time = 90;
//              strat_info.conf.delay_between_our_scan = 90;
//              strat_info.conf.scan_opp_min_time = 30;
//              strat_info.conf.delay_between_opp_scan = 90;
	} else if (!strcmp_P(res->arg1, PSTR("one_on_disc"))) {
//              strat_info.conf.flags =
//                      STRAT_CONF_ONLY_ONE_ON_DISC;
//              strat_info.conf.scan_our_min_time = 90;
//              strat_info.conf.delay_between_our_scan = 90;
//              strat_info.conf.scan_opp_min_time = 90;
//              strat_info.conf.delay_between_opp_scan = 90;
	}
	strat_info.dump_enabled = 1;
	strat_dump_conf();
}

static const prog_char str_strat_conf_arg0[] = "strat_conf";
static parse_token_string_t cmd_strat_conf_arg0 =
TOKEN_STRING_INITIALIZER(struct cmd_strat_conf_result, arg0,
			 str_strat_conf_arg0);
static const prog_char str_strat_conf_arg1[] =
    "show#base#big3#base_check#big3_check#offensive_early#offensive_late#one_on_disc";
static parse_token_string_t cmd_strat_conf_arg1 =
TOKEN_STRING_INITIALIZER(struct cmd_strat_conf_result, arg1,
			 str_strat_conf_arg1);

static const prog_char help_strat_conf[] = "configure strat options";
parse_inst_t cmd_strat_conf = {
	.f = cmd_strat_conf_parsed,
	.data = NULL,
	.help_str = help_strat_conf,
	.tokens = {
		(prog_void *)&cmd_strat_conf_arg0,
		(prog_void *)&cmd_strat_conf_arg1,
		NULL,
	},
};

/**********************************************************/
/* strat configuration */

struct cmd_strat_conf2_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	fixed_string_t arg2;
};

static void cmd_strat_conf2_parsed(void *parsed_result, void *data)
{
	struct cmd_strat_conf2_result *res = parsed_result;
//	uint8_t bit = 0;
	uint8_t on;

	if (!strcmp_P(res->arg2, PSTR("on")))
		on = 1;
	else
		on = 0;

//      if (!strcmp_P(res->arg1, PSTR("one_temple_on_disc")))
//              bit = STRAT_CONF_ONLY_ONE_ON_DISC;
//      else if (!strcmp_P(res->arg1, PSTR("bypass_static2")))
//              bit = STRAT_CONF_BYPASS_STATIC2;
//      else if (!strcmp_P(res->arg1, PSTR("take_one_lintel")))
//              bit = STRAT_CONF_TAKE_ONE_LINTEL;
//      else if (!strcmp_P(res->arg1, PSTR("skip_when_check_fail")))
//              bit = STRAT_CONF_TAKE_ONE_LINTEL;
//      else if (!strcmp_P(res->arg1, PSTR("store_static2")))
//              bit = STRAT_CONF_STORE_STATIC2;
//      else if (!strcmp_P(res->arg1, PSTR("big3_temple")))
//              bit = STRAT_CONF_BIG_3_TEMPLE;
//      else if (!strcmp_P(res->arg1, PSTR("early_opp_scan")))
//              bit = STRAT_CONF_EARLY_SCAN;
//      else if (!strcmp_P(res->arg1, PSTR("push_opp_cols")))
//              bit = STRAT_CONF_PUSH_OPP_COLS;
//
//	if (on)
//		strat_info.conf.flags |= bit;
//	else
//		strat_info.conf.flags &= (~bit);

	strat_info.dump_enabled = 1;
	strat_dump_conf();
}

static const prog_char str_strat_conf2_arg0[] = "strat_conf";
static parse_token_string_t cmd_strat_conf2_arg0 =
TOKEN_STRING_INITIALIZER(struct cmd_strat_conf2_result, arg0,
			 str_strat_conf2_arg0);
static const prog_char str_strat_conf2_arg1[] =
    "push_opp_cols#one_temple_on_disc#bypass_static2#take_one_lintel#skip_when_check_fail#store_static2#big3_temple#early_opp_scan";
static parse_token_string_t cmd_strat_conf2_arg1 =
TOKEN_STRING_INITIALIZER(struct cmd_strat_conf2_result, arg1,
			 str_strat_conf2_arg1);
static const prog_char str_strat_conf2_arg2[] = "on#off";
static parse_token_string_t cmd_strat_conf2_arg2 =
TOKEN_STRING_INITIALIZER(struct cmd_strat_conf2_result, arg2,
			 str_strat_conf2_arg2);

static const prog_char help_strat_conf2[] = "configure strat options";
parse_inst_t cmd_strat_conf2 = {
	.f = cmd_strat_conf2_parsed,
	.data = NULL,
	.help_str = help_strat_conf2,
	.tokens = {
		(prog_void *)&cmd_strat_conf2_arg0,
		(prog_void *)&cmd_strat_conf2_arg1,
		(prog_void *)&cmd_strat_conf2_arg2,
		NULL,
	},
};

/**********************************************************/
/* strat configuration */

struct cmd_strat_conf3_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	uint8_t arg2;
};

static void cmd_strat_conf3_parsed(void *parsed_result, void *data)
{
//      struct cmd_strat_conf3_result *res = parsed_result;

//      if (!strcmp_P(res->arg1, PSTR("scan_our_min_time"))) {
//              if (res->arg2 > 90)
//                      res->arg2 = 90;
//              strat_info.conf.scan_our_min_time = res->arg2;
//      }
//      else if (!strcmp_P(res->arg1, PSTR("delay_between_our_scan"))) {
//              if (res->arg2 > 90)
//                      res->arg2 = 90;
//              strat_info.conf.delay_between_our_scan = res->arg2;
//      }
	strat_info.dump_enabled = 1;
	strat_dump_conf();
}

static const prog_char str_strat_conf3_arg0[] = "strat_conf";
static parse_token_string_t cmd_strat_conf3_arg0 =
TOKEN_STRING_INITIALIZER(struct cmd_strat_conf3_result, arg0,
			 str_strat_conf3_arg0);
static const prog_char str_strat_conf3_arg1[] =
    "scan_our_min_time#delay_between_our_scan";
static parse_token_string_t cmd_strat_conf3_arg1 =
TOKEN_STRING_INITIALIZER(struct cmd_strat_conf3_result, arg1,
			 str_strat_conf3_arg1);
static parse_token_num_t cmd_strat_conf3_arg2 =
TOKEN_NUM_INITIALIZER(struct cmd_strat_conf3_result, arg2, UINT16);

static const prog_char help_strat_conf3[] = "configure strat options";
parse_inst_t cmd_strat_conf3 = {
	.f = cmd_strat_conf3_parsed,
	.data = NULL,
	.help_str = help_strat_conf3,
	.tokens = {
		(prog_void *)&cmd_strat_conf3_arg0,
		(prog_void *)&cmd_strat_conf3_arg1,
		(prog_void *)&cmd_strat_conf3_arg2,
		NULL,
	},
};

/**********************************************************/
/* Subtraj */

/* this structure is filled when cmd_subtraj is parsed successfully */
struct cmd_subtraj_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	int32_t arg2;
	int32_t arg3;
	int32_t arg4;
	int32_t arg5;
};

static void cmd_subtraj_parsed(void *parsed_result, void *data)
{
//	struct cmd_subtraj_result *res = parsed_result;
	uint8_t err = 0;
	mainboard.flags |= DO_CS;
	dump_flags();
	motion_print_position();

/*
	if (strcmp_P(res->arg1, PSTR("diag3")) == 0) {
		err = strat_beginning(STRAT_CONF_FIRST_DIAG);
	} else if (strcmp_P(res->arg1, PSTR("diag5")) == 0) {
		err = strat_beginning(STRAT_CONF_SECOND_DIAG_5);
	} else if (strcmp_P(res->arg1, PSTR("diag6d")) == 0) {
		err = strat_beginning(STRAT_CONF_SECOND_DIAG_6);
	} else if (strcmp_P(res->arg1, PSTR("diag6o")) == 0) {
		err = strat_beginning(STRAT_CONF_SECOND_DIAG_6_OFFENSIVE);
	} else if (strcmp_P(res->arg1, PSTR("diag7d")) == 0) {
		err = strat_beginning(STRAT_CONF_SECOND_DIAG_7_DEFENSIVE);
	} else if (strcmp_P(res->arg1, PSTR("diag7o")) == 0) {
		err = strat_beginning(STRAT_CONF_SECOND_DIAG_7_OFFENSIVE);
	}
*/

//
//#define STRAT_CONF_FIRST_DIAG               0x01
//#define STRAT_CONF_SECOND_DIAG_5            0x02
//#define STRAT_CONF_SECOND_DIAG_6            0x04
//#define STRAT_CONF_SECOND_DIAG_6_OFFENSIVE  0x08
//#define STRAT_CONF_SECOND_DIAG_7_OFFENSIVE  0x10
//#define STRAT_CONF_SECOND_DIAG_7_DEFENSIVE  0x20

	printf_P(PSTR("substrat returned %s\r\n"), motion_get_err(err));
	trajectory_hardstop(&mainboard.traj);
}

static const prog_char str_subtraj_arg0[] = "subtraj";
static parse_token_string_t cmd_subtraj_arg0 =
TOKEN_STRING_INITIALIZER(struct cmd_subtraj_result, arg0, str_subtraj_arg0);
static const prog_char str_subtraj_arg1[] =
    "diag3#diag5#diag6d#diag6o#diag7d#diag7o";
static parse_token_string_t cmd_subtraj_arg1 =
TOKEN_STRING_INITIALIZER(struct cmd_subtraj_result, arg1, str_subtraj_arg1);

static const prog_char help_subtraj[] = "Test sub-trajectories";
parse_inst_t cmd_subtraj = {
	.f = cmd_subtraj_parsed,
	.data = NULL,
	.help_str = help_subtraj,
	.tokens = {
		(prog_void *)&cmd_subtraj_arg0,
		(prog_void *)&cmd_subtraj_arg1,
		NULL,
	},
};
