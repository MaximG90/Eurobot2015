/*
 *  Copyright Droids Corporation (2008)
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

#include <pid.h>
#include <quadramp.h>
#include <control_system_manager.h>
#include <vect_base.h>
#include <lines.h>
#include <polygon.h>
#include <blocking_detection_manager.h>

#include <parse.h>
#include <parse_string.h>
#include <parse_num.h>

#include "main.h"
#include "cs.h"
#include "cmdline.h"

struct csb_list {
	const const prog_char *name;
	struct cs_block *csb;
};

static const prog_char csb_angle_str[] = "angle";
static const prog_char csb_distance_str[] = "distance";
static struct csb_list csb_list[] = {
	{ .name = csb_angle_str, .csb = &mainboard.angle},
	{ .name = csb_distance_str, .csb = &mainboard.distance},
};

struct cmd_cs_result {
	fixed_string_t cmdname;
	fixed_string_t csname;
};

struct cmd_csb_show_result {
	struct cmd_cs_result cs;
	fixed_string_t show;
};

/* token to be used for all cs-related commands */
static const prog_char str_csb_name[] = "angle#distance";
static parse_token_string_t cmd_csb_name_tok =
TOKEN_STRING_INITIALIZER(struct cmd_cs_result, csname, str_csb_name);

static struct cs_block *cs_from_name(const char *name)
{
	int i;

	for (i = 0; i < (sizeof(csb_list) / sizeof(*csb_list)); i++) {
		if (!strcmp_P(name, csb_list[i].name))
			return csb_list[i].csb;
	}

	return NULL;
}

/* -----------------------------------------------------------------------------
 * gain - Show or modify control system PID gains
 *
 * Usage: gain angle|distance show|values
 *
 * show: Display PID gains
 * values: p i d
 */

struct cmd_gain_result {
	struct cmd_cs_result cs;
	int16_t p;
	int16_t i;
	int16_t d;
};

static void cmd_gain_parsed(void *parsed_result, void *show)
{
	struct cmd_gain_result *res = parsed_result;
	struct cs_block *csb;

	csb = cs_from_name(res->cs.csname);
	if (csb == NULL) {
		printf_P(PSTR("null csb\r\n"));
		return;
	}

	if (!show)
		pid_set_gains(&csb->pid, res->p, res->i, res->d);

	printf_P(PSTR("%s %s %d %d %d\r\n"),
		 res->cs.cmdname,
		 res->cs.csname,
		 pid_get_gain_P(&csb->pid),
		 pid_get_gain_I(&csb->pid), pid_get_gain_D(&csb->pid));
}

static const prog_char str_gain_arg0[] = "gain";
static parse_token_string_t cmd_gain_arg0 =
TOKEN_STRING_INITIALIZER(struct cmd_gain_result, cs.cmdname, str_gain_arg0);
static parse_token_num_t cmd_gain_p =
TOKEN_NUM_INITIALIZER(struct cmd_gain_result, p, INT16);
static parse_token_num_t cmd_gain_i =
TOKEN_NUM_INITIALIZER(struct cmd_gain_result, i, INT16);
static parse_token_num_t cmd_gain_d =
TOKEN_NUM_INITIALIZER(struct cmd_gain_result, d, INT16);

static const prog_char help_gain[] = "Set gain values for PID";
parse_inst_t cmd_gain = {
	.f = cmd_gain_parsed,
	.data = NULL,
	.help_str = help_gain,
	.tokens = {
		(prog_void *)&cmd_gain_arg0,
		(prog_void *)&cmd_csb_name_tok,
		(prog_void *)&cmd_gain_p,
		(prog_void *)&cmd_gain_i,
		(prog_void *)&cmd_gain_d,
		NULL,
	},
};

static const prog_char str_gain_show_arg[] = "show";
static parse_token_string_t cmd_gain_show_arg =
TOKEN_STRING_INITIALIZER(struct cmd_csb_show_result, show, str_gain_show_arg);

static const prog_char help_gain_show[] = "Show gain values for PID";
parse_inst_t cmd_gain_show = {
	.f = cmd_gain_parsed,
	.data = (void *)1,
	.help_str = help_gain_show,
	.tokens = {
		(prog_void *)&cmd_gain_arg0,
		(prog_void *)&cmd_csb_name_tok,
		(prog_void *)&cmd_gain_show_arg,
		NULL,
	},
};

/* -----------------------------------------------------------------------------
 * speed - Show or modify speed value for the ramp filter
 *
 * Usage: speed angle|distance show|value
 *
 * show: Display the ramp filter speed value
 * value: Set the ramp filter speed value
 */

struct cmd_speed_result {
	struct cmd_cs_result cs;
	uint16_t s;
};

static void cmd_speed_parsed(void *parsed_result, void *show)
{
	struct cmd_speed_result *res = parsed_result;
	struct cs_block *csb;

	csb = cs_from_name(res->cs.csname);
	if (csb == NULL) {
		printf_P(PSTR("null csb\r\n"));
		return;
	}

#if notyet
	if (!show)
		ramp_set_vars(&csb->ramp, res->s, res->s);	/* set speed */

	printf_P(PSTR("%s %lu\r\n"), res->cs.csname, ext.r_b.var_pos);
#else
	printf_P(PSTR("TODO\r\n"));
#endif
}

static const prog_char str_speed_arg0[] = "speed";
static parse_token_string_t cmd_speed_arg0 =
TOKEN_STRING_INITIALIZER(struct cmd_speed_result, cs.cmdname, str_speed_arg0);
static parse_token_num_t cmd_speed_s =
TOKEN_NUM_INITIALIZER(struct cmd_speed_result, s, UINT16);

static const prog_char help_speed[] = "Set speed values for ramp filter";
parse_inst_t cmd_speed = {
	.f = cmd_speed_parsed,
	.data = NULL,
	.help_str = help_speed,
	.tokens = {
		(prog_void *)&cmd_speed_arg0,
		(prog_void *)&cmd_csb_name_tok,
		(prog_void *)&cmd_speed_s,
		NULL,
	},
};

static const prog_char str_speed_show_arg[] = "show";
static parse_token_string_t cmd_speed_show_arg =
TOKEN_STRING_INITIALIZER(struct cmd_csb_show_result, show,
			 str_speed_show_arg);

static const prog_char help_speed_show[] = "Show speed values for ramp filter";
parse_inst_t cmd_speed_show = {
	.f = cmd_speed_parsed,
	.data = (void *)1,
	.help_str = help_speed_show,
	.tokens = {
		(prog_void *)&cmd_speed_arg0,
		(prog_void *)&cmd_csb_name_tok,
		(prog_void *)&cmd_speed_show_arg,
		NULL,
	},
};

/* -----------------------------------------------------------------------------
 * derivative_filter - Show or modify control system derivative filter settings
 *
 * Usage: derivative_filter angle|distance show|value
 *
 * show: Display the control system PID derivative filter value.
 * value: Set the control system PID derivative filter value.
 */

struct cmd_derivate_filter_result {
	struct cmd_cs_result cs;
	uint8_t size;
};

static void cmd_derivate_filter_parsed(void *parsed_result, void *show)
{
	struct cmd_derivate_filter_result *res = parsed_result;
	struct cs_block *csb;

	csb = cs_from_name(res->cs.csname);
	if (csb == NULL) {
		printf_P(PSTR("null csb\r\n"));
		return;
	}

	if (!show)
		pid_set_derivate_filter(&csb->pid, res->size);

	printf_P(PSTR("%s %s %u\r\n"),
		 res->cs.cmdname,
		 res->cs.csname, pid_get_derivate_filter(&csb->pid));
}

static const prog_char str_derivate_filter_arg0[] = "derivate_filter";
static parse_token_string_t cmd_derivate_filter_arg0 =
TOKEN_STRING_INITIALIZER(struct cmd_derivate_filter_result, cs.cmdname,
			 str_derivate_filter_arg0);
static parse_token_num_t cmd_derivate_filter_size =
TOKEN_NUM_INITIALIZER(struct cmd_derivate_filter_result, size, UINT32);

static const prog_char help_derivate_filter[] =
    "Set derivate_filter values for PID (size)";
parse_inst_t cmd_derivate_filter = {
	.f = cmd_derivate_filter_parsed,
	.data = (void *)1,
	.help_str = help_derivate_filter,
	.tokens = {
		(prog_void *)&cmd_derivate_filter_arg0,
		(prog_void *)&cmd_csb_name_tok,
		(prog_void *)&cmd_derivate_filter_size,
		NULL,
	},
};

static const prog_char str_derivate_filter_show_arg[] = "show";
static parse_token_string_t cmd_derivate_filter_show_arg =
TOKEN_STRING_INITIALIZER(struct cmd_csb_show_result, show,
			 str_derivate_filter_show_arg);

static const prog_char help_derivate_filter_show[] = "Show derivate_filter values for PID";
parse_inst_t cmd_derivate_filter_show = {
	.f = cmd_derivate_filter_parsed,
	.data = NULL,
	.help_str = help_derivate_filter_show,
	.tokens = {
		(prog_void *)&cmd_derivate_filter_arg0,
		(prog_void *)&cmd_csb_name_tok,
		(prog_void *)&cmd_derivate_filter_show_arg,
		NULL,
	},
};

/* -----------------------------------------------------------------------------
 * consign - Set control system set points
 *
 * Usage: consign angle|distance value
 *
 * Set the control system set point to the given integer value.
 */

struct cmd_consign_result {
	struct cmd_cs_result cs;
	int32_t p;
};

static void cmd_consign_parsed(void *parsed_result, void *data)
{
	struct cmd_consign_result *res = parsed_result;
	struct cs_block *csb;

	csb = cs_from_name(res->cs.csname);
	if (csb == NULL) {
		printf_P(PSTR("null csb\r\n"));
		return;
	}

	cs_set_consign(&csb->cs, res->p);
}

static const prog_char str_consign_arg0[] = "consign";
static parse_token_string_t cmd_consign_arg0 =
TOKEN_STRING_INITIALIZER(struct cmd_consign_result, cs.cmdname,
			 str_consign_arg0);
static parse_token_num_t cmd_consign_p =
TOKEN_NUM_INITIALIZER(struct cmd_consign_result, p, INT32);

static const prog_char help_consign[] = "Set consign value";
parse_inst_t cmd_consign = {
	.f = cmd_consign_parsed,
	.data = NULL,
	.help_str = help_consign,
	.tokens = {
		(prog_void *)&cmd_consign_arg0,
		(prog_void *)&cmd_csb_name_tok,
		(prog_void *)&cmd_consign_p,
		NULL,
	},
};

/* -----------------------------------------------------------------------------
 * maximum - Show or modify control system PID limits
 *
 * Usage: maximum angle|distance show|values
 *
 * show: Display PID limits
 * values: in i out
 */

struct cmd_maximum_result {
	struct cmd_cs_result cs;
	uint32_t in;
	uint32_t i;
	uint32_t out;
};

static void cmd_maximum_parsed(void *parsed_result, void *show)
{
	struct cmd_maximum_result *res = parsed_result;
	struct cs_block *csb;

	csb = cs_from_name(res->cs.csname);
	if (csb == NULL) {
		printf_P(PSTR("null csb\r\n"));
		return;
	}

	if (!show)
		pid_set_maximums(&csb->pid, res->in, res->i, res->out);

	printf_P(PSTR("maximum %s %lu %lu %lu\r\n"),
		 res->cs.csname,
		 pid_get_max_in(&csb->pid),
		 pid_get_max_I(&csb->pid), pid_get_max_out(&csb->pid));
}

static const prog_char str_maximum_arg0[] = "maximum";
static parse_token_string_t cmd_maximum_arg0 =
TOKEN_STRING_INITIALIZER(struct cmd_maximum_result, cs.cmdname,
			 str_maximum_arg0);
static parse_token_num_t cmd_maximum_in =
TOKEN_NUM_INITIALIZER(struct cmd_maximum_result, in, UINT32);
static parse_token_num_t cmd_maximum_i =
TOKEN_NUM_INITIALIZER(struct cmd_maximum_result, i, UINT32);
static parse_token_num_t cmd_maximum_out =
TOKEN_NUM_INITIALIZER(struct cmd_maximum_result, out, UINT32);

static const prog_char help_maximum[] = "Set maximum values for PID (in, I, out)";
parse_inst_t cmd_maximum = {
	.f = cmd_maximum_parsed,
	.data = NULL,
	.help_str = help_maximum,
	.tokens = {
		(prog_void *)&cmd_maximum_arg0,
		(prog_void *)&cmd_csb_name_tok,
		(prog_void *)&cmd_maximum_in,
		(prog_void *)&cmd_maximum_i,
		(prog_void *)&cmd_maximum_out,
		NULL,
	},
};

static const prog_char str_maximum_show_arg[] = "show";
static parse_token_string_t cmd_maximum_show_arg =
TOKEN_STRING_INITIALIZER(struct cmd_csb_show_result, show,
			 str_maximum_show_arg);

static const prog_char help_maximum_show[] = "Show maximum values for PID";
parse_inst_t cmd_maximum_show = {
	.f = cmd_maximum_parsed,
	.data = (void *)1,
	.help_str = help_maximum_show,
	.tokens = {
		(prog_void *)&cmd_maximum_arg0,
		(prog_void *)&cmd_csb_name_tok,
		(prog_void *)&cmd_maximum_show_arg,
		NULL,
	},
};

/* -----------------------------------------------------------------------------
 * quadramp - Show or modify control system quadramp parameters
 *
 * Usage: quadramp angle|distance show|values
 *
 * show: Display quadramp filter status
 * values: ap an sp sn
 */

struct cmd_quadramp_result {
	struct cmd_cs_result cs;
	uint32_t ap;
	uint32_t an;
	uint32_t sp;
	uint32_t sn;
};

static void cmd_quadramp_parsed(void *parsed_result, void *show)
{
	struct cmd_quadramp_result *res = parsed_result;
	struct cs_block *csb;

	csb = cs_from_name(res->cs.csname);
	if (csb == NULL) {
		printf_P(PSTR("null csb\r\n"));
		return;
	}

	if (!show) {
		quadramp_set_1st_order_vars(&csb->qr, res->sp, res->sn);
		quadramp_set_2nd_order_vars(&csb->qr, res->ap, res->an);
	}

	printf_P(PSTR("quadramp %s %ld %ld %ld %ld\r\n"),
		 res->cs.csname,
		 csb->qr.var_2nd_ord_pos,
		 csb->qr.var_2nd_ord_neg,
		 csb->qr.var_1st_ord_pos, csb->qr.var_1st_ord_neg);
}

static const prog_char str_quadramp_arg0[] = "quadramp";
static parse_token_string_t cmd_quadramp_arg0 =
TOKEN_STRING_INITIALIZER(struct cmd_quadramp_result, cs.cmdname,
			 str_quadramp_arg0);
static parse_token_num_t cmd_quadramp_ap =
TOKEN_NUM_INITIALIZER(struct cmd_quadramp_result, ap, UINT32);
static parse_token_num_t cmd_quadramp_an =
TOKEN_NUM_INITIALIZER(struct cmd_quadramp_result, an, UINT32);
static parse_token_num_t cmd_quadramp_sp =
TOKEN_NUM_INITIALIZER(struct cmd_quadramp_result, sp, UINT32);
static parse_token_num_t cmd_quadramp_sn =
TOKEN_NUM_INITIALIZER(struct cmd_quadramp_result, sn, UINT32);

static const prog_char help_quadramp[] = "Set quadramp values (acc+, acc-, speed+, speed-)";
parse_inst_t cmd_quadramp = {
	.f = cmd_quadramp_parsed,
	.data = NULL,
	.help_str = help_quadramp,
	.tokens = {
		(prog_void *)&cmd_quadramp_arg0,
		(prog_void *)&cmd_csb_name_tok,
		(prog_void *)&cmd_quadramp_ap,
		(prog_void *)&cmd_quadramp_an,
		(prog_void *)&cmd_quadramp_sp,
		(prog_void *)&cmd_quadramp_sn,
		NULL,
	},
};

static const prog_char str_quadramp_show_arg[] = "show";
static parse_token_string_t cmd_quadramp_show_arg =
TOKEN_STRING_INITIALIZER(struct cmd_csb_show_result, show,
			 str_quadramp_show_arg);

static const prog_char help_quadramp_show[] = "Get quadramp values for control system";
parse_inst_t cmd_quadramp_show = {
	.f = cmd_quadramp_parsed,
	.data = (void *)1,
	.help_str = help_quadramp_show,
	.tokens = {
		(prog_void *)&cmd_quadramp_arg0,
		(prog_void *)&cmd_csb_name_tok,
		(prog_void *)&cmd_quadramp_show_arg,
		NULL,
	},
};

/* -----------------------------------------------------------------------------
 * cs_status - Show control system status
 *
 * Usage: cs_status angle|distance pid_show|pid_loop_show|show|loop_show|on|off
 *
 * pid_show: Display PID status
 * pid_loop_show: Display PID status in a loop
 * show: Display constrol system status
 * loop_show: Display control system status in a loop
 * on: Enable the control system
 * off: Disable the control system
 */

struct cmd_cs_status_result {
	struct cmd_cs_result cs;
	fixed_string_t arg;
};

static void cmd_cs_status_parsed(void *parsed_result, void *data)
{
	struct cmd_cs_status_result *res = parsed_result;
	struct cs_block *csb;
	uint8_t print_pid = 0, print_cs = 0;
	uint8_t loop = 0;

	csb = cs_from_name(res->cs.csname);
	if (csb == NULL) {
		printf_P(PSTR("null csb\r\n"));
		return;
	}

	if (strcmp_P(res->arg, PSTR("on")) == 0) {
		csb->on = 1;
		printf_P(PSTR("%s is on\r\n"), res->cs.csname);
		return;
	}

	if (strcmp_P(res->arg, PSTR("off")) == 0) {
		csb->on = 0;
		printf_P(PSTR("%s is off\r\n"), res->cs.csname);
		return;
	}

	if (strcmp_P(res->arg, PSTR("show")) == 0) {
		print_cs = 1;
	} else if (strcmp_P(res->arg, PSTR("loop_show")) == 0) {
		loop = 1;
		print_cs = 1;
	} else if (strcmp_P(res->arg, PSTR("pid_show")) == 0) {
		print_pid = 1;
	} else if (strcmp_P(res->arg, PSTR("pid_loop_show")) == 0) {
		print_pid = 1;
		loop = 1;
	}

	printf_P(PSTR("%s cs is %s\r\n"), res->cs.csname,
		 csb->on ? "on" : "off");
	do {
		if (print_cs)
			dump_cs(res->cs.csname, &csb->cs);
		if (print_pid)
			dump_pid(res->cs.csname, &csb->pid);
		wait_ms(100);
	} while (loop && !cmdline_keypressed());
}

static const prog_char str_cs_status_arg0[] = "cs_status";
static parse_token_string_t cmd_cs_status_arg0 =
TOKEN_STRING_INITIALIZER(struct cmd_cs_status_result, cs.cmdname,
			 str_cs_status_arg0);
static const prog_char str_cs_status_arg[] = "pid_show#pid_loop_show#show#loop_show#on#off";
static parse_token_string_t cmd_cs_status_arg =
TOKEN_STRING_INITIALIZER(struct cmd_cs_status_result, arg, str_cs_status_arg);

static const prog_char help_cs_status[] = "Show cs status";
parse_inst_t cmd_cs_status = {
	.f = cmd_cs_status_parsed,
	.data = NULL,
	.help_str = help_cs_status,
	.tokens = {
		(prog_void *)&cmd_cs_status_arg0,
		(prog_void *)&cmd_csb_name_tok,
		(prog_void *)&cmd_cs_status_arg,
		NULL,
	},
};

/* -----------------------------------------------------------------------------
 * blocking - Control blocking detection
 *
 * Usage: blocking angle|distance show
 * 	  blocking angle|distance k1 k2 i cpt
 *
 * show: Display the current blocking detection values
 */

struct cmd_blocking_i_result {
	struct cmd_cs_result cs;
	int32_t k1;
	int32_t k2;
	uint32_t i;
	uint16_t cpt;
};

static void cmd_blocking_i_parsed(void *parsed_result, void *show)
{
	struct cmd_blocking_i_result *res = parsed_result;

	struct cs_block *csb;

	csb = cs_from_name(res->cs.csname);
	if (csb == NULL) {
		printf_P(PSTR("null csb\r\n"));
		return;
	}

	if (!show)
		bd_set_current_thresholds(&csb->bd, res->k1, res->k2,
					  res->i, res->cpt);

	printf_P(PSTR("%s %s %ld %ld %ld %d\r\n"),
		 res->cs.cmdname,
		 res->cs.csname,
		 csb->bd.k1, csb->bd.k2, csb->bd.i_thres, csb->bd.cpt_thres);
}

static const prog_char str_blocking_i_arg0[] = "blocking";
static parse_token_string_t cmd_blocking_i_arg0 =
TOKEN_STRING_INITIALIZER(struct cmd_blocking_i_result, cs.cmdname,
			 str_blocking_i_arg0);
static parse_token_num_t cmd_blocking_i_k1 =
TOKEN_NUM_INITIALIZER(struct cmd_blocking_i_result, k1, INT32);
static parse_token_num_t cmd_blocking_i_k2 =
TOKEN_NUM_INITIALIZER(struct cmd_blocking_i_result, k2, INT32);
static parse_token_num_t cmd_blocking_i_i =
TOKEN_NUM_INITIALIZER(struct cmd_blocking_i_result, i, UINT32);
static parse_token_num_t cmd_blocking_i_cpt =
TOKEN_NUM_INITIALIZER(struct cmd_blocking_i_result, cpt, UINT16);

static const prog_char help_blocking_i[] = "Set blocking detection values (k1, k2, i, cpt)";
parse_inst_t cmd_blocking_i = {
	.f = cmd_blocking_i_parsed,
	.data = NULL,
	.help_str = help_blocking_i,
	.tokens = {
		(prog_void *)&cmd_blocking_i_arg0,
		(prog_void *)&cmd_csb_name_tok,
		(prog_void *)&cmd_blocking_i_k1,
		(prog_void *)&cmd_blocking_i_k2,
		(prog_void *)&cmd_blocking_i_i,
		(prog_void *)&cmd_blocking_i_cpt,
		NULL,
	},
};

static const prog_char str_blocking_i_show_arg[] = "show";
static parse_token_string_t cmd_blocking_i_show_arg =
TOKEN_STRING_INITIALIZER(struct cmd_csb_show_result, show,
			 str_blocking_i_show_arg);

static const prog_char help_blocking_i_show[] = "Show blocking detection values";
parse_inst_t cmd_blocking_i_show = {
	.f = cmd_blocking_i_parsed,
	.data = (void *)1,
	.help_str = help_blocking_i_show,
	.tokens = {
		(prog_void *)&cmd_blocking_i_arg0,
		(prog_void *)&cmd_csb_name_tok,
		(prog_void *)&cmd_blocking_i_show_arg,
		NULL,
	},
};
