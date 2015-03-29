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

#include <ax12.h>
#include <uart.h>
#include <pwm_ng.h>
#include <clock_time.h>

#include <parse.h>
#include <parse_string.h>
#include <parse_num.h>

#include "main.h"

struct ax12_command {
	const const prog_char *name;
	uint8_t addr;
	uint8_t size;
};

static const char ax12_cmd_0[] PROGMEM = "cw_angle_limit";
static const char ax12_cmd_1[] PROGMEM = "ccw_angle_limit";
static const char ax12_cmd_2[] PROGMEM = "max_torque";
static const char ax12_cmd_3[] PROGMEM = "down_calibration";
static const char ax12_cmd_4[] PROGMEM = "up_calibration";
static const char ax12_cmd_5[] PROGMEM = "torque_limit";
static const char ax12_cmd_6[] PROGMEM = "position";
static const char ax12_cmd_7[] PROGMEM = "speed";
static const char ax12_cmd_8[] PROGMEM = "load";
static const char ax12_cmd_9[] PROGMEM = "moving_speed";
static const char ax12_cmd_10[] PROGMEM = "model";
static const char ax12_cmd_11[] PROGMEM = "goal_pos";
static const char ax12_cmd_12[] PROGMEM = "punch";
static const char ax12_cmd_13[] PROGMEM = "firmware";
static const char ax12_cmd_14[] PROGMEM = "id";
static const char ax12_cmd_15[] PROGMEM = "baudrate";
static const char ax12_cmd_16[] PROGMEM = "delay";
static const char ax12_cmd_17[] PROGMEM = "high_lim_temp";
static const char ax12_cmd_18[] PROGMEM = "low_lim_volt";
static const char ax12_cmd_19[] PROGMEM = "high_lim_volt";
static const char ax12_cmd_20[] PROGMEM = "status_return";
static const char ax12_cmd_21[] PROGMEM = "alarm_led";
static const char ax12_cmd_22[] PROGMEM = "alarm_shutdown";
static const char ax12_cmd_23[] PROGMEM = "torque_enable";
static const char ax12_cmd_24[] PROGMEM = "led";
static const char ax12_cmd_25[] PROGMEM = "cw_comp_margin";
static const char ax12_cmd_26[] PROGMEM = "ccw_comp_margin";
static const char ax12_cmd_27[] PROGMEM = "cw_comp_slope";
static const char ax12_cmd_28[] PROGMEM = "ccw_comp_slope";
static const char ax12_cmd_29[] PROGMEM = "voltage";
static const char ax12_cmd_30[] PROGMEM = "temp";
static const char ax12_cmd_31[] PROGMEM = "reginst";
static const char ax12_cmd_32[] PROGMEM = "moving";
static const char ax12_cmd_33[] PROGMEM = "lock";

static const struct ax12_command commands[] = {
	{ ax12_cmd_0, AA_CW_ANGLE_LIMIT_L, 16, },
	{ ax12_cmd_1, AA_CCW_ANGLE_LIMIT_L, 16 },
	{ ax12_cmd_2, AA_MAX_TORQUE_L, 16, },
	{ ax12_cmd_3, AA_DOWN_CALIBRATION_L, 16, },
	{ ax12_cmd_4, AA_UP_CALIBRATION_L, 16, },
	{ ax12_cmd_5, AA_TORQUE_LIMIT_L, 16, },
	{ ax12_cmd_6, AA_PRESENT_POSITION_L, 16, },
	{ ax12_cmd_7, AA_PRESENT_SPEED_L, 16, },
	{ ax12_cmd_8, AA_PRESENT_LOAD_L, 16, },
	{ ax12_cmd_9, AA_MOVING_SPEED_L, 16, },
	{ ax12_cmd_10, AA_MODEL_NUMBER_L, 16, },
	{ ax12_cmd_11, AA_GOAL_POSITION_L, 16, },
	{ ax12_cmd_12, AA_PUNCH_L, 16, },
	{ ax12_cmd_13, AA_FIRMWARE, 8, },
	{ ax12_cmd_14, AA_ID, 8, },
	{ ax12_cmd_15, AA_BAUD_RATE, 8, },
	{ ax12_cmd_16, AA_DELAY_TIME, 8, },
	{ ax12_cmd_17, AA_HIGHEST_LIMIT_TEMP, 8, },
	{ ax12_cmd_18, AA_LOWEST_LIMIT_VOLTAGE, 8, },
	{ ax12_cmd_19, AA_HIGHEST_LIMIT_VOLTAGE, 8, },
	{ ax12_cmd_20, AA_STATUS_RETURN_LEVEL, 8, },
	{ ax12_cmd_21, AA_ALARM_LED, 8, },
	{ ax12_cmd_22, AA_ALARM_SHUTDOWN, 8, },
	{ ax12_cmd_23, AA_TORQUE_ENABLE, 8, },
	{ ax12_cmd_24, AA_LED, 8, },
	{ ax12_cmd_25, AA_CW_COMPLIANCE_MARGIN, 8, },
	{ ax12_cmd_26, AA_CCW_COMPLIANCE_MARGIN, 8, },
	{ ax12_cmd_27, AA_CW_COMPLIANCE_SLOPE, 8, },
	{ ax12_cmd_28, AA_CCW_COMPLIANCE_SLOPE, 8, },
	{ ax12_cmd_29, AA_PRESENT_VOLTAGE, 8, },
	{ ax12_cmd_30, AA_PRESENT_TEMP, 8, },
	{ ax12_cmd_31, AA_PRESENT_REGINST, 8, },
	{ ax12_cmd_32, AA_MOVING, 8, },
	{ ax12_cmd_33, AA_LOCK, 8, },
};

static const struct ax12_command *ax12_command_info(const char *name)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(commands); ++i) {
		if (strcmp_P(name, commands[i].name) == 0)
			return &commands[i];
	}

	return NULL;
}

/* -----------------------------------------------------------------------------
 * ax12_stress - Stress test an AX12 servo
 *
 * Usage: ax12_stress id iter
 */

struct cmd_ax12_stress_result {
	fixed_string_t arg0;
	uint8_t id;
	uint16_t iter;
};

static void cmd_ax12_stress_parsed(void *parsed_result, void *data)
{
	struct cmd_ax12_stress_result *res = parsed_result;
	microseconds t = time_get_us2();
	unsigned int nb_errs = 0;
	unsigned int i;
	uint8_t val;

	for (i = 0; i < res->iter; i++) {
		if (AX12_read_byte(&gen.ax12, res->id, AA_ID, &val) != 0)
			nb_errs++;
	}

	printf_P(PSTR("%u errors / %u\r\n"), nb_errs, res->iter);
	t = (time_get_us2() - t) / 1000;
	printf_P(PSTR("Test done in %d ms\r\n"), (int)t);
}

static const const prog_char str_ax12_stress_arg0[] = "ax12_stress";
static const parse_token_string_t cmd_ax12_stress_arg0 =
TOKEN_STRING_INITIALIZER(struct cmd_ax12_stress_result, arg0,
			 str_ax12_stress_arg0);
static const parse_token_num_t cmd_ax12_stress_id =
TOKEN_NUM_INITIALIZER(struct cmd_ax12_stress_result, id, UINT8);
static const parse_token_num_t cmd_ax12_stress_iter =
TOKEN_NUM_INITIALIZER(struct cmd_ax12_stress_result, iter, UINT16);

static const const prog_char help_ax12_stress[] = "Stress an AX12 with 'read id' commands (id, iter)";
parse_inst_t cmd_ax12_stress = {
	.f = cmd_ax12_stress_parsed,
	.data = NULL,
	.help_str = help_ax12_stress,
	.tokens = {
		(prog_void *)&cmd_ax12_stress_arg0,
		(prog_void *)&cmd_ax12_stress_id,
		(prog_void *)&cmd_ax12_stress_iter,
		NULL,
	},
};

/* -----------------------------------------------------------------------------
 * ax12_baudrate - Set the AX12 port baudrate
 *
 * Usage: ax12_baudrate rate
 */

struct cmd_baudrate_result {
	fixed_string_t arg0;
	uint32_t arg1;
};

static void cmd_baudrate_parsed(void *parsed_result, void *data)
{
	struct cmd_baudrate_result *res = parsed_result;
	struct uart_config c;

	printf_P(PSTR("%u\r\n"), UBRR2);
	uart_getconf(2, &c);
	c.baudrate = res->arg1;
	uart_setconf(2, &c);
	printf_P(PSTR("%u\r\n"), UBRR2);
}

static const const prog_char str_baudrate_arg0[] = "ax12_baudrate";
static const parse_token_string_t cmd_baudrate_arg0 =
TOKEN_STRING_INITIALIZER(struct cmd_baudrate_result, arg0, str_baudrate_arg0);
static const parse_token_num_t cmd_baudrate_arg1 =
TOKEN_NUM_INITIALIZER(struct cmd_baudrate_result, arg1, UINT32);

static const const prog_char help_baudrate[] = "Change AX12 baudrate";
parse_inst_t cmd_ax12_baudrate = {
	.f = cmd_baudrate_parsed,
	.data = NULL,
	.help_str = help_baudrate,
	.tokens = {
		(prog_void *)&cmd_baudrate_arg0,
		(prog_void *)&cmd_baudrate_arg1,
		NULL,
	},
};

/* -----------------------------------------------------------------------------
 * ax12_read - Read an AX12 register
 *
 * Usage: ax12_read id register
 */

struct cmd_read_result {
	fixed_string_t arg0;
	uint8_t id;
	fixed_string_t reg;
};

static void cmd_read_parsed(void *parsed_result, void *data)
{
	struct cmd_read_result *res = parsed_result;
	const struct ax12_command *cmd = ax12_command_info(res->reg);
	uint16_t val16;
	uint8_t val8;
	uint8_t ret;

	if (cmd->size == 16) {
		ret = AX12_read_int(&gen.ax12, res->id, cmd->addr, &val16);
	} else {
		ret = AX12_read_byte(&gen.ax12, res->id, cmd->addr, &val8);
		val16 = val8;
	}

	if (ret)
		printf_P(PSTR("AX12 error %d\r\n"), (int8_t)ret);
	else
		printf_P(PSTR("%s: %d [0x%.4x]\r\n"), res->reg, val16, val16);
}

static const const prog_char str_read_arg0[] = "ax12_read";
static const parse_token_string_t cmd_read_arg0 =
TOKEN_STRING_INITIALIZER(struct cmd_read_result, arg0, str_read_arg0);
static const const prog_char str_read_reg[] =
    "moving_speed#model#goal_pos#cw_angle_limit#ccw_angle_limit#"
    "max_torque#down_calibration#up_calibration#torque_limit#"
    "position#speed#load#punch#"
    "id#firmware#baudrate#delay#high_lim_temp#"
    "low_lim_volt#high_lim_volt#status_return#alarm_led#"
    "alarm_shutdown#torque_enable#led#cw_comp_margin#"
    "ccw_comp_margin#cw_comp_slope#ccw_comp_slope#"
    "voltage#temp#reginst#moving#lock";
static const parse_token_string_t cmd_read_reg =
TOKEN_STRING_INITIALIZER(struct cmd_read_result, reg, str_read_reg);
static const parse_token_num_t cmd_read_id =
TOKEN_NUM_INITIALIZER(struct cmd_read_result, id, UINT8);

static const const prog_char help_read[] = "Read an AX12 register (id, reg)";
parse_inst_t cmd_ax12_read = {
	.f = cmd_read_parsed,
	.data = NULL,
	.help_str = help_read,
	.tokens = {
		(prog_void *)&cmd_read_arg0,
		(prog_void *)&cmd_read_id,
		(prog_void *)&cmd_read_reg,
		NULL,
	},
};

/* -----------------------------------------------------------------------------
 * ax12_write - Write an AX12 register
 *
 * Usage: ax12_write id register value
 */

struct cmd_write_result {
	fixed_string_t arg0;
	uint8_t id;
	fixed_string_t reg;
	uint16_t val;
};

static void cmd_write_parsed(void *parsed_result, void *data)
{
	struct cmd_write_result *res = parsed_result;
	const struct ax12_command *cmd = ax12_command_info(res->reg);
	uint8_t ret;

	printf_P(PSTR("writing to %s (%u) @%u: %u (0x%04x)\r\n"), res->reg,
		 cmd->addr, res->id, res->val, res->val);

	if (cmd->size == 16)
		ret = AX12_write_int(&gen.ax12, res->id, cmd->addr, res->val);
	else
		ret = AX12_write_byte(&gen.ax12, res->id, cmd->addr, res->val);

	if (ret)
		printf_P(PSTR("AX12 error %d\r\n"), (int8_t)ret);
}

static const const prog_char str_write_arg0[] = "ax12_write";
static const parse_token_string_t cmd_write_arg0 =
TOKEN_STRING_INITIALIZER(struct cmd_write_result, arg0, str_write_arg0);
static const const prog_char str_write_reg[] =
    "moving_speed#goal_pos#cw_angle_limit#ccw_angle_limit#"
    "max_torque#torque_limit#punch#"
    "id#baudrate#delay#high_lim_temp#"
    "low_lim_volt#high_lim_volt#status_return#alarm_led#"
    "alarm_shutdown#torque_enable#led#cw_comp_margin#"
    "ccw_comp_margin#cw_comp_slope#ccw_comp_slope#" "reginst#lock";
static const parse_token_string_t cmd_write_reg =
TOKEN_STRING_INITIALIZER(struct cmd_write_result, reg, str_write_reg);
static const parse_token_num_t cmd_write_id =
TOKEN_NUM_INITIALIZER(struct cmd_write_result, id, UINT8);
static const parse_token_num_t cmd_write_val =
TOKEN_NUM_INITIALIZER(struct cmd_write_result, val, UINT16);

static const const prog_char help_write[] = "Write an AX12 register (id, reg, value)";
parse_inst_t cmd_ax12_write = {
	.f = cmd_write_parsed,
	.data = NULL,
	.help_str = help_write,
	.tokens = {
		(prog_void *)&cmd_write_arg0,
		(prog_void *)&cmd_write_id,
		(prog_void *)&cmd_write_reg,
		(prog_void *)&cmd_write_val,
		NULL,
	},
};
