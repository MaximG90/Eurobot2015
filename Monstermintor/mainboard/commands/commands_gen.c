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

#include <adc.h>
#include <encoders_hctl.h>
#include <pwm_ng.h>
#include <clock_time.h>
#include <spi.h>

#include <scheduler.h>

#include <parse.h>
#include <parse_string.h>
#include <parse_num.h>

#include <diagnostic.h>

#include "bootloader.h"
#include "cmdline.h"
#include "main.h"
#include "sensor.h"
#include "tones.h"

/* -----------------------------------------------------------------------------
 * reset - Reset the board
 *
 * Usage: reset
 */

struct cmd_reset_result {
	fixed_string_t arg0;
};

static void cmd_reset_parsed(void *parsed_result, void *data)
{
	reset();
}

static const prog_char str_reset_arg0[] = "reset";
static parse_token_string_t cmd_reset_arg0 =
TOKEN_STRING_INITIALIZER(struct cmd_reset_result, arg0, str_reset_arg0);

static const prog_char help_reset[] = "Reset the board";
parse_inst_t cmd_reset = {
	.f = cmd_reset_parsed,
	.data = NULL,
	.help_str = help_reset,
	.tokens = {
		(prog_void *)&cmd_reset_arg0,
		NULL,
	},
};

/* -----------------------------------------------------------------------------
 * bootloader - Launch the boot loader
 *
 * Usage: bootloader
 */

struct cmd_bootloader_result {
	fixed_string_t arg0;
};

static void cmd_bootloader_parsed(void *parsed_result, void *data)
{
	bootloader();
}

static const prog_char str_bootloader_arg0[] = "bootloader";
static parse_token_string_t cmd_bootloader_arg0 =
TOKEN_STRING_INITIALIZER(struct cmd_bootloader_result, arg0,
			 str_bootloader_arg0);

static const prog_char help_bootloader[] = "Launch the bootloader";
parse_inst_t cmd_bootloader = {
	.f = cmd_bootloader_parsed,
	.data = NULL,
	.help_str = help_bootloader,
	.tokens = {
		(prog_void *)&cmd_bootloader_arg0,
		NULL,
	},
};

/* -----------------------------------------------------------------------------
 * buzzer - Play a tone on the buzzer
 *
 * Usage: buzzer tone time
 * tone: DO|SOL|LA
 * time: duration in milliseconds
 */

struct cmd_buzzer_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	int16_t arg2;
};

static void cmd_buzzer_parsed(void *parsed_result, void *data)
{
	struct cmd_buzzer_result *res = parsed_result;
	uint8_t tone;

#ifndef HOST_VERSION
	if (!strcmp_P(res->arg1, PSTR("DO")))
		tone = DO_TONE;
	else if (!strcmp_P(res->arg1, PSTR("SOL")))
		tone = SOL_TONE;
	else if (!strcmp_P(res->arg1, PSTR("LA")))
		tone = LA_TONE;
	else
		tone = NONE_TONE;

	if (tone != NONE_TONE) {
		//PORTJ |= tone;
		printf_P(PSTR("wait for %d ms\n"), res->arg2);
		wait_ms(res->arg2);
		//PORTJ &= NONE_TONE;
	}
#endif
	printf_P(PSTR("done\r\n"));
}

static const prog_char str_buzzer_arg0[] = "buzzer";
static parse_token_string_t cmd_buzzer_arg0 =
TOKEN_STRING_INITIALIZER(struct cmd_buzzer_result, arg0, str_buzzer_arg0);
static const prog_char str_buzzer_arg1[] = "DO#SOL#LA";
static parse_token_string_t cmd_buzzer_arg1 =
TOKEN_STRING_INITIALIZER(struct cmd_buzzer_result, arg1, str_buzzer_arg1);
static parse_token_num_t cmd_buzzer_arg2 =
TOKEN_NUM_INITIALIZER(struct cmd_buzzer_result, arg2, INT16);

static const prog_char help_buzzer[] = "buzzer tones (DO/SOL/LA) for xx ms";
parse_inst_t cmd_buzzer = {
	.f = cmd_buzzer_parsed,
	.data = NULL,
	.help_str = help_buzzer,
	.tokens = {
		(prog_void *)&cmd_buzzer_arg0,
		(prog_void *)&cmd_buzzer_arg1,
		(prog_void *)&cmd_buzzer_arg2,
		NULL,
	},
};

/* -----------------------------------------------------------------------------
 * encoders - Show encoders values
 *
 * Usage: encoders show|hex|reset
 *
 * show: Display cached encoders values
 * display: Display the raw encoders values
 * reset: Reset the encoders values to 0
 */

struct cmd_encoders_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
};

static void cmd_encoders_parsed(void *parsed_result, void *data)
{
	struct cmd_encoders_result *res = parsed_result;
	uint8_t lsb[4];
	uint8_t msb[4];
	uint8_t i, flags;

	if (!strcmp_P(res->arg1, PSTR("reset"))) {
		encoders_hctl_set_value((void *)0, 0);
		encoders_hctl_set_value((void *)1, 0);
		encoders_hctl_set_value((void *)2, 0);
		encoders_hctl_set_value((void *)3, 0);
		return;
	}

	if (!strcmp_P(res->arg1, PSTR("hex"))) {
		flags = mainboard.flags;
		mainboard.flags &= ~DO_ENCODERS;
		while (!cmdline_keypressed()) {
			spi_slave_select(0);
			for (i = 0; i < 4; i++) {
#ifndef HOST_VERSION
				lsb[i] = spi_receive_byte();
				msb[i] = spi_receive_byte();
#endif
			}
			spi_slave_deselect(0);

			printf_P(PSTR
				 ("Encoders brut hex: % .4X % .4X % .4X % .4X\r\n"),
				 msb[0] << 8 | lsb[0], msb[1] << 8 | lsb[1],
				 msb[2] << 8 | lsb[2], msb[3] << 8 | lsb[3]);

			wait_ms(100);
		}
		mainboard.flags = flags;
		return;
	}

	/* show */
	while (!cmdline_keypressed()) {
		printf_P(PSTR("% .8ld % .8ld % .8ld % .8ld\r\n"),
			 encoders_hctl_get_value((void *)0),
			 encoders_hctl_get_value((void *)1),
			 encoders_hctl_get_value((void *)2),
			 encoders_hctl_get_value((void *)3));
		wait_ms(100);
	}
}

static const prog_char str_encoders_arg0[] = "encoders";
static parse_token_string_t cmd_encoders_arg0 =
TOKEN_STRING_INITIALIZER(struct cmd_encoders_result, arg0, str_encoders_arg0);
static const prog_char str_encoders_arg1[] = "hex#show#reset";
static parse_token_string_t cmd_encoders_arg1 =
TOKEN_STRING_INITIALIZER(struct cmd_encoders_result, arg1, str_encoders_arg1);

static const prog_char help_encoders[] = "Show encoders values";
parse_inst_t cmd_encoders = {
	.f = cmd_encoders_parsed,
	.data = NULL,
	.help_str = help_encoders,
	.tokens = {
		(prog_void *)&cmd_encoders_arg0,
		(prog_void *)&cmd_encoders_arg1,
		NULL,
	},
};

/* -----------------------------------------------------------------------------
 * scheduler - Show scheduler events
 *
 * Usage: scheduler show
 *
 * Display the scheduler event structure table.
 */

struct cmd_scheduler_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
};

static void cmd_scheduler_parsed(void *parsed_result, void *data)
{
	scheduler_dump_events();
	//scheduler_stats_dump();
}

static const prog_char str_scheduler_arg0[] = "scheduler";
static parse_token_string_t cmd_scheduler_arg0 =
TOKEN_STRING_INITIALIZER(struct cmd_scheduler_result, arg0, str_scheduler_arg0);
static const prog_char str_scheduler_arg1[] = "show";
static parse_token_string_t cmd_scheduler_arg1 =
TOKEN_STRING_INITIALIZER(struct cmd_scheduler_result, arg1, str_scheduler_arg1);

static const prog_char help_scheduler[] = "Show scheduler events";
parse_inst_t cmd_scheduler = {
	.f = cmd_scheduler_parsed,
	.data = NULL,
	.help_str = help_scheduler,
	.tokens = {
		(prog_void *)&cmd_scheduler_arg0,
		(prog_void *)&cmd_scheduler_arg1,
		NULL,
	},
};

/* -----------------------------------------------------------------------------
 * pwm - Set motor PWM commands
 *
 * Usage: pwm motor value
 *
 * motor: 1(4A) | 2(4B) | 3(1A) | 4(1B) | s1(3C) | s2(5A) | s3(5B) | s4(5C)
 * value: PWM value [-4096; 4095]
 *
 * Set the PWM command value for DC motors (1-4) or servo motors (s1-s4).
 */

struct cmd_pwm_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	int16_t arg2;
};

static void cmd_pwm_parsed(void *parsed_result, void *data)
{
	struct cmd_pwm_result *res = parsed_result;
	void *pwm_ptr = NULL;

	if (!strcmp_P(res->arg1, PSTR("1(4A)")))
		pwm_ptr = &gen.pwm1_4A;
	else if (!strcmp_P(res->arg1, PSTR("2(4B)")))
		pwm_ptr = &gen.pwm2_4B;
	else if (!strcmp_P(res->arg1, PSTR("3(1A)")))
		pwm_ptr = &gen.pwm3_1A;
	else if (!strcmp_P(res->arg1, PSTR("4(1B)")))
		pwm_ptr = &gen.pwm4_1B;

	else if (!strcmp_P(res->arg1, PSTR("s1(3C)")))
		pwm_ptr = &gen.servo1;
	else if (!strcmp_P(res->arg1, PSTR("s2(5A)")))
		pwm_ptr = &gen.servo2;
	else if (!strcmp_P(res->arg1, PSTR("s3(5B)")))
		pwm_ptr = &gen.servo3;
	else if (!strcmp_P(res->arg1, PSTR("s3(5C)")))
		pwm_ptr = &gen.servo4;

	if (pwm_ptr)
		pwm_ng_set(pwm_ptr, res->arg2);

	printf_P(PSTR("done\r\n"));
}

static const prog_char str_pwm_arg0[] = "pwm";
static parse_token_string_t cmd_pwm_arg0 =
TOKEN_STRING_INITIALIZER(struct cmd_pwm_result, arg0, str_pwm_arg0);
static const prog_char str_pwm_arg1[] =
    "1(4A)#2(4B)#3(1A)#4(1B)#s1(3C)#s2(5A)#s3(5B)#s4(5C)";
static parse_token_string_t cmd_pwm_arg1 =
TOKEN_STRING_INITIALIZER(struct cmd_pwm_result, arg1, str_pwm_arg1);
static parse_token_num_t cmd_pwm_arg2 =
TOKEN_NUM_INITIALIZER(struct cmd_pwm_result, arg2, INT16);

static const prog_char help_pwm[] = "Set pwm values [-4096 ; 4095]";
parse_inst_t cmd_pwm = {
	.f = cmd_pwm_parsed,
	.data = NULL,
	.help_str = help_pwm,
	.tokens = {
		(prog_void *)&cmd_pwm_arg0,
		(prog_void *)&cmd_pwm_arg1,
		(prog_void *)&cmd_pwm_arg2,
		NULL,
	},
};

/* -----------------------------------------------------------------------------
 * adc - Display ADC values
 *
 * Usage: adc show|loop_show
 */

struct cmd_adc_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
};

static void cmd_adc_parsed(void *parsed_result, void *data)
{
	struct cmd_adc_result *res = parsed_result;
	uint8_t loop = 0;
	uint8_t i;

	if (!strcmp_P(res->arg1, PSTR("loop_show")))
		loop = 1;

	do {
		printf_P(PSTR("BAT values: "));
		for (i = 0; i < 3; i++) {
			printf_P(PSTR("%.4d (%.4dmv) "), sensor_get_adc(i),
				 sensor_get_adc_mv(i));
		}
		printf_P(PSTR("\r\n"));

		printf_P(PSTR("MOT feedback values: "));
		for (i = 3; i < 7; i++) {
			printf_P(PSTR("%.4d (%.4dmv) "), sensor_get_adc(i),
				 sensor_get_adc_mv(i));
		}
		printf_P(PSTR("\r\n"));

		printf_P(PSTR("ADC values: "));
		for (i = 7; i < ADC_MAX; i++) {
			printf_P(PSTR("%.4d (%.4dmv) "), sensor_get_adc(i),
				 sensor_get_adc_mv(i));
		}
		printf_P(PSTR("\r\n"));

		wait_ms(100);
	} while (loop && !cmdline_keypressed());
}

static const prog_char str_adc_arg0[] = "adc";
static parse_token_string_t cmd_adc_arg0 =
TOKEN_STRING_INITIALIZER(struct cmd_adc_result, arg0, str_adc_arg0);
static const prog_char str_adc_arg1[] = "show#loop_show";
static parse_token_string_t cmd_adc_arg1 =
TOKEN_STRING_INITIALIZER(struct cmd_adc_result, arg1, str_adc_arg1);

static const prog_char help_adc[] = "Show adc values";
parse_inst_t cmd_adc = {
	.f = cmd_adc_parsed,
	.data = NULL,
	.help_str = help_adc,
	.tokens = {
		(prog_void *)&cmd_adc_arg0,
		(prog_void *)&cmd_adc_arg1,
		NULL,
	},
};

/* -----------------------------------------------------------------------------
 * sensor - Display sensor values
 *
 * Usage: sensor show|loop_show
 */

struct cmd_sensor_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
};

static void cmd_sensor_parsed(void *parsed_result, void *data)
{
	struct cmd_sensor_result *res = parsed_result;
	uint8_t loop = 0;
	uint8_t i;

	if (!strcmp_P(res->arg1, PSTR("loop_show")))
		loop = 1;

	do {
		printf_P(PSTR("SENSOR values: "));
		for (i = 0; i < SENSOR_MAX; i++)
			printf_P(PSTR("%d "), !!sensor_get(i));
		printf_P(PSTR("\r\n"));
		wait_ms(100);
	} while (loop && !cmdline_keypressed());
}

static const prog_char str_sensor_arg0[] = "sensor";
static parse_token_string_t cmd_sensor_arg0 =
TOKEN_STRING_INITIALIZER(struct cmd_sensor_result, arg0, str_sensor_arg0);
static const prog_char str_sensor_arg1[] = "show#loop_show";
static parse_token_string_t cmd_sensor_arg1 =
TOKEN_STRING_INITIALIZER(struct cmd_sensor_result, arg1, str_sensor_arg1);

static const prog_char help_sensor[] = "Show sensor values";
parse_inst_t cmd_sensor = {
	.f = cmd_sensor_parsed,
	.data = NULL,
	.help_str = help_sensor,
	.tokens = {
		(prog_void *)&cmd_sensor_arg0,
		(prog_void *)&cmd_sensor_arg1,
		NULL,
	},
};

/* -----------------------------------------------------------------------------
 * log - Configure logging
 *
 * Usage: log show
 * 	  log level num
 * 	  log type uart|rs|servo|traj|oa|strat|ext|sensor|bd|cs on|off
 */

struct cmd_log_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	uint8_t arg2;
	fixed_string_t arg3;
};

/* keep it sync with string choice */
static const const prog_char uart_log[] = "uart";
static const const prog_char rs_log[] = "rs";
static const const prog_char traj_log[] = "traj";
static const const prog_char oa_log[] = "oa";
static const const prog_char strat_log[] = "strat";
static const const prog_char sensor_log[] = "sensor";
static const const prog_char block_log[] = "bd";
static const const prog_char cs_log[] = "cs";

struct log_name_and_num {
	const const prog_char *name;
	uint8_t num;
};

static const struct log_name_and_num log_name_and_num[] = {
	{uart_log, E_UART},
	{rs_log, E_ROBOT_SYSTEM},
	{traj_log, E_TRAJECTORY},
	{oa_log, E_OA},
	{strat_log, E_USER_STRAT},
	{sensor_log, E_USER_SENSOR},
	{block_log, E_BLOCKING_DETECTION_MANAGER},
	{cs_log, E_USER_CS},
};

static uint8_t log_name2num(const char *s)
{
	uint8_t i;

	for (i = 0; i < ARRAY_SIZE(log_name_and_num); i++) {
		if (!strcmp_P(s, log_name_and_num[i].name))
			return log_name_and_num[i].num;
	}
	return 0;
}

const const prog_char *log_num2name(uint8_t num)
{
	uint8_t i;

	for (i = 0; i < ARRAY_SIZE(log_name_and_num); i++) {
		if (num == log_name_and_num[i].num)
			return log_name_and_num[i].name;
	}
	return NULL;
}

static void cmd_log_do_show(void)
{
	const const prog_char *name;
	uint8_t empty = 1;
	uint8_t i;

	printf_P(PSTR("log level is %d\r\n"), gen.log_level);
	for (i = 0; i < NB_LOGS; i++) {
		name = log_num2name(gen.logs[i]);
		if (name) {
			printf_P(PSTR("log type %S is on\r\n"), name);
			empty = 0;
		}
	}

	if (empty)
		printf_P(PSTR("no log configured\r\n"));
}

static void cmd_log_parsed(void *parsed_result, void *data)
{
	struct cmd_log_result *res = (struct cmd_log_result *)parsed_result;

	if (!strcmp_P(res->arg1, PSTR("level")))
		gen.log_level = res->arg2;

	cmd_log_do_show();
}

static const prog_char str_log_arg0[] = "log";
static parse_token_string_t cmd_log_arg0 =
TOKEN_STRING_INITIALIZER(struct cmd_log_result, arg0, str_log_arg0);
static const prog_char str_log_arg1[] = "level";
static parse_token_string_t cmd_log_arg1 =
TOKEN_STRING_INITIALIZER(struct cmd_log_result, arg1, str_log_arg1);
static parse_token_num_t cmd_log_arg2 =
TOKEN_NUM_INITIALIZER(struct cmd_log_result, arg2, INT32);

static const prog_char help_log[] = "Set log options: level (0 -> 5)";
parse_inst_t cmd_log = {
	.f = cmd_log_parsed,
	.data = NULL,
	.help_str = help_log,
	.tokens = {
		(prog_void *)&cmd_log_arg0,
		(prog_void *)&cmd_log_arg1,
		(prog_void *)&cmd_log_arg2,
		NULL,
	},
};

static const prog_char str_log_arg1_show[] = "show";
static parse_token_string_t cmd_log_arg1_show =
TOKEN_STRING_INITIALIZER(struct cmd_log_result, arg1, str_log_arg1_show);

static const prog_char help_log_show[] = "Show configured logs";
parse_inst_t cmd_log_show = {
	.f = cmd_log_parsed,
	.data = NULL,
	.help_str = help_log_show,
	.tokens = {
		(prog_void *)&cmd_log_arg0,
		(prog_void *)&cmd_log_arg1_show,
		NULL,
	},
};

struct cmd_log_type_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	fixed_string_t arg2;
	fixed_string_t arg3;
};

static void cmd_log_type_parsed(void *parsed_result, void *data)
{
	struct cmd_log_type_result *res =
	    (struct cmd_log_type_result *)parsed_result;
	uint8_t lognum;
	uint8_t i;

	lognum = log_name2num(res->arg2);
	if (lognum == 0) {
		printf_P(PSTR("Cannot find log num\r\n"));
		return;
	}

	if (!strcmp_P(res->arg3, PSTR("on"))) {
		for (i = 0; i < NB_LOGS; i++) {
			if (gen.logs[i] == lognum) {
				printf_P(PSTR("Already on\r\n"));
				return;
			}
		}
		for (i = 0; i < NB_LOGS; i++) {
			if (gen.logs[i] == 0) {
				gen.logs[i] = lognum;
				break;
			}
		}
		if (i == NB_LOGS)
			printf_P(PSTR("no more room\r\n"));
	} else {
		for (i = 0; i < NB_LOGS; i++) {
			if (gen.logs[i] == lognum) {
				gen.logs[i] = 0;
				break;
			}
		}
		if (i == NB_LOGS)
			printf_P(PSTR("already off\r\n"));
	}
	cmd_log_do_show();
}

static const prog_char str_log_arg1_type[] = "type";
static parse_token_string_t cmd_log_arg1_type =
TOKEN_STRING_INITIALIZER(struct cmd_log_type_result, arg1, str_log_arg1_type);
/* keep it sync with log_name_and_num above */
static const prog_char str_log_arg2_type[] = "uart#rs#servo#traj#oa#strat#ext#sensor#bd#cs";
static parse_token_string_t cmd_log_arg2_type =
TOKEN_STRING_INITIALIZER(struct cmd_log_type_result, arg2, str_log_arg2_type);
static const prog_char str_log_arg3[] = "on#off";
static parse_token_string_t cmd_log_arg3 =
TOKEN_STRING_INITIALIZER(struct cmd_log_type_result, arg3, str_log_arg3);

static const prog_char help_log_type[] = "Set log type";
parse_inst_t cmd_log_type = {
	.f = cmd_log_type_parsed,
	.data = NULL,
	.help_str = help_log_type,
	.tokens = {
		(prog_void *)&cmd_log_arg0,
		(prog_void *)&cmd_log_arg1_type,
		(prog_void *)&cmd_log_arg2_type,
		(prog_void *)&cmd_log_arg3,
		NULL,
	},
};

/* -----------------------------------------------------------------------------
 * stack_space - Display remaining stack space
 *
 * Usage: stack_space
 */

struct cmd_stack_space_result {
	fixed_string_t arg0;
};

static void cmd_stack_space_parsed(void *parsed_result, void *data)
{
	printf("res stack: %d\r\n", min_stack_space_available());
}

static const prog_char str_stack_space_arg0[] = "stack_space";
static parse_token_string_t cmd_stack_space_arg0 =
TOKEN_STRING_INITIALIZER(struct cmd_stack_space_result, arg0,
			 str_stack_space_arg0);

static const prog_char help_stack_space[] = "Display remaining stack space";
parse_inst_t cmd_stack_space = {
	.f = cmd_stack_space_parsed,
	.data = NULL,
	.help_str = help_stack_space,
	.tokens = {
		(prog_void *) & cmd_stack_space_arg0,
		NULL,
	},
};

/* -----------------------------------------------------------------------------
 * ir - Enable/disable IR transmitters
 *
 * Usage: ir on|off
 */

struct cmd_ir_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
};

static void cmd_ir_parsed(void *parsed_result, void *data)
{
	struct cmd_ir_result *res = (struct cmd_ir_result *)parsed_result;

	if (!strcmp_P(res->arg1, PSTR("on")))
		mainboard.ir = 1;
	else
		mainboard.ir = 0;
}

static const prog_char str_ir_arg0[] = "ir";
static parse_token_string_t cmd_ir_arg0 =
TOKEN_STRING_INITIALIZER(struct cmd_ir_result, arg0, str_ir_arg0);
static const prog_char str_ir_arg1[] = "on#off";
static parse_token_string_t cmd_ir_arg1 =
TOKEN_STRING_INITIALIZER(struct cmd_ir_result, arg1, str_ir_arg1);

static const prog_char help_ir[] = "enable/disable IR transmitters";
parse_inst_t cmd_ir = {
	.f = cmd_ir_parsed,
	.data = NULL,
	.help_str = help_ir,
	.tokens = {
		(prog_void *)&cmd_ir_arg0,
		(prog_void *)&cmd_ir_arg1,
		NULL,
	},
};
