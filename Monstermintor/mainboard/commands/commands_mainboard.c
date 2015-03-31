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
#ifndef HOST_VERSION
#include <avr/eeprom.h>
#endif

#include <pwm_ng.h>
#include <spi.h>
#include <clock_time.h>
#include <uart.h>

#include <pid.h>
#include <quadramp.h>
#include <control_system_manager.h>
#include <trajectory_manager.h>
#include <vect_base.h>
#include <lines.h>
#include <polygon.h>
#include <obstacle_avoidance.h>
#include <blocking_detection_manager.h>
#include <robot_system.h>
#include <position_manager.h>

#include <rdline.h>
#include <parse.h>
#include <parse_string.h>
#include <parse_num.h>

#include "../common/eeprom_mapping.h"

#include "actuator.h"
#include "cmdline.h"
#include "main.h"
#include "motion.h"
#include "motor.h"
#include "sensor.h"
#include "strat.h"

/* -----------------------------------------------------------------------------
 * event - Enable/disable event processing
 *
 * Usage: event all|encoders|cs|rs|pos|bd|timer|power on|off|show
 */

struct cmd_event_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	fixed_string_t arg2;
};

static void cmd_event_parsed(void *parsed_result, void *data)
{
	struct cmd_event_result *res = parsed_result;
	uint8_t bit = 0;

	if (!strcmp_P(res->arg1, PSTR("all"))) {
		bit = DO_ENCODERS | DO_CS | DO_RS | DO_POS |
		      DO_BD | DO_TIMER | DO_POWER;

		if (!strcmp_P(res->arg2, PSTR("on")))
			mainboard.flags |= bit;
		else if (!strcmp_P(res->arg2, PSTR("off")))
			mainboard.flags &= bit;
		else {		/* show */

			printf_P(PSTR("encoders is %s\r\n"),
				 (DO_ENCODERS & mainboard.
				  flags) ? "on" : "off");
			printf_P(PSTR("cs is %s\r\n"),
				 (DO_CS & mainboard.flags) ? "on" : "off");
			printf_P(PSTR("rs is %s\r\n"),
				 (DO_RS & mainboard.flags) ? "on" : "off");
			printf_P(PSTR("pos is %s\r\n"),
				 (DO_POS & mainboard.flags) ? "on" : "off");
			printf_P(PSTR("bd is %s\r\n"),
				 (DO_BD & mainboard.flags) ? "on" : "off");
			printf_P(PSTR("timer is %s\r\n"),
				 (DO_TIMER & mainboard.flags) ? "on" : "off");
			printf_P(PSTR("power is %s\r\n"),
				 (DO_POWER & mainboard.flags) ? "on" : "off");
		}
		return;
	}

	if (!strcmp_P(res->arg1, PSTR("encoders")))
		bit = DO_ENCODERS;
	else if (!strcmp_P(res->arg1, PSTR("cs"))) {
		motion_hardstop();
		bit = DO_CS;
	} else if (!strcmp_P(res->arg1, PSTR("rs")))
		bit = DO_RS;
	else if (!strcmp_P(res->arg1, PSTR("pos")))
		bit = DO_POS;
	else if (!strcmp_P(res->arg1, PSTR("bd")))
		bit = DO_BD;
	else if (!strcmp_P(res->arg1, PSTR("timer"))) {
		time_reset();
		bit = DO_TIMER;
	} else if (!strcmp_P(res->arg1, PSTR("power")))
		bit = DO_POWER;

	if (!strcmp_P(res->arg2, PSTR("on")))
		mainboard.flags |= bit;
	else if (!strcmp_P(res->arg2, PSTR("off"))) {
		if (bit == DO_CS) {
			pwm_ng_set(LEFT_PWM, 0);
			pwm_ng_set(RIGHT_PWM, 0);
		}
		mainboard.flags &= (~bit);
	}
	printf_P(PSTR("%s is %s\r\n"), res->arg1,
		 (bit & mainboard.flags) ? "on" : "off");
}

static const prog_char str_event_arg0[] = "event";
static const parse_pgm_token_string_t cmd_event_arg0 =
TOKEN_STRING_INITIALIZER(struct cmd_event_result, arg0, str_event_arg0);
static const prog_char str_event_arg1[] = "all#encoders#cs#rs#pos#bd#timer#power";
static const parse_pgm_token_string_t cmd_event_arg1 =
TOKEN_STRING_INITIALIZER(struct cmd_event_result, arg1, str_event_arg1);
static const prog_char str_event_arg2[] = "on#off#show";
static const parse_pgm_token_string_t cmd_event_arg2 =
TOKEN_STRING_INITIALIZER(struct cmd_event_result, arg2, str_event_arg2);

static const prog_char help_event[] = "Enable/disable events";
const parse_pgm_inst_t  cmd_event = {
	.f = cmd_event_parsed,
	.data = NULL,
	.help_str = help_event,
	.tokens = {
		(prog_void *)&cmd_event_arg0,
		(prog_void *)&cmd_event_arg1,
		(prog_void *)&cmd_event_arg2,
		NULL,
	},
};

/* -----------------------------------------------------------------------------
 * spi_test - Test SPI communications
 *
 * Usage: spi_test
 *
 * Send and receive bytes on the SPI bus and display the results.
 */

struct cmd_spi_test_result {
	fixed_string_t arg0;
};

static void cmd_spi_test_parsed(void *parsed_result, void *data)
{
	uint16_t i = 0;
	uint16_t ret, ret2;

	if (mainboard.flags & DO_ENCODERS) {
		printf_P(PSTR("Disable encoder event first\r\n"));
		return;
	}

	do {
		spi_slave_select(0);
		ret = spi_send_and_receive_byte(i);
		ret2 = spi_send_and_receive_byte(i);
		spi_slave_deselect(0);

		if ((i & 0x7ff) == 0)
			printf_P(PSTR("Sent %.4x twice, received %x %x\r\n"),
				 i, ret, ret2);

		i++;
	}
	while (!cmdline_keypressed());
}

static const prog_char str_spi_test_arg0[] = "spi_test";
static const parse_pgm_token_string_t cmd_spi_test_arg0 =
TOKEN_STRING_INITIALIZER(struct cmd_spi_test_result, arg0, str_spi_test_arg0);

static const prog_char help_spi_test[] = "Test the SPI";
const parse_pgm_inst_t  cmd_spi_test = {
	.f = cmd_spi_test_parsed,
	.data = NULL,
	.help_str = help_spi_test,
	.tokens = {
		(prog_void *)&cmd_spi_test_arg0,
		NULL,
	},
};

/* -----------------------------------------------------------------------------
 * start - Start the game
 *
 * Usage: start color debug|match
 *
 * Start the game in the selected color and debug mode.
 */

struct cmd_start_result {
	fixed_string_t arg0;
	fixed_string_t color;
	fixed_string_t debug;
};

static void cmd_start_parsed(void *parsed_result, void *data)
{
	struct cmd_start_result *res = parsed_result;
	uint8_t old_level = gen.log_level;

	gen.logs[NB_LOGS] = E_USER_STRAT;
	if (!strcmp_P(res->debug, PSTR("debug"))) {
		strat_info.dump_enabled = 1;
		gen.log_level = 5;
	} else {
		strat_info.dump_enabled = 0;
		gen.log_level = 0;
	}

	if (!strcmp_P(res->color, PSTR(ROBOT_COLOR_DEFAULT_NAME)))
		mainboard.our_color = ROBOT_COLOR_DEFAULT;
	else if (!strcmp_P(res->color, PSTR(ROBOT_COLOR_ALTERNATE_NAME)))
		mainboard.our_color = ROBOT_COLOR_ALTERNATE;

	printf_P(PSTR("Press a key when ready\r\n"));
	while (!cmdline_keypressed()) {};

	strat_start();

	gen.logs[NB_LOGS] = 0;
	gen.log_level = old_level;
}

static const prog_char str_start_arg0[] = "start";
static const parse_pgm_token_string_t cmd_start_arg0 =
TOKEN_STRING_INITIALIZER(struct cmd_start_result, arg0, str_start_arg0);
static const prog_char str_start_color[] = ROBOT_COLOR_DEFAULT_NAME "#" ROBOT_COLOR_ALTERNATE_NAME;
static const parse_pgm_token_string_t cmd_start_color =
TOKEN_STRING_INITIALIZER(struct cmd_start_result, color, str_start_color);
static const prog_char str_start_debug[] = "debug#match";
static const parse_pgm_token_string_t cmd_start_debug =
TOKEN_STRING_INITIALIZER(struct cmd_start_result, debug, str_start_debug);

static const prog_char help_start[] = "Start the robot (start " \
				ROBOT_COLOR_DEFAULT_NAME "#" ROBOT_COLOR_ALTERNATE_NAME \
				" debug#match)";
const parse_pgm_inst_t  cmd_start = {
	.f = cmd_start_parsed,
	.data = NULL,
	.help_str = help_start,
	.tokens = {
		(prog_void *)&cmd_start_arg0,
		(prog_void *)&cmd_start_color,
		(prog_void *)&cmd_start_debug,
		NULL,
	},
};

/* -----------------------------------------------------------------------------
 * interact - Switch to interactive mode
 *
 * Usage: interact
 *
 * In interactive mode, the robot can be controlled through the serial port
 * using the keyboard arrows and space key.
 */

struct cmd_interact_result {
	fixed_string_t arg0;
};

static void print_cs(void)
{
	printf_P(PSTR("cons_d=% .8ld cons_a=% .8ld fil_d=% .8ld fil_a=% .8ld "
		      "err_d=% .8ld err_a=% .8ld out_d=% .8ld out_a=% .8ld\r\n"),
		 cs_get_consign(&mainboard.distance.cs),
		 cs_get_consign(&mainboard.angle.cs),
		 cs_get_filtered_consign(&mainboard.distance.cs),
		 cs_get_filtered_consign(&mainboard.angle.cs),
		 cs_get_error(&mainboard.distance.cs),
		 cs_get_error(&mainboard.angle.cs),
		 cs_get_out(&mainboard.distance.cs),
		 cs_get_out(&mainboard.angle.cs));
}

static void print_time(void)
{
	printf_P(PSTR("time %d\r\n"), time_get_s());
}

static void print_sensors(void)
{
#ifdef notyet
	if (sensor_start_switch())
		printf_P(PSTR("Start switch | "));
	else
		printf_P(PSTR("             | "));

	if (IR_DISP_SENSOR())
		printf_P(PSTR("IR disp | "));
	else
		printf_P(PSTR("        | "));

	printf_P(PSTR("\r\n"));
#endif
}

static void print_pid(void)
{
	printf_P(PSTR("P=% .8ld I=% .8ld D=% .8ld out=% .8ld | "
		      "P=% .8ld I=% .8ld D=% .8ld out=% .8ld\r\n"),
		 pid_get_value_in(&mainboard.distance.pid) *
		 pid_get_gain_P(&mainboard.distance.pid),
		 pid_get_value_I(&mainboard.distance.pid) *
		 pid_get_gain_I(&mainboard.distance.pid),
		 pid_get_value_D(&mainboard.distance.pid) *
		 pid_get_gain_D(&mainboard.distance.pid),
		 pid_get_value_out(&mainboard.distance.pid),
		 pid_get_value_in(&mainboard.angle.pid) *
		 pid_get_gain_P(&mainboard.angle.pid),
		 pid_get_value_I(&mainboard.angle.pid) *
		 pid_get_gain_I(&mainboard.angle.pid),
		 pid_get_value_D(&mainboard.angle.pid) *
		 pid_get_gain_D(&mainboard.angle.pid),
		 pid_get_value_out(&mainboard.angle.pid));
}

#define PRINT_POS	(1 << 0)
#define PRINT_PID	(1 << 1)
#define PRINT_CS	(1 << 2)
#define PRINT_SENSORS	(1 << 3)
#define PRINT_TIME	(1 << 4)
#define PRINT_BLOCKING	(1 << 5)

static void cmd_interact_parsed(void *parsed_result, void *data)
{
	struct vt100 vt100;
	uint8_t print = 0;
	int8_t cmd;
	int c;

	vt100_init(&vt100);

	printf_P(PSTR("Display debugs:\r\n"
		      "  1:pos\r\n"
		      "  2:pid\r\n"
		      "  3:cs\r\n" "  4:sensors\r\n" "  5:time\r\n"
		      /* "  6:blocking\r\n" */
		      "Commands:\r\n"
		      "  arrows:move\r\n" "  space:stop\r\n" "  q:quit\r\n"));

	/* stop motors */
	mainboard.flags &= (~DO_CS);
	pwm_set_and_save(LEFT_PWM, 0);
	pwm_set_and_save(RIGHT_PWM, 0);

	while (1) {
		if (print & PRINT_POS)
			motion_log_position();

		if (print & PRINT_PID)
			print_pid();

		if (print & PRINT_CS)
			print_cs();

		if (print & PRINT_SENSORS)
			print_sensors();

		if (print & PRINT_TIME)
			print_time();

/*		if (print & PRINT_BLOCKING)
			printf_P(PSTR("%s %s blocking=%d\r\n"),
				 mainboard.blocking ? "BLOCK1":"      ",
				 rs_is_blocked(&mainboard.rs) ? "BLOCK2":"      ",
				 rs_get_blocking(&mainboard.rs));
*/

		wait_ms(10);
		c = cmdline_getchar();
		if (c == -1)
			continue;

		cmd = vt100_parser(&vt100, c);
		if (cmd == -2)
			continue;

		if (cmd == -1) {
			switch (c) {
			case '1':
				print ^= PRINT_POS;
				break;
			case '2':
				print ^= PRINT_PID;
				break;
			case '3':
				print ^= PRINT_CS;
				break;
			case '4':
				print ^= PRINT_SENSORS;
				break;
			case '5':
				print ^= PRINT_TIME;
				break;
			case '6':
				print ^= PRINT_BLOCKING;
				break;

			case 'q':
				if (mainboard.flags & DO_CS)
					motion_hardstop();
				pwm_set_and_save(LEFT_PWM, 0);
				pwm_set_and_save(RIGHT_PWM, 0);
				return;
			case ' ':
				pwm_set_and_save(LEFT_PWM, 0);
				pwm_set_and_save(RIGHT_PWM, 0);
				break;
			}
		} else {
			switch (cmd) {
			case KEY_UP_ARR:
				pwm_set_and_save(LEFT_PWM, 1200);
				pwm_set_and_save(RIGHT_PWM, 1200);
				break;
			case KEY_LEFT_ARR:
				pwm_set_and_save(LEFT_PWM, -1200);
				pwm_set_and_save(RIGHT_PWM, 1200);
				break;
			case KEY_DOWN_ARR:
				pwm_set_and_save(LEFT_PWM, -1200);
				pwm_set_and_save(RIGHT_PWM, -1200);
				break;
			case KEY_RIGHT_ARR:
				pwm_set_and_save(LEFT_PWM, 1200);
				pwm_set_and_save(RIGHT_PWM, -1200);
				break;
			}
		}
	}
}

static const prog_char str_interact_arg0[] = "interact";
static const parse_pgm_token_string_t cmd_interact_arg0 =
TOKEN_STRING_INITIALIZER(struct cmd_interact_result, arg0, str_interact_arg0);

static const prog_char help_interact[] = "Interactive mode";
const parse_pgm_inst_t  cmd_interact = {
	.f = cmd_interact_parsed,
	.data = NULL,
	.help_str = help_interact,
	.tokens = {
		(prog_void *)&cmd_interact_arg0,
		NULL,
	},
};

/* -----------------------------------------------------------------------------
 * color - Set the player color
 *
 * Usage: color color
 */

struct cmd_color_result {
	fixed_string_t arg0;
	fixed_string_t color;
};

static void cmd_color_parsed(void *parsed_result, void *data)
{
	struct cmd_color_result *res = (struct cmd_color_result *)parsed_result;

	if (!strcmp_P(res->color, PSTR(ROBOT_COLOR_DEFAULT_NAME)))
		mainboard.our_color = ROBOT_COLOR_DEFAULT;
	else if (!strcmp_P(res->color, PSTR(ROBOT_COLOR_ALTERNATE_NAME)))
		mainboard.our_color = ROBOT_COLOR_ALTERNATE;

	printf_P(PSTR("Done\r\n"));
}

static const prog_char str_color_arg0[] = "color";
static const parse_pgm_token_string_t cmd_color_arg0 =
TOKEN_STRING_INITIALIZER(struct cmd_color_result, arg0, str_color_arg0);
static const prog_char str_color_color[] = ROBOT_COLOR_DEFAULT_NAME "#" ROBOT_COLOR_ALTERNATE_NAME;
static const parse_pgm_token_string_t cmd_color_color =
TOKEN_STRING_INITIALIZER(struct cmd_color_result, color, str_color_color);

static const prog_char help_color[] = "Set our color";
const parse_pgm_inst_t  cmd_color = {
	.f = cmd_color_parsed,
	.data = NULL,
	.help_str = help_color,
	.tokens = {
		(prog_void *)&cmd_color_arg0,
		(prog_void *)&cmd_color_color,
		NULL,
	},
};

/* -----------------------------------------------------------------------------
 * rs - Display robot system values
 *
 * Usage: rs show
 *
 * show: display the robot system values (angle and distance control systems and
 *       PWM commands)
 */

struct cmd_rs_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
};

static void cmd_rs_parsed(void *parsed_result, void *data)
{
	do {
		printf_P(PSTR("angle cons=% .6ld in=% .6ld out=% .6ld / "),
			 cs_get_consign(&mainboard.angle.cs),
			 cs_get_filtered_feedback(&mainboard.angle.cs),
			 cs_get_out(&mainboard.angle.cs));
		printf_P(PSTR("distance cons=% .6ld in=% .6ld out=% .6ld / "),
			 cs_get_consign(&mainboard.distance.cs),
			 cs_get_filtered_feedback(&mainboard.distance.cs),
			 cs_get_out(&mainboard.distance.cs));
		printf_P(PSTR("l=% .4ld r=% .4ld\r\n"), mainboard.pwm_l,
			 mainboard.pwm_r);
		wait_ms(100);
	}
	while (!cmdline_keypressed());
}

static const prog_char str_rs_arg0[] = "rs";
static const parse_pgm_token_string_t cmd_rs_arg0 =
TOKEN_STRING_INITIALIZER(struct cmd_rs_result, arg0, str_rs_arg0);
static const prog_char str_rs_arg1[] = "show";
static const parse_pgm_token_string_t cmd_rs_arg1 =
TOKEN_STRING_INITIALIZER(struct cmd_rs_result, arg1, str_rs_arg1);

static const prog_char help_rs[] = "Show rs (robot system) values";
const parse_pgm_inst_t  cmd_rs = {
	.f = cmd_rs_parsed,
	.data = NULL,
	.help_str = help_rs,
	.tokens = {
		(prog_void *)&cmd_rs_arg0,
		(prog_void *)&cmd_rs_arg1,
		NULL,
	},
};
