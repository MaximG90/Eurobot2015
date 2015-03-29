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

#include <stdlib.h>
#include <aversive/pgmspace.h>
#include <parse.h>

/* commands_ax12.c */
extern parse_inst_t cmd_ax12_stress;
extern parse_inst_t cmd_ax12_baudrate;
extern parse_inst_t cmd_ax12_read;
extern parse_inst_t cmd_ax12_write;

/* commands_gen.c */
extern parse_inst_t cmd_reset;
extern parse_inst_t cmd_bootloader;
extern parse_inst_t cmd_buzzer;
extern parse_inst_t cmd_encoders;
extern parse_inst_t cmd_pwm;
extern parse_inst_t cmd_adc;
extern parse_inst_t cmd_sensor;
extern parse_inst_t cmd_log;
extern parse_inst_t cmd_log_show;
extern parse_inst_t cmd_log_type;
extern parse_inst_t cmd_stack_space;
extern parse_inst_t cmd_scheduler;
extern parse_inst_t cmd_ir;

/* commands_cs.c */
extern parse_inst_t cmd_gain;
extern parse_inst_t cmd_gain_show;
extern parse_inst_t cmd_speed;
extern parse_inst_t cmd_speed_show;
extern parse_inst_t cmd_derivate_filter;
extern parse_inst_t cmd_derivate_filter_show;
extern parse_inst_t cmd_consign;
extern parse_inst_t cmd_maximum;
extern parse_inst_t cmd_maximum_show;
extern parse_inst_t cmd_quadramp;
extern parse_inst_t cmd_quadramp_show;
extern parse_inst_t cmd_cs_status;
extern parse_inst_t cmd_blocking_i;
extern parse_inst_t cmd_blocking_i_show;

/* commands_mainboard.c */
extern parse_inst_t cmd_event;
extern parse_inst_t cmd_spi_test;
extern parse_inst_t cmd_start;
extern parse_inst_t cmd_interact;
extern parse_inst_t cmd_color;
extern parse_inst_t cmd_rs;

/* commands_traj.c */
extern parse_inst_t cmd_traj_speed;
extern parse_inst_t cmd_traj_speed_show;
extern parse_inst_t cmd_trajectory;
extern parse_inst_t cmd_trajectory_show;
extern parse_inst_t cmd_rs_gains;
extern parse_inst_t cmd_rs_gains_show;
extern parse_inst_t cmd_track;
extern parse_inst_t cmd_track_show;
extern parse_inst_t cmd_pt_list;
extern parse_inst_t cmd_pt_list_append;
extern parse_inst_t cmd_pt_list_del;
extern parse_inst_t cmd_pt_list_show;
extern parse_inst_t cmd_goto1;
extern parse_inst_t cmd_goto2;
extern parse_inst_t cmd_goto3;
extern parse_inst_t cmd_position;
extern parse_inst_t cmd_position_set;
extern parse_inst_t cmd_strat_info;
extern parse_inst_t cmd_strat_conf;
extern parse_inst_t cmd_strat_conf2;
extern parse_inst_t cmd_strat_conf3;
extern parse_inst_t cmd_subtraj;
extern parse_inst_t cmd_motion;

/* in progmem */
parse_ctx_t main_ctx[] = {

	/* commands_ax12.c */
	(parse_inst_t *) & cmd_ax12_stress,
	(parse_inst_t *) & cmd_ax12_baudrate,
	(parse_inst_t *) & cmd_ax12_read,
	(parse_inst_t *) & cmd_ax12_write,

	/* commands_gen.c */
	(parse_inst_t *) & cmd_reset,
	(parse_inst_t *) & cmd_bootloader,
	(parse_inst_t *) & cmd_buzzer,
	(parse_inst_t *) & cmd_encoders,
	(parse_inst_t *) & cmd_pwm,
	(parse_inst_t *) & cmd_adc,
	(parse_inst_t *) & cmd_sensor,
	(parse_inst_t *) & cmd_log,
	(parse_inst_t *) & cmd_log_show,
	(parse_inst_t *) & cmd_log_type,
	(parse_inst_t *) & cmd_stack_space,
	(parse_inst_t *) & cmd_scheduler,
	(parse_inst_t *) & cmd_ir,

	/* commands_cs.c */
	(parse_inst_t *) & cmd_gain,
	(parse_inst_t *) & cmd_gain_show,
	(parse_inst_t *) & cmd_speed,
	(parse_inst_t *) & cmd_speed_show,
	(parse_inst_t *) & cmd_consign,
	(parse_inst_t *) & cmd_derivate_filter,
	(parse_inst_t *) & cmd_derivate_filter_show,
	(parse_inst_t *) & cmd_maximum,
	(parse_inst_t *) & cmd_maximum_show,
	(parse_inst_t *) & cmd_quadramp,
	(parse_inst_t *) & cmd_quadramp_show,
	(parse_inst_t *) & cmd_cs_status,
	(parse_inst_t *) & cmd_blocking_i,
	(parse_inst_t *) & cmd_blocking_i_show,

	/* commands_mainboard.c */
	(parse_inst_t *) & cmd_event,
	(parse_inst_t *) & cmd_spi_test,
	(parse_inst_t *) & cmd_start,
	(parse_inst_t *) & cmd_interact,
	(parse_inst_t *) & cmd_color,
	(parse_inst_t *) & cmd_rs,

	/* commands_traj.c */
	(parse_inst_t *) & cmd_traj_speed,
	(parse_inst_t *) & cmd_traj_speed_show,
	(parse_inst_t *) & cmd_trajectory,
	(parse_inst_t *) & cmd_trajectory_show,
	(parse_inst_t *) & cmd_rs_gains,
	(parse_inst_t *) & cmd_rs_gains_show,
	(parse_inst_t *) & cmd_track,
	(parse_inst_t *) & cmd_track_show,
	(parse_inst_t *) & cmd_pt_list,
	(parse_inst_t *) & cmd_pt_list_append,
	(parse_inst_t *) & cmd_pt_list_del,
	(parse_inst_t *) & cmd_pt_list_show,
	(parse_inst_t *) & cmd_goto1,
	(parse_inst_t *) & cmd_goto2,
	(parse_inst_t *) & cmd_position,
	(parse_inst_t *) & cmd_position_set,
	(parse_inst_t *) & cmd_strat_info,
	(parse_inst_t *) & cmd_strat_conf,
	(parse_inst_t *) & cmd_strat_conf2,
	(parse_inst_t *) & cmd_strat_conf3,
	(parse_inst_t *) & cmd_subtraj,
	(parse_inst_t *) & cmd_motion,
	NULL,
};
