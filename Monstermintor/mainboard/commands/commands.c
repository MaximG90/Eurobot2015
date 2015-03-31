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
extern const parse_pgm_inst_t  const  cmd_ax12_stress;
extern const parse_pgm_inst_t  const  cmd_ax12_baudrate;
extern const parse_pgm_inst_t  const  cmd_ax12_read;
extern const parse_pgm_inst_t  const  cmd_ax12_write;

/* commands_gen.c */
extern const parse_pgm_inst_t  const  cmd_reset;
extern const parse_pgm_inst_t  const  cmd_bootloader;
extern const parse_pgm_inst_t  const  cmd_buzzer;
extern const parse_pgm_inst_t  const  cmd_encoders;
extern const parse_pgm_inst_t  const  cmd_pwm;
extern const parse_pgm_inst_t  const  cmd_adc;
extern const parse_pgm_inst_t  const  cmd_sensor;
extern const parse_pgm_inst_t  const  cmd_log;
extern const parse_pgm_inst_t  const  cmd_log_show;
extern const parse_pgm_inst_t  const  cmd_log_type;
extern const parse_pgm_inst_t  const  cmd_stack_space;
extern const parse_pgm_inst_t  const  cmd_scheduler;
extern const parse_pgm_inst_t  const  cmd_ir;

/* commands_cs.c */
extern const parse_pgm_inst_t  const  cmd_gain;
extern const parse_pgm_inst_t  const  cmd_gain_show;
extern const parse_pgm_inst_t  const  cmd_speed;
extern const parse_pgm_inst_t  const  cmd_speed_show;
extern const parse_pgm_inst_t  const  cmd_derivate_filter;
extern const parse_pgm_inst_t  const  cmd_derivate_filter_show;
extern const parse_pgm_inst_t  const  cmd_consign;
extern const parse_pgm_inst_t  const  cmd_maximum;
extern const parse_pgm_inst_t  const  cmd_maximum_show;
extern const parse_pgm_inst_t  const  cmd_quadramp;
extern const parse_pgm_inst_t  const  cmd_quadramp_show;
extern const parse_pgm_inst_t  const  cmd_cs_status;
extern const parse_pgm_inst_t  const  cmd_blocking_i;
extern const parse_pgm_inst_t  const  cmd_blocking_i_show;

/* commands_mainboard.c */
extern const parse_pgm_inst_t  const  cmd_event;
extern const parse_pgm_inst_t  const  cmd_spi_test;
extern const parse_pgm_inst_t  const  cmd_start;
extern const parse_pgm_inst_t  const  cmd_interact;
extern const parse_pgm_inst_t  const  cmd_color;
extern const parse_pgm_inst_t  const  cmd_rs;

/* commands_traj.c */
extern const parse_pgm_inst_t  const  cmd_traj_speed;
extern const parse_pgm_inst_t  const  cmd_traj_speed_show;
extern const parse_pgm_inst_t  const  cmd_trajectory;
extern const parse_pgm_inst_t  const  cmd_trajectory_show;
extern const parse_pgm_inst_t  const  cmd_rs_gains;
extern const parse_pgm_inst_t  const  cmd_rs_gains_show;
extern const parse_pgm_inst_t  const  cmd_track;
extern const parse_pgm_inst_t  const  cmd_track_show;
extern const parse_pgm_inst_t  const  cmd_pt_list;
extern const parse_pgm_inst_t  const  cmd_pt_list_append;
extern const parse_pgm_inst_t  const  cmd_pt_list_del;
extern const parse_pgm_inst_t  const  cmd_pt_list_show;
extern const parse_pgm_inst_t  const  cmd_goto1;
extern const parse_pgm_inst_t  const  cmd_goto2;
extern const parse_pgm_inst_t  const  cmd_goto3;
extern const parse_pgm_inst_t  const  cmd_position;
extern const parse_pgm_inst_t  const  cmd_position_set;
extern const parse_pgm_inst_t  const  cmd_strat_info;
extern const parse_pgm_inst_t  const  cmd_strat_conf;
extern const parse_pgm_inst_t  const  cmd_strat_conf2;
extern const parse_pgm_inst_t  const  cmd_strat_conf3;
extern const parse_pgm_inst_t  const  cmd_subtraj;
extern const parse_pgm_inst_t  const  cmd_motion;

/* in progmem */
parse_pgm_ctx_t const main_ctx[] = {

	/* commands_ax12.c */
	(const parse_pgm_inst_t  *) & cmd_ax12_stress,
	(const parse_pgm_inst_t  *) & cmd_ax12_baudrate,
	(const parse_pgm_inst_t  *) & cmd_ax12_read,
	(const parse_pgm_inst_t  *) & cmd_ax12_write,

	/* commands_gen.c */
	(const parse_pgm_inst_t  *) & cmd_reset,
	(const parse_pgm_inst_t  *) & cmd_bootloader,
	(const parse_pgm_inst_t  *) & cmd_buzzer,
	(const parse_pgm_inst_t  *) & cmd_encoders,
	(const parse_pgm_inst_t  *) & cmd_pwm,
	(const parse_pgm_inst_t  *) & cmd_adc,
	(const parse_pgm_inst_t  *) & cmd_sensor,
	(const parse_pgm_inst_t  *) & cmd_log,
	(const parse_pgm_inst_t  *) & cmd_log_show,
	(const parse_pgm_inst_t  *) & cmd_log_type,
	(const parse_pgm_inst_t  *) & cmd_stack_space,
	(const parse_pgm_inst_t  *) & cmd_scheduler,
	(const parse_pgm_inst_t  *) & cmd_ir,

	/* commands_cs.c */
	(const parse_pgm_inst_t  *) & cmd_gain,
	(const parse_pgm_inst_t  *) & cmd_gain_show,
	(const parse_pgm_inst_t  *) & cmd_speed,
	(const parse_pgm_inst_t  *) & cmd_speed_show,
	(const parse_pgm_inst_t  *) & cmd_consign,
	(const parse_pgm_inst_t  *) & cmd_derivate_filter,
	(const parse_pgm_inst_t  *) & cmd_derivate_filter_show,
	(const parse_pgm_inst_t  *) & cmd_maximum,
	(const parse_pgm_inst_t  *) & cmd_maximum_show,
	(const parse_pgm_inst_t  *) & cmd_quadramp,
	(const parse_pgm_inst_t  *) & cmd_quadramp_show,
	(const parse_pgm_inst_t  *) & cmd_cs_status,
	(const parse_pgm_inst_t  *) & cmd_blocking_i,
	(const parse_pgm_inst_t  *) & cmd_blocking_i_show,

	/* commands_mainboard.c */
	(const parse_pgm_inst_t  *) & cmd_event,
	(const parse_pgm_inst_t  *) & cmd_spi_test,
	(const parse_pgm_inst_t  *) & cmd_start,
	(const parse_pgm_inst_t  *) & cmd_interact,
	(const parse_pgm_inst_t  *) & cmd_color,
	(const parse_pgm_inst_t  *) & cmd_rs,

	/* commands_traj.c */
	(const parse_pgm_inst_t  *) & cmd_traj_speed,
	(const parse_pgm_inst_t  *) & cmd_traj_speed_show,
	(const parse_pgm_inst_t  *) & cmd_trajectory,
	(const parse_pgm_inst_t  *) & cmd_trajectory_show,
	(const parse_pgm_inst_t  *) & cmd_rs_gains,
	(const parse_pgm_inst_t  *) & cmd_rs_gains_show,
	(const parse_pgm_inst_t  *) & cmd_track,
	(const parse_pgm_inst_t  *) & cmd_track_show,
	(const parse_pgm_inst_t  *) & cmd_pt_list,
	(const parse_pgm_inst_t  *) & cmd_pt_list_append,
	(const parse_pgm_inst_t  *) & cmd_pt_list_del,
	(const parse_pgm_inst_t  *) & cmd_pt_list_show,
	(const parse_pgm_inst_t  *) & cmd_goto1,
	(const parse_pgm_inst_t  *) & cmd_goto2,
	(const parse_pgm_inst_t  *) & cmd_position,
	(const parse_pgm_inst_t  *) & cmd_position_set,
	(const parse_pgm_inst_t  *) & cmd_strat_info,
	(const parse_pgm_inst_t  *) & cmd_strat_conf,
	(const parse_pgm_inst_t  *) & cmd_strat_conf2,
	(const parse_pgm_inst_t  *) & cmd_strat_conf3,
	(const parse_pgm_inst_t  *) & cmd_subtraj,
	(const parse_pgm_inst_t  *) & cmd_motion,
	NULL,
};
