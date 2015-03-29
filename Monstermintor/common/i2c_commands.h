/*
 *  Copyright Droids Corporation (2007)
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

#ifndef _I2C_COMMANDS_H_
#define _I2C_COMMANDS_H_

#define I2C_MAINBOARD_ADDR	1
#define I2C_MECHBOARD_ADDR	2
#define I2C_SENSORBOARD_ADDR	3

struct i2c_cmd_hdr {
	uint8_t cmd;
};

/** for all board / generic command **/

#define I2C_CMD_GENERIC_LED_CONTROL	0x00

struct i2c_cmd_led_control {
	struct i2c_cmd_hdr hdr;
	uint8_t led_num:7;
	uint8_t state:1;
};

/****/

#define I2C_CMD_GENERIC_SET_COLOR	0x01

struct i2c_cmd_generic_color {
	struct i2c_cmd_hdr hdr;
	uint8_t color;
};

/** for sensorboard only **/

#define I2C_CMD_SENSORBOARD_SET_BEACON	0x04

struct i2c_cmd_sensorboard_start_beacon {
	struct i2c_cmd_hdr hdr;
	uint8_t enable;
};

#define I2C_REQ_SENSORBOARD_STATUS	0x82
/** send to sensorboard **/
struct i2c_req_sensorboard_status {
	struct i2c_cmd_hdr hdr;

	/* position sent by mainboard */
	int16_t x;
	int16_t y;
	int16_t a;
};

#define I2C_ANS_SENSORBOARD_STATUS	0x83

/** refresh by sensorboard **/
struct i2c_ans_sensorboard_status {
	struct i2c_cmd_hdr hdr;

	uint8_t status;
#define I2C_OPPONENT_NOT_THERE		-1000
	int16_t opponent_x;
	int16_t opponent_y;
	int16_t opponent_a;
	int16_t opponent_d;
};

#endif /* _I2C_COMMANDS_H_ */
