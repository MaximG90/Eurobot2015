/*
 *  Copyright Droids Corporation, Microb Technology, Eirbot (2008)
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

#ifndef __STRAT_H__
#define __STRAT_H__

#include <stdbool.h>

#include <aversive.h>

/*
 * Positions are measured in a frame based on a cartesian coordinate system
 * centered on the top left corner. Angles are measured in the
 * trigonometric direction.
 *
 *                                     x <----o
 *    +------+===+===+---+---+===+===+------+ |
 *    |      | 5 |   |   |   |   |   |      | |
 *    |      +---+---+---+---+---+---+      | v
 *    |      | 4 | ° |   |   | ° |   |      | y
 *    |      +---+---+---+---+---+---+      |
 *    |      | 3 |   |   |   |   |   |      |  // PAS A JOUR !
 *    |      +---+---+---+---+---+---+      |
 *  y |      | 2 | ° |   |   | ° |   |      |
 *  ^ |      +---+---+---+---+---+---+      |
 *  | |------+ 1 |   |   |   |   |   +------|
 *  | |      +---+---+---+---+---+---+      |
 *  | | roug | 0 | 1 | 2 | 3 | 4 | 5 | red  |
 *  | +------+---+---+---+---+---+---+------+
 *  o---------->  x
 *
 *  To keep the coordinates system simple, the trigonometric direction must be
 *  preserved when the robot starts in the green or yellow area. If we kept the same axis
 *  origin, angles would have to be inverted for relative displacements and
 *  subtracted from pi for absolute displacements. This makes software
 *  development error prone. To avoid this, we move the coordinate system origin
 *  to the opposite corner instead. Relative and absolute angles are then both
 *  inverted.
 */

#define ROBOT_COLOR_DEFAULT_NAME	"yellow"
#define ROBOT_COLOR_ALTERNATE_NAME	"green"

/* convert coords according to our color */
#define COLOR_X(x)			(x)
#define COLOR_Y(y)			((mainboard.our_color==ROBOT_COLOR_DEFAULT)? (y) : (AREA_Y-(y)))
#define COLOR_A(a)			((mainboard.our_color==ROBOT_COLOR_DEFAULT)? (a) : (-a))

/* offset for the dissymetry of the coders */
#define OFFSET_Y(pos)			((mainboard.our_color==ROBOT_COLOR_DEFAULT)? (pos) : (pos-10))

/* area */
#define AREA_X				3000
#define AREA_Y				2000

#define START_X				(ROBOT_BACK_LENGTH + 70) 
#define START_Y				1000 // pour calibration
#define START_A				0

#define SQUARE_SIZE			250
#define SQUARE_CENTER_X(x)		((x)*SQUARE_SIZE + SQUARE_SIZE/2)
#define SQUARE_CENTER_Y(y)		((y)*SQUARE_SIZE + SQUARE_SIZE/2)

#define SQUARE_NUMBER_X(x)		((x)/ SQUARE_SIZE)
#define SQUARE_NUMBER_Y(y)		((y) / SQUARE_SIZE)

//#define PAWS_DIAM			200

/* default speeds / max 2700 */
#define SPEED_DIST_FAST			3000
#define SPEED_ANGLE_FAST		3000
#define SPEED_DIST_MIDDLE_FAST		2000
#define SPEED_ANGLE_MIDDLE_FAST		2000
#define SPEED_DIST_SLOW			1500
#define SPEED_ANGLE_SLOW		1500
#define SPEED_DIST_VERY_SLOW		350
#define SPEED_ANGLE_VERY_SLOW		350

#define ID_AX12_LEFT 	2
#define ID_AX12_RIGHT	1

/*
 * strat_info - strategy information
 */
enum strat_mode {
	STRAT_MODE_DEFAULT = 0,
	STRAT_MODE_HOMOLOGATION = 1,
};

#define STRAT_OPTION_CLEAR_DISTRIBUTION_ZONE	(1 << 0)
#define STRAT_OPTION_USE_ARMS			(1 << 1)
#define STRAT_OPTION_DETECT_OBSTACLE		(1 << 2)
#define STRAT_OPTION_PUNCH_PAWNS		(1 << 3)
#define STRAT_OPTION_RESET_POSITION		(1 << 4)
#define STRAT_OPTION_WAIT_START			(1 << 5)

/*
 * Other
 */

struct strat_info {
	uint8_t dump_enabled;
	bool game_started;

	struct {
		enum strat_mode mode;
		uint8_t options;
	} conf;
};

extern struct strat_info strat_info;

void strat_dump_info(const char *caller);
void strat_dump_conf(void);
void strat_reset_info(void);
void strat_start(void);
void strat_dump_flags(void);
void strat_auto_position(void);

#endif /* __STRAT_H__ */
