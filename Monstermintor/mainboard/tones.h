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

#ifndef __TONES_H__
#define __TONES_H__

/* Tones
 *
 * PJ(3)(4)
 * 0  0 => none
 * 0  1 => DO  (261,63 Hz)
 * 1  0 => SOL (not implemented)
 * 1  1 => LA  (440 Hz)
 */

#define TONE_BUZ1		_BV(4)
#define TONE_BUZ2		_BV(3)
#define TONE_MASK		(TONE_BUZ1 | TONE_BUZ2)

#define DO_TONE			TONE_BUZ1
#define SOL_TONE		TONE_BUZ2
#define LA_TONE			(TONE_BUZ1 | TONE_BUZ2)
#define NONE_TONE		~(TONE_BUZ1 | TONE_BUZ2)

#endif /* __TONES_H__ */
