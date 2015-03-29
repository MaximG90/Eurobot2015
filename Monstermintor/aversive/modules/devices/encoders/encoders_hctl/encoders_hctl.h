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
 *
 *  Revision : $Id: encoders_hctl.h,v 1.1.2.1 2009-02-20 20:24:21 zer0 Exp $
 *
 *  Olivier MATZ <zer0@droids-corp.org>
 */

#ifndef _encoders_hctl_H_
#define _encoders_hctl_H_


 // 2032 Moteurs
#define XY   A1
#define OE   A5 
#define EN1  A2
#define EN2  A3
#define SEL1 A4
#define SEL2 A0
#define RSTX A7 
#define RSTY A6

 // 2022 Lift
#define OE2   3 
#define SEL12 4
#define SEL22 5
#define RST2 2 


/** 
 * Initialisation of encoders, variables
 */
void encoders_hctl_init(void);


/** 
 * Extract encoder value.
 */
int32_t encoders_hctl_get_value(void *encoder);

/** 
 * Set an encoder value
 */
void encoders_hctl_set_value(void *encoder, int32_t val);

#endif
