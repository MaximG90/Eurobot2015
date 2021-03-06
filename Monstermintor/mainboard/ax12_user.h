/*  
 *  Copyright Droids Corporation
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

#ifndef __AX12_SERVO_H__
#define __AX12_SERVO_H__

#include <ax12.h>

/* This is the ax12 user interface. It initializes the aversive AX12
 * module so that it works in background, using interrupt driver uart.
 *
 * Be carreful, a call to AX12 module is synchronous and uses
 * interruptions, so interrupts must be enabled. On the other side, a
 * call _must not_ interrupt another one. That's why all calls to the
 * module are done either in init() functions or in a scheduler event
 * with prio=ARM_PRIO.
 */

void ax12_port_init(struct AX12 *ax12);

#endif /* __AX12_USER_H__ */
