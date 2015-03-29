/*
 *  Copyright Droids Corporation
 *  Olivier Matz <zer0@droids-corp.org>
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

#ifndef __CS_H__
#define __CS_H__

#include <control_system_manager.h>
#include <pid.h>

void monstravr_cs_init(void);
void dump_flags(void);
void dump_cs(const char *name, struct cs *cs);
void dump_cs_debug(const char *name, struct cs *cs);
void dump_cs_simple(struct cs *cs);
void dump_pid(const char *name, struct pid_filter *pid);

#endif /* __CS_H__ */
