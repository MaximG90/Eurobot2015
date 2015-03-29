/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 */

#ifndef __LIFT_H__
#define __LIFT_H__

#include <stdbool.h>

void lift_init(void);
void lift_start(void);
void lift_stop(void);
void lift_drop(void);
bool lift_is_stopped(void);
bool lift_is_empty(void);
bool lift_is_full(void);

#endif /* __LIFT_H__ */
