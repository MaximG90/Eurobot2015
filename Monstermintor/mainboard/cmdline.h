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

#ifndef __CMDLINE_H__
#define __CMDLINE_H__

#include <aversive.h>

#include <uart.h>

#define CMDLINE_UART 0

/* uart rx callback for reset() */
void emergency(char c);

/* log function */
void mylog(struct error *e, ...);

/* launch cmdline */
int cmdline_interact(void);
#ifndef HOST_VERSION
static inline uint8_t cmdline_keypressed(void)
{
	return (uart_recv_nowait(CMDLINE_UART) != -1);
}
#else
static inline uint8_t cmdline_keypressed(void)
{
	return (uart_recv_nowait(CMDLINE_UART) != -1);
}
#endif
static inline int16_t cmdline_getchar(void)
{
	return uart_recv_nowait(CMDLINE_UART);
}

static inline uint8_t cmdline_getchar_wait(void)
{
	return uart_recv(CMDLINE_UART);
}

void cmdline_interrupt(void);

#endif /* __CMDLINE_H__ */
