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

#include <aversive.h>
#include <aversive/list.h>
#include <aversive/error.h>

#include <ax12.h>
#include <clock_time.h>
#include <uart.h>

#include "ax12_user.h"
#include "main.h"

/*
 * Cmdline interface for AX12. Use the PC to command a daisy-chain of
 * AX12 actuators with a nice command line interface.
 * 
 * The circuit should be as following:
 *
 *    |----------|
 *    |	    uart0|------->--- PC (baudrate=57600)
 *    |		 |-------<---
 *    |	atmega128|
 *    |		 |
 *    |	    uart2|---->---+-- AX12 (baudrate 57600)
 *    |		 |----<---| 
 *    |----------|
 *
 * Note that RX and TX pins of UART2 are connected together to provide
 * a half-duplex UART emulation.
 *
 */

#define UART_AX12_NUM		2
#define UCSRxB			UCSR2B

/*
 * AX12 response timeout, in us. The reception code uses the software us2 timer
 * to check for timeouts, so the real timeout value will depend on the timer
 * precision. If we start waiting right before the timer task gets scheduled,
 * the timeout value will be effectively rounded down to the closest multiple of
 * TIME_PRECISION (or more accurately to TIME_PRECISION rounded down to the
 * closest multiple of SCHEDULER_UNIT).
 *
 * TIME_PRECISION is set to 25ms, use the same timeout value.
 */
#define AX12_TIMEOUT		25000UL

/* for atmega256 */
#ifndef TXEN
#define TXEN			TXEN2
#endif

/*
 * We use synchronous access (not interrupt driven) to the hardware UART,
 * because we have to be sure that the transmission/reception is really finished
 * when we return from the functions.
 *
 * We don't use the CM-5 circuit as described in the AX12 documentation, we
 * simply connect TX and RX and use TXEN + RXEN + DDR to manage the port
 * directions.
 */

static volatile uint8_t ax12_state = AX12_STATE_READ;
extern struct cirbuf g_tx_fifo[];	/* uart fifo */
static volatile uint8_t ax12_nsent = 0;

/* Called by ax12 module to send a character on serial line. Count the number of
 * transmitted bytes. It will be used in ax12_recv_char() to drop the bytes that
 * we transmitted.
 */
static int8_t ax12_send_char(uint8_t c)
{
	uart_send(UART_AX12_NUM, c);
	ax12_nsent++;
	return 0;
}

/* Called by uart module when the character has been written in UDR. It does not
 * mean that the byte is physically transmitted.
 */
static void ax12_send_callback(char c)
{
	if (ax12_state == AX12_STATE_READ) {
		/* disable TX when last byte is pushed. */
		if (CIRBUF_IS_EMPTY(&g_tx_fifo[UART_AX12_NUM]))
			UCSRxB &= ~(1 << TXEN);
	}
}

/* Called by ax12 module when we want to receive a char. Note that we also
 * receive the bytes we sent ! So we need to drop them.
 */
static int16_t ax12_recv_char(void)
{
	microseconds t = time_get_us2();
	int c;

	while (1) {
		c = uart_recv_nowait(UART_AX12_NUM);
		if (c != -1) {
			if (ax12_nsent == 0)
				return c;
			ax12_nsent--;
		}

		/* 5 ms timeout */
		if ((time_get_us2() - t) > AX12_TIMEOUT)
			return -1;
	}
	return c;
}

/* Called by ax12 module when we want to switch serial line. As we work in
 * interruption mode, this function can be called to switch back in read mode
 * even if the bytes are not really transmitted on the line. That's why in this
 * case we do nothing, we will fall back in read mode in any case when xmit is
 * finished -- see ax12_send_callback().
 */
static void ax12_switch_uart(uint8_t state)
{
	uint8_t flags;

	if (state == AX12_STATE_WRITE) {
		IRQ_LOCK(flags);
		ax12_nsent = 0;
		while (uart_recv_nowait(UART_AX12_NUM) != -1) ;
		UCSRxB |= 1 << TXEN;
		ax12_state = AX12_STATE_WRITE;
		IRQ_UNLOCK(flags);
	} else {
		IRQ_LOCK(flags);
		if (CIRBUF_IS_EMPTY(&g_tx_fifo[UART_AX12_NUM]))
			UCSRxB &= ~(1 << TXEN);
		ax12_state = AX12_STATE_READ;
		IRQ_UNLOCK(flags);
	}
}

void ax12_port_init(struct AX12 *ax12)
{
	AX12_init(ax12);
	AX12_set_hardware_send(ax12, ax12_send_char);
	AX12_set_hardware_recv(ax12, ax12_recv_char);
	AX12_set_hardware_switch(ax12, ax12_switch_uart);
	uart_register_tx_event(UART_AX12_NUM, ax12_send_callback);
}
