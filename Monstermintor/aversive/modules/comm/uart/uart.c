/*  
 *  Copyright Droids Corporation, Microb Technology, Eirbot (2005)
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
 *  Revision : $Id: uart.c,v 1.33.4.7 2009-01-23 23:08:42 zer0 Exp $
 *
 */

/* Olivier MATZ, Droids-corp 2004 - 2009 */

#include <aversive.h>
#include <aversive/list.h>

#include <uart.h>
#include <uart_defs.h>
#include <uart_private.h>

struct cirbuf g_tx_fifo[UART_HW_NUM];
struct cirbuf g_rx_fifo[UART_HW_NUM];

/* global vars are initialized to 0 (NULL) */
event *rx_event[UART_HW_NUM];
event *tx_event[UART_HW_NUM];

const struct regs uart_regs[UART_HW_NUM] = {
#ifdef UDR0
        {
                .udr = &UDR0,
                .ucsra = &UCSR0A,
                .ucsrb = &UCSR0B,
                .ucsrc = &UCSR0C,
                .ubrrl = &UBRR0L,
                .ubrrh = &UBRR0H,
        },
#endif
#ifdef UDR1
        {
                .udr = &UDR1,
                .ucsra = &UCSR1A,
                .ucsrb = &UCSR1B,
                .ucsrc = &UCSR1C,
                .ubrrl = &UBRR1L,
                .ubrrh = &UBRR1H,
        },
#endif
#ifdef UDR2
        {
                .udr = &UDR2,
                .ucsra = &UCSR2A,
                .ucsrb = &UCSR2B,
                .ucsrc = &UCSR2C,
                .ubrrl = &UBRR2L,
                .ubrrh = &UBRR2H,
        },
#endif
#ifdef UDR3
        {
                .udr = &UDR3,
                .ucsra = &UCSR3A,
                .ucsrb = &UCSR3B,
                .ucsrc = &UCSR3C,
                .ubrrl = &UBRR3L,
                .ubrrh = &UBRR3H,
        },
#endif
};

/**
 * This is the interruption function which occurs when the entire
 * frame in the transmit shift register has been shifted out and 
 * there is no new data in the transmit buffer.
 */
#ifdef UART0_COMPILE
#ifndef USART0_UDRE_vect
#if defined(USART_UDRE_vect)
#define USART0_UDRE_vect USART_UDRE_vect
#elif defined(SIG_USART0_DATA)
#define USART0_UDRE_vect SIG_USART0_DATA
#elif defined(SIG_UART0_DATA)
#define USART0_UDRE_vect SIG_UART0_DATA
#endif
#endif
SIGNAL(USART0_UDRE_vect)
{
	uart_send_next_char(0);
}
#endif
#ifdef UART1_COMPILE
#ifndef USART1_UDRE_vect
#if defined(SIG_USART1_DATA)
#define USART1_UDRE_vect SIG_USART1_DATA
#elif defined(SIG_UART1_DATA)
#define USART1_UDRE_vect SIG_UART1_DATA
#endif
#endif
SIGNAL(USART1_UDRE_vect)
{
	uart_send_next_char(1);
}
#endif
#ifdef UART2_COMPILE
#ifndef USART2_UDRE_vect
#if defined(SIG_USART2_DATA)
#define USART2_UDRE_vect SIG_USART2_DATA
#elif defined(SIG_UART2_DATA)
#define USART2_UDRE_vect SIG_UART2_DATA
#endif
#endif
SIGNAL(USART2_UDRE_vect)
{
	uart_send_next_char(2);
}
#endif
#ifdef UART3_COMPILE
#ifndef USART3_UDRE_vect
#if defined(SIG_USART3_DATA)
#define USART3_UDRE_vect SIG_USART3_DATA
#elif defined(SIG_UART3_DATA)
#define USART3_UDRE_vect SIG_UART3_DATA
#endif
#endif
SIGNAL(USART3_UDRE_vect)
{
	uart_send_next_char(3);
}
#endif

static void uart_recv_next_char(uint8_t num);

/**
 * This is the interruption function which occurs when there is
 * a new unread data in the reception buffer.
 */
#ifdef UART0_COMPILE
#ifndef USART0_RXC_vect
#if defined(USART_RXC_vect)
#define USART0_RXC_vect USART_RXC_vect
#elif defined(USART_RX_vect)
#define USART0_RXC_vect USART_RX_vect
#elif defined(USART0_RX_vect)
#define USART0_RXC_vect USART0_RX_vect
#else
#if defined(SIG_USART0_RECV)
#define USART0_RXC_vect SIG_USART0_RECV
#elif defined(SIG_UART0_RECV)
#define USART0_RXC_vect SIG_UART0_RECV
#endif
#endif
#endif
SIGNAL(USART0_RXC_vect)
{
	uart_recv_next_char(0);
}
#endif
#ifdef UART1_COMPILE
#ifndef USART1_RXC_vect
#if defined(USART1_RX_vect)
#define USART1_RXC_vect USART1_RX_vect
#else
#if defined(SIG_USART1_RECV)
#define USART1_RXC_vect SIG_USART1_RECV
#elif defined(SIG_UART1_RECV)
#define USART1_RXC_vect SIG_UART1_RECV
#endif
#endif
#endif
SIGNAL(USART1_RXC_vect)
{
	uart_recv_next_char(1);
}
#endif
#ifdef UART2_COMPILE
#ifndef USART2_RXC_vect
#if defined(USART2_RX_vect)
#define USART2_RXC_vect USART2_RX_vect
#else
#if defined(SIG_USART2_RECV)
#define USART2_RXC_vect SIG_USART2_RECV
#elif defined(SIG_UART2_RECV)
#define USART2_RXC_vect SIG_UART2_RECV
#endif
#endif
#endif
SIGNAL(USART2_RXC_vect)
{
	uart_recv_next_char(2);
}
#endif
#ifdef UART3_COMPILE
#ifndef USART3_RXC_vect
#if defined(USART3_RX_vect)
#define USART3_RXC_vect USART3_RX_vect
#else
#if defined(SIG_USART3_RECV)
#define USART3_RX_vect SIG_USART3_RECV
#elif defined(SIG_UART3_RECV)
#define USART3_RX_vect SIG_UART3_RECV
#endif
#endif
#endif
SIGNAL(USART3_RXC_vect)
{
	uart_recv_next_char(3);
}
#endif


/** 
 * transmit next character of fifo if any, and call the event function.
 * This function is executed with intr locked.
 */
void uart_send_next_char(uint8_t num)
{
#ifdef CONFIG_MODULE_UART_9BITS
	if (uart_getconf_nbits(num) == 9) {
		int elt = 0;

		/* for 9 bits, it uses 2 places in the fifo */
		if (CIRBUF_GET_LEN(&g_tx_fifo[num]) < 2) {
			cbi(*uart_regs[num].ucsrb, UDRIE);
			return;
		}

		cirbuf_get_buf_tail(&g_tx_fifo[num], (char *)&elt, 2);
		cirbuf_del_buf_tail(&g_tx_fifo[num], 2);

		uart_set_udr_9bits(num, elt);
		sbi(*uart_regs[num].ucsrb, UDRIE);
	}
	else /* 5, 6, 7 or 8 bits */
#endif /* CONFIG_MODULE_UART_9BITS */
	{ 
		char elt = 0;

		if (CIRBUF_IS_EMPTY(&g_tx_fifo[num])) {
			cbi(*uart_regs[num].ucsrb, UDRIE);
			return;
		}

		elt = cirbuf_get_tail(&g_tx_fifo[num]);
		cirbuf_del_tail(&g_tx_fifo[num]);
		uart_set_udr(num, elt);
		sbi(*uart_regs[num].ucsrb, UDRIE);
	}
}

/**
 * UART RX Interrupt
 */
static void uart_recv_next_char(uint8_t num)
{
#ifdef CONFIG_MODULE_UART_9BITS
	if (uart_getconf_nbits() == 9) {
		int elt = 0;

		elt = uart_get_udr_9bits(num);
		if (CIRBUF_GET_FREELEN(&g_rx_fifo[num]) >= 2) {
			cirbuf_add_buf_head(&g_rx_fifo[num], (char *)&elt, 2);
		}

		if (rx_event[num])
			((event_9bits *)rx_event[num])(elt);
	}
	else 
#endif /* CONFIG_MODULE_UART_9BITS */
	{
		char elt = 0;

		elt = uart_get_udr(num);
		if (!CIRBUF_IS_FULL(&g_rx_fifo[num])) {
			cirbuf_add_head(&g_rx_fifo[num], elt);
		}

		if (rx_event[num])
			rx_event[num](elt);
	}
}

/* init all uarts */
void uart_init(void)
{
#if (defined UDR0) && (defined UART0_COMPILE)
	uart_setconf(0, NULL);
#endif

#if (defined UDR1) && (defined UART1_COMPILE)
	uart_setconf(1, NULL);
#endif

#if (defined UDR2) && (defined UART2_COMPILE)
	uart_setconf(2, NULL);
#endif

#if (defined UDR3) && (defined UART3_COMPILE)
	uart_setconf(3, NULL);
#endif
}
