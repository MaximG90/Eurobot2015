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

#ifndef __LEDS_H__
#define __LEDS_H__

#define LED_TOGGLE(port, bit) do {		\
		if (port & _BV(bit))		\
			port &= ~_BV(bit);	\
		else				\
			port |= _BV(bit);	\
	} while(0)

#define LED_BLUE_ON() 	sbi(PORTD, 7) // LED : Alive
#define LED_BLUE_OFF() 	cbi(PORTD, 7) // Pin 38
#define LED_BLUE_TOGGLE() 	LED_TOGGLE(PORTD, 7)

#define LED2_ON() 	sbi(PORTG, 1) // Pin 40
#define LED2_OFF() 	cbi(PORTG, 1)
#define LED2_TOGGLE() 	LED_TOGGLE(PORTG, 1)

#define LED3_ON() 	sbi(PORTL, 7) // Pin 42
#define LED3_OFF() 	cbi(PORTL, 7)
#define LED3_TOGGLE() 	LED_TOGGLE(PORTL, 7)

#define LED4_ON() 	sbi(PORTL, 5) // Pin 44
#define LED4_OFF() 	cbi(PORTL, 5)
#define LED4_TOGGLE() 	LED_TOGGLE(PORTL, 5)

#define LED5_ON() 	sbi(PORTL, 2) // Pin 47
#define LED5_OFF() 	cbi(PORTL, 2)
#define LED5_TOGGLE() 	LED_TOGGLE(PORTL, 2)

#define LED6_ON() 	sbi(PORTL, 6) // Pin 43
#define LED6_OFF() 	cbi(PORTL, 6)
#define LED6_TOGGLE() 	LED_TOGGLE(PORTL, 6)

#define LED7_ON() 	sbi(PORTG, 0) // Pin 41
#define LED7_OFF() 	cbi(PORTG, 0)
#define LED7_TOGGLE() 	LED_TOGGLE(PORTG, 0)

#define LED8_ON() 	sbi(PORTG, 2) // Pin 39
#define LED8_OFF() 	cbi(PORTG, 2)
#define LED8_TOGGLE() 	LED_TOGGLE(PORTG, 2)

#endif /* __LEDS_H__ */
