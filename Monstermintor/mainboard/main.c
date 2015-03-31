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
#include "Arduino.h"
#include <stdio.h>
#include <string.h>
#include <aversive.h>
#include <aversive/pgmspace.h>
#include <aversive/queue.h>
#include <aversive/wait.h>
#include <aversive/error.h>
#include <avr/eeprom.h>
#include <trajectory_manager.h>
#include <vect_base.h>
#include <polygon.h>
#include <pwm_ng.h>
#include <clock_time.h>
#include <encoders_hctl.h>
#include <scheduler.h>
#include <timer.h>
#include <uart.h>
#include <adc.h>
#include "../common/eeprom_mapping.h"
#include "actuator.h"
#include "ax12_user.h"
#include "cmdline.h"
#include "cs.h"
#include "eeprom.h"
#include "leds.h"
#include "lift.h"
#include "main.h"
#include "motion.h"
#include "motor.h"
#include "obstacle.h"
#include "robot.h"
#include "sensor.h"
#include "strat.h"
#include "tones.h"

#if __AVR_LIBC_VERSION__ == 10602UL
#error "won't work with this version"
#endif

struct genboard gen;
struct mainboard mainboard;

void do_led_blink(void *dummy)
{
	LED_BLUE_TOGGLE();
}

static void main_timer_interrupt(void)
{
	static uint8_t cpt = 0;

	cpt++;
	if ((cpt & 0x3) == 0)
		scheduler_interrupt();
}

static void io_init(void)
{
	/* Brake */
	BRAKE_DDR();
	BRAKE_OFF();

	/* PWM timers */
	TCCR1A = 0;
	TCCR3A = 0;
	TCCR4A = 0;
	TCCR5A = 0;

	/* Buzzer */
	//DDRJ |= TONE_BUZ1 | TONE_BUZ2;
	//PORTJ &= ~TONE_MASK;

	/* LEDS */
	
		

	/* Pull-up on the AX12 data bus */
	PORTH |= 1 << 1;

	LED_BLUE_OFF();
	LED2_OFF();
	LED3_OFF();
	LED4_OFF();
	LED5_OFF();
	LED6_OFF();
	LED7_OFF();
	LED8_OFF();
}

static void motors_init(void)
{
	/* DC motors PWM
	 *
	 * 10-bit PWM in Phase Correct mode
	 * 16 Mhz / 1024(=2^10) / 2
	 * PRESCALER_DIV_1	-> 7812,5 Hz
	 * PRESCALER_DIV_8	->  976,5 Hz
	 * PRESCALER_DIV_64 ->	122,2 Hz
	 */
	PWM_NG_TIMER_16BITS_INIT(4, TIMER_16_MODE_PWM_PC_10, TIMER1_PRESCALER_DIV_1);
	PWM_NG_TIMER_16BITS_INIT(1, TIMER_16_MODE_PWM_PC_10, TIMER4_PRESCALER_DIV_8);

	PWM_NG_INIT16(&gen.pwm1_4A, 4, A, 10, PWM_NG_MODE_SPECIAL_SIGN | PWM_NG_MODE_SIGNED, &PORTH, 5);
	PWM_NG_INIT16(&gen.pwm2_4B, 4, B, 10, PWM_NG_MODE_SPECIAL_SIGN | PWM_NG_MODE_SIGNED, &PORTH, 6);
	PWM_NG_INIT16(&gen.pwm3_1A, 1, A, 10, PWM_NG_MODE_SPECIAL_SIGN | PWM_NG_MODE_SIGNED, &PORTB, 4);
	pwm_ng_set(&gen.pwm1_4A, 0);
	pwm_ng_set(&gen.pwm2_4B, 0);
	pwm_ng_set(&gen.pwm3_1A, 0);
	
}

static void print_banner(void)
{
	unsigned int i;

	printf_P(PSTR("\r\n"));
	printf_P(PSTR("\r\n"));
	for (i = 0; i < 80; i++)
		printf_P(PSTR("*"));
	printf_P(PSTR("\r\n\t\t        Mainboard Monsterminator\n\r"));
	for (i = 0; i < 80; i++)
		printf_P(PSTR("*"));
	printf_P(PSTR("\r\n"));
	printf_P(PSTR("\t                Compiled on %s at %s\n\r"), __DATE__, __TIME__);
	printf_P(PSTR("\t                with GCC version (%s)\n\r"), __VERSION__);
	printf_P(PSTR("\t                AVR_LIBC version (%d.%d.%d)\n\r"), __AVR_LIBC_MAJOR__,
		 __AVR_LIBC_MINOR__, __AVR_LIBC_REVISION__);


printf_P(PSTR("	   ###################################################    \r\n"));
printf_P(PSTR("	   ###################################################    \r\n"));
printf_P(PSTR("	   ###################################################    \r\n"));
printf_P(PSTR("	   ###################################################    \r\n"));
printf_P(PSTR("	   ###################################################    \r\n"));
printf_P(PSTR("	   ###################################################    \r\n"));
printf_P(PSTR("	   ###################################################    \r\n"));
printf_P(PSTR("	   #############  #######      #######################    \r\n"));
printf_P(PSTR("	   ###########      #####       ###   ################    \r\n"));
printf_P(PSTR("	   #########        ####         ####   ##############    \r\n"));
printf_P(PSTR("	   #########      #####     #    ######   ############    \r\n"));
printf_P(PSTR("	   #########       ####    ###    ###      ###########    \r\n"));
printf_P(PSTR("	   ########   ###    #    #####    #    ##  ##########    \r\n"));
printf_P(PSTR("	   ###############       #######      ###### #########    \r\n"));
printf_P(PSTR("	   #################    #########   ##################    \r\n"));
printf_P(PSTR("	   #################      #####  #   #################    \r\n"));
printf_P(PSTR("	   ################   #    ##    ##   ################    \r\n"));
printf_P(PSTR("	   ###############   ####      #####  ################    \r\n"));
printf_P(PSTR("	   ###############  ######     ######  ###############    \r\n"));
printf_P(PSTR("	   ##############  #####    #    #####  ##############    \r\n"));
printf_P(PSTR("	   #############  #####   ####    ##### ##############    \r\n"));
printf_P(PSTR("	   ############  ####################### #############    \r\n"));
printf_P(PSTR("	   ############ ########################  ############    \r\n"));
printf_P(PSTR("	   ########### ##########################  ###########    \r\n"));
printf_P(PSTR("	   ########## ############################ ###########    \r\n"));
printf_P(PSTR("	   ########## ############################ ###########    \r\n"));
printf_P(PSTR("	   ###################################################    \r\n"));
printf_P(PSTR("	   ###################################################    \r\n"));
printf_P(PSTR("	    #################################################     \r\n"));
printf_P(PSTR("	     ###############################################      \r\n"));
printf_P(PSTR("	      ############################################        \r\n"));
printf_P(PSTR("	                         #######                          \r\n"));
printf_P(PSTR("	                           ###                            \r\n"));
printf_P(PSTR("	                            #                             \r\n"));

}


/*
	for (int i = 0; i<10;i++)
	{
		uint8_t test;
		test = AX12_write_byte(&right_ax12, 2, AA_ID, 4);
		printf("check %d \n\r", test);
		uint8_t val;
		uint8_t returnid;
		returnid= AX12_read_byte(&right_ax12, i, AA_ID, &val);

		printf("ID : %d \n \r", i);
		printf("Value : %d \n \r", returnid);
		wait_ms(200);
	}

	for (int i=0;i<200;i++)
	{
		guide_set_position(ARM_POSITION_LEFT(i));
		wait_ms(1000);
		printf("check boucle \n\r");
		i+=9;
	}
*/

/* -----------------------------------------------------------------------------
 * Main
 */

int main(void)
{
	/* ATMega I/O */
	io_init();
	
	/* Arduino timer 0 */
	init();
	
	memset(&gen, 0, sizeof(gen));
	memset(&mainboard, 0, sizeof(mainboard));
	mainboard.flags = DO_ENCODERS | DO_RS | DO_POS | DO_POWER | DO_BD;

	/* UARTs */
	uart_init();

	/* Console with command line */
	fdevopen(uart0_dev_send, uart0_dev_recv);
	uart_register_rx_event(0, emergency);

	/* Logs */
	error_register_emerg(mylog);
	error_register_error(mylog);
	error_register_warning(mylog);
	error_register_notice(mylog);
	error_register_debug(mylog);

	/* Load configuration parameters */
	eeprom_read_config();

	/* DC and servo motors */
	motors_init();
	ax12_port_init(&gen.ax12);

	/* Timers */
	timer_init();
	timer0_register_OV_intr(main_timer_interrupt);

	/* Time and scheduler */
	scheduler_init();
	time_init(TIME_PRIO);

	/* Sensors and encoders (SPI) */
	sensor_init();
	encoders_hctl_init();

	/* All CS management */
	monstravr_cs_init();
	 

	
	/* strat */
	gen.logs[0] = E_USER_STRAT;
	gen.log_level = 5;
	strat_reset_info();

	scheduler_add_periodical_event_priority(do_led_blink, NULL,
						250000L / SCHEDULER_UNIT,
						LED_PRIO);

	sei();
  
	print_banner();

	actuators_default_position();
	lift_init();
	
	
	while (1) {

		
		cmdline_interact();

		
		if (sensor_get(S_START_COLOR)) {
			mainboard.our_color = ROBOT_COLOR_DEFAULT;
			NOTICE(E_USER_STRAT, "our color : " ROBOT_COLOR_DEFAULT_NAME);
		} else {
			mainboard.our_color = ROBOT_COLOR_ALTERNATE;
			NOTICE(E_USER_STRAT, "our color : " ROBOT_COLOR_ALTERNATE_NAME);
		}

		mainboard.flags |= DO_CS;

		printf_P(PSTR("auto position"));
		strat_auto_position();

		printf_P(PSTR("start strat"));

		strat_start();
		
			/*
			struct AX12 *ax12 = &gen.ax12; 
			AX12_set_position(ax12, ID_AX12_GUIDE, WING_RETRACTED);
			wait_ms(2000);
			AX12_set_position(ax12, ID_AX12_GUIDE, WING_RETRACTED);
			wait_ms(2000);
			AX12_set_position(ax12, ID_AX12_GUIDE, WING_RETRACTED);
			wait_ms(2000);*/

	}

	return 0;
}
