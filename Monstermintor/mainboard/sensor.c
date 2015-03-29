/*
 *  Copyright Droids Corporation (2009)
 *  Olivier MATZ <zer0@droids-corp.org>
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

#include <avr/sleep.h>

#include <aversive.h>
#include <aversive/error.h>
#include <aversive/wait.h>

#include <adc.h>
#include <scheduler.h>

#include "cmdline.h"
#include "leds.h"
#include "main.h"
#include "sensor.h"
#include "tones.h"

/* -----------------------------------------------------------------------------
 * ADC
 */

struct adc_infos {
	uint16_t config;
	int16_t value;
	int16_t value_mv;
	int16_t prev_val;
	int16_t coef_mul;
	int16_t(*filter) (struct adc_infos *, int16_t);
};

/*
 * RII filters for ADC inputs.
 *
 * - rii_light() reaches 90% of the value in 4 samples
 * - rii_medium() reaches 90% of the value in 8 samples
 * - rii_strong() reaches 90% of the value in 16 samples
 */

int16_t rii_light(struct adc_infos *adc, int16_t val)
{
	adc->prev_val = val + (int32_t) adc->prev_val / 2;
	return adc->prev_val / 2;
}

int16_t rii_medium(struct adc_infos * adc, int16_t val)
{
	adc->prev_val = val + ((int32_t) adc->prev_val * 3) / 4;
	return adc->prev_val / 4;
}

int16_t rii_strong(struct adc_infos * adc, int16_t val)
{
	adc->prev_val = val + ((int32_t) adc->prev_val * 7) / 8;
	return adc->prev_val / 8;
}

/*
 * The input resistive divider can be compensated using a multiplicative
 * coefficient. Use the DIV_R1_R2 macro to compute the coefficient for given
 * values of R1 and R2.
*/
#define DIV_R1_R2(r1,r2)	(uint16_t)((float)(((r1 + r2) / (float)r1) * 5000. / 1023.))
#define DIV_2K2_10K		DIV_R1_R2(2200, 10000)
#define DIV_1K76_10K		DIV_R1_R2(1760, 10000)
#define DIV_4K7_10K		DIV_R1_R2(4700, 10000)
#define NO_DIV			5000		/* No voltage divisor */

#define ADC_CONF(x)		(ADC_REF_AVCC | ADC_MODE_INT | MUX_ADC##x)

#define DECLARE_ADC(name, index, _filter, div) \
	[name] = { .config = ADC_CONF(index), \
		   .filter = rii_ ## _filter, \
		   .coef_mul = div }

static struct adc_infos adc_infos[ADC_MAX] = {
	DECLARE_ADC(ADC_BATTERY1, 1, strong, DIV_1K76_10K),	/* 12V */
	DECLARE_ADC(ADC_BATTERY2, 9, strong, DIV_1K76_10K),	/* 24V */
	DECLARE_ADC(ADC_BATTERY3, 8, strong, DIV_4K7_10K),	/*  8V */

	DECLARE_ADC(ADC_CURRENT_MOT1, 4, strong, NO_DIV),
	DECLARE_ADC(ADC_CURRENT_MOT2, 5, strong, NO_DIV),
	DECLARE_ADC(ADC_CURRENT_MOT3, 6, strong, NO_DIV),
	DECLARE_ADC(ADC_CURRENT_MOT4, 7, strong, NO_DIV),

	DECLARE_ADC(ADC_CSENSE1, 10, medium, NO_DIV),
	DECLARE_ADC(ADC_CSENSE2, 11, medium, NO_DIV),
	DECLARE_ADC(ADC_CSENSE3, 12, medium, NO_DIV),
	DECLARE_ADC(ADC_CSENSE4, 13, medium, NO_DIV),
};

static uint8_t adc_cycles = 0;

static void adc_event(int16_t result)
{
	static uint8_t i = 0;

	/* Filter ADC result if needed. */
	if (adc_infos[i].filter)
		adc_infos[i].value = adc_infos[i].filter(&adc_infos[i], result);
	else
		adc_infos[i].value = result;

	/* Update mv value. */
	if (adc_infos[i].coef_mul != NO_DIV)
		adc_infos[i].value_mv =
		    adc_infos[i].value * adc_infos[i].coef_mul;
	else
		adc_infos[i].value_mv =
		    adc_infos[i].value * 5000. / 1023.;

	i++;
	if (i >= ADC_MAX) {
		/* Conversions will be restarted by do_adc(). */
		if (adc_cycles < 255)
			adc_cycles++;

		i = 0;
		return;
	}

	adc_launch(adc_infos[i].config);
}

static void do_adc(void *dummy)
{
	/* Launch the first ADC conversion. Subsequent conversions will be
	 * started from the ADC event handlers.
	 */
	adc_launch(adc_infos[0].config);
}

uint16_t sensor_get_adc(uint8_t i)
{
	uint16_t value;
	uint8_t flags;

	IRQ_LOCK(flags);
	value = adc_infos[i].value;
	IRQ_UNLOCK(flags);

	return value;
}

uint16_t sensor_get_adc_mv(uint8_t i)
{
	uint16_t value;
	uint8_t flags;

	IRQ_LOCK(flags);
	value = adc_infos[i].value_mv;
	IRQ_UNLOCK(flags);

	return value;
}

/* -----------------------------------------------------------------------------
 * Battery management
 */

/*
 * Warning level: 3.0V/cell
 * Emergency level: 2.5V/cell
 *
 * The power supply board should handle emergency conditions before we do.
 */

/* First battery, 13.2V - 4 cells LiFePo4 */
#define WARNING_BAT1_LOW		12000
#define EMERGENCY_BAT1_LOW		10000

/* Second battery, same as second battery */
#define BATTERY_2_ENABLED
#undef BATTERY_2_SERIE
#define WARNING_BAT2_LOW		12000
#define EMERGENCY_BAT2_LOW		10000

static uint8_t low_bat_warning = 0;
static uint8_t low_bat_emergency = 0;

static void do_battery_manager(void *dummy)
{
	uint16_t bat1;
#ifdef BATTERY_2_ENABLED
	uint16_t bat2;
#endif /* BATTERY_2_ENABLED */

	/* Wait for the ADC results to be stable. */
	if (adc_cycles < 255)
		return;

	bat1 = sensor_get_adc_mv(ADC_BATTERY1);

	if (bat1 <= WARNING_BAT1_LOW)
		low_bat_warning++;
	if (bat1 <= EMERGENCY_BAT1_LOW)
		low_bat_emergency++;

#ifdef BATTERY_2_ENABLED
	bat2 = sensor_get_adc_mv(ADC_BATTERY2);
#ifdef BATTERY_2_SERIE
	bat2 = bat2 - bat1;
#endif

	if (bat2 <= WARNING_BAT2_LOW)
		low_bat_warning++;
	if (bat2 <= EMERGENCY_BAT2_LOW)
		low_bat_emergency++;
#endif /* BATTERY_2_ENABLED */

	/* Turn LED8 on and play a warning tone when the battery reaches the
	 * warning level.
	 */
	low_bat_warning = 0;
	low_bat_emergency = 0;
	
	if (low_bat_warning & 0x10) {
		LED8_ON();
	} else {
		LED8_OFF();
	}

	/* Turn LED2 on, stop all motors and sleep forever when the battery
	 * reaches the emergency level.
	 */
	if (low_bat_emergency) {
		LED2_ON();
		printf_P(PSTR("---> Low batteries (%u), emergency stop <---\r\n"),
			 bat1);

		/*
		mainboard.flags = 0;
		pwm_ng_set(&gen.pwm1_4A, 0);
		pwm_ng_set(&gen.pwm2_4B, 0);
		pwm_ng_set(&gen.pwm3_1A, 0);
		pwm_ng_set(&gen.pwm4_1B, 0);

		*/
		wait_ms(100);

		//cli();
		
		//while (1)
		//	sleep_mode();
	}
}

/* -----------------------------------------------------------------------------
 * Boolean sensors
 *
 * sensor mapping (surment pas mis à jour, SE MEFIER) :
 * 0-3:  PORTK[2:5] (CAP[1:4], ADC[10:13])  (Exemple : capteur lift : Cap1=>PK2
 * 4-5:  PORTL[4:5] (CAP[5:6])
 * 6-7:  PORTE[3:4] (CAP[7:8])
 * 8-15: reserved for remote sensors
 */

struct sensor_filter {
	uint8_t filter;
	uint8_t prev;
	uint8_t thres_off;
	uint8_t thres_on;
	uint8_t cpt;
	uint8_t invert;
};

#define DECLARE_SENSOR(name) \
	[name] = { 1, 0, 0, 1, 0, 0 }
#define DECLARE_SENSOR_INVERT(name) \
	[name] = { 1, 0, 0, 1, 0, 1 }
#define DECLARE_SENSOR_FILTER(name, filter, off, on) \
	[name] = { filter, 0, off, on, 0, 0 }
#define DECLARE_SENSOR_FILTER_INVERT(name, filter, off, on) \
	[name] = { filter, 0, off, on, 0, 1 }

/*
 * Declare boolean sensors with the DECLARE_SENSOR_* macros.
 */
static struct sensor_filter sensor_filter[SENSOR_MAX] = {
	DECLARE_SENSOR_FILTER_INVERT(S_START_SWITCH, 10, 3, 7),
	DECLARE_SENSOR_INVERT(S_START_COLOR),
	DECLARE_SENSOR_INVERT(S_AUTO_POS),
	DECLARE_SENSOR(S_LIFT),
	DECLARE_SENSOR(S_LIFT_2),
};

/*
 * Filtered sensor values.
 */
static uint16_t sensor_filtered = 0;

uint16_t sensor_get_all(void)
{
	uint16_t sensors;
	uint8_t flags;

	IRQ_LOCK(flags);
	sensors = sensor_filtered;
	IRQ_UNLOCK(flags);

	return sensors;
}

uint8_t sensor_get(uint8_t i)
{
	uint16_t sensors = sensor_get_all();

	return !!(sensors & (1 << i));
}

static uint16_t sensor_read(void)
{
  /* Arduino Max : 
  * CAP: 1	2	6	9	10
  * 	PK2	PK3	PL1	PL4	PL3
  */
	uint16_t sensors;

	sensors  = (uint16_t)((PINK & (_BV(2) | _BV(3) | _BV(4) | _BV(5))) >> 2); // 2-3
	sensors |= (uint16_t)((PINL & (_BV(0) | _BV(1))) << 4); // 1
	sensors |= (uint16_t)(((PINE & (_BV(3) | _BV(4))) >> 3) << 6); // Useless
	sensors |= (uint16_t)(((PINL & (_BV(4) | _BV(3))) >> 4) << 8); // 3-4

	return sensors;
}

static void do_boolean_sensors(void *dummy)
{
	uint16_t sensors = sensor_read();
	uint16_t filtered = 0;
	uint8_t flags;
	uint8_t i;

	for (i = 0; i < SENSOR_MAX; i++) {
		if ((1 << i) & sensors) {
			if (sensor_filter[i].cpt < sensor_filter[i].filter)
				sensor_filter[i].cpt++;
			if (sensor_filter[i].cpt >= sensor_filter[i].thres_on)
				sensor_filter[i].prev = 1;
		} else {
			if (sensor_filter[i].cpt > 0)
				sensor_filter[i].cpt--;
			if (sensor_filter[i].cpt <= sensor_filter[i].thres_off)
				sensor_filter[i].prev = 0;
		}

		/* Update filtered value and optionally invert it. */
		if (sensor_filter[i].prev)
			filtered |= 1 << i;
		if (sensor_filter[i].invert)
			filtered ^= 1 << i;
	}

	IRQ_LOCK(flags);
	sensor_filtered = filtered;
	IRQ_UNLOCK(flags);
}

/* -----------------------------------------------------------------------------
 * Infra-red
 *
 * The infra-red emitters are connected to CAP7 and CAP8, driven through FETs to
 * output up to 400mA. The FETs are controlled by the CPLD which generates a
 * 38 kHz clock controlled by PJ5. The receiver is connected to CAP5
 */

static void ir_enable_transmitters(void)
{
	//PORTJ |= _BV(5);
}

static void ir_disable_transmitters(void)
{
	//PORTJ &= ~_BV(5);
}

static void do_ir(void)
{
	static uint8_t toggle = 0;
	static uint8_t ir_ok = 0;

	if (!mainboard.ir) {
		ir_disable_transmitters();
		return;
	}

	if (toggle & 0x1) {
		ir_enable_transmitters();
	} else {
		if ((PINL & _BV(0)) == 0)
			ir_ok++;
		ir_disable_transmitters();
	}
	toggle++;

	if (toggle > 10) {
		if (ir_ok >= 4) {
			LED5_OFF();
		} else {
			LED5_ON();
		}

		toggle = 0;
		ir_ok = 0;
	}
}

/* -----------------------------------------------------------------------------
 * Initialization and main task
 */

static void do_sensors(void *dummy)
{
	//do_adc(NULL);
	//do_battery_manager(NULL);
	do_boolean_sensors(NULL);

	/* Stop the command line and start the robot when the auto-position
	 * sensor is pressed.
	 */
	if (sensor_get(S_AUTO_POS))
		cmdline_interrupt();

	//do_ir();
}

void sensor_init(void)
{
	/* Activate internal pull-ups on CAP 1-10 */
	PORTK |= _BV(2) | _BV(3) ; // 2 capteurs de proximite
	PORTL |= _BV(1) | _BV(3) | _BV(4); // Depart - Calibration - Choix camp
	

	//adc_init();
	//adc_register_event(adc_event);

	scheduler_add_periodical_event_priority(do_sensors, NULL,
						10000L / SCHEDULER_UNIT,
						ADC_PRIO);
}
