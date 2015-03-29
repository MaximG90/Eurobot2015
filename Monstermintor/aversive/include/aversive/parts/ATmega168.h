/*  
 *  Copyright Droids Corporation, Microb Technology, Eirbot (2009)
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
 *  Revision : $Id $
 *
 */

/* WARNING : this file is automatically generated by scripts.
 * You should not edit it. If you find something wrong in it,
 * write to zer0@droids-corp.org */


/* prescalers timer 0 */
#define TIMER0_PRESCALER_DIV_0          0
#define TIMER0_PRESCALER_DIV_1          1
#define TIMER0_PRESCALER_DIV_8          2
#define TIMER0_PRESCALER_DIV_64         3
#define TIMER0_PRESCALER_DIV_256        4
#define TIMER0_PRESCALER_DIV_1024       5
#define TIMER0_PRESCALER_DIV_FALL       6
#define TIMER0_PRESCALER_DIV_RISE       7

#define TIMER0_PRESCALER_REG_0          0
#define TIMER0_PRESCALER_REG_1          1
#define TIMER0_PRESCALER_REG_2          8
#define TIMER0_PRESCALER_REG_3          64
#define TIMER0_PRESCALER_REG_4          256
#define TIMER0_PRESCALER_REG_5          1024
#define TIMER0_PRESCALER_REG_6          -1
#define TIMER0_PRESCALER_REG_7          -2

/* prescalers timer 1 */
#define TIMER1_PRESCALER_DIV_0          0
#define TIMER1_PRESCALER_DIV_1          1
#define TIMER1_PRESCALER_DIV_8          2
#define TIMER1_PRESCALER_DIV_64         3
#define TIMER1_PRESCALER_DIV_256        4
#define TIMER1_PRESCALER_DIV_1024       5
#define TIMER1_PRESCALER_DIV_FALL       6
#define TIMER1_PRESCALER_DIV_RISE       7

#define TIMER1_PRESCALER_REG_0          0
#define TIMER1_PRESCALER_REG_1          1
#define TIMER1_PRESCALER_REG_2          8
#define TIMER1_PRESCALER_REG_3          64
#define TIMER1_PRESCALER_REG_4          256
#define TIMER1_PRESCALER_REG_5          1024
#define TIMER1_PRESCALER_REG_6          -1
#define TIMER1_PRESCALER_REG_7          -2

/* prescalers timer 2 */
#define TIMER2_PRESCALER_DIV_0          0
#define TIMER2_PRESCALER_DIV_1          1
#define TIMER2_PRESCALER_DIV_8          2
#define TIMER2_PRESCALER_DIV_32         3
#define TIMER2_PRESCALER_DIV_64         4
#define TIMER2_PRESCALER_DIV_128        5
#define TIMER2_PRESCALER_DIV_256        6
#define TIMER2_PRESCALER_DIV_1024       7

#define TIMER2_PRESCALER_REG_0          0
#define TIMER2_PRESCALER_REG_1          1
#define TIMER2_PRESCALER_REG_2          8
#define TIMER2_PRESCALER_REG_3          32
#define TIMER2_PRESCALER_REG_4          64
#define TIMER2_PRESCALER_REG_5          128
#define TIMER2_PRESCALER_REG_6          256
#define TIMER2_PRESCALER_REG_7          1024


/* available timers */
#define TIMER0_AVAILABLE
#define TIMER0A_AVAILABLE
#define TIMER0B_AVAILABLE
#define TIMER1_AVAILABLE
#define TIMER1A_AVAILABLE
#define TIMER1B_AVAILABLE
#define TIMER2_AVAILABLE
#define TIMER2A_AVAILABLE
#define TIMER2B_AVAILABLE

/* overflow interrupt number */
#define SIG_OVERFLOW0_NUM 0
#define SIG_OVERFLOW1_NUM 1
#define SIG_OVERFLOW2_NUM 2
#define SIG_OVERFLOW_TOTAL_NUM 3

/* output compare interrupt number */
#define SIG_OUTPUT_COMPARE0A_NUM 0
#define SIG_OUTPUT_COMPARE0B_NUM 1
#define SIG_OUTPUT_COMPARE1A_NUM 2
#define SIG_OUTPUT_COMPARE1B_NUM 3
#define SIG_OUTPUT_COMPARE2A_NUM 4
#define SIG_OUTPUT_COMPARE2B_NUM 5
#define SIG_OUTPUT_COMPARE_TOTAL_NUM 6

/* Pwm nums */
#define PWM0A_NUM 0
#define PWM0B_NUM 1
#define PWM1A_NUM 2
#define PWM1B_NUM 3
#define PWM2A_NUM 4
#define PWM2B_NUM 5
#define PWM_TOTAL_NUM 6

/* input capture interrupt number */
#define SIG_INPUT_CAPTURE1_NUM 0
#define SIG_INPUT_CAPTURE_TOTAL_NUM 1


/* ADMUX */
#define MUX0_REG             ADMUX
#define MUX1_REG             ADMUX
#define MUX2_REG             ADMUX
#define MUX3_REG             ADMUX
#define ADLAR_REG            ADMUX
#define REFS0_REG            ADMUX
#define REFS1_REG            ADMUX

/* WDTCSR */
#define WDP0_REG             WDTCSR
#define WDP1_REG             WDTCSR
#define WDP2_REG             WDTCSR
#define WDE_REG              WDTCSR
#define WDCE_REG             WDTCSR
#define WDP3_REG             WDTCSR
#define WDIE_REG             WDTCSR
#define WDIF_REG             WDTCSR

/* EEDR */
#define EEDR0_REG            EEDR
#define EEDR1_REG            EEDR
#define EEDR2_REG            EEDR
#define EEDR3_REG            EEDR
#define EEDR4_REG            EEDR
#define EEDR5_REG            EEDR
#define EEDR6_REG            EEDR
#define EEDR7_REG            EEDR

/* ACSR */
#define ACIS0_REG            ACSR
#define ACIS1_REG            ACSR
#define ACIC_REG             ACSR
#define ACIE_REG             ACSR
#define ACI_REG              ACSR
#define ACO_REG              ACSR
#define ACBG_REG             ACSR
#define ACD_REG              ACSR

/* OCR2B */
#define OCR2B_0_REG          OCR2B
#define OCR2B_1_REG          OCR2B
#define OCR2B_2_REG          OCR2B
#define OCR2B_3_REG          OCR2B
#define OCR2B_4_REG          OCR2B
#define OCR2B_5_REG          OCR2B
#define OCR2B_6_REG          OCR2B
#define OCR2B_7_REG          OCR2B

/* OCR2A */
#define OCR2A_0_REG          OCR2A
#define OCR2A_1_REG          OCR2A
#define OCR2A_2_REG          OCR2A
#define OCR2A_3_REG          OCR2A
#define OCR2A_4_REG          OCR2A
#define OCR2A_5_REG          OCR2A
#define OCR2A_6_REG          OCR2A
#define OCR2A_7_REG          OCR2A

/* SPDR */
#define SPDR0_REG            SPDR
#define SPDR1_REG            SPDR
#define SPDR2_REG            SPDR
#define SPDR3_REG            SPDR
#define SPDR4_REG            SPDR
#define SPDR5_REG            SPDR
#define SPDR6_REG            SPDR
#define SPDR7_REG            SPDR

/* SPSR */
#define SPI2X_REG            SPSR
#define WCOL_REG             SPSR
#define SPIF_REG             SPSR

/* SPH */
#define SP8_REG              SPH
#define SP9_REG              SPH
#define SP10_REG             SPH

/* ICR1L */
#define ICR1L0_REG           ICR1L
#define ICR1L1_REG           ICR1L
#define ICR1L2_REG           ICR1L
#define ICR1L3_REG           ICR1L
#define ICR1L4_REG           ICR1L
#define ICR1L5_REG           ICR1L
#define ICR1L6_REG           ICR1L
#define ICR1L7_REG           ICR1L

/* PRR */
#define PRADC_REG            PRR
#define PRUSART0_REG         PRR
#define PRSPI_REG            PRR
#define PRTIM1_REG           PRR
#define PRTIM0_REG           PRR
#define PRTIM2_REG           PRR
#define PRTWI_REG            PRR

/* TWSR */
#define TWPS0_REG            TWSR
#define TWPS1_REG            TWSR
#define TWS3_REG             TWSR
#define TWS4_REG             TWSR
#define TWS5_REG             TWSR
#define TWS6_REG             TWSR
#define TWS7_REG             TWSR

/* UCSR0A */
#define MPCM0_REG            UCSR0A
#define U2X0_REG             UCSR0A
#define UPE0_REG             UCSR0A
#define DOR0_REG             UCSR0A
#define FE0_REG              UCSR0A
#define UDRE0_REG            UCSR0A
#define TXC0_REG             UCSR0A
#define RXC0_REG             UCSR0A

/* PORTD */
#define PORTD0_REG           PORTD
#define PORTD1_REG           PORTD
#define PORTD2_REG           PORTD
#define PORTD3_REG           PORTD
#define PORTD4_REG           PORTD
#define PORTD5_REG           PORTD
#define PORTD6_REG           PORTD
#define PORTD7_REG           PORTD

/* UCSR0B */
#define TXB80_REG            UCSR0B
#define RXB80_REG            UCSR0B
#define UCSZ02_REG           UCSR0B
#define TXEN0_REG            UCSR0B
#define RXEN0_REG            UCSR0B
#define UDRIE0_REG           UCSR0B
#define TXCIE0_REG           UCSR0B
#define RXCIE0_REG           UCSR0B

/* PORTB */
#define PORTB0_REG           PORTB
#define PORTB1_REG           PORTB
#define PORTB2_REG           PORTB
#define PORTB3_REG           PORTB
#define PORTB4_REG           PORTB
#define PORTB5_REG           PORTB
#define PORTB6_REG           PORTB
#define PORTB7_REG           PORTB

/* PORTC */
#define PORTC0_REG           PORTC
#define PORTC1_REG           PORTC
#define PORTC2_REG           PORTC
#define PORTC3_REG           PORTC
#define PORTC4_REG           PORTC
#define PORTC5_REG           PORTC
#define PORTC6_REG           PORTC

/* UDR0 */
#define UDR0_0_REG           UDR0
#define UDR0_1_REG           UDR0
#define UDR0_2_REG           UDR0
#define UDR0_3_REG           UDR0
#define UDR0_4_REG           UDR0
#define UDR0_5_REG           UDR0
#define UDR0_6_REG           UDR0
#define UDR0_7_REG           UDR0

/* EICRA */
#define ISC00_REG            EICRA
#define ISC01_REG            EICRA
#define ISC10_REG            EICRA
#define ISC11_REG            EICRA

/* DIDR0 */
#define ADC0D_REG            DIDR0
#define ADC1D_REG            DIDR0
#define ADC2D_REG            DIDR0
#define ADC3D_REG            DIDR0
#define ADC4D_REG            DIDR0
#define ADC5D_REG            DIDR0

/* DIDR1 */
#define AIN0D_REG            DIDR1
#define AIN1D_REG            DIDR1

/* ASSR */
#define TCR2BUB_REG          ASSR
#define TCR2AUB_REG          ASSR
#define OCR2BUB_REG          ASSR
#define OCR2AUB_REG          ASSR
#define TCN2UB_REG           ASSR
#define AS2_REG              ASSR
#define EXCLK_REG            ASSR

/* CLKPR */
#define CLKPS0_REG           CLKPR
#define CLKPS1_REG           CLKPR
#define CLKPS2_REG           CLKPR
#define CLKPS3_REG           CLKPR
#define CLKPCE_REG           CLKPR

/* SREG */
#define C_REG                SREG
#define Z_REG                SREG
#define N_REG                SREG
#define V_REG                SREG
#define S_REG                SREG
#define H_REG                SREG
#define T_REG                SREG
#define I_REG                SREG

/* DDRB */
#define DDB0_REG             DDRB
#define DDB1_REG             DDRB
#define DDB2_REG             DDRB
#define DDB3_REG             DDRB
#define DDB4_REG             DDRB
#define DDB5_REG             DDRB
#define DDB6_REG             DDRB
#define DDB7_REG             DDRB

/* DDRC */
#define DDC0_REG             DDRC
#define DDC1_REG             DDRC
#define DDC2_REG             DDRC
#define DDC3_REG             DDRC
#define DDC4_REG             DDRC
#define DDC5_REG             DDRC
#define DDC6_REG             DDRC

/* TCCR1A */
#define WGM10_REG            TCCR1A
#define WGM11_REG            TCCR1A
#define COM1B0_REG           TCCR1A
#define COM1B1_REG           TCCR1A
#define COM1A0_REG           TCCR1A
#define COM1A1_REG           TCCR1A

/* TCCR1C */
#define FOC1B_REG            TCCR1C
#define FOC1A_REG            TCCR1C

/* TCCR1B */
#define CS10_REG             TCCR1B
#define CS11_REG             TCCR1B
#define CS12_REG             TCCR1B
#define WGM12_REG            TCCR1B
#define WGM13_REG            TCCR1B
#define ICES1_REG            TCCR1B
#define ICNC1_REG            TCCR1B

/* OSCCAL */
#define CAL0_REG             OSCCAL
#define CAL1_REG             OSCCAL
#define CAL2_REG             OSCCAL
#define CAL3_REG             OSCCAL
#define CAL4_REG             OSCCAL
#define CAL5_REG             OSCCAL
#define CAL6_REG             OSCCAL
#define CAL7_REG             OSCCAL

/* GPIOR1 */
#define GPIOR10_REG          GPIOR1
#define GPIOR11_REG          GPIOR1
#define GPIOR12_REG          GPIOR1
#define GPIOR13_REG          GPIOR1
#define GPIOR14_REG          GPIOR1
#define GPIOR15_REG          GPIOR1
#define GPIOR16_REG          GPIOR1
#define GPIOR17_REG          GPIOR1

/* GPIOR0 */
#define GPIOR00_REG          GPIOR0
#define GPIOR01_REG          GPIOR0
#define GPIOR02_REG          GPIOR0
#define GPIOR03_REG          GPIOR0
#define GPIOR04_REG          GPIOR0
#define GPIOR05_REG          GPIOR0
#define GPIOR06_REG          GPIOR0
#define GPIOR07_REG          GPIOR0

/* GPIOR2 */
#define GPIOR20_REG          GPIOR2
#define GPIOR21_REG          GPIOR2
#define GPIOR22_REG          GPIOR2
#define GPIOR23_REG          GPIOR2
#define GPIOR24_REG          GPIOR2
#define GPIOR25_REG          GPIOR2
#define GPIOR26_REG          GPIOR2
#define GPIOR27_REG          GPIOR2

/* PCICR */
#define PCIE0_REG            PCICR
#define PCIE1_REG            PCICR
#define PCIE2_REG            PCICR

/* TCNT2 */
#define TCNT2_0_REG          TCNT2
#define TCNT2_1_REG          TCNT2
#define TCNT2_2_REG          TCNT2
#define TCNT2_3_REG          TCNT2
#define TCNT2_4_REG          TCNT2
#define TCNT2_5_REG          TCNT2
#define TCNT2_6_REG          TCNT2
#define TCNT2_7_REG          TCNT2

/* TCNT0 */
#define TCNT0_0_REG          TCNT0
#define TCNT0_1_REG          TCNT0
#define TCNT0_2_REG          TCNT0
#define TCNT0_3_REG          TCNT0
#define TCNT0_4_REG          TCNT0
#define TCNT0_5_REG          TCNT0
#define TCNT0_6_REG          TCNT0
#define TCNT0_7_REG          TCNT0

/* TWAR */
#define TWGCE_REG            TWAR
#define TWA0_REG             TWAR
#define TWA1_REG             TWAR
#define TWA2_REG             TWAR
#define TWA3_REG             TWAR
#define TWA4_REG             TWAR
#define TWA5_REG             TWAR
#define TWA6_REG             TWAR

/* TCCR0B */
#define CS00_REG             TCCR0B
#define CS01_REG             TCCR0B
#define CS02_REG             TCCR0B
#define WGM02_REG            TCCR0B
#define FOC0B_REG            TCCR0B
#define FOC0A_REG            TCCR0B

/* TCCR0A */
#define WGM00_REG            TCCR0A
#define WGM01_REG            TCCR0A
#define COM0B0_REG           TCCR0A
#define COM0B1_REG           TCCR0A
#define COM0A0_REG           TCCR0A
#define COM0A1_REG           TCCR0A

/* TIFR2 */
#define TOV2_REG             TIFR2
#define OCF2A_REG            TIFR2
#define OCF2B_REG            TIFR2

/* TIFR0 */
#define TOV0_REG             TIFR0
#define OCF0A_REG            TIFR0
#define OCF0B_REG            TIFR0

/* TIFR1 */
#define TOV1_REG             TIFR1
#define OCF1A_REG            TIFR1
#define OCF1B_REG            TIFR1
#define ICF1_REG             TIFR1

/* GTCCR */
#define PSRSYNC_REG          GTCCR
#define TSM_REG              GTCCR
#define PSRASY_REG           GTCCR

/* TWBR */
#define TWBR0_REG            TWBR
#define TWBR1_REG            TWBR
#define TWBR2_REG            TWBR
#define TWBR3_REG            TWBR
#define TWBR4_REG            TWBR
#define TWBR5_REG            TWBR
#define TWBR6_REG            TWBR
#define TWBR7_REG            TWBR

/* ICR1H */
#define ICR1H0_REG           ICR1H
#define ICR1H1_REG           ICR1H
#define ICR1H2_REG           ICR1H
#define ICR1H3_REG           ICR1H
#define ICR1H4_REG           ICR1H
#define ICR1H5_REG           ICR1H
#define ICR1H6_REG           ICR1H
#define ICR1H7_REG           ICR1H

/* OCR1BL */
#define OCR1BL0_REG          OCR1BL
#define OCR1BL1_REG          OCR1BL
#define OCR1BL2_REG          OCR1BL
#define OCR1BL3_REG          OCR1BL
#define OCR1BL4_REG          OCR1BL
#define OCR1BL5_REG          OCR1BL
#define OCR1BL6_REG          OCR1BL
#define OCR1BL7_REG          OCR1BL

/* PCIFR */
#define PCIF0_REG            PCIFR
#define PCIF1_REG            PCIFR
#define PCIF2_REG            PCIFR

/* SPL */
#define SP0_REG              SPL
#define SP1_REG              SPL
#define SP2_REG              SPL
#define SP3_REG              SPL
#define SP4_REG              SPL
#define SP5_REG              SPL
#define SP6_REG              SPL
#define SP7_REG              SPL

/* OCR1BH */
#define OCR1BH0_REG          OCR1BH
#define OCR1BH1_REG          OCR1BH
#define OCR1BH2_REG          OCR1BH
#define OCR1BH3_REG          OCR1BH
#define OCR1BH4_REG          OCR1BH
#define OCR1BH5_REG          OCR1BH
#define OCR1BH6_REG          OCR1BH
#define OCR1BH7_REG          OCR1BH

/* EECR */
#define EERE_REG             EECR
#define EEPE_REG             EECR
#define EEMPE_REG            EECR
#define EERIE_REG            EECR
#define EEPM0_REG            EECR
#define EEPM1_REG            EECR

/* SMCR */
#define SE_REG               SMCR
#define SM0_REG              SMCR
#define SM1_REG              SMCR
#define SM2_REG              SMCR

/* TWCR */
#define TWIE_REG             TWCR
#define TWEN_REG             TWCR
#define TWWC_REG             TWCR
#define TWSTO_REG            TWCR
#define TWSTA_REG            TWCR
#define TWEA_REG             TWCR
#define TWINT_REG            TWCR

/* TCCR2A */
#define WGM20_REG            TCCR2A
#define WGM21_REG            TCCR2A
#define COM2B0_REG           TCCR2A
#define COM2B1_REG           TCCR2A
#define COM2A0_REG           TCCR2A
#define COM2A1_REG           TCCR2A

/* TCCR2B */
#define CS20_REG             TCCR2B
#define CS21_REG             TCCR2B
#define CS22_REG             TCCR2B
#define WGM22_REG            TCCR2B
#define FOC2B_REG            TCCR2B
#define FOC2A_REG            TCCR2B

/* UBRR0H */
#define UBRR8_REG            UBRR0H
#define UBRR9_REG            UBRR0H
#define UBRR10_REG           UBRR0H
#define UBRR11_REG           UBRR0H

/* UBRR0L */
#define UBRR0_REG            UBRR0L
#define UBRR1_REG            UBRR0L
#define UBRR2_REG            UBRR0L
#define UBRR3_REG            UBRR0L
#define UBRR4_REG            UBRR0L
#define UBRR5_REG            UBRR0L
#define UBRR6_REG            UBRR0L
#define UBRR7_REG            UBRR0L

/* EEARH */
#define EEAR8_REG            EEARH

/* EEARL */
#define EEAR0_REG            EEARL
#define EEAR1_REG            EEARL
#define EEAR2_REG            EEARL
#define EEAR3_REG            EEARL
#define EEAR4_REG            EEARL
#define EEAR5_REG            EEARL
#define EEAR6_REG            EEARL
#define EEAR7_REG            EEARL

/* MCUCR */
#define IVCE_REG             MCUCR
#define IVSEL_REG            MCUCR
#define PUD_REG              MCUCR

/* MCUSR */
#define PORF_REG             MCUSR
#define EXTRF_REG            MCUSR
#define BORF_REG             MCUSR
#define WDRF_REG             MCUSR

/* TWDR */
#define TWD0_REG             TWDR
#define TWD1_REG             TWDR
#define TWD2_REG             TWDR
#define TWD3_REG             TWDR
#define TWD4_REG             TWDR
#define TWD5_REG             TWDR
#define TWD6_REG             TWDR
#define TWD7_REG             TWDR

/* OCR1AH */
#define OCR1AH0_REG          OCR1AH
#define OCR1AH1_REG          OCR1AH
#define OCR1AH2_REG          OCR1AH
#define OCR1AH3_REG          OCR1AH
#define OCR1AH4_REG          OCR1AH
#define OCR1AH5_REG          OCR1AH
#define OCR1AH6_REG          OCR1AH
#define OCR1AH7_REG          OCR1AH

/* ADCSRA */
#define ADPS0_REG            ADCSRA
#define ADPS1_REG            ADCSRA
#define ADPS2_REG            ADCSRA
#define ADIE_REG             ADCSRA
#define ADIF_REG             ADCSRA
#define ADATE_REG            ADCSRA
#define ADSC_REG             ADCSRA
#define ADEN_REG             ADCSRA

/* ADCSRB */
#define ADTS0_REG            ADCSRB
#define ADTS1_REG            ADCSRB
#define ADTS2_REG            ADCSRB
#define ACME_REG             ADCSRB

/* OCR0A */
#define OCROA_0_REG          OCR0A
#define OCROA_1_REG          OCR0A
#define OCROA_2_REG          OCR0A
#define OCROA_3_REG          OCR0A
#define OCROA_4_REG          OCR0A
#define OCROA_5_REG          OCR0A
#define OCROA_6_REG          OCR0A
#define OCROA_7_REG          OCR0A

/* OCR0B */
#define OCR0B_0_REG          OCR0B
#define OCR0B_1_REG          OCR0B
#define OCR0B_2_REG          OCR0B
#define OCR0B_3_REG          OCR0B
#define OCR0B_4_REG          OCR0B
#define OCR0B_5_REG          OCR0B
#define OCR0B_6_REG          OCR0B
#define OCR0B_7_REG          OCR0B

/* TCNT1L */
#define TCNT1L0_REG          TCNT1L
#define TCNT1L1_REG          TCNT1L
#define TCNT1L2_REG          TCNT1L
#define TCNT1L3_REG          TCNT1L
#define TCNT1L4_REG          TCNT1L
#define TCNT1L5_REG          TCNT1L
#define TCNT1L6_REG          TCNT1L
#define TCNT1L7_REG          TCNT1L

/* DDRD */
#define DDD0_REG             DDRD
#define DDD1_REG             DDRD
#define DDD2_REG             DDRD
#define DDD3_REG             DDRD
#define DDD4_REG             DDRD
#define DDD5_REG             DDRD
#define DDD6_REG             DDRD
#define DDD7_REG             DDRD

/* UCSR0C */
#define UCPOL0_REG           UCSR0C
#define UCSZ00_REG           UCSR0C
#define UCSZ01_REG           UCSR0C
#define USBS0_REG            UCSR0C
#define UPM00_REG            UCSR0C
#define UPM01_REG            UCSR0C
#define UMSEL00_REG          UCSR0C
#define UMSEL01_REG          UCSR0C

/* SPMCSR */
#define SELFPRGEN_REG        SPMCSR
#define PGERS_REG            SPMCSR
#define PGWRT_REG            SPMCSR
#define BLBSET_REG           SPMCSR
#define RWWSRE_REG           SPMCSR
#define RWWSB_REG            SPMCSR
#define SPMIE_REG            SPMCSR

/* TCNT1H */
#define TCNT1H0_REG          TCNT1H
#define TCNT1H1_REG          TCNT1H
#define TCNT1H2_REG          TCNT1H
#define TCNT1H3_REG          TCNT1H
#define TCNT1H4_REG          TCNT1H
#define TCNT1H5_REG          TCNT1H
#define TCNT1H6_REG          TCNT1H
#define TCNT1H7_REG          TCNT1H

/* ADCL */
#define ADCL0_REG            ADCL
#define ADCL1_REG            ADCL
#define ADCL2_REG            ADCL
#define ADCL3_REG            ADCL
#define ADCL4_REG            ADCL
#define ADCL5_REG            ADCL
#define ADCL6_REG            ADCL
#define ADCL7_REG            ADCL

/* ADCH */
#define ADCH0_REG            ADCH
#define ADCH1_REG            ADCH
#define ADCH2_REG            ADCH
#define ADCH3_REG            ADCH
#define ADCH4_REG            ADCH
#define ADCH5_REG            ADCH
#define ADCH6_REG            ADCH
#define ADCH7_REG            ADCH

/* TIMSK2 */
#define TOIE2_REG            TIMSK2
#define OCIE2A_REG           TIMSK2
#define OCIE2B_REG           TIMSK2

/* EIMSK */
#define INT0_REG             EIMSK
#define INT1_REG             EIMSK

/* TIMSK0 */
#define TOIE0_REG            TIMSK0
#define OCIE0A_REG           TIMSK0
#define OCIE0B_REG           TIMSK0

/* TIMSK1 */
#define TOIE1_REG            TIMSK1
#define OCIE1A_REG           TIMSK1
#define OCIE1B_REG           TIMSK1
#define ICIE1_REG            TIMSK1

/* PCMSK0 */
#define PCINT0_REG           PCMSK0
#define PCINT1_REG           PCMSK0
#define PCINT2_REG           PCMSK0
#define PCINT3_REG           PCMSK0
#define PCINT4_REG           PCMSK0
#define PCINT5_REG           PCMSK0
#define PCINT6_REG           PCMSK0
#define PCINT7_REG           PCMSK0

/* PCMSK1 */
#define PCINT8_REG           PCMSK1
#define PCINT9_REG           PCMSK1
#define PCINT10_REG          PCMSK1
#define PCINT11_REG          PCMSK1
#define PCINT12_REG          PCMSK1
#define PCINT13_REG          PCMSK1
#define PCINT14_REG          PCMSK1

/* PCMSK2 */
#define PCINT16_REG          PCMSK2
#define PCINT17_REG          PCMSK2
#define PCINT18_REG          PCMSK2
#define PCINT19_REG          PCMSK2
#define PCINT20_REG          PCMSK2
#define PCINT21_REG          PCMSK2
#define PCINT22_REG          PCMSK2
#define PCINT23_REG          PCMSK2

/* PINC */
#define PINC0_REG            PINC
#define PINC1_REG            PINC
#define PINC2_REG            PINC
#define PINC3_REG            PINC
#define PINC4_REG            PINC
#define PINC5_REG            PINC
#define PINC6_REG            PINC

/* PINB */
#define PINB0_REG            PINB
#define PINB1_REG            PINB
#define PINB2_REG            PINB
#define PINB3_REG            PINB
#define PINB4_REG            PINB
#define PINB5_REG            PINB
#define PINB6_REG            PINB
#define PINB7_REG            PINB

/* EIFR */
#define INTF0_REG            EIFR
#define INTF1_REG            EIFR

/* PIND */
#define PIND0_REG            PIND
#define PIND1_REG            PIND
#define PIND2_REG            PIND
#define PIND3_REG            PIND
#define PIND4_REG            PIND
#define PIND5_REG            PIND
#define PIND6_REG            PIND
#define PIND7_REG            PIND

/* TWAMR */
#define TWAM0_REG            TWAMR
#define TWAM1_REG            TWAMR
#define TWAM2_REG            TWAMR
#define TWAM3_REG            TWAMR
#define TWAM4_REG            TWAMR
#define TWAM5_REG            TWAMR
#define TWAM6_REG            TWAMR

/* OCR1AL */
#define OCR1AL0_REG          OCR1AL
#define OCR1AL1_REG          OCR1AL
#define OCR1AL2_REG          OCR1AL
#define OCR1AL3_REG          OCR1AL
#define OCR1AL4_REG          OCR1AL
#define OCR1AL5_REG          OCR1AL
#define OCR1AL6_REG          OCR1AL
#define OCR1AL7_REG          OCR1AL

/* SPCR */
#define SPR0_REG             SPCR
#define SPR1_REG             SPCR
#define CPHA_REG             SPCR
#define CPOL_REG             SPCR
#define MSTR_REG             SPCR
#define DORD_REG             SPCR
#define SPE_REG              SPCR
#define SPIE_REG             SPCR

/* pins mapping */
#define ICP1_PORT PORTB
#define ICP1_BIT 0
#define CLKO_PORT PORTB
#define CLKO_BIT 0
#define PCINT0_PORT PORTB
#define PCINT0_BIT 0

#define OC1A_PORT PORTB
#define OC1A_BIT 1
#define PCINT1_PORT PORTB
#define PCINT1_BIT 1

#define SS_PORT PORTB
#define SS_BIT 2
#define OC1B_PORT PORTB
#define OC1B_BIT 2
#define PCINT2_PORT PORTB
#define PCINT2_BIT 2

#define MOSI_PORT PORTB
#define MOSI_BIT 3
#define OC2A_PORT PORTB
#define OC2A_BIT 3
#define PCINT3_PORT PORTB
#define PCINT3_BIT 3

#define MISO_PORT PORTB
#define MISO_BIT 4
#define PCINT4_PORT PORTB
#define PCINT4_BIT 4

#define SCK_PORT PORTB
#define SCK_BIT 5
#define PCINT5_PORT PORTB
#define PCINT5_BIT 5

#define XTAL1_PORT PORTB
#define XTAL1_BIT 6
#define TOSC1_PORT PORTB
#define TOSC1_BIT 6
#define PCINT6_PORT PORTB
#define PCINT6_BIT 6

#define XTAL2_PORT PORTB
#define XTAL2_BIT 7
#define TOSC2_PORT PORTB
#define TOSC2_BIT 7
#define PCINT7_PORT PORTB
#define PCINT7_BIT 7

#define ADC0_PORT PORTC
#define ADC0_BIT 0
#define PCINT8_PORT PORTC
#define PCINT8_BIT 0

#define ADC1_PORT PORTC
#define ADC1_BIT 1
#define PCINT9_PORT PORTC
#define PCINT9_BIT 1

#define ADC2_PORT PORTC
#define ADC2_BIT 2
#define PCINT10_PORT PORTC
#define PCINT10_BIT 2

#define ADC3_PORT PORTC
#define ADC3_BIT 3
#define PCINT11_PORT PORTC
#define PCINT11_BIT 3

#define ADC4_PORT PORTC
#define ADC4_BIT 4
#define SDA_PORT PORTC
#define SDA_BIT 4
#define PCINT12_PORT PORTC
#define PCINT12_BIT 4

#define ADC5_PORT PORTC
#define ADC5_BIT 5
#define SCL_PORT PORTC
#define SCL_BIT 5
#define PCINT13_PORT PORTC
#define PCINT13_BIT 5

#define RESET_PORT PORTC
#define RESET_BIT 6
#define PCINT14_PORT PORTC
#define PCINT14_BIT 6

#define RXD_PORT PORTD
#define RXD_BIT 0
#define PCINT16_PORT PORTD
#define PCINT16_BIT 0

#define TXD_PORT PORTD
#define TXD_BIT 1
#define PCINT17_PORT PORTD
#define PCINT17_BIT 1

#define INT0_PORT PORTD
#define INT0_BIT 2
#define PCINT18_PORT PORTD
#define PCINT18_BIT 2

#define PCINT19_PORT PORTD
#define PCINT19_BIT 3
#define OC2B_PORT PORTD
#define OC2B_BIT 3
#define INT1_PORT PORTD
#define INT1_BIT 3

#define XCK_PORT PORTD
#define XCK_BIT 4
#define T0_PORT PORTD
#define T0_BIT 4
#define PCINT20_PORT PORTD
#define PCINT20_BIT 4

#define T1_PORT PORTD
#define T1_BIT 5
#define OC0B_PORT PORTD
#define OC0B_BIT 5
#define PCINT21_PORT PORTD
#define PCINT21_BIT 5

#define AIN0_PORT PORTD
#define AIN0_BIT 6
#define OC0A_PORT PORTD
#define OC0A_BIT 6
#define PCINT22_PORT PORTD
#define PCINT22_BIT 6

#define AIN1_PORT PORTD
#define AIN1_BIT 7
#define PCINT23_PORT PORTD
#define PCINT23_BIT 7


