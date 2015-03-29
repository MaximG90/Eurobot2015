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
 *
 *  Revision : $Id: encoders_hctl.c,v 1.1.2.3 2009-04-07 20:00:46 zer0 Exp $
 *
 *  Olivier MATZ <zer0@droids-corp.org>
 */

/* This modules handles encoders: values are read through a SPI
 * interface. Basically, frames are formatted with 4 words of 16 bits,
 * describing the values of the 4 encoders. */

#ifndef HOST_VERSION

#include <string.h>

#include <aversive.h>
#include <aversive/parts.h>
#include <encoders_hctl.h>
#include <config/encoders_hctl_config.h>


#include "Arduino.h"


static int32_t  g_encoders_hctl_values[encoders_hctl_NUMBER];
/* static  */int32_t g_encoders_hctl_previous[encoders_hctl_NUMBER];


/* Initialisation of encoders, variables */
void encoders_hctl_init(void)
{
  init();   // initialisation timer arduino si on utilise la fonction delay();
  // Initialisation
    
   DDRC = B00000000;   // sets Arduino Mega (ATMEL ATMEGA) Digital pins 37-30 as inputs from HCTL-2032 - D0 to D7
   DDRA = B00000000;   // sets Arduino Mega (ATMEL ATMEGA) Digital pins 22-29 as inputs from HCTL-2022 - D0 to D7
   
    
   pinMode(XY,   OUTPUT);
   pinMode(OE,   OUTPUT);
   pinMode(EN1,  OUTPUT);
   pinMode(EN2,  OUTPUT);
   pinMode(SEL1, OUTPUT);
   pinMode(SEL2, OUTPUT);
   pinMode(RSTX, OUTPUT);
   pinMode(RSTY, OUTPUT);
   
   pinMode(OE2,   OUTPUT);
   pinMode(SEL12,  OUTPUT);
   pinMode(SEL22, OUTPUT);
   pinMode(RST2,  OUTPUT);
   
   // Communicates with a HCTL-2032 IC to set the count mode
   // see Avago/Agilent/HP HCTL-2032 PDF for details
   // Uniquement le mode 4x car les autres modes offrent une précision moindre (useless)
   digitalWrite(EN1, HIGH);
   digitalWrite(EN2, LOW);
   
   // A vérifier (voir datasheet) : 
   digitalWrite(XY, LOW);
   digitalWrite(OE, HIGH);
   digitalWrite(OE2, HIGH);// 2
   digitalWrite(SEL1, LOW);
   digitalWrite(SEL2, HIGH);
   digitalWrite(SEL12, LOW);// 2
   digitalWrite(SEL22, HIGH);// 2

   
   // Reset encoder 1-1
   digitalWrite(RSTX, LOW);
   delayMicroseconds(1);
   digitalWrite(RSTX, HIGH);
   delayMicroseconds(1);
   // Reset encodeur 1-2
   digitalWrite(RSTY, LOW);
   delayMicroseconds(1);
   digitalWrite(RSTY, HIGH);
   delayMicroseconds(1);
   // Reset encodeur 2-1
   digitalWrite(RST2, LOW);
   delayMicroseconds(1);
   digitalWrite(RST2, HIGH);
   delayMicroseconds(1);
   
   
   // Fin Initialisation
   
   
	memset(g_encoders_hctl_previous, 0, sizeof(g_encoders_hctl_previous));
	memset(g_encoders_hctl_values, 0, sizeof(g_encoders_hctl_values));

}




/* Extract encoder value */
int32_t encoders_hctl_get_value(void *encoder)
{

	uint8_t flags;
	int32_t busByte;
	int32_t count;
if ((int)encoder==2)
{
	digitalWrite(OE2,   LOW); // Début lécture 
	digitalWrite(SEL12, LOW);
	digitalWrite(SEL22, HIGH); // Pour lecture octet poid faible

	delayMicroseconds(1);
	busByte = PINA;
	count   = busByte;
	count <<= 8;

	digitalWrite(SEL12, HIGH);
	digitalWrite(SEL22, HIGH);
	delayMicroseconds(1);
	busByte = PINA;
	count  += busByte;
	count <<= 8;

	digitalWrite(SEL12, LOW);
	digitalWrite(SEL22, LOW);
	delayMicroseconds(1);
	busByte = PINA;
	count  += busByte;
	count <<= 8;

	digitalWrite(SEL12, HIGH);
	digitalWrite(SEL22, LOW);
	delayMicroseconds(1);
	busByte = PINA;
	count  += busByte;
	
	digitalWrite(OE2,  HIGH); // Fin de lécture 
} // fin lift

if ((int)encoder==0 | (int)encoder==1)
{
      if (encoder==0)
	digitalWrite(XY,   LOW); // Selection de l'encodeur 1 ou 2 (ici : ?)
      if (encoder==1)
	digitalWrite(XY,   HIGH); // Selection de l'encodeur 1 ou 2 (ici : ?)

	digitalWrite(OE,   LOW); // Début lécture 
	digitalWrite(SEL1, LOW);
	digitalWrite(SEL2, HIGH); // Pour lecture octet poid faible

	delayMicroseconds(1);
	busByte = PINC;
	count   = busByte;
	count <<= 8;

	digitalWrite(SEL1, HIGH);
	digitalWrite(SEL2, HIGH);
	delayMicroseconds(1);
	busByte = PINC;
	count  += busByte;
	count <<= 8;

	digitalWrite(SEL1, LOW);
	digitalWrite(SEL2, LOW);
	delayMicroseconds(1);
	busByte = PINC;
	count  += busByte;
	count <<= 8;

	digitalWrite(SEL1, HIGH);
	digitalWrite(SEL2, LOW);
	delayMicroseconds(1);
	busByte = PINC;
	count  += busByte;
	
	digitalWrite(OE,  HIGH); // Fin de lécture 
       
}
   IRQ_LOCK(flags);
   g_encoders_hctl_values[(int)encoder] = count;
   IRQ_UNLOCK(flags);

    
   return g_encoders_hctl_values[(int)encoder];
}

/* Set an encoder value */
void encoders_hctl_set_value(void *encoder, int32_t val)
{
	uint8_t flags;

	IRQ_LOCK(flags);
	g_encoders_hctl_values[(int)encoder] = val;
	IRQ_UNLOCK(flags);
}

#endif
