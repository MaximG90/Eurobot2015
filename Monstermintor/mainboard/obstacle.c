/*
 *  Copyright iRobotique (2011)
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

#include "main.h"
#include "obstacle.h"
#include "sensor.h"
#include "strat.h"
#include "twi.h"

static bool detection_enabled = false;

/*
 * obstacle_detection_enable - Enable/disable obstacle detection
 */
void obstacle_detection_enable(bool enable)
{
	detection_enabled = enable;
}

/*
 * obstacle_will_collide - Check whether we will collide with an obstacle
 *
 * The only obstacle we handle is the opponent, detected using reflective
 * sensors oriented towards the front of the robot.
 */
bool obstacle_will_collide(void)
{
  int dist_adv;
  if (!detection_enabled)
      return false;

    twi_init();        // join i2c bus (address optional for master)
    requestFrom(4, 1);    // request 1 bytes from slave device #4
    dist_adv = read(); // receive a byte 
     if (dist_adv >9) // On recoit bien la dist_adv et non l'angle
    {
       
      if(dist_adv>100 && dist_adv!=255) // Seuil [10-254]
      {
	return true;
      }
    }
    return false;
  
}


int8_t detect_angle (void)
{
  int cadran;
  int8_t move=0;
  
    twi_init();        // join i2c bus (address optional for master)
    requestFrom(4, 1);    // request 1 bytes from slave device #4
    cadran = read(); // receive a byte 
    
    printf("c : %d \n \r", cadran);
    if (cadran<9) // On recoit bien le cadran et non la dist
    {
         printf("cadran : %d \n \r", cadran);
	// DERIERE
	if(cadran==0) //45° Derriere a droite
	    {move=-45;}
	if(cadran==1) //90° On tourne a droite ou a gauche?
	    {move=90;}
	if(cadran==2) //135° Derriere a gauche
	    {move=45;}
	//DEVANT
	if(cadran==4) //225° Vers la gauche
	    {move=-45;}
	if(cadran==5) //270° DROIT DEVANT
	    {move=-90;}
	if(cadran==6) //315° Vers la droite
	    {move=45;}
	
    }
    else
	    {move=0;}
 printf("move : %d \n \r", move);
return move;

  
}

