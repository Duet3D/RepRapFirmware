/****************************************************************************************************

RepRapFirmware - Move

This is all the code to deal with movement and kinematics.

-----------------------------------------------------------------------------------------------------

Version 0.1

18 November 2012

Adrian Bowyer
RepRap Professional Ltd
http://reprappro.com

Licence: GPL

****************************************************************************************************/

#include "RepRapFirmware.h"

Move::Move(Platform* p)
{
  Serial.println("Move constructor"); 
  platform = p;
  lastTime = platform->time();
  platform->setDirection(X_AXIS, FORWARDS);
  platform->setDirection(Y_AXIS, FORWARDS);
  platform->setDirection(Z_AXIS, FORWARDS);
  platform->setDirection(3, FORWARDS);
}

void Move::spin()
{
   unsigned long t = platform->time();
   if(t - lastTime < 300)
     return;
   lastTime = t;
   //Serial.println("tick");
   platform->step(X_AXIS);
   platform->step(Y_AXIS);
   platform->step(Z_AXIS);
   platform->step(3);
}
