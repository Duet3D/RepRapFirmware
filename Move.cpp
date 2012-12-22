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
  time = platform->time();
  platform->setDirection(X_AXIS, FORWARDS);
}

void Move::spin()
{
   unsigned long t = platform->time();
   if(t - time < 3000000)
     return;
   time = t;
   Serial.println("tick");
   platform->step(X_AXIS);
}
