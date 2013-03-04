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
  //Serial.println("Move constructor"); 
  platform = p;
  active = false;
}

void Move::Init()
{
  lastTime = platform->Time();
  platform->SetDirection(X_AXIS, FORWARDS);
  platform->SetDirection(Y_AXIS, FORWARDS);
  platform->SetDirection(Z_AXIS, FORWARDS);
  platform->SetDirection(3, FORWARDS);
  active = true;  
}

void Move::Exit()
{
  active = false;
}

void Move::Spin()
{
  if(!active)
    return;
    
   unsigned long t = platform->Time();
   if(t - lastTime < 300)
     return;
   lastTime = t;
   //Serial.println("tick");
/*  platform->step(X_AXIS);
   platform->step(Y_AXIS);
   platform->step(Z_AXIS);
   platform->step(3);
   */
}
