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

Move::Move(Platform* p, GCodes* g)
{
  //Serial.println("Move constructor"); 
  platform = p;
  gCodes = g;
  active = false;
}

void Move::Init()
{
  lastTime = platform->Time();
  for(char i = 0; i < DRIVES; i++)
    platform->SetDirection(i, FORWARDS);
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
  Qmove();
}


void Move::Qmove()
{
  if(!gCodes->ReadMove(nextMove))
    return;
  
}
