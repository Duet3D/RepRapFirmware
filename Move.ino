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
  platform = p;
  gCodes = g;
  active = false;
}

void Move::Init()
{
  lastTime = platform->Time();
  for(char i = 0; i < DRIVES; i++)
    platform->SetDirection(i, FORWARDS);
  for(char i = 0; i <= AXES; i++)
    currentPosition[i] = 0.0;  
  active = true;
  currentFeedrate = START_FEED_RATE;  
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
  platform->Message(HOST_MESSAGE, "Move - got a move:");
  for(char i = 0; i <= DRIVES; i++)
  {
    ftoa(scratchString, nextMove[i], 3);
    platform->Message(HOST_MESSAGE, scratchString);
    platform->Message(HOST_MESSAGE, ", ");
  }
  platform->Message(HOST_MESSAGE, "<br>\n");
  
  // Make it so
  
  for(char i = 0; i <= AXES; i++)
    currentPosition[i] = nextMove[i]; 
}

void Move::GetCurrentState(float m[])
{
  for(char i = 0; i < DRIVES; i++)
  {
    if(i < AXES)
      m[i] = currentPosition[i];
    else
      m[i] = 0.0;
  }
  m[DRIVES] = currentFeedrate;
}
