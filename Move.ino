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
  active = false;
  platform = p;
  gCodes = g;
  dda = new DDA(this, platform);
}

void Move::Init()
{
  lastTime = platform->Time();
  for(char i = 0; i < DRIVES; i++)
    platform->SetDirection(i, FORWARDS);
  for(char i = 0; i <= AXES; i++)
    currentPosition[i] = 0.0;  
  lastTime = platform->Time();
  currentFeedrate = START_FEED_RATE;
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
  //char scratchString[STRING_LENGTH];
  if(!gCodes->ReadMove(nextMove))
    return;
/*  platform->Message(HOST_MESSAGE, "Move - got a move:");
  for(char i = 0; i <= DRIVES; i++)
  {
    ftoa(scratchString, nextMove[i], 3);
    platform->Message(HOST_MESSAGE, scratchString);
    platform->Message(HOST_MESSAGE, ", ");
  }
  platform->Message(HOST_MESSAGE, "\n");*/
  
  boolean work = dda->Init(currentPosition, nextMove);
  
  for(char i = 0; i <= AXES; i++)
    currentPosition[i] = nextMove[i];
    
  if(work)
    dda->Start();
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

//****************************************************************************************************

DDA::DDA(Move* m, Platform* p)
{
  active = false;
  move = m;
  platform = p;
}

boolean DDA::Init(float currentPosition[], float targetPosition[])
{
  char drive;
  active = false;
  totalSteps = -1;
  for(drive = 0; drive < DRIVES; drive++)
  {
    if(drive < AXES)
      delta[drive] = (long)((targetPosition[drive] - currentPosition[drive])*platform->DriveStepsPerUnit(drive));  //Absolute
    else
      delta[drive] = (long)(targetPosition[drive]*platform->DriveStepsPerUnit(drive));  // Relative
    directions[drive] = (delta[drive] >= 0);
    delta[drive] = abs(delta[drive]);
    if(delta[drive] > totalSteps)
      totalSteps = delta[drive];    
  }
  if(totalSteps <= 0)
    return false;
  counter[0] = totalSteps/2;
  for(drive = 1; drive < DRIVES; drive++)
    counter[drive] = counter[0];
  stepCountDown = totalSteps;
  return true;
}

void DDA::Start()
{
  for(char drive = 0; drive < DRIVES; drive++)
    platform->SetDirection(drive, directions[drive]);
  platform->SetInterrupt(300);
  active = true;  
}

void DDA::Step()
{
  if(!active)
    return;
    
  for(char drive = 0; drive < DRIVES; drive++)
  {
    counter[drive] += delta[drive];
    if(counter[drive] > 0)
    {
      platform->Step(drive);
      counter[drive] -= totalSteps;
    }
  }
  
  // Deal with feedrates here
  
  stepCountDown--;
  active = stepCountDown > 0;
}
