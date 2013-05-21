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
  char i;
  
  for(i = 0; i < DRIVES; i++)
    platform->SetDirection(i, FORWARDS);
  for(i = 0; i <= AXES; i++)
    currentPosition[i] = 0.0;  
  lastTime = platform->Time();
  currentFeedrate = START_FEED_RATE;
  float dx = 1.0/platform->DriveStepsPerUnit(X_AXIS);
  float dy = 1.0/platform->DriveStepsPerUnit(Y_AXIS);
  float dz = 1.0/platform->DriveStepsPerUnit(Z_AXIS);
  stepDistances[0] = dx; // Should never be used.  Wrong, but safer than 0.0...
  stepDistances[1] = dx;
  stepDistances[2] = dy;
  stepDistances[3] = sqrt(dx*dx + dy*dy);
  stepDistances[4] = dz;
  stepDistances[5] = sqrt(dx*dx + dz*dz);
  stepDistances[6] = sqrt(dy*dy + dz*dz);
  stepDistances[7] = sqrt(dx*dx + dy*dy + dz*dz);

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
    dda->Start(true);
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
  float dist = 0; // X+Y+Z
  float d;
  char axesMoving = 0;
  for(drive = 0; drive < DRIVES; drive++)
  {
    if(drive < AXES)
    {
      d = targetPosition[drive] - currentPosition[drive];
      delta[drive] = (long)(d*platform->DriveStepsPerUnit(drive));  //Absolute
      dist += d*d;
    } else
      delta[drive] = (long)(targetPosition[drive]*platform->DriveStepsPerUnit(drive));  // Relative
    if(delta[drive] >= 0)
      directions[drive] = FORWARDS;
    else
      directions[drive] = BACKWARDS;
    delta[drive] = abs(delta[drive]);
    if(drive == X_AXIS && delta[drive] > 0)
      axesMoving |= 1;
    if(drive == Y_AXIS && delta[drive] > 0)
      axesMoving |= 2;
    if(drive == Z_AXIS && delta[drive] > 0)
      axesMoving |= 4;      
    if(delta[drive] > totalSteps)
      totalSteps = delta[drive];    
  }
  if(totalSteps <= 0)
    return false;
  counter[0] = totalSteps/2;
  for(drive = 1; drive < DRIVES; drive++)
    counter[drive] = counter[0];
  dist = sqrt(dist);
  float acc;
  if(axesMoving|4)
  {
    acc = platform->Acceleration(Z_AXIS);
    velocity = platform->Jerk(Z_AXIS);
  } else
  {
    acc = platform->Acceleration(X_AXIS);
    velocity = platform->Jerk(X_AXIS);
  }
  
  timeStep = move->stepDistances[1]/velocity;
  d = 0.5*(targetPosition[DRIVES]*targetPosition[DRIVES] - velocity*velocity)/acc; // d = (v^2 - u^2)/2a
  stopAStep = (long)((d*totalSteps)/dist);
  startDStep = totalSteps - stopAStep;
  if(stopAStep > startDStep)
  {
      stopAStep = totalSteps/2;
      startDStep = stopAStep + 1;
  }
  stepCount = 0;
  return true;
}

void DDA::Start(boolean noTest)
{
  for(char drive = 0; drive < DRIVES; drive++)
    platform->SetDirection(drive, directions[drive]);
  if(noTest)
    platform->SetInterrupt((long)(1.0e6*timeStep));
  active = true;  
}

void DDA::Step(boolean noTest)
{
  if(!active && noTest)
    return;
  
  char axesMoving = 0;
  
  counter[X_AXIS] += delta[X_AXIS];
  if(counter[X_AXIS] > 0)
  {
    if(noTest)
      platform->Step(X_AXIS);
    axesMoving |= 1;
    counter[X_AXIS] -= totalSteps;
  }  
  
  counter[Y_AXIS] += delta[Y_AXIS];
  if(counter[Y_AXIS] > 0)
  {
    if(noTest)
      platform->Step(Y_AXIS);
    axesMoving |= 2;
    counter[Y_AXIS] -= totalSteps;
  }  
  
  counter[Z_AXIS] += delta[Z_AXIS];
  if(counter[Z_AXIS] > 0)
  {
    if(noTest)
      platform->Step(Z_AXIS);
    axesMoving |= 4;
    counter[Z_AXIS] -= totalSteps;
  }  
  
    
  for(char drive = AXES; drive < DRIVES; drive++)
  {
    counter[drive] += delta[drive];
    if(counter[drive] > 0)
    {
      if(noTest)
        platform->Step(drive);
      counter[drive] -= totalSteps;
    }
  }
  
  if(axesMoving)
  {
    if(stepCount < stopAStep)
    {
      timeStep = move->stepDistances[axesMoving]/velocity;
      if(axesMoving & 4)
        velocity += platform->Acceleration(Z_AXIS)*timeStep;
      else
        velocity += platform->Acceleration(X_AXIS)*timeStep;
      if(noTest)
        platform->SetInterrupt((long)(1.0e6*timeStep));
    }
    if(stepCount >= startDStep)
    {
      timeStep = move->stepDistances[axesMoving]/velocity;
      if(axesMoving & 4)
        velocity -= platform->Acceleration(Z_AXIS)*timeStep;
      else
        velocity -= platform->Acceleration(X_AXIS)*timeStep;
      if(noTest)
        platform->SetInterrupt((long)(1.0e6*timeStep));
    }
  }
  
  stepCount++;
  active = stepCount < totalSteps;
  if(!active && noTest)
    platform->SetInterrupt(-1);
}
