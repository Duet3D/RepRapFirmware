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
  if(!gCodes->ReadMove(nextMove))
    return;
    
  //FIXME
  float u = platform->Jerk(X_AXIS);
  float v = u;
  
  boolean work = dda->Init(currentPosition, nextMove, u, v);
  
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

/*

This sets up the DDA to take us between two positions and extrude states.
The start velocity is u, and the end one is v.  The requested maximum feedrate
is in targetPosition[DRIVES].

Almost everything that needs to be done to set this up is also useful
for GCode look-ahead, so this one function is used for both.  It flags when
u and v cannot be satisfied with the distance available and reduces them 
proportionately to give values that can just be achieved, which is why they
are passed by reference.

The return value is true for an actual move, false for a zero-length (i.e. null) move.

*/
boolean DDA::Init(float currentPosition[], float targetPosition[], float& u, float& v)
{
  char drive;
  active = false;
  velocitiesAltered = false;
  totalSteps = -1;
  distance = 0; // X+Y+Z
  float d;
  char axesMoving = 0;
  
  // How far are we going, both in steps and in mm?
  
  for(drive = 0; drive < DRIVES; drive++)
  {
    if(drive < AXES)
    {
      d = targetPosition[drive] - currentPosition[drive];
      delta[drive] = (long)(d*platform->DriveStepsPerUnit(drive));  //Absolute
      distance += d*d;
    } else
      delta[drive] = (long)(targetPosition[drive]*platform->DriveStepsPerUnit(drive));  // Relative
    if(delta[drive] >= 0)
      directions[drive] = FORWARDS;
    else
      directions[drive] = BACKWARDS;
    delta[drive] = abs(delta[drive]);
    
    // Which axes (if any) are moving?
    
    if(drive == X_AXIS && delta[drive] > 0)
      axesMoving |= 1;
    if(drive == Y_AXIS && delta[drive] > 0)
      axesMoving |= 2;
    if(drive == Z_AXIS && delta[drive] > 0)
      axesMoving |= 4;   
    
    // Keep track of the biggest drive move in totalSteps
    
    if(delta[drive] > totalSteps)
      totalSteps = delta[drive];    
  }
  
  // Not going anywhere?
  
  if(totalSteps <= 0)
    return false;
  
  // Set up the DDA
  
  counter[0] = totalSteps/2;
  for(drive = 1; drive < DRIVES; drive++)
    counter[drive] = counter[0];
  
  // Acceleration and velocity calculations
  
  distance = sqrt(distance);
  float acc;
  
  if(axesMoving|4) // Z involved?
  {
    acc = platform->Acceleration(Z_AXIS);
    velocity = platform->Jerk(Z_AXIS);
  } else // No - only X, Y and Es
  {
    acc = platform->Acceleration(X_AXIS);
    velocity = platform->Jerk(X_AXIS);
  }
 
  // If velocities requested are (almost) zero, set them to the jerk
  
  if(v < 0.01)
    v = velocity; 
  if(u < 0.01)
    u = velocity;

  // At which DDA step should we stop accelerating?
  
  d = 0.5*(targetPosition[DRIVES]*targetPosition[DRIVES] - u*u)/acc; // d = (v1^2 - v0^2)/2a
  stopAStep = (long)((d*totalSteps)/distance);
  
  // At which DDA step should we start decelerating?
  
  d = 0.5*(v*v - targetPosition[DRIVES]*targetPosition[DRIVES])/acc;  // This should be 0 or negative...
  startDStep = totalSteps + (long)((d*totalSteps)/distance);
  
  // If acceleration stop is at or after deceleration start, then the distance moved
  // is not enough to get to full speed.
  
  if(stopAStep >= startDStep)
  {
    // Work out the point at which to stop accelerating and then
    // immediately start decelerating.
    
    dCross = 0.5*(0.5*(v*v - u*u)/acc + distance);
    
    if(dCross < 0.0 || dCross > distance)
    {
      // With the acceleration available, it is not possible
      // to satisfy u and v within the distance; reduce u and v 
      // proportionately to get ones that work and flag the fact.  
      // The result is two velocities that can just be accelerated
      // or decelerated between over the distance to get
      // from one to the other.
      
      float k = v/u;
      u = 2.0*acc*distance/(k*k - 1);
      if(u >= 0.0)
      {
        u = sqrt(u);
        v = k*u;
      } else
      {
        v = sqrt(-u);
        u = v/k;
      }
      
      dCross = 0.5*(0.5*(v*v - u*u)/acc + distance);
      velocitiesAltered = true;
    }
    
    // The DDA steps at which acceleration stops and deceleration starts
    
    stopAStep = (long)((dCross*totalSteps)/distance);
    startDStep = stopAStep + 1;
  }
  
  // The initial velocity
  
  velocity = u;
  
  // Sanity check
  
  if(velocity <= 0.0)
  {
    velocity = 1.0;
    platform->Message(HOST_MESSAGE, "DDA.Init(): Zero or negative initial velocity!");
  }
  
  // How far have we gone?
  
  stepCount = 0;
  
  // Guess that the first DDA move will be in roughly the direction 
  // recorded in axesMoving.  This is a simple heuristic, and any
  // small error will be forgotten with the very next step.
  
  timeStep = move->stepDistances[axesMoving]/velocity;
  
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
