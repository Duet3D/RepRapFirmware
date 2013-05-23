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
  unsigned char i, j;
  
  for(i = 0; i < DRIVES; i++)
    platform->SetDirection(i, FORWARDS);
  for(i = 0; i <= AXES; i++)
    currentPosition[i] = 0.0;  

  float d, e;
  
  // The stepDistances arrays are look-up tables of the Euclidean distance 
  // between the start and end of a step.  If the step is just along one axis,
  // it's just that axis's step length.  If it's more, it is a Pythagoran 
  // sum of all the axis steps that take part.
  
  for(i = 0; i < (1<<AXES); i++)
  {
    d = 0.0;
    for(j = 0; j < AXES; j++)
    {
       if(i & (1<<j))
       {
          e = 1.0/platform->DriveStepsPerUnit(j);
          d += e*e;
       }
    }
    stepDistances[i] = sqrt(d);
  }
  
  for(i = 0; i < (1<<(DRIVES-AXES)); i++)
  {
    d = 0.0;
    for(j = 0; j < (DRIVES-AXES); j++)
    {
       if(i & (1<<j))
       {
          e = 1.0/platform->DriveStepsPerUnit(AXES + j);
          d += e*e;
       }
    }
    extruderStepDistances[i] = sqrt(d);
  }
  
  // We don't want 0.  If no axes/extruders are moving these should never be used.
  // But try to be safe.
  
  stepDistances[0] = 1.0/platform->DriveStepsPerUnit(AXES);
  extruderStepDistances[0] = stepDistances[0];
  
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
  if(!gCodes->ReadMove(nextMove))
    return;
    
  //FIXME
  float u = 0.0; // This will provoke the code to select the jerk values.
  float v = 0.0;
  
  boolean work = dda->Init(currentPosition, nextMove, u, v);
  
  for(char i = 0; i < AXES; i++)
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

DDA::Init(...) 

Sets up the DDA to take us between two positions and extrude states.
The start velocity is u, and the end one is v.  The requested maximum feedrate
is in targetPosition[DRIVES].

Almost everything that needs to be done to set this up is also useful
for GCode look-ahead, so this one function is used for both.  It flags when
u and v cannot be satisfied with the distance available and reduces them 
proportionately to give values that can just be achieved, which is why they
are passed by reference.

The return value is true for an actual move, false for a zero-length (i.e. null) move.

Every drive has an acceleration associated with it, so when more than one drive is
moving there have to be rules of precedence that say which acceleration (and which
jerk value) to use.

The rules are these:

  if Z is moving
    Use Z acceleration
  else if X and/or Y are moving
    Use X acceleration
  else
    Use the acceleration for the extruder that's moving.

In the case of multiple extruders moving at once, their minimum acceleration (and its
associated jerk) are used.  The variables axesMoving and extrudersMoving track what's 
going on.  The bits in the char axesMoving are ORed:

  msb -> 00000ZYX <- lsb
  
a 1 meaning that that axis is moving.  The bits of extrudersMoving contain a similar
pattern for the moving extruders.
    
Note that all this assumes that X and Y accelerations are equal, though in fact there is a 
value stored for each.

In the case of only extruders moving, the distance moved is taken to be the Pythagoran distance in
the configuration space of the extruders.

TODO: Worry about having more than eight extruders...

*/

boolean DDA::Init(float currentPosition[], float targetPosition[], float& u, float& v)
{
  char drive;
  active = false;
  velocitiesAltered = false;
  totalSteps = -1;
  distance = 0.0; // X+Y+Z
  float eDistance = 0.0;
  float d;
  unsigned char axesMoving = 0;
  unsigned char extrudersMoving = 0;
  
  // How far are we going, both in steps and in mm?
  
  for(drive = 0; drive < DRIVES; drive++)
  {
    if(drive < AXES)
    {
      d = targetPosition[drive] - currentPosition[drive];  //Absolute
      distance += d*d;
      delta[drive] = (long)(d*platform->DriveStepsPerUnit(drive));
      if(delta[drive])
        axesMoving |= 1<<drive;
    } else
    {
      delta[drive] = (long)(targetPosition[drive]*platform->DriveStepsPerUnit(drive));  // Relative
      eDistance += targetPosition[drive]*targetPosition[drive];
      if(delta[drive])
        extrudersMoving |= 1<<(drive - AXES);
    }
    
    if(delta[drive] >= 0)
      directions[drive] = FORWARDS;
    else
      directions[drive] = BACKWARDS;
    delta[drive] = abs(delta[drive]);  
    
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
  
  if(axesMoving & (1<<Z_AXIS)) // Z involved?
  {
    acceleration = platform->Acceleration(Z_AXIS);
    jerk = platform->Jerk(Z_AXIS);
  } else if(axesMoving) // X or Y involved?
  {
    acceleration = platform->Acceleration(X_AXIS);
    jerk = platform->Jerk(X_AXIS);
  } else // Must be extruders only
  {
    acceleration = FLT_MAX; // Slight hack
    distance = sqrt(eDistance);
    for(drive = AXES; drive < DRIVES; drive++)
    {
      if(extrudersMoving & (1<<(drive - AXES)))
      {
        if(platform->Acceleration(drive) < acceleration)
        {
          acceleration = platform->Acceleration(drive);
          jerk = platform->Jerk(drive);
        }
      }
    }    
  }
 
  // If velocities requested are (almost) zero, set them to the jerk
  
  if(v < 0.01)
    v = jerk; 
  if(u < 0.01)
    u = jerk;

  // At which DDA step should we stop accelerating?  targetPosition[DRIVES] contains
  // the desired feedrate.
  
  d = 0.5*(targetPosition[DRIVES]*targetPosition[DRIVES] - u*u)/acceleration; // d = (v1^2 - v0^2)/2a
  stopAStep = (long)((d*totalSteps)/distance);
  
  // At which DDA step should we start decelerating?
  
  d = 0.5*(v*v - targetPosition[DRIVES]*targetPosition[DRIVES])/acceleration;  // This should be 0 or negative...
  startDStep = totalSteps + (long)((d*totalSteps)/distance);
  
  // If acceleration stop is at or after deceleration start, then the distance moved
  // is not enough to get to full speed.
  
  if(stopAStep >= startDStep)
  {
    // Work out the point at which to stop accelerating and then
    // immediately start decelerating.
    
    dCross = 0.5*(0.5*(v*v - u*u)/acceleration + distance);
    
    if(dCross < 0.0 || dCross > distance)
    {
      // With the acceleration available, it is not possible
      // to satisfy u and v within the distance; reduce u and v 
      // proportionately to get ones that work and flag the fact.  
      // The result is two velocities that can just be accelerated
      // or decelerated between over the distance to get
      // from one to the other.
      
      float k = v/u;
      u = 2.0*acceleration*distance/(k*k - 1);
      if(u >= 0.0)
      {
        u = sqrt(u);
        v = k*u;
      } else
      {
        v = sqrt(-u);
        u = v/k;
      }
      
      dCross = 0.5*(0.5*(v*v - u*u)/acceleration + distance);
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
  
  unsigned char axesMoving = 0;
  unsigned char extrudersMoving = 0;
  
  for(char drive = 0; drive < DRIVES; drive++)
  {
    counter[drive] += delta[drive];
    if(counter[drive] > 0)
    {
      if(noTest)
        platform->Step(drive);
      counter[drive] -= totalSteps;
      
      if(drive < AXES)
        axesMoving |= 1<<drive;
      else
        extrudersMoving |= 1<<(drive - AXES);
    }
  }
  
  // Simple Euler integration to get velocities.
  // Maybe one day do a Runge-Kutta?
  
  if(stepCount < stopAStep)
  {
    if(axesMoving)
      timeStep = move->stepDistances[axesMoving]/velocity;
    else
      timeStep = move->extruderStepDistances[extrudersMoving]/velocity;
    velocity += acceleration*timeStep;
    if(noTest)
        platform->SetInterrupt((long)(1.0e6*timeStep));
  }
  
  if(stepCount >= startDStep)
  {
    if(axesMoving)
      timeStep = move->stepDistances[axesMoving]/velocity;
    else
      timeStep = move->extruderStepDistances[extrudersMoving]/velocity;
    velocity -= acceleration*timeStep;
    if(noTest)
      platform->SetInterrupt((long)(1.0e6*timeStep));
  }
  
  stepCount++;
  active = stepCount < totalSteps;
  
  if(!active && noTest)           //???
    platform->SetInterrupt(-1);
}

//****************************************************************************************************

DDARingBuffer::DDARingBuffer(Move* m, Platform* p)
{
  platform = p;
  for(addPointer = 0; addPointer < RING_LENGTH; addPointer++)
   ring[addPointer] = new DDA(m, p);
  addPointer = 0;
  getPointer = 0;
  locked = false; 
}

