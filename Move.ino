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
  int8_t i;
  active = false;
  platform = p;
  gCodes = g;
  
  // Build the DDA ring
  
  ddaRingAddPointer = new DDA(this, platform, NULL);
  dda = ddaRingAddPointer;
  for(i = 1; i < DDA_RING_LENGTH; i++)
    dda = new DDA(this, platform, dda);
  ddaRingAddPointer->next = dda;
  
  dda = NULL;
  
  // Build the lookahead ring
  
  lookAheadRingAddPointer = new LookAhead(this, platform, NULL);
  lookAheadRingGetPointer = lookAheadRingAddPointer;
  for(i = 1; i < LOOK_AHEAD_RING_LENGTH; i++)
    lookAheadRingGetPointer = new LookAhead(this, platform, lookAheadRingGetPointer);
  lookAheadRingAddPointer->next = lookAheadRingGetPointer;
  
  // Set the backwards pointers and flag them all as free
  
  lookAheadRingGetPointer = lookAheadRingAddPointer; 
  for(i = 0; i <= LOOK_AHEAD_RING_LENGTH; i++)
  {
    lookAheadRingAddPointer = lookAheadRingAddPointer->Next();
    lookAheadRingAddPointer->previous = lookAheadRingGetPointer;
    lookAheadRingGetPointer = lookAheadRingAddPointer;
  }    
  
  lookAheadDDA = new DDA(this, platform, NULL);
  
}

void Move::Init()
{
  int8_t i, j;
  
  for(i = 0; i < DRIVES; i++)
    platform->SetDirection(i, FORWARDS);
    
  // Set the current position to the origin
  
  for(i = 0; i <= AXES; i++)
    currentPosition[i] = 0.0;
  currentFeedrate = START_FEED_RATE;
  
  // Empty the rings
  
  ddaRingGetPointer = ddaRingAddPointer; 
  ddaRingLocked = false;
  
  for(i = 0; i <= LOOK_AHEAD_RING_LENGTH; i++)
  {
    lookAheadRingAddPointer->Release();
    lookAheadRingAddPointer = lookAheadRingAddPointer->Next();
  }
  
  lookAheadRingGetPointer = lookAheadRingAddPointer;
  lookAheadRingCount = 0;
  
  addNoMoreMoves = false;
  
  // Put the origin on the lookahead ring with zero velocity so it corresponds with the currentPosition
  
  for(i = 0; i < DRIVES; i++)
    nextMove[i] = 0.0;
  nextMove[DRIVES] = currentFeedrate;
  checkEndStopsOnNextMove = false;
  LookAheadRingAdd(nextMove, 0.0, false);
  
  // Now remove it from the ring; it will remain as what is now the
  // previous move, so the first real move will see that as
  // the place to move from.  Flag it as fully processed.
  
  LookAheadRingGet()->Release();
  
  // The stepDistances arrays are look-up tables of the Euclidean distance 
  // between the start and end of a step.  If the step is just along one axis,
  // it's just that axis's step length.  If it's more, it is a Pythagoran 
  // sum of all the axis steps that take part.
  
  float d, e;
  
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
  

  currentFeedrate = START_FEED_RATE;
  lastTime = platform->Time();
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
    
  DoLookAhead();
 
  if(!DDARingFull())
  {
     LookAhead* nextFromLookAhead = LookAheadRingGet();
     if(nextFromLookAhead != NULL)
     {
       if(!DDARingAdd(nextFromLookAhead))
         platform->Message(HOST_MESSAGE, "Can't add to non-full DDA ring!\n"); // Should never happen...
     }
  }
  
  if(addNoMoreMoves || LookAheadRingFull())
   return;
    
  if(gCodes->ReadMove(nextMove, checkEndStopsOnNextMove))
  {
    if(GetMovementType(currentPosition, nextMove) == noMove)
    {
      currentFeedrate = nextMove[DRIVES]; // Might be G1 with just an F field
      return;
    }
    if(!LookAheadRingAdd(nextMove, 0.0, checkEndStopsOnNextMove))
      platform->Message(HOST_MESSAGE, "Can't add to non-full look ahead ring!\n"); // Should never happen...
    for(int8_t i = 0; i < AXES; i++)
      currentPosition[i] = nextMove[i];
    currentFeedrate = nextMove[DRIVES];
  }
}

boolean Move::GetCurrentState(float m[])
{
  if(LookAheadRingFull())
    return false;
    
  for(int8_t i = 0; i < DRIVES; i++)
  {
    if(i < AXES)
      m[i] = currentPosition[i];
    else
      m[i] = 0.0;
  }
  m[DRIVES] = currentFeedrate;
  return true;
}

int8_t Move::GetMovementType(float p0[], float p1[])
{
  int8_t result = noMove;
  for(int8_t drive = 0; drive < DRIVES; drive++)
  {
    if(drive < AXES)
    {
      if( (long)roundf((p1[drive] - p0[drive])*platform->DriveStepsPerUnit(drive)) )
      {
        if(drive == Z_AXIS)
          result |= zMove;
        else
          result |= xyMove;
      }
    } else
    {
      if( (long)roundf(p1[drive]*platform->DriveStepsPerUnit(drive)) )
        result |= eMove;
    }
  }
  return result;
}


boolean Move::DDARingAdd(LookAhead* lookAhead)
{
  if(GetDDARingLock())
  {
    if(DDARingFull())
    {
      ReleaseDDARingLock();
      return false;
    }
    if(ddaRingAddPointer->Active())
    {
      platform->Message(HOST_MESSAGE, "Attempt to alter an active ring buffer entry!\n"); // Should never happen...
      ReleaseDDARingLock();
      return false;
    }
    
    // We don't care about Init()'s return value - that should all have been sorted
    // out by LookAhead.
    
    float u, v;
    ddaRingAddPointer->Init(lookAhead, u, v);
    ddaRingAddPointer = ddaRingAddPointer->Next();
    ReleaseDDARingLock();
    return true;
  }
  return false;
}


DDA* Move::DDARingGet()
{
  DDA* result = NULL;
  if(GetDDARingLock())
  {
    if(DDARingEmpty())
    {
      ReleaseDDARingLock();
      return NULL;
    }
    result = ddaRingGetPointer;
    ddaRingGetPointer = ddaRingGetPointer->Next();
    ReleaseDDARingLock();
    return result;
  }
  return NULL;
}


void Move::DoLookAhead()
{
  if(LookAheadRingEmpty())
    return;
    
  LookAhead* n0;
  LookAhead* n1;
  LookAhead* n2;
  
  float u, v;
    

/*    n2 = lookAheadRingAddPointer->Previous();
    n1 = n2->Previous();
    n0 = n1->Previous();
    while(n1 != lookAheadRingGetPointer)
    {
      if(n1->Processed() & vCosineSet)
      {
        u = n0->V();
        v = n1->V();
        if(lookAheadDDA->Init(n0->EndPosition(), n1->EndPosition(), u, v) & change)
        {
          n0->SetV(u);
          n1->SetV(v); 
        }
      }
      n2 = n1;
      n1 = n0;
      n0 = n0->Previous();
    } 
  }*/
  if(addNoMoreMoves || !gCodes->PrintingAFile() || lookAheadRingCount > 1)
  {  
    n1 = lookAheadRingGetPointer;
    n0 = n1->Previous();
    n2 = n1->Next();
    while(n2 != lookAheadRingAddPointer)
    {
      if(n1->Processed() == unprocessed)
      {
        float c = n1->Cosine();
        c = n1->EndPoint()[DRIVES]*c;
        if(c <= 0)
        {
          int8_t mt = GetMovementType(n0->EndPoint(), n1->EndPoint());
          if(mt & zMove)
            c = platform->InstantDv(Z_AXIS);
          else if (mt & xyMove)
            c = platform->InstantDv(X_AXIS);
          else
            c = platform->InstantDv(AXES); // value for first extruder - slight hack
        }
        n1->SetV(c);
        //n1->SetProcessed(vCosineSet);
        n1->SetProcessed(complete);
      }
      n0 = n1;
      n1 = n2;
      n2 = n2->Next();
    }
    if(addNoMoreMoves || !gCodes->PrintingAFile())
    {
      n1->SetV(0);
      n1->SetProcessed(complete);
    }
  }
}


void Move::Interrupt()
{
  // Have we got a live DDA?
  
  if(dda == NULL)
  {
    // No - see if a new one is available.
    
    dda = DDARingGet();    
    if(dda != NULL)
      dda->Start(true);  // Yes - got it.  So fire it up.
    return;   
  }
  
  // We have a DDA.  Has it finished?
  
  if(dda->Active())
  {
    // No - it's still live.  Step it and return.
    
    dda->Step(true);
    return;
  }
  
  // Yes - it's finished.  Throw it away so the code above will then find a new one.
  
  dda = NULL;
}


boolean Move::LookAheadRingAdd(float ep[], float vv, boolean ce)
{
    if(LookAheadRingFull())
      return false;
    lookAheadRingAddPointer->Init(ep, vv, ce);
    lookAheadRingAddPointer = lookAheadRingAddPointer->Next();
    lookAheadRingCount++;
    return true;
}


LookAhead* Move::LookAheadRingGet()
{
  LookAhead* result;
  if(LookAheadRingEmpty())
    return NULL;
  result = lookAheadRingGetPointer;
  if(!(result->Processed() & complete))
    return NULL;
  lookAheadRingGetPointer = lookAheadRingGetPointer->Next();
  lookAheadRingCount--;
  return result;
}


// FIXME
// This function is never normally called.  It is a test to time
// the interrupt function.  To activate it, uncomment the line that calls
// this in Platform.ino.

void Move::InterruptTime()
{
/*  char buffer[50];
  float a[] = {1.0, 2.0, 3.0, 4.0, 5.0};
  float b[] = {2.0, 3.0, 4.0, 5.0, 6.0};
  float u = 50;
  float v = 50;
  lookAheadDDA->Init(a, b, u, v);
  lookAheadDDA->Start(false);
  unsigned long t = platform->Time();
  for(long i = 0; i < 100000; i++) 
    lookAheadDDA->Step(false);
  t = platform->Time() - t;
  platform->Message(HOST_MESSAGE, "Time for 100000 calls of the interrupt function: ");
  sprintf(buffer, "%ld", t);
  platform->Message(HOST_MESSAGE, buffer);
  platform->Message(HOST_MESSAGE, " microseconds.\n");*/
}

//****************************************************************************************************

DDA::DDA(Move* m, Platform* p, DDA* n)
{
  active = false;
  move = m;
  platform = p;
  next = n;
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

The return value is indicates if the move is a trapezium or triangle, and if
the u and u values need to be changed.

Every drive has an acceleration associated with it, so when more than one drive is
moving there have to be rules of precedence that say which acceleration (and which
instantDv value) to use.

The rules are these:

  if Z is moving
    Use Z acceleration
  else if X and/or Y are moving
    Use X acceleration
  else
    Use the acceleration for the extruder that's moving.

In the case of multiple extruders moving at once, their minimum acceleration (and its
associated instantDv) are used.  The variables axesMoving and extrudersMoving track what's 
going on.  The bits in the int8_t axesMoving are ORed:

  msb -> 00000ZYX <- lsb
  
a 1 meaning that that axis is moving.  The bits of extrudersMoving contain a similar
pattern for the moving extruders.
    
Note that all this assumes that X and Y accelerations are equal, though in fact there is a 
value stored for each.

In the case of only extruders moving, the distance moved is taken to be the Pythagoran distance in
the configuration space of the extruders.

TODO: Worry about having more than eight extruders...

*/

MovementProfile DDA::Init(LookAhead* lookAhead, float& u, float& v)
{
  int8_t drive;
  active = false;
  MovementProfile result = moving;
  totalSteps = -1;
  distance = 0.0; // X+Y+Z
  float eDistance = 0.0;
  float d;
  float* targetPosition = lookAhead->EndPoint();
  v = lookAhead->V();
  float* positionNow = lookAhead->Previous()->EndPoint();
  u = lookAhead->Previous()->V();
  checkEndStops = lookAhead->CheckEndStops();
  
  // How far are we going, both in steps and in mm?
  
  for(drive = 0; drive < DRIVES; drive++)
  {
    if(drive < AXES)
    {
      d = targetPosition[drive] - positionNow[drive];  //Absolute
      distance += d*d;
      delta[drive] = (long)roundf(d*platform->DriveStepsPerUnit(drive));
    } else
    {
      delta[drive] = (long)roundf(targetPosition[drive]*platform->DriveStepsPerUnit(drive));  // Relative
      eDistance += targetPosition[drive]*targetPosition[drive];
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
  
  // Not going anywhere?  Should have been chucked away before we got here.
  
  if(totalSteps <= 0)
  {
    platform->Message(HOST_MESSAGE, "DDA.Init(): Null movement!\n");
    return result;
  }
  
  // Set up the DDA
  
  result = moving;
  
  counter[0] = -totalSteps/2;
  for(drive = 1; drive < DRIVES; drive++)
    counter[drive] = counter[0];
  
  // Acceleration and velocity calculations
  
  distance = sqrt(distance);
  
  // Decide the appropriate acceleration and instantDv values
  // timeStep is set here to the distance of the
  // corresponding axis step.  It will be divided
  // by a velocity later. 

  if(delta[Z_AXIS]) // Z involved?
  {
    acceleration = platform->Acceleration(Z_AXIS);
    instantDv = platform->InstantDv(Z_AXIS);
    timeStep = 1.0/platform->DriveStepsPerUnit(Z_AXIS);
  } else if(delta[X_AXIS] || delta[Y_AXIS]) // X or Y involved?
  {
    acceleration = platform->Acceleration(X_AXIS);
    instantDv = platform->InstantDv(X_AXIS);
    timeStep = 1.0/platform->DriveStepsPerUnit(X_AXIS); // Slight hack
  } else // Must be extruders only
  {
    acceleration = FLT_MAX; // Slight hack
    distance = sqrt(eDistance);
    for(drive = AXES; drive < DRIVES; drive++)
    {
      if(delta[drive])
      {
        if(platform->Acceleration(drive) < acceleration)
        {
          acceleration = platform->Acceleration(drive);
          instantDv = platform->InstantDv(drive);
          timeStep = 1.0/platform->DriveStepsPerUnit(drive);
        }
      }
    }    
  }
 
  // If velocities requested are (almost) zero, set them to instantDv
  
  if(v < 0.01) // Set change here?
    v = instantDv; 
  if(u < 0.01)
    u = instantDv;

  // At which DDA step should we stop accelerating?  targetPosition[DRIVES] contains
  // the desired feedrate.
  
  d = 0.5*(targetPosition[DRIVES]*targetPosition[DRIVES] - u*u)/acceleration; // d = (v1^2 - v0^2)/2a
  stopAStep = (long)roundf((d*totalSteps)/distance);
  
  // At which DDA step should we start decelerating?
  
  d = 0.5*(v*v - targetPosition[DRIVES]*targetPosition[DRIVES])/acceleration;  // This should be 0 or negative...
  startDStep = totalSteps + (long)roundf((d*totalSteps)/distance);
  
  // If acceleration stop is at or after deceleration start, then the distance moved
  // is not enough to get to full speed.
  
  if(stopAStep >= startDStep)
  {
    result = noFlat;
    
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
      
      result = change;
      
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
    platform->Message(HOST_MESSAGE, "DDA.Init(): Zero or negative initial velocity!\n");
  }
  
  // How far have we gone?
  
  stepCount = 0;
  
  // timeStep is an axis step distance at this point; divide it by the
  // velocity to get time.
  
  timeStep = timeStep/velocity;
  
  lookAhead->Release();
  
  return result;
}

void DDA::Start(boolean noTest)
{
  for(int8_t drive = 0; drive < DRIVES; drive++)
    platform->SetDirection(drive, directions[drive]);
  if(noTest)
    platform->SetInterrupt((long)(1.0e6*timeStep)); // microseconds
  active = true;  
}

void DDA::Step(boolean noTest)
{
  if(!active && noTest)
    return;
  
  uint8_t axesMoving = 0;
  uint8_t extrudersMoving = 0;
  
  for(int8_t drive = 0; drive < DRIVES; drive++)
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
        
      // Hit anything?
  
      if(checkEndStops)
      {
        EndStopHit esh = platform->Stopped(drive);
        if(esh == lowHit)
        {
          move->HitLowStop(drive);
          active = false;
        }
        if(esh == highHit)
        {
          move->HitHighStop(drive);
          active = false;
        }
      }        
    }
  }
  
  // May have hit a stop, so test active here
  
  if(active) 
  {
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
  }
  
  if(!active && noTest)
    platform->SetInterrupt(STANDBY_INTERRUPT_RATE);
}

//***************************************************************************************************

LookAhead::LookAhead(Move* m, Platform* p, LookAhead* n)
{
  move = m;
  platform = p;
  next = n;
}

void LookAhead::Init(float ep[], float vv, boolean ce)
{
  v = vv;
  for(int8_t i = 0; i <= DRIVES; i++)
    endPoint[i] = ep[i];
  
  checkEndStops = ce;
  
  // Cosines are lazily evaluated; flag this
  // as unevaluated
  
  cosine = 2.0;
    
  // Only bother with lookahead when we
  // are printing a file, so set processed
  // complete when we aren't.
  
  if(reprap.GetGCodes()->PrintingAFile())
    processed = unprocessed;
  else
    processed = complete|vCosineSet|upPass;
}

// This returns the cosine of the angle between
// the movement up to this, and the movement
// away from this.  Note that it
// includes Z movements, though Z values will almost always 
// not change.  Uses lazy evaluation.

float LookAhead::Cosine()
{
  if(cosine < 1.5)
    return cosine;
    
  cosine = 0.0;
  float a2 = 0.0;
  float b2 = 0.0;
  float m1;
  float m2;
  for(int8_t i = 0; i < AXES; i++)
  {
    m1 = endPoint[i] - Previous()->endPoint[i];
    m2 = Next()->endPoint[i] - endPoint[i];
    a2 += m1*m1;
    b2 += m2*m2;
    cosine += m1*m2;
  }
  
  cosine = cosine/( (float)sqrt(a2) * (float)sqrt(b2) );
  return cosine;
}





