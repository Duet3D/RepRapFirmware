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
  
  // Set the lookahead backwards pointers
  
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
  
  // Put the origin on the lookahead ring with default velocity in the previous
  // position to the first one that will be used.
  
  lastMove = lookAheadRingAddPointer->Previous();
  
  for(i = 0; i < DRIVES; i++)
    lastMove->SetDriveZeroEndSpeed(0.0, i);
    
  lastMove->SetDriveZeroEndSpeed(START_FEED_RATE, DRIVES);
  
  checkEndStopsOnNextMove = false;
  
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

  currentFeedrate = -1.0;

  SetIdentityTransform();

  lastZHit = 0.0;
  zProbing = false;

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
    
  // Do some look-ahead work, if there's any to do
    
  DoLookAhead();
  
  // If there's space in the DDA ring, and there are completed
  // moves in the look-ahead ring, transfer them.
 
  if(!DDARingFull())
  {
     LookAhead* nextFromLookAhead = LookAheadRingGet();
     if(nextFromLookAhead != NULL)
     {
       if(!DDARingAdd(nextFromLookAhead))
         platform->Message(HOST_MESSAGE, "Can't add to non-full DDA ring!\n"); // Should never happen...
     }
  }
  
  // If we either don't want to, or can't, add to the look-ahead ring, go home.
  
  if(addNoMoreMoves || LookAheadRingFull())
   return;
 
  // If there's a G Code move available, add it to the look-ahead
  // ring for processing.
  
  if(gCodes->ReadMove(nextMove, checkEndStopsOnNextMove))
  {
	Transform(nextMove);

    currentFeedrate = nextMove[DRIVES]; // Might be G1 with just an F field
    
    int8_t mt = GetMovementType(lastMove->EndPoint(), nextMove);
    
    // Throw it away if there's no real movement.
    
    if(mt == noMove) 
       return;
     
    // Real move - record its feedrate with it, not here.
    
    currentFeedrate = -1.0;
    
    // Promote minimum feedrates
    
    if(mt & xyMove)
      nextMove[DRIVES] = fmax(nextMove[DRIVES], platform->InstantDv(X_AXIS));
    else if(mt & eMove) 
      nextMove[DRIVES] = fmax(nextMove[DRIVES], platform->InstantDv(AXES));
    else
      nextMove[DRIVES] = fmax(nextMove[DRIVES], platform->InstantDv(Z_AXIS));
      
    // Restrict maximum feedrates; assumes xy overrides e overrides z FIXME??
    
    if(mt & xyMove)
      nextMove[DRIVES] = fmin(nextMove[DRIVES], platform->MaxFeedrate(X_AXIS));
    else if(mt & eMove)
      nextMove[DRIVES] = fmin(nextMove[DRIVES], platform->MaxFeedrate(AXES)); // Picks up the value for the first extruder.  FIXME?
    else // Must be z
      nextMove[DRIVES] = fmin(nextMove[DRIVES], platform->MaxFeedrate(Z_AXIS));  // Assumes X and Y are equal.  FIXME?
    
    if(!LookAheadRingAdd(nextMove, 0.0, checkEndStopsOnNextMove))
      platform->Message(HOST_MESSAGE, "Can't add to non-full look ahead ring!\n"); // Should never happen...
  }
}


void Move::Diagnostics() 
{
  platform->Message(HOST_MESSAGE, "Move Diagnostics:\n");
/*  if(active)
    platform->Message(HOST_MESSAGE, " active\n");
  else
    platform->Message(HOST_MESSAGE, " not active\n");
  
  platform->Message(HOST_MESSAGE, " look ahead ring count: ");
  sprintf(scratchString, "%d\n", lookAheadRingCount);
  platform->Message(HOST_MESSAGE, scratchString);
  if(dda == NULL)
    platform->Message(HOST_MESSAGE, " dda: NULL\n");
  else
  {
    if(dda->Active())
      platform->Message(HOST_MESSAGE, " dda: active\n");
    else
      platform->Message(HOST_MESSAGE, " dda: not active\n");
    
  }
  if(ddaRingLocked)
    platform->Message(HOST_MESSAGE, " dda ring is locked\n");
  else
    platform->Message(HOST_MESSAGE, " dda ring is not locked\n");
  if(addNoMoreMoves)
    platform->Message(HOST_MESSAGE, " addNoMoreMoves is true\n\n");
  else
    platform->Message(HOST_MESSAGE, " addNoMoreMoves is false\n\n"); 
    */
}

// This returns false if it is not possible
// to use the result as the basis for the
// next move because the look ahead ring
// is full.  True otherwise.

bool Move::GetCurrentState(float m[])
{
  if(LookAheadRingFull())
    return false;
    
  for(int8_t i = 0; i < DRIVES; i++)
  {
    if(i < AXES)
      m[i] = lastMove->EndPoint()[i];
    else
      m[i] = 0.0;
  }
  if(currentFeedrate >= 0.0)
    m[DRIVES] = currentFeedrate;
  else
    m[DRIVES] = lastMove->EndPoint()[DRIVES];
  currentFeedrate = -1.0;
  InverseTransform(m);
  return true;
}

// Classify a move between two points.
// Is it (a combination of):
//   A Z movement?
//   An XY movement?
//   Extruder movements?

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

// Take an item from the look-ahead ring and add it to the DDA ring, if
// possible.

bool Move::DDARingAdd(LookAhead* lookAhead)
{
  if(GetDDARingLock())
  {
    if(DDARingFull())
    {
      ReleaseDDARingLock();
      return false;
    }
    if(ddaRingAddPointer->Active())  // Should never happen...
    {
      platform->Message(HOST_MESSAGE, "Attempt to alter an active ring buffer entry!\n");
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

// Get a movement from the DDA ring, if we can.

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

// Do the look-ahead calculations

void Move::DoLookAhead()
{
  if(LookAheadRingEmpty())
    return;
  
  LookAhead* n0;
  LookAhead* n1;
  LookAhead* n2;
  
  float u, v;
  
  // If there are a reasonable number of moves in there (LOOK_AHEAD), or if we are
  // doing single moves with no other move immediately following on, run up and down
  // the moves using the DDA Init() function to reduce the start or the end speed
  // or both to the maximum that can be achieved because of the requirements of
  // the adjacent moves. 
    
  if(addNoMoreMoves || !gCodes->PrintingAFile() || lookAheadRingCount > LOOK_AHEAD)
  { 
    
    // Run up the moves
    
    n1 = lookAheadRingGetPointer;
    n0 = n1->Previous();
    n2 = n1->Next();
    while(n2 != lookAheadRingAddPointer)
    {
      if(!(n1->Processed() & complete))
      {
        if(n1->Processed() & vCosineSet)
        {
          u = n0->V();
          v = n1->V();
          if(lookAheadDDA->Init(n1, u, v) & change)
          {
            n0->SetV(u);
            n1->SetV(v); 
          }
        }
      }
      n0 = n1;
      n1 = n2;
      n2 = n2->Next();
    }
    
    // Now run down
    
    do
    { 
      if(!(n1->Processed() & complete))
      {
        if(n1->Processed() & vCosineSet)
        {
          u = n0->V();
          v = n1->V();
          if(lookAheadDDA->Init(n1, u, v) & change)
          {
            n0->SetV(u);
            n1->SetV(v); 
          }
          n1->SetProcessed(complete);
        }
      }
      n2 = n1;
      n1 = n0;
      n0 = n0->Previous();      
    }while(n0 != lookAheadRingGetPointer);
    n0->SetProcessed(complete);
  }

  // If there are any new unprocessed moves in there, set their end speeds
  // according to the cosine of the angle between them.
  
  if(addNoMoreMoves || !gCodes->PrintingAFile() || lookAheadRingCount > 1)
  {  
    n1 = lookAheadRingGetPointer;
    n0 = n1->Previous();
    n2 = n1->Next();
    while(n2 != lookAheadRingAddPointer)
    {
      if(n1->Processed() == unprocessed)
      {
        float c = fmin(n1->EndPoint()[DRIVES], n2->EndPoint()[DRIVES]);
        c = c*n1->Cosine();
        if(c <= 0)
        {
          int8_t mt = GetMovementType(n0->EndPoint(), n1->EndPoint());

          // Assumes xy overrides z overrides e

          if(mt & xyMove)
            c = platform->InstantDv(Z_AXIS);
          else if (mt & zMove)
            c = platform->InstantDv(X_AXIS);
          else
            c = platform->InstantDv(AXES); // value for first extruder FIXME??
        }
        n1->SetV(c);
        n1->SetProcessed(vCosineSet);
        //n1->SetProcessed(complete);
      } 
      n0 = n1;
      n1 = n2;
      n2 = n2->Next();
    }
    
    // If we are just doing one isolated move, set its end velocity to 0.
    
    if(addNoMoreMoves || !gCodes->PrintingAFile())
    {
      n1->SetV(0);
      n1->SetProcessed(complete);
    }
  }
}

// This is the function that's called by the timer interrupt to step the motors.

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


bool Move::LookAheadRingAdd(float ep[], float vv, bool ce)
{
    if(LookAheadRingFull())
      return false;
    if(!(lookAheadRingAddPointer->Processed() & released))
      platform->Message(HOST_MESSAGE, "Attempt to alter a non-released lookahead ring entry!\n"); // Should never happen...
    lookAheadRingAddPointer->Init(ep, vv, ce);
    lastMove = lookAheadRingAddPointer;
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

void Move::SetIdentityTransform()
{
	aX = 0.0;
	aY = 0.0;
	aC = 0.0;
}

void Move::Transform(float move[])
{
	move[2] = move[2] + aX*move[0] + aY*move[1] + aC;
}

void Move::InverseTransform(float move[])
{
	move[2] = move[2] - (aX*move[0] + aY*move[1] + aC);
}

void Move::SetProbedBedPlane()
{
	float xj, yj, zj;
	float xk, yk, zk;
	float xl, yl, zl;
	float xkj, ykj, zkj;
	float xlj, ylj, zlj;
	float a, b, c, d;   // Implicit plane equation - what we need to do a proper job

	if(!reprap.GetGCodes()->GetProbeCoordinates(0, xj, yj, zj))
		platform->Message(HOST_MESSAGE, "Attempt to set bed plane when probing is incomplete!\n");
	if(!reprap.GetGCodes()->GetProbeCoordinates(1, xk, yk, zk))
			platform->Message(HOST_MESSAGE, "Attempt to set bed plane when probing is incomplete!\n");
	if(!reprap.GetGCodes()->GetProbeCoordinates(2, xl, yl, zl))
			platform->Message(HOST_MESSAGE, "Attempt to set bed plane when probing is incomplete!\n");
	xkj = xk - xj;
	ykj = yk - yj;
	zkj = zk - zj;
	xlj = xl - xj;
	ylj = yl - yj;
	zlj = zl - zj;
	a = ykj*zlj - zkj*ylj;
	b = zkj*xlj - xkj*zlj;
	c = xkj*ylj - ykj*xlj;
	d = -(xk*a + yk*b + zk*c);
	aX = -a/c;
	aY = -b/c;
	aC = -d/c;
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

  if X and/or Y are moving
    Use X acceleration
  else if Z is moving
  	  Use Z acceleration
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

TODO: Worry about having more than eight extruders; X and Y behaving radically differently...

*/

MovementProfile DDA::Init(LookAhead* lookAhead, float& u, float& v)
{
  int8_t drive;
  active = false;
  myLookAheadEntry = lookAhead;
  MovementProfile result = moving;
  totalSteps = -1;
  distance = 0.0; // X+Y+Z
  float eDistance = 0.0;
  float d;
  float* targetPosition = myLookAheadEntry->EndPoint();
  v = lookAhead->V();
  float* positionNow = myLookAheadEntry->Previous()->EndPoint();
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
    myLookAheadEntry->Release();
    return result;
  }
  
  // Set up the DDA
  
  counter[0] = -totalSteps/2;
  for(drive = 1; drive < DRIVES; drive++)
    counter[drive] = counter[0];
  
  // Acceleration and velocity calculations
  
  distance = sqrt(distance);
  
  // Decide the appropriate acceleration and instantDv values
  // timeStep is set here to the distance of the
  // corresponding axis step.  It will be divided
  // by a velocity later. 

  if(delta[X_AXIS] || delta[Y_AXIS]) // X or Y involved?
  {
    acceleration = platform->Acceleration(X_AXIS);
    instantDv = platform->InstantDv(X_AXIS);
    timeStep = 1.0/platform->DriveStepsPerUnit(X_AXIS);  // Slight hack
  } else if (delta[Z_AXIS])// Z involved?
  {
    acceleration = platform->Acceleration(Z_AXIS);
    instantDv = platform->InstantDv(Z_AXIS);
    timeStep = 1.0/platform->DriveStepsPerUnit(Z_AXIS);
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
  
  if(v < instantDv) // Set change here?
    v = instantDv; 
  if(u < instantDv)
    u = instantDv;
  if(targetPosition[DRIVES] < instantDv)
    targetPosition[DRIVES] = instantDv;

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
  
  return result;
}

void DDA::Start(bool noTest)
{
  for(int8_t drive = 0; drive < DRIVES; drive++)
    platform->SetDirection(drive, directions[drive]);
  if(noTest)
    platform->SetInterrupt(timeStep); // seconds
  active = true;  
}

void DDA::Step(bool noTest)
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
          move->HitLowStop(drive, myLookAheadEntry, this);
          active = false;
        }
        if(esh == highHit)
        {
          move->HitHighStop(drive, myLookAheadEntry, this);
          active = false;
        }
      }        
    }
  }
  
  // May have hit a stop, so test active here
  
  if(active) 
  {
    if(axesMoving)
      timeStep = move->stepDistances[axesMoving]/velocity;
    else
      timeStep = move->extruderStepDistances[extrudersMoving]/velocity;
      
    // Simple Euler integration to get velocities.
    // Maybe one day do a Runge-Kutta?
  
    if(stepCount < stopAStep)
      velocity += acceleration*timeStep;
    if(stepCount >= startDStep)
      velocity -= acceleration*timeStep;
    
    // Euler is only approximate.
    
    if(velocity < instantDv)
      velocity = instantDv;
      
    stepCount++;
    active = stepCount < totalSteps;
    
    if(noTest)
      platform->SetInterrupt(timeStep);
  }
  
  if(!active && noTest)
  {
    myLookAheadEntry->Release();
    platform->SetInterrupt(STANDBY_INTERRUPT_RATE);
  }
}

//***************************************************************************************************

LookAhead::LookAhead(Move* m, Platform* p, LookAhead* n)
{
  move = m;
  platform = p;
  next = n;
}

void LookAhead::Init(float ep[], float vv, bool ce)
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
  
  if(a2 <= 0.0 || b2 <= 0.0)
  {
    cosine = 0.5; // Why not?
    return cosine;
  }
 
  cosine = cosine/( (float)sqrt(a2) * (float)sqrt(b2) );
  return cosine;
}





