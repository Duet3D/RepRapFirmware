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

#ifndef MOVE_H
#define MOVE_H

#define DDA_RING_LENGTH 5
#define LOOK_AHEAD_RING_LENGTH 20
#define LOOK_AHEAD 7

enum MovementProfile
{
  moving = 0,  // Ordinary trapezoidal-velocity-profile movement
  noFlat = 1,  // Triangular profile movement
  change = 2   // To make this movement, the initial and/or final velocities must change
};

// The possible states of a movement in the look-ahead ring as the look-ahead is
// being done.

enum MovementState
{
  unprocessed = 0,
  vCosineSet = 1,
  upPass = 2,
  complete = 4,
  released = 8
};

enum MovementType
{
  noMove = 0,
  xyMove = 1,
  zMove = 2,
  eMove = 4 
};

enum PointCoordinateSet
{
	unset = 0,
	xSet = 1,
	ySet = 2,
	zSet = 4
};

/**
 * This class implements a look-ahead buffer for moves.  It allows colinear
 * moves not to decelerate between them, sets velocities at ends and beginnings
 * for angled moves, and so on.  Entries are joined in a doubly-linked list
 * to form a ring buffer.
 */
class LookAhead
{  
public:

	friend class Move;
	friend class DDA;

protected:
	LookAhead(Move* m, Platform* p, LookAhead* n);
	void Init(long ep[], float feedRate, float vv, bool ce, int8_t mt); // Set up this move
	LookAhead* Next();													// Next one in the ring
	LookAhead* Previous();												// Previous one in the ring
	long* MachineEndPoints();											// Endpoints of a move in machine coordinates
	float MachineToEndPoint(int8_t drive);								// Convert a move endpoint to real mm coordinates
	static float MachineToEndPoint(int8_t drive, long coord);			// Convert any number to a real coordinate
	static long EndPointToMachine(int8_t drive, float coord);			// Convert real mm to a machine coordinate
	int8_t GetMovementType();											// What sort of move is this?
	float FeedRate();													// How fast is the maximum speed for this move
	float V();															// The speed at the end of the move
	void SetV(float vv);												// Set the end speed
	void SetFeedRate(float f);											// Set the desired feedrate
	int8_t Processed();													// Where we are in the look-ahead prediction sequence
	void SetProcessed(MovementState ms);								// Set where we are the the look ahead processing
	void SetDriveCoordinateAndZeroEndSpeed(float a, int8_t drive);		// Force an end ppoint and st its speed to stopped
	bool CheckEndStops();												// Are we checking endstops on this move?
	void Release();														// This move has been processed and executed

private:

	Move* move;						// The main movement control class
	Platform* platform;				// The RepRap machine
	LookAhead* next;				// Next entry in the ring
	LookAhead* previous;			// Previous entry in the ring
	long endPoint[DRIVES+1];  		// Machine coordinates of the endpoint.  Should never use the +1, but safety first
	int8_t movementType;			// XY move, Z move, extruder only etc
	float Cosine();					// The angle between the previous move and this one
    bool checkEndStops;				// Check endstops for this move
    float cosine;					// Store for the cosine value - the function uses lazy evaluation
    float v;        				// The feedrate we can actually do
    float feedRate; 				// The requested feedrate
    float instantDv;				// The slowest speed we can move at. > 0
    volatile int8_t processed;		// The stage in the look ahead process that this move is at.
};

/**
 * This implements an integer space machine coordinates Bressenham-style DDA to step the drives.
 * DDAs are stored in a linked list forming a ring buffer.
 */
class DDA
{
public:

	friend class Move;
	friend class LookAhead;

protected:
	DDA(Move* m, Platform* p, DDA* n);
	MovementProfile Init(LookAhead* lookAhead, float& u, float& v); // Set up the DDA.  Also used experimentally in look ahead.
	void Start(bool noTest);										// Start executing the DDA.  I.e. move the move.
	void Step();													// Take one step of the DDA.  Called by timed interrupt.
	bool Active();													// Is the DDA running?
	DDA* Next();													// Next entry in the ring
	float InstantDv();												// The lowest speed that may be used

private:
	MovementProfile AccelerationCalculation(float& u, float& v, 	// Compute acceleration profiles
			MovementProfile result);
	void SetXYAcceleration();										// Compute an XY acceleration
	void SetEAcceleration(float eDistance);							// Compute an extruder acceleration

	Move* move;								// The main movement control class
	Platform* platform;						// The RepRap machine
	DDA* next;								// The next one in the ring
	LookAhead* myLookAheadEntry;			// The look-ahead entry corresponding to this DDA
	long counter[DRIVES];					// Step counters
	long delta[DRIVES];						// How far to move each drive
	bool directions[DRIVES];				// Forwards or backwards?
	long totalSteps;						// Total number of steps for this move
	long stepCount;							// How many steps we have already taken
	bool checkEndStops;						// Are we checking endstops?
    float timeStep;							// The current timestep (seconds)
    float velocity;							// The current velocity
    long stopAStep;							// The stepcount at which we stop accelerating
    long startDStep;						// The stepcount at which we start decelerating
    float distance;							// How long is the move in real distance
    float acceleration;						// The acceleration to use
    float instantDv;						// The lowest possible velocity
    volatile bool active;					// Is the DDA running?
};


/**
 * This is the master movement class.  It controls all movement in the machine.
 */

class Move
{   
  public:
  
    Move(Platform* p, GCodes* g);
    void Init();								// Start me up
    void Spin();								// Called in a tight loop to keep the class going
    void Exit();								// Shut down
    bool GetCurrentState(float m[]); 			// Return the current position if possible.  Send false otherwise
    void LiveCoordinates(float m[]); 			// Gives the last point at the end of the last complete DDA
    void Interrupt();							// The hardware's (i.e. platform's)  interrupt should call this.
    void InterruptTime();						// Test function - not used
    bool AllMovesAreFinished();					// Is the look-ahead ring empty?  Stops more moves being added as well.
    void ResumeMoving();						// Allow moves to be added after a call to AllMovesAreFinished()
    void DoLookAhead();							// Run the look-ahead procedure
    void HitLowStop(int8_t drive,				// What to do when a low endstop is hit
    		LookAhead* la, DDA* hitDDA);
    void HitHighStop(int8_t drive, 				// What to do when a high endstop is hit
    		LookAhead* la, DDA* hitDDA);
    void SetPositions(float move[]);			// Force the coordinates to be these
    void SetLiveCoordinates(float coords[]);	// Force the live coordinates (see above) to be these
    void SetXBedProbePoint(int index, float x);	// Record the X coordinate of a probe point
    void SetYBedProbePoint(int index, float y);	// Record the Y coordinate of a probe point
    void SetZBedProbePoint(int index, float z);	// Record the Z coordinate of a probe point
    float xBedProbePoint(int index);			// Get the X coordinate of a probe point
    float yBedProbePoint(int index);			// Get the Y coordinate of a probe point
    float zBedProbePoint(int index);			// Get the Z coordinate of a probe point
    int NumberOfProbePoints();					// How many points to probe have been set?  0 if incomplete
    int NumberOfXYProbePoints();				// How many XY coordinates of probe points have been set (Zs may not have been probed yet)
    bool AllProbeCoordinatesSet(int index);		// XY, and Z all set for this one?
    bool XYProbeCoordinatesSet(int index);		// Just XY set for this one?
    void SetZProbing(bool probing);				// Set the Z probe live
    void SetProbedBedEquation();				// When we have a full set of probed points, work out the bed's equation
    float SecondDegreeTransformZ(float x, float y); // Used for second degree bed equation
    float GetLastProbedZ();						// What was the Z when the probe last fired?
    void SetAxisCompensation(int8_t axis, float tangent); // Set an axis-pair compensation angle
    void SetIdentityTransform();				// Cancel the bed equation; does not reset axis angle compensation
    void Transform(float move[]);				// Take a position and apply the bed and the axis-angle compensations
    void InverseTransform(float move[]);		// Go from a transformed point back to user coordinates
    void Diagnostics();							// Report useful stuff
    float ComputeCurrentCoordinate(int8_t drive,// Turn a DDA value back into a real world coordinate
    		LookAhead* la, DDA* runningDDA);
    void SetStepHypotenuse();					// Set up the hypotenuse lengths for multiple axis steps, like step both X and Y
    

    friend class DDA;
    
  private:
  
    bool DDARingAdd(LookAhead* lookAhead);
    DDA* DDARingGet();
    bool DDARingEmpty();
    bool NoLiveMovement();
    bool DDARingFull();
    bool GetDDARingLock();
    void ReleaseDDARingLock();
    bool LookAheadRingEmpty();
    bool LookAheadRingFull();
    bool LookAheadRingAdd(long ep[], float feedRate, float vv, bool ce, int8_t movementType);
    LookAhead* LookAheadRingGet();
    int8_t GetMovementType(long sp[], long ep[]);

    float liveCoordinates[DRIVES + 1];
    
    Platform* platform;
    GCodes* gCodes;
    
    DDA* dda;
    DDA* ddaRingAddPointer;
    DDA* ddaRingGetPointer;
    volatile bool ddaRingLocked;
    
    LookAhead* lookAheadRingAddPointer;
    LookAhead* lookAheadRingGetPointer;
    LookAhead* lastMove;
    DDA* lookAheadDDA;
    int lookAheadRingCount;

    float lastTime;
    bool addNoMoreMoves;
    bool active;
    float currentFeedrate;
    float nextMove[DRIVES + 1];  // Extra is for feedrate
    float stepDistances[(1<<AXES)]; // Index bits: lsb -> dx, dy, dz <- msb
    float extruderStepDistances[(1<<(DRIVES-AXES))]; // NB - limits us to 5 extruders
    long nextMachineEndPoints[DRIVES+1];
    float xBedProbePoints[NUMBER_OF_PROBE_POINTS];
    float yBedProbePoints[NUMBER_OF_PROBE_POINTS];
    float zBedProbePoints[NUMBER_OF_PROBE_POINTS];
    uint8_t probePointSet[NUMBER_OF_PROBE_POINTS];
    float aX, aY, aC; // Bed plane explicit equation z' = z + aX*x + aY*y + aC
    float tanXY, tanYZ, tanXZ; // 90 degrees + angle gives angle between axes
    float xRectangle, yRectangle;
    float lastZHit;
    bool zProbing;
    bool secondDegreeCompensation;
    float longWait;
};

//********************************************************************************************************

inline LookAhead* LookAhead::Next()
{
  return next;
}

inline LookAhead* LookAhead::Previous()
{
  return previous;
}


inline void LookAhead::SetV(float vv)
{
  v = vv;
}

inline float LookAhead::V() 
{
  return v;
}

inline float LookAhead::MachineToEndPoint(int8_t drive)
{
	if(drive >= DRIVES)
		platform->Message(HOST_MESSAGE, "MachineToEndPoint() called for feedrate!\n");
	return ((float)(endPoint[drive]))/platform->DriveStepsPerUnit(drive);
}


inline float LookAhead::FeedRate()
{
	return feedRate;
}

inline void LookAhead::SetFeedRate(float f)
{
	feedRate = f;
}

inline int8_t LookAhead::Processed() 
{
  return processed;
}

inline void LookAhead::SetProcessed(MovementState ms)
{
  if(ms == unprocessed)
    processed = unprocessed;
  else
    processed |= ms;
}

inline void LookAhead::Release()
{
	 processed = released;
}

inline bool LookAhead::CheckEndStops()
{
  return checkEndStops;
}

inline void LookAhead::SetDriveCoordinateAndZeroEndSpeed(float a, int8_t drive)
{
  endPoint[drive] = EndPointToMachine(drive, a);
  cosine = 2.0;
  v = 0.0; 
}

inline long* LookAhead::MachineEndPoints()
{
	return endPoint;
}

inline int8_t LookAhead::GetMovementType()
{
	return movementType;
}

//******************************************************************************************************

inline bool DDA::Active()
{
  return active;
}

inline DDA* DDA::Next()
{
  return next;
}

inline float DDA::InstantDv()
{
  return instantDv;
}


//***************************************************************************************

inline bool Move::DDARingEmpty()
{
  return ddaRingGetPointer == ddaRingAddPointer;
}

inline bool Move::NoLiveMovement()
{
  if(dda != NULL)
    return false;
  return DDARingEmpty();
}

// Leave a gap of 2 as the last Get result may still be being processed

inline bool Move::DDARingFull()
{
  return ddaRingAddPointer->Next()->Next() == ddaRingGetPointer;
}

inline bool Move::LookAheadRingEmpty()
{
  return lookAheadRingCount == 0;
}

// Leave a gap of 2 as the last Get result may still be being processed

inline bool Move::LookAheadRingFull()
{
  if(!(lookAheadRingAddPointer->Processed() & released))
    return true;
  return lookAheadRingAddPointer->Next()->Next() == lookAheadRingGetPointer;  // probably not needed; just return the bool in the if above
}

inline bool Move::GetDDARingLock()
{
  if(ddaRingLocked)
    return false;
  ddaRingLocked = true;
  return true;
}

inline void Move::ReleaseDDARingLock()
{
  ddaRingLocked = false;
}

inline void Move::LiveCoordinates(float m[])
{
	for(int8_t drive = 0; drive <= DRIVES; drive++)
		m[drive] = liveCoordinates[drive];
	InverseTransform(m);
}


// These are the actual numbers that we want to be the coordinates, so
// don't transform them.

inline void Move::SetLiveCoordinates(float coords[])
{
	for(int8_t drive = 0; drive <= DRIVES; drive++)
		liveCoordinates[drive] = coords[drive];
}

// To wait until all the current moves in the buffers are
// complete, call this function repeatedly and wait for it to
// return true.  Then do whatever you wanted to do after all
// current moves have finished.  THEN CALL THE ResumeMoving() FUNCTION
// OTHERWISE NOTHING MORE WILL EVER HAPPEN.

inline bool Move::AllMovesAreFinished()
{
  addNoMoreMoves = true;
  return LookAheadRingEmpty() && NoLiveMovement();
}

inline void Move::ResumeMoving()
{
  addNoMoreMoves = false;
}

inline void Move::SetXBedProbePoint(int index, float x)
{
	if(index < 0 || index >= NUMBER_OF_PROBE_POINTS)
	{
		platform->Message(HOST_MESSAGE, "Z probe point  X index out of range.\n");
		return;
	}
	xBedProbePoints[index] = x;
	probePointSet[index] |= xSet;
}

inline void Move::SetYBedProbePoint(int index, float y)
{
	if(index < 0 || index >= NUMBER_OF_PROBE_POINTS)
	{
		platform->Message(HOST_MESSAGE, "Z probe point Y index out of range.\n");
		return;
	}
	yBedProbePoints[index] = y;
	probePointSet[index] |= ySet;
}

inline void Move::SetZBedProbePoint(int index, float z)
{
	if(index < 0 || index >= NUMBER_OF_PROBE_POINTS)
	{
		platform->Message(HOST_MESSAGE, "Z probe point Z index out of range.\n");
		return;
	}
	zBedProbePoints[index] = z;
	probePointSet[index] |= zSet;
}

inline float Move::xBedProbePoint(int index)
{
	return xBedProbePoints[index];
}

inline float Move::yBedProbePoint(int index)
{
	return yBedProbePoints[index];
}

inline float Move::zBedProbePoint(int index)
{
	return zBedProbePoints[index];
}

inline void Move::SetZProbing(bool probing)
{
	zProbing = probing;
}

inline float Move::GetLastProbedZ()
{
	return lastZHit;
}

inline bool Move::AllProbeCoordinatesSet(int index)
{
	return probePointSet[index] == (xSet | ySet | zSet);
}

inline bool Move::XYProbeCoordinatesSet(int index)
{
	return (probePointSet[index]  & xSet) &&  (probePointSet[index]  & ySet);
}

inline int Move::NumberOfProbePoints()
{
	if(AllProbeCoordinatesSet(0) && AllProbeCoordinatesSet(1) && AllProbeCoordinatesSet(2))
	{
		if(AllProbeCoordinatesSet(3))
			return 4;
		else
			return 3;
	}
	return 0;
}

inline int Move::NumberOfXYProbePoints()
{
	if(XYProbeCoordinatesSet(0) && XYProbeCoordinatesSet(1) && XYProbeCoordinatesSet(2))
	{
		if(XYProbeCoordinatesSet(3))
			return 4;
		else
			return 3;
	}
	return 0;
}

/*
 * Transform to a ruled-surface quadratic.  The corner points for interpolation are indexed:
 *
 *   ^  [1]      [2]
 *   |
 *   Y
 *   |
 *   |  [0]      [3]
 *      -----X---->
 *
 *   The values of x and y are transformed to put them in the interval [0, 1].
 */
inline float Move::SecondDegreeTransformZ(float x, float y)
{
	x = (x - xBedProbePoints[0])*xRectangle;
	y = (y - yBedProbePoints[0])*yRectangle;
	return (1.0 - x)*(1.0 - y)*zBedProbePoints[0] + x*(1.0 - y)*zBedProbePoints[3] + (1.0 - x)*y*zBedProbePoints[1] + x*y*zBedProbePoints[2];
}



inline void Move::HitLowStop(int8_t drive, LookAhead* la, DDA* hitDDA)
{
	float hitPoint = 0.0;
	if(drive == Z_AXIS)
	{
		if(zProbing)
		{
			// Executing G32, so record the Z position at which we hit the end stop
			if (gCodes->GetAxisIsHomed(drive))
			{
				// Z-axis has already been homed, so just record the height of the bed at this point
				lastZHit = ComputeCurrentCoordinate(drive, la, hitDDA);
				la->SetDriveCoordinateAndZeroEndSpeed(lastZHit, drive);
				lastZHit = lastZHit - platform->ZProbeStopHeight();
			}
			else
			{
				// Z axis has not yet been homed, so treat this probe as a homing command
				la->SetDriveCoordinateAndZeroEndSpeed(platform->ZProbeStopHeight(), drive);
				lastZHit = 0.0;
			}
			return;
		} else
		{
			// Executing G30, so set the current Z height to the value at which the end stop is triggered
			lastZHit = platform->ZProbeStopHeight();
			hitPoint = lastZHit;
		}
	}
	la->SetDriveCoordinateAndZeroEndSpeed(hitPoint, drive);
}

inline void Move::HitHighStop(int8_t drive, LookAhead* la, DDA* hitDDA)
{
  la->SetDriveCoordinateAndZeroEndSpeed(platform->AxisLength(drive), drive);
}

inline float Move::ComputeCurrentCoordinate(int8_t drive, LookAhead* la, DDA* runningDDA)
{
	float previous = la->Previous()->MachineToEndPoint(drive);
	if(runningDDA->totalSteps <= 0)
		return previous;
	return previous + (la->MachineToEndPoint(drive) - previous)*(float)runningDDA->stepCount/(float)runningDDA->totalSteps;
}



#endif
