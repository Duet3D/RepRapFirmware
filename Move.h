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
#define LOOK_AHEAD_RING_LENGTH 30
#define LOOK_AHEAD 20    // Must be less than LOOK_AHEAD_RING_LENGTH

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
	friend class Move;
	friend class DDA;

protected:

	LookAhead(Move* m, Platform* p, LookAhead* n);
	void Init(long ep[], float requsestedFeedRate, float minSpeed, 		// Set up this move
			float maxSpeed, float acceleration, bool ce);
	LookAhead* Next();													// Next one in the ring
	LookAhead* Previous();												// Previous one in the ring
	const long* MachineCoordinates() const;								// Endpoints of a move in machine coordinates
	float MachineToEndPoint(int8_t drive) const;						// Convert a move endpoint to real mm coordinates
	static float MachineToEndPoint(int8_t drive, long coord);			// Convert any number to a real coordinate
	static long EndPointToMachine(int8_t drive, float coord);			// Convert real mm to a machine coordinate
	float FeedRate();													// How fast is the set speed for this move
	float MinSpeed();													// What is the slowest that this move can be
	float MaxSpeed();													// What is the fastest this move can be
	float Acceleration();												// What is the acceleration available for this move
	float V();															// The speed at the end of the move
	void SetV(float vv);												// Set the end speed
	void SetFeedRate(float f);											// Set the desired feedrate
	int8_t Processed() const;											// Where we are in the look-ahead prediction sequence
	void SetProcessed(MovementState ms);								// Set where we are the the look ahead processing
	void SetDriveCoordinateAndZeroEndSpeed(float a, int8_t drive);		// Force an end ppoint and st its speed to stopped
	bool CheckEndStops() const;											// Are we checking endstops on this move?
	void Release();														// This move has been processed and executed

private:

	Move* move;						// The main movement control class
	Platform* platform;				// The RepRap machine
	LookAhead* next;				// Next entry in the ring
	LookAhead* previous;			// Previous entry in the ring
	long endPoint[DRIVES+1];  		// Machine coordinates of the endpoint.  Should never use the +1, but safety first
	float Cosine();					// The angle between the previous move and this one
    bool checkEndStops;				// Check endstops for this move
    float cosine;					// Store for the cosine value - the function uses lazy evaluation
    float v;        				// The feedrate we can actually do
    float requestedFeedrate; 		// The requested feedrate
    float minSpeed;					// The slowest that this move may run at
    float maxSpeed;					// The fastest this move may run at
    float acceleration;				// The fastest acceleration allowed
    volatile int8_t processed;		// The stage in the look ahead process that this move is at.
};

/**
 * This implements an integer space machine coordinates Bressenham-style DDA to step the drives.
 * DDAs are stored in a linked list forming a ring buffer.
 */
class DDA
{
	friend class Move;
	friend class LookAhead;

protected:

	DDA(Move* m, Platform* p, DDA* n);
	MovementProfile Init(LookAhead* lookAhead, float& u, float& v); // Set up the DDA.  Also used experimentally in look ahead.
	void Start();													// Start executing the DDA.  I.e. move the move.
	void Step();													// Take one step of the DDA.  Called by timed interrupt.
	bool Active() const;
	DDA* Next();													// Next entry in the ring
	float InstantDv() const;

private:

	MovementProfile AccelerationCalculation(float& u, float& v, 	// Compute acceleration profiles
			MovementProfile result);

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
    float feedRate;
    volatile bool active;					// Is the DDA running?
};

/**
 * This is the master movement class.  It controls all movement in the machine.
 */
class Move
{
    friend class DDA;

  public:
  
    Move(Platform* p, GCodes* g);
    void Init();								// Start me up
    void Spin();								// Called in a tight loop to keep the class going
    void Exit();								// Shut down
    bool GetCurrentUserPosition(float m[]); 	// Return the current position in transformed coords if possible.  Send false otherwise
												// DANGER!!! the above function is mis-named because it has the side-effect of clearing currentFeedrate!!!
    void LiveCoordinates(float m[]) const;		// Gives the last point at the end of the last complete DDA transformed to user coords
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
    void SetFeedrate(float feedRate);			// Sometimes we want to override the feedrate
    void SetLiveCoordinates(float coords[]);	// Force the live coordinates (see above) to be these
    void SetXBedProbePoint(int index, float x);	// Record the X coordinate of a probe point
    void SetYBedProbePoint(int index, float y);	// Record the Y coordinate of a probe point
    void SetZBedProbePoint(int index, float z);	// Record the Z coordinate of a probe point
    float xBedProbePoint(int index) const;		// Get the X coordinate of a probe point
    float yBedProbePoint(int index) const;		// Get the Y coordinate of a probe point
    float zBedProbePoint(int index)const ;		// Get the Z coordinate of a probe point
    int NumberOfProbePoints() const;				// How many points to probe have been set?  0 if incomplete
    int NumberOfXYProbePoints() const;			// How many XY coordinates of probe points have been set (Zs may not have been probed yet)
    bool AllProbeCoordinatesSet(int index) const;	// XY, and Z all set for this one?
    bool XYProbeCoordinatesSet(int index) const;	// Just XY set for this one?
    void SetZProbing(bool probing);				// Set the Z probe live
    void SetProbedBedEquation();				// When we have a full set of probed points, work out the bed's equation
    float SecondDegreeTransformZ(float x, float y) const; // Used for second degree bed equation
    float GetLastProbedZ() const;				// What was the Z when the probe last fired?
    void SetAxisCompensation(int8_t axis, float tangent); // Set an axis-pair compensation angle
    void SetIdentityTransform();				// Cancel the bed equation; does not reset axis angle compensation
    void Transform(float move[]) const;			// Take a position and apply the bed and the axis-angle compensations
    void InverseTransform(float move[]) const;	// Go from a transformed point back to user coordinates
    void Diagnostics();							// Report useful stuff
    float ComputeCurrentCoordinate(int8_t drive,// Turn a DDA value back into a real world coordinate
    		LookAhead* la, DDA* runningDDA);
    void SetStepHypotenuse();					// Set up the hypotenuse lengths for multiple axis steps, like step both X and Y at once
    float Normalise(float v[], int8_t dimensions);  // Normalise a vector to unit length
    void Absolute(float v[], int8_t dimensions);	// Put a vector in the positive hyperquadrant
    float Magnitude(const float v[], int8_t dimensions);  // Return the length of a vector
    void Scale(float v[], float scale,				// Multiply a vector by a scalar
    		int8_t dimensions);
    float VectorBoxIntersection(const float v[],  // Compute the length that a vector would have to have to...
    		const float box[], int8_t dimensions);// ...just touch the surface of a hyperbox.
    
  private:
  
    void BedTransform(float move[]) const;			    // Take a position and apply the bed compensations
    bool GetCurrentMachinePosition(float m[]);			// Get the current position in untransformed coords if possible. Return false otherwise
    													// DANGER!!! the above function is mis-named because it has the side-effect of clearing currentFeedrate!!!
    void InverseBedTransform(float move[]) const;	    // Go from a bed-transformed point back to user coordinates
    void AxisTransform(float move[]) const;			    // Take a position and apply the axis-angle compensations
    void InverseAxisTransform(float move[]) const;	    // Go from an axis transformed point back to user coordinates
    bool DDARingAdd(LookAhead* lookAhead);				// Add a processed look-ahead entry to the DDA ring
    DDA* DDARingGet();									// Get the next DDA ring entry to be run
    bool DDARingEmpty() const;
    bool NoLiveMovement() const;
    bool DDARingFull() const;
    bool GetDDARingLock();								// Lock the ring so only this function may access it
    void ReleaseDDARingLock();							// Release the DDA ring lock
    bool LookAheadRingEmpty() const;					// Anything there?
    bool LookAheadRingFull() const;						// Any more room?
    bool LookAheadRingAdd(long ep[], float requestedFeedRate, 	// Add an entry to the look-ahead ring for processing
    		float minSpeed, float maxSpeed,
    		float acceleration, bool ce);
    void PrintMove(LookAhead* lookAhead);				// For diagnostics
    LookAhead* LookAheadRingGet();						// Get the next entry from the look-ahead ring

    Platform* platform;									// The RepRap machine
    GCodes* gCodes;										// The G Codes processing class
    
    // These implement the DDA ring
    
    DDA* dda;
    DDA* ddaRingAddPointer;
    DDA* ddaRingGetPointer;
    volatile bool ddaRingLocked;
    
    // These implement the look-ahead ring

    LookAhead* lookAheadRingAddPointer;
    LookAhead* lookAheadRingGetPointer;
    LookAhead* lastMove;
    DDA* lookAheadDDA;
    int lookAheadRingCount;

    float lastTime;									// The last time we were called (secs)
    bool addNoMoreMoves;							// If true, allow no more moves to be added to the look-ahead
    bool active;									// Are we live and running?
    float currentFeedrate;							// Err... the current feed rate...
    float liveCoordinates[DRIVES + 1];				// The last endpoint that the machine moved to
    float nextMove[DRIVES + 1];  					// The endpoint of the next move to processExtra entry is for feedrate
    float normalisedDirectionVector[DRIVES];		// Used to hold a unit-length vector in the direction of motion
    float stepDistances[(1<<DRIVES)];				// The length of steps in different numbers of dimensions
    long nextMachineEndPoints[DRIVES+1];			// The next endpoint in machine coordinates (i.e. steps)
    float xBedProbePoints[NUMBER_OF_PROBE_POINTS];	// The X coordinates of the points on the bed at which to probe
    float yBedProbePoints[NUMBER_OF_PROBE_POINTS];	// The X coordinates of the points on the bed at which to probe
    float zBedProbePoints[NUMBER_OF_PROBE_POINTS];	// The X coordinates of the points on the bed at which to probe
    uint8_t probePointSet[NUMBER_OF_PROBE_POINTS];	// Has the XY of this point been set?  Has the Z been probed?
    float aX, aY, aC; 								// Bed plane explicit equation z' = z + aX*x + aY*y + aC
    float tanXY, tanYZ, tanXZ; 						// Axis compensation - 90 degrees + angle gives angle between axes
    float xRectangle, yRectangle;					// The side lengths of the rectangle used for second-degree bed compensation
    float lastZHit;									// The last Z value hit by the probe
    bool zProbing;									// Are we bed probing as well as moving?
    bool secondDegreeCompensation;					// Are we using second degree bed compensation.  If not, linear
    float longWait;									// A long time for things that need to be done occasionally
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


inline float LookAhead::MachineToEndPoint(int8_t drive) const
{
	if(drive >= DRIVES)
	{
		platform->Message(HOST_MESSAGE, "MachineToEndPoint() called for feedrate!\n");
		return 0.0;
	}
	return ((float)(endPoint[drive]))/platform->DriveStepsPerUnit(drive);
}

inline float LookAhead::FeedRate()
{
	return requestedFeedrate;
}

inline float LookAhead::MinSpeed()
{
	return minSpeed;
}

inline float LookAhead::MaxSpeed()
{
	return maxSpeed;
}

inline float LookAhead::Acceleration()
{
	return acceleration;
}

inline void LookAhead::SetV(float vv)
{
  v = vv;
}

inline float LookAhead::V()
{
  return v;
}

inline void LookAhead::SetFeedRate(float f)
{
	requestedFeedrate = f;
	v = f;
}

inline int8_t LookAhead::Processed() const
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

inline bool LookAhead::CheckEndStops() const
{
  return checkEndStops;
}

inline void LookAhead::SetDriveCoordinateAndZeroEndSpeed(float a, int8_t drive)
{
  endPoint[drive] = EndPointToMachine(drive, a);
  cosine = 2.0;
  v = platform->InstantDv(platform->SlowestDrive());
}

inline const long* LookAhead::MachineCoordinates() const
{
	return endPoint;
}

//inline int8_t LookAhead::GetMovementType()
//{
//	return movementType;
//}

//******************************************************************************************************

inline bool DDA::Active() const
{
  return active;
}

inline DDA* DDA::Next()
{
  return next;
}

inline float DDA::InstantDv() const
{
  return instantDv;
}


//***************************************************************************************

inline bool Move::DDARingEmpty() const
{
  return ddaRingGetPointer == ddaRingAddPointer;
}

inline bool Move::NoLiveMovement() const
{
  if(dda != NULL)
    return false;
  return DDARingEmpty();
}

// Leave a gap of 2 as the last Get result may still be being processed

inline bool Move::DDARingFull() const
{
  return ddaRingAddPointer->Next()->Next() == ddaRingGetPointer;
}

inline bool Move::LookAheadRingEmpty() const
{
  return lookAheadRingCount == 0;
}

// Leave a gap of 2 as the last Get result may still be being processed

inline bool Move::LookAheadRingFull() const
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

inline void Move::LiveCoordinates(float m[]) const
{
	for(int8_t drive = 0; drive <= DRIVES; drive++)
	{
		m[drive] = liveCoordinates[drive];
	}
	InverseTransform(m);
}


// These are the actual numbers that we want to be the coordinates, so
// don't transform them.

inline void Move::SetLiveCoordinates(float coords[])
{
	for(int8_t drive = 0; drive <= DRIVES; drive++)
	{
		liveCoordinates[drive] = coords[drive];
	}
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

inline float Move::xBedProbePoint(int index) const
{
	return xBedProbePoints[index];
}

inline float Move::yBedProbePoint(int index) const
{
	return yBedProbePoints[index];
}

inline float Move::zBedProbePoint(int index) const
{
	return zBedProbePoints[index];
}

inline void Move::SetZProbing(bool probing)
{
	zProbing = probing;
}

inline float Move::GetLastProbedZ() const
{
	return lastZHit;
}

inline bool Move::AllProbeCoordinatesSet(int index) const
{
	return probePointSet[index] == (xSet | ySet | zSet);
}

inline bool Move::XYProbeCoordinatesSet(int index) const
{
	return (probePointSet[index]  & xSet) &&  (probePointSet[index]  & ySet);
}

inline int Move::NumberOfProbePoints() const
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

inline int Move::NumberOfXYProbePoints() const
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
inline float Move::SecondDegreeTransformZ(float x, float y) const
{
	x = (x - xBedProbePoints[0])*xRectangle;
	y = (y - yBedProbePoints[0])*yRectangle;
	return (1.0 - x)*(1.0 - y)*zBedProbePoints[0] + x*(1.0 - y)*zBedProbePoints[3] + (1.0 - x)*y*zBedProbePoints[1] + x*y*zBedProbePoints[2];
}



inline void Move::HitLowStop(int8_t drive, LookAhead* la, DDA* hitDDA)
{
	float hitPoint = platform->AxisMinimum(drive);
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
				gCodes->SetAxisIsHomed(drive);
				lastZHit = hitPoint;
			}
			return;
		} else
		{
			// Executing G30, so set the current Z height to the value at which the end stop is triggered
			// Transform it first so that the height is correct in user coordinates
			float xyzPoint[DRIVES + 1];
			LiveCoordinates(xyzPoint);
			xyzPoint[Z_AXIS] = lastZHit = platform->ZProbeStopHeight();
			Transform(xyzPoint);
			hitPoint = xyzPoint[Z_AXIS];
		}
	}
	la->SetDriveCoordinateAndZeroEndSpeed(hitPoint, drive);
	gCodes->SetAxisIsHomed(drive);
}

inline void Move::HitHighStop(int8_t drive, LookAhead* la, DDA* hitDDA)
{
  la->SetDriveCoordinateAndZeroEndSpeed(platform->AxisMaximum(drive), drive);
  gCodes->SetAxisIsHomed(drive);
}

inline float Move::ComputeCurrentCoordinate(int8_t drive, LookAhead* la, DDA* runningDDA)
{
	float previous = la->Previous()->MachineToEndPoint(drive);
	if(runningDDA->totalSteps <= 0)
		return previous;
	return previous + (la->MachineToEndPoint(drive) - previous)*(float)runningDDA->stepCount/(float)runningDDA->totalSteps;
}



#endif
