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


class LookAhead
{  
public:

	friend class Move;
	friend class DDA;

protected:
	LookAhead(Move* m, Platform* p, LookAhead* n);
	void Init(long ep[], float feedRate, float vv, bool ce, int8_t mt);
	LookAhead* Next();
	LookAhead* Previous();
	long* MachineEndPoints();
	float MachineToEndPoint(int8_t drive);
	static float MachineToEndPoint(int8_t drive, long coord);
	static long EndPointToMachine(int8_t drive, float coord);
	int8_t GetMovementType();
	float FeedRate();
	float V();
	void SetV(float vv);
	void SetFeedRate(float f);
	int8_t Processed();
	void SetProcessed(MovementState ms);
	void SetDriveCoordinateAndZeroEndSpeed(float a, int8_t drive);
	bool CheckEndStops();
	void Release();

private:

	Move* move;
	Platform* platform;
	LookAhead* next;
	LookAhead* previous;
	long endPoint[DRIVES+1];  // Should never use the +1, but safety first
	int8_t movementType;
	float Cosine();
    bool checkEndStops;
    float cosine;
    float v;        // The feedrate we can actually do
    float feedRate; // The requested feedrate
    float instantDv;
    volatile int8_t processed;
};


class DDA
{
public:

	friend class Move;
	friend class LookAhead;

protected:
	DDA(Move* m, Platform* p, DDA* n);
	MovementProfile Init(LookAhead* lookAhead, float& u, float& v);
	void Start(bool noTest);
	void Step(bool noTest);
	bool Active();
	DDA* Next();
	float InstantDv();

private:
	MovementProfile AccelerationCalculation(float& u, float& v, MovementProfile result);
	void SetXYAcceleration();
	void SetEAcceleration(float eDistance);
	Move* move;
	Platform* platform;
	DDA* next;
	LookAhead* myLookAheadEntry;
	long counter[DRIVES];
	long delta[DRIVES];
	bool directions[DRIVES];
	long totalSteps;
	long stepCount;
	bool checkEndStops;
    float timeStep;
    float velocity;
    long stopAStep;
    long startDStep;
    float distance;
    float acceleration;
    float instantDv;
    volatile bool active;
};




class Move
{   
  public:
  
    Move(Platform* p, GCodes* g);
    void Init();
    void Spin();
    void Exit();
    bool GetCurrentState(float m[]); // takes account of all the reings and delays
    void LiveCoordinates(float m[]); // Just gives the last point at the end of the last DDA
    void Interrupt();
    void InterruptTime();
    bool AllMovesAreFinished();
    void ResumeMoving();
    void DoLookAhead();
    void HitLowStop(int8_t drive, LookAhead* la, DDA* hitDDA);
    void HitHighStop(int8_t drive, LookAhead* la, DDA* hitDDA);
    void SetPositions(float move[]);
    void SetZProbing(bool probing);
    void SetProbedBedPlane();
    float GetLastProbedZ();
    void SetAxisCompensation(int8_t axis, float tangent);
    void SetIdentityTransform();
    void Transform(float move[]);
    void InverseTransform(float move[]);
    void Diagnostics();
    float ComputeCurrentCoordinate(int8_t drive, LookAhead* la, DDA* runningDDA);
    
  friend class DDA;

  protected:
    float liveCoordinates[DRIVES + 1];
    
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
    bool checkEndStopsOnNextMove;
    float currentFeedrate;
    float nextMove[DRIVES + 1];  // Extra is for feedrate
    float stepDistances[(1<<AXES)]; // Index bits: lsb -> dx, dy, dz <- msb
    float extruderStepDistances[(1<<(DRIVES-AXES))]; // NB - limits us to 5 extruders
    long nextMachineEndPoints[DRIVES+1];
    float aX, aY, aC; // Bed plane explicit equation z' = z + aX*x + aY*y + aC
    bool zPlaneSet;
    float tanXY, tanYZ, tanXZ; // 90 degrees + angle gives angle between axes
    float lastZHit;
    bool zProbing;
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
  //SetProcessed(released);
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

//inline bool Move::ZProbing()
//{
//	return zProbing;
//}

inline void Move::SetZProbing(bool probing)
{
	zProbing = probing;
}

inline float Move::GetLastProbedZ()
{
	return lastZHit;
}

inline void Move::SetAxisCompensation(int8_t axis, float tangent)
{
	switch(axis)
	{
	case X_AXIS:
		tanXY = tangent;
		break;
	case Y_AXIS:
		tanYZ = tangent;
		break;
	case Z_AXIS:
		tanXZ = tangent;
		break;
	default:
		platform->Message(HOST_MESSAGE, "SetAxisCompensation: dud axis.\n");
	}
}

inline void Move::HitLowStop(int8_t drive, LookAhead* la, DDA* hitDDA)
{
	float hitPoint = 0.0;
	if(drive == Z_AXIS)
	{
		if(zProbing)
		{
			lastZHit = ComputeCurrentCoordinate(drive, la, hitDDA);
			la->SetDriveCoordinateAndZeroEndSpeed(lastZHit, drive);
			lastZHit = lastZHit - platform->ZProbeStopHeight();
			return;
		} else
		{
			lastZHit = platform->ZProbeStopHeight(); // Should never be used.
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
