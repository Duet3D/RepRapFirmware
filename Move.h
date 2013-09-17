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
#define SMALL_Z_MOVE 0.03 // If a Z movement is less than this fraction of an XY move, the movement is classed as XY

enum MovementProfile
{
  moving = 0,  // Ordinary trapezoidal-velocity-profile movement
  noFlat = 1,  // Triangular profile movement
  change = 2   // To make this movement, the initial and/or final velocities must change
};

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
    LookAhead(Move* m, Platform* p, LookAhead* n);
    void Init(float ep[], float vv, bool ce);
    LookAhead* Next();
    LookAhead* Previous();
    float* EndPoint();
    float V();
    void SetV(float vv);
    int8_t Processed();
    void SetProcessed(MovementState ms);
    void SetDriveZeroEndSpeed(float a, int8_t drive);
    bool CheckEndStops();
    void Release();
    
  friend class Move;
    
  private:
    Move* move;
    Platform* platform;
    LookAhead* next;
    LookAhead* previous;
    float endPoint[DRIVES+1];
    float Cosine();
    bool checkEndStops;
    float cosine;
    float v;
    float instantDv;
    volatile int8_t processed;
};


class DDA
{
  public: 
    DDA(Move* m, Platform* p, DDA* n);
    MovementProfile Init(LookAhead* lookAhead, float& u, float& v);
    void Start(bool noTest);
    void Step(bool noTest);
    bool Active();
    DDA* Next();
    float InstantDv();
    
  friend class Move;

  private:
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
    float dCross;
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
    bool GetCurrentState(float m[]);
    void Interrupt();
    void InterruptTime();
    bool AllMovesAreFinished();
    void ResumeMoving();
    void DoLookAhead();
    void HitLowStop(int8_t drive, LookAhead* la, DDA* hitDDA);
    void HitHighStop(int8_t drive, LookAhead* la, DDA* hitDDA);
    void SetZProbing(bool probing);
    void SetProbedBedPlane();
    float GetLastProbedZ();
    void SetIdentityTransform();
    void Transform(float move[]);
    void InverseTransform(float move[]);
    void Diagnostics();
    float ComputeCurrentCoordinate(int8_t drive, LookAhead* la, DDA* runningDDA);
    
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
    bool LookAheadRingAdd(float ep[], float vv, bool ce);
    LookAhead* LookAheadRingGet();
    int8_t GetMovementType(float sp[], float ep[]);

    
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
    float aX, aY, aC;
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

inline float* LookAhead::EndPoint() 
{
  return endPoint;
}


inline float LookAhead::V() 
{
  return v;
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
  SetProcessed(released);
}

inline bool LookAhead::CheckEndStops()
{
  return checkEndStops;
}

inline void LookAhead::SetDriveZeroEndSpeed(float a, int8_t drive)
{
  endPoint[drive] = a;
  cosine = 2.0;
  v = 0.0; 
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

inline void Move::SetZProbing(bool probing)
{
	zProbing = probing;
}

inline float Move::GetLastProbedZ()
{
	return lastZHit;
}

inline void Move::HitLowStop(int8_t drive, LookAhead* la, DDA* hitDDA)
{
  float hitPoint = 0.0;
  if(drive == Z_AXIS && zProbing)
  {
	  lastZHit = ComputeCurrentCoordinate(drive, la, hitDDA);
	  hitPoint = lastZHit;
  }
  la->SetDriveZeroEndSpeed(hitPoint, drive);
}

inline void Move::HitHighStop(int8_t drive, LookAhead* la, DDA* hitDDA)
{
  la->SetDriveZeroEndSpeed(platform->AxisLength(drive), drive);
}

inline float Move::ComputeCurrentCoordinate(int8_t drive, LookAhead* la, DDA* runningDDA)
{
	if(runningDDA->totalSteps <= 0)
		return 0.0;
	return la->Previous()->endPoint[drive] + (la->endPoint[drive] - la->Previous()->endPoint[drive])*(float)runningDDA->stepCount/(float)runningDDA->totalSteps;
}


#endif
