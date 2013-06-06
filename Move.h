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
  change = 2   // To make this movement, the initial and final velocities must change
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
    void Init(float ep[], float vv, boolean ce);
    LookAhead* Next();
    LookAhead* Previous();
    float* EndPoint();
    float V();
    void SetV(float vv);
    int8_t Processed();
    void SetProcessed(MovementState ms);
    boolean CheckEndStops();
    void Release();
    
  friend class Move;
    
  private:
    Move* move;
    Platform* platform;
    LookAhead* next;
    LookAhead* previous;
    float endPoint[DRIVES+1];
    float Cosine();
    boolean checkEndStops;
    float cosine;
    float v;
    int8_t processed;
};


class DDA
{
  public: 
    DDA(Move* m, Platform* p, DDA* n);
    MovementProfile Init(LookAhead* lookAhead, float& u, float& v);
    void Start(boolean noTest);
    void Step(boolean noTest);
    boolean Active();
    DDA* Next();
    
  friend class Move;

  private:
    Move* move;
    Platform* platform;
    DDA* next;
    long counter[DRIVES];
    long delta[DRIVES];
    boolean directions[DRIVES];
    long totalSteps;
    long stepCount;
    boolean checkEndStops;
    float timeStep;
    float velocity;
    long stopAStep;
    long startDStep;
    float distance;
    float dCross;
    float acceleration;
    float instantDv;
    volatile boolean active;
};




class Move
{   
  public:
  
    Move(Platform* p, GCodes* g);
    void Init();
    void Spin();
    void Exit();
    boolean GetCurrentState(float m[]);
    void Interrupt();
    void InterruptTime();
    boolean AllMovesAreFinished();
    void ResumeMoving();
    void DoLookAhead();
    void HitLowStop(int8_t drive);
    void HitHighStop(int8_t drive);
    
  friend class DDA;
    
  private:
  
    boolean DDARingAdd(LookAhead* lookAhead);
    DDA* DDARingGet();
    boolean DDARingEmpty();
    boolean DDARingFull();
    boolean GetDDARingLock();
    void ReleaseDDARingLock();
    boolean LookAheadRingEmpty();
    boolean LookAheadRingFull();
    boolean LookAheadRingAdd(float ep[], float vv, boolean ce);
    LookAhead* LookAheadRingGet();
    int8_t GetMovementType(float sp[], float ep[]);

    
    Platform* platform;
    GCodes* gCodes;
    
    DDA* dda;
    DDA* ddaRingAddPointer;
    DDA* ddaRingGetPointer;
    volatile boolean ddaRingLocked;
    
    LookAhead* lookAheadRingAddPointer;
    LookAhead* lookAheadRingGetPointer;
    LookAhead* larWaiting;
    DDA* lookAheadDDA;
    int lookAheadRingCount;

    unsigned long lastTime;
    boolean addNoMoreMoves;
    boolean active;
    boolean moveWaiting;
    boolean checkEndStopsOnNextMove;
    float currentFeedrate;
    float currentPosition[AXES]; // Note - drives above AXES are always relative moves
    float nextMove[DRIVES + 1];  // Extra is for feedrate
    float stepDistances[(1<<AXES)]; // Index bits: lsb -> dx, dy, dz <- msb
    float extruderStepDistances[(1<<(DRIVES-AXES))]; // NB - limits us to 5 extruders
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
  if(ms == 0)
    processed = 0;
  else
    processed |= ms;
}

inline void LookAhead::Release()
{
  processed = released;
}

inline boolean LookAhead::CheckEndStops() 
{
  return checkEndStops;
}

//******************************************************************************************************

inline boolean DDA::Active()
{
  return active;
}

inline DDA* DDA::Next()
{
  return next;
}


//***************************************************************************************

inline boolean Move::DDARingEmpty()
{
  return ddaRingGetPointer == ddaRingAddPointer;
}

// Leave a gap of 2 as the last Get result may still be being processed

inline boolean Move::DDARingFull()
{
  return ddaRingAddPointer->Next()->Next() == ddaRingGetPointer;
}

inline boolean Move::LookAheadRingEmpty()
{
  return lookAheadRingCount == 0;
}

// Leave a gap of 2 as the last Get result may still be being processed

inline boolean Move::LookAheadRingFull()
{
  return lookAheadRingAddPointer->Next()->Next() == lookAheadRingGetPointer;
}

inline boolean Move::GetDDARingLock()
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

inline boolean Move::AllMovesAreFinished()
{
  addNoMoreMoves = true;
  return LookAheadRingEmpty() && DDARingEmpty();
}

inline void Move::ResumeMoving()
{
  addNoMoreMoves = false;
}

inline void Move::HitLowStop(int8_t drive)
{
  currentPosition[drive] = 0.0;
}

inline void Move::HitHighStop(int8_t drive)
{
  currentPosition[drive] = platform->AxisLength(drive);
}


#endif
