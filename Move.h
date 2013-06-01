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

enum MovementProfile
{
  nothing = 0, // No movement (i.e. end-position == start).
  moving = 1,  // Ordinary trapezoidal-velocity-profile movement
  noFlat = 2,  // Triangular profile movement
  change = 3   // To make this movement, the initial and final velocities must change
};


class DDA
{
  public: 
    DDA(Move* m, Platform* p, DDA* n);
    MovementProfile Init(float currentPosition[], float targetPosition[], float& u, float& v);
    void Start(boolean noTest);
    void Step(boolean noTest);
    boolean Active();
    DDA* Next();
    
  friend class Move;

  private:
    Move* move;
    Platform* platform;
    DDA* next;
    long counter[DRIVES+1];
    long delta[DRIVES+1];
    boolean directions[DRIVES+1];
    long totalSteps;
    long stepCount;
    float timeStep;
    float velocity;
    long stopAStep;
    long startDStep;
    float distance;
    float dCross;
    float acceleration;
    float jerk;
    volatile boolean active;
};

class LookAhead
{  
  public:
    LookAhead(Move* m, Platform* p, LookAhead* n);
    LookAhead* Next();
    LookAhead* Previous();
    void Init(float m[], float uu, float vv);
    void SetUV(float uu, float vv);
    float* Movement();
    float U();
    float V();
    boolean Processed();
    void SetProcessed();
    
  friend class Move;
    
  private:
    Move* move;
    Platform* platform;
    LookAhead* next;
    LookAhead* previous;
    float Cosine(LookAhead* a);
    float movement[DRIVES+1]; // Last is feedrate
    float u, v;
    boolean processed;
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
    
  friend class DDA;
    
  private:
  
    boolean DDARingAdd(float currentPosition[], float targetPosition[], float& u, float& v);
    DDA* DDARingGet();
    boolean DDARingEmpty();
    boolean DDARingFull();
    boolean GetDDARingLock();
    void ReleaseDDARingLock();
    boolean LookAheadRingEmpty();
    boolean LookAheadRingFull();
    boolean LookAheadRingAdd(float m[], float uu, float vv);
    LookAhead* LookAheadRingGet();
    
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
    float currentFeedrate;
    float currentPosition[AXES]; // Note - drives above AXES are always relative moves
    float nextMove[DRIVES + 1];  // Extra is for feedrate
    float stepDistances[(1<<AXES)]; // Index bits: lsb -> dx, dy, dz <- msb
    float extruderStepDistances[(1<<(DRIVES-AXES))]; // NB - limits us to 5 extruders
};

//********************************************************************************************************

inline boolean DDA::Active()
{
  return active;
}

inline DDA* DDA::Next()
{
  return next;
}

inline LookAhead* LookAhead::Next()
{
  return next;
}

inline LookAhead* LookAhead::Previous()
{
  return previous;
}

inline void LookAhead::SetUV(float uu, float vv)
{
  u = uu;
  v = vv;
}

inline float* LookAhead::Movement() 
{
  return movement;
}

inline float LookAhead::U() 
{
  return u;
}

inline float LookAhead::V() 
{
  return v;
}

inline boolean LookAhead::Processed() 
{
  return processed;
}

inline void LookAhead::SetProcessed() 
{
  processed = true;
}

inline boolean Move::DDARingEmpty()
{
  return ddaRingGetPointer == ddaRingAddPointer;
}

// Leave a gap of 2 as the last Get result may still be being processed

inline boolean Move::DDARingFull()
{
  return ddaRingGetPointer->Next()->Next() == ddaRingAddPointer;
}

inline boolean Move::LookAheadRingEmpty()
{
  return lookAheadRingCount == 0;
}

// Leave a gap of 2 as the last Get result may still be being processed

inline boolean Move::LookAheadRingFull()
{
  return lookAheadRingGetPointer->Next()->Next() == lookAheadRingAddPointer;
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


#endif
