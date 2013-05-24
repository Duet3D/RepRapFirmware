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

#define BUFFER_LENGTH 10

class DDA
{  
  public:
    DDA(Move* m, Platform* p);
    boolean Init(float currentPosition[], float targetPosition[], float& u, float& v);
    void Start(boolean noTest);
    void Step(boolean noTest);
    boolean Active();
    boolean VelocitiesAltered();

  private:  
    Move* move;
    Platform* platform;
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
    boolean velocitiesAltered;
    volatile boolean active;
};

//*********************************************************************************************

class DDARingBuffer
{
  public:
   DDARingBuffer(Move* m, Platform* p);
   boolean Add(float currentPosition[], float targetPosition[], float& u, float& v);
   DDA* Get();

  private:
   boolean Empty();
   boolean Full();
   boolean getLock();
   void releaseLock();
   
   Platform* platform;
   DDA* ring[RING_LENGTH];
   volatile char addPointer;
   volatile char getPointer;
   volatile boolean locked;
};

//**************************************************************************************************

class Move
{   
  public:
  
    Move(Platform* p, GCodes* g);
    void Init();
    void Spin();
    void Exit();
    void Qmove();
    void GetCurrentState(float m[]);
    void Interrupt();

  friend class DDA;
    
  private:
  
    Platform* platform;
    GCodes* gCodes;
    DDA* dda;
    DDARingBuffer* ddaRingBuffer;
    unsigned long lastTime;
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

inline boolean DDA::VelocitiesAltered()
{
  return velocitiesAltered;
}

//*****************************************************************************************************

inline boolean DDARingBuffer::getLock()
{
  if(locked)
    return false;
  locked = true;
  return true;
}

inline void DDARingBuffer::releaseLock()
{
  if(!locked)
    platform->Message(HOST_MESSAGE, "Attempt to unlock an already unlocked ring buffer!\n");
  locked = false;
  return;
}

inline boolean DDARingBuffer::Add(float currentPosition[], float targetPosition[], float& u, float& v)
{
  if(getLock())
  {
    if(Full())
    {
      releaseLock();
      return false;
    }
    if(ring[addPointer]->Active())
    {
      platform->Message(HOST_MESSAGE, "Attempt to alter an active ring buffer entry!\n");
      releaseLock();
      return false;
    }
    if(!ring[addPointer]->Init(currentPosition, targetPosition, u, v))
    {
      // Throw it away
      releaseLock();
      return true;
    }
    addPointer++;
    if(addPointer >= RING_LENGTH)
      addPointer = 0;
    releaseLock();
    return true;
  }
  return false;
}


inline DDA* DDARingBuffer::Get()
{
  DDA* result = NULL;
  if(getLock())
  {
    if(Empty())
    {
      releaseLock();
      return NULL;
    }
    result = ring[getPointer];
    getPointer++;
    if(getPointer >= RING_LENGTH)
      getPointer = 0;
    releaseLock();
    return result;
  }
  return NULL;
}

inline boolean DDARingBuffer::Empty()
{
  return getPointer == addPointer;
}

// Leave a gap of 2 as the last Get result may still be being processed

inline boolean DDARingBuffer::Full()
{
  if(getPointer == 0)
    return addPointer == RING_LENGTH - 2;
  if(getPointer == 1)
    return addPointer == RING_LENGTH - 1;    
  return addPointer == getPointer - 2;
}

//**********************************************************************************************

inline void Move::Interrupt()
{
  if(dda == NULL)
  {
    dda = ddaRingBuffer->Get();
    if(dda != NULL)
      dda->Start(true);
    return;   
  }
  
  if(dda->Active())
  {
    dda->Step(true);
    return;
  }
  
  dda = NULL;
}


#endif
