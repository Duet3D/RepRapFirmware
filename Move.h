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
    unsigned long lastTime;
    boolean active;
    float currentFeedrate;
    float currentPosition[AXES]; // Note - drives above AXES are always relative moves
    float nextMove[DRIVES + 1];  // Extra is for feedrate
    float stepDistances[(1<<AXES)]; // Index bits: lsb -> dx, dy, dz <- msb
    float extruderStepDistances[(1<<(DRIVES-AXES))]; // NB - limits us to 5 extruders
};

inline boolean DDA::Active()
{
  return active;
}

inline boolean DDA::VelocitiesAltered()
{
  return velocitiesAltered;
}

inline void Move::Interrupt()
{
  dda->Step(true);
}


#endif
