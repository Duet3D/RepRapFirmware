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
    boolean Init(float currentPosition[], float targetPosition[]);
    void Start();
    void Step();
    boolean Active();

  private:        
    Move* move;
    Platform* platform;
    long counter[DRIVES+1];
    long delta[DRIVES+1];
    boolean directions[DRIVES+1];
    long totalSteps;
    long stepCountDown;
    boolean active;
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


    
  private:
  
    Platform* platform;
    GCodes* gCodes;
    DDA* dda;
    unsigned long lastTime;
    boolean active;
    float currentFeedrate;
    float currentPosition[AXES]; // Note - drives above AXES are always relative moves
    float nextMove[DRIVES + 1];  // Extra is for feedrate
};

inline boolean DDA::Active()
{
  return active;
}

inline void Move::Interrupt()
{
  dda->Step();
}


#endif
