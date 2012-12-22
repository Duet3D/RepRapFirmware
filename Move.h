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

class Move
{   
  public:
  
    Move(Platform* p);
    void spin();
    
  private:
  
    Platform* platform;
    unsigned long time;
  
};

#endif
