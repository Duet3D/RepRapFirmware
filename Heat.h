/****************************************************************************************************

RepRapFirmware - Heat

This is all the code to deal with heat and temperature.

-----------------------------------------------------------------------------------------------------

Version 0.1

18 November 2012

Adrian Bowyer
RepRap Professional Ltd
http://reprappro.com

Licence: GPL

****************************************************************************************************/

#ifndef HEAT_H
#define HEAT_H

class PID
{
  public:
  
    PID();
    
  private:
  
};

class Heat
{
    
  public:
  
    Heat(Platform* p);
    void spin();
    
  private:
  
  Platform* platform;
  unsigned long time;
  
};


#endif
