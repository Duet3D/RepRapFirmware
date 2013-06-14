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
  
    PID(Platform* p, int8_t h);
    void Init();
    void Spin();
    void SetTemperature(float t);
  
  private:
  
    Platform* platform;
    float temperature;
    int8_t heater;
};

class Heat
{
    
  public:
  
    Heat(Platform* p, GCodes* g);
    void Spin();
    void Init();
    void Exit();
    
  private:
  
    Platform* platform;
    GCodes* gCodes;
    boolean active;
    PID* pids[HEATERS];
    float lastTime;
};


//***********************************************************************************************************

inline void PID::SetTemperature(float t)
{
  temperature = t;
}



#endif
