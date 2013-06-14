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
  
    PID(int8_t h);
    void Init(float st, float p, float i, float d, float w);
    void Spin();
    void SetTemperature(float t);
  
  private:
  
    float temperature;
    float kp;
    float ki;
    float kd;
    float kw;
    int8_t heater;
};

class Heat
{
    
  public:
  
    Heat(Platform* p, GCodes* g);
    void Spin();
    void Init(float st);
    void Exit();
    
  private:
  
    Platform* platform;
    GCodes* gCodes;
    boolean active;
    PID* pids[HEATERS];
    float lastTime;
    float sampleTime;
};


//***********************************************************************************************************

inline void PID::SetTemperature(float t)
{
  temperature = t;
}



#endif
