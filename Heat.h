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
    void SetActiveTemperature(const float& t);
    void SetStandbyTemperature(const float& t);
    void Activate();
    void Standby();
    float GetTemperature();
  
  private:
  
    Platform* platform;
    float activeTemperature;
    float standbyTemperature;
    float temperature;
    float lastTemperature;
    float temp_iState;
    float temp_dState;
    bool active;
    int8_t heater;
};

class Heat
{
    
  public:
  
    Heat(Platform* p, GCodes* g);
    void Spin();
    void Init();
    void Exit();
    void SetActiveTemperature(int8_t heater, const float& t);
    void SetStandbyTemperature(int8_t heater, const float& t);
    void Activate(int8_t heater);
    void Standby(int8_t heater);
    float GetTemperature(int8_t heater);
    void Diagnostics();
    
  private:
  
    Platform* platform;
    GCodes* gCodes;
    bool active;
    PID* pids[HEATERS];
    float lastTime;
};


//***********************************************************************************************************

inline void PID::SetActiveTemperature(const float& t)
{
  activeTemperature = t;
}

inline void PID::SetStandbyTemperature(const float& t)
{
  standbyTemperature = t;
}

inline float PID::GetTemperature()
{
  return temperature;
}

inline void PID::Activate()
{
  active = true;
}

inline void PID::Standby()
{
  active = false;
}

inline void Heat::SetActiveTemperature(int8_t heater, const float& t)
{
  pids[heater]->SetActiveTemperature(t);
}

inline void Heat::SetStandbyTemperature(int8_t heater, const float& t)
{
  pids[heater]->SetStandbyTemperature(t);
}

inline float Heat::GetTemperature(int8_t heater)
{
  return pids[heater]->GetTemperature();
}

inline void Heat::Activate(int8_t heater)
{
  pids[heater]->Activate();
}

inline void Heat::Standby(int8_t heater)
{
  pids[heater]->Standby();
}

#endif
