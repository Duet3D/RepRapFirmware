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
    float GetActiveTemperature();
    void SetStandbyTemperature(const float& t);
    float GetStandbyTemperature();
    void Activate();
    void Standby();
    bool Active();
    void ResetFault();
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
    int8_t badTemperatureCount;
    bool temperatureFault;
};

class Heat
{
    
  public:
  
    Heat(Platform* p, GCodes* g);
    void Spin();
    void Init();
    void Exit();
    void SetActiveTemperature(int8_t heater, const float& t);
    float GetActiveTemperature(int8_t heater);
    void SetStandbyTemperature(int8_t heater, const float& t);
    float GetStandbyTemperature(int8_t heater);
    void Activate(int8_t heater);
    void Standby(int8_t heater);
    float GetTemperature(int8_t heater);
    void ResetFault(int8_t heater);
    bool AllHeatersAtSetTemperatures();
    void Diagnostics();
    
  private:
  
    Platform* platform;
    GCodes* gCodes;
    bool active;
    PID* pids[HEATERS];
    float lastTime;
    float longWait;
};


//***********************************************************************************************************

inline bool PID::Active()
{
	return active;
}

inline void PID::SetActiveTemperature(const float& t)
{
  activeTemperature = t;
}

inline float PID::GetActiveTemperature()
{
  return activeTemperature;
}

inline void PID::SetStandbyTemperature(const float& t)
{
  standbyTemperature = t;
}

inline float PID::GetStandbyTemperature()
{
  return standbyTemperature;
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

inline void PID::ResetFault()
{
	temperatureFault = false;
	badTemperatureCount = 0;
}


inline void Heat::SetActiveTemperature(int8_t heater, const float& t)
{
  pids[heater]->SetActiveTemperature(t);
}

inline float Heat::GetActiveTemperature(int8_t heater)
{
  return pids[heater]->GetActiveTemperature();
}

inline void Heat::SetStandbyTemperature(int8_t heater, const float& t)
{
  pids[heater]->SetStandbyTemperature(t);
}

inline float Heat::GetStandbyTemperature(int8_t heater)
{
  return pids[heater]->GetStandbyTemperature();
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

inline void Heat::ResetFault(int8_t heater)
{
  pids[heater]->ResetFault();
}



#endif
