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
    void SetActiveTemperature(float t);
    float GetActiveTemperature() const;
    void SetStandbyTemperature(float t);
    float GetStandbyTemperature() const;
    void Activate();
    void Standby();
    bool Active() const;
    void ResetFault();
    float GetTemperature() const;
  
  private:
  
    Platform* platform;
    float activeTemperature;
    float standbyTemperature;
    float temperature;
    float lastTemperature;
    float temp_iState;
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
    void SetActiveTemperature(int8_t heater, float t);
    float GetActiveTemperature(int8_t heater) const;
    void SetStandbyTemperature(int8_t heater, float t);
    float GetStandbyTemperature(int8_t heater) const;
    void Activate(int8_t heater);
    void Standby(int8_t heater);
    float GetTemperature(int8_t heater) const;
    void ResetFault(int8_t heater);
    bool AllHeatersAtSetTemperatures() const;
    bool HeaterAtSetTemperature(int8_t heater) const;			// Is a specific heater at temperature within tolerance?
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

inline bool PID::Active() const
{
	return active;
}

inline void PID::SetActiveTemperature(float t)
{
  activeTemperature = t;
}

inline float PID::GetActiveTemperature() const
{
  return activeTemperature;
}

inline void PID::SetStandbyTemperature(float t)
{
  standbyTemperature = t;
}

inline float PID::GetStandbyTemperature() const
{
  return standbyTemperature;
}

inline float PID::GetTemperature() const
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


inline void Heat::SetActiveTemperature(int8_t heater, float t)
{
  if (heater >= 0 && heater < HEATERS)
  {
    pids[heater]->SetActiveTemperature(t);
  }
}

inline float Heat::GetActiveTemperature(int8_t heater) const
{
	return (heater >= 0 && heater < HEATERS) ? pids[heater]->GetActiveTemperature() : ABS_ZERO;
}

inline void Heat::SetStandbyTemperature(int8_t heater, float t)
{
  if (heater >= 0 && heater < HEATERS)
  {
    pids[heater]->SetStandbyTemperature(t);
  }
}

inline float Heat::GetStandbyTemperature(int8_t heater) const
{
  return (heater >= 0 && heater < HEATERS) ? pids[heater]->GetStandbyTemperature() : ABS_ZERO;
}

inline float Heat::GetTemperature(int8_t heater) const
{
  return (heater >= 0 && heater < HEATERS) ? pids[heater]->GetTemperature() : ABS_ZERO;
}

inline void Heat::Activate(int8_t heater)
{
  if (heater >= 0 && heater < HEATERS)
  {
    pids[heater]->Activate();
  }
}

inline void Heat::Standby(int8_t heater)
{
  if (heater >= 0 && heater < HEATERS)
  {
    pids[heater]->Standby();
  }
}

inline void Heat::ResetFault(int8_t heater)
{
  if (heater >= 0 && heater < HEATERS)
  {
    pids[heater]->ResetFault();
  }
}



#endif
