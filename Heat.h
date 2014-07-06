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

/**
 * This class implements a PID controller for the heaters
 */

class PID
{
  friend class Heat;
  protected:
  
    PID(Platform* p, int8_t h);
    void Init();									// (Re)Set everything to start
    void Spin();									// Called in a tight loop to keep things running
    void SetActiveTemperature(float t);
    float GetActiveTemperature() const;
    void SetStandbyTemperature(float t);
    float GetStandbyTemperature() const;
    void Activate();								// Switch from idle to active
    void Standby();									// Switch from active to idle
    bool Active() const;							// Are we active?
    void SwitchOff();								// Not even standby - all heater power off
    bool SwitchedOff();								// Are we switched off?
    void ResetFault();								// Reset a fault condition - only call this if you know what you are doing
    float GetTemperature() const;					// Get the current temperature

  private:

    void SwitchOn();
  
    Platform* platform;								// The instance of the class that is the RepRap hardware
    float activeTemperature;						// The required active temperature
    float standbyTemperature;						// The required standby temperature
    float temperature;								// The current temperature
    float lastTemperature;							// The previous current temperature
    float temp_iState;								// The integral PID component
    bool active;									// Are we active or standby?
    bool switchedOff;								// Becomes false when someone tells us our active or standby temperatures
    int8_t heater;									// The index of our heater
    int8_t badTemperatureCount;						// Count of sequential dud readings
    bool temperatureFault;							// Has our heater developed a fault?
};

/**
 * The master class that controls all the heaters in the RepRap machine
 */

class Heat
{
    
  public:
  
    Heat(Platform* p, GCodes* g);
    void Spin();												// Called in a tight loop to keep everything going
    void Init();												// Set everything up
    void Exit();												// Shut everything down
    void SetActiveTemperature(int8_t heater, float t);
    float GetActiveTemperature(int8_t heater) const;
    void SetStandbyTemperature(int8_t heater, float t);
    float GetStandbyTemperature(int8_t heater) const;
    void Activate(int8_t heater);								// Turn on a heater
    void Standby(int8_t heater);								// Set a heater idle
    float GetTemperature(int8_t heater) const;					// Get the temperature of a heater
    bool SwitchedOff(int8_t heater);						    // Is this heater in use?
    void ResetFault(int8_t heater);								// Reset a heater fault - only call this if you know what you are doing
    bool AllHeatersAtSetTemperatures(bool includingBed) const;	// Is everything at temperature within tolerance?
    bool HeaterAtSetTemperature(int8_t heater) const;			// Is a specific heater at temperature within tolerance?
    void Diagnostics();											// Output useful information
    
  private:
  
    Platform* platform;							// The instance of the RepRap hardware class
    GCodes* gCodes;								// The instance of the G Code interpreter class
    bool active;								// Are we active?
    PID* pids[HEATERS];							// A PID controller for each heater
    float lastTime;								// The last time our Spin() was called
    float longWait;								// Long time for things that happen occasionally
};


//***********************************************************************************************************

inline bool PID::Active() const
{
	return active;
}

inline void PID::SetActiveTemperature(float t)
{
  SwitchOn();
  activeTemperature = t;
}

inline float PID::GetActiveTemperature() const
{
  return activeTemperature;
}

inline void PID::SetStandbyTemperature(float t)
{
  SwitchOn();
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
  SwitchOn();
  active = true;
}

inline void PID::Standby()
{
  SwitchOn();
  active = false;
}

inline void PID::ResetFault()
{
	temperatureFault = false;
	badTemperatureCount = 0;
}

inline void PID::SwitchOff()
{
	platform->SetHeater(heater, 0.0);
	active = false;
	switchedOff = true;
}


inline bool PID::SwitchedOff()
{
	return switchedOff;
}

//**********************************************************************************

// Heat

inline bool Heat::SwitchedOff(int8_t heater)
{
	return pids[heater]->SwitchedOff();
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
