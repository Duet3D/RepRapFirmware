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
public:
  
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
    bool SwitchedOff() const;						// Are we switched off?
	bool FaultOccurred() const;						// Has a heater fault occurred?
    void ResetFault();								// Reset a fault condition - only call this if you know what you are doing
    float GetTemperature() const;					// Get the current temperature
    float GetAveragePWM() const;					// Return the running average PWM to the heater. Answer is a fraction in [0, 1].
    uint32_t GetLastSampleTime() const;				// Return when the temp sensor was last sampled
    float GetAccumulator() const;					// Return the integral accumulator

private:

    void SwitchOn();
    void SetHeater(float power) const;				// power is a fraction in [0,1]

    Platform* platform;								// The instance of the class that is the RepRap hardware
    float activeTemperature;						// The required active temperature
    float standbyTemperature;						// The required standby temperature
    float temperature;								// The current temperature
    float lastTemperature;							// The previous current temperature
    float temp_iState;								// The integral PID component
    bool active;									// Are we active or standby?
    bool switchedOff;								// Becomes false when someone tells us our active or standby temperatures
    int8_t heater;									// The index of our heater
    uint8_t badTemperatureCount;					// Count of sequential dud readings
    bool temperatureFault;							// Has our heater developed a fault?
    float timeSetHeating;							// When we were switched on
    bool heatingUp;									// Are we heating up?
    float averagePWM;								// The running average of the PWM.
	uint32_t lastSampleTime;						// Time when the temperature was last sampled by Spin()
};

/**
 * The master class that controls all the heaters in the RepRap machine
 */

class Heat
{
  public:
	// Enumeration to describe the status of a heater. Note that the web interface returns the numerical values, so don't change them.
	enum HeaterStatus { HS_off = 0, HS_standby = 1, HS_active = 2, HS_fault = 3 };
  
    Heat(Platform* p);
    void Spin();												// Called in a tight loop to keep everything going
    void Init();												// Set everything up
    void Exit();												// Shut everything down

	bool ColdExtrude() const;									// Is cold extrusion allowed?
	void AllowColdExtrude();									// Allow cold extrusion
	void DenyColdExtrude();										// Deny cold extrusion

	int8_t GetBedHeater() const;								// Get hot bed heater number
	void SetBedHeater(int8_t heater);							// Set hot bed heater number

	int8_t GetChamberHeater() const;							// Get chamber heater number
	void SetChamberHeater(int8_t heater);						// Set chamber heater number

    void SetActiveTemperature(int8_t heater, float t);
    float GetActiveTemperature(int8_t heater) const;
    void SetStandbyTemperature(int8_t heater, float t);
    float GetStandbyTemperature(int8_t heater) const;
    void Activate(int8_t heater);								// Turn on a heater
    void Standby(int8_t heater);								// Set a heater idle
    float GetTemperature(int8_t heater) const;					// Get the temperature of a heater
    HeaterStatus GetStatus(int8_t heater) const;				// Get the off/standby/active status
    void SwitchOff(int8_t heater);								// Turn off a specific heater
    void SwitchOffAll();										// Turn all heaters off
    void ResetFault(int8_t heater);								// Reset a heater fault - only call this if you know what you are doing
    bool AllHeatersAtSetTemperatures(bool includingBed) const;	// Is everything at temperature within tolerance?
    bool HeaterAtSetTemperature(int8_t heater) const;			// Is a specific heater at temperature within tolerance?
    void Diagnostics();											// Output useful information
    float GetAveragePWM(int8_t heater) const;					// Return the running average PWM to the heater as a fraction in [0, 1].

    bool UseSlowPwm(int8_t heater) const;						// Queried by the Platform class
    uint32_t GetLastSampleTime(int8_t heater) const;

  private:
  
    Platform* platform;							// The instance of the RepRap hardware class

    bool active;								// Are we active?
    PID* pids[HEATERS];							// A PID controller for each heater

	bool coldExtrude;							// Is cold extrusion allowed?
	int8_t bedHeater;							// Index of the hot bed heater to use or -1 if none is available
	int8_t chamberHeater;						// Index of the chamber heater to use or -1 if none is available

    float lastTime;								// The last time our Spin() was called
    float longWait;								// Long time for things that happen occasionally
};


//***********************************************************************************************************

inline bool PID::Active() const
{
	return active;
}

inline float PID::GetActiveTemperature() const
{
	return activeTemperature;
}

inline float PID::GetStandbyTemperature() const
{
	return standbyTemperature;
}

inline float PID::GetTemperature() const
{
	return temperature;
}

inline bool PID::FaultOccurred() const
{
	return temperatureFault;
}

inline bool PID::SwitchedOff() const
{
	return switchedOff;
}

inline uint32_t PID::GetLastSampleTime() const
{
	return lastSampleTime;
}

inline float PID::GetAccumulator() const
{
	return temp_iState;
}


//**********************************************************************************

// Heat

inline bool Heat::ColdExtrude() const
{
	return coldExtrude;
}

inline void Heat::AllowColdExtrude()
{
	coldExtrude = true;
}

inline void Heat::DenyColdExtrude()
{
	coldExtrude = false;
}

inline int8_t Heat::GetBedHeater() const
{
	return bedHeater;
}

inline void Heat::SetBedHeater(int8_t heater)
{
	bedHeater = heater;
}

inline int8_t Heat::GetChamberHeater() const
{
	return chamberHeater;
}

inline void Heat::SetChamberHeater(int8_t heater)
{
	chamberHeater = heater;
}

inline Heat::HeaterStatus Heat::GetStatus(int8_t heater) const
{
	if (heater < 0 || heater >= HEATERS)
	{
		return HS_off;
	}

	return (pids[heater]->FaultOccurred() ? HS_fault
			: pids[heater]->SwitchedOff()) ? HS_off
				: (pids[heater]->Active()) ? HS_active
					: HS_standby;
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

inline void Heat::SwitchOff(int8_t heater)
{
	if (heater >= 0 && heater < HEATERS)
	{
		pids[heater]->SwitchOff();
	}
}

inline void Heat::SwitchOffAll()
{
	for (size_t heater = 0; heater < HEATERS; ++heater)
	{
		pids[heater]->SwitchOff();
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

inline float Heat::GetAveragePWM(int8_t heater) const
{
	return pids[heater]->GetAveragePWM();
}

inline uint32_t Heat::GetLastSampleTime(int8_t heater) const
{
	return pids[heater]->GetLastSampleTime();
}

inline bool Heat::UseSlowPwm(int8_t heater) const
{
	return heater == bedHeater || heater == chamberHeater;
}

#endif
