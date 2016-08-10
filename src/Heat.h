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
 * The master class that controls all the heaters in the RepRap machine
 */

class PID;

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
    void Diagnostics(MessageType mtype);						// Output useful information
    float GetAveragePWM(int8_t heater) const;					// Return the running average PWM to the heater as a fraction in [0, 1].

    bool UseSlowPwm(int8_t heater) const;						// Queried by the Platform class
    uint32_t GetLastSampleTime(int8_t heater) const;

private:
  
    Platform* platform;											// The instance of the RepRap hardware class

    bool active;												// Are we active?
    PID* pids[HEATERS];											// A PID controller for each heater

	bool coldExtrude;											// Is cold extrusion allowed?
	int8_t bedHeater;											// Index of the hot bed heater to use or -1 if none is available
	int8_t chamberHeater;										// Index of the chamber heater to use or -1 if none is available

    float lastTime;												// The last time our Spin() was called
    float longWait;												// Long time for things that happen occasionally
};

//***********************************************************************************************************

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

#endif
