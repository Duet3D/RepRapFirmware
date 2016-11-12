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

#include "Pid.h"

class Heat
{
public:
	// Enumeration to describe the status of a heater. Note that the web interface returns the numerical values, so don't change them.
	enum HeaterStatus { HS_off = 0, HS_standby = 1, HS_active = 2, HS_fault = 3, HS_tuning = 4 };

	Heat(Platform* p);
	void Spin();												// Called in a tight loop to keep everything going
	void Init();												// Set everything up
	void Exit();												// Shut everything down

	bool ColdExtrude() const;									// Is cold extrusion allowed?
	void AllowColdExtrude();									// Allow cold extrusion
	void DenyColdExtrude();										// Deny cold extrusion

	int8_t GetBedHeater() const									// Get hot bed heater number
	post(-1 <= result; result < HEATERS);

	void SetBedHeater(int8_t heater)							// Set hot bed heater number
	pre(-1 <= heater; heater < HEATERS);

	int8_t GetChamberHeater() const								// Get chamber heater number
	post(-1 <= result; result < HEATERS);

	void SetChamberHeater(int8_t heater)						// Set chamber heater number
	pre(-1 <= heater; heater < HEATERS);

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
	bool HeaterAtSetTemperature(int8_t heater, bool waitWhenCooling) const;	// Is a specific heater at temperature within tolerance?
	void Diagnostics(MessageType mtype);						// Output useful information

	float GetAveragePWM(size_t heater) const					// Return the running average PWM to the heater as a fraction in [0, 1].
	pre(heater < HEATERS);

	bool UseSlowPwm(int8_t heater) const;						// Queried by the Platform class

	uint32_t GetLastSampleTime(size_t heater) const
	pre(heater < HEATERS);

	void StartAutoTune(size_t heater, float temperature, float maxPwm, StringRef& reply) // Auto tune a PID
	pre(heater < HEATERS);

	bool IsTuning(size_t heater) const							// Return true if the specified heater is auto tuning
	pre(heater < HEATERS);

	void GetAutoTuneStatus(StringRef& reply) const;				// Get the status of the current or last auto tune

	const FopDt& GetHeaterModel(size_t heater) const			// Get the process model for the specified heater
	pre(heater < HEATERS);

	bool SetHeaterModel(size_t heater, float gain, float tc, float td, float maxPwm, bool usePid) // Set the heater process model
	pre(heater < HEATERS);

	bool IsModelUsed(size_t heater) const						// Is the heater using the PID parameters calculated form the model?
	pre(heater < HEATERS);

	void UseModel(size_t heater, bool b)						// Use or don't use the model to provide the PID parameters
	pre(heater < HEATERS);

	void GetHeaterProtection(size_t heater, float& maxTempExcursion, float& maxFaultTime) const
	pre(heater < HEATERS);

	void SetHeaterProtection(size_t heater, float maxTempExcursion, float maxFaultTime)
	pre(heater < HEATERS);

	bool IsHeaterEnabled(size_t heater) const					// Is this heater enabled?
	pre(heater < HEATERS);

private:
	Platform* platform;											// The instance of the RepRap hardware class
	PID* pids[HEATERS];											// A PID controller for each heater

	uint32_t lastTime;											// The last time our Spin() was called
	float longWait;												// Long time for things that happen occasionally

	bool active;												// Are we active?
	bool coldExtrude;											// Is cold extrusion allowed?
	int8_t bedHeater;											// Index of the hot bed heater to use or -1 if none is available
	int8_t chamberHeater;										// Index of the chamber heater to use or -1 if none is available
	int8_t heaterBeingTuned;									// which PID is currently being tuned
	int8_t lastHeaterTuned;										// which PID we last finished tuning
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

// Get the process model for the specified heater
inline const FopDt& Heat::GetHeaterModel(size_t heater) const
{
	return pids[heater]->GetModel();
}

// Set the heater process model
inline bool Heat::SetHeaterModel(size_t heater, float gain, float tc, float td, float maxPwm, bool usePid)
{
	return pids[heater]->SetModel(gain, tc, td, maxPwm, usePid);
}

// Is the heater using the PID parameters calculated form the model?
inline bool Heat::IsModelUsed(size_t heater) const
{
	return pids[heater]->IsModelUsed();
}

// Use or don't use the model to provide the PID parameters
inline void Heat::UseModel(size_t heater, bool b)
{
	pids[heater]->UseModel(b);
}

// Is the heater enabled?
inline bool Heat::IsHeaterEnabled(size_t heater) const
{
	return pids[heater]->IsHeaterEnabled();
}

inline void Heat::GetHeaterProtection(size_t heater, float& maxTempExcursion, float& maxFaultTime) const
{
	pids[heater]->GetHeaterProtection(maxTempExcursion, maxFaultTime);
}

inline void Heat::SetHeaterProtection(size_t heater, float maxTempExcursion, float maxFaultTime)
{
	pids[heater]->SetHeaterProtection(maxTempExcursion, maxFaultTime);
}

#endif
