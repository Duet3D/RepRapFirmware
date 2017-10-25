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

#include "RepRapFirmware.h"
#include "Pid.h"
#include "MessageType.h"

class TemperatureSensor;
class GCodeBuffer;

class Heat
{
public:
	// Enumeration to describe the status of a heater. Note that the web interface returns the numerical values, so don't change them.
	enum HeaterStatus { HS_off = 0, HS_standby = 1, HS_active = 2, HS_fault = 3, HS_tuning = 4 };

	Heat(Platform& p);
	void Spin();												// Called in a tight loop to keep everything going
	void Init();												// Set everything up
	void Exit();												// Shut everything down
	void ResetHeaterModels();									// Reset all active heater models to defaults

	bool ColdExtrude() const;									// Is cold extrusion allowed?
	void AllowColdExtrude(bool b);								// Allow or deny cold extrusion

	int8_t GetBedHeater() const									// Get hot bed heater number
	post(-1 <= result; result < Heaters);

	void SetBedHeater(int8_t heater)							// Set hot bed heater number
	pre(-1 <= heater; heater < Heaters);

	int8_t GetChamberHeater() const								// Get chamber heater number
	post(-1 <= result; result < Heaters);

	void SetChamberHeater(int8_t heater)						// Set chamber heater number
	pre(-1 <= heater; heater < Heaters);

	void SetActiveTemperature(int8_t heater, float t);
	float GetActiveTemperature(int8_t heater) const;
	void SetStandbyTemperature(int8_t heater, float t);
	float GetStandbyTemperature(int8_t heater) const;
	void SetTemperatureLimit(int8_t heater, float t);
	float GetTemperatureLimit(int8_t heater) const;
	void Activate(int8_t heater);								// Turn on a heater
	void Standby(int8_t heater, const Tool* tool);				// Set a heater to standby
	float GetTemperature(int8_t heater) const;					// Get the temperature of a heater
	float GetTargetTemperature(int8_t heater) const;			// Get the target temperature
	HeaterStatus GetStatus(int8_t heater) const;				// Get the off/standby/active status
	void SwitchOff(int8_t heater);								// Turn off a specific heater
	void SwitchOffAll();										// Turn all heaters off
	void ResetFault(int8_t heater);								// Reset a heater fault - only call this if you know what you are doing
	bool AllHeatersAtSetTemperatures(bool includingBed) const;	// Is everything at temperature within tolerance?
	bool HeaterAtSetTemperature(int8_t heater, bool waitWhenCooling) const;	// Is a specific heater at temperature within tolerance?
	void Diagnostics(MessageType mtype);						// Output useful information

	float GetAveragePWM(size_t heater) const					// Return the running average PWM to the heater as a fraction in [0, 1].
	pre(heater < Heaters);

	bool UseSlowPwm(int8_t heater) const;						// Queried by the Platform class

	uint32_t GetLastSampleTime(size_t heater) const
	pre(heater < Heaters);

	void StartAutoTune(size_t heater, float temperature, float maxPwm, StringRef& reply) // Auto tune a PID
	pre(heater < Heaters);

	bool IsTuning(size_t heater) const							// Return true if the specified heater is auto tuning
	pre(heater < Heaters);

	void GetAutoTuneStatus(StringRef& reply) const;				// Get the status of the current or last auto tune

	const FopDt& GetHeaterModel(size_t heater) const			// Get the process model for the specified heater
	pre(heater < Heaters);

	bool SetHeaterModel(size_t heater, float gain, float tc, float td, float maxPwm, bool usePid) // Set the heater process model
	pre(heater < Heaters);

	void GetHeaterProtection(size_t heater, float& maxTempExcursion, float& maxFaultTime) const
	pre(heater < Heaters);

	void SetHeaterProtection(size_t heater, float maxTempExcursion, float maxFaultTime)
	pre(heater < Heaters);

	bool IsHeaterEnabled(size_t heater) const					// Is this heater enabled?
	pre(heater < Heaters);

	float GetHighestTemperatureLimit() const;					// Get the highest temperature limit of any heater

	void SetM301PidParameters(size_t heater, const M301PidParameters& params)
	pre(heater < Heaters);

	bool WriteModelParameters(FileStore *f) const;				// Write heater model parameters to file returning true if no error

	int GetHeaterChannel(size_t heater) const;					// Return the channel used by a particular heater, or -1 if not configured
	bool SetHeaterChannel(size_t heater, int channel);			// Set the channel used by a heater, returning true if bad heater or channel number
	bool ConfigureHeaterSensor(size_t heater, unsigned int mcode, GCodeBuffer& gb, StringRef& reply, bool& error);	// Configure the temperature sensor for a channel
	const char *GetHeaterName(size_t heater) const;				// Get the name of a heater, or nullptr if it hasn't been named

	float GetTemperature(size_t heater, TemperatureError& err); // Result is in degrees Celsius

	const Tool* GetLastStandbyTool(int heater) const
	pre(heater >= 0; heater < Heaters)
	{
		return lastStandbyTools[heater];
	}

	bool WriteBedAndChamberTempSettings(FileStore *f) const;	// Save some resume information

#if HAS_VOLTAGE_MONITOR
	void SuspendHeaters(bool sus);								// Suspend the heaters to conserve power
#endif

private:
	Heat(const Heat&);											// Private copy constructor to prevent copying
	TemperatureSensor **GetSensor(size_t heater);				// Get a pointer to the temperature sensor entry
	TemperatureSensor * const *GetSensor(size_t heater) const;	// Get a pointer to the temperature sensor entry

	Platform& platform;											// The instance of the RepRap hardware class

	PID* pids[Heaters];											// A PID controller for each heater
	const Tool* lastStandbyTools[Heaters];						// The last tool that caused the corresponding heater to be set to standby

	TemperatureSensor *heaterSensors[Heaters];					// The sensor used by the real heaters
	TemperatureSensor *virtualHeaterSensors[MaxVirtualHeaters];	// Sensors for virtual heaters

	uint32_t lastTime;											// The last time our Spin() was called
	uint32_t longWait;											// Long time for things that happen occasionally

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

inline void Heat::AllowColdExtrude(bool b)
{
	coldExtrude = b;
}

inline int8_t Heat::GetBedHeater() const
{
	return bedHeater;
}

inline int8_t Heat::GetChamberHeater() const
{
	return chamberHeater;
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
