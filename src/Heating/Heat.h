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
#include "GCodes/GCodeResult.h"

class TemperatureSensor;
class HeaterProtection;
class GCodeBuffer;

class Heat
{
public:
	// Enumeration to describe the status of a heater. Note that the web interface returns the numerical values, so don't change them.
	enum HeaterStatus { HS_off = 0, HS_standby = 1, HS_active = 2, HS_fault = 3, HS_tuning = 4 };

	Heat(Platform& p);
#ifdef RTOS
	void Task();
#else
	void Spin();												// Called in a tight loop to keep everything going
#endif
	void Init();												// Set everything up
	void Exit();												// Shut everything down
	void ResetHeaterModels();									// Reset all active heater models to defaults

	bool ColdExtrude() const;									// Is cold extrusion allowed?
	void AllowColdExtrude(bool b);								// Allow or deny cold extrusion

	int8_t GetBedHeater(size_t index) const						// Get a hot bed heater number
	pre(index < NumBedHeaters);
	void SetBedHeater(size_t index, int8_t heater)				// Set a hot bed heater number
	pre(index < NumBedHeaters; -1 <= heater; heater < Heaters);
	bool IsBedHeater(int8_t heater) const;						// Check if this heater is a bed heater

	int8_t GetChamberHeater(size_t index) const					// Get a chamber heater number
	pre(index < NumChamberHeaters);
	void SetChamberHeater(size_t index, int8_t heater)			// Set a chamber heater number
	pre(index < NumChamberHeaters; -1 <= heater; heater < Heaters);
	bool IsChamberHeater(int8_t heater) const;					// Check if this heater is a chamber heater

	void SetActiveTemperature(int8_t heater, float t);
	float GetActiveTemperature(int8_t heater) const;
	void SetStandbyTemperature(int8_t heater, float t);
	float GetStandbyTemperature(int8_t heater) const;
	float GetHighestTemperatureLimit(int8_t heater) const;
	float GetLowestTemperatureLimit(int8_t heater) const;
	void Activate(int8_t heater);								// Turn on a heater
	void Standby(int8_t heater, const Tool* tool);				// Set a heater to standby
	float GetTemperature(int8_t heater) const;					// Get the temperature of a heater
	float GetTargetTemperature(int8_t heater) const;			// Get the target temperature
	HeaterStatus GetStatus(int8_t heater) const;				// Get the off/standby/active status
	void SwitchOff(int8_t heater);								// Turn off a specific heater
	void SwitchOffAll(bool includingChamberAndBed);				// Turn all heaters off
	void ResetFault(int8_t heater);								// Reset a heater fault - only call this if you know what you are doing
	bool AllHeatersAtSetTemperatures(bool includingBed) const;	// Is everything at temperature within tolerance?
	bool HeaterAtSetTemperature(int8_t heater, bool waitWhenCooling) const;	// Is a specific heater at temperature within tolerance?
	void Diagnostics(MessageType mtype);						// Output useful information

	float GetAveragePWM(size_t heater) const					// Return the running average PWM to the heater as a fraction in [0, 1].
	pre(heater < Heaters);

	bool IsBedOrChamberHeater(int8_t heater) const;						// Queried by the Platform class

	uint32_t GetLastSampleTime(size_t heater) const
	pre(heater < Heaters);

	void StartAutoTune(size_t heater, float temperature, float maxPwm, const StringRef& reply) // Auto tune a PID
	pre(heater < Heaters);

	bool IsTuning(size_t heater) const							// Return true if the specified heater is auto tuning
	pre(heater < Heaters);

	void GetAutoTuneStatus(const StringRef& reply) const;		// Get the status of the current or last auto tune

	const FopDt& GetHeaterModel(size_t heater) const			// Get the process model for the specified heater
	pre(heater < Heaters);

	bool SetHeaterModel(size_t heater, float gain, float tc, float td, float maxPwm, float voltage, bool usePid, bool inverted, PwmFrequency pwmFreq) // Set the heater process model
	pre(heater < Heaters);

	bool IsHeaterSignalInverted(size_t heater)					// Set PWM signal inversion
	pre(heater < Heaters);

	void SetHeaterSignalInverted(size_t heater, bool IsInverted)	// Set PWM signal inversion
	pre(heater < Heaters);

	void GetFaultDetectionParameters(size_t heater, float& maxTempExcursion, float& maxFaultTime) const
	pre(heater < Heaters);

	void SetFaultDetectionParameters(size_t heater, float maxTempExcursion, float maxFaultTime)
	pre(heater < Heaters);

	bool IsHeaterEnabled(size_t heater) const					// Is this heater enabled?
	pre(heater < Heaters);

	float GetHighestTemperatureLimit() const;					// Get the highest temperature limit of any heater

	void SetM301PidParameters(size_t heater, const M301PidParameters& params)
	pre(heater < Heaters);

	bool WriteModelParameters(FileStore *f) const;				// Write heater model parameters to file returning true if no error

	int GetHeaterChannel(size_t heater) const;					// Return the channel used by a particular heater, or -1 if not configured
	bool SetHeaterChannel(size_t heater, int channel);			// Set the channel used by a heater, returning true if bad heater or channel number
	GCodeResult ConfigureHeaterSensor(size_t heater, unsigned int mcode, GCodeBuffer& gb, const StringRef& reply);	// Configure the temperature sensor for a channel
	const char *GetHeaterName(size_t heater) const;				// Get the name of a heater, or nullptr if it hasn't been named

	HeaterProtection& AccessHeaterProtection(size_t index) const;	// Return the protection parameters of the given index
	void UpdateHeaterProtection();								// Updates the PIDs and HeaterProtection items when a heater is remapped

	bool CheckHeater(size_t heater)								// Check if the heater is able to operate
	pre(heater < Heaters);

	float GetTemperature(size_t heater, TemperatureError& err); // Result is in degrees Celsius

	const Tool* GetLastStandbyTool(int heater) const
	pre(heater >= 0; heater < Heaters)
	{
		return lastStandbyTools[heater];
	}

	bool WriteBedAndChamberTempSettings(FileStore *f) const;	// Save some resume information

	void SuspendHeaters(bool sus);								// Suspend the heaters to conserve power

private:
	Heat(const Heat&);											// Private copy constructor to prevent copying

	TemperatureSensor **GetSensor(size_t heater);				// Get a pointer to the temperature sensor entry
	TemperatureSensor * const *GetSensor(size_t heater) const;	// Get a pointer to the temperature sensor entry

	Platform& platform;											// The instance of the RepRap hardware class

	HeaterProtection *heaterProtections[Heaters + NumExtraHeaterProtections];	// Heater protection instances to guarantee legal heater temperature ranges

	PID* pids[Heaters];											// A PID controller for each heater
	const Tool* lastStandbyTools[Heaters];						// The last tool that caused the corresponding heater to be set to standby

	TemperatureSensor *heaterSensors[Heaters];					// The sensor used by the real heaters
	TemperatureSensor *virtualHeaterSensors[MaxVirtualHeaters];	// Sensors for virtual heaters

#ifdef RTOS
	uint32_t lastWakeTime;
#else
	uint32_t lastTime;											// The last time our Spin() was called
	bool active;												// Are we active?
#endif

	bool coldExtrude;											// Is cold extrusion allowed?
	int8_t bedHeaters[NumBedHeaters];							// Indices of the hot bed heaters to use or -1 if none is available
	int8_t chamberHeaters[NumChamberHeaters];					// Indices of the chamber heaters to use or -1 if none is available
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

inline int8_t Heat::GetBedHeater(size_t index) const
{
	return bedHeaters[index];
}

inline int8_t Heat::GetChamberHeater(size_t index) const
{
	return chamberHeaters[index];
}

// Get the process model for the specified heater
inline const FopDt& Heat::GetHeaterModel(size_t heater) const
{
	return pids[heater]->GetModel();
}

// Set the heater process model
inline bool Heat::SetHeaterModel(size_t heater, float gain, float tc, float td, float maxPwm, float voltage, bool usePid, bool inverted, PwmFrequency pwmFreq)
{
	return pids[heater]->SetModel(gain, tc, td, maxPwm, voltage, usePid, inverted, pwmFreq);
}

inline bool Heat::IsHeaterSignalInverted(size_t heater)
{
	return pids[heater]->IsHeaterSignalInverted();
}

inline void Heat::SetHeaterSignalInverted(size_t heater, bool IsInverted)
{
	pids[heater]->SetHeaterSignalInverted(IsInverted);
}

// Is the heater enabled?
inline bool Heat::IsHeaterEnabled(size_t heater) const
{
	return pids[heater]->IsHeaterEnabled();
}

inline void Heat::GetFaultDetectionParameters(size_t heater, float& maxTempExcursion, float& maxFaultTime) const
{
	pids[heater]->GetFaultDetectionParameters(maxTempExcursion, maxFaultTime);
}

inline void Heat::SetFaultDetectionParameters(size_t heater, float maxTempExcursion, float maxFaultTime)
{
	pids[heater]->SetFaultDetectionParameters(maxTempExcursion, maxFaultTime);
}

#endif
