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

	Heat();

	// Methods that don't relate to a particuar heater
	void Task();
	void Init();												// Set everything up
	void Exit();												// Shut everything down
	void ResetHeaterModels();									// Reset all active heater models to defaults

	bool ColdExtrude() const;									// Is cold extrusion allowed?
	void AllowColdExtrude(bool b);								// Allow or deny cold extrusion
	float GetExtrusionMinTemp() const;							// Get minimum extrusion temperature
	float GetRetractionMinTemp() const;							// Get minimum retraction temperature
	void SetExtrusionMinTemp(float t);							// Set minimum extrusion temperature
	void SetRetractionMinTemp(float t);							// Set minimum retraction temperature

	int GetBedHeater(size_t index) const						// Get a hot bed heater number
	pre(index < NumBedHeaters);
	void SetBedHeater(size_t index, int heater)					// Set a hot bed heater number
	pre(index < NumBedHeaters; -1 <= heater; heater < MaxHeaters);
	bool IsBedHeater(int heater) const;							// Check if this heater is a bed heater

	int GetChamberHeater(size_t index) const					// Get a chamber heater number
	pre(index < NumChamberHeaters);
	void SetChamberHeater(size_t index, int heater)				// Set a chamber heater number
	pre(index < NumChamberHeaters; -1 <= heater; heater < MaxHeaters);
	bool IsChamberHeater(int heater) const;						// Check if this heater is a chamber heater

	bool AllHeatersAtSetTemperatures(bool includingBed, float tolerance) const;	// Is everything at temperature within tolerance?

	void SwitchOffAll(bool includingChamberAndBed);				// Turn all heaters off
	void ResetFault(int heater);								// Reset a heater fault for a specific heater or all heaters

	void GetAutoTuneStatus(const StringRef& reply) const;		// Get the status of the current or last auto tune

	GCodeResult ConfigureSensor(GCodeBuffer& gb, const StringRef& reply);	// Create a sensor or change the parameters for an existing sensor

	HeaterProtection& AccessHeaterProtection(size_t index) const;	// Return the protection parameters of the given index
	void UpdateHeaterProtection();								// Updates the PIDs and HeaterProtection items when a heater is remapped

	void SuspendHeaters(bool sus);								// Suspend the heaters to conserve power

	TemperatureSensor *GetSensor(int sn) const;					// Get a pointer to the temperature sensor with the specified number
	TemperatureSensor *GetSensorAtOrAbove(unsigned int sn) const;	// Get a pointer to the first temperature sensor with the specified or higher number

	float GetSensorTemperature(int sensorNum, TemperatureError& err) const; // Result is in degrees Celsius

	float GetHighestTemperatureLimit() const;					// Get the highest temperature limit of any heater

	void Diagnostics(MessageType mtype);						// Output useful information

	// Methods that relate to a particular heater
	const char *GetHeaterName(size_t heater) const;				// Get the name of a heater, or nullptr if it hasn't been named
	float GetAveragePWM(size_t heater) const					// Return the running average PWM to the heater as a fraction in [0, 1].
	pre(heater < MaxHeaters);

	bool IsBedOrChamberHeater(int heater) const;				// Queried by the Platform class

	uint32_t GetLastSampleTime(size_t heater) const
	pre(heater < MaxHeaters);

	float GetHeaterTemperature(size_t heater) const;			 // Result is in degrees Celsius

	const Tool* GetLastStandbyTool(int heater) const
	pre(heater >= 0; heater < MaxHeaters)
	{
		return lastStandbyTools[heater];
	}

	bool IsHeaterEnabled(size_t heater) const					// Is this heater enabled?
	pre(heater < MaxHeaters);

	float GetActiveTemperature(int heater) const;
	float GetStandbyTemperature(int heater) const;
	float GetHighestTemperatureLimit(int heater) const;
	float GetLowestTemperatureLimit(int heater) const;
	float GetHeaterTemperature(int heater) const;				// Get the current temperature of a heater
	float GetTargetTemperature(int heater) const;				// Get the target temperature
	HeaterStatus GetStatus(int heater) const;					// Get the off/standby/active status
	bool HeaterAtSetTemperature(int heater, bool waitWhenCooling, float tolerance) const;

	GCodeResult ConfigureHeater(size_t heater, GCodeBuffer& gb, const StringRef& reply);
	void SetActiveTemperature(int heater, float t);
	void SetStandbyTemperature(int heater, float t);
	void Activate(int heater);									// Turn on a heater
	void Standby(int heater, const Tool* tool);					// Set a heater to standby
	void SwitchOff(int heater);									// Turn off a specific heater
																// Is a specific heater at temperature within tolerance?
	void StartAutoTune(size_t heater, float temperature, float maxPwm, const StringRef& reply) // Auto tune a PID
	pre(heater < MaxHeaters);

	bool IsTuning(size_t heater) const							// Return true if the specified heater is auto tuning
	pre(heater < MaxHeaters);

	const FopDt& GetHeaterModel(size_t heater) const			// Get the process model for the specified heater
	pre(heater < MaxHeaters);

	bool SetHeaterModel(size_t heater, float gain, float tc, float td, float maxPwm, float voltage, bool usePid, bool inverted) // Set the heater process model
	pre(heater < MaxHeaters);

	void GetFaultDetectionParameters(size_t heater, float& maxTempExcursion, float& maxFaultTime) const
	pre(heater < MaxHeaters);

	void SetFaultDetectionParameters(size_t heater, float maxTempExcursion, float maxFaultTime)
	pre(heater < MaxHeaters);

	bool CheckHeater(size_t heater)								// Check if the heater is able to operate
	pre(heater < MaxHeaters);

	void SetM301PidParameters(size_t heater, const M301PidParameters& params)
	pre(heater < MaxHeaters);

#if HAS_MASS_STORAGE
	bool WriteModelParameters(FileStore *f) const;				// Write heater model parameters to file returning true if no error
	bool WriteBedAndChamberTempSettings(FileStore *f) const;	// Save some resume information
#endif

private:
	Heat(const Heat&) = delete;									// Private copy constructor to prevent copying

	void RemoveSensor(unsigned int sensorNum);
	void InsertSensor(TemperatureSensor *sensor);

	TemperatureSensor *sensorsRoot;								// The sensor list
	HeaterProtection *heaterProtections[MaxHeaters + NumExtraHeaterProtections];	// Heater protection instances to guarantee legal heater temperature ranges

	PID* pids[MaxHeaters];									// A PID controller for each heater
	const Tool* lastStandbyTools[MaxHeaters];				// The last tool that caused the corresponding heater to be set to standby

	float extrusionMinTemp;										// Minimum temperature to allow regular extrusion
	float retractionMinTemp;									// Minimum temperature to allow regular retraction
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

inline float Heat::GetExtrusionMinTemp() const
{
	return extrusionMinTemp;
}

inline float Heat::GetRetractionMinTemp() const
{
	return retractionMinTemp;
}

inline void Heat::SetExtrusionMinTemp(float t)
{
	extrusionMinTemp = t;
}

inline void Heat::SetRetractionMinTemp(float t)
{
	retractionMinTemp = t;
}

inline int Heat::GetBedHeater(size_t index) const
{
	return bedHeaters[index];
}

inline int Heat::GetChamberHeater(size_t index) const
{
	return chamberHeaters[index];
}

// Get the process model for the specified heater
inline const FopDt& Heat::GetHeaterModel(size_t heater) const
{
	return pids[heater]->GetModel();
}

// Set the heater process model
inline bool Heat::SetHeaterModel(size_t heater, float gain, float tc, float td, float maxPwm, float voltage, bool usePid, bool inverted)
{
	return pids[heater]->SetModel(gain, tc, td, maxPwm, voltage, usePid, inverted);
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
