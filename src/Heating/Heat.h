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
#include "RepRap.h"
#include "Heater.h"
#include "TemperatureError.h"
#include "MessageType.h"
#include "GCodes/GCodeResult.h"
#include <RTOSIface/RTOSIface.h>

class TemperatureSensor;
class HeaterMonitor;
class GCodeBuffer;
class CanMessageSensorTemperatures;
class CanMessageHeatersStatus;

class Heat INHERIT_OBJECT_MODEL
{
public:
	Heat() noexcept;
	Heat(const Heat&) = delete;

	// Methods that don't relate to a particular heater
	[[noreturn]] void HeaterTask() noexcept;
	void Init() noexcept;												// Set everything up
	void Exit() noexcept;												// Shut everything down
	void ResetHeaterModels() noexcept;									// Reset all active heater models to defaults

	bool ColdExtrude() const noexcept;									// Is cold extrusion allowed?
	void AllowColdExtrude(bool b) noexcept;								// Allow or deny cold extrusion
	float GetExtrusionMinTemp() const noexcept;							// Get minimum extrusion temperature
	float GetRetractionMinTemp() const noexcept;						// Get minimum retraction temperature
	void SetExtrusionMinTemp(float t) noexcept;							// Set minimum extrusion temperature
	void SetRetractionMinTemp(float t) noexcept;						// Set minimum retraction temperature

	int GetBedHeater(size_t index) const noexcept						// Get a hot bed heater number
	pre(index < NumBedHeaters);
	void SetBedHeater(size_t index, int heater)	 noexcept				// Set a hot bed heater number
	pre(index < NumBedHeaters; -1 <= heater; heater < MaxHeaters);
	bool IsBedHeater(int heater) const noexcept;						// Check if this heater is a bed heater

	int GetChamberHeater(size_t index) const noexcept					// Get a chamber heater number
	pre(index < NumChamberHeaters);
	void SetChamberHeater(size_t index, int heater)	 noexcept			// Set a chamber heater number
	pre(index < NumChamberHeaters; -1 <= heater; heater < MaxHeaters);
	bool IsChamberHeater(int heater) const noexcept;					// Check if this heater is a chamber heater

	bool AllHeatersAtSetTemperatures(bool includingBed, float tolerance) const noexcept;	// Is everything at temperature within tolerance?

	void SwitchOffAll(bool includingChamberAndBed) noexcept;			// Turn all heaters off
	void SuspendHeaters(bool sus) noexcept;								// Suspend the heaters to conserve power or while probing
	GCodeResult ResetFault(int heater, const StringRef& reply) noexcept;	// Reset a heater fault for a specific heater or all heaters

	GCodeResult SetOrReportHeaterModel(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);
	GCodeResult TuneHeater(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);
	GCodeResult ConfigureSensor(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);		// Create a sensor or change the parameters for an existing sensor
	GCodeResult SetPidParameters(unsigned int heater, GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException); // Set the P/I/D parameters for a heater
	GCodeResult HandleM143(GCodeBuffer &gb, const StringRef &reply) THROWS(GCodeException);	// Configure heater protection (M143)

	[[noreturn]] void SensorsTask() noexcept;
	static void EnsureSensorsTask() noexcept;

	ReadLockedPointer<TemperatureSensor> FindSensor(int sn) const noexcept;	// Get a pointer to the temperature sensor entry
	ReadLockedPointer<TemperatureSensor> FindSensorAtOrAbove(unsigned int sn) const noexcept;	// Get a pointer to the first temperature sensor with the specified or higher number

	float GetSensorTemperature(int sensorNum, TemperatureError& err) const noexcept; // Result is in degrees Celsius

	float GetHighestTemperatureLimit() const noexcept;					// Get the highest temperature limit of any heater
	size_t GetNumHeatersToReport() const noexcept;
	size_t GetNumSensorsToReport() const noexcept;

	void Diagnostics(MessageType mtype) noexcept;						// Output useful information

	// Methods that relate to a particular heater
	const char *GetHeaterSensorName(size_t heater) const noexcept;		// Get the name of the sensor associated with heater, or nullptr if it hasn't been named
	float GetAveragePWM(size_t heater) const noexcept					// Return the running average PWM to the heater as a fraction in [0, 1].
	pre(heater < MaxHeaters);

	bool IsBedOrChamberHeater(int heater) const noexcept;				// Queried by the Platform class

	float GetHeaterTemperature(size_t heater) const noexcept;			// Result is in degrees Celsius

	const Tool* GetLastStandbyTool(int heater) const noexcept
	pre(heater >= 0; heater < MaxHeaters)
	{
		return lastStandbyTools[heater];
	}

	bool IsHeaterEnabled(size_t heater)	const noexcept;					// Is this heater enabled?
	pre(heater < MaxHeaters);

	float GetActiveTemperature(int heater) const noexcept;
	float GetStandbyTemperature(int heater) const noexcept;
	float GetHighestTemperatureLimit(int heater) const noexcept;
	float GetLowestTemperatureLimit(int heater) const noexcept;
	float GetTargetTemperature(int heater) const noexcept;				// Get the target temperature
	float GetHeaterTemperature(int heater) const noexcept;				// Get the current temperature of a heater
	HeaterStatus GetStatus(int heater) const noexcept;					// Get the off/standby/active status
	bool HeaterAtSetTemperature(int heater, bool waitWhenCooling, float tolerance) const noexcept;

	GCodeResult ConfigureHeater(GCodeBuffer& gb, const StringRef& reply);
	GCodeResult ConfigureHeaterMonitoring(size_t heater, GCodeBuffer& gb, const StringRef& reply);

	void SetActiveTemperature(int heater, float t) THROWS(GCodeException) { SetTemperature(heater, t, true); }
	void SetStandbyTemperature(int heater, float t) THROWS(GCodeException) { SetTemperature(heater, t, false); }
	GCodeResult Activate(int heater, const StringRef& reply) noexcept;	// Turn on a heater
	void Standby(int heater, const Tool* tool) noexcept;				// Set a heater to standby
	void SwitchOff(int heater) noexcept;								// Turn off a specific heater

#if HAS_MASS_STORAGE
	bool WriteModelParameters(FileStore *f) const noexcept;				// Write heater model parameters to file returning true if no error
	bool WriteBedAndChamberTempSettings(FileStore *f) const noexcept;	// Save some resume information
#endif

#if SUPPORT_CAN_EXPANSION
	void ProcessRemoteSensorsReport(CanAddress src, const CanMessageSensorTemperatures& msg) noexcept;
	void ProcessRemoteHeatersReport(CanAddress src, const CanMessageHeatersStatus& msg) noexcept;
#endif

	static ReadWriteLock sensorsLock;							// needs to be public so that the OMT in EndstopsManager can lock it

protected:
	DECLARE_OBJECT_MODEL
	OBJECT_MODEL_ARRAY(bedHeaters)
	OBJECT_MODEL_ARRAY(chamberHeaters)
	OBJECT_MODEL_ARRAY(heaters)

private:
	ReadLockedPointer<Heater> FindHeater(int heater) const noexcept;
	void DeleteSensor(unsigned int sn) noexcept;
	void InsertSensor(TemperatureSensor *newSensor) noexcept;
	void SetTemperature(int heater, float t, bool activeNotStandby) THROWS(GCodeException);

	static ReadWriteLock heatersLock;

	uint8_t volatile sensorCount;
	TemperatureSensor * volatile sensorsRoot;					// The sensor list

	Heater* heaters[MaxHeaters];								// A local or remote heater
	const Tool* lastStandbyTools[MaxHeaters];					// The last tool that caused the corresponding heater to be set to standby

	float extrusionMinTemp;										// Minimum temperature to allow regular extrusion
	float retractionMinTemp;									// Minimum temperature to allow regular retraction
	bool coldExtrude;											// Is cold extrusion allowed?
	int8_t bedHeaters[MaxBedHeaters];							// Indices of the hot bed heaters to use or -1 if none is available
	int8_t chamberHeaters[MaxChamberHeaters];					// Indices of the chamber heaters to use or -1 if none is available
	int8_t heaterBeingTuned;									// which PID is currently being tuned
	int8_t lastHeaterTuned;										// which PID we last finished tuning
};

//***********************************************************************************************************

inline bool Heat::ColdExtrude() const noexcept
{
	return coldExtrude;
}

inline void Heat::AllowColdExtrude(bool b) noexcept
{
	coldExtrude = b;
	reprap.HeatUpdated();
}

inline float Heat::GetExtrusionMinTemp() const noexcept
{
	return extrusionMinTemp;
}

inline float Heat::GetRetractionMinTemp() const noexcept
{
	return retractionMinTemp;
}

inline void Heat::SetExtrusionMinTemp(float t) noexcept
{
	extrusionMinTemp = t;
	reprap.HeatUpdated();
}

inline void Heat::SetRetractionMinTemp(float t) noexcept
{
	retractionMinTemp = t;
	reprap.HeatUpdated();
}

inline int Heat::GetBedHeater(size_t index) const noexcept
{
	return bedHeaters[index];
}

inline int Heat::GetChamberHeater(size_t index) const noexcept
{
	return chamberHeaters[index];
}

#endif
