/*
 * Heater.h
 *
 *  Created on: 24 Jul 2019
 *      Author: David
 */

#ifndef SRC_HEATING_HEATER_H_
#define SRC_HEATING_HEATER_H_

#include <RepRapFirmware.h>
#include <NamedEnum.h>
#include "FOPDT.h"
#include "GCodes/GCodeResult.h"
#include <ObjectModel/ObjectModel.h>

#if SUPPORT_CAN_EXPANSION
# include "CanId.h"
#endif

class HeaterProtection;
struct CanHeaterReport;

// Enumeration to describe the status of a heater. Note that the web interface returns the numerical values, so don't change them.
NamedEnum(HeaterStatus, uint8_t, off, standby, active, fault, tuning, offline);

class Heater INHERIT_OBJECT_MODEL
{
public:
	Heater(unsigned int num) noexcept;
	virtual ~Heater() noexcept;

	// Configuration methods
	virtual GCodeResult ConfigurePortAndSensor(const char *portName, PwmFrequency freq, unsigned int sensorNumber, const StringRef& reply) = 0;
	virtual GCodeResult SetPwmFrequency(PwmFrequency freq, const StringRef& reply) = 0;
	virtual GCodeResult ReportDetails(const StringRef& reply) const noexcept = 0;

	virtual float GetTemperature() const noexcept = 0;					// Get the current temperature
	virtual float GetAveragePWM() const noexcept = 0;					// Return the running average PWM to the heater. Answer is a fraction in [0, 1].
	virtual GCodeResult ResetFault(const StringRef& reply) noexcept = 0;	// Reset a fault condition - only call this if you know what you are doing
	virtual void SwitchOff() noexcept = 0;
	virtual void Spin() noexcept = 0;
	virtual void StartAutoTune(float targetTemp, float maxPwm, const StringRef& reply) noexcept = 0;	// Start an auto tune cycle for this PID
	virtual void GetAutoTuneStatus(const StringRef& reply) const = 0;	// Get the auto tune status or last result
	virtual void Suspend(bool sus) noexcept = 0;							// Suspend the heater to conserve power or while doing Z probing
	virtual float GetAccumulator() const noexcept = 0;					// Get the inertial term accumulator

#if SUPPORT_CAN_EXPANSION
	virtual void UpdateRemoteStatus(CanAddress src, const CanHeaterReport& report) noexcept = 0;
#endif

	HeaterStatus GetStatus() const noexcept;								// Get the status of the heater
	unsigned int GetHeaterNumber() const noexcept { return heaterNumber; }
	const char *GetSensorName() const noexcept;							// Get the name of the sensor for this heater, or nullptr if it hasn't been named
	void SetActiveTemperature(float t) noexcept;
	float GetActiveTemperature() const noexcept { return activeTemperature; }
	void SetStandbyTemperature(float t) noexcept;
	float GetStandbyTemperature() const noexcept { return standbyTemperature; }
	GCodeResult Activate(const StringRef& reply) noexcept;				// Switch from idle to active
	void Standby() noexcept;												// Switch from active to idle

	void GetFaultDetectionParameters(float& pMaxTempExcursion, float& pMaxFaultTime) const noexcept
		{ pMaxTempExcursion = maxTempExcursion; pMaxFaultTime = maxHeatingFaultTime; }

	GCodeResult SetFaultDetectionParameters(float pMaxTempExcursion, float pMaxFaultTime, const StringRef& reply) noexcept;

	float GetHighestTemperatureLimit() const noexcept;					// Get the highest temperature limit
	float GetLowestTemperatureLimit() const noexcept;					// Get the lowest temperature limit
	void SetHeaterProtection(HeaterProtection *h) noexcept;

	const FopDt& GetModel() const noexcept { return model; }				// Get the process model
	GCodeResult SetModel(float gain, float tc, float td, float maxPwm, float voltage, bool usePid, bool inverted, const StringRef& reply) noexcept;	// Set the process model
	void SetModelDefaults() noexcept;

	bool IsHeaterEnabled() const noexcept								// Is this heater enabled?
		{ return model.IsEnabled(); }

	void SetM301PidParameters(const M301PidParameters& params) noexcept
		{ model.SetM301PidParameters(params); }

	bool CheckGood() const noexcept;

protected:
	DECLARE_OBJECT_MODEL

	enum class HeaterMode : uint8_t
	{
		// The order of these is important because we test "mode > HeatingMode::suspended" to determine whether the heater is active
		// and "mode >= HeatingMode::off" to determine whether the heater is either active or suspended
		fault,
		offline,
		off,
		suspended,
		heating,
		cooling,
		stable,
		// All states from here onwards must be PID tuning states because function IsTuning assumes that
		tuning0,
		tuning1,
		tuning2,
		tuning3,
		lastTuningMode = tuning3
	};

	virtual void ResetHeater() noexcept = 0;
	virtual HeaterMode GetMode() const noexcept = 0;
	virtual GCodeResult SwitchOn(const StringRef& reply) noexcept = 0;
	virtual GCodeResult UpdateModel(const StringRef& reply) noexcept = 0;
	virtual GCodeResult UpdateFaultDetectionParameters(const StringRef& reply) noexcept = 0;

	int GetSensorNumber() const noexcept { return sensorNumber; }
	void SetSensorNumber(int sn) noexcept { sensorNumber = sn; }
	float GetMaxTemperatureExcursion() const noexcept { return maxTempExcursion; }
	float GetMaxHeatingFaultTime() const noexcept { return maxHeatingFaultTime; }
	float GetTargetTemperature() const noexcept { return (active) ? activeTemperature : standbyTemperature; }
	HeaterProtection *GetHeaterProtections() const noexcept { return heaterProtection; }

	FopDt model;

private:
	bool CheckProtection() const noexcept;			// Check heater protection elements and return true if everything is good

	unsigned int heaterNumber;
	int sensorNumber;								// the sensor number used by this heater
	float activeTemperature;						// The required active temperature
	float standbyTemperature;						// The required standby temperature
	float maxTempExcursion;							// The maximum temperature excursion permitted while maintaining the setpoint
	float maxHeatingFaultTime;						// How long a heater fault is permitted to persist before a heater fault is raised
	HeaterProtection *heaterProtection;				// The first element of assigned heater protection items

	bool active;									// Are we active or standby?
};

#endif /* SRC_HEATING_HEATER_H_ */
