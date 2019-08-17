/*
 * Heater.h
 *
 *  Created on: 24 Jul 2019
 *      Author: David
 */

#ifndef SRC_HEATING_HEATER_H_
#define SRC_HEATING_HEATER_H_

#include "RepRapFirmware.h"
#include "FOPDT.h"
#include "GCodes/GCodeResult.h"

#if SUPPORT_CAN_EXPANSION
# include "CanId.h"
#endif

class HeaterProtection;

// Enumeration to describe the status of a heater. Note that the web interface returns the numerical values, so don't change them.
// Status 'running' is not returned to the web interface, we return active or standby instead.
enum class HeaterStatus { off = 0, standby = 1, active = 2, fault = 3, tuning = 4, running = 5 };

class Heater
{
public:
	Heater(unsigned int num);
	virtual ~Heater();

	virtual float GetTemperature() const = 0;					// Get the current temperature
	virtual float GetAveragePWM() const = 0;					// Return the running average PWM to the heater. Answer is a fraction in [0, 1].
	virtual void ResetFault() = 0;								// Reset a fault condition - only call this if you know what you are doing
	virtual void SwitchOff() = 0;
	virtual void Spin() = 0;
	virtual void StartAutoTune(float targetTemp, float maxPwm, const StringRef& reply) = 0;	// Start an auto tune cycle for this PID
	virtual void GetAutoTuneStatus(const StringRef& reply) const = 0;	// Get the auto tune status or last result
	virtual void Suspend(bool sus) = 0;							// Suspend the heater to conserve power or while doing Z probing
	virtual float GetAccumulator() const = 0;					// Get the inertial term accumulator
	virtual GCodeResult ConfigurePortAndSensor(GCodeBuffer& gb, const StringRef& reply) = 0;
	virtual void ReleasePort() = 0;								// If it's a local heater, turn it off and release its port. If it is remote, delete the remote heater.

	unsigned int GetHeaterNumber() const { return heaterNumber; }
	const char *GetSensorName() const;							// Get the name of the sensor for this heater, or nullptr if it hasn't been named
	HeaterStatus GetStatus() const;								// Get the status of the heater
	void SetActiveTemperature(float t);
	float GetActiveTemperature() const { return activeTemperature; }
	void SetStandbyTemperature(float t);
	float GetStandbyTemperature() const { return standbyTemperature; }
	void Activate();											// Switch from idle to active
	void Standby();												// Switch from active to idle

	void GetFaultDetectionParameters(float& pMaxTempExcursion, float& pMaxFaultTime) const
		{ pMaxTempExcursion = maxTempExcursion; pMaxFaultTime = maxHeatingFaultTime; }

	void SetFaultDetectionParameters(float pMaxTempExcursion, float pMaxFaultTime)
		{ maxTempExcursion = pMaxTempExcursion; maxHeatingFaultTime = pMaxFaultTime; }

	float GetHighestTemperatureLimit() const;					// Get the highest temperature limit
	float GetLowestTemperatureLimit() const;					// Get the lowest temperature limit
	void SetHeaterProtection(HeaterProtection *h);

	const FopDt& GetModel() const { return model; }				// Get the process model

	GCodeResult SetModel(float gain, float tc, float td, float maxPwm, float voltage, bool usePid, bool inverted, const StringRef& reply);	// Set the process model

	bool IsHeaterEnabled() const								// Is this heater enabled?
		{ return model.IsEnabled(); }

	void SetM301PidParameters(const M301PidParameters& params)
		{ model.SetM301PidParameters(params); }

	TemperatureSensor *GetSensor() const;
	bool CheckGood() const;

protected:
	enum class HeaterMode : uint8_t
	{
		// The order of these is important because we test "mode > HeatingMode::suspended" to determine whether the heater is active
		// and "mode >= HeatingMode::off" to determine whether the heater is either active or suspended
		fault,
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

	virtual void ResetHeater() = 0;
	virtual HeaterMode GetMode() const = 0;
	virtual void SwitchOn() = 0;
	virtual GCodeResult UpdateModel(const StringRef& reply) = 0;

	int GetSensorNumber() const { return sensorNumber; }
	void SetSensorNumber(int sn) { sensorNumber = sn; }
	float GetMaxTemperatureExcursion() const { return maxTempExcursion; }
	float GetMaxHeatingFaultTime() const { return maxHeatingFaultTime; }
	float GetTargetTemperature() const { return (active) ? activeTemperature : standbyTemperature; }
	HeaterProtection *GetHeaterProtections() const { return heaterProtection; }

	FopDt model;

private:
	bool CheckProtection() const;					// Check heater protection elements and return true if everything is good

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
