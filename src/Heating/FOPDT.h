/*
 * FOPDT.h
 *
 *  Created on: 16 Aug 2016
 *      Author: David
 *
 *  Class to represent the parameters of a first order process with dead time
 */

#ifndef SRC_HEATING_FOPDT_H_
#define SRC_HEATING_FOPDT_H_

#include "RepRapFirmware.h"
#include "ObjectModel/ObjectModel.h"

// This is how PID parameters are stored internally
struct PidParameters
{
	float kP;			// controller (not model) gain
	float recipTi;		// reciprocal of controller integral time
	float tD;			// controller differential time
};

// This is how PID parameters are given in M301 commands
struct M301PidParameters
{
	float kP;
	float kI;
	float kD;
};

#if HAS_MASS_STORAGE
class FileStore;
#endif

#if SUPPORT_CAN_EXPANSION
struct CanMessageHeaterModelNewNew;
#endif

class FopDt INHERIT_OBJECT_MODEL
{
public:
	FopDt() noexcept;

	void Reset() noexcept;
	bool SetParameters(float phr, float pbcr, float pfcr, float pcrExponent, float pdt, float pMaxPwm, float temperatureLimit, float pVoltage, bool pUsePid, bool pInverted) noexcept;
	void SetDefaultToolParameters() noexcept;
	void SetDefaultBedOrChamberParameters() noexcept;
#if SUPPORT_REMOTE_COMMANDS
	bool SetParameters(const CanMessageHeaterModelNewNew& msg, float temperatureLimit) noexcept;
#endif

	// Stored parameters
	float GetHeatingRate() const noexcept { return heatingRate; }
	float GetBasicCoolingRate() const noexcept { return basicCoolingRate; }
	float GetFanCoolingRate() const noexcept { return fanCoolingRate; }
	float GetCoolingRateExponent() const noexcept { return coolingRateExponent; }
	float GetDeadTime() const noexcept { return deadTime; }
	float GetMaxPwm() const noexcept { return maxPwm; }
	float GetVoltage() const noexcept { return standardVoltage; }
	bool UsePid() const noexcept { return usePid; }
	bool IsInverted() const noexcept { return inverted; }
	bool IsEnabled() const noexcept { return enabled; }

	float EstimateRequiredPwm(float temperatureRise, float fanPwm) const noexcept;
	float EstimateMaxTemperatureRise() const noexcept;

	float GetNetHeatingRate(float temperatureRise, float fanPwm, float heaterPwm) const noexcept;
	float CorrectPwmForVoltage(float requiredPwm, float actualVoltage) const noexcept;
	float GetPwmCorrectionForFan(float temperatureRise, float fanPwmChange) const noexcept;
	void CalcPidConstants(float targetTemperature) noexcept;

	void AppendM307Command(unsigned int heaterNumber, const StringRef& str, bool includeVoltage) const noexcept;
	void AppendM301Command(unsigned int heaterNumber, const StringRef& str) const noexcept;
	void AppendModelParameters(unsigned int heaterNumber, const StringRef& str, bool includeVoltage) const noexcept;

	// Derived parameters
	bool ArePidParametersOverridden() const noexcept { return pidParametersOverridden; }
	M301PidParameters GetM301PidParameters(bool forLoadChange) const noexcept;
	void SetM301PidParameters(const M301PidParameters& params) noexcept;

	const PidParameters& GetPidParameters(bool forLoadChange) const noexcept
	{
		return (forLoadChange) ? loadChangeParams : setpointChangeParams;
	}

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE
//	bool WriteParameters(FileStore *f, size_t heater) const noexcept;		// write the model parameters to file returning true if no error
#endif

#if SUPPORT_CAN_EXPANSION
	void SetupCanMessage(unsigned int heater, CanMessageHeaterModelNewNew& msg) const noexcept;
#endif

protected:
	DECLARE_OBJECT_MODEL

private:
	float GetCoolingRate(float temperatureRise, float fanPwm) const noexcept;
	void SetRawPidParameters(float p_kP, float p_recipTi, float p_tD) noexcept;
	static float EstimateMaxTemperatureRise(float hr, float cr, float cre) noexcept;

	float heatingRate;						// the rate at which the heater heats up at full PWM with no cooling
	float basicCoolingRate;					// the rate at which the heater cools down when it is 100C above ambient and the fan is off
	float fanCoolingRate;					// the additional cooling rate at 100C above ambient with the fan on at full PWM
	float coolingRateExponent;				// how the basic cooling rate varies with temperature difference
	float deadTime;
	float maxPwm;
	float standardVoltage;					// power voltage reading at which tuning was done, or 0 if unknown
	bool enabled;
	bool usePid;
	bool inverted;
	bool pidParametersOverridden;

	PidParameters setpointChangeParams;		// parameters for handling changes in the setpoint
	PidParameters loadChangeParams;			// parameters for handling changes in the load
};

#endif /* SRC_HEATING_FOPDT_H_ */
