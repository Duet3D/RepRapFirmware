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
#include <HeaterModel.h>

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
struct CanMessageHeaterModelV3;
#endif

class FopDt INHERIT_OBJECT_MODEL
{
public:
	FopDt() noexcept;

	void Reset() noexcept;
	bool SetParameters(float phr, float pbcr, float pfcr, float pcrExponent, float pdt, float pMaxPwm, float pVoltage, bool pUsePid, bool pInverted, const StringRef& reply) noexcept;
	void SetDefaultModel(const HeaterModel& model) noexcept;
#if SUPPORT_REMOTE_COMMANDS
	bool SetParameters(const CanMessageHeaterModelV3& msg, const StringRef& reply) noexcept;
	const HeaterModel& GetBasicModel() const noexcept { return basicModel; }
#endif

	// Stored parameters
	float GetHeatingRate() const noexcept { return basicModel.heatingRate; }
	float GetBasicCoolingRate() const noexcept { return basicModel.basicCoolingRate; }
	float GetFanCoolingRate() const noexcept { return basicModel.fanCoolingRate; }
	float GetCoolingRateExponent() const noexcept { return basicModel.coolingRateExponent; }
	float GetDeadTime() const noexcept { return basicModel.deadTime; }
	float GetMaxPwm() const noexcept { return maxPwm; }
	float GetVoltage() const noexcept { return basicModel.standardVoltage; }
	bool UsePid() const noexcept { return basicModel.usePid; }
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
	void SetupCanMessage(unsigned int heater, CanMessageHeaterModelV3& msg) const noexcept;
#endif

protected:
	DECLARE_OBJECT_MODEL

private:
	float GetCoolingRate(float temperatureRise, float fanPwm) const noexcept;
	void SetRawPidParameters(float p_kP, float p_recipTi, float p_tD) noexcept;
	static float EstimateMaxTemperatureRise(float hr, float cr, float cre) noexcept;

	HeaterModel basicModel;
	float maxPwm;
	bool enabled;
	bool inverted;
	bool pidParametersOverridden;

	PidParameters setpointChangeParams;		// parameters for handling changes in the setpoint
	PidParameters loadChangeParams;			// parameters for handling changes in the load
};

#endif /* SRC_HEATING_FOPDT_H_ */
