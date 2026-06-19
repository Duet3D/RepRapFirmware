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

#include <RepRapFirmware.h>
#include <ObjectModel/ObjectModel.h>
#include <HeaterModel.h>

#if SUPPORT_CAN_EXPANSION
class StringRef;
class CanMessageHeaterModelV3;
#endif

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

	float EstimateRequiredPwm(float temperatureRise, float fanPwm, float actualVoltage, float filamentPwm) const noexcept;
	float EstimateMaxTemperatureRise() const noexcept;

	float GetExpectedHeatingRate(float temperatureRise, float fanPwm, float heaterPwm, float actualVoltage, float filamentPwm) const noexcept;
	float GetPwmCorrectionForFan(float temperatureRise, float oldFanPwm, float newFanPwm) const noexcept;
	void CalcPidConstants(float targetTemperature) noexcept;
	float CalculateBasicCoolingRate(float temperatureRise, float coolingRate) const noexcept;				// Calculate the basic cooling rate from measurements
	float CalculateFanCoolingRate(float temperatureRise, float coolingRate, float fanPwm) const noexcept;	// Calculate the fan cooling rate from measurements

	void AppendM307Command(unsigned int heaterNumber, const StringRef& str, bool includeVoltage) const noexcept;
	void AppendModelParameters(unsigned int heaterNumber, const StringRef& str, bool includeVoltage) const noexcept;

	// Derived parameters
	M301PidParameters GetM301PidParameters(bool forLoadChange) const noexcept;

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
	static float EstimateMaxTemperatureRise(float hr, float cr, float cre) noexcept;

	HeaterModel basicModel;
	float maxPwm;
	bool enabled;
	bool inverted;

	PidParameters setpointChangeParams;		// parameters for handling changes in the setpoint
	PidParameters loadChangeParams;			// parameters for handling changes in the load
};

// Get an estimate of the expected heating rate at the specified temperature rise and PWM. The result may be negative.
inline float FopDt::GetExpectedHeatingRate(float temperatureRise, float fanPwm, float heaterPwm, float actualVoltage, float filamentPwm) const noexcept
{
	return basicModel.GetExpectedHeatingRate(temperatureRise, fanPwm, heaterPwm, actualVoltage, filamentPwm);
}

// Get an estimate of the heater PWM required to maintain a specified temperature
inline float FopDt::EstimateRequiredPwm(float temperatureRise, float fanPwm, float actualVoltage, float filamentPwm) const noexcept
{
	return basicModel.GetExpectedPwm(temperatureRise, fanPwm, actualVoltage, filamentPwm);
}

inline float FopDt::GetPwmCorrectionForFan(float temperatureRise, float oldFanPwm, float newFanPwm) const noexcept
{
	return basicModel.GetPwmCorrectionForFan(temperatureRise, oldFanPwm, newFanPwm);
}

inline float FopDt::EstimateMaxTemperatureRise() const noexcept
{
	return basicModel.EstimateMaxTemperatureRise();
}

// Calculate the basic cooling rate from measurements
inline float FopDt::CalculateBasicCoolingRate(float temperatureRise, float coolingRate) const noexcept
{
	return basicModel.CalculateBasicCoolingRate(temperatureRise, coolingRate);
}

// Calculate the fan cooling rate from measurements
inline float FopDt::CalculateFanCoolingRate(float temperatureRise, float coolingRate, float fanPwm) const noexcept
{
	return basicModel.CalculateFanCoolingRate(temperatureRise, coolingRate, fanPwm);
}

#endif /* SRC_HEATING_FOPDT_H_ */
