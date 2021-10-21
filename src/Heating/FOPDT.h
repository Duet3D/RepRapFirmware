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
struct CanMessageUpdateHeaterModelNew;
#endif

class FopDt INHERIT_OBJECT_MODEL
{
public:
	FopDt() noexcept;

	void Clear() noexcept;
	bool SetParameters(float phr, float pcrFanOff, float pcrFanOn, float pdt, float pMaxPwm, float temperatureLimit, float pVoltage, bool pUsePid, bool pInverted) noexcept;
	void SetDefaultToolParameters() noexcept;
	void SetDefaultBedOrChamberParameters() noexcept;

	// Stored parameters
	float GetHeatingRate() const noexcept { return heatingRate; }
	float GetCoolingRateFanOff() const noexcept { return coolingRateFanOff; }
	float GetCoolingRateFanOn() const noexcept { return coolingRateFanOff + coolingRateChangeFanOn; }
	float GetCoolingRateChangeFanOn() const noexcept { return coolingRateChangeFanOn; }
	float GetDeadTime() const noexcept { return deadTime; }
	float GetMaxPwm() const noexcept { return maxPwm; }
	float GetVoltage() const noexcept { return standardVoltage; }
	bool UsePid() const noexcept { return usePid; }
	bool IsInverted() const noexcept { return inverted; }
	bool IsEnabled() const noexcept { return enabled; }

	// Derived parameters
	float GetGainFanOff() const noexcept { return heatingRate/coolingRateFanOff; }
	float GetTimeConstantFanOff() const noexcept { return 1.0/coolingRateFanOff; }
	float GetTimeConstantFanOn() const noexcept { return 1.0/GetCoolingRateFanOn(); }
	bool ArePidParametersOverridden() const noexcept { return pidParametersOverridden; }
	M301PidParameters GetM301PidParameters(bool forLoadChange) const noexcept;
	void SetM301PidParameters(const M301PidParameters& params) noexcept;
	void SetRawPidParameters(float p_kP, float p_recipTi, float p_tD) noexcept;

	const PidParameters& GetPidParameters(bool forLoadChange) const noexcept
	{
		return (forLoadChange) ? loadChangeParams : setpointChangeParams;
	}

#if HAS_MASS_STORAGE || HAS_LINUX_INTERFACE
	bool WriteParameters(FileStore *f, size_t heater) const noexcept;		// erite the model parameters to file returning true if no error
#endif

#if SUPPORT_CAN_EXPANSION
	void SetupCanMessage(unsigned int heater, CanMessageUpdateHeaterModelNew& msg) const noexcept;
#endif

protected:
	DECLARE_OBJECT_MODEL

private:
	void CalcPidConstants() noexcept;

	float heatingRate;
	float coolingRateFanOff;
	float coolingRateChangeFanOn;
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
