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
struct CanMessageUpdateHeaterModel;
#endif

class FopDt INHERIT_OBJECT_MODEL
{
public:
	FopDt() noexcept;

	bool SetParameters(float pg, float ptc, float pdt, float pMaxPwm, float temperatureLimit, float pVoltage, bool pUsePid, bool pInverted) noexcept;

	float GetGain() const noexcept { return gain; }
	float GetTimeConstant() const noexcept { return timeConstant; }
	float GetDeadTime() const noexcept { return deadTime; }
	float GetMaxPwm() const noexcept { return maxPwm; }
	float GetVoltage() const noexcept { return standardVoltage; }
	bool UsePid() const noexcept { return usePid; }
	bool IsInverted() const noexcept { return inverted; }
	bool IsEnabled() const noexcept { return enabled; }
	bool ArePidParametersOverridden() const noexcept { return pidParametersOverridden; }
	M301PidParameters GetM301PidParameters(bool forLoadChange) const noexcept;
	void SetM301PidParameters(const M301PidParameters& params) noexcept;

	const PidParameters& GetPidParameters(bool forLoadChange) const noexcept
	{
		return (forLoadChange) ? loadChangeParams : setpointChangeParams;
	}

#if HAS_MASS_STORAGE
	bool WriteParameters(FileStore *f, size_t heater) const noexcept;		// erite the model parameters to file returning true if no error
#endif

#if SUPPORT_CAN_EXPANSION
	void SetupCanMessage(unsigned int heater, CanMessageUpdateHeaterModel& msg) const noexcept;
#endif

protected:
	DECLARE_OBJECT_MODEL

private:
	void CalcPidConstants() noexcept;

	float gain;
	float timeConstant;
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
