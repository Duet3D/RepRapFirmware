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

struct PidParams
{
	float kP;			// controller (not model) gain
	float recipTi;		// reciprocal of controller integral time
	float tD;			// controller differential time
};

class FopDt
{
public:
	FopDt();

	bool SetParameters(float pg, float ptc, float pdt, float pMaxPwm, bool pUsePid);

	float GetGain() const { return gain; }
	float GetTimeConstant() const { return timeConstant; }
	float GetDeadTime() const { return deadTime; }
	float GetMaxPwm() const { return maxPwm; }
	bool UsePid() const { return usePid; }
	bool IsEnabled() const { return enabled; }

	const PidParams& GetPidParameters(bool forLoadChange) const
	{
		return (forLoadChange) ? loadChangeParams : setpointChangeParams;
	}

private:
	void CalcPidConstants();

	float gain;
	float timeConstant;
	float deadTime;
	float maxPwm;
	bool usePid;
	bool enabled;

	PidParams setpointChangeParams;		// parameters for handling changes in the setpoint
	PidParams loadChangeParams;			// parameters for handling changes in the load
};

#endif /* SRC_HEATING_FOPDT_H_ */
