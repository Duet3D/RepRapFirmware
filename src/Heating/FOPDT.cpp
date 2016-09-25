/*
 * FOPDT.cpp
 *
 *  Created on: 16 Aug 2016
 *      Author: David
 */

#include "FOPDT.h"
#include "Core.h"

// Check the model parameters are sensible, if they are then save them and return true.
bool FopDt::SetParameters(float pg, float ptc, float pdt, float pMaxPwm, bool pUsePid)
{
	if (pg > 10.0 && pg < 1500.0 && pdt > 0.1 && ptc > 2 * pdt && pMaxPwm > 0.2 && pMaxPwm <= 1.0)
	{
		gain = pg;
		timeConstant = ptc;
		deadTime = pdt;
		maxPwm = pMaxPwm;
		usePid = pUsePid;
		CalcPidConstants();
		return true;
	}
	return false;
}

/* Re-calculate the PID parameters.
 * For some possible formulas, see "Comparison of some well-known PID tuning formulas", Computers and Chemical Engineering 30 (2006) 1416–1423,
 * available at http://www.ece.ualberta.ca/~marquez/journal_publications_files/papers/tan_cce_06.pdf
 * Here are some examples, where r = td/tc:
 *    Cohen-Coon (modified to use half the original Kc value):
 *     Kc = (0.67/G) * (r + 0.185)
 *     Ti = 2.5 * td * (tc + 0.185 * td)/(tc + 0.611 * td)
 *     Td = 0.37 * td * tc/(tc + 0.185 * td)
 *    Ho et al, best response to setpoint changes:
 *     Kc = (1.086/G) * (r^-0.869
 *     Ti = tc/(0.74 - 0.13 * r)
 *     Td = 0.348 * tc * r^0.914
 *    IAE-setpoint:
 *     Kc = (0.65/G) * r^-1.04432
 *     Ti = tc/(0.9895 + 0.09539 * r)
 *     Td = 0.50814 * tc * r^1.08433
 *    Ho et al, best response to load changes:
 *     Kc = (1.435/G) * r^-0.921
 *     Ti = 1.14 * tc * r^0.749
 *     Td = 0.482 * tc * r^1.137
 *    ITAE-load:
 *     Kc = (0.77902/G) * r^-1.06401
 *     Ti = (tc/1.14311) * r^0.70949
 *     Td = 0.57137 * tc * r^1.03826
 * However, none of these works well in this application. The setpoint-based methods have integral times comparable to the process time
 * constant. This makes them very slow to reach that target temperature. Typically, the power is reduced too soon, so the temperature
 * flattens out too soon it and then it takes a very long time for the integral term to accumulate to the required value. The load-based
 * ones tend to have massive overshoot when the setpoint is changed, and even in the steady state some of them have marginal stability.
 */

void FopDt::CalcPidConstants()
{
	const float timeFrac = deadTime/timeConstant;
	loadChangeParams.kP = 0.7/(gain * timeFrac);
//	loadChangeParams.recipTi = 1.0/(deadTime * 2.0);										// Ti = 2 * deadTime (this is what we used in version 1.15c)
	loadChangeParams.recipTi = (1.0/1.14)/(pow(timeConstant, 0.25) * pow(deadTime, 0.75));	// Ti = 1.14 * timeConstant^0.25 * deadTime^0.75 (Ho et al)
	loadChangeParams.tD = deadTime * 0.7;

	setpointChangeParams.kP = 0.7/(gain * timeFrac);
	setpointChangeParams.recipTi = 1.0/timeConstant;										// Ti = time constant
	setpointChangeParams.tD = deadTime * 0.7;
}

// End
