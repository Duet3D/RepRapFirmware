/*
 * FOPDT.cpp
 *
 *  Created on: 16 Aug 2016
 *      Author: David
 */

#include "FOPDT.h"

#if HAS_MASS_STORAGE
# include "Storage/FileStore.h"
#endif

#if SUPPORT_CAN_EXPANSION
# include <CanMessageFormats.h>
#endif

#if SUPPORT_OBJECT_MODEL

// Object model table and functions
// Note: if using GCC version 7.3.1 20180622 and lambda functions are used in this table, you must compile this file with option -std=gnu++17.
// Otherwise the table will be allocated in RAM instead of flash, which wastes too much RAM.

// Macro to build a standard lambda function that includes the necessary type conversions
#define OBJECT_MODEL_FUNC(...) OBJECT_MODEL_FUNC_BODY(FopDt, __VA_ARGS__)
#define OBJECT_MODEL_FUNC_IF(...) OBJECT_MODEL_FUNC_IF_BODY(FopDt, __VA_ARGS__)

constexpr ObjectModelTableEntry FopDt::objectModelTable[] =
{
	// Within each group, these entries must be in alphabetical order
	// 0. FopDt members
	{ "deadTime",			OBJECT_MODEL_FUNC(self->deadTime, 1),												ObjectModelEntryFlags::none },
	{ "enabled",			OBJECT_MODEL_FUNC(self->enabled),													ObjectModelEntryFlags::none },
	{ "gain",				OBJECT_MODEL_FUNC(self->gain, 1),													ObjectModelEntryFlags::none },
	{ "inverted",			OBJECT_MODEL_FUNC(self->inverted),													ObjectModelEntryFlags::none },
	{ "maxPwm",				OBJECT_MODEL_FUNC(self->maxPwm, 2),													ObjectModelEntryFlags::none },
	{ "pid",				OBJECT_MODEL_FUNC(self, 1),															ObjectModelEntryFlags::none },
	{ "standardVoltage",	OBJECT_MODEL_FUNC(self->standardVoltage, 1),										ObjectModelEntryFlags::none },
	{ "timeConstant",		OBJECT_MODEL_FUNC(self->timeConstant, 1),											ObjectModelEntryFlags::none },

	// 1. PID members
	{ "d",					OBJECT_MODEL_FUNC(self->loadChangeParams.tD * self->loadChangeParams.kP, 1),		ObjectModelEntryFlags::none },
	{ "i",					OBJECT_MODEL_FUNC(self->loadChangeParams.recipTi * self->loadChangeParams.kP, 1),	ObjectModelEntryFlags::none },
	{ "overridden",			OBJECT_MODEL_FUNC(self->pidParametersOverridden),									ObjectModelEntryFlags::none },
	{ "p",					OBJECT_MODEL_FUNC(self->loadChangeParams.kP, 1),									ObjectModelEntryFlags::none },
	{ "used",				OBJECT_MODEL_FUNC(self->usePid),													ObjectModelEntryFlags::none },
};

constexpr uint8_t FopDt::objectModelTableDescriptor[] = { 2, 8, 5 };

DEFINE_GET_OBJECT_MODEL_TABLE(FopDt)

#endif

// Heater 6 on the Duet 0.8.5 is disabled by default at startup so that we can use fan 2.
// Set up sensible defaults here in case the user enables the heater without specifying values for all the parameters.
FopDt::FopDt() noexcept
	: gain(DefaultHotEndHeaterGain), timeConstant(DefaultHotEndHeaterTimeConstant), deadTime(DefaultHotEndHeaterDeadTime), maxPwm(1.0), standardVoltage(0.0),
	  enabled(false), usePid(true), inverted(false), pidParametersOverridden(false)
{
}

// Check the model parameters are sensible, if they are then save them and return true.
bool FopDt::SetParameters(float pg, float ptc, float pdt, float pMaxPwm, float temperatureLimit, float pVoltage, bool pUsePid, bool pInverted) noexcept
{
	if (pg == -1.0 && ptc == -1.0 && pdt == -1.0)
	{
		// Setting all parameters to -1 disables the heater control completely so we can use the pin for other purposes
		enabled = false;
		return true;
	}

	// DC 2017-06-20: allow S down to 0.01 for one of our OEMs (use > 0.0099 because >= 0.01 doesn't work due to rounding error)
	const float maxGain = max<float>(1500.0, temperatureLimit + 500.0);
	if (pg > 10.0 && pg <= maxGain && pdt > 0.099 && ptc >= 2 * pdt && pMaxPwm > 0.0099 && pMaxPwm <= 1.0)
	{
		gain = pg;
		timeConstant = ptc;
		deadTime = pdt;
		maxPwm = pMaxPwm;
		standardVoltage = pVoltage;
		usePid = pUsePid;
		inverted = pInverted;
		enabled = true;
		CalcPidConstants();
		return true;
	}
	return false;
}

// Get the PID parameters as reported by M301
M301PidParameters FopDt::GetM301PidParameters(bool forLoadChange) const noexcept
{
	M301PidParameters rslt;
	const PidParameters& pp = GetPidParameters(forLoadChange);
	const float reportedKp = pp.kP * 255.0;
	rslt.kP = reportedKp;
	rslt.kI = pp.recipTi * reportedKp;
	rslt.kD = pp.tD * reportedKp;
	return rslt;
}

// Override the PID parameters. We set both sets to the same parameters.
void FopDt::SetM301PidParameters(const M301PidParameters& pp) noexcept
{
	loadChangeParams.kP = setpointChangeParams.kP = pp.kP * (1.0/255.0);
	loadChangeParams.recipTi = setpointChangeParams.recipTi = pp.kI/pp.kP;
	loadChangeParams.tD = setpointChangeParams.tD = pp.kD/pp.kP;
	pidParametersOverridden = true;
}

#if HAS_MASS_STORAGE

// Write the model parameters to file returning true if no error
bool FopDt::WriteParameters(FileStore *f, size_t heater) const noexcept
{
	String<StringLength256> scratchString;
	scratchString.printf("M307 H%u A%.1f C%.1f D%.1f S%.2f V%.1f B%d\n",
							heater, (double)gain, (double)timeConstant, (double)deadTime, (double)maxPwm, (double)standardVoltage, (usePid) ? 0 : 1);
	bool ok = f->Write(scratchString.c_str());
	if (ok && pidParametersOverridden)
	{
		const M301PidParameters pp = GetM301PidParameters(false);
		scratchString.printf("M301 H%u P%.1f I%.3f D%.1f\n", heater, (double)pp.kP, (double)pp.kI, (double)pp.kD);
		ok = f->Write(scratchString.c_str());
	}
	return ok;
}

#endif

/* Re-calculate the PID parameters.
 * For some possible formulas, see "Comparison of some well-known PID tuning formulas", Computers and Chemical Engineering 30 (2006) 1416�1423,
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

void FopDt::CalcPidConstants() noexcept
{
	const float timeFrac = deadTime/timeConstant;
	loadChangeParams.kP = 0.7/(gain * timeFrac);
	loadChangeParams.recipTi = (1.0/1.14)/(powf(timeConstant, 0.25) * powf(deadTime, 0.75));	// Ti = 1.14 * timeConstant^0.25 * deadTime^0.75 (Ho et al)
	loadChangeParams.tD = deadTime * 0.7;

	setpointChangeParams.kP = 0.7/(gain * timeFrac);
	setpointChangeParams.recipTi = 1.0/(powf(timeConstant, 0.5) * powf(deadTime, 0.5));			// Ti = timeConstant^0.5 * deadTime^0.5
	setpointChangeParams.tD = deadTime * 0.7;

	pidParametersOverridden = false;
}

#if SUPPORT_CAN_EXPANSION

void FopDt::SetupCanMessage(unsigned int heater, CanMessageUpdateHeaterModel& msg) const noexcept
{
	msg.heater = heater;
	msg.gain = gain;
	msg.timeConstant = timeConstant;
	msg.deadTime = deadTime;
	msg.maxPwm = maxPwm;
	msg.standardVoltage = standardVoltage;
	msg.enabled = enabled;
	msg.usePid = usePid;
	msg.inverted = inverted;
	msg.pidParametersOverridden = pidParametersOverridden;

	msg.kP = setpointChangeParams.kP;
	msg.recipTi = setpointChangeParams.recipTi;
	msg.tD = setpointChangeParams.tD;
}

#endif

// End
