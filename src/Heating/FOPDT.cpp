/*
 * FOPDT.cpp
 *
 *  Created on: 16 Aug 2016
 *      Author: David
 */

#include "FOPDT.h"

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE
# include <Storage/FileStore.h>
#endif

#if SUPPORT_CAN_EXPANSION
# include <CanMessageFormats.h>
#endif

#if SUPPORT_OBJECT_MODEL

// Object model table and functions
// Note: if using GCC version 7.3.1 20180622 and lambda functions are used in this table, you must compile this file with option -std=gnu++17.
// Otherwise the table will be allocated in RAM instead of flash, which wastes too much RAM.

// Macro to build a standard lambda function that includes the necessary type conversions
#define OBJECT_MODEL_FUNC(...)					OBJECT_MODEL_FUNC_BODY(FopDt, __VA_ARGS__)
#define OBJECT_MODEL_FUNC_IF(_condition, ...)	OBJECT_MODEL_FUNC_IF_BODY(FopDt, _condition, __VA_ARGS__)

constexpr ObjectModelTableEntry FopDt::objectModelTable[] =
{
	// Within each group, these entries must be in alphabetical order
	// 0. FopDt members
	{ "coolingExp",			OBJECT_MODEL_FUNC(self->basicModel.coolingRateExponent, 1),							ObjectModelEntryFlags::none },
	{ "coolingRate",		OBJECT_MODEL_FUNC(self->basicModel.basicCoolingRate, 3),							ObjectModelEntryFlags::none },
	{ "deadTime",			OBJECT_MODEL_FUNC(self->basicModel.deadTime, 1),									ObjectModelEntryFlags::none },
	{ "enabled",			OBJECT_MODEL_FUNC(self->enabled),													ObjectModelEntryFlags::none },
	{ "fanCoolingRate",		OBJECT_MODEL_FUNC(self->basicModel.fanCoolingRate, 3),								ObjectModelEntryFlags::none },
	{ "heatingRate",		OBJECT_MODEL_FUNC(self->basicModel.heatingRate, 3),									ObjectModelEntryFlags::none },
	{ "inverted",			OBJECT_MODEL_FUNC(self->inverted),													ObjectModelEntryFlags::none },
	{ "maxPwm",				OBJECT_MODEL_FUNC(self->maxPwm, 2),													ObjectModelEntryFlags::none },
	{ "pid",				OBJECT_MODEL_FUNC(self, 1),															ObjectModelEntryFlags::none },
	{ "standardVoltage",	OBJECT_MODEL_FUNC(self->standardVoltage, 1),										ObjectModelEntryFlags::none },

	// 1. PID members
	{ "d",					OBJECT_MODEL_FUNC(self->loadChangeParams.tD * self->loadChangeParams.kP, 3),		ObjectModelEntryFlags::none },
	{ "i",					OBJECT_MODEL_FUNC(self->loadChangeParams.recipTi * self->loadChangeParams.kP, 4),	ObjectModelEntryFlags::none },
	{ "overridden",			OBJECT_MODEL_FUNC(self->pidParametersOverridden),									ObjectModelEntryFlags::none },
	{ "p",					OBJECT_MODEL_FUNC(self->loadChangeParams.kP, 5),									ObjectModelEntryFlags::none },
	{ "used",				OBJECT_MODEL_FUNC(self->basicModel.usePid),											ObjectModelEntryFlags::none },
};

constexpr uint8_t FopDt::objectModelTableDescriptor[] = { 2, 10, 5 };

DEFINE_GET_OBJECT_MODEL_TABLE(FopDt)

#endif

// The heater model is disabled until the user declares the heater to be a bed, chamber or tool heater
FopDt::FopDt() noexcept
{
	Reset();
}

// Check the model parameters are sensible, if they are then save them and return true. If not then write an error message to reply and return false.
bool FopDt::SetParameters(float phr, float pbcr, float pfcr, float pcrExponent, float pdt, float pMaxPwm, float pVoltage, bool pUsePid, bool pInverted, const StringRef& reply) noexcept
{
	// DC 2017-06-20: allow S down to 0.01 for one of our OEMs (use > 0.0099 because >= 0.01 doesn't work due to rounding error)
	const char *_ecv_array _ecv_null const err =
		  (phr/pbcr < 0.1) ? "estimated temperature rise too small"					// minimum 10C temperature rise (same as with earlier heater model)
		: (pfcr < 0.0) ? "fan reduces cooling rate"
		: (pcrExponent < 1.0 || pcrExponent > 1.6) ? "cooling rate exponent out of range"
		: (pdt <= 0.0099) ? "dead time too small"
		: (pdt * pbcr > 50.0) ? "dead time too close to cooling time constant"		// dead time less than half the cooling time constant
		: (pMaxPwm <= 0.0099 || pMaxPwm > 1.0) ? "PWM out of range"
		: nullptr;
	if (err == nullptr)
	{
		basicModel.heatingRate = phr;
		basicModel.basicCoolingRate = pbcr;
		basicModel.fanCoolingRate = pfcr;
		basicModel.coolingRateExponent = pcrExponent;
		basicModel.deadTime = pdt;
		basicModel.usePid = pUsePid;
		maxPwm = pMaxPwm;
		standardVoltage = pVoltage;
		inverted = pInverted;
		enabled = true;
		CalcPidConstants(100.0);
		return true;
	}
	reply.printf("bad model parameters: %s", err);
	return false;
}

#if SUPPORT_REMOTE_COMMANDS

// Check the model parameters are sensible, if they are then save them and return true. If not then write an error message to reply and return false.
bool FopDt::SetParameters(const CanMessageHeaterModelV2& msg, const StringRef& reply) noexcept
{
	// DC 2017-06-20: allow S down to 0.01 for one of our OEMs (use > 0.0099 because >= 0.01 doesn't work due to rounding error)
	const char *_ecv_array _ecv_null const err =
		  (msg.heatingRate/msg.basicCoolingRate < 0.1) ? "estimated temperature rise too small"					// minimum 10C temperature rise (same as with earlier heater model)
		: (msg.fanCoolingRate < 0.0) ? "fan reduces cooling rate"
		: (msg.coolingRateExponent < 1.0 || msg.coolingRateExponent > 1.6) ? "cooling rate exponent out of range"
		: (msg.deadTime <= 0.0099) ? "dead time too small"
		: (msg.deadTime * msg.basicCoolingRate > 50.0) ? "dead time too close to cooling time constant"			// dead time less than half the cooling time constant
		: (msg.maxPwm <= 0.0099 || msg.maxPwm > 1.0) ? "PWM out of range"
		: nullptr;
	if (err == nullptr)
	{
		basicModel.heatingRate = msg.heatingRate;
		basicModel.basicCoolingRate = msg.basicCoolingRate;
		basicModel.fanCoolingRate = msg.fanCoolingRate;
		basicModel.coolingRateExponent = msg.coolingRateExponent;
		basicModel.deadTime = msg.deadTime;
		basicModel.usePid = msg.usePid;
		maxPwm = msg.maxPwm;
		standardVoltage = msg.standardVoltage;
		inverted = msg.inverted;
		pidParametersOverridden = msg.pidParametersOverridden;

		if (msg.pidParametersOverridden)
		{
			SetRawPidParameters(msg.kP, msg.recipTi, msg.tD);
		}
		else
		{
			CalcPidConstants(100.0);
		}
		enabled = true;
		return true;
	}
	reply.printf("bad model parameters: %s", err);
	return false;
}

#endif

void FopDt::Reset() noexcept
{
	SetDefaultModel(DefaultToolHeaterModel);		// set some values so that we don't report rubbish in the OM
	enabled = false;								// heater is disabled until the parameters are set
}

void FopDt::SetDefaultModel(const HeaterModel& model) noexcept
{
	basicModel = model;
	maxPwm = 1.0;
	standardVoltage = 0.0;
	inverted = pidParametersOverridden = false;
	CalcPidConstants(basicModel.typicalTemperature);
	enabled = true;
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
	SetRawPidParameters(pp.kP * (1.0/255.0), pp.kI/pp.kP, pp.kD/pp.kP);
}

void FopDt::SetRawPidParameters(float p_kP, float p_recipTi, float p_tD) noexcept
{
	loadChangeParams.kP = setpointChangeParams.kP = p_kP;
	loadChangeParams.recipTi = setpointChangeParams.recipTi = p_recipTi;
	loadChangeParams.tD = setpointChangeParams.tD = p_tD;
	pidParametersOverridden = true;
}

// Append a M307 command describing this heater followed by a newline to the string
void FopDt::AppendM307Command(unsigned int heaterNumber, const StringRef& str, bool includeVoltage) const noexcept
{
	str.catf("M307 H%u R%.3f K%.3f:%.3f D%.2f E%.2f S%.2f B%d",
				heaterNumber,
				(double)basicModel.heatingRate,
				(double)basicModel.basicCoolingRate,
				(double)basicModel.fanCoolingRate,
				(double)basicModel.deadTime,
				(double)basicModel.coolingRateExponent,
				(double)maxPwm,
				(basicModel.usePid) ? 0 : 1);
	if (inverted)
	{
		str.cat(" I1");
	}
	if (includeVoltage)
	{
		str.catf(" V%.1f", (double)standardVoltage);
	}
	str.cat('\n');
}

// If PID parameters are overridden, append a M307 command for this heater followed by a newline to the string
void FopDt::AppendM301Command(unsigned int heaterNumber, const StringRef& str) const noexcept
{
	if (pidParametersOverridden)
	{
		const M301PidParameters pp = GetM301PidParameters(false);
		str.catf("M301 H%u P%.1f I%.3f D%.1f\n", heaterNumber, (double)pp.kP, (double)pp.kI, (double)pp.kD);
	}
}

// Append the model parameters to a reply string
void FopDt::AppendModelParameters(unsigned int heaterNumber, const StringRef& str, bool includeVoltage) const noexcept
{
	const char *_ecv_array const mode = (!basicModel.usePid) ? "bang-bang"
								: (pidParametersOverridden) ? "custom PID"
									: "PID";
	str.catf("Heater %u: heating rate %.3f, cooling rate %.3f", heaterNumber, (double)basicModel.heatingRate, (double)basicModel.basicCoolingRate);
	if (basicModel.fanCoolingRate > 0.0)
	{
		str.catf("/%.3f", (double)basicModel.fanCoolingRate);
	}
	str.catf(", dead time %.2f, max PWM %.2f, mode %s", (double)basicModel.deadTime, (double)maxPwm, mode);
	if (inverted)
	{
		str.cat(", reverse control");
	}
	if (includeVoltage)
	{
		str.catf(", calibrated at %.1fV", (double)standardVoltage);
	}
	str.lcatf("Predicted max temperature rise %d" DEGREE_SYMBOL "C", (int)EstimateMaxTemperatureRise());
	if (basicModel.usePid)
	{
		M301PidParameters params = GetM301PidParameters(false);
		str.lcatf("PID parameters: heating P%.1f I%.3f D%.1f", (double)params.kP, (double)params.kI, (double)params.kD);
		params = GetM301PidParameters(true);
		str.catf(", steady P%.1f I%.3f D%.1f", (double)params.kP, (double)params.kI, (double)params.kD);
	}
}

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

void FopDt::CalcPidConstants(float targetTemperature) noexcept
{
	if (!pidParametersOverridden)
	{
		// Calculate the cooling rate per degC at this temperature. We assume the fan is at 20% speed.
		const float temperatureRise = max<float>(targetTemperature - NormalAmbientTemperature, 1.0);		// avoid division by zero!
		const float averageCoolingRatePerDegC = GetCoolingRate(temperatureRise, 0.2)/temperatureRise;
		loadChangeParams.kP = 0.7/(basicModel.heatingRate * basicModel.deadTime);
		loadChangeParams.recipTi = powf(averageCoolingRatePerDegC, 0.25)/(1.14 * powf(basicModel.deadTime, 0.75));		// Ti = 1.14 * timeConstant^0.25 * deadTime^0.75 (Ho et al)
		loadChangeParams.tD = basicModel.deadTime * 0.7;

		setpointChangeParams.kP = 0.7/(basicModel.heatingRate * basicModel.deadTime);
		setpointChangeParams.recipTi = powf(averageCoolingRatePerDegC, 0.5)/powf(basicModel.deadTime, 0.5);			// Ti = timeConstant^0.5 * deadTime^0.5
		setpointChangeParams.tD = basicModel.deadTime * 0.7;
	}
}

// Adjust the actual heater PWM for supply voltage
float FopDt::CorrectPwmForVoltage(float requiredPwm, float actualVoltage) const noexcept
{
	if (requiredPwm < maxPwm && standardVoltage >= 10.0 && actualVoltage >= 10.0)
	{
		requiredPwm *= fsquare(standardVoltage/actualVoltage);
	}
	return min<float>(requiredPwm, maxPwm);
}

float FopDt::GetPwmCorrectionForFan(float temperatureRise, float fanPwmChange) const noexcept
{
	return temperatureRise * 0.01 * basicModel.fanCoolingRate * fanPwmChange / basicModel.heatingRate;
}

// Calculate the expected cooling rate for a given temperature rise above ambient
float FopDt::GetCoolingRate(float temperatureRise, float fanPwm) const noexcept
{
	temperatureRise *= 0.01;
	// If the temperature rise is negative then we must not try to raise it to a non-integral power!
	const float adjustedTemperatureRise = (temperatureRise < 0.0) ? -powf(-temperatureRise, basicModel.coolingRateExponent) : powf(temperatureRise, basicModel.coolingRateExponent);
	return basicModel.basicCoolingRate * adjustedTemperatureRise + temperatureRise * basicModel.fanCoolingRate * fanPwm;
}

// Get an estimate of the expected heating rate at the specified temperature rise and PWM. The result may be negative.
float FopDt::GetNetHeatingRate(float temperatureRise, float fanPwm, float heaterPwm) const noexcept
{
	return basicModel.heatingRate * heaterPwm - GetCoolingRate(temperatureRise, fanPwm);
}

// Get an estimate of the heater PWM required to maintain a specified temperature
float FopDt::EstimateRequiredPwm(float temperatureRise, float fanPwm) const noexcept
{
	return GetCoolingRate(temperatureRise, fanPwm)/basicModel.heatingRate;
}

float FopDt::EstimateMaxTemperatureRise() const noexcept
{
	return EstimateMaxTemperatureRise(basicModel.heatingRate, basicModel.basicCoolingRate, basicModel.coolingRateExponent);
}

/*static*/ float FopDt::EstimateMaxTemperatureRise(float hr, float cr, float cre) noexcept
{
	return 100.0 * powf(hr/cr, 1.0/cre);
}

#if SUPPORT_CAN_EXPANSION

void FopDt::SetupCanMessage(unsigned int heater, CanMessageHeaterModelV2& msg) const noexcept
{
	msg.heater = heater;
	msg.heatingRate = basicModel.heatingRate;
	msg.basicCoolingRate = basicModel.basicCoolingRate;
	msg.fanCoolingRate = basicModel.fanCoolingRate;
	msg.fZero = 0.0;
	msg.coolingRateExponent = basicModel.coolingRateExponent;
	msg.deadTime = basicModel.deadTime;
	msg.maxPwm = maxPwm;
	msg.standardVoltage = standardVoltage;
	msg.enabled = enabled;
	msg.usePid = basicModel.usePid;
	msg.inverted = inverted;
	msg.pidParametersOverridden = pidParametersOverridden;

	msg.kP = setpointChangeParams.kP;
	msg.recipTi = setpointChangeParams.recipTi;
	msg.tD = setpointChangeParams.tD;
}

#endif

// End
