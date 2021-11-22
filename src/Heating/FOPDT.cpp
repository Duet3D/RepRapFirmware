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
#define OBJECT_MODEL_FUNC(...) OBJECT_MODEL_FUNC_BODY(FopDt, __VA_ARGS__)
#define OBJECT_MODEL_FUNC_IF(...) OBJECT_MODEL_FUNC_IF_BODY(FopDt, __VA_ARGS__)

constexpr ObjectModelTableEntry FopDt::objectModelTable[] =
{
	// Within each group, these entries must be in alphabetical order
	// 0. FopDt members
	{ "coolingExp",			OBJECT_MODEL_FUNC(self->coolingRateExponent, 1),									ObjectModelEntryFlags::none },
	{ "deadTime",			OBJECT_MODEL_FUNC(self->deadTime, 1),												ObjectModelEntryFlags::none },
	{ "enabled",			OBJECT_MODEL_FUNC(self->enabled),													ObjectModelEntryFlags::none },
//	{ "gain",				OBJECT_MODEL_FUNC(self->GetGainFanOff(), 1),										ObjectModelEntryFlags::none },	// legacy, to be removed
	{ "heatingRate",		OBJECT_MODEL_FUNC(self->heatingRate, 3),											ObjectModelEntryFlags::none },
	{ "inverted",			OBJECT_MODEL_FUNC(self->inverted),													ObjectModelEntryFlags::none },
	{ "maxPwm",				OBJECT_MODEL_FUNC(self->maxPwm, 2),													ObjectModelEntryFlags::none },
	{ "pid",				OBJECT_MODEL_FUNC(self, 1),															ObjectModelEntryFlags::none },
	{ "standardVoltage",	OBJECT_MODEL_FUNC(self->standardVoltage, 1),										ObjectModelEntryFlags::none },
//	{ "timeConstant",		OBJECT_MODEL_FUNC(self->GetTimeConstantFanOff(), 1),								ObjectModelEntryFlags::none },	// legacy, to be removed
//	{ "timeConstantFansOn",	OBJECT_MODEL_FUNC(self->GetTimeConstantFanOn(), 1),									ObjectModelEntryFlags::none },	// legacy, to be removed

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

// The heater model is disabled until the user declares the heater to be a bed, chamber or tool heater
FopDt::FopDt() noexcept
{
	Reset();
}

// Check the model parameters are sensible, if they are then save them and return true.
bool FopDt::SetParameters(float phr, float pbcr, float pfcr, float pcrExponent, float pdt, float pMaxPwm, float temperatureLimit, float pVoltage, bool pUsePid, bool pInverted) noexcept
{
	// DC 2017-06-20: allow S down to 0.01 for one of our OEMs (use > 0.0099 because >= 0.01 doesn't work due to rounding error)
	const float maxTempIncrease = max<float>(1500.0, temperatureLimit + 500.0);
	if (   phr/pbcr > 0.1															// minimum 10C temperature rise (same as with earlier heater model)
		&& EstimateMaxTemperatureRise(phr, pbcr, pcrExponent) <= maxTempIncrease	// max temperature increase within limits
		&& pfcr >= 0.0
		&& pcrExponent >= 1.0
		&& pcrExponent <= 1.6
		&& pdt > 0.099
		&& pdt * pbcr <= 50.0														// dead time less than half cooling time constant
		&& pMaxPwm > 0.0099
		&& pMaxPwm <= 1.0
	   )
	{
		heatingRate = phr;
		basicCoolingRate = pbcr;
		fanCoolingRate = pfcr;
		coolingRateExponent = pcrExponent;
		deadTime = pdt;
		maxPwm = pMaxPwm;
		standardVoltage = pVoltage;
		usePid = pUsePid;
		inverted = pInverted;
		enabled = true;
		CalcPidConstants(100.0);
		return true;
	}
	return false;
}

#if SUPPORT_REMOTE_COMMANDS

bool FopDt::SetParameters(const CanMessageHeaterModelNewNew& msg, float temperatureLimit) noexcept
{
	// DC 2017-06-20: allow S down to 0.01 for one of our OEMs (use > 0.0099 because >= 0.01 doesn't work due to rounding error)
	const float maxTempIncrease = max<float>(1500.0, temperatureLimit + 500.0);
	if (   msg.heatingRate/msg.basicCoolingRate > 0.1								// minimum 10C temperature rise (same as with earlier heater model)
		&& EstimateMaxTemperatureRise(msg.heatingRate, msg.basicCoolingRate, msg.coolingRateExponent) <= maxTempIncrease
																					// max temperature increase within limits
		&& msg.fanCoolingRate >= 0.0
		&& msg.coolingRateExponent >= 1.0
		&& msg.coolingRateExponent <= 1.6
		&& msg.deadTime > 0.099
		&& msg.deadTime * msg.basicCoolingRate <= 50.0								// dead time less than half the cooling time constant
		&& msg.maxPwm > 0.0099
		&& msg.maxPwm <= 1.0
	   )
	{
		heatingRate = msg.heatingRate;
		basicCoolingRate = msg.basicCoolingRate;
		fanCoolingRate = msg.fanCoolingRate;
		coolingRateExponent = msg.coolingRateExponent;
		deadTime = msg.deadTime;
		maxPwm = msg.maxPwm;
		standardVoltage = msg.standardVoltage;
		usePid = msg.usePid;
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
	return false;
}

#endif

void FopDt::Reset() noexcept
{
	SetDefaultToolParameters();						// set some values so that we don't report rubbish in the OM
	enabled = false;								// heater is disabled until the parameters are set
}

// Set up default parameters for a tool heater and enable the model
void FopDt::SetDefaultToolParameters() noexcept
{
	heatingRate = DefaultToolHeaterHeatingRate;
	basicCoolingRate = DefaultToolHeaterBasicCoolingRate;
	deadTime = DefaultToolHeaterDeadTime;
	fanCoolingRate = 0.0;
	coolingRateExponent = DefaultToolHeaterCoolingRateExponent;
	maxPwm = 1.0;
	standardVoltage = 0.0;
	usePid = true;
	inverted = pidParametersOverridden = false;
	CalcPidConstants(200.0);
	enabled = true;
}

// Set up default parameters for a bed/chamber heater and enable the model
void FopDt::SetDefaultBedOrChamberParameters() noexcept
{
	heatingRate = DefaultBedHeaterHeatingRate;
	basicCoolingRate = DefaultBedHeaterBasicCoolingRate;
	deadTime = DefaultBedHeaterDeadTime;
	fanCoolingRate = 0.0;
	coolingRateExponent = DefaultBedHeaterCoolingRateExponent;
	maxPwm = 1.0;
	standardVoltage = 0.0;
	usePid = false;
	inverted = pidParametersOverridden = false;
	CalcPidConstants(60.0);
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
				(double)heatingRate,
				(double)basicCoolingRate,
				(double)fanCoolingRate,
				(double)deadTime,
				(double)coolingRateExponent,
				(double)maxPwm,
				(usePid) ? 0 : 1);
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
	const char* const mode = (!usePid) ? "bang-bang"
								: (pidParametersOverridden) ? "custom PID"
									: "PID";
	str.catf("Heater %u: heating rate %.3f, cooling rate %.3f", heaterNumber, (double)heatingRate, (double)basicCoolingRate);
	if (fanCoolingRate > 0.0)
	{
		str.catf("/%.3f", (double)fanCoolingRate);
	}
	str.catf(", dead time %.2f, max PWM %.2f, mode %s", (double)deadTime, (double)maxPwm, mode);
	if (inverted)
	{
		str.cat(", reverse control");
	}
	if (includeVoltage)
	{
		str.catf(", calibrated at %.1fV", (double)standardVoltage);
	}
	str.lcatf("Predicted max temperature rise %d" DEGREE_SYMBOL "C", (int)EstimateMaxTemperatureRise());
	if (usePid)
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
		const float averageCoolingRate = GetCoolingRate(targetTemperature - NormalAmbientTemperature, 0.2);
		loadChangeParams.kP = 0.7/(heatingRate * deadTime);
		loadChangeParams.recipTi = powf(averageCoolingRate, 0.25)/(1.14 * powf(deadTime, 0.75));	// Ti = 1.14 * timeConstant^0.25 * deadTime^0.75 (Ho et al)
		loadChangeParams.tD = deadTime * 0.7;

		setpointChangeParams.kP = 0.7/(heatingRate * deadTime);
		setpointChangeParams.recipTi = powf(averageCoolingRate, 0.5)/powf(deadTime, 0.5);			// Ti = timeConstant^0.5 * deadTime^0.5
		setpointChangeParams.tD = deadTime * 0.7;
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
	return temperatureRise * 0.01 * fanCoolingRate / heatingRate;
}

// Calculate the expected cooling rate for a given temperature rise above ambient
float FopDt::GetCoolingRate(float temperatureRise, float fanPwm) const noexcept
{
	temperatureRise *= 0.01;
	return basicCoolingRate * powf(temperatureRise, coolingRateExponent) + temperatureRise * fanCoolingRate * fanPwm;
}

// Get an estimate of the expected heating rate at the specified temperature rise and PWM. The result may be negative.
float FopDt::GetNetHeatingRate(float temperatureRise, float fanPwm, float heaterPwm) const noexcept
{
	return heatingRate * heaterPwm - GetCoolingRate(temperatureRise, fanPwm);
}

// Get an estimate of the heater PWM required to maintain a specified temperature
float FopDt::EstimateRequiredPwm(float temperatureRise, float fanPwm) const noexcept
{
	return GetCoolingRate(temperatureRise, fanPwm)/heatingRate;
}

float FopDt::EstimateMaxTemperatureRise() const noexcept
{
	return EstimateMaxTemperatureRise(heatingRate, basicCoolingRate, coolingRateExponent);
}

/*static*/ float FopDt::EstimateMaxTemperatureRise(float hr, float cr, float cre) noexcept
{
	return 100.0 * powf(hr/cr, 1.0/cre);
}

#if SUPPORT_CAN_EXPANSION

void FopDt::SetupCanMessage(unsigned int heater, CanMessageHeaterModelNewNew& msg) const noexcept
{
	msg.heater = heater;
	msg.heatingRate = heatingRate;
	msg.basicCoolingRate = basicCoolingRate;
	msg.fanCoolingRate = fanCoolingRate;
	msg.fZero = 0.0;
	msg.coolingRateExponent = coolingRateExponent;
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
