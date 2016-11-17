/****************************************************************************************************

RepRapFirmware - Heat

This is all the code to deal with heat and temperature.

-----------------------------------------------------------------------------------------------------

Version 0.1

18 November 2012

Adrian Bowyer
RepRap Professional Ltd
http://reprappro.com

Licence: GPL

****************************************************************************************************/

#include "RepRapFirmware.h"
#include "Pid.h"

Heat::Heat(Platform* p)
	: platform(p), active(false), coldExtrude(false), bedHeater(DefaultBedHeater), chamberHeater(-1), heaterBeingTuned(-1), lastHeaterTuned(-1)
{
	for (size_t heater = 0; heater < HEATERS; heater++)
	{
		pids[heater] = new PID(platform, heater);
	}
}

void Heat::Init()
{
	for (int heater = 0; heater < HEATERS; heater++)
	{
		if (heater == bedHeater || heater == chamberHeater)
		{
			pids[heater]->Init(DefaultBedHeaterGain, DefaultBedHeaterTimeConstant, DefaultBedHeaterDeadTime, DefaultBedTemperatureLimit, false);
		}
#ifndef DUET_NG
		else if (heater == HEATERS - 1)
		{
			// Heater 6 pin is shared with fan 1. By default we support fan 1, so disable heater 6.
			pids[heater]->Init(-1.0, -1.0, -1.0, DefaultExtruderTemperatureLimit, true);
		}
#endif
		else
		{
			pids[heater]->Init(DefaultHotEndHeaterGain, DefaultHotEndHeaterTimeConstant, DefaultHotEndHeaterDeadTime, DefaultExtruderTemperatureLimit, true);
		}
	}
	lastTime = millis() - platform->HeatSampleInterval();		// flag the PIDS as due for spinning
	longWait = platform->Time();
	coldExtrude = false;
	active = true;
}

void Heat::Exit()
{
	for (size_t heater = 0; heater < HEATERS; heater++)
	{
		pids[heater]->SwitchOff();
	}
	platform->Message(HOST_MESSAGE, "Heat class exited.\n");
	active = false;
}

void Heat::Spin()
{
	if (active)
	{
		// See if it is time to spin the PIDs
		const uint32_t now = millis();
		if (now - lastTime >= platform->HeatSampleInterval())
		{
			lastTime = now;
			for (size_t heater=0; heater < HEATERS; heater++)
			{
				pids[heater]->Spin();
			}

			// See if we have finished tuning a PID
			if (heaterBeingTuned != -1 && !pids[heaterBeingTuned]->IsTuning())
			{
				lastHeaterTuned = heaterBeingTuned;
				heaterBeingTuned = -1;
			}
		}
	}
	platform->ClassReport(longWait);
}

void Heat::Diagnostics(MessageType mtype)
{
	platform->MessageF(mtype, "=== Heat ===\nBed heater = %d, chamber heater = %d\n", bedHeater, chamberHeater);
	for (size_t heater=0; heater < HEATERS; heater++)
	{
		if (pids[heater]->Active())
		{
			platform->MessageF(mtype, "Heater %d is on, I-accum = %.1f\n", heater, pids[heater]->GetAccumulator());
		}
	}
}

bool Heat::AllHeatersAtSetTemperatures(bool includingBed) const
{
	for(int8_t heater = 0; heater < HEATERS; heater++)
	{
		if (!HeaterAtSetTemperature(heater, true) && (includingBed || heater != bedHeater))
		{
			return false;
		}
	}
	return true;
}

//query an individual heater
bool Heat::HeaterAtSetTemperature(int8_t heater, bool waitWhenCooling) const
{
	// If it hasn't anything to do, it must be right wherever it is...
	if (heater < 0 || heater >= HEATERS || pids[heater]->SwitchedOff() || pids[heater]->FaultOccurred())
	{
		return true;
	}

	const float dt = GetTemperature(heater);
	const float target = (pids[heater]->Active()) ? GetActiveTemperature(heater) : GetStandbyTemperature(heater);
	return (target < TEMPERATURE_LOW_SO_DONT_CARE)
		|| (fabsf(dt - target) <= TEMPERATURE_CLOSE_ENOUGH)
		|| (target < dt && !waitWhenCooling);
}

Heat::HeaterStatus Heat::GetStatus(int8_t heater) const
{
	if (heater < 0 || heater >= HEATERS)
	{
		return HS_off;
	}

	return (pids[heater]->FaultOccurred()) ? HS_fault
			: (pids[heater]->SwitchedOff()) ? HS_off
				: (pids[heater]->IsTuning()) ? HS_tuning
					: (pids[heater]->Active()) ? HS_active
						: HS_standby;
}

void Heat::SetActiveTemperature(int8_t heater, float t)
{
	if (heater >= 0 && heater < HEATERS)
	{
		pids[heater]->SetActiveTemperature(t);
	}
}

float Heat::GetActiveTemperature(int8_t heater) const
{
	return (heater >= 0 && heater < HEATERS) ? pids[heater]->GetActiveTemperature() : ABS_ZERO;
}

void Heat::SetStandbyTemperature(int8_t heater, float t)
{
	if (heater >= 0 && heater < HEATERS)
	{
		pids[heater]->SetStandbyTemperature(t);
	}
}

float Heat::GetStandbyTemperature(int8_t heater) const
{
	return (heater >= 0 && heater < HEATERS) ? pids[heater]->GetStandbyTemperature() : ABS_ZERO;
}

void Heat::SetTemperatureLimit(int8_t heater, float t)
{
	if (heater >= 0 && heater < HEATERS)
	{
		pids[heater]->SetTemperatureLimit(t);
	}
}

float Heat::GetTemperatureLimit(int8_t heater) const
{
	return (heater >= 0 && heater < HEATERS) ? pids[heater]->GetTemperatureLimit() : ABS_ZERO;
}

float Heat::GetTemperature(int8_t heater) const
{
	return (heater >= 0 && heater < HEATERS) ? pids[heater]->GetTemperature() : ABS_ZERO;
}

void Heat::Activate(int8_t heater)
{
	if (heater >= 0 && heater < HEATERS)
	{
		pids[heater]->Activate();
	}
}

void Heat::SwitchOff(int8_t heater)
{
	if (heater >= 0 && heater < HEATERS)
	{
		pids[heater]->SwitchOff();
	}
}

void Heat::SwitchOffAll()
{
	for (size_t heater = 0; heater < HEATERS; ++heater)
	{
		pids[heater]->SwitchOff();
	}
}

void Heat::Standby(int8_t heater)
{
	if (heater >= 0 && heater < HEATERS)
	{
		pids[heater]->Standby();
	}
}

void Heat::ResetFault(int8_t heater)
{
	if (heater >= 0 && heater < HEATERS)
	{
		pids[heater]->ResetFault();
	}
}

float Heat::GetAveragePWM(size_t heater) const
{
	return pids[heater]->GetAveragePWM();
}

uint32_t Heat::GetLastSampleTime(size_t heater) const
{
	return pids[heater]->GetLastSampleTime();
}

bool Heat::UseSlowPwm(int8_t heater) const
{
	return heater == bedHeater || heater == chamberHeater;
}

// Auto tune a PID
void Heat::StartAutoTune(size_t heater, float temperature, float maxPwm, StringRef& reply)
{
	if (heaterBeingTuned == -1)
	{
		heaterBeingTuned = (int8_t)heater;
		pids[heater]->StartAutoTune(temperature, maxPwm, reply);
	}
	else
	{
		// Trying to start a new auto tune, but we are already tuning a heater
		reply.printf("Error: cannot start auto tuning heater %u because heater %d is being tuned", heater, heaterBeingTuned);
	}
}

bool Heat::IsTuning(size_t heater) const
{
	return pids[heater]->IsTuning();
}

void Heat::GetAutoTuneStatus(StringRef& reply) const
{
	int8_t whichPid = (heaterBeingTuned == -1) ? lastHeaterTuned : heaterBeingTuned;
	if (whichPid != -1)
	{
		pids[whichPid]->GetAutoTuneStatus(reply);
	}
	else
	{
		reply.copy("No heater has been tuned yet");
	}
}

// Get the highest temperature limit of any heater
float Heat::GetHighestTemperatureLimit() const
{
	float limit = ABS_ZERO;
	for (size_t h = 0; h < HEATERS; ++h)
	{
		if (h < reprap.GetToolHeatersInUse() || (int)h == bedHeater || (int)h == chamberHeater)
		{
			const float t = pids[h]->GetTemperatureLimit();
			if (t > limit)
			{
				limit = t;
			}
		}
	}
	return limit;
}

// End
