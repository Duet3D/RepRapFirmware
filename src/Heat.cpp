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

Heat::Heat(Platform* p) : platform(p), active(false), coldExtrude(false), bedHeater(BED_HEATER), chamberHeater(-1)
{
	for (size_t heater = 0; heater < HEATERS; heater++)
	{
		pids[heater] = new PID(platform, heater);
	}
}

void Heat::Init()
{
	for (size_t heater = 0; heater < HEATERS; heater++)
	{
		pids[heater]->Init();
	}
	lastTime = platform->Time();
	longWait = lastTime;
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
		float t = platform->Time();
		if (t - lastTime >= platform->HeatSampleTime())
		{
			lastTime = t;
			for (size_t heater=0; heater < HEATERS; heater++)
			{
				pids[heater]->Spin();
			}
		}
	}
	platform->ClassReport(longWait);
}

void Heat::Diagnostics(MessageType mtype)
{
	platform->MessageF(mtype, "Heat Diagnostics:\nBed heater = %d, chamber heater = %d\n", bedHeater, chamberHeater);
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
	size_t firstHeater = 	(bedHeater == -1) ? E0_HEATER :
							(includingBed) ? min<int8_t>(bedHeater, E0_HEATER) : E0_HEATER;

	for(size_t heater = firstHeater; heater < HEATERS; heater++)
	{
		if (!HeaterAtSetTemperature(heater))
		{
			return false;
		}
	}
	return true;
}

//query an individual heater
bool Heat::HeaterAtSetTemperature(int8_t heater) const
{
	// If it hasn't anything to do, it must be right wherever it is...
	if (heater < 0 || heater >= HEATERS || pids[heater]->SwitchedOff() || pids[heater]->FaultOccurred())
	{
		return true;
	}

	const float dt = GetTemperature(heater);
	const float target = (pids[heater]->Active()) ? GetActiveTemperature(heater) : GetStandbyTemperature(heater);
	return (target < TEMPERATURE_LOW_SO_DONT_CARE) || (fabs(dt - target) <= TEMPERATURE_CLOSE_ENOUGH);
}

Heat::HeaterStatus Heat::GetStatus(int8_t heater) const
{
	if (heater < 0 || heater >= HEATERS)
	{
		return HS_off;
	}

	return (pids[heater]->FaultOccurred()) ? HS_fault
			: (pids[heater]->SwitchedOff()) ? HS_off
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

float Heat::GetAveragePWM(int8_t heater) const
{
	return pids[heater]->GetAveragePWM();
}

uint32_t Heat::GetLastSampleTime(int8_t heater) const
{
	return pids[heater]->GetLastSampleTime();
}

bool Heat::UseSlowPwm(int8_t heater) const
{
	return heater == bedHeater || heater == chamberHeater;
}

// End
