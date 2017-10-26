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

#include "Heat.h"
#include "Platform.h"
#include "RepRap.h"
#include "Sensors/TemperatureSensor.h"

#if SUPPORT_DHT_SENSOR
# include "Sensors/DhtSensor.h"
#endif

Heat::Heat(Platform& p)
	: platform(p), active(false), coldExtrude(false), bedHeater(DefaultBedHeater), chamberHeater(DefaultChamberHeater), heaterBeingTuned(-1), lastHeaterTuned(-1)
{
	for (size_t heater : ARRAY_INDICES(pids))
	{
		pids[heater] = new PID(platform, heater);
	}
}

// Reset all heater models to defaults. Called when running M502.
void Heat::ResetHeaterModels()
{
	for (size_t heater : ARRAY_INDICES(pids))
	{
		if (pids[heater]->IsHeaterEnabled())
		{
			if ((int)heater == DefaultBedHeater || (int)heater == DefaultChamberHeater)
			{
				pids[heater]->SetModel(DefaultBedHeaterGain, DefaultBedHeaterTimeConstant, DefaultBedHeaterDeadTime, 1.0, false);
			}
			else
			{
				pids[heater]->SetModel(DefaultHotEndHeaterGain, DefaultHotEndHeaterTimeConstant, DefaultHotEndHeaterDeadTime, 1.0, true);
			}
		}
	}
}

void Heat::SetBedHeater(int8_t heater)
{
	if (bedHeater >= 0)
	{
		pids[bedHeater]->SwitchOff();
	}
	bedHeater = heater;
}

void Heat::SetChamberHeater(int8_t heater)
{
	if (chamberHeater >= 0)
	{
		pids[chamberHeater]->SwitchOff();
	}
	chamberHeater = heater;
}

void Heat::Init()
{
	// Set up the real heaters and the corresponding PIDs
	for (size_t heater = 0; heater < Heaters; ++heater)
	{
		heaterSensors[heater] = nullptr;			// no temperature sensor assigned yet
		if ((int)heater == DefaultBedHeater || (int)heater == DefaultChamberHeater)
		{
			pids[heater]->Init(DefaultBedHeaterGain, DefaultBedHeaterTimeConstant, DefaultBedHeaterDeadTime, DefaultBedTemperatureLimit, false);
		}
#if defined(DUET_06_085)
		else if (heater == Heaters - 1)
		{
			// On the Duet 085, the heater 6 pin is also the fan 1 pin. By default we support fan 1, so disable heater 6.
			pids[heater]->Init(-1.0, -1.0, -1.0, DefaultExtruderTemperatureLimit, true);
		}
#endif
		else
		{
			pids[heater]->Init(DefaultHotEndHeaterGain, DefaultHotEndHeaterTimeConstant, DefaultHotEndHeaterDeadTime, DefaultExtruderTemperatureLimit, true);
		}
		lastStandbyTools[heater] = nullptr;
	}

	// Set up the virtual heaters
	// Clear the user-defined virtual heaters
	for (TemperatureSensor* &v : virtualHeaterSensors)
	{
		v = nullptr;
	}

	// Set up default virtual heaters for MCU temperature and TMC driver overheat sensors
#if HAS_CPU_TEMP_SENSOR
	virtualHeaterSensors[0] = TemperatureSensor::Create(CpuTemperatureSenseChannel);
	virtualHeaterSensors[0]->SetHeaterName("MCU");				// name this virtual heater so that it appears in DWC
#endif
#if HAS_SMART_DRIVERS
	virtualHeaterSensors[1] = TemperatureSensor::Create(FirstTmcDriversSenseChannel);
	virtualHeaterSensors[2] = TemperatureSensor::Create(FirstTmcDriversSenseChannel + 1);
#endif

	lastTime = millis() - platform.HeatSampleInterval();		// flag the PIDS as due for spinning
	longWait = millis();
	coldExtrude = false;
	active = true;
}

void Heat::Exit()
{
	for (PID *pid : pids)
	{
		pid->SwitchOff();
	}
	active = false;
}

void Heat::Spin()
{
	if (active)
	{
		// See if it is time to spin the PIDs
		const uint32_t now = millis();
		if (now - lastTime >= platform.HeatSampleInterval())
		{
			lastTime = now;
			for (size_t heater=0; heater < Heaters; heater++)
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

#if SUPPORT_DHT_SENSOR
		// If the DHT temperature sensor is active, it needs to be spinned too
		DhtSensor::Spin();
#endif
	}
	platform.ClassReport(longWait);
}

void Heat::Diagnostics(MessageType mtype)
{
	platform.MessageF(mtype, "=== Heat ===\nBed heater = %d, chamber heater = %d\n", bedHeater, chamberHeater);
	for (size_t heater : ARRAY_INDICES(pids))
	{
		if (pids[heater]->Active())
		{
			platform.MessageF(mtype, "Heater %d is on, I-accum = %.1f\n", heater, (double)(pids[heater]->GetAccumulator()));
		}
	}
}

bool Heat::AllHeatersAtSetTemperatures(bool includingBed) const
{
	for (size_t heater : ARRAY_INDICES(pids))
	{
		if (!HeaterAtSetTemperature(heater, true) && (includingBed || (int)heater != bedHeater))
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
	if (heater < 0 || heater >= (int)Heaters || pids[heater]->SwitchedOff() || pids[heater]->FaultOccurred())
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
	if (heater < 0 || heater >= (int)Heaters)
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
	if (heater >= 0 && heater < (int)Heaters)
	{
		pids[heater]->SetActiveTemperature(t);
	}
}

float Heat::GetActiveTemperature(int8_t heater) const
{
	return (heater >= 0 && heater < (int)Heaters) ? pids[heater]->GetActiveTemperature() : ABS_ZERO;
}

void Heat::SetStandbyTemperature(int8_t heater, float t)
{
	if (heater >= 0 && heater < (int)Heaters)
	{
		pids[heater]->SetStandbyTemperature(t);
	}
}

float Heat::GetStandbyTemperature(int8_t heater) const
{
	return (heater >= 0 && heater < (int)Heaters) ? pids[heater]->GetStandbyTemperature() : ABS_ZERO;
}

void Heat::SetTemperatureLimit(int8_t heater, float t)
{
	if (heater >= 0 && heater < (int)Heaters)
	{
		pids[heater]->SetTemperatureLimit(t);
	}
}

float Heat::GetTemperatureLimit(int8_t heater) const
{
	return (heater >= 0 && heater < (int)Heaters) ? pids[heater]->GetTemperatureLimit() : ABS_ZERO;
}

// Get the current temperature of a heater
float Heat::GetTemperature(int8_t heater) const
{
	return (heater >= 0 && heater < (int)Heaters) ? pids[heater]->GetTemperature() : ABS_ZERO;
}

// Get the target temperature of a heater
float Heat::GetTargetTemperature(int8_t heater) const
{
	const Heat::HeaterStatus hs = GetStatus(heater);
	return (hs == HS_active) ? GetActiveTemperature(heater)
			: (hs == HS_standby) ? GetStandbyTemperature(heater)
				: 0.0;
}

void Heat::Activate(int8_t heater)
{
	if (heater >= 0 && heater < (int)Heaters)
	{
		pids[heater]->Activate();
	}
}

void Heat::SwitchOff(int8_t heater)
{
	if (heater >= 0 && heater < (int)Heaters)
	{
		pids[heater]->SwitchOff();
		lastStandbyTools[heater] = nullptr;
	}
}

void Heat::SwitchOffAll()
{
	for (PID *p : pids)
	{
		p->SwitchOff();
	}
}

void Heat::Standby(int8_t heater, const Tool *tool)
{
	if (heater >= 0 && heater < (int)Heaters)
	{
		pids[heater]->Standby();
		lastStandbyTools[heater] = tool;
	}
}

void Heat::ResetFault(int8_t heater)
{
	if (heater >= 0 && heater < (int)Heaters)
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
	for (size_t h : ARRAY_INDICES(pids))
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

// Override the model-generated PID parameters
void Heat::SetM301PidParameters(size_t heater, const M301PidParameters& params)
{
	pids[heater]->SetM301PidParameters(params);
}

// Write heater model parameters to file returning true if no error
bool Heat::WriteModelParameters(FileStore *f) const
{
	bool ok = f->Write("; Heater model parameters\n");
	for (size_t h : ARRAY_INDICES(pids))
	{
		const FopDt& model = pids[h]->GetModel();
		if (model.IsEnabled())
		{
			ok = model.WriteParameters(f, h);
		}
	}
	return ok;
}

// Return the channel used by a particular heater, or -1 if not configured
int Heat::GetHeaterChannel(size_t heater) const
{
	const TemperatureSensor * const * const spp = GetSensor(heater);
	return (spp != nullptr && *spp != nullptr) ? (*spp)->GetSensorChannel() : -1;
}

// Set the channel used by a heater, returning true if bad heater or channel number
bool Heat::SetHeaterChannel(size_t heater, int channel)
{
	TemperatureSensor ** const spp = GetSensor(heater);
	if (spp == nullptr)
	{
		return true;		// bad heater number
	}

	TemperatureSensor *sp = TemperatureSensor::Create(channel);
	if (sp == nullptr)
	{
		return true;		// bad channel number
	}

	delete *spp;			// release the old sensor object, if any
	*spp = sp;
	return false;
}

// Configure the temperature sensor for a channel
bool Heat::ConfigureHeaterSensor(unsigned int mcode, size_t heater, GCodeBuffer& gb, StringRef& reply, bool& error)
{
	TemperatureSensor ** const spp = GetSensor(heater);
	if (spp == nullptr || *spp == nullptr)
	{
		reply.printf("heater %d is not configured", heater);
		error = true;
		return false;
	}

	return (*spp)->Configure(mcode, heater, gb, reply, error);
}

// Get a pointer to the temperature sensor entry, or nullptr if the heater number is bad
TemperatureSensor **Heat::GetSensor(size_t heater)
{
	if (heater < Heaters)
	{
		return &heaterSensors[heater];
	}
	if (heater >= FirstVirtualHeater && heater < FirstVirtualHeater + ARRAY_SIZE(virtualHeaterSensors))
	{
		return &virtualHeaterSensors[heater - FirstVirtualHeater];
	}
	return nullptr;
}

// Get a pointer to the temperature sensor entry, or nullptr if the heater number is bad (const version of above)
TemperatureSensor * const *Heat::GetSensor(size_t heater) const
{
	if (heater < Heaters)
	{
		return &heaterSensors[heater];
	}
	if (heater >= FirstVirtualHeater && heater < FirstVirtualHeater + ARRAY_SIZE(virtualHeaterSensors))
	{
		return &virtualHeaterSensors[heater - FirstVirtualHeater];
	}
	return nullptr;
}

// Get the name of a heater, or nullptr if it hasn't been named
const char *Heat::GetHeaterName(size_t heater) const
{
	const TemperatureSensor * const * const spp = GetSensor(heater);
	return (spp == nullptr || *spp == nullptr) ? nullptr : (*spp)->GetHeaterName();
}

// Get the temperature of a real or virtual heater
float Heat::GetTemperature(size_t heater, TemperatureError& err)
{
	TemperatureSensor ** const spp = GetSensor(heater);
	if (spp == nullptr)
	{
		err = TemperatureError::unknownHeater;
		return BAD_ERROR_TEMPERATURE;
	}

	if (*spp == nullptr)
	{
		err = TemperatureError::unknownChannel;
		return BAD_ERROR_TEMPERATURE;
	}

	float t;
	err = (*spp)->GetTemperature(t);
	if (err != TemperatureError::success)
	{
		t = BAD_ERROR_TEMPERATURE;
	}
	return t;
}

#if HAS_VOLTAGE_MONITOR

// Suspend the heaters to conserve power
void Heat::SuspendHeaters(bool sus)
{
	for (PID *p : pids)
	{
		p->Suspend(sus);
	}
}

#endif

// Save some resume information returning true if successful.
// We assume that the bed and chamber heaters are either on and active, or off (not on standby).
bool Heat::WriteBedAndChamberTempSettings(FileStore *f) const
{
	char bufSpace[100];
	StringRef buf(bufSpace, ARRAY_SIZE(bufSpace));
	if (bedHeater >= 0 && pids[bedHeater]->Active() && !pids[bedHeater]->SwitchedOff())
	{
		buf.printf("M140 S%.1f\n", (double)GetActiveTemperature(bedHeater));
	}
	if (chamberHeater >= 0 && pids[chamberHeater]->Active() && !pids[chamberHeater]->SwitchedOff())
	{
		buf.printf("M141 S%.1f\n", (double)GetActiveTemperature(chamberHeater));
	}
	return (buf.Length() == 0) || f->Write(buf.Pointer());
}

// End
