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
#include "HeaterProtection.h"
#include "Platform.h"
#include "RepRap.h"
#include "Sensors/TemperatureSensor.h"

#if SUPPORT_DHT_SENSOR
# include "Sensors/DhtSensor.h"
#endif

#ifdef RTOS

# include "Tasks.h"

constexpr uint32_t HeaterTaskStackWords = 400;			// task stack size in dwords, must be large enough for auto tuning
static Task<HeaterTaskStackWords> heaterTask;

extern "C" void HeaterTask(void * pvParameters)
{
	reprap.GetHeat().Task();
}

#endif

Heat::Heat(Platform& p)
	: platform(p),
#ifndef RTOS
	  active(false),
#endif
	  coldExtrude(false), heaterBeingTuned(-1), lastHeaterTuned(-1)
{
	ARRAY_INIT(bedHeaters, DefaultBedHeaters);
	ARRAY_INIT(chamberHeaters, DefaultChamberHeaters);

	for (size_t index : ARRAY_INDICES(heaterProtections))
	{
		heaterProtections[index] = new HeaterProtection(index);
	}

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
			if (IsBedOrChamberHeater(heater))
			{
				pids[heater]->SetModel(DefaultBedHeaterGain, DefaultBedHeaterTimeConstant, DefaultBedHeaterDeadTime, 1.0, 0.0, false, false, 0);
			}
			else
			{
				pids[heater]->SetModel(DefaultHotEndHeaterGain, DefaultHotEndHeaterTimeConstant, DefaultHotEndHeaterDeadTime, 1.0, 0.0, true, false, 0);
			}
		}
	}
}

void Heat::Init()
{
	// Initialise the heater protection items first
	for (size_t index : ARRAY_INDICES(heaterProtections))
	{
		HeaterProtection * const prot = heaterProtections[index];

		const float tempLimit = (IsBedOrChamberHeater(index)) ? DefaultBedTemperatureLimit : DefaultHotEndTemperatureLimit;
		prot->Init(tempLimit);

		if (index < NumHeaters)
		{
			pids[index]->SetHeaterProtection(prot);
		}
	}

	// Then set up the real heaters and the corresponding PIDs
	for (size_t heater : ARRAY_INDICES(pids))
	{
		heaterSensors[heater] = nullptr;			// no temperature sensor assigned yet
#ifdef PCCB
		// PCCB has no heaters by default, but we pretend that the LED outputs are heaters. So disable the PID controllers.
		pids[heater]->Init(-1.0, -1.0, -1.0, true, false);
#else
		if (IsBedOrChamberHeater(heater))
		{
			pids[heater]->Init(DefaultBedHeaterGain, DefaultBedHeaterTimeConstant, DefaultBedHeaterDeadTime, false, false);
		}
#if defined(DUET_06_085)
		else if (heater == NumHeaters - 1)
		{
			// On the Duet 085, the heater 6 pin is also the fan 1 pin. By default we support fan 1, so disable heater 6.
			pids[heater]->Init(-1.0, -1.0, -1.0, true, false);
		}
#endif
		else
		{
			pids[heater]->Init(DefaultHotEndHeaterGain, DefaultHotEndHeaterTimeConstant, DefaultHotEndHeaterDeadTime, true, false);
		}
#endif
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
#ifndef PCCB
	virtualHeaterSensors[2] = TemperatureSensor::Create(FirstTmcDriversSenseChannel + 1);
#endif
#endif

#if SUPPORT_DHT_SENSOR
	// Initialise static fields of the DHT sensor
	DhtSensorHardwareInterface::InitStatic();
#endif

	extrusionMinTemp = HOT_ENOUGH_TO_EXTRUDE;
	retractionMinTemp = HOT_ENOUGH_TO_RETRACT;
	coldExtrude = false;

#ifdef RTOS
	heaterTask.Create(HeaterTask, "HEAT", nullptr, TaskPriority::HeatPriority);
#else
	lastTime = millis() - HeatSampleIntervalMillis;		// flag the PIDS as due for spinning
	active = true;
#endif
}

void Heat::Exit()
{
	for (PID *pid : pids)
	{
		pid->SwitchOff();
	}

#ifdef RTOS
	heaterTask.Suspend();
#else
	active = false;
#endif
}

#ifdef RTOS

void Heat::Task()
{
	lastWakeTime = xTaskGetTickCount();
	for (;;)
	{
		for (PID *& p : pids)
		{
			p->Spin();
		}

		// See if we have finished tuning a PID
		if (heaterBeingTuned != -1 && !pids[heaterBeingTuned]->IsTuning())
		{
			lastHeaterTuned = heaterBeingTuned;
			heaterBeingTuned = -1;
		}

		reprap.KickHeatTaskWatchdog();

		// Delay until it is time again
		vTaskDelayUntil(&lastWakeTime, HeatSampleIntervalMillis);
	}
}

#else

void Heat::Spin()
{
	if (active)
	{
		// See if it is time to spin the PIDs
		const uint32_t now = millis();
		if (now - lastTime >= HeatSampleIntervalMillis)
		{
			lastTime = now;
			for (PID *& p : pids)
			{
				p->Spin();
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
}

#endif

void Heat::Diagnostics(MessageType mtype)
{
	platform.Message(mtype, "=== Heat ===\nBed heaters =");
	for (int8_t bedHeater : bedHeaters)
	{
		platform.MessageF(mtype, " %d", bedHeater);
	}
	platform.Message(mtype, ", chamberHeaters =");
	for (int8_t chamberHeater : chamberHeaters)
	{
		platform.MessageF(mtype, " %d", chamberHeater);
	}
	platform.Message(mtype, "\n");

	for (size_t heater : ARRAY_INDICES(pids))
	{
		if (pids[heater]->Active())
		{
			platform.MessageF(mtype, "Heater %d is on, I-accum = %.1f\n", heater, (double)(pids[heater]->GetAccumulator()));
		}
	}
}

bool Heat::AllHeatersAtSetTemperatures(bool includingBed, float tolerance) const
{
	for (size_t heater : ARRAY_INDICES(pids))
	{
		if (!HeaterAtSetTemperature(heater, true, tolerance) && (includingBed || !IsBedHeater(heater)))
		{
			return false;
		}
	}
	return true;
}

//query an individual heater
bool Heat::HeaterAtSetTemperature(int8_t heater, bool waitWhenCooling, float tolerance) const
{
	// If it hasn't anything to do, it must be right wherever it is...
	if (heater < 0 || heater >= (int)NumHeaters || pids[heater]->SwitchedOff() || pids[heater]->FaultOccurred())
	{
		return true;
	}

	const float dt = GetTemperature(heater);
	const float target = (pids[heater]->Active()) ? GetActiveTemperature(heater) : GetStandbyTemperature(heater);
	return (target < TEMPERATURE_LOW_SO_DONT_CARE)
		|| (fabsf(dt - target) <= tolerance)
		|| (target < dt && !waitWhenCooling);
}

Heat::HeaterStatus Heat::GetStatus(int8_t heater) const
{
	if (heater < 0 || heater >= (int)NumHeaters)
	{
		return HS_off;
	}

	return (pids[heater]->FaultOccurred()) ? HS_fault
			: (pids[heater]->SwitchedOff()) ? HS_off
				: (pids[heater]->IsTuning()) ? HS_tuning
					: (pids[heater]->Active()) ? HS_active
						: HS_standby;
}

void Heat::SetBedHeater(size_t index, int8_t heater)
{
	const int bedHeater = bedHeaters[index];
	if (bedHeater >= 0)
	{
		pids[bedHeater]->SwitchOff();
	}
	bedHeaters[index] = heater;
}

bool Heat::IsBedHeater(int8_t heater) const
{
	for (int8_t bedHeater : bedHeaters)
	{
		if (heater == bedHeater)
		{
			return true;
		}
	}
	return false;
}

void Heat::SetChamberHeater(size_t index, int8_t heater)
{
	const int chamberHeater = chamberHeaters[index];
	if (chamberHeater >= 0)
	{
		pids[chamberHeater]->SwitchOff();
	}
	chamberHeaters[index] = heater;
}

bool Heat::IsChamberHeater(int8_t heater) const
{
	for (int8_t chamberHeater : chamberHeaters)
	{
		if (heater == chamberHeater)
		{
			return true;
		}
	}
	return false;
}

void Heat::SetActiveTemperature(int8_t heater, float t)
{
	if (heater >= 0 && heater < (int)NumHeaters)
	{
		pids[heater]->SetActiveTemperature(t);
	}
}

float Heat::GetActiveTemperature(int8_t heater) const
{
	return (heater >= 0 && heater < (int)NumHeaters) ? pids[heater]->GetActiveTemperature() : ABS_ZERO;
}

void Heat::SetStandbyTemperature(int8_t heater, float t)
{
	if (heater >= 0 && heater < (int)NumHeaters)
	{
		pids[heater]->SetStandbyTemperature(t);
	}
}

float Heat::GetStandbyTemperature(int8_t heater) const
{
	return (heater >= 0 && heater < (int)NumHeaters) ? pids[heater]->GetStandbyTemperature() : ABS_ZERO;
}

float Heat::GetHighestTemperatureLimit(int8_t heater) const
{
	float limit = BadErrorTemperature;
	if (heater >= 0 && heater < (int)NumHeaters)
	{
		for (const HeaterProtection *prot : heaterProtections)
		{
			if (prot->GetSupervisedHeater() == heater && prot->GetTrigger() == HeaterProtectionTrigger::TemperatureExceeded)
			{
				const float t = prot->GetTemperatureLimit();
				if (limit == BadErrorTemperature || t > limit)
				{
					limit = t;
				}
			}
		}
	}
	return limit;
}

float Heat::GetLowestTemperatureLimit(int8_t heater) const
{
	float limit = ABS_ZERO;
	if (heater >= 0 && heater < (int)NumHeaters)
	{
		for (const HeaterProtection *prot : heaterProtections)
		{
			if (prot->GetSupervisedHeater() == heater && prot->GetTrigger() == HeaterProtectionTrigger::TemperatureTooLow)
			{
				const float t = prot->GetTemperatureLimit();
				if (limit == ABS_ZERO || t < limit)
				{
					limit = t;
				}
			}
		}
	}
	return limit;
}

// Get the current temperature of a heater
float Heat::GetTemperature(int8_t heater) const
{
	return (heater >= 0 && heater < (int)NumHeaters) ? pids[heater]->GetTemperature() : ABS_ZERO;
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
	if (heater >= 0 && heater < (int)NumHeaters)
	{
		pids[heater]->Activate();
	}
}

void Heat::SwitchOff(int8_t heater)
{
	if (heater >= 0 && heater < (int)NumHeaters)
	{
		pids[heater]->SwitchOff();
		lastStandbyTools[heater] = nullptr;
	}
}

void Heat::SwitchOffAll(bool includingChamberAndBed)
{
	for (int heater = 0; heater < (int)NumHeaters; ++heater)
	{
		if (includingChamberAndBed || !IsBedOrChamberHeater(heater))
		{
			pids[heater]->SwitchOff();
		}
	}
}

void Heat::Standby(int8_t heater, const Tool *tool)
{
	if (heater >= 0 && heater < (int)NumHeaters)
	{
		pids[heater]->Standby();
		lastStandbyTools[heater] = tool;
	}
}

void Heat::ResetFault(int8_t heater)
{
	if (heater >= 0 && heater < (int)NumHeaters)
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

bool Heat::IsBedOrChamberHeater(int8_t heater) const
{
	return IsBedHeater(heater) || IsChamberHeater(heater);
}

// Auto tune a PID
void Heat::StartAutoTune(size_t heater, float temperature, float maxPwm, const StringRef& reply)
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

void Heat::GetAutoTuneStatus(const StringRef& reply) const
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
	for (HeaterProtection *prot : heaterProtections)
	{
		if (prot->GetHeater() >= 0 && prot->GetTrigger() == HeaterProtectionTrigger::TemperatureExceeded)
		{
			const float t = prot->GetTemperatureLimit();
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
GCodeResult Heat::ConfigureHeaterSensor(unsigned int mcode, size_t heater, GCodeBuffer& gb, const StringRef& reply)
{
	TemperatureSensor ** const spp = GetSensor(heater);
	if (spp == nullptr || *spp == nullptr)
	{
		reply.printf("heater %d is not configured", heater);
		return GCodeResult::error;
	}

	return (*spp)->Configure(mcode, heater, gb, reply);
}

// Get a pointer to the temperature sensor entry, or nullptr if the heater number is bad
TemperatureSensor **Heat::GetSensor(size_t heater)
{
	if (heater < NumHeaters)
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
	if (heater < NumHeaters)
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

// Return the protection parameters of the given index
HeaterProtection& Heat::AccessHeaterProtection(size_t index) const
{
	if (index >= FirstExtraHeaterProtection && index < FirstExtraHeaterProtection + NumExtraHeaterProtections)
	{
		return *heaterProtections[index + NumHeaters - FirstExtraHeaterProtection];
	}
	return *heaterProtections[index];
}

// Updates the PIDs and HeaterProtection items after a heater change
void Heat::UpdateHeaterProtection()
{
	// Reassign the first mapped heater protection item of each PID where applicable
	// and rebuild the linked list of heater protection elements per heater
	for (size_t heater : ARRAY_INDICES(pids))
	{
		// Rebuild linked lists
		HeaterProtection *firstProtectionItem = nullptr;
		HeaterProtection *lastElementInList = nullptr;
		for (HeaterProtection *prot : heaterProtections)
		{
			if (prot->GetHeater() == (int)heater)
			{
				if (firstProtectionItem == nullptr)
				{
					firstProtectionItem = prot;
					prot->SetNext(nullptr);
				}
				else if (lastElementInList == nullptr)
				{
					firstProtectionItem->SetNext(prot);
					lastElementInList = prot;
				}
				else
				{
					lastElementInList->SetNext(prot);
					lastElementInList = prot;
				}
			}
		}

		// Update reference to the first item so that we can achieve better performance
		pids[heater]->SetHeaterProtection(firstProtectionItem);
	}
}

// Check if the heater is able to operate returning true if everything is OK
bool Heat::CheckHeater(size_t heater)
{
	return !pids[heater]->FaultOccurred() && pids[heater]->CheckProtection();
}

// Get the temperature of a real or virtual heater
float Heat::GetTemperature(size_t heater, TemperatureError& err)
{
	TemperatureSensor * const * const spp = GetSensor(heater);
	if (spp == nullptr)
	{
		err = TemperatureError::unknownHeater;
		return BadErrorTemperature;
	}

	if (*spp == nullptr)
	{
		err = TemperatureError::unknownChannel;
		return BadErrorTemperature;
	}

	float t;
	err = (*spp)->GetTemperature(t);
	if (err != TemperatureError::success)
	{
		t = BadErrorTemperature;
	}
	return t;
}

// Suspend the heaters to conserve power or while doing Z probing
void Heat::SuspendHeaters(bool sus)
{
	for (PID *p : pids)
	{
		p->Suspend(sus);
	}
}

// Save some resume information returning true if successful.
// We assume that the bed and chamber heaters are either on and active, or off (not on standby).
bool Heat::WriteBedAndChamberTempSettings(FileStore *f) const
{
	String<100> bufSpace;
	const StringRef buf = bufSpace.GetRef();
	for (size_t index : ARRAY_INDICES(bedHeaters))
	{
		const int bedHeater = bedHeaters[index];
		if (bedHeater >= 0 && pids[bedHeater]->Active() && !pids[bedHeater]->SwitchedOff())
		{
			buf.printf("M140 P%u S%.1f\n", index, (double)GetActiveTemperature(bedHeater));
		}
	}
	for (size_t index : ARRAY_INDICES(chamberHeaters))
	{
		const int chamberHeater = chamberHeaters[index];
		if (chamberHeater >= 0 && pids[chamberHeater]->Active() && !pids[chamberHeater]->SwitchedOff())
		{
			buf.printf("M141 P%u S%.1f\n", index, (double)GetActiveTemperature(chamberHeater));
		}
	}
	return (buf.strlen() == 0) || f->Write(buf.c_str());
}

// End
