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
#include "LocalHeater.h"
#include "HeaterProtection.h"
#include "Platform.h"
#include "RepRap.h"
#include "Sensors/TemperatureSensor.h"
#include "GCodes/GCodeBuffer/GCodeBuffer.h"
#include "Tasks.h"

#if SUPPORT_DHT_SENSOR
# include "Sensors/DhtSensor.h"
#endif

#if SUPPORT_CAN_EXPANSION
# include "CanId.h"
# include "RemoteHeater.h"
#endif

constexpr uint32_t HeaterTaskStackWords = 400;			// task stack size in dwords, must be large enough for auto tuning
static Task<HeaterTaskStackWords> heaterTask;

extern "C" [[noreturn]] void HeaterTask(void * pvParameters)
{
	reprap.GetHeat().Task();
}

Heat::Heat()
	: sensorsRoot(nullptr), newSensors(nullptr), coldExtrude(false), heaterBeingTuned(-1), lastHeaterTuned(-1)
{
	ARRAY_INIT(bedHeaters, DefaultBedHeaters);
	ARRAY_INIT(chamberHeaters, DefaultChamberHeaters);

	for (size_t index : ARRAY_INDICES(heaterProtections))
	{
		heaterProtections[index] = new HeaterProtection(index);
	}

	for (Heater*& h : heaters)
	{
		h = nullptr;
	}

	// Then set up the real heaters and the corresponding PIDs
	for (const Tool*& t : lastStandbyTools)
	{
		t = nullptr;
	}

}

Heater *Heat::FindHeater(int heater) const
{
	return (heater < 0 || heater >= (int)MaxHeaters) ? nullptr : heaters[heater];
}

// Process M307
GCodeResult Heat::SetOrReportHeaterModel(GCodeBuffer& gb, const StringRef& reply)
{
	if (gb.Seen('H'))
	{
		const unsigned int heater = gb.GetUIValue();
		Heater * const h = FindHeater(heater);
		if (h != nullptr)
		{
			const FopDt& model = h->GetModel();
			bool seen = false;
			float gain = model.GetGain(),
				tc = model.GetTimeConstant(),
				td = model.GetDeadTime(),
				maxPwm = model.GetMaxPwm(),
				voltage = model.GetVoltage();
			int32_t dontUsePid = model.UsePid() ? 0 : 1;
			int32_t inversionParameter = 0;

			gb.TryGetFValue('A', gain, seen);
			gb.TryGetFValue('C', tc, seen);
			gb.TryGetFValue('D', td, seen);
			gb.TryGetIValue('B', dontUsePid, seen);
			gb.TryGetFValue('S', maxPwm, seen);
			gb.TryGetFValue('V', voltage, seen);
			gb.TryGetIValue('I', inversionParameter, seen);

			if (seen)
			{
				const bool inverseTemperatureControl = (inversionParameter == 1 || inversionParameter == 3);
				const GCodeResult rslt = h->SetModel(gain, tc, td, maxPwm, voltage, dontUsePid == 0, inverseTemperatureControl, reply);
				if (rslt != GCodeResult::ok)
				{
					return rslt;
				}
			}
			else if (!model.IsEnabled())
			{
				reply.printf("Heater %u is disabled", heater);
			}
			else
			{
				const char* const mode = (!model.UsePid()) ? "bang-bang"
											: (model.ArePidParametersOverridden()) ? "custom PID"
												: "PID";
				reply.printf("Heater %u model: gain %.1f, time constant %.1f, dead time %.1f, max PWM %.2f, calibration voltage %.1f, mode %s", heater,
							 (double)model.GetGain(), (double)model.GetTimeConstant(), (double)model.GetDeadTime(), (double)model.GetMaxPwm(), (double)model.GetVoltage(), mode);
				if (model.IsInverted())
				{
					reply.cat(", inverted temperature control");
				}
				if (model.UsePid())
				{
					// When reporting the PID parameters, we scale them by 255 for compatibility with older firmware and other firmware
					M301PidParameters params = model.GetM301PidParameters(false);
					reply.catf("\nComputed PID parameters for setpoint change: P%.1f, I%.3f, D%.1f", (double)params.kP, (double)params.kI, (double)params.kD);
					params = model.GetM301PidParameters(true);
					reply.catf("\nComputed PID parameters for load change: P%.1f, I%.3f, D%.1f", (double)params.kP, (double)params.kI, (double)params.kD);
				}
			}
			return GCodeResult::ok;
		}

		reply.printf("Heater %u not found", heater);
		return GCodeResult::error;
	}

	return GCodeResult::badOrMissingParameter;
}

// Process M301 or M304. 'heater' is the default heater number to use.
GCodeResult Heat::SetPidParameters(unsigned int heater, GCodeBuffer& gb, const StringRef& reply)
{
	if (gb.Seen('H'))
	{
		heater = gb.GetUIValue();
	}

	Heater * const h = FindHeater(heater);
	if (h != nullptr)
	{
		const FopDt& model = h->GetModel();
		M301PidParameters pp = model.GetM301PidParameters(false);
		bool seen = false;
		gb.TryGetFValue('P', pp.kP, seen);
		gb.TryGetFValue('I', pp.kI, seen);
		gb.TryGetFValue('D', pp.kD, seen);

		if (seen)
		{
			h->SetM301PidParameters(pp);
		}
		else if (!model.UsePid())
		{
			reply.printf("Heater %d is in bang-bang mode", heater);
		}
		else if (model.ArePidParametersOverridden())
		{
			reply.printf("Heater %d P:%.1f I:%.3f D:%.1f", heater, (double)pp.kP, (double)pp.kI, (double)pp.kD);
		}
		else
		{
			reply.printf("Heater %d uses model-derived PID parameters. Use M307 H%d to view them", heater, heater);
		}
		return GCodeResult::ok;
	}

	reply.printf("Heater %u not found", heater);
	return GCodeResult::error;
}

// Is the heater enabled?
bool Heat::IsHeaterEnabled(size_t heater) const
{
	Heater * const h = FindHeater(heater);
	return h != nullptr && h->IsHeaterEnabled();
}

// Get a pointer to the temperature sensor entry, or nullptr if the sensor number is bad
TemperatureSensor *Heat::GetSensor(int sn) const
{
	if (sn >= 0)
	{
		TaskCriticalSectionLocker lock;		// make sure the linked list doesn't change while we are searching it
		for (TemperatureSensor *sensor = sensorsRoot; sensor != nullptr; sensor = sensor->GetNext())
		{
			if (sensor->GetSensorNumber() == sn)
			{
				return sensor;
			}
		}
	}
	return nullptr;
}

// Get a pointer to the first temperature sensor with the specified or higher number
TemperatureSensor *Heat::GetSensorAtOrAbove(unsigned int sn) const
{
	TaskCriticalSectionLocker lock;		// make sure the linked list doesn't change while we are searching it

	for (TemperatureSensor *sensor = sensorsRoot; sensor != nullptr; sensor = sensor->GetNext())
	{
		if (sensor->GetSensorNumber() >= (int)sn)
		{
			return sensor;
		}
	}
	return nullptr;
}

// Reset all heater models to defaults. Called when running M502.
void Heat::ResetHeaterModels()
{
	for (Heater* h : heaters)
	{
		if (h != nullptr && h->IsHeaterEnabled())
		{
			h->SetModelDefaults();
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
	}

#if SUPPORT_DHT_SENSOR
	// Initialise static fields of the DHT sensor
	DhtSensorHardwareInterface::InitStatic();
#endif

	extrusionMinTemp = HOT_ENOUGH_TO_EXTRUDE;
	retractionMinTemp = HOT_ENOUGH_TO_RETRACT;
	coldExtrude = false;

	heaterTask.Create(HeaterTask, "HEAT", nullptr, TaskPriority::HeatPriority);
}

void Heat::Exit()
{
	for (Heater *h : heaters)
	{
		if (h != nullptr)
		{
			h->SwitchOff();
		}
	}

	heaterTask.Suspend();
}

[[noreturn]] void Heat::Task()
{
	uint32_t lastWakeTime = xTaskGetTickCount();
	for (;;)
	{
		// Walk the sensor list and poll all sensors except those flagged for deletion. Don't mess with the list during this pass because polling may need to acquire mutexes.
		TemperatureSensor *currentSensor = sensorsRoot;
		bool sawDeletedSensor = false;
		while (currentSensor != nullptr)
		{
			if (currentSensor->GetSensorNumber() < 0)
			{
				sawDeletedSensor = true;
			}
			else
			{
				currentSensor->Poll();
			}
			currentSensor = currentSensor->GetNext();
		}

		// If we saw any sensors flagged for deletion, delete them, locking out other tasks while we do this
		if (sawDeletedSensor)
		{
			TaskCriticalSectionLocker lock;

			currentSensor = sensorsRoot;
			TemperatureSensor *lastSensor = nullptr;
			while (currentSensor != nullptr)
			{
				if (currentSensor->GetSensorNumber() < 0)
				{
					TemperatureSensor *sensorToDelete = currentSensor;
					currentSensor = currentSensor->GetNext();
					if (lastSensor == nullptr)
					{
						sensorsRoot = currentSensor;
					}
					else
					{
						lastSensor->SetNext(currentSensor);
					}
					delete sensorToDelete;
				}
				else
				{
					lastSensor = currentSensor;
					currentSensor = currentSensor->GetNext();
				}
			}
		}

		// Insert any new sensors. We don't poll them yet because they may only just have finished being initialised so they may not accept another transaction yet.
		for (;;)
		{
			TaskCriticalSectionLocker lock;
			TemperatureSensor *currentNewSensor = newSensors;
			if (currentNewSensor == nullptr)
			{
				break;
			}
			newSensors = currentNewSensor->GetNext();
			TemperatureSensor *prev = nullptr;
			TemperatureSensor *ts = sensorsRoot;
			for (;;)
			{
				if (ts == nullptr || ts->GetSensorNumber() > currentNewSensor->GetSensorNumber())
				{
					currentNewSensor->SetNext(ts);
					if (prev == nullptr)
					{
						sensorsRoot = currentNewSensor;
					}
					else
					{
						prev->SetNext(currentNewSensor);
					}
					break;
				}
				prev = ts;
				ts = ts->GetNext();
			}
		}

		// Spin the heaters
		for (Heater *h : heaters)
		{
			if (h != nullptr)
			{
				h->Spin();
			}
		}

		// See if we have finished tuning a PID
		if (heaterBeingTuned != -1 && heaters[heaterBeingTuned]->GetStatus() != HeaterStatus::tuning)
		{
			lastHeaterTuned = heaterBeingTuned;
			heaterBeingTuned = -1;
		}

		reprap.KickHeatTaskWatchdog();

		// Delay until it is time again
		vTaskDelayUntil(&lastWakeTime, HeatSampleIntervalMillis);
	}
}

void Heat::Diagnostics(MessageType mtype)
{
	Platform& platform = reprap.GetPlatform();
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

	for (size_t heater : ARRAY_INDICES(heaters))
	{
		if (heaters[heater] != nullptr && heaters[heater]->GetStatus() == HeaterStatus::active)
		{
			platform.MessageF(mtype, "Heater %u is on, I-accum = %.1f\n", heater, (double)(heaters[heater]->GetAccumulator()));
		}
	}
}

// Configure a heater
GCodeResult Heat::ConfigureHeater(size_t heater, GCodeBuffer& gb, const StringRef& reply)
{
	if (heater < MaxHeaters)
	{
		Heater *oldHeater = heaters[heater];

#if SUPPORT_CAN_EXPANSION
		CanAddress boardAddr = CanId::NoAddress;
		if (gb.Seen('C'))
		{
			String<StringLength20> portName;
			if (!gb.GetReducedString(portName.GetRef()))
			{
				reply.copy("Missing port name");
				return GCodeResult::error;
			}
			boardAddr = IoPort::RemoveBoardAddress(portName.GetRef());
		}

		if (boardAddr == CanId::NoAddress)
		{
			// No port given, so just configure the existing heater if there is one
			if (oldHeater == nullptr)
			{
				reply.printf("Port name needed to create new heater %u", heater);
				return GCodeResult::error;
			}
		}
		else
		{
			// A port has been provided, so create a new heater
			if (oldHeater == nullptr)
			{
				heaters[heater] = (boardAddr == 0) ? (Heater *)new LocalHeater(heater) : new RemoteHeater(heater, boardAddr);
			}
			else
			{
				oldHeater->ReleasePort();
				heaters[heater] = (boardAddr == 0) ? (Heater *)new LocalHeater(*oldHeater) : new RemoteHeater(*oldHeater, boardAddr);
				delete oldHeater;
			}
		}
#else
		if (oldHeater == nullptr)
		{
			heaters[heater] = new LocalHeater(heater);
		}
#endif
		return heaters[heater]->ConfigurePortAndSensor(gb, reply);
	}

	reply.copy("Heater number out of range");
	return GCodeResult::error;
}

bool Heat::AllHeatersAtSetTemperatures(bool includingBed, float tolerance) const
{
	for (size_t heater : ARRAY_INDICES(heaters))
	{
		if (!HeaterAtSetTemperature(heater, true, tolerance) && (includingBed || !IsBedHeater(heater)))
		{
			return false;
		}
	}
	return true;
}

//query an individual heater
bool Heat::HeaterAtSetTemperature(int heater, bool waitWhenCooling, float tolerance) const
{
	Heater * const h = FindHeater(heater);
	if (h != nullptr)
	{
		const HeaterStatus stat = h->GetStatus();
		if (stat == HeaterStatus::active || stat == HeaterStatus::standby)
		{
			const float dt = h->GetTemperature();
			const float target = (stat == HeaterStatus::active) ? h->GetActiveTemperature() : h->GetStandbyTemperature();
			return (target < TEMPERATURE_LOW_SO_DONT_CARE)
				|| (fabsf(dt - target) <= tolerance)
				|| (target < dt && !waitWhenCooling);

		}
	}

	// If the heater doesn't exist or is switched off or in a fault state, there is nothing to wait for
	return true;
}

HeaterStatus Heat::GetStatus(int heater) const
{
	Heater * const h = FindHeater(heater);
	return (h == nullptr) ? HeaterStatus::off : heaters[heater]->GetStatus();
}

void Heat::SetBedHeater(size_t index, int heater)
{
	Heater * const h = FindHeater(bedHeaters[index]);
	if (h != nullptr)
	{
		h->SwitchOff();
	}
	bedHeaters[index] = heater;
}

bool Heat::IsBedHeater(int heater) const
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

void Heat::SetChamberHeater(size_t index, int heater)
{
	Heater * const h = FindHeater(chamberHeaters[index]);
	if (h != nullptr)
	{
		h->SwitchOff();
	}
	chamberHeaters[index] = heater;
}

bool Heat::IsChamberHeater(int heater) const
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

void Heat::SetActiveTemperature(int heater, float t)
{
	Heater * const h = FindHeater(heater);
	if (h != nullptr)
	{
		h->SetActiveTemperature(t);
	}
}

float Heat::GetActiveTemperature(int heater) const
{
	Heater * const h = FindHeater(heater);
	return (h == nullptr) ? ABS_ZERO : h->GetActiveTemperature();
}

void Heat::SetStandbyTemperature(int heater, float t)
{
	Heater * const h = FindHeater(heater);
	if (h != nullptr)
	{
		h->SetStandbyTemperature(t);
	}
}

float Heat::GetStandbyTemperature(int heater) const
{
	Heater * const h = FindHeater(heater);
	return (h == nullptr) ? ABS_ZERO : h->GetStandbyTemperature();
}

float Heat::GetHighestTemperatureLimit(int heater) const
{
	float limit = BadErrorTemperature;
	if (heater >= 0 && heater < (int)MaxHeaters)
	{
		for (const HeaterProtection *prot : heaterProtections)
		{
			if (prot->GetHeater() == heater && prot->GetTrigger() == HeaterProtectionTrigger::TemperatureExceeded)
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

float Heat::GetLowestTemperatureLimit(int heater) const
{
	float limit = ABS_ZERO;
	if (heater >= 0 && heater < (int)MaxHeaters)
	{
		for (const HeaterProtection *prot : heaterProtections)
		{
			if (prot->GetHeater() == heater && prot->GetTrigger() == HeaterProtectionTrigger::TemperatureTooLow)
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

// Get the current temperature of a real or virtual heater
// Return ABS_ZERO if the heater doesn't exist. The Z probe class relies on this.
float Heat::GetHeaterTemperature(int heater) const
{
	Heater * const h = FindHeater(heater);
	return (h == nullptr) ? ABS_ZERO : h->GetTemperature();
}

// Get the target temperature of a heater
float Heat::GetTargetTemperature(int heater) const
{
	const HeaterStatus hs = GetStatus(heater);
	return (hs == HeaterStatus::active) ? GetActiveTemperature(heater)
			: (hs == HeaterStatus::standby) ? GetStandbyTemperature(heater)
				: 0.0;
}

void Heat::Activate(int heater)
{
	Heater * const h = FindHeater(heater);
	if (h != nullptr)
	{
		h->Activate();
	}
}

void Heat::SwitchOff(int heater)
{
	Heater * const h = FindHeater(heater);
	if (h != nullptr)
	{
		h->SwitchOff();
		lastStandbyTools[heater] = nullptr;
	}
}

void Heat::SwitchOffAll(bool includingChamberAndBed)
{
	for (int heater = 0; heater < (int)MaxHeaters; ++heater)
	{
		Heater * const h = heaters[heater];
		if (h != nullptr && (includingChamberAndBed || !IsBedOrChamberHeater(heater)))
		{
			h->SwitchOff();
		}
	}
}

void Heat::Standby(int heater, const Tool *tool)
{
	Heater * const h = FindHeater(heater);
	if (h != nullptr)
	{
		h->Standby();
		lastStandbyTools[heater] = tool;
	}
}

void Heat::ResetFault(int heater)
{
	Heater * const h = FindHeater(heater);
	if (h != nullptr)
	{
		h->ResetFault();
	}
}

float Heat::GetAveragePWM(size_t heater) const
{
	Heater * const h = FindHeater(heater);
	return (h == nullptr) ? 0.0 : h->GetAveragePWM();
}

bool Heat::IsBedOrChamberHeater(int heater) const
{
	return IsBedHeater(heater) || IsChamberHeater(heater);
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

#if HAS_MASS_STORAGE

// Write heater model parameters to file returning true if no error
bool Heat::WriteModelParameters(FileStore *f) const
{
	bool ok = f->Write("; Heater model parameters\n");
	for (size_t h : ARRAY_INDICES(heaters))
	{
		if (heaters[h] != nullptr)
		{
			const FopDt& model = heaters[h]->GetModel();
			if (model.IsEnabled())
			{
				ok = model.WriteParameters(f, h);
			}
		}
	}
	return ok;
}

#endif

// Process M570
GCodeResult Heat::ConfigureHeaterMonitoring(size_t heater, GCodeBuffer& gb, const StringRef& reply)
{
	if (heater < MaxHeaters && heaters[heater] != nullptr)
	{
		bool seenValue = false;
		float maxTempExcursion, maxFaultTime;
		heaters[heater]->GetFaultDetectionParameters(maxTempExcursion, maxFaultTime);
		gb.TryGetFValue('P', maxFaultTime, seenValue);
		gb.TryGetFValue('T', maxTempExcursion, seenValue);
		if (seenValue)
		{
			heaters[heater]->SetFaultDetectionParameters(maxTempExcursion, maxFaultTime);
		}
		else
		{
			reply.printf("Heater %u allowed excursion %.1f" DEGREE_SYMBOL "C, fault trigger time %.1f seconds", heater, (double)maxTempExcursion, (double)maxFaultTime);
		}
	}
	return GCodeResult::ok;
}

// Process M303
GCodeResult Heat::TuneHeater(GCodeBuffer& gb, const StringRef& reply)
{
	if (gb.Seen('H'))
	{
		const unsigned int heater = gb.GetUIValue();
		Heater * const h = FindHeater(heater);
		if (h != nullptr)
		{
			const float temperature = (gb.Seen('S')) ? gb.GetFValue()
										: reprap.GetHeat().IsBedHeater(heater) ? 75.0
										: reprap.GetHeat().IsChamberHeater(heater) ? 50.0
										: 200.0;
			const float maxPwm = (gb.Seen('P')) ? gb.GetFValue() : h->GetModel().GetMaxPwm();
			if (!h->CheckGood())
			{
				reply.copy("Heater is not ready to perform PID auto-tuning");
			}
			else if (maxPwm < 0.1 || maxPwm > 1.0)
			{
				reply.copy("Invalid PWM value");
			}
			else
			{
				if (heaterBeingTuned == -1)
				{
					heaterBeingTuned = (int8_t)heater;
					h->StartAutoTune(temperature, maxPwm, reply);
					return GCodeResult::ok;
				}
				else
				{
					// Trying to start a new auto tune, but we are already tuning a heater
					reply.printf("Error: cannot start auto tuning heater %u because heater %d is being tuned", heater, heaterBeingTuned);
				}
			}
		}
		else
		{
			reply.printf("Heater %u not found", heater);
		}
		return GCodeResult::error;
	}
	else
	{
		// Report the auto tune status
		const int whichPid = (heaterBeingTuned == -1) ? lastHeaterTuned : heaterBeingTuned;
		Heater * const h = FindHeater(whichPid);

		if (h != nullptr)
		{
			h->GetAutoTuneStatus(reply);
		}
		else
		{
			reply.copy("No heater has been tuned yet");
		}
		return GCodeResult::ok;
	}
}

// Process M308
GCodeResult Heat::ConfigureSensor(GCodeBuffer& gb, const StringRef& reply)
{
	if (gb.Seen('S'))
	{
		const unsigned sensorNum = gb.GetUIValue();
		if (sensorNum >= MaxSensorsInSystem)
		{
			reply.copy("Sensor number out of range");
			return GCodeResult::error;
		}

#if SUPPORT_CAN_EXPANSION
		// Set boardAddress to the board number that the port is on, or NoCanAddress if the port was not given
		CanAddress boardAddress;
		String<StringLength20> portName;
		if (gb.Seen('P'))
		{
			if (!gb.GetReducedString(portName.GetRef()))
			{
				reply.copy("Missing port name");
				return GCodeResult::error;
			}
			boardAddress = IoPort::RemoveBoardAddress(portName.GetRef());
		}
		else
		{
			boardAddress = CanId::NoAddress;
		}
#endif
		TemperatureSensor *sensor;
		bool newSensor = gb.Seen('Y');
		if (newSensor)
		{
			TemperatureSensor * const oldSensor = GetSensor(sensorNum);
			if (oldSensor != nullptr)
			{
				oldSensor->FlagForDeletion();
			}

			String<StringLength20> typeName;
			if (!gb.GetReducedString(typeName.GetRef()))
			{
				reply.copy("Missing sensor type name");
				return GCodeResult::error;
			}
#if SUPPORT_CAN_EXPANSION
			if (boardAddress == CanId::NoAddress)
			{
				reply.copy("Missing port name");
				return GCodeResult::error;
			}
			sensor = TemperatureSensor::Create(sensorNum, boardAddress, typeName.c_str(), reply);
#else
			sensor = TemperatureSensor::Create(sensorNum, typeName.c_str(), reply);
#endif
			if (sensor == nullptr)
			{
				return GCodeResult::error;
			}
		}
		else
		{
			sensor = GetSensor(sensorNum);
			if (sensor == nullptr)
			{
				reply.printf("Sensor %u does not exist", sensorNum);
				return GCodeResult::error;
			}
#if SUPPORT_CAN_EXPANSION
			const int existingBoardAddress = sensor->GetBoardAddress();
			if (boardAddress == CanId::NoAddress)
			{
				boardAddress = existingBoardAddress;
			}
			else if (boardAddress != existingBoardAddress)
			{
				reply.copy("Sensor type parameter needed when moving a sensor to a different board");
				return GCodeResult::error;
			}
#endif
		}

		const GCodeResult rslt = sensor->Configure(gb, reply);
		if (newSensor)
		{
			if (rslt == GCodeResult::ok)
			{
				TaskCriticalSectionLocker lock;

				sensor->SetNext(newSensors);
				newSensors = sensor;
			}
			else
			{
				delete sensor;
			}
		}
		return rslt;
	}

	reply.copy("Missing sensor number parameter");
	return GCodeResult::error;
}

// Get the name of a heater, or nullptr if it hasn't been named
const char *Heat::GetHeaterSensorName(size_t heater) const
{
	Heater * const h = FindHeater(heater);
	return (h != nullptr) ? h->GetSensorName() : nullptr;
}

// Return the protection parameters of the given index
HeaterProtection& Heat::AccessHeaterProtection(size_t index) const
{
	if (index >= FirstExtraHeaterProtection && index < FirstExtraHeaterProtection + NumExtraHeaterProtections)
	{
		return *heaterProtections[index + MaxHeaters - FirstExtraHeaterProtection];
	}
	return *heaterProtections[index];
}

// Updates the PIDs and HeaterProtection items after a heater change
void Heat::UpdateHeaterProtection()
{
	// Reassign the first mapped heater protection item of each PID where applicable
	// and rebuild the linked list of heater protection elements per heater
	for (size_t heater : ARRAY_INDICES(heaters))
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
		if (heaters[heater] != nullptr)
		{
			heaters[heater]->SetHeaterProtection(firstProtectionItem);
		}
	}
}

// Get the temperature of a sensor
float Heat::GetSensorTemperature(int sensorNum, TemperatureError& err) const
{
	TemperatureSensor * const sensor = GetSensor(sensorNum);
	if (sensor != nullptr)
	{
		float temp;
		err = sensor->GetLatestTemperature(temp);
		return temp;
	}

	err = TemperatureError::unknownSensor;
	return BadErrorTemperature;
}

// Get the temperature of a heater
float Heat::GetHeaterTemperature(size_t heater) const
{
	Heater * const h = FindHeater(heater);
	return (h == nullptr) ? ABS_ZERO : h->GetTemperature();
}

// Suspend the heaters to conserve power or while doing Z probing
void Heat::SuspendHeaters(bool sus)
{
	for (Heater *h : heaters)
	{
		if (h != nullptr)
		{
			h->Suspend(sus);
		}
	}
}

#if HAS_MASS_STORAGE

// Save some resume information returning true if successful.
// We assume that the bed and chamber heaters are either on and active, or off (not on standby).
bool Heat::WriteBedAndChamberTempSettings(FileStore *f) const
{
	String<100> bufSpace;
	const StringRef buf = bufSpace.GetRef();
	for (size_t index : ARRAY_INDICES(bedHeaters))
	{
		Heater * const h = FindHeater(bedHeaters[index]);
		if (h != nullptr && h->GetStatus() == HeaterStatus::active)
		{
			buf.printf("M140 P%u S%.1f\n", index, (double)h->GetActiveTemperature());
		}
	}
	for (size_t index : ARRAY_INDICES(chamberHeaters))
	{
		Heater * const h = FindHeater(chamberHeaters[index]);
		if (h != nullptr && h->GetStatus() == HeaterStatus::active)
		{
			buf.printf("M141 P%u S%.1f\n", index, (double)h->GetActiveTemperature());
		}
	}
	return (buf.strlen() == 0) || f->Write(buf.c_str());
}

#endif

#if SUPPORT_CAN_EXPANSION

void Heat::UpdateRemoteSensorTemperature(unsigned int sensor, const CanTemperatureReport& report)
{
	TemperatureSensor * const ts = GetSensor(sensor);
	if (ts != nullptr)
	{
		ts->UpdateRemoteTemperature(report);
	}
}

#endif

// End
