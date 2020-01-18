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
#include <TaskPriorities.h>

#if SUPPORT_DHT_SENSOR
# include "Sensors/DhtSensor.h"
#endif

#if SUPPORT_CAN_EXPANSION
# include "RemoteHeater.h"
# include <CanId.h>
# include <CanMessageFormats.h>
# include <CanMessageBuffer.h>
# include "CAN/CanInterface.h"
#endif

constexpr uint32_t HeaterTaskStackWords = 400;			// task stack size in dwords, must be large enough for auto tuning
static Task<HeaterTaskStackWords> heaterTask;

extern "C" [[noreturn]] void HeaterTaskStart(void * pvParameters) noexcept
{
	reprap.GetHeat().HeaterTask();
}

static constexpr uint16_t SensorsTaskStackWords = 100;		// task stack size in dwords. 80 was not enough. Use 300 if debugging is enabled.
static Task<SensorsTaskStackWords> *sensorsTask = nullptr;

extern "C" [[noreturn]] void SensorsTaskStart(void * pvParameters) noexcept
{
	reprap.GetHeat().SensorsTask();
}

#if SUPPORT_OBJECT_MODEL
// Object model table and functions
// Note: if using GCC version 7.3.1 20180622 and lambda functions are used in this table, you must compile this file with option -std=gnu++17.
// Otherwise the table will be allocated in RAM instead of flash, which wastes too much RAM.

constexpr ObjectModelArrayDescriptor Heat::heatersArrayDescriptor =
{
	&heatersLock,
	[] (const ObjectModel *self, const ObjectExplorationContext&) noexcept -> size_t { return ((const Heat*)self)->GetNumHeatersToReport(); },
	[] (const ObjectModel *self, ObjectExplorationContext& context) noexcept -> ExpressionValue { return ExpressionValue(((const Heat*)self)->heaters[context.GetLastIndex()]); }
};

constexpr ObjectModelArrayDescriptor Heat::sensorsArrayDescriptor =
{
	&sensorsLock,
	[] (const ObjectModel *self, const ObjectExplorationContext&) noexcept -> size_t { return ((const Heat*)self)->GetNumSensorsToReport(); },
	[] (const ObjectModel *self, ObjectExplorationContext& context) noexcept -> ExpressionValue { return ExpressionValue(((const Heat*)self)->FindSensor(context.GetLastIndex()).Ptr()); }
};

// Macro to build a standard lambda function that includes the necessary type conversions
#define OBJECT_MODEL_FUNC(...) OBJECT_MODEL_FUNC_BODY(Heat, __VA_ARGS__)

constexpr ObjectModelTableEntry Heat::objectModelTable[] =
{
	// These entries must be in alphabetical order
	// 0. Heat class
	{ "coldExtrudeTemperature",	OBJECT_MODEL_FUNC(self->extrusionMinTemp, 1),		ObjectModelEntryFlags::none},
	{ "coldRetractTemperature", OBJECT_MODEL_FUNC(self->retractionMinTemp, 1),		ObjectModelEntryFlags::none},
	{ "heaters",				OBJECT_MODEL_FUNC_NOSELF(&heatersArrayDescriptor),	ObjectModelEntryFlags::live },
	{ "sensors",				OBJECT_MODEL_FUNC_NOSELF(&sensorsArrayDescriptor),	ObjectModelEntryFlags::live },
};

constexpr uint8_t Heat::objectModelTableDescriptor[] = { 1, 4 };

DEFINE_GET_OBJECT_MODEL_TABLE(Heat)

#endif

ReadWriteLock Heat::heatersLock;
ReadWriteLock Heat::sensorsLock;

Heat::Heat() noexcept
	: sensorCount(0), sensorsRoot(nullptr), coldExtrude(false), heaterBeingTuned(-1), lastHeaterTuned(-1)
{
	for (int8_t& h : bedHeaters)
	{
		h = -1;
	}
#if !defined(DUET3)
	bedHeaters[0] = DefaultBedHeater;
#endif

	for (int8_t& h : chamberHeaters)
	{
		h = -1;
	}

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

ReadLockedPointer<Heater> Heat::FindHeater(int heater) const noexcept
{
	ReadLocker locker(heatersLock);
	return ReadLockedPointer<Heater>(locker, (heater < 0 || heater >= (int)MaxHeaters) ? nullptr : heaters[heater]);
}

// Process M307
GCodeResult Heat::SetOrReportHeaterModel(GCodeBuffer& gb, const StringRef& reply)
{
	gb.MustSee('H');
	const unsigned int heater = gb.GetUIValue();
	const auto h = FindHeater(heater);
	if (h.IsNotNull())
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

// Process M301 or M304. 'heater' is the default heater number to use.
GCodeResult Heat::SetPidParameters(unsigned int heater, GCodeBuffer& gb, const StringRef& reply)
{
	if (gb.Seen('H'))
	{
		heater = gb.GetUIValue();
	}

	const auto h = FindHeater(heater);
	if (h.IsNotNull())
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
bool Heat::IsHeaterEnabled(size_t heater) const noexcept
{
	const auto h = FindHeater(heater);
	return h.IsNotNull() && h->IsHeaterEnabled();
}

// Get a pointer to the temperature sensor entry, or nullptr if the heater number is bad
ReadLockedPointer<TemperatureSensor> Heat::FindSensor(int sn) const noexcept
{
	ReadLocker locker(sensorsLock);

	for (TemperatureSensor *sensor = sensorsRoot; sensor != nullptr; sensor = sensor->GetNext())
	{
		if ((int)sensor->GetSensorNumber() == sn)
		{
			return ReadLockedPointer<TemperatureSensor>(locker, sensor);
		}
		if ((int)sensor->GetSensorNumber() > sn)
		{
			break;
		}
	}
	return ReadLockedPointer<TemperatureSensor>(locker, nullptr);
}

// Get a pointer to the first temperature sensor with the specified or higher number
ReadLockedPointer<TemperatureSensor> Heat::FindSensorAtOrAbove(unsigned int sn) const noexcept
{
	ReadLocker locker(sensorsLock);

	for (TemperatureSensor *sensor = sensorsRoot; sensor != nullptr; sensor = sensor->GetNext())
	{
		if (sensor->GetSensorNumber() >= sn)
		{
			return ReadLockedPointer<TemperatureSensor>(locker, sensor);
		}
	}
	return ReadLockedPointer<TemperatureSensor>(locker, nullptr);
}


// Reset all heater models to defaults. Called when running M502.
void Heat::ResetHeaterModels() noexcept
{
	ReadLocker lock(heatersLock);

	for (Heater* h : heaters)
	{
		if (h != nullptr && h->IsHeaterEnabled())
		{
			h->SetModelDefaults();
		}
	}
}

void Heat::Init() noexcept
{
	// Initialise the heater protection items first
	for (size_t index : ARRAY_INDICES(heaterProtections))
	{
		HeaterProtection * const prot = heaterProtections[index];

		const float tempLimit = (IsBedOrChamberHeater(index)) ? DefaultBedTemperatureLimit : DefaultHotEndTemperatureLimit;
		prot->Init(tempLimit);
	}

	extrusionMinTemp = HOT_ENOUGH_TO_EXTRUDE;
	retractionMinTemp = HOT_ENOUGH_TO_RETRACT;
	coldExtrude = false;

	heaterTask.Create(HeaterTaskStart, "HEAT", nullptr, TaskPriority::HeatPriority);
}

void Heat::Exit() noexcept
{
	{
		ReadLocker Lock(heatersLock);

		for (Heater *h : heaters)
		{
			if (h != nullptr)
			{
				h->SwitchOff();
			}
		}
	}

	heaterTask.Suspend();
}

[[noreturn]] void Heat::HeaterTask() noexcept
{
	uint32_t lastWakeTime = xTaskGetTickCount();
	for (;;)
	{
		// Walk the sensor list and poll all sensors
		{
			ReadLocker lock(sensorsLock);
			TemperatureSensor *currentSensor = sensorsRoot;
			while (currentSensor != nullptr)
			{
				currentSensor->Poll();
				currentSensor = currentSensor->GetNext();
			}
		}

		// Spin the heaters
		{
			ReadLocker lock(heatersLock);
			for (Heater *h : heaters)
			{
				if (h != nullptr)
				{
					h->Spin();
				}
			}
		}

		// See if we have finished tuning a PID
		if (heaterBeingTuned != -1)
		{
			const auto h = FindHeater(heaterBeingTuned);
			if (h.IsNull() || h->GetStatus() != HeaterStatus::tuning)
			{
				lastHeaterTuned = heaterBeingTuned;
				heaterBeingTuned = -1;
			}
		}

		reprap.KickHeatTaskWatchdog();

#if SUPPORT_CAN_EXPANSION
		// Broadcast our sensor temperatures
		CanMessageBuffer * buf = CanMessageBuffer::Allocate();
		if (buf != nullptr)
		{
			CanMessageSensorTemperatures * const msg = buf->SetupBroadcastMessage<CanMessageSensorTemperatures>(CanInterface::GetCanAddress());
			msg->whichSensors = 0;
			unsigned int sensorsFound = 0;
			unsigned int currentSensorNumber = 0;
			for (;;)
			{
				const auto sensor = FindSensorAtOrAbove(currentSensorNumber);
				if (sensor.IsNull())
				{
					break;
				}
				const unsigned int sn = sensor->GetSensorNumber();
				if (sensor->GetBoardAddress() == CanInterface::GetCanAddress())
				{
					msg->whichSensors |= (uint64_t)1u << sn;
					float temperature;
					msg->temperatureReports[sensorsFound].errorCode = (uint8_t)sensor->GetLatestTemperature(temperature);
					msg->temperatureReports[sensorsFound].temperature = temperature;
					++sensorsFound;
				}
				currentSensorNumber = (unsigned int)sn + 1u;
			}
			if (sensorsFound == 0)
			{
				// Don't send an empty report
				CanMessageBuffer::Free(buf);
			}
			else
			{
				buf->dataLength = msg->GetActualDataLength(sensorsFound);
				CanInterface::SendBroadcast(buf);
			}
		}
#endif

		// Delay until it is time again
		vTaskDelayUntil(&lastWakeTime, HeatSampleIntervalMillis);
	}
}


/* static */ void Heat::EnsureSensorsTask() noexcept
{
	TaskCriticalSectionLocker lock; // make sure we don't create the task more than once

	if (sensorsTask == nullptr)
	{
		sensorsTask = new Task<SensorsTaskStackWords>;
		sensorsTask->Create(SensorsTaskStart, "SENSORS", nullptr, TaskPriority::SensorsPriority);
	}
}

// Code executed by the SensorsTask.
// This is run at the same priority as the Heat task, so it must not sit in any spin loops.
/*static*/ [[noreturn]] void Heat::SensorsTask() noexcept
{
	auto lastWakeTime = xTaskGetTickCount();
	for (;;)
	{
		unsigned int sensorNumber = 0;
		uint8_t sensorCountSinceLastDelay = 0;
		uint8_t totalWaitTime = 0;
		// Walk the sensor list one by one and poll each sensor
		for (;;)
		{
			uint8_t delay = 0;

			// We need this block to have the ReadLockPointer below go out of scope as early as possible
			{
				const auto sensor = FindSensorAtOrAbove(sensorNumber);

				// End of the list reached - start over after short delay
				if (sensor.IsNull())
				{
					break;
				}
				++sensorCountSinceLastDelay;
				sensorNumber = sensor->GetSensorNumber() + 1;

				if (sensor->PollInTask())
				{
					// Coming here sensorCount cannot be 0 since we got at least one sensor returning true
					delay = ((SensorsTaskTotalDelay/sensorCount)*sensorCountSinceLastDelay);
				}
			}

			if (delay > 0)
			{
				vTaskDelayUntil(&lastWakeTime, delay);
				totalWaitTime += delay;
			}
		}

		// Delay until it is time again
		if (totalWaitTime < SensorsTaskTotalDelay)
		{
			vTaskDelayUntil(&lastWakeTime, (SensorsTaskTotalDelay-totalWaitTime));
		}
	}
}

void Heat::Diagnostics(MessageType mtype) noexcept
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
		bool found;
		float acc;
		{
			const auto h = FindHeater(heater);
			found = h.IsNotNull() && h->GetStatus() == HeaterStatus::active;
			if (found)
			{
				acc = h->GetAccumulator();
			}
		}
		if (found)
		{
			platform.MessageF(mtype, "Heater %u is on, I-accum = %.1f\n", heater, (double)acc);
		}
	}
}

// Configure a heater. Invoked by M950.
GCodeResult Heat::ConfigureHeater(size_t heater, GCodeBuffer& gb, const StringRef& reply)
{
	if (heater >= MaxHeaters)
	{
		reply.copy("Heater number out of range");
		return GCodeResult::error;
	}

	if (gb.Seen('C'))
	{
		String<StringLength50> pinName;
		gb.GetReducedString(pinName.GetRef());

#if SUPPORT_CAN_EXPANSION
		const CanAddress board = IoPort::RemoveBoardAddress(pinName.GetRef());
#endif
		if (StringEqualsIgnoreCase(pinName.c_str(), NoPinName))
		{
			// Setting the pin name to "nil" deletes the heater
			WriteLocker lock(heatersLock);
			Heater *oldHeater = nullptr;
			std::swap(oldHeater, heaters[heater]);
			delete oldHeater;
			return GCodeResult::ok;
		}

		if (!gb.Seen('T'))
		{
			reply.copy("Missing sensor number");
			return GCodeResult::error;
		}
		const unsigned int sensorNumber = gb.GetUIValue();

		WriteLocker lock(heatersLock);

		Heater *oldHeater = nullptr;
		std::swap(oldHeater, heaters[heater]);
		delete oldHeater;

		const PwmFrequency freq = (gb.Seen('Q')) ? gb.GetPwmFrequency() : DefaultFanPwmFreq;

#if SUPPORT_CAN_EXPANSION
		Heater * const newHeater = (board != CanId::MasterAddress) ? (Heater *)new RemoteHeater(heater, board) : new LocalHeater(heater);
#else
		Heater * const newHeater = new LocalHeater(heater);
#endif
		const GCodeResult rslt = newHeater->ConfigurePortAndSensor(pinName.c_str(), freq, sensorNumber, reply);
		if (rslt == GCodeResult::ok || rslt == GCodeResult::warning)
		{
			heaters[heater] = newHeater;
		}
		else
		{
			delete newHeater;
		}
		return rslt;
	}

	if (gb.Seen('T'))
	{
		reply.copy("Can't change sensor number of existing heater");
		return GCodeResult::error;
	}

	const auto h = FindHeater(heater);
	if (h.IsNull())
	{
		reply.printf("Heater %u does not exist", (unsigned int)heater);
		return GCodeResult::error;
	}

	if (gb.Seen('Q'))
	{
		return h->SetPwmFrequency(gb.GetPwmFrequency(), reply);
	}

	return h->ReportDetails(reply);
}

bool Heat::AllHeatersAtSetTemperatures(bool includingBed, float tolerance) const noexcept
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
bool Heat::HeaterAtSetTemperature(int heater, bool waitWhenCooling, float tolerance) const noexcept
{
	const auto h = FindHeater(heater);
	if (h.IsNotNull())
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

HeaterStatus Heat::GetStatus(int heater) const noexcept
{
	const auto h = FindHeater(heater);
	return (h.IsNull()) ? HeaterStatus::off : heaters[heater]->GetStatus();
}

void Heat::SetBedHeater(size_t index, int heater) noexcept
{
	const auto h = FindHeater(bedHeaters[index]);
	if (h.IsNotNull())
	{
		h->SwitchOff();
	}
	bedHeaters[index] = heater;
}

bool Heat::IsBedHeater(int heater) const noexcept
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

void Heat::SetChamberHeater(size_t index, int heater) noexcept
{
	const auto h = FindHeater(chamberHeaters[index]);
	if (h.IsNotNull())
	{
		h->SwitchOff();
	}
	chamberHeaters[index] = heater;
}

bool Heat::IsChamberHeater(int heater) const noexcept
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

void Heat::SetActiveTemperature(int heater, float t) noexcept
{
	const auto h = FindHeater(heater);
	if (h.IsNotNull())
	{
		h->SetActiveTemperature(t);
	}
}

float Heat::GetActiveTemperature(int heater) const noexcept
{
	const auto h = FindHeater(heater);
	return (h.IsNull()) ? ABS_ZERO : h->GetActiveTemperature();
}

void Heat::SetStandbyTemperature(int heater, float t) noexcept
{
	const auto h = FindHeater(heater);
	if (h.IsNotNull())
	{
		h->SetStandbyTemperature(t);
	}
}

float Heat::GetStandbyTemperature(int heater) const noexcept
{
	const auto h = FindHeater(heater);
	return (h.IsNull()) ? ABS_ZERO : h->GetStandbyTemperature();
}

float Heat::GetHighestTemperatureLimit(int heater) const noexcept
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

float Heat::GetLowestTemperatureLimit(int heater) const noexcept
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
float Heat::GetHeaterTemperature(int heater) const noexcept
{
	const auto h = FindHeater(heater);
	return (h.IsNull()) ? ABS_ZERO : h->GetTemperature();
}

// Get the target temperature of a heater
float Heat::GetTargetTemperature(int heater) const noexcept
{
	const HeaterStatus hs = GetStatus(heater);
	return (hs == HeaterStatus::active) ? GetActiveTemperature(heater)
			: (hs == HeaterStatus::standby) ? GetStandbyTemperature(heater)
				: 0.0;
}

GCodeResult Heat::Activate(int heater, const StringRef& reply) noexcept
{
	const auto h = FindHeater(heater);
	if (h.IsNotNull())
	{
		return h->Activate(reply);
	}
	reply.copy("Heater %d not found", heater);
	return GCodeResult::error;
}

void Heat::SwitchOff(int heater) noexcept
{
	const auto h = FindHeater(heater);
	if (h.IsNotNull())
	{
		h->SwitchOff();
		lastStandbyTools[heater] = nullptr;
	}
}

void Heat::SwitchOffAll(bool includingChamberAndBed) noexcept
{
	ReadLocker lock(heatersLock);

	for (int heater = 0; heater < (int)MaxHeaters; ++heater)
	{
		Heater * const h = heaters[heater];
		if (h != nullptr && (includingChamberAndBed || !IsBedOrChamberHeater(heater)))
		{
			h->SwitchOff();
		}
	}
}

void Heat::Standby(int heater, const Tool *tool) noexcept
{
	const auto h = FindHeater(heater);
	if (h.IsNotNull())
	{
		h->Standby();
		lastStandbyTools[heater] = tool;
	}
}

GCodeResult Heat::ResetFault(int heater, const StringRef& reply) noexcept
{
	// This gets called for all heater numbers when clearing all temperature faults, so don't report an error if the heater was not found
	const auto h = FindHeater(heater);
	return (h.IsNotNull()) ? h->ResetFault(reply) : GCodeResult::ok;
}

float Heat::GetAveragePWM(size_t heater) const noexcept
{
	const auto h = FindHeater(heater);
	return (h.IsNull()) ? 0.0 : h->GetAveragePWM();
}

bool Heat::IsBedOrChamberHeater(int heater) const noexcept
{
	return IsBedHeater(heater) || IsChamberHeater(heater);
}

// Get the highest temperature limit of any heater
float Heat::GetHighestTemperatureLimit() const noexcept
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
bool Heat::WriteModelParameters(FileStore *f) const noexcept
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
	const auto h = FindHeater(heater);
	if (h.IsNull())
	{
		reply.printf("Heater %u not found", heater);
		return GCodeResult::error;
	}

	bool seenValue = false;
	float maxTempExcursion, maxFaultTime;
	h->GetFaultDetectionParameters(maxTempExcursion, maxFaultTime);
	gb.TryGetFValue('P', maxFaultTime, seenValue);
	gb.TryGetFValue('T', maxTempExcursion, seenValue);
	if (seenValue)
	{
		return h->SetFaultDetectionParameters(maxTempExcursion, maxFaultTime, reply);
	}

	reply.printf("Heater %u allowed excursion %.1f" DEGREE_SYMBOL "C, fault trigger time %.1f seconds", heater, (double)maxTempExcursion, (double)maxFaultTime);
	return GCodeResult::ok;
}

// Process M303
GCodeResult Heat::TuneHeater(GCodeBuffer& gb, const StringRef& reply)
{
	if (gb.Seen('H'))
	{
		const unsigned int heater = gb.GetUIValue();
		const auto h = FindHeater(heater);
		if (h.IsNotNull())
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
		const auto h = FindHeater(whichPid);

		if (h.IsNotNull())
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
	gb.MustSee('S');
	const unsigned sensorNum = gb.GetUIValue();
	if (sensorNum >= MaxSensors)
	{
		reply.copy("Sensor number out of range");
		return GCodeResult::error;
	}

#if SUPPORT_CAN_EXPANSION
	// Set boardAddress to the board number that the port is on, or NoAddress if the port was not given
	CanAddress boardAddress;
	String<StringLength20> portName;
	if (gb.Seen('P'))
	{
		gb.GetReducedString(portName.GetRef());
		boardAddress = IoPort::RemoveBoardAddress(portName.GetRef());
	}
	else
	{
		boardAddress = CanId::NoAddress;
	}
#endif
	if (gb.Seen('Y'))
	{
		// Creating a new sensor
		WriteLocker lock(sensorsLock);

		DeleteSensor(sensorNum);

		String<StringLength20> typeName;
		gb.GetReducedString(typeName.GetRef());

#if SUPPORT_CAN_EXPANSION
		if (boardAddress == CanId::NoAddress)
		{
			boardAddress = CanId::MasterAddress;		// no port name was given, so default to master
		}
		TemperatureSensor * const newSensor = TemperatureSensor::Create(sensorNum, boardAddress, typeName.c_str(), reply);
#else
		TemperatureSensor * const newSensor = TemperatureSensor::Create(sensorNum, typeName.c_str(), reply);
#endif
		if (newSensor == nullptr)
		{
			return GCodeResult::error;
		}

		const GCodeResult rslt = newSensor->Configure(gb, reply);
		if (rslt == GCodeResult::ok)
		{
			InsertSensor(newSensor);
		}
		else
		{
			delete newSensor;
		}
		return rslt;
	}

	// Modifying or reporting on an existing sensor
	const auto sensor = FindSensor(sensorNum);
	if (sensor.IsNull())
	{
		reply.printf("Sensor %u does not exist", sensorNum);
		return GCodeResult::error;
	}

#if SUPPORT_CAN_EXPANSION
	const CanAddress existingBoardAddress = sensor->GetBoardAddress();
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
	return sensor->Configure(gb, reply);
}

// Get the name of a heater, or nullptr if it hasn't been named
const char *Heat::GetHeaterSensorName(size_t heater) const noexcept
{
	const auto h = FindHeater(heater);
	return (h.IsNotNull()) ? h->GetSensorName() : nullptr;
}

// Configure heater protection (M143). Returns true if an error occurred
GCodeResult Heat::SetHeaterProtection(GCodeBuffer& gb, const StringRef& reply)
{
	WriteLocker lock(heatersLock);

	bool seen = false;
	int32_t heaterNumber = 1;			// default to extruder 1 if no heater number provided
	gb.TryGetIValue('H', heaterNumber, seen);
	const int index = (gb.Seen('P')) ? gb.GetIValue() : heaterNumber;

	if (   index < 0
		|| (index >= (int)MaxHeaters && index < (int)FirstExtraHeaterProtection)
		|| index >= (int)(FirstExtraHeaterProtection + MaxExtraHeaterProtections)
	   )
	{
		reply.printf("Invalid heater protection item '%d'", index);
		return GCodeResult::error;
	}

	HeaterProtection &item = (index >= (int)FirstExtraHeaterProtection)
								? *heaterProtections[index - FirstExtraHeaterProtection + MaxHeaters]
									: *heaterProtections[index];
	// Set heater to control
	if (seen && heaterNumber != item.GetHeater())
	{
		const int oldHeaterNumber = item.GetHeater();
		item.SetHeater(heaterNumber);
		UpdateHeaterProtection(oldHeaterNumber);
		UpdateHeaterProtection(heaterNumber);
	}

	// Set sensor that supervises the heater
	if (gb.Seen('X'))
	{
		item.SetSensorNumber(gb.GetIValue());
		seen = true;
	}

	// Set trigger action
	if (gb.Seen('A'))
	{
		const int action = gb.GetIValue();
		if (action < 0 || action > (int)MaxHeaterProtectionAction)
		{
			reply.printf("Invalid heater protection action '%d'", action);
		}

		seen = true;
		item.SetAction(static_cast<HeaterProtectionAction>(action));
	}

	// Set trigger condition
	if (gb.Seen('C'))
	{
		const int trigger = gb.GetIValue();
		if (trigger < 0 || trigger > (int)MaxHeaterProtectionTrigger)
		{
			reply.printf("Invalid heater protection trigger '%d'", trigger);
		}

		seen = true;
		item.SetTrigger(static_cast<HeaterProtectionTrigger>(trigger));
	}

	// Set temperature limit
	if (gb.Seen('S'))
	{
		const float limit = gb.GetFValue();
		if (limit <= BadLowTemperature || limit >= BadErrorTemperature)
		{
			reply.copy("Invalid temperature limit");
			return GCodeResult::error;
		}

		seen = true;
		item.SetTemperatureLimit(limit);
	}

	// Report current parameters
	if (!seen)
	{
		if (item.GetHeater() < 0)
		{
			reply.printf("Temperature protection item %d is not configured", index);
		}
		else
		{
			const char *actionString, *triggerString;
			switch (item.GetAction())
			{
			case HeaterProtectionAction::GenerateFault:
				actionString = "generate a heater fault";
				break;
			case HeaterProtectionAction::PermanentSwitchOff:
				actionString = "permanently switch off";
				break;
			case HeaterProtectionAction::TemporarySwitchOff:
				actionString = "temporarily switch off";
				break;
			default:
				actionString = "(undefined)";
				break;
			}

			switch (item.GetTrigger())
			{
			case HeaterProtectionTrigger::TemperatureExceeded:
				triggerString = "exceeds";
				break;
			case HeaterProtectionTrigger::TemperatureTooLow:
				triggerString = "falls below";
				break;
			default:
				triggerString = "(undefined)";
				break;
			}

			reply.printf("Temperature protection item %d is configured for heater %d and uses sensor %d to %s if the temperature %s %.1f" DEGREE_SYMBOL "C",
					index, item.GetHeater(), item.GetSensorNumber(), actionString, triggerString, (double)item.GetTemperatureLimit());
		}
	}

	return GCodeResult::ok;
}

// Updates the PIDs and HeaterProtection items after a heater change. Caller must already have a write lock on the heaters.
void Heat::UpdateHeaterProtection(int heaterNumber) noexcept
{
	auto h = FindHeater(heaterNumber);
	if (h.IsNotNull())
	{
		// Rebuild linked lists
		h->SetHeaterProtection(nullptr);
		HeaterProtection *firstProtectionItem = nullptr;
		HeaterProtection *lastElementInList = nullptr;
		for (HeaterProtection *prot : heaterProtections)
		{
			if (prot->GetHeater() == heaterNumber)
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

		h->SetHeaterProtection(firstProtectionItem);
	}
}

// Get the temperature of a sensor
float Heat::GetSensorTemperature(int sensorNum, TemperatureError& err) const noexcept
{
	const auto sensor = FindSensor(sensorNum);
	if (sensor.IsNotNull())
	{
		float temp;
		err = sensor->GetLatestTemperature(temp);
		return temp;
	}

	err = TemperatureError::unknownSensor;
	return BadErrorTemperature;
}

// Return the highest used heater number plus one. Used by RepRap.cpp to shorten responses by omitting unused trailing heater numbers.
size_t Heat::GetNumHeatersToReport() const noexcept
{
	size_t ret = ARRAY_SIZE(heaters);
	while (ret != 0 && heaters[ret - 1] == nullptr)
	{
		--ret;
	}
	return ret;
}

// Return the highest used sensor number plus one
size_t Heat::GetNumSensorsToReport() const noexcept
{
	ReadLocker lock(sensorsLock);

	const TemperatureSensor *s = sensorsRoot;
	if (s == nullptr)
	{
		return 0;
	}

	// Find the last sensor in the list
	while (s->GetNext() != nullptr)
	{
		s = s->GetNext();
	}
	return s->GetSensorNumber() + 1;
}

// Get the temperature of a heater
float Heat::GetHeaterTemperature(size_t heater) const noexcept
{
	const auto h = FindHeater(heater);
	return (h.IsNull()) ? ABS_ZERO : h->GetTemperature();
}

// Suspend the heaters to conserve power or while doing Z probing
void Heat::SuspendHeaters(bool sus) noexcept
{
	for (Heater *h : heaters)
	{
		if (h != nullptr)
		{
			h->Suspend(sus);
		}
	}
}

// Delete a sensor, if there is one. Must write-lock the sensors lock before calling this.
void Heat::DeleteSensor(unsigned int sn) noexcept
{
	TemperatureSensor *currentSensor = sensorsRoot;
	TemperatureSensor *lastSensor = nullptr;

	while (currentSensor != nullptr)
	{
		if (currentSensor->GetSensorNumber() == sn)
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
			--sensorCount;
			break;
		}

		lastSensor = currentSensor;
		currentSensor = currentSensor->GetNext();
	}
}

// Insert a sensor. Must write-lock the sensors lock before calling this.
void Heat::InsertSensor(TemperatureSensor *newSensor) noexcept
{
	TemperatureSensor *prev = nullptr;
	TemperatureSensor *ts = sensorsRoot;
	for (;;)
	{
		if (ts == nullptr || ts->GetSensorNumber() > newSensor->GetSensorNumber())
		{
			newSensor->SetNext(ts);
			if (prev == nullptr)
			{
				sensorsRoot = newSensor;
			}
			else
			{
				prev->SetNext(newSensor);
			}
			++sensorCount;
			break;
		}
		prev = ts;
		ts = ts->GetNext();
	}
}

#if HAS_MASS_STORAGE

// Save some resume information returning true if successful.
// We assume that the bed and chamber heaters are either on and active, or off (not on standby).
bool Heat::WriteBedAndChamberTempSettings(FileStore *f) const noexcept
{
	String<100> bufSpace;
	const StringRef buf = bufSpace.GetRef();
	for (size_t index : ARRAY_INDICES(bedHeaters))
	{
		const auto h = FindHeater(bedHeaters[index]);
		if (h.IsNotNull() && h->GetStatus() == HeaterStatus::active)
		{
			buf.printf("M140 P%u S%.1f\n", index, (double)h->GetActiveTemperature());
		}
	}
	for (size_t index : ARRAY_INDICES(chamberHeaters))
	{
		const auto h = FindHeater(chamberHeaters[index]);
		if (h.IsNotNull() && h->GetStatus() == HeaterStatus::active)
		{
			buf.printf("M141 P%u S%.1f\n", index, (double)h->GetActiveTemperature());
		}
	}
	return (buf.strlen() == 0) || f->Write(buf.c_str());
}

#endif

#if SUPPORT_CAN_EXPANSION

void Heat::ProcessRemoteSensorsReport(CanAddress src, const CanMessageSensorTemperatures& msg) noexcept
{
	uint64_t sensorsReported = msg.whichSensors;
	size_t index = 0;
	for (unsigned int sensor = 0; sensor < 64 && sensorsReported != 0; ++sensor)
	{
		if (((uint8_t)sensorsReported & 1u) != 0)
		{
			if (index < ARRAY_SIZE(msg.temperatureReports))
			{
				const auto ts = FindSensor(sensor);
				if (ts.IsNotNull())
				{
					ts->UpdateRemoteTemperature(src, msg.temperatureReports[index]);
				}
			}
			++index;
		}
		sensorsReported >>= 1;
	}
}

void Heat::ProcessRemoteHeatersReport(CanAddress src, const CanMessageHeatersStatus& msg) noexcept
{
	uint64_t heatersReported = msg.whichHeaters;
	size_t index = 0;
	for (unsigned int heaterNum = 0; heaterNum < 64 && heatersReported != 0; ++heaterNum)
	{
		if (((uint8_t)heatersReported & 1u) != 0)
		{
			if (index < ARRAY_SIZE(msg.reports))
			{
				const auto h = FindHeater(heaterNum);
				if (h.IsNotNull())
				{
					h->UpdateRemoteStatus(src, msg.reports[index]);
				}
			}
			++index;
		}
		heatersReported >>= 1;
	}
}

#endif

// End
