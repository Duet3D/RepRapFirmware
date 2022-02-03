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
#include "HeaterMonitor.h"
#include <Platform/Platform.h>
#include <Platform/RepRap.h>
#include "Sensors/TemperatureSensor.h"
#include <GCodes/GCodeBuffer/GCodeBuffer.h>
#include <Tools/Tool.h>
#include <Platform/TaskPriorities.h>
#include <General/Portability.h>

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

#if SUPPORT_REMOTE_COMMANDS
# include <CanMessageGenericParser.h>
# include <CanMessageGenericTables.h>
# include "Sensors/RemoteSensor.h"
#endif

#ifdef DUET3_ATE
# include <Duet3Ate.h>
#endif

#if SUPPORT_CAN_EXPANSION
constexpr uint32_t HeaterTaskStackWords = 470;			// task stack size in dwords, must be large enough for auto tuning and a local CAN buffer
#else
constexpr uint32_t HeaterTaskStackWords = 420;			// task stack size in dwords, must be large enough for auto tuning. 400 was not quite enough for one Duet WiFi user running 3.2.2.
#endif

static Task<HeaterTaskStackWords> heaterTask;

extern "C" [[noreturn]] void HeaterTaskStart(void * pvParameters) noexcept
{
	reprap.GetHeat().HeaterTask();
}

#if SUPPORT_OBJECT_MODEL
// Object model table and functions
// Note: if using GCC version 7.3.1 20180622 and lambda functions are used in this table, you must compile this file with option -std=gnu++17.
// Otherwise the table will be allocated in RAM instead of flash, which wastes too much RAM.

constexpr ObjectModelArrayDescriptor Heat::bedHeatersArrayDescriptor =
{
	&heatersLock,
	[] (const ObjectModel *self, const ObjectExplorationContext&) noexcept -> size_t { return MaxBedHeaters; },
	[] (const ObjectModel *self, ObjectExplorationContext& context) noexcept -> ExpressionValue { return ExpressionValue((int32_t)((const Heat*)self)->bedHeaters[context.GetLastIndex()]); }
};

constexpr ObjectModelArrayDescriptor Heat::chamberHeatersArrayDescriptor =
{
	&heatersLock,
	[] (const ObjectModel *self, const ObjectExplorationContext&) noexcept -> size_t { return MaxChamberHeaters; },
	[] (const ObjectModel *self, ObjectExplorationContext& context) noexcept -> ExpressionValue { return ExpressionValue((int32_t)((const Heat*)self)->chamberHeaters[context.GetLastIndex()]); }
};

constexpr ObjectModelArrayDescriptor Heat::heatersArrayDescriptor =
{
	&heatersLock,
	[] (const ObjectModel *self, const ObjectExplorationContext&) noexcept -> size_t { return ((const Heat*)self)->GetNumHeatersToReport(); },
	[] (const ObjectModel *self, ObjectExplorationContext& context) noexcept -> ExpressionValue { return ExpressionValue(((const Heat*)self)->heaters[context.GetLastIndex()]); }
};

// Macro to build a standard lambda function that includes the necessary type conversions
#define OBJECT_MODEL_FUNC(...) OBJECT_MODEL_FUNC_BODY(Heat, __VA_ARGS__)

constexpr ObjectModelTableEntry Heat::objectModelTable[] =
{
	// These entries must be in alphabetical order
	// 0. Heat class
	{ "bedHeaters",				OBJECT_MODEL_FUNC_NOSELF(&bedHeatersArrayDescriptor), 							ObjectModelEntryFlags::none },
	{ "chamberHeaters",			OBJECT_MODEL_FUNC_NOSELF(&chamberHeatersArrayDescriptor),				 		ObjectModelEntryFlags::none },
	{ "coldExtrudeTemperature",	OBJECT_MODEL_FUNC((self->coldExtrude) ? 0.0f : self->extrusionMinTemp, 1),		ObjectModelEntryFlags::none },
	{ "coldRetractTemperature", OBJECT_MODEL_FUNC((self->coldExtrude) ? 0.0f : self->retractionMinTemp, 1),		ObjectModelEntryFlags::none },
	{ "heaters",				OBJECT_MODEL_FUNC_NOSELF(&heatersArrayDescriptor),								ObjectModelEntryFlags::live },
};

constexpr uint8_t Heat::objectModelTableDescriptor[] = { 1, 5 };

DEFINE_GET_OBJECT_MODEL_TABLE(Heat)

#endif

ReadWriteLock Heat::heatersLock;
ReadWriteLock Heat::sensorsLock;

Heat::Heat() noexcept
	: sensorCount(0), sensorsRoot(nullptr), sensorOrderingErrors(0), coldExtrude(false), heaterBeingTuned(-1), lastHeaterTuned(-1)
#if SUPPORT_REMOTE_COMMANDS
	, newHeaterFaultState(0), newDriverFaultState(0)
#endif
{
	for (int8_t& h : bedHeaters)
	{
		h = -1;
	}

	for (int8_t& h : chamberHeaters)
	{
		h = -1;
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
GCodeResult Heat::SetOrReportHeaterModel(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	const unsigned int heater = gb.GetLimitedUIValue('H', MaxHeaters);
	const auto h = FindHeater(heater);
	if (h.IsNotNull())
	{
		return h->SetOrReportModel(heater, gb, reply);
	}

	reply.printf("Heater %u not found", heater);
	return GCodeResult::error;
}

// Process M301 or M304. 'heater' is the default heater number to use.
GCodeResult Heat::SetPidParameters(unsigned int heater, GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
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
			reprap.HeatUpdated();
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
			h->ClearModelAndMonitors();
		}
	}
}

void Heat::Init() noexcept
{
	extrusionMinTemp = DefaultMinExtrusionTemperature;
	retractionMinTemp = DefaultMinRetractionTemperature;
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

#if SUPPORT_REMOTE_COMMANDS

void Heat::SendHeatersStatus(CanMessageBuffer& buf) noexcept
{
	CanMessageHeatersStatus * const msg = buf.SetupStatusMessage<CanMessageHeatersStatus>(CanInterface::GetCanAddress(), CanInterface::GetCurrentMasterAddress());
	msg->whichHeaters = 0;
	unsigned int heatersFound = 0;

	{
		ReadLocker lock(heatersLock);

		for (size_t heater = 0; heater < MaxHeaters; ++heater)
		{
			Heater * const h = heaters[heater];
			if (h != nullptr)
			{
				msg->whichHeaters |= (uint64_t)1u << heater;
				msg->reports[heatersFound].mode = h->GetModeByte();
				msg->reports[heatersFound].averagePwm = (uint8_t)(h->GetAveragePWM() * 255.0);
				msg->reports[heatersFound].SetTemperature(h->GetTemperature());
				++heatersFound;
			}
		}
	}

	if (heatersFound != 0)
	{
		buf.dataLength = msg->GetActualDataLength(heatersFound);
		CanInterface::SendMessageNoReplyNoFree(&buf);
	}
}

#endif

[[noreturn]] void Heat::HeaterTask() noexcept
{
	uint32_t nextWakeTime = millis();
	for (;;)
	{
		// Wait until we are woken or it's time to send another regular broadcast. If we are really unlucky, we could end up waiting for one tick too long.
		nextWakeTime += HeatSampleIntervalMillis;
		int32_t delayTime = (int32_t)(nextWakeTime - millis());
		if (delayTime > 0)
		{
			TaskBase::Take((uint32_t)delayTime);
		}

#if SUPPORT_CAN_EXPANSION
		CanMessageBuffer buf(nullptr);
#endif

#if SUPPORT_REMOTE_COMMANDS
		if (CanInterface::InExpansionMode())
		{
			// Check whether we have any urgent messages to send
			if (newDriverFaultState == 1)
			{
				newDriverFaultState = 2;
				reprap.GetPlatform().SendDriversStatus(buf);
			}

			// Check whether we have new heater fault status messages to send
			if (newHeaterFaultState == 1)
			{
				newHeaterFaultState = 2;
				SendHeatersStatus(buf);
			}
		}
#endif

		// Check whether it is time to poll sensors and PIDs and send regular messages
		if ((int32_t)(millis() - nextWakeTime) >= 0)
		{
#if SUPPORT_REMOTE_COMMANDS
			// Announce ourselves to the main board, if it hasn't acknowledged us already
			CanInterface::SendAnnounce(&buf);
#endif

			// Walk the sensor list and poll all sensors. The list is in increasing sensor number order.
			{
#if SUPPORT_CAN_EXPANSION
				// Set up to broadcast our sensor temperatures
				CanMessageSensorTemperatures * const msg = buf.SetupBroadcastMessage<CanMessageSensorTemperatures>(CanInterface::GetCanAddress());
				msg->whichSensors = 0;
				unsigned int sensorsFound = 0;
#endif
				{
#if SUPPORT_CAN_EXPANSION
					unsigned int nextUnreportedSensor = 0;
#endif
					ReadLocker lock(sensorsLock);
					TemperatureSensor *currentSensor = sensorsRoot;
					while (currentSensor != nullptr)
					{
						currentSensor->Poll();
#if SUPPORT_CAN_EXPANSION
						if (currentSensor->GetBoardAddress() == CanInterface::GetCanAddress() && sensorsFound < ARRAY_SIZE(msg->temperatureReports))
						{
							const unsigned int sn = currentSensor->GetSensorNumber();
							if (sn >= nextUnreportedSensor && sn < 64)
							{
								msg->whichSensors |= (uint64_t)1u << currentSensor->GetSensorNumber();
								float temperature;
								msg->temperatureReports[sensorsFound].errorCode = (uint8_t)currentSensor->GetLatestTemperature(temperature);
								msg->temperatureReports[sensorsFound].SetTemperature(temperature);
								++sensorsFound;
								nextUnreportedSensor = sn + 1;
							}
							else
							{
								// We have a duplicate sensor number, or the sensors list is not ordered by sensor number, or the sensor number is out of range
								// Don't send its temperature because that will mess up the relationship between the bitmap and the sensor data in the message
								++sensorOrderingErrors;
							}
						}
#endif
						currentSensor = currentSensor->GetNext();
					}
				}
#if SUPPORT_CAN_EXPANSION
				if (sensorsFound != 0)							// don't send an empty report
				{
					buf.dataLength = msg->GetActualDataLength(sensorsFound);
					CanInterface::SendBroadcastNoFree(&buf);
				}
#endif
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

			// See if we have finished tuning a heater
			if (heaterBeingTuned != -1)
			{
				const auto h = FindHeater(heaterBeingTuned);
				if (h.IsNull() || h->GetStatus() != HeaterStatus::tuning)
				{
					lastHeaterTuned = heaterBeingTuned;
					heaterBeingTuned = -1;
				}
#if SUPPORT_REMOTE_COMMANDS
				else if (CanInterface::InExpansionMode())
				{
					auto msg = buf.SetupStatusMessage<CanMessageHeaterTuningReport>(CanInterface::GetCanAddress(), CanInterface::GetCurrentMasterAddress());
					if (LocalHeater::GetTuningCycleData(*msg))
					{
						msg->SetStandardFields(heaterBeingTuned);
						CanInterface::SendMessageNoReplyNoFree(&buf);
					}
				}
#endif
			}

#if SUPPORT_REMOTE_COMMANDS
			if (CanInterface::InExpansionMode())
			{
				if (newHeaterFaultState == 0)
				{
					SendHeatersStatus(buf);						// send the status of our heaters
				}
				else
				{
					newHeaterFaultState = 0;					// we recently sent it, so send it again next time
				}

				// Send our fan RPMs
				{
					CanMessageFansReport * const msg = buf.SetupStatusMessage<CanMessageFansReport>(CanInterface::GetCanAddress(), CanInterface::GetCurrentMasterAddress());
					const unsigned int numReported = reprap.GetFansManager().PopulateFansReport(*msg);
					if (numReported != 0)
					{
						buf.dataLength = msg->GetActualDataLength(numReported);
						CanInterface::SendMessageNoReplyNoFree(&buf);
					}
				}

				if (newDriverFaultState == 0)
				{
					reprap.GetPlatform().SendDriversStatus(buf);			// send the status of our drivers
				}
				else
				{
					newDriverFaultState = 0;					// we recently sent it, so send it again next time
				}

				// Send a board health message
				{
					CanMessageBoardStatus * const boardStatusMsg = buf.SetupStatusMessage<CanMessageBoardStatus>(CanInterface::GetCanAddress(), CanInterface::GetCurrentMasterAddress());
					boardStatusMsg->Clear();

					// We must add fields in the following order: VIN, V12, MCU temperature
					size_t index = 0;
#if HAS_VOLTAGE_MONITOR
					boardStatusMsg->values[index++] = reprap.GetPlatform().GetPowerVoltages();
					boardStatusMsg->hasVin = true;
#endif
#if HAS_12V_MONITOR
					boardStatusMsg->values[index++] = reprap.GetPlatform().GetV12Voltages();
					boardStatusMsg->hasV12 = true;
#endif
#if HAS_CPU_TEMP_SENSOR
					boardStatusMsg->values[index++] = reprap.GetPlatform().GetMcuTemperatures();
					boardStatusMsg->hasMcuTemp = true;
#endif
					buf.dataLength = boardStatusMsg->GetActualDataLength();
					CanInterface::SendMessageNoReplyNoFree(&buf);
				}
			}
#endif

			reprap.KickHeatTaskWatchdog();
		}
	}
}

void Heat::Diagnostics(MessageType mtype) noexcept
{
	Platform& platform = reprap.GetPlatform();
	platform.Message(mtype, "=== Heat ===\n");
	String<StringLength100> str;
	str.copy("Bed heaters");
	for (int8_t bedHeater : bedHeaters)
	{
		str.catf(" %d", bedHeater);
	}
	str.cat(", chamber heaters");
	for (int8_t chamberHeater : chamberHeaters)
	{
		str.catf(" %d", chamberHeater);
	}
	str.catf(", ordering errs %u\n", sensorOrderingErrors);
	platform.Message(mtype, str.c_str());

	for (size_t heater : ARRAY_INDICES(heaters))
	{
		auto h = FindHeater(heater);
		if (h.IsNotNull() && h->GetStatus() == HeaterStatus::active)
		{
			const float acc = h->GetAccumulator();
			h.Release();
			platform.MessageF(mtype, "Heater %u is on, I-accum = %.1f\n", heater, (double)acc);
		}
	}
}

// Configure a heater. Invoked by M950.
GCodeResult Heat::ConfigureHeater(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	const size_t heater = gb.GetLimitedUIValue('H', MaxHeaters);

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
			DeleteObject(heaters[heater]);
			reprap.HeatUpdated();
			return GCodeResult::ok;
		}

		gb.MustSee('T');
		const unsigned int sensorNumber = gb.GetUIValue();

		WriteLocker lock(heatersLock);
		DeleteObject(heaters[heater]);

		const PwmFrequency freq = (gb.Seen('Q')) ? min<PwmFrequency>(gb.GetPwmFrequency(), MaxHeaterPwmFrequency) : DefaultHeaterPwmFreq;

#if SUPPORT_CAN_EXPANSION
		Heater * const newHeater = (board != CanInterface::GetCanAddress()) ? (Heater *)new RemoteHeater(heater, board) : new LocalHeater(heater);
#else
		Heater * const newHeater = new LocalHeater(heater);
#endif
		const GCodeResult rslt = newHeater->ConfigurePortAndSensor(pinName.c_str(), freq, sensorNumber, reply);
		if (Succeeded(rslt))
		{
			heaters[heater] = newHeater;
		}
		else
		{
			delete newHeater;
		}
		reprap.HeatUpdated();
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
		const GCodeResult rslt = h->SetPwmFrequency(gb.GetPwmFrequency(), reply);
		reprap.HeatUpdated();
		return rslt;
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
			const bool cooling = h->IsCoolingDevice();
			return (!cooling && target < TemperatureSoLowDontCare)
				|| (cooling && target > TemperatureSoHighDontCare)
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
	// First, turn off any existing bed heater in this slot
	{
		const auto h = FindHeater(bedHeaters[index]);
		if (h.IsNotNull())
		{
			h->SwitchOff();
		}
	}
	bedHeaters[index] = heater;
	{
		const auto h = FindHeater(heater);
		if (h.IsNotNull())
		{
			h->SetAsBedOrChamberHeater();
		}
	}
	reprap.HeatUpdated();
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
	// First, turn off any existing chamber heater in this slot
	{
		const auto h = FindHeater(chamberHeaters[index]);
		if (h.IsNotNull())
		{
			h->SwitchOff();
		}
	}
	chamberHeaters[index] = heater;
	const auto h = FindHeater(heater);
	if (h.IsNotNull())
	{
		h->SetAsBedOrChamberHeater();
	}
	reprap.HeatUpdated();
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

// This is called when a tool is created that uses this heater
void Heat::SetAsToolHeater(int8_t heater) const noexcept
{
	const auto h = FindHeater(heater);
	if (h.IsNotNull())
	{
		h->SetAsToolHeater();
	}
	reprap.HeatUpdated();

}

void Heat::SetTemperature(int heater, float t, bool activeNotStandby) THROWS(GCodeException)
{
	const auto h = FindHeater(heater);
	if (h.IsNotNull())
	{
		h->SetTemperature(t, activeNotStandby);
	}
}

float Heat::GetActiveTemperature(int heater) const noexcept
{
	const auto h = FindHeater(heater);
	return (h.IsNull()) ? ABS_ZERO : h->GetActiveTemperature();
}

float Heat::GetStandbyTemperature(int heater) const noexcept
{
	const auto h = FindHeater(heater);
	return (h.IsNull()) ? ABS_ZERO : h->GetStandbyTemperature();
}

float Heat::GetHighestTemperatureLimit(int heater) const noexcept
{
	const auto h = FindHeater(heater);
	return (h.IsNull()) ? BadErrorTemperature : h->GetHighestTemperatureLimit();
}

float Heat::GetLowestTemperatureLimit(int heater) const noexcept
{
	const auto h = FindHeater(heater);
	return (h.IsNull()) ? ABS_ZERO : h->GetLowestTemperatureLimit();
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

GCodeResult Heat::SetActiveOrStandby(int heater, const Tool *tool, bool active, const StringRef& reply) noexcept
{
	const auto h = FindHeater(heater);
	if (h.IsNotNull())
	{
		const GCodeResult rslt = h->SetActiveOrStandby(active, reply);
		if (rslt == GCodeResult::ok && !active)
		{
			lastStandbyTools[heater] = tool;
		}
		return rslt;
	}
	reply.printf("Heater %d not found", heater);
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

// Turn off all local heaters. Safe to call from an ISR. Called only from the tick ISR.
void Heat::SwitchOffAllLocalFromISR() noexcept
{
	for (Heater* h : heaters)
	{
		if (h != nullptr
#if SUPPORT_CAN_EXPANSION
			&& h->IsLocal()
#endif
		   )
		{
			h->SwitchOff();
		}
	}
}

void Heat::FeedForwardAdjustment(unsigned int heater, float fanPwmChange, float extrusionChange) const noexcept
{
	const auto h = FindHeater(heater);
	if (h.IsNotNull())
	{
		h->FeedForwardAdjustment(fanPwmChange, extrusionChange);
	}
}

// This one is called from an ISR so we must not get a lock
void Heat::SetExtrusionFeedForward(unsigned int heater, float pwm) const noexcept
{
	if (heater < MaxHeaters)
	{
		Heater * const h = heaters[heater];
		if (h != nullptr)
		{
			h->SetExtrusionFeedForward(pwm);
		}
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
	ReadLocker lock(heatersLock);
	for (const Heater *h : heaters)
	{
		if (h != nullptr)
		{
			const float tlimit = h->GetHighestTemperatureLimit();
			if (tlimit > limit)
			{
				limit = tlimit;
			}
		}
	}
	return limit;
}

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE

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
				String<StringLength256> scratchString;
				model.AppendM307Command(h, scratchString.GetRef(), !IsBedOrChamberHeater(h));
				model.AppendM301Command(h, scratchString.GetRef());
				ok = f->Write(scratchString.c_str());
			}
		}
	}
	return ok;
}

#endif

// Process M570
GCodeResult Heat::ConfigureHeaterMonitoring(size_t heater, GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
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
GCodeResult Heat::TuneHeater(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	// To tune a heater, a heater number and/or a tool number musty be given
	FansBitmap fans;
	int heaterNumber;
	const bool seenHeater = gb.Seen('H');
	if (seenHeater)
	{
		heaterNumber = gb.GetIValue();
	}
	const bool seenTool = gb.Seen('T');
	if (seenTool)
	{
		const int toolNumber = gb.GetIValue();
		const auto tool = reprap.GetTool(toolNumber);
		if (tool.IsNull())
		{
			reply.printf("tool %d not found", toolNumber);
			return GCodeResult::error;
		}
		if (seenHeater)
		{
			if (!tool->UsesHeater(heaterNumber))
			{
				reply.printf("tool %d does not use heater %d", toolNumber, heaterNumber);
				return GCodeResult::error;
			}
		}
		else if (tool->HeaterCount() == 0)
		{
			reply.printf("tool %d has no heaters", toolNumber);
			return GCodeResult::error;
		}
		else
		{
			heaterNumber = tool->GetHeater(0);
		}
		fans = tool->GetFanMapping();
	}

	if (seenHeater || seenTool)
	{
		if (heaterBeingTuned != -1)
		{
			// Trying to start a new auto tune, but we are already tuning a heater
			reply.printf("Error: cannot start a new auto tune because heater %d is being tuned", heaterBeingTuned);
			return GCodeResult::error;
		}

		const auto h = FindHeater(heaterNumber);
		if (h.IsNull())
		{
			reply.printf("Heater %u not found", heaterNumber);
			return GCodeResult::error;
		}

		const GCodeResult rslt = h->StartAutoTune(gb, reply, fans);
		if (Succeeded(rslt))
		{
			heaterBeingTuned = (int8_t)heaterNumber;
		}
		return rslt;
	}

	// If we get here then neither T nor H was given, so report the auto tune status
	const int whichPid = (heaterBeingTuned == -1) ? lastHeaterTuned : heaterBeingTuned;
	const auto h = FindHeater(whichPid);

	if (h.IsNotNull())
	{
		h->GetAutoTuneStatus(reply);
	}
	else
	{
		reply.copy("No heater has been tuned since startup");
	}
	return GCodeResult::ok;
}

// Process M308
GCodeResult Heat::ConfigureSensor(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
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
	CanAddress boardAddress = CanId::NoAddress;
#endif

	if (gb.Seen('P'))
	{
		String<StringLength50> portName;
		gb.GetReducedString(portName.GetRef());
#if SUPPORT_CAN_EXPANSION
		boardAddress = IoPort::RemoveBoardAddress(portName.GetRef());
#else
		if (!IoPort::RemoveBoardAddress(portName.GetRef()))
		{
			reply.lcat("Board address of port must be 0");
			return GCodeResult::error;
		}
#endif
		if (portName.EqualsIgnoreCase(NoPinName))					// if deleting this sensor
		{
			WriteLocker lock(sensorsLock);
			DeleteSensor(sensorNum);
			return GCodeResult::ok;
		}
	}

	if (gb.Seen('Y'))
	{
		// Creating a new sensor
		WriteLocker lock(sensorsLock);

		DeleteSensor(sensorNum);

		String<StringLength50> typeName;							// StringLength20 is too short for "thermocouple-max31856"
		gb.GetReducedString(typeName.GetRef());

#if SUPPORT_CAN_EXPANSION
		if (boardAddress == CanId::NoAddress)
		{
			boardAddress = CanInterface::GetCanAddress();			// no port name was given, so default to local
		}
		TemperatureSensor * const newSensor = TemperatureSensor::Create(sensorNum, boardAddress, typeName.c_str(), reply);
#else
		TemperatureSensor * const newSensor = TemperatureSensor::Create(sensorNum, typeName.c_str(), reply);
#endif
		if (newSensor == nullptr)
		{
			return GCodeResult::error;
		}

		bool changed = false;
		try
		{
			const GCodeResult rslt = newSensor->Configure(gb, reply, changed);
			if (Succeeded(rslt))
			{
				InsertSensor(newSensor);
			}
			else
			{
				delete newSensor;
			}
			return rslt;
		}
		catch (const GCodeException&)
		{
			delete newSensor;
			throw;
		}
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

	bool changed = false;
	const GCodeResult rslt = sensor->Configure(gb, reply, changed);
	if (changed)
	{
		reprap.SensorsUpdated();
	}
	return rslt;
}

// Get the name of a heater, or nullptr if it hasn't been named
const char *Heat::GetHeaterSensorName(size_t heater) const noexcept
{
	const auto h = FindHeater(heater);
	return (h.IsNotNull()) ? h->GetSensorName() : nullptr;
}

// Configure heater protection (M143). Returns true if an error occurred
GCodeResult Heat::HandleM143(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	const size_t heaterNumber = (gb.Seen('H')) ? gb.GetLimitedUIValue('H', MaxHeaters) : 1;
	const auto h = FindHeater(heaterNumber);
	if (h.IsNull())
	{
		reply.printf("Heater %u does not exist", heaterNumber);
		return GCodeResult::error;
	}

	return h->ConfigureMonitor(gb, reply);
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
			reprap.SensorsUpdated();
			break;
		}

		lastSensor = currentSensor;
		currentSensor = currentSensor->GetNext();
	}
}

// Insert a sensor. Must write-lock the sensors lock before calling this. The sensors list is kept in order of increasing sensor number.
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
			reprap.SensorsUpdated();
			break;
		}
		prev = ts;
		ts = ts->GetNext();
	}
}

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE

// Save some resume information returning true if successful.
// We assume that the bed and chamber heaters are either on and active, or off (not on standby).
bool Heat::WriteBedAndChamberTempSettings(FileStore *f) const noexcept
{
	String<StringLength256> bufSpace;
	const StringRef buf = bufSpace.GetRef();
	for (size_t index : ARRAY_INDICES(bedHeaters))
	{
		const auto h = FindHeater(bedHeaters[index]);
		if (h.IsNotNull() && h->GetStatus() == HeaterStatus::active)
		{
			buf.catf("M140 P%u S%.1f\n", index, (double)h->GetActiveTemperature());
		}
	}
	for (size_t index : ARRAY_INDICES(chamberHeaters))
	{
		const auto h = FindHeater(chamberHeaters[index]);
		if (h.IsNotNull() && h->GetStatus() == HeaterStatus::active)
		{
			buf.catf("M141 P%u S%.1f\n", index, (double)h->GetActiveTemperature());
		}
	}
	return (buf.strlen() == 0) || f->Write(buf.c_str());
}

#endif

#if SUPPORT_CAN_EXPANSION

void Heat::ProcessRemoteSensorsReport(CanAddress src, const CanMessageSensorTemperatures& msg) noexcept
{
	Bitmap<uint64_t> sensorsReported(msg.whichSensors);
	sensorsReported.Iterate([this, src, &msg](unsigned int sensor, unsigned int index)
								{
									if (index < ARRAY_SIZE(msg.temperatureReports))
									{
										const CanSensorReport& sr = msg.temperatureReports[index];
										auto ts = FindSensor(sensor);
										if (ts.IsNotNull())
										{
											ts->UpdateRemoteTemperature(src, sr);
										}
# ifdef DUET3_ATE
										else
										{
											Duet3Ate::ProcessOrphanedSensorReport(src, sensor, sr);
										}
# elif SUPPORT_REMOTE_COMMANDS
										else if (CanInterface::InExpansionMode())
										{
											// Create a new RemoteSensor
											ts.Release();
											RemoteSensor * const rs = new RemoteSensor(sensor, src);
											rs->UpdateRemoteTemperature(src, sr);
											InsertSensor(rs);
										}
# endif
									}
								}
							);
}

void Heat::ProcessRemoteHeatersReport(CanAddress src, const CanMessageHeatersStatus& msg) noexcept
{
	Bitmap<uint64_t> heatersReported(msg.whichHeaters);
	heatersReported.Iterate([this, src, &msg](unsigned int heaterNum, unsigned int index)
								{
									if (index < ARRAY_SIZE(msg.reports))
									{
										const auto h = FindHeater(heaterNum);
										if (h.IsNotNull())
										{
											h->UpdateRemoteStatus(src, msg.reports[index]);
										}
									}
								}
							);
}

void Heat::ProcessRemoteHeaterTuningReport(CanAddress src, const CanMessageHeaterTuningReport& msg) noexcept
{
	const auto h = FindHeater(msg.heater);
	if (h.IsNotNull())
	{
		h->UpdateHeaterTuning(src, msg);
	}
}

#endif

#if SUPPORT_REMOTE_COMMANDS

static GCodeResult UnknownHeater(unsigned int heater, const StringRef& reply) noexcept
{
	reply.printf("Board %u does not have heater %u", CanInterface::GetCanAddress(), heater);
	return GCodeResult::error;
}

GCodeResult Heat::ConfigureHeater(const CanMessageGeneric& msg, const StringRef& reply) noexcept
{
	CanMessageGenericParser parser(msg, M950HeaterParams);
	uint16_t heater;
	if (!parser.GetUintParam('H', heater))
	{
		return GCodeResult::remoteInternalError;
	}

	if (heater >= MaxHeaters)
	{
		reply.copy("Heater number out of range");
		return GCodeResult::error;
	}

	PwmFrequency freq = DefaultFanPwmFreq;
	const bool seenFreq = parser.GetUintParam('Q', freq);

	String<StringLength50> pinName;
	if (parser.GetStringParam('C', pinName.GetRef()))
	{
		uint16_t sensorNumber;
		if (!parser.GetUintParam('T', sensorNumber))
		{
			reply.copy("Missing sensor number");
			return GCodeResult::error;
		}

		WriteLocker lock(heatersLock);

		Heater *oldHeater = nullptr;
		std::swap(oldHeater, heaters[heater]);
		delete oldHeater;

		Heater *newHeater = new LocalHeater(heater);
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

	const auto h = FindHeater(heater);
	if (h.IsNull())
	{
		return UnknownHeater(heater, reply);
	}

	if (seenFreq)
	{
		return h->SetPwmFrequency(freq, reply);
	}

	return h->ReportDetails(reply);
}

GCodeResult Heat::ProcessM307New(const CanMessageHeaterModelNewNew& msg, const StringRef& reply) noexcept
{
	const auto h = FindHeater(msg.heater);
	return (h.IsNotNull()) ? h->SetModel(msg.heater, msg, reply) : UnknownHeater(msg.heater, reply);
}

GCodeResult Heat::SetTemperature(const CanMessageSetHeaterTemperature& msg, const StringRef& reply) noexcept
{
	const auto h = FindHeater(msg.heaterNumber);
	return (h.IsNotNull()) ? h->SetTemperature(msg, reply) : UnknownHeater(msg.heaterNumber, reply);
}

GCodeResult Heat::SetFaultDetection(const CanMessageSetHeaterFaultDetectionParameters& msg, const StringRef& reply) noexcept
{
	const auto h = FindHeater(msg.heater);
	return (h.IsNotNull())
			? h->SetFaultDetectionParameters(msg.maxTempExcursion, msg.maxFaultTime, reply)
				: UnknownHeater(msg.heater, reply);
}

GCodeResult Heat::SetHeaterMonitors(const CanMessageSetHeaterMonitors& msg, const StringRef& reply) noexcept
{
	const auto h = FindHeater(msg.heater);
	return (h.IsNotNull()) ? h->SetHeaterMonitors(msg, reply) : UnknownHeater(msg.heater, reply);
}

GCodeResult Heat::TuningCommand(const CanMessageHeaterTuningCommand& msg, const StringRef& reply) noexcept
{
	if (heaterBeingTuned != -1 && heaterBeingTuned != (int)msg.heaterNumber)
	{
		reply.printf("Heater %d is already being tuned", heaterBeingTuned);
		return GCodeResult::error;
	}
	const auto h = FindHeater(msg.heaterNumber);
	if (h.IsNull())
	{
		return UnknownHeater(msg.heaterNumber, reply);
	}
	heaterBeingTuned = (int)msg.heaterNumber;			// setting this is OK even if we are stopping or fail to start tuning, because we check it in the heater task loop
	return h->TuningCommand(msg, reply);
}

GCodeResult Heat::FeedForward(const CanMessageHeaterFeedForward& msg, const StringRef& reply) noexcept
{
	const auto h = FindHeater(msg.heaterNumber);
	if (h.IsNull())
	{
		return UnknownHeater(msg.heaterNumber, reply);
	}
	h->FeedForwardAdjustment(msg.fanPwmAdjustment, msg.extrusionAdjustment);
	return GCodeResult::ok;
}

GCodeResult Heat::ProcessM308(const CanMessageGeneric& msg, const StringRef& reply) noexcept
{
	CanMessageGenericParser parser(msg, M308NewParams);
	uint16_t sensorNum;
	if (parser.GetUintParam('S', sensorNum))
	{
		if (sensorNum < MaxSensors)
		{
			// Check for deleting the sensor by assigning a null port. Borrow the sensor type name string temporarily for this.
			String<StringLength20> sensorTypeName;
			if (parser.GetStringParam('P', sensorTypeName.GetRef()) && sensorTypeName.EqualsIgnoreCase(NoPinName))
			{
				WriteLocker lock(sensorsLock);
				DeleteSensor(sensorNum);
				return GCodeResult::ok;
			}

			if (parser.GetStringParam('Y', sensorTypeName.GetRef()))
			{
				WriteLocker lock(sensorsLock);

				DeleteSensor(sensorNum);

				TemperatureSensor * const newSensor = TemperatureSensor::Create(sensorNum, CanInterface::GetCanAddress(), sensorTypeName.c_str(), reply);
				if (newSensor == nullptr)
				{
					return GCodeResult::error;
				}

				const GCodeResult rslt = newSensor->Configure(parser, reply);
				if (rslt == GCodeResult::ok || rslt == GCodeResult::warning)
				{
					InsertSensor(newSensor);
				}
				else
				{
					delete newSensor;
				}
				return rslt;
			}

			const auto sensor = FindSensor(sensorNum);
			if (sensor.IsNull())
			{
				reply.printf("Sensor %u does not exist", sensorNum);
				return GCodeResult::error;
			}
			return sensor->Configure(parser, reply);
		}
		else
		{
			reply.copy("Sensor number out of range");
			return GCodeResult::error;
		}
	}

	reply.copy("Missing sensor number parameter");
	return GCodeResult::error;
}

#endif

// End
