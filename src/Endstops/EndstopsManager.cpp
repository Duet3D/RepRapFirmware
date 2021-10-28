/*
 * Endstop.cpp
 *
 *  Created on: 3 Apr 2019
 *      Author: David
 */

#include "EndstopsManager.h"

#include "Endstop.h"
#include "SwitchEndstop.h"
#include "StallDetectionEndstop.h"
#include "ZProbeEndstop.h"

#include "ZProbe.h"
#include "LocalZProbe.h"
#include "RemoteZProbe.h"

#include <Platform/RepRap.h>
#include <GCodes/GCodeBuffer/GCodeBuffer.h>
#include <GCodes/GCodes.h>
#include <Movement/Move.h>
#include <Platform/OutputMemory.h>
#include <Heating/Heat.h>
#include <Heating/Sensors/TemperatureSensor.h>

#if SUPPORT_CAN_EXPANSION
# include <CanMessageBuffer.h>
#endif

ReadWriteLock EndstopsManager::endstopsLock;
ReadWriteLock EndstopsManager::zProbesLock;

#if SUPPORT_OBJECT_MODEL

// Object model table and functions
// Note: if using GCC version 7.3.1 20180622 and lambda functions are used in this table, you must compile this file with option -std=gnu++17.
// Otherwise the table will be allocated in RAM instead of flash, which wastes too much RAM.

// Macro to build a standard lambda function that includes the necessary type conversions
#define OBJECT_MODEL_FUNC(...) OBJECT_MODEL_FUNC_BODY(EndstopsManager, __VA_ARGS__)

constexpr ObjectModelArrayDescriptor EndstopsManager::sensorsArrayDescriptor =
{
	&Heat::sensorsLock,
	[] (const ObjectModel *self, const ObjectExplorationContext&) noexcept -> size_t { return reprap.GetHeat().GetNumSensorsToReport(); },
	[] (const ObjectModel *self, ObjectExplorationContext& context) noexcept -> ExpressionValue { return ExpressionValue(reprap.GetHeat().FindSensor(context.GetLastIndex()).Ptr()); }
};

constexpr ObjectModelArrayDescriptor EndstopsManager::endstopsArrayDescriptor =
{
	&endstopsLock,
	[] (const ObjectModel *self, const ObjectExplorationContext&) noexcept -> size_t { return reprap.GetGCodes().GetTotalAxes(); },
	[] (const ObjectModel *self, ObjectExplorationContext& context) noexcept -> ExpressionValue
					{ return ExpressionValue(((const EndstopsManager*)self)->FindEndstop(context.GetLastIndex()).Ptr()); }
};

constexpr ObjectModelArrayDescriptor EndstopsManager::filamentMonitorsArrayDescriptor =
{
	&FilamentMonitor::filamentMonitorsLock,
	[] (const ObjectModel *self, const ObjectExplorationContext&) noexcept -> size_t { return FilamentMonitor::GetNumMonitorsToReport(); },
	[] (const ObjectModel *self, ObjectExplorationContext& context) noexcept -> ExpressionValue { return ExpressionValue(FilamentMonitor::GetMonitorAlreadyLocked(context.GetLastIndex())); }
};

constexpr ObjectModelArrayDescriptor EndstopsManager::gpinArrayDescriptor =
{
	nullptr,
	[] (const ObjectModel *self, const ObjectExplorationContext&) noexcept -> size_t { return reprap.GetPlatform().GetNumGpInputsToReport(); },
	[] (const ObjectModel *self, ObjectExplorationContext& context) noexcept -> ExpressionValue
					{
						const GpInputPort& port = reprap.GetPlatform().GetGpInPort(context.GetLastIndex());
						return (port.IsUnused()) ? ExpressionValue(nullptr) : ExpressionValue(&port);
					}
};

constexpr ObjectModelArrayDescriptor EndstopsManager::probesArrayDescriptor =
{
	&zProbesLock,
	[] (const ObjectModel *self, const ObjectExplorationContext&) noexcept -> size_t { return ((const EndstopsManager*)self)->GetNumProbesToReport(); },
	[] (const ObjectModel *self, ObjectExplorationContext& context) noexcept -> ExpressionValue
					{ return ExpressionValue(((const EndstopsManager*)self)->GetZProbe(context.GetLastIndex()).Ptr()); }
};

constexpr ObjectModelTableEntry EndstopsManager::objectModelTable[] =
{
	// Within each group, these entries must be in alphabetical order
	// 0. sensors members
	{ "analog",				OBJECT_MODEL_FUNC_NOSELF(&sensorsArrayDescriptor),				ObjectModelEntryFlags::live },
	{ "endstops",			OBJECT_MODEL_FUNC_NOSELF(&endstopsArrayDescriptor), 			ObjectModelEntryFlags::live },
	{ "filamentMonitors",	OBJECT_MODEL_FUNC_NOSELF(&filamentMonitorsArrayDescriptor),		ObjectModelEntryFlags::live },
	{ "gpIn",				OBJECT_MODEL_FUNC_NOSELF(&gpinArrayDescriptor), 				ObjectModelEntryFlags::live },
	{ "probes",				OBJECT_MODEL_FUNC_NOSELF(&probesArrayDescriptor),				ObjectModelEntryFlags::live },
};

constexpr uint8_t EndstopsManager::objectModelTableDescriptor[] = { 1, 5 };

DEFINE_GET_OBJECT_MODEL_TABLE(EndstopsManager)

#endif

EndstopsManager::EndstopsManager() noexcept
		: activeEndstops(nullptr),
#if HAS_STALL_DETECT
		  extrudersEndstop(nullptr),
#endif
		  isHomingMove(false)
{
	for (Endstop *& es : axisEndstops)
	{
		es = nullptr;
	}
	for (ZProbe *& zp : zProbes)
	{
		zp = nullptr;
	}
}

void EndstopsManager::Init() noexcept
{
	activeEndstops = nullptr;

#if ALLOCATE_DEFAULT_PORTS
	// Configure default endstops
	String<1> dummy;
	for (size_t axis = 0; axis < ARRAY_SIZE(DefaultEndstopPinNames); ++axis)
	{
		SwitchEndstop * const sw = new SwitchEndstop(axis, EndStopPosition::lowEndStop);
		sw->Configure(DefaultEndstopPinNames[axis], dummy.GetRef());
		axisEndstops[axis] = sw;
	}
#endif

	// Z probes
	reprap.GetPlatform().InitZProbeFilters();

#if ALLOCATE_DEFAULT_PORTS
	LocalZProbe * const zp = new LocalZProbe(0);
	zp->AssignPorts(DefaultZProbePinNames, dummy.GetRef());
	zProbes[0] = zp;
#endif

	defaultZProbe = new DummyZProbe(0);			// we must always have a non-null current Z probe so we use this one if none is defined
}

ReadLockedPointer<Endstop> EndstopsManager::FindEndstop(size_t axis) const noexcept
{
	ReadLocker lock(endstopsLock);
	return ReadLockedPointer<Endstop>(lock, (axis < MaxAxes) ? axisEndstops[axis] : nullptr);
}

ReadLockedPointer<ZProbe> EndstopsManager::GetZProbe(size_t index) const noexcept
{
	ReadLocker lock(zProbesLock);
	return ReadLockedPointer<ZProbe>(lock, (index < ARRAY_SIZE(zProbes)) ? zProbes[index] : nullptr);
}

// Return the current Z probe if there is one, else a default Z probe
ReadLockedPointer<ZProbe> EndstopsManager::GetZProbeOrDefault(size_t index) const noexcept
{
	ReadLocker lock(zProbesLock);
	return ReadLockedPointer<ZProbe>(lock,
										(index < ARRAY_SIZE(zProbes) && zProbes[index] != nullptr)
										? zProbes[index]
										: defaultZProbe);
}

ZProbe& EndstopsManager::GetDefaultZProbeFromISR() const noexcept
{
	return (zProbes[0] != nullptr)
			? *zProbes[0]
			: *defaultZProbe;
}

// Add an endstop to the active list
void EndstopsManager::AddToActive(EndstopOrZProbe& e) noexcept
{
	e.SetNext(activeEndstops);
	activeEndstops = &e;
}

// Set up the active endstop list according to the axes commanded to move in a G0/G1 S1/S3 command. Return true if successful.
bool EndstopsManager::EnableAxisEndstops(AxesBitmap axes, bool forHoming, bool& reduceAcceleration) noexcept
{
	activeEndstops = nullptr;
	reduceAcceleration = false;
	isHomingMove = forHoming && axes.IsNonEmpty();
	const Kinematics& kin = reprap.GetMove().GetKinematics();
	while (axes.IsNonEmpty())
	{
		const unsigned int axis = axes.LowestSetBit();
		axes.ClearBit(axis);
		Endstop * const es = axisEndstops[axis];
		if (es != nullptr && es->Prime(kin, reprap.GetPlatform().GetAxisDriversConfig(axis)))
		{
			AddToActive(*es);
			if (es->ShouldReduceAcceleration())
			{
				reduceAcceleration = true;
			}
		}
		else
		{
			activeEndstops = nullptr;
			return false;
		}
	}
	return true;
}

// Set up the active endstops for Z probing, returning true if successful
bool EndstopsManager::EnableZProbe(size_t probeNumber, bool probingAway) noexcept
{
	activeEndstops = nullptr;
	isHomingMove = false;
	if (probeNumber < MaxZProbes && zProbes[probeNumber] != nullptr)
	{
		zProbes[probeNumber]->SetProbingAway(probingAway);
		AddToActive(*zProbes[probeNumber]);
	}
	return true;
}

// Enable extruder endstops. This adds to any existing axis endstops, so you must call EnableAxisEndstops before calling this.
bool EndstopsManager::EnableExtruderEndstops(ExtrudersBitmap extruders) noexcept
{
	if (extruders.IsNonEmpty())
	{
#if HAS_STALL_DETECT
		if (extrudersEndstop == nullptr)
		{
			extrudersEndstop = new StallDetectionEndstop;
		}
		DriversBitmap drivers;
		while (extruders.IsNonEmpty())
		{
			const unsigned int extruder = extruders.LowestSetBit();
			extruders.ClearBit(extruder);
			const DriverId driver = reprap.GetPlatform().GetExtruderDriver(extruder);
# if SUPPORT_CAN_EXPANSION
			if (driver.IsLocal())
			{
				drivers.SetBit(driver.localDriver);
			}
			else
			{
				//TODO remote stall detect endstop
				return false;
			}
# else
			drivers.SetBit(driver.localDriver);
# endif
		}

		extrudersEndstop->SetDrivers(drivers);
		AddToActive(*extrudersEndstop);
#else
		return false;
#endif
	}
	return true;
}

// Check the endstops.
// If an endstop has triggered, remove it from the active list and return its details
EndstopHitDetails EndstopsManager::CheckEndstops() noexcept
{
	EndstopHitDetails ret;									// the default constructor will clear all fields
	EndstopOrZProbe *actioned = nullptr;
	for (EndstopOrZProbe *esp = activeEndstops; esp != nullptr; esp = esp->GetNext())
	{
		EndstopHitDetails hd = esp->CheckTriggered();
		if (hd.GetAction() == EndstopHitAction::stopAll)
		{
			activeEndstops = nullptr;						// no need to do anything else
			if (!isHomingMove)
			{
				hd.setAxisHigh = false;
				hd.setAxisLow = false;
			}
			return hd;
		}
		if (hd.GetAction() > ret.GetAction())
		{
			ret = hd;
			actioned = esp;
		}
	}

	if (ret.GetAction() != EndstopHitAction::none)
	{
		if (actioned->Acknowledge(ret))
		{
			// The actioned endstop has completed so remove it from the active list
			EndstopOrZProbe *prev = nullptr;
			for (EndstopOrZProbe *es = activeEndstops; es != nullptr; )
			{
				if (es == actioned)
				{
					if (prev == nullptr)
					{
						activeEndstops = es->GetNext();
					}
					else
					{
						prev->SetNext(es->GetNext());
					}
					break;
				}
				prev = es;
				es = es->GetNext();
			}
		}
		if (!isHomingMove)
		{
			ret.setAxisHigh = false;
			ret.setAxisLow = false;
		}
	}
	return ret;
}

// Configure the endstops in response to M574
GCodeResult EndstopsManager::HandleM574(GCodeBuffer& gb, const StringRef& reply, OutputBuffer*& outbuf) THROWS(GCodeException)
{
	// First count how many axes we are configuring, and lock movement if necessary
	unsigned int axesSeen = 0;
	size_t lastAxisSeen = 0;											// dummy initialisation to avoid compiler warnings
	EndStopPosition lastPosSeen = EndStopPosition::noEndStop;			// dummy initialisation to avoid compiler warnings
	for (size_t axis = 0; axis < reprap.GetGCodes().GetTotalAxes(); ++axis)
	{
		if (gb.Seen(reprap.GetGCodes().GetAxisLetters()[axis]))
		{
			lastPosSeen = (EndStopPosition)gb.GetUIValue();
			if (lastPosSeen >= EndStopPosition::numPositions)
			{
				reply.copy("Invalid endstop position");
				return GCodeResult::error;
			}
			lastAxisSeen = axis;
			++axesSeen;
		}
	}

	if (axesSeen == 0)
	{
		// Report current configuration
		// The response can get very long, so allocate an output buffer
		if (outbuf == nullptr && !OutputBuffer::Allocate(outbuf))
		{
			return GCodeResult::notFinished;
		}

		outbuf->copy("Endstop configuration:");
		ReadLocker lock(endstopsLock);

		for (size_t axis = 0; axis < reprap.GetGCodes().GetTotalAxes(); ++axis)
		{
			outbuf->catf("\n%c: ", reprap.GetGCodes().GetAxisLetters()[axis]);
			if (axisEndstops[axis] == nullptr)
			{
				outbuf->cat("none");
			}
			else
			{
				outbuf->cat((axisEndstops[axis]->GetAtHighEnd()) ? "high end " : "low end ");
				reply.Clear();
				axisEndstops[axis]->AppendDetails(reply);
				outbuf->cat(reply.c_str());
			}
		}
		return GCodeResult::ok;
	}

	// If we get here then axes were specified so we are setting endstop parameters
	if (!reprap.GetGCodes().LockMovementAndWaitForStandstill(gb))
	{
		return GCodeResult::notFinished;
	}

	activeEndstops = nullptr;			// we may be about to remove endstops, so make sure they are not in the active list

	const EndStopType inputType = (gb.Seen('S')) ? (EndStopType)gb.GetUIValue() : EndStopType::inputPin;
	if (inputType >= EndStopType::numInputTypes)
	{
		reply.copy("invalid input type");
		return GCodeResult::error;
	}
	if (inputType == EndStopType::unused_wasActiveLow)
	{
		reply.copy("endstop type 0 is no longer supported. Use type 1 and invert the input pin instead.");
		return GCodeResult::error;
	}

	if (gb.Seen('P'))					// we use P not C, because C may be an axis
	{
		// Setting the port number(s), so there must be just one axis and we must be using switch-type endstops
		if (axesSeen > 1 || inputType != EndStopType::inputPin)
		{
			reply.copy("Invalid use of P parameter");
			return GCodeResult::error;
		}

		WriteLocker lock(endstopsLock);

		delete axisEndstops[lastAxisSeen];
		axisEndstops[lastAxisSeen] = nullptr;
		SwitchEndstop * const sw = new SwitchEndstop(lastAxisSeen, lastPosSeen);
		const GCodeResult rslt = sw->Configure(gb, reply);
		if (rslt == GCodeResult::ok)
		{
			axisEndstops[lastAxisSeen] = sw;
		}
		else
		{
			delete sw;
		}
		return rslt;
	}

	// No P parameter, so there may be multiple axes
	for (size_t axis = 0; axis < reprap.GetGCodes().GetTotalAxes(); ++axis)
	{
		const char c = reprap.GetGCodes().GetAxisLetters()[axis];
		if (gb.Seen(c))
		{
			const EndStopPosition pos = (EndStopPosition)gb.GetUIValue();		// we range-checked this earlier
			WriteLocker lock(endstopsLock);

			if (pos == EndStopPosition::noEndStop)
			{
				DeleteObject(axisEndstops[axis]);
			}
			else
			{
				switch (inputType.ToBaseType())
				{
#if HAS_STALL_DETECT
				case EndStopType::motorStallAny:
					// Asking for stall detection endstop, so we can delete any existing endstop(s) and create new ones
					ReplaceObject(axisEndstops[axis], new StallDetectionEndstop(axis, pos, false));
					break;

				case EndStopType::motorStallIndividual:
					// Asking for stall detection endstop, so we can delete any existing endstop(s) and create new ones
					ReplaceObject(axisEndstops[axis], new StallDetectionEndstop(axis, pos, true));
					break;
#else
				case EndStopType::motorStallAny:
				case EndStopType::motorStallIndividual:
					DeleteObject(axisEndstops[axis]);
					reply.copy("Stall detection not supported by this hardware");
					return GCodeResult::error;
#endif
				case EndStopType::zProbeAsEndstop:
					// Asking for a ZProbe or stall detection endstop, so we can delete any existing endstop(s) and create new ones
					ReplaceObject(axisEndstops[axis], new ZProbeEndstop(axis, pos));
					break;

				case EndStopType::inputPin:
					if (   axisEndstops[axis] == nullptr
						|| axisEndstops[axis]->GetEndstopType() != EndStopType::inputPin
					   )
					{
						// Asking for a switch endstop but we don't already have one, so we don't know what pin number(s) it should use
						reply.printf("Pin name needed for switch-type endstop on %c axis", c);
						return GCodeResult::error;
					}
					else
					{
						axisEndstops[axis]->SetAtHighEnd(pos == EndStopPosition::highEndStop);
					}
					break;

				default:
					break;
				}
			}
		}
	}

	return GCodeResult::ok;
}

EndStopPosition EndstopsManager::GetEndStopPosition(size_t axis) const noexcept
{
	return (axisEndstops[axis] == nullptr) ? EndStopPosition::noEndStop
			: (axisEndstops[axis]->GetAtHighEnd()) ? EndStopPosition::highEndStop
				: EndStopPosition::lowEndStop;
}

// Return true if we are using a bed probe to home Z
bool EndstopsManager::HomingZWithProbe() const noexcept
{
	return axisEndstops[Z_AXIS] == nullptr || axisEndstops[Z_AXIS]->GetEndstopType() == EndStopType::zProbeAsEndstop;
}

bool EndstopsManager::Stopped(size_t axis) const noexcept
{
	return (axisEndstops[axis] != nullptr) && axisEndstops[axis]->Stopped();
}

void EndstopsManager::GetM119report(const StringRef& reply) noexcept
{
	reply.copy("Endstops - ");
	for (size_t axis = 0; axis < reprap.GetGCodes().GetTotalAxes(); ++axis)
	{
		const char * const status = (axisEndstops[axis] == nullptr)
										? "no endstop"
											: TranslateEndStopResult(axisEndstops[axis]->Stopped(), axisEndstops[axis]->GetAtHighEnd());
		reply.catf("%c: %s, ", reprap.GetGCodes().GetAxisLetters()[axis], status);
	}
	reply.catf("Z probe: %s", TranslateEndStopResult(GetZProbeOrDefault(0)->Stopped(), false));
}

const char *EndstopsManager::TranslateEndStopResult(bool hit, bool atHighEnd) noexcept
{
	if (hit)
	{
		return (atHighEnd) ? "at max stop" : "at min stop";
	}

	return "not stopped";
}

void EndstopsManager::SetZProbeDefaults() noexcept
{
	zProbes[0]->SetDefaults();
	for (size_t i = 0; i < MaxZProbes; ++i)
	{
		DeleteObject(zProbes[i]);
	}
}

// Program the Z probe
GCodeResult EndstopsManager::ProgramZProbe(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	const unsigned int probeNumber = (gb.Seen('K')) ? gb.GetLimitedUIValue('K', MaxZProbes) : 0;
	ReadLocker lock(zProbesLock);
	ZProbe * const zProbe = zProbes[probeNumber];
	if (zProbe == nullptr)
	{
		reply.copy("Invalid Z probe index");
		return GCodeResult::error;
	}

	if (gb.Seen('S'))
	{
		uint32_t zProbeProgram[MaxZProbeProgramBytes];
		size_t len = MaxZProbeProgramBytes;
		gb.GetUnsignedArray(zProbeProgram, len, false);
		if (len != 0)
		{
			for (size_t i = 0; i < len; ++i)
			{
				if (zProbeProgram[i] > 255)
				{
					reply.copy("Out of range value in program bytes");
					return GCodeResult::error;
				}
			}
			return zProbe->SendProgram(zProbeProgram, len, reply);
		}
	}
	reply.copy("No program bytes provided");
	return GCodeResult::error;
}

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE

bool EndstopsManager::WriteZProbeParameters(FileStore *f, bool includingG31) const noexcept
{
	bool ok = true;
	bool written = false;
	for (size_t i = 0; i < MaxZProbes; ++i)
	{
		ZProbe * const zp = zProbes[i];
		if (zp != nullptr)
		{
			if (includingG31)
			{
				zp->SetSaveToConfigOverride();
			}
			if (zp->GetSaveToConfigOverride())
			{
				if (!written)
				{
					ok = f->Write("; Z probe parameters\n");
					written = true;
				}
				if (ok)
				{
					ok = zp->WriteParameters(f, i);
				}
			}
		}
	}
	return ok;
}

#endif

// Handle M558
GCodeResult EndstopsManager::HandleM558(GCodeBuffer& gb, const StringRef &reply) THROWS(GCodeException)
{
	const unsigned int probeNumber = (gb.Seen('K')) ? gb.GetLimitedUIValue('K', MaxZProbes) : 0;

	// Check what sort of Z probe we need and where it is, so see whether we need to delete any existing one and create a new one.
	// If there is no probe, we need a new one; and if it is not a motor stall one then a port number must be given.
	// If we are switching between motor stall and any other type, we need a new one. A port must be specified unless it is motor stall.
	// If it not a motor stall probe and a port number is given, we need a new one in case it is on a different board.
	// If it is a motor stall endstop, there should not be a port specified, but we can ignore the port if it is present
	uint32_t probeType = (uint32_t)ZProbeType::none;
	bool seenType = false;
	gb.TryGetUIValue('P', probeType, seenType);
	if (   seenType
		&& (   probeType >= (uint32_t)ZProbeType::numTypes
			|| probeType == (uint32_t)ZProbeType::e1Switch_obsolete
			|| probeType == (uint32_t)ZProbeType::endstopSwitch_obsolete
			|| probeType == (uint32_t)ZProbeType::zSwitch_obsolete
		   )
	   )
	{
		reply.printf("Invalid Z probe type %" PRIu32, probeType);
		return GCodeResult::error;
	}

	WriteLocker lock(zProbesLock);

	ZProbe * const existingProbe = zProbes[probeNumber];
	if (existingProbe == nullptr && !seenType)
	{
		reply.printf("Z probe %u not found", probeNumber);
		return GCodeResult::error;
	}

	const bool seenPort = gb.Seen('C');
	bool seen = seenType || seenPort;

	if (seen)									// we need a new probe if we have seen either P or C
	{
		if (!seenType)							// if a port is specified then the type must be specified too
		{
			reply.copy("Missing Z probe type number");
			return GCodeResult::error;
		}

		DeleteObject(zProbes[probeNumber]);		// delete the old probe first, the new one might use the same ports

		ZProbe *newProbe;
		switch ((ZProbeType)probeType)
		{
		case ZProbeType::none:
			newProbe = new DummyZProbe(probeNumber);
			break;

		case ZProbeType::zMotorStall:
			newProbe = new MotorStallZProbe(probeNumber);
			break;

		default:
			if (!seenPort)
			{
				reply.copy("Missing Z probe pin name(s)");
				return GCodeResult::error;
			}
			{
#if SUPPORT_CAN_EXPANSION
				String<StringLength20> pinNames;
				gb.Seen('C');
				gb.GetReducedString(pinNames.GetRef());
				const CanAddress boardAddress = IoPort::RemoveBoardAddress(pinNames.GetRef());
				if (boardAddress != CanInterface::GetCanAddress())
				{
					RemoteZProbe *newRemoteProbe = new RemoteZProbe(probeNumber, boardAddress, (ZProbeType)probeType);
					const GCodeResult rslt = newRemoteProbe->Create(pinNames.GetRef(), reply);
					if (rslt != GCodeResult::ok)
					{
						delete newRemoteProbe;
						return rslt;
					}
					newProbe = newRemoteProbe;
				}
				else
#endif
				{
					newProbe = new LocalZProbe(probeNumber);
				}
			}
			break;
		}

		const GCodeResult rslt = newProbe->Configure(gb, reply, seen);
		if (rslt == GCodeResult::ok || rslt == GCodeResult::warning)
		{
			zProbes[probeNumber] = newProbe;
		}
		return rslt;
	}

	// If we get here then there is an existing probe of the correct type and we just need to change its configuration
	return zProbes[probeNumber]->Configure(gb, reply, seen);
}

// Set or print the Z probe. Called by G31.
// Note that G31 P or G31 P0 prints the parameters of the currently-selected Z probe.
GCodeResult EndstopsManager::HandleG31(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	const unsigned int probeNumber = (gb.Seen('K')) ? gb.GetLimitedUIValue('K', MaxZProbes) : 0;
	ReadLocker lock(zProbesLock);
	ZProbe * const zp = zProbes[probeNumber];
	if (zp == nullptr)
	{
		reply.copy("Invalid Z probe index");
		return GCodeResult::error;
	}

	return zp->HandleG31(gb, reply);
}

#if SUPPORT_OBJECT_MODEL

size_t EndstopsManager::GetNumProbesToReport() const noexcept
{
	size_t ret = MaxZProbes;
	while (ret != 0 && zProbes[ret - 1] == nullptr)
	{
		--ret;
	}
	return ret;
}

#endif


#if SUPPORT_CAN_EXPANSION

// Handle signalling of a remote switch change, when the handle indicates that it is being used as an endstop.
void EndstopsManager::HandleRemoteEndstopChange(CanAddress src, uint8_t handleMajor, uint8_t handleMinor, bool state) noexcept
{
	if (handleMajor < ARRAY_SIZE(axisEndstops))
	{
		Endstop * const es = axisEndstops[handleMajor];
		if (es != nullptr)
		{
			es->HandleRemoteInputChange(src, handleMinor, state);
		}
	}
}

// Handle signalling of a remote switch change, when the handle indicates that it is being used as a Z probe.
void EndstopsManager::HandleRemoteZProbeChange(CanAddress src, uint8_t handleMajor, uint8_t handleMinor, bool state) noexcept
{
	if (handleMajor < ARRAY_SIZE(zProbes))
	{
		ZProbe * const zp = zProbes[handleMajor];
		if (zp != nullptr)
		{
			zp->HandleRemoteInputChange(src, handleMinor, state);
		}
	}
}

// This is called when we update endstop states because of a message from a remote board.
// In time we may use it to help implement interrupt-driven local endstops too, but for now those are checked in the step ISR by a direct call to DDA::CheckEndstops().
void EndstopsManager::OnEndstopOrZProbeStatesChanged() noexcept
{
	const uint32_t oldPrio = ChangeBasePriority(NvicPriorityStep);		// shut out the step interrupt

	DDA * const currentDda = reprap.GetMove().GetMainDDARing().GetCurrentDDA();
	if (currentDda != nullptr && currentDda->IsCheckingEndstops())
	{
		Platform& p = reprap.GetPlatform();
		currentDda->CheckEndstops(p);
		if (currentDda->GetState() == DDA::completed)
		{
			reprap.GetMove().GetMainDDARing().OnMoveCompleted(currentDda, p);
		}
	}

	RestoreBasePriority(oldPrio);										// allow step interrupts again
}

#endif

// End
