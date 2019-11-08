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

#include "RepRap.h"
#include "GCodes/GCodeBuffer/GCodeBuffer.h"
#include "GCodes/GCodes.h"
#include "Movement/Move.h"

#if SUPPORT_CAN_EXPANSION
# include "CanMessageBuffer.h"
#endif

ReadWriteLock EndstopsManager::endstopsLock;					// used to lock both endstops and Z probes

EndstopsManager::EndstopsManager() : activeEndstops(nullptr), isHomingMove(false)
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

void EndstopsManager::Init()
{
	activeEndstops = nullptr;

#if ALLOCATE_DEFAULT_PORTS
	// Configure default endstops
	String<1> dummy;
	for (size_t axis = 0; axis < ARRAY_SIZE(DefaultEndstopPinNames); ++axis)
	{
		SwitchEndstop * const sw = new SwitchEndstop(axis, EndStopPosition::lowEndStop);
		sw->Configure(DefaultEndstopPinNames[axis], dummy.GetRef(), EndStopInputType::activeHigh);
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
	currentZProbeNumber = 0;
}

// Add an endstop to the active list
void EndstopsManager::AddToActive(EndstopOrZProbe& e)
{
	e.SetNext(activeEndstops);
	activeEndstops = &e;
}

// Set up the active endstop list according to the axes commanded to move in a G0/G1 S1/S3 command. Return true if successful.
bool EndstopsManager::EnableAxisEndstops(AxesBitmap axes, bool forHoming)
{
	activeEndstops = nullptr;
	isHomingMove = forHoming;
	const Kinematics& kin = reprap.GetMove().GetKinematics();
	for (size_t axis = 0; axis < reprap.GetGCodes().GetVisibleAxes(); ++axis)
	{
		if (IsBitSet(axes, axis))
		{
			if (axisEndstops[axis] != nullptr && axisEndstops[axis]->Prime(kin, reprap.GetPlatform().GetAxisDriversConfig(axis)))
			{
				AddToActive(*axisEndstops[axis]);
			}
			else
			{
				activeEndstops = nullptr;
				return false;
			}
		}
	}
	return true;
}

// Set up the active endstops for Z probing, returning true if successful
bool EndstopsManager::EnableZProbe(size_t probeNumber, bool probingAway)
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

// Enable extruder endstops
bool EndstopsManager::EnableExtruderEndstop(size_t extruder)
{
#ifdef NO_EXTRUDER_ENDSTOPS
	// not supported for now
	return false;
#else
	qq;		//TODO
#endif
}

// Check the endstops.
// If an endstop has triggered, remove it from the active list, return its action, and return a pointer to it via 'es'.
EndstopHitDetails EndstopsManager::CheckEndstops(bool goingSlow)
{
	EndstopHitDetails ret;									// the default constructor will clear all fields
	EndstopOrZProbe *actioned = nullptr;
	for (EndstopOrZProbe *esp = activeEndstops; esp != nullptr; esp = esp->GetNext())
	{
		EndstopHitDetails hd = esp->CheckTriggered(goingSlow);
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

	if (ret.GetAction() > EndstopHitAction::reduceSpeed)
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
GCodeResult EndstopsManager::HandleM574(GCodeBuffer& gb, const StringRef& reply)
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
		reply.copy("Endstop configuration");
		char sep = ':';
		ReadLocker lock(endstopsLock);

		for (size_t axis = 0; axis < reprap.GetGCodes().GetTotalAxes(); ++axis)
		{
			reply.catf("%c %c: ", sep, reprap.GetGCodes().GetAxisLetters()[axis]);
			sep = ',';
			if (axisEndstops[axis] == nullptr)
			{
				reply.cat("none");
			}
			else
			{
				reply.cat((axisEndstops[axis]->GetAtHighEnd()) ? "high end " : "low end ");
				axisEndstops[axis]->AppendDetails(reply);
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

	const EndStopInputType inputType = (gb.Seen('S')) ? (EndStopInputType)gb.GetUIValue() : EndStopInputType::activeHigh;
	if (inputType >= EndStopInputType::numInputTypes)
	{
		reply.copy("invalid input type");
		return GCodeResult::error;
	}

	if (gb.Seen('P'))					// we use P not C, because C may be an axis
	{
		// Setting the port number(s), so there must be just one axis and we must be using switch-type endstops
		if (axesSeen > 1 || (inputType != EndStopInputType::activeLow && inputType != EndStopInputType::activeHigh))
		{
			reply.copy("Invalid use of P parameter");
			return GCodeResult::error;
		}

		WriteLocker lock(endstopsLock);

		delete axisEndstops[lastAxisSeen];
		axisEndstops[lastAxisSeen] = nullptr;
		SwitchEndstop * const sw = new SwitchEndstop(lastAxisSeen, lastPosSeen);
		const GCodeResult rslt = sw->Configure(gb, reply, inputType);
		axisEndstops[lastAxisSeen] = sw;
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
				delete axisEndstops[axis];
				axisEndstops[axis] = nullptr;
			}
			else
			{
				switch (inputType)
				{
				case EndStopInputType::motorStallAny:
					// Asking for stall detection endstop, so we can delete any existing endstop(s) and create new ones
					delete axisEndstops[axis];
					axisEndstops[axis] = new StallDetectionEndstop(axis, pos, false);
					break;

				case EndStopInputType::motorStallIndividual:
					// Asking for stall detection endstop, so we can delete any existing endstop(s) and create new ones
					delete axisEndstops[axis];
					axisEndstops[axis] = new StallDetectionEndstop(axis, pos, true);
					break;

				case EndStopInputType::zProbeAsEndstop:
					// Asking for a ZProbe or stall detection endstop, so we can delete any existing endstop(s) and create new ones
					delete axisEndstops[axis];
					axisEndstops[axis] = new ZProbeEndstop(axis, pos);
					break;

				case EndStopInputType::activeHigh:
				case EndStopInputType::activeLow:
					if (   axisEndstops[axis] == nullptr
						|| (axisEndstops[axis]->GetEndstopType() != EndStopInputType::activeHigh && axisEndstops[axis]->GetEndstopType() != EndStopInputType::activeLow)
					   )
					{
						// Asking for a switch endstop but we don't already have one, so we don't know what pin number(s) it should use
						reply.printf("Pin name needed for switch-type endstop on %c axis", c);
						return GCodeResult::error;
					}
					else
					{
						((SwitchEndstop *)axisEndstops[axis])->Reconfigure(pos, inputType);
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

EndStopPosition EndstopsManager::GetEndStopPosition(size_t axis) const pre(axis < MaxAxes)
{
	return (axisEndstops[axis] == nullptr) ? EndStopPosition::noEndStop
			: (axisEndstops[axis]->GetAtHighEnd()) ? EndStopPosition::highEndStop
				: EndStopPosition::lowEndStop;
}

// Return true if we are using a bed probe to home Z
bool EndstopsManager::HomingZWithProbe() const
{
	return axisEndstops[Z_AXIS] == nullptr || axisEndstops[Z_AXIS]->GetEndstopType() == EndStopInputType::zProbeAsEndstop;
}

EndStopHit EndstopsManager::Stopped(size_t axis) const
{
	return (axisEndstops[axis] == nullptr) ? EndStopHit::noStop : axisEndstops[axis]->Stopped();
}

void EndstopsManager::GetM119report(const StringRef& reply)
{
	reply.copy("Endstops - ");
	for (size_t axis = 0; axis < reprap.GetGCodes().GetTotalAxes(); ++axis)
	{
		const char * const status = (axisEndstops == nullptr)
										? "no endstop"
											: TranslateEndStopResult(axisEndstops[axis]->Stopped(), axisEndstops[axis]->GetAtHighEnd());
		reply.catf("%c: %s, ", reprap.GetGCodes().GetAxisLetters()[axis], status);
	}
	reply.catf("Z probe: %s", TranslateEndStopResult(GetCurrentZProbe().Stopped(), false));
}

const char *EndstopsManager::TranslateEndStopResult(EndStopHit es, bool atHighEnd)
{
	switch (es)
	{
	case EndStopHit::atStop:
		return (atHighEnd) ? "at max stop" : "at min stop";

	case EndStopHit::nearStop:
		return "near stop";

	case EndStopHit::noStop:
	default:
		return "not stopped";
	}
}

ZProbe& EndstopsManager::GetCurrentZProbe() const
{
	ZProbe * const zp = zProbes[currentZProbeNumber];
	return (zp == nullptr) ? *defaultZProbe : *zp;
}

ZProbe *EndstopsManager::GetZProbe(size_t num) const
{
	return (num < ARRAY_SIZE(zProbes)) ? zProbes[num] : nullptr;
}

void EndstopsManager::SetZProbeDefaults()
{
	zProbes[0]->SetDefaults();
	for (size_t i = 0; i < MaxZProbes; ++i)
	{
		delete zProbes[i];
	}
}

// Program the Z probe
GCodeResult EndstopsManager::ProgramZProbe(GCodeBuffer& gb, const StringRef& reply)
{
	const uint32_t probeNumber = (gb.Seen('K')) ? gb.GetUIValue() : currentZProbeNumber;
	if (probeNumber >= MaxZProbes || zProbes[probeNumber] == nullptr)
	{
		reply.copy("Invalid Z probe index");
		return GCodeResult::error;
	}

	ZProbe * const zProbe = zProbes[probeNumber];
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

#if HAS_MASS_STORAGE

bool EndstopsManager::WriteZProbeParameters(FileStore *f, bool includingG31) const
{
	bool ok = true;
	bool written = false;
	for (size_t i = 0; i < MaxZProbes; ++i)
	{
		if (zProbes[i] != nullptr && (includingG31 || zProbes[i]->GetSaveToConfigOverride()))
		{
			if (!written)
			{
				ok = f->Write("; Z probe parameters\n");
				written = true;
			}
			if (ok)
			{
				ok = zProbes[i]->WriteParameters(f, i);
			}
		}
	}
	return ok;
}

#endif

// Handle M558
GCodeResult EndstopsManager::HandleM558(GCodeBuffer& gb, const StringRef &reply)
{
	const unsigned int probeNumber = (gb.Seen('K')) ? gb.GetUIValue() : currentZProbeNumber;
	if (probeNumber >= MaxZProbes)
	{
		reply.copy("Invalid Z probe index");
		return GCodeResult::error;
	}

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

	WriteLocker lock(endstopsLock);

	ZProbe * const existingProbe = zProbes[probeNumber];
	if (existingProbe == nullptr && !seenType)
	{
		reply.printf("Z probe %u not found", probeNumber);
		return GCodeResult::error;
	}

	const bool seenPort = gb.Seen('C');
	const bool needNewProbe =  (existingProbe == nullptr)
							|| (   seenType
								&& probeType != (uint32_t)existingProbe->GetProbeType()
								&& (   probeType == (uint32_t)ZProbeType::zMotorStall
									|| probeType == (uint32_t)ZProbeType::none
									|| existingProbe->GetProbeType() == ZProbeType::zMotorStall
									|| existingProbe->GetProbeType() == ZProbeType::none
								   )
								)
							|| (seenPort && existingProbe->GetProbeType() != ZProbeType::zMotorStall && existingProbe->GetProbeType() != ZProbeType::none);

	bool seen = seenType || seenPort;
	if (needNewProbe)
	{
		if (!seenType)
		{
			reply.copy("Missing Z probe type number");
			return GCodeResult::error;
		}

		zProbes[probeNumber] = nullptr;
		delete existingProbe;					// delete the old probe first, the new one might use the same ports

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
				if (boardAddress != CanId::MasterAddress)
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

	// If we get get then there is an existing probe and we just need to change its configuration
	return zProbes[probeNumber]->Configure(gb, reply, seen);
}

// Set or print the Z probe. Called by G31.
// Note that G31 P or G31 P0 prints the parameters of the currently-selected Z probe.
GCodeResult EndstopsManager::HandleG31(GCodeBuffer& gb, const StringRef& reply)
{
	uint32_t probeNumber = currentZProbeNumber;
	bool seenK = false;
	gb.TryGetUIValue('K', probeNumber, seenK);
	if (probeNumber >= MaxZProbes || zProbes[probeNumber] == nullptr)
	{
		reply.copy("Invalid Z probe index");
		return GCodeResult::error;
	}

	return zProbes[probeNumber]->HandleG31(gb, reply);
}

#if SUPPORT_CAN_EXPANSION

// Handle signalling of a remote switch change, when the handle indicates that it is being used as an endstop.
// We must re-use or free the buffer.
void EndstopsManager::HandleRemoteInputChange(CanAddress src, uint8_t handleMajor, uint8_t handleMinor, bool state)
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

// This is called when we update endstop states because of a message from a remote board.
// In time we may use it to help implement interrupt-driven local endstops too, but for now those are checked in the step ISR by a direct call to DDA::CheckEndstops().
void EndstopsManager::OnEndstopStatesChanged()
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

	RestoreBasePriority(oldPrio);								// allow step interrupts again
}

#endif

// End
