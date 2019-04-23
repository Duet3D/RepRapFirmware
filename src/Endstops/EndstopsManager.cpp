/*
 * Endstop.cpp
 *
 *  Created on: 3 Apr 2019
 *      Author: David
 */

#include "EndstopsManager.h"
#include "Endstop.h"
#include "ZProbe.h"
#include "RepRap.h"
#include "GCodes/GCodeBuffer.h"
#include "GCodes/GCodes.h"
#include "Movement/Move.h"

EndstopsManager::EndstopsManager() : activeEndstops(nullptr)
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
	String<1> dummy;

	// Configure default endstops
	for (size_t axis = 0; axis < ARRAY_SIZE(DefaultEndstopPinNames); ++axis)
	{
		SwitchEndstop * const sw = new SwitchEndstop(axis, EndStopPosition::lowEndStop);
		sw->Configure(DefaultEndstopPinNames[axis], dummy.GetRef(), EndStopInputType::activeHigh);
		axisEndstops[axis] = sw;
	}

	// Z PROBE
	reprap.GetPlatform().InitZProbeFilters();

	ZProbe * const zp = new ZProbe();
	zp->AssignPorts(DefaultZProbePinNames, dummy.GetRef());
	zProbes[0] = zp;
	currentZProbeNumber = 0;

}

// Add an endstop to the active list
void EndstopsManager::AddToActive(EndstopOrZProbe& e)
{
	e.SetNext(activeEndstops);
	activeEndstops = &e;
}

// Set up the active endstop list according to the axes commanded to move in a G0/G1 S1/S3 command
void EndstopsManager::EnableAxisEndstops(AxesBitmap axes)
{
	activeEndstops = nullptr;
	const Kinematics& kin = reprap.GetMove().GetKinematics();
	for (size_t axis = 0; axis < reprap.GetGCodes().GetVisibleAxes(); ++axis)
	{
		if (IsBitSet(axes, axis) && axisEndstops[axis] != nullptr)
		{
			axisEndstops[axis]->Prime(kin, reprap.GetPlatform().GetAxisDriversConfig(axis));
			AddToActive(*axisEndstops[axis]);
		}
	}
}

// Set up the active endstops for Z probing
void EndstopsManager::EnableZProbe(size_t probeNumber)
{
	activeEndstops = nullptr;
	if (probeNumber < MaxZProbes && zProbes[probeNumber] != nullptr)
	{
		AddToActive(*zProbes[probeNumber]);
	}
}

// Check the endstops.
// If an endstop has triggered, remove it from the active list, return its action, and return a pointer to it via 'es'.
EndstopHitDetails EndstopsManager::CheckEndstops(bool goingSlow)
{
	EndstopHitDetails ret;									// the default constructor will clear all fields
	EndstopOrZProbe *actioned = nullptr;
	for (EndstopOrZProbe *esp = activeEndstops; esp != nullptr; esp = esp->GetNext())
	{
		const EndstopHitDetails hd = esp->CheckTriggered(goingSlow);
		if (hd.GetAction() == EndstopHitAction::stopAll)
		{
			activeEndstops = nullptr;						// no need to do anything else
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
		for (size_t axis = 0; axis < reprap.GetGCodes().GetTotalAxes(); ++axis)
		{
			const char c = reprap.GetGCodes().GetAxisLetters()[axis];
			if (axisEndstops[axis] == nullptr)
			{
				reply.catf("%c %c: none", sep, c);
			}
			else
			{
				EndStopInputType inputType = axisEndstops[axis]->GetEndstopType();
				reply.catf("%c %c: %s %s",
							sep, c,
							(axisEndstops[axis]->GetAtHighEnd()) ? "high end" : "low end",
							(inputType == EndStopInputType::activeHigh) ? "active high switch"
								: (inputType == EndStopInputType::activeLow) ? "active low switch"
									: (inputType == EndStopInputType::zProbeAsEndstop) ? "Z probe"
										: (inputType == EndStopInputType::motorStallAny) ? "motor stall (any motor)"
											: (inputType == EndStopInputType::motorStallIndividual) ? "motor stall (individual motors)"
											 : "unknown type"
						 );
				axisEndstops[axis]->AppendPinNames(reply);
			}
			sep = ',';
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

		delete axisEndstops[lastAxisSeen];
		axisEndstops[lastAxisSeen] = nullptr;
		SwitchEndstop * const sw = new SwitchEndstop(lastAxisSeen, lastPosSeen);
		const bool ok = sw->Configure(gb, reply, inputType);
		axisEndstops[lastAxisSeen] = sw;
		if (!ok)
		{
			return GCodeResult::error;
		}
	}
	else
	{
		// No P parameter, so there may be multiple axes
		for (size_t axis = 0; axis < reprap.GetGCodes().GetTotalAxes(); ++axis)
		{
			const char c = reprap.GetGCodes().GetAxisLetters()[axis];
			if (gb.Seen(c))
			{
				const EndStopPosition pos = (EndStopPosition)gb.GetUIValue();		// we range-checked this earlier
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
							reply.printf("Logical pin number needed for witch-type endstop on %c axis", c);
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
	return axisEndstops[axis]->Stopped();
}

void EndstopsManager::GetM119report(const StringRef& reply)
{
	reply.copy("Endstops - ");
	for (size_t axis = 0; axis < reprap.GetGCodes().GetTotalAxes(); ++axis)
	{
		reply.catf("%c: %s, ", reprap.GetGCodes().GetAxisLetters()[axis], TranslateEndStopResult(Stopped(axis), axisEndstops[axis]->GetAtHighEnd()));
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

// Allocate ports to the specified Z probe returning true if successful
bool EndstopsManager::AssignZProbePorts(GCodeBuffer& gb, const StringRef& reply, size_t probeNumber)
{
	if (probeNumber < MaxZProbes)
	{
		if (zProbes[probeNumber] == nullptr)
		{
			zProbes[probeNumber] = new ZProbe();
		}
		return zProbes[probeNumber]->AssignPorts(gb, reply);
	}
	reply.copy("Z probe number out of range");
	return false;
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
			zProbeProg.SendProgram(zProbeProgram, len);
			return GCodeResult::ok;
		}
	}
	reply.copy("No program bytes provided");
	return GCodeResult::error;
}

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

// Handle M558
GCodeResult EndstopsManager::HandleM558(GCodeBuffer& gb, const StringRef &reply)
{
	const unsigned int probeNumber = (gb.Seen('K')) ? gb.GetUIValue() : currentZProbeNumber;
	if (probeNumber >= MaxZProbes || zProbes[probeNumber] == nullptr)
	{
		reply.copy("Invalid Z probe index");
		return GCodeResult::error;
	}

	return zProbes[probeNumber]->HandleM558(gb, reply, probeNumber);
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

	return zProbes[probeNumber]->HandleG31(gb, reply, seenK);
}

// End
