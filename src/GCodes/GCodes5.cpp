/*
 * GCodes5.cpp
 *
 *  Created on: 8 Mar 2022
 *      Author: David
 *
 *  Purpose: Tool management
 */

#include "GCodes.h"
#include "GCodeBuffer/GCodeBuffer.h"
#include <Tools/Tool.h>
#include <Platform/RepRap.h>
#include <Heating/Heat.h>

// Check if the specified heater is used by a current tool other than the specified one
bool GCodes::IsHeaterUsedByDifferentCurrentTool(int heaterNumber, const Tool *tool) const noexcept
{
	for (const MovementState& ms : moveStates)
	{
		if (ms.currentTool != nullptr && ms.currentTool != tool && ms.currentTool->UsesHeater(heaterNumber))
		{
			return true;
		}
	}
	return false;
}

// Report the temperatures of one tool in M105 format
void GCodes::ReportToolTemperatures(const StringRef& reply, const Tool *tool, bool includeNumber) const noexcept
{
	if (tool != nullptr && tool->HeaterCount() != 0)
	{
		if (reply.strlen() != 0)
		{
			reply.cat(' ');
		}
		if (includeNumber)
		{
			reply.catf("T%u", tool->Number());
		}
		else
		{
			reply.cat("T");
		}

		Heat& heat = reprap.GetHeat();
		char sep = ':';
		for (size_t i = 0; i < tool->HeaterCount(); ++i)
		{
			const int heater = tool->GetHeater(i);
			reply.catf("%c%.1f /%.1f", sep, (double)heat.GetHeaterTemperature(heater), (double)heat.GetTargetTemperature(heater));
			sep = ' ';
		}
	}
}

#if SUPPORT_ASYNC_MOVES

// Handle M400
GCodeResult GCodes::ExecuteM400(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	if (LockCurrentMovementSystemAndWaitForStandstill(gb))
	{
		uint32_t param = 0;
		bool seen = false;
		gb.TryGetLimitedUIValue('S', param, seen, 2);
		if (param != 1)
		{
			// M400 releases axes/extruders that are not owned by the current tool unless the S1 parameter is present
			// Workaround for loss of correct coordinates when M400 is used in deployprobe.g or retractprobe.g (issue 978)
			// Review this after we have implemented a check for moving unowned axes and/or automatic axis allocation for internally-generated moves
			bool doingProbing = false;
			const GCodeMachineState *mc = &gb.LatestMachineState();
			do
			{
				if (mc->GetState() >= GCodeState::firstProbingState && mc->GetState() <= GCodeState::lastProbingState)
				{
					doingProbing = true;
					break;
				}
				mc = mc->GetPrevious();
			}
			while (mc != nullptr);
			if (!doingProbing)
			// End of workaround
			{
				GetMovementState(gb).ReleaseNonToolAxesAndExtruders();
			}
		}
		return GCodeResult::ok;
	}
	return GCodeResult::notFinished;
}

// Handle M596
GCodeResult GCodes::SelectMovementQueue(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	if (gb.Seen('P'))
	{
		UnlockMovement(gb);							// in case we are in a macro - avoid unlocking the wrong movement system later
		const MovementSystemNumber queueNumber = gb.GetLimitedUIValue('P', ARRAY_SIZE(moveStates));
		gb.SetActiveQueueNumber(queueNumber);
		reprap.InputsUpdated();
	}
	else
	{
		reply.printf("Motion system %u is active", gb.GetActiveQueueNumber());
	}
	return GCodeResult::ok;
}

// Handle M597
GCodeResult GCodes::CollisionAvoidance(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	// Find the two specified axes
	int lowerAxisNumber = -1, upperAxisNumber = -1;
	float lowerValue, upperValue;
	for (unsigned int i = 0; i < numVisibleAxes; ++i)
	{
		if (gb.Seen(axisLetters[i]))
		{
			if (lowerAxisNumber < 0)
			{
				lowerAxisNumber = i;
				lowerValue = gb.GetFValue();
			}
			else
			{
				upperAxisNumber = i;
				upperValue = gb.GetFValue();
				break;
			}
		}
	}

	if (upperAxisNumber >= 0)
	{
		// Seen two axes, so go ahead
		if (upperValue == lowerValue)
		{
			reply.copy("Axis values must be different");
			return GCodeResult::error;
		}
		if (upperValue < lowerValue)
		{
			std::swap(upperValue, lowerValue);
			std::swap(upperAxisNumber, lowerAxisNumber);
			collisionChecker.Set(lowerAxisNumber, upperAxisNumber, upperValue - lowerValue);
		}
	}
	else if (lowerAxisNumber >= 0)
	{
		reply.copy("Only one axis specified");
		return GCodeResult::error;
	}
	else if (collisionChecker.IsValid())
	{
		reply.printf("For collision avoidance, axis %c position must be at least %.1fmm higher than axis %c",
						axisLetters[collisionChecker.GetUpperAxis()], (double)collisionChecker.GetMinSeparation(), axisLetters[collisionChecker.GetLowerAxis()]);
	}
	else
	{
		reply.copy("Collision avoidance is not active");
	}
	return GCodeResult::ok;
}

// Handle M598
GCodeResult GCodes::SyncMovementSystems(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	return (DoSync(gb)) ? GCodeResult::ok : GCodeResult::notFinished;
}

// Handle M606
GCodeResult GCodes::ForkInputReader(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	if (gb.Seen('S'))
	{
		if (gb.GetChannel() != GCodeChannel::File)
		{
			reply.copy("this command is valid only when running a job from a stored file");
			return GCodeResult::warning;
		}

		(void)gb.GetLimitedUIValue('S', 1, 2);			// currently only S1 is valid

		if (gb.ExecutingAll())							// ignore the command if we are already forked
		{
			File2GCode()->ForkFrom(gb);					// duplicate the input settings and file stack of File to File2
			reprap.InputsUpdated();
		}
	}
	else
	{
		reply.printf("File reader is running in %s mode", (FileGCode()->ExecutingAll()) ? "standard" : "forked");
	}
	return GCodeResult::ok;
}

#endif

#if SUPPORT_KEEPOUT_ZONES

// Handle M599
GCodeResult GCodes::DefineKeepoutZone(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	// Get the optional P parameter. Currently it may only be zero.
	uint32_t zoneNumber = 0;
	bool seen = false;
	gb.TryGetLimitedUIValue('P', zoneNumber, seen, 1);

	return keepoutZone.Configure(gb, reply);
}

#endif

GCodeResult GCodes::HandleM486(GCodeBuffer &gb, const StringRef &reply, OutputBuffer*& buf) THROWS(GCodeException)
{
	bool seen = false;
	if (gb.Seen('T'))
	{
		// Specify how many objects. May be useful for a user interface.
		seen = true;
		buildObjects.HandleM486T(gb.GetUIValue());
	}

	if (gb.Seen('S'))
	{
		// Specify which object we are about to print
		seen = true;
		buildObjects.UseM486Labelling();

		const int num = gb.GetIValue();
		if (num >= 0 && num < (int)MaxTrackedObjects && gb.Seen('A'))
		{
			String<StringLength100> objectName;
			gb.GetQuotedString(objectName.GetRef());
			buildObjects.SetM486Label(num, objectName.c_str());
		}
		ChangeToObject(gb, num);
	}

	const bool seenC = gb.Seen('C');
	if (seenC || gb.Seen('P'))
	{
		// Cancel an object
		seen = true;
		const int objectToCancel = (seenC) ? GetMovementState(gb).currentObjectNumber : (int)gb.GetUIValue();
		if (objectToCancel < 0)
		{
			reply.copy("No current object");
			return GCodeResult::error;
		}

		if (buildObjects.CancelObject((unsigned int)objectToCancel))
		{
			for (MovementState& ms : moveStates)
			{
				if (objectToCancel == ms.currentObjectNumber)
				{
					ms.StopPrinting(gb);
				}
			}
			reply.printf("Object %d cancelled", objectToCancel);
		}
	}

	if (gb.Seen('U'))
	{
		// Resume an object
		seen = true;
		const unsigned int objectToResume = gb.GetUIValue();
		if (buildObjects.ResumeObject(objectToResume))
		{
			for (MovementState& ms : moveStates)
			{
				if ((int)objectToResume == ms.currentObjectNumber)
				{
					ms.ResumePrinting(gb);
				}
			}
			reprap.JobUpdated();
		}
	}

	if (!seen)
	{
		// List objects on build plate
		if (!OutputBuffer::Allocate(buf))
		{
			return GCodeResult::notFinished;
		}

		buildObjects.ListObjects(buf);
	}

	return GCodeResult::ok;
}

// This is called when we have found an object label in a comment
void GCodes::StartObject(GCodeBuffer& gb, const char *_ecv_array label) noexcept
{
	if (!buildObjects.IsUsingM486Naming())
	{
		const size_t objectNumber = buildObjects.GetObjectNumber(label);
		ChangeToObject(gb, objectNumber);
	}
}

// This is called when we have found a "stop printing object" comment
void GCodes::StopObject(GCodeBuffer& gb) noexcept
{
	if (!buildObjects.IsUsingM486Naming())
	{
		ChangeToObject(gb, -1);
	}
}

void GCodes::ChangeToObject(GCodeBuffer& gb, int objectNumber) noexcept
{
	MovementState& ms = GetMovementState(gb);
	ms.currentObjectNumber = objectNumber;
	const bool cancelCurrentObject = buildObjects.CheckObject(objectNumber);
	if (cancelCurrentObject && !ms.currentObjectCancelled)
	{
		ms.StopPrinting(gb);
	}
	else if (!cancelCurrentObject && ms.currentObjectCancelled)
	{
		ms.ResumePrinting(gb);
	}
}

// Process M204
GCodeResult GCodes::ConfigureAccelerations(GCodeBuffer&gb, const StringRef& reply) THROWS(GCodeException)
{
	MovementState& ms = GetMovementState(gb);
	bool seen = false;
	if (gb.Seen('S'))
	{
		// For backwards compatibility with old versions of Marlin (e.g. for Cura and the Prusa fork of slic3r), set both accelerations
		seen = true;
		ms.maxTravelAcceleration = ms.maxPrintingAcceleration = max<float>(gb.GetAcceleration(), ConvertAcceleration(MinimumAcceleration));
	}
	if (gb.Seen('P'))
	{
		seen = true;
		ms.maxPrintingAcceleration = max<float>(gb.GetAcceleration(), ConvertAcceleration(MinimumAcceleration));
	}
	if (gb.Seen('T'))
	{
		seen = true;
		ms.maxTravelAcceleration = max<float>(gb.GetAcceleration(), ConvertAcceleration(MinimumAcceleration));
	}
	if (seen)
	{
		reprap.MoveUpdated();
	}
	else
	{
		reply.printf("Maximum printing acceleration %.1f, maximum travel acceleration %.1f mm/sec^2",
						(double)InverseConvertAcceleration(ms.maxPrintingAcceleration), (double)InverseConvertAcceleration(ms.maxTravelAcceleration));
	}
	return GCodeResult::ok;
}

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE

// Save some resume information, returning true if successful
// We assume that the tool configuration doesn't change, only the temperatures and the mix
bool GCodes::WriteToolSettings(FileStore *f, const StringRef& buf) const noexcept
{
	// First write the settings of all tools
	bool ok = true;
	ReadLocker lock(Tool::toolListLock);
	for (const Tool *t = Tool::GetToolList(); t != nullptr && ok; t = t->Next())
	{
		ok = t->WriteSettings(f, buf);
	}
	return ok;
}

// Save some information in config-override.g
bool GCodes::WriteToolParameters(FileStore *f, const bool forceWriteOffsets) const noexcept
{
	bool ok = true, written = false;
	ReadLocker lock(Tool::toolListLock);
	for (const Tool *t = Tool::GetToolList(); ok && t != nullptr; t = t->Next())
	{
		const AxesBitmap axesProbed = t->GetAxisOffsetsProbed();
		if (axesProbed.IsNonEmpty() || forceWriteOffsets)
		{
			String<StringLength256> scratchString;
			if (!written)
			{
				scratchString.copy("; Probed tool offsets\n");
				written = true;
			}
			scratchString.catf("G10 P%d", t->Number());
			for (size_t axis = 0; axis < GetVisibleAxes(); ++axis)
			{
				if (forceWriteOffsets || axesProbed.IsBitSet(axis))
				{
					scratchString.catf(" %c%.3f", GetAxisLetters()[axis], (double)(t->GetOffset(axis)));
				}
			}
			scratchString.cat('\n');
			ok = f->Write(scratchString.c_str());
		}
	}
	return ok;
}

#endif

// End
