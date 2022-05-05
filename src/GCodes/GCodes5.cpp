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

// Handle M596
GCodeResult GCodes::SelectMovementQueue(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	const unsigned int queueNumber = gb.GetLimitedUIValue('P', ARRAY_SIZE(moveStates));
	UnlockMovement(gb);							// in case we are in a macro - avoid unlocking the wrong movement system later
	gb.SetActiveQueueNumber(queueNumber);
	reprap.InputsUpdated();
	return GCodeResult::ok;
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
			String<StringLength50> objectName;
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
		ms.maxTravelAcceleration = ms.maxPrintingAcceleration = gb.GetAcceleration();
	}
	if (gb.Seen('P'))
	{
		seen = true;
		ms.maxPrintingAcceleration = gb.GetAcceleration();
	}
	if (gb.Seen('T'))
	{
		seen = true;
		ms.maxTravelAcceleration = gb.GetAcceleration();
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
bool GCodes::WriteToolSettings(FileStore *f, const MovementState& ms) const noexcept
{
	// First write the settings of all tools except the current one and the command to select them if they are on standby
	bool ok = true;
	ReadLocker lock(Tool::toolListLock);
	for (const Tool *t = Tool::GetToolList(); t != nullptr && ok; t = t->Next())
	{
		if (t != ms.currentTool)
		{
			ok = t->WriteSettings(f);
		}
	}

	// Finally write the settings of the active tool and the commands to select it. If no current tool, just deselect all tools.
	if (ok)
	{
		if (ms.currentTool == nullptr)
		{
			ok = f->Write("T-1 P0\n");
		}
		else
		{
			ok = ms.currentTool->WriteSettings(f);
			if (ok)
			{
				String<StringLength20> buf;
				buf.printf("T%u P0\n", ms.currentTool->Number());
				ok = f->Write(buf.c_str());
			}
		}
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
			for (size_t axis = 0; axis < MaxAxes; ++axis)
			{
				if (forceWriteOffsets || axesProbed.IsBitSet(axis))
				{
					scratchString.catf(" %c%.2f", GetAxisLetters()[axis], (double)(t->GetOffset(axis)));
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
