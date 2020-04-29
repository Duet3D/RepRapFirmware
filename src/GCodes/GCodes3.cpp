/*
 * GCodes3.cpp
 *
 *  Created on: 5 Dec 2017
 *      Author: David
 *  This file contains functions that are called form file GCodes2.cpp to execute various G and M codes.
 */

#include <Movement/StraightProbeSettings.h>
#include "GCodes.h"

#include "GCodeBuffer/GCodeBuffer.h"
#include "Heating/Heat.h"
#include "Movement/Move.h"
#include "RepRap.h"
#include "Tools/Tool.h"
#include "Endstops/ZProbe.h"
#include "PrintMonitor.h"
#include "Tasks.h"
#include "Hardware/I2C.h"

#if HAS_WIFI_NETWORKING
# include "FirmwareUpdater.h"
#endif

#if SUPPORT_TMC2660
# include "Movement/StepperDrivers/TMC2660.h"
#endif
#if SUPPORT_TMC22xx
# include "Movement/StepperDrivers/TMC22xx.h"
#endif
#if SUPPORT_TMC51xx
# include "Movement/StepperDrivers/TMC51xx.h"
#endif

#if SUPPORT_CAN_EXPANSION
# include <CAN/CanInterface.h>
# include <CAN/ExpansionManager.h>
#endif

#include "Wire.h"

// Deal with G60
GCodeResult GCodes::SavePosition(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	const uint32_t sParam = (gb.Seen('S')) ? gb.GetUIValue() : 0;
	if (sParam < ARRAY_SIZE(numberedRestorePoints))
	{
		SavePosition(numberedRestorePoints[sParam], gb);
		numberedRestorePoints[sParam].toolNumber = reprap.GetCurrentToolNumber();
		return GCodeResult::ok;
	}

	reply.copy("Bad restore point number");
	return GCodeResult::error;
}

// This handles G92. Return true if completed, false if it needs to be called again.
GCodeResult GCodes::SetPositions(GCodeBuffer& gb) THROWS(GCodeException)
{
	// Don't wait for the machine to stop if only extruder drives are being reset.
	// This avoids blobs and seams when the gcode uses absolute E coordinates and periodically includes G92 E0.
	AxesBitmap axesIncluded;
	for (size_t axis = 0; axis < numVisibleAxes; ++axis)
	{
		if (gb.Seen(axisLetters[axis]))
		{
			const float axisValue = gb.GetFValue();
			if (axesIncluded.IsEmpty())
			{
				if (!LockMovementAndWaitForStandstill(gb))	// lock movement and get current coordinates
				{
					return GCodeResult::notFinished;
				}
			}
			axesIncluded.SetBit(axis);
			currentUserPosition[axis] = gb.ConvertDistance(axisValue);
		}
	}

	// Handle any E parameter in the G92 command
	if (gb.Seen(extrudeLetter))
	{
		virtualExtruderPosition = gb.GetDistance();
	}

	if (axesIncluded.IsNonEmpty())
	{
		ToolOffsetTransform(currentUserPosition, moveBuffer.coords);
		if (reprap.GetMove().GetKinematics().LimitPosition(moveBuffer.coords, nullptr, numVisibleAxes, AxesBitmap::MakeLowestNBits(numVisibleAxes), false, limitAxes)
			!= LimitPositionResult::ok												// pretend that all axes are homed
		   )
		{
			ToolOffsetInverseTransform(moveBuffer.coords, currentUserPosition);		// make sure the limits are reflected in the user position
		}
		reprap.GetMove().SetNewPosition(moveBuffer.coords, true);
		axesHomed |= reprap.GetMove().GetKinematics().AxesAssumedHomed(axesIncluded);
		if (axesIncluded.IsBitSet(Z_AXIS))
		{
			zDatumSetByProbing -= false;
		}

#if SUPPORT_ROLAND
		if (reprap.GetRoland()->Active())
		{
			for(size_t axis = 0; axis < AXES; axis++)
			{
				if (!reprap.GetRoland()->ProcessG92(moveBuffer[axis], axis))
				{
					return GCodeResult::notFinished;
				}
			}
		}
#endif
	}

	return GCodeResult::ok;
}

// Offset the axes by the X, Y, and Z amounts in the M226 code in gb. The actual movement occurs on the next move command.
// It's not clear from the description in the reprap.org wiki whether offsets are cumulative or not. We now assume they are not.
// Note that M206 offsets are actually negative offsets.
GCodeResult GCodes::OffsetAxes(GCodeBuffer& gb, const StringRef& reply)
{
	bool seen = false;
	for (size_t axis = 0; axis < numVisibleAxes; axis++)
	{
		if (gb.Seen(axisLetters[axis]))
		{
			workplaceCoordinates[currentCoordinateSystem][axis] = -gb.GetDistance();
			seen = true;
		}
	}

	if (seen)
	{
		reprap.MoveUpdated();
	}
	else
	{
		reply.printf("Axis offsets:");
		for (size_t axis = 0; axis < numVisibleAxes; axis++)
		{
			reply.catf(" %c%.2f", axisLetters[axis], -(double)(gb.InverseConvertDistance(workplaceCoordinates[0][axis])));
		}
	}

	return GCodeResult::ok;
}

#if SUPPORT_WORKPLACE_COORDINATES

// Set workspace coordinates
GCodeResult GCodes::GetSetWorkplaceCoordinates(GCodeBuffer& gb, const StringRef& reply, bool compute)
{
	const uint32_t cs = (gb.Seen('P')) ? gb.GetIValue() : currentCoordinateSystem + 1;
	if (cs > 0 && cs <= NumCoordinateSystems)
	{
		bool seen = false;
		for (size_t axis = 0; axis < numVisibleAxes; axis++)
		{
			if (gb.Seen(axisLetters[axis]))
			{
				const float coord = gb.GetDistance();
				if (!seen)
				{
					if (!LockMovementAndWaitForStandstill(gb))						// make sure the user coordinates are stable and up to date
					{
						return GCodeResult::notFinished;
					}
					seen = true;
				}
				workplaceCoordinates[cs - 1][axis] = (compute) ? currentUserPosition[axis] - coord : coord;
			}
		}

		if (seen)
		{
			reprap.MoveUpdated();
		}
		else
		{
			reply.printf("Origin of workplace %" PRIu32 ":", cs);
			for (size_t axis = 0; axis < numVisibleAxes; axis++)
			{
				reply.catf(" %c%.2f", axisLetters[axis], (double)gb.InverseConvertDistance(workplaceCoordinates[cs - 1][axis]));
			}
		}
		return GCodeResult::ok;
	}

	return GCodeResult::badOrMissingParameter;
}

# if HAS_MASS_STORAGE

// Save all the workplace coordinate offsets to file returning true if successful. Used by M500 and by SaveResumeInfo.
bool GCodes::WriteWorkplaceCoordinates(FileStore *f) const noexcept
{
	if (!f->Write("; Workplace coordinates\n"))
	{
		return false;
	}

	for (size_t cs = 0; cs < NumCoordinateSystems; ++cs)
	{
		String<StringLength100> scratchString;
		scratchString.printf("G10 L2 P%u", cs + 1);
		for (size_t axis = 0; axis < numVisibleAxes; ++axis)
		{
			scratchString.catf(" %c%.2f", axisLetters[axis], (double)workplaceCoordinates[cs][axis]);
		}
		scratchString.cat('\n');
		if (!f->Write(scratchString.c_str()))
		{
			return false;
		}
	}
	return true;
}

#endif

#endif

// Define the probing grid, called when we see an M557 command
GCodeResult GCodes::DefineGrid(GCodeBuffer& gb, const StringRef &reply)
{
	if (!LockMovement(gb))							// to ensure that probing is not already in progress
	{
		return GCodeResult::notFinished;
	}

	bool seenX = false, seenY = false, seenR = false, seenP = false, seenS = false;
	float xValues[2];
	float yValues[2];
	float spacings[2] = { DefaultGridSpacing, DefaultGridSpacing };

	if (gb.TryGetFloatArray('X', 2, xValues, reply, seenX, false))
	{
		return GCodeResult::error;
	}
	if (gb.TryGetFloatArray('Y', 2, yValues, reply, seenY, false))
	{
		return GCodeResult::error;
	}

	uint32_t numPoints[2];
	if (gb.TryGetUIArray('P', 2, numPoints, reply, seenP, true))
	{
		return GCodeResult::error;
	}
	if (!seenP)
	{
		if (gb.TryGetFloatArray('S', 2, spacings, reply, seenS, true))
		{
			return GCodeResult::error;
		}
	}

	float radius = -1.0;
	gb.TryGetFValue('R', radius, seenR);

	if (!seenX && !seenY && !seenR && !seenS && !seenP)
	{
		// Just print the existing grid parameters
		if (defaultGrid.IsValid())
		{
			reply.copy("Grid: ");
			defaultGrid.PrintParameters(reply);
		}
		else
		{
			reply.copy("Grid is not defined");
		}
		return GCodeResult::ok;
	}

	if (seenX != seenY)
	{
		reply.copy("specify both or neither of X and Y in M557");
		return GCodeResult::error;
	}

	if (!seenX && !seenR)
	{
		// Must have given just the S or P parameter
		reply.copy("specify at least radius or X and Y ranges in M557");
		return GCodeResult::error;
	}

	if (seenX)
	{
		// Seen both X and Y
		if (seenP)
		{
			if (spacings[0] >= 2 && xValues[1] > xValues[0])
			{
				spacings[0] = (xValues[1] - xValues[0])/(numPoints[0] - 1);
			}
			if (spacings[1] >= 2 && yValues[1] > yValues[0])
			{
				spacings[1] = (yValues[1] - yValues[0])/(numPoints[1] - 1);
			}
		}
	}
	else
	{
		// Seen R
		if (radius > 0.0)
		{
			float effectiveXRadius;
			if (seenP && numPoints[0] >= 2)
			{
				effectiveXRadius = radius - 0.1;
				if (numPoints[1] % 2 == 0)
				{
					effectiveXRadius *= sqrtf(1.0 - 1.0/(float)((numPoints[1] - 1) * (numPoints[1] - 1)));
				}
				spacings[0] = (2 * effectiveXRadius)/(numPoints[0] - 1);
			}
			else
			{
				effectiveXRadius = floorf((radius - 0.1)/spacings[0]) * spacings[0];
			}
			xValues[0] = -effectiveXRadius;
			xValues[1] =  effectiveXRadius + 0.1;

			float effectiveYRadius;
			if (seenP && numPoints[1] >= 2)
			{
				effectiveYRadius = radius - 0.1;
				if (numPoints[0] % 2 == 0)
				{
					effectiveYRadius *= sqrtf(1.0 - 1.0/(float)((numPoints[0] - 1) * (numPoints[0] - 1)));
				}
				spacings[1] = (2 * effectiveYRadius)/(numPoints[1] - 1);
			}
			else
			{
				effectiveYRadius = floorf((radius - 0.1)/spacings[1]) * spacings[1];
			}
			yValues[0] = -effectiveYRadius;
			yValues[1] =  effectiveYRadius + 0.1;
		}
		else
		{
			reply.copy("M577 radius must be positive unless X and Y are specified");
			return GCodeResult::error;
		}
	}

	const bool ok = defaultGrid.Set(xValues, yValues, radius, spacings);
	reprap.MoveUpdated();
	if (ok)
	{
		return GCodeResult::ok;
	}

	const float xRange = (seenX) ? xValues[1] - xValues[0] : 2 * radius;
	const float yRange = (seenX) ? yValues[1] - yValues[0] : 2 * radius;
	reply.copy("bad grid definition: ");
	defaultGrid.PrintError(xRange, yRange, reply);
	return GCodeResult::error;
}


#if HAS_MASS_STORAGE || HAS_LINUX_INTERFACE

// Handle M37 to simulate a whole file
GCodeResult GCodes::SimulateFile(GCodeBuffer& gb, const StringRef &reply, const StringRef& file, bool updateFile)
{
	if (reprap.GetPrintMonitor().IsPrinting())
	{
		reply.copy("cannot simulate while a file is being printed");
		return GCodeResult::error;
	}

# if HAS_MASS_STORAGE
	if (
#  if HAS_LINUX_INTERFACE
		reprap.UsingLinuxInterface() ||
#  endif
		QueueFileToPrint(file.c_str(), reply))
# endif
	{
		if (simulationMode == 0)
		{
			axesHomedBeforeSimulation = axesHomed;
			axesHomed = AxesBitmap::MakeLowestNBits(numVisibleAxes);	// pretend all axes are homed
			SavePosition(simulationRestorePoint, gb);
			simulationRestorePoint.feedRate = gb.MachineState().feedRate;
		}
		simulationTime = 0.0;
		exitSimulationWhenFileComplete = true;
#if HAS_LINUX_INTERFACE
		updateFileWhenSimulationComplete = updateFile && !reprap.UsingLinuxInterface();
#else
		updateFileWhenSimulationComplete = updateFile;
#endif
		simulationMode = 1;
		reprap.GetMove().Simulate(simulationMode);
		reprap.GetPrintMonitor().StartingPrint(file.c_str());
#if HAS_LINUX_INTERFACE
		if (!reprap.UsingLinuxInterface())
#endif
		{
			// If using a SBC, this is already called when the print file info is set
			StartPrinting(true);
		}
		reply.printf("Simulating print of file %s", file.c_str());
		return GCodeResult::ok;
	}

	return GCodeResult::error;
}

// Handle M37 to change the simulation mode
GCodeResult GCodes::ChangeSimulationMode(GCodeBuffer& gb, const StringRef &reply, uint32_t newSimulationMode)
{
	if (newSimulationMode != simulationMode)
	{
		if (!LockMovementAndWaitForStandstill(gb))
		{
			return GCodeResult::notFinished;
		}

		if (newSimulationMode == 0)
		{
			EndSimulation(&gb);
		}
		else
		{
			if (simulationMode == 0)
			{
				// Starting a new simulation, so save the current position
				axesHomedBeforeSimulation = axesHomed;
				axesHomed = AxesBitmap::MakeLowestNBits(numVisibleAxes);	// pretend all axes are homed
				SavePosition(simulationRestorePoint, gb);
			}
			simulationTime = 0.0;
		}
		exitSimulationWhenFileComplete = updateFileWhenSimulationComplete = false;
		simulationMode = (uint8_t)newSimulationMode;
		reprap.GetMove().Simulate(simulationMode);
	}
	return GCodeResult::ok;
}

#endif

// Handle M577
GCodeResult GCodes::WaitForPin(GCodeBuffer& gb, const StringRef &reply)
{
	AxesBitmap endstopsToWaitFor;
	for (size_t axis = 0; axis < numTotalAxes; ++axis)
	{
		if (gb.Seen(axisLetters[axis]))
		{
			endstopsToWaitFor.SetBit(axis);
		}
	}

	InputPortsBitmap portsToWaitFor;
	if (gb.Seen('P'))
	{
		uint32_t inputNumbers[MaxGpInPorts];
		size_t numValues = MaxGpInPorts;
		gb.GetUnsignedArray(inputNumbers, numValues, false);
		portsToWaitFor = InputPortsBitmap::MakeFromArray(inputNumbers, numValues);
	}

	const bool activeHigh = (!gb.Seen('S') || gb.GetUIValue() >= 1);
	Platform& platform = reprap.GetPlatform();
	const bool ok = endstopsToWaitFor.IterateWhile([&platform, activeHigh](unsigned int axis, bool)->bool
								{
									const bool stopped = platform.GetEndstops().Stopped(axis) == EndStopHit::atStop;
									return stopped == activeHigh;
								}
							 )
				&& portsToWaitFor.IterateWhile([&platform, activeHigh](unsigned int port, bool)->bool
								{
									return (port >= MaxGpInPorts || platform.GetGpInPort(port).GetState() == activeHigh);
								}
							 );
	return (ok) ? GCodeResult::ok : GCodeResult::notFinished;
}

// Handle M581
GCodeResult GCodes::ConfigureTrigger(GCodeBuffer& gb, const StringRef& reply)
{
	gb.MustSee('T');
	const unsigned int triggerNumber = gb.GetUIValue();
	if (triggerNumber < MaxTriggers)
	{
		return triggers[triggerNumber].Configure(triggerNumber, gb, reply);
	}

	reply.copy("Trigger number out of range");
	return GCodeResult::error;
}

// Handle M582
GCodeResult GCodes::CheckTrigger(GCodeBuffer& gb, const StringRef& reply)
{
	gb.MustSee('T');
	const unsigned int triggerNumber = gb.GetUIValue();
	if (triggerNumber < MaxTriggers)
	{
		if (triggers[triggerNumber].CheckLevel())
		{
			triggersPending.SetBit(triggerNumber);
		}
		return GCodeResult::ok;
	}

	reply.copy("Trigger number out of range");
	return GCodeResult::error;
}

// Deal with a M584
GCodeResult GCodes::DoDriveMapping(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	if (!LockMovementAndWaitForStandstill(gb))				// we also rely on this to retrieve the current motor positions to moveBuffer
	{
		return GCodeResult::notFinished;
	}

	bool seen = false;
	const size_t originalVisibleAxes = numVisibleAxes;
	const char *lettersToTry = AllowedAxisLetters;
	char c;
	while ((c = *lettersToTry) != 0)
	{
		if (gb.Seen(c))
		{
			// Found an axis letter. Get the drivers to assign to this axis.
			seen = true;
			size_t numValues = MaxDriversPerAxis;
			DriverId drivers[MaxDriversPerAxis];
			gb.GetDriverIdArray(drivers, numValues);

			// Find the drive number allocated to this axis, or allocate a new one if necessary
			size_t drive = 0;
			while (drive < numTotalAxes && axisLetters[drive] != c)
			{
				++drive;
			}
			if (drive < MaxAxes)
			{
				if (drive == numTotalAxes)
				{
					// We are creating a new axis
					axisLetters[drive] = c;								// assign the drive to this drive letter
					++numTotalAxes;
					if (numTotalAxes + numExtruders > MaxAxesPlusExtruders)
					{
						--numExtruders;
					}
					numVisibleAxes = numTotalAxes;						// assume any new axes are visible unless there is a P parameter
					float initialCoords[MaxAxes];
					reprap.GetMove().GetKinematics().GetAssumedInitialPosition(drive + 1, initialCoords);
					moveBuffer.coords[drive] = initialCoords[drive];	// user has defined a new axis, so set its position
					ToolOffsetInverseTransform(moveBuffer.coords, currentUserPosition);
					reprap.MoveUpdated();
				}
				platform.SetAxisDriversConfig(drive, numValues, drivers);
			}
		}
		++lettersToTry;
	}

	if (gb.Seen(extrudeLetter))
	{
		seen = true;
		size_t numValues = MaxExtruders;
		DriverId drivers[MaxExtruders];
		gb.GetDriverIdArray(drivers, numValues);
		numExtruders = numValues;
		for (size_t i = 0; i < numValues; ++i)
		{
			platform.SetExtruderDriver(i, drivers[i]);
		}
	}

	if (gb.Seen('P'))
	{
		seen = true;
		const unsigned int nva = gb.GetUIValue();
		if (nva >= MinAxes && nva <= numTotalAxes)
		{
			numVisibleAxes = nva;
		}
		else
		{
			reply.copy("Invalid number of visible axes");
			return GCodeResult::error;
		}
	}

	if (seen)
	{
		reprap.MoveUpdated();
		if (numVisibleAxes > originalVisibleAxes)
		{
			// In the DDA ring, the axis positions for invisible non-moving axes are not always copied over from previous moves.
			// So if we have more visible axes than before, then we need to update their positions to get them in sync.
			ToolOffsetTransform(currentUserPosition, moveBuffer.coords);	// ensure that the position of any new axes are updated in moveBuffer
			reprap.GetMove().SetNewPosition(moveBuffer.coords, true);		// tell the Move system where the axes are
		}
	}
	else
	{
		reply.copy("Driver assignments:");
		bool printed = false;
		for (size_t drive = 0; drive < numTotalAxes; ++ drive)
		{
			reply.cat(' ');
			const AxisDriversConfig& axisConfig = platform.GetAxisDriversConfig(drive);
			char c = axisLetters[drive];
			for (size_t i = 0; i < axisConfig.numDrivers; ++i)
			{
				printed = true;
				const DriverId id = axisConfig.driverNumbers[i];
				reply.catf("%c" PRIdriverId, c, DRIVER_ID_PRINT_ARGS(id));
				c = ':';
			}
		}
		if (numExtruders != 0)
		{
			reply.cat(' ');
			char c = extrudeLetter;
			for (size_t extruder = 0; extruder < numExtruders; ++extruder)
			{
				const DriverId id = platform.GetExtruderDriver(extruder);
				reply.catf("%c" PRIdriverId, c, DRIVER_ID_PRINT_ARGS(id));
				c = ':';
			}
		}
		if (!printed)
		{
			reply.cat(" none");
		}
		reply.catf(", %u axes visible", numVisibleAxes);
	}

	return GCodeResult::ok;
}
// Handle G38.[2-5]
GCodeResult GCodes::StraightProbe(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	const int8_t fraction = gb.GetCommandFraction();
	if (fraction < 2 || fraction > 5) {
		return GCodeResult::warningNotSupported;
	}
	/*
	 * It is an error if:
	 * # the current point is the same as the programmed point.
	 * # no axis word is used
	 * # the feed rate is zero
	 * # the probe is already in the target state
	 */

	StraightProbeSettings& sps = reprap.GetMove().GetStraightProbeSettings();
	sps.Reset();


	switch (fraction)
	{
	case 2:
		sps.SetStraightProbeType(StraightProbeType::towardsWorkpieceErrorOnFailure);
		break;

	case 3:
		sps.SetStraightProbeType(StraightProbeType::towardsWorkpiece);
		break;

	case 4:
		sps.SetStraightProbeType(StraightProbeType::awayFromWorkpieceErrorOnFailure);
		break;

	case 5:
		sps.SetStraightProbeType(StraightProbeType::awayFromWorkpiece);
		break;
	}

	// Get the target coordinates and check if we would move at all
	float target[MaxAxes];
	ToolOffsetTransform(currentUserPosition, target);
	bool seen = false;
	bool doesMove = false;
	for (size_t axis = 0; axis < numVisibleAxes; axis++)
	{
		if (gb.Seen(axisLetters[axis]))
		{
			seen = true;
			const float axisTarget = gb.GetFValue();
			if (axisTarget != target[axis])
			{
				doesMove = true;
			}
			target[axis] = axisTarget;
			sps.AddMovingAxis(axis);
		}
	}

	// No axis letters seen
	if (!seen)
	{
		// Signal error for G38.2 and G38.4
		if (sps.SignalError())
		{
			reply.copy("No axis specified.");
			return GCodeResult::error;
		}
		return GCodeResult::ok;
	}

	// At least one axis seen but it would not result in movement
	else if (!doesMove)
	{
		// Signal error for G38.2 and G38.4
		if (sps.SignalError())
		{
			reply.copy("Target equals current position.");
			return GCodeResult::error;
		}
		return GCodeResult::ok;
	}
	sps.SetTarget(target);

	// See whether we are using a user-defined Z probe or just current one
	const size_t probeToUse = gb.Seen('P') ? gb.GetUIValue() : 0;

	// Check if this probe exists to not run into a nullptr dereference later
	if (platform.GetEndstops().GetZProbe(probeToUse).IsNull())
	{
		reply.catf("Invalid probe number: %d", probeToUse);
		return GCodeResult::error;
	}
	sps.SetZProbeToUse(probeToUse);

	gb.SetState(GCodeState::straightProbe0);
	return GCodeResult::ok;
}

// Deal with a M585
GCodeResult GCodes::ProbeTool(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	if (reprap.GetCurrentTool() == nullptr)
	{
		reply.copy("No tool selected!");
		return GCodeResult::error;
	}

	if (!LockMovementAndWaitForStandstill(gb))
	{
		return GCodeResult::notFinished;
	}

	// See whether we are using a Z probe or just endstops
	unsigned int probeNumberToUse;
	const bool useProbe = gb.Seen('P');
	if (useProbe)
	{
		probeNumberToUse = gb.GetUIValue();
		if (platform.GetEndstops().GetZProbe(probeNumberToUse).IsNull())
		{
			reply.copy("Invalid probe number");
			return GCodeResult::error;
		}
	}

	for (size_t axis = 0; axis < numTotalAxes; axis++)
	{
		if (gb.Seen(axisLetters[axis]))
		{
			// Save the current axis coordinates
			SavePosition(toolChangeRestorePoint, gb);

			// Prepare another move similar to G1 .. S3
			moveBuffer.SetDefaults(numVisibleAxes);
			moveBuffer.moveType = 3;
			moveBuffer.canPauseAfter = false;

			// Decide which way and how far to go
			if (gb.Seen('R'))
			{
				// Use relative probing radius if the R parameter is present
				moveBuffer.coords[axis] += gb.GetFValue();
			}
			else
			{
				// Move to axis minimum if S1 is passed or to the axis maximum otherwise
				moveBuffer.coords[axis] = (gb.Seen('S') && gb.GetIValue() > 0) ? platform.AxisMinimum(axis) : platform.AxisMaximum(axis);
			}

			// Deal with feed rate
			if (gb.Seen(feedrateLetter))
			{
				const float rate = gb.GetDistance();
				gb.MachineState().feedRate = rate * SecondsToMinutes;	// don't apply the speed factor to homing and other special moves
			}
			moveBuffer.feedRate = gb.MachineState().feedRate;

			const bool probeOk = (useProbe)
									? platform.GetEndstops().EnableZProbe(probeNumberToUse)
										: platform.GetEndstops().EnableAxisEndstops(AxesBitmap::MakeFromBits(axis), false);
			if (!probeOk)
			{
				reply.copy("Failed to prime endstop or probe");
				AbortPrint(gb);
				return GCodeResult::error;
			}

			moveBuffer.checkEndstops = true;

			// Kick off new movement
			NewMoveAvailable(1);
			gb.SetState(GCodeState::probingToolOffset);
		}
	}

	return GCodeResult::ok;
}

GCodeResult GCodes::FindCenterOfCavity(GCodeBuffer& gb, const StringRef& reply, const bool towardsMin) THROWS(GCodeException)
{
	if (reprap.GetCurrentTool() == nullptr)
	{
		reply.copy("No tool selected!");
		return GCodeResult::error;
	}

	if (!LockMovementAndWaitForStandstill(gb))
	{
		return GCodeResult::notFinished;
	}

	// See whether we are using a Z probe or just endstops
	unsigned int probeNumberToUse;
	const bool useProbe = gb.Seen('P');
	if (useProbe)
	{
		probeNumberToUse = gb.GetUIValue();
		if (platform.GetEndstops().GetZProbe(probeNumberToUse).IsNull())
		{
			reply.copy("Invalid probe number");
			return GCodeResult::error;
		}
	}

	for (size_t axis = 0; axis < numVisibleAxes; axis++)
	{
		if (gb.Seen(axisLetters[axis]))
		{

			// Prepare a move similar to G1 .. S3
			moveBuffer.SetDefaults(numVisibleAxes);
			moveBuffer.moveType = 3;
			moveBuffer.canPauseAfter = false;

			moveBuffer.coords[axis] = towardsMin ? platform.AxisMinimum(axis) : platform.AxisMaximum(axis);

			// Deal with feed rate
			gb.MustSee(feedrateLetter);
			const float rate = gb.GetDistance();
			moveBuffer.feedRate = gb.MachineState().feedRate = rate * SecondsToMinutes;		// don't apply the speed factor to homing and other special moves

			const bool probeOk = (useProbe)
									? platform.GetEndstops().EnableZProbe(probeNumberToUse)
										: platform.GetEndstops().EnableAxisEndstops(AxesBitmap::MakeFromBits(axis), false);
			if (!probeOk)
			{
				reply.copy("Failed to prime endstop or probe");
				AbortPrint(gb);
				return GCodeResult::error;
			}

			moveBuffer.checkEndstops = true;

			// Kick off new movement
			NewMoveAvailable(1);

			if (towardsMin)
			{
				gb.SetState(GCodeState::findCenterOfCavityMin);
			}
			else
			{
				gb.SetState(GCodeState::findCenterOfCavityMax);
			}
			// Only do one axis at a time
			break;
		}
	}
	return GCodeResult::ok;
}

// Deal with a M905
GCodeResult GCodes::SetDateTime(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	tm timeInfo;
	(void)platform.GetDateTime(timeInfo);
	bool seen = false;

	if (gb.Seen('P'))
	{
		seen = true;

		// Set date
		String<12> dateString;
		gb.GetPossiblyQuotedString(dateString.GetRef());
		if (SafeStrptime(dateString.c_str(), "%Y-%m-%d", &timeInfo) == nullptr)
		{
			reply.copy("Invalid date format");
			return GCodeResult::error;
		}
	}

	if (gb.Seen('S'))
	{
		seen = true;

		// Set time
		String<12> timeString;
		gb.GetPossiblyQuotedString(timeString.GetRef());
		if (SafeStrptime(timeString.c_str(), "%H:%M:%S", &timeInfo) == nullptr)
		{
			reply.copy("Invalid time format");
			return GCodeResult::error;
		}
	}

	if (seen)
	{
		platform.SetDateTime(mktime(&timeInfo));
	}
	else
	{
		// Report current date and time
		if (platform.IsDateTimeSet())
		{
			reply.printf("Current date and time: %04u-%02u-%02u %02u:%02u:%02u",
					timeInfo.tm_year + 1900, timeInfo.tm_mon + 1, timeInfo.tm_mday,
					timeInfo.tm_hour, timeInfo.tm_min, timeInfo.tm_sec);
		}
		else
		{
			reply.copy("Clock has not been set");
		}
	}

	return GCodeResult::ok;
}

// Handle M997
GCodeResult GCodes::UpdateFirmware(GCodeBuffer& gb, const StringRef &reply)
{
	if (!LockMovementAndWaitForStandstill(gb))
	{
		return GCodeResult::notFinished;
	}

#if SUPPORT_CAN_EXPANSION
	if (gb.Seen('B'))
	{
		const uint32_t boardNumber = gb.GetUIValue();
		if (boardNumber != CanId::MasterAddress)
		{
			return reprap.GetExpansion().UpdateRemoteFirmware(boardNumber, gb, reply);
		}
	}
#endif

	reprap.GetHeat().SwitchOffAll(true);				// turn all heaters off because the main loop may get suspended
	DisableDrives();									// all motors off

	if (firmwareUpdateModuleMap == 0)					// have we worked out which modules to update?
	{
		// Find out which modules we have been asked to update
		if (gb.Seen('S'))
		{
			uint32_t modulesToUpdate[3];
			size_t numUpdateModules = ARRAY_SIZE(modulesToUpdate);
			gb.GetUnsignedArray(modulesToUpdate, numUpdateModules, false);
			for (size_t i = 0; i < numUpdateModules; ++i)
			{
				uint32_t t = modulesToUpdate[i];
				if (t >= NumFirmwareUpdateModules)
				{
					reply.printf("Invalid module number '%" PRIu32 "'\n", t);
					firmwareUpdateModuleMap = 0;
					return GCodeResult::error;
					break;
				}
				firmwareUpdateModuleMap |= (1u << t);
			}
		}
		else
		{
			firmwareUpdateModuleMap = (1u << 0);		// no modules specified, so update module 0 to match old behaviour
		}

		if (firmwareUpdateModuleMap == 0)
		{
			return GCodeResult::ok;						// nothing to update
		}

		// Check prerequisites of all modules to be updated, if any are not met then don't update any of them
#if HAS_WIFI_NETWORKING
		if (!FirmwareUpdater::CheckFirmwareUpdatePrerequisites(firmwareUpdateModuleMap, reply))
		{
			firmwareUpdateModuleMap = 0;
			return GCodeResult::error;
		}
#endif
		if ((firmwareUpdateModuleMap & 1) != 0 && !reprap.CheckFirmwareUpdatePrerequisites(reply))
		{
			firmwareUpdateModuleMap = 0;
			return GCodeResult::error;
		}
	}

	// If we get here then we have the module map, and all prerequisites are satisfied
	isFlashing = true;										// this tells the web interface and PanelDue that we are about to flash firmware
	if (!gb.DoDwellTime(1000))								// wait a second so all HTTP clients and PanelDue are notified
	{
		return GCodeResult::notFinished;
	}

	gb.SetState(GCodeState::flashing1);
	return GCodeResult::ok;
}

// Handle M260 - send and possibly receive via I2C
GCodeResult GCodes::SendI2c(GCodeBuffer& gb, const StringRef &reply)
{
#if defined(I2C_IFACE)
	if (gb.Seen('A'))
	{
		const uint32_t address = gb.GetUIValue();
		uint32_t numToReceive = 0;
		bool seenR;
		gb.TryGetUIValue('R', numToReceive, seenR);
		int32_t values[MaxI2cBytes];
		size_t numToSend;
		if (gb.Seen('B'))
		{
			numToSend = MaxI2cBytes;
			gb.GetIntArray(values, numToSend, false);		//TODO allow hex values
		}
		else
		{
			numToSend = 0;
		}

		if (numToSend + numToReceive != 0)
		{
			if (numToSend + numToReceive > MaxI2cBytes)
			{
				numToReceive = MaxI2cBytes - numToSend;
			}
			uint8_t bValues[MaxI2cBytes];
			for (size_t i = 0; i < numToSend; ++i)
			{
				bValues[i] = (uint8_t)values[i];
			}

			I2C::Init();
			const size_t bytesTransferred = I2C::Transfer(address, bValues, numToSend, numToReceive);

			if (bytesTransferred < numToSend)
			{
				reply.copy("I2C transmission error");
				return GCodeResult::error;
			}
			else if (numToReceive != 0)
			{
				reply.copy("Received");
				if (bytesTransferred == numToSend)
				{
					reply.cat(" nothing");
				}
				else
				{
					for (size_t i = numToSend; i < bytesTransferred; ++i)
					{
						reply.catf(" %02x", bValues[i]);
					}
				}
			}
			return (bytesTransferred == numToSend + numToReceive) ? GCodeResult::ok : GCodeResult::error;
		}
	}

	return GCodeResult::badOrMissingParameter;
#else
	reply.copy("I2C not available");
	return GCodeResult::error;
#endif
}

// Handle M261
GCodeResult GCodes::ReceiveI2c(GCodeBuffer& gb, const StringRef &reply)
{
#if defined(I2C_IFACE)
	if (gb.Seen('A'))
	{
		const uint32_t address = gb.GetUIValue();
		if (gb.Seen('B'))
		{
			const uint32_t numBytes = gb.GetUIValue();
			if (numBytes > 0 && numBytes <= MaxI2cBytes)
			{
				I2C::Init();

				uint8_t bValues[MaxI2cBytes];
				const size_t bytesRead = I2C::Transfer(address, bValues, 0, numBytes);

				reply.copy("Received");
				if (bytesRead == 0)
				{
					reply.cat(" nothing");
				}
				else
				{
					for (size_t i = 0; i < bytesRead; ++i)
					{
						reply.catf(" %02x", bValues[i]);
					}
				}

				return (bytesRead == numBytes) ? GCodeResult::ok : GCodeResult::error;
			}
		}
	}

	return GCodeResult::badOrMissingParameter;
#else
	reply.copy("I2C not available");
	return GCodeResult::error;
#endif
}

// Deal with M569
GCodeResult GCodes::ConfigureDriver(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	gb.MustSee('P');
	const DriverId id = gb.GetDriverId();
#if SUPPORT_CAN_EXPANSION
	if (id.boardAddress != CanId::MasterAddress)
	{
		return CanInterface::ConfigureRemoteDriver(id, gb, reply);
	}
#endif
	const uint8_t drive = id.localDriver;
	if (drive < NumDirectDrivers)
	{
		bool seen = false;
		if (gb.Seen('S'))
		{
			if (!LockMovementAndWaitForStandstill(gb))
			{
				return GCodeResult::notFinished;
			}
			seen = true;
			platform.SetDirectionValue(drive, gb.GetIValue() != 0);
		}
		if (gb.Seen('R'))
		{
			if (!LockMovementAndWaitForStandstill(gb))
			{
				return GCodeResult::notFinished;
			}
			seen = true;
			platform.SetEnableValue(drive, (int8_t)gb.GetIValue());
		}
		if (gb.Seen('T'))
		{
			seen = true;
			float timings[4];
			size_t numTimings = ARRAY_SIZE(timings);
			gb.GetFloatArray(timings, numTimings, true);
			if (numTimings != ARRAY_SIZE(timings))
			{
				reply.copy("bad timing parameter");
				return GCodeResult::error;
			}
			platform.SetDriverStepTiming(drive, timings);
		}

#if HAS_SMART_DRIVERS
		{
			uint32_t val;
			if (gb.TryGetUIValue('D', val, seen))	// set driver mode
			{
				if (!SmartDrivers::SetDriverMode(drive, val))
				{
					reply.printf("Driver %u does not support mode '%s'", drive, TranslateDriverMode(val));
					return GCodeResult::error;
				}
			}

			if (gb.TryGetUIValue('C', val, seen))		// set chopper control register
			{
				if (!SmartDrivers::SetRegister(drive, SmartDriverRegister::chopperControl, val))
				{
					reply.printf("Bad ccr for driver %u", drive);
					return GCodeResult::error;
				}
			}

			if (gb.TryGetUIValue('F', val, seen))		// set off time
			{
				if (!SmartDrivers::SetRegister(drive, SmartDriverRegister::toff, val))
				{
					reply.printf("Bad off time for driver %u", drive);
					return GCodeResult::error;
				}
			}

			if (gb.TryGetUIValue('B', val, seen))		// set blanking time
			{
				if (!SmartDrivers::SetRegister(drive, SmartDriverRegister::tblank, val))
				{
					reply.printf("Bad blanking time for driver %u", drive);
					return GCodeResult::error;
				}
			}

			if (gb.TryGetUIValue('V', val, seen))		// set microstep interval for changing from stealthChop to spreadCycle
			{
				if (!SmartDrivers::SetRegister(drive, SmartDriverRegister::tpwmthrs, val))
				{
					reply.printf("Bad mode change microstep interval for driver %u", drive);
					return GCodeResult::error;
				}
			}

#if SUPPORT_TMC51xx
			if (gb.TryGetUIValue('H', val, seen))		// set coolStep threshold
			{
				if (!SmartDrivers::SetRegister(drive, SmartDriverRegister::thigh, val))
				{
					reply.printf("Bad high speed microstep interval for driver %u", drive);
					return GCodeResult::error;
				}
			}
#endif
		}

		if (gb.Seen('Y'))								// set spread cycle hysteresis
		{
			seen = true;
			uint32_t hvalues[3];
			size_t numHvalues = 3;
			gb.GetUnsignedArray(hvalues, numHvalues, false);
			if (numHvalues == 2 || numHvalues == 3)
			{
				// There is a constraint on the sum of HSTRT and HEND, so set HSTART then HEND then HSTART again because one may go up and the other down
				(void)SmartDrivers::SetRegister(drive, SmartDriverRegister::hstart, hvalues[0]);
				bool ok = SmartDrivers::SetRegister(drive, SmartDriverRegister::hend, hvalues[1]);
				if (ok)
				{
					ok = SmartDrivers::SetRegister(drive, SmartDriverRegister::hstart, hvalues[0]);
				}
				if (ok && numHvalues == 3)
				{
					ok = SmartDrivers::SetRegister(drive, SmartDriverRegister::hdec, hvalues[2]);
				}
				if (!ok)
				{
					reply.printf("Bad hysteresis setting for driver %u", drive);
					return GCodeResult::error;
				}
			}
			else
			{
				reply.copy("Expected 2 or 3 Y values");
				return GCodeResult::error;
			}
		}
#endif
		if (!seen)
		{
			reply.printf("Drive %u runs %s, active %s enable, step timing ",
							drive,
							(platform.GetDirectionValue(drive)) ? "forwards" : "in reverse",
							(platform.GetEnableValue(drive) > 0) ? "high" : "low");
			float timings[4];
			const bool isSlowDriver = platform.GetDriverStepTiming(drive, timings);
			if (isSlowDriver)
			{
				reply.catf("%.1f:%.1f:%.1f:%.1fus", (double)timings[0], (double)timings[1], (double)timings[2], (double)timings[3]);
			}
			else
			{
				reply.cat("fast");
			}
#if HAS_SMART_DRIVERS
			if (drive < platform.GetNumSmartDrivers())
			{
				reply.catf(", mode %s, ccr 0x%05" PRIx32 ", toff %" PRIu32 ", tblank %" PRIu32 ", hstart/hend/hdec %" PRIu32 "/%" PRIu32 "/%" PRIu32,
						TranslateDriverMode(SmartDrivers::GetDriverMode(drive)),
						SmartDrivers::GetRegister(drive, SmartDriverRegister::chopperControl),
						SmartDrivers::GetRegister(drive, SmartDriverRegister::toff),
						SmartDrivers::GetRegister(drive, SmartDriverRegister::tblank),
						SmartDrivers::GetRegister(drive, SmartDriverRegister::hstart),
						SmartDrivers::GetRegister(drive, SmartDriverRegister::hend),
						SmartDrivers::GetRegister(drive, SmartDriverRegister::hdec)
					);

#if SUPPORT_TMC2660
				{
					const uint32_t mstepPos = SmartDrivers::GetRegister(drive, SmartDriverRegister::mstepPos);
					if (mstepPos < 1024)
					{
						reply.catf(", pos %" PRIu32, mstepPos);
					}
					else
					{
						reply.cat(", pos unknown");
					}
				}
#elif SUPPORT_TMC22xx || SUPPORT_TMC51xx
				{
					const uint32_t tpwmthrs = SmartDrivers::GetRegister(drive, SmartDriverRegister::tpwmthrs);
					const uint32_t mstepPos = SmartDrivers::GetRegister(drive, SmartDriverRegister::mstepPos);
					const uint32_t axis = SmartDrivers::GetAxisNumber(drive);
					bool bdummy;
					const float mmPerSec = (12000000.0 * SmartDrivers::GetMicrostepping(drive, bdummy))/(256 * tpwmthrs * platform.DriveStepsPerUnit(axis));
					reply.catf(", pos %" PRIu32", tpwmthrs %" PRIu32 " (%.1f mm/sec)", mstepPos, tpwmthrs, (double)mmPerSec);
				}
#endif

#if SUPPORT_TMC51xx
				{
					const uint32_t thigh = SmartDrivers::GetRegister(drive, SmartDriverRegister::thigh);
					const uint32_t axis = SmartDrivers::GetAxisNumber(drive);
					bool bdummy;
					const float mmPerSec = (12000000.0 * SmartDrivers::GetMicrostepping(drive, bdummy))/(256 * thigh * platform.DriveStepsPerUnit(axis));
					reply.catf(", thigh %" PRIu32 " (%.1f mm/sec)", thigh, (double)mmPerSec);
				}
#endif
			}
#endif
		}
	}
	return GCodeResult::ok;
}

// Change a live extrusion factor
void GCodes::ChangeExtrusionFactor(unsigned int extruder, float factor) noexcept
{
	if (segmentsLeft != 0 && moveBuffer.applyM220M221)
	{
		moveBuffer.coords[ExtruderToLogicalDrive(extruder)] *= factor/extrusionFactors[extruder];	// last move not gone, so update it
	}
	extrusionFactors[extruder] = factor;
	reprap.MoveUpdated();
}

// Deploy the Z probe unless it has already been deployed explicitly
// The required next state must be set up (e.g. by gb.SetState()) before calling this
void GCodes::DeployZProbe(GCodeBuffer& gb, int code) noexcept
{
	auto zp = reprap.GetPlatform().GetEndstops().GetZProbe(currentZProbeNumber);
	if (zp.IsNotNull() && zp->GetProbeType() != ZProbeType::none && !zp->IsDeployedByUser())
	{
		String<StringLength20> fileName;
		fileName.printf(DEPLOYPROBE "%u.g", currentZProbeNumber);
		if (!DoFileMacro(gb, fileName.c_str(), false, code))
		{
			DoFileMacro(gb, DEPLOYPROBE ".g", false, code);
		}
	}
}

// Retract the Z probe unless it was deployed explicitly (in which case, wait for the user to retract it explicitly)
// The required next state must be set up (e.g. by gb.SetState()) before calling this
void GCodes::RetractZProbe(GCodeBuffer& gb, int code) noexcept
{
	auto zp = reprap.GetPlatform().GetEndstops().GetZProbe(currentZProbeNumber);
	if (zp.IsNotNull() && zp->GetProbeType() != ZProbeType::none && !zp->IsDeployedByUser())
	{
		String<StringLength20> fileName;
		fileName.printf(RETRACTPROBE "%u.g", currentZProbeNumber);
		if (!DoFileMacro(gb, fileName.c_str(), false, code))
		{
			DoFileMacro(gb, RETRACTPROBE ".g", false, code);
		}
	}
}

// Process a whole-line comment returning true if completed
bool GCodes::ProcessWholeLineComment(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	static const char * const StartStrings[] =
	{
		"printing object",			// slic3r
		"MESH",						// Cura
		"process",					// S3D
		"stop printing object",		// slic3r
		"layer",					// S3D "; layer 1, z=0.200"
		"LAYER",					// Ideamaker, Cura (followed by layer number starting at zero)
		"BEGIN_LAYER_OBJECT z=",	// KISSlicer (followed by Z height)
		"HEIGHT"					// Ideamaker
	};

	const char * text = gb.GetCompleteParameters();
	while (*text == ' ')
	{
		++text;
	}

	for (size_t i = 0; i < ARRAY_SIZE(StartStrings); ++i)
	{
		if (StringStartsWith(text, StartStrings[i]))
		{
			text += strlen(StartStrings[i]);
			if (*text == ' ' || *text == ':')			// need this test to avoid recognising "processName" as "process"
			{
				do
				{
					++text;
				}
				while (*text == ' ' || *text == ':');

				switch (i)
				{
				case 0:		// printing object (slic3r)
				case 1:		// MESH (Cura)
				case 2:		// process (S3D)
#if TRACK_OBJECT_NAMES
					buildObjects.StartObject(gb, text);
#endif
					break;

				case 3:		// stop printing object
#if TRACK_OBJECT_NAMES
					buildObjects.StopObject(gb);
#endif
					break;

				case 4:		// layer (counting from 1)
				case 5:		// layer (counting from 0)
					{
						const char *endptr;
						const uint32_t layer = StrToU32(text, &endptr);
						if (endptr != text)
						{
							reprap.GetPrintMonitor().SetLayerNumber((i == 5) ? layer + 1 : layer);
						}
						text = endptr;
						if (!StringStartsWith(text, ", z = "))		// S3D gives us the height too
						{
							break;
						}
						text += 6;			// skip ", z = "
					}
					// no break

				case 6:		// new layer, but we are given the Z height, not the layer number
				case 7:
					{
						const char *endptr;
						const float layerZ = SafeStrtof(text, &endptr);
						if (endptr != text)
						{
							reprap.GetPrintMonitor().SetLayerZ(layerZ);
						}
					}
					break;
				}
				break;
			}
		}
	}
	return true;
}

// End
