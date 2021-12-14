/*
 * GCodes3.cpp
 *
 *  Created on: 5 Dec 2017
 *      Author: David
 *  This file contains functions that are called form file GCodes2.cpp to execute various G and M codes.
 */

#include "GCodes.h"

#include "GCodeBuffer/GCodeBuffer.h"
#include <Heating/Heat.h>
#include <Movement/Move.h>
#include <Platform/RepRap.h>
#include <Platform/Event.h>
#include <Tools/Tool.h>
#include <Endstops/ZProbe.h>
#include <PrintMonitor/PrintMonitor.h>
#include <Platform/Tasks.h>
#include <Hardware/I2C.h>

#if HAS_WIFI_NETWORKING || HAS_AUX_DEVICES || HAS_MASS_STORAGE || HAS_SBC_INTERFACE
# include <Comms/FirmwareUpdater.h>
#endif

#if SUPPORT_TMC2660
# include <Movement/StepperDrivers/TMC2660.h>
#endif
#if SUPPORT_TMC22xx
# include <Movement/StepperDrivers/TMC22xx.h>
#endif
#if SUPPORT_TMC51xx
# include <Movement/StepperDrivers/TMC51xx.h>
#endif

#if SUPPORT_CAN_EXPANSION
# include <CAN/CanInterface.h>
# include <CAN/ExpansionManager.h>
# include <ClosedLoop/ClosedLoop.h>
#endif

#ifdef I2C_IFACE
# include <Wire.h>
#endif

#ifdef DUET3_ATE
# include <Duet3Ate.h>
#endif

#include <cctype>

// Deal with G60
GCodeResult GCodes::SavePosition(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	uint32_t sParam = 0;
	bool dummySeen;
	gb.TryGetLimitedUIValue('S', sParam, dummySeen, NumRestorePoints);
	SavePosition(numberedRestorePoints[sParam], gb);
	return GCodeResult::ok;
}

// This handles G92. Return true if completed, false if it needs to be called again.
GCodeResult GCodes::SetPositions(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
#if SUPPORT_COORDINATE_ROTATION
	if (g68Angle != 0.0 && gb.SeenAny("XY") && gb.DoingCoordinateRotation())
	{
		reply.copy("not supported when coordinate rotation is in effect");
		return GCodeResult::error;
	}
#endif

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
			moveState.currentUserPosition[axis] = gb.ConvertDistance(axisValue);
		}
	}

	// Handle any E parameter in the G92 command
	if (gb.Seen(extrudeLetter))
	{
		virtualExtruderPosition = gb.GetDistance();
	}

	if (axesIncluded.IsNonEmpty())
	{
		ToolOffsetTransform(moveState.currentUserPosition, moveState.coords);

		if (reprap.GetMove().GetKinematics().LimitPosition(moveState.coords, nullptr, numVisibleAxes, axesIncluded, false, limitAxes) != LimitPositionResult::ok)
		{
			ToolOffsetInverseTransform(moveState.coords, moveState.currentUserPosition);	// make sure the limits are reflected in the user position
		}
		reprap.GetMove().SetNewPosition(moveState.coords, true);
		if (!IsSimulating())
		{
			axesHomed |= reprap.GetMove().GetKinematics().AxesAssumedHomed(axesIncluded);
			axesVirtuallyHomed = axesHomed;
			if (axesIncluded.IsBitSet(Z_AXIS))
			{
				zDatumSetByProbing = false;
			}
			reprap.MoveUpdated();				// because we may have updated axesHomed or zDatumSetByProbing
		}

#if SUPPORT_ROLAND
		if (reprap.GetRoland()->Active())
		{
			for(size_t axis = 0; axis < AXES; axis++)
			{
				if (!reprap.GetRoland()->ProcessG92(moveState[axis], axis))
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
			workplaceCoordinates[moveState.currentCoordinateSystem][axis] = -gb.GetDistance();
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
	// No P parameter or P0 (LinuxCNC extension) means use current coordinate system
	uint32_t cs = 0;
	bool dummySeen;
	gb.TryGetLimitedUIValue('P', cs, dummySeen, NumCoordinateSystems + 1);		// allow 0..NumCoordinateSystems inclusive
	if (cs == 0)
	{
		cs = moveState.currentCoordinateSystem + 1;
	}

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
			workplaceCoordinates[cs - 1][axis] = (compute) ? moveState.currentUserPosition[axis] - coord : coord;
		}
	}

	if (seen)
	{
		reprap.MoveUpdated();
		String<StringLengthLoggedCommand> scratch;
		gb.AppendFullCommand(scratch.GetRef());
		platform.Message(MessageType::LogInfo, scratch.c_str());
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

# if HAS_MASS_STORAGE || HAS_SBC_INTERFACE

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
GCodeResult GCodes::DefineGrid(GCodeBuffer& gb, const StringRef &reply) THROWS(GCodeException)
{
	if (!LockMovement(gb))							// to ensure that probing is not already in progress
	{
		return GCodeResult::notFinished;
	}

	bool seenR = false, seenP = false, seenS = false;
	char axesLetters[2] = { 'X', 'Y'};
	float axis0Values[2];
	float axis1Values[2];
	float spacings[2] = { DefaultGridSpacing, DefaultGridSpacing };

	size_t axesSeenCount = 0;
	for (size_t axis = 0; axis < numVisibleAxes; axis++)
	{
		if (gb.Seen(axisLetters[axis]))
		{
			if (axisLetters[axis] == 'Z')
			{
				reply.copy("Z axis is not allowed for mesh leveling");
				return GCodeResult::error;
			}
			else if (axesSeenCount > 2)
			{
				reply.copy("Mesh leveling expects exactly two axes");
				return GCodeResult::error;
			}
			bool dummy;
			if (gb.TryGetFloatArray(
					axisLetters[axis],
					2,
					(axesSeenCount == 0) ? axis0Values : axis1Values,
					reply,
					dummy,
					false))
			{
				return GCodeResult::error;
			}
			axesLetters[axesSeenCount] = axisLetters[axis];
			++axesSeenCount;
		}
	}
	if (axesSeenCount == 1)
	{
		reply.copy("Specify zero or two axes in M557");
		return GCodeResult::error;
	}
	const bool axesSeen = axesSeenCount > 0;

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

	if (!axesSeen && !seenR && !seenS && !seenP)
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

	if (!axesSeen && !seenR)
	{
		// Must have given just the S or P parameter
		reply.copy("specify at least radius or two axis ranges in M557");
		return GCodeResult::error;
	}

	if (axesSeen)
	{
		// Seen both axes
		if (seenP)
		{
			// In the following, we multiply the spacing by 0.9999 to ensure that when we divide the axis range by the spacing, we get the correct number of points
			// Otherwise, for some values we occasionally get one less point
			if (spacings[0] >= 2 && axis0Values[1] > axis0Values[0])
			{
				spacings[0] = (axis0Values[1] - axis0Values[0])/(numPoints[0] - 1) * 0.9999;
			}
			if (spacings[1] >= 2 && axis1Values[1] > axis1Values[0])
			{
				spacings[1] = (axis1Values[1] - axis1Values[0])/(numPoints[1] - 1) * 0.9999;
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
					effectiveXRadius *= fastSqrtf(1.0 - 1.0/(float)((numPoints[1] - 1) * (numPoints[1] - 1)));
				}
				spacings[0] = (2 * effectiveXRadius)/(numPoints[0] - 1);
			}
			else
			{
				effectiveXRadius = floorf((radius - 0.1)/spacings[0]) * spacings[0];
			}
			axis0Values[0] = -effectiveXRadius;
			axis0Values[1] =  effectiveXRadius + 0.1;

			float effectiveYRadius;
			if (seenP && numPoints[1] >= 2)
			{
				effectiveYRadius = radius - 0.1;
				if (numPoints[0] % 2 == 0)
				{
					effectiveYRadius *= fastSqrtf(1.0 - 1.0/(float)((numPoints[0] - 1) * (numPoints[0] - 1)));
				}
				spacings[1] = (2 * effectiveYRadius)/(numPoints[1] - 1);
			}
			else
			{
				effectiveYRadius = floorf((radius - 0.1)/spacings[1]) * spacings[1];
			}
			axis1Values[0] = -effectiveYRadius;
			axis1Values[1] =  effectiveYRadius + 0.1;
		}
		else
		{
			reply.copy("M577 radius must be positive unless X and Y are specified");
			return GCodeResult::error;
		}
	}

	const bool ok = defaultGrid.Set(axesLetters, axis0Values, axis1Values, radius, spacings);
	reprap.MoveUpdated();
	if (ok)
	{
		return GCodeResult::ok;
	}

	const float axis1Range = axesSeen ? axis0Values[1] - axis0Values[0] : 2 * radius;
	const float axis2Range = axesSeen ? axis1Values[1] - axis1Values[0] : 2 * radius;
	reply.copy("bad grid definition: ");
	defaultGrid.PrintError(axis1Range, axis2Range, reply);
	return GCodeResult::error;
}


#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE || HAS_EMBEDDED_FILES

// Handle M37 to simulate a whole file
GCodeResult GCodes::SimulateFile(GCodeBuffer& gb, const StringRef &reply, const StringRef& file, bool updateFile)
{
	if (reprap.GetPrintMonitor().IsPrinting())
	{
		reply.copy("cannot simulate while a file is being printed");
		return GCodeResult::error;
	}

# if HAS_MASS_STORAGE || HAS_EMBEDDED_FILES
	if (
#  if HAS_SBC_INTERFACE
		reprap.UsingSbcInterface() ||
#  endif
		QueueFileToPrint(file.c_str(), reply))
# endif
	{
		if (!IsSimulating())
		{
			axesVirtuallyHomed = AxesBitmap::MakeLowestNBits(numVisibleAxes);	// pretend all axes are homed
			SavePosition(simulationRestorePoint, gb);
			simulationRestorePoint.feedRate = gb.LatestMachineState().feedRate;
		}
		simulationTime = 0.0;
		exitSimulationWhenFileComplete = true;
# if HAS_SBC_INTERFACE
		updateFileWhenSimulationComplete = updateFile && !reprap.UsingSbcInterface();
# else
		updateFileWhenSimulationComplete = updateFile;
# endif
		simulationMode = SimulationMode::normal;
		reprap.GetMove().Simulate(simulationMode);
		reprap.GetPrintMonitor().StartingPrint(file.c_str());
		StartPrinting(true);
		reply.printf("Simulating print of file %s", file.c_str());
		return GCodeResult::ok;
	}

	return GCodeResult::error;
}

// Handle M37 to change the simulation mode
GCodeResult GCodes::ChangeSimulationMode(GCodeBuffer& gb, const StringRef &reply, SimulationMode newSimMode) THROWS(GCodeException)
{
	if (newSimMode != simulationMode)
	{
		if (!LockMovementAndWaitForStandstill(gb))
		{
			return GCodeResult::notFinished;
		}

		if (newSimMode == SimulationMode::off)
		{
			EndSimulation(&gb);
		}
		else
		{
			if (!IsSimulating())
			{
				// Starting a new simulation, so save the current position
				axesVirtuallyHomed = AxesBitmap::MakeLowestNBits(numVisibleAxes);	// pretend all axes are homed
				SavePosition(simulationRestorePoint, gb);
			}
			simulationTime = 0.0;
		}
		exitSimulationWhenFileComplete = updateFileWhenSimulationComplete = false;
		simulationMode = newSimMode;
		reprap.GetMove().Simulate(newSimMode);
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
	Platform& pfm = platform;
	const bool ok = endstopsToWaitFor.IterateWhile([&pfm, activeHigh](unsigned int axis, unsigned int)->bool
								{
									const bool stopped = pfm.GetEndstops().Stopped(axis);
									return stopped == activeHigh;
								}
							 )
				&& portsToWaitFor.IterateWhile([&pfm, activeHigh](unsigned int port, unsigned int)->bool
								{
									return (port >= MaxGpInPorts || pfm.GetGpInPort(port).GetState() == activeHigh);
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

	bool seen = false, seenExtrude = false;
	GCodeResult rslt = GCodeResult::ok;

	const size_t originalVisibleAxes = numVisibleAxes;
	const char *lettersToTry = AllowedAxisLetters;
	char c;

#if SUPPORT_CAN_EXPANSION
	AxesBitmap axesToUpdate;
#endif

	const AxisWrapType newAxesType = (gb.Seen('R')) ? (AxisWrapType)gb.GetLimitedUIValue('R', (unsigned int)AxisWrapType::undefined) : AxisWrapType::undefined;
	const bool seenS = gb.Seen('S');
	const bool newAxesAreNistRotational = seenS && gb.GetLimitedUIValue('S', 2) == 1;
	while ((c = *lettersToTry) != 0)
	{
		if (gb.Seen(c))
		{
			// Found an axis letter. Get the drivers to assign to this axis.
			seen = true;
			size_t numValues = MaxDriversPerAxis;
			DriverId drivers[MaxDriversPerAxis];
			gb.GetDriverIdArray(drivers, numValues);

			// Check the driver array for out-of-range drives
			for (size_t i = 0; i < numValues; )
			{
				const DriverId driver = drivers[i];
				bool deleteItem = false;
#if SUPPORT_CAN_EXPANSION
				if (driver.IsRemote())
				{
					// Currently we don't have a way of determining how many drivers each board has, but we have a limit of 3 per board
					const ExpansionBoardData * const data = reprap.GetExpansion().GetBoardDetails(driver.boardAddress);
					if (data != nullptr && driver.localDriver >= data->numDrivers)
					{
						deleteItem = true;
					}
				}
				else
#endif
				if (driver.localDriver >= NumDirectDrivers)
				{
					deleteItem = true;
				}

				if (deleteItem)
				{
#if SUPPORT_CAN_EXPANSION
					reply.lcatf("Driver %u.%u does not exist", driver.boardAddress, driver.localDriver);
#else
					reply.lcatf("Driver %u does not exist", driver.localDriver);
#endif
					rslt = GCodeResult::error;
					--numValues;
					for (size_t j = i; j < numValues; ++j)
					{
						drivers[j] = drivers[j + 1];
					}
				}
				else
				{
					++i;
				}
			}
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
					const AxisWrapType wrapType = (newAxesType != AxisWrapType::undefined) ? newAxesType
													: (c >= 'A' && c <= 'D') ? AxisWrapType::wrapAt360			// default A thru D to rotational but not continuous
														: AxisWrapType::noWrap;									// default other axes to linear
					const bool isNistRotational = (seenS) ? newAxesAreNistRotational : (c >= 'A' && c <= 'D');
					platform.SetAxisType(drive, wrapType, isNistRotational);
					++numTotalAxes;
					if (numTotalAxes + numExtruders > MaxAxesPlusExtruders)
					{
						--numExtruders;
					}
					numVisibleAxes = numTotalAxes;						// assume any new axes are visible unless there is a P parameter
					float initialCoords[MaxAxes];
					reprap.GetMove().GetKinematics().GetAssumedInitialPosition(drive + 1, initialCoords);
					moveState.coords[drive] = initialCoords[drive];	// user has defined a new axis, so set its position
					ToolOffsetInverseTransform(moveState.coords, moveState.currentUserPosition);
					reprap.MoveUpdated();
				}
				platform.SetAxisDriversConfig(drive, numValues, drivers);
#if SUPPORT_CAN_EXPANSION
				axesToUpdate.SetBit(drive);
#endif
			}
		}
		++lettersToTry;
	}

	if (gb.Seen(extrudeLetter))
	{
		seenExtrude = true;
		size_t numValues = MaxExtruders;
		DriverId drivers[MaxExtruders];
		gb.GetDriverIdArray(drivers, numValues);
		numExtruders = numValues;
		for (size_t i = 0; i < numValues; ++i)
		{
			platform.SetExtruderDriver(i, drivers[i]);
#if SUPPORT_CAN_EXPANSION
			axesToUpdate.SetBit(ExtruderToLogicalDrive(i));
#endif
		}
		if (FilamentMonitor::CheckDriveAssignments(reply) && rslt == GCodeResult::ok)
		{
			rslt = GCodeResult::warning;
		}
	}

	if (gb.Seen('P'))
	{
		seen = true;
		const unsigned int nva = gb.GetUIValue();
		if (nva >= MinVisibleAxes && nva <= numTotalAxes)
		{
			numVisibleAxes = nva;
		}
		else
		{
			reply.lcat("Invalid number of visible axes");
			rslt = GCodeResult::error;
		}
	}

	if (seen || seenExtrude)
	{
		reprap.MoveUpdated();
		if (numVisibleAxes > originalVisibleAxes)
		{
			// In the DDA ring, the axis positions for invisible non-moving axes are not always copied over from previous moves.
			// So if we have more visible axes than before, then we need to update their positions to get them in sync.
			ToolOffsetTransform(moveState.currentUserPosition, moveState.coords);	// ensure that the position of any new axes are updated in moveBuffer
			reprap.GetMove().SetNewPosition(moveState.coords, true);		// tell the Move system where the axes are
		}
#if SUPPORT_CAN_EXPANSION
		rslt = max(rslt, platform.UpdateRemoteStepsPerMmAndMicrostepping(axesToUpdate, reply));
#endif
		return rslt;
	}

	reply.copy("Driver assignments:");
	bool printed = false;
	for (size_t axis = 0; axis < numTotalAxes; ++ axis)
	{
		reply.cat(' ');
		const AxisDriversConfig& axisConfig = platform.GetAxisDriversConfig(axis);
		if (platform.IsAxisRotational(axis))
		{
			reply.cat("(r)");
		}
		if (platform.IsAxisContinuous(axis))
		{
			reply.cat("(c)");
		}
#if 0	// shortcut axes not implemented yet
		if (platform.IsAxisShortcutAllowed(axis))
		{
			reply.cat("(s)");
		}
#endif

		char c = axisLetters[axis];
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
	return GCodeResult::ok;
}

#if SUPPORT_REMOTE_COMMANDS

// Switch the board into expansion mode. We map all drivers to individual axes.
void GCodes::SwitchToExpansionMode() noexcept
{
	numExtruders = 0;
	numVisibleAxes = numTotalAxes = NumDirectDrivers;
	FilamentMonitor::DeleteAll();
	memcpy(axisLetters, AllowedAxisLetters, sizeof(axisLetters));
	for (size_t axis = 0; axis < NumDirectDrivers; ++axis)
	{
		DriverId driver;
		driver.SetLocal(axis);
		platform.SetAxisDriversConfig(axis, 1, &driver);
	}
	isRemotePrinting = false;
}

#endif

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

	straightProbeSettings.Reset();

	switch (fraction)
	{
	case 2:
		straightProbeSettings.SetStraightProbeType(StraightProbeType::towardsWorkpieceErrorOnFailure);
		break;

	case 3:
		straightProbeSettings.SetStraightProbeType(StraightProbeType::towardsWorkpiece);
		break;

	case 4:
		straightProbeSettings.SetStraightProbeType(StraightProbeType::awayFromWorkpieceErrorOnFailure);
		break;

	case 5:
		straightProbeSettings.SetStraightProbeType(StraightProbeType::awayFromWorkpiece);
		break;
	}

	// Get the target coordinates (as user position) and check if we would move at all
	float userPositionTarget[MaxAxes];
	memcpyf(userPositionTarget, moveState.currentUserPosition, numVisibleAxes);

	bool seen = false;
	bool doesMove = false;
	for (size_t axis = 0; axis < numVisibleAxes; axis++)
	{
		if (gb.Seen(axisLetters[axis]))
		{
			seen = true;

			// Get the user provided target coordinate
			// - If prefixed by G53 add the ToolOffset that will be subtracted below in ToolOffsetTransform as we ignore any offsets when G53 is active
			// - otherwise add current workplace offsets so we go where the user expects to go
			// comparable to hoe DoStraightMove/DoArcMove does it
			const float axisTarget = gb.GetDistance() + (gb.LatestMachineState().g53Active ? GetCurrentToolOffset(axis) : GetWorkplaceOffset(axis));
			if (axisTarget != userPositionTarget[axis])
			{
				doesMove = true;
			}
			userPositionTarget[axis] = axisTarget;
			straightProbeSettings.AddMovingAxis(axis);
		}
	}

	// No axis letters seen
	if (!seen)
	{
		// Signal error for G38.2 and G38.4
		if (straightProbeSettings.SignalError())
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
		if (straightProbeSettings.SignalError())
		{
			reply.copy("Target equals current position.");
			return GCodeResult::error;
		}
		return GCodeResult::ok;
	}
	// Convert target user position to machine coordinates and save them in StraightProbeSettings
	ToolOffsetTransform(userPositionTarget, straightProbeSettings.GetTarget());

	// See whether we are using a user-defined Z probe or just current one
	const size_t probeToUse = (gb.Seen('K') || gb.Seen('P')) ? gb.GetUIValue() : 0;

	// Check if this probe exists to not run into a nullptr dereference later
	if (platform.GetEndstops().GetZProbe(probeToUse).IsNull())
	{
		reply.catf("Invalid probe number: %d", probeToUse);
		return GCodeResult::error;
	}
	straightProbeSettings.SetZProbeToUse(probeToUse);

	gb.SetState(GCodeState::straightProbe0);
	return GCodeResult::ok;
}

// Search for and return an axis, throw if none found or that axis hasn't been homed. On return we can fetch the parameter value after the axis letter.
size_t GCodes::FindAxisLetter(GCodeBuffer& gb) THROWS(GCodeException)
{
	for (size_t axis = 0; axis < numVisibleAxes; axis++)
	{
		if (gb.Seen(axisLetters[axis]))
		{
			if (IsAxisHomed(axis))
			{
				return axis;
			}
			throw GCodeException(gb.GetLineNumber(), -1, "%c axis has not been homed", (uint32_t)axisLetters[axis]);
		}
	}

	throw GCodeException(gb.GetLineNumber(), -1, "No axis specified");
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

	// Get the feed rate and axis
	gb.MustSee(feedrateLetter);
	m585Settings.feedRate = gb.LatestMachineState().feedRate = gb.GetSpeed();		// don't apply the speed factor to homing and other special moves
	m585Settings.axisNumber = FindAxisLetter(gb);
	m585Settings.offset = gb.GetDistance();

	// See whether we are using a Z probe or just endstops
	if (gb.Seen('K'))
	{
		(void)SetZProbeNumber(gb, 'K');						// throws if the probe doesn't exist
		m585Settings.useProbe = true;
	}
	else if (gb.Seen('P'))
	{
		(void)SetZProbeNumber(gb, 'P');						// throws if the probe doesn't exist
		m585Settings.useProbe = true;
	}
	else
	{
		m585Settings.useProbe = false;
	}

	// Decide which way and how far to go
	ToolOffsetTransform(moveState.currentUserPosition, moveState.coords);
	m585Settings.probingLimit = (gb.Seen('R')) ? moveState.coords[m585Settings.axisNumber] + gb.GetDistance()
								: (gb.Seen('S') && gb.GetIValue() > 0) ? platform.AxisMinimum(m585Settings.axisNumber)
									: platform.AxisMaximum(m585Settings.axisNumber);
	if (m585Settings.useProbe)
	{
		gb.SetState(GCodeState::probingToolOffset1);
		DeployZProbe(gb);
	}
	else
	{
		gb.SetState(GCodeState::probingToolOffset3);		// skip the Z probe stuff
	}

	return GCodeResult::ok;
}

// Set up a probing move for M675. If using a Z probe, it has already been deployed
// Return true if successful, else SetError has been called to save the error message
bool GCodes::SetupM585ProbingMove(GCodeBuffer& gb) noexcept
{
	bool reduceAcceleration;
	if (m585Settings.useProbe)
	{
		const auto zp = platform.GetZProbeOrDefault(currentZProbeNumber);
		if (zp->Stopped())
		{
			gb.LatestMachineState().SetError("Probe already triggered before probing move started");
			return false;
		}
		if (!platform.GetEndstops().EnableZProbe(currentZProbeNumber) || !zp->SetProbing(true))
		{
			gb.LatestMachineState().SetError("Failed to enable probe");
			return false;
		}
		reduceAcceleration = true;
	}
	else if (!platform.GetEndstops().EnableAxisEndstops(AxesBitmap::MakeFromBits(m585Settings.axisNumber), false, reduceAcceleration))
	{
		gb.LatestMachineState().SetError("Failed to enable endstop");
		return false;
	}

	SetMoveBufferDefaults();
	ToolOffsetTransform(moveState.currentUserPosition, moveState.coords);
	moveState.feedRate = m585Settings.feedRate;
	moveState.coords[m585Settings.axisNumber] = m585Settings.probingLimit;
	moveState.reduceAcceleration = reduceAcceleration;
	moveState.checkEndstops = true;
	moveState.canPauseAfter = false;
	zProbeTriggered = false;
	NewMoveAvailable(1);
	return true;
}

GCodeResult GCodes::FindCenterOfCavity(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
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

	// Get the feed rate, backoff distance, and axis
	gb.MustSee(feedrateLetter);
	m675Settings.feedRate = gb.LatestMachineState().feedRate = gb.GetSpeed();		// don't apply the speed factor to homing and other special moves
	m675Settings.backoffDistance = gb.Seen('R') ? gb.GetDistance() : 5.0;
	m675Settings.axisNumber = FindAxisLetter(gb);

	// Get the probe number from the K or P parameter
	const char probeLetter = gb.MustSee('K', 'P');				// throws if neither character is found
	(void)SetZProbeNumber(gb, probeLetter);						// throws if the probe doesn't exist
	gb.SetState(GCodeState::findCenterOfCavity1);
	DeployZProbe(gb);

	return GCodeResult::ok;
}

// Set up a probing move for M675. If using a Z probe, it has already been deployed
// Return true if successful, else SetError has been called to save the error message
bool GCodes::SetupM675ProbingMove(GCodeBuffer& gb, bool towardsMin) noexcept
{
	const auto zp = platform.GetZProbeOrDefault(currentZProbeNumber);
	if (zp->Stopped())
	{
		gb.LatestMachineState().SetError("Probe already triggered before probing move started");
		return false;
	}
	if (!platform.GetEndstops().EnableZProbe(currentZProbeNumber) || !zp->SetProbing(true))
	{
		gb.LatestMachineState().SetError("Failed to enable probe");
		return false;
	}

	SetMoveBufferDefaults();
	ToolOffsetTransform(moveState.currentUserPosition, moveState.coords);
	moveState.coords[m675Settings.axisNumber] = towardsMin ? platform.AxisMinimum(m675Settings.axisNumber) : platform.AxisMaximum(m675Settings.axisNumber);
	moveState.feedRate = m675Settings.feedRate;
	moveState.checkEndstops = true;
	moveState.canPauseAfter = false;
	zProbeTriggered = false;
	NewMoveAvailable(1);						// kick off the move
	return true;
}

void GCodes::SetupM675BackoffMove(GCodeBuffer& gb, float position) noexcept
{
	SetMoveBufferDefaults();
	ToolOffsetTransform(moveState.currentUserPosition, moveState.coords);
	moveState.coords[m675Settings.axisNumber] = position;
	moveState.feedRate = m675Settings.feedRate;
	moveState.canPauseAfter = false;
	NewMoveAvailable(1);
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

#if HAS_WIFI_NETWORKING || HAS_AUX_DEVICES || HAS_MASS_STORAGE || HAS_SBC_INTERFACE

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
		if (boardNumber != CanInterface::GetCanAddress())
		{
			return reprap.GetExpansion().UpdateRemoteFirmware(boardNumber, gb, reply);
		}
	}
#endif

#if HAS_AUX_DEVICES && ALLOW_ARBITRARY_PANELDUE_PORT	// Disabled until we allow PanelDue on another port
	if (gb.Seen('A'))
	{
		serialChannelForPanelDueFlashing = gb.GetLimitedUIValue('A', NumSerialChannels, 1);
	}
#endif

#ifdef DUET3_ATE
	Duet3Ate::PowerOffEUT();
#endif

	reprap.GetHeat().SwitchOffAll(true);				// turn all heaters off because the main loop may get suspended
	DisableDrives();									// all motors off

	if (firmwareUpdateModuleMap.IsEmpty())					// have we worked out which modules to update?
	{
		// Find out which modules we have been asked to update
		if (gb.Seen('S'))
		{
			uint32_t modulesToUpdate[5];
			size_t numUpdateModules = ARRAY_SIZE(modulesToUpdate);
			gb.GetUnsignedArray(modulesToUpdate, numUpdateModules, false);
			for (size_t i = 0; i < numUpdateModules; ++i)
			{
				uint32_t t = modulesToUpdate[i];
				if (t >= FirmwareUpdater::NumUpdateModules)
				{
					reply.printf("Invalid module number '%" PRIu32 "'\n", t);
					firmwareUpdateModuleMap.Clear();
					return GCodeResult::error;
					break;
				}
				firmwareUpdateModuleMap.SetBit(t);
			}
		}
		else
		{
			firmwareUpdateModuleMap.SetBit(0);		// no modules specified, so update module 0 to match old behaviour
		}

		if (firmwareUpdateModuleMap.IsEmpty())
		{
			return GCodeResult::ok;						// nothing to update
		}

		String<MaxFilenameLength> filenameString;
		if (gb.Seen('P'))
		{
			if (firmwareUpdateModuleMap.CountSetBits() > 1)
			{
				reply.copy("Filename can only be provided when updating exactly one module\n");
				firmwareUpdateModuleMap.Clear();
				return GCodeResult::error;
			}
			gb.GetQuotedString(filenameString.GetRef());
		}

		// Check prerequisites of all modules to be updated, if any are not met then don't update any of them
#if HAS_WIFI_NETWORKING || HAS_AUX_DEVICES
		const auto result = FirmwareUpdater::CheckFirmwareUpdatePrerequisites(
				firmwareUpdateModuleMap, gb, reply,
# if HAS_AUX_DEVICES
				serialChannelForPanelDueFlashing,
#else
				0,
#endif
				filenameString.GetRef());
		if (result != GCodeResult::ok)
		{
			firmwareUpdateModuleMap.Clear();
			return result;
		}
#endif
		if (firmwareUpdateModuleMap.IsBitSet(0) && !reprap.CheckFirmwareUpdatePrerequisites(reply, filenameString.GetRef()))
		{
			firmwareUpdateModuleMap.Clear();
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

#endif

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
	size_t drivesCount = numVisibleAxes;
	DriverId driverIds[drivesCount];
	gb.GetDriverIdArray(driverIds, drivesCount);

	bool const isEncoderReading = (gb.GetCommandFraction() == 3);
	if (isEncoderReading)
	{
		reply.copy("[");
	}

	// Hangprinter needs M569 to support multiple P parameters in M569.3 and M569.4. This poses a problem for other uses of M569 because the output may be too long
	// to fit in the reply buffer, and we can only use an OutputBuffer instead if the overall result is success.
	// Therefore we only support multiple P parameters for subfunctions 3 and 4.
	GCodeResult res = GCodeResult::ok;
	for (size_t i = 0; i < drivesCount; ++i)
	{
		DriverId const id = driverIds[i];
		res =
#if SUPPORT_CAN_EXPANSION
			(id.IsRemote())
				? CanInterface::ConfigureRemoteDriver(id, gb, reply)
					:
#endif
					ConfigureLocalDriver(gb, reply, id.localDriver);
		if (res != GCodeResult::ok || (!isEncoderReading && gb.GetCommandFraction() != 4))
		{
			break;
		}
	}

	if (isEncoderReading && res == GCodeResult::ok)
	{
		reply.cat(" ],\n");
	}
	return res;
}

GCodeResult GCodes::ConfigureLocalDriver(GCodeBuffer& gb, const StringRef& reply, uint8_t drive) THROWS(GCodeException)
{
	if (drive >= platform.GetNumActualDirectDrivers())
	{
		reply.printf("Driver number %u out of range", drive);
		return GCodeResult::error;
	}

	switch (gb.GetCommandFraction())
	{
	case 0:
	case -1:
		return ConfigureLocalDriverBasicParameters(gb, reply, drive);

	case 1:
	case 3:
	case 5:
	case 6:
		// Main board drivers do not support closed loop modes, or reading encoders
		reply.copy("Command is not supported on local drivers");
		return GCodeResult::error;


#if SUPPORT_TMC22xx || SUPPORT_TMC51xx
	case 2:			// read/write smart driver register
		{
			gb.MustSee('R');
			const uint8_t regNum = gb.GetLimitedUIValue('R', 0, 0x80);
			if (gb.Seen('V'))
			{
				const uint32_t regVal = gb.GetUIValue();
				return SmartDrivers::SetAnyRegister(drive, reply, regNum, regVal);
			}
			return SmartDrivers::GetAnyRegister(drive, reply, regNum);
		}
#endif

	case 7:			// configure brake
		return platform.ConfigureDriverBrakePort(gb, reply, drive);

	default:
		return GCodeResult::warningNotSupported;
	}
}

GCodeResult GCodes::ConfigureLocalDriverBasicParameters(GCodeBuffer& gb, const StringRef& reply, uint8_t drive) THROWS(GCodeException)
{
	if (gb.SeenAny("RS"))
	{
		if (!LockMovementAndWaitForStandstill(gb))
		{
			return GCodeResult::notFinished;
		}
	}

	bool seen = false;
	if (gb.Seen('S'))
	{
		seen = true;
		platform.SetDirectionValue(drive, gb.GetIValue() != 0);
	}
	if (gb.Seen('R'))
	{
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
		// Print the basic parameters common to all types of driver
		reply.printf("Drive %u runs %s, active %s enable, timing ",
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
			// It's a smart driver, so print the parameters common to all modes, except for the position
			reply.catf(", mode %s, ccr 0x%05" PRIx32 ", toff %" PRIu32 ", tblank %" PRIu32,
					TranslateDriverMode(SmartDrivers::GetDriverMode(drive)),
					SmartDrivers::GetRegister(drive, SmartDriverRegister::chopperControl),
					SmartDrivers::GetRegister(drive, SmartDriverRegister::toff),
					SmartDrivers::GetRegister(drive, SmartDriverRegister::tblank)
				);

# if SUPPORT_TMC51xx
			{
				const uint32_t thigh = SmartDrivers::GetRegister(drive, SmartDriverRegister::thigh);
				const uint32_t axis = SmartDrivers::GetAxisNumber(drive);
				bool bdummy;
				const float mmPerSec = (12000000.0 * SmartDrivers::GetMicrostepping(drive, bdummy))/(256 * thigh * platform.DriveStepsPerUnit(axis));
				reply.catf(", thigh %" PRIu32 " (%.1f mm/sec)", thigh, (double)mmPerSec);
			}
# endif

			// Print the additional parameters that are relevant in the current mode
			if (SmartDrivers::GetDriverMode(drive) == DriverMode::spreadCycle)
			{
				reply.catf(", hstart/hend/hdec %" PRIu32 "/%" PRIu32 "/%" PRIu32,
							SmartDrivers::GetRegister(drive, SmartDriverRegister::hstart),
							SmartDrivers::GetRegister(drive, SmartDriverRegister::hend),
							SmartDrivers::GetRegister(drive, SmartDriverRegister::hdec)
						  );
			}

# if SUPPORT_TMC22xx || SUPPORT_TMC51xx
			if (SmartDrivers::GetDriverMode(drive) == DriverMode::stealthChop)
			{
				const uint32_t tpwmthrs = SmartDrivers::GetRegister(drive, SmartDriverRegister::tpwmthrs);
				const uint32_t axis = SmartDrivers::GetAxisNumber(drive);
				bool bdummy;
				const float mmPerSec = (12000000.0 * SmartDrivers::GetMicrostepping(drive, bdummy))/(256 * tpwmthrs * platform.DriveStepsPerUnit(axis));
				const uint32_t pwmScale = SmartDrivers::GetRegister(drive, SmartDriverRegister::pwmScale);
				const uint32_t pwmAuto = SmartDrivers::GetRegister(drive, SmartDriverRegister::pwmAuto);
				const unsigned int pwmScaleSum = pwmScale & 0xFF;
				const int pwmScaleAuto = (int)((((pwmScale >> 16) & 0x01FF) ^ 0x0100) - 0x0100);
				const unsigned int pwmOfsAuto = pwmAuto & 0xFF;
				const unsigned int pwmGradAuto = (pwmAuto >> 16) & 0xFF;
				reply.catf(", tpwmthrs %" PRIu32 " (%.1f mm/sec)"", pwmScaleSum %u, pwmScaleAuto %d, pwmOfsAuto %u, pwmGradAuto %u",
							tpwmthrs, (double)mmPerSec, pwmScaleSum, pwmScaleAuto, pwmOfsAuto, pwmGradAuto);
			}
# endif
			// Finally, print the microstep position
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
		}
#endif
	}
	return GCodeResult::ok;
}

#if SUPPORT_COORDINATE_ROTATION

// Handle G68
GCodeResult GCodes::HandleG68(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	if (!LockMovementAndWaitForStandstill(gb))
	{
		return GCodeResult::notFinished;
	}
	if (gb.CurrentFileMachineState().selectedPlane != 0)
	{
		reply.copy("this command may only be used when the selected plane is XY");
		return GCodeResult::error;
	}

	float angle, centreX, centreY;
	gb.MustSee('R');
	angle = gb.GetFValue();
	gb.MustSee('A', 'X');
	centreX = gb.GetFValue();
	gb.MustSee('B', 'Y');
	centreY= gb.GetFValue();

	g68Centre[0] = centreX + GetWorkplaceOffset(0);
	g68Centre[1] = centreY + GetWorkplaceOffset(1);
	if (gb.Seen('I'))
	{
		g68Angle += angle;
	}
	else
	{
		g68Angle = angle;
	}
	return GCodeResult::ok;
}

// Account for coordinate rotation. Only called wheh the angle to rotate is nonzero, so we don't check that here.
void GCodes::RotateCoordinates(float angleDegrees, float coords[2]) const noexcept
{
	const float angle = angleDegrees * DegreesToRadians;
	const float newX = (coords[0] - g68Centre[0]) * cosf(angle)    + (coords[1] - g68Centre[1]) * sinf(angle) + g68Centre[0];
	const float newY = (coords[0] - g68Centre[0]) * (-sinf(angle)) + (coords[1] - g68Centre[1]) * cosf(angle) + g68Centre[1];
	coords[0] = newX;
	coords[1] = newY;
}

#endif

// Change a live extrusion factor
void GCodes::ChangeExtrusionFactor(unsigned int extruder, float factor) noexcept
{
	if (moveState.segmentsLeft != 0 && moveState.applyM220M221)
	{
		moveState.coords[ExtruderToLogicalDrive(extruder)] *= factor/extrusionFactors[extruder];	// last move not gone, so update it
	}
	extrusionFactors[extruder] = factor;
	reprap.MoveUpdated();
}

// Deploy the Z probe unless it has already been deployed explicitly
// The required next state must be set up (e.g. by gb.SetState()) before calling this
void GCodes::DeployZProbe(GCodeBuffer& gb) noexcept
{
	auto zp = reprap.GetPlatform().GetEndstops().GetZProbe(currentZProbeNumber);
	if (zp.IsNotNull() && zp->GetProbeType() != ZProbeType::none && !zp->IsDeployedByUser())
	{
		String<StringLength20> fileName;
		fileName.printf(DEPLOYPROBE "%u.g", currentZProbeNumber);
		if (!DoFileMacro(gb, fileName.c_str(), false, SystemHelperMacroCode) && currentZProbeNumber == 0)
		{
			DoFileMacro(gb, DEPLOYPROBE ".g", false, SystemHelperMacroCode);
		}
	}
}

// Retract the Z probe unless it was deployed explicitly (in which case, wait for the user to retract it explicitly)
// The required next state must be set up (e.g. by gb.SetState()) before calling this
void GCodes::RetractZProbe(GCodeBuffer& gb) noexcept
{
	auto zp = reprap.GetPlatform().GetEndstops().GetZProbe(currentZProbeNumber);
	if (zp.IsNotNull() && zp->GetProbeType() != ZProbeType::none && !zp->IsDeployedByUser())
	{
		String<StringLength20> fileName;
		fileName.printf(RETRACTPROBE "%u.g", currentZProbeNumber);
		if (!DoFileMacro(gb, fileName.c_str(), false, SystemHelperMacroCode) && currentZProbeNumber == 0)
		{
			DoFileMacro(gb, RETRACTPROBE ".g", false, SystemHelperMacroCode);
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
		"; --- layer",				// KiriMoto (the line starts with ;;)
		"BEGIN_LAYER_OBJECT z=",	// KISSlicer (followed by Z height)
		"HEIGHT",					// Ideamaker
		"PRINTING",					// Ideamaker
		"REMAINING_TIME",			// Ideamaker
		"LAYER_CHANGE"				// SuperSlicer
	};

	String<StringLength100> comment;
	gb.GetCompleteParameters(comment.GetRef());
	const char *fullText = comment.c_str();
	while (*fullText == ' ')
	{
		++fullText;
	}

	for (size_t i = 0; i < ARRAY_SIZE(StartStrings); ++i)
	{
		if (StringStartsWith(fullText, StartStrings[i]))
		{
			const char *text = fullText + strlen(StartStrings[i]);
			if (!isalpha(*text) && *text != '_')			// need this test to avoid recognising "processName" as "process"
			{
				while (*text == ' ' || *text == ':')
				{
					++text;
				}

				switch (i)
				{
				case 1:		// MESH (Cura)
#if TRACK_OBJECT_NAMES
					if (StringStartsWith(text, "NONMESH"))
					{
						buildObjects.StopObject(gb);
					}
					else
					{
						buildObjects.StartObject(gb, text);
					}
#endif
					break;

				case 9:		// PRINTING (Ideamaker)
#if TRACK_OBJECT_NAMES
					if (StringStartsWith(text, "NON-OBJECT"))
					{
						buildObjects.StopObject(gb);
					}
					else
					{
						buildObjects.StartObject(gb, text);
					}
#endif
					break;

				case 0:		// printing object (slic3r)
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
				case 6:		// layer (counting from 0)
					{
						const char *endptr;
						const int32_t layer = StrToI32(text, &endptr);		// IdeaMaker uses negative layer numbers for the raft, so read a signed number here
						if (endptr != text && layer >= 0)
						{
							reprap.GetPrintMonitor().SetLayerNumber((uint32_t)((i == 4) ? layer : layer + 1));
						}
						text = endptr;
						if (!StringStartsWith(text, ", z = "))				// S3D gives us the height too
						{
							break;
						}
						text += 6;			// skip ", z = "
					}
					// no break

				case 7:		// new layer, but we are given the Z height, not the layer number
				case 8:
					{
						const char *endptr;
						const float layerZ = SafeStrtof(text, &endptr);
						if (endptr != text)
						{
							reprap.GetPrintMonitor().SetLayerZ(layerZ);
						}
					}
					break;

				case 10:	// REMAINING_TIME (Ideamaker), followed by time in seconds as an integer
					{
						const char *endptr;
						const uint32_t secondsRemaining = StrToU32(text, &endptr);
						if (endptr != text)
						{
							reprap.GetPrintMonitor().SetSlicerTimeLeft(secondsRemaining);
						}
					}
					break;

				case 11:	// LAYER_CHANGE (SuperSlicer). No layer number provided.
					reprap.GetPrintMonitor().LayerChange();
					break;
				}
				break;
			}
		}
	}
	return true;
}

// Handle M957
GCodeResult GCodes::RaiseEvent(GCodeBuffer& gb, const StringRef &reply) THROWS(GCodeException)
{
	String<StringLength50> temp;
	gb.MustSee('E');
	gb.GetQuotedString(temp.GetRef(), false);
	const EventType et(temp.c_str());
	if (!et.IsValid())
	{
		reply.copy("Invalid event type");
		return GCodeResult::error;
	}

	const unsigned int devNum = gb.GetLimitedUIValue('D', 256);
	const unsigned int param = (gb.Seen('P')) ? gb.GetUIValue() : 0;
	const unsigned int boardAddress = (gb.Seen('B')) ? gb.GetUIValue() : CanInterface::GetCanAddress();
	temp.Clear();
	if (gb.Seen('S'))
	{
		gb.GetQuotedString(temp.GetRef(), true);
	}

	const bool added = Event::AddEvent(et, param, boardAddress, devNum, "%s", temp.c_str());
	if (added)
	{
		return GCodeResult::ok;
	}
	reply.copy("a similar event is already queued");
	return GCodeResult::warning;
}

// Process an event. The autoPauseGCode buffer calls this when there is a new event to be processed.
// This is a separate function because it allocates strings on the stack.
void GCodes::ProcessEvent(GCodeBuffer& gb) noexcept
{
	// Get the event message
	String<StringLength100> eventText;
	const MessageType mt = Event::GetTextDescription(eventText.GetRef());
	platform.MessageF(mt, "%s\n", eventText.c_str());					// tell the user about the event and log it

	// Get the name of the macro file that we should look for
	String<StringLength50> macroName;
	Event::GetMacroFileName(macroName.GetRef());
	if (platform.SysFileExists(macroName.c_str()))
	{
		// Set up the macro parameters
		VariableSet vars;
		Event::GetParameters(vars);
		vars.InsertNewParameter("S", ExpressionValue(StringHandle(eventText.c_str())));

		// Run the macro
		gb.SetState(GCodeState::finishedProcessingEvent);				// cancel the event when we have finished processing it
		if (DoFileMacro(gb, macroName.c_str(), false, AsyncSystemMacroCode, vars))
		{
			return;
		}
	}

	// We didn't execute the macro, so do the default action
	if (Event::GetDefaultPauseReason() == PrintPausedReason::dontPause)
	{
		Event::FinishedProcessing();									// nothing more to do
	}
	else
	{
		// It's a serious event that causes the print to pause by default, so send an alert
		String<StringLength100> eventText;
		Event::GetTextDescription(eventText.GetRef());
		const bool isPrinting = IsReallyPrinting();
		platform.SendAlert(GenericMessage, eventText.c_str(), (isPrinting) ? "Printing paused" : "Event notification", 1, 0.0, AxesBitmap());
		if (IsReallyPrinting())
		{
			// We are going to pause. It may need to wait for the movement lock, so do it in a new state.
			gb.SetState(GCodeState::processingEvent);
		}
		else
		{
			Event::FinishedProcessing();
		}
	}
}

#if !HAS_MASS_STORAGE && !HAS_EMBEDDED_FILES && defined(DUET_NG)

// Function called by RepRap.cpp to enable PanelDue by default in the Duet 2 SBC build
void GCodes::SetAux0CommsProperties(uint32_t mode) const noexcept
{
	auxGCode->SetCommsProperties(mode);
}

#endif

// End
