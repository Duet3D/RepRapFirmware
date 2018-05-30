/*
 * GCodes3.cpp
 *
 *  Created on: 5 Dec 2017
 *      Author: David
 *  This file contains functions that are called form file GCodes2.cpp to execute various G and M codes.
 */

#include "GCodes.h"

#include "GCodeBuffer.h"
#include "Heating/Heat.h"
#include "Movement/Move.h"
#include "RepRap.h"
#include "Tools/Tool.h"
#include "PrintMonitor.h"

#if HAS_WIFI_NETWORKING
# include "FirmwareUpdater.h"
#endif

#include "Wire.h"

// Set or print the Z probe. Called by G31.
// Note that G31 P or G31 P0 prints the parameters of the currently-selected Z probe.
GCodeResult GCodes::SetPrintZProbe(GCodeBuffer& gb, const StringRef& reply)
{
	ZProbeType probeType;
	bool seenT = false;
	if (gb.Seen('T'))
	{
		unsigned int tp = gb.GetUIValue();
		if (tp == 0 || tp >= (unsigned int)ZProbeType::numTypes)
		{
			reply.copy("Invalid Z probe type");
			return GCodeResult::error;
		}

		probeType = (ZProbeType)tp;
		seenT = true;
	}
	else
	{
		probeType = platform.GetZProbeType();
		seenT = false;
	}

	ZProbe params = platform.GetZProbeParameters(probeType);
	bool seen = false;
	gb.TryGetFValue(axisLetters[X_AXIS], params.xOffset, seen);
	gb.TryGetFValue(axisLetters[Y_AXIS], params.yOffset, seen);
	gb.TryGetFValue(axisLetters[Z_AXIS], params.triggerHeight, seen);
	gb.TryGetIValue('P', params.adcValue, seen);

	if (gb.Seen('C'))
	{
		params.temperatureCoefficient = gb.GetFValue();
		seen = true;
		if (gb.Seen('S'))
		{
			params.calibTemperature = gb.GetFValue();
		}
		else
		{
			// Use the current bed temperature as the calibration temperature if no value was provided
			params.calibTemperature = platform.GetZProbeTemperature();
		}
	}

	if (seen)
	{
		if (!LockMovementAndWaitForStandstill(gb))
		{
			return GCodeResult::notFinished;
		}
		platform.SetZProbeParameters(probeType, params);
	}
	else if (seenT)
	{
		// Don't bother printing temperature coefficient and calibration temperature because we will probably remove them soon
		reply.printf("Threshold %" PRIi32 ", trigger height %.2f, offsets X%.1f Y%.1f", params.adcValue, (double)params.triggerHeight, (double)params.xOffset, (double)params.yOffset);
	}
	else
	{
		const int v0 = platform.GetZProbeReading();
		int v1, v2;
		switch (platform.GetZProbeSecondaryValues(v1, v2))
		{
		case 1:
			reply.printf("%d (%d)", v0, v1);
			break;
		case 2:
			reply.printf("%d (%d, %d)", v0, v1, v2);
			break;
		default:
			reply.printf("%d", v0);
			break;
		}
	}
	return GCodeResult::ok;
}

// Deal with G60
GCodeResult GCodes::SavePosition(GCodeBuffer& gb,const  StringRef& reply)
{
	const uint32_t sParam = (gb.Seen('S')) ? gb.GetUIValue() : 0;
	if (sParam < ARRAY_SIZE(numberedRestorePoints))
	{
		SavePosition(numberedRestorePoints[sParam], gb);
		return GCodeResult::ok;
	}

	reply.copy("Bad restore point number");
	return GCodeResult::error;
}

// This handles G92. Return true if completed, false if it needs to be called again.
GCodeResult GCodes::SetPositions(GCodeBuffer& gb)
{
	// Don't wait for the machine to stop if only extruder drives are being reset.
	// This avoids blobs and seams when the gcode uses absolute E coordinates and periodically includes G92 E0.
	AxesBitmap axesIncluded = 0;
	for (size_t axis = 0; axis < numVisibleAxes; ++axis)
	{
		if (gb.Seen(axisLetters[axis]))
		{
			const float axisValue = gb.GetFValue();
			if (axesIncluded == 0)
			{
				if (!LockMovementAndWaitForStandstill(gb))	// lock movement and get current coordinates
				{
					return GCodeResult::notFinished;
				}
			}
			SetBit(axesIncluded, axis);
			currentUserPosition[axis] = axisValue * distanceScale;
		}
	}

	// Handle any E parameter in the G92 command
	if (gb.Seen(extrudeLetter))
	{
		virtualExtruderPosition = gb.GetFValue() * distanceScale;
	}

	if (axesIncluded != 0)
	{
		ToolOffsetTransform(currentUserPosition, moveBuffer.coords);
		if (reprap.GetMove().GetKinematics().LimitPosition(moveBuffer.coords, numVisibleAxes, LowestNBits<AxesBitmap>(numVisibleAxes), false))	// pretend that all axes are homed
		{
			ToolOffsetInverseTransform(moveBuffer.coords, currentUserPosition);		// make sure the limits are reflected in the user position
		}
		reprap.GetMove().SetNewPosition(moveBuffer.coords, true);
		axesHomed |= reprap.GetMove().GetKinematics().AxesAssumedHomed(axesIncluded);

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
GCodeResult GCodes::OffsetAxes(GCodeBuffer& gb)
{
	for (size_t axis = 0; axis < numVisibleAxes; axis++)
	{
		if (gb.Seen(axisLetters[axis]))
		{
#if SUPPORT_WORKPLACE_COORDINATES
			workplaceCoordinates[0][axis]
#else
			axisOffsets[axis]
#endif
						 = -(gb.GetFValue() * distanceScale);
		}
	}

	return GCodeResult::ok;
}

#if SUPPORT_WORKPLACE_COORDINATES

// Set workspace coordinates
GCodeResult GCodes::GetSetWorkplaceCoordinates(GCodeBuffer& gb, const StringRef& reply, bool compute)
{
	if (gb.Seen('P'))
	{
		const uint32_t cs = gb.GetIValue();
		if (cs > 0 && cs <= NumCoordinateSystems)
		{
			bool seen = false;
			for (size_t axis = 0; axis < numVisibleAxes; axis++)
			{
				if (gb.Seen(axisLetters[axis]))
				{
					const float coord = gb.GetFValue() * distanceScale;
					if (!seen)
					{
						if (!LockMovementAndWaitForStandstill(gb))						// make sure the user coordinates are stable and up to date
						{
							return GCodeResult::notFinished;
						}
						seen = true;
					}
					workplaceCoordinates[cs - 1][axis] = (compute)
														? currentUserPosition[axis] - coord + workplaceCoordinates[currentCoordinateSystem][axis]
															: coord;
				}
			}

			if (seen)
			{
				ToolOffsetInverseTransform(moveBuffer.coords, currentUserPosition);		// update user coordinates in case we are using the workspace we just changed
			}
			else
			{
				reply.printf("Origin of workplace %" PRIu32 ":", cs);
				for (size_t axis = 0; axis < numVisibleAxes; axis++)
				{
					reply.catf(" %c%.2f", axisLetters[axis], (double)workplaceCoordinates[cs - 1][axis]);
				}
			}
			return GCodeResult::ok;
		}
	}

	return GCodeResult::badOrMissingParameter;
}

#endif

// Define the probing grid, called when we see an M557 command
GCodeResult GCodes::DefineGrid(GCodeBuffer& gb, const StringRef &reply)
{
	if (gb.Seen('P'))
	{
		reply.copy("Error: M557 P parameter is no longer supported. Use a bed.g file instead.\n");
		return GCodeResult::error;
	}

	if (!LockMovement(gb))							// to ensure that probing is not already in progress
	{
		return GCodeResult::notFinished;
	}

	bool seenX = false, seenY = false, seenR = false, seenS = false;
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
	if (gb.TryGetFloatArray('S', 2, spacings, reply, seenS, true))
	{
		return GCodeResult::error;
	}

	float radius = -1.0;
	gb.TryGetFValue('R', radius, seenR);

	if (!seenX && !seenY && !seenR && !seenS)
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
		reply.copy("specify both or neither of X and Y in M577");
		return GCodeResult::error;
	}

	if (!seenX && !seenR)
	{
		// Must have given just the S parameter
		reply.copy("specify at least radius or X,Y ranges in M577");
		return GCodeResult::error;
	}

	if (!seenX)
	{
		if (radius > 0)
		{
			const float effectiveXRadius = floorf((radius - 0.1)/spacings[0]) * spacings[0];
			xValues[0] = -effectiveXRadius;
			xValues[1] =  effectiveXRadius + 0.1;

			const float effectiveYRadius = floorf((radius - 0.1)/spacings[1]) * spacings[1];
			yValues[0] = -effectiveYRadius;
			yValues[1] =  effectiveYRadius + 0.1;
		}
		else
		{
			reply.copy("M577 radius must be positive unless X and Y are specified");
			return GCodeResult::error;
		}
	}

	if (defaultGrid.Set(xValues, yValues, radius, spacings))
	{
		return GCodeResult::ok;
	}

	const float xRange = (seenX) ? xValues[1] - xValues[0] : 2 * radius;
	const float yRange = (seenX) ? yValues[1] - yValues[0] : 2 * radius;
	reply.copy("bad grid definition: ");
	defaultGrid.PrintError(xRange, yRange, reply);
	return GCodeResult::error;
}

// Handle M37 to simulate a whole file
GCodeResult GCodes::SimulateFile(GCodeBuffer& gb, const StringRef &reply, const StringRef& file, bool updateFile)
{
	if (reprap.GetPrintMonitor().IsPrinting())
	{
		reply.copy("cannot simulate while a file is being printed");
		return GCodeResult::error;
	}

	if (QueueFileToPrint(file.c_str(), reply))
	{
		if (simulationMode == 0)
		{
			axesHomedBeforeSimulation = axesHomed;
			axesHomed = LowestNBits<AxesBitmap>(numVisibleAxes);	// pretend all axes are homed
			reprap.GetMove().GetCurrentUserPosition(simulationRestorePoint.moveCoords, 0, reprap.GetCurrentXAxes(), reprap.GetCurrentYAxes());
			simulationRestorePoint.feedRate = gb.MachineState().feedRate;
		}
		simulationTime = 0.0;
		exitSimulationWhenFileComplete = true;
		updateFileWhenSimulationComplete = updateFile;
		simulationMode = 1;
		reprap.GetMove().Simulate(simulationMode);
		reprap.GetPrintMonitor().StartingPrint(file.c_str());
		StartPrinting(true);
		reply.printf("Simulating print of file %s", file.c_str());
		return GCodeResult::ok;
	}

	return GCodeResult::error;
}

// handle M37 to change the simulation mode
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
				axesHomed = LowestNBits<AxesBitmap>(numVisibleAxes);	// pretend all axes are homed
				reprap.GetMove().GetCurrentUserPosition(simulationRestorePoint.moveCoords, 0, reprap.GetCurrentXAxes(), reprap.GetCurrentYAxes());
				simulationRestorePoint.feedRate = gb.MachineState().feedRate;
			}
			simulationTime = 0.0;
		}
		exitSimulationWhenFileComplete = updateFileWhenSimulationComplete = false;
		simulationMode = (uint8_t)newSimulationMode;
		reprap.GetMove().Simulate(simulationMode);
	}
	return GCodeResult::ok;
}

// Handle M558
GCodeResult GCodes::SetOrReportZProbe(GCodeBuffer& gb, const StringRef &reply)
{
	bool seenType = false, seenParam = false;

	// We must get and set the Z probe type first before setting the dive height etc., because different probe types may have different parameters
	if (gb.Seen('P'))		// probe type
	{
		platform.SetZProbeType(gb.GetIValue());
		seenType = true;
		DoDwellTime(gb, 100);				// delay a little to allow the averaging filters to accumulate data from the new source
	}

	ZProbe params = platform.GetCurrentZProbeParameters();
	gb.TryGetFValue('H', params.diveHeight, seenParam);		// dive height

	if (gb.Seen('F'))		// feed rate i.e. probing speed
	{
		params.probeSpeed = gb.GetFValue() * SecondsToMinutes;
		seenParam = true;
	}

	if (gb.Seen('T'))		// travel speed to probe point
	{
		params.travelSpeed = gb.GetFValue() * SecondsToMinutes;
		seenParam = true;
	}

	if (gb.Seen('I'))
	{
		params.invertReading = (gb.GetIValue() != 0);
		seenParam = true;
	}

	if (gb.Seen('B'))
	{
		params.turnHeatersOff = (gb.GetIValue() == 1);
		seenParam = true;
	}

	gb.TryGetFValue('R', params.recoveryTime, seenParam);	// Z probe recovery time
	gb.TryGetFValue('S', params.tolerance, seenParam);		// tolerance when multi-tapping

	if (gb.Seen('A'))
	{
		params.maxTaps = gb.GetUIValue();
		seenParam = true;
	}

	if (seenParam)
	{
		platform.SetZProbeParameters(platform.GetZProbeType(), params);
	}

	if (!(seenType || seenParam))
	{
		reply.printf("Z Probe type %u, invert %s, dive height %.1fmm, probe speed %dmm/min, travel speed %dmm/min, recovery time %.2f sec, heaters %s, max taps %u, max diff %.2f",
						(unsigned int)platform.GetZProbeType(), (params.invertReading) ? "yes" : "no", (double)params.diveHeight,
						(int)(params.probeSpeed * MinutesToSeconds), (int)(params.travelSpeed * MinutesToSeconds),
						(double)params.recoveryTime,
						(params.turnHeatersOff) ? "suspended" : "normal",
						params.maxTaps, (double)params.tolerance);
	}
	return GCodeResult::ok;
}

// Handle M581 and M582
GCodeResult GCodes::CheckOrConfigureTrigger(GCodeBuffer& gb, const StringRef& reply, int code)
{
	if (gb.Seen('T'))
	{
		unsigned int triggerNumber = gb.GetIValue();
		if (triggerNumber < MaxTriggers)
		{
			if (code == 582)
			{
				uint32_t states = platform.GetAllEndstopStates();
				if ((triggers[triggerNumber].rising & states) != 0 || (triggers[triggerNumber].falling & ~states) != 0)
				{
					SetBit(triggersPending, triggerNumber);
				}
			}
			else
			{
				bool seen = false;
				if (gb.Seen('C'))
				{
					seen = true;
					triggers[triggerNumber].condition = gb.GetIValue();
				}
				else if (triggers[triggerNumber].IsUnused())
				{
					triggers[triggerNumber].condition = 0;		// this is a new trigger, so set no condition
				}
				if (gb.Seen('S'))
				{
					seen = true;
					int sval = gb.GetIValue();
					TriggerInputsBitmap triggerMask = 0;
					for (size_t axis = 0; axis < numTotalAxes; ++axis)
					{
						if (gb.Seen(axisLetters[axis]))
						{
							SetBit(triggerMask, axis);
						}
					}
					if (gb.Seen(extrudeLetter))
					{
						uint32_t eStops[MaxExtruders];
						size_t numEntries = MaxExtruders;
						gb.GetUnsignedArray(eStops, numEntries, false);
						for (size_t i = 0; i < numEntries; ++i)
						{
							if (eStops[i] < MaxExtruders)
							{
								SetBit(triggerMask, eStops[i] + E0_AXIS);
							}
						}
					}
					switch(sval)
					{
					case -1:
						if (triggerMask == 0)
						{
							triggers[triggerNumber].rising = triggers[triggerNumber].falling = 0;
						}
						else
						{
							triggers[triggerNumber].rising &= (~triggerMask);
							triggers[triggerNumber].falling &= (~triggerMask);
						}
						break;

					case 0:
						triggers[triggerNumber].falling |= triggerMask;
						break;

					case 1:
						triggers[triggerNumber].rising |= triggerMask;
						break;

					default:
						platform.Message(ErrorMessage, "Bad S parameter in M581 command\n");
					}
				}
				if (!seen)
				{
					reply.printf("Trigger %u fires on a rising edge on ", triggerNumber);
					ListTriggers(reply, triggers[triggerNumber].rising);
					reply.cat(" or a falling edge on ");
					ListTriggers(reply, triggers[triggerNumber].falling);
					reply.cat(" endstop inputs");
					if (triggers[triggerNumber].condition == 1)
					{
						reply.cat(" when printing from SD card");
					}
				}
			}
			return GCodeResult::ok;
		}
		else
		{
			reply.copy("Trigger number out of range");
			return GCodeResult::error;
		}
	}

	reply.copy("Missing T parameter");
	return GCodeResult::error;
}

// Deal with a M584
GCodeResult GCodes::DoDriveMapping(GCodeBuffer& gb, const StringRef& reply)
{
	if (!LockMovementAndWaitForStandstill(gb))				// we also rely on this to retrieve the current motor positions to moveBuffer
	{
		return GCodeResult::notFinished;
	}

	bool seen = false, badDrive = false;
	const char *lettersToTry = "XYZUVWABC";
	char c;
	while ((c = *lettersToTry) != 0)
	{
		if (gb.Seen(c))
		{
			// Found an axis letter. Get the drivers to assign to this axis.
			seen = true;
			size_t numValues = MaxDriversPerAxis;
			uint32_t drivers[MaxDriversPerAxis];
			gb.GetUnsignedArray(drivers, numValues, false);

			// Check all the driver numbers are in range
			AxisDriversConfig config;
			config.numDrivers = numValues;
			for (size_t i = 0; i < numValues; ++i)
			{
				if (drivers[i] >= DRIVES)
				{
					badDrive = true;
				}
				else
				{
					config.driverNumbers[i] = (uint8_t)drivers[i];
				}
			}

			if (badDrive)
			{
				break;
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
					axisLetters[drive] = c;						// assign the drive to this drive letter
					moveBuffer.coords[drive] = 0.0;				// user has defined a new axis, so set its position
					currentUserPosition[drive] = 0.0;			// set its requested user position too in case it is visible
					++numTotalAxes;
					numVisibleAxes = numTotalAxes;				// assume any new axes are visible unless there is a P parameter
				}
				reprap.GetMove().SetNewPosition(moveBuffer.coords, true);	// tell the Move system where any new axes are
				platform.SetAxisDriversConfig(drive, config);
				if (numTotalAxes + numExtruders > DRIVES)
				{
					numExtruders = DRIVES - numTotalAxes;		// if we added axes, we may have fewer extruders now
				}
			}
		}
		++lettersToTry;
	}

	if (gb.Seen(extrudeLetter))
	{
		seen = true;
		size_t numValues = DRIVES - numTotalAxes;
		uint32_t drivers[MaxExtruders];
		gb.GetUnsignedArray(drivers, numValues, false);
		numExtruders = numValues;
		for (size_t i = 0; i < numValues; ++i)
		{
			if (drivers[i] >= DRIVES)
			{
				badDrive = true;
			}
			else
			{
				platform.SetExtruderDriver(i, (uint8_t)drivers[i]);
			}
		}
	}

	if (badDrive)
	{
		reply.copy("Invalid driver number");
		return GCodeResult::error;
	}
	else
	{
		if (gb.Seen('P'))
		{
			seen = true;
			const int nva = gb.GetIValue();
			if (nva >= (int)MinAxes && (unsigned int)nva <= numTotalAxes)
			{
				numVisibleAxes = (size_t)nva;
			}
			else
			{
				reply.copy("Invalid number of visible axes");
				return GCodeResult::error;
			}
		}

		if (!seen)
		{
			reply.copy("Driver assignments:");
			for (size_t drive = 0; drive < numTotalAxes; ++ drive)
			{
				reply.cat(' ');
				const AxisDriversConfig& axisConfig = platform.GetAxisDriversConfig(drive);
				char c = axisLetters[drive];
				for (size_t i = 0; i < axisConfig.numDrivers; ++i)
				{
					reply.catf("%c%u", c, axisConfig.driverNumbers[i]);
					c = ':';
				}
			}
			reply.cat(' ');
			char c = extrudeLetter;
			for (size_t extruder = 0; extruder < numExtruders; ++extruder)
			{
				reply.catf("%c%u", c, platform.GetExtruderDriver(extruder));
				c = ':';
			}
			reply.catf(", %u axes visible", numVisibleAxes);
		}
	}

	return GCodeResult::ok;
}

// Deal with a M585
GCodeResult GCodes::ProbeTool(GCodeBuffer& gb, const StringRef& reply)
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

	for (size_t axis = 0; axis < numTotalAxes; axis++)
	{
		if (gb.Seen(axisLetters[axis]))
		{
			// Get parameters first and check them
			const int endStopToUse = gb.Seen('E') ? gb.GetIValue() : -1;
			if (endStopToUse > (int)DRIVES)
			{
				reply.copy("Invalid endstop number");
				return GCodeResult::error;
				break;
			}

			// Save the current axis coordinates
			memcpy(toolChangeRestorePoint.moveCoords, currentUserPosition, ARRAY_SIZE(currentUserPosition) * sizeof(currentUserPosition[0]));

			// Prepare another move similar to G1 .. S3
			moveBuffer.moveType = 3;
			if (endStopToUse < 0)
			{
				moveBuffer.endStopsToCheck = 0;
				SetBit(moveBuffer.endStopsToCheck, axis);
			}
			else
			{
				moveBuffer.endStopsToCheck = UseSpecialEndstop;
				SetBit(moveBuffer.endStopsToCheck, endStopToUse);
			}
			moveBuffer.xAxes = DefaultXAxisMapping;
			moveBuffer.yAxes = DefaultYAxisMapping;
			moveBuffer.usePressureAdvance = false;
			moveBuffer.filePos = noFilePosition;
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

			// Zero every extruder drive
			for (size_t drive = numTotalAxes; drive < DRIVES; drive++)
			{
				moveBuffer.coords[drive] = 0.0;
			}
			moveBuffer.hasExtrusion = false;

			// Deal with feed rate
			if (gb.Seen(feedrateLetter))
			{
				const float rate = gb.GetFValue() * distanceScale;
				gb.MachineState().feedRate = rate * SecondsToMinutes;	// don't apply the speed factor to homing and other special moves
			}
			moveBuffer.feedRate = gb.MachineState().feedRate;

			// Kick off new movement
			NewMoveAvailable(1);
			gb.SetState(GCodeState::probingToolOffset);
		}
	}

	return GCodeResult::ok;
}

// Deal with a M905
GCodeResult GCodes::SetDateTime(GCodeBuffer& gb, const StringRef& reply)
{
	const time_t now = platform.GetDateTime();
	struct tm timeInfo;
	gmtime_r(&now, &timeInfo);
	bool seen = false;

	if (gb.Seen('P'))
	{
		seen = true;

		// Set date
		String<12> dateString;
		gb.GetPossiblyQuotedString(dateString.GetRef());
		if (strptime(dateString.c_str(), "%Y-%m-%d", &timeInfo) == nullptr)
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
		if (strptime(timeString.c_str(), "%H:%M:%S", &timeInfo) == nullptr)
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
		if ((firmwareUpdateModuleMap & 1) != 0 && !platform.CheckFirmwareUpdatePrerequisites(reply))
		{
			firmwareUpdateModuleMap = 0;
			return GCodeResult::error;
		}
	}

	// If we get here then we have the module map, and all prerequisites are satisfied
	isFlashing = true;										// this tells the web interface and PanelDue that we are about to flash firmware
	if (DoDwellTime(gb, 1000) == GCodeResult::notFinished)	// wait a second so all HTTP clients and PanelDue are notified
	{
		return GCodeResult::notFinished;
	}

	gb.SetState(GCodeState::flashing1);
	return GCodeResult::ok;
}

// Handle M260
GCodeResult GCodes::SendI2c(GCodeBuffer& gb, const StringRef &reply)
{
#if defined(I2C_IFACE)
	if (gb.Seen('A'))
	{
		uint32_t address = gb.GetUIValue();
		if (gb.Seen('B'))
		{
			int32_t values[MaxI2cBytes];
			size_t numValues = MaxI2cBytes;
			gb.GetIntArray(values, numValues, false);
			if (numValues != 0)
			{
				platform.InitI2c();
				I2C_IFACE.beginTransmission((int)address);
				for (size_t i = 0; i < numValues; ++i)
				{
					I2C_IFACE.write((uint8_t)values[i]);
				}
				if (I2C_IFACE.endTransmission() == 0)
				{
					return GCodeResult::ok;
				}
				reply.copy("I2C transmission error");
				return GCodeResult::error;
			}
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
		uint32_t address = gb.GetUIValue();
		if (gb.Seen('B'))
		{
			uint32_t numBytes = gb.GetUIValue();
			if (numBytes > 0 && numBytes <= MaxI2cBytes)
			{
				platform.InitI2c();
				I2C_IFACE.requestFrom(address, numBytes);
				reply.copy("Received");
				const uint32_t now = millis();
				do
				{
					if (I2C_IFACE.available() != 0)
					{
						const unsigned int b = I2C_IFACE.read() & 0x00FF;
						reply.catf(" %02x", b);
						--numBytes;
					}
				} while (numBytes != 0 && now - millis() < 3);

				return GCodeResult::ok;
			}
		}
	}

	return GCodeResult::badOrMissingParameter;
#else
	reply.copy("I2C not available");
	return GCodeResult::error;
#endif
}

// End
