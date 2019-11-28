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

#include "Wire.h"

// Set or print the Z probe. Called by G31.
// Note that G31 P or G31 P0 prints the parameters of the currently-selected Z probe.
GCodeResult GCodes::SetPrintZProbe(GCodeBuffer& gb, const StringRef& reply)
{
	ZProbeType probeType;
	bool seenT = false;
	if (gb.Seen('T'))
	{
		const unsigned int tp = gb.GetUIValue();
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
	if (gb.Seen('P'))
	{
		seen = true;
		params.adcValue = gb.GetIValue();
	}

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
		if (gb.MachineState().runningM501)
		{
			params.saveToConfigOverride = true;			// we are loading these parameters from config-override.g, so a subsequent M500 should save them to config-override.g
		}
		platform.SetZProbeParameters(probeType, params);
	}
	else
	{
		if (seenT)
		{
			reply.printf("Probe type %u:", (unsigned int)probeType);
		}
		else
		{
			const int v0 = platform.GetZProbeReading();
			int v1, v2;
			switch (platform.GetZProbeSecondaryValues(v1, v2))
			{
			case 1:
				reply.printf("Current reading %d (%d)", v0, v1);
				break;
			case 2:
				reply.printf("Current reading %d (%d, %d)", v0, v1, v2);
				break;
			default:
				reply.printf("Current reading %d", v0);
				break;
			}
		}
		reply.catf(", threshold %d, trigger height %.2f, offsets X%.1f Y%.1f", params.adcValue, (double)params.triggerHeight, (double)params.xOffset, (double)params.yOffset);
	}
	return GCodeResult::ok;
}

// Deal with G60
GCodeResult GCodes::SavePosition(GCodeBuffer& gb, const StringRef& reply)
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
			currentUserPosition[axis] = gb.ConvertDistance(axisValue);
		}
	}

	// Handle any E parameter in the G92 command
	if (gb.Seen(extrudeLetter))
	{
		virtualExtruderPosition = gb.GetDistance();
	}

	if (axesIncluded != 0)
	{
		ToolOffsetTransform(currentUserPosition, moveBuffer.coords);
		if (reprap.GetMove().GetKinematics().LimitPosition(moveBuffer.coords, nullptr, numVisibleAxes, LowestNBits<AxesBitmap>(numVisibleAxes), false, limitAxes)
			!= LimitPositionResult::ok												// pretend that all axes are homed
		   )
		{
			ToolOffsetInverseTransform(moveBuffer.coords, currentUserPosition);		// make sure the limits are reflected in the user position
		}
		reprap.GetMove().SetNewPosition(moveBuffer.coords, true);
		axesHomed |= reprap.GetMove().GetKinematics().AxesAssumedHomed(axesIncluded);
		if (IsBitSet(axesIncluded, Z_AXIS))
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
#if SUPPORT_WORKPLACE_COORDINATES
			workplaceCoordinates[currentCoordinateSystem][axis]
#else
			axisOffsets[axis]
#endif
						 = -gb.GetDistance();
			seen = true;
		}
	}

	if (!seen)
	{
		reply.printf("Axis offsets:");
		for (size_t axis = 0; axis < numVisibleAxes; axis++)
		{
			reply.catf(" %c%.2f", axisLetters[axis],
#if SUPPORT_WORKPLACE_COORDINATES
				-(double)(gb.InverseConvertDistance(workplaceCoordinates[0][axis]))
#else
				-(double)(gb.InverseConvertDistance(axisOffsets[axis]))
#endif
													 );
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

		if (!seen)
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

// Save all the workplace coordinate offsets to file returning true if successful. Used by M500 and by SaveResumeInfo.
bool GCodes::WriteWorkplaceCoordinates(FileStore *f) const
{
	for (size_t cs = 0; cs < NumCoordinateSystems; ++cs)
	{
		String<ScratchStringLength> scratchString;
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
		reply.copy("specify both or neither of X and Y in M577");
		return GCodeResult::error;
	}

	if (!seenX && !seenR)
	{
		// Must have given just the S or P parameter
		reply.copy("specify at least radius or X and Y ranges in M577");
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
			SavePosition(simulationRestorePoint, gb);
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

// Handle M558
GCodeResult GCodes::SetOrReportZProbe(GCodeBuffer& gb, const StringRef &reply)
{
	bool seen = false;

	// We must get and set the Z probe type first before setting the dive height etc., because different probe types may have different parameters
	uint32_t requestedChannel = 0;
	if (gb.Seen('P'))		// probe type
	{
		seen = true;
		uint32_t requestedType = gb.GetUIValue();
		switch (requestedType)
		{
		case (uint32_t)ZProbeType::endstopSwitch:
			requestedChannel = E0_AXIS;						// default channel if no C parameter present
			break;

		case (uint32_t)ZProbeType::e1Switch_obsolete:
			requestedChannel = E0_AXIS + 1;
			requestedType = (uint32_t)ZProbeType::endstopSwitch;
			break;

		case (uint32_t)ZProbeType::zSwitch_obsolete:
			requestedChannel = Z_AXIS;
			requestedType = (uint32_t)ZProbeType::endstopSwitch;
			break;

		default:
			break;
		}
		platform.SetZProbeType(requestedType);
		DoDwellTime(gb, 100);								// delay a little to allow the averaging filters to accumulate data from the new source
	}

	// Do the input channel next so that 'seen' will be true only if the type and/or the channel has been specified
	if (gb.Seen('C'))										// input channel
	{
		requestedChannel = gb.GetUIValue();
		seen = true;
	}

	ZProbe params = platform.GetCurrentZProbeParameters();
	if (seen)												// if seen P and/or C
	{
		params.inputChannel = requestedChannel;				// set the input to the default one for this type or the requested one
	}

	gb.TryGetFValue('H', params.diveHeight, seen);			// dive height
	if (gb.Seen('F'))										// feed rate i.e. probing speed
	{
		params.probeSpeed = gb.GetFValue() * SecondsToMinutes;
		seen = true;
	}

	if (gb.Seen('T'))		// travel speed to probe point
	{
		params.travelSpeed = gb.GetFValue() * SecondsToMinutes;
		seen = true;
	}

	if (gb.Seen('I'))
	{
		params.invertReading = (gb.GetIValue() != 0);
		seen = true;
	}

	if (gb.Seen('B'))
	{
		params.turnHeatersOff = (gb.GetIValue() == 1);
		seen = true;
	}

	gb.TryGetFValue('R', params.recoveryTime, seen);		// Z probe recovery time
	gb.TryGetFValue('S', params.tolerance, seen);			// tolerance when multi-tapping

	if (gb.Seen('A'))
	{
		params.maxTaps = min<uint32_t>(gb.GetUIValue(), ZProbe::MaxTapsLimit);
		seen = true;
	}

	if (seen)
	{
		platform.SetZProbeParameters(platform.GetZProbeType(), params);
	}
	else
	{
		reply.printf("Z Probe type %u, input %u, invert %s, dive height %.1fmm, probe speed %dmm/min, travel speed %dmm/min, recovery time %.2f sec, heaters %s, max taps %u, max diff %.2f",
						(unsigned int)platform.GetZProbeType(), params.inputChannel, (params.invertReading) ? "yes" : "no", (double)params.diveHeight,
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

	bool seen = false;
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

			// Find the drive number allocated to this axis, or allocate a new one if necessary
			size_t drive = 0;
			while (drive < numTotalAxes && axisLetters[drive] != c)
			{
				++drive;
			}
			if (drive == numTotalAxes && drive < MaxAxes)
			{
				axisLetters[drive] = c;								// assign the drive to this drive letter
				++numTotalAxes;
				numVisibleAxes = numTotalAxes;						// assume any new axes are visible unless there is a P parameter
				float initialCoords[MaxAxes];
				reprap.GetMove().GetKinematics().GetAssumedInitialPosition(drive + 1, initialCoords);
				moveBuffer.coords[drive] = initialCoords[drive];	// user has defined a new axis, so set its position
				ToolOffsetInverseTransform(moveBuffer.coords, currentUserPosition);
			}
			platform.SetAxisDriversConfig(drive, numValues, drivers);
			if (numTotalAxes + numExtruders > MaxTotalDrivers)
			{
				numExtruders = MaxTotalDrivers - numTotalAxes;		// if we added axes, we may have fewer extruders now
			}
		}
		++lettersToTry;
	}

	if (gb.Seen(extrudeLetter))
	{
		seen = true;
		size_t numValues = MaxTotalDrivers - numTotalAxes;
		uint32_t drivers[MaxExtruders];
		gb.GetUnsignedArray(drivers, numValues, false);
		numExtruders = numValues;
		for (size_t i = 0; i < numValues; ++i)
		{
			platform.SetExtruderDriver(i, (uint8_t)min<uint32_t>(drivers[i], 255));
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
		// In the DDA ring, the axis positions for invisible non-moving axes are not always copied over from previous moves.
		// So if we have more visible axes than before, then we need to update their positions to get them in sync.
		// We could do this only when we increase the number of visible axes, but for simplicity we do it always.
		reprap.GetMove().SetNewPosition(moveBuffer.coords, true);		// tell the Move system where the axes are
	}
	else
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
		if (numExtruders != 0)
		{
			reply.cat(' ');
			char c = extrudeLetter;
			for (size_t extruder = 0; extruder < numExtruders; ++extruder)
			{
				reply.catf("%c%u", c, platform.GetExtruderDriver(extruder));
				c = ':';
			}
		}
		reply.catf(", %u axes visible", numVisibleAxes);
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
			if (endStopToUse > (int)NumEndstops)
			{
				reply.copy("Invalid endstop number");
				return GCodeResult::error;
				break;
			}

			// Save the current axis coordinates
			SavePosition(toolChangeRestorePoint, gb);

			// Prepare another move similar to G1 .. S3
			moveBuffer.SetDefaults(numVisibleAxes);
			moveBuffer.moveType = 3;
			moveBuffer.canPauseAfter = false;

			if (endStopToUse < 0)
			{
				moveBuffer.endStopsToCheck = 0;
				SetBit(moveBuffer.endStopsToCheck, axis);
			}
			else
			{
				moveBuffer.endStopsToCheck = UseSpecialEndstop;
				SetBit(moveBuffer.endStopsToCheck, endStopToUse);

				if (gb.Seen('L') && gb.GetIValue() == 0)
				{
					// By default custom endstops are active-high when triggered, so allow this to be inverted
					moveBuffer.endStopsToCheck |= ActiveLowEndstop;
				}
			}

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
			for (size_t drive = numTotalAxes; drive < MaxTotalDrivers; drive++)
			{
				moveBuffer.coords[drive] = 0.0;
			}
			moveBuffer.hasExtrusion = false;

			// Deal with feed rate
			if (gb.Seen(feedrateLetter))
			{
				const float rate = gb.GetDistance();
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

GCodeResult GCodes::FindCenterOfCavity(GCodeBuffer& gb, const StringRef& reply, const bool towardsMin)
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

	for (size_t axis = 0; axis < numVisibleAxes; axis++)
	{
		if (gb.Seen(axisLetters[axis]))
		{

			SetMoveBufferDefaults();

			// Prepare a move similar to G1 .. S3
			moveBuffer.moveType = 3;
			SetBit(moveBuffer.endStopsToCheck, axis);
			axesToSenseLength = 0;

			doingArcMove = false;

			moveBuffer.canPauseAfter = false;
			moveBuffer.hasExtrusion = false;

			moveBuffer.coords[axis] = towardsMin ? platform.AxisMinimum(axis) : platform.AxisMaximum(axis);

			// Deal with feed rate
			if (gb.Seen(feedrateLetter))
			{
				const float rate = gb.ConvertDistance(gb.GetFValue());
				gb.MachineState().feedRate = rate * SecondsToMinutes;	// don't apply the speed factor to homing and other special moves
			}
			else
			{
				reply.copy("No feed rate provided.");
				return GCodeResult::badOrMissingParameter;
			}
			moveBuffer.feedRate = gb.MachineState().feedRate;

			if (towardsMin)
			{
				gb.SetState(GCodeState::findCenterOfCavityMin);
			}
			else
			{
				gb.SetState(GCodeState::findCenterOfCavityMax);
			}

			// Kick off new movement
			NewMoveAvailable(1);

			// Only do one axis at a time
			break;
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
GCodeResult GCodes::ConfigureDriver(GCodeBuffer& gb,const  StringRef& reply)
{
	if (gb.Seen('P'))
	{
		const size_t drive = gb.GetIValue();
		if (drive < MaxTotalDrivers)
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
				if (gb.TryGetUIValue('H', val, seen))		// set microstep interval for changing from stealthChop to spreadCycle
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
					reply.copy("Expected 2 or 3 H values");
					return GCodeResult::error;
				}
			}
#endif
			if (!seen)
			{
				reply.printf("Drive %u runs %s, active %s enable, step timing ",
							drive,
							(platform.GetDirectionValue(drive)) ? "forwards" : "in reverse",
							(platform.GetEnableValue(drive)) ? "high" : "low");
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
#endif

#if SUPPORT_TMC22xx || SUPPORT_TMC51xx
					{
						const uint32_t tpwmthrs = SmartDrivers::GetRegister(drive, SmartDriverRegister::tpwmthrs);
						const uint32_t axis = SmartDrivers::GetAxisNumber(drive);
						bool bdummy;
						const float mmPerSec = (12000000.0 * platform.GetDriverMicrostepping(drive, bdummy))/(256 * tpwmthrs * platform.DriveStepsPerUnit(axis));
						reply.catf(", tpwmthrs %" PRIu32 " (%.1f mm/sec)", tpwmthrs, (double)mmPerSec);
					}
#endif

#if SUPPORT_TMC51xx
					{
						const uint32_t thigh = SmartDrivers::GetRegister(drive, SmartDriverRegister::thigh);
						const uint32_t axis = SmartDrivers::GetAxisNumber(drive);
						bool bdummy;
						const float mmPerSec = (12000000.0 * platform.GetDriverMicrostepping(drive, bdummy))/(256 * thigh * platform.DriveStepsPerUnit(axis));
						reply.catf(", thigh %" PRIu32 " (%.1f mm/sec)", thigh, (double)mmPerSec);
					}
#endif
				}
#endif
			}
		}
	}
	return GCodeResult::ok;
}

// Set the heater model (M307)
GCodeResult GCodes::SetHeaterModel(GCodeBuffer& gb, const StringRef& reply)
{
	if (gb.Seen('H'))
	{
		const unsigned int heater = gb.GetUIValue();
		if (heater < NumHeaters)
		{
			const FopDt& model = reprap.GetHeat().GetHeaterModel(heater);
			bool seen = false;
			float gain = model.GetGain(),
				tc = model.GetTimeConstant(),
				td = model.GetDeadTime(),
				maxPwm = model.GetMaxPwm(),
				voltage = model.GetVoltage();
			uint32_t freq = model.GetPwmFrequency();
			int32_t dontUsePid = model.UsePid() ? 0 : 1;
			int32_t inversionParameter = 0;

			gb.TryGetFValue('A', gain, seen);
			gb.TryGetFValue('C', tc, seen);
			gb.TryGetFValue('D', td, seen);
			gb.TryGetIValue('B', dontUsePid, seen);
			gb.TryGetFValue('S', maxPwm, seen);
			gb.TryGetFValue('V', voltage, seen);
			gb.TryGetIValue('I', inversionParameter, seen);
			gb.TryGetUIValue('F', freq, seen);

			if (seen)
			{
				const bool inverseTemperatureControl = (inversionParameter == 1 || inversionParameter == 3);
				if (!reprap.GetHeat().SetHeaterModel(heater, gain, tc, td, maxPwm, voltage,
														dontUsePid == 0, inverseTemperatureControl, (uint16_t)min<uint32_t>(freq, MaxHeaterPwmFrequency)))
				{
					reply.copy("bad model parameters");
					return GCodeResult::error;
				}

				const bool invertedPwmSignal = (inversionParameter == 2 || inversionParameter == 3);
				reprap.GetHeat().SetHeaterSignalInverted(heater, invertedPwmSignal);
			}
			else if (!model.IsEnabled())
			{
				reply.printf("Heater %u is disabled", heater);
			}
			else
			{
				const char* mode = (!model.UsePid()) ? "bang-bang"
									: (model.ArePidParametersOverridden()) ? "custom PID"
										: "PID";
				const bool pwmSignalInverted = reprap.GetHeat().IsHeaterSignalInverted(heater);
				const char* const inverted = model.IsInverted()
										? (pwmSignalInverted ? "PWM signal and temperature control" : "temperature control")
										: (pwmSignalInverted ? "PWM signal" : "no");

				reply.printf("Heater %u model: gain %.1f, time constant %.1f, dead time %.1f, max PWM %.2f, calibration voltage %.1f, mode %s, inverted %s, frequency ",
						heater, (double)model.GetGain(), (double)model.GetTimeConstant(), (double)model.GetDeadTime(), (double)model.GetMaxPwm(), (double)model.GetVoltage(), mode, inverted);
				if (model.GetPwmFrequency() == 0)
				{
					reply.cat("default");
				}
				else
				{
					reply.catf("%uHz", model.GetPwmFrequency());
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
		}
	}
	return GCodeResult::ok;
}

// Change a live extrusion factor
void GCodes::ChangeExtrusionFactor(unsigned int extruder, float factor)
{
	if (segmentsLeft != 0 && !moveBuffer.isFirmwareRetraction)
	{
		moveBuffer.coords[extruder + numTotalAxes] *= factor/extrusionFactors[extruder];	// last move not gone, so update it
	}
	extrusionFactors[extruder] = factor;
}

// End
