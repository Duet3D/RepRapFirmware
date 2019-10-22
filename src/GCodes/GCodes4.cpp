// State machine function for GCode class

#include "GCodes.h"
#include "GCodeBuffer/GCodeBuffer.h"
#include "RepRap.h"
#include "Movement/Move.h"
#include "Tools/Tool.h"
#include "Heating/Heat.h"
#include "Endstops/ZProbe.h"

#if HAS_WIFI_NETWORKING
# include "FirmwareUpdater.h"
#endif

// Execute a step of the state machine
void GCodes::RunStateMachine(GCodeBuffer& gb, const StringRef& reply)
{
#if HAS_LINUX_INTERFACE
	// Wait for the G-code replies and abort requests to go before anything else is done in the state machine
	if (reprap.UsingLinuxInterface() && (gb.IsAbortRequested() || gb.IsAbortAllRequested()))
	{
		return;
	}
#endif

	// Perform the next operation of the state machine for this gcode source
	bool error = false;

	switch (gb.GetState())
	{
	case GCodeState::waitingForSpecialMoveToComplete:
		if (LockMovementAndWaitForStandstill(gb))		// movement should already be locked, but we need to wait for standstill and fetch the current position
		{
			// Check whether we made any G1 S3 moves and need to set the axis limits
			for (size_t axis = 0; axis < numTotalAxes; ++axis)
			{
				if (IsBitSet<AxesBitmap>(axesToSenseLength, axis))
				{
					const EndStopPosition stopType = platform.GetEndstops().GetEndStopPosition(axis);
					if (stopType == EndStopPosition::highEndStop)
					{
						platform.SetAxisMaximum(axis, moveBuffer.coords[axis], true);
					}
					else if (stopType == EndStopPosition::lowEndStop)
					{
						platform.SetAxisMinimum(axis, moveBuffer.coords[axis], true);
					}
				}
			}

			if (gb.MachineState().compatibility == Compatibility::nanoDLP && !DoingFileMacro())
			{
				reply.copy("Z_move_comp");
			}
			gb.SetState(GCodeState::normal);
		}
		break;

	case GCodeState::waitingForSegmentedMoveToGo:
		// Wait for all segments of the arc move to go into the movement queue and check whether an error occurred
		switch (segMoveState)
		{
		case SegmentedMoveState::inactive:					// move completed without error
			gb.SetState(GCodeState::normal);
			break;

		case SegmentedMoveState::aborted:					// move terminated abnormally
			if (!LockMovementAndWaitForStandstill(gb))		// update the the user position from the machine position at which we stop
			{
				break;
			}
			reply.copy("G1/G2/G3: intermediate position outside machine limits");
			error = true;
			gb.SetState(GCodeState::normal);
			if (machineType != MachineType::fff)
			{
				AbortPrint(gb);
			}
			break;

		case SegmentedMoveState::active:					// move still ongoing
			break;
		}
		break;

	case GCodeState::probingToolOffset:
		if (LockMovementAndWaitForStandstill(gb))
		{
			Tool * const currentTool = reprap.GetCurrentTool();
			if (currentTool != nullptr)
			{
				for (size_t axis = 0; axis < numTotalAxes; ++axis)
				{
					if (gb.Seen(axisLetters[axis]))
					{
						// We get here when the tool probe has been activated. In this case we know how far we
						// went (i.e. the difference between our start and end positions) and if we need to
						// incorporate any correction factors. That's why we only need to set the final tool
						// offset to this value in order to finish the tool probing.
						const float coord = toolChangeRestorePoint.moveCoords[axis] - currentUserPosition[axis] + gb.GetFValue();
						currentTool->SetOffset(axis, coord, true);
						break;
					}
				}
			}
			gb.SetState(GCodeState::normal);
		}
		break;

	case GCodeState::findCenterOfCavityMin:
		if (LockMovementAndWaitForStandstill(gb))
		{
			// We're trying to find the center of the cavity and we've moved all the way back until the corresponding
			// endstop has been triggered. This means we can save the minimum position
			SavePosition(findCenterOfCavityRestorePoint, gb);

			// Move away from the endstop
			const float rVal = gb.Seen('R') ? gb.GetFValue() : 5.0;
			for (size_t axis = 0; axis < numVisibleAxes; ++axis)
			{
				if (gb.Seen(axisLetters[axis]))
				{
					moveBuffer.SetDefaults(numVisibleAxes);
					ToolOffsetTransform(currentUserPosition, moveBuffer.coords);
					moveBuffer.coords[axis] += rVal;					// add R to the current position

					moveBuffer.feedRate = findCenterOfCavityRestorePoint.feedRate;
					moveBuffer.canPauseAfter = false;

					NewMoveAvailable(1);
					break;
				}
			}
			gb.SetState(GCodeState::findCenterOfCavityR);
		}
		break;

	case GCodeState::findCenterOfCavityR:
		if (LockMovementAndWaitForStandstill(gb))
		{
			// Kick off another probing move to the axis maximum
			FindCenterOfCavity(gb, reply, false);
		}
		break;

	case GCodeState::findCenterOfCavityMax:
		if (LockMovementAndWaitForStandstill(gb))
		{
			// We get here when both the minimum and maximum values have been probed
			for (size_t axis = 0; axis < numVisibleAxes; ++axis)
			{
				if (gb.Seen(axisLetters[axis]))
				{
					// Get the current position of the axis to calculate center-point below
					float currentCoords[MaxAxesPlusExtruders];
					ToolOffsetTransform(currentUserPosition, currentCoords);

					moveBuffer.SetDefaults(numVisibleAxes);
					RestorePosition(findCenterOfCavityRestorePoint, &gb);
					ToolOffsetTransform(currentUserPosition, moveBuffer.coords);
					moveBuffer.coords[axis] += (currentCoords[axis] - moveBuffer.coords[axis]) / 2;

					NewMoveAvailable(1);
					gb.SetState(GCodeState::waitingForSpecialMoveToComplete);
					break;
				}
			}
		}
		break;

	case GCodeState::homing1:
		if (toBeHomed == 0)
		{
			gb.SetState(GCodeState::normal);
		}
		else
		{
			String<RepRapPasswordLength> nextHomingFileName;
			AxesBitmap mustHomeFirst = reprap.GetMove().GetKinematics().GetHomingFileName(toBeHomed, axesHomed, numVisibleAxes, nextHomingFileName.GetRef());
			if (mustHomeFirst != 0)
			{
				// Error, can't home this axes
				reply.copy("Must home these axes:");
				AppendAxes(reply, mustHomeFirst);
				reply.cat(" before homing these:");
				AppendAxes(reply, toBeHomed);
				error = true;
				toBeHomed = 0;
				gb.SetState(GCodeState::normal);
			}
			else
			{
				gb.SetState(GCodeState::homing2);
				if (!DoFileMacro(gb, nextHomingFileName.c_str(), false, 28))
				{
					reply.printf("Homing file %s not found", nextHomingFileName.c_str());
					error = true;
					gb.SetState(GCodeState::normal);
				}
			}
		}
		break;

	case GCodeState::homing2:
		if (LockMovementAndWaitForStandstill(gb))		// movement should already be locked, but we need to wait for the previous homing move to complete
		{
			// Test whether the previous homing move homed any axes
			if ((toBeHomed & axesHomed) == 0)
			{
				reply.copy("Homing failed");
				error = true;
				gb.SetState(GCodeState::normal);
			}
			else
			{
				toBeHomed &= ~axesHomed;
				gb.SetState((toBeHomed == 0) ? GCodeState::normal : GCodeState::homing1);
			}
		}
		break;

	case GCodeState::toolChange0: 		// Run tfree for the old tool (if any)
	case GCodeState::m109ToolChange0:	// Run tfree for the old tool (if any)
		doingToolChange = true;
		SaveFanSpeeds();
		SavePosition(toolChangeRestorePoint, gb);
		gb.AdvanceState();
		if ((gb.MachineState().toolChangeParam & TFreeBit) != 0)
		{
			const Tool * const oldTool = reprap.GetCurrentTool();
			if (oldTool != nullptr && AllAxesAreHomed())
			{
				String<ShortScratchStringLength> scratchString;
				scratchString.printf("tfree%d.g", oldTool->Number());
				DoFileMacro(gb, scratchString.c_str(), false, 0);		// don't pass the T code here because it may be negative
			}
		}
		break;

	case GCodeState::toolChange1:		// Release the old tool (if any), then run tpre for the new tool
	case GCodeState::m109ToolChange1:	// Release the old tool (if any), then run tpre for the new tool
		if (LockMovementAndWaitForStandstill(gb))		// wait for tfree.g to finish executing
		{
			const Tool * const oldTool = reprap.GetCurrentTool();
			if (oldTool != nullptr)
			{
				reprap.StandbyTool(oldTool->Number(), simulationMode != 0);
			}
			gb.AdvanceState();
			if (reprap.GetTool(gb.MachineState().newToolNumber) != nullptr && AllAxesAreHomed() && (gb.MachineState().toolChangeParam & TPreBit) != 0)
			{
				String<ShortScratchStringLength> scratchString;
				scratchString.printf("tpre%d.g", gb.MachineState().newToolNumber);
				DoFileMacro(gb, scratchString.c_str(), false, 0);
			}
		}
		break;

	case GCodeState::toolChange2:		// Select the new tool (even if it doesn't exist - that just deselects all tools) and run tpost
	case GCodeState::m109ToolChange2:	// Select the new tool (even if it doesn't exist - that just deselects all tools) and run tpost
		if (LockMovementAndWaitForStandstill(gb))		// wait for tpre.g to finish executing
		{
			reprap.SelectTool(gb.MachineState().newToolNumber, simulationMode != 0);
			UpdateCurrentUserPosition();					// get the actual position of the new tool

			gb.AdvanceState();
			if (AllAxesAreHomed())
			{
				if (reprap.GetCurrentTool() != nullptr && (gb.MachineState().toolChangeParam & TPostBit) != 0)
				{
					String<ShortScratchStringLength> scratchString;
					scratchString.printf("tpost%d.g", gb.MachineState().newToolNumber);
					DoFileMacro(gb, scratchString.c_str(), false, 0);
				}
			}
		}
		break;

	case GCodeState::toolChangeComplete:
	case GCodeState::m109ToolChangeComplete:
		if (LockMovementAndWaitForStandstill(gb))		// wait for tpost.g to finish executing
		{
			// Restore the original Z axis user position, so that different tool Z offsets work even if the first move after the tool change doesn't have a Z coordinate
			// Only do this if we are running as an FDM printer, because it's not appropriate for CNC machines.
			if (machineType == MachineType::fff)
			{
				currentUserPosition[Z_AXIS] = toolChangeRestorePoint.moveCoords[Z_AXIS];
			}
			gb.MachineState().feedRate = toolChangeRestorePoint.feedRate;
			// We don't restore the default fan speed in case the user wants to use a different one for the new tool
			doingToolChange = false;

			if (gb.GetState() == GCodeState::toolChangeComplete)
			{
				gb.SetState(GCodeState::normal);
			}
			else
			{
				UnlockAll(gb);							// allow movement again
				gb.AdvanceState();
			}
		}
		break;

	case GCodeState::m109WaitForTemperature:
		if (cancelWait || simulationMode != 0 || ToolHeatersAtSetTemperatures(reprap.GetCurrentTool(), gb.MachineState().waitWhileCooling, TEMPERATURE_CLOSE_ENOUGH))
		{
			cancelWait = isWaiting = false;
			gb.SetState(GCodeState::normal);
		}
		else
		{
			CheckReportDue(gb, reply);
			isWaiting = true;
		}
		break;

	case GCodeState::pausing1:
		if (LockMovementAndWaitForStandstill(gb))
		{
			gb.AdvanceState();
			if (AllAxesAreHomed())
			{
				DoFileMacro(gb, PAUSE_G, true, 25);
			}
		}
		break;

	case GCodeState::filamentChangePause1:
		if (LockMovementAndWaitForStandstill(gb))
		{
			gb.AdvanceState();
			if (AllAxesAreHomed())
			{
				if (!DoFileMacro(gb, FILAMENT_CHANGE_G, false, 600))
				{
					DoFileMacro(gb, PAUSE_G, true, 600);
				}
			}
		}
		break;

	case GCodeState::pausing2:
	case GCodeState::filamentChangePause2:
		if (LockMovementAndWaitForStandstill(gb))
		{
			reply.printf((gb.GetState() == GCodeState::filamentChangePause2) ? "Printing paused for filament change at" : "Printing paused at");
			for (size_t axis = 0; axis < numVisibleAxes; ++axis)
			{
				reply.catf(" %c%.1f", axisLetters[axis], (double)pauseRestorePoint.moveCoords[axis]);
			}
			platform.MessageF(LogMessage, "%s\n", reply.c_str());
			gb.SetState(GCodeState::normal);
		}
		break;

	case GCodeState::resuming1:
	case GCodeState::resuming2:
		// Here when we have just finished running the resume macro file.
		// Move the head back to the paused location
		if (LockMovementAndWaitForStandstill(gb))
		{
			float currentZ = moveBuffer.coords[Z_AXIS];
			for (size_t axis = 0; axis < numVisibleAxes; ++axis)
			{
				currentUserPosition[axis] = pauseRestorePoint.moveCoords[axis];
			}
			ToolOffsetTransform(currentUserPosition, moveBuffer.coords);
			SetMoveBufferDefaults();
			moveBuffer.feedRate = DefaultFeedRate * SecondsToMinutes;	// ask for a good feed rate, we may have paused during a slow move
			if (gb.GetState() == GCodeState::resuming1 && currentZ > pauseRestorePoint.moveCoords[Z_AXIS])
			{
				// First move the head to the correct XY point, then move it down in a separate move
				moveBuffer.coords[Z_AXIS] = currentZ;
				gb.SetState(GCodeState::resuming2);
			}
			else
			{
				// Just move to the saved position in one go
				gb.SetState(GCodeState::resuming3);
			}
			NewMoveAvailable(1);
		}
		break;

	case GCodeState::resuming3:
		if (LockMovementAndWaitForStandstill(gb))
		{
			for (size_t i = 0; i < NumTotalFans; ++i)
			{
				reprap.GetFansManager().SetFanValue(i, pausedFanSpeeds[i]);
			}
			virtualExtruderPosition = pauseRestorePoint.virtualExtruderPosition;	// reset the extruder position in case we are receiving absolute extruder moves
			moveBuffer.virtualExtruderPosition = pauseRestorePoint.virtualExtruderPosition;
			fileGCode->MachineState().feedRate = pauseRestorePoint.feedRate;
			moveFractionToSkip = pauseRestorePoint.proportionDone;
			isPaused = false;
			reply.copy("Printing resumed");
			platform.Message(LogMessage, "Printing resumed\n");
			gb.SetState(GCodeState::normal);
		}
		break;

	case GCodeState::flashing1:
#if HAS_WIFI_NETWORKING
		if (&gb == auxGCode)								// if M997 S1 is sent from USB, don't keep sending temperature reports
		{
			CheckReportDue(gb, reply);						// this is so that the ATE gets status reports and can tell when flashing is complete
		}

		// Update additional modules before the main firmware
		if (FirmwareUpdater::IsReady())
		{
			bool updating = false;
			for (unsigned int module = 1; module < NumFirmwareUpdateModules; ++module)
			{
				if ((firmwareUpdateModuleMap & (1u << module)) != 0)
				{
					firmwareUpdateModuleMap &= ~(1u << module);
					FirmwareUpdater::UpdateModule(module);
					updating = true;
					break;
				}
			}
			if (!updating)
			{
				gb.SetState(GCodeState::flashing2);
			}
		}
#else
		gb.SetState(GCodeState::flashing2);
#endif
		break;

	case GCodeState::flashing2:
		if ((firmwareUpdateModuleMap & 1) != 0)
		{
			// Update main firmware
			firmwareUpdateModuleMap = 0;
			platform.UpdateFirmware();
			// The above call does not return unless an error occurred
		}
		isFlashing = false;
		gb.SetState(GCodeState::normal);
		break;

	case GCodeState::stoppingWithHeatersOff:	// MO or M1 after executing stop.g/sleep.g if present
		reprap.GetHeat().SwitchOffAll(true);
		// no break

	case GCodeState::stoppingWithHeatersOn:		// M0 H1 or M1 H1 after executing stop.g/sleep.g if present
		if (LockMovementAndWaitForStandstill(gb))
		{
			platform.SetDriversIdle();
			gb.SetState(GCodeState::normal);
		}
		break;

	// States used for grid probing
	case GCodeState::gridProbing1:		// ready to move to next grid probe point
		{
			// Move to the current probe point
			Move& move = reprap.GetMove();
			const GridDefinition& grid = move.AccessHeightMap().GetGrid();
			const float x = grid.GetXCoordinate(gridXindex);
			const float y = grid.GetYCoordinate(gridYindex);
			if (grid.IsInRadius(x, y))
			{
				if (move.IsAccessibleProbePoint(x, y))
				{
					SetMoveBufferDefaults();
					const ZProbe& zp = platform.GetCurrentZProbe();
					moveBuffer.coords[X_AXIS] = x - zp.GetXOffset();
					moveBuffer.coords[Y_AXIS] = y - zp.GetYOffset();
					moveBuffer.coords[Z_AXIS] = zp.GetStartingHeight();
					moveBuffer.feedRate = zp.GetTravelSpeed();
					NewMoveAvailable(1);

					tapsDone = 0;
					g30zHeightErrorSum = 0.0;
					g30zHeightErrorLowestDiff = 1000.0;

					gb.AdvanceState();
				}
				else
				{
					platform.MessageF(WarningMessage, "Skipping grid point (%.1f, %.1f) because Z probe cannot reach it\n", (double)x, (double)y);
					gb.SetState(GCodeState::gridProbing6);
				}
			}
			else
			{
				gb.SetState(GCodeState::gridProbing6);
			}
		}
		break;

	case GCodeState::gridProbing2a:		// ready to probe the current grid probe point (we return to this state when doing the second and subsequent taps)
		if (LockMovementAndWaitForStandstill(gb))
		{
			gb.AdvanceState();
			if (platform.GetCurrentZProbeType() == ZProbeType::blTouch)
			{
				DoFileMacro(gb, DEPLOYPROBE_G, false, 29);				// bltouch needs to be redeployed prior to each probe point
			}
		}
		break;

	case GCodeState::gridProbing2b:		// ready to probe the current grid probe point
		if (LockMovementAndWaitForStandstill(gb))
		{
			lastProbedTime = millis();
			const ZProbe& zp = platform.GetCurrentZProbe();
			if (zp.GetProbeType() != ZProbeType::none && zp.GetTurnHeatersOff())
			{
				reprap.GetHeat().SuspendHeaters(true);
			}
			gb.AdvanceState();
		}
		break;

	case GCodeState::gridProbing3:	// ready to probe the current grid probe point
		{
			const ZProbe& zp = platform.GetCurrentZProbe();
			if (millis() - lastProbedTime >= (uint32_t)(zp.GetRecoveryTime() * SecondsToMillis))
			{
				// Probe the bed at the current XY coordinates
				// Check for probe already triggered at start
				if (zp.GetProbeType() == ZProbeType::none)
				{
					// No Z probe, so do manual mesh levelling instead
					UnlockAll(gb);															// release the movement lock to allow manual Z moves
					gb.AdvanceState();														// resume at next state when user has finished adjusting the height
					doingManualBedProbe = true;												// suspend the Z movement limit
					DoManualBedProbe(gb);
				}
				else if (platform.GetCurrentZProbe().Stopped() == EndStopHit::atStop)
				{
					reprap.GetHeat().SuspendHeaters(false);
					platform.Message(ErrorMessage, "Z probe already triggered before probing move started");
					gb.SetState(GCodeState::normal);
					if (zp.GetProbeType() != ZProbeType::none && !probeIsDeployed)
					{
						DoFileMacro(gb, RETRACTPROBE_G, false, 29);
					}
					break;
				}
				else
				{
					zProbeTriggered = false;
					SetMoveBufferDefaults();
					platform.GetCurrentZProbe().SetProbing(true);
					platform.GetEndstops().EnableCurrentZProbe();
					moveBuffer.checkEndstops = true;
					moveBuffer.reduceAcceleration = true;
					moveBuffer.coords[Z_AXIS] = -zp.GetDiveHeight() + zp.GetActualTriggerHeight();
					moveBuffer.feedRate = zp.GetProbingSpeed();
					NewMoveAvailable(1);
					gb.AdvanceState();
				}
			}
		}
		break;

	case GCodeState::gridProbing4:	// ready to lift the probe after probing the current grid probe point
		if (LockMovementAndWaitForStandstill(gb))
		{
			doingManualBedProbe = false;
			++tapsDone;
			reprap.GetHeat().SuspendHeaters(false);
			const ZProbe& zp = platform.GetCurrentZProbe();
			if (zp.GetProbeType() == ZProbeType::none)
			{
				// No Z probe, so we are doing manual mesh levelling. Take the current Z height as the height error.
				g30zHeightError = moveBuffer.coords[Z_AXIS];
			}
			else
			{
				platform.GetCurrentZProbe().SetProbing(false);
				if (!zProbeTriggered)
				{
					platform.Message(ErrorMessage, "Z probe was not triggered during probing move\n");
					gb.SetState(GCodeState::normal);
					if (zp.GetProbeType() != ZProbeType::none && !probeIsDeployed)
					{
						DoFileMacro(gb, RETRACTPROBE_G, false, 29);
					}
					break;
				}

				g30zHeightError = moveBuffer.coords[Z_AXIS] - zp.GetActualTriggerHeight();
				g30zHeightErrorSum += g30zHeightError;
			}

			gb.AdvanceState();
			if (platform.GetCurrentZProbeType() == ZProbeType::blTouch)
			{
				DoFileMacro(gb, RETRACTPROBE_G, false, 29);			// bltouch needs to be retracted when it triggers
			}
		}
		break;

	case GCodeState::gridProbing4a:	// ready to lift the probe after probing the current grid probe point
		// Move back up to the dive height
		SetMoveBufferDefaults();
		{
			const ZProbe& zp = platform.GetCurrentZProbe();
			moveBuffer.coords[Z_AXIS] = zp.GetStartingHeight();
			moveBuffer.feedRate = zp.GetTravelSpeed();
		}
		NewMoveAvailable(1);
		gb.AdvanceState();
		break;

	case GCodeState::gridProbing5:	// finished probing a point and moved back to the dive height
		if (LockMovementAndWaitForStandstill(gb))
		{
			// See whether we need to do any more taps
			const ZProbe& params = platform.GetCurrentZProbe();
			bool acceptReading = false;
			if (params.GetMaxTaps() < 2)
			{
				acceptReading = true;
			}
			else if (tapsDone >= 2)
			{
				g30zHeightErrorLowestDiff = min<float>(g30zHeightErrorLowestDiff, fabsf(g30zHeightError - g30PrevHeightError));
				if (params.GetTolerance() > 0.0)
				{
					if (g30zHeightErrorLowestDiff <= params.GetTolerance())
					{
						g30zHeightError = (g30zHeightError + g30PrevHeightError)/2;
						acceptReading = true;
					}
				}
				else if (tapsDone == params.GetMaxTaps())
				{
					g30zHeightError = g30zHeightErrorSum/tapsDone;
					acceptReading = true;
				}
			}

			if (acceptReading)
			{
				reprap.GetMove().AccessHeightMap().SetGridHeight(gridXindex, gridYindex, g30zHeightError);
				gb.AdvanceState();
			}
			else if (tapsDone < params.GetMaxTaps())
			{
				// Tap again
				lastProbedTime = millis();
				g30PrevHeightError = g30zHeightError;
				gb.SetState(GCodeState::gridProbing2a);
			}
			else
			{
				platform.Message(ErrorMessage, "Z probe readings not consistent\n");
				gb.SetState(GCodeState::normal);
				if (params.GetProbeType() != ZProbeType::none && !probeIsDeployed)
				{
					DoFileMacro(gb, RETRACTPROBE_G, false, 29);
				}
			}
		}
		break;

	case GCodeState::gridProbing6:	// ready to compute the next probe point
		{
			const HeightMap& hm = reprap.GetMove().AccessHeightMap();
			if (gridYindex & 1)
			{
				// Odd row, so decreasing X
				if (gridXindex == 0)
				{
					++gridYindex;
				}
				else
				{
					--gridXindex;
				}
			}
			else
			{
				// Even row, so increasing X
				if (gridXindex + 1 == hm.GetGrid().NumXpoints())
				{
					++gridYindex;
				}
				else
				{
					++gridXindex;
				}
			}

			if (gridYindex == hm.GetGrid().NumYpoints())
			{
				// Done all the points
				gb.AdvanceState();
				if (platform.GetCurrentZProbeType() != ZProbeType::none && !probeIsDeployed)
				{
					DoFileMacro(gb, RETRACTPROBE_G, false, 29);
				}
			}
			else
			{
				gb.SetState(GCodeState::gridProbing1);
			}
		}
		break;

	case GCodeState::gridProbing7:
		// Finished probing the grid, and retracted the probe if necessary
		{
			float mean, deviation, minError, maxError;
			const uint32_t numPointsProbed = reprap.GetMove().AccessHeightMap().GetStatistics(mean, deviation, minError, maxError);
			if (numPointsProbed >= 4)
			{
				reply.printf("%" PRIu32 " points probed, min error %.3f, max error %.3f, mean %.3f, deviation %.3f\n",
								numPointsProbed, (double)minError, (double)maxError, (double)mean, (double)deviation);
#if HAS_MASS_STORAGE
# if HAS_LINUX_INTERFACE
				if (!reprap.UsingLinuxInterface())
				{
# endif
					error = TrySaveHeightMap(DefaultHeightMapFile, reply);
# if HAS_LINUX_INTERFACE
				}
# endif
#endif
				reprap.GetMove().AccessHeightMap().ExtrapolateMissing();
				reprap.GetMove().UseMesh(true);
				const float absMean = fabsf(mean);
				if (absMean >= 0.05 && absMean >= 2 * deviation)
				{
					platform.Message(WarningMessage, "the height map has a substantial Z offset. Suggest use Z-probe to establish Z=0 datum, then re-probe the mesh.\n");
				}
			}
			else
			{
				reply.copy("Too few points probed");
				error = true;
			}
		}
		if (!error)
		{
			reprap.GetPlatform().MessageF(LogMessage, "%s\n", reply.c_str());
		}
		gb.SetState(GCodeState::normal);
		break;

	// States used for G30 probing
	case GCodeState::probingAtPoint0:
		// Initial state when executing G30 with a P parameter. Start by moving to the dive height at the current position.
		SetMoveBufferDefaults();
		{
			const ZProbe& zp = platform.GetCurrentZProbe();
			moveBuffer.coords[Z_AXIS] = zp.GetStartingHeight();
			moveBuffer.feedRate = zp.GetTravelSpeed();
		}
		NewMoveAvailable(1);
		gb.AdvanceState();
		break;

	case GCodeState::probingAtPoint1:
		// The move to raise/lower the head to the correct dive height has been commanded.
		if (LockMovementAndWaitForStandstill(gb))
		{
			// Head is at the dive height but needs to be moved to the correct XY position. The XY coordinates have already been stored.
			SetMoveBufferDefaults();
			(void)reprap.GetMove().GetProbeCoordinates(g30ProbePointIndex, moveBuffer.coords[X_AXIS], moveBuffer.coords[Y_AXIS], true);
			const ZProbe& zp = platform.GetCurrentZProbe();
			moveBuffer.coords[Z_AXIS] = zp.GetStartingHeight();
			moveBuffer.feedRate = zp.GetTravelSpeed();
			NewMoveAvailable(1);

			InitialiseTaps();
			gb.AdvanceState();
		}
		break;

	case GCodeState::probingAtPoint2a:								// note we return to this state when doing the second and subsequent taps
		// Executing G30 with a P parameter. The move to put the head at the specified XY coordinates has been commanded.
		// OR initial state when executing G30 with no P parameter (must call InitialiseTaps first)
		if (LockMovementAndWaitForStandstill(gb))
		{
			gb.AdvanceState();
			if (platform.GetCurrentZProbeType() == ZProbeType::blTouch)
			{
				DoFileMacro(gb, DEPLOYPROBE_G, false, 30);			// bltouch needs to be redeployed prior to each probe point
			}
		}
		break;

	case GCodeState::probingAtPoint2b:
		if (LockMovementAndWaitForStandstill(gb))
		{
			// Head has finished moving to the correct XY position
			lastProbedTime = millis();								// start the probe recovery timer
			if (platform.GetCurrentZProbe().GetTurnHeatersOff())
			{
				reprap.GetHeat().SuspendHeaters(true);
			}
			gb.AdvanceState();
		}
		break;

	case GCodeState::probingAtPoint3:
		// Executing G30 with a P parameter. The move to put the head at the specified XY coordinates has been completed and the recovery timer started.
		// OR executing G30 without a P parameter, and the recovery timer has been started.
		if (millis() - lastProbedTime >= (uint32_t)(platform.GetCurrentZProbe().GetRecoveryTime() * SecondsToMillis))
		{
			// The probe recovery time has elapsed, so we can start the probing  move
			const ZProbe& zp = platform.GetCurrentZProbe();
			if (zp.GetProbeType() == ZProbeType::none)
			{
				// No Z probe, so we are doing manual 'probing'
				UnlockAll(gb);															// release the movement lock to allow manual Z moves
				gb.AdvanceState();														// resume at the next state when the user has finished
				doingManualBedProbe = true;												// suspend the Z movement limit
				DoManualBedProbe(gb);
			}
			else if (platform.GetCurrentZProbe().Stopped() == EndStopHit::atStop)		// check for probe already triggered at start
			{
				// Z probe is already triggered at the start of the move, so abandon the probe and record an error
				reprap.GetHeat().SuspendHeaters(false);
				platform.Message(ErrorMessage, "Z probe already triggered at start of probing move\n");
				if (g30ProbePointIndex >= 0)
				{
					reprap.GetMove().SetZBedProbePoint(g30ProbePointIndex, zp.GetDiveHeight(), true, true);
				}
				gb.SetState(GCodeState::normal);										// no point in doing anything else
				if (zp.GetProbeType() != ZProbeType::none && !probeIsDeployed)
				{
					DoFileMacro(gb, RETRACTPROBE_G, false, 30);
				}
			}
			else
			{
				zProbeTriggered = false;
				SetMoveBufferDefaults();
				platform.GetCurrentZProbe().SetProbing(true);
				platform.GetEndstops().EnableCurrentZProbe();
				moveBuffer.checkEndstops = true;
				moveBuffer.reduceAcceleration = true;
				moveBuffer.coords[Z_AXIS] = (IsAxisHomed(Z_AXIS))
											? platform.AxisMinimum(Z_AXIS) - zp.GetDiveHeight() + zp.GetActualTriggerHeight()	// Z axis has been homed, so no point in going very far
											: -1.1 * platform.AxisTotalLength(Z_AXIS);	// Z axis not homed yet, so treat this as a homing move
				moveBuffer.feedRate = zp.GetProbingSpeed();
				NewMoveAvailable(1);
				gb.AdvanceState();
			}
		}
		break;

	case GCodeState::probingAtPoint4:
		// Executing G30. The probe wasn't triggered at the start of the move, and the probing move has been commanded.
		if (LockMovementAndWaitForStandstill(gb))
		{
			// Probing move has stopped
			reprap.GetHeat().SuspendHeaters(false);
			doingManualBedProbe = false;
			hadProbingError = false;
			++tapsDone;
			const ZProbe& zp = platform.GetCurrentZProbe();
			if (zp.GetProbeType() == ZProbeType::none)
			{
				// No Z probe, so we are doing manual mesh levelling. Take the current Z height as the height error.
				g30zStoppedHeight = g30zHeightError = moveBuffer.coords[Z_AXIS];
			}
			else
			{
				platform.GetCurrentZProbe().SetProbing(false);
				if (!zProbeTriggered)
				{
					platform.Message(ErrorMessage, "Z probe was not triggered during probing move\n");
					g30zHeightErrorSum = g30zHeightError = 0.0;
					hadProbingError = true;
				}
				else
				{
					// Successful probing
					float m[MaxAxes];
					reprap.GetMove().GetCurrentMachinePosition(m, false);		// get height without bed compensation
					g30zStoppedHeight = m[Z_AXIS] - g30HValue;					// save for later
					g30zHeightError = g30zStoppedHeight - zp.GetActualTriggerHeight();
					g30zHeightErrorSum += g30zHeightError;
				}
			}

			if (g30ProbePointIndex < 0)											// if no P parameter
			{
				// Simple G30 probing move
				if (g30SValue == -1 || g30SValue == -2 || g30SValue == -3)
				{
					// G30 S-1 command taps once and reports the height, S-2 sets the tool offset to the negative of the current height, S-3 sets the Z probe trigger height
					gb.SetState(GCodeState::probingAtPoint7);					// special state for reporting the stopped height at the end
					if (zp.GetProbeType() != ZProbeType::none && !probeIsDeployed)
					{
						DoFileMacro(gb, RETRACTPROBE_G, false, 30);					// retract the probe before moving to the new state
					}
					break;
				}

				if (tapsDone == 1 && !hadProbingError)
				{
					// Reset the Z axis origin according to the height error so that we can move back up to the dive height
					moveBuffer.coords[Z_AXIS] = zp.GetActualTriggerHeight();
					reprap.GetMove().SetNewPosition(moveBuffer.coords, false);
					reprap.GetMove().SetZeroHeightError(moveBuffer.coords);
					g30zHeightErrorSum = g30zHeightError = 0;					// there is no longer any height error from this probe
					SetAxisIsHomed(Z_AXIS);										// this is only correct if the Z axis is Cartesian-like, but other architectures must be homed before probing anyway
					zDatumSetByProbing = true;
				}
			}

			gb.AdvanceState();
			if (zp.GetProbeType() == ZProbeType::blTouch)
			{
				DoFileMacro(gb, RETRACTPROBE_G, false, 30);							// bltouch needs to be retracted when it triggers
			}
		}
		break;

	case GCodeState::probingAtPoint4a:
		// Move back up to the dive height before we change anything, in particular before we adjust leadscrews
		SetMoveBufferDefaults();
		{
			const ZProbe& zp = platform.GetCurrentZProbe();
			moveBuffer.coords[Z_AXIS] = zp.GetStartingHeight();
			moveBuffer.feedRate = zp.GetTravelSpeed();
		}
		NewMoveAvailable(1);
		gb.AdvanceState();
		break;

	case GCodeState::probingAtPoint5:
		// Here when we have moved the head back up to the dive height
		if (LockMovementAndWaitForStandstill(gb))
		{
			// See whether we need to do any more taps
			const ZProbe& params = platform.GetCurrentZProbe();
			bool acceptReading = false;
			if (params.GetMaxTaps() < 2)
			{
				acceptReading = true;
			}
			else if (tapsDone >= 2)
			{
				g30zHeightErrorLowestDiff = min<float>(g30zHeightErrorLowestDiff, fabsf(g30zHeightError - g30PrevHeightError));
				if (params.GetTolerance() > 0.0 && g30zHeightErrorLowestDiff <= params.GetTolerance())
				{
					g30zHeightError = (g30zHeightError + g30PrevHeightError)/2;
					acceptReading = true;
				}
			}

			if (!acceptReading)
			{
				if (tapsDone < params.GetMaxTaps())
				{
					// Tap again
					g30PrevHeightError = g30zHeightError;
					lastProbedTime = millis();
					gb.SetState(GCodeState::probingAtPoint2a);
					break;
				}

				// We no longer flag this as a probing error, instead we take the average and issue a warning
				g30zHeightError = g30zHeightErrorSum/tapsDone;
				if (params.GetTolerance() > 0.0)			// zero or negative tolerance means always average all readings, so no warning message
				{
					platform.Message(WarningMessage, "Z probe readings not consistent\n");
				}
			}

			if (g30ProbePointIndex >= 0)
			{
				reprap.GetMove().SetZBedProbePoint(g30ProbePointIndex, g30zHeightError, true, hadProbingError);
			}
			else
			{
				// Setting the Z height with G30
				moveBuffer.coords[Z_AXIS] -= g30zHeightError;
				reprap.GetMove().SetNewPosition(moveBuffer.coords, false);
				reprap.GetMove().SetZeroHeightError(moveBuffer.coords);
				ToolOffsetInverseTransform(moveBuffer.coords, currentUserPosition);
			}
			gb.AdvanceState();
			if (platform.GetCurrentZProbeType() != ZProbeType::none && !probeIsDeployed)
			{
				DoFileMacro(gb, RETRACTPROBE_G, false, 30);
			}
		}
		break;

	case GCodeState::probingAtPoint6:
		// Here when we have finished probing with a P parameter and have retracted the probe if necessary
		if (LockMovementAndWaitForStandstill(gb))		// retracting the Z probe
		{
			if (g30SValue == 1)
			{
				// G30 with a silly Z value and S=1 is equivalent to G30 with no parameters in that it sets the current Z height
				// This is useful because it adjusts the XY position to account for the probe offset.
				moveBuffer.coords[Z_AXIS] -= g30zHeightError;
				reprap.GetMove().SetNewPosition(moveBuffer.coords, false);
				ToolOffsetInverseTransform(moveBuffer.coords, currentUserPosition);
			}
			else if (g30SValue >= -1)
			{
				error = reprap.GetMove().FinishedBedProbing(g30SValue, reply);
				if (!error && reprap.GetMove().GetKinematics().SupportsAutoCalibration())
				{
					zDatumSetByProbing = true;			// if we successfully auto calibrated or adjusted leadscrews, we've set the Z datum by probing
				}
			}
			gb.SetState(GCodeState::normal);
		}
		break;

	case GCodeState::probingAtPoint7:
		// Here when we have finished executing G30 S-1 or S-2 or S-3 including retracting the probe if necessary
		if (g30SValue == -3)
		{
			// Adjust the Z probe trigger height to the stop height
			ZProbe& zp = platform.GetCurrentZProbe();
			zp.SetTriggerHeight(g30zStoppedHeight);
			reply.printf("Z probe trigger height set to %.3f mm", (double)g30zStoppedHeight);
		}
		else if (g30SValue == -2)
		{
			// Adjust the Z offset of the current tool to account for the height error
			Tool * const tool = reprap.GetCurrentTool();
			if (tool == nullptr)
			{
				platform.Message(ErrorMessage, "Tool was deselected during G30 S-2 command\n");
				error = true;
			}
			else
			{
				tool->SetOffset(Z_AXIS, -g30zHeightError, true);
				ToolOffsetInverseTransform(moveBuffer.coords, currentUserPosition);		// update user coordinates to reflect the new tool offset
			}
		}
		else
		{
			// Just print the stop height
			reply.printf("Stopped at height %.3f mm", (double)g30zStoppedHeight);
		}
		gb.SetState(GCodeState::normal);
		break;


	case GCodeState::straightProbe0:		// ready to deploy the probe
		if (LockMovementAndWaitForStandstill(gb))
		{
			const StraightProbeSettings& sps = reprap.GetMove().GetStraightProbeSettings();
			const ZProbe& zp = *(platform.GetEndstops().GetZProbe(sps.GetZProbeToUse()));
			gb.AdvanceState();
			if (zp.GetProbeType() != ZProbeType::none && !probeIsDeployed)
			{
				DoFileMacro(gb, DEPLOYPROBE_G, false, 38);
			}
		}
		break;

	case GCodeState::straightProbe1:
		if (LockMovementAndWaitForStandstill(gb))
		{
			const StraightProbeSettings& sps = reprap.GetMove().GetStraightProbeSettings();
			const ZProbe& zp = *(platform.GetEndstops().GetZProbe(sps.GetZProbeToUse()));
			lastProbedTime = millis();			// start the probe recovery timer
			if (zp.GetTurnHeatersOff())
			{
				reprap.GetHeat().SuspendHeaters(true);
			}
			gb.AdvanceState();
		}
		break;

	case GCodeState::straightProbe2:
		// Executing G38. The probe has been deployed and the recovery timer has been started.
		if (millis() - lastProbedTime >= (uint32_t)(platform.GetCurrentZProbe().GetRecoveryTime() * SecondsToMillis))
		{
			// The probe recovery time has elapsed, so we can start the probing  move
			const StraightProbeSettings& sps = reprap.GetMove().GetStraightProbeSettings();
			const bool probingAway = sps.ProbingAway();
			const ZProbe& zp = *(platform.GetEndstops().GetZProbe(sps.GetZProbeToUse()));
			if (zp.GetProbeType() == ZProbeType::none)
			{
				// No Z probe, so we are doing manual 'probing'
				UnlockAll(gb);															// release the movement lock to allow manual Z moves
				gb.AdvanceState();														// resume at the next state when the user has finished

				String<MaxMessageLength> message;
				message.printf("Adjust postion until the reference point just %s the target, then press OK", probingAway ? "looses contact to" : "touches");
				DoManualProbe(gb, message.c_str(), "Manual Straight Probe", sps.GetMovingAxes());
			}
			else if ((!probingAway && zp.Stopped() == EndStopHit::atStop)
					|| (probingAway && zp.Stopped() != EndStopHit::atStop))		// check for probe already in target state at start
			{
				// Z probe is already in target state at the start of the move, so abandon the probe and signal an error if the type demands so
				reprap.GetHeat().SuspendHeaters(false);
				if (sps.SignalError())
				{
					platform.MessageF(ErrorMessage, "Probe %s triggered at start of probing move\n", probingAway ? "not" : "already");
					error = true;
				}
				gb.SetState(GCodeState::normal);										// no point in doing anything else
				if (zp.GetProbeType() != ZProbeType::none && !probeIsDeployed)
				{
					DoFileMacro(gb, RETRACTPROBE_G, false, 38);
				}
			}
			else
			{
				zProbeTriggered = false;
				SetMoveBufferDefaults();
				zp.SetProbing(true);
				platform.GetEndstops().EnableZProbe(sps.GetZProbeToUse(), probingAway);
				moveBuffer.checkEndstops = true;
				moveBuffer.reduceAcceleration = true;
				sps.SetCoordsToTarget(moveBuffer.coords);
				moveBuffer.feedRate = zp.GetProbingSpeed();
				NewMoveAvailable(1);
				gb.AdvanceState();
			}
		}
		break;

	case GCodeState::straightProbe3:
		// Executing G38. The probe wasn't in target state at the start of the move, and the probing move has been commanded.
		if (LockMovementAndWaitForStandstill(gb))
		{
			// Probing move has stopped
			reprap.GetHeat().SuspendHeaters(false);
			const StraightProbeSettings& sps = reprap.GetMove().GetStraightProbeSettings();
			const bool probingAway = sps.ProbingAway();
			const ZProbe& zp = *(platform.GetEndstops().GetZProbe(sps.GetZProbeToUse()));
			if (zp.GetProbeType() != ZProbeType::none)
			{
				zp.SetProbing(false);
				if (!zProbeTriggered && sps.SignalError())
				{
					platform.MessageF(ErrorMessage, "Z probe %s during probing move\n", probingAway ? "did not loose contact" : "was not triggered");
					error = true;
				}
			}

			gb.SetState(GCodeState::normal);
			if (zp.GetProbeType() != ZProbeType::none && !probeIsDeployed)
			{
				DoFileMacro(gb, RETRACTPROBE_G, false, 38);					// retract the probe before moving to the new state
			}
		}
		break;

	// Firmware retraction/un-retraction states
	case GCodeState::doingFirmwareRetraction:
		// We just did the retraction part of a firmware retraction, now we need to do the Z hop
		if (segmentsLeft == 0)
		{
			const AxesBitmap xAxes = reprap.GetCurrentXAxes();
			const AxesBitmap yAxes = reprap.GetCurrentYAxes();
			reprap.GetMove().GetCurrentUserPosition(moveBuffer.coords, 0, xAxes, yAxes);
			SetMoveBufferDefaults();
			moveBuffer.coords[Z_AXIS] += retractHop;
			moveBuffer.feedRate = platform.MaxFeedrate(Z_AXIS);
			moveBuffer.filePos = (&gb == fileGCode) ? gb.GetFilePosition() : noFilePosition;
			moveBuffer.canPauseAfter = false;			// don't pause after a retraction because that could cause too much retraction
			currentZHop = retractHop;
			NewMoveAvailable(1);
			gb.SetState(GCodeState::normal);
		}
		break;

	case GCodeState::doingFirmwareUnRetraction:
		// We just undid the Z-hop part of a firmware un-retraction, now we need to do the un-retract
		if (segmentsLeft == 0)
		{
			const Tool * const tool = reprap.GetCurrentTool();
			if (tool != nullptr)
			{
				const uint32_t xAxes = reprap.GetCurrentXAxes();
				const uint32_t yAxes = reprap.GetCurrentYAxes();
				reprap.GetMove().GetCurrentUserPosition(moveBuffer.coords, 0, xAxes, yAxes);
				SetMoveBufferDefaults();
				for (size_t i = 0; i < tool->DriveCount(); ++i)
				{
					moveBuffer.coords[MaxAxes + tool->Drive(i)] = retractLength + retractExtra;
				}
				moveBuffer.feedRate = unRetractSpeed;
				moveBuffer.isFirmwareRetraction = true;
				moveBuffer.filePos = (&gb == fileGCode) ? gb.GetFilePosition() : noFilePosition;
				moveBuffer.canPauseAfter = true;
				NewMoveAvailable(1);
			}
			gb.SetState(GCodeState::normal);
		}
		break;

	case GCodeState::loadingFilament:
		// We just returned from the filament load macro
		if (reprap.GetCurrentTool() != nullptr)
		{
			reprap.GetCurrentTool()->GetFilament()->Load(filamentToLoad);
			if (reprap.Debug(moduleGcodes))
			{
				platform.MessageF(LoggedGenericMessage, "Filament %s loaded", filamentToLoad);
			}
		}
		gb.SetState(GCodeState::normal);
		break;

	case GCodeState::unloadingFilament:
		// We just returned from the filament unload macro
		if (reprap.GetCurrentTool() != nullptr)
		{
			if (reprap.Debug(moduleGcodes))
			{
				platform.MessageF(LoggedGenericMessage, "Filament %s unloaded", reprap.GetCurrentTool()->GetFilament()->GetName());
			}
			reprap.GetCurrentTool()->GetFilament()->Unload();
		}
		gb.SetState(GCodeState::normal);
		break;

#if HAS_VOLTAGE_MONITOR
	case GCodeState::powerFailPausing1:
		if (gb.IsReady() || gb.IsExecuting())
		{
			gb.SetFinished(ActOnCode(gb, reply));							// execute the pause script
		}
		else
		{
# if HAS_MASS_STORAGE
			SaveResumeInfo(true);											// create the resume file so that we can resume after power down
# endif
			platform.Message(LoggedGenericMessage, "Print auto-paused due to low voltage\n");
			gb.SetState(GCodeState::normal);
		}
		break;
#endif

#if HAS_MASS_STORAGE
	case GCodeState::timingSDwrite:
		for (uint32_t writtenThisTime = 0; writtenThisTime < 100 * 1024; )
		{
			if (timingBytesWritten >= timingBytesRequested)
			{
				sdTimingFile->Close();
				const uint32_t ms = millis() - timingStartMillis;
				const float fileMbytes = (float)timingBytesWritten/(float)(1024 * 1024);
				const float mbPerSec = (fileMbytes * 1000.0)/(float)ms;
				reply.printf("SD write speed for %.1fMbyte file was %.2fMbytes/sec", (double)fileMbytes, (double)mbPerSec);
				platform.Delete(platform.GetGCodeDir(), TimingFileName);
				gb.SetState(GCodeState::normal);
				break;
			}

			if (!sdTimingFile->Write(reply.c_str(), reply.Capacity()))
			{
				sdTimingFile->Close();
				platform.Delete(platform.GetGCodeDir(), TimingFileName);
				reply.copy("Error writing to timing file");
				error = true;
				gb.SetState(GCodeState::normal);
				break;
			}
			timingBytesWritten += reply.Capacity();
			writtenThisTime += reply.Capacity();
		}
		break;
#endif

#if HAS_LINUX_INTERFACE
	case GCodeState::doingUnsupportedCode:
		//TODO
	case GCodeState::doingUserMacro:
		// We get here when a macro file has been cancelled via M99 or M292 P1
		gb.SetState(GCodeState::normal);
		break;
#endif

	default:				// should not happen
		platform.Message(ErrorMessage, "Undefined GCodeState\n");
		gb.SetState(GCodeState::normal);
		break;
	}

	if (gb.GetState() == GCodeState::normal)
	{
		// We completed a command, so unlock resources and tell the host about it
		gb.StopTimer();
		UnlockAll(gb);
		if (!error && gb.MachineState().errorMessage != nullptr)
		{
			reply.copy(gb.MachineState().errorMessage);
			error = true;
		}
		gb.MachineState().errorMessage = nullptr;
		HandleReply(gb, (error) ? GCodeResult::error : GCodeResult::ok, reply.c_str());
	}
}

// End
