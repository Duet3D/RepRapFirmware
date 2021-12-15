// State machine function for GCode class

#include "GCodes.h"
#include "GCodeBuffer/GCodeBuffer.h"
#include <Platform/RepRap.h>
#include <Platform/Event.h>
#include <Movement/Move.h>
#include <Tools/Tool.h>
#include <Heating/Heat.h>
#include <Endstops/ZProbe.h>

#if HAS_SBC_INTERFACE
# include <SBC/SbcInterface.h>
#endif

#if HAS_WIFI_NETWORKING || HAS_AUX_DEVICES
# include <Comms/FirmwareUpdater.h>
#endif

// Execute a step of the state machine
// CAUTION: don't allocate any long strings or other large objects directly within this function.
// The reason is that this function calls FinishedBedProbing(), which on a delta calls DoAutoCalibration(), which uses lots of stack.
// So any large local objects allocated here increase the amount of MAIN stack size needed.
void GCodes::RunStateMachine(GCodeBuffer& gb, const StringRef& reply) noexcept
{
#if HAS_SBC_INTERFACE
	// Wait for the G-code replies and abort requests to go before anything else is done in the state machine
	if (reprap.UsingSbcInterface() && (gb.IsAbortRequested() || gb.IsAbortAllRequested()))
	{
		return;
	}
	bool reportPause = false;
#endif

	// Perform the next operation of the state machine for this gcode source
	GCodeResult stateMachineResult = GCodeResult::ok;

	const GCodeState state = gb.GetState();
	switch (state)
	{
	case GCodeState::waitingForSpecialMoveToComplete:
	case GCodeState::abortWhenMovementFinished:
		if (LockMovementAndWaitForStandstill(gb))		// movement should already be locked, but we need to wait for standstill and fetch the current position
		{
			// Check whether we made any G1 S3 moves and need to set the axis limits
			axesToSenseLength.Iterate([this](unsigned int axis, unsigned int)
										{
											const EndStopPosition stopType = platform.GetEndstops().GetEndStopPosition(axis);
											if (stopType == EndStopPosition::highEndStop)
											{
												platform.SetAxisMaximum(axis, moveState.coords[axis], true);
											}
											else if (stopType == EndStopPosition::lowEndStop)
											{
												platform.SetAxisMinimum(axis, moveState.coords[axis], true);
											}
										}
									);

			if (gb.LatestMachineState().compatibility == Compatibility::NanoDLP && !DoingFileMacro())
			{
				reply.copy("Z_move_comp");
			}

			gb.SetState(GCodeState::normal);
			if (state == GCodeState::abortWhenMovementFinished)
			{
				AbortPrint(gb);
			}
		}
		break;

	case GCodeState::waitingForSegmentedMoveToGo:
		// Wait for all segments of the arc move to go into the movement queue and check whether an error occurred
		switch (moveState.segMoveState)
		{
		case SegmentedMoveState::inactive:					// move completed without error
			gb.SetState(GCodeState::normal);
			break;

		case SegmentedMoveState::aborted:					// move terminated abnormally
			if (!LockMovementAndWaitForStandstill(gb))		// update the the user position from the machine position at which we stop
			{
				break;
			}
			gb.LatestMachineState().SetError("G1/G2/G3: intermediate position outside machine limits");
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

	case GCodeState::probingToolOffset2:					// Z probe has been deployed and recovery timer is running
		{
			const auto zp = platform.GetZProbeOrDefault(currentZProbeNumber);
			if (millis() - lastProbedTime >= (uint32_t)(zp->GetRecoveryTime() * SecondsToMillis))
			{
				gb.AdvanceState();
			}
		}
		break;

	case GCodeState::probingToolOffset3:					// executing M585 with an endstop, or with a probe after deploying the probe and waiting for the recovery time
		if (SetupM585ProbingMove(gb))
		{
			gb.AdvanceState();
		}
		else
		{
			AbortPrint(gb);
			gb.SetState(GCodeState::checkError);
			if (m585Settings.useProbe)
			{
				reprap.GetHeat().SuspendHeaters(false);
				RetractZProbe(gb);
			}
		}
		break;

	case GCodeState::probingToolOffset4:					// executing M585, probing move has started
		if (LockMovementAndWaitForStandstill(gb))
		{
			if (m585Settings.useProbe)
			{
				const auto zp = platform.GetZProbeOrDefault(currentZProbeNumber);
				zp->SetProbing(false);
				reprap.GetHeat().SuspendHeaters(false);
				if (!zProbeTriggered)
				{
					gb.LatestMachineState().SetError("Probe was not triggered during probing move");
					gb.SetState(GCodeState::checkError);
					RetractZProbe(gb);
					break;
				}
			}

			Tool * const currentTool = reprap.GetCurrentTool();
			if (currentTool != nullptr)
			{
				// We get here when the tool probe has been activated. In this case we know how far we
				// went (i.e. the difference between our start and end positions) and if we need to
				// incorporate any correction factors. That's why we only need to set the final tool
				// offset to this value in order to finish the tool probing.
				const float coord = toolChangeRestorePoint.moveCoords[m585Settings.axisNumber] - moveState.currentUserPosition[m585Settings.axisNumber] + m585Settings.offset;
				currentTool->SetOffset(m585Settings.axisNumber, coord, true);
			}
			gb.SetState(GCodeState::normal);
			if (m585Settings.useProbe)
			{
				RetractZProbe(gb);
			}
		}
		break;


	case GCodeState::findCenterOfCavity1:						// Executing M675 using a Z probe, have already deployed the probe
	case GCodeState::probingToolOffset1:						// Executing M585 using a probe, which we have deployed
		if (LockMovementAndWaitForStandstill(gb))
		{
			lastProbedTime = millis();							// start the recovery timer
			const auto zp = platform.GetZProbeOrDefault(currentZProbeNumber);
			if (zp->GetProbeType() != ZProbeType::none && zp->GetTurnHeatersOff())
			{
				reprap.GetHeat().SuspendHeaters(true);
			}
			gb.AdvanceState();
		}
		break;

	case GCodeState::findCenterOfCavity2:						// Executing M675 using a Z probe, recovery timer is running
		{
			const auto zp = platform.GetZProbeOrDefault(currentZProbeNumber);
			if (millis() - lastProbedTime >= (uint32_t)(zp->GetRecoveryTime() * SecondsToMillis))
			{
				if (SetupM675ProbingMove(gb, true))
				{
					gb.AdvanceState();
				}
				else
				{
					AbortPrint(gb);
					gb.SetState(GCodeState::checkError);
					reprap.GetHeat().SuspendHeaters(false);
					RetractZProbe(gb);
				}
			}
		}
		break;

	case GCodeState::findCenterOfCavity3:						// Executing M675, min probing move has started
		if (LockMovementAndWaitForStandstill(gb))
		{
			const auto zp = platform.GetZProbeOrDefault(currentZProbeNumber);
			zp->SetProbing(false);
			if (zProbeTriggered)
			{
				m675Settings.minDistance = moveState.currentUserPosition[m675Settings.axisNumber];
				SetupM675BackoffMove(gb, m675Settings.minDistance + m675Settings.backoffDistance);
				gb.AdvanceState();
			}
			else
			{
				gb.LatestMachineState().SetError("Probe was not triggered during probing move");
				gb.SetState(GCodeState::checkError);
				reprap.GetHeat().SuspendHeaters(false);
				RetractZProbe(gb);
			}
		}
		break;

	case GCodeState::findCenterOfCavity4:						// Executing M675, backoff move from min has started
		if (LockMovementAndWaitForStandstill(gb))
		{
			if (SetupM675ProbingMove(gb, false))
			{
				gb.AdvanceState();
			}
			else
			{
				AbortPrint(gb);
				gb.SetState(GCodeState::checkError);
				reprap.GetHeat().SuspendHeaters(false);
				RetractZProbe(gb);
			}
		}
		break;

	case GCodeState::findCenterOfCavity5:						// Executing M675, max probing move has started
		if (LockMovementAndWaitForStandstill(gb))
		{
			reprap.GetHeat().SuspendHeaters(false);
			const auto zp = platform.GetZProbeOrDefault(currentZProbeNumber);
			zp->SetProbing(false);
			if (zProbeTriggered)
			{
				const float centre = (m675Settings.minDistance + moveState.currentUserPosition[m675Settings.axisNumber])/2;
				SetupM675BackoffMove(gb, centre);
				gb.AdvanceState();
			}
			else
			{
				gb.LatestMachineState().SetError("Probe was not triggered during probing move");
				gb.SetState(GCodeState::checkError);
				RetractZProbe(gb);
			}
		}
		break;

	case GCodeState::findCenterOfCavity6:						// Executing M675, move to centre has started
		if (LockMovementAndWaitForStandstill(gb))
		{
			gb.SetState(GCodeState::normal);
			RetractZProbe(gb);
		}
		break;

	case GCodeState::homing1:
		if (toBeHomed.IsEmpty())
		{
			gb.SetState(GCodeState::normal);
		}
		else
		{
			String<StringLength20> nextHomingFileName;
			const AxesBitmap mustHomeFirst = reprap.GetMove().GetKinematics().GetHomingFileName(toBeHomed, axesHomed, numVisibleAxes, nextHomingFileName.GetRef());
			if (mustHomeFirst.IsNonEmpty())
			{
				// Error, can't home this axes
				reply.copy("Must home axes [");
				AppendAxes(reply, mustHomeFirst);
				reply.cat("] before homing [");
				AppendAxes(reply, toBeHomed);
				reply.cat(']');
				stateMachineResult = GCodeResult::error;
				toBeHomed.Clear();
				gb.SetState(GCodeState::normal);
			}
			else
			{
				gb.SetState(GCodeState::homing2);
				if (!DoFileMacro(gb, nextHomingFileName.c_str(), false, SystemHelperMacroCode))
				{
					reply.printf("Homing file %s not found", nextHomingFileName.c_str());
					stateMachineResult = GCodeResult::error;
					toBeHomed.Clear();
					gb.SetState(GCodeState::normal);
				}
			}
		}
		break;

	case GCodeState::homing2:
		if (LockMovementAndWaitForStandstill(gb))		// movement should already be locked, but we need to wait for the previous homing move to complete
		{
			// Test whether the previous homing move homed any axes
			if (toBeHomed.Disjoint(axesHomed))
			{
				reply.copy("Homing failed");
				stateMachineResult = GCodeResult::error;
				toBeHomed.Clear();
				gb.SetState(GCodeState::normal);
			}
			else
			{
				toBeHomed &= ~axesHomed;
				gb.SetState((toBeHomed.IsEmpty()) ? GCodeState::normal : GCodeState::homing1);
			}
		}
		break;

	case GCodeState::toolChange0: 						// run tfree for the old tool (if any)
	case GCodeState::m109ToolChange0:					// run tfree for the old tool (if any)
		doingToolChange = true;
		SavePosition(toolChangeRestorePoint, gb);
		toolChangeRestorePoint.toolNumber = reprap.GetCurrentToolNumber();
		toolChangeRestorePoint.fanSpeed = lastDefaultFanSpeed;
		reprap.SetPreviousToolNumber();
		gb.AdvanceState();

		// If the tool is in the firmware-retracted state, there may be some Z hop applied, which we must remove
		moveState.currentUserPosition[Z_AXIS] += moveState.currentZHop;
		moveState.currentZHop = 0.0;

		if ((toolChangeParam & TFreeBit) != 0)
		{
			const Tool * const oldTool = reprap.GetCurrentTool();
			if (oldTool != nullptr)						// 2020-04-29: run tfree file even if not all axes have been homed
			{
				String<StringLength20> scratchString;
				scratchString.printf("tfree%d.g", oldTool->Number());
				DoFileMacro(gb, scratchString.c_str(), false, ToolChangeMacroCode);		// don't pass the T code here because it may be negative
			}
		}
		break;

	case GCodeState::toolChange1:						// release the old tool (if any), then run tpre for the new tool
	case GCodeState::m109ToolChange1:					// release the old tool (if any), then run tpre for the new tool
		if (LockMovementAndWaitForStandstill(gb))		// wait for tfree.g to finish executing
		{
			const Tool * const oldTool = reprap.GetCurrentTool();
			if (oldTool != nullptr)
			{
				reprap.StandbyTool(oldTool->Number(), IsSimulating());
				UpdateCurrentUserPosition(gb);			// the tool offset may have changed, so get the current position
			}
			gb.AdvanceState();
			if (reprap.GetTool(newToolNumber).IsNotNull() && (toolChangeParam & TPreBit) != 0)	// 2020-04-29: run tpre file even if not all axes have been homed
			{
				String<StringLength20> scratchString;
				scratchString.printf("tpre%d.g", newToolNumber);
				DoFileMacro(gb, scratchString.c_str(), false, ToolChangeMacroCode);
			}
		}
		break;

	case GCodeState::toolChange2:						// select the new tool if it exists and run tpost
	case GCodeState::m109ToolChange2:					// select the new tool if it exists and run tpost
		if (LockMovementAndWaitForStandstill(gb))		// wait for tpre.g to finish executing
		{
			reprap.SelectTool(newToolNumber, IsSimulating());
			UpdateCurrentUserPosition(gb);				// get the actual position of the new tool

			gb.AdvanceState();
			if (reprap.GetCurrentTool() != nullptr && (toolChangeParam & TPostBit) != 0)	// 2020-04-29: run tpost file even if not all axes have been homed
			{
				String<StringLength20> scratchString;
				scratchString.printf("tpost%d.g", newToolNumber);
				DoFileMacro(gb, scratchString.c_str(), false, ToolChangeMacroCode);
			}
		}
		break;

	case GCodeState::toolChangeComplete:
	case GCodeState::m109ToolChangeComplete:
		if (LockMovementAndWaitForStandstill(gb))		// wait for the move to height to finish
		{
			gb.LatestMachineState().feedRate = toolChangeRestorePoint.feedRate;
			// We don't restore the default fan speed in case the user wants to use a different one for the new tool
			doingToolChange = false;

			if (gb.GetState() == GCodeState::toolChangeComplete)
			{
				gb.SetState(GCodeState::normal);
			}
			else
			{
				UnlockAll(gb);							// allow movement again
				gb.AdvanceState();						// advance to m109WaitForTemperature
			}
		}
		break;

	case GCodeState::m109WaitForTemperature:
		if (cancelWait || IsSimulating() || ToolHeatersAtSetTemperatures(reprap.GetCurrentTool(), gb.LatestMachineState().waitWhileCooling, TEMPERATURE_CLOSE_ENOUGH))
		{
			cancelWait = isWaiting = false;
			gb.SetState(GCodeState::normal);
		}
		else
		{
			isWaiting = true;
		}
		break;

	case GCodeState::pausing1:
	case GCodeState::eventPausing1:
		if (LockMovementAndWaitForStandstill(gb))
		{
			gb.AdvanceState();
			if (AllAxesAreHomed())
			{
				DoFileMacro(gb, PAUSE_G, true, SystemHelperMacroCode);
			}
		}
		break;

	case GCodeState::filamentChangePause1:
		if (LockMovementAndWaitForStandstill(gb))
		{
			gb.AdvanceState();
			if (AllAxesAreHomed())
			{
				if (!DoFileMacro(gb, FILAMENT_CHANGE_G, false, SystemHelperMacroCode))
				{
					DoFileMacro(gb, PAUSE_G, true, SystemHelperMacroCode);
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
			platform.MessageF(LogWarn, "%s\n", reply.c_str());
			pauseState = PauseState::paused;
#if HAS_SBC_INTERFACE
			reportPause = reprap.UsingSbcInterface();
#endif
			gb.SetState(GCodeState::normal);
		}
		break;

	case GCodeState::eventPausing2:
		if (LockMovementAndWaitForStandstill(gb))
		{
			pauseState = PauseState::paused;
#if HAS_SBC_INTERFACE
			reportPause = reprap.UsingSbcInterface();
#endif
			gb.SetState(GCodeState::finishedProcessingEvent);
		}
		break;

	case GCodeState::resuming1:
	case GCodeState::resuming2:
		// Here when we have just finished running the resume macro file.
		// Move the head back to the paused location
		if (LockMovementAndWaitForStandstill(gb))
		{
			const float currentZ = moveState.coords[Z_AXIS];
			for (size_t axis = 0; axis < numVisibleAxes; ++axis)
			{
				moveState.currentUserPosition[axis] = pauseRestorePoint.moveCoords[axis];
			}
			SetMoveBufferDefaults();
			ToolOffsetTransform(moveState.currentUserPosition, moveState.coords);
			moveState.feedRate = ConvertSpeedFromMmPerMin(DefaultFeedRate);	// ask for a good feed rate, we may have paused during a slow move
			moveState.tool = reprap.GetCurrentTool();							// needed so that bed compensation is applied correctly
			if (gb.GetState() == GCodeState::resuming1 && currentZ > pauseRestorePoint.moveCoords[Z_AXIS])
			{
				// First move the head to the correct XY point, then move it down in a separate move
				moveState.coords[Z_AXIS] = currentZ;
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
			// We no longer restore the paused fan speeds automatically on resuming, because that messes up the print cooling fan speed if a tool change has been done
			// They can be restored manually in resume.g if required
			virtualExtruderPosition = pauseRestorePoint.virtualExtruderPosition;	// reset the extruder position in case we are receiving absolute extruder moves
			moveState.virtualExtruderPosition = pauseRestorePoint.virtualExtruderPosition;
			fileGCode->LatestMachineState().feedRate = pauseRestorePoint.feedRate;
			moveFractionToSkip = pauseRestorePoint.proportionDone;
			restartInitialUserC0 = pauseRestorePoint.initialUserC0;
			restartInitialUserC1 = pauseRestorePoint.initialUserC1;
			reply.copy("Printing resumed");
			platform.Message(LogWarn, "Printing resumed\n");
			pauseState = PauseState::notPaused;
			if (pausedInMacro)
			{
				fileGCode->OriginalMachineState().firstCommandAfterRestart = true;
			}
			gb.SetState(GCodeState::normal);
		}
		break;

	case GCodeState::cancelling:
		if (LockMovementAndWaitForStandstill(gb))		// wait until cancel.g has completely finished
		{
			pauseState = PauseState::notPaused;
			gb.SetState(GCodeState::normal);
		}
		break;

	case GCodeState::flashing1:
#if HAS_WIFI_NETWORKING || HAS_AUX_DEVICES

		// Update additional modules before the main firmware
		if (FirmwareUpdater::IsReady())
		{
			bool updating = false;
			String<MaxFilenameLength> filenameString;
			try
			{
				bool dummy;
				gb.TryGetQuotedString('P', filenameString.GetRef(), dummy);
			}
			catch (const GCodeException&) { }
			for (unsigned int module = 1; module < FirmwareUpdater::NumUpdateModules; ++module)
			{
				if (firmwareUpdateModuleMap.IsBitSet(module))
				{
					firmwareUpdateModuleMap.ClearBit(module);
					FirmwareUpdater::UpdateModule(module, serialChannelForPanelDueFlashing, filenameString.GetRef());
					updating = true;
					isFlashingPanelDue = (module == FirmwareUpdater::PanelDueFirmwareModule);
					break;
				}
			}
			if (!updating)
			{
				isFlashingPanelDue = false;
				gb.SetState(GCodeState::flashing2);
			}
		}
# if HAS_AUX_DEVICES
		else
		{
			PanelDueUpdater* const panelDueUpdater = platform.GetPanelDueUpdater();
			if (panelDueUpdater != nullptr)
			{
				panelDueUpdater->Spin();
			}
		}
# endif
#else
		gb.SetState(GCodeState::flashing2);
#endif
		break;

	case GCodeState::flashing2:
		if (firmwareUpdateModuleMap.IsBitSet(0))
		{
			// Update main firmware
			firmwareUpdateModuleMap.Clear();
			String<MaxFilenameLength> filenameString;
			try
			{
				bool dummy;
				gb.TryGetQuotedString('P', filenameString.GetRef(), dummy);
				reprap.UpdateFirmware(filenameString.GetRef());
				// The above call does not return unless an error occurred
			}
			catch (const GCodeException&) { }
		}
		isFlashing = false;
		gb.SetState(GCodeState::normal);
		break;

	case GCodeState::stopping:	// MO or M1 after executing stop.g/sleep.g if present
		if (LockMovementAndWaitForStandstill(gb))
		{
			pauseState = PauseState::notPaused;
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
			const float axis0Coord = grid.GetCoordinate(0, gridAxis0index);
			const float axis1Coord = grid.GetCoordinate(1, gridAxis1index);
			if (grid.IsInRadius(axis0Coord, axis1Coord))
			{
				const size_t axis0Num = grid.GetAxisNumber(0);
				const size_t axis1Num = grid.GetAxisNumber(1);
				AxesBitmap axes;
				axes.SetBit(axis0Num);
				axes.SetBit(axis1Num);
				float axesCoords[MaxAxes];
				memcpy(axesCoords, moveState.coords, sizeof(axesCoords));				// copy current coordinates of all other axes in case they are relevant to IsReachable
				const auto zp = platform.GetZProbeOrDefault(currentZProbeNumber);
				axesCoords[axis0Num] = axis0Coord - zp->GetOffset(axis0Num);
				axesCoords[axis1Num] = axis1Coord - zp->GetOffset(axis1Num);
				axesCoords[Z_AXIS] = zp->GetStartingHeight();
				if (move.IsAccessibleProbePoint(axesCoords, axes))
				{
					SetMoveBufferDefaults();
					moveState.coords[axis0Num] = axesCoords[axis0Num];
					moveState.coords[axis1Num] = axesCoords[axis1Num];
					moveState.coords[Z_AXIS] = zp->GetStartingHeight();
					moveState.feedRate = zp->GetTravelSpeed();
					NewMoveAvailable(1);

					InitialiseTaps(false);
					gb.AdvanceState();
				}
				else
				{
					platform.MessageF(WarningMessage, "Skipping grid point %c=%.1f, %c=%.1f because Z probe cannot reach it\n", grid.GetAxisLetter(0), (double)axis0Coord, grid.GetAxisLetter(1), (double)axis1Coord);
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
			if (platform.GetZProbeOrDefault(currentZProbeNumber)->GetProbeType() == ZProbeType::blTouch)
			{
				DeployZProbe(gb);
			}
		}
		break;

	case GCodeState::gridProbing2b:		// ready to probe the current grid probe point
		if (LockMovementAndWaitForStandstill(gb))
		{
			lastProbedTime = millis();														// start the recovery timer
			const auto zp = platform.GetZProbeOrDefault(currentZProbeNumber);
			if (zp->GetProbeType() != ZProbeType::none && zp->GetTurnHeatersOff())
			{
				reprap.GetHeat().SuspendHeaters(true);
			}
			gb.AdvanceState();
		}
		break;

	case GCodeState::gridProbing3:		// ready to probe the current grid probe point
		{
			const auto zp = platform.GetZProbeOrDefault(currentZProbeNumber);
			if (millis() - lastProbedTime >= (uint32_t)(zp->GetRecoveryTime() * SecondsToMillis))
			{
				// Probe the bed at the current XY coordinates
				// Check for probe already triggered at start
				if (zp->GetProbeType() == ZProbeType::none)
				{
					// No Z probe, so do manual mesh levelling instead
					UnlockAll(gb);															// release the movement lock to allow manual Z moves
					gb.AdvanceState();														// resume at next state when user has finished adjusting the height
					doingManualBedProbe = true;												// suspend the Z movement limit
					DoManualBedProbe(gb);
				}
				else if (zp->Stopped())
				{
					reprap.GetHeat().SuspendHeaters(false);
					gb.LatestMachineState().SetError("Probe already triggered before probing move started");
					gb.SetState(GCodeState::checkError);
					RetractZProbe(gb);
					break;
				}
				else
				{
					zProbeTriggered = false;
					SetMoveBufferDefaults();
					if (!platform.GetEndstops().EnableZProbe(currentZProbeNumber) || !zp->SetProbing(true))
					{
						gb.LatestMachineState().SetError("Failed to enable probe");
						gb.SetState(GCodeState::checkError);
						RetractZProbe(gb);
						break;
					}
					moveState.checkEndstops = true;
					moveState.reduceAcceleration = true;
					moveState.coords[Z_AXIS] = -zp->GetDiveHeight() + zp->GetActualTriggerHeight();
					moveState.feedRate = zp->GetProbingSpeed(tapsDone);
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
			const auto zp = platform.GetZProbeOrDefault(currentZProbeNumber);
			if (zp->GetProbeType() == ZProbeType::none)
			{
				// No Z probe, so we are doing manual mesh levelling. Take the current Z height as the height error.
				g30zHeightError = moveState.coords[Z_AXIS];
			}
			else
			{
				zp->SetProbing(false);
				if (!zProbeTriggered)
				{
					gb.LatestMachineState().SetError("Probe was not triggered during probing move");
					gb.SetState(GCodeState::checkError);
					RetractZProbe(gb);
					break;
				}

				// Grid probing never does an additional fast tap, so we can always include this tap in the average
				g30zHeightError = moveState.coords[Z_AXIS] - zp->GetActualTriggerHeight();
				g30zHeightErrorSum += g30zHeightError;
			}

			gb.AdvanceState();
			if (zp->GetProbeType() == ZProbeType::blTouch)		// bltouch needs to be retracted when it triggers
			{
				RetractZProbe(gb);
			}
		}
		break;

	case GCodeState::gridProbing4a:	// ready to lift the probe after probing the current grid probe point
		// Move back up to the dive height
		SetMoveBufferDefaults();
		{
			const auto zp = platform.GetZProbeOrDefault(currentZProbeNumber);
			moveState.coords[Z_AXIS] = zp->GetStartingHeight();
			moveState.feedRate = zp->GetTravelSpeed();
		}
		NewMoveAvailable(1);
		gb.AdvanceState();
		break;

	case GCodeState::gridProbing5:	// finished probing a point and moved back to the dive height
		if (LockMovementAndWaitForStandstill(gb))
		{
			// See whether we need to do any more taps
			const auto zp = platform.GetZProbeOrDefault(currentZProbeNumber);
			bool acceptReading = false;
			if (zp->GetMaxTaps() < 2)
			{
				acceptReading = true;
			}
			else if (tapsDone >= 2)
			{
				g30zHeightErrorLowestDiff = min<float>(g30zHeightErrorLowestDiff, fabsf(g30zHeightError - g30PrevHeightError));
				if (zp->GetTolerance() > 0.0)
				{
					if (g30zHeightErrorLowestDiff <= zp->GetTolerance())
					{
						g30zHeightError = (g30zHeightError + g30PrevHeightError)/2;
						acceptReading = true;
					}
				}
				else if (tapsDone == (int)zp->GetMaxTaps())
				{
					g30zHeightError = g30zHeightErrorSum/tapsDone;
					acceptReading = true;
				}
			}

			if (acceptReading)
			{
				reprap.GetMove().AccessHeightMap().SetGridHeight(gridAxis0index, gridAxis1index, g30zHeightError);
				gb.AdvanceState();
			}
			else if (tapsDone < (int)zp->GetMaxTaps())
			{
				// Tap again
				lastProbedTime = millis();
				g30PrevHeightError = g30zHeightError;
				gb.SetState(GCodeState::gridProbing2a);
			}
			else
			{
				gb.LatestMachineState().SetError("Z probe readings not consistent");
				gb.SetState(GCodeState::checkError);
				RetractZProbe(gb);
			}
		}
		break;

	case GCodeState::gridProbing6:	// ready to compute the next probe point
		{
			const HeightMap& hm = reprap.GetMove().AccessHeightMap();
			if (gridAxis1index & 1)
			{
				// Odd row, so decreasing X
				if (gridAxis0index == 0)
				{
					++gridAxis1index;
				}
				else
				{
					--gridAxis0index;
				}
			}
			else
			{
				// Even row, so increasing X
				if (gridAxis0index + 1 == hm.GetGrid().NumAxisPoints(0))
				{
					++gridAxis1index;
				}
				else
				{
					++gridAxis0index;
				}
			}

			if (gridAxis1index == hm.GetGrid().NumAxisPoints(1))
			{
				// Done all the points
				gb.AdvanceState();
				RetractZProbe(gb);
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
			float minError, maxError;
			Deviation deviation;
			const uint32_t numPointsProbed = reprap.GetMove().AccessHeightMap().GetStatistics(deviation, minError, maxError);
			if (numPointsProbed >= 4)
			{
				reprap.GetMove().SetLatestMeshDeviation(deviation);
				reply.printf("%" PRIu32 " points probed, min error %.3f, max error %.3f, mean %.3f, deviation %.3f\n",
								numPointsProbed, (double)minError, (double)maxError, (double)deviation.GetMean(), (double)deviation.GetDeviationFromMean());
#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE
				if (TrySaveHeightMap(DefaultHeightMapFile, reply))
				{
					stateMachineResult = GCodeResult::error;
				}
#endif
				reprap.GetMove().AccessHeightMap().ExtrapolateMissing();
				reprap.GetMove().UseMesh(true);
				const float absMean = fabsf(deviation.GetMean());
				if (absMean >= 0.05 && absMean >= 2 * deviation.GetDeviationFromMean())
				{
					platform.Message(WarningMessage, "the height map has a substantial Z offset. Suggest use Z-probe to establish Z=0 datum, then re-probe the mesh.\n");
				}
			}
			else
			{
				gb.LatestMachineState().SetError("Too few points probed");
			}
		}
		if (stateMachineResult == GCodeResult::ok)
		{
			reprap.GetPlatform().MessageF(LogWarn, "%s\n", reply.c_str());
		}
		gb.SetState(GCodeState::normal);
		break;

	// States used for G30 probing
	case GCodeState::probingAtPoint0:
		// Initial state when executing G30 with a P parameter. Start by moving to the dive height at the current position.
		SetMoveBufferDefaults();
		{
			const auto zp = platform.GetZProbeOrDefault(currentZProbeNumber);
			moveState.coords[Z_AXIS] = zp->GetStartingHeight();
			moveState.feedRate = zp->GetTravelSpeed();
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
			(void)reprap.GetMove().GetProbeCoordinates(g30ProbePointIndex, moveState.coords[X_AXIS], moveState.coords[Y_AXIS], true);
			const auto zp = platform.GetZProbeOrDefault(currentZProbeNumber);
			moveState.coords[Z_AXIS] = zp->GetStartingHeight();
			moveState.feedRate = zp->GetTravelSpeed();
			NewMoveAvailable(1);

			InitialiseTaps(false);
			gb.AdvanceState();
		}
		break;

	case GCodeState::probingAtPoint2a:								// note we return to this state when doing the second and subsequent taps
		// Executing G30 with a P parameter. The move to put the head at the specified XY coordinates has been commanded.
		// OR initial state when executing G30 with no P parameter (must call InitialiseTaps first)
		if (LockMovementAndWaitForStandstill(gb))
		{
			gb.AdvanceState();
			if (platform.GetZProbeOrDefault(currentZProbeNumber)->GetProbeType() == ZProbeType::blTouch)	// bltouch needs to be redeployed prior to each probe point
			{
				DeployZProbe(gb);
			}
		}
		break;

	case GCodeState::probingAtPoint2b:
		if (LockMovementAndWaitForStandstill(gb))
		{
			// Head has finished moving to the correct XY position and BLTouch has been deployed
			lastProbedTime = millis();								// start the probe recovery timer
			if (platform.GetZProbeOrDefault(currentZProbeNumber)->GetTurnHeatersOff())
			{
				reprap.GetHeat().SuspendHeaters(true);
			}
			gb.AdvanceState();
		}
		break;

	case GCodeState::probingAtPoint3:
		// Executing G30 with a P parameter. The move to put the head at the specified XY coordinates has been completed and the recovery timer started.
		// OR executing G30 without a P parameter, and the recovery timer has been started.
		{
			const auto zp = platform.GetZProbeOrDefault(currentZProbeNumber);
			if (millis() - lastProbedTime >= (uint32_t)(zp->GetRecoveryTime() * SecondsToMillis))
			{
				// The probe recovery time has elapsed, so we can start the probing  move
				if (zp->GetProbeType() == ZProbeType::none)
				{
					// No Z probe, so we are doing manual 'probing'
					UnlockAll(gb);															// release the movement lock to allow manual Z moves
					gb.AdvanceState();														// resume at the next state when the user has finished
					doingManualBedProbe = true;												// suspend the Z movement limit
					DoManualBedProbe(gb);
				}
				else if (zp->Stopped())														// check for probe already triggered at start
				{
					// Z probe is already triggered at the start of the move, so abandon the probe and record an error
					reprap.GetHeat().SuspendHeaters(false);
					gb.LatestMachineState().SetError("Probe already triggered at start of probing move");
					if (g30ProbePointIndex >= 0)
					{
						reprap.GetMove().SetZBedProbePoint(g30ProbePointIndex, zp->GetDiveHeight(), true, true);
					}
					gb.SetState(GCodeState::checkError);									// no point in doing anything else
					RetractZProbe(gb);
				}
				else
				{
					zProbeTriggered = false;
					SetMoveBufferDefaults();
					if (!platform.GetEndstops().EnableZProbe(currentZProbeNumber) || !zp->SetProbing(true))
					{
						gb.LatestMachineState().SetError("Failed to enable probe");
						gb.SetState(GCodeState::checkError);
						RetractZProbe(gb);
						break;
					}

					moveState.checkEndstops = true;
					moveState.reduceAcceleration = true;
					moveState.coords[Z_AXIS] = (IsAxisHomed(Z_AXIS))
												? platform.AxisMinimum(Z_AXIS) - zp->GetDiveHeight() + zp->GetActualTriggerHeight()	// Z axis has been homed, so no point in going very far
												: -1.1 * platform.AxisTotalLength(Z_AXIS);	// Z axis not homed yet, so treat this as a homing move
					moveState.feedRate = zp->GetProbingSpeed(tapsDone);
					NewMoveAvailable(1);
					gb.AdvanceState();
				}
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
			const auto zp = platform.GetZProbeOrDefault(currentZProbeNumber);
			if (zp->GetProbeType() == ZProbeType::none)
			{
				// No Z probe, so we are doing manual mesh levelling. Take the current Z height as the height error.
				g30zHeightError = moveState.coords[Z_AXIS];
				zp->SetLastStoppedHeight(g30zHeightError);
			}
			else
			{
				zp->SetProbing(false);
				if (!zProbeTriggered)
				{
					gb.LatestMachineState().SetError("Probe was not triggered during probing move");
					g30zHeightErrorSum = g30zHeightError = 0.0;
					hadProbingError = true;
				}
				else
				{
					// Successful probing
					float m[MaxAxes];
					reprap.GetMove().GetCurrentMachinePosition(m, false);		// get height without bed compensation
					const float g30zStoppedHeight = m[Z_AXIS] - g30HValue;		// save for later
					zp->SetLastStoppedHeight(g30zStoppedHeight);
					if (tapsDone > 0)											// don't accumulate the result of we are doing fast-then-slow probing and this was the fast probe
					{
						g30zHeightError = g30zStoppedHeight - zp->GetActualTriggerHeight();
						g30zHeightErrorSum += g30zHeightError;
					}
				}
			}

			if (g30ProbePointIndex < 0)											// if no P parameter
			{
				// Simple G30 probing move
				if (g30SValue == -1 || g30SValue == -2 || g30SValue == -3)
				{
					// G30 S-1 command taps once and reports the height, S-2 sets the tool offset to the negative of the current height, S-3 sets the Z probe trigger height
					gb.SetState(GCodeState::probingAtPoint7);					// special state for reporting the stopped height at the end
					RetractZProbe(gb);											// retract the probe before moving to the new state
					break;
				}

				if (tapsDone <= 1 && !hadProbingError)
				{
					// Reset the Z axis origin according to the height error so that we can move back up to the dive height
					moveState.coords[Z_AXIS] = zp->GetActualTriggerHeight();
					reprap.GetMove().SetNewPosition(moveState.coords, false);

					// Find the coordinates of the Z probe to pass to SetZeroHeightError
					float tempCoords[MaxAxes];
					memcpyf(tempCoords, moveState.coords, ARRAY_SIZE(tempCoords));
					tempCoords[X_AXIS] += zp->GetOffset(X_AXIS);
					tempCoords[Y_AXIS] += zp->GetOffset(Y_AXIS);
					reprap.GetMove().SetZeroHeightError(tempCoords);
					ToolOffsetInverseTransform(moveState.coords, moveState.currentUserPosition);

					g30zHeightErrorSum = g30zHeightError = 0;					// there is no longer any height error from this probe
					SetAxisIsHomed(Z_AXIS);										// this is only correct if the Z axis is Cartesian-like, but other architectures must be homed before probing anyway
					zDatumSetByProbing = true;
				}
			}

			gb.AdvanceState();
			if (zp->GetProbeType() == ZProbeType::blTouch)						// bltouch needs to be retracted when it triggers
			{
				RetractZProbe(gb);
			}
		}
		break;

	case GCodeState::probingAtPoint4a:
		// Move back up to the dive height before we change anything, in particular before we adjust leadscrews
		SetMoveBufferDefaults();
		{
			const auto zp = platform.GetZProbeOrDefault(currentZProbeNumber);
			moveState.coords[Z_AXIS] = zp->GetStartingHeight();
			moveState.feedRate = zp->GetTravelSpeed();
		}
		NewMoveAvailable(1);
		gb.AdvanceState();
		break;

	case GCodeState::probingAtPoint5:
		// Here when we have moved the head back up to the dive height
		if (LockMovementAndWaitForStandstill(gb))
		{
			// See whether we need to do any more taps
			const auto zp = platform.GetZProbeOrDefault(currentZProbeNumber);
			bool acceptReading = false;
			if (zp->GetMaxTaps() < 2 && tapsDone == 1)
			{
				acceptReading = true;
			}
			else if (tapsDone >= 2)
			{
				g30zHeightErrorLowestDiff = min<float>(g30zHeightErrorLowestDiff, fabsf(g30zHeightError - g30PrevHeightError));
				if (zp->GetTolerance() > 0.0 && g30zHeightErrorLowestDiff <= zp->GetTolerance())
				{
					g30zHeightError = (g30zHeightError + g30PrevHeightError)/2;
					acceptReading = true;
				}
			}

			if (!acceptReading)
			{
				if (tapsDone < (int)zp->GetMaxTaps())
				{
					// Tap again
					g30PrevHeightError = g30zHeightError;
					lastProbedTime = millis();
					gb.SetState(GCodeState::probingAtPoint2a);
					break;
				}

				// We no longer flag this as a probing error, instead we take the average and issue a warning
				g30zHeightError = g30zHeightErrorSum/tapsDone;
				if (zp->GetTolerance() > 0.0)			// zero or negative tolerance means always average all readings, so no warning message
				{
					gb.LatestMachineState().SetError("Z probe readings not consistent");
				}
			}

			if (g30ProbePointIndex >= 0)
			{
				reprap.GetMove().SetZBedProbePoint(g30ProbePointIndex, g30zHeightError, true, hadProbingError);
			}
			else
			{
				// Setting the Z height with G30
				moveState.coords[Z_AXIS] -= g30zHeightError;
				reprap.GetMove().SetNewPosition(moveState.coords, false);

				// Find the coordinates of the Z probe to pass to SetZeroHeightError
				float tempCoords[MaxAxes];
				memcpyf(tempCoords, moveState.coords, ARRAY_SIZE(tempCoords));
				tempCoords[X_AXIS] += zp->GetOffset(X_AXIS);
				tempCoords[Y_AXIS] += zp->GetOffset(Y_AXIS);
				reprap.GetMove().SetZeroHeightError(tempCoords);
				ToolOffsetInverseTransform(moveState.coords, moveState.currentUserPosition);
			}
			gb.AdvanceState();
			if (zp->GetProbeType() != ZProbeType::blTouch)			// if it's a BLTouch then we have already retracted it
			{
				RetractZProbe(gb);
			}
		}
		break;

	case GCodeState::probingAtPoint6:
		// Here when we have finished probing and have retracted the probe if necessary
		if (LockMovementAndWaitForStandstill(gb))		// retracting the Z probe
		{
			if (g30SValue == 1)
			{
				// G30 with a silly Z value and S=1 is equivalent to G30 with no parameters in that it sets the current Z height
				// This is useful because it adjusts the XY position to account for the probe offset.
				moveState.coords[Z_AXIS] -= g30zHeightError;
				reprap.GetMove().SetNewPosition(moveState.coords, false);
				ToolOffsetInverseTransform(moveState.coords, moveState.currentUserPosition);
			}
			else if (g30SValue >= -1)
			{
				if (reprap.GetMove().FinishedBedProbing(g30SValue, reply))
				{
					stateMachineResult = GCodeResult::error;
				}
				else if (reprap.GetMove().GetKinematics().SupportsAutoCalibration())
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
			const auto zp = platform.GetZProbeOrDefault(currentZProbeNumber);
			zp->SetTriggerHeight(zp->GetLastStoppedHeight());
			reply.printf("Z probe trigger height set to %.3f mm", (double)zp->GetLastStoppedHeight());
		}
		else if (g30SValue == -2)
		{
			// Adjust the Z offset of the current tool to account for the height error
			Tool * const tool = reprap.GetCurrentTool();
			if (tool == nullptr)
			{
				gb.LatestMachineState().SetError("Tool was deselected during G30 S-2 command");
			}
			else
			{
				tool->SetOffset(Z_AXIS, -g30zHeightError, true);
				ToolOffsetInverseTransform(moveState.coords, moveState.currentUserPosition);	// update user coordinates to reflect the new tool offset
			}
		}
		else
		{
			// Just print the stop height
			reply.printf("Stopped at height %.3f mm", (double)platform.GetZProbeOrDefault(currentZProbeNumber)->GetLastStoppedHeight());
		}
		gb.SetState(GCodeState::normal);
		break;

	case GCodeState::straightProbe0:			// ready to deploy the probe
		if (LockMovementAndWaitForStandstill(gb))
		{
			gb.AdvanceState();
			currentZProbeNumber = straightProbeSettings.GetZProbeToUse();
			DeployZProbe(gb);
		}
		break;

	case GCodeState::straightProbe1:
		if (LockMovementAndWaitForStandstill(gb))
		{
			const auto zp = platform.GetEndstops().GetZProbe(straightProbeSettings.GetZProbeToUse());
			lastProbedTime = millis();			// start the probe recovery timer
			if (zp.IsNotNull() && zp->GetTurnHeatersOff())
			{
				reprap.GetHeat().SuspendHeaters(true);
			}
			gb.AdvanceState();
		}
		break;

	case GCodeState::straightProbe2:
		// Executing G38. The probe has been deployed and the recovery timer has been started.
		{
			if (millis() - lastProbedTime >= (uint32_t)(platform.GetZProbeOrDefault(straightProbeSettings.GetZProbeToUse())->GetRecoveryTime() * SecondsToMillis))
			{
				// The probe recovery time has elapsed, so we can start the probing  move
				const auto zp = platform.GetEndstops().GetZProbe(straightProbeSettings.GetZProbeToUse());
				if (zp.IsNull() || zp->GetProbeType() == ZProbeType::none)
				{
					// No Z probe, so we are doing manual 'probing'
					UnlockAll(gb);															// release the movement lock to allow manual Z moves
					gb.AdvanceState();														// resume at the next state when the user has finished
					DoStraightManualProbe(gb, straightProbeSettings);						// call out to separate function because it used a lot of stack
				}
				else
				{
					const bool probingAway = straightProbeSettings.ProbingAway();
					const bool atStop = zp->Stopped();
					if (probingAway != atStop)
					{
						// Z probe is already in target state at the start of the move, so abandon the probe and signal an error if the type demands so
						reprap.GetHeat().SuspendHeaters(false);
						if (straightProbeSettings.SignalError())
						{
							gb.LatestMachineState().SetError((probingAway) ? "Probe not triggered at start of probing move" : "Probe already triggered at start of probing move");
						}
						gb.SetState(GCodeState::checkError);								// no point in doing anything else
						RetractZProbe(gb);
					}
					else
					{
						zProbeTriggered = false;
						SetMoveBufferDefaults();
						if (!platform.GetEndstops().EnableZProbe(straightProbeSettings.GetZProbeToUse(), probingAway) || !zp->SetProbing(true))
						{
							gb.LatestMachineState().SetError("Failed to enable probe");
							gb.SetState(GCodeState::checkError);
							RetractZProbe(gb);
							break;
						}

						moveState.checkEndstops = true;
						moveState.reduceAcceleration = true;
						straightProbeSettings.SetCoordsToTarget(moveState.coords);
						moveState.feedRate = zp->GetProbingSpeed(0);
						NewMoveAvailable(1);
						gb.AdvanceState();
					}
				}
			}
		}
		break;

	case GCodeState::straightProbe3:
		// Executing G38. The probe wasn't in target state at the start of the move, and the probing move has been commanded.
		if (LockMovementAndWaitForStandstill(gb))
		{
			// Probing move has stopped
			reprap.GetHeat().SuspendHeaters(false);
			const bool probingAway = straightProbeSettings.ProbingAway();
			const auto zp = platform.GetEndstops().GetZProbe(straightProbeSettings.GetZProbeToUse());
			if (zp.IsNotNull() && zp->GetProbeType() != ZProbeType::none)
			{
				zp->SetProbing(false);
				if (!zProbeTriggered && straightProbeSettings.SignalError())
				{
					gb.LatestMachineState().SetError((probingAway) ? "Probe did not lose contact during probing move" : "Probe was not triggered during probing move");
				}
			}

			gb.SetState(GCodeState::checkError);
			RetractZProbe(gb);								// retract the probe before moving to the new state
		}
		break;

	// Firmware retraction/un-retraction states
	case GCodeState::doingFirmwareRetraction:
		// We just did the retraction part of a firmware retraction, now we need to do the Z hop
		if (moveState.segmentsLeft == 0)
		{
			const Tool * const tool = reprap.GetCurrentTool();
			if (tool != nullptr)
			{
				SetMoveBufferDefaults();
				moveState.tool = tool;
				reprap.GetMove().GetCurrentUserPosition(moveState.coords, 0, moveState.tool);
				moveState.coords[Z_AXIS] += tool->GetRetractHop();
				moveState.feedRate = platform.MaxFeedrate(Z_AXIS);
				moveState.filePos = (&gb == fileGCode) ? gb.GetFilePosition() : noFilePosition;
				moveState.canPauseAfter = false;			// don't pause after a retraction because that could cause too much retraction
				moveState.currentZHop = tool->GetRetractHop();
				NewMoveAvailable(1);
			}
			gb.SetState(GCodeState::normal);
		}
		break;

	case GCodeState::doingFirmwareUnRetraction:
		// We just undid the Z-hop part of a firmware un-retraction, now we need to do the un-retract
		if (moveState.segmentsLeft == 0)
		{
			const Tool * const tool = reprap.GetCurrentTool();
			if (tool != nullptr && tool->DriveCount() != 0)
			{
				SetMoveBufferDefaults();
				moveState.tool = tool;
				reprap.GetMove().GetCurrentUserPosition(moveState.coords, 0, tool);
				for (size_t i = 0; i < tool->DriveCount(); ++i)
				{
					moveState.coords[ExtruderToLogicalDrive(tool->GetDrive(i))] = tool->GetRetractLength() + tool->GetRetractExtra();
				}
				moveState.feedRate = tool->GetUnRetractSpeed() * tool->DriveCount();
				moveState.filePos = (&gb == fileGCode) ? gb.GetFilePosition() : noFilePosition;
				moveState.canPauseAfter = true;
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
# if HAS_MASS_STORAGE || HAS_SBC_INTERFACE
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
				const uint32_t ms = millis() - timingStartMillis;
				const float fileMbytes = (float)timingBytesWritten/(float)(1024 * 1024);
				const float mbPerSec = (fileMbytes * 1000.0)/(float)ms;
				platform.MessageF(gb.GetResponseMessageType(), "SD write speed for %.1fMbyte file was %.2fMbytes/sec\n", (double)fileMbytes, (double)mbPerSec);
				sdTimingFile->Close();

				sdTimingFile = platform.OpenFile(platform.GetGCodeDir(), TimingFileName, OpenMode::read);
				if (sdTimingFile == nullptr)
				{
					platform.Delete(platform.GetGCodeDir(), TimingFileName);
					gb.LatestMachineState().SetError("Failed to re-open timing file");
					gb.SetState(GCodeState::normal);
					break;
				}

				platform.Message(gb.GetResponseMessageType(), "Testing SD card read speed...\n");
				timingBytesWritten = 0;
				timingStartMillis = millis();
				gb.SetState(GCodeState::timingSDread);
				break;
			}

			const unsigned int bytesToWrite = min<size_t>(reply.Capacity(), timingBytesRequested - timingBytesWritten);
			if (!sdTimingFile->Write(reply.c_str(), bytesToWrite))
			{
				sdTimingFile->Close();
				platform.Delete(platform.GetGCodeDir(), TimingFileName);
				gb.LatestMachineState().SetError("Failed to write to timing file");
				gb.SetState(GCodeState::normal);
				break;
			}
			timingBytesWritten += bytesToWrite;
			writtenThisTime += bytesToWrite;
		}
		break;

	case GCodeState::timingSDread:
		for (uint32_t readThisTime = 0; readThisTime < 100 * 1024; )
		{
			if (timingBytesWritten >= timingBytesRequested)
			{
				const uint32_t ms = millis() - timingStartMillis;
				const float fileMbytes = (float)timingBytesWritten/(float)(1024 * 1024);
				const float mbPerSec = (fileMbytes * 1000.0)/(float)ms;
				sdTimingFile->Close();
				reply.printf("SD read speed for %.1fMbyte file was %.2fMbytes/sec", (double)fileMbytes, (double)mbPerSec);
				platform.Delete(platform.GetGCodeDir(), TimingFileName);
				gb.SetState(GCodeState::normal);
				break;
			}

			const unsigned int bytesToRead = min<size_t>(reply.Capacity(), timingBytesRequested - timingBytesWritten);
			if (sdTimingFile->Read(reply.Pointer(), bytesToRead) != (int)bytesToRead)
			{
				sdTimingFile->Close();
				platform.Delete(platform.GetGCodeDir(), TimingFileName);
				gb.LatestMachineState().SetError("Failed to read from timing file");
				gb.SetState(GCodeState::normal);
				break;
			}
			timingBytesWritten += bytesToRead;
			readThisTime += bytesToRead;
		}
		break;
#endif

#if HAS_SBC_INTERFACE
	case GCodeState::waitingForAcknowledgement:	// finished M291 and the SBC expects a response next
#endif
	case GCodeState::checkError:				// we return to this state after running the retractprobe macro when there may be a stored error message
		gb.SetState(GCodeState::normal);
		break;

	// Here when we need to execute the default action for an event because the macro file was not found, the the default action involves pausing the print.
	// We have already sent an alert.
	case GCodeState::processingEvent:
		if (pauseState != PauseState::resuming)						// if we are resuming, wait for the resume to complete
		{
			if (pauseState != PauseState::notPaused)
			{
				gb.SetState(GCodeState::finishedProcessingEvent);	// already paused or pausing
			}
			else if (LockMovementAndWaitForStandstill(gb))
			{
				const PrintPausedReason pauseReason = Event::GetDefaultPauseReason();
				gb.SetState(GCodeState::finishedProcessingEvent);
				DoPause(gb, pauseReason, (pauseReason == PrintPausedReason::driverError) ? GCodeState::eventPausing2 : GCodeState::eventPausing1);
			}
		}
		break;

	// Here when we have finished processing an event
	case GCodeState::finishedProcessingEvent:
		Event::FinishedProcessing();
		gb.SetState(GCodeState::normal);
		break;

	default:				// should not happen
		gb.LatestMachineState().SetError("Undefined GCodeState");
		gb.SetState(GCodeState::normal);
		break;
	}

	if (gb.GetState() == GCodeState::normal)
	{
		// We completed a command, so unlock resources and tell the host about it
		gb.StopTimer();
		UnlockAll(gb);
		gb.LatestMachineState().RetrieveStateMachineResult(stateMachineResult, reply);
		HandleReply(gb, stateMachineResult, reply.c_str());

		CheckForDeferredPause(gb);
	}

#if HAS_SBC_INTERFACE
	if (reportPause)
	{
		fileGCode->Invalidate();
		reprap.GetSbcInterface().ReportPause();
	}
#endif
}

// Do a manual probe. This is in its own function to reduce the amount of stack space needed by RunStateMachine(). See the comment at the top of that function.
void GCodes::DoStraightManualProbe(GCodeBuffer& gb, const StraightProbeSettings& sps)
{
	String<StringLength256> message;
	message.printf("Adjust position until the reference point just %s the target, then press OK", sps.ProbingAway() ? "loses contact with" : "touches");
	DoManualProbe(gb, message.c_str(), "Manual Straight Probe", sps.GetMovingAxes());
}

// End
