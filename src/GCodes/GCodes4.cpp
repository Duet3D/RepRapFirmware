// State machine function for GCode class

#include "GCodes.h"
#include "GCodeBuffer/GCodeBuffer.h"
#include <Platform/RepRap.h>
#include <Platform/Event.h>
#include <PrintMonitor/PrintMonitor.h>
#include <Movement/Move.h>
#include <Tools/Tool.h>
#include <Heating/Heat.h>
#include <Endstops/ZProbe.h>

#if SUPPORT_CAN_EXPANSION
# include <CAN/CanMotion.h>
#endif

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

	MovementState& ms = GetMovementState(gb);
	const GCodeState state = gb.GetState();
	switch (state)
	{
	case GCodeState::waitingForSpecialMoveToComplete:
	case GCodeState::abortWhenMovementFinished:
		if (   LockCurrentMovementSystemAndWaitForStandstill(gb)		// movement should already be locked, but we need to wait for standstill and fetch the current position
#if SUPPORT_CAN_EXPANSION
			&& CanMotion::FinishedReverting()
#endif
		   )
		{
			// Check whether we made any G1 S3 moves and need to set the axis limits
			axesToSenseLength.Iterate([this, &ms](unsigned int axis, unsigned int)
										{
											const EndStopPosition stopType = platform.GetEndstops().GetEndStopPosition(axis);
											if (stopType == EndStopPosition::highEndStop)
											{
												platform.SetAxisMaximum(axis, ms.coords[axis], true);
											}
											else if (stopType == EndStopPosition::lowEndStop)
											{
												platform.SetAxisMinimum(axis, ms.coords[axis], true);
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
		switch (ms.segMoveState)
		{
		case SegmentedMoveState::inactive:					// move completed without error
			gb.SetState(GCodeState::normal);
			break;

		case SegmentedMoveState::aborted:					// move terminated abnormally
			if (!LockCurrentMovementSystemAndWaitForStandstill(gb))	// update the the user position from the machine position at which we stop
			{
				break;
			}
			gb.LatestMachineState().SetError("intermediate position outside machine limits");
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
		if (LockCurrentMovementSystemAndWaitForStandstill(gb))
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

			if (ms.currentTool != nullptr)
			{
				// We get here when the tool probe has been activated. In this case we know how far we
				// went (i.e. the difference between our start and end positions) and if we need to
				// incorporate any correction factors. That's why we only need to set the final tool
				// offset to this value in order to finish the tool probing.
				const float coord = ms.GetToolChangeRestorePoint().moveCoords[m585Settings.axisNumber] - ms.currentUserPosition[m585Settings.axisNumber] + m585Settings.offset;
				ms.currentTool->SetOffset(m585Settings.axisNumber, coord, true);
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
		if (LockCurrentMovementSystemAndWaitForStandstill(gb))
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
		if (LockCurrentMovementSystemAndWaitForStandstill(gb))
		{
			const auto zp = platform.GetZProbeOrDefault(currentZProbeNumber);
			zp->SetProbing(false);
			if (zProbeTriggered)
			{
				m675Settings.minDistance = ms.currentUserPosition[m675Settings.axisNumber];
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
		if (LockCurrentMovementSystemAndWaitForStandstill(gb))
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
		if (LockCurrentMovementSystemAndWaitForStandstill(gb))
		{
			reprap.GetHeat().SuspendHeaters(false);
			const auto zp = platform.GetZProbeOrDefault(currentZProbeNumber);
			zp->SetProbing(false);
			if (zProbeTriggered)
			{
				const float centre = (m675Settings.minDistance + ms.currentUserPosition[m675Settings.axisNumber])/2;
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
		if (LockCurrentMovementSystemAndWaitForStandstill(gb))
		{
			gb.SetState(GCodeState::normal);
			RetractZProbe(gb);
		}
		break;

	case GCodeState::homing1:
		// We should only ever get here when toBeHomed is not empty
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
		if (LockCurrentMovementSystemAndWaitForStandstill(gb))		// movement should already be locked, but we need to wait for the previous homing move to complete
		{
			// Test whether the previous homing move homed any axes
			if (toBeHomed.Disjoint(axesHomed))
			{
				//debugPrintf("tbh %04x, ah %04x\n", (unsigned int)toBeHomed.GetRaw(), (unsigned int)axesHomed.GetRaw());
				reply.copy("Failed to home axes ");
				AppendAxes(reply, toBeHomed);
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
		SavePosition(gb, ToolChangeRestorePointNumber);
		ms.GetToolChangeRestorePoint().toolNumber = ms.GetCurrentToolNumber();
		ms.GetToolChangeRestorePoint().fanSpeed = ms.virtualFanSpeed;
		ms.SetPreviousToolNumber();
		reprap.StateUpdated();							// tell DWC/DSF that a restore point, nextToolNumber and the previousToolNumber have been updated
		gb.AdvanceState();

		// If the tool is in the firmware-retracted state, there may be some Z hop applied, which we must remove
		if (ms.currentTool != nullptr)
		{
			ms.currentUserPosition[Z_AXIS] += ms.currentTool->GetActualZHop();
			ms.currentTool->SetActualZHop(0.0);
			if ((ms.toolChangeParam & TFreeBit) != 0)
			{
				String<StringLength20> scratchString;
				scratchString.printf(TFREE "%d.g", ms.currentTool->Number());
				if (!DoFileMacro(gb, scratchString.c_str(), false, ToolChangeMacroCode))		// don't pass the T code here because it may be negative
				{
					DoFileMacro(gb, TFREE ".g", false, ToolChangeMacroCode);
				}
			}
		}
		break;

	case GCodeState::toolChange1:						// release the old tool (if any), then run tpre for the new tool
	case GCodeState::m109ToolChange1:					// release the old tool (if any), then run tpre for the new tool
		if (LockCurrentMovementSystemAndWaitForStandstill(gb))	// wait for tfree.g to finish executing
		{
			if (ms.currentTool != nullptr)
			{
				if (!IsSimulating())
				{
					ms.currentTool->Standby();
				}
				ms.currentTool = nullptr;
				UpdateCurrentUserPosition(gb);			// the tool offset may have changed, so get the current position
			}

			gb.AdvanceState();

			if (Tool::GetLockedTool(ms.newToolNumber).IsNotNull() && (ms.toolChangeParam & TPreBit) != 0)	// 2020-04-29: run tpre file even if not all axes have been homed
			{
				String<StringLength20> scratchString;
				scratchString.printf(TPRE "%d.g", ms.newToolNumber);
				if (!DoFileMacro(gb, scratchString.c_str(), false, ToolChangeMacroCode))
				{
					DoFileMacro(gb, TPRE ".g", false, ToolChangeMacroCode);
				}
			}
		}
		break;

	case GCodeState::toolChange2:						// select the new tool if it exists and run tpost
	case GCodeState::m109ToolChange2:					// select the new tool if it exists and run tpost
		if (LockCurrentMovementSystemAndWaitForStandstill(gb))	// wait for tpre.g to finish executing
		{
#if SUPPORT_ASYNC_MOVES
# if PREALLOCATE_TOOL_AXES
			{
				auto newTool = Tool::GetLockedTool(ms.newToolNumber);
				if (newTool.IsNull())
				{
					ms.ReleaseAllOwnedAxesAndExtruders();	// release all axes and extruders that this movement system owns
				}
				else
				{
					const AxesBitmap newToolAxes = newTool->GetXYAxesAndExtruders();
					const AxesBitmap axesToRelease = ms.GetAxesAndExtrudersOwned() & ~newToolAxes;
					ms.ReleaseAxesAndExtruders(axesToRelease);

					const AxesBitmap axesToAllocate = newToolAxes & ~ms.GetAxesAndExtrudersOwned();
					try
					{
						AllocateAxes(gb, ms, axesToAllocate, ParameterLettersToBitmap("XY"));
					}
					catch (const GCodeException& exc)
					{
						// We failed to allocate the new axes/extruders that we need. Release all axes and extruders that this movement system owns.
						ms.ReleaseAllOwnedAxesAndExtruders();
						gb.LatestMachineState().SetError(exc);
						gb.SetState(GCodeState::normal);
						break;
					}
				}
			}
# else
			ms.ReleaseAllOwnedAxesAndExtruders();
# endif
#endif
			ms.SelectTool(ms.newToolNumber, IsSimulating());
			UpdateCurrentUserPosition(gb);				// get the user position of the XY axes for the new tool

			gb.AdvanceState();
			if (ms.currentTool != nullptr && (ms.toolChangeParam & TPostBit) != 0)	// 2020-04-29: run tpost file even if not all axes have been homed
			{
				String<StringLength20> scratchString;
				scratchString.printf(TPOST "%d.g", ms.newToolNumber);
				if (!DoFileMacro(gb, scratchString.c_str(), false, ToolChangeMacroCode))
				{
					DoFileMacro(gb, TPOST ".g", false, ToolChangeMacroCode);
				}
			}
		}
		break;

	case GCodeState::toolChangeComplete:
	case GCodeState::m109ToolChangeComplete:
		if (LockCurrentMovementSystemAndWaitForStandstill(gb))	// wait for the move to height to finish
		{
			gb.LatestMachineState().feedRate = ms.GetToolChangeRestorePoint().feedRate;
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
		if (gb.IsCancelWaitRequested() || IsSimulating() || ToolHeatersAtSetTemperatures(ms.currentTool, gb.LatestMachineState().waitWhileCooling, TemperatureCloseEnough, gb.IsFileChannel()))
		{
			gb.SetState(GCodeState::normal);
		}
		break;

	case GCodeState::pausing1:
	case GCodeState::eventPausing1:
		if (LockAllMovementSystemsAndWaitForStandstill(gb))
		{
			gb.AdvanceState();
			if (AllAxesAreHomed())
			{
				DoFileMacro(gb, PAUSE_G, true, SystemHelperMacroCode);
			}
		}
		break;

	case GCodeState::filamentChangePause1:
		if (LockAllMovementSystemsAndWaitForStandstill(gb))
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
		if (LockAllMovementSystemsAndWaitForStandstill(gb))
		{
			reply.printf((gb.GetState() == GCodeState::filamentChangePause2) ? "Printing paused for filament change at" : "Printing paused at");
			for (size_t axis = 0; axis < numVisibleAxes; ++axis)
			{
				reply.catf(" %c%.1f", axisLetters[axis], (double)ms.GetPauseRestorePoint().moveCoords[axis]);
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
		if (LockAllMovementSystemsAndWaitForStandstill(gb))
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
		if (LockAllMovementSystemsAndWaitForStandstill(gb))
		{
#if SUPPORT_ASYNC_MOVES
			bool zPendingRestore = false;
			for (MovementState& tempMs : moveStates)
			{
				SetMoveBufferDefaults(tempMs);
				for (size_t axis = 0; axis < numVisibleAxes; ++axis)
				{
					// We may restore this axis if either this motion system owns it or this is motion system 0 and the axis is free
					if (   (tempMs.GetAxesAndExtrudersOwned().IsBitSet(axis) || (tempMs.GetMsNumber() == 0 && IsAxisFree(axis)))
						&& tempMs.currentUserPosition[axis] != tempMs.GetPauseRestorePoint().moveCoords[axis]
					   )
					{
						// This motion system may restore the position of this axis
						if (axis == Z_AXIS && state == GCodeState::resuming1)
						{
							zPendingRestore = true;								// restore Z next time
							continue;
						}

						if (!tempMs.GetAxesAndExtrudersOwned().IsBitSet(axis))		// if we don't own the axis
						{
							try
							{
								AllocateAxes(gb, tempMs, AxesBitmap::MakeFromBits(axis), ParameterLettersBitmap());
							}
							catch (const GCodeException& exc)
							{
								continue;											// we failed to allocate the axis - should not happen
							}
						}

						// AllocateAxes updates the user coordinates, so we need to set them up here not earlier
						tempMs.currentUserPosition[axis] = tempMs.GetPauseRestorePoint().moveCoords[axis];
						if (platform.IsAxisLinear(axis))
						{
							tempMs.linearAxesMentioned = true;
						}
						else if (platform.IsAxisRotational(axis))
						{
							tempMs.rotationalAxesMentioned = true;
						}
					}
				}

				ToolOffsetTransform(tempMs);
				tempMs.feedRate = ConvertSpeedFromMmPerMin(DefaultFeedRate);	// ask for a good feed rate, we may have paused during a slow move
				NewSingleSegmentMoveAvailable(tempMs);
			}
			gb.SetState((zPendingRestore) ? GCodeState::resuming2 : GCodeState::resuming3);
#else
			const bool restoreZ = (gb.GetState() != GCodeState::resuming1 || ms.coords[Z_AXIS] <= ms.GetPauseRestorePoint().moveCoords[Z_AXIS]);
			for (size_t axis = 0; axis < numVisibleAxes; ++axis)
			{
				if (   ms.currentUserPosition[axis] != ms.GetPauseRestorePoint().moveCoords[axis]
					&& (restoreZ || axis != Z_AXIS)
				   )
				{
					ms.currentUserPosition[axis] = ms.GetPauseRestorePoint().moveCoords[axis];
					if (platform.IsAxisLinear(axis))
					{
						ms.linearAxesMentioned = true;
					}
					else if (platform.IsAxisRotational(axis))
					{
						ms.rotationalAxesMentioned = true;
					}
				}
			}

			ToolOffsetTransform(ms);
			ms.feedRate = ConvertSpeedFromMmPerMin(DefaultFeedRate);	// ask for a good feed rate, we may have paused during a slow move
			gb.SetState((restoreZ) ? GCodeState::resuming3 : GCodeState::resuming2);
			NewSingleSegmentMoveAvailable(ms);
#endif
		}
		break;

	case GCodeState::resuming3:
		if (LockAllMovementSystemsAndWaitForStandstill(gb))
		{
			// We no longer restore the paused fan speeds automatically on resuming, because that messes up the print cooling fan speed if a tool change has been done
			// They can be restored manually in resume.g if required
#if SUPPORT_ASYNC_MOVES
			FilePosition earliestFileOffset = noFilePosition;

			for (MovementState& tempMs : moveStates)
			{
				tempMs.ReleaseNonToolAxesAndExtruders();
				tempMs.ResumeAfterPause();

				GCodeBuffer* fgb = GetFileGCode(tempMs.GetMsNumber());
				if (fgb->IsExecuting())
				{
					if (tempMs.GetMsNumber() == 0 || tempMs.GetPauseRestorePoint().filePos < earliestFileOffset)
					{
						earliestFileOffset = tempMs.GetPauseRestorePoint().filePos;
					}
					if (tempMs.GetMsNumber() == 0 || !FileGCode()->ExecutingAll())
					{
						fgb->LatestMachineState().feedRate = tempMs.GetPauseRestorePoint().feedRate;
						if (tempMs.pausedInMacro)
						{
							fgb->OriginalMachineState().firstCommandAfterRestart = true;
						}
					}
				}
			}

			// If the file input stream has been forked then we are good to go.
			// If File is executing both streams then we need to restart it from the earliest offset and using the movement system that was active at that point.
			if (FileGCode()->ExecutingAll() && earliestFileOffset != noFilePosition)
			{
				FileGCode()->RestartFrom(earliestFileOffset);
				const MovementSystemNumber msNumber = (moveStates[0].GetPauseRestorePoint().filePos > earliestFileOffset) ? 1
														: (moveStates[1].GetPauseRestorePoint().filePos > earliestFileOffset) ? 0
															: pausedMovementSystemNumber;
				FileGCode()->SetActiveQueueNumber(msNumber);
			}
#else
			ms.ResumeAfterPause();
			FileGCode()->LatestMachineState().feedRate = ms.GetPauseRestorePoint().feedRate;
			if (ms.pausedInMacro)
			{
				FileGCode()->OriginalMachineState().firstCommandAfterRestart = true;
			}
#endif
			reply.copy("Printing resumed");
			platform.Message(LogWarn, "Printing resumed\n");
			pauseState = PauseState::notPaused;
			gb.SetState(GCodeState::normal);
		}
		break;

	case GCodeState::cancelling:
		if (LockAllMovementSystemsAndWaitForStandstill(gb))		// wait until cancel.g has completely finished
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
# if SUPPORT_PANELDUE_FLASH
					FirmwareUpdater::UpdateModule(module, serialChannelForPanelDueFlashing, filenameString.GetRef());
					isFlashingPanelDue = (module == FirmwareUpdater::PanelDueFirmwareModule);
# else
					FirmwareUpdater::UpdateModule(module, 0, filenameString.GetRef());
# endif
					updating = true;
					break;
				}
			}
			if (!updating)
			{
# if SUPPORT_PANELDUE_FLASH
				isFlashingPanelDue = false;
# endif
				gb.SetState(GCodeState::flashing2);
			}
		}
# if SUPPORT_PANELDUE_FLASH
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
#if HAS_MASS_STORAGE
		if (firmwareUpdateModuleMap.IsBitSet(0))
		{
			// Update main firmware
			firmwareUpdateModuleMap.Clear();
			try
			{
				String<MaxFilenameLength> filenameString;
				bool dummy;
				gb.TryGetQuotedString('P', filenameString.GetRef(), dummy);
				reprap.UpdateFirmware(IAP_UPDATE_FILE, filenameString.c_str());
				// The above call does not return unless an error occurred
			}
			catch (const GCodeException&) { }
		}
#endif
		isFlashing = false;
		gb.SetState(GCodeState::normal);
		break;

	case GCodeState::stopping:			// here when a print has finished, need to execute stop.g
	case GCodeState::stoppingFromCode:
		if (LockAllMovementSystemsAndWaitForStandstill(gb))
		{
#if SUPPORT_ASYNC_MOVES
			gb.ExecuteAll();			// only fileGCode gets here so it needs to execute moves for all commands
#endif
			gb.SetState(GCodeState::stopped);
			if (!DoFileMacro(gb, STOP_G, false, (state == GCodeState::stoppingFromCode) ? SystemHelperMacroCode : AsyncSystemMacroCode))
			{
				reprap.GetHeat().SwitchOffAll(true);
			}
		}
		break;

	case GCodeState::stopped:
		reprap.GetPrintMonitor().StoppedPrint();
		gb.SetState(GCodeState::normal);
		break;

	// States used for grid probing
	case GCodeState::gridProbing1:		// ready to move to next grid probe point
		{
			// Move to the current probe point
			Move& move = reprap.GetMove();
			const HeightMap& hm = move.AccessHeightMap();
			if (hm.CanProbePoint(gridAxis0Index, gridAxis1Index))
			{
				const GridDefinition& grid = hm.GetGrid();
				const float axis0Coord = grid.GetCoordinate(0, gridAxis0Index);
				const float axis1Coord = grid.GetCoordinate(1, gridAxis1Index);
				const size_t axis0Num = grid.GetAxisNumber(0);
				const size_t axis1Num = grid.GetAxisNumber(1);
				AxesBitmap axes;
				axes.SetBit(axis0Num);
				axes.SetBit(axis1Num);
				float axesCoords[MaxAxes];
				memcpy(axesCoords, ms.coords, sizeof(axesCoords));					// copy current coordinates of all other axes in case they are relevant to IsReachable
				const auto zp = platform.GetZProbeOrDefault(currentZProbeNumber);
				zp->PrepareForUse(false);											// needed to calculate the actual trigger height when using a scanning Z probe
				axesCoords[axis0Num] = axis0Coord - zp->GetOffset(axis0Num);
				axesCoords[axis1Num] = axis1Coord - zp->GetOffset(axis1Num);
				axesCoords[Z_AXIS] =
#if SUPPORT_SCANNING_PROBES
									(zp->GetProbeType() == ZProbeType::scanningAnalog) ? zp->GetScanningHeight() :
#endif
										zp->GetStartingHeight(true);
				if (move.IsAccessibleProbePoint(axesCoords, axes))
				{
					SetMoveBufferDefaults(ms);
					ms.coords[axis0Num] = axesCoords[axis0Num];
					ms.coords[axis1Num] = axesCoords[axis1Num];
					ms.coords[Z_AXIS] = axesCoords[Z_AXIS];
					ms.feedRate = zp->GetTravelSpeed();
					ms.linearAxesMentioned = ms.rotationalAxesMentioned = true;		// assume that both linear and rotational axes might be moving
					NewSingleSegmentMoveAvailable(ms);

#if SUPPORT_SCANNING_PROBES
					if (zp->GetProbeType() == ZProbeType::scanningAnalog)
					{
						gb.SetState(GCodeState::gridScanning1);
					}
					else
#endif
					{
						InitialiseTaps(false);										// grid probing never does fast-then-slow probing
						gb.AdvanceState();
					}
				}
				else
				{
					platform.MessageF(WarningMessage, "Skipping grid point %c=%.1f, %c=%.1f because the Z probe cannot reach it\n", grid.GetAxisLetter(0), (double)axis0Coord, grid.GetAxisLetter(1), (double)axis1Coord);
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
		if (LockCurrentMovementSystemAndWaitForStandstill(gb))
		{
			gb.AdvanceState();
			if (platform.GetZProbeOrDefault(currentZProbeNumber)->GetProbeType() == ZProbeType::blTouch)
			{
				DeployZProbe(gb);
			}
		}
		break;

	case GCodeState::gridProbing2b:		// ready to probe the current grid probe point
		if (LockCurrentMovementSystemAndWaitForStandstill(gb))
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
					SetMoveBufferDefaults(ms);
					if (!platform.GetEndstops().EnableZProbe(currentZProbeNumber) || !zp->SetProbing(true))
					{
						gb.LatestMachineState().SetError("Failed to enable probe");
						gb.SetState(GCodeState::checkError);
						RetractZProbe(gb);
						break;
					}
					ms.checkEndstops = true;
					ms.reduceAcceleration = true;
					ms.coords[Z_AXIS] = -zp->GetDiveHeight(-1) + zp->GetActualTriggerHeight();
					ms.feedRate = zp->GetProbingSpeed(tapsDone);
					ms.linearAxesMentioned = true;
					NewSingleSegmentMoveAvailable(ms);
					gb.AdvanceState();
				}
			}
		}
		break;

	case GCodeState::gridProbing4:	// ready to lift the probe after probing the current grid probe point
		if (LockCurrentMovementSystemAndWaitForStandstill(gb))
		{
			doingManualBedProbe = false;
			++tapsDone;
			reprap.GetHeat().SuspendHeaters(false);
			const auto zp = platform.GetZProbeOrDefault(currentZProbeNumber);
			if (zp->GetProbeType() == ZProbeType::none)
			{
				// No Z probe, so we are doing manual mesh levelling. Take the current Z height as the height error.
				g30zHeightError = ms.coords[Z_AXIS];
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
				g30zHeightError = ms.coords[Z_AXIS] - zp->GetActualTriggerHeight();
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
		{
			const auto zp = platform.GetZProbeOrDefault(currentZProbeNumber);
			CheckIfMoreTapsNeeded(gb, *zp);

			// Move back up to the dive height
			SetMoveBufferDefaults(ms);
			ms.coords[Z_AXIS] = zp->GetStartingHeight(acceptReading, g30zHeightError);
			ms.feedRate = zp->GetTravelSpeed();
			ms.linearAxesMentioned = true;
			NewSingleSegmentMoveAvailable(ms);
			gb.AdvanceState();
		}
		break;

	case GCodeState::gridProbing5:	// finished probing a point and moved back to the dive height
		if (LockCurrentMovementSystemAndWaitForStandstill(gb))
		{
			if (acceptReading)
			{
				reprap.GetMove().AccessHeightMap().SetGridHeight(gridAxis0Index, gridAxis1Index, g30zHeightError);
				gb.AdvanceState();
			}
			else
			{
				// Tap again
				lastProbedTime = millis();
				g30PrevHeightError = g30zHeightError;
				gb.SetState(GCodeState::gridProbing2a);
			}
		}
		break;

	case GCodeState::gridProbing6:	// ready to compute the next probe point
		{
			const HeightMap& hm = reprap.GetMove().AccessHeightMap();
			if (gridAxis1Index & 1u)
			{
				// Odd row, so decreasing X
				if (gridAxis0Index == 0)
				{
					++gridAxis1Index;
				}
				else
				{
					--gridAxis0Index;
				}
			}
			else
			{
				// Even row, so increasing X
				if (gridAxis0Index + 1u == hm.GetGrid().NumAxisPoints(0))
				{
					++gridAxis1Index;
				}
				else
				{
					++gridAxis0Index;
				}
			}

			if (gridAxis1Index == hm.GetGrid().NumAxisPoints(1))
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
		// Finished probing or scanning the grid, and retracted the probe if necessary
		if (scanningResult != GCodeResult::ok)
		{
			stateMachineResult = scanningResult;
			// scanningResult may contain a CAN error code, or will be GCodeResult::error if it was a generic bad reading error
			if (scanningResult == GCodeResult::error)
			{
				reply.copy("Bad reading from scanning probe - try recalibrating the probe");
			}
			reprap.GetMove().AccessHeightMap().ClearGridHeights();
		}
		else
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

#if SUPPORT_SCANNING_PROBES

	// States used for grid scanning
	case GCodeState::gridScanning1:		// Here when we have moved to the first accessible point at the start of a row
		if (LockCurrentMovementSystemAndWaitForStandstill(gb))
		{
			// Iterate through points on this row looking for the last reachable one
			HeightMap& hm = reprap.GetMove().AccessHeightMap();
			const auto zp = platform.GetZProbeOrDefault(currentZProbeNumber);
			zp->SetProbing(true);
			const GridDefinition& grid = hm.GetGrid();
			const size_t axis0Num = grid.GetAxisNumber(0);
			const size_t axis1Num = grid.GetAxisNumber(1);
			lastAxis0Index = gridAxis0Index;
			for (;;)
			{
				size_t newAxis0Index;
				if (gridAxis1Index & 1u)
				{
					// Odd row, so decreasing X
					if (lastAxis0Index == 0)
					{
						break;
					}
					newAxis0Index = lastAxis0Index - 1;
				}
				else
				{
					// Even row, so increasing X
					newAxis0Index = lastAxis0Index + 1;
					if (newAxis0Index == hm.GetGrid().NumAxisPoints(0))
					{
						break;
					}
				}
				if (!hm.CanProbePoint(newAxis0Index, gridAxis1Index))
				{
					break;
				}

				AxesBitmap axes;
				axes.SetBit(axis0Num);
				axes.SetBit(axis1Num);
				float axesCoords[MaxAxes];
				memcpy(axesCoords, ms.coords, sizeof(axesCoords));					// copy current coordinates of all other axes in case they are relevant to IsReachable
				axesCoords[axis0Num] = grid.GetCoordinate(0, newAxis0Index) - zp->GetOffset(axis0Num);
				axesCoords[axis1Num] = grid.GetCoordinate(1, gridAxis1Index) - zp->GetOffset(axis1Num);
				axesCoords[Z_AXIS] = zp->GetScanningHeight();
				if (!reprap.GetMove().IsAccessibleProbePoint(axesCoords, axes))
				{
					break;
				}
				lastAxis0Index = newAxis0Index;
			}

			gb.AdvanceState();

			// We are over the point given by [gridAxis0index, gridAxis1index]. Scan up to [lastAxis0Index, gridAxis1index]. This may be a single point.
			float heightError;
			const GCodeResult rslt = zp->GetCalibratedReading(heightError);
			if (rslt != GCodeResult::ok)
			{
				if (scanningResult == GCodeResult::ok)
				{
					scanningResult = rslt;
				}
			}
			else
			{
				hm.SetGridHeight(gridAxis0Index, gridAxis1Index, -heightError);

				if (lastAxis0Index != gridAxis0Index)			// if more than one point
				{
					SetMoveBufferDefaults(ms);
					ms.coords[axis0Num] = grid.GetCoordinate(0, lastAxis0Index) - zp->GetOffset(axis0Num);
					ms.coords[axis1Num] = grid.GetCoordinate(1, gridAxis1Index) - zp->GetOffset(axis1Num);
					ms.coords[Z_AXIS] = zp->GetScanningHeight();
					ms.feedRate = zp->GetScanningSpeed();
					ms.linearAxesMentioned = platform.IsAxisLinear(axis0Num);
					ms.rotationalAxesMentioned = platform.IsAxisRotational(axis0Num);
					ms.segmentsLeftToStartAt = ms.totalSegments = (unsigned int)abs((int)lastAxis0Index - (int)gridAxis0Index);
					ms.firstSegmentFractionToSkip = 0.0;
					ms.scanningProbeMove = true;

					// Adjust the axis 0 index so that the laser task will store the reading at the correct location in the grid
					if (gridAxis1Index & 1u)
					{
						--gridAxis0Index;
					}
					else
					{
						++gridAxis0Index;
					}

					NewMoveAvailable(ms);
				}
			}
		}
		break;

	case GCodeState::gridScanning2:		// Here when we have scanned a row
		if (LockCurrentMovementSystemAndWaitForStandstill(gb))
		{
			const auto zp = platform.GetZProbeOrDefault(currentZProbeNumber);
			zp->SetProbing(false);

			if (scanningResult != GCodeResult::ok)
			{
				gb.SetState(GCodeState::gridProbing7);
				RetractZProbe(gb);
				break;
			}

			HeightMap& hm = reprap.GetMove().AccessHeightMap();

			// Advance to the start or end of the next row
			++gridAxis1Index;
			if (gridAxis1Index == hm.GetGrid().NumAxisPoints(1))
			{
				// Done all the points
				gb.SetState(GCodeState::gridProbing7);
				RetractZProbe(gb);
			}
			else
			{
				gridAxis0Index = (gridAxis1Index & 1u)
									? hm.GetGrid().NumAxisPoints(0) - 1				// new row number is odd so go to the end of it
									: 0;											// new row is even, so go to start of it
				gb.SetState(GCodeState::gridProbing1);
			}
		}
		break;

#endif

	// States used for G30 probing
	case GCodeState::probingAtPoint0:
		// Initial state when executing G30 with a P parameter. The Z probe has been deployed. Start by moving to the dive height at the current position.
		if (LockCurrentMovementSystemAndWaitForStandstill(gb))
		{
			SetMoveBufferDefaults(ms);
			const auto zp = platform.GetZProbeOrDefault(currentZProbeNumber);
			ms.coords[Z_AXIS] = zp->GetStartingHeight(true);
			ms.feedRate = zp->GetTravelSpeed();
			ms.linearAxesMentioned = true;
			NewSingleSegmentMoveAvailable(ms);
			gb.AdvanceState();
		}
		break;

	case GCodeState::probingAtPoint1:
		// The move to raise/lower the head to the correct dive height has been commanded.
		if (LockCurrentMovementSystemAndWaitForStandstill(gb))
		{
			// Head is at the dive height but needs to be moved to the correct XY position. The XY coordinates have already been stored.
			SetMoveBufferDefaults(ms);
			(void)reprap.GetMove().GetProbeCoordinates(g30ProbePointIndex, ms.coords[X_AXIS], ms.coords[Y_AXIS], true);
			const auto zp = platform.GetZProbeOrDefault(currentZProbeNumber);
			ms.coords[Z_AXIS] = zp->GetStartingHeight(true);
			ms.feedRate = zp->GetTravelSpeed();
			ms.linearAxesMentioned = ms.rotationalAxesMentioned = true;		// assume that both linear and rotational axes might be moving
			NewSingleSegmentMoveAvailable(ms);

			InitialiseTaps(false);									// don't do fast-then-slow probing
			gb.AdvanceState();
		}
		break;

	case GCodeState::probingAtPoint2a:								// note we return to this state when doing the second and subsequent taps
		// Executing G30 with a P parameter. The move to put the head at the specified XY coordinates has been commanded.
		// OR initial state when executing G30 with no P parameter (must call InitialiseTaps first)
		if (LockCurrentMovementSystemAndWaitForStandstill(gb))
		{
			gb.AdvanceState();
			if (platform.GetZProbeOrDefault(currentZProbeNumber)->GetProbeType() == ZProbeType::blTouch)	// bltouch needs to be redeployed prior to each probe point
			{
				DeployZProbe(gb);
			}
		}
		break;

	case GCodeState::probingAtPoint2b:
		if (LockCurrentMovementSystemAndWaitForStandstill(gb))
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
						reprap.GetMove().SetZBedProbePoint(g30ProbePointIndex, zp->GetDiveHeight(tapsDone), true, true);
					}
					gb.SetState(GCodeState::checkError);									// no point in doing anything else
					RetractZProbe(gb);
				}
				else
				{
					zProbeTriggered = false;
					SetMoveBufferDefaults(ms);
					if (!platform.GetEndstops().EnableZProbe(currentZProbeNumber) || !zp->SetProbing(true))
					{
						gb.LatestMachineState().SetError("Failed to enable probe");
						gb.SetState(GCodeState::checkError);
						RetractZProbe(gb);
						break;
					}

					ms.checkEndstops = true;
					ms.reduceAcceleration = true;
					ms.coords[Z_AXIS] = (IsAxisHomed(Z_AXIS))
												? platform.AxisMinimum(Z_AXIS) - zp->GetDiveHeight(-1) + zp->GetActualTriggerHeight()	// Z axis has been homed, so no point in going very far
												: -1.1 * platform.AxisTotalLength(Z_AXIS);	// Z axis not homed yet, so treat this as a homing move
					ms.feedRate = zp->GetProbingSpeed(tapsDone);
					ms.linearAxesMentioned = true;
					NewSingleSegmentMoveAvailable(ms);
					gb.AdvanceState();
				}
			}
		}
		break;

	case GCodeState::probingAtPoint4:
		// Executing G30. The probe wasn't triggered at the start of the move, and the probing move has been commanded.
		if (LockCurrentMovementSystemAndWaitForStandstill(gb))
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
				g30zHeightError = ms.coords[Z_AXIS];
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
					reprap.GetMove().GetCurrentMachinePosition(m, ms.GetMsNumber(), false);		// get height without bed compensation
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
					ms.coords[Z_AXIS] = zp->GetActualTriggerHeight();
#if SUPPORT_ASYNC_MOVES
					ms.OwnedAxisCoordinateUpdated(Z_AXIS);
#endif
					reprap.GetMove().SetNewPosition(ms.coords, ms.GetMsNumber(), false);

					// Find the coordinates of the Z probe to pass to SetZeroHeightError
					float tempCoords[MaxAxes];
					memcpyf(tempCoords, ms.coords, ARRAY_SIZE(tempCoords));
					tempCoords[X_AXIS] += zp->GetOffset(X_AXIS);
					tempCoords[Y_AXIS] += zp->GetOffset(Y_AXIS);
					reprap.GetMove().SetZeroHeightError(tempCoords);
					ToolOffsetInverseTransform(ms);

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
		{
			// Decode whether we need more taps at the current point. This affects the dive height that we move back up to.
			const auto zp = platform.GetZProbeOrDefault(currentZProbeNumber);
			CheckIfMoreTapsNeeded(gb, *zp);

			// Move back up to the dive height before we change anything, in particular before we adjust leadscrews
			SetMoveBufferDefaults(ms);
			ms.coords[Z_AXIS] = zp->GetStartingHeight(acceptReading, g30zHeightError);
			ms.feedRate = zp->GetTravelSpeed();
			ms.linearAxesMentioned = true;
			NewSingleSegmentMoveAvailable(ms);
			gb.AdvanceState();
		}
		break;

	case GCodeState::probingAtPoint5:
		// Here when we have moved the head back up to the dive height
		if (LockCurrentMovementSystemAndWaitForStandstill(gb))
		{
			// See whether we need to do any more taps
			const auto zp = platform.GetZProbeOrDefault(currentZProbeNumber);
			if (!acceptReading)
			{
				// Tap again
				g30PrevHeightError = g30zHeightError;
				lastProbedTime = millis();
				gb.SetState(GCodeState::probingAtPoint2a);
				break;
			}

			if (g30ProbePointIndex >= 0)
			{
				reprap.GetMove().SetZBedProbePoint(g30ProbePointIndex, g30zHeightError, true, hadProbingError);
			}
			else
			{
				// Setting the Z height with G30
				ms.coords[Z_AXIS] -= g30zHeightError;
				reprap.GetMove().SetNewPosition(ms.coords, ms.GetMsNumber(), false);

				// Find the coordinates of the Z probe to pass to SetZeroHeightError
				float tempCoords[MaxAxes];
				memcpyf(tempCoords, ms.coords, ARRAY_SIZE(tempCoords));
				tempCoords[X_AXIS] += zp->GetOffset(X_AXIS);
				tempCoords[Y_AXIS] += zp->GetOffset(Y_AXIS);
				reprap.GetMove().SetZeroHeightError(tempCoords);
				ToolOffsetInverseTransform(ms);
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
		if (LockCurrentMovementSystemAndWaitForStandstill(gb))		// retracting the Z probe
		{
			if (g30SValue == 1)
			{
				// G30 with a silly Z value and S=1 is equivalent to G30 with no parameters in that it sets the current Z height
				// This is useful because it adjusts the XY position to account for the probe offset.
				ms.coords[Z_AXIS] -= g30zHeightError;
				reprap.GetMove().SetNewPosition(ms.coords, ms.GetMsNumber(), false);
				ToolOffsetInverseTransform(ms);
			}
			else if (g30SValue >= -1)
			{
				if (reprap.GetMove().FinishedBedProbing(ms.GetMsNumber(), g30SValue, reply))
				{
					stateMachineResult = GCodeResult::error;
				}
				else if (reprap.GetMove().GetKinematics().SupportsAutoCalibration())
				{
					zDatumSetByProbing = true;			// if we successfully auto calibrated or adjusted leadscrews, we've set the Z datum by probing
					// Auto calibration may have adjusted the motor positions and the geometry, so the head may now be at a different position
#if SUPPORT_ASYNC_MOVES
					ms.SaveOwnAxisCoordinates();
#endif
					UpdateUserPositionFromMachinePosition(gb, ms);
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
			if (ms.currentTool == nullptr)
			{
				gb.LatestMachineState().SetError("Tool was deselected during G30 S-2 command");
			}
			else
			{
				ms.currentTool->SetOffset(Z_AXIS, -g30zHeightError, true);
				ToolOffsetInverseTransform(ms);			// update user coordinates to reflect the new tool offset
			}
		}
		else
		{
			// Just print the stop height
			reply.printf("Stopped at height %.3f mm", (double)platform.GetZProbeOrDefault(currentZProbeNumber)->GetLastStoppedHeight());
		}
		gb.SetState(GCodeState::normal);
		break;

	case GCodeState::straightProbe0:					// ready to deploy the probe
		if (LockCurrentMovementSystemAndWaitForStandstill(gb))
		{
			gb.AdvanceState();
			currentZProbeNumber = straightProbeSettings.GetZProbeToUse();
			DeployZProbe(gb);
		}
		break;

	case GCodeState::straightProbe1:
		if (LockCurrentMovementSystemAndWaitForStandstill(gb))
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
						SetMoveBufferDefaults(ms);
						if (!platform.GetEndstops().EnableZProbe(straightProbeSettings.GetZProbeToUse(), probingAway) || !zp->SetProbing(true))
						{
							gb.LatestMachineState().SetError("Failed to enable probe");
							gb.SetState(GCodeState::checkError);
							RetractZProbe(gb);
							break;
						}

						ms.checkEndstops = true;
						ms.reduceAcceleration = true;
						straightProbeSettings.SetCoordsToTarget(ms.coords);
						ms.feedRate = zp->GetProbingSpeed(0);
						ms.linearAxesMentioned = ms.rotationalAxesMentioned = true;
						NewSingleSegmentMoveAvailable(ms);
						gb.AdvanceState();
					}
				}
			}
		}
		break;

	case GCodeState::straightProbe3:
		// Executing G38. The probe wasn't in target state at the start of the move, and the probing move has been commanded.
		if (LockCurrentMovementSystemAndWaitForStandstill(gb))
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

#if SUPPORT_SCANNING_PROBES

	// Scanning probe calibration states
	case GCodeState::probeCalibration1:
		// We just deployed the Z probe, read to start calibrating. Move t the trigger height plus the scanning range.
		if (LockCurrentMovementSystemAndWaitForStandstill(gb))
		{
			SetMoveBufferDefaults(ms);
			{
				const auto zp = platform.GetZProbeOrDefault(currentZProbeNumber);
				ms.coords[Z_AXIS] = calibrationStartingHeight;
				ms.feedRate = zp->GetTravelSpeed();
			}
			ms.linearAxesMentioned = true;
			numCalibrationReadingsTaken = 0;
			NewSingleSegmentMoveAvailable(ms);
			gb.AdvanceState();
		}
		break;

	case GCodeState::probeCalibration2:
		// We have moved to the trigger height plus scanning range
		if (LockCurrentMovementSystemAndWaitForStandstill(gb))
		{
			const auto zp = platform.GetZProbeOrDefault(currentZProbeNumber);
			if (numCalibrationReadingsTaken == 0)
			{
				zp->SetProbing(true);
			}

			delay(20);																// allow some settling time
			float dummyHeightError;
			(void)zp->GetCalibratedReading(dummyHeightError);						// needed to update the raw reading
			const uint32_t reading = zp->GetRawReading();
			if (reading == 0)
			{
				// A reading of zero indicates an error e.g. LDC1612 amplitude error
				reply.copy("sensor error during calibration");
				stateMachineResult = GCodeResult::error;
				gb.SetState(GCodeState::normal);
			}
			else
			{
				calibrationReadings[numCalibrationReadingsTaken] = (int32_t)reading;
				++numCalibrationReadingsTaken;
				if (numCalibrationReadingsTaken == numPointsToCollect)
				{
					zp->SetProbing(false);
					gb.AdvanceState();
					RetractZProbe(gb);
				}
				else
				{
					SetMoveBufferDefaults(ms);
					ms.coords[Z_AXIS] = calibrationStartingHeight - (numCalibrationReadingsTaken * heightChangePerPoint);
					ms.feedRate = zp->GetProbingSpeed(1);
					ms.linearAxesMentioned = true;
					NewSingleSegmentMoveAvailable(ms);
				}
			}
		}
		break;

	case GCodeState::probeCalibration3:
		// We have finished taking calibration readings
		{
			auto zp = platform.GetZProbeOrDefault(currentZProbeNumber);
			zp->CalibrateScanningProbe(calibrationReadings, numCalibrationReadingsTaken, heightChangePerPoint, reply);
		}
		gb.SetState(GCodeState::normal);
		break;

#endif

	// Firmware retraction/un-retraction states
	case GCodeState::doingFirmwareRetraction:
		// We just did the retraction part of a firmware retraction, now we need to do the Z hop
		if (ms.segmentsLeft == 0)
		{
			Tool * const t = ms.currentTool;
			if (t != nullptr)								// this should always be true
			{
#if SUPPORT_ASYNC_MOVES
				// We already allocated the Z axes to this MS when we began the retraction, so no need to do it here
#endif
				SetMoveBufferDefaults(ms);
				ms.movementTool = t;
				reprap.GetMove().GetCurrentUserPosition(ms.coords, ms.GetMsNumber(), 0, t);
				memcpyf(ms.initialCoords, ms.coords, ARRAY_SIZE(ms.initialCoords));
				const AxesBitmap zAxes = t->GetZAxisMap();

				// See if we can apply the requested Z hop without exceeding machine limits
				float zHopToUse = t->GetConfiguredRetractHop();
				zAxes.Iterate([&ms, &zHopToUse](unsigned int axis, unsigned int)->void { ms.coords[axis] += zHopToUse; });
				if (reprap.GetMove().GetKinematics().LimitPosition(ms.coords, nullptr, numVisibleAxes, AxesBitmap::MakeFromBits(Z_AXIS), true, true) != LimitPositionResult::ok)
				{
					// We can't apply Z hop to all the Z axes without exceeding machine limits
					zAxes.Iterate([&ms, &zHopToUse](unsigned int axis, unsigned int)->void { zHopToUse = min<float>(zHopToUse, ms.coords[axis] - ms.initialCoords[axis]); });
					if (zHopToUse <= 0.0)
					{
						gb.SetState(GCodeState::normal);
						break;
					}
					zAxes.Iterate([&ms, zHopToUse](unsigned int axis, unsigned int)->void { ms.coords[axis] = ms.initialCoords[axis] + zHopToUse; });
				}

				t->SetActualZHop(zHopToUse);
				ms.feedRate = ConvertSpeedFromMmPerSec(ImpossiblyHighFeedRate);		// we rely on the DDA init code to limit the feed rate to what is achievable on each axis;
				ms.filePos = gb.GetJobFilePosition();
				ms.canPauseAfter = false;											// don't pause after a retraction because that could cause too much retraction
				ms.linearAxesMentioned = true;
				NewSingleSegmentMoveAvailable(ms);
			}
			gb.SetState(GCodeState::normal);
		}
		break;

	case GCodeState::doingFirmwareUnRetraction:
		// We just undid the Z-hop part of a firmware un-retraction, now we need to do the un-retract
		if (ms.segmentsLeft == 0)
		{
			const Tool * const t = ms.currentTool;
			if (t != nullptr && t->DriveCount() != 0)
			{
				SetMoveBufferDefaults(ms);
				ms.movementTool = t;
				reprap.GetMove().GetCurrentUserPosition(ms.coords, ms.GetMsNumber(), 0, ms.currentTool);
				for (size_t i = 0; i < t->DriveCount(); ++i)
				{
					ms.coords[ExtruderToLogicalDrive(t->GetDrive(i))] = t->GetRetractLength() + t->GetRetractExtra();
				}
				ms.feedRate = t->GetUnRetractSpeed() * t->DriveCount();
				ms.filePos = gb.GetJobFilePosition();
				ms.canPauseAfter = true;
				NewSingleSegmentMoveAvailable(ms);
			}
			gb.SetState(GCodeState::normal);
		}
		break;

	case GCodeState::loadingFilament:
		// We just returned from the filament load macro
		if (ms.currentTool != nullptr)
		{
			ms.currentTool->GetFilament()->Load(filamentToLoad);
			if (reprap.Debug(Module::Gcodes))
			{
				platform.MessageF(LoggedGenericMessage, "Filament %s loaded", filamentToLoad);
			}
		}
		gb.SetState(GCodeState::normal);
		break;

	case GCodeState::unloadingFilament:
		// We just returned from the filament unload macro
		if (ms.currentTool != nullptr)
		{
			if (reprap.Debug(Module::Gcodes))
			{
				platform.MessageF(LoggedGenericMessage, "Filament %s unloaded", ms.currentTool->GetFilament()->GetName());
			}
			ms.currentTool->GetFilament()->Unload();
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
				platform.MessageF(gb.GetResponseMessageType(), "SD write speed for %.1fMByte file was %.2fMBytes/sec\n", (double)fileMbytes, (double)mbPerSec);
				sdTimingFile->Close();

				sdTimingFile = platform.OpenFile(Platform::GetGCodeDir(), TimingFileName, OpenMode::read);
				if (sdTimingFile == nullptr)
				{
					(void)platform.Delete(Platform::GetGCodeDir(), TimingFileName);
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
				(void)platform.Delete(Platform::GetGCodeDir(), TimingFileName);
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
				reply.printf("SD read speed for %.1fMByte file was %.2fMBytes/sec", (double)fileMbytes, (double)mbPerSec);
				(void)platform.Delete(Platform::GetGCodeDir(), TimingFileName);
				gb.SetState(GCodeState::normal);
				break;
			}

			const unsigned int bytesToRead = min<size_t>(reply.Capacity(), timingBytesRequested - timingBytesWritten);
			if (sdTimingFile->Read(reply.Pointer(), bytesToRead) != (int)bytesToRead)
			{
				sdTimingFile->Close();
				(void)platform.Delete(Platform::GetGCodeDir(), TimingFileName);
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
			else
			{
				const PrintPausedReason pauseReason = Event::GetDefaultPauseReason();
				// In the following, if DoPause fails because it can't get the movement lock then it will not change the state, so we will return here to try again
				(void)DoAsynchronousPause(gb, pauseReason, (pauseReason == PrintPausedReason::driverError) ? GCodeState::eventPausing2 : GCodeState::eventPausing1);
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
		gb.LatestMachineState().RetrieveStateMachineResult(gb, reply, stateMachineResult);
		HandleReply(gb, stateMachineResult, reply.c_str());

		CheckForDeferredPause(gb);
	}

#if HAS_SBC_INTERFACE
	if (reportPause)
	{
		FileGCode()->Invalidate();
# if SUPPORT_ASYNC_MOVES
		File2GCode()->Invalidate();
# endif
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
