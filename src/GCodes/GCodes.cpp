/****************************************************************************************************

 RepRapFirmware - G Codes

 This class interprets G Codes from one or more sources, and calls the functions in Move, Heat etc
 that drive the machine to do what the G Codes command.

 Most of the functions in here are designed not to wait, and they return a boolean.  When you want them to do
 something, you call them.  If they return false, the machine can't do what you want yet.  So you go away
 and do something else.  Then you try again.  If they return true, the thing you wanted done has been done.

 -----------------------------------------------------------------------------------------------------

 Version 0.1

 13 February 2013

 Adrian Bowyer
 RepRap Professional Ltd
 http://reprappro.com

 Licence: GPL

 ****************************************************************************************************/

#include "GCodes.h"
#include "GCodeBuffer.h"
#include "GCodeQueue.h"
#include "Heating/Heat.h"
#include "Platform.h"
#include "Movement/Move.h"
#include "Scanner.h"
#include "PrintMonitor.h"
#include "RepRap.h"
#include "Tools/Tool.h"

#ifdef DUET_NG
#include "FirmwareUpdater.h"
#endif

const char GCodes::axisLetters[MaxAxes] = AXES_('X', 'Y', 'Z', 'U', 'V', 'W', 'A', 'B', 'C');

const size_t gcodeReplyLength = 2048;			// long enough to pass back a reasonable number of files in response to M20

GCodes::GCodes(Platform& p) :
	platform(p), active(false), isFlashing(false),
	fileBeingHashed(nullptr), lastWarningMillis(0)
{
	httpInput = new RegularGCodeInput(true);
	telnetInput = new RegularGCodeInput(true);
	fileInput = new FileGCodeInput();
	serialInput = new StreamGCodeInput(SERIAL_MAIN_DEVICE);
	auxInput = new StreamGCodeInput(SERIAL_AUX_DEVICE);

	httpGCode = new GCodeBuffer("http", HTTP_MESSAGE, false);
	telnetGCode = new GCodeBuffer("telnet", TELNET_MESSAGE, true);
	fileGCode = new GCodeBuffer("file", GENERIC_MESSAGE, true);
	serialGCode = new GCodeBuffer("serial", HOST_MESSAGE, true);
	auxGCode = new GCodeBuffer("aux", AUX_MESSAGE, false);
	daemonGCode = new GCodeBuffer("daemon", GENERIC_MESSAGE, false);
	queuedGCode = new GCodeBuffer("queue", GENERIC_MESSAGE, false);
#ifdef DUET_NG
	autoPauseGCode = new GCodeBuffer("autopause", GENERIC_MESSAGE, false);
#endif
	codeQueue = new GCodeQueue();
}

void GCodes::Exit()
{
	active = false;
}

void GCodes::Init()
{
	numVisibleAxes = numTotalAxes = XYZ_AXES;			// must set this up before calling Reset()
	numExtruders = MaxExtruders;
	Reset();

	distanceScale = 1.0;
	arcSegmentLength = DefaultArcSegmentLength;
	rawExtruderTotal = 0.0;
	for (size_t extruder = 0; extruder < MaxExtruders; extruder++)
	{
		lastRawExtruderPosition[extruder] = 0.0;
		rawExtruderTotalByDrive[extruder] = 0.0;
	}
	eofString = EOF_STRING;
	eofStringCounter = 0;
	eofStringLength = strlen(eofString);
	runningConfigFile = false;
	doingToolChange = false;
	toolChangeParam = DefaultToolChangeParam;
	active = true;
	fileSize = 0;
	longWait = platform.Time();
	limitAxes = true;
	SetAllAxesNotHomed();
	for (size_t i = 0; i < NUM_FANS; ++i)
	{
		pausedFanSpeeds[i] = 0.0;
	}
	lastDefaultFanSpeed = pausedDefaultFanSpeed = 0.0;

	retractLength = DefaultRetractLength;
	retractExtra = 0.0;
	currentZHop = retractHop = 0.0;
	retractSpeed = unRetractSpeed = DefaultRetractSpeed * SecondsToMinutes;
	isRetracted = false;
	lastAuxStatusReportType = -1;						// no status reports requested yet

#if SUPPORT_SCANNER
	reprap.GetScanner().SetGCodeBuffer(serialGCode);
#endif
}

// This is called from Init and when doing an emergency stop
void GCodes::Reset()
{
	// Here we could reset the input sources as well, but this would mess up M122\nM999
	// because both codes are sent at once from the web interface. Hence we don't do this here

	httpGCode->Reset();
	telnetGCode->Reset();
	fileGCode->Reset();
	serialGCode->Reset();
	auxGCode->Reset();
	auxGCode->SetCommsProperties(1);					// by default, we require a checksum on the aux port
	daemonGCode->Reset();
	queuedGCode->Reset();
#ifdef DUET_NG
	autoPauseGCode->Reset();
#endif

	nextGcodeSource = 0;

	fileToPrint.Close();
	fileBeingWritten = nullptr;
	speedFactor = SecondsToMinutes;						// default is just to convert from mm/minute to mm/second

	for (size_t i = 0; i < MaxExtruders; ++i)
	{
		extrusionFactors[i] = volumetricExtrusionFactors[i] = 1.0;
	}
	for (size_t i = 0; i < MaxAxes; ++i)
	{
		axisOffsets[i] = 0.0;
		axisScaleFactors[i] = 1.0;
	}

	ClearMove();
	ClearBabyStepping();
	moveBuffer.xAxes = DefaultXAxisMapping;
	moveBuffer.yAxes = DefaultYAxisMapping;
#if SUPPORT_IOBITS
	moveBuffer.ioBits = 0;
#endif

	reprap.GetMove().GetKinematics().GetAssumedInitialPosition(numVisibleAxes, moveBuffer.coords);
	ToolOffsetInverseTransform(moveBuffer.coords, currentUserPosition);

	for (size_t i = numTotalAxes; i < DRIVES; ++i)
	{
		moveBuffer.coords[i] = 0.0;
	}

	pauseRestorePoint.Init();
	toolChangeRestorePoint.Init();

	for (size_t i = 0; i < MaxTriggers; ++i)
	{
		triggers[i].Init();
	}
	triggersPending = 0;

	simulationMode = 0;
	simulationTime = 0.0;
	isPaused = false;
#ifdef DUET_NG
	isAutoPaused = resumeInfoSaved = false;
#endif
	doingToolChange = false;
	doingManualBedProbe = false;
	pausePending = false;
	probeIsDeployed = false;
	moveBuffer.filePos = noFilePosition;
	lastEndstopStates = platform.GetAllEndstopStates();
	firmwareUpdateModuleMap = 0;

	codeQueue->Clear();
	cancelWait = isWaiting = displayNoToolWarning = displayDeltaNotHomedWarning = false;

	for (size_t i = 0; i < NumResources; ++i)
	{
		resourceOwners[i] = nullptr;
	}
}

bool GCodes::DoingFileMacro() const
{
	for (const GCodeBuffer *gb : gcodeSources)
	{
		if (gb->IsDoingFileMacro())
		{
			return true;
		}
	}
	return false;
}

float GCodes::FractionOfFilePrinted() const
{
	const FileData& fileBeingPrinted = fileGCode->OriginalMachineState().fileState;
	return (fileBeingPrinted.IsLive()) ? fileBeingPrinted.FractionRead() : -1.0;
}

// Start running the config file
// We use triggerCGode as the source to prevent any triggers being executed until we have finished
bool GCodes::RunConfigFile(const char* fileName)
{
	runningConfigFile = DoFileMacro(*daemonGCode, fileName, false);
	return runningConfigFile;
}

// Return true if the daemon is busy running config.g or a trigger file
bool GCodes::IsDaemonBusy() const
{
	return daemonGCode->MachineState().fileState.IsLive();
}

// Copy the feed rate etc. from the daemon to the input channels
void GCodes::CopyConfigFinalValues(GCodeBuffer& gb)
{
	for (size_t i = 0; i < ARRAY_SIZE(gcodeSources); ++i)
	{
		gcodeSources[i]->MachineState().CopyStateFrom(gb.MachineState());
	}
}

void GCodes::Spin()
{
	if (!active)
	{
		return;
	}

	CheckTriggers();

	// Get the GCodeBuffer that we want to work from
	GCodeBuffer& gb = *(gcodeSources[nextGcodeSource]);

	// Set up a buffer for the reply
	char replyBuffer[gcodeReplyLength];
	StringRef reply(replyBuffer, ARRAY_SIZE(replyBuffer));
	reply.Clear();

	if (gb.GetState() == GCodeState::normal)
	{
		if (gb.MachineState().messageAcknowledged)
		{
			const bool wasCancelled = gb.MachineState().messageCancelled;
			Pop(gb);

			if (wasCancelled)
			{
				if (gb.MachineState().previous == nullptr)
				{
					CancelPrint();
				}
				else
				{
					FileMacroCyclesReturn(gb);
				}
			}
		}
		else
		{
			StartNextGCode(gb, reply);
		}
	}
	else
	{
		// Perform the next operation of the state machine for this gcode source
		bool error = false;

		switch (gb.GetState())
		{
		case GCodeState::waitingForMoveToComplete:
			if (LockMovementAndWaitForStandstill(gb))		// movement should already be locked, but we need to wait for standstill and fetch the current position
			{
				// Check whether we made any G1 S3 moves and need to set the axis limits
				for (size_t axis = 0; axis < numVisibleAxes; ++axis)
				{
					if (IsBitSet<AxesBitmap>(axesToSenseLength, axis))
					{
						EndStopType stopType;
						bool dummy;
						platform.GetEndStopConfiguration(axis, stopType, dummy);
						if (stopType == EndStopType::highEndStop)
						{
							platform.SetAxisMaximum(axis, moveBuffer.coords[axis]);
						}
						else if (stopType == EndStopType::lowEndStop)
						{
							platform.SetAxisMinimum(axis, moveBuffer.coords[axis]);
						}
					}
				}
				gb.SetState(GCodeState::normal);
			}
			break;

		case GCodeState::homing1:
			if (toBeHomed == 0)
			{
				gb.SetState(GCodeState::normal);
			}
			else
			{
				AxesBitmap mustHomeFirst;
				const char *nextHomingFile = reprap.GetMove().GetKinematics().GetHomingFileName(toBeHomed, axesHomed, numVisibleAxes, mustHomeFirst);
				if (nextHomingFile == nullptr)
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
					if (!DoFileMacro(gb, nextHomingFile, false))
					{
						reply.printf("Homing file %s not found", nextHomingFile);
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
			gb.AdvanceState();
			if ((toolChangeParam & TFreeBit) != 0)
			{
				const Tool * const oldTool = reprap.GetCurrentTool();
				if (oldTool != nullptr && AllAxesAreHomed())
				{
					scratchString.printf("tfree%d.g", oldTool->Number());
					DoFileMacro(gb, scratchString.Pointer(), false);
				}
			}
			break;

		case GCodeState::toolChange1:		// Release the old tool (if any), then run tpre for the new tool
		case GCodeState::m109ToolChange1:	// Release the old tool (if any), then run tpre for the new tool
			{
				const Tool * const oldTool = reprap.GetCurrentTool();
				if (oldTool != nullptr)
				{
					reprap.StandbyTool(oldTool->Number());
				}
			}
			gb.AdvanceState();
			if (reprap.GetTool(newToolNumber) != nullptr && AllAxesAreHomed() && (toolChangeParam & TPreBit) != 0)
			{
				scratchString.printf("tpre%d.g", newToolNumber);
				DoFileMacro(gb, scratchString.Pointer(), false);
			}
			break;

		case GCodeState::toolChange2:		// Select the new tool (even if it doesn't exist - that just deselects all tools) and run tpost
		case GCodeState::m109ToolChange2:	// Select the new tool (even if it doesn't exist - that just deselects all tools) and run tpost
			reprap.SelectTool(newToolNumber);
			gb.AdvanceState();
			if (AllAxesAreHomed())
			{
				if (reprap.GetTool(newToolNumber) != nullptr && (toolChangeParam & TPostBit) != 0)
				{
					scratchString.printf("tpost%d.g", newToolNumber);
					DoFileMacro(gb, scratchString.Pointer(), false);
				}
			}
			break;

		case GCodeState::toolChangeComplete:
			doingToolChange = false;
			gb.SetState(GCodeState::normal);
			break;

		case GCodeState::m109ToolChangeComplete:
			doingToolChange = false;
			UnlockAll(gb);									// allow movement again
			if (cancelWait || ToolHeatersAtSetTemperatures(reprap.GetCurrentTool(), gb.MachineState().waitWhileCooling))
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
				gb.SetState(GCodeState::pausing2);
				if (AllAxesAreHomed())
				{
#ifdef DUET_NG
					DoFileMacro(gb, (isAutoPaused) ? POWER_FAIL_G : PAUSE_G, true);
#else
					DoFileMacro(gb, PAUSE_G, true);
#endif
				}
			}
			break;

		case GCodeState::pausing2:
			if (LockMovementAndWaitForStandstill(gb))
			{
#ifdef DUET_NG
				reprap.GetHeat().SuspendHeaters(false);			// resume the heaters, we may have turned them off while we executed the pause macro
#endif
				reply.copy("Printing paused");
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
				for (size_t drive = 0; drive < numVisibleAxes; ++drive)
				{
					currentUserPosition[drive] =  pauseRestorePoint.moveCoords[drive];
				}
				ToolOffsetTransform(currentUserPosition, moveBuffer.coords);
				for (size_t drive = numTotalAxes; drive < DRIVES; ++drive)
				{
					moveBuffer.coords[drive] = 0.0;
				}
				moveBuffer.feedRate = DefaultFeedrate * SecondsToMinutes;	// ask for a good feed rate, we may have paused during a slow move
				moveBuffer.moveType = 0;
				moveBuffer.endStopsToCheck = 0;
				moveBuffer.usePressureAdvance = false;
				moveBuffer.filePos = noFilePosition;
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
				segmentsLeft = 1;
			}
			break;

		case GCodeState::resuming3:
			if (LockMovementAndWaitForStandstill(gb))
			{
				for (size_t i = 0; i < NUM_FANS; ++i)
				{
					platform.SetFanValue(i, pausedFanSpeeds[i]);
				}
				for (size_t drive = numTotalAxes; drive < DRIVES; ++drive)
				{
					lastRawExtruderPosition[drive - numTotalAxes] = pauseRestorePoint.moveCoords[drive];	// reset the extruder position in case we are receiving absolute extruder moves
				}
				moveBuffer.virtualExtruderPosition = pauseRestorePoint.virtualExtruderPosition;
				fileGCode->MachineState().feedrate = pauseRestorePoint.feedRate;
				Tool * const ct = reprap.GetCurrentTool();
				if (ct != nullptr)
				{
					ct->virtualExtruderPosition = pauseRestorePoint.virtualExtruderPosition;
				}
				isPaused = false;
				reply.copy("Printing resumed");
				gb.SetState(GCodeState::normal);
			}
			break;

		case GCodeState::flashing1:
#ifdef DUET_NG
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

		case GCodeState::stopping:		// MO after executing stop.g if present
		case GCodeState::sleeping:		// M1 after executing sleep.g if present
			// Deselect the active tool and turn off all heaters, unless parameter Hn was used with n > 0
			if (!gb.Seen('H') || gb.GetIValue() <= 0)
			{
				Tool* tool = reprap.GetCurrentTool();
				if (tool != nullptr)
				{
					reprap.StandbyTool(tool->Number());
				}
				reprap.GetHeat().SwitchOffAll();
			}

			// chrishamm 2014-18-10: Although RRP says M0 is supposed to turn off all drives and heaters,
			// I think M1 is sufficient for this purpose. Leave M0 for a normal reset.
			if (gb.GetState() == GCodeState::sleeping)
			{
				DisableDrives();
			}
			else
			{
				platform.SetDriversIdle();
			}
			gb.SetState(GCodeState::normal);
			break;

		// States used for grid probing
		case GCodeState::gridProbing1:	// ready to move to next grid probe point
			{
				// Move to the current probe point
				Move& move = reprap.GetMove();
				const GridDefinition& grid = move.AccessBedProbeGrid().GetGrid();
				const float x = grid.GetXCoordinate(gridXindex);
				const float y = grid.GetYCoordinate(gridYindex);
				if (grid.IsInRadius(x, y))
				{
					if (move.IsAccessibleProbePoint(x, y))
					{
						moveBuffer.moveType = 0;
						moveBuffer.endStopsToCheck = 0;
						moveBuffer.usePressureAdvance = false;
						moveBuffer.filePos = noFilePosition;
						moveBuffer.coords[X_AXIS] = x - platform.GetCurrentZProbeParameters().xOffset;
						moveBuffer.coords[Y_AXIS] = y - platform.GetCurrentZProbeParameters().yOffset;
						moveBuffer.coords[Z_AXIS] = platform.GetZProbeStartingHeight();
						moveBuffer.feedRate = platform.GetZProbeTravelSpeed();
						moveBuffer.xAxes = DefaultXAxisMapping;
						moveBuffer.yAxes = DefaultYAxisMapping;
						segmentsLeft = 1;
						gb.AdvanceState();
					}
					else
					{
						platform.MessageF(GENERIC_MESSAGE, "Warning: skipping grid point (%.1f, %.1f) because Z probe cannot reach it\n", x, y);
						gb.SetState(GCodeState::gridProbing5);
					}
				}
				else
				{
					gb.SetState(GCodeState::gridProbing5);
				}
			}
			break;

		case GCodeState::gridProbing2:		// ready to probe the current grid probe point
			if (LockMovementAndWaitForStandstill(gb))
			{
				lastProbedTime = millis();
				gb.AdvanceState();
			}
			break;

		case GCodeState::gridProbing3:	// ready to probe the current grid probe point
			if (millis() - lastProbedTime >= (uint32_t)(reprap.GetPlatform().GetCurrentZProbeParameters().recoveryTime * SecondsToMillis))
			{
				// Probe the bed at the current XY coordinates
				// Check for probe already triggered at start
				if (platform.GetZProbeType() == 0)
				{
					// No Z probe, so do manual mesh levelling instead
					UnlockAll(gb);															// release the movement lock to allow manual Z moves
					gb.AdvanceState();														// resume at next state when user has finished adjusting the height
					doingManualBedProbe = true;												// suspend the Z movement limit
					DoManualProbe(gb);
				}
				else if (reprap.GetPlatform().GetZProbeResult() == EndStopHit::lowHit)
				{
					platform.Message(GENERIC_MESSAGE, "Error: Z probe already triggered before probing move started");
					gb.SetState(GCodeState::normal);
					if (platform.GetZProbeType() != 0 && !probeIsDeployed)
					{
						DoFileMacro(gb, RETRACTPROBE_G, false);
					}
					break;
				}
				else
				{
					zProbeTriggered = false;
					platform.SetProbing(true);
					moveBuffer.moveType = 0;
					moveBuffer.endStopsToCheck = ZProbeActive;
					moveBuffer.usePressureAdvance = false;
					moveBuffer.filePos = noFilePosition;
					moveBuffer.coords[Z_AXIS] = -platform.GetZProbeDiveHeight();
					moveBuffer.feedRate = platform.GetCurrentZProbeParameters().probeSpeed;
					moveBuffer.xAxes = DefaultXAxisMapping;
					moveBuffer.yAxes = DefaultYAxisMapping;
					segmentsLeft = 1;
					gb.AdvanceState();
				}
			}
			break;

		case GCodeState::gridProbing4:	// ready to lift the probe after probing the current grid probe point
			if (LockMovementAndWaitForStandstill(gb))
			{
				doingManualBedProbe = false;
				float heightError;
				if (platform.GetZProbeType() == 0)
				{
					// No Z probe, so we are doing manual mesh levelling. Take the current Z height as the height error.
					heightError = moveBuffer.coords[Z_AXIS];
				}
				else
				{
					platform.SetProbing(false);
					if (!zProbeTriggered)
					{
						platform.Message(GENERIC_MESSAGE, "Error: Z probe was not triggered during probing move");
						gb.SetState(GCodeState::normal);
						if (platform.GetZProbeType() != 0 && !probeIsDeployed)
						{
							DoFileMacro(gb, RETRACTPROBE_G, false);
						}
						break;
					}

					heightError = moveBuffer.coords[Z_AXIS] - platform.ZProbeStopHeight();
				}
				reprap.GetMove().AccessBedProbeGrid().SetGridHeight(gridXindex, gridYindex, heightError);

				// Move back up to the dive height
				moveBuffer.moveType = 0;
				moveBuffer.endStopsToCheck = 0;
				moveBuffer.usePressureAdvance = false;
				moveBuffer.filePos = noFilePosition;
				moveBuffer.coords[Z_AXIS] = platform.GetZProbeStartingHeight();
				moveBuffer.feedRate = platform.GetZProbeTravelSpeed();
				moveBuffer.xAxes = DefaultXAxisMapping;
				moveBuffer.yAxes = DefaultYAxisMapping;
				segmentsLeft = 1;
				gb.SetState(GCodeState::gridProbing5);
			}
			break;

		case GCodeState::gridProbing5:	// ready to compute the next probe point
			if (LockMovementAndWaitForStandstill(gb))
			{
				const GridDefinition& grid = reprap.GetMove().AccessBedProbeGrid().GetGrid();
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
					if (gridXindex + 1 == grid.NumXpoints())
					{
						++gridYindex;
					}
					else
					{
						++gridXindex;
					}
				}
				if (gridYindex == grid.NumYpoints())
				{
					// Done all the points
					gb.AdvanceState();
					if (platform.GetZProbeType() != 0 && !probeIsDeployed)
					{
						DoFileMacro(gb, RETRACTPROBE_G, false);
					}
				}
				else
				{
					gb.SetState(GCodeState::gridProbing1);
				}
			}
			break;

		case GCodeState::gridprobing6:
			// Finished probing the grid, and retracted the probe if necessary
			{
				float mean, deviation;
				const uint32_t numPointsProbed = reprap.GetMove().AccessBedProbeGrid().GetStatistics(mean, deviation);
				if (numPointsProbed >= 4)
				{
					reply.printf("%u points probed, mean error %.3f, deviation %.3f\n", numPointsProbed, mean, deviation);
					error = SaveHeightMap(gb, reply);
					reprap.GetMove().AccessBedProbeGrid().ExtrapolateMissing();
					reprap.GetMove().UseMesh(true);
				}
				else
				{
					reply.copy("Too few points probed");
					error = true;
				}
			}
			gb.SetState(GCodeState::normal);
			break;

		// States used for G30 probing
		case GCodeState::probingAtPoint0:
			// Initial state when executing G30 with a P parameter. Start by moving to the dive height at the current position.
			moveBuffer.moveType = 0;
			moveBuffer.endStopsToCheck = 0;
			moveBuffer.usePressureAdvance = false;
			moveBuffer.filePos = noFilePosition;
			moveBuffer.coords[Z_AXIS] = platform.GetZProbeStartingHeight();
			moveBuffer.feedRate = platform.GetZProbeTravelSpeed();
			moveBuffer.xAxes = DefaultXAxisMapping;
			moveBuffer.yAxes = DefaultYAxisMapping;
			segmentsLeft = 1;
			gb.AdvanceState();
			break;

		case GCodeState::probingAtPoint1:
			// The move to raise/lower the head to the correct dive height has been commanded.
			if (LockMovementAndWaitForStandstill(gb))
			{
				// Head is at the dive height but needs to be moved to the correct XY position.
				// The XY coordinates have already been stored.
				moveBuffer.moveType = 0;
				moveBuffer.endStopsToCheck = 0;
				moveBuffer.usePressureAdvance = false;
				moveBuffer.filePos = noFilePosition;
				(void)reprap.GetMove().GetProbeCoordinates(g30ProbePointIndex, moveBuffer.coords[X_AXIS], moveBuffer.coords[Y_AXIS], true);
				moveBuffer.coords[Z_AXIS] = platform.GetZProbeStartingHeight();
				moveBuffer.feedRate = platform.GetZProbeTravelSpeed();
				moveBuffer.xAxes = DefaultXAxisMapping;
				moveBuffer.yAxes = DefaultYAxisMapping;
				segmentsLeft = 1;

				gb.AdvanceState();
			}
			break;

		case GCodeState::probingAtPoint2:
			// Executing G30 with a P parameter. The move to put the head at the specified XY coordinates has been commanded.
			// OR initial state when executing G30 with no P parameter
			if (LockMovementAndWaitForStandstill(gb))
			{
				// Head has finished moving to the correct XY position
				lastProbedTime = millis();			// start the probe recovery timer
				gb.AdvanceState();
			}
			break;

		case GCodeState::probingAtPoint3:
			// Executing G30 with a P parameter. The move to put the head at the specified XY coordinates has been completed and the recovery timer started.
			// OR executing G30 without a P parameter, and the recovery timer has been started.
			if (millis() - lastProbedTime >= (uint32_t)(platform.GetCurrentZProbeParameters().recoveryTime * SecondsToMillis))
			{
				// The probe recovery time has elapsed, so we can start the probing  move
				if (platform.GetZProbeType() == 0)
				{
					// No Z probe, so we are doing manual 'probing'
					UnlockAll(gb);															// release the movement lock to allow manual Z moves
					gb.AdvanceState();														// resume at the next state when the user has finished
					doingManualBedProbe = true;												// suspend the Z movement limit
					DoManualProbe(gb);
				}
				else if (reprap.GetPlatform().GetZProbeResult() == EndStopHit::lowHit)		// check for probe already triggered at start
				{
					// Z probe is already triggered at the start of the move, so abandon the probe and record an error
					platform.Message(GENERIC_MESSAGE, "Error: Z probe already triggered at start of probing move\n");
					if (g30ProbePointIndex >= 0)
					{
						reprap.GetMove().SetZBedProbePoint(g30ProbePointIndex, platform.GetZProbeDiveHeight(), true, true);
					}
					if (platform.GetZProbeType() != 0 && !probeIsDeployed)
					{
						DoFileMacro(gb, RETRACTPROBE_G, false);
					}
					gb.SetState(GCodeState::normal);										// no point in doing anything else
				}
				else
				{
					zProbeTriggered = false;
					platform.SetProbing(true);
					moveBuffer.moveType = 0;
					moveBuffer.endStopsToCheck = ZProbeActive;
					moveBuffer.usePressureAdvance = false;
					moveBuffer.filePos = noFilePosition;
					moveBuffer.coords[Z_AXIS] = (GetAxisIsHomed(Z_AXIS))
												? -platform.GetZProbeDiveHeight()			// Z axis has been homed, so no point in going very far
												: -1.1 * platform.AxisTotalLength(Z_AXIS);	// Z axis not homed yet, so treat this as a homing move
					moveBuffer.feedRate = platform.GetCurrentZProbeParameters().probeSpeed;
					moveBuffer.xAxes = DefaultXAxisMapping;
					moveBuffer.yAxes = DefaultYAxisMapping;
					segmentsLeft = 1;
					gb.AdvanceState();
				}
			}
			break;

		case GCodeState::probingAtPoint4:
			// Executing G30. The probe wasn't triggered at the start of the move, and the probing move has been commanded.
			if (LockMovementAndWaitForStandstill(gb))
			{
				// Probing move has stopped
				doingManualBedProbe = false;
				bool probingError = false;
				float heightError;
				if (platform.GetZProbeType() == 0)
				{
					// No Z probe, so we are doing manual mesh levelling. Take the current Z height as the height error.
					heightError = moveBuffer.coords[Z_AXIS];
				}
				else
				{
					platform.SetProbing(false);
					if (!zProbeTriggered)
					{
						platform.Message(GENERIC_MESSAGE, "Error: Z probe was not triggered during probing move\n");
						heightError = 0.0;
						probingError = true;
					}
					else
					{
						// Successful probing
						float heightAdjust = 0.0;
						bool dummy;
						gb.TryGetFValue('H', heightAdjust, dummy);
						heightError = moveBuffer.coords[Z_AXIS] - (platform.ZProbeStopHeight() + heightAdjust);
					}
				}

				if (g30ProbePointIndex < 0)
				{
					// Simple G30 probing move
					gb.SetState(GCodeState::normal);								// usually nothing more to do except perhaps retract the probe
					if (!probingError)
					{
						if (g30SValue == -1)
						{
							// G30 S-1 command
							float m[DRIVES];
							reprap.GetMove().GetCurrentMachinePosition(m, false);	// get height without bed compensation
							zStoppedHeight = m[Z_AXIS];								// save for later
							gb.SetState(GCodeState::probingAtPoint7);				// special state for reporting the stopped height at the end
						}
						else
						{
							// Reset the Z axis origin according to the height error
							moveBuffer.coords[Z_AXIS] -= heightError;
							reprap.GetMove().SetNewPosition(moveBuffer.coords, false);
							ToolOffsetInverseTransform(moveBuffer.coords, currentUserPosition);
							SetAxisIsHomed(Z_AXIS);									//TODO this is only correct if the Z axis is Cartesian-like!
							lastProbedZ = 0.0;
						}
					}

					if (platform.GetZProbeType() != 0 && !probeIsDeployed)
					{
						DoFileMacro(gb, RETRACTPROBE_G, false);						// retract the probe before moving to the new state
					}
				}
				else
				{
					//TODO the following is only correct if the Z axis is Cartesian-like!
					// Probing with a probe point index number
					if (!GetAxisIsHomed(Z_AXIS))
					{
						// The Z axis has not yet been homed, so treat this probe as a homing move.
						moveBuffer.coords[Z_AXIS] -= heightError;			// reset the Z origin
						reprap.GetMove().SetNewPosition(moveBuffer.coords, false);
						ToolOffsetInverseTransform(moveBuffer.coords, currentUserPosition);
						SetAxisIsHomed(Z_AXIS);
						heightError = 0.0;
					}
					reprap.GetMove().SetZBedProbePoint(g30ProbePointIndex, heightError, true, probingError);

					// Move back up to the dive height before we change anything, in particular before we adjust leadscrews
					moveBuffer.moveType = 0;
					moveBuffer.endStopsToCheck = 0;
					moveBuffer.usePressureAdvance = false;
					moveBuffer.filePos = noFilePosition;
					moveBuffer.coords[Z_AXIS] = platform.GetZProbeStartingHeight();
					moveBuffer.feedRate = platform.GetZProbeTravelSpeed();
					moveBuffer.xAxes = DefaultXAxisMapping;
					moveBuffer.yAxes = DefaultYAxisMapping;
					segmentsLeft = 1;
					gb.AdvanceState();
				}
			}
			break;

		case GCodeState::probingAtPoint5:
			// Here when we were probing at a numbered point and we have moved the head back up to the dive height
			if (LockMovementAndWaitForStandstill(gb))
			{
				gb.AdvanceState();
				if (platform.GetZProbeType() != 0 && !probeIsDeployed)
				{
					DoFileMacro(gb, RETRACTPROBE_G, false);
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
					moveBuffer.coords[Z_AXIS] += lastProbedZ;
					reprap.GetMove().SetNewPosition(moveBuffer.coords, false);
					ToolOffsetInverseTransform(moveBuffer.coords, currentUserPosition);
					lastProbedZ = 0.0;
				}
				else if (g30SValue > -2)
				{
					reprap.GetMove().FinishedBedProbing(g30SValue, reply);
				}
			}

			gb.SetState(GCodeState::normal);
			break;

		case GCodeState::probingAtPoint7:
			// Here when we have finished executing G30 S-1 including retracting the probe if necessary
			reply.printf("Stopped at height %.3f mm", zStoppedHeight);
			gb.SetState(GCodeState::normal);
			break;

		// Firmware retraction/un-retraction states
		case GCodeState::doingFirmwareRetraction:
			// We just did the retraction part of a firmware retraction, now we need to do the Z hop
			if (segmentsLeft == 0)
			{
				const AxesBitmap xAxes = reprap.GetCurrentXAxes();
				const AxesBitmap yAxes = reprap.GetCurrentYAxes();
				reprap.GetMove().GetCurrentUserPosition(moveBuffer.coords, 0, xAxes, yAxes);
				for (size_t i = numTotalAxes; i < DRIVES; ++i)
				{
					moveBuffer.coords[i] = 0.0;
				}
				moveBuffer.feedRate = platform.MaxFeedrate(Z_AXIS);
				moveBuffer.coords[Z_AXIS] += retractHop;
				currentZHop = retractHop;
				moveBuffer.moveType = 0;
				moveBuffer.isFirmwareRetraction = true;
				moveBuffer.usePressureAdvance = false;
				moveBuffer.filePos = (&gb == fileGCode) ? gb.GetFilePosition(fileInput->BytesCached()) : noFilePosition;
				moveBuffer.canPauseAfter = false;			// don't pause after a retraction because that could cause too much retraction
				moveBuffer.xAxes = xAxes;
				moveBuffer.yAxes = yAxes;
				segmentsLeft = 1;
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
					for (size_t i = numTotalAxes; i < DRIVES; ++i)
					{
						moveBuffer.coords[i] = 0.0;
					}
					for (size_t i = 0; i < tool->DriveCount(); ++i)
					{
						moveBuffer.coords[numTotalAxes + tool->Drive(i)] = retractLength + retractExtra;
					}
					moveBuffer.feedRate = unRetractSpeed;
					moveBuffer.moveType = 0;
					moveBuffer.isFirmwareRetraction = true;
					moveBuffer.usePressureAdvance = false;
					moveBuffer.filePos = (&gb == fileGCode) ? gb.MachineState().fileState.GetPosition() - fileInput->BytesCached() : noFilePosition;
					moveBuffer.canPauseAfter = true;
					moveBuffer.xAxes = xAxes;
					moveBuffer.yAxes = yAxes;
					segmentsLeft = 1;
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
					platform.MessageF(GENERIC_MESSAGE, "Filament %s loaded", filamentToLoad);
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
					platform.MessageF(GENERIC_MESSAGE, "Filament %s unloaded", reprap.GetCurrentTool()->GetFilament()->GetName());
				}
				reprap.GetCurrentTool()->GetFilament()->Unload();
			}
			gb.SetState(GCodeState::normal);
			break;

		default:				// should not happen
			platform.Message(GENERIC_MESSAGE, "Error: undefined GCodeState\n");
			gb.SetState(GCodeState::normal);
			break;
		}

		if (gb.GetState() == GCodeState::normal)
		{
			// We completed a command, so unlock resources and tell the host about it
			gb.timerRunning = false;
			UnlockAll(gb);
			HandleReply(gb, error, reply.Pointer());
		}
	}

	// Move on to the next gcode source ready for next time
	++nextGcodeSource;
	if (nextGcodeSource == ARRAY_SIZE(gcodeSources))
	{
		nextGcodeSource = 0;
	}

	// Check if we need to display a warning
	const uint32_t now = millis();
	if (now - lastWarningMillis >= MinimumWarningInterval)
	{
		if (displayNoToolWarning)
		{
			platform.Message(GENERIC_MESSAGE, "Attempting to extrude with no tool selected.\n");
			displayNoToolWarning = false;
			lastWarningMillis = now;
		}
		if (displayDeltaNotHomedWarning)
		{
			platform.Message(GENERIC_MESSAGE, "Attempt to move the head of a Delta or SCARA printer before homing the towers\n");
			displayDeltaNotHomedWarning = false;
			lastWarningMillis = now;
		}
	}
	platform.ClassReport(longWait);
}

// Start a new gcode, or continue to execute one that has already been started:
void GCodes::StartNextGCode(GCodeBuffer& gb, StringRef& reply)
{
	if (IsPaused() && &gb == fileGCode)
	{
		// We are paused, so don't process any more gcodes from the file being printed.
		// There is a potential issue here if fileGCode holds any locks, so unlock everything.
		UnlockAll(gb);
	}
	else if (gb.IsReady() || gb.IsExecuting())
	{
		gb.SetFinished(ActOnCode(gb, reply));
	}
	else if (gb.MachineState().fileState.IsLive())
	{
		DoFilePrint(gb, reply);
	}
	else if (&gb == queuedGCode)
	{
		// Code queue
		codeQueue->FillBuffer(queuedGCode);
	}
	else if (&gb == httpGCode)
	{
		// Webserver
		httpInput->FillBuffer(httpGCode);
	}
	else if (&gb == telnetGCode)
	{
		// Telnet
		telnetInput->FillBuffer(telnetGCode);
	}
	else if (   &gb == serialGCode
#if SUPPORT_SCANNER
			 && !reprap.GetScanner().IsRegistered()
#endif
			)
	{
		// USB interface. This line may be shared with a 3D scanner
		serialInput->FillBuffer(serialGCode);
	}
	else if (&gb == auxGCode)
	{
		// Aux serial port (typically PanelDue)
		if (auxInput->FillBuffer(auxGCode))
		{
			// by default we assume no PanelDue is attached
			platform.SetAuxDetected();
		}
	}
}

void GCodes::DoFilePrint(GCodeBuffer& gb, StringRef& reply)
{
	FileData& fd = gb.MachineState().fileState;

	// Do we have more data to process?
	if (fileInput->ReadFromFile(fd))
	{
		// Yes - fill up the GCodeBuffer and run the next code
		if (fileInput->FillBuffer(&gb))
		{
			gb.SetFinished(ActOnCode(gb, reply));
		}
	}
	else
	{
		// We have reached the end of the file. Check for the last line of gcode not ending in newline.
		if (!gb.StartingNewCode())				// if there is something in the buffer
		{
			if (gb.Put('\n')) 					// in case there wasn't a newline ending the file
			{
				gb.SetFinished(ActOnCode(gb, reply));
				return;
			}
		}

		gb.Init();								// mark buffer as empty

		if (gb.MachineState().previous == nullptr)
		{
			// Finished printing SD card file
			// Don't close the file until all moves have been completed, in case the print gets paused.
			// Also, this keeps the state as 'Printing' until the print really has finished.
			if (LockMovementAndWaitForStandstill(gb))
			{
				fileInput->Reset();
				fd.Close();
				UnlockAll(gb);
				reprap.GetPrintMonitor().StoppedPrint();
				pausePending = false;
#ifdef DUET_NG
				platform.GetMassStorage()->Delete(platform.GetSysDir(), RESUME_AFTER_POWER_FAIL_G, true);
#endif
				if (platform.Emulating() == marlin)
				{
					// Pronterface expects a "Done printing" message
					HandleReply(gb, false, "Done printing file");
				}
			}
		}
		else
		{
			// Finished a macro or finished processing config.g
			fileInput->Reset();
			fd.Close();
			if (runningConfigFile)
			{
				CopyConfigFinalValues(gb);
				runningConfigFile = false;
			}
			Pop(gb);
			gb.Init();
			if (gb.GetState() == GCodeState::normal)
			{
				UnlockAll(gb);
				HandleReply(gb, false, "");
				if (pausePending && &gb == fileGCode && !gb.IsDoingFileMacro())
				{
					const char* const m226Command = "M226\n";
					gb.Put(m226Command, strlen(m226Command));
					pausePending = false;
				}
			}
		}
	}
}

// Check for and execute triggers
void GCodes::CheckTriggers()
{
	// Check for endstop state changes that activate new triggers
	const TriggerInputsBitmap oldEndstopStates = lastEndstopStates;
	lastEndstopStates = platform.GetAllEndstopStates();
	const TriggerInputsBitmap risen = lastEndstopStates & ~oldEndstopStates,
					  	  	  fallen = ~lastEndstopStates & oldEndstopStates;
	unsigned int lowestTriggerPending = MaxTriggers;
	for (unsigned int triggerNumber = 0; triggerNumber < MaxTriggers; ++triggerNumber)
	{
		const Trigger& ct = triggers[triggerNumber];
		if (   ((ct.rising & risen) != 0 || (ct.falling & fallen) != 0)
			&& (ct.condition == 0 || (ct.condition == 1 && reprap.GetPrintMonitor().IsPrinting()))
		   )
		{
			SetBit(triggersPending, triggerNumber);
		}
		if (triggerNumber < lowestTriggerPending && IsBitSet(triggersPending, triggerNumber))
		{
			lowestTriggerPending = triggerNumber;
		}
	}

	// If any triggers are pending, activate the one with the lowest number
	if (lowestTriggerPending == 0)
	{
		ClearBit(triggersPending, lowestTriggerPending);			// clear the trigger
		DoEmergencyStop();
	}
	else if (lowestTriggerPending < MaxTriggers						// if a trigger is pending
			 && !IsDaemonBusy()
			 && daemonGCode->GetState() == GCodeState::normal		// and we are not already executing a trigger or config.g
			)
	{
		if (lowestTriggerPending == 1)
		{
			if (isPaused || !reprap.GetPrintMonitor().IsPrinting())
			{
				ClearBit(triggersPending, lowestTriggerPending);	// ignore a pause trigger if we are already paused
			}
			else if (LockMovement(*daemonGCode))					// need to lock movement before executing the pause macro
			{
				ClearBit(triggersPending, lowestTriggerPending);	// clear the trigger
				DoPause(*daemonGCode, false);
			}
		}
		else
		{
			ClearBit(triggersPending, lowestTriggerPending);		// clear the trigger
			char buffer[25];
			StringRef filename(buffer, ARRAY_SIZE(buffer));
			filename.printf(SYS_DIR "trigger%u.g", lowestTriggerPending);
			DoFileMacro(*daemonGCode, filename.Pointer(), true);
		}
	}
}

// Execute an emergency stop
void GCodes::DoEmergencyStop()
{
	reprap.EmergencyStop();
	Reset();
	platform.Message(GENERIC_MESSAGE, "Emergency Stop! Reset the controller to continue.");
}

// Pause the print. Before calling this, check that we are doing a file print that isn't already paused and get the movement lock.
void GCodes::DoPause(GCodeBuffer& gb, bool isAuto)
{
	if (&gb == fileGCode)
	{
		// Pausing a file print because of a command in the file itself
		SavePosition(pauseRestorePoint, gb);
	}
	else
	{
		// Pausing a file print via another input source
		const bool movesSkipped = reprap.GetMove().PausePrint(pauseRestorePoint);		// tell Move we wish to pause the current print

		if (movesSkipped)
		{
			// The PausePrint call has filled in the restore point with machine coordinates
			ToolOffsetInverseTransform(pauseRestorePoint.moveCoords, currentUserPosition);	// transform the returned coordinates to user coordinates

			if (segmentsLeft != 0)
			{
				for (size_t drive = numTotalAxes; drive < DRIVES; ++drive)
				{
					pauseRestorePoint.moveCoords[drive] += moveBuffer.coords[drive];	// add on the extrusion in the move not yet taken
				}
				ClearMove();
			}
		}
		else if (segmentsLeft != 0 && moveBuffer.canPauseBefore)
		{
			// We were not able to skip any moves, however we can skip the move that is waiting
			ToolOffsetInverseTransform(moveBuffer.initialCoords, currentUserPosition);
			pauseRestorePoint.feedRate = moveBuffer.feedRate;
			pauseRestorePoint.virtualExtruderPosition = moveBuffer.virtualExtruderPosition;
			pauseRestorePoint.filePos = moveBuffer.filePos;
#if SUPPORT_IOBITS
			pauseRestorePoint.ioBits = moveBuffer.ioBits;
#endif
			for (size_t drive = numTotalAxes; drive < DRIVES; ++drive)
			{
				pauseRestorePoint.moveCoords[drive] = moveBuffer.coords[drive];			// store the extrusion in the move not yet taken
			}
			ClearMove();
		}
		else
		{
			// We were not able to skip any moves, and if there is a move waiting then we can't skip that one either
			pauseRestorePoint.feedRate = fileGCode->MachineState().feedrate;
			pauseRestorePoint.virtualExtruderPosition = GetVirtualExtruderPosition();

			// TODO: when we use RTOS there is a possible race condition in the following,
			// because we might try to pause when a waiting move has just been added but before the gcode buffer has been re-initialised ready for the next command
			pauseRestorePoint.filePos = fileGCode->GetFilePosition(fileInput->BytesCached());
#if SUPPORT_IOBITS
			pauseRestorePoint.ioBits = moveBuffer.ioBits;
#endif
			for (size_t drive = numTotalAxes; drive < DRIVES; ++drive)
			{
				pauseRestorePoint.moveCoords[drive] = 0.0;								// we haven't omitted any extrusion
			}
		}

		// Replace the pauses machine coordinates by user coordinates, which we updated earlier
		for (size_t axis = 0; axis < numVisibleAxes; ++axis)
		{
			pauseRestorePoint.moveCoords[axis] = currentUserPosition[axis];
		}

		// The extruder positions in the pause restore point currently give the amount of extrusion skipped. Replaced by the extruder position at the pause point.
		for (size_t drive = numTotalAxes; drive < DRIVES; ++drive)
		{
			pauseRestorePoint.moveCoords[drive] = lastRawExtruderPosition[drive - numTotalAxes] - pauseRestorePoint.moveCoords[drive];
		}

		// If we skipped any moves, reset the file pointer to the start of the first move we need to replay
		// The following could be delayed until we resume the print
		FileData& fdata = fileGCode->MachineState().fileState;
		if (fdata.IsLive() && pauseRestorePoint.filePos != noFilePosition)
		{
			fdata.Seek(pauseRestorePoint.filePos);										// replay the abandoned instructions when we resume
			fileInput->Reset();															// clear the buffered data
		}

		codeQueue->PurgeEntries();

		if (reprap.Debug(moduleGcodes))
		{
			platform.MessageF(GENERIC_MESSAGE, "Paused print, file offset=%u\n", pauseRestorePoint.filePos);
		}
	}

	SaveFanSpeeds();
	gb.SetState(GCodeState::pausing1);
	isPaused = true;

#ifdef DUET_NG
	isAutoPaused = isAuto;
	resumeInfoSaved = false;
#endif
}

bool GCodes::IsPaused() const
{
	return isPaused && !IsPausing() && !IsResuming();
}

bool GCodes::IsPausing() const
{
	const GCodeState topState = fileGCode->OriginalMachineState().state;
	return topState == GCodeState::pausing1 || topState == GCodeState::pausing2;
}

bool GCodes::IsResuming() const
{
	const GCodeState topState = fileGCode->OriginalMachineState().state;
	return topState == GCodeState::resuming1 || topState == GCodeState::resuming2 || topState == GCodeState::resuming3;
}

bool GCodes::IsRunning() const
{
	return !IsPaused() && !IsPausing() && !IsResuming();
}

#ifdef DUET_NG

// Try to pause the current SD card print, returning true if successful, false if needs to be called again
bool GCodes::AutoPause()
{
	if (IsPausing() || IsResuming())
	{
		return false;
	}

	if (isPaused)
	{
		if (!resumeInfoSaved)
		{
			SaveResumeInfo();
		}
	}
	else if (reprap.GetPrintMonitor().IsPrinting())
	{
		if (!LockMovement(*autoPauseGCode))
		{
			return false;
		}
		reprap.GetHeat().SuspendHeaters(true);			// turn heaters off to conserve power for the motors to execute the pause
		DoPause(*autoPauseGCode, true);
		SaveResumeInfo();
		platform.Message(GENERIC_MESSAGE, "Print auto-paused due to low voltage\n");
	}
	return true;
}

// Suspend the printer, only ever called after it has been auto paused
bool GCodes::AutoShutdown()
{
	if (isPaused && isAutoPaused)
	{
		if (!LockMovementAndWaitForStandstill(*autoPauseGCode))
		{
			return false;
		}
		CancelPrint();
		platform.Message(GENERIC_MESSAGE, "Print cancelled due to low voltage\n");
		return true;
	}
	return true;
}

// Resume printing, normally only ever called after it has been auto paused
bool GCodes::AutoResume()
{
	if (isPaused && isAutoPaused)
	{
		autoPauseGCode->SetState(GCodeState::resuming1);
		if (AllAxesAreHomed())
		{
			DoFileMacro(*autoPauseGCode, POWER_RESTORE_G, true);
		}
		platform.Message(GENERIC_MESSAGE, "Print auto-resumed\n");
	}
	return true;
}

// Permit printing to be resumed after it has been suspended
bool GCodes::AutoResumeAfterShutdown()
{
	// Currently we don't do anything here
	return true;
}

void GCodes::SaveResumeInfo()
{
	const char* const printingFilename = reprap.GetPrintMonitor().GetPrintingFilename();
	if (printingFilename != nullptr)
	{
		FileStore * const f = platform.GetFileStore(platform.GetSysDir(), RESUME_AFTER_POWER_FAIL_G, true);
		if (f == nullptr)
		{
			platform.MessageF(GENERIC_MESSAGE, "Error: failed to create file %s", RESUME_AFTER_POWER_FAIL_G);
		}
		else
		{
			char bufferSpace[200];
			StringRef buf(bufferSpace, ARRAY_SIZE(bufferSpace));

			// Write the header comment
			buf.printf("; File \"%s\" resume print after auto-pause", printingFilename);
			if (reprap.GetPlatform().IsDateTimeSet())
			{
				time_t timeNow = reprap.GetPlatform().GetDateTime();
				const struct tm * const timeInfo = gmtime(&timeNow);
				buf.catf(" at %04u-%02u-%02u %02u:%02u",
								timeInfo->tm_year + 1900, timeInfo->tm_mon, timeInfo->tm_mday, timeInfo->tm_hour, timeInfo->tm_min);
			}
			buf.cat('\n');
			bool ok = f->Write(buf.Pointer())
					&& reprap.GetHeat().WriteBedAndChamberTempSettings(f)	// turn on bed and chamber heaters
					&& reprap.WriteToolSettings(f)							// set tool temperatures, tool mix ratios etc.
					&& reprap.GetMove().WriteResumeSettings(f);				// load grid, if we are using one
			if (ok)
			{
				buf.printf("M98 P%s\n", RESUME_PROLOGUE_G);					// call the prologue - must contain at least M116
				ok = f->Write(buf.Pointer())
					&& platform.WriteFanSettings(f);						// set the speeds of non-thermostatic fans
			}
			if (ok)
			{
				buf.printf("M106 S%.2f\n", lastDefaultFanSpeed);
				ok = f->Write(buf.Pointer());
			}
			if (ok)
			{
				buf.printf("M116\nM290 S%.3f\nM23 %s\nM26 S%u\n", currentBabyStepZOffset, printingFilename, pauseRestorePoint.filePos);
				ok = f->Write(buf.Pointer());								// write baby stepping offset, filename and file position
			}
			if (ok)
			{
				buf.copy("G1 F6000");										// start building command to restore head position
				for (size_t axis = 0; axis < numVisibleAxes; ++axis)
				{
					buf.catf(" %c%.2f", axisLetters[axis], pauseRestorePoint.moveCoords[axis]);
				}
				buf.catf("\nG1 F%.1f P%u\nM24\n", pauseRestorePoint.feedRate * MinutesToSeconds, (unsigned int)pauseRestorePoint.ioBits);
				ok = f->Write(buf.Pointer());								// restore feed rate and output bits
			}
			if (!f->Close())
			{
				ok = false;
			}
			if (ok)
			{
				platform.Message(GENERIC_MESSAGE, "Resume-after-power-fail state saved\n");
			}
			else
			{
				platform.GetMassStorage()->Delete(platform.GetSysDir(), RESUME_AFTER_POWER_FAIL_G, true);
				platform.MessageF(GENERIC_MESSAGE, "Error: failed to write or close file %s\n", RESUME_AFTER_POWER_FAIL_G);
			}
		}

		resumeInfoSaved = true;			// say we saved it even if there was an error, to avoid constant retrying
	}
}

#endif

void GCodes::Diagnostics(MessageType mtype)
{
	platform.Message(mtype, "=== GCodes ===\n");
	platform.MessageF(mtype, "Segments left: %u\n", segmentsLeft);
	platform.MessageF(mtype, "Stack records: %u allocated, %u in use\n", GCodeMachineState::GetNumAllocated(), GCodeMachineState::GetNumInUse());
	const GCodeBuffer * const movementOwner = resourceOwners[MoveResource];
	platform.MessageF(mtype, "Movement lock held by %s\n", (movementOwner == nullptr) ? "null" : movementOwner->GetIdentity());

	for (size_t i = 0; i < ARRAY_SIZE(gcodeSources); ++i)
	{
		gcodeSources[i]->Diagnostics(mtype);
	}

	codeQueue->Diagnostics(mtype);
}

// Lock movement and wait for pending moves to finish.
// As a side-effect it loads moveBuffer with the last position and feedrate for you.
bool GCodes::LockMovementAndWaitForStandstill(const GCodeBuffer& gb)
{
	// Lock movement to stop another source adding moves to the queue
	if (!LockMovement(gb))
	{
		return false;
	}

	// Last one gone?
	if (segmentsLeft != 0)
	{
		return false;
	}

	// Wait for all the queued moves to stop so we get the actual last position
	if (!reprap.GetMove().AllMovesAreFinished())
	{
		return false;
	}

	// Get the current positions. These may not be the same as the ones we remembered from last time if we just did a special move.
	reprap.GetMove().GetCurrentUserPosition(moveBuffer.coords, 0, reprap.GetCurrentXAxes(), reprap.GetCurrentYAxes());
	memcpy(moveBuffer.initialCoords, moveBuffer.coords, numVisibleAxes * sizeof(moveBuffer.initialCoords[0]));
	ToolOffsetInverseTransform(moveBuffer.coords, currentUserPosition);
	return true;
}

// Save (some of) the state of the machine for recovery in the future.
bool GCodes::Push(GCodeBuffer& gb)
{
	const bool ok = gb.PushState();
	if (!ok)
	{
		platform.Message(GENERIC_MESSAGE, "Push(): stack overflow!\n");
	}
	return ok;
}

// Recover a saved state
void GCodes::Pop(GCodeBuffer& gb)
{
	if (!gb.PopState())
	{
		platform.Message(GENERIC_MESSAGE, "Pop(): stack underflow!\n");
	}
}

// Set up the extrusion and feed rate of a move for the Move class
// 'moveType' is the S parameter in the G0 or G1 command, or zero for a G2 or G3 command
// Returns true if this gcode is valid so far, false if it should be discarded
bool GCodes::LoadExtrusionAndFeedrateFromGCode(GCodeBuffer& gb, int moveType)
{
	// Zero every extruder drive as some drives may not be moved
	for (size_t drive = numTotalAxes; drive < DRIVES; drive++)
	{
		moveBuffer.coords[drive] = 0.0;
	}
	moveBuffer.hasExtrusion = false;

	// Deal with feed rate
	if (moveType >= 0 && gb.Seen(feedrateLetter))
	{
		const float rate = gb.GetFValue() * distanceScale;
		gb.MachineState().feedrate = (moveType == 0)
										? rate * speedFactor
										: rate * SecondsToMinutes;		// don't apply the speed factor to homing and other special moves
	}
	moveBuffer.feedRate = gb.MachineState().feedrate;

	// If we are extruding, check that we have a tool to extrude with
	if (gb.Seen(extrudeLetter))
	{
		moveBuffer.hasExtrusion = true;

		Tool* const tool = reprap.GetCurrentTool();
		if (tool == nullptr)
		{
			displayNoToolWarning = true;
			return false;
		}

		const size_t eMoveCount = tool->DriveCount();
		if (eMoveCount != 0)
		{
			// Set the drive values for this tool
			float eMovement[MaxExtruders];
			size_t mc = eMoveCount;
			gb.GetFloatArray(eMovement, mc, false);

			if (mc == 1 && eMoveCount > 1)
			{
				// There are multiple extruders present but only one value has been specified, so use mixing
				const float moveArg = eMovement[0] * distanceScale;
				float requestedExtrusionAmount;
				if (gb.MachineState().drivesRelative)
				{
					requestedExtrusionAmount = moveArg;
					tool->virtualExtruderPosition += moveArg;
				}
				else
				{
					requestedExtrusionAmount = moveArg - tool->virtualExtruderPosition;
					tool->virtualExtruderPosition = moveArg;
				}

				for (size_t eDrive = 0; eDrive < eMoveCount; eDrive++)
				{
					const int drive = tool->Drive(eDrive);
					const float extrusionAmount = requestedExtrusionAmount * tool->GetMix()[eDrive];
					lastRawExtruderPosition[drive] += extrusionAmount;
					rawExtruderTotalByDrive[drive] += extrusionAmount;
					rawExtruderTotal += extrusionAmount;
					moveBuffer.coords[drive + numTotalAxes] = extrusionAmount * extrusionFactors[drive] * volumetricExtrusionFactors[drive];
				}

			}
			else
			{
				// Either there is only one extruder associated with this tool, or individual extrusion amounts have been provided
				for (size_t eDrive = 0; eDrive < eMoveCount; eDrive++)
				{
					const int drive = tool->Drive(eDrive);
					const float moveArg = eMovement[eDrive] * distanceScale;
					const float extrusionAmount = (gb.MachineState().drivesRelative)
												? moveArg
												: moveArg - lastRawExtruderPosition[drive];
					lastRawExtruderPosition[drive] += extrusionAmount;
					rawExtruderTotalByDrive[drive] += extrusionAmount;
					rawExtruderTotal += extrusionAmount;
					moveBuffer.coords[drive + numTotalAxes] = extrusionAmount * extrusionFactors[drive] * volumetricExtrusionFactors[drive];
				}
			}
		}
	}
	return true;
}

// Get the virtual extruder position of the current tool
float GCodes::GetVirtualExtruderPosition() const
{
	const Tool * const ct = reprap.GetCurrentTool();
	return (ct == nullptr) ? 0.0 : ct->virtualExtruderPosition;
}

// Execute a straight move returning true if an error was written to 'reply'
// We have already acquired the movement lock and waited for the previous move to be taken.
bool GCodes::DoStraightMove(GCodeBuffer& gb, StringRef& reply)
{
	// Set up default move parameters
	moveBuffer.endStopsToCheck = 0;
	moveBuffer.moveType = 0;
	doingArcMove = false;
	moveBuffer.xAxes = reprap.GetCurrentXAxes();
	moveBuffer.yAxes = reprap.GetCurrentYAxes();
	moveBuffer.usePressureAdvance = false;
	moveBuffer.filePos = (&gb == fileGCode) ? gb.GetFilePosition(fileInput->BytesCached()) : noFilePosition;
	moveBuffer.virtualExtruderPosition = GetVirtualExtruderPosition();
	axesToSenseLength = 0;

	// Check to see if the move is a 'homing' move that endstops are checked on.
	if (gb.Seen('S'))
	{
		int ival = gb.GetIValue();
		if (ival == 1 || ival == 2 || ival == 3)
		{
			moveBuffer.moveType = ival;
			moveBuffer.xAxes = DefaultXAxisMapping;
			moveBuffer.yAxes = DefaultYAxisMapping;
		}

		if (ival == 1 || ival == 3)
		{
			for (size_t i = 0; i < numTotalAxes; ++i)
			{
				if (gb.Seen(axisLetters[i]))
				{
					SetBit(moveBuffer.endStopsToCheck, i);
				}
			}

			if (ival == 1)
			{
				moveBuffer.endStopsToCheck |= HomeAxes;
			}
			else
			{
				axesToSenseLength = moveBuffer.endStopsToCheck;
			}
		}
		else if (ival == 99)		// temporary code to log Z probe change positions
		{
			moveBuffer.endStopsToCheck |= LogProbeChanges;
		}
	}

	// Check for damaging moves on a delta or SCARA printer
	const KinematicsType kinType = reprap.GetMove().GetKinematics().GetKinematicsType();
	if (moveBuffer.moveType == 0)
	{
		// Regular move. If it's a delta or SCARA printer, all axes must be homed first.
		if (!AllAxesAreHomed() && (kinType == KinematicsType::linearDelta || kinType == KinematicsType::scara))
		{
			// The user may be attempting to move a delta printer to an XYZ position before homing the axes
			// This may be damaging and is almost certainly a user mistake, so ignore the move. But allow extruder-only moves.
			if (gb.Seen(axisLetters[X_AXIS]) || gb.Seen(axisLetters[Y_AXIS]) || gb.Seen(axisLetters[Z_AXIS]))
			{
				displayDeltaNotHomedWarning = true;
				return false;
			}
		}
	}
	else
	{
		// Special move. If on a delta, movement must be relative.
		if (!gb.MachineState().axesRelative && kinType == KinematicsType::linearDelta)
		{
			reply.copy("Attempt to move the motors of a Delta printer to absolute positions");
			return true;
		}
	}

	moveBuffer.canPauseAfter = (moveBuffer.endStopsToCheck == 0);
	moveBuffer.canPauseBefore = true;

	// Check for 'R' parameter to move relative to a restore point
	int rParam = (moveBuffer.moveType == 0 && gb.Seen('R')) ? gb.GetIValue() : 0;
	const RestorePoint * const rp = (rParam == 1) ? &pauseRestorePoint : (rParam == 2) ? &toolChangeRestorePoint : nullptr;

#if SUPPORT_IOBITS
	// Update the iobits parameter
	if (rp != nullptr)
	{
		moveBuffer.ioBits = rp->ioBits;
	}
	else if (gb.Seen('P'))
	{
		moveBuffer.ioBits = (IoBits_t)gb.GetIValue();
	}
	else
	{
		// Leave moveBuffer.ioBits alone so that we keep the previous value
	}
#endif

	if (reprap.GetMove().IsRawMotorMove(moveBuffer.moveType))
	{
		// This is a raw motor move, so we need the current raw motor positions in moveBuffer.coords
		reprap.GetMove().GetCurrentUserPosition(moveBuffer.coords, moveBuffer.moveType, reprap.GetCurrentXAxes(), reprap.GetCurrentYAxes());
	}

	// Set up the initial coordinates
	memcpy(moveBuffer.initialCoords, moveBuffer.coords, numVisibleAxes * sizeof(moveBuffer.initialCoords[0]));

	// Deal with XYZ movement
	const float initialX = currentUserPosition[X_AXIS];
	const float initialY = currentUserPosition[Y_AXIS];
	bool includesAxisMovement = (rp != nullptr);
	for (size_t axis = 0; axis < numVisibleAxes; axis++)
	{
		if (gb.Seen(axisLetters[axis]))
		{
			includesAxisMovement = true;
			const float moveArg = gb.GetFValue() * distanceScale;
			if (moveBuffer.moveType != 0)
			{
				if (gb.MachineState().axesRelative)
				{
					moveBuffer.coords[axis] += moveArg;
				}
				else
				{
					moveBuffer.coords[axis] = moveArg;
				}
			}
			else if (rp != nullptr)
			{
				currentUserPosition[axis] = moveArg + rp->moveCoords[axis];
			}
			else if (gb.MachineState().axesRelative)
			{
				currentUserPosition[axis] += moveArg;
			}
			else
			{
				currentUserPosition[axis] = moveArg;
			}
		}
		else if (rp != nullptr)
		{
			currentUserPosition[axis] = rp->moveCoords[axis];
		}
	}

	// Deal with extrusion and feed rate
	LoadExtrusionAndFeedrateFromGCode(gb, moveBuffer.moveType);

	// Set up the move. We must assign segmentsLeft last, so that when we move to RTOS, the move won't be picked up by the Move process before it is complete.
	// Note that if this is an extruder-only move, we don't do axis movements to allow for tool offset changes, we defer those until an axis moves.
	if (moveBuffer.moveType != 0)
	{
		// It's a raw motor move, so do it in a single segment and wait for it to complete
		segmentsLeft = 1;
		gb.SetState(GCodeState::waitingForMoveToComplete);
	}
	else if (!includesAxisMovement)
	{
		segmentsLeft = 1;														// extruder-only movement
	}
	else
	{
		ToolOffsetTransform(currentUserPosition, moveBuffer.coords);			// apply tool offset, baby stepping, Z hop and axis scaling
		AxesBitmap effectiveAxesHomed = axesHomed;
		if (doingManualBedProbe)
		{
			ClearBit(effectiveAxesHomed, Z_AXIS);								// if doing a manual Z probe, don't limit the Z movement
		}
		if (limitAxes && reprap.GetMove().GetKinematics().LimitPosition(moveBuffer.coords, numVisibleAxes, effectiveAxesHomed))
		{
			ToolOffsetInverseTransform(moveBuffer.coords, currentUserPosition);	// make sure the limits are reflected in the user position
		}

		// Flag whether we should use pressure advance, if there is any extrusion in this move.
		// We assume it is a normal printing move needing pressure advance if there is forward extrusion and XYU.. movement.
		// The movement code will only apply pressure advance if there is forward extrusion, so we only need to check for XYU.. movement here.
		for (size_t axis = 0; axis < numVisibleAxes; ++axis)
		{
			if (axis != Z_AXIS && moveBuffer.coords[axis] != moveBuffer.initialCoords[axis])
			{
				moveBuffer.usePressureAdvance = true;
				break;
			}
		}

		// Apply segmentation if necessary
		// Note for when we use RTOS: as soon as we set segmentsLeft nonzero, the Move process will assume that the move is ready to take, so this must be the last thing we do.
		const Kinematics& kin = reprap.GetMove().GetKinematics();
		if (kin.UseSegmentation() && (moveBuffer.hasExtrusion || !kin.UseRawG0()))
		{
			// This kinematics approximates linear motion by means of segmentation
			const float xyLength = sqrtf(fsquare(currentUserPosition[X_AXIS] - initialX) + fsquare(currentUserPosition[Y_AXIS] - initialY));
			const float moveTime = xyLength/moveBuffer.feedRate;			// this is a best-case time, often the move will take longer
			segmentsLeft = max<unsigned int>(1, min<unsigned int>(xyLength/kin.GetMinSegmentLength(), (unsigned int)(moveTime * kin.GetSegmentsPerSecond())));
		}
		else if (reprap.GetMove().IsUsingMesh())
		{
			const HeightMap& heightMap = reprap.GetMove().AccessBedProbeGrid();
			segmentsLeft = max<unsigned int>(1, heightMap.GetMinimumSegments(currentUserPosition[X_AXIS] - initialX, currentUserPosition[Y_AXIS] - initialY));
		}
		else
		{
			segmentsLeft = 1;
		}
	}

	return false;
}

// Execute an arc move returning true if it was badly-formed
// We already have the movement lock and the last move has gone
// Currently, we do not process new babystepping when executing an arc move
bool GCodes::DoArcMove(GCodeBuffer& gb, bool clockwise)
{
	// Get the axis parameters. X Y I J are compulsory, Z is optional.
	if (!gb.Seen('X')) return true;
	const float xParam = gb.GetFValue() * distanceScale;
	if (!gb.Seen('Y')) return true;
	const float yParam = gb.GetFValue() * distanceScale;
	if (!gb.Seen('I')) return true;
	const float iParam = gb.GetFValue() * distanceScale;
	if (!gb.Seen('J')) return true;
	const float jParam = gb.GetFValue() * distanceScale;

	memcpy(moveBuffer.initialCoords, moveBuffer.coords, numVisibleAxes * sizeof(moveBuffer.initialCoords[0]));

	// Save the arc centre user coordinates for later
	const float userArcCentreX = currentUserPosition[X_AXIS] + iParam;
	const float userArcCentreY = currentUserPosition[Y_AXIS] + jParam;

	// Work out the new user position
	const bool axesRelative = gb.MachineState().axesRelative;
	if (axesRelative)
	{
		currentUserPosition[X_AXIS] += xParam;
		currentUserPosition[Y_AXIS] += yParam;
	}
	else
	{
		currentUserPosition[X_AXIS] = xParam;
		currentUserPosition[Y_AXIS] = yParam;
	}

	// Get the optional Z parameter
	if (gb.Seen('Z'))
	{
		const float zParam = gb.GetFValue() * distanceScale;
		if (axesRelative)
		{
			currentUserPosition[Z_AXIS] += zParam;
		}
		else
		{
			currentUserPosition[Z_AXIS] = zParam;
		}
	}

	ToolOffsetTransform(currentUserPosition, moveBuffer.coords);			// set the final position
	if (limitAxes && reprap.GetMove().GetKinematics().LimitPosition(moveBuffer.coords, numVisibleAxes, axesHomed))
	{
		ToolOffsetInverseTransform(moveBuffer.coords, currentUserPosition);	// make sure the limits are reflected in the user position
	}

	// Compute the angle at which we stop
	const float finalTheta = atan2(currentUserPosition[Y_AXIS] - userArcCentreY, currentUserPosition[X_AXIS] - userArcCentreX);

	// Set up default move parameters
	moveBuffer.endStopsToCheck = 0;
	moveBuffer.moveType = 0;
	moveBuffer.canPauseAfter = moveBuffer.canPauseBefore = true;
	moveBuffer.xAxes = reprap.GetCurrentXAxes();
	moveBuffer.yAxes = reprap.GetCurrentYAxes();
	moveBuffer.virtualExtruderPosition = GetVirtualExtruderPosition();
	moveBuffer.endStopsToCheck = 0;
	moveBuffer.usePressureAdvance = true;
	moveBuffer.filePos = (&gb == fileGCode) ? gb.GetFilePosition(fileInput->BytesCached()) : noFilePosition;

	// Set up the arc centre coordinates and record which axes behave like an X axis.
	// The I and J parameters are always relative to present position.
	// For X and Y we need to set up the arc centre for each axis that X or Y is mapped to.
	for (size_t axis = 0; axis < numVisibleAxes; ++axis)
	{
		if (IsBitSet(moveBuffer.xAxes, axis))
		{
			arcCentre[axis] = moveBuffer.initialCoords[axis] + iParam;
		}
		else if (IsBitSet(moveBuffer.yAxes, axis))
		{
			arcCentre[axis] = moveBuffer.initialCoords[axis] + jParam;
		}
	}

	LoadExtrusionAndFeedrateFromGCode(gb, moveBuffer.moveType);

	arcRadius = sqrtf(iParam * iParam + jParam * jParam);
	arcCurrentAngle = atan2(-jParam, -iParam);

	// Calculate the total angle moved, which depends on which way round we are going
	float totalArc = (clockwise) ? arcCurrentAngle - finalTheta : finalTheta - arcCurrentAngle;
	if (totalArc < 0)
	{
		totalArc += 2 * PI;
	}

	// Compute how many segments we need to move, but don't store it yet
	const unsigned int segsLeft = max<unsigned int>((unsigned int)((arcRadius * totalArc)/arcSegmentLength + 0.8), 1u);
	arcAngleIncrement = totalArc/segsLeft;
	if (clockwise)
	{
		arcAngleIncrement = -arcAngleIncrement;
	}
	doingArcMove = true;

	segmentsLeft = segsLeft;		// must do this last for RTOS
//	debugPrintf("Radius %.2f, initial angle %.1f, increment %.1f, segments %u\n",
//				arcRadius, arcCurrentAngle * RadiansToDegrees, arcAngleIncrement * RadiansToDegrees, segmentsLeft);
	return false;
}

// The Move class calls this function to find what to do next.
bool GCodes::ReadMove(RawMove& m)
{
	if (segmentsLeft == 0)
	{
		return false;
	}

	m = moveBuffer;

	if (segmentsLeft == 1)
	{
		// If there is just 1 segment left, it doesn't matter if it is an arc move or not, just move to the end position
		ClearMove();
	}
	else
	{
		// This move needs to be divided into 2 or more segments
		m.canPauseAfter = false;							// we can only pause after the final segment
		moveBuffer.canPauseBefore = false;					// we can't pause before any of the remaining segments

		// Do the axes
		if (doingArcMove)
		{
			arcCurrentAngle += arcAngleIncrement;
		}

		for (size_t drive = 0; drive < numVisibleAxes; ++drive)
		{
			if (doingArcMove && drive != Z_AXIS)
			{
				if (IsBitSet(moveBuffer.yAxes, drive))
				{
					// Y axis or a substitute Y axis
					moveBuffer.initialCoords[drive] = arcCentre[drive] + arcRadius * sinf(arcCurrentAngle);
				}
				else if (IsBitSet(moveBuffer.xAxes, drive))
				{
					// X axis or a substitute X axis
					moveBuffer.initialCoords[drive] = arcCentre[drive] + arcRadius * cosf(arcCurrentAngle);
				}
			}
			else
			{
				const float movementToDo = (moveBuffer.coords[drive] - moveBuffer.initialCoords[drive])/segmentsLeft;
				moveBuffer.initialCoords[drive] += movementToDo;
			}
			m.coords[drive] = moveBuffer.initialCoords[drive];
		}

		// Do the extruders
		for (size_t drive = numTotalAxes; drive < DRIVES; ++drive)
		{
			const float extrusionToDo = moveBuffer.coords[drive]/segmentsLeft;
			m.coords[drive] = extrusionToDo;
			moveBuffer.coords[drive] -= extrusionToDo;
		}

		--segmentsLeft;
	}

	return true;
}

void GCodes::ClearMove()
{
	doingArcMove = false;
	moveBuffer.endStopsToCheck = 0;
	moveBuffer.moveType = 0;
	moveBuffer.isFirmwareRetraction = false;
	segmentsLeft = 0;				// do this last
}

// Run a file macro. Prior to calling this, 'state' must be set to the state we want to enter when the macro has been completed.
// Return true if the file was found or it wasn't and we were asked to report that fact.
bool GCodes::DoFileMacro(GCodeBuffer& gb, const char* fileName, bool reportMissing, bool runningM502)
{
	FileStore * const f = platform.GetFileStore(platform.GetSysDir(), fileName, false);
	if (f == nullptr)
	{
		if (reportMissing)
		{
			// Don't use snprintf into scratchString here, because fileName may be aliased to scratchString
			platform.MessageF(GENERIC_MESSAGE, "Macro file %s not found.\n", fileName);
			return true;
		}
		return false;
	}

	if (!Push(gb))
	{
		return true;
	}
	gb.MachineState().fileState.Set(f);
	gb.MachineState().doingFileMacro = true;
	gb.MachineState().runningM502 = runningM502;
	gb.SetState(GCodeState::normal);
	gb.Init();
	return true;
}

void GCodes::FileMacroCyclesReturn(GCodeBuffer& gb)
{
	if (gb.IsDoingFileMacro())
	{
		FileData &file = gb.MachineState().fileState;
		fileInput->Reset(file);
		file.Close();

		gb.PopState();
		gb.Init();
	}
}

// This handles G92. Return true if completed, false if it needs to be called again.
bool GCodes::SetPositions(GCodeBuffer& gb)
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
					return false;
				}
			}
			SetBit(axesIncluded, axis);
			currentUserPosition[axis] = axisValue;
		}
	}

	// Handle any E parameter in the G92 command. If we get an error, ignore it and do the axes anyway.
	if (gb.Seen(extrudeLetter))
	{
		Tool* const tool = reprap.GetCurrentTool();
		if (tool != nullptr)
		{
			const size_t eMoveCount = tool->DriveCount();
			if (eMoveCount != 0)
			{
				float eMovement[MaxExtruders];
				size_t mc = eMoveCount;
				gb.GetFloatArray(eMovement, mc, false);
				if (mc == 1 && eMoveCount > 1)
				{
					// The tool has multiple extruders, but only one position was given. Treat it as the mix position.
					tool->virtualExtruderPosition = gb.GetFValue() * distanceScale;
				}
				else
				{
					for (size_t eDrive = 0; eDrive < eMoveCount; eDrive++)
					{
						lastRawExtruderPosition[tool->Drive(eDrive)] = eMovement[eDrive] * distanceScale;
					}
				}
			}
		}
	}

	if (axesIncluded != 0)
	{
		ToolOffsetTransform(currentUserPosition, moveBuffer.coords);
		if (reprap.GetMove().GetKinematics().LimitPosition(moveBuffer.coords, numVisibleAxes, LowestNBits<AxesBitmap>(numVisibleAxes)))	// pretend that all axes are homed
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
					return false;
				}
			}
		}
#endif
	}

	return true;
}

// Offset the axes by the X, Y, and Z amounts in the M code in gb. The actual movement occurs on the next move command.
// It's not clear from the description in the reprap.org wiki whether offsets are cumulative or not. We assume they are.
bool GCodes::OffsetAxes(GCodeBuffer& gb)
{
	for (size_t drive = 0; drive < numVisibleAxes; drive++)
	{
		if (gb.Seen(axisLetters[drive]))
		{
			axisOffsets[drive] += gb.GetFValue() * distanceScale;
		}
	}

	return true;
}

// Home one or more of the axes.  Which ones are decided by the
// booleans homeX, homeY and homeZ.
// Returns true if completed, false if needs to be called again.
// 'reply' is only written if there is an error.
// 'error' is false on entry, gets changed to true if there is an error.
bool GCodes::DoHome(GCodeBuffer& gb, StringRef& reply, bool& error)
{
	if (!LockMovementAndWaitForStandstill(gb))
	{
		return false;
	}

#if SUPPORT_ROLAND
	// Deal with a Roland configuration
	if (reprap.GetRoland()->Active())
	{
		bool rolHome = reprap.GetRoland()->ProcessHome();
		if (rolHome)
		{
			for(size_t axis = 0; axis < AXES; axis++)
			{
				axisIsHomed[axis] = true;
			}
		}
		return rolHome;
	}
#endif

	// Find out which axes we have been asked to home
	toBeHomed = 0;
	for (size_t axis = 0; axis < numTotalAxes; ++axis)
	{
		if (gb.Seen(axisLetters[axis]))
		{
			SetBit(toBeHomed, axis);
			SetAxisNotHomed(axis);
		}
	}

	if (toBeHomed == 0)
	{
		SetAllAxesNotHomed();		// homing everything
		toBeHomed = LowestNBits<AxesBitmap>(numVisibleAxes);
	}

	gb.SetState(GCodeState::homing1);
	return true;
}

// This is called to execute a G30.
// It sets wherever we are as the probe point P (probePointIndex) then probes the bed, or gets all its parameters from the arguments.
// If X or Y are specified, use those; otherwise use the machine's coordinates.  If no Z is specified use the machine's coordinates.
// If it is specified and is greater than SILLY_Z_VALUE (i.e. greater than -9999.0) then that value is used.
// If it's less than SILLY_Z_VALUE the bed is probed and that value is used.
// Return true if an error occurs.
// We already own the movement lock before this is called.
bool GCodes::ExecuteG30(GCodeBuffer& gb, StringRef& reply)
{
	g30SValue = (gb.Seen('S')) ? gb.GetIValue() : -2;		// S-2 is equivalent to having no S parameter
	g30ProbePointIndex = -1;
	bool seenP = false;
	gb.TryGetIValue('P', g30ProbePointIndex, seenP);
	if (seenP)
	{
		if (g30ProbePointIndex < 0 || g30ProbePointIndex >= (int)MaxProbePoints)
		{
			reply.copy("Z probe point index out of range");
			return true;
		}
		else
		{
			// Set the specified probe point index to the specified coordinates
			const float x = (gb.Seen(axisLetters[X_AXIS])) ? gb.GetFValue() : currentUserPosition[X_AXIS];
			const float y = (gb.Seen(axisLetters[Y_AXIS])) ? gb.GetFValue() : currentUserPosition[Y_AXIS];
			const float z = (gb.Seen(axisLetters[Z_AXIS])) ? gb.GetFValue() : currentUserPosition[Z_AXIS];
			reprap.GetMove().SetXYBedProbePoint((size_t)g30ProbePointIndex, x, y);

			if (z > SILLY_Z_VALUE)
			{
				// Just set the height error to the specified Z coordinate
				reprap.GetMove().SetZBedProbePoint((size_t)g30ProbePointIndex, z, false, false);
				if (g30SValue > -2)
				{
					reprap.GetMove().FinishedBedProbing(g30SValue, reply);
				}
			}
			else
			{
				// Do a Z probe at the specified point.
				gb.SetState(GCodeState::probingAtPoint0);
				if (platform.GetZProbeType() != 0 && !probeIsDeployed)
				{
					DoFileMacro(gb, DEPLOYPROBE_G, false);
				}
			}
		}
	}
	else
	{
		// G30 without P parameter. This probes the current location starting from the current position.
		// If S=-1 it just reports the stopped height, else it resets the Z origin.
		gb.SetState(GCodeState::probingAtPoint2);
		if (platform.GetZProbeType() != 0 && !probeIsDeployed)
		{
			DoFileMacro(gb, DEPLOYPROBE_G, false);
		}
	}
	return false;
}

// Decide which device to display a message box on
MessageType GCodes::GetMessageBoxDevice(GCodeBuffer& gb) const
{
	MessageType mt = gb.GetResponseMessageType();
	if (mt == GENERIC_MESSAGE)
	{
		// Command source was the file being printed, or a trigger. Send the message to PanelDue if there is one, else to the web server.
		mt = (lastAuxStatusReportType >= 0) ? AUX_MESSAGE : HTTP_MESSAGE;
	}
	return mt;
}

// Do a manual bed probe. On entry the state variable is the state we want to return to when the user has finished adjusting the height.
void GCodes::DoManualProbe(GCodeBuffer& gb)
{
	if (Push(gb))										// stack the machine state including the file position
	{
		gb.MachineState().fileState.Close();							// stop reading from file
		gb.MachineState().waitingForAcknowledgement = true;				// flag that we are waiting for acknowledgement
		const MessageType mt = GetMessageBoxDevice(gb);
		platform.SendAlert(mt, "Adjust height until the nozzle just touches the bed, then press OK", "Manual bed probing", 2, 0.0, true);
	}
}

// Set or print the Z probe. Called by G31.
// Note that G31 P or G31 P0 prints the parameters of the currently-selected Z probe.
bool GCodes::SetPrintZProbe(GCodeBuffer& gb, StringRef& reply)
{
	int32_t zProbeType = 0;
	bool seenT = false;
	gb.TryGetIValue('T',zProbeType, seenT);
	if (zProbeType == 0)
	{
		zProbeType = platform.GetZProbeType();
	}
	ZProbeParameters params = platform.GetZProbeParameters(zProbeType);
	bool seen = false;
	gb.TryGetFValue(axisLetters[X_AXIS], params.xOffset, seen);
	gb.TryGetFValue(axisLetters[Y_AXIS], params.yOffset, seen);
	gb.TryGetFValue(axisLetters[Z_AXIS], params.height, seen);
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
			return false;
		}
		platform.SetZProbeParameters(zProbeType, params);
	}
	else if (seenT)
	{
		// Don't bother printing temperature coefficient and calibration temperature because we will probably remove them soon
		reply.printf("Threshold %d, trigger height %.2f, offsets X%.1f Y%.1f", params.adcValue, params.height, params.xOffset, params.yOffset);
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
	return true;
}

// Define the probing grid, returning true if error
// Called when we see an M557 command with no P parameter
bool GCodes::DefineGrid(GCodeBuffer& gb, StringRef &reply)
{
	bool seenX = false, seenY = false, seenR = false, seenS = false;
	float xValues[2];
	float yValues[2];
	float spacings[2] = { DefaultGridSpacing, DefaultGridSpacing };

	if (gb.TryGetFloatArray('X', 2, xValues, reply, seenX, false))
	{
		return true;
	}
	if (gb.TryGetFloatArray('Y', 2, yValues, reply, seenY, false))
	{
		return true;
	}
	if (gb.TryGetFloatArray('S', 2, spacings, reply, seenS, true))
	{
		return true;
	}

	float radius = -1.0;
	gb.TryGetFValue('R', radius, seenR);

	if (!seenX && !seenY && !seenR && !seenS)
	{
		// Just print the existing grid parameters
		const GridDefinition& grid = reprap.GetMove().AccessBedProbeGrid().GetGrid();
		if (grid.IsValid())
		{
			reply.copy("Grid: ");
			grid.PrintParameters(reply);
		}
		else
		{
			reply.copy("Grid is not defined");
		}
		return false;
	}

	if (seenX != seenY)
	{
		reply.copy("specify both or neither of X and Y in M577");
		return true;
	}

	if (!seenX && !seenR)
	{
		// Must have given just the S parameter
		reply.copy("specify at least radius or X,Y ranges in M577");
		return true;

	}

	if (!seenX)
	{
		if (radius > 0)
		{
			const float effectiveXRadius = floor((radius - 0.1)/spacings[0]) * spacings[0];
			xValues[0] = -effectiveXRadius;
			xValues[1] =  effectiveXRadius + 0.1;

			const float effectiveYRadius = floor((radius - 0.1)/spacings[1]) * spacings[1];
			yValues[0] = -effectiveYRadius;
			yValues[1] =  effectiveYRadius + 0.1;
		}
		else
		{
			reply.copy("M577 radius must be positive unless X and Y are specified");
			return true;
		}
	}
	GridDefinition newGrid(xValues, yValues, radius, spacings);		// create a new grid
	if (newGrid.IsValid())
	{
		reprap.GetMove().AccessBedProbeGrid().SetGrid(newGrid);
		return false;
	}
	else
	{
		const float xRange = (seenX) ? xValues[1] - xValues[0] : 2 * radius;
		const float yRange = (seenX) ? yValues[1] - yValues[0] : 2 * radius;
		reply.copy("bad grid definition: ");
		newGrid.PrintError(xRange, yRange, reply);
		return true;
	}
}

// Start probing the grid, returning true if we didn't because of an error.
// Prior to calling this the movement system must be locked.
bool GCodes::ProbeGrid(GCodeBuffer& gb, StringRef& reply)
{
	Move& move = reprap.GetMove();
	if (!move.AccessBedProbeGrid().GetGrid().IsValid())
	{
		reply.copy("No valid grid defined for G29 bed probing");
		return true;
	}

	if (!AllAxesAreHomed())
	{
		reply.copy("Must home printer before G29 bed probing");
		return true;
	}

	gridXindex = gridYindex = 0;

	move.AccessBedProbeGrid().ClearGridHeights();
	move.SetIdentityTransform();
	gb.SetState(GCodeState::gridProbing1);
	if (platform.GetZProbeType() != 0 && !probeIsDeployed)
	{
		DoFileMacro(gb, DEPLOYPROBE_G, false);
	}
	return false;
}

bool GCodes::LoadHeightMap(GCodeBuffer& gb, StringRef& reply) const
{
	reprap.GetMove().SetIdentityTransform();					// stop using old-style bed compensation and clear the height map

	char heightMapFileName[FILENAME_LENGTH];
	bool seen = false;
	gb.TryGetQuotedString('P', heightMapFileName, ARRAY_SIZE(heightMapFileName), seen);
	if (!seen)
	{
		strcpy(heightMapFileName, DefaultHeightMapFile);
	}

	FileStore * const f = platform.GetFileStore(platform.GetSysDir(), heightMapFileName, false);
	if (f == nullptr)
	{
		reply.printf("Height map file %s not found", heightMapFileName);
		return true;
	}

	reply.printf("Failed to load height map from file %s: ", heightMapFileName);	// set up error message to append to
	HeightMap& heightMap = reprap.GetMove().AccessBedProbeGrid();
	const bool err = heightMap.LoadFromFile(f, reply);
	f->Close();
	if (err)
	{
		heightMap.ClearGridHeights();			// make sure we don't end up with a partial height map
	}
	else
	{
		reply.Clear();											// wipe the error message
	}

	reprap.GetMove().UseMesh(!err);
	return err;
}

// Save the height map and append the success or error message to 'reply', returning true if an error occurred
// Called by G29 and M374. Both use the P parameter to provide the filename.
bool GCodes::SaveHeightMap(GCodeBuffer& gb, StringRef& reply) const
{
	char heightMapFileName[FILENAME_LENGTH];
	bool seen = false;
	gb.TryGetQuotedString('P', heightMapFileName, ARRAY_SIZE(heightMapFileName), seen);
	if (!seen)
	{
		strcpy(heightMapFileName, DefaultHeightMapFile);
	}

	FileStore * const f = platform.GetFileStore(platform.GetSysDir(), heightMapFileName, true);
	bool err;
	if (f == nullptr)
	{
		reply.catf("Failed to create height map file %s", heightMapFileName);
		err = true;
	}
	else
	{
		err = reprap.GetMove().AccessBedProbeGrid().SaveToFile(f);
		f->Close();
		if (err)
		{
			platform.GetMassStorage()->Delete(platform.GetSysDir(), heightMapFileName);
			reply.catf("Failed to save height map to file %s", heightMapFileName);
		}
		else
		{
			reply.catf("Height map saved to file %s", heightMapFileName);
		}
	}
	return err;
}

// Return the current coordinates as a printable string.
// Coordinates are updated at the end of each movement, so this won't tell you where you are mid-movement.
void GCodes::GetCurrentCoordinates(StringRef& s) const
{
	float liveCoordinates[DRIVES];
	reprap.GetMove().LiveCoordinates(liveCoordinates, reprap.GetCurrentXAxes(), reprap.GetCurrentYAxes());
	const Tool * const currentTool = reprap.GetCurrentTool();
	if (currentTool != nullptr)
	{
		const float * const offset = currentTool->GetOffset();
		for (size_t i = 0; i < numVisibleAxes; ++i)
		{
			liveCoordinates[i] += offset[i];
		}
	}

	s.Clear();
	for (size_t axis = 0; axis < numVisibleAxes; ++axis)
	{
		s.catf("%c: %.3f ", axisLetters[axis], liveCoordinates[axis]);
	}
	for (size_t i = numTotalAxes; i < DRIVES; i++)
	{
		s.catf("E%u: %.1f ", i - numTotalAxes, liveCoordinates[i]);
	}

	// Print the axis stepper motor positions as Marlin does, as an aid to debugging.
	// Don't bother with the extruder endpoints, they are zero after any non-extruding move.
	s.cat(" Count");
	for (size_t i = 0; i < numVisibleAxes; ++i)
	{
		s.catf(" %d", reprap.GetMove().GetEndPoint(i));
	}
}

bool GCodes::OpenFileToWrite(GCodeBuffer& gb, const char* directory, const char* fileName, const FilePosition size, const bool binaryWrite, const uint32_t fileCRC32)
{
	fileBeingWritten = platform.GetFileStore(directory, fileName, true);
	eofStringCounter = 0;
	fileSize = size;
	if (fileBeingWritten == nullptr)
	{
		platform.MessageF(GENERIC_MESSAGE, "Can't open GCode file \"%s\" for writing.\n", fileName);
		return false;
	}
	else
	{
		gb.SetCRC32(fileCRC32);
		gb.SetBinaryWriting(binaryWrite);
		gb.SetWritingFileDirectory(directory);
		return true;
	}
}

void GCodes::WriteHTMLToFile(GCodeBuffer& gb, char b)
{
	if (fileBeingWritten == nullptr)
	{
		platform.Message(GENERIC_MESSAGE, "Attempt to write to a null file.\n");
		return;
	}

	if ((b == eofString[eofStringCounter]) && (fileSize == 0))
	{
		eofStringCounter++;
		if (eofStringCounter >= eofStringLength)
		{
			FinishWrite(gb);
		}
	}
	else
	{
		if (eofStringCounter != 0)
		{
			for (uint8_t i = 0; i < eofStringCounter; i++)
			{
				fileBeingWritten->Write(eofString[i]);
			}
			eofStringCounter = 0;
		}
		fileBeingWritten->Write(b);		// writing one character at a time isn't very efficient, but uploading HTML files via USB is rarely done these days
		if (fileSize > 0 && fileBeingWritten->Length() >= fileSize)
		{
			FinishWrite(gb);
		}
	}
}

void GCodes::FinishWrite(GCodeBuffer& gb)
{
	const char* r;
	fileBeingWritten->Close();
	if ((gb.GetCRC32() != fileBeingWritten->GetCRC32()) && (gb.GetCRC32() != 0))
	{
		r = "Error: CRC32 checksum doesn't match";
	}
	else
	{
		r = (platform.Emulating() == marlin) ? "Done saving file." : "";
	}
	fileBeingWritten = nullptr;
	gb.SetBinaryWriting(false);
	gb.SetWritingFileDirectory(nullptr);

	HandleReply(gb, false, r);
}

void GCodes::WriteGCodeToFile(GCodeBuffer& gb)
{
	if (fileBeingWritten == nullptr)
	{
		platform.Message(GENERIC_MESSAGE, "Attempt to write to a null file.\n");
		return;
	}

	// End of file?
	if (gb.Seen('M'))
	{
		if (gb.GetIValue() == 29)
		{
			fileBeingWritten->Close();
			fileBeingWritten = nullptr;
			gb.SetWritingFileDirectory(nullptr);
			const char* r = (platform.Emulating() == marlin) ? "Done saving file." : "";
			HandleReply(gb, false, r);
			return;
		}
	}

	// Resend request?
	if (gb.Seen('G'))
	{
		if (gb.GetIValue() == 998)
		{
			if (gb.Seen('P'))
			{
				scratchString.printf("%d\n", gb.GetIValue());
				HandleReply(gb, false, scratchString.Pointer());
				return;
			}
		}
	}

	fileBeingWritten->Write(gb.Buffer());
	fileBeingWritten->Write('\n');
	HandleReply(gb, false, "");
}

// Set up a file to print, but don't print it yet.
void GCodes::QueueFileToPrint(const char* fileName)
{
	FileStore * const f = platform.GetFileStore(platform.GetGCodeDir(), fileName, false);
	if (f != nullptr)
	{
		// Cancel current print if there is any
		if (!reprap.GetPrintMonitor().IsPrinting())
		{
			CancelPrint();
		}

		fileGCode->SetToolNumberAdjust(0);	// clear tool number adjustment

		// Reset all extruder positions when starting a new print
		for (size_t extruder = 0; extruder < MaxExtruders; extruder++)
		{
			lastRawExtruderPosition[extruder] = 0.0;
			rawExtruderTotalByDrive[extruder] = 0.0;
		}
		rawExtruderTotal = 0.0;
		reprap.GetMove().ResetExtruderPositions();

		fileToPrint.Set(f);
		fileOffsetToPrint = 0;
	}
	else
	{
		platform.MessageF(GENERIC_MESSAGE, "GCode file \"%s\" not found\n", fileName);
	}
}

void GCodes::DeleteFile(const char* fileName)
{
	if (!platform.GetMassStorage()->Delete(platform.GetGCodeDir(), fileName))
	{
		platform.MessageF(GENERIC_MESSAGE, "Could not delete file \"%s\"\n", fileName);
	}
}

// Function to handle dwell delays. Returns true for dwell finished, false otherwise.
bool GCodes::DoDwell(GCodeBuffer& gb)
{
	int32_t dwell;
	if (gb.Seen('S'))
	{
		dwell = (int32_t)(gb.GetFValue() * 1000.0);		// S values are in seconds
	}
	else if (gb.Seen('P'))
	{
		dwell = gb.GetIValue();							// P value are in milliseconds
	}
	else
	{
		return true;  // No time given - throw it away
	}

	if (dwell <= 0)
	{
		return true;
	}

#if SUPPORT_ROLAND
	// Deal with a Roland configuration
	if (reprap.GetRoland()->Active())
	{
		return reprap.GetRoland()->ProcessDwell(dwell);
	}
#endif

	// Wait for all the queued moves to stop
	if (!LockMovementAndWaitForStandstill(gb))
	{
		return false;
	}

	if (simulationMode != 0)
	{
		simulationTime += (float)dwell * 0.001;
		return true;
	}
	else
	{
		return DoDwellTime(gb, (uint32_t)dwell);
	}
}

bool GCodes::DoDwellTime(GCodeBuffer& gb, uint32_t dwellMillis)
{
	const uint32_t now = millis();

	// Are we already in the dwell?
	if (gb.timerRunning)
	{
		if (now - gb.whenTimerStarted >= dwellMillis)
		{
			gb.timerRunning = false;
			return true;
		}
		return false;
	}

	// New dwell - set it up
	gb.whenTimerStarted = now;
	gb.timerRunning = true;
	return false;
}

// Set offset, working and standby temperatures for a tool. I.e. handle a G10.
bool GCodes::SetOrReportOffsets(GCodeBuffer &gb, StringRef& reply, bool& error)
{
	Tool *tool;
	if (gb.Seen('P'))
	{
		int toolNumber = gb.GetIValue();
		toolNumber += gb.GetToolNumberAdjust();
		tool = reprap.GetTool(toolNumber);

		if (tool == nullptr)
		{
			reply.printf("Attempt to set/report offsets and temperatures for non-existent tool: %d", toolNumber);
			error = true;
			return true;
		}
	}
	else
	{
		tool = reprap.GetCurrentTool();
		if (tool == nullptr)
		{
			reply.printf("Attempt to set/report offsets and temperatures for no selected tool");
			error = true;
			return true;
		}
	}

	// Deal with setting offsets
	float offset[MaxAxes];
	for (size_t i = 0; i < MaxAxes; ++i)
	{
		offset[i] = tool->GetOffset()[i];
	}

	bool settingOffset = false;
	for (size_t axis = 0; axis < numVisibleAxes; ++axis)
	{
		gb.TryGetFValue(axisLetters[axis], offset[axis], settingOffset);
	}
	if (settingOffset)
	{
		if (!LockMovement(gb))
		{
			return false;
		}
		tool->SetOffset(offset);
	}

	// Deal with setting temperatures
	bool settingTemps = false;
	size_t hCount = tool->HeaterCount();
	float standby[Heaters];
	float active[Heaters];
	if (hCount > 0)
	{
		tool->GetVariables(standby, active);
		if (gb.Seen('R'))
		{
			gb.GetFloatArray(standby, hCount, true);
			settingTemps = true;
		}
		if (gb.Seen('S'))
		{
			gb.GetFloatArray(active, hCount, true);
			settingTemps = true;
		}

		if (settingTemps && simulationMode == 0)
		{
			tool->SetVariables(standby, active);
		}
	}

	if (!settingOffset && !settingTemps)
	{
		// Print offsets and temperatures
		reply.printf("Tool %d offsets:", tool->Number());
		for (size_t axis = 0; axis < numVisibleAxes; ++axis)
		{
			reply.catf(" %c%.2f", axisLetters[axis], offset[axis]);
		}
		if (hCount != 0)
		{
			reply.cat(", active/standby temperature(s):");
			for (size_t heater = 0; heater < hCount; heater++)
			{
				reply.catf(" %.1f/%.1f", active[heater], standby[heater]);
			}
		}
	}
	return true;
}

// Create a new tool definition, returning true if an error was reported
bool GCodes::ManageTool(GCodeBuffer& gb, StringRef& reply)
{
	if (!gb.Seen('P'))
	{
		// DC temporary code to allow tool numbers to be adjusted so that we don't need to edit multi-media files generated by slic3r
		if (gb.Seen('S'))
		{
			int adjust = gb.GetIValue();
			gb.SetToolNumberAdjust(adjust);
		}
		return false;
	}

	// Check tool number
	bool seen = false;
	const int toolNumber = gb.GetIValue();
	if (toolNumber < 0)
	{
		reply.copy("Tool number must be positive");
		return true;
	}

	// Check tool name
	char name[ToolNameLength];
	if (gb.Seen('S'))
	{
		if (!gb.GetQuotedString(name, ARRAY_SIZE(name)))
		{
			reply.copy("Invalid tool name");
			return true;
		}
		seen = true;
	}
	else
	{
		strcpy(name, "");
	}

	// Check drives
	long drives[MaxExtruders];  		// There can never be more than we have...
	size_t dCount = numExtruders;	// Sets the limit and returns the count
	if (gb.Seen('D'))
	{
		gb.GetLongArray(drives, dCount);
		seen = true;
	}
	else
	{
		dCount = 0;
	}

	// Check heaters
	long heaters[Heaters];
	size_t hCount = Heaters;
	if (gb.Seen('H'))
	{
		gb.GetLongArray(heaters, hCount);
		seen = true;
	}
	else
	{
		hCount = 0;
	}

	// Check X axis mapping
	AxesBitmap xMap;
	if (gb.Seen('X'))
	{
		long xMapping[MaxAxes];
		size_t xCount = numVisibleAxes;
		gb.GetLongArray(xMapping, xCount);
		xMap = LongArrayToBitMap<AxesBitmap>(xMapping, xCount) & LowestNBits<AxesBitmap>(numVisibleAxes);
		seen = true;
	}
	else
	{
		xMap = DefaultXAxisMapping;					// by default map X axis straight through
	}

	// Check Y axis mapping
	AxesBitmap yMap;
	if (gb.Seen('Y'))
	{
		long yMapping[MaxAxes];
		size_t yCount = numVisibleAxes;
		gb.GetLongArray(yMapping, yCount);
		yMap = LongArrayToBitMap<AxesBitmap>(yMapping, yCount) & LowestNBits<AxesBitmap>(numVisibleAxes);
		seen = true;
	}
	else
	{
		yMap = DefaultYAxisMapping;					// by default map X axis straight through
	}

	if ((xMap & yMap) != 0)
	{
		reply.copy("Cannot map bith X and Y to the aame axis");
		return true;
	}

	// Check for fan mapping
	FansBitmap fanMap;
	if (gb.Seen('F'))
	{
		long fanMapping[NUM_FANS];
		size_t fanCount = NUM_FANS;
		gb.GetLongArray(fanMapping, fanCount);
		fanMap = LongArrayToBitMap<FansBitmap>(fanMapping, fanCount) & LowestNBits<FansBitmap>(NUM_FANS);
		seen = true;
	}
	else
	{
		fanMap = 1;					// by default map fan 0 to fan 0
	}

	if (seen)
	{
		// Add or delete tool, so start by deleting the old one with this number, if any
		reprap.DeleteTool(reprap.GetTool(toolNumber));

		// M563 P# D-1 H-1 removes an existing tool
		if (dCount == 1 && hCount == 1 && drives[0] == -1 && heaters[0] == -1)
		{
			// nothing more to do
		}
		else
		{
			Tool* const tool = Tool::Create(toolNumber, name, drives, dCount, heaters, hCount, xMap, yMap, fanMap, reply);
			if (tool == nullptr)
			{
				return true;
			}
			reprap.AddTool(tool);
		}
	}
	else
	{
		reprap.PrintTool(toolNumber, reply);
	}
	return false;
}

// Does what it says.
void GCodes::DisableDrives()
{
	for (size_t drive = 0; drive < DRIVES; drive++)
	{
		platform.DisableDrive(drive);
	}
	SetAllAxesNotHomed();
}

void GCodes::SetMACAddress(GCodeBuffer& gb)
{
	uint8_t mac[6];
	const char* ipString = gb.GetString();
	uint8_t sp = 0;
	uint8_t spp = 0;
	uint8_t ipp = 0;
	while (ipString[sp])
	{
		if (ipString[sp] == ':')
		{
			mac[ipp] = strtoul(&ipString[spp], nullptr, 16);
			ipp++;
			if (ipp > 5)
			{
				break;
			}
			sp++;
			spp = sp;
		}
		else
		{
			sp++;
		}
	}
	if (ipp == 5)
	{
		mac[ipp] = strtoul(&ipString[spp], nullptr, 16);
		platform.SetMACAddress(mac);
	}
	else
	{
		platform.MessageF(GENERIC_MESSAGE, "Dud MAC address: %s\n", gb.Buffer());
	}
}

bool GCodes::ChangeMicrostepping(size_t drive, int microsteps, int mode) const
{
	bool dummy;
	unsigned int oldSteps = platform.GetMicrostepping(drive, mode, dummy);
	bool success = platform.SetMicrostepping(drive, microsteps, mode);
	if (success && mode <= 1)							// modes higher than 1 are used for special functions
	{
		// We changed the microstepping, so adjust the steps/mm to compensate
		float stepsPerMm = platform.DriveStepsPerUnit(drive);
		if (stepsPerMm > 0)
		{
			platform.SetDriveStepsPerUnit(drive, stepsPerMm * (float)microsteps / (float)oldSteps);
		}
	}
	return success;
}

// Set the speeds of fans mapped for the current tool to lastDefaultFanSpeed
void GCodes::SetMappedFanSpeed()
{
	if (reprap.GetCurrentTool() == nullptr)
	{
		platform.SetFanValue(0, lastDefaultFanSpeed);
	}
	else
	{
		const uint32_t fanMap = reprap.GetCurrentTool()->GetFanMapping();
		for (size_t i = 0; i < NUM_FANS; ++i)
		{
			if (IsBitSet(fanMap, i))
			{
				platform.SetFanValue(i, lastDefaultFanSpeed);
			}
		}
	}
}

// Save the speeds of all fans
void GCodes::SaveFanSpeeds()
{
	for (size_t i = 0; i < NUM_FANS; ++i)
	{
		pausedFanSpeeds[i] = platform.GetFanValue(i);
	}
	pausedDefaultFanSpeed = lastDefaultFanSpeed;
}

// Handle sending a reply back to the appropriate interface(s).
// Note that 'reply' may be empty. If it isn't, then we need to append newline when sending it.
// Also, gb may be null if we were executing a trigger macro.
void GCodes::HandleReply(GCodeBuffer& gb, bool error, const char* reply)
{
	// Don't report "ok" responses if a (macro) file is being processed
	// Also check that this response was triggered by a gcode
	if ((gb.MachineState().doingFileMacro || &gb == fileGCode) && reply[0] == 0)
	{
		return;
	}

	// Second UART device, e.g. dc42's PanelDue. Do NOT use emulation for this one!
	if (&gb == auxGCode)
	{
		platform.AppendAuxReply(reply);
		return;
	}

	const Compatibility c = (&gb == serialGCode || &gb == telnetGCode) ? platform.Emulating() : me;
	const MessageType type = gb.GetResponseMessageType();
	const char* const response = (gb.Seen('M') && gb.GetIValue() == 998) ? "rs " : "ok";
	const char* emulationType = 0;

	switch (c)
	{
	case me:
	case reprapFirmware:
		if (error)
		{
			platform.Message(type, "Error: ");
		}
		platform.Message(type, reply);
		platform.Message(type, "\n");
		return;

	case marlin:
		// We don't need to handle M20 here because we always allocate an output buffer for that one
		if (gb.Seen('M') && gb.GetIValue() == 28)
		{
			platform.Message(type, response);
			platform.Message(type, "\n");
			platform.Message(type, reply);
			platform.Message(type, "\n");
			return;
		}

		if ((gb.Seen('M') && gb.GetIValue() == 105) || (gb.Seen('M') && gb.GetIValue() == 998))
		{
			platform.Message(type, response);
			platform.Message(type, " ");
			platform.Message(type, reply);
			platform.Message(type, "\n");
			return;
		}

		if (reply[0] != 0 && !gb.IsDoingFileMacro())
		{
			platform.Message(type, reply);
			platform.Message(type, "\n");
			platform.Message(type, response);
			platform.Message(type, "\n");
		}
		else if (reply[0] != 0)
		{
			platform.Message(type, reply);
			platform.Message(type, "\n");
		}
		else
		{
			platform.Message(type, response);
			platform.Message(type, "\n");
		}
		return;

	case teacup:
		emulationType = "teacup";
		break;
	case sprinter:
		emulationType = "sprinter";
		break;
	case repetier:
		emulationType = "repetier";
		break;
	default:
		emulationType = "unknown";
	}

	if (emulationType != 0)
	{
		platform.MessageF(type, "Emulation of %s is not yet supported.\n", emulationType);	// don't send this one to the web as well, it concerns only the USB interface
	}
}

void GCodes::HandleReply(GCodeBuffer& gb, bool error, OutputBuffer *reply)
{
	// Although unlikely, it's possible that we get a nullptr reply. Don't proceed if this is the case
	if (reply == nullptr)
	{
		return;
	}

	// Second UART device, e.g. dc42's PanelDue. Do NOT use emulation for this one!
	if (&gb == auxGCode)
	{
		platform.AppendAuxReply(reply);
		return;
	}

	const Compatibility c = (&gb == serialGCode || &gb == telnetGCode) ? platform.Emulating() : me;
	const MessageType type = gb.GetResponseMessageType();
	const char* const response = (gb.Seen('M') && gb.GetIValue() == 998) ? "rs " : "ok";
	const char* emulationType = nullptr;

	switch (c)
	{
	case me:
	case reprapFirmware:
		if (error)
		{
			platform.Message(type, "Error: ");
		}
		platform.Message(type, reply);
		return;

	case marlin:
		if (gb.Seen('M') && gb.GetIValue() == 20)
		{
			platform.Message(type, "Begin file list\n");
			platform.Message(type, reply);
			platform.Message(type, "End file list\n");
			platform.Message(type, response);
			platform.Message(type, "\n");
			return;
		}

		if (gb.Seen('M') && gb.GetIValue() == 28)
		{
			platform.Message(type, response);
			platform.Message(type, "\n");
			platform.Message(type, reply);
			return;
		}

		if ((gb.Seen('M') && gb.GetIValue() == 105) || (gb.Seen('M') && gb.GetIValue() == 998))
		{
			platform.Message(type, response);
			platform.Message(type, " ");
			platform.Message(type, reply);
			return;
		}

		if (reply->Length() != 0 && !gb.IsDoingFileMacro())
		{
			platform.Message(type, reply);
			platform.Message(type, "\n");
			platform.Message(type, response);
			platform.Message(type, "\n");
		}
		else if (reply->Length() != 0)
		{
			platform.Message(type, reply);
		}
		else
		{
			OutputBuffer::ReleaseAll(reply);
			platform.Message(type, response);
			platform.Message(type, "\n");
		}
		return;

	case teacup:
		emulationType = "teacup";
		break;
	case sprinter:
		emulationType = "sprinter";
		break;
	case repetier:
		emulationType = "repetier";
		break;
	default:
		emulationType = "unknown";
	}

	// If we get here then we didn't handle the message, so release the buffer(s)
	OutputBuffer::ReleaseAll(reply);
	if (emulationType != 0)
	{
		platform.MessageF(type, "Emulation of %s is not yet supported.\n", emulationType);	// don't send this one to the web as well, it concerns only the USB interface
	}
}

// Set PID parameters (M301 or M304 command). 'heater' is the default heater number to use.
void GCodes::SetPidParameters(GCodeBuffer& gb, int heater, StringRef& reply)
{
	if (gb.Seen('H'))
	{
		heater = gb.GetIValue();
	}

	if (heater >= 0 && heater < (int)Heaters)
	{
		const FopDt& model = reprap.GetHeat().GetHeaterModel(heater);
		M301PidParameters pp = model.GetM301PidParameters(false);
		bool seen = false;
		gb.TryGetFValue('P', pp.kP, seen);
		gb.TryGetFValue('I', pp.kI, seen);
		gb.TryGetFValue('D', pp.kD, seen);

		if (seen)
		{
			reprap.GetHeat().SetM301PidParameters(heater, pp);
		}
		else if (!model.UsePid())
		{
			reply.printf("Heater %d is in bang-bang mode", heater);
		}
		else if (model.ArePidParametersOverridden())
		{
			reply.printf("Heater %d P:%.1f I:%.3f D:%.1f", heater, pp.kP, pp.kI, pp.kD);
		}
		else
		{
			reply.printf("Heater %d uses model-derived PID parameters. Use M307 H%d to view them", heater, heater);
		}
	}
}

// Process M305, returning true if an error occurs
bool GCodes::SetHeaterParameters(GCodeBuffer& gb, StringRef& reply)
{
	if (gb.Seen('P'))
	{
		const int heater = gb.GetIValue();
		if ((heater >= 0 && heater < (int)Heaters) || (heater >= (int)FirstVirtualHeater && heater < (int)(FirstVirtualHeater + MaxVirtualHeaters)))
		{
			Heat& heat = reprap.GetHeat();
			const int oldChannel = heat.GetHeaterChannel(heater);
			bool seen = false;
			int32_t channel = oldChannel;
			gb.TryGetIValue('X', channel, seen);
			if (!seen && oldChannel < 0)
			{
				// For backwards compatibility, if no sensor has been configured on this channel then assume the thermistor normally associated with the heater
				if (heater < (int)Heaters)
				{
					channel = heater;
				}
				else
				{
					reply.printf("Virtual heater %d is not configured", heater);
					return false;
				}
			}

			if (channel != oldChannel)
			{
				if (heat.SetHeaterChannel(heater, channel))
				{
					reply.printf("unable to use sensor channel %d on heater %d", channel, heater);
					return true;
				}
			}

			bool hadError = false;
			heat.ConfigureHeaterSensor(305, (unsigned int)heater, gb, reply, hadError);
			return hadError;
		}
		else
		{
			reply.printf("heater number %d is out of range", heater);
			return true;
		}
	}
	return false;
}

void GCodes::SetToolHeaters(Tool *tool, float temperature)
{
	if (tool == nullptr)
	{
		platform.Message(GENERIC_MESSAGE, "Setting temperature: no tool selected.\n");
		return;
	}

	float standby[Heaters];
	float active[Heaters];
	tool->GetVariables(standby, active);
	for (size_t h = 0; h < tool->HeaterCount(); h++)
	{
		active[h] = temperature;
	}
	tool->SetVariables(standby, active);
}

// Retract or un-retract filament, returning true if movement has been queued, false if this needs to be called again
bool GCodes::RetractFilament(GCodeBuffer& gb, bool retract)
{
	if (retract != isRetracted && (retractLength != 0.0 || retractHop != 0.0 || (!retract && retractExtra != 0.0)))
	{
		if (!LockMovement(gb))
		{
			return false;
		}

		if (segmentsLeft != 0)
		{
			return false;
		}

		// New code does the retraction and the Z hop as separate moves
		// Get ready to generate a move
		const uint32_t xAxes = reprap.GetCurrentXAxes();
		const uint32_t yAxes = reprap.GetCurrentYAxes();
		reprap.GetMove().GetCurrentUserPosition(moveBuffer.coords, 0, xAxes, yAxes);
		for (size_t i = numTotalAxes; i < DRIVES; ++i)
		{
			moveBuffer.coords[i] = 0.0;
		}
		moveBuffer.moveType = 0;
		moveBuffer.isFirmwareRetraction = true;
		moveBuffer.usePressureAdvance = false;
		moveBuffer.filePos = (&gb == fileGCode) ? gb.GetFilePosition(fileInput->BytesCached()) : noFilePosition;
		moveBuffer.xAxes = xAxes;
		moveBuffer.yAxes = yAxes;

		if (retract)
		{
			// Set up the retract move
			const Tool * const tool = reprap.GetCurrentTool();
			if (tool != nullptr)
			{
				for (size_t i = 0; i < tool->DriveCount(); ++i)
				{
					moveBuffer.coords[numTotalAxes + tool->Drive(i)] = -retractLength;
				}
				moveBuffer.feedRate = retractSpeed;
				moveBuffer.canPauseAfter = false;			// don't pause after a retraction because that could cause too much retraction
				segmentsLeft = 1;
			}
			if (retractHop > 0.0)
			{
				gb.SetState(GCodeState::doingFirmwareRetraction);
			}
		}
		else if (retractHop > 0.0)
		{
			// Set up the reverse Z hop move
			moveBuffer.feedRate = platform.MaxFeedrate(Z_AXIS);
			moveBuffer.coords[Z_AXIS] -= retractHop;
			currentZHop = 0.0;
			moveBuffer.canPauseAfter = false;			// don't pause in the middle of a command
			segmentsLeft = 1;
			gb.SetState(GCodeState::doingFirmwareUnRetraction);
		}
		else
		{
			// No retract hop, so just un-retract
			const Tool * const tool = reprap.GetCurrentTool();
			if (tool != nullptr)
			{
				for (size_t i = 0; i < tool->DriveCount(); ++i)
				{
					moveBuffer.coords[numTotalAxes + tool->Drive(i)] = retractLength + retractExtra;
				}
				moveBuffer.feedRate = unRetractSpeed;
				moveBuffer.canPauseAfter = true;
				segmentsLeft = 1;
			}
		}
		isRetracted = retract;
	}
	return true;
}

// Load the specified filament into a tool
bool GCodes::LoadFilament(GCodeBuffer& gb, StringRef& reply, bool &error)
{
	Tool *tool = reprap.GetCurrentTool();
	if (tool == nullptr)
	{
		reply.copy("No tool selected");
		error = true;
		return true;
	}

	if (tool->GetFilament() == nullptr)
	{
		reply.copy("Loading filament into the selected tool is not supported");
		error = true;
		return true;
	}

	if (gb.Seen('S'))
	{
		char filamentName[FilamentNameLength];
		if (!gb.GetQuotedString(filamentName, ARRAY_SIZE(filamentName)) || StringEquals(filamentName, ""))
		{
			reply.copy("Invalid filament name");
			error = true;
			return true;
		}

		if (StringContains(filamentName, ",") >= 0)
		{
			reply.copy("Filament names must not contain commas");
			error = true;
			return true;
		}

		if (StringEquals(filamentName, tool->GetFilament()->GetName()))
		{
			// Filament already loaded - nothing to do
			return true;
		}

		if (tool->GetFilament()->IsLoaded())
		{
			reply.copy("Unload the current filament before you attempt to load another one");
			error = true;
			return true;
		}

		if (!reprap.GetPlatform().GetMassStorage()->DirectoryExists(FILAMENTS_DIRECTORY, filamentName))
		{
			reply.copy("Filament configuration directory not found");
			error = true;
			return true;
		}

		if (Filament::IsInUse(filamentName))
		{
			reply.copy("One filament type can be only assigned to a single tool");
			error = true;
			return true;
		}

		SafeStrncpy(filamentToLoad, filamentName, ARRAY_SIZE(filamentToLoad));
		gb.SetState(GCodeState::loadingFilament);

		scratchString.printf("%s%s/%s", FILAMENTS_DIRECTORY, filamentName, LOAD_FILAMENT_G);
		DoFileMacro(gb, scratchString.Pointer(), true);
		return true;
	}
	else if (tool->GetFilament()->IsLoaded())
	{
		reply.printf("Loaded filament in the selected tool: %s", tool->GetFilament()->GetName());
		return true;
	}
	else
	{
		reply.printf("No filament loaded in the selected tool");
		return true;
	}
}

// Unload the current filament from a tool
bool GCodes::UnloadFilament(GCodeBuffer& gb, StringRef& reply, bool &error)
{
	Tool *tool = reprap.GetCurrentTool();
	if (tool == nullptr)
	{
		reply.copy("No tool selected");
		error = true;
		return true;
	}

	if (tool->GetFilament() == nullptr)
	{
		reply.copy("Unloading filament from this tool is not supported");
		error = true;
		return true;
	}

	if (!tool->GetFilament()->IsLoaded())
	{
		// Filament already unloaded - nothing to do
		return true;
	}

	gb.SetState(GCodeState::unloadingFilament);
	scratchString.printf("%s%s/%s", FILAMENTS_DIRECTORY, tool->GetFilament()->GetName(), UNLOAD_FILAMENT_G);
	DoFileMacro(gb, scratchString.Pointer(), true);
	return true;
}

// Return the amount of filament extruded
float GCodes::GetRawExtruderPosition(size_t extruder) const
{
	return (extruder < numExtruders) ? lastRawExtruderPosition[extruder] : 0.0;
}

float GCodes::GetRawExtruderTotalByDrive(size_t extruder) const
{
	return (extruder < numExtruders) ? rawExtruderTotalByDrive[extruder] : 0.0;
}

// Cancel the current SD card print.
// This is called from Pid.cpp when there is a heater fault, and from elsewhere in this module.
void GCodes::CancelPrint()
{
	segmentsLeft = 0;
	isPaused = false;

	FileData& fileBeingPrinted = fileGCode->OriginalMachineState().fileState;

	fileInput->Reset(fileBeingPrinted);
	fileGCode->Init();

	if (fileBeingPrinted.IsLive())
	{
		fileBeingPrinted.Close();
	}

	reprap.GetPrintMonitor().StoppedPrint();

	reprap.GetMove().ResetMoveCounters();
	codeQueue->Clear();
}

// Return true if all the heaters for the specified tool are at their set temperatures
bool GCodes::ToolHeatersAtSetTemperatures(const Tool *tool, bool waitWhenCooling) const
{
	if (tool != nullptr)
	{
		for (size_t i = 0; i < tool->HeaterCount(); ++i)
		{
			if (!reprap.GetHeat().HeaterAtSetTemperature(tool->Heater(i), waitWhenCooling))
			{
				return false;
			}
		}
	}
	return true;
}

// Set the current position, optionally applying bed and axis compensation
void GCodes::SetMachinePosition(const float positionNow[DRIVES], bool doBedCompensation)
{
	memcpy(moveBuffer.coords, positionNow, sizeof(moveBuffer.coords[0] * numTotalAxes));
	reprap.GetMove().SetNewPosition(positionNow, doBedCompensation);
}

// Get the current position from the Move class
void GCodes::GetCurrentUserPosition()
{
	reprap.GetMove().GetCurrentUserPosition(moveBuffer.coords, 0, reprap.GetCurrentXAxes(), reprap.GetCurrentYAxes());
	ToolOffsetInverseTransform(moveBuffer.coords, currentUserPosition);
}

// Save position to a restore point
void GCodes::SavePosition(RestorePoint& rp, const GCodeBuffer& gb) const
{
	for (size_t axis = 0; axis < numVisibleAxes; ++axis)
	{
		rp.moveCoords[axis] = currentUserPosition[axis];
	}

	for (size_t drive = numTotalAxes; drive < DRIVES; ++drive)
	{
		rp.moveCoords[drive] = lastRawExtruderPosition[drive - numTotalAxes];	// get current extruder positions into pausedMoveBuffer
	}

	rp.feedRate = gb.MachineState().feedrate;
	rp.virtualExtruderPosition = GetVirtualExtruderPosition();

#if SUPPORT_IOBITS
	rp.ioBits = moveBuffer.ioBits;
#endif
}

// Restore user position form a restore point
void GCodes::RestorePosition(const RestorePoint& rp, GCodeBuffer& gb)
{
	for (size_t axis = 0; axis < numVisibleAxes; ++axis)
	{
		currentUserPosition[axis] = rp.moveCoords[axis];
	}
	gb.MachineState().feedrate = rp.feedRate;
#if SUPPORT_IOBITS
	moveBuffer.ioBits = rp.ioBits;
#endif
}

// Convert user coordinates to head reference point coordinates, optionally allowing for X axis mapping
// If the X axis is mapped to some other axes not including X, then the X coordinate of coordsOut will be left unchanged. So make sure it is suitably initialised before calling this.
void GCodes::ToolOffsetTransform(const float coordsIn[MaxAxes], float coordsOut[MaxAxes])
{
	const Tool * const currentTool = reprap.GetCurrentTool();
	if (currentTool == nullptr)
	{
		for (size_t axis = 0; axis < numVisibleAxes; ++axis)
		{
			coordsOut[axis] = (coordsIn[axis] * axisScaleFactors[axis]) - axisOffsets[axis];
		}
	}
	else
	{
		const AxesBitmap xAxes = currentTool->GetXAxisMap();
		const AxesBitmap yAxes = currentTool->GetYAxisMap();
		for (size_t axis = 0; axis < numVisibleAxes; ++axis)
		{
			if (   (axis != X_AXIS || IsBitSet<AxesBitmap>(xAxes, X_AXIS))
				&& (axis != Y_AXIS || IsBitSet<AxesBitmap>(yAxes, Y_AXIS))
			   )
			{
				const float totalOffset = currentTool->GetOffset()[axis] + axisOffsets[axis];
				const size_t inputAxis = (IsBitSet<AxesBitmap>(xAxes, axis)) ? X_AXIS
										: (IsBitSet<AxesBitmap>(yAxes, axis)) ? Y_AXIS
											: axis;
				coordsOut[axis] = (coordsIn[inputAxis] * axisScaleFactors[axis]) - totalOffset;
			}
		}
	}
	coordsOut[Z_AXIS] += (currentZHop + currentBabyStepZOffset);
}

// Convert head reference point coordinates to user coordinates, allowing for XY axis mapping
// Caution: coordsIn and coordsOut may address the same array!
void GCodes::ToolOffsetInverseTransform(const float coordsIn[MaxAxes], float coordsOut[MaxAxes])
{
	const Tool * const currentTool = reprap.GetCurrentTool();
	if (currentTool == nullptr)
	{
		for (size_t axis = 0; axis < numVisibleAxes; ++axis)
		{
			coordsOut[axis] = coordsIn[axis]/axisScaleFactors[axis];
		}
	}
	else
	{
		const uint32_t xAxes = reprap.GetCurrentXAxes();
		const uint32_t yAxes = reprap.GetCurrentYAxes();
		float xCoord = 0.0, yCoord = 0.0;
		size_t numXAxes = 0, numYAxes = 0;
		for (size_t axis = 0; axis < numVisibleAxes; ++axis)
		{
			coordsOut[axis] = coordsIn[axis] + currentTool->GetOffset()[axis];
			if (IsBitSet(xAxes, axis))
			{
				xCoord += coordsIn[axis]/axisScaleFactors[axis] + currentTool->GetOffset()[axis];
				++numXAxes;
			}
			if (IsBitSet(yAxes, axis))
			{
				yCoord += coordsIn[axis]/axisScaleFactors[axis] + currentTool->GetOffset()[axis];
				++numYAxes;
			}
		}
		if (numXAxes != 0)
		{
			coordsOut[X_AXIS] = xCoord/numXAxes;
		}
		if (numYAxes != 0)
		{
			coordsOut[Y_AXIS] = yCoord/numYAxes;
		}
	}
	coordsOut[Z_AXIS] -= (currentZHop + currentBabyStepZOffset)/axisScaleFactors[Z_AXIS];
}

const char *GCodes::TranslateEndStopResult(EndStopHit es)
{
	switch (es)
	{
	case EndStopHit::lowHit:
		return "at min stop";

	case EndStopHit::highHit:
		return "at max stop";

	case EndStopHit::lowNear:
		return "near min stop";

	case EndStopHit::noStop:
	default:
		return "not stopped";
	}
}

// Append a list of trigger endstops to a message
void GCodes::ListTriggers(StringRef reply, TriggerInputsBitmap mask)
{
	if (mask == 0)
	{
		reply.cat("(none)");
	}
	else
	{
		bool printed = false;
		for (unsigned int i = 0; i < DRIVES; ++i)
		{
			if (IsBitSet(mask, i))
			{
				if (printed)
				{
					reply.cat(' ');
				}
				if (i < numVisibleAxes)
				{
					reply.cat(axisLetters[i]);
				}
				else if (i >= numTotalAxes)
				{
					reply.catf("E%d", i - numTotalAxes);
				}
				printed = true;
			}
		}
	}
}

// M38 (SHA1 hash of a file) implementation:
bool GCodes::StartHash(const char* filename)
{
	// Get a FileStore object
	fileBeingHashed = platform.GetFileStore(FS_PREFIX, filename, false);
	if (fileBeingHashed == nullptr)
	{
		return false;
	}

	// Start hashing
	SHA1Reset(&hash);
	return true;
}

bool GCodes::AdvanceHash(StringRef &reply)
{
	// Read and process some more data from the file
	uint32_t buf32[(FILE_BUFFER_SIZE + 3) / 4];
	char *buffer = reinterpret_cast<char *>(buf32);

	int bytesRead = fileBeingHashed->Read(buffer, FILE_BUFFER_SIZE);
	if (bytesRead != -1)
	{
		SHA1Input(&hash, reinterpret_cast<const uint8_t *>(buffer), bytesRead);

		if (bytesRead != FILE_BUFFER_SIZE)
		{
			// Calculate and report the final result
			SHA1Result(&hash);
			for(size_t i = 0; i < 5; i++)
			{
				reply.catf("%x", hash.Message_Digest[i]);
			}

			// Clean up again
			fileBeingHashed->Close();
			fileBeingHashed = nullptr;
			return true;
		}
		return false;
	}

	// Something went wrong, we cannot read any more from the file
	fileBeingHashed->Close();
	fileBeingHashed = nullptr;
	return true;
}

bool GCodes::AllAxesAreHomed() const
{
	const AxesBitmap allAxes = LowestNBits<AxesBitmap>(numVisibleAxes);
	return (axesHomed & allAxes) == allAxes;
}

void GCodes::SetAllAxesNotHomed()
{
	axesHomed = 0;
}

// Write the config-override file returning true if an error occurred
bool GCodes::WriteConfigOverrideFile(StringRef& reply, const char *fileName) const
{
	FileStore * const f = platform.GetFileStore(platform.GetSysDir(), fileName, true);
	if (f == nullptr)
	{
		reply.printf("Failed to create file %s", fileName);
		return true;
	}

	bool ok = f->Write("; This is a system-generated file - do not edit\n");
	if (ok)
	{
		ok = reprap.GetMove().GetKinematics().WriteCalibrationParameters(f);
	}
	if (ok)
	{
		ok = reprap.GetHeat().WriteModelParameters(f);
	}
	if (ok)
	{
		ok = platform.WriteZProbeParameters(f);
	}
	if (!f->Close())
	{
		ok = false;
	}
	if (!ok)
	{
		reply.printf("Failed to write file %s", fileName);
		platform.GetMassStorage()->Delete(platform.GetSysDir(), fileName);
	}
	return !ok;
}

// Store a standard-format temperature report in 'reply'. This doesn't put a newline character at the end.
void GCodes::GenerateTemperatureReport(StringRef& reply) const
{
	Heat& heat = reprap.GetHeat();

	// Pronterface, Repetier etc. expect the temperatures to be reported for T0, T1 etc.
	// So scan the tool list and report the temperature of the heaters associated with each tool.
	// If the user configures tools T0, T1 etc. with 1 heater each, that will return what these programs expect.
	reply.copy("T");
	char sep = ':';
	for (const Tool *tool = reprap.GetFirstTool(); tool != nullptr; tool = tool->Next())
	{
		for (size_t i = 0; i < tool->HeaterCount(); ++i)
		{
			const int heater = tool->Heater(i);
			reply.catf("%c%.1f /%.1f", sep, heat.GetTemperature(heater), heat.GetTargetTemperature(heater));
			sep = ' ';
		}
	}

	const int bedHeater = heat.GetBedHeater();
	if (bedHeater >= 0)
	{
		reply.catf(" B:%.1f /%.1f", heat.GetTemperature(bedHeater), heat.GetTargetTemperature(bedHeater));
	}

	const int chamberHeater = heat.GetChamberHeater();
	if (chamberHeater >= 0)
	{
		reply.catf(" C:%.1f /%.1f", heat.GetTemperature(chamberHeater), heat.GetTargetTemperature(chamberHeater));
	}
}

// Check whether we need to report temperatures or status.
// 'reply' is a convenient buffer that is free for us to use.
void GCodes::CheckReportDue(GCodeBuffer& gb, StringRef& reply) const
{
	const uint32_t now = millis();
	if (gb.timerRunning)
	{
		if (now - gb.whenTimerStarted >= 1000)
		{
			if (platform.Emulating() == marlin && (&gb == serialGCode || &gb == telnetGCode))
			{
				// In Marlin emulation mode we should return a standard temperature report every second
				GenerateTemperatureReport(reply);
				reply.cat('\n');
				platform.Message(HOST_MESSAGE, reply.Pointer());
			}
			if (lastAuxStatusReportType >= 0)
			{
				// Send a standard status response for PanelDue
				OutputBuffer * const statusBuf = GenerateJsonStatusResponse(0, -1, ResponseSource::AUX);
				if (statusBuf != nullptr)
				{
					platform.AppendAuxReply(statusBuf);
				}
			}
			gb.whenTimerStarted = now;
		}
	}
	else
	{
		gb.whenTimerStarted = now;
		gb.timerRunning = true;
	}
}

// Generate a M408 response
// Return the output buffer containing the response, or nullptr if we failed
OutputBuffer *GCodes::GenerateJsonStatusResponse(int type, int seq, ResponseSource source) const
{
	OutputBuffer *statusResponse = nullptr;
	switch (type)
	{
		case 0:
		case 1:
			statusResponse = reprap.GetLegacyStatusResponse(type + 2, seq);
			break;

		case 2:
		case 3:
		case 4:
			statusResponse = reprap.GetStatusResponse(type - 1, source);
			break;

		case 5:
			statusResponse = reprap.GetConfigResponse();
			break;
	}
	if (statusResponse != nullptr)
	{
		statusResponse->cat('\n');
	}
	return statusResponse;
}

// Resource locking/unlocking

// Lock the resource, returning true if success.
// Locking the same resource more than once only locks it once, there is no lock count held.
bool GCodes::LockResource(const GCodeBuffer& gb, Resource r)
{
	if (resourceOwners[r] == &gb)
	{
		return true;
	}
	if (resourceOwners[r] == nullptr)
	{
		resourceOwners[r] = &gb;
		SetBit(gb.MachineState().lockedResources, r);
		return true;
	}
	return false;
}

bool GCodes::LockHeater(const GCodeBuffer& gb, int heater)
{
	if (heater >= 0 && heater < (int)Heaters)
	{
		return LockResource(gb, HeaterResourceBase + heater);
	}
	return true;
}

bool GCodes::LockFan(const GCodeBuffer& gb, int fan)
{
	if (fan >= 0 && fan < (int)NUM_FANS)
	{
		return LockResource(gb, FanResourceBase + fan);
	}
	return true;
}

// Lock the unshareable parts of the file system
bool GCodes::LockFileSystem(const GCodeBuffer &gb)
{
	return LockResource(gb, FileSystemResource);
}

// Lock movement
bool GCodes::LockMovement(const GCodeBuffer& gb)
{
	return LockResource(gb, MoveResource);
}

// Release all locks, except those that were owned when the current macro was started
void GCodes::UnlockAll(const GCodeBuffer& gb)
{
	const GCodeMachineState * const mc = gb.MachineState().previous;
	const uint32_t resourcesToKeep = (mc == nullptr) ? 0 : mc->lockedResources;
	for (size_t i = 0; i < NumResources; ++i)
	{
		if (resourceOwners[i] == &gb && !IsBitSet(resourcesToKeep, i))
		{
			resourceOwners[i] = nullptr;
			ClearBit(gb.MachineState().lockedResources, i);
		}
	}
}

// Append a list of axes to a string
void GCodes::AppendAxes(StringRef& reply, AxesBitmap axes) const
{
	for (size_t axis = 0; axis < numVisibleAxes; ++axis)
	{
		if (IsBitSet(axes, axis))
		{
			reply.cat(axisLetters[axis]);
		}
	}
}

// End
