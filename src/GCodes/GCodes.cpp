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
#include "Heating/HeaterProtection.h"
#include "Platform.h"
#include "Movement/Move.h"
#include "Scanner.h"
#include "PrintMonitor.h"
#include "RepRap.h"
#include "Tools/Tool.h"

#if HAS_WIFI_NETWORKING
# include "FirmwareUpdater.h"
#endif

const size_t gcodeReplyLength = 2048;			// long enough to pass back a reasonable number of files in response to M20

GCodes::GCodes(Platform& p) :
	platform(p), machineType(MachineType::fff), active(false),
#if HAS_VOLTAGE_MONITOR
	powerFailScript(nullptr),
#endif
	isFlashing(false), fileBeingHashed(nullptr), lastWarningMillis(0)
{
	httpInput = new RegularGCodeInput;
	telnetInput = new RegularGCodeInput;
	fileInput = new FileGCodeInput();
	serialInput = new StreamGCodeInput(SERIAL_MAIN_DEVICE);
	auxInput = new StreamGCodeInput(SERIAL_AUX_DEVICE);

	httpGCode = new GCodeBuffer("http", HttpMessage, false);
	telnetGCode = new GCodeBuffer("telnet", TelnetMessage, true);
	fileGCode = new GCodeBuffer("file", GenericMessage, true);
	serialGCode = new GCodeBuffer("serial", UsbMessage, true);
	auxGCode = new GCodeBuffer("aux", LcdMessage, false);
	daemonGCode = new GCodeBuffer("daemon", GenericMessage, false);
#if SUPPORT_12864_LCD
	lcdGCode = new GCodeBuffer("lcd", GenericMessage, false);
#endif
	queuedGCode = new GCodeBuffer("queue", GenericMessage, false);
	autoPauseGCode = new GCodeBuffer("autopause", GenericMessage, false);
	codeQueue = new GCodeQueue();
}

void GCodes::Exit()
{
	active = false;
}

void GCodes::Init()
{
	numVisibleAxes = numTotalAxes = XYZ_AXES;			// must set this up before calling Reset()
	memset(axisLetters, 0, sizeof(axisLetters));
	axisLetters[0] = 'X';
	axisLetters[1] = 'Y';
	axisLetters[2] = 'Z';

	numExtruders = MaxExtruders;
	Reset();

	distanceScale = 1.0;
	arcSegmentLength = DefaultArcSegmentLength;
	virtualExtruderPosition = rawExtruderTotal = 0.0;
	for (size_t extruder = 0; extruder < MaxExtruders; extruder++)
	{
		rawExtruderTotalByDrive[extruder] = 0.0;
	}
	eofString = EOF_STRING;
	eofStringCounter = 0;
	eofStringLength = strlen(eofString);
	runningConfigFile = false;
	doingToolChange = false;
	active = true;
	fileSize = 0;
	longWait = millis();
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

	spindleMaxRpm = DefaultMaxSpindleRpm;
	laserMaxPower = DefaultMaxLaserPower;

	heaterFaultState = HeaterFaultState::noFault;
	heaterFaultTime = 0;
	heaterFaultTimeout = DefaultHeaterFaultTimeout;

#if SUPPORT_SCANNER
	reprap.GetScanner().SetGCodeBuffer(serialGCode);
#endif
}

// This is called from Init and when doing an emergency stop
void GCodes::Reset()
{
	// Here we could reset the input sources as well, but this would mess up M122\nM999
	// because both codes are sent at once from the web interface. Hence we don't do this here.
	httpGCode->Reset();
	telnetGCode->Reset();
	fileGCode->Reset();
	serialGCode->Reset();
	auxGCode->Reset();
	auxGCode->SetCommsProperties(1);					// by default, we require a checksum on the aux port
	daemonGCode->Reset();
#if SUPPORT_12864_LCD
	lcdGCode->Reset();
#endif
	queuedGCode->Reset();
	autoPauseGCode->Reset();

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
		axisScaleFactors[i] = 1.0;
#if SUPPORT_WORKPLACE_COORDINATES
		for (size_t j = 0; j < 10; ++j)
		{
			workplaceCoordinates[j][i] = 0.0;
		}
#else
		axisOffsets[i] = 0.0;
#endif
	}

#if SUPPORT_WORKPLACE_COORDINATES
	currentCoordinateSystem = 0;
#endif

	for (size_t i = 0; i < DRIVES; ++i)
	{
		moveBuffer.coords[i] = 0.0;						// clear out all axis and extruder coordinates
	}

	ClearMove();
	ClearBabyStepping();
	moveBuffer.xAxes = DefaultXAxisMapping;
	moveBuffer.yAxes = DefaultYAxisMapping;
	moveBuffer.virtualExtruderPosition = 0.0;

#if SUPPORT_IOBITS
	moveBuffer.ioBits = 0;
#endif

	reprap.GetMove().GetKinematics().GetAssumedInitialPosition(numVisibleAxes, moveBuffer.coords);
	ToolOffsetInverseTransform(moveBuffer.coords, currentUserPosition);

	pauseRestorePoint.Init();
	toolChangeRestorePoint.Init();

	for (size_t i = 0; i < MaxTriggers; ++i)
	{
		triggers[i].Init();
	}
	triggersPending = 0;

	simulationMode = 0;
	exitSimulationWhenFileComplete = false;
	simulationTime = 0.0;
	isPaused = false;
#if HAS_VOLTAGE_MONITOR
	isPowerFailPaused = false;
#endif
	doingToolChange = false;
	doingManualBedProbe = false;
	pausePending = false;
	probeIsDeployed = false;
	moveBuffer.filePos = noFilePosition;
	lastEndstopStates = platform.GetAllEndstopStates();
	firmwareUpdateModuleMap = 0;
	lastFilamentError = FilamentSensorStatus::ok;

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
	if (!fileBeingPrinted.IsLive())
	{
		return -1.0;
	}

	const FilePosition len = fileBeingPrinted.Length();
	if (len == 0)
	{
		return 0.0;
	}

    const FilePosition bytesCached = fileGCode->IsDoingFileMacro() ? 0: fileInput->BytesCached();
	return (float)(fileBeingPrinted.GetPosition() - bytesCached) / (float)len;
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
	CheckHeaterFault();
	CheckFilament();

	// Get the GCodeBuffer that we want to process a command from. Give priority to auto-pause.
	GCodeBuffer *gbp = autoPauseGCode;
	if (gbp->IsCompletelyIdle() && !(gbp->MachineState().fileState.IsLive()))
	{
		gbp = gcodeSources[nextGcodeSource];
		++nextGcodeSource;											// move on to the next gcode source ready for next time
		if (nextGcodeSource == ARRAY_SIZE(gcodeSources) - 1)		// the last one is autoPauseGCode, so don't do it again
		{
			nextGcodeSource = 0;
		}
	}
	GCodeBuffer& gb = *gbp;

	// Set up a buffer for the reply
	String<gcodeReplyLength> reply;

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
					StopPrint(false);
				}
				else
				{
					FileMacroCyclesReturn(gb);
				}
			}
		}
		else
		{
			StartNextGCode(gb, reply.GetRef());
		}
	}
	else
	{
		RunStateMachine(gb, reply.GetRef());			// Execute the state machine
	}

	// Check if we need to display a warning
	const uint32_t now = millis();
	if (now - lastWarningMillis >= MinimumWarningInterval)
	{
		if (displayNoToolWarning)
		{
			platform.Message(ErrorMessage, "Attempting to extrude with no tool selected.\n");
			displayNoToolWarning = false;
			lastWarningMillis = now;
		}
		if (displayDeltaNotHomedWarning)
		{
			platform.Message(ErrorMessage, "Attempt to move the head of a Delta or SCARA printer before homing it\n");
			displayDeltaNotHomedWarning = false;
			lastWarningMillis = now;
		}
	}
	platform.ClassReport(longWait);
}

// Execute a step of the state machine
void GCodes::RunStateMachine(GCodeBuffer& gb, const StringRef& reply)
{
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
					EndStopPosition stopType;
					EndStopInputType dummy;
					platform.GetEndStopConfiguration(axis, stopType, dummy);
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
			gb.SetState(GCodeState::normal);
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
						currentTool->SetOffset(axis, (toolChangeRestorePoint.moveCoords[axis] - currentUserPosition[axis]) + gb.GetFValue(), true);
						break;
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
		memcpy(toolChangeRestorePoint.moveCoords, currentUserPosition, MaxAxes * sizeof(currentUserPosition[0]));
		toolChangeRestorePoint.feedRate = gb.MachineState().feedrate;
		gb.AdvanceState();
		if ((gb.MachineState().toolChangeParam & TFreeBit) != 0)
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
				scratchString.printf("tpre%d.g", gb.MachineState().newToolNumber);
				DoFileMacro(gb, scratchString.Pointer(), false);
			}
		}
		break;

	case GCodeState::toolChange2:		// Select the new tool (even if it doesn't exist - that just deselects all tools) and run tpost
	case GCodeState::m109ToolChange2:	// Select the new tool (even if it doesn't exist - that just deselects all tools) and run tpost
		if (LockMovementAndWaitForStandstill(gb))		// wait for tpre.g to finish executing
		{
			reprap.SelectTool(gb.MachineState().newToolNumber, simulationMode != 0);
			GetCurrentUserPosition();					// get the actual position of the new tool

			gb.AdvanceState();
			if (AllAxesAreHomed())
			{
				if (reprap.GetTool(gb.MachineState().newToolNumber) != nullptr && (gb.MachineState().toolChangeParam & TPostBit) != 0)
				{
					scratchString.printf("tpost%d.g", gb.MachineState().newToolNumber);
					DoFileMacro(gb, scratchString.Pointer(), false);
				}
			}
		}
		break;

	case GCodeState::toolChangeComplete:
	case GCodeState::m109ToolChangeComplete:
		if (LockMovementAndWaitForStandstill(gb))		// wait for tpost.g to finish executing
		{
			// Restore the original Z axis user position, so that different tool Z offsets work even if the first move after the tool change doesn't have a Z coordinate
			currentUserPosition[Z_AXIS] = toolChangeRestorePoint.moveCoords[Z_AXIS];
			gb.MachineState().feedrate = toolChangeRestorePoint.feedRate;
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
		if (cancelWait || simulationMode != 0 || ToolHeatersAtSetTemperatures(reprap.GetCurrentTool(), gb.MachineState().waitWhileCooling))
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
				DoFileMacro(gb, PAUSE_G, true);
			}
		}
		break;

	case GCodeState::pausing2:
		if (LockMovementAndWaitForStandstill(gb))
		{
			reply.copy("Printing paused");
			platform.Message(LogMessage, "Printing paused\n");
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
			moveBuffer.isCoordinated = false;
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
			totalSegments = 1;
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
			virtualExtruderPosition = pauseRestorePoint.virtualExtruderPosition;	// reset the extruder position in case we are receiving absolute extruder moves
			moveBuffer.virtualExtruderPosition = pauseRestorePoint.virtualExtruderPosition;
			fileGCode->MachineState().feedrate = pauseRestorePoint.feedRate;
			moveFractionToSkip = pauseRestorePoint.proportionDone;
			isPaused = false;
			reply.copy("Printing resumed");
			platform.Message(LogMessage, "Printing resumed\n");
			gb.SetState(GCodeState::normal);
		}
		break;

	case GCodeState::flashing1:
#if HAS_WIFI_NETWORKING
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
				reprap.StandbyTool(tool->Number(), simulationMode != 0);
			}
			reprap.GetHeat().SwitchOffAll(true);
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
			const GridDefinition& grid = move.AccessHeightMap().GetGrid();
			const float x = grid.GetXCoordinate(gridXindex);
			const float y = grid.GetYCoordinate(gridYindex);
			if (grid.IsInRadius(x, y))
			{
				if (move.IsAccessibleProbePoint(x, y))
				{
					moveBuffer.moveType = 0;
					moveBuffer.isCoordinated = false;
					moveBuffer.endStopsToCheck = 0;
					moveBuffer.usePressureAdvance = false;
					moveBuffer.filePos = noFilePosition;
					moveBuffer.coords[X_AXIS] = x - platform.GetCurrentZProbeParameters().xOffset;
					moveBuffer.coords[Y_AXIS] = y - platform.GetCurrentZProbeParameters().yOffset;
					moveBuffer.coords[Z_AXIS] = platform.GetZProbeStartingHeight();
					moveBuffer.feedRate = platform.GetZProbeTravelSpeed();
					moveBuffer.xAxes = DefaultXAxisMapping;
					moveBuffer.yAxes = DefaultYAxisMapping;
					totalSegments = 1;
					segmentsLeft = 1;
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

	case GCodeState::gridProbing2a:		// ready to probe the current grid probe point
		if (LockMovementAndWaitForStandstill(gb))
		{
			gb.AdvanceState();
			if (platform.GetZProbeType() == ZProbeType::blTouch)
			{
				DoFileMacro(gb, DEPLOYPROBE_G, false);				// bltouch needs to be redeployed prior to each probe point
			}
		}
		break;

	case GCodeState::gridProbing2b:		// ready to probe the current grid probe point
		if (LockMovementAndWaitForStandstill(gb))
		{
			lastProbedTime = millis();
			tapsDone = 0;
			g30zHeightErrorSum = 0.0;
			g30zHeightErrorLowestDiff = 1000.0;
			if (platform.GetZProbeType() != ZProbeType::none && platform.GetCurrentZProbeParameters().turnHeatersOff)
			{
				reprap.GetHeat().SuspendHeaters(true);
			}
			gb.AdvanceState();
		}
		break;

	case GCodeState::gridProbing3:	// ready to probe the current grid probe point
		if (millis() - lastProbedTime >= (uint32_t)(platform.GetCurrentZProbeParameters().recoveryTime * SecondsToMillis))
		{
			// Probe the bed at the current XY coordinates
			// Check for probe already triggered at start
			if (platform.GetZProbeType() == ZProbeType::none)
			{
				// No Z probe, so do manual mesh levelling instead
				UnlockAll(gb);															// release the movement lock to allow manual Z moves
				gb.AdvanceState();														// resume at next state when user has finished adjusting the height
				doingManualBedProbe = true;												// suspend the Z movement limit
				DoManualProbe(gb);
			}
			else if (platform.GetZProbeResult() == EndStopHit::lowHit)
			{
				reprap.GetHeat().SuspendHeaters(false);
				platform.Message(ErrorMessage, "Z probe already triggered before probing move started");
				gb.SetState(GCodeState::normal);
				if (platform.GetZProbeType() != ZProbeType::none && !probeIsDeployed)
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
				moveBuffer.isCoordinated = false;
				moveBuffer.endStopsToCheck = ZProbeActive;
				moveBuffer.usePressureAdvance = false;
				moveBuffer.filePos = noFilePosition;
				moveBuffer.coords[Z_AXIS] = -platform.GetZProbeDiveHeight();
				moveBuffer.feedRate = platform.GetCurrentZProbeParameters().probeSpeed;
				moveBuffer.xAxes = DefaultXAxisMapping;
				moveBuffer.yAxes = DefaultYAxisMapping;
				totalSegments = 1;
				segmentsLeft = 1;
				gb.AdvanceState();
			}
		}
		break;

	case GCodeState::gridProbing4:	// ready to lift the probe after probing the current grid probe point
		if (LockMovementAndWaitForStandstill(gb))
		{
			doingManualBedProbe = false;
			++tapsDone;
			reprap.GetHeat().SuspendHeaters(false);
			if (platform.GetZProbeType() == ZProbeType::none)
			{
				// No Z probe, so we are doing manual mesh levelling. Take the current Z height as the height error.
				g30zHeightError = moveBuffer.coords[Z_AXIS];
			}
			else
			{
				platform.SetProbing(false);
				if (!zProbeTriggered)
				{
					platform.Message(ErrorMessage, "Z probe was not triggered during probing move\n");
					gb.SetState(GCodeState::normal);
					if (platform.GetZProbeType() != ZProbeType::none && !probeIsDeployed)
					{
						DoFileMacro(gb, RETRACTPROBE_G, false);
					}
					break;
				}

				g30zHeightError = moveBuffer.coords[Z_AXIS] - platform.ZProbeStopHeight();
				g30zHeightErrorSum += g30zHeightError;
			}

			// Move back up to the dive height
			moveBuffer.moveType = 0;
			moveBuffer.isCoordinated = false;
			moveBuffer.endStopsToCheck = 0;
			moveBuffer.usePressureAdvance = false;
			moveBuffer.filePos = noFilePosition;
			moveBuffer.coords[Z_AXIS] = platform.GetZProbeStartingHeight();
			moveBuffer.feedRate = platform.GetZProbeTravelSpeed();
			moveBuffer.xAxes = DefaultXAxisMapping;
			moveBuffer.yAxes = DefaultYAxisMapping;
			totalSegments = 1;
			segmentsLeft = 1;
			gb.AdvanceState();

			if (platform.GetZProbeType() == ZProbeType::blTouch)
			{
				DoFileMacro(gb, RETRACTPROBE_G, false);			// bltouch needs to be retracted whehn it triggers
			}
		}
		break;

	case GCodeState::gridProbing5:	// finished probing a point and moved back to the dive height
		if (LockMovementAndWaitForStandstill(gb))
		{
			// See whether we need to do any more taps
			const ZProbe& params = platform.GetCurrentZProbeParameters();
			if (tapsDone >= 2)
			{
				g30zHeightErrorLowestDiff = min<float>(g30zHeightErrorLowestDiff, fabsf(g30zHeightError - g30PrevHeightError));
			}
			const bool acceptReading = (params.maxTaps < 2 || (tapsDone >= 2 && g30zHeightErrorLowestDiff <= params.tolerance));
			if (acceptReading)
			{
				reprap.GetMove().AccessHeightMap().SetGridHeight(gridXindex, gridYindex, g30zHeightError);
				gb.AdvanceState();
			}
			else if (tapsDone < params.maxTaps)
			{
				// Tap again
				lastProbedTime = millis();
				g30PrevHeightError = g30zHeightError;
				gb.SetState(GCodeState::gridProbing3);
			}
			else
			{
				platform.Message(ErrorMessage, "Z probe readings not consistent\n");
				gb.SetState(GCodeState::normal);
				if (platform.GetZProbeType() != ZProbeType::none && !probeIsDeployed)
				{
					DoFileMacro(gb, RETRACTPROBE_G, false);
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
				if (platform.GetZProbeType() != ZProbeType::none && !probeIsDeployed)
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

	case GCodeState::gridProbing7:
		// Finished probing the grid, and retracted the probe if necessary
		{
			float mean, deviation;
			const uint32_t numPointsProbed = reprap.GetMove().AccessHeightMap().GetStatistics(mean, deviation);
			if (numPointsProbed >= 4)
			{
				reply.printf("%" PRIu32 " points probed, mean error %.3f, deviation %.3f\n", numPointsProbed, (double)mean, (double)deviation);
				error = SaveHeightMap(gb, reply);
				reprap.GetMove().AccessHeightMap().ExtrapolateMissing();
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
		moveBuffer.isCoordinated = false;
		moveBuffer.endStopsToCheck = 0;
		moveBuffer.usePressureAdvance = false;
		moveBuffer.filePos = noFilePosition;
		moveBuffer.coords[Z_AXIS] = platform.GetZProbeStartingHeight();
		moveBuffer.feedRate = platform.GetZProbeTravelSpeed();
		moveBuffer.xAxes = DefaultXAxisMapping;
		moveBuffer.yAxes = DefaultYAxisMapping;
		totalSegments = 1;
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
			moveBuffer.isCoordinated = false;
			moveBuffer.endStopsToCheck = 0;
			moveBuffer.usePressureAdvance = false;
			moveBuffer.filePos = noFilePosition;
			(void)reprap.GetMove().GetProbeCoordinates(g30ProbePointIndex, moveBuffer.coords[X_AXIS], moveBuffer.coords[Y_AXIS], true);
			moveBuffer.coords[Z_AXIS] = platform.GetZProbeStartingHeight();
			moveBuffer.feedRate = platform.GetZProbeTravelSpeed();
			moveBuffer.xAxes = DefaultXAxisMapping;
			moveBuffer.yAxes = DefaultYAxisMapping;
			totalSegments = 1;
			segmentsLeft = 1;

			gb.AdvanceState();
		}
		break;

	case GCodeState::probingAtPoint2a:
		if (LockMovementAndWaitForStandstill(gb))
		{
			gb.AdvanceState();
			if (platform.GetZProbeType() == ZProbeType::blTouch)
			{
				DoFileMacro(gb, DEPLOYPROBE_G, false);				// bltouch needs to be redeployed prior to each probe point
			}
		}
		break;

	case GCodeState::probingAtPoint2b:
		// Executing G30 with a P parameter. The move to put the head at the specified XY coordinates has been commanded.
		// OR initial state when executing G30 with no P parameter
		if (LockMovementAndWaitForStandstill(gb))
		{
			// Head has finished moving to the correct XY position
			lastProbedTime = millis();			// start the probe recovery timer
			tapsDone = 0;
			g30zHeightErrorSum = 0.0;
			g30zHeightErrorLowestDiff = 1000.0;
			if (platform.GetZProbeType() != ZProbeType::none && platform.GetCurrentZProbeParameters().turnHeatersOff)
			{
				reprap.GetHeat().SuspendHeaters(true);
			}
			gb.AdvanceState();
		}
		break;

	case GCodeState::probingAtPoint3:
		// Executing G30 with a P parameter. The move to put the head at the specified XY coordinates has been completed and the recovery timer started.
		// OR executing G30 without a P parameter, and the recovery timer has been started.
		if (millis() - lastProbedTime >= (uint32_t)(platform.GetCurrentZProbeParameters().recoveryTime * SecondsToMillis))
		{
			// The probe recovery time has elapsed, so we can start the probing  move
			if (platform.GetZProbeType() == ZProbeType::none)
			{
				// No Z probe, so we are doing manual 'probing'
				UnlockAll(gb);															// release the movement lock to allow manual Z moves
				gb.AdvanceState();														// resume at the next state when the user has finished
				doingManualBedProbe = true;												// suspend the Z movement limit
				DoManualProbe(gb);
			}
			else if (platform.GetZProbeResult() == EndStopHit::lowHit)		// check for probe already triggered at start
			{
				// Z probe is already triggered at the start of the move, so abandon the probe and record an error
				reprap.GetHeat().SuspendHeaters(false);
				platform.Message(ErrorMessage, "Z probe already triggered at start of probing move\n");
				if (g30ProbePointIndex >= 0)
				{
					reprap.GetMove().SetZBedProbePoint(g30ProbePointIndex, platform.GetZProbeDiveHeight(), true, true);
				}
				gb.SetState(GCodeState::normal);										// no point in doing anything else
				if (platform.GetZProbeType() != ZProbeType::none && !probeIsDeployed)
				{
					DoFileMacro(gb, RETRACTPROBE_G, false);
				}
			}
			else
			{
				zProbeTriggered = false;
				platform.SetProbing(true);
				moveBuffer.moveType = 0;
				moveBuffer.isCoordinated = false;
				moveBuffer.endStopsToCheck = ZProbeActive;
				moveBuffer.usePressureAdvance = false;
				moveBuffer.filePos = noFilePosition;
				moveBuffer.coords[Z_AXIS] = (GetAxisIsHomed(Z_AXIS))
											? -platform.GetZProbeDiveHeight()			// Z axis has been homed, so no point in going very far
											: -1.1 * platform.AxisTotalLength(Z_AXIS);	// Z axis not homed yet, so treat this as a homing move
				moveBuffer.feedRate = platform.GetCurrentZProbeParameters().probeSpeed;
				moveBuffer.xAxes = DefaultXAxisMapping;
				moveBuffer.yAxes = DefaultYAxisMapping;
				totalSegments = 1;
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
			reprap.GetHeat().SuspendHeaters(false);
			doingManualBedProbe = false;
			hadProbingError = false;
			++tapsDone;
			if (platform.GetZProbeType() == ZProbeType::none)
			{
				// No Z probe, so we are doing manual mesh levelling. Take the current Z height as the height error.
				g30zHeightError = moveBuffer.coords[Z_AXIS];
			}
			else
			{
				platform.SetProbing(false);
				if (!zProbeTriggered)
				{
					platform.Message(ErrorMessage, "Z probe was not triggered during probing move\n");
					g30zHeightError = 0.0;
					hadProbingError = true;
				}
				else
				{
					// Successful probing
					float heightAdjust = 0.0;
					bool dummy;
					gb.TryGetFValue('H', heightAdjust, dummy);
					float m[MaxAxes];
					reprap.GetMove().GetCurrentMachinePosition(m, false);		// get height without bed compensation
					g30zStoppedHeight = m[Z_AXIS] - heightAdjust;				// save for later
					g30zHeightError = g30zStoppedHeight - platform.ZProbeStopHeight();
					g30zHeightErrorSum += g30zHeightError;
				}
			}

			if (g30ProbePointIndex < 0)
			{
				// Simple G30 probing move
				gb.SetState(GCodeState::normal);								// usually nothing more to do except perhaps retract the probe
				if (!hadProbingError)
				{
					if (g30SValue == -1 || g30SValue == -2)
					{
						// G30 S-1 or S-2 command
						gb.SetState(GCodeState::probingAtPoint7);				// special state for reporting the stopped height at the end
					}
					else
					{
						// Reset the Z axis origin according to the height error
						moveBuffer.coords[Z_AXIS] -= g30zHeightError;
						reprap.GetMove().SetNewPosition(moveBuffer.coords, false);
						ToolOffsetInverseTransform(moveBuffer.coords, currentUserPosition);
						SetAxisIsHomed(Z_AXIS);									//TODO this is only correct if the Z axis is Cartesian-like!
					}
				}

				if (platform.GetZProbeType() != ZProbeType::none && !probeIsDeployed)
				{
					DoFileMacro(gb, RETRACTPROBE_G, false);						// retract the probe before moving to the new state
				}
			}
			else
			{
				// Move back up to the dive height before we change anything, in particular before we adjust leadscrews
				moveBuffer.moveType = 0;
				moveBuffer.isCoordinated = false;
				moveBuffer.endStopsToCheck = 0;
				moveBuffer.usePressureAdvance = false;
				moveBuffer.filePos = noFilePosition;
				moveBuffer.coords[Z_AXIS] = platform.GetZProbeStartingHeight();
				moveBuffer.feedRate = platform.GetZProbeTravelSpeed();
				moveBuffer.xAxes = DefaultXAxisMapping;
				moveBuffer.yAxes = DefaultYAxisMapping;
				totalSegments = 1;
				segmentsLeft = 1;
				gb.AdvanceState();

				if (platform.GetZProbeType() == ZProbeType::blTouch)
				{
					DoFileMacro(gb, RETRACTPROBE_G, false);						// bltouch needs to be retracted when it triggers
				}
			}
		}
		break;

	case GCodeState::probingAtPoint5:
		// Here when we were probing at a numbered point and we have moved the head back up to the dive height
		if (LockMovementAndWaitForStandstill(gb))
		{
			// See whether we need to do any more taps
			const ZProbe& params = platform.GetCurrentZProbeParameters();
			if (tapsDone >= 2)
			{
				g30zHeightErrorLowestDiff = min<float>(g30zHeightErrorLowestDiff, fabsf(g30zHeightError - g30PrevHeightError));
			}
			const bool acceptReading = (hadProbingError || params.maxTaps < 2 || (tapsDone >= 2 && g30zHeightErrorLowestDiff <= params.tolerance));
			if (!acceptReading && tapsDone < params.maxTaps)
			{
				// Tap again
				g30PrevHeightError = g30zHeightError;
				lastProbedTime = millis();
				gb.SetState(GCodeState::probingAtPoint3);
			}
			else
			{
				if (acceptReading)
				{
					if (tapsDone >= 2)
					{
						g30zHeightError = (g30zHeightError + g30PrevHeightError)/2;		// take the average of the two readings
					}
				}
				else
				{
					// We no longer flag this as a probing error, instead we take the average and issue a warning
					platform.Message(WarningMessage, "Z probe readings not consistent\n");
					g30zHeightError = g30zHeightErrorSum/tapsDone;
				}

				reprap.GetMove().SetZBedProbePoint(g30ProbePointIndex, g30zHeightError, true, hadProbingError);
				gb.AdvanceState();
				if (platform.GetZProbeType() != ZProbeType::none && !probeIsDeployed)
				{
					DoFileMacro(gb, RETRACTPROBE_G, false);
				}
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
			}
		}

		gb.SetState(GCodeState::normal);
		break;

	case GCodeState::probingAtPoint7:
		// Here when we have finished executing G30 S-1 or S-2 including retracting the probe if necessary
		if (g30SValue == -2)
		{
			// Adjust the Z offset of the current tool to account for the height error
			Tool *const tool = reprap.GetCurrentTool();
			if (tool == nullptr)
			{
				platform.Message(ErrorMessage, "G30 S-2 commanded with no tool selected");
				error = true;
			}
			else
			{
				tool->SetOffset(Z_AXIS, tool->GetOffset(Z_AXIS) + g30zHeightError, true);
			}
		}
		else
		{
			// Just print the stop height
			reply.printf("Stopped at height %.3f mm", (double)g30zStoppedHeight);
		}
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
			moveBuffer.isCoordinated = false;
			moveBuffer.isFirmwareRetraction = true;
			moveBuffer.usePressureAdvance = false;
			moveBuffer.filePos = (&gb == fileGCode) ? gb.GetFilePosition(fileInput->BytesCached()) : noFilePosition;
			moveBuffer.canPauseAfter = false;			// don't pause after a retraction because that could cause too much retraction
			moveBuffer.xAxes = xAxes;
			moveBuffer.yAxes = yAxes;
			totalSegments = 1;
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
				moveBuffer.isCoordinated = false;
				moveBuffer.isFirmwareRetraction = true;
				moveBuffer.usePressureAdvance = false;
				moveBuffer.filePos = (&gb == fileGCode) ? gb.MachineState().fileState.GetPosition() - fileInput->BytesCached() : noFilePosition;
				moveBuffer.canPauseAfter = true;
				moveBuffer.xAxes = xAxes;
				moveBuffer.yAxes = yAxes;
				totalSegments = 1;
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
			SaveResumeInfo(true);											// create the resume file so that we can resume after power down
			platform.Message(LoggedGenericMessage, "Print auto-paused due to low voltage\n");
			gb.SetState(GCodeState::normal);
		}
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
		gb.timerRunning = false;
		UnlockAll(gb);
		HandleReply(gb, error, reply.Pointer());
	}
}

// Start a new gcode, or continue to execute one that has already been started:
void GCodes::StartNextGCode(GCodeBuffer& gb, const StringRef& reply)
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

void GCodes::DoFilePrint(GCodeBuffer& gb, const StringRef& reply)
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
			// Finished printing SD card file.
			// We never get here if the file ends in M0 because CancelPrint gets called directly in that case.
			// Don't close the file until all moves have been completed, in case the print gets paused.
			// Also, this keeps the state as 'Printing' until the print really has finished.
			if (   LockMovementAndWaitForStandstill(gb)					// wait until movement has finished
				&& IsCodeQueueIdle()									// must also wait until deferred command queue has caught up
			   )
			{
				StopPrint(true);
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
					gb.Put("M226");
					pausePending = false;
				}
			}
		}
	}
}

// Restore positions etc. when exiting simulation mode
void GCodes::EndSimulation(GCodeBuffer *gb)
{
	// Ending a simulation, so restore the position
	RestorePosition(simulationRestorePoint, gb);
	ToolOffsetTransform(currentUserPosition, moveBuffer.coords);
	reprap.GetMove().SetNewPosition(simulationRestorePoint.moveCoords, true);
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
			if (!IsReallyPrinting())
			{
				ClearBit(triggersPending, lowestTriggerPending);	// ignore a pause trigger if we are already paused
			}
			else if (LockMovement(*daemonGCode))					// need to lock movement before executing the pause macro
			{
				ClearBit(triggersPending, lowestTriggerPending);	// clear the trigger
				DoPause(*daemonGCode, PauseReason::trigger, "Print paused by external trigger");
			}
		}
		else
		{
			ClearBit(triggersPending, lowestTriggerPending);		// clear the trigger
			String<25> filename;
			filename.GetRef().printf(SYS_DIR "trigger%u.g", lowestTriggerPending);
			DoFileMacro(*daemonGCode, filename.Pointer(), true);
		}
	}
}

// Check for and respond to filament errors
void GCodes::CheckFilament()
{
	if (   lastFilamentError != FilamentSensorStatus::ok			// check for a filament error
		&& IsReallyPrinting()
		&& autoPauseGCode->IsCompletelyIdle()
		&& LockMovement(*autoPauseGCode)							// need to lock movement before executing the pause macro
	   )
	{
		String<100> filamentErrorString;
		filamentErrorString.GetRef().printf("Extruder %u reports %s", lastFilamentErrorExtruder, FilamentMonitor::GetErrorMessage(lastFilamentError));
		DoPause(*autoPauseGCode, PauseReason::filament, filamentErrorString.Pointer());
		lastFilamentError = FilamentSensorStatus::ok;
		platform.Message(LogMessage, filamentErrorString.c_str());
	}
}

// Log a filament error. Called by Platform when a filament sensor reports an incorrect status and a print is in progress.
void GCodes::FilamentError(size_t extruder, FilamentSensorStatus fstat)
{
	if (lastFilamentError == FilamentSensorStatus::ok)
	{
		lastFilamentErrorExtruder = extruder;
		lastFilamentError = fstat;
	}
}

// Execute an emergency stop
void GCodes::DoEmergencyStop()
{
	reprap.EmergencyStop();
	Reset();
	platform.Message(GenericMessage, "Emergency Stop! Reset the controller to continue.");
}

// Pause the print. Before calling this, check that we are doing a file print that isn't already paused and get the movement lock.
void GCodes::DoPause(GCodeBuffer& gb, PauseReason reason, const char *msg)
{
	if (&gb == fileGCode)
	{
		// Pausing a file print because of a command in the file itself
		SavePosition(pauseRestorePoint, gb);
	}
	else
	{
		// Pausing a file print via another input source or for some other reason
		const bool movesSkipped = reprap.GetMove().PausePrint(pauseRestorePoint);		// tell Move we wish to pause the current print

		if (movesSkipped)
		{
			// The PausePrint call has filled in the restore point with machine coordinates
			ToolOffsetInverseTransform(pauseRestorePoint.moveCoords, currentUserPosition);	// transform the returned coordinates to user coordinates
			ClearMove();
		}
		else if (segmentsLeft != 0 && moveBuffer.canPauseBefore)
		{
			// We were not able to skip any moves, however we can skip the move that is waiting
			pauseRestorePoint.virtualExtruderPosition = moveBuffer.virtualExtruderPosition;
			pauseRestorePoint.filePos = moveBuffer.filePos;
			pauseRestorePoint.feedRate = moveBuffer.feedRate;
			ToolOffsetInverseTransform(pauseRestorePoint.moveCoords, currentUserPosition);	// transform the returned coordinates to user coordinates
			ClearMove();
		}
		else
		{
			// We were not able to skip any moves, and if there is a move waiting then we can't skip that one either
			pauseRestorePoint.feedRate = fileGCode->MachineState().feedrate;
			pauseRestorePoint.virtualExtruderPosition = virtualExtruderPosition;

			// TODO: when we use RTOS there is a possible race condition in the following,
			// because we might try to pause when a waiting move has just been added but before the gcode buffer has been re-initialised ready for the next command
			pauseRestorePoint.filePos = fileGCode->GetFilePosition(fileInput->BytesCached());
#if SUPPORT_IOBITS
			pauseRestorePoint.ioBits = moveBuffer.ioBits;
#endif
		}

		// Replace the paused machine coordinates by user coordinates, which we updated earlier
		for (size_t axis = 0; axis < numVisibleAxes; ++axis)
		{
			pauseRestorePoint.moveCoords[axis] = currentUserPosition[axis];
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
			platform.MessageF(GenericMessage, "Paused print, file offset=%" PRIu32 "\n", pauseRestorePoint.filePos);
		}
	}

	SaveFanSpeeds();
	pauseRestorePoint.toolNumber = reprap.GetCurrentToolNumber();

	if (simulationMode == 0)
	{
		SaveResumeInfo(false);															// create the resume file so that we can resume after power down
	}

	gb.SetState(GCodeState::pausing1);
	isPaused = true;

	if (msg != nullptr)
	{
		platform.SendAlert(GenericMessage, msg, "Printing paused", 1, 0.0, 0);
	}
}

bool GCodes::IsPaused() const
{
	return isPaused && !IsPausing() && !IsResuming();
}

bool GCodes::IsPausing() const
{
	GCodeState topState = fileGCode->OriginalMachineState().state;
	if (topState == GCodeState::pausing1 || topState == GCodeState::pausing2)
	{
		return true;
	}

	topState = daemonGCode->OriginalMachineState().state;
	if (topState == GCodeState::pausing1 || topState == GCodeState::pausing2)
	{
		return true;
	}

	topState = autoPauseGCode->OriginalMachineState().state;
	if (   topState == GCodeState::pausing1
		|| topState == GCodeState::pausing2
#if HAS_VOLTAGE_MONITOR
		|| topState == GCodeState::powerFailPausing1
#endif
	   )
	{
		return true;
	}

	return false;
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

// Return true if we are printing from SD card and not pausing, paused or resuming
// TODO make this independent of PrintMonitor
bool GCodes::IsReallyPrinting() const
{
	return reprap.GetPrintMonitor().IsPrinting() && IsRunning();
}

#if HAS_VOLTAGE_MONITOR || HAS_SMART_DRIVERS

// Do an emergency pause following loss of power or a motor stall returning true if successful, false if needs to be retried
bool GCodes::DoEmergencyPause()
{
	if (!autoPauseGCode->IsCompletelyIdle())
	{
		return false;							// we can't pause if the auto pause thread is busy already
	}

	// Save the resume info, stop movement immediately and run the low voltage pause script to lift the nozzle etc.
	GrabMovement(*autoPauseGCode);

	const bool movesSkipped = reprap.GetMove().LowPowerPause(pauseRestorePoint);
	if (movesSkipped)
	{
		// The PausePrint call has filled in the restore point with machine coordinates
		ToolOffsetInverseTransform(pauseRestorePoint.moveCoords, currentUserPosition);	// transform the returned coordinates to user coordinates
		ClearMove();
	}
	else if (segmentsLeft != 0 && moveBuffer.filePos != noFilePosition)
	{
		// We were not able to skip any moves, however we can skip the remaining segments of this current move
		ToolOffsetInverseTransform(moveBuffer.initialCoords, currentUserPosition);
		pauseRestorePoint.feedRate = moveBuffer.feedRate;
		pauseRestorePoint.virtualExtruderPosition = moveBuffer.virtualExtruderPosition;
		pauseRestorePoint.filePos = moveBuffer.filePos;
		pauseRestorePoint.proportionDone = (float)(totalSegments - segmentsLeft)/(float)totalSegments;
#if SUPPORT_IOBITS
		pauseRestorePoint.ioBits = moveBuffer.ioBits;
#endif
		ClearMove();
	}
	else
	{
		// We were not able to skip any moves, and if there is a move waiting then we can't skip that one either
		pauseRestorePoint.feedRate = fileGCode->MachineState().feedrate;
		pauseRestorePoint.virtualExtruderPosition = virtualExtruderPosition;

		// TODO: when we use RTOS there is a possible race condition in the following,
		// because we might try to pause when a waiting move has just been added but before the gcode buffer has been re-initialised ready for the next command
		pauseRestorePoint.filePos = fileGCode->GetFilePosition(fileInput->BytesCached());
		pauseRestorePoint.proportionDone = 0.0;
#if SUPPORT_IOBITS
		pauseRestorePoint.ioBits = moveBuffer.ioBits;
#endif
	}

	// Replace the paused machine coordinates by user coordinates, which we updated earlier
	for (size_t axis = 0; axis < numVisibleAxes; ++axis)
	{
		pauseRestorePoint.moveCoords[axis] = currentUserPosition[axis];
	}

	SaveFanSpeeds();
	pauseRestorePoint.toolNumber = reprap.GetCurrentToolNumber();
	isPaused = true;

	return true;
}

#endif

#if HAS_VOLTAGE_MONITOR

// Try to pause the current SD card print, returning true if successful, false if needs to be called again
bool GCodes::LowVoltagePause()
{
	if (simulationMode != 0)
	{
		return true;								// ignore the low voltage indication
	}

	reprap.GetHeat().SuspendHeaters(true);			// turn the heaters off to conserve power for the motors to execute the pause
	if (IsResuming())
	{
		// This is an unlucky situation, because the resume macro is probably being run, which will probably lower the head back on to the print.
		// It may well be that the power loss will prevent the resume macro being completed. If not, try again when the print has been resumed.
		return false;
	}

	if (IsPausing())
	{
		// We are in the process of pausing already, so the resume info has already been saved.
		// With luck the retraction and lifting of the head in the pause.g file has been done already.
		return true;
	}

	if (IsPaused())
	{
		// Resume info has already been saved, and resuming will be prevented while the power is low
		return true;
	}

	if (reprap.GetPrintMonitor().IsPrinting())
	{
		if (!DoEmergencyPause())
		{
			return false;
		}

		// Run the auto-pause script
		if (powerFailScript != nullptr)
		{
			autoPauseGCode->Put(powerFailScript);
		}
		autoPauseGCode->SetState(GCodeState::powerFailPausing1);
		isPowerFailPaused = true;

		// Don't do any more here, we want the auto pause thread to run as soon as possible
	}

	return true;
}

// Resume printing, normally only ever called after it has been paused because if low voltage.
// If the pause was short enough, resume automatically.
bool GCodes::LowVoltageResume()
{
	reprap.GetHeat().SuspendHeaters(false);			// turn the heaters on again
	if (isPaused && isPowerFailPaused)
	{
		isPowerFailPaused = false;					// pretend it's a normal pause
		// Run resurrect.g automatically
		//TODO qq;
		//platform.Message(LoggedGenericMessage, "Print auto-resumed\n");
	}
	return true;
}

#endif

#if HAS_SMART_DRIVERS

// Pause the print because the specified driver has reported a stall
bool GCodes::PauseOnStall(DriversBitmap stalledDrivers)
{
	if (!IsReallyPrinting())
	{
		return true;								// if not printing, acknowledge it but take no action
	}
	if (!autoPauseGCode->IsCompletelyIdle())
	{
		return false;								// can't handle it yet
	}
	if (!LockMovement(*autoPauseGCode))
	{
		return false;
	}

	String<100> stallErrorString;
	stallErrorString.GetRef().printf("Stall detected on driver(s)");
	ListDrivers(stallErrorString.GetRef(), stalledDrivers);
	DoPause(*autoPauseGCode, PauseReason::stall, stallErrorString.Pointer());
	platform.Message(LogMessage, stallErrorString.c_str());
	return true;
}

// Re-home and resume the print because the specified driver has reported a stall
bool GCodes::ReHomeOnStall(DriversBitmap stalledDrivers)
{
	if (!IsReallyPrinting())
	{
		return true;								// if not printing, acknowledge it but take no action
	}
	if (!DoEmergencyPause())
	{
		return false;								// can't handle it yet
	}

	autoPauseGCode->SetState(GCodeState::resuming1); // set up to resume after rehoming
	DoFileMacro(*autoPauseGCode, REHOME_G, true);	// run the SD card rehome-and-resume script
	return true;
}

#endif

void GCodes::SaveResumeInfo(bool wasPowerFailure)
{
	const char* const printingFilename = reprap.GetPrintMonitor().GetPrintingFilename();
	if (printingFilename != nullptr)
	{
		FileStore * const f = platform.OpenFile(platform.GetSysDir(), RESUME_AFTER_POWER_FAIL_G, OpenMode::write);
		if (f == nullptr)
		{
			platform.MessageF(ErrorMessage, "Failed to create file %s", RESUME_AFTER_POWER_FAIL_G);
		}
		else
		{
			String<200> bufferSpace;
			const StringRef buf = bufferSpace.GetRef();

			// Write the header comment
			buf.printf("; File \"%s\" resume print after %s", printingFilename, (wasPowerFailure) ? "power failure" : "print paused");
			if (platform.IsDateTimeSet())
			{
				time_t timeNow = platform.GetDateTime();
				const struct tm * const timeInfo = gmtime(&timeNow);
				buf.catf(" at %04u-%02u-%02u %02u:%02u",
								timeInfo->tm_year + 1900, timeInfo->tm_mon + 1, timeInfo->tm_mday, timeInfo->tm_hour, timeInfo->tm_min);
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
				buf.printf("M106 S%.2f\n", (double)lastDefaultFanSpeed);
				ok = f->Write(buf.Pointer());
			}
			if (ok)
			{
				buf.printf("M116\nM290 S%.3f\n", (double)currentBabyStepZOffset);
				ok = f->Write(buf.Pointer());								// write baby stepping offset
			}
			if (ok && fileGCode->OriginalMachineState().volumetricExtrusion)
			{
				buf.copy("M200 ");
				char c = 'D';
				for (size_t i = 0; i < numExtruders; ++i)
				{
					buf.catf("%c%.03f", c, (double)volumetricExtrusionFactors[i]);
					c = ':';
				}
				buf.cat('\n');
				ok = f->Write(buf.Pointer());								// write volumetric extrusion factors
			}
			if (ok)
			{
				buf.printf("G92 E%.5f\n%s\n", (double)virtualExtruderPosition, (fileGCode->OriginalMachineState().drivesRelative) ? "M83" : "M82");
				ok = f->Write(buf.Pointer());								// write virtual extruder position and absolute/relative extrusion flag
			}
			if (ok)
			{
				buf.printf("M23 %s\nM26 S%" PRIu32 " P%.3f\n", printingFilename, pauseRestorePoint.filePos, (double)pauseRestorePoint.proportionDone);
				ok = f->Write(buf.Pointer());								// write filename and file position
			}
			if (ok)
			{
				// Build the commands to restore the head position. These assume that we are working in mm.
				// Start with a vertical move to 2mm above the final Z position
				buf.printf("G0 F6000 Z%.3f\n", (double)(pauseRestorePoint.moveCoords[Z_AXIS] + 2.0));

				// Now set all the other axes
				buf.cat("G0 F6000");
				for (size_t axis = 0; axis < numVisibleAxes; ++axis)
				{
					if (axis != Z_AXIS)
					{
						buf.catf(" %c%.2f", axisLetters[axis], (double)pauseRestorePoint.moveCoords[axis]);
					}
				}

				// Now move down to the correct Z height
				buf.catf("\nG0 F6000 Z%.3f\n", (double)pauseRestorePoint.moveCoords[Z_AXIS]);

				// Set the feed rate
				buf.catf("G1 F%.1f", (double)(pauseRestorePoint.feedRate * MinutesToSeconds));
#if SUPPORT_IOBITS
				buf.catf(" P%u", (unsigned int)pauseRestorePoint.ioBits);
#endif
				buf.cat("\nM24\n");
				ok = f->Write(buf.Pointer());								// restore feed rate and output bits
			}
			if (!f->Close())
			{
				ok = false;
			}
			if (ok)
			{
				platform.Message(LoggedGenericMessage, "Resume-after-power-fail state saved\n");
			}
			else
			{
				platform.GetMassStorage()->Delete(platform.GetSysDir(), RESUME_AFTER_POWER_FAIL_G, true);
				platform.MessageF(ErrorMessage, "Failed to write or close file %s\n", RESUME_AFTER_POWER_FAIL_G);
			}
		}
	}
}

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
		platform.Message(ErrorMessage, "Push(): stack overflow!\n");
	}
	return ok;
}

// Recover a saved state
void GCodes::Pop(GCodeBuffer& gb)
{
	if (!gb.PopState())
	{
		platform.Message(ErrorMessage, "Pop(): stack underflow!\n");
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

			if (mc == 1)
			{
				// There may be multiple extruders present but only one value has been specified, so use mixing
				const float moveArg = eMovement[0] * distanceScale;
				float requestedExtrusionAmount;
				if (gb.MachineState().drivesRelative)
				{
					requestedExtrusionAmount = moveArg;
				}
				else
				{
					requestedExtrusionAmount = moveArg - virtualExtruderPosition;
					virtualExtruderPosition = moveArg;
				}

				for (size_t eDrive = 0; eDrive < eMoveCount; eDrive++)
				{
					const int drive = tool->Drive(eDrive);
					float extrusionAmount = requestedExtrusionAmount * tool->GetMix()[eDrive];
					if (extrusionAmount != 0.0)
					{
						if (gb.MachineState().volumetricExtrusion)
						{
							extrusionAmount *= volumetricExtrusionFactors[drive];
						}
						rawExtruderTotalByDrive[drive] += extrusionAmount;
						if (!doingToolChange)			// don't count extrusion done in tool change macros towards total filament consumed, it distorts the print progress
						{
							rawExtruderTotal += extrusionAmount;
						}
						moveBuffer.coords[drive + numTotalAxes] = extrusionAmount * extrusionFactors[drive];
#if HAS_SMART_DRIVERS
						if (moveBuffer.moveType == 1)
						{
							SetBit(moveBuffer.endStopsToCheck, drive + numTotalAxes);
						}
#endif
					}
				}
			}
			else
			{
				// Individual extrusion amounts have been provided. This is supported in relative extrusion mode only.
				if (gb.MachineState().drivesRelative)
				{
					for (size_t eDrive = 0; eDrive < eMoveCount; eDrive++)
					{
						const int drive = tool->Drive(eDrive);
						float extrusionAmount = eMovement[eDrive] * distanceScale;
						if (extrusionAmount != 0.0)
						{
							if (gb.MachineState().volumetricExtrusion)
							{
								extrusionAmount *= volumetricExtrusionFactors[drive];
							}
							rawExtruderTotalByDrive[drive] += extrusionAmount;
							if (!doingToolChange)		// don't count extrusion done in tool change macros towards total filament consumed, it distorts the print progress
							{
								rawExtruderTotal += extrusionAmount;
							}
							moveBuffer.coords[drive + numTotalAxes] = extrusionAmount * extrusionFactors[drive] * volumetricExtrusionFactors[drive];
#if HAS_SMART_DRIVERS
							if (moveBuffer.moveType == 1)
							{
								SetBit(moveBuffer.endStopsToCheck, drive + numTotalAxes);
							}
#endif
						}
					}
				}
				else
				{
					platform.Message(ErrorMessage, "Multiple E parameters in G1 commands are not supported in absolute extrusion mode\n");
				}
			}
		}
	}
	return true;
}

// Execute a straight move returning true if an error was written to 'reply'
// We have already acquired the movement lock and waited for the previous move to be taken.
bool GCodes::DoStraightMove(GCodeBuffer& gb, const StringRef& reply, bool isCoordinated)
{
	// Set up default move parameters
	moveBuffer.isCoordinated = isCoordinated;
	moveBuffer.endStopsToCheck = 0;
	moveBuffer.moveType = 0;
	moveBuffer.xAxes = reprap.GetCurrentXAxes();
	moveBuffer.yAxes = reprap.GetCurrentYAxes();
	moveBuffer.usePressureAdvance = false;
	axesToSenseLength = 0;

	// Check to see if the move is a 'homing' move that endstops are checked on.
	// We handle S1 parameters affecting extrusion elsewhere.
	if (gb.Seen('S'))
	{
		const int ival = gb.GetIValue();
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
		// Regular move. If it's a delta or SCARA printer, the XYZ axes must be homed first.
		constexpr AxesBitmap xyzAxes = LowestNBits<AxesBitmap>(Z_AXIS);
		if ((axesHomed & xyzAxes) != xyzAxes && (kinType == KinematicsType::linearDelta || kinType == KinematicsType::scara) && simulationMode == 0)
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

	if (moveBuffer.moveType != 0)
	{
		// This may be a raw motor move, in which case we need the current raw motor positions in moveBuffer.coords.
		// If it isn't a raw motor move, it will still be applied without axis or bed transform applies,
		// so make sure the initial coordinates don't have those either to avoid unwanted Z movement.
		reprap.GetMove().GetCurrentUserPosition(moveBuffer.coords, moveBuffer.moveType, reprap.GetCurrentXAxes(), reprap.GetCurrentYAxes());
	}

	// Set up the initial coordinates
	memcpy(moveBuffer.initialCoords, moveBuffer.coords, numVisibleAxes * sizeof(moveBuffer.initialCoords[0]));

	// Deal with XYZ movement
	const float initialX = currentUserPosition[X_AXIS];
	const float initialY = currentUserPosition[Y_AXIS];
	AxesBitmap axesMentioned = 0;
	for (size_t axis = 0; axis < numVisibleAxes; axis++)
	{
		if (gb.Seen(axisLetters[axis]))
		{
			SetBit(axesMentioned, axis);
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
		// If a restore point is being used (G1 R parameter) then we used to set any coordinates that were not mentioned to the restore point values.
		// But that causes issues for tool change on IDEX machines because we end up restoring the U axis when we shouldn't.
		// So we no longer do that, and the user must mention any axes that he wants restored e.g. G1 R2 X0 Y0.
	}

	// Deal with extrusion and feed rate
	LoadExtrusionAndFeedrateFromGCode(gb, moveBuffer.moveType);

	// Set up the move. We must assign segmentsLeft last, so that when we move to RTOS, the move won't be picked up by the Move process before it is complete.
	// Note that if this is an extruder-only move, we don't do axis movements to allow for tool offset changes, we defer those until an axis moves.
	if (moveBuffer.moveType != 0)
	{
		// It's a raw motor move, so do it in a single segment and wait for it to complete
		totalSegments = 1;
		gb.SetState(GCodeState::waitingForSpecialMoveToComplete);
	}
	else if (axesMentioned == 0)
	{
		totalSegments = 1;
	}
	else
	{
		ToolOffsetTransform(currentUserPosition, moveBuffer.coords, axesMentioned);	// apply tool offset, axis mapping, baby stepping, Z hop and axis scaling
		AxesBitmap effectiveAxesHomed = axesHomed;
		if (doingManualBedProbe)
		{
			ClearBit(effectiveAxesHomed, Z_AXIS);								// if doing a manual Z probe, don't limit the Z movement
		}
		if (limitAxes && reprap.GetMove().GetKinematics().LimitPosition(moveBuffer.coords, numVisibleAxes, effectiveAxesHomed, moveBuffer.isCoordinated))
		{
			ToolOffsetInverseTransform(moveBuffer.coords, currentUserPosition);	// make sure the limits are reflected in the user position
		}

		// Flag whether we should use pressure advance, if there is any extrusion in this move.
		// We assume it is a normal printing move needing pressure advance if there is forward extrusion and XYU.. movement.
		// The movement code will only apply pressure advance if there is forward extrusion, so we only need to check for XYU.. movement here.
		{
			AxesBitmap axesMentionedExceptZ = axesMentioned;
			ClearBit(axesMentionedExceptZ, Z_AXIS);
			moveBuffer.usePressureAdvance = moveBuffer.hasExtrusion && (axesMentionedExceptZ != 0);
		}

		// Apply segmentation if necessary. To speed up simulation on SCARA printers, we don't apply kinematics segmentation when simulating.
		// Note for when we use RTOS: as soon as we set segmentsLeft nonzero, the Move process will assume that the move is ready to take, so this must be the last thing we do.
		const Kinematics& kin = reprap.GetMove().GetKinematics();
		if (kin.UseSegmentation() && simulationMode != 1 && (moveBuffer.hasExtrusion || isCoordinated || !kin.UseRawG0()))
		{
			// This kinematics approximates linear motion by means of segmentation.
			// We assume that the segments will be smaller than the mesh spacing.
			const float xyLength = sqrtf(fsquare(currentUserPosition[X_AXIS] - initialX) + fsquare(currentUserPosition[Y_AXIS] - initialY));
			const float moveTime = xyLength/moveBuffer.feedRate;			// this is a best-case time, often the move will take longer
			totalSegments = (unsigned int)max<int>(1, min<int>(rintf(xyLength/kin.GetMinSegmentLength()), rintf(moveTime * kin.GetSegmentsPerSecond())));
		}
		else if (reprap.GetMove().IsUsingMesh())
		{
			const HeightMap& heightMap = reprap.GetMove().AccessHeightMap();
			totalSegments = max<unsigned int>(1, heightMap.GetMinimumSegments(currentUserPosition[X_AXIS] - initialX, currentUserPosition[Y_AXIS] - initialY));
		}
		else
		{
			totalSegments = 1;
		}
	}

	doingArcMove = false;
	FinaliseMove(gb);
	return false;
}

// Execute an arc move, returning true if it was badly-formed
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
	if (limitAxes && reprap.GetMove().GetKinematics().LimitPosition(moveBuffer.coords, numVisibleAxes, axesHomed, true))
	{
		ToolOffsetInverseTransform(moveBuffer.coords, currentUserPosition);	// make sure the limits are reflected in the user position
	}

	// Compute the angle at which we stop
	const float finalTheta = atan2(currentUserPosition[Y_AXIS] - userArcCentreY, currentUserPosition[X_AXIS] - userArcCentreX);

	// Set up default move parameters
	moveBuffer.endStopsToCheck = 0;
	moveBuffer.moveType = 0;
	moveBuffer.xAxes = reprap.GetCurrentXAxes();
	moveBuffer.yAxes = reprap.GetCurrentYAxes();
	moveBuffer.isCoordinated = true;

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
	moveBuffer.usePressureAdvance = moveBuffer.hasExtrusion;

	arcRadius = sqrtf(iParam * iParam + jParam * jParam);
	arcCurrentAngle = atan2(-jParam, -iParam);

	// Calculate the total angle moved, which depends on which way round we are going
	float totalArc = (clockwise) ? arcCurrentAngle - finalTheta : finalTheta - arcCurrentAngle;
	if (totalArc < 0)
	{
		totalArc += 2 * PI;
	}

	// Compute how many segments we need to move, but don't store it yet
	totalSegments = max<unsigned int>((unsigned int)((arcRadius * totalArc)/arcSegmentLength + 0.8), 1u);
	arcAngleIncrement = totalArc/totalSegments;
	if (clockwise)
	{
		arcAngleIncrement = -arcAngleIncrement;
	}

	doingArcMove = true;
	FinaliseMove(gb);
//	debugPrintf("Radius %.2f, initial angle %.1f, increment %.1f, segments %u\n",
//				arcRadius, arcCurrentAngle * RadiansToDegrees, arcAngleIncrement * RadiansToDegrees, segmentsLeft);
	return false;
}

// Adjust the move parameters to account for segmentation and/or  part of the move having been done already
void GCodes::FinaliseMove(const GCodeBuffer& gb)
{
	moveBuffer.canPauseAfter = (moveBuffer.endStopsToCheck == 0);
	moveBuffer.canPauseBefore = true;
	moveBuffer.filePos = (&gb == fileGCode) ? gb.GetFilePosition(fileInput->BytesCached()) : noFilePosition;
	moveBuffer.virtualExtruderPosition = virtualExtruderPosition;

	if (totalSegments > 1)
	{
		for (size_t drive = numTotalAxes; drive < DRIVES; ++drive)
		{
			moveBuffer.coords[drive] /= totalSegments;							// change the extrusion to extrusion per segment
		}

		if (moveFractionToSkip != 0.0)
		{
			const float fseg = floor(totalSegments * moveFractionToSkip);		// round down to the start of a move
			segmentsLeftToStartAt = totalSegments - (unsigned int)fseg;
			firstSegmentFractionToSkip = (moveFractionToSkip * totalSegments) - fseg;
			segmentsLeft = totalSegments;										// do this last, ready for RTOS
			return;
		}
	}

	segmentsLeftToStartAt = totalSegments;
	firstSegmentFractionToSkip = moveFractionToSkip;

	segmentsLeft = totalSegments;												// do this last, ready for RTOS
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
		if (segmentsLeftToStartAt == 1 && firstSegmentFractionToSkip != 0.0)	// if this is the segment we are starting at and we need to skip some of it
		{
			// Reduce the extrusion by the amount to be skipped
			for (size_t drive = numTotalAxes; drive < DRIVES; ++drive)
			{
				m.coords[drive] *= (1.0 - firstSegmentFractionToSkip);
			}
		}
		m.proportionLeft = 0.0;
		ClearMove();
	}
	else
	{
		// This move needs to be divided into 2 or more segments
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

		if (segmentsLeftToStartAt < segmentsLeft)
		{
			// We are resuming a print part way through a move and we printed this segment already
			--segmentsLeft;
			return false;
		}
		if (segmentsLeftToStartAt == segmentsLeft && firstSegmentFractionToSkip != 0.0)	// if this is the segment we are starting at and we need to skip some of it
		{
			// Reduce the extrusion by the amount to be skipped
			for (size_t drive = numTotalAxes; drive < DRIVES; ++drive)
			{
				m.coords[drive] *= (1.0 - firstSegmentFractionToSkip);
			}
		}
		--segmentsLeft;

		m.proportionLeft = (float)segmentsLeft/(float)totalSegments;
	}

	return true;
}

void GCodes::ClearMove()
{
	segmentsLeft = 0;
	doingArcMove = false;
	moveBuffer.endStopsToCheck = 0;
	moveBuffer.moveType = 0;
	moveBuffer.isFirmwareRetraction = false;
	moveFractionToSkip = 0.0;
}

// Run a file macro. Prior to calling this, 'state' must be set to the state we want to enter when the macro has been completed.
// Return true if the file was found or it wasn't and we were asked to report that fact.
bool GCodes::DoFileMacro(GCodeBuffer& gb, const char* fileName, bool reportMissing, int codeRunning)
{
	FileStore * const f = platform.OpenFile(platform.GetSysDir(), fileName, OpenMode::read);
	if (f == nullptr)
	{
		if (reportMissing)
		{
			// Don't use snprintf into scratchString here, because fileName may be aliased to scratchString
			platform.MessageF(WarningMessage, "Macro file %s not found.\n", fileName);
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
	gb.MachineState().runningM501 = (codeRunning == 501);
	gb.MachineState().runningM502 = (codeRunning == 502);
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

// Home one or more of the axes
// 'reply' is only written if there is an error.
GCodeResult GCodes::DoHome(GCodeBuffer& gb, const StringRef& reply)
{
	if (!LockMovementAndWaitForStandstill(gb))
	{
		return GCodeResult::notFinished;
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
	return GCodeResult::ok;
}

// This is called to execute a G30.
// It sets wherever we are as the probe point P (probePointIndex) then probes the bed, or gets all its parameters from the arguments.
// If X or Y are specified, use those; otherwise use the machine's coordinates.  If no Z is specified use the machine's coordinates.
// If it is specified and is greater than SILLY_Z_VALUE (i.e. greater than -9999.0) then that value is used.
// If it's less than SILLY_Z_VALUE the bed is probed and that value is used.
// We already own the movement lock before this is called.
GCodeResult GCodes::ExecuteG30(GCodeBuffer& gb, const StringRef& reply)
{
	g30SValue = (gb.Seen('S')) ? gb.GetIValue() : -3;		// S-3 is equivalent to having no S parameter
	g30ProbePointIndex = -1;
	bool seenP = false;
	gb.TryGetIValue('P', g30ProbePointIndex, seenP);
	if (seenP)
	{
		if (g30ProbePointIndex < 0 || g30ProbePointIndex >= (int)MaxProbePoints)
		{
			reply.copy("Z probe point index out of range");
			return GCodeResult::error;
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
				if (g30SValue >= -1)
				{
					return GetGCodeResultFromError(reprap.GetMove().FinishedBedProbing(g30SValue, reply));
				}
			}
			else
			{
				// Do a Z probe at the specified point.
				gb.SetState(GCodeState::probingAtPoint0);
				if (platform.GetZProbeType() != ZProbeType::none && platform.GetZProbeType() != ZProbeType::blTouch && !probeIsDeployed)
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
		gb.SetState(GCodeState::probingAtPoint2a);
		if (platform.GetZProbeType() != ZProbeType::none && !probeIsDeployed)
		{
			DoFileMacro(gb, DEPLOYPROBE_G, false);
		}
	}
	return GCodeResult::ok;
}

// Decide which device to display a message box on
MessageType GCodes::GetMessageBoxDevice(GCodeBuffer& gb) const
{
	MessageType mt = gb.GetResponseMessageType();
	if (mt == GenericMessage)
	{
		// Command source was the file being printed, or a trigger. Send the message to PanelDue if there is one, else to the web server.
		mt = (lastAuxStatusReportType >= 0) ? LcdMessage : HttpMessage;
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
		platform.SendAlert(mt, "Adjust height until the nozzle just touches the bed, then press OK", "Manual bed probing", 2, 0.0, MakeBitmap<AxesBitmap>(Z_AXIS));
	}
}

// Start probing the grid, returning true if we didn't because of an error.
// Prior to calling this the movement system must be locked.
GCodeResult GCodes::ProbeGrid(GCodeBuffer& gb, const StringRef& reply)
{
	if (!defaultGrid.IsValid())
	{
		reply.copy("No valid grid defined for bed probing");
		return GCodeResult::error;
	}

	if (!AllAxesAreHomed())
	{
		reply.copy("Must home printer before bed probing");
		return GCodeResult::error;
	}

	Move& move = reprap.GetMove();
	move.AccessHeightMap().SetGrid(defaultGrid);
	move.AccessHeightMap().ClearGridHeights();
	move.SetIdentityTransform();
	gridXindex = gridYindex = 0;
	gb.SetState(GCodeState::gridProbing1);

	if (platform.GetZProbeType() != ZProbeType::none && platform.GetZProbeType() != ZProbeType::blTouch && !probeIsDeployed)
	{
		DoFileMacro(gb, DEPLOYPROBE_G, false);
	}
	return GCodeResult::ok;
}

bool GCodes::LoadHeightMap(GCodeBuffer& gb, const StringRef& reply) const
{
	reprap.GetMove().SetIdentityTransform();					// stop using old-style bed compensation and clear the height map

	String<MaxFilenameLength> heightMapFileName;
	bool seen = false;
	gb.TryGetQuotedString('P', heightMapFileName.GetRef(), seen);
	if (!seen)
	{
		heightMapFileName.copy(DefaultHeightMapFile);
	}

	FileStore * const f = platform.OpenFile(platform.GetSysDir(), heightMapFileName.c_str(), OpenMode::read);
	if (f == nullptr)
	{
		reply.printf("Height map file %s not found", heightMapFileName.c_str());
		return true;
	}

	reply.printf("Failed to load height map from file %s: ", heightMapFileName.c_str());	// set up error message to append to
	HeightMap& heightMap = reprap.GetMove().AccessHeightMap();
	const bool err = heightMap.LoadFromFile(f, reply);
	f->Close();
	if (err)
	{
		heightMap.ClearGridHeights();							// make sure we don't end up with a partial height map
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
bool GCodes::SaveHeightMap(GCodeBuffer& gb, const StringRef& reply) const
{
	String<MaxFilenameLength> heightMapFileName;
	bool seen = false;
	gb.TryGetQuotedString('P', heightMapFileName.GetRef(), seen);
	if (!seen)
	{
		heightMapFileName.copy(DefaultHeightMapFile);
	}

	FileStore * const f = platform.OpenFile(platform.GetSysDir(), heightMapFileName.c_str(), OpenMode::write);
	bool err;
	if (f == nullptr)
	{
		reply.catf("Failed to create height map file %s", heightMapFileName.c_str());
		err = true;
	}
	else
	{
		err = reprap.GetMove().AccessHeightMap().SaveToFile(f);
		f->Close();
		if (err)
		{
			platform.GetMassStorage()->Delete(platform.GetSysDir(), heightMapFileName.c_str());
			reply.catf("Failed to save height map to file %s", heightMapFileName.c_str());
		}
		else
		{
			reply.catf("Height map saved to file %s", heightMapFileName.c_str());
		}
	}
	return err;
}

// Return the current coordinates as a printable string.
// Coordinates are updated at the end of each movement, so this won't tell you where you are mid-movement.
void GCodes::GetCurrentCoordinates(const StringRef& s) const
{
	float liveCoordinates[DRIVES];
	reprap.GetMove().LiveCoordinates(liveCoordinates, reprap.GetCurrentXAxes(), reprap.GetCurrentYAxes());
	const Tool * const currentTool = reprap.GetCurrentTool();
	if (currentTool != nullptr)
	{
		for (size_t i = 0; i < numVisibleAxes; ++i)
		{
			liveCoordinates[i] += currentTool->GetOffset(i);
		}
	}

	s.Clear();
	for (size_t axis = 0; axis < numVisibleAxes; ++axis)
	{
		// Don't put a space after the colon in the response, it confuses Pronterface
		s.catf("%c:%.3f ", axisLetters[axis], (double)liveCoordinates[axis]);
	}
	for (size_t i = numTotalAxes; i < DRIVES; i++)
	{
		s.catf("E%u:%.1f ", i - numTotalAxes, (double)liveCoordinates[i]);
	}

	// Print the axis stepper motor positions as Marlin does, as an aid to debugging.
	// Don't bother with the extruder endpoints, they are zero after any non-extruding move.
	s.cat(" Count");
	for (size_t i = 0; i < numVisibleAxes; ++i)
	{
		s.catf(" %" PRIi32, reprap.GetMove().GetEndPoint(i));
	}

	// Add the user coordinates because they may be different from the machine coordinates under some conditions
	s.cat(" User");
	for (size_t i = 0; i < numVisibleAxes; ++i)
	{
		s.catf(" %.1f", (double)currentUserPosition[i]);
	}
}

bool GCodes::OpenFileToWrite(GCodeBuffer& gb, const char* directory, const char* fileName, const FilePosition size, const bool binaryWrite, const uint32_t fileCRC32)
{
	fileBeingWritten = platform.OpenFile(directory, fileName, OpenMode::write);
	eofStringCounter = 0;
	fileSize = size;
	if (fileBeingWritten == nullptr)
	{
		platform.MessageF(ErrorMessage, "Failed to open GCode file \"%s\" for writing.\n", fileName);
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
		platform.Message(ErrorMessage, "Attempt to write to a null file.\n");
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
		platform.Message(ErrorMessage, "Attempt to write to a null file.\n");
		return;
	}

	if (gb.GetCommandLetter() == 'M')
	{
		if (gb.GetCommandNumber() == 29)						// end of file?
		{
			fileBeingWritten->Close();
			fileBeingWritten = nullptr;
			gb.SetWritingFileDirectory(nullptr);
			const char* r = (platform.Emulating() == marlin) ? "Done saving file." : "";
			HandleReply(gb, false, r);
			return;
		}
	}
	else if (gb.GetCommandLetter() == 'G' && gb.GetCommandNumber() == 998)						// resend request?
	{
		if (gb.Seen('P'))
		{
			scratchString.printf("%" PRIi32 "\n", gb.GetIValue());
			HandleReply(gb, false, scratchString.Pointer());
			return;
		}
	}

	fileBeingWritten->Write(gb.Buffer());
	fileBeingWritten->Write('\n');
	HandleReply(gb, false, "");
}

// Set up a file to print, but don't print it yet.
// If successful return true, else write an error message to reply and return false
bool GCodes::QueueFileToPrint(const char* fileName, const StringRef& reply)
{
	FileStore * const f = platform.OpenFile(platform.GetGCodeDir(), fileName, OpenMode::read);
	if (f != nullptr)
	{
		fileGCode->SetToolNumberAdjust(0);								// clear tool number adjustment
		fileGCode->MachineState().volumetricExtrusion = false;			// default to non-volumetric extrusion

		// Reset all extruder positions when starting a new print
		virtualExtruderPosition = 0.0;
		for (size_t extruder = 0; extruder < MaxExtruders; extruder++)
		{
			rawExtruderTotalByDrive[extruder] = 0.0;
		}
		rawExtruderTotal = 0.0;
		reprap.GetMove().ResetExtruderPositions();

		fileToPrint.Set(f);
		fileOffsetToPrint = 0;
		moveFractionToStartAt = 0.0;
		return true;
	}

	reply.printf("GCode file \"%s\" not found\n", fileName);
	return false;
}

// Start printing the file already selected
void GCodes::StartPrinting(bool fromStart)
{
	fileGCode->OriginalMachineState().fileState.MoveFrom(fileToPrint);
	fileInput->Reset();
	lastFilamentError = FilamentSensorStatus::ok;
	reprap.GetPrintMonitor().StartedPrint();
	platform.MessageF(LogMessage,
						(simulationMode == 0) ? "Started printing file %s\n" : "Started simulating printing file %s\n",
							reprap.GetPrintMonitor().GetPrintingFilename());
	if (fromStart)
	{
		// Get the fileGCode to execute the start macro so that any M82/M83 codes will be executed in the correct context
		DoFileMacro(*fileGCode, START_G, false);
	}
}

// Function to handle dwell delays. Returns true for dwell finished, false otherwise.
GCodeResult GCodes::DoDwell(GCodeBuffer& gb)
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
		return GCodeResult::ok;  // No time given - throw it away
	}

	if (dwell <= 0)
	{
		return GCodeResult::ok;
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
		return GCodeResult::notFinished;
	}

	if (simulationMode != 0)
	{
		simulationTime += (float)dwell * 0.001;
		return GCodeResult::ok;
	}

	return DoDwellTime(gb, (uint32_t)dwell);
}

GCodeResult GCodes::DoDwellTime(GCodeBuffer& gb, uint32_t dwellMillis)
{
	const uint32_t now = millis();

	// Are we already in the dwell?
	if (gb.timerRunning)
	{
		if (now - gb.whenTimerStarted >= dwellMillis)
		{
			gb.timerRunning = false;
			return GCodeResult::ok;
		}
		return GCodeResult::notFinished;
	}

	// New dwell - set it up
	gb.whenTimerStarted = now;
	gb.timerRunning = true;
	return GCodeResult::notFinished;
}

// Set offset, working and standby temperatures for a tool. I.e. handle a G10.
GCodeResult GCodes::SetOrReportOffsets(GCodeBuffer &gb, const StringRef& reply)
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
			return GCodeResult::error;
		}
	}
	else
	{
		tool = reprap.GetCurrentTool();
		if (tool == nullptr)
		{
			reply.printf("Attempt to set/report offsets and temperatures for no selected tool");
			return GCodeResult::error;
		}
	}

	bool settingOffset = false;
	for (size_t axis = 0; axis < numVisibleAxes; ++axis)
	{
		if (gb.Seen(axisLetters[axis]))
		{
			if (!LockMovement(gb))
			{
				return GCodeResult::notFinished;
			}
			settingOffset = true;
			tool->SetOffset(axis, gb.GetFValue(), gb.MachineState().runningM501);
		}
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
			reply.catf(" %c%.2f", axisLetters[axis], (double)tool->GetOffset(axis));
		}
		if (hCount != 0)
		{
			reply.cat(", active/standby temperature(s):");
			for (size_t heater = 0; heater < hCount; heater++)
			{
				reply.catf(" %.1f/%.1f", (double)active[heater], (double)standby[heater]);
			}
		}
	}
	return GCodeResult::ok;
}

// Create a new tool definition, returning true if an error was reported
bool GCodes::ManageTool(GCodeBuffer& gb, const StringRef& reply)
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
	String<ToolNameLength> name;
	if (gb.Seen('S'))
	{
		if (!gb.GetQuotedString(name.GetRef()))
		{
			reply.copy("Invalid tool name");
			return true;
		}
		seen = true;
	}

	// Check drives
	int32_t drives[MaxExtruders]; 	 		// There can never be more than we have...
	size_t dCount = numExtruders;			// Sets the limit and returns the count
	if (gb.Seen('D'))
	{
		gb.GetIntArray(drives, dCount, false);
		seen = true;
	}
	else
	{
		dCount = 0;
	}

	// Check heaters
	int32_t heaters[Heaters];
	size_t hCount = Heaters;
	if (gb.Seen('H'))
	{
		gb.GetIntArray(heaters, hCount, false);
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
		uint32_t xMapping[MaxAxes];
		size_t xCount = numVisibleAxes;
		gb.GetUnsignedArray(xMapping, xCount, false);
		xMap = UnsignedArrayToBitMap<AxesBitmap>(xMapping, xCount) & LowestNBits<AxesBitmap>(numVisibleAxes);
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
		uint32_t yMapping[MaxAxes];
		size_t yCount = numVisibleAxes;
		gb.GetUnsignedArray(yMapping, yCount, false);
		yMap = UnsignedArrayToBitMap<AxesBitmap>(yMapping, yCount) & LowestNBits<AxesBitmap>(numVisibleAxes);
		seen = true;
	}
	else
	{
		yMap = DefaultYAxisMapping;					// by default map X axis straight through
	}

	if ((xMap & yMap) != 0)
	{
		reply.copy("Cannot map both X and Y to the same axis");
		return true;
	}

	// Check for fan mapping
	FansBitmap fanMap;
	if (gb.Seen('F'))
	{
		uint32_t fanMapping[NUM_FANS];
		size_t fanCount = NUM_FANS;
		gb.GetUnsignedArray(fanMapping, fanCount, false);
		fanMap = UnsignedArrayToBitMap<FansBitmap>(fanMapping, fanCount) & LowestNBits<FansBitmap>(NUM_FANS);
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
			Tool* const tool = Tool::Create(toolNumber, name.c_str(), drives, dCount, heaters, hCount, xMap, yMap, fanMap, reply);
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

bool GCodes::ChangeMicrostepping(size_t drive, unsigned int microsteps, int mode) const
{
	bool dummy;
	const unsigned int oldSteps = platform.GetMicrostepping(drive, mode, dummy);
	const bool success = platform.SetMicrostepping(drive, microsteps, mode);
	if (success && mode <= 1)							// modes higher than 1 are used for special functions
	{
		// We changed the microstepping, so adjust the steps/mm to compensate
		platform.SetDriveStepsPerUnit(drive, platform.DriveStepsPerUnit(drive) * (float)microsteps / (float)oldSteps);
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
void GCodes::HandleReply(GCodeBuffer& gb, bool error, const char* reply)
{
	// Don't report "ok" responses if a (macro) file is being processed
	// Also check that this response was triggered by a gcode
	if ((gb.MachineState().doingFileMacro || &gb == fileGCode) && reply[0] == 0)
	{
		return;
	}

	// Second UART device, e.g. PanelDue. Do NOT use emulation for this one!
	if (&gb == auxGCode)
	{
		platform.AppendAuxReply(reply);
		return;
	}

	const Compatibility c = (&gb == serialGCode || &gb == telnetGCode) ? platform.Emulating() : me;
	const MessageType type = gb.GetResponseMessageType();
	const char* const response = (gb.GetCommandLetter() == 'M' && gb.GetCommandNumber() == 998) ? "rs " : "ok";
	const char* emulationType = nullptr;

	switch (c)
	{
	case me:
	case reprapFirmware:
		platform.MessageF((error) ? (MessageType)(type | ErrorMessageFlag) : type, "%s\n", reply);
		break;

	case marlin:
		// We don't need to handle M20 here because we always allocate an output buffer for that one
		if (gb.GetCommandLetter() == 'M' && gb.GetCommandNumber() == 28)
		{
			platform.MessageF(type, "%s\n%s\n", response, reply);
		}
		else if (gb.GetCommandLetter() == 'M' && (gb.GetCommandNumber() == 105 || gb.GetCommandNumber() == 998))
		{
			platform.MessageF(type, "%s %s\n", response, reply);
		}
		else if (reply[0] != 0 && !gb.IsDoingFileMacro())
		{
			platform.MessageF(type, "%s\n%s\n", reply, response);
		}
		else if (reply[0] != 0)
		{
			platform.MessageF(type, "%s\n", reply);
		}
		else
		{
			platform.MessageF(type, "%s\n", response);
		}
		break;

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

	if (emulationType != nullptr)
	{
		platform.MessageF(type, "Emulation of %s is not yet supported.\n", emulationType);
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
	const char* const response = (gb.GetCommandLetter() == 'M' && gb.GetCommandNumber() == 998) ? "rs " : "ok";
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
		if (gb.GetCommandLetter() =='M' && gb.GetCommandNumber() == 20)
		{
			platform.Message(type, "Begin file list\n");
			platform.Message(type, reply);
			platform.Message(type, "End file list\n");
			platform.Message(type, response);
			platform.Message(type, "\n");
			return;
		}

		if (gb.GetCommandLetter() == 'M' && gb.GetCommandNumber() == 28)
		{
			platform.Message(type, response);
			platform.Message(type, "\n");
			platform.Message(type, reply);
			return;
		}

		if (gb.GetCommandLetter() =='M' && (gb.GetCommandNumber() == 105 || gb.GetCommandNumber() == 998))
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

// Configure heater protection (M143). Returns true if an error occurred
bool GCodes::SetHeaterProtection(GCodeBuffer& gb, const StringRef& reply)
{
	const int index = (gb.Seen('P'))
						? gb.GetIValue()
						: (gb.Seen('H')) ? gb.GetIValue() : 1;		// default to extruder 1 if no heater number provided
	if ((index < 0 || index >= (int)Heaters) && (index < (int)FirstExtraHeaterProtection || index >= (int)(FirstExtraHeaterProtection + NumExtraHeaterProtections)))
	{
		reply.printf("Invalid heater protection item '%d'", index);
		return true;
	}

	HeaterProtection &item = reprap.GetHeat().AccessHeaterProtection(index);

	// Set heater to control
	bool seen = false;
	if (gb.Seen('P') && gb.Seen('H'))
	{
		const int heater = gb.GetIValue();
		if (heater > (int)Heaters)									// allow negative values to disable heater protection
		{
			reply.printf("Invalid heater number '%d'", heater);
			return true;
		}

		seen = true;
		item.SetHeater(heater);
	}

	// Set heater to supervise
	if (gb.Seen('X'))
	{
		const int heater = gb.GetIValue();
		if ((heater < 0 || heater > (int)Heaters) && (heater < (int)FirstVirtualHeater || heater >= (int)(FirstVirtualHeater + MaxVirtualHeaters)))
		{
			reply.printf("Invalid heater number '%d'", heater);
			return true;
		}

		seen = true;
		item.SetSupervisedHeater(heater);
	}

	// Set trigger action
	if (gb.Seen('A'))
	{
		const int action = gb.GetIValue();
		if (action < 0 || action > (int)MaxHeaterProtectionAction)
		{
			reply.printf("Invalid heater protection action '%d'", action);
		}

		seen = true;
		item.SetAction(static_cast<HeaterProtectionAction>(action));
	}

	// Set trigger condition
	if (gb.Seen('C'))
	{
		const int trigger = gb.GetIValue();
		if (trigger < 0 || trigger > (int)MaxHeaterProtectionTrigger)
		{
			reply.printf("Invalid heater protection trigger '%d'", trigger);
		}

		seen = true;
		item.SetTrigger(static_cast<HeaterProtectionTrigger>(trigger));
	}

	// Set temperature limit
	if (gb.Seen('S'))
	{
		const float limit = gb.GetFValue();
		if (limit <= BAD_LOW_TEMPERATURE || limit >= BAD_ERROR_TEMPERATURE)
		{
			reply.copy("Invalid temperature limit");
			return true;
		}

		seen = true;
		item.SetTemperatureLimit(limit);
	}

	// Report current parameters
	if (!seen)
	{
		if (item.GetHeater() < 0)
		{
			reply.printf("Temperature protection item %d is not configured", index);
		}
		else
		{
			const char *actionString, *triggerString;
			switch (item.GetAction())
			{
			case HeaterProtectionAction::GenerateFault:
				actionString = "generate a heater fault";
				break;
			case HeaterProtectionAction::PermanentSwitchOff:
				actionString = "permanently switch off";
				break;
			case HeaterProtectionAction::TemporarySwitchOff:
				actionString = "temporarily switch off";
				break;
			default:
				actionString = "(undefined)";
				break;
			}

			switch (item.GetTrigger())
			{
			case HeaterProtectionTrigger::TemperatureExceeded:
				triggerString = "exceeds";
				break;
			case HeaterProtectionTrigger::TemperatureTooLow:
				triggerString = "falls below";
				break;
			default:
				triggerString = "(undefined)";
				break;
			}

			reply.printf("Temperature protection item %d is configured for heater %d and supervises heater %d to %s if the temperature %s %.1f" DEGREE_SYMBOL "C",
					index, item.GetHeater(), item.GetSupervisedHeater(), actionString, triggerString, (double)item.GetTemperatureLimit());
		}
	}

	return false;
}

// Set PID parameters (M301 or M304 command). 'heater' is the default heater number to use.
void GCodes::SetPidParameters(GCodeBuffer& gb, int heater, const StringRef& reply)
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
			reply.printf("Heater %d P:%.1f I:%.3f D:%.1f", heater, (double)pp.kP, (double)pp.kI, (double)pp.kD);
		}
		else
		{
			reply.printf("Heater %d uses model-derived PID parameters. Use M307 H%d to view them", heater, heater);
		}
	}
}

// Process M305
GCodeResult GCodes::SetHeaterParameters(GCodeBuffer& gb, const StringRef& reply)
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
				if (heater < (int)Heaters && (gb.Seen('R') || gb.Seen('T') || gb.Seen('B')))	// if it has a thermistor and we are trying to configure it
				{
					channel = heater;
				}
				else
				{
					reply.printf("heater %d is not configured", heater);
					return GCodeResult::error;
				}
			}

			if (channel != oldChannel)
			{
				if (heat.SetHeaterChannel(heater, channel))
				{
					reply.printf("unable to use sensor channel %" PRIi32 " on heater %d", channel, heater);
					return GCodeResult::error;
				}
			}

			bool hadError = false;
			heat.ConfigureHeaterSensor(305, (unsigned int)heater, gb, reply, hadError);
			return GetGCodeResultFromError(hadError);
		}
		else
		{
			reply.printf("heater number %d is out of range", heater);
			return GCodeResult::error;
		}
	}
	return GCodeResult::ok;
}

void GCodes::SetToolHeaters(Tool *tool, float temperature, bool both)
{
	if (tool == nullptr)
	{
		platform.Message(ErrorMessage, "Setting temperature: no tool selected.\n");
		return;
	}

	float standby[Heaters];
	float active[Heaters];
	tool->GetVariables(standby, active);
	for (size_t h = 0; h < tool->HeaterCount(); h++)
	{
		active[h] = temperature;
		if (both)
		{
			standby[h] = temperature;
		}
	}
	tool->SetVariables(standby, active);
}

// Retract or un-retract filament, returning true if movement has been queued, false if this needs to be called again
GCodeResult GCodes::RetractFilament(GCodeBuffer& gb, bool retract)
{
	if (retract != isRetracted && (retractLength != 0.0 || retractHop != 0.0 || (!retract && retractExtra != 0.0)))
	{
		if (!LockMovement(gb))
		{
			return GCodeResult::notFinished;
		}

		if (segmentsLeft != 0)
		{
			return GCodeResult::notFinished;
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
		moveBuffer.isCoordinated = false;
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
			moveBuffer.coords[Z_AXIS] -= currentZHop;
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
	return GCodeResult::ok;
}

// Load the specified filament into a tool
GCodeResult GCodes::LoadFilament(GCodeBuffer& gb, const StringRef& reply)
{
	Tool * const tool = reprap.GetCurrentTool();
	if (tool == nullptr)
	{
		reply.copy("No tool selected");
		return GCodeResult::error;
	}

	if (tool->GetFilament() == nullptr)
	{
		reply.copy("Loading filament into the selected tool is not supported");
		return GCodeResult::error;
	}

	if (gb.Seen('S'))
	{
		String<FilamentNameLength> filamentName;
		if (!gb.GetQuotedString(filamentName.GetRef()) || filamentName.IsEmpty())
		{
			reply.copy("Invalid filament name");
			return GCodeResult::error;
		}

		if (StringContains(filamentName.c_str(), ",") >= 0)
		{
			reply.copy("Filament names must not contain commas");
			return GCodeResult::error;
		}

		if (StringEquals(filamentName.c_str(), tool->GetFilament()->GetName()))
		{
			// Filament already loaded - nothing to do
			return GCodeResult::ok;
		}

		if (tool->GetFilament()->IsLoaded())
		{
			reply.copy("Unload the current filament before you attempt to load another one");
			return GCodeResult::error;
		}

		if (!platform.GetMassStorage()->DirectoryExists(FILAMENTS_DIRECTORY, filamentName.c_str()))
		{
			reply.copy("Filament configuration directory not found");
			return GCodeResult::error;
		}

		if (Filament::IsInUse(filamentName.c_str()))
		{
			reply.copy("One filament type can be only assigned to a single tool");
			return GCodeResult::error;
		}

		SafeStrncpy(filamentToLoad, filamentName.c_str(), ARRAY_SIZE(filamentToLoad));
		gb.SetState(GCodeState::loadingFilament);

		scratchString.printf("%s%s/%s", FILAMENTS_DIRECTORY, filamentName.c_str(), LOAD_FILAMENT_G);
		DoFileMacro(gb, scratchString.Pointer(), true);
		return GCodeResult::ok;
	}
	else if (tool->GetFilament()->IsLoaded())
	{
		reply.printf("Loaded filament in the selected tool: %s", tool->GetFilament()->GetName());
		return GCodeResult::ok;
	}
	else
	{
		reply.printf("No filament loaded in the selected tool");
		return GCodeResult::ok;
	}
}

// Unload the current filament from a tool
GCodeResult GCodes::UnloadFilament(GCodeBuffer& gb, const StringRef& reply)
{
	Tool *tool = reprap.GetCurrentTool();
	if (tool == nullptr)
	{
		reply.copy("No tool selected");
		return GCodeResult::error;
	}

	if (tool->GetFilament() == nullptr)
	{
		reply.copy("Unloading filament from this tool is not supported");
		return GCodeResult::error;
	}

	if (!tool->GetFilament()->IsLoaded())
	{
		// Filament already unloaded - nothing to do
		return GCodeResult::ok;
	}

	gb.SetState(GCodeState::unloadingFilament);
	scratchString.printf("%s%s/%s", FILAMENTS_DIRECTORY, tool->GetFilament()->GetName(), UNLOAD_FILAMENT_G);
	DoFileMacro(gb, scratchString.Pointer(), true);
	return GCodeResult::ok;
}

float GCodes::GetRawExtruderTotalByDrive(size_t extruder) const
{
	return (extruder < numExtruders) ? rawExtruderTotalByDrive[extruder] : 0.0;
}

// Return true if the code queue is idle
bool GCodes::IsCodeQueueIdle() const
{
	return queuedGCode->IsIdle() && codeQueue->IsIdle();
}

// Cancel the current SD card print.
// This is called from Pid.cpp when there is a heater fault, and from elsewhere in this module.
void GCodes::StopPrint(bool normalCompletion)
{
	segmentsLeft = 0;
	isPaused = pausePending = false;

	FileData& fileBeingPrinted = fileGCode->OriginalMachineState().fileState;

	fileInput->Reset(fileBeingPrinted);
	fileGCode->Init();

	if (fileBeingPrinted.IsLive())
	{
		fileBeingPrinted.Close();
	}

	reprap.GetMove().ResetMoveCounters();
	codeQueue->Clear();

	UnlockAll(*fileGCode);

	const char *printingFilename = reprap.GetPrintMonitor().GetPrintingFilename();
	if (printingFilename == nullptr)
	{
		printingFilename = "(unknown)";
	}

	if (exitSimulationWhenFileComplete)
	{
		exitSimulationWhenFileComplete = false;
		simulationMode = 0;
		reprap.GetMove().Simulate(simulationMode);
		EndSimulation(nullptr);
		const uint32_t simMinutes = lrintf((reprap.GetMove().GetSimulationTime() + simulationTime)/60.0);
		platform.MessageF(LoggedGenericMessage, "File %s will print in %" PRIu32 "h %" PRIu32 "m plus heating time\n",
								printingFilename, simMinutes/60u, simMinutes % 60u);
	}
	else if (reprap.GetPrintMonitor().IsPrinting())
	{
		if (platform.Emulating() == marlin)
		{
			// Pronterface expects a "Done printing" message
			platform.Message(UsbMessage, "Done printing file\n");
		}
		const uint32_t printMinutes = lrintf(reprap.GetPrintMonitor().GetPrintDuration()/60.0);
		platform.MessageF(LoggedGenericMessage, "%s printing file %s, print time was %" PRIu32 "h %" PRIu32 "m\n",
			(normalCompletion) ? "Finished" : "Cancelled",
			printingFilename, printMinutes/60u, printMinutes % 60u);
	}

	reprap.GetPrintMonitor().StoppedPrint();		// must do this after printing the simulation details because it clears the filename
	if (normalCompletion && simulationMode == 0)
	{
		platform.GetMassStorage()->Delete(platform.GetSysDir(), RESUME_AFTER_POWER_FAIL_G, true);
	}
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

	rp.feedRate = gb.MachineState().feedrate;
	rp.virtualExtruderPosition = virtualExtruderPosition;

#if SUPPORT_IOBITS
	rp.ioBits = moveBuffer.ioBits;
#endif
}

// Restore user position from a restore point
void GCodes::RestorePosition(const RestorePoint& rp, GCodeBuffer *gb)
{
	for (size_t axis = 0; axis < numVisibleAxes; ++axis)
	{
		currentUserPosition[axis] = rp.moveCoords[axis];
	}
	if (gb != nullptr)
	{
		gb->MachineState().feedrate = rp.feedRate;
	}

#if SUPPORT_IOBITS
	moveBuffer.ioBits = rp.ioBits;
#endif
}

// Convert user coordinates to head reference point coordinates, optionally allowing for X axis mapping
// If the X axis is mapped to some other axes not including X, then the X coordinate of coordsOut will be left unchanged. So make sure it is suitably initialised before calling this.
void GCodes::ToolOffsetTransform(const float coordsIn[MaxAxes], float coordsOut[MaxAxes], AxesBitmap explicitAxes)
{
	const Tool * const currentTool = reprap.GetCurrentTool();
	if (currentTool == nullptr)
	{
		for (size_t axis = 0; axis < numVisibleAxes; ++axis)
		{
			const float totalOffset =
#if SUPPORT_WORKPLACE_COORDINATES
				workplaceCoordinates[currentCoordinateSystem][axis];
#else
				axisOffsets[axis];
#endif
			coordsOut[axis] = (coordsIn[axis] * axisScaleFactors[axis]) + totalOffset;
		}
	}
	else
	{
		const AxesBitmap xAxes = currentTool->GetXAxisMap();
		const AxesBitmap yAxes = currentTool->GetYAxisMap();
		for (size_t axis = 0; axis < numVisibleAxes; ++axis)
		{
			if (   (axis != X_AXIS || IsBitSet(xAxes, X_AXIS))
				&& (axis != Y_AXIS || IsBitSet(yAxes, Y_AXIS))
			   )
			{
				const float totalOffset =
#if SUPPORT_WORKPLACE_COORDINATES
					workplaceCoordinates[currentCoordinateSystem][axis]
#else
					axisOffsets[axis]
#endif
					- currentTool->GetOffset(axis);
				const size_t inputAxis = (IsBitSet(explicitAxes, axis)) ? axis
										: (IsBitSet(xAxes, axis)) ? X_AXIS
											: (IsBitSet(yAxes, axis)) ? Y_AXIS
												: axis;
				coordsOut[axis] = (coordsIn[inputAxis] * axisScaleFactors[axis]) + totalOffset;
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
			const float totalOffset =
#if SUPPORT_WORKPLACE_COORDINATES
				workplaceCoordinates[currentCoordinateSystem][axis];
#else
				axisOffsets[axis];
#endif
			coordsOut[axis] = (coordsIn[axis] - totalOffset) / axisScaleFactors[axis];
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
			const float totalOffset =
#if SUPPORT_WORKPLACE_COORDINATES
				workplaceCoordinates[currentCoordinateSystem][axis]
#else
				axisOffsets[axis]
#endif
				- currentTool->GetOffset(axis);
			coordsOut[axis] = coordsIn[axis] + currentTool->GetOffset(axis);
			if (IsBitSet(xAxes, axis))
			{
				xCoord += coordsIn[axis]/axisScaleFactors[axis] - totalOffset;
				++numXAxes;
			}
			if (IsBitSet(yAxes, axis))
			{
				yCoord += coordsIn[axis]/axisScaleFactors[axis] - totalOffset;
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

	case EndStopHit::nearStop:
		return "near stop";

	case EndStopHit::noStop:
	default:
		return "not stopped";
	}
}

// Append a list of trigger endstops to a message
void GCodes::ListTriggers(const StringRef& reply, TriggerInputsBitmap mask)
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
	fileBeingHashed = platform.OpenFile(FS_PREFIX, filename, OpenMode::read);
	if (fileBeingHashed == nullptr)
	{
		return false;
	}

	// Start hashing
	SHA1Reset(&hash);
	return true;
}

GCodeResult GCodes::AdvanceHash(const StringRef &reply)
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
				reply.catf("%" PRIx32, hash.Message_Digest[i]);
			}

			// Clean up again
			fileBeingHashed->Close();
			fileBeingHashed = nullptr;
			return GCodeResult::ok;
		}
		return GCodeResult::notFinished;
	}

	// Something went wrong, we cannot read any more from the file
	fileBeingHashed->Close();
	fileBeingHashed = nullptr;
	return GCodeResult::ok;
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
bool GCodes::WriteConfigOverrideFile(GCodeBuffer& gb, const StringRef& reply, const char *fileName) const
{
	FileStore * const f = platform.OpenFile(platform.GetSysDir(), fileName, OpenMode::write);
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
		ok = platform.WritePlatformParameters(f, gb.Seen('P') && gb.GetIValue() == 31);
	}

	if (ok)
	{
		ok = reprap.WriteToolParameters(f);
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
void GCodes::GenerateTemperatureReport(const StringRef& reply) const
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
			reply.catf("%c%.1f /%.1f", sep, (double)heat.GetTemperature(heater), (double)heat.GetTargetTemperature(heater));
			sep = ' ';
		}
	}

	const int bedHeater = (NumBedHeaters > 0) ? heat.GetBedHeater(0) : -1;				// default to first heated bed
	if (bedHeater >= 0)
	{
		reply.catf(" B:%.1f /%.1f", (double)heat.GetTemperature(bedHeater), (double)heat.GetTargetTemperature(bedHeater));
	}

	const int chamberHeater = (NumChamberHeaters > 0) ? heat.GetChamberHeater(0) : -1;	// default to first chamber heater
	if (chamberHeater >= 0)
	{
		reply.catf(" C:%.1f /%.1f", (double)heat.GetTemperature(chamberHeater), (double)heat.GetTargetTemperature(chamberHeater));
	}
}

// Check whether we need to report temperatures or status.
// 'reply' is a convenient buffer that is free for us to use.
void GCodes::CheckReportDue(GCodeBuffer& gb, const StringRef& reply) const
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
				platform.Message(UsbMessage, reply.Pointer());
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

// Grab the movement lock even if another GCode source has it
// CAUTION: this will be unsafe when we move to RTOS
void GCodes::GrabResource(const GCodeBuffer& gb, Resource r)
{
	if (resourceOwners[r] != &gb)
	{
		if (resourceOwners[r] != nullptr)
		{
			GCodeMachineState *m = &(resourceOwners[r]->MachineState());
			do
			{
				ClearBit(m->lockedResources, r);
				m = m->previous;
			}
			while (m != nullptr);
		}
		resourceOwners[r] = &gb;
	}
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

void GCodes::GrabMovement(const GCodeBuffer& gb)
{
	GrabResource(gb, MoveResource);
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
void GCodes::AppendAxes(const StringRef& reply, AxesBitmap axes) const
{
	for (size_t axis = 0; axis < numVisibleAxes; ++axis)
	{
		if (IsBitSet(axes, axis))
		{
			reply.cat(axisLetters[axis]);
		}
	}
}

// Get the name of the current machine mode
const char* GCodes::GetMachineModeString() const
{
	switch (machineType)
	{
	case MachineType::fff:
		return "FFF";
	case MachineType::cnc:
		return "CNC";
	case MachineType::laser:
		return "Laser";
	default:
		return "Unknown";
	}
}

// Respond to a heater fault. The heater has already been turned off and its status set to 'fault' when this is called from the Heat module.
// The Heat module will generate an appropriate error message, so on need to do that here.
void GCodes::HandleHeaterFault(int heater)
{
	if (heaterFaultState == HeaterFaultState::noFault && fileGCode->OriginalMachineState().fileState.IsLive())
	{
		heaterFaultState = HeaterFaultState::pausePending;
		heaterFaultTime = millis();
	}
}

// Check for and respond to a heater fault, returning true if we should exit
void GCodes::CheckHeaterFault()
{
	switch (heaterFaultState)
	{
	case HeaterFaultState::noFault:
	default:
		break;

	case HeaterFaultState::pausePending:
		if (   IsReallyPrinting()
			&& autoPauseGCode->IsCompletelyIdle()
			&& LockMovement(*autoPauseGCode)							// need to lock movement before executing the pause macro
		   )
		{
			reprap.GetHeat().SwitchOffAll(false);						// turn off all extruder heaters
			DoPause(*autoPauseGCode, PauseReason::heaterFault, "Heater fault");
			heaterFaultState = HeaterFaultState::timing;
		}
		else if (IsPausing() || IsPaused())
		{
			heaterFaultState = HeaterFaultState::timing;
		}
		// no break

	case HeaterFaultState::timing:
		if (millis() - heaterFaultTime >= heaterFaultTimeout)
		{
			StopPrint(false);
			reprap.GetHeat().SwitchOffAll(true);
			platform.MessageF(ErrorMessage, "Shutting down due to un-cleared heater fault after %lu seconds\n", heaterFaultTimeout/1000);
			heaterFaultState = HeaterFaultState::stopping;
		}
		break;

	case HeaterFaultState::stopping:
		if (millis() - heaterFaultTime >= 2000)			// wait 2 seconds for the message to be picked up by DWC and PanelDue
		{
			platform.AtxPowerOff(false);
			heaterFaultState = HeaterFaultState::stopped;
		}
		break;
	}
}

#if SUPPORT_12864_LCD

// Process a GCode command from the 12864 LCD returning true if the command was accepted
bool GCodes::ProcessCommandFromLcd(const char *cmd)
{
	if (lcdGCode->IsCompletelyIdle())
	{
		lcdGCode->Put(cmd);
		return true;
	}
	return false;
}

#endif

// End
