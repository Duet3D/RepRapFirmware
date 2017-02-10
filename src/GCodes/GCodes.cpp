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
#include "Heating/Heat.h"
#include "Platform.h"
#include "Movement/Move.h"
#include "PrintMonitor.h"
#include "RepRap.h"
#include "Tool.h"
#include "Webserver.h"

#ifdef DUET_NG
#include "FirmwareUpdater.h"
#endif

#define DEGREE_SYMBOL	"\xC2\xB0"				// degree-symbol encoding in UTF8

const char GCodes::axisLetters[MAX_AXES] = { 'X', 'Y', 'Z', 'U', 'V', 'W' };

const char* PAUSE_G = "pause.g";
const char* HomingFileNames[MAX_AXES] = { "homex.g", "homey.g", "homez.g", "homeu.g", "homev.g", "homew.g" };
const char* HOME_ALL_G = "homeall.g";
const char* HOME_DELTA_G = "homedelta.g";
const char* DefaultHeightMapFile = "heightmap.csv";

const size_t gcodeReplyLength = 2048;			// long enough to pass back a reasonable number of files in response to M20

void GCodes::RestorePoint::Init()
{
	for (size_t i = 0; i < DRIVES; ++i)
	{
		moveCoords[i] = 0.0;
	}
	feedRate = DefaultFeedrate * SecondsToMinutes;
}

GCodes::GCodes(Platform* p, Webserver* w) :
	platform(p), webserver(w), active(false), isFlashing(false),
	fileBeingHashed(nullptr)
{
	httpGCode = new GCodeBuffer("http", HTTP_MESSAGE);
	telnetGCode = new GCodeBuffer("telnet", TELNET_MESSAGE);
	fileGCode = new GCodeBuffer("file", GENERIC_MESSAGE);
	serialGCode = new GCodeBuffer("serial", HOST_MESSAGE);
	auxGCode = new GCodeBuffer("aux", AUX_MESSAGE);
	daemonGCode = new GCodeBuffer("daemon", GENERIC_MESSAGE);
}

void GCodes::Exit()
{
	platform->Message(HOST_MESSAGE, "GCodes class exited.\n");
	active = false;
}

void GCodes::Init()
{
	Reset();
	numAxes = MIN_AXES;
	numExtruders = MaxExtruders;
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
	offSetSet = false;
	runningConfigFile = false;
	doingToolChange = false;
	active = true;
	longWait = platform->Time();
	dwellTime = longWait;
	limitAxes = true;
	for(size_t axis = 0; axis < MAX_AXES; axis++)
	{
		axisScaleFactors[axis] = 1.0;
	}
	SetAllAxesNotHomed();
	for (size_t i = 0; i < NUM_FANS; ++i)
	{
		pausedFanValues[i] = 0.0;
	}
	lastDefaultFanSpeed = 0.0;

	retractLength = DefaultRetractLength;
	retractExtra = 0.0;
	retractHop = 0.0;
	retractSpeed = unRetractSpeed = DefaultRetractSpeed * SecondsToMinutes;
	isRetracted = false;
}

// This is called from Init and when doing an emergency stop
void GCodes::Reset()
{
	httpGCode->Init();
	telnetGCode->Init();
	fileGCode->Init();
	serialGCode->Init();
	auxGCode->Init();
	auxGCode->SetCommsProperties(1);					// by default, we require a checksum on the aux port
	daemonGCode->Init();

	nextGcodeSource = 0;

	fileToPrint.Close();
	fileBeingWritten = NULL;
	dwellWaiting = false;
	probeCount = 0;
	cannedCycleMoveCount = 0;
	cannedCycleMoveQueued = false;
	speedFactor = SecondsToMinutes;						// default is just to convert from mm/minute to mm/second
	for (size_t i = 0; i < MaxExtruders; ++i)
	{
		extrusionFactors[i] = 1.0;
	}
	for (size_t i = 0; i < DRIVES; ++i)
	{
		moveBuffer.coords[i] = 0.0;
	}
	moveBuffer.xAxes = DefaultXAxisMapping;

	pauseRestorePoint.Init();
	toolChangeRestorePoint.Init();

	ClearMove();

	for (size_t i = 0; i < MaxTriggers; ++i)
	{
		triggers[i].Init();
	}
	triggersPending = 0;

	simulationMode = 0;
	simulationTime = 0.0;
	isPaused = false;
	moveBuffer.filePos = noFilePosition;
	lastEndstopStates = platform->GetAllEndstopStates();
	firmwareUpdateModuleMap = 0;

	cancelWait = isWaiting = false;

	for (size_t i = 0; i < NumResources; ++i)
	{
		resourceOwners[i] = nullptr;
	}
}

bool GCodes::DoingFileMacro() const
{
	return fileGCode->IsDoingFileMacro();
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
		StartNextGCode(gb, reply);
	}
	else
	{
		// Perform the next operation of the state machine for this gcode source
		bool error = false;

		switch (gb.GetState())
		{
		case GCodeState::waitingForMoveToComplete:
			if (LockMovementAndWaitForStandstill(gb))
			{
				gb.SetState(GCodeState::normal);
			}
			break;

		case GCodeState::homing:
			if (toBeHomed == 0)
			{
				gb.SetState(GCodeState::normal);
			}
			else
			{
				for (size_t axis = 0; axis < numAxes; ++axis)
				{
					// Leave the Z axis until all other axes are done
					if ((toBeHomed & (1u << axis)) != 0 && (axis != Z_AXIS || toBeHomed == (1u << Z_AXIS)))
					{
						toBeHomed &= ~(1u << axis);
						DoFileMacro(gb, HomingFileNames[axis], true);
						break;
					}
				}
			}
			break;

		case GCodeState::setBed:
			if (DoSingleZProbeAtPoint(gb, probeCount, 0.0))
			{
				probeCount++;
				if (probeCount >= reprap.GetMove()->NumberOfXYProbePoints())
				{
					reprap.GetMove()->FinishedBedProbing(0, reply);
					gb.SetState(GCodeState::normal);
				}
			}
			break;

		case GCodeState::toolChange0: 		// Run tfree for the old tool (if any)
		case GCodeState::m109ToolChange0:	// Run tfree for the old tool (if any)
			doingToolChange = true;
			gb.AdvanceState();
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
				const Tool *oldTool = reprap.GetCurrentTool();
				if (oldTool != NULL)
				{
					reprap.StandbyTool(oldTool->Number());
				}
			}
			gb.AdvanceState();
			if (reprap.GetTool(newToolNumber) != nullptr && AllAxesAreHomed())
			{
				scratchString.printf("tpre%d.g", newToolNumber);
				DoFileMacro(gb, scratchString.Pointer(), false);
			}
			break;

		case GCodeState::toolChange2:		// Select the new tool (even if it doesn't exist - that just deselects all tools) and run tpost
		case GCodeState::m109ToolChange2:	// Select the new tool (even if it doesn't exist - that just deselects all tools) and run tpost
			reprap.SelectTool(newToolNumber);
			gb.AdvanceState();
			if (reprap.GetTool(newToolNumber) != nullptr && AllAxesAreHomed())
			{
				scratchString.printf("tpost%d.g", newToolNumber);
				DoFileMacro(gb, scratchString.Pointer(), false);
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
			// In Marlin emulation mode we should return some sort of (undocumented) message here every second...
			isWaiting = true;
			break;

		case GCodeState::pausing1:
			if (LockMovementAndWaitForStandstill(gb))
			{
				gb.SetState(GCodeState::pausing2);
				DoFileMacro(gb, PAUSE_G, true);
			}
			break;

		case GCodeState::pausing2:
			if (LockMovementAndWaitForStandstill(gb))
			{
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
				for (size_t drive = 0; drive < numAxes; ++drive)
				{
					moveBuffer.coords[drive] =  pauseRestorePoint.moveCoords[drive];
				}
				for (size_t drive = numAxes; drive < DRIVES; ++drive)
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
					platform->SetFanValue(i, pausedFanValues[i]);
				}
				for (size_t drive = numAxes; drive < DRIVES; ++drive)
				{
					lastRawExtruderPosition[drive - numAxes] = pauseRestorePoint.moveCoords[drive];	// reset the extruder position in case we are receiving absolute extruder moves
				}
				fileGCode->MachineState().feedrate = pauseRestorePoint.feedRate;
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
				platform->UpdateFirmware();
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
				reprap.GetHeat()->SwitchOffAll();
			}

			// chrishamm 2014-18-10: Although RRP says M0 is supposed to turn off all drives and heaters,
			// I think M1 is sufficient for this purpose. Leave M0 for a normal reset.
			if (gb.GetState() == GCodeState::sleeping)
			{
				DisableDrives();
			}
			else
			{
				platform->SetDriversIdle();
			}
			gb.SetState(GCodeState::normal);
			break;

		case GCodeState::gridProbing1:	// ready to move to next grid probe point
			{
				// Move to the current probe point
				const GridDefinition& grid = reprap.GetMove()->AccessBedProbeGrid().GetGrid();
				const float x = grid.GetXCoordinate(gridXindex);
				const float y = grid.GetYCoordinate(gridYindex);
				if (grid.IsInRadius(x, y) && platform->IsAccessibleProbePoint(x, y))
				{
					moveBuffer.moveType = 0;
					moveBuffer.endStopsToCheck = 0;
					moveBuffer.usePressureAdvance = false;
					moveBuffer.filePos = noFilePosition;
					moveBuffer.coords[X_AXIS] = x - platform->GetCurrentZProbeParameters().xOffset;
					moveBuffer.coords[Y_AXIS] = y - platform->GetCurrentZProbeParameters().yOffset;
					moveBuffer.coords[Z_AXIS] = platform->GetZProbeStartingHeight();
					moveBuffer.feedRate = platform->GetZProbeTravelSpeed();
					moveBuffer.xAxes = 0;
					segmentsLeft = 1;
					gb.AdvanceState();
				}
				else
				{
					gb.SetState(GCodeState::gridProbing4);
				}
			}
			break;

		case GCodeState::gridProbing2:	// ready to probe the current grid probe point
			if (LockMovementAndWaitForStandstill(gb))
			{
				lastProbedTime = millis();
				gb.AdvanceState();
			}
			break;

		case GCodeState::gridProbing2a:	// ready to probe the current grid probe point
			if (millis() - lastProbedTime >= (uint32_t)(reprap.GetPlatform()->GetCurrentZProbeParameters().recoveryTime * SecondsToMillis))
			{
				// Probe the bed at the current XY coordinates
				// Check for probe already triggered at start
				if (reprap.GetPlatform()->GetZProbeResult() == EndStopHit::lowHit)
				{
					reply.copy("Z probe already triggered before probing move started");
					error = true;
					gb.SetState(GCodeState::normal);
					break;
				}

				zProbeTriggered = false;
				platform->SetProbing(true);
				moveBuffer.moveType = 0;
				moveBuffer.endStopsToCheck = ZProbeActive;
				moveBuffer.usePressureAdvance = false;
				moveBuffer.filePos = noFilePosition;
				moveBuffer.coords[Z_AXIS] = -platform->GetZProbeDiveHeight();
				moveBuffer.feedRate = platform->GetCurrentZProbeParameters().probeSpeed;
				moveBuffer.xAxes = 0;
				segmentsLeft = 1;
				gb.SetState(GCodeState::gridProbing3);
			}
			break;

		case GCodeState::gridProbing3:	// ready to lift the probe after probing the current grid probe point
			if (LockMovementAndWaitForStandstill(gb))
			{
				platform->SetProbing(false);
				if (!zProbeTriggered)
				{
					reply.copy("Z probe was not triggered during probing move");
					error = true;
					gb.SetState(GCodeState::normal);
					break;
				}

				const float heightError = moveBuffer.coords[Z_AXIS] - platform->ZProbeStopHeight();
				reprap.GetMove()->AccessBedProbeGrid().SetGridHeight(gridXindex, gridYindex, heightError);

				// Move back up to the dive height
				moveBuffer.moveType = 0;
				moveBuffer.endStopsToCheck = 0;
				moveBuffer.usePressureAdvance = false;
				moveBuffer.filePos = noFilePosition;
				moveBuffer.coords[Z_AXIS] = platform->GetZProbeStartingHeight();
				moveBuffer.feedRate = platform->GetZProbeTravelSpeed();
				moveBuffer.xAxes = 0;
				segmentsLeft = 1;
				gb.SetState(GCodeState::gridProbing4);
			}
			break;

		case GCodeState::gridProbing4:	// ready to compute the next probe point
			if (LockMovementAndWaitForStandstill(gb))
			{
				const GridDefinition& grid = reprap.GetMove()->AccessBedProbeGrid().GetGrid();
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
					// Finished probing the grid
					float mean, deviation;
					const uint32_t numPointsProbed = reprap.GetMove()->AccessBedProbeGrid().GetStatistics(mean, deviation);
					if (numPointsProbed >= 4)
					{
						reply.printf("%u points probed, mean error %.3f, deviation %.3f\n", numPointsProbed, mean, deviation);
						error = SaveHeightMap(gb, reply);
						reprap.GetMove()->AccessBedProbeGrid().UseHeightMap(true);
					}
					else
					{
						reply.copy("Too few points probed");
						error = true;
					}
					gb.SetState(GCodeState::normal);
				}
				else
				{
					gb.SetState(GCodeState::gridProbing1);
				}
			}
			break;

		default:				// should not happen
			platform->Message(GENERIC_MESSAGE, "Error: undefined GCodeState\n");
			gb.SetState(GCodeState::normal);
			break;
		}

		if (gb.GetState() == GCodeState::normal)
		{
			// We completed a command, so unlock resources and tell the host about it
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

	platform->ClassReport(longWait);
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
	else if (&gb == httpGCode)
	{
		// Webserver
		for (unsigned int i = 0; i < 16 && webserver->GCodeAvailable(WebSource::HTTP); ++i)
		{
			const char b = webserver->ReadGCode(WebSource::HTTP);
			if (gb.Put(b))
			{
				// We have a complete gcode
				if (gb.WritingFileDirectory() != nullptr)
				{
					WriteGCodeToFile(gb);
					gb.SetFinished(true);
				}
				else
				{
					gb.SetFinished(ActOnCode(gb, reply));
				}
				break;
			}
		}
	}
	else if (&gb == telnetGCode)
	{
		// Telnet
		for (unsigned int i = 0; i < GCODE_LENGTH && webserver->GCodeAvailable(WebSource::Telnet); ++i)
		{
			char b = webserver->ReadGCode(WebSource::Telnet);
			if (gb.Put(b))
			{
				gb.SetFinished(ActOnCode(gb, reply));
				break;
			}
		}
	}
	else if (&gb == serialGCode)
	{
		// USB interface
		for (unsigned int i = 0; i < 16 && platform->GCodeAvailable(SerialSource::USB); ++i)
		{
			const char b = platform->ReadFromSource(SerialSource::USB);
			// Check the special case of uploading the reprap.htm file
			if (gb.WritingFileDirectory() == platform->GetWebDir())
			{
				WriteHTMLToFile(gb, b);
			}
			else if (gb.Put(b))	// add char to buffer and test whether the gcode is complete
			{
				// We have a complete gcode
				if (gb.WritingFileDirectory() != nullptr)
				{
					WriteGCodeToFile(gb);
					gb.SetFinished(true);
				}
				else
				{
					gb.SetFinished(ActOnCode(gb, reply));
				}
				break;
			}
		}
	}
	else if (&gb == auxGCode)
	{
		// Aux serial port (typically PanelDue)
		for (unsigned int i = 0; i < 16 && platform->GCodeAvailable(SerialSource::AUX); ++i)
		{
			char b = platform->ReadFromSource(SerialSource::AUX);
			if (gb.Put(b))	// add char to buffer and test whether the gcode is complete
			{
				platform->SetAuxDetected();
				gb.SetFinished(ActOnCode(gb, reply));
				break;
			}
		}
	}
}

void GCodes::DoFilePrint(GCodeBuffer& gb, StringRef& reply)
{
	FileData& fd = gb.MachineState().fileState;
	for (int i = 0; i < 50 && fd.IsLive(); ++i)
	{
		char b;
		if (fd.Read(b))
		{
			if (gb.Put(b))
			{
				gb.SetFinished(ActOnCode(gb, reply));
				return;
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

			// Don't close the file until all moves have been completed, in case the print gets paused.
			// Also, this keeps the state as 'Printing' until the print really has finished.
			if (LockMovementAndWaitForStandstill(gb))
			{
				fd.Close();
				if (gb.MachineState().previous == nullptr)
				{
					// Finished printing SD card file
					UnlockAll(gb);
					reprap.GetPrintMonitor()->StoppedPrint();
					if (platform->Emulating() == marlin)
					{
						// Pronterface expects a "Done printing" message
						HandleReply(gb, false, "Done printing file");
					}
				}
				else
				{
					// Finished a macro or finished processing config.g
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
					}
				}
			}
			return;
		}
	}
}

// Check for and execute triggers
void GCodes::CheckTriggers()
{
	// Check for endstop state changes that activate new triggers
	const TriggerMask oldEndstopStates = lastEndstopStates;
	lastEndstopStates = platform->GetAllEndstopStates();
	const TriggerMask risen = lastEndstopStates & ~oldEndstopStates,
					  fallen = ~lastEndstopStates & oldEndstopStates;
	unsigned int lowestTriggerPending = MaxTriggers;
	for (unsigned int triggerNumber = 0; triggerNumber < MaxTriggers; ++triggerNumber)
	{
		const Trigger& ct = triggers[triggerNumber];
		if (   ((ct.rising & risen) != 0 || (ct.falling & fallen) != 0)
			&& (ct.condition == 0 || (ct.condition == 1 && reprap.GetPrintMonitor()->IsPrinting()))
		   )
		{
			triggersPending |= (1u << triggerNumber);
		}
		if (triggerNumber < lowestTriggerPending && (triggersPending & (1u << triggerNumber)) != 0)
		{
			lowestTriggerPending = triggerNumber;
		}
	}

	// If any triggers are pending, activate the one with the lowest number
	if (lowestTriggerPending == 0)
	{
		triggersPending &= ~(1u << lowestTriggerPending);			// clear the trigger
		DoEmergencyStop();
	}
	else if (lowestTriggerPending < MaxTriggers						// if a trigger is pending
			 && !IsDaemonBusy()
			 && daemonGCode->GetState() == GCodeState::normal		// and we are not already executing a trigger or config.g
			)
	{
		if (lowestTriggerPending == 1)
		{
			if (isPaused || !reprap.GetPrintMonitor()->IsPrinting())
			{
				triggersPending &= ~(1u << lowestTriggerPending);	// ignore a pause trigger if we are already paused
			}
			else if (LockMovement(*daemonGCode))					// need to lock movement before executing the pause macro
			{
				triggersPending &= ~(1u << lowestTriggerPending);	// clear the trigger
				DoPause(*daemonGCode);
			}
		}
		else
		{
			triggersPending &= ~(1u << lowestTriggerPending);		// clear the trigger
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
	platform->Message(GENERIC_MESSAGE, "Emergency Stop! Reset the controller to continue.");
}

// Pause the print. Before calling this, check that we are doing a file print that isn't already paused and get the movement lock.
void GCodes::DoPause(GCodeBuffer& gb)
{
	if (&gb == fileGCode)
	{
		// Pausing a file print because of a command in the file itself
		for (size_t drive = 0; drive < numAxes; ++drive)
		{
			pauseRestorePoint.moveCoords[drive] = moveBuffer.coords[drive];
		}
		for (size_t drive = numAxes; drive < DRIVES; ++drive)
		{
			pauseRestorePoint.moveCoords[drive] = lastRawExtruderPosition[drive - numAxes];	// get current extruder positions into pausedMoveBuffer
		}
		pauseRestorePoint.feedRate = gb.MachineState().feedrate;
	}
	else
	{
		// Pausing a file print via another input source
		pauseRestorePoint.feedRate = fileGCode->MachineState().feedrate;			// the call to PausePrint may or may not change this
		FilePosition fPos = reprap.GetMove()->PausePrint(pauseRestorePoint.moveCoords, pauseRestorePoint.feedRate, reprap.GetCurrentXAxes());
																					// tell Move we wish to pause the current print
		FileData& fdata = fileGCode->MachineState().fileState;
		if (fPos != noFilePosition && fdata.IsLive())
		{
			fdata.Seek(fPos);														// replay the abandoned instructions if/when we resume
		}
		if (segmentsLeft != 0)
		{
			for (size_t drive = numAxes; drive < DRIVES; ++drive)
			{
				pauseRestorePoint.moveCoords[drive] += moveBuffer.coords[drive];	// add on the extrusion in the move not yet taken
			}
			ClearMove();
		}

		for (size_t drive = numAxes; drive < DRIVES; ++drive)
		{
			pauseRestorePoint.moveCoords[drive] = lastRawExtruderPosition[drive - numAxes] - pauseRestorePoint.moveCoords[drive];
		}

		//TODO record the virtual extruder positions of mixing tools too. But that's very hard to do unless we store it in the move.

		if (reprap.Debug(moduleGcodes))
		{
			platform->MessageF(GENERIC_MESSAGE, "Paused print, file offset=%u\n", fPos);
		}
	}

	for (size_t i = 0; i < NUM_FANS; ++i)
	{
		pausedFanValues[i] = platform->GetFanValue(i);
	}
	gb.SetState(GCodeState::pausing1);
	isPaused = true;
}

void GCodes::Diagnostics(MessageType mtype)
{
	platform->Message(mtype, "=== GCodes ===\n");
	platform->MessageF(mtype, "Segments left: %u\n", segmentsLeft);
	platform->MessageF(mtype, "Stack records: %u allocated, %u in use\n", GCodeMachineState::GetNumAllocated(), GCodeMachineState::GetNumInUse());
	const GCodeBuffer *movementOwner = resourceOwners[MoveResource];
	platform->MessageF(mtype, "Movement lock held by %s\n", (movementOwner == nullptr) ? "null" : movementOwner->GetIdentity());

	for (size_t i = 0; i < ARRAY_SIZE(gcodeSources); ++i)
	{
		gcodeSources[i]->Diagnostics(mtype);
	}
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
	if (!reprap.GetMove()->AllMovesAreFinished())
	{
		return false;
	}

	// Allow movement again
	reprap.GetMove()->ResumeMoving();

	// Get the current positions. These may not be the same as the ones we remembered from last time if we just did a special move.
	reprap.GetMove()->GetCurrentUserPosition(moveBuffer.coords, 0, reprap.GetCurrentXAxes());
	memcpy(moveBuffer.initialCoords, moveBuffer.coords, numAxes * sizeof(moveBuffer.initialCoords[0]));
	return true;
}

// Save (some of) the state of the machine for recovery in the future.
bool GCodes::Push(GCodeBuffer& gb)
{
	const bool ok = gb.PushState();
	if (!ok)
	{
		platform->Message(GENERIC_MESSAGE, "Push(): stack overflow!\n");
	}
	return ok;
}

// Recover a saved state
void GCodes::Pop(GCodeBuffer& gb)
{
	if (!gb.PopState())
	{
		platform->Message(GENERIC_MESSAGE, "Pop(): stack underflow!\n");
	}
}

// Set up the extrusion and feed rate of a move for the Move class
// 'moveType' is the S parameter in the G0 or G1 command, or zero for a G2 or G3 command, or -1 for a G92 command
// Returns true if this gcode is valid so far, false if it should be discarded
bool GCodes::LoadExtrusionAndFeedrateFromGCode(GCodeBuffer& gb, int moveType)
{
	// Zero every extruder drive as some drives may not be moved
	for (size_t drive = numAxes; drive < DRIVES; drive++)
	{
		moveBuffer.coords[drive] = 0.0;
	}

	// Deal with feed rate
	if (moveType >= 0 && gb.Seen(feedrateLetter))
	{
		const float rate = gb.GetFValue() * distanceScale;
		gb.MachineState().feedrate = (moveType == 0)
										? rate * speedFactor
										: rate * SecondsToMinutes;		// don't apply the speed factor to homing and other special moves
	}
	moveBuffer.feedRate = gb.MachineState().feedrate;

	// First do extrusion, and check, if we are extruding, that we have a tool to extrude with
	if (gb.Seen(extrudeLetter))
	{
		Tool* const tool = reprap.GetCurrentTool();
		if (tool == nullptr)
		{
			platform->Message(GENERIC_MESSAGE, "Attempting to extrude with no tool selected.\n");
			return false;
		}
		const size_t eMoveCount = tool->DriveCount();
		if (eMoveCount > 0)
		{
			// Set the drive values for this tool.
			// chrishamm-2014-10-03: Do NOT check extruder temperatures here, because we may be executing queued codes like M116
			if (tool->GetMixing())
			{
				const float moveArg = gb.GetFValue() * distanceScale;
				if (moveType == -1)			// if doing G92
				{
					tool->virtualExtruderPosition = moveArg;
				}
				else
				{
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
						moveBuffer.coords[drive + numAxes] = extrusionAmount * extrusionFactors[drive];
					}
				}
			}
			else
			{
				float eMovement[MaxExtruders];
				size_t mc = eMoveCount;
				gb.GetFloatArray(eMovement, mc, false);
				if (eMoveCount != mc)
				{
					platform->MessageF(GENERIC_MESSAGE, "Wrong number of extruder drives for the selected tool: %s\n", gb.Buffer());
					return false;
				}

				for (size_t eDrive = 0; eDrive < eMoveCount; eDrive++)
				{
					const int drive = tool->Drive(eDrive);
					const float moveArg = eMovement[eDrive] * distanceScale;
					if (moveType == -1)
					{
						lastRawExtruderPosition[drive] = moveArg;
					}
					else
					{
						const float extrusionAmount = (gb.MachineState().drivesRelative)
													? moveArg
													: moveArg - lastRawExtruderPosition[drive];
						lastRawExtruderPosition[drive] += extrusionAmount;
						rawExtruderTotalByDrive[drive] += extrusionAmount;
						rawExtruderTotal += extrusionAmount;
						moveBuffer.coords[drive + numAxes] = extrusionAmount * extrusionFactors[drive];
					}
				}
			}
		}
	}
	return true;
}

// Set up the axis coordinates of a move for the Move class
// Move expects all axis movements to be absolute, and all extruder drive moves to be relative.  This function serves that.
// 'moveType' is the S parameter in the G0 or G1 command, or -1 if we are doing G92.
// For regular (type 0) moves, we apply limits and do X axis mapping.
// Returns the number of segments in the move
unsigned int GCodes::LoadMoveBufferFromGCode(GCodeBuffer& gb, int moveType)
{
	const Tool * const currentTool = reprap.GetCurrentTool();
	unsigned int numSegments = 1;
	for (size_t axis = 0; axis < numAxes; axis++)
	{
		if (gb.Seen(axisLetters[axis]))
		{
			float moveArg = gb.GetFValue() * distanceScale * axisScaleFactors[axis];
			if (moveType == -1)						// if doing G92
			{
				SetAxisIsHomed(axis);				// doing a G92 defines the absolute axis position
				moveBuffer.coords[axis] = moveArg;
			}
			else if (axis == X_AXIS && moveType == 0 && currentTool != nullptr)
			{
				// Perform X axis mapping
				const uint32_t xMap = currentTool->GetXAxisMap();
				for (size_t mappedAxis = 0; mappedAxis < numAxes; ++mappedAxis)
				{
					if ((xMap & (1u << mappedAxis)) != 0)
					{
						float mappedMoveArg = moveArg;
						if (gb.MachineState().axesRelative)
						{
							mappedMoveArg += moveBuffer.coords[mappedAxis];
						}
						else
						{
							mappedMoveArg -= currentTool->GetOffset()[mappedAxis];	// adjust requested position to compensate for tool offset
						}
						const HeightMap& heightMap = reprap.GetMove()->AccessBedProbeGrid();
						if (heightMap.UsingHeightMap())
						{
							const unsigned int minSegments = heightMap.GetMinimumSegments(fabs(mappedMoveArg - moveBuffer.coords[mappedAxis]));
							if (minSegments > numSegments)
							{
								numSegments = minSegments;
							}
						}
						moveBuffer.coords[mappedAxis] = mappedMoveArg;
					}
				}
			}
			else
			{
				if (gb.MachineState().axesRelative)
				{
					moveArg += moveBuffer.coords[axis];
				}
				else if (moveType == 0)
				{
					if (axis == Z_AXIS && isRetracted)
					{
						moveArg += retractHop;						// handle firmware retraction on layer change
					}
					if (currentTool != nullptr)
					{
						moveArg -= currentTool->GetOffset()[axis];	// adjust requested position to compensate for tool offset
					}
				}

				if (axis != Z_AXIS && moveType == 0)
				{
					// Segment the move if necessary
					const HeightMap& heightMap = reprap.GetMove()->AccessBedProbeGrid();
					if (heightMap.UsingHeightMap())
					{
						const unsigned int minSegments = reprap.GetMove()->AccessBedProbeGrid().GetMinimumSegments(fabs(moveArg - moveBuffer.coords[axis]));
						if (minSegments > numSegments)
						{
							numSegments = minSegments;
						}
					}
				}
				moveBuffer.coords[axis] = moveArg;
			}
		}
	}

	// If doing a regular move and applying limits, limit all axes
	if (   moveType == 0
		&& limitAxes
#if SUPPORT_ROLAND
		&& !reprap.GetRoland()->Active()
#endif
	   )
	{
		if (!reprap.GetMove()->IsDeltaMode())
		{
			// Cartesian or CoreXY printer, so limit those axes that have been homed
			for (size_t axis = 0; axis < numAxes; axis++)
			{
				if (GetAxisIsHomed(axis))
				{
					float& f = moveBuffer.coords[axis];
					if (f < platform->AxisMinimum(axis))
					{
						f = platform->AxisMinimum(axis);
					}
					else if (f > platform->AxisMaximum(axis))
					{
						f = platform->AxisMaximum(axis);
					}
				}
			}
		}
		else if (AllAxesAreHomed())			// this is to allow extruder-only moves before homing
		{
			// If axes have been homed on a delta printer and this isn't a homing move, check for movements outside limits.
			// Skip this check if axes have not been homed, so that extruder-only moved are allowed before homing
			// Constrain the move to be within the build radius
			const float diagonalSquared = fsquare(moveBuffer.coords[X_AXIS]) + fsquare(moveBuffer.coords[Y_AXIS]);
			if (diagonalSquared > reprap.GetMove()->GetDeltaParams().GetPrintRadiusSquared())
			{
				const float factor = sqrtf(reprap.GetMove()->GetDeltaParams().GetPrintRadiusSquared() / diagonalSquared);
				moveBuffer.coords[X_AXIS] *= factor;
				moveBuffer.coords[Y_AXIS] *= factor;
			}

			// Constrain the end height of the move to be no greater than the homed height and no lower than -0.2mm
			moveBuffer.coords[Z_AXIS] = max<float>(platform->AxisMinimum(Z_AXIS),
					min<float>(moveBuffer.coords[Z_AXIS], reprap.GetMove()->GetDeltaParams().GetHomedHeight()));
		}
	}

	return numSegments;
}

// This function is called for a G Code that makes a move.
// If the Move class can't receive the move (i.e. things have to wait), return 0.
// If we have queued the move and the caller doesn't need to wait for it to complete, return 1.
// If we need to wait for the move to complete before doing another one (e.g. because endstops are checked in this move), return 2.
int GCodes::SetUpMove(GCodeBuffer& gb, StringRef& reply)
{
	// Last one gone yet?
	if (segmentsLeft != 0)
	{
		return 0;
	}

	// Check to see if the move is a 'homing' move that endstops are checked on.
	moveBuffer.endStopsToCheck = 0;
	moveBuffer.moveType = 0;
	doingArcMove = false;
	moveBuffer.xAxes = reprap.GetCurrentXAxes();
	if (gb.Seen('S'))
	{
		int ival = gb.GetIValue();
		if (ival == 1 || ival == 2)
		{
			moveBuffer.moveType = ival;
			moveBuffer.xAxes = 0;			// don't do bed compensation
		}

		if (ival == 1)
		{
			for (size_t i = 0; i < numAxes; ++i)
			{
				if (gb.Seen(axisLetters[i]))
				{
					moveBuffer.endStopsToCheck |= (1u << i);
				}
			}
		}
		else if (ival == 99)		// temporary code to log Z probe change positions
		{
			moveBuffer.endStopsToCheck |= LogProbeChanges;
		}
	}

	if (reprap.GetMove()->IsDeltaMode())
	{
		// Extra checks to avoid damaging delta printers
		if (moveBuffer.moveType != 0 && !gb.MachineState().axesRelative)
		{
			// We have been asked to do a move without delta mapping on a delta machine, but the move is not relative.
			// This may be damaging and is almost certainly a user mistake, so ignore the move.
			reply.copy("Attempt to move the motors of a delta printer to absolute positions");
			return 1;
		}

		if (moveBuffer.moveType == 0 && !AllAxesAreHomed())
		{
			// The user may be attempting to move a delta printer to an XYZ position before homing the axes
			// This may be damaging and is almost certainly a user mistake, so ignore the move. But allow extruder-only moves.
			if (gb.Seen(axisLetters[X_AXIS]) || gb.Seen(axisLetters[Y_AXIS]) || gb.Seen(axisLetters[Z_AXIS]))
			{
				reply.copy("Attempt to move the head of a delta printer before homing the towers");
				return 1;
			}
		}
	}

	// Load the last position into moveBuffer
#if SUPPORT_ROLAND
	if (reprap.GetRoland()->Active())
	{
		reprap.GetRoland()->GetCurrentRolandPosition(moveBuffer);
	}
	else
#endif
	{
		reprap.GetMove()->GetCurrentUserPosition(moveBuffer.coords, moveBuffer.moveType, reprap.GetCurrentXAxes());
	}

	// Load the move buffer with either the absolute movement required or the relative movement required
	memcpy(moveBuffer.initialCoords, moveBuffer.coords, numAxes * sizeof(moveBuffer.initialCoords[0]));
	if (LoadExtrusionAndFeedrateFromGCode(gb, moveBuffer.moveType))
	{
		segmentsLeft = LoadMoveBufferFromGCode(gb, moveBuffer.moveType);
		if (segmentsLeft != 0)
		{
			// Flag whether we should use pressure advance, if there is any extrusion in this move.
			// We assume it is a normal printing move needing pressure advance if there is forward extrusion and XYU.. movement.
			// The movement code will only apply pressure advance if there is forward extrusion, so we only need to check for XYU.. movement here.
			moveBuffer.usePressureAdvance = false;
			for (size_t axis = 0; axis < numAxes; ++axis)
			{
				if (axis != Z_AXIS && moveBuffer.coords[axis] != moveBuffer.initialCoords[axis])
				{
					moveBuffer.usePressureAdvance = true;
					break;
				}
			}
			moveBuffer.filePos = (&gb == fileGCode) ? gb.MachineState().fileState.GetPosition() : noFilePosition;
			moveBuffer.canPauseAfter = (moveBuffer.endStopsToCheck == 0);
			//debugPrintf("Queue move pos %u\n", moveFilePos);
		}
	}
	return (moveBuffer.moveType != 0 || moveBuffer.endStopsToCheck != 0) ? 2 : 1;
}

// Execute an arc move returning true if it was badly-formed
// We already have the movement lock and the last move has gone
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

	// Adjust them for relative/absolute coordinates, tool offset, and X axis mapping. Also get the optional Z parameter
	const Tool * const currentTool = reprap.GetCurrentTool();
	const bool axesRelative = gb.MachineState().axesRelative;
	memcpy(moveBuffer.initialCoords, moveBuffer.coords, numAxes * sizeof(moveBuffer.initialCoords[0]));

	if (gb.Seen('Z'))
	{
		const float zParam = gb.GetFValue() * distanceScale;
		if (axesRelative)
		{
			moveBuffer.coords[Z_AXIS] += zParam;
		}
		else
		{
			moveBuffer.coords[Z_AXIS] = zParam;
			if (currentTool != nullptr)
			{
				moveBuffer.coords[Z_AXIS] -= currentTool->GetOffset()[Z_AXIS];
			}
		}
	}

	// The I and J parameters are always relative to present position
	arcCentre[Y_AXIS] = moveBuffer.initialCoords[Y_AXIS] + jParam;

	if (currentTool != nullptr)
	{
		// Record which axes behave like an X axis
		arcAxesMoving = currentTool->GetXAxisMap() & ~((1 << Y_AXIS) | (1 << Z_AXIS));

		// Sort out the Y axis
		if (axesRelative)
		{
			moveBuffer.coords[Y_AXIS] += yParam;
		}
		else
		{
			moveBuffer.coords[Y_AXIS] = yParam - currentTool->GetOffset()[Y_AXIS];
		}

		// Deal with the X axes
		for (size_t axis = 0; axis < numAxes; ++axis)
		{
			arcCentre[axis] = moveBuffer.initialCoords[axis] + iParam;
			if ((arcAxesMoving & (1 << axis)) != 0)
			{
				if (axesRelative)
				{
					moveBuffer.coords[axis] += xParam;
				}
				else
				{
					moveBuffer.coords[axis] = xParam - currentTool->GetOffset()[axis];
				}
			}
		}
	}
	else
	{
		arcAxesMoving = (1 << X_AXIS);
		arcCentre[X_AXIS] = moveBuffer.initialCoords[X_AXIS] + iParam;
		if (axesRelative)
		{
			moveBuffer.coords[X_AXIS] += xParam;
			moveBuffer.coords[Y_AXIS] += yParam;
		}
		else
		{
			moveBuffer.coords[X_AXIS] = xParam;
			moveBuffer.coords[Y_AXIS] = yParam;
		}
	}

	moveBuffer.endStopsToCheck = 0;
	moveBuffer.moveType = 0;
	moveBuffer.xAxes = reprap.GetCurrentXAxes();
	if (LoadExtrusionAndFeedrateFromGCode(gb, moveBuffer.moveType))		// this reports an error if necessary, so no need to return true if it fails
	{
		arcRadius = sqrtf(iParam * iParam + jParam * jParam);
		arcCurrentAngle = atan2(-jParam, -iParam);
		const float finalTheta = atan2(moveBuffer.coords[Y_AXIS] - arcCentre[Y_AXIS], moveBuffer.coords[X_AXIS] - arcCentre[X_AXIS]);

		// Calculate the total angle moved, which depends on which way round we are going
		float totalArc = (clockwise) ? arcCurrentAngle - finalTheta : finalTheta - arcCurrentAngle;
		if (totalArc < 0)
		{
			totalArc += 2 * PI;
		}
		segmentsLeft = max<unsigned int>((unsigned int)((arcRadius * totalArc)/arcSegmentLength + 0.8), 1);
		arcAngleIncrement = totalArc/segmentsLeft;
		if (clockwise)
		{
			arcAngleIncrement = -arcAngleIncrement;
		}
		doingArcMove = true;
		moveBuffer.usePressureAdvance = true;
//		debugPrintf("Radius %.2f, initial angle %.1f, increment %.1f, segments %u\n",
//				arcRadius, arcCurrentAngle * RadiansToDegrees, arcAngleIncrement * RadiansToDegrees, segmentsLeft);
	}
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
		// This move needs to be divided into 2 or more segments. We can only pause after the final segment.
		m.canPauseAfter = false;

		// Do the axes
		if (doingArcMove)
		{
			arcCurrentAngle += arcAngleIncrement;
		}

		for (size_t drive = 0; drive < numAxes; ++drive)
		{
			if (doingArcMove && drive != Z_AXIS)
			{
				if (drive == Y_AXIS)
				{
					moveBuffer.initialCoords[drive] = arcCentre[drive] + arcRadius * sinf(arcCurrentAngle);
				}
				else if ((arcAxesMoving & (1 << drive)) != 0)
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
		for (size_t drive = numAxes; drive < DRIVES; ++drive)
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
	segmentsLeft = 0;
	doingArcMove = false;
	moveBuffer.endStopsToCheck = 0;
	moveBuffer.moveType = 0;
	moveBuffer.isFirmwareRetraction = false;
}

// Run a file macro. Prior to calling this, 'state' must be set to the state we want to enter when the macro has been completed.
// Return true if the file was found or it wasn't and we were asked to report that fact.
bool GCodes::DoFileMacro(GCodeBuffer& gb, const char* fileName, bool reportMissing, bool runningM502)
{
	FileStore * const f = platform->GetFileStore(platform->GetSysDir(), fileName, false);
	if (f == nullptr)
	{
		if (reportMissing)
		{
			// Don't use snprintf into scratchString here, because fileName may be aliased to scratchString
			platform->MessageF(GENERIC_MESSAGE, "Macro file %s not found.\n", fileName);
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
	if (gb.MachineState().doingFileMacro)
	{
		gb.PopState();
		gb.Init();
	}
}

// To execute any move, call this until it returns true.
// There is only one copy of the canned cycle variable so you must acquire the move lock before calling this.
bool GCodes::DoCannedCycleMove(GCodeBuffer& gb, EndstopChecks ce)
{
	if (LockMovementAndWaitForStandstill(gb))
	{
		if (cannedCycleMoveQueued)		// if the move has already been queued, it must have finished
		{
			Pop(gb);
			cannedCycleMoveQueued = false;
			return true;
		}

		// Otherwise, the move has not been queued yet
		if (!Push(gb))
		{
			return true;				// stack overflow
		}
		gb.MachineState().state = gb.MachineState().previous->state;	// stay in the same state

		for (size_t drive = 0; drive < DRIVES; drive++)
		{
			switch(cannedMoveType[drive])
			{
			case CannedMoveType::none:
				break;
			case CannedMoveType::relative:
				moveBuffer.coords[drive] += cannedMoveCoords[drive];
				break;
			case CannedMoveType::absolute:
				moveBuffer.coords[drive] = cannedMoveCoords[drive];
				break;
			}
		}
		moveBuffer.feedRate = cannedFeedRate;
		moveBuffer.xAxes = 0;
		moveBuffer.endStopsToCheck = ce;
		moveBuffer.filePos = noFilePosition;
		moveBuffer.usePressureAdvance = false;
		segmentsLeft = 1;
		cannedCycleMoveQueued = true;
		if ((ce & ZProbeActive) != 0)
		{
			platform->SetProbing(true);
		}
	}
	return false;
}

// This handles G92. Return true if completed, false if it needs to be called again.
bool GCodes::SetPositions(GCodeBuffer& gb)
{
	if (gb.Seen('R') && gb.GetIValue() == 1)
	{
		// Babystepping command. All coordinates except Z are ignored.
		if (gb.Seen('Z'))
		{
			const float babystepAmount = gb.GetFValue() * distanceScale;
			if (fabs(babystepAmount) <= 1.0)		// limit babystepping to 1mm
			{
				reprap.GetMove()->Babystep(babystepAmount);
			}
		}
	}
	else
	{
		// Don't pause the machine if only extruder drives are being reset (DC, 2015-09-06).
		// This avoids blobs and seams when the gcode uses absolute E coordinates and periodically includes G92 E0.
		bool includingAxes = false;
		for (size_t drive = 0; drive < numAxes; ++drive)
		{
			if (gb.Seen(axisLetters[drive]))
			{
				includingAxes = true;
				break;
			}
		}

		if (includingAxes)
		{
			if (!LockMovementAndWaitForStandstill(gb))	// lock movement and get current coordinates
			{
				return false;
			}
		}
		else if (segmentsLeft != 0)			// wait for previous move to be taken so that GetCurrentUserPosition returns the correct value
		{
			return false;
		}

		// Handle any E parameter in the G92 command. If we get an error, ignore it and do the axes anyway.
		(void)LoadExtrusionAndFeedrateFromGCode(gb, -1);

		if (includingAxes)
		{
			(void)LoadMoveBufferFromGCode(gb, -1);

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
			SetPositions(moveBuffer.coords);
		}
	}
	return true;
}

// Offset the axes by the X, Y, and Z amounts in the M code in gb.  Say the machine is at [10, 20, 30] and
// the offsets specified are [8, 2, -5].  The machine will move to [18, 22, 25] and henceforth consider that point
// to be [10, 20, 30].
bool GCodes::OffsetAxes(GCodeBuffer& gb)
{
	if (!offSetSet)
	{
		if (!LockMovementAndWaitForStandstill(gb))
		{
			return false;
		}
		for (size_t drive = 0; drive < DRIVES; drive++)
		{
			cannedMoveType[drive] = CannedMoveType::none;
			if (drive < numAxes)
			{
				record[drive] = moveBuffer.coords[drive];
				if (gb.Seen(axisLetters[drive]))
				{
					cannedMoveCoords[drive] = gb.GetFValue() * distanceScale;
					cannedMoveType[drive] = CannedMoveType::relative;
				}
			}
			else
			{
				record[drive] = 0.0;
			}
		}

		if (gb.Seen(feedrateLetter)) // Has the user specified a feedrate?
		{
			cannedFeedRate = gb.GetFValue() * distanceScale * SecondsToMinutes;
		}
		else
		{
			cannedFeedRate = DefaultFeedrate;
		}

		offSetSet = true;
	}

	if (DoCannedCycleMove(gb, 0))
	{
		// Restore positions
		for (size_t drive = 0; drive < DRIVES; drive++)
		{
			moveBuffer.coords[drive] = record[drive];
		}
		reprap.GetMove()->SetLiveCoordinates(record);	// This doesn't transform record
		reprap.GetMove()->SetPositions(record);			// This does
		offSetSet = false;
		return true;
	}

	return false;
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

	if (reprap.GetMove()->IsDeltaMode())
	{
		SetAllAxesNotHomed();
		DoFileMacro(gb, HOME_DELTA_G, true);
	}
	else
	{
		toBeHomed = 0;
		for (size_t axis = 0; axis < numAxes; ++axis)
		{
			if (gb.Seen(axisLetters[axis]))
			{
				toBeHomed |= (1u << axis);
				SetAxisNotHomed(axis);
			}
		}

		if (toBeHomed == 0 || toBeHomed == ((1u << numAxes) - 1))
		{
			// Homing everything
			SetAllAxesNotHomed();
			DoFileMacro(gb, HOME_ALL_G, true);
		}
		else if (   platform->MustHomeXYBeforeZ()
				 && ((toBeHomed & (1u << Z_AXIS)) != 0)
				 && ((toBeHomed | axesHomed | (1u << Z_AXIS)) != ((1u << numAxes) - 1))
				)
		{
			// We can only home Z if both X and Y have already been homed or are being homed
			reply.copy("Must home all other axes before homing Z");
			error = true;
		}
		else
		{
			gb.SetState(GCodeState::homing);
		}
	}
	return true;
}

// This lifts Z a bit, moves to the probe XY coordinates (obtained by a call to GetProbeCoordinates() ),
// probes the bed height, and records the Z coordinate probed.  If you want to program any general
// internal canned cycle, this shows how to do it.
// On entry, probePointIndex specifies which of the points this is.
bool GCodes::DoSingleZProbeAtPoint(GCodeBuffer& gb, size_t probePointIndex, float heightAdjust)
{
	reprap.GetMove()->SetIdentityTransform(); 		// It doesn't matter if these are called repeatedly

	for (size_t drive = 0; drive < DRIVES; drive++)
	{
		cannedMoveType[drive] = CannedMoveType::none;
	}

	switch (cannedCycleMoveCount)
	{
	case 0: // Move Z to the dive height. This only does anything on the first move; on all the others Z is already there
		cannedMoveCoords[Z_AXIS] = platform->GetZProbeStartingHeight();
		cannedMoveType[Z_AXIS] = CannedMoveType::absolute;
		cannedFeedRate = platform->GetZProbeTravelSpeed();
		if (DoCannedCycleMove(gb, 0))
		{
			cannedCycleMoveCount++;
		}
		return false;

	case 1:	// Move to the correct XY coordinates
		(void)reprap.GetMove()->GetProbeCoordinates(probePointIndex, cannedMoveCoords[X_AXIS], cannedMoveCoords[Y_AXIS], true);
		cannedMoveType[X_AXIS] = CannedMoveType::absolute;
		cannedMoveType[Y_AXIS] = CannedMoveType::absolute;
		// NB - we don't use the Z value
		cannedFeedRate = platform->GetZProbeTravelSpeed();
		if (DoCannedCycleMove(gb, 0))
		{
			lastProbedTime = millis();
			cannedCycleMoveCount++;
		}
		return false;

	case 2:	// Probe the bed
		if (millis() - lastProbedTime >= (uint32_t)(platform->GetCurrentZProbeParameters().recoveryTime * SecondsToMillis))
		{
			const float height = (GetAxisIsHomed(Z_AXIS))
									? 2 * platform->GetZProbeDiveHeight()			// Z axis has been homed, so no point in going very far
									: 1.1 * platform->AxisTotalLength(Z_AXIS);		// Z axis not homed yet, so treat this as a homing move
			switch(DoZProbe(gb, height))
			{
			case 0:
				// Z probe is already triggered at the start of the move, so abandon the probe and record an error
				platform->Message(GENERIC_MESSAGE, "Error: Z probe already triggered at start of probing move\n");
				cannedCycleMoveCount++;
				reprap.GetMove()->SetZBedProbePoint(probePointIndex, platform->GetZProbeDiveHeight(), true, true);
				break;

			case 1:
				// Z probe did not trigger
				platform->Message(GENERIC_MESSAGE, "Error: Z probe was not triggered during probing move\n");
				cannedCycleMoveCount++;
				reprap.GetMove()->SetZBedProbePoint(probePointIndex, -(platform->GetZProbeDiveHeight()), true, true);
				break;

			case 2:
				// Successful probing
				if (GetAxisIsHomed(Z_AXIS))
				{
					lastProbedZ = moveBuffer.coords[Z_AXIS] - (platform->ZProbeStopHeight() + heightAdjust);
				}
				else
				{
					// The Z axis has not yet been homed, so treat this probe as a homing move.
					moveBuffer.coords[Z_AXIS] = platform->ZProbeStopHeight() + heightAdjust;
					SetPositions(moveBuffer.coords);
					SetAxisIsHomed(Z_AXIS);
					lastProbedZ = 0.0;
				}
				reprap.GetMove()->SetZBedProbePoint(probePointIndex, lastProbedZ, true, false);
				cannedCycleMoveCount++;
				break;

			default:
				break;
			}
		}
		return false;

	case 3:	// Raise the head back up to the dive height
		cannedMoveCoords[Z_AXIS] = platform->GetZProbeStartingHeight();
		cannedMoveType[Z_AXIS] = CannedMoveType::absolute;
		cannedFeedRate = platform->GetZProbeTravelSpeed();
		if (DoCannedCycleMove(gb, 0))
		{
			cannedCycleMoveCount = 0;
			return true;
		}
		return false;

	default: // should not happen
		cannedCycleMoveCount = 0;
		return true;
	}
}

// This simply moves down till the Z probe/switch is triggered. Call it repeatedly until it returns true.
// Called when we do a G30 with no P parameter.
bool GCodes::DoSingleZProbe(GCodeBuffer& gb, StringRef& reply, bool reportOnly, float heightAdjust)
{
	switch (DoZProbe(gb, 1.1 * platform->AxisTotalLength(Z_AXIS)))
	{
	case 0:		// failed
		platform->Message(GENERIC_MESSAGE, "Error: Z probe already triggered at start of probing move\n");
		return true;

	case 1:
		platform->Message(GENERIC_MESSAGE, "Error: Z probe was not triggered during probing move\n");
		return true;

	case 2:		// success
		if (reportOnly)
		{
			float m[DRIVES];
			reprap.GetMove()->GetCurrentMachinePosition(m, false);
			reply.printf("Stopped at height %.3f mm", m[Z_AXIS]);
		}
		else
		{
			moveBuffer.coords[Z_AXIS] = platform->ZProbeStopHeight() + heightAdjust;
			SetPositions(moveBuffer.coords, false);		// set positions WITHOUT (very important) applying bed compensation
			SetAxisIsHomed(Z_AXIS);
			reprap.GetMove()->GetCurrentUserPosition(moveBuffer.coords, 0, reprap.GetCurrentXAxes());	// update the user position
			lastProbedZ = 0.0;
		}
		return true;

	default:	// not finished yet
		return false;
	}
}

// Do a Z probe cycle up to the maximum specified distance.
// Returns -1 if not complete yet
// Returns 0 if Z probe already triggered at start of probing
// Returns 1 if Z probe didn't trigger
// Returns 2 if success, with the current position in moveBuffer
int GCodes::DoZProbe(GCodeBuffer& gb, float distance)
{
	if (platform->GetZProbeType() == ZProbeTypeDelta)
	{
		const ZProbeParameters& params = platform->GetCurrentZProbeParameters();
		return reprap.GetMove()->DoDeltaProbe(params.extraParam, 1.0, params.probeSpeed, distance);	//TODO second parameter
	}
	else
	{
		// Check for probe already triggered at start
		if (!cannedCycleMoveQueued)
		{
			if (reprap.GetPlatform()->GetZProbeResult() == EndStopHit::lowHit)
			{
				return 0;
			}
			zProbeTriggered = false;
		}

		// Do a normal canned cycle Z movement with Z probe enabled
		for (size_t drive = 0; drive < DRIVES; drive++)
		{
			cannedMoveType[drive] = CannedMoveType::none;
		}

		cannedMoveCoords[Z_AXIS] = -distance;
		cannedMoveType[Z_AXIS] = CannedMoveType::relative;
		cannedFeedRate = platform->GetCurrentZProbeParameters().probeSpeed;

		if (DoCannedCycleMove(gb, ZProbeActive))
		{
			platform->SetProbing(false);
			return (zProbeTriggered) ? 2 : 1;
		}
		return -1;
	}
}

// This is called to execute a G30.
// It sets wherever we are as the probe point P (probePointIndex)
// then probes the bed, or gets all its parameters from the arguments.
// If X or Y are specified, use those; otherwise use the machine's
// coordinates.  If no Z is specified use the machine's coordinates.  If it
// is specified and is greater than SILLY_Z_VALUE (i.e. greater than -9999.0)
// then that value is used.  If it's less than SILLY_Z_VALUE the bed is
// probed and that value is used.
// Call this repeatedly until it returns true.
bool GCodes::SetSingleZProbeAtAPosition(GCodeBuffer& gb, StringRef& reply)
{
	if (reprap.GetMove()->IsDeltaMode() && !AllAxesAreHomed())
	{
		reply.copy("Must home before bed probing");
		return true;
	}

	float heightAdjust = 0.0;
	bool dummy;
	gb.TryGetFValue('H', heightAdjust, dummy);

	if (!gb.Seen('P'))
	{
		const bool reportOnly = (gb.Seen('S') && gb.GetIValue() < 0);
		return DoSingleZProbe(gb, reply, reportOnly, heightAdjust);
	}

	const int probePointIndex = gb.GetIValue();
	if (probePointIndex < 0 || (unsigned int)probePointIndex >= MaxProbePoints)
	{
		reprap.GetPlatform()->Message(GENERIC_MESSAGE, "Z probe point index out of range.\n");
		return true;
	}

	const float x = (gb.Seen(axisLetters[X_AXIS])) ? gb.GetFValue() : moveBuffer.coords[X_AXIS];
	const float y = (gb.Seen(axisLetters[Y_AXIS])) ? gb.GetFValue() : moveBuffer.coords[Y_AXIS];
	const float z = (gb.Seen(axisLetters[Z_AXIS])) ? gb.GetFValue() : moveBuffer.coords[Z_AXIS];

	reprap.GetMove()->SetXBedProbePoint(probePointIndex, x);
	reprap.GetMove()->SetYBedProbePoint(probePointIndex, y);

	if (z > SILLY_Z_VALUE)
	{
		reprap.GetMove()->SetZBedProbePoint(probePointIndex, z, false, false);
		if (gb.Seen('S'))
		{
			reprap.GetMove()->FinishedBedProbing(gb.GetIValue(), reply);
		}
		return true;
	}
	else
	{
		if (DoSingleZProbeAtPoint(gb, probePointIndex, heightAdjust))
		{
			if (gb.Seen('S'))
			{
				const int sParam = gb.GetIValue();
				if (sParam == 1)
				{
					// G30 with a silly Z value and S=1 is equivalent to G30 with no parameters in that it sets the current Z height
					// This is useful because it adjusts the XY position to account for the probe offset.
					moveBuffer.coords[Z_AXIS] += lastProbedZ;
					SetPositions(moveBuffer.coords);
					lastProbedZ = 0.0;
				}
				else
				{
					reprap.GetMove()->FinishedBedProbing(sParam, reply);
				}
			}
			return true;
		}
	}

	return false;
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
		zProbeType = platform->GetZProbeType();
	}
	ZProbeParameters params = platform->GetZProbeParameters(zProbeType);
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
			params.calibTemperature = platform->GetZProbeTemperature();
		}
	}

	if (seen)
	{
		if (!LockMovementAndWaitForStandstill(gb))
		{
			return false;
		}
		platform->SetZProbeParameters(zProbeType, params);
	}
	else if (seenT)
	{
		// Don't bother printing temperature coefficient and calibration temperature because we will probably remove them soon
		reply.printf("Threshold %d, trigger height %.2f, offsets X%.1f Y%.1f", params.adcValue, params.height, params.xOffset, params.yOffset);
	}
	else
	{
		const int v0 = platform->GetZProbeReading();
		int v1, v2;
		switch (platform->GetZProbeSecondaryValues(v1, v2))
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

	if (gb.Seen('X'))
	{
		size_t count = 2;
		gb.GetFloatArray(xValues, count, false);
		if (count == 2)
		{
			seenX = true;
		}
		else
		{
			reply.copy("ERROR: Wrong number of X values in M577, need 2");
			return true;
		}
	}
	if (gb.Seen('Y'))
	{
		size_t count = 2;
		gb.GetFloatArray(yValues, count, false);
		if (count == 2)
		{
			seenY = true;
		}
		else
		{
			reply.copy("ERROR: Wrong number of Y values in M577, need 2");
			return true;
		}
	}

	float radius = -1.0;
	gb.TryGetFValue('R', radius, seenR);
	float spacing = DefaultGridSpacing;
	gb.TryGetFValue('S', spacing, seenS);

	if (!seenX && !seenY && !seenR && !seenS)
	{
		// Just print the existing grid parameters
		const GridDefinition& grid = reprap.GetMove()->AccessBedProbeGrid().GetGrid();
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
		reply.copy("ERROR: specify both or neither of X and Y in M577");
		return true;
	}

	if (!seenX && !seenR)
	{
		// Must have given just the S parameter
		reply.copy("ERROR: specify at least radius or X,Y ranges in M577");
		return true;

	}

	if (!seenX)
	{
		if (radius > 0)
		{
			const float effectiveRadius = floor((radius - 0.1)/spacing) * spacing;
			xValues[0] = yValues[0] = -effectiveRadius;
			xValues[1] = yValues[1] = effectiveRadius + 0.1;
		}
		else
		{
			reply.copy("ERROR: M577 radius must be positive unless X and Y are specified");
			return true;
		}
	}
	GridDefinition newGrid(xValues, yValues, radius, spacing);		// create a new grid
	if (newGrid.IsValid())
	{
		reprap.GetMove()->AccessBedProbeGrid().SetGrid(newGrid);
		return false;
	}
	else
	{
		reply.copy("ERROR: bad grid definition: ");
		newGrid.PrintError(reply);
		return true;
	}
}

// Start probing the grid, returning true if we didn't because of an error.
// Prior to calling this the movement system must be locked.
bool GCodes::ProbeGrid(GCodeBuffer& gb, StringRef& reply)
{
	Move * const move = reprap.GetMove();
	if (!move->AccessBedProbeGrid().GetGrid().IsValid())
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

	HeightMap& heightMap = move->AccessBedProbeGrid();
	heightMap.UseHeightMap(false);
	heightMap.ClearGridHeights();
	move->SetIdentityTransform();
	gb.SetState(GCodeState::gridProbing1);
	return false;
}

bool GCodes::LoadHeightMap(GCodeBuffer& gb, StringRef& reply) const
{
	reprap.GetMove()->SetIdentityTransform();					// stop using old-style bed compensation and clear the height map
	const char* heightMapFileName;
	if (gb.Seen('P'))
	{
		heightMapFileName = gb.GetString();
	}
	else
	{
		heightMapFileName = DefaultHeightMapFile;
	}
	FileStore * const f = platform->GetFileStore(platform->GetSysDir(), heightMapFileName, false);
	if (f == nullptr)
	{
		reply.printf("Height map file %s not found", heightMapFileName);
		return true;
	}

	reply.printf("Failed to load height map from file %s: ", heightMapFileName);	// set up error message to append to
	HeightMap& heightMap = reprap.GetMove()->AccessBedProbeGrid();
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

	heightMap.UseHeightMap(!err);
	return err;
}

// Save the height map and append the success or error message to 'reply', returning true if an error occurred
// Called by G29 and M374. Both use the P parameter to provide the filename.
bool GCodes::SaveHeightMap(GCodeBuffer& gb, StringRef& reply) const
{
	const char* heightMapFileName;
	if (gb.Seen('P'))
	{
		heightMapFileName = gb.GetString();
		if (heightMapFileName[0] == 0)
		{
			reply.cat("No height map file name provided");
			return false;							// no file name provided, which is legitimate for G29
		}
	}
	else
	{
		heightMapFileName = DefaultHeightMapFile;
	}

	Platform * const platform = reprap.GetPlatform();
	FileStore * const f = platform->GetFileStore(platform->GetSysDir(), heightMapFileName, true);
	bool err;
	if (f == nullptr)
	{
		reply.catf("Failed to create height map file %s", heightMapFileName);
		err = true;
	}
	else
	{
		err = reprap.GetMove()->AccessBedProbeGrid().SaveToFile(f);
		f->Close();
		if (err)
		{
			platform->GetMassStorage()->Delete(platform->GetSysDir(), heightMapFileName);
			reply.catf("Failed to save height map to file %s", heightMapFileName);
		}
		else
		{
			reply.catf("Height map saved to file %s", heightMapFileName);
		}
	}
	return err;
}

// Clear the height map
void GCodes::ClearHeightMap() const
{
	HeightMap& heightMap = reprap.GetMove()->AccessBedProbeGrid();
	heightMap.ClearGridHeights();
	heightMap.UseHeightMap(false);
}

// Return the current coordinates as a printable string.
// Coordinates are updated at the end of each movement, so this won't tell you where you are mid-movement.
void GCodes::GetCurrentCoordinates(StringRef& s) const
{
	float liveCoordinates[DRIVES];
	reprap.GetMove()->LiveCoordinates(liveCoordinates, reprap.GetCurrentXAxes());
	const Tool *currentTool = reprap.GetCurrentTool();
	if (currentTool != nullptr)
	{
		const float *offset = currentTool->GetOffset();
		for (size_t i = 0; i < numAxes; ++i)
		{
			liveCoordinates[i] += offset[i];
		}
	}

	s.Clear();
	for (size_t axis = 0; axis < numAxes; ++axis)
	{
		s.catf("%c: %.2f ", axisLetters[axis], liveCoordinates[axis]);
	}
	for (size_t i = numAxes; i < DRIVES; i++)
	{
		s.catf("E%u: %.1f ", i - numAxes, liveCoordinates[i]);
	}

	// Print the axis stepper motor positions as Marlin does, as an aid to debugging.
	// Don't bother with the extruder endpoints, they are zero after any non-extruding move.
	s.cat(" Count");
	for (size_t i = 0; i < numAxes; ++i)
	{
		s.catf(" %d", reprap.GetMove()->GetEndPoint(i));
	}
}

bool GCodes::OpenFileToWrite(GCodeBuffer& gb, const char* directory, const char* fileName)
{
	fileBeingWritten = platform->GetFileStore(directory, fileName, true);
	eofStringCounter = 0;
	if (fileBeingWritten == NULL)
	{
		platform->MessageF(GENERIC_MESSAGE, "Can't open GCode file \"%s\" for writing.\n", fileName);
		return false;
	}
	else
	{
		gb.SetWritingFileDirectory(directory);
		return true;
	}
}

void GCodes::WriteHTMLToFile(GCodeBuffer& gb, char b)
{
	if (fileBeingWritten == NULL)
	{
		platform->Message(GENERIC_MESSAGE, "Attempt to write to a null file.\n");
		return;
	}

	if (eofStringCounter != 0 && b != eofString[eofStringCounter])
	{
		fileBeingWritten->Write(eofString);
		eofStringCounter = 0;
	}

	if (b == eofString[eofStringCounter])
	{
		eofStringCounter++;
		if (eofStringCounter >= eofStringLength)
		{
			fileBeingWritten->Close();
			fileBeingWritten = NULL;
			gb.SetWritingFileDirectory(NULL);
			const char* r = (platform->Emulating() == marlin) ? "Done saving file." : "";
			HandleReply(gb, false, r);
			return;
		}
	}
	else
	{
		fileBeingWritten->Write(b);
	}
}

void GCodes::WriteGCodeToFile(GCodeBuffer& gb)
{
	if (fileBeingWritten == NULL)
	{
		platform->Message(GENERIC_MESSAGE, "Attempt to write to a null file.\n");
		return;
	}

	// End of file?
	if (gb.Seen('M'))
	{
		if (gb.GetIValue() == 29)
		{
			fileBeingWritten->Close();
			fileBeingWritten = NULL;
			gb.SetWritingFileDirectory(NULL);
			const char* r = (platform->Emulating() == marlin) ? "Done saving file." : "";
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
	FileStore * const f = platform->GetFileStore(platform->GetGCodeDir(), fileName, false);
	if (f != nullptr)
	{
		// Cancel current print if there is any
		if (!reprap.GetPrintMonitor()->IsPrinting())
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
		reprap.GetMove()->ResetExtruderPositions();

		fileToPrint.Set(f);
	}
	else
	{
		platform->MessageF(GENERIC_MESSAGE, "GCode file \"%s\" not found\n", fileName);
	}
}

void GCodes::DeleteFile(const char* fileName)
{
	if (!platform->GetMassStorage()->Delete(platform->GetGCodeDir(), fileName))
	{
		platform->MessageF(GENERIC_MESSAGE, "Could not delete file \"%s\"\n", fileName);
	}
}

// Function to handle dwell delays.  Return true for dwell finished, false otherwise.
bool GCodes::DoDwell(GCodeBuffer& gb)
{
	float dwell;
	if (gb.Seen('S'))
	{
		dwell = gb.GetFValue();
	}
	else if (gb.Seen('P'))
	{
		dwell = 0.001 * (float) gb.GetIValue(); // P values are in milliseconds; we need seconds
	}
	else
	{
		return true;  // No time given - throw it away
	}

#if SUPPORT_ROLAND
	// Deal with a Roland configuration
	if (reprap.GetRoland()->Active())
	{
		return reprap.GetRoland()->ProcessDwell(gb.GetLValue());
	}
#endif

	// Wait for all the queued moves to stop
	if (!LockMovementAndWaitForStandstill(gb))
	{
		return false;
	}

	if (simulationMode != 0)
	{
		simulationTime += dwell;
		return true;
	}
	else
	{
		return DoDwellTime(dwell);
	}
}

bool GCodes::DoDwellTime(float dwell)
{
	// Are we already in the dwell?
	if (dwellWaiting)
	{
		if (platform->Time() - dwellTime >= 0.0)
		{
			dwellWaiting = false;
			reprap.GetMove()->ResumeMoving();
			return true;
		}
		return false;
	}

	// New dwell - set it up
	dwellWaiting = true;
	dwellTime = platform->Time() + dwell;
	return false;
}

// Set offset, working and standby temperatures for a tool. I.e. handle a G10.
bool GCodes::SetOrReportOffsets(GCodeBuffer &gb, StringRef& reply)
{
	if (gb.Seen('P'))
	{
		int8_t toolNumber = gb.GetIValue();
		toolNumber += gb.GetToolNumberAdjust();
		Tool* tool = reprap.GetTool(toolNumber);
		if (tool == NULL)
		{
			reply.printf("Attempt to set/report offsets and temperatures for non-existent tool: %d", toolNumber);
			return true;
		}

		// Deal with setting offsets
		float offset[MAX_AXES];
		for (size_t i = 0; i < MAX_AXES; ++i)
		{
			offset[i] = tool->GetOffset()[i];
		}

		bool settingOffset = false;
		for (size_t axis = 0; axis < numAxes; ++axis)
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
		float standby[HEATERS];
		float active[HEATERS];
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
			reply.printf("Tool %d offsets:", toolNumber);
			for (size_t axis = 0; axis < numAxes; ++axis)
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
	}
	return true;
}

void GCodes::ManageTool(GCodeBuffer& gb, StringRef& reply)
{
	if (!gb.Seen('P'))
	{
		// DC temporary code to allow tool numbers to be adjusted so that we don't need to edit multi-media files generated by slic3r
		if (gb.Seen('S'))
		{
			int adjust = gb.GetIValue();
			gb.SetToolNumberAdjust(adjust);
		}
		return;
	}

	// Check tool number
	bool seen = false;
	const int toolNumber = gb.GetIValue();
	if (toolNumber < 0)
	{
		platform->Message(GENERIC_MESSAGE, "Tool number must be positive!\n");
		return;
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
	long heaters[HEATERS];
	size_t hCount = HEATERS;
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
	uint32_t xMap;
	if (gb.Seen('X'))
	{
		long xMapping[MAX_AXES];
		size_t xCount = numAxes;
		gb.GetLongArray(xMapping, xCount);
		xMap = LongArrayToBitMap(xMapping, xCount) & ((1u << numAxes) - 1);
		seen = true;
	}
	else
	{
		xMap = 1;					// by default map X axis straight through
	}

	// Check for fan mapping
	uint32_t fanMap;
	if (gb.Seen('F'))
	{
		long fanMapping[NUM_FANS];
		size_t fanCount = NUM_FANS;
		gb.GetLongArray(fanMapping, fanCount);
		fanMap = LongArrayToBitMap(fanMapping, fanCount) & ((1u << NUM_FANS) - 1);
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
			Tool* tool = Tool::Create(toolNumber, drives, dCount, heaters, hCount, xMap, fanMap);
			if (tool != nullptr)
			{
				reprap.AddTool(tool);
			}
		}
	}
	else
	{
		reprap.PrintTool(toolNumber, reply);
	}
}

// Does what it says.
void GCodes::DisableDrives()
{
	for (size_t drive = 0; drive < DRIVES; drive++)
	{
		platform->DisableDrive(drive);
	}
	SetAllAxesNotHomed();
}

// Does what it says.
void GCodes::SetEthernetAddress(GCodeBuffer& gb, int mCode)
{
	byte eth[4];
	const char* ipString = gb.GetString();
	uint8_t sp = 0;
	uint8_t spp = 0;
	uint8_t ipp = 0;
	while (ipString[sp])
	{
		if (ipString[sp] == '.')
		{
			eth[ipp] = atoi(&ipString[spp]);
			ipp++;
			if (ipp > 3)
			{
				platform->MessageF(GENERIC_MESSAGE, "Dud IP address: %s\n", gb.Buffer());
				return;
			}
			sp++;
			spp = sp;
		}
		else
		{
			sp++;
		}
	}
	eth[ipp] = atoi(&ipString[spp]);
	if (ipp == 3)
	{
		switch (mCode)
		{
		case 552:
			platform->SetIPAddress(eth);
			break;
		case 553:
			platform->SetNetMask(eth);
			break;
		case 554:
			platform->SetGateWay(eth);
			break;

		default:
			platform->Message(GENERIC_MESSAGE, "Setting ether parameter - dud code.\n");
		}
	}
	else
	{
		platform->MessageF(GENERIC_MESSAGE, "Dud IP address: %s\n", gb.Buffer());
	}
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
			mac[ipp] = strtoul(&ipString[spp], NULL, 16);
			ipp++;
			if (ipp > 5)
			{
				platform->MessageF(GENERIC_MESSAGE, "Dud MAC address: %s\n", gb.Buffer());
				return;
			}
			sp++;
			spp = sp;
		}
		else
		{
			sp++;
		}
	}
	mac[ipp] = strtoul(&ipString[spp], NULL, 16);
	if (ipp == 5)
	{
		platform->SetMACAddress(mac);
	}
	else
	{
		platform->MessageF(GENERIC_MESSAGE, "Dud MAC address: %s\n", gb.Buffer());
	}
}

bool GCodes::ChangeMicrostepping(size_t drive, int microsteps, int mode) const
{
	bool dummy;
	unsigned int oldSteps = platform->GetMicrostepping(drive, dummy);
	bool success = platform->SetMicrostepping(drive, microsteps, mode);
	if (success && mode <= 1)							// modes higher than 1 are used for special functions
	{
		// We changed the microstepping, so adjust the steps/mm to compensate
		float stepsPerMm = platform->DriveStepsPerUnit(drive);
		if (stepsPerMm > 0)
		{
			platform->SetDriveStepsPerUnit(drive, stepsPerMm * (float)microsteps / (float)oldSteps);
		}
	}
	return success;
}

// Set the speeds of fans mapped for the current tool to lastDefaultFanSpeed
void GCodes::SetMappedFanSpeed()
{
	if (reprap.GetCurrentTool() == nullptr)
	{
		platform->SetFanValue(0, lastDefaultFanSpeed);
	}
	else
	{
		const uint32_t fanMap = reprap.GetCurrentTool()->GetFanMapping();
		for (size_t i = 0; i < NUM_FANS; ++i)
		{
			if ((fanMap & (1u << i)) != 0)
			{
				platform->SetFanValue(i, lastDefaultFanSpeed);
			}
		}
	}
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
		platform->AppendAuxReply(reply);
		return;
	}

	const Compatibility c = (&gb == serialGCode || &gb == telnetGCode) ? platform->Emulating() : me;
	MessageType type = GENERIC_MESSAGE;
	if (&gb == httpGCode)
	{
		type = HTTP_MESSAGE;
	}
	else if (&gb == telnetGCode)
	{
		type = TELNET_MESSAGE;
	}
	else if (&gb == serialGCode)
	{
		type = HOST_MESSAGE;
	}

	const char* response = (gb.Seen('M') && gb.GetIValue() == 998) ? "rs " : "ok";
	const char* emulationType = 0;

	switch (c)
	{
		case me:
		case reprapFirmware:
			if (error)
			{
				platform->Message(type, "Error: ");
			}
			platform->Message(type, reply);
			platform->Message(type, "\n");
			return;

		case marlin:
			// We don't need to handle M20 here because we always allocate an output buffer for that one
			if (gb.Seen('M') && gb.GetIValue() == 28)
			{
				platform->Message(type, response);
				platform->Message(type, "\n");
				platform->Message(type, reply);
				platform->Message(type, "\n");
				return;
			}

			if ((gb.Seen('M') && gb.GetIValue() == 105) || (gb.Seen('M') && gb.GetIValue() == 998))
			{
				platform->Message(type, response);
				platform->Message(type, " ");
				platform->Message(type, reply);
				platform->Message(type, "\n");
				return;
			}

			if (reply[0] != 0 && !gb.IsDoingFileMacro())
			{
				platform->Message(type, reply);
				platform->Message(type, "\n");
				platform->Message(type, response);
				platform->Message(type, "\n");
			}
			else if (reply[0] != 0)
			{
				platform->Message(type, reply);
				platform->Message(type, "\n");
			}
			else
			{
				platform->Message(type, response);
				platform->Message(type, "\n");
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
		platform->MessageF(type, "Emulation of %s is not yet supported.\n", emulationType);	// don't send this one to the web as well, it concerns only the USB interface
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
		platform->AppendAuxReply(reply);
		return;
	}

	const Compatibility c = (&gb == serialGCode || &gb == telnetGCode) ? platform->Emulating() : me;
	MessageType type = GENERIC_MESSAGE;
	if (&gb == httpGCode)
	{
		type = HTTP_MESSAGE;
	}
	else if (&gb == telnetGCode)
	{
		type = TELNET_MESSAGE;
	}
	else if (&gb == serialGCode)
	{
		type = HOST_MESSAGE;
	}

	const char* response = (gb.Seen('M') && gb.GetIValue() == 998) ? "rs " : "ok";
	const char* emulationType = nullptr;

	switch (c)
	{
		case me:
		case reprapFirmware:
			if (error)
			{
				platform->Message(type, "Error: ");
			}
			platform->Message(type, reply);
			return;

		case marlin:
			if (gb.Seen('M') && gb.GetIValue() == 20)
			{
				platform->Message(type, "Begin file list\n");
				platform->Message(type, reply);
				platform->Message(type, "End file list\n");
				platform->Message(type, response);
				platform->Message(type, "\n");
				return;
			}

			if (gb.Seen('M') && gb.GetIValue() == 28)
			{
				platform->Message(type, response);
				platform->Message(type, "\n");
				platform->Message(type, reply);
				return;
			}

			if ((gb.Seen('M') && gb.GetIValue() == 105) || (gb.Seen('M') && gb.GetIValue() == 998))
			{
				platform->Message(type, response);
				platform->Message(type, " ");
				platform->Message(type, reply);
				return;
			}

			if (reply->Length() != 0 && !gb.IsDoingFileMacro())
			{
				platform->Message(type, reply);
				platform->Message(type, "\n");
				platform->Message(type, response);
				platform->Message(type, "\n");
			}
			else if (reply->Length() != 0)
			{
				platform->Message(type, reply);
			}
			else
			{
				OutputBuffer::ReleaseAll(reply);
				platform->Message(type, response);
				platform->Message(type, "\n");
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
		platform->MessageF(type, "Emulation of %s is not yet supported.\n", emulationType);	// don't send this one to the web as well, it concerns only the USB interface
	}
}

// Set PID parameters (M301 or M304 command). 'heater' is the default heater number to use.
void GCodes::SetPidParameters(GCodeBuffer& gb, int heater, StringRef& reply)
{
	if (gb.Seen('H'))
	{
		heater = gb.GetIValue();
	}

	if (heater >= 0 && heater < HEATERS)
	{
		const FopDt& model = reprap.GetHeat()->GetHeaterModel(heater);
		M301PidParameters pp = model.GetM301PidParameters(false);
		bool seen = false;
		gb.TryGetFValue('P', pp.kP, seen);
		gb.TryGetFValue('I', pp.kI, seen);
		gb.TryGetFValue('D', pp.kD, seen);

		if (seen)
		{
			reprap.GetHeat()->SetM301PidParameters(heater, pp);
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

void GCodes::SetHeaterParameters(GCodeBuffer& gb, StringRef& reply)
{
	if (gb.Seen('P'))
	{
		int heater = gb.GetIValue();
		if (heater >= 0 && heater < HEATERS)
		{
			Thermistor& th = platform->GetThermistor(heater);
			bool seen = false;

			// We must set the 25C resistance and beta together in order to calculate Rinf. Check for these first.
			float r25 = th.GetR25();
			float beta = th.GetBeta();
			float shC = th.GetShc();
			float seriesR = th.GetSeriesR();

			gb.TryGetFValue('T', r25, seen);
			gb.TryGetFValue('B', beta, seen);
			gb.TryGetFValue('C', shC, seen);
			gb.TryGetFValue('R', seriesR, seen);
			if (seen)
			{
				th.SetParameters(r25, beta, shC, seriesR);
			}

			if (gb.Seen('L'))
			{
				th.SetLowOffset((int8_t)constrain<int>(gb.GetIValue(), -100, 100));
				seen = true;
			}
			if (gb.Seen('H'))
			{
				th.SetHighOffset((int8_t)constrain<int>(gb.GetIValue(), -100, 100));
				seen = true;
			}

			if (gb.Seen('X'))
			{
				int thermistor = gb.GetIValue();
				if (   (0 <= thermistor && thermistor < HEATERS)
					|| ((int)FirstThermocoupleChannel <= thermistor && thermistor < (int)(FirstThermocoupleChannel + MaxSpiTempSensors))
					|| ((int)FirstRtdChannel <= thermistor && thermistor < (int)(FirstRtdChannel + MaxSpiTempSensors))
				   )
				{
					platform->SetThermistorNumber(heater, thermistor);
				}
				else
				{
					platform->MessageF(GENERIC_MESSAGE, "Thermistor number %d is out of range\n", thermistor);
				}
				seen = true;
			}

			if (!seen)
			{
				reply.printf("T:%.1f B:%.1f C:%.2e R:%.1f L:%d H:%d X:%d",
						th.GetR25(), th.GetBeta(), th.GetShc(), th.GetSeriesR(),
						th.GetLowOffset(), th.GetHighOffset(), platform->GetThermistorNumber(heater));
			}
		}
		else
		{
			platform->MessageF(GENERIC_MESSAGE, "Heater number %d is out of range\n", heater);
		}
	}
}

void GCodes::SetToolHeaters(Tool *tool, float temperature)
{
	if (tool == NULL)
	{
		platform->Message(GENERIC_MESSAGE, "Setting temperature: no tool selected.\n");
		return;
	}

	float standby[HEATERS];
	float active[HEATERS];
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
		const Tool * const tool = reprap.GetCurrentTool();
		if (tool != nullptr)
		{
			const size_t nDrives = tool->DriveCount();
			if (nDrives != 0)
			{
				if (segmentsLeft != 0)
				{
					return false;
				}

				const uint32_t xAxes = reprap.GetCurrentXAxes();
				reprap.GetMove()->GetCurrentUserPosition(moveBuffer.coords, 0, xAxes);
				for (size_t i = numAxes; i < DRIVES; ++i)
				{
					moveBuffer.coords[i] = 0.0;
				}
				// Set the feed rate. If there is any Z hop then we need to pass the Z speed, else we pass the extrusion speed.
				const float speedToUse = (retract) ? retractSpeed : unRetractSpeed;
				moveBuffer.feedRate = (retractHop == 0.0 || retractLength == 0.0)
										? speedToUse
										: speedToUse * retractHop/retractLength;
				moveBuffer.coords[Z_AXIS] += (retract) ? retractHop : -retractHop;
				const float lengthToUse = (retract) ? -retractLength : retractLength + retractExtra;
				for (size_t i = 0; i < nDrives; ++i)
				{
					moveBuffer.coords[numAxes + tool->Drive(i)] = lengthToUse;
				}

				moveBuffer.moveType = 0;
				moveBuffer.isFirmwareRetraction = true;
				moveBuffer.usePressureAdvance = false;
				moveBuffer.filePos = (&gb == fileGCode) ? gb.MachineState().fileState.GetPosition() : noFilePosition;
				moveBuffer.canPauseAfter = !retract;			// don't pause after a retraction because that could cause too much retraction
				moveBuffer.xAxes = xAxes;
				segmentsLeft = 1;
			}
		}
		isRetracted = retract;
	}
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

	fileGCode->Init();
	FileData& fileBeingPrinted = fileGCode->OriginalMachineState().fileState;
	if (fileBeingPrinted.IsLive())
	{
		fileBeingPrinted.Close();
	}

	reprap.GetPrintMonitor()->StoppedPrint();
}

// Return true if all the heaters for the specified tool are at their set temperatures
bool GCodes::ToolHeatersAtSetTemperatures(const Tool *tool, bool waitWhenCooling) const
{
	if (tool != NULL)
	{
		for (size_t i = 0; i < tool->HeaterCount(); ++i)
		{
			if (!reprap.GetHeat()->HeaterAtSetTemperature(tool->Heater(i), waitWhenCooling))
			{
				return false;
			}
		}
	}
	return true;
}

// Set the current position, optionally applying bed and axis compensation
void GCodes::SetPositions(const float positionNow[DRIVES], bool doBedCompensation)
{
	float newPos[DRIVES];
	memcpy(newPos, positionNow, sizeof(newPos));			// copy to local storage because Transform modifies it
	reprap.GetMove()->Transform(newPos, reprap.GetCurrentXAxes(), doBedCompensation);
	reprap.GetMove()->SetLiveCoordinates(newPos);
	reprap.GetMove()->SetPositions(newPos);
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
void GCodes::ListTriggers(StringRef reply, TriggerMask mask)
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
			if ((mask & (1u << i)) != 0)
			{
				if (printed)
				{
					reply.cat(' ');
				}
				if (i < numAxes)
				{
					reply.cat(axisLetters[i]);
				}
				else
				{
					reply.catf("E%d", i - numAxes);
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
	fileBeingHashed = platform->GetFileStore(FS_PREFIX, filename, false);
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
	const uint32_t allAxes = (1u << numAxes) - 1;
	return (axesHomed & allAxes) == allAxes;
}

void GCodes::SetAllAxesNotHomed()
{
	axesHomed = 0;
}

// Write the config-override file returning true if an error occurred
bool GCodes::WriteConfigOverrideFile(StringRef& reply, const char *fileName) const
{
	FileStore * const f = platform->GetFileStore(platform->GetSysDir(), fileName, true);
	if (f == nullptr)
	{
		reply.printf("Failed to create file %s", fileName);
		return true;
	}

	bool ok = f->Write("; This is a system-generated file - do not edit\n");
	if (ok)
	{
		ok = reprap.GetMove()->GetDeltaParams().WriteParameters(f);
	}
	if (ok)
	{
		ok = reprap.GetHeat()->WriteModelParameters(f);
	}
	if (ok)
	{
		ok = platform->WriteZProbeParameters(f);
	}
	if (!f->Close())
	{
		ok = false;
	}
	if (!ok)
	{
		reply.printf("Failed to write file %s", fileName);
		platform->GetMassStorage()->Delete(platform->GetSysDir(), fileName);
	}
	return !ok;
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
		gb.MachineState().lockedResources |= (1u << r);
		return true;
	}
	return false;
}

bool GCodes::LockHeater(const GCodeBuffer& gb, int heater)
{
	if (heater >= 0 && heater < HEATERS)
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
		if (resourceOwners[i] == &gb && ((1u << i) & resourcesToKeep) == 0)
		{
			resourceOwners[i] = nullptr;
			gb.MachineState().lockedResources &= ~(1u << i);
		}
	}
}

// Convert an array of longs to a bit map
/*static*/ uint32_t GCodes::LongArrayToBitMap(const long *arr, size_t numEntries)
{
	uint32_t res = 0;
	for (size_t i = 0; i < numEntries; ++i)
	{
		const long f = arr[i];
		if (f >= 0 && f < 32)
		{
			res |= 1u << (unsigned int)f;
		}
	}
	return res;
}

// End
