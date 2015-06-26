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

#include "RepRapFirmware.h"

#define DEGREE_SYMBOL	"\xC2\xB0"				// degree-symbol encoding in UTF8

const char GCodes::axisLetters[AXES] =
{ 'X', 'Y', 'Z' };

const size_t gcodeReplyLength = 2048;			// long enough to pass back a reasonable number of files in response to M20

GCodes::GCodes(Platform* p, Webserver* w) :
		active(false), platform(p), webserver(w), stackPointer(0)
{
	webGCode = new GCodeBuffer(platform, "web: ");
	fileGCode = new GCodeBuffer(platform, "file: ");
	serialGCode = new GCodeBuffer(platform, "serial: ");
	auxGCode = new GCodeBuffer(platform, "aux: ");
	fileMacroGCode = new GCodeBuffer(platform, "macro: ");
}

void GCodes::Exit()
{
	platform->Message(BOTH_MESSAGE, "GCodes class exited.\n");
	active = false;
}

void GCodes::Init()
{
	Reset();
	distanceScale = 1.0;
	for (unsigned int extruder = 0; extruder < DRIVES - AXES; extruder++)
	{
		lastRawExtruderPosition[extruder] = 0.0;
	}
	configFile = NULL;
	eofString = EOF_STRING;
	eofStringCounter = 0;
	eofStringLength = strlen(eofString);
	offSetSet = false;
	zProbesSet = false;
	active = true;
	longWait = platform->Time();
	dwellTime = longWait;
	limitAxes = true;
	SetAllAxesNotHomed();
	coolingInverted = false;
}

// This is called from Init and when doing an emergency stop
void GCodes::Reset()
{
	webGCode->Init();
	fileGCode->Init();
	serialGCode->Init();
	auxGCode->Init();
	auxGCode->SetCommsProperties(1);					// by default, we require a checksum on the aux port
	fileMacroGCode->Init();
	moveAvailable = false;
	fileBeingPrinted.Close();
	fileToPrint.Close();
	fileBeingWritten = NULL;
	endStopsToCheck = 0;
	doingFileMacro = false;
	dwellWaiting = false;
	stackPointer = 0;
	state = GCodeState::normal;
	drivesRelative = true;
	axesRelative = false;
	probeCount = 0;
	cannedCycleMoveCount = 0;
	cannedCycleMoveQueued = false;
	speedFactor = 1.0 / minutesToSeconds;				// default is just to convert from mm/minute to mm/second
	speedFactorChange = 1.0;
	for (size_t i = 0; i < DRIVES - AXES; ++i)
	{
		extrusionFactors[i] = 1.0;
	}
	for (size_t i = 0; i < DRIVES; ++i)
	{
		pausedMoveBuffer[i] = 0.0;
	}
	auxDetected = false;
	simulating = false;
	simulationTime = 0.0;
	isPaused = false;
	filePos = moveFilePos = noFilePosition;
}

float GCodes::FractionOfFilePrinted() const
{
	if (isPaused)
	{
		return (fileToPrint.IsLive()) ? fileToPrint.FractionRead() : -1.0;
	}
	if (stackPointer == 0)
	{
		return (fileBeingPrinted.IsLive() && !doingFileMacro) ? fileBeingPrinted.FractionRead() : -1.0;
	}
	return (stack[0].fileState.IsLive() && !stack[0].doingFileMacro) ? stack[0].fileState.FractionRead() : -1.0;
}

void GCodes::DoFilePrint(GCodeBuffer* gb, StringRef& reply)
{
	for (int i = 0; i < 50 && fileBeingPrinted.IsLive(); ++i)
	{
		char b;
		if (fileBeingPrinted.Read(b))
		{
			if (gb->StartingNewCode() && gb == fileGCode)
			{
				filePos = fileBeingPrinted.GetPosition() - 1;
				//debugPrintf("Set file pos %u\n", filePos);
			}
			if (gb->Put(b))
			{
				gb->SetFinished(ActOnCode(gb, reply));
				break;
			}
		}
		else
		{
			// We have reached the end of the file.
			// Don't close the file until all moves have been completed, in case the print gets paused.
			// Also, this keeps the state as 'Printing' until the print really has finished.
			if (!gb->StartingNewCode())		// if there is something in the buffer
			{
				if (gb->Put('\n')) 			// in case there wasn't one ending the file
				{
					gb->SetFinished(ActOnCode(gb, reply));
				}
				else
				{
					gb->Init();
				}
			}
			else if (AllMovesAreFinishedAndMoveBufferIsLoaded())
			{
				fileBeingPrinted.Close();
				if (gb == fileGCode)
				{
					reprap.GetPrintMonitor()->StoppedPrint();
				}
			}
			break;
		}
	}
}

void GCodes::Spin()
{
	if (!active)
		return;

	char replyBuffer[gcodeReplyLength];
	StringRef reply(replyBuffer, ARRAY_SIZE(replyBuffer));
	reply.Clear();

	// Check for M105 poll requests from Pronterface and PanelDue so that the status is kept up to date during execution of file macros etc.
	// No need to read multiple characters at a time in this case because the polling rate is quite low.
	if (!serialGCode->Active() && serialGCode->WritingFileDirectory() == nullptr
			&& (platform->GetLine()->Status() & (uint8_t)IOStatus::byteAvailable))
	{
		char b;
		platform->GetLine()->Read(b);
		if (serialGCode->Put(b))	// add char to buffer and test whether the gcode is complete
		{
			if (serialGCode->IsPollRequest())
			{
				serialGCode->SetFinished(ActOnCode(serialGCode, reply));
				return;
			}
		}
	}

	if (!auxGCode->Active() && (platform->GetAux()->Status() & (uint8_t)IOStatus::byteAvailable))
	{
		char b;
		platform->GetAux()->Read(b);
		if (auxGCode->Put(b))		// add char to buffer and test whether the gcode is complete
		{
			auxDetected = true;
			if (auxGCode->IsPollRequest())
			{
				auxGCode->SetFinished(ActOnCode(auxGCode, reply));
				return;
			}
		}
	}

	// Perform the next operation of the state machine
	// Note: if we change the state to 'normal' from another state, we must call HandleReply to tell the host about the command we have just completed.
	switch (state)
	{
	case GCodeState::normal:
		StartNextGCode(reply);
		break;

	case GCodeState::waitingForMoveToComplete:
		if (AllMovesAreFinishedAndMoveBufferIsLoaded())
		{
			HandleReply(false, gbCurrent, nullptr, 'G', 1, false);
			state = GCodeState::normal;
		}
		break;

	case GCodeState::homing:
		if (toBeHomed & (1 << X_AXIS))
		{
			toBeHomed &= ~(1 << X_AXIS);
			DoFileMacro(HOME_X_G);
		}
		else if (toBeHomed & (1 << Y_AXIS))
		{
			toBeHomed &= ~(1 << Y_AXIS);
			DoFileMacro(HOME_Y_G);
		}
		else if (toBeHomed & (1 << Z_AXIS))
		{
			toBeHomed &= ~(1 << Z_AXIS);
			DoFileMacro(HOME_Z_G);
		}
		else
		{
			HandleReply(false, gbCurrent, nullptr, 'G', 28, false);
			state = GCodeState::normal;
		}
		break;

	case GCodeState::setBed1:
		reprap.GetMove()->SetIdentityTransform();
		probeCount = 0;
		state = GCodeState::setBed2;
		// no break

	case GCodeState::setBed2:
		{
			int numProbePoints = reprap.GetMove()->NumberOfXYProbePoints();
			if (DoSingleZProbeAtPoint(probeCount))
			{
				probeCount++;
				if (probeCount >= numProbePoints)
				{
					zProbesSet = true;
					reprap.GetMove()->FinishedBedProbing(0, reply);
					HandleReply(false, gbCurrent, reply.Pointer(), 'G', 32, false);
					state = GCodeState::normal;
				}
			}
		}
		break;

	case GCodeState::toolChange1: // Release the old tool (if any)
		{
			const Tool *oldTool = reprap.GetCurrentTool();
			if (oldTool != NULL)
			{
				reprap.StandbyTool(oldTool->Number());
			}
		}
		state = GCodeState::toolChange2;
		if (reprap.GetTool(newToolNumber) != NULL && AllAxesAreHomed())
		{
			scratchString.printf("tpre%d.g", newToolNumber);
			DoFileMacro(scratchString.Pointer(), false);
		}
		break;

	case GCodeState::toolChange2: // Select the new tool (even if it doesn't exist - that just deselects all tools)
		reprap.SelectTool(newToolNumber);
		state = GCodeState::toolChange3;
		if (reprap.GetTool(newToolNumber) != NULL && AllAxesAreHomed())
		{
			scratchString.printf("tpost%d.g", newToolNumber);
			DoFileMacro(scratchString.Pointer(), false);
		}
		break;

	case GCodeState::toolChange3:
		HandleReply(false, gbCurrent, nullptr, 'T', newToolNumber, false);
		state = GCodeState::normal;
		break;

	case GCodeState::pausing1:
		if (AllMovesAreFinishedAndMoveBufferIsLoaded())
		{
			state = GCodeState::pausing2;
			DoFileMacro(PAUSE_G);
		}
		break;

	case GCodeState::pausing2:
		HandleReply(false, gbCurrent, "Printing paused\n", 'M', 25, false);
		state = GCodeState::normal;
		break;

	case GCodeState::resuming1:
	case GCodeState::resuming2:
		// Here when we have just finished running the resume macro file.
		// Move the head back to the paused location
		if (AllMovesAreFinishedAndMoveBufferIsLoaded())
		{
			float currentZ = moveBuffer[Z_AXIS];
			for (size_t drive = 0; drive < AXES; ++drive)
			{
				moveBuffer[drive] =  pausedMoveBuffer[drive];
			}
			for (size_t drive = AXES; drive < DRIVES; ++drive)
			{
				moveBuffer[drive] = 0.0;
			}
			moveBuffer[DRIVES] = 5000 / minutesToSeconds;	// ask for a good feed rate, we may have paused during a slow move
			moveType = 0;
			endStopsToCheck = 0;
			moveFilePos = noFilePosition;
			if (state == GCodeState::resuming1 && currentZ > pausedMoveBuffer[Z_AXIS])
			{
				// First move the head to the correct XY point, then move it down in a separate move
				moveBuffer[Z_AXIS] = currentZ;
				state = GCodeState::resuming2;
			}
			else
			{
				// Just move to the saved position in one go
				state = GCodeState::resuming3;
			}
			moveAvailable = true;
		}
		break;

	case GCodeState::resuming3:
		if (AllMovesAreFinishedAndMoveBufferIsLoaded())
		{
			platform->SetFanValue(pausedFanValue);
			fileBeingPrinted.MoveFrom(fileToPrint);
			for (size_t drive = AXES; drive < DRIVES; ++drive)
			{
				lastRawExtruderPosition[drive - AXES] = pausedMoveBuffer[drive];	// reset the extruder position in case we are receiving absolute extruder moves
			}
			moveBuffer[DRIVES] = pausedMoveBuffer[DRIVES];
			reprap.GetMove()->SetFeedrate(pausedMoveBuffer[DRIVES]);
			fileGCode->Resume();
			isPaused = false;
			HandleReply(false, gbCurrent, "Printing resumed\n", 'M', 24, false);
			state = GCodeState::normal;
		}
		break;

	default:				// should not happen
		break;
	}
}

void GCodes::StartNextGCode(StringRef& reply)
{
	// If a file macro is running, we don't allow anything to interrupt it
	if (doingFileMacro)
	{
		// Complete the current move (must do this before checking whether we have finished the file in case it didn't end in newline)
		if (fileMacroGCode->Active())
		{
			fileMacroGCode->SetFinished(ActOnCode(fileMacroGCode, reply));
		}
		else if (fileBeingPrinted.IsLive())				// Have we finished the file?
		{
			DoFilePrint(fileMacroGCode, reply);			// No - Do more of the file
		}
		else if (AllMovesAreFinishedAndMoveBufferIsLoaded())
		{
			Pop();
			fileMacroGCode->Init();
		}
		return;
	}

	// Check each of the sources of G Codes (web, serial, queued and file) to
	// see if they are finished in order to feed them new codes.
	//
	// Note the order establishes a priority: web, serial, queued, file.
	// If file weren't last, then the others would never get a look in when
	// a file was being printed.

	if (!webGCode->Active() && webserver->GCodeAvailable())
	{
		int8_t i = 0;
		do
		{
			char b = webserver->ReadGCode();
			if (webGCode->Put(b))
			{
				// we have a complete gcode
				if (webGCode->WritingFileDirectory() != NULL)
				{
					WriteGCodeToFile(webGCode);
				}
				else
				{
					webGCode->SetFinished(ActOnCode(webGCode, reply));
				}
				break;	// stop after receiving a complete gcode in case we haven't finished processing it
			}
			++i;
		} while (i < 16 && webserver->GCodeAvailable());
		platform->ClassReport(longWait);
		return;
	}

	// Now the serial interfaces.

	if (platform->GetLine()->Status() & (uint8_t)IOStatus::byteAvailable)
	{
		// First check the special case of uploading the reprap.htm file
		if (serialGCode->WritingFileDirectory() == platform->GetWebDir())
		{
			char b;
			platform->GetLine()->Read(b);
			WriteHTMLToFile(b, serialGCode);
			platform->ClassReport(longWait);
			return;
		}

		// Otherwise just deal in general with incoming bytes from the serial interface
		else if (!serialGCode->Active())
		{
			// Read several bytes instead of just one. This approximately doubles the speed of file uploading.
			int8_t i = 0;
			do
			{
				char b;
				platform->GetLine()->Read(b);
				if (serialGCode->Put(b))	// add char to buffer and test whether the gcode is complete
				{
					// we have a complete gcode
					if (serialGCode->WritingFileDirectory() != NULL)
					{
						WriteGCodeToFile(serialGCode);
						serialGCode->SetFinished(true);
					}
					else
					{
						serialGCode->SetFinished(ActOnCode(serialGCode, reply));
					}
					break;	// stop after receiving a complete gcode in case we haven't finished processing it
				}
				++i;
			} while (i < 16 && (platform->GetLine()->Status() & (uint8_t)IOStatus::byteAvailable));
			platform->ClassReport(longWait);
			return;
		}
	}

	// Now run the G-Code buffers. It's important to fill up the G-Code buffers before we do this,
	// otherwise we wouldn't have a chance to pause/cancel running prints.
	if (!auxGCode->Active() && (platform->GetAux()->Status() & (uint8_t)IOStatus::byteAvailable))
	{
		int8_t i = 0;
		do
		{
			char b;
			platform->GetAux()->Read(b);
			if (auxGCode->Put(b))	// add char to buffer and test whether the gcode is complete
			{
				auxDetected = true;
				auxGCode->SetFinished(ActOnCode(auxGCode, reply));
				break;	// stop after receiving a complete gcode in case we haven't finished processing it
			}
			++i;
		} while (i < 16 && (platform->GetAux()->Status() & (uint8_t)IOStatus::byteAvailable));
	}
	else if (webGCode->Active())
	{
		// Note: Direct web-printing has been dropped, so it's safe to execute web codes immediately
		webGCode->SetFinished(ActOnCode(webGCode, reply));
	}
	else if (serialGCode->Active())
	{
		// We want codes from the serial interface to be queued unless the print has been paused
		serialGCode->SetFinished(ActOnCode(serialGCode, reply));
	}
	else if (auxGCode->Active())
	{
		// Same goes for our auxiliary interface
		auxGCode->SetFinished(ActOnCode(auxGCode, reply));
	}
	else if (fileGCode->Active())
	{
		fileGCode->SetFinished(ActOnCode(fileGCode, reply));
	}
	else
	{
		DoFilePrint(fileGCode, reply);					// else see if there is anything to print from file
	}

	platform->ClassReport(longWait);
}

void GCodes::Diagnostics()
{
	platform->AppendMessage(BOTH_MESSAGE, "GCodes Diagnostics:\n");
	platform->AppendMessage(BOTH_MESSAGE, "Move available? %s\n", moveAvailable ? "yes" : "no");
}

// The wait till everything's done function.  If you need the machine to
// be idle before you do something (for example homing an axis, or shutting down) call this
// until it returns true.  As a side-effect it loads moveBuffer with the last
// position and feedrate for you.

bool GCodes::AllMovesAreFinishedAndMoveBufferIsLoaded()
{
	// Last one gone?
	if (moveAvailable)
		return false;

	// Wait for all the queued moves to stop so we get the actual last position and feedrate
	if (!reprap.GetMove()->AllMovesAreFinished())
		return false;
	reprap.GetMove()->ResumeMoving();

	reprap.GetMove()->GetCurrentUserPosition(moveBuffer, 0);
	return true;
}

// Save (some of) the state of the machine for recovery in the future.
void GCodes::Push()
{
	if (stackPointer >= StackSize)
	{
		platform->Message(BOTH_ERROR_MESSAGE, "Push(): stack overflow!\n");
		return;
	}

	stack[stackPointer].state = state;
	stack[stackPointer].gb = gbCurrent;
	stack[stackPointer].feedrate = moveBuffer[DRIVES];
	stack[stackPointer].fileState.CopyFrom(fileBeingPrinted);
	stack[stackPointer].drivesRelative = drivesRelative;
	stack[stackPointer].axesRelative = axesRelative;
	stack[stackPointer].doingFileMacro = doingFileMacro;
	stackPointer++;
	platform->PushMessageIndent();
}

// Recover a saved state
void GCodes::Pop()
{
	if (stackPointer < 1)
	{
		platform->Message(BOTH_ERROR_MESSAGE, "Pop(): stack underflow!\n");
		return;
	}

	stackPointer--;
	state = stack[stackPointer].state;
	gbCurrent = stack[stackPointer].gb;
	moveBuffer[DRIVES] = stack[stackPointer].feedrate;
	reprap.GetMove()->SetFeedrate(moveBuffer[DRIVES]);
	fileBeingPrinted.MoveFrom(stack[stackPointer].fileState);
	drivesRelative = stack[stackPointer].drivesRelative;
	axesRelative = stack[stackPointer].axesRelative;
	doingFileMacro = stack[stackPointer].doingFileMacro;

	platform->PopMessageIndent();
}

// Move expects all axis movements to be absolute, and all
// extruder drive moves to be relative.  This function serves that.
// If applyLimits is true and we have homed the relevant axes, then we don't allow movement beyond the bed.
// Returns true if we have a legal move (or G92 argument), false if this gcode should be discarded

bool GCodes::LoadMoveBufferFromGCode(GCodeBuffer *gb, bool doingG92, bool applyLimits)
{
	// Zero every extruder drive as some drives may not be changed
	for (size_t drive = AXES; drive < DRIVES; drive++)
	{
		moveBuffer[drive] = 0.0;
	}

	// Deal with feed rate
	if (gb->Seen(feedrateLetter))
	{
		moveBuffer[DRIVES] = gb->GetFValue() * distanceScale * speedFactor;
	}

	// First do extrusion, and check, if we are extruding, that we have a tool to extrude with
	Tool* tool = reprap.GetCurrentTool();
	if (gb->Seen(extrudeLetter))
	{
		if (tool == NULL)
		{
			platform->Message(BOTH_ERROR_MESSAGE, "Attempting to extrude with no tool selected.\n");
			return false;
		}
		int eMoveCount = tool->DriveCount();
		if (eMoveCount > 0)
		{
			float eMovement[DRIVES - AXES];
			if (tool->Mixing())
			{
				float length = gb->GetFValue();
				for (size_t drive = 0; drive < tool->DriveCount(); drive++)
				{
					eMovement[drive] = length * tool->GetMix()[drive];
				}
			}
			else
			{
				gb->GetFloatArray(eMovement, eMoveCount);
				if (tool->DriveCount() != eMoveCount)
				{
					platform->Message(BOTH_ERROR_MESSAGE, "Wrong number of extruder drives for the selected tool: %s\n", gb->Buffer());
					return false;
				}
			}

			// Set the drive values for this tool.
			// zpl-2014-10-03: Do NOT check extruder temperatures here, because we may be executing queued codes like M116
			for (size_t eDrive = 0; eDrive < eMoveCount; eDrive++)
			{
				int drive = tool->Drive(eDrive);
				float moveArg = eMovement[eDrive] * distanceScale;
				if (doingG92)
				{
					moveBuffer[drive + AXES] = 0.0;		// no move required
					lastRawExtruderPosition[drive] = moveArg;
				}
				else if (drivesRelative)
				{
					moveBuffer[drive + AXES] = moveArg * extrusionFactors[drive];
					lastRawExtruderPosition[drive] += moveArg;
				}
				else
				{
					moveBuffer[drive + AXES] = (moveArg - lastRawExtruderPosition[drive]) * extrusionFactors[drive];
					lastRawExtruderPosition[drive] = moveArg;
				}
			}
		}
	}

	// Now the movement axes
	const Tool *currentTool = reprap.GetCurrentTool();
	for (size_t axis = 0; axis < AXES; axis++)
	{
		if (gb->Seen(axisLetters[axis]))
		{
			float moveArg = gb->GetFValue() * distanceScale;
			if (doingG92)
			{
				axisIsHomed[axis] = true;		// doing a G92 defines the absolute axis position
			}
			else
			{
				if (axesRelative)
				{
					moveArg += moveBuffer[axis];
				}
				else if (currentTool != NULL)
				{
					moveArg -= currentTool->GetOffset()[axis];// adjust requested position to compensate for tool offset
				}

				// If on a Cartesian printer and applying limits, limit all axes
				if (applyLimits && axisIsHomed[axis] && !reprap.GetMove()->IsDeltaMode())
				{
					if (moveArg < platform->AxisMinimum(axis))
					{
						moveArg = platform->AxisMinimum(axis);
					}
					else if (moveArg > platform->AxisMaximum(axis))
					{
						moveArg = platform->AxisMaximum(axis);
					}
				}
			}
			moveBuffer[axis] = moveArg;
		}
	}

	// If axes have been homed on a delta printer and this isn't a homing move, check for movements outside limits.
	// Skip this check if axes have not been homed, so that extruder-only moved are allowed before homing
	if (applyLimits && reprap.GetMove()->IsDeltaMode() && AllAxesAreHomed())
	{
		// Constrain the move to be within the build radius
		float diagonalSquared = fsquare(moveBuffer[X_AXIS]) + fsquare(moveBuffer[Y_AXIS]);
		if (diagonalSquared > reprap.GetMove()->GetDeltaParams().GetPrintRadiusSquared())
		{
			float factor = sqrtf(reprap.GetMove()->GetDeltaParams().GetPrintRadiusSquared() / diagonalSquared);
			moveBuffer[X_AXIS] *= factor;
			moveBuffer[Y_AXIS] *= factor;
		}

		// Constrain the end height of the move to be no greater than the homed height and no lower than -0.2mm
		moveBuffer[Z_AXIS] = max<float>(platform->AxisMinimum(Z_AXIS),
				min<float>(moveBuffer[Z_AXIS], reprap.GetMove()->GetDeltaParams().GetHomedHeight()));
	}

	return true;
}

// This function is called for a G Code that makes a move.
// If the Move class can't receive the move (i.e. things have to wait), return 0.
// If we have queued the move and the caller doesn't need to wait for it to complete, return 1.
// If we need to wait for the move to complete before doing another one (e.g. because endstops are checked in this move), return 2.

int GCodes::SetUpMove(GCodeBuffer *gb, StringRef& reply)
{
	// Last one gone yet?
	if (moveAvailable)
	{
		return 0;
	}

	// Check to see if the move is a 'homing' move that endstops are checked on.
	endStopsToCheck = 0;
	moveType = 0;
	if (gb->Seen('S'))
	{
		int ival = gb->GetIValue();
		if (ival == 1 || ival == 2)
		{
			moveType = ival;
		}

		if (ival == 1)
		{
			for (size_t i = 0; i < AXES; ++i)
			{
				if (gb->Seen(axisLetters[i]))
				{
					endStopsToCheck |= (1 << i);
				}
			}
		}
	}

	if (reprap.GetMove()->IsDeltaMode())
	{
		// Extra checks to avoid damaging delta printers
		if (moveType != 0 && !axesRelative)
		{
			// We have been asked to do a move without delta mapping on a delta machine, but the move is not relative.
			// This may be damaging and is almost certainly a user mistake, so ignore the move.
			reply.copy("Attempt to move the motors of a delta printer to absolute positions\n");
			return 1;
		}

		if (moveType == 0 && !AllAxesAreHomed())
		{
			// The user may be attempting to move a delta printer to an XYZ position before homing the axes
			// This may be damaging and is almost certainly a user mistake, so ignore the move. But allow extruder-only moves.
			if (gb->Seen(axisLetters[X_AXIS]) || gb->Seen(axisLetters[Y_AXIS]) || gb->Seen(axisLetters[Z_AXIS]))
			{
				reply.copy("Attempt to move the head of a delta printer before homing the towers\n");
				return 1;
			}
		}
	}

	// Load the last position and feed rate into moveBuffer
	reprap.GetMove()->GetCurrentUserPosition(moveBuffer, moveType);

	moveBuffer[DRIVES] *= speedFactorChange;		// account for any change in the speed factor since the last move
	speedFactorChange = 1.0;

	// Load the move buffer with either the absolute movement required or the relative movement required
	moveAvailable = LoadMoveBufferFromGCode(gb, false, limitAxes && moveType == 0);
	if (moveAvailable)
	{
		moveFilePos = (gb == fileGCode) ? filePos : noFilePosition;
		//debugPrintf("Queue move pos %u\n", moveFilePos);
	}
	return (moveType != 0) ? 2 : 1;
}

// The Move class calls this function to find what to do next.

bool GCodes::ReadMove(float m[], EndstopChecks& ce, uint8_t& rMoveType, FilePosition& fPos)
{
	if (!moveAvailable)
	{
		return false;
	}

	for (size_t i = 0; i <= DRIVES; i++)			// 1 more for feedrate
	{
		m[i] = moveBuffer[i];
	}
	ce = endStopsToCheck;
	rMoveType = moveType;
	fPos = moveFilePos;
	ClearMove();
	return true;
}

void GCodes::ClearMove()
{
	moveAvailable = false;
	endStopsToCheck = 0;
	moveType = 0;
}

// Run a file macro. Prior to calling this, 'state' must be set to the state we want to enter when the macro has been completed.
// Return true if the file was found.
bool GCodes::DoFileMacro(const char* fileName, bool reportMissing)
{
	FileStore *f = platform->GetFileStore((fileName[0] == '/') ? "0:" : platform->GetSysDir(), fileName, false);
	if (f == NULL)
	{
		if (reportMissing)
		{
			// Don't use snprintf into scratchString here, because fileName may be aliased to scratchString
			platform->Message(BOTH_ERROR_MESSAGE, "Macro file %s not found.\n", fileName);
		}
		return false;
	}

	Push();
	fileBeingPrinted.Set(f);
	doingFileMacro = true;
	fileMacroGCode->Init();
	state = GCodeState::normal;
	return true;
}

void GCodes::FileMacroCyclesReturn()
{
	if (doingFileMacro)
	{
		Pop();
		fileMacroGCode->Init();
	}
}

// To execute any move, call this until it returns true.
// moveToDo[] entries corresponding with false entries in action[] will
// be ignored.  Recall that moveToDo[DRIVES] should contain the feedrate
// you want (if action[DRIVES] is true).
bool GCodes::DoCannedCycleMove(EndstopChecks ce)
{
	if (AllMovesAreFinishedAndMoveBufferIsLoaded())
	{
		if (cannedCycleMoveQueued)		// if the move has already been queued, it must have finished
		{
			Pop();
			cannedCycleMoveQueued = false;
			return true;
		}

		// Otherwise, the move has not been queued yet
		Push();

		for (size_t drive = 0; drive <= DRIVES; drive++)
		{
			if (activeDrive[drive])
			{
				moveBuffer[drive] = moveToDo[drive];
			}
		}
		endStopsToCheck = ce;
		cannedCycleMoveQueued = true;
		moveAvailable = true;
	}
	return false;
}

// This sets positions.  I.e. it handles G92.
bool GCodes::SetPositions(GCodeBuffer *gb)
{
	if (!AllMovesAreFinishedAndMoveBufferIsLoaded())
	{
		return false;
	}

	reprap.GetMove()->GetCurrentUserPosition(moveBuffer, 0);		// make sure move buffer is up to date
	if (LoadMoveBufferFromGCode(gb, true, false))
	{
		SetPositions(moveBuffer);
	}

	return true;
}

// Offset the axes by the X, Y, and Z amounts in the M code in gb.  Say the machine is at [10, 20, 30] and
// the offsets specified are [8, 2, -5].  The machine will move to [18, 22, 25] and henceforth consider that point
// to be [10, 20, 30].
bool GCodes::OffsetAxes(GCodeBuffer* gb)
{
	if (!offSetSet)
	{
		if (!AllMovesAreFinishedAndMoveBufferIsLoaded())
			return false;
		for (size_t drive = 0; drive <= DRIVES; drive++)
		{
			if (drive < AXES || drive == DRIVES)
			{
				record[drive] = moveBuffer[drive];
				moveToDo[drive] = moveBuffer[drive];
			}
			else
			{
				record[drive] = 0.0;
				moveToDo[drive] = 0.0;
			}
			activeDrive[drive] = false;
		}

		for (size_t axis = 0; axis < AXES; axis++)
		{
			if (gb->Seen(axisLetters[axis]))
			{
				moveToDo[axis] += gb->GetFValue();
				activeDrive[axis] = true;
			}
		}

		if (gb->Seen(feedrateLetter)) // Has the user specified a feedrate?
		{
			moveToDo[DRIVES] = gb->GetFValue();
			activeDrive[DRIVES] = true;
		}

		offSetSet = true;
	}

	if (DoCannedCycleMove(0))
	{
		//LoadMoveBufferFromArray(record);
		for (size_t drive = 0; drive <= DRIVES; drive++)
		{
			moveBuffer[drive] = record[drive];
		}
		reprap.GetMove()->SetLiveCoordinates(record);	// This doesn't transform record
		reprap.GetMove()->SetPositions(record);			// This does
		offSetSet = false;
		return true;
	}

	return false;
}

// This lifts Z a bit, moves to the probe XY coordinates (obtained by a call to GetProbeCoordinates() ),
// probes the bed height, and records the Z coordinate probed.  If you want to program any general
// internal canned cycle, this shows how to do it.
// On entry, probePointIndex specifies which of the points this is.
bool GCodes::DoSingleZProbeAtPoint(int probePointIndex)
{
	reprap.GetMove()->SetIdentityTransform(); 		// It doesn't matter if these are called repeatedly

	for (size_t drive = 0; drive <= DRIVES; drive++)
	{
		activeDrive[drive] = false;
	}

	switch (cannedCycleMoveCount)
	{
	case 0: // Move Z to the dive height. This only does anything on the first move; on all the others Z is already there
		moveToDo[Z_AXIS] = platform->GetZProbeDiveHeight() + max<float>(platform->ZProbeStopHeight(), 0.0);
		activeDrive[Z_AXIS] = true;
		moveToDo[DRIVES] = platform->MaxFeedrate(Z_AXIS);
		activeDrive[DRIVES] = true;
		if (DoCannedCycleMove(0))
		{
			cannedCycleMoveCount++;
		}
		return false;

	case 1:	// Move to the correct XY coordinates
		GetProbeCoordinates(probePointIndex, moveToDo[X_AXIS], moveToDo[Y_AXIS], moveToDo[Z_AXIS]);
		activeDrive[X_AXIS] = true;
		activeDrive[Y_AXIS] = true;
		// NB - we don't use the Z value
		moveToDo[DRIVES] = platform->MaxFeedrate(X_AXIS);
		activeDrive[DRIVES] = true;
		if (DoCannedCycleMove(0))
		{
			cannedCycleMoveCount++;
		}
		return false;

	case 2:	// Probe the bed
		{
			const float height = (axisIsHomed[Z_AXIS])
									? 2 * platform->GetZProbeDiveHeight()			// Z axis has been homed, so no point in going very far
									: 1.1 * platform->AxisTotalLength(Z_AXIS);		// Z axis not homed yet, so treat this as a homing move
			switch(DoZProbe(height))
			{
			case 0:
				// Z probe is already triggered at the start of the move, so abandon the probe and record an error
				platform->Message(BOTH_ERROR_MESSAGE, "Z probe warning: probe already triggered at start of probing move\n");
				cannedCycleMoveCount++;
				reprap.GetMove()->SetZBedProbePoint(probePointIndex, platform->GetZProbeDiveHeight(), true, true);
				break;

			case 1:
				if (axisIsHomed[Z_AXIS])
				{
					lastProbedZ = moveBuffer[Z_AXIS] - platform->ZProbeStopHeight();
				}
				else
				{
					// The Z axis has not yet been homed, so treat this probe as a homing move.
					moveBuffer[Z_AXIS] = platform->ZProbeStopHeight();
					SetPositions(moveBuffer);
					axisIsHomed[Z_AXIS] = true;
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
		moveToDo[Z_AXIS] = platform->GetZProbeDiveHeight() + max<float>(platform->ZProbeStopHeight(), 0.0);
		activeDrive[Z_AXIS] = true;
		moveToDo[DRIVES] = platform->MaxFeedrate(Z_AXIS);
		activeDrive[DRIVES] = true;
		if (DoCannedCycleMove(0))
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
bool GCodes::DoSingleZProbe()
{
	switch (DoZProbe(1.1 * platform->AxisTotalLength(Z_AXIS)))
	{
	case 0:		// failed
		return true;

	case 1:		// success
		moveBuffer[Z_AXIS] = platform->ZProbeStopHeight();
		SetPositions(moveBuffer);
		axisIsHomed[Z_AXIS] = true;
		lastProbedZ = 0.0;
		return true;

	default:	// not finished yet
		return false;
	}
}

// Do a Z probe cycle up to the maximum specified distance.
// Returns -1 if not complete yet
// Returns 0 if failed
// Returns 1 if success, with lastProbedZ set to the height we stopped at and the current position in moveBuffer
int GCodes::DoZProbe(float distance)
{
	if (platform->GetZProbeType() == 5)
	{
		const ZProbeParameters& params = platform->GetZProbeParameters();
		return reprap.GetMove()->DoDeltaProbe(params.param1, params.param2, platform->HomeFeedRate(Z_AXIS), distance);
	}
	else
	{
		if (!cannedCycleMoveQueued && reprap.GetPlatform()->GetZProbeResult() == EndStopHit::lowHit)
		{
			return 0;
		}

		// Do a normal canned cycle Z movement with Z probe enabled
		for (size_t drive = 0; drive <= DRIVES; drive++)
		{
			activeDrive[drive] = false;
		}

		moveToDo[Z_AXIS] = -distance;
		activeDrive[Z_AXIS] = true;
		moveToDo[DRIVES] = platform->HomeFeedRate(Z_AXIS);
		activeDrive[DRIVES] = true;

		if (DoCannedCycleMove(ZProbeActive))
		{
			return 1;
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
bool GCodes::SetSingleZProbeAtAPosition(GCodeBuffer *gb, StringRef& reply)
{
	if (!AllMovesAreFinishedAndMoveBufferIsLoaded())
		return false;

	if (!gb->Seen('P'))
		return DoSingleZProbe();

	int probePointIndex = gb->GetIValue();
	if (probePointIndex < 0 || probePointIndex >= MaxProbePoints)
	{
		reprap.GetPlatform()->Message(BOTH_ERROR_MESSAGE, "Z probe point index out of range.\n");
		return true;
	}

	float x = (gb->Seen(axisLetters[X_AXIS])) ? gb->GetFValue() : moveBuffer[X_AXIS];
	float y = (gb->Seen(axisLetters[Y_AXIS])) ? gb->GetFValue() : moveBuffer[Y_AXIS];
	float z = (gb->Seen(axisLetters[Z_AXIS])) ? gb->GetFValue() : moveBuffer[Z_AXIS];

	reprap.GetMove()->SetXBedProbePoint(probePointIndex, x);
	reprap.GetMove()->SetYBedProbePoint(probePointIndex, y);

	if (z > SILLY_Z_VALUE)
	{
		reprap.GetMove()->SetZBedProbePoint(probePointIndex, z, false, false);
		if (gb->Seen('S'))
		{
			zProbesSet = true;
			reprap.GetMove()->FinishedBedProbing(gb->GetIValue(), reply);
		}
		return true;
	}
	else
	{
		if (DoSingleZProbeAtPoint(probePointIndex))
		{
			if (gb->Seen('S'))
			{
				zProbesSet = true;
				int sParam = gb->GetIValue();
				if (sParam == 1)
				{
					// G30 with a silly Z value and S=1 is equivalent to G30 with no parameters in that it sets the current Z height
					// This is useful because it adjusts the XY position to account for the probe offset.
					moveBuffer[Z_AXIS] += lastProbedZ;
					SetPositions(moveBuffer);
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

// This probes multiple points on the bed, then sets the bed transformation to compensate for the bed not quite being the plane Z = 0.
// Called to execute a G32 command.
void GCodes::SetBedEquationWithProbe(int sParam, StringRef& reply)
{
	// zpl-2014-10-09: In order to stay compatible with old firmware versions, only execute bed.g if it is actually present in the sys directory
	if (!DoFileMacro(SET_BED_EQUATION, false))
	{
		state = GCodeState::setBed1;		// no bed.g file, so use the coordinates specified by M557
	}
}

// This returns the (X, Y) points to probe the bed at probe point count.  When probing, it returns false.
// If called after probing has ended it returns true, and the Z coordinate probed is also returned.
bool GCodes::GetProbeCoordinates(int count, float& x, float& y, float& z) const
{
	const ZProbeParameters& rp = platform->GetZProbeParameters();
	x = reprap.GetMove()->XBedProbePoint(count) - rp.xOffset;
	y = reprap.GetMove()->YBedProbePoint(count) - rp.yOffset;
	z = reprap.GetMove()->ZBedProbePoint(count);
	return zProbesSet;
}

bool GCodes::SetPrintZProbe(GCodeBuffer* gb, StringRef& reply)
{
	if (!AllMovesAreFinishedAndMoveBufferIsLoaded())
		return false;

	ZProbeParameters params = platform->GetZProbeParameters();
	bool seen = false;
	if (gb->Seen(axisLetters[X_AXIS]))
	{
		params.xOffset = gb->GetFValue();
		seen = true;
	}
	if (gb->Seen(axisLetters[Y_AXIS]))
	{
		params.yOffset = gb->GetFValue();
		seen = true;
	}
	if (gb->Seen(axisLetters[Z_AXIS]))
	{
		params.height = gb->GetFValue();
		seen = true;
	}
	if (gb->Seen('P'))
	{
		params.adcValue = gb->GetIValue();
		seen = true;
	}
	if (gb->Seen('C'))
	{
		params.temperatureCoefficient = gb->GetFValue();
		seen = true;
		if (gb->Seen('S'))
		{
			params.calibTemperature = gb->GetFValue();
		}
		else
		{
			// Use the current bed temperature as the calibration temperature if no value was provided
			params.calibTemperature = platform->GetTemperature(HOT_BED);
		}
	}

	if (seen)
	{
		platform->SetZProbeParameters(params);
	}
	else
	{
		int v0 = platform->ZProbe();
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

// Return the current coordinates as a printable string.  Coordinates
// are updated at the end of each movement, so this won't tell you
// where you are mid-movement.

//Fixed to deal with multiple extruders

void GCodes::GetCurrentCoordinates(StringRef& s) const
{
	float liveCoordinates[DRIVES];
	reprap.GetMove()->LiveCoordinates(liveCoordinates);
	const Tool *currentTool = reprap.GetCurrentTool();
	if (currentTool != NULL)
	{
		const float *offset = currentTool->GetOffset();
		for (size_t i = 0; i < AXES; ++i)
		{
			liveCoordinates[i] += offset[i];
		}
	}

	s.printf("X:%.2f Y:%.2f Z:%.2f ", liveCoordinates[X_AXIS], liveCoordinates[Y_AXIS], liveCoordinates[Z_AXIS]);
	for (size_t i = AXES; i < DRIVES; i++)
	{
		s.catf("E%u:%.1f ", i - AXES, liveCoordinates[i]);
	}
}

bool GCodes::OpenFileToWrite(const char* directory, const char* fileName, GCodeBuffer *gb)
{
	fileBeingWritten = platform->GetFileStore(directory, fileName, true);
	eofStringCounter = 0;
	if (fileBeingWritten == NULL)
	{
		platform->Message(BOTH_ERROR_MESSAGE, "Can't open GCode file \"%s\" for writing.\n", fileName);
		return false;
	}
	else
	{
		gb->SetWritingFileDirectory(directory);
		return true;
	}
}

void GCodes::WriteHTMLToFile(char b, GCodeBuffer *gb)
{
	if (fileBeingWritten == NULL)
	{
		platform->Message(BOTH_ERROR_MESSAGE, "Attempt to write to a null file.\n");
		return;
	}

	if (eofStringCounter != 0 && b != eofString[eofStringCounter])
	{
		for (size_t i = 0; i < eofStringCounter; ++i)
		{
			fileBeingWritten->Write(eofString[i]);
		}
		eofStringCounter = 0;
	}

	if (b == eofString[eofStringCounter])
	{
		eofStringCounter++;
		if (eofStringCounter >= eofStringLength)
		{
			fileBeingWritten->Close();
			fileBeingWritten = NULL;
			gb->SetWritingFileDirectory(NULL);
			const char* r = (platform->Emulating() == marlin) ? "Done saving file." : "";
			HandleReply(false, gb, r, 'M', 560, false);
			return;
		}
	}
	else
	{
		fileBeingWritten->Write(b);
	}
}

void GCodes::WriteGCodeToFile(GCodeBuffer *gb)
{
	if (fileBeingWritten == NULL)
	{
		platform->Message(BOTH_ERROR_MESSAGE, "Attempt to write to a null file.\n");
		return;
	}

	// End of file?
	if (gb->Seen('M'))
	{
		if (gb->GetIValue() == 29)
		{
			fileBeingWritten->Close();
			fileBeingWritten = NULL;
			gb->SetWritingFileDirectory(NULL);
			const char* r = (platform->Emulating() == marlin) ? "Done saving file." : "";
			HandleReply(false, gb, r, 'M', 29, false);
			return;
		}
	}

	// Resend request?
	if (gb->Seen('G'))
	{
		if (gb->GetIValue() == 998)
		{
			if (gb->Seen('P'))
			{
				scratchString.printf("%d\n", gb->GetIValue());
				HandleReply(false, gb, scratchString.Pointer(), 'G', 998, true);
				return;
			}
		}
	}

	fileBeingWritten->Write(gb->Buffer());
	fileBeingWritten->Write('\n');
	HandleReply(false, gb, "", 'G', 1, false);
}

// Set up a file to print, but don't print it yet.
void GCodes::QueueFileToPrint(const char* fileName)
{
	FileStore *f = platform->GetFileStore(platform->GetGCodeDir(), fileName, false);
	if (f != NULL)
	{
		// Cancel current print if there is any
		if (!reprap.GetPrintMonitor()->IsPrinting())
		{
			CancelPrint();
		}

		fileGCode->SetToolNumberAdjust(0);	// clear tool number adjustment

		// Reset all extruder positions when starting a new print
		for (size_t extruder = AXES; extruder < DRIVES; extruder++)
		{
			lastRawExtruderPosition[extruder - AXES] = 0.0;
		}

		fileToPrint.Set(f);
	}
	else
	{
		platform->Message(BOTH_ERROR_MESSAGE, "GCode file \"%s\" not found\n", fileName);
	}
}

void GCodes::DeleteFile(const char* fileName)
{
	if (!platform->GetMassStorage()->Delete(platform->GetGCodeDir(), fileName))
	{
		platform->Message(BOTH_ERROR_MESSAGE, "Could not delete file \"%s\"\n", fileName);
	}
}

// Send the config file to USB in response to an M503 command.
// This is not used for processing M503 requests received via the webserver.
bool GCodes::SendConfigToLine()
{
	if (configFile == NULL)
	{
		configFile = platform->GetFileStore(platform->GetSysDir(), platform->GetConfigFile(), false);
		if (configFile == NULL)
		{
			platform->Message(BOTH_ERROR_MESSAGE, "Configuration file not found\n");
			return true;
		}
		platform->GetLine()->Write('\n', true);
	}

	char b;
	while (configFile->Read(b))
	{
		platform->GetLine()->Write(b, true);
		if (b == '\n')
			return false;
	}

	platform->GetLine()->Write('\n', true);
	configFile->Close();
	configFile = NULL;
	return true;
}

// Function to handle dwell delays.  Return true for dwell finished, false otherwise.
bool GCodes::DoDwell(GCodeBuffer *gb)
{
	if (!gb->Seen('P'))
		return true;  // No time given - throw it away

	float dwell = 0.001 * (float) gb->GetLValue(); // P values are in milliseconds; we need seconds

			// Wait for all the queued moves to stop
	if (!reprap.GetMove()->AllMovesAreFinished())
		return false;

	if (simulating)
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

void GCodes::SetOrReportOffsets(StringRef& reply, GCodeBuffer *gb)
{
	if (gb->Seen('P'))
	{
		int8_t toolNumber = gb->GetIValue();
		toolNumber += gb->GetToolNumberAdjust();
		Tool* tool = reprap.GetTool(toolNumber);
		if (tool == NULL)
		{
			reply.printf("Attempt to set/report offsets and temperatures for non-existent tool: %d\n", toolNumber);
			return;
		}

		// Deal with setting offsets
		float offset[AXES];
		for (size_t i = 0; i < AXES; ++i)
		{
			offset[i] = tool->GetOffset()[i];
		}

		bool settingOffset = false;
		if (gb->Seen('X'))
		{
			offset[X_AXIS] = gb->GetFValue();
			settingOffset = true;
		}
		if (gb->Seen('Y'))
		{
			offset[Y_AXIS] = gb->GetFValue();
			settingOffset = true;
		}
		if (gb->Seen('Z'))
		{
			offset[Z_AXIS] = gb->GetFValue();
			settingOffset = true;
		}
		if (settingOffset)
		{
			tool->SetOffset(offset);
		}

		// Deal with setting temperatures
		bool settingTemps = false;
		int hCount = tool->HeaterCount();
		float standby[HEATERS];
		float active[HEATERS];
		if (hCount > 0)
		{
			tool->GetVariables(standby, active);
			if (gb->Seen('R'))
			{
				gb->GetFloatArray(standby, hCount);
				settingTemps = true;
			}
			if (gb->Seen('S'))
			{
				gb->GetFloatArray(active, hCount);
				settingTemps = true;
			}

			if (settingTemps && !simulating)
			{
				tool->SetVariables(standby, active);
			}
		}

		if (!settingOffset && !settingTemps)
		{
			// Print offsets and temperatures
			reply.printf("Tool %d offsets: X%.1f Y%.1f Z%.1f", toolNumber, offset[X_AXIS], offset[Y_AXIS],
					offset[Z_AXIS]);
			if (hCount != 0)
			{
				reply.cat(", active/standby temperature(s):");
				for (int8_t heater = 0; heater < hCount; heater++)
				{
					reply.catf(" %.1f/%.1f", active[heater], standby[heater]);
				}
			}
		}
	}
}

void GCodes::ManageTool(GCodeBuffer *gb, StringRef& reply)
{
	if (!gb->Seen('P'))
	{
		// DC temporary code to allow tool numbers to be adjusted so that we don't need to edit multi-media files generated by slic3r
		if (gb->Seen('S'))
		{
			int adjust = gb->GetIValue();
			gb->SetToolNumberAdjust(adjust);
		}
		return;
	}

	// Check tool number
	bool seen = false;
	int toolNumber = gb->GetLValue();
	if (toolNumber < 0)
	{
		platform->Message(BOTH_ERROR_MESSAGE, "Tool number must be positive!\n");
		return;
	}

	// Check drives
	long drives[DRIVES - AXES];  // There can never be more than we have...
	int dCount = DRIVES - AXES;  // Sets the limit and returns the count
	if (gb->Seen('D'))
	{
		gb->GetLongArray(drives, dCount);
		seen = true;
	}
	else
	{
		dCount = 0;
	}

	// Check heaters
	long heaters[HEATERS];
	int hCount = HEATERS;
	if (gb->Seen('H'))
	{
		gb->GetLongArray(heaters, hCount);
		seen = true;
	}
	else
	{
		hCount = 0;
	}

	if (seen)
	{
		// Add or delete tool
		// M563 P# D-1 H-1 removes an existing tool
		if (dCount == 1 && hCount == 1 && drives[0] == -1 && heaters[0] == -1)
		{
			Tool *tool = reprap.GetTool(toolNumber);
			reprap.DeleteTool(tool);
		}
		else
		{
			Tool* tool = new Tool(toolNumber, drives, dCount, heaters, hCount);
			reprap.AddTool(tool);
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

void GCodes::SetEthernetAddress(GCodeBuffer *gb, int mCode)
{
	byte eth[4];
	const char* ipString = gb->GetString();
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
				platform->Message(BOTH_ERROR_MESSAGE, "Dud IP address: %s\n", gb->Buffer());
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
			platform->Message(BOTH_ERROR_MESSAGE, "Setting ether parameter - dud code.\n");
		}
	}
	else
	{
		platform->Message(BOTH_ERROR_MESSAGE, "Dud IP address: %s\n", gb->Buffer());
	}
}

void GCodes::SetMACAddress(GCodeBuffer *gb)
{
	uint8_t mac[6];
	const char* ipString = gb->GetString();
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
				platform->Message(BOTH_ERROR_MESSAGE, "Dud MAC address: %s\n", gb->Buffer());
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
		platform->Message(BOTH_ERROR_MESSAGE, "Dud MAC address: %s\n", gb->Buffer());
	}
}

// Handle sending a reply back to the appropriate interface(s).
// Note that 'reply' may be null.
void GCodes::HandleReply(bool error, const GCodeBuffer *gb, const char* reply, char gMOrT, int code, bool resend)
{
	// UART device, may be used for auxiliary purposes e.g. LCD control panel
	if (gb == auxGCode && reply != nullptr && reply[0] == '{')
	{
		// Command was received from the aux interface (LCD display) and looks to be JSON-formatted, so send the response only to the aux interface.
		platform->GetAux()->Write(reply);
		return;
	}

	// Deal with sending a reply to the web interface and/or Panel Due via the polling mechanism.
	// The browser and PanelDue only fetch replies once a second or so. Therefore, when we send a web command in the middle of an SD card print,
	// in order to be sure that we see the reply in the web interface or PanelDue, we do not allow empty responses to overwrite more useful responses.
	if (reply != nullptr && reply[0] != 0// don't send empty replies to the web server, they overwrite more useful replies
	&& (gMOrT != 'M' || (code != 111 && code != 122))// web server reply for M111 and M122 is handled before we get here
			&& (gMOrT != 'M' || (code != 105 && code != 20) || gb == webGCode)// don't echo M105 requests and file lists from Pronterface etc. to the web server
			)
	{
		platform->Message((error) ? WEB_ERROR_MESSAGE : WEB_MESSAGE, reply);
	}

	const Compatibility c = (gb == serialGCode) ? platform->Emulating() : me;
	const char* response = (resend) ? "rs " : "ok";
	const char* emulationType = 0;

	switch (c)
	{
	case me:
	case reprapFirmware:
		if (error)
		{
			platform->GetLine()->Write("Error: ");
		}
		if (reply)
		{
			platform->GetLine()->Write(reply);
		}
		return;

	case marlin:
		if (gMOrT == 'M' && code == 20)
		{
			platform->GetLine()->Write("Begin file list\n");
			if (reply)
			{
				platform->GetLine()->Write(reply);
			}
			platform->GetLine()->Write("End file list\n");
			platform->GetLine()->Write(response);
			platform->GetLine()->Write('\n');
			return;
		}

		if (gMOrT == 'M' && code == 28)
		{
			platform->GetLine()->Write(response);
			platform->GetLine()->Write('\n');
			if (reply)
			{
				platform->GetLine()->Write(reply);
			}
			return;
		}

		if ((gMOrT == 'M' && code == 105) || (gMOrT == 'G' && code == 998))
		{
			platform->GetLine()->Write(response);
			if (reply)
			{
				platform->GetLine()->Write(' ');
				platform->GetLine()->Write(reply);
			}
			return;
		}

		if (reply && reply[0])
		{
			platform->GetLine()->Write(reply);
		}
		platform->GetLine()->Write(response);
		platform->GetLine()->Write('\n');
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
		platform->Message(HOST_MESSAGE, "Emulation of %s is not yet supported.\n", emulationType);// don't send this one to the web as well, it concerns only the USB interface
	}
}

// Set PID parameters (M301 or M304 command). 'heater' is the default heater number to use.
void GCodes::SetPidParameters(GCodeBuffer *gb, int heater, StringRef& reply)
{
	if (gb->Seen('H'))
	{
		heater = gb->GetIValue();
	}

	if (heater >= 0 && heater < HEATERS)
	{
		PidParameters pp = platform->GetPidParameters(heater);
		bool seen = false;
		if (gb->Seen('P'))
		{
			pp.kP = gb->GetFValue();
			seen = true;
		}
		if (gb->Seen('I'))
		{
			pp.kI = gb->GetFValue() / platform->HeatSampleTime();
			seen = true;
		}
		if (gb->Seen('D'))
		{
			pp.kD = gb->GetFValue() * platform->HeatSampleTime();
			seen = true;
		}
		if (gb->Seen('T'))
		{
			pp.kT = gb->GetFValue();
			seen = true;
		}
		if (gb->Seen('S'))
		{
			pp.kS = gb->GetFValue();
			seen = true;
		}
		if (gb->Seen('W'))
		{
			pp.pidMax = gb->GetFValue();
			seen = true;
		}
		if (gb->Seen('B'))
		{
			pp.fullBand = gb->GetFValue();
			seen = true;
		}

		if (seen)
		{
			platform->SetPidParameters(heater, pp);
		}
		else
		{
			reply.printf("Heater %d P:%.2f I:%.3f D:%.2f T:%.2f S:%.2f W:%.1f B:%.1f\n",
					heater, pp.kP, pp.kI * platform->HeatSampleTime(), pp.kD / platform->HeatSampleTime(),
					pp.kT, pp.kS, pp.pidMax, pp.fullBand);
		}
	}
}

void GCodes::SetHeaterParameters(GCodeBuffer *gb, StringRef& reply)
{
	if (gb->Seen('P'))
	{
		int heater = gb->GetIValue();
		if (heater >= 0 && heater < HEATERS)
		{
			PidParameters pp = platform->GetPidParameters(heater);
			bool seen = false;

			// We must set the 25C resistance and beta together in order to calculate Rinf. Check for these first.
			float r25, beta;
			if (gb->Seen('T'))
			{
				r25 = gb->GetFValue();
				seen = true;
			}
			else
			{
				r25 = pp.GetThermistorR25();
			}
			if (gb->Seen('B'))
			{
				beta = gb->GetFValue();
				seen = true;
			}
			else
			{
				beta = pp.GetBeta();
			}

			if (seen)	// if see R25 or Beta or both
			{
				pp.SetThermistorR25AndBeta(r25, beta);					// recalculate Rinf
			}

			// Now do the other parameters
			if (gb->Seen('R'))
			{
				pp.thermistorSeriesR = gb->GetFValue();
				seen = true;
			}
			if (gb->Seen('L'))
			{
				pp.adcLowOffset = gb->GetFValue();
				seen = true;
			}
			if (gb->Seen('H'))
			{
				pp.adcHighOffset = gb->GetFValue();
				seen = true;
			}
			if (gb->Seen('X'))
			{
				int thermistor = gb->GetIValue();
				if (thermistor >= 0 && thermistor < HEATERS)
				{
					platform->SetThermistorNumber(heater, thermistor);
				}
				else
				{
					platform->Message(BOTH_ERROR_MESSAGE, "Thermistor number %d is out of range\n", thermistor);
				}
				seen = true;
			}

			if (seen)
			{
				platform->SetPidParameters(heater, pp);
			}
			else
			{
				reply.printf("T:%.1f B:%.1f R:%.1f L:%.1f H:%.1f X:%d\n", r25, beta, pp.thermistorSeriesR,
						pp.adcLowOffset, pp.adcHighOffset, platform->GetThermistorNumber(heater));
			}
		}
		else
		{
			platform->Message(BOTH_ERROR_MESSAGE, "Heater number %d is out of range\n", heater);
		}
	}
}

void GCodes::SetToolHeaters(Tool *tool, float temperature)
{
	if (tool == NULL)
	{
		platform->Message(BOTH_ERROR_MESSAGE, "Setting temperature: no tool selected.\n");
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

// If the code to act on is completed, this returns true,
// otherwise false.  It is called repeatedly for a given
// code until it returns true for that code.

bool GCodes::ActOnCode(GCodeBuffer *gb, StringRef& reply)
{
	// Discard empty buffers right away
	if (gb->IsEmpty())
	{
		return true;
	}

	gbCurrent = gb;

	// M-code parameters might contain letters T and G, e.g. in filenames.
	// dc42 assumes that G-and T-code parameters never contain the letter M.
	// Therefore we must check for an M-code first.
	if (gb->Seen('M'))
	{
		return HandleMcode(gb, reply);
	}
	// dc42 doesn't think a G-code parameter ever contains letter T, or a T-code ever contains letter G.
	// So it doesn't matter in which order we look for them.
	if (gb->Seen('G'))
	{
		return HandleGcode(gb, reply);
	}
	if (gb->Seen('T'))
	{
		return HandleTcode(gb, reply);
	}

	// An invalid or queued buffer gets discarded
	HandleReply(false, gb, nullptr, 'X', 0, false);
	return true;
}

bool GCodes::HandleGcode(GCodeBuffer* gb, StringRef& reply)
{
	bool result = true;
	bool error = false;
	bool resend = false;

	int code = gb->GetIValue();
	if (simulating && code != 0 && code != 1 && code != 4 && code != 10 && code != 20 && code != 21 && code != 90
			&& code != 91 && code != 92)
	{
		return true;			// we only simulate some gcodes
	}

	switch (code)
	{
	case 0: // There are no rapid moves...
	case 1: // Ordinary move
		// Check for 'R' parameter here to go back to the coordinates at which the print was paused
		if (gb->Seen('R') && gb->GetIValue() > 0 && (IsPaused() || IsPausing() || IsResuming()))
		{
			if (moveAvailable)
			{
				return false;
			}
			for (size_t axis = 0; axis < AXES; ++axis)
			{
				float offset = gb->Seen(axisLetters[axis]) ? gb->GetFValue() * distanceScale : 0.0;
				moveBuffer[axis] = pausedMoveBuffer[axis] + offset;
			}
			for (size_t drive = AXES; drive < DRIVES; ++drive)
			{
				moveBuffer[drive] = 0.0;
			}
			if (gb->Seen(feedrateLetter))
			{
				moveBuffer[DRIVES] = gb->GetFValue();
			}
			moveAvailable = true;
		}
		else
		{
			int res = SetUpMove(gb, reply);
			if (res == 2)
			{
				state = GCodeState::waitingForMoveToComplete;
			}
			result = (res != 0);
		}
		break;

	case 4: // Dwell
		result = DoDwell(gb);
		break;

	case 10: // Set/report offsets and temperatures
		SetOrReportOffsets(reply, gb);
		break;

	case 20: // Inches (which century are we living in, here?)
		distanceScale = INCH_TO_MM;
		break;

	case 21: // mm
		distanceScale = 1.0;
		break;

	case 28: // Home
		if (!AllMovesAreFinishedAndMoveBufferIsLoaded())
		{
			return false;
		}

		if (reprap.GetMove()->IsDeltaMode())
		{
			SetAllAxesNotHomed();
			DoFileMacro(HOME_DELTA_G);
		}
		else
		{
			toBeHomed = 0;
			for (size_t axis = 0; axis < AXES; ++axis)
			{
				if (gb->Seen(axisLetters[axis]))
				{
					toBeHomed |= (1 << axis);
					axisIsHomed[axis] = false;
				}
			}

			if (toBeHomed == 0 || toBeHomed == ((1 << X_AXIS) | (1 << Y_AXIS) | (1 << Z_AXIS)))
			{
				// Homing everything
				SetAllAxesNotHomed();
				DoFileMacro(HOME_ALL_G);
			}
			else if (   platform->MustHomeXYBeforeZ()
					 && ((toBeHomed & (1 << Z_AXIS)) != 0)
					 && (   (((toBeHomed & (1 << X_AXIS)) == 0) && !axisIsHomed[X_AXIS])
					 	 || (((toBeHomed & (1 << Y_AXIS)) == 0) && !axisIsHomed[Y_AXIS])
						)
					)
			{
				// We can only home Z if both X and Y have already been homed or are being homed
				reply.copy("Must home X and Y before homing Z");
				error = true;
			}
			else
			{
				state = GCodeState::homing;
			}
		}
		break;

	case 30: // Z probe/manually set at a position and set that as point P
		if (!AllMovesAreFinishedAndMoveBufferIsLoaded())
		{
			return false;
		}
		result = SetSingleZProbeAtAPosition(gb, reply);
		break;

	case 31: // Return the probe value, or set probe variables
		if (!AllMovesAreFinishedAndMoveBufferIsLoaded())
		{
			return false;
		}
		result = SetPrintZProbe(gb, reply);
		break;

	case 32: // Probe Z at multiple positions and generate the bed transform
		if (!AllMovesAreFinishedAndMoveBufferIsLoaded())
		{
			return false;
		}

		if (!(axisIsHomed[X_AXIS] && axisIsHomed[Y_AXIS]))
		{
			// We can only do bed levelling if X and Y have already been homed
			reply.copy("Must home X and Y before bed probing\n");
			error = true;
		}
		else
		{
			int sParam = (gb->Seen('S')) ? gb->GetIValue() : 0;
			SetBedEquationWithProbe(sParam, reply);
		}
		break;

	case 90: // Absolute coordinates
		// DC 2014-07-21 we no longer change the extruder settings in response to G90/G91 commands
		//drivesRelative = false;
		axesRelative = false;
		break;

	case 91: // Relative coordinates
		// DC 2014-07-21 we no longer change the extruder settings in response to G90/G91 commands
		//drivesRelative = true; // Non-axis movements (i.e. extruders)
		axesRelative = true;   // Axis movements (i.e. X, Y and Z)
		break;

	case 92: // Set position
		result = SetPositions(gb);
		break;

	default:
		error = true;
		reply.printf("invalid G Code: %s\n", gb->Buffer());
	}
	if (result && state == GCodeState::normal)
	{
		HandleReply(error, gb, reply.Pointer(), 'G', code, resend);
	}
	return result;
}

bool GCodes::HandleMcode(GCodeBuffer* gb, StringRef& reply)
{
	bool result = true;
	bool error = false;
	bool resend = false;

	int code = gb->GetIValue();
	if (simulating && (code < 20 || code > 37) && code != 82 && code != 83 && code != 111 && code != 105 && code != 122
			&& code != 999)
	{
		return true;			// we don't yet simulate most M codes
	}

	switch (code)
	{
	case 0: // Stop
	case 1: // Sleep
		if (!AllMovesAreFinishedAndMoveBufferIsLoaded())
			return false;

		if (fileBeingPrinted.IsLive())
		{
			fileBeingPrinted.Close();
		}

		// Deselect the active tool
		{
			Tool* tool = reprap.GetCurrentTool();
			if (tool != NULL)
			{
				reprap.StandbyTool(tool->Number());
			}
		}

		// zpl 2014-18-10: Although RRP says M0 is supposed to turn off all drives and heaters,
		// I think M1 is sufficient for this purpose. Leave M0 for a normal reset.
		if (code == 1)
		{
			DisableDrives();
		}
		else
		{
			platform->SetDrivesIdle();
		}

		reprap.GetHeat()->SwitchOffAll();
		if (isPaused)
		{
			isPaused = false;
			reply.copy("Print cancelled\n");
		}

		// Reset everything
		CancelPrint();
		break;

	case 18: // Motors off
	case 84:
		if (!AllMovesAreFinishedAndMoveBufferIsLoaded())
			return false;
		{
			bool seen = false;
			for (size_t axis = 0; axis < AXES; axis++)
			{
				if (gb->Seen(axisLetters[axis]))
				{
					axisIsHomed[axis] = false;
					platform->DisableDrive(axis);
					seen = true;
				}
			}

			if (gb->Seen(extrudeLetter))
			{
				long int eDrive[DRIVES - AXES];
				int eCount = DRIVES - AXES;
				gb->GetLongArray(eDrive, eCount);
				for (int i = 0; i < eCount; i++)
				{
					seen = true;
					if (eDrive[i] < 0 || eDrive[i] >= DRIVES - AXES)
					{
						reply.printf("Invalid extruder number specified: %ld\n", eDrive[i]);
						error = true;
						break;
					}
					platform->DisableDrive(AXES + eDrive[i]);
				}
			}

			if (!seen)
			{
				DisableDrives();
			}
		}
		break;

	case 20:		// List files on SD card
	{
		int sparam = (gb->Seen('S')) ? gb->GetIValue() : 0;
		const char* dir = (gb->Seen('P')) ? gb->GetString() : platform->GetGCodeDir();

		if (sparam == 2)
		{
			reprap.GetFilesResponse(reply, dir, true);			// Send the file list in JSON format
			reply.cat("\n");
		}
		else
		{
			// To mimic the behaviour of the official RepRapPro firmware:
			// If we are emulating RepRap then we print "GCode files:\n" at the start, otherwise we don't.
			// If we are emulating Marlin and the code came via the serial/USB interface, then we don't put quotes around the names and we separate them with newline;
			// otherwise we put quotes around them and separate them with comma.
			if (platform->Emulating() == me || platform->Emulating() == reprapFirmware)
			{
				reply.copy("GCode files:\n");
			}
			else
			{
				reply.Clear();
			}

			bool encapsulate_list = (gb != serialGCode || platform->Emulating() != marlin);
			FileInfo file_info;
			if (platform->GetMassStorage()->FindFirst(dir, file_info))
			{
				// iterate through all entries and append each file name
				do
				{
					if (encapsulate_list)
					{
						reply.catf("%c%s%c%c", FILE_LIST_BRACKET, file_info.fileName, FILE_LIST_BRACKET, FILE_LIST_SEPARATOR);
					}
					else
					{
						reply.catf("%s\n", file_info.fileName);
					}
				} while (platform->GetMassStorage()->FindNext(file_info));

				if (encapsulate_list)
				{
					// remove the last separator and replace by newline
					reply[reply.strlen() - 1] = '\n';
				}
			}
			else
			{
				reply.cat("NONE\n");
			}
		}
	}
		break;

	case 21: // Initialise SD - ignore
		break;

	case 23: // Set file to print
	case 32: // Select file and start SD print
		if (isPaused)
		{
			reply.copy("Cannot set file to print, because another print is still paused. Run M0 or M1 first.");
			break;
		}

		if (code == 32 && !AllMovesAreFinishedAndMoveBufferIsLoaded())
		{
			return false;
		}

		{
			const char* filename = gb->GetUnprecedentedString();
			QueueFileToPrint(filename);
			if (fileToPrint.IsLive())
			{
				reprap.GetPrintMonitor()->StartingPrint(filename);
				if (platform->Emulating() == marlin && gb == serialGCode)
				{
					reply.copy("File opened\nFile selected\n");
				}
				else
				{
					// Command came from web interface or PanelDue, or not emulating Marlin, so send a nicer response
					reply.printf("File %s selected for printing\n", filename);
				}

				if (code == 32)
				{
					fileBeingPrinted.MoveFrom(fileToPrint);
					reprap.GetPrintMonitor()->StartedPrint();
				}
			}
			else
			{
				reply.printf("Failed to open file %s\n", filename);
			}
		}
		break;

	case 24: // Print/resume-printing the selected file
		if (!AllMovesAreFinishedAndMoveBufferIsLoaded())
		{
			return false;
		}

		if (!fileToPrint.IsLive())
		{
			reply.copy("Cannot print, because no file is selected!\n");
			error = true;
		}
		else if (isPaused)
		{
			fileGCode->Resume();
			state = GCodeState::resuming1;
			DoFileMacro(RESUME_G);
		}
		else
		{
			fileBeingPrinted.MoveFrom(fileToPrint);
			reprap.GetPrintMonitor()->StartedPrint();
		}
		break;

	case 226: // Gcode Initiated Pause
		if (!AllMovesAreFinishedAndMoveBufferIsLoaded())
			return false;
		// no break

	case 25: // Pause the print
		if (isPaused)
		{
			reply.copy("Printing is already paused!!\n");
			error = true;
		}
		else if (!reprap.GetPrintMonitor()->IsPrinting())
		{
			reply.copy("Cannot pause print, because no file is being printed!\n");
			error = true;
		}
		else if (doingFileMacro)
		{
			reply.copy("Cannot pause macro files, wait for it to complete first!\n");
			error = true;
		}
		else
		{
			if (code == 25)
			{
				// Pausing a print via another input source
				FilePosition fPos = reprap.GetMove()->PausePrint(pausedMoveBuffer);	// tell Move we wish to pause the current print
				if (fPos != noFilePosition && fileBeingPrinted.IsLive())
				{
					fileBeingPrinted.Seek(fPos);						// replay the abandoned instructions if/when we resume
				}
				fileGCode->Init();
				if (moveAvailable)
				{
					for (size_t drive = AXES; drive < DRIVES; ++drive)
					{
						pausedMoveBuffer[drive] += moveBuffer[drive];	// add on the extrusion in the move not yet taken
					}
					ClearMove();
				}

				for (size_t drive = AXES; drive < DRIVES; ++drive)
				{
					pausedMoveBuffer[drive] = lastRawExtruderPosition[drive - AXES]  - pausedMoveBuffer[drive];
				}

				if (reprap.Debug(moduleGcodes))
				{
					platform->Message(BOTH_MESSAGE, "Paused print, file offset=%u\n", fPos);
				}
			}
			else
			{
				// Pausing a file print because of a command in the file itself
				for (size_t drive = 0; drive < AXES; ++drive)
				{
					pausedMoveBuffer[drive] = moveBuffer[drive];
				}
				for (size_t drive = AXES; drive < DRIVES; ++drive)
				{
					pausedMoveBuffer[drive] = lastRawExtruderPosition[drive - AXES];	// get current extruder positions into pausedMoveBuffer
				}
				pausedMoveBuffer[DRIVES] = moveBuffer[DRIVES];
			}

			pausedFanValue = platform->GetFanValue();
			fileToPrint.MoveFrom(fileBeingPrinted);
			fileGCode->Pause();
			state = GCodeState::pausing1;
			isPaused = true;
		}
		break;

	case 26: // Set SD position
		if (gb->Seen('S'))
		{
			long value = gb->GetLValue();
			if (value < 0)
			{
				reply.copy("SD positions can't be negative!\n");
				error = true;
			}
			else if (fileBeingPrinted.IsLive())
			{
				if (!fileBeingPrinted.Seek(value))
				{
					reply.copy("The specified SD position is invalid!\n");
					error = true;
				}
			}
			else if (fileToPrint.IsLive())
			{
				if (!fileToPrint.Seek(value))
				{
					reply.copy("The specified SD position is invalid!\n");
					error = true;
				}
			}
			else
			{
				reply.copy("Cannot set SD file position, because no print is in progress!\n");
				error = true;
			}
		}
		else
		{
			reply.copy("You must specify the SD position in bytes using the S parameter.\n");
			error = true;
		}
		break;

	case 27: // Report print status - Deprecated
		if (!reprap.GetPrintMonitor()->IsPrinting())
		{
			reply.copy("SD printing.\n");
		}
		else
		{
			reply.copy("Not SD printing.\n");
		}
		break;

	case 28: // Write to file
		{
			const char* str = gb->GetUnprecedentedString();
			bool ok = OpenFileToWrite(platform->GetGCodeDir(), str, gb);
			if (ok)
			{
				reply.printf("Writing to file: %s\n", str);
			}
			else
			{
				reply.printf("Can't open file %s for writing.\n", str);
				error = true;
			}
		}
		break;

	case 29: // End of file being written; should be intercepted before getting here
		reply.copy("GCode end-of-file being interpreted.\n");
		break;

	case 30:	// Delete file
		DeleteFile(gb->GetUnprecedentedString());
		break;

		// For case 32, see case 24

	case 36:	// Return file information
	{
		const char* filename = gb->GetUnprecedentedString(true);	// get filename, or NULL if none provided
		reprap.GetPrintMonitor()->GetFileInfoResponse(reply, filename);
		reply.cat("\n");
	}
		break;

	case 37:	// Simulation mode on/off
		if (gb->Seen('S'))
		{
			if (!AllMovesAreFinishedAndMoveBufferIsLoaded())
			{
				return false;
			}

			bool wasSimulating = simulating;
			simulating = gb->GetIValue() != 0;
			reprap.GetMove()->Simulate(simulating);

			if (simulating)
			{
				simulationTime = 0.0;
				if (!wasSimulating)
				{
					// Starting a new simulation, so save the current position
					reprap.GetMove()->GetCurrentUserPosition(savedMoveBuffer, 0);
				}
			}
			else if (wasSimulating)
			{
				// Ending a simulation, so restore the position
				SetPositions(savedMoveBuffer);
				reprap.GetMove()->SetFeedrate(savedMoveBuffer[DRIVES]);
			}
		}
		else
		{
			reply.printf("Simulation mode: %s, move time: %.1f sec, other time: %.1f sec\n",
					(simulating) ? "on" : "off", simulationTime, reprap.GetMove()->GetSimulationTime());
		}
		break;

	case 80:	// ATX power on
		platform->SetAtxPower(true);
		break;

	case 81:	// ATX power off
		if (!AllMovesAreFinishedAndMoveBufferIsLoaded())
			return false;
		platform->SetAtxPower(false);
		break;

	case 82:	// Use absolute extruder positioning
		if (drivesRelative)		// don't reset the absolute extruder position if it was already absolute
		{
			for (int8_t extruder = AXES; extruder < DRIVES; extruder++)
			{
				lastRawExtruderPosition[extruder - AXES] = 0.0;
			}
			drivesRelative = false;
		}
		break;

	case 83:	// Use relative extruder positioning
		if (!drivesRelative)	// don't reset the absolute extruder position if it was already relative
		{
			for (int8_t extruder = AXES; extruder < DRIVES; extruder++)
			{
				lastRawExtruderPosition[extruder - AXES] = 0.0;
			}
			drivesRelative = true;
		}
		break;

		// For case 84, see case 18

	case 85: // Set inactive time
		break;

	case 92: // Set/report steps/mm for some axes
		if (!AllMovesAreFinishedAndMoveBufferIsLoaded())
		{
			return false;
		}

		{
			// Save the current positions as we may need them later
			float positionNow[DRIVES];
			Move *move = reprap.GetMove();
			move->GetCurrentUserPosition(positionNow, 0);

			bool seen = false;
			for (size_t axis = 0; axis < AXES; axis++)
			{
				if (gb->Seen(axisLetters[axis]))
				{
					platform->SetDriveStepsPerUnit(axis, gb->GetFValue());
					seen = true;
				}
			}

			if (gb->Seen(extrudeLetter))
			{
				seen = true;
				float eVals[DRIVES - AXES];
				int eCount = DRIVES - AXES;
				gb->GetFloatArray(eVals, eCount);

				// The user may not have as many extruders as we allow for, so just set the ones for which a value is provided
				for (int e = 0; e < eCount; e++)
				{
					platform->SetDriveStepsPerUnit(AXES + e, eVals[e]);
				}
			}

			if (seen)
			{
				// On a delta, if we change the drive steps/mm then we need to recalculate the motor positions
				SetPositions(positionNow);
			}
			else
			{
				reply.printf("Steps/mm: X: %.3f, Y: %.3f, Z: %.3f, E: ", platform->DriveStepsPerUnit(X_AXIS),
						platform->DriveStepsPerUnit(Y_AXIS), platform->DriveStepsPerUnit(Z_AXIS));
				for (size_t drive = AXES; drive < DRIVES; drive++)
				{
					reply.catf("%.3f", platform->DriveStepsPerUnit(drive));
					if (drive < DRIVES - 1)
					{
						reply.cat(":");
					}
				}
				reply.cat("\n");
			}
		}
		break;

	case 98: // Call Macro/Subprogram
		if (!AllMovesAreFinishedAndMoveBufferIsLoaded())
		{
			return false;
		}

		if (gb->Seen('P'))
		{
			DoFileMacro(gb->GetString());
		}
		break;

	case 99: // Return from Macro/Subprogram
		if (!AllMovesAreFinishedAndMoveBufferIsLoaded())
		{
			return false;
		}

		FileMacroCyclesReturn();
		break;

	case 104: // Deprecated.  This sets the active temperature of every heater of the active tool
		if (gb->Seen('S'))
		{
			float temperature = gb->GetFValue();
			Tool* tool;
			if (gb->Seen('T'))
			{
				int toolNumber = gb->GetIValue();
				toolNumber += gb->GetToolNumberAdjust();
				tool = reprap.GetTool(toolNumber);
			}
			else
			{
				tool = reprap.GetCurrentTool();
			}
			SetToolHeaters(tool, temperature);
		}
		break;

	case 105: // Get Extruder Temperature / Get Status Message
		{
			int param = (gb->Seen('S')) ? gb->GetIValue() : 0;
			int seq = (gb->Seen('R')) ? gb->GetIValue() : -1;
			switch (param)
			{
			// case 1 is reserved for future Pronterface versions, see
			// http://reprap.org/wiki/G-code#M105:_Get_Extruder_Temperature

			case 2:
				reprap.GetLegacyStatusResponse(reply, 2, seq);	// send JSON-formatted status response
				reply.cat("\n");
				break;

			case 3:
				reprap.GetLegacyStatusResponse(reply, 3, seq);	// send extended JSON-formatted response
				reply.cat("\n");
				break;

			case 4:
				reprap.GetStatusResponse(reply, 3, seq, false);	// send print status JSON-formatted response
				reply.cat("\n");
				break;

			default:
				reply.copy("T:");
				for (int8_t heater = 1; heater < HEATERS; heater++)
				{
					Heat::HeaterStatus hs = reprap.GetHeat()->GetStatus(heater);
					if (hs != Heat::HS_off && hs != Heat::HS_fault)
					{
						reply.catf("%.1f ", reprap.GetHeat()->GetTemperature(heater));
					}
				}
				reply.catf("B:%.1f\n", reprap.GetHeat()->GetTemperature(HOT_BED));
				break;
			}
		}
		break;

	case 106: // Set/report fan values
	{
		bool seen = false;

		if (gb->Seen('I'))		// Invert cooling
		{
			coolingInverted = (gb->GetIValue() > 0);
			seen = true;
		}

		if (gb->Seen('S'))		// Set new fan value
		{
			float f = gb->GetFValue();
			f = min<float>(f, 255.0);
			f = max<float>(f, 0.0);
			seen = true;
			if (coolingInverted)
			{
				// Check if 1.0 or 255.0 may be used as the maximum value
				platform->SetFanValue((f <= 1.0 ? 1.0 : 255.0) - f);
			}
			else
			{
				platform->SetFanValue(f);
			}
		}
		else
		{
			float f = coolingInverted ? (1.0 - platform->GetFanValue()) : platform->GetFanValue();
			reply.printf("Fan value: %d%%, Cooling inverted: %s\n", (byte) (f * 100.0), coolingInverted ? "yes" : "no");
		}
	}
		break;

	case 107: // Fan off - deprecated
		platform->SetFanValue(coolingInverted ? 255.0 : 0.0);
		break;

	case 109: // Deprecated
		if (!AllMovesAreFinishedAndMoveBufferIsLoaded())
			return false;

		if (gb->Seen('S'))
		{
			float temperature = gb->GetFValue();
			Tool *tool;
			if (gb->Seen('T'))
			{
				int toolNumber = gb->GetIValue();
				toolNumber += gb->GetToolNumberAdjust();
				tool = reprap.GetTool(toolNumber);
			}
			else
			{
				tool = reprap.GetCurrentTool();
			}
			SetToolHeaters(tool, temperature);
			result = ToolHeatersAtSetTemperatures(tool);
		}
		break;

	case 110: // Set line numbers - line numbers are dealt with in the GCodeBuffer class
		break;

	case 111: // Debug level
		if (gb->Seen('S'))
		{
			bool dbv = (gb->GetIValue() != 0);
			if (gb->Seen('P'))
			{
				reprap.SetDebug(static_cast<Module>(gb->GetIValue()), dbv);
			}
			else
			{
				reprap.SetDebug(dbv);
			}
		}
		else
		{
			reprap.PrintDebug();
		}
		break;

	case 112: // Emergency stop - acted upon in Webserver, but also here in case it comes from USB etc.
		reprap.EmergencyStop();
		Reset();
		reply.copy("Emergency Stop! Reset the controller to continue.\n");
		break;

	case 114: // Deprecated
		GetCurrentCoordinates(reply);
		reply.cat("\n");
		break;

	case 115: // Print firmware version
		reply.printf("FIRMWARE_NAME:%s FIRMWARE_VERSION:%s ELECTRONICS:%s DATE:%s\n", NAME, VERSION, ELECTRONICS, DATE);
		break;

	case 116: // Wait for everything, especially set temperatures
		if (!AllMovesAreFinishedAndMoveBufferIsLoaded())
		{
			return false;
		}
		if (gb->Seen('P'))
		{
			// Wait for the heaters associated with the specified tool to be ready
			int toolNumber = gb->GetIValue();
			toolNumber += gb->GetToolNumberAdjust();
			if (!ToolHeatersAtSetTemperatures(reprap.GetTool(toolNumber)))
			{
				return false;
			}
			result = true;
		}
		else
		{
			// Wait for all heaters to be ready
			result = reprap.GetHeat()->AllHeatersAtSetTemperatures(true);
		}
		break;

	case 117:	// Display message
		reprap.SetMessage(gb->GetUnprecedentedString());
		break;

	case 119:
		{
			reply.copy("Endstops - ");
			char comma = ',';
			for (size_t axis = 0; axis < AXES; axis++)
			{
				const char* es;
				switch (platform->Stopped(axis))
				{
				case EndStopHit::lowHit:
					es = "at min stop";
					break;

				case EndStopHit::highHit:
					es = "at max stop";
					break;

				case EndStopHit::lowNear:
					es = "near min stop";
					break;

				case EndStopHit::noStop:
				default:
					es = "not stopped";
				}

				if (axis == AXES - 1)
				{
					comma = ' ';
				}

				reply.catf("%c: %s%c ", axisLetters[axis], es, comma);
			}
			reply.cat("\n");
		}
		break;

	case 120:
		if (!AllMovesAreFinishedAndMoveBufferIsLoaded())
		{
			return false;
		}
		Push();
		break;

	case 121:
		if (!AllMovesAreFinishedAndMoveBufferIsLoaded())
		{
			return false;
		}
		Pop();
		break;

	case 122:
		{
			int val = (gb->Seen('P')) ? gb->GetIValue() : 0;
			if (val == 0)
			{
				reprap.Diagnostics();
			}
			else
			{
				platform->DiagnosticTest(val);
			}
		}
		break;

	case 126: // Valve open
		reply.copy("M126 - valves not yet implemented\n");
		break;

	case 127: // Valve closed
		reply.copy("M127 - valves not yet implemented\n");
		break;

	case 135: // Set PID sample interval
		if (gb->Seen('S'))
		{
			platform->SetHeatSampleTime(gb->GetFValue() * 0.001);  // Value is in milliseconds; we want seconds
		}
		else
		{
			reply.printf("Heat sample time is %.3f seconds.\n", platform->HeatSampleTime());
		}
		break;

	case 140: // Set bed temperature
		if (gb->Seen('S'))
		{
			if (HOT_BED >= 0)
			{
				reprap.GetHeat()->SetActiveTemperature(HOT_BED, gb->GetFValue());
				reprap.GetHeat()->Activate(HOT_BED);
			}
		}
		if (gb->Seen('R'))
		{
			if (HOT_BED >= 0)
			{
				reprap.GetHeat()->SetStandbyTemperature(HOT_BED, gb->GetFValue());
			}
		}
		break;

	case 141: // Chamber temperature
		reply.copy("M141 - heated chamber not yet implemented\n");
		break;

//	case 160: //number of mixing filament drives  TODO: With tools defined, is this needed?
//    	if(gb->Seen('S'))
//		{
//			platform->SetMixingDrives(gb->GetIValue());
//		}
//		break;

	case 190: // Deprecated...
		if (!AllMovesAreFinishedAndMoveBufferIsLoaded())	// tell Move not to wait for more moves
		{
			return false;
		}

		if (gb->Seen('S'))
		{
			if (HOT_BED >= 0)
			{
				reprap.GetHeat()->SetActiveTemperature(HOT_BED, gb->GetFValue());
				reprap.GetHeat()->Activate(HOT_BED);
				result = reprap.GetHeat()->HeaterAtSetTemperature(HOT_BED);
			}
		}
		break;

	case 201: // Set/print axis accelerations  FIXME - should these be in /min not /sec ?
	{
		bool seen = false;
		for (size_t axis = 0; axis < AXES; axis++)
		{
			if (gb->Seen(axisLetters[axis]))
			{
				platform->SetAcceleration(axis, gb->GetFValue() * distanceScale);
				seen = true;
			}
		}

		if (gb->Seen(extrudeLetter))
		{
			seen = true;
			float eVals[DRIVES - AXES];
			int eCount = DRIVES - AXES;
			gb->GetFloatArray(eVals, eCount);
			for (size_t e = 0; e < eCount; e++)
			{
				platform->SetAcceleration(AXES + e, eVals[e] * distanceScale);
			}
		}

		if (!seen)
		{
			reply.printf("Accelerations: X: %.1f, Y: %.1f, Z: %.1f, E: ",
					platform->Acceleration(X_AXIS) / distanceScale, platform->Acceleration(Y_AXIS) / distanceScale,
					platform->Acceleration(Z_AXIS) / distanceScale);
			for (size_t drive = AXES; drive < DRIVES; drive++)
			{
				reply.catf("%.1f", platform->Acceleration(drive) / distanceScale);
				if (drive < DRIVES - 1)
				{
					reply.cat(":");
				}
			}
			reply.cat("\n");
		}
	}
		break;

	case 203: // Set/print maximum feedrates
		{
			bool seen = false;
			for (size_t axis = 0; axis < AXES; axis++)
			{
				if (gb->Seen(axisLetters[axis]))
				{
					platform->SetMaxFeedrate(axis, gb->GetFValue() * distanceScale * secondsToMinutes); // G Code feedrates are in mm/minute; we need mm/sec
					seen = true;
				}
			}

			if (gb->Seen(extrudeLetter))
			{
				seen = true;
				float eVals[DRIVES - AXES];
				int eCount = DRIVES - AXES;
				gb->GetFloatArray(eVals, eCount);
				for (size_t e = 0; e < eCount; e++)
				{
					platform->SetMaxFeedrate(AXES + e, eVals[e] * distanceScale * secondsToMinutes);
				}
			}

			if (!seen)
			{
				reply.printf("Maximum feedrates: X: %.1f, Y: %.1f, Z: %.1f, E: ",
						platform->MaxFeedrate(X_AXIS) / (distanceScale * secondsToMinutes),
						platform->MaxFeedrate(Y_AXIS) / (distanceScale * secondsToMinutes),
						platform->MaxFeedrate(Z_AXIS) / (distanceScale * secondsToMinutes));
				for (size_t drive = AXES; drive < DRIVES; drive++)
				{
					reply.catf("%.1f", platform->MaxFeedrate(drive) / (distanceScale * secondsToMinutes));
					if (drive < DRIVES - 1)
					{
						reply.cat(":");
					}
				}
				reply.cat("\n");
			}
		}
		break;

	case 205: //M205 advanced settings:  minimum travel speed S=while printing T=travel only,  B=minimum segment time X= maximum xy jerk, Z=maximum Z jerk
		// This is superseded in this firmware by M codes for the separate types (e.g. M566).
		break;

	case 206:  // Offset axes - Deprecated
		result = OffsetAxes(gb);
		break;

	case 208: // Set/print maximum axis lengths. If there is an S parameter with value 1 then we set the min value, else we set the max value.
		{
			bool setMin = (gb->Seen('S') ? (gb->GetIValue() == 1) : false);
			bool seen = false;
			for (size_t axis = 0; axis < AXES; axis++)
			{
				if (gb->Seen(axisLetters[axis]))
				{
					float value = gb->GetFValue() * distanceScale;
					if (setMin)
					{
						platform->SetAxisMinimum(axis, value);
					}
					else
					{
						platform->SetAxisMaximum(axis, value);
					}
					seen = true;
				}
			}

			if (!seen)
			{
				reply.copy("Axis limits - ");
				char comma = ',';
				for (size_t axis = 0; axis < AXES; axis++)
				{
					if (axis == AXES - 1)
					{
						comma = '\n';
					}
					reply.catf("%c: %.1f min, %.1f max%c ", axisLetters[axis], platform->AxisMinimum(axis),
							platform->AxisMaximum(axis), comma);
				}
			}
		}
		break;

	case 210: // Set/print homing feed rates
		{
			bool seen = false;
			for (size_t axis = 0; axis < AXES; axis++)
			{
				if (gb->Seen(axisLetters[axis]))
				{
					float value = gb->GetFValue() * distanceScale * secondsToMinutes;
					platform->SetHomeFeedRate(axis, value);
					seen = true;
				}
			}

			if (!seen)
			{
				reply.copy("Homing feedrates (mm/min) - ");
				char comma = ',';
				for (size_t axis = 0; axis < AXES; axis++)
				{
					if (axis == AXES - 1)
					{
						comma = ' ';
					}

					reply.catf("%c: %.1f%c ", axisLetters[axis],
							platform->HomeFeedRate(axis) * minutesToSeconds / distanceScale, comma);
				}
				reply.cat("\n");
			}
		}
		break;

	case 220:	// Set/report speed factor override percentage
		if (gb->Seen('S'))
		{
			float newSpeedFactor = (gb->GetFValue() / 100.0) * secondsToMinutes;// include the conversion from mm/minute to mm/second
			if (newSpeedFactor > 0)
			{
				float ratio = newSpeedFactor / speedFactor;
				speedFactor = newSpeedFactor;
				if (moveAvailable)
				{
					// The last move has not gone yet, so we can update it
					moveBuffer[DRIVES] *= ratio;
				}
				else
				{
					speedFactorChange *= ratio;
				}
			}
			else
			{
				reply.printf("Invalid speed factor specified.\n");
				error = true;
			}
		}
		else
		{
			reply.printf("Speed factor override: %.1f%%\n", speedFactor * minutesToSeconds * 100.0);
		}
		break;

	case 221:	// Set/report extrusion factor override percentage
		{
			int extruder = 0;
			if (gb->Seen('D'))	// D parameter (if present) selects the extruder number
			{
				extruder = gb->GetIValue();
			}

			if (gb->Seen('S'))	// S parameter sets the override percentage
			{
				float extrusionFactor = gb->GetFValue() / 100.0;
				if (extruder >= 0 && extruder < DRIVES - AXES && extrusionFactor >= 0)
				{
					if (moveAvailable)
					{
						moveBuffer[extruder + AXES] *= extrusionFactor/extrusionFactors[extruder];	// last move not gone, so update it
					}
					extrusionFactors[extruder] = extrusionFactor;
				}
			}
			else
			{
				reply.printf("Extrusion factor override for extruder %d: %.1f%%\n", extruder,
						extrusionFactors[extruder] * 100.0);
			}
		}
		break;

		// For case 226, see case 25

	case 300:	// Beep
		if (gb->Seen('P'))
		{
			int ms = gb->GetIValue();
			if (gb->Seen('S'))
			{
				reprap.Beep(gb->GetIValue(), ms);
			}
		}
		break;

	case 301: // Set/report hot end PID values
		SetPidParameters(gb, 1, reply);
		break;

	case 302: // Allow, deny or report cold extrudes
		if (gb->Seen('P'))
		{
			if (gb->GetIValue() > 0)
			{
				reprap.AllowColdExtrude();
			}
			else
			{
				reprap.DenyColdExtrude();
			}
		}
		else
		{
			reply.printf("Cold extrudes are %s, use M302 P[1/0] to allow or deny them\n",
					reprap.ColdExtrude() ? "enabled" : "disabled");
		}
		break;

	case 304: // Set/report heated bed PID values
		if (HOT_BED >= 0)
		{
			SetPidParameters(gb, HOT_BED, reply);
		}
		break;

	case 305: // Set/report specific heater parameters
		SetHeaterParameters(gb, reply);
		break;

	case 400: // Wait for current moves to finish
		if (!AllMovesAreFinishedAndMoveBufferIsLoaded())
			return false;
		break;

	case 404: // Filament width and nozzle diameter
		{
			bool seen = false;

			if (gb->Seen('N'))
			{
				platform->SetFilamentWidth(gb->GetFValue());
				seen = true;
			}
			if (gb->Seen('D'))
			{
				platform->SetNozzleDiameter(gb->GetFValue());
				seen = true;
			}

			if (!seen)
			{
				reply.printf("Filament width: %.2fmm, nozzle diameter: %.2fmm\n", platform->GetFilamentWidth(), platform->GetNozzleDiameter());
			}
		}
		break;

	case 408: // Get status in JSON format
		{
			int param = (gb->Seen('S')) ? gb->GetIValue() : 0;
			int seq = (gb->Seen('R')) ? gb->GetIValue() : -1;
			switch (param)
			{
			case 0:
			case 1:
				reprap.GetLegacyStatusResponse(reply, param + 2, seq);	// send JSON-formatted status response
				reply.cat("\n");
				break;

			case 2:
			case 3:
			case 4:
				reprap.GetStatusResponse(reply, param - 1, seq, false);	// send extended JSON-formatted response
				reply.cat("\n");
				break;

			case 5:
				reprap.GetConfigResponse(reply);						// send JSON-formatted machine config
				reply.cat("\n");
				break;

			}
		}
		break;

	case 500: // Store parameters in EEPROM
		reprap.GetPlatform()->WriteNvData();
		break;

	case 501: // Load parameters from EEPROM
		reprap.GetPlatform()->ReadNvData();
		if (gb->Seen('S'))
		{
			reprap.GetPlatform()->SetAutoSave(gb->GetIValue() > 0);
		}
		break;

	case 502: // Revert to default "factory settings"
		reprap.GetPlatform()->ResetNvData();
		break;

	case 503: // List variable settings
		result = SendConfigToLine();
		break;

	case 540: // Set/report MAC address
		if (gb->Seen('P'))
		{
			SetMACAddress(gb);
		}
		else
		{
			const byte* mac = platform->MACAddress();
			reply.printf("MAC: %x:%x:%x:%x:%x:%x\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
		}
		break;

	case 550: // Set/report machine name
		if (gb->Seen('P'))
		{
			reprap.SetName(gb->GetString());
		}
		else
		{
			reply.printf("RepRap name: %s\n", reprap.GetName());
		}
		break;

	case 551: // Set password (no option to report it)
		if (gb->Seen('P'))
		{
			reprap.SetPassword(gb->GetString());
		}
		break;

	case 552: // Enable/Disable network and/or Set/Get IP address
		// dc42: IMO providing a gcode to enable/disable network access is a waste of space, when it is easier and safer to unplug the network cable.
		// But for compatibility with RRP official firmware, here it is.
		{
			bool seen = false;
			if (gb->Seen('S')) // Has the user turned the network off?
			{
				seen = true;
				if (gb->GetIValue())
				{
					reprap.GetNetwork()->Enable();
				}
				else
				{
					reprap.GetNetwork()->Disable();
				}
			}

			if (gb->Seen('P'))
			{
				seen = true;
				SetEthernetAddress(gb, code);
			}

			if (gb->Seen('R'))
			{
				reprap.GetNetwork()->SetHttpPort(gb->GetIValue());
				seen = true;
			}

			if (!seen)
			{
				const byte *config_ip = platform->IPAddress();
				const byte *actual_ip = reprap.GetNetwork()->IPAddress();
				reply.printf("Network is %s, configured IP address: %d.%d.%d.%d, actual IP address: %d.%d.%d.%d, HTTP port: %d\n",
						reprap.GetNetwork()->IsEnabled() ? "enabled" : "disabled",
						config_ip[0], config_ip[1], config_ip[2], config_ip[3], actual_ip[0], actual_ip[1], actual_ip[2], actual_ip[3],
						reprap.GetNetwork()->GetHttpPort());
			}
		}
		break;

	case 553: // Set/Get netmask
		if (gb->Seen('P'))
		{
			SetEthernetAddress(gb, code);
		}
		else
		{
			const byte *nm = platform->NetMask();
			reply.printf("Net mask: %d.%d.%d.%d\n ", nm[0], nm[1], nm[2], nm[3]);
		}
		break;

	case 554: // Set/Get gateway
		if (gb->Seen('P'))
		{
			SetEthernetAddress(gb, code);
		}
		else
		{
			const byte *gw = platform->GateWay();
			reply.printf("Gateway: %d.%d.%d.%d\n ", gw[0], gw[1], gw[2], gw[3]);
		}
		break;

	case 555: // Set/report firmware type to emulate
		if (gb->Seen('P'))
		{
			platform->SetEmulating((Compatibility) gb->GetIValue());
		}
		else
		{
			reply.copy("Emulating ");
			switch (platform->Emulating())
			{
			case me:
			case reprapFirmware:
				reply.cat("RepRap Firmware (i.e. in native mode)");
				break;

			case marlin:
				reply.cat("Marlin");
				break;

			case teacup:
				reply.cat("Teacup");
				break;

			case sprinter:
				reply.cat("Sprinter");
				break;

			case repetier:
				reply.cat("Repetier");
				break;

			default:
				reply.catf("Unknown: (%d)", platform->Emulating());
			}
			reply.cat("\n");
		}
		break;

	case 556: // Axis compensation
		if (gb->Seen('S'))
		{
			float value = gb->GetFValue();
			for (int8_t axis = 0; axis < AXES; axis++)
			{
				if (gb->Seen(axisLetters[axis]))
				{
					reprap.GetMove()->SetAxisCompensation(axis, gb->GetFValue() / value);
				}
			}
		}
		else
		{
			reply.printf("Axis compensations - XY: %.5f, YZ: %.5f, ZX: %.5f\n",
					reprap.GetMove()->AxisCompensation(X_AXIS), reprap.GetMove()->AxisCompensation(Y_AXIS),
					reprap.GetMove()->AxisCompensation(Z_AXIS));
		}
		break;

	case 557: // Set/report Z probe point coordinates
		if (gb->Seen('P'))
		{
			int point = gb->GetIValue();
			if (point < 0 || point >= MaxProbePoints)
			{
				reprap.GetPlatform()->Message(BOTH_ERROR_MESSAGE, "Z probe point index out of range.\n");
			}
			else
			{
				bool seen = false;
				if (gb->Seen(axisLetters[X_AXIS]))
				{
					reprap.GetMove()->SetXBedProbePoint(point, gb->GetFValue());
					seen = true;
				}
				if (gb->Seen(axisLetters[Y_AXIS]))
				{
					reprap.GetMove()->SetYBedProbePoint(point, gb->GetFValue());
					seen = true;
				}

				if (!seen)
				{
					reply.printf("Probe point %d - [%.1f, %.1f]\n", point, reprap.GetMove()->XBedProbePoint(point),
							reprap.GetMove()->YBedProbePoint(point));
				}
			}
		}
		break;

	case 558: // Set or report Z probe type and for which axes it is used
	{
		bool seen = false;
		bool zProbeAxes[AXES];
		platform->GetZProbeAxes(zProbeAxes);
		for (size_t axis = 0; axis < AXES; axis++)
		{
			if (gb->Seen(axisLetters[axis]))
			{
				zProbeAxes[axis] = (gb->GetIValue() > 0);
				seen = true;
			}
		}
		if (seen)
		{
			platform->SetZProbeAxes(zProbeAxes);
		}

		// We must get and set the Z probe type first before setting the dive height, because different probe types may have different dive heights
		if (gb->Seen('P'))
		{
			platform->SetZProbeType(gb->GetIValue());
			seen = true;
		}

		if (gb->Seen('H'))
		{
			platform->SetZProbeDiveHeight(gb->GetIValue());
			seen = true;
		}

		if (gb->Seen('R'))
		{
			platform->SetZProbeChannel(gb->GetIValue());
			seen = true;
		}

		if (gb->Seen('S'))
		{
			ZProbeParameters params = platform->GetZProbeParameters();
			params.param1 = gb->GetFValue();
			platform->SetZProbeParameters(params);
			seen = true;
		}

		if (gb->Seen('T'))
		{
			ZProbeParameters params = platform->GetZProbeParameters();
			params.param2 = gb->GetFValue();
			platform->SetZProbeParameters(params);
			seen = true;
		}

		if (!seen)
		{
			reply.printf("Z Probe type %d, channel %d, dive height %.1f", platform->GetZProbeType(), platform->GetZProbeChannel(), platform->GetZProbeDiveHeight());
			if (platform->GetZProbeType() == 5)
			{
				ZProbeParameters params = platform->GetZProbeParameters();
				reply.catf(", parameters %.2f %.2f", params.param1, params.param2);
			}
			reply.cat(", used for these axes:");
			for (size_t axis = 0; axis < AXES; axis++)
			{
				if (zProbeAxes[axis])
				{
					reply.catf(" %c", axisLetters[axis]);
				}
			}
			reply.cat("\n");
		}
	}
		break;

	case 559: // Upload config.g or another gcode file to put in the sys directory
	{
		const char* str = (gb->Seen('P') ? gb->GetString() : platform->GetConfigFile());
		bool ok = OpenFileToWrite(platform->GetSysDir(), str, gb);
		if (ok)
		{
			reply.printf("Writing to file: %s\n", str);
		}
		else
		{
			reply.printf("Can't open file %s for writing.\n", str);
			error = true;
		}
	}
		break;

	case 560: // Upload reprap.htm or another web interface file
	{
		const char* str = (gb->Seen('P') ? gb->GetString() : INDEX_PAGE);
		bool ok = OpenFileToWrite(platform->GetWebDir(), str, gb);
		if (ok)
		{
			reply.printf("Writing to file: %s\n", str);
		}
		else
		{
			reply.printf("Can't open file %s for writing.\n", str);
			error = true;
		}
	}
		break;

	case 561:
		reprap.GetMove()->SetIdentityTransform();
		break;

	case 562: // Reset temperature fault - use with great caution
		if (gb->Seen('P'))
		{
			int iValue = gb->GetIValue();
			reprap.GetHeat()->ResetFault(iValue);
		}
		break;

	case 563: // Define tool
		ManageTool(gb, reply);
		break;

	case 564: // Think outside the box?
		if (gb->Seen('S'))
		{
			limitAxes = (gb->GetIValue() != 0);
		}
		else
		{
			reply.printf("Movement outside the bed is %spermitted\n", (limitAxes) ? "not " : "");
		}
		break;

	case 566: // Set/print maximum jerk speeds
	{
		bool seen = false;
		for (int8_t axis = 0; axis < AXES; axis++)
		{
			if (gb->Seen(axisLetters[axis]))
			{
				platform->SetInstantDv(axis, gb->GetFValue() * distanceScale * secondsToMinutes); // G Code feedrates are in mm/minute; we need mm/sec
				seen = true;
			}
		}

		if (gb->Seen(extrudeLetter))
		{
			seen = true;
			float eVals[DRIVES - AXES];
			int eCount = DRIVES - AXES;
			gb->GetFloatArray(eVals, eCount);
			for (int8_t e = 0; e < eCount; e++)
			{
				platform->SetInstantDv(AXES + e, eVals[e] * distanceScale * secondsToMinutes);
			}
		}
		else if (!seen)
		{
			reply.printf("Maximum jerk rates: X: %.1f, Y: %.1f, Z: %.1f, E: ",
					platform->ConfiguredInstantDv(X_AXIS) / (distanceScale * secondsToMinutes),
					platform->ConfiguredInstantDv(Y_AXIS) / (distanceScale * secondsToMinutes),
					platform->ConfiguredInstantDv(Z_AXIS) / (distanceScale * secondsToMinutes));
			for (int8_t drive = AXES; drive < DRIVES; drive++)
			{
				reply.catf("%.1f%c", platform->ConfiguredInstantDv(drive) / (distanceScale * secondsToMinutes),
						(drive < DRIVES - 1) ? ':' : '\n');
			}
		}
	}
		break;

	case 567: // Set/report tool mix ratios
		if (gb->Seen('P'))
		{
			int8_t tNumber = gb->GetIValue();
			Tool* tool = reprap.GetTool(tNumber);
			if (tool != NULL)
			{
				if (gb->Seen(extrudeLetter))
				{
					float eVals[DRIVES - AXES];
					int eCount = tool->DriveCount();
					gb->GetFloatArray(eVals, eCount);
					if (eCount != tool->DriveCount())
					{
						reply.printf("Setting mix ratios - wrong number of E drives: %s\n", gb->Buffer());
					}
					else
					{
						tool->DefineMix(eVals);
					}
				}
				else
				{
					reply.printf("Tool %d mix ratios: ", tNumber);
					char sep = ':';
					for (int8_t drive = 0; drive < tool->DriveCount(); drive++)
					{
						reply.catf("%.3f%c", tool->GetMix()[drive], sep);
						if (drive >= tool->DriveCount() - 2)
						{
							sep = '\n';
						}
					}
				}
			}
		}
		break;

	case 568: // Turn on/off automatic tool mixing
		if (gb->Seen('P'))
		{
			Tool* tool = reprap.GetTool(gb->GetIValue());
			if (tool != NULL)
			{
				if (gb->Seen('S'))
					if (gb->GetIValue() != 0)
						tool->TurnMixingOn();
					else
						tool->TurnMixingOff();
			}
		}
		break;

	case 569: // Set/report axis direction
		if (gb->Seen('P'))
		{
			int8_t drive = gb->GetIValue();
			if (gb->Seen('S'))
			{
				platform->SetDirectionValue(drive, gb->GetIValue());
			}
			else
			{
				reply.printf("A %d sends drive %d forwards.\n", (int) platform->GetDirectionValue(drive), drive);
			}
		}
		break;

	case 570: // Set/report heater timeout
		if (gb->Seen('S'))
		{
			platform->SetTimeToHot(gb->GetFValue());
		}
		else
		{
			reply.printf("Time allowed to get to temperature: %.1f seconds.\n", platform->TimeToHot());
		}
		break;

	case 571: // Set output on extrude
		if (gb->Seen('S'))
		{
			platform->SetExtrusionAncilliaryPWM(gb->GetFValue());
		}
		else
		{
			reply.printf("Extrusion ancillary PWM: %.3f.\n", platform->GetExtrusionAncilliaryPWM());
		}
		break;

	case 572: // Set/report elastic compensation
		if (gb->Seen('P'))
		{
			size_t drive = gb->GetIValue();
			if (gb->Seen('S'))
			{
				platform->SetElasticComp(drive, gb->GetFValue());
			}
			else
			{
				reply.printf("Elastic compensation for drive %u is %.3f seconds\n", drive, platform->GetElasticComp(drive));
			}
		}
		break;

	case 573:
		if (gb->Seen('P'))
		{
			int heater = gb->GetIValue();
			if (heater >= 0 && heater < HEATERS)
			{
				reply.printf("Average heater %d PWM: %.3f.\n", heater, reprap.GetHeat()->GetAveragePWM(heater));
			}
		}
		break;

	case 574: // Set endstop configuration
		{
			bool seen = false;
			bool logicLevel = (gb->Seen('S')) ? (gb->GetIValue() != 0) : true;
			for (size_t axis = 0; axis <= AXES; ++axis)
			{
				const char letter = (axis == AXES) ? extrudeLetter : axisLetters[axis];
				if (gb->Seen(letter))
				{
					int ival = gb->GetIValue();
					if (ival >= 0 && ival <= 3)
					{
						platform->SetEndStopConfiguration(axis, (EndStopType) ival, logicLevel);
						seen = true;
					}
				}
			}
			if (!seen)
			{
				reply.copy("Endstop configuration:");
				for (size_t axis = 0; axis < AXES; ++axis)
				{
					EndStopType config;
					bool logic;
					platform->GetEndStopConfiguration(axis, config, logic);
					reply.catf(" %c %s %s %c", axisLetters[axis],
							(config == EndStopType::highEndStop) ? "high end" : (config == EndStopType::lowEndStop) ? "low end" : "none",
							(config == EndStopType::noEndStop) ? "" : (logic) ? " (active high)" : " (active low)",
							(axis == AXES - 1) ? '\n' : ',');
				}
			}
		}
		break;

	case 575: // Set communications parameters
		if (gb->Seen('P'))
		{
			size_t chan = gb->GetIValue();
			if (chan < NUM_SERIAL_CHANNELS)
			{
				bool seen = false;
				if (gb->Seen('B'))
				{
					platform->SetBaudRate(chan, gb->GetIValue());
					seen = true;
				}
				if (gb->Seen('S'))
				{
					uint32_t val = gb->GetIValue();
					platform->SetCommsProperties(chan, val);
					switch (chan)
					{
					case 0:
						serialGCode->SetCommsProperties(val);
						break;
					case 1:
						auxGCode->SetCommsProperties(val);
						break;
					default:
						break;
					}
					seen = true;
				}
				if (!seen)
				{
					uint32_t cp = platform->GetCommsProperties(chan);
					reply.printf("Channel %d: baud rate %d, %s checksum\n", chan, platform->GetBaudRate(chan),
							(cp & 1) ? "requires" : "does not require");
				}
			}
		}
		break;

	case 665: // Set delta configuration
		if (!AllMovesAreFinishedAndMoveBufferIsLoaded())
		{
			return false;
		}
		{
			float positionNow[DRIVES];
			Move *move = reprap.GetMove();
			move->GetCurrentUserPosition(positionNow, 0);					// get the current position, we may need it later
			DeltaParameters& params = move->AccessDeltaParams();
			bool wasInDeltaMode = params.IsDeltaMode();						// remember whether we were in delta mode
			bool seen = false;

			if (gb->Seen('L'))
			{
				params.SetDiagonal(gb->GetFValue() * distanceScale);
				seen = true;
			}
			if (gb->Seen('R'))
			{
				params.SetRadius(gb->GetFValue() * distanceScale);
				seen = true;
			}
			if (gb->Seen('B'))
			{
				params.SetPrintRadius(gb->GetFValue() * distanceScale);
				seen = true;
			}
			if (gb->Seen('H'))
			{
				params.SetHomedHeight(gb->GetFValue() * distanceScale);
				seen = true;
			}
			if (seen)
			{
				// If we have changed between Cartesian and Delta mode, we need to reset the motor coordinates to agree with the XYZ xoordinates.
				// This normally happens only when we process the M665 command in config.g. Also flag that the machine is not homed.
				if (params.IsDeltaMode() != wasInDeltaMode)
				{
					SetPositions(positionNow);
				}
				SetAllAxesNotHomed();
			}
			else
			{
				if (params.IsDeltaMode())
				{
					reply.printf("Diagonal %.2f, delta radius %.2f, homed height %.2f, bed radius %.1f, X %.1f" DEGREE_SYMBOL ", Y %.1f" DEGREE_SYMBOL "\n",
							params.GetDiagonal() / distanceScale, params.GetRadius() / distanceScale,
							params.GetHomedHeight() / distanceScale, params.GetPrintRadius() / distanceScale,
							params.GetXCorrection(), params.GetYCorrection());
				}
				else
				{
					reply.printf("Printer is not in delta mode\n");
				}
			}
		}
		break;

	case 666: // Set delta endstop adjustments
		{
			DeltaParameters& params = reprap.GetMove()->AccessDeltaParams();
			bool seen = false;
			if (gb->Seen('X'))
			{
				params.SetEndstopAdjustment(X_AXIS, gb->GetFValue());
				seen = true;
			}
			if (gb->Seen('Y'))
			{
				params.SetEndstopAdjustment(Y_AXIS, gb->GetFValue());
				seen = true;
			}
			if (gb->Seen('Z'))
			{
				params.SetEndstopAdjustment(Z_AXIS, gb->GetFValue());
				seen = true;
			}
			if (!seen)
			{
				reply.printf("Endstop adjustments X%.2f Y%.2f Z%.2f\n",
						params.GetEndstopAdjustment(X_AXIS), params.GetEndstopAdjustment(Y_AXIS), params.GetEndstopAdjustment(Z_AXIS));
			}
		}
		break;

	case 667: // Set CoreXY mode
		{
			Move* move = reprap.GetMove();
			if (gb->Seen('S'))
			{
				float positionNow[DRIVES];
				move->GetCurrentUserPosition(positionNow, 0);					// get the current position, we may need it later
				int newMode = gb->GetIValue();
				if (newMode != move->GetCoreXYMode())
				{
					move->SetCoreXYMode(newMode);
					SetPositions(positionNow);
					SetAllAxesNotHomed();
				}
			}
			else
			{
				reply.printf("Printer mode is %s\n", move->GetGeometryString());
			}
		}
		break;

	case 906: // Set/report Motor currents
		{
			bool seen = false;
			for (size_t axis = 0; axis < AXES; axis++)
			{
				if (gb->Seen(axisLetters[axis]))
				{
					platform->SetMotorCurrent(axis, gb->GetFValue());
					seen = true;
				}
			}

			if (gb->Seen(extrudeLetter))
			{
				float eVals[DRIVES - AXES];
				int eCount = DRIVES - AXES;
				gb->GetFloatArray(eVals, eCount);
				// 2014-09-29 DC42: we no longer insist that the user supplies values for all possible extruder drives
				for (size_t e = 0; e < eCount; e++)
				{
					platform->SetMotorCurrent(AXES + e, eVals[e]);
				}
				seen = true;
			}

			if (gb->Seen('I'))
			{
				float idleFactor = gb->GetFValue();
				if (idleFactor >= 0 && idleFactor <= 100.0)
				{
					platform->SetIdleCurrentFactor(idleFactor/100.0);
					seen = true;
				}
			}

			if (!seen)
			{
				reply.printf("Axis currents (mA) - X:%d, Y:%d, Z:%d, E:", (int) platform->MotorCurrent(X_AXIS),
						(int) platform->MotorCurrent(Y_AXIS), (int) platform->MotorCurrent(Z_AXIS));
				for (size_t drive = AXES; drive < DRIVES; drive++)
				{
					reply.catf("%d%c", (int) platform->MotorCurrent(drive), (drive < DRIVES - 1) ? ':' : ',');
				}
				reply.catf(" idle factor %d\n", (int)(platform->GetIdleCurrentFactor() * 100.0));
			}
		}
		break;

	case 998:
		// The input handling code replaces the gcode by this when it detects a checksum error.
		// Since we have no way of asking for the line to be re-sent, just report an error.
		// If the line number is zero, then it probably came from PanelDue and was caused by input buffer overflow
		// while we were busy doing a macro, so just ignore it.
		if (gb->Seen('P'))
		{
			int val = gb->GetIValue();
			if (val != 0)
			{
				reply.printf("Checksum error on line %d\n", val);
				resend = true;
			}
		}
		break;

	case 999:
		result = DoDwellTime(0.5);		// wait half a second to allow the response to be sent back to the web server, otherwise it may retry
		if (result)
		{
			uint16_t reason = (gb->Seen('P') && StringStartsWith(gb->GetString(), "ERASE"))
											? SoftwareResetReason::erase
											: SoftwareResetReason::user;
			platform->SoftwareReset(reason);			// doesn't return
		}
		break;

	default:
		error = true;
		reply.printf("invalid M Code: %s\n", gb->Buffer());
	}

	// Note that we send a reply to M105 requests even if the status is not 'normal', because we reply to these requests even when we are in other states
	if (result && (state == GCodeState::normal || code == 105))
	{
		HandleReply(error, gb, reply.Pointer(), 'M', code, resend);
	}
	return result;
}

bool GCodes::HandleTcode(GCodeBuffer* gb, StringRef& reply)
{
	if (!AllMovesAreFinishedAndMoveBufferIsLoaded())
	{
		return false;
	}

	newToolNumber = gb->GetIValue();
	newToolNumber += gb->GetToolNumberAdjust();
	if (simulating)						// we don't yet simulate any T codes
	{
		HandleReply(false, gb, "", 'T', newToolNumber, false);
	}
	else
	{
		// If old and new are the same still follow the sequence - the user may want the macros run.
		Tool *oldTool = reprap.GetCurrentTool();
		state = GCodeState::toolChange1;
		if (oldTool != NULL && AllAxesAreHomed())
		{
			scratchString.printf("tfree%d.g", oldTool->Number());
			DoFileMacro(scratchString.Pointer(), false);
		}
	}
	return true;
}

// Return the amount of filament extruded
float GCodes::GetRawExtruderPosition(size_t extruder) const
{
	return (extruder < (DRIVES - AXES)) ? lastRawExtruderPosition[extruder] : 0;
}

// Pause the current SD card print. Called from the web interface.
void GCodes::PauseSDPrint()
{
	if (fileBeingPrinted.IsLive())
	{
		fileToPrint.MoveFrom(fileBeingPrinted);
		fileGCode->Pause();		// if we are executing some sort of wait command, pause it until we restart
	}
}

// Cancel the current SD card print
void GCodes::CancelPrint()
{
	moveAvailable = false;

	fileGCode->Init();

	if (fileBeingPrinted.IsLive())
	{
		fileBeingPrinted.Close();
	}

	reprap.GetPrintMonitor()->StoppedPrint();
}

// Return true if all the heaters for the specified tool are at their set temperatures
bool GCodes::ToolHeatersAtSetTemperatures(const Tool *tool) const
{
	if (tool != NULL)
	{
		for (int i = 0; i < tool->HeaterCount(); ++i)
		{
			if (!reprap.GetHeat()->HeaterAtSetTemperature(tool->Heater(i)))
			{
				return false;
			}
		}
	}
	return true;
}

// Set the current position
void GCodes::SetPositions(float positionNow[DRIVES])
{
	// Transform the position so that e.g. if the user does G92 Z0,
	// the position we report (which gets inverse-transformed) really is Z=0 afterwards
	reprap.GetMove()->Transform(positionNow);
	reprap.GetMove()->SetLiveCoordinates(positionNow);
	reprap.GetMove()->SetPositions(positionNow);
}

bool GCodes::IsPaused() const
{
	return isPaused && !IsPausing() && !IsResuming();
}

bool GCodes::IsPausing() const
{
	const GCodeState topState = (stackPointer == 0) ? state : stack[0].state;
	return topState == GCodeState::pausing1 || topState == GCodeState::pausing2;
}

bool GCodes::IsResuming() const
{
	const GCodeState topState = (stackPointer == 0) ? state : stack[0].state;
	return topState == GCodeState::resuming1 || topState == GCodeState::resuming2 || topState == GCodeState::resuming3;
}

// End
