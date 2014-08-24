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

const float secondsToMinutes = 1.0/60.0;

GCodes::GCodes(Platform* p, Webserver* w)
{
	active = false;
	platform = p;
	webserver = w;
	webGCode = new GCodeBuffer(platform, "web: ");
	fileGCode = new GCodeBuffer(platform, "file: ");
	serialGCode = new GCodeBuffer(platform, "serial: ");
	cannedCycleGCode = new GCodeBuffer(platform, "macro: ");
}

void GCodes::Exit()
{
	platform->Message(BOTH_MESSAGE, "GCodes class exited.\n");
	active = false;
}

void GCodes::Init()
{
	Reset();
	drivesRelative = true;
	axesRelative = false;
	axisLetters = AXIS_LETTERS;
	distanceScale = 1.0;
	for (int8_t i = 0; i < DRIVES - AXES; i++)
	{
		lastPos[i] = 0.0;
	}
	configFile = NULL;
	eofString = EOF_STRING;
	eofStringCounter = 0;
	eofStringLength = strlen(eofString);
	homeX = false;
	homeY = false;
	homeZ = false;
	offSetSet = false;
	zProbesSet = false;
	active = true;
	longWait = platform->Time();
	dwellTime = longWait;
	limitAxes = true;
	axisIsHomed[X_AXIS] = axisIsHomed[Y_AXIS] = axisIsHomed[Z_AXIS] = false;
	toolChangeSequence = 0;
	coolingInverted = false;
}

// This is called from Init and when doing an emergency stop
void GCodes::Reset()
{
	webGCode->Init();
	fileGCode->Init();
	serialGCode->Init();
	cannedCycleGCode->Init();
	moveAvailable = false;
	fileBeingPrinted.Close();
	fileToPrint.Close();
	fileBeingWritten = NULL;
	endStopsToCheck = 0;
	doingCannedCycleFile = false;
	dwellWaiting = false;
	stackPointer = 0;
	waitingForMoveToComplete = false;
	probeCount = 0;
	cannedCycleMoveCount = 0;
	cannedCycleMoveQueued = false;
	speedFactor = 1.0/60.0;				// default is just to convert from mm/minute to mm/second
	speedFactorChange = 1.0;
	for (size_t i = 0; i < DRIVES - AXES; ++i)
	{
		extrusionFactors[i] = 1.0;
	}
}

void GCodes::DoFilePrint(GCodeBuffer* gb)
{
	char b;

	if (fileBeingPrinted.IsLive())
	{
		if (fileBeingPrinted.Read(b))
		{
			if (gb->Put(b))
			{
				gb->SetFinished(ActOnCode(gb));
			}
		}
		else
		{
			if (gb->Put('\n')) // In case there wasn't one ending the file
			{
				gb->SetFinished(ActOnCode(gb));
			}
			fileBeingPrinted.Close();
		}
	}
}

void GCodes::Spin()
{
	if (!active)
		return;

	// Check each of the sources of G Codes (web, serial, and file) to
	// see if what they are doing has been done.  If it hasn't, return without
	// looking at anything else.
	//
	// Note the order establishes a priority: web first, then serial, and file
	// last.  If file weren't last, then the others would never get a look in when
	// a file was being printed.

	if (webGCode->Active())
	{
		webGCode->SetFinished(ActOnCode(webGCode));
		platform->ClassReport("GCodes", longWait);
		return;
	}

	if (serialGCode->Active())
	{
		serialGCode->SetFinished(ActOnCode(serialGCode));
		platform->ClassReport("GCodes", longWait);
		return;
	}

	if (fileGCode->Active())
	{
		fileGCode->SetFinished(ActOnCode(fileGCode));
		platform->ClassReport("GCodes", longWait);
		return;
	}

	// Now check if a G Code byte is available from each of the sources
	// in the same order for the same reason.

	if (webserver->GCodeAvailable())
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
					webGCode->SetFinished(ActOnCode(webGCode));
				}
				break;	// stop after receiving a complete gcode in case we haven't finished processing it
			}
			++i;
		} while (i < 16 && webserver->GCodeAvailable());
		platform->ClassReport("GCodes", longWait);
		return;
	}

	// Now the serial interface.  First check the special case of our
	// uploading the reprap.htm file

	if (serialGCode->WritingFileDirectory() == platform->GetWebDir())
	{
		if (platform->GetLine()->Status() & byteAvailable)
		{
			char b;
			platform->GetLine()->Read(b);
			WriteHTMLToFile(b, serialGCode);
		}
	}
	else
	{
		// Otherwise just deal in general with incoming bytes from the serial interface

		if (platform->GetLine()->Status() & byteAvailable)
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
					}
					else
					{
						serialGCode->SetFinished(ActOnCode(serialGCode));
					}
					break;	// stop after receiving a complete gcode in case we haven't finished processing it
				}
				++i;
			} while (i < 16 && (platform->GetLine()->Status() & byteAvailable));
			platform->ClassReport("GCodes", longWait);
			return;
		}
	}

	DoFilePrint(fileGCode);

	platform->ClassReport("GCodes", longWait);
}

void GCodes::Diagnostics()
{
	platform->AppendMessage(BOTH_MESSAGE, "GCodes Diagnostics:\n");
}

// The wait till everything's done function.  If you need the machine to
// be idle before you do something (for example homeing an axis, or shutting down) call this
// until it returns true.  As a side-effect it loads moveBuffer with the last
// position and feedrate for you.

bool GCodes::AllMovesAreFinishedAndMoveBufferIsLoaded()
{
	// Last one gone?
	if(moveAvailable)
		return false;

	// Wait for all the queued moves to stop so we get the actual last position and feedrate
	if(!reprap.GetMove()->AllMovesAreFinished())
		return false;
	reprap.GetMove()->ResumeMoving();

	// Load the last position; If Move can't accept more, return false - should never happen
	if(!reprap.GetMove()->GetCurrentUserPosition(moveBuffer))
		return false;

	return true;
}

// Save (some of) the state of the machine for recovery in the future.
// Call repeatedly till it returns true.

bool GCodes::Push()
{
	if(stackPointer >= STACK)
	{
		platform->Message(BOTH_ERROR_MESSAGE, "Push(): stack overflow!\n");
		return true;
	}

	if(!AllMovesAreFinishedAndMoveBufferIsLoaded())
		return false;

	drivesRelativeStack[stackPointer] = drivesRelative;
	axesRelativeStack[stackPointer] = axesRelative;
	feedrateStack[stackPointer] = moveBuffer[DRIVES];
	fileStack[stackPointer].CopyFrom(fileBeingPrinted);
	stackPointer++;
	platform->PushMessageIndent();
	return true;
}

// Recover a saved state.  Call repeatedly till it returns true.

bool GCodes::Pop()
{
	if(stackPointer < 1)
	{
		platform->Message(BOTH_ERROR_MESSAGE, "Pop(): stack underflow!\n");
		return true;
	}

	if(!AllMovesAreFinishedAndMoveBufferIsLoaded())
		return false;

	stackPointer--;
	drivesRelative = drivesRelativeStack[stackPointer];
	axesRelative = axesRelativeStack[stackPointer];
	fileBeingPrinted.MoveFrom(fileStack[stackPointer]);
	platform->PopMessageIndent();

	// Remember for next time if we have just been switched to absolute drive moves
	// DC 2014-07-16: the following code is wrong, it messes up the absolute extruder position (typically it resets it to zero) and does nothing useful as far as I can see.
	// So I am commenting it out.
	//for(int8_t i = AXES; i < DRIVES; i++)
	//{
	//  lastPos[i - AXES] = moveBuffer[i];
	//}

	// Set the correct feedrate

	moveBuffer[DRIVES] = feedrateStack[stackPointer];

	endStopsToCheck = 0;
	moveAvailable = true;
	return true;
}

// Move expects all axis movements to be absolute, and all
// extruder drive moves to be relative.  This function serves that.
// If applyLimits is true and we have homed the relevant axes, then we don't allow movement beyond the bed.

bool GCodes::LoadMoveBufferFromGCode(GCodeBuffer *gb, bool doingG92, bool applyLimits)
{
	// Zero every extruder drive as some drives may not be changed

	for(int8_t drive = AXES; drive < DRIVES; drive++)
	{
		moveBuffer[drive] = 0.0;
	}

	// First do extrusion, and check, if we are extruding, that we have a tool to extrude with

	Tool* tool = reprap.GetCurrentTool();
	if(gb->Seen(EXTRUDE_LETTER))
	{
		if(tool == NULL)
		{
			platform->Message(BOTH_ERROR_MESSAGE, "Attempting to extrude with no tool selected.\n");
			return false;
		}
		int eMoveCount = tool->DriveCount();
		float eMovement[DRIVES-AXES];
		gb->GetFloatArray(eMovement, eMoveCount);
		if(tool->DriveCount() != eMoveCount)
		{
			scratchString.printf("Wrong number of extruder drives for the selected tool: %s\n", gb->Buffer());
			platform->Message(HOST_MESSAGE, scratchString);
			return false;
		}

		// Set the drive values for this tool.

		for(int8_t eDrive = 0; eDrive < eMoveCount; eDrive++)
		{
			int8_t drive = tool->Drive(eDrive);
			float moveArg = eMovement[eDrive] * distanceScale;
			if(doingG92)
			{
				moveBuffer[drive + AXES] = 0.0;		// no move required
				lastPos[drive] = moveArg;
			}
			else if(drivesRelative)
			{
				moveBuffer[drive + AXES] = moveArg * extrusionFactors[drive];
				lastPos[drive] += moveArg;
			}
			else
			{
				moveBuffer[drive + AXES] = (moveArg - lastPos[drive]) * extrusionFactors[drive];
				lastPos[drive] = moveArg;
			}
		}
	}

	// Now the movement axes

	for(uint8_t axis = 0; axis < AXES; axis++)
	{
		if(gb->Seen(axisLetters[axis]))
		{
			float moveArg = gb->GetFValue() * distanceScale;
			if (axesRelative && !doingG92)
			{
				moveArg += moveBuffer[axis];
			}
			if (applyLimits && axis < 2 && axisIsHomed[axis] && !doingG92)	// limit X & Y moves unless doing G92.  FIXME: No Z for the moment as we often need to move -ve to set the origin
			{
				if (moveArg < platform->AxisMinimum(axis))
				{
					moveArg = platform->AxisMinimum(axis);
				} else if (moveArg > platform->AxisMaximum(axis))
				{
					moveArg = platform->AxisMaximum(axis);
				}
			}
			moveBuffer[axis] = moveArg;
			if (doingG92)
			{
				axisIsHomed[axis] = true;		// doing a G92 defines the absolute axis position
			}
		}
	}

	// Deal with feed rate

	if(gb->Seen(FEEDRATE_LETTER))
	{
		moveBuffer[DRIVES] = gb->GetFValue() * distanceScale * speedFactor; // G Code feedrates are in mm/minute; we need mm/sec
	}

	return true;
}



// This function is called for a G Code that makes a move.
// If the Move class can't receive the move (i.e. things have to wait), return 0.
// If we have queued the move and the caller doesn't need to wait for it to complete, return 1.
// If we need to wait for the move to complete before doing another one (because endstops are checked in this move), return 2.

int GCodes::SetUpMove(GCodeBuffer *gb)
{
	// Last one gone yet?
	if (moveAvailable)
		return 0;

	// Load the last position and feed rate into moveBuffer; If Move can't accept more, return false
	if (!reprap.GetMove()->GetCurrentUserPosition(moveBuffer))
		return 0;

	moveBuffer[DRIVES] *= speedFactorChange;		// account for any change in the speed factor since the last move
	speedFactorChange = 1.0;

	// Check to see if the move is a 'homing' move that endstops are checked on.
	endStopsToCheck = 0;
	if (gb->Seen('S'))
	{
		if (gb->GetIValue() == 1)
		{
			for (unsigned int i = 0; i < AXES; ++i)
			{
				if (gb->Seen(axisLetters[i]))
				{
					endStopsToCheck |= (1 << i);
				}
			}
		}
	}

	// Load the move buffer with either the absolute movement required or the relative movement required
	moveAvailable = LoadMoveBufferFromGCode(gb, false, (endStopsToCheck == 0) && limitAxes);
	return (endStopsToCheck != 0) ? 2 : 1;
}

// The Move class calls this function to find what to do next.

bool GCodes::ReadMove(float m[], EndstopChecks& ce)
{
	if (!moveAvailable)
		return false;
	for (int8_t i = 0; i <= DRIVES; i++) // 1 more for feedrate
	{
		m[i] = moveBuffer[i];
	}
	ce = endStopsToCheck;
	moveAvailable = false;
	endStopsToCheck = 0;
	return true;
}

bool GCodes::DoFileCannedCycles(const char* fileName)
{
	// Have we started the file?

	if (!doingCannedCycleFile)
	{
		// No

		if (!Push())
			return false;

		FileStore *f = platform->GetFileStore(platform->GetSysDir(), fileName, false);
		if (f == NULL)
		{
			// Don't use snprintf into scratchString here, because fileName may be aliased to scratchString
			platform->Message(HOST_MESSAGE, "Macro file ");
			platform->Message(HOST_MESSAGE, fileName);
			platform->Message(HOST_MESSAGE, " not found.\n");
			if(!Pop())
			{
				platform->Message(HOST_MESSAGE, "Cannot pop the stack.\n");
			}
			return true;
		}
		fileBeingPrinted.Set(f);
		doingCannedCycleFile = true;
		cannedCycleGCode->Init();
		return false;
	}

	// Complete the current move (must do this before checking whether we have finished the file in case it didn't end in newline)

	if (cannedCycleGCode->Active())
	{
		cannedCycleGCode->SetFinished(ActOnCode(cannedCycleGCode));
		return false;
	}

	// Have we finished the file?

	if (!fileBeingPrinted.IsLive())
	{
		// Yes

		if (!Pop())
			return false;
		doingCannedCycleFile = false;
		cannedCycleGCode->Init();
		return true;
	}

	// No - Do more of the file

	DoFilePrint(cannedCycleGCode);
	return false;
}

bool GCodes::FileCannedCyclesReturn()
{
	if (!doingCannedCycleFile)
		return true;

	if (!AllMovesAreFinishedAndMoveBufferIsLoaded())
		return false;

	doingCannedCycleFile = false;
	cannedCycleGCode->Init();

	fileBeingPrinted.Close();
	return true;
}

// To execute any move, call this until it returns true.
// moveToDo[] entries corresponding with false entries in action[] will
// be ignored.  Recall that moveToDo[DRIVES] should contain the feedrate
// you want (if action[DRIVES] is true).

bool GCodes::DoCannedCycleMove(EndstopChecks ce)
{
	// Is the move already running?

	if (cannedCycleMoveQueued)
	{ // Yes.
		if (!Pop()) // Wait for the move to finish then restore the state
			return false;
		cannedCycleMoveQueued = false;
		return true;
	}
	else
	{ // No.
		if (!Push()) // Wait for the RepRap to finish whatever it was doing, save it's state, and load moveBuffer[] with the current position.
			return false;
		for (int8_t drive = 0; drive <= DRIVES; drive++)
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
		return false;

	if(LoadMoveBufferFromGCode(gb, true, false))
	{
		// Transform the position so that e.g. if the user does G92 Z0,
		// the position we report (which gets inverse-transformed) really is Z=0 afterwards
		reprap.GetMove()->Transform(moveBuffer);
		reprap.GetMove()->SetLiveCoordinates(moveBuffer);
		reprap.GetMove()->SetPositions(moveBuffer);
		reprap.GetMove()->SetFeedrate(platform->InstantDv(platform->SlowestDrive()));  // On a G92 we must effectively be stationary
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
		for (int8_t drive = 0; drive <= DRIVES; drive++)
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

		for (int8_t axis = 0; axis < AXES; axis++)
		{
			if (gb->Seen(axisLetters[axis]))
			{
				moveToDo[axis] += gb->GetFValue();
				activeDrive[axis] = true;
			}
		}

		if(gb->Seen(FEEDRATE_LETTER)) // Has the user specified a feedrate?
		{
			moveToDo[DRIVES] = gb->GetFValue();
			activeDrive[DRIVES] = true;
		}

		offSetSet = true;
	}

	if (DoCannedCycleMove(0))
	{
		//LoadMoveBufferFromArray(record);
		for (int drive = 0; drive <= DRIVES; drive++)
		{
			moveBuffer[drive] = record[drive];
		}
		reprap.GetMove()->SetLiveCoordinates(record); // This doesn't transform record
		reprap.GetMove()->SetPositions(record);        // This does
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
bool GCodes::DoHome(StringRef& reply, bool& error)
//pre(reply.upb == STRING_LENGTH)
{
	if (homeX && homeY && homeZ)
	{
		if (DoFileCannedCycles(HOME_ALL_G))
		{
			homeX = false;
			homeY = false;
			homeZ = false;
			return true;
		}
		return false;
	}

	if (homeX)
	{
		if (DoFileCannedCycles(HOME_X_G))
		{
			homeX = false;
			return NoHome();
		}
		return false;
	}

	if (homeY)
	{
		if (DoFileCannedCycles(HOME_Y_G))
		{
			homeY = false;
			return NoHome();
		}
		return false;
	}

	if (homeZ)
	{
		if (platform->MustHomeXYBeforeZ() && (!axisIsHomed[X_AXIS] || !axisIsHomed[Y_AXIS]))
		{
			// We can only home Z if X and Y have already been homed
			reply.copy("Must home X and Y before homing Z");
			error = true;
			homeZ = false;
			return true;
		}
		if (DoFileCannedCycles(HOME_Z_G))
		{
			homeZ = false;
			return NoHome();
		}
		return false;
	}

	// Should never get here

	endStopsToCheck = 0;
	moveAvailable = false;

	return true;
}

// This lifts Z a bit, moves to the probe XY coordinates (obtained by a call to GetProbeCoordinates() ),
// probes the bed height, and records the Z coordinate probed.  If you want to program any general
// internal canned cycle, this shows how to do it.

bool GCodes::DoSingleZProbeAtPoint()
{
	reprap.GetMove()->SetIdentityTransform(); // It doesn't matter if these are called repeatedly

	for (int8_t drive = 0; drive <= DRIVES; drive++)
	{
		activeDrive[drive] = false;
	}

	switch (cannedCycleMoveCount)
	{
	case 0: // Raise Z to 5mm. This only does anything on the first move; on all the others Z is already there
		moveToDo[Z_AXIS] = Z_DIVE;
		activeDrive[Z_AXIS] = true;
		moveToDo[DRIVES] = platform->MaxFeedrate(Z_AXIS);
		activeDrive[DRIVES] = true;
		reprap.GetMove()->SetZProbing(false);
		if (DoCannedCycleMove(0))
		{
			cannedCycleMoveCount++;
		}
		return false;

	case 1:	// Move to the correct XY coordinates
		GetProbeCoordinates(probeCount, moveToDo[X_AXIS], moveToDo[Y_AXIS], moveToDo[Z_AXIS]);
		activeDrive[X_AXIS] = true;
		activeDrive[Y_AXIS] = true;
		// NB - we don't use the Z value
		moveToDo[DRIVES] = platform->MaxFeedrate(X_AXIS);
		activeDrive[DRIVES] = true;
		reprap.GetMove()->SetZProbing(false);
		if (DoCannedCycleMove(0))
		{
			cannedCycleMoveCount++;
			platform->SetZProbing(true);	// do this here because we only want to call it once
		}
		return false;

	case 2:	// Probe the bed
		moveToDo[Z_AXIS] = -2.0 * platform->AxisMaximum(Z_AXIS);
		activeDrive[Z_AXIS] = true;
		moveToDo[DRIVES] = platform->HomeFeedRate(Z_AXIS);
		activeDrive[DRIVES] = true;
		reprap.GetMove()->SetZProbing(true);
		if (DoCannedCycleMove(1 << Z_AXIS))
		{
			cannedCycleMoveCount++;
			platform->SetZProbing(false);
		}
		return false;

	case 3:	// Raise the head 5mm
		moveToDo[Z_AXIS] = Z_DIVE;
		activeDrive[Z_AXIS] = true;
		moveToDo[DRIVES] = platform->MaxFeedrate(Z_AXIS);
		activeDrive[DRIVES] = true;
		reprap.GetMove()->SetZProbing(false);
		if (DoCannedCycleMove(0))
		{
			cannedCycleMoveCount++;
		}
		return false;

	default:
		cannedCycleMoveCount = 0;
		reprap.GetMove()->SetZBedProbePoint(probeCount, reprap.GetMove()->GetLastProbedZ());
		return true;
	}
}

// This simply moves down till the Z probe/switch is triggered.

bool GCodes::DoSingleZProbe()
{
	for (int8_t drive = 0; drive <= DRIVES; drive++)
	{
		activeDrive[drive] = false;
	}

	switch (cannedCycleMoveCount)
	{
	case 0:
		platform->SetZProbing(true);	// we only want to call this once
		++cannedCycleMoveCount;
		return false;

	case 1:
		moveToDo[Z_AXIS] = -1.1 * platform->AxisTotalLength(Z_AXIS);
		activeDrive[Z_AXIS] = true;
		moveToDo[DRIVES] = platform->HomeFeedRate(Z_AXIS);
		activeDrive[DRIVES] = true;
		if (DoCannedCycleMove(1 << Z_AXIS))
		{
			cannedCycleMoveCount++;
			probeCount = 0;
			platform->SetZProbing(false);
		}
		return false;

	default:
		cannedCycleMoveCount = 0;
		return true;
	}
}

// This sets wherever we are as the probe point P (probePointIndex)
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

	float x = (gb->Seen(axisLetters[X_AXIS])) ? gb->GetFValue() : moveBuffer[X_AXIS];
	float y = (gb->Seen(axisLetters[Y_AXIS])) ? gb->GetFValue() : moveBuffer[Y_AXIS];
	float z = (gb->Seen(axisLetters[Z_AXIS])) ? gb->GetFValue() : moveBuffer[Z_AXIS];

	probeCount = probePointIndex;
	reprap.GetMove()->SetXBedProbePoint(probeCount, x);
	reprap.GetMove()->SetYBedProbePoint(probeCount, y);

	if (z > SILLY_Z_VALUE)
	{
		reprap.GetMove()->SetZBedProbePoint(probeCount, z);
		reprap.GetMove()->SetZProbing(false); // Not really needed, but let's be safe
		probeCount = 0;
		if (gb->Seen('S'))
		{
			zProbesSet = true;
			reprap.GetMove()->SetProbedBedEquation(reply);
		}
		return true;
	}
	else
	{
		if (DoSingleZProbeAtPoint())
		{
			probeCount = 0;
			reprap.GetMove()->SetZProbing(false);
			if (gb->Seen('S'))
			{
				zProbesSet = true;
				reprap.GetMove()->SetProbedBedEquation(reply);
			}
			return true;
		}
	}

	return false;
}

// This probes multiple points on the bed (three in a
// triangle or four in the corners), then sets the bed transformation to compensate
// for the bed not quite being the plane Z = 0.

bool GCodes::DoMultipleZProbe(StringRef& reply)
{
	if (reprap.GetMove()->NumberOfXYProbePoints() < 3)
	{
		reply.copy("Bed probing: there needs to be 3 or more points set.\n");
		return true;
	}

	if (DoSingleZProbeAtPoint())
	{
		probeCount++;
	}
	if (probeCount >= reprap.GetMove()->NumberOfXYProbePoints())
	{
		probeCount = 0;
		zProbesSet = true;
		reprap.GetMove()->SetZProbing(false);
		reprap.GetMove()->SetProbedBedEquation(reply);
		return true;
	}
	return false;
}

// This returns the (X, Y) points to probe the bed at probe point count.  When probing,
// it returns false.  If called after probing has ended it returns true, and the Z coordinate
// probed is also returned.

bool GCodes::GetProbeCoordinates(int count, float& x, float& y, float& z) const
{
	x = reprap.GetMove()->XBedProbePoint(count);
	y = reprap.GetMove()->YBedProbePoint(count);
	z = reprap.GetMove()->ZBedProbePoint(count);
	return zProbesSet;
}

bool GCodes::SetPrintZProbe(GCodeBuffer* gb, StringRef& reply)
{
	if (!AllMovesAreFinishedAndMoveBufferIsLoaded())
		return false;

	if (gb->Seen(axisLetters[Z_AXIS]))
	{
		ZProbeParameters params;
		platform->GetZProbeParameters(params);
		params.height = gb->GetFValue();
		if (gb->Seen('P'))
		{
			params.adcValue = gb->GetIValue();
		}
		if (gb->Seen('S'))
		{
			params.calibTemperature = gb->GetFValue();
		}
		else
		{
			// Use the current bed temperature as the calibration temperature if no value was provided
			params.calibTemperature = platform->GetTemperature(HOT_BED);
		}
		if (gb->Seen('C'))
		{
			params.temperatureCoefficient = gb->GetFValue();
		}
		else
		{
			params.temperatureCoefficient = 0.0;
		}
		platform->SetZProbeParameters(params);
	}
	else
	{
		int v0 = platform->ZProbe();
		int v1, v2;
		switch(platform->GetZProbeSecondaryValues(v1, v2))
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

const char* GCodes::GetCurrentCoordinates() const
{
	float liveCoordinates[DRIVES + 1];
	reprap.GetMove()->LiveCoordinates(liveCoordinates);

	scratchString.printf("X:%.2f Y:%.2f Z:%.2f ", liveCoordinates[X_AXIS], liveCoordinates[Y_AXIS], liveCoordinates[Z_AXIS]);
	for(int i = AXES; i< DRIVES; i++)
	{
		scratchString.catf("E%d:%.1f ", i-AXES, liveCoordinates[i]);
	}
	return scratchString.Pointer();
}

bool GCodes::OpenFileToWrite(const char* directory, const char* fileName, GCodeBuffer *gb)
{
	fileBeingWritten = platform->GetFileStore(directory, fileName, true);
	eofStringCounter = 0;
	if (fileBeingWritten == NULL)
	{
		platform->Message(HOST_MESSAGE, "Can't open GCode file for writing.\n");
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
		platform->Message(HOST_MESSAGE, "Attempt to write to a null file.\n");
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
			HandleReply(false, gb == serialGCode, r, 'M', 560, false);
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
		platform->Message(HOST_MESSAGE, "Attempt to write to a null file.\n");
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
			HandleReply(false, gb == serialGCode, r, 'M', 29, false);
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
				scratchString.printf("%s", gb->GetIValue());
				HandleReply(false, gb == serialGCode, scratchString.Pointer(), 'G', 998, true);
				return;
			}
		}
	}

	fileBeingWritten->Write(gb->Buffer());
	fileBeingWritten->Write('\n');
	HandleReply(false, gb == serialGCode, "", 'G', 1, false);
}

// Set up a file to print, but don't print it yet.

void GCodes::QueueFileToPrint(const char* fileName)
{
	fileToPrint.Close();
	fileGCode->CancelPause();	// if we paused it and then asked to print a new file, cancel any pending command
	FileStore *f = platform->GetFileStore(platform->GetGCodeDir(), fileName, false);
	if (f != NULL)
	{
		// Reset all extruder positions when starting a new print
		for (int8_t extruder = AXES; extruder < DRIVES; extruder++)
		{
			lastPos[extruder - AXES] = 0.0;
		}

		fileToPrint.Set(f);
		if (!fileBeingPrinted.IsLive())
		{
			fileGCode->SetToolNumberAdjust(0);	// clear tool number adjustment
		}
	}
	else
	{
		platform->Message(BOTH_ERROR_MESSAGE, "GCode file not found\n");
	}
}

void GCodes::DeleteFile(const char* fileName)
{
	if(!platform->GetMassStorage()->Delete(platform->GetGCodeDir(), fileName))
	{
		scratchString.printf("Unsuccessful attempt to delete: %s\n", fileName);
		platform->Message(BOTH_ERROR_MESSAGE, scratchString);
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
			platform->Message(HOST_MESSAGE, "Configuration file not found\n");
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

// Function to handle dwell delays.  Return true for
// dwell finished, false otherwise.

bool GCodes::DoDwell(GCodeBuffer *gb)
{
	if(!gb->Seen('P'))
		return true;  // No time given - throw it away

	float dwell = 0.001 * (float) gb->GetLValue(); // P values are in milliseconds; we need seconds

	// Wait for all the queued moves to stop

	if (!reprap.GetMove()->AllMovesAreFinished())
		return false;

	return DoDwellTime(dwell);
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

// Set working and standby temperatures for
// a tool.  I.e. handle a G10.

void GCodes::SetOrReportOffsets(StringRef& reply, GCodeBuffer *gb)
{
	if(gb->Seen('P'))
	{
		int8_t toolNumber = gb->GetIValue();
		toolNumber += gb->GetToolNumberAdjust();
		Tool* tool = reprap.GetTool(toolNumber);
		if(tool == NULL)
		{
			reply.printf("Attempt to set/report offsets and temperatures for non-existent tool: %d\n", toolNumber);
			return;
		}
		float standby[HEATERS];
		float active[HEATERS];
		tool->GetVariables(standby, active);
		int hCount = tool->HeaterCount();
		if(hCount > 0)
		{
			bool setting = false;
			if(gb->Seen('R'))
			{
				gb->GetFloatArray(standby, hCount);
				setting = true;
			}
			if(gb->Seen('S'))
			{
				gb->GetFloatArray(active, hCount);
				setting = true;
			}
			if(setting)
			{
				tool->SetVariables(standby, active);
			}
			else
			{
				reply.printf("Tool %d - Active/standby temperature(s): ", toolNumber);
				for(int8_t heater = 0; heater < hCount; heater++)
				{
					reply.catf("%.1f/%.1f ", active[heater], standby[heater]);
				}
			}
		}
	}
}

void GCodes::AddNewTool(GCodeBuffer *gb, StringRef& reply)
{
	if(!gb->Seen('P'))
	{
		// DC temporary code to allow tool numbers to be adjusted so that we don't need to edit multi-media files generated by slic3r
		if (gb->Seen('S'))
		{
			int adjust = gb->GetIValue();
			gb->SetToolNumberAdjust(adjust);
		}
		return;
	}

	int toolNumber = gb->GetLValue();
	bool seen = false;

	long drives[DRIVES - AXES];  // There can never be more than we have...
	int dCount = DRIVES - AXES;  // Sets the limit and returns the count
	if(gb->Seen('D'))
	{
		gb->GetLongArray(drives, dCount);
		seen = true;
	}
	else
	{
		dCount = 0;
	}

	long heaters[HEATERS];
	int hCount = HEATERS;
	if(gb->Seen('H'))
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
		Tool* tool = new Tool(toolNumber, drives, dCount, heaters, hCount);
		reprap.AddTool(tool);
	}
	else
	{
		reprap.PrintTool(toolNumber, reply);
	}
}

// Does what it says.

bool GCodes::DisableDrives()
{
	if (!AllMovesAreFinishedAndMoveBufferIsLoaded())
		return false;
	for (int8_t drive = 0; drive < DRIVES; drive++)
	{
		platform->Disable(drive);
	}
	return true;
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
				platform->Message(HOST_MESSAGE, "Dud IP address: ");
				platform->Message(HOST_MESSAGE, gb->Buffer());
				platform->Message(HOST_MESSAGE, "\n");
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
			platform->Message(HOST_MESSAGE, "Setting ether parameter - dud code.");
		}
	}
	else
	{
		platform->Message(HOST_MESSAGE, "Dud IP address: ");
		platform->Message(HOST_MESSAGE, gb->Buffer());
		platform->Message(HOST_MESSAGE, "\n");
	}
}


void GCodes::SetMACAddress(GCodeBuffer *gb)
{
	uint8_t mac[6];
	const char* ipString = gb->GetString();
	uint8_t sp = 0;
	uint8_t spp = 0;
	uint8_t ipp = 0;
	while(ipString[sp])
	{
		if(ipString[sp] == ':')
		{
			mac[ipp] = strtoul(&ipString[spp], NULL, 16);
			ipp++;
			if(ipp > 5)
			{
				platform->Message(HOST_MESSAGE, "Dud MAC address: ");
				platform->Message(HOST_MESSAGE, gb->Buffer());
				platform->Message(HOST_MESSAGE, "\n");
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
	if(ipp == 5)
	{
		platform->SetMACAddress(mac);
	}
	else
	{
		platform->Message(HOST_MESSAGE, "Dud MAC address: ");
		platform->Message(HOST_MESSAGE, gb->Buffer());
		platform->Message(HOST_MESSAGE, "\n");
	}
//	snprintf(scratchString, STRING_LENGTH, "MAC: %x:%x:%x:%x:%x:%x\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
//	platform->Message(HOST_MESSAGE, scratchString);
}

void GCodes::HandleReply(bool error, bool fromLine, const char* reply, char gMOrT, int code, bool resend)
{
	if (gMOrT != 'M' || (code != 111 && code != 122))	// web server reply for M111 and M122 is handled before we get here
	{
		platform->Message((error) ? WEB_ERROR_MESSAGE : WEB_MESSAGE, reply);
	}

	Compatibility c = platform->Emulating();
	if (!fromLine)
	{
		c = me;
	}

	const char* response = "ok";
	if (resend)
	{
		response = "rs ";
	}

	const char* s = 0;

	switch (c)
	{
	case me:
	case reprapFirmware:
		if (!reply[0])
			return;
		if (error)
		{
			platform->GetLine()->Write("Error: ");
		}
		platform->GetLine()->Write(reply);
		platform->GetLine()->Write("\n");
		return;

	case marlin:
		if (gMOrT == 'M' && code == 20)
		{
			platform->GetLine()->Write("Begin file list\n");
			platform->GetLine()->Write(reply);
			platform->GetLine()->Write("\nEnd file list\n");
			platform->GetLine()->Write(response);
			platform->GetLine()->Write("\n");
			return;
		}

		if (gMOrT == 'M' && code == 28)
		{
			platform->GetLine()->Write(response);
			platform->GetLine()->Write("\n");
			platform->GetLine()->Write(reply);
			platform->GetLine()->Write("\n");
			return;
		}

		if ((gMOrT == 'M' && code == 105) || (gMOrT == 'G' && code == 998))
		{
			platform->GetLine()->Write(response);
			platform->GetLine()->Write(" ");
			platform->GetLine()->Write(reply);
			platform->GetLine()->Write("\n");
			return;
		}

		if (reply[0])
		{
			platform->GetLine()->Write(reply);
			platform->GetLine()->Write("\n");
		}
		platform->GetLine()->Write(response);
		platform->GetLine()->Write("\n");
		return;

	case teacup:
		s = "teacup";
		break;
	case sprinter:
		s = "sprinter";
		break;
	case repetier:
		s = "repetier";
		break;
	default:
		s = "unknown";
	}

	if (s != 0)
	{
		scratchString.printf("Emulation of %s is not yet supported.\n", s);
		platform->Message(HOST_MESSAGE, scratchString);
	}
}

// Set PID parameters (M301 or M303 command). 'heater' is the default heater number to use.
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
						heater, pp.kP, pp.kI, pp.kD, pp.kT, pp.kS, pp.pidMax, pp.fullBand);
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

			if (seen)
			{
				platform->SetPidParameters(heater, pp);
			}
			else
			{
				reply.printf("T:%.1f B:%.1f R:%.1f L:%.1f H:%.1f\n",
						r25, beta, pp.thermistorSeriesR, pp.adcLowOffset, pp.adcHighOffset);
			}
		}
	}
}

void GCodes::SetToolHeaters(Tool *tool, float temperature)
{
	if(tool == NULL)
	{
		platform->Message(HOST_MESSAGE, "Setting temperature: no tool selected.\n");
		return;
	}

	float standby[HEATERS];
	float active[HEATERS];
	tool->GetVariables(standby, active);
	for(int8_t h = 0; h < tool->HeaterCount(); h++)
	{
		active[h] = temperature;
	}
	tool->SetVariables(standby, active);
}

// If the code to act on is completed, this returns true,
// otherwise false.  It is called repeatedly for a given
// code until it returns true for that code.

bool GCodes::ActOnCode(GCodeBuffer *gb)
{
	// M-code parameters might contain letters T and G, e.g. in filenames.
	// dc42 assumes that G-and T-code parameters never contain the letter M.
	// Therefore we must check for an M-code first.
	if (gb->Seen('M'))
	{
		return HandleMcode(gb);
	}
	// dc42 doesn't think a G-code parameter ever contains letter T, or a T-code ever contains letter G.
	// So it doesn't matter in which order we look for them.
	if (gb->Seen('G'))
	{
		return HandleGcode(gb);
	}
	if (gb->Seen('T'))
	{
		return HandleTcode(gb);
	}

	// An empty buffer gets discarded
	HandleReply(false, gb == serialGCode, "", 'X', 0, false);
	return true;
}

bool GCodes::HandleGcode(GCodeBuffer* gb)
{
	bool result = true;
	bool error = false;
	bool resend = false;
	char replyBuffer[STRING_LENGTH];
	StringRef reply(replyBuffer, ARRAY_SIZE(replyBuffer));
	reply.Clear();

	int code = gb->GetIValue();
	switch (code)
	{
	case 0: // There are no rapid moves...
	case 1: // Ordinary move
		if (waitingForMoveToComplete)
		{
			// We have already set up this move, but it does endstop checks, so wait for it to complete.
			// Otherwise, if the next move uses relative coordinates, it will be incorrectly calculated.
			result = AllMovesAreFinishedAndMoveBufferIsLoaded();
			if (result)
			{
				waitingForMoveToComplete = false;
			}
		}
		else
		{
			int res = SetUpMove(gb);
			if (res == 2)
			{
				waitingForMoveToComplete = true;
			}
			result = (res == 1);
		}
		break;

	case 4: // Dwell
		result = DoDwell(gb);
		break;

	case 10: // Set/report offsets
		SetOrReportOffsets(reply, gb);
		break;

	case 20: // Inches (which century are we living in, here?)
		distanceScale = INCH_TO_MM;
		break;

	case 21: // mm
		distanceScale = 1.0;
		break;

	case 28: // Home
		if (NoHome())
		{
			homeX = gb->Seen(axisLetters[X_AXIS]);
			homeY = gb->Seen(axisLetters[Y_AXIS]);
			homeZ = gb->Seen(axisLetters[Z_AXIS]);
			if (NoHome())
			{
				homeX = true;
				homeY = true;
				homeZ = true;
			}
		}
		result = DoHome(reply, error);
		break;

	case 30: // Z probe/manually set at a position and set that as point P
		result = SetSingleZProbeAtAPosition(gb, reply);
		break;

	case 31: // Return the probe value, or set probe variables
		result = SetPrintZProbe(gb, reply);
		break;

	case 32: // Probe Z at multiple positions and generate the bed transform
		if (!(axisIsHomed[X_AXIS] && axisIsHomed[Y_AXIS]))
		{
			// We can only do bed levelling if X and Y have already been homed
			reply.copy("Must home X and Y before bed probing");
			error = true;
			result = true;
		}
		else
		{
			result = DoMultipleZProbe(reply);
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
		reply.printf("invalid G Code: %s", gb->Buffer());
	}
	if (result)
	{
		HandleReply(error, gb == serialGCode, reply.Pointer(), 'G', code, resend);
	}
	return result;
}

bool GCodes::HandleMcode(GCodeBuffer* gb)
{
	bool result = true;
	bool error = false;
	bool resend = false;
	char replyBuffer[STRING_LENGTH];
	StringRef reply(replyBuffer, ARRAY_SIZE(replyBuffer));
	reply.Clear();

	int code = gb->GetIValue();
	switch (code)
	{
	case 0: // Stop
	case 1: // Sleep
		if (!AllMovesAreFinishedAndMoveBufferIsLoaded())
			return false;

		if (fileBeingPrinted.IsLive())
		{
			fileToPrint.MoveFrom(fileBeingPrinted);
		}

		// Deselect the active tool
		{
			Tool* tool = reprap.GetCurrentTool();
			if(tool != NULL)
			{
				reprap.StandbyTool(tool->Number());
			}
		}

		if (!DisableDrives())
			return false;

		reprap.GetHeat()->SwitchOffAll();
		break;

	case 18: // Motors off
		result = DisableDrives();
		break;

	case 20:  // Deprecated...
		{
			bool encapsulate_list;
			if (platform->Emulating() == me || platform->Emulating() == reprapFirmware)
			{
				reply.copy("GCode files:\n");
				encapsulate_list = false;
			}
			else
			{
				reply.Clear();
				encapsulate_list = true;
			}

			FileInfo file_info;
			if (platform->GetMassStorage()->FindFirst(platform->GetGCodeDir(), file_info))
			{
				// iterate through all entries and append each file name
				do {
					if (encapsulate_list)
					{
						reply.catf("%c%s%c%c", FILE_LIST_BRACKET, file_info.fileName, FILE_LIST_BRACKET, FILE_LIST_SEPARATOR);
					}
					else
					{
						reply.catf("%s\n", file_info.fileName);
					}
				} while (platform->GetMassStorage()->FindNext(file_info));

				// remove the last character
				reply[reply.strlen() - 1] = 0;
			}
			else
			{
				reply.cat("NONE");
			}
		}
		break;

	case 21: // Initialise SD - ignore
		break;

	case 23: // Set file to print
		QueueFileToPrint(gb->GetUnprecedentedString());
		if (fileToPrint.IsLive() && platform->Emulating() == marlin)
		{
			reply.copy("File opened\nFile selected\n");
		}
		break;

	case 24: // Print/resume-printing the selected file
		if (fileBeingPrinted.IsLive())
			break;
		fileBeingPrinted.MoveFrom(fileToPrint);
		break;

	case 25: // Pause the print
		fileToPrint.MoveFrom(fileBeingPrinted);
		break;

	case 27: // Report print status - Deprecated
		if (fileBeingPrinted.IsLive())
		{
			reply.copy("SD printing.");
		}
		else
		{
			reply.copy("Not SD printing.");
		}
		break;

	case 28: // Write to file
		{
			const char* str = gb->GetUnprecedentedString();
			bool ok = OpenFileToWrite(platform->GetGCodeDir(), str, gb);
			if (ok)
			{
				reply.printf("Writing to file: %s", str);
			}
			else
			{
				reply.printf("Can't open file %s for writing.\n", str);
				error = true;
			}
		}
		break;

	case 29: // End of file being written; should be intercepted before getting here
		platform->Message(HOST_MESSAGE, "GCode end-of-file being interpreted.\n");
		break;

	case 30:	// Delete file
		DeleteFile(gb->GetUnprecedentedString());
		break;

	case 80:	// ATX power on
	case 81:	// ATX power off
		platform->SetAtxPower(code == 80);
		break;

	case 82:	// Use absolute extruder positioning
		if (drivesRelative)		// don't reset the absolute extruder position if it was already absolute
		{
			for (int8_t extruder = AXES; extruder < DRIVES; extruder++)
			{
				lastPos[extruder - AXES] = 0.0;
			}
			drivesRelative = false;
		}
		break;

	case 83:	// Use relative extruder positioning
		if (!drivesRelative)	// don't reset the absolute extruder position if it was already relative
		{
			for (int8_t extruder = AXES; extruder < DRIVES; extruder++)
			{
				lastPos[extruder - AXES] = 0.0;
			}
			drivesRelative = true;
		}
		break;

	case 84: // Motors off - deprecated, use M18
		result = DisableDrives();
		break;

	case 85: // Set inactive time
		break;

	case 92: // Set/report steps/mm for some axes
		{
			bool seen = false;
			for(int8_t axis = 0; axis < AXES; axis++)
			{
				if(gb->Seen(axisLetters[axis]))
				{
					platform->SetDriveStepsPerUnit(axis, gb->GetFValue());
					seen = true;
				}
			}

			if(gb->Seen(EXTRUDE_LETTER))
			{
				seen = true;
				float eVals[DRIVES-AXES];
				int eCount = DRIVES-AXES;
				gb->GetFloatArray(eVals, eCount);
				if(eCount != DRIVES-AXES)
				{
					scratchString.printf("Setting steps/mm - wrong number of E drives: %s\n", gb->Buffer());
					platform->Message(HOST_MESSAGE, scratchString);
				}
				else
				{
					for(int8_t e = 0; e < eCount; e++)
					{
						platform->SetDriveStepsPerUnit(AXES + e, eVals[e]);
					}
				}
			}

			if(!seen)
			{
				reply.printf("Steps/mm: X: %.3f, Y: %.3f, Z: %.3f, E: ",
						platform->DriveStepsPerUnit(X_AXIS), platform->DriveStepsPerUnit(Y_AXIS),
						platform->DriveStepsPerUnit(Z_AXIS));
				for(int8_t drive = AXES; drive < DRIVES; drive++)
				{
					reply.catf("%.3f", platform->DriveStepsPerUnit(drive));
					if(drive < DRIVES-1)
					{
						reply.cat(":");
					}
				}
			}
		}
		break;

	case 98:
		if (gb->Seen('P'))
		{
			result = DoFileCannedCycles(gb->GetString());
		}
		break;

	case 99:
		result = FileCannedCyclesReturn();
		break;

    case 104: // Deprecated.  This sets the active temperature of every heater of the active tool
    	if(gb->Seen('S'))
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

	case 105: // Deprecated...
		reply.copy("T:");
		for(int8_t heater = 1; heater < HEATERS; heater++)
		{
			if(reprap.GetHeat()->GetStatus(heater) != Heat::HS_off)
			{
				reply.catf("%.1f ", reprap.GetHeat()->GetTemperature(heater));
			}
		}
		reply.catf("B: %.1f ", reprap.GetHeat()->GetTemperature(0));
		break;
   
	case 106: // Fan on or off
		if (gb->Seen('I'))
		{
			coolingInverted = (gb->GetIValue() > 0);
		}
		if (gb->Seen('S'))
		{
			float f = gb->GetFValue();
			f = min<float>(f, 255.0);
			f = max<float>(f, 0.0);
			if (coolingInverted)
			{
				// Check if 1.0 or 255.0 may be used as the maximum value
				platform->CoolingFan((f <= 1.0 ? 1.0 : 255.0) - f);
			}
			else
			{
				platform->CoolingFan(f);
			}
		}
		break;

	case 107: // Fan off - deprecated
		platform->CoolingFan(coolingInverted ? 255.0 : 0.0);
		break;

    case 109: // Deprecated
    	if(gb->Seen('S'))
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
			if (!AllMovesAreFinishedAndMoveBufferIsLoaded())	// tell Move not to wait for more moves
			{
				return false;
			}
			result = ToolHeatersAtSetTemperatures(tool);
    	}
    	break;

	case 110: // Set line numbers - line numbers are dealt with in the GCodeBuffer class
		break;

	case 111: // Debug level
    	if(gb->Seen('S'))
    	{
    		int dbv = gb->GetIValue();
    		if(dbv == WEB_DEBUG_TRUE)
    		{
    			reprap.GetWebserver()->WebDebug(true);
    		}
    		else if (dbv == WEB_DEBUG_FALSE)
    		{
    			reprap.GetWebserver()->WebDebug(false);
    		}
    		else
    		{
    			reprap.SetDebug(dbv);
    		}
    	}
		break;

	case 112: // Emergency stop - acted upon in Webserver, but also here in case it comes from USB etc.
		reprap.EmergencyStop();
		break;

	case 114: // Deprecated
		{
			const char* str = GetCurrentCoordinates();
			if (str != 0)
			{
				reply.copy(str);
			}
			else
			{
				result = false;
			}
		}
		break;

	case 115: // Print firmware version
		reply.printf("FIRMWARE_NAME:%s FIRMWARE_VERSION:%s ELECTRONICS:%s DATE:%s", NAME, VERSION, ELECTRONICS, DATE);
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

    case 119:
		{
			reply.copy("Endstops - ");
			char comma = ',';
			for(int8_t axis = 0; axis < AXES; axis++)
			{
				const char* es;
				switch(platform->Stopped(axis))
				{
					case lowHit:
						es = "at min stop";
						break;

					case highHit:
						es = "at max stop";
						break;

					case lowNear:
						es = "near min stop";
						break;

					case noStop:
					default:
						es = "not stopped";
				}

				if(axis == AXES - 1)
				{
					comma = ' ';
				}

				reply.catf("%c: %s%c ", axisLetters[axis], es, comma);
			}
		}
		break;

	case 120:
		result = Push();
		break;

	case 121:
		result = Pop();
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
		platform->Message(HOST_MESSAGE, "M126 - valves not yet implemented\n");
		break;

	case 127: // Valve closed
		platform->Message(HOST_MESSAGE, "M127 - valves not yet implemented\n");
		break;

	case 135: // Set PID sample interval
		if(gb->Seen('S'))
		{
			platform->SetHeatSampleTime(gb->GetFValue() * 0.001);  // Value is in milliseconds; we want seconds
		}
		else
		{
			reply.printf("Heat sample time is %.3f seconds.", platform->HeatSampleTime());
		}
		break;

	case 140: // Set bed temperature
		if(gb->Seen('S'))
		{
			if(HOT_BED >= 0)
			{
				reprap.GetHeat()->SetActiveTemperature(HOT_BED, gb->GetFValue());
				reprap.GetHeat()->Activate(HOT_BED);
			}
		}
		if(gb->Seen('R'))
		{
			if(HOT_BED >= 0)
			{
				reprap.GetHeat()->SetStandbyTemperature(HOT_BED, gb->GetFValue());
			}
		}
		break;

	case 141: // Chamber temperature
		platform->Message(HOST_MESSAGE, "M141 - heated chamber not yet implemented\n");
		break;

//	case 160: //number of mixing filament drives  TODO: With tools defined, is this needed?
//    	if(gb->Seen('S'))
//		{
//			int iValue=gb->GetIValue();
//			platform->SetMixingDrives(iValue);
//		}
//		break;

	case 190: // Deprecated...
		if(gb->Seen('S'))
		{
			if(HOT_BED >= 0)
			{
				reprap.GetHeat()->SetActiveTemperature(HOT_BED, gb->GetFValue());
				reprap.GetHeat()->Activate(HOT_BED);
				if (!AllMovesAreFinishedAndMoveBufferIsLoaded())	// tell Move not to wait for more moves
				{
					return false;
				}
				result = reprap.GetHeat()->HeaterAtSetTemperature(HOT_BED);
			}
		}
		break;

	case 201: // Set/print axis accelerations  FIXME - should these be in /min not /sec ?
		{
			bool seen = false;
			for(int8_t axis = 0; axis < AXES; axis++)
			{
				if(gb->Seen(axisLetters[axis]))
				{
					platform->SetAcceleration(axis, gb->GetFValue()*distanceScale);
					seen = true;
				}
			}

			if(gb->Seen(EXTRUDE_LETTER))
			{
				seen = true;
				float eVals[DRIVES-AXES];
				int eCount = DRIVES-AXES;
				gb->GetFloatArray(eVals, eCount);
				if(eCount != DRIVES-AXES)
				{
					scratchString.printf("Setting accelerations - wrong number of E drives: %s\n", gb->Buffer());
					platform->Message(HOST_MESSAGE, scratchString);
				}
				else
				{
					for(int8_t e = 0; e < eCount; e++)
					{
						platform->SetAcceleration(AXES + e, eVals[e]*distanceScale);
					}
				}
			}

			if(!seen)
			{
				reply.printf("Accelerations: X: %.1f, Y: %.1f, Z: %.1f, E: ",
						platform->Acceleration(X_AXIS)/distanceScale, platform->Acceleration(Y_AXIS)/distanceScale,
						platform->Acceleration(Z_AXIS)/distanceScale);
				for(int8_t drive = AXES; drive < DRIVES; drive++)
				{
					reply.catf("%.1f", platform->Acceleration(drive)/distanceScale);
					if(drive < DRIVES-1)
					{
						reply.cat(":");
					}
				}
			}
    	}
    	break;

    case 203: // Set/print maximum feedrates
       	{
       		bool seen = false;
			for(int8_t axis = 0; axis < AXES; axis++)
			{
				if(gb->Seen(axisLetters[axis]))
				{
					platform->SetMaxFeedrate(axis, gb->GetFValue() * distanceScale * secondsToMinutes); // G Code feedrates are in mm/minute; we need mm/sec
					seen = true;
				}
			}

			if(gb->Seen(EXTRUDE_LETTER))
			{
				seen = true;
				float eVals[DRIVES-AXES];
				int eCount = DRIVES-AXES;
				gb->GetFloatArray(eVals, eCount);
				if(eCount != DRIVES-AXES)
				{
					scratchString.printf("Setting feedrates - wrong number of E drives: %s\n", gb->Buffer());
					platform->Message(HOST_MESSAGE, scratchString);
				}
				else
				{
					for(int8_t e = 0; e < eCount; e++)
					{
						platform->SetMaxFeedrate(AXES + e, eVals[e] * distanceScale * secondsToMinutes);
					}
				}
			}

			if(!seen)
			{
				reply.printf("Maximum feedrates: X: %.1f, Y: %.1f, Z: %.1f, E: ",
						platform->MaxFeedrate(X_AXIS)/(distanceScale * secondsToMinutes), platform->MaxFeedrate(Y_AXIS)/(distanceScale * secondsToMinutes),
						platform->MaxFeedrate(Z_AXIS)/(distanceScale * secondsToMinutes));
				for(int8_t drive = AXES; drive < DRIVES; drive++)
				{
					reply.catf("%.1f", platform->MaxFeedrate(drive)/(distanceScale * secondsToMinutes));
					if(drive < DRIVES-1)
					{
						reply.cat(":");
					}
				}
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
			bool setMin = (gb->Seen('S') ? (gb->GetIValue() == 1): false);
			bool seen = false;
			for (int8_t axis = 0; axis < AXES; axis++)
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
				for(int8_t axis = 0; axis < AXES; axis++)
				{
					if(axis == AXES - 1)
					{
						comma = ' ';
					}
					reply.catf("%c: %.1f min, %.1f max%c ", axisLetters[axis],
							platform->AxisMinimum(axis), platform->AxisMaximum(axis), comma);
				}
			}
		}
		break;

	case 210: // Set/print homing feed rates
		{
			bool seen = false;
			for (int8_t axis = 0; axis < AXES; axis++)
			{
				if (gb->Seen(axisLetters[axis]))
				{
					float value = gb->GetFValue() * distanceScale * secondsToMinutes;
					platform->SetHomeFeedRate(axis, value);
					seen = true;
				}
			}

			if(!seen)
			{
				reply.copy("Homing feedrates (mm/min) - ");
				char comma = ',';
				for(int8_t axis = 0; axis < AXES; axis++)
				{
					if(axis == AXES - 1)
					{
						comma = ' ';
					}

					reply.catf("%c: %.1f%c ", axisLetters[axis],
							platform->HomeFeedRate(axis) * 60.0 / distanceScale, comma);
				}
			}
		}
		break;

	case 220:	// set speed factor override percentage
		if (gb->Seen('S'))
		{
			float newSpeedFactor = gb->GetFValue()/(60.0 * 100.0);		// include the conversion from mm/minute to mm/second
			if (newSpeedFactor > 0)
			{
				speedFactorChange *= newSpeedFactor/speedFactor;
				speedFactor = newSpeedFactor;
			}
		}
		else
		{
			reply.printf("Speed factor override: %.1f%%\n", speedFactor * (60.0 * 100.0));
		}
		break;

	case 221:	// set extrusion factor override percentage
		{
			int drive;
			if (gb->Seen('D'))	// D parameter (if present) selects the extruder drive number
			{
				drive = gb->GetIValue();
			}
			else
			{
				drive = 0;		// default to drive 0 if not specified
			}

			if (gb->Seen('S'))	// S parameter sets the override percentage
			{
				float extrusionFactor = gb->GetFValue()/100.0;
				if (drive >= 0 && drive < DRIVES - AXES && extrusionFactor >= 0)
				{
					extrusionFactors[drive] = extrusionFactor;
				}
			}
			else
			{
				reply.printf("Extrusion factor override for drive %d: %.1f%%\n", drive, extrusionFactors[drive] * 100.0);

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
			reply.printf("Cold extrudes are %s, use M302 P[1/0] to allow or deny them",
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

	case 503: // List variable settings
		result = SendConfigToLine();
		break;

    case 540: // Set/report MAC address
    	if(gb->Seen('P'))
    	{
    	    SetMACAddress(gb);
    	}
		else
		{
			const byte* mac = platform->MACAddress();
			reply.printf("MAC: %x:%x:%x:%x:%x:%x ", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
		}
    	break;

	case 550: // Set/report machine name
		if (gb->Seen('P'))
		{
			reprap.GetWebserver()->SetName(gb->GetString());
		}
		else
		{
			reply.printf("RepRap name: %s ", reprap.GetWebserver()->GetName());
		}
		break;

	case 551: // Set password (no option to report it)
		if (gb->Seen('P'))
		{
			reprap.GetWebserver()->SetPassword(gb->GetString());
		}
		break;

	case 552: // Set/Get IP address
		if (gb->Seen('P'))
		{
			SetEthernetAddress(gb, code);
		}
		else
		{
			const byte *ip = platform->IPAddress();
			reply.printf("IP address: %d.%d.%d.%d\n ", ip[0], ip[1], ip[2], ip[3]);
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
			switch(platform->Emulating())
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
			reply.printf("Axis compensations - XY: %.5f, YZ: %.5f, ZX: %.5f ",
					reprap.GetMove()->AxisCompensation(X_AXIS),
					reprap.GetMove()->AxisCompensation(Y_AXIS),
					reprap.GetMove()->AxisCompensation(Z_AXIS));
		}
		break;

	case 557: // Set/report Z probe point coordinates
		if (gb->Seen('P'))
		{
			int point = gb->GetIValue();
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

			if(!seen)
			{
				reply.printf("Probe point %d - [%.1f, %.1f]", point,
						reprap.GetMove()->XBedProbePoint(point),
						reprap.GetMove()->YBedProbePoint(point));
			}
		}
		break;

    case 558: // Set or report Z probe type and for which axes it is used
		{
			bool seen = false;
			if(gb->Seen('P'))
			{
				platform->SetZProbeType(gb->GetIValue());
				seen = true;
			}

			bool zProbeAxes[AXES];
			platform->GetZProbeAxes(zProbeAxes);
			for(int axis=0; axis<AXES; axis++)
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
			else
			{
				reply.printf("Z Probe type is %d and it is used for these axes:", platform->GetZProbeType());
				for(int axis=0; axis<AXES; axis++)
				{
					if (zProbeAxes[axis])
					{
						reply.catf(" %c", axisLetters[axis]);
					}
				}
			}
		}
    	break;

	case 559: // Upload config.g or another gcode file to put in the sys directory
		{
			const char* str = (gb->Seen('P') ? gb->GetString() : platform->GetConfigFile());
			bool ok = OpenFileToWrite(platform->GetSysDir(), str, gb);
			if (ok)
			{
				reply.printf("Writing to file: %s", str);
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
				reply.printf("Writing to file: %s", str);
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
    	AddNewTool(gb, reply);
    	break;

    case 564: // Think outside the box?
    	if(gb->Seen('S'))
    	{
    	    limitAxes = (gb->GetIValue() != 0);
    	}
		break;

    case 566: // Set/print minimum feedrates
       	{
       		bool seen = false;
        	for(int8_t axis = 0; axis < AXES; axis++)
        	{
        		if(gb->Seen(axisLetters[axis]))
        		{
        			platform->SetInstantDv(axis, gb->GetFValue()*distanceScale * secondsToMinutes); // G Code feedrates are in mm/minute; we need mm/sec
        			seen = true;
        		}
        	}

        	if(gb->Seen(EXTRUDE_LETTER))
        	{
        		seen = true;
        		float eVals[DRIVES-AXES];
        		int eCount = DRIVES-AXES;
        		gb->GetFloatArray(eVals, eCount);
        		if(eCount != DRIVES-AXES)
        		{
        			reply.printf("Setting feedrates - wrong number of E drives: %s\n", gb->Buffer());
        		}
        		else
        		{
        			for(int8_t e = 0; e < eCount; e++)
        			{
        				platform->SetInstantDv(AXES + e, eVals[e] * distanceScale * secondsToMinutes);
        			}
        		}
        	}
        	else if(!seen)
        	{
        		reply.printf("Minimum feedrates: X: %.1f, Y: %.1f, Z: %.1f, E: ",
        				platform->InstantDv(X_AXIS)/(distanceScale * secondsToMinutes), platform->InstantDv(Y_AXIS)/(distanceScale * secondsToMinutes),
        				platform->InstantDv(Z_AXIS)/(distanceScale * secondsToMinutes));
            	for(int8_t drive = AXES; drive < DRIVES; drive++)
            	{
            		reply.catf("%.1f", platform->InstantDv(drive)/(distanceScale * secondsToMinutes));
            		if(drive < DRIVES-1)
            		{
            			reply.cat(":");
            		}
            	}
        	}
       	}
    	break;

    case 569: // Set/report axis direction
		if(gb->Seen('P'))
		{
			int8_t drive = gb->GetIValue();
			if(gb->Seen('S'))
			{
				platform->SetDirectionValue(drive, gb->GetIValue());
			}
			else
			{
				reply.printf("A %d sends drive %d forwards.", (int)platform->GetDirectionValue(drive), drive);
			}
		}
		break;

    case 906: // Set/report Motor currents
		{
			bool seen = false;
			for(int8_t axis = 0; axis < AXES; axis++)
			{
				if(gb->Seen(axisLetters[axis]))
				{
					platform->SetMotorCurrent(axis, gb->GetFValue());
					seen = true;
				}
			}

			if(gb->Seen(EXTRUDE_LETTER))
			{
				float eVals[DRIVES-AXES];
				int eCount = DRIVES-AXES;
				gb->GetFloatArray(eVals, eCount);
				if(eCount != DRIVES-AXES)
				{
					reply.printf("Setting motor currents - wrong number of E drives: %s\n", gb->Buffer());
				}
				else
				{
					for(int8_t e = 0; e < eCount; e++)
					{
						platform->SetMotorCurrent(AXES + e, eVals[e]);
					}
				}
			}
			else if (!seen)
			{
				reply.printf("Axis currents (mA) - X:%.1f, Y:%.1f, Z:%.1f, E:",
						platform->MotorCurrent(X_AXIS), platform->MotorCurrent(Y_AXIS),
						platform->MotorCurrent(Z_AXIS));
				for(int8_t drive = AXES; drive < DRIVES; drive++)
				{
					reply.catf("%.1f", platform->MotorCurrent(drive));
					if(drive < DRIVES-1)
					{
						reply.cat(":");
					}
				}
			}
		}
		break;

	case 998:
		if (gb->Seen('P'))
		{
			reply.printf("%d", gb->GetIValue());
			resend = true;
		}
		break;

	case 999:
		result = DoDwellTime(0.5);		// wait half a second to allow the response to be sent back to the web server, otherwise it may retry
		if (result)
		{
			platform->SoftwareReset(SoftwareResetReason::user);			// doesn't return
		}
		break;

	default:
		error = true;
		reply.printf("invalid M Code: %s", gb->Buffer());
	}
	if (result)
	{
		HandleReply(error, gb == serialGCode, reply.Pointer(), 'M', code, resend);
	}
	return result;
}

bool GCodes::HandleTcode(GCodeBuffer* gb)
{
    int code = gb->GetIValue();
    code += gb->GetToolNumberAdjust();
	bool result = ChangeTool(code);
    if(result)
    {
    	HandleReply(false, gb == serialGCode, "", 'T', code, false);
    }
    return result;
}

// Return the amount of filament extruded
float GCodes::GetExtruderPosition(uint8_t extruder) const
{
	return (extruder < (DRIVES - AXES)) ? lastPos[extruder] : 0;
}
  
bool GCodes::ChangeTool(int newToolNumber)
{
    Tool* oldTool = reprap.GetCurrentTool();
    Tool* newTool = reprap.GetTool(newToolNumber);

    // If old and new are the same still follow the sequence -
    // The user may want the macros run.

    switch(toolChangeSequence)
    {
    case 0: // Pre-release sequence for the old tool (if any)
    	if(oldTool != NULL)
    	{
    		scratchString.printf("tfree%d.g", oldTool->Number());
    		if(DoFileCannedCycles(scratchString.Pointer()))
    		{
    			toolChangeSequence++;
    		}
    	}
    	else
    	{
    		toolChangeSequence++;
    	}
    	return false;

    case 1: // Release the old tool (if any)
    	if(oldTool != NULL)
    	{
    		reprap.StandbyTool(oldTool->Number());
    	}
    	toolChangeSequence++;
    	return false;

    case 2: // Run the pre-tool-change canned cycle for the new tool (if any)
    	if(newTool != NULL)
    	{
    		scratchString.printf("tpre%d.g", newToolNumber);
    		if(DoFileCannedCycles(scratchString.Pointer()))
    		{
    			toolChangeSequence++;
    		}
    	}
    	else
    	{
    		toolChangeSequence++;
    	}
    	return false;

    case 3: // Select the new tool (even if it doesn't exist - that just deselects all tools)
    	reprap.SelectTool(newToolNumber);
   		toolChangeSequence++;
    	return false;

    case 4: // Run the post-tool-change canned cycle for the new tool (if any)
    	if(newTool != NULL)
    	{
    		scratchString.printf("tpost%d.g", newToolNumber);
    		if(DoFileCannedCycles(scratchString.Pointer()))
    		{
    			toolChangeSequence++;
    		}
    	}
    	else
    	{
    		toolChangeSequence++;
    	}
    	return false;

    case 5: // All done
    	toolChangeSequence = 0;
    	return true;

    default:
    	platform->Message(HOST_MESSAGE, "Tool change - dud sequence number.\n");
    }

    toolChangeSequence = 0;
    return true;
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

//*************************************************************************************

// This class stores a single G Code and provides functions to allow it to be parsed

GCodeBuffer::GCodeBuffer(Platform* p, const char* id)
{
	platform = p;
	identity = id;
	writingFileDirectory = NULL; // Has to be done here as Init() is called every line.
	toolNumberAdjust = 0;
}

void GCodeBuffer::Init()
{
	gcodePointer = 0;
	readPointer = -1;
	inComment = false;
	state = idle;
}

int GCodeBuffer::CheckSum()
{
	int cs = 0;
	for (int i = 0; gcodeBuffer[i] != '*' && gcodeBuffer[i] != 0; i++)
	{
		cs = cs ^ gcodeBuffer[i];
	}
	cs &= 0xff;  // Defensive programming...
	return cs;
}

// Add a byte to the code being assembled.  If false is returned, the code is
// not yet complete.  If true, it is complete and ready to be acted upon.

bool GCodeBuffer::Put(char c)
{
	bool result = false;
	gcodeBuffer[gcodePointer] = c;

	if (c == ';')
	{
		inComment = true;
	}

	if (c == '\n' || !c)
	{
		gcodeBuffer[gcodePointer] = 0;
		Init();
		if (reprap.Debug() && gcodeBuffer[0] && !writingFileDirectory) // Don't bother with blank/comment lines
		{
			platform->Message(HOST_MESSAGE, identity);
			platform->Message(HOST_MESSAGE, gcodeBuffer);
			platform->Message(HOST_MESSAGE, "\n");
		}

		// Deal with line numbers and checksums

		if (Seen('*'))
		{
			int csSent = GetIValue();
			int csHere = CheckSum();
			Seen('N');
			if (csSent != csHere)
			{
				snprintf(gcodeBuffer, GCODE_LENGTH, "M998 P%d", GetIValue());
				Init();
				result = true;
				return result;
			}

			// Strip out the line number and checksum

			while (gcodeBuffer[gcodePointer] != ' ' && gcodeBuffer[gcodePointer])
			{
				gcodePointer++;
			}

			// Anything there?

			if (!gcodeBuffer[gcodePointer])
			{
				// No...
				gcodeBuffer[0] = 0;
				Init();
				result = true;
				return result;
			}

			// Yes...

			gcodePointer++;
			int gp2 = 0;
			while (gcodeBuffer[gcodePointer] != '*' && gcodeBuffer[gcodePointer])
			{
				gcodeBuffer[gp2] = gcodeBuffer[gcodePointer++];
				gp2++;
			}
			gcodeBuffer[gp2] = 0;
			Init();
		}

		result = true;
	}
	else
	{
		if (!inComment || writingFileDirectory)
		{
			gcodePointer++;
		}
	}

	if (gcodePointer >= GCODE_LENGTH)
	{
		platform->Message(HOST_MESSAGE, "G Code buffer length overflow.\n");
		gcodePointer = 0;
		gcodeBuffer[0] = 0;
	}

	return result;
}

// Is 'c' in the G Code string?
// Leave the pointer there for a subsequent read.

bool GCodeBuffer::Seen(char c)
{
	readPointer = 0;
	for (;;)
	{
		char b = gcodeBuffer[readPointer];
		if (b == 0 || b == ';') break;
		if (b == c) return true;
		++readPointer;
	}
	readPointer = -1;
	return false;
}

// Get a float after a G Code letter found by a call to Seen()

float GCodeBuffer::GetFValue()
{
	if (readPointer < 0)
	{
		platform->Message(HOST_MESSAGE, "GCodes: Attempt to read a GCode float before a search.\n");
		readPointer = -1;
		return 0.0;
	}
	float result = (float) strtod(&gcodeBuffer[readPointer + 1], 0);
	readPointer = -1;
	return result;
}

// Get a :-separated list of floats after a key letter

const void GCodeBuffer::GetFloatArray(float a[], int& returnedLength)
{
	int length = 0;
	if(readPointer < 0)
	{
		platform->Message(HOST_MESSAGE, "GCodes: Attempt to read a GCode float array before a search.\n");
		readPointer = -1;
		returnedLength = 0;
		return;
	}

	bool inList = true;
	while(inList)
	{
		if(length >= returnedLength) // Array limit has been set in here
		{
			scratchString.printf("GCodes: Attempt to read a GCode float array that is too long: %s\n", gcodeBuffer);
			platform->Message(HOST_MESSAGE, scratchString);
			readPointer = -1;
			returnedLength = 0;
			return;
		}
		a[length] = (float)strtod(&gcodeBuffer[readPointer + 1], 0);
		length++;
		readPointer++;
		while(gcodeBuffer[readPointer] && (gcodeBuffer[readPointer] != ' ') && (gcodeBuffer[readPointer] != LIST_SEPARATOR))
		{
			readPointer++;
		}
		if(gcodeBuffer[readPointer] != LIST_SEPARATOR)
		{
			inList = false;
		}
	}

	// Special case if there is one entry and returnedLength requests several.
	// Fill the array with the first entry.

	if(length == 1 && returnedLength > 1)
	{
		for(int8_t i = 1; i < returnedLength; i++)
		{
			a[i] = a[0];
		}
	}
	else
	{
		returnedLength = length;
	}

	readPointer = -1;
}

// Get a :-separated list of longs after a key letter

const void GCodeBuffer::GetLongArray(long l[], int& returnedLength)
{
	int length = 0;
	if(readPointer < 0)
	{
		platform->Message(HOST_MESSAGE, "GCodes: Attempt to read a GCode long array before a search.\n");
		readPointer = -1;
		return;
	}

	bool inList = true;
	while(inList)
	{
		if(length >= returnedLength) // Array limit has been set in here
		{
			scratchString.printf("GCodes: Attempt to read a GCode long array that is too long: %s\n", gcodeBuffer);
			platform->Message(HOST_MESSAGE, scratchString);
			readPointer = -1;
			returnedLength = 0;
			return;
		}
		l[length] = strtol(&gcodeBuffer[readPointer + 1], 0, 0);
		length++;
		readPointer++;
		while(gcodeBuffer[readPointer] && (gcodeBuffer[readPointer] != ' ') && (gcodeBuffer[readPointer] != LIST_SEPARATOR))
		{
			readPointer++;
		}
		if(gcodeBuffer[readPointer] != LIST_SEPARATOR)
		{
			inList = false;
		}
	}
	returnedLength = length;
	readPointer = -1;
}

// Get a string after a G Code letter found by a call to Seen().
// It will be the whole of the rest of the GCode string, so strings
// should always be the last parameter.

const char* GCodeBuffer::GetString()
{
	if (readPointer < 0)
	{
		platform->Message(HOST_MESSAGE, "GCodes: Attempt to read a GCode string before a search.\n");
		readPointer = -1;
		return "";
	}
	const char* result = &gcodeBuffer[readPointer + 1];
	readPointer = -1;
	return result;
}

// This returns a pointer to the end of the buffer where a
// string starts.  It assumes that an M or G search has
// been done followed by a GetIValue(), so readPointer will
// be -1.  It absorbs "M/Gnnn " (including the space) from the
// start and returns a pointer to the next location.

// This is provided for legacy use, in particular in the M23
// command that sets the name of a file to be printed.  In
// preference use GetString() which requires the string to have
// been preceded by a tag letter.

const char* GCodeBuffer::GetUnprecedentedString()
{
	readPointer = 0;
	while (gcodeBuffer[readPointer] && gcodeBuffer[readPointer] != ' ')
	{
		readPointer++;
	}

	if (!gcodeBuffer[readPointer])
	{
		platform->Message(HOST_MESSAGE, "GCodes: String expected but not seen.\n");
		readPointer = -1;
		return gcodeBuffer; // Good idea?
	}

	const char* result = &gcodeBuffer[readPointer + 1];
	readPointer = -1;
	return result;
}

// Get an long after a G Code letter

long GCodeBuffer::GetLValue()
{
	if (readPointer < 0)
	{
		platform->Message(HOST_MESSAGE, "GCodes: Attempt to read a GCode int before a search.\n");
		readPointer = -1;
		return 0;
	}
	long result = strtol(&gcodeBuffer[readPointer + 1], 0, 0);
	readPointer = -1;
	return result;
}

