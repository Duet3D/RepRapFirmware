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
   platform->Message(HOST_MESSAGE, "GCodes class exited.\n");
   active = false;
}

void GCodes::Init()
{
  webGCode->Init();
  fileGCode->Init();
  serialGCode->Init();
  cannedCycleGCode->Init();
  webGCode->SetFinished(true);
  fileGCode->SetFinished(true);
  serialGCode->SetFinished(true);
  cannedCycleGCode->SetFinished(true);
  moveAvailable = false;
  drivesRelative = true;
  axesRelative = false;
  checkEndStops = false;
  gCodeLetters = GCODE_LETTERS;
  distanceScale = 1.0;
  for(int8_t i = 0; i < DRIVES - AXES; i++)
    lastPos[i] = 0.0;
  fileBeingPrinted = NULL;
  fileToPrint = NULL;
  fileBeingWritten = NULL;
  configFile = NULL;
  doingCannedCycleFile = false;
  eofString = EOF_STRING;
  eofStringCounter = 0;
  eofStringLength = strlen(eofString);
  homeX = false;
  homeY = false;
  homeZ = false;
  homeAxisMoveCount = 0;
  offSetSet = false;
  dwellWaiting = false;
  stackPointer = 0;
  selectedHead = -1;
  gFeedRate = platform->MaxFeedrate(Z_AXIS); // Typically the slowest
  zProbesSet = false;
  probeCount = 0;
  cannedCycleMoveCount = 0;
  cannedCycleMoveQueued = false;
  active = true;
  longWait = platform->Time();
  dwellTime = longWait;
}

void GCodes::doFilePrint(GCodeBuffer* gb)
{
	char b;

	if(fileBeingPrinted != NULL)
	{
		if(fileBeingPrinted->Read(b))
		{
			if(gb->Put(b))
				gb->SetFinished(ActOnGcode(gb));
		} else
		{
			if(gb->Put('\n')) // In case there wasn't one ending the file
				gb->SetFinished(ActOnGcode(gb));
			fileBeingPrinted->Close();
			fileBeingPrinted = NULL;
		}
	}
}

void GCodes::Spin()
{
  if(!active)
    return;
    
  // Check each of the sources of G Codes (web, serial, and file) to
  // see if what they are doing has been done.  If it hasn't, return without
  // looking at anything else.
  //
  // Note the order establishes a priority: web first, then serial, and file
  // last.  If file weren't last, then the others would never get a look in when
  // a file was being printed.

  if(!webGCode->Finished())
  {
    webGCode->SetFinished(ActOnGcode(webGCode));
    platform->ClassReport("GCodes", longWait);
    return;
  }
  
  if(!serialGCode->Finished())
  {
    serialGCode->SetFinished(ActOnGcode(serialGCode));
    platform->ClassReport("GCodes", longWait);
    return;
  }  

  if(!fileGCode->Finished())
  {
    fileGCode->SetFinished(ActOnGcode(fileGCode));
    platform->ClassReport("GCodes", longWait);
    return;
  }

  // Now check if a G Code byte is available from each of the sources
  // in the same order for the same reason.

  if(webserver->GCodeAvailable())
  {
	  char b = webserver->ReadGCode();
	  if(webGCode->Put(b))
	  {
		  if(webGCode->WritingFileDirectory() != NULL)
			  WriteGCodeToFile(webGCode);
		  else
			  webGCode->SetFinished(ActOnGcode(webGCode));
	  }
	  platform->ClassReport("GCodes", longWait);
	  return;
  }
  
  // Now the serial interface.  First check the special case of our
  // uploading the reprap.htm file

  if(serialGCode->WritingFileDirectory() == platform->GetWebDir())
  {
	  if(platform->GetLine()->Status() & byteAvailable)
	  {
		  char b;
		  platform->GetLine()->Read(b);
		  WriteHTMLToFile(b, serialGCode);
	  }
  } else
  {
	  // Otherwise just deal in general with incoming bytes from the serial interface

	  if(platform->GetLine()->Status() & byteAvailable)
	  {
		  // Read several bytes instead of just one. This is mainly to speed up file uploading.
		  int8_t i = 0;
		  do
		  {
			  char b;
			  platform->GetLine()->Read(b);
			  if(serialGCode->Put(b))	// add char to buffer and test whether the gcode is complete
			  {
				  // we have a complete gcode
				  if(serialGCode->WritingFileDirectory() != NULL)
				  {
					  WriteGCodeToFile(serialGCode);
				  }
				  else
				  {
					  serialGCode->SetFinished(ActOnGcode(serialGCode));
				  }
				  break;	// stop after receiving a complete gcode in case we haven't finished processing it
			  }
			  ++i;
		  } while (i < 16 && (platform->GetLine()->Status() & byteAvailable));
		  platform->ClassReport("GCodes", longWait);
		  return;
	  }
  }

  doFilePrint(fileGCode);

  platform->ClassReport("GCodes", longWait);
}

void GCodes::Diagnostics() 
{
  platform->Message(HOST_MESSAGE, "GCodes Diagnostics:\n");
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
  
  if(!reprap.GetMove()->GetCurrentState(moveBuffer))
    return false;
  
  return true;  
}

// Save (some of) the state of the machine for recovery in the future.
// Call repeatedly till it returns true.

bool GCodes::Push()
{
  if(stackPointer >= STACK)
  {
    platform->Message(HOST_MESSAGE, "Push(): stack overflow!\n");
    return true;
  }
  
  if(!AllMovesAreFinishedAndMoveBufferIsLoaded())
    return false;
  
  drivesRelativeStack[stackPointer] = drivesRelative;
  axesRelativeStack[stackPointer] = axesRelative;
  feedrateStack[stackPointer] = gFeedRate; 
  fileStack[stackPointer] = fileBeingPrinted;
  stackPointer++;
  platform->PushMessageIndent();
  return true;
}

// Recover a saved state.  Call repeatedly till it returns true.

bool GCodes::Pop()
{
  if(stackPointer <= 0)
  {
    platform->Message(HOST_MESSAGE, "Pop(): stack underflow!\n");
    return true;  
  }
  
  if(!AllMovesAreFinishedAndMoveBufferIsLoaded())
    return false;
    
  stackPointer--;
  drivesRelative = drivesRelativeStack[stackPointer];
  axesRelative = axesRelativeStack[stackPointer];
  fileBeingPrinted = fileStack[stackPointer];
  platform->PopMessageIndent();
  // Remember for next time if we have just been switched
  // to absolute drive moves
  
  for(int8_t i = AXES; i < DRIVES; i++)
    lastPos[i - AXES] = moveBuffer[i];
  
  // Do a null move to set the correct feedrate
  
  gFeedRate = feedrateStack[stackPointer];
  moveBuffer[DRIVES] = gFeedRate;
  
  checkEndStops = false;
  moveAvailable = true;
  return true;
}

// Move expects all axis movements to be absolute, and all
// extruder drive moves to be relative.  This function serves that.

void GCodes::LoadMoveBufferFromGCode(GCodeBuffer *gb, bool doingG92)
{
	float absE;

	for(uint8_t i = 0; i < DRIVES; i++)
	{
	    if(i < AXES)
	    {
	      if(gb->Seen(gCodeLetters[i]))
	      {
	        if(!axesRelative || doingG92)
	        	moveBuffer[i] = gb->GetFValue()*distanceScale;
	        else
	        	moveBuffer[i] += gb->GetFValue()*distanceScale;
	      }
	    } else
	    {
	      if(gb->Seen(gCodeLetters[i]))
	      {
	        if(drivesRelative || doingG92)
	          moveBuffer[i] = gb->GetFValue()*distanceScale;
	        else
	        {
	          absE = gb->GetFValue()*distanceScale;
	          moveBuffer[i] = absE - lastPos[i - AXES];
	          lastPos[i - AXES] = absE;
	        }
	      }
	    }
	}

	// Deal with feedrate

	if(gb->Seen(gCodeLetters[DRIVES]))
	  gFeedRate = gb->GetFValue()*distanceScale*0.016666667; // G Code feedrates are in mm/minute; we need mm/sec

	moveBuffer[DRIVES] = gFeedRate;  // We always set it, as Move may have modified the last one.
}


// This function is called for a G Code that makes a move.
// If the Move class can't receive the move (i.e. things have to wait)
// this returns false, otherwise true.

bool GCodes::SetUpMove(GCodeBuffer *gb)
{
  // Last one gone yet?
  
  if(moveAvailable)
    return false;
    
  // Load the last position; If Move can't accept more, return false
  
  if(!reprap.GetMove()->GetCurrentState(moveBuffer))
    return false;
  
  LoadMoveBufferFromGCode(gb, false);
  
  checkEndStops = false;
  if(gb->Seen('S'))
  {
	  if(gb->GetIValue() == 1)
		  checkEndStops = true;
  }

  moveAvailable = true;
  return true; 
}

// The Move class calls this function to find what to do next.

bool GCodes::ReadMove(float m[], bool& ce)
{
    if(!moveAvailable)
      return false; 
    for(int8_t i = 0; i <= DRIVES; i++) // 1 more for feedrate
      m[i] = moveBuffer[i];
    ce = checkEndStops;
    moveAvailable = false;
    checkEndStops = false;
    return true;
}


bool GCodes::DoFileCannedCycles(char* fileName)
{
	// Have we started the file?

	if(!doingCannedCycleFile)
	{
		// No

		if(!Push())
			return false;

		fileBeingPrinted = platform->GetFileStore(platform->GetSysDir(), fileName, false);
		if(fileBeingPrinted == NULL)
		{
			platform->Message(HOST_MESSAGE, "Canned cycle GCode file not found - ");
			platform->Message(HOST_MESSAGE, fileName);
			platform->Message(HOST_MESSAGE, "\n");
			if(!Pop())
				platform->Message(HOST_MESSAGE, "Cannot pop the stack.\n");
			return true;
		}
		doingCannedCycleFile = true;
		cannedCycleGCode->Init();
		return false;
	}

	// Have we finished the file?

	if(fileBeingPrinted == NULL)
	{
		// Yes

		if(!Pop())
			return false;
		doingCannedCycleFile = false;
		cannedCycleGCode->Init();
		return true;
	}

	// No - Do more of the file

	if(!cannedCycleGCode->Finished())
	{
		cannedCycleGCode->SetFinished(ActOnGcode(cannedCycleGCode));
	    return false;
	}

	doFilePrint(cannedCycleGCode);

	return false;
}

bool GCodes::FileCannedCyclesReturn()
{
	if(!doingCannedCycleFile)
		return true;

	if(!AllMovesAreFinishedAndMoveBufferIsLoaded())
		return false;

	doingCannedCycleFile = false;
	cannedCycleGCode->Init();

	if(fileBeingPrinted != NULL)
		fileBeingPrinted->Close();

	fileBeingPrinted = NULL;
	return true;
}

// To execute any move, call this until it returns true.
// moveToDo[] entries corresponding with false entries in action[] will
// be ignored.  Recall that moveToDo[DRIVES] should contain the feedrate
// you want (if action[DRIVES] is true).

bool GCodes::DoCannedCycleMove(bool ce)
{
	// Is the move already running?

	if(cannedCycleMoveQueued)
	{ // Yes.
		if(!Pop()) // Wait for the move to finish then restore the state
			return false;
		cannedCycleMoveQueued = false;
		return true;
	} else
	{ // No.
		if(!Push()) // Wait for the RepRap to finish whatever it was doing, save it's state, and load moveBuffer[] with the current position.
			return false;
		for(int8_t drive = 0; drive <= DRIVES; drive++)
		{
			if(activeDrive[drive])
				moveBuffer[drive] = moveToDo[drive];
		}
		checkEndStops = ce;
		cannedCycleMoveQueued = true;
		moveAvailable = true;
	}
	return false;
}

// This sets positions.  I.e. it handles G92.

bool GCodes::SetPositions(GCodeBuffer *gb)
{
	if(!AllMovesAreFinishedAndMoveBufferIsLoaded())
		return false;

	LoadMoveBufferFromGCode(gb, true);
	reprap.GetMove()->SetLiveCoordinates(moveBuffer);
	reprap.GetMove()->SetPositions(moveBuffer);

	return true;
}

// Offset the axes by the X, Y, and Z amounts in the M code in gb.  Say the machine is at [10, 20, 30] and
// the offsets specified are [8, 2, -5].  The machine will move to [18, 22, 25] and henceforth consider that point
// to be [10, 20, 30].

bool GCodes::OffsetAxes(GCodeBuffer* gb)
{
	if(!offSetSet)
	{
		if(!AllMovesAreFinishedAndMoveBufferIsLoaded())
		    return false;
		for(int8_t drive = 0; drive <= DRIVES; drive++)
		{
			if(drive < AXES || drive == DRIVES)
			{
				record[drive] = moveBuffer[drive];
				moveToDo[drive] = moveBuffer[drive];
			} else
			{
				record[drive] = 0.0;
				moveToDo[drive] = 0.0;
			}
			activeDrive[drive] = false;
		}

		for(int8_t axis = 0; axis < AXES; axis++)
		{
			if(gb->Seen(gCodeLetters[axis]))
			{
				moveToDo[axis] += gb->GetFValue();
				activeDrive[axis] = true;
			}
		}

		if(gb->Seen(gCodeLetters[DRIVES])) // Has the user specified a feedrate?
		{
			moveToDo[DRIVES] = gb->GetFValue();
			activeDrive[DRIVES] = true;
		}

		offSetSet = true;
	}


	if(DoCannedCycleMove(false))
	{
		//LoadMoveBufferFromArray(record);
		for(int drive = 0; drive <= DRIVES; drive++)
			moveBuffer[drive] = record[drive];
		reprap.GetMove()->SetLiveCoordinates(record);  // This doesn't transform record
		reprap.GetMove()->SetPositions(record);        // This does
		offSetSet = false;
		return true;
	}

	return false;
}

// Home one or more of the axes.  Which ones are decided by the
// booleans homeX, homeY and homeZ.

bool GCodes::DoHome()
{
	if(homeX && homeY && homeZ)
	{
		if(DoFileCannedCycles(HOME_ALL_G))
		{
			homeAxisMoveCount = 0;
			homeX = false;
			homeY = false;
			homeZ = false;
			return true;
		}
		return false;
	}

	if(homeX)
	{
		if(DoFileCannedCycles(HOME_X_G))
		{
			homeAxisMoveCount = 0;
			homeX = false;
			return NoHome();
		}
		return false;
	}


	if(homeY)
	{
		if(DoFileCannedCycles(HOME_Y_G))
		{
			homeAxisMoveCount = 0;
			homeY = false;
			return NoHome();
		}
		return false;
	}


	if(homeZ)
	{
		if(DoFileCannedCycles(HOME_Z_G))
		{
			homeAxisMoveCount = 0;
			homeZ = false;
			return NoHome();
		}
		return false;
	}

	// Should never get here

	checkEndStops = false;
	moveAvailable = false;
	homeAxisMoveCount = 0;

	return true;
}

// This lifts Z a bit, moves to the probe XY coordinates (obtained by a call to GetProbeCoordinates() ),
// probes the bed height, and records the Z coordinate probed.  If you want to program any general
// internal canned cycle, this shows how to do it.

bool GCodes::DoSingleZProbeAtPoint()
{
	float x, y, z;

	reprap.GetMove()->SetIdentityTransform();  // It doesn't matter if these are called repeatedly

	for(int8_t drive = 0; drive <= DRIVES; drive++)
		activeDrive[drive] = false;

	switch(cannedCycleMoveCount)
	{
	case 0:  // This only does anything on the first move; on all the others Z is already there
		moveToDo[Z_AXIS] = Z_DIVE;
		activeDrive[Z_AXIS] = true;
		moveToDo[DRIVES] = platform->HomeFeedRate(Z_AXIS);
		activeDrive[DRIVES] = true;
		reprap.GetMove()->SetZProbing(false);
		if(DoCannedCycleMove(false))
			cannedCycleMoveCount++;
		return false;

	case 1:
		GetProbeCoordinates(probeCount, moveToDo[X_AXIS], moveToDo[Y_AXIS], moveToDo[Z_AXIS]);
		activeDrive[X_AXIS] = true;
		activeDrive[Y_AXIS] = true;
		// NB - we don't use the Z value
		moveToDo[DRIVES] = platform->HomeFeedRate(X_AXIS);
		activeDrive[DRIVES] = true;
		reprap.GetMove()->SetZProbing(false);
		if(DoCannedCycleMove(false))
			cannedCycleMoveCount++;
		return false;

	case 2:
		moveToDo[Z_AXIS] = -2.0*platform->AxisLength(Z_AXIS);
		activeDrive[Z_AXIS] = true;
		moveToDo[DRIVES] = platform->HomeFeedRate(Z_AXIS);
		activeDrive[DRIVES] = true;
		reprap.GetMove()->SetZProbing(true);
		if(DoCannedCycleMove(true))
			cannedCycleMoveCount++;
		return false;

	case 3:
		moveToDo[Z_AXIS] = Z_DIVE;
		activeDrive[Z_AXIS] = true;
		moveToDo[DRIVES] = platform->HomeFeedRate(Z_AXIS);
		activeDrive[DRIVES] = true;
		reprap.GetMove()->SetZProbing(false);
		if(DoCannedCycleMove(false))
			cannedCycleMoveCount++;
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
	if(!AllMovesAreFinishedAndMoveBufferIsLoaded())
			return false;

	for(int8_t drive = 0; drive <= DRIVES; drive++)
		activeDrive[drive] = false;

	moveToDo[Z_AXIS] = -1.1*platform->AxisLength(Z_AXIS);
	activeDrive[Z_AXIS] = true;
	moveToDo[DRIVES] = platform->HomeFeedRate(Z_AXIS);
	activeDrive[DRIVES] = true;
	if(DoCannedCycleMove(true))
	{
		cannedCycleMoveCount = 0;
		probeCount = 0;
		return true;
	}
	return false;
}

// This sets wherever we are as the probe point P (probePointIndex)
// then probes the bed, or gets all its parameters from the arguments.
// If X or Y are specified, use those; otherwise use the machine's
// coordinates.  If no Z is specified use the machine's coordinates.  If it
// is specified and is greater than SILLY_Z_VALUE (i.e. greater than -9999.0)
// then that value is used.  If it's less than SILLY_Z_VALUE the bed is
// probed and that value is used.

bool GCodes::SetSingleZProbeAtAPosition(GCodeBuffer *gb)
{
	if(!AllMovesAreFinishedAndMoveBufferIsLoaded())
		return false;

	if(!gb->Seen('P'))
		return DoSingleZProbe();

	int probePointIndex = gb->GetIValue();

	float x, y, z;
	if(gb->Seen(gCodeLetters[X_AXIS]))
		x = gb->GetFValue();
	else
		x = moveBuffer[X_AXIS];
	if(gb->Seen(gCodeLetters[Y_AXIS]))
		y = gb->GetFValue();
	else
		y = moveBuffer[Y_AXIS];
	if(gb->Seen(gCodeLetters[Z_AXIS]))
		z = gb->GetFValue();
	else
		z = moveBuffer[Z_AXIS];

	probeCount = probePointIndex;
	reprap.GetMove()->SetXBedProbePoint(probeCount, x);
	reprap.GetMove()->SetYBedProbePoint(probeCount, y);

	if(z > SILLY_Z_VALUE)
	{
		reprap.GetMove()->SetZBedProbePoint(probeCount, z);
		reprap.GetMove()->SetZProbing(false); // Not really needed, but let's be safe
		probeCount = 0;
		if(gb->Seen('S'))
		{
			zProbesSet = true;
			reprap.GetMove()->SetProbedBedEquation();
		}
		return true;
	} else
	{
		if(DoSingleZProbeAtPoint())
		{
			probeCount = 0;
			reprap.GetMove()->SetZProbing(false);
			if(gb->Seen('S'))
			{
				zProbesSet = true;
				reprap.GetMove()->SetProbedBedEquation();
			}
			return true;
		}
	}

	return false;
}

// This probes multiple points on the bed (three in a
// triangle or four in the corners), then sets the bed transformation to compensate
// for the bed not quite being the plane Z = 0.

bool GCodes::DoMultipleZProbe()
{
	if(reprap.GetMove()->NumberOfXYProbePoints() < 3)
	{
		platform->Message(HOST_MESSAGE, "Bed probing: there needs to be 3 or more points set.\n");
		return true;
	}

	if(DoSingleZProbeAtPoint())
		probeCount++;
	if(probeCount >= reprap.GetMove()->NumberOfXYProbePoints())
	{
		probeCount = 0;
		zProbesSet = true;
		reprap.GetMove()->SetZProbing(false);
		reprap.GetMove()->SetProbedBedEquation();
		return true;
	}
	return false;
}

// This returns the (X, Y) points to probe the bed at probe point count.  When probing,
// it returns false.  If called after probing has ended it returns true, and the Z coordinate
// probed is also returned.

bool GCodes::GetProbeCoordinates(int count, float& x, float& y, float& z)
{
	x = reprap.GetMove()->xBedProbePoint(count);
	y = reprap.GetMove()->yBedProbePoint(count);
	z = reprap.GetMove()->zBedProbePoint(count);
	return zProbesSet;
}

bool GCodes::SetPrintZProbe(GCodeBuffer* gb, char* reply)
{
	if(!AllMovesAreFinishedAndMoveBufferIsLoaded())
		return false;
	if(gb->Seen(gCodeLetters[Z_AXIS]))
	{
		platform->SetZProbeStopHeight(gb->GetFValue());
		if(gb->Seen('P'))
		{
			platform->SetZProbe(gb->GetIValue());
		}
	} else
		snprintf(reply, STRING_LENGTH, "%d", platform->ZProbe());
	return true;
}

// Return the current coordinates as a printable string.  Coordinates
// are updated at the end of each movement, so this won't tell you
// where you are mid-movement.

// FIXME - needs to deal with multiple extruders

char* GCodes::GetCurrentCoordinates()
{
	float liveCoordinates[DRIVES+1];
	reprap.GetMove()->LiveCoordinates(liveCoordinates);

	snprintf(scratchString, STRING_LENGTH, "X:%f Y:%f Z:%f E:%f", liveCoordinates[X_AXIS], liveCoordinates[Y_AXIS], liveCoordinates[Z_AXIS], liveCoordinates[AXES]);
	return scratchString;
}

char* GCodes::OpenFileToWrite(char* directory, char* fileName, GCodeBuffer *gb)
{
	fileBeingWritten = platform->GetFileStore(directory, fileName, true);
	if(fileBeingWritten == NULL)
		  platform->Message(HOST_MESSAGE, "Can't open GCode file for writing.\n");
	else
		gb->SetWritingFileDirectory(directory);

	eofStringCounter = 0;
}

void GCodes::WriteHTMLToFile(char b, GCodeBuffer *gb)
{
	char reply[1];
	reply[0] = 0;

	if(fileBeingWritten == NULL)
	{
		platform->Message(HOST_MESSAGE, "Attempt to write to a null file.\n");
		return;
	}

	fileBeingWritten->Write(b);

	if(b == eofString[eofStringCounter])
	{
		eofStringCounter++;
		if(eofStringCounter >= eofStringLength)
		{
			fileBeingWritten->Close();
			fileBeingWritten = NULL;
			gb->SetWritingFileDirectory(NULL);
			char* r = reply;
			if(platform->Emulating() == marlin)
				r = "Done saving file.";
			HandleReply(false, gb == serialGCode , r, 'M', 560, false);
			return;
		}
	} else
		eofStringCounter = 0;
}

void GCodes::WriteGCodeToFile(GCodeBuffer *gb)
{
	char reply[1];
	reply[0] = 0;

	if(fileBeingWritten == NULL)
	{
		platform->Message(HOST_MESSAGE, "Attempt to write to a null file.\n");
		return;
	}

	// End of file?

	if(gb->Seen('M'))
	{
		if(gb->GetIValue() == 29)
		{
			fileBeingWritten->Close();
			fileBeingWritten = NULL;
			gb->SetWritingFileDirectory(NULL);
			char* r = reply;
			if(platform->Emulating() == marlin)
				r = "Done saving file.";
			HandleReply(false, gb == serialGCode , r, 'M', 29, false);
			return;
		}
	}

	// Resend request?

	if(gb->Seen('G'))
	{
		if(gb->GetIValue() == 998)
		{
			if(gb->Seen('P'))
			{
				snprintf(scratchString, STRING_LENGTH, "%s", gb->GetIValue());
				HandleReply(false, gb == serialGCode , scratchString, 'G', 998, true);
				return;
			}
		}
	}

	fileBeingWritten->Write(gb->Buffer());
	fileBeingWritten->Write('\n');
	HandleReply(false, gb == serialGCode , reply, 'G', 1, false);
}

// Set up a file to print, but don't print it yet.

void GCodes::QueueFileToPrint(char* fileName)
{
  fileToPrint = platform->GetFileStore(platform->GetGCodeDir(), fileName, false);
  if(fileToPrint == NULL)
	  platform->Message(HOST_MESSAGE, "GCode file not found\n");
}


bool GCodes::SendConfigToLine()
{
	if(configFile == NULL)
	{
		configFile = platform->GetFileStore(platform->GetSysDir(), platform->GetConfigFile(), false);
		if(configFile == NULL)
		{
			platform->Message(HOST_MESSAGE, "Configuration file not found\n");
			return true;
		}
		platform->GetLine()->Write('\n');
	}

	char b;

	while(configFile->Read(b))
	{
		platform->GetLine()->Write(b);
		if(b == '\n')
			return false;
	}

	platform->GetLine()->Write('\n');
	configFile->Close();
	configFile = NULL;
	return true;
}


// Function to handle dwell delays.  Return true for
// dwell finished, false otherwise.

bool GCodes::DoDwell(GCodeBuffer *gb)
{
  float dwell;
  
  if(gb->Seen('P'))
    dwell = 0.001*(float)gb->GetLValue(); // P values are in milliseconds; we need seconds
  else
    return true;  // No time given - throw it away
      
  // Wait for all the queued moves to stop
      
  if(!reprap.GetMove()->AllMovesAreFinished())
    return false;
      
  // Are we already in the dwell?
      
  if(dwellWaiting)
  {
    if(platform->Time() - dwellTime >= 0.0)
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

// Set distance offsets and working and standby temperatures for
// an extruder.  I.e. handle a G10.

bool GCodes::SetOffsets(GCodeBuffer *gb)
{
  int8_t head;
  if(gb->Seen('P'))
  {
    head = gb->GetIValue() + 1;  // 0 is the Bed
    if(gb->Seen('R'))
      reprap.GetHeat()->SetStandbyTemperature(head, gb->GetFValue());
      
    if(gb->Seen('S'))
      reprap.GetHeat()->SetActiveTemperature(head, gb->GetFValue());
    // FIXME - do X, Y and Z
  }
  return true;  
}

// Does what it says.

bool GCodes::DisableDrives()
{
	if(!AllMovesAreFinishedAndMoveBufferIsLoaded())
	   return false;
	for(int8_t drive = 0; drive < DRIVES; drive++)
		platform->Disable(drive);
	return true;
}

// Does what it says.

bool GCodes::StandbyHeaters()
{
	if(!AllMovesAreFinishedAndMoveBufferIsLoaded())
		return false;
	for(int8_t heater = 0; heater < HEATERS; heater++)
		reprap.GetHeat()->Standby(heater);
	selectedHead = -1;
	return true;
}

void GCodes::SetEthernetAddress(GCodeBuffer *gb, int mCode)
{
	byte eth[4];
	char* ipString = gb->GetString();
	uint8_t sp = 0;
	uint8_t spp = 0;
	uint8_t ipp = 0;
	while(ipString[sp])
	{
		if(ipString[sp] == '.')
		{
			ipString[sp] = 0;
			eth[ipp] = atoi(&ipString[spp]);
			ipString[sp] = '.';
			ipp++;
			if(ipp > 3)
			{
				platform->Message(HOST_MESSAGE, "Dud IP address: ");
				platform->Message(HOST_MESSAGE, gb->Buffer());
				platform->Message(HOST_MESSAGE, "\n");
				return;
			}
			sp++;
			spp = sp;
		}else
			sp++;
	}
	eth[ipp] = atoi(&ipString[spp]);
	if(ipp == 3)
	{
		switch(mCode)
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
	} else
	{
		platform->Message(HOST_MESSAGE, "Dud IP address: ");
		platform->Message(HOST_MESSAGE, gb->Buffer());
		platform->Message(HOST_MESSAGE, "\n");
	}
}

void GCodes::HandleReply(bool error, bool fromLine, char* reply, char gMOrT, int code, bool resend)
{
	Compatibility c = platform->Emulating();
	if(!fromLine)
		c = me;

	char* response = "ok";
	if(resend)
		response = "rs ";

	char* s = 0;

	switch(c)
	{
	case me:
	case reprapFirmware:
		if(!reply[0])
			return;
		if(error)
			platform->GetLine()->Write("Error: ");
		platform->GetLine()->Write(reply);
		platform->GetLine()->Write("\n");
		return;

	case marlin:

		if(gMOrT == 'M' && code == 20)
		{
			platform->GetLine()->Write("Begin file list\n");
			platform->GetLine()->Write(reply);
			platform->GetLine()->Write("\nEnd file list\n");
			platform->GetLine()->Write(response);
			platform->GetLine()->Write("\n");
			return;
		}

		if(gMOrT == 'M' && code == 28)
		{
			platform->GetLine()->Write(response);
			platform->GetLine()->Write("\n");
			platform->GetLine()->Write(reply);
			platform->GetLine()->Write("\n");
			return;
		}

		if( (gMOrT == 'M' && code == 105) || (gMOrT == 'G' && code == 998))
		{
			platform->GetLine()->Write(response);
			platform->GetLine()->Write(" ");
			platform->GetLine()->Write(reply);
			platform->GetLine()->Write("\n");
			return;
		}

		if(reply[0])
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

	if(s != 0)
	{
		snprintf(scratchString, STRING_LENGTH, "Emulation of %s is not yet supported.\n", s);
		platform->Message(HOST_MESSAGE, scratchString);
	}

}

// If the GCode to act on is completed, this returns true,
// otherwise false.  It is called repeatedly for a given
// GCode until it returns true for that code.

bool GCodes::ActOnGcode(GCodeBuffer *gb)
{
  int code;
  float value;
  int iValue;
  char* str;
  bool result = true;
  bool error = false;
  bool resend = false;
  bool seen;
  char reply[STRING_LENGTH];

  reply[0] = 0;

  if(gb->Seen('G'))
  {
    code = gb->GetIValue();
    switch(code)
    {
    case 0: // There are no rapid moves...
    case 1: // Ordinary move
      result = SetUpMove(gb);
      break;
      
    case 4: // Dwell
      result = DoDwell(gb);
      break;
      
    case 10: // Set offsets
      result = SetOffsets(gb);
      break;
    
    case 20: // Inches (which century are we living in, here?)
      distanceScale = INCH_TO_MM;
      break;
    
    case 21: // mm
      distanceScale = 1.0;
      break;
    
    case 28: // Home
      if(NoHome())
      {
    	homeAxisMoveCount = 0;
        homeX = gb->Seen(gCodeLetters[X_AXIS]);
        homeY = gb->Seen(gCodeLetters[Y_AXIS]);
        homeZ = gb->Seen(gCodeLetters[Z_AXIS]);
        if(NoHome())
        {
          homeX = true;
          homeY = true;
          homeZ = true;
        }
      }
      result = DoHome();
      break;

    case 30: // Z probe/manually set at a position and set that as point P
    	result = SetSingleZProbeAtAPosition(gb);
    	break;

    case 31: // Return the probe value, or set probe variables
    	result = SetPrintZProbe(gb, reply);
    	break;

    case 32: // Probe Z at multiple positions and generate the bed transform
    	result = DoMultipleZProbe();
    	break;

    case 90: // Absolute coordinates
      drivesRelative = false;
      axesRelative = false;
      break;
      
    case 91: // Relative coordinates
      drivesRelative = true; // Non-axis movements (i.e. extruders)
      axesRelative = true;   // Axis movements (i.e. X, Y and Z)
      break;
      
    case 92: // Set position
      result = SetPositions(gb);
      break;
      
    default:
    	error = true;
    	snprintf(reply, STRING_LENGTH, "invalid G Code: %s", gb->Buffer());
    }
    if(result)
    	HandleReply(error, gb == serialGCode, reply, 'G', code, resend);
    return result;
  }
  
  if(gb->Seen('M'))
  {
    code = gb->GetIValue();
    switch(code)
    {
    case 0: // Stop
    case 1: // Sleep
      if(fileBeingPrinted != NULL)
      {
    	  fileToPrint = fileBeingPrinted;
    	  fileBeingPrinted = NULL;
      }
      if(!DisableDrives())
    	  return false;
      if(!StandbyHeaters())
    	  return false; // Should never happen
      break;
    
    case 18: // Motors off
      result = DisableDrives();
      break;
      
    case 20:  // Deprecated...
      if(platform->Emulating() == me || platform->Emulating() == reprapFirmware)
    	  snprintf(reply, STRING_LENGTH, "GCode files:\n%s", platform->GetMassStorage()->FileList(platform->GetGCodeDir(), gb == serialGCode));
      else
    	  snprintf(reply, STRING_LENGTH, "%s", platform->GetMassStorage()->FileList(platform->GetGCodeDir(), gb == serialGCode));
      break;

    case 21: // Initialise SD - ignore
    	break;

    case 23: // Set file to print
      QueueFileToPrint(gb->GetUnprecedentedString());
      if(platform->Emulating() == marlin)
    	  snprintf(reply, STRING_LENGTH, "%s", "File opened\nFile selected\n");
      break;
      
    case 24: // Print/resume-printing the selected file
      if(fileBeingPrinted != NULL)
    	  break;
      fileBeingPrinted = fileToPrint;
      fileToPrint = NULL;
      break;
      
    case 25: // Pause the print
    	fileToPrint = fileBeingPrinted;
    	fileBeingPrinted = NULL;
    	break;

    case 27: // Report print status - Depricated
    	if(this->PrintingAFile())
    		strncpy(reply, "SD printing.", STRING_LENGTH);
    	else
    		strncpy(reply, "Not SD printing.", STRING_LENGTH);
    	break;

    case 28: // Write to file
    	str = gb->GetUnprecedentedString();
    	OpenFileToWrite(platform->GetGCodeDir(), str, gb);
    	snprintf(reply, STRING_LENGTH, "Writing to file: %s", str);
    	break;

    case 29: // End of file being written; should be intercepted before getting here
    	platform->Message(HOST_MESSAGE, "GCode end-of-file being interpreted.\n");
    	break;

    case 82:
    	for(int8_t extruder = AXES; extruder < DRIVES; extruder++)
    		lastPos[extruder - AXES] = 0.0;
    	drivesRelative = false;
    	break;

    case 83:
    	for(int8_t extruder = AXES; extruder < DRIVES; extruder++)
    		lastPos[extruder - AXES] = 0.0;
    	drivesRelative = true;

    	break;

    case 84: // Motors off - deprecated, use M18
        result = DisableDrives();
        break;

    case 85: // Set inactive time
    	break;

    case 92: // Set/report steps/mm for some axes
    	seen = false;
    	for(int8_t drive = 0; drive < DRIVES; drive++)
    		if(gb->Seen(gCodeLetters[drive]))
    		{
    			platform->SetDriveStepsPerUnit(drive, gb->GetFValue());
    			seen = true;
    		}
    	reprap.GetMove()->SetStepHypotenuse();
    	if(!seen)
    		snprintf(reply, STRING_LENGTH, "Steps/mm: X: %d, Y: %d, Z: %d, E: %d",
    				(int)platform->DriveStepsPerUnit(X_AXIS), (int)platform->DriveStepsPerUnit(Y_AXIS),
    				(int)platform->DriveStepsPerUnit(Z_AXIS), (int)platform->DriveStepsPerUnit(AXES)); // FIXME - needs to do multiple extruders
        break;


    case 98:
    	if(gb->Seen('P'))
    		result = DoFileCannedCycles(gb->GetString());
    	break;

    case 99:
    	result = FileCannedCyclesReturn();
    	break;

    case 104: // Depricated
    	if(gb->Seen('S'))
    	{
    		reprap.GetHeat()->SetActiveTemperature(1, gb->GetFValue()); // 0 is the bed
    		reprap.GetHeat()->Activate(1);
    	}
    	break;

    case 105: // Deprecated...
    	strncpy(reply, "T:", STRING_LENGTH);
    	for(int8_t heater = HEATERS - 1; heater > 0; heater--)
    	{
    		strncat(reply, ftoa(0, reprap.GetHeat()->GetTemperature(heater), 1), STRING_LENGTH);
    		strncat(reply, " ", STRING_LENGTH);
    	}
    	strncat(reply, "B:", STRING_LENGTH);
    	strncat(reply, ftoa(0, reprap.GetHeat()->GetTemperature(0), 1), STRING_LENGTH);
    	break;
   
    case 106: // Fan on or off
    	if(gb->Seen('S'))
    		platform->CoolingFan(gb->GetFValue());
      break;
    
    case 107: // Fan off - depricated
    	platform->CoolingFan(0.0);
      break;
      
    case 110: // Set line numbers - line numbers are dealt with in the GCodeBuffer class
    	break;

    case 111: // Debug level
    	if(gb->Seen('S'))
    		reprap.SetDebug(gb->GetIValue());
    	break;

    case 112: // Emergency stop - acted upon in Webserver
    	break;

    case 114: // Deprecated
    	str = GetCurrentCoordinates();
    	if(str != 0)
    	{
    		strncpy(reply, str, STRING_LENGTH);
    	} else
    		result = false;
    	break;

    case 115: // Print firmware version
    	snprintf(reply, STRING_LENGTH, "FIRMWARE_NAME:%s FIRMWARE_VERSION:%s ELECTRONICS:%s DATE:%s", NAME, VERSION, ELECTRONICS, DATE);
    	break;

    case 109: // Depricated
    	if(gb->Seen('S'))
    	{
    		reprap.GetHeat()->SetActiveTemperature(1, gb->GetFValue()); // 0 is the bed
    		reprap.GetHeat()->Activate(1);
    	}
    case 116: // Wait for everything, especially set temperatures
    	if(!AllMovesAreFinishedAndMoveBufferIsLoaded())
    		return false;
    	result = reprap.GetHeat()->AllHeatersAtSetTemperatures();
    	break;

    case 120:
    	result = Push();
    	break;

    case 121:
      result = Pop();
      break;
    
    case 122:
      reprap.Diagnostics();
      break;
      
    case 126: // Valve open
      platform->Message(HOST_MESSAGE, "M126 - valves not yet implemented\n");
      break;
      
    case 127: // Valve closed
      platform->Message(HOST_MESSAGE, "M127 - valves not yet implemented\n");
      break;
      
    case 135: // Set PID sample interval
    	break;

    case 140: // Set bed temperature
      if(gb->Seen('S'))
      {
        reprap.GetHeat()->SetActiveTemperature(0, gb->GetFValue());
        reprap.GetHeat()->Activate(0);
      }
      break;
    
    case 141: // Chamber temperature
      platform->Message(HOST_MESSAGE, "M141 - heated chamber not yet implemented\n");
      break;

    case 201: // Set axis accelerations
    	for(int8_t drive = 0; drive < DRIVES; drive++)
    	{
    		if(gb->Seen(gCodeLetters[drive]))
    		{
    			value = gb->GetFValue();
    		}else{
    			value = -1;
    		}
    		platform->SetAcceleration(drive, value);
    	}
    	break;

    case 203: // Set maximum feedrates
    	for(int8_t drive = 0; drive < DRIVES; drive++)
    	{
    		if(gb->Seen(gCodeLetters[drive]))
    		{
    			value = gb->GetFValue()*distanceScale*0.016666667; // G Code feedrates are in mm/minute; we need mm/sec;
    			platform->SetMaxFeedrate(drive, value);
    		}
    	}
    	break;

    case 205:  //M205 advanced settings:  minimum travel speed S=while printing T=travel only,  B=minimum segment time X= maximum xy jerk, Z=maximum Z jerk
    	break;

    case 206:  // Offset axes
    	result = OffsetAxes(gb);
    	break;

    case 208: // Set maximum axis lengths
    	for(int8_t axis = 0; axis < AXES; axis++)
    	{
    		if(gb->Seen(gCodeLetters[axis]))
    		{
    			value = gb->GetFValue()*distanceScale;
    			platform->SetAxisLength(axis, value);
    		}
    	}
    	break;

    case 210: // Set homing feedrates
    	for(int8_t axis = 0; axis < AXES; axis++)
    	{
    		if(gb->Seen(gCodeLetters[axis]))
    		{
    			value = gb->GetFValue()*distanceScale*0.016666667;
    			platform->SetHomeFeedRate(axis, value);
    		}
    	}
    	break;

    case 301: // Set PID values
    	break;

    case 302: // Allow cold extrudes
    	break;

    case 304: // Set thermistor parameters
    	break;

    case 503: // list variable settings
    	result = SendConfigToLine();
    	break;

    case 550: // Set machine name
        	if(gb->Seen('P'))
        		reprap.GetWebserver()->SetName(gb->GetString());
        	break;

    case 551: // Set password
    	if(gb->Seen('P'))
    		reprap.GetWebserver()->SetPassword(gb->GetString());
    	break;

    case 552: // Set/Get IP address
    	if(gb->Seen('P'))
    		SetEthernetAddress(gb, code);
    	else
    	{
    		byte *ip = platform->IPAddress();
    		snprintf(reply, STRING_LENGTH, "IP address: %d.%d.%d.%d\n ", ip[0], ip[1], ip[2], ip[3]);
    	}
    	break;

    case 553: // Set/Get netmask
    	if(gb->Seen('P'))
    		SetEthernetAddress(gb, code);
    	else
    	{
    		byte *nm = platform->NetMask();
    		snprintf(reply, STRING_LENGTH, "Net mask: %d.%d.%d.%d\n ", nm[0], nm[1], nm[2], nm[3]);
    	}
    	break;

    case 554: // Set/Get gateway
    	if(gb->Seen('P'))
    		SetEthernetAddress(gb, code);
    	else
    	{
    		byte *gw = platform->GateWay();
    		snprintf(reply, STRING_LENGTH, "Gateway: %d.%d.%d.%d\n ", gw[0], gw[1], gw[2], gw[3]);
    	}
    	break;

    case 555: // Set firmware type to emulate
    	if(gb->Seen('P'))
    		platform->SetEmulating((Compatibility)gb->GetIValue());
    	break;

    case 556: // Axis compensation
    	if(gb->Seen('S'))
    	{
    		value = gb->GetFValue();
    		for(int8_t axis = 0; axis < AXES; axis++)
    			if(gb->Seen(gCodeLetters[axis]))
    				reprap.GetMove()->SetAxisCompensation(axis, gb->GetFValue()/value);
    	}
    	break;

    case 557: // Set Z probe point coordinates
    	if(gb->Seen('P'))
    	{
    		iValue = gb->GetIValue();
    		if(gb->Seen(gCodeLetters[X_AXIS]))
    			reprap.GetMove()->SetXBedProbePoint(iValue, gb->GetFValue());
    		if(gb->Seen(gCodeLetters[Y_AXIS]))
    		    reprap.GetMove()->SetYBedProbePoint(iValue, gb->GetFValue());
    	}
    	break;

    case 558: // Set Z probe type
    	if(gb->Seen('P'))
    		platform->SetZProbeType(gb->GetIValue());
    	break;

    case 559: // Upload config.g
    	if(gb->Seen('P'))
    		str = gb->GetString();
    	else
    		str = platform->GetConfigFile();
        OpenFileToWrite(platform->GetSysDir(), str, gb);
        snprintf(reply, STRING_LENGTH, "Writing to file: %s", str);
    	break;

    case 560: // Upload reprap.htm
         str = INDEX_PAGE;
         OpenFileToWrite(platform->GetWebDir(), str, gb);
         snprintf(reply, STRING_LENGTH, "Writing to file: %s", str);
     	break;

    case 561:
    	reprap.GetMove()->SetIdentityTransform();
    	break;

    case 562: // Reset temperature fault - use with great caution
    	if(gb->Seen('P'))
    	{
    	    iValue = gb->GetIValue();
    	    reprap.GetHeat()->ResetFault(iValue);
    	}
    	break;

//    case 876: // TEMPORARY - this will go away...
//    	if(gb->Seen('P'))
//    	{
//    		iValue = gb->GetIValue();
//    		if(iValue != 1)
//    			platform->SetHeatOn(0);
//    		else
//    			platform->SetHeatOn(1);
//    	}
//    	break;

    case 900:
    	result = DoFileCannedCycles("homex.g");
    	break;

    case 901:
    	result = DoFileCannedCycles("homey.g");
    	break;



    case 906: // Set Motor currents
    	for(uint8_t i = 0; i < DRIVES; i++)
    	{
    		if(gb->Seen(gCodeLetters[i]))
    		{
    			value = gb->GetFValue(); // mA
    			platform->SetMotorCurrent(i, value);
    		}
    	}
    	break;

    case 998:
    	if(gb->Seen('P'))
    	{
    	    snprintf(reply, STRING_LENGTH, "%s", gb->GetIValue());
    	    resend = true;
    	}
    	break;
     
    default:
      error = true;
      snprintf(reply, STRING_LENGTH, "invalid M Code: %s", gb->Buffer());
    }
    if(result)
    	HandleReply(error, gb == serialGCode, reply, 'M', code, resend);
    return result;
  }
  
  if(gb->Seen('T'))
  {
    code = gb->GetIValue();
    if(code == selectedHead)
    {
    	if(result)
    		HandleReply(error, gb == serialGCode, reply, 'T', code, resend);
    	return result;
    }

    error = true;
    for(int8_t i = AXES; i < DRIVES; i++)
    {
    	if(selectedHead == i - AXES)
    		reprap.GetHeat()->Standby(selectedHead + 1); // + 1 because 0 is the Bed
    }
    for(int8_t i = AXES; i < DRIVES; i++)
    {    
      if(code == i - AXES)
      {
        selectedHead = code;
        reprap.GetHeat()->Activate(selectedHead + 1); // 0 is the Bed
        error = false;
      }
    }

    if(error)
      snprintf(reply, STRING_LENGTH, "Invalid T Code: %s", gb->Buffer());

    if(result)
    	HandleReply(error, gb == serialGCode, reply, 'T', code, resend);
    return result;
  }
  
  // An empty buffer jumps to here and gets discarded

  if(result)
  	HandleReply(error, gb == serialGCode, reply, 'X', code, resend);

  return result;
}



//*************************************************************************************

// This class stores a single G Code and provides functions to allow it to be parsed

GCodeBuffer::GCodeBuffer(Platform* p, char* id)
{ 
  platform = p;
  identity = id;
  writingFileDirectory = NULL;  // Has to be done here as Init() is called every line.
}

void GCodeBuffer::Init()
{
  gcodePointer = 0;
  readPointer = -1;
  inComment = false;   
}

int GCodeBuffer::CheckSum()
{
	int cs = 0;
	for(int i = 0; gcodeBuffer[i] != '*' && gcodeBuffer[i] != NULL; i++)
	   cs = cs ^ gcodeBuffer[i];
	cs &= 0xff;  // Defensive programming...
	return cs;
}

// Add a byte to the code being assembled.  If false is returned, the code is
// not yet complete.  If true, it is complete and ready to be acted upon.

bool GCodeBuffer::Put(char c)
{
  bool result = false;
  gcodeBuffer[gcodePointer] = c;
  
  if(c == ';')
    inComment = true;
    
  if(c == '\n' || !c)
  {
    gcodeBuffer[gcodePointer] = 0;
    Init();
    if(reprap.Debug() && gcodeBuffer[0]) // Don't bother with blank/comment lines
    {
      platform->Message(HOST_MESSAGE, identity);
      platform->Message(HOST_MESSAGE, gcodeBuffer);
      platform->Message(HOST_MESSAGE, "\n"); 
    }

    // Deal with line numbers and checksums

    if(Seen('*'))
    {
    	int csSent = GetIValue();
    	int csHere = CheckSum();
    	Seen('N');
    	if(csSent != csHere)
    	{
    		snprintf(gcodeBuffer, GCODE_LENGTH, "M998 P%d", GetIValue());
    		Init();
    		result = true;
    		return result;
    	}

    	// Strip out the line number and checksum

    	while(gcodeBuffer[gcodePointer] != ' ' && gcodeBuffer[gcodePointer])
    		gcodePointer++;

    	// Anything there?

    	if(!gcodeBuffer[gcodePointer])
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
    	while(gcodeBuffer[gcodePointer] != '*' && gcodeBuffer[gcodePointer])
    	{
    		gcodeBuffer[gp2] = gcodeBuffer[gcodePointer++];
    		gp2++;
    	}
    	gcodeBuffer[gp2] = 0;
    	Init();
    }

    result = true;
  } else
  {
    if(!inComment)
      gcodePointer++;
  }
  
  if(gcodePointer >= GCODE_LENGTH)
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
  while(gcodeBuffer[readPointer])
  {
    if(gcodeBuffer[readPointer] == c)
      return true;
    readPointer++;
  }
  readPointer = -1;
  return false;
}

// Get a float after a G Code letter found by a call to Seen()

float GCodeBuffer::GetFValue()
{
  if(readPointer < 0)
  {
     platform->Message(HOST_MESSAGE, "GCodes: Attempt to read a GCode float before a search.\n");
     return 0.0;
  }
  float result = (float)strtod(&gcodeBuffer[readPointer + 1], 0);
  readPointer = -1;
  return result; 
}

// Get a string after a G Code letter found by a call to Seen().
// It will be the whole of the rest of the GCode string, so strings
// should always be the last parameter.

char* GCodeBuffer::GetString()
{
	if(readPointer < 0)
	{
		platform->Message(HOST_MESSAGE, "GCodes: Attempt to read a GCode string before a search.\n");
		return "";
	}
	char* result = &gcodeBuffer[readPointer+1];
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

char* GCodeBuffer::GetUnprecedentedString()
{
  readPointer = 0;
  while(gcodeBuffer[readPointer] && gcodeBuffer[readPointer] != ' ')
	  readPointer++;

  if(!gcodeBuffer[readPointer])
  {
     platform->Message(HOST_MESSAGE, "GCodes: String expected but not seen.\n");
     return gcodeBuffer; // Good idea?
  }

  char* result = &gcodeBuffer[readPointer+1];
  readPointer = -1;
  return result;
}


// Get an long after a G Code letter

long GCodeBuffer::GetLValue()
{
  if(readPointer < 0)
  {
    platform->Message(HOST_MESSAGE, "GCodes: Attempt to read a GCode int before a search.\n");
    return 0;
  }
  long result = strtol(&gcodeBuffer[readPointer + 1], 0, 0);
  readPointer = -1;
  return result;  
}




