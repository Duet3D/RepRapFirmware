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
}

void GCodes::Exit()
{
   active = false;
}

void GCodes::Init()
{
  webGCode->Init();
  fileGCode->Init();
  serialGCode->Init();
  webGCode->SetFinished(true);
  fileGCode->SetFinished(true);
  serialGCode->SetFinished(true);
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
  homeX = false;
  homeY = false;
  homeZ = false;
  homeZFinalMove = false;
  dwellWaiting = false;
  stackPointer = 0;
  selectedHead = -1;
  gFeedRate = platform->MaxFeedrate(Z_AXIS); // Typically the slowest
  for(int i = 0; i < NUMBER_OF_PROBE_POINTS; i++)
	  bedZs[i] = 0.0;
  zProbesSet = false;
  probeCount = 0;
  cannedCycleMoveCount = 0;
  cannedCycleMoveQueued = false;
  active = true;
  dwellTime = platform->Time();
}

void GCodes::Spin()
{
  if(!active)
    return;
    
  char b;

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
    return;
  }
  
  if(!serialGCode->Finished())
  {
    serialGCode->SetFinished(ActOnGcode(serialGCode));
    return;
  }  

  if(!fileGCode->Finished())
  {
    fileGCode->SetFinished(ActOnGcode(fileGCode));
    return;
  }  

  // Now check if a G Code byte is available from each of the sources
  // in the same order for the same reason.

  if(webserver->GCodeAvailable())
  {
    if(webGCode->Put(webserver->ReadGCode()))
      webGCode->SetFinished(ActOnGcode(webGCode));
    return;
  }
  
  if(platform->GetLine()->Status() & byteAvailable)
  {
	platform->GetLine()->Read(b);
    if(serialGCode->Put(b))
      serialGCode->SetFinished(ActOnGcode(serialGCode));
    return;
  }  
  
  if(fileBeingPrinted != NULL)
  {
     if(fileBeingPrinted->Read(b))
     {
       if(fileGCode->Put(b))
         fileGCode->SetFinished(ActOnGcode(fileGCode));
     } else
     {
        if(fileGCode->Put('\n')) // In case there wasn't one ending the file
         fileGCode->SetFinished(ActOnGcode(fileGCode));
        fileBeingPrinted->Close();
        fileBeingPrinted = NULL;
     }
  }
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
  stackPointer++;
  
  return true;
}

// Recover a saved state.  Call repeatedly till it returns true.

bool GCodes::Pop()
{
  if(stackPointer <= 0)
  {
    platform->Message(HOST_MESSAGE, "Push(): stack underflow!\n");
    return true;  
  }
  
  if(!AllMovesAreFinishedAndMoveBufferIsLoaded())
    return false;
    
  stackPointer--;
  drivesRelative = drivesRelativeStack[stackPointer];
  axesRelative = axesRelativeStack[stackPointer];
  
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
  
  LoadMoveBufferFromGCode(gb);
  
  checkEndStops = false;
  moveAvailable = true;
  return true; 
}

// The Move class calls this function to find what to do next.

bool GCodes::ReadMove(float* m, bool& ce)
{
    if(!moveAvailable)
      return false; 
    for(int8_t i = 0; i <= DRIVES; i++) // 1 more for F
      m[i] = moveBuffer[i];
    ce = checkEndStops;
    moveAvailable = false;
    checkEndStops = false;
    return true;
}

// To execute any move, call this until it returns true.
// moveToDo[] entries corresponding with false entries in action[] will
// be ignored.  Recall that moveToDo[DRIVES] should contain the feedrate
// you want (if action[DRIVES] is true).

bool GCodes::DoCannedCycleMove(float moveToDo[], bool action[], bool ce)
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
		if(!Push()) // Wait for the RepRap to finish whatever it was doing and save it's state
			return false;
		for(int8_t drive = 0; drive <= DRIVES; drive++)
		{
			if(action[drive])
				moveBuffer[drive] = moveToDo[drive];
		}
		checkEndStops = ce;
		cannedCycleMoveQueued = true;
		moveAvailable = true;
	}
	return false;
}

// Home one or more of the axes.  Which ones are decided by the
// booleans homeX, homeY and homeZ.

bool GCodes::DoHome()
{
	// Treat more or less like any other move
	// Do one axis at a time, starting with X.

	float moveToDo[DRIVES+1];
	bool action[DRIVES+1];
	for(int8_t drive = 0; drive < DRIVES; drive++)
		action[drive] = false;
	action[DRIVES] = true;

	if(homeX)
	{
		action[X_AXIS] = true;
		moveToDo[X_AXIS] = -2.0*platform->AxisLength(X_AXIS);
		moveToDo[DRIVES] = platform->HomeFeedRate(X_AXIS);
		if(DoCannedCycleMove(moveToDo, action, true))
		{
			homeX = false;
			return NoHome();
		}
		return false;
	}

	if(homeY)
	{
		action[Y_AXIS] = true;
		moveToDo[Y_AXIS] = -2.0*platform->AxisLength(Y_AXIS);
		moveToDo[DRIVES] = platform->HomeFeedRate(Y_AXIS);
		if(DoCannedCycleMove(moveToDo, action, true))
		{
			homeY = false;
			return NoHome();
		}
		return false;
	}

	if(homeZ)
	{
		action[Z_AXIS] = true;
		moveToDo[DRIVES] = platform->HomeFeedRate(Z_AXIS);
		if(homeZFinalMove)
		{
			moveToDo[Z_AXIS] = 0.0;
			if(DoCannedCycleMove(moveToDo, action, false))
			{
				homeZFinalMove = false;
				homeZ = false;
				return NoHome();
			}
			return false;
		}else
		{
			moveToDo[Z_AXIS] = -2.0*platform->AxisLength(Z_AXIS);
			if(DoCannedCycleMove(moveToDo, action, true))
				homeZFinalMove = true;
			return false;
		}
	}

	// Should never get here

	checkEndStops = false;
	moveAvailable = false;

	return true;
}

// This lifts Z a bit, moves to the probe XY coordinates (obtained by a call to GetProbeCoordinates() ),
// probes the bed height, and records the Z coordinate probed.  If you want to program any general
// internal canned cycle, this shows how to do it.

bool GCodes::DoSingleZProbe()
{
	float x, y, z;

	reprap.GetMove()->SetIdentityTransform();  // It doesn't matter if these are called repeatedly

	float moveToDo[DRIVES+1];
	bool action[DRIVES+1];
	for(int8_t drive = 0; drive <= DRIVES; drive++)
		action[drive] = false;

	switch(cannedCycleMoveCount)
	{
	case 0:
		moveToDo[Z_AXIS] = Z_DIVE;
		action[Z_AXIS] = true;
		moveToDo[DRIVES] = platform->HomeFeedRate(Z_AXIS);
		action[DRIVES] = true;
		reprap.GetMove()->SetZProbing(false);
		if(DoCannedCycleMove(moveToDo, action, false))
			cannedCycleMoveCount++;
		return false;

	case 1:
		GetProbeCoordinates(probeCount, moveToDo[X_AXIS], moveToDo[Y_AXIS], moveToDo[Z_AXIS]);
		action[X_AXIS] = true;
		action[Y_AXIS] = true;
		// NB - we don't use the Z value
		moveToDo[DRIVES] = platform->HomeFeedRate(X_AXIS);
		action[DRIVES] = true;
		reprap.GetMove()->SetZProbing(false);
		if(DoCannedCycleMove(moveToDo, action, false))
			cannedCycleMoveCount++;
		return false;

	case 2:
		moveToDo[Z_AXIS] = -2.0*platform->AxisLength(Z_AXIS);
		action[Z_AXIS] = true;
		moveToDo[DRIVES] = platform->HomeFeedRate(Z_AXIS);
		action[DRIVES] = true;
		reprap.GetMove()->SetZProbing(true);
		if(DoCannedCycleMove(moveToDo, action, true))
		{
//			platform->GetLine()->Write(platform->ZProbe());
			cannedCycleMoveCount++;
		}
		return false;

	default:
		cannedCycleMoveCount = 0;
		bedZs[probeCount] = reprap.GetMove()->GetLastProbedZ();
		return true;
	}
}

// This probes multiple points on the bed (usually three in a
// triangle), then sets the bed transformation to compensate
// for the bed not quite being the plane Z = 0.

bool GCodes::DoMultipleZProbe()
{
	if(DoSingleZProbe())
		probeCount++;
	if(probeCount >= NUMBER_OF_PROBE_POINTS)
	{
		probeCount = 0;
		zProbesSet = true;
		reprap.GetMove()->SetZProbing(false);
		reprap.GetMove()->SetProbedBedPlane();
		return true;
	}
	return false;
}

// This returns the (X, Y) points to probe the bed at probe point count.  When probing,
// it returns false.  If called after probing has ended it returns true, and the Z coordinate
// probed is also returned.

bool GCodes::GetProbeCoordinates(int count, float& x, float& y, float& z)
{
	switch(count)
	{
	case 0:
		x = 0.2*platform->AxisLength(X_AXIS);
		y = 0.2*platform->AxisLength(Y_AXIS);
		break;
	case 1:
		x = 0.8*platform->AxisLength(X_AXIS);
		y = 0.2*platform->AxisLength(Y_AXIS);
		break;
	case 2:
		x = 0.5*platform->AxisLength(X_AXIS);
		y = 0.8*platform->AxisLength(Y_AXIS);
		break;
	default:
		platform->Message(HOST_MESSAGE, "probeCount beyond maximum requested.\n");
		break;
	}
	z = bedZs[count];
	return zProbesSet;
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

// Set up a file to print, but don't print it yet.

void GCodes::QueueFileToPrint(char* fileName)
{
  fileToPrint = platform->GetFileStore(platform->GetGCodeDir(), fileName, false);
  if(fileToPrint == NULL)
	  platform->Message(HOST_MESSAGE, "GCode file not found\n");
}

// Run the configuration G Code file to set up the machine.  Usually just called once
// on re-boot.

void GCodes::RunConfigurationGCodes()
{
	  fileToPrint = platform->GetFileStore(platform->GetSysDir(), platform->GetConfigFile(), false);
	  if(fileToPrint == NULL)
	  {
		  platform->Message(HOST_MESSAGE, "Configuration file not found\n");
		  return;
	  }
      fileBeingPrinted = fileToPrint;
      fileToPrint = NULL;
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


// Move expects all axis movements to be absolute, and all
// extruder drive moves to be relative.  This function serves that.

void GCodes::LoadMoveBufferFromGCode(GCodeBuffer *gb)
{
	float absE;

	for(uint8_t i = 0; i < DRIVES; i++)
	{
	    if(i < AXES)
	    {
	      if(gb->Seen(gCodeLetters[i]))
	      {
	        if(axesRelative)
	          moveBuffer[i] += gb->GetFValue()*distanceScale;
	        else
	          moveBuffer[i] = gb->GetFValue()*distanceScale;
	      }
	    } else
	    {
	      if(gb->Seen(gCodeLetters[i]))
	      {
	        if(drivesRelative)
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

// This sets positions.  I.e. it handles G92.

bool GCodes::SetPositions(GCodeBuffer *gb)
{
	if(!AllMovesAreFinishedAndMoveBufferIsLoaded())
		return false;

	LoadMoveBufferFromGCode(gb);

	reprap.GetMove()->SetPositions(moveBuffer);

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
	for(int8_t heater = 0; heater < DRIVES; heater++)
		reprap.GetHeat()->Standby(heater);
	return true;
}

void GCodes::HandleReply(bool error, bool fromLine, char* reply)
{
	if(!reply[0])
		return;
	if(error)
		platform->GetLine()->Write("Error: ");
	platform->GetLine()->Write(reply);
	platform->GetLine()->Write("\n");
}

// If the GCode to act on is completed, this returns true,
// otherwise false.  It is called repeatedly for a given
// GCode until it returns true for that code.

bool GCodes::ActOnGcode(GCodeBuffer *gb)
{
  int code;
  float value;
  char* str;
  bool result = true;
  bool error = false;
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

    case 31: // Return the probe value, or set probe variables
    	if(gb->Seen(gCodeLetters[Z_AXIS]))
    	{
    		platform->SetZProbeStopHeight(gb->GetFValue());
    		if(gb->Seen('P'))
    		{
    			platform->SetZProbe(gb->GetIValue());
    		}
    	} else
    		snprintf(reply, STRING_LENGTH, "%d", platform->ZProbe());
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
    if(result == true)
    	HandleReply(error, gb == serialGCode, reply);
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
      snprintf(reply, STRING_LENGTH, "GCode files:\n%s", platform->GetMassStorage()->FileList(platform->GetGCodeDir()));
      break;

    case 23: // Set file to print
      QueueFileToPrint(gb->GetUnprecedentedString());
      break;
      
    case 24: // Print/resume-printing the selected file
      fileBeingPrinted = fileToPrint;
      fileToPrint = NULL;
      break;
      
    case 25: // Pause the print
    	fileToPrint = fileBeingPrinted;
    	fileBeingPrinted = NULL;
    	break;

    case 82:
    	if(drivesRelative)
    		for(int8_t extruder = AXES; extruder < DRIVES; extruder++)
    		    lastPos[extruder - AXES] = 0.0;
    	drivesRelative = false;
    	break;

    case 83:
    	if(!drivesRelative)
    		for(int8_t extruder = AXES; extruder < DRIVES; extruder++)
    			lastPos[extruder - AXES] = 0.0;
    	drivesRelative = true;

    	break;

    case 85: // Set inactive time
    	break;

    case 92: // Set steps/mm for each axis
    	for(int8_t drive = 0; drive < DRIVES; drive++)
    	{
    		if(gb->Seen(gCodeLetters[drive]))
    		{
    			value = gb->GetFValue();
    		}else{
    			value = -1;
    		}
    		platform->SetDriveStepsPerUnit(drive, value);
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
   
    case 106: // Fan on
    	if(gb->Seen('S'))
    		platform->CoolingFan(gb->GetFValue());
      break;
    
    case 107: // Fan off - depricated
    	platform->CoolingFan(0.0);
      break;
      
    case 112: // Emergency stop
    	reprap.EmergencyStop();
    	break;

    case 111: // Debug level
    	if(gb->Seen('S'))
    		reprap.SetDebug(gb->GetIValue());
    	break;

    case 114: // Deprecated
    	str = GetCurrentCoordinates();
    	if(str != 0)
    	{
    		strncpy(reply, str, STRING_LENGTH);
    	} else
    		result = false;
    	break;

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

    case 500: // Set password
    	if(gb->Seen('P'))
    		reprap.GetWebserver()->SetPassword(gb->GetString());
    	break;

    case 501: // Set machine name
    	if(gb->Seen('P'))
    		reprap.GetWebserver()->SetName(gb->GetString());
    	break;

    case 502: // Set IP address
    	break;

    case 503: // Set firmware type to emulate
    	if(gb->Seen('P'))
    		platform->SetEmulating((Compatibility)gb->GetIValue());
    	break;

    case 504: // Axis compensation
    	if(gb->Seen('S'))
    	{
    		value = gb->GetFValue();
    		for(int8_t axis = 0; axis < AXES; axis++)
    			if(gb->Seen(gCodeLetters[axis]))
    				reprap.GetMove()->SetAxisCompensation(axis, gb->GetFValue()/value);
    	}
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
     
    default:
      error = true;
      snprintf(reply, STRING_LENGTH, "invalid M Code: %s", gb->Buffer());
    }
    if(result == true)
    	HandleReply(error, gb == serialGCode, reply);
    return result;
  }
  
  if(gb->Seen('T'))
  {
    code = gb->GetIValue();
    if(code == selectedHead)
      return result;
      
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
      snprintf(reply, STRING_LENGTH, "invalid T Code: %s", gb->Buffer());

    if(result == true)
    	HandleReply(error, gb == serialGCode, reply);
    return result;
  }
  
  // An empty buffer jumps to here and gets discarded

  if(result == true)
  	HandleReply(error, gb == serialGCode, reply);

  return result;
}



//*************************************************************************************

// This class stores a single G Code and provides functions to allow it to be parsed

GCodeBuffer::GCodeBuffer(Platform* p, char* id)
{ 
  platform = p;
  identity = id; 
}

void GCodeBuffer::Init()
{
  gcodePointer = 0;
  readPointer = -1;
  inComment = false;   
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




