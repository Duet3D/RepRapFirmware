/*
 * GCodes2.cpp
 *
 *  Created on: 3 Dec 2016
 *      Author: David
 *
 *  This file contains the code to see what G, M or T command we have and start processing it.
 */

#include "RepRapFirmware.h"

#ifdef DUET_NG
#include "FirmwareUpdater.h"
#endif

const char* BED_EQUATION_G = "bed.g";
const char* RESUME_G = "resume.g";
const char* CANCEL_G = "cancel.g";
const char* STOP_G = "stop.g";
const char* SLEEP_G = "sleep.g";
const char* CONFIG_OVERRIDE_G = "config-override.g";
const char* DEPLOYPROBE_G = "deployprobe.g";
const char* RETRACTPROBE_G = "retractprobe.g";

const float MinServoPulseWidth = 544.0, MaxServoPulseWidth = 2400.0;
const uint16_t ServoRefreshFrequency = 50;

// If the code to act on is completed, this returns true,
// otherwise false.  It is called repeatedly for a given
// code until it returns true for that code.
bool GCodes::ActOnCode(GCodeBuffer& gb, StringRef& reply)
{
	// Discard empty buffers right away
	if (gb.IsEmpty())
	{
		return true;
	}

	// M-code parameters might contain letters T and G, e.g. in filenames.
	// dc42 assumes that G-and T-code parameters never contain the letter M.
	// Therefore we must check for an M-code first.
	if (gb.Seen('M'))
	{
		return HandleMcode(gb, reply);
	}
	// dc42 doesn't think a G-code parameter ever contains letter T, or a T-code ever contains letter G.
	// So it doesn't matter in which order we look for them.
	if (gb.Seen('G'))
	{
		return HandleGcode(gb, reply);
	}
	if (gb.Seen('T'))
	{
		return HandleTcode(gb, reply);
	}

	// An invalid or queued buffer gets discarded
	HandleReply(gb, false, "");
	return true;
}

bool GCodes::HandleGcode(GCodeBuffer& gb, StringRef& reply)
{
	bool result = true;
	bool error = false;

	int code = gb.GetIValue();
	if (simulationMode != 0 && code != 0 && code != 1 && code != 4 && code != 10 && code != 20 && code != 21 && code != 90 && code != 91 && code != 92)
	{
		return true;			// we only simulate some gcodes
	}
	if (gb.MachineState().runningM502 && code != 31)
	{
		return true;			// when running M502 the only gcode we execute is G31
	}

	switch (code)
	{
	case 0: // There are no rapid moves...
	case 1: // Ordinary move
		if (!LockMovement(gb))
		{
			return false;
		}
		{
			// Check for 'R' parameter here to go back to the coordinates at which the print was paused
			// NOTE: restore point 2 (tool change) won't work when changing tools on dual axis machines because of X axis mapping.
			// We could possibly fix this by saving the virtual X axis position instead of the physical axis positions.
			// However, slicers normally command the tool to the correct place after a tool change, so we don't need this feature anyway.
			int rParam = (gb.Seen('R')) ? gb.GetIValue() : 0;
			RestorePoint *rp = (rParam == 1) ? &pauseRestorePoint : (rParam == 2) ? &toolChangeRestorePoint : nullptr;
			if (rp != nullptr)
			{
				if (segmentsLeft != 0)
				{
					return false;
				}
				for (size_t axis = 0; axis < numAxes; ++axis)
				{
					float offset = gb.Seen(axisLetters[axis]) ? gb.GetFValue() * distanceScale : 0.0;
					moveBuffer.coords[axis] = rp->moveCoords[axis] + offset;
				}
				// For now we don't handle extrusion at the same time
				for (size_t drive = numAxes; drive < DRIVES; ++drive)
				{
					moveBuffer.coords[drive] = 0.0;
				}
				moveBuffer.feedRate = (gb.Seen(feedrateLetter)) ? gb.GetFValue() : gb.MachineState().feedrate;
				moveBuffer.filePos = noFilePosition;
				moveBuffer.usePressureAdvance = false;
				segmentsLeft = 1;
			}
			else
			{
				int res = SetUpMove(gb, reply);
				if (res == 2)
				{
					gb.SetState(GCodeState::waitingForMoveToComplete);
				}
				result = (res != 0);
			}
		}
		break;

	case 4: // Dwell
		result = DoDwell(gb);
		break;

	case 10: // Set/report offsets and temperatures, or retract
		if (gb.Seen('P'))
		{
			if (!SetOrReportOffsets(gb, reply))
			{
				return false;
			}
		}
		else
		{
			if (!LockMovement(gb))
			{
				return false;
			}
			result = RetractFilament(gb, true);
		}
		break;

	case 11: // Un-retract
		if (!LockMovement(gb))
		{
			return false;
		}
		result = RetractFilament(gb, false);
		break;

	case 20: // Inches (which century are we living in, here?)
		distanceScale = INCH_TO_MM;
		break;

	case 21: // mm
		distanceScale = 1.0;
		break;

	case 28: // Home
		result = DoHome(gb, reply, error);
		break;

	case 29: // Grid-based bed probing
		if (!LockMovementAndWaitForStandstill(gb))		// do this first to make sure that a new grid isn't being defined
		{
			return false;
		}
		{
			const int sparam = (gb.Seen('S')) ? gb.GetIValue() : 0;
			switch(sparam)
			{
			case 0:		// probe and save height map
				error = ProbeGrid(gb, reply);
				break;

			case 1:		// load height map file
				error = LoadHeightMap(gb, reply);
				break;

			default:	// clear height map
				reprap.GetMove()->AccessBedProbeGrid().ClearGridHeights();
				break;
			}
		}
		break;

	case 30: // Z probe/manually set at a position and set that as point P
		if (!LockMovementAndWaitForStandstill(gb))
		{
			return false;
		}
		if (reprap.GetMove()->IsDeltaMode() && !AllAxesAreHomed())
		{
			reply.copy("Must home a delta printer before bed probing");
			error = true;
		}
		else
		{
			result = SetSingleZProbeAtAPosition(gb, reply);
		}
		break;

	case 31: // Return the probe value, or set probe variables
		result = SetPrintZProbe(gb, reply);
		break;

	case 32: // Probe Z at multiple positions and generate the bed transform
		if (!LockMovementAndWaitForStandstill(gb))
		{
			return false;
		}

		// Try to execute bed.g
		if (!DoFileMacro(gb, BED_EQUATION_G, reprap.GetMove()->IsDeltaMode()))
		{
			// If we get here then we are not on a delta printer and there is no bed.g file
			if (GetAxisIsHomed(X_AXIS) && GetAxisIsHomed(Y_AXIS))
			{
				gb.SetState(GCodeState::setBed1);		// no bed.g file, so use the coordinates specified by M557
			}
			else
			{
				// We can only do bed levelling if X and Y have already been homed
				reply.copy("Must home X and Y before bed probing");
				error = true;
			}
		}
		break;

	case 90: // Absolute coordinates
		gb.MachineState().axesRelative = false;
		break;

	case 91: // Relative coordinates
		gb.MachineState().axesRelative = true;   // Axis movements (i.e. X, Y and Z)
		break;

	case 92: // Set position
		result = SetPositions(gb);
		break;

	default:
		error = true;
		reply.printf("invalid G Code: %s", gb.Buffer());
	}

	if (result && gb.GetState() == GCodeState::normal)
	{
		UnlockAll(gb);
		HandleReply(gb, error, reply.Pointer());
	}
	return result;
}

bool GCodes::HandleMcode(GCodeBuffer& gb, StringRef& reply)
{
	bool result = true;
	bool error = false;

	const int code = gb.GetIValue();
	if (simulationMode != 0 && (code < 20 || code > 37) && code != 0 && code != 1 && code != 82 && code != 83 && code != 105 && code != 111 && code != 112 && code != 122 && code != 408 && code != 999)
	{
		return true;			// we don't yet simulate most M codes
	}
	if (gb.MachineState().runningM502 && code != 301 && code != 307 && code != 558 && code != 665 && code != 666)
	{
		return true;			// when running M502 the only mcodes we execute are 301, 307, 558, 665 and 666
	}

	switch (code)
	{
	case 0: // Stop
	case 1: // Sleep
		if (!LockMovementAndWaitForStandstill(gb))
		{
			return false;
		}
		{
			bool wasPaused = isPaused;			// isPaused gets cleared by CancelPrint
			CancelPrint();
			if (wasPaused)
			{
				reply.copy("Print cancelled");
				// If we are cancelling a paused print with M0 and cancel.g exists then run it and do nothing else
				if (code == 0)
				{
					if (DoFileMacro(gb, CANCEL_G, false))
					{
						break;
					}
				}
			}
		}

		gb.SetState((code == 0) ? GCodeState::stopping : GCodeState::sleeping);
		DoFileMacro(gb, (code == 0) ? STOP_G : SLEEP_G, false);
		break;

#if SUPPORT_ROLAND
	case 3: // Spin spindle
		if (reprap.GetRoland()->Active())
		{
			if (gb.Seen('S'))
			{
				result = reprap.GetRoland()->ProcessSpindle(gb.GetFValue());
			}
		}
		break;
#endif

	case 18: // Motors off
	case 84:
		if (!LockMovementAndWaitForStandstill(gb))
		{
			return false;
		}
		{
			bool seen = false;
			for (size_t axis = 0; axis < numAxes; axis++)
			{
				if (gb.Seen(axisLetters[axis]))
				{
					SetAxisNotHomed(axis);
					platform->DisableDrive(axis);
					seen = true;
				}
			}

			if (gb.Seen(extrudeLetter))
			{
				long int eDrive[MaxExtruders];
				size_t eCount = numExtruders;
				gb.GetLongArray(eDrive, eCount);
				for (size_t i = 0; i < eCount; i++)
				{
					seen = true;
					if (eDrive[i] < 0 || (size_t)eDrive[i] >= numExtruders)
					{
						reply.printf("Invalid extruder number specified: %ld", eDrive[i]);
						error = true;
						break;
					}
					platform->DisableDrive(numAxes + eDrive[i]);
				}
			}

			if (gb.Seen('S'))
			{
				seen = true;

				float idleTimeout = gb.GetFValue();
				if (idleTimeout < 0.0)
				{
					reply.copy("Idle timeouts cannot be negative!");
					error = true;
				}
				else
				{
					reprap.GetMove()->SetIdleTimeout(idleTimeout);
				}
			}

			if (!seen)
			{
				DisableDrives();
			}
		}
		break;

	case 20:		// List files on SD card
		if (!LockFileSystem(gb))		// don't allow more than one at a time to avoid contention on output buffers
		{
			return false;
		}
		{
			OutputBuffer *fileResponse;
			const int sparam = (gb.Seen('S')) ? gb.GetIValue() : 0;
			const char* dir = (gb.Seen('P')) ? gb.GetString() : platform->GetGCodeDir();

			if (sparam == 2)
			{
				fileResponse = reprap.GetFilesResponse(dir, true);		// Send the file list in JSON format
				fileResponse->cat('\n');
			}
			else
			{
				if (!OutputBuffer::Allocate(fileResponse))
				{
					// Cannot allocate an output buffer, try again later
					return false;
				}

				// To mimic the behaviour of the official RepRapPro firmware:
				// If we are emulating RepRap then we print "GCode files:\n" at the start, otherwise we don't.
				// If we are emulating Marlin and the code came via the serial/USB interface, then we don't put quotes around the names and we separate them with newline;
				// otherwise we put quotes around them and separate them with comma.
				if (platform->Emulating() == me || platform->Emulating() == reprapFirmware)
				{
					fileResponse->copy("GCode files:\n");
				}

				bool encapsulateList = ((&gb != serialGCode && &gb != telnetGCode) || platform->Emulating() != marlin);
				FileInfo fileInfo;
				if (platform->GetMassStorage()->FindFirst(dir, fileInfo))
				{
					// iterate through all entries and append each file name
					do {
						if (encapsulateList)
						{
							fileResponse->catf("%c%s%c%c", FILE_LIST_BRACKET, fileInfo.fileName, FILE_LIST_BRACKET, FILE_LIST_SEPARATOR);
						}
						else
						{
							fileResponse->catf("%s\n", fileInfo.fileName);
						}
					} while (platform->GetMassStorage()->FindNext(fileInfo));

					if (encapsulateList)
					{
						// remove the last separator
						(*fileResponse)[fileResponse->Length() - 1] = 0;
					}
				}
				else
				{
					fileResponse->cat("NONE\n");
				}
			}

			UnlockAll(gb);
			HandleReply(gb, false, fileResponse);
			return true;
		}

	case 21: // Initialise SD card
		if (!LockFileSystem(gb))		// don't allow more than one at a time to avoid contention on output buffers
		{
			return false;
		}
		{
			size_t card = (gb.Seen('P')) ? gb.GetIValue() : 0;
			result = platform->GetMassStorage()->Mount(card, reply, true);
		}
		break;

	case 22: // Release SD card
		if (!LockFileSystem(gb))		// don't allow more than one at a time to avoid contention on output buffers
		{
			return false;
		}
		{
			size_t card = (gb.Seen('P')) ? gb.GetIValue() : 0;
			result = platform->GetMassStorage()->Unmount(card, reply);
		}
		break;

	case 23: // Set file to print
	case 32: // Select file and start SD print
		if (fileGCode->OriginalMachineState().fileState.IsLive())
		{
			reply.copy("Cannot set file to print, because a file is already being printed");
			error = true;
			break;
		}

		if (code == 32 && !LockMovementAndWaitForStandstill(gb))
		{
			return false;
		}
		{
			const char* filename = gb.GetUnprecedentedString();
			if (filename != nullptr)
			{
				QueueFileToPrint(filename);
				if (fileToPrint.IsLive())
				{
					reprap.GetPrintMonitor()->StartingPrint(filename);
					if (platform->Emulating() == marlin && (&gb == serialGCode || &gb == telnetGCode))
					{
						reply.copy("File opened\nFile selected");
					}
					else
					{
						// Command came from web interface or PanelDue, or not emulating Marlin, so send a nicer response
						reply.printf("File %s selected for printing", filename);
					}

					if (code == 32)
					{
						fileGCode->OriginalMachineState().fileState.MoveFrom(fileToPrint);
						reprap.GetPrintMonitor()->StartedPrint();
					}
				}
				else
				{
					reply.printf("Failed to open file %s", filename);
				}
			}
		}
		break;

	case 24: // Print/resume-printing the selected file
		if (!LockMovementAndWaitForStandstill(gb))
		{
			return false;
		}

		if (isPaused)
		{
			gb.SetState(GCodeState::resuming1);
			DoFileMacro(gb, RESUME_G, true);
		}
		else if (!fileToPrint.IsLive())
		{
			reply.copy("Cannot print, because no file is selected!");
			error = true;
		}
		else
		{
			fileGCode->OriginalMachineState().fileState.MoveFrom(fileToPrint);
			reprap.GetPrintMonitor()->StartedPrint();
		}
		break;

	case 226: // Gcode Initiated Pause
		if (&gb == fileGCode)			// ignore M226 if it did't come from within a file being printed
		{
			if (!LockMovement(gb))					// lock movement before calling DoPause
			{
				return false;
			}
			DoPause(gb);
		}
		break;

	case 25: // Pause the print
		if (isPaused)
		{
			reply.copy("Printing is already paused!!");
			error = true;
		}
		else if (!reprap.GetPrintMonitor()->IsPrinting())
		{
			reply.copy("Cannot pause print, because no file is being printed!");
			error = true;
		}
		else
		{
			if (!LockMovement(gb))					// lock movement before calling DoPause
			{
				return false;
			}
			DoPause(gb);
		}
		break;

	case 26: // Set SD position
		if (gb.Seen('S'))
		{
			const FilePosition value = gb.GetIValue();
			if (value < 0)
			{
				reply.copy("SD positions can't be negative!");
				error = true;
			}
			else if (fileGCode->OriginalMachineState().fileState.IsLive())
			{
				if (!fileGCode->OriginalMachineState().fileState.Seek(value))
				{
					reply.copy("The specified SD position is invalid!");
					error = true;
				}
			}
			else if (fileToPrint.IsLive())
			{
				if (!fileToPrint.Seek(value))
				{
					reply.copy("The specified SD position is invalid!");
					error = true;
				}
			}
			else
			{
				reply.copy("Cannot set SD file position, because no print is in progress!");
				error = true;
			}
		}
		else
		{
			reply.copy("You must specify the SD position in bytes using the S parameter.");
			error = true;
		}
		break;

	case 27: // Report print status - Deprecated
		if (reprap.GetPrintMonitor()->IsPrinting())
		{
			// Pronterface keeps sending M27 commands if "Monitor status" is checked, and it specifically expects the following response syntax
			FileData& fileBeingPrinted = fileGCode->OriginalMachineState().fileState;
			reply.printf("SD printing byte %lu/%lu", fileBeingPrinted.GetPosition(), fileBeingPrinted.Length());
		}
		else
		{
			reply.copy("Not SD printing.");
		}
		break;

	case 28: // Write to file
		{
			const char* str = gb.GetUnprecedentedString();
			if (str != nullptr)
			{
				bool ok = OpenFileToWrite(gb, platform->GetGCodeDir(), str);
				if (ok)
				{
					reply.printf("Writing to file: %s", str);
				}
				else
				{
					reply.printf("Can't open file %s for writing.", str);
					error = true;
				}
			}
		}
		break;

	case 29: // End of file being written; should be intercepted before getting here
		reply.copy("GCode end-of-file being interpreted.");
		break;

	case 30:	// Delete file
		{
			const char *filename = gb.GetUnprecedentedString();
			if (filename != nullptr)
			{
				DeleteFile(filename);
			}
		}
		break;

		// For case 32, see case 24

	case 36:	// Return file information
		if (!LockFileSystem(gb))									// getting file info takes several calls and isn't reentrant
		{
			return false;
		}
		{
			const char* filename = gb.GetUnprecedentedString(true);	// get filename, or nullptr if none provided
			OutputBuffer *fileInfoResponse;
			result = reprap.GetPrintMonitor()->GetFileInfoResponse(filename, fileInfoResponse);
			if (result)
			{
				fileInfoResponse->cat('\n');
				UnlockAll(gb);
				HandleReply(gb, false, fileInfoResponse);
				return true;
			}
		}
		break;

	case 37:	// Simulation mode on/off
		if (gb.Seen('S'))
		{
			if (!LockMovementAndWaitForStandstill(gb))
			{
				return false;
			}

			bool wasSimulating = (simulationMode != 0);
			simulationMode = (uint8_t)gb.GetIValue();
			reprap.GetMove()->Simulate(simulationMode);

			if (simulationMode != 0)
			{
				simulationTime = 0.0;
				if (!wasSimulating)
				{
					// Starting a new simulation, so save the current position
					reprap.GetMove()->GetCurrentUserPosition(simulationRestorePoint.moveCoords, 0, reprap.GetCurrentXAxes());
					simulationRestorePoint.feedRate = gb.MachineState().feedrate;
				}
			}
			else if (wasSimulating)
			{
				// Ending a simulation, so restore the position
				SetPositions(simulationRestorePoint.moveCoords);
				for (size_t i = 0; i < DRIVES; ++i)
				{
					moveBuffer.coords[i] = simulationRestorePoint.moveCoords[i];
				}
				gb.MachineState().feedrate = simulationRestorePoint.feedRate;
			}
		}
		else
		{
			reply.printf("Simulation mode: %s, move time: %.1f sec, other time: %.1f sec",
					(simulationMode != 0) ? "on" : "off", reprap.GetMove()->GetSimulationTime(), simulationTime);
		}
		break;

	case 38: // Report SHA1 of file
		if (!LockFileSystem(gb))								// getting file hash takes several calls and isn't reentrant
		{
			return false;
		}
		if (fileBeingHashed == nullptr)
		{
			// See if we can open the file and start hashing
			const char* filename = gb.GetUnprecedentedString(true);
			if (StartHash(filename))
			{
				// Hashing is now in progress...
				result = false;
			}
			else
			{
				error = true;
				reply.printf("Cannot open file: %s", filename);
			}
		}
		else
		{
			// This can take some time. All the actual heavy lifting is in dedicated methods
			result = AdvanceHash(reply);
		}
		break;

	case 42:	// Turn an output pin on or off
		if (gb.Seen('P'))
		{
			const int logicalPin = gb.GetIValue();
			Pin pin;
			bool invert;
			if (platform->GetFirmwarePin(logicalPin, PinAccess::pwm, pin, invert))
			{
				if (gb.Seen('S'))
				{
					float val = gb.GetFValue();
					if (val > 1.0)
					{
						val /= 255.0;
					}
					val = constrain<float>(val, 0.0, 1.0);
					if (invert)
					{
						val = 1.0 - val;
					}
					Platform::WriteAnalog(pin, val, DefaultPinWritePwmFreq);
				}
				// Ignore the command if no S parameter provided
			}
			else
			{
				reply.printf("Logical pin %d is not available for writing", logicalPin);
				error = true;
			}
		}
		break;

	case 80:	// ATX power on
		platform->SetAtxPower(true);
		break;

	case 81:	// ATX power off
		if (!LockMovementAndWaitForStandstill(gb))
		{
			return false;
		}
		platform->SetAtxPower(false);
		break;

	case 82:	// Use absolute extruder positioning
		if (gb.MachineState().drivesRelative)		// don't reset the absolute extruder position if it was already absolute
		{
			for (size_t extruder = 0; extruder < MaxExtruders; extruder++)
			{
				lastRawExtruderPosition[extruder] = 0.0;
			}
			gb.MachineState().drivesRelative = false;
		}
		break;

	case 83:	// Use relative extruder positioning
		if (!gb.MachineState().drivesRelative)	// don't reset the absolute extruder position if it was already relative
		{
			for (size_t extruder = 0; extruder < MaxExtruders; extruder++)
			{
				lastRawExtruderPosition[extruder] = 0.0;
			}
			gb.MachineState().drivesRelative = true;
		}
		break;

		// For case 84, see case 18

	case 85: // Set inactive time
		break;

	case 92: // Set/report steps/mm for some axes
		{
			// Save the current positions as we may need them later
			float positionNow[DRIVES];
			Move *move = reprap.GetMove();
			move->GetCurrentUserPosition(positionNow, 0, reprap.GetCurrentXAxes());

			bool seen = false;
			for (size_t axis = 0; axis < numAxes; axis++)
			{
				if (gb.Seen(axisLetters[axis]))
				{
					if (!LockMovementAndWaitForStandstill(gb))
					{
						return false;
					}
					platform->SetDriveStepsPerUnit(axis, gb.GetFValue());
					seen = true;
				}
			}

			if (gb.Seen(extrudeLetter))
			{
				if (!LockMovementAndWaitForStandstill(gb))
				{
					return false;
				}
				seen = true;
				float eVals[MaxExtruders];
				size_t eCount = numExtruders;
				gb.GetFloatArray(eVals, eCount, true);

				// The user may not have as many extruders as we allow for, so just set the ones for which a value is provided
				for (size_t e = 0; e < eCount; e++)
				{
					platform->SetDriveStepsPerUnit(numAxes + e, eVals[e]);
				}
			}

			if (seen)
			{
				// On a delta, if we change the drive steps/mm then we need to recalculate the motor positions
				SetPositions(positionNow);
			}
			else
			{
				reply.copy("Steps/mm: ");
				for (size_t axis = 0; axis < numAxes; ++axis)
				{
					reply.catf("%c: %.3f, ", axisLetters[axis], platform->DriveStepsPerUnit(axis));
				}
				reply.catf("E:");
				char sep = ' ';
				for (size_t extruder = 0; extruder < numExtruders; extruder++)
				{
					reply.catf("%c%.3f", sep, platform->DriveStepsPerUnit(extruder + numAxes));
					sep = ':';
				}
			}
		}
		break;

	case 98: // Call Macro/Subprogram
		if (gb.Seen('P'))
		{
			DoFileMacro(gb, gb.GetString(), true);
		}
		break;

	case 99: // Return from Macro/Subprogram
		FileMacroCyclesReturn(gb);
		break;

	case 104: // Deprecated.  This sets the active temperature of every heater of the active tool
		if (gb.Seen('S'))
		{
			float temperature = gb.GetFValue();
			Tool* tool;
			if (gb.Seen('T'))
			{
				int toolNumber = gb.GetIValue();
				toolNumber += gb.GetToolNumberAdjust();
				tool = reprap.GetTool(toolNumber);
			}
			else
			{
				tool = reprap.GetCurrentOrDefaultTool();
			}
			SetToolHeaters(tool, temperature);
		}
		break;

	case 105: // Get temperatures
		{
			const int8_t bedHeater = reprap.GetHeat()->GetBedHeater();
			const int8_t chamberHeater = reprap.GetHeat()->GetChamberHeater();
			reply.copy("T:");
			for (int8_t heater = 0; heater < HEATERS; heater++)
			{
				if (heater != bedHeater && heater != chamberHeater)
				{
					Heat::HeaterStatus hs = reprap.GetHeat()->GetStatus(heater);
					if (hs != Heat::HS_off && hs != Heat::HS_fault)
					{
						reply.catf("%.1f ", reprap.GetHeat()->GetTemperature(heater));
					}
				}
			}
			if (bedHeater >= 0)
			{
				reply.catf("B:%.1f", reprap.GetHeat()->GetTemperature(bedHeater));
			}
			else
			{
				// I'm not sure whether Pronterface etc. can handle a missing bed temperature, so return zero
				reply.cat("B:0.0");
			}
			if (chamberHeater >= 0.0)
			{
				reply.catf(" C:%.1f", reprap.GetHeat()->GetTemperature(chamberHeater));
			}
		}
		break;

	case 106: // Set/report fan values
		{
			bool seenFanNum = false;
			int32_t fanNum = 0;			// Default to the first fan
			gb.TryGetIValue('P', fanNum, seenFanNum);
			if (fanNum < 0 || fanNum > (int)NUM_FANS)
			{
				reply.printf("Fan number %d is invalid, must be between 0 and %u", fanNum, NUM_FANS);
			}
			else
			{
				Fan& fan = platform->GetFan(fanNum);
				if (!fan.IsEnabled())
				{
					reply.printf("Fan number %d is disabled", fanNum);
				}
				else
				{
					bool seen = false;
					if (gb.Seen('I'))		// Invert cooling
					{
						const int invert = gb.GetIValue();
						if (invert < 0)
						{
							fan.Disable();
						}
						else
						{
							fan.SetInverted(invert > 0);
						}
						seen = true;
					}

					if (gb.Seen('F'))		// Set PWM frequency
					{
						fan.SetPwmFrequency(gb.GetFValue());
						seen = true;
					}

					if (gb.Seen('T'))		// Set thermostatic trigger temperature
					{
						seen = true;
						fan.SetTriggerTemperature(gb.GetFValue());
					}

					if (gb.Seen('B'))		// Set blip time
					{
						seen = true;
						fan.SetBlipTime(gb.GetFValue());
					}

					if (gb.Seen('L'))		// Set minimum speed
					{
						seen = true;
						fan.SetMinValue(gb.GetFValue());
					}

					if (gb.Seen('H'))		// Set thermostatically-controller heaters
					{
						seen = true;
						long heaters[HEATERS];
						size_t numH = HEATERS;
						gb.GetLongArray(heaters, numH);
						// Note that M106 H-1 disables thermostatic mode. The following code implements that automatically.
						uint16_t hh = 0;
						for (size_t h = 0; h < numH; ++h)
						{
							const int hnum = heaters[h];
							if (hnum >= 0 && hnum < HEATERS)
							{
								hh |= (1u << (unsigned int)hnum);
							}
						}
						if (hh != 0)
						{
							platform->SetFanValue(fanNum, 1.0);			// default the fan speed to full for safety
						}
						fan.SetHeatersMonitored(hh);
					}

					if (gb.Seen('S'))		// Set new fan value - process this after processing 'H' or it may not be acted on
					{
						const float f = constrain<float>(gb.GetFValue(), 0.0, 255.0);
						if (seen || seenFanNum)
						{
							platform->SetFanValue(fanNum, f);
						}
						else
						{
							// We are processing an M106 S### command with no other recognised parameters and we have a tool selected.
							// Apply the fan speed setting to the fans in the fan mapping for the current tool.
							lastDefaultFanSpeed = f;
							SetMappedFanSpeed();
						}
					}
					else if (gb.Seen('R'))
					{
						const int i = gb.GetIValue();
						switch(i)
						{
						case 0:
						case 1:
							// Restore fan speed to value when print was paused
							platform->SetFanValue(fanNum, pausedFanValues[fanNum]);
							break;
						case 2:
							// Set the speeds of mapped fans to the last known value. Fan number is ignored.
							SetMappedFanSpeed();
							break;
						default:
							break;
						}
					}
					else if (!seen)
					{
						reply.printf("Fan%i frequency: %dHz, speed: %d%%, min: %d%%, blip: %.2f, inverted: %s",
										fanNum,
										(int)(fan.GetPwmFrequency()),
										(int)(fan.GetValue() * 100.0),
										(int)(fan.GetMinValue() * 100.0),
										fan.GetBlipTime(),
										(fan.GetInverted()) ? "yes" : "no");
						uint16_t hh = fan.GetHeatersMonitored();
						if (hh != 0)
						{
							reply.catf(", trigger: %dC, heaters:", (int)fan.GetTriggerTemperature());
							for (unsigned int i = 0; i < HEATERS; ++i)
							{
								if ((hh & (1u << i)) != 0)
								{
									reply.catf(" %u", i);
								}
							}
						}
					}
				}
			}
		}
		break;

	case 107: // Fan off - deprecated
		platform->SetFanValue(0, 0.0);		//T3P3 as deprecated only applies to fan0
		break;

	case 108: // Cancel waiting for temperature
		if (isWaiting)
		{
			cancelWait = true;
		}
		break;

	case 109: // Deprecated in RRF, but widely generated by slicers
		{
			float temperature;
			if (gb.Seen('R'))
			{
				gb.MachineState().waitWhileCooling = true;
				temperature = gb.GetFValue();
			}
			else if (gb.Seen('S'))
			{
				gb.MachineState().waitWhileCooling = false;
				temperature = gb.GetFValue();
			}
			else
			{
				break;		// no target temperature given
			}

			Tool *tool;
			if (gb.Seen('T'))
			{
				int toolNumber = gb.GetIValue();
				toolNumber += gb.GetToolNumberAdjust();
				tool = reprap.GetTool(toolNumber);
			}
			else
			{
				tool = reprap.GetCurrentOrDefaultTool();
			}

			SetToolHeaters(tool, temperature);
			if (tool == nullptr)
			{
				break;			// SetToolHeaters will already have generated an error message
			}

			// M109 implies waiting for temperature to be reached, so it doesn't make sense unless the tool has been selected.
			// Sadly, many slicers use M109 without selecting the tool first. So we select it here if necessary.
			if (tool != reprap.GetCurrentTool())
			{
				if (!LockMovementAndWaitForStandstill(gb))
				{
					return false;
				}

				newToolNumber = tool->Number();
				StartToolChange(gb, true);
			}
			else
			{
				gb.SetState(GCodeState::m109ToolChangeComplete);
			}
		}
		break;

	case 110: // Set line numbers - line numbers are dealt with in the GCodeBuffer class
		break;

	case 111: // Debug level
		if (gb.Seen('S'))
		{
			bool dbv = (gb.GetIValue() != 0);
			if (gb.Seen('P'))
			{
				reprap.SetDebug(static_cast<Module>(gb.GetIValue()), dbv);
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
		DoEmergencyStop();
		break;

	case 114:
		GetCurrentCoordinates(reply);
		break;

	case 115: // Print firmware version or set hardware type
		if (gb.Seen('P'))
		{
			platform->SetBoardType((BoardType)gb.GetIValue());
		}
		else
		{
			reply.printf("FIRMWARE_NAME: %s FIRMWARE_VERSION: %s ELECTRONICS: %s", FIRMWARE_NAME, VERSION, platform->GetElectronicsString());
#ifdef DUET_NG
			const char* expansionName = DuetExpansion::GetExpansionBoardName();
			if (expansionName != nullptr)
			{
				reply.catf(" + %s", expansionName);
			}
#endif
			reply.catf(" FIRMWARE_DATE: %s", DATE);
		}
		break;

	case 116: // Wait for set temperatures
		{
			bool seen = false;
			if (gb.Seen('P'))
			{
				// Wait for the heaters associated with the specified tool to be ready
				int toolNumber = gb.GetIValue();
				toolNumber += gb.GetToolNumberAdjust();
				if (!cancelWait && !ToolHeatersAtSetTemperatures(reprap.GetTool(toolNumber), true))
				{
					isWaiting = true;
					return false;
				}
				seen = true;
			}

			if (gb.Seen('H'))
			{
				// Wait for specified heaters to be ready
				long heaters[HEATERS];
				size_t heaterCount = HEATERS;
				gb.GetLongArray(heaters, heaterCount);
				if (!cancelWait)
				{
					for (size_t i=0; i<heaterCount; i++)
					{
						if (!reprap.GetHeat()->HeaterAtSetTemperature(heaters[i], true))
						{
							isWaiting = true;
							return false;
						}
					}
				}
				seen = true;
			}

			if (gb.Seen('C'))
			{
				// Wait for chamber heater to be ready
				const int8_t chamberHeater = reprap.GetHeat()->GetChamberHeater();
				if (chamberHeater != -1)
				{
					if (!cancelWait && !reprap.GetHeat()->HeaterAtSetTemperature(chamberHeater, true))
					{
						isWaiting = true;
						return false;
					}
				}
				seen = true;
			}

			// Wait for all heaters to be ready
			if (!seen && !cancelWait && !reprap.GetHeat()->AllHeatersAtSetTemperatures(true))
			{
				isWaiting = true;
				return false;
			}

			// If we get here, there is nothing more to wait for
			cancelWait = isWaiting = false;
		}
		break;

	case 117:	// Display message
		{
			const char *msg = gb.GetUnprecedentedString(true);
			reprap.SetMessage((msg == nullptr) ? "" : msg);
		}
		break;

	case 119:
		reply.copy("Endstops - ");
		for (size_t axis = 0; axis < numAxes; axis++)
		{
			reply.catf("%c: %s, ", axisLetters[axis], TranslateEndStopResult(platform->Stopped(axis)));
		}
		reply.catf("Z probe: %s", TranslateEndStopResult(platform->GetZProbeResult()));
		break;

	case 120:
		Push(gb);
		break;

	case 121:
		Pop(gb);
		break;

	case 122:
		{
			int val = (gb.Seen('P')) ? gb.GetIValue() : 0;
			if (val == 0)
			{
				reprap.Diagnostics(gb.GetResponseMessageType());
			}
			else
			{
				platform->DiagnosticTest(val);
			}
		}
		break;

	case 135: // Set PID sample interval
		if (gb.Seen('S'))
		{
			platform->SetHeatSampleTime(gb.GetFValue() * 0.001);  // Value is in milliseconds; we want seconds
		}
		else
		{
			reply.printf("Heat sample time is %.3f seconds", platform->GetHeatSampleTime());
		}
		break;

	case 140: // Set bed temperature
		{
			int8_t bedHeater;
			if (gb.Seen('H'))
			{
				bedHeater = gb.GetIValue();
				if (bedHeater < 0)
				{
					// Make sure we stay within reasonable boundaries...
					bedHeater = -1;

					// If we're disabling the hot bed, make sure the old heater is turned off
					reprap.GetHeat()->SwitchOff(reprap.GetHeat()->GetBedHeater());
				}
				else if (bedHeater >= HEATERS)
				{
					reply.copy("Invalid heater number!");
					error = true;
					break;
				}
				reprap.GetHeat()->SetBedHeater(bedHeater);

				if (bedHeater < 0)
				{
					// Stop here if the hot bed has been disabled
					break;
				}
			}
			else
			{
				bedHeater = reprap.GetHeat()->GetBedHeater();
				if (bedHeater < 0)
				{
					reply.copy("Hot bed is not present!");
					error = true;
					break;
				}
			}

			if(gb.Seen('S'))
			{
				float temperature = gb.GetFValue();
				if (temperature < NEARLY_ABS_ZERO)
				{
					reprap.GetHeat()->SwitchOff(bedHeater);
				}
				else
				{
					reprap.GetHeat()->SetActiveTemperature(bedHeater, temperature);
					reprap.GetHeat()->Activate(bedHeater);
				}
			}
			if(gb.Seen('R'))
			{
				reprap.GetHeat()->SetStandbyTemperature(bedHeater, gb.GetFValue());
			}
		}
		break;

	case 141: // Chamber temperature
		{
			bool seen = false;
			if (gb.Seen('H'))
			{
				seen = true;

				int heater = gb.GetIValue();
				if (heater < 0)
				{
					const int8_t currentHeater = reprap.GetHeat()->GetChamberHeater();
					if (currentHeater != -1)
					{
						reprap.GetHeat()->SwitchOff(currentHeater);
					}

					reprap.GetHeat()->SetChamberHeater(-1);
				}
				else if (heater < HEATERS)
				{
					reprap.GetHeat()->SetChamberHeater(heater);
				}
				else
				{
					reply.copy("Bad heater number specified!");
					error = true;
				}
			}

			if (gb.Seen('S'))
			{
				seen = true;

				const int8_t currentHeater = reprap.GetHeat()->GetChamberHeater();
				if (currentHeater != -1)
				{
					float temperature = gb.GetFValue();

					if (temperature < NEARLY_ABS_ZERO)
					{
						reprap.GetHeat()->SwitchOff(currentHeater);
					}
					else
					{
						reprap.GetHeat()->SetActiveTemperature(currentHeater, temperature);
						reprap.GetHeat()->Activate(currentHeater);
					}
				}
				else
				{
					reply.copy("No chamber heater has been set up yet!");
					error = true;
				}
			}

			if (!seen)
			{
				const int8_t currentHeater = reprap.GetHeat()->GetChamberHeater();
				if (currentHeater != -1)
				{
					reply.printf("Chamber heater %d is currently at %.1fC", currentHeater, reprap.GetHeat()->GetTemperature(currentHeater));
				}
				else
				{
					reply.copy("No chamber heater has been configured yet.");
				}
			}
		}
		break;

	case 143: // Set temperature limit
		{
			const int heater = (gb.Seen('H')) ? gb.GetIValue() : 1;		// default to extruder 1 if no heater number provided
			if (heater < 0 || heater >= HEATERS)
			{
				reply.copy("Invalid heater number");
				error = true;
			}
			else if (gb.Seen('S'))
			{
				const float limit = gb.GetFValue();
				if (limit > BAD_LOW_TEMPERATURE && limit < BAD_ERROR_TEMPERATURE)
				{
					reprap.GetHeat()->SetTemperatureLimit(heater, limit);
				}
				else
				{
					reply.copy("Invalid temperature limit");
					error = true;
				}
			}
			else
			{
				reply.printf("Temperature limit for heater %d is %.1fC", heater, reprap.GetHeat()->GetTemperatureLimit(heater));
			}
		}
		break;

	case 144: // Set bed to standby
		{
			const int8_t bedHeater = reprap.GetHeat()->GetBedHeater();
			if (bedHeater >= 0)
			{
				reprap.GetHeat()->Standby(bedHeater);
			}
		}
		break;

	case 190: // Set bed temperature and wait
	case 191: // Set chamber temperature and wait
		{
			const int8_t heater = (code == 191) ? reprap.GetHeat()->GetChamberHeater() : reprap.GetHeat()->GetBedHeater();
			if (heater >= 0)
			{
				float temperature;
				bool waitWhenCooling;
				if (gb.Seen('R'))
				{
					waitWhenCooling = true;
					temperature = gb.GetFValue();
				}
				else if (gb.Seen('S'))
				{
					waitWhenCooling = false;
					temperature = gb.GetFValue();
				}
				else
				{
					break;		// no target temperature given
				}

				reprap.GetHeat()->SetActiveTemperature(heater, temperature);
				reprap.GetHeat()->Activate(heater);
				if (cancelWait || reprap.GetHeat()->HeaterAtSetTemperature(heater, waitWhenCooling))
				{
					cancelWait = isWaiting = false;
					break;
				}
				// In Marlin emulation mode we should return some sort of (undocumented) message here every second...
				isWaiting = true;
				return false;
			}
		}
		break;

	case 201: // Set/print axis accelerations
		{
			bool seen = false;
			for (size_t axis = 0; axis < numAxes; axis++)
			{
				if (gb.Seen(axisLetters[axis]))
				{
					platform->SetAcceleration(axis, gb.GetFValue() * distanceScale);
					seen = true;
				}
			}

			if (gb.Seen(extrudeLetter))
			{
				seen = true;
				float eVals[MaxExtruders];
				size_t eCount = numExtruders;
				gb.GetFloatArray(eVals, eCount, true);
				for (size_t e = 0; e < eCount; e++)
				{
					platform->SetAcceleration(numAxes + e, eVals[e] * distanceScale);
				}
			}

			if (!seen)
			{
				reply.printf("Accelerations: ");
				for (size_t axis = 0; axis < numAxes; ++axis)
				{
					reply.catf("%c: %.1f, ", axisLetters[axis], platform->Acceleration(axis) / distanceScale);
				}
				reply.cat("E:");
				char sep = ' ';
				for (size_t extruder = 0; extruder < numExtruders; extruder++)
				{
					reply.catf("%c%.1f", sep, platform->Acceleration(extruder + numAxes) / distanceScale);
					sep = ':';
				}
			}
		}
		break;

	case 203: // Set/print maximum feedrates
		{
			bool seen = false;
			for (size_t axis = 0; axis < numAxes; ++axis)
			{
				if (gb.Seen(axisLetters[axis]))
				{
					platform->SetMaxFeedrate(axis, gb.GetFValue() * distanceScale * secondsToMinutes); // G Code feedrates are in mm/minute; we need mm/sec
					seen = true;
				}
			}

			if (gb.Seen(extrudeLetter))
			{
				seen = true;
				float eVals[MaxExtruders];
				size_t eCount = numExtruders;
				gb.GetFloatArray(eVals, eCount, true);
				for (size_t e = 0; e < eCount; e++)
				{
					platform->SetMaxFeedrate(numAxes + e, eVals[e] * distanceScale * secondsToMinutes);
				}
			}

			if (!seen)
			{
				reply.copy("Maximum feedrates: ");
				for (size_t axis = 0; axis < numAxes; ++axis)
				{
					reply.catf("%c: %.1f, ", axisLetters[axis], platform->MaxFeedrate(axis) / (distanceScale * secondsToMinutes));
				}
				reply.cat("E:");
				char sep = ' ';
				for (size_t extruder = 0; extruder < numExtruders; extruder++)
				{
					reply.catf("%c%.1f", sep, platform->MaxFeedrate(extruder + numAxes) / (distanceScale * secondsToMinutes));
					sep = ':';
				}
			}
		}
		break;

	case 206:  // Offset axes - Deprecated
		result = OffsetAxes(gb);
		break;

	case 207: // Set firmware retraction details
		{
			bool seen = false;
			if (gb.Seen('S'))
			{
				retractLength = max<float>(gb.GetFValue(), 0.0);
				seen = true;
			}
			if (gb.Seen('R'))
			{
				retractExtra = max<float>(gb.GetFValue(), -retractLength);
				seen = true;
			}
			if (gb.Seen('F'))
			{
				unRetractSpeed = retractSpeed = max<float>(gb.GetFValue(), 60.0);
				seen = true;
			}
			if (gb.Seen('T'))	// must do this one after 'F'
			{
				unRetractSpeed = max<float>(gb.GetFValue(), 60.0);
				seen = true;
			}
			if (gb.Seen('Z'))
			{
				retractHop = max<float>(gb.GetFValue(), 0.0);
				seen = true;
			}
			if (!seen)
			{
				reply.printf("Retraction settings: length %.2f/%.2fmm, speed %d/%dmm/min, Z hop %.2fmm",
						retractLength, retractLength + retractExtra, (int)retractSpeed, (int)unRetractSpeed, retractHop);
			}
		}
		break;

	case 208: // Set/print maximum axis lengths. If there is an S parameter with value 1 then we set the min value, else we set the max value.
		{
			bool setMin = (gb.Seen('S') ? (gb.GetIValue() == 1) : false);
			bool seen = false;
			for (size_t axis = 0; axis < numAxes; axis++)
			{
				if (gb.Seen(axisLetters[axis]))
				{
					const float value = gb.GetFValue() * distanceScale;
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
				reply.copy("Axis limits ");
				char sep = '-';
				for (size_t axis = 0; axis < numAxes; axis++)
				{
					reply.catf("%c %c: %.1f min, %.1f max", sep, axisLetters[axis], platform->AxisMinimum(axis),
							platform->AxisMaximum(axis));
					sep = ',';
				}
			}
		}
		break;

	case 210: // Set/print homing feed rates
		// This is no longer used, but for backwards compatibility we don't report an error
		break;

	case 220:	// Set/report speed factor override percentage
		if (gb.Seen('S'))
		{
			float newSpeedFactor = (gb.GetFValue() / 100.0) * secondsToMinutes;	// include the conversion from mm/minute to mm/second
			if (newSpeedFactor > 0.0)
			{
				gb.MachineState().feedrate *= newSpeedFactor / speedFactor;
				if (segmentsLeft != 0 && !moveBuffer.isFirmwareRetraction)
				{
					// The last move has not gone yet, so we can update it
					moveBuffer.feedRate *= newSpeedFactor / speedFactor;
				}
				speedFactor = newSpeedFactor;
			}
			else
			{
				reply.printf("Invalid speed factor specified.");
				error = true;
			}
		}
		else
		{
			reply.printf("Speed factor override: %.1f%%", speedFactor * minutesToSeconds * 100.0);
		}
		break;

	case 221:	// Set/report extrusion factor override percentage
		{
			int32_t extruder = 0;
			bool dummy;
			gb.TryGetIValue('D', extruder, dummy);

			if (extruder >= 0 && extruder < (int32_t)numExtruders)
			{
				if (gb.Seen('S'))	// S parameter sets the override percentage
				{
					const float extrusionFactor = gb.GetFValue() / 100.0;
					if (extrusionFactor >= 0.0)
					{
						if (segmentsLeft != 0 && !moveBuffer.isFirmwareRetraction)
						{
							moveBuffer.coords[extruder + numAxes] *= extrusionFactor/extrusionFactors[extruder];	// last move not gone, so update it
						}
						extrusionFactors[extruder] = extrusionFactor;
					}
				}
				else
				{
					reply.printf("Extrusion factor override for extruder %d: %.1f%%", extruder, extrusionFactors[extruder] * 100.0);
				}
			}
		}
		break;

		// For case 226, see case 25

	case 280:	// Servos
		if (gb.Seen('P'))
		{
			const int servoIndex = gb.GetIValue();
			Pin servoPin;
			bool invert;
			if (platform->GetFirmwarePin(servoIndex, PinAccess::servo, servoPin, invert))
			{
				if (gb.Seen('I') && gb.GetIValue() > 0)
				{
					invert = !invert;
				}

				if (gb.Seen('S'))
				{
					float angleOrWidth = gb.GetFValue();
					if (angleOrWidth < 0.0)
					{
						// Disable the servo by setting the pulse width to zero
						Platform::WriteAnalog(servoPin, (invert) ? 1.0 : 0.0, ServoRefreshFrequency);
					}
					else
					{
						if (angleOrWidth < MinServoPulseWidth)
						{
							// User gave an angle so convert it to a pulse width in microseconds
							angleOrWidth = (min<float>(angleOrWidth, 180.0) * ((MaxServoPulseWidth - MinServoPulseWidth) / 180.0)) + MinServoPulseWidth;
						}
						else if (angleOrWidth > MaxServoPulseWidth)
						{
							angleOrWidth = MaxServoPulseWidth;
						}
						float pwm = angleOrWidth * (ServoRefreshFrequency/1e6);
						if (invert)
						{
							pwm = 1.0 - pwm;
						}
						Platform::WriteAnalog(servoPin, pwm, ServoRefreshFrequency);
					}
				}
				// We don't currently allow the servo position to be read back
			}
			else
			{
				platform->MessageF(GENERIC_MESSAGE, "Error: Invalid servo index %d in M280 command\n", servoIndex);
			}
		}
		break;

	case 300:	// Beep
		{
			const int ms = (gb.Seen('P')) ? gb.GetIValue() : 1000;			// time in milliseconds
			const int freq = (gb.Seen('S')) ? gb.GetIValue() : 4600;		// 4600Hz produces the loudest sound on a PanelDue
			reprap.Beep(freq, ms);
		}
		break;

	case 301: // Set/report hot end PID values
		SetPidParameters(gb, 1, reply);
		break;

	case 302: // Allow, deny or report cold extrudes
		if (gb.Seen('P'))
		{
			reprap.GetHeat()->AllowColdExtrude(gb.GetIValue() > 0);
		}
		else
		{
			reply.printf("Cold extrusion is %s, use M302 P[1/0] to allow/deny it",
					reprap.GetHeat()->ColdExtrude() ? "allowed" : "denied");
		}
		break;

	case 303: // Run PID tuning
		if (gb.Seen('H'))
		{
			const size_t heater = gb.GetIValue();
			const float temperature = (gb.Seen('S')) ? gb.GetFValue() : 225.0;
			const float maxPwm = (gb.Seen('P')) ? gb.GetFValue() : 0.5;
			if (heater < HEATERS && maxPwm >= 0.1 && maxPwm <= 1.0 && temperature >= 55.0 && temperature <= reprap.GetHeat()->GetTemperatureLimit(heater))
			{
				reprap.GetHeat()->StartAutoTune(heater, temperature, maxPwm, reply);
			}
			else
			{
				reply.printf("Bad parameter in M303 command");
			}
		}
		else
		{
			reprap.GetHeat()->GetAutoTuneStatus(reply);
		}
		break;

	case 304: // Set/report heated bed PID values
		{
			const int8_t bedHeater = reprap.GetHeat()->GetBedHeater();
			if (bedHeater >= 0)
			{
				SetPidParameters(gb, bedHeater, reply);
			}
		}
		break;

	case 305: // Set/report specific heater parameters
		SetHeaterParameters(gb, reply);
		break;

	case 307: // Set heater process model parameters
		if (gb.Seen('H'))
		{
			size_t heater = gb.GetIValue();
			if (heater < HEATERS)
			{
				const FopDt& model = reprap.GetHeat()->GetHeaterModel(heater);
				bool seen = false;
				float gain = model.GetGain(),
					tc = model.GetTimeConstant(),
					td = model.GetDeadTime(),
					maxPwm = model.GetMaxPwm();
				int32_t dontUsePid = model.UsePid() ? 0 : 1;

				gb.TryGetFValue('A', gain, seen);
				gb.TryGetFValue('C', tc, seen);
				gb.TryGetFValue('D', td, seen);
				gb.TryGetIValue('B', dontUsePid, seen);
				gb.TryGetFValue('S', maxPwm, seen);

				if (seen)
				{
					if (!reprap.GetHeat()->SetHeaterModel(heater, gain, tc, td, maxPwm, dontUsePid == 0))
					{
						reply.copy("Error: bad model parameters");
					}
				}
				else if (!model.IsEnabled())
				{
					reply.printf("Heater %u is disabled", heater);
				}
				else
				{
					const char* mode = (!model.UsePid()) ? "bang-bang"
										: (model.ArePidParametersOverridden()) ? "custom PID"
											: "PID";
					reply.printf("Heater %u model: gain %.1f, time constant %.1f, dead time %.1f, max PWM %.2f, mode: %s",
							heater, model.GetGain(), model.GetTimeConstant(), model.GetDeadTime(), model.GetMaxPwm(), mode);
					if (model.UsePid())
					{
						// When reporting the PID parameters, we scale them by 255 for compatibility with older firmware and other firmware
						M301PidParameters params = model.GetM301PidParameters(false);
						reply.catf("\nSetpoint change: P%.1f, I%.3f, D%.1f", params.kP, params.kI, params.kD);
						params = model.GetM301PidParameters(true);
						reply.catf("\nLoad change: P%.1f, I%.3f, D%.1f", params.kP, params.kI, params.kD);
					}
				}
			}
		}
		break;

	case 350: // Set/report microstepping
		{
			// interp is currently an int not a bool, because we use special values of interp to set the chopper control register
			int32_t interp = 0;
			bool dummy;
			gb.TryGetIValue('I', interp, dummy);

			bool seen = false;
			for (size_t axis = 0; axis < numAxes; axis++)
			{
				if (gb.Seen(axisLetters[axis]))
				{
					if (!LockMovementAndWaitForStandstill(gb))
					{
						return false;
					}
					seen = true;
					const int microsteps = gb.GetIValue();
					if (ChangeMicrostepping(axis, microsteps, interp))
					{
						SetAxisNotHomed(axis);
					}
					else
					{
						platform->MessageF(GENERIC_MESSAGE, "Drive %c does not support %dx microstepping%s\n",
												axisLetters[axis], microsteps, (interp) ? " with interpolation" : "");
					}
				}
			}

			if (gb.Seen(extrudeLetter))
			{
				if (!LockMovementAndWaitForStandstill(gb))
				{
					return false;
				}
				seen = true;
				long eVals[MaxExtruders];
				size_t eCount = numExtruders;
				gb.GetLongArray(eVals, eCount);
				for (size_t e = 0; e < eCount; e++)
				{
					if (!ChangeMicrostepping(numAxes + e, (int)eVals[e], interp))
					{
						platform->MessageF(GENERIC_MESSAGE, "Drive E%u does not support %dx microstepping%s\n",
												e, (int)eVals[e], (interp) ? " with interpolation" : "");
					}
				}
			}

			if (!seen)
			{
				reply.copy("Microstepping - ");
				for (size_t axis = 0; axis < numAxes; ++axis)
				{
					bool interp;
					const int microsteps = platform->GetMicrostepping(axis, interp);
					reply.catf("%c:%d%s, ", axisLetters[axis], microsteps, (interp) ? "(on)" : "");
				}
				reply.cat("E");
				for (size_t extruder = 0; extruder < numExtruders; extruder++)
				{
					bool interp;
					const int microsteps = platform->GetMicrostepping(extruder + numAxes, interp);
					reply.catf(":%d%s", microsteps, (interp) ? "(on)" : "");
				}
			}
		}
		break;

	case 374: // Save grid and height map to file
		error = SaveHeightMap(gb, reply);
		break;

	case 375: // Load grid and height map from file and enable compensation
		if (!LockMovementAndWaitForStandstill(gb))
		{
			return false;
		}
		error = LoadHeightMap(gb, reply);
		break;

	case 376: // Set taper height
		{
			HeightMap& heightMap = reprap.GetMove()->AccessBedProbeGrid();
			if (gb.Seen('H'))
			{
				heightMap.SetTaperHeight(gb.GetFValue());
			}
			else if (heightMap.GetTaperHeight() > 0.0)
			{
				reply.printf("Bed compensation taper height is %.1fmm", heightMap.GetTaperHeight());
			}
			else
			{
				reply.copy("Bed compensation is not tapered");
			}
		}
		break;

	case 400: // Wait for current moves to finish
		if (!LockMovementAndWaitForStandstill(gb))
		{
			return false;
		}
		break;

	case 401: // Deploy Z probe
		DoFileMacro(gb, DEPLOYPROBE_G, false);
		break;

	case 402: // Retract Z probe
		DoFileMacro(gb, RETRACTPROBE_G, false);
		break;

	case 404: // Filament width and nozzle diameter
		{
			bool seen = false;

			if (gb.Seen('N'))
			{
				platform->SetFilamentWidth(gb.GetFValue());
				seen = true;
			}
			if (gb.Seen('D'))
			{
				platform->SetNozzleDiameter(gb.GetFValue());
				seen = true;
			}

			if (!seen)
			{
				reply.printf("Filament width: %.2fmm, nozzle diameter: %.2fmm", platform->GetFilamentWidth(), platform->GetNozzleDiameter());
			}
		}
		break;

	case 408: // Get status in JSON format
		{
			int type = gb.Seen('S') ? gb.GetIValue() : 0;
			int seq = gb.Seen('R') ? gb.GetIValue() : -1;

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
					statusResponse = reprap.GetStatusResponse(type - 1, (&gb == auxGCode) ? ResponseSource::AUX : ResponseSource::Generic);
					break;

				case 5:
					statusResponse = reprap.GetConfigResponse();
					break;
			}

			if (statusResponse != nullptr)
			{
				UnlockAll(gb);
				statusResponse->cat('\n');
				HandleReply(gb, false, statusResponse);
				return true;
			}
		}
		break;

	case 500: // Store parameters in EEPROM
		error = WriteConfigOverrideFile(reply, CONFIG_OVERRIDE_G);
		break;

	case 501: // Load parameters from EEPROM
		DoFileMacro(gb, "config-override.g", true);
		break;

	case 502: // Revert to default "factory settings"
		reprap.GetHeat()->ResetHeaterModels();				// in case some heaters have no M307 commands in config.g
		reprap.GetMove()->AccessDeltaParams().Init();		// in case M665 and M666 in config.g don't define all the parameters
		platform->SetZProbeDefaults();
		DoFileMacro(gb, "config.g", true, true);
		break;

	case 503: // List variable settings
		{
			if (!LockFileSystem(gb))
			{
				return false;
			}

			// Need a valid output buffer to continue...
			OutputBuffer *configResponse;
			if (!OutputBuffer::Allocate(configResponse))
			{
				// No buffer available, try again later
				return false;
			}

			// Read the entire file
			FileStore * const f = platform->GetFileStore(platform->GetSysDir(), platform->GetConfigFile(), false);
			if (f == nullptr)
			{
				error = true;
				reply.copy("Configuration file not found!");
			}
			else
			{
				char fileBuffer[FILE_BUFFER_SIZE];
				size_t bytesRead,
					bytesLeftForWriting = OutputBuffer::GetBytesLeft(configResponse);
				while ((bytesRead = f->Read(fileBuffer, FILE_BUFFER_SIZE)) > 0 && bytesLeftForWriting > 0)
				{
					// Don't write more data than we can process
					if (bytesRead < bytesLeftForWriting)
					{
						bytesLeftForWriting -= bytesRead;
					}
					else
					{
						bytesRead = bytesLeftForWriting;
						bytesLeftForWriting = 0;
					}

					// Write it
					configResponse->cat(fileBuffer, bytesRead);
				}
				f->Close();

				UnlockAll(gb);
				HandleReply(gb, false, configResponse);
				return true;
			}
		}
		break;

	case 540: // Set/report MAC address
		if (gb.Seen('P'))
		{
			SetMACAddress(gb);
		}
		else
		{
			const byte* mac = platform->MACAddress();
			reply.printf("MAC: %x:%x:%x:%x:%x:%x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
		}
		break;

	case 550: // Set/report machine name
		if (gb.Seen('P'))
		{
			reprap.SetName(gb.GetString());
		}
		else
		{
			reply.printf("RepRap name: %s", reprap.GetName());
		}
		break;

	case 551: // Set password (no option to report it)
		if (gb.Seen('P'))
		{
			reprap.SetPassword(gb.GetString());
		}
		break;

	case 552: // Enable/Disable network and/or Set/Get IP address
		{
			bool seen = false;
			if (gb.Seen('P'))
			{
				seen = true;
				SetEthernetAddress(gb, code);
			}

			if (gb.Seen('R'))
			{
				reprap.GetNetwork()->SetHttpPort(gb.GetIValue());
				seen = true;
			}

			// Process this one last in case the IP address is changed and the network enabled in the same command
			if (gb.Seen('S')) // Has the user turned the network on or off?
			{
				seen = true;
				if (gb.GetIValue() != 0)
				{
					reprap.GetNetwork()->Enable();
				}
				else
				{
					reprap.GetNetwork()->Disable();
				}
			}

			if (!seen)
			{
				const byte *config_ip = platform->GetIPAddress();
				const byte *actual_ip = reprap.GetNetwork()->GetIPAddress();
				reply.printf("Network is %s, configured IP address: %d.%d.%d.%d, actual IP address: %d.%d.%d.%d, HTTP port: %d",
						reprap.GetNetwork()->IsEnabled() ? "enabled" : "disabled",
						config_ip[0], config_ip[1], config_ip[2], config_ip[3], actual_ip[0], actual_ip[1], actual_ip[2], actual_ip[3],
						reprap.GetNetwork()->GetHttpPort());
			}
		}
		break;

	case 553: // Set/Get netmask
		if (gb.Seen('P'))
		{
			SetEthernetAddress(gb, code);
		}
		else
		{
			const byte *nm = platform->NetMask();
			reply.printf("Net mask: %d.%d.%d.%d ", nm[0], nm[1], nm[2], nm[3]);
		}
		break;

	case 554: // Set/Get gateway
		if (gb.Seen('P'))
		{
			SetEthernetAddress(gb, code);
		}
		else
		{
			const byte *gw = platform->GateWay();
			reply.printf("Gateway: %d.%d.%d.%d ", gw[0], gw[1], gw[2], gw[3]);
		}
		break;

	case 555: // Set/report firmware type to emulate
		if (gb.Seen('P'))
		{
			platform->SetEmulating((Compatibility) gb.GetIValue());
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
		}
		break;

	case 556: // Axis compensation (we support only X, Y, Z)
		if (gb.Seen('S'))
		{
			float value = gb.GetFValue();
			for (size_t axis = 0; axis <= Z_AXIS; axis++)
			{
				if (gb.Seen(axisLetters[axis]))
				{
					reprap.GetMove()->SetAxisCompensation(axis, gb.GetFValue() / value);
				}
			}
		}
		else
		{
			reply.printf("Axis compensations - XY: %.5f, YZ: %.5f, ZX: %.5f",
					reprap.GetMove()->AxisCompensation(X_AXIS), reprap.GetMove()->AxisCompensation(Y_AXIS),
					reprap.GetMove()->AxisCompensation(Z_AXIS));
		}
		break;

	case 557: // Set/report Z probe point coordinates
		if (gb.Seen('P'))
		{
			const int point = gb.GetIValue();
			if (point < 0 || (unsigned int)point >= MaxProbePoints)
			{
				reprap.GetPlatform()->Message(GENERIC_MESSAGE, "Z probe point index out of range.\n");
			}
			else
			{
				bool seen = false;
				if (gb.Seen(axisLetters[X_AXIS]))
				{
					reprap.GetMove()->SetXBedProbePoint(point, gb.GetFValue());
					seen = true;
				}
				if (gb.Seen(axisLetters[Y_AXIS]))
				{
					reprap.GetMove()->SetYBedProbePoint(point, gb.GetFValue());
					seen = true;
				}

				if (!seen)
				{
					reply.printf("Probe point %d - [%.1f, %.1f]", point, reprap.GetMove()->XBedProbePoint(point), reprap.GetMove()->YBedProbePoint(point));
				}
			}
		}
		else
		{
			LockMovement(gb);							// to ensure that probing is not already in progress
			error = DefineGrid(gb, reply);
		}
		break;

	case 558: // Set or report Z probe type and for which axes it is used
		{
			bool seenAxes = false, seenType = false, seenParam = false;
			uint32_t zProbeAxes = platform->GetZProbeAxes();
			for (size_t axis = 0; axis < numAxes; axis++)
			{
				if (gb.Seen(axisLetters[axis]))
				{
					if (gb.GetIValue() > 0)
					{
						zProbeAxes |= (1u << axis);
					}
					else
					{
						zProbeAxes &= ~(1u << axis);
					}
					seenAxes = true;
				}
			}
			if (seenAxes)
			{
				platform->SetZProbeAxes(zProbeAxes);
			}

			// We must get and set the Z probe type first before setting the dive height etc., because different probe types may have different parameters
			if (gb.Seen('P'))		// probe type
			{
				platform->SetZProbeType(gb.GetIValue());
				seenType = true;
			}

			ZProbeParameters params = platform->GetCurrentZProbeParameters();
			gb.TryGetFValue('H', params.diveHeight, seenParam);		// dive height

			if (gb.Seen('F'))		// feed rate i.e. probing speed
			{
				params.probeSpeed = gb.GetFValue() * secondsToMinutes;
				seenParam = true;
			}

			if (gb.Seen('T'))		// travel speed to probe point
			{
				params.travelSpeed = gb.GetFValue() * secondsToMinutes;
				seenParam = true;
			}

			if (gb.Seen('I'))
			{
				params.invertReading = (gb.GetIValue() != 0);
				seenParam = true;
			}

			gb.TryGetFValue('R', params.recoveryTime, seenParam);	// Z probe recovery time
			gb.TryGetFValue('S', params.extraParam, seenParam);		// extra parameter for experimentation

			if (seenParam)
			{
				platform->SetZProbeParameters(platform->GetZProbeType(), params);
			}

			if (!(seenAxes || seenType || seenParam))
			{
				reply.printf("Z Probe type %d, invert %s, dive height %.1fmm, probe speed %dmm/min, travel speed %dmm/min, recovery time %.2f sec",
								platform->GetZProbeType(), (params.invertReading) ? "yes" : "no", params.diveHeight,
								(int)(params.probeSpeed * minutesToSeconds), (int)(params.travelSpeed * minutesToSeconds), params.recoveryTime);
				if (platform->GetZProbeType() == ZProbeTypeDelta)
				{
					reply.catf(", extra parameter %.2f", params.extraParam);
				}
				reply.cat(", used for axes:");
				for (size_t axis = 0; axis < numAxes; axis++)
				{
					if ((zProbeAxes & (1u << axis)) != 0)
					{
						reply.catf(" %c", axisLetters[axis]);
					}
				}
			}
		}
		break;

	case 559: // Upload config.g or another gcode file to put in the sys directory
	{
		const char* str = (gb.Seen('P') ? gb.GetString() : platform->GetConfigFile());
		const bool ok = OpenFileToWrite(gb, platform->GetSysDir(), str);
		if (ok)
		{
			reply.printf("Writing to file: %s", str);
		}
		else
		{
			reply.printf("Can't open file %s for writing.", str);
			error = true;
		}
	}
		break;

	case 560: // Upload reprap.htm or another web interface file
	{
		const char* str = (gb.Seen('P') ? gb.GetString() : INDEX_PAGE_FILE);
		const bool ok = OpenFileToWrite(gb, platform->GetWebDir(), str);
		if (ok)
		{
			reply.printf("Writing to file: %s", str);
		}
		else
		{
			reply.printf("Can't open file %s for writing.", str);
			error = true;
		}
	}
		break;

	case 561: // Set identity transform (also clears bed probe grid)
		reprap.GetMove()->SetIdentityTransform();
		break;

	case 562: // Reset temperature fault - use with great caution
		if (gb.Seen('P'))
		{
			const int heater = gb.GetIValue();
			if (heater >= 0 && heater < HEATERS)
			{
				reprap.ClearTemperatureFault(heater);
			}
			else
			{
				reply.copy("Invalid heater number.\n");
				error = true;
			}
		}
		break;

	case 563: // Define tool
		ManageTool(gb, reply);
		break;

	case 564: // Think outside the box?
		if (gb.Seen('S'))
		{
			limitAxes = (gb.GetIValue() != 0);
		}
		else
		{
			reply.printf("Movement outside the bed is %spermitted", (limitAxes) ? "not " : "");
		}
		break;

	case 566: // Set/print maximum jerk speeds
	{
		bool seen = false;
		for (size_t axis = 0; axis < numAxes; axis++)
		{
			if (gb.Seen(axisLetters[axis]))
			{
				platform->SetInstantDv(axis, gb.GetFValue() * distanceScale * secondsToMinutes); // G Code feedrates are in mm/minute; we need mm/sec
				seen = true;
			}
		}

		if (gb.Seen(extrudeLetter))
		{
			seen = true;
			float eVals[MaxExtruders];
			size_t eCount = numExtruders;
			gb.GetFloatArray(eVals, eCount, true);
			for (size_t e = 0; e < eCount; e++)
			{
				platform->SetInstantDv(numAxes + e, eVals[e] * distanceScale * secondsToMinutes);
			}
		}
		else if (!seen)
		{
			reply.copy("Maximum jerk rates: ");
			for (size_t axis = 0; axis < numAxes; ++axis)
			{
				reply.catf("%c: %.1f, ", axisLetters[axis], platform->ConfiguredInstantDv(axis) / (distanceScale * secondsToMinutes));
			}
			reply.cat("E:");
			char sep = ' ';
			for (size_t extruder = 0; extruder < numExtruders; extruder++)
			{
				reply.catf("%c%.1f", sep, platform->ConfiguredInstantDv(extruder + numAxes) / (distanceScale * secondsToMinutes));
				sep = ':';
			}
		}
	}
		break;

	case 567: // Set/report tool mix ratios
		if (gb.Seen('P'))
		{
			int8_t tNumber = gb.GetIValue();
			Tool* const tool = reprap.GetTool(tNumber);
			if (tool != NULL)
			{
				if (gb.Seen(extrudeLetter))
				{
					float eVals[MaxExtruders];
					size_t eCount = tool->DriveCount();
					gb.GetFloatArray(eVals, eCount, false);
					if (eCount != tool->DriveCount())
					{
						reply.printf("Setting mix ratios - wrong number of E drives: %s", gb.Buffer());
					}
					else
					{
						tool->DefineMix(eVals);
					}
				}
				else
				{
					reply.printf("Tool %d mix ratios:", tNumber);
					char sep = ' ';
					for (size_t drive = 0; drive < tool->DriveCount(); drive++)
					{
						reply.catf("%c%.3f", sep, tool->GetMix()[drive]);
						sep = ':';
					}
				}
			}
		}
		break;

	case 568: // Turn on/off automatic tool mixing
		if (gb.Seen('P'))
		{
			Tool* const tool = reprap.GetTool(gb.GetIValue());
			if (tool != NULL)
			{
				if (gb.Seen('S'))
				{
					tool->SetMixing(gb.GetIValue() != 0);
				}
				else
				{
					reply.printf("Tool %d mixing is %s", tool->Number(), (tool->GetMixing()) ? "enabled" : "disabled");
				}
			}
		}
		break;

	case 569: // Set/report axis direction
		if (gb.Seen('P'))
		{
			const size_t drive = gb.GetIValue();
			if (drive < DRIVES)
			{
				bool seen = false;
				if (gb.Seen('S'))
				{
					if (!LockMovementAndWaitForStandstill(gb))
					{
						return false;
					}
					platform->SetDirectionValue(drive, gb.GetIValue() != 0);
					seen = true;
				}
				if (gb.Seen('R'))
				{
					if (!LockMovementAndWaitForStandstill(gb))
					{
						return false;
					}
					platform->SetEnableValue(drive, gb.GetIValue() != 0);
					seen = true;
				}
				if (gb.Seen('T'))
				{
					platform->SetDriverStepTiming(drive, gb.GetFValue());
					seen = true;
				}
				bool badParameter = false;
				for (size_t axis = 0; axis < numAxes; ++axis)
				{
					if (gb.Seen(axisLetters[axis]))
					{
						badParameter = true;
					}
				}
				if (gb.Seen(extrudeLetter))
				{
					badParameter = true;
				}
				if (badParameter)
				{
					platform->Message(GENERIC_MESSAGE, "Error: M569 no longer accepts XYZE parameters; use M584 instead\n");
				}
				else if (!seen)
				{
					reply.printf("A %d sends drive %u forwards, a %d enables it, and the minimum pulse width is %.1f microseconds",
								(int)platform->GetDirectionValue(drive), drive,
								(int)platform->GetEnableValue(drive),
								platform->GetDriverStepTiming(drive));
				}
			}
		}
		break;

	case 570: // Set/report heater timeout
		if (gb.Seen('H'))
		{
			const size_t heater = gb.GetIValue();
			bool seen = false;
			if (heater < HEATERS)
			{
				float maxTempExcursion, maxFaultTime;
				reprap.GetHeat()->GetHeaterProtection(heater, maxTempExcursion, maxFaultTime);
				gb.TryGetFValue('P', maxFaultTime, seen);
				gb.TryGetFValue('T', maxTempExcursion, seen);
				if (seen)
				{
					reprap.GetHeat()->SetHeaterProtection(heater, maxTempExcursion, maxFaultTime);
				}
				else
				{
					reply.printf("Heater %u allowed excursion %.1fC, fault trigger time %.1f seconds", heater, maxTempExcursion, maxFaultTime);
				}
			}
		}
		else if (gb.Seen('S'))
		{
			reply.copy("M570 S parameter is no longer required or supported");
		}
		break;

	case 571: // Set output on extrude
		{
			bool seen = false;
			if (gb.Seen('P'))
			{
				const int pwmPin = gb.GetIValue();
				if (!platform->SetExtrusionAncilliaryPwmPin(pwmPin))
				{
					reply.printf("Logical pin %d is already in use or not available for use by M571", pwmPin);
					break;			// don't process 'S' parameter if the pin was wrong
				}
				seen = true;
			}
			if (gb.Seen('F'))
			{
				platform->SetExtrusionAncilliaryPwmFrequency(gb.GetFValue());
			}
			if (gb.Seen('S'))
			{
				platform->SetExtrusionAncilliaryPwmValue(gb.GetFValue());
				seen = true;
			}
			if (!seen)
			{
				reply.printf("Extrusion ancillary PWM %.3f at %.1fHz on pin %u",
								platform->GetExtrusionAncilliaryPwmValue(),
								platform->GetExtrusionAncilliaryPwmFrequency(),
								platform->GetExtrusionAncilliaryPwmPin());
			}
		}
		break;

	case 572: // Set/report elastic compensation
		if (gb.Seen('D'))
		{
			// New usage: specify the extruder drive using the D parameter
			const size_t extruder = gb.GetIValue();
			if (gb.Seen('S'))
			{
				platform->SetPressureAdvance(extruder, gb.GetFValue());
			}
			else
			{
				reply.printf("Pressure advance for extruder %u is %.3f seconds", extruder, platform->GetPressureAdvance(extruder));
			}
		}
		break;

	case 573: // Report heater average PWM
		if (gb.Seen('P'))
		{
			const int heater = gb.GetIValue();
			if (heater >= 0 && heater < HEATERS)
			{
				reply.printf("Average heater %d PWM: %.3f", heater, reprap.GetHeat()->GetAveragePWM(heater));
			}
		}
		break;

	case 574: // Set endstop configuration
		{
			bool seen = false;
			const bool logicLevel = (gb.Seen('S')) ? (gb.GetIValue() != 0) : true;
			for (size_t axis = 0; axis < numAxes; ++axis)
			{
				if (gb.Seen(axisLetters[axis]))
				{
					const int ival = gb.GetIValue();
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
				EndStopType config;
				bool logic;
				for (size_t axis = 0; axis < numAxes; ++axis)
				{
					platform->GetEndStopConfiguration(axis, config, logic);
					reply.catf(" %c %s (active %s),", axisLetters[axis],
							(config == EndStopType::highEndStop) ? "high end" : (config == EndStopType::lowEndStop) ? "low end" : "none",
							(config == EndStopType::noEndStop) ? "" : (logic) ? "high" : "low");
				}
			}
		}
		break;

	case 575: // Set communications parameters
		if (gb.Seen('P'))
		{
			size_t chan = gb.GetIValue();
			if (chan < NUM_SERIAL_CHANNELS)
			{
				bool seen = false;
				if (gb.Seen('B'))
				{
					platform->SetBaudRate(chan, gb.GetIValue());
					seen = true;
				}
				if (gb.Seen('S'))
				{
					uint32_t val = gb.GetIValue();
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
					reply.printf("Channel %d: baud rate %d, %s checksum", chan, platform->GetBaudRate(chan),
							(cp & 1) ? "requires" : "does not require");
				}
			}
		}
		break;

	case 577: // Wait until endstop is triggered
		if (gb.Seen('S'))
		{
			// Determine trigger type
			EndStopHit triggerCondition;
			switch (gb.GetIValue())
			{
				case 1:
					triggerCondition = EndStopHit::lowHit;
					break;
				case 2:
					triggerCondition = EndStopHit::highHit;
					break;
				case 3:
					triggerCondition = EndStopHit::lowNear;
					break;
				default:
					triggerCondition = EndStopHit::noStop;
					break;
			}

			// Axis endstops
			for (size_t axis=0; axis < numAxes; axis++)
			{
				if (gb.Seen(axisLetters[axis]))
				{
					if (platform->Stopped(axis) != triggerCondition)
					{
						result = false;
						break;
					}
				}
			}

			// Extruder drives
			size_t eDriveCount = MaxExtruders;
			long eDrives[MaxExtruders];
			if (gb.Seen(extrudeLetter))
			{
				gb.GetLongArray(eDrives, eDriveCount);
				for(size_t extruder = 0; extruder < eDriveCount; extruder++)
				{
					const size_t eDrive = eDrives[extruder];
					if (eDrive >= MaxExtruders)
					{
						reply.copy("Invalid extruder drive specified!");
						error = result = true;
						break;
					}

					if (platform->Stopped(eDrive + E0_AXIS) != triggerCondition)
					{
						result = false;
						break;
					}
				}
			}
		}
		break;

#if SUPPORT_INKJET
	case 578: // Fire Inkjet bits
		if (!AllMovesAreFinishedAndMoveBufferIsLoaded())
		{
			return false;
		}

		if (gb.Seen('S')) // Need to handle the 'P' parameter too; see http://reprap.org/wiki/G-code#M578:_Fire_inkjet_bits
		{
			platform->Inkjet(gb.GetIValue());
		}
		break;
#endif

	case 579: // Scale Cartesian axes (mostly for Delta configurations)
		{
			bool seen = false;
			for (size_t axis = 0; axis < numAxes; axis++)
			{
				gb.TryGetFValue(axisLetters[axis], axisScaleFactors[axis], seen);
			}

			if (!seen)
			{
				char sep = ':';
				reply.copy("Axis scale factors");
				for(size_t axis = 0; axis < numAxes; axis++)
				{
					reply.catf("%c %c: %.3f", sep, axisLetters[axis], axisScaleFactors[axis]);
					sep = ',';
				}
			}
		}
		break;

#if SUPPORT_ROLAND
	case 580: // (De)Select Roland mill
		if (gb.Seen('R'))
		{
			if (gb.GetIValue())
			{
				reprap.GetRoland()->Activate();
				if (gb.Seen('P'))
				{
					result = reprap.GetRoland()->RawWrite(gb.GetString());
				}
			}
			else
			{
				result = reprap.GetRoland()->Deactivate();
			}
		}
		else
		{
			reply.printf("Roland is %s.", reprap.GetRoland()->Active() ? "active" : "inactive");
		}
		break;
#endif

	case 581: // Configure external trigger
	case 582: // Check external trigger
		if (gb.Seen('T'))
		{
			unsigned int triggerNumber = gb.GetIValue();
			if (triggerNumber < MaxTriggers)
			{
				if (code == 582)
				{
					uint32_t states = platform->GetAllEndstopStates();
					if ((triggers[triggerNumber].rising & states) != 0 || (triggers[triggerNumber].falling & ~states) != 0)
					{
						triggersPending |= (1u << triggerNumber);
					}
				}
				else
				{
					bool seen = false;
					if (gb.Seen('C'))
					{
						seen = true;
						triggers[triggerNumber].condition = gb.GetIValue();
					}
					else if (triggers[triggerNumber].IsUnused())
					{
						triggers[triggerNumber].condition = 0;		// this is a new trigger, so set no condition
					}
					if (gb.Seen('S'))
					{
						seen = true;
						int sval = gb.GetIValue();
						TriggerMask triggerMask = 0;
						for (size_t axis = 0; axis < numAxes; ++axis)
						{
							if (gb.Seen(axisLetters[axis]))
							{
								triggerMask |= (1u << axis);
							}
						}
						if (gb.Seen(extrudeLetter))
						{
							long eStops[MaxExtruders];
							size_t numEntries = MaxExtruders;
							gb.GetLongArray(eStops, numEntries);
							for (size_t i = 0; i < numEntries; ++i)
							{
								if (eStops[i] >= 0 && (unsigned long)eStops[i] < MaxExtruders)
								{
									triggerMask |= (1u << (eStops[i] + E0_AXIS));
								}
							}
						}
						switch(sval)
						{
						case -1:
							if (triggerMask == 0)
							{
								triggers[triggerNumber].rising = triggers[triggerNumber].falling = 0;
							}
							else
							{
								triggers[triggerNumber].rising &= (~triggerMask);
								triggers[triggerNumber].falling &= (~triggerMask);
							}
							break;

						case 0:
							triggers[triggerNumber].falling |= triggerMask;
							break;

						case 1:
							triggers[triggerNumber].rising |= triggerMask;
							break;

						default:
							platform->Message(GENERIC_MESSAGE, "Bad S parameter in M581 command\n");
						}
					}
					if (!seen)
					{
						reply.printf("Trigger %u fires on a rising edge on ", triggerNumber);
						ListTriggers(reply, triggers[triggerNumber].rising);
						reply.cat(" or a falling edge on ");
						ListTriggers(reply, triggers[triggerNumber].falling);
						reply.cat(" endstop inputs");
						if (triggers[triggerNumber].condition == 1)
						{
							reply.cat(" when printing from SD card");
						}
					}
				}
			}
			else
			{
				platform->Message(GENERIC_MESSAGE, "Trigger number out of range\n");
			}
		}
		break;

	case 584: // Set axis/extruder to stepper driver(s) mapping
		if (!LockMovementAndWaitForStandstill(gb))	// we also rely on this to retrieve the current motor positions to moveBuffer
		{
			return false;
		}
		{
			bool seen = false, badDrive = false;
			for (size_t drive = 0; drive < MAX_AXES; ++drive)
			{
				if (gb.Seen(axisLetters[drive]))
				{
					seen = true;
					size_t numValues = MaxDriversPerAxis;
					long drivers[MaxDriversPerAxis];
					gb.GetLongArray(drivers, numValues);

					// Check all the driver numbers are in range
					bool badAxis = false;
					AxisDriversConfig config;
					config.numDrivers = numValues;
					for (size_t i = 0; i < numValues; ++i)
					{
						if ((unsigned long)drivers[i] >= DRIVES)
						{
							badAxis = true;
						}
						else
						{
							config.driverNumbers[i] = (uint8_t)drivers[i];
						}
					}
					if (badAxis)
					{
						badDrive = true;
					}
					else
					{
						while (numAxes <= drive)
						{
							moveBuffer.coords[numAxes] = 0.0;		// user has defined a new axis, so set its position
							++numAxes;
						}
						SetPositions(moveBuffer.coords);			// tell the Move system where any new axes are
						platform->SetAxisDriversConfig(drive, config);
						if (numAxes + numExtruders > DRIVES)
						{
							numExtruders = DRIVES - numAxes;		// if we added axes, we may have fewer extruders now
						}
					}
				}
			}

			if (gb.Seen(extrudeLetter))
			{
				seen = true;
				size_t numValues = DRIVES - numAxes;
				long drivers[MaxExtruders];
				gb.GetLongArray(drivers, numValues);
				numExtruders = numValues;
				for (size_t i = 0; i < numValues; ++i)
				{
					if ((unsigned long)drivers[i] >= DRIVES)
					{
						badDrive = true;
					}
					else
					{
						platform->SetExtruderDriver(i, (uint8_t)drivers[i]);
					}
				}
			}

			if (badDrive)
			{
				platform->Message(GENERIC_MESSAGE, "Error: invalid drive number in M584 command\n");
			}
			else if (!seen)
			{
				reply.copy("Driver assignments:");
				for (size_t drive = 0; drive < numAxes; ++ drive)
				{
					reply.cat(' ');
					const AxisDriversConfig& axisConfig = platform->GetAxisDriversConfig(drive);
					char c = axisLetters[drive];
					for (size_t i = 0; i < axisConfig.numDrivers; ++i)
					{
						reply.catf("%c%u", c, axisConfig.driverNumbers[i]);
						c = ':';
					}
				}
				reply.cat(' ');
				char c = extrudeLetter;
				for (size_t extruder = 0; extruder < numExtruders; ++extruder)
				{
					reply.catf("%c%u", c, platform->GetExtruderDriver(extruder));
					c = ':';
				}
			}
		}
		break;

	case 665: // Set delta configuration
		if (!LockMovementAndWaitForStandstill(gb))
		{
			return false;
		}
		{
			float positionNow[DRIVES];
			Move *move = reprap.GetMove();
			move->GetCurrentUserPosition(positionNow, 0, reprap.GetCurrentXAxes()); // get the current position, we may need it later
			DeltaParameters& params = move->AccessDeltaParams();
			const bool wasInDeltaMode = params.IsDeltaMode();						// remember whether we were in delta mode
			bool seen = false;

			if (gb.Seen('L'))
			{
				params.SetDiagonal(gb.GetFValue());
				seen = true;
			}
			if (gb.Seen('R'))
			{
				params.SetRadius(gb.GetFValue());
				seen = true;
			}
			if (gb.Seen('B'))
			{
				params.SetPrintRadius(gb.GetFValue());
				seen = true;
			}
			if (gb.Seen('X'))
			{
				// X tower position correction
				params.SetXCorrection(gb.GetFValue());
				seen = true;
			}
			if (gb.Seen('Y'))
			{
				// Y tower position correction
				params.SetYCorrection(gb.GetFValue());
				seen = true;
			}
			if (gb.Seen('Z'))
			{
				// Y tower position correction
				params.SetZCorrection(gb.GetFValue());
				seen = true;
			}

			// The homed height must be done last, because it gets recalculated when some of the other factors are changed
			if (gb.Seen('H'))
			{
				params.SetHomedHeight(gb.GetFValue());
				seen = true;
			}

			if (seen)
			{
				move->SetCoreXYMode(0);		// CoreXYMode needs to be zero when executing special moves on a delta

				// If we have changed between Cartesian and Delta mode, we need to reset the motor coordinates to agree with the XYZ coordinates.
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
					reply.printf("Diagonal %.3f, delta radius %.3f, homed height %.3f, bed radius %.1f"
								 ", X %.3f" DEGREE_SYMBOL ", Y %.3f" DEGREE_SYMBOL ", Z %.3f" DEGREE_SYMBOL,
								 	 params.GetDiagonal(), params.GetRadius(),
								 	 params.GetHomedHeight(), params.GetPrintRadius(),
								 	 params.GetXCorrection(), params.GetYCorrection(), params.GetZCorrection());
				}
				else
				{
					reply.printf("Printer is not in delta mode");
				}
			}
		}
		break;

	case 666: // Set delta endstop adjustments
		if (!LockMovementAndWaitForStandstill(gb))
		{
			return false;
		}
		{
			DeltaParameters& params = reprap.GetMove()->AccessDeltaParams();
			bool seen = false;
			if (gb.Seen('X'))
			{
				params.SetEndstopAdjustment(X_AXIS, gb.GetFValue());
				seen = true;
			}
			if (gb.Seen('Y'))
			{
				params.SetEndstopAdjustment(Y_AXIS, gb.GetFValue());
				seen = true;
			}
			if (gb.Seen('Z'))
			{
				params.SetEndstopAdjustment(Z_AXIS, gb.GetFValue());
				seen = true;
			}
			if (gb.Seen('A'))
			{
				params.SetXTilt(gb.GetFValue() * 0.01);
				seen = true;
			}
			if (gb.Seen('B'))
			{
				params.SetYTilt(gb.GetFValue() * 0.01);
				seen = true;
			}

			if (seen)
			{
				SetAllAxesNotHomed();
			}
			else
			{
				reply.printf("Endstop adjustments X%.2f Y%.2f Z%.2f, tilt X%.2f%% Y%.2f%%",
						params.GetEndstopAdjustment(X_AXIS), params.GetEndstopAdjustment(Y_AXIS), params.GetEndstopAdjustment(Z_AXIS),
						params.GetXTilt() * 100.0, params.GetYTilt() * 100.0);
			}
		}
		break;

	case 667: // Set CoreXY mode
		if (!LockMovementAndWaitForStandstill(gb))
		{
			return false;
		}
		{
			Move* const move = reprap.GetMove();
			bool seen = false;
			float positionNow[DRIVES];
			move->GetCurrentUserPosition(positionNow, 0, reprap.GetCurrentXAxes());	// get the current position, we may need it later
			if (gb.Seen('S'))
			{
				move->SetCoreXYMode(gb.GetIValue());
				seen = true;
			}
			for (size_t axis = 0; axis < numAxes; ++axis)
			{
				if (gb.Seen(axisLetters[axis]))
				{
					move->SetCoreAxisFactor(axis, gb.GetFValue());
					seen = true;
				}
			}

			if (seen)
			{
				SetPositions(positionNow);
				SetAllAxesNotHomed();
			}
			else
			{
				reply.printf("Printer mode is %s with axis factors", move->GetGeometryString());
				for (size_t axis = 0; axis < numAxes; ++axis)
				{
					reply.catf(" %c:%f", axisLetters[axis], move->GetCoreAxisFactor(axis));
				}
			}
		}
		break;

	case 905: // Set current RTC date and time
		{
			const time_t now = platform->GetDateTime();
			struct tm * const timeInfo = gmtime(&now);
			bool seen = false;

			if (gb.Seen('P'))
			{
				// Set date
				const char * const dateString = gb.GetString();
				if (strptime(dateString, "%Y-%m-%d", timeInfo) != nullptr)
				{
					if (!platform->SetDate(mktime(timeInfo)))
					{
						reply.copy("Could not set date");
						error = true;
						break;
					}
				}
				else
				{
					reply.copy("Invalid date format");
					error = true;
					break;
				}

				seen = true;
			}

			if (gb.Seen('S'))
			{
				// Set time
				const char * const timeString = gb.GetString();
				if (strptime(timeString, "%H:%M:%S", timeInfo) != nullptr)
				{
					if (!platform->SetTime(mktime(timeInfo)))
					{
						reply.copy("Could not set time");
						error = true;
						break;
					}
				}
				else
				{
					reply.copy("Invalid time format");
					error = true;
					break;
				}
				seen = true;
			}

			// TODO: Add correction parameters for SAM4E

			if (!seen)
			{
				// Report current date and time
				reply.printf("Current date and time: %04u-%02u-%02u %02u:%02u:%02u",
						timeInfo->tm_year + 1900, timeInfo->tm_mon + 1, timeInfo->tm_mday,
						timeInfo->tm_hour, timeInfo->tm_min, timeInfo->tm_sec);

				if (!platform->IsDateTimeSet())
				{
					reply.cat("\nWarning: RTC has not been configured yet!");
				}
			}
		}
		break;

	case 906: // Set/report Motor currents
	case 913: // Set/report motor current percent
		{
			bool seen = false;
			for (size_t axis = 0; axis < numAxes; axis++)
			{
				if (gb.Seen(axisLetters[axis]))
				{
					if (!LockMovementAndWaitForStandstill(gb))
					{
						return false;
					}
					platform->SetMotorCurrent(axis, gb.GetFValue(), code == 913);
					seen = true;
				}
			}

			if (gb.Seen(extrudeLetter))
			{
				if (!LockMovementAndWaitForStandstill(gb))
				{
					return false;
				}

				float eVals[MaxExtruders];
				size_t eCount = numExtruders;
				gb.GetFloatArray(eVals, eCount, true);
				// 2014-09-29 DC42: we no longer insist that the user supplies values for all possible extruder drives
				for (size_t e = 0; e < eCount; e++)
				{
					platform->SetMotorCurrent(numAxes + e, eVals[e], code == 913);
				}
				seen = true;
			}

			if (code == 906 && gb.Seen('I'))
			{
				const float idleFactor = gb.GetFValue();
				if (idleFactor >= 0 && idleFactor <= 100.0)
				{
					platform->SetIdleCurrentFactor(idleFactor/100.0);
					seen = true;
				}
			}

			if (!seen)
			{
				reply.copy((code == 913) ? "Motor current % of normal - " : "Motor current (mA) - ");
				for (size_t axis = 0; axis < numAxes; ++axis)
				{
					reply.catf("%c:%d, ", axisLetters[axis], (int)platform->GetMotorCurrent(axis, code == 913));
				}
				reply.cat("E");
				for (size_t extruder = 0; extruder < numExtruders; extruder++)
				{
					reply.catf(":%d", (int)platform->GetMotorCurrent(extruder + numAxes, code == 913));
				}
				if (code == 906)
				{
					reply.catf(", idle factor %d%%", (int)(platform->GetIdleCurrentFactor() * 100.0));
				}
			}
		}
		break;

	case 911: // Set power monitor threshold voltages
		reply.printf("M911 not implemented yet");
		break;

	case 912: // Set electronics temperature monitor adjustment
		// Currently we ignore the P parameter (i.e. temperature measurement channel)
		if (gb.Seen('S'))
		{
			platform->SetMcuTemperatureAdjust(gb.GetFValue());
		}
		else
		{
			reply.printf("MCU temperature calibration adjustment is %.1f" DEGREE_SYMBOL "C", platform->GetMcuTemperatureAdjust());
		}
		break;

	// For case 913, see 906

	case 997: // Perform firmware update
		if (!LockMovementAndWaitForStandstill(gb))
		{
			return false;
		}
		reprap.GetHeat()->SwitchOffAll();					// turn all heaters off because the main loop may get suspended
		DisableDrives();									// all motors off

		if (firmwareUpdateModuleMap == 0)					// have we worked out which modules to update?
		{
			// Find out which modules we have been asked to update
			if (gb.Seen('S'))
			{
				long modulesToUpdate[3];
				size_t numUpdateModules = ARRAY_SIZE(modulesToUpdate);
				gb.GetLongArray(modulesToUpdate, numUpdateModules);
				for (size_t i = 0; i < numUpdateModules; ++i)
				{
					long t = modulesToUpdate[i];
					if (t < 0 || (unsigned long)t >= NumFirmwareUpdateModules)
					{
						platform->MessageF(GENERIC_MESSAGE, "Invalid module number '%ld'\n", t);
						firmwareUpdateModuleMap = 0;
						break;
					}
					firmwareUpdateModuleMap |= (1u << (unsigned int)t);
				}
			}
			else
			{
				firmwareUpdateModuleMap = (1u << 0);			// no modules specified, so update module 0 to match old behaviour
			}

			if (firmwareUpdateModuleMap == 0)
			{
				break;										// nothing to update
			}

			// Check prerequisites of all modules to be updated, if any are not met then don't update any of them
#ifdef DUET_NG
			if (!FirmwareUpdater::CheckFirmwareUpdatePrerequisites(firmwareUpdateModuleMap))
			{
				firmwareUpdateModuleMap = 0;
				break;
			}
#endif
			if ((firmwareUpdateModuleMap & 1) != 0 && !platform->CheckFirmwareUpdatePrerequisites())
			{
				firmwareUpdateModuleMap = 0;
				break;
			}
		}

		// If we get here then we have the module map, and all prerequisites are satisfied
		isFlashing = true;					// this tells the web interface and PanelDue that we are about to flash firmware
		if (!DoDwellTime(1.0))				// wait a second so all HTTP clients and PanelDue are notified
		{
			return false;
		}

		gb.SetState(GCodeState::flashing1);
		break;

	case 998:
		// The input handling code replaces the gcode by this when it detects a checksum error.
		// Since we have no way of asking for the line to be re-sent, just report an error.
		if (gb.Seen('P'))
		{
			const int val = gb.GetIValue();
			if (val != 0)
			{
				reply.printf("Checksum error on line %d", val);
			}
		}
		break;

	case 999:
		result = DoDwellTime(0.5);			// wait half a second to allow the response to be sent back to the web server, otherwise it may retry
		if (result)
		{
			reprap.EmergencyStop();			// this disables heaters and drives - Duet WiFi pre-production boards need drives disabled here
			uint16_t reason = (gb.Seen('P') && StringStartsWith(gb.GetString(), "ERASE"))
											? (uint16_t)SoftwareResetReason::erase
											: (uint16_t)SoftwareResetReason::user;
			platform->SoftwareReset(reason);			// doesn't return
		}
		break;

	default:
		error = true;
		reply.printf("unsupported command: %s", gb.Buffer());
	}

	if (result && gb.GetState() == GCodeState::normal)
	{
		UnlockAll(gb);
		HandleReply(gb, error, reply.Pointer());
	}
	return result;
}

bool GCodes::HandleTcode(GCodeBuffer& gb, StringRef& reply)
{
	if (gb.MachineState().runningM502)
	{
		return true;			// when running M502 we don't execute T commands
	}

	if (!LockMovementAndWaitForStandstill(gb))
	{
		return false;
	}

	newToolNumber = gb.GetIValue();
	newToolNumber += gb.GetToolNumberAdjust();

	// TODO for the tool change restore point to be useful, we should undo any X axis mapping and remove any tool offsets
	for (size_t drive = 0; drive < DRIVES; ++drive)
	{
		toolChangeRestorePoint.moveCoords[drive] = moveBuffer.coords[drive];
	}
	toolChangeRestorePoint.feedRate = gb.MachineState().feedrate;

	if (simulationMode == 0)						// we don't yet simulate any T codes
	{
		const Tool * const oldTool = reprap.GetCurrentTool();
		// If old and new are the same we no longer follow the sequence. User can deselect and then reselect the tool if he wants the macros run.
		if (oldTool == nullptr || oldTool->Number() != newToolNumber)
		{
			StartToolChange(gb, false);
			return true;							// proceeding with state machine, so don't unlock or send a reply
		}
	}

	// If we get here, we have finished
	UnlockAll(gb);
	HandleReply(gb, false, "");
	return true;
}

// End
