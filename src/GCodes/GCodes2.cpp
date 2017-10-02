/*
 * GCodes2.cpp
 *
 *  Created on: 3 Dec 2016
 *      Author: David
 *
 *  This file contains the code to see what G, M or T command we have and start processing it.
 */

#include "GCodes.h"

#include "GCodeBuffer.h"
#include "GCodeQueue.h"
#include "Heating/Heat.h"
#include "Movement/Move.h"
#include "Network.h"
#include "Scanner.h"
#include "PrintMonitor.h"
#include "RepRap.h"
#include "Tools/Tool.h"
#include "FilamentSensors/FilamentSensor.h"
#include "Version.h"

#if SUPPORT_IOBITS
# include "PortControl.h"
#endif

#ifdef DUET_NG
# include "FirmwareUpdater.h"
#endif

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

	// Can we queue this code?
	if (gb.CanQueueCodes() && codeQueue->QueueCode(gb, segmentsLeft))
	{
		HandleReply(gb, false, "");
		return true;
	}

	// G29 string parameters may contain the letter M, and various M-code string parameters may contain the letter G.
	// So we now look for the first G, M or T in the command.
	switch (gb.GetCommandLetter())
	{
	case 'G':
		return HandleGcode(gb, reply);
	case 'M':
		return HandleMcode(gb, reply);
	case 'T':
		return HandleTcode(gb, reply);
	default:
		// An invalid command gets discarded
		HandleReply(gb, false, "");
		return true;
	}
}

bool GCodes::HandleGcode(GCodeBuffer& gb, StringRef& reply)
{
	GCodeResult result = GCodeResult::ok;
	const int code = gb.GetIValue();
	if (simulationMode != 0 && code > 4 && code != 10 && code != 11 && code != 20 && code != 21 && code != 90 && code != 91 && code != 92)
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
		if (segmentsLeft != 0)
		{
			return false;
		}
		if (DoStraightMove(gb, reply, code == 1))
		{
			result = GCodeResult::error;
		}
		break;

	case 2: // Clockwise arc
	case 3: // Anti clockwise arc
		// We only support X and Y axes in these (and optionally Z for corkscrew moves), but you can map them to other axes in the tool definitions
		if (!LockMovement(gb))
		{
			return false;
		}
		if (segmentsLeft != 0)
		{
			return false;
		}
		if (DoArcMove(gb, code == 2))
		{
			reply.copy("Invalid G2 or G3 command");
			result = GCodeResult::error;
		}
		break;

	case 4: // Dwell
		result = DoDwell(gb);
		break;

	case 10: // Set/report offsets and temperatures, or retract
		{
			bool modifyingTool = gb.Seen('P') || gb.Seen('R') || gb.Seen('S');
			for (size_t axis = 0; axis < numVisibleAxes; ++axis)
			{
				modifyingTool |= gb.Seen(axisLetters[axis]);
			}

			if (modifyingTool)
			{
				if (simulationMode != 0)
				{
					break;
				}
				result = SetOrReportOffsets(gb, reply);
			}
			else
			{
				result = RetractFilament(gb, true);
			}
		}
		break;

	case 11: // Un-retract
		result = RetractFilament(gb, false);
		break;

	case 20: // Inches (which century are we living in, here?)
		distanceScale = InchToMm;
		break;

	case 21: // mm
		distanceScale = 1.0;
		break;

	case 28: // Home
		result = DoHome(gb, reply);
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
				result = ProbeGrid(gb, reply);
				break;

			case 1:		// load height map file
				result = GetGCodeResultFromError(LoadHeightMap(gb, reply));
				break;

			default:	// clear height map
				reprap.GetMove().SetIdentityTransform();
				break;
			}
		}
		break;

	case 30: // Z probe/manually set at a position and set that as point P
		if (!LockMovementAndWaitForStandstill(gb))
		{
			return false;
		}

		if ((reprap.GetMove().GetKinematics().AxesToHomeBeforeProbing() & ~axesHomed) != 0)
		{
			reply.copy("Insufficient axes homed for bed probing");
			result = GCodeResult::error;
		}
		else
		{
			result = ExecuteG30(gb, reply);
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

		// We need to unlock the movement system here in case there is no Z probe and we are doing manual probing.
		// Otherwise, even though the bed probing code calls UnlockAll when doing a manual bed probe, the movement system
		// remains locked because the current MachineState object already held the lock when the macro file was started,
		// which means that no gcode source other than the one that executed G32 is allowed to jog the Z axis.
		UnlockAll(gb);

		DoFileMacro(gb, BED_EQUATION_G, true);	// Try to execute bed.g
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
		reply.printf("invalid G Code: %s", gb.Buffer());
		result = GCodeResult::error;
	}

	if (result == GCodeResult::notFinished)
	{
		return false;
	}

	if (result == GCodeResult::notSupportedInCurrentMode)
	{
		reply.printf("G%d command is not supported in machine mode %s", code, GetMachineModeString());
	}

	if (gb.GetState() == GCodeState::normal)
	{
		UnlockAll(gb);
		HandleReply(gb, result != GCodeResult::ok, reply.Pointer());
	}
	return true;
}

bool GCodes::HandleMcode(GCodeBuffer& gb, StringRef& reply)
{
	GCodeResult result = GCodeResult::ok;

	const int code = gb.GetIValue();
	if (   simulationMode != 0
		&& (code < 20 || code > 37)
		&& code != 0 && code != 1 && code != 82 && code != 83 && code != 105 && code != 109 && code != 111 && code != 112 && code != 122
		&& code != 204 && code != 207 && code != 408 && code != 999)
	{
		return true;			// we don't simulate most M codes
	}
	if (gb.MachineState().runningM502 && code != 301 && code != 307 && code != 558 && code != 665 && code != 666)
	{
		return true;			// when running M502 the only mcodes we execute are 301, 307, 558, 665 and 666
	}

	switch (code)
	{
	case 0: // Stop
	case 1: // Sleep
		if (   !LockMovementAndWaitForStandstill(gb)	// wait until everything has stopped
			|| !IsCodeQueueIdle()						// must also wait until deferred command queue has caught up
		   )
		{
			return false;
		}
		{
			bool wasPaused = isPaused;					// isPaused gets cleared by CancelPrint
			CancelPrint(true, &gb == fileGCode);		// if this is normal end-of-print commanded by the file, deleted the ressurrect.g file
			if (wasPaused)
			{
				reply.copy("Print cancelled");
				// If we are cancelling a paused print with M0 and we are homed and cancel.g exists then run it and do nothing else
				if (code == 0 && AllAxesAreHomed() && DoFileMacro(gb, CANCEL_G, false))
				{
					break;
				}
			}
		}

		gb.SetState((code == 0) ? GCodeState::stopping : GCodeState::sleeping);
		DoFileMacro(gb, (code == 0) ? STOP_G : SLEEP_G, false);
		break;

	case 3: // Spin spindle clockwise
		if (gb.Seen('S'))
		{
			switch (machineType)
			{
			case MachineType::cnc:
				reprap.GetPlatform().SetSpindlePwm(gb.GetFValue()/spindleMaxRpm);
				break;

			case MachineType::laser:
				reprap.GetPlatform().SetLaserPwm(gb.GetFValue()/laserMaxPower);
				break;

			default:
#if SUPPORT_ROLAND
				if (reprap.GetRoland()->Active())
				{
					result = reprap.GetRoland()->ProcessSpindle(gb.GetFValue());
				}
				else
#endif
				{
					result = GCodeResult::notSupportedInCurrentMode;
				}
				break;
			}
		}
		break;

	case 4: // Spin spindle counter clockwise
		if (gb.Seen('S'))
		{
			if (machineType == MachineType::cnc)
			{
				reprap.GetPlatform().SetSpindlePwm(-gb.GetFValue()/spindleMaxRpm);
			}
			else
			{
				result = GCodeResult::notSupportedInCurrentMode;
			}
		}
		break;

	case 5: // Spindle motor off
		switch (machineType)
		{
		case MachineType::cnc:
			reprap.GetPlatform().SetSpindlePwm(0.0);
			break;

		case MachineType::laser:
			reprap.GetPlatform().SetLaserPwm(0.0);
			break;

		default:
#if SUPPORT_ROLAND
			if (reprap.GetRoland()->Active())
			{
				result = reprap.GetRoland()->ProcessSpindle(0.0);
			}
			else
#endif
			{
				result = GCodeResult::notSupportedInCurrentMode;
			}
			break;
		}
		break;

	case 18: // Motors off
	case 84:
		if (!LockMovementAndWaitForStandstill(gb))
		{
			return false;
		}
		{
			bool seen = false;
			for (size_t axis = 0; axis < numTotalAxes; axis++)
			{
				if (gb.Seen(axisLetters[axis]))
				{
					SetAxisNotHomed(axis);
					platform.DisableDrive(axis);
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
						result = GCodeResult::error;
						break;
					}
					platform.DisableDrive(numTotalAxes + eDrive[i]);
				}
			}

			if (gb.Seen('S'))
			{
				seen = true;

				const float idleTimeout = gb.GetFValue();
				if (idleTimeout < 0.0)
				{
					reply.copy("Idle timeouts cannot be negative");
					result = GCodeResult::error;
				}
				else
				{
					reprap.GetMove().SetIdleTimeout(idleTimeout);
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
			const char* dir = (gb.Seen('P')) ? gb.GetString() : platform.GetGCodeDir();

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
				if (platform.Emulating() == me || platform.Emulating() == reprapFirmware)
				{
					fileResponse->copy("GCode files:\n");
				}

				bool encapsulateList = ((&gb != serialGCode && &gb != telnetGCode) || platform.Emulating() != marlin);
				FileInfo fileInfo;
				if (platform.GetMassStorage()->FindFirst(dir, fileInfo))
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
					} while (platform.GetMassStorage()->FindNext(fileInfo));

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
			const size_t card = (gb.Seen('P')) ? gb.GetIValue() : 0;
			result = GetGCodeResultFromError(platform.GetMassStorage()->Mount(card, reply, true)) ;
		}
		break;

	case 22: // Release SD card
		if (!LockFileSystem(gb))		// don't allow more than one at a time to avoid contention on output buffers
		{
			return false;
		}
		{
			const size_t card = (gb.Seen('P')) ? gb.GetIValue() : 0;
			result = GetGCodeResultFromError(platform.GetMassStorage()->Unmount(card, reply));
		}
		break;

	case 23: // Set file to print
	case 32: // Select file and start SD print
		// We now allow a file that is being printed to chain to another file. This is required for the resume-after-power-fail functionality.
		if (fileGCode->OriginalMachineState().fileState.IsLive() && (&gb) != fileGCode)
		{
			reply.copy("Cannot set file to print, because a file is already being printed");
			result = GCodeResult::error;
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
				if (QueueFileToPrint(filename, reply))
				{
					reprap.GetPrintMonitor().StartingPrint(filename);
					if (platform.Emulating() == marlin && (&gb == serialGCode || &gb == telnetGCode))
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
						StartPrinting();
					}
				}
				else
				{
					result = GCodeResult::error;
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
			if (AllAxesAreHomed())
			{
				DoFileMacro(gb, RESUME_G, true);
			}
		}
		else if (!fileToPrint.IsLive())
		{
			reply.copy("Cannot print, because no file is selected!");
			result = GCodeResult::error;
		}
		else
		{
			if (fileOffsetToPrint != 0)
			{
				// We executed M23 to set the file offset, which normally means that we are executing resurrect.g.
				// We need to copy the absolute/relative and volumetric extrusion flags over
				fileGCode->OriginalMachineState().drivesRelative = gb.MachineState().drivesRelative;
				fileGCode->OriginalMachineState().volumetricExtrusion = gb.MachineState().volumetricExtrusion;
				fileToPrint.Seek(fileOffsetToPrint);
			}
			StartPrinting();
		}
		break;

	case 226: // Gcode Initiated Pause
		if (&gb == fileGCode)						// ignore M226 if it did't come from within a file being printed
		{
			if (gb.IsDoingFileMacro())
			{
				pausePending = true;
			}
			else
			{
				if (!LockMovement(gb))					// lock movement before calling DoPause
				{
					return false;
				}
				DoPause(gb, false);
			}
		}
		break;

	case 25: // Pause the print
		if (isPaused)
		{
			reply.copy("Printing is already paused!!");
			result = GCodeResult::error;
		}
		else if (!reprap.GetPrintMonitor().IsPrinting())
		{
			reply.copy("Cannot pause print, because no file is being printed!");
			result = GCodeResult::error;
		}
		else if (fileGCode->IsDoingFileMacro())
		{
			pausePending = true;
		}
		else
		{
			if (!LockMovement(gb))					// lock movement before calling DoPause
			{
				return false;
			}
			DoPause(gb, false);
		}
		break;

	case 26: // Set SD position
		// This is used between executing M23 to set up the file to print, and M25 to print it
		if (gb.Seen('S'))
		{
			// Ideally we would get an unsigned value here in case of the file offset being >2Gb
			fileOffsetToPrint = (FilePosition)gb.GetIValue();
		}
		break;

	case 27: // Report print status - Deprecated
		if (reprap.GetPrintMonitor().IsPrinting())
		{
			// Pronterface keeps sending M27 commands if "Monitor status" is checked, and it specifically expects the following response syntax
			FileData& fileBeingPrinted = fileGCode->OriginalMachineState().fileState;
			reply.printf("SD printing byte %lu/%lu", fileBeingPrinted.GetPosition() - fileInput->BytesCached(), fileBeingPrinted.Length());
		}
		else
		{
			reply.copy("Not SD printing.");
		}
		break;

	case 28: // Write to file
		{
			const char* const str = gb.GetUnprecedentedString();
			if (str != nullptr)
			{
				const bool ok = OpenFileToWrite(gb, platform.GetGCodeDir(), str, 0, false, 0);
				if (ok)
				{
					reply.printf("Writing to file: %s", str);
				}
				else
				{
					reply.printf("Can't open file %s for writing.", str);
					result = GCodeResult::error;
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
				platform.GetMassStorage()->Delete(platform.GetGCodeDir(), filename, false);;
			}
		}
		break;

		// For case 32, see case 23

	case 36:	// Return file information
		if (!LockFileSystem(gb))									// getting file info takes several calls and isn't reentrant
		{
			return false;
		}
		{
			const char* filename = gb.GetUnprecedentedString(true);	// get filename, or nullptr if none provided
			OutputBuffer *fileInfoResponse;
			bool done = reprap.GetPrintMonitor().GetFileInfoResponse(filename, fileInfoResponse);
			if (done)
			{
				fileInfoResponse->cat('\n');
				UnlockAll(gb);
				HandleReply(gb, false, fileInfoResponse);
			}
			return done;
		}
		break;

	case 37:	// Simulation mode on/off, or simulate a whole file
		{
			bool seen = false;
			uint32_t newSimulationMode;
			String<FILENAME_LENGTH> simFileName;

			gb.TryGetPossiblyQuotedString('P', simFileName.GetRef(), seen);
			if (seen)
			{
				newSimulationMode = 1;			// default to simulation mode 1 when a filename is given
			}
			else
			{
				gb.TryGetUIValue('S', newSimulationMode, seen);
			}

			if (seen && newSimulationMode != simulationMode)
			{
				if (!LockMovementAndWaitForStandstill(gb))
				{
					return false;
				}

				const bool wasSimulating = (simulationMode != 0);
				simulationMode = (uint8_t)newSimulationMode;
				reprap.GetMove().Simulate(simulationMode);
				if (simFileName.IsEmpty() || simulationMode == 0)
				{
					// It's a simulation mode change command
					exitSimulationWhenFileComplete = false;
					if (simulationMode != 0)
					{
						simulationTime = 0.0;
						if (!wasSimulating)
						{
							// Starting a new simulation, so save the current position
							reprap.GetMove().GetCurrentUserPosition(simulationRestorePoint.moveCoords, 0, reprap.GetCurrentXAxes(), reprap.GetCurrentYAxes());
							simulationRestorePoint.feedRate = gb.MachineState().feedrate;
						}
					}
					else if (wasSimulating)
					{
						EndSimulation(&gb);
					}
				}
				else
				{
					// Simulate a whole file and then stop simulating
					simulationTime = 0.0;
					if (!wasSimulating)
					{
						// Starting a new simulation, so save the current position
						reprap.GetMove().GetCurrentUserPosition(simulationRestorePoint.moveCoords, 0, reprap.GetCurrentXAxes(), reprap.GetCurrentYAxes());
						simulationRestorePoint.feedRate = gb.MachineState().feedrate;
					}
					if (QueueFileToPrint(simFileName.c_str(), reply))
					{
						exitSimulationWhenFileComplete = true;
						reprap.GetPrintMonitor().StartingPrint(simFileName.c_str());
						StartPrinting();
						reply.printf("Simulating print of file %s", simFileName.c_str());
					}
					else
					{
						simulationMode = 0;
						reprap.GetMove().Simulate(0);
						result = GCodeResult::error;
					}
				}
			}
			else
			{
				reply.printf("Simulation mode: %s, move time: %.1f sec, other time: %.1f sec",
						(simulationMode != 0) ? "on" : "off", (double)reprap.GetMove().GetSimulationTime(), (double)simulationTime);
			}
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
				result = GCodeResult::notFinished;
			}
			else
			{
				reply.printf("Cannot open file: %s", filename);
				result = GCodeResult::error;
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
			const LogicalPin logicalPin = gb.GetIValue();
			Pin pin;
			bool invert;
			if (platform.GetFirmwarePin(logicalPin, PinAccess::pwm, pin, invert))
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

					const uint16_t freq = (gb.Seen('F')) ? (uint16_t)constrain<int32_t>(gb.GetIValue(), 1, 65536) : DefaultPinWritePwmFreq;
					IoPort::WriteAnalog(pin, val, freq);
				}
				// Ignore the command if no S parameter provided
			}
			else
			{
				reply.printf("Logical pin %d is not available for writing", logicalPin);
				result = GCodeResult::error;
			}
		}
		break;

	case 80:	// ATX power on
		platform.SetAtxPower(true);
		break;

	case 81:	// ATX power off
		if (!LockMovementAndWaitForStandstill(gb))
		{
			return false;
		}
		platform.SetAtxPower(false);
		break;

	case 82:	// Use absolute extruder positioning
		gb.MachineState().drivesRelative = false;
		break;

	case 83:	// Use relative extruder positioning
		gb.MachineState().drivesRelative = true;
		break;

		// For case 84, see case 18

	case 85: // Set inactive time
		break;

	case 92: // Set/report steps/mm for some axes
		{
			bool seen = false;
			for (size_t axis = 0; axis < numTotalAxes; axis++)
			{
				if (gb.Seen(axisLetters[axis]))
				{
					if (!LockMovementAndWaitForStandstill(gb))
					{
						return false;
					}
					platform.SetDriveStepsPerUnit(axis, gb.GetFValue());
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
					platform.SetDriveStepsPerUnit(numTotalAxes + e, eVals[e]);
				}
			}

			if (seen)
			{
				// On a delta, if we change the drive steps/mm then we need to recalculate the motor positions
				reprap.GetMove().SetNewPosition(moveBuffer.coords, true);
			}
			else
			{
				reply.copy("Steps/mm: ");
				for (size_t axis = 0; axis < numTotalAxes; ++axis)
				{
					reply.catf("%c: %.3f, ", axisLetters[axis], (double)platform.DriveStepsPerUnit(axis));
				}
				reply.catf("E:");
				char sep = ' ';
				for (size_t extruder = 0; extruder < numExtruders; extruder++)
				{
					reply.catf("%c%.3f", sep, (double)platform.DriveStepsPerUnit(extruder + numTotalAxes));
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

	case 101: // Un-retract
		result = RetractFilament(gb, false);
		break;

	case 102:
		// S3D generates this command once at the start of he print job if firmware retraction is enabled.
		// It's not documented, so we just ignore it rather than generate an error message.
		break;

	case 103: // Retract
		result = RetractFilament(gb, true);
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
		GenerateTemperatureReport(reply);
		break;

	case 106: // Set/report fan values
		{
			bool seenFanNum = false;
			int32_t fanNum = 0;			// Default to the first fan
			gb.TryGetIValue('P', fanNum, seenFanNum);
			bool error = false;
			const bool processed = platform.ConfigureFan(code, fanNum, gb, reply, error);
			result = GetGCodeResultFromError(error);

			// ConfigureFan only processes S parameters if there were other parameters to process
			if (!processed && gb.Seen('S'))
			{
				const float f = constrain<float>(gb.GetFValue(), 0.0, 255.0);
				if (seenFanNum)
				{
					platform.SetFanValue(fanNum, f);
				}
				else
				{
					// We are processing an M106 S### command with no other recognised parameters and we have a tool selected.
					// Apply the fan speed setting to the fans in the fan mapping for the current tool.
					lastDefaultFanSpeed = f;
					SetMappedFanSpeed();
				}
			}

			// ConfigureFan doesn't process R parameters
			if (gb.Seen('R'))
			{
				// Restore fan speed to value when print was paused
				if (seenFanNum)
				{
					platform.SetFanValue(fanNum, pausedFanSpeeds[fanNum]);
				}
				else
				{
					lastDefaultFanSpeed = pausedDefaultFanSpeed;
					SetMappedFanSpeed();
				}
			}
		}
		break;

	case 107: // Fan 0 off - deprecated
		platform.SetFanValue(0, 0.0);
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

			if (simulationMode == 0)
			{
				SetToolHeaters(tool, temperature);
			}
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
				toolChangeParam = (simulationMode == 0) ? 0 : DefaultToolChangeParam;
				gb.SetState(GCodeState::m109ToolChange0);
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
			platform.SetBoardType((BoardType)gb.GetIValue());
		}
		else
		{
			reply.printf("FIRMWARE_NAME: %s FIRMWARE_VERSION: %s ELECTRONICS: %s", FIRMWARE_NAME, VERSION, platform.GetElectronicsString());
#ifdef DUET_NG
			const char* const expansionName = DuetExpansion::GetExpansionBoardName();
			if (expansionName != nullptr)
			{
				reply.catf(" + %s", expansionName);
			}
			const char* const additionalExpansionName = DuetExpansion::GetAdditionalExpansionBoardName();
			if (additionalExpansionName != nullptr)
			{
				reply.catf(" + %s", additionalExpansionName);
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
					CheckReportDue(gb, reply);				// check whether we need to send a temperature or status report
					isWaiting = true;
					return false;
				}
				seen = true;
			}

			if (gb.Seen('H'))
			{
				// Wait for specified heaters to be ready
				long heaters[Heaters];
				size_t heaterCount = Heaters;
				gb.GetLongArray(heaters, heaterCount);
				if (!cancelWait)
				{
					for (size_t i=0; i<heaterCount; i++)
					{
						if (!reprap.GetHeat().HeaterAtSetTemperature(heaters[i], true))
						{
							CheckReportDue(gb, reply);		// check whether we need to send a temperature or status report
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
				const int8_t chamberHeater = reprap.GetHeat().GetChamberHeater();
				if (chamberHeater != -1)
				{
					if (!cancelWait && !reprap.GetHeat().HeaterAtSetTemperature(chamberHeater, true))
					{
						CheckReportDue(gb, reply);			// check whether we need to send a temperature or status report
						isWaiting = true;
						return false;
					}
				}
				seen = true;
			}

			// Wait for all heaters to be ready
			if (!seen && !cancelWait && !reprap.GetHeat().AllHeatersAtSetTemperatures(true))
			{
				CheckReportDue(gb, reply);					// check whether we need to send a temperature or status report
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
		for (size_t axis = 0; axis < numTotalAxes; axis++)
		{
			reply.catf("%c: %s, ", axisLetters[axis], TranslateEndStopResult(platform.Stopped(axis)));
		}
		reply.catf("Z probe: %s", TranslateEndStopResult(platform.GetZProbeResult()));
		break;

	case 120:
		Push(gb);
		break;

	case 121:
		Pop(gb);
		break;

	case 122:
		{
			const int val = (gb.Seen('P')) ? gb.GetIValue() : 0;
			if (val == 0)
			{
				reprap.Diagnostics(gb.GetResponseMessageType());
			}
			else
			{
				platform.DiagnosticTest(val);
			}
		}
		break;

	case 135: // Set PID sample interval
		if (gb.Seen('S'))
		{
			platform.SetHeatSampleTime(gb.GetFValue() * 0.001);  // Value is in milliseconds; we want seconds
		}
		else
		{
			reply.printf("Heat sample time is %.3f seconds", (double)platform.GetHeatSampleTime());
		}
		break;

	case 140: // Set bed temperature
		{
			Heat& heat = reprap.GetHeat();
			int8_t bedHeater;
			if (gb.Seen('H'))
			{
				bedHeater = gb.GetIValue();
				if (bedHeater < 0)
				{
					// Make sure we stay within reasonable boundaries...
					bedHeater = -1;
				}
				else if (bedHeater >= (int)Heaters)
				{
					reply.copy("Invalid heater number");
					result = GCodeResult::error;
					break;
				}
				heat.SetBedHeater(bedHeater);
				platform.UpdateConfiguredHeaters();

				if (bedHeater < 0)
				{
					// Stop here if the hot bed has been disabled
					break;
				}
			}
			else
			{
				bedHeater = heat.GetBedHeater();
				if (bedHeater < 0)
				{
					reply.copy("Hot bed is not present");
					result = GCodeResult::error;
					break;
				}
			}

			if (gb.Seen('S'))
			{
				const float temperature = gb.GetFValue();
				if (temperature < NEARLY_ABS_ZERO)
				{
					heat.SwitchOff(bedHeater);
				}
				else
				{
					heat.SetActiveTemperature(bedHeater, temperature);
					heat.Activate(bedHeater);
				}
			}
			if (gb.Seen('R'))
			{
				heat.SetStandbyTemperature(bedHeater, gb.GetFValue());
			}
		}
		break;

	case 141: // Chamber temperature
		{
			Heat& heat = reprap.GetHeat();
			bool seen = false;
			if (gb.Seen('H'))
			{
				seen = true;

				int heater = gb.GetIValue();
				if (heater < 0)
				{
					heater = -1;
				}
				else if (heater >= (int)Heaters)
				{
					reply.copy("Bad heater number specified!");
					result = GCodeResult::error;
					break;
				}
				else
				{
					heat.SetChamberHeater(heater);
					platform.UpdateConfiguredHeaters();
				}
			}

			if (gb.Seen('S'))
			{
				seen = true;

				const int8_t currentHeater = heat.GetChamberHeater();
				if (currentHeater != -1)
				{
					const float temperature = gb.GetFValue();
					if (temperature < NEARLY_ABS_ZERO)
					{
						heat.SwitchOff(currentHeater);
					}
					else
					{
						heat.SetActiveTemperature(currentHeater, temperature);
						heat.Activate(currentHeater);
					}
				}
				else
				{
					reply.copy("No chamber heater has been set up yet!");
					result = GCodeResult::error;
				}
			}

			if (!seen)
			{
				const int8_t currentHeater = reprap.GetHeat().GetChamberHeater();
				if (currentHeater != -1)
				{
					reply.printf("Chamber heater %d is currently at %.1f" DEGREE_SYMBOL "C", currentHeater, (double)reprap.GetHeat().GetTemperature(currentHeater));
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
			if (heater < 0 || heater >= (int)Heaters)
			{
				reply.copy("Invalid heater number");
				result = GCodeResult::error;
			}
			else if (gb.Seen('S'))
			{
				const float limit = gb.GetFValue();
				if (limit > BAD_LOW_TEMPERATURE && limit < BAD_ERROR_TEMPERATURE)
				{
					reprap.GetHeat().SetTemperatureLimit(heater, limit);
				}
				else
				{
					reply.copy("Invalid temperature limit");
					result = GCodeResult::error;
				}
			}
			else
			{
				reply.printf("Temperature limit for heater %d is %.1f" DEGREE_SYMBOL "C", heater, (double)reprap.GetHeat().GetTemperatureLimit(heater));
			}
		}
		break;

	case 144: // Set bed to standby
		{
			const int8_t bedHeater = reprap.GetHeat().GetBedHeater();
			if (bedHeater >= 0)
			{
				reprap.GetHeat().Standby(bedHeater, nullptr);
			}
		}
		break;

	case 190: // Set bed temperature and wait
	case 191: // Set chamber temperature and wait
		{
			const int8_t heater = (code == 191) ? reprap.GetHeat().GetChamberHeater() : reprap.GetHeat().GetBedHeater();
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

				reprap.GetHeat().SetActiveTemperature(heater, temperature);
				reprap.GetHeat().Activate(heater);
				if (cancelWait || reprap.GetHeat().HeaterAtSetTemperature(heater, waitWhenCooling))
				{
					cancelWait = isWaiting = false;
					break;
				}

				CheckReportDue(gb, reply);			// check whether we need to send a temperature or status report
				isWaiting = true;
				return false;
			}
		}
		break;

	case 200: // Set filament diameter for volumetric extrusion and enable/disable volumetric extrusion
		if (gb.Seen('D'))
		{
			float diameters[MaxExtruders];
			size_t len = MaxExtruders;
			gb.GetFloatArray(diameters, len, true);
			for (size_t i = 0; i < len; ++i)
			{
				const float d = diameters[i];
				volumetricExtrusionFactors[i] = (d <= 0.0) ? 1.0 : 4.0/(fsquare(d) * PI);
			}
			gb.MachineState().volumetricExtrusion = (diameters[0] > 0.0);
		}
		else if (!gb.MachineState().volumetricExtrusion)
		{
			reply.copy("Volumetric extrusion is disabled for this input source");
		}
		else
		{
			reply.copy("Filament diameters for volumetric extrusion:");
			for (size_t i = 0; i < numExtruders; ++i)
			{
				const float vef = volumetricExtrusionFactors[i];
				if (vef == 1.0)
				{
					reply.cat(" n/a");
				}
				else
				{
					reply.catf(" %.03f", (double)(2.0 * sqrtf(vef/PI)));
				}
			}
		}
		break;

	case 201: // Set/print axis accelerations
		{
			bool seen = false;
			for (size_t axis = 0; axis < numVisibleAxes; axis++)
			{
				if (gb.Seen(axisLetters[axis]))
				{
					platform.SetAcceleration(axis, gb.GetFValue() * distanceScale);
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
					platform.SetAcceleration(numTotalAxes + e, eVals[e] * distanceScale);
				}
			}

			if (!seen)
			{
				reply.printf("Accelerations: ");
				for (size_t axis = 0; axis < numVisibleAxes; ++axis)
				{
					reply.catf("%c: %.1f, ", axisLetters[axis], (double)(platform.Acceleration(axis) / distanceScale));
				}
				reply.cat("E:");
				char sep = ' ';
				for (size_t extruder = 0; extruder < numExtruders; extruder++)
				{
					reply.catf("%c%.1f", sep, (double)(platform.Acceleration(extruder + numTotalAxes) / distanceScale));
					sep = ':';
				}
			}
		}
		break;

	case 203: // Set/print maximum feedrates
		{
			bool seen = false;
			for (size_t axis = 0; axis < numVisibleAxes; ++axis)
			{
				if (gb.Seen(axisLetters[axis]))
				{
					platform.SetMaxFeedrate(axis, gb.GetFValue() * distanceScale * SecondsToMinutes); // G Code feedrates are in mm/minute; we need mm/sec
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
					platform.SetMaxFeedrate(numTotalAxes + e, eVals[e] * distanceScale * SecondsToMinutes);
				}
			}

			if (!seen)
			{
				reply.copy("Maximum feedrates: ");
				for (size_t axis = 0; axis < numVisibleAxes; ++axis)
				{
					reply.catf("%c: %.1f, ", axisLetters[axis], (double)(platform.MaxFeedrate(axis) / (distanceScale * SecondsToMinutes)));
				}
				reply.cat("E:");
				char sep = ' ';
				for (size_t extruder = 0; extruder < numExtruders; extruder++)
				{
					reply.catf("%c%.1f", sep, (double)(platform.MaxFeedrate(extruder + numTotalAxes) / (distanceScale * SecondsToMinutes)));
					sep = ':';
				}
			}
		}
		break;

	case 204: // Set max travel and printing accelerations
		{
			bool seen = false;
			if (gb.Seen('S') && platform.Emulating() == marlin)
			{
				// For backwards compatibility e.g. with Cura, set both accelerations as Marlin does.
				const float acc = gb.GetFValue();
				platform.SetMaxPrintingAcceleration(acc);
				platform.SetMaxTravelAcceleration(acc);
				seen = true;
			}
			if (gb.Seen('P'))
			{
				platform.SetMaxPrintingAcceleration(gb.GetFValue());
				seen = true;
			}
			if (gb.Seen('T'))
			{
				platform.SetMaxTravelAcceleration(gb.GetFValue());
				seen = true;
			}
			if (!seen)
			{
				reply.printf("Maximum printing acceleration %.1f, maximum travel acceleration %.1f", (double)platform.GetMaxPrintingAcceleration(), (double)platform.GetMaxTravelAcceleration());
			}
		}
		break;

	case 206: // Offset axes - Deprecated
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
			if (gb.Seen('R'))	// must do this one after 'S'
			{
				retractExtra = max<float>(gb.GetFValue(), -retractLength);
				seen = true;
			}
			if (gb.Seen('F'))
			{
				unRetractSpeed = retractSpeed = max<float>(gb.GetFValue(), 60.0) * SecondsToMinutes;
				seen = true;
			}
			if (gb.Seen('T'))	// must do this one after 'F'
			{
				unRetractSpeed = max<float>(gb.GetFValue(), 60.0) * SecondsToMinutes;
				seen = true;
			}
			if (gb.Seen('Z'))
			{
				retractHop = max<float>(gb.GetFValue(), 0.0);
				seen = true;
			}
			if (!seen)
			{
				reply.printf("Retraction/un-retraction settings: length %.2f/%.2fmm, speed %d/%dmm/min, Z hop %.2fmm",
					(double)retractLength, (double)(retractLength + retractExtra), (int)(retractSpeed * MinutesToSeconds), (int)(unRetractSpeed * MinutesToSeconds), (double)retractHop);
			}
		}
		break;

	case 208: // Set/print maximum axis lengths. If there is an S parameter with value 1 then we set the min value, else we set the max value.
		{
			bool setMin = (gb.Seen('S') ? (gb.GetIValue() == 1) : false);
			bool seen = false;
			for (size_t axis = 0; axis < numVisibleAxes; axis++)
			{
				if (gb.Seen(axisLetters[axis]))
				{
					const float value = gb.GetFValue() * distanceScale;
					if (setMin)
					{
						platform.SetAxisMinimum(axis, value);
					}
					else
					{
						platform.SetAxisMaximum(axis, value);
					}
					seen = true;
				}
			}

			if (!seen)
			{
				reply.copy("Axis limits ");
				char sep = '-';
				for (size_t axis = 0; axis < numVisibleAxes; axis++)
				{
					reply.catf("%c %c: %.1f min, %.1f max", sep, axisLetters[axis], (double)platform.AxisMinimum(axis), (double)platform.AxisMaximum(axis));
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
			float newSpeedFactor = (gb.GetFValue() * 0.01) * SecondsToMinutes;	// include the conversion from mm/minute to mm/second
			if (newSpeedFactor > 0.0)
			{
				// Update the feed rate for ALL input sources, and all feed rates on the stack
				const float speedFactorRatio = newSpeedFactor / speedFactor;
				for (size_t i = 0; i < ARRAY_SIZE(gcodeSources); ++i)
				{
					GCodeMachineState *ms = &gcodeSources[i]->MachineState();
					while (ms != nullptr)
					{
						ms->feedrate *= speedFactorRatio;
						ms = ms->previous;
					}
				}
				// If the last move hasn't gone yet, update its feed rate too if it is not a firmware retraction
				if (segmentsLeft != 0 && !moveBuffer.isFirmwareRetraction)
				{
					moveBuffer.feedRate *= speedFactorRatio;
				}
				speedFactor = newSpeedFactor;
			}
			else
			{
				reply.printf("Invalid speed factor specified.");
				result = GCodeResult::error;
			}
		}
		else
		{
			reply.printf("Speed factor override: %.1f%%", (double)(speedFactor * MinutesToSeconds * 100.0));
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
							moveBuffer.coords[extruder + numTotalAxes] *= extrusionFactor/extrusionFactors[extruder];	// last move not gone, so update it
						}
						extrusionFactors[extruder] = extrusionFactor;
					}
				}
				else
				{
					reply.printf("Extrusion factor override for extruder %" PRIi32 ": %.1f%%", extruder, (double)(extrusionFactors[extruder] * 100.0));
				}
			}
		}
		break;

		// For case 226, see case 25

	case 280:	// Servos
		if (gb.Seen('P'))
		{
			const LogicalPin servoIndex = gb.GetIValue();
			Pin servoPin;
			bool invert;
			if (platform.GetFirmwarePin(servoIndex, PinAccess::servo, servoPin, invert))
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
						IoPort::WriteAnalog(servoPin, (invert) ? 1.0 : 0.0, ServoRefreshFrequency);
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
						IoPort::WriteAnalog(servoPin, pwm, ServoRefreshFrequency);
					}
				}
				// We don't currently allow the servo position to be read back
			}
			else
			{
				platform.MessageF(ErrorMessage, "Invalid servo index %d in M280 command\n", servoIndex);
			}
		}
		break;

	case 290:	// Baby stepping
		if (gb.Seen('S'))
		{
			if (!LockMovement(gb))
			{
				return false;
			}
			const float babyStepAmount = constrain<float>(gb.GetFValue(), -1.0, 1.0);
			currentBabyStepZOffset += babyStepAmount;
			const float amountPushed = reprap.GetMove().PushBabyStepping(babyStepAmount);
			moveBuffer.initialCoords[Z_AXIS] += amountPushed;

			// The following causes all the remaining baby stepping that we didn't manage to push to be added to the [remainder of the] currently-executing move, if there is one.
			// This could result in an abrupt Z movement, however the move will be processed as normal so the jerk limit will be honoured.
			moveBuffer.coords[Z_AXIS] += babyStepAmount;
		}
		else
		{
			reply.printf("Baby stepping offset is %.3fmm", (double)GetBabyStepOffset());
		}
		break;

	case 291:	// Display message, optionally wait for acknowledgement
		{
			String<MESSAGE_LENGTH> title;
			bool seen = false;
			gb.TryGetQuotedString('R', title.GetRef(), seen);

			String<MESSAGE_LENGTH> message;
			gb.TryGetQuotedString('P', message.GetRef(), seen);
			if (seen)
			{
				int32_t sParam = 1;
				gb.TryGetIValue('S', sParam, seen);
				if (sParam < 0 || sParam > 3)
				{
					reply.copy("Invalid message box mode");
					result = GCodeResult::error;
					break;
				}

				float tParam;
				if (sParam == 0 || sParam == 1)
				{
					tParam = DefaultMessageTimeout;
					gb.TryGetFValue('T', tParam, seen);
				}
				else
				{
					tParam = 0.0;
				}

				AxesBitmap axisControls = 0;
				for (size_t axis = 0; axis < numTotalAxes; axis++)
				{
					if (gb.Seen(axisLetters[axis]) && gb.GetIValue() > 0)
					{
						SetBit(axisControls, axis);
					}
				}

				// If we need to wait for an acknowledgement, save the state and set waiting
				if ((sParam == 2 || sParam == 3) && Push(gb))						// stack the machine state including the file position
				{
					gb.MachineState().fileState.Close();							// stop reading from file
					gb.MachineState().waitingForAcknowledgement = true;				// flag that we are waiting for acknowledgement
				}

				// TODO: consider displaying the message box on all relevant devices. Acknowledging any one of them needs to clear them all.
				// Currently, if mt is http or aux or generic, we display the message box both in DWC and on PanelDue.
				const MessageType mt = GetMessageBoxDevice(gb);						// get the display device
				platform.SendAlert(mt, message.c_str(), title.c_str(), (int)sParam, tParam, axisControls);
			}
		}
		break;

	case 292:	// Acknowledge message
		{
			reprap.ClearAlert();

			const bool cancelled = (gb.Seen('P') && gb.GetIValue() == 1);
			for (GCodeBuffer* targetGb : gcodeSources)
			{
				if (targetGb != nullptr)
				{
					targetGb->MessageAcknowledged(cancelled);
				}
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
			reprap.GetHeat().AllowColdExtrude(gb.GetIValue() > 0);
		}
		else
		{
			reply.printf("Cold extrusion is %s, use M302 P[1/0] to allow/deny it",
					reprap.GetHeat().ColdExtrude() ? "allowed" : "denied");
		}
		break;

	case 303: // Run PID tuning
		if (gb.Seen('H'))
		{
			const int heater = gb.GetIValue();
			const float temperature = (gb.Seen('S')) ? gb.GetFValue()
										: heater == reprap.GetHeat().GetBedHeater() ? 75.0
										: heater == reprap.GetHeat().GetChamberHeater() ? 50.0
										: 200.0;
			const float maxPwm = (gb.Seen('P')) ? gb.GetFValue() : 1.0;
			if (heater < 0 || heater >= (int)Heaters)
			{
				reply.copy("Bad heater number in M303 command");
			}
			else if (temperature >= reprap.GetHeat().GetTemperatureLimit(heater))
			{
				reply.copy("Target temperature must be below temperature limit for this heater");
			}
			else if (maxPwm < 0.1 || maxPwm > 1.0)
			{
				reply.copy("Invalid PWM in M303 command");
			}
			else
			{
				reprap.GetHeat().StartAutoTune(heater, temperature, maxPwm, reply);
			}
		}
		else
		{
			reprap.GetHeat().GetAutoTuneStatus(reply);
		}
		break;

	case 304: // Set/report heated bed PID values
		{
			const int8_t bedHeater = reprap.GetHeat().GetBedHeater();
			if (bedHeater >= 0)
			{
				SetPidParameters(gb, bedHeater, reply);
			}
		}
		break;

	case 305: // Set/report specific heater parameters
		result = SetHeaterParameters(gb, reply);
		break;

	case 307: // Set heater process model parameters
		if (gb.Seen('H'))
		{
			const int heater = gb.GetIValue();
			if (heater >= 0 && heater < (int)Heaters)
			{
				const FopDt& model = reprap.GetHeat().GetHeaterModel(heater);
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
					if (!reprap.GetHeat().SetHeaterModel(heater, gain, tc, td, maxPwm, dontUsePid == 0))
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
							heater, (double)model.GetGain(), (double)model.GetTimeConstant(), (double)model.GetDeadTime(), (double)model.GetMaxPwm(), mode);
					if (model.UsePid())
					{
						// When reporting the PID parameters, we scale them by 255 for compatibility with older firmware and other firmware
						M301PidParameters params = model.GetM301PidParameters(false);
						reply.catf("\nComputed PID parameters for setpoint change: P%.1f, I%.3f, D%.1f", (double)params.kP, (double)params.kI, (double)params.kD);
						params = model.GetM301PidParameters(true);
						reply.catf("\nComputed PID parameters for load change: P%.1f, I%.3f, D%.1f", (double)params.kP, (double)params.kI, (double)params.kD);
					}
				}
			}
		}
		break;

	case 350: // Set/report microstepping
		{
			// interp is currently an int not a bool, because we use special values of interp to set the chopper control register
			int32_t mode = 0;						// this is usually the interpolation rwquested (0 = off, 1 = on)
			bool dummy;
			gb.TryGetIValue('I', mode, dummy);

			bool seen = false;
			for (size_t axis = 0; axis < numTotalAxes; axis++)
			{
				if (gb.Seen(axisLetters[axis]))
				{
					if (!LockMovementAndWaitForStandstill(gb))
					{
						return false;
					}
					seen = true;
					const int microsteps = gb.GetIValue();
					if (ChangeMicrostepping(axis, microsteps, mode))
					{
						SetAxisNotHomed(axis);
					}
					else
					{
						reply.printf("Drive %c does not support %dx microstepping%s", axisLetters[axis], microsteps, ((mode) ? " with interpolation" : ""));
						result = GCodeResult::error;
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
					if (!ChangeMicrostepping(numTotalAxes + e, (int)eVals[e], mode))
					{
						reply.printf("Drive E%u does not support %dx microstepping%s", e, (int)eVals[e], ((mode) ? " with interpolation" : ""));
						result = GCodeResult::error;
					}
				}
			}

			if (!seen)
			{
				reply.copy("Microstepping - ");
				for (size_t axis = 0; axis < numTotalAxes; ++axis)
				{
					bool interp;
					const int microsteps = platform.GetMicrostepping(axis, mode, interp);
					reply.catf("%c:%d%s, ", axisLetters[axis], microsteps, (interp) ? "(on)" : "");
				}
				reply.cat("E");
				for (size_t extruder = 0; extruder < numExtruders; extruder++)
				{
					bool interp;
					const int microsteps = platform.GetMicrostepping(extruder + numTotalAxes, mode, interp);
					reply.catf(":%d%s", microsteps, (interp) ? "(on)" : "");
				}
			}
		}
		break;

	case 374: // Save grid and height map to file
		result = GetGCodeResultFromError(SaveHeightMap(gb, reply));
		break;

	case 375: // Load grid and height map from file and enable compensation
		if (!LockMovementAndWaitForStandstill(gb))
		{
			return false;
		}
		result = GetGCodeResultFromError(LoadHeightMap(gb, reply));
		break;

	case 376: // Set taper height
		{
			Move& move = reprap.GetMove();
			if (gb.Seen('H'))
			{
				move.SetTaperHeight(gb.GetFValue());
			}
			else if (move.GetTaperHeight() > 0.0)
			{
				reply.printf("Bed compensation taper height is %.1fmm", (double)move.GetTaperHeight());
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
		if (platform.GetZProbeType() != 0)
		{
			probeIsDeployed = true;
			DoFileMacro(gb, DEPLOYPROBE_G, false);
		}
		break;

	case 402: // Retract Z probe
		if (platform.GetZProbeType() != 0)
		{
			probeIsDeployed = false;
			DoFileMacro(gb, RETRACTPROBE_G, false);
		}
		break;

	case 404: // Filament width and nozzle diameter
		{
			bool seen = false;

			if (gb.Seen('N'))
			{
				platform.SetFilamentWidth(gb.GetFValue());
				seen = true;
			}
			if (gb.Seen('D'))
			{
				platform.SetNozzleDiameter(gb.GetFValue());
				seen = true;
			}

			if (!seen)
			{
				reply.printf("Filament width: %.2fmm, nozzle diameter: %.2fmm", (double)platform.GetFilamentWidth(), (double)platform.GetNozzleDiameter());
			}
		}
		break;

	case 408: // Get status in JSON format
		{
			const int type = gb.Seen('S') ? gb.GetIValue() : 0;
			const int seq = gb.Seen('R') ? gb.GetIValue() : -1;
			if (&gb == auxGCode)
			{
				lastAuxStatusReportType = type;
			}

			OutputBuffer * const statusResponse = GenerateJsonStatusResponse(type, seq, (&gb == auxGCode) ? ResponseSource::AUX : ResponseSource::Generic);

			if (statusResponse != nullptr)
			{
				UnlockAll(gb);
				HandleReply(gb, false, statusResponse);
				return true;
			}
		}
		break;

	case 450: // Report printer mode
		reply.printf("PrinterMode:%s", GetMachineModeString());
		break;

	case 451: // Select FFF printer mode
		machineType = MachineType::fff;
		break;

	case 452: // Select laser mode
		machineType = MachineType::laser;
		if (gb.Seen('P'))
		{
			int lp = gb.GetIValue();
			if (lp < 0 || lp > 65535)
			{
				lp = NoLogicalPin;
			}
			if (reprap.GetPlatform().SetLaserPin(lp))
			{
				reply.copy("Laser mode selected");
			}
			else
			{
				reply.copy("Bad M452 P parameter");
				result = GCodeResult::error;
			}
		}
		if (result == GCodeResult::ok && gb.Seen('F'))
		{
			reprap.GetPlatform().SetLaserPwmFrequency(gb.GetFValue());
		}
		if (result == GCodeResult::ok && gb.Seen('R'))
		{
			laserMaxPower = max<float>(1.0, gb.GetFValue());
		}
		break;

	case 453: // Select CNC mode
		machineType = MachineType::cnc;
		if (gb.Seen('P'))
		{
			int32_t pins[2] = { NoLogicalPin, NoLogicalPin };
			size_t numPins = 2;
			gb.GetLongArray(pins, numPins);
			if (pins[0] < 0 || pins[0] > 65535)
			{
				pins[0] = NoLogicalPin;
			}
			if (numPins < 2 || pins[1] < 0 || pins[1] > 65535)
			{
				pins[1] = NoLogicalPin;
			}
			if (reprap.GetPlatform().SetSpindlePins(pins[0], pins[1]))
			{
				reply.copy("CNC mode selected");
			}
			else
			{
				reply.copy("Bad M453 P parameter");
				result = GCodeResult::error;
			}
		}
		if (result == GCodeResult::ok && gb.Seen('F'))
		{
			reprap.GetPlatform().SetSpindlePwmFrequency(gb.GetFValue());
		}
		if (result == GCodeResult::ok && gb.Seen('R'))
		{
			spindleMaxRpm = max<float>(1.0, gb.GetFValue());
		}
		break;

	case 500: // Store parameters in EEPROM
		result = GetGCodeResultFromError(WriteConfigOverrideFile(reply, CONFIG_OVERRIDE_G));
		break;

	case 501: // Load parameters from EEPROM
		DoFileMacro(gb, "config-override.g", true);
		break;

	case 502: // Revert to default "factory settings"
		reprap.GetHeat().ResetHeaterModels();							// in case some heaters have no M307 commands in config.g
		reprap.GetMove().GetKinematics().SetCalibrationDefaults();		// in case M665/M666/M667/M669 in config.g don't define all the parameters
		platform.SetZProbeDefaults();
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
			FileStore * const f = platform.GetFileStore(platform.GetSysDir(), platform.GetConfigFile(), OpenMode::read);
			if (f == nullptr)
			{
				reply.copy("Configuration file not found!");
				result = GCodeResult::error;
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
			const byte* mac = platform.MACAddress();
			reply.printf("MAC: %x:%x:%x:%x:%x:%x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
		}
		break;

	case 550: // Set/report machine name
		{
			String<MACHINE_NAME_LENGTH> name;
			bool seen = false;
			gb.TryGetPossiblyQuotedString('P', name.GetRef(), seen);
			if (seen)
			{
				reprap.SetName(name.c_str());
			}
			else
			{
				reply.printf("RepRap name: %s", reprap.GetName());
			}
		}
		break;

	case 551: // Set password (no option to report it)
		{
			String<PASSWORD_LENGTH> password;
			bool seen = false;
			gb.TryGetPossiblyQuotedString('P', password.GetRef(), seen);
			if (seen)
			{
				reprap.SetPassword(password.c_str());
			}
		}
		break;

	case 552: // Enable/Disable network and/or Set/Get IP address
		{
			bool seen = false;

#ifdef DUET_WIFI
			if (gb.Seen('S')) // Has the user turned the network on or off?
			{
				const int enableValue = gb.GetIValue();
				seen = true;

				char ssidBuffer[SsidLength + 1];
				StringRef ssid(ssidBuffer, ARRAY_SIZE(ssidBuffer));
				if (gb.Seen('P') && !gb.GetQuotedString(ssid))
				{
					reply.copy("Bad or missing SSID in M552 command");
					result = GCodeResult::error;
				}
				else
				{
					reprap.GetNetwork().Enable(enableValue, ssid, reply);
				}
			}
#else
			if (gb.Seen('P'))
			{
				seen = true;
				uint8_t eth[4];
				if (gb.GetIPAddress(eth))
				{
					platform.SetIPAddress(eth);
				}
				else
				{
					reply.copy("Bad IP address");
					result = GCodeResult::error;
					break;
				}
			}

			// Process this one last in case the IP address is changed and the network enabled in the same command
			if (gb.Seen('S')) // Has the user turned the network on or off?
			{
				seen = true;
				reprap.GetNetwork().Enable(gb.GetIValue(), reply);
			}
#endif

			if (!seen)
			{
				result = GetGCodeResultFromError(reprap.GetNetwork().GetNetworkState(reply));
			}
		}
		break;

	case 553: // Set/Get netmask
		if (gb.Seen('P'))
		{
			uint8_t eth[4];
			if (gb.GetIPAddress(eth))
			{
				platform.SetNetMask(eth);
			}
			else
			{
				reply.copy("Bad IP address");
				result = GCodeResult::error;
			}
		}
		else
		{
			const uint8_t * const nm = platform.NetMask();
			reply.printf("Net mask: %d.%d.%d.%d ", nm[0], nm[1], nm[2], nm[3]);
		}
		break;

	case 554: // Set/Get gateway
		if (gb.Seen('P'))
		{
			uint8_t eth[4];
			if (gb.GetIPAddress(eth))
			{
				platform.SetGateWay(eth);
			}
			else
			{
				reply.copy("Bad IP address");
				result = GCodeResult::error;
			}
		}
		else
		{
			const uint8_t * const gw = platform.GateWay();
			reply.printf("Gateway: %d.%d.%d.%d ", gw[0], gw[1], gw[2], gw[3]);
		}
		break;

	case 555: // Set/report firmware type to emulate
		if (gb.Seen('P'))
		{
			platform.SetEmulating((Compatibility) gb.GetIValue());
		}
		else
		{
			reply.copy("Emulating ");
			switch (platform.Emulating())
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
				reply.catf("Unknown: (%d)", platform.Emulating());
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
					reprap.GetMove().SetAxisCompensation(axis, gb.GetFValue() / value);
				}
			}
		}
		else
		{
			reply.printf("Axis compensations - XY: %.5f, YZ: %.5f, ZX: %.5f",
				(double)reprap.GetMove().AxisCompensation(X_AXIS), (double)reprap.GetMove().AxisCompensation(Y_AXIS), (double)reprap.GetMove().AxisCompensation(Z_AXIS));
		}
		break;

	case 557: // Set/report Z probe point coordinates
		if (gb.Seen('P'))
		{
			reply.copy("Error: M557 P parameter is no longer supported. use a bed.g file instead.\n");
		}
		else
		{
			LockMovement(gb);							// to ensure that probing is not already in progress
			result = GetGCodeResultFromError(DefineGrid(gb, reply));
		}
		break;

	case 558: // Set or report Z probe type and for which axes it is used
		{
			bool seenAxes = false, seenType = false, seenParam = false;
			AxesBitmap zProbeAxes = platform.GetZProbeAxes();
			for (size_t axis = 0; axis < numVisibleAxes; axis++)
			{
				if (gb.Seen(axisLetters[axis]))
				{
					if (gb.GetIValue() > 0)
					{
						SetBit(zProbeAxes, axis);
					}
					else
					{
						ClearBit(zProbeAxes, axis);
					}
					seenAxes = true;
				}
			}
			if (seenAxes)
			{
				platform.SetZProbeAxes(zProbeAxes);
			}

			// We must get and set the Z probe type first before setting the dive height etc., because different probe types may have different parameters
			if (gb.Seen('P'))		// probe type
			{
				platform.SetZProbeType(gb.GetIValue());
				seenType = true;
			}

			ZProbeParameters params = platform.GetCurrentZProbeParameters();
			gb.TryGetFValue('H', params.diveHeight, seenParam);		// dive height

			if (gb.Seen('F'))		// feed rate i.e. probing speed
			{
				params.probeSpeed = gb.GetFValue() * SecondsToMinutes;
				seenParam = true;
			}

			if (gb.Seen('T'))		// travel speed to probe point
			{
				params.travelSpeed = gb.GetFValue() * SecondsToMinutes;
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
				platform.SetZProbeParameters(platform.GetZProbeType(), params);
			}

			if (!(seenAxes || seenType || seenParam))
			{
				reply.printf("Z Probe type %d, invert %s, dive height %.1fmm, probe speed %dmm/min, travel speed %dmm/min, recovery time %.2f sec",
								platform.GetZProbeType(), (params.invertReading) ? "yes" : "no", (double)params.diveHeight,
								(int)(params.probeSpeed * MinutesToSeconds), (int)(params.travelSpeed * MinutesToSeconds), (double)params.recoveryTime);
				reply.cat(", used for axes:");
				for (size_t axis = 0; axis < numVisibleAxes; axis++)
				{
					if (IsBitSet(zProbeAxes, axis))
					{
						reply.catf(" %c", axisLetters[axis]);
					}
				}
			}
		}
		break;

	case 559:
	case 560: // Binary writing
	{
		const char* folder = platform.GetSysDir();
		const char* defaultFile = platform.GetConfigFile();
		if (code == 560)
		{
			folder = platform.GetWebDir();
			defaultFile = INDEX_PAGE_FILE;
		}
		const char* filename = (gb.Seen('P') ? gb.GetString() : defaultFile);
		const FilePosition size = (gb.Seen('S') ? (FilePosition)gb.GetIValue() : 0);
		const uint32_t crc32 = (gb.Seen('C') ? gb.GetUIValue() : 0);
		const bool ok = OpenFileToWrite(gb, folder, filename, size, true, crc32);
		if (ok)
		{
			reply.printf("Writing to file: %s", filename);
		}
		else
		{
			reply.printf("Can't open file %s for writing.", filename);
			result = GCodeResult::error;
		}
	}
	break;

	case 561: // Set identity transform (also clears bed probe grid)
		reprap.GetMove().SetIdentityTransform();
		break;

	case 562: // Reset temperature fault - use with great caution
		if (gb.Seen('P'))
		{
			const int heater = gb.GetIValue();
			if (heater >= 0 && heater < (int)Heaters)
			{
				reprap.ClearTemperatureFault(heater);
			}
			else
			{
				reply.copy("Invalid heater number.\n");
				result = GCodeResult::error;
			}
		}
		break;

	case 563: // Define tool
		result = GetGCodeResultFromError(ManageTool(gb, reply));
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
			for (size_t axis = 0; axis < numVisibleAxes; axis++)
			{
				if (gb.Seen(axisLetters[axis]))
				{
					platform.SetInstantDv(axis, gb.GetFValue() * distanceScale * SecondsToMinutes); // G Code feedrates are in mm/minute; we need mm/sec
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
					platform.SetInstantDv(numTotalAxes + e, eVals[e] * distanceScale * SecondsToMinutes);
				}
			}
			else if (!seen)
			{
				reply.copy("Maximum jerk rates: ");
				for (size_t axis = 0; axis < numVisibleAxes; ++axis)
				{
					reply.catf("%c: %.1f, ", axisLetters[axis], (double)(platform.ConfiguredInstantDv(axis) / (distanceScale * SecondsToMinutes)));
				}
				reply.cat("E:");
				char sep = ' ';
				for (size_t extruder = 0; extruder < numExtruders; extruder++)
				{
					reply.catf("%c%.1f", sep, (double)(platform.ConfiguredInstantDv(extruder + numTotalAxes) / (distanceScale * SecondsToMinutes)));
					sep = ':';
				}
			}
		}
		break;

	case 567: // Set/report tool mix ratios
		if (gb.Seen('P'))
		{
			const int8_t tNumber = gb.GetIValue();
			Tool* const tool = reprap.GetTool(tNumber);
			if (tool != nullptr)
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
						reply.catf("%c%.3f", sep, (double)tool->GetMix()[drive]);
						sep = ':';
					}
				}
			}
		}
		break;

	case 568: // Turn on/off automatic tool mixing
		reply.copy("The M568 command is no longer needed");
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
					platform.SetDirectionValue(drive, gb.GetIValue() != 0);
					seen = true;
				}
				if (gb.Seen('R'))
				{
					if (!LockMovementAndWaitForStandstill(gb))
					{
						return false;
					}
					platform.SetEnableValue(drive, gb.GetIValue() != 0);
					seen = true;
				}
				if (gb.Seen('T'))
				{
					platform.SetDriverStepTiming(drive, gb.GetFValue());
					seen = true;
				}
				bool badParameter = false;
				for (size_t axis = 0; axis < numTotalAxes; ++axis)
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
					platform.Message(ErrorMessage, "M569 no longer accepts XYZE parameters; use M584 instead\n");
				}
				else if (!seen)
				{
					reply.printf("A %d sends drive %u forwards, a %d enables it, and the minimum pulse width is %.1f microseconds",
								(int)platform.GetDirectionValue(drive), drive,
								(int)platform.GetEnableValue(drive),
								(double)platform.GetDriverStepTiming(drive));
				}
			}
		}
		break;

	case 570: // Set/report heater timeout
		if (gb.Seen('H'))
		{
			const int heater = gb.GetIValue();
			bool seen = false;
			if (heater >= 0 && heater < (int)Heaters)
			{
				float maxTempExcursion, maxFaultTime;
				reprap.GetHeat().GetHeaterProtection(heater, maxTempExcursion, maxFaultTime);
				gb.TryGetFValue('P', maxFaultTime, seen);
				gb.TryGetFValue('T', maxTempExcursion, seen);
				if (seen)
				{
					reprap.GetHeat().SetHeaterProtection(heater, maxTempExcursion, maxFaultTime);
				}
				else
				{
					reply.printf("Heater %u allowed excursion %.1f" DEGREE_SYMBOL "C, fault trigger time %.1f seconds", heater, (double)maxTempExcursion, (double)maxFaultTime);
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
				if (!platform.SetExtrusionAncilliaryPwmPin(pwmPin))
				{
					reply.printf("Logical pin %d is already in use or not available for use by M571", pwmPin);
					break;			// don't process 'S' parameter if the pin was wrong
				}
				seen = true;
			}
			if (gb.Seen('F'))
			{
				platform.SetExtrusionAncilliaryPwmFrequency(gb.GetFValue());
			}
			if (gb.Seen('S'))
			{
				platform.SetExtrusionAncilliaryPwmValue(gb.GetFValue());
				seen = true;
			}
			if (!seen)
			{
				reply.printf("Extrusion ancillary PWM %.3f at %.1fHz on pin %u",
								(double)platform.GetExtrusionAncilliaryPwmValue(),
								(double)platform.GetExtrusionAncilliaryPwmFrequency(),
								platform.GetExtrusionAncilliaryPwmPin());
			}
		}
		break;

	case 572: // Set/report pressure advance
		if (gb.Seen('S'))
		{
			const float advance = gb.GetFValue();
			if (gb.Seen('D'))
			{
				long int eDrive[MaxExtruders];
				size_t eCount = MaxExtruders;
				gb.GetLongArray(eDrive, eCount);
				for (size_t i = 0; i < eCount; i++)
				{
					if (eDrive[i] < 0 || (size_t)eDrive[i] >= numExtruders)
					{
						reply.printf("Invalid extruder number '%ld'", eDrive[i]);
						result = GCodeResult::error;
						break;
					}
					platform.SetPressureAdvance(eDrive[i], advance);
				}
			}
		}
		else
		{
			reply.copy("Extruder pressure advance");
			char c = ':';
			for (size_t i = 0; i < numExtruders; ++i)
			{
				reply.catf("%c %.3f", c, (double)platform.GetPressureAdvance(i));
				c = ',';
			}
		}
		break;

	case 573: // Report heater average PWM
		if (gb.Seen('P'))
		{
			const int heater = gb.GetIValue();
			if (heater >= 0 && heater < (int)Heaters)
			{
				reply.printf("Average heater %d PWM: %.3f", heater, (double)reprap.GetHeat().GetAveragePWM(heater));
			}
		}
		break;

	case 574: // Set endstop configuration
		{
			bool seen = false;
			const bool logicLevel = (gb.Seen('S')) ? (gb.GetIValue() != 0) : true;
			for (size_t axis = 0; axis < numTotalAxes; ++axis)
			{
				if (gb.Seen(axisLetters[axis]))
				{
					const int ival = gb.GetIValue();
					if (ival >= 0 && ival <= 3)
					{
						platform.SetEndStopConfiguration(axis, (EndStopType) ival, logicLevel);
						seen = true;
					}
				}
			}
			if (!seen)
			{
				reply.copy("Endstop configuration:");
				EndStopType config;
				bool logic;
				for (size_t axis = 0; axis < numTotalAxes; ++axis)
				{
					platform.GetEndStopConfiguration(axis, config, logic);
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
					platform.SetBaudRate(chan, gb.GetIValue());
					seen = true;
				}
				if (gb.Seen('S'))
				{
					uint32_t val = gb.GetIValue();
					platform.SetCommsProperties(chan, val);
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
					uint32_t cp = platform.GetCommsProperties(chan);
					reply.printf("Channel %d: baud rate %" PRIu32 ", %s checksum", chan, platform.GetBaudRate(chan), (cp & 1) ? "requires" : "does not require");
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
			for (size_t axis=0; axis < numTotalAxes; axis++)
			{
				if (gb.Seen(axisLetters[axis]))
				{
					if (platform.Stopped(axis) != triggerCondition)
					{
						result = GCodeResult::notFinished;
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
						result = GCodeResult::error;
						break;
					}

					if (platform.Stopped(eDrive + E0_AXIS) != triggerCondition)
					{
						result = GCodeResult::notFinished;
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
			platform.Inkjet(gb.GetIValue());
		}
		break;
#endif

	case 579: // Scale Cartesian axes (mostly for Delta configurations)
		{
			bool seen = false;
			for (size_t axis = 0; axis < numVisibleAxes; axis++)
			{
				gb.TryGetFValue(axisLetters[axis], axisScaleFactors[axis], seen);
			}

			if (!seen)
			{
				char sep = ':';
				reply.copy("Axis scale factors");
				for(size_t axis = 0; axis < numVisibleAxes; axis++)
				{
					reply.catf("%c %c: %.3f", sep, axisLetters[axis], (double)axisScaleFactors[axis]);
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
					uint32_t states = platform.GetAllEndstopStates();
					if ((triggers[triggerNumber].rising & states) != 0 || (triggers[triggerNumber].falling & ~states) != 0)
					{
						SetBit(triggersPending, triggerNumber);
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
						TriggerInputsBitmap triggerMask = 0;
						for (size_t axis = 0; axis < numTotalAxes; ++axis)
						{
							if (gb.Seen(axisLetters[axis]))
							{
								SetBit(triggerMask, axis);
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
									SetBit(triggerMask, eStops[i] + E0_AXIS);
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
							platform.Message(ErrorMessage, "Bad S parameter in M581 command\n");
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
				reply.copy("Trigger number out of range");
				result = GCodeResult::error;
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
			for (size_t drive = 0; drive < MaxAxes; ++drive)
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
						while (numTotalAxes <= drive)
						{
							moveBuffer.coords[numTotalAxes] = 0.0;		// user has defined a new axis, so set its position
							currentUserPosition[numTotalAxes] = 0.0;	// set its requested user position too in case it is visible
							++numTotalAxes;
						}
						numVisibleAxes = numTotalAxes;					// assume all axes are visible unless there is a P parameter
						reprap.GetMove().SetNewPosition(moveBuffer.coords, true);	// tell the Move system where any new axes are
						platform.SetAxisDriversConfig(drive, config);
						if (numTotalAxes + numExtruders > DRIVES)
						{
							numExtruders = DRIVES - numTotalAxes;		// if we added axes, we may have fewer extruders now
						}
					}
				}
			}

			if (gb.Seen(extrudeLetter))
			{
				seen = true;
				size_t numValues = DRIVES - numTotalAxes;
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
						platform.SetExtruderDriver(i, (uint8_t)drivers[i]);
					}
				}
			}

			if (badDrive)
			{
				reply.copy("Invalid drive number in M584 command");
				result = GCodeResult::error;
			}
			else
			{
				if (gb.Seen('P'))
				{
					seen = true;
					const int nva = gb.GetIValue();
					if (nva >= (int)MinAxes && (unsigned int)nva <= numTotalAxes)
					{
						numVisibleAxes = (size_t)nva;
					}
					else
					{
						reply.copy("Invalid number of visible axes in M584 command");
						result = GCodeResult::error;
					}
				}

				if (!seen)
				{
					reply.copy("Driver assignments:");
					for (size_t drive = 0; drive < numTotalAxes; ++ drive)
					{
						reply.cat(' ');
						const AxisDriversConfig& axisConfig = platform.GetAxisDriversConfig(drive);
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
						reply.catf("%c%u", c, platform.GetExtruderDriver(extruder));
						c = ':';
					}
				}
			}
		}
		break;

	case 586: // Configure network protocols
		if (gb.Seen('P'))
		{
			const int protocol = gb.GetIValue();
			if (gb.Seen('S'))
			{
				const bool enable = (gb.GetIValue() == 1);
				if (enable)
				{
					const int port = (gb.Seen('R')) ? gb.GetIValue() : -1;
					const int secure = (gb.Seen('T')) ? gb.GetIValue() : -1;
					reprap.GetNetwork().EnableProtocol(protocol, port, secure, reply);
				}
				else
				{
					reprap.GetNetwork().DisableProtocol(protocol, reply);
				}
				break;
			}
		}

		// Default to reporting current protocols if P or S parameter missing
		reprap.GetNetwork().ReportProtocols(reply);
		break;

#ifdef DUET_WIFI
	case 587:	// Add WiFi network or list remembered networks
		if (gb.Seen('S'))
		{
			WirelessConfigurationData config;
			memset(&config, 0, sizeof(config));
			String<ARRAY_SIZE(config.ssid)> ssid;
			bool ok = gb.GetQuotedString(ssid.GetRef());
			if (ok)
			{
				SafeStrncpy(config.ssid, ssid.c_str(), ARRAY_SIZE(config.ssid));
				String<ARRAY_SIZE(config.password)> password;
				ok = gb.Seen('P') && gb.GetQuotedString(password.GetRef());
				if (ok)
				{
					SafeStrncpy(config.password, password.c_str(), ARRAY_SIZE(config.password));
				}
			}
			if (ok && gb.Seen('I'))
			{
				ok = gb.GetIPAddress(config.ip);
			}
			if (ok && gb.Seen('J'))
			{
				ok = gb.GetIPAddress(config.gateway);
			}
			if (ok && gb.Seen('K'))
			{
				ok = gb.GetIPAddress(config.netmask);
			}
			if (ok)
			{
				const int32_t rslt = reprap.GetNetwork().SendCommand(NetworkCommand::networkAddSsid, 0, 0, &config, sizeof(config), nullptr, 0);
				if (rslt != ResponseEmpty)
				{
					reply.copy("Failed to add SSID to remembered list");
					result = GCodeResult::error;
				}
			}
			else
			{
				reply.copy("Bad parameter in M587 command");
				result = GCodeResult::error;
			}
		}
		else
		{
			// List remembered networks
			const size_t declaredBufferLength = (MaxRememberedNetworks + 1) * (SsidLength + 1) + 1;	// enough for all the remembered SSIDs with newline terminator, plus an extra null
			uint32_t buffer[NumDwords(declaredBufferLength + 1)];
			const int32_t rslt = reprap.GetNetwork().SendCommand(NetworkCommand::networkListSsids, 0, 0, nullptr, 0, buffer, declaredBufferLength);
			if (rslt >= 0)
			{
				char* const cbuf = reinterpret_cast<char *>(buffer);
				cbuf[declaredBufferLength] = 0;						// ensure null terminated

				// DuetWiFiServer 1.19beta7 and later include the SSID used in access point mode at the start
				char *bufp = strchr(cbuf, '\n');
				if (bufp == nullptr)
				{
					bufp = cbuf;			// must be an old version of DuetWiFiServer
				}
				else
				{
					++bufp;					// slip the first entry
				}

				// If there is a trailing newline, remove it
				{
					const size_t len = strlen(bufp);
					if (len != 0 && bufp[len - 1] == '\n')
					{
						bufp[len - 1] = 0;
					}
				}

				if (strlen(bufp) == 0)
				{
					reply.copy("No remembered networks");
				}
				else
				{
					OutputBuffer *response;
					if (!OutputBuffer::Allocate(response))
					{
						return false;		// try again later
					}
					response->copy("Remembered networks:\n");
					response->cat(bufp);
					HandleReply(gb, false, response);
					return true;
				}
			}
			else
			{
				reply.copy("Failed to retrieve network list");
				result = GCodeResult::error;
			}
		}
		break;

	case 588:	// Forget WiFi network
		if (gb.Seen('S'))
		{
			String<SsidLength> ssidText;
			if (gb.GetQuotedString(ssidText.GetRef()))
			{
				if (strcmp(ssidText.c_str(), "*") == 0)
				{
					const int32_t rslt = reprap.GetNetwork().SendCommand(NetworkCommand::networkFactoryReset, 0, 0, nullptr, 0, nullptr, 0);
					if (rslt != ResponseEmpty)
					{
						reply.copy("Failed to reset the WiFi module to factory settings");
						result = GCodeResult::error;
					}
				}
				else
				{
					uint32_t ssid32[NumDwords(SsidLength)];				// need a dword-aligned buffer for SendCommand
					memcpy(ssid32, ssidText.c_str(), SsidLength);
					const int32_t rslt = reprap.GetNetwork().SendCommand(NetworkCommand::networkDeleteSsid, 0, 0, ssid32, SsidLength, nullptr, 0);
					if (rslt != ResponseEmpty)
					{
						reply.copy("Failed to remove SSID from remembered list");
						result = GCodeResult::error;
					}
				}
			}
			else
			{
				reply.copy("Bad parameter in M588 command");
				result = GCodeResult::error;
			}
		}
		break;

	case 589:	// Configure access point
		if (gb.Seen('S'))
		{
			WirelessConfigurationData config;
			memset(&config, 0, sizeof(config));
			String<SsidLength> ssid;
			bool ok = gb.GetQuotedString(ssid.GetRef());
			if (ok)
			{
				if (strcmp(ssid.c_str(), "*") == 0)
				{
					// Delete the access point details
					memset(&config, 0xFF, sizeof(config));
				}
				else
				{
					SafeStrncpy(config.ssid, ssid.c_str(), ARRAY_SIZE(config.ssid));
					String<ARRAY_SIZE(config.password)> password;
					ok = gb.Seen('P') && gb.GetQuotedString(password.GetRef());
					if (ok)
					{
						SafeStrncpy(config.password, password.c_str(), ARRAY_SIZE(config.password));
						if (gb.Seen('I'))
						{
							ok = gb.GetIPAddress(config.ip);
							config.channel = (gb.Seen('C')) ? gb.GetIValue() : 0;
						}
					}
					else
					{
						ok = false;
					}
				}
			}
			if (ok)
			{
				const int32_t rslt = reprap.GetNetwork().SendCommand(NetworkCommand::networkConfigureAccessPoint, 0, 0, &config, sizeof(config), nullptr, 0);
				if (rslt != ResponseEmpty)
				{
					reply.copy("Failed to configure access point parameters");
					result = GCodeResult::error;
				}
			}
			else
			{
				reply.copy("Bad or missing parameter in M589 command");
				result = GCodeResult::error;
			}
		}
		else
		{
			const size_t declaredBufferLength = (MaxRememberedNetworks + 1) * (SsidLength + 1) + 1;	// enough for all the remembered SSIDs with null terminator, plus an extra null
			uint32_t buffer[NumDwords(declaredBufferLength + 1)];
			const int32_t rslt = reprap.GetNetwork().SendCommand(NetworkCommand::networkListSsids, 0, 0, nullptr, 0, buffer, declaredBufferLength);
			if (rslt >= 0)
			{
				char* const cbuf = reinterpret_cast<char *>(buffer);
				cbuf[declaredBufferLength] = 0;						// ensure null terminated
				char *p = strchr(cbuf, '\n');
				if (p != nullptr)
				{
					*p = 0;
				}
				reply.printf("Own SSID: %s", (cbuf[0] == 0) ? "not configured" : cbuf);
			}
			else
			{
				reply.copy("Failed to remove SSID from remembered list");
				result = GCodeResult::error;
			}
		}
		break;
#endif

	case 591: // Configure filament sensor
		if (gb.Seen('D'))
		{
			int extruder = gb.GetIValue();
			if (extruder >= 0 && extruder < (int)numExtruders)
			{
				bool seen = false;
				long sensorType;
				gb.TryGetIValue('P', sensorType, seen);
				if (seen)
				{
					FilamentSensor::SetFilamentSensorType(extruder, sensorType);
				}

				FilamentSensor *sensor = FilamentSensor::GetFilamentSensor(extruder);
				if (sensor != nullptr)
				{
					// Configure the sensor
					const bool error = sensor->Configure(gb, reply, seen);
					result = GetGCodeResultFromError(error);
					if (error)
					{
						FilamentSensor::SetFilamentSensorType(extruder, 0);		// delete the sensor
					}
				}
				else if (!seen)
				{
					reply.printf("Extruder drive %d has no filament sensor", extruder);
				}
			}
		}
		break;

	case 593: // Configure filament properties
		// TODO: We may need this code later to restrict specific filaments to certain tools or to reset filament counters.
		break;

	case 665: // Set delta configuration
		if (!LockMovementAndWaitForStandstill(gb))
		{
			return false;
		}
		{
			Move& move = reprap.GetMove();

			bool changedMode = false;
			if ((gb.Seen('L') || gb.Seen('D')) && move.GetKinematics().GetKinematicsType() != KinematicsType::linearDelta)
			{
				// Not in delta mode, so switch to it
				changedMode = true;
				move.SetKinematics(KinematicsType::linearDelta);
			}
			bool error = false;
			const bool changed = move.GetKinematics().Configure(code, gb, reply, error);
			if (changedMode)
			{
				move.GetKinematics().GetAssumedInitialPosition(numVisibleAxes, moveBuffer.coords);
				ToolOffsetInverseTransform(moveBuffer.coords, currentUserPosition);
			}
			if (changed || changedMode)
			{
				if (reprap.GetMove().GetKinematics().LimitPosition(moveBuffer.coords, numVisibleAxes, axesHomed))
				{
					ToolOffsetInverseTransform(moveBuffer.coords, currentUserPosition);	// make sure the limits are reflected in the user position
				}
				reprap.GetMove().SetNewPosition(moveBuffer.coords, true);
				SetAllAxesNotHomed();
			}
			result = GetGCodeResultFromError(error);
		}
		break;

	case 666: // Set delta endstop adjustments
		if (!LockMovementAndWaitForStandstill(gb))
		{
			return false;
		}
		{
			bool error = false;
			const bool changed = reprap.GetMove().GetKinematics().Configure(code, gb, reply, error);
			if (changed)
			{
				SetAllAxesNotHomed();
			}
			result = GetGCodeResultFromError(error);
		}
		break;

	case 667: // Set CoreXY mode
		if (!LockMovementAndWaitForStandstill(gb))
		{
			return false;
		}
		{
			Move& move = reprap.GetMove();
			const KinematicsType oldK = move.GetKinematics().GetKinematicsType();		// get the current kinematics type so we can tell whether it changed

			bool seen = false;
			bool changedToCartesian = false;
			if (gb.Seen('S'))
			{
				// Switch to the correct CoreXY mode
				const int mode = gb.GetIValue();
				switch (mode)
				{
				case 0:
					move.SetKinematics(KinematicsType::cartesian);
					changedToCartesian = true;
					break;

				case 1:
					move.SetKinematics(KinematicsType::coreXY);
					break;

				case 2:
					move.SetKinematics(KinematicsType::coreXZ);
					break;

				default:
					reply.printf("Mode %d is not valid in M667 command\n", mode);
					result = GCodeResult::error;
					break;
				}
				seen = true;
			}

			if (result == GCodeResult::ok)
			{
				if (!changedToCartesian)		// don't ask the kinematics to process M667 if we switched to Cartesian mode
				{
					bool error = false;
					if (move.GetKinematics().Configure(667, gb, reply, error))
					{
						seen = true;
					}
					result = GetGCodeResultFromError(error);
				}

				if (seen)
				{
					// We changed something, so reset the positions and set all axes not homed
					if (move.GetKinematics().GetKinematicsType() != oldK)
					{
						move.GetKinematics().GetAssumedInitialPosition(numVisibleAxes, moveBuffer.coords);
						ToolOffsetInverseTransform(moveBuffer.coords, currentUserPosition);
					}
					if (reprap.GetMove().GetKinematics().LimitPosition(moveBuffer.coords, numVisibleAxes, axesHomed))
					{
						ToolOffsetInverseTransform(moveBuffer.coords, currentUserPosition);	// make sure the limits are reflected in the user position
					}
					reprap.GetMove().SetNewPosition(moveBuffer.coords, true);
					SetAllAxesNotHomed();
				}
			}
		}
		break;

	case 669:	// Set kinematics and parameters for SCARA and other kinematics that don't use M665, M666 or M667
		if (!LockMovementAndWaitForStandstill(gb))
		{
			return false;
		}
		{
			Move& move = reprap.GetMove();
			const KinematicsType oldK = move.GetKinematics().GetKinematicsType();		// get the current kinematics type so we can tell whether it changed

			bool seen = false;
			if (gb.Seen('K'))
			{
				const int nk = gb.GetIValue();
				if (nk < 0 || nk >= (int)KinematicsType::unknown || !move.SetKinematics(static_cast<KinematicsType>(nk)))
				{
					reply.printf("Unknown kinematics type %d in M669 command", nk);
					result = GCodeResult::error;
					break;
				}
				seen = true;
			}
			bool error = false;
			if (move.GetKinematics().Configure(code, gb, reply, error))
			{
				seen = true;
			}
			result = GetGCodeResultFromError(error);

			if (seen)
			{
				// We changed something significant, so reset the positions and set all axes not homed
				if (move.GetKinematics().GetKinematicsType() != oldK)
				{
					move.GetKinematics().GetAssumedInitialPosition(numVisibleAxes, moveBuffer.coords);
					ToolOffsetInverseTransform(moveBuffer.coords, currentUserPosition);
				}
				if (reprap.GetMove().GetKinematics().LimitPosition(moveBuffer.coords, numVisibleAxes, axesHomed))
				{
					ToolOffsetInverseTransform(moveBuffer.coords, currentUserPosition);	// make sure the limits are reflected in the user position
				}
				reprap.GetMove().SetNewPosition(moveBuffer.coords, true);
				SetAllAxesNotHomed();
			}
		}
		break;

#if SUPPORT_IOBITS
	case 670:
		result = GetGCodeResultFromError(reprap.GetPortControl().Configure(gb, reply));
		break;
#endif

	case 671:	// Set Z leadscrew positions
		if (!LockMovementAndWaitForStandstill(gb))
		{
			return false;
		}
		{
			bool error = false;
			(void)reprap.GetMove().GetKinematics().Configure(code, gb, reply, error);
			result = GetGCodeResultFromError(error);
		}
		break;

	case 672: // Program Z probe
		result = GetGCodeResultFromError(platform.ProgramZProbe(gb, reply));
		break;

	case 701: // Load filament
		result = LoadFilament(gb, reply);
		break;

	case 702: // Unload filament
		result = UnloadFilament(gb, reply);
		break;

#if SUPPORT_SCANNER
	case 750: // Enable 3D scanner extension
		reprap.GetScanner().Enable();
		break;

	case 751: // Register 3D scanner extension over USB
		if (&gb == serialGCode)
		{
			if (reprap.GetScanner().IsEnabled())
			{
				result = GetGCodeResultFromFinished(reprap.GetScanner().Register());
			}
			else
			{
				reply.copy("Scanner extension is not enabled");
				result = GCodeResult::error;
			}
		}
		else
		{
			reply.copy("Invalid source for this M-code");
			result = GCodeResult::error;
		}
		break;

	case 752: // Start 3D scan
		if (gb.Seen('P'))
		{
			const char *file = gb.GetString();
			if (gb.Seen('S'))
			{
				const int sParam = gb.GetIValue();
				if (reprap.GetScanner().IsEnabled())
				{
					if (reprap.GetScanner().IsRegistered())
					{
						result = GetGCodeResultFromFinished(reprap.GetScanner().StartScan(file, sParam));
					}
					else
					{
						reply.copy("Scanner is not registered");
						result = GCodeResult::error;
					}
				}
				else
				{
					reply.copy("Scanner extension is not enabled");
					result = GCodeResult::error;
				}
			}
			else
			{
				reply.copy("Missing length/degree parameter");
				result = GCodeResult::error;
			}
		}
		else
		{
			reply.copy("Missing filename");
			result = GCodeResult::error;
		}
		break;

	case 753: // Cancel current 3D scanner action
		if (reprap.GetScanner().IsEnabled())
		{
			if (reprap.GetScanner().IsRegistered())
			{
				result = GetGCodeResultFromFinished(reprap.GetScanner().Cancel());
			}
			else
			{
				reply.copy("Scanner is not registered");
				result = GCodeResult::error;
			}
		}
		else
		{
			reply.copy("Scanner extension is not enabled");
			result = GCodeResult::error;
		}
		break;

	case 754: // Calibrate scanner
		if (reprap.GetScanner().IsEnabled())
		{
			if (reprap.GetScanner().IsRegistered())
			{
				result = GetGCodeResultFromFinished(reprap.GetScanner().Calibrate());
			}
			else
			{
				reply.copy("Scanner is not registered");
				result = GCodeResult::error;
			}
		}
		else
		{
			reply.copy("Scanner extension is not enabled");
			result = GCodeResult::error;
		}
		break;

	case 755: // Set alignment mode for 3D scanner
		if (reprap.GetScanner().IsEnabled())
		{
			if (reprap.GetScanner().IsRegistered())
			{
				const bool on = (gb.Seen('P') && gb.GetIValue() > 0);
				result = GetGCodeResultFromFinished(reprap.GetScanner().SetAlignment(on));
			}
			else
			{
				reply.copy("Scanner is not registered");
				result = GCodeResult::error;
			}
		}
		else
		{
			reply.copy("Scanner extension is not enabled");
			result = GCodeResult::error;
		}
		break;

	case 756: // Shutdown 3D scanner
		if (reprap.GetScanner().IsEnabled())
		{
			if (reprap.GetScanner().IsRegistered())
			{
				result = GetGCodeResultFromFinished(reprap.GetScanner().Shutdown());
			}
			else
			{
				reply.copy("Scanner is not registered");
				result = GCodeResult::error;
			}
		}
		else
		{
			reply.copy("Scanner extension is not enabled");
			result = GCodeResult::error;
		}
		break;
#else
	case 750:
	case 751:
	case 752:
	case 753:
	case 754:
	case 755:
	case 756:
		reply.copy("Scanner support not built-in");
		result = GCodeResult::error;
		break;
#endif

	case 905: // Set current RTC date and time
		{
			const time_t now = platform.GetDateTime();
			struct tm timeInfo;
			gmtime_r(&now, &timeInfo);
			bool seen = false;

			if (gb.Seen('P'))
			{
				seen = true;

				// Set date
				const char * const dateString = gb.GetString();
				if (strptime(dateString, "%Y-%m-%d", &timeInfo) == nullptr)
				{
					reply.copy("M905: Invalid date format");
					result = GCodeResult::error;
					break;
				}
			}

			if (gb.Seen('S'))
			{
				seen = true;

				// Set time
				const char * const timeString = gb.GetString();
				if (strptime(timeString, "%H:%M:%S", &timeInfo) == nullptr)
				{
					reply.copy("M905: Invalid time format");
					result = GCodeResult::error;
					break;
				}
			}

			if (seen)
			{
				platform.SetDateTime(mktime(&timeInfo));
			}
			else
			{
				// Report current date and time
				if (platform.IsDateTimeSet())
				{
					reply.printf("Current date and time: %04u-%02u-%02u %02u:%02u:%02u",
							timeInfo.tm_year + 1900, timeInfo.tm_mon + 1, timeInfo.tm_mday,
							timeInfo.tm_hour, timeInfo.tm_min, timeInfo.tm_sec);
				}
				else
				{
					reply.copy("Clock has not been set");
				}
			}
		}
		break;

	case 906: // Set/report Motor currents
	case 913: // Set/report motor current percent
		{
			bool seen = false;
			for (size_t axis = 0; axis < numTotalAxes; axis++)
			{
				if (gb.Seen(axisLetters[axis]))
				{
					if (!LockMovementAndWaitForStandstill(gb))
					{
						return false;
					}
					platform.SetMotorCurrent(axis, gb.GetFValue(), code == 913);
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
					platform.SetMotorCurrent(numTotalAxes + e, eVals[e], code == 913);
				}
				seen = true;
			}

			if (code == 906 && gb.Seen('I'))
			{
				const float idleFactor = gb.GetFValue();
				if (idleFactor >= 0 && idleFactor <= 100.0)
				{
					platform.SetIdleCurrentFactor(idleFactor/100.0);
					seen = true;
				}
			}

			if (!seen)
			{
				reply.copy((code == 913) ? "Motor current % of normal - " : "Motor current (mA) - ");
				for (size_t axis = 0; axis < numTotalAxes; ++axis)
				{
					reply.catf("%c:%d, ", axisLetters[axis], (int)platform.GetMotorCurrent(axis, code == 913));
				}
				reply.cat("E");
				for (size_t extruder = 0; extruder < numExtruders; extruder++)
				{
					reply.catf(":%d", (int)platform.GetMotorCurrent(extruder + numTotalAxes, code == 913));
				}
				if (code == 906)
				{
					reply.catf(", idle factor %d%%", (int)(platform.GetIdleCurrentFactor() * 100.0));
				}
			}
		}
		break;

#ifdef DUET_NG
	case 911: // Enable auto save
		result = GetGCodeResultFromError(platform.ConfigureAutoSave(gb, reply));
		break;
#endif

	case 912: // Set electronics temperature monitor adjustment
		// Currently we ignore the P parameter (i.e. temperature measurement channel)
		if (gb.Seen('S'))
		{
			platform.SetMcuTemperatureAdjust(gb.GetFValue());
		}
		else
		{
			reply.printf("MCU temperature calibration adjustment is %.1f" DEGREE_SYMBOL "C", (double)platform.GetMcuTemperatureAdjust());
		}
		break;

	// For case 913, see 906

#if defined(__ALLIGATOR__)
	case 914: 				// Set/Get J14 Expansion Voltage Level Translator on Port J5, 5.5V or 3.3V
			  	  	  	  	// Get Piggy module presence status
		if (gb.Seen('S'))
		{
			const int voltageValue = gb.GetIValue();
			if (voltageValue != 5 && voltageValue != 3 )
			{
				reply.printf("The Expansion Voltage Translator does not support %dV. \n Only 5V or 3V are supported.",voltageValue);
			}
			else
			{
				// Change Voltage translator level
				digitalWrite(ExpansionVoltageLevelPin, voltageValue == 5);
			}
		}
		else
		{
			// Change Voltage translator level Status
			reply.printf("The Voltage of Expansion Translator is %dV \nPiggy module %s",
					digitalRead(ExpansionVoltageLevelPin) ? 5 : 3 ,
					digitalRead(ExpansionPiggyDetectPin) ? "not detected" : "detected");
		}
		break;
#endif

	case 929: // Start/stop event logging
		result = GetGCodeResultFromError(platform.ConfigureLogging(gb, reply));
		break;

	case 997: // Perform firmware update
		if (!LockMovementAndWaitForStandstill(gb))
		{
			return false;
		}
		reprap.GetHeat().SwitchOffAll();					// turn all heaters off because the main loop may get suspended
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
						platform.MessageF(ErrorMessage, "Invalid module number '%ld'\n", t);
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
			if ((firmwareUpdateModuleMap & 1) != 0 && !platform.CheckFirmwareUpdatePrerequisites())
			{
				firmwareUpdateModuleMap = 0;
				break;
			}
		}

		// If we get here then we have the module map, and all prerequisites are satisfied
		isFlashing = true;										// this tells the web interface and PanelDue that we are about to flash firmware
		if (DoDwellTime(gb, 1000) == GCodeResult::notFinished)	// wait a second so all HTTP clients and PanelDue are notified
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
		result = DoDwellTime(gb, 500);		// wait half a second to allow the response to be sent back to the web server, otherwise it may retry
		if (result != GCodeResult::notFinished)
		{
			reprap.EmergencyStop();			// this disables heaters and drives - Duet WiFi pre-production boards need drives disabled here
			uint16_t reason = (gb.Seen('P') && StringStartsWith(gb.GetString(), "ERASE"))
											? (uint16_t)SoftwareResetReason::erase
											: (uint16_t)SoftwareResetReason::user;
			platform.SoftwareReset(reason);			// doesn't return
		}
		break;

	default:
		reply.printf("Unsupported command: %s", gb.Buffer());
		result = GCodeResult::error;
		break;
	}

	if (result == GCodeResult::notFinished)
	{
		return false;
	}

	if (result == GCodeResult::notSupportedInCurrentMode)
	{
		reply.printf("M%d command is not supported in machine mode %s", code, GetMachineModeString());
	}

	if (gb.GetState() == GCodeState::normal)
	{
		gb.timerRunning = false;
		UnlockAll(gb);
		HandleReply(gb, result != GCodeResult::ok, reply.Pointer());
	}
	return true;
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

	if (strlen(gb.Buffer()) > 1)
	{
		// See if the tool can be changed
		newToolNumber = gb.GetIValue();
		newToolNumber += gb.GetToolNumberAdjust();

		const Tool * const oldTool = reprap.GetCurrentTool();
		// If old and new are the same we no longer follow the sequence. User can deselect and then reselect the tool if he wants the macros run.
		if (oldTool == nullptr || oldTool->Number() != newToolNumber)
		{
			toolChangeParam = (simulationMode != 0) ? 0
								: gb.Seen('P') ? gb.GetIValue()
									: DefaultToolChangeParam;
			gb.SetState(GCodeState::toolChange0);
			return true;							// proceeding with state machine, so don't unlock or send a reply
		}
	}
	else
	{
		// Report the tool number in use if no parameter is passed
		const Tool * const tool = reprap.GetCurrentTool();
		if (tool == nullptr)
		{
			reply.copy("No tool is selected.");
		}
		else
		{
			reply.printf("Tool %d is selected.", tool->Number());
		}
	}

	// If we get here, we have finished
	UnlockAll(gb);
	HandleReply(gb, false, reply.Pointer());
	return true;
}

// End
