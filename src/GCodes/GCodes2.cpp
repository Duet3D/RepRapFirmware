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
#include "FilamentMonitors/FilamentMonitor.h"
#include "Libraries/General/IP4String.h"
#include "Version.h"

#if SUPPORT_IOBITS
# include "PortControl.h"
#endif

#if HAS_WIFI_NETWORKING
# include "FirmwareUpdater.h"
#endif

#if SUPPORT_12864_LCD
# include "Display/Display.h"
#endif

#include <utility>			// for std::swap

// If the code to act on is completed, this returns true, otherwise false.
// It is called repeatedly for a given code until it returns true for that code.
bool GCodes::ActOnCode(GCodeBuffer& gb, const StringRef& reply)
{
	// Can we queue this code?
	if (gb.CanQueueCodes() && codeQueue->ShouldQueueCode(gb))
	{
		// Don't queue any GCodes if there are segments not yet picked up by Move, because in the event that a segment corresponds to no movement,
		// the move gets discarded, which throws out the count of scheduled moves and hence the synchronisation
		if (segmentsLeft != 0)
		{
			return false;
		}

		if (codeQueue->QueueCode(gb))
		{
			HandleReply(gb, false, "");
			return true;
		}
	}

	switch (gb.GetCommandLetter())
	{
	case 'G':
		if (gb.HasCommandNumber())
		{
			return HandleGcode(gb, reply);
		}
		break;

	case 'M':
		if (gb.HasCommandNumber())
		{
			return HandleMcode(gb, reply);
		}
		break;

	case 'T':
		return HandleTcode(gb, reply);

	default:
		break;
	}

	reply.printf("Bad command: %s", gb.Buffer());
	HandleReply(gb, true, reply.Pointer());
	return true;
}

bool GCodes::HandleGcode(GCodeBuffer& gb, const StringRef& reply)
{
	GCodeResult result = GCodeResult::ok;
	const int code = gb.GetCommandNumber();
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
	case 0: // Rapid move
	case 1: // Ordinary move
		if (segmentsLeft != 0)			// do this check first to avoid locking movement unnecessarily
		{
			return false;
		}
		if (!LockMovement(gb))
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
		if (segmentsLeft != 0)			// do this check first to avoid locking movement unnecessarily
		{
			return false;
		}
		if (!LockMovement(gb))
		{
			return false;
		}
		if (DoArcMove(gb, code == 2))
		{
			reply.copy("Invalid arc parameters");
			result = GCodeResult::error;
		}
		break;

	case 4: // Dwell
		result = DoDwell(gb);
		break;

	case 10: // Set/report offsets and temperatures, or retract
		{
#if SUPPORT_WORKPLACE_COORDINATES
			if (gb.Seen('L'))
			{
				if (gb.GetUIValue() == 2)
				{
					result = GetSetWorkplaceCoordinates(gb, reply);
				}
				else
				{
					result = GCodeResult::badOrMissingParameter;
				}
			}
			else
#endif
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

#if SUPPORT_WORKPLACE_COORDINATES
	case 53:	// Switch to coordinate system 0
	case 54:	// Switch to coordinate system 1
	case 55:	// Switch to coordinate system 2
	case 56:	// Switch to coordinate system 3
	case 57:	// Switch to coordinate system 4
	case 58:	// Switch to coordinate system 5
	case 59:	// Switch to coordinate system 6,7,8,9
		{
			unsigned int cs = code - 53;
			if (code == 59)
			{
				const int8_t fraction = gb.GetCommandFraction();
				if (fraction >= 0)
				{
					cs += (unsigned int) fraction;
				}
			}
			if (cs < ARRAY_SIZE(workplaceCoordinates))
			{
				currentCoordinateSystem = cs;
			}
			else
			{
				result = GCodeResult::notSupported;
			}
		}
		break;
#endif

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
		result = GCodeResult::notSupported;
	}

	return HandleResult(gb, result, reply);
}

bool GCodes::HandleMcode(GCodeBuffer& gb, const StringRef& reply)
{
	GCodeResult result = GCodeResult::ok;

	const int code = gb.GetCommandNumber();
	if (   simulationMode != 0
		&& (code < 20 || code > 37)
		&& code != 0 && code != 1 && code != 82 && code != 83 && code != 105 && code != 109 && code != 111 && code != 112 && code != 122
		&& code != 200 && code != 204 && code != 207 && code != 408 && code != 999)
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
		// Don't allow M0 or M1 to stop a print, unless the print is paused or the command comes from the file being printed itself.
		if (reprap.GetPrintMonitor().IsPrinting() && &gb != fileGCode && !IsPaused())
		{
			reply.copy("Pause the print before attempting to cancel it");
			result = GCodeResult::error;
		}
		else
		{
			if (   !LockMovementAndWaitForStandstill(gb)	// wait until everything has stopped
				|| !IsCodeQueueIdle()						// must also wait until deferred command queue has caught up
			   )
			{
				return false;
			}

			const bool wasPaused = isPaused;			// isPaused gets cleared by CancelPrint
			const bool wasSimulating = IsSimulating();	// simulationMode may get cleared by CancelPrint
			StopPrint(&gb == fileGCode);				// if this is normal end-of-print commanded by the file, delete the resurrect.g file

			if (!wasSimulating)							// don't run any macro files or turn heaters off etc. if we were simulating before we stopped the print
			{
				// If we are cancelling a paused print with M0 and we are homed and cancel.g exists then run it and do nothing else
				if (wasPaused && code == 0 && AllAxesAreHomed() && DoFileMacro(gb, CANCEL_G, false))
				{
					break;
				}

				gb.SetState((code == 0) ? GCodeState::stopping : GCodeState::sleeping);
				DoFileMacro(gb, (code == 0) ? STOP_G : SLEEP_G, false);
			}
		}
		break;

	case 3: // Spin spindle clockwise
		if (gb.Seen('S'))
		{
			switch (machineType)
			{
			case MachineType::cnc:
				platform.SetSpindlePwm(gb.GetFValue()/spindleMaxRpm);
				break;

			case MachineType::laser:
				platform.SetLaserPwm(gb.GetFValue()/laserMaxPower);
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
				platform.SetSpindlePwm(-gb.GetFValue()/spindleMaxRpm);
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
			platform.SetSpindlePwm(0.0);
			break;

		case MachineType::laser:
			platform.SetLaserPwm(0.0);
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
				uint32_t eDrive[MaxExtruders];
				size_t eCount = numExtruders;
				gb.GetUnsignedArray(eDrive, eCount, false);
				for (size_t i = 0; i < eCount; i++)
				{
					seen = true;
					if (eDrive[i] >= numExtruders)
					{
						reply.printf("Invalid extruder number specified: %" PRIu32, eDrive[i]);
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
			String<MaxFilenameLength> dir;
			if (gb.Seen('P'))
			{
				gb.GetPossiblyQuotedString(dir.GetRef());
			}
			else
			{
				dir.copy(platform.GetGCodeDir());
			}

			if (sparam == 2)
			{
				fileResponse = reprap.GetFilesResponse(dir.Pointer(), true);		// Send the file list in JSON format
				fileResponse->cat('\n');
			}
			else
			{
				if (!OutputBuffer::Allocate(fileResponse))
				{
					return false;													// Cannot allocate an output buffer, try again later
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
				if (platform.GetMassStorage()->FindFirst(dir.Pointer(), fileInfo))
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
			result = platform.GetMassStorage()->Mount(card, reply, true);
		}
		break;

	case 22: // Release SD card
		if (!LockFileSystem(gb))		// don't allow more than one at a time to avoid contention on output buffers
		{
			return false;
		}
		{
			const size_t card = (gb.Seen('P')) ? gb.GetIValue() : 0;
			result = platform.GetMassStorage()->Unmount(card, reply);
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
			String<MaxFilenameLength> filename;
			if (gb.GetUnprecedentedString(filename.GetRef()))
			{
				if (QueueFileToPrint(filename.Pointer(), reply))
				{
					reprap.GetPrintMonitor().StartingPrint(filename.Pointer());
					if (platform.Emulating() == marlin && (&gb == serialGCode || &gb == telnetGCode))
					{
						reply.copy("File opened\nFile selected");
					}
					else
					{
						// Command came from web interface or PanelDue, or not emulating Marlin, so send a nicer response
						reply.printf("File %s selected for printing", filename.Pointer());
					}

					if (code == 32)
					{
						StartPrinting(true);
					}
				}
				else
				{
					result = GCodeResult::error;
				}
			}
			else
			{
				reply.copy("Filename expected");
				result = GCodeResult::error;
			}
		}
		break;

	case 24: // Print/resume-printing the selected file
		if (IsPausing() || IsResuming())
		{
			// ignore the resume request
		}
		else
		{
			if (!LockMovementAndWaitForStandstill(gb))
			{
				return false;
			}

			if (IsPaused())
			{
#if HAS_VOLTAGE_MONITOR
				if (!platform.IsPowerOk())
				{
					reply.copy("Cannot resume while power voltage is low");
					result = GCodeResult::error;
				}
				else
#endif
				{
					gb.SetState(GCodeState::resuming1);
					if (AllAxesAreHomed())
					{
						DoFileMacro(gb, RESUME_G, true);
					}
				}
			}
			else if (!fileToPrint.IsLive())
			{
				reply.copy("Cannot print, because no file is selected!");
				result = GCodeResult::error;
			}
			else
			{
#if HAS_VOLTAGE_MONITOR
				if (!platform.IsPowerOk())
				{
					reply.copy("Cannot start a print while power voltage is low");
					result = GCodeResult::error;
				}
				else
#endif
				{
					bool fromStart = (fileOffsetToPrint == 0);
					if (!fromStart)
					{
						// We executed M23 to set the file offset, which normally means that we are executing resurrect.g.
						// We need to copy the absolute/relative and volumetric extrusion flags over
						fileGCode->OriginalMachineState().drivesRelative = gb.MachineState().drivesRelative;
						fileGCode->OriginalMachineState().feedrate = gb.MachineState().feedrate;
						fileGCode->OriginalMachineState().volumetricExtrusion = gb.MachineState().volumetricExtrusion;
						fileToPrint.Seek(fileOffsetToPrint);
						moveFractionToSkip = moveFractionToStartAt;
					}
					StartPrinting(fromStart);
				}
			}
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
				DoPause(gb, PauseReason::gcode, nullptr);
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
			DoPause(gb, PauseReason::user, nullptr);
		}
		break;

	case 26: // Set SD position
		// This is used between executing M23 to set up the file to print, and M25 to print it
		if (gb.Seen('S'))
		{
			fileOffsetToPrint = (FilePosition)gb.GetUIValue();
			if (gb.Seen('P'))
			{
				moveFractionToStartAt = constrain<float>(gb.GetFValue(), 0.0, 1.0);
			}
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
			String<MaxFilenameLength> filename;
			if (gb.GetUnprecedentedString(filename.GetRef()))
			{
				const bool ok = OpenFileToWrite(gb, platform.GetGCodeDir(), filename.Pointer(), 0, false, 0);
				if (ok)
				{
					reply.printf("Writing to file: %s", filename.Pointer());
				}
				else
				{
					reply.printf("Can't open file %s for writing.", filename.Pointer());
					result = GCodeResult::error;
				}
			}
			else
			{
				reply.copy("Filename expected");
				result = GCodeResult::error;
			}
		}
		break;

	case 29: // End of file being written; should be intercepted before getting here
		reply.copy("GCode end-of-file being interpreted.");
		break;

	case 30:	// Delete file
		{
			String<MaxFilenameLength> filename;
			if (gb.GetUnprecedentedString(filename.GetRef()))
			{
				platform.GetMassStorage()->Delete(platform.GetGCodeDir(), filename.Pointer(), false);
			}
			else
			{
				reply.copy("Filename expected");
				result = GCodeResult::error;
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
			String<MaxFilenameLength> filename;
			const bool gotFilename = gb.GetUnprecedentedString(filename.GetRef());
			OutputBuffer *fileInfoResponse;
			const bool done = reprap.GetPrintMonitor().GetFileInfoResponse((gotFilename) ? filename.Pointer() : nullptr, fileInfoResponse);
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
			String<MaxFilenameLength> simFileName;

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
						StartPrinting(true);
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
			String<MaxFilenameLength> filename;
			if (gb.GetUnprecedentedString(filename.GetRef()))
			{
				if (StartHash(filename.Pointer()))
				{
					// Hashing is now in progress...
					result = GCodeResult::notFinished;
				}
				else
				{
					reply.printf("Cannot open file: %s", filename.Pointer());
					result = GCodeResult::error;
				}
			}
			else
			{
				reply.copy("Filename expected");
				result = GCodeResult::error;
			}
		}
		else
		{
			// This can take some time. All the actual heavy lifting is in dedicated methods
			result = AdvanceHash(reply);
		}
		break;

	case 39:	// Return SD card info
		{
			uint32_t slot = 0;
			bool dummy;
			gb.TryGetUIValue('P', slot, dummy);
			int32_t format = 0;
			gb.TryGetIValue('S', format, dummy);
			uint64_t capacity, freeSpace;
			uint32_t speed;
			const MassStorage::InfoResult res = platform.GetMassStorage()->GetCardInfo(slot, capacity, freeSpace, speed);
			if (format == 2)
			{
				reply.printf("{\"SDinfo\":{\"slot\":%" PRIu32 ",\"present\":", slot);
				if (res == MassStorage::InfoResult::ok)
				{
					reply.catf("1,\"capacity\":%" PRIu64 ",\"free\":%" PRIu64 ",\"speed\":%" PRIu32 "}}", capacity, freeSpace, speed);
				}
				else
				{
					reply.cat("0}}");
				}
			}
			else
			{
				switch(res)
				{
				case MassStorage::InfoResult::badSlot:
				default:
					reply.printf("Bad SD slot number: %" PRIu32, slot);
					result = GCodeResult::error;
					break;

				case MassStorage::InfoResult::noCard:
					reply.printf("No SD card mounted in slot %" PRIu32, slot);
					result = GCodeResult::error;
					break;

				case MassStorage::InfoResult::ok:
					reply.printf("SD card in slot %" PRIu32 ": capacity %.2fGb, free space %.2fGb, speed %.2fMBytes/sec",
									slot, (double)capacity/(1000*1000*1000), (double)freeSpace/(1000*1000*1000), (double)speed/(1000*1000));
					break;
				}
			}
		}
		break;

	case 42:	// Turn an output pin on or off
		if (gb.Seen('P'))
		{
			const LogicalPin logicalPin = gb.GetIValue();
			if (gb.Seen('S'))
			{
				float val = gb.GetFValue();
				if (val > 1.0)
				{
					val /= 255.0;
				}
				val = constrain<float>(val, 0.0, 1.0);

				// The SX1509B I/O expander chip doesn't seem to work if you set PWM mode and then set digital output mode.
				// This cases a problem if M42 is used to write to some pins and then M670 is used to set up the G1 P parameter port mapping.
				// The first part of the fix for this is to not select PWM mode if we don't need to.
				bool usePwm;
				uint16_t freq;
				if (gb.Seen('F'))
				{
					freq = constrain<int32_t>(gb.GetIValue(), 1, 65536);
					usePwm = true;
				}
				else
				{
					freq = DefaultPinWritePwmFreq;
					usePwm = (val != 0.0 && val != 1.0);
				}

				Pin pin;
				bool invert;
				if (platform.GetFirmwarePin(logicalPin, (usePwm) ? PinAccess::pwm : PinAccess::write, pin, invert))
				{
					if (invert)
					{
						val = 1.0 - val;
					}

					if (usePwm)
					{
						IoPort::WriteAnalog(pin, val, freq);
					}
					else
					{
						IoPort::WriteDigital(pin, val == 1.0);
					}
				}
				else
				{
					reply.printf("Logical pin %d is not available for writing", logicalPin);
					result = GCodeResult::error;
				}
			}
		}
		break;

	case 80:	// ATX power on
		platform.AtxPowerOn();
		break;

	case 81:	// ATX power off
		if (!LockMovementAndWaitForStandstill(gb))
		{
			return false;
		}
		platform.AtxPowerOff(gb.Seen('S') && gb.GetUIValue() != 0);
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
			String<MaxFilenameLength> filename;
			gb.GetPossiblyQuotedString(filename.GetRef());
			DoFileMacro(gb, filename.Pointer(), true);
		}
		break;

	case 99: // Return from Macro/Subprogram
		FileMacroCyclesReturn(gb);
		break;

	case 101: // Un-retract, generated by S3D if "Include M101/101/103" is enabled
		result = RetractFilament(gb, false);
		break;

	case 102:
		// S3D generates this command just before each explicit retraction command if both explicit retraction and "Include M101/101/103" are enabled.
		// Old versions of S3D also generate it once at the start of each prnit file if "Include M101/101/103" is enabled.
		// It's not documented, so we just ignore it rather than generate an error message.
		break;

	case 103: // Retract, generated by S3D if "Include M101/101/103" is enabled
		result = RetractFilament(gb, true);
		break;

	// For case 104, see 109

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

	case 107: // Fan off - deprecated
		lastDefaultFanSpeed = 0.0;
		SetMappedFanSpeed();
		break;

	case 108: // Cancel waiting for temperature
		if (isWaiting)
		{
			cancelWait = true;
		}
		break;

	case 104:
	case 109: // Deprecated in RRF, but widely generated by slicers
		// New behaviour from 1.20beta12:
		// M109 Snnn
		// - If no tools are active, set Tool 0 to active
		// - Set active tool's active and standby temperatures to Snnn
		//
		// M109 Tnnn Snnn
		// - If no tools are active, set Tnnn to active
		// - If another tool is active but Tnnn is off, set Tnnn to standby
		// - Set Tnnn's active and standby temperatures to Snnn
		// M104 does the same but doesn't ever select a tool
		{
			// Get the temperature to set
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

			// Find the tool that the command applies to.
			// This is the tool specified in the T parameter, else the current tool if there is one, else the default tool
			Tool *applicableTool;
			if (gb.Seen('T'))
			{
				int toolNumber = gb.GetIValue();
				toolNumber += gb.GetToolNumberAdjust();
				applicableTool = reprap.GetTool(toolNumber);
			}
			else
			{
				applicableTool = reprap.GetCurrentOrDefaultTool();
			}

			// Check that we have a tool
			if (applicableTool == nullptr)
			{
				reply.copy("Invalid tool number");
				result = GCodeResult::error;
				break;
			}

			// Set the heater temperatures for that tool. We set the standby temperatures as well as the active ones,
			// because any slicer that uses M109 doesn't understand that there are separate active and standby temperatures.
			if (simulationMode == 0)
			{
				SetToolHeaters(applicableTool, temperature, true);
			}

			Tool *currentTool = reprap.GetCurrentTool();
			if (code == 109 && currentTool == nullptr)
			{
				// Switch to the tool
				if (!LockMovementAndWaitForStandstill(gb))
				{
					return false;
				}

				gb.MachineState().newToolNumber = applicableTool->Number();
				gb.MachineState().toolChangeParam = (simulationMode == 0) ? 0 : DefaultToolChangeParam;
				gb.SetState(GCodeState::m109ToolChange0);
			}
			else
			{
				// If we already have an active tool and we are setting temperatures for a different tool, set that tool's heaters to standby in case it is off
				if (applicableTool == currentTool)
				{
					// Even though the tool is selected, we may have turned it off e.g. when upgrading the WiFi firmware or following a heater fault that has been cleared.
					// So make sure the tool heaters are on.
					reprap.SelectTool(applicableTool->Number(), simulationMode != 0);
				}
				else
				{
					reprap.StandbyTool(applicableTool->Number(), simulationMode != 0);
				}

				if (code == 109)
				{
					gb.SetState(GCodeState::m109WaitForTemperature);
				}
			}
		}
		break;

	case 110: // Set line numbers - line numbers are dealt with in the GCodeBuffer class
		break;

	case 111: // Debug level
		if (gb.Seen('S'))
		{
			const bool dbv = (gb.GetIValue() != 0);
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
			if (!cancelWait && gb.Seen('P'))
			{
				// Wait for the heaters associated with the specified tool to be ready
				int toolNumber = gb.GetIValue();
				toolNumber += gb.GetToolNumberAdjust();
				if (!ToolHeatersAtSetTemperatures(reprap.GetTool(toolNumber), true))
				{
					CheckReportDue(gb, reply);				// check whether we need to send a temperature or status report
					isWaiting = true;
					return false;
				}
				seen = true;
			}

			if (!cancelWait && gb.Seen('H'))
			{
				// Wait for specified heaters to be ready
				uint32_t heaters[Heaters];
				size_t heaterCount = Heaters;
				gb.GetUnsignedArray(heaters, heaterCount, false);

				for (size_t i = 0; i < heaterCount; i++)
				{
					if (!reprap.GetHeat().HeaterAtSetTemperature(heaters[i], true))
					{
						CheckReportDue(gb, reply);			// check whether we need to send a temperature or status report
						isWaiting = true;
						return false;
					}
				}
				seen = true;
			}

			if (!cancelWait && gb.Seen('C'))
			{
				// Wait for specified chamber(s) to be ready
				uint32_t chamberIndices[NumChamberHeaters];
				size_t chamberCount = NumChamberHeaters;
				gb.GetUnsignedArray(chamberIndices, chamberCount, false);

				if (chamberCount == 0)
				{
					// If no values are specified, wait for all chamber heaters
					for (size_t i = 0; i < NumChamberHeaters; i++)
					{
						const int8_t heater = reprap.GetHeat().GetChamberHeater(i);
						if (heater >= 0 && !reprap.GetHeat().HeaterAtSetTemperature(heater, true))
						{
							CheckReportDue(gb, reply);		// check whether we need to send a temperature or status report
							isWaiting = true;
							return false;
						}
					}
				}
				else
				{
					// Otherwise wait only for the specified chamber heaters
					for (size_t i = 0; i < chamberCount; i++)
					{
						if (chamberIndices[i] >= 0 && chamberIndices[i] < NumChamberHeaters)
						{
							const int8_t heater = reprap.GetHeat().GetChamberHeater(chamberIndices[i]);
							if (heater >= 0 && !reprap.GetHeat().HeaterAtSetTemperature(heater, true))
							{
								CheckReportDue(gb, reply);	// check whether we need to send a temperature or status report
								isWaiting = true;
								return false;
							}
						}
					}
				}
				seen = true;
			}

			// Wait for all heaters except chamber(s) to be ready
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
			String<MaxFilenameLength> msg;
			gb.GetUnprecedentedString(msg.GetRef());
			reprap.SetMessage(msg.Pointer());
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
				result = GetGCodeResultFromError(platform.DiagnosticTest(gb, reply, val));
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

	case 140: // Bed temperature
	case 141: // Chamber temperature
		{
			Heat& heat = reprap.GetHeat();
			bool seen = false;

			// Check if the heater index is passed
			int index = gb.Seen('P') ? gb.GetIValue() : 0;
			if (index < 0 || index >= (int)((code == 140) ? NumBedHeaters : NumChamberHeaters))
			{
				reply.printf("Invalid heater index '%d'", index);
				result = GCodeResult::error;
				break;
			}

			// See if the heater number is being set
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
					reply.printf("Invalid heater number '%d'", heater);
					result = GCodeResult::error;
					break;
				}

				if (code == 141)
				{
					heat.SetChamberHeater(index, heater);
				}
				else
				{
					heat.SetBedHeater(index, heater);
				}
				platform.UpdateConfiguredHeaters();
			}

			const int8_t currentHeater = (code == 141) ? heat.GetChamberHeater(index) : heat.GetBedHeater(index);
			const char* heaterName = (code == 141) ? "chamber" : "bed";

			// Active temperature
			if (gb.Seen('S'))
			{
				seen = true;
				if (currentHeater < 0)
				{
					reply.printf("No %s heater has been configured for slot %d", heaterName, index);
					result = GCodeResult::error;
				}
				else
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
			}

			if (gb.Seen('R'))
			{
				seen = true;
				if (currentHeater < 0)
				{
					reply.printf("No %s heater has been configured for slot %d", heaterName, index);
					result = GCodeResult::error;
				}
				else
				{
					heat.SetStandbyTemperature(currentHeater, gb.GetFValue());
				}
			}

			if (!seen)
			{
				if (currentHeater < 0)
				{
					reply.printf("No %s heater has been configuredt for slot %d", heaterName, index);
				}
				else
				{
					reply.printf("%c%s heater %d (slot %d) is currently at %.1f" DEGREE_SYMBOL "C",
						toupper(heaterName[0]), heaterName + 1, currentHeater, index, (double)reprap.GetHeat().GetTemperature(currentHeater));
				}
			}
		}
		break;

	case 143: // Configure heater protection
		result = GetGCodeResultFromError(SetHeaterProtection(gb, reply));
		break;

	case 144: // Set bed to standby
		{
			int index = gb.Seen('P') ? gb.GetIValue() : 0;
			if (index < 0 || index >= (int)NumBedHeaters)
			{
				reply.printf("Invalid bed heater index '%d'", index);
				result = GCodeResult::error;
				break;
			}

			const int8_t bedHeater = reprap.GetHeat().GetBedHeater(index);
			if (bedHeater >= 0)
			{
				reprap.GetHeat().Standby(bedHeater, nullptr);
			}
		}
		break;

	case 190: // Set bed temperature and wait
	case 191: // Set chamber temperature and wait
		{
			// Check if the heater index is passed
			int index = gb.Seen('P') ? gb.GetIValue() : 0;
			if (index < 0 || index >= (int)((code == 190) ? NumBedHeaters : NumChamberHeaters))
			{
				reply.printf("Invalid heater index '%d'", index);
				result = GCodeResult::error;
				break;
			}

			const int8_t heater = (code == 191) ? reprap.GetHeat().GetChamberHeater(index) : reprap.GetHeat().GetBedHeater(index);
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
					reply.catf(" %.03f", (double)(2.0/sqrtf(vef * PI)));
				}
			}
		}
		break;

	case 201: // Set/print axis accelerations
		{
			bool seen = false;
			for (size_t axis = 0; axis < numTotalAxes; axis++)
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
				for (size_t axis = 0; axis < numTotalAxes; ++axis)
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
			for (size_t axis = 0; axis < numTotalAxes; ++axis)
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
				for (size_t axis = 0; axis < numTotalAxes; ++axis)
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

	case 206: // Offset axes
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
			for (size_t axis = 0; axis < numTotalAxes; axis++)
			{
				if (gb.Seen(axisLetters[axis]))
				{
					const float value = gb.GetFValue() * distanceScale;
					if (setMin)
					{
						platform.SetAxisMinimum(axis, value, gb.MachineState().runningM501);
					}
					else
					{
						platform.SetAxisMaximum(axis, value, gb.MachineState().runningM501);
					}
					seen = true;
				}
			}

			if (!seen)
			{
				reply.copy("Axis limits ");
				char sep = '-';
				for (size_t axis = 0; axis < numTotalAxes; axis++)
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
				reply.printf("Invalid speed factor");
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

	case 260:	// I2C send
		result = SendI2c(gb, reply);
		break;

	case 261:	// I2C send
		result = ReceiveI2c(gb, reply);
		break;

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
		if (gb.Seen('S') || gb.Seen('Z'))
		{
			if (!LockMovement(gb))
			{
				return false;
			}
			const float fval = constrain<float>(gb.GetFValue(), -1.0, 1.0);
			const bool absolute = (gb.Seen('R') && gb.GetIValue() == 0);
			float difference;
			if (absolute)
			{
				difference = fval - currentBabyStepZOffset;
				currentBabyStepZOffset = fval;
			}
			else
			{
				difference = fval;
				currentBabyStepZOffset += fval;
			}
			const float amountPushed = reprap.GetMove().PushBabyStepping(difference);
			moveBuffer.initialCoords[Z_AXIS] += amountPushed;

			// The following causes all the remaining baby stepping that we didn't manage to push to be added to the [remainder of the] currently-executing move, if there is one.
			// This could result in an abrupt Z movement, however the move will be processed as normal so the jerk limit will be honoured.
			moveBuffer.coords[Z_AXIS] += difference;
		}
		else
		{
			reply.printf("Baby stepping offset is %.3fmm", (double)GetBabyStepOffset());
		}
		break;

	case 291:	// Display message, optionally wait for acknowledgement
		{
			String<MaxMessageLength> title;
			bool seen = false;
			gb.TryGetQuotedString('R', title.GetRef(), seen);

			String<MaxMessageLength> message;
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
			const unsigned int ms = (gb.Seen('P')) ? gb.GetUIValue() : 1000;			// time in milliseconds
			const unsigned int freq = (gb.Seen('S')) ? gb.GetUIValue() : 4600;			// 4600Hz produces the loudest sound on a PanelDue
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
										: reprap.GetHeat().IsBedHeater(heater) ? 75.0
										: reprap.GetHeat().IsChamberHeater(heater) ? 50.0
										: 200.0;
			const float maxPwm = (gb.Seen('P')) ? gb.GetFValue() : reprap.GetHeat().GetHeaterModel(heater).GetMaxPwm();
			if (heater < 0 || heater >= (int)Heaters)
			{
				reply.copy("Bad heater number in M303 command");
			}
			else if (!reprap.GetHeat().CheckHeater(heater))
			{
				reply.copy("Heater is not ready to perform PID auto-tuning");
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
		SetPidParameters(gb, 0, reply);
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
					maxPwm = model.GetMaxPwm(),
					voltage = model.GetVoltage();
				uint32_t freq = model.GetPwmFrequency();
				int32_t dontUsePid = model.UsePid() ? 0 : 1;
				int32_t inversionParameter = 0;

				gb.TryGetFValue('A', gain, seen);
				gb.TryGetFValue('C', tc, seen);
				gb.TryGetFValue('D', td, seen);
				gb.TryGetIValue('B', dontUsePid, seen);
				gb.TryGetFValue('S', maxPwm, seen);
				gb.TryGetFValue('V', voltage, seen);
				gb.TryGetIValue('I', inversionParameter, seen);
				gb.TryGetUIValue('F', freq, seen);

				if (seen)
				{
					const bool inverseTemperatureControl = (inversionParameter == 1 || inversionParameter == 3);
					if (!reprap.GetHeat().SetHeaterModel(heater, gain, tc, td, maxPwm, voltage,
															dontUsePid == 0, inverseTemperatureControl, (uint16_t)min<uint32_t>(freq, MaxHeaterPwmFrequency)))
					{
						reply.copy("Error: bad model parameters");
					}

					const bool invertedPwmSignal = (inversionParameter == 2 || inversionParameter == 3);
					reprap.GetHeat().SetHeaterSignalInverted(heater, invertedPwmSignal);
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
					const bool pwmSignalInverted = reprap.GetHeat().IsHeaterSignalInverted(heater);
					const char* inverted = model.IsInverted()
											? (pwmSignalInverted ? "PWM signal and temperature control" : "temperature control")
											: (pwmSignalInverted ? "PWM signal" : "no");

					reply.printf("Heater %u model: gain %.1f, time constant %.1f, dead time %.1f, max PWM %.2f, calibration voltage %.1f, mode %s, inverted %s, frequency ",
							heater, (double)model.GetGain(), (double)model.GetTimeConstant(), (double)model.GetDeadTime(), (double)model.GetMaxPwm(), (double)model.GetVoltage(), mode, inverted);
					if (model.GetPwmFrequency() == 0)
					{
						reply.cat("default");
					}
					else
					{
						reply.catf("%uHz", model.GetPwmFrequency());
					}
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
			int32_t mode = 0;						// this is usually the interpolation requested (0 = off, 1 = on)
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
					const unsigned int microsteps = gb.GetUIValue();
					if (ChangeMicrostepping(axis, microsteps, mode))
					{
						SetAxisNotHomed(axis);
					}
					else
					{
						reply.printf("Drive %c does not support %ux microstepping%s", axisLetters[axis], microsteps, ((mode) ? " with interpolation" : ""));
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
				uint32_t eVals[MaxExtruders];
				size_t eCount = numExtruders;
				gb.GetUnsignedArray(eVals, eCount, true);
				for (size_t e = 0; e < eCount; e++)
				{
					if (!ChangeMicrostepping(numTotalAxes + e, (int)eVals[e], mode))
					{
						reply.printf("Drive E%u does not support %ux microstepping%s", e, (unsigned int)eVals[e], ((mode) ? " with interpolation" : ""));
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
					const unsigned int microsteps = platform.GetMicrostepping(axis, mode, interp);
					reply.catf("%c:%u%s, ", axisLetters[axis], microsteps, (interp) ? "(on)" : "");
				}
				reply.cat("E");
				for (size_t extruder = 0; extruder < numExtruders; extruder++)
				{
					bool interp;
					const unsigned int microsteps = platform.GetMicrostepping(extruder + numTotalAxes, mode, interp);
					reply.catf(":%u%s", microsteps, (interp) ? "(on)" : "");
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
		if (platform.GetZProbeType() != ZProbeType::none)
		{
			probeIsDeployed = true;
			DoFileMacro(gb, DEPLOYPROBE_G, false);
		}
		break;

	case 402: // Retract Z probe
		if (platform.GetZProbeType() != ZProbeType::none)
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
			uint32_t lp = gb.GetUIValue();
			if (lp > 65535)
			{
				lp = NoLogicalPin;
			}
			if (platform.SetLaserPin((LogicalPin)lp))
			{
				reply.copy("Laser mode selected");
			}
			else
			{
				reply.copy("Bad P parameter");
				result = GCodeResult::error;
			}
		}
		if (result == GCodeResult::ok && gb.Seen('F'))
		{
			platform.SetLaserPwmFrequency(gb.GetFValue());
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
			uint32_t pins[2] = { NoLogicalPin, NoLogicalPin };
			size_t numPins = 2;
			gb.GetUnsignedArray(pins, numPins, false);
			if (pins[0] > 65535)
			{
				pins[0] = NoLogicalPin;
			}
			if (numPins < 2 || pins[1] < 0 || pins[1] > 65535)
			{
				pins[1] = NoLogicalPin;
			}
			if (platform.SetSpindlePins(pins[0], pins[1]))
			{
				reply.copy("CNC mode selected");
			}
			else
			{
				reply.copy("Bad P parameter");
				result = GCodeResult::error;
			}
		}
		if (result == GCodeResult::ok && gb.Seen('F'))
		{
			platform.SetSpindlePwmFrequency(gb.GetFValue());
		}
		if (result == GCodeResult::ok && gb.Seen('R'))
		{
			spindleMaxRpm = max<float>(1.0, gb.GetFValue());
		}
		break;

	case 500: // Store parameters in EEPROM
		result = GetGCodeResultFromError(WriteConfigOverrideFile(gb, reply, CONFIG_OVERRIDE_G));
		break;

	case 501: // Load parameters from EEPROM
		DoFileMacro(gb, "config-override.g", true, code);
		break;

	case 502: // Revert to default "factory settings"
		reprap.GetHeat().ResetHeaterModels();							// in case some heaters have no M307 commands in config.g
		reprap.GetMove().GetKinematics().SetCalibrationDefaults();		// in case M665/M666/M667/M669 in config.g don't define all the parameters
		platform.SetZProbeDefaults();
		DoFileMacro(gb, "config.g", true, code);
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
			FileStore * const f = platform.OpenFile(platform.GetSysDir(), platform.GetConfigFile(), OpenMode::read);
			if (f == nullptr)
			{
				reply.copy("Configuration file not found");
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
		{
			const unsigned int interface = (gb.Seen('I') ? gb.GetUIValue() : 0);
			if (gb.Seen('P'))
			{
				uint8_t mac[6];
				if (gb.GetMacAddress(mac))
				{
					reprap.GetNetwork().SetMacAddress(interface, mac);
				}
				else
				{
					reply.copy("Bad MAC address");
					result = GCodeResult::error;
				}
			}
			else
			{
				const uint8_t * const mac = reprap.GetNetwork().GetMacAddress(interface);
				reply.printf("MAC: %02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
			}
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
			const unsigned int interface = (gb.Seen('I') ? gb.GetUIValue() : 0);

			String<SsidBufferLength> ssid;
			if (reprap.GetNetwork().IsWiFiInterface(interface))
			{
				if (gb.Seen('S')) // Has the user turned the network on or off?
				{
					const int enableValue = gb.GetIValue();
					seen = true;

					if (gb.Seen('P') && !gb.GetQuotedString(ssid.GetRef()))
					{
						reply.copy("Bad or missing SSID");
						result = GCodeResult::error;
					}
					else
					{
						result = reprap.GetNetwork().EnableInterface(interface, enableValue, ssid.GetRef(), reply);
					}
				}
			}
			else
			{
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
					result = reprap.GetNetwork().EnableInterface(interface, gb.GetIValue(), ssid.GetRef(), reply);
				}
			}

			if (!seen)
			{
				result = reprap.GetNetwork().GetNetworkState(interface, reply);
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
			const float value = gb.GetFValue();
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
		result = DefineGrid(gb, reply);
		break;

	case 558: // Set or report Z probe type and for which axes it is used
		result = SetOrReportZProbe(gb, reply);
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
		String<MaxFilenameLength> filename;
		if (gb.Seen('P'))
		{
			gb.GetPossiblyQuotedString(filename.GetRef());
		}
		else
		{
			filename.copy(defaultFile);
		}
		const FilePosition size = (gb.Seen('S') ? (FilePosition)gb.GetIValue() : 0);
		const uint32_t crc32 = (gb.Seen('C') ? gb.GetUIValue() : 0);
		const bool ok = OpenFileToWrite(gb, folder, filename.Pointer(), size, true, crc32);
		if (ok)
		{
			reply.printf("Writing to file: %s", filename.Pointer());
		}
		else
		{
			reply.printf("Can't open file %s for writing.", filename.Pointer());
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
				reply.printf("Invalid heater number '%d'", heater);
				result = GCodeResult::error;
			}
		}
		else
		{
			// Clear all heater faults
			for (int heater = 0; heater < (int)Heaters; ++heater)
			{
				reprap.ClearTemperatureFault(heater);
			}
		}
		heaterFaultState = HeaterFaultState::noFault;
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
			for (size_t axis = 0; axis < numTotalAxes; axis++)
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
				for (size_t axis = 0; axis < numTotalAxes; ++axis)
				{
					reply.catf("%c: %.1f, ", axisLetters[axis], (double)(platform.GetInstantDv(axis) / (distanceScale * SecondsToMinutes)));
				}
				reply.cat("E:");
				char sep = ' ';
				for (size_t extruder = 0; extruder < numExtruders; extruder++)
				{
					reply.catf("%c%.1f", sep, (double)(platform.GetInstantDv(extruder + numTotalAxes) / (distanceScale * SecondsToMinutes)));
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
					platform.SetEnableValue(drive, (int8_t)gb.GetIValue());
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

	case 570: // Set/report heater monitoring
		{
			bool seen = false;
			if (gb.Seen('S'))
			{
				seen = true;
				heaterFaultTimeout = gb.GetUIValue() * (60 * 1000);
			}
			if (gb.Seen('H'))
			{
				const int heater = gb.GetIValue();
				if (heater >= 0 && heater < (int)Heaters)
				{
					bool seenValue = false;
					float maxTempExcursion, maxFaultTime;
					reprap.GetHeat().GetFaultDetectionParameters(heater, maxTempExcursion, maxFaultTime);
					gb.TryGetFValue('P', maxFaultTime, seenValue);
					gb.TryGetFValue('T', maxTempExcursion, seenValue);
					if (seenValue)
					{
						reprap.GetHeat().SetFaultDetectionParameters(heater, maxTempExcursion, maxFaultTime);
					}
					else if (!seen)
					{
						reply.printf("Heater %u allowed excursion %.1f" DEGREE_SYMBOL "C, fault trigger time %.1f seconds", heater, (double)maxTempExcursion, (double)maxFaultTime);
					}
				}
				seen = true;
			}
			if (!seen)
			{
				reply.printf("Print will be terminated if a heater fault is not reset within %" PRIu32 " minutes", heaterFaultTimeout/(60 * 1000));
			}
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
				uint32_t eDrive[MaxExtruders];
				size_t eCount = MaxExtruders;
				gb.GetUnsignedArray(eDrive, eCount, false);
				for (size_t i = 0; i < eCount; i++)
				{
					if (eDrive[i] >= numExtruders)
					{
						reply.printf("Invalid extruder number '%" PRIu32 "'", eDrive[i]);
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
			const uint8_t inputType = (gb.Seen('S')) ? gb.GetUIValue() : 1;
			for (size_t axis = 0; axis < numTotalAxes; ++axis)
			{
				if (gb.Seen(axisLetters[axis]))
				{
					const int ival = gb.GetIValue();
					if (ival >= 0 && ival <= 3)
					{
						platform.SetEndStopConfiguration(axis, (EndStopPosition) ival, (EndStopInputType)inputType);
						seen = true;
					}
				}
			}
			if (!seen)
			{
				reply.copy("Endstop configuration:");
				EndStopPosition config;
				EndStopInputType inputType;
				for (size_t axis = 0; axis < numTotalAxes; ++axis)
				{
					platform.GetEndStopConfiguration(axis, config, inputType);
					reply.catf(" %c: %s", axisLetters[axis],
								(config == EndStopPosition::highEndStop) ? "high end"
									: (config == EndStopPosition::lowEndStop) ? "low end"
										: "none");
					if (config == EndStopPosition::noEndStop)
					{
						reply.cat(',');
					}
					else
					{
						reply.catf(" %s,",
									(inputType == EndStopInputType::activeHigh) ? "active high switch"
										: (inputType == EndStopInputType::activeLow) ? "active low switch"
											: (inputType == EndStopInputType::zProbe) ? "Z probe"
												: (inputType == EndStopInputType::motorStall) ? "motor stall"
													: "unknown type"
									);
					}
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

	case 577: // Wait until endstop input is triggered
		if (gb.Seen('S'))
		{
			// Determine trigger type
			bool triggerCondition = (gb.GetIValue() > 0);

			// Axis endstops
			for (size_t axis=0; axis < numTotalAxes; axis++)
			{
				if (gb.Seen(axisLetters[axis]))
				{
					if (platform.EndStopInputState(axis) != triggerCondition)
					{
						result = GCodeResult::notFinished;
						break;
					}
				}
			}

			// Extruder drives
			if (gb.Seen(extrudeLetter))
			{
				size_t eDriveCount = MaxExtruders;
				uint32_t eDrives[MaxExtruders];
				gb.GetUnsignedArray(eDrives, eDriveCount, false);
				for (size_t extruder = 0; extruder < eDriveCount; extruder++)
				{
					const size_t eDrive = eDrives[extruder];
					if (eDrive >= MaxExtruders)
					{
						reply.printf("Invalid extruder drive '%u'", eDrive);
						result = GCodeResult::error;
						break;
					}

					if (platform.EndStopInputState(eDrive + E0_AXIS) != triggerCondition)
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
		result = CheckOrConfigureTrigger(gb, reply, code);
		break;

	case 584: // Set axis/extruder to stepper driver(s) mapping
		result = DoDriveMapping(gb, reply);
		break;

	case 585: // Probe Tool
		result = ProbeTool(gb, reply);
		break;

	case 586: // Configure network protocols
		{
			const unsigned int interface = (gb.Seen('I') ? gb.GetUIValue() : 0);

			if (gb.Seen('P'))
			{
				const unsigned int protocol = gb.GetUIValue();
				if (gb.Seen('S'))
				{
					const bool enable = (gb.GetIValue() == 1);
					if (enable)
					{
						const int port = (gb.Seen('R')) ? gb.GetIValue() : -1;
						const int secure = (gb.Seen('T')) ? gb.GetIValue() : -1;
						result = reprap.GetNetwork().EnableProtocol(interface, protocol, port, secure, reply);
					}
					else
					{
						result = reprap.GetNetwork().DisableProtocol(interface, protocol, reply);
					}
				}
			}
			else
			{
				// Default to reporting current protocols if P or S parameter missing
				result = reprap.GetNetwork().ReportProtocols(interface, reply);
			}
		}
		break;

#if HAS_WIFI_NETWORKING
	case 587:	// Add WiFi network or list remembered networks
	case 588:	// Forget WiFi network
	case 589:	// Configure access point
		{
			OutputBuffer *longReply = nullptr;
			result = reprap.GetNetwork().HandleWiFiCode(code, gb, reply, longReply);
			if (longReply != nullptr)
			{
				if (result == GCodeResult::ok)
				{
					UnlockAll(gb);
					HandleReply(gb, false, longReply);
					return true;
				}
				OutputBuffer::ReleaseAll(longReply);
			}
		}
		break;
#endif

	case 591: // Configure filament sensor
		if (gb.Seen('D'))
		{
			const unsigned int extruder = gb.GetUIValue();
			if (extruder < numExtruders)
			{
				bool seen = false;
				long sensorType;
				gb.TryGetIValue('P', sensorType, seen);
				if (seen)
				{
					FilamentMonitor::SetFilamentSensorType(extruder, sensorType);
				}

				FilamentMonitor *sensor = FilamentMonitor::GetFilamentSensor(extruder);
				if (sensor != nullptr)
				{
					// Configure the sensor
					const bool error = sensor->Configure(gb, reply, seen);
					result = GetGCodeResultFromError(error);
					if (error)
					{
						FilamentMonitor::SetFilamentSensorType(extruder, 0);		// delete the sensor
					}
				}
				else if (!seen)
				{
					reply.printf("Extruder drive %u has no filament sensor", extruder);
				}
			}
		}
		break;

#if SUPPORT_NONLINEAR_EXTRUSION
	case 592: // Configure nonlinear extrusion
		if (gb.Seen('D'))
		{
			const unsigned int extruder = gb.GetUIValue();
			bool seen = false;
			float a = 0.0, b = 0.0, limit = DefaultNonlinearExtrusionLimit;
			gb.TryGetFValue('A', a, seen);
			gb.TryGetFValue('B', b, seen);
			gb.TryGetFValue('L', limit, seen);
			if (seen)
			{
				platform.SetNonlinearExtrusion(extruder, a, b, limit);
			}
			else
			{
				platform.GetExtrusionCoefficients(extruder, a, b, limit);
				reply.printf("Drive %u nonlinear extrusion coefficients: A=%.3f, B=%.4f, limit=%.2f", extruder, (double)a, (double)b, (double)limit);
			}
		}
		break;
#endif

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
				if (reprap.GetMove().GetKinematics().LimitPosition(moveBuffer.coords, numVisibleAxes, axesHomed, false))
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
					reply.printf("Mode %d is not valid", mode);
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
					if (reprap.GetMove().GetKinematics().LimitPosition(moveBuffer.coords, numVisibleAxes, axesHomed, false))
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
					reply.printf("Unknown kinematics type %d", nk);
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
				if (reprap.GetMove().GetKinematics().LimitPosition(moveBuffer.coords, numVisibleAxes, axesHomed, false))
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
			reply.copy("Invalid source");
			result = GCodeResult::error;
		}
		break;

	case 752: // Start 3D scan
		if (gb.Seen('P'))
		{
			String<MaxFilenameLength> file;
			gb.GetPossiblyQuotedString(file.GetRef());
			if (gb.Seen('S'))
			{
				const int sParam = gb.GetIValue();
				if (reprap.GetScanner().IsEnabled())
				{
					if (reprap.GetScanner().IsRegistered())
					{
						result = GetGCodeResultFromFinished(reprap.GetScanner().StartScan(file.Pointer(), sParam));
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
		result = SetDateTime(gb, reply);
		break;

	case 906: // Set/report Motor currents
	case 913: // Set/report motor current percent
#if HAS_SMART_DRIVERS
	case 917: // Set/report standstill motor current percentage
#endif
		// Note that we no longer wait for movement to stop. This is so that we can use these commands (in particular, M913) in the M911 power fail script.
		{
			bool seen = false;
			for (size_t axis = 0; axis < numTotalAxes; axis++)
			{
				if (gb.Seen(axisLetters[axis]))
				{
					platform.SetMotorCurrent(axis, gb.GetFValue(), code);
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
					platform.SetMotorCurrent(numTotalAxes + e, eVals[e], code);
				}
			}

			if (code == 906 && gb.Seen('I'))
			{
				seen = true;
				const float idleFactor = gb.GetFValue();
				if (idleFactor >= 0 && idleFactor <= 100.0)
				{
					platform.SetIdleCurrentFactor(idleFactor/100.0);
				}
			}

			if (!seen)
			{
				reply.copy(	(code == 913) ? "Motor current % of normal - "
#if HAS_SMART_DRIVERS
								: (code == 917) ? "Motor standstill current % of normal - "
#endif
									: "Motor current (mA) - "
						);
				for (size_t axis = 0; axis < numTotalAxes; ++axis)
				{
					reply.catf("%c:%d, ", axisLetters[axis], (int)platform.GetMotorCurrent(axis, code));
				}
				reply.cat("E");
				for (size_t extruder = 0; extruder < numExtruders; extruder++)
				{
					reply.catf(":%d", (int)platform.GetMotorCurrent(extruder + numTotalAxes, code));
				}
				if (code == 906)
				{
					reply.catf(", idle factor %d%%", (int)(platform.GetIdleCurrentFactor() * 100.0));
				}
			}
		}
		break;

#if HAS_VOLTAGE_MONITOR
	case 911: // Enable auto save
		if (gb.Seen('S'))
		{
			const float saveVoltage = gb.GetFValue();
			if (saveVoltage < 10.0)
			{
				platform.DisableAutoSave();
			}
			else
			{
				float resumeVoltage = saveVoltage + 1.0;		// set up default resume voltage
				bool dummy;
				gb.TryGetFValue('R', resumeVoltage, dummy);

				String<80> powerFailString;
				bool seenCommandString = false;
				gb.TryGetQuotedString('P', powerFailString.GetRef(), seenCommandString);
				if (seenCommandString)
				{
					// Replace the power fail script atomically
					char *newPowerFailScript = new char[powerFailString.strlen() + 1];
					strcpy(newPowerFailScript, powerFailString.c_str());
					std::swap(newPowerFailScript, powerFailScript);
					delete[] newPowerFailScript;
				}
				else if (powerFailScript == nullptr)
				{
					reply.copy("No power fail script provided");
					result = GCodeResult::error;
					break;
				}
				platform.EnableAutoSave(saveVoltage, resumeVoltage);
			}
		}
		else
		{
			float saveVoltage, resumeVoltage;
			if (platform.GetAutoSaveSettings(saveVoltage, resumeVoltage))
			{
				reply.printf("Auto save voltage %.1fV, resume %.1fV, script \"%s\"", (double)saveVoltage, (double)resumeVoltage, (powerFailScript == nullptr) ? "" : powerFailScript);
			}
			else
			{
				reply.copy("Auto save is disabled");
			}
		}
		break;
#endif

#if HAS_CPU_TEMP_SENSOR
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
#endif

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

#if HAS_STALL_DETECT
	case 915:
		result = GetGCodeResultFromError(platform.ConfigureStallDetection(gb, reply));
		break;
#endif

#if HAS_VOLTAGE_MONITOR
	case 916:
		if (!platform.GetMassStorage()->FileExists(platform.GetSysDir(), RESUME_AFTER_POWER_FAIL_G))
		{
			reply.copy("No resume file found");
			result = GCodeResult::error;
		}
		else if (!platform.GetMassStorage()->FileExists(platform.GetSysDir(), RESUME_PROLOGUE_G))
		{
			reply.printf("Resume prologue file '%s' not found", RESUME_PROLOGUE_G);
			result = GCodeResult::error;
		}
		else
		{
			DoFileMacro(gb, RESUME_AFTER_POWER_FAIL_G, true);
		}
		break;
#endif

		// 917 set standstill current reduction not yet supported

#if SUPPORT_12864_LCD
	case 918: // Configure direct-connect display
		result = reprap.GetDisplay().Configure(gb, reply);
		break;
#endif

	case 929: // Start/stop event logging
		result = GetGCodeResultFromError(platform.ConfigureLogging(gb, reply));
		break;

	case 997: // Perform firmware update
		result = UpdateFirmware(gb, reply);
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
			bool doErase;
			if (gb.Seen('P'))
			{
				String<8> eraseString;
				gb.GetPossiblyQuotedString(eraseString.GetRef());
				doErase = StringStartsWith(eraseString.Pointer(), "ERASE");
			}
			else
			{
				doErase = false;
			}
			const uint16_t reason = (doErase)
									? (uint16_t)SoftwareResetReason::erase
									: (uint16_t)SoftwareResetReason::user;
			platform.SoftwareReset(reason);			// doesn't return
		}
		break;

	default:
		result = GCodeResult::notSupported;
		break;
	}

	return HandleResult(gb, result, reply);
}

bool GCodes::HandleTcode(GCodeBuffer& gb, const StringRef& reply)
{
	if (gb.MachineState().runningM502)
	{
		return true;			// when running M502 we don't execute T commands
	}

	bool seen = false;
	int toolNum;
	if (gb.HasCommandNumber())
	{
		seen = true;
		toolNum = gb.GetCommandNumber();
		toolNum += gb.GetToolNumberAdjust();
	}
	else if (gb.Seen('R') && gb.GetIValue() == 1)
	{
		seen = true;
		toolNum = pauseRestorePoint.toolNumber;
	}

	if (seen)
	{
		if (!LockMovementAndWaitForStandstill(gb))
		{
			return false;
		}

		const Tool * const oldTool = reprap.GetCurrentTool();
		// If old and new are the same we no longer follow the sequence. User can deselect and then reselect the tool if he wants the macros run.
		if (oldTool == nullptr || oldTool->Number() != toolNum)
		{
			gb.MachineState().newToolNumber = toolNum;
			gb.MachineState().toolChangeParam = (simulationMode != 0) ? 0
													: gb.Seen('P') ? gb.GetUIValue()
														: DefaultToolChangeParam;
			gb.SetState(GCodeState::toolChange0);
			return true;							// proceeding with state machine, so don't unlock or send a reply
		}
		else
		{
			// Even though the tool is selected, we may have turned it off e.g. when upgrading the WiFi firmware or following a heater fault that has been cleared.
			// So make sure the tool heaters are on.
			reprap.SelectTool(toolNum, simulationMode != 0);
		}
	}
	else
	{
		// Report the tool number in use if no parameter is passed
		const Tool * const tool = reprap.GetCurrentTool();
		if (tool == nullptr)
		{
			reply.copy("No tool is selected");
		}
		else
		{
			reply.printf("Tool %d is selected", tool->Number());
		}
	}

	// If we get here, we have finished
	UnlockAll(gb);
	HandleReply(gb, false, reply.Pointer());
	return true;
}

// This is called to deal with the result of processing a G- or M-code
bool GCodes::HandleResult(GCodeBuffer& gb, GCodeResult rslt, const StringRef& reply)
{
	switch (rslt)
	{
	case GCodeResult::notFinished:
		return false;

	case GCodeResult::notSupported:
		gb.PrintCommand(reply);
		reply.cat(" command is not supported");
		break;

	case GCodeResult::notSupportedInCurrentMode:
		gb.PrintCommand(reply);
		reply.catf(" command is not supported in machine mode %s", GetMachineModeString());
		break;

	case GCodeResult::badOrMissingParameter:
		gb.PrintCommand(reply);
		reply.cat(": bad or missing parameter");
		break;

	case GCodeResult::error:
		gb.PrintCommand(scratchString);
		scratchString.cat(": ");
		reply.Prepend(scratchString.Pointer());
		break;

	default:
		break;
	}

	if (gb.GetState() == GCodeState::normal)
	{
		gb.timerRunning = false;
		UnlockAll(gb);
		HandleReply(gb, rslt != GCodeResult::ok, reply.Pointer());
	}
	return true;
}

// End
