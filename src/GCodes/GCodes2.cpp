/*
 * GCodes2.cpp
 *
 *  Created on: 3 Dec 2016
 *      Author: David
 *
 *  This file contains the code to see what G, M or T command we have and start processing it.
 */

#include "GCodes.h"

#include "GCodeBuffer/GCodeBuffer.h"
#include "GCodeException.h"
#include "GCodeQueue.h"
#include "Heating/Heat.h"
#if HAS_LINUX_INTERFACE
# include "Linux/LinuxInterface.h"
#endif
#include "Movement/Move.h"
#include "Networking/Network.h"
#include "Scanner.h"
#include "PrintMonitor.h"
#include "RepRap.h"
#include "Tools/Tool.h"
#include "Endstops/ZProbe.h"
#include "FilamentMonitors/FilamentMonitor.h"
#include "General/IP4String.h"
#include "Movement/StepperDrivers/DriverMode.h"
#include <Hardware/SoftwareReset.h>
#include <Hardware/ExceptionHandlers.h>
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

#if SUPPORT_DOTSTAR_LED
# include "Fans/DotStarLed.h"
#endif

#if SUPPORT_CAN_EXPANSION
# include <CAN/CanInterface.h>
# include <CAN/ExpansionManager.h>
#endif

#ifdef DUET3_ATE
# include <Duet3Ate.h>
#endif

#if HAS_MASS_STORAGE
# include "Logger.h"
#endif

#include <utility>			// for std::swap

// If the code to act on is completed, this returns true, otherwise false.
// It is called repeatedly for a given code until it returns true for that code.
bool GCodes::ActOnCode(GCodeBuffer& gb, const StringRef& reply) noexcept
{
	try
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

			if (codeQueue->QueueCode(gb, reprap.GetMove().GetScheduledMoves() + segmentsLeft))
			{
				HandleReply(gb, GCodeResult::ok, "");
				return true;
			}

			return false;		// we should queue this code but we can't, so wait until we can either execute it or queue it
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

		case 'Q':
			return HandleQcode(gb, reply);

		default:
			break;
		}
	}
	catch (const GCodeException& e)
	{
		e.GetMessage(reply, &gb);
		HandleReply(gb, GCodeResult::error, reply.c_str());
		return true;
	}

	// If we get here then we didn't see a command that was worth parsing
	reply.printf("Bad command: ");
	gb.AppendFullCommand(reply);
	HandleReply(gb, GCodeResult::error, reply.c_str());
	return true;
}

bool GCodes::HandleGcode(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	GCodeResult result = GCodeResult::ok;
	const int code = gb.GetCommandNumber();
	if (simulationMode != 0 && code > 4 && code != 10 && code != 11 && code != 20 && code != 21 && (code < 53 || code > 59) && (code < 90 || code > 92))
	{
		HandleReply(gb, GCodeResult::ok, "");
		return true;					// we only simulate some gcodes
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
		{
			const char* err = nullptr;
			if (!DoStraightMove(gb, code == 1, err))
			{
				return false;
			}
			if (err != nullptr)
			{
				AbortPrint(gb);
				gb.SetState(GCodeState::waitingForSpecialMoveToComplete);	// force the user position to be restored
				gb.MachineState().SetError(err);	// must do this *after* calling SetState
			}
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
		{
			const char* err = nullptr;
			if (!DoArcMove(gb, code == 2, err))
			{
				return false;
			}
			if (err != nullptr)
			{
				AbortPrint(gb);
				gb.SetState(GCodeState::waitingForSpecialMoveToComplete);	// force the user position to be restored
				gb.MachineState().SetError(err);	// must do this *after* calling SetState
			}
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
				const uint32_t ival = gb.GetUIValue();
				switch (ival)
				{
				case 1:
					result = SetOrReportOffsets(gb, reply);			// same as G10 with offsets and no L parameter
					break;

				case 2:
					result = GetSetWorkplaceCoordinates(gb, reply, false);
					break;

				case 20:
					result = GetSetWorkplaceCoordinates(gb, reply, true);
					break;

				default:
					result = GCodeResult::badOrMissingParameter;
					break;
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

	case 17:	// Select XY plane for G2/G3
		break;	// we only support the XY plane, so this is a NOP

	case 18:	// Select XZ plane
	case 19:	// Select YZ plane
		result = GCodeResult::errorNotSupported;
		break;

	case 20: // Inches (which century are we living in, here?)
		gb.MachineState().usingInches = true;
		break;

	case 21: // mm
		gb.MachineState().usingInches = false;
		break;

	case 28: // Home
		result = DoHome(gb, reply);
		break;

	case 29: // Grid-based bed probing

#if HAS_LINUX_INTERFACE
		// Pass file- and system-related commands to DSF if they came from somewhere else.
		// They will be passed back to us via a binary buffer or separate SPI message if necessary.
		if (reprap.UsingLinuxInterface() && reprap.GetLinuxInterface().IsConnected() && !gb.IsBinary())
		{
			gb.SendToSbc();
			return false;
		}
#endif
		if (!LockMovementAndWaitForStandstill(gb))			// do this first to make sure that a new grid isn't being defined
		{
			return false;
		}
		{
			int sparam;
			if (gb.Seen('S'))
			{
				sparam = gb.GetIValue();
			}
			else if (DoFileMacro(gb, MESH_G, false, 29))	// no S parameter found so try to execute mesh.g
			{
				break;
			}
			else
			{
				sparam = 0;									// mesh.g not found, so treat G29 the same as G29 S0
			}

			switch(sparam)
			{
			case 0:		// probe and save height map
				result = ProbeGrid(gb, reply);
				break;

			case 1:		// load height map file
#if HAS_MASS_STORAGE
				result = LoadHeightMap(gb, reply);
#else
				result = GCodeResult::errorNotSupported;
#endif
				break;

			case 2:		// clear height map
				ClearBedMapping();
				break;

			case 3:		// save height map to names file
#if HAS_MASS_STORAGE
				result = SaveHeightMap(gb, reply);
#else
				result = GCodeResult::errorNotSupported;
#endif
				break;

			default:
				result = GCodeResult::badOrMissingParameter;
				break;
			}
		}
		break;

	case 30: // Z probe/manually set at a position and set that as point P
		if (!LockMovementAndWaitForStandstill(gb))
		{
			return false;
		}

		if (reprap.GetMove().GetKinematics().AxesToHomeBeforeProbing().Intersects(~axesVirtuallyHomed))
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
		result = platform.GetEndstops().HandleG31(gb, reply);
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

		DoFileMacro(gb, BED_EQUATION_G, true, 32);	// Try to execute bed.g
		break;

	case 38: // Straight probe - move until either the probe is triggered or the commanded move ends
		if (!LockMovementAndWaitForStandstill(gb))
		{
			return false;
		}
		result = StraightProbe(gb, reply);
		break;

	case 53:	// Temporarily use machine coordinates
		gb.MachineState().g53Active = true;
		break;

#if SUPPORT_WORKPLACE_COORDINATES
	case 54:	// Switch to coordinate system 1
	case 55:	// Switch to coordinate system 2
	case 56:	// Switch to coordinate system 3
	case 57:	// Switch to coordinate system 4
	case 58:	// Switch to coordinate system 5
	case 59:	// Switch to coordinate system 6,7,8,9
		{
			unsigned int cs = code - 54;
			if (code == 59)
			{
				const int8_t fraction = gb.GetCommandFraction();
				if (fraction > 0)
				{
					cs += (unsigned int)fraction;
				}
			}
			if (cs < NumCoordinateSystems)
			{
				currentCoordinateSystem = cs;											// this is the zero-base coordinate system number
				reprap.MoveUpdated();
				gb.MachineState().g53Active = false;									// cancel any active G53
			}
			else
			{
				result = GCodeResult::errorNotSupported;
			}
		}
		break;
#endif

	case 60: // Save position
		result = SavePosition(gb, reply);
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
		// See if there is a file in /sys named Gxx.g
		if (code >= 0 && code < 10000)
		{
			String<StringLength20> macroName;
			macroName.printf("G%d.g", code);
			if (DoFileMacro(gb, macroName.c_str(), false, 98))
			{
				break;
			}
		}
		result = GCodeResult::warningNotSupported;
	}

	return HandleResult(gb, result, reply, nullptr);
}

bool GCodes::HandleMcode(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	const int code = gb.GetCommandNumber();

	if (   simulationMode != 0
		&& (code < 20 || code > 37)
		&& code != 0 && code != 1 && code != 82 && code != 83 && code != 105 && code != 109 && code != 111 && code != 112 && code != 122
		&& code != 200 && code != 204 && code != 207 && code != 408 && code != 409 && code != 486 && code != 999)
	{
		HandleReply(gb, GCodeResult::ok, "");
		return true;			// we don't simulate most M codes
	}

#if HAS_LINUX_INTERFACE
	// Pass file- and system-related commands to DSF if they came from somewhere else. They will be passed back to us via a binary buffer or separate SPI message if necessary.
	if (   reprap.UsingLinuxInterface() && reprap.GetLinuxInterface().IsConnected() && !gb.IsBinary()
		&& (   code == 0 || code == 1
			|| code == 20 || code == 21 || code == 22 || code == 23 || code == 24 || code == 26 || code == 27 || code == 28 || code == 29
			|| code == 30 || code == 32 || code == 36 || code == 37 || code == 38 || code == 39
			|| code == 112
			|| code == 374 || code == 375
			|| code == 470 || code == 471
			|| code == 500 || code == 503 || code == 505 || code == 550
			|| code == 703
			|| code == 905 || code == 929 || code == 997 || code == 999
		   )
	   )
	{
		gb.SendToSbc();
		return false;
	}
#endif

	OutputBuffer *outBuf = nullptr;

	try
	{
#ifdef DUET3_ATE
		if (code >= 1000)
		{
			const GCodeResult rc = Duet3Ate::HandleAteMCode(code, gb, reply);
			return HandleResult(gb, rc, reply, outBuf);
		}
#endif

		GCodeResult result = GCodeResult::ok;
		switch (code)
		{
		case 0: // Stop
		case 1: // Sleep
			// Don't allow M0 or M1 to stop a print, unless the print is paused or the command comes from the file being printed itself.
			if (reprap.GetPrintMonitor().IsPrinting() && &gb != fileGCode && pauseState != PauseState::paused)
			{
				reply.copy("Pause the print before attempting to cancel it");
				result = GCodeResult::error;
			}
			else if (   !LockMovementAndWaitForStandstill(gb)	// wait until everything has stopped
					 || !IsCodeQueueIdle()						// must also wait until deferred command queue has caught up
					)
			{
				return false;
			}
			else
			{
				const auto oldPauseState = pauseState;			// pauseState gets reset by CancelPrint
				const bool wasSimulating = IsSimulating();		// simulationMode may get cleared by CancelPrint
				isWaiting = cancelWait = false;					// we may have been waiting for temperatures to be reached
				StopPrint((&gb == fileGCode) ? StopPrintReason::normalCompletion : StopPrintReason::userCancelled);

				if (!wasSimulating)								// don't run any macro files or turn heaters off etc. if we were simulating before we stopped the print
				{
					// If we are cancelling a paused print with M0 and we are homed and cancel.g exists then run it and do nothing else
					if (oldPauseState != PauseState::notPaused && code == 0 && AllAxesAreHomed() && DoFileMacro(gb, CANCEL_G, false, 0))
					{
						break;
					}

					const bool leaveHeatersOn = (gb.Seen('H') && gb.GetIValue() > 0);
					gb.SetState((leaveHeatersOn) ? GCodeState::stoppingWithHeatersOn : GCodeState::stoppingWithHeatersOff);
					(void)DoFileMacro(gb, (code == 0) ? STOP_G : SLEEP_G, false, code);
				}
			}
			break;

		case 3: // Spin spindle clockwise
			{
				const uint32_t slot = gb.Seen('P') ? gb.GetLimitedUIValue('P', MaxSpindles) : 0;
				if (gb.Seen('S'))
				{
					switch (machineType)
					{
					case MachineType::cnc:
						platform.AccessSpindle(slot).SetRpm(gb.GetIValue());
						break;

#if SUPPORT_LASER
					case MachineType::laser:
						if (segmentsLeft != 0)
						{
							return false;						// don't modify moves that haven't gone yet
						}
						moveBuffer.laserPwmOrIoBits.laserPwm = ConvertLaserPwm(gb.GetFValue());
						break;
#endif

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
				else if (machineType == MachineType::cnc && gb.Seen('R'))
				{
					const unsigned int rpNumber = gb.GetUIValue();
					if (rpNumber >= NumRestorePoints)
					{
						reply.copy("Invalid restore point number");
						result = GCodeResult::error;
					}
					else
					{
						platform.AccessSpindle(slot).SetRpm(numberedRestorePoints[rpNumber].spindleSpeeds[slot]);
					}
				}
			}
			break;

		case 4: // Spin spindle counter clockwise
			if (machineType == MachineType::cnc)
			{
				const uint32_t slot = (gb.Seen('P')) ? gb.GetLimitedUIValue('P', MaxSpindles) : 0;
				gb.MustSee('S');
				platform.AccessSpindle(slot).SetRpm(-gb.GetIValue());
			}
			else
			{
				result = GCodeResult::notSupportedInCurrentMode;
			}
			break;

		case 5: // Spindle motor off
			switch (machineType)
			{
			case MachineType::cnc:
				if (gb.Seen('P'))
				{
					// Turn off specific spindle
					const uint32_t slot = gb.GetLimitedUIValue('P', MaxSpindles);
					platform.AccessSpindle(slot).TurnOff();
				}
				else
				{
					// Turn off every spindle if no 'P' parameter is present
					for (size_t i = 0; i < MaxSpindles; i++)
					{
						platform.AccessSpindle(i).TurnOff();
					}
				}
				break;

#if SUPPORT_LASER
			case MachineType::laser:
				if (segmentsLeft != 0)
				{
					return false;						// don't modify moves that haven't gone yet
				}
				moveBuffer.laserPwmOrIoBits.Clear();
				break;
#endif

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
						platform.DisableDrivers(axis);
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
						platform.DisableDrivers(ExtruderToLogicalDrive(eDrive[i]));
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

#if HAS_MASS_STORAGE
		case 20:		// List files on SD card
			if (!LockFileSystem(gb))		// don't allow more than one at a time to avoid contention on output buffers
			{
				return false;
			}
			{
				const int sparam = (gb.Seen('S')) ? gb.GetIValue() : 0;
				const unsigned int rparam = (gb.Seen('R')) ? gb.GetUIValue() : 0;
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
					outBuf = reprap.GetFilesResponse(dir.c_str(), rparam, true);	// send the file list in JSON format
					if (outBuf == nullptr)
					{
						reply.copy("{\"err\":-1}");
					}
				}
				else if (sparam == 3)
				{
					outBuf = reprap.GetFilelistResponse(dir.c_str(), rparam);
					if (outBuf == nullptr)
					{
						reply.copy("{\"err\":-1}");
					}
				}
				else
				{
					if (!OutputBuffer::Allocate(outBuf))
					{
						return false;												// cannot allocate an output buffer, try again later
					}

					// To mimic the behaviour of the official RepRapPro firmware:
					// If we are emulating RepRap then we print "GCode files:\n" at the start, otherwise we don't.
					// If we are emulating Marlin and the code came via the serial/USB interface, then we don't put quotes around the names and we separate them with newline;
					// otherwise we put quotes around them and separate them with comma.
					if (gb.MachineState().compatibility == Compatibility::Default || gb.MachineState().compatibility == Compatibility::RepRapFirmware)
					{
						outBuf->copy("GCode files:\n");
					}

					const bool encapsulateList = gb.MachineState().compatibility != Compatibility::Marlin;
					FileInfo fileInfo;
					if (MassStorage::FindFirst(dir.c_str(), fileInfo))
					{
						// iterate through all entries and append each file name
						do {
							if (encapsulateList)
							{
								outBuf->catf("%c%s%c%c", FILE_LIST_BRACKET, fileInfo.fileName.c_str(), FILE_LIST_BRACKET, FILE_LIST_SEPARATOR);
							}
							else
							{
								outBuf->catf("%s\n", fileInfo.fileName.c_str());
							}
						} while (MassStorage::FindNext(fileInfo));

						if (encapsulateList)
						{
							// remove the last separator
							(*outBuf)[outBuf->Length() - 1] = 0;
						}
					}
					else
					{
						outBuf->cat("NONE\n");
					}
				}
			}
			break;

		case 21: // Initialise SD card
			if (!LockFileSystem(gb))		// don't allow more than one at a time to avoid contention on output buffers
			{
				return false;
			}
			{
				const size_t card = (gb.Seen('P')) ? gb.GetIValue() : 0;
				result = MassStorage::Mount(card, reply, true);
			}
			break;

		case 22: // Release SD card
			if (!LockFileSystem(gb))		// don't allow more than one at a time to avoid contention on output buffers
			{
				return false;
			}
			{
				const size_t card = (gb.Seen('P')) ? gb.GetIValue() : 0;
				result = MassStorage::Unmount(card, reply);
			}
			break;

		case 23: // Set file to print
		case 32: // Select file and start SD print
			// We now allow a file that is being printed to chain to another file. This is required for the resume-after-power-fail functionality.
			if (fileGCode->IsDoingFile() && (&gb) != fileGCode)
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
				gb.GetUnprecedentedString(filename.GetRef());
				if (QueueFileToPrint(filename.c_str(), reply))
				{
					reprap.GetPrintMonitor().StartingPrint(filename.c_str());
					if (gb.MachineState().compatibility == Compatibility::Marlin)
					{
						reply.copy("File opened\nFile selected");
					}
					else
					{
						// Command came from web interface or PanelDue, or not emulating Marlin, so send a nicer response
						reply.printf("File %s selected for printing", filename.c_str());
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
			break;
#endif

		case 24: // Print/resume-printing the selected file
			if (pauseState == PauseState::pausing || pauseState == PauseState::resuming)
			{
				// ignore the resume request
			}
			else
			{
				if (!LockMovementAndWaitForStandstill(gb))
				{
					return false;
				}

				if (pauseState == PauseState::paused)
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
						pauseState = PauseState::resuming;
						gb.SetState(GCodeState::resuming1);
						if (AllAxesAreHomed())
						{
							DoFileMacro(gb, RESUME_G, true, 24);
						}
					}
				}
#if HAS_MASS_STORAGE
				else if (!fileToPrint.IsLive())
				{
					reply.copy("Cannot print, because no file is selected!");
					result = GCodeResult::error;
				}
				else
				{
# if HAS_VOLTAGE_MONITOR
					if (!platform.IsPowerOk())
					{
						reply.copy("Cannot start a print while power voltage is low");
						result = GCodeResult::error;
					}
					else
# endif
					{
						bool fromStart = (fileOffsetToPrint == 0);
						if (!fromStart)
						{
							// We executed M26 to set the file offset, which normally means that we are executing resurrect.g.
							// We need to copy the absolute/relative and volumetric extrusion flags over
							fileGCode->OriginalMachineState().CopyStateFrom(gb.MachineState());
							fileToPrint.Seek(fileOffsetToPrint);
							moveFractionToSkip = restartMoveFractionDone;
						}
						StartPrinting(fromStart);
					}
				}
#endif
			}
			break;

		case 226: // Synchronous pause, normally initiated from within the file being printed
		case 601:
			if (pauseState == PauseState::notPaused)
			{
				if (gb.IsDoingFileMacro())
				{
					pausePending = true;
				}
				else
				{
					if (!LockMovementAndWaitForStandstill(gb))	// lock movement before calling DoPause, also wait for movement to complete
					{
						return false;
					}
					DoPause(gb, PauseReason::gcode, nullptr);
				}
			}
			break;

		case 600: // Filament change pause, synchronous
			if (pauseState == PauseState::notPaused)
			{
				if (fileGCode->IsDoingFileMacro())
				{
					filamentChangePausePending = true;
					if (&gb != fileGCode)
					{
						return false;							// wait for the current macro to finish
					}
				}
				else
				{
					if (!LockMovementAndWaitForStandstill(gb))	// lock movement before calling DoPause, also wait for movement to complete
					{
						return false;
					}
					DoPause(gb, PauseReason::filamentChange, nullptr);
				}
			}
			break;

		case 25: // Pause the print
			if (pauseState != PauseState::notPaused)
			{
				reply.copy("Printing is already paused!");
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
				if (&gb != fileGCode)
				{
					return false;						// wait for the current macro to finish
				}
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

#if HAS_MASS_STORAGE
		case 26: // Set SD position
			// This is used between executing M23 to set up the file to print, and M25 to print it
			gb.MustSee('S');
			fileOffsetToPrint = (FilePosition)gb.GetUIValue();
			restartMoveFractionDone = (gb.Seen('P')) ? constrain<float>(gb.GetFValue(), 0.0, 1.0) : 0.0;
			restartInitialUserX = (gb.Seen('X')) ? gb.GetFValue() : 0.0;
			restartInitialUserY = (gb.Seen('Y')) ? gb.GetFValue() : 0.0;
			break;

		case 27: // Report print status - Deprecated
			if (reprap.GetPrintMonitor().IsPrinting())
			{
				// Pronterface keeps sending M27 commands if "Monitor status" is checked, and it specifically expects the following response syntax
				FileData& fileBeingPrinted = fileGCode->OriginalMachineState().fileState;
				// In case there are short periods of time when PrintMonitor says a file is printing but the file is not open, or DSF passes M27 to us, check that we have a file
				if (fileBeingPrinted.IsLive())
				{
					reply.printf("SD printing byte %lu/%lu", GetFilePosition(), fileBeingPrinted.Length());
					break;
				}
			}
			reply.copy("Not SD printing.");
			break;

		case 28: // Write to file
			{
				String<MaxFilenameLength> filename;
				gb.GetUnprecedentedString(filename.GetRef());
				const bool ok = gb.OpenFileToWrite(platform.GetGCodeDir(), filename.c_str(), 0, false, 0);
				if (ok)
				{
					reply.printf("Writing to file: %s", filename.c_str());
				}
				else
				{
					reply.printf("Can't open file %s for writing.", filename.c_str());
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
				gb.GetUnprecedentedString(filename.GetRef());
				result = (platform.Delete(platform.GetGCodeDir(), filename.c_str())) ? GCodeResult::ok : GCodeResult::warning;
			}
			break;
#endif

		// For case 32, see case 23

#if HAS_MASS_STORAGE || HAS_LINUX_INTERFACE
		case 36:	// Return file information
# if HAS_LINUX_INTERFACE
			if (reprap.UsingLinuxInterface())
			{
				reprap.GetFileInfoResponse(nullptr, outBuf, true);
			}
			else
# endif
			{
# if HAS_MASS_STORAGE
				if (!LockFileSystem(gb))									// getting file info takes several calls and isn't reentrant
				{
					return false;
				}

				String<MaxFilenameLength> filename;
				gb.GetUnprecedentedString(filename.GetRef(), true);
				const bool done = reprap.GetFileInfoResponse((filename.IsEmpty()) ? nullptr : filename.c_str(), outBuf, false);
				if (outBuf != nullptr)
				{
					outBuf->cat('\n');
				}
				result = (done) ? GCodeResult::ok : GCodeResult::notFinished;
# endif
			}
			break;

		case 37:	// Simulation mode on/off, or simulate a whole file
# if HAS_LINUX_INTERFACE
			if (reprap.UsingLinuxInterface() && !gb.IsBinary())
			{
				reply.copy("M37 can be only started from the Linux interface");
				result = GCodeResult::error;
			}
			else
# endif
			{
				bool seen = false;
				String<MaxFilenameLength> simFileName;

				gb.TryGetPossiblyQuotedString('P', simFileName.GetRef(), seen);
				if (seen)
				{
					const bool updateFile = !gb.Seen('F') || gb.GetUIValue() == 1;
					result = SimulateFile(gb, reply, simFileName.GetRef(), updateFile);
				}
				else
				{
					uint32_t newSimulationMode;
					gb.TryGetUIValue('S', newSimulationMode, seen);
					if (seen)
					{
						result = ChangeSimulationMode(gb, reply, newSimulationMode);
					}
					else
					{
						reply.printf("Simulation mode: %s, move time: %.1f sec, other time: %.1f sec",
								(simulationMode != 0) ? "on" : "off", (double)reprap.GetMove().GetSimulationTime(), (double)simulationTime);
					}
				}
			}
			break;
#endif

#if HAS_MASS_STORAGE
		case 38: // Report SHA1 of file
			if (!LockFileSystem(gb))								// getting file hash takes several calls and isn't reentrant
			{
				return false;
			}
			if (fileBeingHashed == nullptr)
			{
				// See if we can open the file and start hashing
				String<MaxFilenameLength> filename;
				gb.GetUnprecedentedString(filename.GetRef());
				if (StartHash(filename.c_str()))
				{
					// Hashing is now in progress...
					result = GCodeResult::notFinished;
				}
				else
				{
					reply.printf("Cannot open file: %s", filename.c_str());
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
				uint32_t clSize;
				const MassStorage::InfoResult res = MassStorage::GetCardInfo(slot, capacity, freeSpace, speed, clSize);
				if (format == 2)
				{
					reply.printf("{\"SDinfo\":{\"slot\":%" PRIu32 ",\"present\":", slot);
					if (res == MassStorage::InfoResult::ok)
					{
						reply.catf("1,\"capacity\":%" PRIu64 ",\"free\":%" PRIu64 ",\"speed\":%" PRIu32 ",\"clsize\":%" PRIu32 "}}", capacity, freeSpace, speed, clSize);
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
						reply.printf("SD card in slot %" PRIu32 ": capacity %.2fGb, free space %.2fGb, speed %.2fMBytes/sec, cluster size ",
										slot, (double)capacity/(1000*1000*1000), (double)freeSpace/(1000*1000*1000), (double)speed/(1000*1000));
						if (clSize < 1024)
						{
							reply.catf("%" PRIu32 " bytes", clSize);
						}
						else
						{
							reply.catf("%" PRIu32 "kb", clSize/1024);
						}
						break;
					}
				}
			}
			break;
#endif

		case 42:	// Turn an output pin on or off
			{
				const uint32_t gpioPortNumber = gb.GetLimitedUIValue('P', MaxGpOutPorts);
				gb.MustSee('S');
				result = platform.GetGpOutPort(gpioPortNumber).WriteAnalog(gpioPortNumber, false, gb.GetPwmValue(), gb, reply);
			}
			break;

		case 80:	// ATX power on
			atxPowerControlled = true;
			platform.AtxPowerOn();
			break;

		case 81:	// ATX power off
			atxPowerControlled = true;
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
				bool seenUstepMultiplier = false;
				uint32_t ustepMultiplier = 0;
				gb.TryGetUIValue('S', ustepMultiplier, seenUstepMultiplier);

				bool seen = false;
#if SUPPORT_CAN_EXPANSION
				AxesBitmap axesToUpdate;
#endif
				for (size_t axis = 0; axis < numTotalAxes; axis++)
				{
					if (gb.Seen(axisLetters[axis]))
					{
						if (!LockMovementAndWaitForStandstill(gb))
						{
							return false;
						}
						platform.SetDriveStepsPerUnit(axis, gb.GetFValue(), ustepMultiplier);
#if SUPPORT_CAN_EXPANSION
						axesToUpdate.SetBit(axis);
#endif
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
						const size_t drive = ExtruderToLogicalDrive(e);
#if SUPPORT_CAN_EXPANSION
						axesToUpdate.SetBit(drive);
#endif
						platform.SetDriveStepsPerUnit(drive, eVals[e], ustepMultiplier);
					}
				}

				if (seen)
				{
					// On a delta, if we change the drive steps/mm then we need to recalculate the motor positions
					reprap.GetMove().SetNewPosition(moveBuffer.coords, true);
#if SUPPORT_CAN_EXPANSION
					result = platform.UpdateRemoteStepsPerMmAndMicrostepping(axesToUpdate, reply);
#endif
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
						reply.catf("%c%.3f", sep, (double)platform.DriveStepsPerUnit(ExtruderToLogicalDrive(extruder)));
						sep = ':';
					}
				}
			}
			break;

		case 98: // Call Macro/Subprogram
			{
				gb.MustSee('P');
				String<MaxFilenameLength> filename;
				gb.GetPossiblyQuotedString(filename.GetRef());
				DoFileMacro(gb, filename.c_str(), true, code);
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
			// Old versions of S3D also generate it once at the start of each print file if "Include M101/101/103" is enabled.
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
				uint32_t fanNum;
				gb.TryGetUIValue('P', fanNum, seenFanNum);
				bool processed;

				// 2018-08-09: only configure the fan if a fan number was given.
				// This avoids M106 Snn failing if we have disabled Fan 0 and mapped the print cooling fan to a different fan.
				if (seenFanNum)
				{
					bool error = false;
					processed = reprap.GetFansManager().ConfigureFan(code, fanNum, gb, reply, error);
					result = GetGCodeResultFromError(error);
				}
				else
				{
					processed = false;
				}

				// ConfigureFan only processes S parameters if there were other parameters to process
				if (!processed && gb.Seen('S'))
				{
					// Convert the parameter to an interval in 0.0..1.0 here so that we save the correct value in lastDefaultFanSpeed
					const float f = gb.GetPwmValue();
					if (seenFanNum)
					{
						result = reprap.GetFansManager().SetFanValue(fanNum, f, reply);
						if (IsMappedFan(fanNum))
						{
							lastDefaultFanSpeed = f;
						}
					}
					else
					{
						// We are processing an M106 S### command with no other recognised parameters and we have a tool selected.
						// Apply the fan speed setting to the fans in the fan mapping for the current tool.
						SetMappedFanSpeed(f);
					}
				}

				// ConfigureFan doesn't process R parameters
				if (gb.Seen('R'))
				{
					// Restore fan speed to value when print was paused
					if (seenFanNum)
					{
						result = reprap.GetFansManager().SetFanValue(fanNum, pausedFanSpeeds[fanNum], reply);
					}
					else
					{
						SetMappedFanSpeed(pausedDefaultFanSpeed);
					}
				}
			}
			break;

		case 107: // Fan off - deprecated
			SetMappedFanSpeed(0.0);
			break;

		case 108: // Cancel waiting for temperature
			if (isWaiting)
			{
				cancelWait = true;
			}
			break;

		case 109: // Deprecated in RRF, but widely generated by slicers
			if (   !LockMovementAndWaitForStandstill(gb)		// wait until movement has finished
				|| !IsCodeQueueIdle()							// also wait until deferred command queue has caught up to avoid out-of-order execution
			   )
			{
				return false;
			}
			UnlockMovement(gb);									// allow babystepping and pausing while heating

			// no break
		case 104:
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
				int32_t toolNumber = 0;
				bool seenT = false;
				gb.TryGetIValue('T', toolNumber, seenT);
				ReadLockedPointer<Tool> const applicableTool = (seenT) ? reprap.GetTool(toolNumber) : reprap.GetCurrentOrDefaultTool();

				// Check that we have a tool
				if (applicableTool.IsNull())
				{
					reply.copy("Invalid tool number");
					result = GCodeResult::error;
					break;
				}

				// Set the heater temperatures for that tool. We set the standby temperatures as well as the active ones,
				// because any slicer that uses M109 doesn't understand that there are separate active and standby temperatures.
				if (simulationMode == 0)
				{
					SetToolHeaters(applicableTool.Ptr(), temperature, true);	// this may throw
				}

				Tool * const currentTool = reprap.GetCurrentTool();
				if (code == 109 && currentTool == nullptr)
				{
					// Switch to the tool
					if (!LockMovementAndWaitForStandstill(gb))
					{
						return false;
					}

					newToolNumber = applicableTool->Number();
					toolChangeParam = (simulationMode != 0) ? 0 : DefaultToolChangeParam;
					gb.SetState(GCodeState::m109ToolChange0);
					result = GCodeResult::ok;
				}
				else
				{
					if (applicableTool.Ptr() == currentTool)
					{
						// Even though the tool is selected, we may have turned it off e.g. when upgrading the WiFi firmware or following a heater fault that has been cleared.
						// So make sure the tool heaters are on.
						reprap.SelectTool(applicableTool->Number(), simulationMode != 0);
					}
					else
					{
						// If we already have an active tool and we are setting temperatures for a different tool, set that tool's heaters to standby in case it is off
						reprap.StandbyTool(applicableTool->Number(), simulationMode != 0);
					}

					if (code == 109 && simulationMode == 0)
					{
						gb.SetState(GCodeState::m109WaitForTemperature);
						result = GCodeResult::ok;
					}
				}
			}
			break;

		case 110: // Set line numbers
			//TODO
			break;

		case 111: // Debug level
			if (gb.Seen('S'))
			{
				const bool dbv = (gb.GetIValue() != 0);
				if (gb.Seen('P'))
				{
					reprap.SetDebug(static_cast<Module>(gb.GetIValue()), dbv);
					reprap.PrintDebug(gb.GetResponseMessageType());
					return true;
				}
				if (dbv)
				{
					// Repetier Host sends M111 with various S parameters to enable echo and similar features, which used to turn on all out debugging.
					// But it's not useful to enable all debugging anyway. So we no longer allow debugging to be enabled without a P parameter.
					reply.copy("Use P parameter to specify which module to debug");
				}
				else
				{
					// M111 S0 still clears all debugging
					reprap.ClearDebug();
				}
			}
			else
			{
				reprap.PrintDebug(gb.GetResponseMessageType());
				return true;
			}
			break;

		case 112: // Emergency stop - acted upon in Webserver, but also here in case it comes from USB etc.
			DoEmergencyStop();
			break;

		case 114:
			GetCurrentCoordinates(reply);
			break;

		case 115: // Print firmware version or set hardware type
#if defined(DUET_NG) || defined(DUET_06_85)
			if (gb.Seen('P'))
			{
				if (runningConfigFile)
				{
					platform.SetBoardType((BoardType)gb.GetIValue());
				}
				else
				{
					reply.copy("Board type can only be set within config.g");
					result = GCodeResult::error;
				}
			}
			else
#endif
			{
#if SUPPORT_CAN_EXPANSION
				if (gb.Seen('B'))
				{
					const uint32_t board = gb.GetUIValue();
					if (board != CanId::MasterAddress)
					{
						result = CanInterface::GetRemoteFirmwareDetails(board, gb, reply);
						break;
					}
				}
#endif
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
			if (   !LockMovementAndWaitForStandstill(gb)		// wait until movement has finished
				|| !IsCodeQueueIdle()							// also wait until deferred command queue has caught up to avoid out-of-order execution
			   )
			{
				return false;
			}

			if (!cancelWait)
			{
				const float tolerance = (gb.Seen('S')) ? max<float>(gb.GetFValue(), 0.1) : TEMPERATURE_CLOSE_ENOUGH;
				bool seen = false;
				if (gb.Seen('P'))
				{
					// Wait for the heaters associated with the specified tool to be ready
					if (!ToolHeatersAtSetTemperatures(reprap.GetTool(gb.GetIValue()).Ptr(), true, tolerance))
					{
						isWaiting = true;
						return false;
					}
					seen = true;
				}

				if (gb.Seen('H'))
				{
					// Wait for specified heaters to be ready
					uint32_t heaters[MaxHeaters];
					size_t heaterCount = MaxHeaters;
					gb.GetUnsignedArray(heaters, heaterCount, false);

					for (size_t i = 0; i < heaterCount; i++)
					{
						if (!reprap.GetHeat().HeaterAtSetTemperature(heaters[i], true, tolerance))
						{
							isWaiting = true;
							return false;
						}
					}
					seen = true;
				}

				if (gb.Seen('C'))
				{
					// Wait for specified chamber(s) to be ready
					uint32_t chamberIndices[MaxChamberHeaters];
					size_t chamberCount = MaxChamberHeaters;
					gb.GetUnsignedArray(chamberIndices, chamberCount, false);

					if (chamberCount == 0)
					{
						// If no values are specified, wait for all chamber heaters
						for (size_t i = 0; i < MaxChamberHeaters; i++)
						{
							const int8_t heater = reprap.GetHeat().GetChamberHeater(i);
							if (heater >= 0 && !reprap.GetHeat().HeaterAtSetTemperature(heater, true, tolerance))
							{
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
							if (chamberIndices[i] >= 0 && chamberIndices[i] < MaxChamberHeaters)
							{
								const int8_t heater = reprap.GetHeat().GetChamberHeater(chamberIndices[i]);
								if (heater >= 0 && !reprap.GetHeat().HeaterAtSetTemperature(heater, true, tolerance))
								{
									isWaiting = true;
									return false;
								}
							}
						}
					}
					seen = true;
				}

				// Wait for all heaters except chamber(s) to be ready
				if (!seen && !reprap.GetHeat().AllHeatersAtSetTemperatures(true, tolerance))
				{
					isWaiting = true;
					return false;
				}
			}

			// If we get here, there is nothing more to wait for
			cancelWait = isWaiting = false;
			break;

		case 117:	// Display message
			{
				String<MediumStringLength> msg;
				gb.GetUnprecedentedString(msg.GetRef(), true);
				reprap.SetMessage(msg.c_str());
			}
			break;

		case 118:	// Echo message on host
			{
				gb.MustSee('S');
				String<GCODE_LENGTH> message;
				gb.GetQuotedString(message.GetRef());

				MessageType type = GenericMessage;
				bool seenP = false;
				if (gb.Seen('P'))
				{
					seenP = true;
					const int32_t param = gb.GetIValue();
					switch (param)
					{
					case 0:		// Generic (default)
						// no need to set it twice
						break;
					case 1:		// USB
						type = UsbMessage;
						break;
					case 2:		// UART port
						type = DirectAuxMessage;
						break;
					case 3:		// HTTP
						type = HttpMessage;
						break;
					case 4:		// Telnet
						type = TelnetMessage;
						break;
#ifdef SERIAL_AUX2_DEVICE
					case 5:		// AUX2
						type = Aux2Message;
						break;
#endif
					default:
						reply.printf("Invalid message type: %" PRIi32, param);
						result = GCodeResult::error;
						break;
					}
				}

#if HAS_MASS_STORAGE
				if (gb.Seen('L'))
				{
					// If we haven't seen a P parameter but seen the L parameter we are going to log
					// only to log file so reset message type first
					if (!seenP)
					{
						type = MessageType::NoDestinationMessage;
					}
					const LogLevel logLevel = (LogLevel) gb.GetLimitedUIValue('L', LogLevel::NumValues, LogLevel::off);
					switch (logLevel.ToBaseType())
					{
					case LogLevel::off:
						type = RemoveLogging(type);
						break;
					case LogLevel::warn:
						type = AddLogWarn(type);
						break;
					case LogLevel::info:
						type = AddLogInfo(type);
						break;
					case LogLevel::debug:
						type = AddLogDebug(type);
						break;
					}
				}
#endif

				if (result != GCodeResult::error)
				{
					// Append newline and send the message to the destinations
					message.cat('\n');
					platform.Message(type, message.c_str());
				}
			}
			break;

		case 119:
			platform.GetEndstops().GetM119report(reply);
			break;

		case 120:
			Push(gb, true);
			break;

		case 121:
			Pop(gb, true);
			break;

		case 122:
			{
				const unsigned int type = (gb.Seen('P')) ? gb.GetIValue() : 0;
				const MessageType mt = (MessageType)(gb.GetResponseMessageType() | PushFlag);	// set the Push flag to combine multiple messages into a single OutputBuffer chain
#if SUPPORT_CAN_EXPANSION
				const uint32_t board = (gb.Seen('B')) ? gb.GetUIValue() : 0;
				if (board != CanId::MasterAddress)
				{
					result = CanInterface::RemoteDiagnostics(mt, board, type, gb, reply);
					break;
				}
#endif
				if (type == 0)
				{
					reprap.Diagnostics(mt);
				}
				else
				{
					result = platform.DiagnosticTest(gb, reply, outBuf, type);
				}
			}
			break;

		// M135 (set PID sample interval) is no longer supported

		case 140: // Bed temperature
		case 141: // Chamber temperature
			{
				Heat& heat = reprap.GetHeat();
				bool seen = false;

				// Check if the heater index is passed
				const unsigned int index = gb.Seen('P') ? gb.GetLimitedUIValue('P', (code == 140) ? MaxBedHeaters : MaxChamberHeaters) : 0;

				// See if the heater number is being set
				if (gb.Seen('H'))
				{
					seen = true;
					int heater = gb.GetIValue();
					if (heater < 0)
					{
						heater = -1;
					}
					else if (heater >= (int)MaxHeaters)
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
				const char* const heaterName = (code == 141) ? "chamber" : "bed";

				// Active temperature
				if (gb.Seen('S'))
				{
					seen = true;
					const float temperature = gb.GetFValue();
					if (currentHeater < 0)
					{
						if (temperature > 0.0)		// turning off a non-existent bed or chamber heater is not an error
						{
							reply.printf("No %s heater has been configured for slot %d", heaterName, index);
							result = GCodeResult::error;
						}
					}
					else
					{
						if (temperature < NEARLY_ABS_ZERO)
						{
							heat.SwitchOff(currentHeater);
						}
						else
						{
							heat.SetActiveTemperature(currentHeater, temperature);		// may throw
							result = heat.Activate(currentHeater, reply);
						}
					}
				}

				// Standby temperature
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
						reply.printf("No %s heater has been configured for slot %d", heaterName, index);
					}
					else
					{
						reply.printf("%c%s heater %d (slot %d) is currently at %.1f" DEGREE_SYMBOL "C",
							toupper(heaterName[0]), heaterName + 1, currentHeater, index, (double)reprap.GetHeat().GetHeaterTemperature(currentHeater));
					}
				}
			}
			break;

		case 143: // Configure heater protection
			result = reprap.GetHeat().HandleM143(gb, reply);
			break;

		case 144: // Set bed to standby, or to active if S1 parameter given
			{
				const unsigned int index = gb.Seen('P') ? gb.GetLimitedUIValue('P', MaxBedHeaters) : 0;
				const int8_t bedHeater = reprap.GetHeat().GetBedHeater(index);
				if (bedHeater >= 0)
				{
					if (gb.Seen('S') && gb.GetIValue() == 1)
					{
						result = reprap.GetHeat().Activate(bedHeater, reply);
					}
					else
					{
						reprap.GetHeat().Standby(bedHeater, nullptr);
					}
				}
			}
			break;

#if SUPPORT_DOTSTAR_LED
		case 150:
			result = DotStarLed::SetColours(gb, reply);
			break;
#endif

		case 190: // Set bed temperature and wait
		case 191: // Set chamber temperature and wait
			if (   !LockMovementAndWaitForStandstill(gb)		// wait until movement has finished
				|| !IsCodeQueueIdle()							// also wait until deferred command queue has caught up to avoid out-of-order execution
			   )
			{
				return false;
			}

			UnlockMovement(gb);									// allow babystepping and pausing while heating
			{
				// Check if the heater index is passed
				const uint32_t index = gb.Seen('P') ? gb.GetLimitedUIValue('P', (code == 190) ? MaxBedHeaters : MaxChamberHeaters) : 0;
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

					reprap.GetHeat().SetActiveTemperature(heater, temperature);		// may throw
					result = reprap.GetHeat().Activate(heater, reply);
					if (cancelWait || reprap.GetHeat().HeaterAtSetTemperature(heater, waitWhenCooling, TEMPERATURE_CLOSE_ENOUGH))
					{
						cancelWait = isWaiting = false;
						break;
					}

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
					volumetricExtrusionFactors[i] = (d <= 0.0) ? 1.0 : 4.0/(fsquare(d) * Pi);
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
						reply.catf(" %.03f", (double)(2.0/sqrtf(vef * Pi)));
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
						platform.SetAcceleration(axis, gb.GetDistance());
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
						platform.SetAcceleration(ExtruderToLogicalDrive(e), gb.ConvertDistance(eVals[e]));
					}
				}

				if (seen)
				{
					reprap.MoveUpdated();
				}
				else
				{
					reply.printf("Accelerations (mm/sec^2): ");
					for (size_t axis = 0; axis < numTotalAxes; ++axis)
					{
						reply.catf("%c: %.1f, ", axisLetters[axis], (double)platform.Acceleration(axis));
					}
					reply.cat("E:");
					char sep = ' ';
					for (size_t extruder = 0; extruder < numExtruders; extruder++)
					{
						reply.catf("%c%.1f", sep, (double)platform.Acceleration(ExtruderToLogicalDrive(extruder)));
						sep = ':';
					}
				}
			}
			break;

		case 203: // Set/print minimum/maximum feedrates
			{
				// Units are mm/sec if S1 is given, else mm/min
				const bool usingMmPerSec = (gb.Seen('S') && gb.GetIValue() == 1);
				const float settingMultiplier = (usingMmPerSec) ? 1.0 : SecondsToMinutes;
				bool seen = false;

				// Do the minimum first, because we constrain the maximum rates to be no lower than it
				if (gb.Seen('I'))
				{
					seen = true;
					platform.SetMinMovementSpeed(gb.GetDistance() * settingMultiplier);
				}

				for (size_t axis = 0; axis < numTotalAxes; ++axis)
				{
					if (gb.Seen(axisLetters[axis]))
					{
						seen = true;
						platform.SetMaxFeedrate(axis, gb.GetDistance() * settingMultiplier);
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
						platform.SetMaxFeedrate(ExtruderToLogicalDrive(e), gb.ConvertDistance(eVals[e]) * settingMultiplier);
					}
				}

				if (seen)
				{
					reprap.MoveUpdated();
				}
				else
				{
					const float reportingMultiplier = (usingMmPerSec) ? 1.0 : MinutesToSeconds;
					reply.printf("Max speeds (mm/%s): ", (usingMmPerSec) ? "sec" : "min");
					for (size_t axis = 0; axis < numTotalAxes; ++axis)
					{
						reply.catf("%c: %.1f, ", axisLetters[axis], (double)(platform.MaxFeedrate(axis) * reportingMultiplier));
					}
					reply.cat("E:");
					char sep = ' ';
					for (size_t extruder = 0; extruder < numExtruders; extruder++)
					{
						reply.catf("%c%.1f", sep, (double)(platform.MaxFeedrate(ExtruderToLogicalDrive(extruder)) * reportingMultiplier));
						sep = ':';
					}
					reply.catf(", min. speed %.2f", (double)(platform.MinMovementSpeed() * reportingMultiplier));
				}
			}
			break;

		case 204: // Set max travel and printing accelerations
			result = reprap.GetMove().ConfigureAccelerations(gb, reply);
			break;

		// For case 205 see case 566

		case 206: // Offset axes
			result = OffsetAxes(gb, reply);
			break;

		case 207: // Set firmware retraction details
			if (gb.Seen('P'))
			{
				const unsigned int toolNumber = gb.GetUIValue();
				auto tool = reprap.GetTool(toolNumber);
				if (tool.IsNull())
				{
					reply.printf("Tool %u does not exist", toolNumber);
					result = GCodeResult::error;
				}
				else
				{
					result = tool->SetFirmwareRetraction(gb, reply, outBuf);
				}
			}
			else
			{
				result = reprap.SetAllToolsFirmwareRetraction(gb, reply, outBuf);
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
						seen = true;
						float values[2];
						size_t numValues = 2;
						gb.GetFloatArray(values, numValues, false);
						bool ok;
						if (numValues == 2)
						{
							ok = values[1] > values[0];
							if (ok)
							{
								platform.SetAxisMinimum(axis, values[0], gb.MachineState().runningM501);
								platform.SetAxisMaximum(axis, values[1], gb.MachineState().runningM501);
							}
						}
						else if (setMin)
						{
							ok = platform.AxisMaximum(axis) > values[0];
							if (ok)
							{
								platform.SetAxisMinimum(axis, values[0], gb.MachineState().runningM501);
							}
						}
						else
						{
							ok = values[0] > platform.AxisMinimum(axis);
							if (ok)
							{
								platform.SetAxisMaximum(axis, values[0], gb.MachineState().runningM501);
							}
						}

						if (!ok)
						{
							reply.printf("%c axis maximum must be greater than minimum", axisLetters[axis]);
							result = GCodeResult::error;
						}
					}
				}

				if (!seen)
				{
					reply.copy("Axis limit");
					char sep = 's';
					for (size_t axis = 0; axis < numTotalAxes; axis++)
					{
						reply.catf("%c %c%.1f:%.1f", sep, axisLetters[axis], (double)platform.AxisMinimum(axis), (double)platform.AxisMaximum(axis));
						sep = ',';
					}
				}
			}
			break;

		case 220:	// Set/report speed factor override percentage
			if (gb.Seen('S'))
			{
				const float newSpeedFactor = gb.GetFValue() * 0.01;
				if (newSpeedFactor >= 0.01)
				{
					// If the last move hasn't gone yet, update its feed rate if it is not a firmware retraction
					if (segmentsLeft != 0 && moveBuffer.applyM220M221)
					{
						moveBuffer.feedRate *= newSpeedFactor / speedFactor;
					}
					speedFactor = newSpeedFactor;
					reprap.MoveUpdated();
				}
				else
				{
					reply.copy("Invalid speed factor");
					result = GCodeResult::error;
				}
			}
			else
			{
				reply.printf("Speed factor: %.1f%%", (double)(speedFactor * 100.0));
			}
			break;

		case 221:	// Set/report extrusion factor override percentage
			{
				uint32_t extruder = 0;
				const bool seenD = gb.Seen('D');
				if (seenD)
				{
					extruder = gb.GetLimitedUIValue('D', numExtruders);
				}

				const Tool * const ct = reprap.GetCurrentTool();
				if (!seenD && ct == nullptr)
				{
					reply.copy("No tool selected");
					result = GCodeResult::error;
				}
				else if (gb.Seen('S'))	// S parameter sets the override percentage
				{
					const float extrusionFactor = gb.GetFValue() * 0.01;
					if (extrusionFactor >= 0.01)
					{
						if (seenD)
						{
							ChangeExtrusionFactor(extruder, extrusionFactor);
						}
						else
						{
							ct->IterateExtruders([this, extrusionFactor](unsigned int extruder) { ChangeExtrusionFactor(extruder, extrusionFactor); });
						}
					}
				}
				else if (seenD)
				{
					reply.printf("Extrusion factor for extruder %" PRIu32 ": %.1f%%", extruder, (double)(extrusionFactors[extruder] * 100.0));
				}
				else
				{
					reply.copy("Extrusion factor(s) for current tool:");
					ct->IterateExtruders([reply, this](unsigned int extruder) { reply.catf(" %.1f%%", (double)(extrusionFactors[extruder] * 100.0)); });
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
			{
				const uint32_t gpioPortNumber = gb.GetLimitedUIValue('P', MaxGpOutPorts);
				gb.MustSee('S');
				float angleOrWidth = gb.GetFValue();
				if (angleOrWidth < 0.0)
				{
					// Disable the servo by setting the pulse width to zero
					angleOrWidth = 0.0;
				}
				else if (angleOrWidth < MinServoPulseWidth)
				{
					// User gave an angle so convert it to a pulse width in microseconds
					angleOrWidth = (min<float>(angleOrWidth, 180.0) * ((MaxServoPulseWidth - MinServoPulseWidth) / 180.0)) + MinServoPulseWidth;
				}
				else if (angleOrWidth > MaxServoPulseWidth)
				{
					angleOrWidth = MaxServoPulseWidth;
				}

				const float pwm = angleOrWidth * (ServoRefreshFrequency/1e6);
				result = platform.GetGpOutPort(gpioPortNumber).WriteAnalog(gpioPortNumber, true, pwm, gb, reply);
			}
			break;

		case 290:	// Baby stepping
			{
				const bool absolute = (gb.Seen('R') && gb.GetIValue() == 0);
				bool seen = false;
				float differences[MaxAxes];
				for (size_t axis = 0; axis < numVisibleAxes; ++axis)
				{
					if (gb.Seen(axisLetters[axis]) || (axis == 2 && gb.Seen('S')))			// S is a synonym for Z
					{
						seen = true;
						const float fval = gb.GetFValue();
						if (absolute)
						{
							differences[axis] = fval - GetTotalBabyStepOffset(axis);
						}
						else
						{
							differences[axis] = constrain<float>(fval, -1.0, 1.0);
						}
					}
					else
					{
						differences[axis] = 0.0;
					}
				}

				if (seen)
				{
					if (!LockMovement(gb))
					{
						return false;
					}

					// Perform babystepping synchronously with moves
					bool haveResidual = false;
					for (size_t axis = 0; axis < numVisibleAxes; ++axis)
					{
						currentBabyStepOffsets[axis] += differences[axis];
						reprap.MoveUpdated();
						const float amountPushed = reprap.GetMove().PushBabyStepping(axis, differences[axis]);
						moveBuffer.initialCoords[axis] += amountPushed;

						// The following causes all the remaining baby stepping that we didn't manage to push to be added to the [remainder of the] currently-executing move, if there is one.
						// This could result in an abrupt Z movement, however the move will be processed as normal so the jerk limit will be honoured.
						moveBuffer.coords[axis] += differences[axis];
						if (amountPushed != differences[axis])
						{
							haveResidual = true;
						}
					}

					if (haveResidual && segmentsLeft == 0 && reprap.GetMove().NoLiveMovement())
					{
						// The pipeline is empty, so execute the babystepping move immediately
						SetMoveBufferDefaults();
						moveBuffer.feedRate = DefaultFeedRate;
						moveBuffer.tool = reprap.GetCurrentTool();
						NewMoveAvailable(1);
					}
				}
				else
				{
					reply.printf("Baby stepping offsets (mm):");
					for (size_t axis = 0; axis < numVisibleAxes; ++axis)
					{
						reply.catf(" %c:%.3f", axisLetters[axis], (double)GetTotalBabyStepOffset(axis));
					}
				}
			}
			break;

		case 291:	// Display message, optionally wait for acknowledgement
			{
				gb.MustSee('P');
				String<MaxMessageLength> message;
				gb.GetQuotedString(message.GetRef());

				bool dummy = false;
				String<MaxMessageLength> title;
				gb.TryGetQuotedString('R', title.GetRef(), dummy);

				int32_t sParam = 1;
				gb.TryGetIValue('S', sParam, dummy);
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
					gb.TryGetFValue('T', tParam, dummy);
				}
				else
				{
					tParam = 0.0;
				}

				if (sParam == 0 && tParam <= 0.0)
				{
					reply.copy("Attempt to create a message box that cannot be dismissed");
					result = GCodeResult::error;
					break;
				}

				AxesBitmap axisControls;
				for (size_t axis = 0; axis < numTotalAxes; axis++)
				{
					if (gb.Seen(axisLetters[axis]) && gb.GetIValue() > 0)
					{
						axisControls.SetBit(axis);
					}
				}

				// Don't lock the movement system, because if we do then only the channel that issues the M291 can move the axes
				if (sParam == 2 || sParam == 3)
				{
#if HAS_LINUX_INTERFACE
					if (reprap.UsingLinuxInterface())
					{
						gb.SetState(GCodeState::waitingForAcknowledgement);
					}
#endif
					if (Push(gb, true))					// stack the machine state including the file position
					{
						UnlockMovement(gb);												// allow movement so that e.g. an SD card print can call M291 and then DWC or PanelDue can be used to jog axes
						gb.WaitForAcknowledgement();						// flag that we are waiting for acknowledgement
					}
				}

				// Display the message box on all relevant devices. Acknowledging any one of them clears them all.
				const MessageType mt = GetMessageBoxDevice(gb);						// get the display device
				platform.SendAlert(mt, message.c_str(), title.c_str(), (int)sParam, tParam, axisControls);
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
				platform.MessageF(MessageType::LogInfo, "M292: cancelled: %s", (cancelled ? "true" : "false"));
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
			result = reprap.GetHeat().SetPidParameters(1, gb, reply);
			break;

		case 302: // Allow, deny or report cold extrudes and configure minimum extrusion/retraction temps
			{
				bool seen = false;
				if (gb.Seen('P'))
				{
					if (!LockMovementAndWaitForStandstill(gb))
					{
						return false;
					}
					seen = true;
					reprap.GetHeat().AllowColdExtrude(gb.GetIValue() > 0);
				}
				if (gb.Seen('S'))
				{
					if (!LockMovementAndWaitForStandstill(gb))
					{
						return false;
					}
					seen = true;
					reprap.GetHeat().SetExtrusionMinTemp(gb.GetFValue());
				}
				if (gb.Seen('R'))
				{
					if (!LockMovementAndWaitForStandstill(gb))
					{
						return false;
					}
					seen = true;
					reprap.GetHeat().SetRetractionMinTemp(gb.GetFValue());
				}
				if (!seen)
				{
					if (reprap.GetHeat().ColdExtrude())
					{
						reply.copy("Cold extrusion is allowed (use M302 P0 to forbid it)");
					}
					else
					{
						reply.printf("Cold extrusion is forbidden (use M302 P1 to allow it), min. extrusion temperature %.1fC, min. retraction temperature %.1fC",
										(double)reprap.GetHeat().GetExtrusionMinTemp(), (double)reprap.GetHeat().GetRetractionMinTemp());
					}
				}
			}
			break;

		case 303: // Run PID tuning
			result = reprap.GetHeat().TuneHeater(gb, reply);
			break;

		case 304: // Set/report heated bed PID values
			result = reprap.GetHeat().SetPidParameters(0, gb, reply);
			break;

		case 305: // Set/report specific heater parameters
			reply.copy("M305 has been replaced by M308 and M950 in RepRapFirmware 3");
			result = GCodeResult::error;
			break;

		case 307: // Set heater process model parameters
			result = reprap.GetHeat().SetOrReportHeaterModel(gb, reply);
			break;

		case 308:
			result = reprap.GetHeat().ConfigureSensor(gb, reply);
			break;

		case 350: // Set/report microstepping
			{
#if SUPPORT_CAN_EXPANSION
				AxesBitmap axesToUpdate;
#endif
				bool interp = (gb.Seen('I') && gb.GetIValue() > 0);
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
#if SUPPORT_CAN_EXPANSION
						axesToUpdate.SetBit(axis);
#endif
						const unsigned int microsteps = gb.GetUIValue();
						if (ChangeMicrostepping(axis, microsteps, interp, reply))
						{
							SetAxisNotHomed(axis);
						}
						else
						{
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
						const size_t drive = ExtruderToLogicalDrive(e);
#if SUPPORT_CAN_EXPANSION
						axesToUpdate.SetBit(drive);
#endif
						if (!ChangeMicrostepping(drive, eVals[e], interp, reply))
						{
							result = GCodeResult::error;
						}
					}
				}

				if (seen)
				{
#if SUPPORT_CAN_EXPANSION
					result = max(result, platform.UpdateRemoteStepsPerMmAndMicrostepping(axesToUpdate, reply));
#endif
				}
				else
				{
					reply.copy("Microstepping - ");
					for (size_t axis = 0; axis < numTotalAxes; ++axis)
					{
						bool actualInterp;
						const unsigned int microsteps = platform.GetMicrostepping(axis, actualInterp);
						reply.catf("%c:%u%s, ", axisLetters[axis], microsteps, (actualInterp) ? "(on)" : "");
					}
					reply.cat("E");
					for (size_t extruder = 0; extruder < numExtruders; extruder++)
					{
						bool actualInterp;
						const unsigned int microsteps = platform.GetMicrostepping(ExtruderToLogicalDrive(extruder), actualInterp);
						reply.catf(":%u%s", microsteps, (actualInterp) ? "(on)" : "");
					}
				}
			}
			break;

#if HAS_MASS_STORAGE
		case 374: // Save grid and height map to file
			result = SaveHeightMap(gb, reply);
			break;

		case 375: // Load grid and height map from file and enable compensation
			if (!LockMovementAndWaitForStandstill(gb))
			{
				return false;
			}
			result = LoadHeightMap(gb, reply);
			break;
#endif

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
			{
				currentZProbeNumber = (gb.Seen('P')) ? gb.GetUIValue() : 0;
				auto zp = platform.GetEndstops().GetZProbe(currentZProbeNumber);
				if (zp.IsNotNull() && zp->GetProbeType() != ZProbeType::none)
				{
					zp->SetDeployedByUser(false);							// pretend that the probe isn't deployed, to make sure we deploy it unconditionally
					DeployZProbe(gb, 401);
					zp->SetDeployedByUser(true);							// probe is now deployed
				}
			}
			break;

		case 402: // Retract Z probe
			{
				currentZProbeNumber = (gb.Seen('P')) ? gb.GetUIValue() : 0;
				auto zp = platform.GetEndstops().GetZProbe(currentZProbeNumber);
				if (zp.IsNotNull() && zp->GetProbeType() != ZProbeType::none)
				{
					zp->SetDeployedByUser(false);							// do this first, otherwise the probe won't be retracted
					RetractZProbe(gb, 402);
				}
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
				const unsigned int type = gb.Seen('S') ? gb.GetUIValue() : 0;
#if SUPPORT_CAN_EXPANSION
				const uint32_t board = (gb.Seen('B')) ? gb.GetUIValue() : 0;
				if (board != 0)
				{
					result = CanInterface::RemoteM408(board, type, gb, reply);
					break;
				}
#endif
				const int seq = gb.Seen('R') ? gb.GetIValue() : -1;
				if (&gb == auxGCode && (type == 0 || type == 2))
				{
					lastAuxStatusReportType = type;
				}

				outBuf = GenerateJsonStatusResponse(type, seq, (&gb == auxGCode) ? ResponseSource::AUX : ResponseSource::Generic);
				if (outBuf == nullptr)
				{
					result = GCodeResult::notFinished;			// we ran out of buffers, so try again later
				}
			}
			break;

#if SUPPORT_OBJECT_MODEL
		case 409: // Get object model values in JSON format
			{
				String<StringLength100> key;
				String<StringLength20> flags;
				bool dummy;
				gb.TryGetQuotedString('K', key.GetRef(), dummy);
				gb.TryGetQuotedString('F', flags.GetRef(), dummy);
				if (&gb == auxGCode)
				{
					lastAuxStatusReportType = ObjectModelAuxStatusReportType;
				}
				outBuf = reprap.GetModelResponse(key.c_str(), flags.c_str());
				if (outBuf == nullptr)
				{
					OutputBuffer::ReleaseAll(outBuf);
					// We don't delay and retry here, in case the user asked for too much of the object model in one go for the output buffers to contain it
					reply.copy("{\"err\":-1}\n");
				}
				if (&gb == auxGCode)
				{
					gb.ResetReportDueTimer();
				}
			}
			break;
#endif

		case 450: // Report printer mode
			reply.printf("PrinterMode:%s", GetMachineModeString());
			break;

		case 451: // Select FFF printer mode
			if (!LockMovementAndWaitForStandstill(gb))
			{
				return false;
			}
			machineType = MachineType::fff;
			reprap.StateUpdated();
			break;

#if SUPPORT_LASER
		case 452: // Select laser mode
			if (!LockMovementAndWaitForStandstill(gb))
			{
				return false;
			}

			machineType = MachineType::laser;
			Move::CreateLaserTask();

			if (gb.Seen('C'))
			{
				if (!platform.AssignLaserPin(gb, reply))
				{
					result = GCodeResult::error;
				}
			}
			if (gb.Seen('F') || gb.Seen('Q'))
			{
				platform.SetLaserPwmFrequency(gb.GetPwmFrequency());
			}
			if (result == GCodeResult::ok)
			{
				if (gb.Seen('S'))
				{
					laserPowerSticky = (gb.GetUIValue() == 1);
				}
				if (gb.Seen('R'))
				{
					laserMaxPower = max<float>(1.0, gb.GetFValue());
				}
			}
			reprap.StateUpdated();
			break;
#endif

		case 453: // Select CNC mode
			if (!LockMovementAndWaitForStandstill(gb))
			{
				return false;
			}

			// M453 may be repeated to set up multiple spindles, so only print the message on the initial switch
			if (machineType != MachineType::cnc)
			{
				machineType = MachineType::cnc;						// switch to CNC mode even if the spindle parameter is bad
				reprap.StateUpdated();
			}

			{
				const uint32_t slot = gb.Seen('S') ? gb.GetLimitedUIValue('S', MaxSpindles) : 0;		// may throw
				result = platform.AccessSpindle(slot).Configure(gb, reply);								// may throw
			}
			break;

#if HAS_MASS_STORAGE
		case 470: // mkdir
			{
				gb.MustSee('P');
				String<MaxFilenameLength> dirName;
				gb.GetQuotedString(dirName.GetRef());
				result = (MassStorage::MakeDirectory(dirName.c_str(), true)) ? GCodeResult::ok : GCodeResult::error;
			}
			break;

		case 471: // move/rename file/directory
			{
				gb.MustSee('S');
				String<MaxFilenameLength> oldVal;
				gb.GetQuotedString(oldVal.GetRef());
				String<MaxFilenameLength> newVal;
				gb.MustSee('T');
				gb.GetQuotedString(newVal.GetRef());
				if (gb.Seen('D') && gb.GetUIValue() == 1 && MassStorage::FileExists(oldVal.c_str()) && MassStorage::FileExists(newVal.c_str()))
				{
					if (MassStorage::Delete(newVal.c_str(), true))
					{
						result = GCodeResult::error;
						break;
					}
				}
				result = (MassStorage::Rename(oldVal.c_str(), newVal.c_str(), true)) ? GCodeResult::ok : GCodeResult::error;
			}
			break;

		case 486: // number object or cancel object
			result = buildObjects.HandleM486(gb, reply, outBuf);
			break;

		case 500: // Store parameters in config-override.g
			result = WriteConfigOverrideFile(gb, reply);
			break;
#endif

		case 501: // Load parameters from config-override.g
			if (!gb.MachineState().runningM502 && !gb.MachineState().runningM501)		// when running M502 we ignore config-override.g
			{
				if (runningConfigFile)
				{
					m501SeenInConfigFile = true;
				}
				DoFileMacro(gb, CONFIG_OVERRIDE_G, true, code);
			}
			break;

		case 502: // Revert to default "factory settings" ignoring values in config-override.g
			if (!gb.MachineState().runningM502)									// avoid recursion
			{
				if (!LockMovementAndWaitForStandstill(gb))
				{
					return false;
				}
				reprap.GetHeat().SwitchOffAll(true);							// turn heaters off before changing the models
				reprap.GetHeat().ResetHeaterModels();							// in case some heaters have no M307 commands in config.g
				reprap.GetMove().GetKinematics().SetCalibrationDefaults();		// in case M665/M666/M667/M669 in config.g don't define all the parameters
				platform.GetEndstops().SetZProbeDefaults();
				DoFileMacro(gb, CONFIG_FILE, true, code);
			}
			break;

#if HAS_MASS_STORAGE
		case 503: // List variable settings
			{
				if (!LockFileSystem(gb))
				{
					return false;
				}

				// Need a valid output buffer to continue
				if (!OutputBuffer::Allocate(outBuf))
				{
					// No buffer available, try again later
					return false;
				}

				// Read the entire file
				FileStore * const f = platform.OpenSysFile(CONFIG_FILE, OpenMode::read);
				if (f == nullptr)
				{
					reply.copy("Configuration file not found");
					result = GCodeResult::error;
				}
				else
				{
					char fileBuffer[FILE_BUFFER_SIZE];
					size_t bytesRead,
						bytesLeftForWriting = OutputBuffer::GetBytesLeft(outBuf);
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
						outBuf->cat(fileBuffer, bytesRead);
					}
					f->Close();
				}
			}
			break;

		case 505:	// set sys folder
			if (gb.Seen('P'))
			{
				// Lock movement to try to prevent other threads opening system files while we change the system path
				if (!LockMovementAndWaitForStandstill(gb))
				{
					return false;
				}
				String<MaxFilenameLength> path;
				gb.GetQuotedString(path.GetRef());
				result = platform.SetSysDir(path.c_str(), reply);
			}
			else
			{
				String<MaxFilenameLength> path;
				platform.AppendSysDir(path.GetRef());
				reply.printf("Sys file path is %s", path.c_str());
			}
			break;
#endif

		case 540: // Set/report MAC address
			if (CheckNetworkCommandAllowed(gb, reply, result))
			{
				const unsigned int interface = (gb.Seen('I') ? gb.GetUIValue() : 0);
				if (gb.Seen('P'))
				{
					MacAddress mac;
					gb.GetMacAddress(mac);
					result = reprap.GetNetwork().SetMacAddress(interface, mac, reply);
				}
				else
				{
					const MacAddress& mac = reprap.GetNetwork().GetMacAddress(interface);
					reply.printf("MAC: %02x:%02x:%02x:%02x:%02x:%02x", mac.bytes[0], mac.bytes[1], mac.bytes[2], mac.bytes[3], mac.bytes[4], mac.bytes[5]);
				}
			}
			break;

		case 550: // Set/report machine name
#if HAS_LINUX_INTERFACE
			if (reprap.UsingLinuxInterface() && !gb.IsBinary())
			{
				result = GCodeResult::errorNotSupported;
			}
			else
#endif
			{
				String<MachineNameLength> name;
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
				String<RepRapPasswordLength> password;
				bool seen = false;
				gb.TryGetPossiblyQuotedString('P', password.GetRef(), seen);
				if (seen)
				{
					reprap.SetPassword(password.c_str());
				}
			}
			break;

		case 552: // Enable/Disable network and/or Set/Get IP address
			if (CheckNetworkCommandAllowed(gb, reply, result))
			{
				bool seen = false;
				const unsigned int interface = (gb.Seen('I')) ? gb.GetUIValue() : 0;

				String<SsidBufferLength> ssid;
				if (reprap.GetNetwork().IsWiFiInterface(interface))
				{
					if (gb.Seen('S')) // Has the user turned the network on or off?
					{
						const int enableValue = gb.GetIValue();
						seen = true;

						if (gb.Seen('P'))
						{
							gb.GetQuotedString(ssid.GetRef());
						}
						result = reprap.GetNetwork().EnableInterface(interface, enableValue, ssid.GetRef(), reply);
					}
				}
				else
				{
					if (gb.Seen('P'))
					{
						seen = true;
						IPAddress eth;
						gb.GetIPAddress(eth);
						platform.SetIPAddress(eth);
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
			if (CheckNetworkCommandAllowed(gb, reply, result))
			{
				if (gb.Seen('P'))
				{
					IPAddress eth;
					gb.GetIPAddress(eth);
					platform.SetNetMask(eth);
				}
				else
				{
					const IPAddress nm = platform.NetMask();
					reply.printf("Net mask: %d.%d.%d.%d ", nm.GetQuad(0), nm.GetQuad(1), nm.GetQuad(2), nm.GetQuad(3));
				}
			}
			break;

		case 554: // Set/Get gateway
			if (CheckNetworkCommandAllowed(gb, reply, result))
			{
				if (gb.Seen('P'))
				{
					IPAddress eth;
					gb.GetIPAddress(eth);
					platform.SetGateWay(eth);
				}
				else
				{
					const IPAddress gw = platform.GateWay();
					reply.printf("Gateway: %d.%d.%d.%d ", gw.GetQuad(0), gw.GetQuad(1), gw.GetQuad(2), gw.GetQuad(3));
				}
			}
			break;

		case 555: // Set/report firmware type to emulate
			if (gb.Seen('P'))
			{
				gb.MachineState().compatibility.Assign(gb.GetIValue());
			}
			else
			{
				reply.printf("Output mode: %s", gb.MachineState().compatibility.ToString());
			}
			break;

		case 556: // Axis compensation (we support only X, Y, Z)
		{
			bool seen = false;

			if (gb.Seen('S'))
			{
				const float value = gb.GetFValue();
				if (value >= 10.0)			// avoid divide by zero and silly results
				{
					for (size_t axis = 0; axis <= Z_AXIS; axis++)
					{
						if (gb.Seen(axisLetters[axis]))
						{
							reprap.GetMove().SetAxisCompensation(axis, gb.GetFValue() / value);
							seen = true;
						}
					}
				}
			}

			if (gb.Seen('P'))
			{
				reprap.GetMove().SetXYCompensation(gb.GetIValue() <= 0);
				seen = true;
			}

			if (!seen)
			{
				reply.printf("Axis compensations - %s: %.5f, YZ: %.5f, ZX: %.5f",
					reprap.GetMove().IsXYCompensated() ? "XY" : "YX",
					(double)reprap.GetMove().AxisCompensation(X_AXIS), (double)reprap.GetMove().AxisCompensation(Y_AXIS), (double)reprap.GetMove().AxisCompensation(Z_AXIS));
			}
			break;
		}

		case 557: // Set/report Z probe point coordinates
			result = DefineGrid(gb, reply);
			break;

		case 558: // Set or report Z probe type and for which axes it is used
			result = platform.GetEndstops().HandleM558(gb, reply);
			break;

#if HAS_MASS_STORAGE
		case 559:
		case 560: // Binary writing
			{
				String<MaxFilenameLength> defaultFolder;
				if (code == 560)
				{
					defaultFolder.copy(platform.GetWebDir());
				}
				else
				{
					platform.AppendSysDir(defaultFolder.GetRef());
				}
				String<MaxFilenameLength> filename;
				gb.MustSee('P');
				gb.GetQuotedString(filename.GetRef());
				const FilePosition size = (gb.Seen('S') ? (FilePosition)gb.GetIValue() : 0);
				const uint32_t crc32 = (gb.Seen('C') ? gb.GetUIValue() : 0);
				const bool ok = gb.OpenFileToWrite(defaultFolder.c_str(), filename.c_str(), size, true, crc32);
				if (ok)
				{
					reply.printf("Writing to file: %s", filename.c_str());
				}
				else
				{
					reply.printf("Can't open file %s for writing", filename.c_str());
					result = GCodeResult::error;
				}
			}
			break;
#endif

		case 561: // Set identity transform and disable height map
			if (!LockMovementAndWaitForStandstill(gb))
			{
				return false;
			}
			ClearBedMapping();
			break;

		case 562: // Reset temperature fault - use with great caution
			if (gb.Seen('P'))
			{
				const unsigned int heater = gb.GetLimitedUIValue('P', MaxHeaters);
				result = reprap.ClearTemperatureFault(heater, reply);
			}
			else
			{
				// Clear all heater faults
				for (unsigned int heater = 0; heater < MaxHeaters; ++heater)
				{
					result = max<GCodeResult>(result, reprap.ClearTemperatureFault(heater, reply));
				}
			}
			heaterFaultState = HeaterFaultState::noFault;
			break;

		case 563: // Define tool
			result = ManageTool(gb, reply);
			break;

		case 564: // Think outside the box?
			{
				bool seen = false;
				if (gb.Seen('S'))
				{
					seen = true;
					limitAxes = (gb.GetIValue() > 0);
				}
				if (gb.Seen('H'))
				{
					seen = true;
					noMovesBeforeHoming = (gb.GetIValue() > 0);
				}
				if (!seen)
				{
					reply.printf("Movement outside the bed is %spermitted, movement before homing is %spermitted", (limitAxes) ? "not " : "", (noMovesBeforeHoming) ? "not " : "");
				}
			}
			break;

		case 205: // Set/print maximum jerk speeds in mm/sec
		case 566: // Set/print maximum jerk speeds in mm/min
			{
				const float multiplier1 = (code == 566) ? SecondsToMinutes : 1.0;
				bool seenAxis = false, seenExtruder = false;
				for (size_t axis = 0; axis < numTotalAxes; axis++)
				{
					if (gb.Seen(axisLetters[axis]))
					{
						platform.SetInstantDv(axis, gb.GetDistance() * multiplier1);
						seenAxis = true;
					}
				}

				if (gb.Seen(extrudeLetter))
				{
					seenExtruder = true;
					float eVals[MaxExtruders];
					size_t eCount = numExtruders;
					gb.GetFloatArray(eVals, eCount, true);
					for (size_t e = 0; e < eCount; e++)
					{
						platform.SetInstantDv(ExtruderToLogicalDrive(e), eVals[e] * multiplier1);
					}
				}

				if (code == 566 && gb.Seen('P'))
				{
					seenAxis = true;
					reprap.GetMove().SetJerkPolicy(gb.GetUIValue());
				}

				if (seenAxis)
				{
					reprap.MoveUpdated();
				}
				else if (!seenExtruder)
				{
					const float multiplier2 = (code == 566) ? MinutesToSeconds : 1.0;
					reply.printf("Maximum jerk rates (mm/%s): ", (code == 566) ? "min" : "sec");
					for (size_t axis = 0; axis < numTotalAxes; ++axis)
					{
						reply.catf("%c: %.1f, ", axisLetters[axis], (double)(platform.GetInstantDv(axis) * multiplier2));
					}
					reply.cat("E:");
					char sep = ' ';
					for (size_t extruder = 0; extruder < numExtruders; extruder++)
					{
						reply.catf("%c%.1f", sep, (double)(platform.GetInstantDv(ExtruderToLogicalDrive(extruder)) * multiplier2));
						sep = ':';
					}
					if (code == 566)
					{
						reply.catf(", jerk policy: %u", reprap.GetMove().GetJerkPolicy());
					}
				}
			}
			break;

		case 567: // Set/report tool mix ratios
			if (gb.Seen('P'))
			{
				const int8_t tNumber = gb.GetIValue();
				ReadLockedPointer<Tool> const tool = reprap.GetTool(tNumber);
				if (tool.IsNotNull())
				{
					if (gb.Seen(extrudeLetter))
					{
						float eVals[MaxExtruders];
						size_t eCount = tool->DriveCount();
						gb.GetFloatArray(eVals, eCount, false);
						if (eCount != tool->DriveCount())
						{
							reply.copy("Setting mix ratios - wrong number of E drives: ");
							gb.AppendFullCommand(reply);
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
			result = ConfigureDriver(gb, reply);
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
					seen = true;
					result = reprap.GetHeat().ConfigureHeaterMonitoring(gb.GetUIValue(), gb, reply);
				}
				if (!seen)
				{
					reply.printf("Print will be terminated if a heater fault is not reset within %" PRIu32 " minutes", heaterFaultTimeout/(60 * 1000));
				}
			}
			break;

		case 571: // Set output on extrude
			result = platform.GetSetAncillaryPwm(gb, reply);
			break;

		case 572: // Set/report pressure advance
			if (gb.Seen('S'))
			{
				const float advance = gb.GetFValue();
				if (!LockMovementAndWaitForStandstill(gb))
				{
					return false;
				}
				result = platform.SetPressureAdvance(advance, gb, reply);
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
			{
				const unsigned int heater = gb.GetLimitedUIValue('P', MaxHeaters);
				reply.printf("Average heater %u PWM: %.3f", heater, (double)reprap.GetHeat().GetAveragePWM(heater));
			}
			break;

		case 574: // Set endstop configuration
			// We may be about to delete endstops, so make sure we are not executing a move that uses them
			if (!LockMovementAndWaitForStandstill(gb))
			{
				return false;
			}
			result = platform.GetEndstops().HandleM574(gb, reply, outBuf);
			break;

		case 575: // Set communications parameters
			{
				const size_t chan = gb.GetLimitedUIValue('P', NumSerialChannels);
				bool seen = false;
				if (gb.Seen('B'))
				{
					platform.SetBaudRate(chan, gb.GetIValue());
					seen = true;
				}

				if (gb.Seen('S'))
				{
					const uint32_t val = gb.GetIValue();
					platform.SetCommsProperties(chan, val);
					if (chan == 0)
					{
						usbGCode->SetCommsProperties(val);
					}
#if HAS_AUX_DEVICES
					else if (chan < NumSerialChannels)
					{
						GCodeBuffer *& gbp = (chan == 1) ? auxGCode : aux2GCode;
						if (gbp != nullptr)
						{
							gbp->SetCommsProperties(val);
							const bool rawMode = (val & 2u) != 0;
							platform.SetAuxRaw(chan - 1, rawMode);
							if (rawMode && !platform.IsAuxEnabled(chan - 1))			// if enabling aux for the first time and in raw mode, set Marlin compatibility
							{
								gbp->MachineState().compatibility = Compatibility::Marlin;
							}
						}
					}
#endif
					seen = true;
				}

				if (seen)
				{
#if HAS_AUX_DEVICES
					if (chan != 0 && !platform.IsAuxEnabled(chan - 1))
					{
						platform.EnableAux(chan - 1);
					}
					else
					{
						platform.ResetChannel(chan);
					}
				}
				else if (chan != 0 && !platform.IsAuxEnabled(chan - 1))
				{
					reply.printf("Channel %u is disabled", chan);
#endif
				}
				else
				{
					const uint32_t cp = platform.GetCommsProperties(chan);
					reply.printf("Channel %d: baud rate %" PRIu32 ", %s checksum", chan, platform.GetBaudRate(chan), (cp & 1) ? "requires" : "does not require");
					if (chan == 0 && SERIAL_MAIN_DEVICE.IsConnected())
					{
						reply.cat(", connected");
					}
#if HAS_AUX_DEVICES
					else if (chan != 0 && platform.IsAuxRaw(chan - 1))
					{
						reply.cat(", raw mode");
					}
#endif
				}
			}
			break;

		case 577: // Wait until endstop input is triggered
			result = WaitForPin(gb, reply);
			break;

#if SUPPORT_INKJET
		case 578: // Fire Inkjet bits
			if (!LockMovementAndWaitForStandstill())
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
			result = ConfigureTrigger(gb, reply);
			break;

		case 582: // Check external trigger
			result = CheckTrigger(gb, reply);
			break;

		case 584: // Set axis/extruder to stepper driver(s) mapping
			result = DoDriveMapping(gb, reply);
			break;

		case 585: // Probe Tool
			result = ProbeTool(gb, reply);
			break;

		case 586: // Configure network protocols
			if (CheckNetworkCommandAllowed(gb, reply, result))
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
			if (CheckNetworkCommandAllowed(gb, reply, result))
			{
				result = reprap.GetNetwork().HandleWiFiCode(code, gb, reply, outBuf);
			}
			break;
#endif

		case 591: // Configure filament sensor
			{
				const unsigned int extruder = gb.GetLimitedUIValue('D', numExtruders);
				result = FilamentMonitor::Configure(gb, reply, extruder);
			}
			break;

#if SUPPORT_NONLINEAR_EXTRUSION
		case 592: // Configure nonlinear extrusion
			{
				const unsigned int extruder = gb.GetLimitedUIValue('D', MaxExtruders);
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

		case 593: // Configure dynamic ringing cancellation
			result = reprap.GetMove().ConfigureDynamicAcceleration(gb, reply);
			break;

#if SUPPORT_ASYNC_MOVES
		case 594:	// Enter or leave height following mode
			result = reprap.GetMove().StartHeightFollowing(gb, reply);
			break;
#endif

		// For cases 600 and 601, see 226

		// M650 (set peel move parameters) and M651 (execute peel move) are no longer handled specially. Use macros to specify what they should do.

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
					if (reprap.GetMove().GetKinematics().LimitPosition(moveBuffer.coords, nullptr, numVisibleAxes, axesVirtuallyHomed, false, false) != LimitPositionResult::ok)
					{
						ToolOffsetInverseTransform(moveBuffer.coords, currentUserPosition);	// make sure the limits are reflected in the user position
					}
					reprap.GetMove().SetNewPosition(moveBuffer.coords, true);
					SetAllAxesNotHomed();
					reprap.MoveUpdated();
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
					reprap.MoveUpdated();
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
				if (gb.Seen('S'))
				{
					// Switch to the correct CoreXY mode
					const int mode = gb.GetIValue();
					switch (mode)
					{
					case 0:
						move.SetKinematics(KinematicsType::cartesian);
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
					if (gb.Seen('X') || gb.Seen('Y') || gb.Seen('Z'))
					{
						reply.copy("M667 XYZ parameters are no longer supported, use M669 matrix parameters instead");
						result = GCodeResult::error;
					}

					if (seen)
					{
						// We changed something, so reset the positions and set all axes not homed
						if (move.GetKinematics().GetKinematicsType() != oldK)
						{
							move.GetKinematics().GetAssumedInitialPosition(numVisibleAxes, moveBuffer.coords);
							ToolOffsetInverseTransform(moveBuffer.coords, currentUserPosition);
						}
						if (reprap.GetMove().GetKinematics().LimitPosition(moveBuffer.coords, nullptr, numVisibleAxes, axesVirtuallyHomed, false, false) != LimitPositionResult::ok)
						{
							ToolOffsetInverseTransform(moveBuffer.coords, currentUserPosition);	// make sure the limits are reflected in the user position
						}
						reprap.GetMove().SetNewPosition(moveBuffer.coords, true);
						SetAllAxesNotHomed();
						reprap.MoveUpdated();
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
					if (reprap.GetMove().GetKinematics().LimitPosition(moveBuffer.coords, nullptr, numVisibleAxes, axesVirtuallyHomed, false, false) != LimitPositionResult::ok)
					{
						ToolOffsetInverseTransform(moveBuffer.coords, currentUserPosition);	// make sure the limits are reflected in the user position
					}
					reprap.GetMove().SetNewPosition(moveBuffer.coords, true);
					SetAllAxesNotHomed();
					reprap.MoveUpdated();
				}
			}
			break;

#if SUPPORT_IOBITS
		case 670:
			Move::CreateLaserTask();
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
			result = platform.GetEndstops().ProgramZProbe(gb, reply);
			break;

		case 673: // Align plane on rotary axis
			if (numTotalAxes < U_AXIS)
			{
				reply.copy("Insufficient axes configured");
				result = GCodeResult::error;
			}
			else if (LockMovementAndWaitForStandstill(gb))
			{
				if (reprap.GetMove().GetNumProbedProbePoints() < 2)
				{
					reply.copy("Insufficient probe points");
					result = GCodeResult::error;
				}
				else if (!AllAxesAreHomed())
				{
					reply.copy("Home the axes first");
					result = GCodeResult::error;
				}
				else
				{
					// See which rotary axis needs to be compensated (if any)
					size_t axisToUse = 0;
					for (size_t axis = U_AXIS; axis < numVisibleAxes; axis++)
					{
						if (gb.Seen(axisLetters[axis]))
						{
							axisToUse = axis;
							break;
						}
					}

					// Get the coordinates of the first two G30 points and calculate how far off the axis is
					float x1, y1, x2, y2;
					const float z1 = reprap.GetMove().GetProbeCoordinates(0, x1, y1, true);
					const float z2 = reprap.GetMove().GetProbeCoordinates(1, x2, y2, true);
					const float a1 = (x1 == x2) ? y1 : x1;
					const float a2 = (x1 == x2) ? y2 : x2;

					// See what kind of compensation we need to perform
					SetMoveBufferDefaults();
					if (axisToUse != 0)
					{
						// An axis letter is given, so try to level the given axis
						const float correctionAngle = atanf((z2 - z1) / (a2 - a1)) * 180.0 / M_PI;
						const float correctionFactor = gb.Seen('S') ? gb.GetFValue() : 1.0;
						moveBuffer.coords[axisToUse] += correctionAngle * correctionFactor;

						reply.printf("%c axis is off by %.2f deg", axisLetters[axisToUse], (double)correctionAngle);
						HandleReply(gb, GCodeResult::notFinished, reply.c_str());
					}
					else if (reprap.GetMove().GetNumProbedProbePoints() >= 4)
					{
						// At least four G30 points are given. This lets us figure out how far off the centre of the axis is
						const float z3 = reprap.GetMove().GetProbeCoordinates(2, x1, y1, true);
						const float z4 = reprap.GetMove().GetProbeCoordinates(3, x2, y2, true);
						const float a3 = (x1 == x2) ? y1 : x1;
						const float a4 = (x1 == x2) ? y2 : x2;

						// Calculate intersection points in [XY] and Z directions
						const float aS = ((a4 - a3) * (a2 * z1 - a1 * z2) - (a2 - a1) * (a4 * z3 - a3 * z4)) /
								((z4 - z3) * (a2 - a1) - (z2 - z1) * (a4 - a3));
						const float zS = ((z1 - z2) * (a4 * z3 - a3 * z4) - (z3 - z4) * (a2 * z1 - a1 * z2)) /
								((z4 - z3) * (a2 - a1) - (z2 - z1) * (a4 - a3));
						moveBuffer.coords[(x1 == x2) ? Y_AXIS : X_AXIS] += aS;
						moveBuffer.coords[Z_AXIS] += zS;

						reply.printf("%c is offset by %.2fmm, Z is offset by %.2fmm", (x2 == x1) ? 'Y' : 'X', (double)aS, (double)zS);
						HandleReply(gb, GCodeResult::notFinished, reply.c_str());
					}
					else
					{
						reply.copy("No rotary axis letter and/or not enough probe points for rotary axis alignment");
						result = GCodeResult::error;
						break;
					}

					// Get the feedrate (if any) and kick off a new move
					if (gb.Seen(feedrateLetter))
					{
						const float rate = gb.ConvertDistance(gb.GetFValue());
						gb.MachineState().feedRate = rate * SecondsToMinutes;		// don't apply the speed factor
					}
					moveBuffer.feedRate = gb.MachineState().feedRate;
					moveBuffer.usingStandardFeedrate = true;
					moveBuffer.tool = reprap.GetCurrentTool();
					NewMoveAvailable(1);

					gb.SetState(GCodeState::waitingForSpecialMoveToComplete);
				}
			}
			else
			{
				result = GCodeResult::notFinished;
			}
			break;

#if false
		// This code is not finished yet
		case 674: // Set Z to center point
			if (LockMovementAndWaitForStandstill(gb))
			{
				if (reprap.GetMove().GetNumProbedProbePoints() < 2)
				{
					reply.copy("Insufficient probe points");
					result = GCodeResult::error;
				}
				else if (!AllAxesAreHomed())
				{
					reply.copy("Home the axes first");
					result = GCodeResult::error;
				}
				else
				{
					float x, y;
					const float z1 = reprap.GetMove().GetProbeCoordinates(0, x, y, true);
					const float z2 = reprap.GetMove().GetProbeCoordinates(1, x, y, true);
					const float offset = gb.Seen('P') ? gb.GetFValue() : 0.0;
					currentUserPosition[Z_AXIS] -= (z1 + z2) / 2.0 + offset;

					ToolOffsetTransform(currentUserPosition, moveBuffer.coords);
					if (reprap.GetMove().GetKinematics().LimitPosition(moveBuffer.coords, numVisibleAxes, LowestNBits<AxesBitmap>(numVisibleAxes), false))	// pretend that all axes are homed
					{
						ToolOffsetInverseTransform(moveBuffer.coords, currentUserPosition);		// make sure the limits are reflected in the user position
					}
					reprap.GetMove().SetNewPosition(moveBuffer.coords, true);
					axesHomed |= reprap.GetMove().GetKinematics().AxesAssumedHomed(MakeBitmap<AxesBitmap>(Z_AXIS));

					reply.printf("Probe points at %.2f %.2f, setting new Z to %.2f", (double)z1, (double)z2, (double)currentUserPosition[Z_AXIS]);
				}
			}
			break;
#endif

		case 675: // Find center of cavity
			result = FindCenterOfCavity(gb, reply);
			break;

		case 701: // Load filament
			result = LoadFilament(gb, reply);
			break;

		case 702: // Unload filament
			result = UnloadFilament(gb, reply);
			break;

		case 703: // Configure Filament
			if (reprap.GetCurrentTool() != nullptr)
			{
				const Filament *filament = reprap.GetCurrentTool()->GetFilament();
				if (filament != nullptr && filament->IsLoaded())
				{
					String<StringLength256> scratchString;
					scratchString.printf("%s%s/%s", FILAMENTS_DIRECTORY, filament->GetName(), CONFIG_FILAMENT_G);
					DoFileMacro(gb, scratchString.c_str(), false, 703);
				}
			}
			else
			{
				result = GCodeResult::error;
				reply.copy("No tool selected");
			}
			break;

#if SUPPORT_SCANNER
		case 750: // Enable 3D scanner extension
			reprap.GetScanner().Enable();
			break;

		case 751: // Register 3D scanner extension over USB
			if (&gb == usbGCode)
			{
				if (reprap.GetScanner().IsEnabled())
				{
					reprap.GetScanner().Register();
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
			{
				gb.MustSee('P');
				String<MaxFilenameLength> file;
				gb.GetPossiblyQuotedString(file.GetRef());
				gb.MustSee('S');
				const int range = gb.GetIValue();
				if (reprap.GetScanner().IsEnabled())
				{
					if (reprap.GetScanner().IsRegistered())
					{
						const int resolution = gb.Seen('R') ? gb.GetIValue() : 100;
						const int mode = gb.Seen('N') ? gb.GetIValue() : 0;
						result = GetGCodeResultFromFinished(reprap.GetScanner().StartScan(file.c_str(), range, resolution, mode));
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
					const int mode = gb.Seen('N') ? gb.GetIValue() : 0;
					result = GetGCodeResultFromFinished(reprap.GetScanner().Calibrate(mode));
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

		case 851: // Set Z probe offset, only for Marlin compatibility
			{
				auto zp = platform.GetZProbeOrDefault(0);
				if (gb.Seen('Z'))
				{
					zp->SetTriggerHeight(-gb.GetFValue());
					zp->SetSaveToConfigOverride();
					reprap.SensorsUpdated();
				}
				else
				{
					reply.printf("Z probe offset is %.2fmm", (double)(-zp->GetConfiguredTriggerHeight()));
				}
			}
			break;

		case 905: // Set current RTC date and time
			result = SetDateTime(gb, reply);
			break;

		case 906: // Set/report Motor currents
		case 913: // Set/report motor current percent
#if HAS_SMART_DRIVERS
		case 917: // Set/report standstill motor current percentage
#endif
#if HAS_VOLTAGE_MONITOR
			if (gb.GetState() != GCodeState::powerFailPausing1)			// we don't wait for movement to stop if we are running the power fail script
#endif
			{
				if (!LockMovementAndWaitForStandstill(gb))
				{
					return false;
				}
			}
			{
				bool seen = false;
				for (size_t axis = 0; axis < numTotalAxes; axis++)
				{
					if (gb.Seen(axisLetters[axis]))
					{
						result = max(result, platform.SetMotorCurrent(axis, gb.GetFValue(), code, reply));
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
						result = max(result, platform.SetMotorCurrent(ExtruderToLogicalDrive(e), eVals[e], code, reply));
					}
				}

				if (code == 906 && gb.Seen('I'))
				{
					seen = true;
					platform.SetIdleCurrentFactor(gb.GetFValue()/100.0);
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
						reply.catf(":%d", (int)platform.GetMotorCurrent(ExtruderToLogicalDrive(extruder), code));
					}
					if (code == 906)
					{
						reply.catf(", idle factor %d%%", (int)(platform.GetIdleCurrentFactor() * 100.0));
					}
				}
			}
			break;

#if HAS_VOLTAGE_MONITOR
		case 911: // Enable auto save on loss of power
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
						reprap.StateUpdated();
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
			result = platform.ConfigureStallDetection(gb, reply, outBuf);
			break;
#endif

#if HAS_MASS_STORAGE
		case 916:
			if (!platform.SysFileExists(RESUME_AFTER_POWER_FAIL_G))
			{
				reply.copy("No resume file found");
				result = GCodeResult::error;
			}
			else if (!platform.SysFileExists(RESUME_PROLOGUE_G))
			{
				reply.printf("Resume prologue file '%s' not found", RESUME_PROLOGUE_G);
				result = GCodeResult::error;
			}
			else
			{
				DoFileMacro(gb, RESUME_AFTER_POWER_FAIL_G, true, 916);
			}
			break;
#endif

		// For case 917, see 906

#if SUPPORT_12864_LCD
		case 918: // Configure direct-connect display
# ifdef DUET_NG
			// On Duet 2 configuring the display may affect the number of supported stepper drivers, so wait until there is no movement
			if (!LockMovementAndWaitForStandstill(gb))
			{
				return false;
			}
# endif
			result = reprap.GetDisplay().Configure(gb, reply);
# ifdef DUET_NG
			platform.AdjustNumDrivers((reprap.GetDisplay().IsPresent()) ? 2 : 0);
# endif
			break;
#endif

		case 929: // Start/stop event logging
#if HAS_MASS_STORAGE
			result = platform.ConfigureLogging(gb, reply);
#else
			result = GCodeResult::warningNotSupported;
#endif
			break;

		case 950:	// configure I/O pins
			result = platform.ConfigurePort(gb, reply);
			break;

#if SUPPORT_ASYNC_MOVES
		case 951:	// configure height control
			result = reprap.GetMove().ConfigureHeightFollowing(gb, reply);
			break;
#endif

#if SUPPORT_CAN_EXPANSION
		case 952:	// change expansion board CAN address
			result = CanInterface::ChangeAddressAndNormalTiming(gb, reply);
			break;

		case 953:	// set CAN-FD data rate
			result = CanInterface::ChangeFastTiming(gb, reply);
			break;
#endif

		case 997:	// Perform firmware update
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
#if SUPPORT_CAN_EXPANSION
			if (gb.Seen('B'))
			{
				const uint32_t address = gb.GetUIValue();
				if (address != CanId::MasterAddress)
				{
					result = reprap.GetExpansion().ResetRemote(address, gb, reply);
					break;
				}
			}
#endif
#if HAS_AUX_DEVICES
			if (gb.Seen('A'))
			{
				const uint32_t serialChannel = gb.GetLimitedUIValue('A', NumSerialChannels, 1);
				const uint32_t auxChannel = serialChannel - 1;
				if (platform.IsAuxEnabled(auxChannel))
				{
					if (gb.Seen('P'))
					{
						String<StringLength20> eraseString;
						gb.GetQuotedString(eraseString.GetRef());
						if (eraseString.Equals("ERASE"))
						{
							platform.AppendAuxReply(auxChannel, panelDueCommandEraseAndReset, true);
						}
					}
					else
					{
						platform.AppendAuxReply(auxChannel, panelDueCommandReset, true);
					}
					break;
				}
			}
#endif

			if (!gb.DoDwellTime(1000))		// wait a second to allow the response to be sent back to the web server, otherwise it may retry
			{
				return false;
			}

			reprap.EmergencyStop();			// this disables heaters and drives - Duet WiFi pre-production boards need drives disabled here
			{
				SoftwareResetReason reason = SoftwareResetReason::user;
				if (gb.Seen('P'))
				{
					String<StringLength20> eraseString;
					gb.GetQuotedString(eraseString.GetRef());
					if (eraseString.Equals("ERASE"))
					{
						reason = SoftwareResetReason::erase;
					}
				}
				SoftwareReset(reason);			// doesn't return
			}
			break;

		default:
			// See if there is a file in /sys named Mxx.g
			if (code >= 0 && code < 10000)
			{
				String<StringLength20> macroName;
				macroName.printf("M%d.g", code);
				if (DoFileMacro(gb, macroName.c_str(), false, code))
				{
					break;
				}
			}
			result = GCodeResult::warningNotSupported;
			break;
		}

		return HandleResult(gb, result, reply, outBuf);
	}
	catch (...)
	{
		OutputBuffer::ReleaseAll(outBuf);
		throw;
	}
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
	}
	else if (gb.Seen('T'))
	{
		// We handle "T{expression}" as if it's "T "{expression}, also DSF may pass a T{expression} command in this way
		seen = true;
		toolNum = gb.GetIValue();
	}
	else if (gb.Seen('R'))
	{
		const unsigned int rpNumber = gb.GetIValue();
		if (rpNumber < ARRAY_SIZE(numberedRestorePoints))
		{
			seen = true;
			toolNum = numberedRestorePoints[rpNumber].toolNumber;
		}
		else
		{
			UnlockAll(gb);
			HandleReply(gb, GCodeResult::error, "T: bad restore point number");
			return true;			// bad restore point number so ignore the T command
		}
	}

	if (seen)
	{
		if (!LockMovementAndWaitForStandstill(gb))
		{
			return false;
		}

		if (buildObjects.IsCurrentObjectCancelled())
		{
			buildObjects.SetVirtualTool(toolNum);				// don't do the tool change, just remember which one we are supposed to use
		}
		else
		{
			const Tool * const oldTool = reprap.GetCurrentTool();
			// If old and new are the same we no longer follow the sequence. User can deselect and then reselect the tool if he wants the macros run.
			if (oldTool == nullptr || oldTool->Number() != toolNum)
			{
				StartToolChange(gb, toolNum, (gb.Seen('P')) ? gb.GetUIValue() : DefaultToolChangeParam);
				return true;									// proceeding with state machine, so don't unlock or send a reply
			}
			else
			{
				// Even though the tool is selected, we may have turned it off e.g. when upgrading the WiFi firmware or following a heater fault that has been cleared.
				// So make sure the tool heaters are on.
				reprap.SelectTool(toolNum, simulationMode != 0);
			}
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
	HandleReply(gb, GCodeResult::ok, reply.c_str());
	return true;
}

// This is called to handle internally-generated codes
bool GCodes::HandleQcode(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	// Currently we don't need to worry about whether we are simulating or not
	switch (gb.GetCommandNumber())
	{
	case 0:	// process a whole-line comment in the print file
		return ProcessWholeLineComment(gb, reply);

	default:
		return true;					// at present these are always internally-generated, so no handling of unknown codes
	}
}

// This is called to deal with the result of processing a G- or M-code
bool GCodes::HandleResult(GCodeBuffer& gb, GCodeResult rslt, const StringRef& reply, OutputBuffer *outBuf) noexcept
{
	if (outBuf != nullptr)
	{
		// We only ever have an OutputBuffer when rslt == GCodeResult::ok
		gb.StopTimer();
		UnlockAll(gb);
		HandleReply(gb, outBuf);
		return true;
	}

	switch (rslt)
	{
	case GCodeResult::notFinished:
		return false;

	case GCodeResult::warningNotSupported:
		gb.PrintCommand(reply);
		reply.cat(": Command is not supported");
		rslt = GCodeResult::warning;
		break;

	case GCodeResult::errorNotSupported:
		if (!gb.IsDoingLocalFile())
		{
			gb.PrintCommand(reply);
			reply.cat(": ");
		}
		reply.cat("Command is not supported");
		rslt = GCodeResult::error;
		break;

	case GCodeResult::notSupportedInCurrentMode:
		if (!gb.IsDoingLocalFile())
		{
			gb.PrintCommand(reply);
			reply.cat(": ");
		}
		reply.catf("Command is not supported in machine mode %s", GetMachineModeString());
		rslt = GCodeResult::error;
		break;

	case GCodeResult::badOrMissingParameter:
		if (!gb.IsDoingLocalFile())
		{
			gb.PrintCommand(reply);
			reply.cat(": ");
		}
		reply.cat("Bad or missing parameter");
		rslt = GCodeResult::error;
		break;

	case GCodeResult::remoteInternalError:
		if (!gb.IsDoingLocalFile())
		{
			gb.PrintCommand(reply);
			reply.cat(": ");
		}
		reply.cat("CAN-connected board reported internal error");
		rslt = GCodeResult::error;
		break;

	case GCodeResult::error:
	case GCodeResult::warning:
		if (!gb.IsDoingLocalFile())
		{
			String<StringLength50> scratchString;
			gb.PrintCommand(scratchString.GetRef());
			reply.Prepend(": ");
			reply.Prepend(scratchString.c_str());
		}
		break;

	default:
		break;
	}

	if (gb.MachineState().GetState() == GCodeState::normal)
	{
		gb.StopTimer();
		UnlockAll(gb);
		HandleReply(gb, rslt, reply.c_str());
	}
	return true;
}

// End
