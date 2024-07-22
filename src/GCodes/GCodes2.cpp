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

#if HAS_SBC_INTERFACE
# include <SBC/SbcInterface.h>
#endif

#include <Movement/Move.h>

#include <PrintMonitor/PrintMonitor.h>
#include <Platform/RepRap.h>
#include <Tools/Tool.h>
#include <Endstops/ZProbe.h>
#include <FilamentMonitors/FilamentMonitor.h>
#include <General/IP4String.h>
#include <Movement/StepperDrivers/DriverMode.h>
#include <Hardware/SoftwareReset.h>
#include <Hardware/ExceptionHandlers.h>
#include <Version.h>

#if SUPPORT_IOBITS
# include <Platform/PortControl.h>
#endif

#if HAS_NETWORKING
# include <Networking/Network.h>
#endif

#if SUPPORT_MQTT
# include <Networking/MQTT/MqttClient.h>
#endif

#if HAS_WIFI_NETWORKING
# include <Comms/FirmwareUpdater.h>
#endif

#if SUPPORT_DIRECT_LCD
# include <Display/Display.h>
#endif

#if SUPPORT_CAN_EXPANSION
# include <CAN/CanInterface.h>
# include <CAN/ExpansionManager.h>
#endif

#if SUPPORT_ACCELEROMETERS
# include <Accelerometers/Accelerometers.h>
#endif

#ifdef DUET3_ATE
# include <Duet3Ate.h>
#endif

#if HAS_MASS_STORAGE
# include <Platform/Logger.h>
#endif

// If the code to act on is completed, this returns true, otherwise false.
// It is called repeatedly for a given code until it returns true for that code.
bool GCodes::ActOnCode(GCodeBuffer& gb, const StringRef& reply) noexcept
{
#if SUPPORT_ASYNC_MOVES
	// If we are running multiple motion systems in single input stream mode and we have resumed after a pause, then we may need to skip some commands
	// Also if we did a synchronous pause then we need to sip the pause command
	{
		if (&gb == FileGCode() && gb.ExecutingAll())
		{
			const FilePosition offsetToSkipTo = GetMovementState(gb).fileOffsetToSkipTo;
			if (offsetToSkipTo != 0)
			{
				const FilePosition jobFilePos = gb.GetJobFilePosition();
				if (jobFilePos < offsetToSkipTo)
				{
					// Skip any command except M596
					if (!(gb.GetCommandLetter() == 'M' && gb.HasCommandNumber() && gb.GetCommandNumber() == 596))
					{
						return true;
					}
				}
				else if (jobFilePos == offsetToSkipTo)
				{
					// If we paused on a synchronous pause command, skip it
					int commandNumber;
					if (   gb.GetCommandLetter() == 'M'
						&& ((commandNumber = gb.GetCommandNumber()) == 226 || commandNumber == 600 || commandNumber == 601 || commandNumber == 25)
					   )
					{
						return true;
					}
				}
				else
				{
					GetMovementState(gb).fileOffsetToSkipTo = 0;			// clear this to speed up the test next time
				}
			}
		}
	}
#endif

	try
	{
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
		gb.StopTimer();
		UnlockAll(gb);
		HandleReply(gb, GCodeResult::error, reply.c_str());
		return true;
	}

	// If we get here then we didn't see a command that was worth parsing
	reply.printf("Bad command: ");
	gb.AppendFullCommand(reply);
	HandleReply(gb, GCodeResult::error, reply.c_str());
	return true;
}

// Handle G-command returning true if the command completed, false if this function needs to be called again to complete it
bool GCodes::HandleGcode(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	const int code = gb.GetCommandNumber();
	if (stopped)
	{
		HandleResult(gb, GCodeResult::stopped, reply, nullptr);
		return true;
	}

	GCodeResult result = GCodeResult::ok;
	if (IsSimulating() && code > 4 && code != 10 && code != 11 && code != 20 && code != 21 && (code < 53 || code > 59) && (code < 90 || code > 94))
	{
		HandleReply(gb, result, "");
		return true;														// we only simulate some gcodes
	}

	// The only queued GCodes are some subfunctions of G10, so we delay checking for queueing this command until we are in case 10 below

#if SUPPORT_ASYNC_MOVES
	const bool executing = gb.Executing();									// this is used by the BREAK_IF_NOT_PRIMARY macro and elsewhere
#endif
	if (gb.GetCommandFraction() > 0
		&& code != 38 && code != 59											// these are the only G-codes we implement that can have fractional parts
	   )
	{
#if SUPPORT_ASYNC_MOVES
		if (executing)
		{
			result = TryMacroFile(gb);
		}
		else
		{
			HandleReply(gb, result, "");
			return true;
		}
#else
		result = TryMacroFile(gb);
#endif
	}
	else
	{
#if SUPPORT_ASYNC_MOVES
# define BREAK_IF_NOT_EXECUTING	if (!executing) { break; }
#else
# define BREAK_IF_NOT_EXECUTING	// nothing
#endif
		switch (code)
		{
		case 0: // Rapid move
		case 1: // Ordinary move
			BREAK_IF_NOT_EXECUTING
			if (GetMovementState(gb).segmentsLeft != 0)						// do this check first to avoid locking movement unnecessarily
			{
				return false;
			}
			if (!LockMovement(gb))
			{
				return false;
			}
			try
			{
				if (!DoStraightMove(gb, code == 1))
				{
					return false;
				}
			} catch (const GCodeException& exc)
			{
				platform.GetEndstops().ClearEndstops();					// DoStraightMove may have enabled endstops before quitting, so disable them
				gb.SetState(GCodeState::abortWhenMovementFinished);		// empty the queue before ending simulation, and force the user position to be restored
				gb.LatestMachineState().SetError(exc);					// must do this *after* calling SetState
			}
			break;

		case 2: // Clockwise arc
		case 3: // Anti clockwise arc
			// We only support X and Y axes in these (and optionally Z for corkscrew moves), but you can map them to other axes in the tool definitions
			BREAK_IF_NOT_EXECUTING
			if (GetMovementState(gb).segmentsLeft != 0)					// do this check first to avoid locking movement unnecessarily
			{
				return false;
			}
			if (!LockMovement(gb))
			{
				return false;
			}
			try
			{
				if (!DoArcMove(gb, code == 2))
				{
					return false;
				}
			} catch (GCodeException& exc)
			{
				gb.SetState(GCodeState::abortWhenMovementFinished);		// empty the queue before ending simulation, and force the user position to be restored
				gb.LatestMachineState().SetError(exc);					// must do this *after* calling SetState
			}
			break;

		case 4: // Dwell
			result = DoDwell(gb);
			break;

		case 10: // Set/report offsets and temperatures, or retract
			{
				if (gb.Seen('L'))
				{
					const uint32_t ival = gb.GetUIValue();
					switch (ival)
					{
					case 1:
						BREAK_IF_NOT_EXECUTING
						result = SetOrReportOffsets(gb, reply, 10);			// same as G10 with offsets and no L parameter
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
				{
					BREAK_IF_NOT_EXECUTING
					bool modifyingTool = gb.Seen('P') || gb.Seen('R') || gb.Seen('S');
					for (size_t axis = 0; axis < numVisibleAxes; ++axis)
					{
						modifyingTool |= gb.Seen(axisLetters[axis]);
					}

					if (modifyingTool)
					{
						if (IsSimulating())
						{
							break;
						}

						// Should we queue this code?
						if (gb.CanQueueCodes())
						{
							GCodeQueue * const codeQueue = GetMovementState(gb).codeQueue;
							if (codeQueue->ShouldQueueG10(gb))
							{
								// Don't queue any GCodes if there are segments not yet picked up by Move, because in the event that a segment corresponds to no movement,
								// the move gets discarded, which throws out the count of scheduled moves and hence the synchronisation
								if (GetMovementState(gb).segmentsLeft == 0 && codeQueue->QueueCode(gb))
								{
									HandleReply(gb, GCodeResult::ok, "");
									return true;
								}

								return false;		// we should queue this code but we can't yet, so wait until we can either execute it or queue it
							}
						}

						// We don't want to queue it
						result = SetOrReportOffsets(gb, reply, 10);
					}
					else
					{
						result = RetractFilament(gb, true);
					}
				}
			}
			break;

		case 11: // Un-retract
			BREAK_IF_NOT_EXECUTING
			result = RetractFilament(gb, false);
			break;

		case 17:	// Select XY plane for G2/G3
		case 18:	// Select XZ plane
		case 19:	// Select YZ plane
			// If the plane is changing, wait for motion to stop so that if we get a power failure then the selected plane will match
			// any not-yet-executed arc moves in the queue and we can resurrect those moves.
			// However, some generators prefix every G2/G3 command with G17/18/19 so don't wait if the plane is not changing.
			{
				const unsigned int newPlane = (unsigned int)code - 17;
				if (newPlane != gb.LatestMachineState().selectedPlane)
				{
					if (!LockCurrentMovementSystemAndWaitForStandstill(gb))
					{
						return false;
					}
					gb.LatestMachineState().selectedPlane = newPlane;
					reprap.InputsUpdated();
				}
			}
			break;

		case 20: // Inches (which century are we living in, here?)
		case 21: // mm
			gb.UseInches(code == 20);
			reprap.InputsUpdated();
			break;

		case 28: // Home
			if (!LockCurrentMovementSystemAndWaitForStandstill(gb))
			{
				return false;
			}
			BREAK_IF_NOT_EXECUTING
			result = DoHome(gb, reply);
			break;

		case 29: // Grid-based bed probing
			if (!LockCurrentMovementSystemAndWaitForStandstill(gb))			// do this first to make sure that a new grid isn't being defined
			{
				return false;
			}
			BREAK_IF_NOT_EXECUTING
			{
				int sparam;
				if (gb.Seen('S'))
				{
					sparam = gb.GetIValue();
				}
				else if (DoFileMacroWithParameters(gb, MESH_G, false, 29))	// no S parameter found so try to execute mesh.g
				{
					break;
				}
				else
				{
					sparam = 0;												// mesh.g not found, so treat G29 the same as G29 S0
				}

				switch(sparam)
				{
				case 0:		// probe and save height map
					result = ProbeGrid(gb, reply);
					break;

				case 1:		// load height map file
#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE
					result = LoadHeightMap(gb, reply);
#else
					result = GCodeResult::errorNotSupported;
#endif
					break;

				case 2:		// clear height map
					ClearBedMapping();
#if SUPPORT_PROBE_POINTS_FILE
					ClearProbePointsInvalid();
#endif
					break;

				case 3:		// save height map to names file
#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE
					result = SaveHeightMap(gb, reply);
#else
					result = GCodeResult::errorNotSupported;
#endif
					break;

				case 4:		// read grid parameters and can/cannot probe map
#if (HAS_MASS_STORAGE || HAS_SBC_INTERFACE) && SUPPORT_PROBE_POINTS_FILE
					result = LoadProbePointsMap(gb, reply);
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
			if (!LockCurrentMovementSystemAndWaitForStandstill(gb))
			{
				return false;
			}

			BREAK_IF_NOT_EXECUTING
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
			BREAK_IF_NOT_EXECUTING
			result = platform.GetEndstops().HandleG31(gb, reply);
			break;

		case 32: // Probe Z at multiple positions and generate the bed transform
			if (!LockCurrentMovementSystemAndWaitForStandstill(gb))
			{
				return false;
			}
			BREAK_IF_NOT_EXECUTING

#if SUPPORT_ASYNC_MOVES
			AllocateAxes(gb, GetMovementState(gb), AxesBitmap::MakeFromBits(Z_AXIS), ParameterLetterToBitmap('Z'));
#endif
			// We need to unlock the movement system here in case there is no Z probe and we are doing manual probing.
			// Otherwise, even though the bed probing code calls UnlockAll when doing a manual bed probe, the movement system
			// remains locked because the current MachineState object already held the lock when the macro file was started,
			// which means that no gcode source other than the one that executed G32 is allowed to jog the Z axis.
			UnlockAll(gb);

			DoFileMacroWithParameters(gb, BED_EQUATION_G, true, 32);	// Try to execute bed.g
			break;

		case 38: // Straight probe - move until either the probe is triggered or the commanded move ends
			if (!LockCurrentMovementSystemAndWaitForStandstill(gb))
			{
				return false;
			}
			BREAK_IF_NOT_EXECUTING
			result = StraightProbe(gb, reply);
			break;

		case 53:	// Temporarily use machine coordinates
			BREAK_IF_NOT_EXECUTING
			gb.LatestMachineState().g53Active = true;
			break;

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
					GetMovementState(gb).currentCoordinateSystem = cs;							// this is the zero-base coordinate system number
					reprap.MoveUpdated();
					gb.LatestMachineState().g53Active = false;									// cancel any active G53
				}
				else
				{
					result = GCodeResult::errorNotSupported;
				}
			}
			break;

		case 60: // Save position
			BREAK_IF_NOT_EXECUTING
			result = SavePosition(gb, reply);
			break;

#if SUPPORT_COORDINATE_ROTATION
		case 68:	// Coordinate rotation
			result = HandleG68(gb, reply);
			break;

		case 69:	// Cancel coordinate rotation
			if (!LockCurrentMovementSystemAndWaitForStandstill(gb))
			{
				return false;
			}
# if SUPPORT_ASYNC_MOVES
			if (gb.Executing())
# endif
			{
				g68Angle = 0.0;
				UpdateCurrentUserPosition(gb);
			}
			break;
#endif

		case 90: // Absolute coordinates
			gb.LatestMachineState().axesRelative = false;
			reprap.InputsUpdated();
			break;

		case 91: // Relative coordinates
			gb.LatestMachineState().axesRelative = true;   // Axis movements (i.e. X, Y and Z)
			reprap.InputsUpdated();
			break;

		case 92: // Set position
			BREAK_IF_NOT_EXECUTING
			result = SetPositions(gb, reply);
			break;

		case 93:	// inverse time mode
			gb.LatestMachineState().inverseTimeMode = true;
			reprap.InputsUpdated();
			break;

		case 94:	//normal feed rate mode
			gb.LatestMachineState().inverseTimeMode = false;
			reprap.InputsUpdated();
			break;

		default:
#if HAS_SBC_INTERFACE
			// Send unknown non-binary codes to DSF so potential plugins can interpret them
			if (reprap.UsingSbcInterface() && reprap.GetSbcInterface().IsConnected() && !gb.IsBinary())
			{
				gb.SendToSbc();
				return false;
			}
#endif
			result = TryMacroFile(gb);
			break;
		}
#undef BREAK_IF_NOT_EXECUTING
	}

	return HandleResult(gb, result, reply, nullptr);
}

bool GCodes::HandleMcode(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	const int code = gb.GetCommandNumber();
	if (stopped && code != 105 && code != 112 && code != 115 && code != 122 && code != 408 && code != 409 && code != 999)
	{
		HandleResult(gb, GCodeResult::stopped, reply, nullptr);
		return true;
	}

	// In simulation mode we don't execute most M-commands
	if (   IsSimulating()
		&& (code < 20 || code > 37)
		&& code != 0 && code != 1 && code != 82 && code != 83 && code != 105 && code != 109 && code != 111 && code != 112 && code != 122
		&& code != 200 && code != 204 && code != 207 && code != 408 && code != 409 && code != 486 && code != 999)
	{
		HandleReply(gb, GCodeResult::ok, "");
		return true;			// we don't simulate most M codes
	}

#if SUPPORT_ASYNC_MOVES
	// If we are not the primary GCode reader for this stream, we don't execute most M-commands
	if (!gb.Executing())
	{
		switch (code)
		{
		case 0:
		case 1:
		case 2:
		case 23:
		case 24:
		case 32:
		case 37:
		case 82:
		case 83:
		case 98:
		case 99:
		case 112:
		case 116:
		case 120:
		case 121:
		case 190:
		case 191:
		case 555:
		case 596:
		case 598:
			// These commands are executed by all GCode processors, at least to start with
			break;

		default:
			// All remaining commands are executed by the primary processor only
			HandleReply(gb, GCodeResult::ok, "");
			return true;
		}
	}
#endif

	// Can we queue this code?
	if (gb.CanQueueCodes())
	{
		GCodeQueue * const codeQueue = GetMovementState(gb).codeQueue;
		if (codeQueue->ShouldQueueMCode(gb))
		{
			// Don't queue any GCodes if there are segments not yet picked up by Move, because in the event that a segment corresponds to no movement,
			// the move gets discarded, which throws out the count of scheduled moves and hence the synchronisation
			if (GetMovementState(gb).segmentsLeft == 0 && codeQueue->QueueCode(gb))
			{
				HandleReply(gb, GCodeResult::ok, "");
				return true;
			}

			return false;		// we should queue this code but we can't yet, so wait until we can either execute it or queue it
		}
	}

#if HAS_SBC_INTERFACE
	// Pass file- and system-related commands to the SBC service if they came from somewhere else.
	// They will be passed back to us via a binary buffer or separate SPI message if necessary.
	if (   reprap.UsingSbcInterface() && reprap.GetSbcInterface().IsConnected() && !gb.IsBinary()
		&& (   (code >=  0 && code <= 2)
			|| (code >= 20 && code <= 24) || (code >= 26 && code <= 30)
			||  code == 32 || code == 36 || code == 37 || code == 38 || code == 39
			|| (code == 98 && gb.Seen('R'))
			||  code == 112
			||  code == 121
			|| (code >= 470 && code <= 472)
			||  code == 503 || code == 505
			||  code == 540 || (code >= 550 && code <= 552) || (code >= 586 && code <= 589)
			||  code == 596 || code == 606
			||  code == 703
			||  code == 905 || code == 929 || code == 997 || code == 999
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

		GCodeResult result;
		if (gb.GetCommandFraction() > 0
			&& code != 36 && code != 201
#if SUPPORT_SCANNING_PROBES
			&& code != 558
#endif
			&& code != 569 && code != 586 && code != 587 // these are the only M-codes we implement that can have fractional parts
		   )
		{
			result = TryMacroFile(gb);
		}
		else
		{
			result = GCodeResult::ok;
			switch (code)
			{
			case 0: // Stop
			case 1: // Sleep
			case 2: // Stop
				if (gb.IsFileChannel())
				{
					// Stopping a job because of a command in the file
#if SUPPORT_ASYNC_MOVES
					if (!DoSync(gb))
					{
						return false;
					}
#else
					if (!LockCurrentMovementSystemAndWaitForStandstill(gb))	// wait until everything has stopped and deferred command queue has caught up
					{
						return false;
					}
#endif
					StopPrint(&gb, StopPrintReason::normalCompletion);
				}
				else if (pauseState == PauseState::paused)
				{
					// Cancelling a print that has been paused
					if (!LockAllMovementSystemsAndWaitForStandstill(gb))	// make sure everything has stopped
					{
						return false;
					}
					const bool wasSimulating = IsSimulating();		// simulationMode may get cleared by CancelPrint
					StopPrint(nullptr, StopPrintReason::userCancelled);
					if (!wasSimulating)								// don't run any macro files or turn heaters off etc. if we were simulating before we stopped the print
					{
						// If cancel.g exists then run it and do nothing else
						gb.SetState(GCodeState::cancelling);
						if (DoFileMacro(gb, CANCEL_G, false, SystemHelperMacroCode))
						{
							pauseState = PauseState::cancelling;
							break;
						}
						if (!DoFileMacro(gb, STOP_G, false, SystemHelperMacroCode))
						{
							reprap.GetHeat().SwitchOffAll(true);
						}
					}
				}
				else
				{
					reply.copy("Pause the print before attempting to cancel it");
					result = GCodeResult::error;
				}
				break;

			case 3: // Spin spindle clockwise
			case 4: // Spin spindle counter clockwise
				{
					MovementState& ms = GetMovementState(gb);
#if SUPPORT_LASER
					if (machineType == MachineType::laser)
					{
						if (code == 3 && gb.Seen('S'))
						{
							if (ms.segmentsLeft != 0)
							{
								return false;						// don't modify moves that haven't gone yet
							}
							ms.laserPixelData.pixelPwm[0] = ConvertLaserPwm(gb.GetNonNegativeFValue());
							ms.laserPixelData.numPixels = 1;
						}
						else
						{
							result = GCodeResult::notSupportedInCurrentMode;
						}
					}
					else
#endif
					{
						// Determine what spindle number we are using
						uint32_t slot;
						if (gb.Seen('P'))
						{
							slot = gb.GetLimitedUIValue('P', MaxSpindles);
						}
						else if (ms.currentTool != nullptr && ms.currentTool->GetSpindleNumber() >= 0)
						{
							slot = ms.currentTool->GetSpindleNumber();
						}
						else
						{
							reply.copy("No P parameter and no active tool with spindle");
							result = GCodeResult::error;
							break;
						}

						Spindle& spindle = platform.AccessSpindle(slot);
						if (gb.Seen('S'))
						{
							const uint32_t rpm = gb.GetUIValue();
							if (ms.currentTool != nullptr && ms.currentTool->GetSpindleNumber() == (int)slot)
							{
								ms.currentTool->SetSpindleRpm(rpm, true);
							}
							else
							{
								spindle.SetConfiguredRpm(rpm, false);
							}
						}
						spindle.SetState((code == 4) ? SpindleState::reverse : SpindleState::forward);
					}
				}
				break;

			case 5: // Spindle motor off
				{
					MovementState& ms = GetMovementState(gb);
#if SUPPORT_LASER
					if (machineType == MachineType::laser)
					{
						if (ms.segmentsLeft != 0)
						{
							return false;						// don't modify moves that haven't gone yet
						}
						ms.laserPixelData.Clear();
					}
					else
#endif
					{
						// Determine what spindle number we are using
						uint32_t slot;
						if (gb.Seen('P'))
						{
							slot = gb.GetLimitedUIValue('P', MaxSpindles);
						}
						else if (ms.currentTool != nullptr && ms.currentTool->GetSpindleNumber() >= 0)
						{
							slot = ms.currentTool->GetSpindleNumber();
						}
						else
						{
							// Turn off every spindle if no 'P' parameter is present and the current tool does not have a spindle
							for (size_t i = 0; i < MaxSpindles; i++)
							{
								platform.AccessSpindle(i).SetState(SpindleState::stopped);
							}
							break;
						}

						platform.AccessSpindle(slot).SetState(SpindleState::stopped);
					}
				}
				break;

			case 17: // Motors on
			case 18: // Motors off
			case 84:
				if (!LockCurrentMovementSystemAndWaitForStandstill(gb))
				{
					return false;
				}
				{
					bool seen = false;
					for (size_t axis = 0; axis < numTotalAxes; axis++)
					{
						if (gb.Seen(axisLetters[axis]))
						{
							if (code == 17)
							{
								platform.EnableDrivers(axis, true);
							}
							else
							{
								SetAxisNotHomed(axis);
								platform.DisableDrivers(axis);
							}
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
							if (code == 17)
							{
								platform.EnableDrivers(ExtruderToLogicalDrive(eDrive[i]), true);
							}
							else
							{
								platform.DisableDrivers(ExtruderToLogicalDrive(eDrive[i]));
							}
						}
					}

					if (gb.Seen('S'))
					{
						seen = true;
						reprap.GetMove().SetIdleTimeout(gb.GetPositiveFValue());
					}

					if (!seen)
					{
						if (code == 17)
						{
							for (size_t axis = 0; axis < numTotalAxes; ++axis)
							{
								reprap.GetPlatform().EnableDrivers(axis, true);
							}
							for (size_t extruder = 0; extruder < numExtruders; ++extruder)
							{
								reprap.GetPlatform().EnableDrivers(ExtruderToLogicalDrive(extruder), true);
							}
						}
						else
						{
							DisableDrives();
						}
					}
				}
				break;

#if HAS_MASS_STORAGE || HAS_EMBEDDED_FILES
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
						dir.copy(Platform::GetGCodeDir());
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
						if (gb.LatestMachineState().compatibility == Compatibility::Default || gb.LatestMachineState().compatibility == Compatibility::RepRapFirmware)
						{
							outBuf->copy("GCode files:\n");
						}

						const bool encapsulateList = gb.LatestMachineState().compatibility != Compatibility::Marlin;
						FileInfo fileInfo;
						if (MassStorage::FindFirst(dir.c_str(), fileInfo))
						{
							// Iterate through all entries and append each file name
							bool first = true;
							do
							{
								if (encapsulateList)
								{
									outBuf->catf((first) ? "\"%s\"" : ",\"%s\"", fileInfo.fileName.c_str());
									first = false;
								}
								else
								{
									outBuf->catf("%s\n", fileInfo.fileName.c_str());
								}
							} while (MassStorage::FindNext(fileInfo));
						}
						else if (!encapsulateList)
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
#endif

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE || HAS_EMBEDDED_FILES
			case 23: // Set file to print
			case 32: // Select file and start SD print
				// We now allow a file that is being printed to chain to another file. This is required for the resume-after-power-fail functionality.
				if (FileGCode()->IsDoingFile() && !gb.IsFileChannel())
				{
					reply.copy("Cannot set file to print, because a file is already being printed");
					result = GCodeResult::error;
					break;
				}

# if SUPPORT_ASYNC_MOVES
				if (!DoSync(gb))
				{
					return false;
				}

				// Currently, both movement systems must execute the same file
				for (MovementState& ms : moveStates)
				{
					ms.fileOffsetToPrint = 0;				// clear this for now, M26 may change it later
					ms.fileOffsetToSkipTo = 0;
				}
# else
				if (code == 32 && !LockCurrentMovementSystemAndWaitForStandstill(gb))
				{
					return false;
				}
				moveStates[0].fileOffsetToPrint = 0;
# endif
				{
					String<MaxFilenameLength> filename;
					gb.GetUnprecedentedString(filename.GetRef());
					if (
# if HAS_SBC_INTERFACE
						reprap.UsingSbcInterface()
#  if HAS_MASS_STORAGE || HAS_EMBEDDED_FILES
						||
#  endif
# endif
# if HAS_MASS_STORAGE || HAS_EMBEDDED_FILES
						QueueFileToPrint(filename.c_str(), reply)
# endif
					   )
					{
						reprap.GetPrintMonitor().StartingPrint(filename.c_str());
						if (gb.LatestMachineState().compatibility == Compatibility::Marlin)
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
#if SUPPORT_ASYNC_MOVES
					if (!DoSync(gb))
					{
						return false;
					}
#else
					if (!LockCurrentMovementSystemAndWaitForStandstill(gb))
					{
						return false;
					}
#endif
					if (pauseState == PauseState::paused)
					{
#if HAS_VOLTAGE_MONITOR
						if (!IsSimulating() && !platform.IsPowerOk())
						{
							reply.copy("Cannot resume while power voltage is low");
							result = GCodeResult::error;
						}
						else
#endif
						{
							pauseState = PauseState::resuming;
							gb.SetState(GCodeState::resuming1);
							if (AllAxesAreHomed() && (!gb.Seen('P') || gb.GetUIValue() != 0))		// P0 parameter skips running resume.g
							{
								DoFileMacro(gb, RESUME_G, true, SystemHelperMacroCode);
							}
						}
					}
#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE || HAS_EMBEDDED_FILES
					else if (
# if HAS_MASS_STORAGE || HAS_EMBEDDED_FILES
								!fileToPrint.IsLive()
# else
								true
# endif
# if HAS_SBC_INTERFACE
								&& !reprap.UsingSbcInterface()
# endif
							)
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
							const bool fromStart = (moveStates[0].fileOffsetToPrint == 0);
							if (!fromStart)
							{
								// We executed M26 to set the file offset, which normally means that we are executing resurrect.g.
								// We need to copy the absolute/relative and volumetric extrusion flags over
								FileGCode()->OriginalMachineState().CopyStateFrom(gb.LatestMachineState());
# if SUPPORT_ASYNC_MOVES
								File2GCode()->OriginalMachineState().CopyStateFrom(gb.LatestMachineState());
# endif
								for (MovementState& ms : moveStates)
								{
									ms.moveFractionToSkip = ms.restartMoveFractionDone;
								}
							}
							StartPrinting(fromStart);
						}
					}
#endif
				}
				break;

			case 226: // Synchronous pause, normally initiated from within the file being printed
			case 600: // Filament change pause, synchronous
			case 601: // Pause, used on Prusa printers
				if (!gb.IsFileChannel())
				{
					reply.copy("use M226 and M601 only within a file being printed");		//TODO handle streaming over USB too
					result = GCodeResult::error;
				}
				else if (pauseState == PauseState::notPaused)
				{
					if (gb.IsDoingFileMacro() && !gb.LatestMachineState().CanRestartMacro())
					{
						if (deferredPauseCommandPending == nullptr)	// filament change pause takes priority
						{
							deferredPauseCommandPending = (code == 600) ? "M600"
																: (code == 601) ? "M601"
																	: (gb.Seen('P') && gb.GetUIValue() == 0) ? "M226 P0"
																		: "M226";
						}
					}
					else if (!DoSynchronousPause(gb,
												(code == 600) ?  PrintPausedReason::filamentChange
													: PrintPausedReason::gcode,
												(code == 600) ? GCodeState::filamentChangePause1
													: (code == 226 && gb.Seen('P') && gb.GetUIValue() == 0) ? GCodeState::pausing2
														: GCodeState::pausing1))
					{
						return false;
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
				else if (gb.IsFileChannel())
				{
					if (!DoSynchronousPause(gb, PrintPausedReason::user, GCodeState::pausing1))
					{
						return false;
					}
				}
				else if (FileGCode()->IsDoingFileMacro() && !FileGCode()->LatestMachineState().CanRestartMacro())
				{
					if (deferredPauseCommandPending == nullptr)		// filament change pause takes priority
					{
						deferredPauseCommandPending = (gb.Seen('P') && gb.GetUIValue() == 0) ? "M226 P0" : "M226";
					}
					if (!gb.IsFileChannel())
					{
						return false;								// wait for the current macro to finish
					}
				}
				else if (!DoAsynchronousPause(gb, PrintPausedReason::user, GCodeState::pausing1))
				{
					return false;
				}
				break;

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE || HAS_EMBEDDED_FILES
			case 26: // Set SD position
				// This is used between executing M23 to set up the file to print, and M24 to print it
				// When using multiple motion systems, execute M26 once in the context of each motion system
				{
					static_assert(sizeof(FilePosition) == sizeof(uint32_t), "Only 32 bits of file position are read");
					gb.MustSee('S');
					MovementState& ms = GetMovementState(gb);
					ms.fileOffsetToPrint = (FilePosition)gb.GetUIValue();
					ms.restartMoveFractionDone = (gb.Seen('P')) ? constrain<float>(gb.GetFValue(), 0.0, 1.0) : 0.0;
					const unsigned int selectedPlane = gb.LatestMachineState().selectedPlane;
					const char c0 = (selectedPlane == 2) ? 'Y' : 'X';
					const char c1 = (selectedPlane == 0) ? 'Y' : 'Z';
					ms.restartInitialUserC0 = (gb.Seen(c0)) ? gb.GetFValue() : 0.0;
					ms.restartInitialUserC1 = (gb.Seen(c1)) ? gb.GetFValue() : 0.0;
				}
				break;
#endif

#if HAS_MASS_STORAGE || HAS_EMBEDDED_FILES
			case 27: // Report print status - Deprecated
				if (reprap.GetPrintMonitor().IsPrinting())
				{
					// Pronterface keeps sending M27 commands if "Monitor status" is checked, and it specifically expects the following response syntax
					FileData& fileBeingPrinted = FileGCode()->OriginalMachineState().fileState;
					// In case there are short periods of time when PrintMonitor says a file is printing but the file is not open, or DSF passes M27 to us, check that we have a file
					if (fileBeingPrinted.IsLive())
					{
						reply.printf("SD printing byte %lu/%lu", GetPrintingFilePosition(), fileBeingPrinted.Length());
						break;
					}
				}
				reply.copy("Not SD printing.");
				break;
#endif

#if HAS_MASS_STORAGE
			case 28: // Write to file
				{
					String<MaxFilenameLength> filename;
					gb.GetUnprecedentedString(filename.GetRef());
					const bool ok = gb.OpenFileToWrite(Platform::GetGCodeDir(), filename.c_str(), 0, false, 0);
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
					if (platform.Delete(Platform::GetGCodeDir(), filename.c_str()))
					{
						result =  GCodeResult::ok;
					}
					else
					{
						reply.copy("delete failed");
						result = GCodeResult::error;
					}
				}
				break;
#endif

			// For case 32, see case 23

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE || HAS_EMBEDDED_FILES
			case 36:	// Return file information
				switch (gb.GetCommandFraction())
				{
				case -1:
				case 0:		// get regular file information
# if HAS_SBC_INTERFACE
					if (reprap.UsingSbcInterface())
					{
						reprap.GetFileInfoResponse(nullptr, outBuf, true);
					}
					else
# endif
					{
# if HAS_MASS_STORAGE || HAS_EMBEDDED_FILES
						if (!LockFileSystem(gb))									// getting file info takes several calls and isn't reentrant
						{
							return false;
						}

						String<MaxFilenameLength> filename;
						gb.GetUnprecedentedString(filename.GetRef(), true);
						result = reprap.GetFileInfoResponse((filename.IsEmpty()) ? nullptr : filename.c_str(), outBuf, false);
# endif
					}
					break;

#if HAS_MASS_STORAGE
				case 1:		// get thumbnail
					{
						String<MaxFilenameLength> filename;
						gb.MustSee('P');
						gb.GetQuotedString(filename.GetRef(), false);
						gb.MustSee('S');
						const FilePosition offset = gb.GetUIValue();
						outBuf = reprap.GetThumbnailResponse(filename.c_str(), offset, true);
						if (outBuf == nullptr)
						{
							return false;											// cannot allocate an output buffer, try again later
						}
					}
					break;
#endif
				default:
					result = GCodeResult::errorNotSupported;
					break;
				}
				break;
#endif

			case 37:	// Simulation mode on/off, or simulate a whole file
			{
#if SUPPORT_ASYNC_MOVES
				if (!DoSync(gb))
				{
					return false;
				}
#endif
				bool seen = false;
#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE || HAS_EMBEDDED_FILES
				String<MaxFilenameLength> simFileName;
				gb.TryGetPossiblyQuotedString('P', simFileName.GetRef(), seen);
				if (seen)
				{
					const bool updateFile = !gb.Seen('F') || gb.GetUIValue() == 1;
					result = SimulateFile(gb, reply, simFileName.GetRef(), updateFile);
				}
				else
#endif
				{
					uint32_t newSimulationMode;
					gb.TryGetLimitedUIValue('S', newSimulationMode, seen, (uint32_t)SimulationMode::highest + 1);
					if (seen)
					{
						result = ChangeSimulationMode(gb, reply, (SimulationMode)newSimulationMode);
					}
					else
					{
						reply.printf("Simulation mode: %s, move time: %.1f sec, other time: %.1f sec",
								(IsSimulating()) ? "on" : "off", (double)reprap.GetMove().GetSimulationTime(), (double)simulationTime);
					}
				}
				break;
			}

#if 0		// was (HAS_MASS_STORAGE || HAS_EMBEDDED_FILES), removed this function to save space on Duet 2
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
#endif

#if HAS_MASS_STORAGE
			case 39:	// Return SD card info
				{
					uint32_t slot = 0;
					bool dummy;
					gb.TryGetUIValue('P', slot, dummy);
					int32_t format = 0;
					gb.TryGetIValue('S', format, dummy);
					MassStorage::SdCardReturnedInfo returnedInfo;
					const MassStorage::InfoResult res = MassStorage::GetCardInfo(slot, returnedInfo);
					if (format == 2)
					{
						reply.printf("{\"SDinfo\":{\"slot\":%" PRIu32 ",\"present\":", slot);
						if (res == MassStorage::InfoResult::ok)
						{
							reply.catf("1,\"capacity\":%" PRIu64 ",\"partitionSize\":%" PRIu64 ",\"free\":%" PRIu64 ",\"speed\":%" PRIu32 ",\"clsize\":%" PRIu32 "}}",
								returnedInfo.cardCapacity, returnedInfo.partitionSize, returnedInfo.freeSpace, returnedInfo.speed, returnedInfo.clSize);
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
							reply.printf("SD card in slot %" PRIu32 ": capacity %.2fGB, partition size %.2fGB, free space %.2fGB, speed %.2fMBytes/sec, cluster size %" PRIu32 "%s",
											slot,
											(double)((float)returnedInfo.cardCapacity * 1e-9),
											(double)((float)returnedInfo.partitionSize * 1e-9),
											(double)((float)returnedInfo.freeSpace * 1e-9),
											(double)((float)returnedInfo.speed * 1e-6),
											(returnedInfo.clSize < 1024) ? returnedInfo.clSize : returnedInfo.clSize/1024,
											(returnedInfo.clSize < 1024) ? " bytes" : "kB"
										);
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

			case 73:	// Slicer-inserted print time values
				result = reprap.GetPrintMonitor().ProcessM73(gb, reply);
				break;

			case 80:	// ATX power on
				result = platform.HandleM80(gb, reply);
				break;

			case 81:	// ATX power off
				if (!LockAllMovementSystemsAndWaitForStandstill(gb))
				{
					return false;
				}
				result = platform.HandleM81(gb, reply);
				break;

			case 82:	// Use absolute extruder positioning
				gb.LatestMachineState().drivesRelative = false;
				reprap.InputsUpdated();
				break;

			case 83:	// Use relative extruder positioning
				gb.LatestMachineState().drivesRelative = true;
				reprap.InputsUpdated();
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
							if (!LockCurrentMovementSystemAndWaitForStandstill(gb))
							{
								return false;
							}
							platform.SetDriveStepsPerUnit(axis, gb.GetPositiveFValue(), ustepMultiplier);
#if SUPPORT_CAN_EXPANSION
							axesToUpdate.SetBit(axis);
#endif
							seen = true;
						}
					}

					if (seen)
					{
						platform.UpdateBacklashSteps();
					}

					if (gb.Seen(extrudeLetter))
					{
						if (!LockCurrentMovementSystemAndWaitForStandstill(gb))
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

#if SUPPORT_ASYNC_MOVES
					if (!gb.Executing())
					{
						break;
					}
#endif
					if (seen)
					{
						// On a delta, if we change the drive steps/mm then we need to recalculate the motor positions
						for (size_t i = 0; i < NumMovementSystems; ++i)
						{
							reprap.GetMove().SetNewPosition(moveStates[i].coords, i, true);
						}
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
				if (gb.Seen('P'))
				{
					String<MaxFilenameLength> filename;
					gb.GetQuotedString(filename.GetRef());
					DoFileMacroWithParameters(gb, filename.c_str(), true, code);
				}
				else if (gb.Seen('R'))
				{
					const bool restartable = (gb.GetUIValue() == 1);
					gb.LatestMachineState().SetMacroRestartable(restartable);
					reprap.InputsUpdated();
					if (restartable)
					{
						CheckForDeferredPause(gb);
					}
				}
				else
				{
					reply.copy("No P or R parameter");
					result = GCodeResult::error;
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
				GenerateTemperatureReport(gb, reply);
				gb.RespondedToStatusRequest(StatusReportType::m105);
				gb.ResetReportDueTimer();
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
							// If this is a print cooling fan for an active tool, set the virtual fan speed in the corresponding MovementState
							for (MovementState& ms : moveStates)
							{
								if (ms.currentTool != nullptr && ms.currentTool->GetFanMapping().IsBitSet(fanNum))
								{
									ms.virtualFanSpeed = f;
								}
							}
						}
						else
						{
							// We are processing an M106 S### command with no other recognised parameters and we have a tool selected.
							// Apply the fan speed setting to the fans in the fan mapping for the current tool.
							SetMappedFanSpeed(&gb, f);
						}
					}

					// ConfigureFan doesn't process R parameters
					if (gb.Seen('R') && !seenFanNum)
					{
						const size_t restorePointNumber = gb.GetLimitedUIValue('R', NumVisibleRestorePoints);
						SetMappedFanSpeed(&gb, GetMovementState(gb).restorePoints[restorePointNumber].fanSpeed);
					}
				}
				break;

			case 107: // Fan off - deprecated
				SetMappedFanSpeed(&gb, 0.0);
				break;

			case 108: // Cancel waiting for temperature
				CancelWaitForTemperatures(false);
				break;

			case 109: // Deprecated in RRF, but widely generated by slicers
				{
					const bool movementWasLocked = gb.LatestMachineState().lockedResources.IsBitSet(
#if SUPPORT_ASYNC_MOVES
						MoveResourceBase + gb.GetQueueNumberToLock()
#else
						MoveResourceBase
#endif
						);
					if (!LockCurrentMovementSystemAndWaitForStandstill(gb))		// wait until movement has finished and deferred command queue has caught up to avoid out-of-order execution
					{
						return false;
					}

					// 2020-02-04 Don't unlock movement if it was already locked, e.g. because M109 was used in a macro
					if (!movementWasLocked)
					{
						UnlockMovement(gb);							// allow babystepping and pausing while heating
					}
				}

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
						gb.LatestMachineState().waitWhileCooling = true;
						temperature = gb.GetFValue();
					}
					else if (gb.Seen('S'))
					{
						gb.LatestMachineState().waitWhileCooling = false;
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
					MovementState& ms = GetMovementState(gb);
					ReadLockedPointer<Tool> const applicableTool = (seenT) ? Tool::GetLockedTool(toolNumber)	// if we were given a tool number, use that
																	: ms.GetLockedCurrentOrDefaultTool();		// else if we have a current tool, use that, else the default tool

					// Check that we have a tool
					if (applicableTool.IsNull())
					{
						reply.copy("Invalid tool number");
						result = GCodeResult::error;
						break;
					}

					// Set the heater temperatures for that tool. We set the standby temperatures as well as the active ones,
					// because any slicer that uses M104 or M109 doesn't understand that there are separate active and standby temperatures.
					if (!IsSimulating())
					{
						SetToolHeaters(gb, applicableTool.Ptr(), temperature);			// this may throw
					}

					if (code == 109 && ms.currentTool == nullptr)
					{
						// Switch to the tool
						if (!LockCurrentMovementSystemAndWaitForStandstill(gb))
						{
							return false;
						}

						ms.newToolNumber = applicableTool->Number();
						ms.toolChangeParam = (IsSimulating()) ? 0 : DefaultToolChangeParam;
						gb.SetState(GCodeState::m109ToolChange0);
						result = GCodeResult::ok;
					}
					else
					{
						if (applicableTool.Ptr() == ms.currentTool)
						{
							// Even though the tool is selected, we may have turned it off e.g. when upgrading the WiFi firmware or following a heater fault that has been cleared.
							// So make sure the tool heaters are on.
							ms.SelectTool(applicableTool->Number(), IsSimulating());
						}
						else if (!IsSimulating())
						{
							// If we already have an active tool and we are setting temperatures for a different tool, set that tool's heaters to standby in case it is off
							applicableTool->Standby();
						}

						if (code == 109 && !IsSimulating())
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
				{
					bool seen = false;
					if (gb.Seen('B'))
					{
						seen = true;
						if (!Platform::SetDebugBufferSize(gb.GetUIValue()))
						{
							// We don't bother with an error message here because this is a debugging function, but we do report that there has been an error
							result = GCodeResult::error;
							break;
						}
					}
					uint32_t flags = 0;
					Module module = Module::none;
					if (gb.Seen('S'))
					{
						flags = gb.GetUIValue();
						if (flags != 0)
						{
							flags = 0xFFFFFFFF;
						}
						seen = true;
					}
					else if (gb.Seen('D'))
					{
						flags = gb.GetUIValue();
						seen = true;
					}
					if (gb.Seen('P'))
					{
						module = static_cast<Module>(gb.GetLimitedUIValue('P', NumRealModules));
						seen = true;
					}
					if (seen)
					{
						if (module != Module::none)
						{
							reprap.SetDebug(module, flags);
							reprap.PrintDebug(gb.GetResponseMessageType());
							return true;
						}
						else if (flags != 0)
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
				}
				break;

			case 112: // Emergency stop - acted upon in Webserver, but also here in case it comes from USB etc.
				DoEmergencyStop();
				break;

			case 114:
				HandleM114(gb, reply);
				break;

			case 115: // Print firmware version or set hardware type
#if SUPPORT_CAN_EXPANSION
				if (gb.Seen('B'))
				{
					const uint32_t board = gb.GetUIValue();
					if (board != CanInterface::GetCanAddress())
					{
						result = CanInterface::GetRemoteFirmwareDetails(board, gb, reply);
						break;
					}
				}
#endif
				reply.printf("FIRMWARE_NAME: %s FIRMWARE_VERSION: %s ELECTRONICS: %s", FIRMWARE_NAME, VERSION, platform.GetElectronicsString());
#if defined(DUET_NG)
				{
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
				}
#elif defined(DUET3MINI) && !defined(FMDC_V03)
				{
					const char *const expansionString = platform.GetExpansionBoardName();
					if (expansionString != nullptr)
					{
						reply.catf(" + %s", expansionString);
					}
				}
#endif
#if defined(DUET3_ATE)
				reply.lcatf("ATE firmware version %s date %s %s", Duet3Ate::GetFirmwareVersionString(), Duet3Ate::GetFirmwareDateString(), Duet3Ate::GetFirmwareTimeString());
#else
				reply.catf(" FIRMWARE_DATE: %s%s", DATE, TIME_SUFFIX);
#endif
				break;

			case 116: // Wait for set temperatures
				if (!LockCurrentMovementSystemAndWaitForStandstill(gb))	// wait until movement has finished and deferred command queue has caught up to avoid out-of-order execution
				{
					return false;
				}
				UnlockMovement(gb);										// allow babystepping and pausing while heating

				if (!gb.IsCancelWaitRequested())
				{
					const float tolerance = (gb.Seen('S')) ? gb.GetPositiveFValue() : TemperatureCloseEnough;
					bool seen = false;

					if (gb.Seen('P'))
					{
						// Wait for the heaters associated with the specified tool to be ready
						if (!ToolHeatersAtSetTemperatures(Tool::GetLockedTool(gb.GetIValue()).Ptr(), true, tolerance, gb.IsFileChannel()))
						{
							return false;
						}
						seen = true;
					}

					if (gb.Seen('H'))
					{
						// Wait for specified heater(s) to be ready
						uint32_t heaters[MaxHeaters];
						size_t heaterCount = MaxHeaters;
						gb.GetUnsignedArray(heaters, heaterCount, false);

						for (size_t i = 0; i < heaterCount; i++)
						{
							if (!reprap.GetHeat().HeaterAtSetTemperature(heaters[i], true, tolerance, gb.IsFileChannel()))
							{
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
								if (heater >= 0 && !reprap.GetHeat().HeaterAtSetTemperature(heater, true, tolerance, gb.IsFileChannel()))
								{
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
									if (heater >= 0 && !reprap.GetHeat().HeaterAtSetTemperature(heater, true, tolerance, gb.IsFileChannel()))
									{
										return false;
									}
								}
							}
						}
						seen = true;
					}

					// Wait for the current tool and slow heaters to be ready
					if (!seen && (
							!ToolHeatersAtSetTemperatures(GetMovementState(gb).GetLockedCurrentTool().Ptr(), true, tolerance, gb.IsFileChannel()) ||
							!reprap.GetHeat().SlowHeatersAtSetTemperatures(tolerance, gb.IsFileChannel())
						))
					{
						return false;
					}
				}
				break;

			case 117:	// Display message
				{
					String<M117StringLength> msg;
					gb.GetUnprecedentedString(msg.GetRef(), true);
					reprap.SetMessage(msg.c_str());
				}
				break;

			case 118:	// Echo message on host
				{
					gb.MustSee('S');
					String<MaxGCodeLength> message;
					gb.GetQuotedString(message.GetRef());

					MessageType type = GenericMessage;
#if HAS_MASS_STORAGE
					bool seenP = false;
#endif
					if (gb.Seen('P'))
					{
#if HAS_MASS_STORAGE
						seenP = true;
#endif
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
						case 6:		// MQTT
							type = MqttMessage;
							break;
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
						const LogLevel logLevel = (LogLevel) gb.GetLimitedUIValue('L', LogLevel::off, LogLevel::NumValues);
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

#if SUPPORT_MQTT
					if ((type & MqttMessage) && (result != GCodeResult::error))
					{
						String<MaxGCodeLength> topic;
						gb.MustSee('T');
						gb.GetQuotedString(topic.GetRef());

						bool seen = false;

						uint32_t qos = 0;
						gb.TryGetLimitedUIValue('Q', qos, seen, 3);

						bool retain = 0;
						gb.TryGetBValue('R', retain, seen);

						bool dup = 0;
						gb.TryGetBValue('D', dup, seen);

						reprap.GetNetwork().MqttPublish(message.c_str(), topic.c_str(), qos, retain, dup);
					}
#endif

					if (result != GCodeResult::error)
					{
						// Append newline and send the message to the destinations,
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
					const uint32_t board = (gb.Seen('B')) ? gb.GetUIValue() : CanInterface::GetCanAddress();
					if (board != CanInterface::GetCanAddress())
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
							if (temperature <= NEARLY_ABS_ZERO)
							{
								heat.SwitchOff(currentHeater);
							}
							else
							{
								heat.SetActiveTemperature(currentHeater, temperature);		// may throw
								result = heat.SetActiveOrStandby(currentHeater, nullptr, true, reply);
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
						const bool setActive = gb.Seen('S') && gb.GetIValue() == 1;
						result = reprap.GetHeat().SetActiveOrStandby(bedHeater, nullptr, setActive, reply);
					}
				}
				break;

#if SUPPORT_LED_STRIPS
			case 150:
				result = reprap.GetPlatform().GetLedStripManager().HandleM150(gb, reply);
				break;
#endif

			case 190: // Set bed temperature and wait
			case 191: // Set chamber temperature and wait
				if (!LockCurrentMovementSystemAndWaitForStandstill(gb))	// wait until movement has finished and deferred command queue has caught up to avoid out-of-order execution
				{
					return false;
				}
				UnlockMovement(gb);										// allow babystepping and pausing while heating

				if (!gb.IsCancelWaitRequested())
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
						result = reprap.GetHeat().SetActiveOrStandby(heater, nullptr, true, reply);
						if (!reprap.GetHeat().HeaterAtSetTemperature(heater, waitWhenCooling, TemperatureCloseEnough, gb.IsFileChannel()))
						{
							return false;
						}
					}
				}
				break;

			case 200: // Set filament diameter for volumetric extrusion and enable/disable volumetric extrusion
				{
					bool seen = gb.Seen('S');
					bool enable = !seen || gb.GetIValue() > 0;
					if (gb.Seen('D'))
					{
						seen = true;
						float diameters[MaxExtruders];
						size_t len = MaxExtruders;
						gb.GetFloatArray(diameters, len, true);
						for (size_t i = 0; i < len; ++i)
						{
							const float d = diameters[i];
							if (d <= 0.0)
							{
								volumetricExtrusionFactors[i] = 1.0;
							}
							else
							{
								filamentDiameters[i] = d;
								volumetricExtrusionFactors[i] = (enable) ? 4.0/(fsquare(d) * Pi) : 1.0;
							}
						}
						reprap.MoveUpdated();
					}

					if (seen)
					{
						gb.LatestMachineState().volumetricExtrusion = enable;
						reprap.InputsUpdated();
					}
					else
					{
						reply.printf("Volumetric extrusion is %sabled for this input, filament diameters:",
										(gb.LatestMachineState().volumetricExtrusion) ? "en" : "dis");
						for (size_t i = 0; i < numExtruders; ++i)
						{
							reply.catf(" %.03f", (double)filamentDiameters[i]);
						}
					}
				}
				break;

			case 201: // Set/print axis accelerations
				{
					const int frac = gb.GetCommandFraction();
					if (frac > 1)
					{
						result = GCodeResult::errorNotSupported;
						break;
					}

					bool seen = false;
					for (size_t axis = 0; axis < numTotalAxes; axis++)
					{
						if (gb.Seen(axisLetters[axis]))
						{
							platform.SetAcceleration(axis, gb.GetAcceleration(), frac == 1);
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
							platform.SetAcceleration(ExtruderToLogicalDrive(e), ConvertAcceleration(eVals[e]), frac == 1);
						}
					}

					if (seen)
					{
						reprap.MoveUpdated();
					}
					else
					{
						reply.copy((frac == 1) ? "Reduced accelerations (mm/sec^2): " : "Accelerations (mm/sec^2): ");
						for (size_t axis = 0; axis < numTotalAxes; ++axis)
						{
							reply.catf("%c: %.1f, ", axisLetters[axis], (double)InverseConvertAcceleration(platform.Acceleration(axis, frac == 1)));
						}
						reply.cat("E:");
						char sep = ' ';
						for (size_t extruder = 0; extruder < numExtruders; extruder++)
						{
							reply.catf("%c%.1f", sep, (double)InverseConvertAcceleration(platform.Acceleration(ExtruderToLogicalDrive(extruder), frac == 1)));
							sep = ':';
						}
					}
				}
				break;

			case 203: // Set/print minimum/maximum feedrates
				{
					// Units are mm/sec if S1 is given, else mm/min
					const bool usingMmPerSec = (gb.Seen('S') && gb.GetIValue() == 1);
					bool seen = false;

					// Do the minimum first, because we constrain the maximum rates to be no lower than it
					if (gb.Seen('I'))
					{
						seen = true;
						platform.SetMinMovementSpeed(gb.GetSpeedFromMm(usingMmPerSec));
					}

					for (size_t axis = 0; axis < numTotalAxes; ++axis)
					{
						if (gb.Seen(axisLetters[axis]))
						{
							seen = true;
							platform.SetMaxFeedrate(axis, gb.GetSpeedFromMm(usingMmPerSec));
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
							platform.SetMaxFeedrate(ExtruderToLogicalDrive(e), ConvertSpeedFromMm(eVals[e], usingMmPerSec));
						}
					}

					if (seen)
					{
						reprap.MoveUpdated();
					}
					else
					{
						reply.printf("Max speeds (%s): ", (usingMmPerSec) ? "mm/sec" : "mm/min");
						for (size_t axis = 0; axis < numTotalAxes; ++axis)
						{
							reply.catf("%c: %.1f, ", axisLetters[axis], (double)InverseConvertSpeedToMm(platform.MaxFeedrate(axis), usingMmPerSec));
						}
						reply.cat("E:");
						char sep = ' ';
						for (size_t extruder = 0; extruder < numExtruders; extruder++)
						{
							reply.catf("%c%.1f", sep, (double)InverseConvertSpeedToMm(platform.MaxFeedrate(ExtruderToLogicalDrive(extruder)), usingMmPerSec));
							sep = ':';
						}
						reply.catf(", min. speed %.2f", (double)InverseConvertSpeedToMm(platform.MinMovementSpeed(), usingMmPerSec));
					}
				}
				break;

			case 204: // Set max travel and printing accelerations
				result = ConfigureAccelerations(gb, reply);
				break;

			// For case 205 see case 566

			case 206: // Offset axes
				result = OffsetAxes(gb, reply);
				break;

			case 207: // Set firmware retraction details
				if (gb.Seen('P'))
				{
					const unsigned int toolNumber = gb.GetUIValue();
					auto tool = Tool::GetLockedTool(toolNumber);
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
					result = Tool::SetAllToolsFirmwareRetraction(gb, reply, outBuf);
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
									platform.SetAxisMinimum(axis, values[0], gb.LatestMachineState().runningM501);
									platform.SetAxisMaximum(axis, values[1], gb.LatestMachineState().runningM501);
								}
							}
							else if (setMin)
							{
								ok = platform.AxisMaximum(axis) > values[0];
								if (ok)
								{
									platform.SetAxisMinimum(axis, values[0], gb.LatestMachineState().runningM501);
								}
							}
							else
							{
								ok = values[0] > platform.AxisMinimum(axis);
								if (ok)
								{
									platform.SetAxisMaximum(axis, values[0], gb.LatestMachineState().runningM501);
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
						reply.copy("Axis limits (mm");
						char sep = ')';
						for (size_t axis = 0; axis < numTotalAxes; axis++)
						{
							reply.catf("%c %c%.1f:%.1f", sep, axisLetters[axis], (double)platform.AxisMinimum(axis), (double)platform.AxisMaximum(axis));
							sep = ',';
						}
					}
				}
				break;

			case 220:	// Set/report speed factor override percentage
				{
					MovementState& ms = GetMovementState(gb);
					if (gb.Seen('S'))
					{
						const float newSpeedFactor = gb.GetPositiveFValue() * 0.01;
						if (newSpeedFactor >= 0.01)
						{
							// If the last move hasn't gone yet, update its feed rate if it is not a firmware retraction
							if (ms.segmentsLeft != 0 && ms.applyM220M221)
							{
								ms.feedRate *= newSpeedFactor / ms.speedFactor;
							}
							ms.speedFactor = newSpeedFactor;
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
						reply.printf("Speed factor: %.1f%%", (double)(ms.speedFactor * 100.0));
					}
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

					const Tool * const ct = GetMovementState(gb).currentTool;
					if (!seenD && ct == nullptr)
					{
						reply.copy("No tool selected");
						result = GCodeResult::error;
					}
					else if (gb.Seen('S'))	// S parameter sets the override percentage
					{
						const float extrusionFactor = gb.GetPositiveFValue() * 0.01;
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

					GpOutputPort& port = platform.GetGpOutPort(gpioPortNumber);
					const float pwm = angleOrWidth * 1.0e-6 * port.GetPwmFrequency();
					result = port.WriteAnalog(gpioPortNumber, true, pwm, gb, reply);
				}
				break;

			case 290:	// Baby stepping
				{
					const bool absolute = (gb.Seen('R') && gb.GetIValue() == 0);
					float differences[MaxAxes];
					AxesBitmap axesMentioned;
					for (size_t axis = 0; axis < numVisibleAxes; ++axis)
					{
						if (gb.Seen(axisLetters[axis]) || (axis == 2 && gb.Seen('S')))			// S is a synonym for Z
						{
							if (!LockMovement(gb))
							{
								return false;
							}

							axesMentioned.SetBit(axis);
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

					if (axesMentioned.IsNonEmpty())
					{
						const bool canMove = !CheckEnoughAxesHomed(axesMentioned);
						if (!canMove && !absolute)
						{
							reply.copy("insufficient axes homed");
							result = GCodeResult::error;
							break;
						}

						// Perform babystepping synchronously with moves. Only move axes that have been flagged as homed.
						bool haveResidual = false;
						MovementState& ms = GetMovementState(gb);
						for (size_t axis = 0; axis < numVisibleAxes; ++axis)
						{
							currentBabyStepOffsets[axis] += differences[axis];
							reprap.MoveUpdated();
							if (IsAxisHomed(axis))
							{
								//TODO find which movement system owns the axis concerned and push the babystepping through that one
								const float amountPushed = reprap.GetMove().PushBabyStepping(0, axis, differences[axis]);
								ms.initialCoords[axis] += amountPushed;

								// The following causes all the remaining baby stepping that we didn't manage to push to be added to the [remainder of the] currently-executing move, if there is one.
								// This could result in an abrupt Z movement, however the move will be processed as normal so the jerk limit will be honoured.
								ms.coords[axis] += differences[axis];
								if (amountPushed != differences[axis])
								{
									haveResidual = true;
								}
							}
						}

						if (canMove && haveResidual && ms.segmentsLeft == 0 && reprap.GetMove().NoLiveMovement())
						{
							// The pipeline is empty, so execute the babystepping move immediately if it is safe to do
							SetMoveBufferDefaults(ms);
							ms.feedRate = ConvertSpeedFromMmPerMin(DefaultFeedRate);
							ms.linearAxesMentioned = axesMentioned.Intersects(reprap.GetPlatform().GetLinearAxes());
							ms.rotationalAxesMentioned = axesMentioned.Intersects(reprap.GetPlatform().GetRotationalAxes());
							NewSingleSegmentMoveAvailable(ms);
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
				result = DoMessageBox(gb, reply);
				break;

			case 292:	// Acknowledge message
				result = AcknowledgeMessage(gb, reply);
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
						if (!LockAllMovementSystemsAndWaitForStandstill(gb))
						{
							return false;
						}
						seen = true;
						reprap.GetHeat().AllowColdExtrude(gb.GetIValue() > 0);
					}
					if (gb.Seen('S'))
					{
						if (!LockAllMovementSystemsAndWaitForStandstill(gb))
						{
							return false;
						}
						seen = true;
						reprap.GetHeat().SetExtrusionMinTemp(gb.GetFValue());
					}
					if (gb.Seen('R'))
					{
						if (!LockAllMovementSystemsAndWaitForStandstill(gb))
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
				reply.copy("M305 has been replaced by M308 and M950");
				result = GCodeResult::error;
				break;

			case 307: // Set heater process model parameters
				result = reprap.GetHeat().SetOrReportHeaterModel(gb, reply);
				break;

			case 308:
				result = reprap.GetHeat().ConfigureSensor(gb, reply);
				break;

			case 309:
				{
					ReadLockedPointer<Tool> const tool = GetSpecifiedOrCurrentTool(gb);
					result = tool->GetSetFeedForward(gb, reply);
				}
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
							if (!LockAllMovementSystemsAndWaitForStandstill(gb))
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

					if (seen)
					{
						platform.UpdateBacklashSteps();
					}

					if (gb.Seen(extrudeLetter))
					{
						if (!LockAllMovementSystemsAndWaitForStandstill(gb))
						{
							return false;
						}
						int32_t eVals[MaxExtruders];
						size_t eCount = numExtruders;
						gb.GetIntArray(eVals, eCount, true);
						for (size_t e = 0; e < eCount; e++)
						{
							if (eVals[e] < 0)
							{
								// ignore negative microstepping values
								continue;
							}

							seen = true;
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

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE
			case 374: // Save grid and height map to file
				result = SaveHeightMap(gb, reply);
				break;

			case 375: // Load grid and height map from file and enable compensation
				if (!LockAllMovementSystemsAndWaitForStandstill(gb))
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
#if SUPPORT_ASYNC_MOVES
				result = ExecuteM400(gb, reply);
#else
				if (!LockCurrentMovementSystemAndWaitForStandstill(gb))
				{
					return false;
				}
#endif
				break;

			case 401: // Deploy Z probe
				{
					currentZProbeNumber = (gb.Seen('P')) ? gb.GetUIValue() : 0;
					auto zp = platform.GetEndstops().GetZProbe(currentZProbeNumber);
					if (zp.IsNotNull() && zp->GetProbeType() != ZProbeType::none)
					{
						zp->SetDeployedByUser(false);							// pretend that the probe isn't deployed, to make sure we deploy it unconditionally
						DeployZProbe(gb);
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
						RetractZProbe(gb);
					}
				}
				break;

			case 404: // Filament width. See also M200.
				// TODO support per-extruder values
				if (gb.Seen('N'))
				{
					platform.SetFilamentWidth(gb.GetPositiveFValue());
					break;
				}
				// no break
			case 407:
				reply.printf("Filament width %.2fmm", (double)platform.GetFilamentWidth());
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

					outBuf = GenerateJsonStatusResponse(type, seq, (&gb == AuxGCode()) ? ResponseSource::AUX : ResponseSource::Generic);
					if (outBuf == nullptr)
					{
						result = GCodeResult::notFinished;			// we ran out of buffers, so try again later
					}
					else if (type == 0)
					{
						gb.RespondedToStatusRequest(StatusReportType::m408);
						gb.ResetReportDueTimer();
					}
				}
				break;

#if SUPPORT_OBJECT_MODEL
			case 409: // Get object model values in JSON format
				{
					String<StringLength100> key;
					bool dummy;
					gb.TryGetQuotedString('K', key.GetRef(), dummy, true);
#if HAS_SBC_INTERFACE
					if (!gb.IsBinary() && (!gb.Seen('R') || gb.GetIValue() <= 0) &&
						reprap.UsingSbcInterface() && reprap.GetSbcInterface().IsConnected())
					{
						// In SBC mode. some keys are provided by DSF unless R1 is present
						const char *keyStart = (key[0] == '#') ? key.c_str() + 1 : key.c_str();
						if (StringStartsWith(keyStart, "network") || StringStartsWith(keyStart, "volumes"))
						{
							gb.SendToSbc();
							return false;
						}
					}
					else if (gb.IsBinary() && gb.Seen('I') && gb.GetIValue() > 0)
					{
						// Allow DSF to increment sequence numbers of its provided OM keys via K"key" I1
						if (key.Equals(("network")))
						{
							reprap.NetworkUpdated();
						}
						else if (key.Equals("volumes"))
						{
							reprap.VolumesUpdated();
						}
						break;
					}
#endif
					String<StringLength20> flags;
					gb.TryGetQuotedString('F', flags.GetRef(), dummy, true);
					{
						MutexLocker lock(reprap.GetObjectModelReportMutex());				// grab the mutex to prevent PanelDue retrieving the OM at the same time, which can result in running out of buffers
						if (OutputBuffer::GetFreeBuffers() < MinimumBuffersForObjectModel)
						{
							return false;													// try again later
						}
						outBuf = reprap.GetModelResponse(&gb, key.c_str(), flags.c_str());
						if (outBuf == nullptr)
						{
							// We don't delay and retry here, in case the user asked for too much of the object model in one go for the output buffers to contain it
							reply.copy("{\"err\":-1}\n");
						}
						else
						{
							gb.RespondedToStatusRequest(StatusReportType::m409);
							gb.ResetReportDueTimer();
						}
					}
				}
				break;
#endif

			case 425: // Backlash compensation
				result = platform.ConfigureBacklashCompensation(gb, reply);
				break;

			case 450: // Report printer mode
				reply.printf("PrinterMode:%s", GetMachineModeString());
				break;

			case 451: // Select FFF printer mode
				if (!LockAllMovementSystemsAndWaitForStandstill(gb))
				{
					return false;
				}
#if SUPPORT_LASER
				platform.ReleaseLaserPin();
#endif
				machineType = MachineType::fff;
				reprap.StateUpdated();
				break;

#if SUPPORT_LASER
			case 452: // Select laser mode
				if (!LockAllMovementSystemsAndWaitForStandstill(gb))
				{
					return false;
				}

				machineType = MachineType::laser;
				for (MovementState& ms : moveStates)
				{
					ms.ResetLaser();
				}

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
						laserMaxPower = max<float>(1.0, gb.GetNonNegativeFValue());
					}
				}

				reprap.StateUpdated();
				break;
#endif

			case 453: // Select CNC mode
				if (!LockAllMovementSystemsAndWaitForStandstill(gb))
				{
					return false;
				}

				// M453 may be repeated to set up multiple spindles, so only print the message on the initial switch
				if (machineType != MachineType::cnc)
				{
#if SUPPORT_LASER
					platform.ReleaseLaserPin();
#endif
					machineType = MachineType::cnc;						// switch to CNC mode even if the spindle parameter is bad
					reprap.StateUpdated();
				}

				if (gb.Seen('S'))
				{
					reply.copy("Spindle management has been moved to M950");
					result = GCodeResult::error;
				}
				break;

#if HAS_MASS_STORAGE
			case 470: // mkdir
				{
					gb.MustSee('P');
					String<MaxFilenameLength> dirName;
					gb.GetQuotedString(dirName.GetRef());
					if (MassStorage::MakeDirectory(dirName.c_str(), true))
					{
						result =  GCodeResult::ok;
					}
					else
					{
						reply.copy("failed to create folder");
						result = GCodeResult::error;
					}
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
					const bool deleteExisting = (gb.Seen('D') && gb.GetUIValue() == 1);
					if (MassStorage::Rename(oldVal.c_str(), newVal.c_str(), deleteExisting, true))
					{
						result = GCodeResult::ok ;
					}
					else
					{
						reply.copy("rename failed");
						result = GCodeResult::error;
					}
				}
				break;

			case 472: // delete file/directory
				{
					gb.MustSee('P');
					String<MaxFilenameLength> path;
					gb.GetQuotedString(path.GetRef());
					const bool recursive = (gb.Seen('R') && gb.GetUIValue() == 1);
					if (MassStorage::Delete(path.GetRef(), ErrorMessageMode::messageAlways, recursive))
					{
						result = GCodeResult::ok;
					}
					else
					{
						reply.copy("delete failed");
						result = GCodeResult::error;
					}
				}
				break;
#endif

			case 486: // number object or cancel object
				result = HandleM486(gb, reply, outBuf);
				break;

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE
			case 500: // Store parameters in config-override.g
				result = WriteConfigOverrideFile(gb, reply);
				break;
#endif

#if HAS_MASS_STORAGE || HAS_EMBEDDED_FILES || HAS_SBC_INTERFACE
			case 501: // Load parameters from config-override.g
				if (!gb.LatestMachineState().runningM502 && !gb.LatestMachineState().runningM501)		// when running M502 we ignore config-override.g
				{
					if (runningConfigFile)
					{
						m501SeenInConfigFile = true;
					}
					DoFileMacro(gb, CONFIG_OVERRIDE_G, true, code);
				}
				break;

			case 502: // Revert to default "factory settings" ignoring values in config-override.g
				if (!gb.LatestMachineState().runningM502)							// avoid recursion
				{
					if (!LockAllMovementSystemsAndWaitForStandstill(gb))
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
#endif

#if HAS_MASS_STORAGE || HAS_EMBEDDED_FILES
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
					if (!LockAllMovementSystemsAndWaitForStandstill(gb))
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

#if HAS_NETWORKING
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
#endif

			case 550: // Set/report machine name
#if HAS_SBC_INTERFACE
				if (reprap.UsingSbcInterface() && !gb.IsBinary())
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
					if (gb.Seen('P'))
					{
						String<RepRapPasswordLength> password;
						gb.GetPossiblyQuotedString(password.GetRef(), true);
						reprap.SetPassword(password.c_str());
					}
				}
				break;

#if HAS_NETWORKING
			case 552: // Enable/Disable network and/or Set/Get IP address
				if (CheckNetworkCommandAllowed(gb, reply, result))
				{
					Network& network = reprap.GetNetwork();
					bool seen = false;
					const unsigned int interface = (gb.Seen('I')) ? gb.GetLimitedUIValue('I', network.GetNumNetworkInterfaces()) : 0;
					String<SsidBufferLength> ssid;

					if (reprap.GetNetwork().IsWiFiInterface(interface))
					{
						if (gb.Seen('P'))
						{
							gb.GetQuotedString(ssid.GetRef());
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
					}

					// Process this one last in case the IP address is changed and the network enabled in the same command
					if (gb.Seen('S'))						// has the user turned the network interface on or off?
					{
						seen = true;
						result = network.EnableInterface(interface, gb.GetIValue(), ssid.GetRef(), reply);
					}

					if (!seen)
					{
						result = network.GetNetworkState(interface, reply);
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
#endif

			case 555: // Set/report firmware type to emulate
				if (gb.Seen('P'))
				{
					gb.LatestMachineState().compatibility.Assign(gb.GetIValue());
					reprap.InputsUpdated();
				}
				else
				{
					reply.printf("Output mode: %s", gb.LatestMachineState().compatibility.ToString());
				}
				break;

			case 556: // Axis compensation (we support only X, Y, Z)
			{
				bool seen = false;
				Move& move = reprap.GetMove();

				if (gb.Seen('S'))
				{
					const float value = gb.GetPositiveFValue();
					for (size_t axis = 0; axis <= Z_AXIS; axis++)
					{
						if (gb.Seen(axisLetters[axis]))
						{
							move.SetAxisCompensation(axis, gb.GetFValue() / value);
							seen = true;
						}
					}
				}

				if (gb.Seen('P'))
				{
					move.SetXYCompensation(gb.GetIValue() <= 0);
					seen = true;
				}

				if (!seen)
				{
					reply.printf("Axis compensations - %s: %.5f, YZ: %.5f, ZX: %.5f",
						move.IsXYCompensated() ? "XY" : "YX",
						(double)move.AxisCompensation(X_AXIS), (double)move.AxisCompensation(Y_AXIS), (double)move.AxisCompensation(Z_AXIS));
				}
				break;
			}

			case 557: // Set/report Z probe point coordinates
				result = DefineGrid(gb, reply);
				break;

			case 558: // Set or report Z probe type and for which axes it is used; M558.1 calibrate Z probe; M558.2 calibrate scanning Z probe drive strength
				result =
#if SUPPORT_SCANNING_PROBES
						(gb.GetCommandFraction() > 2) ? TryMacroFile(gb) :
#endif
							platform.GetEndstops().HandleM558(gb, reply);
				break;

#if HAS_MASS_STORAGE
			case 559:
			case 560: // Binary writing
				{
					String<MaxFilenameLength> defaultFolder;
					if (code == 560)
					{
						defaultFolder.copy(Platform::GetWebDir());
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
				if (!LockAllMovementSystemsAndWaitForStandstill(gb))
				{
					return false;
				}
				ClearBedMapping();
				break;

			case 562: // Reset temperature fault - use with great caution
				if (gb.Seen('P'))
				{
					const unsigned int heater = gb.GetLimitedUIValue('P', MaxHeaters);
					result = Tool::ClearTemperatureFault(heater, reply);
				}
				else
				{
					// Clear all heater faults
					for (unsigned int heater = 0; heater < MaxHeaters; ++heater)
					{
						result = max<GCodeResult>(result, Tool::ClearTemperatureFault(heater, reply));
					}
				}
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
					if (seen)
					{
						reprap.MoveUpdated();
					}
					else
					{
						reply.printf("Movement outside the bed is %spermitted, movement before homing is %spermitted", (limitAxes) ? "not " : "", (noMovesBeforeHoming) ? "not " : "");
					}
				}
				break;

			case 205: // Set/print maximum jerk speeds in mm/sec
			case 566: // Set/print maximum jerk speeds in mm/min
				{
					const bool useMmPerSec = (code == 205);
					bool seenAxis = false, seenExtruder = false;
					for (size_t axis = 0; axis < numTotalAxes; axis++)
					{
						if (gb.Seen(axisLetters[axis]))
						{
							platform.SetInstantDv(axis, gb.GetSpeedFromMm(useMmPerSec));
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
							platform.SetInstantDv(ExtruderToLogicalDrive(e), ConvertSpeedFromMm(eVals[e], useMmPerSec));
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
						reply.printf("Maximum jerk rates (%s): ", (useMmPerSec) ? "mm/sec" : "mm/min");
						for (size_t axis = 0; axis < numTotalAxes; ++axis)
						{
							reply.catf("%c: %.1f, ", axisLetters[axis], (double)InverseConvertSpeedToMm(platform.GetInstantDv(axis), useMmPerSec));
						}
						reply.cat("E:");
						char sep = ' ';
						for (size_t extruder = 0; extruder < numExtruders; extruder++)
						{
							reply.catf("%c%.1f", sep, (double)InverseConvertSpeedToMm(platform.GetInstantDv(ExtruderToLogicalDrive(extruder)), useMmPerSec));
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
				{
					ReadLockedPointer<Tool> const tool = GetSpecifiedOrCurrentTool(gb);
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
						reply.printf("Tool %d mix ratios:", tool->Number());
						char sep = ' ';
						for (size_t drive = 0; drive < tool->DriveCount(); drive++)
						{
							reply.catf("%c%.3f", sep, (double)tool->GetMix()[drive]);
							sep = ':';
						}
					}
				}
				break;

			case 568: // Tool Settings
				result = SetOrReportOffsets(gb, reply, 568);
				break;

			case 569: // Set/report axis direction
				result = ConfigureDriver(gb, reply);
				break;

			case 570: // Set/report heater monitoring
				gb.MustSee('H');
				result = reprap.GetHeat().ConfigureHeaterMonitoring(gb.GetUIValue(), gb, reply);
				break;

			case 571: // Set output on extrude
				result = platform.GetSetAncillaryPwm(gb, reply);
				break;

			case 572: // Set/report pressure advance
				result = reprap.GetMove().ConfigurePressureAdvance(gb, reply);
				break;

			// case 573 was report heater average PWM but is no longer supported because you can use "echo heat/heaters[N].avgPwm" instead

			case 574: // Set endstop configuration
				result = platform.GetEndstops().HandleM574(gb, reply, outBuf);				// this will lock movement if it is going to make any changes
				break;

			case 575: // Set communications parameters
				{
					const size_t chan = gb.GetLimitedUIValue('P', NumSerialChannels);
					GCodeBuffer * const gbp = (chan == 0) ? UsbGCode() : (chan == 1) ? AuxGCode() : Aux2GCode();
					bool seen = false;
					if (gb.Seen('B'))
					{
						platform.SetBaudRate(chan, gb.GetUIValue());
						seen = true;
					}

					if (gb.Seen('S'))
					{
						const uint32_t val = gb.GetUIValue();
						platform.SetCommsProperties(chan, val);
						if (gbp != nullptr)
						{
							gbp->SetCommsProperties(val);
#if HAS_AUX_DEVICES
							if (chan != 0)
							{
								const bool rawMode = (val & 2u) != 0;
								platform.SetAuxRaw(chan - 1, rawMode);
								if (rawMode && !platform.IsAuxEnabled(chan - 1))			// if enabling aux for the first time and in raw mode, set Marlin compatibility
								{
									gbp->LatestMachineState().compatibility = Compatibility::Marlin;
								}
							}
#endif
						}
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
						reply.printf("Channel %d: baud rate %" PRIu32 ", %s%s", chan, platform.GetBaudRate(chan),
										(chan != 0 && platform.IsAuxRaw(chan - 1)) ? "raw mode, " : "",
										(cp & 4) ? "requires CRC"
											: (cp & 1) ? "requires checksum or CRC"
												: "does not require checksum or CRC"
									);
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

#if HAS_SBC_INTERFACE
			case 576: // Set SPI communication parameters
				if (reprap.UsingSbcInterface())
				{
					result = reprap.GetSbcInterface().HandleM576(gb, reply);
				}
				else
				{
					reply.copy("Board is running in standalone mode");
					result = GCodeResult::error;
				}
				break;
#endif

			case 577: // Wait until endstop input is triggered
				result = WaitForPin(gb, reply);
				break;

			// M598 was Fire Inkjet bits

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

			// case 580 was (De)Select Roland mill

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

#if HAS_NETWORKING
			case 586: // Configure network protocols
				if (CheckNetworkCommandAllowed(gb, reply, result))
				{
					const unsigned int interface = (gb.Seen('I') ? gb.GetUIValue() : 0);
					switch (gb.GetCommandFraction())
					{
						case -1:
							{
								bool seen = false;
# if SUPPORT_HTTP
								if (gb.Seen('C'))
								{
									String<StringLength20> corsSite;
									gb.GetQuotedString(corsSite.GetRef(), true);
									reprap.GetNetwork().SetCorsSite(corsSite.c_str());
									seen = true;
								}
# endif

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

											IPAddress ip;

											if (protocol == MqttProtocol)
											{
												gb.MustSee('H');
												{
													gb.GetIPAddress(ip);
												}
											}

											result = reprap.GetNetwork().EnableProtocol(interface, protocol, port,
																						ip.GetV4LittleEndian(), secure, reply);
										}
										else
										{
											result = reprap.GetNetwork().DisableProtocol(interface, protocol, reply);
										}
										seen = true;
									}
								}

								if (!seen)
								{
# if SUPPORT_HTTP
									if (reprap.GetNetwork().GetCorsSite() != nullptr)
									{
										reply.printf("CORS enabled for site '%s'", reprap.GetNetwork().GetCorsSite());
									}
									else
									{
										reply.copy("CORS disabled");
									}
# endif
									// Default to reporting current protocols if P or S parameter missing
									result = reprap.GetNetwork().ReportProtocols(interface, reply);
								}
							}
							break;

						case MqttProtocol:
# if SUPPORT_MQTT
							result = MqttClient::Configure(gb, reply);
# else
							reply.copy("MQTT protocol not supported by this firmware");
							result = GCodeResult::error;
# endif
							break;

						default:
							break;
					}

				}
				break;
#endif

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
						const NonlinearExtrusion& nl = platform.GetExtrusionCoefficients(extruder);
						reply.printf("Drive %u nonlinear extrusion coefficients: A=%.3f, B=%.4f, limit=%.2f", extruder, (double)nl.A, (double)nl.B, (double)nl.limit);
					}
				}
				break;
#endif

			case 593: // Configure dynamic ringing cancellation
				result = reprap.GetMove().GetAxisShaper().Configure(gb, reply);
				break;

#if SUPPORT_ASYNC_MOVES
			case 594:	// Enter or leave height following mode
				result = reprap.GetMove().StartHeightFollowing(gb, reply);
				break;
#endif

			case 595:	// Configure movement queue size
				result = reprap.GetMove().ConfigureMovementQueue(gb, reply);
				break;

#if SUPPORT_ASYNC_MOVES
			case 596:	// Select movement queue
				result = SelectMovementQueue(gb, reply);
				break;

			case 597:	// Collision avoidance
				result = CollisionAvoidance(gb, reply);
				break;

			case 598:	// Sync
				result = SyncMovementSystems(gb, reply);
				break;
#endif

#if SUPPORT_KEEPOUT_ZONES
			case 599:	// Define keepout zone
				result = DefineKeepoutZone(gb, reply);
				break;
#endif

			// For cases 600 and 601, see 226

#if SUPPORT_ASYNC_MOVES
			case 606:	// fork input reader
				result = ForkInputReader(gb, reply);
				break;
#endif

			// M650 (set peel move parameters) and M651 (execute peel move) are no longer handled specially. Use macros to specify what they should do.

#if SUPPORT_LINEAR_DELTA
			case 665: // Set delta configuration
				if (!LockAllMovementSystemsAndWaitForStandstill(gb))
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
						for (MovementState& ms : moveStates)
						{
							move.GetKinematics().GetAssumedInitialPosition(numVisibleAxes, ms.coords);
							ToolOffsetInverseTransform(ms);
						}
					}
					if (changed || changedMode)
					{
						for (MovementState& ms : moveStates)
						{
							if (move.GetKinematics().LimitPosition(ms.coords, nullptr, numVisibleAxes, axesVirtuallyHomed, false, false) != LimitPositionResult::ok)
							{
								ToolOffsetInverseTransform(ms);					// make sure the limits are reflected in the user position
							}
							move.SetNewPosition(ms.coords, ms.GetMsNumber(), true);
						}
						SetAllAxesNotHomed();
						reprap.MoveUpdated();
					}
					result = GetGCodeResultFromError(error);
				}
				break;

			case 666: // Set delta endstop adjustments
				if (!LockAllMovementSystemsAndWaitForStandstill(gb))
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
#endif
			case 667: // Set CoreXY mode
				reply.copy("M667 is no longer supported - use M669 instead");
				result = GCodeResult::error;
				break;

			case 669:	// Set kinematics and parameters for non-delta kinematics
				if (!LockAllMovementSystemsAndWaitForStandstill(gb))
				{
					return false;
				}
				{
					Move& move = reprap.GetMove();
					const KinematicsType oldK = move.GetKinematics().GetKinematicsType();		// get the current kinematics type so we can tell whether it changed

					bool seen = false;
					if (gb.Seen('K'))
					{
						const unsigned int nk = gb.GetUIValue();
						if (nk >= (unsigned int)KinematicsType::unknown || !move.SetKinematics(static_cast<KinematicsType>(nk)))
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
						for (MovementState& ms : moveStates)
						{
							if (move.GetKinematics().GetKinematicsType() != oldK)
							{
								move.GetKinematics().GetAssumedInitialPosition(numVisibleAxes, ms.coords);
								ToolOffsetInverseTransform(ms);
							}
							if (move.GetKinematics().LimitPosition(ms.coords, nullptr, numVisibleAxes, axesVirtuallyHomed, false, false) != LimitPositionResult::ok)
							{
								ToolOffsetInverseTransform(ms);				// make sure the limits are reflected in the user position
							}
							move.SetNewPosition(ms.coords, ms.GetMsNumber(), true);
						}
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
				if (!LockAllMovementSystemsAndWaitForStandstill(gb))
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
				if (numTotalAxes <= U_AXIS)
				{
					reply.copy("Insufficient axes configured");
					result = GCodeResult::error;
				}
				else if (!LockAllMovementSystemsAndWaitForStandstill(gb))
				{
					result = GCodeResult::notFinished;
				}
				else if (!AllAxesAreHomed())
				{
					reply.copy("Home the axes first");
					result = GCodeResult::error;
				}
				else
				{
					Move& move = reprap.GetMove();
					if (move.GetNumProbedProbePoints() < 2)
					{
						reply.copy("Insufficient probe points");
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
						const float z1 = move.GetProbeCoordinates(0, x1, y1, true);
						const float z2 = move.GetProbeCoordinates(1, x2, y2, true);
						const float a1 = (x1 == x2) ? y1 : x1;
						const float a2 = (x1 == x2) ? y2 : x2;

						// See what kind of compensation we need to perform
						MovementState& ms = GetMovementState(gb);
						SetMoveBufferDefaults(ms);
						if (axisToUse != 0)
						{
							if (!reprap.GetPlatform().IsAxisRotational(axisToUse))
							{
								reply.printf("%c axis is not rotary", axisLetters[axisToUse]);
								result = GCodeResult::error;
								break;
							}

							// An axis letter is given, so try to level the given axis
							const float correctionAngle = atanf((z2 - z1) / (a2 - a1)) * 180.0 / M_PI;
							const float correctionFactor = gb.Seen('S') ? gb.GetPositiveFValue() : 1.0;
							ms.coords[axisToUse] += correctionAngle * correctionFactor;
							ms.rotationalAxesMentioned = true;

							reply.printf("%c axis is off by %.2f deg", axisLetters[axisToUse], (double)correctionAngle);
							HandleReply(gb, GCodeResult::notFinished, reply.c_str());
						}
						else if (move.GetNumProbedProbePoints() >= 4)
						{
							// At least four G30 points are given. This lets us figure out how far off the centre of the axis is
							const float z3 = move.GetProbeCoordinates(2, x1, y1, true);
							const float z4 = move.GetProbeCoordinates(3, x2, y2, true);
							const float a3 = (x1 == x2) ? y1 : x1;
							const float a4 = (x1 == x2) ? y2 : x2;

							// Calculate intersection points in [XY] and Z directions
							const float aS = ((a4 - a3) * (a2 * z1 - a1 * z2) - (a2 - a1) * (a4 * z3 - a3 * z4)) /
									((z4 - z3) * (a2 - a1) - (z2 - z1) * (a4 - a3));
							const float zS = ((z1 - z2) * (a4 * z3 - a3 * z4) - (z3 - z4) * (a2 * z1 - a1 * z2)) /
									((z4 - z3) * (a2 - a1) - (z2 - z1) * (a4 - a3));
							ms.coords[(x1 == x2) ? Y_AXIS : X_AXIS] += aS;
							ms.coords[Z_AXIS] += zS;
							ms.linearAxesMentioned = true;

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
							gb.LatestMachineState().feedRate = gb.GetSpeed();		// don't apply the speed factor
						}
						ms.feedRate = gb.LatestMachineState().feedRate;
						ms.usingStandardFeedrate = true;
						NewSingleSegmentMoveAvailable(ms);

						gb.SetState(GCodeState::waitingForSpecialMoveToComplete);
					}
				}
				break;

#if 0
			// This code is not finished yet
			case 674: // Set Z to center point
				if (LockMovementAndWaitForStandstill(gb))
				{
					Move& move = reprap.GetMove();
					if (move.GetNumProbedProbePoints() < 2)
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
						const float z1 = move.GetProbeCoordinates(0, x, y, true);
						const float z2 = move.GetProbeCoordinates(1, x, y, true);
						const float offset = gb.Seen('P') ? gb.GetFValue() : 0.0;
						currentUserPosition[Z_AXIS] -= (z1 + z2) / 2.0 + offset;

						ToolOffsetTransform(currentUserPosition, moveState.coords);
						if (move.GetKinematics().LimitPosition(moveState.coords, numVisibleAxes, LowestNBits<AxesBitmap>(numVisibleAxes), false))	// pretend that all axes are homed
						{
							ToolOffsetInverseTransform(moveState.coords, currentUserPosition);		// make sure the limits are reflected in the user position
						}
						move.SetNewPosition(moveState.coords, true);
						axesHomed |= move.GetKinematics().AxesAssumedHomed(MakeBitmap<AxesBitmap>(Z_AXIS));

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
				{
					const Tool * const currentTool = GetMovementState(gb).currentTool;
					if (currentTool != nullptr)
					{
						const Filament *filament = currentTool->GetFilament();
						if (filament != nullptr && filament->IsLoaded())
						{
							String<StringLength256> scratchString;
							scratchString.printf("%s%s/%s", FILAMENTS_DIRECTORY, filament->GetName(), CONFIG_FILAMENT_G);
							DoFileMacro(gb, scratchString.c_str(), false, SystemHelperMacroCode);
						}
					}
					else
					{
						result = GCodeResult::error;
						reply.copy("No tool selected");
					}
				}
				break;

#if 0
			case 750:	// was: Enable 3D scanner extension
			case 751:	// was: Register 3D scanner extension over USB
			case 752:	// was: Start 3D scan
			case 753:	// was: Cancel current 3D scanner action
			case 754:	// was: Calibrate scanner
			case 755:	// was: Set alignment mode for 3D scanner
			case 756:	// was: Shutdown 3D scanner
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
#if HAS_SMART_DRIVERS || SUPPORT_CAN_EXPANSION
			case 917: // Set/report standstill motor current percentage
#endif
				{
					bool seen = false;
					for (size_t axis = 0; axis < numTotalAxes; axis++)
					{
						if (gb.Seen(axisLetters[axis]))
						{
							if (!seen
#if HAS_VOLTAGE_MONITOR
									&& gb.GetState() != GCodeState::powerFailPausing1	// we don't wait for movement to stop if we are running the power fail script
#endif
							   )
							{
								if (!LockAllMovementSystemsAndWaitForStandstill(gb))
								{
									return false;
								}
							}
							seen = true;
							result = max(result, platform.SetMotorCurrent(axis, gb.GetNonNegativeFValue(), code, reply));	// note, we must allow zero in M913 power fail scripts
						}
					}

					if (gb.Seen(extrudeLetter))
					{
						if (!seen
#if HAS_VOLTAGE_MONITOR
								&& gb.GetState() != GCodeState::powerFailPausing1		// we don't wait for movement to stop if we are running the power fail script
#endif
						   )
						{
							if (!LockAllMovementSystemsAndWaitForStandstill(gb))
							{
								return false;
							}
						}
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
						platform.SetIdleCurrentFactor(gb.GetNonNegativeFValue()/100.0);
					}

					if (seen)
					{
						reprap.MoveUpdated();
					}
					else
					{
						reply.copy(	(code == 913) ? "Motor current % of normal - "
#if HAS_SMART_DRIVERS || SUPPORT_CAN_EXPANSION
									: (code == 917) ? "Motor standstill current % of normal - "
#endif
											: "Motor current (mA) - "
								);
						for (size_t axis = 0; axis < numTotalAxes; ++axis)
						{
							reply.catf("%c:%d, ", axisLetters[axis], platform.GetMotorCurrent(axis, code));
						}
						reply.cat("E");
						for (size_t extruder = 0; extruder < numExtruders; extruder++)
						{
							reply.catf(":%d", platform.GetMotorCurrent(ExtruderToLogicalDrive(extruder), code));
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
					const float saveVoltage = gb.GetPositiveFValue();
					if (saveVoltage < 10.0)
					{
						platform.DisableAutoSave();
					}
					else
					{
						float resumeVoltage = saveVoltage + 1.0;		// set up default resume voltage
						bool dummy;
						gb.TryGetFValue('R', resumeVoltage, dummy);

						String<StringLength256> powerFailString;
						bool seenCommandString = false;
						gb.TryGetQuotedString('P', powerFailString.GetRef(), seenCommandString);
						if (seenCommandString)
						{
							// Replace the power fail script atomically
							char *newPowerFailScript = new char[powerFailString.strlen() + 1];
							strcpy(newPowerFailScript, powerFailString.c_str());
							ReplaceObject(powerFailScript, newPowerFailScript);
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
			// case 914 was Alligator board specific

#if HAS_STALL_DETECT || SUPPORT_CAN_EXPANSION
			case 915:
				result = platform.ConfigureStallDetection(gb, reply, outBuf);
				break;
#endif

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE
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
					DoFileMacro(gb, RESUME_AFTER_POWER_FAIL_G, true, SystemHelperMacroCode);
				}
				break;
#endif

			// For case 917, see 906

#if SUPPORT_DIRECT_LCD
			case 918: // Configure direct-connect display
# ifdef DUET_NG
				// On Duet 2 configuring the display may affect the number of supported stepper drivers, so wait until there is no movement
				if (!LockAllMovementSystemsAndWaitForStandstill(gb))
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

#if SUPPORT_REMOTE_COMMANDS
			case 954:	// configure as expansion board
				{
					CanAddress addr = gb.GetLimitedUIValue('A', 1, CanId::MaxCanAddress + 1);
					CanInterface::SwitchToExpansionMode(addr, false);
				}
				break;
#endif

#if SUPPORT_ACCELEROMETERS
			case 955:	// configure accelerometer
				result = Accelerometers::ConfigureAccelerometer(gb, reply);
				break;

			case 956:	// start accelerometer
				result = Accelerometers::StartAccelerometer(gb, reply);
				break;
#endif

			case 957:	// raise event
				result = RaiseEvent(gb, reply);
				break;

#if HAS_WIFI_NETWORKING || HAS_AUX_DEVICES || HAS_MASS_STORAGE || HAS_SBC_INTERFACE
			case 997:	// Perform firmware update
				result = UpdateFirmware(gb, reply);
				break;
#endif

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
#ifdef DUET3_ATE
				Duet3Ate::PowerOffEUT();
#endif
#if SUPPORT_CAN_EXPANSION
				if (gb.Seen('B'))
				{
					const uint32_t address = gb.GetUIValue();
					if (address != CanInterface::GetCanAddress())
					{
						result = reprap.GetExpansion().ResetRemote(address, gb, reply);
						break;
					}
				}
#endif
#if SUPPORT_PANELDUE_FLASH
				if (gb.Seen('A'))
				{
					const uint32_t serialChannel = gb.GetLimitedUIValue('A', 1, NumSerialChannels);
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
#if HAS_SBC_INTERFACE
				// Send unknown non-binary codes to DSF so potential plugins can interpret them
				if (reprap.UsingSbcInterface() && reprap.GetSbcInterface().IsConnected() && !gb.IsBinary())
				{
					gb.SendToSbc();
					return false;
				}
#endif
				result = TryMacroFile(gb);
				break;
			}
		}

		return HandleResult(gb, result, reply, outBuf);
	}
	catch (...)
	{
		OutputBuffer::ReleaseAll(outBuf);
		throw;
	}
}

// Try to find a macro file that implements a G or M command
GCodeResult GCodes::TryMacroFile(GCodeBuffer& gb) THROWS(GCodeException)
{
	const int code = gb.GetCommandNumber();
	if (code >= 0 && code < 10000)
	{
		String<StringLength20> macroName;
		const int fraction = gb.GetCommandFraction();
		if (fraction > 0)
		{
			macroName.printf("%c%d.%d.g", gb.GetCommandLetter(), code, fraction);
		}
		else
		{
			macroName.printf("%c%d.g", gb.GetCommandLetter(), code);
		}
		if (DoFileMacroWithParameters(gb, macroName.c_str(), false, code))
		{
			return GCodeResult::ok;
		}
	}
	return GCodeResult::warningNotSupported;
}

bool GCodes::HandleTcode(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	if (stopped)
	{
		HandleResult(gb, GCodeResult::stopped, reply, nullptr);
		return true;
	}

	if (gb.LatestMachineState().runningM502)
	{
		return true;			// when running M502 we don't execute T commands
	}

#if SUPPORT_ASYNC_MOVES
	if (!gb.Executing())
	{
		// This is not the active MS, so ignore the command
		UnlockAll(gb);
		HandleReply(gb, GCodeResult::ok, "");
		return true;
	}
#endif

	bool seen = false;
	int toolNum;
	MovementState& ms = GetMovementState(gb);

	if (gb.HasCommandNumber())
	{
		seen = true;
		toolNum = gb.GetCommandNumber();
	}
	else if (gb.Seen('T'))
	{
		// We handle "T T{expression}" as if it's "T "{expression}, also DSF may pass a T{expression} command in this way
		seen = true;
		toolNum = gb.GetIValue();
	}
	else if (gb.Seen('R'))
	{
		const unsigned int rpNumber = gb.GetLimitedUIValue('R', NumVisibleRestorePoints);
		seen = true;
		toolNum = ms.restorePoints[rpNumber].toolNumber;
	}

	if (seen)
	{
		if (!LockCurrentMovementSystemAndWaitForStandstill(gb))
		{
			return false;
		}

#if SUPPORT_ASYNC_MOVES
		if (toolNum >= 0)
		{
			// Check that the other motion system isn't already using the requested tool or changing to it
			for (const MovementState& ms2 : moveStates)
			{
				if (&ms2 != &ms && (toolNum == ms2.GetCurrentToolNumber() || toolNum == ms2.newToolNumber))
				{
					UnlockAll(gb);
					reply.printf("Tool %d is already in use by another motion system", toolNum);
					HandleReply(gb, GCodeResult::error, reply.c_str());
					return true;
				}
			}
		}
#endif
		ms.newToolNumber = toolNum;							// claim the new tool to prevent the other movement system doing so
		if (ms.IsCurrentObjectCancelled())
		{
			// Don't do the tool change, just remember which one we are supposed to use in 'newToolNumber'
		}
		else if (ms.GetCurrentToolNumber() == toolNum)		// if old and new are the same we no longer follow the sequence. User can deselect and then reselect the tool if he wants the macros run.
		{
			// Even though the tool is selected, we may have turned it off e.g. when upgrading the WiFi firmware or following a heater fault that has been cleared. So make sure the tool heaters are on.
			ms.SelectTool(toolNum, IsSimulating());
		}
		else
		{
			StartToolChange(gb, ms, (gb.Seen('P')) ? gb.GetUIValue() : DefaultToolChangeParam);
			return true;									// proceeding with state machine, so don't unlock or send a reply
		}
	}
	else
	{
		// Report the tool number in use if no parameter is passed
		const int toolNum = ms.GetCurrentToolNumber();
		if (toolNum < 0)
		{
			reply.copy("No tool is selected");
		}
		else
		{
			reply.printf("Tool %d is selected", toolNum);
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
#if SUPPORT_ASYNC_MOVES
	if (!gb.Executing())
	{
		return true;
	}
#endif

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
	gb.LatestMachineState().commandRepeated = (rslt == GCodeResult::notFinished);

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

	case GCodeResult::stopped:
		gb.PrintCommand(reply);
		reply.cat(": Machine is halted");
		rslt = GCodeResult::error;
		break;

#if SUPPORT_CAN_EXPANSION
	case GCodeResult::remoteInternalError:
		if (!gb.IsDoingLocalFile())
		{
			gb.PrintCommand(reply);
			reply.cat(": ");
		}
		reply.cat("CAN-connected board reported internal error");
		rslt = GCodeResult::error;
		break;

	case GCodeResult::noCanBuffer:
		reply.lcat(NoCanBufferMessage);
		break;

	case GCodeResult::canResponseTimeout:
		// Usually we have a more detailed message in 'reply' already, but if not then add a standard message
		if (reply.IsEmpty()) { reply.copy("CAN response timeout"); }
		break;
#endif

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

	if (gb.LatestMachineState().GetState() == GCodeState::normal)
	{
		gb.StopTimer();
		UnlockAll(gb);
		HandleReply(gb, rslt, reply.c_str());
	}
	return true;
}

// End
