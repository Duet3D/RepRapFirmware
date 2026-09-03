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

#include "GCodeBuffer/GCodeBuffer.h"
#include "GCodeBuffer/ExpressionParser.h"
#include "GCodeQueue.h"
#include <Devices.h>
#include <Heating/Heat.h>
#include <Platform/Platform.h>
#include <Movement/Move.h>
#include <Movement/MoveDebugFlags.h>
#include <PrintMonitor/PrintMonitor.h>
#include <Platform/RepRap.h>
#include <Platform/Tasks.h>
#include <Platform/Event.h>
#include <Tools/Tool.h>
#include <Endstops/ZProbe.h>
#include <ObjectModel/Variable.h>
#include <AsyncSerial.h>

#if HAS_SBC_INTERFACE
# include <SBC/SbcInterface.h>
#endif

#if SUPPORT_REMOTE_COMMANDS
# include <CAN/CanInterface.h>
#endif

constexpr const char *_ecv_array TargetUnreachableText = "target position outside machine limits";		// message used for both G0/1 and G2/3 moves

#if HAS_SBC_INTERFACE
constexpr unsigned int MaxSbcFileCodesPerSpin = 8;		// maximum number of buffered SBC codes to fetch and execute per Spin call
#endif

#if NUM_ASYNC_CHANNELS != 0
// Support for emergency stop from PanelDue
bool GCodes::emergencyStopCommanded = false;

void GCodes::CommandEmergencyStop(AsyncSerial *p) noexcept
{
	emergencyStopCommanded = true;
}
#endif

GCodes::GCodes(Platform& p) noexcept :
#if NUM_ASYNC_CHANNELS != 0 && ALLOW_ARBITRARY_PANELDUE_PORT
	serialChannelForPanelDueFlashing(1),
#endif
	platform(p),
	machineType(MachineType::fff), active(false), stopped(false),
#if HAS_VOLTAGE_MONITOR
	powerFailScript(nullptr),
#endif
	isFlashing(false),
#if SUPPORT_PANELDUE_FLASH
	isFlashingPanelDue(false),
#endif
	lastWarningMillis(0)
#if HAS_MASS_STORAGE
	, sdTimingFile(nullptr)
#endif
{
#if HAS_MASS_STORAGE || HAS_EMBEDDED_FILES
	FileGCodeInput * const fileInput = new FileGCodeInput();
#else
	FileGCodeInput * const fileInput = nullptr;
#endif
	gcodeSources[GCodeChannel::ToBaseType(GCodeChannel::File)] = new GCodeBuffer(GCodeChannel::File, nullptr, fileInput, GenericMessage);
	moveStates[0].codeQueue = new GCodeQueue();
	gcodeSources[GCodeChannel::ToBaseType(GCodeChannel::Queue)] = new GCodeBuffer(GCodeChannel::Queue, moveStates[0].codeQueue, fileInput, GenericMessage);

#if SUPPORT_ASYNC_MOVES
# if HAS_MASS_STORAGE || HAS_EMBEDDED_FILES
	FileGCodeInput * const file2Input = new FileGCodeInput();		// use a separate FileInput object so that both file input streams can run concurrently
# else
	FileGCodeInput * const file2Input = nullptr;
# endif
	gcodeSources[GCodeChannel::ToBaseType(GCodeChannel::File2)] = new GCodeBuffer(GCodeChannel::File2, nullptr, file2Input, GenericMessage);
	moveStates[1].codeQueue = new GCodeQueue();
	gcodeSources[GCodeChannel::ToBaseType(GCodeChannel::Queue2)] = new GCodeBuffer(GCodeChannel::Queue2, moveStates[1].codeQueue, fileInput, GenericMessage);
	gcodeSources[GCodeChannel::ToBaseType(GCodeChannel::Queue2)]->SetActiveQueueNumber(1);		// so that all commands read from this queue get executed on queue #1 instead of the default #0
#else
	gcodeSources[GCodeChannel::ToBaseType(GCodeChannel::File2)] = nullptr;
	gcodeSources[GCodeChannel::ToBaseType(GCodeChannel::Queue2)] = nullptr;
#endif
#if SUPPORT_HTTP || HAS_SBC_INTERFACE
	httpInput = new NetworkGCodeInput(HttpMessage);
	gcodeSources[GCodeChannel::ToBaseType(GCodeChannel::HTTP)] = new GCodeBuffer(GCodeChannel::HTTP, httpInput, fileInput, HttpMessage);
#else
	gcodeSources[GCodeChannel::ToBaseType(GCodeChannel::HTTP)] = nullptr;
#endif // SUPPORT_HTTP || HAS_SBC_INTERFACE
#if SUPPORT_TELNET || HAS_SBC_INTERFACE
	telnetInput = new NetworkGCodeInput(TelnetMessage);
	gcodeSources[GCodeChannel::ToBaseType(GCodeChannel::Telnet)] = new GCodeBuffer(GCodeChannel::Telnet, telnetInput, fileInput, TelnetMessage, Compatibility::Marlin);
#else
	gcodeSources[GCodeChannel::ToBaseType(GCodeChannel::Telnet)] = nullptr;
#endif // SUPPORT_TELNET || HAS_SBC_INTERFACE
#ifdef SERIAL_USB_DEVICE
# if SAME5x && !CORE_USES_TINYUSB
	// SAME5x USB driver already uses an efficient buffer for receiving data from USB.
	// Note: this path does not support out-of-band emergency command detection (M112)
	StreamGCodeInput * const usbInput = new StreamGCodeInput(SERIAL_USB_DEVICE);
# else
	// Old USB driver and tinyusb drivers are inefficient when read in single-character mode
	usbInput = new UsbGCodeInput(SERIAL_USB_DEVICE, UsbMessage);
# endif
	gcodeSources[GCodeChannel::ToBaseType(GCodeChannel::USB)] = new GCodeBuffer(GCodeChannel::USB, usbInput, fileInput, UsbMessage, Compatibility::Marlin);
#elif HAS_SBC_INTERFACE
	gcodeSources[GCodeChannel::ToBaseType(GCodeChannel::USB)] = new GCodeBuffer(GCodeChannel::USB, nullptr, fileInput, UsbMessage, Compatibility::Marlin);
#else
	gcodeSources[GCodeChannel::ToBaseType(GCodeChannel::USB)] = nullptr;
#endif

#ifdef SERIAL_USB2_DEVICE
	usb2Input = new UsbGCodeInput(SERIAL_USB2_DEVICE, Usb2Message);
	gcodeSources[GCodeChannel::ToBaseType(GCodeChannel::USB2)] = new GCodeBuffer(GCodeChannel::USB2, usb2Input, fileInput, Usb2Message, Compatibility::Marlin);
#else
	gcodeSources[GCodeChannel::ToBaseType(GCodeChannel::USB2)] = nullptr;
#endif

#if NUM_ASYNC_CHANNELS != 0
	StreamGCodeInput * const auxInput = new StreamGCodeInput(*reprap.GetPlatform().GetAsyncPort(0));
	gcodeSources[GCodeChannel::ToBaseType(GCodeChannel::Aux)] = new GCodeBuffer(GCodeChannel::Aux, auxInput, fileInput, AuxMessage);
#elif HAS_SBC_INTERFACE
	gcodeSources[GCodeChannel::ToBaseType(GCodeChannel::Aux)] = new GCodeBuffer(GCodeChannel::Aux, nullptr, fileInput, AuxMessage);
#else
	gcodeSources[GCodeChannel::ToBaseType(GCodeChannel::Aux)] = nullptr;
#endif

	gcodeSources[GCodeChannel::ToBaseType(GCodeChannel::Trigger)] = new GCodeBuffer(GCodeChannel::Trigger, nullptr, fileInput, GenericMessage);

#if SUPPORT_DIRECT_LCD || HAS_SBC_INTERFACE
	gcodeSources[GCodeChannel::ToBaseType(GCodeChannel::LCD)] = new GCodeBuffer(GCodeChannel::LCD, nullptr, fileInput, LcdMessage);
#else
	gcodeSources[GCodeChannel::ToBaseType(GCodeChannel::LCD)] = nullptr;
#endif

#if HAS_SBC_INTERFACE
	gcodeSources[GCodeChannel::ToBaseType(GCodeChannel::SBC)] = new GCodeBuffer(GCodeChannel::SBC, nullptr, fileInput, GenericMessage);
#else
	gcodeSources[GCodeChannel::ToBaseType(GCodeChannel::SBC)] = nullptr;
#endif
	gcodeSources[GCodeChannel::ToBaseType(GCodeChannel::Daemon)] = new GCodeBuffer(GCodeChannel::Daemon, nullptr, fileInput, GenericMessage);
#if NUM_ASYNC_CHANNELS > 1
	StreamGCodeInput * const aux2Input = new StreamGCodeInput(*reprap.GetPlatform().GetAsyncPort(1));
	gcodeSources[GCodeChannel::ToBaseType(GCodeChannel::Aux2)] = new GCodeBuffer(GCodeChannel::Aux2, aux2Input, fileInput, Aux2Message);
#elif HAS_SBC_INTERFACE
	gcodeSources[GCodeChannel::ToBaseType(GCodeChannel::Aux2)] = new GCodeBuffer(GCodeChannel::Aux2, nullptr, fileInput, Aux2Message);
#else
	gcodeSources[GCodeChannel::ToBaseType(GCodeChannel::Aux2)] = nullptr;
#endif
	gcodeSources[GCodeChannel::ToBaseType(GCodeChannel::Autopause)] = new GCodeBuffer(GCodeChannel::Autopause, nullptr, fileInput, GenericMessage);
}

void GCodes::Exit() noexcept
{
	active = false;
}

void GCodes::Init() noexcept
{
	numVisibleAxes = numTotalAxes = XYZ_AXES;			// must set this up before calling Reset()
	memset(axisLetters, 0, sizeof(axisLetters));
	axisLetters[0] = 'X';
	axisLetters[1] = 'Y';
	axisLetters[2] = 'Z';
	allAxisLetters = ParameterLettersToBitmap("XYZ");

	numExtruders = 0;

	Reset();

	rawExtruderTotal = 0.0;
	for (float& f : rawExtruderTotalByDrive)
	{
		f = 0.0;
	}

	runningConfigFile = daemonRunning = false;
	m501SeenInConfigFile = false;
	doingToolChange = false;
	active = true;
	limitAxes = limitAxesRelative = noMovesBeforeHoming = true;
	SetAllAxesNotHomed();

	laserMaxPower = DefaultMaxLaserPower;
	laserPowerSticky = false;
}

// This is called from Init and when doing an emergency stop
void GCodes::Reset() noexcept
{
	// Here we could reset the input sources as well, but this would mess up M122\nM999
	// because both codes are sent at once from the web interface. Hence we don't do this here.
	for (GCodeBuffer *_ecv_null gb : gcodeSources)
	{
		if (gb != nullptr)
		{
			gb->Reset();
		}
	}

	if (AuxGCode() != nullptr)
	{
		AuxGCode()->Enable(1);									// by default, we require a checksum or CRC on the first aux port
	}

	nextGcodeSource = 0;

#if HAS_MASS_STORAGE || HAS_EMBEDDED_FILES
	fileToPrint.Close();
#endif

	for (size_t i = 0; i < MaxExtruders; ++i)
	{
		extrusionFactors[i] = 1.0;
		filamentDiameters[i] = DefaultFilamentDiameter;
		volumetricExtrusionFactors[i] = 4.0/(fsquare(DefaultFilamentDiameter) * Pi);
	}

	for (size_t i = 0; i < MaxAxes; ++i)
	{
		axisScaleFactors[i] = 1.0;
		for (size_t j = 0; j < NumCoordinateSystems; ++j)
		{
			workplaceCoordinates[j][i] = 0.0;
		}
	}

	// Initialise each movement system, except for the initial positions
	for (MovementSystemNumber i = 0; i < NumMovementSystems; ++i)
	{
		MovementState& ms = moveStates[i];
		ms.Init(i);
	}

	SetInitialAxisAndDrivePositions();

	for (float& f : currentBabyStepOffsets)
	{
		f = 0.0;										// clear babystepping before calling ToolOffsetInverseTransform
	}

	for (TriggerItem& tr : triggers)
	{
		tr.Init();
	}
	triggersPending.Clear();

	simulationMode = SimulationMode::off;
	exitSimulationWhenFileComplete = updateFileWhenSimulationComplete = false;
	simulationTime = 0.0;
	lastDuration = 0;

	pauseState = PauseState::notPaused;
#if SUPPORT_ASYNC_MOVES
	numMotionSystemsUsed = 1;
	pausedMovementSystemNumber = 0;
#endif
#if HAS_VOLTAGE_MONITOR
	isPowerFailPaused = false;
#endif
	doingToolChange = false;
	doingManualBedProbe = false;
#if SUPPORT_PROBE_POINTS_FILE
	ClearProbePointsInvalid();
#endif
	deferredPauseCommandPending = nullptr;
	firmwareUpdateModuleMap.Clear();
	isFlashing = false;
#if SUPPORT_PANELDUE_FLASH
	isFlashingPanelDue = false;
#endif
	currentZProbeNumber = 0;

	buildObjects.Init();

	displayNoToolWarning = false;

	for (const GCodeBuffer *_ecv_null & gbp : resourceOwners)
	{
		gbp = nullptr;
	}
}

// Set the initial coordinates and motor positions
void GCodes::SetInitialAxisAndDrivePositions() noexcept
{
	// Determine the initial machine coordinates assumes for this kinematics
	float initialPosition[MaxAxesPlusExtruders];
	memsetf(initialPosition, 0.0, ARRAY_SIZE(initialPosition));
	reprap.GetMove().GetKinematics().GetAssumedInitialPosition(numVisibleAxes, initialPosition);
	(void)reprap.GetMove().GetKinematics().LimitPosition(initialPosition, nullptr, numVisibleAxes, AxesBitmap::MakeLowestNBits(numVisibleAxes), false, false);

	// Convert those coordinates to endpoints, store then in lastKnownEndpoints, and set the motor positions to those endpoints
	MovementState::SetInitialMotorPositions(initialPosition);

	// Copy those endpoints into each DDARing, copy the machine coordinates into each MS, and convert them to user coordinates
	for (MovementSystemNumber i = 0; i < NumMovementSystems; ++i)
	{
		MovementState& ms = moveStates[i];
		ms.SetInitialMachineCoordinates(initialPosition);
		ToolOffsetInverseTransform(ms);
	}
}

// Adjust an endpoint following a change to steps/mm
void GCodes::AdjustEndpoint(size_t drive, float ratio) const noexcept
{
	for (const MovementState& ms : moveStates)
	{
		ms.SaveOwnDriveCoordinates();
	}
	MovementState::AdjustEndpoint(drive, ratio);
	for (size_t i = 0; i < NumMovementSystems; ++i)
	{
		reprap.GetMove().ChangeSingleEndpointAfterHoming(i, drive, MovementState::GetLastKnownEndpoints()[drive]);
	}
}

// Set the GCodeBuffer (if any) associated with a serial channel
GCodeBuffer *_ecv_null GCodes::GetSerialGCodeBuffer(size_t serialPortNumber) const noexcept
{
	switch (serialPortNumber)
	{
	case 0:						return UsbGCode();
#ifdef SERIAL_USB2_DEVICE
	case 1:						return Usb2GCode();
#endif
	case FirstAuxChannel:		return AuxGCode();
	case FirstAuxChannel + 1:	return Aux2GCode();
	default:					return nullptr;
	}
}

// Return true if any channel other than the daemon is executing a file macro
bool GCodes::DoingFileMacro() const noexcept
{
	for (const GCodeBuffer *_ecv_null gbp : gcodeSources)
	{
		if (gbp != nullptr && gbp != DaemonGCode() && gbp->IsDoingFileMacro())
		{
			return true;
		}
	}
	return false;
}

// Return true if any channel is waiting for a message acknowledgement
bool GCodes::WaitingForAcknowledgement() const noexcept
{
	for (const GCodeBuffer *_ecv_null gbp : gcodeSources)
	{
		if (gbp != nullptr && gbp->LatestMachineState().waitingForAcknowledgement)
		{
			return true;
		}
	}
	return false;
}

FilePosition GCodes::GetPrintingFilePosition() const noexcept
{
#if HAS_SBC_INTERFACE || HAS_MASS_STORAGE || HAS_EMBEDDED_FILES
	return FileGCode()->GetPrintingFilePosition(false);
#else
	return 0;
#endif
}

// Start running the config file
bool GCodes::RunConfigFile(const char *_ecv_array fileName, bool isMainConfigFile) noexcept
{
	const bool ret = DoFileMacro(*TriggerGCode(), fileName, false, AsyncSystemMacroCode);
	if (ret && isMainConfigFile)
	{
		runningConfigFile = true;
	}
	return ret;
}

// Return true if the trigger G-code buffer is busy running config.g or a trigger file
bool GCodes::IsTriggerBusy() const noexcept
{
	return TriggerGCode()->IsDoingFile();
}

// Copy the feed rate etc. from the channel that was running config.g to the input channels
void GCodes::CheckFinishedRunningConfigFile(GCodeBuffer& gb) noexcept
{
	if (runningConfigFile && gb.GetChannel() == GCodeChannel::Trigger)
	{
		gb.LatestMachineState().GetPrevious()->CopyStateFrom(gb.LatestMachineState());	// so that M83 etc. in nested files don't get forgotten
		if (gb.LatestMachineState().GetPrevious()->GetPrevious() == nullptr)			// if we've finished running config.g rather than a macro it called
		{
			for (GCodeBuffer *_ecv_null gb2 : gcodeSources)
			{
				if (gb2 != nullptr && gb2 != &gb)
				{
					gb2->LatestMachineState().CopyStateFrom(gb.LatestMachineState());
				}
			}
			runningConfigFile = false;
		}
		reprap.InputsUpdated();
	}
}

void GCodes::Spin() noexcept
{
	if (!active)
	{
		return;
	}

#if NUM_ASYNC_CHANNELS != 0
	if (emergencyStopCommanded)
	{
		DoEmergencyStop();
		while (reprap.GetPlatform().GetAsyncPort(0)->read() >= 0) { }
# if NUM_ASYNC_CHANNELS > 1
		while (reprap.GetPlatform().GetAsyncPort(1)->read() >= 0) { }
# endif
		emergencyStopCommanded = false;
		return;
	}
#endif

#if defined(SERIAL_USB_DEVICE) && (!SAME5x || CORE_USES_TINYUSB)
	// Read from USB into the input buffer and check for out-of-band urgent commands (M112/M122/M108)
	usbInput->Spin(*UsbGCode());
# if defined(SERIAL_USB2_DEVICE)
	usb2Input->Spin(*Usb2GCode());
# endif
#endif

	CheckTriggers();

	// The autoPause buffer has priority, so spin that one first. It may have to wait for other buffers to release locks etc.
	(void)SpinGCodeBuffer(*AutoPauseGCode());

	// Use round-robin scheduling for the other input sources
	// Scan the GCode input channels until we find one that we can do some useful work with, or we have scanned them all.
	// The idea is that when a single GCode input channel is active, we do some useful work every time we come through this polling loop, not once every N times (N = number of input channels)
	const size_t originalNextGCodeSource = nextGcodeSource;
	do
	{
		GCodeBuffer *_ecv_null const gbp = gcodeSources[nextGcodeSource];
		++nextGcodeSource;													// move on to the next gcode source ready for next time
		if (nextGcodeSource == GCodeChannel::Autopause)
		{
			++nextGcodeSource;												// don't do autoPause again
		}
		if (nextGcodeSource == ARRAY_SIZE(gcodeSources))
		{
			nextGcodeSource = 0;
		}
		if (gbp != nullptr && (gbp != AuxGCode() || !IsFlashingPanelDue()))	// skip auxGCode while flashing PanelDue is in progress
		{
			if (SpinGCodeBuffer(*gbp))										// if we did something useful
			{
				break;
			}
		}
	} while (nextGcodeSource != originalNextGCodeSource);


#if HAS_SBC_INTERFACE
	// Need to check if the print has been stopped by the SBC
	if (reprap.UsingSbcInterface() && reprap.GetSbcInterface().IsPrintAborted())
	{
		StopPrint(nullptr, StopPrintReason::abort);
	}
#endif

	// Check if we have had a serious movement error
	const MovementError err = GetLastMovementError();
	if (err != MovementError::ok)
	{
		platform.MessageF(ErrorMessage, "Operation halted due to movement error: %s\n", GetMovementErrorText(err));
		StopPrint(nullptr, StopPrintReason::abort);
	}

	// Check if we need to display a warning
	const uint32_t now = millis();
	if (now - lastWarningMillis >= MinimumWarningInterval)
	{
		if (displayNoToolWarning)
		{
			platform.Message(ErrorMessage, "Attempting to extrude with no tool selected.\n");
			displayNoToolWarning = false;
			lastWarningMillis = now;
		}
	}
}


// Do some work on an input channel, returning true if we did something significant
bool GCodes::SpinGCodeBuffer(GCodeBuffer& gb) noexcept
{
	// Set up a buffer for the reply
	String<GCodeReplyLength> reply;
	bool result;

	MutexLocker gbLock(gb.mutex);
	if (gb.GetState() == GCodeState::normal)
	{
		if (gb.LatestMachineState().messageAcknowledged)
		{
			const bool shouldAbort = gb.LatestMachineState().messageShouldAbort;
			gb.PopState(true);											// this could fail if the current macro has already been aborted

			if (shouldAbort)
			{
				if (gb.LatestMachineState().GetPrevious() == nullptr)
				{
#if HAS_SBC_INTERFACE
					if (reprap.UsingSbcInterface())
					{
						gb.AbortFile(false);
					}
#endif
					StopPrint(nullptr, StopPrintReason::userCancelled);
				}
				else
				{
					FileMacroCyclesReturn(gb);
				}
			}
			result = shouldAbort;
		}
		else
		{
			result = StartNextGCode(gb, reply.GetRef());
		}
	}
	else
	{
		RunStateMachine(gb, reply.GetRef());                            // execute the state machine
		result = true;													// assume we did something useful (not necessarily true, e.g. could be waiting for movement to stop)
	}

	if ((gb.IsExecuting()
#if HAS_SBC_INTERFACE
			|| gb.IsSendRequested() || gb.IsExecutingOnSbc()
#endif
		) || (gb.IsWaitingForTemperatures())							// this is needed to get reports sent when the GB is waiting for temperatures to be reached
	   )
	{
		CheckReportDue(gb, reply.GetRef());
	}

	return result;
}

// Start a new gcode, or continue to execute one that has already been started. Return true if we found something significant to do.
bool GCodes::StartNextGCode(GCodeBuffer& gb, const StringRef& reply) noexcept
{
	// There are special rules for fileGCode because it needs to suspend when paused:
	// - if the pause state is paused or resuming, don't execute
	// - if the state is pausing then don't execute, unless we are executing a macro (because it could be the pause macro or filament change macro)
	// - if there is a deferred pause pending, don't execute once we have finished the current macro
	if (   gb.IsFileChannel()
		&& (   pauseState > PauseState::pausing				// paused or resuming
			|| (!gb.IsDoingFileMacro() && (deferredPauseCommandPending != nullptr || pauseState == PauseState::pausing))
		   )
	   )
	{
		// We are paused or pausing, so don't process any more gcodes from the file being printed.
		// There is a potential issue here if fileGCode holds any locks, so unlock everything.
		UnlockAll(gb);
	}
	else if (gb.IsReady() || gb.IsExecuting())
	{
		gb.SetFinished(ActOnCode(gb, reply));
		return true;
	}
	else if (gb.IsDoingFile())
	{
		return DoFilePrint(gb, reply);
	}
	else if (&gb == AutoPauseGCode() && !gb.LatestMachineState().waitingForAcknowledgement)
	{
		if (Event::StartProcessing())
		{
			ProcessEvent(gb);								// call out to separate function to avoid increasing stack usage of this function
		}
	}
	else if (&gb == DaemonGCode()
#if SUPPORT_REMOTE_COMMANDS
			 && !CanInterface::InExpansionMode()			// looking for the daemon.g file increases the loop time too much
#endif
			)
	{
		// Check whether the daemon really is idle and not executing daemon.g, not just waiting for a M291 acknowledgement or something else
		if (   !reprap.IsProcessingConfig()
			&& gb.LatestMachineState().GetPrevious() == nullptr
			&& !gb.LatestMachineState().DoingFile()
		   )
		{
			// Delay 1 or 10 seconds, then try to open and run daemon.g. No error if it is not found.
			if (gb.DoDwellTime((daemonRunning) ? 10000 : 1000))
			{
				daemonRunning = true;
				return DoFileMacro(gb, DAEMON_G, false, AsyncSystemMacroCode);
			}
		}
	}
	else
	{
		const bool gotCommand =
#if HAS_SBC_INTERFACE
			// Two SBC-mode guards on accepting input here:
			// - IsExecutingOnSbc: a code has been handed to DSF for processing (see SendToSbc), so don't
			//   feed the buffer another code until that one has been resolved
			// - IsDoingFile: while DSF is serving a file/macro on this channel, accepting a text-based
			//   code (e.g. from PanelDue) would clear lastCodeFromSbc on the file's stack level and
			//   misroute that file's code replies to the analog channel instead of back to DSF. Waiting
			//   for a message-box acknowledgement is the exception, as M292 must stay answerable from PanelDue
			!gb.IsExecutingOnSbc() &&
			(!reprap.UsingSbcInterface() || !gb.IsDoingFile() || gb.LatestMachineState().waitingForAcknowledgement) &&
#endif
			(gb.GetNormalInput() != nullptr) && gb.GetNormalInput()->FillBuffer(&gb);
		if (gotCommand)
		{
			gb.DecodeCommand();
			bool done;
			try
			{
				done = gb.CheckMetaCommand(reply);
			}
			catch (const GCodeException& e)
			{
				e.GetMessage(reply, &gb);
				HandleReplyPreserveResult(gb, GCodeResult::error, reply.c_str());
				gb.Init();
				return true;
			}

			if (done)
			{
				HandleReplyPreserveResult(gb, GCodeResult::ok, reply.c_str());
				return true;
			}
		}
#if HAS_SBC_INTERFACE
		else if (reprap.UsingSbcInterface())
		{
			return reprap.GetSbcInterface().FillBuffer(gb);
		}
#endif
	}
	return false;
}

// Try to continue with a print from file, returning true if we did anything significant
bool GCodes::DoFilePrint(GCodeBuffer& gb, const StringRef& reply) noexcept
{
#if HAS_SBC_INTERFACE
	if (reprap.UsingSbcInterface())
	{
		if (gb.IsFileFinished())
		{
			if (gb.IsFileChannel() && gb.LatestMachineState().GetPrevious() == nullptr)
			{
				// Finished printing SD card file.
				// We never get here if the file ends in M0 because CancelPrint gets called directly in that case.
				// Don't close the file until all moves have been completed, in case the print gets paused.
				// Also, this keeps the state as 'Printing' until the print really has finished.
# if SUPPORT_ASYNC_MOVES
				if (!DoSync(gb))												// wait until the other input stream has caught up
				{
					return false;
				}
# else
				if (!LockCurrentMovementSystemAndWaitForStandstill(gb))			// wait until movement has finished and deferred command queue has caught up
				{
					return false;
				}
# endif
				StopPrint(&gb, StopPrintReason::normalCompletion);
				return true;
			}

			if (!gb.IsMacroFileClosed())
			{
				// Finished a macro or finished processing config.g
				gb.MacroFileClosed();
				CheckFinishedRunningConfigFile(gb);

				// Pop the stack and notify the SBC that we have closed the file
				Pop(gb, false);
				gb.Init();
				gb.LatestMachineState().firstCommandAfterRestart = false;

				// Send a final code response
				if (gb.GetState() == GCodeState::normal)
				{
					UnlockAll(gb);
					if (!gb.LatestMachineState().lastCodeFromSbc || gb.LatestMachineState().macroStartedByCode)
					{
						HandleReply(gb, GCodeResult::ok, "");
					}
					CheckForDeferredPause(gb);
				}
				return true;
			}
			return false;
		}
		else
		{
			// Make sure we read from regular inputs as well when waiting for message acknowledgments while executing (binary) macros
			if (gb.LatestMachineState().waitingForAcknowledgement && gb.GetNormalInput() != nullptr)
			{
				if (gb.GetNormalInput()->FillBuffer(&gb))
				{
					gb.DecodeCommand();
					return true;
				}
			}

			// Executing fetched codes immediately and amortising the main loop iteration cost over several codes speeds up simulated prints considerably.
			// In regular prints unfinished move codes end the batch early, so the DDA ring keeps its usual fill level
			bool didWork = false;
			for (size_t codesDone = 0; codesDone < MaxSbcFileCodesPerSpin; codesDone++)
			{
				// Read the next buffered code from the SBC interface
				if (!reprap.GetSbcInterface().FillBuffer(gb))
				{
					break;
				}
				didWork = true;

				// Run the code and stop if we cannot do any more
				reply.Clear();
				const bool finished = ActOnCode(gb, reply);
				gb.SetFinished(finished);
				if (!finished || gb.GetState() != GCodeState::normal ||
					(gb.IsFileChannel() && (pauseState != PauseState::notPaused || deferredPauseCommandPending != nullptr)))
				{
					break;
				}
			}
			return didWork;
		}
	}
	else
#endif
	{
#if HAS_MASS_STORAGE || HAS_EMBEDDED_FILES
		FileData& fd = gb.LatestMachineState().fileState;

		// Do we have more data to process?
		switch (gb.GetFileInput()->ReadFromFile(fd))
		{
		case GCodeInputReadResult::haveData:
			if (gb.GetFileInput()->FillBuffer(&gb))
			{
				bool done;
				try
				{
					done = gb.CheckMetaCommand(reply);
				}
				catch (const GCodeException& e)
				{
					e.GetMessage(reply, &gb);
					HandleReplyPreserveResult(gb, GCodeResult::error, reply.c_str());
					gb.Init();
					AbortPrint(gb);
					return true;
				}

				if (done)
				{
					HandleReplyPreserveResult(gb, GCodeResult::ok, reply.c_str());
				}
				else
				{
					gb.DecodeCommand();
					if (gb.IsReady())
					{
						gb.SetFinished(ActOnCode(gb, reply));
					}
				}
			}
			return true;

		case GCodeInputReadResult::error:
		default:
			AbortPrint(gb);
			return true;

		case GCodeInputReadResult::noData:
			// We have reached the end of the file. Check for the last line of gcode not ending in newline.
			if (gb.FileEnded())										// append a newline if necessary and deal with any pending file write
			{
				// If we get here then the file didn't end in newline and there is a command waiting to be processed
				bool done;
				try
				{
					done = gb.CheckMetaCommand(reply);
				}
				catch (const GCodeException& e)
				{
					e.GetMessage(reply, &gb);
					HandleReply(gb, GCodeResult::error, reply.c_str());
					gb.Init();
					AbortPrint(gb);
					return true;
				}

				if (done)
				{
					// Send the reply to the meta command
					HandleReply(gb, GCodeResult::ok, reply.c_str());
				}
				else
				{
					// There wasn't a meta command so parse it as a regular command
					gb.DecodeCommand();
					if (gb.IsReady())
					{
						gb.SetFinished(ActOnCode(gb, reply));
					}
				}
				return true;
			}

			if (gb.IsFileChannel() && gb.LatestMachineState().GetPrevious() == nullptr)
			{
				// Finished printing SD card file.
				// We never get here if the file ends in M0 because CancelPrint gets called directly in that case.
				// Don't close the file until all moves have been completed, in case the print gets paused.
				// Also, this keeps the state as 'Printing' until the print really has finished.
# if SUPPORT_ASYNC_MOVES
				if (!DoSync(gb))												// wait until the other input stream has caught up
				{
					return false;
				}
# else
				if (!LockCurrentMovementSystemAndWaitForStandstill(gb))			// wait until movement has finished and deferred command queue has caught up
				{
					return false;
				}
# endif
				StopPrint(&gb, StopPrintReason::normalCompletion);
			}
			else
			{
				// Finished a macro or finished processing config.g
				gb.GetFileInput()->Reset(fd);
				fd.Close();
				CheckFinishedRunningConfigFile(gb);
				Pop(gb, false);
				gb.Init();
				if (gb.GetState() == GCodeState::normal)
				{
					UnlockAll(gb);
					HandleReply(gb, GCodeResult::ok, "");
					CheckForDeferredPause(gb);
				}
			}
			return true;
		}
#endif
	}
	return false;
}

// Restore positions etc. when exiting simulation mode
void GCodes::EndSimulation(GCodeBuffer *null gb) noexcept
{
	// Ending a simulation, so restore the position
	MovementState::RestoreEndpointsAfterSimulating();					// restore the endpoints
	Move& move = reprap.GetMove();
	move.SetMotorPositions(RawMove::allLogicalDrives, MovementState::GetLastKnownEndpoints(), false);
	for (MovementState& ms : moveStates)
	{
		const RestorePoint& rp = ms.GetSimulationRestorePoint();
		RestorePosition(ms, rp);										// this restores the tool and the position, although we don't need to restore the position because we are going to overwrite it
		if (gb != nullptr && ms.GetNumber() != 0)
		{
			gb->LatestMachineState().feedRate = rp.originalFeedRate;	// restore the feed rate
		}
		ms.SelectTool(rp.toolNumber, true);								// set the restored tool number as selected

		move.MotorStepsToCartesian(MovementState::GetLastKnownEndpoints(), numVisibleAxes, numTotalAxes, ms.initialCoords);
		move.InverseAxisAndBedTransform(ms.initialCoords, ms.currentTool);
		ToolOffsetInverseTransform(ms);
	}
	axesVirtuallyHomed = axesHomed;
	reprap.MoveUpdated();
}

// Check for and execute triggers
void GCodes::CheckTriggers() noexcept
{
	for (unsigned int i = 0; i < MaxTriggers; ++i)
	{
		if (!triggersPending.IsBitSet(i) && triggers[i].Check(i))
		{
			triggersPending.SetBit(i);
		}
	}

	// If any triggers are pending, activate the one with the lowest number
	if (triggersPending.IsNonEmpty())
	{
		const unsigned int lowestTriggerPending = triggersPending.LowestSetBit();
		if (lowestTriggerPending == 0)
		{
			triggersPending.ClearBit(lowestTriggerPending);								// clear the trigger
			DoEmergencyStop();
		}
		else if (!IsTriggerBusy() && TriggerGCode()->GetState() == GCodeState::normal)	// if we are not already executing a trigger or config.g
		{
			if (lowestTriggerPending == 1)
			{
				if (!IsReallyPrinting())
				{
					triggersPending.ClearBit(lowestTriggerPending);						// ignore a pause trigger if we are already paused or not printing
				}
				else if (DoAsynchronousPause(*TriggerGCode(), PrintPausedReason::trigger, GCodeState::pausing1))
				{
					reprap.SendSimpleAlert(GenericMessage, "Print paused by external trigger", "Printing paused");
					triggersPending.ClearBit(lowestTriggerPending);						// clear the trigger
				}
			}
			else
			{
				triggersPending.ClearBit(lowestTriggerPending);							// clear the trigger
				String<StringLength20> filename;
				filename.printf("trigger%u.g", lowestTriggerPending);
				DoFileMacro(*TriggerGCode(), filename.c_str(), true, AsyncSystemMacroCode);
			}
		}
	}
}

// Execute an emergency stop
void GCodes::DoEmergencyStop() noexcept
{
	reprap.EmergencyStop();
	Reset();
	platform.Message(GenericMessage, "Emergency Stop! Reset the controller to continue.\n");
}

// reprap.EmergencyStop calls this to shut down this module
void GCodes::EmergencyStop() noexcept
{
	for (GCodeBuffer *_ecv_null gbp : gcodeSources)
	{
		if (gbp != nullptr)
		{
			AbortPrint(*gbp);
		}
	}
#if SUPPORT_LASER
	for (MovementState& ms : moveStates)
	{
		ms.laserPixelData.Clear();
	}
#endif
	stopped = true;
}

// Pause the print because of a command in the print file itself
// Return true if successful, false if we can't yet
bool GCodes::DoSynchronousPause(GCodeBuffer& gb, PrintPausedReason reason, GCodeState newState) noexcept
{
	// Pausing because of a command in the file itself
	if (!LockCurrentMovementSystemAndWaitForStandstill(gb))
	{
		return false;
	}

	MovementState& ms = GetMovementState(gb);
#if SUPPORT_ASYNC_MOVES
	if (gb.ExecutingAll())
	{
		pausedMovementSystemNumber = ms.GetNumber();
	}
#endif

	ms.pausedInMacro = false;
	ms.SavePosition(PauseRestorePointNumber, numVisibleAxes, gb.LatestMachineState().feedRate, gb.GetJobFilePosition());

#if SUPPORT_LASER
	if (machineType == MachineType::laser)
	{
		ms.laserPixelData.Clear();				// turn off the laser when we start moving
	}
#endif

	ms.GetPauseRestorePoint().toolNumber = ms.GetCurrentToolNumber();
	ms.GetPauseRestorePoint().fanSpeed = ms.virtualFanSpeed;

#if HAS_SBC_INTERFACE
	if (reprap.UsingSbcInterface())
	{
		// Prepare notification for the SBC
		// FIXME This should pass the file position of the first and second file, or noFilePosition if not applicable
		reprap.GetSbcInterface().SetPauseReason(ms.GetPauseRestorePoint().filePos, noFilePosition, reason);
	}
#endif

	if (ms.GetPauseRestorePoint().filePos == noFilePosition)
	{
		// Make sure we expose usable values (which noFilePosition is not)
		ms.GetPauseRestorePoint().filePos = 0;
	}

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE
	if (!IsSimulating())
	{
		//TODO need to sync here!
		SaveResumeInfo(false);															// create the resume file so that we can resume after power down
	}
#endif

	gb.SetState(newState);
	pauseState = PauseState::pausing;

	reprap.StateUpdated();																// test DWC/DSF that we have changed a restore point (copy in 'state')
	reprap.MotionSystemUpdated();														// tell DWC/DSF that a restore point (copy in 'move.motionSystems')
	return true;
}

// Pause the print because of an event or command not in the file being printed
// Return true if successful, false if we can't yet
// Before calling this, check that we are doing a file print that isn't already paused
// 'gb' is the GCode buffer that commanded the pause
bool GCodes::DoAsynchronousPause(GCodeBuffer& gb, PrintPausedReason reason, GCodeState newState) noexcept
{
	if (!LockAllMovement(gb))
	{
		return false;
	}

#if HAS_SBC_INTERFACE
	// Collect each motion system's pause file offset so the SBC gets one notification carrying both
	FilePosition pausePositions[NumMovementSystems];
	for (size_t i = 0; i < NumMovementSystems; ++i)
	{
		pausePositions[i] = noFilePosition;
	}
#endif

	for (MovementState& ms : moveStates)
	{
		ms.pausedInMacro = false;

		const bool movesSkipped = reprap.GetMove().PausePrint(ms);						// tell Move we wish to pause this queue
		GCodeBuffer& fgb = *GetFileGCode(ms.GetNumber());
		if (movesSkipped)
		{
			// The PausePrint call has filled in the restore point with machine coordinates
			ToolOffsetInverseTransform(ms, ms.GetPauseRestorePoint().moveCoords, ms.currentUserPosition);	// transform the returned coordinates to user coordinates
			ms.ClearMove();
		}
		else if (ms.segmentsLeft != 0)
		{
			// We were not able to skip any moves, however we can skip the move that is waiting
			ms.GetPauseRestorePoint().virtualExtruderPosition = ms.raw.moveStartVirtualExtruderPosition;
			ms.GetPauseRestorePoint().filePos = ms.raw.filePos;
			ms.GetPauseRestorePoint().gCommandNumber = ms.raw.gCommandNumber;
			ms.GetPauseRestorePoint().originalFeedRate = ms.raw.originalFeedRate;
			ms.GetPauseRestorePoint().proportionDone = ms.GetProportionDone();
			ms.GetPauseRestorePoint().initialUserC0 = ms.raw.initialUserC0;
			ms.GetPauseRestorePoint().initialUserC1 = ms.raw.initialUserC1;
			ToolOffsetInverseTransform(ms, ms.GetPauseRestorePoint().moveCoords, ms.currentUserPosition);	// transform the returned coordinates to user coordinates
			ms.ClearMove();
		}
		else
		{
			// We were not able to skip any moves, and there is no move waiting
			ms.GetPauseRestorePoint().originalFeedRate = fgb.LatestMachineState().feedRate;
			ms.GetPauseRestorePoint().virtualExtruderPosition = ms.latestVirtualExtruderPosition;
			ms.GetPauseRestorePoint().proportionDone = 0.0;

#if SUPPORT_ASYNC_MOVES
			if (fgb.ExecutingAll())
			{
				pausedMovementSystemNumber = fgb.GetActiveQueueNumber();									// this will used if we didn't skip any moves
			}
#endif
			// TODO: when using RTOS there is a possible race condition in the following,
			// because we might try to pause when a waiting move has just been added but before the gcode buffer has been re-initialised ready for the next command
			ms.GetPauseRestorePoint().filePos = fgb.GetPrintingFilePosition(true);
			ms.GetPauseRestorePoint().gCommandNumber = ms.raw.gCommandNumber;
			while (fgb.LatestMachineState().doingFileMacro)													// must call this after GetFilePosition because this changes IsDoingFileMacro
			{
				ms.pausedInMacro = true;
				fgb.PopState(false);
			}
#if SUPPORT_LASER || SUPPORT_IOBITS
			ms.GetPauseRestorePoint().laserPwmOrIoBits = ms.raw.laserPwmOrIoBits;
#endif
		}

		// The user position may no longer match where the machine actually stops if a queued or read-ahead move was discarded above,
		// so make sure it is re-read from the motors at the next standstill before the pause macro can run
		ms.positionMayBeInaccurate = true;

		// Replace the paused machine coordinates by user coordinates, which we updated earlier if they were returned by Move::PausePrint
		for (size_t axis = 0; axis < numVisibleAxes; ++axis)
		{
			ms.GetPauseRestorePoint().moveCoords[axis] = ms.currentUserPosition[axis];
		}

#if HAS_SBC_INTERFACE
		if (reprap.UsingSbcInterface())
		{
			fgb.Init();															// clear the next move
			UnlockAll(fgb);														// release any locks it had
		}
		else
#endif
		{
#if HAS_MASS_STORAGE
			// If we skipped any moves, reset the file pointer to the start of the first move we need to replay
			// The following could be delayed until we resume the print
			if (ms.GetPauseRestorePoint().filePos != noFilePosition)
			{
				FileData& fdata = fgb.LatestMachineState().fileState;
				if (fdata.IsLive())
				{
					fgb.RestartFrom(ms.GetPauseRestorePoint().filePos);			// TODO we ought to restore the line number too, but currently we don't save it
					// restore the modal G0/G1/G2/G3 context in case the file uses implied command letters
					fgb.SetModalGCommand(ms.GetPauseRestorePoint().gCommandNumber);
					UnlockAll(fgb);												// release any locks it had
				}
			}
#endif
		}

		ms.codeQueue->PurgeEntries();

		if (reprap.Debug(Module::Gcodes))
		{
			platform.MessageF(GenericMessage, "Paused print, file offset=%" PRIu32 "\n", ms.GetPauseRestorePoint().filePos);
		}

#if SUPPORT_LASER
		if (machineType == MachineType::laser)
		{
			ms.laserPixelData.Clear();				// turn off the laser when we start moving
		}
#endif

		ms.GetPauseRestorePoint().toolNumber = ms.GetCurrentToolNumber();
		ms.GetPauseRestorePoint().fanSpeed = ms.virtualFanSpeed;

#if HAS_SBC_INTERFACE
		// Capture the per-MS pause offset before the noFilePosition normalization below so the SBC sees
		// the real "unknown" marker (noFilePosition) rather than a fabricated zero
		pausePositions[ms.GetNumber()] = ms.GetPauseRestorePoint().filePos;
#endif

		if (ms.GetPauseRestorePoint().filePos == noFilePosition)
		{
			// Make sure we expose usable values (which noFilePosition is not)
			ms.GetPauseRestorePoint().filePos = 0;
		}
	}

#if HAS_SBC_INTERFACE
	if (reprap.UsingSbcInterface())
	{
		reprap.GetSbcInterface().SetPauseReason(pausePositions[0], (NumMovementSystems > 1) ? pausePositions[1] : noFilePosition, reason);
	}
#endif

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE
	if (!IsSimulating())
	{
		//TODO need to sync here!
		SaveResumeInfo(false);															// create the resume file so that we can resume after power down
	}
#endif

	gb.SetState(newState);
	pauseState = PauseState::pausing;
	CancelWaitForTemperatures(true);

	reprap.StateUpdated();																// tell DWC/DSF that we have changed the state and a restore point (copy in 'state')
	reprap.MotionSystemUpdated();														// tell DWC/DSF that a restore point (copy in 'move.motionSystems')
	return true;
}

// Check if a pause is pending, action it if so
void GCodes::CheckForDeferredPause(GCodeBuffer& gb) noexcept
{
	if (gb.IsFileChannel() && !gb.IsDoingFileMacro(true) && deferredPauseCommandPending != nullptr && !doingToolChange)
	{
		gb.PutAndDecode(deferredPauseCommandPending);
		deferredPauseCommandPending = nullptr;
	}
}

// Return true if we are printing from SD card and not pausing, paused or resuming
// Called by the filament monitor code to see whether we need to activate the filament monitors, and from other places
// TODO make this independent of PrintMonitor
bool GCodes::IsReallyPrinting() const noexcept
{
#if SUPPORT_REMOTE_COMMANDS
	if (CanInterface::InExpansionMode())
	{
		return isRemotePrinting;
	}
#endif

	return reprap.GetPrintMonitor().IsPrinting() && pauseState == PauseState::notPaused;
}

bool GCodes::IsReallyPrintingOrResuming() const noexcept
{
	return reprap.GetPrintMonitor().IsPrinting() && (pauseState == PauseState::notPaused || pauseState == PauseState::resuming);
}

// Check if the print is being cancelled. This may be the case because the print was paused
// and is now being cancelled (M0/M1/M2) or because or because the second file GB has
// not finished executing after the first file GB had encountered an error.
bool GCodes::IsCancellingPrint() const noexcept
{
	return (pauseState == PauseState::cancelling)
#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE
			|| (   !reprap.GetPrintMonitor().IsPrinting()
				&& pauseState == PauseState::notPaused
				&& (   FileGCode()->IsExecuting()
# if SUPPORT_ASYNC_MOVES
					|| File2GCode()->IsExecuting()
# endif
				   )
				)
#endif
			;
}

// Return true if the first file channel is waiting for a heater to reach temperature
bool GCodes::IsHeatingUp() const noexcept
{
	return FileGCode()->IsWaitingForTemperatures();
}

// Stop waiting for temperatures to be reached, optionally just in the file(s) being printed
void GCodes::CancelWaitForTemperatures(bool onlyInPrintFiles) noexcept
{
	for (GCodeBuffer *_ecv_null gb : gcodeSources)
	{
		if (gb != nullptr && gb->IsWaitingForTemperatures() && (!onlyInPrintFiles || (gb->IsFileChannel() && (!gb->IsDoingFileMacro() || gb->LatestMachineState().CanRestartMacro()))))
		{
			gb->CancelWaitForTemperatures();
		}
	}
}

#if HAS_VOLTAGE_MONITOR || HAS_STALL_DETECT

// Do an emergency pause following loss of power or a motor stall returning true if successful, false if needs to be retried
bool GCodes::DoEmergencyPause() noexcept
{
	if (!AutoPauseGCode()->IsCompletelyIdle())
	{
		return false;							// we can't pause if the auto pause thread is busy already
	}

	// Save the resume info, stop movement immediately and run the low voltage pause script to lift the nozzle etc.
	GrabMovement(*AutoPauseGCode());

	for (MovementState& ms : moveStates)
	{
		// When we use RTOS there is a possible race condition in the following, because we might try to pause when a waiting move has just been added
		// but before the gcode buffer has been re-initialised ready for the next command. So start a critical section.
		TaskCriticalSectionLocker lock;

		const bool movesSkipped = reprap.GetMove().LowPowerOrStallPause(ms);
		if (movesSkipped)
		{
			// The LowPowerOrStallPause call has filled in the restore point with machine coordinates
			ToolOffsetInverseTransform(ms, ms.GetPauseRestorePoint().moveCoords, ms.currentUserPosition);	// transform the returned coordinates to user coordinates
			ms.ClearMove();
		}
		else if (ms.segmentsLeft != 0 && ms.raw.filePos != noFilePosition)
		{
			// We were not able to skip any moves, however we can skip the remaining segments of this current move
			ToolOffsetInverseTransform(ms, ms.initialCoords, ms.currentUserPosition);
			ms.GetPauseRestorePoint().originalFeedRate = ms.raw.originalFeedRate;
			ms.GetPauseRestorePoint().virtualExtruderPosition = ms.raw.moveStartVirtualExtruderPosition;
			ms.GetPauseRestorePoint().filePos = ms.raw.filePos;
			ms.GetPauseRestorePoint().gCommandNumber = ms.raw.gCommandNumber;
			ms.GetPauseRestorePoint().proportionDone = ms.GetProportionDone();
			ms.GetPauseRestorePoint().initialUserC0 = ms.raw.initialUserC0;
			ms.GetPauseRestorePoint().initialUserC1 = ms.raw.initialUserC1;
#if SUPPORT_LASER || SUPPORT_IOBITS
			ms.GetPauseRestorePoint().laserPwmOrIoBits = ms.raw.laserPwmOrIoBits;
#endif
			ms.ClearMove();
		}
		else
		{
			// We were not able to skip any moves, and if there is a move waiting then we can't skip that one either
			ms.GetPauseRestorePoint().originalFeedRate = FileGCode()->LatestMachineState().feedRate;
			ms.GetPauseRestorePoint().virtualExtruderPosition = ms.latestVirtualExtruderPosition;

			ms.GetPauseRestorePoint().filePos = FileGCode()->GetPrintingFilePosition(true);	//TODO separate restore point per channel
			ms.GetPauseRestorePoint().gCommandNumber = ms.raw.gCommandNumber;
			ms.GetPauseRestorePoint().proportionDone = 0.0;

#if SUPPORT_LASER || SUPPORT_IOBITS
			ms.GetPauseRestorePoint().laserPwmOrIoBits = ms.raw.laserPwmOrIoBits;
#endif
		}

		// The aborted move may have been stopped partway through, so make sure the position is re-read from the motors at the next standstill
		ms.positionMayBeInaccurate = true;

#if HAS_SBC_INTERFACE
		if (reprap.UsingSbcInterface() && ms.GetNumber() == 0)
		{
			// FIXME This needs to report the file position of the second file as well (if applicable)
			PrintPausedReason reason = platform.IsPowerOk() ? PrintPausedReason::stall : PrintPausedReason::lowVoltage;
			reprap.GetSbcInterface().SetEmergencyPauseReason(ms.GetPauseRestorePoint().filePos, noFilePosition, reason);
			reprap.GetSbcInterface().EventOccurred(true);
		}
#endif

		ms.codeQueue->PurgeEntries();

		// Replace the paused machine coordinates by user coordinates, which we updated earlier
		for (size_t axis = 0; axis < numVisibleAxes; ++axis)
		{
			ms.GetPauseRestorePoint().moveCoords[axis] = ms.currentUserPosition[axis];
		}

		if (ms.GetPauseRestorePoint().filePos == noFilePosition)
		{
			// Make sure we expose usable values (which noFilePosition is not)
			ms.GetPauseRestorePoint().filePos = 0;
		}
		ms.GetPauseRestorePoint().toolNumber = ms.GetCurrentToolNumber();
		ms.GetPauseRestorePoint().fanSpeed = ms.virtualFanSpeed;
	}

	pauseState = PauseState::paused;

	return true;
}

#endif

#if HAS_VOLTAGE_MONITOR

// Try to pause the current SD card print, returning true if successful, false if needs to be called again
bool GCodes::LowVoltagePause() noexcept
{
	if (IsSimulating())
	{
		return true;								// ignore the low voltage indication
	}

	reprap.GetHeat().SuspendHeaters(true);			// turn the heaters off to conserve power for the motors to execute the pause
	switch (pauseState)
	{
	case PauseState::resuming:
		// This is an unlucky situation, because the resume macro is probably being run, which will probably lower the head back on to the print.
		// It may well be that the power loss will prevent the resume macro being completed. If not, try again when the print has been resumed.
		return false;

	case PauseState::pausing:
		// We are in the process of pausing already, so the resume info has already been saved.
		// With luck the retraction and lifting of the head in the pause.g file has been done already.
		return true;

	case PauseState::paused:
		// Resume info has already been saved, and resuming will be prevented while the power is low
		return true;

	default:
		break;
	}

	if (reprap.GetPrintMonitor().IsPrinting())
	{
		if (!DoEmergencyPause())
		{
			return false;
		}

		// Run the auto-pause script
		if (powerFailScript != nullptr)
		{
			AutoPauseGCode()->PutAndDecode(powerFailScript);
		}
		AutoPauseGCode()->SetState(GCodeState::powerFailPausing1);
		isPowerFailPaused = true;

		// Don't do any more here, we want the auto pause thread to run as soon as possible
	}

	return true;
}

// Resume printing, normally only ever called after it has been paused because if low voltage.
// If the pause was short enough, resume automatically.
bool GCodes::LowVoltageResume() noexcept
{
	reprap.GetHeat().SuspendHeaters(false);			// turn the heaters on again
	if (pauseState != PauseState::notPaused && isPowerFailPaused)
	{
		isPowerFailPaused = false;					// pretend it's a normal pause
		// Run resurrect.g automatically
		//TODO qq;
		//platform.Message(LoggedGenericMessage, "Print auto-resumed\n");
	}
	return true;
}

#endif

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE

// Write the resurrect.g file
void GCodes::SaveResumeInfo(bool wasPowerFailure) noexcept
{
	const char *_ecv_array _ecv_null const printingFilename = reprap.GetPrintMonitor().GetPrintingFilename();
	if (printingFilename != nullptr)
	{
		FileStore *_ecv_null const f = platform.OpenSysFile(RESUME_AFTER_POWER_FAIL_G, OpenMode::write);
		if (f == nullptr)
		{
			platform.MessageF(ErrorMessage, "Failed to create file %s\n", RESUME_AFTER_POWER_FAIL_G);
		}
		else
		{
			String<StringLength256> buf;

			// Write the header comment
			buf.printf("; Resume printing file \"%s\" after %s", printingFilename, (wasPowerFailure) ? "power failure" : "print paused");
			tm timeInfo;
			if (platform.GetDateTime(timeInfo))
			{
				buf.catf(" at %04u-%02u-%02u %02u:%02u",
								timeInfo.tm_year + 1900, timeInfo.tm_mon + 1, timeInfo.tm_mday, timeInfo.tm_hour, timeInfo.tm_min);
			}
			buf.cat('\n');

			// Set bed and chamber heaters and restore too settings
			bool ok = f->Write(buf.c_str())
					&& reprap.GetHeat().WriteBedAndChamberTempSettings(f)	// turn on bed and chamber heaters
					&& reprap.GetMove().WriteResumeSettings(f);				// load grid, if we are using one
			if (ok)
			{
				ok = WriteToolSettings(f, buf.GetRef());					// set tool temperatures, tool mix ratios and tool heater states
			}

			if (ok)
			{
				ok = buildObjects.WriteObjectDirectory(f);					// write the state of printing objects
			}

			if (ok)
			{
				// Run resurrect-prologue.g. Pass the machine coordinates of all axes to it.
				float coords[MaxAxes];
				ToolOffsetTransform(moveStates[0], moveStates[0].GetPauseRestorePoint().moveCoords, coords);
#if SUPPORT_ASYNC_MOVES
				// For the second and subsequent movement systems, only update the positions of the owned axes
				for (size_t i = 1; i < NumMovementSystems; ++i)
				{
					float coords2[MaxAxes];
					ToolOffsetTransform(moveStates[i], moveStates[i].GetPauseRestorePoint().moveCoords, coords2);
					moveStates[i].GetAxesAndExtrudersOwned().Iterate([&coords, coords2](unsigned int bitNum, unsigned int) noexcept -> void { coords[bitNum] = coords2[bitNum]; });
				}
#endif
				String<StringLength100> buf2, buf3;

				// We no longer use G92 to restore the positions because we don't know whether a tool is loaded.
				buf.printf("G21\nM98 P\"%s\"", RESUME_PROLOGUE_G);		// set units to mm and call the prologue, passing the machine positions of the axes
				buf2.copy("\nM290 R0");									// set babystepping offsets
				buf3.copy("\nM205");									// set printing jerk (this is currently stored globally, not per movement system)
				for (size_t axis = 0; axis < numVisibleAxes; ++axis)
				{
					buf.catf (" %c%.3f", axisLetters[axis], (double)coords[axis]);
					buf2.catf(" %c%.3f", axisLetters[axis], (double)GetTotalBabyStepOffset(axis));
					buf3.catf(" %c%.2f", axisLetters[axis], (double)InverseConvertSpeedToMmPerSec(reprap.GetMove().GetPrintingInstantDv(axis)));
				}
				buf3.cat('\n');
				ok = f->Write(buf.c_str()) && f->Write(buf2.c_str()) && f->Write(buf3.c_str());									// write baby stepping offsets
			}

			// Restore the coordinate offsets of all workplaces (all motion systems use the same workplace offsets)
			if (ok)
			{
				ok = WriteWorkplaceCoordinates(f);
			}

			if (ok)
			{
#if SUPPORT_ASYNC_MOVES
				// Select each motion system in turn and write its settings
				for (size_t i = 0; ok && i < NumMovementSystems; ++i)
				{
					buf.printf("M596 P%u\n", i);
					ok = f->Write(buf.c_str()) && SaveMoveStateResumeInfo(moveStates[i], f, printingFilename, buf.GetRef());
				}

				// Go back to motion system 0
				if (ok) { ok = f->Write("M596 P0\n"); }
#else
				ok = SaveMoveStateResumeInfo(moveStates[0], f, printingFilename, buf.GetRef());
#endif
			}

			if (ok)
			{
				ok = reprap.GetFansManager().WriteFanSettings(f);			// set the speeds of all non-thermostatic fans after setting the default fan speeds
			}

			if (ok)
			{
				buf.printf("M302 P%u\n", (reprap.GetHeat().ColdExtrude()) ? 1 : 0);
				ok = f->Write(buf.c_str());									// write cold extrusion enabled/disabled
			}

#if SUPPORT_ASYNC_MOVES
			// If we were running in forked mode, fork the input reader
			if (ok && !FileGCode()->ExecutingAll())
			{
				ok = f->Write("M606 S1\n");
			}
#endif
			if (ok)
			{
				ok = f->Write("M24\n");										// resume printing
			}
			if (!f->Close())
			{
				ok = false;
			}
			if (ok)
			{
				platform.Message(LoggedGenericMessage, "Resume state saved\n");
			}
			else
			{
				platform.DeleteSysFile(RESUME_AFTER_POWER_FAIL_G);
				platform.MessageF(ErrorMessage, "Failed to write or close file %s\n", RESUME_AFTER_POWER_FAIL_G);
			}
		}
	}
}

// Write a portion of the resurrect.g file for a single movement system
// 'buf' is a convenient 256-byte buffer we can use
bool GCodes::SaveMoveStateResumeInfo(const MovementState& ms, FileStore * const f, const char *_ecv_array printingFilename, const StringRef& buf) noexcept
{
	// Write the current printing object
	buf.printf("M486 S%d\n", ms.currentObjectNumber);
	bool ok = f->Write(buf.c_str());


	// Now that we have homed, we can run the tool change files for the current tool
	const int toolNumber =  ms.GetCurrentToolNumber();
	if (ok && toolNumber >= 0)
	{
		buf.printf("T%d\n", toolNumber);					// select the required tool
		if (ms.currentTool->GetSpindleNumber() >= 0)
		{
			// Set the spindle RPM
			const Spindle& spindle = platform.AccessSpindle(ms.currentTool->GetSpindleNumber() >= 0);
			switch (spindle.GetState().RawValue())
			{
			case SpindleState::stopped:
			default:
				break;										// selecting the tool will have stopped the spindle

			case SpindleState::forward:
				buf.catf("M3 S%" PRIu32 " G4 S2\n", spindle.GetRpm());
				break;

			case SpindleState::reverse:
				buf.catf("M4 S%" PRIu32 " G4 S2\n", spindle.GetRpm());
				break;
			}
		}
		ok = f->Write(buf.c_str());							// write tool selection
	}

	if (ok)
	{
		// Switch to the correct workplace. 'currentCoordinateSystem' is 0-based.
		if (ms.currentCoordinateSystem <= 5)
		{
			buf.printf("G%u\n", 54 + ms.currentCoordinateSystem);
		}
		else
		{
			buf.printf("G59.%u\n", ms.currentCoordinateSystem - 5);
		}
		ok = f->Write(buf.c_str());
	}

#if SUPPORT_COORDINATE_ROTATION
	if (ok)
	{
		ok = WriteCoordinateRotation(f, ms);
	}
#endif

	const GCodeMachineState& oms = GetFileGCode(ms.GetNumber())->OriginalMachineState();
	if (ok && oms.volumetricExtrusion)
	{
		buf.copy("M200 ");
		char c = 'D';
		for (size_t i = 0; i < numExtruders; ++i)
		{
			buf.catf("%c%.03f", c, (double)volumetricExtrusionFactors[i]);
			c = ':';
		}
		buf.cat('\n');
		ok = f->Write(buf.c_str());									// write volumetric extrusion factors
	}
	if (ok)
	{
		buf.printf("M106 S%.2f\n", (double)ms.virtualFanSpeed);
		ok = f->Write(buf.c_str());									// set the speed of the print fan after we have selected the tool
	}
	if (ok)
	{
		buf.printf("M116\nG92 E%.5f\n%s\n%s\n", (double)ms.latestVirtualExtruderPosition,
				(oms.drivesRelative) ? "M83" : "M82",
					(oms.inverseTimeMode) ? "G93" : "G94");
		ok = f->Write(buf.c_str());									// write virtual extruder position, absolute/relative extrusion flag, and inverse time mode/normal mode flag
	}

	const RestorePoint& pauseRestorePoint = ms.GetPauseRestorePoint();
	if (ok)
	{
		const unsigned int selectedPlane = oms.selectedPlane;
		buf.printf("G%u\n", selectedPlane + 17);
#if SUPPORT_ASYNC_MOVES
		if (ms.GetNumber() == 0)
#endif
		{
			buf.catf("M23 \"%s\"\n", printingFilename);
		}
		buf.catf("M26 S%" PRIu32, pauseRestorePoint.filePos);
		if (pauseRestorePoint.gCommandNumber >= 0)
		{
			buf.catf(" C%d", (int)pauseRestorePoint.gCommandNumber);		// modal G0/G1/G2/G3 command to restore, omitted if unknown
		}
		if (pauseRestorePoint.proportionDone > 0.0)
		{
			buf.catf(" P%.3f %c%.3f %c%.3f",
					(double)pauseRestorePoint.proportionDone,
					(selectedPlane == 2) ? 'Y' : 'X', (double)pauseRestorePoint.initialUserC0,
					(selectedPlane == 0) ? 'Y' : 'Z', (double)pauseRestorePoint.initialUserC1);
		}
		buf.cat('\n');
		ok = f->Write(buf.c_str());									// write G17/18/19, filename and file position, and if necessary proportion done and initial XY position
	}

#if SUPPORT_ASYNC_MOVES
	const AxesBitmap ownedAxes = ms.GetAxesAndExtrudersOwned();
	if (ok && ownedAxes.IsBitSet(Z_AXIS))
#else
	if (ok)
#endif
	{
		// Build the commands to restore the head position. These assume that we are working in mm.
		// Start with a vertical move to 2mm above the final Z position
		buf.printf("G0 F6000 Z%.3f\n", (double)(pauseRestorePoint.moveCoords[Z_AXIS] + 2.0));
		ok = f->Write(buf.c_str());
	}

	if (ok)
	{
		// Now set all the other axes
		buf.copy("G0 F6000");
		bool restoredAxes = false;
		for (size_t axis = 0; axis < numVisibleAxes; ++axis)
		{
#if SUPPORT_ASYNC_MOVES
			bool restoreAxis;
			switch (axis)
			{
			case X_AXIS:
				restoreAxis = Tool::GetXAxes(ms.currentTool).IterateWhile([ownedAxes](unsigned int mappedAxis, unsigned int) noexcept -> bool { return ownedAxes.IsBitSet(mappedAxis); });
				break;

			case Y_AXIS:
				restoreAxis = Tool::GetYAxes(ms.currentTool).IterateWhile([ownedAxes](unsigned int mappedAxis, unsigned int) noexcept -> bool { return ownedAxes.IsBitSet(mappedAxis); });
				break;

			case Z_AXIS:
				restoreAxis = false;
				break;							// we already did the Z axis

			default:
				restoreAxis = ms.GetAxesAndExtrudersOwned().IsBitSet(axis);
				break;
			}
			if (restoreAxis)					// we already did the Z axis
#else
			if (axis != Z_AXIS)
#endif
			{
				restoredAxes = true;
				buf.catf(" %c%.3f", axisLetters[axis], (double)pauseRestorePoint.moveCoords[axis]);
			}
		}
		if (restoredAxes)
		{
			buf.cat('\n');
			ok = f->Write(buf.c_str());
		}
	}

	// Now move down to the correct Z height
#if SUPPORT_ASYNC_MOVES
	if (ok && ownedAxes.IsBitSet(Z_AXIS))
#else
	if (ok)
#endif
	{
		buf.printf("G0 F6000 Z%.3f\n", (double)pauseRestorePoint.moveCoords[Z_AXIS]);
		ok = f->Write(buf.c_str());
	}
	if (ok)
	{
		// Set the feed rate
		buf.printf("G1 F%.1f", (double)pauseRestorePoint.originalFeedRate);
#if SUPPORT_LASER
		if (machineType == MachineType::laser)
		{
			buf.catf(" S%u", (unsigned int)pauseRestorePoint.laserPwmOrIoBits.laserPwm);
		}
		else
#endif
#if SUPPORT_IOBITS
		{
			buf.catf(" P%u", (unsigned int)pauseRestorePoint.laserPwmOrIoBits.ioBits);
		}
#endif
		buf.cat('\n');
		ok = f->Write(buf.c_str());									// restore feed rate and output bits or laser power
	}
	if (ok)
	{
		buf.printf("M204 P%.1f T%.1f\n", (double)InverseConvertAcceleration(ms.raw.maxPrintingAcceleration), (double)InverseConvertAcceleration(ms.raw.maxTravelAcceleration));
		ok = f->Write(buf.c_str());									// restore printing and travel accelerations
	}
	if (ok)
	{
		ok = f->Write((oms.usingInches) ? "G20\n" : "G21\n");		// restore inches/mm
	}
	return ok;
}

#endif

void GCodes::Diagnostics(const StringRef& reply) noexcept
{
	reply.copy("=== GCodes ===\nMovement locks held by ");
	for (unsigned int i = 0; i < NumMovementSystems; ++i)
	{
		const GCodeBuffer *_ecv_null const movementOwner = resourceOwners[MoveResourceBase + i];
		if (i != 0)
		{
			reply.cat(", ");
		}
		reply.cat((movementOwner == nullptr) ? "null" : movementOwner->GetChannel().ToString());
	}

	for (GCodeBuffer *_ecv_null gb : gcodeSources)
	{
		if (gb != nullptr)
		{
			gb->Diagnostics(reply);
		}
	}

}

#if SUPPORT_ASYNC_MOVES

// Get the file GCode buffer that processes commands for this movement system
GCodeBuffer *_ecv_null GCodes::GetFileGCode(unsigned int msNumber) const noexcept
{
	return (msNumber == 0 || FileGCode()->ExecutingAll()) ? FileGCode() : File2GCode();
}

// Lock the movement system that we currently use and wait for it to stop
bool GCodes::LockCurrentMovementSystemAndWaitForStandstill(GCodeBuffer& gb) noexcept
{
	return LockMovementSystemAndWaitForStandstill(gb, gb.GetActiveQueueNumber());
}

// Lock all movement systems and wait for them to stop
bool GCodes::LockAllMovementSystemsAndWaitForStandstill(GCodeBuffer& gb) noexcept
{
	unsigned int i = 0;
	while (LockMovementSystemAndWaitForStandstill(gb, i))
	{
		++i;
		if (i == NumMovementSystems)
		{
			return true;
		}
	}

	// We failed to lock the ith movement system. Release the lower ones we took, otherwise two channels each holding their own system deadlock waiting for the other
	UnlockMovementTakenBelow(gb, i);
	return false;
}

#endif

// Lock movement and wait for pending moves to finish.
// Return true if successful, false if we need to try again later.
// As a side-effect it updates the user coordinates from the machine coordinates.
bool GCodes::LockMovementSystemAndWaitForStandstill(GCodeBuffer& gb, MovementSystemNumber msNumber) noexcept
{
	// Lock movement to stop another source adding moves to the queue
	if (!LockResource(gb, MoveResourceBase + msNumber))
	{
		return false;
	}

	MovementState& ms = moveStates[msNumber];
	if (ms.segmentsLeft != 0)						// has the last move generated been fully transferred to the movement queue?
	{
		return false;								// if no
	}

	Move& move = reprap.GetMove();
	switch (gb.GetChannel().ToBaseType())
	{
	case GCodeChannel::Queue:
	case GCodeChannel::Queue2:
		break;

	default:
		if (!move.WaitingForAllMovesFinished(msNumber
#if SUPPORT_ASYNC_MOVES
															, ms.raw.logicalDrivesOwned
#endif
														)
		   )
		{
			return false;
		}

		if (!(
#if SUPPORT_ASYNC_MOVES
				((msNumber == 1) ? Queue2GCode() : QueuedGCode())
#else
				QueuedGCode()
#endif
							->IsIdle() && ms.codeQueue->IsIdle()))
		{
			return false;
		}
		break;
	}

	gb.MotionStopped();									// must do this after we have finished waiting, so that we don't stop waiting when executing G4

	// Re-read the position from the motors only if the last move could have stopped short of its commanded target
	// (endstop/probe/stall/raw move). After an ordinary move the commanded coordinates are exact, so reading them
	// back would replace them with motor-step-rounded values and, in CNC mode, make a following G2/G3 fail the tight
	// arc radius check.
	if (ms.positionMayBeInaccurate)
	{
		ms.positionMayBeInaccurate = false;
		ms.UpdateOwnedDriveEndpointsFromMotors();			// need to do this if the move was stopped prematurely e.g. due to an endstop switch or a Z probe
		move.MotorStepsToCartesian(MovementState::GetLastKnownEndpoints(), numVisibleAxes, numTotalAxes, ms.raw.coords);
		move.UpdateStartCoordinates(ms.GetNumber(), ms.raw.coords);
		move.InverseAxisAndBedTransform(ms.raw.coords, ms.currentTool);
		UpdateUserPositionFromMachinePosition(gb, ms);
	}

#if SUPPORT_ASYNC_MOVES
	collisionChecker.ResetPositions(ms.raw.coords, ms.GetAxesAndExtrudersOwned());

	// Release the axes and extruders that this movement system owns, except those used by the current tool
	if (gb.AllStatesNormal())							// don't release them if we are in the middle of a state machine operation e.g. probing the bed
	{
# if PREALLOCATE_TOOL_AXES
		if (ms.currentTool != nullptr)
		{
			const AxesBitmap currentToolAxes = ms.currentTool->GetXYAxesAndExtruders();
			const AxesBitmap axesToRelease = ms.GetAxesAndExtrudersOwned() & ~currentToolAxes;
			ms.ReleaseAxesAndExtruders(axesToRelease);
		}
		else
# endif
		{
			ms.ReleaseAllOwnedAxesAndExtruders();
		}
	}
#endif
	return true;
}

// Save (some of) the state of the machine for recovery in the future.
// If the passed filename is non-null then we are executing a new macro. If is is null then we are saving the state but staying in the same file.
bool GCodes::Push(GCodeBuffer& gb, bool withinSameFile) noexcept
{
	const bool ok = gb.PushState(withinSameFile);
	if (!ok)
	{
		platform.MessageF(ErrorMessage, "Push(): stack overflow on %s\n", gb.GetChannel().ToString());
		AbortPrint(gb);
	}
	return ok;
}

// Recover a saved state
void GCodes::Pop(GCodeBuffer& gb, bool withinSameFile) noexcept
{
	if (!gb.PopState(withinSameFile))
	{
		platform.MessageF(ErrorMessage, "Pop(): stack underflow on %s\n", gb.GetChannel().ToString());
	}
	reprap.InputsUpdated();
}

// Set up the feed rate of a move for the Move class
// ms.moveType, ms.isCoordinated, ms.linearAxesMentioned and ms.rotationalAxesMentioned must be set up before calling this
void GCodes::LoadFeedrateFromGCode(GCodeBuffer& gb, MovementState& ms) THROWS(GCodeException)
{
	// Deal with feed rate, also determine whether M220 and M221 speed and extrusion factors apply to this move
	if (ms.raw.isCoordinated || machineType == MachineType::fff)
	{
		ms.raw.applyM220M221 = (ms.raw.moveType == 0 && (ms.raw.linearAxesMentioned || ms.raw.rotationalAxesMentioned) && !gb.LatestMachineState().runningSystemMacro);
		ms.raw.inverseTimeMode = gb.LatestMachineState().inverseTimeMode;
		if (ms.raw.inverseTimeMode)
		{
			if (!gb.Seen(feedrateLetter))
			{
				gb.ThrowGCodeException("Feed rate must be specified with every move when using inverse time mode");
			}
			const float feedRate = (StepClockRate * 60)/gb.GetPositiveFValue();			// get the requested move duration in step clocks
			ms.raw.feedRate = (ms.raw.applyM220M221)
							? feedRate/ms.speedFactor
								: feedRate;
		}
		else
		{
			if (gb.Seen(feedrateLetter))
			{
				gb.LatestMachineState().feedRate = gb.GetFValue();						// we now keep the raw value in the GCodeBuffer as we don't know whether to convert from inches yet, maybe not until the next command
			}
			const float convertedFeedRate = gb.ConvertSpeed(gb.LatestMachineState().feedRate, ms.raw.linearAxesMentioned || !ms.raw.rotationalAxesMentioned);
			ms.raw.feedRate = (ms.raw.applyM220M221)
							?  convertedFeedRate * ms.speedFactor
								: convertedFeedRate;
		}
		ms.raw.usingStandardFeedrate = true;
	}
	else
	{
		ms.raw.applyM220M221 = false;
		ms.raw.feedRate = ConvertSpeedFromMmPerMin(MaximumG0FeedRate);						// use maximum feed rate, the M203 parameters will limit it
		ms.raw.usingStandardFeedrate = false;
	}

	ms.raw.originalFeedRate = (float16_t)gb.LatestMachineState().feedRate;
}

// Set up the extrusion of a move, returning true if there is any extrusion
// moveBuffer.moveType, moveBuffer.isCoordinated, ms.moveType, ms.feedRate, ms.linearAxesMentioned and ms.rotationalAxesMentioned must be set up before calling this
bool GCodes::LoadExtrusionFromGCode(GCodeBuffer& gb, MovementState& ms) THROWS(GCodeException)
{
	// Zero every extruder drive as some drives may not be moved
	for (size_t drive = numTotalAxes; drive < MaxAxesPlusExtruders; drive++)
	{
		ms.raw.coords[drive] = 0.0;
	}
	bool hasExtrusion = false;
	ms.raw.hasPositiveExtrusion = false;
	ms.raw.moveStartVirtualExtruderPosition = ms.latestVirtualExtruderPosition;	// save this before we update it

	// Check if we are extruding
	if (gb.Seen(extrudeLetter))												// DC 2018-08-07: at E3D's request, extrusion is now recognised even on uncoordinated moves
	{
		// Check that we have a tool to extrude with
		const Tool *_ecv_null const tool = ms.currentTool;
		if (tool == nullptr)
		{
			displayNoToolWarning = true;
			return false;
		}

		ExtrudersBitmap extrudersMoving;
#if SUPPORT_ASYNC_MOVES && !PREALLOCATE_TOOL_AXES
		AxesBitmap logicalDrivesMoving;
#endif
		float cookedTotalExtrusion = 0.0;
		const size_t eMoveCount = tool->DriveCount();
		if (eMoveCount != 0)
		{
			// Set the drive values for this tool
			float eMovement[MaxExtruders];
			size_t mc = eMoveCount;
			gb.GetFloatArray(eMovement, mc, false);

			if (mc == 1)
			{
				// There may be multiple extruders present but only one value has been specified, so use mixing
				const float moveArg = gb.ConvertDistance(eMovement[0]);
				float requestedExtrusionAmount;
				if (gb.LatestMachineState().drivesRelative)
				{
					requestedExtrusionAmount = moveArg;
					ms.latestVirtualExtruderPosition += moveArg;
				}
				else
				{
					requestedExtrusionAmount = moveArg - ms.latestVirtualExtruderPosition;
					ms.latestVirtualExtruderPosition = moveArg;
				}

				hasExtrusion = true;
				if (requestedExtrusionAmount > 0.0)
				{
					ms.raw.hasPositiveExtrusion = true;
				}

				// rawExtruderTotal is used to calculate print progress, so it must be based on the requested extrusion from the slicer
				// before accounting for mixing, extrusion factor etc.
				// We used to have 'isPrintingMove &&' in the condition too, but this excluded wipe-while-retracting moves, so it gave wrong results for % print complete.
				// We still exclude extrusion during tool changing and other macros, because that is extrusion not known to the slicer.
				if (ms.raw.moveType == 0 && !gb.IsDoingFileMacro())
				{
					rawExtruderTotal += requestedExtrusionAmount;
				}

				float totalMix = 0.0;
				for (size_t eDrive = 0; eDrive < eMoveCount; eDrive++)
				{
					const float thisMix = tool->GetMix()[eDrive];
					if (thisMix != 0.0)
					{
						totalMix += thisMix;
						const int extruder = tool->GetDrive(eDrive);
						float extrusionAmount = requestedExtrusionAmount * thisMix;
						if (gb.LatestMachineState().volumetricExtrusion)
						{
							extrusionAmount *= volumetricExtrusionFactors[extruder];
						}
						if (ms.raw.moveType == 0 && !gb.IsDoingFileMacro())
						{
							rawExtruderTotalByDrive[extruder] += extrusionAmount;
						}

						const float cookedExtrusionAmount = (ms.raw.applyM220M221)
															? extrusionAmount * extrusionFactors[extruder]
															: extrusionAmount;
						ms.raw.coords[ExtruderToLogicalDrive(extruder)] = cookedExtrusionAmount;
						cookedTotalExtrusion += cookedExtrusionAmount;
						extrudersMoving.SetBit(extruder);
#if SUPPORT_ASYNC_MOVES && !PREALLOCATE_TOOL_AXES
						logicalDrivesMoving.SetBit(ExtruderToLogicalDrive(extruder));
#endif
					}
				}
				if (!(ms.raw.linearAxesMentioned || ms.raw.rotationalAxesMentioned) && ms.raw.usingStandardFeedrate)
				{
					// For E3D: If the total mix ratio is greater than 1.0 then we should scale the feed rate accordingly, e.g. for dual serial extruder drives
					ms.raw.feedRate *= totalMix;
				}
			}
			else
			{
				// Individual extrusion amounts have been provided. This is supported in relative extrusion mode only.
				// Note, if this is an extruder-only movement then the feed rate will apply to the total of all active extruders
				if (gb.LatestMachineState().drivesRelative)
				{
					for (size_t eDrive = 0; eDrive < mc; eDrive++)
					{
						const int extruder = tool->GetDrive(eDrive);
						float extrusionAmount = gb.ConvertDistance(eMovement[eDrive]);
						if (extrusionAmount != 0.0)
						{
							hasExtrusion = true;
							if (extrusionAmount > 0.0)
							{
								ms.raw.hasPositiveExtrusion = true;
							}

							if (gb.LatestMachineState().volumetricExtrusion)
							{
								extrusionAmount *= volumetricExtrusionFactors[extruder];
							}

							if (eDrive < mc && ms.raw.moveType == 0 && !gb.IsDoingFileMacro())
							{
								rawExtruderTotalByDrive[extruder] += extrusionAmount;
								rawExtruderTotal += extrusionAmount;
							}
							const float cookedExtrusionAmount = (ms.raw.applyM220M221)
																? extrusionAmount * extrusionFactors[extruder]
																: extrusionAmount;
							ms.raw.coords[ExtruderToLogicalDrive(extruder)] = cookedExtrusionAmount;
							cookedTotalExtrusion += cookedExtrusionAmount;
							extrudersMoving.SetBit(extruder);
#if SUPPORT_ASYNC_MOVES && !PREALLOCATE_TOOL_AXES
							logicalDrivesMoving.SetBit(ExtruderToLogicalDrive(extruder));
#endif
						}
					}
				}
				else
				{
					gb.ThrowGCodeException("multiple E parameters in G1 commands are not supported in absolute extrusion mode");
				}
			}
		}

#if SUPPORT_ASYNC_MOVES && !PREALLOCATE_TOOL_AXES
		AllocateAxes(gb, ms, logicalDrivesMoving, ParameterLettersBitmap());
#endif
		if ((ms.raw.moveType == 1 || ms.raw.moveType == 4) && cookedTotalExtrusion != 0.0)
		{
			// Enable extruder endstops for the extruders moving
			// First calculate the extruder speeds so that stall detection endstops can be validated.
			// We checked before calling this that no axes are moving.
			// Divide by the absolute total extrusion so that each speed is signed with the direction of that extruder's movement
			float speeds[MaxExtruders];
			for (size_t i = 0; i < GetNumExtruders(); ++i)
			{
				speeds[i] = ms.raw.coords[ExtruderToLogicalDrive(i)] * ms.raw.feedRate / fabsf(cookedTotalExtrusion);
			}
			bool reduceAcceleration;
			platform.GetEndstops().EnableExtruderEndstops(extrudersMoving, speeds, reduceAcceleration);			// this will throw if the endstops can't be enabled
			if (reduceAcceleration)
			{
				ms.raw.reduceAcceleration = true;
			}
		}
	}
	return hasExtrusion;
}

// Check that enough axes have been homed, returning true if insufficient axes homed
bool GCodes::CheckEnoughAxesHomed(AxesBitmap axesToMove) noexcept
{
	return (reprap.GetMove().GetKinematics().MustBeHomedAxes(axesToMove, noMovesBeforeHoming) & ~axesVirtuallyHomed).IsNonEmpty();
}

// Test whether a set of axes and a current Z height involves mesh bed compensation
// Caller must set up ms.isCoodinated before calling this
bool GCodes::IsUsingMeshCompensation(const MovementState& ms, ParameterLettersBitmap axesMentioned) const noexcept
{
	if ((ms.raw.isCoordinated || machineType == MachineType::fff) && reprap.GetMove().IsUsingMesh())
	{
		const GridDefinition& grid = reprap.GetMove().GetGrid();
		if (axesMentioned.IsAnyBitSet(ParameterLetterToBitNumber(grid.GetAxisLetter(0)), ParameterLetterToBitNumber(grid.GetAxisLetter(1))))
		{
			const float taperHeight = reprap.GetMove().GetTaperHeight();
			return (taperHeight == 0.0 || ms.currentUserPosition[Z_AXIS] < taperHeight);
		}
	}
	return false;
}

// Execute a straight move. We have already acquired the movement lock and waited for the previous move to be taken.
// Return false if we can't execute it yet, throw an exception if we can't execute it at all, and return true if we have queued it or it contains no movement.
bool GCodes::DoStraightMove(GCodeBuffer& gb, bool isCoordinated) THROWS(GCodeException)
{
	MovementState& ms = GetMovementState(gb);

	// Set up default move parameters
	ms.raw.movementTool = ms.currentTool;
	ms.raw.moveType = 0;
	ms.raw.isCoordinated = isCoordinated;
	ms.raw.checkEndstops = false;
	ms.raw.reduceAcceleration = false;
	ms.raw.usePressureAdvance = false;
	ms.raw.linearAxesMentioned = false;
	ms.raw.rotationalAxesMentioned = false;

#if SUPPORT_SCANNING_PROBES
	ms.raw.scanningProbeMove = false;
#endif

	ms.axesToSenseLength.Clear();
	ms.axesToHome.Clear();

	// Check to see if the move is a 'homing' move that endstops are checked on and for which X and Y axis mapping is not applied
	{
		uint32_t moveType;
		bool dummy;
		if (gb.TryGetLimitedUIValue('H', moveType, dummy, 5))
		{
			if (moveType != 0)
			{
				if (!LockCurrentMovementSystemAndWaitForStandstill(gb))
				{
					return false;
				}
				ms.raw.moveType = moveType;
				ms.raw.movementTool = nullptr;
			}
		}
	}

	Move& move = reprap.GetMove();

#if SUPPORT_ASYNC_MOVES
	// We need to check for moving unowned axes right at the start in case we need to fetch axis positions before processing the command
	ParameterLettersBitmap axisLettersMentioned = gb.AllParameters() & allAxisLetters;
	const bool meshCompensationInUse = (ms.raw.moveType == 0) && IsUsingMeshCompensation(ms, axisLettersMentioned);
	if (ms.raw.moveType == 0)
	{
		if (meshCompensationInUse)
		{
			axisLettersMentioned.SetBit(ParameterLetterToBitNumber('Z'));		// if we are using mesh compensation then Z will probably be moving
		}
		axisLettersMentioned.ClearBits(ms.GetOwnedAxisLetters());
		if (axisLettersMentioned.IsNonEmpty())
		{
			AllocateAxisLetters(gb, ms, axisLettersMentioned);
		}
	}
	else
	{
		// Homing/raw-motor moves: tool axis mapping is not applied, so allocate axes by literal letter
		AllocateLogicalDrivesFromLetters(gb, ms, axisLettersMentioned);
	}
#endif

	if (ms.moveFractionToSkip > 0.0)
	{
		ms.raw.initialUserC0 = ms.restartInitialUserC0;
		ms.raw.initialUserC1 = ms.restartInitialUserC1;
	}
	else
	{
		const unsigned int selectedPlane = gb.LatestMachineState().selectedPlane;
		ms.raw.initialUserC0 = ms.currentUserPosition[(selectedPlane == 2) ? Y_AXIS : X_AXIS];
		ms.raw.initialUserC1 = ms.currentUserPosition[(selectedPlane == 0) ? Y_AXIS : Z_AXIS];
	}

	// Check for 'R' parameter to move relative to a restore point
	const RestorePoint *_ecv_null rp = nullptr;
	if (ms.raw.moveType == 0 && gb.Seen('R'))
	{
		const uint32_t rParam = gb.GetUIValue();
		if (rParam < NumVisibleRestorePoints)
		{
			rp = &ms.restorePoints[rParam];
		}
		else
		{
			gb.ThrowGCodeException("bad restore point number");
		}
	}

	// Check for laser power setting or IOBITS
#if SUPPORT_LASER || SUPPORT_IOBITS
	if (rp != nullptr)
	{
		ms.raw.laserPwmOrIoBits = rp->laserPwmOrIoBits;
# if SUPPORT_LASER
		ms.laserPixelData = rp->laserPixelData;
# endif
	}
# if SUPPORT_LASER
	else if (machineType == MachineType::laser)
	{
		if (gb.Seen('S'))
		{
			float pixelBuffer[MaxLaserPixelsPerMove];
			ms.laserPixelData.numPixels = MaxLaserPixelsPerMove;
			gb.GetFloatArray(pixelBuffer, ms.laserPixelData.numPixels, false);
			for (size_t i = 0; i < ms.laserPixelData.numPixels; ++i)
			{
				ms.laserPixelData.pixelPwm[i] = ConvertLaserPwm(pixelBuffer[i]);
			}
		}
		else if (!laserPowerSticky)
		{
			ms.laserPixelData.Clear();
		}
	}
# endif
# if SUPPORT_IOBITS
	else
	{
		// Update the iobits parameter
		if (gb.Seen('P'))
		{
			ms.raw.laserPwmOrIoBits.ioBits = (IoBits_t)gb.GetIValue();
		}
		else
		{
			// Leave moveBuffer.ioBits alone so that we keep the previous value
		}
	}
# endif
#endif

	if (ms.raw.moveType != 0)
	{
		// This may be a raw motor move, in which case we need the current raw motor positions in moveBuffer.coords.
		if (move.IsRawMotorMove(ms.raw.moveType))
		{
			int32_t endPoints[MaxAxesPlusExtruders];
			move.GetLastEndpoints(ms.GetNumber(), RawMove::allLogicalDrives, endPoints);
			for (size_t axis = 0; axis < numTotalAxes; ++axis)
			{
				ms.raw.coords[axis] = move.MotorStepsToMovement(axis, endPoints[axis]);
			}
		}
		else
		{
			// It isn't a raw motor move, but it will still be applied without axis or bed transform applied,
			// so make sure the initial coordinates don't have those either to avoid unwanted Z movement.
			move.GetCurrentMachinePosition(ms.raw.coords, ms.GetNumber());
		}
	}

	// Set up the initial coordinates
	memcpyf(ms.initialCoords, ms.raw.coords, numVisibleAxes);

	// Save the current position, we need it possibly later
	float initialUserPosition[MaxAxes];
	memcpyf(initialUserPosition, ms.currentUserPosition, numVisibleAxes);

	// Restore the state we have already changed, called before throwing to abandon the move.
	// Only safe to call after LoadExtrusionFromGCode has set up moveStartVirtualExtruderPosition for this move
	const auto abandonMove = [this, &ms, &initialUserPosition]() noexcept
	{
		memcpyf(ms.currentUserPosition, initialUserPosition, numVisibleAxes);
		memcpyf(ms.raw.coords, ms.initialCoords, numVisibleAxes);
		ms.latestVirtualExtruderPosition = ms.raw.moveStartVirtualExtruderPosition;
	};

	AxesBitmap axesMentioned;
	for (size_t axis = 0; axis < numVisibleAxes; axis++)
	{
		if (gb.Seen(axisLetters[axis]))
		{
			// If it is a special move on a delta, movement must be relative.
			if (ms.raw.moveType != 0 && !gb.LatestMachineState().axesRelative && move.GetKinematics().GetKinematicsType() == KinematicsType::linearDelta)
			{
				gb.ThrowGCodeException("attempt to move individual motors of a delta machine to absolute positions");
			}

			axesMentioned.SetBit(axis);
			float moveArg;
			if (move.IsAxisRotational(axis))
			{
				ms.raw.rotationalAxesMentioned = true;
				moveArg = gb.GetFValue();
			}
			else
			{
				ms.raw.linearAxesMentioned = true;
				moveArg = gb.GetDistance();
			}
			if (ms.raw.moveType != 0)
			{
				// Special moves update the move buffer directly, bypassing the user coordinates
				if (gb.LatestMachineState().axesRelative)
				{
					ms.raw.coords[axis] += moveArg * (1.0 - ms.moveFractionToSkip);
				}
				else
				{
					ms.raw.coords[axis] = moveArg;
				}
			}
			else if (rp != nullptr)
			{
				ms.currentUserPosition[axis] = moveArg + rp->moveCoords[axis];
				// When a restore point is being used (G1 R parameter) then we used to set any coordinates that were not mentioned to the restore point values.
				// But that causes issues for tool change on IDEX machines because we end up restoring the U axis when we shouldn't.
				// So we no longer do that, and the user must mention any axes that he wants restored e.g. G1 R2 X0 Y0.
			}
			else if (gb.LatestMachineState().axesRelative)
			{
				ms.currentUserPosition[axis] += moveArg * (1.0 - ms.moveFractionToSkip);
			}
			else if (gb.LatestMachineState().g53Active)
			{
				ms.currentUserPosition[axis] = moveArg + ms.GetCurrentToolOffset(axis)/axisScaleFactors[axis];	// g53 ignores tool offsets as well as workplace coordinates
			}
			else if (gb.LatestMachineState().runningSystemMacro)
			{
				ms.currentUserPosition[axis] = moveArg;									// don't apply workplace offsets to commands in system macros
			}
			else
			{
				ms.currentUserPosition[axis] = moveArg + GetWorkplaceOffset(gb, axis);
			}
		}
	}

	try
	{
		LoadFeedrateFromGCode(gb, ms);													// set up feedrate before we do the endstop calculations
	}
	catch (const GCodeException&)
	{
		memcpyf(ms.currentUserPosition, initialUserPosition, numVisibleAxes);			// undo the user position update because this move will not be executed
		throw;
	}

	AxesBitmap realAxesMoving;															// we'll need this later but only if ms.moveType == 0
	if (ms.raw.moveType ==  0)
	{
#if SUPPORT_COORDINATE_ROTATION
		// Update the list of axes mentioned to allow for cross coupling between X and Y
		if (ms.g68Angle != 0.0 && gb.DoingCoordinateRotation())
		{
			const AxesBitmap xAndY = AxesBitmap::MakeFromBits(X_AXIS, Y_AXIS);
			if (axesMentioned.Intersects(xAndY))
			{
				axesMentioned |= xAndY;													// if either X or Y is moving and we are rotating coordinates, both are moving
			}
		}
#endif
		// Check and record which real axes this movement system is moving
		if (ms.currentTool == nullptr)
		{
			realAxesMoving = axesMentioned;
		}
		else
		{
			realAxesMoving = axesMentioned & ~AxesBitmap::MakeFromBits(X_AXIS, Y_AXIS, Z_AXIS);
			if (axesMentioned.IsBitSet(X_AXIS))
			{
				realAxesMoving |= ms.currentTool->GetXAxisMap();
			}
			if (axesMentioned.IsBitSet(Y_AXIS))
			{
				realAxesMoving |= ms.currentTool->GetYAxisMap();
			}
			if (axesMentioned.IsBitSet(Z_AXIS))
			{
				realAxesMoving |= ms.currentTool->GetZAxisMap();
			}
		}

		if (!doingManualBedProbe && CheckEnoughAxesHomed(realAxesMoving))
		{
			memcpyf(ms.currentUserPosition, initialUserPosition, numVisibleAxes);	// undo the user position update because this move will not be executed
			gb.ThrowGCodeException("insufficient axes homed");
		}
	}
	else if (ms.raw.moveType != 2)
	{
		if (axesMentioned.IsNonEmpty() && gb.Seen(extrudeLetter))
		{
			// If we enable both axis and extruder endstops then the extrusion speed calculation goes wrong, so don't allow it (I can't think of a reasonable use case for it)
			gb.ThrowGCodeException("cannot enable both axis and extruder endstops in the same move");
		}

		switch (ms.raw.moveType)
		{
		case 3:
			ms.axesToSenseLength = axesMentioned & AxesBitmap::MakeLowestNBits(numTotalAxes);
			break;

		case 1:
			ms.axesToHome = axesMentioned & AxesBitmap::MakeLowestNBits(numTotalAxes);
			break;

		default:
			break;
		}

		// Calculate the approximate axis top speeds so that EnableAxisEndstops can validate stall endstops. Unfortunately this duplicates code in DDA::InitStandardMove.
		// We assume that the homing move hasn't been commanded at a speed that exceeds any of the individual axis maximum speeds.
		// The feed rate refers to the composite linear axis movement. We assume hat we don't have both linear and rotational movement.
		float speeds[MaxAxes];
		const Kinematics &_ecv_from kin = move.GetKinematics();
		if (kin.GetHomingMode() == HomingMode::homeCartesianAxes)
		{
			// The endstops are on axes, so calculate the axis speeds and then convert them to drive speeds
			float totalDistanceSquared = 0.0;
			for (size_t axis = 0; axis < numVisibleAxes; ++axis)
			{
				speeds[axis] = ms.raw.coords[axis] - ms.initialCoords[axis];				// get movement amount
				totalDistanceSquared += fsquare(speeds[axis]);
			}
			//debugPrintf("speeds1a %.3g, %.3g, %.3g dsquared %.3g\n", (double)speeds[0], (double)speeds[1], (double)speeds[2], (double)totalDistanceSquared);
			const float speedFactor = ms.raw.feedRate/sqrtf(totalDistanceSquared);
			for (size_t axis = 0; axis < numVisibleAxes; ++axis)
			{
				speeds[axis] *= speedFactor;
			}

			//debugPrintf("speeds2 %.3g, %.3g, %.3g\n", (double)speeds[0], (double)speeds[1], (double)speeds[2]);
			// Convert the speeds to logical drive speeds
			kin.ConvertAxisAmountsToLogicalDriveAmounts(speeds, numVisibleAxes, numTotalAxes);
			//debugPrintf("speeds3 %.3g, %.3g, %.3g\n", (double)speeds[0], (double)speeds[1], (double)speeds[2]);
		}
		else
		{
			// The endstops are on individual logical drives
			// We already fetched the raw motor coordinates, so we can use the movement amounts directly. The feed rate refers to the axis that is moving the most.
			float maxDistance = 0.0;
			for (size_t axis = 0; axis < numVisibleAxes; ++axis)
			{
				speeds[axis] -= ms.initialCoords[axis];									// convert to movement amount
				if (speeds[axis] > maxDistance) { maxDistance = speeds[axis]; }
			}
			const float speedFactor = ms.raw.feedRate/maxDistance;
			for (size_t axis = 0; axis < numVisibleAxes; ++axis)
			{
				speeds[axis] *= speedFactor;
			}
		}

		bool reduceAcceleration;
		platform.GetEndstops().EnableAxisEndstops(axesMentioned & AxesBitmap::MakeLowestNBits(numTotalAxes), speeds, ms.raw.moveType == 1, reduceAcceleration); 	// throws if endstops can't be enabled
		if (reduceAcceleration)
		{
			ms.raw.reduceAcceleration = true;
		}
		ms.raw.checkEndstops = true;
		ms.endstopsTriggered.Clear();
	}

	bool hasExtrusion;
	try
	{
		hasExtrusion = LoadExtrusionFromGCode(gb, ms);										// for type 1 moves, this must be called after calling EnableAxisEndstops, because EnableExtruderEndstop assumes that
	}
	catch (const GCodeException&)
	{
		abandonMove();
		throw;
	}
	if (hasExtrusion || axesMentioned.IsNonEmpty())											// if there is no movement at all, skip further processing and don't pass the move on the the Move system
	{
		if (ms.IsFirstMoveSincePrintingResumed())											// if this is the first move after skipping an object
		{
			if (!LockCurrentMovementSystemAndWaitForStandstill(gb))							// update the user position from the machine position
			{
				return false;
			}
			ms.DoneMoveSincePrintingResumed();
			if (hasExtrusion)
			{
				TravelToStartPoint(gb, ms);													// don't start a printing move from the wrong place
				return false;
			}
		}

		if (ms.raw.hasPositiveExtrusion && axesMentioned.IsNonEmpty())
		{
			// Update the object coordinates limits. For efficiency, we only update the final coordinate.
			// Except in the case of a straight line that is only one extrusion width wide, this is sufficient.
			buildObjects.UpdateObjectCoordinates(ms.currentObjectNumber, ms.currentUserPosition, axesMentioned);
		}

		// Set up the move. We must assign segmentsLeft last, so that when Move runs as a separate task the move won't be picked up by the Move process before it is complete.
		// Note that if this is an extruder-only move, we don't do axis movements to allow for tool offset changes, we defer those until an axis moves.
		if (ms.raw.moveType != 0)
		{
			// It's a raw motor move, so do it in a single segment and wait for it to complete
			ms.totalSegments = 1;
			gb.SetState(GCodeState::waitingForSpecialMoveToComplete);
		}
		else if (axesMentioned.IsEmpty())
		{
			ms.totalSegments = 1;												// it's an extruder only move
		}
		else
		{
#if SUPPORT_COORDINATE_ROTATION
			if (ms.g68Angle != 0.0 && gb.DoingCoordinateRotation())
			{
				float coords[MaxAxes];
				memcpyf(coords, ms.currentUserPosition, MaxAxes);
				RotateCoordinates(ms, ms.g68Angle, coords);
				ToolOffsetTransform(ms, coords, ms.raw.coords, axesMentioned);
			}
			else
#endif
			{
				ToolOffsetTransform(ms, axesMentioned);							// apply tool offset, baby stepping, Z hop and axis scaling
			}

#if SUPPORT_KEEPOUT_ZONES
			if (keepoutZone.DoesLineIntrude(ms.initialCoords, ms.raw.coords))
			{
				abandonMove();
				gb.ThrowGCodeException("straight move would enter keepout zone");
			}
#endif

#if SUPPORT_ASYNC_MOVES
			if (!collisionChecker.UpdatePositions(ms.raw.coords, axesHomed))
			{
				abandonMove();
				gb.ThrowGCodeException("potential collision detected");
			}
#endif

			// Only limit the positions of axes that have been mentioned explicitly.
			// This avoids at least two problems:
			// 1. When supporting multiple motion systems, if a M208 axis limit was changed and an axis coordinate was outside that limit,
			//    but we don't own the axis, then if we move that axis there will be a problem when SaveOwnAxisCoordinates is called
			//    because the new coordinate won't be saved.
			// 2. If a linear axis is being limited, but the move is for a rotational axis that is already in the correct position,
			//    then the code in DDA::InitStandardMove will throw it away because neither linearAxesMoving nor rotationalAxesMoving will be set.
			//    This was an actual problem on my tool changer.
			AxesBitmap axesToLimit = axesVirtuallyHomed & realAxesMoving;
			if (doingManualBedProbe)
			{
				axesToLimit.ClearBit(Z_AXIS);									// if doing a manual Z probe, don't limit the Z movement
			}

			const LimitPositionResult lp = move.GetKinematics().LimitPosition(ms.raw.coords, ms.initialCoords, numVisibleAxes, axesToLimit, ms.raw.isCoordinated, limitAxes);
			switch (lp)
			{
			case LimitPositionResult::adjusted:
			case LimitPositionResult::adjustedAndIntermediateUnreachable:
				if (!gb.LatestMachineState().axesRelative || !limitAxesRelative)
				{
					abandonMove();
					gb.ThrowGCodeException(TargetUnreachableText);				// unreachable moves are errors unless relative moves are being clamped
				}
				ToolOffsetInverseTransform(ms);									// make sure the limits are reflected in the user position
				if (lp == LimitPositionResult::adjusted)
				{
					break;														// we can reach the intermediate positions, so nothing more to do
				}
				[[fallthrough]];

			case LimitPositionResult::intermediateUnreachable:
				if (   ms.raw.isCoordinated
					&& (   (machineType == MachineType::fff && !ms.raw.hasPositiveExtrusion)
#if SUPPORT_LASER || SUPPORT_IOBITS
						|| (machineType == MachineType::laser && ms.laserPixelData.numPixels == 0)
#endif
					   )
				   )
				{
					// It's a coordinated travel move on a 3D printer or laser cutter, with no extrusion or laser, so see whether an uncoordinated move will work
					const LimitPositionResult lp2 = move.GetKinematics().LimitPosition(ms.raw.coords, ms.initialCoords, numVisibleAxes, axesToLimit, false, limitAxes);
					if (lp2 == LimitPositionResult::ok)
					{
						ms.raw.isCoordinated = false;								// change it to an uncoordinated move
						break;
					}
				}
				abandonMove();
				gb.ThrowGCodeException("target position not reachable from current position");		// we can't bring the move within limits, so this is a definite error
				[[fallthrough]];

			case LimitPositionResult::ok:
			default:
				break;
			}

			// If we are emulating Marlin for nanoDLP then we need to set a special end state
			if (gb.LatestMachineState().compatibility == Compatibility::NanoDLP && !DoingFileMacro())
			{
				gb.SetState(GCodeState::waitingForSpecialMoveToComplete);
			}

			// Flag whether we should use pressure advance, if there is any extrusion in this move.
			// We assume it is a normal printing move needing pressure advance if there is forward extrusion and XYU... movement (we don't count Z).
			// The movement code will only apply pressure advance if there is forward extrusion, so we only need to check for XYU... movement here.
			if (ms.raw.hasPositiveExtrusion)
			{
				AxesBitmap axesMentionedExceptZ = axesMentioned;
				axesMentionedExceptZ.ClearBit(Z_AXIS);
				ms.raw.usePressureAdvance = axesMentionedExceptZ.IsNonEmpty();
			}

			// Apply segmentation if necessary
			// As soon as we set segmentsLeft nonzero, the Move process will assume that the move is ready to take, so this must be the last thing we do.
			const Kinematics &_ecv_from kin = move.GetKinematics();
			const SegmentationType st = kin.GetSegmentationType();
			float moveLengthSquared = fsquare(ms.currentUserPosition[X_AXIS] - initialUserPosition[X_AXIS]) + fsquare(ms.currentUserPosition[Y_AXIS] - initialUserPosition[Y_AXIS]);
			if (st.useZSegmentation)
			{
				moveLengthSquared += fsquare(ms.currentUserPosition[Z_AXIS] - initialUserPosition[Z_AXIS]);
			}
			const float moveLength = fastSqrtf(moveLengthSquared);
			const float moveTime = moveLength/(ms.raw.feedRate * StepClockRate);		// this is a best-case time, often the move will take longer

#if SUPPORT_LASER
			if (machineType == MachineType::laser && isCoordinated && ms.laserPixelData.numPixels > 1)
			{
				ms.totalSegments = ms.laserPixelData.numPixels;			// we must use one segment per pixel
			}
			else
#endif
			{

				// To speed up simulation on SCARA printers, we don't apply kinematics segmentation when simulating.
				if (st.useSegmentation && simulationMode != SimulationMode::normal && (ms.raw.hasPositiveExtrusion || ms.raw.isCoordinated || st.useG0Segmentation))
				{
					// This kinematics approximates linear motion by means of segmentation
					ms.totalSegments = (unsigned int)max<long>(1, lrintf(min<float>(moveLength * kin.GetReciprocalMinSegmentLength(), moveTime * kin.GetSegmentsPerSecond())));
				}
				else
				{
					ms.totalSegments = 1;
				}

				// If we are applying mesh compensation, set the segment size to be smaller than the mesh spacing.
				// Do not use segmentation if the requested tool Z position is higher than the configured taper height
#if !SUPPORT_ASYNC_MOVES
				const bool meshCompensationInUse = IsUsingMeshCompensation(ms, gb.AllParameters() & allAxisLetters);
#endif
				if (meshCompensationInUse)
				{
					const HeightMap& heightMap = move.AccessHeightMap();
					const GridDefinition& grid = heightMap.GetGrid();
					const unsigned int minMeshSegments = heightMap.GetMinimumSegments(
							ms.currentUserPosition[grid.GetAxisNumber(0)] - initialUserPosition[grid.GetAxisNumber(0)],
							ms.currentUserPosition[grid.GetAxisNumber(1)] - initialUserPosition[grid.GetAxisNumber(1)]
					);
					if (minMeshSegments > ms.totalSegments)
					{
						ms.totalSegments = minMeshSegments;
					}
				}
			}

			// The step clock wraps around every ~45 minutes (a bit less on Duet 2) which causes issues if the move will take more than about half this time.
			// So if the move will take more than about 5 minutes, segment it.
			ms.totalSegments = max<unsigned int>(ms.totalSegments, (unsigned int)(moveTime * (1.0/(float)MaxSegmentTime)));
		}

		ms.doingArcMove = false;
		FinaliseMove(gb, ms);
	}
	UnlockAll(gb);			// allow pause
	return true;
}

// Execute an arc move
// We already have the movement lock and the last move has gone
// Currently, we do not process new babystepping when executing an arc move
// Return true if finished, false if needs to be called again
// If an error occurs, return true with 'err' assigned
bool GCodes::DoArcMove(GCodeBuffer& gb, bool clockwise) THROWS(GCodeException)
{
	MovementState& ms = GetMovementState(gb);

	// Set up default move parameters
	ms.raw.movementTool = ms.currentTool;
	ms.raw.moveType = 0;
	ms.raw.isCoordinated = true;													// must set this before calling IsUsingMeshCompensation
	ms.raw.checkEndstops = false;
	ms.raw.reduceAcceleration = false;
	ms.raw.linearAxesMentioned = false;
	ms.raw.rotationalAxesMentioned = false;

#if SUPPORT_SCANNING_PROBES
	ms.raw.scanningProbeMove = false;
#endif

	// The planes are XY, ZX and YZ depending on the G17/G18/G19 setting. We must use ZX instead of XZ to get the correct arc direction.
	const unsigned int selectedPlane = gb.LatestMachineState().selectedPlane;
	const unsigned int axis0 = (unsigned int[]){ X_AXIS, Z_AXIS, Y_AXIS }[selectedPlane];
	const unsigned int axis1 = (axis0 + 1) % 3;

#if SUPPORT_ASYNC_MOVES
	// We need to check for moving unowned axes right at the start in case we need to fetch axis positions before processing the command
	ParameterLettersBitmap axisLettersMentioned = gb.AllParameters() & allAxisLetters;
	axisLettersMentioned.SetBit(ParameterLetterToBitNumber('X') + axis0);		// add in the implicit axes
	axisLettersMentioned.SetBit(ParameterLetterToBitNumber('X') + axis1);
	if (IsUsingMeshCompensation(ms, axisLettersMentioned))
	{
		axisLettersMentioned.SetBit(ParameterLetterToBitNumber('Z'));			// if we are using mesh compensation then Z will probably be moving
	}
	axisLettersMentioned.ClearBits(ms.GetOwnedAxisLetters());
	if (axisLettersMentioned.IsNonEmpty())
	{
		AllocateAxisLetters(gb, ms, axisLettersMentioned);
	}
#endif

	if (ms.moveFractionToSkip > 0.0)
	{
		ms.raw.initialUserC0 = ms.restartInitialUserC0;
		ms.raw.initialUserC1 = ms.restartInitialUserC1;
	}
	else
	{
		ms.raw.initialUserC0 = ms.currentUserPosition[axis0];
		ms.raw.initialUserC1 = ms.currentUserPosition[axis1];
	}

	// Get the axis parameters
	float newAxis0Pos, newAxis1Pos;
	if (gb.Seen(axisLetters[axis0]))
	{
		newAxis0Pos = gb.GetDistance();
		if (gb.LatestMachineState().axesRelative)
		{
			newAxis0Pos += ms.raw.initialUserC0;
		}
		else if (gb.LatestMachineState().g53Active)
		{
			newAxis0Pos += ms.GetCurrentToolOffset(axis0)/axisScaleFactors[axis0];
		}
		else if (!gb.LatestMachineState().runningSystemMacro)
		{
			newAxis0Pos += GetWorkplaceOffset(gb, axis0);
		}
	}
	else
	{
		newAxis0Pos = ms.raw.initialUserC0;
	}

	if (gb.Seen(axisLetters[axis1]))
	{
		newAxis1Pos = gb.GetDistance();
		if (gb.LatestMachineState().axesRelative)
		{
			newAxis1Pos += ms.raw.initialUserC1;
		}
		else if (gb.LatestMachineState().g53Active)
		{
			newAxis1Pos += ms.GetCurrentToolOffset(axis1)/axisScaleFactors[axis1];
		}
		else if (!gb.LatestMachineState().runningSystemMacro)
		{
			newAxis1Pos += GetWorkplaceOffset(gb, axis1);
		}
	}
	else
	{
		newAxis1Pos = ms.raw.initialUserC1;
	}

	float iParam, jParam;
	const bool radiusFormat = gb.Seen('R');
	if (radiusFormat)
	{
		// We've been given a radius, which takes precedence over I and J parameters
		const float rParam = gb.GetDistance();

		// Get the XY coordinates of the midpoints between the start and end points X and Y distances between start and end points
		const float deltaAxis0 = newAxis0Pos - ms.raw.initialUserC0;
		const float deltaAxis1 = newAxis1Pos - ms.raw.initialUserC1;

		const float dSquared = fsquare(deltaAxis0) + fsquare(deltaAxis1);	// square of the distance between start and end points

		// The distance between start and end points must not be zero
		if (dSquared == 0.0)
		{
			gb.ThrowGCodeException("distance between start and end points must not be zero when specifying a radius");
		}

		// The perpendicular must have a real length (possibly zero)
		const float hSquared = fsquare(rParam) - dSquared/4;				// square of the length of the perpendicular from the mid point to the arc centre

		// When the arc is exactly 180deg, rounding error may make hSquared slightly negative instead of zero
		float hDivD;
		if (hSquared >= 0.0)
		{
			hDivD = fastSqrtf(hSquared/dSquared);
		}
		else
		{
			if (hSquared < -0.02 * fsquare(rParam))							// allow the radius to be up to 1% too short
			{
				gb.ThrowGCodeException("radius is too small to reach endpoint");
			}
			hDivD = 0.0;													// this has the effect of increasing the radius slightly so that the maths works
		}

		ms.arcRadius = fabsf(rParam);

		// If hDivD is nonzero then there are two possible positions for the arc centre, giving a short arc (less than 180deg) or a long arc (more than 180deg).
		// According to https://www.cnccookbook.com/cnc-g-code-arc-circle-g02-g03/ we should choose the shorter arc if the radius is positive, the longer one if it is negative.
		// If the arc is clockwise then a positive value of h/d gives the smaller arc. If the arc is anticlockwise then it's the other way round.
		if ((clockwise && rParam < 0.0) || (!clockwise && rParam > 0.0))
		{
			hDivD = -hDivD;
		}
		iParam = deltaAxis0/2 + deltaAxis1 * hDivD;
		jParam = deltaAxis1/2 - deltaAxis0 * hDivD;
	}
	else
	{
		// Centre format arc
		if (gb.Seen((char)('I' + axis0)))
		{
			iParam = gb.GetDistance();
		}
		else
		{
			iParam = 0.0;
		}

		if (gb.Seen((char)('I' + axis1)))
		{
			jParam = gb.GetDistance();
		}
		else
		{
			jParam = 0.0;
		}

		if (iParam == 0.0 && jParam == 0.0)			// at least one of IJK must be specified and nonzero
		{
			gb.ThrowGCodeException("no I J K or R parameter");
		}

		ms.arcRadius = fastSqrtf(fsquare(iParam) + fsquare(jParam));
	}

	// Save the arc centre user coordinates for later
	float userArcCentre[2] = { ms.raw.initialUserC0 + iParam, ms.raw.initialUserC1 + jParam };

	if (!radiusFormat)
	{
		// From section 3.5.3.2 of the NIST standard, in centre format it is an error if:
		// "when the arc is projected on the selected plane, the distance from the current point to the center differs from the distance
		// from the end point to the center by more than 0.0002 inch (if inches are being used) or 0.002 millimeter (if millimeters are being used)."
		// In non-CNC modes we allow a larger error because slicers and ArcWelder may not output coordinates to a resolution of 0.002mm
		const float finalRadius = fastSqrtf(fsquare(newAxis0Pos - userArcCentre[0]) + fsquare(newAxis1Pos - userArcCentre[1]));
		const float maxRadiusDifference = (machineType == MachineType::cnc)
											? ((gb.LatestMachineState().usingInches) ? MaxCncRadiusErrorInches * InchToMm : MaxCncRadiusErrorMm)
												: MaxNonCncRadiusError;
		if (fabsf(finalRadius - ms.arcRadius) > maxRadiusDifference)
		{
			gb.ThrowGCodeException("final radius not equal to initial radius");
		}
	}

	memcpyf(ms.initialCoords, ms.raw.coords, numVisibleAxes);

	// Save the current position in case we have to abandon the move
	float initialUserPosition[MaxAxes];
	memcpyf(initialUserPosition, ms.currentUserPosition, numVisibleAxes);

	// Set the new user position
	ms.currentUserPosition[axis0] = newAxis0Pos;
	ms.currentUserPosition[axis1] = newAxis1Pos;

	// CNC machines usually do a full circle if the initial and final XY coordinates are the same.
	// Usually this is because X and Y were not given, but repeating the coordinates is permitted.
	const bool wholeCircle = (ms.raw.initialUserC0 == newAxis0Pos && ms.raw.initialUserC1 == newAxis1Pos);

	// Get any additional axes
	AxesBitmap axesMentioned;
	axesMentioned.SetBit(axis0);
	axesMentioned.SetBit(axis1);
	ms.raw.linearAxesMentioned = true;														// XYZ axes are always linear
	for (size_t axis = 0; axis < numVisibleAxes; axis++)
	{
		if (axis != axis0 && axis != axis1 && gb.Seen(axisLetters[axis]))
		{
			axesMentioned.SetBit(axis);
			float moveArg;
			if (reprap.GetMove().IsAxisRotational(axis))
			{
				ms.raw.rotationalAxesMentioned = true;
				moveArg = gb.GetFValue();
			}
			else
			{
				moveArg = gb.GetDistance();
			}

			if (gb.LatestMachineState().axesRelative)
			{
				ms.currentUserPosition[axis] += moveArg * (1.0 - ms.moveFractionToSkip);
			}
			else if (gb.LatestMachineState().g53Active)
			{
				ms.currentUserPosition[axis] = moveArg + ms.GetCurrentToolOffset(axis)/axisScaleFactors[axis];	// g53 ignores tool offsets as well as workplace coordinates
			}
			else if (gb.LatestMachineState().runningSystemMacro)
			{
				ms.currentUserPosition[axis] = moveArg;									// don't apply workplace offsets to commands in system macros
			}
			else
			{
				ms.currentUserPosition[axis] = moveArg + GetWorkplaceOffset(gb, axis);
			}
		}
	}

	// Check enough axes have been homed
	AxesBitmap realAxesMoving;
	if (ms.currentTool == nullptr)
	{
		realAxesMoving = axesMentioned;
	}
	else
	{
		realAxesMoving = axesMentioned & ~AxesBitmap::MakeFromBits(X_AXIS, Y_AXIS, Z_AXIS);
		if (axesMentioned.IsBitSet(X_AXIS))
		{
			realAxesMoving |= ms.currentTool->GetXAxisMap();
		}
		if (axesMentioned.IsBitSet(Y_AXIS))
		{
			realAxesMoving |= ms.currentTool->GetYAxisMap();
		}
		if (axesMentioned.IsBitSet(Z_AXIS))
		{
			realAxesMoving |= ms.currentTool->GetZAxisMap();
		}
	}

	if (CheckEnoughAxesHomed(realAxesMoving))
	{
		memcpyf(ms.currentUserPosition, initialUserPosition, numVisibleAxes);	// undo the user position update because this move will not be executed
		gb.ThrowGCodeException("insufficient axes homed");
	}

	// Compute the initial and final angles. Do this before we possibly rotate the coordinates of the arc centre.
	float finalTheta = atan2f(ms.currentUserPosition[axis1] - userArcCentre[1], ms.currentUserPosition[axis0] - userArcCentre[0]);
	ms.arcCurrentAngle = atan2f(-jParam, -iParam);

	// Transform to machine coordinates and check that it is within limits

#if SUPPORT_COORDINATE_ROTATION
	// Apply coordinate rotation to the final and the centre coordinates
	if (ms.g68Angle != 0.0 && gb.DoingCoordinateRotation())
	{
		const AxesBitmap xAndY = AxesBitmap::MakeFromBits(X_AXIS, Y_AXIS);
		if (axesMentioned.Intersects(xAndY))
		{
			axesMentioned |= xAndY;												// if either X or Y is moving and we are rotating coordinates, both are moving
		}
		float coords[MaxAxes];
		memcpyf(coords, ms.currentUserPosition, MaxAxes);
		RotateCoordinates(ms, ms.g68Angle, coords);
		ToolOffsetTransform(ms, coords, ms.raw.coords, axesMentioned);				// set the final position
		RotateCoordinates(ms, ms.g68Angle, userArcCentre);
		finalTheta -= ms.g68Angle * DegreesToRadians;
		ms.arcCurrentAngle -= ms.g68Angle * DegreesToRadians;
	}
	else
#endif
	{
		ToolOffsetTransform(ms, axesMentioned);									// set the final position
	}

#if SUPPORT_ASYNC_MOVES
	// Check the final position for collisions. We check the intermediate positions as we go.
	collisionChecker.UpdatePositions(ms.raw.coords, axesHomed);
#endif

	if (reprap.GetMove().GetKinematics().LimitPosition(ms.raw.coords, nullptr, numVisibleAxes, axesVirtuallyHomed, true, limitAxes) != LimitPositionResult::ok)
	{
		memcpyf(ms.currentUserPosition, initialUserPosition, numVisibleAxes);
		memcpyf(ms.raw.coords, ms.initialCoords, numVisibleAxes);
		gb.ThrowGCodeException(TargetUnreachableText);							// abandon the move
	}

	// Set up the arc centre coordinates and record which axes behave like an X axis.
	// The I and J parameters are always relative to present position.
	// For X and Y we need to set up the arc centre for each axis that X or Y is mapped to.
	const AxesBitmap axis0Mapping = ms.GetCurrentAxisMapping(axis0);
	const AxesBitmap axis1Mapping = ms.GetCurrentAxisMapping(axis1);
	for (size_t axis = 0; axis < numVisibleAxes; ++axis)
	{
		if (axis0Mapping.IsBitSet(axis))
		{
			ms.arcCentre[axis] = (userArcCentre[0] * axisScaleFactors[axis]) + currentBabyStepOffsets[axis] - Tool::GetOffset(ms.currentTool, axis);
		}
		else if (axis1Mapping.IsBitSet(axis))
		{
			ms.arcCentre[axis] = (userArcCentre[1] * axisScaleFactors[axis]) + currentBabyStepOffsets[axis] - Tool::GetOffset(ms.currentTool, axis);
		}
	}

#if SUPPORT_KEEPOUT_ZONES
	if (keepoutZone.DoesArcIntrude(ms.initialCoords, ms.raw.coords, ms.arcCurrentAngle, finalTheta, ms.arcCentre, ms.arcRadius, axis0Mapping, axis1Mapping, clockwise, wholeCircle))
	{
		memcpyf(ms.currentUserPosition, initialUserPosition, numVisibleAxes);
		memcpyf(ms.raw.coords, ms.initialCoords, numVisibleAxes);
		gb.ThrowGCodeException("arc move would enter keepout zone");
	}
#endif

	try
	{
		LoadFeedrateFromGCode(gb, ms);
	}
	catch (const GCodeException&)
	{
		memcpyf(ms.currentUserPosition, initialUserPosition, numVisibleAxes);			// undo the user position update because this move will not be executed
		throw;
	}

	bool hasExtrusion;
	try
	{
		hasExtrusion = LoadExtrusionFromGCode(gb, ms);
	}
	catch (const GCodeException&)
	{
		// The extruder position restore is safe here because LoadExtrusionFromGCode refreshes moveStartVirtualExtruderPosition before it can throw
		memcpyf(ms.currentUserPosition, initialUserPosition, numVisibleAxes);
		memcpyf(ms.raw.coords, ms.initialCoords, numVisibleAxes);
		ms.latestVirtualExtruderPosition = ms.raw.moveStartVirtualExtruderPosition;
		throw;
	}
	if (ms.IsFirstMoveSincePrintingResumed())
	{
		if (!LockCurrentMovementSystemAndWaitForStandstill(gb))		// update the user position from the machine position
		{
			return false;
		}
		ms.DoneMoveSincePrintingResumed();
		if (hasExtrusion)											// check whether this is the first move after skipping an object and is extruding
		{
			TravelToStartPoint(gb, ms);								// don't start a printing move from the wrong point
			return false;
		}
	}

	if (ms.raw.hasPositiveExtrusion)
	{
		//TODO ideally we should calculate the min and max X and Y coordinates of the entire arc here and call UpdateObjectCoordinates twice.
		// But it is currently rare to use G2/G3 with extrusion, so for now we don't bother.
		buildObjects.UpdateObjectCoordinates(ms.currentObjectNumber, ms.currentUserPosition, AxesBitmap::MakeLowestNBits(2));
	}

#if SUPPORT_LASER
	if (machineType == MachineType::laser)
	{
		if (gb.Seen('S'))
		{
			ms.laserPixelData.pixelPwm[0] = ConvertLaserPwm(gb.GetFValue());
			ms.laserPixelData.numPixels = 1;
		}
		else if (laserPowerSticky)
		{
			// leave the laser PWM alone because this is what LaserWeb expects
		}
		else
		{
			ms.laserPixelData.Clear();
		}
	}
#endif
#if SUPPORT_LASER && SUPPORT_IOBITS
	else
#endif
#if SUPPORT_IOBITS
	{
		// Update the iobits parameter
		if (gb.Seen('P'))
		{
			ms.raw.laserPwmOrIoBits.ioBits = (IoBits_t)gb.GetIValue();
		}
		else
		{
			// Leave moveBuffer.ioBits alone so that we keep the previous value
		}
	}
#endif

	ms.raw.usePressureAdvance = ms.raw.hasPositiveExtrusion;

	// Calculate the total angle moved, which depends on which way round we are going
	float totalArc;
	if (wholeCircle)
	{
		totalArc = TwoPi;
	}
	else
	{
		totalArc = (clockwise) ? ms.arcCurrentAngle - finalTheta : finalTheta - ms.arcCurrentAngle;
		if (totalArc < 0.0)
		{
			totalArc += TwoPi;
		}
	}

	// Compute how many segments to use
	// For the arc to deviate up to MaxArcDeviation from the ideal, the segment length should be sqrtf(8 * arcRadius * MaxArcDeviation + fsquare(MaxArcDeviation))
	// We leave out the square term because it is very small
	// In CNC applications even very small deviations can be visible, so we use a smaller segment length at low speeds
	const float arcSegmentLength = constrain<float>
									(	min<float>(fastSqrtf(8 * ms.arcRadius * MaxArcDeviation), ms.raw.feedRate * StepClockRate * (1.0/MaxArcSegmentsPerSec)),
										MinArcSegmentLength,
										MaxArcSegmentLength
									);
	ms.totalSegments = max<unsigned int>((unsigned int)((ms.arcRadius * totalArc)/arcSegmentLength + 0.8), 1u);
	ms.arcAngleIncrement = totalArc/ms.totalSegments;
	if (clockwise)
	{
		ms.arcAngleIncrement = -ms.arcAngleIncrement;
	}
	ms.angleIncrementSine = sinf(ms.arcAngleIncrement);
	ms.angleIncrementCosine = cosf(ms.arcAngleIncrement);
	ms.segmentsTillNextFullCalc = 0;

	ms.arcAxis0 = axis0;
	ms.arcAxis1 = axis1;
	ms.doingArcMove = true;
	ms.xyPlane = (selectedPlane == 0);
	FinaliseMove(gb, ms);
	UnlockAll(gb);			// allow pause
//	debugPrintf("Radius %.2f, initial angle %.1f, increment %.1f, segments %u\n",
//				arcRadius, arcCurrentAngle * RadiansToDegrees, arcAngleIncrement * RadiansToDegrees, segmentsLeft);
	return true;
}

// Adjust the move parameters to account for segmentation and/or part of the move having been done already
void GCodes::FinaliseMove(GCodeBuffer& gb, MovementState& ms) noexcept
{
	ms.raw.canPauseAfter = !ms.raw.checkEndstops && !ms.doingArcMove;		// pausing during an arc move isn't safe because the arc centre get recomputed incorrectly when we resume
	ms.raw.filePos = gb.GetJobFilePosition();
	ms.raw.gCommandNumber = (int8_t)gb.GetCommandNumber();			// gb is at a G0/G1/G2/G3 command here, so remember it for the modal context
	gb.MotionCommanded();

	if (ms.IsCurrentObjectCancelled())
	{
#if SUPPORT_LASER
		if (machineType == MachineType::laser)
		{
			platform.SetLaserPwm(0);
		}
#endif
	}
	else
	{
		if (ms.totalSegments > 1)
		{
			ms.segMoveState = SegmentedMoveState::active;
			gb.SetState(GCodeState::waitingForSegmentedMoveToGo);

			for (size_t extruder = 0; extruder < numExtruders; ++extruder)
			{
				ms.raw.coords[ExtruderToLogicalDrive(extruder)] /= ms.totalSegments;	// change the extrusion to extrusion per segment
			}

			if (ms.moveFractionToSkip != 0.0)
			{
				const float fseg = floorf(ms.totalSegments * ms.moveFractionToSkip);	// round down to the start of a move
				ms.segmentsLeftToStartAt = ms.totalSegments - (unsigned int)fseg;
				ms.firstSegmentFractionToSkip = (ms.moveFractionToSkip * ms.totalSegments) - fseg;
				NewMoveAvailable(ms);
				return;
			}
		}
		else
		{
			ms.segMoveState = SegmentedMoveState::inactive;
		}

		ms.segmentsLeftToStartAt = ms.totalSegments;
		ms.firstSegmentFractionToSkip = ms.moveFractionToSkip;

		NewMoveAvailable(ms);
	}
}

// Set up a move to travel to the resume point. Return true if successful, false if needs to be called again.
// This is only called if the first move commanded since changing from a cancelled object to a non-cancelled object or to no object involves extrusion.
// By the time this is called, the user position has been overwritten with the final position of the pending move, so we can't use it.
// But the expected position was saved in the resume object restore point when the state changed from printing a cancelled object to printing a live object.
void GCodes::TravelToStartPoint(GCodeBuffer& gb, MovementState& ms) noexcept
{
	ms.SetDefaults(numTotalAxes);
	SetMoveBufferDefaults(ms);
	ToolOffsetTransform(ms);
	const RestorePoint& rp = ms.GetResumeObjectRestorePoint();
	ToolOffsetTransform(ms, rp.moveCoords, ms.raw.coords);
	ms.raw.originalFeedRate = (float16_t)rp.originalFeedRate;
	ms.raw.feedRate = gb.ConvertSpeed(rp.originalFeedRate, true);
	ms.raw.movementTool = ms.currentTool;
	ms.raw.linearAxesMentioned = ms.raw.rotationalAxesMentioned = true;			// assume that both linear and rotational axes might be moving
#if SUPPORT_ASYNC_MOVES
	ms.AllocateAxes(rp.axesAndExtrudersOwned, ParameterLettersBitmap());
#endif
	NewSegmentableMoveAvailable(ms);
}

// The Move class calls this function to find what to do next. It takes its own copy of the move because it adjusts the coordinates.
// Returns true if a new move was copied to 'm'.
bool GCodes::ReadMove(MovementSystemNumber queueNumber, RawMove& m) noexcept
{
	MovementState& ms = moveStates[queueNumber];
	if (ms.segmentsLeft == 0)
	{
		return false;
	}

	while (true)		// loop while we skip move segments
	{
#if SUPPORT_LASER
		// If it's a straight move in laser mode, sort out the laser power
		if (machineType == MachineType::laser)
		{
			ms.raw.laserPwmOrIoBits.laserPwm = (ms.raw.isCoordinated && ms.segmentsLeft < ms.laserPixelData.numPixels && !ms.doingArcMove)
											? ms.laserPixelData.pixelPwm[ms.laserPixelData.numPixels - ms.segmentsLeft]
												: (ms.raw.isCoordinated && ms.laserPixelData.numPixels == 1)
												  ? ms.laserPixelData.pixelPwm[0]
													: 0u;
		}
#endif
		m = ms.raw;

		if (ms.segmentsLeft == 1)
		{
			// If there is just 1 segment left, it doesn't matter if it is an arc move or not, just move to the end position
			if (ms.segmentsLeftToStartAt == 1 && ms.firstSegmentFractionToSkip != 0.0)	// if this is the segment we are starting at and we need to skip some of it
			{
				// Reduce the extrusion by the amount to be skipped
				for (size_t extruder = 0; extruder < numExtruders; ++extruder)
				{
					m.coords[ExtruderToLogicalDrive(extruder)] *= (1.0 - ms.firstSegmentFractionToSkip);
				}
			}
			m.proportionDone = 1.0;
			if (ms.doingArcMove)
			{
				m.canPauseAfter = true;									// we can pause after the final segment of an arc move
			}
			ms.ClearMove();
		}
		else
		{
			// This move needs to be divided into 2 or more segments
			// Do the axes
			AxesBitmap axisMap0, axisMap1;
			if (ms.doingArcMove)
			{
				ms.arcCurrentAngle += ms.arcAngleIncrement;
				if (ms.segmentsTillNextFullCalc == 0)
				{
					// Do the full calculation
					ms.segmentsTillNextFullCalc = SegmentsPerFulArcCalculation;
					ms.currentAngleCosine = cosf(ms.arcCurrentAngle);
					ms.currentAngleSine = sinf(ms.arcCurrentAngle);
				}
				else
				{
					// Speed up the computation by doing two multiplications and an addition or subtraction instead of a sine or cosine
					--ms.segmentsTillNextFullCalc;
					const float newCosine = ms.currentAngleCosine * ms.angleIncrementCosine - ms.currentAngleSine   * ms.angleIncrementSine;
					const float newSine   = ms.currentAngleSine   * ms.angleIncrementCosine + ms.currentAngleCosine * ms.angleIncrementSine;
					ms.currentAngleCosine = newCosine;
					ms.currentAngleSine = newSine;
				}
				axisMap0 = Tool::GetAxisMapping(ms.raw.movementTool, ms.arcAxis0);
				axisMap1 = Tool::GetAxisMapping(ms.raw.movementTool, ms.arcAxis1);
#if 0	// we don't use this yet
				ms.cosXyAngle = (ms.xyPlane) ? ms.angleIncrementCosine : 1.0;
#endif
			}

			for (size_t drive = 0; drive < numVisibleAxes; ++drive)
			{
				float newCoordinate;
				if (axisMap1.IsBitSet(drive))
				{
					// Axis1 or a substitute in the selected arc plane
					newCoordinate = ms.arcCentre[drive] + ms.arcRadius * axisScaleFactors[drive] * ms.currentAngleSine;
				}
				else if (axisMap0.IsBitSet(drive))
				{
					// Axis0 or a substitute in the selected arc plane
					newCoordinate = ms.arcCentre[drive] + ms.arcRadius * axisScaleFactors[drive] * ms.currentAngleCosine;
				}
				else
				{
					// This axis is not moving in an arc
					const float movementToDo = (ms.raw.coords[drive] - ms.initialCoords[drive])/ms.segmentsLeft;
					newCoordinate = ms.initialCoords[drive] += movementToDo;
				}
				m.coords[drive] = ms.initialCoords[drive] = newCoordinate;
			}

			if (ms.segmentsLeftToStartAt < ms.segmentsLeft)
			{
				// We are resuming a print part way through a move and we printed this segment already
				--ms.segmentsLeft;
				continue;
			}

			// Limit the end position at each segment. This is needed for arc moves on any printer, and for [segmented] straight moves on SCARA printers.
			if (   reprap.GetMove().GetKinematics().LimitPosition(m.coords, nullptr, numVisibleAxes, axesVirtuallyHomed, true, limitAxes) != LimitPositionResult::ok
#if SUPPORT_ASYNC_MOVES
				|| !collisionChecker.UpdatePositions(m.coords, axesHomed)
#endif
			   )
			{
				ms.segMoveState = SegmentedMoveState::aborted;
				ms.doingArcMove = false;
				ms.segmentsLeft = 0;
				return false;
			}

			if (ms.segmentsLeftToStartAt == ms.segmentsLeft && ms.firstSegmentFractionToSkip != 0.0)	// if this is the segment we are starting at and we need to skip some of it
			{
				// Reduce the extrusion by the amount to be skipped
				for (size_t extruder = 0; extruder < numExtruders; ++extruder)
				{
					m.coords[ExtruderToLogicalDrive(extruder)] *= (1.0 - ms.firstSegmentFractionToSkip);
				}
			}
			--ms.segmentsLeft;

			m.proportionDone = ms.GetProportionDone();
		}

		return true;
	}
}

// Flag that a new single-segment move is available for consumption by the Move subsystem
// Code that sets up a new move should ensure that segmentsLeft is zero, then set up all the move parameters,
// then call this function to update SegmentsLeft safely in a multi-threaded environment
void GCodes::NewSingleSegmentMoveAvailable(MovementState& ms) noexcept
{
	ms.segmentsLeftToStartAt = ms.totalSegments = 1;
	NewMoveAvailable(ms);
}

// Flag that a new move that should be segmented is available for consumption by the Move subsystem
// Code that sets up a new move should ensure that segmentsLeft is zero, then set up all the move parameters,
// then call this function to update SegmentsLeft safely in a multi-threaded environment
void GCodes::NewSegmentableMoveAvailable(MovementState& ms) noexcept
{
	const Kinematics &_ecv_from kin = reprap.GetMove().GetKinematics();
	const SegmentationType st = kin.GetSegmentationType();
	if (st.useSegmentation)
	{
		// This kinematics approximates linear motion by means of segmentation
		// To establish the effective XY movement distance, pick one X and one Y axis
		const size_t effectiveXAxis = ms.GetCurrentXAxes().LowestSetBit();
		const size_t effectiveYAxis = ms.GetCurrentYAxes().LowestSetBit();
		float moveLengthSquared = fsquare(ms.raw.coords[effectiveXAxis] - ms.initialCoords[effectiveXAxis]) + fsquare(ms.raw.coords[effectiveYAxis] - ms.initialCoords[effectiveYAxis]);
		if (st.useZSegmentation)
		{
			const size_t effectiveZAxis = ms.GetCurrentZAxes().LowestSetBit();
			moveLengthSquared += fsquare(ms.raw.coords[effectiveZAxis] - ms.initialCoords[effectiveZAxis]);
		}
		const float moveLength = fastSqrtf(moveLengthSquared);
		const float moveTime = moveLength/(ms.raw.feedRate * StepClockRate);	// this is a best-case time, often the move will take longer
		ms.totalSegments = (unsigned int)max<long>(1, lrintf(min<float>(moveLength * kin.GetReciprocalMinSegmentLength(), moveTime * kin.GetSegmentsPerSecond())));
	}
	else
	{
		ms.totalSegments = 1;
	}
	ms.segmentsLeftToStartAt = ms.totalSegments;
	NewMoveAvailable(ms);
}

// Flag that a new move is available for consumption by the Move subsystem
// This version is for when totalSegments has already be set up.
void GCodes::NewMoveAvailable(MovementState& ms) noexcept
{
	if (ms.raw.checkEndstops || ms.raw.moveType != 0)
	{
		ms.positionMayBeInaccurate = true;		// this move can stop short of its commanded target, so re-read the position at the next standstill
	}
	const unsigned int sl = ms.totalSegments;
	__DMB();									// make sure that the move details have been written first
	ms.segmentsLeft = sl;						// set the number of segments to indicate that a move is available to be taken
	reprap.GetMove().MoveAvailable();			// notify the Move task that we have a move
}

// Cancel any macro or print in progress
void GCodes::AbortPrint(GCodeBuffer& gb) noexcept
{
	PauseSequenceAborted(gb);					// if we are aborting a pause sequence before it committed, settle the pause first
	AbortStateMachine(gb);						// clean up state machine side effects at all stack levels before unwinding
	(void)gb.AbortFile(true);					// stop executing any files or macros that this GCodeBuffer is running
	if (gb.IsFileChannel())						// if the current command came from a file being printed
	{
#if SUPPORT_ASYNC_MOVES
		GCodeBuffer* otherGb = (gb.GetChannel() == GCodeChannel::File) ? _ecv_not_null(File2GCode()) : _ecv_not_null(FileGCode());
		if (otherGb->IsDoingFile() && (!otherGb->IsDoingFileMacro() || otherGb->LatestMachineState().CanRestartMacro()))
		{
			AbortStateMachine(*otherGb);		// clean up state machine side effects for the other reader too
			(void)otherGb->AbortFile(true);		// stop processing commands from the other file reader too
		}
#endif
		StopPrint(nullptr, StopPrintReason::abort);
		gb.Init();								// invalidate the file channel here as the other one may be still busy (possibly in a macro)
#if SUPPORT_ASYNC_MOVES
		otherGb->Init();						// also reset the other file reader to clear sync state and buffer state
#endif
	}
}

// When a pause sequence (pause.g / filament-change.g) is aborted, the stack frame that would have advanced pauseState from
// pausing to paused is discarded with the rest of the aborted frames, leaving the machine stuck reporting "pausing" forever.
// If this GCodeBuffer still holds such a frame while pauseState is pausing, commit the pause now so it settles to paused (resumable)
// instead of hanging. The file-channel path in AbortPrint runs StopPrint afterwards, which resets to notPaused as before
void GCodes::PauseSequenceAborted(GCodeBuffer& gb) noexcept
{
	if (pauseState != PauseState::pausing)
	{
		return;
	}

	for (const GCodeMachineState *_ecv_null ms = &gb.LatestMachineState(); ms != nullptr; ms = ms->GetPrevious())
	{
		const GCodeState state = ms->GetState();
		if (state == GCodeState::pausing2 || state == GCodeState::eventPausing2 || state == GCodeState::filamentChangePause2)
		{
			pauseState = PauseState::paused;
			reprap.StateUpdated();
			return;
		}
	}
}

// Simplified version of DoFileMacro, see below
bool GCodes::DoFileMacro(GCodeBuffer& gb, const char *_ecv_array fileName, bool reportMissing, int codeRunning) noexcept
{
	VariableSet emptyVars;
	return DoFileMacro(gb, fileName, reportMissing, codeRunning, emptyVars);
}

bool GCodes::DoFileMacroWithParameters(GCodeBuffer& gb, const char *_ecv_array fileName, bool reportMissing, int codeRunning) THROWS(GCodeException)
{
	VariableSet vars;
	gb.AddParameters(vars, codeRunning);
	return DoFileMacro(gb, fileName, reportMissing, codeRunning, vars);
}

// Run a file macro. Prior to calling this, 'state' must be set to the state we want to enter when the macro has been completed.
// Return true if the file was found or it wasn't and we were asked to report that fact. If the file wasn't found and we were not asked to report that, return false.
// 'codeRunning' is the G or M command we are running, or 0 for a tool change file. In particular:
// 501 = running M501
// 502 = running M502
// 98 = running a macro explicitly via M98
// otherwise it is either the G- or M-code being executed, or ToolChangeMacroCode for a tool change file, or SystemMacroCode for another system file
bool GCodes::DoFileMacro(GCodeBuffer& gb, const char *_ecv_array fileName, bool reportMissing, int codeRunning, VariableSet& initialVariables) noexcept
{
	if (   codeRunning != AsyncSystemMacroCode
		&& gb.IsFileChannel()
		&& gb.LatestMachineState().GetPrevious() == nullptr)
	{
		// This macro was invoked directly from the print file by M98, G28, G29, G32 etc. so record the file location of that command so that we can restart it
		gb.SavePrintingFilePosition();
	}

#if HAS_SBC_INTERFACE
	if (reprap.UsingSbcInterface())
	{
		if (!gb.RequestMacroFile(fileName, gb.LatestMachineState().lastCodeFromSbc && codeRunning != AsyncSystemMacroCode))
		{
			if (reportMissing)
			{
				MessageType mt = (gb.LatestMachineState().lastCodeFromSbc && codeRunning != SystemHelperMacroCode)
									? (MessageType)(gb.GetResponseMessageType() | WarningMessageFlag | PushFlag)
										: WarningMessage;
				platform.MessageF(mt, "Macro file %s not found\n", fileName);
				return true;
			}

			return false;
		}

		if (!Push(gb, false))
		{
			gb.AbortFile(false);
			return true;
		}
		gb.GetVariables().AssignFrom(initialVariables);
		gb.StartNewFile(fileName);
		if (gb.IsMacroEmpty())
		{
			gb.SetFileFinished();
		}
	}
	else
#endif
	{
#if HAS_MASS_STORAGE || HAS_EMBEDDED_FILES
		FileStore *_ecv_null const f = platform.OpenSysFile(fileName, OpenMode::read);
		if (f == nullptr)
		{
			if (reportMissing)
			{
				platform.MessageF(WarningMessage, "Macro file %s not found\n", fileName);
				return true;
			}
			return false;
		}

		if (!Push(gb, false))
		{
			f->Close();
			return true;
		}
		gb.GetVariables().AssignFrom(initialVariables);
		gb.LatestMachineState().fileState.Set(f);
		gb.StartNewFile(fileName);
		gb.GetFileInput()->Reset(gb.LatestMachineState().fileState);
#else
		if (reportMissing)
		{
			platform.MessageF(WarningMessage, "Macro file %s not found\n", fileName);
		}
		return reportMissing;
#endif
	}

#if HAS_SBC_INTERFACE || HAS_MASS_STORAGE || HAS_EMBEDDED_FILES
	gb.LatestMachineState().doingFileMacro = true;

	// The following three flags need to be inherited in the case that a system macro calls another macro, e.g.homeall.g calls homez.g. The Push call copied them over already.
	switch (codeRunning)
	{
	case 501:
		gb.LatestMachineState().runningM501 = true;
		gb.LatestMachineState().runningSystemMacro = true;			// running a system macro e.g. homing, so don't use workplace coordinates
		break;

	case 502:
		gb.LatestMachineState().runningM502 = true;
		gb.LatestMachineState().runningSystemMacro = true;			// running a system macro e.g. homing, so don't use workplace coordinates
		break;

	case SystemHelperMacroCode:
	case AsyncSystemMacroCode:
	case ToolChangeMacroCode:
	case 29:
	case 32:
		gb.LatestMachineState().runningSystemMacro = true;			// running a system macro e.g. homing, so don't use workplace coordinates
		break;

	default:
		break;
	}

	gb.SetState(GCodeState::normal);
	gb.Init();

# if HAS_SBC_INTERFACE
	if (!reprap.UsingSbcInterface() && codeRunning != AsyncSystemMacroCode)
# endif
	{
		// Don't notify DSF when files are requested asynchronously, it creates excessive traffic
		reprap.InputsUpdated();
	}
	return true;
#endif
}

// Return true if the macro being executed by fileGCode was restarted
bool GCodes::GetMacroRestarted() const noexcept
{
	const GCodeMachineState& ms = FileGCode()->LatestMachineState();
	return ms.doingFileMacro && ms.GetPrevious() != nullptr && ms.GetPrevious()->firstCommandAfterRestart;
}

void GCodes::FileMacroCyclesReturn(GCodeBuffer& gb) noexcept
{
	//const int retValue = (gb.Seen('P')) ? gb.GetIValue() : 0;		// for when we allow M99 to return 'result'
	if (gb.IsDoingFileMacro())
	{
#if HAS_SBC_INTERFACE
		if (reprap.UsingSbcInterface())
		{
			gb.AbortFile(false);
		}
		else
#endif
		{
#if HAS_MASS_STORAGE || HAS_EMBEDDED_FILES
			FileData &file = gb.LatestMachineState().fileState;
			gb.GetFileInput()->Reset(file);
			file.Close();

			gb.PopState(false);
#endif
		}

		gb.Init();
	}
}

// Home one or more of the axes. Taking the movement lock and syncing has already been done.
// 'reply' is only written if there is an error.
GCodeResult GCodes::DoHome(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	// We have the movement lock so we have exclusive access to the homing flags
	if (toBeHomed.IsNonEmpty())
	{
		reply.copy("G28 may not be used within a homing file");
		return GCodeResult::error;
	}

	// Find out which axes we have been asked to home
	for (size_t axis = 0; axis < numTotalAxes; ++axis)
	{
		if (gb.Seen(axisLetters[axis]))
		{
			toBeHomed.SetBit(axis);
			SetAxisNotHomed(axis);
		}
	}

	if (toBeHomed.IsEmpty())
	{
		SetAllAxesNotHomed();		// homing everything
		toBeHomed = AxesBitmap::MakeLowestNBits(numVisibleAxes);
	}

	gb.SetState(GCodeState::homing1);
	return GCodeResult::ok;
}

// Return the current coordinates as a printable string. Used only to implement M114.
// Coordinates are updated at the end of each movement, so this won't tell you where you are mid-movement.
// We only report the coordinates of the current movement system.
void GCodes::HandleM114(GCodeBuffer& gb, const StringRef& s) const noexcept
{
	const MovementState& ms = GetConstMovementState(gb);

	// Start with the axis coordinates
	s.Clear();
	for (size_t axis = 0; axis < numVisibleAxes; ++axis)
	{
		// Don't put a space after the colon in the response, it confuses Pronterface
		s.catf("%c:%.3f ", axisLetters[axis], (double)HideNan(GetUserCoordinate(ms, axis)));
	}

	// Now the virtual extruder position, for Octoprint
	s.catf("E:%.3f ", (double)ms.latestVirtualExtruderPosition);

	// Now the extruder coordinates
	for (size_t i = 0; i < numExtruders; i++)
	{
		s.catf("E%u:%.1f ", i, (double)ms.LiveMachineCoordinate(ExtruderToLogicalDrive(i)));
	}

	// Print the axis stepper motor positions as Marlin does, as an aid to debugging.
	// Don't bother with the extruder endpoints, they are zero after any non-extruding move.
	s.cat("Count");
	for (size_t i = 0; i < numVisibleAxes; ++i)
	{
		s.catf(" %" PRIi32, reprap.GetMove().GetLiveMotorPosition(i));
	}

	// Add the machine coordinates because they may be different from the user coordinates under some conditions
	s.cat(" Machine");
	float machineCoordinates[MaxAxes];
	ToolOffsetTransform(ms, ms.currentUserPosition, machineCoordinates);
	for (size_t axis = 0; axis < numVisibleAxes; ++axis)
	{
		s.catf(" %.3f", (double)HideNan(machineCoordinates[axis]));
	}

	// Add the bed compensation
	const float machineZ = machineCoordinates[Z_AXIS];
	reprap.GetMove().AxisAndBedTransform(machineCoordinates, ms.currentTool, true);
	s.catf(" Bed comp %.3f", (double)(machineCoordinates[Z_AXIS] - machineZ));
}

#if HAS_MASS_STORAGE || HAS_EMBEDDED_FILES
// Set up a file to print, but don't print it yet.
// If successful return true, else write an error message to reply and return false
bool GCodes::QueueFileToPrint(const char *_ecv_array fileName, const StringRef& reply) noexcept
{
	FileStore *_ecv_null const f = platform.OpenFile(Platform::GetGCodeDir(), fileName, OpenMode::read);
	if (f != nullptr)
	{
		fileToPrint.Set(f);
		return true;
	}

	reply.printf("GCode file \"%s\" not found\n", fileName);
	return false;
}
#endif

// Start printing the file already selected.
// We must hold the movement lock and wait for all moves to finish before calling this because of the calls to ResetMoveCounters and ResetExtruderPositions.
void GCodes::StartPrinting(bool fromStart) noexcept
{
	buildObjects.Init();
	for (MovementState& ms : moveStates)
	{
		ms.InitObjectCancellation();
	}

	reprap.GetMove().ResetMoveCounters();

	if (fromStart)															// if not resurrecting a print
	{
		for (MovementState& ms : moveStates)
		{
			ms.fileOffsetToPrint = 0;
# if SUPPORT_ASYNC_MOVES
			ms.fileOffsetToSkipTo = 0;
# endif
			ms.restartMoveFractionDone = 0.0;
			ms.restartGCommandNumber = -1;
			ms.latestVirtualExtruderPosition = ms.raw.moveStartVirtualExtruderPosition = 0.0;
		}

		FileGCode()->LatestMachineState().volumetricExtrusion = false;		// default to non-volumetric extrusion
		FileGCode()->LatestMachineState().selectedPlane = 0;				// default G2 and G3 moves to XY plane
		FileGCode()->LatestMachineState().inverseTimeMode = false;			// default to standard feedrate mode
#if SUPPORT_ASYNC_MOVES
		FileGCode()->ExecuteAll();											// execute commands for all movement systems initially
#endif
	}

#if HAS_MASS_STORAGE
# if HAS_SBC_INTERFACE
	if (!reprap.UsingSbcInterface())
# endif
	{
		fileToPrint.Seek(moveStates[0].fileOffsetToPrint);
# if SUPPORT_ASYNC_MOVES
		if (!FileGCode()->ExecutingAll())
		{
			// Running M23 when the File reader has already been forked, probably by the resume-after-power-fail code
			//TODO
		}
# endif
	}
#endif

	for (size_t extruder = 0; extruder < MaxExtruders; extruder++)
	{
		rawExtruderTotalByDrive[extruder] = 0.0;
	}
	rawExtruderTotal = 0.0;
	reprap.GetMove().ResetExtruderPositions();

#if HAS_SBC_INTERFACE
	if (!reprap.UsingSbcInterface())
#endif
	{
#if HAS_MASS_STORAGE || HAS_EMBEDDED_FILES
		FileGCode()->OriginalMachineState().fileState.MoveFrom(fileToPrint);
		FileGCode()->GetFileInput()->Reset(FileGCode()->OriginalMachineState().fileState);
#endif
	}

	reprap.GetPrintMonitor().StartedPrint();
	const char *_ecv_array _ecv_null const printingFilename = reprap.GetPrintMonitor().GetPrintingFilename();
	FileGCode()->StartNewFile(printingFilename);
	platform.MessageF(LogWarn, (IsSimulating()) ? "Started simulating printing file %s\n" : "Started printing file %s\n", printingFilename);
	if (fromStart)
	{
		DoFileMacro(*FileGCode(), START_G, false, AsyncSystemMacroCode);		// get fileGCode to execute the start macro so that any M82/M83 codes will be executed in the correct context
	}
	else
	{
		FileGCode()->LatestMachineState().firstCommandAfterRestart = true;
#if SUPPORT_ASYNC_MOVES
		File2GCode()->LatestMachineState().firstCommandAfterRestart = true;
#endif
	}
}

// Function to handle dwell delays. Returns true for dwell finished, false otherwise.
GCodeResult GCodes::DoDwell(GCodeBuffer& gb) THROWS(GCodeException)
{
	// Wait for all the queued moves to stop. Only do this if motion has been commanded from this GCode stream since we last waited for motion to stop.
	// This is so that G4 can be used in a trigger or daemon macro file without pausing motion, when the macro doesn't itself command any motion.
	if (gb.WasMotionCommanded())
	{
		if (!LockCurrentMovementSystemAndWaitForStandstill(gb))
		{
			return GCodeResult::notFinished;
		}
	}

	UnlockAll(gb);																	// don't hang on to the movement lock while we delay

	const int32_t dwell = (gb.Seen('S')) ? (int32_t)(gb.GetFValue() * 1000.0)		// S values are in seconds
							: (gb.Seen('P')) ? gb.GetIValue()						// P value are in milliseconds
								: 0;
	if (dwell <= 0)
	{
		return GCodeResult::ok;
	}

	if (   IsSimulating()															// if we are simulating then simulate the G4...
		&& &gb != DaemonGCode()														// ...unless it comes from the daemon...
		&& &gb != TriggerGCode()													// ...or a trigger...
		&& (gb.IsFileChannel() || !exitSimulationWhenFileComplete)					// ...or we are simulating a file and this command doesn't come from the file
	   )
	{
		simulationTime += (float)dwell * 0.001;
		return GCodeResult::ok;
	}

	return (gb.DoDwellTime((uint32_t)dwell)) ? GCodeResult::ok : GCodeResult::notFinished;
}

// Get the tool specified by the P parameter, or the current tool if no P parameter
ReadLockedPointer<Tool> GCodes::GetSpecifiedOrCurrentTool(GCodeBuffer& gb) THROWS(GCodeException)
{
	int tNumber;
	if (gb.Seen('P'))
	{
		tNumber = (int)gb.GetUIValue();
	}
	else
	{
		tNumber = GetMovementState(gb).GetCurrentToolNumber();
		if (tNumber < 0)
		{
			gb.ThrowGCodeException("No tool number given and no current tool");
		}
	}

	ReadLockedPointer<Tool> tool = Tool::GetLockedTool(tNumber);
	if (tool.IsNull())
	{
		gb.ThrowGCodeException("Invalid tool number");
	}
	return tool;
}

// Set offset, working and standby temperatures for a tool. i.e. handle a G10 or M568.
GCodeResult GCodes::SetOrReportOffsets(GCodeBuffer &gb, const StringRef& reply, int code) THROWS(GCodeException)
{
	ReadLockedPointer<Tool> const tool = GetSpecifiedOrCurrentTool(gb);
	bool settingOffset = false;
	if (code == 10)			// Only G10 can set tool offsets
	{
		for (size_t axis = 0; axis < numVisibleAxes; ++axis)
		{
			if (gb.Seen(axisLetters[axis]))
			{
				if (!LockMovement(gb))
				{
					return GCodeResult::notFinished;
				}
				settingOffset = true;
				tool->SetOffset(axis, gb.GetFValue(), gb.LatestMachineState().runningM501);
			}
		}

		if (settingOffset)
		{
			ToolOffsetInverseTransform(GetMovementState(gb));	// update user coordinates to reflect the new tool offset, in case we have this tool selected
		}
	}

	// Deal with setting temperatures
	bool settingTemps = false;
	size_t hCount = tool->HeaterCount();
	if (hCount > 0)
	{
		if (gb.Seen('R'))
		{
			settingTemps = true;
			if (!IsSimulating())
			{
				float standby[MaxHeaters];
				gb.GetFloatArray(standby, hCount, true);
				for (size_t h = 0; h < hCount; ++h)
				{
					tool->SetToolHeaterStandbyTemperature(h, standby[h]);
				}
			}
		}
		if (gb.Seen('S'))
		{
			settingTemps = true;
			if (!IsSimulating())
			{
				float activeTemps[MaxHeaters];
				gb.GetFloatArray(activeTemps, hCount, true);
				for (size_t h = 0; h < hCount; ++h)
				{
					tool->SetToolHeaterActiveTemperature(h, activeTemps[h]);		// may throw
				}
			}
		}
	}

	bool settingOther = false;
	if (code == 568)					// Only M568 can set spindle RPM and change tool heater states
	{
		// Deal with spindle RPM
		if (tool->GetSpindleNumber() > -1)
		{
			if (gb.Seen('F'))
			{
				settingOther = true;
				if (!IsSimulating())
				{
					tool->SetSpindleRpm(gb, gb.GetUIValue(), GetMovementState(gb).currentTool == tool.Ptr());
				}
			}
		}

		// Deal with tool heater states
		uint32_t newHeaterState;
		if (gb.TryGetLimitedUIValue('A', newHeaterState, settingOther, 3))
		{
			switch (newHeaterState)
			{
			case 0:			// turn heaters off
				tool->HeatersToOff();
				break;

			case 1:			// turn heaters to standby, except any that are used by a different active tool
				tool->HeatersToActiveOrStandby(false);
				break;

			case 2:			// set heaters to their active temperatures, except any that are used by a different active tool
				tool->HeatersToActiveOrStandby(true);
				break;
			}
		}
	}

	if (!settingOffset && !settingTemps && !settingOther)
	{
		// Print offsets and temperatures
		reply.printf("Tool %d", tool->Number());
		char c;

		// Print the tool offsets if we are executing G10
		if (code == 10)
		{
			reply.cat(": offsets");
			for (size_t axis = 0; axis < numVisibleAxes; ++axis)
			{
				reply.catf(" %c%.3f", axisLetters[axis], (double)tool->GetOffset(axis));
			}
			c = ',';
		}
		else
		{
			c = ':';
		}

		// Print the heater active/standby temperatures whichever code we are executing
		if (hCount != 0)
		{
			reply.catf("%c active/standby temperature(s)", c);
			c = ',';
			for (size_t heater = 0; heater < hCount; heater++)
			{
				reply.catf(" %.1f/%.1f", (double)tool->GetToolHeaterActiveTemperature(heater), (double)tool->GetToolHeaterStandbyTemperature(heater));
			}
		}

		// Print the spindle number if we are executing M568
		if (code == 568 && tool->GetSpindleNumber() > -1)
		{
			reply.catf("%c spindle %d@%" PRIu32 "rpm", c, tool->GetSpindleNumber(), tool->GetSpindleRpm());
		}
	}
	else
	{
#if 0 // Do not warn about deprecation for now
		if (code == 10 && settingTemps)
		{
			reply.lcat("This use of G10 is deprecated. Please use M568 to set tool temperatures.");
			return GCodeResult::warning;
		}
#endif
		String<StringLengthLoggedCommand> scratch;
		gb.AppendFullCommand(scratch.GetRef());
		platform.Message(MessageType::LogInfo, scratch.c_str());
	}

	return GCodeResult::ok;
}

// Create a new tool definition
GCodeResult GCodes::ManageTool(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	// Check tool number
	const unsigned int toolNumber = gb.GetLimitedUIValue('P', MaxTools);

	bool seen = false;

	// Check tool name
	String<MaxToolNameLength> name;
	if (gb.Seen('S'))
	{
		gb.GetQuotedString(name.GetRef());
		seen = true;
	}

	// Check drives
	int32_t drives[MaxExtrudersPerTool];
	size_t dCount = MaxExtrudersPerTool;	// Sets the limit and returns the count
	if (gb.Seen('D'))
	{
		gb.GetIntArray(drives, dCount, false);
		seen = true;
	}
	else
	{
		dCount = 0;
	}

	// Check heaters
	int32_t heaters[MaxHeatersPerTool];
	size_t hCount = MaxHeatersPerTool;
	if (gb.Seen('H'))
	{
		gb.GetIntArray(heaters, hCount, false);
		seen = true;
	}
	else
	{
		hCount = 0;
	}

	// Check X axis mapping
	AxesBitmap xMap;
	if (gb.Seen('X'))
	{
		uint32_t xMapping[MaxAxes];
		size_t xCount = numVisibleAxes;
		gb.GetUnsignedArray(xMapping, xCount, false);
		xMap = AxesBitmap::MakeFromArray(xMapping, xCount) & AxesBitmap::MakeLowestNBits(numVisibleAxes);
		seen = true;
	}
	else
	{
		xMap = DefaultXAxisMapping;					// by default map X axis straight through
	}

	// Check Y axis mapping
	AxesBitmap yMap;
	if (gb.Seen('Y'))
	{
		uint32_t yMapping[MaxAxes];
		size_t yCount = numVisibleAxes;
		gb.GetUnsignedArray(yMapping, yCount, false);
		yMap = AxesBitmap::MakeFromArray(yMapping, yCount) & AxesBitmap::MakeLowestNBits(numVisibleAxes);
		seen = true;
	}
	else
	{
		yMap = DefaultYAxisMapping;					// by default map X axis straight through
	}

	// Check Z axis mapping
	AxesBitmap zMap;
	if (gb.Seen('Z'))
	{
		uint32_t zMapping[MaxAxes];
		size_t zCount = numVisibleAxes;
		gb.GetUnsignedArray(zMapping, zCount, false);
		zMap = AxesBitmap::MakeFromArray(zMapping, zCount) & AxesBitmap::MakeLowestNBits(numVisibleAxes);
		seen = true;
	}
	else
	{
		zMap = DefaultZAxisMapping;					// by default map Z axis straight through
	}

	if (xMap.Intersects(yMap) || xMap.Intersects(zMap) || yMap.Intersects(zMap))
	{
		reply.copy("Cannot map two or more of X,Y,Z to the same axis");
		return GCodeResult::error;
	}

	// Check for fan mapping
	FansBitmap fanMap;
	if (gb.Seen('F'))
	{
		int32_t fanMapping[MaxFans];				// use a signed array so that F-1 will result in no fans at all
		size_t fanCount = MaxFans;
		gb.GetIntArray(fanMapping, fanCount, false);
		fanMap = FansBitmap::MakeFromArray(fanMapping, fanCount) & FansBitmap::MakeLowestNBits(MaxFans);
		seen = true;
	}
	else
	{
		fanMap.SetBit(0);							// by default map fan 0 to fan 0
	}

	// Check for a spindle to attach
	int8_t spindleNumber = -1;
	size_t sCount = 0;
	if (gb.Seen('R'))
	{
		seen = true;
		spindleNumber = gb.GetLimitedIValue('R', -1, MaxSpindles);
		++sCount;
	}

	if (seen)
	{
		if (!LockAllMovementSystemsAndWaitForStandstill(gb))
		{
			return GCodeResult::notFinished;
		}

		// Check if filament support is being enforced
		const int filamentDrive = (gb.Seen('L')) ? gb.GetIValue()
									: ((dCount == 1) ? drives[0]
										: -1);

		// Add or delete tool, so start by deleting the old one with this number, if any
		for (MovementState& ms : moveStates)
		{
			if (ms.GetCurrentToolNumber() == (int)toolNumber)
			{
				ms.SelectTool(-1, false);
			}
		}
		Tool::DeleteTool(toolNumber);

		// M563 P# D-1 H-1 [R-1] removes an existing tool
		if (dCount == 1 && hCount == 1 && drives[0] == -1 && heaters[0] == -1 && (sCount == 0 || (sCount == 1 && spindleNumber == -1)))
		{
			// nothing more to do
		}
		else
		{
			Tool *_ecv_null const tool = Tool::Create(toolNumber, name.c_str(), drives, dCount, heaters, hCount, xMap, yMap, zMap, fanMap, filamentDrive, sCount, spindleNumber, reply);
			if (tool == nullptr)
			{
				return GCodeResult::error;
			}
			Tool::AddTool(tool);
		}
	}
	else
	{
		ReadLockedPointer<Tool> const tool = Tool::GetLockedTool(toolNumber);
		if (tool.IsNotNull())
		{
			tool->PrintTool(reply);
		}
		else
		{
			reply.copy("Error: Attempt to print details of non-existent tool.\n");
			return GCodeResult::error;
		}
	}
	return GCodeResult::ok;
}

// Does what it says.
void GCodes::DisableDrives() noexcept
{
	reprap.GetMove().DisableAllDrivers();
	SetAllAxesNotHomed();
}

bool GCodes::ChangeMicrostepping(size_t drive, unsigned int microsteps, bool interp, const StringRef& reply) const noexcept
{
	Move& move = reprap.GetMove();
	const unsigned int oldSteps = move.GetMicrostepping(drive);
	const bool success = move.SetMicrostepping(drive, microsteps, interp, reply);
	if (success)
	{
		// We changed the microstepping, so adjust the steps/mm to compensate
		const float ratio = move.SetDriveStepsPerMm(drive, move.DriveStepsPerMm(drive), oldSteps);
		AdjustEndpoint(drive, ratio);
	}
	return success;
}

// Set the speeds of fans mapped for the current tool to lastDefaultFanSpeed.
// gbp is nullptr when called from the Display subsystem.
void GCodes::SetMappedFanSpeed(const GCodeBuffer *null gbp, float f) noexcept
{
	MovementState& ms = (gbp == nullptr) ? moveStates[0] : GetMovementState(*gbp);
	ms.virtualFanSpeed = f;
	if (ms.currentTool == nullptr)
	{
		reprap.GetFansManager().SetFanValue(0, f);
	}
	else
	{
		ms.currentTool->SetFansPwm(f);
	}
}

// Handle sending a reply back to the appropriate interface(s) and update lastResult
// Note that 'reply' may be empty. If it isn't, then we need to append newline when sending it.
void GCodes::HandleReply(GCodeBuffer& gb, GCodeResult rslt, const char *_ecv_array reply) noexcept
{
	gb.SetLastResult(rslt);
	HandleReplyPreserveResult(gb, rslt, reply);
}

// Handle sending a reply back to the appropriate interface(s) but don't update lastResult
// Note that 'reply' may be empty. If it isn't, then we need to append newline when sending it.
void GCodes::HandleReplyPreserveResult(GCodeBuffer& gb, GCodeResult rslt, const char *_ecv_array reply) noexcept
{
#if HAS_SBC_INTERFACE
	// Deal with replies to the SBC
	if (gb.LatestMachineState().lastCodeFromSbc)
	{
		MessageType type = gb.GetResponseMessageType();
		if (rslt == GCodeResult::notFinished || gb.HasJustStartedMacro() ||
			(gb.LatestMachineState().waitingForAcknowledgement && gb.IsMessagePromptPending()))
		{
			if (reply[0] == 0)
			{
				// Don't send empty push messages
				return;
			}
			type = (MessageType)(type | PushFlag);
		}

		if (rslt == GCodeResult::warning)
		{
			type = AddWarning(type);
		}
		else if (rslt == GCodeResult::error)
		{
			type = AddError(type);
			if (reprap.IsProcessingConfig())
			{
				reprap.SaveConfigError(gb.LatestMachineState().fname, gb.GetLineNumber(), reply);
			}
		}

		platform.Message(type, reply);
		return;
	}
#endif

	// Don't report empty responses if a file or macro is being processed, or if the GCode was queued, or to PanelDue
	if (   reply[0] == 0
		&& (   gb.IsFileChannel() || &gb == QueuedGCode() || &gb == TriggerGCode() || &gb == AutoPauseGCode() || &gb == DaemonGCode()
#if SUPPORT_ASYNC_MOVES
			|| &gb == Queue2GCode()
#endif
#if NUM_ASYNC_CHANNELS != 0
			|| (&gb == AuxGCode() && !platform.IsChanRaw(FirstAuxChannel))
# if NUM_ASYNC_CHANNELS > 1
			|| (&gb == Aux2GCode() && !platform.IsChanRaw(FirstAuxChannel + 1))
# endif
#endif
			|| gb.IsDoingFileMacro(true)
		   )
	   )
	{
		return;
	}

	if (rslt == GCodeResult::error && reprap.IsProcessingConfig())
	{
		reprap.SaveConfigError(gb.LatestMachineState().fname, gb.GetLineNumber(), reply);
	}

	const MessageType initialMt = gb.GetResponseMessageType();
	const MessageType mt = (rslt == GCodeResult::error) ? AddError(initialMt)
							: (rslt == GCodeResult::warning) ? AddWarning(initialMt)
								: initialMt;

	switch (gb.LatestMachineState().compatibility.RawValue())
	{
	case Compatibility::Default:
	case Compatibility::RepRapFirmware:
		// DWC expects a reply from every code, so we must even send empty responses
		if (reply[0] != 0 || gb.IsLastCommand() || &gb == HttpGCode())
		{
			platform.MessageF(&gb, mt, "%s\n", reply);
		}
		break;

	case Compatibility::NanoDLP:				// nanoDLP is like Marlin except that G0 and G1 commands return "Z_move_comp<LF>" before "ok<LF>"
	case Compatibility::Marlin:
	default:
		if (gb.IsLastCommand() && !gb.IsDoingFileMacro(true))
		{
			// Put "ok" at the end
			const char *_ecv_array const response = (gb.GetCommandLetter() == 'M' && gb.GetCommandNumber() == 998) ? "rs " : "ok";
			// We don't need to handle M20 here because we always allocate an output buffer for that one
			if (gb.GetCommandLetter() == 'M' && (gb.GetCommandNumber() == 105 || gb.GetCommandNumber() == 998))
			{
				platform.MessageF(mt, "%s %s\n", response, reply);
			}
			else if (gb.GetCommandLetter() == 'M' && gb.GetCommandNumber() == 28)
			{
				platform.MessageF(mt, "%s\n%s\n", response, reply);
			}
			else if (reply[0] != 0)
			{
				platform.MessageF(mt, "%s\n%s\n", reply, response);
			}
			else
			{
				platform.MessageF(mt, "%s\n", response);
			}
		}
		else if (reply[0] != 0)
		{
			platform.MessageF(mt, "%s\n", reply);
		}
		break;
	}
}

// Handle a successful response when the response is in an OutputBuffer
void GCodes::HandleReply(GCodeBuffer& gb, OutputBuffer *_ecv_null reply) noexcept
{
	gb.SetLastResult(GCodeResult::ok);

	// Although unlikely, it's possible that we get a nullptr reply. Don't proceed if this is the case
	if (reply == nullptr)
	{
		return;
	}

#if HAS_SBC_INTERFACE
	// Deal with replies to the SBC
	if (gb.LatestMachineState().lastCodeFromSbc)
	{
		platform.Message(gb.GetResponseMessageType(), reply);
		return;
	}
#endif

	const MessageType type = gb.GetResponseMessageType();
	const char *_ecv_array const response = (gb.GetCommandLetter() == 'M' && gb.GetCommandNumber() == 998) ? "rs " : "ok";

	switch (gb.LatestMachineState().compatibility.RawValue())
	{
	case Compatibility::Default:
	case Compatibility::RepRapFirmware:
		platform.Message(&gb, type, reply);
		return;

	case Compatibility::Marlin:
	case Compatibility::NanoDLP:
	default:
		if (gb.GetCommandLetter() == 'M')
		{
			// The response to some M-codes is handled differently in Marlin mode
			if (   gb.GetCommandNumber() == 20											// M20 in Marlin mode adds text around the file list
				&& ((*reply)[0] != '{' || (*reply)[1] != '"')							// ...but don't if it looks like a JSON response
			   )
			{
				platform.Message(type, "Begin file list\n");
				platform.Message(type, reply);
				platform.MessageF(type, "End file list\n%s\n", response);
				return;
			}

			if (gb.GetCommandNumber() == 28)
			{
				platform.MessageF(type, "%s\n", response);
				platform.Message(type, reply);
				return;
			}

			if (gb.GetCommandNumber() == 105 || gb.GetCommandNumber() == 998)
			{
				platform.MessageF(type, "%s ", response);
				platform.Message(type, reply);
				return;
			}
		}

		if (reply->Length() != 0)
		{
			platform.Message(type, reply);
			if (!gb.IsDoingFileMacro())
			{
				platform.MessageF(type, "\n%s\n", response);
			}
		}
		else
		{
			OutputBuffer::ReleaseAll(reply);
			platform.MessageF(type, "%s\n", response);
		}
		return;
	}

	// If we get here then we didn't handle the message, so release the buffer(s)
	OutputBuffer::ReleaseAll(reply);
}

// Set all a tool's heaters active and standby temperatures, for M104/M109
void GCodes::SetToolHeaters(const GCodeBuffer& gb, Tool *_ecv_null tool, float temperature) THROWS(GCodeException)
{
	if (tool == nullptr)
	{
		gb.ThrowGCodeException("setting temperature: no tool selected");
	}

	for (size_t h = 0; h < tool->HeaterCount(); h++)
	{
		tool->SetToolHeaterActiveTemperature(h, temperature);
		tool->SetToolHeaterStandbyTemperature(h, temperature);
	}
}

// Retract or un-retract filament, returning true if movement has been queued, false if this needs to be called again
GCodeResult GCodes::RetractFilament(GCodeBuffer& gb, bool retract) THROWS(GCodeException)
{
	MovementState& ms = GetMovementState(gb);
	if (!ms.IsCurrentObjectCancelled())
	{
		Tool *_ecv_null const currentTool = ms.currentTool;
		if (currentTool != nullptr && retract != currentTool->IsRetracted())
		{
			// We potentially need to retract/hop or unhop/untrtract
			const bool needRetraction = currentTool->DriveCount() != 0 && (currentTool->GetRetractLength() != 0.0 || (!retract && currentTool->GetRetractExtra() != 0.0));
			const bool needZhop = ((retract) ? currentTool->GetConfiguredRetractHop() > 0.0 : currentTool->GetActualZHop() > 0.0)
								&& currentTool->GetZAxisMap().IterateWhile([this](unsigned int axis, unsigned int) noexcept -> bool { return IsAxisHomed(axis); } );	// only hop if Z axes have been homed
			if (needRetraction || needZhop)
			{
				if (!LockMovement(gb))
				{
					return GCodeResult::notFinished;
				}

				if (ms.segmentsLeft != 0)
				{
					return GCodeResult::notFinished;
				}

				SetMoveBufferDefaults(ms);
				ms.raw.movementTool = ms.currentTool;
				ms.raw.filePos = gb.GetJobFilePosition();

				// Allocate any axes and extruders that we re going to use
#if SUPPORT_ASYNC_MOVES
# if PREALLOCATE_TOOL_AXES
				if (needZhop)
				{
					AllocateAxes(gb, ms, currentTool->GetZAxisMap(), ParameterLettersBitmap());
				}
# else
				AxesBitmap drivesMoving;
				for (size_t i = 0; i < currentTool->DriveCount(); ++i)
				{
					const size_t logicalDrive = ExtruderToLogicalDrive(currentTool->GetDrive(i));
					drivesMoving.SetBit(logicalDrive);
				}
				if (needZhop)
				{
					drivesMoving |= currentTool->GetZAxisMap();
					AllocateAxes(gb, ms, drivesMoving, ParameterLettersBitmap());
				}
				else
				{
					AllocateAxes(gb, ms, drivesMoving, ParameterLettersBitmap());
				}
# endif
#endif
				if (retract)
				{
					// If the current tool has any drivers, set up the retract move
					if (needRetraction)
					{
						for (size_t i = 0; i < currentTool->DriveCount(); ++i)
						{
							const size_t logicalDrive = ExtruderToLogicalDrive(currentTool->GetDrive(i));
							ms.raw.coords[logicalDrive] = -currentTool->GetRetractLength();
						}
						ms.raw.feedRate = currentTool->GetRetractSpeed() * currentTool->DriveCount();
						ms.raw.canPauseAfter = false;											// don't pause after a retraction because that could cause too much retraction
						NewSingleSegmentMoveAvailable(ms);
					}
					if (needZhop)
					{
						gb.SetState(GCodeState::doingFirmwareRetraction);
					}
				}
				else
				{
					if (needZhop)
					{
						// Set up the reverse Z hop move
						const float zHopToUse = currentTool->GetActualZHop();
						currentTool->GetZAxisMap().Iterate([&ms, zHopToUse](unsigned int axis, unsigned int) noexcept -> void
															{
																ms.raw.coords[axis] -= zHopToUse;
															}
														  );
						ms.raw.feedRate = ConvertSpeedFromMmPerSec(ImpossiblyHighFeedRate);		// we rely on the DDA init code to limit the feed rate to what is achievable on each axis
						currentTool->SetActualZHop(0.0);
						ms.raw.canPauseAfter = false;											// don't pause in the middle of this command
						ms.raw.linearAxesMentioned = true;
						NewSingleSegmentMoveAvailable(ms);
					}
					if (needRetraction)
					{
						gb.SetState(GCodeState::doingFirmwareUnRetraction);
					}
				}
				currentTool->SetRetracted(retract);
			}
		}
	}
	return GCodeResult::ok;
}

// Load the specified filament into a tool
GCodeResult GCodes::LoadFilament(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	Tool *_ecv_null const tool = GetMovementState(gb).currentTool;
	if (tool == nullptr)
	{
		reply.copy("No tool selected");
		return GCodeResult::error;
	}

	if (tool->GetFilament() == nullptr)
	{
		reply.copy("Loading filament into the selected tool is not supported");
		return GCodeResult::error;
	}

	bool seen = false;
	bool runMacro = true;
	gb.TryGetBValue('P', runMacro, seen);

	if (gb.Seen('S'))
	{
		String<FilamentNameLength> filamentName;
		gb.GetQuotedString(filamentName.GetRef());

		if (filamentName.Contains(',') >= 0)
		{
			reply.copy("Filament names must not contain commas");
			return GCodeResult::error;
		}

		if (filamentName.EqualsIgnoreCase(tool->GetFilament()->GetName()))
		{
			// Filament already loaded - nothing to do
			return GCodeResult::ok;
		}

		if (tool->GetFilament()->IsLoaded())
		{
			reply.copy("Unload the current filament before you attempt to load another one");
			return GCodeResult::error;
		}

		SafeStrncpy(filamentToLoad, filamentName.c_str(), ARRAY_SIZE(filamentToLoad));
		gb.SetState(GCodeState::loadingFilament);

		String<StringLength256> scratchString;
		scratchString.printf("%s%s/%s", FILAMENTS_DIRECTORY, filamentName.c_str(), LOAD_FILAMENT_G);
		if (runMacro)
		{
			DoFileMacro(gb, scratchString.c_str(), true, SystemHelperMacroCode);
		}
	}
	else if (tool->GetFilament()->IsLoaded())
	{
		reply.printf("Loaded filament in the selected tool: %s", tool->GetFilament()->GetName());
	}
	else
	{
		reply.printf("No filament loaded in the selected tool");
	}
	return GCodeResult::ok;
}

// Unload the current filament from a tool
GCodeResult GCodes::UnloadFilament(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	Tool *_ecv_null const tool = GetMovementState(gb).currentTool;
	if (tool == nullptr)
	{
		reply.copy("No tool selected");
		return GCodeResult::error;
	}

	if (tool->GetFilament() == nullptr)
	{
		reply.copy("Unloading filament from this tool is not supported");
		return GCodeResult::error;
	}

	bool seen = false;
	bool runMacro = true;
	gb.TryGetBValue('P', runMacro, seen);

	if (tool->GetFilament()->IsLoaded())			// if no filament is loaded, nothing to do
	{
		gb.SetState(GCodeState::unloadingFilament);
		String<StringLength256> scratchString;
		scratchString.printf("%s%s/%s", FILAMENTS_DIRECTORY, tool->GetFilament()->GetName(), UNLOAD_FILAMENT_G);
		if (runMacro)
		{
			DoFileMacro(gb, scratchString.c_str(), true, SystemHelperMacroCode);
		}
	}
	return GCodeResult::ok;
}

float GCodes::GetRawExtruderTotalByDrive(size_t extruder) const noexcept
{
	return (extruder < numExtruders) ? rawExtruderTotalByDrive[extruder] : 0.0;
}

// Cancel the current SD card print.
// This is called from Pid.cpp when there is a heater fault, and from elsewhere in this module.
// When called to stop a print normally, this is called by both fileGCode and file2GCode.
// If the reason is normal completion then gbp is either File or File2; otherwise it is nullptr.
void GCodes::StopPrint(GCodeBuffer *_ecv_null gbp, StopPrintReason reason) noexcept
{
	deferredPauseCommandPending = nullptr;
	pauseState = PauseState::notPaused;

#if HAS_SBC_INTERFACE || HAS_MASS_STORAGE || HAS_EMBEDDED_FILES
	if (gbp != nullptr)
	{
		gbp->ClosePrintFile();
		UnlockAll(*gbp);
# if SUPPORT_ASYNC_MOVES
		if (gbp == FileGCode())
		{
			gbp->ExecuteAll();					// un-fork
		}
		else
		{
			return;								// we are a forked input stream so just exit
		}
# endif
	}
	else
	{
		FileGCode()->ClosePrintFile();
		UnlockAll(*FileGCode());
# if SUPPORT_ASYNC_MOVES
		FileGCode()->ExecuteAll();				// un-fork
		FileGCode()->syncState = GCodeBuffer::SyncState::running;
		FileGCode()->syncPointId = 0;
		File2GCode()->ClosePrintFile();
		UnlockAll(*File2GCode());
		File2GCode()->ExecuteAll();				// un-fork File2 too, so stale M598 from SBC takes the fast path
		File2GCode()->syncState = GCodeBuffer::SyncState::running;
		File2GCode()->syncPointId = 0;
# endif
	}
#endif

	// Don't call ResetMoveCounters here because we can't be sure that the movement queue is empty

	for (MovementState& ms : moveStates)
	{
		ms.positionMayBeInaccurate = true;		// the discarded move (if any) may have already updated the user position, so re-read it at the next standstill
		ms.segmentsLeft = 0;
		ms.codeQueue->Clear();
#if SUPPORT_LASER
		ms.laserPixelData.Clear();
#endif
		// Deal with the Z hop from a G10 that has not been undone by G11
		if (ms.currentTool != nullptr)
		{
			ms.currentUserPosition[Z_AXIS] += ms.currentTool->GetActualZHop();
			ms.currentTool->SetActualZHop(0.0);
			ms.currentTool->SetRetracted(false);
		}
#if SUPPORT_ASYNC_MOVES
		// A motion system other than the primary one cannot run any code after the job has ended, so if it kept its axes or its tool then the next job could not allocate them
		if (&ms != &moveStates[0])
		{
			ms.ReleaseAllOwnedAxesAndExtruders();
			if (ms.currentTool != nullptr)
			{
				if (!IsSimulating())
				{
					ms.currentTool->Standby();
				}
				ms.raw.movementTool = ms.currentTool = nullptr;
			}
			ms.newToolNumber = -1;
		}
#endif
	}

	const char *_ecv_array _ecv_null printingFilename = reprap.GetPrintMonitor().GetPrintingFilename();
	if (printingFilename == nullptr)
	{
		printingFilename = "(unknown)";
	}

	if (exitSimulationWhenFileComplete)
	{
		const float simSeconds = reprap.GetMove().GetSimulationTime() + simulationTime;
#if HAS_MASS_STORAGE
		if (updateFileWhenSimulationComplete && reason == StopPrintReason::normalCompletion)
		{
			MassStorage::RecordSimulationTime(printingFilename, lrintf(simSeconds));
		}
#endif

		exitSimulationWhenFileComplete = false;
		updateFileWhenSimulationComplete = false;
		simulationMode = SimulationMode::off;					// do this after we append the simulation info to the file so that DWC doesn't try to reload the file info too soon
		reprap.GetMove().Simulate(simulationMode);
		EndSimulation(nullptr);
		reprap.GetPrintMonitor().StoppedPrint();

		const uint32_t simMinutes = lrintf(simSeconds/60.0);
		if (reason == StopPrintReason::normalCompletion)
		{
			lastDuration = (unsigned int)simSeconds;
			platform.MessageF(LoggedGenericMessage, "File %s will print in %" PRIu32 "h %" PRIu32 "m plus heating time\n",
									printingFilename, simMinutes/60u, simMinutes % 60u);
		}
		else
		{
			lastDuration = 0;
			platform.MessageF(LoggedGenericMessage, "Cancelled simulating file %s after %" PRIu32 "h %" PRIu32 "m simulated time\n",
									printingFilename, simMinutes/60u, simMinutes % 60u);
		}
	}
	else if (reprap.GetPrintMonitor().IsPrinting())
	{
		if (reason == StopPrintReason::abort)
		{
			reprap.GetHeat().SwitchOffAll(true);				// turn all heaters off
			switch (machineType)
			{
			case MachineType::cnc:
				for (size_t i = 0; i < MaxSpindles; i++)
				{
					platform.AccessSpindle(i).SetState(SpindleState::stopped);
				}
				break;

#if SUPPORT_LASER
			case MachineType::laser:
				platform.SetLaserPwm(0);
				break;
#endif

			default:
				break;
			}
		}

		// Pronterface expects a "Done printing" message
		if (UsbGCode()->LatestMachineState().compatibility == Compatibility::Marlin)
		{
			platform.Message(UsbMessage, "Done printing file\n");
		}
#if SUPPORT_TELNET
		if (TelnetGCode()->LatestMachineState().compatibility == Compatibility::Marlin)
		{
			platform.Message(TelnetMessage, "Done printing file\n");
		}
#endif
		const uint32_t printSeconds = lrintf(reprap.GetPrintMonitor().GetPrintDuration());
		const uint32_t printMinutes = printSeconds/60;
		lastDuration = (reason == StopPrintReason::normalCompletion) ? printSeconds : 0;
		platform.MessageF(LoggedGenericMessage, "%s printing file %s, print time was %" PRIu32 "h %" PRIu32 "m\n",
			(reason == StopPrintReason::normalCompletion) ? "Finished" : "Cancelled",
			printingFilename, printMinutes/60u, printMinutes % 60u);
#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE
		if (reason == StopPrintReason::normalCompletion)
		{
			platform.DeleteSysFile(RESUME_AFTER_POWER_FAIL_G);
			if (FileGCode()->GetState() == GCodeState::normal)		// this should always be the case
			{
				const GCodeState newState = FileGCode()->IsExecuting() ? GCodeState::stoppingFromCode : GCodeState::stopping;
				FileGCode()->SetState(newState);					// set fileGCode (which should be the one calling this) to run stop.g
			}
		}
		else
#endif
		{
			reprap.GetPrintMonitor().StoppedPrint();
		}
	}

	CancelWaitForTemperatures(true);
	buildObjects.Init();
	FileGCode()->OriginalMachineState().ClearBlocks();		// delete any local variables that the job file created
#if SUPPORT_ASYNC_MOVES
	File2GCode()->OriginalMachineState().ClearBlocks();		// delete any local variables that the job file created
#endif
}

// Return true if all the heaters for the specified tool are at their set temperatures
bool GCodes::ToolHeatersAtSetTemperatures(const Tool *_ecv_null tool, bool waitWhenCooling, float tolerance, bool waitOnFault) const noexcept
{
	if (tool != nullptr)
	{
		for (size_t i = 0; i < tool->HeaterCount(); ++i)
		{
			if (!reprap.GetHeat().HeaterAtSetTemperature(tool->GetHeater(i), waitWhenCooling, tolerance, waitOnFault))
			{
				return false;
			}
		}
	}
	return true;
}

// Get the current position from the Move class
void GCodes::UpdateCurrentUserPosition(const GCodeBuffer& gb) noexcept
{
	MovementState& ms = GetMovementState(gb);
	reprap.GetMove().GetCurrentUserPosition(ms.raw.coords, ms.GetNumber(), true, ms.currentTool);
	UpdateUserPositionFromMachinePosition(gb, ms);
}

// Update the user position from the machine position
void GCodes::UpdateUserPositionFromMachinePosition(const GCodeBuffer& gb, MovementState& ms) noexcept
{
	ToolOffsetInverseTransform(ms);
#if SUPPORT_COORDINATE_ROTATION
	if (ms.g68Angle != 0.0 && gb.DoingCoordinateRotation())
	{
		RotateCoordinates(ms, -ms.g68Angle, ms.currentUserPosition);
	}
#endif
}

// Save position etc. to a restore point.
// Note that restore point coordinates are not affected by workplace coordinate offsets. This allows them to be used in resume.g.
void GCodes::SavePosition(const GCodeBuffer& gb, unsigned int restorePointNumber) noexcept
{
	MovementState& ms = GetMovementState(gb);
	ms.SavePosition(restorePointNumber, numVisibleAxes, gb.LatestMachineState().feedRate, gb.GetJobFilePosition());
}

// Restore user position from a restore point. Also restore the laser power, but not the spindle speed (the user must do that explicitly).
// Currently this is only called to restore position after simulation.
void GCodes::RestorePosition(MovementState& ms, const RestorePoint& rp) noexcept
{
	for (size_t axis = 0; axis < numVisibleAxes; ++axis)
	{
		ms.currentUserPosition[axis] = rp.moveCoords[axis];
	}

	ms.raw.initialUserC0 = rp.initialUserC0;
	ms.raw.initialUserC1 = rp.initialUserC1;

#if SUPPORT_LASER || SUPPORT_IOBITS
	ms.raw.laserPwmOrIoBits = rp.laserPwmOrIoBits;
#endif
#if SUPPORT_LASER
	ms.laserPixelData = rp.laserPixelData;
#endif
}

// Convert user coordinates to head reference point coordinates, optionally allowing for X and Y axis mapping
// If the X (or Y) axis is mapped to some other axes not including X (or Y), then the X (or Y) coordinate of coordsOut will be left unchanged.
// So make sure it is suitably initialised before calling this.
void GCodes::ToolOffsetTransform(const MovementState& ms, const float coordsIn[MaxAxes], float coordsOut[MaxAxes], AxesBitmap explicitAxes) const noexcept
{
	if (ms.currentTool == nullptr)
	{
		for (size_t axis = 0; axis < numVisibleAxes; ++axis)
		{
			coordsOut[axis] = (coordsIn[axis] * axisScaleFactors[axis]) + currentBabyStepOffsets[axis];
		}
	}
	else
	{
		const AxesBitmap xAxes = ms.currentTool->GetXAxisMap();
		const AxesBitmap yAxes = ms.currentTool->GetYAxisMap();
		const AxesBitmap zAxes = ms.currentTool->GetZAxisMap();
		for (size_t axis = 0; axis < numVisibleAxes; ++axis)
		{
			if (   (axis != X_AXIS || xAxes.IsBitSet(X_AXIS))
				&& (axis != Y_AXIS || yAxes.IsBitSet(Y_AXIS))
				&& (axis != Z_AXIS || zAxes.IsBitSet(Z_AXIS))
			   )
			{
				float totalOffset = currentBabyStepOffsets[axis] - ms.currentTool->GetOffset(axis);
				if (zAxes.IsBitSet(axis))
				{
					totalOffset += ms.currentTool->GetActualZHop();
				}
				const size_t inputAxis = (explicitAxes.IsBitSet(axis)) ? axis
										: (xAxes.IsBitSet(axis)) ? X_AXIS
											: (yAxes.IsBitSet(axis)) ? Y_AXIS
												: (zAxes.IsBitSet(axis)) ? Z_AXIS
													: axis;
				coordsOut[axis] = (coordsIn[inputAxis] * axisScaleFactors[axis]) + totalOffset;
			}
		}
	}
}

// Convert user coordinates to head reference point coordinates
void GCodes::ToolOffsetTransform(MovementState& ms, AxesBitmap explicitAxes) const noexcept
{
	ToolOffsetTransform(ms, ms.currentUserPosition, ms.raw.coords, explicitAxes);
}

// Convert head reference point coordinates to user coordinates
// Caution: coordsIn and coordsOut may address the same array!
void GCodes::ToolOffsetInverseTransform(const MovementState& ms, const float coordsIn[MaxAxes], float coordsOut[MaxAxes]) const noexcept
{
	if (ms.currentTool == nullptr)
	{
		for (size_t axis = 0; axis < numVisibleAxes; ++axis)
		{
			coordsOut[axis] = (coordsIn[axis] - currentBabyStepOffsets[axis])/axisScaleFactors[axis];
		}
	}
	else
	{
		const AxesBitmap xAxes = ms.GetCurrentXAxes();
		const AxesBitmap yAxes = ms.GetCurrentYAxes();
		const AxesBitmap zAxes = ms.GetCurrentZAxes();
		float xCoord = 0.0, yCoord = 0.0, zCoord = 0.0;
		size_t numXAxes = 0, numYAxes = 0, numZAxes = 0;
		for (size_t axis = 0; axis < numVisibleAxes; ++axis)
		{
			float totalOffset = currentBabyStepOffsets[axis] - ms.currentTool->GetOffset(axis);
			if (zAxes.IsBitSet(axis))
			{
				totalOffset += ms.currentTool->GetActualZHop();
			}
			const float coord = (coordsIn[axis] - totalOffset)/axisScaleFactors[axis];
			coordsOut[axis] = coord;
			if (xAxes.IsBitSet(axis))
			{
				xCoord += coord;
				++numXAxes;
			}
			if (yAxes.IsBitSet(axis))
			{
				yCoord += coord;
				++numYAxes;
			}
			if (zAxes.IsBitSet(axis))
			{
				zCoord += coord;
				++numZAxes;
			}
		}
		if (numXAxes != 0)
		{
			coordsOut[X_AXIS] = xCoord/(float)numXAxes;
		}
		if (numYAxes != 0)
		{
			coordsOut[Y_AXIS] = yCoord/(float)numYAxes;
		}
		if (numZAxes != 0)
		{
			coordsOut[Z_AXIS] = zCoord/(float)numZAxes;
		}
	}
}

// Convert head reference point coordinates to user coordinates, allowing for XY axis mapping
void GCodes::ToolOffsetInverseTransform(MovementState& ms) const noexcept
{
	ToolOffsetInverseTransform(ms, ms.raw.coords, ms.currentUserPosition);
}

// Get the current user coordinate and remove the coordinate rotation and workplace offset
float GCodes::GetUserCoordinate(const MovementState& ms, size_t axis) const noexcept
{
	return (axis < numTotalAxes) ? ms.currentUserPosition[axis] - GetWorkplaceOffset(axis, ms.currentCoordinateSystem) : 0.0;
}

bool GCodes::AllAxesAreHomed() const noexcept
{
	const AxesBitmap allAxes = AxesBitmap::MakeLowestNBits(numVisibleAxes);
	return (axesVirtuallyHomed & allAxes) == allAxes;
}

// Tell us that the axis is now homed
void GCodes::SetAxisIsHomed(unsigned int axis) noexcept
{
	if (!IsSimulating())
	{
		axesHomed.SetBit(axis);
		axesVirtuallyHomed = axesHomed;
		reprap.MoveUpdated();
	}
}

// Tell us that the axis is not homed
void GCodes::SetAxisNotHomed(unsigned int axis) noexcept
{
	if (!IsSimulating())
	{
		axesHomed.ClearBit(axis);
		axesVirtuallyHomed = axesHomed;
		if (axis == Z_AXIS)
		{
			zDatumSetByProbing = false;
		}
		Tool::CheckZHopsValid(axesHomed);
		reprap.MoveUpdated();
	}
}

// Flag all axes as not homed
void GCodes::SetAllAxesNotHomed() noexcept
{
	if (!IsSimulating())
	{
		axesHomed.Clear();
		axesVirtuallyHomed = axesHomed;
		zDatumSetByProbing = false;
		Tool::CheckZHopsValid(axesHomed);
		reprap.MoveUpdated();
	}
}

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE

// Write the config-override file returning true if an error occurred
GCodeResult GCodes::WriteConfigOverrideFile(GCodeBuffer& gb, const StringRef& reply) const THROWS(GCodeException)
{
	const char *_ecv_array const fileName = CONFIG_OVERRIDE_G;
	FileStore *_ecv_null const f = platform.OpenSysFile(fileName, OpenMode::write);
	if (f == nullptr)
	{
		reply.printf("Failed to create file %s", fileName);
		return GCodeResult::error;
	}

	bool ok = WriteConfigOverrideHeader(f);
	if (ok)
	{
		ok = reprap.GetMove().GetKinematics().WriteCalibrationParameters(f);
	}
	if (ok)
	{
		ok = reprap.GetHeat().WriteModelParameters(f);
	}

	// M500 can have a Pnn:nn parameter to enable extra data being saved
	// P10 will enable saving of tool offsets even if they have not been determined via M585
	bool p10 = false;

	// P31 will include G31 Z probe value
	bool p31 = false;

# if SUPPORT_COORDINATE_ROTATION
	// P68 will include G68 coordinate rotation for the first movement system
	bool p68 = false;
#endif

	if (gb.Seen('P'))
	{
		uint32_t pVals[3];
		size_t pCount = 3;
		gb.GetUnsignedArray(pVals, pCount, false);
		for (size_t i = 0; i < pCount; i++)
		{
			switch (pVals[i])
			{
			case 10:
				p10 = true;
				break;

			case 31:
				p31 = true;
				break;

# if SUPPORT_COORDINATE_ROTATION
			case 68:
				p68 = true;
				break;
#endif
			default:
				break;					// other values are ignored without warning
			}
		}
	}
	if (ok)
	{
		ok = reprap.GetMove().WriteMoveParameters(f);
	}
	if (ok)
	{
		ok = platform.WritePlatformParameters(f, p31);
	}

	if (ok)
	{
		ok = WriteToolParameters(f, p10);
	}

	if (ok)
	{
		ok = WriteWorkplaceCoordinates(f);
	}

# if SUPPORT_COORDINATE_ROTATION
	if (ok && p68)
	{
		ok = WriteCoordinateRotation(f, GetPrimaryMovementState());
	}
#endif

	if (!f->Close())
	{
		ok = false;
	}

	if (!ok)
	{
		reply.printf("Failed to write file %s", fileName);
		platform.DeleteSysFile(fileName);
		return GCodeResult::error;
	}

	if (!m501SeenInConfigFile)
	{
		reply.copy("No M501 command was executed in config.g");
		return GCodeResult::warning;
	}

	return GCodeResult::ok;
}

// Write the config-override header returning true if success
// This is implemented as a separate function to avoid allocating a buffer on the stack and then calling functions that also allocate buffers on the stack
bool GCodes::WriteConfigOverrideHeader(FileStore *f) const noexcept
{
	String<MaxFilenameLength> buf;
	buf.copy("; config-override.g file generated in response to M500");
	tm timeInfo;
	if (platform.GetDateTime(timeInfo))
	{
		buf.catf(" at %04u-%02u-%02u %02u:%02u",
						timeInfo.tm_year + 1900, timeInfo.tm_mon + 1, timeInfo.tm_mday, timeInfo.tm_hour, timeInfo.tm_min);
	}
	buf.cat('\n');
	bool ok = f->Write(buf.c_str());
	if (ok)
	{
		ok = f->Write("; This is a system-generated file - do not edit\n");
	}
	return ok;
}

#endif

// Store a M105-format temperature report in 'reply'. This doesn't put a newline character at the end.
void GCodes::GenerateTemperatureReport(const GCodeBuffer& gb, const StringRef& reply) const noexcept
{
	{
		ReadLocker lock(Tool::toolListLock);

		// The following is believed to be compatible with Marlin and Octoprint, based on thread https://github.com/foosel/OctoPrint/issues/2590#issuecomment-385023980
		ReportToolTemperatures(reply, GetConstMovementState(gb).currentTool, false);

		for (const Tool *_ecv_null tool = Tool::GetToolList(); tool != nullptr; tool = tool->Next())
		{
			ReportToolTemperatures(reply, tool, true);
		}
	}

	Heat& heat = reprap.GetHeat();
	for (size_t hn = 0; hn < MaxBedHeaters && heat.GetBedHeater(hn) >= 0; ++hn)
	{
		if (hn == 0)
		{
			if (reply.strlen() != 0)
			{
				reply.cat(' ');
			}
			reply.cat("B:");
		}
		else
		{
			reply.catf(" B%u:", hn);
		}
		const int8_t heater = heat.GetBedHeater(hn);
		reply.catf("%.1f /%.1f", (double)heat.GetHeaterTemperature(heater), (double)heat.GetTargetTemperature(heater));
	}

	for (size_t hn = 0; hn < MaxChamberHeaters && heat.GetChamberHeater(hn) >= 0; ++hn)
	{
		if (hn == 0)
		{
			if (reply.strlen() != 0)
			{
				reply.cat(' ');
			}
			reply.cat("C:");
		}
		else
		{
			reply.catf(" C%u:", hn);
		}
		const int8_t heater = heat.GetChamberHeater(hn);
		reply.catf("%.1f /%.1f", (double)heat.GetHeaterTemperature(heater), (double)heat.GetTargetTemperature(heater));
	}
}

// Check whether we need to report temperatures or status.
// 'reply' is a convenient buffer that is free for us to use.
void GCodes::CheckReportDue(GCodeBuffer& gb, const StringRef& reply) const noexcept
{
	if ((&gb == UsbGCode()
#ifdef SERIAL_USB2_DEVICE
		 || &gb == Usb2GCode()
#endif
		 || &gb == AuxGCode() || &gb == Aux2GCode()) &&
		gb.IsReportDue())
	{
		switch (gb.GetLastStatusReportType())
		{
		case StatusReportType::m105:
			GenerateTemperatureReport(gb, reply);
			if (reply.strlen() > 0)
			{
				reply.cat('\n');
				platform.Message(gb.GetNativeResponseMessageType(), reply.c_str());
				reply.Clear();
			}
			break;

		case StatusReportType::m409:
			try
			{
				OutputBuffer *_ecv_null statusBuf;
				{
					MutexLocker lock(reprap.GetObjectModelReportMutex());
					if (OutputBuffer::GetFreeBuffers() < MinimumBuffersForObjectModel) { break; }
					// In the following, pass nullptr instead of &gb so that no line number is included with the JSON response.
					// DuetScreen relies on this so that it knows the response was generated automatically.
					statusBuf = reprap.GetModelResponse(nullptr, "", "d99fi");									// may throw
				}
				if (statusBuf != nullptr)
				{
					platform.Message(gb.GetNativeResponseMessageType(), statusBuf);
				}
			}
			catch (const GCodeException&)
			{
				// If GetModelResponse threw then don't bother sending a status report
			}
			break;

		default:
			break;
		}
	}
}

// Initiate a tool change. Caller has already checked that the correct tool isn't loaded and set up ms.newToolNumber.
void GCodes::StartToolChange(GCodeBuffer& gb, MovementState& ms, uint8_t param) noexcept
{
	ms.toolChangeParam = (IsSimulating()) ? 0u : param;
	gb.SetState(GCodeState::toolChange0);
}

// Set up some default values in the move buffer for special moves, e.g. for Z probing and firmware retraction
void GCodes::SetMoveBufferDefaults(MovementState& ms) noexcept
{
	ms.SetDefaults(numTotalAxes);
	memcpyf(ms.initialCoords, ms.raw.coords, numVisibleAxes);
}

// Resource locking/unlocking

// Lock the resource, returning true if success.
// Locking the same resource more than once only locks it once, there is no lock count held.
bool GCodes::LockResource(const GCodeBuffer& gb, Resource r) noexcept
{
	if (resourceOwners[r] == &gb)
	{
		return true;
	}
	if (resourceOwners[r] == nullptr)
	{
		resourceOwners[r] = &gb;
		gb.LatestMachineState().lockedResources.SetBit(r);
		return true;
	}
	return false;
}

// Grab the movement lock even if another GCode source has it
void GCodes::GrabResource(const GCodeBuffer& gb, Resource r) noexcept
{
	if (resourceOwners[r] != &gb)
	{
		// Note, we now leave the resource bit set in the original owning GCodeBuffer machine state
		resourceOwners[r] = &gb;
		gb.LatestMachineState().lockedResources.SetBit(r);
	}
}

// Lock the unshareable parts of the file system
bool GCodes::LockFileSystem(const GCodeBuffer &gb) noexcept
{
	return LockResource(gb, FileSystemResource);
}

#if SUPPORT_ASYNC_MOVES

// The movement lock is special because we have one for each motion system

// Lock the movement system that we currently use
bool GCodes::LockMovement(const GCodeBuffer& gb) noexcept
{
	return LockMovement(gb, gb.GetQueueNumberToLock());
}

// Lock movement on all motion systems
bool GCodes::LockAllMovement(const GCodeBuffer& gb) noexcept
{
	for (unsigned int i = 0; i < NumMovementSystems; ++i)
	{
		if (!LockMovement(gb, i))
		{
			UnlockMovementTakenBelow(gb, i);		// release the lower locks we took to avoid deadlock
			return false;
		}
	}
	return true;
}

// Release movement locks greater than the specified one
void GCodes::UnlockMovementFrom(const GCodeBuffer& gb, MovementSystemNumber msNumber) noexcept
{
	while (msNumber < NumMovementSystems)
	{
		UnlockMovement(gb, msNumber);
		++msNumber;
	}
}

// Release movement locks below the specified one that we took ourselves, keeping any that were held when the current macro started
void GCodes::UnlockMovementTakenBelow(const GCodeBuffer& gb, MovementSystemNumber msNumber) noexcept
{
	const GCodeMachineState *_ecv_null const mc = gb.LatestMachineState().GetPrevious();
	for (MovementSystemNumber i = 0; i < msNumber; i++)
	{
		if (mc == nullptr || !mc->lockedResources.IsBitSet(MoveResourceBase + i))
		{
			UnlockMovement(gb, i);
		}
	}
}

// Release all movement locks that we own
void GCodes::UnlockMovement(const GCodeBuffer& gb) noexcept
{
	UnlockMovementFrom(gb, 0);
}

// Grab all movement locks even if other channels owns them
void GCodes::GrabMovement(const GCodeBuffer& gb) noexcept
{
	for (unsigned int i = 0; i < NumMovementSystems; ++i)
	{
		GrabResource(gb, MoveResourceBase + i);
	}
}

#endif

// Unlock the resource if we own it
void GCodes::UnlockResource(const GCodeBuffer& gb, Resource r) noexcept
{
	if (resourceOwners[r] == &gb)
	{
		// Note, we leave the bit set in previous stack levels! This is needed e.g. to allow M291 blocking messages to be used in homing files.
		gb.LatestMachineState().lockedResources.ClearBit(r);
		resourceOwners[r] = nullptr;
	}
}

// Release all locks, except those that were owned when the current macro was started
void GCodes::UnlockAll(const GCodeBuffer& gb) noexcept
{
	const GCodeMachineState *_ecv_null const mc = gb.LatestMachineState().GetPrevious();
	const GCodeMachineState::ResourceBitmap resourcesToKeep = (mc == nullptr) ? GCodeMachineState::ResourceBitmap() : mc->lockedResources;
	for (size_t i = 0; i < NumResources; ++i)
	{
		if (resourceOwners[i] == &gb && !resourcesToKeep.IsBitSet(i))
		{
			if (i >= MoveResourceBase && i < MoveResourceBase + NumMovementSystems && mc == nullptr)
			{
				// In case homing was aborted because of an exception, we need to clear toBeHomed when releasing the movement lock
				//debugPrintf("tbh %04x clearing\n", (unsigned int)toBeHomed.GetRaw());
				toBeHomed.Clear();
			}
			resourceOwners[i] = nullptr;
			gb.LatestMachineState().lockedResources.ClearBit(i);
		}
	}
}

// Append a list of axes to a string
void GCodes::AppendAxes(const StringRef& reply, AxesBitmap axes) const noexcept
{
	axes.Iterate([&reply, this](unsigned int axis, unsigned int) noexcept -> void { reply.cat(this->axisLetters[axis]); });
}

// Get the name of the current machine mode
const char *_ecv_array GCodes::GetMachineModeString() const noexcept
{
	switch (machineType)
	{
	case MachineType::fff:
		return "FFF";
	case MachineType::cnc:
		return "CNC";
	case MachineType::laser:
		return "Laser";
	default:
		return "Unknown";
	}
}

// Return a current extrusion factor as a fraction
float GCodes::GetExtrusionFactor(size_t extruder) noexcept
{
	return (extruder < numExtruders) ? extrusionFactors[extruder] : 0.0;
}

size_t GCodes::GetAxisNumberForLetter(const char axisLetter) const noexcept
{
	for (size_t i = 0; i < numTotalAxes; ++i)
	{
		if (axisLetters[i] == axisLetter)
		{
			return i;
		}
	}
	return MaxAxes;
}

Pwm_t GCodes::ConvertLaserPwm(float reqVal) const noexcept
{
	return (uint16_t)constrain<long>(lrintf((reqVal * 65535)/laserMaxPower), 0, 65535);
}

void GCodes::ActivateHeightmap(bool activate) noexcept
{
	reprap.GetMove().UseMesh(activate);
	if (activate)
	{
		// Update the current position to allow for any bed compensation at the current XY coordinates
		for (MovementState& ms : moveStates)
		{
			reprap.GetMove().GetCurrentUserPosition(ms.raw.coords, ms.GetNumber(), true, ms.currentTool);
			ToolOffsetInverseTransform(ms);							// update user coordinates to reflect any height map offset at the current position
		}
	}
}

// Check that we are allowed to perform network-related commands
// Return true if we are; else return false and set 'reply' and 'result' appropriately
// On entry, 'reply' is empty and 'result' is GCodeResult::ok
bool GCodes::CheckNetworkCommandAllowed(GCodeBuffer& gb, const StringRef& reply, GCodeResult& result) noexcept
{
	if (gb.LatestMachineState().runningM502)			// when running M502 we don't execute network-related commands
	{
		return false;									// just ignore the command but report success
	}

#if HAS_SBC_INTERFACE
	if (reprap.UsingSbcInterface())
	{
		// Networking is disabled when using the SBC interface, to save RAM
		reply.copy("On-board firmware cannot process network-related commands when using an attached Single Board Computer");
		result = GCodeResult::error;
		return false;
	}
#endif

	return true;
}

// Get the movement system that owns a particular axis
void GCodes::RecordEndstopTriggered(size_t axis, HomingMode hmode) noexcept
{
	if (axis != NO_AXIS)
	{
#if SUPPORT_ASYNC_MOVES
		for (MovementState& ms : moveStates)
		{
			if (   (hmode == HomingMode::homeCartesianAxes && ms.GetAxesAndExtrudersOwned().IsBitSet(axis))
				|| (hmode == HomingMode::homeIndividualDrives && ms.raw.logicalDrivesOwned.IsBitSet(axis))
			   )
			{
				ms.endstopsTriggered.SetBit(axis);
				return;
			}
		}
#else
		moveStates[0].endstopsTriggered.SetBit(axis);
#endif
	}
}

#if SUPPORT_ASYNC_MOVES

// Get a reference to the movement state associated with the specified GCode buffer
MovementState& GCodes::GetMovementState(const GCodeBuffer& gb) noexcept
{
	return moveStates[gb.GetActiveQueueNumber()];
}

// Get a reference to the movement state associated with the specified GCode buffer
const MovementState& GCodes::GetConstMovementState(const GCodeBuffer& gb) const noexcept
{
	return moveStates[gb.GetActiveQueueNumber()];
}

const MovementState& GCodes::GetCurrentMovementState(const ObjectExplorationContext& context) const noexcept
{
	const GCodeBuffer *_ecv_null gb = context.GetGCodeBuffer();
	if (gb == nullptr)
	{
# if HAS_NETWORKING
		gb = HttpGCode();				// assume the request came from the network
# else
		return moveStates[0];
# endif
	}
	return GetConstMovementState(*gb);
}

// Allocate additional axes and/or extruders to a movement state returning true if successful, false if another movement state owns it already
// This relies on cooperative scheduling between different GCodeBuffer objects
// The axLetters argument need not be complete, but having it may in future avoid re-allocating axes that are allocated in this call
void GCodes::AllocateAxes(const GCodeBuffer& gb, MovementState& ms, AxesBitmap axes, ParameterLettersBitmap axLetters) THROWS(GCodeException)
{
	//debugPrintf("Allocating axes %04" PRIx32 " letters %08" PRIx32 " command %u\n", axes.GetRaw(), axLetters.GetRaw(), gb.GetCommandNumber());
	const LogicalDrivesBitmap badDrives = ms.AllocateAxes(axes, axLetters);
	//debugPrintf("alloc done\n");
	if (!badDrives.IsEmpty())
	{
		if (reprap.GetDebugFlags(Module::Move).IsBitSet(MoveDebugFlags::AxisAllocation))
		{
			debugPrintf("Failed to allocate drives %07" PRIx32 " to MS %u, axis letters %08"
#if defined(DUET3)
				PRIx64
#else
				PRIx32
#endif
				"\n",
				badDrives.GetRaw(), ms.GetNumber(), axLetters.GetRaw());
		}
		gb.ThrowGCodeException("Drive %c is already used by a different motion system", (unsigned int)axisLetters[badDrives.LowestSetBit()]);
	}
	UpdateUserPositionFromMachinePosition(gb, ms);
}

// Allocate additional axes by letter when we are doing a standard move. The axis letters are as in the GCode, before we account for coordinate rotation and axis mapping.
// We must clear out owned axis letters on a tool change, or when coordinate rotation is changed from zero to nonzero
void GCodes::AllocateAxisLetters(const GCodeBuffer& gb, MovementState& ms, ParameterLettersBitmap axLetters) THROWS(GCodeException)
{
# if SUPPORT_COORDINATE_ROTATION
	// If we are rotating coordinates then X implies Y and vice versa
	if (ms.g68Angle != 0.0)
	{
		if (axLetters.IsBitSet(ParameterLetterToBitNumber('X')))
		{
			axLetters.SetBit(ParameterLetterToBitNumber('Y'));
		}
		if (axLetters.IsBitSet(ParameterLetterToBitNumber('Y')))
		{
			axLetters.SetBit(ParameterLetterToBitNumber('X'));
		}
	}
# endif

	AxesBitmap newAxes;
	for (size_t axis = 0; axis < numVisibleAxes; ++axis)
	{
		const char c = axisLetters[axis];
		const unsigned int axisLetterBitNumber = ParameterLetterToBitNumber(c);
		if (axLetters.IsBitSet(axisLetterBitNumber))
		{
			if (axis == 0)			// axis 0 is always X
			{
				newAxes |= Tool::GetXAxes(ms.currentTool);
			}
			else if (axis == 1)		// axis 1 is always Y
			{
				newAxes |= Tool::GetYAxes(ms.currentTool);
			}
			else
			{
				newAxes.SetBit(axis);
			}
		}
	}
	AllocateAxes(gb, ms, newAxes, axLetters);
}

// Allocate axes by letter when we are doing a special move. Do not update the map of owned axes letters.
void GCodes::AllocateAxesDirectFromLetters(const GCodeBuffer& gb, MovementState& ms, ParameterLettersBitmap axLetters) THROWS(GCodeException)
{
	AxesBitmap newAxes;
	for (size_t axis = 0; axis < numVisibleAxes; ++axis)
	{
		const char c = axisLetters[axis];
		const unsigned int axisLetterBitNumber = ParameterLetterToBitNumber(c);
		if (axLetters.IsBitSet(axisLetterBitNumber))
		{
			newAxes.SetBit(axis);
		}
	}
	AllocateAxes(gb, ms, newAxes, ParameterLettersBitmap());			// don't own the letters!
}

// Allocate drivers by letter for a raw motor move
void GCodes::AllocateLogicalDrivesFromLetters(const GCodeBuffer& gb, MovementState& ms, ParameterLettersBitmap axLetters) THROWS(GCodeException)
{
	LogicalDrivesBitmap newDrives;
	for (size_t axis = 0; axis < numVisibleAxes; ++axis)
	{
		const char c = axisLetters[axis];
		const unsigned int axisLetterBitNumber = ParameterLetterToBitNumber(c);
		if (axLetters.IsBitSet(axisLetterBitNumber))
		{
			newDrives.SetBit(axis);
		}
	}
	const LogicalDrivesBitmap badDrives = ms.AllocateDrives(newDrives);
	if (!badDrives.IsEmpty())
	{
		if (reprap.GetDebugFlags(Module::Move).IsBitSet(MoveDebugFlags::AxisAllocation))
		{
			debugPrintf("Failed to allocate drives %07" PRIx32 " to MS %u"
#if defined(DUET3)
				PRIx64
#else
				PRIx32
#endif
				"\n",
				badDrives.GetRaw(), ms.GetNumber());
		}
		gb.ThrowGCodeException("Drive %c is already used by a different motion system", (unsigned int)axisLetters[badDrives.LowestSetBit()]);
	}
}

// Test whether an axis is unowned
bool GCodes::IsAxisFree(unsigned int axis) const noexcept
{
	for (const MovementState& ms : moveStates)
	{
		if (ms.GetAxesAndExtrudersOwned().IsBitSet(axis))
		{
			return false;
		}
	}
	return true;
}

// Synchronise motion systems and update user coordinates.
// Return true if synced, false if we need to wait longer.
bool GCodes::SyncWith(GCodeBuffer& thisGb, const GCodeBuffer& otherGb) noexcept
{
	if (thisGb.ExecutingAll())
	{
		return LockAllMovementSystemsAndWaitForStandstill(thisGb);
	}

	const MovementSystemNumber ownQueue = thisGb.GetOwnQueueNumber();
	MovementState& ownMs = moveStates[ownQueue];

	if (!LockMovementSystemAndWaitForStandstill(thisGb, ownQueue))
	{
		return false;
	}

	bool synced = false;		// assume failure
	switch (thisGb.syncState)
	{
	case GCodeBuffer::SyncState::running:
		thisGb.syncPointId = thisGb.ComputeSyncPointId();			// capture sync point identity (file position, stack depth, loop iterations)
		thisGb.syncState = GCodeBuffer::SyncState::syncing;				// tell other input channels that we are waiting for sync
		//debugPrintf("Channel %u changed state to syncing, %u\n", thisGb.GetChannel().ToBaseType(), __LINE__);
		[[fallthrough]];
	case GCodeBuffer::SyncState::syncing:
		if (otherGb.syncState == GCodeBuffer::SyncState::running)
		{
			// If the other reader is no longer doing a file, it will never reach a sync point - abort
			if (!otherGb.IsDoingFile())
			{
				platform.MessageF(ErrorMessage, "Other file reader is no longer active while waiting at sync point, aborting print\n");
				UnlockMovement(thisGb, ownQueue);
				AbortPrint(thisGb);
				return true;
			}
			// The other input channel has not caught up with this sync point yet, so wait for it.
			// We don't infer that the other reader skipped the barrier from file position alone,
			// because nested macros/conditionals can legitimately cause the readers to advance at different rates.
			break;
		}

		// Both readers are syncing. Check that they reached the same sync point.
		// The sync point ID incorporates the file position, macro stack depth, and while-loop iteration counts.
		if (thisGb.syncPointId != otherGb.syncPointId)
		{
			// The two file readers reached different M598 barriers - abort the print
			platform.MessageF(ErrorMessage, "File readers reached different sync points (IDs %" PRIu32 " and %" PRIu32 "), aborting print\n",
								thisGb.syncPointId, otherGb.syncPointId);
			UnlockMovement(thisGb, ownQueue);
			AbortPrint(thisGb);
			return true;		// return true so the caller does not loop; AbortPrint handles cleanup
		}

		// If we get here then the other input channel is also syncing, so it's safe to use the machine axis coordinates of the axes it owns to update our user coordinates
		ownMs.UpdateCoordinatesFromLastKnownEndpoints();

		// Now that we no longer need to read axis coordinates from the other motion system, flag that we have finished syncing
		thisGb.syncState = GCodeBuffer::SyncState::synced;
		//debugPrintf("Channel %u changed state to synced, %u\n", thisGb.GetChannel().ToBaseType(), __LINE__);
		[[fallthrough]];
	case GCodeBuffer::SyncState::synced:
		switch (otherGb.syncState)
		{
		case GCodeBuffer::SyncState::running:
			// Other input channel has already left the barrier, so we can leave it too.
			thisGb.syncState = GCodeBuffer::SyncState::running;
			//debugPrintf("Channel %u changed state to running, %u\n", thisGb.GetChannel().ToBaseType(), __LINE__);
			synced = true;
			break;

		case GCodeBuffer::SyncState::syncing:
			// Other input channel hasn't noticed that we are fully synced yet
			break;

		case GCodeBuffer::SyncState::synced:
			// Both input channels have reached the barrier, so we can continue immediately.
			thisGb.syncState = GCodeBuffer::SyncState::running;
			//debugPrintf("Channel %u changed state to running, %u\n", thisGb.GetChannel().ToBaseType(), __LINE__);
			synced = true;
			break;
		}
		break;
	}

	if (!synced)
	{
		// We must release the movement lock if syncing failed, so that if an input wants to pause the print it can get the lock
		UnlockMovement(thisGb, ownQueue);
	}
	return synced;
}

// Empty both motion queues and synchronise the other motion system with this one. Return true if done, false if we need to wait for it to catch up.
bool GCodes::DoSync(GCodeBuffer& gb) noexcept
{
	const bool rslt = (&gb == FileGCode()) ? SyncWith(gb, *File2GCode())
			: (&gb == File2GCode()) ? SyncWith(gb, *FileGCode())
				: LockAllMovementSystemsAndWaitForStandstill(gb);			// if we're not a file input then just wait for all motion systems to stop
	return rslt;
}

#endif

#if HAS_MASS_STORAGE

// Start timing SD card file writing
GCodeResult GCodes::StartSDTiming(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	const float bytesReq = (gb.Seen('S')) ? gb.GetFValue() : 10.0;
	const bool useCrc = (gb.Seen('C') && gb.GetUIValue() != 0);
	timingBytesRequested = (uint32_t)(bytesReq * (float)(1024 * 1024));
	FileStore *_ecv_null const f = platform.OpenFile(Platform::GetGCodeDir(), TimingFileName, (useCrc) ? OpenMode::writeWithCrc : OpenMode::write, timingBytesRequested);
	if (f == nullptr)
	{
		reply.copy("Failed to create file");
		return GCodeResult::error;
	}
	sdTimingFile = f;

	platform.Message(gb.GetResponseMessageType(), "Testing SD card write speed...\n");
	timingBytesWritten = 0;
	timingStartMillis = millis();
	gb.SetState(GCodeState::timingSDwrite);
	return GCodeResult::ok;
}

#endif

#if SUPPORT_DIRECT_LCD

// Set the speed factor. Value passed is a fraction.
void GCodes::SetPrimarySpeedFactor(float factor) noexcept
{
	moveStates[0].speedFactor = constrain<float>(factor, 0.1, 5.0);
}

// Set an extrusion factor
void GCodes::SetExtrusionFactor(size_t extruder, float factor) noexcept
{
	if (extruder < numExtruders)
	{
		extrusionFactors[extruder] = constrain<float>(factor, 0.0, 2.0);
	}
}

// Process a GCode command from the 12864 LCD returning true if the command was accepted
bool GCodes::ProcessCommandFromLcd(const char *_ecv_array cmd) noexcept
{
	if (LcdGCode()->IsCompletelyIdle())
	{
		LcdGCode()->PutAndDecode(cmd);
		return true;
	}
	return false;
}

int GCodes::GetHeaterNumber(unsigned int itemNumber) const noexcept
{
	if (itemNumber < 80)
	{
		ReadLockedPointer<Tool> const tool = (itemNumber == 79) ? GetPrimaryMovementState().GetLockedCurrentTool() : Tool::GetLockedTool(itemNumber);
		return (tool.IsNotNull() && tool->HeaterCount() != 0) ? tool->GetHeater(0) : -1;
	}
	if (itemNumber < 90)
	{
		return (itemNumber < 80 + MaxBedHeaters) ? reprap.GetHeat().GetBedHeater(itemNumber - 80) : -1;
	}
	return (itemNumber < 90 + MaxChamberHeaters) ? reprap.GetHeat().GetChamberHeater(itemNumber - 90) : -1;
}

float GCodes::GetItemCurrentTemperature(unsigned int itemNumber) const noexcept
{
	return reprap.GetHeat().GetHeaterTemperature(GetHeaterNumber(itemNumber));
}

float GCodes::GetItemActiveTemperature(unsigned int itemNumber) const noexcept
{
	if (itemNumber < 80)
	{
		ReadLockedPointer<Tool> const tool = (itemNumber == 79) ? GetPrimaryMovementState().GetLockedCurrentTool() : Tool::GetLockedTool(itemNumber);
		return (tool.IsNotNull()) ? tool->GetToolHeaterActiveTemperature(0) : 0.0;
	}

	return reprap.GetHeat().GetActiveTemperature(GetHeaterNumber(itemNumber));
}

float GCodes::GetItemStandbyTemperature(unsigned int itemNumber) const noexcept
{
	if (itemNumber < 80)
	{
		ReadLockedPointer<Tool> const tool = (itemNumber == 79) ? GetPrimaryMovementState().GetLockedCurrentTool() : Tool::GetLockedTool(itemNumber);
		return (tool.IsNotNull()) ? tool->GetToolHeaterStandbyTemperature(0) : 0.0;
	}

	return reprap.GetHeat().GetStandbyTemperature(GetHeaterNumber(itemNumber));
}

void GCodes::SetItemActiveTemperature(unsigned int itemNumber, float temp) noexcept
{
	try
	{
		if (itemNumber < 80)
		{
			ReadLockedPointer<Tool> const tool = (itemNumber == 79) ? GetPrimaryMovementState().GetLockedCurrentTool() : Tool::GetLockedTool(itemNumber);
			if (tool.IsNotNull())
			{
				tool->SetToolHeaterActiveTemperature(0, temp);
				if (tool->Number() == GetPrimaryMovementState().GetCurrentToolNumber() && temp > NEARLY_ABS_ZERO)
				{
					tool->HeatersToActiveOrStandby(true);				// if it's the current tool then make sure it is active
				}
			}
		}
		else
		{
			const int heaterNumber = GetHeaterNumber(itemNumber);
			reprap.GetHeat().SetActiveTemperature(heaterNumber, temp);
			if (temp > NEARLY_ABS_ZERO)
			{
				String<1> dummy;
				reprap.GetHeat().SetActiveOrStandby(heaterNumber, nullptr, true, dummy.GetRef());
			}
		}
	}
	catch (const GCodeException&) { }
}

void GCodes::SetItemStandbyTemperature(unsigned int itemNumber, float temp) noexcept
{
	try
	{
		if (itemNumber < 80)
		{
			ReadLockedPointer<Tool> const tool = (itemNumber == 79) ? GetPrimaryMovementState().GetLockedCurrentTool() : Tool::GetLockedTool(itemNumber);
			if (tool.IsNotNull())
			{
				tool->SetToolHeaterStandbyTemperature(0, temp);
			}
		}
		else
		{
			reprap.GetHeat().SetStandbyTemperature(GetHeaterNumber(itemNumber), temp);
		}
	}
	catch (const GCodeException&) { }
}

// Evaluate a visibility expression string and return it
bool GCodes::EvaluateConditionForDisplay(const char *_ecv_array str) const noexcept
{
	try
	{
		ExpressionParser parser(LcdGCode(), str);
		return parser.ParseBoolean();
	}
	catch (const GCodeException&)
	{
		return false;
	}
}

// Evaluate a value string returning true if an error occurred
bool GCodes::EvaluateValueForDisplay(const char *_ecv_array str, ExpressionValue& expr) const noexcept
{
	try
	{
		ExpressionParser parser(LcdGCode(), str);
		expr = parser.Parse();
		return false;
	}
	catch (const GCodeException&)
	{
		expr.SetNull(nullptr);
		return true;
	}
}

#endif

// End
