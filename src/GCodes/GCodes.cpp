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
#include "GCodeQueue.h"
#include "Heating/Heat.h"
#include "Platform.h"
#include "Movement/Move.h"
#include "Scanner.h"
#include "PrintMonitor.h"
#include "RepRap.h"
#include "Tools/Tool.h"
#include "Endstops/ZProbe.h"

#if SUPPORT_DOTSTAR_LED
# include "Fans/DotStarLed.h"
#endif

#if HAS_LINUX_INTERFACE
# include "Linux/LinuxInterface.h"
#endif

#ifdef SERIAL_AUX_DEVICE
// Support for emergency stop from PanelDue
bool GCodes::emergencyStopCommanded = false;

void GCodes::CommandEmergencyStop(UARTClass *p) noexcept
{
	emergencyStopCommanded = true;
}
#endif

GCodes::GCodes(Platform& p) noexcept :
	platform(p), machineType(MachineType::fff), active(false),
#if HAS_VOLTAGE_MONITOR
	powerFailScript(nullptr),
#endif
	isFlashing(false), lastWarningMillis(0), atxPowerControlled(false)
#if HAS_MASS_STORAGE
	, sdTimingFile(nullptr)
#endif
{
#if HAS_MASS_STORAGE
	fileBeingHashed = nullptr;
	FileGCodeInput * const fileInput = new FileGCodeInput();
#else
	FileGCodeInput * const fileInput = nullptr;
#endif
	fileGCode = new GCodeBuffer(GCodeChannel::File, nullptr, fileInput, GenericMessage);

# if SUPPORT_HTTP || HAS_LINUX_INTERFACE
	httpInput = new NetworkGCodeInput();
	httpGCode = new GCodeBuffer(GCodeChannel::HTTP, httpInput, fileInput, HttpMessage);
# else
	httpGCode = nullptr;
# endif // SUPPORT_HTTP || HAS_LINUX_INTERFACE
# if SUPPORT_TELNET || HAS_LINUX_INTERFACE
	telnetInput = new NetworkGCodeInput();
	telnetGCode = new GCodeBuffer(GCodeChannel::Telnet, telnetInput, fileInput, TelnetMessage, Compatibility::Marlin);
# else
	telnetGCode = nullptr;
# endif // SUPPORT_TELNET || HAS_LINUX_INTERFACE

#if defined(SERIAL_MAIN_DEVICE)
	StreamGCodeInput * const usbInput = new StreamGCodeInput(SERIAL_MAIN_DEVICE);
	usbGCode = new GCodeBuffer(GCodeChannel::USBchan, usbInput, fileInput, UsbMessage, Compatibility::Marlin);
#elif HAS_LINUX_INTERFACE
	usbGCode = new GCodeBuffer(GCodeChannel::USBchan, nullptr, fileInput, UsbMessage, Compatbility::marlin);
#else
	usbGCode = nullptr;
#endif

#if defined(SERIAL_AUX_DEVICE)
	StreamGCodeInput * const auxInput = new StreamGCodeInput(SERIAL_AUX_DEVICE);
	auxGCode = new GCodeBuffer(GCodeChannel::Aux, auxInput, fileInput, AuxMessage);
#elif HAS_LINUX_INTERFACE
	auxGCode = new GCodeBuffer(GCodeChannel::Aux, nullptr, fileInput, AuxMessage);
#else
	auxGCode = nullptr;
#endif

	triggerGCode = new GCodeBuffer(GCodeChannel::Trigger, nullptr, fileInput, GenericMessage);

	codeQueue = new GCodeQueue();
	queuedGCode = new GCodeBuffer(GCodeChannel::Queue, codeQueue, fileInput, GenericMessage);

#if SUPPORT_12864_LCD || HAS_LINUX_INTERFACE
	lcdGCode = new GCodeBuffer(GCodeChannel::LCD, nullptr, fileInput, LcdMessage);
#else
	lcdGCode = nullptr;
#endif

#if HAS_LINUX_INTERFACE
	spiGCode = new GCodeBuffer(GCodeChannel::SBC, nullptr, fileInput, GenericMessage);
#else
	spiGCode = nullptr;
#endif
	daemonGCode = new GCodeBuffer(GCodeChannel::Daemon, nullptr, fileInput, GenericMessage);
#if defined(SERIAL_AUX2_DEVICE)
	StreamGCodeInput * const aux2Input = new StreamGCodeInput(SERIAL_AUX2_DEVICE);
	aux2GCode = new GCodeBuffer(GCodeChannel::Aux2, aux2Input, fileInput, Aux2Message);
#elif HAS_LINUX_INTERFACE
	aux2GCode = new GCodeBuffer(GCodeChannel::Aux2, nullptr, fileInput, Aux2Message);
#else
	aux2GCode = nullptr;
#endif
	autoPauseGCode = new GCodeBuffer(GCodeChannel::Autopause, nullptr, fileInput, GenericMessage);
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

	numExtruders = NumDefaultExtruders;

	Reset();

	virtualExtruderPosition = rawExtruderTotal = 0.0;
	for (float& f : rawExtruderTotalByDrive)
	{
		f = 0.0;
	}

	runningConfigFile = false;
	m501SeenInConfigFile = false;
	doingToolChange = false;
	active = true;
	limitAxes = noMovesBeforeHoming = true;
	SetAllAxesNotHomed();

	for (float& f : pausedFanSpeeds)
	{
		f = 0.0;
	}
	lastDefaultFanSpeed = pausedDefaultFanSpeed = 0.0;

	lastAuxStatusReportType = -1;						// no status reports requested yet

	laserMaxPower = DefaultMaxLaserPower;
	laserPowerSticky = false;

	heaterFaultState = HeaterFaultState::noFault;
	heaterFaultTime = 0;
	heaterFaultTimeout = DefaultHeaterFaultTimeout;

#if SUPPORT_SCANNER
	reprap.GetScanner().SetGCodeBuffer(usbGCode);
#endif

#if SUPPORT_DOTSTAR_LED
	DotStarLed::Init();
#endif

#if defined(SERIAL_AUX_DEVICE) && !defined(__LPC17xx__)
	SERIAL_AUX_DEVICE.SetInterruptCallback(GCodes::CommandEmergencyStop);
#endif
}

// This is called from Init and when doing an emergency stop
void GCodes::Reset() noexcept
{
	// Here we could reset the input sources as well, but this would mess up M122\nM999
	// because both codes are sent at once from the web interface. Hence we don't do this here.
	for (GCodeBuffer *gb : gcodeSources)
	{
		if (gb != nullptr)
		{
			gb->Reset();
		}
	}

	if (auxGCode != nullptr)
	{
		auxGCode->SetCommsProperties(1);				// by default, we require a checksum on the aux port
	}

	nextGcodeSource = 0;

#if HAS_MASS_STORAGE
	fileToPrint.Close();
#endif
	speedFactor = 1.0;

	for (size_t i = 0; i < MaxExtruders; ++i)
	{
		extrusionFactors[i] = volumetricExtrusionFactors[i] = 1.0;
	}

	for (size_t i = 0; i < MaxAxes; ++i)
	{
		axisScaleFactors[i] = 1.0;
		for (size_t j = 0; j < NumCoordinateSystems; ++j)
		{
			workplaceCoordinates[j][i] = 0.0;
		}
	}

	currentCoordinateSystem = 0;

	for (float& f : moveBuffer.coords)
	{
		f = 0.0;										// clear out all axis and extruder coordinates
	}

	ClearMove();

	for (float& f : currentBabyStepOffsets)
	{
		f = 0.0;										// clear babystepping before calling ToolOffsetInverseTransform
	}

	currentZHop = 0.0;									// clear this before calling ToolOffsetInverseTransform
	lastPrintingMoveHeight = -1.0;
	newToolNumber = -1;
	moveBuffer.tool = nullptr;
	moveBuffer.virtualExtruderPosition = 0.0;

#if SUPPORT_LASER || SUPPORT_IOBITS
	moveBuffer.laserPwmOrIoBits.Clear();
#endif

	reprap.GetMove().GetKinematics().GetAssumedInitialPosition(numVisibleAxes, moveBuffer.coords);
	ToolOffsetInverseTransform(moveBuffer.coords, currentUserPosition);

	for (RestorePoint& rp : numberedRestorePoints)
	{
		rp.Init();
	}

	for (Trigger& tr : triggers)
	{
		tr.Init();
	}
	triggersPending.Clear();

	simulationMode = 0;
	exitSimulationWhenFileComplete = updateFileWhenSimulationComplete = false;
	simulationTime = 0.0;
	lastDuration = 0;

	isPaused = false;
#if HAS_VOLTAGE_MONITOR
	isPowerFailPaused = false;
#endif
	doingToolChange = false;
	doingManualBedProbe = false;
#if HAS_LINUX_INTERFACE
	lastFilePosition = noFilePosition;
#endif
	pausePending = filamentChangePausePending = false;
	moveBuffer.filePos = noFilePosition;
	firmwareUpdateModuleMap = 0;
	lastFilamentError = FilamentSensorStatus::ok;
	currentZProbeNumber = 0;

	buildObjects.Init();

	codeQueue->Clear();
	cancelWait = isWaiting = displayNoToolWarning = false;

	for (const GCodeBuffer*& gbp : resourceOwners)
	{
		gbp = nullptr;
	}
}

// Return true if any channel other than the daemon is executing a file macro
bool GCodes::DoingFileMacro() const noexcept
{
	for (const GCodeBuffer *gbp : gcodeSources)
	{
		if (gbp != nullptr && gbp != daemonGCode && gbp->IsDoingFileMacro())
		{
			return true;
		}
	}
	return false;
}

// Return true if any channel is waiting for a message acknowledgement
bool GCodes::WaitingForAcknowledgement() const noexcept
{
	for (const GCodeBuffer *gbp : gcodeSources)
	{
		if (gbp != nullptr && gbp->MachineState().waitingForAcknowledgement)
		{
			return true;
		}
	}
	return false;
}

// Return the current position of the file being printed in bytes.
// Unlike other methods returning file positions it never returns noFilePosition
FilePosition GCodes::GetFilePosition() const noexcept
{
#if HAS_LINUX_INTERFACE
	if (reprap.UsingLinuxInterface())
	{
		const FilePosition pos = (fileGCode->GetFilePosition() == noFilePosition)
				? lastFilePosition
					: fileGCode->GetFilePosition();
		return (pos == noFilePosition) ? 0 : pos;
	}
	else
#endif
	{

#if HAS_MASS_STORAGE
		const FileData& fileBeingPrinted = fileGCode->OriginalMachineState().fileState;
		if (!fileBeingPrinted.IsLive())
		{
			return 0;
		}

		const FilePosition pos = (fileGCode->IsDoingFileMacro())
				? fileBeingPrinted.GetPosition()					// the position before we started executing the macro
					: fileGCode->GetFilePosition();					// the actual position, allowing for bytes cached but not yet processed

		return (pos == noFilePosition) ? 0 : pos;
#else
		return 0;
#endif
	}
}

// Start running the config file
bool GCodes::RunConfigFile(const char* fileName) noexcept
{
	runningConfigFile = DoFileMacro(*triggerGCode, fileName, false);
	return runningConfigFile;
}

// Return true if the daemon is busy running config.g or a trigger file
bool GCodes::IsDaemonBusy() const noexcept
{
	return triggerGCode->IsDoingFile();
}

// Copy the feed rate etc. from the channel that was running config.g to the input channels
void GCodes::CheckFinishedRunningConfigFile(GCodeBuffer& gb) noexcept
{
	if (runningConfigFile)
	{
		gb.MachineState().GetPrevious()->CopyStateFrom(gb.MachineState());	// so that M83 etc. in  nested file don't get forgotten
		if (gb.MachineState().GetPrevious()->GetPrevious() == nullptr)
		{
			for (GCodeBuffer *gb2 : gcodeSources)
			{
				if (gb2 != nullptr && gb2 != &gb)
				{
					gb2->MachineState().CopyStateFrom(gb.MachineState());
				}
			}
			runningConfigFile = false;
		}
	}
}

// Set up to do the first of a possibly multi-tap probe
void GCodes::InitialiseTaps() noexcept
{
	tapsDone = 0;
	g30zHeightErrorSum = 0.0;
	g30zHeightErrorLowestDiff = 1000.0;
}

void GCodes::Spin() noexcept
{
	if (!active)
	{
		return;
	}

#ifdef SERIAL_AUX_DEVICE
	if (emergencyStopCommanded)
	{
		DoEmergencyStop();
		while (SERIAL_AUX_DEVICE.read() >= 0) { }
		emergencyStopCommanded = false;
		return;
	}
#endif

	CheckTriggers();
	CheckHeaterFault();
	CheckFilament();

	// Get the GCodeBuffer that we want to process a command from. Use round-robin scheduling but give priority to auto-pause.
	GCodeBuffer *gbp = autoPauseGCode;
	if (gbp->IsCompletelyIdle()
#if HAS_MASS_STORAGE || HAS_LINUX_INTERFACE
		&& !gbp->MachineState().DoingFile()
#endif
	   )	// if autoPause is not active
	{
		do
		{
			gbp = gcodeSources[nextGcodeSource];
			++nextGcodeSource;										// move on to the next gcode source ready for next time
			if (nextGcodeSource == ARRAY_SIZE(gcodeSources) - 1)	// the last one is autoPauseGCode, so don't do it again
			{
				nextGcodeSource = 0;
			}
		} while (gbp == nullptr);									// we must have at least one GCode source, so this can't loop indefinitely
	}
	GCodeBuffer& gb = *gbp;

	// Set up a buffer for the reply
	String<GCodeReplyLength> reply;

	if (gb.GetState() == GCodeState::normal)
	{
		if (gb.MachineState().messageAcknowledged)
		{
			const bool wasCancelled = gb.MachineState().messageCancelled;
			gb.PopState(false);                                                             // this could fail if the current macro has already been aborted
#if HAS_LINUX_INTERFACE
			if (reprap.UsingLinuxInterface())
			{
				// Send an empty response to DCS in order to let it pop its internal stack
				HandleReplyPreserveResult(gb, GCodeResult::ok, "");
			}
#endif

			if (wasCancelled)
			{
				if (gb.MachineState().GetPrevious() == nullptr)
				{
					StopPrint(StopPrintReason::userCancelled);
				}
				else
				{
					FileMacroCyclesReturn(gb);
				}
			}
		}
		else
		{
			StartNextGCode(gb, reply.GetRef());
		}
	}
	else
	{
		RunStateMachine(gb, reply.GetRef());                            // execute the state machine
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

// Start a new gcode, or continue to execute one that has already been started:
void GCodes::StartNextGCode(GCodeBuffer& gb, const StringRef& reply) noexcept
{
	if (IsPaused() && &gb == fileGCode && !gb.IsDoingFileMacro())
	{
		// We are paused, so don't process any more gcodes from the file being printed.
		// There is a potential issue here if fileGCode holds any locks, so unlock everything.
		UnlockAll(gb);
	}
	else if (gb.IsReady())
	{
		bool done;
		try
		{
			done = gb.CheckMetaCommand(reply);
		}
		catch (GCodeException& e)
		{
			e.GetMessage(reply, &gb);
			HandleReplyPreserveResult(gb, GCodeResult::error, reply.c_str());
			gb.Init();
			return;
		}

		if (done)
		{
			HandleReplyPreserveResult(gb, GCodeResult::ok, reply.c_str());
		}
		else
		{
			gb.SetFinished(ActOnCode(gb, reply));
		}
	}
	else if (gb.IsExecuting())
	{
		gb.SetFinished(ActOnCode(gb, reply));
	}
	else if (gb.IsDoingFile())
	{
		DoFilePrint(gb, reply);
	}
	else if (&gb == daemonGCode)
	{
		// Delay 1 second, then try to open and run daemon.g. No error if it is not found.
		if (   !reprap.IsProcessingConfig()
			&& gb.DoDwellTime(1000)
		   )
		{
			DoFileMacro(gb, DAEMON_G, false, -1);
		}
	}
	else
#if SUPPORT_SCANNER
		 if (!(&gb == usbGCode && reprap.GetScanner().IsRegistered()))
#endif
	{
		const bool gotCommand = (gb.GetNormalInput() != nullptr) && gb.GetNormalInput()->FillBuffer(&gb);
		if (gotCommand)
		{
			gb.DecodeCommand();
		}
#if HAS_LINUX_INTERFACE
		else if (reprap.UsingLinuxInterface())
		{
			reprap.GetLinuxInterface().FillBuffer(gb);
		}
#endif
	}
}

void GCodes::DoFilePrint(GCodeBuffer& gb, const StringRef& reply) noexcept
{
#if HAS_LINUX_INTERFACE
	if (reprap.UsingLinuxInterface())
	{
		if (!gb.IsDoingFileMacro() && gb.GetFilePosition() != noFilePosition)
		{
			lastFilePosition = gb.GetFilePosition();
		}

		if (gb.IsFileFinished())
		{
			gb.Init();								// mark buffer as empty

			if (gb.MachineState().GetPrevious() == nullptr)
			{
				// Finished printing SD card file.
				// We never get here if the file ends in M0 because CancelPrint gets called directly in that case.
				// Don't close the file until all moves have been completed, in case the print gets paused.
				// Also, this keeps the state as 'Printing' until the print really has finished.
				if (   LockMovementAndWaitForStandstill(gb)					// wait until movement has finished
					&& IsCodeQueueIdle()									// must also wait until deferred command queue has caught up
				   )
				{
					StopPrint(StopPrintReason::normalCompletion);
				}
			}
			else
			{
				// Finished a macro or finished processing config.g
				gb.MachineState().CloseFile();
				CheckFinishedRunningConfigFile(gb);
				const bool hadFileError = gb.MachineState().fileError;
				Pop(gb, false);
				gb.Init();

				// Prepare the response to the command that invoked the macro
				GCodeResult rslt = GCodeResult::ok;
				if (gb.GetState() == GCodeState::doingUnsupportedCode)
				{
					gb.SetState(GCodeState::normal);
					UnlockAll(gb);

					if (hadFileError)
					{
						gb.PrintCommand(reply);
						reply.cat(": Command is not supported");
						rslt = GCodeResult::warning;
					}
				}
				else if (gb.GetState() == GCodeState::doingUserMacro)
				{
					// M98 needs its own state in SBC mode to make sure the code does not completed before
					// the file has been requested. This will become obsolete in RRF 3.02.
					gb.SetState(GCodeState::normal);
					UnlockAll(gb);
				}
				else if (gb.GetState() == GCodeState::loadingFilament && hadFileError)
				{
					// Don't perform final filament assignment if the load macro could not be processed
					gb.SetState(GCodeState::normal);
				}

				// Reset the GCodeBuffer to non-binary input if necessary, so that if we are in Marlin mode we will send an OK response
				if (gb.GetState() == GCodeState::normal)
				{
					if (!gb.IsDoingFileMacro() && gb.GetNormalInput() != nullptr)
					{
						// Need to send a final response to the SBC if the request came from a code so it can pop its stack
						HandleReplyPreserveResult(gb, (gb.GetState() == GCodeState::normal) ? rslt : GCodeResult::ok, reply.c_str());
						gb.FinishedBinaryMode();
					}

					UnlockAll(gb);
					HandleReply(gb, rslt, reply.c_str());
				}
			}
		}
		else if (filamentChangePausePending && &gb == fileGCode && !gb.IsDoingFileMacro())
		{
			gb.PutAndDecode("M600");
			filamentChangePausePending = false;
		}
		else if (pausePending && &gb == fileGCode && !gb.IsDoingFileMacro())
		{
			gb.PutAndDecode("M226");
			pausePending = false;
		}
		else
		{
			reprap.GetLinuxInterface().FillBuffer(gb);
		}
	}
	else
#endif
	{
#if HAS_MASS_STORAGE
		FileData& fd = gb.MachineState().fileState;

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
				catch (GCodeException& e)
				{
					e.GetMessage(reply, &gb);
					HandleReplyPreserveResult(gb, GCodeResult::error, reply.c_str());
					gb.Init();
					AbortPrint(gb);
					break;
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
			break;

		case GCodeInputReadResult::error:
			AbortPrint(gb);
			break;

		case GCodeInputReadResult::noData:
			// We have reached the end of the file. Check for the last line of gcode not ending in newline.
			if (gb.FileEnded())							// append a newline if necessary and deal with any pending file write
			{
				bool done;
				try
				{
					done = gb.CheckMetaCommand(reply);
				}
				catch (GCodeException& e)
				{
					e.GetMessage(reply, &gb);
					HandleReply(gb, GCodeResult::error, reply.c_str());
					gb.Init();
					AbortPrint(gb);
					break;
				}

				if (done)
				{
					HandleReply(gb, GCodeResult::ok, reply.c_str());
				}
				else
				{
					gb.DecodeCommand();
					if (gb.IsReady())
					{
						gb.SetFinished(ActOnCode(gb, reply));
					}
				}
				break;
			}
			gb.Init();								// mark buffer as empty

			if (gb.MachineState().GetPrevious() == nullptr)
			{
				// Finished printing SD card file.
				// We never get here if the file ends in M0 because CancelPrint gets called directly in that case.
				// Don't close the file until all moves have been completed, in case the print gets paused.
				// Also, this keeps the state as 'Printing' until the print really has finished.
				if (   LockMovementAndWaitForStandstill(gb)					// wait until movement has finished
					&& IsCodeQueueIdle()									// must also wait until deferred command queue has caught up
				   )
				{
					StopPrint(StopPrintReason::normalCompletion);
				}
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
			break;
		}
#endif
	}
}

// Restore positions etc. when exiting simulation mode
void GCodes::EndSimulation(GCodeBuffer *gb) noexcept
{
	// Ending a simulation, so restore the position
	RestorePosition(simulationRestorePoint, gb);
	ToolOffsetTransform(currentUserPosition, moveBuffer.coords);
	reprap.GetMove().SetNewPosition(moveBuffer.coords, true);
	axesHomed = axesHomedBeforeSimulation;
}

// Check for and execute triggers
void GCodes::CheckTriggers() noexcept
{
	for (unsigned int i = 0; i < MaxTriggers; ++i)
	{
		if (!triggersPending.IsBitSet(i) && triggers[i].Check())
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
			triggersPending.ClearBit(lowestTriggerPending);			// clear the trigger
			DoEmergencyStop();
		}
		else if (!IsDaemonBusy() && triggerGCode->GetState() == GCodeState::normal)	// if we are not already executing a trigger or config.g
		{
			if (lowestTriggerPending == 1)
			{
				if (!IsReallyPrinting())
				{
					triggersPending.ClearBit(lowestTriggerPending);	// ignore a pause trigger if we are already paused or not printing
				}
				else if (LockMovement(*triggerGCode))					// need to lock movement before executing the pause macro
				{
					triggersPending.ClearBit(lowestTriggerPending);	// clear the trigger
					DoPause(*triggerGCode, PauseReason::trigger, "Print paused by external trigger");
				}
			}
			else
			{
				triggersPending.ClearBit(lowestTriggerPending);		// clear the trigger
				String<StringLength20> filename;
				filename.printf("trigger%u.g", lowestTriggerPending);
				DoFileMacro(*triggerGCode, filename.c_str(), true);
			}
		}
	}
}

// Check for and respond to filament errors
void GCodes::CheckFilament() noexcept
{
	if (   lastFilamentError != FilamentSensorStatus::ok			// check for a filament error
		&& IsReallyPrinting()
		&& autoPauseGCode->IsCompletelyIdle()
		&& LockMovement(*autoPauseGCode)							// need to lock movement before executing the pause macro
	   )
	{
		String<MediumStringLength> filamentErrorString;
		filamentErrorString.printf("Extruder %u reports %s", lastFilamentErrorExtruder, FilamentMonitor::GetErrorMessage(lastFilamentError));
		DoPause(*autoPauseGCode, PauseReason::filament, filamentErrorString.c_str());
		lastFilamentError = FilamentSensorStatus::ok;
		filamentErrorString.cat('\n');
		platform.Message(LogMessage, filamentErrorString.c_str());
	}
}

// Log a filament error. Called by Platform when a filament sensor reports an incorrect status and a print is in progress.
void GCodes::FilamentError(size_t extruder, FilamentSensorStatus fstat) noexcept
{
	if (lastFilamentError == FilamentSensorStatus::ok)
	{
		lastFilamentErrorExtruder = extruder;
		lastFilamentError = fstat;
	}
}

// Execute an emergency stop
void GCodes::DoEmergencyStop() noexcept
{
	reprap.EmergencyStop();
	Reset();
	platform.Message(GenericMessage, "Emergency Stop! Reset the controller to continue.\n");
}

// Pause the print. Before calling this, check that we are doing a file print that isn't already paused and get the movement lock.
void GCodes::DoPause(GCodeBuffer& gb, PauseReason reason, const char *msg) noexcept
{
	if (&gb == fileGCode)
	{
		// Pausing a file print because of a command in the file itself
		SavePosition(pauseRestorePoint, gb);
	}
	else
	{
		// Pausing a file print via another input source or for some other reason
		pauseRestorePoint.feedRate = fileGCode->MachineState().feedRate;				// set up the default

		// Save the spindle speeds if we are in CNC mode
		//TODO if we have to execute some more moves, there might be a pending M3 command in the GCodeQueue and we should allow for that
		SaveSpindleSpeeds(pauseRestorePoint);

		const bool movesSkipped = reprap.GetMove().PausePrint(pauseRestorePoint);		// tell Move we wish to pause the current print
		if (movesSkipped)
		{
			// The PausePrint call has filled in the restore point with machine coordinates
			ToolOffsetInverseTransform(pauseRestorePoint.moveCoords, currentUserPosition);	// transform the returned coordinates to user coordinates
			ClearMove();
		}
		else if (segmentsLeft != 0)
		{
			// We were not able to skip any moves, however we can skip the move that is waiting
			pauseRestorePoint.virtualExtruderPosition = moveBuffer.virtualExtruderPosition;
			pauseRestorePoint.filePos = moveBuffer.filePos;
			pauseRestorePoint.feedRate = moveBuffer.feedRate;
			pauseRestorePoint.proportionDone = (float)(totalSegments - segmentsLeft)/(float)totalSegments;
			pauseRestorePoint.initialUserX = moveBuffer.initialUserX;
			pauseRestorePoint.initialUserY = moveBuffer.initialUserY;
			ToolOffsetInverseTransform(pauseRestorePoint.moveCoords, currentUserPosition);	// transform the returned coordinates to user coordinates
			ClearMove();
		}
		else
		{
			// We were not able to skip any moves, and there is no move waiting
			pauseRestorePoint.feedRate = fileGCode->MachineState().feedRate;
			pauseRestorePoint.virtualExtruderPosition = virtualExtruderPosition;
			pauseRestorePoint.proportionDone = 0.0;

			// TODO: when using RTOS there is a possible race condition in the following,
			// because we might try to pause when a waiting move has just been added but before the gcode buffer has been re-initialised ready for the next command
			pauseRestorePoint.filePos = fileGCode->GetFilePosition();
#if SUPPORT_LASER || SUPPORT_IOBITS
			pauseRestorePoint.laserPwmOrIoBits = moveBuffer.laserPwmOrIoBits;
#endif
		}

		// Replace the paused machine coordinates by user coordinates, which we updated earlier if they were returned by Move::PausePrint
		for (size_t axis = 0; axis < numVisibleAxes; ++axis)
		{
			pauseRestorePoint.moveCoords[axis] = currentUserPosition[axis];
		}

#if HAS_LINUX_INTERFACE
		if (reprap.UsingLinuxInterface())
		{
			fileGCode->Init();															// clear the next move
			UnlockAll(*fileGCode);														// release any locks it had
		}
		else
		{
#endif
#if HAS_MASS_STORAGE
			// If we skipped any moves, reset the file pointer to the start of the first move we need to replay
			// The following could be delayed until we resume the print
			if (pauseRestorePoint.filePos != noFilePosition)
			{
				FileData& fdata = fileGCode->MachineState().fileState;
				if (fdata.IsLive())
				{
					fileGCode->RestartFrom(pauseRestorePoint.filePos);						// TODO we ought to restore the line number too, but currently we don't save it
					UnlockAll(*fileGCode);													// release any locks it had
				}
			}
#endif
#if HAS_LINUX_INTERFACE
		}
#endif

		codeQueue->PurgeEntries();

		if (reprap.Debug(moduleGcodes))
		{
			platform.MessageF(GenericMessage, "Paused print, file offset=%" PRIu32 "\n", pauseRestorePoint.filePos);
		}
	}

#if SUPPORT_LASER
	if (machineType == MachineType::laser)
	{
		moveBuffer.laserPwmOrIoBits.laserPwm = 0;		// turn off the laser when we start moving
	}
#endif

	SaveFanSpeeds();
	pauseRestorePoint.toolNumber = reprap.GetCurrentToolNumber();

#if HAS_MASS_STORAGE
	if (simulationMode == 0)
	{
		SaveResumeInfo(false);															// create the resume file so that we can resume after power down
	}
#endif

	gb.SetState((reason == PauseReason::filamentChange) ? GCodeState::filamentChangePause1 : GCodeState::pausing1);
	isPaused = true;

#if HAS_LINUX_INTERFACE
	if (reprap.UsingLinuxInterface())
	{
		// Get the print pause reason that is compatible with the API
		PrintPausedReason pauseReason = PrintPausedReason::user;
		switch (reason)
		{
		case PauseReason::gcode:
			pauseReason = PrintPausedReason::gcode;
			break;
		case PauseReason::filamentChange:
			pauseReason = PrintPausedReason::filamentChange;
			break;
		case PauseReason::trigger:
			pauseReason = PrintPausedReason::trigger;
			break;
		case PauseReason::heaterFault:
			pauseReason = PrintPausedReason::heaterFault;
			break;
		case PauseReason::filament:
			pauseReason = PrintPausedReason::filament;
			break;
# if HAS_SMART_DRIVERS
		case PauseReason::stall:
			pauseReason = PrintPausedReason::stall;
			break;
# endif
# if HAS_VOLTAGE_MONITOR
		case PauseReason::lowVoltage:
			pauseReason = PrintPausedReason::lowVoltage;
			break;
# endif
		default:
			pauseReason = PrintPausedReason::user;
			break;
		}

		// Prepare notification for the Linux side
		lastFilePosition = pauseRestorePoint.filePos;
		reprap.GetLinuxInterface().SetPauseReason(pauseRestorePoint.filePos, pauseReason);
	}
#endif

	if (msg != nullptr)
	{
		platform.SendAlert(GenericMessage, msg, "Printing paused", 1, 0.0, AxesBitmap());
	}
}

// Check if a pause is pending, action it if so
void GCodes::CheckForDeferredPause(GCodeBuffer& gb) noexcept
{
	if (&gb == fileGCode && !gb.IsDoingFileMacro())
	{
		if (filamentChangePausePending)
		{
			gb.PutAndDecode("M600");
			filamentChangePausePending = false;
		}
		else if (pausePending)
		{
			gb.PutAndDecode("M226");
			pausePending = false;
		}
	}
}

bool GCodes::IsPaused() const noexcept
{
	return isPaused && !IsPausing() && !IsResuming();
}

bool GCodes::IsPausing() const noexcept
{
	return pausePending || fileGCode->IsPausing() || triggerGCode->IsPausing() || autoPauseGCode->IsPausing() || daemonGCode->IsPausing();
}

bool GCodes::IsResuming() const noexcept
{
	return fileGCode->IsResuming();
}

bool GCodes::IsRunning() const noexcept
{
	return !IsPaused() && !IsPausing() && !IsResuming();
}

// Return true if we are printing from SD card and not pausing, paused or resuming
// TODO make this independent of PrintMonitor
bool GCodes::IsReallyPrinting() const noexcept
{
	return reprap.GetPrintMonitor().IsPrinting() && IsRunning();
}

// Return true if the SD card print is waiting for a heater to reach temperature
bool GCodes::IsHeatingUp() const noexcept
{
	int num;
	return fileGCode->IsExecuting()
		&& fileGCode->GetCommandLetter() == 'M'
		&& ((num = fileGCode->GetCommandNumber()) == 109 || num == 116 || num == 190 || num == 191);
}

#if HAS_VOLTAGE_MONITOR || HAS_STALL_DETECT

// Do an emergency pause following loss of power or a motor stall returning true if successful, false if needs to be retried
bool GCodes::DoEmergencyPause() noexcept
{
	if (!autoPauseGCode->IsCompletelyIdle())
	{
		return false;							// we can't pause if the auto pause thread is busy already
	}

	// Save the resume info, stop movement immediately and run the low voltage pause script to lift the nozzle etc.
	GrabMovement(*autoPauseGCode);

	// When we use RTOS there is a possible race condition in the following, because we might try to pause when a waiting move has just been added
	// but before the gcode buffer has been re-initialised ready for the next command. So start a critical section.
	TaskCriticalSectionLocker lock;

	const bool movesSkipped = reprap.GetMove().LowPowerOrStallPause(pauseRestorePoint);
	if (movesSkipped)
	{
		// The PausePrint call has filled in the restore point with machine coordinates
		ToolOffsetInverseTransform(pauseRestorePoint.moveCoords, currentUserPosition);	// transform the returned coordinates to user coordinates
		ClearMove();
	}
	else if (segmentsLeft != 0 && moveBuffer.filePos != noFilePosition)
	{
		// We were not able to skip any moves, however we can skip the remaining segments of this current move
		ToolOffsetInverseTransform(moveBuffer.initialCoords, currentUserPosition);
		pauseRestorePoint.feedRate = moveBuffer.feedRate;
		pauseRestorePoint.virtualExtruderPosition = moveBuffer.virtualExtruderPosition;
		pauseRestorePoint.filePos = moveBuffer.filePos;
		pauseRestorePoint.proportionDone = (float)(totalSegments - segmentsLeft)/(float)totalSegments;
		pauseRestorePoint.initialUserX = moveBuffer.initialUserX;
		pauseRestorePoint.initialUserY = moveBuffer.initialUserY;
#if SUPPORT_LASER || SUPPORT_IOBITS
		pauseRestorePoint.laserPwmOrIoBits = moveBuffer.laserPwmOrIoBits;
#endif
		ClearMove();
	}
	else
	{
		// We were not able to skip any moves, and if there is a move waiting then we can't skip that one either
		pauseRestorePoint.feedRate = fileGCode->MachineState().feedRate;
		pauseRestorePoint.virtualExtruderPosition = virtualExtruderPosition;

		pauseRestorePoint.filePos = fileGCode->GetFilePosition();
		pauseRestorePoint.proportionDone = 0.0;

#if SUPPORT_LASER || SUPPORT_IOBITS
		pauseRestorePoint.laserPwmOrIoBits = moveBuffer.laserPwmOrIoBits;
#endif
	}

	codeQueue->PurgeEntries();

	// Replace the paused machine coordinates by user coordinates, which we updated earlier
	for (size_t axis = 0; axis < numVisibleAxes; ++axis)
	{
		pauseRestorePoint.moveCoords[axis] = currentUserPosition[axis];
	}

	SaveFanSpeeds();
	pauseRestorePoint.toolNumber = reprap.GetCurrentToolNumber();
	isPaused = true;

	return true;
}

#endif

#if HAS_VOLTAGE_MONITOR

// Try to pause the current SD card print, returning true if successful, false if needs to be called again
bool GCodes::LowVoltagePause() noexcept
{
	if (simulationMode != 0)
	{
		return true;								// ignore the low voltage indication
	}

	reprap.GetHeat().SuspendHeaters(true);			// turn the heaters off to conserve power for the motors to execute the pause
	if (IsResuming())
	{
		// This is an unlucky situation, because the resume macro is probably being run, which will probably lower the head back on to the print.
		// It may well be that the power loss will prevent the resume macro being completed. If not, try again when the print has been resumed.
		return false;
	}

	if (IsPausing())
	{
		// We are in the process of pausing already, so the resume info has already been saved.
		// With luck the retraction and lifting of the head in the pause.g file has been done already.
		return true;
	}

	if (IsPaused())
	{
		// Resume info has already been saved, and resuming will be prevented while the power is low
		return true;
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
			autoPauseGCode->PutAndDecode(powerFailScript);
		}
		autoPauseGCode->SetState(GCodeState::powerFailPausing1);
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
	if (isPaused && isPowerFailPaused)
	{
		isPowerFailPaused = false;					// pretend it's a normal pause
		// Run resurrect.g automatically
		//TODO qq;
		//platform.Message(LoggedGenericMessage, "Print auto-resumed\n");
	}
	return true;
}

#endif

#if HAS_SMART_DRIVERS

// Pause the print because the specified driver has reported a stall
bool GCodes::PauseOnStall(DriversBitmap stalledDrivers) noexcept
{
	if (!IsReallyPrinting())
	{
		return true;								// if not printing, acknowledge it but take no action
	}
	if (!autoPauseGCode->IsCompletelyIdle())
	{
		return false;								// can't handle it yet
	}
	if (!LockMovement(*autoPauseGCode))
	{
		return false;
	}

	String<MediumStringLength> stallErrorString;
	stallErrorString.printf("Stall detected on driver(s)");
	ListDrivers(stallErrorString.GetRef(), stalledDrivers);
	DoPause(*autoPauseGCode, PauseReason::stall, stallErrorString.c_str());
	stallErrorString.cat('\n');
	platform.Message(LogMessage, stallErrorString.c_str());
	return true;
}

// Re-home and resume the print because the specified driver has reported a stall
bool GCodes::ReHomeOnStall(DriversBitmap stalledDrivers) noexcept
{
	if (!IsReallyPrinting())
	{
		return true;								// if not printing, acknowledge it but take no action
	}
	if (!DoEmergencyPause())
	{
		return false;								// can't handle it yet
	}

	autoPauseGCode->SetState(GCodeState::resuming1); // set up to resume after rehoming
	DoFileMacro(*autoPauseGCode, REHOME_G, true); 	// run the SD card rehome-and-resume script
	return true;
}

#endif

#if HAS_MASS_STORAGE

void GCodes::SaveResumeInfo(bool wasPowerFailure) noexcept
{
#if HAS_LINUX_INTERFACE
	if (reprap.UsingLinuxInterface())
	{
		return;			// we can't yet save to the Pi
	}
#endif
	const char* const printingFilename = reprap.GetPrintMonitor().GetPrintingFilename();
	if (printingFilename != nullptr)
	{
		FileStore * const f = platform.OpenSysFile(RESUME_AFTER_POWER_FAIL_G, OpenMode::write);
		if (f == nullptr)
		{
			platform.MessageF(ErrorMessage, "Failed to create file %s\n", RESUME_AFTER_POWER_FAIL_G);
		}
		else
		{
			String<FormatStringLength> buf;

			// Write the header comment
			buf.printf("; File \"%s\" resume print after %s", printingFilename, (wasPowerFailure) ? "power failure" : "print paused");
			tm timeInfo;
			if (platform.GetDateTime(timeInfo))
			{
				buf.catf(" at %04u-%02u-%02u %02u:%02u",
								timeInfo.tm_year + 1900, timeInfo.tm_mon + 1, timeInfo.tm_mday, timeInfo.tm_hour, timeInfo.tm_min);
			}
			buf.cat("\nG21\n");												// set units to mm because we will be writing positions in mm
			bool ok = f->Write(buf.c_str())
					&& reprap.GetHeat().WriteBedAndChamberTempSettings(f)	// turn on bed and chamber heaters
					&& reprap.GetMove().WriteResumeSettings(f);				// load grid, if we are using one
			if (ok)
			{
				// Write a G92 command to say where the head is. This is useful if we can't Z-home the printer with a print on the bed and the Z steps/mm is high.
				// The paused coordinates include any tool offsets and baby step offsets, so remove those.
				// Also ensure that no tool is selected, in case config.g selects one and it has an offset.
				buf.copy("T-1 P0\nG92");
				for (size_t axis = 0; axis < numVisibleAxes; ++axis)
				{
					const float totalOffset = currentBabyStepOffsets[axis] - GetCurrentToolOffset(axis);
					buf.catf(" %c%.3f", axisLetters[axis], (double)(pauseRestorePoint.moveCoords[axis] - totalOffset));
				}
				buf.cat("\nG60 S1\n");										// save the coordinates as restore point 1 too
				ok = f->Write(buf.c_str());
			}
			if (ok)
			{
				ok = reprap.WriteToolSettings(f);							// set tool temperatures, tool mix ratios etc. and select the current tool without running tool change files
			}
			if (ok)
			{
				buf.printf("M98 P\"%s\"\n", RESUME_PROLOGUE_G);				// call the prologue
				ok = f->Write(buf.c_str());
			}
			if (ok)
			{
				buf.copy("M116\nM290");										// wait for temperatures and start writing baby stepping offsets
				for (size_t axis = 0; axis < numVisibleAxes; ++axis)
				{
					buf.catf(" %c%.3f", axisLetters[axis], (double)GetTotalBabyStepOffset(axis));
				}
				buf.cat(" R0\n");
				ok = f->Write(buf.c_str());									// write baby stepping offsets
			}

			// Now that we have homed, we can run the tool change files for the current tool
			const Tool * const ct = reprap.GetCurrentTool();
			if (ok && ct != nullptr)
			{
				buf.printf("T-1 P0\nT%u P6\n", ct->Number());				// deselect the current tool without running tfree, and select it running tpre and tpost
				ok = f->Write(buf.c_str());									// write tool selection
			}

#if SUPPORT_WORKPLACE_COORDINATES
			// Restore the coordinate offsets of all workplaces
			if (ok)
			{
				ok = WriteWorkplaceCoordinates(f);
			}

			if (ok)
			{
				// Switch to the correct workplace. 'currentCoordinateSystem' is 0-based.
				if (currentCoordinateSystem <= 5)
				{
					buf.printf("G%u\n", 54 + currentCoordinateSystem);
				}
				else
				{
					buf.printf("G59.%u\n", currentCoordinateSystem - 5);
				}
				ok = f->Write(buf.c_str());
			}
#else
			if (ok)
			{
				buf.copy("M206");
				for (size_t axis = 0; axis < numVisibleAxes; ++axis)
				{
					buf.catf(" %c%.3f", axisLetters[axis], (double)-workplaceCoordinates[0][axis]);
				}
				buf.cat('\n');
				ok = f->Write(buf.c_str());
			}
#endif

			if (ok && fileGCode->OriginalMachineState().volumetricExtrusion)
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
				buf.printf("M106 S%.2f\n", (double)lastDefaultFanSpeed);
				ok = f->Write(buf.c_str())									// set the speed of the print fan after we have selected the tool
					&& reprap.GetFansManager().WriteFanSettings(f);			// set the speeds of all non-thermostatic fans after setting the default fan speed
			}
			if (ok)
			{
				buf.printf("M116\nG92 E%.5f\n%s\n", (double)virtualExtruderPosition, (fileGCode->OriginalMachineState().drivesRelative) ? "M83" : "M82");
				ok = f->Write(buf.c_str());									// write virtual extruder position and absolute/relative extrusion flag
			}
			if (ok)
			{
				buf.printf("M23 \"%s\"\nM26 S%" PRIu32, printingFilename, pauseRestorePoint.filePos);
				if (pauseRestorePoint.proportionDone > 0.0)
				{
					buf.catf(" P%.3f X%.3f Y%.3f", (double)pauseRestorePoint.proportionDone, (double)pauseRestorePoint.initialUserX, (double)pauseRestorePoint.initialUserY);
				}
				buf.cat('\n');
				ok = f->Write(buf.c_str());									// write filename and file position, and if necessary proportion done and initial XY position
			}
			if (ok)
			{
				// Build the commands to restore the head position. These assume that we are working in mm.
				// Start with a vertical move to 2mm above the final Z position
				buf.printf("G0 F6000 Z%.3f\n", (double)(pauseRestorePoint.moveCoords[Z_AXIS] + 2.0));

				// Now set all the other axes
				buf.cat("G0 F6000");
				for (size_t axis = 0; axis < numVisibleAxes; ++axis)
				{
					if (axis != Z_AXIS)
					{
						buf.catf(" %c%.3f", axisLetters[axis], (double)pauseRestorePoint.moveCoords[axis]);
					}
				}

				// Now move down to the correct Z height
				buf.catf("\nG0 F6000 Z%.3f\n", (double)pauseRestorePoint.moveCoords[Z_AXIS]);

				// Set the feed rate
				buf.catf("G1 F%.1f", (double)(pauseRestorePoint.feedRate * MinutesToSeconds));
#if SUPPORT_LASER
				if (machineType == MachineType::laser)
				{
					buf.catf(" S%u", (unsigned int)pauseRestorePoint.laserPwmOrIoBits.laserPwm);
				}
				else
				{
#endif
#if SUPPORT_IOBITS
					buf.catf(" P%u", (unsigned int)pauseRestorePoint.laserPwmOrIoBits.ioBits);
#endif
#if SUPPORT_LASER
				}
#endif
				buf.cat("\n");
				ok = f->Write(buf.c_str());									// restore feed rate and output bits or laser power
			}

			if (ok)
			{
				buf.printf("%s\nM24\n", (fileGCode->OriginalMachineState().usingInches) ? "G20" : "G21");
				ok = f->Write(buf.c_str());									// restore inches/mm and resume printing
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

#endif

void GCodes::Diagnostics(MessageType mtype) noexcept
{
	platform.Message(mtype, "=== GCodes ===\n");
	platform.MessageF(mtype, "Segments left: %u\n", segmentsLeft);
	const GCodeBuffer * const movementOwner = resourceOwners[MoveResource];
	platform.MessageF(mtype, "Movement lock held by %s\n", (movementOwner == nullptr) ? "null" : movementOwner->GetChannel().ToString());

	for (GCodeBuffer *gb : gcodeSources)
	{
		if (gb != nullptr)
		{
			gb->Diagnostics(mtype);
		}
	}

	codeQueue->Diagnostics(mtype);
}

// Lock movement and wait for pending moves to finish.
// As a side-effect it loads moveBuffer with the last position and feedrate for you.
bool GCodes::LockMovementAndWaitForStandstill(GCodeBuffer& gb) noexcept
{
	// Lock movement to stop another source adding moves to the queue
	if (!LockMovement(gb))
	{
		return false;
	}

	// Last one gone?
	if (segmentsLeft != 0)
	{
		return false;
	}

	// Wait for all the queued moves to stop so we get the actual last position
	if (!reprap.GetMove().AllMovesAreFinished(true))
	{
		return false;
	}

	gb.MotionStopped();								// must do this after we have finished waiting, so that we don't stop waiting when executing G4

	// Get the current positions. These may not be the same as the ones we remembered from last time if we just did a special move.
	UpdateCurrentUserPosition();
	return true;
}

// Save (some of) the state of the machine for recovery in the future.
bool GCodes::Push(GCodeBuffer& gb, bool withinSameFile)
{
	const bool ok = gb.PushState(withinSameFile);
	if (!ok)
	{
		platform.Message(ErrorMessage, "Push(): stack overflow\n");
		AbortPrint(gb);
	}
	return ok;
}

// Recover a saved state
void GCodes::Pop(GCodeBuffer& gb, bool withinSameFile)
{
	if (!gb.PopState(withinSameFile))
	{
		platform.Message(ErrorMessage, "Pop(): stack underflow\n");
	}
}

// Set up the extrusion and feed rate of a move for the Move class
// 'moveBuffer.moveType' and 'moveBuffer.isCoordinated' must be set up before calling this
// 'isPrintingMove' is true if there is any axis movement
// Returns nullptr if this gcode is valid so far, or an error message if it should be discarded
const char * GCodes::LoadExtrusionAndFeedrateFromGCode(GCodeBuffer& gb, bool isPrintingMove)
{
	// Deal with feed rate, also determine whether M220 and M221 speed and extrusion factors apply to this move
	if (moveBuffer.isCoordinated || machineType == MachineType::fff)
	{
		moveBuffer.applyM220M221 = (moveBuffer.moveType == 0 && isPrintingMove && !gb.IsDoingFileMacro());
		if (gb.Seen(feedrateLetter))
		{
			gb.MachineState().feedRate = gb.GetDistance() * SecondsToMinutes;	// update requested speed, not allowing for speed factor
		}
		moveBuffer.feedRate = (moveBuffer.applyM220M221)
								? speedFactor * gb.MachineState().feedRate
								: gb.MachineState().feedRate;
		moveBuffer.usingStandardFeedrate = true;
	}
	else
	{
		moveBuffer.applyM220M221 = false;
		moveBuffer.feedRate = DefaultG0FeedRate;					// use maximum feed rate, the M203 parameters will limit it
		moveBuffer.usingStandardFeedrate = false;
	}

	// Zero every extruder drive as some drives may not be moved
	for (size_t drive = numTotalAxes; drive < MaxAxesPlusExtruders; drive++)
	{
		moveBuffer.coords[drive] = 0.0;
	}
	moveBuffer.hasExtrusion = false;
	moveBuffer.virtualExtruderPosition = virtualExtruderPosition;	// save this before we update it
	ExtrudersBitmap extrudersMoving;

	// Check if we are extruding
	if (gb.Seen(extrudeLetter))							// DC 2018-08-07: at E3D's request, extrusion is now recognised even on uncoordinated moves
	{
		// Check that we have a tool to extrude with
		Tool* const tool = reprap.GetCurrentTool();
		if (tool == nullptr)
		{
			displayNoToolWarning = true;
			return nullptr;
		}

		moveBuffer.hasExtrusion = true;
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
				if (gb.MachineState().drivesRelative)
				{
					requestedExtrusionAmount = moveArg;
				}
				else
				{
					requestedExtrusionAmount = moveArg - virtualExtruderPosition;
					virtualExtruderPosition = moveArg;
				}

				// rawExtruderTotal is used to calculate print progress, so it must be based on the requested extrusion from the slicer
				// before accounting for mixing, extrusion factor etc.
				// We used to have 'isPrintingMove &&' in the condition too, but this excluded wipe-while-retracting moves, so it gave wrong results for % print complete.
				// We still exclude extrusion during tool changing and other macros, because that is extrusion not known to the slicer.
				if (moveBuffer.moveType == 0 && !gb.IsDoingFileMacro())
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
						const int extruder = tool->Drive(eDrive);
						float extrusionAmount = requestedExtrusionAmount * thisMix;
						if (gb.MachineState().volumetricExtrusion)
						{
							extrusionAmount *= volumetricExtrusionFactors[extruder];
						}
						if (moveBuffer.moveType == 0 && !gb.IsDoingFileMacro())
						{
							rawExtruderTotalByDrive[extruder] += extrusionAmount;
						}

						moveBuffer.coords[ExtruderToLogicalDrive(extruder)] = (moveBuffer.applyM220M221)
																				? extrusionAmount * extrusionFactors[extruder]
																				: extrusionAmount;
						extrudersMoving.SetBit(extruder);
					}
				}
				if (!isPrintingMove && moveBuffer.usingStandardFeedrate)
				{
					// For E3D: If the total mix ratio is greater than 1.0 then we should scale the feed rate accordingly, e.g. for dual serial extruder drives
					moveBuffer.feedRate *= totalMix;
				}
			}
			else
			{
				// Individual extrusion amounts have been provided. This is supported in relative extrusion mode only.
				// Note, if this is an extruder-only movement then the feed rate will apply to the total of all active extruders
				if (gb.MachineState().drivesRelative)
				{
					for (size_t eDrive = 0; eDrive < eMoveCount; eDrive++)
					{
						const int extruder = tool->Drive(eDrive);
						float extrusionAmount = gb.ConvertDistance(eMovement[eDrive]);
						if (extrusionAmount != 0.0)
						{
							if (gb.MachineState().volumetricExtrusion)
							{
								extrusionAmount *= volumetricExtrusionFactors[extruder];
							}

							if (moveBuffer.moveType == 0 && !gb.IsDoingFileMacro())
							{
								rawExtruderTotalByDrive[extruder] += extrusionAmount;
								rawExtruderTotal += extrusionAmount;
							}
							moveBuffer.coords[ExtruderToLogicalDrive(extruder)] = (moveBuffer.applyM220M221)
																					? extrusionAmount * extrusionFactors[extruder]
																					: extrusionAmount;
							extrudersMoving.SetBit(extruder);
						}
					}
				}
				else
				{
					return "Multiple E parameters in G1 commands are not supported in absolute extrusion mode";
				}
			}
		}
	}

	if (moveBuffer.moveType == 1)
	{
		if (!platform.GetEndstops().EnableExtruderEndstops(extrudersMoving))
		{
			return "Failed to enable extruder endstops";
		}
	}

	return nullptr;
}

// Check that enough axes have been homed, returning true if insufficient axes homed
bool GCodes::CheckEnoughAxesHomed(AxesBitmap axesMoved) noexcept
{
	return (reprap.GetMove().GetKinematics().MustBeHomedAxes(axesMoved, noMovesBeforeHoming) & ~axesHomed).IsNonEmpty();
}

// Execute a straight move
// If not ready, return false
// If we can't execute the move, return true with 'err' set to the error message
// Else return true with 'err' left alone (it is set to nullptr on entry)
// We have already acquired the movement lock and waited for the previous move to be taken.
bool GCodes::DoStraightMove(GCodeBuffer& gb, bool isCoordinated, const char *& err)
{
	if (moveFractionToSkip > 0.0)
	{
		moveBuffer.initialUserX = restartInitialUserX;
		moveBuffer.initialUserY = restartInitialUserY;
	}
	else
	{
		moveBuffer.initialUserX = currentUserPosition[X_AXIS];
		moveBuffer.initialUserY = currentUserPosition[Y_AXIS];
	}

	// Set up default move parameters
	moveBuffer.isCoordinated = isCoordinated;
	moveBuffer.checkEndstops = false;
	moveBuffer.reduceAcceleration = false;
	moveBuffer.moveType = 0;
	moveBuffer.tool = reprap.GetCurrentTool();
	moveBuffer.usePressureAdvance = false;
	axesToSenseLength.Clear();

	// Check to see if the move is a 'homing' move that endstops are checked on.
	// We handle H1 parameters affecting extrusion elsewhere.
	if (gb.Seen('H') || (machineType != MachineType::laser && gb.Seen('S')))
	{
		const int ival = gb.GetIValue();
		if (ival >= 1 && ival <= 3)
		{
			if (!LockMovementAndWaitForStandstill(gb))
			{
				return false;
			}
			moveBuffer.moveType = ival;
			moveBuffer.tool = nullptr;
		}
		if (!gb.Seen('H'))
		{
			platform.Message(MessageType::WarningMessage, "Obsolete use of S parameter on G1 command. Use H parameter instead.\n");
		}
	}

	// Check for 'R' parameter to move relative to a restore point
	const RestorePoint * rp = nullptr;
	if (moveBuffer.moveType == 0 && gb.Seen('R'))
	{
		const uint32_t rParam = gb.GetUIValue();
		if (rParam < ARRAY_SIZE(numberedRestorePoints))
		{
			rp = &numberedRestorePoints[rParam];
		}
		else
		{
			err = "G0/G1: bad restore point number";
			return true;
		}
	}

	// Check for laser power setting or IOBITS
#if SUPPORT_LASER || SUPPORT_IOBITS
	if (rp != nullptr)
	{
		moveBuffer.laserPwmOrIoBits = rp->laserPwmOrIoBits;
	}
# if SUPPORT_LASER
	else if (machineType == MachineType::laser)
	{
		if (gb.Seen('S'))
		{
			moveBuffer.laserPwmOrIoBits.laserPwm = ConvertLaserPwm(gb.GetFValue());
		}
		else if (moveBuffer.moveType != 0)
		{
			moveBuffer.laserPwmOrIoBits.laserPwm = 0;			// it's a homing move or similar, so turn the laser off
		}
		else if (laserPowerSticky)
		{
			// Leave the laser PWM alone because this is what LaserWeb expects. If it is an uncoordinated move then the motion system will turn the laser off.
			// This means that after a G0 move, the next G1 move will default to the same power as the previous G1 move, as LightBurn expects.
		}
		else
		{
			moveBuffer.laserPwmOrIoBits.laserPwm = 0;
		}
	}
# endif
# if SUPPORT_IOBITS
	else
	{
		// Update the iobits parameter
		if (gb.Seen('P'))
		{
			moveBuffer.laserPwmOrIoBits.ioBits = (IoBits_t)gb.GetIValue();
		}
		else
		{
			// Leave moveBuffer.ioBits alone so that we keep the previous value
		}
	}
# endif
#endif

	if (moveBuffer.moveType != 0)
	{
		// This may be a raw motor move, in which case we need the current raw motor positions in moveBuffer.coords.
		// If it isn't a raw motor move, it will still be applied without axis or bed transform applied,
		// so make sure the initial coordinates don't have those either to avoid unwanted Z movement.
		reprap.GetMove().GetCurrentUserPosition(moveBuffer.coords, moveBuffer.moveType, reprap.GetCurrentTool());
	}

	// Set up the initial coordinates
	memcpy(moveBuffer.initialCoords, moveBuffer.coords, numVisibleAxes * sizeof(moveBuffer.initialCoords[0]));

	// Deal with axis movement
	const float initialXY[2] = { currentUserPosition[X_AXIS], currentUserPosition[Y_AXIS] };
	AxesBitmap axesMentioned;
	for (size_t axis = 0; axis < numVisibleAxes; axis++)
	{
		if (gb.Seen(axisLetters[axis]))
		{
			// If it is a special move on a delta, movement must be relative.
			if (moveBuffer.moveType != 0 && !gb.MachineState().axesRelative && reprap.GetMove().GetKinematics().GetKinematicsType() == KinematicsType::linearDelta)
			{
				err = "G0/G1: attempt to move individual motors of a delta machine to absolute positions";
				return true;
			}

			axesMentioned.SetBit(axis);
			const float moveArg = gb.GetDistance();
			if (moveBuffer.moveType != 0)
			{
				// Special moves update the move buffer directly, bypassing the user coordinates
				if (gb.MachineState().axesRelative)
				{
					moveBuffer.coords[axis] += moveArg * (1.0 - moveFractionToSkip);
				}
				else
				{
					moveBuffer.coords[axis] = moveArg;
				}
			}
			else if (rp != nullptr)
			{
				currentUserPosition[axis] = moveArg + rp->moveCoords[axis];
				// When a restore point is being used (G1 R parameter) then we used to set any coordinates that were not mentioned to the restore point values.
				// But that causes issues for tool change on IDEX machines because we end up restoring the U axis when we shouldn't.
				// So we no longer do that, and the user must mention any axes that he wants restored e.g. G1 R2 X0 Y0.
			}
			else if (gb.MachineState().axesRelative)
			{
				currentUserPosition[axis] += moveArg * (1.0 - moveFractionToSkip);
			}
			else if (gb.MachineState().g53Active)
			{
				currentUserPosition[axis] = moveArg + GetCurrentToolOffset(axis);	// g53 ignores tool offsets as well as workplace coordinates
			}
			else if (gb.MachineState().runningSystemMacro)
			{
				currentUserPosition[axis] = moveArg;								// don't apply workplace offsets to commands in system macros
			}
			else
			{
				currentUserPosition[axis] = moveArg + GetWorkplaceOffset(axis);
			}
		}
	}

	// Check enough axes have been homed
	switch (moveBuffer.moveType)
	{
	case 0:
		if (!doingManualBedProbe && CheckEnoughAxesHomed(axesMentioned))
		{
			err = "G0/G1: insufficient axes homed";
			return true;
		}
		break;

	case 3:
		axesToSenseLength = axesMentioned & AxesBitmap::MakeLowestNBits(numTotalAxes);
		// no break
	case 1:
		if (!platform.GetEndstops().EnableAxisEndstops(axesMentioned & AxesBitmap::MakeLowestNBits(numTotalAxes), moveBuffer.moveType == 1))
		{
			err = "Failed to enable endstops";
			return true;
		}
		moveBuffer.checkEndstops = true;
		break;

	case 2:
	default:
		break;
	}

	err = LoadExtrusionAndFeedrateFromGCode(gb, axesMentioned.IsNonEmpty());	// for type 1 moves, this must be called after calling EnableAxisEndstops, because EnableExtruderEndstop assumes that
	if (err != nullptr)
	{
		return true;
	}

	const bool isPrintingMove = moveBuffer.hasExtrusion && axesMentioned.IsNonEmpty();
	if (buildObjects.IsFirstMoveSincePrintingResumed())							// if this is the first move after skipping an object
	{
		if (isPrintingMove)
		{
			if (TravelToStartPoint(gb))											// don't start a printing move from the wrong place
			{
				buildObjects.DoneMoveSincePrintingResumed();
			}
			return false;
		}
		else if (axesMentioned.IsNonEmpty())									// don't count G1 Fxxx as a travel move
		{
			buildObjects.DoneMoveSincePrintingResumed();
		}
	}

#if TRACK_OBJECT_NAMES
	if (isPrintingMove)
	{
		// Update the object coordinates limits. For efficiency, we only update the final coordinate.
		// Except in the case of a straight line that is only one extrusion width wide, this is sufficient.
		buildObjects.UpdateObjectCoordinates(currentUserPosition, axesMentioned);
	}
#endif

	// Set up the move. We must assign segmentsLeft last, so that when Move runs as a separate task the move won't be picked up by the Move process before it is complete.
	// Note that if this is an extruder-only move, we don't do axis movements to allow for tool offset changes, we defer those until an axis moves.
	if (moveBuffer.moveType != 0)
	{
		// It's a raw motor move, so do it in a single segment and wait for it to complete
		totalSegments = 1;
		gb.SetState(GCodeState::waitingForSpecialMoveToComplete);
	}
	else if (axesMentioned.IsEmpty())
	{
		totalSegments = 1;
	}
	else
	{
		if (&gb == fileGCode && !gb.IsDoingFileMacro() && moveBuffer.hasExtrusion && axesMentioned.Intersects(XyAxes))
		{
			lastPrintingMoveHeight = currentUserPosition[Z_AXIS];
		}

		ToolOffsetTransform(currentUserPosition, moveBuffer.coords, axesMentioned);
																				// apply tool offset, baby stepping, Z hop and axis scaling
		AxesBitmap effectiveAxesHomed = axesHomed;
		if (doingManualBedProbe)
		{
			effectiveAxesHomed.ClearBit(Z_AXIS);								// if doing a manual Z probe, don't limit the Z movement
		}

		const LimitPositionResult lp = reprap.GetMove().GetKinematics().LimitPosition(moveBuffer.coords, moveBuffer.initialCoords, numVisibleAxes, effectiveAxesHomed, moveBuffer.isCoordinated, limitAxes);
		switch (lp)
		{
		case LimitPositionResult::adjusted:
		case LimitPositionResult::adjustedAndIntermediateUnreachable:
			if (machineType != MachineType::fff)
			{
				err = "G0/G1: target position outside machine limits";		// it's a laser or CNC so this is a definite error
				return true;
			}
			ToolOffsetInverseTransform(moveBuffer.coords, currentUserPosition);	// make sure the limits are reflected in the user position
			if (lp == LimitPositionResult::adjusted)
			{
				break;														// we can reach the intermediate positions, so nothing more to do
			}
			// no break

		case LimitPositionResult::intermediateUnreachable:
			if (   moveBuffer.isCoordinated
				&& (   (machineType == MachineType::fff && !moveBuffer.hasExtrusion)
#if SUPPORT_LASER || SUPPORT_IOBITS
					|| (machineType == MachineType::laser && moveBuffer.laserPwmOrIoBits.laserPwm == 0)
#endif
				   )
			   )
			{
				// It's a coordinated travel move on a 3D printer or laser cutter, so see whether an uncoordinated move will work
				const LimitPositionResult lp2 = reprap.GetMove().GetKinematics().LimitPosition(moveBuffer.coords, moveBuffer.initialCoords, numVisibleAxes, effectiveAxesHomed, false, limitAxes);
				if (lp2 == LimitPositionResult::ok)
				{
					moveBuffer.isCoordinated = false;						// change it to an uncoordinated move
					break;
				}
			}
			err = "G0/G1: target position not reachable from current position";		// we can't bring the move within limits, so this is a definite error
			return true;

		case LimitPositionResult::ok:
		default:
			break;
		}

		// If we are emulating Marlin for nanoDLP then we need to set a special end state
		if (gb.MachineState().compatibility == Compatibility::NanoDLP && !DoingFileMacro())
		{
			gb.SetState(GCodeState::waitingForSpecialMoveToComplete);
		}

		// Flag whether we should use pressure advance, if there is any extrusion in this move.
		// We assume it is a normal printing move needing pressure advance if there is forward extrusion and XYU.. movement.
		// The movement code will only apply pressure advance if there is forward extrusion, so we only need to check for XYU.. movement here.
		{
			AxesBitmap axesMentionedExceptZ = axesMentioned;
			axesMentionedExceptZ.ClearBit(Z_AXIS);
			moveBuffer.usePressureAdvance = moveBuffer.hasExtrusion && axesMentionedExceptZ.IsNonEmpty();
		}

		// Apply segmentation if necessary. To speed up simulation on SCARA printers, we don't apply kinematics segmentation when simulating.
		// Note for when we use RTOS: as soon as we set segmentsLeft nonzero, the Move process will assume that the move is ready to take, so this must be the last thing we do.
		const Kinematics& kin = reprap.GetMove().GetKinematics();
		if (kin.UseSegmentation() && simulationMode != 1 && (moveBuffer.hasExtrusion || moveBuffer.isCoordinated || !kin.UseRawG0()))
		{
			// This kinematics approximates linear motion by means of segmentation.
			// We assume that the segments will be smaller than the mesh spacing.
			const float xyLength = sqrtf(fsquare(currentUserPosition[X_AXIS] - initialXY[0]) + fsquare(currentUserPosition[Y_AXIS] - initialXY[1]));
			const float moveTime = xyLength/moveBuffer.feedRate;			// this is a best-case time, often the move will take longer
			totalSegments = (unsigned int)max<int>(1, min<int>(rintf(xyLength/kin.GetMinSegmentLength()), rintf(moveTime * kin.GetSegmentsPerSecond())));
		}
		else if (reprap.GetMove().IsUsingMesh() && (moveBuffer.isCoordinated || machineType == MachineType::fff))
		{
			const HeightMap& heightMap = reprap.GetMove().AccessHeightMap();
			totalSegments = max<unsigned int>(1, heightMap.GetMinimumSegments(currentUserPosition[X_AXIS] - initialXY[0], currentUserPosition[Y_AXIS] - initialXY[1]));
		}
		else
		{
			totalSegments = 1;
		}
	}

	doingArcMove = false;
	FinaliseMove(gb);
	UnlockAll(gb);			// allow pause
	err = nullptr;
	return true;
}

// Execute an arc move
// We already have the movement lock and the last move has gone
// Currently, we do not process new babystepping when executing an arc move
// Return true if finished, false if needs to be called again
// If an error occurs, return true with 'err' assigned
bool GCodes::DoArcMove(GCodeBuffer& gb, bool clockwise, const char *& err)
{
	if (moveFractionToSkip > 0.0)
	{
		moveBuffer.initialUserX = restartInitialUserX;
		moveBuffer.initialUserY = restartInitialUserY;
	}
	else
	{
		moveBuffer.initialUserX = currentUserPosition[X_AXIS];
		moveBuffer.initialUserY = currentUserPosition[Y_AXIS];
	}

	// Get the axis parameters
	float newX, newY;
	if (gb.Seen('X'))
	{
		newX = gb.GetDistance();
		if (gb.MachineState().axesRelative)
		{
			newX += moveBuffer.initialUserX;
		}
		else if (gb.MachineState().g53Active)
		{
			newX += GetCurrentToolOffset(X_AXIS);
		}
		else if (!gb.MachineState().runningSystemMacro)
		{
			newX += GetWorkplaceOffset(X_AXIS);
		}
	}
	else
	{
		newX = moveBuffer.initialUserX;
	}

	if (gb.Seen('Y'))
	{
		newY = gb.GetDistance();
		if (gb.MachineState().axesRelative)
		{
			newY += moveBuffer.initialUserY;
		}
		else if (gb.MachineState().g53Active)
		{
			newY += GetCurrentToolOffset(Y_AXIS);
		}
		else if (!gb.MachineState().runningSystemMacro)
		{
			newY += GetWorkplaceOffset(Y_AXIS);
		}
	}
	else
	{
		newY = moveBuffer.initialUserY;
	}

	float iParam, jParam;
	if (gb.Seen('R'))
	{
		// We've been given a radius, which takes precedence over I and J parameters
		const float rParam = gb.GetDistance();

		// Get the XY coordinates of the midpoints between the start and end points X and Y distances between start and end points
		const float deltaX = newX - moveBuffer.initialUserX;
		const float deltaY = newY - moveBuffer.initialUserY;

		const float dSquared = fsquare(deltaX) + fsquare(deltaY);	// square of the distance between start and end points
		const float hSquared = fsquare(rParam) - dSquared/4;		// square of the length of the perpendicular from the mid point to the arc centre

		// The distance between start and end points must not be zero, and the perpendicular must have a real length (possibly zero)
		if (dSquared == 0.0 || hSquared < 0.0)
		{
			err = "G2/G3: bad combination of parameter values";
			return true;
		}

		float hDivD = sqrtf(hSquared/dSquared);
		if (clockwise)
		{
			hDivD = -hDivD;
		}
		iParam = deltaX/2 + deltaY * hDivD;
		jParam = deltaY/2 - deltaX * hDivD;
	}
	else
	{
		if (gb.Seen('I'))
		{
			iParam = gb.GetDistance();
		}
		else
		{
			iParam = 0.0;
		}

		if (gb.Seen('J'))
		{
			jParam = gb.GetDistance();
		}
		else
		{
			jParam = 0.0;
		}

		if (iParam == 0.0 && jParam == 0.0)			// at least one of IJ must be specified and nonzero
		{
			err = "G2/G3: no I or J or R parameter";
			return true;
		}
	}

	memcpy(moveBuffer.initialCoords, moveBuffer.coords, numVisibleAxes * sizeof(moveBuffer.initialCoords[0]));

	// Save the arc centre user coordinates for later
	const float userArcCentreX = moveBuffer.initialUserX + iParam;
	const float userArcCentreY = moveBuffer.initialUserY + jParam;

	// Work out the new user position
	currentUserPosition[X_AXIS] = newX;
	currentUserPosition[Y_AXIS] = newY;

	// CNC machines usually do a full circle if the initial and final XY coordinates are the same.
	// Usually this is because X and Y were not given, but repeating the coordinates is permitted.
	const bool wholeCircle = (moveBuffer.initialUserX == currentUserPosition[X_AXIS] && moveBuffer.initialUserY == currentUserPosition[Y_AXIS]);

	// Get any additional axes
	AxesBitmap axesMentioned = XyAxes;
	for (size_t axis = Z_AXIS; axis < numVisibleAxes; axis++)
	{
		if (gb.Seen(axisLetters[axis]))
		{
			const float moveArg = gb.GetDistance();
			if (gb.MachineState().axesRelative)
			{
				currentUserPosition[axis] += moveArg * (1.0 - moveFractionToSkip);
			}
			else if (gb.MachineState().g53Active)
			{
				currentUserPosition[axis] = moveArg + GetCurrentToolOffset(axis);	// g53 ignores tool offsets as well as workplace coordinates
			}
			else if (gb.MachineState().runningSystemMacro)
			{
				currentUserPosition[axis] = moveArg;								// don't apply workplace offsets to commands in system macros
			}
			else
			{
				currentUserPosition[axis] = moveArg + GetWorkplaceOffset(axis);
			}
			axesMentioned.SetBit(axis);
		}
	}

	// Check enough axes have been homed
	if (CheckEnoughAxesHomed(axesMentioned))
	{
		err = "G2/G3: insufficient axes homed";
		return true;
	}

	// Transform to machine coordinates and check that it is within limits
	ToolOffsetTransform(currentUserPosition, moveBuffer.coords, axesMentioned);			// set the final position
	if (reprap.GetMove().GetKinematics().LimitPosition(moveBuffer.coords, nullptr, numVisibleAxes, axesHomed, true, limitAxes) != LimitPositionResult::ok)
	{
		err = "G2/G3: outside machine limits";				// abandon the move
		return true;
	}

	// Compute the angle at which we stop
	const float finalTheta = atan2(currentUserPosition[Y_AXIS] - userArcCentreY, currentUserPosition[X_AXIS] - userArcCentreX);

	// Set up default move parameters
	moveBuffer.checkEndstops = false;
	moveBuffer.reduceAcceleration = false;
	moveBuffer.moveType = 0;
	moveBuffer.tool = reprap.GetCurrentTool();
	moveBuffer.isCoordinated = true;

	// Set up the arc centre coordinates and record which axes behave like an X axis.
	// The I and J parameters are always relative to present position.
	// For X and Y we need to set up the arc centre for each axis that X or Y is mapped to.
	const AxesBitmap xAxes = reprap.GetCurrentXAxes();
	const AxesBitmap yAxes = reprap.GetCurrentYAxes();
	for (size_t axis = 0; axis < numVisibleAxes; ++axis)
	{
		if (xAxes.IsBitSet(axis))
		{
			arcCentre[axis] = (userArcCentreX * axisScaleFactors[axis]) + currentBabyStepOffsets[axis] - Tool::GetOffset(reprap.GetCurrentTool(), X_AXIS);
		}
		else if (yAxes.IsBitSet(axis))
		{
			arcCentre[axis] = (userArcCentreY * axisScaleFactors[axis]) + currentBabyStepOffsets[axis] - Tool::GetOffset(reprap.GetCurrentTool(), Y_AXIS);
		}
	}

	err = LoadExtrusionAndFeedrateFromGCode(gb, true);
	if (err != nullptr)
	{
		return true;
	}

	if (buildObjects.IsFirstMoveSincePrintingResumed())
	{
		if (moveBuffer.hasExtrusion)							// check whether this is the first move after skipping an object and is extruding
		{
			if (TravelToStartPoint(gb))							// don't start a printing move from the wrong point
			{
				buildObjects.DoneMoveSincePrintingResumed();
			}
			return false;
		}
		else
		{
			buildObjects.DoneMoveSincePrintingResumed();
		}
	}

#if TRACK_OBJECT_NAMES
	if (moveBuffer.hasExtrusion)
	{
		//TODO ideally we should calculate the min and max X and Y coordinates of the entire arc here and call UpdateObjectCoordinates twice.
		// But it is currently very rare to use G2/G3 with extrusion, so for now we don't bother.
		buildObjects.UpdateObjectCoordinates(currentUserPosition, AxesBitmap::MakeLowestNBits(2));
	}
#endif

#if SUPPORT_LASER
	if (machineType == MachineType::laser)
	{
		if (gb.Seen('S'))
		{
			moveBuffer.laserPwmOrIoBits.laserPwm = ConvertLaserPwm(gb.GetFValue());
		}
		else if (laserPowerSticky)
		{
			// leave the laser PWM alone because this is what LaserWeb expects
		}
		else
		{
			moveBuffer.laserPwmOrIoBits.laserPwm = 0;
		}
	}
# if SUPPORT_IOBITS
	else
# endif
#endif
#if SUPPORT_IOBITS
	{
		// Update the iobits parameter
		if (gb.Seen('P'))
		{
			moveBuffer.laserPwmOrIoBits.ioBits = (IoBits_t)gb.GetIValue();
		}
		else
		{
			// Leave moveBuffer.ioBits alone so that we keep the previous value
		}
	}
#endif

	moveBuffer.usePressureAdvance = moveBuffer.hasExtrusion;

	arcRadius = sqrtf(iParam * iParam + jParam * jParam);
	arcCurrentAngle = atan2(-jParam, -iParam);

	// Calculate the total angle moved, which depends on which way round we are going
	float totalArc;
	if (wholeCircle)
	{
		totalArc = TwoPi;
	}
	else
	{
		totalArc = (clockwise) ? arcCurrentAngle - finalTheta : finalTheta - arcCurrentAngle;
		if (totalArc < 0.0)
		{
			totalArc += TwoPi;
		}
	}

	// Compute how many segments to use
	// For the arc to deviate up to MaxArcDeviation from the ideal, the segment length should be sqrt(8 * arcRadius * MaxArcDeviation + fsquare(MaxArcDeviation))
	// We leave out the square term because it is very small
	// In CNC applications even very small deviations can be visible, so we use a smaller segment length at low speeds
	const float arcSegmentLength = constrain<float>
									(	min<float>(sqrt(8 * arcRadius * MaxArcDeviation), moveBuffer.feedRate * (1.0/MinArcSegmentsPerSec)),
										MinArcSegmentLength,
										MaxArcSegmentLength
									);
	totalSegments = max<unsigned int>((unsigned int)((arcRadius * totalArc)/arcSegmentLength + 0.8), 1u);
	arcAngleIncrement = totalArc/totalSegments;
	if (clockwise)
	{
		arcAngleIncrement = -arcAngleIncrement;
	}

	doingArcMove = true;
	FinaliseMove(gb);
	UnlockAll(gb);			// allow pause
//	debugPrintf("Radius %.2f, initial angle %.1f, increment %.1f, segments %u\n",
//				arcRadius, arcCurrentAngle * RadiansToDegrees, arcAngleIncrement * RadiansToDegrees, segmentsLeft);
	return true;
}

// Adjust the move parameters to account for segmentation and/or part of the move having been done already
void GCodes::FinaliseMove(GCodeBuffer& gb) noexcept
{
	moveBuffer.canPauseAfter = !moveBuffer.checkEndstops && !doingArcMove;		// pausing during an arc move isn't safe because the arc centre get recomputed incorrectly when we resume
	moveBuffer.filePos = (&gb == fileGCode) ? gb.GetFilePosition() : noFilePosition;
	gb.MotionCommanded();

	if (buildObjects.IsCurrentObjectCancelled())
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
		if (totalSegments > 1)
		{
			segMoveState = SegmentedMoveState::active;
			gb.SetState(GCodeState::waitingForSegmentedMoveToGo);

			for (size_t extruder = 0; extruder < numExtruders; ++extruder)
			{
				moveBuffer.coords[ExtruderToLogicalDrive(extruder)] /= totalSegments;	// change the extrusion to extrusion per segment
			}

			if (moveFractionToSkip != 0.0)
			{
				const float fseg = floor(totalSegments * moveFractionToSkip);		// round down to the start of a move
				segmentsLeftToStartAt = totalSegments - (unsigned int)fseg;
				firstSegmentFractionToSkip = (moveFractionToSkip * totalSegments) - fseg;
				NewMoveAvailable();
				return;
			}
		}
		else
		{
			segMoveState = SegmentedMoveState::inactive;
		}

		segmentsLeftToStartAt = totalSegments;
		firstSegmentFractionToSkip = moveFractionToSkip;

		NewMoveAvailable();
	}
}

// Set up a move to travel to the resume point. Return true if successful, false if needs to be called again.
// By the time this is called, the user position has been overwritten with the final position of the pending move, so we can't use it.
// But the expected position was saved by buildObjects when the state changed from printing a cancelled object to printing a live object.
bool GCodes::TravelToStartPoint(GCodeBuffer& gb) noexcept
{
	if (!LockMovementAndWaitForStandstill(gb))				// update the user position from the machine position
	{
		return false;
	}

	SetMoveBufferDefaults();
	ToolOffsetTransform(currentUserPosition, moveBuffer.initialCoords);
	ToolOffsetTransform(buildObjects.GetInitialPosition().moveCoords, moveBuffer.coords);
	moveBuffer.feedRate = buildObjects.GetInitialPosition().feedRate;
	moveBuffer.tool = reprap.GetCurrentTool();
	NewMoveAvailable(1);
	return true;
}

// The Move class calls this function to find what to do next.
bool GCodes::ReadMove(RawMove& m) noexcept
{
	if (segmentsLeft == 0)
	{
		return false;
	}

	m = moveBuffer;

	if (segmentsLeft == 1)
	{
		// If there is just 1 segment left, it doesn't matter if it is an arc move or not, just move to the end position
		if (segmentsLeftToStartAt == 1 && firstSegmentFractionToSkip != 0.0)	// if this is the segment we are starting at and we need to skip some of it
		{
			// Reduce the extrusion by the amount to be skipped
			for (size_t extruder = 0; extruder < numExtruders; ++extruder)
			{
				m.coords[ExtruderToLogicalDrive(extruder)] *= (1.0 - firstSegmentFractionToSkip);
			}
		}
		m.proportionDone = 1.0;
		if (doingArcMove)
		{
			m.canPauseAfter = true;			// we can pause after the final segment of an arc move
		}
		ClearMove();
	}
	else
	{
		// This move needs to be divided into 2 or more segments
		// Do the axes
		if (doingArcMove)
		{
			arcCurrentAngle += arcAngleIncrement;
		}

		for (size_t drive = 0; drive < numVisibleAxes; ++drive)
		{
			if (doingArcMove && drive != Z_AXIS && Tool::GetYAxes(moveBuffer.tool).IsBitSet(drive))
			{
				// Y axis or a substitute Y axis
				moveBuffer.initialCoords[drive] = arcCentre[drive] + arcRadius * axisScaleFactors[drive] * sinf(arcCurrentAngle);
			}
			else if (doingArcMove && drive != Z_AXIS && Tool::GetXAxes(moveBuffer.tool).IsBitSet(drive))
			{
				// X axis or a substitute X axis
				moveBuffer.initialCoords[drive] = arcCentre[drive] + arcRadius * axisScaleFactors[drive] * cosf(arcCurrentAngle);
			}
			else
			{
				const float movementToDo = (moveBuffer.coords[drive] - moveBuffer.initialCoords[drive])/segmentsLeft;
				moveBuffer.initialCoords[drive] += movementToDo;
			}
			m.coords[drive] = moveBuffer.initialCoords[drive];
		}

		if (segmentsLeftToStartAt < segmentsLeft)
		{
			// We are resuming a print part way through a move and we printed this segment already
			--segmentsLeft;
			return false;
		}

		// Limit the end position at each segment. This is needed for arc moves on any printer, and for [segmented] straight moves on SCARA printers.
		if (reprap.GetMove().GetKinematics().LimitPosition(m.coords, nullptr, numVisibleAxes, axesHomed, true, limitAxes) != LimitPositionResult::ok)
		{
			segMoveState = SegmentedMoveState::aborted;
			doingArcMove = false;
			segmentsLeft = 0;
			return false;
		}

		if (segmentsLeftToStartAt == segmentsLeft && firstSegmentFractionToSkip != 0.0)	// if this is the segment we are starting at and we need to skip some of it
		{
			// Reduce the extrusion by the amount to be skipped
			for (size_t extruder = 0; extruder < numExtruders; ++extruder)
			{
				m.coords[ExtruderToLogicalDrive(extruder)] *= (1.0 - firstSegmentFractionToSkip);
			}
		}
		--segmentsLeft;

		m.proportionDone = (float)(totalSegments - segmentsLeft)/(float)totalSegments;
	}

	return true;
}

void GCodes::ClearMove() noexcept
{
	TaskCriticalSectionLocker lock;				// make sure that other tasks sees a consistent memory state

	segmentsLeft = 0;
	segMoveState = SegmentedMoveState::inactive;
	doingArcMove = false;
	moveBuffer.checkEndstops = false;
	moveBuffer.reduceAcceleration = false;
	moveBuffer.moveType = 0;
	moveBuffer.applyM220M221 = false;
	moveFractionToSkip = 0.0;
}

// Cancel any macro or print in progress
void GCodes::AbortPrint(GCodeBuffer& gb) noexcept
{
	(void)gb.AbortFile(true);					// stop executing any files or macros that this GCodeBuffer is running
	if (&gb == fileGCode)						// if the current command came from a file being printed
	{
		StopPrint(StopPrintReason::abort);
	}
}

// Cancel everything
void GCodes::EmergencyStop() noexcept
{
	for (GCodeBuffer *gbp : gcodeSources)
	{
		if (gbp != nullptr)
		{
			AbortPrint(*gbp);
		}
	}
#if SUPPORT_LASER
	moveBuffer.laserPwmOrIoBits.laserPwm = 0;
#endif
}

// Run a file macro. Prior to calling this, 'state' must be set to the state we want to enter when the macro has been completed.
// Return true if the file was found or it wasn't and we were asked to report that fact.
// 'codeRunning' is the G or M command we are running, or 0 for a tool change file. In particular:
// 501 = running M501
// 502 = running M502
// 98 = running a macro explicitly via M98
// otherwise it is either the G- or M-code being executed, or 0 for a tool change file, or -1 for another system file
// FIXME: sort this out, in particular when the call to RequestMacroFile needs the fromCode parameter to be true
bool GCodes::DoFileMacro(GCodeBuffer& gb, const char* fileName, bool reportMissing, int codeRunning) noexcept
{
#if HAS_LINUX_INTERFACE
	if (reprap.UsingLinuxInterface())
	{
		if (!Push(gb, false))
		{
			return true;
		}

		gb.RequestMacroFile(fileName, reportMissing, gb.IsBinary() && codeRunning >= 0);
	}
	else
#endif
	{
#if HAS_MASS_STORAGE
		FileStore * const f = platform.OpenSysFile(fileName, OpenMode::read);
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
		gb.MachineState().fileState.Set(f);
		gb.StartNewFile();
		gb.GetFileInput()->Reset(gb.MachineState().fileState);
#else
		if (reportMissing)
		{
			platform.MessageF(WarningMessage, "Macro file %s not found\n", fileName);
		}
		return reportMissing;
#endif
	}

#if HAS_LINUX_INTERFACE || HAS_MASS_STORAGE
	gb.MachineState().doingFileMacro = true;
	gb.MachineState().runningM501 = (codeRunning == 501);
	gb.MachineState().runningM502 = (codeRunning == 502);
	if (codeRunning != 98)
	{
		gb.MachineState().runningSystemMacro = true; 	// running a system macro e.g. homing or tool change, so don't use workplace coordinates
	}
	gb.SetState(GCodeState::normal);
	gb.Init();
	return true;
#endif
}

void GCodes::FileMacroCyclesReturn(GCodeBuffer& gb) noexcept
{
	if (gb.IsDoingFileMacro())
	{
#if HAS_LINUX_INTERFACE
		if (reprap.UsingLinuxInterface())
		{
			gb.AbortFile(false);
		}
		else
#endif
		{
#if HAS_MASS_STORAGE
			FileData &file = gb.MachineState().fileState;
			gb.GetFileInput()->Reset(file);
			file.Close();

			gb.PopState(true);
#endif
		}

		gb.Init();
	}
}

// Home one or more of the axes
// 'reply' is only written if there is an error.
GCodeResult GCodes::DoHome(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	if (!LockMovementAndWaitForStandstill(gb))
	{
		return GCodeResult::notFinished;
	}

#if SUPPORT_ROLAND
	// Deal with a Roland configuration
	if (reprap.GetRoland()->Active())
	{
		bool rolHome = reprap.GetRoland()->ProcessHome();
		if (rolHome)
		{
			for(size_t axis = 0; axis < AXES; axis++)
			{
				axisIsHomed[axis] = true;
			}
		}
		return rolHome;
	}
#endif

	// Find out which axes we have been asked to home
	toBeHomed.Clear();
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

// This is called to execute a G30.
// It sets wherever we are as the probe point P (probePointIndex) then probes the bed, or gets all its parameters from the arguments.
// If X or Y are specified, use those; otherwise use the machine's coordinates.  If no Z is specified use the machine's coordinates.
// If it is specified and is greater than SILLY_Z_VALUE (i.e. greater than -9999.0) then that value is used.
// If it's less than SILLY_Z_VALUE the bed is probed and that value is used.
// We already own the movement lock before this is called.
GCodeResult GCodes::ExecuteG30(GCodeBuffer& gb, const StringRef& reply)
{
	g30SValue = (gb.Seen('S')) ? gb.GetIValue() : -4;		// S-4 or lower is equivalent to having no S parameter
	if (g30SValue == -2 && reprap.GetCurrentTool() == nullptr)
	{
		reply.copy("G30 S-2 commanded with no tool selected");
		return GCodeResult::error;
	}

	g30HValue = (gb.Seen('H')) ? gb.GetFValue() : 0.0;
	g30ProbePointIndex = -1;
	bool seenP = false;
	gb.TryGetIValue('P', g30ProbePointIndex, seenP);
	if (seenP)
	{
		if (g30ProbePointIndex < 0 || g30ProbePointIndex >= (int)MaxProbePoints)
		{
			reply.copy("Z probe point index out of range");
			return GCodeResult::error;
		}
		else
		{
			// Set the specified probe point index to the specified coordinates
			const float x = (gb.Seen(axisLetters[X_AXIS])) ? gb.GetFValue() : currentUserPosition[X_AXIS];
			const float y = (gb.Seen(axisLetters[Y_AXIS])) ? gb.GetFValue() : currentUserPosition[Y_AXIS];
			const float z = (gb.Seen(axisLetters[Z_AXIS])) ? gb.GetFValue() : currentUserPosition[Z_AXIS];
			reprap.GetMove().SetXYBedProbePoint((size_t)g30ProbePointIndex, x, y);

			if (z > SILLY_Z_VALUE)
			{
				// Just set the height error to the specified Z coordinate
				reprap.GetMove().SetZBedProbePoint((size_t)g30ProbePointIndex, z, false, false);
				if (g30SValue >= -1)
				{
					return GetGCodeResultFromError(reprap.GetMove().FinishedBedProbing(g30SValue, reply));
				}
			}
			else
			{
				// Do a Z probe at the specified point.
				const auto zp = SetZProbeNumber(gb);			// may throw, so do this before changing the state
				gb.SetState(GCodeState::probingAtPoint0);
				if (zp->GetProbeType() != ZProbeType::blTouch)
				{
					DeployZProbe(gb, 30);
				}
			}
		}
	}
	else
	{
		// G30 without P parameter. This probes the current location starting from the current position.
		// If S=-1 it just reports the stopped height, else it resets the Z origin.
		(void)SetZProbeNumber(gb);								// may throw, so do this before changing the state

		InitialiseTaps();
		gb.SetState(GCodeState::probingAtPoint2a);
		DeployZProbe(gb, 30);
	}
	return GCodeResult::ok;
}

// Set up currentZProbeNumber and return the probe
ReadLockedPointer<ZProbe> GCodes::SetZProbeNumber(GCodeBuffer& gb) THROWS(GCodeException)
{
	const uint32_t probeNumber = (gb.Seen('K')) ? gb.GetLimitedUIValue('K', MaxZProbes) : 0;
	auto zp = reprap.GetPlatform().GetEndstops().GetZProbe(probeNumber);
	if (zp.IsNull())
	{
		throw GCodeException(gb.GetLineNumber(), -1, "Z probe %" PRIu32 " not found", probeNumber);
	}
	currentZProbeNumber = (uint8_t)probeNumber;
	return zp;
}

// Decide which device to display a message box on
MessageType GCodes::GetMessageBoxDevice(GCodeBuffer& gb) const
{
	MessageType mt = gb.GetResponseMessageType();
	if (mt == GenericMessage)
	{
		// Command source was the file being printed, or a trigger. Send the message to PanelDue if there is one, else to the web server.
		mt = (lastAuxStatusReportType >= 0) ? AuxMessage : HttpMessage;
	}
	return mt;
}

void GCodes::DoManualProbe(GCodeBuffer& gb, const char *message, const char *title, const AxesBitmap axes)
{
	if (Push(gb, true))													// stack the machine state including the file position and set the state to GCodeState::normal
	{
		gb.MachineState().WaitForAcknowledgement();						// flag that we are waiting for acknowledgement
		const MessageType mt = GetMessageBoxDevice(gb);
		platform.SendAlert(mt, message, title, 2, 0.0, axes);
	}
}

// Do a manual bed probe. On entry the state variable is the state we want to return to when the user has finished adjusting the height.
void GCodes::DoManualBedProbe(GCodeBuffer& gb)
{
	DoManualProbe(gb, "Adjust height until the nozzle just touches the bed, then press OK", "Manual bed probing", AxesBitmap::MakeFromBits(Z_AXIS));
}

// Start probing the grid, returning true if we didn't because of an error.
// Prior to calling this the movement system must be locked.
GCodeResult GCodes::ProbeGrid(GCodeBuffer& gb, const StringRef& reply)
{
	if (!defaultGrid.IsValid())
	{
		reply.copy("No valid grid defined for bed probing");
		return GCodeResult::error;
	}

	if (!AllAxesAreHomed())
	{
		reply.copy("Must home printer before bed probing");
		return GCodeResult::error;
	}

	const auto zp = SetZProbeNumber(gb);			// may throw, so do this before changing the state

	reprap.GetMove().AccessHeightMap().SetGrid(defaultGrid);
	ClearBedMapping();
	gridXindex = gridYindex = 0;

	gb.SetState(GCodeState::gridProbing1);
	if (zp->GetProbeType() != ZProbeType::blTouch)
	{
		DeployZProbe(gb, 29);
	}
	return GCodeResult::ok;
}

#if HAS_MASS_STORAGE

GCodeResult GCodes::LoadHeightMap(GCodeBuffer& gb, const StringRef& reply)
{
#if HAS_LINUX_INTERFACE
	// If we have a Linux interface and we're using it, the Linux components will take care of file I/O and this should not be called.
	if (reprap.UsingLinuxInterface())
	{
		reply.copy("Cannot use height map on local SD card when SBC interface is used");
		return GCodeResult::error;
	}
#endif

	ClearBedMapping();

	String<MaxFilenameLength> heightMapFileName;
	bool seen = false;
	gb.TryGetQuotedString('P', heightMapFileName.GetRef(), seen);
	if (!seen)
	{
		heightMapFileName.copy(DefaultHeightMapFile);
	}

	String<MaxFilenameLength> fullName;
	platform.MakeSysFileName(fullName.GetRef(), heightMapFileName.c_str());
	FileStore * const f = MassStorage::OpenFile(fullName.c_str(), OpenMode::read, 0);
	if (f == nullptr)
	{
		reply.printf("Height map file %s not found", fullName.c_str());
		return GCodeResult::error;
	}

	reply.printf("Failed to load height map from file %s: ", fullName.c_str());	// set up error message to append to
	const bool err = reprap.GetMove().LoadHeightMapFromFile(f, fullName.c_str(), reply);
	f->Close();

	ActivateHeightmap(!err);
	if (err)
	{
		return GCodeResult::error;
	}

	reply.Clear();						// get rid of the error message
	if (!zDatumSetByProbing && platform.GetZProbeOrDefault(0)->GetProbeType() != ZProbeType::none)	//TODO store Z probe number in height map
	{
		reply.copy("the height map was loaded when the current Z=0 datum was not determined probing. This may result in a height offset.");
		return GCodeResult::warning;
	}

	return GCodeResult::ok;
}

// Save the height map and append the success or error message to 'reply', returning true if an error occurred
bool GCodes::TrySaveHeightMap(const char *filename, const StringRef& reply) const noexcept
{
#if HAS_LINUX_INTERFACE
	// If we have a Linux interface and we're using it, the Linux components will take care of file I/O.
	if (reprap.UsingLinuxInterface())
	{
		reply.copy("Cannot use height map on local SD card when SBC interface is used");
		return true;
	}
#endif

	String<MaxFilenameLength> fullName;
	platform.MakeSysFileName(fullName.GetRef(), filename);
	FileStore * const f = MassStorage::OpenFile(fullName.c_str(), OpenMode::write, 0);
	bool err;
	if (f == nullptr)
	{
		reply.catf("Failed to create height map file %s", fullName.c_str());
		err = true;
	}
	else
	{
		err = reprap.GetMove().SaveHeightMapToFile(f, fullName.c_str());
		f->Close();
		if (err)
		{
			MassStorage::Delete(fullName.c_str(), false);
			reply.catf("Failed to save height map to file %s", fullName.c_str());
		}
		else
		{
			reply.catf("Height map saved to file %s", fullName.c_str());
		}
	}
	return err;
}

// Save the height map to the file specified by P parameter
GCodeResult GCodes::SaveHeightMap(GCodeBuffer& gb, const StringRef& reply) const
{
	// No need to check if we're using the Linux interface here, because TrySaveHeightMap does that
	if (gb.Seen('P'))
	{
		String<MaxFilenameLength> heightMapFileName;
		gb.GetQuotedString(heightMapFileName.GetRef());
		return GetGCodeResultFromError(TrySaveHeightMap(heightMapFileName.c_str(), reply));
	}
	return GetGCodeResultFromError(TrySaveHeightMap(DefaultHeightMapFile, reply));
}

#endif

// Stop using bed compensation
void GCodes::ClearBedMapping()
{
	reprap.GetMove().SetIdentityTransform();
	reprap.GetMove().GetCurrentUserPosition(moveBuffer.coords, 0, reprap.GetCurrentTool());
	ToolOffsetInverseTransform(moveBuffer.coords, currentUserPosition);		// update user coordinates to remove any height map offset there was at the current position
}

// Return the current coordinates as a printable string.
// Coordinates are updated at the end of each movement, so this won't tell you where you are mid-movement.
void GCodes::GetCurrentCoordinates(const StringRef& s) const noexcept
{
	// Start with the axis coordinates
	s.Clear();
	for (size_t axis = 0; axis < numVisibleAxes; ++axis)
	{
		// Don't put a space after the colon in the response, it confuses Pronterface
		s.catf("%c:%.3f ", axisLetters[axis], HideNan(GetUserCoordinate(axis)));
	}

	// Now the virtual extruder position, for Octoprint
	s.catf("E:%.3f ", (double)virtualExtruderPosition);

	// Now the extruder coordinates
	for (size_t i = 0; i < numExtruders; i++)
	{
		s.catf("E%u:%.1f ", i, (double)reprap.GetMove().LiveCoordinate(ExtruderToLogicalDrive(i), reprap.GetCurrentTool()));
	}

	// Print the axis stepper motor positions as Marlin does, as an aid to debugging.
	// Don't bother with the extruder endpoints, they are zero after any non-extruding move.
	s.cat("Count");
	for (size_t i = 0; i < numVisibleAxes; ++i)
	{
		s.catf(" %" PRIi32, reprap.GetMove().GetEndPoint(i));
	}

	// Add the machine coordinates because they may be different from the user coordinates under some conditions
	s.cat(" Machine");
	float machineCoordinates[MaxAxes];
	ToolOffsetTransform(currentUserPosition, machineCoordinates);
	for (size_t axis = 0; axis < numVisibleAxes; ++axis)
	{
		s.catf(" %.3f", HideNan(machineCoordinates[axis]));
	}

	// Add the bed compensation
	const float machineZ = machineCoordinates[Z_AXIS];
	reprap.GetMove().AxisAndBedTransform(machineCoordinates, reprap.GetCurrentTool(), true);
	s.catf(" Bed comp %.3f", (double)(machineCoordinates[Z_AXIS] - machineZ));
}

#if HAS_MASS_STORAGE
// Set up a file to print, but don't print it yet.
// If successful return true, else write an error message to reply and return false
bool GCodes::QueueFileToPrint(const char* fileName, const StringRef& reply) noexcept
{
	FileStore * const f = platform.OpenFile(platform.GetGCodeDir(), fileName, OpenMode::read);
	if (f != nullptr)
	{
		fileToPrint.Set(f);
		fileOffsetToPrint = 0;
		restartMoveFractionDone = 0.0;
		return true;
	}

	reply.printf("GCode file \"%s\" not found\n", fileName);
	return false;
}
#endif

// Start printing the file already selected. We must hold the movement lock and wait for all moves to finish before calling this, because of the call to ResetMoveCounters.
void GCodes::StartPrinting(bool fromStart) noexcept
{
	buildObjects.Init();
	reprap.GetMove().ResetMoveCounters();

	if (fromStart)													// if not resurrecting a print
	{
		fileGCode->MachineState().volumetricExtrusion = false;		// default to non-volumetric extrusion
		virtualExtruderPosition = 0.0;
	}

	for (size_t extruder = 0; extruder < MaxExtruders; extruder++)
	{
		rawExtruderTotalByDrive[extruder] = 0.0;
	}
	rawExtruderTotal = 0.0;
	reprap.GetMove().ResetExtruderPositions();

#if HAS_LINUX_INTERFACE
	if (reprap.UsingLinuxInterface())
	{
		lastFilePosition = noFilePosition;
		fileGCode->OriginalMachineState().SetFileExecuting();
	}
	else
#endif
	{
#if HAS_MASS_STORAGE
		fileGCode->OriginalMachineState().fileState.MoveFrom(fileToPrint);
		fileGCode->StartNewFile();
		fileGCode->GetFileInput()->Reset(fileGCode->OriginalMachineState().fileState);
#endif
	}

	lastFilamentError = FilamentSensorStatus::ok;
	lastPrintingMoveHeight = -1.0;
	reprap.GetPrintMonitor().StartedPrint();
	platform.MessageF(LogMessage,
						(simulationMode == 0) ? "Started printing file %s\n" : "Started simulating printing file %s\n",
							reprap.GetPrintMonitor().GetPrintingFilename());
	if (fromStart)
	{
		// Get the fileGCode to execute the start macro so that any M82/M83 codes will be executed in the correct context
		DoFileMacro(*fileGCode, START_G, false);
	}
}

// Function to handle dwell delays. Returns true for dwell finished, false otherwise.
GCodeResult GCodes::DoDwell(GCodeBuffer& gb) THROWS(GCodeException)
{
	int32_t dwell;
	if (gb.Seen('S'))
	{
		dwell = (int32_t)(gb.GetFValue() * 1000.0);		// S values are in seconds
	}
	else if (gb.Seen('P'))
	{
		dwell = gb.GetIValue();							// P value are in milliseconds
	}
	else
	{
		return GCodeResult::ok;  // No time given - throw it away
	}

	if (dwell <= 0)
	{
		return GCodeResult::ok;
	}

#if SUPPORT_ROLAND
	// Deal with a Roland configuration
	if (reprap.GetRoland()->Active())
	{
		return reprap.GetRoland()->ProcessDwell(dwell);
	}
#endif

	// Wait for all the queued moves to stop. Only do this if motion has been commanded from this GCode stream since we last waited for motion to stop.
	// This is so that G4 can be used in a trigger or daemon macro file without pausing motion, when the macro doesn't itself command any motion.
	if (gb.WasMotionCommanded())
	{
		if (!LockMovementAndWaitForStandstill(gb))
		{
			return GCodeResult::notFinished;
		}
	}

	if (simulationMode != 0)
	{
		simulationTime += (float)dwell * 0.001;
		return GCodeResult::ok;
	}

	return (gb.DoDwellTime((uint32_t)dwell)) ? GCodeResult::ok : GCodeResult::notFinished;
}

// Set offset, working and standby temperatures for a tool. I.e. handle a G10.
GCodeResult GCodes::SetOrReportOffsets(GCodeBuffer &gb, const StringRef& reply) THROWS(GCodeException)
{
	int32_t toolNumber = 0;
	bool seenP = false;
	gb.TryGetIValue('P', toolNumber, seenP);

	ReadLockedPointer<Tool> const tool = (seenP) ? reprap.GetTool(toolNumber) : reprap.GetLockedCurrentTool();
	if (tool.IsNull())
	{
		if (seenP)
		{
			reply.printf("Attempt to set/report offsets and temperatures for non-existent tool: %" PRIi32, toolNumber);
		}
		else
		{
			reply.printf("Attempt to set/report offsets and temperatures for no selected tool");
		}
		return GCodeResult::error;
	}

	bool settingOffset = false;
	for (size_t axis = 0; axis < numVisibleAxes; ++axis)
	{
		if (gb.Seen(axisLetters[axis]))
		{
			if (!LockMovement(gb))
			{
				return GCodeResult::notFinished;
			}
			settingOffset = true;
			tool->SetOffset(axis, gb.GetFValue(), gb.MachineState().runningM501);
		}
	}

	if (settingOffset)
	{
		ToolOffsetInverseTransform(moveBuffer.coords, currentUserPosition);		// update user coordinates to reflect the new tool offset, in case we have this tool selected
	}

	// Deal with setting temperatures
	bool settingTemps = false;
	size_t hCount = tool->HeaterCount();
	if (hCount > 0)
	{
		if (gb.Seen('R'))
		{
			settingTemps = true;
			if (simulationMode == 0)
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
			if (simulationMode == 0)
			{
				float active[MaxHeaters];
				gb.GetFloatArray(active, hCount, true);
				for (size_t h = 0; h < hCount; ++h)
				{
					tool->SetToolHeaterActiveTemperature(h, active[h]);		// may throw
				}
			}
		}
	}

	if (!settingOffset && !settingTemps)
	{
		// Print offsets and temperatures
		reply.printf("Tool %d offsets:", tool->Number());
		for (size_t axis = 0; axis < numVisibleAxes; ++axis)
		{
			reply.catf(" %c%.2f", axisLetters[axis], (double)tool->GetOffset(axis));
		}
		if (hCount != 0)
		{
			reply.cat(", active/standby temperature(s):");
			for (size_t heater = 0; heater < hCount; heater++)
			{
				reply.catf(" %.1f/%.1f", (double)tool->GetToolHeaterActiveTemperature(heater), (double)tool->GetToolHeaterStandbyTemperature(heater));
			}
		}
	}
	return GCodeResult::ok;
}

// Create a new tool definition
GCodeResult GCodes::ManageTool(GCodeBuffer& gb, const StringRef& reply)
{
	// Check tool number
	const unsigned int toolNumber = gb.GetLimitedUIValue('P', MaxTools);

	bool seen = false;

	// Check tool name
	String<ToolNameLength> name;
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

	if (xMap.Intersects(yMap))
	{
		reply.copy("Cannot map both X and Y to the same axis");
		return GCodeResult::error;
	}

	// Check for fan mapping
	FansBitmap fanMap;
	if (gb.Seen('F'))
	{
		uint32_t fanMapping[MaxFans];
		size_t fanCount = MaxFans;
		gb.GetUnsignedArray(fanMapping, fanCount, false);
		fanMap = FansBitmap::MakeFromArray(fanMapping, fanCount) & FansBitmap::MakeLowestNBits(MaxFans);
		seen = true;
	}
	else
	{
		fanMap.SetBit(0);							// by default map fan 0 to fan 0
	}

	if (seen)
	{
		if (!LockMovementAndWaitForStandstill(gb))
		{
			return GCodeResult::notFinished;
		}

		// Check if filament support is being enforced
		const int filamentDrive = (gb.Seen('L')) ? gb.GetIValue()
									: ((dCount == 1) ? drives[0]
										: -1);

		// Add or delete tool, so start by deleting the old one with this number, if any
		reprap.DeleteTool(toolNumber);

		// M563 P# D-1 H-1 removes an existing tool
		if (dCount == 1 && hCount == 1 && drives[0] == -1 && heaters[0] == -1)
		{
			// nothing more to do
		}
		else
		{
			Tool* const tool = Tool::Create(toolNumber, name.c_str(), drives, dCount, heaters, hCount, xMap, yMap, fanMap, filamentDrive, reply);
			if (tool == nullptr)
			{
				return GCodeResult::error;
			}
			reprap.AddTool(tool);
		}
	}
	else
	{
		reprap.PrintTool(toolNumber, reply);
	}
	return GCodeResult::ok;
}

// Does what it says.
void GCodes::DisableDrives() noexcept
{
	platform.DisableAllDrivers();
	SetAllAxesNotHomed();
}

bool GCodes::ChangeMicrostepping(size_t drive, unsigned int microsteps, bool interp, const StringRef& reply) const noexcept
{
	bool dummy;
	const unsigned int oldSteps = platform.GetMicrostepping(drive, dummy);
	const bool success = platform.SetMicrostepping(drive, microsteps, interp, reply);
	if (success)
	{
		// We changed the microstepping, so adjust the steps/mm to compensate
		platform.SetDriveStepsPerUnit(drive, platform.DriveStepsPerUnit(drive), oldSteps);
	}
	return success;
}

// Set the speeds of fans mapped for the current tool to lastDefaultFanSpeed
void GCodes::SetMappedFanSpeed(float f) noexcept
{
	lastDefaultFanSpeed = f;
	const Tool * const ct = reprap.GetCurrentTool();
	if (ct == nullptr)
	{
		reprap.GetFansManager().SetFanValue(0, f);
	}
	else
	{
		const FansBitmap fanMap = ct->GetFanMapping();
		fanMap.Iterate([f](unsigned int i, unsigned int) noexcept { reprap.GetFansManager().SetFanValue(i, f); });
	}
}

// Return true if this fan number is currently being used as a print cooling fan
bool GCodes::IsMappedFan(unsigned int fanNumber) noexcept
{
	const Tool * const ct = reprap.GetCurrentTool();
	return (ct == nullptr)
			? fanNumber == 0
				: ct->GetFanMapping().IsBitSet(fanNumber);
}

// Save the speeds of all fans
void GCodes::SaveFanSpeeds() noexcept
{
	for (size_t i = 0; i < MaxFans; ++i)
	{
		pausedFanSpeeds[i] = reprap.GetFansManager().GetFanValue(i);
	}
	pausedDefaultFanSpeed = lastDefaultFanSpeed;
}

// Handle sending a reply back to the appropriate interface(s) and update lastResult
// Note that 'reply' may be empty. If it isn't, then we need to append newline when sending it.
void GCodes::HandleReply(GCodeBuffer& gb, GCodeResult rslt, const char* reply) noexcept
{
	gb.SetLastResult(rslt);
	HandleReplyPreserveResult(gb, rslt, reply);
}

// Handle sending a reply back to the appropriate interface(s) but dpm't update lastResult
// Note that 'reply' may be empty. If it isn't, then we need to append newline when sending it.
void GCodes::HandleReplyPreserveResult(GCodeBuffer& gb, GCodeResult rslt, const char *reply) noexcept
{
#if HAS_LINUX_INTERFACE
	// Deal with replies to the Linux interface
	if (gb.IsBinary())
	{
		MessageType type = gb.GetResponseMessageType();
		if (rslt == GCodeResult::notFinished || gb.IsMacroRequested() ||
			(gb.MachineState().waitingForAcknowledgement && !gb.MachineState().waitingForAcknowledgementSent))
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
			type = (MessageType)(type | WarningMessageFlag | LogMessage);
		}
		else if (rslt == GCodeResult::error)
		{
			type = (MessageType)(type | ErrorMessageFlag | LogMessage);
		}

		platform.Message(type, reply);
		return;
	}
#endif

	// Don't report empty responses if a file or macro is being processed, or if the GCode was queued
	// Also check that this response was triggered by a gcode
	if (   reply[0] == 0
		&& (   (gb.MachineState().doingFileMacro && !gb.MachineState().waitingForAcknowledgement)			// we must acknowledge M292
			|| &gb == fileGCode || &gb == queuedGCode || &gb == triggerGCode || &gb == autoPauseGCode || (&gb == auxGCode && !platform.IsAuxRaw())
		   )
	   )
	{
		return;
	}

	const MessageType initialMt = gb.GetResponseMessageType();
	const MessageType mt = (rslt == GCodeResult::error) ? (MessageType)(initialMt | ErrorMessageFlag | LogMessage)
							: (rslt == GCodeResult::warning) ? (MessageType)(initialMt | WarningMessageFlag | LogMessage)
								: initialMt;

	switch (gb.MachineState().compatibility.RawValue())
	{
	case Compatibility::Default:
	case Compatibility::RepRapFirmware:
		// DWC 2 expects a reply from every command even if it is just a newline, so don't suppress empty responses
		platform.MessageF(mt, "%s\n", reply);
		break;

	case Compatibility::NanoDLP:		// nanoDLP is like Marlin except that G0 and G1 commands return "Z_move_comp<LF>" before "ok<LF>"
	case Compatibility::Marlin:
		{
			const char* const response = (gb.GetCommandLetter() == 'M' && gb.GetCommandNumber() == 998) ? "rs " : "ok";
			// We don't need to handle M20 here because we always allocate an output buffer for that one
			if (gb.GetCommandLetter() == 'M' && gb.GetCommandNumber() == 28)
			{
				platform.MessageF(mt, "%s\n%s\n", response, reply);
			}
			else if (gb.GetCommandLetter() == 'M' && (gb.GetCommandNumber() == 105 || gb.GetCommandNumber() == 998))
			{
				platform.MessageF(mt, "%s %s\n", response, reply);
			}
			else if (reply[0] != 0 && !gb.IsDoingFileMacro())
			{
				platform.MessageF(mt, "%s\n%s\n", reply, response);
			}
			else if (reply[0] != 0)
			{
				platform.MessageF(mt, "%s\n", reply);
			}
			else
			{
				platform.MessageF(mt, "%s\n", response);
			}
		}
		break;

	case Compatibility::Teacup:
	case Compatibility::Sprinter:
	case Compatibility::Repetier:
	default:
		platform.MessageF(mt, "Emulation of %s is not yet supported.\n", gb.MachineState().compatibility.ToString());
		break;
	}
}

// Handle a successful response when the response is in an OutputBuffer
void GCodes::HandleReply(GCodeBuffer& gb, OutputBuffer *reply) noexcept
{
	gb.SetLastResult(GCodeResult::ok);

	// Although unlikely, it's possible that we get a nullptr reply. Don't proceed if this is the case
	if (reply == nullptr)
	{
		return;
	}

#if HAS_LINUX_INTERFACE
	// Deal with replies to the Linux interface
	if (gb.IsBinary())
	{
		platform.Message(gb.GetResponseMessageType(), reply);
		return;
	}
#endif

	// Second UART device, e.g. dc42's PanelDue. Do NOT use emulation for this one!
	if (&gb == auxGCode && !platform.IsAuxRaw())
	{
		platform.AppendAuxReply(reply, (*reply)[0] == '{');
		return;
	}

	const MessageType type = gb.GetResponseMessageType();
	const char* const response = (gb.GetCommandLetter() == 'M' && gb.GetCommandNumber() == 998) ? "rs " : "ok";

	switch (gb.MachineState().compatibility.RawValue())
	{
	case Compatibility::Default:
	case Compatibility::RepRapFirmware:
		platform.Message(type, reply);
		return;

	case Compatibility::Marlin:
	case Compatibility::NanoDLP:
		if (gb.GetCommandLetter() =='M' && gb.GetCommandNumber() == 20)
		{
			platform.Message(type, "Begin file list\n");
			platform.Message(type, reply);
			platform.Message(type, "End file list\n");
			platform.Message(type, response);
			platform.Message(type, "\n");
			return;
		}

		if (gb.GetCommandLetter() == 'M' && gb.GetCommandNumber() == 28)
		{
			platform.Message(type, response);
			platform.Message(type, "\n");
			platform.Message(type, reply);
			return;
		}

		if (gb.GetCommandLetter() =='M' && (gb.GetCommandNumber() == 105 || gb.GetCommandNumber() == 998))
		{
			platform.Message(type, response);
			platform.Message(type, " ");
			platform.Message(type, reply);
			return;
		}

		if (reply->Length() != 0 && !gb.IsDoingFileMacro())
		{
			platform.Message(type, reply);
			platform.Message(type, "\n");
			platform.Message(type, response);
			platform.Message(type, "\n");
		}
		else if (reply->Length() != 0)
		{
			platform.Message(type, reply);
		}
		else
		{
			OutputBuffer::ReleaseAll(reply);
			platform.Message(type, response);
			platform.Message(type, "\n");
		}
		return;

	case Compatibility::Teacup:
	case Compatibility::Sprinter:
	case Compatibility::Repetier:
	default:
		platform.MessageF(type, "Emulation of %s is not yet supported.\n", gb.MachineState().compatibility.ToString());
		break;
	}

	// If we get here then we didn't handle the message, so release the buffer(s)
	OutputBuffer::ReleaseAll(reply);
}

void GCodes::SetToolHeaters(Tool *tool, float temperature, bool both) THROWS(GCodeException)
{
	if (tool == nullptr)
	{
		throw GCodeException(-1, -1, "setting temperature: no tool selected\n");
	}

	for (size_t h = 0; h < tool->HeaterCount(); h++)
	{
		tool->SetToolHeaterActiveTemperature(h, temperature);
		if (both)
		{
			tool->SetToolHeaterStandbyTemperature(h, temperature);
		}
	}
}

// Retract or un-retract filament, returning true if movement has been queued, false if this needs to be called again
GCodeResult GCodes::RetractFilament(GCodeBuffer& gb, bool retract)
{
	if (!buildObjects.IsCurrentObjectCancelled())
	{
		Tool* const currentTool = reprap.GetCurrentTool();
		if (  currentTool != nullptr
			&& retract != currentTool->IsRetracted()
			&& (currentTool->GetRetractLength() != 0.0 || currentTool->GetRetractHop() != 0.0 || (!retract && currentTool->GetRetractExtra() != 0.0))
		   )
		{
			if (!LockMovement(gb))
			{
				return GCodeResult::notFinished;
			}

			if (segmentsLeft != 0)
			{
				return GCodeResult::notFinished;
			}

			// New code does the retraction and the Z hop as separate moves
			// Get ready to generate a move
			SetMoveBufferDefaults();
			moveBuffer.tool = reprap.GetCurrentTool();
			reprap.GetMove().GetCurrentUserPosition(moveBuffer.coords, 0, moveBuffer.tool);
			moveBuffer.filePos = (&gb == fileGCode) ? gb.GetFilePosition() : noFilePosition;

			if (retract)
			{
				// Set up the retract move
				const Tool * const tool = reprap.GetCurrentTool();
				if (tool != nullptr && tool->DriveCount() != 0)
				{
					for (size_t i = 0; i < tool->DriveCount(); ++i)
					{
						moveBuffer.coords[ExtruderToLogicalDrive(tool->Drive(i))] = -currentTool->GetRetractLength();
					}
					moveBuffer.feedRate = currentTool->GetRetractSpeed() * tool->DriveCount();
					moveBuffer.canPauseAfter = false;			// don't pause after a retraction because that could cause too much retraction
					NewMoveAvailable(1);
				}
				if (currentTool->GetRetractHop() > 0.0)
				{
					gb.SetState(GCodeState::doingFirmwareRetraction);
				}
			}
			else if (currentZHop > 0.0)
			{
				// Set up the reverse Z hop move
				moveBuffer.feedRate = platform.MaxFeedrate(Z_AXIS);
				moveBuffer.coords[Z_AXIS] -= currentZHop;
				currentZHop = 0.0;
				moveBuffer.canPauseAfter = false;			// don't pause in the middle of a command
				NewMoveAvailable(1);
				gb.SetState(GCodeState::doingFirmwareUnRetraction);
			}
			else
			{
				// No retract hop, so just un-retract
				const Tool * const tool = reprap.GetCurrentTool();
				if (tool != nullptr && tool->DriveCount() != 0)
				{
					for (size_t i = 0; i < tool->DriveCount(); ++i)
					{
						moveBuffer.coords[ExtruderToLogicalDrive(tool->Drive(i))] = currentTool->GetRetractLength() + currentTool->GetRetractExtra();
					}
					moveBuffer.feedRate = currentTool->GetUnRetractSpeed() * tool->DriveCount();
					moveBuffer.canPauseAfter = true;
					NewMoveAvailable(1);
				}
			}
			currentTool->SetRetracted(retract);
		}
	}
	return GCodeResult::ok;
}

// Load the specified filament into a tool
GCodeResult GCodes::LoadFilament(GCodeBuffer& gb, const StringRef& reply)
{
	Tool * const tool = reprap.GetCurrentTool();
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

		if (Filament::IsInUse(filamentName.c_str()))
		{
			reply.copy("One filament type can be only assigned to a single tool");
			return GCodeResult::error;
		}

		SafeStrncpy(filamentToLoad, filamentName.c_str(), ARRAY_SIZE(filamentToLoad));
		gb.SetState(GCodeState::loadingFilament);

		String<StringLength256> scratchString;
		scratchString.printf("%s%s/%s", FILAMENTS_DIRECTORY, filamentName.c_str(), LOAD_FILAMENT_G);
		DoFileMacro(gb, scratchString.c_str(), true, 701);
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
GCodeResult GCodes::UnloadFilament(GCodeBuffer& gb, const StringRef& reply)
{
	Tool * const tool = reprap.GetCurrentTool();
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

	if (tool->GetFilament()->IsLoaded())			// if no filament is loaded, nothing to do
	{
		gb.SetState(GCodeState::unloadingFilament);
		String<StringLength256> scratchString;
		scratchString.printf("%s%s/%s", FILAMENTS_DIRECTORY, tool->GetFilament()->GetName(), UNLOAD_FILAMENT_G);
		DoFileMacro(gb, scratchString.c_str(), true, 702);
	}
	return GCodeResult::ok;
}

float GCodes::GetRawExtruderTotalByDrive(size_t extruder) const noexcept
{
	return (extruder < numExtruders) ? rawExtruderTotalByDrive[extruder] : 0.0;
}

// Return true if the code queue is idle
bool GCodes::IsCodeQueueIdle() const noexcept
{
	return queuedGCode->IsIdle() && codeQueue->IsIdle();
}

// Cancel the current SD card print.
// This is called from Pid.cpp when there is a heater fault, and from elsewhere in this module.
void GCodes::StopPrint(StopPrintReason reason) noexcept
{
	segmentsLeft = 0;
	isPaused = pausePending = filamentChangePausePending = false;

#if HAS_LINUX_INTERFACE
	if (reprap.UsingLinuxInterface())
	{
		lastFilePosition = noFilePosition;
		fileGCode->MachineState().CloseFile();
		fileGCode->Init();
	}
	else
#endif
	{
#if HAS_MASS_STORAGE
		FileData& fileBeingPrinted = fileGCode->OriginalMachineState().fileState;

		fileGCode->GetFileInput()->Reset(fileBeingPrinted);
		fileGCode->Init();

		if (fileBeingPrinted.IsLive())
		{
			fileBeingPrinted.Close();
		}
#endif
	}

	// Don't call ReserMoveCounters here because we can't be sure that the movement queue is empty
	codeQueue->Clear();

	UnlockAll(*fileGCode);

	// Deal with the Z hop from a G10 that has not been undone by G11
	Tool* const currentTool = reprap.GetCurrentTool();
	if (currentTool != nullptr && currentTool->IsRetracted())
	{
		currentUserPosition[Z_AXIS] += currentZHop;
		currentZHop = 0.0;
		currentTool->SetRetracted(false);
	}

	const char *printingFilename = reprap.GetPrintMonitor().GetPrintingFilename();
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
		simulationMode = 0;							// do this after we append the simulation info to the file so that DWC doesn't try to reload the file info too soon
		reprap.GetMove().Simulate(simulationMode);
		EndSimulation(nullptr);

		const uint32_t simMinutes = lrintf(simSeconds/60.0);
		if (reason == StopPrintReason::normalCompletion)
		{
			lastDuration = simSeconds;
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
			reprap.GetHeat().SwitchOffAll(true);	// turn all heaters off
			switch (machineType)
			{
			case MachineType::cnc:
				for (size_t i = 0; i < MaxSpindles; i++)
				{
					platform.AccessSpindle(i).TurnOff();
				}
				break;

#if SUPPORT_LASER
			case MachineType::laser:
				platform.SetLaserPwm(0);
				moveBuffer.laserPwmOrIoBits.laserPwm = 0;
				break;
#endif

			default:
				break;
			}
		}

		// Pronterface expects a "Done printing" message
		if (usbGCode->MachineState().compatibility == Compatibility::Marlin)
		{
			platform.Message(UsbMessage, "Done printing file\n");
		}
#if SUPPORT_TELNET
		if (telnetGCode->MachineState().compatibility == Compatibility::Marlin)
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
#if HAS_MASS_STORAGE
		if (   reason == StopPrintReason::normalCompletion
			&& simulationMode == 0
# if HAS_LINUX_INTERFACE
			&& !reprap.UsingLinuxInterface()
# endif
		   )
		{
			platform.DeleteSysFile(RESUME_AFTER_POWER_FAIL_G);
		}
#endif
	}

	updateFileWhenSimulationComplete = false;
	reprap.GetPrintMonitor().StoppedPrint();		// must do this after printing the simulation details not before, because it clears the filename and pause time
	buildObjects.Init();
}

// Return true if all the heaters for the specified tool are at their set temperatures
bool GCodes::ToolHeatersAtSetTemperatures(const Tool *tool, bool waitWhenCooling, float tolerance) const noexcept
{
	if (tool != nullptr)
	{
		for (size_t i = 0; i < tool->HeaterCount(); ++i)
		{
			if (!reprap.GetHeat().HeaterAtSetTemperature(tool->Heater(i), waitWhenCooling, tolerance))
			{
				return false;
			}
		}
	}
	return true;
}

// Get the current position from the Move class
void GCodes::UpdateCurrentUserPosition() noexcept
{
	reprap.GetMove().GetCurrentUserPosition(moveBuffer.coords, 0, reprap.GetCurrentTool());
	ToolOffsetInverseTransform(moveBuffer.coords, currentUserPosition);
}

// Save position etc. to a restore point.
// Note that restore point coordinates are not affected by workplace coordinate offsets. This allows them to be used in resume.g.
void GCodes::SavePosition(RestorePoint& rp, const GCodeBuffer& gb) const noexcept
{
	for (size_t axis = 0; axis < numVisibleAxes; ++axis)
	{
		rp.moveCoords[axis] = currentUserPosition[axis];
	}

	rp.feedRate = gb.MachineState().feedRate;
	rp.virtualExtruderPosition = virtualExtruderPosition;
	rp.filePos = gb.GetFilePosition();

	SaveSpindleSpeeds(rp);

#if SUPPORT_LASER || SUPPORT_IOBITS
	rp.laserPwmOrIoBits = moveBuffer.laserPwmOrIoBits;
#endif
}

// Save spindle speeds to a restore point
void GCodes::SaveSpindleSpeeds(RestorePoint& rp) const noexcept
{
	if (machineType == MachineType::cnc)
	{
		for (unsigned int i = 0; i < MaxSpindles; ++i)
		{
			rp.spindleSpeeds[i] = platform.AccessSpindle(i).GetRpm();
		}
	}
}

// Restore user position from a restore point. Also restore the laser power, but not the spindle speed (the user must do that explicitly).
void GCodes::RestorePosition(const RestorePoint& rp, GCodeBuffer *gb) noexcept
{
	for (size_t axis = 0; axis < numVisibleAxes; ++axis)
	{
		currentUserPosition[axis] = rp.moveCoords[axis];
	}

	if (gb != nullptr)
	{
		gb->MachineState().feedRate = rp.feedRate;
	}

	moveBuffer.initialUserX = rp.initialUserX;
	moveBuffer.initialUserY = rp.initialUserY;

#if SUPPORT_LASER || SUPPORT_IOBITS
	moveBuffer.laserPwmOrIoBits = rp.laserPwmOrIoBits;
#endif
}

// Convert user coordinates to head reference point coordinates, optionally allowing for X axis mapping
// If the X axis is mapped to some other axes not including X, then the X coordinate of coordsOut will be left unchanged.
// So make sure it is suitably initialised before calling this.
void GCodes::ToolOffsetTransform(const float coordsIn[MaxAxes], float coordsOut[MaxAxes], AxesBitmap explicitAxes) const noexcept
{
	const Tool * const currentTool = reprap.GetCurrentTool();
	if (currentTool == nullptr)
	{
		for (size_t axis = 0; axis < numVisibleAxes; ++axis)
		{
			coordsOut[axis] = (coordsIn[axis] * axisScaleFactors[axis]) + currentBabyStepOffsets[axis];
		}
	}
	else
	{
		const AxesBitmap xAxes = currentTool->GetXAxisMap();
		const AxesBitmap yAxes = currentTool->GetYAxisMap();
		for (size_t axis = 0; axis < numVisibleAxes; ++axis)
		{
			if (   (axis != X_AXIS || xAxes.IsBitSet(X_AXIS))
				&& (axis != Y_AXIS || yAxes.IsBitSet(Y_AXIS))
			   )
			{
				const float totalOffset = currentBabyStepOffsets[axis] - currentTool->GetOffset(axis);
				const size_t inputAxis = (explicitAxes.IsBitSet(axis)) ? axis
										: (xAxes.IsBitSet(axis)) ? X_AXIS
											: (yAxes.IsBitSet(axis)) ? Y_AXIS
												: axis;
				coordsOut[axis] = (coordsIn[inputAxis] * axisScaleFactors[axis]) + totalOffset;
			}
		}
	}
	coordsOut[Z_AXIS] += currentZHop;
}

// Convert head reference point coordinates to user coordinates, allowing for XY axis mapping
// Caution: coordsIn and coordsOut may address the same array!
void GCodes::ToolOffsetInverseTransform(const float coordsIn[MaxAxes], float coordsOut[MaxAxes]) const noexcept
{
	const Tool * const currentTool = reprap.GetCurrentTool();
	if (currentTool == nullptr)
	{
		for (size_t axis = 0; axis < numVisibleAxes; ++axis)
		{
			coordsOut[axis] = (coordsIn[axis] - currentBabyStepOffsets[axis])/axisScaleFactors[axis];
		}
	}
	else
	{
		const AxesBitmap xAxes = reprap.GetCurrentXAxes();
		const AxesBitmap yAxes = reprap.GetCurrentYAxes();
		float xCoord = 0.0, yCoord = 0.0;
		size_t numXAxes = 0, numYAxes = 0;
		for (size_t axis = 0; axis < numVisibleAxes; ++axis)
		{
			const float totalOffset = currentBabyStepOffsets[axis] - currentTool->GetOffset(axis);
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
		}
		if (numXAxes != 0)
		{
			coordsOut[X_AXIS] = xCoord/numXAxes;
		}
		if (numYAxes != 0)
		{
			coordsOut[Y_AXIS] = yCoord/numYAxes;
		}
	}
	coordsOut[Z_AXIS] -= currentZHop/axisScaleFactors[Z_AXIS];
}

// Get an axis offset of the current tool
float GCodes::GetCurrentToolOffset(size_t axis) const noexcept
{
	const Tool* const tool = reprap.GetCurrentTool();
	return (tool == nullptr) ? 0.0 : tool->GetOffset(axis);
}

// Get the current user coordinate and remove the workplace offset
float GCodes::GetUserCoordinate(size_t axis) const noexcept
{
	return (axis < numTotalAxes) ? currentUserPosition[axis] - GetWorkplaceOffset(axis) : 0.0;
}

#if HAS_MASS_STORAGE

// M38 (SHA1 hash of a file) implementation:
bool GCodes::StartHash(const char* filename) noexcept
{
	// Get a FileStore object
	fileBeingHashed = platform.OpenFile(FS_PREFIX, filename, OpenMode::read);
	if (fileBeingHashed == nullptr)
	{
		return false;
	}

	// Start hashing
	SHA1Reset(&hash);
	return true;
}

GCodeResult GCodes::AdvanceHash(const StringRef &reply) noexcept
{
	// Read and process some more data from the file
	uint32_t buf32[(FILE_BUFFER_SIZE + 3) / 4];
	char *buffer = reinterpret_cast<char *>(buf32);

	int bytesRead = fileBeingHashed->Read(buffer, FILE_BUFFER_SIZE);
	if (bytesRead != -1)
	{
		SHA1Input(&hash, reinterpret_cast<const uint8_t *>(buffer), bytesRead);

		if (bytesRead != FILE_BUFFER_SIZE)
		{
			// Calculate and report the final result
			SHA1Result(&hash);
			for (size_t i = 0; i < 5; i++)
			{
				reply.catf("%08" PRIx32, hash.Message_Digest[i]);
			}

			// Clean up again
			fileBeingHashed->Close();
			fileBeingHashed = nullptr;
			return GCodeResult::ok;
		}
		return GCodeResult::notFinished;
	}

	// Something went wrong, we cannot read any more from the file
	fileBeingHashed->Close();
	fileBeingHashed = nullptr;
	return GCodeResult::ok;
}

#endif

bool GCodes::AllAxesAreHomed() const noexcept
{
	const AxesBitmap allAxes = AxesBitmap::MakeLowestNBits(numVisibleAxes);
	return (axesHomed & allAxes) == allAxes;
}

// Tell us that the axis is now homed
void GCodes::SetAxisIsHomed(unsigned int axis) noexcept
{
	axesHomed.SetBit(axis);
}

// Tell us that the axis is not homed
void GCodes::SetAxisNotHomed(unsigned int axis) noexcept
{
	axesHomed.ClearBit(axis);
	if (axis == Z_AXIS)
	{
		zDatumSetByProbing = false;
	}
}

// Flag all axes as not homed
void GCodes::SetAllAxesNotHomed() noexcept
{
	axesHomed.Clear();
	zDatumSetByProbing = false;
}

#if HAS_MASS_STORAGE

// Write the config-override file returning true if an error occurred
GCodeResult GCodes::WriteConfigOverrideFile(GCodeBuffer& gb, const StringRef& reply) const noexcept
{
	const char* const fileName = CONFIG_OVERRIDE_G;
	FileStore * const f = platform.OpenSysFile(fileName, OpenMode::write);
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
	if (gb.Seen('P'))
	{
		uint32_t pVals[2];
		size_t pCount = 2;
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
			}
		}
	}
	if (ok)
	{
		ok = platform.WritePlatformParameters(f, p31);
	}

	if (ok)
	{
		ok = reprap.WriteToolParameters(f, p10);
	}

#if SUPPORT_WORKPLACE_COORDINATES
	if (ok)
	{
		ok = WriteWorkplaceCoordinates(f);
	}
#endif

	if (ok)
	{
		ok = buildObjects.WriteObjectDirectory(f);
	}

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
void GCodes::GenerateTemperatureReport(const StringRef& reply) const noexcept
{
	reprap.ReportAllToolTemperatures(reply);

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
void GCodes::CheckReportDue(GCodeBuffer& gb, const StringRef& reply) const
{
	if (gb.DoDwellTime(1000))
	{
		if (gb.MachineState().compatibility == Compatibility::Marlin)
		{
			// In Marlin emulation mode we should return a standard temperature report every second
			GenerateTemperatureReport(reply);
			reply.cat('\n');
			platform.Message(UsbMessage, reply.c_str());
		}
		if (lastAuxStatusReportType >= 0)
		{
			// Send a standard status response for PanelDue
			OutputBuffer * const statusBuf = GenerateJsonStatusResponse(lastAuxStatusReportType, -1, ResponseSource::AUX);
			if (statusBuf != nullptr)
			{
				platform.AppendAuxReply(statusBuf, true);
			}
		}
		gb.StartTimer();
	}
}

// Generate a M408 response
// Return the output buffer containing the response, or nullptr if we failed
OutputBuffer *GCodes::GenerateJsonStatusResponse(int type, int seq, ResponseSource source) const noexcept
{
	OutputBuffer *statusResponse = nullptr;
	switch (type)
	{
		case 0:
		case 1:
			statusResponse = reprap.GetLegacyStatusResponse(type + 2, seq);
			break;

		default:				// need a default clause to prevent the command hanging by always returning a null buffer
			type = 2;
			// no break
		case 2:
		case 3:
		case 4:
			statusResponse = reprap.GetStatusResponse(type - 1, source);
			break;

		case 5:
			statusResponse = reprap.GetConfigResponse();
			break;
	}
	if (statusResponse != nullptr)
	{
		statusResponse->cat('\n');
		if (statusResponse->HadOverflow())
		{
			OutputBuffer::ReleaseAll(statusResponse);
		}
	}
	return statusResponse;
}

// Initiate a tool change. Caller has already checked that the correct tool isn't loaded.
void GCodes::StartToolChange(GCodeBuffer& gb, int toolNum, uint8_t param) noexcept
{
	newToolNumber = toolNum;
	toolChangeParam = (simulationMode != 0) ? 0 : param;
	gb.SetState(GCodeState::toolChange0);
}

// Set up some default values in the move buffer for special moves, e.g. for Z probing and firmware retraction
void GCodes::SetMoveBufferDefaults() noexcept
{
	moveBuffer.SetDefaults(numTotalAxes);
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
		gb.MachineState().lockedResources.SetBit(r);
		return true;
	}
	return false;
}

// Grab the movement lock even if another GCode source has it
// CAUTION: this will be unsafe when we move to RTOS
void GCodes::GrabResource(const GCodeBuffer& gb, Resource r) noexcept
{
	if (resourceOwners[r] != &gb)
	{
		if (resourceOwners[r] != nullptr)
		{
			GCodeMachineState *m = &(resourceOwners[r]->MachineState());
			do
			{
				m->lockedResources.ClearBit(r);
				m = m->GetPrevious();
			}
			while (m != nullptr);
		}
		resourceOwners[r] = &gb;
	}
}

// Lock the unshareable parts of the file system
bool GCodes::LockFileSystem(const GCodeBuffer &gb) noexcept
{
	return LockResource(gb, FileSystemResource);
}

// Lock movement
bool GCodes::LockMovement(const GCodeBuffer& gb) noexcept
{
	return LockResource(gb, MoveResource);
}

void GCodes::GrabMovement(const GCodeBuffer& gb) noexcept
{
	GrabResource(gb, MoveResource);
}

void GCodes::UnlockMovement(const GCodeBuffer& gb) noexcept
{
	UnlockResource(gb, MoveResource);
}

// Unlock the resource if we own it
void GCodes::UnlockResource(const GCodeBuffer& gb, Resource r) noexcept
{
	if (resourceOwners[r] == &gb)
	{
		GCodeMachineState * mc = &gb.MachineState();
		do
		{
			mc->lockedResources.ClearBit(r);
			mc = mc->GetPrevious();
		} while (mc != nullptr);
		resourceOwners[r] = nullptr;
	}
}

// Release all locks, except those that were owned when the current macro was started
void GCodes::UnlockAll(const GCodeBuffer& gb) noexcept
{
	const GCodeMachineState * const mc = gb.MachineState().GetPrevious();
	const GCodeMachineState::ResourceBitmap resourcesToKeep = (mc == nullptr) ? GCodeMachineState::ResourceBitmap() : mc->lockedResources;
	for (size_t i = 0; i < NumResources; ++i)
	{
		if (resourceOwners[i] == &gb && !resourcesToKeep.IsBitSet(i))
		{
			resourceOwners[i] = nullptr;
			gb.MachineState().lockedResources.ClearBit(i);
		}
	}
}

// Append a list of axes to a string
void GCodes::AppendAxes(const StringRef& reply, AxesBitmap axes) const noexcept
{
	axes.Iterate([reply, this](unsigned int axis, unsigned int) noexcept { reply.cat(this->axisLetters[axis]); });
}

// Get the name of the current machine mode
const char* GCodes::GetMachineModeString() const noexcept
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

// Respond to a heater fault. The heater has already been turned off and its status set to 'fault' when this is called from the Heat module.
// The Heat module will generate an appropriate error message, so no need to do that here.
void GCodes::HandleHeaterFault() noexcept
{
	if (heaterFaultState == HeaterFaultState::noFault && fileGCode->OriginalMachineState().DoingFile())
	{
		heaterFaultState = HeaterFaultState::pausePending;
		heaterFaultTime = millis();
	}
}

// Check for and respond to a heater fault, returning true if we should exit
void GCodes::CheckHeaterFault() noexcept
{
	switch (heaterFaultState)
	{
	case HeaterFaultState::noFault:
	default:
		break;

	case HeaterFaultState::pausePending:
		if (   IsReallyPrinting()
			&& autoPauseGCode->IsCompletelyIdle()
			&& LockMovement(*autoPauseGCode)							// need to lock movement before executing the pause macro
		   )
		{
			reprap.GetHeat().SwitchOffAll(false);						// turn off all extruder heaters
			DoPause(*autoPauseGCode, PauseReason::heaterFault, "Heater fault");
			heaterFaultState = HeaterFaultState::timing;
		}
		else if (IsPausing() || IsPaused())
		{
			heaterFaultState = HeaterFaultState::timing;
		}
		// no break

	case HeaterFaultState::timing:
		if (millis() - heaterFaultTime >= heaterFaultTimeout)
		{
			StopPrint(StopPrintReason::abort);
			reprap.GetHeat().SwitchOffAll(true);
			platform.MessageF(ErrorMessage, "Shutting down due to un-cleared heater fault after %lu seconds\n", heaterFaultTimeout/1000);
			heaterFaultState = HeaterFaultState::stopping;
			heaterFaultTime = millis();
		}
		break;

	case HeaterFaultState::stopping:
		if (millis() - heaterFaultTime >= 1000)			// wait 1 second for the message to be picked up by DWC and PanelDue
		{
			platform.AtxPowerOff(false);
			heaterFaultState = HeaterFaultState::stopped;
		}
		break;
	}
}

// Return a current extrusion factor as a fraction
float GCodes::GetExtrusionFactor(size_t extruder) noexcept
{
	return (extruder < numExtruders) ? extrusionFactors[extruder] : 0.0;
}

Pwm_t GCodes::ConvertLaserPwm(float reqVal) const noexcept
{
	return (uint16_t)constrain<long>(lrintf((reqVal * 65535)/laserMaxPower), 0, 65535);
}

// Get the height in user coordinates of the last printing move
bool GCodes::GetLastPrintingHeight(float& height) const noexcept
{
	if (IsReallyPrinting() && lastPrintingMoveHeight > 0.0)
	{
		height = lastPrintingMoveHeight;
		return true;
	}
	return false;
}

// Assign the heightmap using the given parameters
void GCodes::AssignGrid(float xRange[2], float yRange[2], float radius, float spacing[2]) noexcept
{
	defaultGrid.Set(xRange, yRange, radius, spacing);
	reprap.GetMove().AccessHeightMap().SetGrid(defaultGrid);
}

void GCodes::ActivateHeightmap(bool activate) noexcept
{
	reprap.GetMove().UseMesh(activate);
	if (activate)
	{
		// Update the current position to allow for any bed compensation at the current XY coordinates
		reprap.GetMove().GetCurrentUserPosition(moveBuffer.coords, 0, reprap.GetCurrentTool());
		ToolOffsetInverseTransform(moveBuffer.coords, currentUserPosition);		// update user coordinates to reflect any height map offset at the current position

#if HAS_LINUX_INTERFACE
		// Set a dummy heightmap filename
		if (reprap.UsingLinuxInterface())
		{
			HeightMap& map = reprap.GetMove().AccessHeightMap();
			map.SetFileName(DefaultHeightMapFile);
		}
#endif
	}
}

#if HAS_MASS_STORAGE

// Start timing SD card file writing
GCodeResult GCodes::StartSDTiming(GCodeBuffer& gb, const StringRef& reply) noexcept
{
	const float bytesReq = (gb.Seen('S')) ? gb.GetFValue() : 10.0;
	const bool useCrc = (gb.Seen('C') && gb.GetUIValue() != 0);
	timingBytesRequested = (uint32_t)(bytesReq * (float)(1024 * 1024));
	FileStore * const f = platform.OpenFile(platform.GetGCodeDir(), TimingFileName, (useCrc) ? OpenMode::writeWithCrc : OpenMode::write, timingBytesRequested);
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

#if SUPPORT_12864_LCD

// Set the speed factor. Value passed is a fraction.
void GCodes::SetSpeedFactor(float factor) noexcept
{
	speedFactor = constrain<float>(factor, 0.1, 5.0);
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
bool GCodes::ProcessCommandFromLcd(const char *cmd) noexcept
{
	if (lcdGCode->IsCompletelyIdle())
	{
		lcdGCode->PutAndDecode(cmd);
		return true;
	}
	return false;
}

int GCodes::GetHeaterNumber(unsigned int itemNumber) const noexcept
{
	if (itemNumber < 80)
	{
		ReadLockedPointer<Tool> const tool = (itemNumber == 79) ? reprap.GetLockedCurrentTool() : reprap.GetTool(itemNumber);
		return (tool.IsNotNull() && tool->HeaterCount() != 0) ? tool->Heater(0) : -1;
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
		ReadLockedPointer<Tool> const tool = (itemNumber == 79) ? reprap.GetLockedCurrentTool() : reprap.GetTool(itemNumber);
		return (tool.IsNotNull()) ? tool->GetToolHeaterActiveTemperature(0) : 0.0;
	}

	return reprap.GetHeat().GetActiveTemperature(GetHeaterNumber(itemNumber));
}

float GCodes::GetItemStandbyTemperature(unsigned int itemNumber) const noexcept
{
	if (itemNumber < 80)
	{
		ReadLockedPointer<Tool> const tool = (itemNumber == 79) ? reprap.GetLockedCurrentTool() : reprap.GetTool(itemNumber);
		return (tool.IsNotNull()) ? tool->GetToolHeaterStandbyTemperature(0) : 0.0;
	}

	return reprap.GetHeat().GetStandbyTemperature(GetHeaterNumber(itemNumber));
}

void GCodes::SetItemActiveTemperature(unsigned int itemNumber, float temp) noexcept
{
	if (itemNumber < 80)
	{
		ReadLockedPointer<Tool> const tool = (itemNumber == 79) ? reprap.GetLockedCurrentTool() : reprap.GetTool(itemNumber);
		if (tool.IsNotNull())
		{
			tool->SetToolHeaterActiveTemperature(0, temp);
		}
	}
	else
	{
		reprap.GetHeat().SetActiveTemperature(GetHeaterNumber(itemNumber), temp);
	}
}

void GCodes::SetItemStandbyTemperature(unsigned int itemNumber, float temp) noexcept
{
	if (itemNumber < 80)
	{
		ReadLockedPointer<Tool> const tool = (itemNumber == 79) ? reprap.GetLockedCurrentTool() : reprap.GetTool(itemNumber);
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

#endif

// End
