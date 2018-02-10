/*
 * Scanner.cpp
 *
 *  Created on: 21 Mar 2017
 *      Author: Christian
 */

#include "Scanner.h"

#if SUPPORT_SCANNER

#include "RepRap.h"
#include "Platform.h"


const char* const ALIGN_ON_G = "align_on.g";
const char* const ALIGN_OFF_G = "align_off.g";
const char* const SCAN_PRE_G = "scan_pre.g";
const char* const SCAN_POST_G = "scan_post.g";
const char* const CALIBRATE_PRE_G = "calibrate_pre.g";
const char* const CALIBRATE_POST_G = "calibrate_post.g";


void Scanner::Init()
{
	longWait = millis();

	enabled = false;
	SetState(ScannerState::Disconnected);
	bufferPointer = 0;
	progress = 0.0f;
	fileBeingUploaded = nullptr;
}

void Scanner::SetState(const ScannerState s)
{
	progress = 0.0f;
	doingGCodes = false;
	state = s;
}

void Scanner::Exit()
{
	if (IsEnabled())
	{
		// Notify the scanner that we cannot complete the current action
		if (state == ScannerState::Scanning || state == ScannerState::Calibrating)
		{
			Cancel();
		}

		// Delete any pending uploads
		if (fileBeingUploaded != nullptr)
		{
			fileBeingUploaded->Close();
			fileBeingUploaded = nullptr;
			platform.GetMassStorage()->Delete(SCANS_DIRECTORY, uploadFilename);
		}

		// Pretend the scanner is no longer connected
		SetState(ScannerState::Disconnected);
	}
}

void Scanner::Spin()
{
	// Is the 3D scanner extension enabled at all and is a device registered?
	if (!IsEnabled() || state == ScannerState::Disconnected)
	{
		platform.ClassReport(longWait);
		return;
	}

	// Check if the scanner device is still present
	if (!SERIAL_MAIN_DEVICE)
	{
		// Scanner has been detached - report a warning if we were scanning or uploading
		if (state == ScannerState::ScanningPre || state == ScannerState::Scanning || state == ScannerState::ScanningPost ||
			state == ScannerState::Uploading)
		{
			platform.Message(WarningMessage, "Scanner disconnected while a 3D scan or upload was in progress");
		}

		// Delete any pending uploads
		if (fileBeingUploaded != nullptr)
		{
			fileBeingUploaded->Close();
			fileBeingUploaded = nullptr;
			platform.GetMassStorage()->Delete(SCANS_DIRECTORY, uploadFilename);
		}
		SetState(ScannerState::Disconnected);

		// Cannot do anything else...
		platform.ClassReport(longWait);
		return;
	}

	// Deal with the current state
	switch (state)
	{
		case ScannerState::EnablingAlign:
			if (!IsDoingFileMacro())
			{
				// Send ALIGN ON to the scanner
				SERIAL_MAIN_DEVICE.write("ALIGN ON\n");
				SERIAL_MAIN_DEVICE.flush();

				SetState(ScannerState::Idle);
			}
			break;

		case ScannerState::DisablingAlign:
			if (!IsDoingFileMacro())
			{
				// Send ALIGN OFF to the scanner
				SERIAL_MAIN_DEVICE.write("ALIGN OFF\n");
				SERIAL_MAIN_DEVICE.flush();

				SetState(ScannerState::Idle);
			}
			break;

		case ScannerState::ScanningPre:
			if (!IsDoingFileMacro())
			{
				// Pre macro complete, build and send SCAN command
				String<MaxFilenameLength + 16> scanCommand;
				scanCommand.GetRef().printf("SCAN %d %s\n", scanParam, scanFilename.c_str());

				SERIAL_MAIN_DEVICE.write(scanCommand.c_str());
				SERIAL_MAIN_DEVICE.flush();

				// Advance to the next state
				SetState(ScannerState::Scanning);
			}
			break;

		// Scanning state is controlled by external 3D scanner

		case ScannerState::ScanningPost:
			if (!IsDoingFileMacro())
			{
				// Post macro complete, reset the state
				SetState(ScannerState::Idle);
			}
			break;

		case ScannerState::CalibratingPre:
			if (!IsDoingFileMacro())
			{
				// Pre macro complete, send CALIBRATE
				SERIAL_MAIN_DEVICE.write("CALIBRATE\n");
				SERIAL_MAIN_DEVICE.flush();

				// Advance to the next state
				SetState(ScannerState::Calibrating);
			}
			break;

		// Calibrating state is controlled by external 3D scanner

		case ScannerState::CalibratingPost:
			if (!IsDoingFileMacro())
			{
				// Post macro complete, reset the state
				SetState(ScannerState::Idle);
			}
			break;

		case ScannerState::Uploading:
			// Write incoming scan data from USB to the file
			while (SERIAL_MAIN_DEVICE.available() > 0)
			{
				char b = static_cast<char>(SERIAL_MAIN_DEVICE.read());
				if (fileBeingUploaded->Write(&b, sizeof(char)))
				{
					uploadBytesLeft--;
					if (uploadBytesLeft == 0)
					{
						if (reprap.Debug(moduleScanner))
						{
							platform.MessageF(HttpMessage, "Finished uploading %u bytes of scan data\n", uploadSize);
						}

						fileBeingUploaded->Close();
						fileBeingUploaded = nullptr;

						SetState(ScannerState::Idle);
						break;
					}
				}
				else
				{
					fileBeingUploaded->Close();
					fileBeingUploaded = nullptr;
					platform.GetMassStorage()->Delete(SCANS_DIRECTORY, uploadFilename);

					platform.Message(ErrorMessage, "Failed to write scan file\n");
					SetState(ScannerState::Idle);
					break;
				}
			}
			break;

		default:
			// Pick up incoming commands only if the GCodeBuffer is idle.
			// The GCodes class will do the processing for us.
			if (serialGCode->IsIdle() && SERIAL_MAIN_DEVICE.available() > 0)
			{
				char b = static_cast<char>(SERIAL_MAIN_DEVICE.read());
				if (b == '\n' || b == '\r')
				{
					buffer[bufferPointer] = 0;
					ProcessCommand();
					bufferPointer = 0;
				}
				else
				{
					buffer[bufferPointer++] = b;
					if (bufferPointer >= ScanBufferSize)
					{
						platform.Message(ErrorMessage, "Scan buffer overflow\n");
						bufferPointer = 0;
					}
				}
			}
			break;
	}

	platform.ClassReport(longWait);
}

// Process incoming commands from the scanner board
void Scanner::ProcessCommand()
{
	// Output some info if debugging is enabled
	if (reprap.Debug(moduleScanner))
	{
		platform.MessageF(HttpMessage, "Scanner request: '%s'\n", buffer);
	}

	// Register request: M751
	if (StringEquals(buffer, "M751"))
	{
		SERIAL_MAIN_DEVICE.write("OK\n");
		SERIAL_MAIN_DEVICE.flush();

		SetState(ScannerState::Idle);
	}

	// G-Code request: GCODE <CODE>
	else if (StringStartsWith(buffer, "GCODE "))
	{
		doingGCodes = true;
		serialGCode->Put(&buffer[6], bufferPointer - 6);
	}

	// Switch to post-processing mode: POSTPROCESS
	else if (StringEquals(buffer, "POSTPROCESS"))
	{
		SetState(ScannerState::PostProcessing);
	}

	// Progress indicator: PROGRESS <PERCENT>
	else if (StringStartsWith(buffer, "PROGRESS "))
	{
		float parsedProgress = atof(&buffer[9]);
		progress = constrain<float>(parsedProgress, 0.0f, 100.0f);
	}

	// Upload request: UPLOAD <SIZE> <FILENAME>
	else if (StringStartsWith(buffer, "UPLOAD "))
	{
		uploadSize = atoi(&buffer[7]);
		uploadFilename = nullptr;
		for(size_t i = 8; i < bufferPointer - 1; i++)
		{
			if (buffer[i] == ' ')
			{
				uploadFilename = &buffer[i+1];
				break;
			}
		}

		if (uploadFilename != nullptr)
		{
			uploadBytesLeft = uploadSize;
			fileBeingUploaded = platform.OpenFile(SCANS_DIRECTORY, uploadFilename, OpenMode::write);
			if (fileBeingUploaded != nullptr)
			{
				SetState(ScannerState::Uploading);
				if (reprap.Debug(moduleScanner))
				{
					platform.MessageF(HttpMessage, "Starting scan upload for file %s (%u bytes total)\n", uploadFilename, uploadSize);
				}
			}
		}
		else
		{
			platform.Message(ErrorMessage, "Malformed scanner upload request\n");
		}
	}

	// Acknowledgement: OK
	else if (StringEquals(buffer, "OK"))
	{
		if (state == ScannerState::Scanning)
		{
			// Scan complete, run scan_post.g
			DoFileMacro(SCAN_POST_G);
			SetState(ScannerState::ScanningPost);
		}
		else if (state == ScannerState::PostProcessing)
		{
			// Post-processing complete, reset the state
			SetState(ScannerState::Idle);
		}
		else if (state == ScannerState::Calibrating)
		{
			// Calibration complete, run calibrate_post.g
			DoFileMacro(CALIBRATE_POST_G);
			SetState(ScannerState::CalibratingPost);
		}
	}

	// Error message: ERROR msg
	else if (StringStartsWith(buffer, "ERROR"))
	{
		// if this command contains a message, report it
		if (bufferPointer > 6)
		{
			platform.MessageF(ErrorMessage, "%s\n", &buffer[6]);
		}

		// reset the state
		SetState(ScannerState::Idle);
	}
}

// Enable the scanner extensions
bool Scanner::Enable()
{
	enabled = true;
	return true;
}

// Register a scanner device
bool Scanner::Register()
{
	if (IsRegistered())
	{
		// Don't do anything if a device is already registered
		return true;
	}

	SERIAL_MAIN_DEVICE.write("OK\n");
	SERIAL_MAIN_DEVICE.flush();

	SetState(ScannerState::Idle);
	return true;
}

// Initiate a new scan
bool Scanner::StartScan(const char *filename, int param)
{
	if (state != ScannerState::Idle)
	{
		// We're doing something else at this moment
		return false;
	}
	if (!serialGCode->IsIdle())
	{
		// The G-code buffer is still blocked
		return false;
	}

	// Copy the scan length/degree and the filename
	scanParam = param;
	scanFilename.copy(filename);

	// Run the scan_pre macro and wait for it to finish
	DoFileMacro(SCAN_PRE_G);
	SetState(ScannerState::ScanningPre);

	return true;
}

// Cancel current 3D scanner action
bool Scanner::Cancel()
{
	if (state == ScannerState::ScanningPre || state == ScannerState::ScanningPost ||
		state == ScannerState::CalibratingPre || state == ScannerState::CalibratingPost)
	{
		// We're waiting for the scan/calibration to start/end, so wait a little longer
		return false;
	}

	if (state != ScannerState::Scanning && state != ScannerState::Calibrating)
	{
		// Only permit this request if the 3D scanner is actually working
		return true;
	}

	// Send CANCEL request to the scanner
	SERIAL_MAIN_DEVICE.write("CANCEL\n");
	SERIAL_MAIN_DEVICE.flush();

	SetState(ScannerState::Idle);
	return true;
}

// Send ALIGN ON/OFF to the 3D scanner
bool Scanner::SetAlignment(bool on)
{
	if (state != ScannerState::Idle)
	{
		// Not ready yet, try again later
		return false;
	}
	if (!serialGCode->IsIdle())
	{
		// The G-code buffer is still blocked
		return false;
	}

	DoFileMacro(on ? ALIGN_ON_G : ALIGN_OFF_G);
	SetState(on ? ScannerState::EnablingAlign : ScannerState::DisablingAlign);
	return true;
}

// Sends SHUTDOWN to the 3D scanner and unregisters it
bool Scanner::Shutdown()
{
	if (state != ScannerState::Idle)
	{
		// Not ready yet, try again later
		return false;
	}

	// Send SHUTDOWN to the scanner
	SERIAL_MAIN_DEVICE.write("SHUTDOWN\n");
	SERIAL_MAIN_DEVICE.flush();

	// Unregister it
	SetState(ScannerState::Disconnected);

	return true;
}

// Calibrate the 3D scanner
bool Scanner::Calibrate()
{
	if (state != ScannerState::Idle)
	{
		// We're doing something else at this moment
		return false;
	}
	if (!serialGCode->IsIdle())
	{
		// The G-code buffer is still blocked
		return false;
	}

	// In theory it would be good to verify if this succeeds, but the scanner client cannot give feedback (yet)
	DoFileMacro(CALIBRATE_PRE_G);
	SetState(ScannerState::CalibratingPre);

	return true;
}

const char Scanner::GetStatusCharacter() const
{
	switch (state)
	{
		case ScannerState::Disconnected:
			return 'D';

		case ScannerState::EnablingAlign:
		case ScannerState::DisablingAlign:
		case ScannerState::Idle:
			return 'I';

		case ScannerState::ScanningPre:
		case ScannerState::Scanning:
		case ScannerState::ScanningPost:
			return 'S';

		case ScannerState::PostProcessing:
			return 'P';

		case ScannerState::CalibratingPre:
		case ScannerState::Calibrating:
		case ScannerState::CalibratingPost:
			return 'C';

		case ScannerState::Uploading:
			return 'U';
	}

	return 'I';
}

// Return the progress of the current operation
float Scanner::GetProgress() const
{
	if (state == ScannerState::Uploading)
	{
		return ((float)(uploadSize - uploadBytesLeft) / (float)uploadSize) * 100.0f;
	}

	return progress;
}

// Is a macro file being executed?
bool Scanner::IsDoingFileMacro() const
{
	return (serialGCode->IsDoingFileMacro() || (serialGCode->Seen('M') && serialGCode->GetIValue() == 98));
}

// Perform a file macro using the GCodeBuffer
void Scanner::DoFileMacro(const char *filename)
{
	if (platform.GetMassStorage()->FileExists(SYS_DIR, filename))
	{
		String<MaxFilenameLength + 7> gcode;
		gcode.GetRef().printf("M98 P%s\n", filename);
		serialGCode->Put(gcode.c_str());
	}
}

#endif
