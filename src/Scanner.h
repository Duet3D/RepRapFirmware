/*
 * Scanner.h
 *
 *  Created on: 21 Mar 2017
 *      Author: Christian
 */

#ifndef SRC_SCANNER_H_
#define SRC_SCANNER_H_

#include "RepRapFirmware.h"
#include "GCodes/GCodeBuffer/GCodeBuffer.h"
#include "ObjectModel/ObjectModel.h"

#if SUPPORT_SCANNER

#define SCANNER_AS_SEPARATE_TASK	(0)					// set to 1 to use a separate task for the scanner (requires RTOS enabled too)
//#define SCANNER_AS_SEPARATE_TASK	(defined(RTOS))		// set to 1 to use a separate task for the scanner (requires RTOS enabled too)

const size_t ScanBufferSize = 128;						// Size of the buffer for incoming commands

enum class ScannerState
{
	Disconnected,		// scanner mode is disabled
	Idle,				// scanner is registered but not active

	EnablingAlign,		// running align_on.g
	DisablingAlign,		// running align_off.g

	ScanningPre,		// running scan_pre.g
	Scanning,			// 3D scanner is scanning
	ScanningPost,		// running scan_post.g

	PostProcessing,		// post-processor is busy

	CalibratingPre,		// running calibrate_pre.g
	Calibrating,		// 3D scanner is calibrating
	CalibratingPost,	// running calibrate_post.g

	Uploading			// uploading a binary file
};

class Scanner INHERIT_OBJECT_MODEL
{
public:
	friend class GCodes;

	Scanner(Platform& p) noexcept : platform(p) { }
	void Init() noexcept;
	void Exit() noexcept;
	void Spin() noexcept;

	bool IsEnabled() const noexcept { return enabled; }			// Is the usage of a 3D scanner enabled?
	bool Enable() noexcept;										// Enable 3D scanner extension. Returns true when done

	bool IsRegistered() const noexcept;							// Is the 3D scanner registered and ready to use?
	void Register() noexcept;									// Register a 3D scanner instance
	// External scanners are automatically unregistered when the main port (USB) is closed

	// Start a new 3D scan. Returns true when the scan has been initiated
	bool StartScan(const char *filename, int param, int resolution, int mode) noexcept;

	bool Cancel() noexcept;										// Cancel current 3D scanner action. Returns true when done
	bool Calibrate(int mode) noexcept;							// Calibrate the 3D scanner. Returns true when done
	bool SetAlignment(bool on) noexcept;						// Send ALIGN ON/OFF to the 3D scanner. Returns true when done
	bool Shutdown() noexcept;									// Send SHUTDOWN to the scanner and unregisters it

	bool DoingGCodes() const noexcept { return doingGCodes; }	// Has the scanner run any G-codes since the last state transition?
	const char GetStatusCharacter() const noexcept;				// Returns the status char for the status response
	float GetProgress() const noexcept;							// Returns the progress of the current action

protected:
	DECLARE_OBJECT_MODEL

private:
	GCodeBuffer *serialGCode;

	void SetGCodeBuffer(GCodeBuffer *gb) noexcept;

	void SetState(const ScannerState s) noexcept;
	void ProcessCommand() noexcept;

	bool IsDoingFileMacro() const noexcept;
	void DoFileMacro(const char *filename) noexcept;

	Platform& platform;

	bool enabled;

	bool doingGCodes;
	float progress;
	ScannerState state;

	char buffer[ScanBufferSize];
	size_t bufferPointer;

	int calibrationMode;

	String<MaxFilenameLength> scanFilename;
	int scanRange, scanResolution, scanMode;

	const char *uploadFilename;
	size_t uploadSize, uploadBytesLeft;
	FileStore *fileBeingUploaded;
};

inline bool Scanner::IsRegistered() const noexcept { return (state != ScannerState::Disconnected); }
inline void Scanner::SetGCodeBuffer(GCodeBuffer *gb) noexcept { serialGCode = gb; }

#endif

#endif
