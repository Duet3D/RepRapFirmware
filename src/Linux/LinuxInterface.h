/*
 * LinuxInterface.h
 *
 *  Created on: 29 Mar 2019
 *      Authors: Christian
 */

#ifndef SRC_LINUX_LINUXINTERFACE_H_
#define SRC_LINUX_LINUXINTERFACE_H_

#include <RepRapFirmware.h>

#if HAS_LINUX_INTERFACE

#include "RTOSIface/RTOSIface.h"

#include "GCodes/GCodes.h"
#include "GCodes/GCodeChannel.h"
#include "GCodes/GCodeFileInfo.h"
#include "LinuxMessageFormats.h"
#include "DataTransfer.h"

class Platform;

class GCodeBuffer;

class OutputBuffer;
class OutputStack;

//#define TRACK_FILE_CODES			// Uncomment this to enable code <-> code reply tracking for the file G-code channel

// G-Code input class for an SPI channel
class LinuxInterface
{
public:
	LinuxInterface() noexcept;

	// The Init method must be called prior to calling any of the other methods. Use reprap.UsingLinuxInterface() to guard calls to other members.
	// OTOH, calling Init when we don't have a SBC connected may cause problems due to noise pickup on the SPI CS and clock inputs
	void Init() noexcept;
	[[noreturn]] void TaskLoop() noexcept;
	void Diagnostics(MessageType mtype) noexcept;
	bool IsConnected() const noexcept { return isConnected; }

	bool HasPrintStarted();
	bool HasPrintStopped();
	StopPrintReason GetPrintStopReason() const { return printStopReason; }
	bool FillBuffer(GCodeBuffer &gb) noexcept;		// Try to fill up the G-code buffer with the next available G-code

	void SetPauseReason(FilePosition position, PrintPausedReason reason) noexcept;	// Notify Linux that the print has been paused

	void HandleGCodeReply(MessageType type, const char *reply) noexcept;	// accessed by Platform
	void HandleGCodeReply(MessageType type, OutputBuffer *buffer) noexcept;	// accessed by Platform

	bool GetFileChunk(const char *filename, uint32_t offset, char *buffer, uint32_t& bufferLength, uint32_t& fileLength) noexcept;

private:
	DataTransfer transfer;
	bool isConnected;
	uint32_t numDisconnects, numTimeouts;

	GCodeFileInfo fileInfo;
	FilePosition pauseFilePosition;
	PrintPausedReason pauseReason;
	bool reportPause, reportPauseWritten, printStarted, printStopped;
	StopPrintReason printStopReason;

	char *codeBuffer;
	volatile uint16_t rxPointer, txPointer, txEnd;
	volatile bool sendBufferUpdate;

	uint32_t iapWritePointer;
	uint32_t iapRamAvailable;											// must be at least 64Kb otherwise the SPI IAP can't work

	// Data needed when a CAN expansion board requests a firmware file chunk
	volatile bool waitingForFileChunk;
	bool fileChunkRequestSent;
	String<MaxFilenameLength> requestedFileName;
	uint32_t requestedFileOffset, requestedFileLength;
	BinarySemaphore requestedFileSemaphore;
	char *requestedFileBuffer;
	int32_t requestedFileDataLength;

	static volatile OutputStack gcodeReply;
	static Mutex gcodeReplyMutex;											// static so that the LinuxInterface is safe to delete even is the mutex is linked into the mutex chain or is in use

#ifdef TRACK_FILE_CODES
	volatile size_t fileCodesRead, fileCodesHandled, fileMacrosRunning, fileMacrosClosing;
#endif

	void InvalidateBufferChannel(GCodeChannel channel) noexcept;            // Invalidate every buffered G-code of the corresponding channel from the buffer ring
};

inline void LinuxInterface::SetPauseReason(FilePosition position, PrintPausedReason reason) noexcept
{
	TaskCriticalSectionLocker locker;
	pauseFilePosition = position;
	pauseReason = reason;
	reportPauseWritten = false;
	reportPause = true;
}

inline bool LinuxInterface::HasPrintStarted()
{
	TaskCriticalSectionLocker locker;
	if (printStarted)
	{
		printStarted = false;
		return true;
	}
	return false;
}

inline bool LinuxInterface::HasPrintStopped()
{
	TaskCriticalSectionLocker locker;
	if (printStopped)
	{
		printStopped = false;
		return true;
	}
	return false;
}

#endif

#endif
