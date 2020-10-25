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
#include "MessageType.h"

class Platform;

class GCodeBuffer;

class OutputBuffer;
class OutputStack;

constexpr size_t MaxFileChunkSize = 448;	// Maximum size of file chunks for reading files from the SBC. Should be a multiple of sizeof(CanMessageFirmwareUpdateResponse::data) for best CAN performance

// G-Code input class for an SPI channel
class LinuxInterface
{
public:
	LinuxInterface() noexcept;
	~LinuxInterface();

	// The Init method must be called prior to calling any of the other methods. Use reprap.UsingLinuxInterface() to guard calls to other members.
	// OTOH, calling Init when we don't have a SBC connected may cause problems due to noise pickup on the SPI CS and clock inputs
	void Init() noexcept;
	void TaskLoop() noexcept;
	void Diagnostics(MessageType mtype) noexcept;
	bool IsConnected() const noexcept;

	bool HasPrintStarted();
	bool HasPrintStopped();
	StopPrintReason GetPrintStopReason() const { return printStopReason; }
	bool FillBuffer(GCodeBuffer &gb) noexcept;		// Try to fill up the G-code buffer with the next available G-code

	void SetPauseReason(FilePosition position, PrintPausedReason reason) noexcept;	// Notify Linux that the print has been paused

	void HandleGCodeReply(MessageType type, const char *reply) noexcept;	// accessed by Platform
	void HandleGCodeReply(MessageType type, OutputBuffer *buffer) noexcept;	// accessed by Platform

#if SUPPORT_CAN_EXPANSION
	const char *GetFileChunk(const char *filename, uint32_t offset, uint32_t maxLength, int32_t& bytesRead, uint32_t &fileLength) noexcept; 	// Request a file chunk and resume the given task when it has been received
#endif

private:
	DataTransfer transfer;
	bool wasConnected;
	uint32_t numDisconnects;

	GCodeFileInfo fileInfo;
	FilePosition pauseFilePosition;
	PrintPausedReason pauseReason;
	bool reportPause, reportPauseWritten, printStarted, printStopped;
	StopPrintReason printStopReason;

	char codeBuffer[SpiCodeBufferSize];
	uint16_t rxPointer, txPointer, txLength;
	bool sendBufferUpdate;

	uint32_t iapWritePointer;
	uint32_t iapRamAvailable;											// must be at least 64Kb otherwise the SPI IAP can't work

#if SUPPORT_CAN_EXPANSION
	// Data needed when a CAN expansion board requests a firmware file chunk
	String<FILENAME_MAX> requestedFileName;
	uint32_t requestedFileOffset, requestedFileLength;
	BinarySemaphore requestedFileSemaphore;
	char requestedFileChunk[MaxFileChunkSize];
	int32_t requestedFileDataLength;
#endif

	static Mutex gcodeReplyMutex;											// static so that the LinuxInterface is safe to delete even is the mutex is linked into the mutex chain or is in use
	OutputStack *gcodeReply;

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
