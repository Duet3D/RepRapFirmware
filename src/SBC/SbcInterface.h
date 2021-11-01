/*
 * SbcInterface.h
 *
 *  Created on: 29 Mar 2019
 *      Authors: Christian
 */

#ifndef SRC_SBC_SBCINTERFACE_H_
#define SRC_SBC_SBCINTERFACE_H_

#include <RepRapFirmware.h>

#if HAS_SBC_INTERFACE

#include "RTOSIface/RTOSIface.h"

#include "GCodes/GCodes.h"
#include "GCodes/GCodeChannel.h"
#include "GCodes/GCodeFileInfo.h"
#include "SbcMessageFormats.h"
#include "DataTransfer.h"

class Platform;

class GCodeBuffer;

class OutputBuffer;
class OutputStack;

//#define TRACK_FILE_CODES			// Uncomment this to enable code <-> code reply tracking for the file G-code channel

// G-Code input class for an SPI channel
class SbcInterface
{
public:
	SbcInterface() noexcept;

	// The Init method must be called prior to calling any of the other methods. Use reprap.UsingSbcInterface() to guard calls to other members.
	// OTOH, calling Init when we don't have a SBC connected may cause problems due to noise pickup on the SPI CS and clock inputs
	void Init() noexcept;
	void Spin() noexcept;														// Only called in standalone mode by the main loop
	[[noreturn]] void TaskLoop() noexcept;
	void Diagnostics(MessageType mtype) noexcept;
	bool IsConnected() const noexcept { return isConnected; }

	void EventOccurred(bool timeCritical = false) noexcept;						// Called when a new event has happened. It can optionally start off a new transfer immediately
	GCodeResult HandleM576(GCodeBuffer& gb, const StringRef& reply) noexcept;	// Set the SPI communication parameters

	bool IsPrintAborted() noexcept;												// Check if the current print has been aborted
	bool FillBuffer(GCodeBuffer &gb) noexcept;									// Try to fill up the G-code buffer with the next available G-code

	void SetPauseReason(FilePosition position, PrintPausedReason reason) noexcept;	// Set parameters for the next pause request
	void SetEmergencyPauseReason(FilePosition position, PrintPausedReason reason) noexcept;	// Set parameters for the next emergency pause request
	void ReportPause() noexcept;												// Report that the print has been paused

	void HandleGCodeReply(MessageType type, const char *reply) noexcept;		// accessed by Platform
	void HandleGCodeReply(MessageType type, OutputBuffer *buffer) noexcept;		// accessed by Platform

	bool GetFileChunk(const char *filename, uint32_t offset, char *buffer, uint32_t& bufferLength, uint32_t& fileLength) noexcept;

	bool FileExists(const char *filename) noexcept;
	bool DeleteFileOrDirectory(const char *fileOrDirectory) noexcept;

	FileHandle OpenFile(const char *filename, OpenMode mode, FilePosition& fileLength, uint32_t preAllocSize = 0) noexcept;
	int ReadFile(FileHandle handle, char *buffer, size_t bufferLength) noexcept;
	bool WriteFile(FileHandle handle, const char *buffer, size_t bufferLength) noexcept;
	bool SeekFile(FileHandle handle, FilePosition offset) noexcept;
	bool TruncateFile(FileHandle handle) noexcept;
	void CloseFile(FileHandle handle) noexcept;

private:
	DataTransfer transfer;
	volatile bool isConnected;
	TransferState state;
	uint32_t numDisconnects, numTimeouts, lastTransferTime;

	uint32_t maxDelayBetweenTransfers, maxFileOpenDelay, numMaxEvents;
	bool skipNextDelay;
	volatile bool delaying;
	volatile uint32_t numEvents;

	GCodeFileInfo fileInfo;
	FilePosition pauseFilePosition;
	PrintPausedReason pauseReason;
	bool reportPause, reportPauseWritten, printAborted;

	char *codeBuffer;
	volatile uint16_t rxPointer, txPointer, txEnd;
	volatile bool sendBufferUpdate;

	uint32_t iapRamAvailable;											// must be at least 32Kb otherwise the SPI IAP can't work

	// Data needed when a CAN expansion board requests a firmware file chunk
	volatile bool waitingForFileChunk;
	bool fileChunkRequestSent;
	String<MaxFilenameLength> requestedFileName;
	uint32_t requestedFileOffset, requestedFileLength;
	BinarySemaphore requestedFileSemaphore;
	char *requestedFileBuffer;
	int32_t requestedFileDataLength;

	// File I/O
	Mutex fileMutex;													// locked while a file operation is performed
	BinarySemaphore fileSemaphore;										// resolved when the requested file operation has finished

	enum class FileOperation {
		none,
		checkFileExists,
		deleteFileOrDirectory,
		openRead,
		openWrite,
		openAppend,
		read,
		write,
		seek,
		truncate,
		close
	} fileOperation;
	bool fileOperationPending;

	const char *filePath;
	FileHandle fileHandle;
	bool fileSuccess;

	uint32_t filePreAllocSize;
	char *fileReadBuffer;
	const char *fileWriteBuffer;
	size_t fileBufferLength;
	FilePosition fileOffset;

	static volatile OutputStack gcodeReply;
	static Mutex gcodeReplyMutex;											// static so that the SbcInterface is safe to delete even is the mutex is linked into the mutex chain or is in use

#ifdef TRACK_FILE_CODES
	volatile size_t fileCodesRead, fileCodesHandled, fileMacrosRunning, fileMacrosClosing;
#endif

	void ExchangeData() noexcept;											// Exchange data between RRF and the SBC
	[[noreturn]] void ReceiveAndStartIap(const char *iapChunk, size_t length) noexcept;	// Receive and start the IAP binary
	void InvalidateResources() noexcept;									// Invalidate local resources on connection errors
	void InvalidateBufferedCodes(GCodeChannel channel) noexcept;           	// Invalidate every buffered G-code of the corresponding channel from the buffer ring
};

inline void SbcInterface::SetPauseReason(FilePosition position, PrintPausedReason reason) noexcept
{
	TaskCriticalSectionLocker locker;
	pauseFilePosition = position;
	pauseReason = reason;
	reportPauseWritten = false;
}

inline void SbcInterface::SetEmergencyPauseReason(FilePosition position, PrintPausedReason reason) noexcept
{
	pauseFilePosition = position;
	pauseReason = reason;
	reportPauseWritten = false;
	reportPause = true;
}

inline void SbcInterface::ReportPause() noexcept
{
	reportPause = true;
}

inline bool SbcInterface::IsPrintAborted() noexcept
{
	TaskCriticalSectionLocker locker;
	if (printAborted)
	{
		printAborted = false;
		return true;
	}
	return false;
}

#endif

#endif
