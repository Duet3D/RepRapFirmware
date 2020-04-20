/*
 * LinuxInterface.h
 *
 *  Created on: 29 Mar 2019
 *      Authors: Christian
 */

#ifndef SRC_LINUX_LINUXINTERFACE_H_
#define SRC_LINUX_LINUXINTERFACE_H_

#include "RTOSIface/RTOSIface.h"

#include "GCodes/GCodeChannel.h"
#include "GCodes/GCodeFileInfo.h"
#include "LinuxMessageFormats.h"
#include "MessageType.h"

class Platform;

class DataTransfer;
class GCodeBuffer;

class OutputBuffer;
class OutputStack;

constexpr size_t MaxFileChunkSize = 448;	// Maximum size of file chunks for reading files from the SBC. Should be a multiple of sizeof(CanMessageFirmwareUpdateResponse::data) for best CAN performance

// G-Code input class for an SPI channel
class LinuxInterface
{
public:
	friend class Platform;

	LinuxInterface();
	void Init();
	void Spin();
	void Diagnostics(MessageType mtype);
	bool IsConnected() const;

	bool FillBuffer(GCodeBuffer &gb);		// Try to fill up the G-code buffer with the next available G-code

	void SetPauseReason(FilePosition position, PrintPausedReason reason);	// Notify Linux that the print has been paused
	const char *GetFileChunk(const char *filename, uint32_t offset, uint32_t maxLength, int32_t& bytesRead, uint32_t &fileLength); 	// Request a file chunk and resume the given task when it has been received

private:
	DataTransfer *transfer;
	bool wasConnected;
	uint32_t numDisconnects;

	GCodeFileInfo fileInfo;
	FilePosition pauseFilePosition;
	PrintPausedReason pauseReason;
	bool reportPause;

	char codeBuffer[SpiCodeBufferSize];
	uint16_t rxPointer, txPointer, txLength;
	bool sendBufferUpdate;

	uint32_t iapWritePointer;

	String<FILENAME_MAX> requestedFileName;
	uint32_t requestedFileOffset, requestedFileLength;
	BinarySemaphore requestedFileSemaphore;
	char requestedFileChunk[MaxFileChunkSize];
	int32_t requestedFileDataLength;

	OutputStack *gcodeReply;
	void HandleGCodeReply(MessageType type, const char *reply);		// accessed by Platform
	void HandleGCodeReply(MessageType type, OutputBuffer *buffer);	// accessed by Platform

	void InvalidateBufferChannel(GCodeChannel channel);				// Invalidate every buffered G-code of the corresponding channel from the buffer ring
};

inline void LinuxInterface::SetPauseReason(FilePosition position, PrintPausedReason reason)
{
	pauseFilePosition = position;
	pauseReason = reason;
	reportPause = true;
}

#endif
