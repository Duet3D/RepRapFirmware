/*
 * LinuxInterface.h
 *
 *  Created on: 29 Mar 2019
 *      Authors: Christian
 */

#ifndef SRC_LINUX_LINUXINTERFACE_H_
#define SRC_LINUX_LINUXINTERFACE_H_

#include "GCodes/GCodeFileInfo.h"
#include "MessageFormats.h"
#include "MessageType.h"

class Platform;

class DataTransfer;
class GCodeBuffer;

class OutputBuffer;
class OutputStack;

// G-Code input class for an SPI channel
class LinuxInterface
{
public:
	friend class Platform;

	LinuxInterface();
	void Init();
	void Spin();
	void Diagnostics(MessageType mtype);

	bool FillBuffer(GCodeBuffer &gb);			// Try to fill up the G-code buffer with the next available G=code

	void SetPauseReason(FilePosition position, PrintPausedReason reason);	// Notify Linux that the print has been paused

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
