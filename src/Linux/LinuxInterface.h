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

	void SetPauseReason(FilePosition position, PrintPausedReason reason);
	void ReportPause();

private:
	DataTransfer *transfer;

	GCodeFileInfo fileInfo;
	FilePosition pauseFilePosition;
	PrintPausedReason pauseReason;
	bool reportPause;

	OutputStack *gcodeReply;
	void HandleGCodeReply(MessageType type, const char *reply);		// accessed by Platform
	void HandleGCodeReply(MessageType type, OutputBuffer *buffer);	// accessed by Platform
};

inline void LinuxInterface::SetPauseReason(FilePosition position, PrintPausedReason reason)
{
	pauseFilePosition = position;
	pauseReason = reason;
}

inline void LinuxInterface::ReportPause()
{
	reportPause = true;
}

#endif
