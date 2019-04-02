/*
 * LinuxInterface.h
 *
 *  Created on: 29 Mar 2019
 *      Authors: Christian
 */

#ifndef SRC_LINUX_LINUXINTERFACE_H_
#define SRC_LINUX_LINUXINTERFACE_H_

#include "GCodes/GCodeFileInfo.h"
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

	void PrintPaused(FilePosition position);

private:
	DataTransfer *transfer;

	GCodeFileInfo fileInfo;
	OutputStack *gcodeReply;

	void HandleGCodeReply(MessageType type, const char *reply);		// accessed by Platform
	void HandleGCodeReply(MessageType type, OutputBuffer *buffer);	// accessed by Platform
};

#endif
