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

class GCodes;
class Platform;

class DataTransfer;
class GCodeBuffer;
class BinaryGCodeBuffer;

class OutputBuffer;
class OutputStack;

// G-Code input class for an SPI channel
class LinuxInterface
{
public:
	friend class GCodes;
	friend class Platform;

	LinuxInterface();
	void Init();
	void Spin();

	bool RequestMacroFile(GCodeBuffer& gb, const char *filename, bool reportMissing);

private:
	DataTransfer *transfer;

	BinaryGCodeBuffer *spiGCodeBuffer;

	GCodeFileInfo fileInfo;
	OutputStack *gcodeReply;
	GCodeBuffer *InitGCodeBuffer();									// accessed by GCodes, will be further enhanced
	void HandleGCodeReply(MessageType type, const char *reply);		// accessed by Platform
	void HandleGCodeReply(MessageType type, OutputBuffer *buffer);	// accessed by Platform
};

#endif
