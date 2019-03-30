/*
 * LinuxInterface.h
 *
 *  Created on: 29 Mar 2019
 *      Authors: Christian
 */

#ifndef SRC_LINUX_LINUXINTERFACE_H_
#define SRC_LINUX_LINUXINTERFACE_H_

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

private:
	DataTransfer *transfer;
	BinaryGCodeBuffer *spiGCodeBuffer;

	GCodeBuffer *InitGCodeBuffer();			// accessed by GCodes, will be further expanded by an enum describing the requested channel
	void HandleGCodeReply(const char *reply);		// accessed by Platform
	void HandleGCodeReply(OutputBuffer *buffer);	// accessed by Platform

protected:
	OutputStack *gcodeReply;
};

#endif
