/*
 * LinuxComm.h
 *
 *  Created on: 16 Jul 2018
 *      Authors: Christian
 */

#ifndef LINUXCOMM_H
#define LINUXCOMM_H

#include <cstdint>
#include "LinuxMessageFormats.h"

class GCodes;
class Platform;
class NetworkGCodeInput;

class OutputBuffer;
class OutputStack;

// G-Code input class for an SPI channel
class LinuxComm
{
public:
	friend class GCodes;
	friend class Platform;

	LinuxComm();
	void Init();
	void Spin();

	void SpiInterrupt();

private:
	void SetGCodeInput(NetworkGCodeInput *input);		// accessed by GCodes
	void HandleGCodeReply(const char *reply);			// accessed by Platform
	void HandleGCodeReply(OutputBuffer *buffer);		// accessed by Platform

protected:
	enum class SpiState
	{
		ReceivingHeader,
		ReceivingData,
		SendingHeader,
		SendingData
	} state;

	size_t bytesProcessed;

	NetworkGCodeInput *gcodeInput;
	OutputStack *gcodeReply;

	void ResetState();
	void SendResponse(int32_t responseOrBytesToWrite);
};

#endif
