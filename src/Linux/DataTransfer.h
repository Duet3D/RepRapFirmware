/*
 * DataTransfer.h
 *
 *  Created on: Mar 29 2019
 *      Author: Christian
 */

#ifndef SRC_LINUX_DATATRANSFER_H_
#define SRC_LINUX_DATATRANSFER_H_

#include <cstddef>

#include "MessageFormats.h"

class OutputBuffer;


class DataTransfer {
public:
	DataTransfer();
	void Init();

	volatile bool IsReady();
	OutputBuffer *WriteCodeResponse(char channel, char type, OutputBuffer *response, bool isComplete);
	// TODO add read/write functions here

	static void SpiInterrupt();

private:
	enum class SpiState
	{
		ExchangingHeader,
		ExchangingHeaderResponse,
		ExchangingData,
		ExchangingDataResponse,
		ProcessingData,

		// This must remain the last entry as long as checksums are not implemented!
		ResettingState
	} state;

	uint32_t sequenceNumber;
	TransferHeader rxHeader, txHeader;
	int32_t rxResponse, txResponse;

	uint32_t rxBuffer[LinuxBufferSize / 4], txBuffer[LinuxBufferSize / 4];
	size_t txPointer;

	void ExchangeHeader();
	void ExchangeResponse(int32_t code);
	void ExchangeData();
	void ForceReset();
};

#endif /* SRC_LINUX_DATATRANSFER_H_ */
