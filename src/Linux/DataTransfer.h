/*
 * DataTransfer.h
 *
 *  Created on: 29 Mar 2019
 *      Author: Christian
 */

#ifndef SRC_LINUX_DATATRANSFER_H_
#define SRC_LINUX_DATATRANSFER_H_

#include <cstddef>

#include "GCodes/GCodeFileInfo.h"
#include "MessageFormats.h"
#include "MessageType.h"

class BinaryGCodeBuffer;
class StringRef;
class OutputBuffer;
class GCodeMachineState;
class HeightMap;

class DataTransfer {
public:
	DataTransfer();
	void Init();
	void Diagnostics(MessageType mtype);

	bool IsConnected() const;								// Check if the connection to DCS is live
	volatile bool IsReady();								// Returns true when data can be read
	void StartNextTransfer();								// Kick off the next transfer
	bool LinuxHadReset() const;								// Check if the remote end reset

	size_t PacketsToRead() const;
	const PacketHeader *ReadPacket();						// Attempt to read the next packet header or return null. Advances the read pointer to the next packet or the packet's data
	const char *ReadData(size_t packetLength);				// Read the packet data and advance to the next packet (if any)
	uint8_t ReadGetObjectModel();							// Read an object model request
	void ReadPrintStartedInfo(size_t packetLength, StringRef& filename, GCodeFileInfo &info);	// Read info about the started file print
	PrintStoppedReason ReadPrintStoppedInfo();				// Read info about why the print has been stopped
	void ReadMacroCompleteInfo(CodeChannel& channel, bool &error);	// Read info about a completed macro file
	void ReadLockUnlockRequest(CodeChannel& channel);		// Read a lock/unlock request

	void ResendPacket(const PacketHeader *packet);
	bool WriteState(uint32_t busyChannels);
	bool WriteObjectModel(uint8_t module, OutputBuffer *data);
	bool WriteCodeReply(MessageType type, OutputBuffer *&response);
	bool WriteMacroRequest(CodeChannel channel, const char *filename, bool reportMissing);
	bool WriteAbortFileRequest(CodeChannel channel);
	bool WriteStackEvent(CodeChannel channel, GCodeMachineState& state);
	bool WritePrintPaused(FilePosition position, PrintPausedReason reason);
	bool WriteHeightMap();
	bool WriteLocked(CodeChannel channel);

	static void SpiInterrupt();

private:
	enum class SpiState
	{
		ExchangingHeader,
		ExchangingHeaderResponse,
		ExchangingData,
		ExchangingDataResponse,
		ProcessingData,
		Resetting
	} state;

	// Transfer properties
	uint32_t lastTransferTime;
	uint16_t currentTransferNumber, lastTransferNumber;
	TransferHeader rxHeader, txHeader;
	uint32_t rxResponse, txResponse;

	// Transfer buffers
	uint32_t rxBuffer32[LinuxTransferBufferSize / 4], txBuffer32[LinuxTransferBufferSize / 4];
	char * const rxBuffer = reinterpret_cast<char *>(rxBuffer32);
	char * const txBuffer = reinterpret_cast<char *>(txBuffer32);
	size_t rxPointer, txPointer;

	// Packet properties
	uint16_t packetId;

	void ExchangeHeader();
	void ExchangeResponse(uint32_t response);
	void ExchangeData();
	void ResetTransfer();
	uint16_t CRC16(const char *buffer, size_t length) const;

	template<typename T> const T *ReadDataHeader();

	// Always keep enough tx space to allow resend requests in case RRF runs out of
	// resources and cannot process an incoming request right away
	size_t FreeTxSpace() const { return LinuxTransferBufferSize - txPointer - rxHeader.numPackets * sizeof(PacketHeader); }

	bool CanWritePacket(size_t dataLength = 0) const;
	PacketHeader *WritePacketHeader(FirmwareRequest request, size_t dataLength = 0, uint16_t resendPacktId = 0);
	void WriteData(const char *data, size_t length);
	template<typename T> T *WriteDataHeader();

	size_t AddPadding(size_t length) const;
};

inline bool DataTransfer::IsConnected() const
{
	return rxHeader.sequenceNumber != 0;
}

inline bool DataTransfer::LinuxHadReset() const
{
	return lastTransferNumber + 1 != rxHeader.sequenceNumber;
}

inline size_t DataTransfer::PacketsToRead() const
{
	return rxHeader.numPackets;
}

inline void DataTransfer::ResendPacket(const PacketHeader *packet)
{
	WritePacketHeader(FirmwareRequest::ResendPacket, 0, packet->id);
}

inline bool DataTransfer::CanWritePacket(size_t dataLength) const
{
	return FreeTxSpace() >= sizeof(PacketHeader) + dataLength;
}

inline size_t DataTransfer::AddPadding(size_t length) const
{
	size_t padding = 4 - length % 4;
	return length + ((padding == 4) ? 0 : padding);
}

#endif /* SRC_LINUX_DATATRANSFER_H_ */
