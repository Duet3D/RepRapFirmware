/*
 * DataTransfer.h
 *
 *  Created on: 29 Mar 2019
 *      Author: Christian
 */

#ifndef SRC_LINUX_DATATRANSFER_H_
#define SRC_LINUX_DATATRANSFER_H_

#include "RepRapFirmware.h"

#if HAS_LINUX_INTERFACE

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
	bool IsReady();											// Returns true when data can be read
	void StartNextTransfer();								// Kick off the next transfer
	bool LinuxHadReset() const;								// Check if the remote end reset

	size_t PacketsToRead() const;
	const PacketHeader *ReadPacket();						// Attempt to read the next packet header or return null. Advances the read pointer to the next packet or the packet's data
	const char *ReadData(size_t packetLength);				// Read the packet data and advance to the next packet (if any)
	uint8_t ReadGetObjectModel();							// Read an object model request
	void ReadPrintStartedInfo(size_t packetLength, StringRef& filename, GCodeFileInfo &info);	// Read info about the started file print
	PrintStoppedReason ReadPrintStoppedInfo();				// Read info about why the print has been stopped
	void ReadMacroCompleteInfo(GCodeChannel& channel, bool &error);	// Read info about a completed macro file
	void ReadHeightMap();									// Read heightmap parameters
	void ReadLockUnlockRequest(GCodeChannel& channel);		// Read a lock/unlock request
	void ReadAssignFilament(int& extruder, StringRef& filamentName);	// Read a request to assign the given filament to an extruder drive
	void ReadFileChunk(char *buffer, int32_t& dataLength, uint32_t& fileLength);	// Read another chunk of a file

	void ResendPacket(const PacketHeader *packet);
	bool WriteObjectModel(uint8_t module, OutputBuffer *data);
	bool WriteCodeBufferUpdate(uint16_t bufferSpace);
	bool WriteCodeReply(MessageType type, OutputBuffer *&response);
	bool WriteMacroRequest(GCodeChannel channel, const char *filename, bool reportMissing, bool fromBinaryCode);
	bool WriteAbortFileRequest(GCodeChannel channel, bool abortAll);
	bool WriteStackEvent(GCodeChannel channel, GCodeMachineState& state);
	bool WritePrintPaused(FilePosition position, PrintPausedReason reason);
	bool WriteHeightMap();
	bool WriteLocked(GCodeChannel channel);
	bool WriteFileChunkRequest(const char *filename, uint32_t offset, uint32_t maxLength);

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
	uint16_t lastTransferNumber;
	unsigned int failedTransfers;

	// Transfer buffers
	// These must be in non-cached memory because we DMA to/from them, see http://ww1.microchip.com/downloads/en/DeviceDoc/Managing-Cache-Coherency-on-Cortex-M7-Based-MCUs-DS90003195A.pdf
	// This in turn means that we must declare them static, so we can only have one DataTransfer instance
	static __nocache TransferHeader rxHeader;
	static __nocache TransferHeader txHeader;
	static __nocache uint32_t rxResponse;
	static __nocache uint32_t txResponse;
	static __nocache uint32_t rxBuffer32[LinuxTransferBufferSize / 4];
	static __nocache uint32_t txBuffer32[LinuxTransferBufferSize / 4];

	static inline char * rxBuffer() { return reinterpret_cast<char *>(rxBuffer32); }
	static inline char * txBuffer() { return reinterpret_cast<char *>(txBuffer32); }

	size_t rxPointer, txPointer;

	// Packet properties
	uint16_t packetId;

	void ExchangeHeader();
	void ExchangeResponse(uint32_t response);
	void ExchangeData();
	void ResetTransfer(bool ownRequest);
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
	return lastTransferTime != 0 && (millis() - lastTransferTime < SpiConnectionTimeout);
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

#endif	// HAS_LINUX_INTERFACE

#endif /* SRC_LINUX_DATATRANSFER_H_ */
