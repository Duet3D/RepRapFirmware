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

#include <GCodes/GCodeFileInfo.h>
#include <GCodes/GCodeChannel.h>
#include "LinuxMessageFormats.h"
#include <RTOSIface/RTOSIface.h>

class BinaryGCodeBuffer;
class StringRef;
class OutputBuffer;
class GCodeMachineState;
class HeightMap;

struct ExpressionValue;

class DataTransfer
{
public:
	DataTransfer() noexcept;
	void Init() noexcept;
	void SetSBCTask(TaskHandle handle) noexcept;
	void Diagnostics(MessageType mtype) noexcept;

	bool IsConnected() const noexcept;														// Check if the connection to DCS is live
	bool IsReady() noexcept;																// Returns true when data can be read
	void StartNextTransfer() noexcept;														// Kick off the next transfer
	bool LinuxHadReset() const noexcept;													// Check if the remote end reset

	size_t PacketsToRead() const noexcept;
	const PacketHeader *ReadPacket() noexcept;												// Attempt to read the next packet header or return null. Advances the read pointer to the next packet or the packet's data
	const char *ReadData(size_t packetLength) noexcept;										// Read the packet data and advance to the next packet (if any)
	void ReadGetObjectModel(size_t packetLength, const StringRef &key, const StringRef &flags) noexcept;		// Read an object model request
	void ReadPrintStartedInfo(size_t packetLength, const StringRef& filename, GCodeFileInfo &info) noexcept;	// Read info about the started file print
	PrintStoppedReason ReadPrintStoppedInfo() noexcept;										// Read info about why the print has been stopped
	GCodeChannel ReadMacroCompleteInfo(bool &error) noexcept;								// Read info about a completed macro file
	bool ReadHeightMap() noexcept;															// Read heightmap parameters
	GCodeChannel ReadCodeChannel() noexcept;												// Read a code channel
	void ReadAssignFilament(int& extruder, const StringRef& filamentName) noexcept;			// Read a request to assign the given filament to an extruder drive
	void ReadFileChunk(char *buffer, int32_t& dataLength, uint32_t& fileLength) noexcept;	// Read another chunk of a file
	GCodeChannel ReadEvaluateExpression(size_t packetLength, const StringRef& expression) noexcept;	// Read an expression request
	bool ReadMessage(MessageType& type, OutputBuffer *buf) noexcept;						// Read a request to output a message
	GCodeChannel ReadSetVariable(bool& createVariable, const StringRef& varName, const StringRef& expression) noexcept;	// Read a variable set request
	GCodeChannel ReadDeleteLocalVariable(const StringRef& varName) noexcept;				// Read a variable deletion request

	void ResendPacket(const PacketHeader *packet) noexcept;
	bool WriteObjectModel(OutputBuffer *data) noexcept;
	bool WriteCodeBufferUpdate(uint16_t bufferSpace) noexcept;
	bool WriteCodeReply(MessageType type, OutputBuffer *&response) noexcept;
	bool WriteMacroRequest(GCodeChannel channel, const char *filename, bool fromCode) noexcept;
	bool WriteAbortFileRequest(GCodeChannel channel, bool abortAll) noexcept;
	bool WriteMacroFileClosed(GCodeChannel channel) noexcept;
	bool WritePrintPaused(FilePosition position, PrintPausedReason reason) noexcept;
	bool WriteHeightMap() noexcept;
	bool WriteLocked(GCodeChannel channel) noexcept;
	bool WriteFileChunkRequest(const char *filename, uint32_t offset, uint32_t maxLength) noexcept;
	bool WriteEvaluationResult(const char *expression, const ExpressionValue& value) noexcept;
	bool WriteEvaluationError(const char *expression, const char *errorMessage) noexcept;
	bool WriteDoCode(GCodeChannel channel, const char *code, size_t length) noexcept;
	bool WriteWaitForAcknowledgement(GCodeChannel channel) noexcept;
	bool WriteMessageAcknowledged(GCodeChannel channel) noexcept;
	bool WriteSetVariableResult(const char *varName, const ExpressionValue& value) noexcept;
	bool WriteSetVariableError(const char *varName, const char *errorMessage) noexcept;

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
	unsigned int failedTransfers, checksumErrors;

	// Transfer buffers

#if SAME70
	// SAME70 has a write-back cache, so these must be in non-cached memory because we DMA to/from them.
	// See http://ww1.microchip.com/downloads/en/DeviceDoc/Managing-Cache-Coherency-on-Cortex-M7-Based-MCUs-DS90003195A.pdf
	// This in turn means that we must declare them static, so we can only have one DataTransfer instance
	static __nocache TransferHeader rxHeader;
	static __nocache TransferHeader txHeader;
	static __nocache uint32_t rxResponse;
	static __nocache uint32_t txResponse;
	alignas(4) static __nocache char rxBuffer[LinuxTransferBufferSize];
	alignas(4) static __nocache char txBuffer[LinuxTransferBufferSize];
#else
	// The other processors we support have write-through cache
	// Allocate the buffers in the object so that we can delete the object and recycle the memory if the SBC interface is not being used
	// Align the headers on 16-byte boundaries so that they span only one cache line
	alignas(16) TransferHeader rxHeader;
	alignas(16) TransferHeader txHeader;
	uint32_t rxResponse;
	uint32_t txResponse;
	char *rxBuffer;				// not allocated until we know we need it
	char *txBuffer;				// not allocated until we know we need it
#endif

	size_t rxPointer, txPointer;

	// Packet properties
	uint16_t packetId;

	void ExchangeHeader() noexcept;
	void ExchangeResponse(uint32_t response) noexcept;
	void ExchangeData() noexcept;
	void ResetTransfer(bool ownRequest) noexcept;
	uint32_t CalcCRC32(const char *buffer, size_t length) const noexcept;

	template<typename T> const T *ReadDataHeader() noexcept;

	// Always keep enough tx space to allow resend requests in case RRF runs out of
	// resources and cannot process an incoming request right away
	size_t FreeTxSpace() const noexcept { return LinuxTransferBufferSize - txPointer - rxHeader.numPackets * sizeof(PacketHeader); }

	bool CanWritePacket(size_t dataLength = 0) const noexcept;
	PacketHeader *WritePacketHeader(FirmwareRequest request, size_t dataLength = 0, uint16_t resendPacktId = 0) noexcept;
	void WriteData(const char *data, size_t length) noexcept;
	template<typename T> T *WriteDataHeader() noexcept;

	size_t AddPadding(size_t length) const noexcept;
};

inline bool DataTransfer::IsConnected() const noexcept
{
	return lastTransferTime != 0 && (millis() - lastTransferTime < SpiConnectionTimeout);
}

inline bool DataTransfer::LinuxHadReset() const noexcept
{
	uint16_t nextTransferNumber = lastTransferNumber + 1;
	return lastTransferNumber != 0 && (nextTransferNumber != rxHeader.sequenceNumber);
}

inline size_t DataTransfer::PacketsToRead() const noexcept
{
	return rxHeader.numPackets;
}

inline void DataTransfer::ResendPacket(const PacketHeader *packet) noexcept
{
	WritePacketHeader(FirmwareRequest::ResendPacket, 0, packet->id);
}

inline bool DataTransfer::CanWritePacket(size_t dataLength) const noexcept
{
	return FreeTxSpace() >= sizeof(PacketHeader) + dataLength;
}

inline size_t DataTransfer::AddPadding(size_t length) const noexcept
{
	size_t extraBytes = (length & 3);
	return (extraBytes == 0) ? length : length + 4 - extraBytes;
}
#endif	// HAS_LINUX_INTERFACE

#endif /* SRC_LINUX_DATATRANSFER_H_ */
