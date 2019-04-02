/*
 * DataTransfer.cpp
 *
 *  Created on: Mar 28, 2019
 *      Author: Christian
 */

#include <algorithm>

#include "DataTransfer.h"

#include "RepRapFirmware.h"
#include "Movement/Move.h"
#include "Movement/BedProbing/Grid.h"
#include "OutputMemory.h"
#include "RepRap.h"

#if HAS_LINUX_INTERFACE

# include "xdmac/xdmac.h"

# define LINUX_SPI				SPI1
# define LINUX_SPI_ID			ID_SPI1
# define LINUX_SPI_IRQn			SPI1_IRQn
# define LINUX_SPI_HANDLER		SPI1_Handler

// XDMAC hardware, see datasheet
const uint32_t LINUX_XDMAC_TX_CH_NUM = 3;
const uint32_t LINUX_XDMAC_RX_CH_NUM = 4;

static xdmac_channel_config_t xdmac_tx_cfg, xdmac_rx_cfg;

volatile bool dataReceived = false;

void setup_spi(void *inBuffer, size_t bytesToRead, const void *outBuffer, size_t bytesToWrite)
{
	// Reset SPI
	spi_reset(LINUX_SPI);
	spi_set_slave_mode(LINUX_SPI);
	spi_disable_mode_fault_detect(LINUX_SPI);
	spi_set_peripheral_chip_select_value(LINUX_SPI, spi_get_pcs(0));
	spi_set_clock_polarity(LINUX_SPI, 0, 0);
	spi_set_clock_phase(LINUX_SPI, 0, 1);
	spi_set_bits_per_transfer(LINUX_SPI, 0, SPI_CSR_BITS_8_BIT);

	// Initialize channel config for transmitter
	xdmac_tx_cfg.mbr_ubc = bytesToWrite;
	xdmac_tx_cfg.mbr_sa = (uint32_t)outBuffer;
	xdmac_tx_cfg.mbr_da = (uint32_t)&(LINUX_SPI->SPI_TDR);
	xdmac_tx_cfg.mbr_cfg = XDMAC_CC_TYPE_PER_TRAN |
		XDMAC_CC_MBSIZE_SINGLE |
		XDMAC_CC_DSYNC_MEM2PER |
		XDMAC_CC_CSIZE_CHK_1 |
		XDMAC_CC_DWIDTH_BYTE |
		XDMAC_CC_SIF_AHB_IF0 |
		XDMAC_CC_DIF_AHB_IF1 |
		XDMAC_CC_SAM_INCREMENTED_AM |
		XDMAC_CC_DAM_FIXED_AM |
		XDMAC_CC_PERID(LINUX_XDMAC_TX_CH_NUM);
	xdmac_tx_cfg.mbr_bc = 0;
	xdmac_tx_cfg.mbr_ds = 0;
	xdmac_tx_cfg.mbr_sus = 0;
	xdmac_tx_cfg.mbr_dus = 0;
	xdmac_configure_transfer(XDMAC, DmacChanLinuxTx, &xdmac_tx_cfg);

	xdmac_channel_set_descriptor_control(XDMAC, DmacChanLinuxTx, 0);
	xdmac_channel_enable(XDMAC, DmacChanLinuxTx);
	xdmac_disable_interrupt(XDMAC, DmacChanLinuxTx);

	// Initialize channel config for receiver
	xdmac_rx_cfg.mbr_ubc = bytesToRead;
	xdmac_rx_cfg.mbr_da = (uint32_t)inBuffer;
	xdmac_rx_cfg.mbr_sa = (uint32_t)&(LINUX_SPI->SPI_RDR);
	xdmac_rx_cfg.mbr_cfg = XDMAC_CC_TYPE_PER_TRAN |
		XDMAC_CC_MBSIZE_SINGLE |
		XDMAC_CC_DSYNC_PER2MEM |
		XDMAC_CC_CSIZE_CHK_1 |
		XDMAC_CC_DWIDTH_BYTE|
		XDMAC_CC_SIF_AHB_IF1 |
		XDMAC_CC_DIF_AHB_IF0 |
		XDMAC_CC_SAM_FIXED_AM |
		XDMAC_CC_DAM_INCREMENTED_AM |
		XDMAC_CC_PERID(LINUX_XDMAC_RX_CH_NUM);
	xdmac_rx_cfg.mbr_bc = 0;
	xdmac_tx_cfg.mbr_ds = 0;
	xdmac_rx_cfg.mbr_sus = 0;
	xdmac_rx_cfg.mbr_dus = 0;
	xdmac_configure_transfer(XDMAC, DmacChanLinuxRx, &xdmac_rx_cfg);

	xdmac_channel_set_descriptor_control(XDMAC, DmacChanLinuxRx, 0);
	xdmac_channel_enable(XDMAC, DmacChanLinuxRx);
	xdmac_disable_interrupt(XDMAC, DmacChanLinuxRx);

	// Enable SPI and notify the RaspPi we are ready
	spi_enable(LINUX_SPI);
	digitalWrite(SamTfrReadyPin, true);
	dataReceived = false;

	// Enable end-of-transfer interrupt
	(void)LINUX_SPI->SPI_SR;						// clear any pending interrupt
	LINUX_SPI->SPI_IER = SPI_IER_NSSR;				// enable the NSS rising interrupt

	NVIC_SetPriority(LINUX_SPI_IRQn, NvicPrioritySpi);
	NVIC_EnableIRQ(LINUX_SPI_IRQn);
}

extern "C" void LINUX_SPI_HANDLER(void)
{
	const uint32_t status = LINUX_SPI->SPI_SR;							// read status and clear interrupt
	LINUX_SPI->SPI_IDR = SPI_IER_NSSR;									// disable the interrupt
	if ((status & SPI_SR_NSSR) != 0)
	{
		// Data has been transferred, disable XDMAC channels
		xdmac_channel_disable(XDMAC, DmacChanLinuxRx);
		xdmac_channel_disable(XDMAC, DmacChanLinuxTx);

		// Disable SPI and indicate that no more data may be exchanged
		spi_disable(LINUX_SPI);
		digitalWrite(SamTfrReadyPin, false);
		dataReceived = true;
	}
}

/*-----------------------------------------------------------------------------------*/

DataTransfer::DataTransfer() : state(SpiState::ExchangingHeader), lastTransferTime(0), sequenceNumber(0),
	rxPointer(0), txPointer(0), packetId(0)
{
	// Prepare TX header
	txHeader.formatCode = LinuxFormatCode;
	txHeader.protocolVersion = LinuxProtocolVersion;
	txHeader.numPackets = 0;
}

void DataTransfer::Init() {
	// Initialise transfer ready pin
	pinMode(SamTfrReadyPin, PinMode::OUTPUT_LOW);

	// Initialise SPI
	spi_enable_clock(LINUX_SPI);
	spi_disable(LINUX_SPI);

	// Kick off first transfer
	ExchangeHeader();
}

const PacketHeader *DataTransfer::ReadPacket()
{
	if (rxPointer >= rxHeader.dataLength)
	{
		return nullptr;
	}

	PacketHeader *header = reinterpret_cast<PacketHeader*>(rxBuffer + rxPointer);
	rxPointer += sizeof(PacketHeader);
	return header;
}

const char *DataTransfer::ReadData(size_t packetLength)
{
	const char *data = rxBuffer + txPointer;
	rxPointer += AddPadding(packetLength);
	return data;
}

void DataTransfer::ReadPrintStartedInfo(size_t packetLength, StringRef& filename, GCodeFileInfo& info)
{
	// Read header
	PrintStartedInfo *header = ReadDataHeader<PrintStartedInfo>();
	info.fileSize = header->fileSize;
	info.lastModifiedTime = header->lastModifiedTime;
	info.layerHeight = header->layerHeight;
	info.firstLayerHeight = header->firstLayerHeight;
	info.objectHeight = header->objectHeight;
	info.printTime = header->printTime;
	info.simulatedTime = header->simulatedTime;
	info.numFilaments = header->numFilaments;

	// Read filaments
	const char *data = ReadData(packetLength - sizeof(PrintStartedInfo));
	size_t filamentsSize = info.numFilaments * sizeof(float);
	memcpy(info.filamentNeeded, data, filamentsSize);
	data += filamentsSize;

	// Read file name
	filename.copy(data, header->filenameLength);
	data += header->filenameLength;

	// Read generated by
	info.generatedBy.copy(data, header->generatedByLength);
}

PrintStoppedReason DataTransfer::ReadPrintStoppedInfo()
{
	PrintStoppedInfo *header = ReadDataHeader<PrintStoppedInfo>();
	return header->reason;
}

void DataTransfer::ReadMacroCompleteInfo(CodeChannel& channel, bool &error)
{
	MacroCompleteInfo *header = ReadDataHeader<MacroCompleteInfo>();
	channel = header->channel;
	error = header->error;
}

void DataTransfer::ExchangeHeader()
{
	// Reset RX transfer header
	rxHeader.formatCode = InvalidFormatCode;
	rxHeader.numPackets = 0;
	rxHeader.protocolVersion = 0;
	rxHeader.sequenceNumber = 0;
	rxHeader.dataLength = 0;
	rxHeader.checksumData = 0;
	rxHeader.checksumHeader = 0;

	// Reset TX transfer header
	txHeader.sequenceNumber = sequenceNumber++;
	txHeader.dataLength = 0;
	txHeader.checksumData = 0;				// TOOD
	txHeader.checksumHeader = 0;			// TODO

	// Set up SPI transfer
	setup_spi(&rxHeader, sizeof(TransferHeader), &txHeader, sizeof(TransferHeader));
	state = SpiState::ExchangingHeader;
}

void DataTransfer::ExchangeResponse(int32_t response)
{
	txResponse = response;
	setup_spi(&rxResponse, sizeof(int32_t), &txResponse, sizeof(int32_t));
}

void DataTransfer::ExchangeData()
{
	setup_spi(rxBuffer, rxHeader.dataLength, txBuffer, txHeader.dataLength);
	state = SpiState::ExchangingData;
}

void DataTransfer::ForceReset()
{
	ExchangeResponse(TransferResponse::RequestStateReset);
	state = SpiState::ResettingState;
}

volatile bool DataTransfer::IsReady()
{
	// TODO implement timeout here resetting the state if no transfer happens within X ms
	if (dataReceived)
	{
		switch (state)
		{
		case SpiState::ExchangingHeader:
			if (rxHeader.formatCode != LinuxFormatCode)
			{
				ExchangeResponse(TransferResponse::BadFormat);
				break;
			}
			if (rxHeader.protocolVersion != LinuxProtocolVersion)
			{
				ExchangeResponse(TransferResponse::BadProtocolVersion);
				break;
			}
#if 0
			if (rxHeader.checksum != CalcChecksum(rxHeader))
			{
				ExchangeResponse(TransferResponse::BadChecksum);
				break;
			}
#endif

			ExchangeResponse(TransferResponse::Success);
			state = SpiState::ExchangingHeaderResponse;
			break;

		case SpiState::ExchangingHeaderResponse:
			switch (rxResponse)
			{
			case TransferResponse::Success:
				if (rxHeader.dataLength == 0 && txHeader.dataLength == 0)
				{
					// No data to send or receive, perform another header exchange
					ExchangeHeader();
				}
				else if (rxHeader.dataLength > LinuxTransferBufferSize)
				{
					// Probably garbage - force reset. This will become obsolete when CRC16 checksums are implemented
					ForceReset();
				}
				else
				{
					// Everything OK, perform data transfer
					ExchangeData();
				}
				break;

			case TransferResponse::BadChecksum:
			case TransferResponse::BadFormat:
			case TransferResponse::BadProtocolVersion:
				// This is entirely handled by DSF, no need to do anything here except to send the header again
				ExchangeHeader();
				break;

			default:
				// Something unexpected happened
				ExchangeResponse(TransferResponse::RequestStateReset);
				state = SpiState::ResettingState;
				break;
			}
			break;

		case SpiState::ExchangingData:
#if 0
			if (CalcChecksum(rxBuffer) != rxHeader.checksumData)
			{
				ExchangeResponse(TransferResponse::BadChecksum);
				break;
			}
#endif

			ExchangeResponse(TransferResponse::Success);
			state = SpiState::ExchangingDataResponse;
			break;

		case SpiState::ExchangingDataResponse:
			if (rxResponse == TransferResponse::Success)
			{
				txPointer = 0;
				txHeader.numPackets = 0;

				state = SpiState::ProcessingData;
				return true;
			}

			if (rxResponse == TransferResponse::BadChecksum)
			{
				ExchangeData();
			}
			else
			{
				ExchangeResponse(TransferResponse::RequestStateReset);
				state = SpiState::ResettingState;
			}
			break;

		case SpiState::ProcessingData:
			// Should never get here. If we do, there is something wrong in the DataProvider
			INTERNAL_ERROR;
			break;

		case SpiState::ResettingState:
			// Something went really wrong so we need to return back to the initial state.
			// Some more garbage may be received until this request is acknowledged so keep on
			// sending reset requests in the hope the other end will acknowledge it eventually
			if (rxResponse != TransferResponse::RequestStateReset)
			{
				ExchangeResponse(TransferResponse::RequestStateReset);
				state = SpiState::ResettingState;
			}
			else
			{
				// Reset complete, attempt to exchange a transfer header once again
				ExchangeHeader();
			}
			break;
		}
		lastTransferTime = millis();
	}
	else if (state != SpiState::ExchangingHeader && millis() - lastTransferTime > MaxTransferTime)
	{
		// Reset failed transfers automatically after a certain time
		state = SpiState::ExchangingHeader;
	}
	return false;
}

template<typename T> T *DataTransfer::ReadDataHeader()
{
	rxPointer += sizeof(T);
	return reinterpret_cast<T*>(rxBuffer + rxPointer);
}

bool DataTransfer::WriteState(uint32_t busyBuffers)
{
	if (!CanWritePacket(sizeof(StateResponse)))
	{
		return false;
	}
	(void)WritePacketHeader(FirmwareRequest::ReportState, sizeof(StateResponse));

	StateResponse *state = WriteDataHeader<StateResponse>();
	state->busyBuffers = busyBuffers;
	return true;
}

OutputBuffer *DataTransfer::WriteCodeResponse(CodeChannel channel, MessageType type, OutputBuffer *response, bool isComplete)
{
	// Try to write the packet header
	if (!CanWritePacket(sizeof(CodeReplyHeader) + 4))
	{
		// Not enough space left
		return response;
	}
	PacketHeader *header = WritePacketHeader(FirmwareRequest::CodeReply);

	// Write code reply header
	CodeReplyHeader *replyHeader = WriteDataHeader<CodeReplyHeader>();
	replyHeader->channel = channel;
	replyHeader->flags = isComplete ? CodeReplyFlags::CodeComplete : CodeReplyFlags::NoFlags;
	if ((type & MessageType::ErrorMessageFlag) != 0)
	{
		replyHeader->flags = (CodeReplyFlags)((uint8_t)replyHeader->flags | (uint8_t)CodeReplyFlags::Error);
	}
	else if ((type & MessageType::WarningMessageFlag) != 0)
	{
		replyHeader->flags = (CodeReplyFlags)((uint8_t)replyHeader->flags | (uint8_t)CodeReplyFlags::Warning);
	}
	replyHeader->padding = 0;

	// Write payload
	size_t bytesWritten = sizeof(CodeReplyHeader);
	if (response != nullptr)
	{
		do
		{
			size_t bytesToCopy = min<size_t>(LinuxTransferBufferSize - txPointer, response->BytesLeft());
			WriteData(response->UnreadData(), bytesToCopy);
			bytesWritten += bytesToCopy;

			response->Taken(bytesToCopy);
			if (response->BytesLeft() == 0)
			{
				response = OutputBuffer::Release(response);
			}
		}
		while (txPointer < LinuxTransferBufferSize && response != nullptr);

		if (response != nullptr)
		{
			// There is more data to come...
			replyHeader->flags = (CodeReplyFlags)((uint8_t)replyHeader->flags | (uint8_t)CodeReplyFlags::Push);
		}
	}

	// Finish packet and return what is left of the output buffer
	header->length = bytesWritten;
	return response;
}

bool DataTransfer::WriteMacroRequest(CodeChannel channel, const char *filename, bool reportMissing)
{
	size_t filenameLength = strlen(filename);
	if (!CanWritePacket(sizeof(MacroRequest) + filenameLength))
	{
		return false;
	}

	// Write packet header
	WritePacketHeader(FirmwareRequest::ExecuteMacro, filenameLength);

	// Write header
	MacroRequest *header = WriteDataHeader<MacroRequest>();
	header->channel = channel;
	header->reportMissing = reportMissing;
	header->padding = 0;

	// Write filename
	WriteData(filename, filenameLength);
	return true;
}

bool DataTransfer::WriteAbortFileRequest(CodeChannel channel)
{
	if (!CanWritePacket(sizeof(AbortFileRequest)))
	{
		return false;
	}

	// Write packet header
	WritePacketHeader(FirmwareRequest::AbortFile, sizeof(AbortFileRequest));

	// Write header
	AbortFileRequest *header = WriteDataHeader<AbortFileRequest>();
	header->channel = channel;
	header->paddingA = 0;
	header->paddingB = 0;
	return true;
}

bool DataTransfer::WriteObjectModel(OutputBuffer *data)
{
	if (!CanWritePacket(data->Length()))
	{
		return false;
	}

	(void)WritePacketHeader(FirmwareRequest::ObjectModel, data->Length());
	while (data != nullptr)
	{
		WriteData(data->UnreadData(), data->BytesLeft());
		data = OutputBuffer::Release(data);
	}
	return true;
}

bool DataTransfer::WriteHeightMap()
{
	const GridDefinition& grid = reprap.GetMove().GetGrid();
	size_t numPoints = reprap.GetMove().AccessHeightMap().UsingHeightMap() ? grid.NumPoints() : 0;
	size_t bytesToWrite = sizeof(HeightMapHeader) + numPoints * sizeof(float);
	if (!CanWritePacket(bytesToWrite))
	{
		return false;
	}

	// Write packet header
	(void)WritePacketHeader(FirmwareRequest::HeightMap, bytesToWrite);

	// Write heightmap header
	HeightMapHeader *header = WriteDataHeader<HeightMapHeader>();
	header->xMin = grid.xMin;
	header->xMax = grid.xMax;
	header->xSpacing = grid.xSpacing;
	header->yMin = grid.yMin;
	header->yMax = grid.yMax;
	header->ySpacing = grid.ySpacing;
	header->radius = grid.radius;
	header->numPoints = numPoints;

	// Write Z points
	if (numPoints != 0)
	{
		float *zPoints = reinterpret_cast<float*>(txBuffer + txPointer + sizeof(HeightMapHeader));
		reprap.GetMove().SaveHeightMapToArray(zPoints);
	}
	return true;
}

PacketHeader *DataTransfer::WritePacketHeader(FirmwareRequest request, size_t dataLength, uint16_t resendPacketId)
{
	// Make sure to stay aligned if the last packet ended with a string
	txPointer = AddPadding(txPointer);

	// Write the next packet data
	PacketHeader *header = reinterpret_cast<PacketHeader*>(txBuffer + txPointer);
	header->request = static_cast<uint16_t>(request);
	header->id = packetId++;
	header->length = dataLength;
	header->resendPacketId = resendPacketId;
	txPointer += sizeof(PacketHeader);
	return header;
}

void DataTransfer::WriteData(const char *data, size_t length)
{
	memcpy(txBuffer + txPointer, data, length);
	txPointer += length;
}

template<typename T> T *DataTransfer::WriteDataHeader()
{
	txPointer += sizeof(T);
	return reinterpret_cast<T*>(txBuffer + txPointer);
}

#endif
