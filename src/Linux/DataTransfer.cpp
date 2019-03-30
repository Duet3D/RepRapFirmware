/*
 * DataTransfer.cpp
 *
 *  Created on: Mar 28, 2019
 *      Author: Christian
 */

#include <algorithm>

#include "DataTransfer.h"

#include <RepRapFirmware.h>
#include <OutputMemory.h>
#include <RepRap.h>

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

DataTransfer::DataTransfer() : state(SpiState::ExchangingHeader), sequenceNumber(0), txPointer(0)
{
	// Prepare TX header. These values will never change
	txHeader.formatCode = LinuxFormatCode;
	txHeader.protocolVersion = LinuxProtocolVersion;
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

void DataTransfer::ExchangeHeader()
{
	// Reset RX transfer header
	rxHeader.formatCode = InvalidFormatCode;
	rxHeader.numPackets = 0;			// TODO
	rxHeader.protocolVersion = 0;
	rxHeader.sequenceNumber = 0;
	rxHeader.dataLength = 0;
	rxHeader.checksumData = 0;
	rxHeader.checksumHeader = 0;

	// Reset TX transfer header
	txHeader.numPackets = 0;			// TODO
	txHeader.sequenceNumber = sequenceNumber++;
	txHeader.dataLength = 0;
	txHeader.checksumData = 0;
	txHeader.checksumHeader = 0;

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
	txPointer = 0;
	setup_spi(&rxBuffer, rxHeader.dataLength, &txBuffer, txHeader.dataLength);
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
				else if (rxHeader.dataLength > LinuxBufferSize)
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
	}
	return false;
}

OutputBuffer *DataTransfer::WriteCodeResponse(char channel, char type, OutputBuffer *response, bool isComplete)
{
	if (txPointer + sizeof(PacketHeader) + sizeof(CodeReplyHeader) > LinuxBufferSize)
	{
		// Not enough space to fit the data...
		return response;
	}

	PacketHeader packetHeader;
	//packetHeader.request =
	// TODO
	memcpy(txBuffer + txPointer, &packetHeader, sizeof(PacketHeader));
	txPointer += sizeof(PacketHeader);

	CodeReplyHeader replyHeader;
	replyHeader.channel = channel;
	replyHeader.flags = 0;	// TODO set flags
	memcpy(txBuffer + txPointer, &replyHeader, sizeof(CodeReplyHeader));
	txPointer += sizeof(CodeReplyHeader);

	if (response != nullptr)
	{
		do
		{
			size_t bytesToCopy = min<size_t>(LinuxBufferSize - txPointer, response->BytesLeft());
			memcpy(txBuffer + txPointer, response->UnreadData(), bytesToCopy);
			txPointer += bytesToCopy;
			response->Taken(bytesToCopy);

			if (response->BytesLeft() == 0)
			{
				response = OutputBuffer::Release(response);
			}
		}
		while (txPointer < LinuxBufferSize && response != nullptr);

		if (response != nullptr)
		{
			// set push flag
		}
	}

	return response;
}

#endif
