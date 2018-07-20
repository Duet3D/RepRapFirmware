/*
 * LinuxComm.cpp
 *
 *  Created on: 16 Jul 2018
 *      Author: Christian
 */


#include <LinuxComm.h>
#include <LinuxMessageFormats.h>

#include "Platform.h"
#include "RepRap.h"
#include "RepRapFirmware.h"
#include "GCodes/GCodeInput.h"

#if HAS_LINUX_INTERFACE

#if !defined(SAME70_TEST_BOARD)
# error Unsupported board
#endif

static_assert(HAS_WIFI_NETWORKING == 0, "Cannot use ESP WiFi and Linux SPI comms because there is only one SPI channel available");

# include "xdmac/xdmac.h"

# define LINUX_SPI				SPI0
# define LINUX_SPI_ID			ID_SPI0
# define LINUX_SPI_IRQn			SPI0_IRQn
# define LINUX_SPI_HANDLER		SPI0_Handler

// Our choice of XDMA channels to use
const uint32_t LINUX_XDMAC_TX_CH = 1;
const uint32_t LINUX_XDMAC_RX_CH = 2;

// XDMAC hardware, see datasheet
const uint32_t LINUX_XDMAC_TX_CH_NUM = 1;
const uint32_t LINUX_XDMAC_RX_CH_NUM = 2;

static xdmac_channel_config_t xdmac_tx_cfg, xdmac_rx_cfg;

static MessageHeaderLinuxToSam rxHeader;
static uint8_t rxBuffer[MaxDataLength];

static MessageHeaderSamToLinux txHeader;
static uint8_t txBuffer[MaxDataLength];
static size_t txBufferSize;

static MessageHeaderSamToLinux busyHeader;

static volatile bool transferPending = false;

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

	// Don't enable the following XDMAC interrupts...
	const uint32_t xdmaint = (XDMAC_CIE_BIE |
		XDMAC_CIE_DIE   |
		XDMAC_CIE_FIE   |
		XDMAC_CIE_RBIE  |
		XDMAC_CIE_WBIE  |
		XDMAC_CIE_ROIE);

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
	xdmac_configure_transfer(XDMAC, LINUX_XDMAC_TX_CH, &xdmac_tx_cfg);

	xdmac_channel_set_descriptor_control(XDMAC, LINUX_XDMAC_TX_CH, 0);
	xdmac_channel_disable_interrupt(XDMAC, LINUX_XDMAC_TX_CH, xdmaint);
	xdmac_channel_enable(XDMAC, LINUX_XDMAC_TX_CH);
	xdmac_disable_interrupt(XDMAC, LINUX_XDMAC_TX_CH);

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
	xdmac_configure_transfer(XDMAC, LINUX_XDMAC_RX_CH, &xdmac_rx_cfg);

	xdmac_channel_set_descriptor_control(XDMAC, LINUX_XDMAC_RX_CH, 0);
	xdmac_channel_disable_interrupt(XDMAC, LINUX_XDMAC_RX_CH, xdmaint);
	xdmac_channel_enable(XDMAC, LINUX_XDMAC_RX_CH);
	xdmac_disable_interrupt(XDMAC, LINUX_XDMAC_RX_CH);

	// Enable SPI again
	transferPending = true;
	spi_enable(LINUX_SPI);

	// Enable end-of-transfer interrupt
	(void)LINUX_SPI->SPI_SR;					// clear any pending interrupt
	LINUX_SPI->SPI_IER = SPI_IER_NSSR;			// enable the NSS rising interrupt

	NVIC_SetPriority(LINUX_SPI_IRQn, NvicPrioritySpi);
	NVIC_EnableIRQ(LINUX_SPI_IRQn);
}


/*-----------------------------------------------------------------------------------*/
// Main class to make the board act as an SPI slave

LinuxComm::LinuxComm() : state(SpiState::ReceivingHeader), gcodeInput(nullptr), gcodeReply(new OutputStack())
{
	busyHeader.formatVersion = LinuxFormatVersion;
	busyHeader.response = ResponseBusy;
}

void LinuxComm::Init()
{
	// Initialise XDMAC
	pmc_enable_periph_clk(ID_XDMAC);

	// Initialise SPI in slave mode (SPI_MODE0)
	spi_enable_clock(LINUX_SPI);
	spi_disable(LINUX_SPI);

	// Prepare for first SPI transfer
	ResetState();
}

void LinuxComm::Spin()
{
	if (transferPending)
	{
		// Still waiting for an SPI transfer to finish
		// TODO: Add a timeout here
		return;
	}

	switch (state)
	{
	case SpiState::ReceivingHeader:
		//debugPrintf("received header, fmt %x request %x (%d bytes payload)\n", rxHeader.formatVersion, (int)rxHeader.request, rxHeader.dataLength);
		if (rxHeader.formatVersion == LinuxFormatVersion)
		{
			if (rxHeader.dataLength < MaxDataLength)
			{
				if (rxHeader.dataLength > 0)
				{
					// Start receiving the command data
					setup_spi(rxBuffer, rxHeader.dataLength, &busyHeader, sizeof(MessageHeaderSamToLinux));
				}
				bytesProcessed = 0;
				state = SpiState::ReceivingData;
			}
			else
			{
				reprap.GetPlatform().Message(ErrorMessage, "Bad data length of received SPI header\n");
				SendResponse(ResponseBadDataLength);
			}
		}
		else
		{
			reprap.GetPlatform().Message(ErrorMessage, "Bad format of received SPI header\n");
			SendResponse(ResponseBadHeaderVersion);
		}
		break;

	case SpiState::ReceivingData:
		//debugPrintf("received data, cmd %d, bytesProcessed %u of %u\n", (int)rxHeader.request, bytesProcessed, rxHeader.dataLength);
		switch (rxHeader.request)
		{
		case LinuxRequest::nullCommand:
			ResetState();
			break;

		case LinuxRequest::doGCode:
			for(size_t i = bytesProcessed; i <  rxHeader.dataLength; i++)
			{
				if (gcodeInput->BufferSpaceLeft() == 0)
				{
					// Stop before we encounter a buffer overflow
					break;
				}
				gcodeInput->Put(SpiMessage, rxBuffer[i]);
				bytesProcessed++;
			}

			if (bytesProcessed == rxHeader.dataLength)
			{
				// G-code has been fully fed into the input class
				ResetState();
			}
			else
			{
				txHeader.response = ResponseBusy;
				setup_spi(nullptr, 0, &txHeader, sizeof(MessageHeaderSamToLinux));
				transferPending = false;
			}
			break;

		case LinuxRequest::getGCodeReply:
			txBufferSize = 0;
			do
			{
				OutputBuffer *buffer = gcodeReply->GetFirstItem();
				if (buffer == nullptr)
				{
					break;
				}

				size_t bytesToCopy = min<size_t>(MaxDataLength - txBufferSize, buffer->BytesLeft());
				memcpy(txBuffer + txBufferSize, buffer->UnreadData(), bytesToCopy);
				txBufferSize += bytesToCopy;
				buffer->Taken(bytesToCopy);

				if (buffer->BytesLeft() == 0)
				{
					buffer = OutputBuffer::Release(buffer);
					gcodeReply->SetFirstItem(buffer);
				}
			}
			while (txBufferSize < MaxDataLength);

			SendResponse(txBufferSize);
			break;

		case LinuxRequest::emergencyStop:
			reprap.EmergencyStop();
			ResetState();
			break;

		default:
			reprap.GetPlatform().Message(ErrorMessage, "Unknown SPI request\n");
			SendResponse(ResponseUnknownCommand);
			break;
		}
		break;

	case SpiState::SendingHeader:
		//debugPrintf("sent header\n");
		if (txBufferSize == 0)
		{
			// Header has been sent; ready to process another request
			ResetState();
		}
		else
		{
			state = SpiState::SendingData;
			setup_spi(nullptr, 0, txBuffer, txBufferSize);
		}
		break;

	case SpiState::SendingData:
		// Transfer complete
		//debugPrintf("sent data\n");
		ResetState();
		break;
	}
}

extern "C" void LINUX_SPI_HANDLER(void)
{
	reprap.GetLinuxComm().SpiInterrupt();
}

void LinuxComm::SpiInterrupt()
{
	const uint32_t status = LINUX_SPI->SPI_SR;							// read status and clear interrupt
	LINUX_SPI->SPI_IDR = SPI_IER_NSSR;									// disable the interrupt
	if ((status & SPI_SR_NSSR) != 0)
	{
		xdmac_channel_disable(XDMAC, LINUX_XDMAC_RX_CH);
		xdmac_channel_disable(XDMAC, LINUX_XDMAC_TX_CH);
		spi_disable(LINUX_SPI);
		transferPending = false;
	}
}

void LinuxComm::SetGCodeInput(NetworkGCodeInput *input)
{
	gcodeInput = input;
}

void LinuxComm::HandleGCodeReply(const char *reply)
{
	OutputBuffer *buffer = gcodeReply->GetLastItem();
	if (buffer == nullptr || buffer->IsReferenced())
	{
		if (!OutputBuffer::Allocate(buffer))
		{
			// No more space available, stop here
			return;
		}
		gcodeReply->Push(buffer);
	}

	buffer->cat(reply);
}

void LinuxComm::HandleGCodeReply(OutputBuffer *buffer)
{
	gcodeReply->Push(buffer);
}

void LinuxComm::ResetState()
{
	rxHeader.formatVersion = InvalidFormatVersion;
	rxHeader.dataLength = 0;
	rxHeader.request = LinuxRequest::nullCommand;

	txHeader.formatVersion = LinuxFormatVersion;
	txHeader.response = ResponseEmpty;

	setup_spi(&rxHeader, sizeof(MessageHeaderLinuxToSam), &txHeader, sizeof(MessageHeaderSamToLinux));
	state = SpiState::ReceivingHeader;
}

void LinuxComm::SendResponse(int32_t responseOrBytesToWrite)
{
	rxHeader.formatVersion = InvalidFormatVersion;

	if (responseOrBytesToWrite <= 0)
	{
		txBufferSize = 0;
	}
	txHeader.response = responseOrBytesToWrite;

	setup_spi(nullptr, 0, &txHeader, sizeof(MessageHeaderSamToLinux));
	state = SpiState::SendingHeader;
}

#endif
// End
