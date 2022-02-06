/*
 * DataTransfer.cpp
 *
 *  Created on: 29 Mar 2019
 *      Author: Christian
 */

#include "DataTransfer.h"

#if HAS_SBC_INTERFACE

#include "SbcInterface.h"

#include <Storage/CRC32.h>
#include <algorithm>

#if defined(DUET_NG) && defined(USE_SBC)

// The PDC seems to be too slow to work reliably without getting transmit underruns, so we use the DMAC now.
# define USE_DMAC			1		// use general DMA controller
# define USE_XDMAC			0		// use XDMA controller
# define USE_DMAC_MANAGER	0		// use SAME5x DmacManager module

#elif defined(DUET3) || defined(SAME70XPLD)

# define USE_DMAC			0		// use general DMA controller
# define USE_XDMAC			1		// use XDMA controller
# define USE_DMAC_MANAGER	0		// use SAME5x DmacManager module

#elif defined(DUET3MINI)

# define USE_DMAC			0		// use general DMA controller
# define USE_XDMAC			0		// use XDMA controller
# define USE_DMAC_MANAGER	1		// use SAME5x DmacManager module
constexpr IRQn SBC_SPI_IRQn = SbcSpiSercomIRQn;

#else
# error Unknown board
#endif

#if USE_DMAC
# include <dmac/dmac.h>
# include <matrix/matrix.h>
# include <pmc/pmc.h>
# include <spi/spi.h>
#endif

#if USE_XDMAC
# include <xdmac/xdmac.h>
#endif

#if USE_DMAC_MANAGER || SAME70
# include <DmacManager.h>
#endif

#if SAME70
# include <spi/spi.h>
#endif

#include <RepRapFirmware.h>
#include <GCodes/GCodeMachineState.h>
#include <Movement/Move.h>
#include <Movement/BedProbing/Grid.h>
#include <ObjectModel/ObjectModel.h>
#include <Platform/OutputMemory.h>
#include <Platform/RepRap.h>
#include <Cache.h>
#include <RTOSIface/RTOSIface.h>

#include <General/IP4String.h>

static TaskHandle sbcTaskHandle = nullptr;

#if USE_DMAC

// Hardware IDs of the SPI transmit and receive DMA interfaces. See atsam datasheet.
const uint32_t SBC_SPI_TX_DMA_HW_ID = 1;
const uint32_t SBC_SPI_RX_DMA_HW_ID = 2;

#endif

#if USE_XDMAC

// XDMAC hardware, see datasheet
constexpr uint32_t SBC_SPI_TX_PERID = (uint32_t)DmaTrigSource::spi1tx;
constexpr uint32_t SBC_SPI_RX_PERID = (uint32_t)DmaTrigSource::spi1rx;

static xdmac_channel_config_t xdmac_tx_cfg, xdmac_rx_cfg;

#endif

volatile bool dataReceived = false;		// warning: on the SAME5x this just means the transfer has started, not necessarily that it has ended!
#if SAME5x
uint32_t transferStartTime = 0;
#endif
volatile bool transferReadyHigh = false;
volatile unsigned int spiTxUnderruns = 0, spiRxOverruns = 0;

static void spi_dma_disable() noexcept
{
#if USE_DMAC
	dmac_channel_disable(DMAC, DmacChanSbcRx);
	dmac_channel_disable(DMAC, DmacChanSbcTx);
#endif

#if USE_XDMAC
	xdmac_channel_disable(XDMAC, DmacChanSbcRx);
	xdmac_channel_disable(XDMAC, DmacChanSbcTx);
#endif

#if USE_DMAC_MANAGER
	DmacManager::DisableChannel(DmacChanSbcRx);
	DmacManager::DisableChannel(DmacChanSbcTx);
#endif
}

#if !SAME5x
static bool spi_dma_check_rx_complete() noexcept
{
#if USE_DMAC
	const uint32_t status = DMAC->DMAC_CHSR;
	if (   ((status & (DMAC_CHSR_ENA0 << DmacChanSbcRx)) == 0)	// controller is not enabled, perhaps because it finished a full buffer transfer
		|| ((status & (DMAC_CHSR_EMPT0 << DmacChanSbcRx)) != 0)	// controller is enabled, probably suspended, and the FIFO is empty
	   )
	{
		// Disable the channel.
		// We also need to set the resume bit, otherwise it remains suspended when we re-enable it.
		DMAC->DMAC_CHDR = (DMAC_CHDR_DIS0 << DmacChanSbcRx) | (DMAC_CHDR_RES0 << DmacChanSbcRx);
		return true;
	}
	return false;

#elif USE_XDMAC
	return (xdmac_channel_get_status(XDMAC) & ((1 << DmacChanSbcRx) | (1 << DmacChanSbcTx))) == 0;
#endif
}

#endif

// Set up the transmit DMA but don't enable it
static void spi_tx_dma_setup(const void *outBuffer, size_t bytesToTransfer) noexcept
pre(bytesToTransfer <= outBuffer.limit)
{
#if USE_DMAC
	DMAC->DMAC_EBCISR;		// clear any pending interrupts

	dmac_channel_set_source_addr(DMAC, DmacChanSbcTx, reinterpret_cast<uint32_t>(outBuffer));
	dmac_channel_set_destination_addr(DMAC, DmacChanSbcTx, reinterpret_cast<uint32_t>(&(SBC_SPI->SPI_TDR)));
	dmac_channel_set_descriptor_addr(DMAC, DmacChanSbcTx, 0);
	dmac_channel_set_ctrlA(DMAC, DmacChanSbcTx,
			bytesToTransfer |
			DMAC_CTRLA_SRC_WIDTH_WORD |
			DMAC_CTRLA_DST_WIDTH_BYTE);
	dmac_channel_set_ctrlB(DMAC, DmacChanSbcTx,
		DMAC_CTRLB_SRC_DSCR |
		DMAC_CTRLB_DST_DSCR |
		DMAC_CTRLB_FC_MEM2PER_DMA_FC |
		DMAC_CTRLB_SRC_INCR_INCREMENTING |
		DMAC_CTRLB_DST_INCR_FIXED);
#endif

#if USE_XDMAC
	xdmac_tx_cfg.mbr_ubc = bytesToTransfer;
	xdmac_tx_cfg.mbr_sa = (uint32_t)outBuffer;
	xdmac_tx_cfg.mbr_da = (uint32_t)&(SBC_SPI->SPI_TDR);
	xdmac_tx_cfg.mbr_cfg = XDMAC_CC_TYPE_PER_TRAN |
		XDMAC_CC_MBSIZE_SINGLE |
		XDMAC_CC_DSYNC_MEM2PER |
		XDMAC_CC_CSIZE_CHK_1 |
		XDMAC_CC_DWIDTH_BYTE |
		XDMAC_CC_SIF_AHB_IF0 |
		XDMAC_CC_DIF_AHB_IF1 |
		XDMAC_CC_SAM_INCREMENTED_AM |
		XDMAC_CC_DAM_FIXED_AM |
		XDMAC_CC_PERID(SBC_SPI_TX_PERID);
	xdmac_tx_cfg.mbr_bc = 0;
	xdmac_tx_cfg.mbr_ds = 0;
	xdmac_tx_cfg.mbr_sus = 0;
	xdmac_tx_cfg.mbr_dus = 0;
	xdmac_configure_transfer(XDMAC, DmacChanSbcTx, &xdmac_tx_cfg);

	xdmac_channel_set_descriptor_control(XDMAC, DmacChanSbcTx, 0);
	xdmac_disable_interrupt(XDMAC, DmacChanSbcTx);
#endif

#if USE_DMAC_MANAGER
	DmacManager::SetSourceAddress(DmacChanSbcTx, outBuffer);
	DmacManager::SetDestinationAddress(DmacChanSbcTx, &(SbcSpiSercom->SPI.DATA.reg));
	DmacManager::SetBtctrl(DmacChanSbcTx, DMAC_BTCTRL_STEPSIZE_X1 | DMAC_BTCTRL_STEPSEL_SRC | DMAC_BTCTRL_SRCINC | DMAC_BTCTRL_BEATSIZE_WORD | DMAC_BTCTRL_BLOCKACT_NOACT);
	DmacManager::SetDataLength(DmacChanSbcTx, (bytesToTransfer + 3) >> 2);			// must do this one last
	DmacManager::SetTriggerSourceSercomTx(DmacChanSbcTx, SbcSpiSercomNumber);
#endif
}

// Set up the receive DMA but don't enable it
static void spi_rx_dma_setup(void *inBuffer, size_t bytesToTransfer) noexcept
pre(bytesToTransfer <= inBuffer.limit)
{
#if USE_DMAC
	DMAC->DMAC_EBCISR;		// clear any pending interrupts

	dmac_channel_set_source_addr(DMAC, DmacChanSbcRx, reinterpret_cast<uint32_t>(&(SBC_SPI->SPI_RDR)));
	dmac_channel_set_destination_addr(DMAC, DmacChanSbcRx, reinterpret_cast<uint32_t>(inBuffer));
	dmac_channel_set_descriptor_addr(DMAC, DmacChanSbcRx, 0);
	dmac_channel_set_ctrlA(DMAC, DmacChanSbcRx,
			bytesToTransfer |
			DMAC_CTRLA_SRC_WIDTH_BYTE |
			DMAC_CTRLA_DST_WIDTH_WORD);
	dmac_channel_set_ctrlB(DMAC, DmacChanSbcRx,
		DMAC_CTRLB_SRC_DSCR |
		DMAC_CTRLB_DST_DSCR |
		DMAC_CTRLB_FC_PER2MEM_DMA_FC |
		DMAC_CTRLB_SRC_INCR_FIXED |
		DMAC_CTRLB_DST_INCR_INCREMENTING);
#endif

#if USE_XDMAC
	xdmac_rx_cfg.mbr_ubc = bytesToTransfer;
	xdmac_rx_cfg.mbr_da = (uint32_t)inBuffer;
	xdmac_rx_cfg.mbr_sa = (uint32_t)&(SBC_SPI->SPI_RDR);
	xdmac_rx_cfg.mbr_cfg = XDMAC_CC_TYPE_PER_TRAN |
		XDMAC_CC_MBSIZE_SINGLE |
		XDMAC_CC_DSYNC_PER2MEM |
		XDMAC_CC_CSIZE_CHK_1 |
		XDMAC_CC_DWIDTH_BYTE|
		XDMAC_CC_SIF_AHB_IF1 |
		XDMAC_CC_DIF_AHB_IF0 |
		XDMAC_CC_SAM_FIXED_AM |
		XDMAC_CC_DAM_INCREMENTED_AM |
		XDMAC_CC_PERID(SBC_SPI_RX_PERID);
	xdmac_rx_cfg.mbr_bc = 0;
	xdmac_tx_cfg.mbr_ds = 0;
	xdmac_rx_cfg.mbr_sus = 0;
	xdmac_rx_cfg.mbr_dus = 0;
	xdmac_configure_transfer(XDMAC, DmacChanSbcRx, &xdmac_rx_cfg);

	xdmac_channel_set_descriptor_control(XDMAC, DmacChanSbcRx, 0);
	xdmac_disable_interrupt(XDMAC, DmacChanSbcRx);
#endif

#if USE_DMAC_MANAGER
	DmacManager::SetSourceAddress(DmacChanSbcRx, &(SbcSpiSercom->SPI.DATA.reg));
	DmacManager::SetDestinationAddress(DmacChanSbcRx, inBuffer);
	DmacManager::SetBtctrl(DmacChanSbcRx, DMAC_BTCTRL_STEPSIZE_X1 | DMAC_BTCTRL_STEPSEL_DST | DMAC_BTCTRL_DSTINC | DMAC_BTCTRL_BEATSIZE_WORD | DMAC_BTCTRL_BLOCKACT_INT);
	DmacManager::SetDataLength(DmacChanSbcRx, (bytesToTransfer + 3) >> 2);			// must do this one last
	DmacManager::SetTriggerSourceSercomRx(DmacChanSbcRx, SbcSpiSercomNumber);
#endif
}

/**
 * \brief Set SPI slave transfer.
 */
static void spi_slave_dma_setup(void *inBuffer, const void *outBuffer, size_t bytesToTransfer) noexcept
pre(bytesToTransfer <= inBuffer.limit; bytesToTransfer <= outBuffer.limit)
{
	spi_dma_disable();
	spi_tx_dma_setup(outBuffer, bytesToTransfer);
	spi_rx_dma_setup(inBuffer, bytesToTransfer);

#if USE_DMAC
	dmac_channel_enable(DMAC, DmacChanSbcRx);
	dmac_channel_enable(DMAC, DmacChanSbcTx);
#endif

#if USE_XDMAC
	xdmac_channel_enable(XDMAC, DmacChanSbcRx);
	xdmac_channel_enable(XDMAC, DmacChanSbcTx);
#endif

#if USE_DMAC_MANAGER
	DmacManager::EnableChannel(DmacChanSbcRx, DmacPrioSbc);
	DmacManager::EnableChannel(DmacChanSbcTx, DmacPrioSbc);
#endif
}

void disable_spi() noexcept
{
	spi_dma_disable();

#if SAME5x
	SbcSpiSercom->SPI.CTRLA.reg &= ~SERCOM_SPI_CTRLA_ENABLE;
	while (SbcSpiSercom->SPI.SYNCBUSY.reg & (SERCOM_SPI_SYNCBUSY_SWRST | SERCOM_SPI_SYNCBUSY_ENABLE)) { };
#else
	spi_disable(SBC_SPI);
#endif
}

static void setup_spi(void *inBuffer, const void *outBuffer, size_t bytesToTransfer) noexcept
pre(bytesToTransfer <= inBuffer.limit; bytesToTransfer <= outBuffer.limit)
{
#if !SAME5x
	// Reset SPI
	spi_reset(SBC_SPI);
	spi_set_slave_mode(SBC_SPI);
	spi_disable_mode_fault_detect(SBC_SPI);
	spi_set_peripheral_chip_select_value(SBC_SPI, spi_get_pcs(0));
	spi_set_clock_polarity(SBC_SPI, 0, 0);
	spi_set_clock_phase(SBC_SPI, 0, 1);
	spi_set_bits_per_transfer(SBC_SPI, 0, SPI_CSR_BITS_8_BIT);
#endif

	// Initialize channel config for transmitter and receiver
	spi_slave_dma_setup(inBuffer, outBuffer, bytesToTransfer);

#if USE_DMAC
	// Configure DMA RX channel
	dmac_channel_set_configuration(DMAC, DmacChanSbcRx,
			DMAC_CFG_SRC_PER(SBC_SPI_RX_DMA_HW_ID) |
			DMAC_CFG_SRC_H2SEL |
			DMAC_CFG_SOD |
			DMAC_CFG_FIFOCFG_ASAP_CFG);

	// Configure DMA TX channel
	dmac_channel_set_configuration(DMAC, DmacChanSbcTx,
			DMAC_CFG_DST_PER(SBC_SPI_TX_DMA_HW_ID) |
			DMAC_CFG_DST_H2SEL |
			DMAC_CFG_SOD |
			DMAC_CFG_FIFOCFG_ASAP_CFG);
#endif

	// Enable SPI and notify the SBC we are ready
#if SAME5x
	SbcSpiSercom->SPI.INTFLAG.reg = 0xFF;						// clear any pending interrupts
	SbcSpiSercom->SPI.INTENSET.reg = SERCOM_SPI_INTENSET_SSL;	// enable the start of transfer (SS low) interrupt
	SbcSpiSercom->SPI.CTRLA.reg |= SERCOM_SPI_CTRLA_ENABLE;
	while (SbcSpiSercom->SPI.SYNCBUSY.reg & (SERCOM_SPI_SYNCBUSY_SWRST | SERCOM_SPI_SYNCBUSY_ENABLE)) { };
#else
	spi_enable(SBC_SPI);

	// Enable end-of-transfer interrupt
	(void)SBC_SPI->SPI_SR;							// clear any pending interrupt
	SBC_SPI->SPI_IER = SPI_IER_NSSR;				// enable the NSS rising interrupt
#endif

	NVIC_SetPriority(SBC_SPI_IRQn, NvicPrioritySpi);
	NVIC_EnableIRQ(SBC_SPI_IRQn);

	// Begin transfer
#if SAME5x
	transferStartTime = 0;
#endif
	transferReadyHigh = !transferReadyHigh;
	digitalWrite(SbcTfrReadyPin, transferReadyHigh);
}


#ifndef SBC_SPI_HANDLER
# error SBC_SPI_HANDLER undefined
#endif

extern "C" void SBC_SPI_HANDLER() noexcept
{
#if SAME5x
	// On the SAM5x we can't get an end-of-transfer interrupt, only a start-of-transfer interrupt.
	// So we can't disable SPI or DMA in this ISR.
	const uint8_t status = SbcSpiSercom->SPI.INTFLAG.reg;
	if ((status & SERCOM_SPI_INTENSET_SSL) != 0)
	{
		SbcSpiSercom->SPI.INTENCLR.reg = SERCOM_SPI_INTENSET_SSL;		// disable the interrupt
		SbcSpiSercom->SPI.INTFLAG.reg = SERCOM_SPI_INTENSET_SSL;		// clear the status

		// Wake up the SBC task
		dataReceived = true;
		TaskBase::GiveFromISR(sbcTaskHandle);
	}
#else
	const uint32_t status = SBC_SPI->SPI_SR;							// read status and clear interrupt
	SBC_SPI->SPI_IDR = SPI_IER_NSSR;									// disable the interrupt
	if ((status & SPI_SR_NSSR) != 0)
	{
		// Data has been transferred, disable transfer ready pin and XDMAC channels
		disable_spi();

		// Check if any error occurred
		if ((status & SPI_SR_OVRES) != 0)
		{
			++spiRxOverruns;
		}
		if ((status & SPI_SR_UNDES) != 0)
		{
			++spiTxUnderruns;
		}

		// Wake up the SBC task
		dataReceived = true;
		TaskBase::GiveFromISR(sbcTaskHandle);
	}
#endif
}

/*-----------------------------------------------------------------------------------*/

// Static data. Note, the startup code we use doesn't make any provision for initialising non-cached memory, other than to zero. So don't specify initial value here

#if SAME70
__nocache TransferHeader DataTransfer::rxHeader;
__nocache TransferHeader DataTransfer::txHeader;
__nocache uint32_t DataTransfer::rxResponse;
__nocache uint32_t DataTransfer::txResponse;
alignas(4) __nocache char DataTransfer::rxBuffer[SbcTransferBufferSize];
alignas(4) __nocache char DataTransfer::txBuffer[SbcTransferBufferSize];
#endif

DataTransfer::DataTransfer() noexcept : state(InternalTransferState::ExchangingData), lastTransferNumber(0), failedTransfers(0), checksumErrors(0),
#if SAME5x
	rxBuffer(nullptr), txBuffer(nullptr),
#endif
	rxPointer(0), txPointer(0), packetId(0)
{
	rxResponse = TransferResponse::Success;
	txResponse = TransferResponse::Success;

	// Prepare RX header
	rxHeader.sequenceNumber = 0;

	// Prepare TX header
	txHeader.formatCode = SbcFormatCode;
	txHeader.protocolVersion = SbcProtocolVersion;
	txHeader.numPackets = 0;
	txHeader.sequenceNumber = 0;
}

void DataTransfer::Init() noexcept
{
	// Initialise transfer ready pin
	pinMode(SbcTfrReadyPin, OUTPUT_LOW);

#if !SAME70
	if (reprap.UsingSbcInterface())
	{
		// Allocate buffers in SBC mode
		rxBuffer = (char *)new uint32_t[(SbcTransferBufferSize + 3)/4];
		txBuffer = (char *)new uint32_t[(SbcTransferBufferSize + 3)/4];
	}
	else
	{
		// Only send back the TX header + response code in standalone mode indicating we're running in standalone mode
		txHeader.formatCode = SbcFormatCodeStandalone;
	}
#endif

#if SAME5x
	// Initialize SPI
	for (Pin p : SbcSpiSercomPins)
	{
		SetPinFunction(p, SbcSpiSercomPinsMode);
	}

	Serial::EnableSercomClock(SbcSpiSercomNumber);
	spi_dma_disable();

	SbcSpiSercom->SPI.CTRLA.reg |= SERCOM_SPI_CTRLA_SWRST;
	while (SbcSpiSercom->SPI.SYNCBUSY.reg & SERCOM_SPI_SYNCBUSY_SWRST) { };
	SbcSpiSercom->SPI.CTRLA.reg = SERCOM_SPI_CTRLA_DIPO(3) | SERCOM_SPI_CTRLA_DOPO(0) | SERCOM_SPI_CTRLA_MODE(2);
	SbcSpiSercom->SPI.CTRLB.reg = SERCOM_SPI_CTRLB_RXEN | SERCOM_SPI_CTRLB_SSDE | SERCOM_SPI_CTRLB_PLOADEN;
	while (SbcSpiSercom->SPI.SYNCBUSY.reg & SERCOM_SPI_SYNCBUSY_MASK) { };
	SbcSpiSercom->SPI.CTRLC.reg = SERCOM_SPI_CTRLC_DATA32B;
#else
	// Initialize SPI
	SetPinFunction(APIN_SBC_SPI_MOSI, SBCPinPeriphMode);
	SetPinFunction(APIN_SBC_SPI_MISO, SBCPinPeriphMode);
	SetPinFunction(APIN_SBC_SPI_SCK, SBCPinPeriphMode);
	SetPinFunction(APIN_SBC_SPI_SS0, SBCPinPeriphMode);

	spi_enable_clock(SBC_SPI);
	spi_disable(SBC_SPI);
#endif

	dataReceived = false;

#if false // if SAME70
	// This does not seem to change anything...
	// The XDMAC is master 4+5 and the SRAM is slave 0+1. Give the XDMAC the highest priority.
	matrix_set_slave_default_master_type(0, MATRIX_DEFMSTR_LAST_DEFAULT_MASTER);
	matrix_set_slave_priority(0, MATRIX_PRAS_M4PR(10));
	matrix_set_slave_priority(1, MATRIX_PRAS_M5PR(11));
	// Set the slave slot cycle limit.
	// If we leave it at the default value of 511 clock cycles, we get transmit underruns due to the HSMCI using the bus for too long.
	// A value of 8 seems to work. I haven't tried other values yet.
	matrix_set_slave_slot_cycle(0, 8);
	matrix_set_slave_slot_cycle(1, 8);
#endif
#if USE_DMAC
	pmc_enable_periph_clk(ID_DMAC);
	dmac_init(DMAC);
	dmac_set_priority_mode(DMAC, DMAC_PRIORITY_ROUND_ROBIN);
	dmac_enable(DMAC);

	// The DMAC is master 4 and the SRAM is slave 0. Give the DMAC the highest priority.
	matrix_set_slave_default_master_type(0, MATRIX_DEFMSTR_LAST_DEFAULT_MASTER);
	matrix_set_slave_priority(0, (3 << MATRIX_PRAS0_M4PR_Pos));
	// Set the slave slot cycle limit.
	// If we leave it at the default value of 511 clock cycles, we get transmit underruns due to the HSMCI using the bus for too long.
	// A value of 8 seems to work. I haven't tried other values yet.
	matrix_set_slave_slot_cycle(0, 8);
#endif

	if (!reprap.UsingSbcInterface())
	{
		// Start off the first transfer in standalone mode
		StartNextTransfer();
	}
}

void DataTransfer::InitFromTask() noexcept
{
	sbcTaskHandle = TaskBase::GetCallerTaskHandle();
}

void DataTransfer::Diagnostics(MessageType mtype) noexcept
{
	reprap.GetPlatform().MessageF(mtype, "Transfer state: %d, failed transfers: %u, checksum errors: %u\n", (int)state, failedTransfers, checksumErrors);
	reprap.GetPlatform().MessageF(mtype, "RX/TX seq numbers: %d/%d\n", (int)rxHeader.sequenceNumber, (int)txHeader.sequenceNumber);
	reprap.GetPlatform().MessageF(mtype, "SPI underruns %u, overruns %u\n", spiTxUnderruns, spiRxOverruns);
}

const PacketHeader *DataTransfer::ReadPacket() noexcept
{
	if (rxPointer >= rxHeader.dataLength)
	{
		return nullptr;
	}

	const PacketHeader *header = reinterpret_cast<const PacketHeader*>(rxBuffer + rxPointer);
	rxPointer += sizeof(PacketHeader);
	return header;
}

const char *DataTransfer::ReadData(size_t dataLength) noexcept
{
	const char *data = rxBuffer + rxPointer;
	rxPointer += AddPadding(dataLength);
	return data;
}

template<typename T> const T *DataTransfer::ReadDataHeader() noexcept
{
	const T *header = reinterpret_cast<const T*>(rxBuffer + rxPointer);
	rxPointer += sizeof(T);
	return header;
}

bool DataTransfer::ReadBoolean() noexcept
{
	const BooleanHeader *header = ReadDataHeader<BooleanHeader>();
	return header->value;
}

void DataTransfer::ReadGetObjectModel(size_t packetLength, const StringRef &key, const StringRef &flags) noexcept
{
	// Read header
	const GetObjectModelHeader *header = ReadDataHeader<GetObjectModelHeader>();
	const char *data = ReadData(packetLength - sizeof(GetObjectModelHeader));

	// Read key
	key.copy(data, header->keyLength);
	data += header->keyLength;

	// Read flags
	flags.copy(data, header->flagsLength);
}

void DataTransfer::ReadPrintStartedInfo(size_t packetLength, const StringRef& filename, GCodeFileInfo& info) noexcept
{
	// Read header
	const PrintStartedHeader *header = ReadDataHeader<PrintStartedHeader>();
	info.Init();
	info.numFilaments = header->numFilaments;
	info.numLayers = header->numLayers;
	info.lastModifiedTime = header->lastModifiedTime;
	info.fileSize = header->fileSize;
	info.layerHeight = header->layerHeight;
	info.objectHeight = header->objectHeight;
	info.printTime = header->printTime;
	info.simulatedTime = header->simulatedTime;
	info.isValid = true;

	// Read filaments
	memset(info.filamentNeeded, 0, ARRAY_SIZE(info.filamentNeeded) * sizeof(float));
	const char *data = ReadData(packetLength - sizeof(PrintStartedHeader));
	size_t filamentsSize = info.numFilaments * sizeof(float);
	memcpyf(info.filamentNeeded, reinterpret_cast<const float *>(data), info.numFilaments);
	data += filamentsSize;

	// Read file name
	filename.copy(data, header->filenameLength);
	data += header->filenameLength;

	// Read generated by
	info.generatedBy.copy(data, header->generatedByLength);
}

PrintStoppedReason DataTransfer::ReadPrintStoppedInfo() noexcept
{
	const PrintStoppedHeader *header = ReadDataHeader<PrintStoppedHeader>();
	return header->reason;
}

GCodeChannel DataTransfer::ReadMacroCompleteInfo(bool &error) noexcept
{
	const MacroCompleteHeader *header = ReadDataHeader<MacroCompleteHeader>();
	error = header->error;
	return GCodeChannel(header->channel);
}

GCodeChannel DataTransfer::ReadCodeChannel() noexcept
{
	const CodeChannelHeader *header = ReadDataHeader<CodeChannelHeader>();
	return GCodeChannel(header->channel);
}

void DataTransfer::ReadFileChunk(char *buffer, int32_t& dataLength, uint32_t& fileLength) noexcept
{
	// Read header
	const FileChunk *header = ReadDataHeader<FileChunk>();
	dataLength = header->dataLength;
	fileLength = header->fileLength;

	// Read file chunk
	if (header->dataLength > 0)
	{
		memcpy(buffer, ReadData(header->dataLength), header->dataLength);
	}
}

GCodeChannel DataTransfer::ReadEvaluateExpression(size_t packetLength, const StringRef& expression) noexcept
{
	// Read header
	const CodeChannelHeader *header = ReadDataHeader<CodeChannelHeader>();

	// Read expression
	size_t expressionLength = packetLength - sizeof(CodeChannelHeader);
	const char *expressionData = ReadData(expressionLength);
	expression.copy(expressionData, expressionLength);

	return GCodeChannel(header->channel);
}

bool DataTransfer::ReadMessage(MessageType& type, OutputBuffer *buf) noexcept
{
	// Read header
	const MessageHeader *header = ReadDataHeader<MessageHeader>();
	type = header->messageType;

	// Read message data and check if the it could be fully read
	const char *messageData = ReadData(header->length);
	return buf->copy(messageData, header->length) == header->length;
}

GCodeChannel DataTransfer::ReadSetVariable(bool& createVariable, const StringRef& varName, const StringRef& expression) noexcept
{
	// Read header
	const SetVariableHeader *header = ReadDataHeader<SetVariableHeader>();
	createVariable = header->createVariable;

	// Read variable name and expression
	const char *data = ReadData(header->variableLength + header->expressionLength);
	varName.copy(data, header->variableLength);
	expression.copy(data + header->variableLength, header->expressionLength);

	return GCodeChannel(header->channel);
}

GCodeChannel DataTransfer::ReadDeleteLocalVariable(const StringRef& varName) noexcept
{
	// Read header
	const DeleteLocalVariableHeader *header = ReadDataHeader<DeleteLocalVariableHeader>();

	// Read variable name
	const char *varNameData = ReadData(header->variableLength);
	varName.copy(varNameData, header->variableLength);

	return GCodeChannel(header->channel);
}

FileHandle DataTransfer::ReadOpenFileResult(FilePosition& fileLength) noexcept
{
	// Read header
	const OpenFileResult *header = ReadDataHeader<OpenFileResult>();

	// Read values
	fileLength = header->fileSize;
	return header->handle;
}

int DataTransfer::ReadFileData(char *buffer, size_t length) noexcept
{
	// Read header
	const FileDataHeader *header = ReadDataHeader<FileDataHeader>();
	int bytesToRead = min<int>(header->bytesRead, length);

	// Read data content if applicable
	if (bytesToRead > 0)
	{
		const char *fileData = ReadData(header->bytesRead);
		memcpy(buffer, fileData, bytesToRead);
	}
	return bytesToRead;
}

void DataTransfer::ExchangeHeader() noexcept
{
	Cache::FlushBeforeDMASend(&txHeader, sizeof(txHeader));
	state = InternalTransferState::ExchangingHeader;
	setup_spi(&rxHeader, &txHeader, sizeof(TransferHeader));
}

void DataTransfer::ExchangeResponse(uint32_t response) noexcept
{
	txResponse = response;
	Cache::FlushBeforeDMASend(&txResponse, sizeof(txResponse));
	state = (state == InternalTransferState::ExchangingHeader) ? InternalTransferState::ExchangingHeaderResponse : InternalTransferState::ExchangingDataResponse;
	setup_spi(&rxResponse, &txResponse, sizeof(uint32_t));
}

void DataTransfer::ExchangeData() noexcept
{
	Cache::FlushBeforeDMASend(txBuffer, txHeader.dataLength);
	size_t bytesToExchange = max<size_t>(rxHeader.dataLength, txHeader.dataLength);
	state = InternalTransferState::ExchangingData;
	setup_spi(rxBuffer, txBuffer, bytesToExchange);
}

void DataTransfer::ResetTransfer(bool ownRequest) noexcept
{
	if (reprap.Debug(moduleSbcInterface))
	{
		debugPrintf(ownRequest ? "Resetting transfer\n" : "Resetting transfer due to Sbc request\n");
	}

	if (ownRequest)
	{
		// Invalidate the data to send
		txResponse = TransferResponse::BadResponse;
		Cache::FlushBeforeDMASend(&txResponse, sizeof(txResponse));
		state = InternalTransferState::Resetting;
		setup_spi(&rxResponse, &txResponse, sizeof(uint32_t));
	}
	else
	{
		// Sbc wants to reset the state
		ExchangeHeader();
	}
}

TransferState DataTransfer::DoTransfer() noexcept
{
	if (dataReceived)
	{
#if SAME5x
		// Unfortunately the SAME5x doesn't have an end-of-transfer interrupt, but SPI transfers typically don't take long
		if (!digitalRead(SbcSSPin))				// transfer is complete if SS is high
		{
			if (transferStartTime == 0)
			{
				transferStartTime = millis();
				return TransferState::finishingTransfer;
			}
			return (millis() - transferStartTime > SpiMaxTransferTime) ? TransferState::connectionTimeout : TransferState::finishingTransfer;
		}
		transferStartTime = 0;

		if (SbcSpiSercom->SPI.STATUS.bit.BUFOVF)
		{
			++spiRxOverruns;
		}
		disable_spi();
#else
		// Wait for the current XDMA transfer to finish. Relying on the XDMAC IRQ for this is does not work well...
		if (!spi_dma_check_rx_complete())
		{
			return TransferState::finishingTransfer;
		}
#endif

		// Transfer has finished
		dataReceived = false;

		switch (state)
		{
		case InternalTransferState::ExchangingHeader:
		{
			Cache::InvalidateAfterDMAReceive(&rxHeader, sizeof(rxHeader));
			// (1) Exchanged transfer headers
			const uint32_t headerResponse = *reinterpret_cast<const uint32_t*>(&rxHeader);
			if (headerResponse == TransferResponse::BadResponse)
			{
				// Sbc wants to restart the transfer
				ResetTransfer(false);
				break;
			}

			const uint32_t checksum = CalcCRC32(reinterpret_cast<const char *>(&rxHeader), sizeof(TransferHeader) - sizeof(uint32_t));
			if (rxHeader.crcHeader != checksum)
			{
				if (reprap.Debug(moduleSbcInterface))
				{
					debugPrintf("Bad header CRC (expected %08" PRIx32 ", got %08" PRIx32 ")\n", rxHeader.crcHeader, checksum);
				}
				ExchangeResponse(TransferResponse::BadHeaderChecksum);
				break;
			}

			if (rxHeader.formatCode != SbcFormatCode)
			{
				ExchangeResponse(TransferResponse::BadFormat);
				break;
			}
			if (rxHeader.protocolVersion != SbcProtocolVersion)
			{
				ExchangeResponse(TransferResponse::BadProtocolVersion);
				break;
			}
			if (rxHeader.dataLength > SbcTransferBufferSize)
			{
				ExchangeResponse(TransferResponse::BadDataLength);
				break;
			}

			ExchangeResponse(TransferResponse::Success);
			break;
		}

		case InternalTransferState::ExchangingHeaderResponse:
			// (2) Exchanged response to transfer header
			Cache::InvalidateAfterDMAReceive(&rxResponse, sizeof(rxResponse));
			if (rxResponse == TransferResponse::Success && txResponse == TransferResponse::Success)
			{
				if (reprap.UsingSbcInterface() && (rxHeader.dataLength != 0 || txHeader.dataLength != 0))
				{
					// Perform the actual data transfer
					ExchangeData();
				}
				else
				{
					// Everything OK
					rxPointer = txPointer = 0;
					packetId = 0;
					state = InternalTransferState::ProcessingData;
					return IsConnectionReset() ? TransferState::connectionReset : TransferState::finished;
				}
			}
			else if (rxResponse == TransferResponse::BadResponse)
			{
				// Sbc wants to restart the transfer
				ResetTransfer(false);
			}
			else if (rxResponse == TransferResponse::BadHeaderChecksum || txResponse == TransferResponse::BadHeaderChecksum)
			{
				// Failed to exchange header, restart the full transfer
				checksumErrors++;
				ExchangeHeader();
			}
			else
			{
				// Received invalid response code
				ResetTransfer(true);
			}
			break;

		case InternalTransferState::ExchangingData:
		{
			Cache::InvalidateAfterDMAReceive(rxBuffer, rxHeader.dataLength);
			// (3) Exchanged data
			if (*reinterpret_cast<uint32_t*>(rxBuffer) == TransferResponse::BadResponse)
			{
				if (reprap.Debug(moduleSbcInterface))
				{
					debugPrintf("Resetting state due to Sbc request\n");
				}
				failedTransfers++;
				ExchangeHeader();
				break;
			}

			const uint32_t checksum = CalcCRC32(rxBuffer, rxHeader.dataLength);
			if (rxHeader.crcData != checksum)
			{
				if (reprap.Debug(moduleSbcInterface))
				{
					debugPrintf("Bad data CRC (expected %08" PRIx32 ", got %08" PRIx32 ")\n", rxHeader.crcData, checksum);
				}
				ExchangeResponse(TransferResponse::BadDataChecksum);
				break;
			}

			ExchangeResponse(TransferResponse::Success);
			break;
		}

		case InternalTransferState::ExchangingDataResponse:
			// (4) Exchanged response to data transfer
			Cache::InvalidateAfterDMAReceive(&rxResponse, sizeof(rxResponse));
			if (rxResponse == TransferResponse::Success && txResponse == TransferResponse::Success)
			{
				// Everything OK
				rxPointer = txPointer = 0;
				packetId = 0;
				state = InternalTransferState::ProcessingData;
				return IsConnectionReset() ? TransferState::connectionReset : TransferState::finished;
			}

			if (rxResponse == TransferResponse::BadResponse)
			{
				// Sbc wants to restart the transfer
				ResetTransfer(false);
			}
			else if (rxResponse == TransferResponse::BadDataChecksum || txResponse == TransferResponse::BadDataChecksum)
			{
				// Resend the data if a checksum error occurred
				checksumErrors++;
				ExchangeData();
			}
			else
			{
				// Received invalid response, reset the SPI transfer
				ResetTransfer(true);
			}
			break;

		case InternalTransferState::Resetting:
			// Transmitted bad response, attempt to start a new transfer
			ExchangeHeader();
			break;

		default:
			// Should never get here. If we do, this probably means that StartNextTransfer has not been called
			ExchangeHeader();
			REPORT_INTERNAL_ERROR;
			break;
		}
	}
	return (state == InternalTransferState::ExchangingHeader) ? TransferState::doingFullTransfer : TransferState::doingPartialTransfer;
}

void DataTransfer::StartNextTransfer() noexcept
{
	lastTransferNumber = rxHeader.sequenceNumber;

	// Reset RX transfer header
	rxHeader.formatCode = InvalidFormatCode;
	rxHeader.numPackets = 0;
	rxHeader.protocolVersion = 0;
	rxHeader.dataLength = 0;
	rxHeader.crcData = 0;
	rxHeader.crcHeader = 0;

	// Set up TX transfer header
	txHeader.numPackets = packetId;
	txHeader.sequenceNumber++;
	txHeader.dataLength = txPointer;
	txHeader.crcData = CalcCRC32(txBuffer, txPointer);
	txHeader.crcHeader = CalcCRC32(reinterpret_cast<const char *>(&txHeader), sizeof(TransferHeader) - sizeof(uint32_t));

	// Begin SPI transfe
	ExchangeHeader();
}

void DataTransfer::ResetConnection(bool fullReset) noexcept
{
	// The Sbc interface is no longer connected...
	disable_spi();
	dataReceived = false;

	// Reset the sequence numbers and clear the data to send
	lastTransferNumber = 0;
	rxHeader.sequenceNumber = 0;
	txHeader.sequenceNumber = 0;
	rxPointer = txPointer = 0;
	packetId = 0;

	// Kick off a new transfer
	if (fullReset)
	{
		transferReadyHigh = false;
	}
	StartNextTransfer();
}

bool DataTransfer::WriteObjectModel(OutputBuffer *data) noexcept
{
	// Try to write the packet header. This packet type cannot deal with truncated messages
	if (!CanWritePacket(data->Length()))
	{
		return false;
	}

	// Write packet header
	(void)WritePacketHeader(FirmwareRequest::ObjectModel, sizeof(StringHeader) + data->Length());

	// Write header
	StringHeader *header = WriteDataHeader<StringHeader>();
	header->length = data->Length();
	header->padding = 0;

	// Write data
	while (data != nullptr)
	{
		WriteData(data->UnreadData(), data->BytesLeft());
		data = OutputBuffer::Release(data);
	}
	return true;
}

bool DataTransfer::WriteCodeBufferUpdate(uint16_t bufferSpace) noexcept
{
	if (!CanWritePacket(sizeof(CodeBufferUpdateHeader)))
	{
		return false;
	}

	// Write packet header
	(void)WritePacketHeader(FirmwareRequest::CodeBufferUpdate, sizeof(CodeBufferUpdateHeader));

	// Write header
	CodeBufferUpdateHeader *header = WriteDataHeader<CodeBufferUpdateHeader>();
	header->bufferSpace = bufferSpace;
	header->padding = 0;
	return true;
}

bool DataTransfer::WriteCodeReply(MessageType type, OutputBuffer *&response) noexcept
{
	// Try to write the packet header. This packet type can deal with truncated messages
	const size_t minBytesToWrite = min<size_t>(16, (response == nullptr) ? 0 : response->Length());
	if (!CanWritePacket(sizeof(MessageHeader) + minBytesToWrite))
	{
		// Not enough space left
		return false;
	}

	// Write packet header
	PacketHeader *header = WritePacketHeader(FirmwareRequest::Message);

	// Write code reply header
	MessageHeader *replyHeader = WriteDataHeader<MessageHeader>();
	replyHeader->messageType = type;
	replyHeader->padding = 0;

	// Write code reply
	size_t bytesWritten = 0;
	if (response != nullptr)
	{
		size_t bytesToCopy;
		do
		{
			bytesToCopy = min<size_t>(FreeTxSpace(), response->BytesLeft());
			if (bytesToCopy == 0)
			{
				break;
			}

			WriteData(response->UnreadData(), bytesToCopy);
			bytesWritten += bytesToCopy;

			response->Taken(bytesToCopy);
			if (response->BytesLeft() == 0)
			{
				response = OutputBuffer::Release(response);
			}
		}
		while (response != nullptr);

		if (response != nullptr)
		{
			// There is more to come...
			replyHeader->messageType = (MessageType)(replyHeader->messageType | PushFlag);
		}
	}

	// Finish the packet
	replyHeader->length = bytesWritten;
	header->length = sizeof(MessageHeader) + bytesWritten;
	return true;
}

bool DataTransfer::WriteMacroRequest(GCodeChannel channel, const char *filename, bool fromCode) noexcept
{
	size_t filenameLength = strlen(filename);
	if (!CanWritePacket(sizeof(ExecuteMacroHeader) + filenameLength))
	{
		return false;
	}

	// Write packet header
	WritePacketHeader(FirmwareRequest::ExecuteMacro, sizeof(ExecuteMacroHeader) + filenameLength);

	// Write header
	ExecuteMacroHeader *header = WriteDataHeader<ExecuteMacroHeader>();
	header->channel = channel.RawValue();
	header->dummy = 0;
	header->fromCode = fromCode;
	header->length = filenameLength;

	// Write filename
	WriteData(filename, filenameLength);
	return true;
}

bool DataTransfer::WriteAbortFileRequest(GCodeChannel channel, bool abortAll) noexcept
{
	if (!CanWritePacket(sizeof(AbortFileHeader)))
	{
		return false;
	}

	// Write packet header
	WritePacketHeader(FirmwareRequest::AbortFile, sizeof(AbortFileHeader));

	// Write header
	AbortFileHeader *header = WriteDataHeader<AbortFileHeader>();
	header->channel = channel.RawValue();
	header->abortAll = abortAll;
	header->padding = 0;
	return true;
}

bool DataTransfer::WriteMacroFileClosed(GCodeChannel channel) noexcept
{
	if (!CanWritePacket(sizeof(CodeChannelHeader)))
	{
		return false;
	}

	// Write packet header
	WritePacketHeader(FirmwareRequest::MacroFileClosed, sizeof(CodeChannelHeader));

	// Write header
	CodeChannelHeader *header = WriteDataHeader<CodeChannelHeader>();
	header->channel = channel.RawValue();
	header->paddingA = 0;
	header->paddingB = 0;
	return true;
}

bool DataTransfer::WritePrintPaused(FilePosition position, PrintPausedReason reason) noexcept
{
	if (!CanWritePacket(sizeof(PrintPausedHeader)))
	{
		return false;
	}

	// Write packet header
	WritePacketHeader(FirmwareRequest::PrintPaused, sizeof(PrintPausedHeader));

	// Write header
	PrintPausedHeader *header = WriteDataHeader<PrintPausedHeader>();
	header->filePosition = position;
	header->pauseReason = reason;
	header->paddingA = 0;
	header->paddingB = 0;
	return true;
}

bool DataTransfer::WriteLocked(GCodeChannel channel) noexcept
{
	if (!CanWritePacket(sizeof(CodeChannelHeader)))
	{
		return false;
	}

	// Write packet header
	WritePacketHeader(FirmwareRequest::Locked, sizeof(CodeChannelHeader));

	// Write header
	CodeChannelHeader * const header = WriteDataHeader<CodeChannelHeader>();
	header->channel = channel.ToBaseType();
	header->paddingA = 0;
	header->paddingB = 0;
	return true;
}

bool DataTransfer::WriteFileChunkRequest(const char *filename, uint32_t offset, uint32_t maxLength) noexcept
{
	const size_t filenameLength = strlen(filename);
	if (!CanWritePacket(sizeof(FileChunkHeader) + filenameLength))
	{
		return false;
	}
	// Write packet header
	(void)WritePacketHeader(FirmwareRequest::FileChunk, sizeof(FileChunkHeader) + filenameLength);

	// Write header
	FileChunkHeader *header = WriteDataHeader<FileChunkHeader>();
	header->offset = offset;
	header->maxLength = maxLength;
	header->filenameLength = filenameLength;

	// Write data
	WriteData(filename, filenameLength);
	return true;
}

bool DataTransfer::WriteEvaluationResult(const char *expression, const ExpressionValue& value) noexcept
{
	// Calculate payload length
	const size_t expressionLength = strlen(expression);
	size_t payloadLength;
	String<StringLength50> rslt;
	switch (value.GetType())
	{
	case TypeCode::None:
	// FIXME Add support for arrays
	case TypeCode::Bool:
	case TypeCode::DriverId_tc:
	case TypeCode::Uint32:
	case TypeCode::Float:
	case TypeCode::Int32:
		payloadLength = expressionLength;
		break;
	case TypeCode::Uint64:
		payloadLength = AddPadding(expressionLength) + sizeof(uint64_t);
		break;
	case TypeCode::CString:
		payloadLength = expressionLength + strlen(value.sVal);
		break;
	case TypeCode::IPAddress_tc:
	case TypeCode::MacAddress_tc:
	case TypeCode::DateTime_tc:
	case TypeCode::Port:
	case TypeCode::UniqueId_tc:
		// All these types are represented as strings
		value.AppendAsString(rslt.GetRef());
		payloadLength = expressionLength + rslt.strlen();
		break;
	case TypeCode::HeapString:
		payloadLength = expressionLength + value.shVal.GetLength();
		break;

	default:
		rslt.printf("unsupported type code %d", (int)value.type);
		payloadLength = expressionLength + rslt.strlen();
		break;
	}

	// Check if it fits
	if (!CanWritePacket(sizeof(EvaluationResultHeader) + payloadLength))
	{
		return false;
	}

	// Write packet header
	(void)WritePacketHeader(FirmwareRequest::EvaluationResult, sizeof(EvaluationResultHeader) + payloadLength);

	// Write partial header
	EvaluationResultHeader *header = WriteDataHeader<EvaluationResultHeader>();
	header->expressionLength = expressionLength;

	// Write expression
	WriteData(expression, expressionLength);

	// Write data type and expression value
	switch (value.GetType())
	{
	case TypeCode::None:
		header->dataType = DataType::Null;
		header->intValue = 0;
		break;
	case TypeCode::Bool:
		header->dataType = DataType::Bool;
		header->intValue = value.bVal ? 1 : 0;
		break;
	case TypeCode::CString:
		header->dataType = DataType::String;
		header->intValue = strlen(value.sVal);
		WriteData(value.sVal, header->intValue);
		break;
	case TypeCode::DriverId_tc:
		header->dataType = DataType::DriverId_dt;
		header->uintValue = value.uVal;
		break;
	case TypeCode::Uint32:
		header->dataType = DataType::UInt;
		header->uintValue = value.uVal;
		break;
	case TypeCode::Float:
		header->dataType = DataType::Float;
		header->floatValue = value.fVal;
		break;
	case TypeCode::Int32:
		header->dataType = DataType::Int;
		header->intValue = value.iVal;
		break;
	case TypeCode::Uint64:
	{
		header->dataType = DataType::ULong;
		header->intValue = 0;
		txPointer = AddPadding(txPointer);		// add padding to remain on a 4-byte boundary
		uint64_t ulVal = value.Get56BitValue();
		WriteData(reinterpret_cast<const char *>(&ulVal), sizeof(uint64_t));
		break;
	}
	case TypeCode::HeapString:
		header->dataType = DataType::String;
		header->intValue = value.shVal.GetLength();
		WriteData(value.shVal.Get().Ptr(), header->intValue);
		break;
	case TypeCode::DateTime_tc:
	case TypeCode::MacAddress_tc:
	case TypeCode::IPAddress_tc:
	case TypeCode::Port:
	case TypeCode::UniqueId_tc:
	default:
		// We have already converted the value to a string in 'rslt'
		header->dataType = (value.GetType() == TypeCode::DateTime_tc) ? DataType::DateTime : DataType::String;
		header->intValue = rslt.strlen();
		WriteData(rslt.c_str(), rslt.strlen());
		break;
	}
	return true;
}

bool DataTransfer::WriteEvaluationError(const char *expression, const char *errorMessage) noexcept
{
	// Check if it fits
	size_t expressionLength = strlen(expression), errorLength = strlen(errorMessage);
	if (!CanWritePacket(sizeof(EvaluationResultHeader) + expressionLength + errorLength))
	{
		return false;
	}

	// Write packet header
	(void)WritePacketHeader(FirmwareRequest::EvaluationResult, sizeof(EvaluationResultHeader) + expressionLength + errorLength);

	// Write partial header
	EvaluationResultHeader *header = WriteDataHeader<EvaluationResultHeader>();
	header->dataType = DataType::Expression;
	header->expressionLength = expressionLength;
	header->intValue = errorLength;

	// Write expression and error message
	WriteData(expression, expressionLength);
	WriteData(errorMessage, errorLength);
	return true;
}

bool DataTransfer::WriteDoCode(GCodeChannel channel, const char *code, size_t length) noexcept
{
	if (!CanWritePacket(sizeof(DoCodeHeader) + length))
	{
		return false;
	}

	// Write packet header
	WritePacketHeader(FirmwareRequest::DoCode, sizeof(DoCodeHeader) + length);

	// Write header
	DoCodeHeader *header = WriteDataHeader<DoCodeHeader>();
	header->channel = channel.RawValue();
	header->length = length;

	// Write code
	WriteData(code, length);
	return true;
}

bool DataTransfer::WriteWaitForAcknowledgement(GCodeChannel channel) noexcept
{
	if (!CanWritePacket(sizeof(CodeChannelHeader)))
	{
		return false;
	}

	// Write packet header
	WritePacketHeader(FirmwareRequest::WaitForMessageAcknowledgment, sizeof(CodeChannelHeader));

	// Write header
	CodeChannelHeader *header = WriteDataHeader<CodeChannelHeader>();
	header->channel = channel.RawValue();
	header->paddingA = 0;
	header->paddingB = 0;
	return true;
}

bool DataTransfer::WriteMessageAcknowledged(GCodeChannel channel) noexcept
{
	if (!CanWritePacket(sizeof(CodeChannelHeader)))
	{
		return false;
	}

	// Write packet header
	WritePacketHeader(FirmwareRequest::MessageAcknowledged, sizeof(CodeChannelHeader));

	// Write header
	CodeChannelHeader *header = WriteDataHeader<CodeChannelHeader>();
	header->channel = channel.RawValue();
	header->paddingA = 0;
	header->paddingB = 0;
	return true;
}

bool DataTransfer::WriteSetVariableResult(const char *varName, const ExpressionValue& value) noexcept
{
	// Calculate payload length
	const size_t varNameLength = strlen(varName);
	size_t payloadLength;
	String<StringLength50> rslt;
	switch (value.GetType())
	{
	// FIXME Add support for arrays
	case TypeCode::Bool:
	case TypeCode::DriverId_tc:
	case TypeCode::Uint32:
	case TypeCode::Float:
	case TypeCode::Int32:
		payloadLength = varNameLength;
		break;
	case TypeCode::CString:
		payloadLength = varNameLength + strlen(value.sVal);
		break;
	case TypeCode::IPAddress_tc:
	case TypeCode::MacAddress_tc:
	case TypeCode::DateTime_tc:
		// All these types are represented as strings (FIXME: should we pass a DateTime over in raw format? Can DSF handle it?)
		value.AppendAsString(rslt.GetRef());
		payloadLength = varNameLength + rslt.strlen();
		break;
	case TypeCode::HeapString:
		payloadLength = varNameLength + value.shVal.GetLength();
		break;

	default:
		rslt.printf("unsupported type code %d", (int)value.type);
		payloadLength = varNameLength + rslt.strlen();
		break;
	}

	// Check if it fits
	if (!CanWritePacket(sizeof(EvaluationResultHeader) + payloadLength))
	{
		return false;
	}

	// Write packet header
	(void)WritePacketHeader(FirmwareRequest::VariableResult, sizeof(EvaluationResultHeader) + payloadLength);

	// Write partial header
	EvaluationResultHeader *header = WriteDataHeader<EvaluationResultHeader>();
	header->expressionLength = varNameLength;

	// Write variable name
	WriteData(varName, varNameLength);

	// Write data type and expression value
	switch (value.GetType())
	{
	case TypeCode::Bool:
		header->dataType = DataType::Bool;
		header->intValue = value.bVal ? 1 : 0;
		break;
	case TypeCode::CString:
		header->dataType = DataType::String;
		header->intValue = strlen(value.sVal);
		WriteData(value.sVal, header->intValue);
		break;
	case TypeCode::DriverId_tc:
		header->dataType = DataType::DriverId_dt;
		header->uintValue = value.uVal;
		break;
	case TypeCode::Uint32:
		header->dataType = DataType::UInt;
		header->uintValue = value.uVal;
		break;
	case TypeCode::Float:
		header->dataType = DataType::Float;
		header->floatValue = value.fVal;
		break;
	case TypeCode::Int32:
		header->dataType = DataType::Int;
		header->intValue = value.iVal;
		break;
	case TypeCode::HeapString:
		header->dataType = DataType::String;
		header->intValue = value.shVal.GetLength();
		WriteData(value.shVal.Get().Ptr(), header->intValue);
		break;
	case TypeCode::DateTime_tc:
	case TypeCode::MacAddress_tc:
	case TypeCode::IPAddress_tc:
	default:
		// We have already converted the value to a string in 'rslt'
		header->dataType = DataType::String;
		header->intValue = rslt.strlen();
		WriteData(rslt.c_str(), rslt.strlen());
		break;
	}
	return true;
}

bool DataTransfer::WriteSetVariableError(const char *varName, const char *errorMessage) noexcept
{
	// Check if it fits
	size_t varNameLength = strlen(varName), errorLength = strlen(errorMessage);
	if (!CanWritePacket(sizeof(EvaluationResultHeader) + varNameLength + errorLength))
	{
		return false;
	}

	// Write packet header
	(void)WritePacketHeader(FirmwareRequest::VariableResult, sizeof(EvaluationResultHeader) + varNameLength + errorLength);

	// Write partial header
	EvaluationResultHeader *header = WriteDataHeader<EvaluationResultHeader>();
	header->dataType = DataType::Expression;
	header->expressionLength = varNameLength;
	header->intValue = errorLength;

	// Write expression and error message
	WriteData(varName, varNameLength);
	WriteData(errorMessage, errorLength);
	return true;
}

bool DataTransfer::WriteCheckFileExists(const char *filename) noexcept
{
	// Check if it fits
	size_t filenameLength = strlen(filename);
	if (!CanWritePacket(sizeof(StringHeader) + filenameLength))
	{
		return false;
	}

	// Write packet header
	(void)WritePacketHeader(FirmwareRequest::CheckFileExists, sizeof(StringHeader) + filenameLength);

	// Write header
	StringHeader *header = WriteDataHeader<StringHeader>();
	header->length = filenameLength;
	header->padding = 0;

	// Write filename
	WriteData(filename, filenameLength);
	return true;
}

bool DataTransfer::WriteDeleteFileOrDirectory(const char *filename) noexcept
{
	// Check if it fits
	size_t filenameLength = strlen(filename);
	if (!CanWritePacket(sizeof(StringHeader) + filenameLength))
	{
		return false;
	}

	// Write packet header
	(void)WritePacketHeader(FirmwareRequest::DeleteFileOrDirectory, sizeof(StringHeader) + filenameLength);

	// Write header
	StringHeader *header = WriteDataHeader<StringHeader>();
	header->length = filenameLength;
	header->padding = 0;

	// Write filename
	WriteData(filename, filenameLength);
	return true;
}

bool DataTransfer::WriteOpenFile(const char *filename, bool forWriting, bool append, uint32_t preAllocSize) noexcept
{
	// Check if it fits
	size_t filenameLength = strlen(filename);
	if (!CanWritePacket(sizeof(OpenFileHeader) + filenameLength))
	{
		return false;
	}

	// Write packet header
	(void)WritePacketHeader(FirmwareRequest::OpenFile, sizeof(OpenFileHeader) + filenameLength);

	// Write partial header
	OpenFileHeader *header = WriteDataHeader<OpenFileHeader>();
	header->forWriting = forWriting;
	header->append = append;
	header->filenameLength = filenameLength;
	header->padding = 0;
	header->preAllocSize = preAllocSize;

	// Write expression and error message
	WriteData(filename, filenameLength);
	return true;
}

bool DataTransfer::WriteReadFile(FileHandle handle, size_t bufferSize) noexcept
{
	// Check if it fits
	if (!CanWritePacket(sizeof(ReadFileHeader)))
	{
		return false;
	}

	// Write packet header
	(void)WritePacketHeader(FirmwareRequest::ReadFile, sizeof(ReadFileHeader));

	// Write data header
	ReadFileHeader *header = WriteDataHeader<ReadFileHeader>();
	header->handle = handle;
	header->maxLength = min<uint32_t>(bufferSize, SbcTransferBufferSize - sizeof(FileDataHeader));
	return true;
}

bool DataTransfer::WriteFileData(FileHandle handle, const char *data, size_t& length) noexcept
{
	// Check if anything can be written
	size_t freeSpace = FreeTxSpace();
	if (freeSpace <= sizeof(FileHandleHeader))
	{
		return false;
	}
	size_t bytesToWrite = min<size_t>(length, freeSpace - sizeof(PacketHeader) - sizeof(FileHandleHeader));

	// Write packet header
	(void)WritePacketHeader(FirmwareRequest::WriteFile, bytesToWrite + sizeof(FileHandleHeader));

	// Write data header
	FileHandleHeader *header = WriteDataHeader<FileHandleHeader>();
	header->handle = handle;

	// Check how much can be written this time and write it
	WriteData(data, bytesToWrite);
	length -= bytesToWrite;
	txPointer = AddPadding(txPointer);
	return true;
}

bool DataTransfer::WriteSeekFile(FileHandle handle, FilePosition offset) noexcept
{
	// Check if it fits
	if (!CanWritePacket(sizeof(SeekFileHeader)))
	{
		return false;
	}

	// Write packet header
	(void)WritePacketHeader(FirmwareRequest::SeekFile, sizeof(SeekFileHeader));

	// Write data header
	SeekFileHeader *header = WriteDataHeader<SeekFileHeader>();
	header->handle = handle;
	header->offset = offset;
	return true;
}

bool DataTransfer::WriteTruncateFile(FileHandle handle) noexcept
{
	// Check if it fits
	if (!CanWritePacket(sizeof(FileHandleHeader)))
	{
		return false;
	}

	// Write packet header
	(void)WritePacketHeader(FirmwareRequest::TruncateFile, sizeof(FileHandleHeader));

	// Write data header
	FileHandleHeader *header = WriteDataHeader<FileHandleHeader>();
	header->handle = handle;
	return true;
}

bool DataTransfer::WriteCloseFile(FileHandle handle) noexcept
{
	// Check if it fits
	if (!CanWritePacket(sizeof(FileHandleHeader)))
	{
		return false;
	}

	// Write packet header
	(void)WritePacketHeader(FirmwareRequest::CloseFile, sizeof(FileHandleHeader));

	// Write data header
	FileHandleHeader *header = WriteDataHeader<FileHandleHeader>();
	header->handle = handle;
	return true;
}

PacketHeader *DataTransfer::WritePacketHeader(FirmwareRequest request, size_t dataLength, uint16_t resendPacketId) noexcept
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

void DataTransfer::WriteData(const char *data, size_t length) noexcept
{
	// Strings can be concatenated here, don't add any padding yet
	memcpy(txBuffer + txPointer, data, length);
	txPointer += length;
}

template<typename T> T *DataTransfer::WriteDataHeader() noexcept
{
	T *header = reinterpret_cast<T*>(txBuffer + txPointer);
	txPointer += sizeof(T);
	return header;
}

uint32_t DataTransfer::CalcCRC32(const char *buffer, size_t length) const noexcept
{
	CRC32 crc;
	crc.Update(buffer, length);
	return crc.Get();
}

#endif
