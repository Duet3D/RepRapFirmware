/*
 * CanInterface.cpp
 *
 *  Created on: 19 Sep 2018
 *      Author: David
 */

#include "CanInterface.h"

#if SUPPORT_CAN_EXPANSION

#include "CanMotion.h"
#include "CanMessageBuffer.h"
#include "Movement/DDA.h"
#include "Movement/DriveMovement.h"
#include "Movement/StepTimer.h"
#include "RTOSIface/RTOSIface.h"

extern "C"
{
	#include "mcan/mcan.h"
	#include "pmc/pmc.h"
}

const unsigned int NumCanBuffers = 40;

static CanAddress boardAddress;

#define USE_BIT_RATE_SWITCH		0

//#define CAN_DEBUG

Mcan* const MCAN_MODULE = MCAN1;
constexpr IRQn MCanIRQn = MCAN1_INT0_IRQn;

static mcan_module mcan_instance;

static volatile uint32_t canStatus = 0;

enum class CanStatusBits : uint32_t
{
	receivedStandardFDMessage = 1,
	receivedExtendedFDMessage = 2,
	receivedStandardFDMessageInFIFO = 4,
	receivedStandardNormalMessageInFIFO0 = 8,
	receivedStandardFDMessageInFIFO0 = 16,
	receivedExtendedFDMessageInFIFO1 = 32,
	acknowledgeError = 0x10000,
	formatError = 0x20000,
	busOff = 0x40000
};

#ifdef CAN_DEBUG
static uint32_t GetAndClearStatusBits()
{
	const uint32_t st = canStatus;
	canStatus = 0;
	return st;
}
#endif

/* mcan_transfer_message_setting */
#define MCAN_TX_BUFFER_INDEX    0

/* mcan_receive_message_setting */
static volatile uint32_t standard_receive_index = 0;
static volatile uint32_t extended_receive_index = 0;
static struct mcan_rx_element_fifo_0 rx_element_fifo_0;
static struct mcan_rx_element_fifo_1 rx_element_fifo_1;
static struct mcan_rx_element_buffer rx_element_buffer;

constexpr uint32_t CanClockIntervalMillis = 20; //10000; //20;

// CanSender management task
constexpr size_t CanSenderTaskStackWords = 400;
static Task<CanSenderTaskStackWords> canSenderTask;

constexpr size_t CanClockTaskStackWords = 300;
static Task<CanSenderTaskStackWords> canClockTask;

static CanMessageBuffer * volatile pendingBuffers;
static CanMessageBuffer * volatile lastBuffer;			// only valid when pendingBuffers != nullptr

// MCAN module initialization.
static void configure_mcan()
{
	// Initialise the CAN hardware
	mcan_config config_mcan;
	mcan_get_config_defaults(&config_mcan);
	mcan_init(&mcan_instance, MCAN_MODULE, &config_mcan);
	mcan_enable_fd_mode(&mcan_instance);

	mcan_extended_message_filter_element et_filter;

	// Set up a filter to receive all request messages addressed to us in FIFO 0
	mcan_get_extended_message_filter_element_default(&et_filter);
	et_filter.F0.bit.EFID1 = (CanId::MasterAddress << CanId::DstAddressShift);
	et_filter.F0.bit.EFEC = MCAN_EXTENDED_MESSAGE_FILTER_ELEMENT_F0_EFEC_STF0M_Val;
	et_filter.F1.bit.EFID2 = (CanId::BoardAddressMask << CanId::DstAddressShift) | CanId::ResponseBit;
	et_filter.F1.bit.EFT = 2;
	mcan_set_rx_extended_filter(&mcan_instance, &et_filter, 0);

	// Set up a filter to receive all broadcast messages also in FIFO 0 (does this include broadcasts that we send?)
	mcan_get_extended_message_filter_element_default(&et_filter);
	et_filter.F0.bit.EFID1 = CanId::BroadcastAddress << CanId::DstAddressShift;
	et_filter.F0.bit.EFEC = MCAN_EXTENDED_MESSAGE_FILTER_ELEMENT_F0_EFEC_STF0M_Val;
	et_filter.F1.bit.EFID2 = (CanId::BoardAddressMask << CanId::DstAddressShift);
	et_filter.F1.bit.EFT = 2;
	mcan_set_rx_extended_filter(&mcan_instance, &et_filter, 1);

	// Set up a filter to receive response messages in FIFO 1
	mcan_get_extended_message_filter_element_default(&et_filter);
	et_filter.F0.bit.EFID1 = (CanId::MasterAddress << CanId::DstAddressShift) | CanId::ResponseBit;
	et_filter.F0.bit.EFEC = MCAN_EXTENDED_MESSAGE_FILTER_ELEMENT_F0_EFEC_STF1M_Val;
	et_filter.F1.bit.EFID2 = (CanId::BoardAddressMask << CanId::DstAddressShift) | CanId::ResponseBit;
	et_filter.F1.bit.EFT = 2;
	mcan_set_rx_extended_filter(&mcan_instance, &et_filter, 2);

	mcan_enable_interrupt(&mcan_instance, (mcan_interrupt_source)(MCAN_FORMAT_ERROR | MCAN_ACKNOWLEDGE_ERROR | MCAN_BUS_OFF | MCAN_RX_FIFO_0_NEW_MESSAGE | MCAN_RX_FIFO_1_NEW_MESSAGE));
	NVIC_ClearPendingIRQ(MCanIRQn);
	NVIC_SetPriority(MCanIRQn, NvicPriorityMCan);
	NVIC_EnableIRQ(MCanIRQn);

	mcan_start(&mcan_instance);
}

extern "C" void CanSenderLoop(void *);
extern "C" void CanClockLoop(void *);

void CanInterface::Init(CanAddress pBoardAddress)
{
	boardAddress = pBoardAddress;

	CanMessageBuffer::Init(NumCanBuffers);
	pendingBuffers = nullptr;

	ConfigurePin(g_APinDescription[APIN_CAN1_TX]);
	ConfigurePin(g_APinDescription[APIN_CAN1_RX]);
	pmc_enable_upll_clock();			// configure_mcan sets up PCLK5 to be the UPLL divided by something, so make sure the UPLL is running
	configure_mcan();

	CanMotion::Init();

	// Create the task that sends CAN messages
	canSenderTask.Create(CanSenderLoop, "CanSender", nullptr, TaskPriority::CanSenderPriority);
	canClockTask.Create(CanClockLoop, "CanClock", nullptr, TaskPriority::CanClockPriority);
}

CanAddress CanInterface::GetCanAddress()
{
	return boardAddress;
}

// Send extended CAN message in fd mode,
static status_code mcan_fd_send_ext_message(uint32_t id_value, const uint8_t *data, size_t dataLength)
{
	const uint32_t dlc = (dataLength <= 8) ? dataLength
							: (dataLength <= 24) ? ((dataLength + 3) >> 2) + 6
								: ((dataLength + 15) >> 4) + 11;
	mcan_tx_element tx_element;
	tx_element.T0.reg = MCAN_TX_ELEMENT_T0_EXTENDED_ID(id_value) | MCAN_TX_ELEMENT_T0_XTD;
	tx_element.T1.reg = MCAN_TX_ELEMENT_T1_DLC(dlc)
						| MCAN_TX_ELEMENT_T1_EFC
#if USE_BIT_RATE_SWITCH
						| MCAN_TX_ELEMENT_T1_BRS
#endif
						| MCAN_TX_ELEMENT_T1_FDF;

	memcpy(tx_element.data, data, dataLength);

	status_code rc = mcan_set_tx_buffer_element(&mcan_instance, &tx_element, MCAN_TX_BUFFER_INDEX);
	if (rc != STATUS_OK)
	{
		DEBUG_HERE;
	}
	else
	{
		rc = mcan_tx_transfer_request(&mcan_instance, 1 << MCAN_TX_BUFFER_INDEX);
		if (rc != STATUS_OK)
		{
			DEBUG_HERE;
		}
	}
	return rc;
}

// Interrupt handler for MCAN, including RX,TX,ERROR and so on processes
void MCAN1_INT0_Handler(void)
{
	const uint32_t status = mcan_read_interrupt_status(&mcan_instance);

	if (status & MCAN_RX_BUFFER_NEW_MESSAGE)
	{
#if 1
		// We don't enable this interrupt, so it should never happen
		mcan_clear_interrupt_status(&mcan_instance, MCAN_RX_BUFFER_NEW_MESSAGE);
#else
		mcan_clear_interrupt_status(&mcan_instance, MCAN_RX_BUFFER_NEW_MESSAGE);
		for (unsigned int i = 0; i < CONF_MCAN1_RX_BUFFER_NUM; i++)
		{
			if (mcan_rx_get_buffer_status(&mcan_instance, i))
			{
				const uint32_t rx_buffer_index = i;
				mcan_rx_clear_buffer_status(&mcan_instance, i);
				mcan_get_rx_buffer_element(&mcan_instance, &rx_element_buffer, rx_buffer_index);
				if (rx_element_buffer.R0.bit.XTD)
				{
					canStatus |= (uint32_t)CanStatusBits::receivedExtendedFDMessage;
				}
				else
				{
					canStatus |= (uint32_t)CanStatusBits::receivedStandardFDMessage;
				}
			}
		}
#endif
	}

	if (status & MCAN_RX_FIFO_0_NEW_MESSAGE)
	{
		mcan_clear_interrupt_status(&mcan_instance, MCAN_RX_FIFO_0_NEW_MESSAGE);
		mcan_get_rx_fifo_0_element(&mcan_instance, &rx_element_fifo_0, standard_receive_index);
		mcan_rx_fifo_acknowledge(&mcan_instance, 0, standard_receive_index);
		standard_receive_index++;
		if (standard_receive_index == CONF_MCAN1_RX_FIFO_0_NUM)
		{
			standard_receive_index = 0;
		}

		if (rx_element_fifo_0.R1.bit.EDL)
		{
			canStatus |= (uint32_t)CanStatusBits::receivedStandardFDMessageInFIFO0;
		}
		else
		{
			canStatus |= (uint32_t)CanStatusBits::receivedStandardNormalMessageInFIFO0;
		}
	}

	if (status & MCAN_RX_FIFO_1_NEW_MESSAGE)
	{
		mcan_clear_interrupt_status(&mcan_instance, MCAN_RX_FIFO_1_NEW_MESSAGE);
		mcan_get_rx_fifo_1_element(&mcan_instance, &rx_element_fifo_1, extended_receive_index);
		mcan_rx_fifo_acknowledge(&mcan_instance, 0, extended_receive_index);
		extended_receive_index++;
		if (extended_receive_index == CONF_MCAN1_RX_FIFO_1_NUM)
		{
			extended_receive_index = 0;
		}

		canStatus |= (uint32_t)CanStatusBits::receivedExtendedFDMessageInFIFO1;
	}

	if ((status & MCAN_ACKNOWLEDGE_ERROR))
	{
		mcan_clear_interrupt_status(&mcan_instance, (mcan_interrupt_source)(MCAN_ACKNOWLEDGE_ERROR));
		canStatus |= (uint32_t)CanStatusBits::acknowledgeError;
	}

	if ((status & MCAN_FORMAT_ERROR))
	{
		mcan_clear_interrupt_status(&mcan_instance, (mcan_interrupt_source)(MCAN_FORMAT_ERROR));
		canStatus |= (uint32_t)CanStatusBits::formatError;
	}

	if (status & MCAN_BUS_OFF)
	{
		mcan_clear_interrupt_status(&mcan_instance, MCAN_BUS_OFF);
		mcan_stop(&mcan_instance);
		canStatus |= (uint32_t)CanStatusBits::busOff;
		configure_mcan();
	}
}

// -------------------- End of code adapted from Atmel quick start example ----------------------------------

static_assert(CONF_MCAN_ELEMENT_DATA_SIZE == sizeof(CanMessage), "Mismatched message sizes");

extern "C" void CanSenderLoop(void *)
{
	for (;;)
	{
		TaskBase::Take(Mutex::TimeoutUnlimited);
		while (pendingBuffers != nullptr)
		{
			CanMessageBuffer *buf;
			{
				TaskCriticalSectionLocker lock;
				buf = pendingBuffers;
				pendingBuffers = buf->next;
			}

			// Send the message. If it is a time sync message, fill in the sending time first.
			if (buf->isTimeSyncMessage)
			{
				buf->msg.sync.timeSent = StepTimer::GetInterruptClocks();
			}
			mcan_fd_send_ext_message(buf->id.GetWholeId(), reinterpret_cast<uint8_t*>(&(buf->msg)), buf->dataLength);

#ifdef CAN_DEBUG
			// Display a debug message too
			debugPrintf("CCCR %08" PRIx32 ", PSR %08" PRIx32 ", ECR %08" PRIx32 ", TXBRP %08" PRIx32 ", TXBTO %08" PRIx32 ", st %08" PRIx32 "\n",
						MCAN1->MCAN_CCCR, MCAN1->MCAN_PSR, MCAN1->MCAN_ECR, MCAN1->MCAN_TXBRP, MCAN1->MCAN_TXBTO, GetAndClearStatusBits());
			buf->msg.DebugPrint();
			delay(50);
			debugPrintf("CCCR %08" PRIx32 ", PSR %08" PRIx32 ", ECR %08" PRIx32 ", TXBRP %08" PRIx32 ", TXBTO %08" PRIx32 ", st %08" PRIx32 "\n",
						MCAN1->MCAN_CCCR, MCAN1->MCAN_PSR, MCAN1->MCAN_ECR, MCAN1->MCAN_TXBRP, MCAN1->MCAN_TXBTO, GetAndClearStatusBits());
#else
			delay(2);		// until we have the transmit fifo working, we need to delay to allow the message to be sent
#endif
			// Free the message buffer.
			CanMessageBuffer::Free(buf);
		}
	}
}

extern "C" void CanClockLoop(void *)
{
	uint32_t lastWakeTime = xTaskGetTickCount();

	for (;;)
	{
		CanMessageBuffer * const buf = CanMessageBuffer::Allocate();
		if (buf != nullptr)
		{
			(void)buf->SetupBroadcastMessage<CanMessageTimeSync>(CanId::MasterAddress, true);
			// The timeSent field is filled in when the message is actually written to the fifo
			CanInterface::Send(buf);
		}
		// Delay until it is time again
		vTaskDelayUntil(&lastWakeTime, CanClockIntervalMillis);
	}
}

// Add a buffer to the end of the send queue
void CanInterface::Send(CanMessageBuffer *buf)
{
	buf->next = nullptr;
	TaskCriticalSectionLocker lock;

	if (pendingBuffers == nullptr)
	{
		pendingBuffers = buf;
	}
	else
	{
		lastBuffer->next = buf;
	}
	lastBuffer = buf;
	canSenderTask.Give();
}

#endif

// End
