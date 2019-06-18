/*
 * CanSender.cpp
 *
 *  Created on: 20 Sep 2018
 *      Author: David
 */

#include "CanSender.h"

#if SUPPORT_CAN_EXPANSION

#include "CanMessageBuffer.h"
#include "Movement/StepTimer.h"
#include "RTOSIface/RTOSIface.h"

extern "C"
{
	#include "mcan/mcan.h"
	#include "pmc/pmc.h"
}

//#define CAN_DEBUG

Mcan* const MCAN_MODULE = MCAN1;
const IRQn MCanIRQn = MCAN1_INT0_IRQn;

// CanSender management task
constexpr size_t CanSenderTaskStackWords = 400;
static Task<CanSenderTaskStackWords> canSenderTask;

static CanMessageBuffer *pendingBuffers;
static CanMessageBuffer *lastBuffer;			// only valid when pendingBuffers != nullptr

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

// -------------------- The following code was adapted from the Atmel quick start example ---------------------------

/* mcan_filter_setting */
#define MCAN_RX_STANDARD_FILTER_INDEX_0    0
#define MCAN_RX_STANDARD_FILTER_INDEX_1    1
#define MCAN_RX_STANDARD_FILTER_ID_0     0x45A
#define MCAN_RX_STANDARD_FILTER_ID_0_BUFFER_INDEX     2
#define MCAN_RX_STANDARD_FILTER_ID_1     0x469
#define MCAN_RX_EXTENDED_FILTER_INDEX_0    0
#define MCAN_RX_EXTENDED_FILTER_INDEX_1    1
#define MCAN_RX_EXTENDED_FILTER_ID_0     0x100000A5
#define MCAN_RX_EXTENDED_FILTER_ID_0_BUFFER_INDEX     1
#define MCAN_RX_EXTENDED_FILTER_ID_1     0x10000096


/* mcan_transfer_message_setting */
#define MCAN_TX_BUFFER_INDEX    0

/* mcan_receive_message_setting */
static volatile uint32_t standard_receive_index = 0;
static volatile uint32_t extended_receive_index = 0;
static struct mcan_rx_element_fifo_0 rx_element_fifo_0;
static struct mcan_rx_element_fifo_1 rx_element_fifo_1;
static struct mcan_rx_element_buffer rx_element_buffer;

// MCAN module initialization.
static void configure_mcan()
{
	// Initialise the CAN hardware
//	mcan_stop(&mcan_instance);		//TEMP!!

	mcan_config config_mcan;
	mcan_get_config_defaults(&config_mcan);
	mcan_init(&mcan_instance, MCAN_MODULE, &config_mcan);
	mcan_enable_fd_mode(&mcan_instance);

	mcan_start(&mcan_instance);

	NVIC_ClearPendingIRQ(MCanIRQn);
	NVIC_SetPriority(MCanIRQn, NvicPriorityMCan);
	NVIC_EnableIRQ(MCanIRQn);
	mcan_enable_interrupt(&mcan_instance, (mcan_interrupt_source)(MCAN_FORMAT_ERROR | MCAN_ACKNOWLEDGE_ERROR | MCAN_BUS_OFF));
}

#if 0

// Set receive standard MCAN ID, dedicated buffer
static void mcan_set_standard_filter_0()
{
	struct mcan_standard_message_filter_element sd_filter;

	mcan_get_standard_message_filter_element_default(&sd_filter);
	sd_filter.S0.bit.SFID2 = MCAN_RX_STANDARD_FILTER_ID_0_BUFFER_INDEX;
	sd_filter.S0.bit.SFID1 = MCAN_RX_STANDARD_FILTER_ID_0;
	sd_filter.S0.bit.SFEC = MCAN_STANDARD_MESSAGE_FILTER_ELEMENT_S0_SFEC_STRXBUF_Val;

	mcan_set_rx_standard_filter(&mcan_instance, &sd_filter, MCAN_RX_STANDARD_FILTER_INDEX_0);
	mcan_enable_interrupt(&mcan_instance, MCAN_RX_BUFFER_NEW_MESSAGE);
}

// Set receive standard MCAN ID,FIFO buffer.
static void mcan_set_standard_filter_1()
{
	struct mcan_standard_message_filter_element sd_filter;

	mcan_get_standard_message_filter_element_default(&sd_filter);
	sd_filter.S0.bit.SFID1 = MCAN_RX_STANDARD_FILTER_ID_1;

	mcan_set_rx_standard_filter(&mcan_instance, &sd_filter, MCAN_RX_STANDARD_FILTER_INDEX_1);
	mcan_enable_interrupt(&mcan_instance, MCAN_RX_FIFO_0_NEW_MESSAGE);
}

// Set receive extended MCAN ID, dedicated buffer
static void mcan_set_extended_filter_0()
{
	struct mcan_extended_message_filter_element et_filter;

	mcan_get_extended_message_filter_element_default(&et_filter);
	et_filter.F0.bit.EFID1 = MCAN_RX_EXTENDED_FILTER_ID_0;
	et_filter.F0.bit.EFEC = MCAN_EXTENDED_MESSAGE_FILTER_ELEMENT_F0_EFEC_STRXBUF_Val;
	et_filter.F1.bit.EFID2 = MCAN_RX_EXTENDED_FILTER_ID_0_BUFFER_INDEX;

	mcan_set_rx_extended_filter(&mcan_instance, &et_filter, MCAN_RX_EXTENDED_FILTER_INDEX_0);
	mcan_enable_interrupt(&mcan_instance, MCAN_RX_BUFFER_NEW_MESSAGE);
}

// Set receive extended MCAN ID,FIFO buffer.
static void mcan_set_extended_filter_1()
{
	mcan_extended_message_filter_element et_filter;

	mcan_get_extended_message_filter_element_default(&et_filter);
	et_filter.F0.bit.EFID1 = MCAN_RX_EXTENDED_FILTER_ID_1;

	mcan_set_rx_extended_filter(&mcan_instance, &et_filter, MCAN_RX_EXTENDED_FILTER_INDEX_1);
	mcan_enable_interrupt(&mcan_instance, MCAN_RX_FIFO_1_NEW_MESSAGE);
}

// Send standard CAN message,
static status_code mcan_send_standard_message(uint32_t id_value, const uint8_t *data)
{
	struct mcan_tx_element tx_element;

	mcan_get_tx_buffer_element_defaults(&tx_element);
	tx_element.T0.reg |= MCAN_TX_ELEMENT_T0_STANDARD_ID(id_value);
	tx_element.T1.bit.DLC = 8;
	for (uint32_t i = 0; i < 8; i++)
	{
		tx_element.data[i] = *data;
		data++;
	}

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

#endif

// Send standard CAN message in fd mode,
static status_code mcan_fd_send_standard_message(uint32_t id_value, const uint8_t *data)
{
	struct mcan_tx_element tx_element;

	mcan_get_tx_buffer_element_defaults(&tx_element);
	tx_element.T0.reg |= MCAN_TX_ELEMENT_T0_STANDARD_ID(id_value);
	tx_element.T1.reg = (MCAN_TX_ELEMENT_T1_DLC(MCAN_TX_ELEMENT_T1_DLC_DATA64_Val) | MCAN_TX_ELEMENT_T1_FDF /*| MCAN_TX_ELEMENT_T1_BRS*/);
	for (uint32_t i = 0; i < CONF_MCAN_ELEMENT_DATA_SIZE; i++)
	{
		tx_element.data[i] = *data;
		data++;
	}

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

#if 0
// Send extended MCAN message,
static status_code mcan_fd_send_extended_message(uint32_t id_value, const uint8_t *data)
{
	struct mcan_tx_element tx_element;
	mcan_get_tx_buffer_element_defaults(&tx_element);
	tx_element.T0.reg |= MCAN_TX_ELEMENT_T0_EXTENDED_ID(id_value) | MCAN_TX_ELEMENT_T0_XTD;
	tx_element.T1.reg = (MCAN_TX_ELEMENT_T1_EFC | MCAN_TX_ELEMENT_T1_DLC(MCAN_TX_ELEMENT_T1_DLC_DATA64_Val) | MCAN_TX_ELEMENT_T1_FDF | MCAN_TX_ELEMENT_T1_BRS);
	for (uint32_t i = 0; i < CONF_MCAN_ELEMENT_DATA_SIZE; i++)
	{
		tx_element.data[i] = *data;
		data++;
	}

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
#endif

// Interrupt handler for MCAN, including RX,TX,ERROR and so on processes
void MCAN1_INT0_Handler(void)
{
	//TODO get rid of all these debugPrintf calls
	const uint32_t status = mcan_read_interrupt_status(&mcan_instance);
	if (status & MCAN_RX_BUFFER_NEW_MESSAGE)
	{
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

static_assert(CONF_MCAN_ELEMENT_DATA_SIZE == sizeof(CanMovementMessage), "Mismatched message sizes");

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

			// Send the message
			buf->msg.timeNow = StepTimer::GetInterruptClocks();
			mcan_fd_send_standard_message(buf->expansionBoardId | 0x0300, reinterpret_cast<uint8_t*>(&(buf->msg)));
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

void CanSender::Init()
{
	pendingBuffers = nullptr;

	ConfigurePin(g_APinDescription[APIN_CAN1_TX]);
	ConfigurePin(g_APinDescription[APIN_CAN1_RX]);
	pmc_enable_upll_clock();			// configure_mcan sets up PCLK5 to be the UPLL divided by something, so make sure the UPLL is running
	configure_mcan();

	// Create the task that sends CAN messages
	canSenderTask.Create(CanSenderLoop, "CanSender", nullptr, TaskPriority::CanSenderPriority);
}

// Add a buffer to the end of the send queue
void CanSender::Send(CanMessageBuffer *buf)
{
	buf->next = nullptr;
	TaskCriticalSectionLocker lock;

	if (pendingBuffers == nullptr)
	{
		pendingBuffers = lastBuffer = buf;
	}
	else
	{
		lastBuffer->next = buf;
	}
	canSenderTask.Give();
}

#endif

// End
