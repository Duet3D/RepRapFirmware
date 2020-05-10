/*
 * CanInterface.cpp
 *
 *  Created on: 19 Sep 2018
 *      Author: David
 */

#include "CanInterface.h"

#if SUPPORT_CAN_EXPANSION

#include "CanMotion.h"
#include "CommandProcessor.h"
#include "CanMessageGenericConstructor.h"
#include <CanMessageBuffer.h>
#include <Movement/DDA.h>
#include <Movement/DriveMovement.h>
#include <Movement/StepTimer.h>
#include <RTOSIface/RTOSIface.h>
#include <TaskPriorities.h>
#include <GCodes/GCodeException.h>
#include <GCodes/GCodeBuffer/GCodeBuffer.h>
#include <Hardware/CanDriver.h>

#if HAS_LINUX_INTERFACE
# include "Linux/LinuxInterface.h"
#endif

#include <memory>

const unsigned int NumCanBuffers = 2 * MaxCanBoards + 10;

constexpr uint32_t MaxMotionSendWait = 20;		// milliseconds
constexpr uint32_t MaxUrgentSendWait = 20;		// milliseconds
constexpr uint32_t MaxTimeSyncSendWait = 20;	// milliseconds
constexpr uint32_t MaxResponseSendWait = 50;	// milliseconds
constexpr uint32_t MaxRequestSendWait = 50;		// milliseconds

#define USE_BIT_RATE_SWITCH		0

constexpr uint32_t MinBitRate = 15;				// MCP2542 has a minimum bite rate of 14.4kbps
constexpr uint32_t MaxBitRate = 5000;

constexpr float MinSamplePoint = 0.5;
constexpr float MaxSamplePoint = 0.95;
constexpr float DefaultSamplePoint = 0.75;

constexpr float MinJumpWidth = 0.05;
constexpr float MaxJumpWidth = 0.5;
constexpr float DefaultJumpWidth = 0.25;

//#define CAN_DEBUG

#ifdef USE_CAN0

Mcan* const MCAN_MODULE = MCAN0;
constexpr IRQn MCanIRQn = MCAN0_INT0_IRQn;
#define MCAN_INT0_Handler	MCAN0_INT0_Handler

#else

Mcan* const MCAN_MODULE = MCAN1;
constexpr IRQn MCanIRQn = MCAN1_INT0_IRQn;
#define MCAN_INT0_Handler	MCAN1_INT0_Handler

#endif

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
static uint32_t GetAndClearStatusBits() noexcept
{
	const uint32_t st = canStatus;
	canStatus = 0;
	return st;
}
#endif

/* mcan_transfer_message_setting */
constexpr uint32_t TxBufferIndexUrgent = 0;
constexpr uint32_t TxBufferIndexTimeSync = 1;
constexpr uint32_t TxBufferIndexMotion = 2;
// We should probably use a FIFO or a queue for the remainder, but for now each has its own message buffer
constexpr uint32_t TxBufferIndexRequest = 3;
constexpr uint32_t TxBufferIndexResponse = 4;
constexpr uint32_t TxBufferBroadcast = 5;

/* mcan_receive_message_setting */
constexpr uint32_t RxFifoIndexBroadcast = 0;
constexpr uint32_t RxFifoIndexRequest = 0;
constexpr uint32_t RxFifoIndexResponse = 1;

constexpr uint32_t CanClockIntervalMillis = 200;

// CanSender management task
constexpr size_t CanSenderTaskStackWords = 400;
static Task<CanSenderTaskStackWords> canSenderTask;

constexpr size_t CanReceiverTaskStackWords = 1000;
static Task<CanReceiverTaskStackWords> canReceiverTask;

constexpr size_t CanClockTaskStackWords = 300;
static Task<CanSenderTaskStackWords> canClockTask;

static CanMessageBuffer * volatile pendingBuffers;
static CanMessageBuffer * volatile lastBuffer;			// only valid when pendingBuffers != nullptr

static TaskHandle taskWaitingOnFifo0 = nullptr;
static TaskHandle taskWaitingOnFifo1 = nullptr;

static uint32_t messagesSent = 0;
static uint32_t longestWaitTime = 0;
static uint16_t longestWaitMessageType = 0;

// MCAN module initialization.
static void configure_mcan() noexcept
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
	et_filter.F0.bit.EFEC = MCAN_EXTENDED_MESSAGE_FILTER_ELEMENT_F0_EFEC_STF0M_Val;		// RxFifoIndexRequest
	et_filter.F1.bit.EFID2 = (CanId::BoardAddressMask << CanId::DstAddressShift) | CanId::ResponseBit;
	et_filter.F1.bit.EFT = 2;
	mcan_set_rx_extended_filter(&mcan_instance, &et_filter, 0);

	// Set up a filter to receive all broadcast messages also in FIFO 0 (does this include broadcasts that we send?)
	mcan_get_extended_message_filter_element_default(&et_filter);
	et_filter.F0.bit.EFID1 = CanId::BroadcastAddress << CanId::DstAddressShift;
	et_filter.F0.bit.EFEC = MCAN_EXTENDED_MESSAGE_FILTER_ELEMENT_F0_EFEC_STF0M_Val;		// RxFifoIndexBroadcast
	et_filter.F1.bit.EFID2 = (CanId::BoardAddressMask << CanId::DstAddressShift);
	et_filter.F1.bit.EFT = 2;
	mcan_set_rx_extended_filter(&mcan_instance, &et_filter, 1);

	// Set up a filter to receive response messages in FIFO 1
	mcan_get_extended_message_filter_element_default(&et_filter);
	et_filter.F0.bit.EFID1 = (CanId::MasterAddress << CanId::DstAddressShift) | CanId::ResponseBit;
	et_filter.F0.bit.EFEC = MCAN_EXTENDED_MESSAGE_FILTER_ELEMENT_F0_EFEC_STF1M_Val;		// RxFifoIndexResponse
	et_filter.F1.bit.EFID2 = (CanId::BoardAddressMask << CanId::DstAddressShift) | CanId::ResponseBit;
	et_filter.F1.bit.EFT = 2;
	mcan_set_rx_extended_filter(&mcan_instance, &et_filter, 2);

	mcan_enable_interrupt(&mcan_instance, (mcan_interrupt_source)(MCAN_FORMAT_ERROR | MCAN_ACKNOWLEDGE_ERROR | MCAN_BUS_OFF | MCAN_RX_FIFO_0_NEW_MESSAGE | MCAN_RX_FIFO_1_NEW_MESSAGE));
	NVIC_ClearPendingIRQ(MCanIRQn);
	NVIC_SetPriority(MCanIRQn, NvicPriorityMCan);
	NVIC_EnableIRQ(MCanIRQn);

	mcan_start(&mcan_instance);
}

static void GetLocalCanTiming(CanTiming& timing) noexcept
{
	const uint32_t nbtp = MCAN_MODULE->MCAN_NBTP;
	const uint32_t tseg1 = (nbtp & MCAN_NBTP_NTSEG1_Msk) >> MCAN_NBTP_NTSEG1_Pos;
	const uint32_t tseg2 = (nbtp & MCAN_NBTP_NTSEG2_Msk) >> MCAN_NBTP_NTSEG2_Pos;
	const uint32_t jw = (nbtp & MCAN_NBTP_NSJW_Msk) >> MCAN_NBTP_NSJW_Pos;
	const uint32_t brp = (nbtp & MCAN_NBTP_NBRP_Msk) >> MCAN_NBTP_NBRP_Pos;
	timing.period = (tseg1 + tseg2 + 3) * (brp + 1);
	timing.tseg1 = (tseg1 + 1) * (brp + 1);
	timing.jumpWidth = (jw + 1) * (brp + 1);
}

static void ChangeLocalCanTiming(const CanTiming& timing) noexcept
{
	// Sort out the bit timing
	uint32_t period = timing.period;
	uint32_t tseg1 = timing.tseg1;
	uint32_t jumpWidth = timing.jumpWidth;
	uint32_t prescaler = 1;						// 48MHz main clock
	uint32_t tseg2;

	for (;;)
	{
		tseg2 = period - tseg1 - 1;
		if (tseg1 <= 32 && tseg2 <= 16 && jumpWidth <= 16)
		{
			break;
		}
		prescaler <<= 1;
		period >>= 1;
		tseg1 >>= 1;
		jumpWidth >>= 1;
	}

	//TODO stop transmissions in an orderly fashion, or postpone initialising CAN until we have the timing data
	MCAN_MODULE->MCAN_CCCR |= MCAN_CCCR_CCE | MCAN_CCCR_INIT;

	MCAN_MODULE->MCAN_NBTP =  ((tseg1 - 1) << MCAN_NBTP_NTSEG1_Pos)
							| ((tseg2 - 1) << MCAN_NBTP_NTSEG2_Pos)
							| ((jumpWidth - 1) << MCAN_NBTP_NSJW_Pos)
							| ((prescaler - 1) << MCAN_NBTP_NBRP_Pos);

	MCAN_MODULE->MCAN_CCCR &= ~MCAN_CCCR_CCE;
}

extern "C" [[noreturn]] void CanSenderLoop(void *) noexcept;
extern "C" [[noreturn]] void CanClockLoop(void *) noexcept;
extern "C" [[noreturn]] void CanReceiverLoop(void *) noexcept;

void CanInterface::Init() noexcept
{
	CanMessageBuffer::Init(NumCanBuffers);
	pendingBuffers = nullptr;

#ifdef USE_CAN0
	ConfigurePin(APIN_CAN0_TX);
	ConfigurePin(APIN_CAN0_RX);
#else
	ConfigurePin(APIN_CAN1_TX);
	ConfigurePin(APIN_CAN1_RX);
#endif
	pmc_enable_upll_clock();			// configure_mcan sets up PCLK5 to be the UPLL divided by something, so make sure the UPLL is running
	configure_mcan();

	CanMotion::Init();

	// Create the task that sends CAN messages
	canClockTask.Create(CanClockLoop, "CanClock", nullptr, TaskPriority::CanClockPriority);
	canSenderTask.Create(CanSenderLoop, "CanSender", nullptr, TaskPriority::CanSenderPriority);
	canReceiverTask.Create(CanReceiverLoop, "CanReceiver", nullptr, TaskPriority::CanReceiverPriority);
}

// Allocate a CAN request ID
CanRequestId CanInterface::AllocateRequestId(CanAddress destination) noexcept
{
	// We probably want to have special request IDs to tell the destination to resync. But for now just increment the ID. Reserve the top bit for future use.
	static uint16_t rid = 0;

	CanRequestId rslt = rid & 0x07FF;
	++rid;
	return rslt;
}

// Allocate a CAN message buffer, throw if failed
CanMessageBuffer *CanInterface::AllocateBuffer(const GCodeBuffer& gb) THROWS(GCodeException)
{
	CanMessageBuffer * const buf = CanMessageBuffer::Allocate();
	if (buf == nullptr)
	{
		throw GCodeException(gb.GetLineNumber(), -1, "no CAN buffer");
	}
	return buf;
}

void CanInterface::CheckCanAddress(uint32_t address, const GCodeBuffer& gb) THROWS(GCodeException)
{
	if (address == 0 || address > CanId::MaxCanAddress)
	{
		throw GCodeException(gb.GetLineNumber(), -1, "CAN address out of range");
	}
}

// Wait for a specified buffer to become free. If it's still not free after the timeout, cancel the pending transmission.
static void WaitForTxBufferFree(uint32_t whichTxBuffer, uint32_t maxWait) noexcept
{
	const uint32_t trigMask = (uint32_t)1 << whichTxBuffer;
	if ((mcan_instance.hw->MCAN_TXBRP & trigMask) != 0)
	{
		// Wait for the timeout period for the message to be sent
		const uint32_t startTime = millis();
		do
		{
			delay(1);
			if ((mcan_instance.hw->MCAN_TXBRP & trigMask) == 0)
			{
				return;
			}
		} while (millis() - startTime < maxWait);

		// The last message still hasn't been sent, so cancel it
		mcan_instance.hw->MCAN_TXBCR = trigMask;
		while ((mcan_instance.hw->MCAN_TXBRP & trigMask) != 0)
		{
			delay(1);
		}
	}
}

// Send extended CAN message in fd mode. The Tx buffer must alrrady be free.
static status_code mcan_fd_send_ext_message_no_wait(uint32_t id_value, const uint8_t *data, size_t dataLength, uint32_t whichTxBuffer) noexcept
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

	status_code rc = mcan_set_tx_buffer_element(&mcan_instance, &tx_element, whichTxBuffer);
	if (rc == STATUS_OK)
	{
		rc = mcan_tx_transfer_request(&mcan_instance, (uint32_t)1 << whichTxBuffer);
	}
	return rc;
}

// Send extended CAN message in fd mode
static status_code mcan_fd_send_ext_message(uint32_t id_value, const uint8_t *data, size_t dataLength, uint32_t whichTxBuffer, uint32_t maxWait) noexcept
{
	WaitForTxBufferFree(whichTxBuffer, maxWait);
	++messagesSent;
	return mcan_fd_send_ext_message_no_wait(id_value, data, dataLength, whichTxBuffer);
}

// Interrupt handler for MCAN, including RX,TX,ERROR and so on processes
extern "C" void MCAN_INT0_Handler() noexcept
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
		if (taskWaitingOnFifo0 != nullptr)
		{
			TaskBase::GiveFromISR(taskWaitingOnFifo0);
		}
	}

	if (status & MCAN_RX_FIFO_1_NEW_MESSAGE)
	{
		mcan_clear_interrupt_status(&mcan_instance, MCAN_RX_FIFO_1_NEW_MESSAGE);
		if (taskWaitingOnFifo1 != nullptr)
		{
			TaskBase::GiveFromISR(taskWaitingOnFifo1);
		}
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

//TODO can we get rid of the CanSender task if we send movement messages via the Tx FIFO?
// This task picks up motion messages and sends them
extern "C" [[noreturn]] void CanSenderLoop(void *) noexcept
{
	for (;;)
	{
		TaskBase::Take(Mutex::TimeoutUnlimited);
		for (;;)
		{
			CanMessageBuffer * const urgentMessage = CanMotion::GetUrgentMessage();
			if (urgentMessage != nullptr)
			{
				mcan_fd_send_ext_message(urgentMessage->id.GetWholeId(), reinterpret_cast<uint8_t*>(&(urgentMessage->msg)), urgentMessage->dataLength, TxBufferIndexUrgent, MaxUrgentSendWait);
			}
			else if (pendingBuffers != nullptr)
			{
				CanMessageBuffer *buf;
				{
					TaskCriticalSectionLocker lock;
					buf = pendingBuffers;
					pendingBuffers = buf->next;
				}

#if 0
				buf->msg.move.DebugPrint();
#endif
				// Send the message
				mcan_fd_send_ext_message(buf->id.GetWholeId(), reinterpret_cast<uint8_t*>(&(buf->msg)), buf->dataLength,
											TxBufferIndexMotion, MaxMotionSendWait);

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
			else
			{
				break;
			}
		}
	}
}

extern "C" [[noreturn]] void CanClockLoop(void *) noexcept
{
	uint32_t lastWakeTime = xTaskGetTickCount();

	for (;;)
	{
		CanMessageBuffer * buf = CanMessageBuffer::Allocate();
		if (buf != nullptr)
		{
			CanMessageTimeSync * const msg = buf->SetupBroadcastMessage<CanMessageTimeSync>(CanId::MasterAddress);
			WaitForTxBufferFree(TxBufferIndexTimeSync, MaxTimeSyncSendWait);			// make sure we can send immediately
			msg->timeSent = StepTimer::GetTimerTicks();
			mcan_fd_send_ext_message_no_wait(buf->id.GetWholeId(), reinterpret_cast<uint8_t*>(&(buf->msg)), buf->dataLength, TxBufferIndexTimeSync);
			CanMessageBuffer::Free(buf);
		}
		// Delay until it is time again
		vTaskDelayUntil(&lastWakeTime, CanClockIntervalMillis);
	}
}

// Members of template class CanDriversData
CanDriversData::CanDriversData() noexcept
{
	numEntries = 0;
}

// Insert a new entry, keeping the list ordered
void CanDriversData::AddEntry(DriverId driver, uint16_t val) noexcept
{
	if (numEntries < ARRAY_SIZE(data))
	{
		// We could do a binary search here but the number of CAN drivers supported isn't huge, so linear search instead
		size_t insertPoint = 0;
		while (insertPoint < numEntries && data[insertPoint].driver < driver)
		{
			++insertPoint;
		}
		memmove(data + (insertPoint + 1), data + insertPoint, (numEntries - insertPoint) * sizeof(data[0]));
		data[insertPoint].driver = driver;
		data[insertPoint].val = val;
		++numEntries;
	}
}

// Get the details of the drivers on the next board and advance startFrom beyond the entries for this board
CanAddress CanDriversData::GetNextBoardDriverBitmap(size_t& startFrom, CanDriversBitmap& driversBitmap) const noexcept
{
	driversBitmap.Clear();
	if (startFrom >= numEntries)
	{
		return CanId::NoAddress;
	}
	const CanAddress boardAddress = data[startFrom].driver.boardAddress;
	do
	{
		driversBitmap.SetBit(data[startFrom].driver.localDriver);
		++startFrom;
	} while (startFrom < numEntries && data[startFrom].driver.boardAddress == boardAddress);
	return boardAddress;
}

// Insert a new entry, keeping the list ordered
void CanDriversList::AddEntry(DriverId driver) noexcept
{
	if (numEntries < ARRAY_SIZE(drivers))
	{
		// We could do a binary search here but the number of CAN drivers supported isn't huge, so linear search instead
		size_t insertPoint = 0;
		while (insertPoint < numEntries && drivers[insertPoint] < driver)
		{
			++insertPoint;
		}

		if (insertPoint == numEntries)
		{
			drivers[numEntries] = driver;
			++numEntries;
		}
		else if (drivers[insertPoint] != driver)
		{
			memmove(drivers + (insertPoint + 1), drivers + insertPoint, (numEntries - insertPoint) * sizeof(drivers[0]));
			drivers[insertPoint] = driver;
			++numEntries;
		}
	}
}

// Get the details of the drivers on the next board and advance startFrom beyond the entries for this board
CanAddress CanDriversList::GetNextBoardDriverBitmap(size_t& startFrom, CanDriversBitmap& driversBitmap) const noexcept
{
	driversBitmap.Clear();
	if (startFrom >= numEntries)
	{
		return CanId::NoAddress;
	}
	const CanAddress boardAddress = drivers[startFrom].boardAddress;
	do
	{
		driversBitmap.SetBit(drivers[startFrom].localDriver);
		++startFrom;
	} while (startFrom < numEntries && drivers[startFrom].boardAddress == boardAddress);
	return boardAddress;
}

// Members of namespace CanInterface, and associated local functions

static bool SetRemoteDriverValues(const CanDriversData& data, const StringRef& reply, CanMessageType mt) noexcept
{
	bool ok = true;
	size_t start = 0;
	for (;;)
	{
		CanDriversBitmap driverBits;
		size_t savedStart = start;
		const CanAddress boardAddress = data.GetNextBoardDriverBitmap(start, driverBits);
		if (boardAddress == CanId::NoAddress)
		{
			break;
		}
		CanMessageBuffer * const buf = CanMessageBuffer::Allocate();
		if (buf == nullptr)
		{
			reply.lcat("No CAN buffer available");
			return false;
		}
		const CanRequestId rid = CanInterface::AllocateRequestId(boardAddress);
		CanMessageMultipleDrivesRequest * const msg = buf->SetupRequestMessage<CanMessageMultipleDrivesRequest>(rid, CanId::MasterAddress, boardAddress, mt);
		msg->driversToUpdate = driverBits.GetRaw();
		size_t numDrivers = 0;
		while (savedStart < start && numDrivers < ARRAY_SIZE(msg->values))
		{
			msg->values[numDrivers] = data.GetElement(savedStart);
			++savedStart;
			++numDrivers;
		}
		buf->dataLength = msg->GetActualDataLength(numDrivers);
		if (CanInterface::SendRequestAndGetStandardReply(buf, rid, reply) != GCodeResult::ok)
		{
			ok = false;
		}
	}
	return ok;
}

static bool SetRemoteDriverStates(const CanDriversList& drivers, const StringRef& reply, uint16_t state) noexcept
{
	bool ok = true;
	size_t start = 0;
	for (;;)
	{
		CanDriversBitmap driverBits;
		size_t savedStart = start;
		const CanAddress boardAddress = drivers.GetNextBoardDriverBitmap(start, driverBits);
		if (boardAddress == CanId::NoAddress)
		{
			break;
		}
		CanMessageBuffer * const buf = CanMessageBuffer::Allocate();
		if (buf == nullptr)
		{
			reply.lcat("No CAN buffer available");
			return false;
		}
		const CanRequestId rid = CanInterface::AllocateRequestId(boardAddress);
		CanMessageMultipleDrivesRequest * const msg = buf->SetupRequestMessage<CanMessageMultipleDrivesRequest>(rid, CanId::MasterAddress, boardAddress, CanMessageType::setDriverStates);
		msg->driversToUpdate = driverBits.GetRaw();
		size_t numDrivers = 0;
		while (savedStart < start && numDrivers < ARRAY_SIZE(msg->values))
		{
			msg->values[numDrivers] = state;
			++savedStart;
			++numDrivers;
		}
		buf->dataLength = msg->GetActualDataLength(numDrivers);
		if (CanInterface::SendRequestAndGetStandardReply(buf, rid, reply) != GCodeResult::ok)
		{
			ok = false;
		}
	}
	return ok;
}

// Add a buffer to the end of the send queue
void CanInterface::SendMotion(CanMessageBuffer *buf) noexcept
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

// Send a request to an expansion board and append the response to 'reply'
GCodeResult CanInterface::SendRequestAndGetStandardReply(CanMessageBuffer *buf, CanRequestId rid, const StringRef& reply, uint8_t *extra) noexcept
{
	taskWaitingOnFifo1 = TaskBase::GetCallerTaskHandle();
	const CanAddress dest = buf->id.Dst();
	mcan_fd_send_ext_message(buf->id.GetWholeId(), reinterpret_cast<uint8_t*>(&(buf->msg)), buf->dataLength, TxBufferIndexRequest, MaxRequestSendWait);
	const uint32_t whenStartedWaiting = millis();
	unsigned int fragmentsReceived = 0;
	const CanMessageType msgType = buf->id.MsgType();								// save for possible error message
	for (;;)
	{
		const uint32_t rxf1s = mcan_instance.hw->MCAN_RXF1S;						// get FIFO 1 status
		if (((rxf1s & MCAN_RXF1S_F1FL_Msk) >> MCAN_RXF1S_F1FL_Pos) != 0)			// if there are any messages
		{
			const uint32_t getIndex = (rxf1s & MCAN_RXF1S_F1GI_Msk) >> MCAN_RXF1S_F1GI_Pos;
			mcan_rx_element elem;
			mcan_get_rx_fifo_1_element(&mcan_instance, &elem, getIndex);			// copy the data (TODO use our own driver, avoid double copying)
			mcan_instance.hw->MCAN_RXF1A = getIndex;								// acknowledge it, release the FIFO entry

			if (elem.R0.bit.XTD == 1 && elem.R0.bit.RTR != 1)						// if extended address and not a remote frame
			{
				// Copy the message and accompanying data to our buffer
				buf->id.SetReceivedId(elem.R0.bit.ID);
				static constexpr uint8_t dlc2len[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};
				buf->dataLength = dlc2len[elem.R1.bit.DLC];
				memcpy(buf->msg.raw, elem.data, buf->dataLength);

				if (   buf->id.MsgType() == CanMessageType::standardReply
					&& buf->id.Src() == dest
					&& (buf->msg.standardReply.requestId == rid || buf->msg.standardReply.requestId == CanRequestIdAcceptAlways)
					&& buf->msg.standardReply.fragmentNumber == fragmentsReceived
				   )
				{
					if (fragmentsReceived == 0)
					{
						reply.lcatn(buf->msg.standardReply.text, buf->msg.standardReply.GetTextLength(buf->dataLength));
						if (extra != nullptr)
						{
							*extra = buf->msg.standardReply.extra;
						}
						uint32_t waitedFor = millis() - whenStartedWaiting;
						if (waitedFor > longestWaitTime)
						{
							longestWaitTime = waitedFor;
							longestWaitMessageType = (uint16_t)msgType;
						}
					}
					else
					{
						reply.catn(buf->msg.standardReply.text, buf->msg.standardReply.GetTextLength(buf->dataLength));
					}
					if (!buf->msg.standardReply.moreFollows)
					{
						const GCodeResult rslt = (GCodeResult)buf->msg.standardReply.resultCode;
						CanMessageBuffer::Free(buf);
						return rslt;
					}
					++fragmentsReceived;
				}
				else
				{
					reply.lcatf("Discarded msg src=%u typ=%u RID=%u exp %u", buf->id.Src(), (unsigned int)buf->id.MsgType(), (unsigned int)buf->msg.standardReply.requestId, rid);
				}
			}
		}
		else
		{
			const uint32_t timeWaiting = millis() - whenStartedWaiting;
			if (timeWaiting >= CanResponseTimeout)
			{
				break;
			}
			TaskBase::Take(CanResponseTimeout - timeWaiting);
		}
	}

	taskWaitingOnFifo1 = nullptr;
	CanMessageBuffer::Free(buf);
	reply.lcatf("Response timeout: CAN addr %u, req type %u, RID=%u", dest, (unsigned int)msgType, (unsigned int)rid);
	return GCodeResult::error;
}

// Send a response to an expansion board and free the buffer
void CanInterface::SendResponse(CanMessageBuffer *buf) noexcept
{
	mcan_fd_send_ext_message(buf->id.GetWholeId(), reinterpret_cast<uint8_t*>(&(buf->msg)), buf->dataLength, TxBufferIndexResponse, MaxResponseSendWait);
	CanMessageBuffer::Free(buf);
}

// Send a broadcast message and free the buffer
void CanInterface::SendBroadcast(CanMessageBuffer *buf) noexcept
{
	mcan_fd_send_ext_message(buf->id.GetWholeId(), reinterpret_cast<uint8_t*>(&(buf->msg)), buf->dataLength, TxBufferBroadcast, MaxResponseSendWait);
	CanMessageBuffer::Free(buf);
}

// Send a request message with no reply expected, and don't free the buffer. Used to send emergency stop messages.
void CanInterface::SendMessageNoReplyNoFree(CanMessageBuffer *buf) noexcept
{
	mcan_fd_send_ext_message(buf->id.GetWholeId(), reinterpret_cast<uint8_t*>(&(buf->msg)), buf->dataLength, TxBufferBroadcast, MaxResponseSendWait);
}

// The CanReceiver task
extern "C" [[noreturn]] void CanReceiverLoop(void *) noexcept
{
	taskWaitingOnFifo0 = TaskBase::GetCallerTaskHandle();
	for (;;)
	{
		const uint32_t rxf0s = mcan_instance.hw->MCAN_RXF0S;						// get FIFO 0 status
		if (((rxf0s & MCAN_RXF0S_F0FL_Msk) >> MCAN_RXF0S_F0FL_Pos) != 0)			// if there are any messages
		{
			CanMessageBuffer *buf = CanMessageBuffer::Allocate();
			if (buf == nullptr)
			{
				delay(2);
			}
			else
			{
				const uint32_t getIndex = (rxf0s & MCAN_RXF0S_F0GI_Msk) >> MCAN_RXF0S_F0GI_Pos;
				mcan_rx_element elem;
				mcan_get_rx_fifo_0_element(&mcan_instance, &elem, getIndex);		// copy the data (TODO use our own driver, avoid double copying)
				mcan_instance.hw->MCAN_RXF0A = getIndex;							// acknowledge it, release the FIFO entry

				if (elem.R0.bit.XTD == 1 && elem.R0.bit.RTR != 1)					// if extended address and not a remote frame
				{
					// Copy the message and accompanying data to a buffer
					buf->id.SetReceivedId(elem.R0.bit.ID);
					static constexpr uint8_t dlc2len[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};
					buf->dataLength = dlc2len[elem.R1.bit.DLC];
					memcpy(buf->msg.raw, elem.data, buf->dataLength);

					CommandProcessor::ProcessReceivedMessage(buf);
				}
				else
				{
					CanMessageBuffer::Free(buf);
				}
			}
		}
		else
		{
			TaskBase::Take();
		}
	}
}

void CanInterface::DisableRemoteDrivers(const CanDriversList& drivers) noexcept
{
	String<1> dummy;
	(void)SetRemoteDriverStates(drivers, dummy.GetRef(), CanMessageMultipleDrivesRequest::driverDisabled);
}

void CanInterface::SetRemoteDriversIdle(const CanDriversList& drivers) noexcept
{
	String<1> dummy;
	(void)SetRemoteDriverStates(drivers, dummy.GetRef(), CanMessageMultipleDrivesRequest::driverIdle);
}

bool CanInterface::SetRemoteStandstillCurrentPercent(const CanDriversData& data, const StringRef& reply) noexcept
{
	return SetRemoteDriverValues(data, reply, CanMessageType::setStandstillCurrentFactor);
}

bool CanInterface::SetRemoteDriverCurrents(const CanDriversData& data, const StringRef& reply) noexcept
{
	return SetRemoteDriverValues(data, reply, CanMessageType::setMotorCurrents);
}

// Set the microstepping on remote drivers, returning true if successful
bool CanInterface::SetRemoteDriverMicrostepping(const CanDriversData& data, const StringRef& reply) noexcept
{
	return SetRemoteDriverValues(data, reply, CanMessageType::setMicrostepping);
}

// Set the pressure advance on remote drivers, returning true if successful
bool CanInterface::SetRemotePressureAdvance(const CanDriversData& data, const StringRef& reply) noexcept
{
	return SetRemoteDriverValues(data, reply, CanMessageType::setPressureAdvance);
}

// Handle M569 for a remote driver
GCodeResult CanInterface::ConfigureRemoteDriver(DriverId driver, GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
pre(driver.IsRemote())
{
	CanMessageGenericConstructor cons(M569Params);
	cons.PopulateFromCommand(gb);
	return cons.SendAndGetResponse(CanMessageType::m569, driver.boardAddress, reply);
}

// Handle M915 for a collection of remote drivers
GCodeResult CanInterface::GetSetRemoteDriverStallParameters(const CanDriversList& drivers, GCodeBuffer& gb, const StringRef& reply, OutputBuffer *& buf) THROWS(GCodeException)
{
	size_t start = 0;
	for (;;)
	{
		CanDriversBitmap driverBits;
		const CanAddress boardAddress = drivers.GetNextBoardDriverBitmap(start, driverBits);
		if (boardAddress == CanId::NoAddress)
		{
			break;
		}

		CanMessageGenericConstructor cons(M915Params);
		cons.AddUParam('d', driverBits.GetRaw());
		cons.PopulateFromCommand(gb);
		if (buf != nullptr)
		{
			reply.Clear();
		}
		const GCodeResult rslt = cons.SendAndGetResponse(CanMessageType::m915, boardAddress, reply);
		if (buf != nullptr)
		{
			buf->lcat(reply.c_str());
		}
		if (rslt != GCodeResult::ok)
		{
			return rslt;
		}
	}
	return GCodeResult::ok;
}

static GCodeResult GetRemoteInfo(uint8_t infoType, uint32_t boardAddress, uint8_t param, GCodeBuffer& gb, const StringRef& reply, uint8_t *extra = nullptr) noexcept
{
	CanInterface::CheckCanAddress(boardAddress, gb);

	CanMessageBuffer * const buf = CanMessageBuffer::Allocate();
	if (buf == nullptr)
	{
		reply.copy("No CAN buffer available");
		return GCodeResult::error;
	}

	const CanRequestId rid = CanInterface::AllocateRequestId(boardAddress);
	auto msg = buf->SetupRequestMessage<CanMessageReturnInfo>(rid, CanId::MasterAddress, (CanAddress)boardAddress);
	msg->type = infoType;
	msg->param = param;
	return CanInterface::SendRequestAndGetStandardReply(buf, rid, reply, extra);
}

// Get diagnostics from an expansion board
GCodeResult CanInterface::RemoteDiagnostics(MessageType mt, uint32_t boardAddress, unsigned int type, GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	CanInterface::CheckCanAddress(boardAddress, gb);

	if (type <= 15)
	{
		Platform& p = reprap.GetPlatform();

		uint8_t currentPart = 0;
		uint8_t lastPart;
		GCodeResult res;
		do
		{
			res = GetRemoteInfo(CanMessageReturnInfo::typeDiagnosticsPart0 + currentPart, boardAddress, type, gb, reply, &lastPart);
			if (res != GCodeResult::ok)
			{
				return res;
			}
			if (type == 0 && currentPart == 0)
			{
				p.MessageF(mt, "Diagnostics for board %u:\n", (unsigned int)boardAddress);
			}
			reply.cat('\n');
			p.Message(mt, reply.c_str());
			reply.Clear();
			++currentPart;
		} while (currentPart <= lastPart);
		return res;
	}

	// It's a diagnostic test
	CanMessageBuffer * const buf = AllocateBuffer(gb);
	const CanRequestId rid = CanInterface::AllocateRequestId(boardAddress);
	auto const msg = buf->SetupRequestMessage<CanMessageDiagnosticTest>(rid, CanId::MasterAddress, (CanAddress)boardAddress);
	msg->testType = type;
	msg->invertedTestType = ~type;
	return SendRequestAndGetStandardReply(buf, rid, reply);			// we may not actually get a reply if the test is one that crashes the expansion board
}

GCodeResult CanInterface::RemoteM408(uint32_t boardAddress, unsigned int type, GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	return GetRemoteInfo(CanMessageReturnInfo::typeM408, boardAddress, type, gb, reply, nullptr);
}

GCodeResult CanInterface::GetRemoteFirmwareDetails(uint32_t boardAddress, GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	return GetRemoteInfo(CanMessageReturnInfo::typeFirmwareVersion, boardAddress, 0, gb, reply);
}

void CanInterface::WakeCanSender() noexcept
{
	canSenderTask.GiveFromISR();
}

// Remote handle functions
GCodeResult CanInterface::CreateHandle(CanAddress boardAddress, RemoteInputHandle h, const char *pinName, uint16_t threshold, uint16_t minInterval,
										bool& currentState, const StringRef& reply) noexcept
{
	CanMessageBuffer * const buf = CanMessageBuffer::Allocate();
	if (buf == nullptr)
	{
		reply.copy("No CAN buffer");
		return GCodeResult::error;
	}

	const CanRequestId rid = AllocateRequestId(boardAddress);
	auto msg = buf->SetupRequestMessage<CanMessageCreateInputMonitor>(rid, CanId::MasterAddress, boardAddress);
	msg->handle = h;
	msg->threshold = threshold;
	msg->minInterval = minInterval;
	SafeStrncpy(msg->pinName, pinName, ARRAY_SIZE(msg->pinName));
	buf->dataLength = msg->GetActualDataLength();

	uint8_t extra;
	const GCodeResult rslt = SendRequestAndGetStandardReply(buf, rid, reply, &extra);
	if (rslt == GCodeResult::ok)
	{
		currentState = (extra != 0);
	}
	return rslt;
}

static GCodeResult ChangeInputMonitor(CanAddress boardAddress, RemoteInputHandle h, uint8_t action, uint16_t param, bool* currentState, const StringRef &reply) noexcept
{
	if (!h.IsValid())
	{
		reply.copy("Invalid remote handle");
		return GCodeResult::error;
	}

	CanMessageBuffer * const buf = CanMessageBuffer::Allocate();
	if (buf == nullptr)
	{
		reply.copy("No CAN buffer");
		return GCodeResult::error;
	}

	const CanRequestId rid = CanInterface::AllocateRequestId(boardAddress);
	auto msg = buf->SetupRequestMessage<CanMessageChangeInputMonitor>(rid, CanId::MasterAddress, boardAddress);
	msg->handle = h;
	msg->action = action;
	msg->param = param;
	uint8_t extra;
	const GCodeResult rslt = CanInterface::SendRequestAndGetStandardReply(buf, rid, reply, &extra);
	if (rslt == GCodeResult::ok && currentState != nullptr)
	{
		*currentState = (extra != 0);
	}
	return rslt;
}

GCodeResult CanInterface::DeleteHandle(CanAddress boardAddress, RemoteInputHandle h, const StringRef &reply) noexcept
{
	return ChangeInputMonitor(boardAddress, h, CanMessageChangeInputMonitor::actionDelete, 0, nullptr, reply);
}

GCodeResult CanInterface::GetHandlePinName(CanAddress boardAddress, RemoteInputHandle h, bool& currentState, const StringRef &reply) noexcept
{
	return ChangeInputMonitor(boardAddress, h, CanMessageChangeInputMonitor::actionReturnPinName, 0, &currentState, reply);
}

GCodeResult CanInterface::EnableHandle(CanAddress boardAddress, RemoteInputHandle h, bool enable, bool &currentState, const StringRef &reply) noexcept
{
	return ChangeInputMonitor(boardAddress, h, (enable) ? CanMessageChangeInputMonitor::actionDoMonitor : CanMessageChangeInputMonitor::actionDontMonitor, 0, &currentState, reply);
}

GCodeResult CanInterface::ChangeHandleResponseTime(CanAddress boardAddress, RemoteInputHandle h, uint16_t responseMillis, bool &currentState, const StringRef &reply) noexcept
{
	return ChangeInputMonitor(boardAddress, h, CanMessageChangeInputMonitor::actionChangeMinInterval, responseMillis, &currentState, reply);
}

void CanInterface::Diagnostics(MessageType mtype) noexcept
{
	reprap.GetPlatform().MessageF(mtype, "=== CAN ===\nMessages sent %" PRIu32 ", longest wait %" PRIu32 "ms for type %u\n",
									messagesSent, longestWaitTime, longestWaitMessageType);
	messagesSent = 0;
	longestWaitTime = 0;
	longestWaitMessageType = 0;
}

GCodeResult CanInterface::WriteGpio(CanAddress boardAddress, uint8_t portNumber, float pwm, bool isServo, const GCodeBuffer& gb, const StringRef &reply) THROWS(GCodeException)
{
	CanMessageBuffer * const buf = AllocateBuffer(gb);
	const CanRequestId rid = CanInterface::AllocateRequestId(boardAddress);
	auto msg = buf->SetupRequestMessage<CanMessageWriteGpio>(rid, CanId::MasterAddress, boardAddress);
	msg->portNumber = portNumber;
	msg->pwm = pwm;
	msg->isServo = isServo;
	return CanInterface::SendRequestAndGetStandardReply(buf, rid, reply, nullptr);
}

GCodeResult CanInterface::ChangeAddressAndNormalTiming(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	// Get the address of the board whose parameters we are changing
	gb.MustSee('B');
	const uint32_t oldAddress = gb.GetUIValue();
	CheckCanAddress(oldAddress, gb);

	// Get the new timing details, if provided
	CanTiming timing;
	bool changeTiming = false;
	if (gb.Seen('S'))
	{
		uint32_t speed = gb.GetUIValue();
		if (speed < MinBitRate || speed > MaxBitRate)
		{
			reply.copy("Data rate out of range");
			return GCodeResult::error;
		}
		speed *= 1000;
		timing.period = (CanTiming::ClockFrequency + speed - 1)/speed;
		const float tseg1 = gb.Seen('T') ? gb.GetFValue() : DefaultSamplePoint;
		if (tseg1 < MinSamplePoint || tseg1 > MaxSamplePoint)
		{
			reply.copy("Sample point out of range");
			return GCodeResult::error;
		}
		timing.tseg1 = lrintf(timing.period * tseg1);

		const float jumpWidth = (gb.Seen('J')) ? gb.GetFValue() : DefaultJumpWidth;
		if (jumpWidth < MinJumpWidth || jumpWidth > MaxJumpWidth)
		{
			reply.copy("Jump width out of range");
			return GCodeResult::error;
		}
		timing.jumpWidth = constrain<uint16_t>(lrintf(timing.period * jumpWidth), 1, timing.period - timing.tseg1 - 2);
		changeTiming = true;
	}

	if (oldAddress == 0)
	{
		if (changeTiming)
		{
			ChangeLocalCanTiming(timing);
		}
		else
		{
			GetLocalCanTiming(timing);
			reply.printf("CAN bus speed %.1fkbps, tseg1 %.2f, jump width %.2f",
							(double)((float)CanTiming::ClockFrequency/(1000 * timing.period)),
							(double)((float)timing.tseg1/(float)timing.period),
							(double)((float)timing.jumpWidth/(float)timing.period));
		}
		return GCodeResult::ok;
	}

	CanMessageBufferHandle buf(AllocateBuffer(gb));
	const CanRequestId rid = CanInterface::AllocateRequestId((uint8_t)oldAddress);
	auto msg = buf.Access()->SetupRequestMessage<CanMessageSetAddressAndNormalTiming>(rid, CanId::MasterAddress, (uint8_t)oldAddress);
	msg->oldAddress = (uint8_t)oldAddress;

	if (gb.Seen('A'))
	{
		const uint32_t newAddress = gb.GetUIValue();
		CheckCanAddress(newAddress, gb);
		msg->newAddress = (uint8_t)newAddress;
		msg->newAddressInverted = (uint8_t)~newAddress;
	}
	else
	{
		msg->newAddress = msg->newAddressInverted = 0;
	}

	msg->doSetTiming = (changeTiming) ? CanMessageSetAddressAndNormalTiming::DoSetTimingYes : CanMessageSetAddressAndNormalTiming::DoSetTimingNo;
	msg->normalTiming = timing;

	return CanInterface::SendRequestAndGetStandardReply(buf.HandOver(), rid, reply, nullptr);
}

GCodeResult CanInterface::ChangeFastTiming(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	return GCodeResult::errorNotSupported;
}

#endif

// End
