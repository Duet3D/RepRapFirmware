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
#include <CanMessageGenericTables.h>
#include <Movement/DDA.h>
#include <Movement/DriveMovement.h>
#include <Movement/Kinematics/HangprinterKinematics.h>
#include <Movement/StepTimer.h>
#include <Movement/Move.h>
#include <RTOSIface/RTOSIface.h>
#include <Platform/RepRap.h>
#include <Platform/Platform.h>
#include <Platform/TaskPriorities.h>
#include <GCodes/GCodes.h>
#include <GCodes/GCodeException.h>
#include <GCodes/GCodeBuffer/GCodeBuffer.h>
#include <ClosedLoop/ClosedLoop.h>
#include <AppNotifyIndices.h>

#if HAS_SBC_INTERFACE
# include "SBC/SbcInterface.h"
#endif

#if SUPPORT_REMOTE_COMMANDS
# include <Version.h>
# include <InputMonitors/InputMonitor.h>
#endif

#include <memory>

#define SUPPORT_CAN		1				// needed by CanDevice.h
#include <CanDevice.h>
#if SAME70
# include <pmc/pmc.h>
#endif

const unsigned int NumCanBuffers = 2 * MaxCanBoards + 10;

constexpr uint32_t MaxMotionSendWait = 20;									// milliseconds
constexpr uint32_t MaxUrgentSendWait = 20;									// milliseconds
constexpr uint32_t MaxTimeSyncSendWait = 2;									// milliseconds
constexpr uint32_t MaxResponseSendWait = CanInterface::UsualSendTimeout;	// milliseconds
constexpr uint32_t MaxRequestSendWait = CanInterface::UsualSendTimeout;		// milliseconds
constexpr uint16_t MaxTimeSyncDelay = 300;									// the maximum normal delay before a CAN time sync message is sent

#define USE_BIT_RATE_SWITCH		0
#define USE_TX_FIFO				1

constexpr uint32_t MinBitRate = 15;											// MCP2542 has a minimum bite rate of 14.4kbps
constexpr uint32_t MaxBitRate = 5000;

constexpr float MinSamplePoint = 0.5;
constexpr float MaxSamplePoint = 0.95;
constexpr float DefaultSamplePoint = 0.75;

constexpr float MinJumpWidth = 0.05;
constexpr float MaxJumpWidth = 0.5;
constexpr float DefaultJumpWidth = 0.25;

static Mutex transactionMutex;

static uint32_t lastTimeSent = 0;
static uint32_t longestWaitTime = 0;
static uint16_t longestWaitMessageType = 0;

static uint32_t peakTimeSyncTxDelay = 0;

#if SUPPORT_REMOTE_COMMANDS
static unsigned int messagesIgnored = 0;
#endif

// Debug
static unsigned int goodTimeStamps = 0;
static unsigned int badTimeStamps = 0;
static unsigned int timeSyncMessagesSent = 0;
// End debug

static volatile uint16_t timeSyncTxTimeStamp;
static volatile bool gotTimeSyncTxTimeStamp = false;

#if !SAME70
static uint16_t lastTimeSyncTxPreparedStamp;
#endif

static CanAddress myAddress =
#ifdef DUET3_ATE
						CanId::ATEMasterAddress;
#else
						CanId::MasterAddress;
#endif

static uint8_t currentTimeSyncMarker = 0xFF;

#if SUPPORT_REMOTE_COMMANDS
static bool inExpansionMode = false;
static bool inTestMode = false;
static bool mainBoardAcknowledgedAnnounce = false;
#endif

//#define CAN_DEBUG

// Define the memory configuration we want to use
constexpr CanDevice::Config Can0Config =
{
	.dataSize = 64,
#if USE_TX_FIFO
	.numTxBuffers = 5,
	.txFifoSize = 16,
#else
	.numTxBuffers = 6,
	.txFifoSize = 2,
#endif
	.numRxBuffers =  0,
	.rxFifo0Size = 32,				// increased from 16 to help with accelerometer and closed loop data collection
	.rxFifo1Size = 16,
	.numShortFilterElements = 0,
# ifdef DUET3_ATE
	.numExtendedFilterElements = 4,
# else
	.numExtendedFilterElements = 3,
# endif
	.txEventFifoSize = 16
};

static_assert(Can0Config.IsValid());

// CAN buffer memory must be in the first 64Kb of RAM (SAME5x) or in non-cached RAM (SAME70), so put it in its own segment
static uint32_t can0Memory[Can0Config.GetMemorySize()] __attribute__ ((section (".CanMessage")));

static CanDevice *can0dev = nullptr;

static unsigned int txTimeouts[Can0Config.numTxBuffers + 1] = { 0 };
static uint32_t lastCancelledId = 0;

#if DUAL_CAN

constexpr CanDevice::Config Can1Config =
{
	.dataSize = 8,
	.numTxBuffers = 2,
	.txFifoSize = 4,
	.numRxBuffers =  0,
	.rxFifo0Size = 16,
	.rxFifo1Size = 16,
	.numShortFilterElements = 1,
	.numExtendedFilterElements = 1,
	.txEventFifoSize = 16
};

static_assert(Can1Config.IsValid());

// CAN buffer memory must be in the first 64Kb of RAM (SAME5x) or in non-cached RAM (SAME70), so put it in its own segment
static uint32_t can1Memory[Can1Config.GetMemorySize()] __attribute__ ((section (".CanMessage")));

static CanDevice *can1dev = nullptr;

#endif

// Transmit buffer usage. All dedicated buffer numbers must be < Can0Config.numTxBuffers.
constexpr auto TxBufferIndexUrgent = CanDevice::TxBufferNumber::buffer0;
constexpr auto TxBufferIndexTimeSync = CanDevice::TxBufferNumber::buffer1;
constexpr auto TxBufferIndexRequest = CanDevice::TxBufferNumber::buffer2;
constexpr auto TxBufferIndexResponse = CanDevice::TxBufferNumber::buffer3;
constexpr auto TxBufferIndexBroadcast = CanDevice::TxBufferNumber::buffer4;

#if USE_TX_FIFO
constexpr auto TxBufferIndexMotion = CanDevice::TxBufferNumber::fifo;				// we send lots of movement messages so use the FIFO for them
#else
constexpr auto TxBufferIndexMotion = CanDevice::TxBufferNumber::buffer5;
#endif

// Receive buffer/FIFO usage. All dedicated buffer numbers must be < Can0Config.numRxBuffers.
constexpr auto RxBufferIndexBroadcast = CanDevice::RxBufferNumber::fifo0;
constexpr auto RxBufferIndexRequest = CanDevice::RxBufferNumber::fifo0;
constexpr auto RxBufferIndexResponse = CanDevice::RxBufferNumber::fifo1;

constexpr uint32_t CanClockIntervalMillis = 200;

// CanSender management task
constexpr size_t CanSenderTaskStackWords = 400;
static Task<CanSenderTaskStackWords> canSenderTask;

constexpr size_t CanReceiverTaskStackWords = 1000;
static Task<CanReceiverTaskStackWords> canReceiverTask;

constexpr size_t CanClockTaskStackWords = 400;			// used to be 300 but RD had a stack overflow
static Task<CanSenderTaskStackWords> canClockTask;

static CanMessageBuffer * volatile pendingMotionBuffers = nullptr;
static CanMessageBuffer * volatile lastMotionBuffer;			// only valid when pendingBuffers != nullptr

#if 0	//unused
static unsigned int numPendingMotionBuffers = 0;
#endif

extern "C" [[noreturn]] void CanSenderLoop(void *) noexcept;
extern "C" [[noreturn]] void CanClockLoop(void *) noexcept;
extern "C" [[noreturn]] void CanReceiverLoop(void *) noexcept;

// Status LED handling

#if SUPPORT_MULTICAST_DISCOVERY

// The STATUS LED is also used to identify one board among several visible to the user
static volatile uint32_t identInitialClocks = 0;		// when we started identifying
static volatile uint32_t identTotalClocks = 0;			// how many step clocks to identify for, zero means until cancelled
static volatile bool identifying = false;

void CanInterface::SetStatusLedIdentify(uint32_t seconds) noexcept
{
	identTotalClocks = seconds * StepClockRate;
	identInitialClocks = StepTimer::GetTimerTicks();
	identifying = true;
}

void CanInterface::SetStatusLedNormal() noexcept
{
	identifying = false;
}

#endif

// This is called only from the CAN clock loop, so inline
static inline void UpdateLed(uint32_t stepClocks) noexcept
{
#if SUPPORT_MULTICAST_DISCOVERY
	if (identifying)
	{
		if (identTotalClocks != 0 && stepClocks - identInitialClocks >= identTotalClocks)
		{
			identifying = 0;							// stop identifying
		}
		else
		{
			// Blink the LED fast. This function gets called every 200ms, so that's the fastest we can blink it without having another task do it.
			reprap.GetPlatform().InvertDiagLed();
			return;
		}
	}
#endif

	// Blink the LED at about 1Hz. Duet 3 expansion boards will blink in sync when they have established clock sync with us.
	reprap.GetPlatform().SetDiagLed((stepClocks & (1u << 19)) != 0);
}

static void InitReceiveFilters() noexcept
{
	// Set up a filter to receive all request messages addressed to us in FIFO 0
	can0dev->SetExtendedFilterElement(0, CanDevice::RxBufferNumber::fifo0,
										CanInterface::GetCanAddress() << CanId::DstAddressShift,
										(CanId::BoardAddressMask << CanId::DstAddressShift) | CanId::ResponseBit);

	// Set up a filter to receive all broadcast messages also in FIFO 0
	can0dev->SetExtendedFilterElement(1, CanDevice::RxBufferNumber::fifo0,
										CanId::BroadcastAddress << CanId::DstAddressShift,
										CanId::BoardAddressMask << CanId::DstAddressShift);

	// Set up a filter to receive response messages in FIFO 1
	can0dev->SetExtendedFilterElement(2, RxBufferIndexResponse,
										(CanInterface::GetCanAddress() << CanId::DstAddressShift) | CanId::ResponseBit,
										(CanId::BoardAddressMask << CanId::DstAddressShift) | CanId::ResponseBit);
# ifdef DUET3_ATE
	// Also respond to requests addressed to board 0 so we can update firmware on ATE boards
	can0dev->SetExtendedFilterElement(3, CanDevice::RxBufferNumber::fifo0,
										CanId::MasterAddress << CanId::DstAddressShift,
										(CanId::BoardAddressMask << CanId::DstAddressShift) | CanId::ResponseBit);
# endif
}

// This is the function called by the transmit event handler when the message marker is nonzero
void TxCallback(uint8_t marker, CanId id, uint16_t timeStamp) noexcept
{
	if (marker == currentTimeSyncMarker)
	{
		timeSyncTxTimeStamp = timeStamp;
		gotTimeSyncTxTimeStamp = true;
		++goodTimeStamps;
	}
	else
	{
		++badTimeStamps;
	}
}

void CanInterface::Init() noexcept
{
	CanMessageBuffer::Init(NumCanBuffers);
	pendingMotionBuffers = nullptr;

	transactionMutex.Create("CanTrans");

#if SAME70
	SetPinFunction(APIN_CAN1_TX, CAN1TXPinPeriphMode);
	SetPinFunction(APIN_CAN1_RX, CAN1RXPinPeriphMode);
# if DUAL_CAN
	SetPinFunction(APIN_CAN0_TX, CAN0PinPeriphMode);
	SetPinFunction(APIN_CAN0_RX, CAN0PinPeriphMode);
# endif
	pmc_enable_upll_clock();			// configure_mcan sets up PCLK5 to be the UPLL divided by something, so make sure the UPLL is running
#elif SAME5x
	SetPinFunction(CanRxPin, CanPinsMode);
	SetPinFunction(CanTxPin, CanPinsMode);
#else
# error Unsupported MCU
#endif

	// Initialise the CAN hardware
	CanTiming timing;
	timing.SetDefaults_1Mb();
	can0dev = CanDevice::Init(0, CanDeviceNumber, Can0Config, can0Memory, timing, nullptr);
	InitReceiveFilters();
	can0dev->Enable();

	CanMotion::Init();

	// Create the task that sends CAN messages
	canClockTask.Create(CanClockLoop, "CanClock", nullptr, TaskPriority::CanClockPriority);
	canSenderTask.Create(CanSenderLoop, "CanSender", nullptr, TaskPriority::CanSenderPriority);
	canReceiverTask.Create(CanReceiverLoop, "CanReceiver", nullptr, TaskPriority::CanReceiverPriority);

#if DUAL_CAN
	timing.SetDefaults_250kb();
	can1dev = CanDevice::Init(1, SecondaryCanDeviceNumber, Can1Config, can1Memory, timing, nullptr);
	can1dev->SetShortFilterElement(0, CanDevice::RxBufferNumber::fifo0, 0, 0);			// set up a filter to receive all messages in FIFO 0
	can1dev->SetExtendedFilterElement(0, CanDevice::RxBufferNumber::fifo0, 0, 0);
	can1dev->Enable();
#endif
}

void CanInterface::Shutdown() noexcept
{
	canClockTask.TerminateAndUnlink();
	canSenderTask.TerminateAndUnlink();
	canReceiverTask.TerminateAndUnlink();

	if (can0dev != nullptr)
	{
		can0dev->DeInit();
		can0dev = nullptr;
	}
}

CanAddress CanInterface::GetCanAddress() noexcept
{
	return myAddress;
}

#if SUPPORT_REMOTE_COMMANDS

bool CanInterface::InExpansionMode() noexcept
{
	return inExpansionMode;
}

bool CanInterface::InTestMode() noexcept
{
	return inTestMode;
}

void CanInterface::LogIgnoredMovementMessage() noexcept
{
	++messagesIgnored;
}

static void ReInit() noexcept
{
	can0dev->Disable();
	InitReceiveFilters();
	can0dev->Enable();
}

void CanInterface::SwitchToExpansionMode(CanAddress addr, bool useTestMode) noexcept
{
	TaskCriticalSectionLocker lock;

	myAddress = addr;
	inExpansionMode = true;
	inTestMode = useTestMode;
	reprap.GetGCodes().SwitchToExpansionMode();
	ReInit();										// reset the CAN filters to account for our new CAN address
}

// Send an announcement message if we haven't had an announce acknowledgement from a main board. On return the buffer is available to use again.
void CanInterface::SendAnnounce(CanMessageBuffer *buf) noexcept
{
	if (inExpansionMode && !mainBoardAcknowledgedAnnounce)
	{
		auto msg = buf->SetupBroadcastMessage<CanMessageAnnounceNew>(myAddress);
		msg->timeSinceStarted = millis();
		msg->numDrivers = NumDirectDrivers;
		msg->usesUf2Binary = BOARD_USES_UF2_BINARY;
		msg->zero = 0;
		memcpy(msg->uniqueId, reprap.GetPlatform().GetUniqueId().GetRaw(), sizeof(msg->uniqueId));
		// Note, board type name, firmware version, firmware date and firmware time are limited to 43 characters in the new
		// We use vertical-bar to separate the three fields: board type, firmware version, date/time
		SafeSnprintf(msg->boardTypeAndFirmwareVersion, ARRAY_SIZE(msg->boardTypeAndFirmwareVersion), "%s|%s|%s%.6s", BOARD_SHORT_NAME, VERSION, DATE, TIME_SUFFIX);
		buf->dataLength = msg->GetActualDataLength();
		SendMessageNoReplyNoFree(buf);
	}
}

// Send an event. The text will be truncated if it is longer than 55 characters.
void CanInterface::RaiseEvent(EventType type, uint16_t param, uint8_t device, const char *format, va_list vargs) noexcept
{
	CanMessageBuffer buf;
	auto msg = buf.SetupStatusMessage<CanMessageEvent>(GetCanAddress(), GetCurrentMasterAddress());
	msg->eventType = type.ToBaseType();;
	msg->deviceNumber = device;
	msg->eventParam = param;
	msg->zero = 0;
	SafeVsnprintf(msg->text, ARRAY_SIZE(msg->text), format, vargs);
	buf.dataLength = msg->GetActualDataLength();
	CanInterface::SendMessageNoReplyNoFree(&buf);
}

void CanInterface::MainBoardAcknowledgedAnnounce() noexcept
{
	mainBoardAcknowledgedAnnounce = true;
}

#endif

// Allocate a CAN request ID
// Currently we reserve the top bit of the 12-bit request ID so that CanRequestIdAcceptAlways is distinct from any genuine request ID.
// Currently we use a single RID sequence for all destination addresses. In future we may use a separate sequence for each address.
// The message buffer is provided so that if the board is not known, we can use the buffer to send a message to it to announce ourselves, but this is not yet implemented
CanRequestId CanInterface::AllocateRequestId(CanAddress destination, CanMessageBuffer *buf) noexcept
{
	static uint16_t rid = 0;

	CanRequestId rslt = rid & CanRequestIdMask;
	++rid;
	return rslt;
}

// Allocate a CAN message buffer, throw if failed
CanMessageBuffer *CanInterface::AllocateBuffer(const GCodeBuffer* gb) THROWS(GCodeException)
{
	CanMessageBuffer * const buf = CanMessageBuffer::Allocate();
	if (buf == nullptr)
	{
		throw GCodeException((gb == nullptr) ? -1 : gb->GetLineNumber(), -1, NoCanBufferMessage);
	}
	return buf;
}

void CanInterface::CheckCanAddress(uint32_t address, const GCodeBuffer& gb) THROWS(GCodeException)
{
	if (address == 0 || address > CanId::MaxCanAddress)
	{
		throw GCodeException(&gb, -1, "CAN address out of range");
	}
}

uint16_t CanInterface::GetTimeStampCounter() noexcept
{
	return can0dev->ReadTimeStampCounter();
}

#if !SAME70

uint16_t CanInterface::GetTimeStampPeriod() noexcept
{
	return can0dev->GetTimeStampPeriod();
}

#endif

// Send a message on the CAN FD channel and record any errors
static void SendCanMessage(CanDevice::TxBufferNumber whichBuffer, uint32_t timeout, CanMessageBuffer *buffer) noexcept
{
	const uint32_t cancelledId = can0dev->SendMessage(whichBuffer, timeout, buffer);
	if (cancelledId != 0)
	{
		++txTimeouts[(unsigned int)whichBuffer];
		lastCancelledId = cancelledId;
	}
}

//TODO can we get rid of the CanSender task if we send movement messages via the Tx FIFO?
// This task picks up motion messages and sends them
extern "C" [[noreturn]] void CanSenderLoop(void *) noexcept
{
	for (;;)
	{
#if SUPPORT_REMOTE_COMMANDS
		if (inExpansionMode)
		{
			// In expansion mode this task just send notifications when the states of input handles change
			CanMessageBuffer buf;
			auto msg = buf.SetupStatusMessage<CanMessageInputChangedNew>(CanInterface::GetCanAddress(), CanInterface::GetCurrentMasterAddress());
			msg->states = 0;
			msg->zero = 0;
			msg->numHandles = 0;

			const uint32_t timeToWait = InputMonitor::AddStateChanges(msg);
			if (msg->numHandles != 0)
			{
				buf.dataLength = msg->GetActualDataLength();
				SendCanMessage(TxBufferIndexUrgent, MaxUrgentSendWait, &buf);
				reprap.GetPlatform().OnProcessingCanMessage();
			}
			TaskBase::TakeIndexed(NotifyIndices::CanSender, timeToWait);		// wait until we are woken up because a message is available, or we time out
		}
		else
#endif
		{
			// In main board mode this task sends urgent messages concerning motion
			for (;;)
			{
				CanMessageBuffer * const urgentMessage = CanMotion::GetUrgentMessage();
				if (urgentMessage != nullptr)
				{
					SendCanMessage(TxBufferIndexUrgent, MaxUrgentSendWait, urgentMessage);
				}
				else if (pendingMotionBuffers != nullptr)
				{
					CanMessageBuffer *buf;
					{
						TaskCriticalSectionLocker lock;
						buf = pendingMotionBuffers;
						pendingMotionBuffers = buf->next;
#if 0	//unused
						--numPendingMotionBuffers;
#endif
					}

					// Send the message
					SendCanMessage(TxBufferIndexMotion, MaxMotionSendWait, buf);
					reprap.GetPlatform().OnProcessingCanMessage();

#ifdef CAN_DEBUG
					// Display a debug message too
					debugPrintf("CCCR %08" PRIx32 ", PSR %08" PRIx32 ", ECR %08" PRIx32 ", TXBRP %08" PRIx32 ", TXBTO %08" PRIx32 ", st %08" PRIx32 "\n",
								MCAN1->MCAN_CCCR, MCAN1->MCAN_PSR, MCAN1->MCAN_ECR, MCAN1->MCAN_TXBRP, MCAN1->MCAN_TXBTO, GetAndClearStatusBits());
					buf->msg.DebugPrint();
					delay(50);
					debugPrintf("CCCR %08" PRIx32 ", PSR %08" PRIx32 ", ECR %08" PRIx32 ", TXBRP %08" PRIx32 ", TXBTO %08" PRIx32 ", st %08" PRIx32 "\n",
								MCAN1->MCAN_CCCR, MCAN1->MCAN_PSR, MCAN1->MCAN_ECR, MCAN1->MCAN_TXBRP, MCAN1->MCAN_TXBTO, GetAndClearStatusBits());
#endif
					// Free the message buffer.
					CanMessageBuffer::Free(buf);
				}
				else
				{
					break;
				}
			}
			TaskBase::TakeIndexed(NotifyIndices::CanSender, Mutex::TimeoutUnlimited);
		}
	}
}

extern "C" [[noreturn]] void CanClockLoop(void *) noexcept
{
	CanMessageBuffer buf;
	uint32_t lastWakeTime = xTaskGetTickCount();
	uint32_t lastRealTimeSent = 0;

	for (;;)
	{
		CanMessageTimeSync * const msg = buf.SetupBroadcastMessage<CanMessageTimeSync>(CanInterface::GetCanAddress());
		msg->lastTimeSent = lastTimeSent;
		msg->lastTimeAcknowledgeDelay = 0;									// assume we don't have the transmit delay available

		currentTimeSyncMarker = ((currentTimeSyncMarker + 1) & 0x0F) | 0xA0;
		buf.marker = currentTimeSyncMarker;
		buf.reportInFifo = 1;

		if (gotTimeSyncTxTimeStamp)
		{
# if SAME70
			// On the SAME70 the step clock is also the external time stamp counter
			const uint32_t timeSyncTxDelay = (timeSyncTxTimeStamp - (uint16_t)lastTimeSent) & 0xFFFF;
# else
			// On the SAME5x the time stamp counter counts CAN bit times divided by 64
			const uint32_t timeSyncTxDelay = (((timeSyncTxTimeStamp - lastTimeSyncTxPreparedStamp) & 0xFFFF) * CanInterface::GetTimeStampPeriod()) >> 6;
# endif
			if (timeSyncTxDelay > peakTimeSyncTxDelay)
			{
				peakTimeSyncTxDelay = timeSyncTxDelay;
			}

			// Occasionally on the SAME70 we get very large delays reported. These delays are not genuine.
			if (timeSyncTxDelay < MaxTimeSyncDelay)
			{
				msg->lastTimeAcknowledgeDelay = timeSyncTxDelay;
			}
			gotTimeSyncTxTimeStamp = false;
		}

		msg->isPrinting = reprap.GetGCodes().IsReallyPrinting();
		msg->zero = 0;

		// Send the real time just once a second unless we also need to send the movement delay
		const uint32_t realTime = (uint32_t)reprap.GetPlatform().GetDateTime();
		const StepTimer::Ticks newMovementDelay = StepTimer::CheckMovementDelayIncreased();
		if (newMovementDelay != 0)
		{
			msg->realTime = realTime;
			lastRealTimeSent = realTime;
			msg->movementDelay = newMovementDelay;
		}
		else if (realTime != lastRealTimeSent)
		{
			msg->realTime = realTime;
			lastRealTimeSent = realTime;
			buf.dataLength = CanMessageTimeSync::SizeWithRealTime;
		}
		else
		{
			buf.dataLength = CanMessageTimeSync::SizeWithoutRealTime;		// send a short message to save CAN bandwidth
		}

#if SAME70
		lastTimeSent = StepTimer::GetTimerTicks();
#else
		{
			AtomicCriticalSectionLocker lock;
			lastTimeSent = StepTimer::GetTimerTicks();
			lastTimeSyncTxPreparedStamp = CanInterface::GetTimeStampCounter();
		}
#endif
		msg->timeSent = lastTimeSent;
		SendCanMessage(TxBufferIndexTimeSync, 0, &buf);
		++timeSyncMessagesSent;

		UpdateLed(lastTimeSent);

		// Delay until it is time again
		vTaskDelayUntil(&lastWakeTime, CanClockIntervalMillis);

#if SUPPORT_REMOTE_COMMANDS
		if (inExpansionMode)
		{
			vTaskDelete(nullptr);											// once in expansion mode we can't revert to main board mode, so we don't need this task any more
		}
#endif

		// Check that the message was sent and get the time stamp
		if (can0dev->IsSpaceAvailable((CanDevice::TxBufferNumber)TxBufferIndexTimeSync, 0))		// if the buffer is free already then the message was sent
		{
			can0dev->PollTxEventFifo(TxCallback);
		}
		else
		{
			(void)can0dev->IsSpaceAvailable((CanDevice::TxBufferNumber)TxBufferIndexTimeSync, MaxTimeSyncSendWait);		// free the buffer
			can0dev->PollTxEventFifo(TxCallback);							// empty the fifo
			gotTimeSyncTxTimeStamp = false;									// ignore any values read from it
		}
	}
}

// Members of namespace CanInterface, and associated local functions

template<class T> static GCodeResult SetRemoteDriverValues(const CanDriversData<T>& data, const StringRef& reply, CanMessageType mt) noexcept
{
	GCodeResult rslt = GCodeResult::ok;
	size_t start = 0;
	for (;;)
	{
		CanDriversBitmap driverBits;
		const size_t savedStart = start;
		const CanAddress boardAddress = data.GetNextBoardDriverBitmap(start, driverBits);
		if (boardAddress == CanId::NoAddress)
		{
			break;
		}
		CanMessageBuffer * const buf = CanMessageBuffer::Allocate();
		if (buf == nullptr)
		{
			return GCodeResult::noCanBuffer;
		}
		const CanRequestId rid = CanInterface::AllocateRequestId(boardAddress, buf);
		CanMessageMultipleDrivesRequest<T> * const msg = buf->SetupRequestMessage<CanMessageMultipleDrivesRequest<T>>(rid, CanInterface::GetCanAddress(), boardAddress, mt);
		msg->driversToUpdate = driverBits.GetRaw();
		const size_t numDrivers = driverBits.CountSetBits();
		for (size_t i = 0; i < numDrivers; ++i)
		{
			msg->values[i] = data.GetElement(savedStart + i);
		}
		buf->dataLength = msg->GetActualDataLength(numDrivers);
		rslt = max(rslt, CanInterface::SendRequestAndGetStandardReply(buf, rid, reply));
	}
	return rslt;
}

// Set remote drivers to enabled, disabled, or idle
// This function is called by both the main task and the Move task.
// As there is no mutual exclusion on putting messages in buffers or receiving replies, when this is called by the Move task
// we send the commands through the FIFO and we use a special RequestId to say that we do not expect a response
static GCodeResult SetRemoteDriverStates(const CanDriversList& drivers, const StringRef& reply, DriverStateControl state) noexcept
{
	GCodeResult rslt = GCodeResult::ok;
	const bool fromMoveTask = TaskBase::GetCallerTaskHandle() == Move::GetMoveTaskHandle();
	size_t start = 0;
	for (;;)
	{
		CanDriversBitmap driverBits;
		const CanAddress boardAddress = drivers.GetNextBoardDriverBitmap(start, driverBits);
		if (boardAddress == CanId::NoAddress)
		{
			break;
		}
		CanMessageBuffer * const buf = CanMessageBuffer::Allocate();
		if (buf == nullptr)
		{
			return GCodeResult::noCanBuffer;
		}
		const CanRequestId rid = (fromMoveTask) ? CanRequestIdNoReplyNeeded : CanInterface::AllocateRequestId(boardAddress, buf);
		const auto msg = buf->SetupRequestMessage<CanMessageMultipleDrivesRequest<DriverStateControl>>(rid, CanInterface::GetCanAddress(), boardAddress, CanMessageType::setDriverStates);
		msg->driversToUpdate = driverBits.GetRaw();
		const size_t numDrivers = driverBits.CountSetBits();
		for (size_t i = 0; i < numDrivers; ++i)
		{
			msg->values[i] = state;
		}
		buf->dataLength = msg->GetActualDataLength(numDrivers);
		if (fromMoveTask)
		{
			CanInterface::SendMotion(buf);														// if it's coming from the Move task then we must send the command via the fifo
		}
		else
		{
			rslt = max(rslt, CanInterface::SendRequestAndGetStandardReply(buf, rid, reply));	// send the command via the usual mechanism
		}
	}
	return rslt;
}

// Add a buffer to the end of the send queue
void CanInterface::SendMotion(CanMessageBuffer *buf) noexcept
{
	buf->next = nullptr;
#if 0
	buf->msg.moveLinear.DebugPrint();
#endif
	{
		TaskCriticalSectionLocker lock;

		if (pendingMotionBuffers == nullptr)
		{
			pendingMotionBuffers = buf;
		}
		else
		{
			lastMotionBuffer->next = buf;
		}
		lastMotionBuffer = buf;
#if 0	//unused
		++numPendingMotionBuffers;
#endif
	}

	canSenderTask.Give(NotifyIndices::CanSender);
}

#if 0	// not currently used

// Get the number of motion messages waiting to be sent through the Tx fifo
unsigned int CanInterface::GetNumPendingMotionMessages() noexcept
{
	return can0dev->NumTxMessagesPending(TxBufferIndexMotion) + numPendingMotionBuffers;
}

#endif

// Send a request to an expansion board and append the response to 'reply'
GCodeResult CanInterface::SendRequestAndGetStandardReply(CanMessageBuffer *buf, CanRequestId rid, const StringRef& reply, uint8_t *extra) noexcept
{
	return SendRequestAndGetCustomReply(buf, rid, reply, extra, CanMessageType::unusedMessageType, [](const CanMessageBuffer*) { });
}

// Send a request to an expansion board and append the response to 'reply'. The response may either be a standard reply or 'replyType'.
GCodeResult CanInterface::SendRequestAndGetCustomReply(CanMessageBuffer *buf, CanRequestId rid, const StringRef& reply, uint8_t *extra, CanMessageType replyType, function_ref_noexcept<void(const CanMessageBuffer*) noexcept> callback) noexcept
{
	if (can0dev == nullptr)
	{
		// Transactions sometimes get requested after we have shut down CAN, e.g. when we destroy filament monitors
		CanMessageBuffer::Free(buf);
		return GCodeResult::error;
	}

	const CanAddress dest = buf->id.Dst();
	const CanMessageType msgType = buf->id.MsgType();								// save for possible error message

	{
		// This code isn't re-entrant and it can get called from a task other than Main to shut the system down, so we need to use a mutex
		MutexLocker lock(transactionMutex);

		SendCanMessage(TxBufferIndexRequest, MaxRequestSendWait, buf);
		reprap.GetPlatform().OnProcessingCanMessage();

		const uint32_t whenStartedWaiting = millis();
		uint32_t timeWaiting = 0;
		unsigned int fragmentsReceived = 0;
		for (;;)
		{
			const uint32_t timeout = (timeWaiting < UsualResponseTimeout) ? UsualResponseTimeout - timeWaiting : 1;
			if (!can0dev->ReceiveMessage(RxBufferIndexResponse, timeout, buf))
			{
				break;
			}

			if (reprap.Debug(Module::CAN))
			{
				buf->DebugPrint("Rx1:");
			}

			// The following code allows for the possibility of receiving a StandardReply (e.g. because an error occurred) when we were expecting a different reply.
			// It assumes that any custom reply we may want has the requestId in the same place as a StandardReply.
			const bool matchesRequest = buf->id.Src() == dest
										&& (buf->msg.standardReply.requestId == rid || buf->msg.standardReply.requestId == CanRequestIdAcceptAlways);
			if (matchesRequest && buf->id.MsgType() == CanMessageType::standardReply && buf->msg.standardReply.fragmentNumber == fragmentsReceived)
			{
				if (fragmentsReceived == 0)
				{
					const size_t textLength = buf->msg.standardReply.GetTextLength(buf->dataLength);
					if (textLength != 0)			// avoid concatenating blank lines to existing output
					{
						reply.lcatn(buf->msg.standardReply.text, textLength);
					}
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
			else if (matchesRequest && buf->id.MsgType() == replyType && fragmentsReceived == 0)
			{
				callback(buf);
				CanMessageBuffer::Free(buf);
				return GCodeResult::ok;
			}
			else
			{
				// We received an unexpected message. Don't tack it on to 'reply' because some replies contain important data, e.g. request for board short name.
				if (buf->id.MsgType() == CanMessageType::standardReply)
				{
					reprap.GetPlatform().MessageF(WarningMessage, "Discarded std reply src=%u RID=%u exp=%u \"%.*s\"\n",
													buf->id.Src(), (unsigned int)buf->msg.standardReply.requestId, rid, buf->msg.standardReply.GetTextLength(buf->dataLength), buf->msg.standardReply.text);
				}
				else
				{
					reprap.GetPlatform().MessageF(WarningMessage, "Discarded msg src=%u typ=%u RID=%u exp=%u\n",
													buf->id.Src(), (unsigned int)buf->id.MsgType(), (unsigned int)buf->msg.standardReply.requestId, rid);
				}
			}
			timeWaiting = millis() - whenStartedWaiting;
		}
	}

	CanMessageBuffer::Free(buf);
	reply.lcatf("CAN response timeout: board %u, req type %u, RID %u", dest, (unsigned int)msgType, (unsigned int)rid);
	return GCodeResult::canResponseTimeout;
}

// Send a response to an expansion board and free the buffer
void CanInterface::SendResponseNoFree(CanMessageBuffer *buf) noexcept
{
	SendCanMessage(TxBufferIndexResponse, MaxResponseSendWait, buf);
}

// Send a broadcast message and free the buffer
void CanInterface::SendBroadcastNoFree(CanMessageBuffer *buf) noexcept
{
	if (can0dev != nullptr)
	{
		SendCanMessage(TxBufferIndexBroadcast, MaxResponseSendWait, buf);
	}
}

// Send a request message with no reply expected, and don't free the buffer. Used to send emergency stop messages.
void CanInterface::SendMessageNoReplyNoFree(CanMessageBuffer *buf) noexcept
{
	if (can0dev != nullptr)
	{
		SendCanMessage(TxBufferIndexBroadcast, MaxResponseSendWait, buf);
	}
}

#if DUAL_CAN

uint32_t CanInterface::SendPlainMessageNoFree(CanMessageBuffer *buf, uint32_t const timeout) noexcept
{
	return (can1dev != nullptr) ? can1dev->SendMessage(CanDevice::TxBufferNumber::fifo, timeout, buf) : 0;
}

bool CanInterface::ReceivePlainMessage(CanMessageBuffer *null buf, uint32_t const timeout) noexcept
{
	return can1dev != nullptr && can1dev->ReceiveMessage(CanDevice::RxBufferNumber::fifo0, timeout, buf);
}

#endif

// The CanReceiver task
extern "C" [[noreturn]] void CanReceiverLoop(void *) noexcept
{
	CanMessageBuffer buf;
	for (;;)
	{
		if (can0dev->ReceiveMessage(RxBufferIndexRequest, TaskBase::TimeoutUnlimited, &buf))
		{
			if (reprap.Debug(Module::CAN))
			{
				buf.DebugPrint("Rx0:");
			}

			CommandProcessor::ProcessReceivedMessage(&buf);
		}
	}
}

// This one is used by ATE
GCodeResult CanInterface::EnableRemoteDrivers(const CanDriversList& drivers, const StringRef& reply) noexcept
{
	return SetRemoteDriverStates(drivers, reply, DriverStateControl(DriverStateControl::driverActive));
}

// This one is used by Prepare and by M17
void CanInterface::EnableRemoteDrivers(const CanDriversList& drivers) noexcept
{
	String<1> dummy;
	(void)EnableRemoteDrivers(drivers, dummy.GetRef());
}

// This one is used by ATE
GCodeResult CanInterface::DisableRemoteDrivers(const CanDriversList& drivers, const StringRef& reply) noexcept
{
	return SetRemoteDriverStates(drivers, reply, DriverStateControl(DriverStateControl::driverDisabled));
}

// This one is used by Prepare
void CanInterface::DisableRemoteDrivers(const CanDriversList& drivers) noexcept
{
	String<1> dummy;
	(void)DisableRemoteDrivers(drivers, dummy.GetRef());
}

void CanInterface::SetRemoteDriversIdle(const CanDriversList& drivers, float idleCurrentFactor) noexcept
{
	String<1> dummy;
	(void)SetRemoteDriverStates(drivers, dummy.GetRef(), DriverStateControl(DriverStateControl::driverIdle, (uint16_t)lrintf(idleCurrentFactor * 100)));
}

GCodeResult CanInterface::SetRemoteStandstillCurrentPercent(const CanDriversData<float>& data, const StringRef& reply) noexcept
{
	return SetRemoteDriverValues(data, reply, CanMessageType::setStandstillCurrentFactor);
}

GCodeResult CanInterface::SetRemoteDriverCurrents(const CanDriversData<float>& data, const StringRef& reply) noexcept
{
	return SetRemoteDriverValues(data, reply, CanMessageType::setMotorCurrents);
}

GCodeResult CanInterface::SetRemoteDriverStepsPerMmAndMicrostepping(const CanDriversData<StepsPerUnitAndMicrostepping>& data, const StringRef& reply) noexcept
{
	return SetRemoteDriverValues(data, reply, CanMessageType::setStepsPerMmAndMicrostepping);
}

// Set the pressure advance on remote drivers, returning true if successful
GCodeResult CanInterface::SetRemotePressureAdvance(const CanDriversData<float>& data, const StringRef& reply) noexcept
{
	return SetRemoteDriverValues(data, reply, CanMessageType::setPressureAdvance);
}

// Handle M569 for a remote driver
GCodeResult CanInterface::ConfigureRemoteDriver(DriverId driver, GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
pre(driver.IsRemote())
{
	switch (gb.GetCommandFraction())
	{
	case -1:
	case 0:
		if (gb.SeenAny("DRSV"))
		{
			if (!reprap.GetGCodes().LockAllMovementSystemsAndWaitForStandstill(gb))
			{
				return GCodeResult::notFinished;
			}
		}
		{
			CanMessageGenericConstructor cons(M569Params);
			cons.PopulateFromCommand(gb);
			return cons.SendAndGetResponse(CanMessageType::m569, driver.boardAddress, reply);
		}

	case 1:
		if (gb.SeenAny("CDEHIRSTV"))
		{
			if (!reprap.GetGCodes().LockAllMovementSystemsAndWaitForStandstill(gb))
			{
				return GCodeResult::notFinished;
			}
		}
		{
			CanMessageGenericConstructor cons(M569Point1Params);
			cons.PopulateFromCommand(gb);
			return cons.SendAndGetResponse(CanMessageType::m569p1, driver.boardAddress, reply);
		}

	case 2:
		{
			gb.MustSee('R');
			CanMessageGenericConstructor cons(M569Point2Params);
			cons.PopulateFromCommand(gb);
			return cons.SendAndGetResponse(CanMessageType::m569p2, driver.boardAddress, reply);
		}

#if DUAL_CAN
	case 3:			// read driver encoder via secondary CAN
		{
			Kinematics& kin = reprap.GetMove().GetKinematics();
			if (kin.GetKinematicsType() == KinematicsType::hangprinter)
			{
				return ((HangprinterKinematics&)kin).ReadODrive3Encoder(driver, gb, reply);
			}
		}
		return GCodeResult::errorNotSupported;
#endif

	case 4:			// set driver torque mode
		// M569.4 is supported both by Hangprinter with ODrives and experimentally by the EXP1HCL and M23CL boards.
		// First check whether we have a suitable EXP1HCL or M23 driver at the specified CAN address.
		// ODrive CAN addresses in this command are between 40 and 43 inclusive so the EXP1HCL or M23CL addresses should avoid that range when using Hangprinter kinematics.
		{
			const ExpansionBoardData *const boardData = reprap.GetExpansion().GetBoardDetails(driver.boardAddress);
			if (boardData != nullptr && boardData->hasClosedLoop)
			{
				CanMessageGenericConstructor cons(M569Point4Params);
				cons.PopulateFromCommand(gb);
				return cons.SendAndGetResponse(CanMessageType::m569p4, driver.boardAddress, reply);
			}
		}
#if DUAL_CAN
		{
			Kinematics& kin = reprap.GetMove().GetKinematics();
			if (kin.GetKinematicsType() == KinematicsType::hangprinter)
			{
				gb.MustSee('T');
				const float torque = gb.GetFValue();
				return ((HangprinterKinematics&)kin).SetODrive3TorqueMode(driver, torque, reply);
			}
		}
#endif
		reply.copy("not supported by this driver");
		return GCodeResult::error;

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE
	case 5:
		return ClosedLoop::StartDataCollection(driver, gb, reply);
#endif

	case 6:
		if (gb.LatestMachineState().commandRepeated)
		{
			// We already sent the tuning command so we just need to get the status. Delay 100ms to avoid overloading the CAN bus.
			if (!gb.DoDwellTime(100))
			{
				return GCodeResult::notFinished;
			}
			CanMessageGenericConstructor cons(M569Point6Params_StatusOnly);
			cons.PopulateFromCommand(gb);
			return cons.SendAndGetResponse(CanMessageType::m569p6, driver.boardAddress, reply);
		}
		else
		{
			// First call, so send the tuning command
			if (!reprap.GetGCodes().LockAllMovementSystemsAndWaitForStandstill(gb))
			{
				return GCodeResult::notFinished;
			}
			CanMessageGenericConstructor cons(M569Point6Params);
			cons.PopulateFromCommand(gb);
			return cons.SendAndGetResponse(CanMessageType::m569p6, driver.boardAddress, reply);
		}

	case 7:
		if (gb.Seen('C'))
		{
			// If a port name if provided, it must match the board ID
			String<StringLength20> portName;
			gb.GetQuotedString(portName.GetRef(), false);
			if (isdigit(portName[0]) && IoPort::RemoveBoardAddress(portName.GetRef()) != driver.boardAddress)
			{
				reply.copy("Brake port must be on same board as driver");
				return GCodeResult::error;
			}
		}
		{
			CanMessageGenericConstructor cons(M569Point7Params);
			cons.PopulateFromCommand(gb);
			return cons.SendAndGetResponse(CanMessageType::m569p7, driver.boardAddress, reply);
		}
#if DUAL_CAN
	case 8:			// read axis force via secondary CAN
		{
			Kinematics& kin = reprap.GetMove().GetKinematics();
			if (kin.GetKinematicsType() == KinematicsType::hangprinter)
			{
				return ((HangprinterKinematics&)kin).ReadODrive3AxisForce(driver, reply);
			}
		}
		return GCodeResult::errorNotSupported;
#endif

	default:
		return GCodeResult::errorNotSupported;
	}
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

static GCodeResult GetRemoteInfo(uint8_t infoType, uint32_t boardAddress, uint8_t param, GCodeBuffer& gb, const StringRef& reply, uint8_t *extra = nullptr) THROWS(GCodeException)
{
	CanInterface::CheckCanAddress(boardAddress, gb);

	CanMessageBuffer * const buf = CanMessageBuffer::Allocate();
	if (buf == nullptr)
	{
		return GCodeResult::noCanBuffer;
	}

	const CanRequestId rid = CanInterface::AllocateRequestId(boardAddress, buf);
	auto msg = buf->SetupRequestMessage<CanMessageReturnInfo>(rid, CanInterface::GetCanAddress(), (CanAddress)boardAddress);
	msg->type = infoType;
	msg->param = param;
	return CanInterface::SendRequestAndGetStandardReply(buf, rid, reply, extra);
}

// Get diagnostics from an expansion board
GCodeResult CanInterface::RemoteDiagnostics(MessageType mt, uint32_t boardAddress, unsigned int type, GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	CanInterface::CheckCanAddress(boardAddress, gb);

	if (type == 0)
	{
		Platform& p = reprap.GetPlatform();
		uint8_t currentPart = 0;
		uint8_t lastPart;
		GCodeResult res;
		do
		{
			// The standard reply buffer is only 256 bytes long. We need a bigger one to receive the software reset data.
			String<StringLength500> infoBuffer;
			res = GetRemoteInfo(CanMessageReturnInfo::typeDiagnosticsPart0 + currentPart, boardAddress, type, gb, infoBuffer.GetRef(), &lastPart);
			if (res != GCodeResult::ok)
			{
				reply.copy(infoBuffer.c_str());
				return res;
			}
			if (type == 0 && currentPart == 0)
			{
				p.MessageF(mt, "Diagnostics for board %u:\n", (unsigned int)boardAddress);
			}
			if (!infoBuffer.IsEmpty())						// driverless boards may return empty response parts
			{
				infoBuffer.cat('\n');						// don't use MessageF, the format buffer is too small
				p.Message(mt, infoBuffer.c_str());
			}
			++currentPart;
		} while (currentPart <= lastPart);
		return res;
	}

	if (type == 1)
	{
		// Request a test report
		CanMessageGenericConstructor cons(M122P1Params);
		cons.PopulateFromCommand(gb);
		return cons.SendAndGetResponse(CanMessageType::testReport, boardAddress, reply);
	}

	// It's a diagnostic test
	CanMessageBuffer * const buf = AllocateBuffer(&gb);
	const CanRequestId rid = CanInterface::AllocateRequestId(boardAddress, buf);
	auto const msg = buf->SetupRequestMessage<CanMessageDiagnosticTest>(rid, GetCanAddress(), (CanAddress)boardAddress);
	msg->testType = type;
	msg->invertedTestType = ~type;
	if (type == (uint16_t)DiagnosticTestType::AccessMemory)
	{
		gb.MustSee('A');
		msg->param32[0] = (uint32_t)gb.GetIValue();			// allow negative values so that we can read high memory addresses
		if (gb.Seen('V'))
		{
			msg->param32[1] = (uint32_t)gb.GetIValue();		// allow negative values so that we can set high values
			msg->param16 = 1;
		}
		else
		{
			msg->param16 = 0;
		}
		msg->param32[2] = (gb.Seen('R')) ? gb.GetUIValue() : 1;
	}
	return SendRequestAndGetStandardReply(buf, rid, reply);	// we may not actually get a reply if the test is one that crashes the expansion board
}

GCodeResult CanInterface::RemoteM408(uint32_t boardAddress, unsigned int type, GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	return GetRemoteInfo(CanMessageReturnInfo::typeM408, boardAddress, type, gb, reply, nullptr);
}

GCodeResult CanInterface::GetRemoteFirmwareDetails(uint32_t boardAddress, GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	return GetRemoteInfo(CanMessageReturnInfo::typeFirmwareVersion, boardAddress, 0, gb, reply);
}

void CanInterface::WakeAsyncSender() noexcept
{
	if (inInterrupt())
	{
		canSenderTask.GiveFromISR(NotifyIndices::CanSender);
	}
	else
	{
		canSenderTask.Give(NotifyIndices::CanSender);
	}
}

// Remote handle functions
GCodeResult CanInterface::CreateHandle(CanAddress boardAddress, RemoteInputHandle h, const char *_ecv_array pinName, uint16_t threshold, uint16_t minInterval,
										bool& currentState, const StringRef& reply) noexcept
{
	CanMessageBuffer * const buf = CanMessageBuffer::Allocate();
	if (buf == nullptr)
	{
		return GCodeResult::noCanBuffer;
	}

	const CanRequestId rid = AllocateRequestId(boardAddress, buf);
	auto msg = buf->SetupRequestMessage<CanMessageCreateInputMonitorNew>(rid, GetCanAddress(), boardAddress);
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

static GCodeResult ChangeInputMonitor(CanAddress boardAddress, RemoteInputHandle h, uint8_t action, uint32_t param, uint8_t* retVal, const StringRef &reply) noexcept
{
	if (!h.IsValid())
	{
		reply.copy("Invalid remote handle");
		return GCodeResult::error;
	}

	CanMessageBuffer * const buf = CanMessageBuffer::Allocate();
	if (buf == nullptr)
	{
		return GCodeResult::noCanBuffer;
	}

	const CanRequestId rid = CanInterface::AllocateRequestId(boardAddress, buf);
	auto msg = buf->SetupRequestMessage<CanMessageChangeInputMonitorNew>(rid, CanInterface::GetCanAddress(), boardAddress);
	msg->handle = h;
	msg->action = action;
	msg->param = param;
	uint8_t extra;
	const GCodeResult rslt = CanInterface::SendRequestAndGetStandardReply(buf, rid, reply, &extra);
	if (rslt == GCodeResult::ok && retVal != nullptr)
	{
		*retVal = extra;
	}
	return rslt;
}

GCodeResult CanInterface::DeleteHandle(CanAddress boardAddress, RemoteInputHandle h, const StringRef &reply) noexcept
{
	return ChangeInputMonitor(boardAddress, h, CanMessageChangeInputMonitorNew::actionDelete, 0, nullptr, reply);
}

GCodeResult CanInterface::GetHandlePinName(CanAddress boardAddress, RemoteInputHandle h, bool& currentState, const StringRef &reply) noexcept
{
	uint8_t rVal;
	const GCodeResult ret = ChangeInputMonitor(boardAddress, h, CanMessageChangeInputMonitorNew::actionReturnPinName, 0, &rVal, reply);
	if (ret < GCodeResult::error)
	{
		currentState = (rVal != 0);
	}
	return ret;
}

GCodeResult CanInterface::EnableHandle(CanAddress boardAddress, RemoteInputHandle h, bool enable, bool &currentState, const StringRef &reply) noexcept
{
	uint8_t rVal;
	const GCodeResult ret =  ChangeInputMonitor(boardAddress, h, (enable) ? CanMessageChangeInputMonitorNew::actionDoMonitor : CanMessageChangeInputMonitorNew::actionDontMonitor, 0, &rVal, reply);
	if (ret < GCodeResult::error)
	{
		currentState = (rVal != 0);
	}
	return ret;
}

GCodeResult CanInterface::ChangeHandleResponseTime(CanAddress boardAddress, RemoteInputHandle h, uint32_t responseMillis, bool &currentState, const StringRef &reply) noexcept
{
	uint8_t rVal;
	const GCodeResult ret =  ChangeInputMonitor(boardAddress, h, CanMessageChangeInputMonitorNew::actionChangeMinInterval, responseMillis, &rVal, reply);
	if (ret < GCodeResult::error)
	{
		currentState = (rVal != 0);
	}
	return ret;
}

GCodeResult CanInterface::ChangeHandleThreshold(CanAddress boardAddress, RemoteInputHandle h, uint32_t threshold, bool &currentState, const StringRef &reply) noexcept
{
	uint8_t rVal;
	const GCodeResult ret =  ChangeInputMonitor(boardAddress, h, CanMessageChangeInputMonitorNew::actionChangeThreshold, threshold, &rVal, reply);
	if (ret < GCodeResult::error)
	{
		currentState = (rVal != 0);
	}
	return ret;
}

GCodeResult CanInterface::SetHandleDriveLevel(CanAddress boardAddress, RemoteInputHandle h, uint32_t driveLevel, uint8_t &returnedDriveLevel, const StringRef &reply) noexcept
{
	return ChangeInputMonitor(boardAddress, h, CanMessageChangeInputMonitorNew::actionSetDriveLevel, driveLevel, &returnedDriveLevel, reply);
}

GCodeResult CanInterface::ReadRemoteHandles(CanAddress boardAddress, RemoteInputHandle mask, RemoteInputHandle pattern, ReadHandlesCallbackFunction callback, CallbackParameter param, const StringRef &reply) noexcept
{
	CanMessageBuffer * const buf = CanMessageBuffer::Allocate();
	if (buf == nullptr)
	{
		return GCodeResult::noCanBuffer;
	}

	const CanRequestId rid = CanInterface::AllocateRequestId(boardAddress, buf);
	auto msg = buf->SetupRequestMessage<CanMessageReadInputsRequest>(rid, GetCanAddress(), boardAddress);
	msg->mask = mask;
	msg->pattern = pattern;
	const GCodeResult rslt = SendRequestAndGetCustomReply(buf, rid, reply, nullptr, CanMessageType::readInputsReply,
															[callback, param](const CanMessageBuffer *buf)
																{
																	auto response = buf->msg.readInputsReply;
																	for (unsigned int i = 0; i < response.numReported; ++i)
																	{
																		callback(param, response.results[i].handle, LoadLEU32(&response.results[i].reading));
																	}
																});
	return rslt;
}

void CanInterface::Diagnostics(MessageType mtype) noexcept
{
	Platform& p = reprap.GetPlatform();
	p.Message(mtype, "=== CAN ===\n");
	// If the user runs M122 after an emergency stop, can0dev will be null
	if (can0dev == nullptr)
	{
		p.Message(mtype, "Disabled\n");
	}
	else
	{
		CanDevice::CanStats stats;
		can0dev->GetAndClearStats(stats);
		p.MessageF(mtype, "Messages queued %u, received %u, lost %u, "
#if SUPPORT_REMOTE_COMMANDS
							"ignored %u, "
#endif
							"errs %u, boc %u\n",
							stats.messagesQueuedForSending, stats.messagesReceived, stats.messagesLost,
#if SUPPORT_REMOTE_COMMANDS
							messagesIgnored,
#endif
							stats.protocolErrors, stats.busOffCount);
	}

#if SUPPORT_REMOTE_COMMANDS
	messagesIgnored = 0;
#endif

	p.MessageF(mtype,
				"Longest wait %" PRIu32 "ms for reply type %u, peak Tx sync delay %" PRIu32
				", free buffers %u (min %u)"
	//debug
				", ts %u/%u/%u"
	//end debug
				"\n",
					longestWaitTime, longestWaitMessageType, peakTimeSyncTxDelay,
					CanMessageBuffer::GetFreeBuffers(), CanMessageBuffer::GetAndClearMinFreeBuffers()
	//debug
					, timeSyncMessagesSent, goodTimeStamps, badTimeStamps
	//end debug
				);

	String<StringLength100> str;
	char c = ' ';
	for (unsigned int& txt : txTimeouts)
	{
		str.catf("%c%u", c, txt);
		txt = 0;
		c = ',';
	}

	if (lastCancelledId != 0)
	{
		CanId id;
		id.SetReceivedId(lastCancelledId);
		lastCancelledId = 0;
		str.catf(" last cancelled message type %u dest %u", (unsigned int)id.MsgType(), id.Dst());
	}

	reprap.GetPlatform().MessageF(mtype, "Tx timeouts%s\n", str.c_str());
	longestWaitTime = 0;
	longestWaitMessageType = 0;
	peakTimeSyncTxDelay = 0;
	timeSyncMessagesSent = goodTimeStamps = badTimeStamps = 0;
}

GCodeResult CanInterface::WriteGpio(CanAddress boardAddress, uint8_t portNumber, float pwm, bool isServo, const GCodeBuffer* gb, const StringRef &reply) noexcept
{
	CanMessageBuffer * const buf = CanMessageBuffer::Allocate();
	if (buf == nullptr)
	{
		return GCodeResult::noCanBuffer;
	}

	const CanRequestId rid = CanInterface::AllocateRequestId(boardAddress, buf);
	auto msg = buf->SetupRequestMessage<CanMessageWriteGpio>(rid, GetCanAddress(), boardAddress);
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
	if (oldAddress != 0)							// we must allow address 0 but CheckCanAddress doesn't allow it
	{
		CheckCanAddress(oldAddress, gb);
	}

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

	if (oldAddress == GetCanAddress())
	{
		if (changeTiming)
		{
			can0dev->SetLocalCanTiming(timing);
		}
		else
		{
			can0dev->GetLocalCanTiming(timing);
			reply.printf("CAN bus speed %.1fkbps, tseg1 %.2f, jump width %.2f",
							(double)((float)CanTiming::ClockFrequency/(1000 * timing.period)),
							(double)((float)timing.tseg1/(float)timing.period),
							(double)((float)timing.jumpWidth/(float)timing.period));
		}
		return GCodeResult::ok;
	}

	CanMessageBufferHandle buf(AllocateBuffer(&gb));
	const CanRequestId rid = CanInterface::AllocateRequestId((uint8_t)oldAddress, buf.Access());
	auto msg = buf.Access()->SetupRequestMessage<CanMessageSetAddressAndNormalTiming>(rid, GetCanAddress(), (uint8_t)oldAddress);
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

// Create a filament monitor but do not configure it
GCodeResult CanInterface::CreateFilamentMonitor(DriverId driver, uint8_t type, const GCodeBuffer& gb, const StringRef &reply) noexcept
{
	try
	{
		CanMessageBuffer* const buf = AllocateBuffer(&gb);
		const CanRequestId rid = CanInterface::AllocateRequestId(driver.boardAddress, buf);
		auto msg = buf->SetupRequestMessage<CanMessageCreateFilamentMonitor>(rid, GetCanAddress(), driver.boardAddress);
		msg->driver = driver.localDriver;
		msg->type = type;
		return SendRequestAndGetStandardReply(buf, rid, reply);
	}
	catch (const GCodeException& ex)
	{
		ex.GetMessage(reply, &gb);
		return GCodeResult::warning;
	}
}

// Configure a filament monitor
GCodeResult CanInterface::ConfigureFilamentMonitor(DriverId driver, GCodeBuffer &gb, const StringRef &reply) THROWS(GCodeException)
{
	CanMessageGenericConstructor cons(ConfigureFilamentMonitorParams);
	cons.AddUParam('d', driver.localDriver);
	cons.PopulateFromCommand(gb);
	return cons.SendAndGetResponse(CanMessageType::configureFilamentMonitor, driver.boardAddress, reply);
}

// Delete a filament monitor. XCalled from a destructor, so no exceptions or error return.
GCodeResult CanInterface::DeleteFilamentMonitor(DriverId driver, GCodeBuffer* gb, const StringRef& reply) noexcept
{
	try
	{
		CanMessageBuffer* const buf = AllocateBuffer(gb);
		const CanRequestId rid = CanInterface::AllocateRequestId(driver.boardAddress, buf);
		auto msg = buf->SetupRequestMessage<CanMessageDeleteFilamentMonitor>(rid, GetCanAddress(), driver.boardAddress);
		msg->driver = driver.localDriver;
		return SendRequestAndGetStandardReply(buf, rid, reply);
	}
	catch (const GCodeException& ex)
	{
		ex.GetMessage(reply, gb);
		return GCodeResult::warning;
	}
}

# if SUPPORT_ACCELEROMETERS

GCodeResult CanInterface::StartAccelerometer(DriverId device, uint8_t axes, uint16_t numSamples, uint8_t mode, const GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	CanMessageBuffer* const buf = AllocateBuffer(&gb);
	const CanRequestId rid = CanInterface::AllocateRequestId(device.boardAddress, buf);
	auto msg = buf->SetupRequestMessage<CanMessageStartAccelerometer>(rid, GetCanAddress(), device.boardAddress);
	msg->deviceNumber = device.localDriver;
	msg->axes = axes;
	msg->numSamples = numSamples;
	msg->delayedStart = 0;
	msg->startTime = false;
	return SendRequestAndGetStandardReply(buf, rid, reply);
}

# endif

GCodeResult CanInterface::StartClosedLoopDataCollection(DriverId device, uint16_t filter, uint16_t numSamples, uint16_t rateRequested, uint8_t movementRequested, uint8_t mode, const GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	CanMessageBuffer* const buf = AllocateBuffer(&gb);
	const CanRequestId rid = CanInterface::AllocateRequestId(device.boardAddress, buf);
	auto msg = buf->SetupRequestMessage<CanMessageStartClosedLoopDataCollection>(rid, GetCanAddress(), device.boardAddress);
	msg->mode = mode;
	msg->filter = filter;
	msg->rate = rateRequested;
	msg->numSamples = numSamples;
	msg->movement = movementRequested;
	msg->deviceNumber = device.localDriver;
	return SendRequestAndGetStandardReply(buf, rid, reply);
}

#if DUAL_CAN

CanId CanInterface::ODrive::ArbitrationId(DriverId const driver, uint8_t const cmd) noexcept {
	const auto arbitration_id = (driver.boardAddress << 5) + cmd;
	CanId canId;
	canId.SetReceivedId(arbitration_id);
	return canId;
}

CanMessageBuffer * CanInterface::ODrive::PrepareSimpleMessage(DriverId const driver, const StringRef& reply) noexcept
{
	// Detect any early return conditions
	if (can1dev == nullptr)
	{
		return nullptr;
	}
	CanMessageBuffer * buf = CanMessageBuffer::Allocate();
	if (buf == nullptr)
	{
		return nullptr;
	}

	// Build the message
	buf->marker = 0;
	buf->extId = false; // ODrive uses 11-bit IDs
	buf->fdMode = false;
	buf->useBrs = false;
	buf->dataLength = 0;
	buf->reportInFifo = false;

	return buf;
}

void CanInterface::ODrive::FlushCanReceiveHardware() noexcept
{
	while (CanInterface::ReceivePlainMessage(nullptr, 0)) { }
}

bool CanInterface::ODrive::GetExpectedSimpleMessage(CanMessageBuffer *buf, DriverId const driver, uint8_t const cmd, const StringRef& reply) noexcept
{
	CanId const expectedId = ArbitrationId(driver, cmd);

	int count = 0;
	bool ok = true;
	do{
		ok = ReceivePlainMessage(buf);
		count++;
	} while (ok && buf->id != expectedId && count < 5);

	ok = ok && buf->id == expectedId;

	if (!ok)
	{
		reply.printf("Message not received");
	}

	return ok;
}
#endif	// DUAL_CAN

#endif

// End
