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
# include <CanMessageGenericParser.h>
# include <Heating/Heat.h>

# if HAS_STALL_DETECT
#  include <Movement/StepperDrivers/SmartDrivers.h>			// for extern declaration of driverStallsToNotify
# endif
#endif

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
constexpr uint32_t MaxPort1BroadcastSendWait = 20;							// milliseconds

// Define how often we send time sync messages. This value and the time interval between sending broadcast status messages (currently 250ms) should be relatively prime.
// The reason is that if we try to send a time sync message just after a board has started broadcasting a status message, the time sync message will get delayed
// until the broadcast message finishes, which could be up to 600us at 1Mbps (the time taken to send a message with a 64-byte payload when not using BRS).
// When we used a 200us interval here, this meant that the same clash would occur 1 second later, and again 1 second after that.
// Using a value here that is relatively prime to 250ms avoids that happening. Alternatively we could add a random element to the interval.
constexpr uint32_t CanClockIntervalMillis = 211;

// Define the maximum time sync delay that we tolerate. Occasionally on the SAME70 we get spurious very long delays, so we must ignore those.
// CAN-FD packets have 42 header bits, up to 64*8 data bits and 45 trailer bits. The header and data parts may have added stuff bits.
// That's a maximum of 554 header+data bits plus up to 20% additional stuff bits, and 45 trailer bits including fixed stuff bits.
// So the maximum number of bits is less than 710, which takes 710us to transmit at 1Mbps. Allow an extra 5us for scheduling delays.
constexpr uint16_t MaxTimeSyncDelay = (uint16_t)MicrosecondsToStepClocks((42 + 64 * 8) * 1.2 + 45 + 5);	// the maximum normal delay before a CAN time sync message is sent, in step clocks

static_assert(MaxTimeSyncDelay >= 400 && MaxTimeSyncDelay <= 1000);			// check it's in the right ball park

#define USE_BIT_RATE_SWITCH		0

constexpr uint32_t MinBitRate = 15;											// MCP2542 has a minimum bite rate of 14.4kbps
constexpr uint32_t MaxBitRate = 5000;
constexpr uint32_t DefaultBitRate = 1000;
constexpr uint32_t DefaultClassicBitRate = 250;								// ODrives default to 250kbps

constexpr float MinSamplePoint = 0.5;
constexpr float MaxSamplePoint = 0.95;

constexpr float MinJumpWidth = 0.05;
constexpr float MaxJumpWidth = 0.5;

static uint32_t longestWaitTime = 0;
static uint16_t longestWaitMessageType = 0;

static uint32_t peakTimeSyncTxDelay = 0;

// Debug
static unsigned int goodTimeStamps = 0;
static unsigned int badTimeStamps = 0;
static unsigned int timeSyncMessagesSent = 0;
// End debug

// Internal port numbers are the indices into CanBusParams, not the CAN numbers on the board: on the MB6HC and MB6XD port 0 (can0dev, task CanRecv1)
// is the main bus on MCAN1, labelled CAN1, and port 1 (can1dev, task CanRecv0) is MCAN0, labelled CAN0. Parameter I and all messages to the user use the CAN numbers
constexpr size_t NumCanPorts = ARRAY_SIZE(CanBusParams);
static_assert(NumCanPorts == 1 + DUAL_CAN);

static Mutex transactionMutexes[NumCanPorts];								// a transaction on one port does not hold up one on the other

struct TimeSyncState
{
	uint32_t lastTimeSent = 0;
#if !SAME70
	uint16_t lastTxPreparedStamp = 0;
#endif
	volatile uint16_t txTimeStamp = 0;
	volatile bool gotTxTimeStamp = false;
	uint8_t marker = 0xFF;													// the marker we use to track time sync message transmit events
};

static TimeSyncState timeSyncStates[NumCanPorts];

static CanAddress myAddress =
#ifdef DUET3_ATE
						CanId::ATEMasterAddress;
#else
						CanId::MasterAddress;
#endif

static uint8_t fastDataRates[NumCanPorts] = { 0 };							// the fast data phase bit rate multiplier minus one. 0 means don't use BRS
static uint8_t dTseg1MinusOnes[NumCanPorts] = { 0 };						// the fast data rate sample point minus one

#if SUPPORT_REMOTE_COMMANDS
static unsigned int messagesIgnored = 0;
static bool inExpansionMode = false;
static bool inTestMode = false;
static bool mainBoardAcknowledgedAnnounce = false;

// Connection timeout handling after loss of time sync, see M959
constexpr uint32_t DefaultConnectionTimeoutMillis = 10000;

static uint32_t connectionTimeoutMillis = DefaultConnectionTimeoutMillis;
static bool hadSyncLock = false;											// true if we have had time sync lock since we switched to expansion mode
static bool syncLockLost = false;											// true while we are in a lost-sync episode
static bool timeoutActioned = false;										// true after we have switched the heaters off in the current lost-sync episode
static bool announceIsReconnect = false;									// true if the next announcement should carry the reconnect flag
static bool announceWasShutDown = false;									// true if the next announcement should report that the heaters were switched off
static unsigned int syncLossCount = 0;
static unsigned int timeoutShutdownCount = 0;
#endif

//#define CAN_DEBUG

// Define the memory configuration we want to use
constexpr CanDevice::Config Can0Config =
{
	.dataSize = 64,
	.numTxBuffers = 5,
	.txFifoSize = 16,
	.numRxBuffers =  0,
	.rxFifo0Size = 32,				// increased from 16 to help with accelerometer and closed loop data collection
	.rxFifo1Size = 16,
	.numShortFilterElements = 0,
#ifdef DUET3_ATE
	.numExtendedFilterElements = 4,
#else
	.numExtendedFilterElements = 3,
#endif
	.txEventFifoSize = 16
};

static_assert(Can0Config.IsValid());

// CAN buffer memory must be in the first 64Kb of RAM (SAME5x) or in non-cached RAM (SAME70), so put it in its own memory section
static uint32_t can0Memory[Can0Config.GetMemorySize()] __attribute__ ((section (".CanMessage")));

static CanDevice *_ecv_null can0dev = nullptr;

static unsigned int txTimeouts[NumCanPorts][Can0Config.numTxBuffers + 1] = { { 0 } };
static uint32_t lastCancelledIds[NumCanPorts] = { 0 };

#if DUAL_CAN

// The second port is either a CAN-FD expansion bus like the first one or a classic CAN bus for ODrives, so it needs the same layout plus a short ID filter
constexpr CanDevice::Config Can1Config =
{
	.dataSize = 64,
	.numTxBuffers = 5,
	.txFifoSize = 16,
	.numRxBuffers =  0,
	.rxFifo0Size = 32,
	.rxFifo1Size = 16,
	.numShortFilterElements = 1,
	.numExtendedFilterElements = Can0Config.numExtendedFilterElements,
	.txEventFifoSize = 16
};

static_assert(Can1Config.IsValid());

// CAN buffer memory must be in the first 64Kb of RAM (SAME5x) or in non-cached RAM (SAME70), so put it in its own segment
static uint32_t can1Memory[Can1Config.GetMemorySize()] __attribute__ ((section (".CanMessage")));

static CanDevice *_ecv_null can1dev = nullptr;

constexpr uint32_t ConflictReportIntervalMillis = 10000;
constexpr uint32_t RestartGraceMillis = 1000;								// expansion boards keep running for 200ms after accepting a reset, and an unacknowledged one keeps announcing

// Bitmap with one bit per CAN address, updated by the receiver tasks and read by any task without a lock
class AddressBitmap
{
public:
	bool IsSet(CanAddress addr) const noexcept { return (words[addr >> 5].load(std::memory_order_relaxed) & Mask(addr)) != 0; }
	void Set(CanAddress addr) noexcept { words[addr >> 5].fetch_or(Mask(addr)); }
	void Clear(CanAddress addr) noexcept { words[addr >> 5].fetch_and(~Mask(addr)); }
	void ClearAll() noexcept;
	bool IsEmpty() const noexcept;
	unsigned int Count() const noexcept;
	uint32_t GetWord(size_t index) const noexcept { return words[index].load(std::memory_order_relaxed); }

	static constexpr size_t NumWords = (CanId::MaxCanAddress >> 5) + 1;

private:
	static constexpr uint32_t Mask(CanAddress addr) noexcept { return 1u << (addr & 31); }

	std::atomic<uint32_t> words[NumWords] = { };
};

void AddressBitmap::ClearAll() noexcept
{
	for (std::atomic<uint32_t>& w : words)
	{
		w.store(0);
	}
}

bool AddressBitmap::IsEmpty() const noexcept
{
	for (const std::atomic<uint32_t>& w : words)
	{
		if (w.load(std::memory_order_relaxed) != 0)
		{
			return false;
		}
	}
	return true;
}

unsigned int AddressBitmap::Count() const noexcept
{
	unsigned int count = 0;
	for (const std::atomic<uint32_t>& w : words)
	{
		count += __builtin_popcount(w.load(std::memory_order_relaxed));
	}
	return count;
}

enum class Port1Mode : uint8_t { off, expansion, classic };

static volatile Port1Mode port1Mode = Port1Mode::off;						// CAN0 stays off until M953 I0 brings it up
static AddressBitmap learnedAddresses[NumCanPorts];							// the port each board was first heard on, at most one bit set per address
static AddressBitmap conflictingAddresses;									// addresses also heard announcing on the port they were not learned on
static AddressBitmap restartingAddresses;									// conflicts cleared by M999 less than RestartGraceMillis ago
static volatile uint32_t whenLastRestarted = 0;
static unsigned int wrongPortMessages = 0;
static uint32_t whenLastConflictReport = 0;
static uint32_t reportedConflicts[AddressBitmap::NumWords] = { 0 };			// only used by ReportConflicts

#endif

constexpr size_t NoPort = NumCanPorts;										// returned by GetPortFor when a board has not been heard on any port

#if DUAL_CAN

static bool IsPort1Up() noexcept
{
	return (can1dev != nullptr) && (port1Mode == Port1Mode::expansion);
}

// Port 1 only transmits once a board has been heard on it, because unacknowledged messages on an empty bus would block its transmit buffers.
// A conflict means that there is a board on each bus
static bool IsPort1Active() noexcept
{
	return IsPort1Up() && (!learnedAddresses[1].IsEmpty() || !conflictingAddresses.IsEmpty());
}

#endif

static CanDevice *_ecv_null GetPortDevice(size_t port) noexcept
{
#if DUAL_CAN
	return (port == 0) ? can0dev : (port == 1 && IsPort1Active()) ? can1dev : nullptr;
#else
	return (port == 0) ? can0dev : nullptr;
#endif
}

// Return the port that messages to the specified board address must use, or NoPort if both buses are up and the board has not been heard on either.
// The firmware update address is never learned, so a 3HC with its address switches at zero can only be configured on port 0. On port 1 it can still update its firmware, because responses go back on the bus the request came from
static size_t GetPortFor(CanAddress address) noexcept
{
#if DUAL_CAN
	if (IsPort1Up() && address != CanId::BroadcastAddress && address != CanId::Exp3HCFirmwareUpdateAddress)
	{
		return learnedAddresses[1].IsSet(address) ? 1
				: learnedAddresses[0].IsSet(address) ? 0
					: NoPort;
	}
#endif
	return 0;
}

static unsigned int GetCanNumber(size_t port) noexcept
{
	return CanBusParams[port].instanceNumber;
}

// Convert the CAN number printed on the board to our port number
static size_t GetPortFromCanNumber(const GCodeBuffer& gb, uint32_t canNumber) THROWS(GCodeException)
{
	for (size_t port = 0; port < NumCanPorts; port++)
	{
		if (CanBusParams[port].instanceNumber == canNumber)
		{
			return port;
		}
	}
	throw GCodeException(&gb, -1, "This board has no CAN%u", canNumber);
}

// Transmit buffer usage. All dedicated buffer numbers must be < Can0Config.numTxBuffers.
constexpr auto TxBufferIndexUrgent = CanDevice::TxBufferNumber::buffer0;
constexpr auto TxBufferIndexTimeSync = CanDevice::TxBufferNumber::buffer1;
constexpr auto TxBufferIndexRequest = CanDevice::TxBufferNumber::buffer2;
constexpr auto TxBufferIndexResponse = CanDevice::TxBufferNumber::buffer3;
constexpr auto TxBufferIndexBroadcast = CanDevice::TxBufferNumber::buffer4;
constexpr auto TxBufferIndexMotion = CanDevice::TxBufferNumber::fifo;				// we send lots of movement messages so use the FIFO for them

// Receive buffer/FIFO usage. All dedicated buffer numbers must be < Can0Config.numRxBuffers.
constexpr auto RxBufferIndexBroadcast = CanDevice::RxBufferNumber::fifo0;
constexpr auto RxBufferIndexRequest = CanDevice::RxBufferNumber::fifo0;
constexpr auto RxBufferIndexResponse = CanDevice::RxBufferNumber::fifo1;

// CanSender management task
constexpr size_t CanSenderTaskStackWords = 400;
static Task<CanSenderTaskStackWords> canSenderTask;

constexpr size_t CanReceiverTaskStackWords = 1000;
static Task<CanReceiverTaskStackWords> canReceiverTask;
#if DUAL_CAN
static Task<CanReceiverTaskStackWords> canReceiver1Task;
static Mutex receiveMutex;													// the command processor expects to handle one received message at a time; take it after a transaction mutex and before port1Lock
static ReadWriteLock port1Lock;												// write-locked together with port 1's transaction mutex while port 1 is reconfigured, read-locked by senders outside transactions
#endif

constexpr size_t CanClockTaskStackWords = 400;			// used to be 300 but RD had a stack overflow
static Task<CanSenderTaskStackWords> canClockTask;

static CanMessageBuffer *_ecv_null volatile pendingMotionBuffers = nullptr;
static CanMessageBuffer * volatile lastMotionBuffer;	// only valid when pendingBuffers != nullptr

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

// Called frequently from Platform::Spin, so that we change the LED state as close to the step clock transition as the expansion boards do
void CanInterface::UpdateStatusLed() noexcept
{
#if SUPPORT_REMOTE_COMMANDS
	if (inExpansionMode)
	{
		const bool synced = StepTimer::CheckSynced();
		CanInterface::UpdateSyncLockState(synced);
		if (!synced)
		{
			// Blink fast to show that we haven't established clock sync with the master
			reprap.GetPlatform().SetDiagLed((StepTimer::GetTimerTicks() & (1u << 17)) != 0);
			return;
		}
	}

	const uint32_t stepClocks = StepTimer::GetMasterTime();
#else
	const uint32_t stepClocks = StepTimer::GetTimerTicks();
#endif

#if SUPPORT_MULTICAST_DISCOVERY
	if (identifying)
	{
		if (identTotalClocks != 0 && stepClocks - identInitialClocks >= identTotalClocks)
		{
			identifying = false;						// stop identifying
		}
		else
		{
			reprap.GetPlatform().SetDiagLed((stepClocks & (1u << 17)) != 0);
			return;
		}
	}
#endif

	// Blink the LED at about 1Hz. Duet 3 expansion boards will blink in sync when they have established clock sync with us
	reprap.GetPlatform().SetDiagLed((stepClocks & (1u << 19)) != 0);
}

static void InitReceiveFilters(CanDevice *dev) noexcept
{
	// Set up a filter to receive all request messages addressed to us in FIFO 0
	dev->SetExtendedFilterElement(0, CanDevice::RxBufferNumber::fifo0,
										CanInterface::GetCanAddress() << CanId::DstAddressShift,
										(CanId::BoardAddressMask << CanId::DstAddressShift) | CanId::ResponseBit);

	// Set up a filter to receive all broadcast messages also in FIFO 0
	dev->SetExtendedFilterElement(1, CanDevice::RxBufferNumber::fifo0,
										CanId::BroadcastAddress << CanId::DstAddressShift,
										CanId::BoardAddressMask << CanId::DstAddressShift);

	// Set up a filter to receive response messages in FIFO 1
	dev->SetExtendedFilterElement(2, RxBufferIndexResponse,
										(CanInterface::GetCanAddress() << CanId::DstAddressShift) | CanId::ResponseBit,
										(CanId::BoardAddressMask << CanId::DstAddressShift) | CanId::ResponseBit);
# ifdef DUET3_ATE
	// Also respond to requests addressed to board 0 so we can update firmware on ATE boards
	dev->SetExtendedFilterElement(3, CanDevice::RxBufferNumber::fifo0,
										CanId::MasterAddress << CanId::DstAddressShift,
										(CanId::BoardAddressMask << CanId::DstAddressShift) | CanId::ResponseBit);
# endif
}

#if DUAL_CAN

// Set up port 1 either as an expansion bus or as a classic CAN bus. Classic messages go to FIFO 1, which is not read by the CanReceiver task.
// The caller must hold port 1's transaction mutex, receiveMutex and the write lock on port1Lock so that nothing is being sent or received on port 1
static void InitPort1(bool classic, const CanTiming& timing) noexcept
{
	can1dev->Disable();														// this discards pending transmissions, see the MCAN Software Initialization section of the SAME70 datasheet
	timeSyncStates[1].gotTxTimeStamp = false;
	if (classic)
	{
		for (unsigned int i = 0; i < Can1Config.numExtendedFilterElements; i++)
		{
			can1dev->DisableExtendedFilterElement(i);
		}
		can1dev->SetShortFilterElement(0, CanDevice::RxBufferNumber::fifo1, 0, 0);
	}
	else
	{
		can1dev->DisableShortFilterElement(0);
		InitReceiveFilters(can1dev);
	}
	port1Mode = classic ? Port1Mode::classic : Port1Mode::expansion;
	fastDataRates[1] = dTseg1MinusOnes[1] = 0;
	can1dev->ChangeLocalCanTiming(timing);
}

#endif

// This is the function called by the transmit event handler when the message marker is nonzero
template<size_t Port> static void TxCallback(uint8_t marker, CanId id, uint16_t timeStamp) noexcept
{
	TimeSyncState& state = timeSyncStates[Port];
	if (marker == state.marker)
	{
		state.txTimeStamp = timeStamp;
		state.gotTxTimeStamp = true;
		goodTimeStamps++;
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

	for (size_t port = 0; port < NumCanPorts; port++)
	{
		transactionMutexes[port].Create((port == 0) ? "CanTrans" : "CanTransAux");
#if SAME70
		SetPinFunction(CanBusParams[port].txPin, CanBusParams[port].txPinFunction);
		SetPinFunction(CanBusParams[port].rxPin, CanBusParams[port].rxPinFunction);
#else
		SetPinFunction(CanBusParams[port].txPin, CanBusParams[port].pinsFunction);
		SetPinFunction(CanBusParams[port].rxPin, CanBusParams[port].pinsFunction);
#endif
	}

#if SAME70
	pmc_enable_upll_clock();			// configure_mcan sets up PCLK5 to be the UPLL divided by something, so make sure the UPLL is running
#endif

	// Initialise the CAN hardware
	CanTiming timing;
	timing.SetDefaults(CanTiming::DefaultCanBitRate);
	can0dev = CanDevice::Init(0, CanBusParams[0].instanceNumber, Can0Config, can0Memory, timing, nullptr,
#if EXTERNAL_CAN_TIMESTAMP_COUNTER
								nullptr
#else
								StepTimer::CanTimingMessageReceived
#endif
							 );
	InitReceiveFilters(can0dev);
	can0dev->Enable();

#if DUAL_CAN
	receiveMutex.Create("CanRecv");
	can1dev = CanDevice::Init(1, CanBusParams[1].instanceNumber, Can1Config, can1Memory, timing, nullptr, nullptr);		// left disabled until M953 I0, so that a classic CAN bus never sees our CAN-FD frames
#endif

	CanMotion::Init();

	// Create the task that sends CAN messages
	canClockTask.Create(CanClockLoop, "CanClock", nullptr, TaskPriority::CanClockPriority);
	canSenderTask.Create(CanSenderLoop, "CanSender", nullptr, TaskPriority::CanSenderPriority);
#if DUAL_CAN
	canReceiverTask.Create(CanReceiverLoop, "CanRecv1", (void *)0, TaskPriority::CanReceiverPriority);			// task names are cut to 9 characters
	canReceiver1Task.Create(CanReceiverLoop, "CanRecv0", (void *)1, TaskPriority::CanReceiverPriority);
#else
	canReceiverTask.Create(CanReceiverLoop, "CanReceiver", (void *)0, TaskPriority::CanReceiverPriority);
#endif
}

void CanInterface::Shutdown() noexcept
{
	canClockTask.TerminateAndUnlink();
	canSenderTask.TerminateAndUnlink();
	canReceiverTask.TerminateAndUnlink();
#if DUAL_CAN
	canReceiver1Task.TerminateAndUnlink();

	if (can1dev != nullptr)
	{
		can1dev->DeInit();
		can1dev = nullptr;
	}
#endif

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
	InitReceiveFilters(can0dev);
	can0dev->Enable();
}

void CanInterface::SwitchToExpansionMode(CanAddress addr, bool useTestMode) noexcept
{
	TaskCriticalSectionLocker lock;

	myAddress = addr;
	inExpansionMode = true;
	inTestMode = useTestMode;
	reprap.GetGCodes().SwitchToExpansionMode();
	reprap.GetMove().SwitchToExpansionMode();
	ReInit();										// reset the CAN filters to account for our new CAN address
}

// Send an announcement message if we haven't had an announce acknowledgement from a main board. On return the buffer is available to use again.
void CanInterface::SendAnnounce(CanMessageBuffer *buf) noexcept
{
	if (inExpansionMode && !mainBoardAcknowledgedAnnounce)
	{
		auto msg = buf->SetupBroadcastMessage<CanMessageAnnounceV1>(myAddress);
		msg->timeSinceStarted = millis();
		msg->numDrivers = NumDirectDrivers;
		msg->usesUf2Binary = BOARD_USES_UF2_BINARY;
		msg->isReconnect = announceIsReconnect;
		msg->wasShutDown = announceWasShutDown;
		msg->noSmartDrivers = !HAS_SMART_DRIVERS;
		memcpy(msg->uniqueId, reprap.GetPlatform().GetUniqueId().GetRaw(), sizeof(msg->uniqueId));
		// Note, board type name, firmware version, firmware date and firmware time are limited to 43 characters in the new
		// We use vertical-bar to separate the three fields: board type, firmware version, date/time
		SafeSnprintf(msg->boardTypeAndFirmwareVersion, ARRAY_SIZE(msg->boardTypeAndFirmwareVersion), "%s|%s|%s", BOARD_SHORT_NAME, VERSION, DateTimeText);
		buf->dataLength = msg->GetActualDataLength();
		SendMessageNoReplyNoFree(buf);
	}
}

// Send an event. The text will be truncated if it is longer than 55 characters.
void CanInterface::RaiseEvent(EventType type, uint16_t param, uint8_t device, const char *_ecv_array format, va_list vargs) noexcept
{
	CanMessageBuffer buf;
	auto msg = buf.SetupRequestMessageNoRid<CanMessageEvent>(GetCanAddress(), GetCurrentMasterAddress());
	msg->eventType = type.ToBaseType();;
	msg->deviceNumber = device;
	msg->eventParam = param;
	SafeVsnprintf(msg->text, ARRAY_SIZE(msg->text), format, vargs);
	buf.dataLength = msg->GetActualDataLength();
	CanInterface::SendMessageNoReplyNoFree(&buf);
}

void CanInterface::MainBoardAcknowledgedAnnounce() noexcept
{
	mainBoardAcknowledgedAnnounce = true;
	announceIsReconnect = announceWasShutDown = false;
}

// Called regularly from Platform::Spin while we are in expansion mode with the current time sync state. If we had time sync lock and lost it for longer than the
// connection timeout, switch the heaters off. When lock is regained after a loss, re-announce ourselves so that the main board learns that we reconnected
// (rather than restarted) and whether the heaters were switched off meanwhile
void CanInterface::UpdateSyncLockState(bool synced) noexcept
{
	if (synced)
	{
		if (syncLockLost)
		{
			announceIsReconnect = true;
			announceWasShutDown = timeoutActioned;
			mainBoardAcknowledgedAnnounce = false;
			syncLockLost = false;
			timeoutActioned = false;
		}
		hadSyncLock = true;
	}
	else if (hadSyncLock)
	{
		if (!syncLockLost)
		{
			syncLockLost = true;
			syncLossCount++;
		}
		if (!timeoutActioned && millis() - StepTimer::GetWhenLastSynced() > connectionTimeoutMillis)
		{
			reprap.GetHeat().SwitchOffAll(true);
			timeoutActioned = true;
			timeoutShutdownCount++;
		}
	}
}

// Process a M959 message from the main board setting or reporting the connection timeout
GCodeResult CanInterface::ProcessM959(const CanMessageGeneric& msg, const StringRef& reply) noexcept
{
	CanMessageGenericParser parser(msg, M959Params);
	uint16_t timeout;
	if (parser.GetUintParam('T', timeout))
	{
		connectionTimeoutMillis = (uint32_t)timeout * 1000;
		return GCodeResult::ok;
	}
	reply.printf("Board %u connection timeout %" PRIu32 " seconds", GetCanAddress(), connectionTimeoutMillis / 1000);
	return GCodeResult::ok;
}

#endif

// Allocate a CAN request ID
// Currently we reserve the top bit of the 12-bit request ID so that CanRequestIdAcceptAlways is distinct from any genuine request ID.
// Currently we use a single RID sequence for all destination addresses. In future we may use a separate sequence for each address.
// The message buffer is provided so that if the board is not known, we can use the buffer to send a message to it to announce ourselves, but this is not yet implemented
CanRequestId CanInterface::AllocateRequestId(CanAddress destination, CanMessageBuffer *buf) noexcept
{
	static std::atomic<uint16_t> rid = 0;
	return rid.fetch_add(1) & CanRequestIdMask;
}

// Allocate a CAN message buffer, throw if failed
CanMessageBuffer *CanInterface::AllocateBuffer(const GCodeBuffer *_ecv_null gb) THROWS(GCodeException)
{
	CanMessageBuffer *_ecv_null const buf = CanMessageBuffer::Allocate();
	if (buf == nullptr)
	{
		throw GCodeException(gb, -1, NoCanBufferMessage);
	}
	return _ecv_not_null(buf);
}

uint16_t CanInterface::GetTimeStampCounter() noexcept
{
	return can0dev->ReadTimeStampCounter();
}

#if !EXTERNAL_CAN_TIMESTAMP_COUNTER

uint16_t CanInterface::GetTimeStampPeriod() noexcept
{
	return can0dev->GetTimeStampPeriod();
}

#endif

// Convert a 16-bit timestamp in a received message to 32-bits.
// We expect the time stamp to be up to a few milliseconds old. If that's not the case, ignore the timestamp in the message and return the current master time.
uint32_t CanInterface::Convert16bitReceivedTimeStampTo32bits(uint16_t ts) noexcept
{
	const uint32_t now = StepTimer::GetTimerTicks();
	const uint16_t delay = (uint16_t)now - ts;
#if 0	//DEBUG
	debugPrintf("Delay=%u\n", delay);
#endif
	return (delay < MillisToStepClocks(10))					// if the time stamp is less than 10ms old
		? now - (uint32_t)delay
			: now;											// time stamp negative or unreliable so ignore it
}

static void SendCanMessageOnDevice(size_t port, CanDevice *_ecv_null dev, CanDevice::TxBufferNumber whichBuffer, uint32_t timeout, CanMessageBuffer *buffer) noexcept
{
	if (dev != nullptr)
	{
		const uint32_t cancelledId = dev->SendMessage(whichBuffer, timeout, buffer);
		if (cancelledId != 0)
		{
			txTimeouts[port][(unsigned int)whichBuffer]++;
			lastCancelledIds[port] = cancelledId;
		}
	}
}

static void SendCanMessageOnPort(size_t port, CanDevice::TxBufferNumber whichBuffer, uint32_t timeout, CanMessageBuffer *buffer) noexcept
{
	SendCanMessageOnDevice(port, GetPortDevice(port), whichBuffer, timeout, buffer);
}

// Send a message on the CAN FD port that serves its destination and record any errors. Broadcasts go out on every CAN FD port
static void SendCanMessage(CanDevice::TxBufferNumber whichBuffer, uint32_t timeout, CanMessageBuffer *buffer) noexcept
{
	const CanAddress dst = buffer->id.Dst();
#if DUAL_CAN
	ReadLocker lock(port1Lock);
#endif
	SendCanMessageOnPort(GetPortFor(dst), whichBuffer, timeout, buffer);
#if DUAL_CAN
	if (dst == CanId::BroadcastAddress)
	{
		// A message nobody acknowledges blocks its buffer until it is cancelled, so a dead port 1 bus must not hold up broadcasts for long
		SendCanMessageOnPort(1, whichBuffer, min(timeout, MaxPort1BroadcastSendWait), buffer);
	}
#endif
}

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
			auto msg = buf.SetupRequestMessageNoRid<CanMessageInputChangedV2>(CanInterface::GetCanAddress(), CanInterface::GetCurrentMasterAddress());
			msg->states = 0;
			msg->numHandles = 0;

#if HAS_STALL_DETECT
			// Start by adding any stall detect endstops pending. Do this first so that it must succeed.
			const uint16_t stallNotifications = SmartDrivers::driverStallsToNotify.exchange(0);
			if (stallNotifications != 0)
			{
				constexpr RemoteInputHandle h(RemoteInputHandle::typeStallEndstop, 0, 0);
				(void)msg->AddEntry(h.asU16(), StepTimer::GetMasterTime(), (uint32_t)stallNotifications, true);
			}
#endif
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
				CanMessageBuffer *_ecv_null const urgentMessage = CanMotion::GetUrgentMessage();
				if (urgentMessage != nullptr)
				{
					SendCanMessage(TxBufferIndexUrgent, MaxUrgentSendWait, urgentMessage);
				}
				else if (pendingMotionBuffers != nullptr)
				{
					CanMessageBuffer *buf;
					{
						TaskCriticalSectionLocker lock;
						buf = _ecv_not_null(pendingMotionBuffers);
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

static CanDevice::TxEventCallbackFunction GetTxCallback(size_t port) noexcept
{
#if DUAL_CAN
	return (port == 0) ? TxCallback<0> : TxCallback<1>;
#else
	return TxCallback<0>;
#endif
}

// Complete the time sync message in the buffer for one port and send it
static void SendTimeSync(size_t port, CanDevice *dev, CanMessageBuffer& buf) noexcept
{
	TimeSyncState& state = timeSyncStates[port];
	CanMessageTimeSync& msg = buf.msg.sync;
	msg.fastDataRate = fastDataRates[port];
	msg.tseg1Minus1 = dTseg1MinusOnes[port];
	msg.lastTimeSent = state.lastTimeSent;
	msg.lastTimeAcknowledgeDelay = 0;										// assume we don't have the transmit delay available

	state.marker = ((state.marker + 1) & 0x0F) | 0xA0;
	buf.marker = state.marker;
	buf.reportInFifo = 1;

	if (state.gotTxTimeStamp)
	{
		// Calculate the delay in sending the last time sync message, in step clocks
# if SAME70
		// On the SAME70 the step clock is also the external time stamp counter
		const uint32_t timeSyncTxDelay = (state.txTimeStamp - (uint16_t)state.lastTimeSent) & 0xFFFF;
# else
		// On the SAME5x the time stamp counter counts CAN bit times. The step clock is the CAN clock divided by 64
		const uint32_t timeSyncTxDelay = ((uint32_t)((state.txTimeStamp - state.lastTxPreparedStamp) & 0xFFFF) * dev->GetTimeStampPeriod()) >> 6;
# endif
		if (timeSyncTxDelay > peakTimeSyncTxDelay)
		{
			peakTimeSyncTxDelay = timeSyncTxDelay;
		}

		// Occasionally on the SAME70 we get very large delays reported. These delays are not genuine
		if (timeSyncTxDelay < MaxTimeSyncDelay)
		{
			msg.lastTimeAcknowledgeDelay = timeSyncTxDelay;
		}
		state.gotTxTimeStamp = false;
	}

#if SAME70
	state.lastTimeSent = StepTimer::GetTimerTicks();
#else
	{
		AtomicCriticalSectionLocker lock;
		state.lastTimeSent = StepTimer::GetTimerTicksWhenInterruptsDisabled();
		state.lastTxPreparedStamp = dev->ReadTimeStampCounter();
	}
#endif
	msg.timeSent = state.lastTimeSent;
	SendCanMessageOnPort(port, TxBufferIndexTimeSync, 0, &buf);
	timeSyncMessagesSent++;
}

// Check that the time sync message was sent and get its time stamp
static void CheckTimeSyncSent(size_t port, CanDevice *dev) noexcept
{
	if (dev->IsSpaceAvailable(TxBufferIndexTimeSync, 0))					// if the buffer is free already then the message was sent
	{
		dev->PollTxEventFifo(GetTxCallback(port));
	}
	else
	{
		(void)dev->IsSpaceAvailable(TxBufferIndexTimeSync, MaxTimeSyncSendWait);		// free the buffer
		dev->PollTxEventFifo(GetTxCallback(port));						// empty the fifo
		timeSyncStates[port].gotTxTimeStamp = false;					// ignore any values read from it
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
		msg->isPrinting = reprap.GetGCodes().IsReallyPrinting();

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

		{
#if DUAL_CAN
			ReadLocker lock(port1Lock);
#endif
			for (size_t port = 0; port < NumCanPorts; port++)
			{
				CanDevice *_ecv_null const dev = GetPortDevice(port);
				if (dev != nullptr)
				{
					SendTimeSync(port, dev, buf);
				}
			}
		}

		// Delay until it is time again
		vTaskDelayUntil(&lastWakeTime, CanClockIntervalMillis);

#if SUPPORT_REMOTE_COMMANDS
		if (inExpansionMode)
		{
			TaskBase::GetCallerTaskHandle()->TerminateAndUnlink();			// once in expansion mode we can't revert to main board mode, so we don't need this task any more
		}
#endif

#if DUAL_CAN
		ReadLocker lock(port1Lock);
#endif
		for (size_t port = 0; port < NumCanPorts; port++)
		{
			CanDevice *_ecv_null const dev = GetPortDevice(port);
			if (dev != nullptr)
			{
				CheckTimeSyncSent(port, dev);
			}
		}
	}
}

// Members of namespace CanInterface, and associated local functions

template<class T> static GCodeResult SetRemoteDriverValues(const CanDriversData<T>& data, const StringRef& reply, CanMessageType mt) noexcept
{
	// This code sends only one message per CAN board, so sizeof<T> must be small enough so that the message fits in a single CAN packet
	static_assert(CanMessageMultipleDrivesRequest<T>::MaxDrivesPerMessage() >= MaxLinearDriversPerCanSlave);

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
		CanMessageBuffer *_ecv_null const buf = CanMessageBuffer::Allocate();
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
		buf->dataLength = msg->GetActualDataLength();
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
		CanMessageBuffer *_ecv_null const buf = CanMessageBuffer::Allocate();
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
		buf->dataLength = msg->GetActualDataLength();
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
GCodeResult CanInterface::SendRequestAndGetStandardReply(CanMessageBuffer *buf, CanRequestId rid, const StringRef& reply, uint8_t *_ecv_null extra, uint32_t *_ecv_null words) noexcept
{
	return SendRequestAndGetCustomReply(buf, rid, reply, extra, words, CanMessageType::unusedMessageType, [](const CanMessageBuffer*) noexcept->void { });
}

// Send a request to an expansion board on the specified port, or on the port it was heard on if 'requestedPort' is PortOfBoard, and append the response to 'reply'.
// The response may either be a standard reply or 'replyType'
static GCodeResult SendRequestOnPortAndGetCustomReply(size_t requestedPort, CanMessageBuffer *buf, CanRequestId rid, const StringRef& reply, uint8_t *_ecv_null extra, uint32_t *_ecv_null words, CanMessageType replyType, function_ref_noexcept<void(const CanMessageBuffer*) noexcept> callback) noexcept
{
	const CanAddress dest = buf->id.Dst();
	const CanMessageType msgType = buf->id.MsgType();								// save for possible error message

#if DUAL_CAN
	// A board can still be announcing itself, e.g. just after M953 brought CAN0 up, so give it the usual response time to be heard
	if (requestedPort == CanInterface::PortOfBoard)
	{
		const uint32_t whenStartedWaiting = millis();
		while (GetPortFor(dest) == NoPort && millis() - whenStartedWaiting < CanInterface::UsualResponseTimeout)
		{
			delay(2);
		}
	}
#endif

	const size_t port = (requestedPort == CanInterface::PortOfBoard) ? GetPortFor(dest) : requestedPort;
	if (port == NoPort)
	{
		CanMessageBuffer::Free(buf);
		reply.lcatf("Board %u has not been heard on CAN%u or CAN%u", dest, GetCanNumber(0), GetCanNumber(1));
		return GCodeResult::error;
	}

	{
		// This code isn't re-entrant and it can get called from a task other than Main to shut the system down, so we need to use a mutex.
		// Port 1 is only reconfigured while holding its mutex as well, so its mode can't change during the transaction. If the board was forgotten
		// or moved to the other bus while we waited for the mutex, the request times out like one sent to a board that has just been unplugged.
		MutexLocker lock(transactionMutexes[port]);

		CanDevice *_ecv_null const dev = GetPortDevice(port);
		if (dev == nullptr)
		{
			// Transactions sometimes get requested after we have shut down CAN, e.g. when we destroy filament monitors
			CanMessageBuffer::Free(buf);
			reply.lcatf("CAN%u is not available", GetCanNumber(port));
			return GCodeResult::error;
		}

		SendCanMessageOnPort(port, TxBufferIndexRequest, MaxRequestSendWait, buf);
		reprap.GetPlatform().OnProcessingCanMessage();

		const uint32_t whenStartedWaiting = millis();
		uint32_t timeWaiting = 0;
		unsigned int fragmentsReceived = 0;
		for (;;)
		{
			const uint32_t timeout = (timeWaiting < CanInterface::UsualResponseTimeout) ? CanInterface::UsualResponseTimeout - timeWaiting : 1;
			if (!dev->ReceiveMessage(RxBufferIndexResponse, timeout, buf))
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
						reply.lcatn(buf->msg.standardReply.GetText(), textLength);
					}
					if (extra != nullptr)
					{
						*extra = buf->msg.standardReply.extra;
					}
					if (words != nullptr)
					{
						// Words the reply didn't carry (e.g. older expansion firmware) read as zero
						for (size_t i = 0; i < CanMessageStandardReply::MaxNumWords; i++)
						{
							words[i] = (i < buf->msg.standardReply.numWords) ? buf->msg.standardReply.GetWord(i) : 0;
						}
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
					reply.catn(buf->msg.standardReply.GetText(), buf->msg.standardReply.GetTextLength(buf->dataLength));
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
	reply.lcatf("Timed out awaiting response from board %u to request type %u", dest, (unsigned int)msgType);
	return GCodeResult::canResponseTimeout;
}

GCodeResult CanInterface::SendRequestAndGetCustomReply(CanMessageBuffer *buf, CanRequestId rid, const StringRef& reply, uint8_t *_ecv_null extra, uint32_t *_ecv_null words, CanMessageType replyType, function_ref_noexcept<void(const CanMessageBuffer*) noexcept> callback) noexcept
{
	return SendRequestOnPortAndGetCustomReply(PortOfBoard, buf, rid, reply, extra, words, replyType, callback);
}

GCodeResult CanInterface::SendRequestOnPortAndGetStandardReply(size_t port, CanMessageBuffer *buf, CanRequestId rid, const StringRef& reply) noexcept
{
	return SendRequestOnPortAndGetCustomReply(port, buf, rid, reply, nullptr, nullptr, CanMessageType::unusedMessageType, [](const CanMessageBuffer*) noexcept->void { });
}

// Send a response to an expansion board and free the buffer
void CanInterface::SendResponseNoFree(CanMessageBuffer *buf) noexcept
{
#if DUAL_CAN
	// Responses are only sent while processing a received message, so they go back on the bus the request came from.
	// That bus has a board on it even if we have not learned its address, e.g. a 3HC bootloader using the firmware update address
	ReadLocker lock(port1Lock);
	SendCanMessageOnDevice(buf->bus, (buf->bus == 0) ? can0dev : IsPort1Up() ? can1dev : nullptr, TxBufferIndexResponse, MaxResponseSendWait, buf);
#else
	SendCanMessage(TxBufferIndexResponse, MaxResponseSendWait, buf);
#endif
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

// Return the CAN number printed on the board of the bus the expansion board was heard on, or -1 if it is not known or the address is in conflict
int CanInterface::GetCanBusNumber(CanAddress address) noexcept
{
#if DUAL_CAN
	if (address > CanId::MaxCanAddress || conflictingAddresses.IsSet(address))
	{
		return -1;
	}
	if (learnedAddresses[1].IsSet(address))
	{
		return (int)GetCanNumber(1);
	}
	if (!learnedAddresses[0].IsSet(address))
	{
		return -1;
	}
#endif
	return (int)GetCanNumber(0);
}

// Get the port that an M952 or M999 for an expansion board must use from its I parameter.
// I is mandatory while the address is in conflict. Return PortOfBoard if I is absent
size_t CanInterface::GetPortForBoardCommand(GCodeBuffer& gb, CanAddress address) THROWS(GCodeException)
{
	if (gb.Seen('I'))
	{
		const size_t port = GetPortFromCanNumber(gb, gb.GetUIValue());
#if DUAL_CAN
		if (!learnedAddresses[port].IsSet(address) && !conflictingAddresses.IsSet(address))
		{
			throw GCodeException(&gb, -1, "Board %u has not been heard on this CAN bus", address);
		}
#endif
		return port;
	}
#if DUAL_CAN
	if (conflictingAddresses.IsSet(address))
	{
		throw GCodeException(&gb, -1, "Board address %u is used on two CAN buses, use parameter I to select the bus", address);
	}
#endif
	return PortOfBoard;
}

// Called after the board at 'address' on 'port' has accepted a reset request, or when no board answered one sent on a specific port, e.g. because the board owning a conflicted address was unplugged.
// If the address was in conflict, the board on the other port now owns it. Frames the reset board sent before it restarted are then dropped
// because they come from the wrong port, and only an announcement from it under the same address after RestartGraceMillis restores the conflict.
// Otherwise forget the port, because the board may come back with a new address.
void CanInterface::RemoteBoardRestarted(CanAddress address, size_t port) noexcept
{
#if DUAL_CAN
	if (can0dev == nullptr)
	{
		return;														// after Shutdown a receiver task may have been deleted while it held receiveMutex
	}
	if (address <= CanId::MaxCanAddress && port != PortOfBoard)
	{
		MutexLocker lock(receiveMutex);
		if (conflictingAddresses.IsSet(address))
		{
			learnedAddresses[1 - port].Set(address);
			learnedAddresses[port].Clear(address);
			whenLastRestarted = millis();
			restartingAddresses.Set(address);
			conflictingAddresses.Clear(address);
			reprap.BoardsUpdated();
			return;
		}
	}
	ForgetBoard(address);
#endif
}

// Forget which port a board is on unless its address is in conflict, so that a board moved to the other bus is learned there
void CanInterface::ForgetBoard(CanAddress address) noexcept
{
#if DUAL_CAN
	if (address <= CanId::MaxCanAddress && can0dev != nullptr)		// after Shutdown a receiver task may have been deleted while it held receiveMutex
	{
		MutexLocker lock(receiveMutex);
		if (!conflictingAddresses.IsSet(address))
		{
			learnedAddresses[0].Clear(address);
			learnedAddresses[1].Clear(address);
			reprap.BoardsUpdated();
		}
	}
#endif
}

#if DUAL_CAN

// Report the addresses in conflict as soon as a new one shows up and then every ConflictReportIntervalMillis
void CanInterface::ReportConflicts() noexcept
{
	if (can0dev == nullptr)
	{
		return;														// after Shutdown a receiver task may have been deleted while it held receiveMutex
	}

	if (!restartingAddresses.IsEmpty() && millis() - whenLastRestarted >= RestartGraceMillis)
	{
		MutexLocker lock(receiveMutex);
		if (millis() - whenLastRestarted >= RestartGraceMillis)
		{
			restartingAddresses.ClearAll();
		}
	}

	if (conflictingAddresses.IsEmpty())
	{
		for (uint32_t& w : reportedConflicts)
		{
			w = 0;													// so that the same conflict coming back is reported at once
		}
		return;
	}

	bool changed = false;
	for (size_t i = 0; i < AddressBitmap::NumWords; i++)
	{
		const uint32_t w = conflictingAddresses.GetWord(i);
		if (w != reportedConflicts[i])
		{
			reportedConflicts[i] = w;
			changed = true;
		}
	}

	if (changed || millis() - whenLastConflictReport >= ConflictReportIntervalMillis)
	{
		whenLastConflictReport = millis();
		String<StringLength100> addresses;
		for (size_t i = 0; i < AddressBitmap::NumWords; i++)
		{
			for (uint32_t w = reportedConflicts[i]; w != 0; w &= w - 1)
			{
				addresses.catf("%s%u", addresses.IsEmpty() ? "" : ", ", (unsigned int)(i * 32 + __builtin_ctz(w)));
			}
		}
		reprap.GetPlatform().MessageF(ErrorMessage, "CAN address %s is used on both CAN%u and CAN%u, change the address of one of the boards using M952 I B A and restart it using M999 I B\n",
										addresses.c_str(), GetCanNumber(0), GetCanNumber(1));
	}
}

uint32_t CanInterface::SendPlainMessageNoFree(CanMessageBuffer *buf, uint32_t timeout) noexcept
{
	ReadLocker lock(port1Lock);
	return ((can1dev != nullptr) && port1Mode == Port1Mode::classic) ? can1dev->SendMessage(CanDevice::TxBufferNumber::fifo, timeout, buf) : 0;
}

bool CanInterface::ReceivePlainMessage(CanMessageBuffer *_ecv_null buf, uint32_t timeout) noexcept
{
	return (can1dev != nullptr) && port1Mode == Port1Mode::classic && can1dev->ReceiveMessage(CanDevice::RxBufferNumber::fifo1, timeout, buf);
}

// Learn the port that a board is on from the first message it sends. Return false if the message came from the other port and must be dropped.
// An announcement on the other port means that a second board uses the same address. Called with receiveMutex held
static bool AcceptReceivedMessage(size_t port, const CanMessageBuffer& buf) noexcept
{
	if (port == 1 && !IsPort1Up())
	{
		return false;												// received before M953 switched CAN0 to classic mode
	}

	const CanAddress src = buf.id.Src();
	if (src > CanId::MaxCanAddress || src == CanId::Exp3HCFirmwareUpdateAddress || src == CanInterface::GetCanAddress())
	{
		return true;												// a 3HC with its address switches at zero uses the firmware update address on either bus, so never learn it
	}

	if (!learnedAddresses[1 - port].IsSet(src))
	{
		if (!learnedAddresses[port].IsSet(src))
		{
			learnedAddresses[port].Set(src);
			reprap.BoardsUpdated();
		}
		return true;
	}

	const CanMessageType msgType = buf.id.MsgType();
	if ((msgType == CanMessageType::announceV0 || msgType == CanMessageType::announceV1)
		&& !conflictingAddresses.IsSet(src)
		&& !(restartingAddresses.IsSet(src) && millis() - whenLastRestarted < RestartGraceMillis))
	{
		conflictingAddresses.Set(src);
		reprap.BoardsUpdated();
	}
	wrongPortMessages++;
	return false;
}

#endif

// The CanReceiver task
extern "C" [[noreturn]] void CanReceiverLoop(void *param) noexcept
{
	const size_t port = (size_t)param;
#if DUAL_CAN
	CanDevice *const dev = (port == 0) ? can0dev : can1dev;
#else
	CanDevice *const dev = can0dev;
#endif
	CanMessageBuffer buf;
	for (;;)
	{
		if (dev->ReceiveMessage(RxBufferIndexRequest, TaskBase::TimeoutUnlimited, &buf))
		{
			buf.bus = port;													// responses are built in this buffer and go back on the same bus
			if (reprap.Debug(Module::CAN))
			{
				buf.DebugPrint((port == 0) ? "Rx0:" : "CAN0 Rx0:");
			}

#if DUAL_CAN
			MutexLocker lock(receiveMutex);
			if (!AcceptReceivedMessage(port, buf))
			{
				continue;
			}
#endif
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
GCodeResult CanInterface::SetRemotePressureAdvance(const CanDriversData<ShortPressureAdvanceParameters>& data, const StringRef& reply) noexcept
{
	return SetRemoteDriverValues(data, reply, CanMessageType::setPressureAdvanceV2);
}

// Handle M569 for a remote driver
GCodeResult CanInterface::ConfigureRemoteDriver(DriverId driver, GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
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
			if (gb.Seen('V'))
			{
				gb.MustSee('R');
			}
			CanMessageGenericConstructor cons(M569Point2Params);
			cons.PopulateFromCommand(gb);
			return cons.SendAndGetResponse(CanMessageType::m569p2, driver.boardAddress, reply);
		}

#if DUAL_CAN
	case 3:			// read driver encoder via secondary CAN
		{
			Kinematics &_ecv_from kin = reprap.GetMove().GetKinematics();
			if (kin.GetLegacyType() == KinematicsType::hangprinter)
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
			const ExpansionBoardData *_ecv_null const boardData = reprap.GetExpansion().GetBoardDetails(driver.boardAddress);
			if (boardData != nullptr && boardData->hasClosedLoop)
			{
				CanMessageGenericConstructor cons(M569Point4Params);
				cons.PopulateFromCommand(gb);
				return cons.SendAndGetResponse(CanMessageType::m569p4, driver.boardAddress, reply);
			}
		}
#if DUAL_CAN
		{
			Kinematics &_ecv_from kin = reprap.GetMove().GetKinematics();
			if (kin.GetLegacyType() == KinematicsType::hangprinter)
			{
				gb.MustSee('P');
				size_t drivesCount = reprap.GetGCodes().GetVisibleAxes();
				DriverId driverIds[drivesCount];
				gb.GetDriverIdArray(driverIds, drivesCount);

				gb.MustSee('T');
				float forces[drivesCount];
				size_t forceCount = drivesCount;
				gb.GetFloatArray(forces, forceCount, true);
				if (forceCount != drivesCount)
				{
					reply.copy("M569.4 requires one T value per P");
					return GCodeResult::error;
				}

				size_t driverIndex = drivesCount;
				for (size_t i = 0; i < drivesCount; ++i)
				{
					if (driverIds[i] == driver)
					{
						driverIndex = i;
						break;
					}
				}
				if (driverIndex == drivesCount)
				{
					reply.copy("M569.4 driver not found in P list");
					return GCodeResult::error;
				}
				return ((HangprinterKinematics&)kin).SetODrive3TorqueMode(driver, forces[driverIndex], reply);
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
			if (isDigit(portName[0]) && IoPort::RemoveBoardAddress(portName.GetRef()) != driver.boardAddress)
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
			Kinematics &_ecv_from kin = reprap.GetMove().GetKinematics();
			if (kin.GetLegacyType() == KinematicsType::hangprinter)
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

// Handle M970 for a remote driver
GCodeResult CanInterface::SetRemoteDriverStepMode(DriverId driver, unsigned int mode, const StringRef& reply) noexcept
{
	try
	{
		CanMessageGenericConstructor cons(M970Params);
		cons.AddDriverIdParam('P', driver);
		cons.AddUParam('S', mode);
		return cons.SendAndGetResponse(CanMessageType::m970, driver.boardAddress, reply);
	}
	catch (const GCodeException& e)
	{
		e.GetMessage(reply, nullptr);
		return GCodeResult::error;
	}
}

// Handle M970.1/M970.2 for a remote driver, sending the Kv or Ka value as parameter 'V' or 'A'
GCodeResult CanInterface::SetRemotePhaseStepParam(DriverId driver, char param, float value, const StringRef& reply) noexcept
{
	try
	{
		CanMessageGenericConstructor cons(M970Params);
		cons.AddDriverIdParam('P', driver);
		cons.AddFParam(param, value);
		return cons.SendAndGetResponse(CanMessageType::m970, driver.boardAddress, reply);
	}
	catch (const GCodeException& e)
	{
		e.GetMessage(reply, nullptr);
		return GCodeResult::error;
	}
}

// Handle M915 for a collection of remote drivers
GCodeResult CanInterface::GetSetRemoteDriverStallParameters(const CanDriversList& drivers, GCodeBuffer& gb, const StringRef& reply, OutputBuffer *_ecv_null & buf) THROWS(GCodeException)
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

// Static buffer to allow a string to be stored that is referred to by a thrown exception.
// This isn't ideal, however I consider it unlikely that both motion systems will execute homing moves involving remote drivers at the same time.
// An alternative would be to enlarge the GCodeException class to store a copy of the string instead of a pointer to it.
static String<StringLength100> enableEndstopsReply;

// Enable a stall endstop on a remote board
void CanInterface::EnableRemoteStallEndstop(DriverId did, float speed, bool useEncoder) THROWS(GCodeException)
{
	CanMessageBuffer *_ecv_null const buf = CanMessageBuffer::Allocate();
	if (buf == nullptr)
	{
		ThrowGCodeException("no CAN buffer available");
	}
	const CanRequestId rid = CanInterface::AllocateRequestId(did.boardAddress, buf);
	auto msg = buf->SetupRequestMessage<CanMessageEnableStallEndstop>(rid, CanInterface::GetCanAddress(), did.boardAddress);
	msg->driverNumber = did.localDriver;
	msg->speed = speed;
	msg->endstopType = useEncoder ? CanMessageEnableStallEndstop::typeEncoder : CanMessageEnableStallEndstop::typeMotorLoad;

	enableEndstopsReply.Clear();
	if (CanInterface::SendRequestAndGetStandardReply(buf, rid, enableEndstopsReply.GetRef(), nullptr) != GCodeResult::ok)
	{
		ThrowGCodeException(enableEndstopsReply.c_str());
	}
}

// Disable all stall endstops on a remote board
void CanInterface::DisableRemoteStallEndstops(CanAddress boardId) noexcept
{
	CanMessageBuffer *_ecv_null const buf = CanMessageBuffer::Allocate();
	if (buf == nullptr)
	{
		return;								// there's very little we can do here in terms of error handling
	}
	const CanRequestId rid = CanInterface::AllocateRequestId(boardId, buf);
	auto msg = buf->SetupRequestMessage<CanMessageEnableStallEndstop>(rid, CanInterface::GetCanAddress(), boardId);
	msg->driverNumber = CanMessageEnableStallEndstop::disableAll;
	msg->speed = 0.0;
	String<1> reply;
	(void)CanInterface::SendRequestAndGetStandardReply(buf, rid, reply.GetRef(), nullptr);
}

static GCodeResult GetRemoteInfo(uint8_t infoType, uint32_t boardAddress, uint8_t param, GCodeBuffer& gb, const StringRef& reply, uint8_t *_ecv_null extra = nullptr) THROWS(GCodeException)
			pre(IsExpansionBoardAddress(boardAddress))
{
	CanMessageBuffer *_ecv_null const buf = CanMessageBuffer::Allocate();
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
	if (type == 0)
	{
		Platform& p = reprap.GetPlatform();
		uint8_t currentPart = 0;
		uint8_t lastPart;
		GCodeResult res;
		do
		{
			// We can use 'reply' to buffer the returned data
			reply.Clear();
			res = GetRemoteInfo(CanMessageReturnInfo::typeDiagnosticsPart0 + currentPart, boardAddress, type, gb, reply, &lastPart);
			if (res != GCodeResult::ok)
			{
				return res;
			}
			if (currentPart == 0)
			{
				p.MessageF(mt, "Diagnostics for board %u:\n", (unsigned int)boardAddress);
			}
			if (!reply.IsEmpty())						// boards may return empty response parts
			{
				reply.cat('\n');
				p.Message(mt, reply.c_str());
			}
			++currentPart;
		} while (currentPart <= lastPart);
		reply.Clear();
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

GCodeResult CanInterface::GetRemoteFirmwareDetails(uint32_t boardAddress, GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	return GetRemoteInfo(CanMessageReturnInfo::typeFirmwareVersion, boardAddress, 0, gb, reply);
}

GCodeResult CanInterface::HandleM111(uint32_t boardAddress, GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	CanMessageGenericConstructor cons(M111Params);
	cons.PopulateFromCommand(gb);
	return cons.SendAndGetResponse(CanMessageType::m111, boardAddress, reply);
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

void CanInterface::WakeAsyncSenderFromIsr() noexcept
{
	canSenderTask.GiveFromISR(NotifyIndices::CanSender);
}

// Remote handle functions
GCodeResult CanInterface::CreateHandle(CanAddress boardAddress, RemoteInputHandle h, const char *_ecv_array pinName, uint16_t threshold, uint16_t minInterval,
										bool *_ecv_null currentState, const StringRef& reply) noexcept
{
	CanMessageBuffer *_ecv_null const buf = CanMessageBuffer::Allocate();
	if (buf == nullptr)
	{
		return GCodeResult::noCanBuffer;
	}

	const CanRequestId rid = AllocateRequestId(boardAddress, buf);
	auto msg = buf->SetupRequestMessage<CanMessageCreateInputMonitorV1>(rid, GetCanAddress(), boardAddress);
	msg->handle = h;
	msg->threshold = threshold;
	msg->minInterval = minInterval;
	SafeStrncpy(msg->pinName, pinName, ARRAY_SIZE(msg->pinName));
	buf->dataLength = msg->GetActualDataLength();

	uint8_t extra;
	const GCodeResult rslt = SendRequestAndGetStandardReply(buf, rid, reply, &extra);
	if (rslt == GCodeResult::ok && currentState != nullptr)
	{
		*currentState = (extra != 0);
	}
	return rslt;
}

static GCodeResult ChangeInputMonitor(CanAddress boardAddress, RemoteInputHandle h, uint8_t action, uint32_t param, uint8_t *_ecv_null retVal, const StringRef &reply, uint32_t *_ecv_null words = nullptr) noexcept
{
	if (!h.IsValid())
	{
		reply.copy("Invalid remote handle");
		return GCodeResult::error;
	}

	CanMessageBuffer *_ecv_null const buf = CanMessageBuffer::Allocate();
	if (buf == nullptr)
	{
		return GCodeResult::noCanBuffer;
	}

	const CanRequestId rid = CanInterface::AllocateRequestId(boardAddress, buf);
	auto msg = buf->SetupRequestMessage<CanMessageChangeInputMonitorV1>(rid, CanInterface::GetCanAddress(), boardAddress);
	msg->handle = h;
	msg->action = action;
	msg->param = param;
	uint8_t extra;
	const GCodeResult rslt = CanInterface::SendRequestAndGetStandardReply(buf, rid, reply, &extra, words);
	if (rslt == GCodeResult::ok && retVal != nullptr)
	{
		*retVal = extra;
	}
	return rslt;
}

GCodeResult CanInterface::DeleteHandle(CanAddress boardAddress, RemoteInputHandle h, const StringRef &reply) noexcept
{
	return ChangeInputMonitor(boardAddress, h, CanMessageChangeInputMonitorV1::actionDelete, 0, nullptr, reply);
}

GCodeResult CanInterface::GetHandlePinName(CanAddress boardAddress, RemoteInputHandle h, bool *_ecv_null currentState, const StringRef &reply) noexcept
{
	uint8_t rVal;
	const GCodeResult ret = ChangeInputMonitor(boardAddress, h, CanMessageChangeInputMonitorV1::actionReturnPinName, 0, &rVal, reply);
	if (ret < GCodeResult::error && currentState != nullptr)
	{
		*currentState = (rVal != 0);
	}
	return ret;
}

GCodeResult CanInterface::EnableHandle(CanAddress boardAddress, RemoteInputHandle h, bool enable, bool *_ecv_null currentState, const StringRef &reply) noexcept
{
	uint8_t rVal;
	const GCodeResult ret =  ChangeInputMonitor(boardAddress, h, (enable) ? CanMessageChangeInputMonitorV1::actionDoMonitor : CanMessageChangeInputMonitorV1::actionDontMonitor, 0, &rVal, reply);
	if (ret < GCodeResult::error && currentState != nullptr)
	{
		*currentState = (rVal != 0);
	}
	return ret;
}

GCodeResult CanInterface::ChangeHandleResponseTime(CanAddress boardAddress, RemoteInputHandle h, uint32_t responseMillis, bool *_ecv_null currentState, const StringRef &reply) noexcept
{
	uint8_t rVal;
	const GCodeResult ret =  ChangeInputMonitor(boardAddress, h, CanMessageChangeInputMonitorV1::actionChangeMinInterval, responseMillis, &rVal, reply);
	if (ret < GCodeResult::error && currentState != nullptr)
	{
		*currentState = (rVal != 0);
	}
	return ret;
}

GCodeResult CanInterface::ChangeHandleThreshold(CanAddress boardAddress, RemoteInputHandle h, int32_t threshold, bool *_ecv_null currentState, const StringRef &reply) noexcept
{
	uint8_t rVal;
	const GCodeResult ret =  ChangeInputMonitor(boardAddress, h, CanMessageChangeInputMonitorV1::actionChangeThreshold, (uint32_t)threshold, &rVal, reply);
	if (ret < GCodeResult::error && currentState != nullptr)
	{
		*currentState = (rVal != 0);
	}
	return ret;
}

GCodeResult CanInterface::ChangeHandleSetTouchMode(CanAddress boardAddress, RemoteInputHandle h, uint32_t sensitivity, const StringRef &reply) noexcept
{
	return ChangeInputMonitor(boardAddress, h, CanMessageChangeInputMonitorV1::actionSelectTouchMode, sensitivity, nullptr, reply);
}

GCodeResult CanInterface::SetHandleDriveLevel(CanAddress boardAddress, RemoteInputHandle h, uint32_t driveLevel, uint8_t &returnedDriveLevel, const StringRef &reply) noexcept
{
	return ChangeInputMonitor(boardAddress, h, CanMessageChangeInputMonitorV1::actionSetDriveLevel, driveLevel, &returnedDriveLevel, reply);
}

GCodeResult CanInterface::ReadRemoteHandles(CanAddress boardAddress, RemoteInputHandle mask, RemoteInputHandle pattern, ReadHandlesCallbackFunction callback, CallbackParameter param, const StringRef &reply) noexcept
{
	CanMessageBuffer *_ecv_null const buf = CanMessageBuffer::Allocate();
	if (buf == nullptr)
	{
		return GCodeResult::noCanBuffer;
	}

	const CanRequestId rid = CanInterface::AllocateRequestId(boardAddress, buf);
	auto msg = buf->SetupRequestMessage<CanMessageReadInputsRequest>(rid, GetCanAddress(), boardAddress);
	msg->mask = mask;
	msg->pattern = pattern;
	const GCodeResult rslt = SendRequestAndGetCustomReply(buf, rid, reply, nullptr, nullptr, CanMessageType::readInputsReplyV0,
															[callback, param](const CanMessageBuffer *bufp) noexcept -> void
																{
																	auto response = bufp->msg.readInputsReplyV0;
																	for (unsigned int i = 0; i < min<unsigned int>(response.numReported, ARRAY_SIZE(response.results)) && sizeof(uint32_t) + (i + 1) * sizeof(response.results[0]) <= bufp->dataLength; i++)
																	{
																		callback(param, response.results[i].handle, LoadLEU32(&response.results[i].reading));
																	}
																});
	return rslt;
}

// Tare an analog handle and/or change its baseline tracking, returning the baseline that the board latched or holds.
// Only the board knows the raw reading, so it has to come back in the reply
GCodeResult CanInterface::TareHandle(CanAddress boardAddress, RemoteInputHandle h, uint32_t mode, int32_t& baseline, const StringRef &reply) noexcept
{
	uint32_t words[CanMessageStandardReply::MaxNumWords];
	const GCodeResult rslt = ChangeInputMonitor(boardAddress, h, CanMessageChangeInputMonitorV1::actionTare, mode, nullptr, reply, words);
	if (rslt == GCodeResult::ok)
	{
		baseline = (int32_t)words[0];
	}
	return rslt;
}

// Process M655 (send request to custom expansion board)
GCodeResult CanInterface::ProcessM655(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	CanAddress addr;
	if (gb.Seen('B'))
	{
		addr = gb.GetCanAddress('B', false, false);
	}
	else if (gb.Seen('C'))
	{
		String<StringLength50> cParam;
		gb.GetReducedString(cParam.GetRef());
		addr = IoPort::RemoveBoardAddress(cParam.GetRef());
	}
	else
	{
		reply.copy("B or C parameter must be provided");
		return GCodeResult::error;
	}

	CanMessageGenericConstructor cons(M655Params);
	cons.PopulateFromCommand(gb);
	return cons.SendAndGetResponse(CanMessageType::m655, addr, reply, nullptr);
}

static void AppendTxTimeouts(const StringRef& reply, size_t port) noexcept
{
	reply.lcat("Tx timeouts");
	char c = ' ';
	for (unsigned int& txt : txTimeouts[port])
	{
		reply.catf("%c%u", c, txt);
		txt = 0;
		c = ',';
	}

	if (lastCancelledIds[port] != 0)
	{
		CanId id;
		id.SetReceivedId(lastCancelledIds[port]);
		lastCancelledIds[port] = 0;
		reply.catf(" last cancelled message type %u dest %u", (unsigned int)id.MsgType(), id.Dst());
	}
}

void CanInterface::Diagnostics(const StringRef& reply) noexcept
{
	reply.copy("=== CAN ===");
	// If the user runs M122 after an emergency stop, can0dev will be null
	if (can0dev == nullptr)
	{
		reply.lcat("Disabled");
	}
	else
	{
		CanDevice::CanStats stats;
		can0dev->GetAndClearStats(stats);
		reply.lcatf("Messages queued %u, received %u, lost %u, "
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

	reply.lcatf("Longest wait %" PRIu32 "ms for reply type %u, peak Tx sync delay %" PRIu32
				", free buffers %u (min %u)"
	//debug
				", ts %u/%u/%u"
	//end debug
				,
					longestWaitTime, longestWaitMessageType, peakTimeSyncTxDelay,
					CanMessageBuffer::GetFreeBuffers(), CanMessageBuffer::GetAndClearMinFreeBuffers()
	//debug
					, timeSyncMessagesSent, goodTimeStamps, badTimeStamps
	//end debug
				);

	AppendTxTimeouts(reply, 0);

#if DUAL_CAN
	if (can1dev != nullptr)
	{
		CanDevice::CanStats stats;
		can1dev->GetAndClearStats(stats);
		switch (port1Mode)
		{
		case Port1Mode::off:
			reply.lcatf("CAN%u: not enabled", GetCanNumber(1));
			break;

		case Port1Mode::classic:
			reply.lcatf("CAN%u: classic CAN", GetCanNumber(1));
			break;

		case Port1Mode::expansion:
			reply.lcatf("CAN%u: boards %u, messages queued %u, received %u, lost %u, wrong bus %u, errs %u, boc %u",
						GetCanNumber(1), learnedAddresses[1].Count(), stats.messagesQueuedForSending, stats.messagesReceived, stats.messagesLost, wrongPortMessages, stats.protocolErrors, stats.busOffCount);
			AppendTxTimeouts(reply, 1);
			if (!conflictingAddresses.IsEmpty())
			{
				reply.lcat("Address conflicts:");
				for (size_t i = 0; i < AddressBitmap::NumWords; i++)
				{
					for (uint32_t w = conflictingAddresses.GetWord(i); w != 0; w &= w - 1)
					{
						reply.catf(" %u", (unsigned int)(i * 32 + __builtin_ctz(w)));
					}
				}
			}
			break;
		}
	}
	wrongPortMessages = 0;
#endif

	longestWaitTime = 0;
	longestWaitMessageType = 0;
	peakTimeSyncTxDelay = 0;
	timeSyncMessagesSent = goodTimeStamps = badTimeStamps = 0;

#if SUPPORT_REMOTE_COMMANDS
	if (InExpansionMode())
	{
		CommandProcessor::AppendBadMotionStats(reply);
		reply.lcatf("Connection timeout %" PRIu32 "s, sync losses %u, timeout shutdowns %u", connectionTimeoutMillis / 1000, syncLossCount, timeoutShutdownCount);
	}
#endif
}

GCodeResult CanInterface::WriteGpio(CanAddress boardAddress, uint8_t portNumber, float pwm, bool isServo, const GCodeBuffer* gb, const StringRef &reply) noexcept
{
	CanMessageBuffer *_ecv_null const buf = CanMessageBuffer::Allocate();
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
	const uint32_t oldAddress = gb.GetCanAddress('B', true, false);

	// Get the new timing details, if provided
	CanTiming timing;
	bool changeTiming = false;
	uint32_t speed;
	if (gb.TryGetLimitedUIValue('S', speed, changeTiming, MinBitRate, MaxBitRate + 1))
	{
		speed *= 1000;
		timing.SetDefaults(speed);

		float f;
		if (gb.TryGetLimitedFValue('T', f, changeTiming, MinSamplePoint, MaxSamplePoint))
		{
			timing.SetNormalSamplePoint(f);
		}

		if (gb.TryGetLimitedFValue('J', f, changeTiming, MinJumpWidth, MaxJumpWidth))
		{
			timing.SetNormalJumpWidth(f);
		}
	}

	if (oldAddress == GetCanAddress())
	{
		if (gb.Seen('I') && GetPortFromCanNumber(gb, gb.GetUIValue()) != 0)
		{
			reply.printf("Use M953 I%u to set the timing of CAN%u", GetCanNumber(1), GetCanNumber(1));
			return GCodeResult::error;
		}
		if (changeTiming)
		{
			can0dev->ChangeLocalCanTiming(timing);
		}
		else
		{
			ReportCanTiming(0, reply);
		}
		return GCodeResult::ok;
	}

	const size_t port = GetPortForBoardCommand(gb, (CanAddress)oldAddress);
	CanMessageBufferHandle buf(AllocateBuffer(&gb));
	const CanRequestId rid = CanInterface::AllocateRequestId((uint8_t)oldAddress, buf.Access());
	auto msg = buf.Access()->SetupRequestMessage<CanMessageSetAddressAndNormalTiming>(rid, GetCanAddress(), (uint8_t)oldAddress);
	buf.Access()->useBrs = false;							// this message is used to report on the fast data rate, so disable bit rate switching
	msg->oldAddress = (uint8_t)oldAddress;

	if (gb.Seen('A'))
	{
		const uint32_t newAddress = gb.GetCanAddress('A', false, false);
		msg->newAddress = (uint8_t)newAddress;
		msg->newAddressInverted = (uint8_t)~newAddress;
	}
	else
	{
		msg->newAddress = msg->newAddressInverted = 0;
	}

	msg->doSetTiming = (changeTiming) ? CanMessageSetAddressAndNormalTiming::DoSetTimingYes : CanMessageSetAddressAndNormalTiming::DoSetTimingNo;
	msg->normalTiming = timing;

	return CanInterface::SendRequestOnPortAndGetStandardReply(port, buf.HandOver(), rid, reply);
}

// Enable CAN in master board mode and set the fast data rate
GCodeResult CanInterface::EnableCan(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	bool seen = false;
	GCodeResult ret = GCodeResult::ok;

#if DUAL_CAN
	if (!gb.SeenAny("ICSTJRUK"))
	{
		ReportCanTiming(0, reply);
		ReportCanTiming(1, reply);
		return GCodeResult::ok;
	}
#endif

	// I is the CAN number on the board, see the note at NumCanPorts
	const size_t port = gb.Seen('I') ? GetPortFromCanNumber(gb, gb.GetUIValue()) : 0;

	bool classic = false;
#if DUAL_CAN
	bool seenPortSetting = false;
	if (port == 1)
	{
		if (can1dev == nullptr)
		{
			reply.printf("CAN%u is disabled", GetCanNumber(1));
			return GCodeResult::error;
		}
		classic = (port1Mode == Port1Mode::classic);
		gb.TryGetBValue('C', classic, seenPortSetting);
	}
	else if (gb.Seen('C'))
	{
		reply.printf("Parameter C is only supported for CAN%u", GetCanNumber(1));
		return GCodeResult::error;
	}
#endif

	// Get the arbitration bit rate
	uint32_t speed = classic ? DefaultClassicBitRate : DefaultBitRate;
	if (gb.TryGetLimitedUIValue('S', speed, seen, MinBitRate, MaxBitRate + 1))
	{
		if (!classic && (speed != DefaultBitRate) && (speed != DefaultBitRate / 2) && (speed != DefaultBitRate / 4))
		{
			reply.copy("Expansion boards only provide automatic support for bit rates 1000, 500 and 250 kbits/sec");
			ret = GCodeResult::warning;
		}
	}

	CanTiming timing;
	timing.SetDefaults(speed * 1000);

	float f;
	if (gb.TryGetLimitedFValue('T', f, seen, MinSamplePoint, MaxSamplePoint))
	{
		timing.SetNormalSamplePoint(f);
	}

	if (gb.TryGetLimitedFValue('J', f, seen, MinJumpWidth, MaxJumpWidth))
	{
		timing.SetNormalJumpWidth(f);
	}

#if SUPPORT_BRS
	uint32_t bitRateMultiplier = fastDataRates[port] + 1;
	if (gb.TryGetLimitedUIValue('R', bitRateMultiplier, seen, 9))
	{
		if (classic)
		{
			reply.copy("Bit rate switching is not supported in classic mode");
			return GCodeResult::error;
		}
		if (bitRateMultiplier == 0 || bitRateMultiplier == 5 || bitRateMultiplier == 7)
		{
			reply.lcatf("Bit rate multiplier must be 1, 2, 3, 4, 6 or 8");
			return GCodeResult::error;
		}
		timing.EnableBrs(bitRateMultiplier);
		if (gb.TryGetLimitedFValue('U', f, seen, MinSamplePoint, MaxSamplePoint))
		{
			timing.SetDataSamplePoint(f);
		}
		if (gb.TryGetLimitedFValue('K', f, seen, MinJumpWidth, MaxJumpWidth))
		{
			timing.SetDataJumpWidth(f);
		}
	}
#endif

#if DUAL_CAN
	if (port == 1)
	{
		if (seen || seenPortSetting)
		{
			MutexLocker transactionLock(transactionMutexes[1]);
			MutexLocker receiveLock(receiveMutex);
			WriteLocker port1WriteLock(port1Lock);
			const Port1Mode newMode = classic ? Port1Mode::classic : Port1Mode::expansion;
			if (seen || newMode != port1Mode)
			{
				if (newMode != port1Mode)
				{
					// The boards heard on CAN0 are no longer reachable, and every conflict involves one of them
					learnedAddresses[1].ClearAll();
					conflictingAddresses.ClearAll();
				}
				InitPort1(classic, timing);
# if SUPPORT_BRS
				if (!classic)
				{
					fastDataRates[1] = bitRateMultiplier - 1;
					dTseg1MinusOnes[1] = timing.dTseg1;
				}
# endif
				reprap.BoardsUpdated();
			}
		}
		else
		{
			ReportCanTiming(port, reply);
		}
		return ret;
	}
#endif

	if (seen)
	{
		{
			AtomicCriticalSectionLocker lock;
			fastDataRates[0] = 0;														// disable BRS
			dTseg1MinusOnes[0] = 0;
		}
		delay(50);																		// allow any existing transactions to complete
		can0dev->ChangeLocalCanTiming(timing);
#if SUPPORT_BRS
		{
			AtomicCriticalSectionLocker lock;
			fastDataRates[0] = bitRateMultiplier - 1;
			dTseg1MinusOnes[0] = timing.dTseg1;
		}
#endif
	}
	else
	{
		ReportCanTiming(port, reply);
	}

	// TODO here we should enable CAN
	return ret;
}

void CanInterface::ReportCanTiming(size_t port, const StringRef& reply) noexcept
{
	CanTiming timing;
#if DUAL_CAN
	if (port == 1)
	{
		if (can1dev == nullptr || port1Mode == Port1Mode::off)
		{
			reply.lcatf("CAN%u is not enabled", GetCanNumber(1));
			return;
		}
		can1dev->GetLocalCanTiming(timing);
		reply.lcatf((port1Mode == Port1Mode::classic) ? "CAN%u in classic mode, " : "CAN%u ", GetCanNumber(1));
	}
	else
#endif
	{
		can0dev->GetLocalCanTiming(timing);
#if DUAL_CAN
		reply.lcatf("CAN%u ", GetCanNumber(0));
#else
		reply.lcat("CAN ");
#endif
	}
	reply.catf("arbitration speed %.1fkbps, sample point %.2f, jump width %.2f, ",
					(double)((float)CanTiming::ClockFrequency/(float)(1000 * timing.period)),
					(double)((float)(timing.nTseg1 + 1)/(float)timing.period),
					(double)((float)timing.nJumpWidth/(float)timing.period));
	const uint8_t fastDataRate = fastDataRates[port];
	if (fastDataRate == 0)
	{
		reply.cat("bit rate switching disabled");
	}
	else
	{
		const uint32_t dataPeriod = timing.period/(fastDataRate + 1);
		reply.catf("data speed %.1fkbps, sample point %.2f, jump width %.2f",
					(double)((float)CanTiming::ClockFrequency/(float)(1000 * dataPeriod)),
					(double)((float)(timing.dTseg1 + 1)/(float)dataPeriod),
					(double)((float)timing.dJumpWidth/(float)dataPeriod));
	}
}

#if SUPPORT_REMOTE_COMMANDS

// Check that the BRS setting we are using matches the one in the time sync message and change it if necessary
void CanInterface::CheckBrs(const CanMessageTimeSync& msg) noexcept
{
	if ((msg.fastDataRate != fastDataRates[0]) || (msg.tseg1Minus1 != dTseg1MinusOnes[0]))
	{
		CanTiming timing;
		can0dev->GetLocalCanTiming(timing);
		timing.EnableBrs(msg.fastDataRate + 1);
		timing.SetDataSamplePointDirect(msg.tseg1Minus1);
		can0dev->ChangeLocalCanTiming(timing);
		fastDataRates[0] = msg.fastDataRate;
		dTseg1MinusOnes[0] = msg.tseg1Minus1;
	}
}

#endif

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

GCodeResult CanInterface::StartAccelerometer(CanAddress boardAddress, size_t deviceNumber, uint8_t axes, uint32_t numSamples, uint8_t mode, const GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	CanMessageBuffer* const buf = AllocateBuffer(&gb);
	const CanRequestId rid = CanInterface::AllocateRequestId(boardAddress, buf);
	auto msg = buf->SetupRequestMessage<CanMessageStartAccelerometer>(rid, GetCanAddress(), boardAddress);
	msg->deviceNumber = deviceNumber;
	msg->axes = axes;
	msg->numSamples = numSamples;
	msg->delayedStart = 0;
	msg->startTime = false;
	return SendRequestAndGetStandardReply(buf, rid, reply);
}

# endif

GCodeResult CanInterface::StartClosedLoopDataCollection(DriverId device, uint16_t filter, uint16_t numSamples, uint16_t p_rateRequested, uint8_t p_movementRequested, uint8_t mode, const GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	CanMessageBuffer* const buf = AllocateBuffer(&gb);
	const CanRequestId rid = CanInterface::AllocateRequestId(device.boardAddress, buf);
	auto msg = buf->SetupRequestMessage<CanMessageStartClosedLoopDataCollection>(rid, GetCanAddress(), device.boardAddress);
	msg->mode = mode;
	msg->filter = filter;
	msg->rate = p_rateRequested;
	msg->numSamples = numSamples;
	msg->movement = p_movementRequested;
	msg->deviceNumber = device.localDriver;
	return SendRequestAndGetStandardReply(buf, rid, reply);
}

#if DUAL_CAN

CanId CanInterface::ODrive::ArbitrationId(DriverId driver, uint8_t cmd) noexcept {
	const auto arbitration_id = (driver.boardAddress << 5) + cmd;
	CanId canId;
	canId.SetReceivedId(arbitration_id);
	return canId;
}

CanMessageBuffer *_ecv_null CanInterface::ODrive::PrepareSimpleMessage(DriverId const driver, const StringRef& reply) noexcept
{
	// Detect any early return conditions. The reply may already hold results for other drivers, so append to it
	if (port1Mode != Port1Mode::classic)
	{
		reply.lcatf("CAN%u must be in classic mode to talk to ODrives, see M953 I%u C1", GetCanNumber(1), GetCanNumber(1));
		return nullptr;
	}
	CanMessageBuffer *_ecv_null buf = (can1dev != nullptr) ? CanMessageBuffer::Allocate() : nullptr;
	if (buf == nullptr)
	{
		reply.lcat(NoCanBufferMessage);
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
