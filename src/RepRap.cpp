#include "RepRap.h"

#include "Movement/Move.h"
#include "Movement/StepTimer.h"
#include "FilamentMonitors/FilamentMonitor.h"
#include "GCodes/GCodes.h"
#include "Heating/Heat.h"
#include "Heating/Sensors/TemperatureSensor.h"
#include "Network.h"
#if HAS_NETWORKING
# include "Networking/HttpResponder.h"
#endif
#include "Platform.h"
#include "Scanner.h"
#include "PrintMonitor.h"
#include "Tools/Tool.h"
#include "Tools/Filament.h"
#include "Endstops/ZProbe.h"
#include "Tasks.h"
#include "Hardware/Cache.h"
#include "Fans/FansManager.h"
#include "Version.h"

#ifdef DUET_NG
# include "DueXn.h"
#endif

#if SUPPORT_IOBITS
# include "PortControl.h"
#endif

#if SUPPORT_12864_LCD
# include "Display/Display.h"
#endif

#if HAS_LINUX_INTERFACE
# include "Linux/LinuxInterface.h"
#endif

#if HAS_HIGH_SPEED_SD
# include "sam/drivers/hsmci/hsmci.h"
# include "conf_sd_mmc.h"
# if SAME70
static_assert(CONF_HSMCI_XDMAC_CHANNEL == DmacChanHsmci, "mismatched DMA channel assignment");
# endif
#endif

#if SUPPORT_CAN_EXPANSION
# include <CAN/CanInterface.h>
# include <CAN/ExpansionManager.h>
#endif

#include "FreeRTOS.h"
#include "task.h"

#if SAME70
# include "Hardware/DmacManager.h"
#endif

// We call vTaskNotifyGiveFromISR from various interrupts, so the following must be true
static_assert(configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY <= NvicPriorityHSMCI, "configMAX_SYSCALL_INTERRUPT_PRIORITY is set too high");

#ifndef __LPC17xx__

static TaskHandle hsmciTask = nullptr;									// the task that is waiting for a HSMCI command to complete

// HSMCI interrupt handler
extern "C" void HSMCI_Handler() noexcept
{
	HSMCI->HSMCI_IDR = 0xFFFFFFFF;										// disable all HSMCI interrupts
#if SAME70
	XDMAC->XDMAC_CHID[DmacChanHsmci].XDMAC_CID = 0xFFFFFFFF;			// disable all DMA interrupts for this channel
#endif
	TaskBase::GiveFromISR(hsmciTask);									// wake up the task
}

#if SAME70

// HSMCI DMA complete callback
void HsmciDmaCallback(CallbackParameter cp) noexcept
{
	HSMCI->HSMCI_IDR = 0xFFFFFFFF;										// disable all HSMCI interrupts
	XDMAC->XDMAC_CHID[DmacChanHsmci].XDMAC_CID = 0xFFFFFFFF;			// disable all DMA interrupts for this channel
	if (hsmciTask != nullptr)
	{
		BaseType_t higherPriorityTaskWoken = pdFALSE;
		vTaskNotifyGiveFromISR(hsmciTask, &higherPriorityTaskWoken);	// wake up the task
		hsmciTask = nullptr;
		portYIELD_FROM_ISR(higherPriorityTaskWoken);
	}
}

#endif

// Callback function from the hsmci driver, called while it is waiting for an SD card operation to complete
// 'stBits' is the set of bits in the HSMCI status register that the caller is interested in.
// The caller keeps calling this function until at least one of those bits is set.
extern "C" void hsmciIdle(uint32_t stBits, uint32_t dmaBits) noexcept
{
	if (   (HSMCI->HSMCI_SR & stBits) == 0
#if SAME70
		&& (XDMAC->XDMAC_CHID[DmacChanHsmci].XDMAC_CIS & dmaBits) == 0
#endif
	   )
	{
		// Suspend this task until we get an interrupt indicating that a status bit that we are interested in has been set
		hsmciTask = TaskBase::GetCallerTaskHandle();
		HSMCI->HSMCI_IER = stBits;
#if SAME70
		DmacManager::SetInterruptCallback(DmacChanHsmci, HsmciDmaCallback, CallbackParameter());
		XDMAC->XDMAC_CHID[DmacChanHsmci].XDMAC_CIE = dmaBits;
		XDMAC->XDMAC_GIE = 1u << DmacChanHsmci;
#endif
		if (TaskBase::Take(200))
		{
			// We timed out waiting for the HSMCI operation to complete
			reprap.GetPlatform().LogError(ErrorCode::HsmciTimeout);
		}
	}
}

#endif //end ifndef LPC

#if SUPPORT_OBJECT_MODEL

// Object model table and functions
// Note: if using GCC version 7.3.1 20180622 and lambda functions are used in this table, you must compile this file with option -std=gnu++17.
// Otherwise the table will be allocate in RAM instead of flash, which wastes too much RAM.

// Macro to build a standard lambda function that includes the necessary type conversions
#define OBJECT_MODEL_FUNC(...) OBJECT_MODEL_FUNC_BODY(RepRap, __VA_ARGS__)
#define OBJECT_MODEL_FUNC_IF(_condition,...) OBJECT_MODEL_FUNC_IF_BODY(RepRap, _condition,__VA_ARGS__)

constexpr ObjectModelArrayDescriptor RepRap::boardsArrayDescriptor =
{
	nullptr,					// no lock needed
#if SUPPORT_CAN_EXPANSION
	[] (const ObjectModel *self, const ObjectExplorationContext&) noexcept -> size_t { return ((const RepRap*)self)->expansion->GetNumExpansionBoards() + 1; },
	[] (const ObjectModel *self, ObjectExplorationContext& context) noexcept -> ExpressionValue
			{	return (context.GetLastIndex() == 0)
						? ExpressionValue(((const RepRap*)self)->platform, 0)
							: ExpressionValue(((const RepRap*)self)->expansion, 0); }
#else
	[] (const ObjectModel *self, const ObjectExplorationContext&) noexcept -> size_t { return 1; },
	[] (const ObjectModel *self, ObjectExplorationContext& context) noexcept -> ExpressionValue { return ExpressionValue(((const RepRap*)self)->platform, 0); }
#endif
};

constexpr ObjectModelArrayDescriptor RepRap::fansArrayDescriptor =
{
	&FansManager::fansLock,
	[] (const ObjectModel *self, const ObjectExplorationContext&) noexcept -> size_t { return ((const RepRap*)self)->fansManager->GetNumFansToReport(); },
	[] (const ObjectModel *self, ObjectExplorationContext& context) noexcept -> ExpressionValue { return ExpressionValue(((const RepRap*)self)->fansManager->FindFan(context.GetLastIndex()).Ptr()); }
};

constexpr ObjectModelArrayDescriptor RepRap::inputsArrayDescriptor =
{
	nullptr,
	[] (const ObjectModel *self, const ObjectExplorationContext&) noexcept -> size_t { return ((const RepRap*)self)->gCodes->GetNumInputs(); },
	[] (const ObjectModel *self, ObjectExplorationContext& context) noexcept -> ExpressionValue { return ExpressionValue(((const RepRap*)self)->gCodes->GetInput(context.GetLastIndex())); }
};

constexpr ObjectModelArrayDescriptor RepRap::gpoutArrayDescriptor =
{
	nullptr,
	[] (const ObjectModel *self, const ObjectExplorationContext&) noexcept -> size_t { return reprap.GetPlatform().GetNumGpOutputsToReport(); },
	[] (const ObjectModel *self, ObjectExplorationContext& context) noexcept -> ExpressionValue
					{
						const GpOutputPort& port = reprap.GetPlatform().GetGpOutPort(context.GetLastIndex());
						return (port.IsUnused()) ? ExpressionValue(nullptr) : ExpressionValue(&port);
					}
};

constexpr ObjectModelArrayDescriptor RepRap::spindlesArrayDescriptor =
{
	nullptr,					// no lock needed
	[] (const ObjectModel *self, const ObjectExplorationContext& context) noexcept -> size_t { return MaxSpindles; },
	[] (const ObjectModel *self, ObjectExplorationContext& context) noexcept -> ExpressionValue { return ExpressionValue(&((const RepRap*)self)->platform->AccessSpindle(context.GetLastIndex())); }
};

constexpr ObjectModelArrayDescriptor RepRap::toolsArrayDescriptor =
{
	&toolListLock,
	[] (const ObjectModel *self, const ObjectExplorationContext&) noexcept -> size_t { return ((const RepRap*)self)->numToolsToReport; },
	[] (const ObjectModel *self, ObjectExplorationContext& context) noexcept -> ExpressionValue { return ExpressionValue(((const RepRap*)self)->GetTool(context.GetLastIndex()).Ptr()); }
};

constexpr ObjectModelArrayDescriptor RepRap::restorePointsArrayDescriptor =
{
	nullptr,
	[] (const ObjectModel *self, const ObjectExplorationContext&) noexcept -> size_t { return NumRestorePoints; },
	[] (const ObjectModel *self, ObjectExplorationContext& context) noexcept -> ExpressionValue
																		{ return ExpressionValue(((const RepRap*)self)->gCodes->GetRestorePoint(context.GetLastIndex())); }
};

constexpr ObjectModelArrayDescriptor RepRap::volumesArrayDescriptor =
{
	nullptr,
#if HAS_MASS_STORAGE
	[] (const ObjectModel *self, const ObjectExplorationContext&) noexcept -> size_t { return MassStorage::GetNumVolumes(); },
	[] (const ObjectModel *self, ObjectExplorationContext& context) noexcept -> ExpressionValue { return ExpressionValue(MassStorage::GetVolume(context.GetLastIndex())); }
#else
	[] (const ObjectModel *self, const ObjectExplorationContext&) noexcept -> size_t { return 0; },
	[] (const ObjectModel *self, ObjectExplorationContext& context) noexcept -> ExpressionValue { return ExpressionValue(nullptr); }
#endif
};

constexpr ObjectModelTableEntry RepRap::objectModelTable[] =
{
	// Within each group, these entries must be in alphabetical order
	// 0. MachineModel root
	{ "boards",					OBJECT_MODEL_FUNC_NOSELF(&boardsArrayDescriptor),						ObjectModelEntryFlags::live },
#if HAS_MASS_STORAGE
	{ "directories",			OBJECT_MODEL_FUNC(self, 1),												ObjectModelEntryFlags::none },
#endif
	{ "fans",					OBJECT_MODEL_FUNC_NOSELF(&fansArrayDescriptor),							ObjectModelEntryFlags::live },
	{ "heat",					OBJECT_MODEL_FUNC(self->heat),											ObjectModelEntryFlags::live },
	{ "inputs",					OBJECT_MODEL_FUNC_NOSELF(&inputsArrayDescriptor),						ObjectModelEntryFlags::live },
	{ "job",					OBJECT_MODEL_FUNC(self->printMonitor),									ObjectModelEntryFlags::live },
	{ "limits",					OBJECT_MODEL_FUNC(self, 2),												ObjectModelEntryFlags::none },
	{ "move",					OBJECT_MODEL_FUNC(self->move),											ObjectModelEntryFlags::live },
	{ "network",				OBJECT_MODEL_FUNC(self->network),										ObjectModelEntryFlags::none },
#if SUPPORT_SCANNER
	{ "scanner",				OBJECT_MODEL_FUNC(self->scanner),										ObjectModelEntryFlags::none },
#endif
	{ "sensors",				OBJECT_MODEL_FUNC(&self->platform->GetEndstops()),						ObjectModelEntryFlags::live },
	{ "seqs",					OBJECT_MODEL_FUNC(self, 6),												ObjectModelEntryFlags::live },
	{ "spindles",				OBJECT_MODEL_FUNC_NOSELF(&spindlesArrayDescriptor),						ObjectModelEntryFlags::live },
	{ "state",					OBJECT_MODEL_FUNC(self, 3),												ObjectModelEntryFlags::live },
	{ "tools",					OBJECT_MODEL_FUNC_NOSELF(&toolsArrayDescriptor),						ObjectModelEntryFlags::live },
	{ "volumes",				OBJECT_MODEL_FUNC_NOSELF(&volumesArrayDescriptor),						ObjectModelEntryFlags::none },

	// 1. MachineModel.directories
#if HAS_MASS_STORAGE
	{ "filaments",				OBJECT_MODEL_FUNC_NOSELF(FILAMENTS_DIRECTORY),							ObjectModelEntryFlags::verbose },
	{ "firmware",				OBJECT_MODEL_FUNC_NOSELF(FIRMWARE_DIRECTORY),							ObjectModelEntryFlags::verbose },
	{ "gCodes",					OBJECT_MODEL_FUNC(self->platform->GetGCodeDir()),						ObjectModelEntryFlags::verbose },
	{ "macros",					OBJECT_MODEL_FUNC(self->platform->GetMacroDir()),						ObjectModelEntryFlags::verbose },
	{ "menu",					OBJECT_MODEL_FUNC_NOSELF(MENU_DIR),										ObjectModelEntryFlags::verbose },
	{ "scans",					OBJECT_MODEL_FUNC_NOSELF(SCANS_DIRECTORY),								ObjectModelEntryFlags::verbose },
	{ "system",					OBJECT_MODEL_FUNC_NOSELF(ExpressionValue::SpecialType::sysDir, 0),		ObjectModelEntryFlags::none },
	{ "web",					OBJECT_MODEL_FUNC(self->platform->GetWebDir()),							ObjectModelEntryFlags::verbose },
#endif

	// 2. MachineModel.limits
	{ "axes",					OBJECT_MODEL_FUNC_NOSELF((int32_t)MaxAxes),								ObjectModelEntryFlags::verbose },
	{ "axesPlusExtruders",		OBJECT_MODEL_FUNC_NOSELF((int32_t)MaxAxesPlusExtruders),				ObjectModelEntryFlags::verbose },
	{ "bedHeaters",				OBJECT_MODEL_FUNC_NOSELF((int32_t)MaxBedHeaters),						ObjectModelEntryFlags::verbose },
#if SUPPORT_CAN_EXPANSION
	{ "boards",					OBJECT_MODEL_FUNC_NOSELF((int32_t)MaxCanBoards + 1),					ObjectModelEntryFlags::verbose },
#else
	{ "boards",					OBJECT_MODEL_FUNC_NOSELF((int32_t)1),									ObjectModelEntryFlags::verbose },
#endif
	{ "chamberHeaters",			OBJECT_MODEL_FUNC_NOSELF((int32_t)MaxChamberHeaters),					ObjectModelEntryFlags::verbose },
#if SUPPORT_CAN_EXPANSION
	{ "drivers",				OBJECT_MODEL_FUNC_NOSELF((int32_t)(NumDirectDrivers + MaxCanDrivers)),	ObjectModelEntryFlags::verbose },
#else
	{ "drivers",				OBJECT_MODEL_FUNC_NOSELF((int32_t)NumDirectDrivers),					ObjectModelEntryFlags::verbose },
#endif
	{ "driversPerAxis",			OBJECT_MODEL_FUNC_NOSELF((int32_t)MaxDriversPerAxis),					ObjectModelEntryFlags::verbose },
	{ "extruders",				OBJECT_MODEL_FUNC_NOSELF((int32_t)MaxExtruders),						ObjectModelEntryFlags::verbose },
	{ "extrudersPerTool",		OBJECT_MODEL_FUNC_NOSELF((int32_t)MaxExtrudersPerTool),					ObjectModelEntryFlags::verbose },
	{ "fans",					OBJECT_MODEL_FUNC_NOSELF((int32_t)MaxFans),								ObjectModelEntryFlags::verbose },
	{ "gpInPorts",				OBJECT_MODEL_FUNC_NOSELF((int32_t)MaxGpInPorts),						ObjectModelEntryFlags::verbose },
	{ "gpOutPorts",				OBJECT_MODEL_FUNC_NOSELF((int32_t)MaxGpOutPorts),						ObjectModelEntryFlags::verbose },
	{ "heaters",				OBJECT_MODEL_FUNC_NOSELF((int32_t)MaxHeaters),							ObjectModelEntryFlags::verbose },
	{ "heatersPerTool",			OBJECT_MODEL_FUNC_NOSELF((int32_t)MaxHeatersPerTool),					ObjectModelEntryFlags::verbose },
	{ "monitorsPerHeater",		OBJECT_MODEL_FUNC_NOSELF((int32_t)MaxMonitorsPerHeater),				ObjectModelEntryFlags::verbose },
	{ "restorePoints",			OBJECT_MODEL_FUNC_NOSELF((int32_t)NumRestorePoints),					ObjectModelEntryFlags::verbose },
	{ "sensors",				OBJECT_MODEL_FUNC_NOSELF((int32_t)MaxSensors),							ObjectModelEntryFlags::verbose },
	{ "spindles",				OBJECT_MODEL_FUNC_NOSELF((int32_t)MaxSpindles),							ObjectModelEntryFlags::verbose },
	{ "tools",					OBJECT_MODEL_FUNC_NOSELF((int32_t)MaxTools),							ObjectModelEntryFlags::verbose },
	{ "trackedObjects",			OBJECT_MODEL_FUNC_NOSELF((int32_t)MaxTrackedObjects),					ObjectModelEntryFlags::verbose },
	{ "triggers",				OBJECT_MODEL_FUNC_NOSELF((int32_t)MaxTriggers),							ObjectModelEntryFlags::verbose },
	// TODO userVariables
#if HAS_MASS_STORAGE
	{ "volumes",				OBJECT_MODEL_FUNC_NOSELF((int32_t)NumSdCards),							ObjectModelEntryFlags::verbose },
#else
	{ "volumes",				OBJECT_MODEL_FUNC_NOSELF((int32_t)0),									ObjectModelEntryFlags::verbose },
#endif
	{ "workplaces",				OBJECT_MODEL_FUNC_NOSELF((int32_t)NumCoordinateSystems),				ObjectModelEntryFlags::verbose },
	{ "zProbeProgramBytes",		OBJECT_MODEL_FUNC_NOSELF((int32_t)MaxZProbeProgramBytes),				ObjectModelEntryFlags::verbose },
	{ "zProbes",				OBJECT_MODEL_FUNC_NOSELF((int32_t)MaxZProbes),							ObjectModelEntryFlags::verbose },

	// 3. MachineModel.state
	{ "atxPower",				OBJECT_MODEL_FUNC_IF(self->gCodes->AtxPowerControlled(), self->platform->AtxPower()),	ObjectModelEntryFlags::live },
	{ "beep",					OBJECT_MODEL_FUNC_IF(self->beepDuration != 0, self, 4),					ObjectModelEntryFlags::none },
	{ "currentTool",			OBJECT_MODEL_FUNC((int32_t)self->GetCurrentToolNumber()),				ObjectModelEntryFlags::live },
	{ "displayMessage",			OBJECT_MODEL_FUNC(self->message.c_str()),								ObjectModelEntryFlags::none },
	{ "gpOut",					OBJECT_MODEL_FUNC_NOSELF(&gpoutArrayDescriptor),						ObjectModelEntryFlags::live },
#if SUPPORT_LASER
	// 2020-04-24: return the configured laser PWM even if the laser is temporarily turned off
	{ "laserPwm",				OBJECT_MODEL_FUNC_IF(self->gCodes->GetMachineType() == MachineType::laser, self->gCodes->GetLaserPwm(), 2),	ObjectModelEntryFlags::live },
#endif
#if HAS_MASS_STORAGE
	{ "logFile",				OBJECT_MODEL_FUNC(self->platform->GetLogFileName()),					ObjectModelEntryFlags::none },
#else
	{ "logFile",				OBJECT_MODEL_FUNC_NOSELF(nullptr),										ObjectModelEntryFlags::none },
#endif
	{ "machineMode",			OBJECT_MODEL_FUNC(self->gCodes->GetMachineModeString()),				ObjectModelEntryFlags::none },
	{ "messageBox",				OBJECT_MODEL_FUNC_IF(self->mbox.active, self, 5),						ObjectModelEntryFlags::none },
	{ "nextTool",				OBJECT_MODEL_FUNC((int32_t)self->gCodes->GetNewToolNumber()),			ObjectModelEntryFlags::live },
#if HAS_VOLTAGE_MONITOR
	{ "powerFailScript",		OBJECT_MODEL_FUNC(self->gCodes->GetPowerFailScript()),					ObjectModelEntryFlags::none },
#endif
	{ "previousTool",			OBJECT_MODEL_FUNC((int32_t)self->previousToolNumber),					ObjectModelEntryFlags::live },
	{ "restorePoints",			OBJECT_MODEL_FUNC_NOSELF(&restorePointsArrayDescriptor),				ObjectModelEntryFlags::none },
	{ "status",					OBJECT_MODEL_FUNC(self->GetStatusString()),								ObjectModelEntryFlags::live },
	{ "time",					OBJECT_MODEL_FUNC(DateTime(self->platform->GetDateTime())),				ObjectModelEntryFlags::live },
	{ "upTime",					OBJECT_MODEL_FUNC_NOSELF((int32_t)((millis64()/1000u) & 0x7FFFFFFF)),	ObjectModelEntryFlags::live },

	// 4. MachineModel.state.beep
	{ "duration",				OBJECT_MODEL_FUNC((int32_t)self->beepDuration),							ObjectModelEntryFlags::none },
	{ "frequency",				OBJECT_MODEL_FUNC((int32_t)self->beepFrequency),						ObjectModelEntryFlags::none },

	// 5. MachineModel.state.messageBox (FIXME acquire MutexLocker when accessing the following)
	{ "axisControls",			OBJECT_MODEL_FUNC((int32_t)self->mbox.controls.GetRaw()),				ObjectModelEntryFlags::none },
	{ "message",				OBJECT_MODEL_FUNC(self->mbox.message.c_str()),							ObjectModelEntryFlags::none },
	{ "mode",					OBJECT_MODEL_FUNC((int32_t)self->mbox.mode),							ObjectModelEntryFlags::none },
	{ "seq",					OBJECT_MODEL_FUNC((int32_t)self->mbox.seq),								ObjectModelEntryFlags::none },
	{ "timeout",				OBJECT_MODEL_FUNC((int32_t)self->mbox.timeout),							ObjectModelEntryFlags::none },
	{ "title",					OBJECT_MODEL_FUNC(self->mbox.title.c_str()),							ObjectModelEntryFlags::none },

	// 6. MachineModel.seqs
	{ "boards",					OBJECT_MODEL_FUNC((int32_t)self->boardsSeq),							ObjectModelEntryFlags::live },
#if HAS_MASS_STORAGE
	{ "directories",			OBJECT_MODEL_FUNC((int32_t)self->directoriesSeq),						ObjectModelEntryFlags::live },
#endif
	{ "fans",					OBJECT_MODEL_FUNC((int32_t)self->fansSeq),								ObjectModelEntryFlags::live },
	{ "heat",					OBJECT_MODEL_FUNC((int32_t)self->heatSeq),								ObjectModelEntryFlags::live },
	{ "inputs",					OBJECT_MODEL_FUNC((int32_t)self->inputsSeq),							ObjectModelEntryFlags::live },
	{ "job",					OBJECT_MODEL_FUNC((int32_t)self->jobSeq),								ObjectModelEntryFlags::live },
	// no need for 'limits' because it never changes
	{ "move",					OBJECT_MODEL_FUNC((int32_t)self->moveSeq),								ObjectModelEntryFlags::live },
#if HAS_NETWORKING
	{ "network",				OBJECT_MODEL_FUNC((int32_t)self->networkSeq),							ObjectModelEntryFlags::live },
	{ "reply",					OBJECT_MODEL_FUNC_NOSELF((int32_t)HttpResponder::GetReplySeq()),		ObjectModelEntryFlags::live },
#endif
#if SUPPORT_SCANNER
	{ "scanner",				OBJECT_MODEL_FUNC((int32_t)self->scannerSeq),							ObjectModelEntryFlags::live },
#endif
	{ "sensors",				OBJECT_MODEL_FUNC((int32_t)self->sensorsSeq),							ObjectModelEntryFlags::live },
	{ "spindles",				OBJECT_MODEL_FUNC((int32_t)self->spindlesSeq),							ObjectModelEntryFlags::live },
	{ "state",					OBJECT_MODEL_FUNC((int32_t)self->stateSeq),								ObjectModelEntryFlags::live },
	{ "tools",					OBJECT_MODEL_FUNC((int32_t)self->toolsSeq),								ObjectModelEntryFlags::live },
#if HAS_MASS_STORAGE
	{ "volumes",				OBJECT_MODEL_FUNC((int32_t)self->volumesSeq),							ObjectModelEntryFlags::live },
#endif
};

constexpr uint8_t RepRap::objectModelTableDescriptor[] =
{
	7,																		// number of sub-tables
	14 + SUPPORT_SCANNER + HAS_MASS_STORAGE,								// root
#if HAS_MASS_STORAGE
	8, 																		// directories
#else
	0,																		// directories
#endif
	25,																		// limits
	14 + HAS_VOLTAGE_MONITOR + SUPPORT_LASER,								// state
	2,																		// state.beep
	6,																		// state.messageBox
	10 + 2 * HAS_NETWORKING + SUPPORT_SCANNER + 2 * HAS_MASS_STORAGE		// seqs
};

DEFINE_GET_OBJECT_MODEL_TABLE(RepRap)

#endif

ReadWriteLock RepRap::toolListLock;

// RepRap member functions.

// Do nothing more in the constructor; put what you want in RepRap:Init()

RepRap::RepRap() noexcept
	: boardsSeq(0), directoriesSeq(0), fansSeq(0), heatSeq(0), inputsSeq(0), jobSeq(0), moveSeq(0),
	  networkSeq(0), scannerSeq(0), sensorsSeq(0), spindlesSeq(0), stateSeq(0), toolsSeq(0), volumesSeq(0),
	  toolList(nullptr), currentTool(nullptr), lastWarningMillis(0),
	  activeExtruders(0), activeToolHeaters(0), numToolsToReport(0),
	  ticksInSpinState(0), heatTaskIdleTicks(0), debug(0),
	  beepFrequency(0), beepDuration(0), beepTimer(0),
	  previousToolNumber(-1),
	  diagnosticsDestination(MessageType::NoDestinationMessage), justSentDiagnostics(false),
	  spinningModule(noModule), stopped(false), active(false), processingConfig(true)
#if HAS_LINUX_INTERFACE
	  , usingLinuxInterface(true)
#endif
{
	OutputBuffer::Init();
	platform = new Platform();
#if HAS_LINUX_INTERFACE
	linuxInterface = new LinuxInterface();				// needs to be allocated early so as to avoid the last 64K of RAM
#endif
	network = new Network(*platform);
	gCodes = new GCodes(*platform);
	move = new Move();
	heat = new Heat();
	printMonitor = new PrintMonitor(*platform, *gCodes);
	fansManager = new FansManager;

#if SUPPORT_ROLAND
	roland = new Roland(*platform);
#endif
#if SUPPORT_SCANNER
	scanner = new Scanner(*platform);
#endif
#if SUPPORT_IOBITS
	portControl = new PortControl();
#endif
#if SUPPORT_12864_LCD
 	display = new Display();
#endif
#if SUPPORT_CAN_EXPANSION
	expansion = new ExpansionManager();
#endif

	SetPassword(DEFAULT_PASSWORD);
	message.Clear();
#if SUPPORT_12864_LCD
	messageSequence = 0;
#endif
}

void RepRap::Init() noexcept
{
	messageBoxMutex.Create("MessageBox");

	platform->Init();
	network->Init();
	SetName(DEFAULT_MACHINE_NAME);		// Network must be initialised before calling this because this calls SetHostName
	gCodes->Init();						// must be called before Move::Init
#if SUPPORT_CAN_EXPANSION
	CanInterface::Init();
#endif
	move->Init();
	heat->Init();
	fansManager->Init();
	printMonitor->Init();
	FilamentMonitor::InitStatic();

#if SUPPORT_ROLAND
	roland->Init();
#endif
#if SUPPORT_SCANNER
	scanner->Init();
#endif
#if SUPPORT_IOBITS
	portControl->Init();
#endif
#if SUPPORT_12864_LCD
	display->Init();
#endif
#if HAS_LINUX_INTERFACE
	linuxInterface->Init();
#endif

	// Set up the timeout of the regular watchdog, and set up the backup watchdog if there is one.
#ifdef __LPC17xx__
	wdt_init(1); // set wdt to 1 second. reset the processor on a watchdog fault
#else
	{
		// The clock frequency for both watchdogs is about 32768/128 = 256Hz
		// The watchdogs on the SAM4E seem to be very timing-sensitive. On the Duet WiFi/Ethernet they were going off spuriously depending on how long the DueX initialisation took.
		// The documentation says you mustn't write to the mode register within 3 slow clocks after kicking the watchdog.
		// I have a theory that the converse is also true, i.e. after enabling the watchdog you mustn't kick it within 3 slow clocks
		// So I've added a delay call before we set 'active' true (which enables kicking the watchdog), and that seems to fix the problem.
		const uint16_t timeout = 32768/128;											// set watchdog timeout to 1 second (max allowed value is 4095 = 16 seconds)
		wdt_init(WDT, WDT_MR_WDRSTEN | WDT_MR_WDDBGHLT, timeout, timeout);			// reset the processor on a watchdog fault, stop it when debugging

#if SAM4E || SAME70
		// The RSWDT must be initialised *after* the main WDT
		const uint16_t rsTimeout = 16384/128;										// set secondary watchdog timeout to 0.5 second (max allowed value is 4095 = 16 seconds)
		rswdt_init(RSWDT, RSWDT_MR_WDFIEN | RSWDT_MR_WDDBGHLT, rsTimeout, rsTimeout);	// generate an interrupt on a watchdog fault
		NVIC_EnableIRQ(WDT_IRQn);													// enable the watchdog interrupt
#endif
		delayMicroseconds(200);														// 200us is about 6 slow clocks
	}
#endif

	active = true;										// must do this before we start the network or call Spin(), else the watchdog may time out

	platform->MessageF(UsbMessage, "\n%s Version %s dated %s\n", FIRMWARE_NAME, VERSION, DATE);

#if HAS_MASS_STORAGE
	// Try to mount the first SD card
	{
		GCodeResult rslt;
		String<100> reply;
		do
		{
			MassStorage::Spin();						// Spin() doesn't get called regularly until after this function completes, and we need it to update the card detect status
			rslt = MassStorage::Mount(0, reply.GetRef(), false);
		}
		while (rslt == GCodeResult::notFinished);

		if (rslt == GCodeResult::ok)
		{
			// Run the configuration file

# if HAS_LINUX_INTERFACE
			usingLinuxInterface = false;				// try to run config.g from the SD card
# endif
			if (RunStartupFile(GCodes::CONFIG_FILE) || RunStartupFile(GCodes::CONFIG_BACKUP_FILE))
			{
				// Processed config.g so OK
			}
			else
			{
# if HAS_LINUX_INTERFACE
				usingLinuxInterface = true;				// we failed to open config.g or default.g so assume we have a SBC connected
# else
				platform->Message(UsbMessage, "Error, no configuration file found\n");
# endif
			}
		}
		else
		{
# if !HAS_LINUX_INTERFACE
			delay(3000);								// Wait a few seconds so users have a chance to see this
			platform->MessageF(UsbMessage, "%s\n", reply.c_str());
# endif
		}
	}
#endif


#if HAS_LINUX_INTERFACE
	if (usingLinuxInterface)
	{
		processingConfig = false;
		gCodes->RunConfigFile(GCodes::CONFIG_FILE);		// we didn't get config.g from SD card so request it from Linux
		network->Activate();							// need to do this here, as the configuration GCodes may set IP address etc.
	}
	else
#endif
	{
		network->Activate();							// need to do this here, as the configuration GCodes may set IP address etc.
#if HAS_MASS_STORAGE
		// If we are running from SD card, run the runonce.g file if it exists, then delete it
		if (RunStartupFile(GCodes::RUNONCE_G))
		{
			platform->DeleteSysFile(GCodes::RUNONCE_G);
		}
#endif
		processingConfig = false;
	}

#if HAS_HIGH_SPEED_SD
	hsmci_set_idle_func(hsmciIdle);
	HSMCI->HSMCI_IDR = 0xFFFFFFFF;	// disable all HSMCI interrupts
	NVIC_EnableIRQ(HSMCI_IRQn);
#endif

	platform->MessageF(UsbMessage, "%s is up and running.\n", FIRMWARE_NAME);

	fastLoop = UINT32_MAX;
	slowLoop = 0;
}

// Run a startup file
bool RepRap::RunStartupFile(const char *filename) noexcept
{
	bool rslt = gCodes->RunConfigFile(filename);
	if (rslt)
	{
		platform->MessageF(UsbMessage, "Executing %s...", filename);
		do
		{
			// GCodes::Spin will process the macro file and ensure IsDaemonBusy returns false when it's done
			Spin();
		} while (gCodes->IsDaemonBusy());
		platform->Message(UsbMessage, "Done!\n");
	}
	return rslt;
}

void RepRap::Exit() noexcept
{
#if HAS_HIGH_SPEED_SD
	hsmci_set_idle_func(nullptr);
#endif
	active = false;
	heat->Exit();
	move->Exit();
	gCodes->Exit();
#if SUPPORT_SCANNER
	scanner->Exit();
#endif
#if SUPPORT_IOBITS
	portControl->Exit();
#endif
#if SUPPORT_12864_LCD
 	display->Exit();
#endif
	network->Exit();
	platform->Exit();
}

void RepRap::Spin() noexcept
{
	if (!active)
	{
		return;
	}

	const uint32_t lastTime = StepTimer::GetTimerTicks();

	ticksInSpinState = 0;
	spinningModule = modulePlatform;
	platform->Spin();

	ticksInSpinState = 0;
	spinningModule = moduleGcodes;
	gCodes->Spin();

	ticksInSpinState = 0;
	spinningModule = moduleMove;
	move->Spin();

#if SUPPORT_ROLAND
	ticksInSpinState = 0;
	spinningModule = moduleRoland;
	roland->Spin();
#endif

#if SUPPORT_SCANNER && !SCANNER_AS_SEPARATE_TASK
	ticksInSpinState = 0;
	spinningModule = moduleScanner;
	scanner->Spin();
#endif

	ticksInSpinState = 0;
	spinningModule = modulePrintMonitor;
	printMonitor->Spin();

	ticksInSpinState = 0;
	spinningModule = moduleFilamentSensors;
	FilamentMonitor::Spin();

#if SUPPORT_12864_LCD
	ticksInSpinState = 0;
	spinningModule = moduleDisplay;
	display->Spin();
#endif

#if HAS_LINUX_INTERFACE
	ticksInSpinState = 0;
	spinningModule = moduleLinuxInterface;
	linuxInterface->Spin();
#endif

	ticksInSpinState = 0;
	spinningModule = noModule;

	// Check if we need to send diagnostics
	if (diagnosticsDestination != MessageType::NoDestinationMessage)
	{
		Diagnostics(diagnosticsDestination);
		diagnosticsDestination = MessageType::NoDestinationMessage;
	}

	// Check if we need to display a cold extrusion warning
	const uint32_t now = millis();
	if (now - lastWarningMillis >= MinimumWarningInterval)
	{
		ReadLocker lock(toolListLock);
		for (Tool *t = toolList; t != nullptr; t = t->Next())
		{
			if (t->DisplayColdExtrudeWarning())
			{
				platform->MessageF(WarningMessage, "Tool %d was not driven because its heater temperatures were not high enough or it has a heater fault\n", t->myNumber);
				lastWarningMillis = now;
			}
		}
	}

	// Check if the beep request can be cleared
	if (beepTimer != 0 && now - beepTimer >= beepDuration)
	{
		beepDuration = beepFrequency = beepTimer = 0;
		StateUpdated();
	}

	// Check if the message box can be hidden
	{
		MutexLocker lock(messageBoxMutex);
		if (mbox.active && mbox.timer != 0 && now - mbox.timer >= mbox.timeout)
		{
			mbox.active = false;
			StateUpdated();
		}
	}

	// Keep track of the loop time
	if (justSentDiagnostics)
	{
		// Sending diagnostics increases the loop time, so don't count it
		justSentDiagnostics = false;
	}
	else
	{
		const uint32_t now = StepTimer::GetTimerTicks();
		const uint32_t dt = now - lastTime;
#if 0 //DEBUG
		if (dt > 1000000)
		{
			platform->MessageF(ErrorMessage, "dt %" PRIu32 " now %08" PRIx32 " last %08" PRIx32 "\n", dt, now, lastTime);
		}
#endif
		if (dt < fastLoop)
		{
			fastLoop = dt;
		}
		if (dt > slowLoop)
		{
			slowLoop = dt;
		}
	}

	RTOSIface::Yield();
}

void RepRap::Timing(MessageType mtype) noexcept
{
	platform->MessageF(mtype, "Slowest loop: %.2fms; fastest: %.2fms\n", (double)(slowLoop * StepTimer::StepClocksToMillis), (double)(fastLoop * StepTimer::StepClocksToMillis));
	fastLoop = UINT32_MAX;
	slowLoop = 0;
}

void RepRap::Diagnostics(MessageType mtype) noexcept
{
	platform->Message(mtype, "=== Diagnostics ===\n");

//	platform->MessageF(mtype, "platform %" PRIx32 ", network %" PRIx32 ", move %" PRIx32 ", heat %" PRIx32 ", gcodes %" PRIx32 ", scanner %"  PRIx32 ", pm %" PRIx32 ", portc %" PRIx32 "\n",
//						(uint32_t)platform, (uint32_t)network, (uint32_t)move, (uint32_t)heat, (uint32_t)gCodes, (uint32_t)scanner, (uint32_t)printMonitor, (uint32_t)portControl);

	// Print the firmware version and board type

#ifdef DUET_NG
	platform->MessageF(mtype, "%s version %s running on %s", FIRMWARE_NAME, VERSION, platform->GetElectronicsString());
	const char* const expansionName = DuetExpansion::GetExpansionBoardName();
	platform->MessageF(mtype, (expansionName == nullptr) ? "\n" : " + %s\n", expansionName);
#elif defined(__LPC17xx__)
	platform->MessageF(mtype, "%s (%s) version %s running on %s at %dMhz\n", FIRMWARE_NAME, lpcBoardName, VERSION, platform->GetElectronicsString(), (int)SystemCoreClock/1000000);
#elif HAS_LINUX_INTERFACE
	platform->MessageF(mtype, "%s version %s running on %s (%s mode)\n", FIRMWARE_NAME, VERSION, platform->GetElectronicsString(),
						(UsingLinuxInterface()) ? "SBC" : "standalone");
#else
	platform->MessageF(mtype, "%s version %s running on %s\n", FIRMWARE_NAME, VERSION, platform->GetElectronicsString());
#endif

#if SUPPORTS_UNIQUE_ID
	platform->MessageF(mtype, "Board ID: %s\n", platform->GetUniqueIdString());
#endif

	// Show the used and free buffer counts. Do this early in case we are running out of them and the diagnostics get truncated.
	OutputBuffer::Diagnostics(mtype);

	// Now print diagnostics for other modules
	Tasks::Diagnostics(mtype);
	platform->Diagnostics(mtype);				// this includes a call to our Timing() function
#if HAS_MASS_STORAGE
	MassStorage::Diagnostics(mtype);
#endif
	move->Diagnostics(mtype);
	heat->Diagnostics(mtype);
	gCodes->Diagnostics(mtype);
	network->Diagnostics(mtype);
	FilamentMonitor::Diagnostics(mtype);
#ifdef DUET_NG
	DuetExpansion::Diagnostics(mtype);
#endif
#if SUPPORT_CAN_EXPANSION
	CanInterface::Diagnostics(mtype);
#endif
#if HAS_LINUX_INTERFACE
	linuxInterface->Diagnostics(mtype);
#endif
	justSentDiagnostics = true;
}

// Turn off the heaters, disable the motors, and deactivate the Heat and Move classes. Leave everything else working.
void RepRap::EmergencyStop() noexcept
{
	stopped = true;				// a useful side effect of setting this is that it prevents Platform::Tick being called, which is needed when loading IAP into RAM

	// Do not turn off ATX power here. If the nozzles are still hot, don't risk melting any surrounding parts by turning fans off.
	//platform->SetAtxPower(false);

	switch (gCodes->GetMachineType())
	{
	case MachineType::cnc:
		for (size_t i = 0; i < MaxSpindles; i++)
		{
			platform->AccessSpindle(i).TurnOff();
		}
		break;

#if SUPPORT_LASER
	case MachineType::laser:
		platform->SetLaserPwm(0);
		break;
#endif

	default:
		break;
	}

	heat->Exit();								// this also turns off all heaters
	move->Exit();								// this stops the motors stepping

#if SUPPORT_CAN_EXPANSION
	expansion->EmergencyStop();
#endif

	gCodes->EmergencyStop();
	platform->StopLogging();
}

// Perform a software reset. 'stk' points to the program counter on the stack if the cause is an exception, otherwise it is nullptr.
void RepRap::SoftwareReset(uint16_t reason, const uint32_t *stk) noexcept
{
	cpu_irq_disable();							// disable interrupts before we call any flash functions. We don't enable them again.
	wdt_restart(WDT);							// kick the watchdog

#if SAM4E || SAME70
	rswdt_restart(RSWDT);						// kick the secondary watchdog
#endif

	Cache::Disable();

#if USE_MPU
	//TODO set the flash memory to strongly-ordered or device instead
	ARM_MPU_Disable();							// disable the MPU
#endif

	if (reason == (uint16_t)SoftwareResetReason::erase)
	{
		EraseAndReset();
 	}
	else
	{
		if (reason != (uint16_t)SoftwareResetReason::user)
		{
			if (SERIAL_MAIN_DEVICE.canWrite() == 0)
			{
				reason |= (uint16_t)SoftwareResetReason::inUsbOutput;	// if we are resetting because we are stuck in a Spin function, record whether we are trying to send to USB
			}

#ifdef SERIAL_AUX_DEVICE
			if (SERIAL_AUX_DEVICE.canWrite() == 0
# ifdef SERIAL_AUX2_DEVICE
				|| SERIAL_AUX2_DEVICE.canWrite() == 0
# endif
			   )
			{
				reason |= (uint16_t)SoftwareResetReason::inAuxOutput;	// if we are resetting because we are stuck in a Spin function, record whether we are trying to send to aux
			}
#endif
		}
		reason |= (uint8_t)reprap.GetSpinningModule();
		if (platform->WasDeliberateError())
		{
			reason |= (uint16_t)SoftwareResetReason::deliberate;
		}

		// Record the reason for the software reset
		// First find a free slot (wear levelling)
		size_t slot = SoftwareResetData::numberOfSlots;
#if defined(__LPC17xx__)
        SoftwareResetData srdBuf[1];
#else
        SoftwareResetData srdBuf[SoftwareResetData::numberOfSlots];
#endif

#if SAM4E || SAM4S || SAME70
		if (flash_read_user_signature(reinterpret_cast<uint32_t*>(srdBuf), sizeof(srdBuf)/sizeof(uint32_t)) == FLASH_RC_OK)
#elif SAM3XA
		DueFlashStorage::read(SoftwareResetData::nvAddress, srdBuf, sizeof(srdBuf));
#elif defined(__LPC17xx__)
		// nothing here
#else
# error
#endif
		{
#if defined(__LPC17xx__)
            while (slot != 0 && LPC_IsSoftwareResetDataSlotVacant(slot - 1))
#else
            while (slot != 0 && srdBuf[slot - 1].isVacant())
#endif
			{
				--slot;
			}
		}

		if (slot == SoftwareResetData::numberOfSlots)
		{
			// No free slots, so erase the area
#if SAM4E || SAM4S || SAME70
			flash_erase_user_signature();
#elif defined(__LPC17xx__)
			LPC_EraseSoftwareResetDataSlots(); // erase the last flash sector
#endif
			memset(srdBuf, 0xFF, sizeof(srdBuf));
			slot = 0;
		}

#if defined(__LPC17xx__)
        srdBuf[0].Populate(reason, (uint32_t)realTime, stk);
#else
        srdBuf[slot].Populate(reason, (uint32_t)platform->GetDateTime(), stk);
#endif

		// Save diagnostics data to Flash
#if SAM4E || SAM4S || SAME70
		flash_write_user_signature(srdBuf, sizeof(srdBuf)/sizeof(uint32_t));
#elif defined(__LPC17xx__)
		LPC_WriteSoftwareResetData(slot, srdBuf, sizeof(srdBuf));
#else
		DueFlashStorage::write(SoftwareResetData::nvAddress, srdBuf, sizeof(srdBuf));
#endif
	}

#if defined(__LPC17xx__)
    LPC_SYSCTL->RSID = 0x3F;					// Clear bits in reset reasons stored in RSID
#else
# ifndef RSTC_MR_KEY_PASSWD
// Definition of RSTC_MR_KEY_PASSWD is missing in the SAM3X ASF files
#  define RSTC_MR_KEY_PASSWD (0xA5u << 24)
# endif
	RSTC->RSTC_MR = RSTC_MR_KEY_PASSWD;			// ignore any signal on the NRST pin for now so that the reset reason will show as Software
#endif
	Reset();
	for(;;) {}
}

void RepRap::SetDebug(Module m, bool enable) noexcept
{
	if (m < numModules)
	{
		if (enable)
		{
			debug |= (1u << m);
		}
		else
		{
			debug &= ~(1u << m);
		}
	}
}

void RepRap::ClearDebug() noexcept
{
	debug = 0;
}

void RepRap::PrintDebug(MessageType mt) noexcept
{
	platform->Message((MessageType)(mt | PushFlag), "Debugging enabled for modules:");
	for (size_t i = 0; i < numModules; i++)
	{
		if ((debug & (1u << i)) != 0)
		{
			platform->MessageF((MessageType)(mt | PushFlag), " %s(%u)", GetModuleName(i), i);
		}
	}

	platform->Message((MessageType)(mt | PushFlag), "\nDebugging disabled for modules:");
	for (size_t i = 0; i < numModules; i++)
	{
		if ((debug & (1u << i)) == 0)
		{
			platform->MessageF((MessageType)(mt | PushFlag), " %s(%u)", GetModuleName(i), i);
		}
	}
	platform->Message(mt, "\n");
}

// Add a tool.
// Prior to calling this, delete any existing tool with the same number
// The tool list is maintained in tool number order.
void RepRap::AddTool(Tool* tool) noexcept
{
	WriteLocker lock(toolListLock);
	Tool** t = &toolList;
	while(*t != nullptr && (*t)->Number() < tool->Number())
	{
		t = &((*t)->next);
	}
	tool->next = *t;
	*t = tool;
	tool->UpdateExtruderAndHeaterCount(activeExtruders, activeToolHeaters, numToolsToReport);
	platform->UpdateConfiguredHeaters();
	ToolsUpdated();
}

void RepRap::DeleteTool(int toolNumber) noexcept
{
	WriteLocker lock(toolListLock);

	// Deselect it if necessary
	if (currentTool != nullptr && currentTool->Number() == toolNumber)
	{
		SelectTool(-1, false);
	}

	// Purge any references to this tool
	Tool * tool = nullptr;
	for (Tool **t = &toolList; *t != nullptr; t = &((*t)->next))
	{
		if ((*t)->Number() == toolNumber)
		{
			tool = *t;
			*t = tool->next;

			// Switch off any associated heaters
			for (size_t i = 0; i < tool->HeaterCount(); i++)
			{
				heat->SwitchOff(tool->Heater(i));
			}

			break;
		}
	}

	// Delete it
	Tool::Delete(tool);

	// Update the number of active heaters and extruder drives
	activeExtruders = activeToolHeaters = numToolsToReport = 0;
	for (Tool *t = toolList; t != nullptr; t = t->Next())
	{
		t->UpdateExtruderAndHeaterCount(activeExtruders, activeToolHeaters, numToolsToReport);
	}
	platform->UpdateConfiguredHeaters();
	ToolsUpdated();
}

// Select the specified tool, putting the existing current tool into standby
void RepRap::SelectTool(int toolNumber, bool simulating) noexcept
{
	ReadLockedPointer<Tool> const newTool = GetTool(toolNumber);
	if (!simulating)
	{
		if (currentTool != nullptr && currentTool != newTool.Ptr())
		{
			currentTool->Standby();
		}
		if (newTool.IsNotNull())
		{
			newTool->Activate();
		}
	}
	currentTool = newTool.Ptr();
}

void RepRap::PrintTool(int toolNumber, const StringRef& reply) const noexcept
{
	ReadLockedPointer<Tool> const tool = GetTool(toolNumber);
	if (tool.IsNotNull())
	{
		tool->Print(reply);
	}
	else
	{
		reply.copy("Error: Attempt to print details of non-existent tool.\n");
	}
}

void RepRap::StandbyTool(int toolNumber, bool simulating) noexcept
{
	ReadLockedPointer<Tool> const tool = GetTool(toolNumber);
	if (tool.IsNotNull())
	{
		if (!simulating)
		{
			tool->Standby();
		}
  		if (currentTool == tool.Ptr())
		{
			currentTool = nullptr;
		}
	}
	else
	{
		platform->MessageF(ErrorMessage, "Attempt to standby a non-existent tool: %d\n", toolNumber);
	}
}

ReadLockedPointer<Tool> RepRap::GetLockedCurrentTool() const noexcept
{
	ReadLocker lock(toolListLock);
	return ReadLockedPointer<Tool>(lock, currentTool);
}

ReadLockedPointer<Tool> RepRap::GetTool(int toolNumber) const noexcept
{
	ReadLocker lock(toolListLock);
	Tool* tool = toolList;
	while (tool != nullptr)
	{
		if (tool->Number() == toolNumber)
		{
			return ReadLockedPointer<Tool>(lock, tool);
		}
		tool = tool->Next();
	}
	return ReadLockedPointer<Tool>(lock, nullptr);		 // not an error
}

// Return the current tool number, or -1 if no tool selected
int RepRap::GetCurrentToolNumber() const noexcept
{
	return (currentTool == nullptr) ? -1 : currentTool->Number();
}

// Get the current tool, or failing that the default tool. May return nullptr if we can't
// Called when a M104 or M109 command doesn't specify a tool number.
ReadLockedPointer<Tool> RepRap::GetCurrentOrDefaultTool() const noexcept
{
	ReadLocker lock(toolListLock);
	// If a tool is already selected, use that one, else use the lowest-numbered tool which is the one at the start of the tool list
	return ReadLockedPointer<Tool>(lock, (currentTool != nullptr) ? currentTool : toolList);
}

// Return the lowest-numbered tool
ReadLockedPointer<Tool> RepRap::GetFirstTool() const noexcept
{
	ReadLocker lock(toolListLock);
	return ReadLockedPointer<Tool>(lock, toolList);
}

bool RepRap::IsHeaterAssignedToTool(int8_t heater) const noexcept
{
	ReadLocker lock(toolListLock);
	for (Tool *tool = toolList; tool != nullptr; tool = tool->Next())
	{
		for (size_t i = 0; i < tool->HeaterCount(); i++)
		{
			if (tool->Heater(i) == heater)
			{
				// It's already in use by some tool
				return true;
			}
		}
	}

	return false;
}

unsigned int RepRap::GetNumberOfContiguousTools() const noexcept
{
	int numTools = 0;
	ReadLocker lock(toolListLock);
	for (const Tool *t = toolList; t != nullptr && t->Number() == numTools; t = t->Next())
	{
		++numTools;
	}
	return (unsigned int)numTools;
}

// Report the temperatures of one tool in M105 format
void RepRap::ReportToolTemperatures(const StringRef& reply, const Tool *tool, bool includeNumber) const noexcept
{
	if (tool != nullptr && tool->HeaterCount() != 0)
	{
		if (reply.strlen() != 0)
		{
			reply.cat(' ');
		}
		if (includeNumber)
		{
			reply.catf("T%u", tool->Number());
		}
		else
		{
			reply.cat("T");
		}

		Heat& heat = reprap.GetHeat();
		char sep = ':';
		for (size_t i = 0; i < tool->HeaterCount(); ++i)
		{
			const int heater = tool->Heater(i);
			reply.catf("%c%.1f /%.1f", sep, (double)heat.GetHeaterTemperature(heater), (double)heat.GetTargetTemperature(heater));
			sep = ' ';
		}
	}
}

void RepRap::ReportAllToolTemperatures(const StringRef& reply) const noexcept
{
	ReadLocker lock(toolListLock);

	// The following is believed to be compatible with Marlin and Octoprint, based on thread https://github.com/foosel/OctoPrint/issues/2590#issuecomment-385023980
	ReportToolTemperatures(reply, currentTool, false);

	for (const Tool *tool = toolList; tool != nullptr; tool = tool->Next())
	{
		ReportToolTemperatures(reply, tool, true);
	}
}

void RepRap::SetAllToolsFirmwareRetraction(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	for (Tool *tool = toolList; tool != nullptr; tool = tool->Next())
	{
		tool->SetFirmwareRetraction(gb, reply);
	}
}

// Get the current axes used as X axes
AxesBitmap RepRap::GetCurrentXAxes() const noexcept
{
	return Tool::GetXAxes(currentTool);
}

// Get the current axes used as Y axes
AxesBitmap RepRap::GetCurrentYAxes() const noexcept
{
	return Tool::GetYAxes(currentTool);
}

// Set the previous tool number. Inline because it is only called from one place.
void RepRap::SetPreviousToolNumber() noexcept
{
	previousToolNumber = (currentTool != nullptr) ? currentTool->Number() : -1;
}

void RepRap::Tick() noexcept
{
	// Kicking the watchdog before it has been initialised may trigger it!
	if (active)
	{
		wdt_restart(WDT);							// kick the watchdog
#if SAM4E || SAME70
		rswdt_restart(RSWDT);						// kick the secondary watchdog
#endif

		if (!stopped)
		{
			platform->Tick();
			++ticksInSpinState;
			++heatTaskIdleTicks;
			const bool heatTaskStuck = (heatTaskIdleTicks >= MaxTicksInSpinState);
			if (heatTaskStuck || ticksInSpinState >= MaxTicksInSpinState)		// if we stall for 20 seconds, save diagnostic data and reset
			{
				stopped = true;
				heat->SwitchOffAll(true);
				platform->EmergencyDisableDrivers();

				// We now save the stack when we get stuck in a spin loop
				__asm volatile("mrs r2, psp");
				register const uint32_t * stackPtr asm ("r2");					// we want the PSP not the MSP
				SoftwareReset(
					(heatTaskStuck) ? (uint16_t)SoftwareResetReason::heaterWatchdog : (uint16_t)SoftwareResetReason::stuckInSpin,
					stackPtr + 5);												// discard uninteresting registers, keep LR PC PSR
			}
		}
	}
}

// Return true if we are close to timeout
bool RepRap::SpinTimeoutImminent() const noexcept
{
	return ticksInSpinState >= HighTicksInSpinState;
}

// Get the JSON status response for the web server (or later for the M105 command).
// Type 1 is the ordinary JSON status response.
// Type 2 is the same except that static parameters are also included.
// Type 3 is the same but instead of static parameters we report print estimation values.
OutputBuffer *RepRap::GetStatusResponse(uint8_t type, ResponseSource source) const noexcept
{
	// Need something to write to...
	OutputBuffer *response;
	if (!OutputBuffer::Allocate(response))
	{
		return nullptr;
	}

	// Machine status
	response->printf("{\"status\":\"%c\",\"coords\":{", GetStatusCharacter());

	// Homed axes
	const size_t numVisibleAxes = gCodes->GetVisibleAxes();
	AppendIntArray(response, "axesHomed", numVisibleAxes, [this](size_t axis) noexcept { return (gCodes->IsAxisHomed(axis)) ? 1 : 0; });

	// XYZ positions
	// Coordinates may be NaNs or infinities, for example when delta or SCARA homing fails. We must replace any NaNs or infinities to avoid JSON parsing errors.
	// Ideally we would report "unknown" or similar for axis positions that are not known because we haven't homed them, but that requires changes to both DWC and PanelDue.
	// So we report 9999.9 instead.

	// First the user coordinates
#if SUPPORT_WORKPLACE_COORDINATES
	response->catf(",\"wpl\":%u,", gCodes->GetWorkplaceCoordinateSystemNumber());
#else
	response->cat(',');
#endif

	AppendFloatArray(response, "xyz", numVisibleAxes, [this](size_t axis) noexcept { return gCodes->GetUserCoordinate(axis); }, 3);

	// Machine coordinates
	response->cat(',');
	AppendFloatArray(response, "machine", numVisibleAxes, [this](size_t axis) noexcept { return move->LiveCoordinate(axis, currentTool); }, 3);

	// Actual extruder positions since power up, last G92 or last M23
	response->cat(',');
	AppendFloatArray(response, "extr", GetExtrudersInUse(), [this](size_t extruder) noexcept { return move->LiveCoordinate(ExtruderToLogicalDrive(extruder), currentTool); }, 1);

	// Current speeds
	response->catf("},\"speeds\":{\"requested\":%.1f,\"top\":%.1f}", (double)move->GetRequestedSpeed(), (double)move->GetTopSpeed());

	// Current tool number
	response->catf(",\"currentTool\":%d", GetCurrentToolNumber());

	// Output notifications
	{
		const bool sendBeep = ((source == ResponseSource::AUX || !platform->IsAuxEnabled()) && beepDuration != 0 && beepFrequency != 0);
		const bool sendMessage = !message.IsEmpty();

		float timeLeft = 0.0;
		MutexLocker lock(messageBoxMutex);

		if (mbox.active && mbox.timer != 0)
		{
			timeLeft = (float)(mbox.timeout) / 1000.0 - (float)(millis() - mbox.timer) / 1000.0;
		}

		if (sendBeep || sendMessage || mbox.active)
		{
			response->cat(",\"output\":{");

			// Report beep values
			if (sendBeep)
			{
				response->catf("\"beepDuration\":%u,\"beepFrequency\":%u", beepDuration, beepFrequency);
				if (sendMessage || mbox.active)
				{
					response->cat(',');
				}
			}

			// Report message
			if (sendMessage)
			{
				response->cat("\"message\":");
				response->EncodeString(message, false);
				if (mbox.active)
				{
					response->cat(',');
				}
			}

			// Report message box
			if (mbox.active)
			{
				response->cat("\"msgBox\":{\"msg\":");
				response->EncodeString(mbox.message, false);
				response->cat(",\"title\":");
				response->EncodeString(mbox.title, false);
				response->catf(",\"mode\":%d,\"seq\":%" PRIu32 ",\"timeout\":%.1f,\"controls\":%u}", mbox.mode, mbox.seq, (double)timeLeft, mbox.controls.GetRaw());
			}
			response->cat('}');
		}
	}

	// ATX power
	response->catf(",\"params\":{\"atxPower\":%d", gCodes->AtxPowerControlled() ? (platform->AtxPower() ? 1 : 0) : -1);

	// Parameters
	{
		// Cooling fan values
		response->cat(',');
		const size_t numFans = fansManager->GetNumFansToReport();
		AppendIntArray(response, "fanPercent", numFans,
						[this](size_t fan) noexcept
						{
							const float fanValue = fansManager->GetFanValue(fan);
							return  (fanValue < 0.0) ? -1 : (int)lrintf(fanValue * 100.0);
						});

		// Cooling fan names
		if (type == 2)
		{
			response->cat(',');
			AppendStringArray(response, "fanNames", numFans, [this](size_t fan) noexcept { return fansManager->GetFanName(fan); });
		}

		// Speed and Extrusion factors in %
		response->catf(",\"speedFactor\":%.1f,", (double)(gCodes->GetSpeedFactor() * 100.0));
		AppendFloatArray(response, "extrFactors", GetExtrudersInUse(), [this](size_t extruder) noexcept { return gCodes->GetExtrusionFactor(extruder) * 100.0; }, 1);

		// Z babystepping
		response->catf(",\"babystep\":%.3f}", (double)gCodes->GetTotalBabyStepOffset(Z_AXIS));

		// G-code reply sequence for webserver (sequence number for AUX is handled later)
		if (source == ResponseSource::HTTP)
		{
			response->catf(",\"seq\":%" PRIu32, network->GetHttpReplySeq());
		}

		// Sensors
		response->cat(",\"sensors\":{");

		// Probe
		const auto zp = platform->GetZProbeOrDefault(0);
		const int v0 = zp->GetReading();
		int v1;
		switch (zp->GetSecondaryValues(v1))
		{
			case 1:
				response->catf("\"probeValue\":%d,\"probeSecondary\":[%d],", v0, v1);
				break;
			default:
				response->catf("\"probeValue\":%d,", v0);
				break;
		}

		// Send fan RPM value(s)
		AppendIntArray(response, "fanRPM", numFans, [this](size_t fan) noexcept { return (int)fansManager->GetFanRPM(fan); });
		response->cat('}');
	}

	/* Temperatures */
	{
		response->cat(",\"temps\":{");

		/* Bed */
		const int8_t bedHeater = (MaxBedHeaters > 0) ? heat->GetBedHeater(0) : -1;
		if (bedHeater != -1)
		{
			response->catf("\"bed\":{\"current\":%.1f,\"active\":%.1f,\"standby\":%.1f,\"state\":%u,\"heater\":%d},",
				(double)heat->GetHeaterTemperature(bedHeater), (double)heat->GetActiveTemperature(bedHeater), (double)heat->GetStandbyTemperature(bedHeater),
					heat->GetStatus(bedHeater).ToBaseType(), bedHeater);
		}

		/* Chamber */
		const int8_t chamberHeater = (MaxChamberHeaters > 0) ? heat->GetChamberHeater(0) : -1;
		if (chamberHeater != -1)
		{
			response->catf("\"chamber\":{\"current\":%.1f,\"active\":%.1f,\"state\":%u,\"heater\":%d},",
				(double)heat->GetHeaterTemperature(chamberHeater), (double)heat->GetActiveTemperature(chamberHeater),
					heat->GetStatus(chamberHeater).ToBaseType(), chamberHeater);
		}

		/* Cabinet */
		const int8_t cabinetHeater = (MaxChamberHeaters > 1) ? heat->GetChamberHeater(1) : -1;
		if (cabinetHeater != -1)
		{
			response->catf("\"cabinet\":{\"current\":%.1f,\"active\":%.1f,\"state\":%u,\"heater\":%d},",
				(double)heat->GetHeaterTemperature(cabinetHeater), (double)heat->GetActiveTemperature(cabinetHeater),
					heat->GetStatus(cabinetHeater).ToBaseType(), cabinetHeater);
		}

		/* Heaters */

		// Current temperatures
		{
			const size_t numHeaters = heat->GetNumHeatersToReport();
			AppendFloatArray(response, "current", numHeaters, [this](size_t heater) noexcept { return heat->GetHeaterTemperature(heater); }, 1);

			// Current states
			response->cat(',');
			AppendIntArray(response, "state", numHeaters, [this](size_t heater) noexcept { return (int)heat->GetStatus(heater).ToBaseType(); });

			// Names of the sensors use to control heaters
			if (type == 2)
			{
				response->cat(',');
				AppendStringArray(response, "names", numHeaters, [this](size_t heater) noexcept { return heat->GetHeaterSensorName(heater); });
			}
		}

		// Tool temperatures
		response->cat(",\"tools\":{\"active\":[");
		{
			ReadLocker lock(toolListLock);
			for (const Tool *tool = toolList; tool != nullptr; tool = tool->Next())
			{
				AppendFloatArray(response, nullptr, tool->heaterCount, [tool](unsigned int n) noexcept { return tool->activeTemperatures[n]; }, 1);
				if (tool->Next() != nullptr)
				{
					response->cat(',');
				}
			}

			response->cat("],\"standby\":[");
			for (const Tool *tool = toolList; tool != nullptr; tool = tool->Next())
			{
				AppendFloatArray(response, nullptr, tool->heaterCount, [tool](unsigned int n) noexcept { return tool->standbyTemperatures[n]; }, 1);
				if (tool->Next() != nullptr)
				{
					response->cat(',');
				}
			}
		}

		// Named extra temperature sensors
		// TODO don't send the ones that we send in "names"
		response->cat("]},\"extra\":[");
		bool first = true;
		unsigned int nextSensorNumber = 0;
		for (;;)
		{
			const auto sensor = heat->FindSensorAtOrAbove(nextSensorNumber);
			if (sensor.IsNull())
			{
				break;
			}
			const char * const nm = sensor->GetSensorName();
			if (nm != nullptr)
			{
				if (!first)
				{
					response->cat(',');
				}
				first = false;
				response->cat("{\"name\":");
				response->EncodeString(nm, false, true);
				float temp;
				(void)sensor->GetLatestTemperature(temp);
				response->catf(",\"temp\":%.1f}", HideNan(temp));
			}
			nextSensorNumber = sensor->GetSensorNumber() + 1;
		}

		response->cat("]}");
	}

	// Time since last reset
	response->catf(",\"time\":%.1f", (double)(millis64()/1000u));

#if SUPPORT_SCANNER
	// Scanner
	if (scanner->IsEnabled())
	{
		response->catf(",\"scanner\":{\"status\":\"%c\"", scanner->GetStatusCharacter());
		response->catf(",\"progress\":%.1f}", (double)(scanner->GetProgress() * 100.0));
	}
#endif

	// Spindles
	if (gCodes->GetMachineType() == MachineType::cnc || type == 2)
	{
		size_t numSpindles = MaxSpindles;
		while (numSpindles != 0 && platform->AccessSpindle(numSpindles - 1).GetToolNumber() == -1)
		{
			--numSpindles;
		}

		if (numSpindles != 0)
		{
			response->cat(",\"spindles\":[");
			for (size_t i = 0; i < numSpindles; i++)
			{
				if (i != 0)
				{
					response->cat(',');
				}

				const Spindle& spindle = platform->AccessSpindle(i);
				response->catf("{\"current\":%.1f,\"active\":%.1f", (double)spindle.GetCurrentRpm(), (double)spindle.GetRpm());
				if (type == 2)
				{
					response->catf(",\"tool\":%d}", spindle.GetToolNumber());
				}
				else
				{
					response->cat('}');
				}
			}
			response->cat(']');
		}
	}

#if SUPPORT_LASER
	if (gCodes->GetMachineType() == MachineType::laser)
	{
		response->catf(",\"laser\":%.1f", (double)(gCodes->GetLaserPwm() * 100.0));		// 2020-04-24: return the configured laser PWM even if the laser is temporarily turned off
	}
#endif

	/* Extended Status Response */
	if (type == 2)
	{
		// Cold Extrude/Retract
		response->catf(",\"coldExtrudeTemp\":%.1f", (double)(heat->ColdExtrude() ? 0.0f : heat->GetExtrusionMinTemp()));
		response->catf(",\"coldRetractTemp\":%.1f", (double)(heat->ColdExtrude() ? 0.0f : heat->GetRetractionMinTemp()));

		// Compensation type
		response->cat(",\"compensation\":");
		if (move->IsUsingMesh())
		{
			response->cat("\"Mesh\"");
		}
		else if (move->GetNumProbePoints() > 0)
		{
			response->catf("\"%u Point\"", move->GetNumProbePoints());
		}
		else
		{
			response->cat("\"None\"");
		}

		// Controllable Fans
		FansBitmap controllableFans;
		for (size_t fan = 0; fan < MaxFans; fan++)
		{
			if (fansManager->IsFanControllable(fan))
			{
				controllableFans.SetBit(fan);
			}
		}
		response->catf(",\"controllableFans\":%lu", controllableFans.GetRaw());

		// Maximum hotend temperature - DWC just wants the highest one
		response->catf(",\"tempLimit\":%.1f", (double)(heat->GetHighestTemperatureLimit()));

		// Endstops
		uint32_t endstops = 0;
		const size_t numTotalAxes = gCodes->GetTotalAxes();
		for (size_t axis = 0; axis < numTotalAxes; axis++)
		{
			if (platform->GetEndstops().Stopped(axis) == EndStopHit::atStop)
			{
				endstops |= (1u << axis);
			}
		}
		response->catf(",\"endstops\":%" PRIu32, endstops);

		// Firmware name, machine geometry and number of axes
		response->catf(",\"firmwareName\":\"%s\",\"firmwareVersion\":\"%s\",\"geometry\":\"%s\",\"axes\":%u,\"totalAxes\":%u,\"axisNames\":\"%s\"",
			FIRMWARE_NAME, VERSION, move->GetGeometryString(), numVisibleAxes, numTotalAxes, gCodes->GetAxisLetters());

#if HAS_MASS_STORAGE
		// Total and mounted volumes
		size_t mountedCards = 0;
		for (size_t i = 0; i < NumSdCards; i++)
		{
			if (MassStorage::IsDriveMounted(i))
			{
				mountedCards |= (1 << i);
			}
		}
		response->catf(",\"volumes\":%u,\"mountedVolumes\":%u", NumSdCards, mountedCards);
#endif

		// Machine mode,
		const char *machineMode = gCodes->GetMachineModeString();
		response->cat(",\"mode\":");
		response->EncodeString(machineMode, strlen(machineMode), false);

		// Machine name
		response->cat(",\"name\":");
		response->EncodeString(myName, false);

		/* Probe */
		{
			const auto zp = platform->GetZProbeOrDefault(0);

			// Trigger threshold, trigger height, type
			response->catf(",\"probe\":{\"threshold\":%d,\"height\":%.2f,\"type\":%u}",
							zp->GetAdcValue(), (double)zp->GetConfiguredTriggerHeight(), (unsigned int)zp->GetProbeType());
		}

		/* Tool Mapping */
		{
			response->cat(",\"tools\":[");
			ReadLocker lock(toolListLock);
			for (Tool *tool = toolList; tool != nullptr; tool = tool->Next())
			{
				// Number
				response->catf("{\"number\":%d,", tool->Number());

				// Name
				const char *toolName = tool->GetName();
				if (toolName[0] != 0)
				{
					response->cat("\"name\":");
					response->EncodeString(toolName, false);
					response->cat(',');
				}

				// Heaters
				AppendIntArray(response, "heaters", tool->HeaterCount(), [tool](size_t heater) noexcept { return tool->Heater(heater); });

				// Extruder drives
				response->cat(',');
				AppendIntArray(response, "drives", tool->DriveCount(), [tool](size_t drive) noexcept { return tool->Drive(drive); });

				// Axis mapping
				response->cat(",\"axisMap\":[[");
				tool->GetXAxisMap().Iterate
					([response](unsigned int xi, unsigned int count) noexcept
						{
							if (count != 0)
							{
								response->cat(',');
							}
							response->catf("%u", xi);
						}
					);
				response->cat("],[");

				tool->GetYAxisMap().Iterate
					([response](unsigned int yi, unsigned int count) noexcept
						{
							if (count != 0)
							{
								response->cat(',');
							}
							response->catf("%u", yi);
						}
					);
				response->cat("]]");

				// Fan mapping
				response->catf(",\"fans\":%lu", tool->GetFanMapping().GetRaw());

				// Filament (if any)
				if (tool->GetFilament() != nullptr)
				{
					const char *filamentName = tool->GetFilament()->GetName();
					response->catf(",\"filament\":");
					response->EncodeString(filamentName, false);
				}

				// Offsets
				response->cat(',');
				AppendFloatArray(response, "offsets", numVisibleAxes, [tool](size_t axis) noexcept { return tool->GetOffset(axis); }, 2);

  				// Do we have any more tools?
				response->cat((tool->Next() != nullptr) ? "}," : "}");
			}
			response->cat(']');
		}

		// MCU temperatures
#if HAS_CPU_TEMP_SENSOR
		{
			const MinMaxCurrent temps = platform->GetMcuTemperatures();
			response->catf(",\"mcutemp\":{\"min\":%.1f,\"cur\":%.1f,\"max\":%.1f}", (double)temps.min, (double)temps.current, (double)temps.max);
		}
#endif

#if HAS_VOLTAGE_MONITOR
		// Power in voltages
		{
			const MinMaxCurrent voltages = platform->GetPowerVoltages();
			response->catf(",\"vin\":{\"min\":%.1f,\"cur\":%.1f,\"max\":%.1f}", (double)voltages.min, (double)voltages.current, (double)voltages.max);
		}
#endif

#if HAS_12V_MONITOR
		// Power in voltages
		{
			const MinMaxCurrent voltages = platform->GetV12Voltages();
			response->catf(",\"v12\":{\"min\":%.1f,\"cur\":%.1f,\"max\":%.1f}", (double)voltages.min, (double)voltages.current, (double)voltages.max);
		}
#endif
	}
	else if (type == 3)
	{
		// Current Layer
		response->catf(",\"currentLayer\":%d", printMonitor->GetCurrentLayer());

		// Current Layer Time
		response->catf(",\"currentLayerTime\":%.1f,", (double)(printMonitor->GetCurrentLayerTime()));

		// Raw Extruder Positions
		AppendFloatArray(response, "extrRaw", GetExtrudersInUse(), [this](size_t extruder) noexcept { return gCodes->GetRawExtruderTotalByDrive(extruder); }, 1);

		// Fraction of file printed
		response->catf(",\"fractionPrinted\":%.1f", (double)((printMonitor->IsPrinting()) ? (printMonitor->FractionOfFilePrinted() * 100.0) : 0.0));

		// Byte position of the file being printed
		response->catf(",\"filePosition\":%lu", gCodes->GetFilePosition());

		// First Layer Duration
		response->catf(",\"firstLayerDuration\":%.1f", (double)(printMonitor->GetFirstLayerDuration()));

		// First Layer Height
		// NB: This shouldn't be needed any more, but leave it here for the case that the file-based first-layer detection fails
		response->catf(",\"firstLayerHeight\":%.2f", (double)(printMonitor->GetFirstLayerHeight()));

		// Print Duration
		response->catf(",\"printDuration\":%.1f", (double)(printMonitor->GetPrintDuration()));

		// Warm-Up Time
		response->catf(",\"warmUpDuration\":%.1f", (double)(printMonitor->GetWarmUpDuration()));

		/* Print Time Estimations */
		{
			// Based on file progress
			response->catf(",\"timesLeft\":{\"file\":%.1f,\"filament\":%.1f,\"layer\":%.1f}",
							(double)(printMonitor->EstimateTimeLeft(fileBased)),
							(double)(printMonitor->EstimateTimeLeft(filamentBased)),
							(double)(printMonitor->EstimateTimeLeft(layerBased)));
		}
	}

	response->cat('}');
	return response;
}

OutputBuffer *RepRap::GetConfigResponse() noexcept
{
	// We need some resources to return a valid config response...
	OutputBuffer *response;
	if (!OutputBuffer::Allocate(response))
	{
		return nullptr;
	}

	const size_t numAxes = gCodes->GetVisibleAxes();

	// Axis minima
	response->copy('{');
	AppendFloatArray(response, "axisMins", numAxes, [this](size_t axis) noexcept { return platform->AxisMinimum(axis); }, 2);

	// Axis maxima
	response->cat(',');
	AppendFloatArray(response, "axisMaxes", numAxes, [this](size_t axis) noexcept { return platform->AxisMaximum(axis); }, 2);

	// Accelerations
	response->cat(',');
	AppendFloatArray(response, "accelerations", MaxAxesPlusExtruders, [this](size_t drive) noexcept { return platform->Acceleration(drive); }, 2);

	// Motor currents
	response->cat(',');
	AppendIntArray(response, "currents", MaxAxesPlusExtruders, [this](size_t drive) noexcept { return (int)platform->GetMotorCurrent(drive, 906); });

	// Firmware details
	response->catf(",\"firmwareElectronics\":\"%s", platform->GetElectronicsString());
#ifdef DUET_NG
	const char* expansionName = DuetExpansion::GetExpansionBoardName();
	if (expansionName != nullptr)
	{
		response->catf(" + %s", expansionName);
	}
	const char* additionalExpansionName = DuetExpansion::GetAdditionalExpansionBoardName();
	if (additionalExpansionName != nullptr)
	{
		response->catf(" + %s", additionalExpansionName);
	}
#endif
	response->cat("\",\"firmwareName\":");
	response->EncodeString(FIRMWARE_NAME, false);
#ifdef BOARD_SHORT_NAME
	response->cat(",\"boardName\":");
	response->EncodeString(BOARD_SHORT_NAME, false);
#endif
	response->cat(",\"firmwareVersion\":");
	response->EncodeString(VERSION, false);

#if HAS_WIFI_NETWORKING
	// If we have WiFi networking, send the WiFi module firmware version
# ifdef DUET_NG
	if (platform->IsDuetWiFi())
	{
# endif
		response->catf(",\"dwsVersion\":\"%s\"", network->GetWiFiServerVersion());
# ifdef DUET_NG
	}
# endif
#endif

	response->catf(",\"firmwareDate\":\"%s\"", DATE);

#if HAS_MASS_STORAGE
	// System files folder
	response->catf(", \"sysdir\":");
	platform->EncodeSysDir(response);
#endif

	// Motor idle parameters
	response->catf(",\"idleCurrentFactor\":%.1f", (double)(platform->GetIdleCurrentFactor() * 100.0));
	response->catf(",\"idleTimeout\":%.1f,", (double)(move->IdleTimeout()));

	// Minimum feedrates
	AppendFloatArray(response, "minFeedrates", MaxAxesPlusExtruders, [this](size_t drive) noexcept { return platform->GetInstantDv(drive); }, 2);

	// Maximum feedrates
	response->cat(',');
	AppendFloatArray(response, "maxFeedrates", MaxAxesPlusExtruders, [this](size_t drive) noexcept { return platform->MaxFeedrate(drive); }, 2);

	// Config file is no longer included, because we can use rr_configfile or M503 instead
	response->cat('}');

	return response;
}

// Get the JSON status response for PanelDue or the old web server.
// Type 0 was the old-style webserver status response, but is no longer supported.
// Type 1 is the new-style webserver status response.
// Type 2 is the M105 S2 response, which is like the new-style status response but some fields are omitted.
// Type 3 is the M105 S3 response, which is like the M105 S2 response except that static values are also included.
// 'seq' is the response sequence number, if it is not -1 and we have a different sequence number then we send the gcode response
OutputBuffer *RepRap::GetLegacyStatusResponse(uint8_t type, int seq) const noexcept
{
	// Need something to write to...
	OutputBuffer *response;
	if (!OutputBuffer::Allocate(response))
	{
		// Should never happen
		return nullptr;
	}

	// Send the status. Note that 'S' has always meant that the machine is halted in this version of the status response, so we use A for pAused.
	char ch = GetStatusCharacter();
	if (ch == 'S')			// if paused then send 'A'
	{
		ch = 'A';
	}
	else if (ch == 'H')		// if halted then send 'S'
	{
		ch = 'S';
	}
	response->printf("{\"status\":\"%c\",\"heaters\":", ch);

	// Send the heater actual temperatures. If there is no bed heater, send zero for PanelDue.
	const int8_t bedHeater = (MaxBedHeaters > 0) ? heat->GetBedHeater(0) : -1;
	ch = ',';
	response->catf("[%.1f", (double)((bedHeater == -1) ? 0.0 : heat->GetHeaterTemperature(bedHeater)));
	for (size_t heater = DefaultE0Heater; heater < GetToolHeatersInUse(); heater++)
	{
		response->catf("%c%.1f", ch, (double)(heat->GetHeaterTemperature(heater)));
		ch = ',';
	}
	response->cat((ch == '[') ? "[]" : "]");

	// Send the heater active temperatures
	response->catf(",\"active\":[%.1f", (double)((bedHeater == -1) ? 0.0 : heat->GetActiveTemperature(bedHeater)));
	for (size_t heater = DefaultE0Heater; heater < GetToolHeatersInUse(); heater++)
	{
		response->catf(",%.1f", (double)(heat->GetActiveTemperature(heater)));
	}
	response->cat(']');

	// Send the heater standby temperatures
	response->catf(",\"standby\":[%.1f", (double)((bedHeater == -1) ? 0.0 : heat->GetStandbyTemperature(bedHeater)));
	for (size_t heater = DefaultE0Heater; heater < GetToolHeatersInUse(); heater++)
	{
		response->catf(",%.1f", (double)(heat->GetStandbyTemperature(heater)));
	}
	response->cat(']');

	// Send the heater statuses (0=off, 1=standby, 2=active, 3 = fault)
	response->catf(",\"hstat\":[%u", (bedHeater == -1) ? 0 : heat->GetStatus(bedHeater).ToBaseType());
	for (size_t heater = DefaultE0Heater; heater < GetToolHeatersInUse(); heater++)
	{
		response->catf(",%u", heat->GetStatus(heater).ToBaseType());
	}
	response->cat("],");

	// User coordinates
	const size_t numVisibleAxes = gCodes->GetVisibleAxes();
	AppendFloatArray(response, "pos", numVisibleAxes, [this](size_t axis) noexcept { return gCodes->GetUserCoordinate(axis); }, 3);

	// Machine coordinates
	response->cat(',');
	AppendFloatArray(response, "machine", numVisibleAxes, [this](size_t axis) noexcept { return move->LiveCoordinate(axis, currentTool); }, 3);

	// Send the speed and extruder override factors
	response->catf(",\"sfactor\":%.1f,", (double)(gCodes->GetSpeedFactor() * 100.0));
	AppendFloatArray(response, "efactor", GetExtrudersInUse(), [this](size_t extruder) noexcept { return gCodes->GetExtrusionFactor(extruder) * 100.0; }, 1);

	// Send the baby stepping offset
	response->catf(",\"babystep\":%.03f", (double)(gCodes->GetTotalBabyStepOffset(Z_AXIS)));

	// Send the current tool number
	response->catf(",\"tool\":%d", GetCurrentToolNumber());

	// Send the Z probe value
	const auto zp = platform->GetZProbeOrDefault(0);
	const int v0 = zp->GetReading();
	int v1;
	switch (zp->GetSecondaryValues(v1))
	{
	case 1:
		response->catf(",\"probe\":\"%d (%d)\"", v0, v1);
		break;
	default:
		response->catf(",\"probe\":\"%d\"", v0);
		break;
	}

	// Send the fan settings, for PanelDue firmware 1.13 and later
	// Currently, PanelDue assumes that the first value is the print cooling fan speed and only uses that one, so send the mapped fan speed first
	response->catf(",\"fanPercent\":[%.1f", (double)(gCodes->GetMappedFanSpeed() * 100.0));
	for (size_t i = 0; i < MaxFans; ++i)
	{
		const float fanValue = fansManager->GetFanValue(i);
		response->catf(",%d", (fanValue < 0.0) ? -1 : (int)lrintf(fanValue * 100.0));
	}
	response->cat("],");

	// Send fan RPM value(s)
	AppendIntArray(response, "fanRPM", fansManager->GetNumFansToReport(), [this](size_t fan) { return (int)fansManager->GetFanRPM(fan);});

	// Send the home state. To keep the messages short, we send 1 for homed and 0 for not homed, instead of true and false.
	response->cat(',');
	AppendIntArray(response, "homed", numVisibleAxes, [this](size_t axis) noexcept { return (gCodes->IsAxisHomed(axis)) ? 1 : 0; });

	if (printMonitor->IsPrinting())
	{
		// Send the fraction printed
		response->catf(",\"fraction_printed\":%.4f", (double)max<float>(0.0, printMonitor->FractionOfFilePrinted()));
	}

	// Short messages are now pushed directly to PanelDue, so don't include them here as well
	// We no longer send the amount of http buffer space here because the web interface doesn't use these forms of status response

	// Deal with the message box.
	// Don't send it if we are flashing firmware, because when we flash firmware we send messages directly to PanelDue and we don't want them to get cleared.
	if (!gCodes->IsFlashing())
	{
		float timeLeft = 0.0;
		MutexLocker lock(messageBoxMutex);

		if (mbox.active && mbox.timer != 0)
		{
			timeLeft = (float)(mbox.timeout) / 1000.0 - (float)(millis() - mbox.timer) / 1000.0;
		}

		if (mbox.active)
		{
			response->catf(",\"msgBox.mode\":%d,\"msgBox.seq\":%" PRIu32 ",\"msgBox.timeout\":%.1f,\"msgBox.controls\":%u",
							mbox.mode, mbox.seq, (double)timeLeft, mbox.controls.GetRaw());
			response->cat(",\"msgBox.msg\":");
			response->EncodeString(mbox.message, false);
			response->cat(",\"msgBox.title\":");
			response->EncodeString(mbox.title, false);
		}
		else
		{
			response->cat(",\"msgBox.mode\":-1");					// tell PanelDue that there is no active message box
		}
	}

	if (type == 2)
	{
		if (printMonitor->IsPrinting())
		{
			// Send estimated times left based on file progress, filament usage, and layers
			response->catf(",\"timesLeft\":[%.1f,%.1f,%.1f]",
					(double)(printMonitor->EstimateTimeLeft(fileBased)),
					(double)(printMonitor->EstimateTimeLeft(filamentBased)),
					(double)(printMonitor->EstimateTimeLeft(layerBased)));
		}
	}
	else if (type == 3)
	{
		// Add the static fields
		response->catf(",\"geometry\":\"%s\",\"axes\":%u,\"totalAxes\":%u,\"axisNames\":\"%s\",\"volumes\":%u,\"numTools\":%u,\"myName\":",
						move->GetGeometryString(), numVisibleAxes, gCodes->GetTotalAxes(), gCodes->GetAxisLetters(), NumSdCards, GetNumberOfContiguousTools());
		response->EncodeString(myName, false);
		response->cat(",\"firmwareName\":");
		response->EncodeString(FIRMWARE_NAME, false);
	}

	response->cat('}');
	return response;
}

#if HAS_MASS_STORAGE

// Get the list of files in the specified directory in JSON format.
// If flagDirs is true then we prefix each directory with a * character.
OutputBuffer *RepRap::GetFilesResponse(const char *dir, unsigned int startAt, bool flagsDirs) noexcept
{
	// Need something to write to...
	OutputBuffer *response;
	if (!OutputBuffer::Allocate(response))
	{
		return nullptr;
	}

	response->copy("{\"dir\":");
	response->EncodeString(dir, false);
	response->catf(",\"first\":%u,\"files\":[", startAt);
	unsigned int err;
	unsigned int nextFile = 0;

	if (!MassStorage::CheckDriveMounted(dir))
	{
		err = 1;
	}
	else if (!MassStorage::DirectoryExists(dir))
	{
		err = 2;
	}
	else
	{
		err = 0;
		FileInfo fileInfo;
		unsigned int filesFound = 0;
		bool gotFile = MassStorage::FindFirst(dir, fileInfo);

		size_t bytesLeft = OutputBuffer::GetBytesLeft(response);	// don't write more bytes than we can

		while (gotFile)
		{
			if (fileInfo.fileName[0] != '.')						// ignore Mac resource files and Linux hidden files
			{
				if (filesFound >= startAt)
				{
					// Make sure we can end this response properly
					if (bytesLeft < fileInfo.fileName.strlen() * 2 + 20)
					{
						// No more space available - stop here
						MassStorage::AbandonFindNext();
						nextFile = filesFound;
						break;
					}

					// Write separator and filename
					if (filesFound != startAt)
					{
						bytesLeft -= response->cat(',');
					}

					bytesLeft -= response->EncodeString(fileInfo.fileName, false, flagsDirs && fileInfo.isDirectory);
				}
				++filesFound;
			}
			gotFile = MassStorage::FindNext(fileInfo);
		}
	}

	if (err != 0)
	{
		response->catf("],\"err\":%u}", err);
	}
	else
	{
		response->catf("],\"next\":%u,\"err\":%u}", nextFile, err);
	}
	return response;
}

// Get a JSON-style filelist including file types and sizes
OutputBuffer *RepRap::GetFilelistResponse(const char *dir, unsigned int startAt) noexcept
{
	// Need something to write to...
	OutputBuffer *response;
	if (!OutputBuffer::Allocate(response))
	{
		return nullptr;
	}

	response->copy("{\"dir\":");
	response->EncodeString(dir, false);
	response->catf(",\"first\":%u,\"files\":[", startAt);
	unsigned int err;
	unsigned int nextFile = 0;

	if (!MassStorage::CheckDriveMounted(dir))
	{
		err = 1;
	}
	else if (!MassStorage::DirectoryExists(dir))
	{
		err = 2;
	}
	else
	{
		err = 0;
		FileInfo fileInfo;
		unsigned int filesFound = 0;
		bool gotFile = MassStorage::FindFirst(dir, fileInfo);
		size_t bytesLeft = OutputBuffer::GetBytesLeft(response);	// don't write more bytes than we can

		while (gotFile)
		{
			if (fileInfo.fileName[0] != '.')			// ignore Mac resource files and Linux hidden files
			{
				if (filesFound >= startAt)
				{
					// Make sure we can end this response properly
					if (bytesLeft < fileInfo.fileName.strlen() * 2 + 50)
					{
						// No more space available - stop here
						MassStorage::AbandonFindNext();
						nextFile = filesFound;
						break;
					}

					// Write delimiter
					if (filesFound != startAt)
					{
						bytesLeft -= response->cat(',');
					}

					// Write another file entry
					bytesLeft -= response->catf("{\"type\":\"%c\",\"name\":", fileInfo.isDirectory ? 'd' : 'f');
					bytesLeft -= response->EncodeString(fileInfo.fileName, false);
					bytesLeft -= response->catf(",\"size\":%" PRIu32, fileInfo.size);

					tm timeInfo;
					gmtime_r(&fileInfo.lastModified, &timeInfo);
					if (timeInfo.tm_year <= /*19*/80)
					{
						// Don't send the last modified date if it is invalid
						bytesLeft -= response->cat('}');
					}
					else
					{
						bytesLeft -= response->catf(",\"date\":\"%04u-%02u-%02uT%02u:%02u:%02u\"}",
								timeInfo.tm_year + 1900, timeInfo.tm_mon + 1, timeInfo.tm_mday, timeInfo.tm_hour, timeInfo.tm_min, timeInfo.tm_sec);
					}
				}
				++filesFound;
			}
			gotFile = MassStorage::FindNext(fileInfo);
		}
	}

	// If there is no error, don't append "err":0 because if we do then DWC thinks there has been an error - looks like it doesn't check the value
	if (err != 0)
	{
		response->catf("],\"err\":%u}", err);
	}
	else
	{
		response->catf("],\"next\":%u}", nextFile);
	}

	return response;
}

#endif

// Get information for the specified file, or the currently printing file (if 'filename' is null or empty), in JSON format
bool RepRap::GetFileInfoResponse(const char *filename, OutputBuffer *&response, bool quitEarly) noexcept
{
	const bool specificFile = (filename != nullptr && filename[0] != 0);
	GCodeFileInfo info;
	if (specificFile)
	{
#if HAS_MASS_STORAGE
		// Poll file info for a specific file
		String<MaxFilenameLength> filePath;
		if (!MassStorage::CombineName(filePath.GetRef(), platform->GetGCodeDir(), filename))
		{
			info.isValid = false;
		}
		else if (!MassStorage::GetFileInfo(filePath.c_str(), info, quitEarly))
		{
			// This may take a few runs...
			return false;
		}
#else
		return false;
#endif
	}
	else if (!printMonitor->GetPrintingFileInfo(info))
	{
		return false;
	}

	if (!OutputBuffer::Allocate(response))
	{
		return false;
	}

	if (info.isValid)
	{
		response->printf("{\"err\":0,\"size\":%lu,",info.fileSize);
		tm timeInfo;
		gmtime_r(&info.lastModifiedTime, &timeInfo);
		if (timeInfo.tm_year > /*19*/80)
		{
			response->catf("\"lastModified\":\"%04u-%02u-%02uT%02u:%02u:%02u\",",
					timeInfo.tm_year + 1900, timeInfo.tm_mon + 1, timeInfo.tm_mday, timeInfo.tm_hour, timeInfo.tm_min, timeInfo.tm_sec);
		}

		response->catf("\"height\":%.2f,\"firstLayerHeight\":%.2f,\"layerHeight\":%.2f,",
					(double)info.objectHeight, (double)info.firstLayerHeight, (double)info.layerHeight);
		if (info.printTime != 0)
		{
			response->catf("\"printTime\":%" PRIu32 ",", info.printTime);
		}
		if (info.simulatedTime != 0)
		{
			response->catf("\"simulatedTime\":%" PRIu32 ",", info.simulatedTime);
		}

		response->cat("\"filament\":");
		char ch = '[';
		if (info.numFilaments == 0)
		{
			response->cat(ch);
		}
		else
		{
			for (size_t i = 0; i < info.numFilaments; ++i)
			{
				response->catf("%c%.1f", ch, (double)info.filamentNeeded[i]);
				ch = ',';
			}
		}
		response->cat("]");

		if (!specificFile)
		{
			response->catf(",\"printDuration\":%d,\"fileName\":", (int)printMonitor->GetPrintDuration());
			response->EncodeString(printMonitor->GetPrintingFilename(), false);
		}

		response->cat(",\"generatedBy\":");
		response->EncodeString(info.generatedBy, false);
		response->cat('}');
	}
	else
	{
		response->copy("{\"err\":1}");
	}
	return true;
}

// Helper functions to write JSON arrays
// Append float array using 1 decimal place
void RepRap::AppendFloatArray(OutputBuffer *buf, const char *name, size_t numValues, std::function<float(size_t)> func, unsigned int numDecimalDigits) noexcept
{
	if (name != nullptr)
	{
		buf->catf("\"%s\":", name);
	}
	buf->cat('[');
	for (size_t i = 0; i < numValues; ++i)
	{
		if (i != 0)
		{
			buf->cat(',');
		}
		buf->catf(GetFloatFormatString(numDecimalDigits), HideNan(func(i)));
	}
	buf->cat(']');
}

void RepRap::AppendIntArray(OutputBuffer *buf, const char *name, size_t numValues, std::function<int(size_t)> func) noexcept
{
	if (name != nullptr)
	{
		buf->catf("\"%s\":", name);
	}
	buf->cat('[');
	for (size_t i = 0; i < numValues; ++i)
	{
		if (i != 0)
		{
			buf->cat(',');
		}
		buf->catf("%d", func(i));
	}
	buf->cat(']');
}

void RepRap::AppendStringArray(OutputBuffer *buf, const char *name, size_t numValues, std::function<const char *(size_t)> func) noexcept
{
	if (name != nullptr)
	{
		buf->catf("\"%s\":", name);
	}
	buf->cat('[');
	for (size_t i = 0; i < numValues; ++i)
	{
		if (i != 0)
		{
			buf->cat(',');
		}
		buf->EncodeString(func(i), true);
	}
	buf->cat(']');
}

#if SUPPORT_OBJECT_MODEL

// Return a query into the object model, or return nullptr if no buffer available
OutputBuffer *RepRap::GetModelResponse(const char *key, const char *flags) const THROWS(GCodeException)
{
	OutputBuffer *outBuf;
	if (OutputBuffer::Allocate(outBuf))
	{
		if (key == nullptr) { key = ""; }
		if (flags == nullptr) { flags = ""; }

		outBuf->printf("{\"key\":");
		outBuf->EncodeString(key, false);
		outBuf->catf(",\"flags\":");
		outBuf->EncodeString(flags, false);

		const bool wantArrayLength = (*key == '#');
		if (wantArrayLength)
		{
			++key;
		}

		outBuf->cat(",\"result\":");
		reprap.ReportAsJson(outBuf, key, flags, wantArrayLength);
		outBuf->cat('}');
	}

	return outBuf;
}

#endif

// Send a beep. We send it to both PanelDue and the web interface.
void RepRap::Beep(unsigned int freq, unsigned int ms) noexcept
{
	// Limit the frequency and duration to sensible values
	freq = constrain<unsigned int>(freq, 50, 10000);
	ms = constrain<unsigned int>(ms, 10, 60000);

	// If there is an LCD device present, make it beep
	bool bleeped = false;
#if SUPPORT_12864_LCD
	if (display->IsPresent())
	{
		display->Beep(freq, ms);
		bleeped = true;
	}
#endif

	if (platform->IsAuxEnabled())
	{
		platform->Beep(freq, ms);
		bleeped = true;
	}

	if (!bleeped)
	{
		beepFrequency = freq;
		beepDuration = ms;
		beepTimer = millis();
		StateUpdated();
	}
}

// Send a short message. We send it to both PanelDue and the web interface.
void RepRap::SetMessage(const char *msg) noexcept
{
	message.copy(msg);
#if SUPPORT_12864_LCD
	++messageSequence;
#endif
	StateUpdated();

	if (platform->IsAuxEnabled())
	{
		platform->SendAuxMessage(msg);
	}
}

// Display a message box on the web interface
void RepRap::SetAlert(const char *msg, const char *title, int mode, float timeout, AxesBitmap controls) noexcept
{
	MutexLocker lock(messageBoxMutex);
	mbox.message.copy(msg);
	mbox.title.copy(title);
	mbox.mode = mode;
	mbox.timer = (timeout <= 0.0) ? 0 : millis();
	mbox.timeout = round(max<float>(timeout, 0.0) * 1000.0);
	mbox.controls = controls;
	mbox.active = true;
	++mbox.seq;
	StateUpdated();
}

// Clear pending message box
void RepRap::ClearAlert() noexcept
{
	MutexLocker lock(messageBoxMutex);
	mbox.active = false;
	StateUpdated();
}

// Get the status index
size_t RepRap::GetStatusIndex() const noexcept
{
	return    (processingConfig)										? 0		// Reading the configuration file
#if HAS_LINUX_INTERFACE && SUPPORT_CAN_EXPANSION
			: (gCodes->IsFlashing() || expansion->IsFlashing())			? 1		// Flashing a new firmware binary
#else
			: (gCodes->IsFlashing())									? 1		// Flashing a new firmware binary
#endif
			: (IsStopped()) 											? 2		// Halted
#if HAS_VOLTAGE_MONITOR
			: (!platform->HasVinPower() && !gCodes->IsSimulating())		? 3		// Off i.e. powered down
#endif
			: (gCodes->IsPausing()) 									? 4		// Pausing / Decelerating
			: (gCodes->IsResuming()) 									? 5		// Resuming
			: (gCodes->IsPaused()) 										? 6		// Paused / Stopped
			: (printMonitor->IsPrinting() && gCodes->IsSimulating())	? 7		// Simulating
			: (printMonitor->IsPrinting())							  	? 8		// Printing
			: (gCodes->IsDoingToolChange())								? 9		// Changing tool
			: (gCodes->DoingFileMacro() || !move->NoLiveMovement() ||
			   gCodes->WaitingForAcknowledgement()) 					? 10	// Busy
			:															  11;	// Idle

}

// Get the status character for the new-style status response
char RepRap::GetStatusCharacter() const noexcept
{
	return "CFHODRSMPTBI"[GetStatusIndex()];
}

const char* RepRap::GetStatusString() const noexcept
{
	static const char *const StatusStrings[] =
	{
		"starting",
		"updating",
		"halted",
		"off",
		"pausing",
		"resuming",
		"paused",
		"simulating",
		"processing",
		"changingTool",
		"busy",
		"idle"
	};
	return StatusStrings[GetStatusIndex()];
}

bool RepRap::NoPasswordSet() const noexcept
{
	return (password[0] == 0 || CheckPassword(DEFAULT_PASSWORD));
}

bool RepRap::CheckPassword(const char *pw) const noexcept
{
	String<RepRapPasswordLength> copiedPassword;
	copiedPassword.CopyAndPad(pw);
	return password.ConstantTimeEquals(copiedPassword);
}

void RepRap::SetPassword(const char* pw) noexcept
{
	password.CopyAndPad(pw);
}

const char *RepRap::GetName() const noexcept
{
	return myName.c_str();
}

void RepRap::SetName(const char* nm) noexcept
{
	myName.copy(nm);

	// Set new DHCP hostname
	network->SetHostname(myName.c_str());
	NetworkUpdated();
}

// Given that we want to extrude/retract the specified extruder drives, check if they are allowed.
// For each disallowed one, log an error to report later and return a bit in the bitmap.
// This may be called by an ISR!
unsigned int RepRap::GetProhibitedExtruderMovements(unsigned int extrusions, unsigned int retractions) noexcept
{
	if (GetHeat().ColdExtrude())
	{
		return 0;
	}

	Tool * const tool = currentTool;
	if (tool == nullptr)
	{
		// This should not happen, but if no tool is selected then don't allow any extruder movement
		return extrusions | retractions;
	}

	unsigned int result = 0;
	for (size_t driveNum = 0; driveNum < tool->DriveCount(); driveNum++)
	{
		const unsigned int extruderDrive = (unsigned int)(tool->Drive(driveNum));
		const unsigned int mask = 1 << extruderDrive;
		if (extrusions & mask)
		{
			if (!tool->ToolCanDrive(true))
			{
				result |= mask;
			}
		}
		else if (retractions & mask)
		{
			if (!tool->ToolCanDrive(false))
			{
				result |= mask;
			}
		}
	}

	return result;
}

void RepRap::FlagTemperatureFault(int8_t dudHeater) noexcept
{
	ReadLocker lock(toolListLock);
	if (toolList != nullptr)
	{
		toolList->FlagTemperatureFault(dudHeater);
	}
}

GCodeResult RepRap::ClearTemperatureFault(int8_t wasDudHeater, const StringRef& reply) noexcept
{
	const GCodeResult rslt = heat->ResetFault(wasDudHeater, reply);
	ReadLocker lock(toolListLock);
	if (toolList != nullptr)
	{
		toolList->ClearTemperatureFault(wasDudHeater);
	}
	return rslt;
}

#if HAS_MASS_STORAGE

// Save some resume information, returning true if successful
// We assume that the tool configuration doesn't change, only the temperatures and the mix
bool RepRap::WriteToolSettings(FileStore *f) noexcept
{
	// First write the settings of all tools except the current one and the command to select them if they are on standby
	bool ok = true;
	ReadLocker lock(toolListLock);
	for (const Tool *t = toolList; t != nullptr && ok; t = t->Next())
	{
		if (t != currentTool)
		{
			ok = t->WriteSettings(f);
		}
	}

	// Finally write the settings of the active tool and the commands to select it. If no current tool, just deselect all tools.
	if (ok)
	{
		if (currentTool == nullptr)
		{
			ok = f->Write("T-1 P0\n");
		}
		else
		{
			ok = currentTool->WriteSettings(f);
			if (ok)
			{
				String<StringLength20> buf;
				buf.printf("T%u P0\n", currentTool->Number());
				ok = f->Write(buf.c_str());
			}
		}
	}
	return ok;
}

// Save some information in config-override.g
bool RepRap::WriteToolParameters(FileStore *f, const bool forceWriteOffsets) noexcept
{
	bool ok = true, written = false;
	ReadLocker lock(toolListLock);
	for (const Tool *t = toolList; ok && t != nullptr; t = t->Next())
	{
		const AxesBitmap axesProbed = t->GetAxisOffsetsProbed();
		if (axesProbed.IsNonEmpty() || forceWriteOffsets)
		{
			String<StringLength256> scratchString;
			if (!written)
			{
				scratchString.copy("; Probed tool offsets\n");
				written = true;
			}
			scratchString.catf("G10 P%d", t->Number());
			for (size_t axis = 0; axis < MaxAxes; ++axis)
			{
				if (forceWriteOffsets || axesProbed.IsBitSet(axis))
				{
					scratchString.catf(" %c%.2f", gCodes->GetAxisLetters()[axis], (double)(t->GetOffset(axis)));
				}
			}
			scratchString.cat('\n');
			ok = f->Write(scratchString.c_str());
		}
	}
	return ok;
}

#endif

// Firmware update operations

#ifdef __LPC17xx__
    #include "LPC/FirmwareUpdate.hpp"
#else

// Check the prerequisites for updating the main firmware. Return True if satisfied, else print a message to 'reply' and return false.
bool RepRap::CheckFirmwareUpdatePrerequisites(const StringRef& reply) noexcept
{
#if HAS_MASS_STORAGE
	FileStore * const firmwareFile = platform->OpenFile(DEFAULT_SYS_DIR, IAP_FIRMWARE_FILE, OpenMode::read);
	if (firmwareFile == nullptr)
	{
		reply.printf("Firmware binary \"%s\" not found", IAP_FIRMWARE_FILE);
		return false;
	}

	// Check that the binary looks sensible. The first word is the initial stack pointer, which should be the top of RAM.
	uint32_t firstDword;
	bool ok = firmwareFile->Read(reinterpret_cast<char*>(&firstDword), sizeof(firstDword)) == (int)sizeof(firstDword);
	firmwareFile->Close();
	if (!ok || firstDword !=
#if SAM3XA
						IRAM1_ADDR + IRAM1_SIZE
#else
						IRAM_ADDR + IRAM_SIZE
#endif
			)
	{
		reply.printf("Firmware binary \"%s\" is not valid for this electronics", IAP_FIRMWARE_FILE);
		return false;
	}

	if (!platform->FileExists(DEFAULT_SYS_DIR, IAP_UPDATE_FILE))
	{
		reply.printf("In-application programming binary \"%s\" not found", IAP_UPDATE_FILE);
		return false;
	}
#endif

	return true;
}

// Update the firmware. Prerequisites should be checked before calling this.
void RepRap::UpdateFirmware() noexcept
{
#if HAS_MASS_STORAGE
	FileStore * const iapFile = platform->OpenFile(DEFAULT_SYS_DIR, IAP_UPDATE_FILE, OpenMode::read);
	if (iapFile == nullptr)
	{
		platform->MessageF(FirmwareUpdateMessage, "IAP file '" IAP_UPDATE_FILE "' not found\n");
		return;
	}

	PrepareToLoadIap();

	// Use RAM-based IAP
	iapFile->Read(reinterpret_cast<char *>(IAP_IMAGE_START), iapFile->Length());
	iapFile->Close();
	StartIap();
#endif
}

void RepRap::PrepareToLoadIap() noexcept
{
#if SUPPORT_12864_LCD
	display->UpdatingFirmware();			// put the firmware update message on the display
#endif

	// Send this message before we start using RAM that may contain message buffers
	platform->Message(AuxMessage, "Updating main firmware\n");
	platform->Message(UsbMessage, "Shutting down USB interface to update main firmware. Try reconnecting after 30 seconds.\n");

	// Allow time for the firmware update message to be sent
	const uint32_t now = millis();
	while (platform->FlushMessages() && millis() - now < 2000) { }

	// The machine will be unresponsive for a few seconds, don't risk damaging the heaters.
	// This also shuts down tasks and interrupts that might make use of the RAM that we are about to load the IAP binary into.
	EmergencyStop();						// this also stops Platform::Tick being called, which is necessary because it access Z probe object in RAM used by IAP
	network->Exit();						// kill the network task to stop it overwriting RAM that we use to hold the IAP

	// Disable the cache because it interferes with flash memory access
	Cache::Disable();

#if USE_MPU
	//TODO consider setting flash memory to strongly-ordered instead
	ARM_MPU_Disable();
#endif
}

void RepRap::StartIap() noexcept
{
	// Disable all interrupts, then reallocate the vector table and program entry point to the new IAP binary
	// This does essentially what the Atmel AT02333 paper suggests (see 3.2.2 ff)

	// Disable all IRQs
	SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk;	// disable the system tick exception
	cpu_irq_disable();
	for (size_t i = 0; i < 8; i++)
	{
		NVIC->ICER[i] = 0xFFFFFFFF;					// Disable IRQs
		NVIC->ICPR[i] = 0xFFFFFFFF;					// Clear pending IRQs
	}

	// Disable all PIO IRQs, because the core assumes they are all disabled when setting them up
	PIOA->PIO_IDR = 0xFFFFFFFF;
	PIOB->PIO_IDR = 0xFFFFFFFF;
	PIOC->PIO_IDR = 0xFFFFFFFF;
#ifdef PIOD
	PIOD->PIO_IDR = 0xFFFFFFFF;
#endif
#ifdef ID_PIOE
	PIOE->PIO_IDR = 0xFFFFFFFF;
#endif

#if HAS_MASS_STORAGE
	// Newer versions of iap4e.bin reserve space above the stack for us to pass the firmware filename
	static const char filename[] = DEFAULT_SYS_DIR IAP_FIRMWARE_FILE;
	const uint32_t topOfStack = *reinterpret_cast<uint32_t *>(IAP_IMAGE_START);
	if (topOfStack + sizeof(filename) <=
# if SAM3XA
						IRAM1_ADDR + IRAM1_SIZE
# else
						IRAM_ADDR + IRAM_SIZE
# endif
	   )
	{
		memcpy(reinterpret_cast<char*>(topOfStack), filename, sizeof(filename));
	}
#endif

#if defined(DUET_NG) || defined(DUET_M)
	IoPort::WriteDigital(DiagPin, false);			// turn the DIAG LED off
#endif

	wdt_restart(WDT);								// kick the watchdog one last time

#if SAM4E || SAME70
	rswdt_restart(RSWDT);							// kick the secondary watchdog
#endif

	// Modify vector table location
	__DSB();
	__ISB();
	SCB->VTOR = ((uint32_t)IAP_IMAGE_START & SCB_VTOR_TBLOFF_Msk);
	__DSB();
	__ISB();

	cpu_irq_enable();

	__asm volatile ("mov r3, %0" : : "r" (IAP_IMAGE_START) : "r3");

	// We are using separate process and handler stacks. Put the process stack 1K bytes below the handler stack.
	__asm volatile ("ldr r1, [r3]");
	__asm volatile ("msr msp, r1");
	__asm volatile ("sub r1, #1024");
	__asm volatile ("mov sp, r1");

	__asm volatile ("isb");
	__asm volatile ("ldr r1, [r3, #4]");
	__asm volatile ("orr r1, r1, #1");
	__asm volatile ("bx r1");
}

#endif

// Helper function for diagnostic tests in Platform.cpp, to cause a deliberate divide-by-zero
/*static*/ uint32_t RepRap::DoDivide(uint32_t a, uint32_t b) noexcept
{
	return a/b;
}

// Helper function for diagnostic tests in Platform.cpp, to calculate sine and cosine
/*static*/ float RepRap::SinfCosf(float angle) noexcept
{
	return sinf(angle) + cosf(angle);
}

// Helper function for diagnostic tests in Platform.cpp, to calculate sine and cosine
/*static*/ double RepRap::SinCos(double angle) noexcept
{
	return sin(angle) + cos(angle);
}

// Report an internal error
void RepRap::ReportInternalError(const char *file, const char *func, int line) const noexcept
{
	platform->MessageF(ErrorMessage, "Internal Error in %s at %s(%d)\n", func, file, line);
}

#if SUPPORT_12864_LCD

const char *RepRap::GetLatestMessage(uint16_t& sequence) const noexcept
{
	sequence = messageSequence;
	return message.c_str();
}

#endif

// End
