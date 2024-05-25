#include "RepRap.h"

#include <Movement/Move.h>
#include <Movement/StepTimer.h>
#include <FilamentMonitors/FilamentMonitor.h>
#include <GCodes/GCodes.h>
#include <Heating/Heat.h>
#include <Heating/Sensors/TemperatureSensor.h>
#include <Networking/Network.h>
#if SUPPORT_HTTP
# include <Networking/HttpResponder.h>
#endif
#include "Platform.h"
#include <GCodes/GCodeBuffer/GCodeBuffer.h>
#include <PrintMonitor/PrintMonitor.h>
#include <Tools/Tool.h>
#include <Tools/Filament.h>
#include <Endstops/ZProbe.h>
#include "Tasks.h"
#include <Cache.h>
#include <Fans/FansManager.h>
#include <Hardware/SoftwareReset.h>
#include <Hardware/ExceptionHandlers.h>
#include <Accelerometers/Accelerometers.h>
#include <CoreNotifyIndices.h>
#include "Version.h"

#ifdef DUET_NG
# include "DueXn.h"
#endif

#if SUPPORT_TMC2660
# include <Movement/StepperDrivers/TMC2660.h>
#endif
#if SUPPORT_TMC22xx
# include <Movement/StepperDrivers/TMC22xx.h>
#endif
#if SUPPORT_TMC51xx
# include <Movement/StepperDrivers/TMC51xx.h>
#endif

#if SUPPORT_IOBITS
# include "PortControl.h"
#endif

#if SUPPORT_DIRECT_LCD
# include <Display/Display.h>
#endif

#if HAS_SBC_INTERFACE
# include <SBC/SbcInterface.h>
#endif

#ifdef DUET3_ATE
# include <Duet3Ate.h>
#endif

#if HAS_HIGH_SPEED_SD

# if !SAME5x
#  include <hsmci/hsmci.h>
#  include <conf_sd_mmc.h>
# endif

# if SAME70
// Check correct DMA channel assigned
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
# include <DmacManager.h>
#endif

#if SAM4S
# include <wdt/wdt.h>
#endif

// We call vTaskNotifyGiveFromISR from various interrupts, so the following must be true
static_assert(configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY <= NvicPriorityHSMCI, "configMAX_SYSCALL_INTERRUPT_PRIORITY is set too high");

// This is the string that identifies the board type and firmware version, that the vector at 0x20 points to.
// The characters after the last space must be the firmware version in standard format, e.g. "3.3.0" or "3.4.0beta4". The firmware build date/time is not included.
extern const char VersionText[] = FIRMWARE_NAME " version " VERSION;

#if HAS_HIGH_SPEED_SD && !SAME5x										// SAME5x uses CoreN2G which makes its own RTOS calls

static TaskHandle hsmciTask = nullptr;									// the task that is waiting for a HSMCI command to complete

// HSMCI interrupt handler
extern "C" void HSMCI_Handler() noexcept
{
	HSMCI->HSMCI_IDR = 0xFFFFFFFF;										// disable all HSMCI interrupts
#if SAME70
	XDMAC->XDMAC_CHID[DmacChanHsmci].XDMAC_CID = 0xFFFFFFFF;			// disable all DMA interrupts for this channel
#endif
	TaskBase::GiveFromISR(hsmciTask, NotifyIndices::Sdhc);				// wake up the task
}

#if SAME70

// HSMCI DMA complete callback
void HsmciDmaCallback(CallbackParameter cb, DmaCallbackReason reason) noexcept
{
	HSMCI->HSMCI_IDR = 0xFFFFFFFF;										// disable all HSMCI interrupts
	XDMAC->XDMAC_CHID[DmacChanHsmci].XDMAC_CID = 0xFFFFFFFF;			// disable all DMA interrupts for this channel
	if (hsmciTask != nullptr)
	{
		TaskBase::GiveFromISR(hsmciTask, NotifyIndices::Sdhc);
		hsmciTask = nullptr;
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
		if (!TaskBase::TakeIndexed(NotifyIndices::Sdhc, 200))
		{
			// We timed out waiting for the HSMCI operation to complete
			reprap.GetPlatform().LogError(ErrorCode::HsmciTimeout);
		}
	}
}

#endif //end if HAS_HIGH_SPEED_SD && !SAME5x

// Object model table and functions
// Note: if using GCC version 7.3.1 20180622 and lambda functions are used in this table, you must compile this file with option -std=gnu++17.
// Otherwise the table will be allocate in RAM instead of flash, which wastes too much RAM.

// Macro to build a standard lambda function that includes the necessary type conversions
#define OBJECT_MODEL_FUNC(...) OBJECT_MODEL_FUNC_BODY(RepRap, __VA_ARGS__)
#define OBJECT_MODEL_FUNC_IF(_condition,...) OBJECT_MODEL_FUNC_IF_BODY(RepRap, _condition,__VA_ARGS__)

constexpr ObjectModelArrayTableEntry RepRap::objectModelArrayTable[] =
{
	// 0. Boards
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
	},
	// 1. Fans
	{
		&FansManager::fansLock,
		[] (const ObjectModel *self, const ObjectExplorationContext&) noexcept -> size_t { return ((const RepRap*)self)->fansManager->GetNumFansToReport(); },
		[] (const ObjectModel *self, ObjectExplorationContext& context) noexcept -> ExpressionValue { return ExpressionValue(((const RepRap*)self)->fansManager->FindFan(context.GetLastIndex()).Ptr()); }
	},
	// 2. Inputs
	{
		nullptr,
		[] (const ObjectModel *self, const ObjectExplorationContext&) noexcept -> size_t { return ((const RepRap*)self)->gCodes->GetNumInputs(); },
		[] (const ObjectModel *self, ObjectExplorationContext& context) noexcept -> ExpressionValue { return ExpressionValue(((const RepRap*)self)->gCodes->GetInput(context.GetLastIndex())); }
	},
	// 3. Spindles
	{
		nullptr,					// no lock needed
		[] (const ObjectModel *self, const ObjectExplorationContext& context) noexcept -> size_t { return MaxSpindles; },
		[] (const ObjectModel *self, ObjectExplorationContext& context) noexcept -> ExpressionValue { return ExpressionValue(&((const RepRap*)self)->platform->AccessSpindle(context.GetLastIndex())); }
	},
	// 4. Tools
	{
		&Tool::toolListLock,
		[] (const ObjectModel *self, const ObjectExplorationContext&) noexcept -> size_t { return Tool::GetNumToolsToReport(); },
		[] (const ObjectModel *self, ObjectExplorationContext& context) noexcept -> ExpressionValue { return ExpressionValue(Tool::GetLockedTool(context.GetLastIndex()).Ptr()); }
	},
	// 5. Volumes
	{
		nullptr,
#if HAS_MASS_STORAGE
		[] (const ObjectModel *self, const ObjectExplorationContext&) noexcept -> size_t { return MassStorage::GetNumVolumes(); },
		[] (const ObjectModel *self, ObjectExplorationContext& context) noexcept -> ExpressionValue { return ExpressionValue(MassStorage::GetVolume(context.GetLastIndex())); }
#else
		[] (const ObjectModel *self, const ObjectExplorationContext&) noexcept -> size_t { return 0; },
		[] (const ObjectModel *self, ObjectExplorationContext& context) noexcept -> ExpressionValue { return ExpressionValue(nullptr); }
#endif
	},
	// 6. GP outputs
	{
		nullptr,
		[] (const ObjectModel *self, const ObjectExplorationContext&) noexcept -> size_t { return reprap.GetPlatform().GetNumGpOutputsToReport(); },
		[] (const ObjectModel *self, ObjectExplorationContext& context) noexcept -> ExpressionValue
						{
							const GpOutputPort& port = reprap.GetPlatform().GetGpOutPort(context.GetLastIndex());
							return (port.IsUnused()) ? ExpressionValue(nullptr) : ExpressionValue(&port);
						}
	},
	// 7. Restore points
	{
		nullptr,
		[] (const ObjectModel *self, const ObjectExplorationContext&) noexcept -> size_t { return NumVisibleRestorePoints; },
		[] (const ObjectModel *self, ObjectExplorationContext& context) noexcept -> ExpressionValue
																			{ return ExpressionValue(&((const RepRap*)self)->gCodes->GetCurrentMovementState(context).restorePoints[context.GetLastIndex()]); }
	},
	// 8. Volume changes
	{
		nullptr,
#if HAS_MASS_STORAGE
		[] (const ObjectModel *self, const ObjectExplorationContext&) noexcept -> size_t { return MassStorage::GetNumVolumes(); },
		[] (const ObjectModel *self, ObjectExplorationContext& context) noexcept -> ExpressionValue
																			{ return ExpressionValue((int32_t)MassStorage::GetVolumeSeq(context.GetLastIndex())); }
#else
		[] (const ObjectModel *self, const ObjectExplorationContext&) noexcept -> size_t { return 0; },
		[] (const ObjectModel *self, ObjectExplorationContext& context) noexcept -> ExpressionValue { return ExpressionValue(nullptr); }
#endif
	}
#if SUPPORT_LED_STRIPS
	,
	// 9. LED strips
	{
		&LedStripManager::ledLock,
		[] (const ObjectModel *self, const ObjectExplorationContext&) noexcept -> size_t { return ((const RepRap*)self)->platform->GetLedStripManager().GetNumLedStrips(); },
		[] (const ObjectModel *self, ObjectExplorationContext& context) noexcept -> ExpressionValue
				{ return ExpressionValue(((const RepRap*)self)->platform->GetLedStripManager().GetLedStrip(context.GetLastIndex())); }
	}
#endif
};

DEFINE_GET_OBJECT_MODEL_ARRAY_TABLE(RepRap)

constexpr unsigned int StateSubTableNumber = 3;		// section number of 'state' in the following
constexpr ObjectModelTableEntry RepRap::objectModelTable[] =
{
	// Within each group, these entries must be in alphabetical order
	// 0. root
	{ "boards",					OBJECT_MODEL_FUNC_ARRAY(0),												ObjectModelEntryFlags::live },
#if HAS_MASS_STORAGE || HAS_EMBEDDED_FILES || HAS_SBC_INTERFACE
	{ "directories",			OBJECT_MODEL_FUNC(self, 1),												ObjectModelEntryFlags::none },
#endif
	{ "fans",					OBJECT_MODEL_FUNC_ARRAY(1),												ObjectModelEntryFlags::live },
	{ "global",					OBJECT_MODEL_FUNC(&(self->globalVariables)),							ObjectModelEntryFlags::none },
	{ "heat",					OBJECT_MODEL_FUNC(self->heat),											ObjectModelEntryFlags::live },
	{ "inputs",					OBJECT_MODEL_FUNC_ARRAY(2),												ObjectModelEntryFlags::live },
	{ "job",					OBJECT_MODEL_FUNC(self->printMonitor),									ObjectModelEntryFlags::live },
#if SUPPORT_LED_STRIPS
	{ "ledStrips",				OBJECT_MODEL_FUNC_ARRAY(9),												ObjectModelEntryFlags::none },
#endif
	{ "limits",					OBJECT_MODEL_FUNC(self, 2),												ObjectModelEntryFlags::none },
	{ "move",					OBJECT_MODEL_FUNC(self->move),											ObjectModelEntryFlags::live },
	// Note, 'network' is needed even if there is no networking, because it contains the machine name
	{ "network",				OBJECT_MODEL_FUNC(self->network),										ObjectModelEntryFlags::none },
	{ "sensors",				OBJECT_MODEL_FUNC(&self->platform->GetEndstops()),						ObjectModelEntryFlags::live },
	{ "seqs",					OBJECT_MODEL_FUNC(self, 5),												ObjectModelEntryFlags::live },
	{ "spindles",				OBJECT_MODEL_FUNC_ARRAY(3),												ObjectModelEntryFlags::live },
	{ "state",					OBJECT_MODEL_FUNC(self, 3),												ObjectModelEntryFlags::live },
	{ "tools",					OBJECT_MODEL_FUNC_ARRAY(4),												ObjectModelEntryFlags::live },
	{ "volumes",				OBJECT_MODEL_FUNC_ARRAY(5),												ObjectModelEntryFlags::none },

	// 1. directories
#if HAS_MASS_STORAGE || HAS_EMBEDDED_FILES || HAS_SBC_INTERFACE
	{ "filaments",				OBJECT_MODEL_FUNC_NOSELF(FILAMENTS_DIRECTORY),							ObjectModelEntryFlags::verbose },
	{ "firmware",				OBJECT_MODEL_FUNC_NOSELF(FIRMWARE_DIRECTORY),							ObjectModelEntryFlags::verbose },
	{ "gCodes",					OBJECT_MODEL_FUNC_NOSELF(Platform::GetGCodeDir()),						ObjectModelEntryFlags::verbose },
	{ "macros",					OBJECT_MODEL_FUNC_NOSELF(Platform::GetMacroDir()),						ObjectModelEntryFlags::verbose },
	{ "menu",					OBJECT_MODEL_FUNC_NOSELF(MENU_DIR),										ObjectModelEntryFlags::verbose },
	{ "system",					OBJECT_MODEL_FUNC_NOSELF(ExpressionValue::SpecialType::sysDir, 0),		ObjectModelEntryFlags::none },
	{ "web",					OBJECT_MODEL_FUNC_NOSELF(Platform::GetWebDir()),						ObjectModelEntryFlags::verbose },
#endif

	// 2. limits
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
	{ "drivers",				OBJECT_MODEL_FUNC((int32_t)self->platform->GetNumActualDirectDrivers()), ObjectModelEntryFlags::verbose },
#endif
	{ "driversPerAxis",			OBJECT_MODEL_FUNC_NOSELF((int32_t)MaxDriversPerAxis),					ObjectModelEntryFlags::verbose },
	{ "extruders",				OBJECT_MODEL_FUNC_NOSELF((int32_t)MaxExtruders),						ObjectModelEntryFlags::verbose },
	{ "extrudersPerTool",		OBJECT_MODEL_FUNC_NOSELF((int32_t)MaxExtrudersPerTool),					ObjectModelEntryFlags::verbose },
	{ "fans",					OBJECT_MODEL_FUNC_NOSELF((int32_t)MaxFans),								ObjectModelEntryFlags::verbose },
	{ "gpInPorts",				OBJECT_MODEL_FUNC_NOSELF((int32_t)MaxGpInPorts),						ObjectModelEntryFlags::verbose },
	{ "gpOutPorts",				OBJECT_MODEL_FUNC_NOSELF((int32_t)MaxGpOutPorts),						ObjectModelEntryFlags::verbose },
	{ "heaters",				OBJECT_MODEL_FUNC_NOSELF((int32_t)MaxHeaters),							ObjectModelEntryFlags::verbose },
	{ "heatersPerTool",			OBJECT_MODEL_FUNC_NOSELF((int32_t)MaxHeatersPerTool),					ObjectModelEntryFlags::verbose },
#if SUPPORT_LED_STRIPS
	{ "ledStrips",				OBJECT_MODEL_FUNC_NOSELF((int32_t)MaxLedStrips),						ObjectModelEntryFlags::verbose },
#endif
	{ "monitorsPerHeater",		OBJECT_MODEL_FUNC_NOSELF((int32_t)MaxMonitorsPerHeater),				ObjectModelEntryFlags::verbose },
	{ "portsPerHeater",			OBJECT_MODEL_FUNC_NOSELF((int32_t)MaxPortsPerHeater),					ObjectModelEntryFlags::verbose },
	{ "restorePoints",			OBJECT_MODEL_FUNC_NOSELF((int32_t)NumVisibleRestorePoints),				ObjectModelEntryFlags::verbose },
	{ "sensors",				OBJECT_MODEL_FUNC_NOSELF((int32_t)MaxSensors),							ObjectModelEntryFlags::verbose },
	{ "spindles",				OBJECT_MODEL_FUNC_NOSELF((int32_t)MaxSpindles),							ObjectModelEntryFlags::verbose },
	{ "tools",					OBJECT_MODEL_FUNC_NOSELF((int32_t)MaxTools),							ObjectModelEntryFlags::verbose },
	{ "trackedObjects",			OBJECT_MODEL_FUNC_NOSELF((int32_t)MaxTrackedObjects),					ObjectModelEntryFlags::verbose },
	{ "triggers",				OBJECT_MODEL_FUNC_NOSELF((int32_t)MaxTriggers),							ObjectModelEntryFlags::verbose },
#if HAS_MASS_STORAGE
	{ "volumes",				OBJECT_MODEL_FUNC_NOSELF((int32_t)MassStorage::GetNumVolumes()),		ObjectModelEntryFlags::verbose },
#else
	{ "volumes",				OBJECT_MODEL_FUNC_NOSELF((int32_t)0),									ObjectModelEntryFlags::verbose },
#endif
	{ "workplaces",				OBJECT_MODEL_FUNC_NOSELF((int32_t)NumCoordinateSystems),				ObjectModelEntryFlags::verbose },
	{ "zProbeProgramBytes",		OBJECT_MODEL_FUNC_NOSELF((int32_t)MaxZProbeProgramBytes),				ObjectModelEntryFlags::verbose },
	{ "zProbes",				OBJECT_MODEL_FUNC_NOSELF((int32_t)MaxZProbes),							ObjectModelEntryFlags::verbose },

	// 3. state (see declaration of StateSubTableNumber above)
	{ "atxPower",				OBJECT_MODEL_FUNC_IF(self->platform->IsAtxPowerControlled(), self->platform->GetAtxPowerState()),	ObjectModelEntryFlags::none },
	{ "atxPowerPort",			OBJECT_MODEL_FUNC_IF(self->platform->IsAtxPowerControlled(), self->platform->GetAtxPowerPort()),	ObjectModelEntryFlags::none },
	{ "beep",					OBJECT_MODEL_FUNC_IF(self->beepDuration != 0, self, 4),					ObjectModelEntryFlags::none },
	{ "currentTool",			OBJECT_MODEL_FUNC((int32_t)self->gCodes->GetCurrentMovementState(context).GetCurrentToolNumber()),	ObjectModelEntryFlags::live },
	{ "deferredPowerDown",		OBJECT_MODEL_FUNC_IF(self->platform->IsAtxPowerControlled(), self->platform->IsDeferredPowerDown()),	ObjectModelEntryFlags::none },
	{ "displayMessage",			OBJECT_MODEL_FUNC(self->message.c_str()),								ObjectModelEntryFlags::none },
	{ "gpOut",					OBJECT_MODEL_FUNC_ARRAY(6),												ObjectModelEntryFlags::live },
#if SUPPORT_LASER
	// 2020-04-24: return the configured laser PWM even if the laser is temporarily turned off
	{ "laserPwm",				OBJECT_MODEL_FUNC_IF(self->gCodes->GetMachineType() == MachineType::laser, self->gCodes->GetLaserPwm(), 2),	ObjectModelEntryFlags::live },
#endif
#if HAS_MASS_STORAGE
	{ "logFile",				OBJECT_MODEL_FUNC(self->platform->GetLogFileName()),					ObjectModelEntryFlags::none },
#else
	{ "logFile",				OBJECT_MODEL_FUNC_NOSELF(nullptr),										ObjectModelEntryFlags::none },
#endif
	{ "logLevel",				OBJECT_MODEL_FUNC(self->platform->GetLogLevel()),						ObjectModelEntryFlags::none },
	{ "machineMode",			OBJECT_MODEL_FUNC(self->gCodes->GetMachineModeString()),				ObjectModelEntryFlags::none },
	{ "macroRestarted",			OBJECT_MODEL_FUNC(self->gCodes->GetMacroRestarted()),					ObjectModelEntryFlags::none },
	{ "messageBox",				OBJECT_MODEL_FUNC_IF_NOSELF(MessageBox::HaveCurrent(), MessageBox::GetCurrent(), 0), ObjectModelEntryFlags::important },
	{ "msUpTime",				OBJECT_MODEL_FUNC_NOSELF((int32_t)(context.GetStartMillis() % 1000u)),	ObjectModelEntryFlags::live },
	{ "nextTool",				OBJECT_MODEL_FUNC((int32_t)self->gCodes->GetCurrentMovementState(context).newToolNumber), ObjectModelEntryFlags::none },
#if HAS_VOLTAGE_MONITOR
	{ "powerFailScript",		OBJECT_MODEL_FUNC(self->gCodes->GetPowerFailScript()),					ObjectModelEntryFlags::none },
#endif
	{ "previousTool",			OBJECT_MODEL_FUNC((int32_t)self->gCodes->GetCurrentMovementState(context).previousToolNumber),	ObjectModelEntryFlags::none },
	{ "restorePoints",			OBJECT_MODEL_FUNC_ARRAY(7),												ObjectModelEntryFlags::none },
	{ "startupError",			OBJECT_MODEL_FUNC_IF(!self->configErrorMessage.IsNull(), self, 6),		ObjectModelEntryFlags::none },
	{ "status",					OBJECT_MODEL_FUNC(self->GetStatusString()),								ObjectModelEntryFlags::live },
	{ "thisActive",
#if SUPPORT_ASYNC_MOVES
								OBJECT_MODEL_FUNC_IF_NOSELF(context.GetGCodeBuffer() != nullptr, context.GetGCodeBuffer()->Executing()), ObjectModelEntryFlags::none },
#else
								OBJECT_MODEL_FUNC_IF_NOSELF(context.GetGCodeBuffer() != nullptr, true),	ObjectModelEntryFlags::verbose },
#endif
	{ "thisInput",				OBJECT_MODEL_FUNC_IF_NOSELF(context.GetGCodeBuffer() != nullptr, (int32_t)context.GetGCodeBuffer()->GetChannel().ToBaseType()),	ObjectModelEntryFlags::verbose },
	{ "time",					OBJECT_MODEL_FUNC(DateTime(self->platform->GetDateTime())),				ObjectModelEntryFlags::live },
	{ "upTime",					OBJECT_MODEL_FUNC_NOSELF((int32_t)((context.GetStartMillis()/1000u) & 0x7FFFFFFF)),	ObjectModelEntryFlags::live },

	// 4. state.beep
	{ "duration",				OBJECT_MODEL_FUNC((int32_t)self->beepDuration),							ObjectModelEntryFlags::none },
	{ "frequency",				OBJECT_MODEL_FUNC((int32_t)self->beepFrequency),						ObjectModelEntryFlags::none },

	// 5. seqs
	{ "boards",					OBJECT_MODEL_FUNC((int32_t)self->boardsSeq),							ObjectModelEntryFlags::live },
#if HAS_MASS_STORAGE || HAS_EMBEDDED_FILES || HAS_SBC_INTERFACE
	{ "directories",			OBJECT_MODEL_FUNC((int32_t)self->directoriesSeq),						ObjectModelEntryFlags::live },
#endif
	{ "fans",					OBJECT_MODEL_FUNC((int32_t)self->fansSeq),								ObjectModelEntryFlags::live },
	{ "global",					OBJECT_MODEL_FUNC((int32_t)self->globalSeq),							ObjectModelEntryFlags::live },
	{ "heat",					OBJECT_MODEL_FUNC((int32_t)self->heatSeq),								ObjectModelEntryFlags::live },
	{ "inputs",					OBJECT_MODEL_FUNC((int32_t)self->inputsSeq),							ObjectModelEntryFlags::live },
	{ "job",					OBJECT_MODEL_FUNC((int32_t)self->jobSeq),								ObjectModelEntryFlags::live },
#if SUPPORT_LED_STRIPS
	{ "ledStrips",				OBJECT_MODEL_FUNC((int32_t)self->ledStripsSeq),							ObjectModelEntryFlags::live },
#endif
	// no need for 'limits' because it never changes
	{ "move",					OBJECT_MODEL_FUNC((int32_t)self->moveSeq),								ObjectModelEntryFlags::live },
	// Note, 'network' is needed even if there is no networking, because it contains the machine name
	{ "network",				OBJECT_MODEL_FUNC((int32_t)self->networkSeq),							ObjectModelEntryFlags::live },
#if HAS_NETWORKING
	{ "reply",					OBJECT_MODEL_FUNC_NOSELF((int32_t)HttpResponder::GetReplySeq()),		ObjectModelEntryFlags::live },
#endif
	{ "sensors",				OBJECT_MODEL_FUNC((int32_t)self->sensorsSeq),							ObjectModelEntryFlags::live },
	{ "spindles",				OBJECT_MODEL_FUNC((int32_t)self->spindlesSeq),							ObjectModelEntryFlags::live },
	{ "state",					OBJECT_MODEL_FUNC((int32_t)self->stateSeq),								ObjectModelEntryFlags::live },
	{ "tools",					OBJECT_MODEL_FUNC((int32_t)self->toolsSeq),								ObjectModelEntryFlags::live },
#if HAS_MASS_STORAGE
	{ "volChanges",				OBJECT_MODEL_FUNC_ARRAY(8),												ObjectModelEntryFlags::live },
	{ "volumes",				OBJECT_MODEL_FUNC((int32_t)self->volumesSeq),							ObjectModelEntryFlags::live },
#endif

	// 6. state.startupError
	{ "file",					OBJECT_MODEL_FUNC(self->configErrorFilename.IncreaseRefCount()),		ObjectModelEntryFlags::none },
	{ "line",					OBJECT_MODEL_FUNC((int32_t)self->configErrorLine),						ObjectModelEntryFlags::none },
	{ "message",				OBJECT_MODEL_FUNC(self->configErrorMessage.IncreaseRefCount()),			ObjectModelEntryFlags::none },
};

ReadWriteLock *_ecv_null RepRap::GetObjectLock(unsigned int tableNumber) const noexcept /*override*/
{
	return (tableNumber == StateSubTableNumber) ? &MessageBox::mboxLock : nullptr;
}

constexpr uint8_t RepRap::objectModelTableDescriptor[] =
{
	7,																						// number of sub-tables
	15 + (HAS_MASS_STORAGE | HAS_EMBEDDED_FILES | HAS_SBC_INTERFACE) + SUPPORT_LED_STRIPS,	// root
#if HAS_MASS_STORAGE || HAS_EMBEDDED_FILES || HAS_SBC_INTERFACE
	7, 																						// directories
#else
	0,																						// directories
#endif
	26 + SUPPORT_LED_STRIPS,																// limits
	22 + HAS_VOLTAGE_MONITOR + SUPPORT_LASER,												// state
	2,																						// state.beep
	12 + HAS_NETWORKING + (2 * HAS_MASS_STORAGE) + (HAS_MASS_STORAGE | HAS_EMBEDDED_FILES | HAS_SBC_INTERFACE) + SUPPORT_LED_STRIPS,	// seqs
	3																						// state.configErr
};

DEFINE_GET_OBJECT_MODEL_TABLE(RepRap)

// RepRap member functions.

// Do nothing more in the constructor; put what you want in RepRap:Init()

RepRap::RepRap() noexcept
	: boardsSeq(0), directoriesSeq(0), fansSeq(0), heatSeq(0), inputsSeq(0), jobSeq(0), ledStripsSeq(0), moveSeq(0), globalSeq(0),
	  networkSeq(0), scannerSeq(0), sensorsSeq(0), spindlesSeq(0), stateSeq(0), toolsSeq(0), volumesSeq(0),
	  lastWarningMillis(0),
	  ticksInSpinState(0), heatTaskIdleTicks(0),
	  beepFrequency(0), beepDuration(0), beepTimer(0),
	  diagnosticsDestination(MessageType::NoDestinationMessage), justSentDiagnostics(false),
	  spinningModule(Module::none), stopped(false), active(false), processingConfig(true)
#if HAS_SBC_INTERFACE
	  , usingSbcInterface(false)						// default to not using the SBC interface until we have checked for config.g on an SD card,
														// because a disconnected SBC interface can generate noise which may trigger interrupts and DMA
#endif
{
	ClearDebug();
	// Don't call constructors for other objects here
}

#if 0

///DEBUG to catch memory corruption
const size_t WatchSize = 32768;
uint32_t *watchBuffer;

static void InitWatchBuffer() noexcept
{
	watchBuffer = (uint32_t*)malloc(WatchSize);
	memset(watchBuffer, 0x5A, WatchSize);
}

static void CheckWatchBuffer(unsigned int module) noexcept
{
	uint32_t *p = watchBuffer, *end = watchBuffer + 32768/sizeof(uint32_t);
	while (p < end)
	{
		if (*p != 0x5A5A5A5A)
		{
			debugPrintf("Address %p data %08" PRIx32 " module %u\n", p, *p, module);
			*p = 0x5A5A5A5A;
		}
		++p;
	}
}

#endif

void RepRap::Init() noexcept
{
	OutputBuffer::Init();
	objectModelReportMutex.Create("ModelReport");

	platform = new Platform();
#if HAS_SBC_INTERFACE
	sbcInterface = new SbcInterface();				// needs to be allocated early on Duet 2 so as to avoid using any of the last 64K of RAM
#endif
	network = new Network(*platform);
	gCodes = new GCodes(*platform);
	move = new Move();
	heat = new Heat();
	printMonitor = new PrintMonitor(*platform, *gCodes);
	fansManager = new FansManager;

#if SUPPORT_IOBITS
	portControl = new PortControl();
#endif
#if SUPPORT_DIRECT_LCD
 	display = new Display();
#endif
#if SUPPORT_CAN_EXPANSION
	expansion = new ExpansionManager();
#endif

	SetPassword(DEFAULT_PASSWORD);
	message.Clear();
#if SUPPORT_DIRECT_LCD
	messageSequence = 0;
#endif

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

#if SUPPORT_IOBITS
	portControl->Init();
#endif
#if SUPPORT_DIRECT_LCD
	display->Init();
#endif
#ifdef DUET3_ATE
	Duet3Ate::Init();
#endif
	// sbcInterface is not initialised until we know we are using it, to prevent a disconnected SBC interface generating interrupts and DMA

	// Set up the timeout of the regular watchdog, and set up the backup watchdog if there is one.
#if SAME5x
	WatchdogInit();
	NVIC_SetPriority(WDT_IRQn, NvicPriorityWatchdog);								// set priority for watchdog interrupts
	NVIC_ClearPendingIRQ(WDT_IRQn);
	NVIC_EnableIRQ(WDT_IRQn);														// enable the watchdog early warning interrupt
#else
	{
		// The clock frequency for both watchdogs is about 32768/128 = 256Hz
		// The watchdogs on the SAM4E seem to be very timing-sensitive. On the Duet WiFi/Ethernet they were going off spuriously depending on how long the DueX initialisation took.
		// The documentation says you mustn't write to the mode register within 3 slow clocks after kicking the watchdog.
		// I have a theory that the converse is also true, i.e. after enabling the watchdog you mustn't kick it within 3 slow clocks
		// So I've added a delay call before we set 'active' true (which enables kicking the watchdog), and that seems to fix the problem.
# if SAM4E || SAME70
		const uint16_t mainTimeout = 49152/128;										// set main (back stop) watchdog timeout to 1.5s second (max allowed value is 4095 = 16 seconds)
		WDT->WDT_MR = WDT_MR_WDRSTEN | WDT_MR_WDDBGHLT | WDT_MR_WDV(mainTimeout) | WDT_MR_WDD(mainTimeout);	// reset the processor on a watchdog fault, stop it when debugging

		// The RSWDT must be initialised *after* the main WDT
		const uint16_t rsTimeout = 32768/128;										// set secondary watchdog timeout to 1 second (max allowed value is 4095 = 16 seconds)
#  if SAME70
		RSWDT->RSWDT_MR = RSWDT_MR_WDFIEN | RSWDT_MR_WDDBGHLT | RSWDT_MR_WDV(rsTimeout) | RSWDT_MR_ALLONES_Msk;		// generate an interrupt on a watchdog fault
		NVIC_SetPriority(RSWDT_IRQn, NvicPriorityWatchdog);							// set priority for watchdog interrupts
		NVIC_ClearPendingIRQ(RSWDT_IRQn);
		NVIC_EnableIRQ(RSWDT_IRQn);													// enable the watchdog interrupt
#  else
		RSWDT->RSWDT_MR = RSWDT_MR_WDFIEN | RSWDT_MR_WDDBGHLT | RSWDT_MR_WDV(rsTimeout) | RSWDT_MR_WDD(rsTimeout);	// generate an interrupt on a watchdog fault
		NVIC_SetPriority(WDT_IRQn, NvicPriorityWatchdog);							// set priority for watchdog interrupts
		NVIC_ClearPendingIRQ(WDT_IRQn);
		NVIC_EnableIRQ(WDT_IRQn);													// enable the watchdog interrupt
#  endif
# else
		// We don't have a RSWDT so set the main watchdog timeout to 1 second
		const uint16_t timeout = 32768/128;											// set watchdog timeout to 1 second (max allowed value is 4095 = 16 seconds)
		wdt_init(WDT, WDT_MR_WDRSTEN | WDT_MR_WDDBGHLT, timeout, timeout);			// reset the processor on a watchdog fault, stop it when debugging
# endif
		delayMicroseconds(200);														// 200us is about 6 slow clocks
	}
#endif

	active = true;										// must do this after we initialise the watchdog but before we start the network or call Spin(), else the watchdog may time out

	delay(100);											// give the tick ISR time to collect voltage readings
	platform->ResetVoltageMonitors();					// get rid of the spurious zero minimum voltage readings

	platform->MessageF(UsbMessage, "%s\n", VersionText);

#if HAS_SBC_INTERFACE && !HAS_MASS_STORAGE
	usingSbcInterface = true;
	FileWriteBuffer::UsingSbcMode();
#endif

#if HAS_MASS_STORAGE || HAS_EMBEDDED_FILES
	{
		// Try to mount the first SD card
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
# if defined(DUET3_MB6HC)
			network->CreateAdditionalInterface();		// do this now because config.g may refer to it
# endif
			// Run the configuration file
			if (!RunStartupFile(GCodes::CONFIG_FILE, true) && !RunStartupFile(GCodes::CONFIG_BACKUP_FILE, true))
			{
				platform->Message(AddWarning(UsbMessage), "no configuration file found\n");
			}
		}
# if HAS_SBC_INTERFACE
		else if (!MassStorage::IsCardDetected(0))		// if we failed to mount the SD card because there was no card in the slot
		{
			usingSbcInterface = true;
			FileWriteBuffer::UsingSbcMode();
		}
# endif
		else
		{
			delay(3000);								// Wait a few seconds so users have a chance to see this
			platform->MessageF(AddWarning(UsbMessage), "%s\n", reply.c_str());
		}
	}
#elif defined(DUET_NG)
	// It's the SBC build of Duet 2 firmware. Enable the PanelDue port so that the ATE can test it.
	platform->SetBaudRate(1, 57600);
	platform->SetCommsProperties(1, 1);
	gCodes->SetAux0CommsProperties(1);
	platform->SetAuxRaw(0, false);
	platform->EnableAux(0);
#endif

#if HAS_SBC_INTERFACE
	if (usingSbcInterface)
	{
		sbcInterface->Init();

		// Keep spinning until the SBC connects
		while (!sbcInterface->IsConnected())
		{
			Spin();
		}

		// Run config.g or config.g.bak
		if (!RunStartupFile(GCodes::CONFIG_FILE, true))
		{
			RunStartupFile(GCodes::CONFIG_BACKUP_FILE, true);
		}

		// runonce.g is executed by the SBC as soon as processingConfig is set to false.
		// As we are running the SBC, save RAM by not activating the network
	}
	else
#endif
	{
		network->Activate();							// need to do this here, as the configuration GCodes may set IP address etc.
#if HAS_MASS_STORAGE
		// If we are running from SD card, run the runonce.g file if it exists, then delete it
		if (RunStartupFile(GCodes::RUNONCE_G, false))
		{
			platform->DeleteSysFile(GCodes::RUNONCE_G);
		}
#endif
	}
	processingConfig = false;

#if HAS_HIGH_SPEED_SD && !SAME5x
	// Switch to giving up the CPU while waiting for a SD operation to complete
	hsmci_set_idle_func(hsmciIdle);
	HSMCI->HSMCI_IDR = 0xFFFFFFFF;						// disable all HSMCI interrupts
	NVIC_EnableIRQ(HSMCI_IRQn);
#endif

	platform->MessageF(UsbMessage, "%s is up and running.\n", FIRMWARE_NAME);

	fastLoop = UINT32_MAX;
	slowLoop = 0;

#if STEP_TIMER_DEBUG
	(void)StepTimer::GetTimerTicks();
	StepTimer::maxInterval = 0;
#endif
}

// Run a startup file
bool RepRap::RunStartupFile(const char *filename, bool isMainConfigFile) noexcept
{
	const bool rslt = gCodes->RunConfigFile(filename, isMainConfigFile);
	if (rslt)
	{
		platform->MessageF(UsbMessage, "Executing %s... ", filename);
		do
		{
			// GCodes::Spin will process the macro file and ensure IsTriggerBusy returns false when it's done
			Spin();
		} while (gCodes->IsTriggerBusy());
		platform->Message(UsbMessage, "Done!\n");
	}
	return rslt;
}

void RepRap::Exit() noexcept
{
#if HAS_HIGH_SPEED_SD && !SAME5x		// SAME5x MCI driver is RTOS-aware so it doesn't need this
	hsmci_set_idle_func(nullptr);
#endif
	active = false;
	heat->Exit();
	move->Exit();
	gCodes->Exit();
#if SUPPORT_IOBITS
	portControl->Exit();
#endif
#if SUPPORT_DIRECT_LCD
 	display->Exit();
#endif
	network->Exit();
	platform->Exit();
#if SUPPORT_ACCELEROMETERS
	Accelerometers::Exit();
#endif
}

void RepRap::Spin() noexcept
{
	if (!active)
	{
		return;
	}

	const uint32_t lastTime = StepTimer::GetTimerTicks();

	ticksInSpinState = 0;
	spinningModule = Module::Platform;
	platform->Spin();

	ticksInSpinState = 0;
	spinningModule = Module::Gcodes;
	gCodes->Spin();

	ticksInSpinState = 0;
	spinningModule = Module::PrintMonitor;
	printMonitor->Spin();

	ticksInSpinState = 0;
	spinningModule = Module::FilamentSensors;
	FilamentMonitor::Spin();

#if SUPPORT_DIRECT_LCD
	ticksInSpinState = 0;
	spinningModule = Module::Display;
	display->Spin();
#endif

#if SUPPORT_CAN_EXPANSION
	ticksInSpinState = 0;
	spinningModule = Module::Expansion;
	expansion->Spin();
#endif

	ticksInSpinState = 0;
	spinningModule = Module::none;

	// Check if we need to send diagnostics
	if (diagnosticsDestination != MessageType::NoDestinationMessage)
	{
		Diagnostics(diagnosticsDestination);
		diagnosticsDestination = MessageType::NoDestinationMessage;
	}

	const uint32_t now = millis();

#if SUPPORT_REMOTE_COMMANDS
	const DeferredCommand defCom = deferredCommand;			// capture volatile variable
	if (defCom != DeferredCommand::none && now - whenDeferredCommandScheduled >= 250)
	{
		switch (defCom)
		{
		case DeferredCommand::reboot:
			SoftwareReset(SoftwareResetReason::user);
			break;

		case DeferredCommand::updateFirmware:
			UpdateFirmware(IAP_CAN_LOADER_FILE, "");
			break;

		default:
			deferredCommand = DeferredCommand::none;
			break;
		}
	}
#endif

	// Check if we need to display a cold extrusion warning
	if (now - lastWarningMillis >= MinimumWarningInterval)
	{
		if (Tool::DisplayColdExtrusionWarnings())
		{
			lastWarningMillis = now;
		}
	}

	// Check if the beep request can be cleared
	if (beepTimer != 0 && now - beepTimer >= beepDuration)
	{
		beepDuration = beepFrequency = beepTimer = 0;
		StateUpdated();
	}

	if (MessageBox::CheckTimeout())
	{
		StateUpdated();
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
	platform->MessageF(mtype, "Slowest loop: %.2fms; fastest: %.2fms\n", (double)(slowLoop * StepClocksToMillis), (double)(fastLoop * StepClocksToMillis));
	fastLoop = UINT32_MAX;
	slowLoop = 0;
}

void RepRap::Diagnostics(MessageType mtype) noexcept
{
	platform->Message(mtype, "=== Diagnostics ===\n");

	// Print the firmware version, board type etc.

#ifdef DUET_NG
	const char* const expansionName = DuetExpansion::GetExpansionBoardName();
#endif

	platform->MessageF(mtype,
		// Format string
		"%s"											// firmware name
		" version %s (%s%s) running on %s"				// firmware version, date, time, electronics
#ifdef DUET_NG
		"%s%s"											// optional DueX expansion board
#endif
#if HAS_SBC_INTERFACE || SUPPORT_REMOTE_COMMANDS
		" (%s mode)"									// standalone, SBC or expansion mode
#endif
		"\n",

		// Parameters to match format string
		FIRMWARE_NAME,
		VERSION, DATE, TIME_SUFFIX, platform->GetElectronicsString()
#ifdef DUET_NG
		, ((expansionName == nullptr) ? "" : " + ")
		, ((expansionName == nullptr) ? "" : expansionName)
#endif
#if HAS_SBC_INTERFACE || SUPPORT_REMOTE_COMMANDS
		,
# if SUPPORT_REMOTE_COMMANDS
						(CanInterface::InExpansionMode()) ? "expansion" :
# endif
# if HAS_SBC_INTERFACE
						(UsingSbcInterface()) ? "SBC" :
# endif
							"standalone"
#endif
	);

	// DEBUG print the module addresses
	//	platform->MessageF(mtype, "platform %" PRIx32 ", network %" PRIx32 ", move %" PRIx32 ", heat %" PRIx32 ", gcodes %" PRIx32 ", scanner %"  PRIx32 ", pm %" PRIx32 ", portc %" PRIx32 "\n",
	//						(uint32_t)platform, (uint32_t)network, (uint32_t)move, (uint32_t)heat, (uint32_t)gCodes, (uint32_t)scanner, (uint32_t)printMonitor, (uint32_t)portControl);

#if MCU_HAS_UNIQUE_ID
	{
		String<StringLength50> idChars;
		platform->GetUniqueId().AppendCharsToString(idChars.GetRef());
		platform->MessageF(mtype, "Board ID: %s\n", idChars.c_str());
	}
#endif

	// Show the used and free buffer counts. Do this early in case we are running out of them and the diagnostics get truncated.
	OutputBuffer::Diagnostics(mtype);

	// If there was an error running config.g, print it
	if (!configErrorMessage.IsNull())
	{
		auto msg  = configErrorMessage.Get();
		auto fname = configErrorFilename.Get();
		platform->MessageF(mtype, "Error in %s line %u while starting up: %s\n", fname.Ptr(), configErrorLine, msg.Ptr());
	}

	// Now print diagnostics for other modules
	Tasks::Diagnostics(mtype);
	platform->Diagnostics(mtype);				// this includes a call to our Timing() function
#if HAS_MASS_STORAGE || HAS_EMBEDDED_FILES
	MassStorage::Diagnostics(mtype);
#endif
	move->Diagnostics(mtype);
	heat->Diagnostics(mtype);
	gCodes->Diagnostics(mtype);
	FilamentMonitor::Diagnostics(mtype);
#ifdef DUET_NG
	DuetExpansion::Diagnostics(mtype);
#endif
#if SUPPORT_CAN_EXPANSION
	CanInterface::Diagnostics(mtype);
#endif
#if HAS_SBC_INTERFACE
	if (usingSbcInterface)
	{
		sbcInterface->Diagnostics(mtype);
	}
	else
#endif
	{
		network->Diagnostics(mtype);
	}

	justSentDiagnostics = true;
}

// Turn off the heaters, disable the motors, and deactivate the Heat, Move and GCodes classes. Leave everything else working.
void RepRap::EmergencyStop() noexcept
{
#ifdef DUET3_ATE
	Duet3Ate::PowerOffEUT();
#endif

	stopped = true;									// a useful side effect of setting this is that it prevents Platform::Tick being called, which is needed when loading IAP into RAM

	// Do not turn off ATX power here. If the nozzles are still hot, don't risk melting any surrounding parts by turning fans off.
	//platform->SetAtxPower(false);

#if SUPPORT_REMOTE_COMMANDS
	if (CanInterface::InExpansionMode())
	{
		platform->EmergencyDisableDrivers();		// disable all local drivers - need to do this to ensure that any motor brakes are re-engaged
	}
	else
#endif
	{
		platform->DisableAllDrivers();				// disable all local and remote drivers - need to do this to ensure that any motor brakes are re-engaged

		switch (gCodes->GetMachineType())
		{
		case MachineType::cnc:
			for (size_t i = 0; i < MaxSpindles; i++)
			{
				platform->AccessSpindle(i).SetState(SpindleState::stopped);
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
	}

	heat->Exit();									// this also turns off all heaters
	move->Exit();									// this stops the motors stepping

#if SUPPORT_CAN_EXPANSION
# if SUPPORT_REMOTE_COMMANDS
	if (CanInterface::InExpansionMode())
	{
		// We can't shut down CAN here because this gets called by the CAN Receive task
	}
	else
# endif
	{
		expansion->EmergencyStop();
	}
#endif

	gCodes->EmergencyStop();
	platform->StopLogging();
}

void RepRap::SetDebug(Module m, uint32_t flags) noexcept
{
	if (m.ToBaseType() < NumRealModules)
	{
		debugMaps[m.ToBaseType()].SetFromRaw(flags);
	}
}

void RepRap::ClearDebug() noexcept
{
	for (DebugFlags& dm : debugMaps)
	{
		dm.Clear();
	}
}

void RepRap::PrintDebug(MessageType mt) noexcept
{
	platform->Message((MessageType)(mt | PushFlag), "Debugging enabled for modules:");
	for (size_t i = 0; i < NumRealModules; i++)
	{
		if (debugMaps[i].IsNonEmpty())
		{
			platform->MessageF((MessageType)(mt | PushFlag), " %s(%u - %#" PRIx32 ")", Module(i).ToString(), i, debugMaps[i].GetRaw());
		}
	}

	platform->Message((MessageType)(mt | PushFlag), "\nDebugging disabled for modules:");
	for (size_t i = 0; i < NumRealModules; i++)
	{
		if (debugMaps[i].IsEmpty())
		{
			platform->MessageF((MessageType)(mt | PushFlag), " %s(%u)", Module(i).ToString(), i);
		}
	}
	platform->Message(mt, "\n");
}

void RepRap::Tick() noexcept
{
	// Kicking the watchdog before it has been initialised may trigger it!
	if (active)
	{
		WatchdogReset();														// kick the watchdog

#if SAM4E || SAME70
		WatchdogResetSecondary();												// kick the secondary watchdog
#endif

#if SUPPORT_DIRECT_LCD
		display->Tick();
#endif
		if (!stopped)
		{
			platform->Tick();
			++ticksInSpinState;
			++heatTaskIdleTicks;
			const bool heatTaskStuck = (heatTaskIdleTicks >= MaxHeatTaskTicksInSpinState);
			if (heatTaskStuck || ticksInSpinState >= MaxMainTaskTicksInSpinState)		// if we stall for 20 seconds, save diagnostic data and reset
			{
				stopped = true;
				heat->SwitchOffAllLocalFromISR();								// can't call SwitchOffAll because remote heaters can't be turned off from inside a ISR
				platform->EmergencyDisableDrivers();

				// Save the stack of the stuck task when we get stuck in a spin loop
				const uint32_t *relevantStackPtr;
				const TaskHandle relevantTask = (heatTaskStuck) ? Heat::GetHeatTask() : Tasks::GetMainTask();
				if (relevantTask == RTOSIface::GetCurrentTask())
				{
					__asm volatile("mrs r2, psp");
					register const uint32_t * stackPtr asm ("r2");				// we want the PSP not the MSP
					relevantStackPtr = stackPtr + 5;							// discard uninteresting registers, keep LR PC PSR
				}
				else
				{
					relevantStackPtr = const_cast<const uint32_t*>(pxTaskGetLastStackTop(relevantTask->GetFreeRTOSHandle()));
					// All registers were saved on the stack, so to get useful return addresses we need to skip most of them.
					// See the port.c files in FreeRTOS for the stack layouts
#if SAME70 || SAM4E || SAME5x
					// ARM Cortex M7 with double precision floating point, or ARM Cortex M4F
					if ((relevantStackPtr[8] & 0x10) == 0)						// test EXC_RETURN FP bit
					{
						relevantStackPtr += 9 + 16;								// skip r4-r11 and r14 and s16-s31
					}
					else
					{
						relevantStackPtr += 9;									// skip r4-r11 and r14
					}
#else
					// ARM Cortex M3 or M4 without floating point
					relevantStackPtr += 8;										// skip r4-r11
#endif
				}
				SoftwareReset((heatTaskStuck) ? SoftwareResetReason::heaterWatchdog : SoftwareResetReason::stuckInSpin, relevantStackPtr);
			}
		}
	}
}

// Return true if we are close to timeout
bool RepRap::SpinTimeoutImminent() const noexcept
{
	return ticksInSpinState >= HighMainTaskTicksInSpinState;
}

// Get the JSON status response for the web server or the M408 command.
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
	response->catf(",\"wpl\":%u,", gCodes->GetPrimaryWorkplaceCoordinateSystemNumber());
	AppendFloatArray(response, "xyz", numVisibleAxes, [this](size_t axis) noexcept { return gCodes->GetUserCoordinate(gCodes->GetPrimaryMovementState(), axis); }, 3);

	// Machine coordinates
	const MovementState& ms = gCodes->GetPrimaryMovementState();				// we only report the primary in this response
	response->cat(',');
	AppendFloatArray(response, "machine", numVisibleAxes, [this, &ms](size_t axis) noexcept { return ms.LiveCoordinate(axis); }, 3);

	// Actual extruder positions since power up, last G92 or last M23
	response->cat(',');
	AppendFloatArray(response, "extr", Tool::GetExtrudersInUse(), [this, &ms](size_t extruder) noexcept { return ms.LiveCoordinate(ExtruderToLogicalDrive(extruder)); }, 1);

	// Current speeds
	response->catf("},\"speeds\":{\"requested\":%.1f,\"top\":%.1f}", (double)move->GetRequestedSpeedMmPerSec(), (double)move->GetTopSpeedMmPerSec());

	// Current tool number
	response->catf(",\"currentTool\":%d", ms.GetCurrentToolNumber());

	// Output notifications
	{
		const bool sendBeep = ((source == ResponseSource::AUX || !platform->IsAuxEnabled(0) || platform->IsAuxRaw(0)) && beepDuration != 0 && beepFrequency != 0);
		const bool sendMessage = !message.IsEmpty();

		const ReadLockedPointer<const MessageBox> mbox(MessageBox::GetLockedCurrent());
		const bool mboxActive = mbox.IsNotNull() && mbox->IsLegacyType();
		if (sendBeep || sendMessage || mboxActive)
		{
			response->cat(",\"output\":{");

			// Report beep values
			if (sendBeep)
			{
				response->catf("\"beepDuration\":%u,\"beepFrequency\":%u", beepDuration, beepFrequency);
				if (sendMessage || mboxActive)
				{
					response->cat(',');
				}
			}

			// Report message
			if (sendMessage)
			{
				response->catf("\"message\":\"%.s\"", message.c_str());
				if (mboxActive)
				{
					response->cat(',');
				}
			}

			// Report message box
			if (mboxActive)
			{
				response->catf("\"msgBox\":{\"msg\":\"%.s\",\"title\":\"%.s\",\"mode\":%d,\"seq\":%" PRIu32 ",\"timeout\":%.1f,\"controls\":%u}",
								mbox->GetMessage(), mbox->GetTitle(), mbox->GetMode(), mbox->GetSeq(), (double)mbox->GetTimeLeft(), (unsigned int)mbox->GetControls().GetRaw());
			}
			response->cat('}');
		}
	}

	// ATX power
	response->catf(",\"params\":{\"atxPower\":%d", platform->IsAtxPowerControlled() ? (platform->GetAtxPowerState() ? 1 : 0) : -1);

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
		response->catf(",\"speedFactor\":%.1f,", (double)(gCodes->GetPrimarySpeedFactor() * 100.0));
		AppendFloatArray(response, "extrFactors", Tool::GetExtrudersInUse(), [this](size_t extruder) noexcept { return gCodes->GetExtrusionFactor(extruder) * 100.0; }, 1);

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
		const int32_t v0 = zp->GetReading();
		int32_t v1;
		switch (zp->GetSecondaryValues(v1))
		{
			case 1:
				response->catf("\"probeValue\":%" PRIi32 ",\"probeSecondary\":[%" PRIi32 "],", v0, v1);
				break;
			default:
				response->catf("\"probeValue\":%" PRIi32 ",", v0);
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
			ReadLocker lock(Tool::toolListLock);
			for (const Tool *tool = Tool::GetToolList(); tool != nullptr; tool = tool->Next())
			{
				AppendFloatArray(response, nullptr, tool->HeaterCount(), [tool](unsigned int n) noexcept { return tool->GetToolHeaterActiveTemperature(n); }, 1);
				if (tool->Next() != nullptr)
				{
					response->cat(',');
				}
			}

			response->cat("],\"standby\":[");
			for (const Tool *tool = Tool::GetToolList(); tool != nullptr; tool = tool->Next())
			{
				AppendFloatArray(response, nullptr, tool->HeaterCount(), [tool](unsigned int n) noexcept { return tool->GetToolHeaterStandbyTemperature(n); }, 1);
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
				float temp;
				(void)sensor->GetLatestTemperature(temp);
				response->catf("{\"name\":\"%.s\",\"temp\":%.1f}", nm, (double)HideNan(temp));
			}
			nextSensorNumber = sensor->GetSensorNumber() + 1;
		}

		response->cat("]}");
	}

	// Time since last reset
	response->catf(",\"time\":%.1f", (double)(millis64()/1000u));

	// Spindles
	if (gCodes->GetMachineType() == MachineType::cnc || type == 2)
	{
		size_t numSpindles = MaxSpindles;
		while (numSpindles != 0 && platform->AccessSpindle(numSpindles - 1).GetState() == SpindleState::unconfigured)
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
				response->catf("{\"current\":%" PRIi32 ",\"active\":%" PRIi32 ",\"state\":\"%s\"}", spindle.GetCurrentRpm(), spindle.GetRpm(), spindle.GetState().ToString());
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
			if (platform->GetEndstops().Stopped(axis))
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
		for (size_t i = 0; i < MassStorage::GetNumVolumes(); i++)
		{
			if (MassStorage::IsDriveMounted(i))
			{
				mountedCards |= (1u << i);
			}
		}
		response->catf(",\"volumes\":%u,\"mountedVolumes\":%u", MassStorage::GetNumVolumes(), mountedCards);
#endif

		// Machine mode and name
		response->catf(",\"mode\":\"%.s\",\"name\":\"%.s\"", gCodes->GetMachineModeString(), myName.c_str());

		// Probe trigger threshold, trigger height, type
		{
			const auto zp = platform->GetZProbeOrDefault(0);
			response->catf(",\"probe\":{\"threshold\":%" PRIi32 ",\"height\":%.2f,\"type\":%u}",
							zp->GetTargetAdcValue(), (double)zp->GetConfiguredTriggerHeight(), (unsigned int)zp->GetProbeType());
		}

		// Tool Mapping
		{
			response->cat(",\"tools\":[");
			ReadLocker lock(Tool::toolListLock);
			for (const Tool *tool = Tool::GetToolList(); tool != nullptr; tool = tool->Next())
			{
				// Number
				response->catf("{\"number\":%d,", tool->Number());

				// Name
				const char * const toolName = tool->GetName();
				if (toolName[0] != 0)
				{
					response->catf("\"name\":\"%.s\",", toolName);
				}

				// Heaters
				AppendIntArray(response, "heaters", tool->HeaterCount(), [tool](size_t heater) noexcept { return tool->GetHeater(heater); });

				// Extruder drives
				response->cat(',');
				AppendIntArray(response, "drives", tool->DriveCount(), [tool](size_t drive) noexcept { return tool->GetDrive(drive); });

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
					response->catf(",\"filament\":\"%.s\"", tool->GetFilament()->GetName());
				}

				// Spindle (if configured)
				if (tool->GetSpindleNumber() > -1)
				{
					response->catf(",\"spindle\":%d,\"spindleRpm\":%" PRIi32, tool->GetSpindleNumber(), tool->GetSpindleRpm());
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
			const MinCurMax temps = platform->GetMcuTemperatures();
			response->catf(",\"mcutemp\":{\"min\":%.1f,\"cur\":%.1f,\"max\":%.1f}", (double)temps.minimum, (double)temps.current, (double)temps.maximum);
		}
#endif

#if HAS_VOLTAGE_MONITOR
		// Power in voltages
		{
			const MinCurMax voltages = platform->GetPowerVoltages();
			response->catf(",\"vin\":{\"min\":%.1f,\"cur\":%.1f,\"max\":%.1f}", (double)voltages.minimum, (double)voltages.current, (double)voltages.maximum);
		}
#endif

#if HAS_12V_MONITOR
		// Power in voltages
		{
			const MinCurMax voltages = platform->GetV12Voltages();
			response->catf(",\"v12\":{\"min\":%.1f,\"cur\":%.1f,\"max\":%.1f}", (double)voltages.minimum, (double)voltages.current, (double)voltages.maximum);
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
		AppendFloatArray(response, "extrRaw", Tool::GetExtrudersInUse(), [this](size_t extruder) noexcept { return gCodes->GetRawExtruderTotalByDrive(extruder); }, 1);

		// Fraction of file printed
		response->catf(",\"fractionPrinted\":%.1f", (double)((printMonitor->IsPrinting()) ? (printMonitor->FractionOfFilePrinted() * 100.0) : 0.0));

		// Byte position of the file being printed
		response->catf(",\"filePosition\":%lu", gCodes->GetPrintingFilePosition());

		// First Layer Duration is no longer included

		// First Layer Height is no longer included

		// Print Duration
		response->catf(",\"printDuration\":%.1f", (double)(printMonitor->GetPrintDuration()));

		// Warm-Up Time
		response->catf(",\"warmUpDuration\":%.1f", (double)(printMonitor->GetWarmUpDuration()));

		/* Print Time Estimations */
		response->catf(",\"timesLeft\":{\"file\":%.1f,\"filament\":%.1f}", (double)(printMonitor->EstimateTimeLeft(fileBased)), (double)(printMonitor->EstimateTimeLeft(filamentBased)));
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
	AppendFloatArray(response, "accelerations", MaxAxesPlusExtruders, [this](size_t drive) noexcept { return InverseConvertAcceleration(platform->NormalAcceleration(drive)); }, 2);

	// Motor currents
	response->cat(',');
	AppendIntArray(response, "currents", MaxAxesPlusExtruders, [this](size_t drive) noexcept { return (int)platform->GetMotorCurrent(drive, 906); });

	// Firmware details
	response->catf(",\"firmwareElectronics\":\"%.s", platform->GetElectronicsString());
#ifdef DUET_NG
	const char* expansionName = DuetExpansion::GetExpansionBoardName();
	if (expansionName != nullptr)
	{
		response->catf(" + %.s", expansionName);
	}
	const char* additionalExpansionName = DuetExpansion::GetAdditionalExpansionBoardName();
	if (additionalExpansionName != nullptr)
	{
		response->catf(" + %.s", additionalExpansionName);
	}
#endif
	response->catf("\",\"firmwareName\":\"%.s\",\"firmwareVersion\":\"%.s\"", FIRMWARE_NAME, VERSION);
#ifdef BOARD_SHORT_NAME
	response->catf(",\"boardName\":\"%.s\"", BOARD_SHORT_NAME);
#endif

#if HAS_WIFI_NETWORKING
	// If we have WiFi networking, send the WiFi module firmware version
# ifdef DUET_NG
	if (platform->IsDuetWiFi())
# endif
	{
		response->catf(",\"dwsVersion\":\"%.s\"", network->GetWiFiServerVersion());
	}
#endif

	response->catf(",\"firmwareDate\":\"%.s\"", DATE);

#if HAS_MASS_STORAGE || HAS_EMBEDDED_FILES
	// System files folder
	response->catf(", \"sysdir\":\"%.s\"", platform->GetSysDir().Ptr());
#endif

	// Motor idle parameters
	response->catf(",\"idleCurrentFactor\":%.1f", (double)(platform->GetIdleCurrentFactor() * 100.0));
	response->catf(",\"idleTimeout\":%.1f,", (double)(move->IdleTimeout()));

	// Minimum feedrates
	AppendFloatArray(response, "minFeedrates", MaxAxesPlusExtruders, [this](size_t drive) noexcept { return InverseConvertSpeedToMmPerSec(platform->GetInstantDv(drive)); }, 2);

	// Maximum feedrates
	response->cat(',');
	AppendFloatArray(response, "maxFeedrates", MaxAxesPlusExtruders, [this](size_t drive) noexcept { return InverseConvertSpeedToMmPerSec(platform->MaxFeedrate(drive)); }, 2);

	// Config file is no longer included, because we can use rr_configfile or M503 instead
	response->cat('}');

	return response;
}

// Get the JSON status response for PanelDue
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
	for (size_t heater = DefaultE0Heater; heater < Tool::GetToolHeatersInUse(); heater++)
	{
		response->catf("%c%.1f", ch, (double)(heat->GetHeaterTemperature(heater)));
		ch = ',';
	}
	response->cat((ch == '[') ? "[]" : "]");

	// Send the heater active temperatures
	response->catf(",\"active\":[%.1f", (double)((bedHeater == -1) ? 0.0 : heat->GetActiveTemperature(bedHeater)));
	for (size_t heater = DefaultE0Heater; heater < Tool::GetToolHeatersInUse(); heater++)
	{
		response->catf(",%.1f", (double)(heat->GetActiveTemperature(heater)));
	}
	response->cat(']');

	// Send the heater standby temperatures
	response->catf(",\"standby\":[%.1f", (double)((bedHeater == -1) ? 0.0 : heat->GetStandbyTemperature(bedHeater)));
	for (size_t heater = DefaultE0Heater; heater < Tool::GetToolHeatersInUse(); heater++)
	{
		response->catf(",%.1f", (double)(heat->GetStandbyTemperature(heater)));
	}
	response->cat(']');

	// Send the heater statuses (0=off, 1=standby, 2=active, 3 = fault)
	response->catf(",\"hstat\":[%u", (bedHeater == -1) ? 0 : heat->GetStatus(bedHeater).ToBaseType());
	for (size_t heater = DefaultE0Heater; heater < Tool::GetToolHeatersInUse(); heater++)
	{
		response->catf(",%u", heat->GetStatus(heater).ToBaseType());
	}
	response->cat("],");

	// User coordinates
	const size_t numVisibleAxes = gCodes->GetVisibleAxes();
	const MovementState& ms = gCodes->GetPrimaryMovementState();
	AppendFloatArray(response, "pos", numVisibleAxes, [this, &ms](size_t axis) noexcept { return gCodes->GetUserCoordinate(ms, axis); }, 3);

	// Machine coordinates
	response->cat(',');
	AppendFloatArray(response, "machine", numVisibleAxes, [this, &ms](size_t axis) noexcept { return ms.LiveCoordinate(axis); }, 3);

	// Send the speed and extruder override factors
	response->catf(",\"sfactor\":%.1f,", (double)(gCodes->GetPrimarySpeedFactor() * 100.0));
	AppendFloatArray(response, "efactor", Tool::GetExtrudersInUse(), [this](size_t extruder) noexcept { return gCodes->GetExtrusionFactor(extruder) * 100.0; }, 1);

	// Send the baby stepping offset
	response->catf(",\"babystep\":%.03f", (double)(gCodes->GetTotalBabyStepOffset(Z_AXIS)));

	// Send the current tool number
	response->catf(",\"tool\":%d", gCodes->GetPrimaryMovementState().GetCurrentToolNumber());

	// Send the Z probe value
	const auto zp = platform->GetZProbeOrDefault(0);
	const int32_t v0 = zp->GetReading();
	int32_t v1;
	switch (zp->GetSecondaryValues(v1))
	{
	case 1:
		response->catf(",\"probe\":\"%" PRIi32 " (%" PRIi32 ")\"", v0, v1);
		break;
	default:
		response->catf(",\"probe\":\"%" PRIi32 "\"", v0);
		break;
	}

	// Send the fan settings, for PanelDue firmware 1.13 and later
	// Currently, PanelDue assumes that the first value is the print cooling fan speed and only uses that one, so send the mapped fan speed first
	response->catf(",\"fanPercent\":[%.1f", (double)(gCodes->GetPrimaryMovementState().virtualFanSpeed * 100.0));
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
		const ReadLockedPointer<const MessageBox> mbox(MessageBox::GetLockedCurrent());
		if (mbox.IsNotNull() && mbox->IsLegacyType())
		{
			response->catf(",\"msgBox.mode\":%d,\"msgBox.seq\":%" PRIu32 ",\"msgBox.timeout\":%.1f,\"msgBox.controls\":%u,\"msgBox.msg\":\"%.s\",\"msgBox.title\":\"%.s\"",
				mbox->GetMode(), mbox->GetSeq(), (double)mbox->GetTimeLeft(), (unsigned int)mbox->GetControls().GetRaw(), mbox->GetMessage(), mbox->GetTitle());
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
			response->catf(",\"timesLeft\":[%.1f,%.1f,0.0]",
					(double)(printMonitor->EstimateTimeLeft(fileBased)),
					(double)(printMonitor->EstimateTimeLeft(filamentBased)));
		}
	}
	else if (type == 3)
	{
		// Add the static fields
		response->catf(",\"geometry\":\"%s\",\"axes\":%u,\"totalAxes\":%u,\"axisNames\":\"%s\",\"volumes\":%u,\"numTools\":%u,\"myName\":\"%.s\",\"firmwareName\":\"%.s\"",
						move->GetGeometryString(), numVisibleAxes, gCodes->GetTotalAxes(), gCodes->GetAxisLetters(),
#if HAS_MASS_STORAGE || HAS_EMBEDDED_FILES
							MassStorage::GetNumVolumes(),
#else
							0,
#endif
								Tool::GetNumberOfContiguousTools(), myName.c_str(), FIRMWARE_NAME);
	}

	response->cat("}\n");			// include a newline to help PanelDue resync
	return response;
}

#if HAS_MASS_STORAGE || HAS_EMBEDDED_FILES

// Get the list of files in the specified directory in JSON format. PanelDue uses this one, so include a newline at the end.
// If flagDirs is true then we prefix each directory with a * character.
OutputBuffer *RepRap::GetFilesResponse(const char *dir, unsigned int startAt, bool flagsDirs) noexcept
{
	// Need something to write to...
	OutputBuffer *response;
	if (!OutputBuffer::Allocate(response))
	{
		return nullptr;
	}

	response->printf("{\"dir\":\"%.s\",\"first\":%u,\"files\":[", dir, startAt);
	unsigned int err;
	unsigned int nextFile = 0;

#if HAS_MASS_STORAGE
	if (!MassStorage::CheckDriveMounted(dir))
	{
		err = 1;
	}
	else
#endif
	if (!MassStorage::DirectoryExists(dir))
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

					bytesLeft -= response->catf((flagsDirs && fileInfo.isDirectory) ? "\"*%.s\"" : "\"%.s\"", fileInfo.fileName.c_str());
				}
				++filesFound;
			}
			gotFile = MassStorage::FindNext(fileInfo);
		}
	}

	if (err != 0)
	{
		response->catf("],\"err\":%u}\n", err);
	}
	else
	{
		response->catf("],\"next\":%u,\"err\":%u}\n", nextFile, err);
	}

	if (response->HadOverflow())
	{
		OutputBuffer::ReleaseAll(response);
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

	response->printf("{\"dir\":\"%.s\",\"first\":%u,\"files\":[", dir, startAt);
	unsigned int err;
	unsigned int nextFile = 0;

#if HAS_MASS_STORAGE
	if (!MassStorage::CheckDriveMounted(dir))
	{
		err = 1;
	}
	else
#endif
	if (!MassStorage::DirectoryExists(dir))
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
					bytesLeft -= response->catf("{\"type\":\"%c\",\"name\":\"%.s\",\"size\":%" PRIu32,
												fileInfo.isDirectory ? 'd' : 'f', fileInfo.fileName.c_str(), fileInfo.size);
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
		response->catf("],\"err\":%u}\n", err);
	}
	else
	{
		response->catf("],\"next\":%u}\n", nextFile);
	}

	if (response->HadOverflow())
	{
		OutputBuffer::ReleaseAll(response);
	}
	return response;
}

#endif

#if HAS_MASS_STORAGE

// Get thumbnail data
// 'offset' is the offset into the file of the thumbnail data that the caller wants.
// It is up to the caller to get the offset right, however we must fail gracefully if the caller passes us a bad offset.
// The offset should always be either the initial offset or the 'next' value passed in a previous call, so it should always be the start of a line.
OutputBuffer *RepRap::GetThumbnailResponse(const char *filename, FilePosition offset, bool forM31point1) noexcept
{
	constexpr unsigned int ThumbnailMaxDataSizeM31 = 1024;			// small enough for PanelDue to buffer
	constexpr unsigned int ThumbnailMaxDataSizeRr = 2600;			// about two TCP messages
	static_assert(ThumbnailMaxDataSizeM31 % 4 == 0, "must be a multiple of to guarantee base64 alignment");
	static_assert(ThumbnailMaxDataSizeRr % 4 == 0, "must be a multiple of to guarantee base64 alignment");

	// Need something to write to...
	OutputBuffer *response;
	if (!OutputBuffer::Allocate(response))
	{
		return nullptr;
	}

	if (forM31point1)
	{
		response->cat("{\"thumbnail\":");
	}
	response->catf("{\"fileName\":\"%.s\",\"offset\":%" PRIu32 ",", filename, offset);

	FileStore *const f = platform->OpenFile(Platform::GetGCodeDir(), filename, OpenMode::read);
	unsigned int err = 0;
	if (f != nullptr)
	{
		if (f->Seek(offset))
		{
			response->cat("\"data\":\"");

			const unsigned int thumbnailMaxDataSize = (forM31point1) ? ThumbnailMaxDataSizeM31 : ThumbnailMaxDataSizeRr;
			for (unsigned int charsWrittenThisCall = 0; charsWrittenThisCall < thumbnailMaxDataSize; )
			{
				// Read a line
				char lineBuffer[MaxGCodeLength];
				const int charsRead = f->ReadLine(lineBuffer, sizeof(lineBuffer));
				if (charsRead <= 0)
				{
					err = 1;
					offset = 0;
					break;
				}

				const FilePosition posOld = offset;
				offset = f->Position();

				const char *p = lineBuffer;

				// Skip white spaces
				while ((p - lineBuffer <= charsRead) && (*p == ';' || *p == ' ' || *p == '\t'))
				{
					++p;
				}

				// Skip empty lines (there shouldn't be any, but just in case there are)
				if (*p == '\n' || *p == '\0')
				{
					continue;
				}

				// Check for end of thumbnail. We'd like to use a regex here but we can't afford the flash space of a regex parser in some build configurations.
				if (   StringStartsWith(p, "thumbnail end") || StringStartsWith(p, "thumbnail_QOI end") || StringStartsWith(p, "thumbnail_JPG end")
					// Also stop if the base64 data has ended, to avoid sending to the end of file if the end marker is missing. We don't want to take too long so just look for space.
					|| strchr(p, ' ') != nullptr
				   )
				{
					offset = 0;
					break;
				}

				const unsigned int charsSkipped = p - lineBuffer;
				const unsigned int charsAvailable = charsRead - charsSkipped;
				unsigned int charsWrittenFromThisLine;
				if (charsAvailable <= thumbnailMaxDataSize - charsWrittenThisCall)
				{
					// Write all the data in this line
					charsWrittenFromThisLine = charsAvailable;
				}
				else
				{
					// Write just enough characters to fill the buffer
					charsWrittenFromThisLine = thumbnailMaxDataSize - charsWrittenThisCall;
					offset = posOld + charsSkipped + charsWrittenFromThisLine;
				}

				// Copy the data
				response->cat(p, charsWrittenFromThisLine);
				charsWrittenThisCall += charsWrittenFromThisLine;
			}

			response->catf("\",\"next\":%" PRIu32 ",", offset);
		}
		f->Close();
	}
	else
	{
		err = 1;
	}

	response->catf(forM31point1 ? "\"err\":%u}}\n" : "\"err\":%u}\n", err);
	return response;
}

#endif

// Get information for the specified file, or the currently printing file (if 'filename' is null or empty), in JSON format
// Return GCodeResult::Warning if the file doesn't exist, else GCodeResult::ok or GCodeResult::notFinished
GCodeResult RepRap::GetFileInfoResponse(const char *filename, OutputBuffer *&response, bool quitEarly) noexcept
{
	const bool specificFile = (filename != nullptr && filename[0] != 0);
	GCodeFileInfo info;
	if (specificFile)
	{
#if HAS_MASS_STORAGE || HAS_EMBEDDED_FILES
		// Poll file info for a specific file
		String<MaxFilenameLength> filePath;
		if (!MassStorage::CombineName(filePath.GetRef(), Platform::GetGCodeDir(), filename))
		{
			info.isValid = false;
		}
		else if (MassStorage::GetFileInfo(filePath.c_str(), info, quitEarly) == GCodeResult::notFinished)
		{
			// This may take a few runs...
			return GCodeResult::notFinished;
		}
#else
		return GCodeResult::warning;
#endif
	}
	else if (!printMonitor->GetPrintingFileInfo(info))
	{
		return GCodeResult::notFinished;
	}

	if (!OutputBuffer::Allocate(response))
	{
		return GCodeResult::notFinished;
	}

	if (info.isValid)
	{
		response->printf("{\"err\":0,\"fileName\":\"%.s\",\"size\":%lu,", ((specificFile) ? filename : printMonitor->GetPrintingFilename()), info.fileSize);
		tm timeInfo;
		gmtime_r(&info.lastModifiedTime, &timeInfo);
		if (timeInfo.tm_year > /*19*/80)
		{
			response->catf("\"lastModified\":\"%04u-%02u-%02uT%02u:%02u:%02u\",",
					timeInfo.tm_year + 1900, timeInfo.tm_mon + 1, timeInfo.tm_mday, timeInfo.tm_hour, timeInfo.tm_min, timeInfo.tm_sec);
		}

		response->catf("\"height\":%.2f,\"layerHeight\":%.2f,\"numLayers\":%u,", (double)info.objectHeight, (double)info.layerHeight, info.numLayers);
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
		response->cat(']');

		if (!specificFile)
		{
			response->catf(",\"printDuration\":%d", (int)printMonitor->GetPrintDuration());
		}

		// See if we have any thumbnails
		response->cat(",\"thumbnails\":");
		if (info.thumbnails[0].IsValid())
		{
			size_t index = 0;
			do
			{
				const GCodeFileInfo::ThumbnailInfo& inf = info.thumbnails[index];
				response->catf("%c{\"width\":%u,\"height\":%u,\"format\":\"%s\",\"offset\":%" PRIu32 ",\"size\":%" PRIu32 "}",
								((index == 0) ? '[' : ','), inf.width, inf.height, inf.format.ToString(), inf.offset, inf.size);
				++index;
			}
			while (index < MaxThumbnails && info.thumbnails[index].IsValid());
		}
		else
		{
			response->cat('[');
		}
		response->cat(']');

		response->catf(",\"generatedBy\":\"%.s\"}\n", info.generatedBy.c_str());
		return GCodeResult::ok;
	}

	response->printf("{\"err\":1,\"fileName\":\"%.s\"}", ((specificFile) ? filename : printMonitor->GetPrintingFilename()));
	return GCodeResult::warning;
}

// Helper functions to write JSON arrays
// Append float array using the specified number of decimal places
void RepRap::AppendFloatArray(OutputBuffer *buf, const char *name, size_t numValues, function_ref_noexcept<float(size_t) noexcept> func, unsigned int numDecimalDigits) noexcept
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
		const float fVal = HideNan(func(i));
		buf->catf(GetFloatFormatString(fVal, numDecimalDigits), (double)fVal);
	}
	buf->cat(']');
}

void RepRap::AppendIntArray(OutputBuffer *buf, const char *name, size_t numValues, function_ref_noexcept<int(size_t) noexcept> func) noexcept
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

void RepRap::AppendStringArray(OutputBuffer *buf, const char *name, size_t numValues, function_ref_noexcept<const char *(size_t) noexcept> func) noexcept
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
		buf->catf("\"%.s\"", func(i));
	}
	buf->cat(']');
}

// Return a query into the object model, or return nullptr if no buffer available
// We append a newline to help PanelDue resync after receiving corrupt or incomplete data. DWC ignores it.
OutputBuffer *RepRap::GetModelResponse(const GCodeBuffer *_ecv_null gb, const char *key, const char *flags) const THROWS(GCodeException)
{
	OutputBuffer *outBuf;
	if (OutputBuffer::Allocate(outBuf))
	{
		if (key == nullptr) { key = ""; }
		if (flags == nullptr) { flags = ""; }

		outBuf->printf("{\"key\":\"%.s\",\"flags\":\"%.s\",\"result\":", key, flags);

		const bool wantArrayLength = (*key == '#');
		if (wantArrayLength)
		{
			++key;
		}

		try
		{
			reprap.ReportAsJson(gb, outBuf, key, flags, wantArrayLength);
			outBuf->cat("}\n");
			if (outBuf->HadOverflow())
			{
				OutputBuffer::ReleaseAll(outBuf);
			}
		}
		catch (...)
		{
			OutputBuffer::ReleaseAll(outBuf);
			throw;
		}
	}

	return outBuf;
}

// Send a beep. We send it to both PanelDue and the web interface.
void RepRap::Beep(unsigned int freq, unsigned int ms) noexcept
{
	// Limit the frequency and duration to sensible values
	freq = constrain<unsigned int>(freq, 50, 10000);
	ms = constrain<unsigned int>(ms, 10, 60000);

	// If there is an LCD device present, make it beep
	bool bleeped = false;
#if SUPPORT_DIRECT_LCD
	if (display->IsPresent())
	{
		display->Beep(freq, ms);
		bleeped = true;
	}
#endif

	if (platform->IsAuxEnabled(0) && !platform->IsAuxRaw(0))
	{
		platform->PanelDueBeep(freq, ms);
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
#if SUPPORT_DIRECT_LCD
	++messageSequence;
#endif
	StateUpdated();

	if (platform->IsAuxEnabled(0) && !platform->IsAuxRaw(0))
	{
		platform->SendPanelDueMessage(0, msg);
	}
	platform->Message(MessageType::LogInfo, msg);
}

// Get the status index
size_t RepRap::GetStatusIndex() const noexcept
{
	return    (processingConfig)										? 0		// Reading the configuration file
#if HAS_SBC_INTERFACE && SUPPORT_CAN_EXPANSION
			: (gCodes->IsFlashing() || expansion->IsFlashing())			? 1		// Flashing a new firmware binary
#else
			: (gCodes->IsFlashing())									? 1		// Flashing a new firmware binary
#endif
			: (IsStopped()) 											? 2		// Halted
#if HAS_VOLTAGE_MONITOR
			: (!platform->HasVinPower() && !gCodes->IsSimulating())		? 3		// Off i.e. powered down
#endif
			: (gCodes->GetPauseState() == PauseState::pausing)			? 4		// Pausing
			: (gCodes->GetPauseState() == PauseState::resuming)			? 5		// Resuming
			: (gCodes->GetPauseState() == PauseState::paused)			? 6		// Paused
			: (gCodes->IsCancellingPrint())								? 7		// Cancelling
			: (printMonitor->IsPrinting())
			  	  ? ((gCodes->IsSimulating())							? 8		// Simulating
			  		  : 												  9		// Printing
			  	  	)
			: (gCodes->IsDoingToolChange())								? 10	// Changing tool
			: (gCodes->DoingFileMacro() || !move->NoLiveMovement() ||
			   gCodes->WaitingForAcknowledgement()) 					? 11	// Busy
			:															  12;	// Idle

}

// Get the status character for the new-style status response
char RepRap::GetStatusCharacter() const noexcept
{
	return "CFHODRSAMPTBI"[GetStatusIndex()];
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
		"cancelling",
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

// Firmware update operations

// Check the prerequisites for updating the main firmware. Return True if satisfied, else print a message to 'reply' and return false.
bool RepRap::CheckFirmwareUpdatePrerequisites(const StringRef& reply, const StringRef& filenameRef) noexcept
{
#if HAS_MASS_STORAGE
	FileStore * const firmwareFile = platform->OpenFile(FIRMWARE_DIRECTORY, filenameRef.IsEmpty() ? IAP_FIRMWARE_FILE : filenameRef.c_str(), OpenMode::read);
	if (firmwareFile == nullptr)
	{
		String<MaxFilenameLength> firmwareBinaryLocation;
		MassStorage::CombineName(firmwareBinaryLocation.GetRef(), FIRMWARE_DIRECTORY, filenameRef.IsEmpty() ? IAP_FIRMWARE_FILE : filenameRef.c_str());
		reply.printf("Firmware binary \"%s\" not found", firmwareBinaryLocation.c_str());
		return false;
	}
#if SAME5x
	// UF2 files consist of 512 byte blocks with (for the SAME5x) 256 bytes of data per block
	if ((firmwareFile->Length() / 512) * 256 > FLASH_SIZE)
#else
	if (firmwareFile->Length() > IFLASH_SIZE)
#endif
	{
		firmwareFile->Close();
		reply.printf("Firmware binary \"%s\" is too big for the available flash memory", filenameRef.c_str());
		return false;
	}

	// Check that the binary looks sensible. The first word is the initial stack pointer, which should be the top of RAM.
	uint32_t firstDword;
	bool ok =
#if SAME5x
		// We use UF2 file format, so look inside the payload
		firmwareFile->Seek(32) &&
#endif

	firmwareFile->Read(reinterpret_cast<char*>(&firstDword), sizeof(firstDword)) == (int)sizeof(firstDword);
	firmwareFile->Close();
	if (!ok || firstDword !=
#if SAME5x
						HSRAM_ADDR + HSRAM_SIZE
#else
						IRAM_ADDR + IRAM_SIZE
#endif
			)
	{
		reply.printf("Firmware binary \"%s\" is not valid for this electronics", FIRMWARE_DIRECTORY IAP_FIRMWARE_FILE);
		return false;
	}

	if (!platform->FileExists(FIRMWARE_DIRECTORY, IAP_UPDATE_FILE) && !platform->FileExists(DEFAULT_SYS_DIR, IAP_UPDATE_FILE))
	{
		reply.printf("In-application programming binary \"%s\" not found", FIRMWARE_DIRECTORY IAP_UPDATE_FILE);
		return false;
	}
#endif

	return true;
}

#if HAS_MASS_STORAGE

// Update the firmware. Prerequisites should be checked before calling this.
void RepRap::UpdateFirmware(const char *iapFilename, const char *iapParam) noexcept
{
	FileStore * iapFile = platform->OpenFile(FIRMWARE_DIRECTORY, iapFilename, OpenMode::read);
	if (iapFile == nullptr)
	{
		iapFile = platform->OpenFile(DEFAULT_SYS_DIR, iapFilename, OpenMode::read);
		if (iapFile == nullptr)
		{
			// This should not happen because we already checked that the file exists, so use a simplified error message
			platform->Message(FirmwareUpdateMessage, "Missing IAP");
			return;
		}
	}

	PrepareToLoadIap();

	// Use RAM-based IAP
	iapFile->Read(reinterpret_cast<char *>(IAP_IMAGE_START), iapFile->Length());
	iapFile->Close();
	StartIap(iapParam);
}

#endif

void RepRap::PrepareToLoadIap() noexcept
{
#if SUPPORT_DIRECT_LCD
	display->UpdatingFirmware();			// put the firmware update message on the display and stop polling the display
#endif

	// Send this message before we start using RAM that may contain message buffers
	platform->Message(AuxMessage, "Updating main firmware\n");
	platform->Message(UsbMessage, "Shutting down USB interface to update main firmware. Try reconnecting after 30 seconds.\n");

	// Allow time for the firmware update message to be sent
	const uint32_t now = millis();
	while (platform->FlushMessages() && millis() - now < 2000) { }

	// The machine will be unresponsive for a few seconds, don't risk damaging the heaters.
	// This also shuts down tasks and interrupts that might make use of the RAM that we are about to load the IAP binary into.
	EmergencyStop();						// this also stops CAN and stops Platform::Tick being called, which is necessary because it may access a Z probe object in RAM which will be overwritten by the IAP
	network->Exit();						// kill the network task to stop it overwriting RAM that we use to hold the IAP
#if HAS_SMART_DRIVERS
	SmartDrivers::Exit();					// stop the drivers being polled via SPI or UART because it may use data in the last 64Kb of RAM
#endif
	FilamentMonitor::Exit();				// stop the filament monitors generating interrupts, we may be about to overwrite them
	fansManager->Exit();					// stop the fan tachos generating interrupts, we may be about to overwrite them
#if SUPPORT_ACCELEROMETERS
	Accelerometers::Exit();					// terminate the accelerometer task, if any
#endif
	if (RTOSIface::GetCurrentTask() != Tasks::GetMainTask())
	{
		Tasks::TerminateMainTask();			// stop the main task if IAP is being written from another task
	}

#ifdef DUET_NG
	DuetExpansion::Exit();					// stop the DueX polling task
#endif
	StopAnalogTask();
	serialUSB.end();
	StopUsbTask();

	Cache::Disable();						// disable the cache because it interferes with flash memory access

#if USE_MPU
	ARM_MPU_Disable();						// make sure we can execute from RAM
#endif

#if 0	// this code doesn't work, it causes a watchdog reset
	// Debug
	memset(reinterpret_cast<char *>(IAP_IMAGE_START), 0x7E, 20 * 1024);
	delay(2000);
	for (char* p = reinterpret_cast<char *>(IAP_IMAGE_START); p < reinterpret_cast<char *>(IAP_IMAGE_START + (20 * 1024)); ++p)
	{
		if (*p != 0x7E)
		{
			SERIAL_AUX_DEVICE.printf("At %08" PRIx32 ": %02x\n", reinterpret_cast<uint32_t>(p), *p);
		}
	}
	SERIAL_AUX_DEVICE.printf("Scan complete\n");
	delay(1000);							// give it time to send the message
#endif
}

void RepRap::StartIap(const char *filename) noexcept
{
	// Disable all interrupts, then reallocate the vector table and program entry point to the new IAP binary
	// This does essentially what the Atmel AT02333 paper suggests (see 3.2.2 ff)

	// Disable all IRQs
	SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk;	// disable the system tick exception
	IrqDisable();
	for (size_t i = 0; i < 8; i++)
	{
		NVIC->ICER[i] = 0xFFFFFFFF;					// Disable IRQs
		NVIC->ICPR[i] = 0xFFFFFFFF;					// Clear pending IRQs
	}

	// Disable all PIO IRQs, because the core assumes they are all disabled when setting them up
#if !SAME5x
	PIOA->PIO_IDR = 0xFFFFFFFF;
	PIOB->PIO_IDR = 0xFFFFFFFF;
	PIOC->PIO_IDR = 0xFFFFFFFF;
# ifdef PIOD
	PIOD->PIO_IDR = 0xFFFFFFFF;
# endif
# ifdef ID_PIOE
	PIOE->PIO_IDR = 0xFFFFFFFF;
# endif
#endif

#if HAS_MASS_STORAGE
	if (filename != nullptr)
	{
		// Newer versions of IAP reserve space above the stack for us to pass the firmware filename
		String<MaxFilenameLength> firmwareFileLocation;
		MassStorage::CombineName(firmwareFileLocation.GetRef(), FIRMWARE_DIRECTORY, filename[0] == 0 ? IAP_FIRMWARE_FILE : filename);
		const uint32_t topOfStack = *reinterpret_cast<uint32_t *>(IAP_IMAGE_START);
		if (topOfStack + firmwareFileLocation.strlen() + 1 <=
# if SAME5x
						HSRAM_ADDR + HSRAM_SIZE
# else
						IRAM_ADDR + IRAM_SIZE
# endif
		   )
		{
			strcpy(reinterpret_cast<char*>(topOfStack), firmwareFileLocation.c_str());
		}
	}
#endif

#if defined(DUET_NG) || defined(DUET_M)
	IoPort::WriteDigital(DiagPin, !DiagOnPolarity);	// turn the DIAG LED off
#endif

	WatchdogReset();								// kick the watchdog one last time

#if SAM4E || SAME70
	WatchdogResetSecondary();						// kick the secondary watchdog
#endif

	// Modify vector table location
	__DSB();
	__ISB();
	SCB->VTOR = ((uint32_t)IAP_IMAGE_START & SCB_VTOR_TBLOFF_Msk);
	__DSB();
	__ISB();

	IrqEnable();

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
	for (;;) { }							// to keep gcc happy
}

// Helper function for diagnostic tests in Platform.cpp, to cause a deliberate divide-by-zero
/*static*/ uint32_t RepRap::DoDivide(uint32_t a, uint32_t b) noexcept
{
	return a/b;
}

// Helper function for diagnostic tests in Platform.cpp, to cause a deliberate bus fault or memory protection error
/*static*/ void RepRap::GenerateBusFault() noexcept
{
#if SAME5x
	(void)*(reinterpret_cast<const volatile char*>(0x30000000));
#elif SAME70
	(void)*(reinterpret_cast<const volatile char*>(0x30000000));
#elif SAM4E || SAM4S
	(void)*(reinterpret_cast<const volatile char*>(0x20800000));
#elif SAM3XA
	(void)*(reinterpret_cast<const volatile char*>(0x20200000));
#else
# error Unsupported processor
#endif
}

// Helper function for diagnostic tests in Platform.cpp, to calculate sine and cosine
/*static*/ float RepRap::SinfCosf(float angle) noexcept
{
	return sinf(angle) + cosf(angle);
}

// Helper function for diagnostic tests in Platform.cpp, to calculate square root
/*static*/ float RepRap::FastSqrtf(float f) noexcept
{
	return ::fastSqrtf(f);
}

// Report an internal error
void RepRap::ReportInternalError(const char *file, const char *func, int line) const noexcept
{
	platform->MessageF(ErrorMessage, "Internal Error in %s at %s(%d)\n", func, file, line);
}

// Message box functions

// Send a message box, which may require an acknowledgement
// sParam = 0 Just display the message box, optional timeout
// sParam = 1 As for 0 but display a Close button as well
// sParam = 2 Display the message box with an OK button, wait for acknowledgement (waiting is set up by the caller)
// sParam = 3 As for 2 but also display a Cancel button
// Returns the message box sequence number
uint32_t RepRap::SendAlert(MessageType mt, const char *_ecv_array message, const char *_ecv_array title, int sParam, float tParam, AxesBitmap controls, MessageBoxLimits *_ecv_null limits) noexcept
{
	WriteLocker lock(MessageBox::mboxLock);

	uint32_t seq;
	if ((mt & (HttpMessage | AuxMessage | LcdMessage | BinaryCodeReplyFlag)) != 0)
	{
		seq = MessageBox::Create(message, title, sParam, tParam, controls, limits);
		StateUpdated();
	}
	else
	{
		seq = 0;
	}

	platform->MessageF(MessageType::LogInfo, "M291: - %s - %s", (strlen(title) > 0 ? title : "[no title]"), message);

	mt = (MessageType)(mt & (UsbMessage | TelnetMessage | Aux2Message));
	if (mt != 0)
	{
		if (strlen(title) > 0)
		{
			platform->MessageF(mt, "- %s -\n", title);
		}
		platform->MessageF(mt, "%s\n", message);
		if (sParam == 2)
		{
			platform->Message(mt, "Send M292 to continue\n");
		}
		else if (sParam == 3)
		{
			platform->Message(mt, "Send M292 to continue or M292 P1 to cancel\n");
		}
	}
	return seq;
}

void RepRap::SendSimpleAlert(MessageType mt, const char *_ecv_array message, const char *_ecv_array title) noexcept
{
	(void)SendAlert(mt, message, title, 1, 0.0, AxesBitmap());
}

// Save the first error message generated while running config.g
void RepRap::SaveConfigError(const char *filename, unsigned int lineNumber, const char *errorMessage) noexcept
{
	if (configErrorMessage.IsNull())
	{
		configErrorLine = lineNumber;
		configErrorFilename.Assign(filename);
		configErrorMessage.Assign(errorMessage);
		StateUpdated();
	}
}

#if SUPPORT_DIRECT_LCD

const char *RepRap::GetLatestMessage(uint16_t& sequence) const noexcept
{
	sequence = messageSequence;
	return message.c_str();
}

#endif

// End
