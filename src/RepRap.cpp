#include "RepRap.h"

#include "Movement/Move.h"
#include "Movement/StepTimer.h"
#include "FilamentMonitors/FilamentMonitor.h"
#include "GCodes/GCodes.h"
#include "Heating/Heat.h"
#include "Heating/Sensors/TemperatureSensor.h"
#include "Network.h"
#include "Platform.h"
#include "Scanner.h"
#include "PrintMonitor.h"
#include "Tools/Tool.h"
#include "Tools/Filament.h"
#include "Endstops/ZProbe.h"
#include "Tasks.h"
#include "Hardware/Cache.h"
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
# include "CAN/CanInterface.h"
#endif

#include "FreeRTOS.h"
#include "task.h"

#if SAME70
# include "Hardware/DmacManager.h"
#endif

// We call vTaskNotifyGiveFromISR from various interrupts, so the following must be true
static_assert(configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY <= NvicPriorityHSMCI, "configMAX_SYSCALL_INTERRUPT_PRIORITY is set too high");

static TaskHandle_t hsmciTask = nullptr;		// the task that is waiting for a HSMCI command to complete

// HSMCI interrupt handler
extern "C" void HSMCI_Handler() noexcept
{
	HSMCI->HSMCI_IDR = 0xFFFFFFFF;										// disable all HSMCI interrupts
#if SAME70
	XDMAC->XDMAC_CHID[DmacChanHsmci].XDMAC_CID = 0xFFFFFFFF;			// disable all DMA interrupts for this channel
#endif
	if (hsmciTask != nullptr)
	{
		BaseType_t higherPriorityTaskWoken = pdFALSE;
		vTaskNotifyGiveFromISR(hsmciTask, &higherPriorityTaskWoken);	// wake up the task
		hsmciTask = nullptr;
		portYIELD_FROM_ISR(higherPriorityTaskWoken);
	}
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
		hsmciTask = xTaskGetCurrentTaskHandle();
		HSMCI->HSMCI_IER = stBits;
#if SAME70
		DmacManager::SetInterruptCallback(DmacChanHsmci, HsmciDmaCallback, CallbackParameter());
		XDMAC->XDMAC_CHID[DmacChanHsmci].XDMAC_CIE = dmaBits;
		XDMAC->XDMAC_GIE = 1u << DmacChanHsmci;
#endif
		if (ulTaskNotifyTake(pdTRUE, 200) == 0)
		{
			// We timed out waiting for the HSMCI operation to complete
			reprap.GetPlatform().LogError(ErrorCode::HsmciTimeout);
		}
	}
}

#if SUPPORT_OBJECT_MODEL

// Object model table and functions
// Note: if using GCC version 7.3.1 20180622 and lambda functions are used in this table, you must compile this file with option -std=gnu++17.
// Otherwise the table will be allocate in RAM instead of flash, which wastes too much RAM.

// Macro to build a standard lambda function that includes the necessary type conversions
#define OBJECT_MODEL_FUNC(_ret) OBJECT_MODEL_FUNC_BODY(RepRap, _ret)

const ObjectModelTableEntry RepRap::objectModelTable[] =
{
	// These entries are temporary pending design of the object model
	//TODO design the object model
	{ "gcodes", OBJECT_MODEL_FUNC(&(self->GetGCodes())), TYPE_OF(ObjectModel), ObjectModelTableEntry::none },
	{ "heat", OBJECT_MODEL_FUNC(&(self->GetHeat())), TYPE_OF(ObjectModel), ObjectModelTableEntry::none },
	{ "meshProbe", OBJECT_MODEL_FUNC(&(self->GetMove().GetGrid())), TYPE_OF(ObjectModel), ObjectModelTableEntry::none },
	{ "move", OBJECT_MODEL_FUNC(&(self->GetMove())), TYPE_OF(ObjectModel), ObjectModelTableEntry::none },
	{ "network", OBJECT_MODEL_FUNC(&(self->GetNetwork())), TYPE_OF(ObjectModel), ObjectModelTableEntry::none },
	{ "randomProbe", OBJECT_MODEL_FUNC(&(self->GetMove().GetProbePoints())), TYPE_OF(ObjectModel), ObjectModelTableEntry::none },
};

DEFINE_GET_OBJECT_MODEL_TABLE(RepRap)

#endif

// RepRap member functions.

// Do nothing more in the constructor; put what you want in RepRap:Init()

RepRap::RepRap() noexcept : toolList(nullptr), currentTool(nullptr), lastWarningMillis(0), activeExtruders(0),
	activeToolHeaters(0), ticksInSpinState(0),
	heatTaskIdleTicks(0),
	debug(0),
	beepFrequency(0), beepDuration(0),
	diagnosticsDestination(MessageType::NoDestinationMessage), justSentDiagnostics(false),
	spinningModule(noModule), stopped(false), active(false), processingConfig(true)
#if HAS_LINUX_INTERFACE
	, usingLinuxInterface(true)
#endif
{
	OutputBuffer::Init();
	platform = new Platform();
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
#if HAS_LINUX_INTERFACE
	linuxInterface = new LinuxInterface();
#endif

	SetPassword(DEFAULT_PASSWORD);
	message.Clear();
	messageSequence = 0;
}

void RepRap::Init() noexcept
{
	toolListMutex.Create("ToolList");
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

	platform->MessageF(UsbMessage, "%s Version %s dated %s\n", FIRMWARE_NAME, VERSION, DATE);

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
			const char *configFile = platform->GetConfigFile();
			if (!platform->SysFileExists(configFile))
			{
				configFile = platform->GetDefaultFile();
			}

# if HAS_LINUX_INTERFACE
			usingLinuxInterface = false;				// try to run config.g from the SD card
# endif
			if (gCodes->RunConfigFile(configFile))
			{
				platform->MessageF(UsbMessage, "\nExecuting %s...", configFile);
				do
				{
					// GCodes::Spin will read the macro and ensure IsDaemonBusy returns false when it's done
					Spin();
				} while (gCodes->IsDaemonBusy());
				platform->Message(UsbMessage, "Done!\n");
			}
			else
			{
# if HAS_LINUX_INTERFACE
				usingLinuxInterface = true;				// we failed to open config.g or default.g so assume we have a SBC connected
# else
				platform->Message(UsbMessage, "\nError, no configuration file found\n");
# endif
			}
		}
		else
		{
# if !HAS_LINUX_INTERFACE
			delay(3000);			// Wait a few seconds so users have a chance to see this
			platform->MessageF(UsbMessage, "%s\n", reply.c_str());
# endif
		}
	}
#endif

	processingConfig = false;

#if HAS_LINUX_INTERFACE
	if (usingLinuxInterface)
	{
		gCodes->RunConfigFile(platform->GetConfigFile());		// we didn't get config.g from SD card so request it from Linux
	}
#endif

	// Enable network (unless it's disabled)
	network->Activate();			// need to do this here, as the configuration GCodes may set IP address etc.

#if HAS_HIGH_SPEED_SD
	hsmci_set_idle_func(hsmciIdle);
	HSMCI->HSMCI_IDR = 0xFFFFFFFF;	// disable all HSMCI interrupts
	NVIC_EnableIRQ(HSMCI_IRQn);
#endif

	platform->MessageF(UsbMessage, "%s is up and running.\n", FIRMWARE_NAME);

	fastLoop = UINT32_MAX;
	slowLoop = 0;
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
		MutexLocker lock(toolListMutex);
		for (Tool *t = toolList; t != nullptr; t = t->Next())
		{
			if (t->DisplayColdExtrudeWarning())
			{
				platform->MessageF(WarningMessage, "Tool %d was not driven because its heater temperatures were not high enough or it has a heater fault\n", t->myNumber);
				lastWarningMillis = now;
			}
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
#else
	platform->MessageF(mtype, "%s version %s running on %s\n", FIRMWARE_NAME, VERSION, platform->GetElectronicsString());
#endif

#if SAM4E || SAM4S || SAME70
	platform->PrintUniqueId(mtype);
#endif

	// Show the used and free buffer counts. Do this early in case we are running out of them and the diagnostics get truncated.
	OutputBuffer::Diagnostics(mtype);

	// Now print diagnostics for other modules
	Tasks::Diagnostics(mtype);
	platform->Diagnostics(mtype);				// this includes a call to our Timing() function
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

	case MachineType::laser:
		platform->SetLaserPwm(0);
		break;

	default:
		break;
	}

	heat->Exit();		// this also turns off all heaters
	move->Exit();

	gCodes->EmergencyStop();
	platform->StopLogging();
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
	MutexLocker lock(toolListMutex);
	Tool** t = &toolList;
	while(*t != nullptr && (*t)->Number() < tool->Number())
	{
		t = &((*t)->next);
	}
	tool->next = *t;
	*t = tool;
	tool->UpdateExtruderAndHeaterCount(activeExtruders, activeToolHeaters);
	platform->UpdateConfiguredHeaters();
}

void RepRap::DeleteTool(Tool* tool) noexcept
{
	// Must have a valid tool...
	if (tool == nullptr)
	{
		return;
	}

	// Deselect it if necessary
	if (GetCurrentTool() == tool)
	{
		SelectTool(-1, false);
	}

	// Switch off any associated heater and remove heater references
	for (size_t i = 0; i < tool->HeaterCount(); i++)
	{
		heat->SwitchOff(tool->Heater(i));
	}

	// Purge any references to this tool
	MutexLocker lock(toolListMutex);
	for (Tool **t = &toolList; *t != nullptr; t = &((*t)->next))
	{
		if (*t == tool)
		{
			*t = tool->next;
			break;
		}
	}

	// Delete it
	Tool::Delete(tool);

	// Update the number of active heaters and extruder drives
	activeExtruders = activeToolHeaters = 0;
	for (Tool *t = toolList; t != nullptr; t = t->Next())
	{
		t->UpdateExtruderAndHeaterCount(activeExtruders, activeToolHeaters);
	}
	platform->UpdateConfiguredHeaters();
}

// Select the specified tool, putting the existing current tool into standby
void RepRap::SelectTool(int toolNumber, bool simulating) noexcept
{
	Tool* const newTool = GetTool(toolNumber);
	if (!simulating)
	{
		if (currentTool != nullptr && currentTool != newTool)
		{
			currentTool->Standby();
		}
		if (newTool != nullptr)
		{
			newTool->Activate();
		}
	}
	currentTool = newTool;
}

void RepRap::PrintTool(int toolNumber, const StringRef& reply) const noexcept
{
	const Tool* const tool = GetTool(toolNumber);
	if (tool != nullptr)
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
	Tool* const tool = GetTool(toolNumber);
	if (tool != nullptr)
	{
		if (!simulating)
		{
			tool->Standby();
		}
  		if (currentTool == tool)
		{
			currentTool = nullptr;
		}
	}
	else
	{
		platform->MessageF(ErrorMessage, "Attempt to standby a non-existent tool: %d\n", toolNumber);
	}
}

Tool* RepRap::GetTool(int toolNumber) const noexcept
{
	MutexLocker lock(toolListMutex);
	Tool* tool = toolList;
	while(tool != nullptr)
	{
		if (tool->Number() == toolNumber)
		{
			return tool;
		}
		tool = tool->Next();
	}
	return nullptr; // Not an error
}

// Return the current tool number, or -1 if no tool selected
int RepRap::GetCurrentToolNumber() const noexcept
{
	return (currentTool == nullptr) ? -1 : currentTool->Number();
}

// Get the current tool, or failing that the default tool. May return nullptr if we can't
// Called when a M104 or M109 command doesn't specify a tool number.
Tool* RepRap::GetCurrentOrDefaultTool() const noexcept
{
	// If a tool is already selected, use that one, else use the lowest-numbered tool which is the one at the start of the tool list
	return (currentTool != nullptr) ? currentTool : toolList;
}

bool RepRap::IsHeaterAssignedToTool(int8_t heater) const noexcept
{
	MutexLocker lock(toolListMutex);
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
	unsigned int numTools = 0;
	while (GetTool(numTools) != nullptr)
	{
		++numTools;
	}
	return numTools;
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
				platform->SoftwareReset(
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
OutputBuffer *RepRap::GetStatusResponse(uint8_t type, ResponseSource source) noexcept
{
	// Need something to write to...
	OutputBuffer *response;
	if (!OutputBuffer::Allocate(response))
	{
		return nullptr;
	}

	// Machine status
	char ch = GetStatusCharacter();
	response->printf("{\"status\":\"%c\",\"coords\":{", ch);

	// Coordinates
	const size_t numVisibleAxes = gCodes->GetVisibleAxes();
	// Homed axes
	response->cat("\"axesHomed\":");
	ch = '[';
	for (size_t axis = 0; axis < numVisibleAxes; ++axis)
	{
		response->catf("%c%d", ch, (gCodes->IsAxisHomed(axis)) ? 1 : 0);
		ch = ',';
	}

	// XYZ positions
	// Coordinates may be NaNs or infinities, for example when delta or SCARA homing fails. We must replace any NaNs or infinities to avoid JSON parsing errors.
	// Ideally we would report "unknown" or similar for axis positions that are not known because we haven't homed them, but that requires changes to both DWC and PanelDue.
	// So we report 9999.9 instead.

	// First the user coordinates
#if SUPPORT_WORKPLACE_COORDINATES
	response->catf("],\"wpl\":%u,\"xyz\":", gCodes->GetWorkplaceCoordinateSystemNumber());
#else
	response->cat("],\"xyz\":");
#endif
	ch = '[';
	for (size_t axis = 0; axis < numVisibleAxes; axis++)
	{
		response->catf("%c%.3f", ch, HideNan(gCodes->GetUserCoordinate(axis)));
		ch = ',';
	}

	// Now the machine coordinates and the extruder coordinates
	{
		float liveCoordinates[MaxAxesPlusExtruders];
#if SUPPORT_ROLAND
		if (roland->Active())
		{
			roland->GetCurrentRolandPosition(liveCoordinates);
		}
		else
#endif
		{
			move->LiveCoordinates(liveCoordinates, currentTool);
		}

		// Machine coordinates
		response->catf("],\"machine\":");		// announce the machine position
		ch = '[';
		for (size_t drive = 0; drive < numVisibleAxes; drive++)
		{
			response->catf("%c%.3f", ch, HideNan(liveCoordinates[drive]));
			ch = ',';
		}

		// Actual and theoretical extruder positions since power up, last G92 or last M23
		response->catf("],\"extr\":");			// announce actual extruder positions
		ch = '[';
		for (size_t extruder = 0; extruder < GetExtrudersInUse(); extruder++)
		{
			response->catf("%c%.1f", ch, HideNan(liveCoordinates[ExtruderToLogicalDrive(extruder)]));
			ch = ',';
		}
		if (ch == '[')							// we may have no extruders
		{
			response->cat(ch);
		}
	}

	// Current speeds
	response->catf("]},\"speeds\":{\"requested\":%.1f,\"top\":%.1f}",
			(double)move->GetRequestedSpeed(), (double)move->GetTopSpeed());

	// Current tool number
	response->catf(",\"currentTool\":%d", GetCurrentToolNumber());

	// Output notifications
	{
		const bool sendBeep = ((source == ResponseSource::AUX || !platform->HaveAux()) && beepDuration != 0 && beepFrequency != 0);
		const bool sendMessage = !message.IsEmpty();

		float timeLeft = 0.0;
		MutexLocker lock(messageBoxMutex);
		if (mbox.active && mbox.timer != 0)
		{
			timeLeft = (float)(mbox.timeout) / 1000.0 - (float)(millis() - mbox.timer) / 1000.0;
			mbox.active = (timeLeft > 0.0);
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
				beepFrequency = beepDuration = 0;
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
				message.Clear();
			}

			// Report message box
			if (mbox.active)
			{
				response->cat("\"msgBox\":{\"msg\":");
				response->EncodeString(mbox.message, false);
				response->cat(",\"title\":");
				response->EncodeString(mbox.title, false);
				response->catf(",\"mode\":%d,\"seq\":%" PRIu32 ",\"timeout\":%.1f,\"controls\":%u}", mbox.mode, mbox.seq, (double)timeLeft, mbox.controls);
			}
			response->cat('}');
		}
	}

	// ATX power
	response->catf(",\"params\":{\"atxPower\":%d", gCodes->AtxPowerControlled() ? (platform->AtxPower() ? 1 : 0) : -1);

	// Parameters
	{
		// Cooling fan values
		const size_t highestFan = fansManager->GetHighestUsedFanNumber();
		response->cat(",\"fanPercent\":");
		ch = '[';
		for (size_t i = 0; i <= highestFan; i++)
		{
			response->catf("%c%d", ch, (int)lrintf(fansManager->GetFanValue(i) * 100.0));
			ch = ',';
		}
		response->cat((ch == '[') ? "[]" : "]");

		// Cooling fan names
		if (type == 2)
		{
			response->cat(",\"fanNames\":");
			ch = '[';
			for (size_t fan = 0; fan <= highestFan; fan++)
			{
				response->cat(ch);
				ch = ',';

				const char *fanName = fansManager->GetFanName(fan);
				response->EncodeString(fanName, true);
			}
			response->cat((ch == '[') ? "[]" : "]");
		}

		// Speed and Extrusion factors in %
		response->catf(",\"speedFactor\":%.1f,\"extrFactors\":", (double)(gCodes->GetSpeedFactor()));
		ch = '[';
		for (size_t extruder = 0; extruder < GetExtrudersInUse(); extruder++)
		{
			response->catf("%c%.1f", ch, (double)(gCodes->GetExtrusionFactor(extruder)));
			ch = ',';
		}
		response->cat((ch == '[') ? "[]" : "]");
		response->catf(",\"babystep\":%.3f}", (double)gCodes->GetTotalBabyStepOffset(Z_AXIS));

		// G-code reply sequence for webserver (sequence number for AUX is handled later)
		if (source == ResponseSource::HTTP)
		{
			response->catf(",\"seq\":%" PRIu32, network->GetHttpReplySeq());
		}

		// Sensors
		response->cat(",\"sensors\":{");

		// Probe
		const int v0 = platform->GetCurrentZProbe().GetReading();
		int v1, v2;
		switch (platform->GetCurrentZProbe().GetSecondaryValues(v1, v2))
		{
			case 1:
				response->catf("\"probeValue\":%d,\"probeSecondary\":[%d]", v0, v1);
				break;
			case 2:
				response->catf("\"probeValue\":%d,\"probeSecondary\":[%d,%d]", v0, v1, v2);
				break;
			default:
				response->catf("\"probeValue\":%d", v0);
				break;
		}

		// Send fan RPM value(s)
		response->cat(",\"fanRPM\":");
		char ch = '[';
		for (size_t i = 0; i <= highestFan; ++i)
		{
			response->catf("%c%" PRIi32, ch, fansManager->GetFanRPM(i));
			ch = ',';
		}
		response->cat("]}");		// end fan RPMs and sensors
	}

	/* Temperatures */
	{
		response->cat(",\"temps\":{");

		/* Bed */
		const int8_t bedHeater = (MaxBedHeaters > 0) ? heat->GetBedHeater(0) : -1;
		if (bedHeater != -1)
		{
			response->catf("\"bed\":{\"current\":%.1f,\"active\":%.1f,\"standby\":%.1f,\"state\":%d,\"heater\":%d},",
				(double)heat->GetHeaterTemperature(bedHeater), (double)heat->GetActiveTemperature(bedHeater), (double)heat->GetStandbyTemperature(bedHeater),
					(int)heat->GetStatus(bedHeater), bedHeater);
		}

		/* Chamber */
		const int8_t chamberHeater = (MaxChamberHeaters > 0) ? heat->GetChamberHeater(0) : -1;
		if (chamberHeater != -1)
		{
			response->catf("\"chamber\":{\"current\":%.1f,\"active\":%.1f,\"state\":%d,\"heater\":%d},",
				(double)heat->GetHeaterTemperature(chamberHeater), (double)heat->GetActiveTemperature(chamberHeater),
					(int)heat->GetStatus(chamberHeater), chamberHeater);
		}

		/* Cabinet */
		const int8_t cabinetHeater = (MaxChamberHeaters > 1) ? heat->GetChamberHeater(1) : -1;
		if (cabinetHeater != -1)
		{
			response->catf("\"cabinet\":{\"current\":%.1f,\"active\":%.1f,\"state\":%d,\"heater\":%d},",
				(double)heat->GetHeaterTemperature(cabinetHeater), (double)heat->GetActiveTemperature(cabinetHeater),
					(int)heat->GetStatus(cabinetHeater), cabinetHeater);
		}

		/* Heaters */

		// Current temperatures
		response->cat("\"current\":");
		ch = '[';
		{
			const size_t highestHeater = heat->GetHighestUsedHeaterNumber();
			for (size_t heater = 0; heater <= highestHeater; heater++)
			{
				response->catf("%c%.1f", ch, (double)heat->GetHeaterTemperature(heater));
				ch = ',';
			}
			response->cat((ch == '[') ? "[]" : "]");

			// Current states
			response->cat(",\"state\":");
			ch = '[';
			for (size_t heater = 0; heater <= highestHeater; heater++)
			{
				response->catf("%c%d", ch, (int)heat->GetStatus(heater));
				ch = ',';
			}
			response->cat((ch == '[') ? "[]" : "]");

			// Names of the sensors use to control heaters
			if (type == 2)
			{
				response->cat(",\"names\":");
				ch = '[';
				for (size_t heater = 0; heater <= highestHeater; heater++)
				{
					response->cat(ch);
					ch = ',';
					response->EncodeString(GetHeat().GetHeaterSensorName(heater), true);
				}
				response->cat((ch == '[') ? "[]" : "]");
			}
		}

		// Tool temperatures
		response->cat(",\"tools\":{\"active\":[");
		{
			MutexLocker lock(toolListMutex);
			for (const Tool *tool = toolList; tool != nullptr; tool = tool->Next())
			{
				ch = '[';
				for (size_t heater = 0; heater < tool->heaterCount; heater++)
				{
					response->catf("%c%.1f", ch, (double)tool->activeTemperatures[heater]);
					ch = ',';
				}
				response->cat((ch == '[') ? "[]" : "]");

				if (tool->Next() != nullptr)
				{
					response->cat(',');
				}
			}

			response->cat("],\"standby\":[");
			for (const Tool *tool = toolList; tool != nullptr; tool = tool->Next())
			{
				ch = '[';
				for (size_t heater = 0; heater < tool->heaterCount; heater++)
				{
					response->catf("%c%.1f", ch, (double)tool->standbyTemperatures[heater]);
					ch = ',';
				}
				response->cat((ch == '[') ? "[]" : "]");

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
		response->catf(",\"progress\":%.1f}", (double)(scanner->GetProgress()));
	}
#endif

	// Spindles
	if (gCodes->GetMachineType() == MachineType::cnc || type == 2)
	{
		int lastConfiguredSpindle = -1;
		for (size_t spindle = 0; spindle < MaxSpindles; spindle++)
		{
			if (platform->AccessSpindle(spindle).GetToolNumber() != -1)
			{
				lastConfiguredSpindle = spindle;
			}
		}

		if (lastConfiguredSpindle != -1)
		{
			response->cat(",\"spindles\":[");
			for (int i = 0; i <= lastConfiguredSpindle; i++)
			{
				if (i > 0)
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

	if (gCodes->GetMachineType() == MachineType::laser)
	{
		response->catf(",\"laser\":%.1f", (double)(platform->GetLaserPwm() * 100.0));
	}

	/* Extended Status Response */
	if (type == 2)
	{
		// Cold Extrude/Retract
		response->catf(",\"coldExtrudeTemp\":%.1f", (double)(heat->ColdExtrude() ? 0.0 : HOT_ENOUGH_TO_EXTRUDE));
		response->catf(",\"coldRetractTemp\":%.1f", (double)(heat->ColdExtrude() ? 0.0 : HOT_ENOUGH_TO_RETRACT));

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
		FansBitmap controllableFans = 0;
		for (size_t fan = 0; fan < MaxFans; fan++)
		{
			if (fansManager->IsFanControllable(fan))
			{
				SetBit(controllableFans, fan);
			}
		}
		response->catf(",\"controllableFans\":%lu", controllableFans);

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
			const ZProbe& zp = platform->GetCurrentZProbe();

			// Trigger threshold, trigger height, type
			response->catf(",\"probe\":{\"threshold\":%d,\"height\":%.2f,\"type\":%u}",
							zp.GetAdcValue(), (double)zp.GetConfiguredTriggerHeight(), (unsigned int)zp.GetProbeType());
		}

		/* Tool Mapping */
		{
			response->cat(",\"tools\":[");
			MutexLocker lock(toolListMutex);
			for (Tool *tool = toolList; tool != nullptr; tool = tool->Next())
			{
				// Number
				response->catf("{\"number\":%d", tool->Number());

				// Name
				const char *toolName = tool->GetName();
				if (toolName[0] != 0)
				{
					response->cat(",\"name\":");
					response->EncodeString(toolName, false);
				}

				// Heaters
				response->cat(",\"heaters\":[");
				for (size_t heater = 0; heater < tool->HeaterCount(); heater++)
				{
					response->catf("%d", tool->Heater(heater));
					if (heater + 1 < tool->HeaterCount())
					{
						response->cat(',');
					}
				}

				// Extruder drives
				response->cat("],\"drives\":[");
				for (size_t drive = 0; drive < tool->DriveCount(); drive++)
				{
					response->catf("%d", tool->Drive(drive));
					if (drive + 1 < tool->DriveCount())
					{
						response->cat(',');
					}
				}

				// Axis mapping
				response->cat("],\"axisMap\":[[");
				bool first = true;
				for (size_t xi = 0; xi < MaxAxes; ++xi)
				{
					if (IsBitSet(tool->GetXAxisMap(), xi))
					{
						if (first)
						{
							first = false;
						}
						else
						{
							response->cat(',');
						}
						response->catf("%u", xi);
					}
				}
				response->cat("],[");
				first = true;
				for (size_t yi = 0; yi < MaxAxes; ++yi)
				{
					if (IsBitSet(tool->GetYAxisMap(), yi))
					{
						if (first)
						{
							first = false;
						}
						else
						{
							response->cat(',');
						}
						response->catf("%u", yi);
					}
				}
				response->cat("]]");

				// Fan mapping
				response->catf(",\"fans\":%lu", tool->GetFanMapping());

				// Filament (if any)
				if (tool->GetFilament() != nullptr)
				{
					const char *filamentName = tool->GetFilament()->GetName();
					response->catf(",\"filament\":");
					response->EncodeString(filamentName, false);
				}

				// Offsets
				response->cat(",\"offsets\":[");
				for (size_t i = 0; i < numVisibleAxes; i++)
				{
					response->catf((i == 0) ? "%.2f" : ",%.2f", (double)tool->GetOffset(i));
				}

  				// Do we have any more tools?
				response->cat((tool->Next() != nullptr) ? "]}," : "]}");
			}
			response->cat(']');
		}

		// MCU temperatures
#if HAS_CPU_TEMP_SENSOR
		{
			float minT, currT, maxT;
			platform->GetMcuTemperatures(minT, currT, maxT);
			response->catf(",\"mcutemp\":{\"min\":%.1f,\"cur\":%.1f,\"max\":%.1f}", (double)minT, (double)currT, (double)maxT);
		}
#endif

#if HAS_VOLTAGE_MONITOR
		// Power in voltages
		{
			float minV, currV, maxV;
			platform->GetPowerVoltages(minV, currV, maxV);
			response->catf(",\"vin\":{\"min\":%.1f,\"cur\":%.1f,\"max\":%.1f}", (double)minV, (double)currV, (double)maxV);
		}
#endif

#if HAS_12V_MONITOR
		// Power in voltages
		{
			float minV, currV, maxV;
			platform->GetV12Voltages(minV, currV, maxV);
			response->catf(",\"v12\":{\"min\":%.1f,\"cur\":%.1f,\"max\":%.1f}", (double)minV, (double)currV, (double)maxV);
		}
#endif
	}
	else if (type == 3)
	{
		// Current Layer
		response->catf(",\"currentLayer\":%d", printMonitor->GetCurrentLayer());

		// Current Layer Time
		response->catf(",\"currentLayerTime\":%.1f", (double)(printMonitor->GetCurrentLayerTime()));

		// Raw Extruder Positions
		response->cat(",\"extrRaw\":");
		ch = '[';
		for (size_t extruder = 0; extruder < GetExtrudersInUse(); extruder++)		// loop through extruders
		{
			response->catf("%c%.1f", ch, (double)(gCodes->GetRawExtruderTotalByDrive(extruder)));
			ch = ',';
		}
		if (ch == '[')
		{
			response->cat(ch);		// no extruders
		}

		// Fraction of file printed
		response->catf("],\"fractionPrinted\":%.1f", (double)((printMonitor->IsPrinting()) ? (printMonitor->FractionOfFilePrinted() * 100.0) : 0.0));

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
			response->catf(",\"timesLeft\":{\"file\":%.1f", (double)(printMonitor->EstimateTimeLeft(fileBased)));

			// Based on filament usage
			response->catf(",\"filament\":%.1f", (double)(printMonitor->EstimateTimeLeft(filamentBased)));

			// Based on layers
			response->catf(",\"layer\":%.1f}", (double)(printMonitor->EstimateTimeLeft(layerBased)));
		}
	}

	if (source == ResponseSource::AUX)
	{
		OutputBuffer *reply = platform->GetAuxGCodeReply();
		if (response != nullptr)
		{
			// Send the response to the last command. Do this last
			response->catf(",\"seq\":%" PRIu32 ",\"resp\":", platform->GetAuxSeq());	// send the response sequence number

			// Send the JSON response
			response->EncodeReply(reply);												// also releases the OutputBuffer chain
		}
	}

	if (response->cat('}') == 0)
	{
		OutputBuffer::ReleaseAll(response);
		return nullptr;
	}

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
	response->copy("{\"axisMins\":");
	char ch = '[';
	for (size_t axis = 0; axis < numAxes; axis++)
	{
		response->catf("%c%.2f", ch, (double)(platform->AxisMinimum(axis)));
		ch = ',';
	}

	// Axis maxima
	response->cat("],\"axisMaxes\":");
	ch = '[';
	for (size_t axis = 0; axis < numAxes; axis++)
	{
		response->catf("%c%.2f", ch, (double)(platform->AxisMaximum(axis)));
		ch = ',';
	}

	// Accelerations
	response->cat("],\"accelerations\":");
	ch = '[';
	for (size_t drive = 0; drive < MaxAxesPlusExtruders; drive++)
	{
		response->catf("%c%.2f", ch, (double)(platform->Acceleration(drive)));
		ch = ',';
	}

	// Motor currents
	response->cat("],\"currents\":");
	ch = '[';
	for (size_t drive = 0; drive < MaxAxesPlusExtruders; drive++)
	{
		response->catf("%c%d", ch, (int)platform->GetMotorCurrent(drive, 906));
		ch = ',';
	}

	// Firmware details
	response->catf("],\"firmwareElectronics\":\"%s", platform->GetElectronicsString());
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

	// Motor idle parameters
	response->catf(",\"idleCurrentFactor\":%.1f", (double)(platform->GetIdleCurrentFactor() * 100.0));
	response->catf(",\"idleTimeout\":%.1f", (double)(move->IdleTimeout()));

	// Minimum feedrates
	response->cat(",\"minFeedrates\":");
	ch = '[';
	for (size_t drive = 0; drive < MaxAxesPlusExtruders; drive++)
	{
		response->catf("%c%.2f", ch, (double)(platform->GetInstantDv(drive)));
		ch = ',';
	}

	// Maximum feedrates
	response->cat("],\"maxFeedrates\":");
	ch = '[';
	for (size_t drive = 0; drive < MaxAxesPlusExtruders; drive++)
	{
		response->catf("%c%.2f", ch, (double)(platform->MaxFeedrate(drive)));
		ch = ',';
	}

	// Config file is no longer included, because we can use rr_configfile or M503 instead
	response->cat("]}");

	return response;
}

// Get the JSON status response for PanelDue or the old web server.
// Type 0 was the old-style webserver status response, but is no longer supported.
// Type 1 is the new-style webserver status response.
// Type 2 is the M105 S2 response, which is like the new-style status response but some fields are omitted.
// Type 3 is the M105 S3 response, which is like the M105 S2 response except that static values are also included.
// 'seq' is the response sequence number, if it is not -1 and we have a different sequence number then we send the gcode response
OutputBuffer *RepRap::GetLegacyStatusResponse(uint8_t type, int seq) noexcept
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
	response->catf(",\"hstat\":[%d", (bedHeater == -1) ? 0 : static_cast<int>(heat->GetStatus(bedHeater)));
	for (size_t heater = DefaultE0Heater; heater < GetToolHeatersInUse(); heater++)
	{
		response->catf(",%d", static_cast<int>(heat->GetStatus(heater)));
	}
	response->cat(']');

	// Send XYZ positions
	const size_t numVisibleAxes = gCodes->GetVisibleAxes();

	// First the user coordinates
	response->catf(",\"pos\":");			// announce the user position
	ch = '[';
	for (size_t axis = 0; axis < numVisibleAxes; axis++)
	{
		// Coordinates may be NaNs, for example when delta or SCARA homing fails. Replace any NaNs or infinities by 9999.9 to prevent JSON parsing errors.
		response->catf("%c%.3f", ch, HideNan(gCodes->GetUserCoordinate(axis)));
		ch = ',';
	}

	// Now the machine coordinates
	float liveCoordinates[MaxAxesPlusExtruders];
	move->LiveCoordinates(liveCoordinates, currentTool);
	response->catf("],\"machine\":");		// announce the machine position
	ch = '[';
	for (size_t drive = 0; drive < numVisibleAxes; drive++)
	{
		response->catf("%c%.3f", ch, HideNan(liveCoordinates[drive]));
		ch = ',';
	}

	// Send the speed and extruder override factors
	response->catf("],\"sfactor\":%.2f,\"efactor\":", (double)(gCodes->GetSpeedFactor()));
	ch = '[';
	for (size_t i = 0; i < GetExtrudersInUse(); ++i)
	{
		response->catf("%c%.2f", ch, (double)(gCodes->GetExtrusionFactor(i)));
		ch = ',';
	}
	response->cat((ch == '[') ? "[]" : "]");

	// Send the baby stepping offset
	response->catf(",\"babystep\":%.03f", (double)(gCodes->GetTotalBabyStepOffset(Z_AXIS)));

	// Send the current tool number
	response->catf(",\"tool\":%d", GetCurrentToolNumber());

	// Send the Z probe value
	const int v0 = platform->GetCurrentZProbe().GetReading();
	int v1, v2;
	switch (platform->GetCurrentZProbe().GetSecondaryValues(v1, v2))
	{
	case 1:
		response->catf(",\"probe\":\"%d (%d)\"", v0, v1);
		break;
	case 2:
		response->catf(",\"probe\":\"%d (%d, %d)\"", v0, v1, v2);
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
		response->catf(",%.1f", (double)(fansManager->GetFanValue(i) * 100.0));
	}
	response->cat(']');

	// Send fan RPM value(s)
	response->cat(",\"fanRPM\":");
	ch = '[';
	for (size_t i = 0; i < MaxFans; ++i)
	{
		response->catf("%c%" PRIi32, ch, fansManager->GetFanRPM(i));
		ch = ',';
	}
	response->cat(']');

	// Send the home state. To keep the messages short, we send 1 for homed and 0 for not homed, instead of true and false.
	response->cat(",\"homed\":");
	ch = '[';
	for (size_t axis = 0; axis < numVisibleAxes; ++axis)
	{
		response->catf("%c%d", ch, (gCodes->IsAxisHomed(axis)) ? 1 : 0);
		ch = ',';
	}
	response->cat(']');

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
			mbox.active = (timeLeft > 0.0);
		}

		if (mbox.active)
		{
			response->catf(",\"msgBox.mode\":%d,\"msgBox.seq\":%" PRIu32 ",\"msgBox.timeout\":%.1f,\"msgBox.controls\":%u",
							mbox.mode, mbox.seq, (double)timeLeft, mbox.controls);
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

	// Send the response to the last command. Do this last because it can be long and may need to be truncated.
	const int auxSeq = (int)platform->GetAuxSeq();
	if (type < 2 || (seq != -1 && auxSeq != seq))
	{

		response->catf(",\"seq\":%d,\"resp\":", auxSeq);					// send the response sequence number

		// Send the JSON response
		response->EncodeReply(platform->GetAuxGCodeReply());				// also releases the OutputBuffer chain
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

					const struct tm * const timeInfo = gmtime(&fileInfo.lastModified);
					if (timeInfo->tm_year <= /*19*/80)
					{
						// Don't send the last modified date if it is invalid
						bytesLeft -= response->cat('}');
					}
					else
					{
						bytesLeft -= response->catf(",\"date\":\"%04u-%02u-%02uT%02u:%02u:%02u\"}",
								timeInfo->tm_year + 1900, timeInfo->tm_mon + 1, timeInfo->tm_mday,
								timeInfo->tm_hour, timeInfo->tm_min, timeInfo->tm_sec);
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
		const struct tm * const timeInfo = gmtime(&info.lastModifiedTime);
		if (timeInfo->tm_year > /*19*/80)
		{
			response->catf("\"lastModified\":\"%04u-%02u-%02uT%02u:%02u:%02u\",",
					timeInfo->tm_year + 1900, timeInfo->tm_mon + 1, timeInfo->tm_mday,
					timeInfo->tm_hour, timeInfo->tm_min, timeInfo->tm_sec);
		}

		response->catf("\"height\":%.2f,\"firstLayerHeight\":%.2f,\"layerHeight\":%.2f,",
					HideNan(info.objectHeight), HideNan(info.firstLayerHeight), HideNan(info.layerHeight));
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
				response->catf("%c%.1f", ch, HideNan(info.filamentNeeded[i]));
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

	if (platform->HaveAux())
	{
		platform->Beep(freq, ms);
		bleeped = true;
	}

	if (!bleeped)
	{
		beepFrequency = freq;
		beepDuration = ms;
	}
}

// Send a short message. We send it to both PanelDue and the web interface.
void RepRap::SetMessage(const char *msg) noexcept
{
	message.copy(msg);
	++messageSequence;

	if (platform->HaveAux())
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
}

// Clear pending message box
void RepRap::ClearAlert() noexcept
{
	MutexLocker lock(messageBoxMutex);
	mbox.active = false;
}

// Get the status character for the new-style status response
char RepRap::GetStatusCharacter() const noexcept
{
	return    (processingConfig)										? 'C'	// Reading the configuration file
#if HAS_LINUX_INTERFACE && SUPPORT_CAN_EXPANSION
			: (gCodes->IsFlashing() || CanInterface::IsFlashing())		? 'F'	// Flashing a new firmware binary
#else
			: (gCodes->IsFlashing())									? 'F'	// Flashing a new firmware binary
#endif
			: (IsStopped()) 											? 'H'	// Halted
#if HAS_VOLTAGE_MONITOR
			: (!platform->HasVinPower() && !gCodes->IsSimulating())		? 'O'	// Off i.e. powered down
#endif
			: (gCodes->IsPausing()) 									? 'D'	// Pausing / Decelerating
			: (gCodes->IsResuming()) 									? 'R'	// Resuming
			: (gCodes->IsPaused()) 										? 'S'	// Paused / Stopped
			: (printMonitor->IsPrinting() && gCodes->IsSimulating())	? 'M'	// Simulating
			: (printMonitor->IsPrinting())							  	? 'P'	// Printing
			: (gCodes->IsDoingToolChange())								? 'T'	// Changing tool
			: (gCodes->DoingFileMacro() || !move->NoLiveMovement()) 	? 'B'	// Busy
			:															  'I';	// Idle
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
	// Users sometimes put a tab character between the machine name and the comment, so allow for this
	myName.copy(nm);

	// Set new DHCP hostname
	network->SetHostname(myName.c_str());
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
	MutexLocker lock(toolListMutex);
	if (toolList != nullptr)
	{
		toolList->FlagTemperatureFault(dudHeater);
	}
}

GCodeResult RepRap::ClearTemperatureFault(int8_t wasDudHeater, const StringRef& reply) noexcept
{
	const GCodeResult rslt = heat->ResetFault(wasDudHeater, reply);
	MutexLocker lock(toolListMutex);
	if (toolList != nullptr)
	{
		toolList->ClearTemperatureFault(wasDudHeater);
	}
	return rslt;
}

#if HAS_MASS_STORAGE

// Save some resume information, returning true if successful
// We assume that the tool configuration doesn't change, only the temperatures and the mix
bool RepRap::WriteToolSettings(FileStore *f) const noexcept
{
	// First write the settings of all tools except the current one and the command to select them if they are on standby
	bool ok = true;
	MutexLocker lock(toolListMutex);
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
bool RepRap::WriteToolParameters(FileStore *f, const bool forceWriteOffsets) const noexcept
{
	bool ok = true, written = false;
	MutexLocker lock(toolListMutex);
	for (const Tool *t = toolList; ok && t != nullptr; t = t->Next())
	{
		const AxesBitmap axesProbed = t->GetAxisOffsetsProbed();
		if (axesProbed != 0 || forceWriteOffsets)
		{
			String<ScratchStringLength> scratchString;
			if (!written)
			{
				scratchString.copy("; Probed tool offsets\n");
				written = true;
			}
			scratchString.catf("G10 P%d", t->Number());
			for (size_t axis = 0; axis < MaxAxes; ++axis)
			{
				if (forceWriteOffsets || IsBitSet(axesProbed, axis))
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

#if SUPPORT_12864_LCD
	display->UpdatingFirmware();			// put the firmware update message on the display
#endif

#if IAP_IN_RAM
	// Send this message before we start using RAM that may contain message buffers
	platform->Message(AuxMessage, "Updating main firmware\n");
	platform->Message(UsbMessage, "Shutting down USB interface to update main firmware. Try reconnecting after 30 seconds.\n");

	// Allow time for the firmware update message to be sent
	const uint32_t now = millis();
	while (platform->FlushMessages() && millis() - now < 2000) { }
#endif

	// The machine will be unresponsive for a few seconds, don't risk damaging the heaters.
	// This also shuts down tasks and interrupts that might make use of the RAM that we are about to load the IAP binary into.
	EmergencyStop();						// this also stops Platform::Tick being called, which is necessary because it access Z probe object in RAM used by IAP
	network->Exit();						// kill the network task to stop it overwriting RAM that we use to hold the IAP

	// Step 0 - disable the cache because it interferes with flash memory access
	Cache::Disable();

#if USE_MPU
	//TODO consider setting flash memory to strongly-ordered instead
	ARM_MPU_Disable();
#endif

#if IAP_IN_RAM
	// Use RAM-based IAP
	iapFile->Read(reinterpret_cast<char *>(IAP_IMAGE_START), iapFile->Length());
#else
	// Step 1 - Write update binary to Flash and overwrite the remaining space with zeros
	// On the SAM3X, leave the last 1KB of Flash memory untouched, so we can reuse the NvData after this update

# if !defined(IFLASH_PAGE_SIZE) && defined(IFLASH0_PAGE_SIZE)
#  define IFLASH_PAGE_SIZE	IFLASH0_PAGE_SIZE
# endif

	// Use a 32-bit aligned buffer. This gives us the option of calling the EFC functions directly in future.
	uint32_t data32[IFLASH_PAGE_SIZE/4];
	char* const data = reinterpret_cast<char *>(data32);

# if SAM4E || SAM4S || SAME70
	// The EWP command is not supported for non-8KByte sectors in the SAM4 and SAME70 series.
	// So we have to unlock and erase the complete 64Kb or 128kb sector first. One sector is always enough to contain the IAP.
	flash_unlock(IAP_IMAGE_START, IAP_IMAGE_END, nullptr, nullptr);
	flash_erase_sector(IAP_IMAGE_START);

	for (uint32_t flashAddr = IAP_IMAGE_START; flashAddr < IAP_IMAGE_END; flashAddr += IFLASH_PAGE_SIZE)
	{
		const int bytesRead = iapFile->Read(data, IFLASH_PAGE_SIZE);

		if (bytesRead > 0)
		{
			// Do we have to fill up the remaining buffer with zeros?
			if (bytesRead != IFLASH_PAGE_SIZE)
			{
				memset(data + bytesRead, 0, sizeof(data[0]) * (IFLASH_PAGE_SIZE - bytesRead));
			}

			// Write one page at a time
			cpu_irq_disable();
			const uint32_t rc = flash_write(flashAddr, data, IFLASH_PAGE_SIZE, 0);
			cpu_irq_enable();

			if (rc != FLASH_RC_OK)
			{
				platform->MessageF(FirmwareUpdateErrorMessage, "flash write failed, code=%" PRIu32 ", address=0x%08" PRIx32 "\n", rc, flashAddr);
				return;
			}

			// Verify written data
			if (memcmp(reinterpret_cast<void *>(flashAddr), data, bytesRead) != 0)
			{
				platform->MessageF(FirmwareUpdateErrorMessage, "verify during flash write failed, address=0x%08" PRIx32 "\n", flashAddr);
				return;
			}
		}
		else
		{
			// Fill up the remaining space with zeros
			memset(data, 0, sizeof(data[0]) * sizeof(data));
			cpu_irq_disable();
			flash_write(flashAddr, data, IFLASH_PAGE_SIZE, 0);
			cpu_irq_enable();
		}
	}

	// Re-lock the whole area
	flash_lock(IAP_IMAGE_START, IAP_IMAGE_END, nullptr, nullptr);

# else	// SAM3X code

	for (uint32_t flashAddr = IAP_FLASH_START; flashAddr < IAP_FLASH_END; flashAddr += IFLASH_PAGE_SIZE)
	{
		const int bytesRead = iapFile->Read(data, IFLASH_PAGE_SIZE);

		if (bytesRead > 0)
		{
			// Do we have to fill up the remaining buffer with zeros?
			if (bytesRead != IFLASH_PAGE_SIZE)
			{
				memset(data + bytesRead, 0, sizeof(data[0]) * (IFLASH_PAGE_SIZE - bytesRead));
			}

			// Write one page at a time
			cpu_irq_disable();

			const char* op = "unlock";
			uint32_t rc = flash_unlock(flashAddr, flashAddr + IFLASH_PAGE_SIZE - 1, nullptr, nullptr);

			if (rc == FLASH_RC_OK)
			{
				op = "write";
				rc = flash_write(flashAddr, data, IFLASH_PAGE_SIZE, 1);
			}
			if (rc == FLASH_RC_OK)
			{
				op = "lock";
				rc = flash_lock(flashAddr, flashAddr + IFLASH_PAGE_SIZE - 1, nullptr, nullptr);
			}
			cpu_irq_enable();

			if (rc != FLASH_RC_OK)
			{
				MessageF(FirmwareUpdateErrorMessage, "flash %s failed, code=%" PRIu32 ", address=0x%08" PRIx32 "\n", op, rc, flashAddr);
				return;
			}
			// Verify written data
			if (memcmp(reinterpret_cast<void *>(flashAddr), data, bytesRead) != 0)
			{
				MessageF(FirmwareUpdateErrorMessage, "verify during flash write failed, address=0x%08" PRIx32 "\n", flashAddr);
				return;
			}
		}
		else
		{
			// Fill up the remaining space
			memset(data, 0, sizeof(data[0]) * sizeof(data));
			cpu_irq_disable();
			flash_unlock(flashAddr, flashAddr + IFLASH_PAGE_SIZE - 1, nullptr, nullptr);
			flash_write(flashAddr, data, IFLASH_PAGE_SIZE, 1);
			flash_lock(flashAddr, flashAddr + IFLASH_PAGE_SIZE - 1, nullptr, nullptr);
			cpu_irq_enable();
		}
	}
# endif
#endif

	iapFile->Close();

	StartIap();
#endif
}

void RepRap::StartIap() noexcept
{
#if !IAP_IN_RAM
	platform->Message(AuxMessage, "Updating main firmware\n");
	platform->Message(UsbMessage, "Shutting down USB interface to update main firmware. Try reconnecting after 30 seconds.\n");

	// Allow time for the firmware update message to be sent
	const uint32_t now = millis();
	while (platform->FlushMessages() && millis() - now < 2000) { }
#endif

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
