/****************************************************************************************************

 RepRapFirmware

 Platform contains all the code and definitions to deal with machine-dependent things such as control
 pins, bed area, number of extruders, tolerable accelerations and speeds and so on.

 -----------------------------------------------------------------------------------------------------

 Version 0.1

 18 November 2012

 Adrian Bowyer
 RepRap Professional Ltd
 http://reprappro.com

 Licence: GPL

 ****************************************************************************************************/

#include "Platform.h"

#include <Devices.h>
#include <Heating/Heat.h>
#include <Movement/DDA.h>
#include <Movement/Move.h>
#include <Movement/StepTimer.h>
#include <Tools/Tool.h>
#include <Endstops/ZProbe.h>
#include <Networking/Network.h>
#include <PrintMonitor/PrintMonitor.h>
#include <FilamentMonitors/FilamentMonitor.h>
#include "RepRap.h"
#include "Heap.h"
#include <GCodes/GCodes.h>
#include <GCodes/GCodeBuffer/GCodeBuffer.h>
#include "Event.h"
#include <Version.h>
#include "Logger.h"
#include "Tasks.h"
#include <Cache.h>
#include <Hardware/Spi/SharedSpiDevice.h>
#include <Math/Isqrt.h>
#include <Hardware/I2C.h>
#include <Hardware/NonVolatileMemory.h>
#include <Storage/CRC32.h>
#include <Accelerometers/Accelerometers.h>

#if SAM4E || SAM4S || SAME70
# include <AnalogIn.h>
using LegacyAnalogIn::AdcBits;
# include <DmacManager.h>
# include <pmc/pmc.h>
# if SAME70
static_assert(NumDmaChannelsUsed <= NumDmaChannelsSupported, "Need more DMA channels in CoreNG");
# endif
#elif SAME5x
# include <AnalogIn.h>
# include <DmacManager.h>
using AnalogIn::AdcBits;			// for compatibility with CoreNG, which doesn't have the AnalogIn namespace
#endif

#include <Libraries/sd_mmc/sd_mmc.h>

#if HAS_WIFI_NETWORKING
# include <Comms/FirmwareUpdater.h>
#endif

#if SUPPORT_DIRECT_LCD
# include "Display/Display.h"
#endif

#if SUPPORT_IOBITS
# include "PortControl.h"
#endif

#if HAS_SBC_INTERFACE
# include "SBC/SbcInterface.h"
# include "SBC/DataTransfer.h"
#endif

#if HAS_NETWORKING
# include "Networking/HttpResponder.h"
# include "Networking/FtpResponder.h"
# include "Networking/TelnetResponder.h"
#endif

#if SUPPORT_CAN_EXPANSION
# include "CAN/CanMessageGenericConstructor.h"
# include "CAN/CanInterface.h"
# include <CanMessageGenericTables.h>
#endif

#if SUPPORT_REMOTE_COMMANDS
# include <CanMessageGenericParser.h>
#endif

#include <climits>

#if !defined(HAS_LWIP_NETWORKING) || !defined(HAS_WIFI_NETWORKING) || !defined(HAS_CPU_TEMP_SENSOR) || !defined(HAS_HIGH_SPEED_SD) \
 || !defined(HAS_SMART_DRIVERS) || !defined(HAS_STALL_DETECT) || !defined(HAS_VOLTAGE_MONITOR) || !defined(HAS_12V_MONITOR) || !defined(HAS_VREF_MONITOR) \
 || !defined(SUPPORT_NONLINEAR_EXTRUSION) || !defined(SUPPORT_ASYNC_MOVES) || !defined(HAS_MASS_STORAGE) || !defined(HAS_EMBEDDED_FILES)
# error Missing feature definition
#endif

#if HAS_VOLTAGE_MONITOR

# if defined(DUET3_MB6HC)

	float Platform::AdcReadingToPowerVoltage(uint16_t adcVal) const noexcept
	{
		return (adcVal * powerMonitorVoltageRange)/(1u << AdcBits);
	}

	uint16_t Platform::PowerVoltageToAdcReading(float voltage) const noexcept
	{
		return (uint16_t)((voltage * (1u << AdcBits))/powerMonitorVoltageRange);
	}

# else

inline constexpr float AdcReadingToPowerVoltage(uint16_t adcVal) noexcept
{
	return adcVal * (PowerMonitorVoltageRange/(1u << AdcBits));
}

inline constexpr uint16_t PowerVoltageToAdcReading(float voltage) noexcept
{
	return (uint16_t)(voltage * ((1u << AdcBits)/PowerMonitorVoltageRange));
}

constexpr uint16_t driverPowerOnAdcReading = PowerVoltageToAdcReading(10.0);			// minimum voltage at which we initialise the drivers
constexpr uint16_t driverPowerOffAdcReading = PowerVoltageToAdcReading(9.5);			// voltages below this flag the drivers as unusable

#endif

# if ENFORCE_MAX_VIN
constexpr uint16_t driverOverVoltageAdcReading = PowerVoltageToAdcReading(29.0);		// voltages above this cause driver shutdown
constexpr uint16_t driverNormalVoltageAdcReading = PowerVoltageToAdcReading(27.5);		// voltages at or below this are normal
# endif

#endif

#if HAS_12V_MONITOR

inline constexpr float AdcReadingToV12Voltage(uint16_t adcVal) noexcept
{
	return adcVal * (V12MonitorVoltageRange/(1u << AdcBits));
}

inline constexpr uint16_t V12VoltageToAdcReading(float voltage) noexcept
{
	return (uint16_t)(voltage * ((1u << AdcBits)/V12MonitorVoltageRange));
}

constexpr uint16_t driverV12OnAdcReading = V12VoltageToAdcReading(10.0);				// minimum voltage at which we initialise the drivers
constexpr uint16_t driverV12OffAdcReading = V12VoltageToAdcReading(9.5);				// voltages below this flag the drivers as unusable

#endif

// Global variable for debugging in tricky situations e.g. within ISRs
int debugLine = 0;

// Global functions

DriversBitmap AxisDriversConfig::GetDriversBitmap() const noexcept
{
	DriversBitmap rslt;
	for (size_t i = 0; i < numDrivers; ++i)
	{
#if SUPPORT_CAN_EXPANSION
		if (driverNumbers[i].IsLocal())
#endif
		{
			rslt.SetBit(driverNumbers[i].localDriver);
		}
	}
	return rslt;
}

//*************************************************************************************************
// Platform class

#if SUPPORT_OBJECT_MODEL

// Object model table and functions
// Note: if using GCC version 7.3.1 20180622 and lambda functions are used in this table, you must compile this file with option -std=gnu++17.
// Otherwise the table will be allocate in RAM instead of flash, which wastes too much RAM.

// Macro to build a standard lambda function that includes the necessary type conversions
#define OBJECT_MODEL_FUNC(...)					OBJECT_MODEL_FUNC_BODY(Platform, __VA_ARGS__)
#define OBJECT_MODEL_FUNC_IF(_condition, ...)	OBJECT_MODEL_FUNC_IF_BODY(Platform, _condition, __VA_ARGS__)

constexpr ObjectModelArrayTableEntry Platform::objectModelArrayTable[] =
{
	// 0. boards[0].drivers
	{
		nullptr,
		[] (const ObjectModel *self, const ObjectExplorationContext& context) noexcept -> size_t { return NumDirectDrivers; },
		[] (const ObjectModel *self, ObjectExplorationContext& context) noexcept -> ExpressionValue { return ExpressionValue(&reprap.GetMove(), 14); }
	}
};

DEFINE_GET_OBJECT_MODEL_ARRAY_TABLE(Platform)

constexpr ObjectModelTableEntry Platform::objectModelTable[] =
{
	// 0. boards[0] members
#if SUPPORT_ACCELEROMETERS
	{ "accelerometer",		OBJECT_MODEL_FUNC_IF(Accelerometers::HasLocalAccelerometer(), self, 4),								ObjectModelEntryFlags::none },
#endif
#if SUPPORT_CAN_EXPANSION
	{ "canAddress",			OBJECT_MODEL_FUNC_NOSELF((int32_t)CanInterface::GetCanAddress()),									ObjectModelEntryFlags::none },
#endif
#if SUPPORT_DIRECT_LCD
	{ "directDisplay",		OBJECT_MODEL_FUNC_IF_NOSELF(reprap.GetDisplay().IsPresent(), &reprap.GetDisplay()),					ObjectModelEntryFlags::none },
#endif
	{ "drivers",			OBJECT_MODEL_FUNC_ARRAY(0),																			ObjectModelEntryFlags::live },
	{ "firmwareDate",		OBJECT_MODEL_FUNC_NOSELF(DATE),																		ObjectModelEntryFlags::none },
	{ "firmwareFileName",	OBJECT_MODEL_FUNC_NOSELF(IAP_FIRMWARE_FILE),														ObjectModelEntryFlags::none },
	{ "firmwareName",		OBJECT_MODEL_FUNC_NOSELF(FIRMWARE_NAME),															ObjectModelEntryFlags::none },
	{ "firmwareVersion",	OBJECT_MODEL_FUNC_NOSELF(VERSION),																	ObjectModelEntryFlags::none },
	{ "freeRam",			OBJECT_MODEL_FUNC_NOSELF((int32_t)Tasks::GetNeverUsedRam()),										ObjectModelEntryFlags::live },
#if HAS_SBC_INTERFACE
	{ "iapFileNameSBC",		OBJECT_MODEL_FUNC_NOSELF(IAP_UPDATE_FILE_SBC),														ObjectModelEntryFlags::none },
#endif
#if HAS_MASS_STORAGE
	{ "iapFileNameSD",		OBJECT_MODEL_FUNC_NOSELF(IAP_UPDATE_FILE),															ObjectModelEntryFlags::none },
#endif
	{ "maxHeaters",			OBJECT_MODEL_FUNC_NOSELF((int32_t)MaxHeaters),														ObjectModelEntryFlags::verbose },
	{ "maxMotors",			OBJECT_MODEL_FUNC_NOSELF((int32_t)NumDirectDrivers),												ObjectModelEntryFlags::verbose },
#if HAS_CPU_TEMP_SENSOR
	{ "mcuTemp",			OBJECT_MODEL_FUNC(self, 1),																			ObjectModelEntryFlags::live },
#endif
#ifdef DUET_NG
	{ "name",				OBJECT_MODEL_FUNC(self->GetBoardName()),															ObjectModelEntryFlags::none },
	{ "shortName",			OBJECT_MODEL_FUNC(self->GetBoardShortName()),														ObjectModelEntryFlags::none },
#else
	{ "name",				OBJECT_MODEL_FUNC_NOSELF(BOARD_NAME),																ObjectModelEntryFlags::none },
	{ "shortName",			OBJECT_MODEL_FUNC_NOSELF(BOARD_SHORT_NAME),															ObjectModelEntryFlags::none },
#endif
	{ "supportsDirectDisplay", OBJECT_MODEL_FUNC_NOSELF(SUPPORT_DIRECT_LCD ? true : false),										ObjectModelEntryFlags::verbose },
#if MCU_HAS_UNIQUE_ID
	{ "uniqueId",			OBJECT_MODEL_FUNC_IF(self->uniqueId.IsValid(), self->uniqueId),										ObjectModelEntryFlags::none },
#endif
#if HAS_12V_MONITOR
	{ "v12",				OBJECT_MODEL_FUNC(self, 3),																			ObjectModelEntryFlags::live },
#endif
#if HAS_VOLTAGE_MONITOR
	{ "vIn",				OBJECT_MODEL_FUNC(self, 2),																			ObjectModelEntryFlags::live },
#endif
#if HAS_WIFI_NETWORKING
	{ "wifiFirmwareFileName", OBJECT_MODEL_FUNC_NOSELF(WIFI_FIRMWARE_FILE),														ObjectModelEntryFlags::none },
#endif
#if HAS_CPU_TEMP_SENSOR
	// 1. boards[0].mcuTemp members
	{ "current",			OBJECT_MODEL_FUNC(self->GetMcuTemperatures().current, 1),											ObjectModelEntryFlags::live },
	{ "max",				OBJECT_MODEL_FUNC(self->GetMcuTemperatures().maximum, 1),											ObjectModelEntryFlags::none },
	{ "min",				OBJECT_MODEL_FUNC(self->GetMcuTemperatures().minimum, 1),											ObjectModelEntryFlags::none },
#endif

	// 2. boards[0].vIn members
#if HAS_VOLTAGE_MONITOR
	{ "current",			OBJECT_MODEL_FUNC(self->GetCurrentPowerVoltage(), 1),												ObjectModelEntryFlags::live },
	{ "max",				OBJECT_MODEL_FUNC(self->GetPowerVoltages().maximum, 1),												ObjectModelEntryFlags::none },
	{ "min",				OBJECT_MODEL_FUNC(self->GetPowerVoltages().minimum, 1),												ObjectModelEntryFlags::none },
#endif

#if HAS_12V_MONITOR
	// 3. boards[0].v12 members
	{ "current",			OBJECT_MODEL_FUNC(self->GetV12Voltages().current, 1),												ObjectModelEntryFlags::live },
	{ "max",				OBJECT_MODEL_FUNC(self->GetV12Voltages().maximum, 1),												ObjectModelEntryFlags::none },
	{ "min",				OBJECT_MODEL_FUNC(self->GetV12Voltages().minimum, 1),												ObjectModelEntryFlags::none },
#endif

#if SUPPORT_ACCELEROMETERS
	// 4. boards[0].accelerometer members
	{ "orientation",		OBJECT_MODEL_FUNC_NOSELF((int32_t)Accelerometers::GetLocalAccelerometerOrientation()),				ObjectModelEntryFlags::none },
	{ "points",				OBJECT_MODEL_FUNC_NOSELF((int32_t)Accelerometers::GetLocalAccelerometerDataPoints()),				ObjectModelEntryFlags::none },
	{ "runs",				OBJECT_MODEL_FUNC_NOSELF((int32_t)Accelerometers::GetLocalAccelerometerRuns()),						ObjectModelEntryFlags::none },
#endif
};

constexpr uint8_t Platform::objectModelTableDescriptor[] =
{
	5,																		// number of sections
	11 + SUPPORT_ACCELEROMETERS + HAS_SBC_INTERFACE + HAS_MASS_STORAGE + HAS_VOLTAGE_MONITOR + HAS_12V_MONITOR + HAS_CPU_TEMP_SENSOR
	  + SUPPORT_CAN_EXPANSION + SUPPORT_DIRECT_LCD + MCU_HAS_UNIQUE_ID + HAS_WIFI_NETWORKING,		// section 0: boards[0]
#if HAS_CPU_TEMP_SENSOR
	3,																		// section 1: mcuTemp
#else
	0,
#endif
#if HAS_VOLTAGE_MONITOR
	3,																		// section 2: vIn
#else
	0,																		// section 2: vIn
#endif
#if HAS_12V_MONITOR
	3,																		// section 3: v12
#else
	0,																		// section 3: v12
#endif
#if SUPPORT_ACCELEROMETERS
	3,																		// section 4: boards[0].accelerometer
#else
	0,
#endif
};

DEFINE_GET_OBJECT_MODEL_TABLE(Platform)

size_t Platform::GetNumGpInputsToReport() const noexcept
{
	size_t ret = MaxGpInPorts;
	while (ret != 0 && gpinPorts[ret - 1].IsUnused())
	{
		--ret;
	}
	return ret;
}

size_t Platform::GetNumGpOutputsToReport() const noexcept
{
	size_t ret = MaxGpOutPorts;
	while (ret != 0 && gpoutPorts[ret - 1].IsUnused())
	{
		--ret;
	}
	return ret;
}

#endif

bool Platform::deliberateError = false;						// true if we deliberately caused an exception for testing purposes

Platform::Platform() noexcept :
#if HAS_MASS_STORAGE
	logger(nullptr),
#endif
	board(DEFAULT_BOARD_TYPE), active(false), errorCodeBits(0),
	nextDriveToPoll(0),
	beepTicksToGo(0),
	lastFanCheckTime(0),
#if SUPPORT_PANELDUE_FLASH
	panelDueUpdater(nullptr),
#endif
#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE || HAS_EMBEDDED_FILES
	sysDir(nullptr),
#endif
	tickState(0), debugCode(0),
	lastDriverPollMillis(0),
#if SUPPORT_CAN_EXPANSION
	whenLastCanMessageProcessed(0),
#endif

#if SUPPORT_LASER
	lastLaserPwm(0.0),
#endif
	powerDownWhenFansStop(false), delayedPowerDown(false)
{
}

static RingBuffer<char> isrDebugBuffer;

// Return true if we have a debug buffer
bool Platform::HasDebugBuffer() noexcept
{
	return isrDebugBuffer.GetCapacity() != 0;
}

// Write a character to the debug buffer
bool Platform::IsrDebugPutc(char c) noexcept
{
	if (c != 0)
	{
		const bool b = isrDebugBuffer.PutItem(c);
		return b;
	}

	return true;
}

// Set the size of the debug buffer returning true if successful
bool Platform::SetDebugBufferSize(uint32_t size) noexcept
{
	if ((size & (size - 1)) == 0)
	{
		isrDebugBuffer.Init(size);
		return true;
	}
	return false;
}

// Initialise the Platform. Note: this is the first module to be initialised, so don't call other modules from here!
void Platform::Init() noexcept
{
#if HAS_LWIP_NETWORKING
	pinMode(EthernetPhyResetPin, OUTPUT_LOW);					// reset the Ethernet Phy chip
#endif

	// Do any board-specific initialisation that needs to be done early and does not depend on the board revision

	// Make sure the on-board drivers are disabled
#if defined(DUET_NG) || defined(PCCB_10)
	pinMode(GlobalTmc2660EnablePin, OUTPUT_HIGH);
#elif defined(DUET_M) || defined(DUET3MINI)
	pinMode(GlobalTmc22xxEnablePin, OUTPUT_HIGH);
#elif defined(DUET3_MB6HC)
	pinMode(GlobalTmc51xxEnablePin, OUTPUT_HIGH);
#endif

	// Make sure any WiFi module is held in reset
#if defined(DUET_NG)
	pinMode(EspResetPin, OUTPUT_LOW);						// reset the WiFi module or the W5500
	pinMode(EspEnablePin, OUTPUT_LOW);
#elif defined(DUET3_MB6HC)
	pinMode(EspEnablePin, OUTPUT_LOW);						// make sure that the Wifi module if present is disabled
#endif

	// Set up the local drivers. Do this after we have read any direction pins that specify the board type.
#if defined(DUET3MINI) && SUPPORT_TMC2240
	// Check whether we have a TMC2240 prototype expansion board connected, before we set the driver direction pins to outputs
	pinMode(DIRECTION_PINS[5], INPUT_PULLUP);
	delayMicroseconds(20);						// give the pullup resistor time to work
	hasTmc2240Expansion = !digitalRead(DIRECTION_PINS[5]);
#endif

	// Sort out which board we are running on (some firmware builds support more than one board variant)
	SetBoardType();

#if MCU_HAS_UNIQUE_ID
	uniqueId.SetFromCurrentBoard();
	if (uniqueId.IsValid())
	{
		uniqueId.GenerateMacAddress(defaultMacAddress);
	}
	else
	{
		defaultMacAddress.SetDefault();
	}
#else
	defaultMacAddress.SetDefault();
#endif

	// Real-time clock
	realTime = 0;

	// Comms
	commsParams[0] = 0;
	usbMutex.Create("USB");
#if SAME5x && !CORE_USES_TINYUSB
    SERIAL_MAIN_DEVICE.Start();
#else
    SERIAL_MAIN_DEVICE.Start(UsbVBusPin);
#endif

#if HAS_AUX_DEVICES
    auxDevices[0].Init(&SERIAL_AUX_DEVICE, AUX_BAUD_RATE);
	commsParams[1] = 1;							// by default we require a checksum on data from the aux port, to guard against overrun errors
#endif

#if defined(SERIAL_AUX2_DEVICE) && !defined(DUET3_ATE)
    auxDevices[1].Init(&SERIAL_AUX2_DEVICE, AUX2_BAUD_RATE);
	commsParams[2] = 0;
#endif

#ifdef DUET3_MB6XD
	pinMode(ModbusTxPin, OUTPUT_LOW);			// turn off the RS485 transmitter
#endif

	// Initialise the IO port subsystem
	IoPort::Init();

	// Shared SPI subsystem
	SharedSpiDevice::Init();

	// File management and SD card interfaces
	for (size_t i = 0; i < NumSdCards; ++i)
	{
		pinMode(SdCardDetectPins[i], INPUT_PULLUP);
	}

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE || HAS_EMBEDDED_FILES
	MassStorage::Init();
#endif

    // Ethernet networking defaults
	ipAddress = DefaultIpAddress;
	netMask = DefaultNetMask;
	gateWay = DefaultGateway;

	// Do hardware dependent initialisation
#if HAS_SMART_DRIVERS
	{
		size_t numSmartDrivers;
# if defined(DUET_NG)
		// Test for presence of a DueX2 or DueX5 expansion board and work out how many TMC2660 drivers we have
		// Call this before set set up the direction pins, because we use pulldown resistors on direction pins to specify the DueXn version.
		expansionBoard = DuetExpansion::DueXnInit();

		switch (expansionBoard)
		{
		case ExpansionBoardType::DueX2:
		case ExpansionBoardType::DueX2_v0_11:
			numSmartDrivers = 7;
			break;
		case ExpansionBoardType::DueX5:
		case ExpansionBoardType::DueX5_v0_11:
			numSmartDrivers = 10;
			break;
		case ExpansionBoardType::none:
		case ExpansionBoardType::DueX0:
		default:
			numSmartDrivers = 5;									// assume that any additional drivers are dumb enable/step/dir ones
			break;
		}

		DuetExpansion::AdditionalOutputInit();

# elif defined(DUET_M)
		numSmartDrivers = MaxSmartDrivers;							// for now we assume that expansion drivers are smart too
# elif defined(PCCB)
		numSmartDrivers = MaxSmartDrivers;
# elif defined(DUET3)
		numSmartDrivers = MaxSmartDrivers;
# elif defined(DUET3MINI)
		numSmartDrivers = MaxSmartDrivers;							// support the expansion board, but don't mind if it's missing
# endif
		reprap.GetMove().SetNumSmartDrivers(numSmartDrivers);
	}
#endif

	// Initialise endstops. On Duet 2 this must be done after testing for an expansion board.
	endstops.Init();

	driversPowered = false;

#if HAS_VOLTAGE_MONITOR
	autoSaveEnabled = false;
	autoSaveState = AutoSaveState::starting;
#endif

#if HAS_SMART_DRIVERS && (HAS_VOLTAGE_MONITOR || HAS_12V_MONITOR)
	warnDriversNotPowered = false;
#endif

	extrusionAncilliaryPwmValue = 0.0;

#if SUPPORT_SPI_SENSORS
	// Enable pullups on all the SPI CS pins. This is required if we are using more than one device on the SPI bus.
	// Otherwise, when we try to initialise the first device, the other devices may respond as well because their CS lines are not high.
	for (Pin p : SpiTempSensorCsPins)
	{
		pinMode(p, INPUT_PULLUP);
	}
#endif

	// If MISO from a MAX31856 board breaks after initialising the MAX31856 then if MISO floats low and reads as all zeros, this looks like a temperature of 0C and no error.
	// Enable the pullup resistor, with luck this will make it float high instead.
#if SAME5x
	// nothing to do here
#else
	pinMode(APIN_USART_SSPI_MISO, INPUT_PULLUP);
#endif

#ifdef PCCB
	// Setup the LED ports as GPIO ports
	for (size_t i = 0; i < ARRAY_SIZE(DefaultGpioPinNames); ++i)
	{
		gpoutPorts[i].Assign(DefaultGpioPinNames[i]);
	}
#endif

	for (size_t thermistor = 0; thermistor < NumThermistorInputs; thermistor++)
	{
		// TODO use ports for these?
		pinMode(TEMP_SENSE_PINS[thermistor], AIN);
		filteredAdcChannels[thermistor] = PinToAdcChannel(TEMP_SENSE_PINS[thermistor]);	// translate the pin number to the SAM ADC channel number;
	}

#if HAS_VREF_MONITOR
	// Set up the VSSA and VREF measurement channels
	pinMode(VssaSensePin, AIN);
	filteredAdcChannels[VssaFilterIndex] = PinToAdcChannel(VssaSensePin);		// translate the pin number to the SAM ADC channel number
	pinMode(VrefSensePin, AIN);
	filteredAdcChannels[VrefFilterIndex] = PinToAdcChannel(VrefSensePin);		// translate the pin number to the SAM ADC channel number
#endif

#if HAS_CPU_TEMP_SENSOR
# if SAME5x
	tpFilter.Init(0);
	AnalogIn::EnableTemperatureSensor(0, tpFilter.CallbackFeedIntoFilter, CallbackParameter(&tpFilter), 1, 0);
	tcFilter.Init(0);
	AnalogIn::EnableTemperatureSensor(1, tcFilter.CallbackFeedIntoFilter, CallbackParameter(&tcFilter), 1, 0);
	TemperatureCalibrationInit();
# else
	filteredAdcChannels[CpuTempFilterIndex] =
#if SAM4E || SAM4S || SAME70
			LegacyAnalogIn::
#endif
			GetTemperatureAdcChannel();
# endif
#endif

	// Initialise all the ADC filters and enable the corresponding ADC channels
	for (size_t filter = 0; filter < NumAdcFilters; ++filter)
	{
		adcFilters[filter].Init(0);
		AnalogInEnableChannel(filteredAdcChannels[filter], true);
	}

	// Hotend configuration
	filamentWidth = FILAMENT_WIDTH;

#if HAS_CPU_TEMP_SENSOR
	// MCU temperature monitoring
	highestMcuTemperature = -273.0;									// the highest temperature we have seen
	lowestMcuTemperature = 2000.0;									// the lowest temperature we have seen
	mcuTemperatureAdjust = 0.0;
#endif

#if HAS_VOLTAGE_MONITOR
	// Power monitoring
	vInMonitorAdcChannel = PinToAdcChannel(PowerMonitorVinDetectPin);
	pinMode(PowerMonitorVinDetectPin, AIN);
	AnalogInEnableChannel(vInMonitorAdcChannel, true);
	currentVin = 0;
	highestVin = 0;
	lowestVin = 9999;
	numVinUnderVoltageEvents = 0;
	previousVinUnderVoltageEvents = 0;
	numVinOverVoltageEvents = 0;
	previousVinOverVoltageEvents = 0;
#endif

#if HAS_12V_MONITOR
	// Power monitoring
	v12MonitorAdcChannel = PinToAdcChannel(PowerMonitorV12DetectPin);
	pinMode(PowerMonitorV12DetectPin, AIN);
	AnalogInEnableChannel(v12MonitorAdcChannel, true);
	currentV12 = highestV12 = 0;
	lowestV12 = 9999;
	numV12UnderVoltageEvents = previousV12UnderVoltageEvents = 0;
#endif

	// Kick everything off
	InitialiseInterrupts();

#ifdef DUET_NG
	DuetExpansion::DueXnTaskInit();								// must initialise interrupt priorities before calling this
#endif
	active = true;
}

#if HAS_VOLTAGE_MONITOR

// Reset the min and max recorded voltages to the current values
void Platform::ResetVoltageMonitors() noexcept
{
	lowestVin = currentVin;
	highestVin = currentVin;

#if HAS_12V_MONITOR
	lowestV12 = highestV12 = currentV12;
#endif

	reprap.BoardsUpdated();
}

float Platform::GetVinVoltage() const noexcept
{
	return AdcReadingToPowerVoltage(currentVin);
}

#endif

// Send the beep command to the aux channel. There is no flow control on this port, so it can't block for long.
void Platform::PanelDueBeep(int freq, int ms) noexcept
{
	MessageF(AuxMessage, "{\"beep_freq\":%d,\"beep_length\":%d}\n", freq, ms);
}

// Send a short message to the aux channel. There is no flow control on this port, so it can't block for long.
void Platform::SendPanelDueMessage(size_t auxNumber, const char *_ecv_array msg) noexcept
{
#if HAS_AUX_DEVICES
	// Don't send anything to PanelDue while we are flashing it
	if (!reprap.GetGCodes().IsFlashingPanelDue())
	{
		auxDevices[auxNumber].SendPanelDueMessage(msg);
	}
#endif
}

void Platform::Exit() noexcept
{
	StopLogging();
#if HAS_MASS_STORAGE || HAS_EMBEDDED_FILES
	MassStorage::CloseAllFiles();
#endif

	// Stop processing data. Don't try to send a message because it will probably never get there.
	active = false;

	// Close down USB and serial ports and release output buffers
	SERIAL_MAIN_DEVICE.end();
	usbOutput.ReleaseAll();

#if HAS_AUX_DEVICES
	for (AuxDevice& dev : auxDevices)
	{
		dev.Disable();
	}
#endif
}

#if HAS_NETWORKING

void Platform::SetIPAddress(IPAddress ip) noexcept
{
	ipAddress = ip;
	reprap.GetNetwork().SetEthernetIPAddress(ipAddress, netMask, gateWay);
}

void Platform::SetGateWay(IPAddress gw) noexcept
{
	gateWay = gw;
	reprap.GetNetwork().SetEthernetIPAddress(ipAddress, netMask, gateWay);
}

void Platform::SetNetMask(IPAddress nm) noexcept
{
	netMask = nm;
	reprap.GetNetwork().SetEthernetIPAddress(ipAddress, netMask, gateWay);
}

#endif

// Flush messages to USB and aux, returning true if there is more to send
bool Platform::FlushMessages() noexcept
{
	bool auxHasMore = false;
#if HAS_AUX_DEVICES
	for (AuxDevice& dev : auxDevices)
	{
		if (dev.Flush())
		{
			auxHasMore = true;
		}
	}
#endif

	// Write non-blocking data to the USB line
	bool usbHasMore = !usbOutput.IsEmpty();				// test first to see if we can avoid getting the mutex
	if (usbHasMore)
	{
		MutexLocker lock(usbMutex);
		OutputBuffer *usbOutputBuffer = usbOutput.GetFirstItem();
		if (usbOutputBuffer == nullptr)
		{
			(void) usbOutput.Pop();
		}
		else if (!SERIAL_MAIN_DEVICE.IsConnected())
		{
			// If the USB port is not opened, free the data left for writing
			OutputBuffer::ReleaseAll(usbOutputBuffer);
			(void) usbOutput.Pop();
		}
		else
		{
			// Write as much data as we can...
			const size_t bytesToWrite = min<size_t>(SERIAL_MAIN_DEVICE.canWrite(), usbOutputBuffer->BytesLeft());
			if (bytesToWrite != 0)
			{
				SERIAL_MAIN_DEVICE.print(usbOutputBuffer->Read(bytesToWrite), bytesToWrite);
			}

			if (usbOutputBuffer->BytesLeft() == 0)
			{
				usbOutput.ReleaseFirstItem();
			}
			else
			{
				usbOutput.ApplyTimeout(UsbTimeout);
			}
		}
		usbHasMore = !usbOutput.IsEmpty();
	}

	return auxHasMore || usbHasMore;
}

void Platform::Spin() noexcept
{
	if (!active)
	{
		return;
	}

#if SUPPORT_REMOTE_COMMANDS
	if (CanInterface::InExpansionMode())
	{
		if (StepTimer::CheckSynced())
		{
			digitalWrite(DiagPin, XNor(DiagOnPolarity, StepTimer::GetMasterTime() & (1u << 19)) != 0);
		}
		else
		{
			digitalWrite(DiagPin, XNor(DiagOnPolarity, StepTimer::GetTimerTicks() & (1u << 17)) != 0);
		}
	}
#endif

#if SUPPORT_CAN_EXPANSION
	// Turn off the ACT LED if it is time to do so
	if (millis() - whenLastCanMessageProcessed > ActLedFlashTime)
	{
		digitalWrite(ActLedPin, !ActOnPolarity);
	}
#endif

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE || HAS_EMBEDDED_FILES
	MassStorage::Spin();
#endif

	// Check for movement errors
	Move& move = reprap.GetMove();
	if (move.HasMovementError())
	{
		const StepErrorDetails details = move.GetStepErrorDetails();
		MessageF(AddError(MessageType::GenericMessage), "Movement halted because a step timing error occurred (code %u). Please reset the controller.\n", details.stepErrorType);
		if (details.stepErrorType == 3)
		{
			MessageF(AddError(MessageType::GenericMessage), "Existing: start=%" PRIu32 " length=%" PRIu32 ", new: start=%" PRIu32 ", overlap=%" PRIu32 " time now=%" PRIu32 "\n",
						details.executingStartTime, details.executingDuration, details.newSegmentStartTime,
						details.executingStartTime + details.executingDuration - details.newSegmentStartTime,
						details.timeNow);
		}
		move.GenerateMovementErrorDebug();
		move.ResetAfterError();
	}

	// Check for debug messages
	while (!isrDebugBuffer.IsEmpty())
	{
		char buf[101];
		const unsigned int charsRead = isrDebugBuffer.GetBlock(buf, sizeof(buf) - 1);
		buf[charsRead] = 0;
		Message(UsbMessage, buf);
	}

	// Try to flush messages to serial ports
	(void)FlushMessages();

	// Check the MCU max and min temperatures
#if HAS_CPU_TEMP_SENSOR
# if SAME5x
	if (tcFilter.IsValid() && tpFilter.IsValid())
# else
	if (adcFilters[CpuTempFilterIndex].IsValid())
# endif
	{
		const float currentMcuTemperature = GetCpuTemperature();
		if (currentMcuTemperature > highestMcuTemperature)
		{
			highestMcuTemperature= currentMcuTemperature;
			reprap.BoardsUpdated();
		}
		if (currentMcuTemperature < lowestMcuTemperature)
		{
			lowestMcuTemperature = currentMcuTemperature;
			reprap.BoardsUpdated();
		}
	}
#endif

	// Diagnostics test
	if (debugCode == (unsigned int)DiagnosticTestType::TestSpinLockup)
	{
		delay(30000);
	}

	// Check whether the TMC drivers need to be initialised.
	// The tick ISR also looks for over-voltage events, but it just disables the drivers without changing driversPowered or numVinOverVoltageEvents
	if (driversPowered)
	{
#if HAS_VOLTAGE_MONITOR
		if (currentVin < driverPowerOffAdcReading)
		{
			driversPowered = false;
			++numVinUnderVoltageEvents;
			lastVinUnderVoltageValue = currentVin;					// save this because the voltage may have changed by the time we report it
			reprap.GetGCodes().SetAllAxesNotHomed();
		}
# if ENFORCE_MAX_VIN
		else if (currentVin > driverOverVoltageAdcReading)
		{
			driversPowered = false;
			++numVinOverVoltageEvents;
			lastVinOverVoltageValue = currentVin;					// save this because the voltage may have changed by the time we report it
			reprap.GetGCodes().SetAllAxesNotHomed();
		}
# endif
		else
#endif

#if HAS_12V_MONITOR && HAS_SMART_DRIVERS
		if (currentV12 < driverV12OffAdcReading)
		{
			driversPowered = false;
			++numV12UnderVoltageEvents;
			lastV12UnderVoltageValue = currentV12;					// save this because the voltage may have changed by the time we report it
			reprap.GetGCodes().SetAllAxesNotHomed();
		}
		else
#endif
		{
			// Check one driver for temperature warning, temperature shutdown etc.
			reprap.GetMove().PollOneDriver(nextDriveToPoll);

			// Advance drive number ready for next time
			++nextDriveToPoll;
			if (nextDriveToPoll == NumDirectDrivers)
			{
				nextDriveToPoll = 0;
			}
		}
	}
#if HAS_VOLTAGE_MONITOR && HAS_12V_MONITOR
	else if (currentVin >= driverPowerOnAdcReading && currentV12 >= driverV12OnAdcReading
# if ENFORCE_MAX_VIN
		 	 && currentVin <= driverNormalVoltageAdcReading
# endif
			)
#elif HAS_VOLTAGE_MONITOR
	else if (currentVin >= driverPowerOnAdcReading
# if ENFORCE_MAX_VIN
		 	 && currentVin <= driverNormalVoltageAdcReading
# endif
			)
#elif HAS_12V_MONITOR
	else if (currentV12 >= driverV12OnAdcReading)
#else
	else
#endif
	{
		driversPowered = true;
#if HAS_SMART_DRIVERS
		reprap.GetMove().DriversJustPoweredUp();
#endif
	}

#if HAS_SMART_DRIVERS
	Move::SpinSmartDrivers(driversPowered);
#endif

	const uint32_t now = millis();

	// Update the time
	if (IsDateTimeSet() && now - timeLastUpdatedMillis >= 1000)
	{
		++realTime;								// this assumes that time_t is a seconds-since-epoch counter, which is not guaranteed by the C standard
		timeLastUpdatedMillis += 1000;
	}

	// Thermostatically-controlled fans (do this after getting TMC driver status)
	// We should call CheckFans frequently so that blip time is terminated at the right time, but we don't need or want to check sensors that often
	const bool checkFanSensors = (now - lastFanCheckTime >= FanCheckInterval);
	const bool thermostaticFanRunning = reprap.GetFansManager().CheckFans(checkFanSensors);
	if (checkFanSensors)
	{
		lastFanCheckTime = now;

		// Check for a deferred or delayed power down
		if (delayedPowerDown || powerDownWhenFansStop)
		{
			do
			{
				if (delayedPowerDown && (int32_t)(now - whenToPowerDown) < 0) break;
				if (powerDownWhenFansStop && thermostaticFanRunning) break;
				powerDownWhenFansStop = delayedPowerDown = false;
				AtxPowerOff();
			} while (false);
		}

		// Check whether it is time to report any faults (do this after checking fans in case driver cooling fans are turned on)
		if (now - lastDriverPollMillis > MinimumWarningInterval)
		{
			bool reported = false;

#if HAS_VOLTAGE_MONITOR
			if (numVinOverVoltageEvents != previousVinOverVoltageEvents)
			{
				MessageF(WarningMessage, "VIN over-voltage event (%.1fV)", (double)AdcReadingToPowerVoltage(lastVinOverVoltageValue));
				previousVinOverVoltageEvents = numVinOverVoltageEvents;
				reported = true;
			}
			if (numVinUnderVoltageEvents != previousVinUnderVoltageEvents)
			{
				MessageF(WarningMessage, "VIN under-voltage event (%.1fV)", (double)AdcReadingToPowerVoltage(lastVinUnderVoltageValue));
				previousVinUnderVoltageEvents = numVinUnderVoltageEvents;
				reported = true;
			}
#endif

#if HAS_12V_MONITOR
			if (numV12UnderVoltageEvents != previousV12UnderVoltageEvents)
			{
				MessageF(WarningMessage, "12V under-voltage event (%.1fV)", (double)AdcReadingToV12Voltage(lastV12UnderVoltageValue));
				previousV12UnderVoltageEvents = numV12UnderVoltageEvents;
				reported = true;
			}
#endif

			// Check for a VSSA fault
#if HAS_VREF_MONITOR
			constexpr uint32_t MaxVssaFilterSum = (15 * (1u << AdcBits) * ThermistorAverageReadings * 4)/2200;		// VSSA fuse should have <= 15 ohms resistance
			if (adcFilters[VssaFilterIndex].GetSum() > MaxVssaFilterSum)
			{
				Message(ErrorMessage, "VSSA fault, check thermistor wiring\n");
				reported = true;
			}
#elif defined(DUET_NG)
			if (   (board == BoardType::DuetWiFi_102 || board == BoardType::DuetEthernet_102)
				&& digitalRead(VssaSensePin)
			   )
			{
				Message(ErrorMessage, "VSSA fault, check thermistor wiring\n");
				reported = true;
			}
#endif

#if HAS_SMART_DRIVERS && (HAS_VOLTAGE_MONITOR || HAS_12V_MONITOR)
			// Check for attempts to move motors when not powered
			if (warnDriversNotPowered)
			{
				Message(ErrorMessage, "Attempt to move motors when VIN is not in range\n");
				warnDriversNotPowered = false;
				reported = true;
			}
#endif
			if (reported)
			{
				lastDriverPollMillis = now;
			}
		}
	}

#if HAS_VOLTAGE_MONITOR
	// Check for auto-pause, shutdown or resume
	if (autoSaveEnabled)
	{
		switch (autoSaveState)
		{
		case AutoSaveState::starting:
			// Some users set the auto resume threshold high to disable auto resume, so prime auto save at the auto save threshold plus half a volt
			if (currentVin >= autoResumeReading || currentVin > autoPauseReading + PowerVoltageToAdcReading(0.5))
			{
				autoSaveState = AutoSaveState::normal;
			}
			break;

		case AutoSaveState::normal:
			if (currentVin < autoPauseReading)
			{
				if (reprap.GetGCodes().LowVoltagePause())
				{
					autoSaveState = AutoSaveState::autoPaused;
				}
			}
			break;

		case AutoSaveState::autoPaused:
			if (currentVin >= autoResumeReading)
			{
				if (reprap.GetGCodes().LowVoltageResume())
				{
					autoSaveState = AutoSaveState::normal;
				}
			}
			break;

		default:
			break;
		}
	}
#endif

#if HAS_MASS_STORAGE
	// Flush the log file if it is time. This may take some time, so do it last.
	if (logger != nullptr)
	{
		logger->Flush(false);
	}
#endif

}

#if HAS_SMART_DRIVERS

// Report driver status conditions that require attention.
// Sets 'reported' if we reported anything, else leaves 'reported' alone.
void Platform::ReportDrivers(MessageType mt, DriversBitmap& whichDrivers, const char *_ecv_array text, bool& reported) noexcept
{
	if (whichDrivers.IsNonEmpty())
	{
		String<StringLength100> scratchString;
		scratchString.printf("%s reported by driver(s)", text);
		whichDrivers.Iterate([&scratchString](unsigned int drive, unsigned int) noexcept { scratchString.catf(" %u", drive); });
		MessageF(mt, "%s\n", scratchString.c_str());
		reported = true;
		whichDrivers.Clear();
	}
}

#endif

#if HAS_VOLTAGE_MONITOR || HAS_12V_MONITOR

bool Platform::HasDriverPower() const noexcept
{
# if HAS_SMART_DRIVERS
	return driversPowered;			// not quite right because drivers are disabled if we get over-voltage too, or if the 12V rail is low, but OK for the status report
# else
	return true;
# endif
}

#endif

#if HAS_VOLTAGE_MONITOR

void Platform::DisableAutoSave() noexcept
{
	autoSaveEnabled = false;
}

bool Platform::IsPowerOk() const noexcept
{
	// FIXME Implement auto-save for the SBC
	return (   !autoSaveEnabled
#if HAS_SBC_INTERFACE
			|| reprap.UsingSbcInterface()
#endif
		   )
		|| currentVin > autoPauseReading;
}

void Platform::EnableAutoSave(float saveVoltage, float resumeVoltage) noexcept
{
	autoPauseReading = PowerVoltageToAdcReading(saveVoltage);
	autoResumeReading = PowerVoltageToAdcReading(resumeVoltage);
	autoSaveEnabled = true;
}

bool Platform::GetAutoSaveSettings(float& saveVoltage, float&resumeVoltage) noexcept
{
	if (autoSaveEnabled)
	{
		saveVoltage = AdcReadingToPowerVoltage(autoPauseReading);
		resumeVoltage = AdcReadingToPowerVoltage(autoResumeReading);
	}
	return autoSaveEnabled;
}

#endif

#if HAS_CPU_TEMP_SENSOR

float Platform::GetCpuTemperature() const noexcept
{
#if SAME5x
	// From the datasheet:
	// T = (tl * vph * tc - th * vph * tc - tl * tp *vch + th * tp * vcl)/(tp * vcl - tp * vch - tc * vpl * tc * vph)
	const uint16_t tc_result = tcFilter.GetSum()/(tcFilter.NumAveraged() << (AnalogIn::AdcBits - 12));
	const uint16_t tp_result = tpFilter.GetSum()/(tpFilter.NumAveraged() << (AnalogIn::AdcBits - 12));

	int32_t result =  (tempCalF1 * tc_result - tempCalF2 * tp_result);
	const int32_t divisor = (tempCalF3 * tp_result - tempCalF4 * tc_result);
	result = (divisor == 0) ? 0 : result/divisor;
	return (float)result/16 + mcuTemperatureAdjust;
#else
	const float voltage = (float)adcFilters[CpuTempFilterIndex].GetSum() * (3.3/(float)((1u << AdcBits) * ThermistorAverageReadings));
# if SAM4E || SAM4S
	return (voltage - 1.44) * (1000.0/4.7) + 27.0 + mcuTemperatureAdjust;			// accuracy at 27C is +/-13C
# elif SAME70
	return (voltage - 0.72) * (1000.0/2.33) + 25.0 + mcuTemperatureAdjust;			// accuracy at 25C is +/-34C
# else
#  error undefined CPU temp conversion
# endif
#endif
}

#endif

//*****************************************************************************************************************
// Interrupts

#if SAME5x
// Set a contiguous range of interrupts to the specified priority
static void SetInterruptPriority(IRQn base, unsigned int num, uint32_t prio)
{
	do
	{
		NVIC_SetPriority(base, prio);
		base = (IRQn)(base + 1);
		--num;
	}
	while (num != 0);
}
#endif

void Platform::InitialiseInterrupts() noexcept
{
	// Watchdog interrupt priority if applicable has already been set up in RepRap::Init

#if HAS_HIGH_SPEED_SD
	NVIC_SetPriority(SdhcIRQn, NvicPriorityHSMCI);						// set priority for SD interface interrupts
#endif

	// Set PanelDue UART interrupt priority is set in AuxDevice::Init
	// WiFi UART interrupt priority is now set in module WiFiInterface

#if SUPPORT_TMC22xx && !SAME5x											// SAME5x uses a DMA interrupt instead of the UART interrupt
# if TMC22xx_HAS_MUX
	NVIC_SetPriority(TMC22xx_UART_IRQn, NvicPriorityDriversSerialTMC);	// set priority for TMC2660 SPI interrupt
# else
	NVIC_SetPriority(TMC22xxUartIRQns[0], NvicPriorityDriversSerialTMC);
	NVIC_SetPriority(TMC22xxUartIRQns[1], NvicPriorityDriversSerialTMC);
# endif
#endif

#if SUPPORT_TMC2660
	NVIC_SetPriority(TMC2660_SPI_IRQn, NvicPriorityDriversSerialTMC);	// set priority for TMC2660 SPI interrupt
#endif

#if HAS_LWIP_NETWORKING
	// Set up the Ethernet interface priority here to because we have access to the priority definitions
# if SAME70 || SAME5x
	NVIC_SetPriority(GMAC_IRQn, NvicPriorityEthernet);
# else
	NVIC_SetPriority(EMAC_IRQn, NvicPriorityEthernet);
# endif
#endif

#if SAME5x
	SetInterruptPriority(DMAC_0_IRQn, 5, NvicPriorityDMA);				// SAME5x DMAC has 5 contiguous IRQ numbers
#elif SAME70
	NVIC_SetPriority(XDMAC_IRQn, NvicPriorityDMA);
#endif

#if SAME5x
	SetInterruptPriority(EIC_0_IRQn, 16, NvicPriorityPins);				// SAME5x EXINT has 16 contiguous IRQ numbers
#else
	NVIC_SetPriority(PIOA_IRQn, NvicPriorityPins);
	NVIC_SetPriority(PIOB_IRQn, NvicPriorityPins);
	NVIC_SetPriority(PIOC_IRQn, NvicPriorityPins);
# ifdef ID_PIOD
	NVIC_SetPriority(PIOD_IRQn, NvicPriorityPins);
# endif
# ifdef ID_PIOE
	NVIC_SetPriority(PIOE_IRQn, NvicPriorityPins);
# endif
#endif

#if SAME5x
	SetInterruptPriority(USB_0_IRQn, 4, NvicPriorityUSB);				// SAME5x USB has 4 contiguous IRQ numbers
#elif SAME70
	NVIC_SetPriority(USBHS_IRQn, NvicPriorityUSB);
#elif SAM4E || SAM4S
	NVIC_SetPriority(UDP_IRQn, NvicPriorityUSB);
#elif SAM3XA
	NVIC_SetPriority(UOTGHS_IRQn, NvicPriorityUSB);
#else
# error Unsupported processor
#endif

#if defined(DUET_NG) || defined(DUET_M)
	NVIC_SetPriority(I2C_IRQn, NvicPriorityTwi);
#endif

#if SUPPORT_CAN_EXPANSION
# if SAME5x
	NVIC_SetPriority(CAN0_IRQn, NvicPriorityCan);
	NVIC_SetPriority(CAN1_IRQn, NvicPriorityCan);
# elif SAME70
	NVIC_SetPriority(MCAN0_INT0_IRQn, NvicPriorityCan);		// we don't use INT1
	NVIC_SetPriority(MCAN1_INT0_IRQn, NvicPriorityCan);		// we don't use INT1
# endif
#endif

	// Tick interrupt for ADC conversions
	tickState = 0;
	currentFilterNumber = 0;
}

//*************************************************************************************************

// Debugging variables
//extern "C" uint32_t longestWriteWaitTime, shortestWriteWaitTime, longestReadWaitTime, shortestReadWaitTime;
//extern uint32_t maxRead, maxWrite;

/*static*/ const char *_ecv_array Platform::GetResetReasonText() noexcept
{
#if SAME5x
	const uint8_t resetReason = RSTC->RCAUSE.reg;
	// The datasheet says only one of these bits will be set
	if (resetReason & RSTC_RCAUSE_POR)		{ return "power up"; }
	if (resetReason & RSTC_RCAUSE_BODCORE)	{ return "core brownout"; }
	if (resetReason & RSTC_RCAUSE_BODVDD)	{ return "Vdd brownout"; }
	if (resetReason & RSTC_RCAUSE_WDT)		{ return "watchdog"; }
	if (resetReason & RSTC_RCAUSE_NVM)		{ return "NVM"; }
	if (resetReason & RSTC_RCAUSE_EXT)		{ return "reset button"; }
	if (resetReason & RSTC_RCAUSE_SYST)		{ return "software"; }
	if (resetReason & RSTC_RCAUSE_BACKUP)	{ return "backup/hibernate"; }
	return "unknown";
#else
	constexpr const char *_ecv_array resetReasons[8] = { "power up", "backup", "watchdog", "software",
# ifdef DUET_NG
	// On the SAM4E a watchdog reset may be reported as a user reset because of the capacitor on the NRST pin.
	// The SAM4S is the same but the Duet Maestro has a diode in the reset circuit to avoid this problem.
									"reset button or watchdog",
# else
									"reset button",
# endif
									"unknown", "unknown", "unknown" };
	return resetReasons[(REG_RSTC_SR & RSTC_SR_RSTTYP_Msk) >> RSTC_SR_RSTTYP_Pos];
#endif
}

#if CORE_USES_TINYUSB	//debug
extern uint32_t numUsbInterrupts;
#endif

// Return diagnostic information
void Platform::Diagnostics(MessageType mtype) noexcept
{
#if 0	// USE_CACHE && (SAM4E || SAME5x)
	// Get the cache statistics before we start messing around with the cache
	const uint32_t cacheCount = Cache::GetHitCount();
#endif

	Message(mtype, "=== Platform ===\n");

	// Debugging support
	if (debugLine != 0)
	{
		MessageF(mtype, "Debug line %d\n", debugLine);
	}

	// Show the up time and reason for the last reset
	const uint32_t now = (uint32_t)(millis64()/1000u);		// get up time in seconds
	MessageF(mtype, "Last reset %02d:%02d:%02d ago, cause: %s\n", (unsigned int)(now/3600), (unsigned int)((now % 3600)/60), (unsigned int)(now % 60), GetResetReasonText());

	// Show the reset code stored at the last software reset
	{
		NonVolatileMemory mem;
		unsigned int slot;
		const SoftwareResetData * const srd = mem.GetLastWrittenResetData(slot);
		if (srd == nullptr)
		{
			Message(mtype, "Last software reset details not available\n");
		}
		else
		{
			srd->Printit(mtype, slot);
		}
	}

	// Show the current error codes
	MessageF(mtype, "Error status: 0x%02" PRIx32 "\n", errorCodeBits);		// we only use the bottom 5 bits at present, so print just 2 characters

#if HAS_AUX_DEVICES
	// Show the aux port status
	for (size_t i = 0; i < ARRAY_SIZE(auxDevices); ++i)
	{
		auxDevices[i].Diagnostics(mtype, i);
	}
#endif

#if 0	//ifdef DUET3MINI
	// Report the processor revision level and analogIn status (trying to debug the spurious zero VIN issue)
	{
		// The DSU clocks are enabled by default so no need to enable them here
		const unsigned int chipVersion = DSU->DID.bit.REVISION;
		uint32_t conversionsStarted, conversionsCompleted, conversionTimeouts, errors;
		AnalogIn::GetDebugInfo(conversionsStarted, conversionsCompleted, conversionTimeouts, errors);
		MessageF(mtype, "MCU revision %u, ADC conversions started %" PRIu32 ", completed %" PRIu32 ", timed out %" PRIu32 ", errs %" PRIu32 "\n",
					chipVersion, conversionsStarted, conversionsCompleted, conversionTimeouts, errors);
	}

#endif

#if HAS_CPU_TEMP_SENSOR
	// Show the MCU temperatures
	const float currentMcuTemperature = GetCpuTemperature();
	MessageF(mtype, "MCU temperature: min %.1f, current %.1f, max %.1f\n",
		(double)lowestMcuTemperature, (double)currentMcuTemperature, (double)highestMcuTemperature);
	lowestMcuTemperature = highestMcuTemperature = currentMcuTemperature;

# if HAS_VOLTAGE_MONITOR
	// No need to call reprap.BoardsUpdated() here because that is done in ResetVoltageMonitors which is called later
# else
	reprap.BoardsUpdated();
# endif
#endif

#if HAS_VOLTAGE_MONITOR
	// Show the supply voltage
	MessageF(mtype, "Supply voltage: min %.1f, current %.1f, max %.1f, under voltage events: %" PRIu32 ", over voltage events: %" PRIu32 ", power good: %s\n",
		(double)AdcReadingToPowerVoltage(lowestVin), (double)AdcReadingToPowerVoltage(currentVin), (double)AdcReadingToPowerVoltage(highestVin),
				numVinUnderVoltageEvents, numVinOverVoltageEvents,
				(HasDriverPower()) ? "yes" : "no");
#endif

#if HAS_12V_MONITOR
	// Show the 12V rail voltage
	MessageF(mtype, "12V rail voltage: min %.1f, current %.1f, max %.1f, under voltage events: %" PRIu32 "\n",
		(double)AdcReadingToV12Voltage(lowestV12), (double)AdcReadingToV12Voltage(currentV12), (double)AdcReadingToV12Voltage(highestV12), numV12UnderVoltageEvents);
#endif

	ResetVoltageMonitors();

	Heap::Diagnostics(mtype, *this);
	Event::Diagnostics(mtype, *this);

	// Show current RTC time
	Message(mtype, "Date/time: ");
	struct tm timeInfo;
	if (gmtime_r(&realTime, &timeInfo) != nullptr)
	{
		MessageF(mtype, "%04u-%02u-%02u %02u:%02u:%02u\n",
				timeInfo.tm_year + 1900, timeInfo.tm_mon + 1, timeInfo.tm_mday,
				timeInfo.tm_hour, timeInfo.tm_min, timeInfo.tm_sec);
	}
	else
	{
		Message(mtype, "not set\n");
	}

#if 0	// USE_CACHE && (SAM4E || SAME5x)
	MessageF(mtype, "Cache data hit count %" PRIu32 "\n", cacheCount);
#endif

	reprap.Timing(mtype);

#if CORE_USES_TINYUSB	//DEBUG
	MessageF(mtype, "USB interrupts %" PRIu32 "\n", numUsbInterrupts);
#endif

#if 0
	// Debugging temperature readings
	const uint32_t div = ThermistorAveragingFilter::NumAveraged() >> 2;		// 2 oversample bits
	MessageF(mtype, "Vssa %" PRIu32 " Vref %" PRIu32 " Temp0 %" PRIu32 " Temp1 %" PRIu32 "\n",
			adcFilters[VssaFilterIndex].GetSum()/div, adcFilters[VrefFilterIndex].GetSum()/div, adcFilters[0].GetSum()/div, adcFilters[1].GetSum()/div);
#endif

#ifdef SOFT_TIMER_DEBUG
	MessageF(mtype, "Soft timer interrupts executed %u, next %u scheduled at %u, now %u\n",
		numSoftTimerInterruptsExecuted, STEP_TC->TC_CHANNEL[STEP_TC_CHAN].TC_RB, lastSoftTimerInterruptScheduledAt, GetTimerTicks());
#endif

#ifdef I2C_IFACE
	const TwoWire::ErrorCounts errs = I2C_IFACE.GetErrorCounts(true);
	MessageF(mtype, "I2C nak errors %" PRIu32 ", send timeouts %" PRIu32 ", receive timeouts %" PRIu32 ", finishTimeouts %" PRIu32 ", resets %" PRIu32 "\n",
		errs.naks, errs.sendTimeouts, errs.recvTimeouts, errs.finishTimeouts, errs.resets);
#endif
}

// Execute a timed square root that takes less than one millisecond
static uint32_t TimedSqrt(uint64_t arg, uint32_t& timeAcc) noexcept
{
	IrqDisable();
	asm volatile("":::"memory");
	uint32_t now1 = SysTick->VAL;
	const uint32_t ret = isqrt64(arg);
	uint32_t now2 = SysTick->VAL;
	asm volatile("":::"memory");
	IrqEnable();
	now1 &= 0x00FFFFFF;
	now2 &= 0x00FFFFFF;
	timeAcc += ((now1 > now2) ? now1 : now1 + (SysTick->LOAD & 0x00FFFFFF) + 1) - now2;
	return ret;
}

GCodeResult Platform::PrintTestReport(GCodeBuffer& gb, const StringRef& reply, OutputBuffer *_ecv_null & buf) const THROWS(GCodeException)
{
	if (!OutputBuffer::Allocate(buf))
	{
		reply.copy("No output buffer");
		return GCodeResult::error;
	}

	bool testFailed = false;
#if HAS_MASS_STORAGE
	// Check the SD card detect and speed
	if (!MassStorage::IsCardDetected(0))
	{
		buf->copy("SD card 0 not detected");
		testFailed = true;
	}
# if HAS_HIGH_SPEED_SD
	else if (sd_mmc_get_interface_speed(0) != ExpectedSdCardSpeed)
	{
		buf->printf("SD card speed %.2fMbytes/sec is unexpected", (double)((float)sd_mmc_get_interface_speed(0) * 0.000001));
		testFailed = true;
	}
# endif
	else
	{
		buf->copy("SD card interface OK");
	}
#endif

#if HAS_CPU_TEMP_SENSOR
	// Check the MCU temperature
	{
		gb.MustSee('T');
		float tempMinMax[2];
		size_t numTemps = 2;
		gb.GetFloatArray(tempMinMax, numTemps, false);
		const float currentMcuTemperature = GetCpuTemperature();
		if (currentMcuTemperature < tempMinMax[0])
		{
			buf->lcatf("MCU temperature %.1f is lower than expected", (double)currentMcuTemperature);
			testFailed = true;
		}
		else if (currentMcuTemperature > tempMinMax[1])
		{
			buf->lcatf("MCU temperature %.1f is higher than expected", (double)currentMcuTemperature);
			testFailed = true;
		}
		else
		{
			buf->lcat("MCU temperature reading OK");
		}
	}
#endif

#if HAS_VOLTAGE_MONITOR
	// Check the supply voltage
	{
		gb.MustSee('V');
		float voltageMinMax[2];
		size_t numVoltages = 2;
		gb.GetFloatArray(voltageMinMax, numVoltages, false);
		const float voltage = AdcReadingToPowerVoltage(currentVin);
		if (voltage < voltageMinMax[0])
		{
			buf->lcatf("VIN voltage reading %.1f is lower than expected", (double)voltage);
			testFailed = true;
		}
		else if (voltage > voltageMinMax[1])
		{
			buf->lcatf("VIN voltage reading %.1f is higher than expected", (double)voltage);
			testFailed = true;
		}
		else
		{
			buf->lcat("VIN voltage reading OK");
		}
	}
#endif

#if HAS_12V_MONITOR
	// Check the 12V rail voltage
	{
		gb.MustSee('W');
		float voltageMinMax[2];
		size_t numVoltages = 2;
		gb.GetFloatArray(voltageMinMax, numVoltages, false);

		const float voltage = AdcReadingToV12Voltage(currentV12);
		if (voltage < voltageMinMax[0])
		{
			buf->lcatf("12V voltage reading %.1f is lower than expected", (double)voltage);
			testFailed = true;
		}
		else if (voltage > voltageMinMax[1])
		{
			buf->lcatf("12V voltage reading %.1f is higher than expected", (double)voltage);
			testFailed = true;
		}
		else
		{
			buf->lcat("12V voltage reading OK");
		}
	}
#endif

#if HAS_SMART_DRIVERS
	// Check the stepper driver status
	bool driversOK = true;
	for (size_t driver = 0; driver < reprap.GetMove().GetNumSmartDrivers(); ++driver)
	{
		const StandardDriverStatus stat = Move::GetSmartDriverStatus(driver, true, false);
		if (stat.ot || stat.otpw)
		{
			buf->lcatf("Driver %u reports over temperature", driver);
			driversOK = false;
		}
		if (stat.s2ga || stat.s2gb || stat.s2vsa || stat.s2vsb)
		{
			buf->lcatf("Driver %u reports short-to-ground", driver);
			driversOK = false;
		}
	}
	if (driversOK)
	{
		buf->lcat("Driver status OK");
	}
	else
	{
		testFailed = true;
	}
#endif
	buf->lcat((testFailed) ? "***** ONE OR MORE CHECKS FAILED *****" : "All checks passed");

#if MCU_HAS_UNIQUE_ID
	if (!testFailed)
	{
		buf->lcat("Board ID: ");
		uniqueId.AppendCharsToBuffer(buf);
	}
#endif
	return GCodeResult::ok;
}

GCodeResult Platform::DiagnosticTest(GCodeBuffer& gb, const StringRef& reply, OutputBuffer *_ecv_null & buf, unsigned int d) THROWS(GCodeException)
{
	switch (d)
	{
	case (unsigned int)DiagnosticTestType::PrintTestReport:
		return PrintTestReport(gb, reply, buf);

	case (unsigned int)DiagnosticTestType::OutputBufferStarvation:
		{
			OutputBuffer *buf;
			while (OutputBuffer::Allocate(buf)) { }
			OutputBuffer::ReleaseAll(buf);
		}
		break;

	case (unsigned int)DiagnosticTestType::SetWriteBuffer:
#if SAME70
		//TODO set cache to write-back instead
		reply.copy("Write buffer not supported on this processor");
		return GCodeResult::error;
#else
		if (gb.Seen('S'))
		{
			if (gb.GetUIValue() > 0)
			{
				SCnSCB->ACTLR &= ~SCnSCB_ACTLR_DISDEFWBUF_Msk;		// enable write buffer
			}
			else
			{
				SCnSCB->ACTLR |= SCnSCB_ACTLR_DISDEFWBUF_Msk;		// disable write buffer
			}
		}
		else
		{
			reply.printf("Write buffer is %s", (SCnSCB->ACTLR & SCnSCB_ACTLR_DISDEFWBUF_Msk) ? "disabled" : "enabled");
		}
		break;
#endif

	case (unsigned int)DiagnosticTestType::TestWatchdog:
		if (!gb.DoDwellTime(1000))								// wait a second to allow the response to be sent back to the web server, otherwise it may retry
		{
			return GCodeResult::notFinished;
		}
		deliberateError = true;
		SysTick->CTRL &= ~(SysTick_CTRL_TICKINT_Msk);			// disable the system tick interrupt so that we get a watchdog timeout reset
		break;

	case (unsigned int)DiagnosticTestType::TestSpinLockup:
		if (!gb.DoDwellTime(1000))								// wait a second to allow the response to be sent back to the web server, otherwise it may retry
		{
			return GCodeResult::notFinished;
		}
		deliberateError = true;
		debugCode = d;											// tell the Spin function to loop
		break;

	case (unsigned int)DiagnosticTestType::TestSerialBlock:		// write an arbitrary message via debugPrintf()
		if (!gb.DoDwellTime(1000))								// wait a second to allow the response to be sent back to the web server, otherwise it may retry
		{
			return GCodeResult::notFinished;
		}
		deliberateError = true;
		debugPrintf("Diagnostic Test\n");
		break;

	case (unsigned int)DiagnosticTestType::DivideByZero:		// do an integer divide by zero to test exception handling
		if (!gb.DoDwellTime(1000))								// wait a second to allow the response to be sent back to the web server, otherwise it may retry
		{
			return GCodeResult::notFinished;
		}
		deliberateError = true;
		(void)RepRap::DoDivide(1, 0);							// call function in another module so it can't be optimised away
		break;

	case (unsigned int)DiagnosticTestType::UnalignedMemoryAccess: // disable unaligned memory accesses
		if (!gb.DoDwellTime(1000))								// wait a second to allow the response to be sent back to the web server, otherwise it may retry
		{
			return GCodeResult::notFinished;
		}
		deliberateError = true;
		SCB->CCR |= SCB_CCR_UNALIGN_TRP_Msk;					// by default, unaligned memory accesses are allowed, so change that
		// We don't actually generate a fault any more, instead we let this function identify existing unaligned accesses in the code
		break;

	case (unsigned int)DiagnosticTestType::BusFault:
#if SAME70 && !USE_MPU
		Message(WarningMessage, "There is no abort area on the SAME70 with MPU disabled");
#else
		if (!gb.DoDwellTime(1000))								// wait a second to allow the response to be sent back to the web server, otherwise it may retry
		{
			return GCodeResult::notFinished;
		}
		deliberateError = true;
		RepRap::GenerateBusFault();
#endif
		break;

	case (unsigned int)DiagnosticTestType::AccessMemory:
		{
			gb.MustSee('A');
			uint32_t address = (uint32_t)gb.GetIValue();		// allow negative values here so that we can read high addresses
			unsigned int numValues = (gb.Seen('R')) ? gb.GetUIValue() : 1;
			int32_t val;
			bool dummy;
			deliberateError = true;								// in case the memory access causes a fault
			if (gb.TryGetIValue('V', val, dummy))				// allow negative values so that we can use values like 0xffffffff
			{
				while (numValues != 0)
				{
					*reinterpret_cast<uint32_t*>(address) = (uint32_t)val;
					address += 4;
					--numValues;
				}
				__DSB();										// allow the write to complete in case it raises a fault
			}
			else
			{
				reply.printf("%08" PRIx32 ":", address);
				while (numValues != 0)
				{
					reply.catf(" %08" PRIx32, *reinterpret_cast<const uint32_t*>(address));
					address += 4;
					--numValues;
				}
			}
			deliberateError = false;
		}
		break;

	case (unsigned int)DiagnosticTestType::PrintMoves:
		DDA::PrintMoves();
		break;

	case (unsigned int)DiagnosticTestType::TimeCalculations:	// Show the square root calculation time. Caution: may disable interrupt for several tens of microseconds.
		{
			constexpr uint32_t iterations = 100;				// use a value that divides into one million
			bool ok1 = true;
			uint32_t tim1 = 0;
			for (uint32_t i = 0; i < iterations; ++i)
			{
				const uint32_t num1 = 0x7fffffff - (67 * i);
				const uint64_t sq = (uint64_t)num1 * num1;
				const uint32_t num1a = TimedSqrt(sq, tim1);
				if (num1a != num1)
				{
					ok1 = false;
				}
			}

			bool ok2 = true;
			uint32_t tim2 = 0;
			for (uint32_t i = 0; i < iterations; ++i)
			{
				const uint32_t num2 = 0x0000ffff - (67 * i);
				const uint64_t sq = (uint64_t)num2 * num2;
				const uint32_t num2a = TimedSqrt(sq, tim2);
				if (num2a != num2)
				{
					ok2 = false;
				}
			}

			// We also time floating point square root so we can compare it with sine/cosine in order to consider various optimisations
			bool ok3 = true;
			uint32_t tim3 = 0;
			float val = 10000.0;
			for (unsigned int i = 0; i < iterations; ++i)
			{
				IrqDisable();
				asm volatile("":::"memory");
				uint32_t now1 = SysTick->VAL;
				const float nval = fastSqrtf(val);
				uint32_t now2 = SysTick->VAL;
				asm volatile("":::"memory");
				IrqEnable();
				now1 &= 0x00FFFFFF;
				now2 &= 0x00FFFFFF;
				tim3 += ((now1 > now2) ? now1 : now1 + (SysTick->LOAD & 0x00FFFFFF) + 1) - now2;
				if (nval != sqrtf(val))
				{
					ok3 = false;
					if (reprap.Debug(Module::Platform))
					{
						debugPrintf("val=%.7e sq=%.7e sqrtf=%.7e\n", (double)val, (double)nval, (double)sqrtf(val));
					}
				}
				val = nval;
			}

			reply.printf("Square roots: 62-bit %.2fus %s, 32-bit %.2fus %s, float %.2fus %s",
						(double)((float)(tim1 * (1'000'000/iterations))/SystemCoreClock), (ok1) ? "ok" : "ERROR",
							(double)((float)(tim2 * (1'000'000/iterations))/SystemCoreClock), (ok2) ? "ok" : "ERROR",
								(double)((float)(tim3 * (1'000'000/iterations))/SystemCoreClock), (ok3) ? "ok" : "ERROR");
		}

		// We now also time sine and cosine in the same test
		{
			uint32_t tim1 = 0;
			constexpr uint32_t iterations = 100;				// use a value that divides into one million
			for (unsigned int i = 0; i < iterations; ++i)
			{
				const float angle = 0.01 * i;

				IrqDisable();
				asm volatile("":::"memory");
				uint32_t now1 = SysTick->VAL;
				(void)RepRap::SinfCosf(angle);
				uint32_t now2 = SysTick->VAL;
				asm volatile("":::"memory");
				IrqEnable();
				now1 &= 0x00FFFFFF;
				now2 &= 0x00FFFFFF;
				tim1 += ((now1 > now2) ? now1 : now1 + (SysTick->LOAD & 0x00FFFFFF) + 1) - now2;
			}

			// We no longer calculate sin and cos for doubles because it pulls in those library functions, which we don't otherwise need
			reply.lcatf("Float sine + cosine: %.2fus", (double)((float)(tim1 * (1'000'000/iterations))/SystemCoreClock));
		}

		break;

	case (unsigned int)DiagnosticTestType::TimeSDWrite:
#if HAS_MASS_STORAGE
		return reprap.GetGCodes().StartSDTiming(gb, reply);
#else
		reply.copy("No SD card interface available");
		return GCodeResult::errorNotSupported;
#endif

	case (unsigned int)DiagnosticTestType::PrintObjectSizes:
		reply.printf(
				"Task %u, DDA %u, DDARing %u, DM %u, MS %u, Tool %u, GCodeBuffer %u, heater %u, mbox %u"
#if HAS_NETWORKING
				", HTTP resp %u, FTP resp %u, Telnet resp %u"
#endif
				, sizeof(TaskBase), sizeof(DDA), sizeof(DDARing), sizeof(DriveMovement), sizeof(MoveSegment), sizeof(Tool), sizeof(GCodeBuffer), sizeof(Heater), sizeof(MessageBox)
#if HAS_NETWORKING
				, sizeof(HttpResponder), sizeof(FtpResponder), sizeof(TelnetResponder)
#endif
			);
		break;

	case (unsigned int)DiagnosticTestType::PrintObjectAddresses:
		MessageF(MessageType::GenericMessage,
					"Platform %08" PRIx32 "-%08" PRIx32
#if HAS_SBC_INTERFACE
					"\nSbcInterface %08" PRIx32 "-%08" PRIx32
#endif
					"\nNetwork %08" PRIx32 "-%08" PRIx32
					"\nGCodes %08" PRIx32 "-%08" PRIx32
					"\nMove %08" PRIx32 "-%08" PRIx32
					"\nHeat %08" PRIx32 "-%08" PRIx32
					"\n"
					, reinterpret_cast<uint32_t>(this), reinterpret_cast<uint32_t>(this) + sizeof(Platform) - 1
#if HAS_SBC_INTERFACE
					, reinterpret_cast<uint32_t>(&reprap.GetSbcInterface())
					, (reinterpret_cast<uint32_t>(&reprap.GetSbcInterface()) == 0) ? 0 : reinterpret_cast<uint32_t>(&reprap.GetSbcInterface()) + sizeof(SbcInterface)
#endif
					, reinterpret_cast<uint32_t>(&reprap.GetNetwork()), reinterpret_cast<uint32_t>(&reprap.GetNetwork()) + sizeof(Network) - 1
					, reinterpret_cast<uint32_t>(&reprap.GetGCodes()), reinterpret_cast<uint32_t>(&reprap.GetGCodes()) + sizeof(GCodes) - 1
					, reinterpret_cast<uint32_t>(&reprap.GetMove()), reinterpret_cast<uint32_t>(&reprap.GetMove()) + sizeof(Move) - 1
					, reinterpret_cast<uint32_t>(&reprap.GetHeat()), reinterpret_cast<uint32_t>(&reprap.GetHeat()) + sizeof(Heat) - 1
				);

		MessageF(MessageType::GenericMessage,
					"PrintMonitor %08" PRIx32 "-%08" PRIx32
					"\nFansManager %08" PRIx32 "-%08" PRIx32
#if SUPPORT_IOBITS
					"\nPortControl %08" PRIx32 "-%08" PRIx32
#endif
#if SUPPORT_DIRECT_LCD
					"\nDisplay %08" PRIx32 "-%08" PRIx32
#endif
#if SUPPORT_CAN_EXPANSION
					"\nExpansionManager %08" PRIx32 "-%08" PRIx32
#endif
					"\n"
					, reinterpret_cast<uint32_t>(&reprap.GetPrintMonitor()), reinterpret_cast<uint32_t>(&reprap.GetPrintMonitor()) + sizeof(PrintMonitor) - 1
					, reinterpret_cast<uint32_t>(&reprap.GetFansManager()), reinterpret_cast<uint32_t>(&reprap.GetFansManager()) + sizeof(FansManager) - 1
#if SUPPORT_IOBITS
					, reinterpret_cast<uint32_t>(&reprap.GetPortControl()), reinterpret_cast<uint32_t>(&reprap.GetPortControl()) + sizeof(PortControl) - 1
#endif
#if SUPPORT_DIRECT_LCD
					, reinterpret_cast<uint32_t>(&reprap.GetDisplay()), reinterpret_cast<uint32_t>(&reprap.GetDisplay()) + sizeof(Display) - 1
#endif
#if SUPPORT_CAN_EXPANSION
					, reinterpret_cast<uint32_t>(&reprap.GetExpansion()), reinterpret_cast<uint32_t>(&reprap.GetExpansion()) + sizeof(ExpansionManager) - 1
#endif
				);
		break;

	case (unsigned int)DiagnosticTestType::TimeCRC32:
		{
			const size_t length = (gb.Seen('S')) ? gb.GetUIValue() : 1024;
			CRC32 crc;
			IrqDisable();
			asm volatile("":::"memory");
			uint32_t now1 = SysTick->VAL;
			crc.Update(
#if SAME5x
						reinterpret_cast<const char *_ecv_array>(HSRAM_ADDR),
#else
						reinterpret_cast<const char *_ecv_array>(IRAM_ADDR),		// for the SAME70 this is in the non-cacheable RAM, which is the usual case when computing a CRC
#endif
						length);
			uint32_t now2 = SysTick->VAL;
			asm volatile("":::"memory");
			IrqEnable();
			now1 &= 0x00FFFFFF;
			now2 &= 0x00FFFFFF;
			uint32_t tim1 = ((now1 > now2) ? now1 : now1 + (SysTick->LOAD & 0x00FFFFFF) + 1) - now2;
			reply.printf("CRC of %u bytes took %.2fus", length, (double)((1'000'000.0f * (float)tim1)/(float)SystemCoreClock));
		}
		break;

	case (unsigned int)DiagnosticTestType::TimeGetTimerTicks:
		{
			unsigned int i = 100;
			IrqDisable();
			asm volatile("":::"memory");
			uint32_t now1 = SysTick->VAL;
			do
			{
				--i;
				(void)StepTimer::GetTimerTicks();
			} while (i != 0);
			uint32_t now2 = SysTick->VAL;
			asm volatile("":::"memory");
			IrqEnable();
			now1 &= 0x00FFFFFF;
			now2 &= 0x00FFFFFF;
			uint32_t tim1 = ((now1 > now2) ? now1 : now1 + (SysTick->LOAD & 0x00FFFFFF) + 1) - now2;
			reply.printf("Reading step timer 100 times took %.2fus", (double)((1'000'000.0f * (float)tim1)/(float)SystemCoreClock));
		}

#if SUPPORT_CAN_EXPANSION
		// Also check the correspondence between the CAN timestamp timer and the step clock
		{
			uint32_t startClocks, endClocks;
			uint16_t startTimeStamp, endTimeStamp;
			{
				AtomicCriticalSectionLocker lock;
				startClocks = StepTimer::GetTimerTicks();
				startTimeStamp = CanInterface::GetTimeStampCounter();
			}
			delay(2);
			{
				AtomicCriticalSectionLocker lock;
				endClocks = StepTimer::GetTimerTicks();
				endTimeStamp = CanInterface::GetTimeStampCounter();
			}
# if SAME70
			const uint32_t tsDiff = (endTimeStamp - startTimeStamp) & 0xFFFF;
# else
			const uint32_t tsDiff = (((endTimeStamp - startTimeStamp) & 0xFFFF) * CanInterface::GetTimeStampPeriod()) >> 6;
# endif
			reply.lcatf("Clock diff %" PRIu32 ", ts diff %" PRIu32, endClocks - startClocks, tsDiff);
		}
#endif
		break;

#if HAS_VOLTAGE_MONITOR
	case (unsigned int)DiagnosticTestType::UndervoltageEvent:
		reprap.GetGCodes().LowVoltagePause();
		break;
#endif

#ifdef DUET_NG
	case (unsigned int)DiagnosticTestType::PrintExpanderStatus:
		reply.printf("Expander status %04X\n", DuetExpansion::DiagnosticRead());
		break;
#endif

	default:
		break;
	}

	return GCodeResult::ok;
}

// Get the index of the averaging filter for an analog port.
// Note, the Thermistor code assumes that this is also the thermistor input number
int Platform::GetAveragingFilterIndex(const IoPort& port) const noexcept
{
	for (size_t i = 0; i < ARRAY_SIZE(TEMP_SENSE_PINS); ++i)
	{
		if (port.GetPin() == TEMP_SENSE_PINS[i])
		{
			return (int)i;
		}
	}
	return -1;
}

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE

// Write the platform parameters to file
bool Platform::WritePlatformParameters(FileStore *f, bool includingG31) const noexcept
{
	return endstops.WriteZProbeParameters(f, includingG31);
}

#endif

//-----------------------------------------------------------------------------------------------------

// USB port functions

void Platform::AppendUsbReply(OutputBuffer *buffer) noexcept
{
	if (!SERIAL_MAIN_DEVICE.IsConnected())
	{
		// If the serial USB line is not open, discard the message right away
		OutputBuffer::ReleaseAll(buffer);
	}
	else
	{
		// Else append incoming data to the stack
		MutexLocker lock(usbMutex);
		usbOutput.Push(buffer);
	}
}

// Aux port functions

GCodeResult Platform::HandleM575(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	// Get the channel specified by the command and the corresponding GCode buffer
	const size_t chan = gb.GetLimitedUIValue('P', NumSerialChannels);
	GCodeBuffer * const gbp = reprap.GetGCodes().GetSerialGCodeBuffer(chan);

#if HAS_AUX_DEVICES
	// If a baud rate has been provided, just store it for later use
	const uint32_t baudRate = (gb.Seen('B')) ? gb.GetUIValue() : 0;
#endif

	// See if a mode is provided
	if (gb.Seen('S'))
	{
		// Translation of M575 S parameter to AuxMode
		static constexpr AuxMode modes[] =
		{
			AuxMode::panelDue,			// basic PanelDue mode,
			AuxMode::panelDue,			// PanelDue mode with CRC or checksum required (default)
			AuxMode::raw,				// basic raw mode
			AuxMode::raw,				// raw mode with CRC or checksum required
			AuxMode::panelDue,			// PanelDue mode with CRC required
			AuxMode::disabled,			// was unused, now treated as disabled
			AuxMode::raw,				// raw mode with CRC required
			AuxMode::device,			// Modbus/Uart mode
		};

		const uint32_t val = gb.GetLimitedUIValue('S', ARRAY_SIZE(modes));
		AuxMode newMode = modes[val];
		if (gbp != nullptr)
		{
			gbp->Disable();							// disable I/O for this buffer
		}

		SetCommsProperties(chan, val);

#if HAS_AUX_DEVICES
		if (chan != 0)
		{
			AuxDevice& dev = auxDevices[chan - 1];
			if (newMode == AuxMode::device)
			{
# if SUPPORT_MODBUS_RTU
				if (gb.Seen('C'))
				{
					String<StringLength50> portName;
					gb.GetQuotedString(portName.GetRef(), false);
					if (!dev.ConfigureDirectionPort(portName.c_str(), reply))
					{
						return GCodeResult::error;
					}
				}
#  if defined(DUET3_MB6XD)
				else if (chan == 2 && board >= BoardType::Duet3_6XD_v102)
				{
					if (!dev.ConfigureDirectionPort(ModbusTxPinName, reply))
					{
						return GCodeResult::error;
					}
				}
#  endif
# endif
			}
			if (baudRate != 0)
			{
				dev.SetBaudRate(baudRate);
			}
			dev.SetMode(newMode);
		}
#endif

		if (   gbp != nullptr
			&& newMode != AuxMode::disabled
			&& newMode != AuxMode::device
		   )
		{
			gbp->Enable(val);						// enable I/O and set the CRC and checksum requirements, also sets Marlin or PanelDue compatibility
		}
	}
#if HAS_AUX_DEVICES
	else if (baudRate != 0)
	{
		if (chan != 0)
		{
			auxDevices[chan - 1].SetBaudRate(baudRate);
			ResetChannel(chan);
		}
	}
#endif
	else
	{
		// Just print the existing configuration
		const uint32_t cp = GetCommsProperties(chan);
		const char *crcMode = (cp & 4) ? "requires CRC"
								: (cp & 1) ? "requires checksum or CRC"
									: "does not require checksum or CRC";
#if HAS_AUX_DEVICES
		if (chan != 0)
		{
			if (!IsAuxEnabled(chan - 1)
				&& (chan >= NumSerialChannels || auxDevices[chan - 1].GetMode() != AuxMode::device)
			   )
			{
				reply.printf("Channel %u is disabled", chan);
			}
			else
			{
				const AuxDevice& dev = auxDevices[chan - 1];
				const char *modeString = (dev.GetMode() == AuxMode::device) ? "Device / modbus RTU" :
											(IsAuxRaw(chan - 1)) ? "raw"
												: "PanelDue";
				reply.printf("Channel %d: baud rate %" PRIu32 ", %s mode, ", chan, GetBaudRate(chan), modeString);
				if (dev.GetMode() == AuxMode::device)
				{
# if SUPPORT_MODBUS_RTU
					reply.cat("Modbus Tx/!Rx port ");
					dev.AppendDirectionPortName(reply);
# endif
				}
				else
				{
					reply.cat(crcMode);
				}
			}
		}
		else
#endif
		{
			reply.printf("Channel 0 (USB): %s", crcMode);
			if (SERIAL_MAIN_DEVICE.IsConnected())
			{
				reply.cat(", connected");
			}
		}
	}
	return GCodeResult::ok;
}

bool Platform::IsAuxEnabled(size_t auxNumber) const noexcept
{
#if HAS_AUX_DEVICES
	return auxNumber < ARRAY_SIZE(auxDevices) && auxDevices[auxNumber].IsEnabledForGCodeIo();
#else
	return false;
#endif
}

bool Platform::IsAuxRaw(size_t auxNumber) const noexcept
{
#if HAS_AUX_DEVICES
	return auxNumber >= ARRAY_SIZE(auxDevices) || auxDevices[auxNumber].IsRaw();
#else
	return true;
#endif
}

static inline uint32_t GetAddress(GCodeBuffer& gb)
{
	uint32_t address = 0;
	if (gb.GetCommandFraction() < 2)
	{
		gb.MustSee('A');
		address = gb.GetUIValue();
	}
	return address;
}

/**
 * Converts a single byte of hex value to its ASCII hex representation.
 *
 * @param hex The hex value to convert.
 * @param asciiHex The buffer to store the ASCII hex representation. The buffer must be at least 2 bytes long.
 *
 * @throws None
 */
static inline void ConvertHexToAsciiHex(uint8_t hex, uint8_t asciiHex[2])
{
	uint8_t hexSplit[2] = {0};
	hexSplit[0] = hex >> 4;	  // Get the upper 4 bits
	hexSplit[1] = hex & 0x0F; // Get the lower 4 bits

	for (size_t i = 0; i < 2; i++)
	{
		uint8_t h = hexSplit[i];

		// If the value is between 0x00 and 0x09, it is a number, otherwise it is a letter
		if (h >= 0 && h <= 9)
		{
			asciiHex[i] = h + 0x30; // ASCII '0' is 0x30
		}
		else if (h >= 0x0A && h <= 0x0F)
		{
			asciiHex[i] = h + 0x41 - 0x0A; // ASCII 'A' is 0x41, subtracting 0x0A shifts the value to 'A'
		}
	}
}

static inline void CalculateNordsonUltimusVCheckSum(uint8_t* data, size_t len, uint8_t checksum[2])
{
	uint16_t sum = 0;
	for (size_t i = 0; i < len; i++)
	{
		sum -= data[i];
	}

	ConvertHexToAsciiHex(sum & 0xFF, checksum); // take last byte of sum and convert to ascii hex
}

// Handle M260 and M260.1 - send and possibly receive via I2C, or send via Modbus
GCodeResult Platform::SendI2cOrModbus(GCodeBuffer& gb, const StringRef &reply) THROWS(GCodeException)
{
	// Get the slave address and bytes or words to send

# if defined(I2C_IFACE) || SUPPORT_MODBUS_RTU
	const uint32_t address = GetAddress(gb);
#endif

	int32_t values[MaxI2cOrModbusValues] = {0};
	size_t numToSend = 0;

	if (gb.Seen('B'))
	{
		numToSend = MaxI2cOrModbusValues;
		gb.GetIntArray(values, numToSend, false);
	}
	else if (gb.Seen('S'))
	{
		String<MaxI2cOrModbusValues> str;
		gb.GetQuotedString(str.GetRef(), false);

		numToSend = str.strlen();
		if (numToSend >= MaxI2cOrModbusValues)
		{
			reply.copy("input string too long");
			return GCodeResult::error;
		}

		for (size_t i = 0; i < numToSend; i++)
		{
			values[i] = (int32_t)str[i];
		}
	}
	else if (gb.GetCommandFraction() > 0)
	{
		reply.copy("missing parameter 'B' or 'S'");
		return GCodeResult::error;
	}

	switch (gb.GetCommandFraction())
	{
# if defined(I2C_IFACE)
	case 0:		// I2C
	case -1:
		{
			uint32_t numToReceive = 0;
			bool seenR;
			gb.TryGetUIValue('R', numToReceive, seenR);

			if (numToSend + numToReceive > MaxI2cOrModbusValues)
			{
				numToReceive = MaxI2cOrModbusValues - numToSend;
			}
			uint8_t bValues[MaxI2cOrModbusValues] = {0};
			for (size_t i = 0; i < numToSend; ++i)
			{
				bValues[i] = (uint8_t)values[i];
			}

			I2C::Init();
			const size_t bytesTransferred = I2C::Transfer(address, bValues, numToSend, numToReceive);

			if (bytesTransferred < numToSend)
			{
				reply.copy("I2C transmission error");
				return GCodeResult::error;
			}
			else if (numToReceive != 0)
			{
				reply.copy("Received");
				if (bytesTransferred == numToSend)
				{
					reply.cat(" nothing");
				}
				else
				{
					for (size_t i = numToSend; i < bytesTransferred; ++i)
					{
						reply.catf(" %02x", bValues[i]);
					}
				}
			}
			return (bytesTransferred == numToSend + numToReceive) ? GCodeResult::ok : GCodeResult::error;
		}
# endif

# if SUPPORT_MODBUS_RTU
	case 1:		// Modbus
		{
			const size_t auxChannel = gb.GetLimitedUIValue('P', 1, NumSerialChannels) - 1;
			if (auxDevices[auxChannel].GetMode() != AuxMode::device)
			{
				reply.copy("Port has not been set to device mode");
				return GCodeResult::error;
			}

			const uint16_t firstRegister = gb.GetLimitedUIValue('R', 1u << 16);
			const uint8_t function = (gb.Seen('F')) ? gb.GetLimitedUIValue('F', 5, 17) : 16;	// default to Modbus function Write Multiple Registers but also allow Write Coils, Write Single Coil
			uint16_t registersToSend[MaxI2cOrModbusValues];
			switch (function)
			{
			case (uint8_t)ModbusFunction::writeSingleCoil:
				if (numToSend != 1)
				{
					reply.copy("Invalid Modbus data");
					return GCodeResult::error;
				}
				registersToSend[0] = (values[0] == 0) ? 0 : 0xFF00;
				break;

			case (uint8_t)ModbusFunction::writeSingleRegister:
				if (numToSend != 1)
				{
					reply.copy("Invalid Modbus data");
					return GCodeResult::error;
				}
				registersToSend[0] = (uint16_t)values[0];
				break;

			case (uint8_t)ModbusFunction::writeMultipleCoils:
				memset(registersToSend, 0, sizeof(registersToSend));
				for (size_t i = 0; i < numToSend; ++i)
				{
					if (values[i] != 0)
					{
						registersToSend[i/16] |= 1u << (i % 16);
					}
				}
				break;

			case (uint8_t)ModbusFunction::writeMultipleRegisters:
				for (size_t i = 0; i < numToSend; ++i)
				{
					registersToSend[i] = (uint16_t)values[i];
				}
				break;

			default:
				reply.copy("Invalid Modbus function number");
				return GCodeResult::error;
			}

			GCodeResult rslt = auxDevices[auxChannel].SendModbusRegisters(address, function, firstRegister, numToSend, (const uint8_t*)registersToSend);
			if (rslt == GCodeResult::ok)
			{
				do
				{
					delay(2);
					rslt = auxDevices[auxChannel].CheckModbusResult();
				} while (rslt == GCodeResult::notFinished);
				if (rslt != GCodeResult::ok)
				{
					reply.copy("no or bad response from Modbus device");
				}
			}
			else
			{
				reply.copy("couldn't initiate Modbus transaction");
			}
			return rslt;
		}
# endif

# if HAS_AUX_DEVICES
	case 2:
	{
		const size_t auxChannel = gb.GetLimitedUIValue('P', 1, NumSerialChannels) - 1;
		if (auxDevices[auxChannel].GetMode() != AuxMode::device)
		{
			reply.copy("Port has not been set to device mode");
			return GCodeResult::error;
		}

		uint8_t data[MaxI2cOrModbusValues] = {0};

		for (size_t i = 0; i < numToSend; i++)
		{
			data[i] = (uint8_t)values[i];
		}

		GCodeResult rslt = auxDevices[auxChannel].SendUartData(data, numToSend);
		if (rslt != GCodeResult::ok)
		{
			reply.copy("couldn't initiate Uart transaction");
		}
		return rslt;
	}

	case 3: // Nordson Ultimus V https://www.manualslib.com/manual/2917329/Nordson-Ultimus-V.html?page=46#manual
	{
		const size_t auxChannel = gb.GetLimitedUIValue('P', 1, NumSerialChannels) - 1;
		if (auxDevices[auxChannel].GetMode() != AuxMode::device)
		{
			reply.copy("Port has not been set to device mode");
			return GCodeResult::error;
		}

		AuxDevice& dev = auxDevices[auxChannel];

		// Send `ENQ`
		uint8_t handshake = 0x05;
		GCodeResult rslt = dev.SendUartData(&handshake, 1);
		if (rslt != GCodeResult::ok)
		{
			reply.copy("failed to send ENQ");
			return rslt;
		}

		// Receive ACK
		rslt = dev.ReadUartData(&handshake, 1);
		if (rslt != GCodeResult::ok)
		{
			reply.copy("failed to receive data when expecting ACK");
			return rslt;
		}

		if (handshake != 0x06) // should received `ACK`
		{
			reply.copy("Ultimus V did not send ACK");
			return rslt;
		}

		uint8_t data[MaxI2cOrModbusValues] = {0};
		uint8_t numBytesAsciiHex[2] = {0};
		ConvertHexToAsciiHex(numToSend, numBytesAsciiHex);

		data[0] = 0x02;				   // `STX`
		data[1] = numBytesAsciiHex[0]; // Num bytes
		data[2] = numBytesAsciiHex[1]; // Num bytes

		for (size_t i = 0; i < numToSend; i++)
		{
			data[i + 3] = (uint8_t)values[i];
		}

		uint8_t checksum[2] = {0};
		CalculateNordsonUltimusVCheckSum(&data[1], numToSend + 2, checksum);

		data[numToSend + 3] = checksum[0];
		data[numToSend + 4] = checksum[1];
		data[numToSend + 5] = 0x03; // `ETX`

		rslt = dev.SendUartData(data, numToSend + 6);
		if (rslt != GCodeResult::ok)
		{
			reply.copy("Handshake complete but failed to send message");
			return rslt;
		}

		// Receive success/failure
		for (size_t i = 0; i < MaxI2cOrModbusValues; i++)
		{
			data[i] = 0;
		}

		rslt = dev.ReadUartData(data, 8);
		if (rslt != GCodeResult::ok)
		{
			reply.copy("Sent message but failed to receive success/failure");
			return rslt;
		}

		static constexpr uint8_t success[] = {0x02, 0x30, 0x32, 0x41, 0x30, 0x32, 0x44, 0x03};
		static constexpr uint8_t failure[] = {0x02, 0x30, 0x32, 0x41, 0x32, 0x32, 0x42, 0x03};

		bool isSuccess = true;
		bool isFailure = true;
		for (size_t i = 0; i < 8; i++)
		{
			if (data[i] != success[i])
			{
				isSuccess = false;
			}
			if (data[i] != failure[i])
			{
				isFailure = false;
			}
		}

		if (isFailure || !isSuccess)
		{
			reply.copy("Nordson Ultimus V failed to process message");
			rslt = GCodeResult::error;
		}

		// End of Transmission
		uint8_t eot = 0x04;
		dev.SendUartData(&eot, 1);	// It is probably of little importance that this is sent since the datasheet says receiving a STX will start a new command. But better safe than sorry.

		return rslt;
	}
#endif

	default:
		return GCodeResult::errorNotSupported;
	}
}

// Handle M261 and M261.1
GCodeResult Platform::ReceiveI2cOrModbus(GCodeBuffer& gb, const StringRef &reply) THROWS(GCodeException)
{
# if defined(I2C_IFACE) || SUPPORT_MODBUS_RTU
	const uint32_t address = GetAddress(gb);
#endif

	const uint32_t numValues = gb.GetLimitedUIValue('B', 0, MaxI2cOrModbusValues + 1);
	String<MaxVariableNameLength> varName;
	bool seenV = false;
	gb.TryGetQuotedString('V', varName.GetRef(), seenV, false);
	Variable *_ecv_null resultVar = nullptr;
	if (seenV)
	{
		if (!Variable::IsValidVariableName(varName.c_str()))
		{
			reply.printf("variable '%s' is not a valid name", varName.c_str());
			return GCodeResult::error;
		}
		auto vset = WriteLockedPointer<VariableSet>(nullptr, &gb.GetVariables());
		Variable *_ecv_null const v = vset->Lookup(varName.c_str(), false);
		if (v != nullptr)
		{
			reply.printf("variable '%s' already exists", varName.c_str());
			return GCodeResult::error;
		}
		resultVar = vset->InsertNew(varName.c_str(), ExpressionValue(), gb.CurrentFileMachineState().GetBlockNesting());
	}

	switch (gb.GetCommandFraction())
	{
#if defined(I2C_IFACE)
	case 0:		// I2C
	case -1:
		{
			I2C::Init();
			uint8_t bValues[MaxI2cOrModbusValues];
			const size_t bytesRead = I2C::Transfer(address, bValues, 0, numValues);

			if (resultVar != nullptr)
			{
				if (bytesRead != 0)
				{
					resultVar->AssignArray(bytesRead, [bValues](size_t index)->ExpressionValue
											{
												return ExpressionValue((int32_t)bValues[index]);
											}
										  );
				}
			}
			else
			{
				reply.copy("Received");
				if (bytesRead == 0)
				{
					reply.cat(" nothing");
				}
				else
				{
					for (size_t i = 0; i < bytesRead; ++i)
					{
						reply.catf(" %02x", bValues[i]);
					}
				}
			}

			return (bytesRead == numValues) ? GCodeResult::ok : GCodeResult::error;
		}
#endif

#if SUPPORT_MODBUS_RTU
	case 1:		// Modbus
		{
			const size_t auxChannel = gb.GetLimitedUIValue('P', 1, NumSerialChannels) - 1;
			if (auxDevices[auxChannel].GetMode() != AuxMode::device)
			{
				reply.copy("Port has not been set to device mode");
				return GCodeResult::error;
			}

			const uint16_t firstRegister = gb.GetLimitedUIValue('R', 1u << 16);
			const uint8_t function = (gb.Seen('F')) ? gb.GetLimitedUIValue('F', 1, 5) : 4;			// default to Modbus function Read Input Registers but also allow Read Holding Registers, Read Coils, Read Inputs
			uint16_t registersToReceive[MaxI2cOrModbusValues];
			GCodeResult rslt = auxDevices[auxChannel].ReadModbusRegisters(address, function, firstRegister, numValues, (uint8_t*)registersToReceive);
			if (rslt == GCodeResult::ok)
			{
				do
				{
					delay(2);
					rslt = auxDevices[auxChannel].CheckModbusResult();
				} while (rslt == GCodeResult::notFinished);
				if (rslt == GCodeResult::ok)
				{
					switch (function)
					{
					case (uint8_t)ModbusFunction::readCoils:
					case (uint8_t)ModbusFunction::readDiscreteInputs:
						if (resultVar != nullptr)
						{
							resultVar->AssignArray(numValues, [registersToReceive](size_t index)->ExpressionValue
													{
														const bool elem = registersToReceive[index / 16] & (1u << (index % 16));
														return ExpressionValue(elem);
													}
												  );
						}
						else
						{
							reply.copy("Received");
							for (size_t i = 0; i < numValues; ++i)
							{
								reply.cat((registersToReceive[i / 16] & (1u << (i % 16))) ? " 1" : " 0");
							}
						}
						break;

					case (uint8_t)ModbusFunction::readHoldingRegisters:
					case (uint8_t)ModbusFunction::readInputRegisters:
					default:
						if (resultVar != nullptr)
						{
							resultVar->AssignArray(numValues, [registersToReceive](size_t index)->ExpressionValue
													{
														return ExpressionValue((int32_t)registersToReceive[index]);
													}
												  );
						}
						else
						{
							reply.copy("Received");
							for (size_t i = 0; i < numValues; ++i)
							{
								reply.catf(" %04x", registersToReceive[i]);
							}
						}
						break;
					}
				}
				else
				{
					reply.copy("no or bad response from Modbus device");
				}
			}
			else
			{
				reply.copy("couldn't initiate Modbus transaction");
			}
			return rslt;
		}
#endif

#if HAS_AUX_DEVICES
	case 2:		// Uart
		{
			const size_t auxChannel = gb.GetLimitedUIValue('P', 1, NumSerialChannels) - 1;
			if (auxDevices[auxChannel].GetMode() != AuxMode::device)
			{
				reply.copy("Port has not been set to device mode");
				return GCodeResult::error;
			}

			uint8_t dataReceived[MaxI2cOrModbusValues];
			GCodeResult rslt = auxDevices[auxChannel].ReadUartData(dataReceived, numValues);
			if (rslt == GCodeResult::ok)
			{
				if (resultVar != nullptr)
				{
					resultVar->AssignArray(numValues, [dataReceived](size_t index)->ExpressionValue
											{
												const uint32_t elem = (uint32_t)dataReceived[index];
												return ExpressionValue(elem);
											}
										  );
				}
				else
				{
					reply.copy("Received (hex)");
					for (size_t i = 0; i < numValues; ++i)
					{
						reply.catf(" %02x", dataReceived[i]);
					}
				}
			}
			else
			{
				reply.copy("couldn't initiate Uart read");
			}
			return rslt;
		}
#endif

	default:
		return GCodeResult::errorNotSupported;
	}
}

#if defined(DUET_NG) && HAS_SBC_INTERFACE

// Enable the PanelDue port so that the ATE can test the board
void Platform::EnablePanelDuePort() noexcept
{
	auxDevices[0].SetBaudRate(57600);
	auxDevices[0].SetMode(AuxMode::panelDue);
	SetCommsProperties(1, 1);
	reprap.GetGCodes().GetSerialGCodeBuffer(1)->Enable(1);
}

#endif

#if SUPPORT_PANELDUE_FLASH
void Platform::InitPanelDueUpdater() noexcept
{
	if (panelDueUpdater == nullptr)
	{
		panelDueUpdater = new PanelDueUpdater();
	}
}
#endif

void Platform::AppendAuxReply(size_t auxNumber, const char *_ecv_array msg, bool rawMessage) noexcept
{
#if HAS_AUX_DEVICES
	if (auxNumber < ARRAY_SIZE(auxDevices))
	{
		// Don't send anything to PanelDue while we are flashing it
		if (auxNumber == 0 && reprap.GetGCodes().IsFlashingPanelDue())
		{
			return;
		}
		auxDevices[auxNumber].AppendAuxReply(msg, rawMessage);
	}
#endif
}

void Platform::AppendAuxReply(size_t auxNumber, OutputBuffer *reply, bool rawMessage) noexcept
{
#if HAS_AUX_DEVICES
	if (auxNumber < ARRAY_SIZE(auxDevices))
	{
		// Don't send anything to PanelDue while we are flashing it
		if (auxNumber == 0 && reprap.GetGCodes().IsFlashingPanelDue())
		{
			OutputBuffer::ReleaseAll(reply);
			return;
		}
		auxDevices[auxNumber].AppendAuxReply(reply, rawMessage);
	}
	else
#endif
	{
		OutputBuffer::ReleaseAll(reply);
	}
}

// Send the specified message to the specified destinations. The Error and Warning flags have already been handled.
void Platform::RawMessage(MessageType type, const char *_ecv_array message) noexcept
{
#if HAS_MASS_STORAGE
	// Deal with logging
	if (logger != nullptr)
	{
		logger->LogMessage(realTime, message, type);
	}
#endif

	// Send the message to the destinations
	if ((type & ImmediateAuxMessage) != 0)
	{
		SendPanelDueMessage(0, message);
	}
	else if ((type & AuxMessage) != 0)
	{
		AppendAuxReply(0, message, message[0] == '{' || (type & RawMessageFlag) != 0);
	}

	if ((type & HttpMessage) != 0)
	{
		reprap.GetNetwork().HandleHttpGCodeReply(message);
	}

	if ((type & TelnetMessage) != 0)
	{
		reprap.GetNetwork().HandleTelnetGCodeReply(message);
	}

	if ((type & Aux2Message) != 0)
	{
		AppendAuxReply(1, message, message[0] == '{' || (type & RawMessageFlag) != 0);
	}

	if ((type & BlockingUsbMessage) != 0)
	{
		// Debug output sends messages in blocking mode. We now give up sending if we are close to software watchdog timeout.
		MutexLocker lock(usbMutex);
		const char *_ecv_array p = message;
		size_t len = strlen(p);
		while (SERIAL_MAIN_DEVICE.IsConnected() && len != 0 && !reprap.SpinTimeoutImminent())
		{
			const size_t written = SERIAL_MAIN_DEVICE.print(p, len);
			len -= written;
			p += written;
		}
		// We no longer flush afterwards
	}
	else if ((type & UsbMessage) != 0)
	{
		// Message that is to be sent via the USB line (non-blocking)
		MutexLocker lock(usbMutex);

		// Ensure we have a valid buffer to write to that isn't referenced for other destinations
		OutputBuffer *usbOutputBuffer = usbOutput.GetLastItem();
		if (usbOutputBuffer == nullptr || usbOutputBuffer->IsReferenced())
		{
			if (OutputBuffer::Allocate(usbOutputBuffer))
			{
				if (usbOutput.Push(usbOutputBuffer))
				{
					usbOutputBuffer->cat(message);
				}
				// else the message buffer has been released, so discard the message
			}
		}
		else
		{
			usbOutputBuffer->cat(message);		// append the message
		}
	}
}

// Note: this overload of Platform::Message does not process the special action flags in the MessageType.
// Also it treats calls to send a blocking USB message the same as ordinary USB messages,
// and calls to send an immediate LCD message the same as ordinary LCD messages
void Platform::Message(const MessageType type, OutputBuffer *buffer) noexcept
{
#if HAS_MASS_STORAGE
	// First deal with logging because it doesn't hang on to the buffer
	if (logger != nullptr)
	{
		logger->LogMessage(realTime, buffer, type);
	}
#endif

	// Now send the message to all the destinations
	unsigned int numDestinations = 0;
	if ((type & (AuxMessage | ImmediateAuxMessage)) != 0)	{ ++numDestinations; }
#ifdef SERIAL_AUX2_DEVICE
	if ((type & Aux2Message) != 0)							{ ++numDestinations; }
#endif
	if ((type & (UsbMessage | BlockingUsbMessage)) != 0)	{ ++numDestinations; }
	if ((type & HttpMessage) != 0)							{ ++numDestinations; }
	if ((type & TelnetMessage) != 0)						{ ++numDestinations; }
#if HAS_SBC_INTERFACE
	if (reprap.UsingSbcInterface() && ((type & GenericMessage) == GenericMessage || (type & BinaryCodeReplyFlag) != 0)) { ++numDestinations; }
#endif

	if (numDestinations == 0)
	{
		OutputBuffer::ReleaseAll(buffer);
	}
	else
	{
		buffer->IncreaseReferences(numDestinations - 1);

		if ((type & (AuxMessage | ImmediateAuxMessage)) != 0)
		{
			AppendAuxReply(0, buffer, ((*buffer)[0] == '{') || (type & RawMessageFlag) != 0);
		}

#ifdef SERIAL_AUX2_DEVICE
		if ((type & Aux2Message) != 0)
		{
			AppendAuxReply(1, buffer, ((*buffer)[0] == '{') || (type & RawMessageFlag) != 0);
		}
#endif

		if ((type & HttpMessage) != 0)
		{
			reprap.GetNetwork().HandleHttpGCodeReply(buffer);
		}

		if ((type & TelnetMessage) != 0)
		{
			reprap.GetNetwork().HandleTelnetGCodeReply(buffer);
		}

		if ((type & (UsbMessage | BlockingUsbMessage)) != 0)
		{
			AppendUsbReply(buffer);
		}

#if HAS_SBC_INTERFACE
		if (reprap.UsingSbcInterface() && ((type & GenericMessage) == GenericMessage || (type & BinaryCodeReplyFlag) != 0))
		{
			reprap.GetSbcInterface().HandleGCodeReply(type, buffer);
		}
#endif
	}
}

void Platform::MessageV(MessageType type, const char *_ecv_array fmt, va_list vargs) noexcept
{
	String<FormatStringLength> formatString;
#if HAS_SBC_INTERFACE
	if (reprap.UsingSbcInterface() && ((type & GenericMessage) == GenericMessage || (type & BinaryCodeReplyFlag) != 0))
	{
		formatString.vprintf(fmt, vargs);
		reprap.GetSbcInterface().HandleGCodeReply(type, formatString.c_str());
		if ((type & BinaryCodeReplyFlag) != 0)
		{
			return;
		}
	}
#endif

	if ((type & ErrorMessageFlag) != 0)
	{
		formatString.copy("Error: ");
		formatString.vcatf(fmt, vargs);
	}
	else if ((type & WarningMessageFlag) != 0)
	{
		formatString.copy("Warning: ");
		formatString.vcatf(fmt, vargs);
	}
	else
	{
		formatString.vprintf(fmt, vargs);
	}

	RawMessage((MessageType)(type & ~(ErrorMessageFlag | WarningMessageFlag)), formatString.c_str());
}

void Platform::MessageF(MessageType type, const char *_ecv_array fmt, ...) noexcept
{
	va_list vargs;
	va_start(vargs, fmt);
	MessageV(type, fmt, vargs);
	va_end(vargs);
}

void Platform::Message(MessageType type, const char *_ecv_array message) noexcept
{
#if HAS_SBC_INTERFACE
	if (reprap.UsingSbcInterface() &&
		((type & BinaryCodeReplyFlag) != 0 || (type & GenericMessage) == GenericMessage || (type & LogOff) != LogOff))
	{
		reprap.GetSbcInterface().HandleGCodeReply(type, message);
		if ((type & BinaryCodeReplyFlag) != 0)
		{
			return;
		}
	}
#endif

	if ((type & (ErrorMessageFlag | WarningMessageFlag)) == 0)
	{
		RawMessage(type, message);
	}
	else
	{
#ifdef DUET3_ATE
		// FormatStringLength is too short for some ATE replies
		OutputBuffer *buf;
		if (OutputBuffer::Allocate(buf))
		{
			buf->copy(((type & ErrorMessageFlag) != 0) ? "Error: " : "Warning: ");
			buf->cat(message);
			Message(type, buf);
		}
		else
#endif
		{
			String<FormatStringLength> formatString;
			formatString.copy(((type & ErrorMessageFlag) != 0) ? "Error: " : "Warning: ");
			formatString.cat(message);
			RawMessage((MessageType)(type & ~(ErrorMessageFlag | WarningMessageFlag)), formatString.c_str());
		}
	}
}

// Send a debug message to USB using minimal stack
void Platform::DebugMessage(const char *_ecv_array fmt, va_list vargs) noexcept
{
	MutexLocker lock(usbMutex);
	vuprintf([](char c) -> bool
				{
					if (c != 0)
					{
						while (SERIAL_MAIN_DEVICE.IsConnected() && !reprap.SpinTimeoutImminent())
						{
							if (SERIAL_MAIN_DEVICE.canWrite() != 0)
							{
								SERIAL_MAIN_DEVICE.write(c);
								return true;
							}
						}
					}
					return false;
				},
				fmt,
				vargs
			);
}

#if HAS_MASS_STORAGE

// Configure logging according to the M929 command received, returning true if error
GCodeResult Platform::ConfigureLogging(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	if (gb.Seen('S'))
	{
		StopLogging();
		const auto logLevel = (LogLevel) gb.GetLimitedUIValue('S', LogLevel::off, LogLevel::NumValues);
		if (logLevel > LogLevel::off)
		{
			// Start logging
			if (logger == nullptr)
			{
				logger = new Logger(logLevel);
			}
			else
			{
				logger->SetLogLevel(logLevel);
			}

			char buf[MaxFilenameLength + 1];
			StringRef filename(buf, ARRAY_SIZE(buf));
			if (gb.Seen('P'))
			{
				gb.GetQuotedString(filename);
			}
			else
			{
				filename.copy(DEFAULT_LOG_FILE);
			}
			return logger->Start(realTime, filename, reply);
		}
	}
	else
	{
		if (logger == nullptr || !logger->IsActive())
		{
			reply.copy("Event logging is disabled");
		}
		else
		{
			const auto logLevel = logger->GetLogLevel();
			reply.printf("Event logging is enabled at log level %s", logLevel.ToString());
		}
	}
	return GCodeResult::ok;
}

// Return the log file name, or nullptr if logging is not active
const char *_ecv_array null Platform::GetLogFileName() const noexcept
{
	return (logger == nullptr) ? nullptr : logger->GetFileName();
}

#endif

const char *_ecv_array Platform::GetLogLevel() const noexcept
{
	static const LogLevel off = LogLevel::off;	// need to have an instance otherwise it will fail .ToString() below
#if HAS_MASS_STORAGE
	return (logger == nullptr) ? off.ToString() : logger->GetLogLevel().ToString();
#else
	return off.ToString();
#endif
}

// This is called from EmergencyStop. It closes the log file and stops logging.
void Platform::StopLogging() noexcept
{
#if HAS_MASS_STORAGE
	if (logger != nullptr)
	{
		logger->Stop(realTime);
	}
#endif
}

GCodeResult Platform::SetBuzzerPort(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	StopBeep();		// Stop beeping before touching the buzzer port assignment
	return buzzerPort.AssignPort(gb, reply, PinUsedBy::gpout, PinAccess::pwm) ? GCodeResult::ok : GCodeResult::error;
}

bool Platform::Beep(unsigned int freq, unsigned int ms) noexcept
{
	if (!buzzerPort.IsValid())
	{
		// Cannot beep if no buzzer port is configured
		return false;
	}

	buzzerPort.SetFrequency(freq);
	buzzerPort.WriteAnalog(0.5);
	beepTicksToGo = ms;
	return true;
}

// Caution: This may be called from an ISR as well!
void Platform::StopBeep() noexcept
{
	beepTicksToGo = 0;
	buzzerPort.WriteAnalog(0.0);
}

bool Platform::GetAtxPowerState() const noexcept
{
	const bool val = PsOnPort.ReadDigital();
	return (PsOnPort.IsHardwareInverted()) ? !val : val;
}

GCodeResult Platform::HandleM80(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	powerDownWhenFansStop = delayedPowerDown = false;				// cancel any pending power down

	GCodeResult rslt;
	if (gb.Seen('C'))
	{
		rslt = GetGCodeResultFromSuccess(PsOnPort.AssignPort(gb, reply, PinUsedBy::gpout, PinAccess::write1));
	}
	else if (PsOnPort.IsValid())
	{
		PsOnPort.WriteDigital(true);
		rslt = GCodeResult::ok;
	}
	else
	{
		reply.copy("No PS_ON port defined");
		rslt = GCodeResult::error;
	}

	reprap.StateUpdated();
	return rslt;
}

GCodeResult Platform::HandleM81(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	GCodeResult rslt;
	if (gb.Seen('C'))
	{
		rslt = GetGCodeResultFromSuccess(PsOnPort.AssignPort(gb, reply, PinUsedBy::gpout, PinAccess::write0));
	}
	else if (PsOnPort.IsValid())
	{
		// This is a power-down command
		powerDownWhenFansStop = gb.Seen('S') && gb.GetUIValue() != 0;
		delayedPowerDown = gb.Seen('D');
		if (delayedPowerDown)
		{
			whenToPowerDown = (gb.GetUIValue() * SecondsToMillis) + millis();
		}
		if (!powerDownWhenFansStop && !delayedPowerDown)
		{
			AtxPowerOff();
		}
		rslt = GCodeResult::ok;
	}
	else
	{
		reply.copy("No PS_ON port defined");
		rslt = GCodeResult::error;
	}
	reprap.StateUpdated();
	return rslt;
}

void Platform::AtxPowerOff() noexcept
{
#if HAS_MASS_STORAGE
	if (logger != nullptr)
	{
		logger->LogMessage(realTime, "Power off commanded", LogWarn);
		logger->Flush(true);
		// We don't call logger->Stop() here because we don't know whether turning off the power will work
	}
#endif

	// The PS_ON pin on Duet 3 is shared with another pin, so only try to turn off ATX power if we know that power is being controlled
	if (IsAtxPowerControlled())
	{
		PsOnPort.WriteDigital(false);
		reprap.StateUpdated();
	}
}

void Platform::SetBaudRate(size_t chan, uint32_t br) noexcept
{
#if HAS_AUX_DEVICES
	if (chan != 0 && chan < NumSerialChannels)
	{
		auxDevices[chan - 1].SetBaudRate(br);
	}
#endif
}

uint32_t Platform::GetBaudRate(size_t chan) const noexcept
{
	return
#if HAS_AUX_DEVICES
		(chan != 0 && chan < NumSerialChannels) ? auxDevices[chan - 1].GetBaudRate() :
#endif
		0;
}

void Platform::SetCommsProperties(size_t chan, uint32_t cp) noexcept
{
	if (chan < NumSerialChannels)
	{
		commsParams[chan] = cp;
	}
}

uint32_t Platform::GetCommsProperties(size_t chan) const noexcept
{
	return (chan < NumSerialChannels) ? commsParams[chan] : 0;
}

// Re-initialise a serial channel.
void Platform::ResetChannel(size_t chan) noexcept
{
	if (chan == 0)
	{
		SERIAL_MAIN_DEVICE.end();
#if SAME5x && !CORE_USES_TINYUSB
        SERIAL_MAIN_DEVICE.Start();
#else
        SERIAL_MAIN_DEVICE.Start(UsbVBusPin);
#endif
	}
#if HAS_AUX_DEVICES
	else if (chan < NumSerialChannels)
	{
		AuxDevice& device = auxDevices[chan - 1];
		AuxMode mode = device.GetMode();
		device.Disable();
		device.SetMode(mode);
	}
#endif
}

#if defined(DUET3_MB6HC)

// This is safe to call before Platform has been created
/*static*/ BoardType Platform::GetMB6HCBoardType() noexcept
{
	// Driver 0 direction has a pulldown resistor on v0.6 and v1.0 boards, but not on v1.01 or v1.02 boards
	// Driver 1 has a pulldown resistor on v0.1 and v1.0 boards, however we don't support v0.1 and we don't care about the difference between v0.6 and v1.0, so we don't need to read it
	// Driver 2 has a pulldown resistor on v1.02 only
	pinMode(DIRECTION_PINS[2], INPUT_PULLUP);
	pinMode(DIRECTION_PINS[0], INPUT_PULLUP);
	delayMicroseconds(20);									// give the pullup resistor time to work
	if (digitalRead(DIRECTION_PINS[2]))
	{
		return (digitalRead(DIRECTION_PINS[0])) ? BoardType::Duet3_6HC_v101 : BoardType::Duet3_6HC_v06_100;
	}
	else
	{
		return (digitalRead(DIRECTION_PINS[0])) ? BoardType::Duet3_6HC_v102 : BoardType::Duet3_6HC_v102b;
	}
}

#endif

#if defined(DUET3_MB6XD)

// This is safe to call before Platform has been created
/*static*/ BoardType Platform::GetMB6XDBoardType() noexcept
{
	// Driver 0 direction has a pulldown resistor on v1.0  boards only
	// Driver 5 direction has a pulldown resistor on 1.01 boards only
	pinMode(DIRECTION_PINS[0], INPUT_PULLUP);
	pinMode(DIRECTION_PINS[1], INPUT_PULLUP);
	pinMode(DIRECTION_PINS[5], INPUT_PULLUP);
	delayMicroseconds(20);									// give the pullup resistor time to work
	if (digitalRead(DIRECTION_PINS[5]))
	{
		return (digitalRead(DIRECTION_PINS[0])) ? BoardType::Duet3_6XD_v01 : BoardType::Duet3_6XD_v100;
	}
	return (digitalRead(DIRECTION_PINS[1])) ? BoardType::Duet3_6XD_v101 : BoardType::Duet3_6XD_v102;
}

#endif

// Set the board type/revision. This must be called quite early, because for some builds it relies on pins not having been programmed for their intended use yet.
// Also do any specific initialisation that varies with the board revision.
void Platform::SetBoardType() noexcept
{
#if defined(DUET3MINI_V04)
	// Test whether this is a WiFi or an Ethernet board by testing for a pulldown resistor on Dir1
	pinMode(DIRECTION_PINS[1], INPUT_PULLUP);
	delayMicroseconds(20);									// give the pullup resistor time to work
	board = (digitalRead(DIRECTION_PINS[1]))				// if SAME54P20A
				? BoardType::Duet3Mini_WiFi
					: BoardType::Duet3Mini_Ethernet;
#elif defined(DUET3_MB6HC)
	board = GetMB6HCBoardType();
	if (board >= BoardType::Duet3_6HC_v102)
	{
		powerMonitorVoltageRange = PowerMonitorVoltageRange_v102;
		DiagPin = DiagPin102;
		ActLedPin = ActLedPin102;
		DiagOnPolarity = DiagOnPolarity102;
	}
	else
	{
		powerMonitorVoltageRange = PowerMonitorVoltageRange_v101;
		DiagPin = DiagPinPre102;
		ActLedPin = ActLedPinPre102;
		DiagOnPolarity = DiagOnPolarityPre102;
	}
	driverPowerOnAdcReading = PowerVoltageToAdcReading(10.0);
	driverPowerOffAdcReading = PowerVoltageToAdcReading(9.5);
#elif defined(DUET3_MB6XD)
	board = GetMB6XDBoardType();
#elif defined(FMDC_V02) || defined(FMDC_V03)
	board = BoardType::FMDC;
#elif defined(DUET_NG)
	// Get ready to test whether the Ethernet module is present, so that we avoid additional delays
	pinMode(W5500ModuleSensePin, INPUT_PULLUP);				// set our UART receive pin to be an input pin and enable the pullup

	// Set up the VSSA sense pin. Older Duet WiFis don't have it connected, so we enable the pulldown resistor to keep it inactive.
	pinMode(VssaSensePin, INPUT_PULLUP);
	delayMicroseconds(10);
	const bool vssaHighVal = digitalRead(VssaSensePin);
	pinMode(VssaSensePin, INPUT_PULLDOWN);
	delayMicroseconds(10);
	const bool vssaLowVal = digitalRead(VssaSensePin);
	const bool vssaSenseWorking = vssaLowVal || !vssaHighVal;
	if (vssaSenseWorking)
	{
		pinMode(VssaSensePin, INPUT);
	}

# if defined(USE_SBC)
	board = (vssaSenseWorking) ? BoardType::Duet2SBC_102 : BoardType::Duet2SBC_10;
# else
	// Test whether the Ethernet module is present
	if (digitalRead(W5500ModuleSensePin))					// the Ethernet module has this pin grounded
	{
		board = (vssaSenseWorking) ? BoardType::DuetWiFi_102 : BoardType::DuetWiFi_10;
	}
	else
	{
		board = (vssaSenseWorking) ? BoardType::DuetEthernet_102 : BoardType::DuetEthernet_10;
	}
# endif
#elif defined(DUET_M)
	board = BoardType::DuetM_10;
#elif defined(PCCB_10)
	board = BoardType::PCCB_v10;
#else
# error Undefined board type
#endif
}

// Get a string describing the electronics
const char *_ecv_array Platform::GetElectronicsString() const noexcept
{
	switch (board)
	{
#if defined(DUET3MINI_V04)
	case BoardType::Duet3Mini_Unknown:		return "Duet 3 " BOARD_SHORT_NAME " unknown variant";
	case BoardType::Duet3Mini_WiFi:			return "Duet 3 " BOARD_SHORT_NAME " WiFi";
	case BoardType::Duet3Mini_Ethernet:		return "Duet 3 " BOARD_SHORT_NAME " Ethernet";
#elif defined(DUET3_MB6HC)
	case BoardType::Duet3_6HC_v06_100:		return "Duet 3 " BOARD_SHORT_NAME " v1.0 or earlier";
	case BoardType::Duet3_6HC_v101:			return "Duet 3 " BOARD_SHORT_NAME " v1.01";
	case BoardType::Duet3_6HC_v102:			return "Duet 3 " BOARD_SHORT_NAME " v1.02 or 1.02a";
	case BoardType::Duet3_6HC_v102b:		return "Duet 3 " BOARD_SHORT_NAME " v1.02b or later";
#elif defined(DUET3_MB6XD)
	case BoardType::Duet3_6XD_v01:			return "Duet 3 " BOARD_SHORT_NAME " v0.1";
	case BoardType::Duet3_6XD_v100:			return "Duet 3 " BOARD_SHORT_NAME " v1.0";
	case BoardType::Duet3_6XD_v101:			return "Duet 3 " BOARD_SHORT_NAME " v1.01";
	case BoardType::Duet3_6XD_v102:			return "Duet 3 " BOARD_SHORT_NAME " v1.02 or later";
#elif defined(FMDC_V02) || defined(FMDC_V03)
	case BoardType::FMDC:					return "Duet 3 " BOARD_SHORT_NAME;
#elif defined(DUET_NG)
	// This is the string that the Duet 2 ATE uses to identify the board. The version number must be at the end.
	case BoardType::DuetWiFi_10:			return "Duet WiFi 1.0 or 1.01";
	case BoardType::DuetWiFi_102:			return "Duet WiFi 1.02 or later";
	case BoardType::DuetEthernet_10:		return "Duet Ethernet 1.0 or 1.01";
	case BoardType::DuetEthernet_102:		return "Duet Ethernet 1.02 or later";
	case BoardType::Duet2SBC_10:			return "Duet 2 + SBC 1.0 or 1.01";
	case BoardType::Duet2SBC_102:			return "Duet 2 + SBC 1.02 or later";
#elif defined(DUET_M)
	case BoardType::DuetM_10:				return "Duet Maestro 1.0";
#elif defined(PCCB_10)
	case BoardType::PCCB_v10:				return "PC001373";
#else
# error Undefined board type
#endif
	default:								return "Unidentified";
	}
}

// Get the board string
const char *_ecv_array Platform::GetBoardString() const noexcept
{
	switch (board)
	{
#if defined(DUET3MINI_V04)
	case BoardType::Duet3Mini_Unknown:		return "duet5lcunknown";
	case BoardType::Duet3Mini_WiFi:			return "duet5lcwifi";
	case BoardType::Duet3Mini_Ethernet:		return "duet5lcethernet";
#elif defined(DUET3_MB6HC)
	case BoardType::Duet3_6HC_v06_100:		return "duet3mb6hc100";
	case BoardType::Duet3_6HC_v101:			return "duet3mb6hc101";
	case BoardType::Duet3_6HC_v102:			return "duet3mb6hc102";
	case BoardType::Duet3_6HC_v102b:		return "duet3mb6hc102b";
#elif defined(DUET3_MB6XD)
	case BoardType::Duet3_6XD_v01:			return "duet3mb6xd001";
	case BoardType::Duet3_6XD_v100:			return "duet3mb6xd100";
	case BoardType::Duet3_6XD_v101:			return "duet3mb6xd101";
	case BoardType::Duet3_6XD_v102:			return "duet3mb6xd102";
#elif defined(FMDC_V02) || defined(FMDC_V03)
	case BoardType::FMDC:					return "fmdc";
#elif defined(DUET_NG)
	case BoardType::DuetWiFi_10:			return "duetwifi10";
	case BoardType::DuetWiFi_102:			return "duetwifi102";
	case BoardType::DuetEthernet_10:		return "duetethernet10";
	case BoardType::DuetEthernet_102:		return "duetethernet102";
	case BoardType::Duet2SBC_10:			return "duet2sbc10";
	case BoardType::Duet2SBC_102:			return "duet2sbc102";
#elif defined(DUET_M)
	case BoardType::DuetM_10:				return "duetmaestro100";
#elif defined(PCCB_10)
	case BoardType::PCCB_v10:				return "pc001373";
#else
# error Undefined board type
#endif
	default:								return "unknown";
	}
}

#ifdef DUET_NG

// Return true if this is a Duet WiFi, false if it is a Duet Ethernet
bool Platform::IsDuetWiFi() const noexcept
{
	return board == BoardType::DuetWiFi_10 || board == BoardType::DuetWiFi_102;
}

const char *_ecv_array Platform::GetBoardName() const noexcept
{
	return (board == BoardType::Duet2SBC_10 || board == BoardType::Duet2SBC_102)
			? BOARD_NAME_SBC
			: (IsDuetWiFi()) ? BOARD_NAME_WIFI : BOARD_NAME_ETHERNET;
}

const char *_ecv_array Platform::GetBoardShortName() const noexcept
{
	return (board == BoardType::Duet2SBC_10 || board == BoardType::Duet2SBC_102)
			? BOARD_SHORT_NAME_SBC
			: (IsDuetWiFi()) ? BOARD_SHORT_NAME_WIFI : BOARD_SHORT_NAME_ETHERNET;
}

#endif

#ifdef DUET3MINI_V04

// Return true if this is a WiFi board, false if it has Ethernet
bool Platform::IsDuetWiFi() const noexcept
{
	return board == BoardType::Duet3Mini_WiFi || board == BoardType::Duet3Mini_Unknown;
}

#endif

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE

bool Platform::Delete(const char *_ecv_array folder, const char *_ecv_array filename) const noexcept
{
	String<MaxFilenameLength> location;
	return MassStorage::CombineName(location.GetRef(), folder, filename) && MassStorage::Delete(location.GetRef(), ErrorMessageMode::messageUnlessMissing);
}

bool Platform::DeleteSysFile(const char *_ecv_array filename) const noexcept
{
	String<MaxFilenameLength> location;
	return MakeSysFileName(location.GetRef(), filename) && MassStorage::Delete(location.GetRef(), ErrorMessageMode::messageUnlessMissing);
}

#endif

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE || HAS_EMBEDDED_FILES

// Open a file
FileStore* Platform::OpenFile(const char *_ecv_array folder, const char *_ecv_array fileName, OpenMode mode, uint32_t preAllocSize) const noexcept
{
	String<MaxFilenameLength> location;
	return (MassStorage::CombineName(location.GetRef(), folder, fileName))
			? MassStorage::OpenFile(location.c_str(), mode, preAllocSize)
				: nullptr;
}

bool Platform::FileExists(const char *_ecv_array folder, const char *_ecv_array filename) const noexcept
{
	String<MaxFilenameLength> location;
	return MassStorage::CombineName(location.GetRef(), folder, filename) && MassStorage::FileExists(location.c_str());
}

// Return a pointer to a string holding the directory where the system files are. Lock the sysdir lock before calling this.
const char *_ecv_array Platform::InternalGetSysDir() const noexcept
{
	return (sysDir != nullptr) ? sysDir : DEFAULT_SYS_DIR;
}

bool Platform::SysFileExists(const char *_ecv_array filename) const noexcept
{
	String<MaxFilenameLength> location;
	return MakeSysFileName(location.GetRef(), filename) && MassStorage::FileExists(location.c_str());
}

FileStore* Platform::OpenSysFile(const char *_ecv_array filename, OpenMode mode) const noexcept
{
	String<MaxFilenameLength> location;
	return (MakeSysFileName(location.GetRef(), filename))
			? MassStorage::OpenFile(location.c_str(), mode, 0)
				: nullptr;
}

bool Platform::MakeSysFileName(const StringRef& result, const char *_ecv_array filename) const noexcept
{
	return MassStorage::CombineName(result, GetSysDir().Ptr(), filename);
}

void Platform::AppendSysDir(const StringRef & path) const noexcept
{
	path.cat(GetSysDir().Ptr());
}

ReadLockedPointer<const char> Platform::GetSysDir() const noexcept
{
	return ReadLockedPointer<const char>(sysDirLock, InternalGetSysDir());
}

#endif

#if HAS_MASS_STORAGE || HAS_EMBEDDED_FILES

// Set the system files path
GCodeResult Platform::SetSysDir(const char *_ecv_array dir, const StringRef& reply) noexcept
{
	String<MaxFilenameLength> newSysDir;
	WriteLocker lock(sysDirLock);

	if (!MassStorage::CombineName(newSysDir.GetRef(), InternalGetSysDir(), dir) || (!newSysDir.EndsWith('/') && newSysDir.cat('/')))
	{
		reply.copy("Path name too long");
		return GCodeResult::error;
	}

	if (!MassStorage::DirectoryExists(newSysDir.GetRef()))
	{
		reply.copy("Path not found");
		return GCodeResult::error;
	}

	newSysDir.cat('/');								// the call to DirectoryExists removed the trailing '/'
	const size_t len = newSysDir.strlen() + 1;
	char* const nsd = new char[len];
	memcpy(nsd, newSysDir.c_str(), len);
	ReplaceObject(sysDir, nsd);
	reprap.DirectoriesUpdated();
	return GCodeResult::ok;
}

#endif

#if SUPPORT_LASER

// CNC and laser support

void Platform::SetLaserPwm(Pwm_t pwm) noexcept
{
	lastLaserPwm = (float)pwm/65535;
	laserPort.WriteAnalog(lastLaserPwm);			// we don't currently have an function that accepts an integer PWM fraction
}

// Return laser PWM in 0..1
float Platform::GetLaserPwm() const noexcept
{
	return lastLaserPwm;
}

bool Platform::AssignLaserPin(GCodeBuffer& gb, const StringRef& reply)
{
	if (!laserPort.AssignPort(gb, reply, PinUsedBy::laser, PinAccess::pwm))
	{
		return false;
	}

#ifdef DUET_NG
	// We can't use DueX fan pins because we need to set the PWM from within the tick ISR
	if (laserPort.GetPin() >= DueXnExpansionStart)
	{
		reply.copy("DueX fan or GPIO pins may not be used to control lasers");
		laserPort.Release();
		return false;
	}
#endif

	SetLaserPwm(0);
	return true;
}

void Platform::ReleaseLaserPin() noexcept
{
	SetLaserPwm(0.0);
	laserPort.Release();
}

void Platform::SetLaserPwmFrequency(PwmFrequency freq) noexcept
{
	laserPort.SetFrequency(freq);
}

#endif

void Platform::InitZProbeFilters() noexcept
{
	zProbeOnFilter.Init(0);
	zProbeOffFilter.Init(0);
}

#if HAS_CPU_TEMP_SENSOR

// CPU temperature
MinCurMax Platform::GetMcuTemperatures() const noexcept
{
	MinCurMax result;
	result.minimum = lowestMcuTemperature;
	result.current = GetCpuTemperature();
	result.maximum = highestMcuTemperature;
	return result;
}

#endif

#if HAS_VOLTAGE_MONITOR

// Power in voltage
MinCurMax Platform::GetPowerVoltages() const noexcept
{
	MinCurMax result;
	result.minimum = AdcReadingToPowerVoltage(lowestVin);
	result.current = AdcReadingToPowerVoltage(currentVin);
	result.maximum = AdcReadingToPowerVoltage(highestVin);
	return result;
}

float Platform::GetCurrentPowerVoltage() const noexcept
{
	return AdcReadingToPowerVoltage(currentVin);
}

#endif

#if HAS_12V_MONITOR

MinCurMax Platform::GetV12Voltages() const noexcept
{
	MinCurMax result;
	result.minimum = AdcReadingToV12Voltage(lowestV12);
	result.current = AdcReadingToV12Voltage(currentV12);
	result.maximum = AdcReadingToV12Voltage(highestV12);
	return result;
}

float Platform::GetCurrentV12Voltage() const noexcept
{
	return AdcReadingToV12Voltage(currentV12);
}

#endif

// Real-time clock

bool Platform::SetDateTime(time_t time) noexcept
{
	struct tm brokenDateTime;
	const bool ok = (gmtime_r(&time, &brokenDateTime) != nullptr);
	if (ok)
	{
		realTime = time;			// set the date and time

		// Write a log message, giving the time since power up in same format as the logger does
		const uint32_t timeSincePowerUp = (uint32_t)(millis64()/1000u);
		MessageF(LogWarn, "Date and time set at power up + %02" PRIu32 ":%02" PRIu32 ":%02" PRIu32 "\n", timeSincePowerUp/3600u, (timeSincePowerUp % 3600u)/60u, timeSincePowerUp % 60u);
		timeLastUpdatedMillis = millis();
	}
	return ok;
}

// Configure an I/O port
GCodeResult Platform::ConfigurePort(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	// Exactly one of EFHJPSR is allowed (also D on MB6HC)
	unsigned int charsPresent = 0;
	for (char c : (const char[]){'D', 'E', 'R', 'J', 'F', 'H', 'P', 'S'})
	{
		charsPresent <<= 1;
		if (gb.Seen(c))
		{
			charsPresent |= 1;
		}
	}

	switch (charsPresent)
	{
	case 1:		// S
		{
			const uint32_t gpioNumber = gb.GetLimitedUIValue('S', MaxGpOutPorts);
			return gpoutPorts[gpioNumber].Configure(gpioNumber, true, gb, reply);
		}

	case 2:		// P
		{
			const uint32_t gpioNumber = gb.GetLimitedUIValue('P', MaxGpOutPorts);
			return gpoutPorts[gpioNumber].Configure(gpioNumber, false, gb, reply);
		}

	case 4:		// H
		return reprap.GetHeat().ConfigureHeater(gb, reply);

	case 8:		// F
		return reprap.GetFansManager().ConfigureFanPort(gb, reply);

	case 16:	// J
		{
			const uint32_t gpinNumber = gb.GetLimitedUIValue('J', MaxGpInPorts);
			return gpinPorts[gpinNumber].Configure(gpinNumber, gb, reply);
		}
	case 32:	// R
		{
			const uint32_t spindleNumber = gb.GetLimitedUIValue('R', MaxSpindles);
			return spindles[spindleNumber].Configure(spindleNumber, gb, reply);
		}

#if SUPPORT_LED_STRIPS
	case 64:	// E
		return ledStripManager.CreateStrip(gb, reply);
#endif

#if defined(DUET3_MB6HC) && HAS_MASS_STORAGE
	case 128:	// D
# if HAS_SBC_INTERFACE
		if (reprap.UsingSbcInterface())
		{
			reply.copy("SD card attached to Duet is not supported in SBC mode");
			return GCodeResult::error;
		}
# endif
		return MassStorage::ConfigureSdCard(gb, reply);
#endif

	default:
#if defined(DUET3_MB6HC) && HAS_MASS_STORAGE
# if SUPPORT_LED_STRIPS
		reply.copy("exactly one of DEFHJPSR must be given");
# else
		reply.copy("exactly one of DFHJPSR must be given");
# endif
#elif SUPPORT_LED_STRIPS
		reply.copy("exactly one of EFHJPSR must be given");
#else
		reply.copy("exactly one of FHJPSR must be given");
#endif
		return GCodeResult::error;
	}
}

#if SUPPORT_CAN_EXPANSION

void Platform::HandleRemoteGpInChange(CanAddress src, uint8_t handleMajor, uint8_t handleMinor, bool state) noexcept
{
	if (handleMajor < MaxGpInPorts)
	{
		gpinPorts[handleMajor].SetState(src, state);
	}
}

void Platform::OnProcessingCanMessage() noexcept
{
	whenLastCanMessageProcessed = millis();
	digitalWrite(ActLedPin, ActOnPolarity);				// turn the ACT LED on
}

#endif

// Configure the ancillary PWM
GCodeResult Platform::GetSetAncillaryPwm(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	bool seen = false;
	if (gb.Seen('P'))
	{
		seen = true;
		if (!extrusionAncilliaryPwmPort.AssignPort(gb, reply, PinUsedBy::gpout, PinAccess::pwm))
		{
			return GCodeResult::error;
		}
		const PwmFrequency freq = (gb.Seen('Q') || gb.Seen('F')) ? gb.GetPwmFrequency() : DefaultPinWritePwmFreq;
		extrusionAncilliaryPwmPort.SetFrequency(freq);
	}
	if (gb.Seen('S'))
	{
		extrusionAncilliaryPwmValue = min<float>(gb.GetFValue(), 1.0);		// negative values are OK, they mean don't set the output
		seen = true;
	}

	if (!seen)
	{
		reply.copy("Extrusion ancillary PWM");
		extrusionAncilliaryPwmPort.AppendFullDetails(reply);
	}
	return GCodeResult::ok;
}

#if MCU_HAS_UNIQUE_ID

// Get a pseudo-random number (not a true random number)
uint32_t Platform::Random() noexcept
{
	return StepTimer::GetTimerTicks() ^ uniqueId.GetHash();
}

#endif

void Platform::SetDiagLed(bool on) const noexcept
{
	digitalWrite(DiagPin, XNor(DiagOnPolarity, on));
}

#if SUPPORT_MULTICAST_DISCOVERY

void Platform::InvertDiagLed() const noexcept
{
	digitalWrite(DiagPin, !digitalRead(DiagPin));
}

#endif

#if HAS_CPU_TEMP_SENSOR && SAME5x

void Platform::TemperatureCalibrationInit() noexcept
{
	// Temperature sense stuff
	constexpr uint32_t NVM_TEMP_CAL_TLI_POS = 0;
	constexpr uint32_t NVM_TEMP_CAL_TLI_SIZE = 8;
	constexpr uint32_t NVM_TEMP_CAL_TLD_POS = 8;
	constexpr uint32_t NVM_TEMP_CAL_TLD_SIZE = 4;
	constexpr uint32_t NVM_TEMP_CAL_THI_POS = 12;
	constexpr uint32_t NVM_TEMP_CAL_THI_SIZE = 8;
	constexpr uint32_t NVM_TEMP_CAL_THD_POS = 20;
	constexpr uint32_t NVM_TEMP_CAL_THD_SIZE = 4;
	constexpr uint32_t NVM_TEMP_CAL_VPL_POS = 40;
	constexpr uint32_t NVM_TEMP_CAL_VPL_SIZE = 12;
	constexpr uint32_t NVM_TEMP_CAL_VPH_POS = 52;
	constexpr uint32_t NVM_TEMP_CAL_VPH_SIZE = 12;
	constexpr uint32_t NVM_TEMP_CAL_VCL_POS = 64;
	constexpr uint32_t NVM_TEMP_CAL_VCL_SIZE = 12;
	constexpr uint32_t NVM_TEMP_CAL_VCH_POS = 76;
	constexpr uint32_t NVM_TEMP_CAL_VCH_SIZE = 12;

	const uint16_t temp_cal_vpl = (*((uint32_t *)(NVMCTRL_TEMP_LOG) + (NVM_TEMP_CAL_VPL_POS / 32)) >> (NVM_TEMP_CAL_VPL_POS % 32))
	               & ((1u << NVM_TEMP_CAL_VPL_SIZE) - 1);
	const uint16_t temp_cal_vph = (*((uint32_t *)(NVMCTRL_TEMP_LOG) + (NVM_TEMP_CAL_VPH_POS / 32)) >> (NVM_TEMP_CAL_VPH_POS % 32))
	               & ((1u << NVM_TEMP_CAL_VPH_SIZE) - 1);
	const uint16_t temp_cal_vcl = (*((uint32_t *)(NVMCTRL_TEMP_LOG) + (NVM_TEMP_CAL_VCL_POS / 32)) >> (NVM_TEMP_CAL_VCL_POS % 32))
	               & ((1u << NVM_TEMP_CAL_VCL_SIZE) - 1);
	const uint16_t temp_cal_vch = (*((uint32_t *)(NVMCTRL_TEMP_LOG) + (NVM_TEMP_CAL_VCH_POS / 32)) >> (NVM_TEMP_CAL_VCH_POS % 32))
	               & ((1u << NVM_TEMP_CAL_VCH_SIZE) - 1);

	const uint8_t temp_cal_tli = (*((uint32_t *)(NVMCTRL_TEMP_LOG) + (NVM_TEMP_CAL_TLI_POS / 32)) >> (NVM_TEMP_CAL_TLI_POS % 32))
	               & ((1u << NVM_TEMP_CAL_TLI_SIZE) - 1);
	const uint8_t temp_cal_tld = (*((uint32_t *)(NVMCTRL_TEMP_LOG) + (NVM_TEMP_CAL_TLD_POS / 32)) >> (NVM_TEMP_CAL_TLD_POS % 32))
	               & ((1u << NVM_TEMP_CAL_TLD_SIZE) - 1);
	const uint16_t temp_cal_tl = ((uint16_t)temp_cal_tli) << 4 | ((uint16_t)temp_cal_tld);

	const uint8_t temp_cal_thi = (*((uint32_t *)(NVMCTRL_TEMP_LOG) + (NVM_TEMP_CAL_THI_POS / 32)) >> (NVM_TEMP_CAL_THI_POS % 32))
	               & ((1u << NVM_TEMP_CAL_THI_SIZE) - 1);
	const uint8_t temp_cal_thd = (*((uint32_t *)(NVMCTRL_TEMP_LOG) + (NVM_TEMP_CAL_THD_POS / 32)) >> (NVM_TEMP_CAL_THD_POS % 32))
	               & ((1u << NVM_TEMP_CAL_THD_SIZE) - 1);
	const uint16_t temp_cal_th = ((uint16_t)temp_cal_thi) << 4 | ((uint16_t)temp_cal_thd);

	tempCalF1 = (int32_t)temp_cal_tl * (int32_t)temp_cal_vph - (int32_t)temp_cal_th * (int32_t)temp_cal_vpl;
	tempCalF2 = (int32_t)temp_cal_tl * (int32_t)temp_cal_vch - (int32_t)temp_cal_th * (int32_t)temp_cal_vcl;
	tempCalF3 = (int32_t)temp_cal_vcl - (int32_t)temp_cal_vch;
	tempCalF4 = (int32_t)temp_cal_vpl - (int32_t)temp_cal_vph;
}

#endif

#if SUPPORT_REMOTE_COMMANDS

GCodeResult Platform::EutHandleM950Gpio(const CanMessageGeneric& msg, const StringRef& reply) noexcept
{
	// Get and validate the port number
	CanMessageGenericParser parser(msg, M950GpioParams);
	uint16_t gpioNumber;
	if (!parser.GetUintParam('P', gpioNumber))
	{
		reply.copy("Missing port number parameter in M950Gpio message");
		return GCodeResult::error;
	}
	if (gpioNumber >= MaxGpOutPorts)
	{
		reply.printf("GPIO port number %u is too high for board %u", gpioNumber, CanInterface::GetCanAddress());
		return GCodeResult::error;
	}

	return gpoutPorts[gpioNumber].AssignFromRemote(gpioNumber, parser, reply);
}

GCodeResult Platform::EutHandleGpioWrite(const CanMessageWriteGpio& msg, const StringRef& reply) noexcept
{
	if (msg.portNumber >= MaxGpOutPorts)
	{
		reply.printf("GPIO port# %u is too high for this board", msg.portNumber);
		return GCodeResult::error;
	}

	gpoutPorts[msg.portNumber].WriteAnalog(msg.pwm);
	return GCodeResult::ok;
}

// Process a 1ms tick interrupt
// This function must be kept fast so as not to disturb the stepper timing, so don't do any floating point maths in here.
// This is what we need to do:
// 1.  Kick off a new ADC conversion.
// 2.  Fetch and process the result of the last ADC conversion.
// 3a. If the last ADC conversion was for the Z probe, toggle the modulation output if using a modulated IR sensor.
// 3b. If the last ADC reading was a thermistor reading, check for an over-temperature situation and turn off the heater if necessary.
//     We do this here because the usual polling loop sometimes gets stuck trying to send data to the USB port.

#endif

void Platform::Tick() noexcept
{
#if !SAME5x
	LegacyAnalogIn::AnalogInFinaliseConversion();
#endif

#if HAS_VOLTAGE_MONITOR || HAS_12V_MONITOR
	if (tickState != 0)
	{
# if HAS_VOLTAGE_MONITOR
		// Read the power input voltage
		currentVin = AnalogInReadChannel(vInMonitorAdcChannel);
		if (currentVin > highestVin)
		{
			highestVin = currentVin;
			reprap.BoardsUpdated();
		}
		if (currentVin < lowestVin)
		{
			lowestVin = currentVin;
			reprap.BoardsUpdated();
		}
# endif

# if HAS_12V_MONITOR
		currentV12 = AnalogInReadChannel(v12MonitorAdcChannel);
		if (currentV12 > highestV12)
		{
			highestV12 = currentV12;
			reprap.BoardsUpdated();
		}
		if (currentV12 < lowestV12)
		{
			lowestV12 = currentV12;
			reprap.BoardsUpdated();
		}
# endif

# if HAS_SMART_DRIVERS && (ENFORCE_MAX_VIN || ENFORCE_MIN_V12)
		if (driversPowered &&
#  if ENFORCE_MAX_VIN && ENFORCE_MIN_V12
			 (currentVin > driverOverVoltageAdcReading || currentV12 < driverV12OffAdcReading)
#  elif ENFORCE_MAX_VIN
			 currentVin > driverOverVoltageAdcReading
#  elif ENFORCE_MIN_V12
			 currentV12 < driverV12OffAdcReading
#  endif
		   )
		{
			reprap.GetMove().TurnSmartDriversOff();
			// We deliberately do not clear driversPowered here or increase the over voltage event count - we let the spin loop handle that
		}
# endif
	}
#endif

	// Turn off the buzzer when needed
	uint32_t locTicks = beepTicksToGo;				// capture volatile variable to reduce code size
	if (locTicks != 0)
	{
		--locTicks;
		if (locTicks == 0)
		{
			StopBeep();
		}
		else
		{
			beepTicksToGo = locTicks;
		}
	}

#if SAME70
	// The SAME70 ADC is noisy, so read a thermistor on every tick so that we can average a greater number of readings
	// Because we are in the tick ISR and no other ISR reads the averaging filter, we can cast away 'volatile' here.
	if (tickState != 0)
	{
		ThermistorAveragingFilter& currentFilter = const_cast<ThermistorAveragingFilter&>(adcFilters[currentFilterNumber]);		// cast away 'volatile'
		currentFilter.ProcessReading(AnalogInReadChannel(filteredAdcChannels[currentFilterNumber]));

		++currentFilterNumber;
		if (currentFilterNumber == NumAdcFilters)
		{
			currentFilterNumber = 0;
		}
	}
#endif

	const ZProbe& currentZProbe = endstops.GetDefaultZProbeFromISR();
	switch (tickState)
	{
	case 1:
	case 3:
		{
#if !SAME70
			// We read a filtered ADC channel on alternate ticks
			// Because we are in the tick ISR and no other ISR reads the averaging filter, we can cast away 'volatile' here.
			ThermistorAveragingFilter& currentFilter = const_cast<ThermistorAveragingFilter&>(adcFilters[currentFilterNumber]);		// cast away 'volatile'
			currentFilter.ProcessReading(AnalogInReadChannel(filteredAdcChannels[currentFilterNumber]));

			++currentFilterNumber;
			if (currentFilterNumber == NumAdcFilters)
			{
				currentFilterNumber = 0;
			}
#endif
			// If we are not using a simple modulated IR sensor, process the Z probe reading on every tick for a faster response.
			// If we are using a simple modulated IR sensor then we need to allow the reading to settle after turning the IR emitter on or off,
			// so on alternate ticks we read it and switch the emitter
			if (currentZProbe.GetProbeType() != ZProbeType::dumbModulated)
			{
				const_cast<ZProbeAveragingFilter&>((tickState == 1) ? zProbeOnFilter : zProbeOffFilter).ProcessReading(currentZProbe.GetRawReading());
			}
			++tickState;
		}
		break;

	case 2:
		const_cast<ZProbeAveragingFilter&>(zProbeOnFilter).ProcessReading(currentZProbe.GetRawReading());
		currentZProbe.SetIREmitter(false);
		++tickState;
		break;

	case 4:			// last conversion started was the Z probe, with IR LED off if modulation is enabled
		const_cast<ZProbeAveragingFilter&>(zProbeOffFilter).ProcessReading(currentZProbe.GetRawReading());
		currentZProbe.SetIREmitter(true);
		tickState = 1;
		break;

	case 0:			// this is the state after initialisation, no conversion has been started
	default:
		currentZProbe.SetIREmitter(true);
		tickState = 1;
		break;
	}

#if SAME70
	// On Duet 3, AFEC1 is used only for thermistors and associated Vref/Vssa monitoring. AFEC0 is used for everything else.
	// To reduce noise, we use x16 hardware averaging on AFEC0 and x256 on AFEC1. This is hard coded in file AnalogIn.cpp in project CoreNG.
	// There is enough time to convert all AFEC0 channels in one tick, but only one AFEC1 channel because of the higher averaging.
	LegacyAnalogIn::AnalogInStartConversion(0x0FFF | (1u << (uint8_t) filteredAdcChannels[currentFilterNumber]));
#elif !SAME5x
	LegacyAnalogIn::AnalogInStartConversion();
#endif
}

// Pragma pop_options is not supported on this platform
//#pragma GCC pop_options

// End
