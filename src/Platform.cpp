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

#include "Heating/Heat.h"
#include "Movement/DDA.h"
#include "Movement/Move.h"
#include "Movement/StepTimer.h"
#include "Tools/Tool.h"
#include "Endstops/ZProbe.h"
#include "Networking/Network.h"
#include "PrintMonitor.h"
#include "FilamentMonitors/FilamentMonitor.h"
#include "RepRap.h"
#include "Scanner.h"
#include "Version.h"
#include "Logger.h"
#include "Tasks.h"
#include <Cache.h>
#include <Hardware/SharedSpi/SharedSpiDevice.h>
#include <Math/Isqrt.h>
#include <Hardware/I2C.h>
#include <Hardware/NonVolatileMemory.h>
#include <Storage/CRC32.h>

#if SAM4E || SAM4S || SAME70
# include <Flash.h>		// for flash_read_unique_id()
#endif

#if SAM4E || SAM4S || SAME70
# include <AnalogIn.h>
# include <DmacManager.h>
using LegacyAnalogIn::AdcBits;
# if SAME70
static_assert(NumDmaChannelsUsed <= NumDmaChannelsSupported, "Need more DMA channels in CoreNG");
# endif
#elif SAME5x
# include <AnalogIn.h>
# include <DmacManager.h>
using AnalogIn::AdcBits;			// for compatibility with CoreNG, which doesn't have the AnalogIn namespace
#elif defined(__LPC17xx__)
# include "LPC/BoardConfig.h"
#else
# include "sam/drivers/tc/tc.h"
# include "sam/drivers/hsmci/hsmci.h"
#endif

#include <Libraries/sd_mmc/sd_mmc.h>

#if SUPPORT_TMC2660
# include "Movement/StepperDrivers/TMC2660.h"
#endif
#if SUPPORT_TMC22xx
# include "Movement/StepperDrivers/TMC22xx.h"
#endif
#if SUPPORT_TMC51xx
# include "Movement/StepperDrivers/TMC51xx.h"
#endif

#if HAS_WIFI_NETWORKING
# include <Comms/FirmwareUpdater.h>
#endif

#if SUPPORT_12864_LCD
# include "Display/Display.h"
#endif

#if SUPPORT_IOBITS
# include "PortControl.h"
#endif

#if HAS_LINUX_INTERFACE
# include "Linux/LinuxInterface.h"
# include "Linux/DataTransfer.h"
#endif

#if HAS_NETWORKING
# include "Networking/HttpResponder.h"
# include "Networking/FtpResponder.h"
# include "Networking/TelnetResponder.h"
#endif

#if SUPPORT_CAN_EXPANSION
# include "CAN/CanMessageGenericConstructor.h"
# include "CAN/CanInterface.h"
#endif

#if SUPPORT_REMOTE_COMMANDS
# include <CanMessageGenericParser.h>
#endif

#include <climits>
#include <utility>					// for std::swap

#if !defined(HAS_LWIP_NETWORKING) || !defined(HAS_WIFI_NETWORKING) || !defined(HAS_CPU_TEMP_SENSOR) || !defined(HAS_HIGH_SPEED_SD) \
 || !defined(HAS_SMART_DRIVERS) || !defined(HAS_STALL_DETECT) || !defined(HAS_VOLTAGE_MONITOR) || !defined(HAS_12V_MONITOR) || !defined(HAS_VREF_MONITOR) \
 || !defined(SUPPORT_NONLINEAR_EXTRUSION) || !defined(SUPPORT_ASYNC_MOVES) || !defined(HAS_MASS_STORAGE)
# error Missing feature definition
#endif

#if HAS_VOLTAGE_MONITOR

inline constexpr float AdcReadingToPowerVoltage(uint16_t adcVal)
{
	return adcVal * (PowerMonitorVoltageRange/(1u << AdcBits));
}

inline constexpr uint16_t PowerVoltageToAdcReading(float voltage)
{
	return (uint16_t)(voltage * ((1u << AdcBits)/PowerMonitorVoltageRange));
}

constexpr uint16_t driverPowerOnAdcReading = PowerVoltageToAdcReading(10.0);			// minimum voltage at which we initialise the drivers
constexpr uint16_t driverPowerOffAdcReading = PowerVoltageToAdcReading(9.5);			// voltages below this flag the drivers as unusable

# if ENFORCE_MAX_VIN
constexpr uint16_t driverOverVoltageAdcReading = PowerVoltageToAdcReading(29.0);		// voltages above this cause driver shutdown
constexpr uint16_t driverNormalVoltageAdcReading = PowerVoltageToAdcReading(27.5);		// voltages at or below this are normal
# endif

#endif

#if HAS_12V_MONITOR

inline constexpr float AdcReadingToV12Voltage(uint16_t adcVal)
{
	return adcVal * (V12MonitorVoltageRange/(1u << AdcBits));
}

inline constexpr uint16_t V12VoltageToAdcReading(float voltage)
{
	return (uint16_t)(voltage * ((1u << AdcBits)/V12MonitorVoltageRange));
}

constexpr uint16_t driverV12OnAdcReading = V12VoltageToAdcReading(10.0);				// minimum voltage at which we initialise the drivers
constexpr uint16_t driverV12OffAdcReading = V12VoltageToAdcReading(9.5);				// voltages below this flag the drivers as unusable

#endif

const float MinStepPulseTiming = 0.2;				// we assume that we always generate step high and low times at least this wide without special action

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
#define OBJECT_MODEL_FUNC(...) OBJECT_MODEL_FUNC_BODY(Platform, __VA_ARGS__)

constexpr ObjectModelArrayDescriptor Platform::axisDriversArrayDescriptor =
{
	nullptr,					// no lock needed
	[] (const ObjectModel *self, const ObjectExplorationContext& context) noexcept -> size_t { return ((const Platform*)self)->axisDrivers[context.GetLastIndex()].numDrivers; },
	[] (const ObjectModel *self, ObjectExplorationContext& context) noexcept -> ExpressionValue
			{ return ExpressionValue(((const Platform*)self)->axisDrivers[context.GetIndex(1)].driverNumbers[context.GetLastIndex()]); }
};

constexpr ObjectModelArrayDescriptor Platform::workplaceOffsetsArrayDescriptor =
{
	nullptr,					// no lock needed
	[] (const ObjectModel *self, const ObjectExplorationContext& context) noexcept -> size_t { return NumCoordinateSystems; },
	[] (const ObjectModel *self, ObjectExplorationContext& context) noexcept -> ExpressionValue
			{ return ExpressionValue(reprap.GetGCodes().GetWorkplaceOffset(context.GetIndex(1), context.GetIndex(0)), 3); }
};

static inline const char* GetFilamentName(size_t extruder) noexcept
{
	const Filament *fil = Filament::GetFilamentByExtruder(extruder);
	return (fil == nullptr) ? "" : fil->GetName();
}

constexpr ObjectModelTableEntry Platform::objectModelTable[] =
{
	// 0. boards[0] members
#if SUPPORT_CAN_EXPANSION
	{ "canAddress",			OBJECT_MODEL_FUNC_NOSELF((int32_t)CanInterface::GetCanAddress()),									ObjectModelEntryFlags::none },
#endif
#if SUPPORT_12864_LCD
	{ "directDisplay",		OBJECT_MODEL_FUNC_IF_NOSELF(reprap.GetDisplay().IsPresent(), &reprap.GetDisplay()),					ObjectModelEntryFlags::none },
#endif
	{ "firmwareDate",		OBJECT_MODEL_FUNC_NOSELF(DATE),																		ObjectModelEntryFlags::none },
	{ "firmwareFileName",	OBJECT_MODEL_FUNC_NOSELF(IAP_FIRMWARE_FILE),														ObjectModelEntryFlags::none },
	{ "firmwareName",		OBJECT_MODEL_FUNC_NOSELF(FIRMWARE_NAME),															ObjectModelEntryFlags::none },
	{ "firmwareVersion",	OBJECT_MODEL_FUNC_NOSELF(VERSION),																	ObjectModelEntryFlags::none },
#if HAS_LINUX_INTERFACE
	{ "iapFileNameSBC",		OBJECT_MODEL_FUNC_NOSELF(IAP_UPDATE_FILE_SBC),														ObjectModelEntryFlags::none },
#endif
#if HAS_MASS_STORAGE
	{ "iapFileNameSD",		OBJECT_MODEL_FUNC_NOSELF(IAP_UPDATE_FILE),															ObjectModelEntryFlags::none },
#endif
	{ "maxHeaters",			OBJECT_MODEL_FUNC_NOSELF((int32_t)MaxHeaters),														ObjectModelEntryFlags::verbose },
	{ "maxMotors",			OBJECT_MODEL_FUNC_NOSELF((int32_t)NumDirectDrivers),												ObjectModelEntryFlags::verbose },
	{ "mcuTemp",			OBJECT_MODEL_FUNC(self, 1),																			ObjectModelEntryFlags::live },
# ifdef DUET_NG
	{ "name",				OBJECT_MODEL_FUNC(self->GetBoardName()),															ObjectModelEntryFlags::none },
	{ "shortName",			OBJECT_MODEL_FUNC(self->GetBoardShortName()),														ObjectModelEntryFlags::none },
# else
	{ "name",				OBJECT_MODEL_FUNC_NOSELF(BOARD_NAME),																ObjectModelEntryFlags::none },
	{ "shortName",			OBJECT_MODEL_FUNC_NOSELF(BOARD_SHORT_NAME),															ObjectModelEntryFlags::none },
# endif
	{ "supportsDirectDisplay", OBJECT_MODEL_FUNC_NOSELF(SUPPORT_12864_LCD ? true : false),										ObjectModelEntryFlags::verbose },
#if MCU_HAS_UNIQUE_ID
	{ "uniqueId",			OBJECT_MODEL_FUNC(self->GetUniqueIdString()),														ObjectModelEntryFlags::none },
#endif
#if HAS_12V_MONITOR
	{ "v12",				OBJECT_MODEL_FUNC(self, 6),																			ObjectModelEntryFlags::live },
#endif
	{ "vIn",				OBJECT_MODEL_FUNC(self, 2),																			ObjectModelEntryFlags::live },

#if HAS_CPU_TEMP_SENSOR
	// 1. mcuTemp members
	{ "current",			OBJECT_MODEL_FUNC(self->GetMcuTemperatures().current, 1),											ObjectModelEntryFlags::live },
	{ "max",				OBJECT_MODEL_FUNC(self->GetMcuTemperatures().max, 1),												ObjectModelEntryFlags::none },
	{ "min",				OBJECT_MODEL_FUNC(self->GetMcuTemperatures().min, 1),												ObjectModelEntryFlags::none },
#endif

	// 2. vIn members
#if HAS_VOLTAGE_MONITOR
	{ "current",			OBJECT_MODEL_FUNC(self->GetCurrentPowerVoltage(), 1),												ObjectModelEntryFlags::live },
	{ "max",				OBJECT_MODEL_FUNC(self->GetPowerVoltages().max, 1),													ObjectModelEntryFlags::none },
	{ "min",				OBJECT_MODEL_FUNC(self->GetPowerVoltages().min, 1),													ObjectModelEntryFlags::none },
#endif

	// 3. move.axes[] members
	{ "acceleration",		OBJECT_MODEL_FUNC(self->Acceleration(context.GetLastIndex()), 1),									ObjectModelEntryFlags::none },
	{ "babystep",			OBJECT_MODEL_FUNC_NOSELF(reprap.GetGCodes().GetTotalBabyStepOffset(context.GetLastIndex()), 3),		ObjectModelEntryFlags::none },
	{ "current",			OBJECT_MODEL_FUNC((int32_t)lrintf(self->GetMotorCurrent(context.GetLastIndex(), 906))),				ObjectModelEntryFlags::none },
	{ "drivers",			OBJECT_MODEL_FUNC_NOSELF(&axisDriversArrayDescriptor),												ObjectModelEntryFlags::none },
	{ "homed",				OBJECT_MODEL_FUNC_NOSELF(reprap.GetGCodes().IsAxisHomed(context.GetLastIndex())),					ObjectModelEntryFlags::none },
	{ "jerk",				OBJECT_MODEL_FUNC(MinutesToSeconds * self->GetInstantDv(context.GetLastIndex()), 1),				ObjectModelEntryFlags::none },
	{ "letter",				OBJECT_MODEL_FUNC_NOSELF(reprap.GetGCodes().GetAxisLetters()[context.GetLastIndex()]),				ObjectModelEntryFlags::none },
	{ "machinePosition",	OBJECT_MODEL_FUNC_NOSELF(reprap.GetMove().LiveCoordinate(context.GetLastIndex(), reprap.GetCurrentTool()), 3),	ObjectModelEntryFlags::live },
	{ "max",				OBJECT_MODEL_FUNC(self->AxisMaximum(context.GetLastIndex()), 2),									ObjectModelEntryFlags::none },
	{ "maxProbed",			OBJECT_MODEL_FUNC(self->axisMaximaProbed.IsBitSet(context.GetLastIndex())),							ObjectModelEntryFlags::none },
	{ "microstepping",		OBJECT_MODEL_FUNC(self, 7),																			ObjectModelEntryFlags::none },
	{ "min",				OBJECT_MODEL_FUNC(self->AxisMinimum(context.GetLastIndex()), 2),									ObjectModelEntryFlags::none },
	{ "minProbed",			OBJECT_MODEL_FUNC(self->axisMinimaProbed.IsBitSet(context.GetLastIndex())),							ObjectModelEntryFlags::none },
	{ "speed",				OBJECT_MODEL_FUNC(MinutesToSeconds * self->MaxFeedrate(context.GetLastIndex()), 1),					ObjectModelEntryFlags::none },
	{ "stepsPerMm",			OBJECT_MODEL_FUNC(self->driveStepsPerUnit[context.GetLastIndex()], 2),								ObjectModelEntryFlags::none },
	{ "userPosition",		OBJECT_MODEL_FUNC_NOSELF(reprap.GetGCodes().GetUserCoordinate(context.GetLastIndex()), 3),			ObjectModelEntryFlags::live },
	{ "visible",			OBJECT_MODEL_FUNC_NOSELF(context.GetLastIndex() < (int32_t)reprap.GetGCodes().GetVisibleAxes()),	ObjectModelEntryFlags::none },
	{ "workplaceOffsets",	OBJECT_MODEL_FUNC_NOSELF(&workplaceOffsetsArrayDescriptor),											ObjectModelEntryFlags::none },

	// 4. move.extruders[] members
	{ "acceleration",		OBJECT_MODEL_FUNC(self->Acceleration(ExtruderToLogicalDrive(context.GetLastIndex())), 1),			ObjectModelEntryFlags::none },
	{ "current",			OBJECT_MODEL_FUNC((int32_t)lrintf(self->GetMotorCurrent(ExtruderToLogicalDrive(context.GetLastIndex()), 906))),	ObjectModelEntryFlags::none },
	{ "driver",				OBJECT_MODEL_FUNC(self->extruderDrivers[context.GetLastIndex()]),									ObjectModelEntryFlags::none },
	{ "factor",				OBJECT_MODEL_FUNC_NOSELF(reprap.GetGCodes().GetExtrusionFactor(context.GetLastIndex()), 2),			ObjectModelEntryFlags::none },
	{ "filament",			OBJECT_MODEL_FUNC_NOSELF(GetFilamentName(context.GetLastIndex())),									ObjectModelEntryFlags::none },
	{ "jerk",				OBJECT_MODEL_FUNC(MinutesToSeconds * self->GetInstantDv(ExtruderToLogicalDrive(context.GetLastIndex())), 1),	ObjectModelEntryFlags::none },
	{ "microstepping",		OBJECT_MODEL_FUNC(self, 8),																			ObjectModelEntryFlags::none },
	{ "nonlinear",			OBJECT_MODEL_FUNC(self, 5),																			ObjectModelEntryFlags::none },
	{ "position",			OBJECT_MODEL_FUNC_NOSELF(ExpressionValue(reprap.GetMove().LiveCoordinate(ExtruderToLogicalDrive(context.GetLastIndex()), reprap.GetCurrentTool()), 1)),	ObjectModelEntryFlags::live },
	{ "pressureAdvance",	OBJECT_MODEL_FUNC(self->GetPressureAdvance(context.GetLastIndex()), 2),								ObjectModelEntryFlags::none },
	{ "rawPosition",		OBJECT_MODEL_FUNC_NOSELF(ExpressionValue(reprap.GetGCodes().GetRawExtruderTotalByDrive(context.GetLastIndex()), 1)), ObjectModelEntryFlags::live },
	{ "speed",				OBJECT_MODEL_FUNC(MinutesToSeconds * self->MaxFeedrate(ExtruderToLogicalDrive(context.GetLastIndex())), 1),	ObjectModelEntryFlags::none },
	{ "stepsPerMm",			OBJECT_MODEL_FUNC(self->driveStepsPerUnit[ExtruderToLogicalDrive(context.GetLastIndex())], 2),		ObjectModelEntryFlags::none },

	// 5. move.extruders[].nonlinear members
	{ "a",					OBJECT_MODEL_FUNC(self->nonlinearExtrusionA[context.GetLastIndex()], 3),							ObjectModelEntryFlags::none },
	{ "b",					OBJECT_MODEL_FUNC(self->nonlinearExtrusionB[context.GetLastIndex()], 3),							ObjectModelEntryFlags::none },
	{ "upperLimit",			OBJECT_MODEL_FUNC(self->nonlinearExtrusionLimit[context.GetLastIndex()], 2),						ObjectModelEntryFlags::none },

#if HAS_12V_MONITOR
	// 6. v12 members
	{ "current",			OBJECT_MODEL_FUNC(self->GetV12Voltages().current, 1),												ObjectModelEntryFlags::live },
	{ "max",				OBJECT_MODEL_FUNC(self->GetV12Voltages().max, 1),													ObjectModelEntryFlags::none },
	{ "min",				OBJECT_MODEL_FUNC(self->GetV12Voltages().min, 1),													ObjectModelEntryFlags::none },
#endif

	// 7. move.axes[].microstepping members
	{ "interpolated",		OBJECT_MODEL_FUNC((self->microstepping[context.GetLastIndex()] & 0x8000) != 0),						ObjectModelEntryFlags::none },
	{ "value",				OBJECT_MODEL_FUNC((int32_t)(self->microstepping[context.GetLastIndex()] & 0x7FFF)),					ObjectModelEntryFlags::none },

	// 8. move.extruders[].microstepping members
	{ "interpolated",		OBJECT_MODEL_FUNC((self->microstepping[ExtruderToLogicalDrive(context.GetLastIndex())] & 0x8000) != 0),		ObjectModelEntryFlags::none },
	{ "value",				OBJECT_MODEL_FUNC((int32_t)(self->microstepping[ExtruderToLogicalDrive(context.GetLastIndex())] & 0x7FFF)),	ObjectModelEntryFlags::none },
};

constexpr uint8_t Platform::objectModelTableDescriptor[] =
{
	9,																		// number of sections
	11 + HAS_LINUX_INTERFACE + HAS_MASS_STORAGE + HAS_12V_MONITOR + SUPPORT_CAN_EXPANSION + SUPPORT_12864_LCD + MCU_HAS_UNIQUE_ID,		// section 0: boards[0]
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
	18,																		// section 3: move.axes[]
	13,																		// section 4: move.extruders[]
	3,																		// section 5: move.extruders[].nonlinear
#if HAS_12V_MONITOR
	3,																		// section 6: v12
#else
	0,																		// section 6: v12
#endif
	2,																		// section 7: move.axes[].microstepping
	2,																		// section 8: move.extruders[].microstepping
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
#if HAS_SMART_DRIVERS
	nextDriveToPoll(0),
#endif
	lastFanCheckTime(0),
#if HAS_AUX_DEVICES
	panelDueUpdater(nullptr),
#endif
#if HAS_MASS_STORAGE
	sysDir(nullptr),
#endif
	tickState(0), debugCode(0),
	lastWarningMillis(0),
#if SUPPORT_LASER
	lastLaserPwm(0.0),
#endif
	deferredPowerDown(false)
{
}

// Initialise the Platform. Note: this is the first module to be initialised, so don't call other modules from here!
void Platform::Init() noexcept
{
#if defined(DUET3) || defined(DUET3MINI)
	pinMode(EthernetPhyResetPin, OUTPUT_LOW);			// hold the Ethernet Phy chip in reset, hopefully this will prevent it being too noisy if Ethernet is not enabled
#endif

	// Deal with power first (we assume this doesn't depend on identifying the board type)
	pinMode(ATX_POWER_PIN, OUTPUT_LOW);

	// Make sure the on-board drivers are disabled
#if defined(DUET_NG) || defined(PCCB_10) || defined(PCCB_08_X5)
	pinMode(GlobalTmc2660EnablePin, OUTPUT_HIGH);
#elif defined(DUET_M) || defined(PCCB_08) || defined(PCCB_08_X5) || defined(DUET3MINI)
	pinMode(GlobalTmc22xxEnablePin, OUTPUT_HIGH);
#endif

	// Sort out which board we are running on (some firmware builds support more than one board variant)
	SetBoardType(BoardType::Auto);

#if SAME70
	DmacManager::Init();
#endif

#if MCU_HAS_UNIQUE_ID
	ReadUniqueId();
#endif

	// Real-time clock
	realTime = 0;

	// Comms
	baudRates[0] = MAIN_BAUD_RATE;
	commsParams[0] = 0;
	usbMutex.Create("USB");
#if SAME5x
    SERIAL_MAIN_DEVICE.Start();
#elif defined(__LPC17xx__)
	SERIAL_MAIN_DEVICE.begin(baudRates[0]);
#else
    SERIAL_MAIN_DEVICE.Start(UsbVBusPin);
#endif

#if HAS_AUX_DEVICES
    auxDevices[0].Init(&SERIAL_AUX_DEVICE);
	baudRates[1] = AUX_BAUD_RATE;
	commsParams[1] = 1;							// by default we require a checksum on data from the aux port, to guard against overrun errors
#endif

#if defined(SERIAL_AUX2_DEVICE) && !defined(DUET3_ATE)
    auxDevices[1].Init(&SERIAL_AUX2_DEVICE);
	baudRates[2] = AUX2_BAUD_RATE;
	commsParams[2] = 0;
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

#if HAS_MASS_STORAGE || HAS_LINUX_INTERFACE
	MassStorage::Init();
#endif

#ifdef __LPC17xx__
	// Load HW pin assignments from sdcard
	BoardConfig::Init();
	pinMode(ATX_POWER_PIN,(ATX_POWER_INVERTED==false)?OUTPUT_LOW:OUTPUT_HIGH);
#else
	// Deal with power first (we assume this doesn't depend on identifying the board type)
	pinMode(ATX_POWER_PIN,OUTPUT_LOW);
#endif

    // Ethernet networking defaults
	ipAddress = DefaultIpAddress;
	netMask = DefaultNetMask;
	gateWay = DefaultGateway;

	// Do hardware dependent initialisation
#if VARIABLE_NUM_DRIVERS
	numActualDirectDrivers = NumDirectDrivers;					// assume they are all available until we know otherwise
#endif

#if HAS_SMART_DRIVERS
# if defined(DUET_NG)
	// Test for presence of a DueX2 or DueX5 expansion board and work out how many TMC2660 drivers we have
	expansionBoard = DuetExpansion::DueXnInit();

	switch (expansionBoard)
	{
	case ExpansionBoardType::DueX2:
		numSmartDrivers = 7;
		break;
	case ExpansionBoardType::DueX5:
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
#endif

#if defined(DUET_06_085)
	ARRAY_INIT(defaultMacAddress, DefaultMacAddress);

	// Motor current setting on Duet 0.6 and 0.8.5
	I2C::Init();
	mcpExpansion.setMCP4461Address(0x2E);		// not required for mcpDuet, as this uses the default address
	ARRAY_INIT(potWipes, POT_WIPES);
	senseResistor = SENSE_RESISTOR;
	maxStepperDigipotVoltage = MAX_STEPPER_DIGIPOT_VOLTAGE;
	stepperDacVoltageRange = STEPPER_DAC_VOLTAGE_RANGE;
	stepperDacVoltageOffset = STEPPER_DAC_VOLTAGE_OFFSET;
#elif defined(__ALLIGATOR__)
	pinMode(EthernetPhyResetPin, INPUT);													// Init Ethernet Phy Reset Pin

	// Alligator Init DAC for motor current vref
	ARRAY_INIT(spiDacCS, SPI_DAC_CS);
	dacAlligator.Init(spiDacCS[0]);
	dacPiggy.Init(spiDacCS[1]);
	// Get macaddress from EUI48 eeprom
	eui48MacAddress.Init(Eui48csPin);
	if (!eui48MacAddress.getEUI48(defaultMacAddress))
	{
		ARRAY_INIT(defaultMacAddress, DefaultMacAddress);
	}

	Microstepping::Init();																	// Init Motor FAULT detect Pin
	pinMode(ExpansionVoltageLevelPin, ExpansionVoltageLevel==3 ? OUTPUT_LOW : OUTPUT_HIGH); // Init Expansion Voltage Level Pin
	pinMode(MotorFaultDetectPin,INPUT);														// Init Motor FAULT detect Pin
	pinMode(ExpansionPiggyDetectPin,INPUT);													// Init Expansion Piggy module presence Pin
	pinMode(FTDIconverterResetPin,INPUT);													// Init FTDI Serial Converter Reset Pin
	pinMode(SpiEEPROMcsPin,OUTPUT_HIGH);													// Init Spi EEPROM Cs pin, not implemented, default unselected
	pinMode(SpiFLASHcsPin,OUTPUT_HIGH);														// Init Spi FLASH Cs pin, not implemented, default unselected
#endif

#if defined(__LPC17xx__)
	if (hasDriverCurrentControl)
	{
		mcp4451.begin();
	}
	Microstepping::Init(); // basic class to remember the Microstepping.
#endif

	// Initialise endstops. On Duet 2 this must be done after testing for an expansion board.
	endstops.Init();

	// Axes
	for (size_t axis = 0; axis < MaxAxes; ++axis)
	{
		axisMinima[axis] = DefaultAxisMinimum;
		axisMaxima[axis] = DefaultAxisMaximum;

		maxFeedrates[axis] = DefaultXYMaxFeedrate;
		accelerations[axis] = DefaultXYAcceleration;
		driveStepsPerUnit[axis] = DefaultXYDriveStepsPerUnit;
		instantDvs[axis] = DefaultXYInstantDv;
	}

	// We use different defaults for the Z axis
	maxFeedrates[Z_AXIS] = DefaultZMaxFeedrate;
	accelerations[Z_AXIS] = DefaultZAcceleration;
	driveStepsPerUnit[Z_AXIS] = DefaultZDriveStepsPerUnit;
	instantDvs[Z_AXIS] = DefaultZInstantDv;

	// Extruders
	for (size_t drive = MaxAxes; drive < MaxAxesPlusExtruders; ++drive)
	{
		maxFeedrates[drive] = DefaultEMaxFeedrate;
		accelerations[drive] = DefaultEAcceleration;
		driveStepsPerUnit[drive] = DefaultEDriveStepsPerUnit;
		instantDvs[drive] = DefaultEInstantDv;
	}

	minimumMovementSpeed = DefaultMinFeedrate;
	axisMaximaProbed.Clear();
	axisMinimaProbed.Clear();
	idleCurrentFactor = DefaultIdleCurrentFactor;

	// Motors

	// Clear out the axis and extruder driver bitmaps
	for (size_t i = 0; i < MaxAxesPlusExtruders; ++i)
	{
		driveDriverBits[i] = 0;
	}

	// Set up the bitmaps for direct driver access
	for (size_t driver = 0; driver < NumDirectDrivers; ++driver)
	{
		driveDriverBits[driver + MaxAxesPlusExtruders] = StepPins::CalcDriverBitmap(driver);
	}

	// Set up the local drivers
	for (size_t driver = 0; driver < NumDirectDrivers; ++driver)
	{
		directions[driver] = true;								// drive moves forwards by default
		enableValues[driver] = 0;								// assume active low enable signal

#if SUPPORT_REMOTE_COMMANDS
		remotePressureAdvance[driver] = 0.0;
#endif
		// Set up the control pins
		pinMode(STEP_PINS[driver], OUTPUT_LOW);
		pinMode(DIRECTION_PINS[driver], OUTPUT_LOW);
#if !defined(DUET3) && !defined(DUET3MINI)
		pinMode(ENABLE_PINS[driver], OUTPUT_HIGH);				// this is OK for the TMC2660 CS pins too
#endif
	}

	// Set up the axis+extruder arrays
	for (size_t drive = 0; drive < MaxAxesPlusExtruders; drive++)
	{
		driverState[drive] = DriverStatus::disabled;
		driveDriverBits[drive] = 0;
		motorCurrents[drive] = 0.0;
		motorCurrentFraction[drive] = 1.0;
		standstillCurrentPercent[drive] = DefaultStandstillCurrentPercent;
		microstepping[drive] = 16 | 0x8000;						// x16 with interpolation
	}

	// Set up default axis mapping
	for (size_t axis = 0; axis < MinAxes; ++axis)
	{
#ifdef PCCB
		const size_t driver = (axis + 1) % 3;					// on PCCB we map axes X Y Z to drivers 1 2 0
#else
		const size_t driver = axis;								// on most boards we map axes straight through to drives
#endif
		axisDrivers[axis].numDrivers = 1;
		axisDrivers[axis].driverNumbers[0].SetLocal(driver);
		driveDriverBits[axis] = StepPins::CalcDriverBitmap(driver);	// overwrite the default value set up earlier
	}
	linearAxes = AxesBitmap::MakeLowestNBits(3);				// XYZ axes are linear

	for (size_t axis = MinAxes; axis < MaxAxes; ++axis)
	{
		axisDrivers[axis].numDrivers = 0;
	}

	// Set up default extruders
	for (size_t extr = 0; extr < MaxExtruders; ++extr)
	{
		extruderDrivers[extr].SetLocal(extr + MinAxes);			// set up default extruder drive mapping
		driveDriverBits[ExtruderToLogicalDrive(extr)] = StepPins::CalcDriverBitmap(extr + MinAxes);
		pressureAdvance[extr] = 0.0;
#if SUPPORT_NONLINEAR_EXTRUSION
		nonlinearExtrusionA[extr] = nonlinearExtrusionB[extr] = 0.0;
		nonlinearExtrusionLimit[extr] = DefaultNonlinearExtrusionLimit;
#endif
	}

	for (uint32_t& entry : slowDriverStepTimingClocks)
	{
		entry = 0;												// reset all to zero as we have no known slow drivers yet
	}
	slowDriversBitmap = 0;										// assume no drivers need extended step pulse timing
	EnableAllSteppingDrivers();									// no drivers disabled

	driversPowered = false;

#if HAS_SMART_DRIVERS
	// Initialise TMC driver module
# if SUPPORT_TMC51xx
	SmartDrivers::Init();
# elif SUPPORT_TMC22xx
#  if TMC22xx_VARIABLE_NUM_DRIVERS
	SmartDrivers::Init(numSmartDrivers);
#  else
	SmartDrivers::Init();
#  endif
# else
	SmartDrivers::Init(ENABLE_PINS, numSmartDrivers);
# endif
	temperatureShutdownDrivers.Clear();
	temperatureWarningDrivers.Clear();
	shortToGroundDrivers.Clear();
	openLoadADrivers.Clear();
	openLoadBDrivers.Clear();
	notOpenLoadADrivers.Clear();
	notOpenLoadBDrivers.Clear();
#endif

#if HAS_STALL_DETECT
	stalledDrivers.Clear();
	logOnStallDrivers.Clear();
	pauseOnStallDrivers.Clear();
	rehomeOnStallDrivers.Clear();
	stalledDriversToLog.Clear();
	stalledDriversToPause.Clear();
	stalledDriversToRehome.Clear();
#endif

#if HAS_VOLTAGE_MONITOR
	autoSaveEnabled = false;
	autoSaveState = AutoSaveState::starting;
#endif

#if HAS_SMART_DRIVERS && (HAS_VOLTAGE_MONITOR || HAS_12V_MONITOR)
	warnDriversNotPowered = false;
#endif

	extrusionAncilliaryPwmValue = 0.0;

	// Initialise the configured heaters to just the default bed heater (there are no default chamber heaters)
	configuredHeaters.Clear();

#if !defined(DUET3) && !defined(DUET3MINI)
	if (DefaultBedHeater >= 0)
	{
		configuredHeaters.SetBit(DefaultBedHeater);
	}
#endif

	// Enable pullups on all the SPI CS pins. This is required if we are using more than one device on the SPI bus.
	// Otherwise, when we try to initialise the first device, the other devices may respond as well because their CS lines are not high.
	for (Pin p : SpiTempSensorCsPins)
	{
		pinMode(p, INPUT_PULLUP);
	}

	// If MISO from a MAX31856 board breaks after initialising the MAX31856 then if MISO floats low and reads as all zeros, this looks like a temperature of 0C and no error.
	// Enable the pullup resistor, with luck this will make it float high instead.
#if SAM3XA
	pinMode(APIN_SHARED_SPI_MISO, INPUT_PULLUP);
#elif defined(__LPC17xx__) || SAME5x
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
	AnalogIn::EnableTemperatureSensor(0, tpFilter.CallbackFeedIntoFilter, &tpFilter, 1, 0);
	tcFilter.Init(0);
	AnalogIn::EnableTemperatureSensor(1, tcFilter.CallbackFeedIntoFilter, &tcFilter, 1, 0);
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
	nozzleDiameter = NOZZLE_DIAMETER;
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
	currentVin = highestVin = 0;
	lowestVin = 9999;
	numVinUnderVoltageEvents = previousVinUnderVoltageEvents = numVinOverVoltageEvents = previousVinOverVoltageEvents = 0;
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

#if MCU_HAS_UNIQUE_ID

// Read the unique ID of the MCU, if it has one
void Platform::ReadUniqueId()
{
# if SAME5x
	for (size_t i = 0; i < 4; ++i)
	{
		uniqueId[i] = *reinterpret_cast<const uint32_t*>(SerialNumberAddresses[i]);
	}
# else
	memset(uniqueId, 0, sizeof(uniqueId));

	const bool cacheWasEnabled = Cache::Disable();
#if SAM4E || SAM4S || SAME70
	const bool success = Flash::ReadUniqueId(uniqueId);
#else
	const bool success = flash_read_unique_id(uniqueId) == 0;
#endif
	if (cacheWasEnabled)
	{
		Cache::Enable();
	}

	if (success)
	{
# endif
		// Put the checksum at the end
		// We only print 30 5-bit characters = 128 data bits + 22 checksum bits. So compress the 32 checksum bits into 22.
		uniqueId[4] = uniqueId[0] ^ uniqueId[1] ^ uniqueId[2] ^ uniqueId[3];
		uniqueId[4] ^= (uniqueId[4] >> 10);

		// On the Duet Ethernet and SAM E70, use the unique chip ID as most of the MAC address.
		// The unique ID is 128 bits long whereas the whole MAC address is only 48 bits,
		// so we can't guarantee that each Duet will get a unique MAC address this way.
		memset(defaultMacAddress.bytes, 0, sizeof(defaultMacAddress.bytes));
		defaultMacAddress.bytes[0] = 0xBE;					// use a fixed first byte with the locally-administered bit set
		const uint8_t * const idBytes = reinterpret_cast<const uint8_t *>(uniqueId);
		for (size_t i = 0; i < 15; ++i)
		{
			defaultMacAddress.bytes[(i % 5) + 1] ^= idBytes[i];
		}

		// Convert the unique ID and checksum to a string as 30 base5 alphanumeric digits
		char *digitPtr = uniqueIdChars;
		for (size_t i = 0; i < 30; ++i)
		{
			if ((i % 5) == 0 && i != 0)
			{
				*digitPtr++ = '-';
			}
			const size_t index = (i * 5) / 32;
			const size_t shift = (i * 5) % 32;
			uint32_t val = uniqueId[index] >> shift;
			if (shift > 32 - 5)
			{
				// We need some bits from the next dword too
				val |= uniqueId[index + 1] << (32 - shift);
			}
			val &= 31;
			char c;
			if (val < 10)
			{
				c = val + '0';
			}
			else
			{
				c = val + ('A' - 10);
				// We have 26 letters in the usual A-Z alphabet and we only need 22 of them plus 0-9.
				// So avoid using letters C, E, I and O which are easily mistaken for G, F, 1 and 0.
				if (c >= 'C')
				{
					++c;
				}
				if (c >= 'E')
				{
					++c;
				}
				if (c >= 'I')
				{
					++c;
				}
				if (c >= 'O')
				{
					++c;
				}
			}
			*digitPtr++ = c;
		}
		*digitPtr = 0;

# if !SAME5x
	}
	else
	{
		defaultMacAddress.SetDefault();
		strcpy(uniqueIdChars, "unknown");
	}
# endif
}

#endif

// Send the beep command to the aux channel. There is no flow control on this port, so it can't block for long.
void Platform::PanelDueBeep(int freq, int ms) noexcept
{
	MessageF(AuxMessage, "{\"beep_freq\":%d,\"beep_length\":%d}\n", freq, ms);
}

// Send a short message to the aux channel. There is no flow control on this port, so it can't block for long.
void Platform::SendPanelDueMessage(size_t auxNumber, const char* msg) noexcept
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
#if HAS_MASS_STORAGE
	MassStorage::CloseAllFiles();
#endif
#if HAS_SMART_DRIVERS
	SmartDrivers::Exit();
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

void Platform::SetIPAddress(IPAddress ip) noexcept
{
	ipAddress = ip;
	reprap.GetNetwork().SetEthernetIPAddress(ipAddress, gateWay, netMask);
}

void Platform::SetGateWay(IPAddress gw) noexcept
{
	gateWay = gw;
	reprap.GetNetwork().SetEthernetIPAddress(ipAddress, gateWay, netMask);
}

void Platform::SetNetMask(IPAddress nm) noexcept
{
	netMask = nm;
	reprap.GetNetwork().SetEthernetIPAddress(ipAddress, gateWay, netMask);
}

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
				SERIAL_MAIN_DEVICE.write(usbOutputBuffer->Read(bytesToWrite), bytesToWrite);
			}

			if (usbOutputBuffer->BytesLeft() == 0)
			{
				usbOutput.ReleaseFirstItem();
			}
			else
			{
				usbOutput.ApplyTimeout(SERIAL_MAIN_TIMEOUT);
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

#if defined(DUET3) || defined(DUET3MINI) || defined(__LPC17xx__)
# if SUPPORT_REMOTE_COMMANDS
	if (CanInterface::InExpansionMode())
	{
		if (StepTimer::IsSynced())
		{
			digitalWrite(DiagPin, XNor(DiagOnPolarity, StepTimer::GetMasterTime() & (1u << 19)) != 0);
		}
		else
		{
			digitalWrite(DiagPin, XNor(DiagOnPolarity, StepTimer::GetTimerTicks() & (1u << 17)) != 0);
		}
	}
	else
# endif
	{
		// Blink the LED at about 2Hz. Duet 3 expansion boards will blink in sync when they have established clock sync with us.
		digitalWrite(DiagPin, XNor(DiagOnPolarity, StepTimer::GetTimerTicks() & (1u << 19)) != 0);
	}
#endif

#if HAS_MASS_STORAGE
	MassStorage::Spin();
#endif

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
		}
		if (currentMcuTemperature < lowestMcuTemperature)
		{
			lowestMcuTemperature = currentMcuTemperature;
		}
	}
#endif

	// Diagnostics test
	if (debugCode == (unsigned int)DiagnosticTestType::TestSpinLockup)
	{
		for (;;) {}
	}

	// Check whether the TMC drivers need to be initialised.
	// The tick ISR also looks for over-voltage events, but it just disables the driver without changing driversPowerd or numVinOverVoltageEvents
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

#if HAS_12V_MONITOR
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
#if HAS_SMART_DRIVERS
			// Check one TMC2660 or TMC2224 for temperature warning or temperature shutdown
			if (enableValues[nextDriveToPoll] >= 0)				// don't poll driver if it is flagged "no poll"
			{
				const uint32_t stat = SmartDrivers::GetAccumulatedStatus(nextDriveToPoll, 0);
				const DriversBitmap mask = DriversBitmap::MakeFromBits(nextDriveToPoll);
				if (stat & TMC_RR_OT)
				{
					temperatureShutdownDrivers |= mask;
				}
				else if (stat & TMC_RR_OTPW)
				{
					temperatureWarningDrivers |= mask;
				}
				if (stat & TMC_RR_S2G)
				{
					shortToGroundDrivers |= mask;
				}
				else
				{
					shortToGroundDrivers &= ~mask;
				}

				// The driver often produces a transient open-load error, especially in stealthchop mode, so we require the condition to persist before we report it.
				// Also, false open load indications persist when in standstill, if the phase has zero current in that position
				if ((stat & TMC_RR_OLA) != 0)
				{
					if (!openLoadATimer.IsRunning())
					{
						openLoadATimer.Start();
						openLoadADrivers.Clear();
						notOpenLoadADrivers.Clear();
					}
					openLoadADrivers |= mask;
				}
				else if (openLoadATimer.IsRunning())
				{
					notOpenLoadADrivers |= mask;
					if (openLoadADrivers.Disjoint(~notOpenLoadADrivers))
					{
						openLoadATimer.Stop();
					}
				}

				if ((stat & TMC_RR_OLB) != 0)
				{
					if (!openLoadBTimer.IsRunning())
					{
						openLoadBTimer.Start();
						openLoadBDrivers.Clear();
						notOpenLoadBDrivers.Clear();
					}
					openLoadBDrivers |= mask;
				}
				else if (openLoadBTimer.IsRunning())
				{
					notOpenLoadBDrivers |= mask;
					if (openLoadBDrivers.Disjoint(~notOpenLoadBDrivers))
					{
						openLoadBTimer.Stop();
					}
				}

# if HAS_STALL_DETECT
				if ((stat & TMC_RR_SG) != 0)
				{
					if (stalledDrivers.Disjoint(mask))
					{
						// This stall is new so check whether we need to perform some action in response to the stall
						if (rehomeOnStallDrivers.Intersects(mask))
						{
							stalledDriversToRehome |= mask;
						}
						else if (pauseOnStallDrivers.Intersects(mask))
						{
							stalledDriversToPause |= mask;
						}
						else if (logOnStallDrivers.Intersects(mask))
						{
							stalledDriversToLog |= mask;
						}
					}
					stalledDrivers |= mask;
				}
				else
				{
					stalledDrivers &= ~mask;
				}
# endif
			}

# if HAS_STALL_DETECT
			// Action any pause or rehome actions due to motor stalls. This may have to be done more than once.
			if (stalledDriversToRehome.IsNonEmpty())
			{
				if (reprap.GetGCodes().ReHomeOnStall(stalledDriversToRehome))
				{
					stalledDriversToRehome.Clear();
				}
			}
			else if (stalledDriversToPause.IsNonEmpty())
			{
				if (reprap.GetGCodes().PauseOnStall(stalledDriversToPause))
				{
					stalledDriversToPause.Clear();
				}
			}
# endif
			// Advance drive number ready for next time
			++nextDriveToPoll;
			if (nextDriveToPoll == numSmartDrivers)
			{
				nextDriveToPoll = 0;
			}
#endif		// HAS_SMART_DRIVERS
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
		openLoadATimer.Stop();
		openLoadBTimer.Stop();
		temperatureShutdownDrivers.Clear();
		temperatureWarningDrivers.Clear();
		shortToGroundDrivers.Clear();
		openLoadADrivers.Clear();
		openLoadBDrivers.Clear();
		notOpenLoadADrivers.Clear();
		notOpenLoadBDrivers.Clear();
#endif
	}

#if HAS_SMART_DRIVERS
	SmartDrivers::Spin(driversPowered);
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

		if (deferredPowerDown && !thermostaticFanRunning)
		{
			AtxPowerOff(false);
		}

		// Check whether it is time to report any faults (do this after checking fans in case driver cooling fans are turned on)
		if (now - lastWarningMillis > MinimumWarningInterval)
		{
			bool reported = false;
#if HAS_SMART_DRIVERS
			ReportDrivers(ErrorMessage, shortToGroundDrivers, "short-to-ground", reported);
			ReportDrivers(ErrorMessage, temperatureShutdownDrivers, "over temperature shutdown", reported);
			if (openLoadATimer.CheckAndStop(OpenLoadTimeout))
			{
				ReportDrivers(WarningMessage, openLoadADrivers, "motor phase A may be disconnected", reported);
			}
			if (openLoadBTimer.CheckAndStop(OpenLoadTimeout))
			{
				ReportDrivers(WarningMessage, openLoadBDrivers, "motor phase B may be disconnected", reported);
			}

			// Don't warn about a hot driver if we recently turned on a fan to cool it
			if (temperatureWarningDrivers.IsNonEmpty())
			{
				const DriversBitmap driversMonitored[NumTmcDriversSenseChannels] =
# ifdef DUET_NG
					{ DriversBitmap::MakeLowestNBits(5), DriversBitmap::MakeLowestNBits(5).ShiftUp(5) };			// first channel is Duet, second is DueX5
# elif defined(DUET_M)
					{ DriversBitmap::MakeLowestNBits(5), DriversBitmap::MakeLowestNBits(2).ShiftUp(5) };			// first channel is Duet, second is daughter board
# else
					{ DriversBitmap::MakeLowestNBits(NumDirectDrivers) };
# endif
				for (unsigned int i = 0; i < NumTmcDriversSenseChannels; ++i)
				{
					if (driversFanTimers[i].IsRunning())
					{
						const bool timedOut = driversFanTimers[i].CheckAndStop(DriverCoolingTimeout);
						if (!timedOut)
						{
							temperatureWarningDrivers &= ~driversMonitored[i];
						}
					}
				}
				ReportDrivers(WarningMessage, temperatureWarningDrivers, "high temperature", reported);
			}
#endif

#if HAS_STALL_DETECT
			// Check for stalled drivers that need to be reported and logged
			if (stalledDriversToLog.IsNonEmpty() && reprap.GetGCodes().IsReallyPrinting())
			{
				String<StringLength100> scratchString;
				ListDrivers(scratchString.GetRef(), stalledDriversToLog);
				stalledDriversToLog.Clear();
				MessageF(WarningMessage, "Driver(s)%s stalled at Z height %.2f", scratchString.c_str(), (double)reprap.GetMove().LiveCoordinate(Z_AXIS, reprap.GetCurrentTool()));
				reported = true;
			}
#endif

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
				MessageF(WarningMessage, "12V under-voltage event (%.1fV)", (double)AdcReadingToPowerVoltage(lastV12UnderVoltageValue));
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
				lastWarningMillis = now;
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
	// Flush the log file it it is time. This may take some time, so do it last.
	if (logger != nullptr)
	{
		logger->Flush(false);
	}
#endif

}

#if HAS_SMART_DRIVERS

// Report driver status conditions that require attention.
// Sets 'reported' if we reported anything, else leaves 'reported' alone.
void Platform::ReportDrivers(MessageType mt, DriversBitmap& whichDrivers, const char* text, bool& reported) noexcept
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

bool Platform::HasVinPower() const noexcept
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
#if HAS_LINUX_INTERFACE
			|| reprap.UsingLinuxInterface()
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
# elif SAM3XA
	return (voltage - 0.8) * (1000.0/2.65) + 27.0 + mcuTemperatureAdjust;			// accuracy at 27C is +/-45C
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

#ifdef __LPC17xx__
	// Interrupt for GPIO pins. Only port 0 and 2 support interrupts and both share EINT3
	NVIC_SetPriority(EINT3_IRQn, NvicPriorityPins);
#elif SAME5x
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
#elif defined(__LPC17xx__)
	NVIC_SetPriority(USB_IRQn, NvicPriorityUSB);
#else
# error Unsupported processor
#endif

#if defined(DUET_NG) || defined(DUET_M) || defined(DUET_06_085)
	NVIC_SetPriority(I2C_IRQn, NvicPriorityTwi);
#elif defined(__LPC17xx__)
	NVIC_SetPriority(I2C0_IRQn, NvicPriorityTwi);
	NVIC_SetPriority(I2C1_IRQn, NvicPriorityTwi);
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

#if defined(__LPC17xx__)
	// set rest of the Timer Interrupt priorities
	// Timer 0 is used for step generation (set elsewhere)
	NVIC_SetPriority(TIMER1_IRQn, 8);                       //Timer 1 is currently unused
	NVIC_SetPriority(TIMER2_IRQn, NvicPriorityTimerServo);  //Timer 2 runs the PWM for Servos at 50hz
	NVIC_SetPriority(TIMER3_IRQn, NvicPriorityTimerPWM);    //Timer 3 runs the microsecond free running timer to generate heater/fan PWM
#endif

	StepTimer::Init();										// initialise the step pulse timer

   // Tick interrupt for ADC conversions
	tickState = 0;
	currentFilterNumber = 0;
}

//*************************************************************************************************

// Debugging variables
//extern "C" uint32_t longestWriteWaitTime, shortestWriteWaitTime, longestReadWaitTime, shortestReadWaitTime;
//extern uint32_t maxRead, maxWrite;

// Return diagnostic information
void Platform::Diagnostics(MessageType mtype) noexcept
{
#if USE_CACHE && (SAM4E || SAME5x)
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

#if SAME5x
	{
		String<StringLength100> resetString;
		resetString.printf("Last reset %02d:%02d:%02d ago, cause", (unsigned int)(now/3600), (unsigned int)((now % 3600)/60), (unsigned int)(now % 60));
		const uint8_t resetReason = RSTC->RCAUSE.reg;
		// The datasheet says only one of these bits will be set, but we don't assume that
		if (resetReason & RSTC_RCAUSE_POR)		{ resetString.cat(": power up"); }
		if (resetReason & RSTC_RCAUSE_BODCORE)	{ resetString.cat(": core brownout"); }
		if (resetReason & RSTC_RCAUSE_BODVDD)	{ resetString.cat(": Vdd brownout"); }
		if (resetReason & RSTC_RCAUSE_WDT)		{ resetString.cat(": watchdog"); }
		if (resetReason & RSTC_RCAUSE_NVM)		{ resetString.cat(": NVM"); }
		if (resetReason & RSTC_RCAUSE_EXT)		{ resetString.cat(": reset button"); }
		if (resetReason & RSTC_RCAUSE_SYST)		{ resetString.cat(": software"); }
		if (resetReason & RSTC_RCAUSE_BACKUP)	{ resetString.cat(": backup/hibernate"); }
		resetString.cat('\n');
		Message(mtype, resetString.c_str());
	}
#elif defined(__LPC17xx__)
	// Reset Reason
	MessageF(mtype, "Last reset %02d:%02d:%02d ago, cause: ",
			 (unsigned int)(now/3600), (unsigned int)((now % 3600)/60), (unsigned int)(now % 60));

	if (LPC_SYSCTL->RSID & RSID_POR) { Message(mtype, "[power up]"); }
	if (LPC_SYSCTL->RSID & RSID_EXTR) { Message(mtype, "[reset button]"); }
	if (LPC_SYSCTL->RSID & RSID_WDTR) { Message(mtype, "[watchdog]"); }
	if (LPC_SYSCTL->RSID & RSID_BODR) { Message(mtype, "[brownout]"); }
	if (LPC_SYSCTL->RSID & RSID_SYSRESET) { Message(mtype, "[software]"); }
	if (LPC_SYSCTL->RSID & RSID_LOCKUP) { Message(mtype, "[lockup]"); }

	Message(mtype, "\n");
#else
	const char* resetReasons[8] = { "power up", "backup", "watchdog", "software",
# ifdef DUET_NG
	// On the SAM4E a watchdog reset may be reported as a user reset because of the capacitor on the NRST pin.
	// The SAM4S is the same but the Duet M has a diode in the reset circuit to avoid this problem.
									"reset button or watchdog",
# else
									"reset button",
# endif
									"?", "?", "?" };
	MessageF(mtype, "Last reset %02d:%02d:%02d ago, cause: %s\n",
			(unsigned int)(now/3600), (unsigned int)((now % 3600)/60), (unsigned int)(now % 60),
			resetReasons[(REG_RSTC_SR & RSTC_SR_RSTTYP_Msk) >> RSTC_SR_RSTTYP_Pos]);
#endif

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
			srd->Print(mtype, slot);
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

#if HAS_CPU_TEMP_SENSOR
	// Show the MCU temperatures
	const float currentMcuTemperature = GetCpuTemperature();
	MessageF(mtype, "MCU temperature: min %.1f, current %.1f, max %.1f\n",
		(double)lowestMcuTemperature, (double)currentMcuTemperature, (double)highestMcuTemperature);
	lowestMcuTemperature = highestMcuTemperature = currentMcuTemperature;
#endif

#if HAS_VOLTAGE_MONITOR
	// Show the supply voltage
	MessageF(mtype, "Supply voltage: min %.1f, current %.1f, max %.1f, under voltage events: %" PRIu32 ", over voltage events: %" PRIu32 ", power good: %s\n",
		(double)AdcReadingToPowerVoltage(lowestVin), (double)AdcReadingToPowerVoltage(currentVin), (double)AdcReadingToPowerVoltage(highestVin),
				numVinUnderVoltageEvents, numVinOverVoltageEvents,
				(HasVinPower()) ? "yes" : "no");
	lowestVin = highestVin = currentVin;
#endif

#if HAS_12V_MONITOR
	// Show the 12V rail voltage
	MessageF(mtype, "12V rail voltage: min %.1f, current %.1f, max %.1f, under voltage events: %" PRIu32 "\n",
		(double)AdcReadingToPowerVoltage(lowestV12), (double)AdcReadingToPowerVoltage(currentV12), (double)AdcReadingToPowerVoltage(highestV12), numV12UnderVoltageEvents);
	lowestV12 = highestV12 = currentV12;
#endif

	// Show the motor position and stall status
	for (size_t drive = 0; drive < NumDirectDrivers; ++drive)
	{
		String<StringLength256> driverStatus;
		driverStatus.printf("Driver %u: position %" PRIi32, drive, reprap.GetMove().GetEndPoint(drive));
#if HAS_SMART_DRIVERS
		if (drive < numSmartDrivers)
		{
			driverStatus.cat(", ");
			SmartDrivers::AppendDriverStatus(drive, driverStatus.GetRef());
		}
#endif
		driverStatus.cat('\n');
		Message(mtype, driverStatus.c_str());
	}

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

#if USE_CACHE && (SAM4E || SAME5x)
	MessageF(mtype, "Cache data hit count %" PRIu32 "\n", cacheCount);
#endif

	reprap.Timing(mtype);

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
	cpu_irq_disable();
	asm volatile("":::"memory");
	uint32_t now1 = SysTick->VAL;
	const uint32_t ret = isqrt64(arg);
	uint32_t now2 = SysTick->VAL;
	asm volatile("":::"memory");
	cpu_irq_enable();
	now1 &= 0x00FFFFFF;
	now2 &= 0x00FFFFFF;
	timeAcc += ((now1 > now2) ? now1 : now1 + (SysTick->LOAD & 0x00FFFFFF) + 1) - now2;
	return ret;
}

GCodeResult Platform::DiagnosticTest(GCodeBuffer& gb, const StringRef& reply, OutputBuffer*& buf, unsigned int d) THROWS(GCodeException)
{
	switch (d)
	{
	case (unsigned int)DiagnosticTestType::PrintTestReport:
		{
			bool testFailed = false;
			if (!OutputBuffer::Allocate(buf))
			{
				reply.copy("No output buffer");
				return GCodeResult::error;
			}

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

				const float voltage = AdcReadingToPowerVoltage(currentV12);
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
			for (size_t driver = 0; driver < numSmartDrivers; ++driver)
			{
				const uint32_t stat = SmartDrivers::GetAccumulatedStatus(driver, 0xFFFFFFFF);
				if ((stat & (TMC_RR_OT || TMC_RR_OTPW)) != 0)
				{
					buf->lcatf("Driver %u reports over temperature", driver);
					driversOK = false;
				}
				if ((stat & TMC_RR_S2G) != 0)
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
				buf->lcatf("Board ID: %s", GetUniqueIdString());
			}
#endif
		}
		break;

	case (int)DiagnosticTestType::OutputBufferStarvation:
		{
			OutputBuffer *buf;
			while (OutputBuffer::Allocate(buf)) { }
			OutputBuffer::ReleaseAll(buf);
		}
		break;

	case (int)DiagnosticTestType::SetWriteBuffer:
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
			const uint32_t address = gb.GetUIValue();
			uint32_t val;
			bool dummy;
			deliberateError = true;								// in case the memory access causes a fault
			if (gb.TryGetUIValue('V', val, dummy))
			{
				*reinterpret_cast<uint32_t*>(address) = val;
				__DSB();										// allow the write to complete in case it raises a fault
			}
			else
			{
				reply.printf("Address %08" PRIx32 " value %08" PRIx32, address, *reinterpret_cast<const uint32_t*>(address));
			}
			deliberateError = false;
		}
		break;

	case (unsigned int)DiagnosticTestType::PrintMoves:
		DDA::PrintMoves();
		break;

	case (unsigned int)DiagnosticTestType::TimeCalculations:	// Show the square root calculation time. Caution: may disable interrupt for several tens of microseconds.
		{
			bool ok1 = true;
			uint32_t tim1 = 0;
			for (uint32_t i = 0; i < 100; ++i)
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
			constexpr uint32_t iterations = 100;				// use a value that divides into one million
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

			reply.printf("Square roots: 62-bit %.2fus %s, 32-bit %.2fus %s",
					(double)((float)(tim1 * (1'000'000/iterations))/SystemCoreClock), (ok1) ? "ok" : "ERROR",
							(double)((float)(tim2 * (1'000'000/iterations))/SystemCoreClock), (ok2) ? "ok" : "ERROR");
		}

		// We now also time sine and cosine in the same test
		{
			uint32_t tim1 = 0;
			constexpr uint32_t iterations = 100;				// use a value that divides into one million
			for (unsigned int i = 0; i < iterations; ++i)
			{
				const float angle = 0.01 * i;

				cpu_irq_disable();
				asm volatile("":::"memory");
				uint32_t now1 = SysTick->VAL;
				(void)RepRap::SinfCosf(angle);
				uint32_t now2 = SysTick->VAL;
				asm volatile("":::"memory");
				cpu_irq_enable();
				now1 &= 0x00FFFFFF;
				now2 &= 0x00FFFFFF;
				tim1 += ((now1 > now2) ? now1 : now1 + (SysTick->LOAD & 0x00FFFFFF) + 1) - now2;
			}

			// We no longer calculate sin and cos for doubles because it pulls in those library functions, which we don't otherwise need
			reply.lcatf("Float sine + cosine: %.2fus", (double)((float)(tim1 * (1'000'000/iterations))/SystemCoreClock));
		}

		// We also time floating point square root so we can compare it with sine/cosine in order to consider various optimisations
		{
			uint32_t tim1 = 0;
			constexpr uint32_t iterations = 100;				// use a value that divides into one million
			float val = 10000.0;
			for (unsigned int i = 0; i < iterations; ++i)
			{

				cpu_irq_disable();
				asm volatile("":::"memory");
				uint32_t now1 = SysTick->VAL;
				val = RepRap::FastSqrtf(val);
				uint32_t now2 = SysTick->VAL;
				asm volatile("":::"memory");
				cpu_irq_enable();
				now1 &= 0x00FFFFFF;
				now2 &= 0x00FFFFFF;
				tim1 += ((now1 > now2) ? now1 : now1 + (SysTick->LOAD & 0x00FFFFFF) + 1) - now2;
			}

			// We no longer calculate sin and cos for doubles because it pulls in those library functions, which we don't otherwise need
			reply.lcatf("Float sqrt: %.2fus", (double)((float)(tim1 * (1'000'000/iterations))/SystemCoreClock));
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
				"DDA %u, DM %u, Tool %u, GCodeBuffer %u, heater %u"
#if HAS_NETWORKING
				", HTTP resp %u, FTP resp %u, Telnet resp %u"
#endif
				, sizeof(DDA), sizeof(DriveMovement), sizeof(Tool), sizeof(GCodeBuffer), sizeof(Heater)
#if HAS_NETWORKING
				, sizeof(HttpResponder), sizeof(FtpResponder), sizeof(TelnetResponder)
#endif
			);
		break;

	case (unsigned int)DiagnosticTestType::PrintObjectAddresses:
		MessageF(MessageType::GenericMessage,
					"Platform %08" PRIx32 "-%08" PRIx32
#if HAS_LINUX_INTERFACE
					"\nLinuxInterface %08" PRIx32 "-%08" PRIx32
#endif
					"\nNetwork %08" PRIx32 "-%08" PRIx32
					"\nGCodes %08" PRIx32 "-%08" PRIx32
					"\nMove %08" PRIx32 "-%08" PRIx32
					"\nHeat %08" PRIx32 "-%08" PRIx32
					, reinterpret_cast<uint32_t>(this), reinterpret_cast<uint32_t>(this) + sizeof(Platform) - 1
#if HAS_LINUX_INTERFACE
					, reinterpret_cast<uint32_t>(&reprap.GetLinuxInterface())
					, (reinterpret_cast<uint32_t>(&reprap.GetLinuxInterface()) == 0) ? 0 : reinterpret_cast<uint32_t>(&reprap.GetLinuxInterface()) + sizeof(LinuxInterface)
#endif
					, reinterpret_cast<uint32_t>(&reprap.GetNetwork()), reinterpret_cast<uint32_t>(&reprap.GetNetwork()) + sizeof(Network) - 1
					, reinterpret_cast<uint32_t>(&reprap.GetGCodes()), reinterpret_cast<uint32_t>(&reprap.GetGCodes()) + sizeof(GCodes) - 1
					, reinterpret_cast<uint32_t>(&reprap.GetMove()), reinterpret_cast<uint32_t>(&reprap.GetMove()) + sizeof(Move) - 1
					, reinterpret_cast<uint32_t>(&reprap.GetHeat()), reinterpret_cast<uint32_t>(&reprap.GetHeat()) + sizeof(Heat) - 1
				);

		MessageF(MessageType::GenericMessage,
					"\nPrintMonitor %08" PRIx32 "-%08" PRIx32
					"\nFansManager %08" PRIx32 "-%08" PRIx32
#if SUPPORT_ROLAND
					"\nRoland %08" PRIx32 "-%08" PRIx32
#endif
#if SUPPORT_SCANNER
					"\nScanner %08" PRIx32 "-%08" PRIx32
#endif
#if SUPPORT_IOBITS
					"\nPortControl %08" PRIx32 "-%08" PRIx32
#endif
#if SUPPORT_12864_LCD
					"\nDisplay %08" PRIx32 "-%08" PRIx32
#endif
#if SUPPORT_CAN_EXPANSION
					"\nExpansionManager %08" PRIx32 "-%08" PRIx32
#endif

					, reinterpret_cast<uint32_t>(&reprap.GetPrintMonitor()), reinterpret_cast<uint32_t>(&reprap.GetPrintMonitor()) + sizeof(PrintMonitor) - 1
					, reinterpret_cast<uint32_t>(&reprap.GetFansManager()), reinterpret_cast<uint32_t>(&reprap.GetFansManager()) + sizeof(FansManager) - 1
#if SUPPORT_ROLAND
					, reinterpret_cast<uint32_t>(&reprap.GetRoland()), reinterpret_cast<uint32_t>(&reprap.GetRoland()) + sizeof(Roland) - 1
#endif
#if SUPPORT_SCANNER
					, reinterpret_cast<uint32_t>(&reprap.GetScanner()), reinterpret_cast<uint32_t>(&reprap.GetScanner()) + sizeof(Scanner) - 1
#endif
#if SUPPORT_IOBITS
					, reinterpret_cast<uint32_t>(&reprap.GetPortControl()), reinterpret_cast<uint32_t>(&reprap.GetPortControl()) + sizeof(PortControl) - 1
#endif
#if SUPPORT_12864_LCD
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
			cpu_irq_disable();
			asm volatile("":::"memory");
			uint32_t now1 = SysTick->VAL;
			crc.Update(
#if SAME5x
						reinterpret_cast<const char*>(HSRAM_ADDR),
#else
						reinterpret_cast<const char*>(IRAM_ADDR),		// for the SAME70 this is in the non-cacheable RAM, which is the usual case when computing a CRC
#endif
						length);
			uint32_t now2 = SysTick->VAL;
			asm volatile("":::"memory");
			cpu_irq_enable();
			now1 &= 0x00FFFFFF;
			now2 &= 0x00FFFFFF;
			uint32_t tim1 = ((now1 > now2) ? now1 : now1 + (SysTick->LOAD & 0x00FFFFFF) + 1) - now2;
			reply.printf("CRC of %u bytes took %.2fus", length, (double)((1'000'000.0f * (float)tim1)/(float)SystemCoreClock));
		}
		break;

	case (unsigned int)DiagnosticTestType::TimeGetTimerTicks:
		{
			unsigned int i = 100;
			cpu_irq_disable();
			asm volatile("":::"memory");
			uint32_t now1 = SysTick->VAL;
			do
			{
				--i;
				(void)StepTimer::GetTimerTicks();
			} while (i != 0);
			uint32_t now2 = SysTick->VAL;
			asm volatile("":::"memory");
			cpu_irq_enable();
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

#ifdef DUET_NG
	case (unsigned int)DiagnosticTestType::PrintExpanderStatus:
		reply.printf("Expander status %04X\n", DuetExpansion::DiagnosticRead());
		break;
#endif

#ifdef __LPC17xx__
	// Diagnostic for LPC board configuration
	case (int)DiagnosticTestType::PrintBoardConfiguration:
		BoardConfig::Diagnostics(gb.GetResponseMessageType());
		break;
#endif

	default:
		break;
	}

	return GCodeResult::ok;
}

#if HAS_SMART_DRIVERS

// This is called when a fan that monitors driver temperatures is turned on when it was off
void Platform::DriverCoolingFansOnOff(DriverChannelsBitmap driverChannelsMonitored, bool on) noexcept
{
	driverChannelsMonitored.Iterate
		([this, on](unsigned int i, unsigned int) noexcept
			{
				if (on)
				{
					this->driversFanTimers[i].Start();
				}
				else
				{
					this->driversFanTimers[i].Stop();
				}
			}
		);
}

#endif

// Get the index of the averaging filter for an analog port.
// Note, the Thermistor code assumes that this is also the thermistor input number
int Platform::GetAveragingFilterIndex(const IoPort& port) const noexcept
{
	for (size_t i = 0; i < NumAdcFilters; ++i)
	{
		if (port.GetAnalogChannel() == filteredAdcChannels[i])
		{
			return (int)i;
		}
	}
	return -1;
}

void Platform::UpdateConfiguredHeaters() noexcept
{
	configuredHeaters.Clear();

	// Check bed heaters
	for (size_t i = 0; i < MaxBedHeaters; i++)
	{
		const int8_t bedHeater = reprap.GetHeat().GetBedHeater(i);
		if (bedHeater >= 0)
		{
			configuredHeaters.SetBit(bedHeater);
		}
	}

	// Check chamber heaters
	for (size_t i = 0; i < MaxChamberHeaters; i++)
	{
		const int8_t chamberHeater = reprap.GetHeat().GetChamberHeater(i);
		if (chamberHeater >= 0)
		{
			configuredHeaters.SetBit(chamberHeater);
		}
	}

	// Check tool heaters
	for (size_t heater = 0; heater < MaxHeaters; heater++)
	{
		if (reprap.IsHeaterAssignedToTool(heater))
		{
			configuredHeaters.SetBit(heater);
		}
	}
}

#if HAS_MASS_STORAGE

// Write the platform parameters to file
bool Platform::WritePlatformParameters(FileStore *f, bool includingG31) const noexcept
{
	bool ok;
	if (axisMinimaProbed.IsNonEmpty() || axisMaximaProbed.IsNonEmpty())
	{
		ok = f->Write("; Probed axis limits\n");
		if (ok)
		{
			ok = WriteAxisLimits(f, axisMinimaProbed, axisMinima, 1);
		}
		if (ok)
		{
			ok = WriteAxisLimits(f, axisMaximaProbed, axisMaxima, 0);
		}
	}
	else
	{
		ok = true;
	}

	if (ok && includingG31)
	{
		ok = endstops.WriteZProbeParameters(f, includingG31);
	}

	return ok;
}

bool Platform::WriteAxisLimits(FileStore *f, AxesBitmap axesProbed, const float limits[MaxAxes], int sParam) noexcept
{
	if (axesProbed.IsEmpty())
	{
		return true;
	}

	String<StringLength100> scratchString;
	scratchString.printf("M208 S%d", sParam);
	axesProbed.Iterate([&scratchString, limits](unsigned int axis, unsigned int) noexcept { scratchString.catf(" %c%.2f", reprap.GetGCodes().GetAxisLetters()[axis], (double)limits[axis]); });
	scratchString.cat('\n');
	return f->Write(scratchString.c_str());
}

#endif

#if SUPPORT_CAN_EXPANSION

// Function to identify and iterate through all drivers attached to an axis or extruder
void Platform::IterateDrivers(size_t axisOrExtruder, stdext::inplace_function<void(uint8_t)> localFunc, stdext::inplace_function<void(DriverId)> remoteFunc) noexcept
{
	if (axisOrExtruder < reprap.GetGCodes().GetTotalAxes())
	{
		for (size_t i = 0; i < axisDrivers[axisOrExtruder].numDrivers; ++i)
		{
			const DriverId id = axisDrivers[axisOrExtruder].driverNumbers[i];
			if (id.IsLocal())
			{
				localFunc(id.localDriver);
			}
			else
			{
				remoteFunc(id);
			}
		}
	}
	else if (axisOrExtruder < MaxAxesPlusExtruders)
	{
		const DriverId id = extruderDrivers[LogicalDriveToExtruder(axisOrExtruder)];
		if (id.IsLocal())
		{
			localFunc(id.localDriver);
		}
		else
		{
			remoteFunc(id);
		}
	}
}

#else

// Function to identify and iterate through all drivers attached to an axis or extruder
void Platform::IterateDrivers(size_t axisOrExtruder, stdext::inplace_function<void(uint8_t)> localFunc) noexcept
{
	if (axisOrExtruder < reprap.GetGCodes().GetTotalAxes())
	{
		for (size_t i = 0; i < axisDrivers[axisOrExtruder].numDrivers; ++i)
		{
			const DriverId id = axisDrivers[axisOrExtruder].driverNumbers[i];
			localFunc(id.localDriver);
		}
	}
	else if (axisOrExtruder < MaxAxesPlusExtruders)
	{
		const DriverId id = extruderDrivers[LogicalDriveToExtruder(axisOrExtruder)];
		localFunc(id.localDriver);
	}
}

#endif

// This is called from the step ISR as well as other places, so keep it fast
// If drive >= DRIVES then we are setting an individual motor direction
void Platform::SetDirection(size_t axisOrExtruder, bool direction) noexcept
{
	const bool isSlowDriver = (GetDriversBitmap(axisOrExtruder) & GetSlowDriversBitmap()) != 0;
	if (isSlowDriver)
	{
		while (StepTimer::GetTimerTicks() - DDA::lastStepLowTime < GetSlowDriverDirHoldClocks()) { }
	}

	if (axisOrExtruder < MaxAxesPlusExtruders)
	{
		IterateLocalDrivers(axisOrExtruder, [this, direction](uint8_t driver) { this->SetDriverDirection(driver, direction); });
	}
	else if (axisOrExtruder < MaxAxesPlusExtruders + NumDirectDrivers)
	{
		SetDriverDirection(axisOrExtruder - MaxAxesPlusExtruders, direction);
	}

	if (isSlowDriver)
	{
		DDA::lastDirChangeTime = StepTimer::GetTimerTicks();
	}
}

// Enable a driver. Must not be called from an ISR, or with interrupts disabled.
void Platform::EnableOneLocalDriver(size_t driver, float requiredCurrent) noexcept
{
#if HAS_SMART_DRIVERS && (HAS_VOLTAGE_MONITOR || HAS_12V_MONITOR)
	if (driver < numSmartDrivers && !driversPowered)
	{
		warnDriversNotPowered = true;
	}
	else
	{
#endif
		UpdateMotorCurrent(driver, requiredCurrent);

#if defined(DUET3) && HAS_SMART_DRIVERS
		SmartDrivers::EnableDrive(driver, true);		// all drivers driven directly by the main board are smart
#elif HAS_SMART_DRIVERS
		if (driver < numSmartDrivers)
		{
			SmartDrivers::EnableDrive(driver, true);
		}
# if !defined(DUET3MINI)		// no enable pins on 5LC
		else
		{
			digitalWrite(ENABLE_PINS[driver], enableValues[driver] > 0);
		}
# endif
#else
		digitalWrite(ENABLE_PINS[driver], enableValues[driver] > 0);
#endif
#if HAS_SMART_DRIVERS && (HAS_VOLTAGE_MONITOR || HAS_12V_MONITOR)
	}
#endif
}

// Disable a driver
void Platform::DisableOneLocalDriver(size_t driver) noexcept
{
	if (driver < GetNumActualDirectDrivers())
	{
#if defined(DUET3) && HAS_SMART_DRIVERS
		SmartDrivers::EnableDrive(driver, false);		// all drivers driven directly by the main board are smart
#elif HAS_SMART_DRIVERS
		if (driver < numSmartDrivers)
		{
			SmartDrivers::EnableDrive(driver, false);
		}
# if !defined(DUET3MINI)		// Duet 5LC has no enable pins
		else
		{
			digitalWrite(ENABLE_PINS[driver], enableValues[driver] <= 0);
		}
# endif
#else
		digitalWrite(ENABLE_PINS[driver], enableValues[driver] <= 0);
#endif
	}
}

// Enable the local drivers for a drive. Must not be called from an ISR, or with interrupts disabled.
void Platform::EnableDrivers(size_t axisOrExtruder) noexcept
{
	if (driverState[axisOrExtruder] != DriverStatus::enabled)
	{
		driverState[axisOrExtruder] = DriverStatus::enabled;
		const float requiredCurrent = motorCurrents[axisOrExtruder] * motorCurrentFraction[axisOrExtruder];
#if SUPPORT_CAN_EXPANSION
		CanDriversList canDriversToEnable;
		IterateDrivers(axisOrExtruder,
						[this, requiredCurrent](uint8_t driver) { EnableOneLocalDriver(driver, requiredCurrent); },
						[&canDriversToEnable](DriverId driver) { canDriversToEnable.AddEntry(driver); }
					  );
		CanInterface::EnableRemoteDrivers(canDriversToEnable);
#else
		IterateDrivers(axisOrExtruder,
						[this, requiredCurrent](uint8_t driver) { EnableOneLocalDriver(driver, requiredCurrent); }
					  );
#endif
	}
}

// Disable the drivers for a drive
void Platform::DisableDrivers(size_t axisOrExtruder) noexcept
{
#if SUPPORT_CAN_EXPANSION
	CanDriversList canDriversToDisable;

	IterateDrivers(axisOrExtruder,
					[this](uint8_t driver) { DisableOneLocalDriver(driver); },
					[&canDriversToDisable](DriverId driver) { canDriversToDisable.AddEntry(driver); }
				  );
	CanInterface::DisableRemoteDrivers(canDriversToDisable);
#else
	IterateDrivers(axisOrExtruder,
					[this](uint8_t driver) { DisableOneLocalDriver(driver); }
				  );
#endif
	driverState[axisOrExtruder] = DriverStatus::disabled;
}

// Disable all drives in an emergency. Called from emergency stop and the tick ISR.
// This is only called in an emergency, so we don't update the driver status
void Platform::EmergencyDisableDrivers() noexcept
{
	for (size_t drive = 0; drive < GetNumActualDirectDrivers(); drive++)
	{
		if (!inInterrupt())		// on the Duet 06/085 we need interrupts running to send the I2C commands to set motor currents
		{
			UpdateMotorCurrent(drive, 0.0);
		}
		DisableOneLocalDriver(drive);
	}
}

void Platform::DisableAllDrivers() noexcept
{
	for (size_t axisOrExtruder = 0; axisOrExtruder < MaxAxesPlusExtruders; axisOrExtruder++)
	{
		DisableDrivers(axisOrExtruder);
	}
}

// Set drives to idle hold if they are enabled. If a drive is disabled, leave it alone.
// Must not be called from an ISR, or with interrupts disabled.
void Platform::SetDriversIdle() noexcept
{
	if (idleCurrentFactor == 0)
	{
		DisableAllDrivers();
		reprap.GetGCodes().SetAllAxesNotHomed();
	}
	else
	{
#if SUPPORT_CAN_EXPANSION
		CanDriversList canDriversToSetIdle;
#endif
		for (size_t axisOrExtruder = 0; axisOrExtruder < MaxAxesPlusExtruders; ++axisOrExtruder)
		{
			if (driverState[axisOrExtruder] == DriverStatus::enabled)
			{
				driverState[axisOrExtruder] = DriverStatus::idle;
				const float current = motorCurrents[axisOrExtruder] * idleCurrentFactor;
				IterateDrivers(axisOrExtruder,
								[this, current](uint8_t driver) { UpdateMotorCurrent(driver, current); }
#if SUPPORT_CAN_EXPANSION
								, [&canDriversToSetIdle](DriverId driver) { canDriversToSetIdle.AddEntry(driver); }
#endif
							  );
			}
		}
#if SUPPORT_CAN_EXPANSION
		CanInterface::SetRemoteDriversIdle(canDriversToSetIdle, idleCurrentFactor);
#endif
	}
}

// Set the current for all drivers on an axis or extruder. Current is in mA.
GCodeResult Platform::SetMotorCurrent(size_t axisOrExtruder, float currentOrPercent, int code, const StringRef& reply) noexcept
{
	switch (code)
	{
	case 906:
		motorCurrents[axisOrExtruder] = currentOrPercent;
		break;

	case 913:
		motorCurrentFraction[axisOrExtruder] = constrain<float>(0.01 * currentOrPercent, 0.0, 1.0);
		break;

#if HAS_SMART_DRIVERS
	case 917:
		standstillCurrentPercent[axisOrExtruder] = constrain<float>(currentOrPercent, 0.0, 100.0);
		break;
#endif

	default:
		return GCodeResult::error;
	}

#if SUPPORT_CAN_EXPANSION
	CanDriversData<float> canDriversToUpdate;

	IterateDrivers(axisOrExtruder,
							[this, axisOrExtruder, code](uint8_t driver)
							{
								if (code == 917)
								{
# if HAS_SMART_DRIVERS
									SmartDrivers::SetStandstillCurrentPercent(driver, standstillCurrentPercent[axisOrExtruder]);
# endif
								}
								else
								{
									UpdateMotorCurrent(driver, motorCurrents[axisOrExtruder] * motorCurrentFraction[axisOrExtruder]);
								}
							},
							[this, axisOrExtruder, code, &canDriversToUpdate](DriverId driver)
							{
								if (code == 917)
								{
									canDriversToUpdate.AddEntry(driver, standstillCurrentPercent[axisOrExtruder]);
								}
								else
								{
									canDriversToUpdate.AddEntry(driver, motorCurrents[axisOrExtruder] * motorCurrentFraction[axisOrExtruder]);
								}
							}
						);
	if (code == 917)
	{
		return CanInterface::SetRemoteStandstillCurrentPercent(canDriversToUpdate, reply);
	}
	else
	{
		return CanInterface::SetRemoteDriverCurrents(canDriversToUpdate, reply);
	}
#else
	IterateDrivers(axisOrExtruder,
							[this, axisOrExtruder, code](uint8_t driver)
							{
								if (code == 917)
								{
# if HAS_SMART_DRIVERS
									SmartDrivers::SetStandstillCurrentPercent(driver, standstillCurrentPercent[axisOrExtruder]);
# endif
								}
								else
								{
									UpdateMotorCurrent(driver, motorCurrents[axisOrExtruder] * motorCurrentFraction[axisOrExtruder]);
								}
							}
	);
	return GCodeResult::ok;
#endif
}

// This must not be called from an ISR, or with interrupts disabled.
void Platform::UpdateMotorCurrent(size_t driver, float current) noexcept
{
	if (driver < GetNumActualDirectDrivers())
	{
#if HAS_SMART_DRIVERS
		if (driver < numSmartDrivers)
		{
			SmartDrivers::SetCurrent(driver, current);
		}
#elif defined (DUET_06_085)
		const uint16_t pot = (unsigned short)((0.256*current*8.0*senseResistor + maxStepperDigipotVoltage/2)/maxStepperDigipotVoltage);
		if (driver < 4)
		{
			mcpDuet.setNonVolatileWiper(potWipes[driver], pot);
			mcpDuet.setVolatileWiper(potWipes[driver], pot);
		}
		else
		{
			if (board == BoardType::Duet_085)
			{
				// Extruder 0 is on DAC channel 0
				if (driver == 4)
				{
					const float dacVoltage = max<float>(current * 0.008 * senseResistor + stepperDacVoltageOffset, 0.0);	// the voltage we want from the DAC relative to its minimum
					const float dac = dacVoltage/stepperDacVoltageRange;
					AnalogOut(DAC0, dac);
				}
				else
				{
					mcpExpansion.setNonVolatileWiper(potWipes[driver-1], pot);
					mcpExpansion.setVolatileWiper(potWipes[driver-1], pot);
				}
			}
			else if (driver < 8)		// on a Duet 0.6 we have a maximum of 8 drives
			{
				mcpExpansion.setNonVolatileWiper(potWipes[driver], pot);
				mcpExpansion.setVolatileWiper(potWipes[driver], pot);
			}
		}
#elif defined(__ALLIGATOR__)
		// Alligator SPI DAC current
		if (driver < 4)  // Onboard DAC
		{
			dacAlligator.setChannel(3-driver, current * 0.102);
		}
		else // Piggy module DAC
		{
			dacPiggy.setChannel(7-driver, current * 0.102);
		}
#elif defined(__LPC17xx__)
		if (hasDriverCurrentControl)
		{
			//Has digipots to set current control for drivers
			//Current is in mA
			const uint16_t pot = (unsigned short) (current * digipotFactor / 1000);
			if (driver < 4)
			{
				mcp4451.setMCP4461Address(0x2C); //A0 and A1 Grounded. (001011 00)
				mcp4451.setVolatileWiper(POT_WIPES[driver], pot);
			}
			else
				mcp4451.setMCP4461Address(0x2D); //A0 Vcc, A1 Grounded. (001011 01)
				mcp4451.setVolatileWiper(POT_WIPES[driver-4], pot);
			}
		}
#else
		// otherwise we can't set the motor current
#endif
	}
}

// Get the configured motor current for an axis or extruder
float Platform::GetMotorCurrent(size_t drive, int code) const noexcept
{
	switch (code)
	{
	case 906:
		return motorCurrents[drive];

	case 913:
		return motorCurrentFraction[drive] * 100.0;

#if HAS_SMART_DRIVERS
	case 917:
		return standstillCurrentPercent[drive];
#endif
	default:
		return 0.0;
	}
}

// Set the motor idle current factor
void Platform::SetIdleCurrentFactor(float f) noexcept
{
	idleCurrentFactor = constrain<float>(f, 0.0, 1.0);
	reprap.MoveUpdated();

#if SUPPORT_CAN_EXPANSION
	CanDriversData<float> canDriversToUpdate;
#endif
	for (size_t axisOrExtruder = 0; axisOrExtruder < MaxAxesPlusExtruders; ++axisOrExtruder)
	{
		if (driverState[axisOrExtruder] == DriverStatus::idle)
		{
			const float requiredCurrent = motorCurrents[axisOrExtruder] * idleCurrentFactor;
			IterateDrivers(axisOrExtruder,
							[this, requiredCurrent](uint8_t driver){ UpdateMotorCurrent(driver, requiredCurrent); }
#if SUPPORT_CAN_EXPANSION
								, [this, requiredCurrent, &canDriversToUpdate](DriverId driver) { canDriversToUpdate.AddEntry(driver, (uint16_t)requiredCurrent); }
#endif
						  );
		}
	}
#if SUPPORT_CAN_EXPANSION
	String<1> dummy;
	(void)CanInterface::SetRemoteDriverCurrents(canDriversToUpdate, dummy.GetRef());
#endif
}

void Platform::SetDriveStepsPerUnit(size_t axisOrExtruder, float value, uint32_t requestedMicrostepping) noexcept
{
	if (requestedMicrostepping != 0)
	{
		const uint32_t currentMicrostepping = microstepping[axisOrExtruder] & 0x7FFF;
		if (currentMicrostepping != requestedMicrostepping)
		{
			value = value * (float)currentMicrostepping / (float)requestedMicrostepping;
		}
	}
	driveStepsPerUnit[axisOrExtruder] = max<float>(value, 1.0);	// don't allow zero or negative
	reprap.MoveUpdated();
}

// Set the microstepping for a driver, returning true if successful
bool Platform::SetDriverMicrostepping(size_t driver, unsigned int microsteps, int mode) noexcept
{
	if (driver < GetNumActualDirectDrivers())
	{
#if HAS_SMART_DRIVERS
		if (driver < numSmartDrivers)
		{
			return SmartDrivers::SetMicrostepping(driver, microsteps, mode);
		}
		else
		{
			// Other drivers only support x16 microstepping.
			// We ignore the interpolation on/off parameter so that e.g. M350 I1 E16:128 won't give an error if E1 supports interpolation but E0 doesn't.
			return microsteps == 16;
		}
#elif defined(__ALLIGATOR__)
		return Microstepping::Set(driver, microsteps); // no mode in Alligator board
#elif defined(__LPC17xx__)
		return Microstepping::Set(driver, microsteps);
#else
		// Assume only x16 microstepping supported
		return microsteps == 16;
#endif
	}
	return false;
}

// Set the microstepping for local drivers, returning true if successful. All drivers for the same axis must use the same microstepping.
// Caller must deal with remote drivers.
bool Platform::SetMicrostepping(size_t axisOrExtruder, int microsteps, bool interp, const StringRef& reply) noexcept
{
	//TODO check that it is a valid microstep setting
	microstepping[axisOrExtruder] = (interp) ? microsteps | 0x8000 : microsteps;
	reprap.MoveUpdated();
	bool ok = true;
	IterateLocalDrivers(axisOrExtruder,
					[this, microsteps, interp, &ok, reply](uint8_t driver) noexcept
					{
						if (!SetDriverMicrostepping(driver, microsteps, interp))
						{
							reply.lcatf("Driver %u does not support x%u microstepping", driver, microsteps);
							if (interp)
							{
								reply.cat(" with interpolation");
							}
							ok = false;
						}
					}
				  );
	return ok;
}

// Get the microstepping for an axis or extruder
unsigned int Platform::GetMicrostepping(size_t axisOrExtruder, bool& interpolation) const noexcept
{
	interpolation = (microstepping[axisOrExtruder] & 0x8000) != 0;
	return microstepping[axisOrExtruder] & 0x7FFF;
}

void Platform::SetEnableValue(size_t driver, int8_t eVal) noexcept
{
	if (driver < GetNumActualDirectDrivers())
	{
		enableValues[driver] = eVal;
		DisableOneLocalDriver(driver);				// disable the drive, because the enable polarity may have been wrong before
#if HAS_SMART_DRIVERS
		if (eVal == -1)
		{
			// User has asked to disable status monitoring for this driver, so clear its error bits
			const DriversBitmap mask = ~DriversBitmap::MakeFromBits(driver);
			temperatureShutdownDrivers &= mask;
			temperatureWarningDrivers &= mask;
			shortToGroundDrivers &= mask;
			openLoadADrivers &= mask;
			openLoadBDrivers &= mask;
		}
#endif
	}
}

void Platform::SetAxisDriversConfig(size_t axis, size_t numValues, const DriverId driverNumbers[]) noexcept
{
	AxisDriversConfig& cfg = axisDrivers[axis];
	cfg.numDrivers = numValues;
	uint32_t bitmap = 0;
	for (size_t i = 0; i < numValues; ++i)
	{
		const DriverId id = driverNumbers[i];
		cfg.driverNumbers[i] = id;
		if (id.IsLocal())
		{
			bitmap |= StepPins::CalcDriverBitmap(id.localDriver);
#if HAS_SMART_DRIVERS
			SmartDrivers::SetAxisNumber(id.localDriver, axis);
#endif
		}
	}
	driveDriverBits[axis] = bitmap;
}

// Set the characteristics of an axis
void Platform::SetAxisType(size_t axis, AxisWrapType wrapType, bool isNistRotational) noexcept
{
	if (isNistRotational)
	{
		rotationalAxes.SetBit(axis);
	}
	else
	{
		linearAxes.SetBit(axis);
	}

	switch (wrapType)
	{
#if 0	// shortcut axes not implemented yet
	case AxisWrapType::wrapWithShortcut:
		shortcutAxes.SetBit(axis);
		// no break
#endif
	case AxisWrapType::wrapAt360:
		continuousAxes.SetBit(axis);
		break;

	default:
		break;
	}
}

// Map an extruder to a driver
void Platform::SetExtruderDriver(size_t extruder, DriverId driver) noexcept
{
	extruderDrivers[extruder] = driver;
	if (driver.IsLocal())
	{
#if HAS_SMART_DRIVERS
		SmartDrivers::SetAxisNumber(driver.localDriver, ExtruderToLogicalDrive(extruder));
#endif
		driveDriverBits[ExtruderToLogicalDrive(extruder)] = StepPins::CalcDriverBitmap(driver.localDriver);
	}
	else
	{
		driveDriverBits[ExtruderToLogicalDrive(extruder)] = 0;
	}
}

void Platform::SetDriverStepTiming(size_t driver, const float microseconds[4]) noexcept
{
	const uint32_t bitmap = StepPins::CalcDriverBitmap(driver);
	slowDriversBitmap &= ~bitmap;								// start by assuming this drive does not need extended timing
	if (slowDriversBitmap == 0)
	{
		for (uint32_t& entry : slowDriverStepTimingClocks)
		{
			entry = 0;											// reset all to zero if we have no known slow drivers
		}
	}

	for (size_t i = 0; i < ARRAY_SIZE(slowDriverStepTimingClocks); ++i)
	{
		if (microseconds[i] > MinStepPulseTiming)
		{
			slowDriversBitmap |= StepPins::CalcDriverBitmap(driver);		// this drive does need extended timing
			const uint32_t clocks = (uint32_t)(((float)StepTimer::StepClockRate * microseconds[i] * 0.000001) + 0.99);	// convert microseconds to step clocks, rounding up
			if (clocks > slowDriverStepTimingClocks[i])
			{
				slowDriverStepTimingClocks[i] = clocks;
			}
		}
	}
}

// Get the driver step timing, returning true if we are using slower timing than standard
bool Platform::GetDriverStepTiming(size_t driver, float microseconds[4]) const noexcept
{
	const bool isSlowDriver = ((slowDriversBitmap & StepPins::CalcDriverBitmap(driver)) != 0);
	for (size_t i = 0; i < 4; ++i)
	{
		microseconds[i] = (isSlowDriver)
							? (float)slowDriverStepTimingClocks[i] * 1000000.0/(float)StepTimer::StepClockRate
								: 0.0;
	}
	return isSlowDriver;
}

//-----------------------------------------------------------------------------------------------------

// USB port functions

void Platform::AppendUsbReply(OutputBuffer *buffer) noexcept
{
	if (   !SERIAL_MAIN_DEVICE.IsConnected()
#if SUPPORT_SCANNER
		|| (reprap.GetScanner().IsRegistered() && !reprap.GetScanner().DoingGCodes())
#endif
	   )
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

bool Platform::IsAuxEnabled(size_t auxNumber) const noexcept
{
#if HAS_AUX_DEVICES
	return auxNumber < ARRAY_SIZE(auxDevices) && auxDevices[auxNumber].IsEnabled();
#else
	return false;
#endif
}

void Platform::EnableAux(size_t auxNumber) noexcept
{
#if HAS_AUX_DEVICES
	if (auxNumber < ARRAY_SIZE(auxDevices) && !auxDevices[auxNumber].IsEnabled())
	{
		auxDevices[auxNumber].Enable(baudRates[auxNumber + 1]);
	}
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

void Platform::SetAuxRaw(size_t auxNumber, bool raw) noexcept
{
#if HAS_AUX_DEVICES
	if (auxNumber < ARRAY_SIZE(auxDevices))
	{
		auxDevices[auxNumber].SetRaw(raw);
	}
#endif
}

#if HAS_AUX_DEVICES
void Platform::InitPanelDueUpdater() noexcept
{
	if (panelDueUpdater == nullptr)
	{
		panelDueUpdater = new PanelDueUpdater();
	}
}
#endif

void Platform::AppendAuxReply(size_t auxNumber, const char *msg, bool rawMessage) noexcept
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
void Platform::RawMessage(MessageType type, const char *message) noexcept
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
		const char *p = message;
		size_t len = strlen(p);
		while (SERIAL_MAIN_DEVICE.IsConnected() && len != 0 && !reprap.SpinTimeoutImminent())
		{
			const size_t written = SERIAL_MAIN_DEVICE.write(p, len);
			len -= written;
			p += written;
		}
		// We no longer flush afterwards
	}
	else if ((type & UsbMessage) != 0)
	{
		// Message that is to be sent via the USB line (non-blocking)
		MutexLocker lock(usbMutex);
#if SUPPORT_SCANNER
		if (!reprap.GetScanner().IsRegistered() || reprap.GetScanner().DoingGCodes())
#endif
		{
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
	size_t numDestinations = 0;
	if ((type & (AuxMessage | ImmediateAuxMessage)) != 0)
	{
		++numDestinations;
	}
	if ((type & (UsbMessage | BlockingUsbMessage)) != 0)
	{
		++numDestinations;
	}
	if ((type & HttpMessage) != 0)
	{
		++numDestinations;
	}
	if ((type & TelnetMessage) != 0)
	{
		++numDestinations;
	}
#if HAS_LINUX_INTERFACE
	if (reprap.UsingLinuxInterface() && ((type & GenericMessage) == GenericMessage || (type & BinaryCodeReplyFlag) != 0))
	{
		++numDestinations;
	}
#endif
#ifdef SERIAL_AUX2_DEVICE
	if ((type & Aux2Message) != 0)
	{
		++numDestinations;
	}
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

		if ((type & HttpMessage) != 0)
		{
			reprap.GetNetwork().HandleHttpGCodeReply(buffer);
		}

		if ((type & TelnetMessage) != 0)
		{
			reprap.GetNetwork().HandleTelnetGCodeReply(buffer);
		}

		if ((type & Aux2Message) != 0)
		{
			AppendAuxReply(1, buffer, ((*buffer)[0] == '{') || (type & RawMessageFlag) != 0);
		}

		if ((type & (UsbMessage | BlockingUsbMessage)) != 0)
		{
			AppendUsbReply(buffer);
		}

#if HAS_LINUX_INTERFACE
		if (reprap.UsingLinuxInterface() && ((type & GenericMessage) == GenericMessage || (type & BinaryCodeReplyFlag) != 0))
		{
			reprap.GetLinuxInterface().HandleGCodeReply(type, buffer);
		}
#endif
	}
}

void Platform::MessageF(MessageType type, const char *fmt, va_list vargs) noexcept
{
	String<FormatStringLength> formatString;
#if HAS_LINUX_INTERFACE
	if (reprap.UsingLinuxInterface() && ((type & GenericMessage) == GenericMessage || (type & BinaryCodeReplyFlag) != 0))
	{
		formatString.vprintf(fmt, vargs);
		reprap.GetLinuxInterface().HandleGCodeReply(type, formatString.c_str());
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

void Platform::MessageF(MessageType type, const char *fmt, ...) noexcept
{
	va_list vargs;
	va_start(vargs, fmt);
	MessageF(type, fmt, vargs);
	va_end(vargs);
}

void Platform::Message(MessageType type, const char *message) noexcept
{
#if HAS_LINUX_INTERFACE
	if (reprap.UsingLinuxInterface() &&
		((type & BinaryCodeReplyFlag) != 0 || (type & GenericMessage) == GenericMessage || (type & LogOff) != LogOff))
	{
		reprap.GetLinuxInterface().HandleGCodeReply(type, message);
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
void Platform::DebugMessage(const char *fmt, va_list vargs) noexcept
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

// Send a message box, which may require an acknowledgement
// sParam = 0 Just display the message box, optional timeout
// sParam = 1 As for 0 but display a Close button as well
// sParam = 2 Display the message box with an OK button, wait for acknowledgement (waiting is set up by the caller)
// sParam = 3 As for 2 but also display a Cancel button
void Platform::SendAlert(MessageType mt, const char *message, const char *title, int sParam, float tParam, AxesBitmap controls) noexcept
{
	if ((mt & (HttpMessage | AuxMessage | LcdMessage | BinaryCodeReplyFlag)) != 0)
	{
		reprap.SetAlert(message, title, sParam, tParam, controls);		// make the RepRap class cache this message until it's picked up by the HTTP clients and/or PanelDue
	}

	MessageF(MessageType::LogInfo, "M291: - %s - %s", (strlen(title) > 0 ? title : "[no title]"), message);

	mt = (MessageType)(mt & (UsbMessage | TelnetMessage));
	if (mt != 0)
	{
		if (strlen(title) > 0)
		{
			MessageF(mt, "- %s -\n", title);
		}
		MessageF(mt, "%s\n", message);
		if (sParam == 2)
		{
			Message(mt, "Send M292 to continue\n");
		}
		else if (sParam == 3)
		{
			Message(mt, "Send M292 to continue or M292 P1 to cancel\n");
		}
	}
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
			logger->Start(realTime, filename);
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
const char *Platform::GetLogFileName() const noexcept
{
	return (logger == nullptr) ? nullptr : logger->GetFileName();
}

#endif

const char *Platform::GetLogLevel() const noexcept
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

bool Platform::AtxPower() const noexcept
{
#ifdef __LPC17xx__
	const bool val = IoPort::ReadPin(ATX_POWER_PIN);
	return (ATX_POWER_INVERTED) ? !val : val;
#else
    return IoPort::ReadPin(ATX_POWER_PIN);
#endif
}

void Platform::AtxPowerOn() noexcept
{
	deferredPowerDown = false;
	IoPort::WriteDigital(ATX_POWER_PIN, true);
}

void Platform::AtxPowerOff(bool defer) noexcept
{
	deferredPowerDown = defer;
	if (!defer)
	{
#if HAS_MASS_STORAGE
		if (logger != nullptr)
		{
			logger->LogMessage(realTime, "Power off commanded", LogWarn);
			logger->Flush(true);
			// We don't call logger->Stop() here because we don't know whether turning off the power will work
		}
#endif
#ifdef __LPC17xx__
		IoPort::WriteDigital(ATX_POWER_PIN, ATX_POWER_INVERTED);
#else
		IoPort::WriteDigital(ATX_POWER_PIN, false);
#endif
	}
}

GCodeResult Platform::SetPressureAdvance(float advance, GCodeBuffer& gb, const StringRef& reply)
{
	GCodeResult rslt = GCodeResult::ok;

#if SUPPORT_CAN_EXPANSION
	CanDriversData<float> canDriversToUpdate;
#endif

	if (gb.Seen('D'))
	{
		uint32_t eDrive[MaxExtruders];
		size_t eCount = MaxExtruders;
		gb.GetUnsignedArray(eDrive, eCount, false);
		for (size_t i = 0; i < eCount; i++)
		{
			const uint32_t extruder = eDrive[i];
			if (extruder >= reprap.GetGCodes().GetNumExtruders())
			{
				reply.printf("Invalid extruder number '%" PRIu32 "'", extruder);
				rslt = GCodeResult::error;
				break;
			}
			pressureAdvance[extruder] = advance;
#if SUPPORT_CAN_EXPANSION
			if (extruderDrivers[extruder].IsRemote())
			{
				canDriversToUpdate.AddEntry(extruderDrivers[extruder], advance);
			}
#endif
		}
	}
	else
	{
		const Tool * const ct = reprap.GetCurrentTool();
		if (ct == nullptr)
		{
			reply.copy("No tool selected");
			rslt = GCodeResult::error;
		}
		else
		{
#if SUPPORT_CAN_EXPANSION
			ct->IterateExtruders([this, advance, &canDriversToUpdate](unsigned int extruder)
									{
										pressureAdvance[extruder] = advance;
										if (extruderDrivers[extruder].IsRemote())
										{
											canDriversToUpdate.AddEntry(extruderDrivers[extruder], advance);
										}
									}
								);
#else
			ct->IterateExtruders([this, advance](unsigned int extruder)
									{
										pressureAdvance[extruder] = advance;
									}
								);
#endif
		}
	}

#if SUPPORT_CAN_EXPANSION
	return max(rslt, CanInterface::SetRemotePressureAdvance(canDriversToUpdate, reply));
#else
	return rslt;
#endif
}

#if SUPPORT_NONLINEAR_EXTRUSION

bool Platform::GetExtrusionCoefficients(size_t extruder, float& a, float& b, float& limit) const noexcept
{
	if (extruder < MaxExtruders)
	{
		a = nonlinearExtrusionA[extruder];
		b = nonlinearExtrusionB[extruder];
		limit = nonlinearExtrusionLimit[extruder];
		return true;
	}
	return false;
}

void Platform::SetNonlinearExtrusion(size_t extruder, float a, float b, float limit) noexcept
{
	if (extruder < MaxExtruders && nonlinearExtrusionLimit[extruder] > 0.0)
	{
		nonlinearExtrusionLimit[extruder] = limit;
		nonlinearExtrusionA[extruder] = a;
		nonlinearExtrusionB[extruder] = b;
	}
}

#endif

void Platform::SetBaudRate(size_t chan, uint32_t br) noexcept
{
	if (chan < NumSerialChannels)
	{
		baudRates[chan] = br;
	}
}

uint32_t Platform::GetBaudRate(size_t chan) const noexcept
{
	return (chan < NumSerialChannels) ? baudRates[chan] : 0;
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
#if SAME5x
        SERIAL_MAIN_DEVICE.Start();
#elif defined(__LPC17xx__)
		SERIAL_MAIN_DEVICE.begin(baudRates[0]);
#else
        SERIAL_MAIN_DEVICE.Start(UsbVBusPin);
#endif
	}
#if HAS_AUX_DEVICES
	else if (chan < NumSerialChannels)
	{
		auxDevices[chan - 1].Disable();
		auxDevices[chan - 1].Enable(baudRates[chan]);
	}
#endif
}

// Set the board type. This must be called quite early, because for some builds it relies on pins not having been programmed for their intended use yet.
void Platform::SetBoardType(BoardType bt) noexcept
{
	if (bt == BoardType::Auto)
	{
#if defined(DUET3MINI_V02)
		// Test whether this is a WiFi or an Ethernet board. Currently we do this based on the processor type.
		const uint16_t deviceId = DSU->DID.reg >> 16;
		board = (deviceId == 0x6184)							// if SAME54P20A
				? BoardType::Duet3Mini_Ethernet
				: (deviceId == 0x6006)							// SAMD51P20A rev D
				  ? BoardType::Duet3Mini_WiFi
					: BoardType::Duet3Mini_Unknown;
#elif defined(DUET3MINI_V04)
		// Test whether this is a WiFi or an Ethernet board by testing for a pulldown resistor on Dir1
		pinMode(DIRECTION_PINS[1], INPUT_PULLUP);
		delayMicroseconds(20);									// give the pullup resistor time to work
		board = (digitalRead(DIRECTION_PINS[1]))				// if SAME54P20A
					? BoardType::Duet3Mini_WiFi
						: BoardType::Duet3Mini_Ethernet;
#elif defined(DUET3)
		// Driver 0 direction has a pulldown resistor on v0.6 and v1.0 boards, but won't on v1.01 boards
		pinMode(DIRECTION_PINS[0], INPUT_PULLUP);
		delayMicroseconds(20);									// give the pullup resistor time to work
		board = (digitalRead(DIRECTION_PINS[0])) ? BoardType::Duet3_v101 : BoardType::Duet3_v06_100;
#elif defined(SAME70XPLD)
		board = BoardType::SAME70XPLD_0;
#elif defined(DUET_NG)
		// Get ready to test whether the Ethernet module is present, so that we avoid additional delays
		pinMode(EspResetPin, OUTPUT_LOW);						// reset the WiFi module or the W5500. We assume that this forces the ESP8266 UART output pin to high impedance.
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
#elif defined(DUET_06_085)
		// Determine whether this is a Duet 0.6 or a Duet 0.8.5 board.
		// If it is a 0.85 board then DAC0 (AKA digital pin 67) is connected to ground via a diode and a 2.15K resistor.
		// So we enable the pullup (value 100K-150K) on pin 67 and read it, expecting a LOW on a 0.8.5 board and a HIGH on a 0.6 board.
		// This may fail if anyone connects a load to the DAC0 pin on a Duet 0.6, hence we implement board selection in M115 as well.
		pinMode(Dac0DigitalPin, INPUT_PULLUP);
		delayMicroseconds(10);
		board = (digitalRead(Dac0DigitalPin)) ? BoardType::Duet_06 : BoardType::Duet_085;
		pinMode(Dac0DigitalPin, INPUT);			// turn pullup off
#elif defined(__RADDS__)
		board = BoardType::RADDS_15;
#elif defined(__ALLIGATOR__)
		board = BoardType::Alligator_2;
#elif defined(PCCB_10)
		board = BoardType::PCCB_v10;
#elif defined(PCCB_08) || defined(PCCB_08_X5)
		board = BoardType::PCCB_v08;
#elif defined(__LPC17xx__)
		board = BoardType::Lpc;
#else
# error Undefined board type
#endif
	}
	else
	{
		board = bt;
	}
}

// Get a string describing the electronics
const char* Platform::GetElectronicsString() const noexcept
{
	switch (board)
	{
#if defined(DUET3MINI)
	case BoardType::Duet3Mini_Unknown:		return "Duet 3 " BOARD_SHORT_NAME " unknown variant";
	case BoardType::Duet3Mini_WiFi:			return "Duet 3 " BOARD_SHORT_NAME " WiFi";
	case BoardType::Duet3Mini_Ethernet:		return "Duet 3 " BOARD_SHORT_NAME " Ethernet";
#elif defined(DUET3)
	case BoardType::Duet3_v06_100:			return "Duet 3 " BOARD_SHORT_NAME " v0.6 or 1.0";
	case BoardType::Duet3_v101:				return "Duet 3 " BOARD_SHORT_NAME " v1.01 or later";
#elif defined(SAME70XPLD)
	case BoardType::SAME70XPLD_0:			return "SAME70-XPLD";
#elif defined(DUET_NG)
	case BoardType::DuetWiFi_10:			return "Duet WiFi 1.0 or 1.01";
	case BoardType::DuetWiFi_102:			return "Duet WiFi 1.02 or later";
	case BoardType::DuetEthernet_10:		return "Duet Ethernet 1.0 or 1.01";
	case BoardType::DuetEthernet_102:		return "Duet Ethernet 1.02 or later";
	case BoardType::Duet2SBC_10:			return "Duet 2 1.0 or 1.01 + SBC";
	case BoardType::Duet2SBC_102:			return "Duet 2 1.02 or later + SBC";
#elif defined(DUET_M)
	case BoardType::DuetM_10:				return "Duet Maestro 1.0";
#elif defined(DUET_06_085)
	case BoardType::Duet_06:				return "Duet 0.6";
	case BoardType::Duet_07:				return "Duet 0.7";
	case BoardType::Duet_085:				return "Duet 0.85";
#elif defined(__RADDS__)
	case BoardType::RADDS_15:				return "RADDS 1.5";
#elif defined(__ALLIGATOR__)
	case BoardType::Alligator_2:			return "Alligator r2";
#elif defined(PCCB_10)
	case BoardType::PCCB_v10:				return "PC001373";
#elif defined(PCCB_08) || defined(PCCB_08_X5)
	case BoardType::PCCB_v08:				return "PCCB 0.8";
#elif defined(__LPC17xx__)
	case BoardType::Lpc:					return LPC_ELECTRONICS_STRING;
#else
# error Undefined board type
#endif
	default:								return "Unidentified";
	}
}

// Get the board string
const char* Platform::GetBoardString() const noexcept
{
	switch (board)
	{
#if defined(DUET3MINI)
	case BoardType::Duet3Mini_Unknown:		return "duet5lcunknown";
	case BoardType::Duet3Mini_WiFi:			return "duet5lcwifi";
	case BoardType::Duet3Mini_Ethernet:		return "duet5lcethernet";
#elif defined(DUET3)
	case BoardType::Duet3_v06_100:			return "duet3mb6hc100";
	case BoardType::Duet3_v101:				return "duet3mb6hc101";
#elif defined(SAME70XPLD)
	case BoardType::SAME70XPLD_0:			return "same70xpld";
#elif defined(DUET_NG)
	case BoardType::DuetWiFi_10:			return "duetwifi10";
	case BoardType::DuetWiFi_102:			return "duetwifi102";
	case BoardType::DuetEthernet_10:		return "duetethernet10";
	case BoardType::DuetEthernet_102:		return "duetethernet102";
	case BoardType::Duet2SBC_10:			return "duet2sbc10";
	case BoardType::Duet2SBC_102:			return "duet2sbc102";
#elif defined(DUET_M)
	case BoardType::DuetM_10:				return "duetmaestro100";
#elif defined(DUET_06_085)
	case BoardType::Duet_06:				return "duet06";
	case BoardType::Duet_07:				return "duet07";
	case BoardType::Duet_085:				return "duet085";
#elif defined(__RADDS__)
	case BoardType::RADDS_15:				return "radds15";
#elif defined(__ALLIGATOR__)
	case BoardType::Alligator_2:			return "alligator2";
#elif defined(PCCB_10)
	case BoardType::PCCB_v10:				return "pc001373";
#elif defined(PCCB_08) || defined(PCCB_08_X5)
	case BoardType::PCCB_v08:				return "pccb08";
#elif defined(__LPC17xx__)
	case BoardType::Lpc:					return LPC_BOARD_STRING;
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

const char *Platform::GetBoardName() const noexcept
{
	return (board == BoardType::Duet2SBC_10 || board == BoardType::Duet2SBC_102)
			? BOARD_NAME_SBC
			: (IsDuetWiFi()) ? BOARD_NAME_WIFI : BOARD_NAME_ETHERNET;
}

const char *Platform::GetBoardShortName() const noexcept
{
	return (board == BoardType::Duet2SBC_10 || board == BoardType::Duet2SBC_102)
			? BOARD_SHORT_NAME_SBC
			: (IsDuetWiFi()) ? BOARD_SHORT_NAME_WIFI : BOARD_SHORT_NAME_ETHERNET;
}

#endif

#ifdef DUET3MINI

// Return true if this is a Duet WiFi, false if it is a Duet Ethernet
bool Platform::IsDuetWiFi() const noexcept
{
	return board == BoardType::Duet3Mini_WiFi || board == BoardType::Duet3Mini_Unknown;
}

#endif

#if HAS_MASS_STORAGE || HAS_LINUX_INTERFACE

// Open a file
FileStore* Platform::OpenFile(const char* folder, const char* fileName, OpenMode mode, uint32_t preAllocSize) const noexcept
{
	String<MaxFilenameLength> location;
	return (MassStorage::CombineName(location.GetRef(), folder, fileName))
			? MassStorage::OpenFile(location.c_str(), mode, preAllocSize)
				: nullptr;
}

bool Platform::FileExists(const char* folder, const char *filename) const noexcept
{
	String<MaxFilenameLength> location;
	return MassStorage::CombineName(location.GetRef(), folder, filename) && MassStorage::FileExists(location.c_str());
}

#endif

#if HAS_MASS_STORAGE

// Return a pointer to a string holding the directory where the system files are. Lock the sysdir lock before calling this.
const char* Platform::InternalGetSysDir() const noexcept
{
	return (sysDir != nullptr) ? sysDir : DEFAULT_SYS_DIR;
}

bool Platform::Delete(const char* folder, const char *filename) const noexcept
{
	String<MaxFilenameLength> location;
	return MassStorage::CombineName(location.GetRef(), folder, filename) && MassStorage::Delete(location.c_str(), true);
}

bool Platform::DirectoryExists(const char *folder, const char *dir) const noexcept
{
	String<MaxFilenameLength> location;
	return MassStorage::CombineName(location.GetRef(), folder, dir) && MassStorage::DirectoryExists(location.c_str());
}

// Set the system files path
GCodeResult Platform::SetSysDir(const char* dir, const StringRef& reply) noexcept
{
	String<MaxFilenameLength> newSysDir;
	WriteLocker lock(sysDirLock);

	if (!MassStorage::CombineName(newSysDir.GetRef(), InternalGetSysDir(), dir) || (!newSysDir.EndsWith('/') && newSysDir.cat('/')))
	{
		reply.copy("Path name too long");
		return GCodeResult::error;
	}

	const size_t len = newSysDir.strlen() + 1;
	char* const nsd = new char[len];
	memcpy(nsd, newSysDir.c_str(), len);
	const char *nsd2 = nsd;
	std::swap(sysDir, nsd2);
	delete nsd2;
	reprap.DirectoriesUpdated();
	return GCodeResult::ok;
}

bool Platform::SysFileExists(const char *filename) const noexcept
{
	String<MaxFilenameLength> location;
	return MakeSysFileName(location.GetRef(), filename) && MassStorage::FileExists(location.c_str());
}

FileStore* Platform::OpenSysFile(const char *filename, OpenMode mode) const noexcept
{
	String<MaxFilenameLength> location;
	return (MakeSysFileName(location.GetRef(), filename))
			? MassStorage::OpenFile(location.c_str(), mode, 0)
				: nullptr;
}

bool Platform::DeleteSysFile(const char *filename) const noexcept
{
	String<MaxFilenameLength> location;
	return MakeSysFileName(location.GetRef(), filename) && MassStorage::Delete(location.c_str(), true);
}

bool Platform::MakeSysFileName(const StringRef& result, const char *filename) const noexcept
{
	return MassStorage::CombineName(result, GetSysDir().Ptr(), filename);
}

void Platform::AppendSysDir(const StringRef & path) const noexcept
{
	path.cat(GetSysDir().Ptr());
}

ReadLockedPointer<const char> Platform::GetSysDir() const noexcept
{
	ReadLocker lock(sysDirLock);
	return ReadLockedPointer<const char>(lock, InternalGetSysDir());
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
	const bool ok = laserPort.AssignPort(gb, reply, PinUsedBy::laser, PinAccess::pwm);
	if (ok)
	{
		SetLaserPwm(0);
	}
	return ok;
}

void Platform::SetLaserPwmFrequency(PwmFrequency freq) noexcept
{
	laserPort.SetFrequency(freq);
}

#endif

// Axis limits
void Platform::SetAxisMaximum(size_t axis, float value, bool byProbing) noexcept
{
	axisMaxima[axis] = value;
	if (byProbing)
	{
		axisMaximaProbed.SetBit(axis);
	}
	reprap.MoveUpdated();
}

void Platform::SetAxisMinimum(size_t axis, float value, bool byProbing) noexcept
{
	axisMinima[axis] = value;
	if (byProbing)
	{
		axisMinimaProbed.SetBit(axis);
	}
	reprap.MoveUpdated();
}

void Platform::InitZProbeFilters() noexcept
{
	zProbeOnFilter.Init(0);
	zProbeOffFilter.Init(0);
}

#if SUPPORT_INKJET

// Fire the inkjet (if any) in the given pattern
// If there is no inkjet, false is returned; if there is one this returns true
// So you can test for inkjet presence with if(platform->Inkjet(0))
bool Platform::Inkjet(int bitPattern) noexcept
{
	if (inkjetBits < 0)
		return false;
	if (!bitPattern)
		return true;

	for(int8_t i = 0; i < inkjetBits; i++)
	{
		if (bitPattern & 1)
		{
			digitalWrite(inkjetSerialOut, 1);			// Write data to shift register

			for(int8_t j = 0; j <= i; j++)
			{
				digitalWrite(inkjetShiftClock, HIGH);
				digitalWrite(inkjetShiftClock, LOW);
				digitalWrite(inkjetSerialOut, 0);
			}

			digitalWrite(inkjetStorageClock, HIGH);		// Transfers data from shift register to output register
			digitalWrite(inkjetStorageClock, LOW);

			digitalWrite(inkjetOutputEnable, LOW);		// Fire the droplet out
			delayMicroseconds(inkjetFireMicroseconds);
			digitalWrite(inkjetOutputEnable, HIGH);

			digitalWrite(inkjetClear, LOW);				// Clear to 0
			digitalWrite(inkjetClear, HIGH);

			delayMicroseconds(inkjetDelayMicroseconds); // Wait for the next bit
		}

		bitPattern >>= 1; // Put the next bit in the units column
	}

	return true;
}
#endif

#if HAS_CPU_TEMP_SENSOR

// CPU temperature
MinMaxCurrent Platform::GetMcuTemperatures() const noexcept
{
	MinMaxCurrent result;
	result.min = lowestMcuTemperature;
	result.current = GetCpuTemperature();
	result.max = highestMcuTemperature;
	return result;
}

#endif

#if HAS_VOLTAGE_MONITOR

// Power in voltage
MinMaxCurrent Platform::GetPowerVoltages() const noexcept
{
	MinMaxCurrent result;
	result.min = AdcReadingToPowerVoltage(lowestVin);
	result.current = AdcReadingToPowerVoltage(currentVin);
	result.max = AdcReadingToPowerVoltage(highestVin);
	return result;
}

float Platform::GetCurrentPowerVoltage() const noexcept
{
	return AdcReadingToPowerVoltage(currentVin);
}

#endif

#if HAS_12V_MONITOR

MinMaxCurrent Platform::GetV12Voltages() const noexcept
{
	MinMaxCurrent result;
	result.min = AdcReadingToPowerVoltage(lowestV12);
	result.current = AdcReadingToPowerVoltage(currentV12);
	result.max = AdcReadingToPowerVoltage(highestV12);
	return result;
}

float Platform::GetCurrentV12Voltage() const noexcept
{
	return AdcReadingToPowerVoltage(currentV12);
}

#endif

#if HAS_SMART_DRIVERS

// TMC driver temperatures
float Platform::GetTmcDriversTemperature(unsigned int boardNumber) const noexcept
{
#if defined(DUET3MINI)
	const DriversBitmap mask = (boardNumber == 0)
							? DriversBitmap::MakeLowestNBits(5)							// drivers 0-4 are on the main board
								: DriversBitmap::MakeLowestNBits(3).ShiftUp(5);			// drivers 5-7 are on the daughter board
#elif defined(DUET3)
	const DriversBitmap mask = DriversBitmap::MakeLowestNBits(6);						// there are 6 drivers, only one board
#elif defined(DUET_NG)
	const DriversBitmap mask = DriversBitmap::MakeLowestNBits(5).ShiftUp(5 * boardNumber);	// there are 5 drivers on each board
#elif defined(DUET_M)
	const DriversBitmap mask = (boardNumber == 0)
							? DriversBitmap::MakeLowestNBits(5)							// drivers 0-4 are on the main board
								: DriversBitmap::MakeLowestNBits(2).ShiftUp(5);			// drivers 5-6 are on the daughter board
#elif defined(PCCB_10)
	const DriversBitmap mask = (boardNumber == 0)
							? DriversBitmap::MakeLowestNBits(2)							// drivers 0,1 are on-board
								: DriversBitmap::MakeLowestNBits(5).ShiftUp(2);			// drivers 2-7 are on the DueX5
#elif defined(PCCB_08_X5)
	const DriversBitmap mask = DriversBitmap::MakeLowestNBits(5);						// all drivers (0-4) are on the DueX, no further expansion supported
#elif defined(PCCB_08)
	const DriversBitmap mask = DriversBitmap::MakeLowestNBits(2);						// drivers 0, 1 are on-board, no expansion supported
#else
# error Undefined board
#endif
	return (temperatureShutdownDrivers.Intersects(mask)) ? 150.0
			: (temperatureWarningDrivers.Intersects(mask)) ? 100.0
				: 0.0;
}

#endif

#if HAS_STALL_DETECT

// Configure the motor stall detection, returning true if an error was encountered
GCodeResult Platform::ConfigureStallDetection(GCodeBuffer& gb, const StringRef& reply, OutputBuffer *& buf) THROWS(GCodeException)
{
	// Build a bitmap of all the drivers referenced
	// First looks for explicit driver numbers
	DriversBitmap drivers;
#if SUPPORT_CAN_EXPANSION
	CanDriversList canDrivers;
#endif
	if (gb.Seen('P'))
	{
		DriverId drives[NumDirectDrivers];
		size_t dCount = NumDirectDrivers;
		gb.GetDriverIdArray(drives, dCount);
		for (size_t i = 0; i < dCount; i++)
		{
			if (drives[i].IsLocal())
			{
				if (drives[i].localDriver >= numSmartDrivers)
				{
					reply.printf("Invalid local drive number '%u'", drives[i].localDriver);
					return GCodeResult::error;
				}
				drivers.SetBit(drives[i].localDriver);
			}
#if SUPPORT_CAN_EXPANSION
			else
			{
				canDrivers.AddEntry(drives[i]);
			}
#endif
		}
	}

	// Now look for axes
	for (size_t axis = 0; axis < reprap.GetGCodes().GetTotalAxes(); ++axis)
	{
		if (gb.Seen(reprap.GetGCodes().GetAxisLetters()[axis]))
		{
			IterateDrivers(axis,
							[&drivers](uint8_t localDriver){ drivers.SetBit(localDriver); }
#if SUPPORT_CAN_EXPANSION
						  , [&canDrivers](DriverId driver){ canDrivers.AddEntry(driver); }
#endif
						  );
		}
	}

	// Look for extruders
	if (gb.Seen('E'))
	{
		uint32_t extruderNumbers[MaxExtruders];
		size_t numSeen = MaxExtruders;
		gb.GetUnsignedArray(extruderNumbers, numSeen, false);
		for (size_t i = 0; i < numSeen; ++i)
		{
			if (extruderNumbers[i] < MaxExtruders)
			{
				const DriverId driver = GetExtruderDriver(extruderNumbers[i]);
				if (driver.IsLocal())
				{
					drivers.SetBit(driver.localDriver);
				}
#if SUPPORT_CAN_EXPANSION
				else
				{
					canDrivers.AddEntry(driver);
				}
#endif
			}
		}
	}

	// Now check for values to change
	bool seen = false;
	if (gb.Seen('S'))
	{
		seen = true;
		const int sgThreshold = gb.GetIValue();
		drivers.Iterate([sgThreshold](unsigned int drive, unsigned int) noexcept { SmartDrivers::SetStallThreshold(drive, sgThreshold); });
	}
	if (gb.Seen('F'))
	{
		seen = true;
		const bool sgFilter = (gb.GetIValue() == 1);
		drivers.Iterate([sgFilter](unsigned int drive, unsigned int) noexcept { SmartDrivers::SetStallFilter(drive, sgFilter); });
	}
	if (gb.Seen('H'))
	{
		seen = true;
		const unsigned int stepsPerSecond = gb.GetUIValue();
		drivers.Iterate([stepsPerSecond](unsigned int drive, unsigned int) noexcept { SmartDrivers::SetStallMinimumStepsPerSecond(drive, stepsPerSecond); });
	}
	if (gb.Seen('T'))
	{
		seen = true;
		const uint32_t coolStepConfig = gb.GetUIValue();
		drivers.Iterate([coolStepConfig](unsigned int drive, unsigned int) noexcept { SmartDrivers::SetRegister(drive, SmartDriverRegister::coolStep, coolStepConfig); } );
	}
	if (gb.Seen('R'))
	{
		seen = true;
		const int action = gb.GetIValue();
		switch (action)
		{
		case 0:
		default:
			logOnStallDrivers &= ~drivers;
			pauseOnStallDrivers &= ~drivers;
			rehomeOnStallDrivers &= ~drivers;
			break;

		case 1:
			rehomeOnStallDrivers &= ~drivers;
			pauseOnStallDrivers &= ~drivers;
			logOnStallDrivers |= drivers;
			break;

		case 2:
			logOnStallDrivers &= ~drivers;
			rehomeOnStallDrivers &= ~drivers;
			pauseOnStallDrivers |= drivers;
			break;

		case 3:
			logOnStallDrivers &= ~drivers;
			pauseOnStallDrivers &= ~drivers;
			rehomeOnStallDrivers |= drivers;
			break;
		}
	}

	if (seen)
	{
#if SUPPORT_CAN_EXPANSION
		return CanInterface::GetSetRemoteDriverStallParameters(canDrivers, gb, reply, buf);
#else
		return GCodeResult::ok;
#endif
	}

	// Print the stall status
	if (!OutputBuffer::Allocate(buf))
	{
		return GCodeResult::notFinished;
	}

	if (drivers.IsEmpty()
#if SUPPORT_CAN_EXPANSION
		&& canDrivers.IsEmpty()
#endif
	   )
	{
		drivers = DriversBitmap::MakeLowestNBits(numSmartDrivers);
	}

	drivers.Iterate
		([buf, this, &reply](unsigned int drive, unsigned int) noexcept
			{
#if SUPPORT_CAN_EXPANSION
				buf->lcatf("Driver 0.%u: ", drive);
#else
				buf->lcatf("Driver %u: ", drive);
#endif
				reply.Clear();										// we use 'reply' as a temporary buffer
				SmartDrivers::AppendStallConfig(drive, reply);
				buf->cat(reply.c_str());
				buf->catf(", action on stall: %s",
							(rehomeOnStallDrivers.IsBitSet(drive)) ? "rehome"
								: (pauseOnStallDrivers.IsBitSet(drive)) ? "pause"
									: (logOnStallDrivers.IsBitSet(drive)) ? "log"
										: "none"
						  );
			}
		);

# if SUPPORT_CAN_EXPANSION
	return CanInterface::GetSetRemoteDriverStallParameters(canDrivers, gb, reply, buf);
# else
	return GCodeResult::ok;
#endif
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
	// Exactly one of FHJPS is allowed
	unsigned int charsPresent = 0;
	for (char c : (const char[]){'R', 'J', 'F', 'H', 'P', 'S'})
	{
		charsPresent <<= 1;
		if (gb.Seen(c))
		{
			charsPresent |= 1;
		}
	}

	switch (charsPresent)
	{
	case 1:
		{
			const uint32_t gpioNumber = gb.GetLimitedUIValue('S', MaxGpOutPorts);
			return gpoutPorts[gpioNumber].Configure(gpioNumber, true, gb, reply);
		}

	case 2:
		{
			const uint32_t gpioNumber = gb.GetLimitedUIValue('P', MaxGpOutPorts);
			return gpoutPorts[gpioNumber].Configure(gpioNumber, false, gb, reply);
		}

	case 4:
		return reprap.GetHeat().ConfigureHeater(gb, reply);

	case 8:
		return reprap.GetFansManager().ConfigureFanPort(gb, reply);

	case 16:
		{
			const uint32_t gpinNumber = gb.GetLimitedUIValue('J', MaxGpInPorts);
			return gpinPorts[gpinNumber].Configure(gpinNumber, gb, reply);
		}
	case 32:
		{
			const uint32_t slot = gb.GetLimitedUIValue('R', MaxSpindles);
			return spindles[slot].Configure(gb, reply);
		}

	default:
		reply.copy("exactly one of FHJPSR must be given");
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

GCodeResult Platform::UpdateRemoteStepsPerMmAndMicrostepping(AxesBitmap axesAndExtruders, const StringRef& reply) noexcept
{
	CanDriversData<StepsPerUnitAndMicrostepping> data;
	axesAndExtruders.Iterate([this, &data](unsigned int axisOrExtruder, unsigned int count) noexcept
								{
									const StepsPerUnitAndMicrostepping driverData(this->driveStepsPerUnit[axisOrExtruder], this->microstepping[axisOrExtruder]);
									this->IterateRemoteDrivers(axisOrExtruder,
																[&data, &driverData](DriverId driver) noexcept
																{
																	data.AddEntry(driver, driverData);
																}
															  );
								}
							);
	return CanInterface::SetRemoteDriverStepsPerMmAndMicrostepping(data, reply);
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
		extrusionAncilliaryPwmPort.AppendDetails(reply);
	}
	return GCodeResult::ok;
}

#if MCU_HAS_UNIQUE_ID

// Get a pseudo-random number (not a true random number)
uint32_t Platform::Random() noexcept
{
	const uint32_t clocks = StepTimer::GetTimerTicks();
	return clocks ^ uniqueId[0] ^ uniqueId[1] ^ uniqueId[2] ^ uniqueId[3];
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

GCodeResult Platform::EutSetMotorCurrents(const CanMessageMultipleDrivesRequest<float>& msg, size_t dataLength, const StringRef& reply) noexcept
{
# if HAS_SMART_DRIVERS
	const auto drivers = Bitmap<uint16_t>::MakeFromRaw(msg.driversToUpdate);
	if (dataLength < msg.GetActualDataLength(drivers.CountSetBits()))
	{
		reply.copy("bad data length");
		return GCodeResult::error;
	}

	GCodeResult rslt = GCodeResult::ok;
	drivers.Iterate([this, &msg, &reply, &rslt](unsigned int driver, unsigned int count) -> void
						{
							if (driver >= NumDirectDrivers)
							{
								reply.lcatf("No such driver %u.%u", CanInterface::GetCanAddress(), driver);
								rslt = GCodeResult::error;
							}
							else
							{
								SetMotorCurrent(driver, msg.values[count], 906, reply);
							}
						}
				   );
	return rslt;
# else
	reply.copy("Setting not available for external drivers");
	return GCodeResult::error;
# endif
}

GCodeResult Platform::EutSetStepsPerMmAndMicrostepping(const CanMessageMultipleDrivesRequest<StepsPerUnitAndMicrostepping>& msg, size_t dataLength, const StringRef& reply) noexcept
{
	const auto drivers = Bitmap<uint16_t>::MakeFromRaw(msg.driversToUpdate);
	if (dataLength < msg.GetActualDataLength(drivers.CountSetBits()))
	{
		reply.copy("bad data length");
		return GCodeResult::error;
	}

	GCodeResult rslt = GCodeResult::ok;
	drivers.Iterate([this, &msg, &reply, &rslt](unsigned int driver, unsigned int count) -> void
						{
							if (driver >= NumDirectDrivers)
							{
								reply.lcatf("No such driver %u.%u", CanInterface::GetCanAddress(), driver);
								rslt = GCodeResult::error;
							}
							else
							{
								SetDriveStepsPerUnit(driver, msg.values[count].GetStepsPerUnit(), 0);
#if HAS_SMART_DRIVERS
								microstepping[driver] = msg.values[count].GetMicrostepping();
								const uint16_t microsteppingOnly = microstepping[driver] & 0x03FF;
								const bool interpolate = (microstepping[driver] & 0x8000) != 0;
								if (!SmartDrivers::SetMicrostepping(driver, microsteppingOnly, interpolate))
								{
									reply.lcatf("Driver %u.%u does not support x%u microstepping", CanInterface::GetCanAddress(), driver, microsteppingOnly);
									if (interpolate)
									{
										reply.cat(" with interpolation");
									}
									rslt = GCodeResult::error;
								}
#endif
							}
						}
					);
	return rslt;
}

GCodeResult Platform::EutHandleSetDriverStates(const CanMessageMultipleDrivesRequest<DriverStateControl>& msg, const StringRef& reply) noexcept
{
	//TODO check message is long enough for the number of drivers specified
	const auto drivers = Bitmap<uint16_t>::MakeFromRaw(msg.driversToUpdate);
	drivers.Iterate([this, &msg](unsigned int driver, unsigned int count) -> void
		{
			switch (msg.values[count].mode)
			{
			case DriverStateControl::driverActive:
				EnableOneLocalDriver(driver, motorCurrents[driver]);
				driverState[driver] = DriverStatus::enabled;
				break;

			case DriverStateControl::driverIdle:
				UpdateMotorCurrent(driver, motorCurrents[driver] * idleCurrentFactor);
				driverState[driver] = DriverStatus::idle;
				break;

			case DriverStateControl::driverDisabled:
			default:
				DisableOneLocalDriver(driver);
				driverState[driver] = DriverStatus::disabled;
				break;
			}
		});
	return GCodeResult::ok;
}

float Platform::EutGetRemotePressureAdvance(size_t driver) const noexcept
{
	return (driver < ARRAY_SIZE(remotePressureAdvance)) ? remotePressureAdvance[driver] : 0.0;
}

GCodeResult Platform::EutSetRemotePressureAdvance(const CanMessageMultipleDrivesRequest<float>& msg, size_t dataLength, const StringRef& reply) noexcept
{
	const auto drivers = Bitmap<uint16_t>::MakeFromRaw(msg.driversToUpdate);
	if (dataLength < msg.GetActualDataLength(drivers.CountSetBits()))
	{
		reply.copy("bad data length");
		return GCodeResult::error;
	}

	GCodeResult rslt = GCodeResult::ok;
	drivers.Iterate([this, &msg, &reply, &rslt](unsigned int driver, unsigned int count) -> void
						{
							if (driver >= NumDirectDrivers)
							{
								reply.lcatf("No such driver %u.%u", CanInterface::GetCanAddress(), driver);
								rslt = GCodeResult::error;
							}
							else
							{
								remotePressureAdvance[driver] = msg.values[count];
							}
						}
				   );
	return rslt;
}

GCodeResult Platform::EutProcessM569(const CanMessageGeneric& msg, const StringRef& reply) noexcept
{
	CanMessageGenericParser parser(msg, M569Params);
	uint8_t drive;
	if (!parser.GetUintParam('P', drive))
	{
		reply.copy("Missing P parameter in CAN message");
		return GCodeResult::error;
	}

	if (drive >= NumDirectDrivers)
	{
		reply.printf("Driver number %u.%u out of range", CanInterface::GetCanAddress(), drive);
		return GCodeResult::error;
	}

	bool seen = false;
	uint8_t direction;
	if (parser.GetUintParam('S', direction))
	{
		seen = true;
		SetDirectionValue(drive, direction != 0);
	}
	int8_t rValue;
	if (parser.GetIntParam('R', rValue))
	{
		seen = true;
		SetEnableValue(drive, rValue);
	}

	float timings[4];
	size_t numTimings = 4;
	if (parser.GetFloatArrayParam('T', numTimings, timings))
	{
		seen = true;
		if (numTimings == 1)
		{
			timings[1] = timings[2] = timings[3] = timings[0];
		}
		else if (numTimings != 4)
		{
			reply.copy("bad timing parameter, expected 1 or 4 values");
			return GCodeResult::error;
		}
		SetDriverStepTiming(drive, timings);
	}

#if HAS_SMART_DRIVERS
	{
		uint32_t val;
		if (parser.GetUintParam('D', val))	// set driver mode
		{
			seen = true;
			if (!SmartDrivers::SetDriverMode(drive, val))
			{
				reply.printf("Driver %u.%u does not support mode '%s'", CanInterface::GetCanAddress(), drive, TranslateDriverMode(val));
				return GCodeResult::error;
			}
		}

		if (parser.GetUintParam('F', val))		// set off time
		{
			seen = true;
			if (!SmartDrivers::SetRegister(drive, SmartDriverRegister::toff, val))
			{
				reply.printf("Bad off time for driver %u", drive);
				return GCodeResult::error;
			}
		}

		if (parser.GetUintParam('B', val))		// set blanking time
		{
			seen = true;
			if (!SmartDrivers::SetRegister(drive, SmartDriverRegister::tblank, val))
			{
				reply.printf("Bad blanking time for driver %u", drive);
				return GCodeResult::error;
			}
		}

		if (parser.GetUintParam('V', val))		// set microstep interval for changing from stealthChop to spreadCycle
		{
			seen = true;
			if (!SmartDrivers::SetRegister(drive, SmartDriverRegister::tpwmthrs, val))
			{
				reply.printf("Bad mode change microstep interval for driver %u", drive);
				return GCodeResult::error;
			}
		}

#if SUPPORT_TMC51xx
		if (parser.GetUintParam('H', val))		// set coolStep threshold
		{
			seen = true;
			if (!SmartDrivers::SetRegister(drive, SmartDriverRegister::thigh, val))
			{
				reply.printf("Bad high speed microstep interval for driver %u", drive);
				return GCodeResult::error;
			}
		}
#endif
	}

	size_t numHvalues = 3;
	const uint8_t *hvalues;
	if (parser.GetArrayParam('Y', ParamDescriptor::ParamType::uint8_array, numHvalues, hvalues))		// set spread cycle hysteresis
	{
		seen = true;
		if (numHvalues == 2 || numHvalues == 3)
		{
			// There is a constraint on the sum of HSTRT and HEND, so set HSTART then HEND then HSTART again because one may go up and the other down
			(void)SmartDrivers::SetRegister(drive, SmartDriverRegister::hstart, hvalues[0]);
			bool ok = SmartDrivers::SetRegister(drive, SmartDriverRegister::hend, hvalues[1]);
			if (ok)
			{
				ok = SmartDrivers::SetRegister(drive, SmartDriverRegister::hstart, hvalues[0]);
			}
			if (ok && numHvalues == 3)
			{
				ok = SmartDrivers::SetRegister(drive, SmartDriverRegister::hdec, hvalues[2]);
			}
			if (!ok)
			{
				reply.printf("Bad hysteresis setting for driver %u", drive);
				return GCodeResult::error;
			}
		}
		else
		{
			reply.copy("Expected 2 or 3 Y values");
			return GCodeResult::error;
		}
	}
#endif
	if (!seen)
	{
		reply.printf("Driver %u.%u runs %s, active %s enable",
						CanInterface::GetCanAddress(),
						drive,
						(GetDirectionValue(drive)) ? "forwards" : "in reverse",
						(GetEnableValue(drive)) ? "high" : "low");

		float timings[4];
		const bool isSlowDriver = GetDriverStepTiming(drive, timings);
		if (isSlowDriver)
		{
			reply.catf(", step timing %.1f:%.1f:%.1f:%.1fus", (double)timings[0], (double)timings[1], (double)timings[2], (double)timings[3]);
		}
		else
		{
			reply.cat(", step timing fast");
		}

#if HAS_SMART_DRIVERS
		// It's a smart driver, so print the parameters common to all modes, except for the position
		reply.catf(", mode %s, ccr 0x%05" PRIx32 ", toff %" PRIu32 ", tblank %" PRIu32,
				TranslateDriverMode(SmartDrivers::GetDriverMode(drive)),
				SmartDrivers::GetRegister(drive, SmartDriverRegister::chopperControl),
				SmartDrivers::GetRegister(drive, SmartDriverRegister::toff),
				SmartDrivers::GetRegister(drive, SmartDriverRegister::tblank)
			);

# if SUPPORT_TMC51xx
		{
			const uint32_t thigh = SmartDrivers::GetRegister(drive, SmartDriverRegister::thigh);
			bool bdummy;
			const float mmPerSec = (12000000.0 * SmartDrivers::GetMicrostepping(drive, bdummy))/(256 * thigh * Platform::DriveStepsPerUnit(drive));
			reply.catf(", thigh %" PRIu32 " (%.1f mm/sec)", thigh, (double)mmPerSec);
		}
# endif

		// Print the additional parameters that are relevant in the current mode
		if (SmartDrivers::GetDriverMode(drive) == DriverMode::spreadCycle)
		{
			reply.catf(", hstart/hend/hdec %" PRIu32 "/%" PRIu32 "/%" PRIu32,
						SmartDrivers::GetRegister(drive, SmartDriverRegister::hstart),
						SmartDrivers::GetRegister(drive, SmartDriverRegister::hend),
						SmartDrivers::GetRegister(drive, SmartDriverRegister::hdec)
					  );
		}

# if SUPPORT_TMC22xx || SUPPORT_TMC51xx
		if (SmartDrivers::GetDriverMode(drive) == DriverMode::stealthChop)
		{
			const uint32_t tpwmthrs = SmartDrivers::GetRegister(drive, SmartDriverRegister::tpwmthrs);
			bool bdummy;
			const float mmPerSec = (12000000.0 * SmartDrivers::GetMicrostepping(drive, bdummy))/(256 * tpwmthrs * Platform::DriveStepsPerUnit(drive));
			const uint32_t pwmScale = SmartDrivers::GetRegister(drive, SmartDriverRegister::pwmScale);
			const uint32_t pwmAuto = SmartDrivers::GetRegister(drive, SmartDriverRegister::pwmAuto);
			const unsigned int pwmScaleSum = pwmScale & 0xFF;
			const int pwmScaleAuto = (int)((((pwmScale >> 16) & 0x01FF) ^ 0x0100) - 0x0100);
			const unsigned int pwmOfsAuto = pwmAuto & 0xFF;
			const unsigned int pwmGradAuto = (pwmAuto >> 16) & 0xFF;
			reply.catf(", tpwmthrs %" PRIu32 " (%.1f mm/sec), pwmScaleSum %u, pwmScaleAuto %d, pwmOfsAuto %u, pwmGradAuto %u",
						tpwmthrs, (double)mmPerSec, pwmScaleSum, pwmScaleAuto, pwmOfsAuto, pwmGradAuto);
		}
# endif
		// Finally, print the microstep position
		{
			const uint32_t mstepPos = SmartDrivers::GetRegister(drive, SmartDriverRegister::mstepPos);
			if (mstepPos < 1024)
			{
				reply.catf(", pos %" PRIu32, mstepPos);
			}
			else
			{
				reply.cat(", pos unknown");
			}
		}
#endif

	}
	return GCodeResult::ok;
}

#endif

// Process a 1ms tick interrupt
// This function must be kept fast so as not to disturb the stepper timing, so don't do any floating point maths in here.
// This is what we need to do:
// 1.  Kick off a new ADC conversion.
// 2.  Fetch and process the result of the last ADC conversion.
// 3a. If the last ADC conversion was for the Z probe, toggle the modulation output if using a modulated IR sensor.
// 3b. If the last ADC reading was a thermistor reading, check for an over-temperature situation and turn off the heater if necessary.
//     We do this here because the usual polling loop sometimes gets stuck trying to send data to the USB port.

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
		}
		if (currentVin < lowestVin)
		{
			lowestVin = currentVin;
		}
# endif

# if HAS_12V_MONITOR
		currentV12 = AnalogInReadChannel(v12MonitorAdcChannel);
		if (currentV12 > highestV12)
		{
			highestV12 = currentV12;
		}
		if (currentV12 < lowestV12)
		{
			lowestV12 = currentV12;
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
			SmartDrivers::TurnDriversOff();
			// We deliberately do not clear driversPowered here or increase the over voltage event count - we let the spin loop handle that
		}
# endif
	}
#endif

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
