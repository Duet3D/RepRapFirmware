/****************************************************************************************************

 RepRapFirmware - Platform: RepRapPro Ormerod with Duet controller

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

#include "RepRapFirmware.h"

#include "sam/drivers/tc/tc.h"
#include "sam/drivers/hsmci/hsmci.h"
#include "sd_mmc.h"

#ifdef DUET_NG
# include "TMC2660.h"
# include "FirmwareUpdater.h"
#endif

extern char _end;
extern "C" char *sbrk(int i);

#ifdef DUET_NG
const uint16_t driverPowerOnAdcReading = (uint16_t)(4096 * 10.0/PowerFailVoltageRange);			// minimum voltage at which we initialise the drivers
const uint16_t driverPowerOffAdcReading = (uint16_t)(4096 * 9.5/PowerFailVoltageRange);			// voltages below this flag the drivers as unusable
const uint16_t driverOverVoltageAdcReading = (uint16_t)(4096 * 29.5/PowerFailVoltageRange);		// voltages above this cause driver shutdown
const uint16_t driverNormalVoltageAdcReading = (uint16_t)(4096 * 27.5/PowerFailVoltageRange);	// voltages at or below this are normal
#endif

const uint8_t memPattern = 0xA5;

static uint32_t fanInterruptCount = 0;				// accessed only in ISR, so no need to declare it volatile
const uint32_t fanMaxInterruptCount = 32;			// number of fan interrupts that we average over
static volatile uint32_t fanLastResetTime = 0;		// time (microseconds) at which we last reset the interrupt count, accessed inside and outside ISR
static volatile uint32_t fanInterval = 0;			// written by ISR, read outside the ISR

const float minStepPulseTiming = 0.2;				// we assume that we always generate step high and low times at least this wide without special action

const int Heater0LogicalPin = 0;
const int Fan0LogicalPin = 20;
const int EndstopXLogicalPin = 40;
const int Special0LogicalPin = 60;
const int DueX5Gpio0LogicalPin = 100;

//#define MOVE_DEBUG

#ifdef MOVE_DEBUG
unsigned int numInterruptsScheduled = 0;
unsigned int numInterruptsExecuted = 0;
uint32_t nextInterruptTime = 0;
uint32_t nextInterruptScheduledAt = 0;
uint32_t lastInterruptTime = 0;
#endif

// Global functions

// Urgent initialisation function
// This is called before general init has been done, and before constructors for C++ static data have been called.
// Therefore, be very careful what you do here!
void UrgentInit()
{
#ifdef DUET_NG
	// When the reset button is pressed on pre-production Duet WiFi boards, if the TMC2660 drivers were previously enabled then we get
	// uncommanded motor movements if the STEP lines pick up any noise. Try to reduce that by initialising the drivers early here.
	// On the production boards the ENN line is pulled high and that prevents motor movements.
	for (size_t drive = 0; drive < DRIVES; ++drive)
	{
		pinMode(STEP_PINS[drive], OUTPUT_LOW);
		pinMode(DIRECTION_PINS[drive], OUTPUT_LOW);
		pinMode(ENABLE_PINS[drive], OUTPUT_HIGH);
	}
#endif
}

// Arduino initialise and loop functions
// Put nothing in these other than calls to the RepRap equivalents
void setup()
{
	// Fill the free memory with a pattern so that we can check for stack usage and memory corruption
	char* heapend = sbrk(0);
	register const char * stack_ptr asm ("sp");
	while (heapend + 16 < stack_ptr)
	{
		*heapend++ = memPattern;
	}

	reprap.Init();
}

void loop()
{
	reprap.Spin();
}

extern "C"
{
	// This intercepts the 1ms system tick. It must return 'false', otherwise the Arduino core tick handler will be bypassed.
	int sysTickHook()
	{
		reprap.Tick();
		return 0;
	}

	void NMI_Handler        () { reprap.GetPlatform()->SoftwareReset((uint16_t)SoftwareResetReason::NMI); }
	void HardFault_Handler  () { reprap.GetPlatform()->SoftwareReset((uint16_t)SoftwareResetReason::hardFault); }
	void MemManage_Handler  () { reprap.GetPlatform()->SoftwareReset((uint16_t)SoftwareResetReason::memManage); }
	void BusFault_Handler   () { reprap.GetPlatform()->SoftwareReset((uint16_t)SoftwareResetReason::busFault); }
	void UsageFault_Handler () { reprap.GetPlatform()->SoftwareReset((uint16_t)SoftwareResetReason::usageFault); }
	void SVC_Handler		() { reprap.GetPlatform()->SoftwareReset((uint16_t)SoftwareResetReason::otherFault); }
	void DebugMon_Handler   () { reprap.GetPlatform()->SoftwareReset((uint16_t)SoftwareResetReason::otherFault); }
	void PendSV_Handler		() { reprap.GetPlatform()->SoftwareReset((uint16_t)SoftwareResetReason::otherFault); }
}



// ZProbeParameters class

void ZProbeParameters::Init(float h)
{
	adcValue = Z_PROBE_AD_VALUE;
	xOffset = yOffset = 0.0;
	height = h;
	calibTemperature = 20.0;
	temperatureCoefficient = 0.0;	// no default temperature correction
	diveHeight = DEFAULT_Z_DIVE;
	probeSpeed = DEFAULT_PROBE_SPEED;
	travelSpeed = DEFAULT_TRAVEL_SPEED;
	recoveryTime = extraParam = 0.0;
	invertReading = false;
}

float ZProbeParameters::GetStopHeight(float temperature) const
{
	return ((temperature - calibTemperature) * temperatureCoefficient) + height;
}

bool ZProbeParameters::WriteParameters(FileStore *f, unsigned int probeType) const
{
	scratchString.printf("G31 T%u P%d X%.1f Y%.1f Z%.2f\n", probeType, adcValue, xOffset, yOffset, height);
	return f->Write(scratchString.Pointer());
}

//*************************************************************************************************
// Platform class

Platform::Platform() :
		board(DEFAULT_BOARD_TYPE), active(false), errorCodeBits(0),
		auxGCodeReply(nullptr), fileStructureInitialised(false), tickState(0), debugCode(0)
{
	// Output
	auxOutput = new OutputStack();
	aux2Output = new OutputStack();
	usbOutput = new OutputStack();

	// Files

	massStorage = new MassStorage(this);

	for (size_t i = 0; i < MAX_FILES; i++)
	{
		files[i] = new FileStore(this);
	}
}

//*******************************************************************************************************************

void Platform::Init()
{
	// Deal with power first
	pinMode(ATX_POWER_PIN, OUTPUT_LOW);

	SetBoardType(BoardType::Auto);

	// Real-time clock
	realTime = 0;

	// Comms
	baudRates[0] = MAIN_BAUD_RATE;
	baudRates[1] = AUX_BAUD_RATE;
#if NUM_SERIAL_CHANNELS >= 2
	baudRates[2] = AUX2_BAUD_RATE;
#endif
	commsParams[0] = 0;
	commsParams[1] = 1;							// by default we require a checksum on data from the aux port, to guard against overrun errors
#if NUM_SERIAL_CHANNELS >= 2
	commsParams[2] = 0;
#endif

	auxDetected = false;
	auxSeq = 0;

	SERIAL_MAIN_DEVICE.begin(baudRates[0]);
	SERIAL_AUX_DEVICE.begin(baudRates[1]);		// this can't be done in the constructor because the Arduino port initialisation isn't complete at that point
#ifdef SERIAL_AUX2_DEVICE
	SERIAL_AUX2_DEVICE.begin(baudRates[2]);
#endif

	compatibility = marlin;				// default to Marlin because the common host programs expect the "OK" response to commands
	ARRAY_INIT(ipAddress, IP_ADDRESS);
	ARRAY_INIT(netMask, NET_MASK);
	ARRAY_INIT(gateWay, GATE_WAY);

#if defined(DUET_NG) && defined(DUET_WIFI)
	memset(macAddress, 0xFF, sizeof(macAddress));
#else
	ARRAY_INIT(macAddress, MAC_ADDRESS);
#endif

	zProbeType = 0;	// Default is to use no Z probe switch
	zProbeAxes = Z_PROBE_AXES;
	SetZProbeDefaults();

	// We need to initialise at least some of the time stuff before we call MassStorage::Init()
	addToTime = 0.0;
	lastTimeCall = 0;
	lastTime = Time();
	longWait = lastTime;

	// File management
	massStorage->Init();

	for (size_t file = 0; file < MAX_FILES; file++)
	{
		files[file]->Init();
	}

	fileStructureInitialised = true;

#if !defined(DUET_NG) && !defined(__RADDS__)
	mcpDuet.begin();							// only call begin once in the entire execution, this begins the I2C comms on that channel for all objects
	mcpExpansion.setMCP4461Address(0x2E);		// not required for mcpDuet, as this uses the default address
#endif

	// DRIVES
	ARRAY_INIT(endStopPins, END_STOP_PINS);
	ARRAY_INIT(maxFeedrates, MAX_FEEDRATES);
	ARRAY_INIT(accelerations, ACCELERATIONS);
	ARRAY_INIT(driveStepsPerUnit, DRIVE_STEPS_PER_UNIT);
	ARRAY_INIT(instantDvs, INSTANT_DVS);

#if !defined(DUET_NG) && !defined(__RADDS__)
	// Motor current setting on Duet 0.6 and 0.8.5
	ARRAY_INIT(potWipes, POT_WIPES);
	senseResistor = SENSE_RESISTOR;
	maxStepperDigipotVoltage = MAX_STEPPER_DIGIPOT_VOLTAGE;
	stepperDacVoltageRange = STEPPER_DAC_VOLTAGE_RANGE;
	stepperDacVoltageOffset = STEPPER_DAC_VOLTAGE_OFFSET;
#endif

	// Z PROBE
	zProbePin = Z_PROBE_PIN;
	zProbeAdcChannel = PinToAdcChannel(zProbePin);
	InitZProbe();		// this also sets up zProbeModulationPin

	// AXES
	ARRAY_INIT(axisMaxima, AXIS_MAXIMA);
	ARRAY_INIT(axisMinima, AXIS_MINIMA);

	idleCurrentFactor = DEFAULT_IDLE_CURRENT_FACTOR;

	// SD card interfaces
	for (size_t i = 0; i < NumSdCards; ++i)
	{
		const Pin p = SdCardDetectPins[i];
		if (p != NoPin)
		{
			setPullup(p, true);
		}
	}

	// Motors
	// Disable parallel writes to all pins. We re-enable them for the step pins.
	PIOA->PIO_OWDR = 0xFFFFFFFF;
	PIOB->PIO_OWDR = 0xFFFFFFFF;
	PIOC->PIO_OWDR = 0xFFFFFFFF;
	PIOD->PIO_OWDR = 0xFFFFFFFF;

	for (size_t drive = 0; drive < DRIVES; drive++)
	{
		enableValues[drive] = false;				// assume active low enable signal
		directions[drive] = true;					// drive moves forwards by default

		// Map axes and extruders straight through
		if (drive < MAX_AXES)
		{
			axisDrivers[drive].numDrivers = 1;
			axisDrivers[drive].driverNumbers[0] = (uint8_t)drive;
			endStopType[drive] =
#if defined(DUET_NG) || defined(__RADDS__)
									EndStopType::lowEndStop;	// default to low endstop
#else
									(drive == Y_AXIS)
									? EndStopType::lowEndStop	// for Ormerod 2/Huxley/Mendel compatibility
									: EndStopType::noEndStop;	// for Ormerod/Huxley/Mendel compatibility
#endif
			endStopLogicLevel[drive] = true;					// assume all endstops use active high logic e.g. normally-closed switch to ground
		}

		if (drive >= MIN_AXES)
		{
			extruderDrivers[drive - MIN_AXES] = (uint8_t)drive;
			SetPressureAdvance(drive - MIN_AXES, 0.0);
		}
		driveDriverBits[drive] = CalcDriverBitmap(drive);

		// Set up the control pins and endstops
		pinMode(STEP_PINS[drive], OUTPUT_LOW);
		pinMode(DIRECTION_PINS[drive], OUTPUT_LOW);
		pinMode(ENABLE_PINS[drive], OUTPUT_HIGH);				// this is OK for the TMC2660 CS pins too

		const PinDescription& pinDesc = g_APinDescription[STEP_PINS[drive]];
		pinDesc.pPort->PIO_OWER = pinDesc.ulPin;				// enable parallel writes to the step pins

		motorCurrents[drive] = 0.0;
		motorCurrentFraction[drive] = 1.0;
		driverState[drive] = DriverStatus::disabled;
	}

	slowDriverStepPulseClocks = 0;								// no extended driver timing configured yet
	slowDrivers = 0;											// assume no drivers need extended step pulse timing

#ifdef DUET_NG
	// Test for presence of a DueX2 or DueX5 expansion board and work out how many TMC2660 drivers we have
	// The SX1509B has an independent power on reset, so give it some time
	delay(200);
	expansionBoard = DuetExpansion::Init();
	switch (expansionBoard)
	{
	case ExpansionBoardType::DueX2:
		numTMC2660Drivers = 7;
		break;
	case ExpansionBoardType::DueX5:
		numTMC2660Drivers = 10;
		break;
	case ExpansionBoardType::none:
	case ExpansionBoardType::DueX0:
	default:
		numTMC2660Drivers = 5;									// assume that additional drivers are dumb enable/step/dir ones
		break;
	}

	// Initialise TMC2660 drivers
	driversPowered = false;
	TMC2660::Init(ENABLE_PINS, numTMC2660Drivers);
#endif

	// Allow extrusion ancilliary PWM to use FAN0 even if FAN0 has not been disabled, for backwards compatibility
	extrusionAncilliaryPwmValue = 0.0;
	extrusionAncilliaryPwmFrequency = DefaultPinWritePwmFreq;
	extrusionAncilliaryPwmLogicalPin = Fan0LogicalPin;
	extrusionAncilliaryPwmFirmwarePin = COOLING_FAN_PINS[0];
	extrusionAncilliaryPwmInvert =
#if defined(DUET_NG) || defined(__RADDS__)
			false;
#else
			(board == BoardType::Duet_06 || board == BoardType::Duet_07);
#endif
	ARRAY_INIT(tempSensePins, TEMP_SENSE_PINS);
	ARRAY_INIT(heatOnPins, HEAT_ON_PINS);
	ARRAY_INIT(spiTempSenseCsPins, SpiTempSensorCsPins);

	configuredHeaters = (DefaultBedHeater >= 0) ? (1 << DefaultBedHeater) : 0;
	heatSampleTicks = HEAT_SAMPLE_TIME * SecondsToMillis;

	// Enable pullups on all the SPI CS pins. This is required if we are using more than one device on the SPI bus.
	// Otherwise, when we try to initialise the first device, the other devices may respond as well because their CS lines are not high.
	for (size_t i = 0; i < MaxSpiTempSensors; ++i)
	{
		setPullup(SpiTempSensorCsPins[i], true);
	}

	for (size_t heater = 0; heater < HEATERS; heater++)
	{
		if (heatOnPins[heater] != NoPin)
		{
			pinMode(heatOnPins[heater], (HEAT_ON) ? OUTPUT_LOW : OUTPUT_HIGH);
		}
		AnalogChannelNumber chan = PinToAdcChannel(tempSensePins[heater]);	// translate the Arduino Due Analog pin number to the SAM ADC channel number
		pinMode(tempSensePins[heater], AIN);
		thermistorAdcChannels[heater] = chan;
		AnalogInEnableChannel(chan, true);

		SetThermistorNumber(heater, heater);								// map the thermistor straight through
		thermistors[heater].SetParameters(
				(heater == DefaultBedHeater) ? BED_R25 : EXT_R25,
				(heater == DefaultBedHeater) ? BED_BETA : EXT_BETA,
				(heater == DefaultBedHeater) ? BED_SHC : EXT_SHC,
				THERMISTOR_SERIES_RS);										// initialise the thermistor parameters
		thermistorFilters[heater].Init(0);
	}

	// Fans
	InitFans();

	// Hotend configuration
	nozzleDiameter = NOZZLE_DIAMETER;
	filamentWidth = FILAMENT_WIDTH;

#if SUPPORT_INKJET
	// Inkjet

	inkjetBits = INKJET_BITS;
	if (inkjetBits >= 0)
	{
		inkjetFireMicroseconds = INKJET_FIRE_MICROSECONDS;
		inkjetDelayMicroseconds = INKJET_DELAY_MICROSECONDS;

		inkjetSerialOut = INKJET_SERIAL_OUT;
		pinMode(inkjetSerialOut, OUTPUT_LOW);

		inkjetShiftClock = INKJET_SHIFT_CLOCK;
		pinMode(inkjetShiftClock, OUTPUT_LOW);

		inkjetStorageClock = INKJET_STORAGE_CLOCK;
		pinMode(inkjetStorageClock, OUTPUT_LOW);

		inkjetOutputEnable = INKJET_OUTPUT_ENABLE;
		pinMode(inkjetOutputEnable, OUTPUT_HIGH);

		inkjetClear = INKJET_CLEAR;
		pinMode(inkjetClear, OUTPUT_HIGH);
	}
#endif

	// MCU temperature and power monitoring
#ifndef __RADDS__
	temperatureAdcChannel = GetTemperatureAdcChannel();
	AnalogInEnableChannel(temperatureAdcChannel, true);
	currentMcuTemperature = highestMcuTemperature = 0;
	lowestMcuTemperature = 4095;
	mcuAlarmTemperature = 80.0;			// need to set the quite high here because the sensor is not be calibrated yet
#endif
	mcuTemperatureAdjust = 0.0;

#ifdef DUET_NG
	vInMonitorAdcChannel = PinToAdcChannel(PowerMonitorVinDetectPin);
	pinMode(PowerMonitorVinDetectPin, AIN);
	AnalogInEnableChannel(vInMonitorAdcChannel, true);
	currentVin = highestVin = 0;
	lowestVin = 9999;
	numUnderVoltageEvents = numOverVoltageEvents = 0;
#endif

	// Clear the spare pin configuration
	memset(logicalPinModes, PIN_MODE_NOT_CONFIGURED, sizeof(logicalPinModes));		// set all pins to "not configured"

	// Kick everything off
	lastTime = Time();
	longWait = lastTime;
	InitialiseInterrupts();		// also sets 'active' to true
}

void Platform::InvalidateFiles(const FATFS *fs)
{
	for (size_t i = 0; i < MAX_FILES; i++)
	{
		files[i]->Invalidate(fs);
	}
}

bool Platform::AnyFileOpen(const FATFS *fs) const
{
	for (size_t i = 0; i < MAX_FILES; i++)
	{
		if (files[i]->IsOpenOn(fs))
		{
			return true;
		}
	}
	return false;
}

// Specify which thermistor channel a particular heater uses
void Platform::SetThermistorNumber(size_t heater, size_t thermistor)
{
	heaterTempChannels[heater] = thermistor;

	// Initialize the associated SPI temperature sensor?
	if (thermistor >= FirstThermocoupleChannel && thermistor < FirstThermocoupleChannel + MaxSpiTempSensors)
	{
		SpiTempSensors[thermistor - FirstThermocoupleChannel].InitThermocouple(SpiTempSensorCsPins[thermistor - FirstThermocoupleChannel]);
	}
	else if (thermistor >= FirstRtdChannel && thermistor < FirstRtdChannel + MaxSpiTempSensors)
	{
		SpiTempSensors[thermistor - FirstRtdChannel].InitRtd(spiTempSenseCsPins[thermistor - FirstRtdChannel]);
	}

	reprap.GetHeat()->ResetFault(heater);
}

int Platform::GetThermistorNumber(size_t heater) const
{
	return heaterTempChannels[heater];
}

void Platform::SetZProbeDefaults()
{
	switchZProbeParameters.Init(0.0);
	irZProbeParameters.Init(Z_PROBE_STOP_HEIGHT);
	alternateZProbeParameters.Init(Z_PROBE_STOP_HEIGHT);
}

void Platform::InitZProbe()
{
	zProbeOnFilter.Init(0);
	zProbeOffFilter.Init(0);

#if defined(DUET_NG) || defined(__RADDS__)
	zProbeModulationPin = Z_PROBE_MOD_PIN;
#else
	zProbeModulationPin = (board == BoardType::Duet_07 || board == BoardType::Duet_085) ? Z_PROBE_MOD_PIN07 : Z_PROBE_MOD_PIN;
#endif

	switch (zProbeType)
	{
	case 1:
	case 2:
		AnalogInEnableChannel(zProbeAdcChannel, true);
		pinMode(zProbePin, AIN);
		pinMode(zProbeModulationPin, OUTPUT_HIGH);		// enable the IR LED
		break;

	case 3:
		AnalogInEnableChannel(zProbeAdcChannel, true);
		pinMode(zProbePin, AIN);
		pinMode(zProbeModulationPin, OUTPUT_LOW);		// enable the alternate sensor
		break;

	case 4:
		AnalogInEnableChannel(zProbeAdcChannel, false);
		pinMode(zProbePin, INPUT_PULLUP);
		pinMode(endStopPins[E0_AXIS], INPUT_PULLUP);
		break;

	case 5:
	default:
		AnalogInEnableChannel(zProbeAdcChannel, false);
		pinMode(zProbePin, INPUT_PULLUP);
		pinMode(zProbeModulationPin, OUTPUT_HIGH);		// set high to take trash80's HE280 probe Teensy adapter out of reset
		break;

	case 6:
		AnalogInEnableChannel(zProbeAdcChannel, false);
		pinMode(zProbePin, INPUT_PULLUP);
		pinMode(endStopPins[E0_AXIS + 1], INPUT_PULLUP);
		break;

	case 7:
		AnalogInEnableChannel(zProbeAdcChannel, false);
		pinMode(zProbePin, INPUT_PULLUP);
		break;	//TODO (DeltaProbe)
	}
}

// Return the Z probe data.
// The ADC readings are 12 bits, so we convert them to 10-bit readings for compatibility with the old firmware.
int Platform::GetZProbeReading() const
{
	int zProbeVal = 0;			// initialised to avoid spurious compiler warning
	if (zProbeOnFilter.IsValid() && zProbeOffFilter.IsValid())
	{
		switch (zProbeType)
		{
		case 1:		// Simple or intelligent IR sensor
		case 3:		// Alternate sensor
		case 4:		// Switch connected to E0 endstop input
		case 5:		// Switch connected to Z probe input
		case 6:		// Switch connected to E1 endstop input
			zProbeVal = (int) ((zProbeOnFilter.GetSum() + zProbeOffFilter.GetSum()) / (8 * Z_PROBE_AVERAGE_READINGS));
			break;

		case 2:		// Dumb modulated IR sensor.
			// We assume that zProbeOnFilter and zProbeOffFilter average the same number of readings.
			// Because of noise, it is possible to get a negative reading, so allow for this.
			zProbeVal = (int) (((int32_t) zProbeOnFilter.GetSum() - (int32_t) zProbeOffFilter.GetSum()) / (int)(4 * Z_PROBE_AVERAGE_READINGS));
			break;

		case 7:		// Delta humming probe
			zProbeVal = (int) ((zProbeOnFilter.GetSum() + zProbeOffFilter.GetSum()) / (8 * Z_PROBE_AVERAGE_READINGS));	//TODO this is temporary
			break;

		default:
			return 0;
		}
	}

	return (GetCurrentZProbeParameters().invertReading) ? 1000 - zProbeVal : zProbeVal;
}

// Return the Z probe secondary values.
int Platform::GetZProbeSecondaryValues(int& v1, int& v2)
{
	if (zProbeOnFilter.IsValid() && zProbeOffFilter.IsValid())
	{
		switch (zProbeType)
		{
		case 2:		// modulated IR sensor
			v1 = (int) (zProbeOnFilter.GetSum() / (4 * Z_PROBE_AVERAGE_READINGS));	// pass back the reading with IR turned on
			return 1;
		default:
			break;
		}
	}
	return 0;
}

void Platform::SetZProbeAxes(uint32_t axes)
{
	zProbeAxes = axes;
}

// Get our best estimate of the Z probe temperature
float Platform::GetZProbeTemperature()
{
	const int8_t bedHeater = reprap.GetHeat()->GetBedHeater();
	if (bedHeater >= 0)
	{
		TemperatureError err;
		const float temp = GetTemperature(bedHeater, err);
		if (err == TemperatureError::success)
		{
			return temp;
		}
	}
	return 25.0;							// assume 25C if we can't read he bed temperature
}

float Platform::ZProbeStopHeight()
{
	return GetCurrentZProbeParameters().GetStopHeight(GetZProbeTemperature());
}

float Platform::GetZProbeDiveHeight() const
{
	return GetCurrentZProbeParameters().diveHeight;
}

float Platform::GetZProbeStartingHeight()
{
	const ZProbeParameters& params = GetCurrentZProbeParameters();
	return params.diveHeight + max<float>(params.GetStopHeight(GetZProbeTemperature()), 0.0);
}

float Platform::GetZProbeTravelSpeed() const
{
	return GetCurrentZProbeParameters().travelSpeed;
}

void Platform::SetZProbeType(int pt)
{
	zProbeType = (pt >= 0 && pt <= 7) ? pt : 0;
	InitZProbe();
}

const ZProbeParameters& Platform::GetZProbeParameters(int32_t probeType) const
{
	switch (probeType)
	{
	case 1:
	case 2:
	case 5:
		return irZProbeParameters;
	case 3:
	case 7:
		return alternateZProbeParameters;
	case 4:
	case 6:
	default:
		return switchZProbeParameters;
	}
}

void Platform::SetZProbeParameters(int32_t probeType, const ZProbeParameters& params)
{
	switch (probeType)
	{
	case 1:
	case 2:
	case 5:
		irZProbeParameters = params;
		break;

	case 3:
	case 7:
		alternateZProbeParameters = params;
		break;

	case 4:
	case 6:
	default:
		switchZProbeParameters = params;
		break;
	}
}

// Return true if the specified point is accessible to the Z probe
bool Platform::IsAccessibleProbePoint(float x, float y) const
{
	x -= GetCurrentZProbeParameters().xOffset;
	y -= GetCurrentZProbeParameters().yOffset;
	return (reprap.GetMove()->IsDeltaMode())
			? x * x + y * y < reprap.GetMove()->GetDeltaParams().GetPrintRadiusSquared()
			: x >= axisMinima[X_AXIS] && y >= axisMinima[Y_AXIS] && x <= axisMaxima[X_AXIS] && y <= axisMaxima[Y_AXIS];
}

// Return true if we must home X and Y before we home Z (i.e. we are using a bed probe)
bool Platform::MustHomeXYBeforeZ() const
{
	return (zProbeType != 0) && ((zProbeAxes & (1 << Z_AXIS)) != 0);
}

// Check the prerequisites for updating the main firmware. Return True if satisfied, else print as message and return false.
bool Platform::CheckFirmwareUpdatePrerequisites()
{
	if (!GetMassStorage()->FileExists(GetSysDir(), IAP_FIRMWARE_FILE))
	{
		MessageF(GENERIC_MESSAGE, "Error: Firmware binary \"%s\" not found\n", IAP_FIRMWARE_FILE);
		return false;
	}

	if (!GetMassStorage()->FileExists(GetSysDir(), IAP_UPDATE_FILE))
	{
		MessageF(GENERIC_MESSAGE, "Error: In-application programming binary \"%s\" not found\n", IAP_UPDATE_FILE);
		return false;
	}

	return true;
}

// Update the firmware. Prerequisites should be checked before calling this.
void Platform::UpdateFirmware()
{
	FileStore *iapFile = GetFileStore(GetSysDir(), IAP_UPDATE_FILE, false);
	if (iapFile == nullptr)
	{
		MessageF(FIRMWARE_UPDATE_MESSAGE, "IAP not found\n");
		return;
	}

	// The machine will be unresponsive for a few seconds, don't risk damaging the heaters...
	reprap.EmergencyStop();

	// Step 1 - Write update binary to Flash and overwrite the remaining space with zeros
	// Leave the last 1KB of Flash memory untouched, so we can reuse the NvData after this update

#if !defined(IFLASH_PAGE_SIZE) && defined(IFLASH0_PAGE_SIZE)
# define IFLASH_PAGE_SIZE	IFLASH0_PAGE_SIZE
#endif

	// Use a 32-bit aligned buffer. This gives us the option of calling the EFC functions directly in future.
	uint32_t data32[IFLASH_PAGE_SIZE/4];
	char* const data = reinterpret_cast<char *>(data32);

#if (SAM4S || SAM4E)
	// The EWP command is not supported for non-8KByte sectors in the SAM4 series.
	// So we have to unlock and erase the complete 64Kb sector first.
	// TODO save the NVRAM area and restore it later

	flash_unlock(IAP_FLASH_START, IAP_FLASH_END, nullptr, nullptr);
	flash_erase_sector(IAP_FLASH_START);

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
			const uint32_t rc = flash_write(flashAddr, data, IFLASH_PAGE_SIZE, 0);
			cpu_irq_enable();

			if (rc != FLASH_RC_OK)
			{
				MessageF(FIRMWARE_UPDATE_MESSAGE, "Error: Flash write failed, code=%u, address=0x%08x\n", rc, flashAddr);
				return;
			}
			// Verify written data
			if (memcmp(reinterpret_cast<void *>(flashAddr), data, bytesRead) != 0)
			{
				MessageF(FIRMWARE_UPDATE_MESSAGE, "Error: Verify during flash write failed, address=0x%08x\n", flashAddr);
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
	flash_lock(IAP_FLASH_START, IAP_FLASH_END, nullptr, nullptr);

#else	// SAM3X code

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
				MessageF(FIRMWARE_UPDATE_MESSAGE, "Error: Flash %s failed, code=%u, address=0x%08x\n", op, rc, flashAddr);
				return;
			}
			// Verify written data
			if (memcmp(reinterpret_cast<void *>(flashAddr), data, bytesRead) != 0)
			{
				MessageF(FIRMWARE_UPDATE_MESSAGE, "Error: Verify during flash write failed, address=0x%08x\n", flashAddr);
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
#endif

	iapFile->Close();
	Message(FIRMWARE_UPDATE_MESSAGE, "Updating main firmware\n");

	// Allow time for the firmware update message to be sent
	uint32_t now = millis();
	while (FlushMessages() && millis() - now < 2000) { }

	// Step 2 - Let the firmware do whatever is necessary before we exit this program
	reprap.Exit();

	// Step 3 - Reallocate the vector table and program entry point to the new IAP binary
	// This does essentially what the Atmel AT02333 paper suggests (see 3.2.2 ff)

	// Disable all IRQs
	cpu_irq_disable();
	for(size_t i = 0; i < 8; i++)
	{
		NVIC->ICER[i] = 0xFFFFFFFF;		// Disable IRQs
		NVIC->ICPR[i] = 0xFFFFFFFF;		// Clear pending IRQs
	}

	// Our SAM3X doesn't support disabling the watchdog, so leave it running.
	// The IAP binary will kick it as soon as it's started

	// Modify vector table location
	__DSB();
	__ISB();
	SCB->VTOR = ((uint32_t)IAP_FLASH_START & SCB_VTOR_TBLOFF_Msk);
	__DSB();
	__ISB();

	// Reset stack pointer, enable IRQs again and start the new IAP binary
	__set_MSP(*(uint32_t *)IAP_FLASH_START);
	cpu_irq_enable();

	void *entryPoint = (void *)(*(uint32_t *)(IAP_FLASH_START + 4));
	goto *entryPoint;
}

// Send the beep command to the aux channel. There is no flow control on this port, so it can't block for long.
void Platform::Beep(int freq, int ms)
{
	MessageF(AUX_MESSAGE, "{\"beep_freq\":%d,\"beep_length\":%d}\n", freq, ms);
}

// Send a short message to the aux channel. There is no flow control on this port, so it can't block for long.
void Platform::SendMessage(const char* msg)
{
	OutputBuffer *buf;
	if (OutputBuffer::Allocate(buf))
	{
		buf->copy("{\"message\":");
		buf->EncodeString(msg, strlen(msg), false, true);
		buf->cat("}\n");
		Message(AUX_MESSAGE, buf);
	}
}

// Note: the use of floating point time will cause the resolution to degrade over time.
// For example, 1ms time resolution will only be available for about half an hour from startup.
// Personally, I (dc42) would rather just maintain and provide the time in milliseconds in a uint32_t.
// This would wrap round after about 49 days, but that isn't difficult to handle.
float Platform::Time()
{
	unsigned long now = micros();
	if (now < lastTimeCall) // Has timer overflowed?
	{
		addToTime += ((float) ULONG_MAX) * TIME_FROM_REPRAP;
	}
	lastTimeCall = now;
	return addToTime + TIME_FROM_REPRAP * (float) now;
}

void Platform::Exit()
{
	// Close all files
	for(size_t i = 0; i < MAX_FILES; i++)
	{
		while (files[i]->inUse)
		{
			files[i]->Close();
		}
	}

	// Release the aux output stack (should release the others too!)
	while (auxGCodeReply != nullptr)
	{
		auxGCodeReply = OutputBuffer::Release(auxGCodeReply);
	}

	// Stop processing data. Don't try to send a message because it will probably never get there.
	active = false;
}

Compatibility Platform::Emulating() const
{
	return (compatibility == reprapFirmware) ? me : compatibility;
}

void Platform::SetEmulating(Compatibility c)
{
	if (c != me && c != reprapFirmware && c != marlin)
	{
		Message(GENERIC_MESSAGE, "Attempt to emulate unsupported firmware.\n");
		return;
	}
	if (c == reprapFirmware)
	{
		c = me;
	}
	compatibility = c;
}

void Platform::UpdateNetworkAddress(byte dst[4], const byte src[4])
{
	for (uint8_t i = 0; i < 4; i++)
	{
		dst[i] = src[i];
	}
}

void Platform::SetIPAddress(byte ip[])
{
	UpdateNetworkAddress(ipAddress, ip);
}

void Platform::SetGateWay(byte gw[])
{
	UpdateNetworkAddress(gateWay, gw);
}

void Platform::SetNetMask(byte nm[])
{
	UpdateNetworkAddress(netMask, nm);
}

// Flush messages to USB and aux, returning true if there is more to send
bool Platform::FlushMessages()
{
	// Write non-blocking data to the AUX line
	OutputBuffer *auxOutputBuffer = auxOutput->GetFirstItem();
	if (auxOutputBuffer != nullptr)
	{
		size_t bytesToWrite = min<size_t>(SERIAL_AUX_DEVICE.canWrite(), auxOutputBuffer->BytesLeft());
		if (bytesToWrite > 0)
		{
			SERIAL_AUX_DEVICE.write(auxOutputBuffer->Read(bytesToWrite), bytesToWrite);
		}

		if (auxOutputBuffer->BytesLeft() == 0)
		{
			auxOutputBuffer = OutputBuffer::Release(auxOutputBuffer);
			auxOutput->SetFirstItem(auxOutputBuffer);
		}
	}

	// Write non-blocking data to the second AUX line
	OutputBuffer *aux2OutputBuffer = aux2Output->GetFirstItem();
	if (aux2OutputBuffer != nullptr)
	{
#ifdef SERIAL_AUX2_DEVICE
		size_t bytesToWrite = min<size_t>(SERIAL_AUX2_DEVICE.canWrite(), aux2OutputBuffer->BytesLeft());
		if (bytesToWrite > 0)
		{
			SERIAL_AUX2_DEVICE.write(aux2OutputBuffer->Read(bytesToWrite), bytesToWrite);
		}

		if (aux2OutputBuffer->BytesLeft() == 0)
		{
			aux2OutputBuffer = OutputBuffer::Release(aux2OutputBuffer);
			aux2Output->SetFirstItem(aux2OutputBuffer);
		}
#else
		aux2OutputBuffer = OutputBuffer::Release(aux2OutputBuffer);
#endif
	}

	// Write non-blocking data to the USB line
	OutputBuffer *usbOutputBuffer = usbOutput->GetFirstItem();
	if (usbOutputBuffer != nullptr)
	{
		if (!SERIAL_MAIN_DEVICE)
		{
			// If the USB port is not opened, free the data left for writing
			OutputBuffer::ReleaseAll(usbOutputBuffer);
			usbOutput->SetFirstItem(nullptr);
		}
		else
		{
			// Write as much data as we can...
			size_t bytesToWrite = min<size_t>(SERIAL_MAIN_DEVICE.canWrite(), usbOutputBuffer->BytesLeft());
			if (bytesToWrite > 0)
			{
				SERIAL_MAIN_DEVICE.write(usbOutputBuffer->Read(bytesToWrite), bytesToWrite);
			}

			if (usbOutputBuffer->BytesLeft() == 0 || usbOutputBuffer->GetAge() > SERIAL_MAIN_TIMEOUT)
			{
				usbOutputBuffer = OutputBuffer::Release(usbOutputBuffer);
				usbOutput->SetFirstItem(usbOutputBuffer);
			}
		}
	}

	return auxOutput->GetFirstItem() != nullptr
		|| aux2Output->GetFirstItem() != nullptr
		|| usbOutput->GetFirstItem() != nullptr;
}

void Platform::Spin()
{
	if (!active)
		return;

	// Check if any files are supposed to be closed
	for (size_t i = 0; i < MAX_FILES; i++)
	{
		if (files[i]->closeRequested)
		{
			// We cannot do this in ISRs, so do it here
			files[i]->Close();
		}
	}

	// Try to flush messages to serial ports
	(void)FlushMessages();

	// Thermostatically-controlled fans
	for (size_t fan = 0; fan < NUM_FANS; ++fan)
	{
		fans[fan].Check();
	}

	// Check the MCU max and min temperatures
#ifndef __RADDS__
	if (currentMcuTemperature > highestMcuTemperature)
	{
		highestMcuTemperature= currentMcuTemperature;
	}
	if (currentMcuTemperature < lowestMcuTemperature && currentMcuTemperature != 0)
	{
		lowestMcuTemperature = currentMcuTemperature;
	}
#endif

	// Diagnostics test
	if (debugCode == (int)DiagnosticTestType::TestSpinLockup)
	{
		for (;;) {}
	}

#ifdef DUET_NG
	// Check whether the TMC drivers need to be initialised.
	// The tick ISR also looks for over-voltage events, but it just disables the driver without changing driversPowerd or numOverVoltageEvents
	if (driversPowered)
	{
		if (currentVin < driverPowerOffAdcReading)
		{
			++numUnderVoltageEvents;
			driversPowered = false;
		}
		else if (currentVin > driverOverVoltageAdcReading)
		{
			driversPowered = false;
			++numOverVoltageEvents;
		}
	}
	else if (currentVin >= driverPowerOnAdcReading && currentVin <= driverNormalVoltageAdcReading)
	{
		driversPowered = true;
	}
	TMC2660::SetDriversPowered(driversPowered);
#endif

	// Update the time
	if (realTime != 0)
	{
		if (millis() - timeLastUpdatedMillis >= 1000)
		{
			++realTime;							// this assumes that time_t is a seconds-since-epoch counter, which is not guaranteed by the C standard
			timeLastUpdatedMillis += 1000;
		}
	}

	ClassReport(longWait);
}

void Platform::SoftwareReset(uint16_t reason)
{
	wdt_restart(WDT);							// kick the watchdog
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
			if (reprap.GetNetwork()->InLwip())
			{
				reason |= (uint16_t)SoftwareResetReason::inLwipSpin;
			}
			if (SERIAL_AUX_DEVICE.canWrite() == 0
#ifdef SERIAL_AUX2_DEVICE
				|| SERIAL_AUX2_DEVICE.canWrite() == 0
#endif
			   )
			{
				reason |= (uint16_t)SoftwareResetReason::inAuxOutput;	// if we are resetting because we are stuck in a Spin function, record whether we are trying to send to aux
			}
		}
		reason |= (uint16_t)reprap.GetSpinningModule();

		// Record the reason for the software reset
		// First find a free slot (wear levelling)
		size_t slot = SoftwareResetData::numberOfSlots;
		SoftwareResetData srdBuf[SoftwareResetData::numberOfSlots];

#ifdef DUET_NG
		if (flash_read_user_signature(reinterpret_cast<uint32_t*>(srdBuf), sizeof(srdBuf)/sizeof(uint32_t)) == FLASH_RC_OK)
#else
		DueFlashStorage::read(SoftwareResetData::nvAddress, srdBuf, sizeof(srdBuf));
#endif
		{
			while (slot != 0 && srdBuf[slot - 1].isVacant())
			{
				--slot;
			}
		}

		if (slot == SoftwareResetData::numberOfSlots)
		{
			// No free slots, so erase the area
#ifdef DUET_NG
			flash_erase_user_signature();
#endif
			memset(srdBuf, 0xFF, sizeof(srdBuf));
			slot = 0;
		}
		srdBuf[slot].magic = SoftwareResetData::magicValue;
		srdBuf[slot].resetReason = reason;
		GetStackUsage(NULL, NULL, &srdBuf[slot].neverUsedRam);

		// Save diagnostics data to Flash
#ifdef DUET_NG
		flash_write_user_signature(srdBuf, sizeof(srdBuf)/sizeof(uint32_t));
#else
		DueFlashStorage::write(SoftwareResetData::nvAddress, srdBuf, sizeof(srdBuf));
#endif
	}
	Reset();
	for(;;) {}
}


//*****************************************************************************************************************

// Interrupts

#ifndef DUET_NG
void NETWORK_TC_HANDLER()
{
	tc_get_status(NETWORK_TC, NETWORK_TC_CHAN);
	reprap.GetNetwork()->Interrupt();
}
#endif

void FanInterrupt()
{
	++fanInterruptCount;
	if (fanInterruptCount == fanMaxInterruptCount)
	{
		uint32_t now = micros();
		fanInterval = now - fanLastResetTime;
		fanLastResetTime = now;
		fanInterruptCount = 0;
	}
}

void Platform::InitialiseInterrupts()
{
	// Set the tick interrupt to the highest priority. We need to to monitor the heaters and kick the watchdog.
	NVIC_SetPriority(SysTick_IRQn, 0);						// set priority for tick interrupts - highest, because it kicks the watchdog

#ifdef DUET_NG
	NVIC_SetPriority(UART0_IRQn, 1);						// set priority for UART interrupt - must be higher than step interrupt
#else
	NVIC_SetPriority(UART_IRQn, 1);							// set priority for UART interrupt - must be higher than step interrupt
#endif

	// Timer interrupt for stepper motors
	// The clock rate we use is a compromise. Too fast and the 64-bit square roots take a long time to execute. Too slow and we lose resolution.
	// We choose a clock divisor of 32, which gives us 0.38us resolution. The next option is 128 which would give 1.524us resolution.
	pmc_set_writeprotect(false);
	pmc_enable_periph_clk((uint32_t) STEP_TC_IRQN);
	tc_init(STEP_TC, STEP_TC_CHAN, TC_CMR_WAVE | TC_CMR_WAVSEL_UP | TC_CMR_TCCLKS_TIMER_CLOCK3);
	STEP_TC->TC_CHANNEL[STEP_TC_CHAN].TC_IDR = ~(uint32_t)0; // interrupts disabled for now
	tc_start(STEP_TC, STEP_TC_CHAN);
	tc_get_status(STEP_TC, STEP_TC_CHAN);					// clear any pending interrupt
	NVIC_SetPriority(STEP_TC_IRQN, 2);						// set high priority for this IRQ; it's time-critical
	NVIC_EnableIRQ(STEP_TC_IRQN);

#ifndef DUET_NG
	// Timer interrupt to keep the networking timers running (called at 16Hz)
	pmc_enable_periph_clk((uint32_t) NETWORK_TC_IRQN);
	tc_init(NETWORK_TC, NETWORK_TC_CHAN, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK2);
	uint32_t rc = (VARIANT_MCK/8)/16;						// 8 because we selected TIMER_CLOCK2 above
	tc_write_ra(NETWORK_TC, NETWORK_TC_CHAN, rc/2);			// 50% high, 50% low
	tc_write_rc(NETWORK_TC, NETWORK_TC_CHAN, rc);
	tc_start(NETWORK_TC, NETWORK_TC_CHAN);
	NETWORK_TC ->TC_CHANNEL[NETWORK_TC_CHAN].TC_IER = TC_IER_CPCS;
	NETWORK_TC ->TC_CHANNEL[NETWORK_TC_CHAN].TC_IDR = ~TC_IER_CPCS;
	NVIC_SetPriority(NETWORK_TC_IRQN, 4);					// step interrupt is more time-critical than this one
	NVIC_EnableIRQ(NETWORK_TC_IRQN);
#endif

	// Interrupt for 4-pin PWM fan sense line
	if (coolingFanRpmPin != NoPin)
	{
		attachInterrupt(coolingFanRpmPin, FanInterrupt, FALLING);
	}

	// Tick interrupt for ADC conversions
	tickState = 0;
	currentHeater = 0;

	active = true;							// this enables the tick interrupt, which keeps the watchdog happy
}

#if 0	// not used
void Platform::DisableInterrupts()
{
	NVIC_DisableIRQ(STEP_IRQN);
#ifdef DUET_NG
	NVIC_DisableIRQ(NETWORK_IRQN);
#endif
}
#endif

//*************************************************************************************************

// Debugging variables
//extern "C" uint32_t longestWriteWaitTime, shortestWriteWaitTime, longestReadWaitTime, shortestReadWaitTime;
//extern uint32_t maxRead, maxWrite;

// This diagnostics function is the first to be called, so it calls Message to start with.
// All other messages generated by this and other diagnostics functions must call AppendMessage.
void Platform::Diagnostics(MessageType mtype)
{
	Message(mtype, "=== Platform ===\n");

	// Print memory stats and error codes to USB and copy them to the current webserver reply
	const char *ramstart =
#ifdef DUET_NG
			(char *) 0x20000000;
#else
			(char *) 0x20070000;
#endif
	const struct mallinfo mi = mallinfo();
	Message(mtype, "Memory usage:\n");
	MessageF(mtype, "Program static ram used: %d\n", &_end - ramstart);
	MessageF(mtype, "Dynamic ram used: %d\n", mi.uordblks);
	MessageF(mtype, "Recycled dynamic ram: %d\n", mi.fordblks);
	uint32_t currentStack, maxStack, neverUsed;
	GetStackUsage(&currentStack, &maxStack, &neverUsed);
	MessageF(mtype, "Current stack ram used: %d\n", currentStack);
	MessageF(mtype, "Maximum stack ram used: %d\n", maxStack);
	MessageF(mtype, "Never used ram: %d\n", neverUsed);

	// Show the up time and reason for the last reset
	const uint32_t now = (uint32_t)Time();		// get up time in seconds
	const char* resetReasons[8] = { "power up", "backup", "watchdog", "software",
#ifdef DUET_NG
	// On the SAM4E a watchdog reset may be reported as a user reset because of the capacitor on the NRST pin
									"reset button or watchdog",
#else
									"reset button",
#endif
									"?", "?", "?" };
	MessageF(mtype, "Last reset %02d:%02d:%02d ago, cause: %s\n",
			(unsigned int)(now/3600), (unsigned int)((now % 3600)/60), (unsigned int)(now % 60),
			resetReasons[(REG_RSTC_SR & RSTC_SR_RSTTYP_Msk) >> RSTC_SR_RSTTYP_Pos]);

	// Show the reset code stored at the last software reset
	{
		SoftwareResetData srdBuf[SoftwareResetData::numberOfSlots];
		memset(srdBuf, 0, sizeof(srdBuf));
		int slot = -1;

#ifdef DUET_NG
		if (flash_read_user_signature(reinterpret_cast<uint32_t*>(srdBuf), sizeof(srdBuf)/sizeof(uint32_t)) == FLASH_RC_OK)
#else
		DueFlashStorage::read(SoftwareResetData::nvAddress, srdBuf, sizeof(srdBuf));
#endif
		{
			// Find the last slot written
			slot = SoftwareResetData::numberOfSlots - 1;
			while (slot >= 0 && srdBuf[slot].magic == 0xFFFF)
			{
				--slot;
			}
		}

		Message(mtype, "Last software reset code: ");
		if (slot >= 0 && srdBuf[slot].magic == SoftwareResetData::magicValue)
		{
			MessageF(mtype, "0x%04x, available RAM %u bytes (slot %d)\n", srdBuf[slot].resetReason, srdBuf[slot].neverUsedRam, slot);
			MessageF(mtype, "Spinning module during software reset: %s\n", moduleName[srdBuf[slot].resetReason & 0x0F]);
		}
		else
		{
			Message(mtype, "not available\n");
		}
	}

	// Show the current error codes
	MessageF(mtype, "Error status: %u\n", errorCodeBits);

	// Show the number of free entries in the file table
	unsigned int numFreeFiles = 0;
	for (size_t i = 0; i < MAX_FILES; i++)
	{
		if (!files[i]->inUse)
		{
			++numFreeFiles;
		}
	}
	MessageF(mtype, "Free file entries: %u\n", numFreeFiles);

	// Show the HSMCI CD pin and speed
#ifdef __RADDS__
	MessageF(mtype, "SD card 0 %s\n", (sd_mmc_card_detected(0) ? "detected" : "not detected"));
#else
	MessageF(mtype, "SD card 0 %s, interface speed: %.1fMBytes/sec\n", (sd_mmc_card_detected(0) ? "detected" : "not detected"), (float)hsmci_get_speed()/1000000.0);
#endif

	// Show the longest SD card write time
	MessageF(mtype, "SD card longest block write time: %.1fms\n", FileStore::GetAndClearLongestWriteTime());

#ifndef __RADDS__
	// Show the MCU temperatures
	MessageF(mtype, "MCU temperature: min %.1f, current %.1f, max %.1f\n",
				AdcReadingToCpuTemperature(lowestMcuTemperature), AdcReadingToCpuTemperature(currentMcuTemperature), AdcReadingToCpuTemperature(highestMcuTemperature));
	lowestMcuTemperature = highestMcuTemperature = currentMcuTemperature;
#endif

#ifdef DUET_NG
	// Show the supply voltage
	MessageF(mtype, "Supply voltage: min %.1f, current %.1f, max %.1f, under voltage events: %u, over voltage events: %u\n",
				AdcReadingToPowerVoltage(lowestVin), AdcReadingToPowerVoltage(currentVin), AdcReadingToPowerVoltage(highestVin),
				numUnderVoltageEvents, numOverVoltageEvents);
	lowestVin = highestVin = currentVin;

	// Show the motor stall status
	if (expansionBoard == ExpansionBoardType::DueX2 || expansionBoard == ExpansionBoardType::DueX5)
	{
		const bool stalled = digitalRead(DueX_SG);
		MessageF(mtype, "Expansion motor(s) stall indication: %s\n", (stalled) ? "yes" : "no");
	}

	for (size_t drive = 0; drive < numTMC2660Drivers; ++drive)
	{
		const uint32_t stat = TMC2660::GetStatus(drive);
		MessageF(mtype, "Driver %d:%s%s%s%s%s%s\n", drive,
						(stat & TMC_RR_SG) ? " stalled" : "",
						(stat & TMC_RR_OT) ? " temperature-shutdown!"
							: (stat & TMC_RR_OTPW) ? " temperature-warning" : "",
						(stat & TMC_RR_S2G) ? " short-to-ground" : "",
						((stat & TMC_RR_OLA) && !(stat & TMC_RR_STST)) ? " open-load-A" : "",
						((stat & TMC_RR_OLB) && !(stat & TMC_RR_STST)) ? " open-load-B" : "",
						(stat & TMC_RR_STST) ? " standstill"
							: (stat & (TMC_RR_SG | TMC_RR_OT | TMC_RR_OTPW | TMC_RR_S2G | TMC_RR_OLA | TMC_RR_OLB))
							  ? "" : " ok"
				);
	}
#endif

	// Show current RTC time
	Message(mtype, "Current date and time: ");
	struct tm timeInfo;
	if (gmtime_r(&realTime, &timeInfo) != nullptr)
	{
		MessageF(mtype, "%04u-%02u-%02u %02u:%02u:%02u\n",
				timeInfo.tm_year + 1900, timeInfo.tm_mon + 1, timeInfo.tm_mday,
				timeInfo.tm_hour, timeInfo.tm_min, timeInfo.tm_sec);
	}
	else
	{
		Message(mtype, "clock not set\n");
	}

// Debug
//MessageF(mtype, "TC_FMR = %08x, PWM_FPE = %08x, PWM_FSR = %08x\n", TC2->TC_FMR, PWM->PWM_FPE, PWM->PWM_FSR);
//MessageF(mtype, "PWM2 period %08x, duty %08x\n", PWM->PWM_CH_NUM[2].PWM_CPRD, PWM->PWM_CH_NUM[2].PWM_CDTY);
//MessageF(mtype, "Shortest/longest times read %.1f/%.1f write %.1f/%.1f ms, %u/%u\n",
//		(float)shortestReadWaitTime/1000, (float)longestReadWaitTime/1000, (float)shortestWriteWaitTime/1000, (float)longestWriteWaitTime/1000,
//		maxRead, maxWrite);
//longestWriteWaitTime = longestReadWaitTime = 0; shortestReadWaitTime = shortestWriteWaitTime = 1000000;

	reprap.Timing(mtype);

#ifdef MOVE_DEBUG
	MessageF(mtype, "Interrupts scheduled %u, done %u, last %u, next %u sched at %u, now %u\n",
			numInterruptsScheduled, numInterruptsExecuted, lastInterruptTime, nextInterruptTime, nextInterruptScheduledAt, GetInterruptClocks());
#endif
}

void Platform::DiagnosticTest(int d)
{
	switch (d)
	{
	case (int)DiagnosticTestType::TestWatchdog:
		SysTick ->CTRL &= ~(SysTick_CTRL_TICKINT_Msk);	// disable the system tick interrupt so that we get a watchdog timeout reset
		break;

	case (int)DiagnosticTestType::TestSpinLockup:
		debugCode = d;									// tell the Spin function to loop
		break;

	case (int)DiagnosticTestType::TestSerialBlock:		// write an arbitrary message via debugPrintf()
		debugPrintf("Diagnostic Test\n");
		break;

	case (int)DiagnosticTestType::PrintMoves:
		DDA::PrintMoves();
		break;

#ifdef DUET_NG
	case (int)DiagnosticTestType::PrintExpanderStatus:
		MessageF(GENERIC_MESSAGE, "Expander status %04X\n", DuetExpansion::DiagnosticRead());
		break;
#endif

	default:
		break;
	}
}

// Return the stack usage and amount of memory that has never been used, in bytes
void Platform::GetStackUsage(uint32_t* currentStack, uint32_t* maxStack, uint32_t* neverUsed) const
{
	const char * const ramend = (const char *)
#if SAM4E
			IRAM_ADDR + IRAM_SIZE;
#else
			IRAM0_ADDR + IRAM_SIZE;
#endif
	register const char * stack_ptr asm ("sp");
	const char * const heapend = sbrk(0);
	const char * stack_lwm = heapend;
	while (stack_lwm < stack_ptr && *stack_lwm == memPattern)
	{
		++stack_lwm;
	}
	if (currentStack != nullptr) { *currentStack = ramend - stack_ptr; }
	if (maxStack != nullptr) { *maxStack = ramend - stack_lwm; }
	if (neverUsed != nullptr) { *neverUsed = stack_lwm - heapend; }
}

void Platform::ClassReport(float &lastTime)
{
	const Module spinningModule = reprap.GetSpinningModule();
	if (reprap.Debug(spinningModule))
	{
		if (Time() - lastTime >= LONG_TIME)
		{
			lastTime = Time();
			MessageF(HOST_MESSAGE, "Class %s spinning.\n", moduleName[spinningModule]);
		}
	}
}

//===========================================================================
//=============================Thermal Settings  ============================
//===========================================================================

// See http://en.wikipedia.org/wiki/Thermistor#B_or_.CE.B2_parameter_equation

// BETA is the B value
// RS is the value of the series resistor in ohms
// R_INF is R0.exp(-BETA/T0), where R0 is the thermistor resistance at T0 (T0 is in kelvin)
// Normally T0 is 298.15K (25 C).

// If the A->D converter has a range of 0..1023 and the measured voltage is V (between 0 and 1023)
// then the thermistor resistance, R = V.RS/(1024 - V)
// and the temperature, T = BETA/ln(R/R_INF)
// To get degrees celsius (instead of kelvin) add -273.15 to T

// Result is in degrees celsius

float Platform::GetTemperature(size_t heater, TemperatureError& err)
{
	// Note that at this point we're actually getting an averaged ADC read, not a "raw" temp.  For thermistors,
	// we're getting an averaged voltage reading which we'll convert to a temperature.
	if (IsThermistorChannel(heater))
	{
		const volatile ThermistorAveragingFilter& filter = thermistorFilters[heater];
		if (filter.IsValid())
		{
			const int32_t averagedReading = filter.GetSum()/(ThermistorAverageReadings >> Thermistor::AdcOversampleBits);
			const float temp = thermistors[heater].CalcTemperature(averagedReading);

			if (temp < MinimumConnectedTemperature)
			{
				// thermistor is disconnected
				err = TemperatureError::openCircuit;
				return ABS_ZERO;
			}

			err = TemperatureError::success;
			return temp;
		}

		// Filter is not ready yet
		err = TemperatureError::busBusy;
		return BAD_ERROR_TEMPERATURE;
	}

	if (IsThermocoupleChannel(heater))
	{
		// MAX31855 thermocouple chip
		float temp;
		err = SpiTempSensors[heaterTempChannels[heater] - FirstThermocoupleChannel].GetThermocoupleTemperature(&temp);
		return (err == TemperatureError::success) ? temp : BAD_ERROR_TEMPERATURE;
	}

	if (IsRtdChannel(heater))
	{
		// MAX31865 RTD chip
		float temp;
		err = SpiTempSensors[heaterTempChannels[heater] - FirstRtdChannel].GetRtdTemperature(&temp);
		return (err == TemperatureError::success) ? temp : BAD_ERROR_TEMPERATURE;
	}

	err = TemperatureError::unknownChannel;
	return BAD_ERROR_TEMPERATURE;
}

// See if we need to turn on a thermostatically-controlled fan
bool Platform::AnyHeaterHot(uint16_t heaters, float t)
{
	for (size_t h = 0; h < HEATERS; ++h)
	{
		// Check if this heater is both monitored by this fan and in use
		if (   ((1 << h) & heaters) != 0
			&& (h < reprap.GetToolHeatersInUse() || (int)h == reprap.GetHeat()->GetBedHeater() || (int)h == reprap.GetHeat()->GetChamberHeater())
		   )
		{
			if (reprap.GetHeat()->IsTuning(h))
			{
				return true;			// when turning the PID for a monitored heater, turn the fan on
			}

			TemperatureError err;
			const float ht = GetTemperature(h, err);
			if (err != TemperatureError::success || ht >= t || ht < BAD_LOW_TEMPERATURE)
			{
				return true;
			}
		}
	}
	return false;
}

// Power is a fraction in [0,1]
void Platform::SetHeater(size_t heater, float power)
{
	if (heatOnPins[heater] != NoPin)
	{
		uint16_t freq = (reprap.GetHeat()->UseSlowPwm(heater)) ? SlowHeaterPwmFreq : NormalHeaterPwmFreq;
		WriteAnalog(heatOnPins[heater], (HEAT_ON) ? power : 1.0 - power, freq);
	}
}

void Platform::UpdateConfiguredHeaters()
{
	configuredHeaters = 0;

	// Check bed heater
	const int8_t bedHeater = reprap.GetHeat()->GetBedHeater();
	if (bedHeater >= 0)
	{
		configuredHeaters |= (1 << bedHeater);
	}

	// Check chamber heater
	const int8_t chamberHeater = reprap.GetHeat()->GetChamberHeater();
	if (chamberHeater >= 0)
	{
		configuredHeaters |= (1 << chamberHeater);
	}

	// Check tool heaters
	for(size_t heater = 0; heater < HEATERS; heater++)
	{
		if (reprap.IsHeaterAssignedToTool(heater))
		{
			configuredHeaters |= (1 << heater);
		}
	}
}

EndStopHit Platform::Stopped(size_t drive) const
{
	if (drive < DRIVES && endStopPins[drive] != NoPin)
	{
		if (drive >= reprap.GetGCodes()->GetNumAxes())
		{
			// Endstop not used for an axis, so no configuration data available.
			// To allow us to see its status in DWC, pretend it is configured as a high-end active high endstop.
			if (ReadPin(endStopPins[drive]))
			{
				return EndStopHit::highHit;
			}
		}
		else if (endStopType[drive] == EndStopType::noEndStop)
		{
			// No homing switch is configured for this axis, so see if we should use the Z probe
			if (zProbeType > 0 && drive < reprap.GetGCodes()->GetNumAxes() && (zProbeAxes & (1 << drive)) != 0)
			{
				return GetZProbeResult();			// using the Z probe as a low homing stop for this axis, so just get its result
			}
		}
		else if (ReadPin(endStopPins[drive]) == endStopLogicLevel[drive])
		{
			return (endStopType[drive] == EndStopType::highEndStop) ? EndStopHit::highHit : EndStopHit::lowHit;
		}
	}
	return EndStopHit::noStop;
}

// Get the statues of all the endstop inputs, regardless of what they are used for. Used for triggers.
uint32_t Platform::GetAllEndstopStates() const
{
	uint32_t rslt = 0;
	for (unsigned int drive = 0; drive < DRIVES; ++drive)
	{
		const Pin pin = endStopPins[drive];
		if (pin != NoPin && ReadPin(pin))
		{
			rslt |= (1 << drive);
		}
	}
	return rslt;
}

// Return the Z probe result. We assume that if the Z probe is used as an endstop, it is used as the low stop.
EndStopHit Platform::GetZProbeResult() const
{
	const int zProbeVal = GetZProbeReading();
	const int zProbeADValue = GetCurrentZProbeParameters().adcValue;
	return (zProbeVal >= zProbeADValue) ? EndStopHit::lowHit
			: (zProbeVal * 10 >= zProbeADValue * 9) ? EndStopHit::lowNear	// if we are at/above 90% of the target value
				: EndStopHit::noStop;
}

// Write the Z probe parameters to file
bool Platform::WriteZProbeParameters(FileStore *f) const
{
	bool ok = f->Write("; Z probe parameters\n");
	if (ok)
	{
		ok = irZProbeParameters.WriteParameters(f, 1);
	}
	if (ok)
	{
		ok = alternateZProbeParameters.WriteParameters(f, 3);
	}
	if (ok)
	{
		ok = switchZProbeParameters.WriteParameters(f, 4);
	}
	return ok;
}

// This is called from the step ISR as well as other places, so keep it fast
void Platform::SetDirection(size_t drive, bool direction)
{
	const size_t numAxes = reprap.GetGCodes()->GetNumAxes();
	if (drive < numAxes)
	{
		for (size_t i = 0; i < axisDrivers[drive].numDrivers; ++i)
		{
			SetDriverDirection(axisDrivers[drive].driverNumbers[i], direction);
		}
	}
	else if (drive < DRIVES)
	{
		SetDriverDirection(extruderDrivers[drive - numAxes], direction);
	}
}

// Enable a driver. Must not be called from an ISR, or with interrupts disabled.
void Platform::EnableDriver(size_t driver)
{
	if (driver < DRIVES && driverState[driver] != DriverStatus::enabled)
	{
		driverState[driver] = DriverStatus::enabled;
		UpdateMotorCurrent(driver);						// the current may have been reduced by the idle timeout

#if defined(DUET_NG)
		if (driver < numTMC2660Drivers)
		{
			TMC2660::EnableDrive(driver, true);
		}
		else
		{
#endif
			digitalWrite(ENABLE_PINS[driver], enableValues[driver]);
#if defined(DUET_NG)
		}
#endif
	}
}

// Disable a driver
void Platform::DisableDriver(size_t driver)
{
	if (driver < DRIVES)
	{
#if defined(DUET_NG)
		if (driver < numTMC2660Drivers)
		{
			TMC2660::EnableDrive(driver, false);
		}
		else
		{
#endif
			digitalWrite(ENABLE_PINS[driver], !enableValues[driver]);
#if defined(DUET_NG)
		}
#endif
		driverState[driver] = DriverStatus::disabled;
	}
}

// Enable the drivers for a drive. Must not be called from an ISR, or with interrupts disabled.
void Platform::EnableDrive(size_t drive)
{
	const size_t numAxes = reprap.GetGCodes()->GetNumAxes();
	if (drive < numAxes)
	{
		for (size_t i = 0; i < axisDrivers[drive].numDrivers; ++i)
		{
			EnableDriver(axisDrivers[drive].driverNumbers[i]);
		}
	}
	else if (drive < DRIVES)
	{
		EnableDriver(extruderDrivers[drive - numAxes]);
	}
}

// Disable the drivers for a drive
void Platform::DisableDrive(size_t drive)
{
	const size_t numAxes = reprap.GetGCodes()->GetNumAxes();
	if (drive < numAxes)
	{
		for (size_t i = 0; i < axisDrivers[drive].numDrivers; ++i)
		{
			DisableDriver(axisDrivers[drive].driverNumbers[i]);
		}
	}
	else if (drive < DRIVES)
	{
		DisableDriver(extruderDrivers[drive - numAxes]);
	}
}

// Set drives to idle hold if they are enabled. If a drive is disabled, leave it alone.
// Must not be called from an ISR, or with interrupts disabled.
void Platform::SetDriversIdle()
{
	for (size_t driver = 0; driver < DRIVES; ++driver)
	{
		if (driverState[driver] == DriverStatus::enabled)
		{
			driverState[driver] = DriverStatus::idle;
			UpdateMotorCurrent(driver);
		}
	}
}

// Set the current for a drive. Current is in mA.
void Platform::SetDriverCurrent(size_t driver, float currentOrPercent, bool isPercent)
{
	if (driver < DRIVES)
	{
		if (isPercent)
		{
			motorCurrentFraction[driver] = 0.01 * currentOrPercent;
		}
		else
		{
			motorCurrents[driver] = currentOrPercent;
		}
		UpdateMotorCurrent(driver);
	}
}

// Set the current for all drivers on an axis or extruder. Current is in mA.
void Platform::SetMotorCurrent(size_t drive, float currentOrPercent, bool isPercent)
{
	const size_t numAxes = reprap.GetGCodes()->GetNumAxes();
	if (drive < numAxes)
	{
		for (size_t i = 0; i < axisDrivers[drive].numDrivers; ++i)
		{
			SetDriverCurrent(axisDrivers[drive].driverNumbers[i], currentOrPercent, isPercent);
		}

	}
	else if (drive < DRIVES)
	{
		SetDriverCurrent(extruderDrivers[drive - numAxes], currentOrPercent, isPercent);
	}
}

// This must not be called from an ISR, or with interrupts disabled.
void Platform::UpdateMotorCurrent(size_t driver)
{
	if (driver < DRIVES)
	{
		float current = motorCurrents[driver];
		if (driverState[driver] == DriverStatus::idle)
		{
			current *= idleCurrentFactor;
		}
		else
		{
			current *= motorCurrentFraction[driver];
		}

#if defined(DUET_NG)
		if (driver < numTMC2660Drivers)
		{
			TMC2660::SetCurrent(driver, current);
		}
		// else we can't set the current
#elif defined (__RADDS__)
		// we can't set the current on RADDS
#else
		unsigned short pot = (unsigned short)((0.256*current*8.0*senseResistor + maxStepperDigipotVoltage/2)/maxStepperDigipotVoltage);
		if (driver < 4)
		{
			mcpDuet.setNonVolatileWiper(potWipes[driver], pot);
			mcpDuet.setVolatileWiper(potWipes[driver], pot);
		}
		else
		{
# ifndef DUET_NG
			if (board == BoardType::Duet_085)
			{
# endif
				// Extruder 0 is on DAC channel 0
				if (driver == 4)
				{
					const float dacVoltage = max<float>(current * 0.008 * senseResistor + stepperDacVoltageOffset, 0.0);	// the voltage we want from the DAC relative to its minimum
					const float dac = dacVoltage/stepperDacVoltageRange;
# ifdef DUET_NG
					AnalogOut(DAC1, dac);
# else
					AnalogOut(DAC0, dac);
# endif
				}
				else
				{
					mcpExpansion.setNonVolatileWiper(potWipes[driver-1], pot);
					mcpExpansion.setVolatileWiper(potWipes[driver-1], pot);
				}
# ifndef DUET_NG
			}
			else if (driver < 8)		// on a Duet 0.6 we have a maximum of 8 drives
			{
				mcpExpansion.setNonVolatileWiper(potWipes[driver], pot);
				mcpExpansion.setVolatileWiper(potWipes[driver], pot);
			}
# endif
		}
#endif
	}
}

// Get the configured motor current for a drive.
// Currently we don't allow multiple motors on a single axis to have different currents, so we can just return the current for the first one.
float Platform::GetMotorCurrent(size_t drive, bool isPercent) const
{
	if (drive < DRIVES)
	{
		const size_t numAxes = reprap.GetGCodes()->GetNumAxes();
		const uint8_t driver = (drive < numAxes) ? axisDrivers[drive].driverNumbers[0] : extruderDrivers[drive - numAxes];
		if (driver < DRIVES)
		{
			return (isPercent) ? motorCurrentFraction[driver] * 100.0 : motorCurrents[driver];
		}
	}
	return 0.0;
}

// Set the motor idle current factor
void Platform::SetIdleCurrentFactor(float f)
{
	idleCurrentFactor = f;
	for (size_t driver = 0; driver < DRIVES; ++driver)
	{
		if (driverState[driver] == DriverStatus::idle)
		{
			UpdateMotorCurrent(driver);
		}
	}
}

// Set the microstepping for a driver, returning true if successful
bool Platform::SetDriverMicrostepping(size_t driver, int microsteps, int mode)
{
	if (driver < DRIVES)
	{
#if defined(DUET_NG)
		if (driver < numTMC2660Drivers)
		{
			return TMC2660::SetMicrostepping(driver, microsteps, mode);
		}
		else
		{
# endif
			// Other drivers only support x16 microstepping.
			// We ignore the interpolation on/off parameter so that e.g. M350 I1 E16:128 won't give an error if E1 supports interpolation but E0 doesn't.
			return microsteps == 16;
#if defined(DUET_NG)
		}
#endif
	}
	return false;
}

// Set the microstepping, returning true if successful. All drivers for the same axis must use the same microstepping.
bool Platform::SetMicrostepping(size_t drive, int microsteps, int mode)
{
	const size_t numAxes = reprap.GetGCodes()->GetNumAxes();
	if (drive < numAxes)
	{
		bool ok = true;
		for (size_t i = 0; i < axisDrivers[drive].numDrivers; ++i)
		{
			ok = SetDriverMicrostepping(axisDrivers[drive].driverNumbers[i], microsteps, mode) && ok;
		}
		return ok;
	}
	else if (drive < DRIVES)
	{
		return SetDriverMicrostepping(extruderDrivers[drive - numAxes], microsteps, mode);
	}
	return false;
}

// Get the microstepping for a driver
unsigned int Platform::GetDriverMicrostepping(size_t driver, bool& interpolation) const
{
#if defined(DUET_NG)
	if (driver < numTMC2660Drivers)
	{
		return TMC2660::GetMicrostepping(driver, interpolation);
	}
#endif
	// On-board drivers only support x16 microstepping without interpolation
	interpolation = false;
	return 16;
}

// Get the microstepping for an axis or extruder
unsigned int Platform::GetMicrostepping(size_t drive, bool& interpolation) const
{
	const size_t numAxes = reprap.GetGCodes()->GetNumAxes();
	if (drive < numAxes)
	{
		return GetDriverMicrostepping(axisDrivers[drive].driverNumbers[0], interpolation);
	}
	else if (drive < DRIVES)
	{
		return GetDriverMicrostepping(extruderDrivers[drive - numAxes], interpolation);
	}
	else
	{
		interpolation = false;
		return 16;
	}
}

void Platform::SetAxisDriversConfig(size_t drive, const AxisDriversConfig& config)
{
	axisDrivers[drive] = config;
	uint32_t bitmap = 0;
	for (size_t i = 0; i < config.numDrivers; ++i)
	{
		bitmap |= CalcDriverBitmap(config.driverNumbers[i]);
	}
	driveDriverBits[drive] = bitmap;
}

// Map an extruder to a driver
void Platform::SetExtruderDriver(size_t extruder, uint8_t driver)
{
	extruderDrivers[extruder] = driver;
	driveDriverBits[extruder + reprap.GetGCodes()->GetNumAxes()] = CalcDriverBitmap(driver);
}

void Platform::SetDriverStepTiming(size_t driver, float microseconds)
{
	if (microseconds < minStepPulseTiming)
	{
		slowDrivers &= ~CalcDriverBitmap(driver);						// this drive does not need extended timing
	}
	else
	{
		const uint32_t clocks = (uint32_t)(((float)DDA::stepClockRate * microseconds/1000000.0) + 0.99);	// convert microseconds to step clocks, rounding up
		if (clocks > slowDriverStepPulseClocks)
		{
			slowDriverStepPulseClocks = clocks;
		}
		slowDrivers |= CalcDriverBitmap(driver);						// this drive does need extended timing
	}
}

float Platform::GetDriverStepTiming(size_t driver) const
{
	return ((slowDrivers & CalcDriverBitmap(driver)) != 0)
			? (float)slowDriverStepPulseClocks * 1000000.0/(float)DDA::stepClockRate
			: 0.0;
}

// Get current cooling fan speed on a scale between 0 and 1
float Platform::GetFanValue(size_t fan) const
{
	return (fan < NUM_FANS) ? fans[fan].GetValue() : -1;
}

// This is a bit of a compromise - old RepRaps used fan speeds in the range
// [0, 255], which is very hardware dependent.  It makes much more sense
// to specify speeds in [0.0, 1.0].  This looks at the value supplied (which
// the G Code reader will get right for a float or an int) and attempts to
// do the right thing whichever the user has done.
void Platform::SetFanValue(size_t fan, float speed)
{
	if (fan < NUM_FANS)
	{
		fans[fan].SetValue(speed);
	}
}

#if !defined(DUET_NG) && !defined(__RADDS__)

// Enable or disable the fan that shares its PWM pin with the last heater. Called when we disable or enable the last heater.
void Platform::EnableSharedFan(bool enable)
{
	fans[NUM_FANS - 1].Init((enable) ? COOLING_FAN_PINS[NUM_FANS - 1] : NoPin, FansHardwareInverted());
}

#endif


// Get current fan RPM
float Platform::GetFanRPM()
{
	// The ISR sets fanInterval to the number of microseconds it took to get fanMaxInterruptCount interrupts.
	// We get 2 tacho pulses per revolution, hence 2 interrupts per revolution.
	// However, if the fan stops then we get no interrupts and fanInterval stops getting updated.
	// We must recognise this and return zero.
	return (fanInterval != 0 && micros() - fanLastResetTime < 3000000U)		// if we have a reading and it is less than 3 second old
			? (float)((30000000U * fanMaxInterruptCount)/fanInterval)		// then calculate RPM assuming 2 interrupts per rev
			: 0.0;															// else assume fan is off or tacho not connected
}

bool Platform::FansHardwareInverted() const
{
#if defined(DUET_NG) || defined(__RADDS__)
	return false;
#else
	// The cooling fan output pin gets inverted on a Duet 0.6 or 0.7
	return board == BoardType::Duet_06 || board == BoardType::Duet_07;
#endif
}

void Platform::InitFans()
{
	for (size_t i = 0; i < NUM_FANS; ++i)
	{
		fans[i].Init(COOLING_FAN_PINS[i], FansHardwareInverted());
	}

	if (NUM_FANS > 1)
	{
#ifdef DUET_NG
		// Set fan 1 to be thermostatic by default, monitoring all heaters except the default bed heater
		fans[1].SetHeatersMonitored(0xFFFF & ~(1 << DefaultBedHeater));
		fans[1].SetValue(1.0);												// set it full on
#else
		// Fan 1 on the Duet 0.8.5 shares its control pin with heater 6. Set it full on to make sure the heater (if present) is off.
		fans[1].SetValue(1.0);												// set it full on
#endif
	}

	coolingFanRpmPin = COOLING_FAN_RPM_PIN;
	lastRpmResetTime = 0.0;
	if (coolingFanRpmPin != NoPin)
	{
		pinModeDuet(coolingFanRpmPin, INPUT_PULLUP, 1500);	// enable pullup and 1500Hz debounce filter (500Hz only worked up to 7000RPM)
	}
}

void Platform::SetMACAddress(uint8_t mac[])
{
	for (size_t i = 0; i < 6; i++)
	{
		macAddress[i] = mac[i];
	}
}

//-----------------------------------------------------------------------------------------------------

FileStore* Platform::GetFileStore(const char* directory, const char* fileName, bool write)
{
	if (!fileStructureInitialised)
	{
		return nullptr;
	}

	for (size_t i = 0; i < MAX_FILES; i++)
	{
		if (!files[i]->inUse)
		{
			if (files[i]->Open(directory, fileName, write))
			{
				files[i]->inUse = true;
				return files[i];
			}
			else
			{
				return nullptr;
			}
		}
	}
	Message(HOST_MESSAGE, "Max open file count exceeded.\n");
	return NULL;
}

void Platform::AppendAuxReply(const char *msg)
{
	// Discard this response if either no aux device is attached or if the response is empty
	if (msg[0] != 0 && HaveAux())
	{
		if (msg[0] == '{')
		{
			// JSON responses are always sent directly to the AUX device
			OutputBuffer *buf;
			if (OutputBuffer::Allocate(buf))
			{
				buf->copy(msg);
				auxOutput->Push(buf);
			}
		}
		else
		{
			// Regular text-based responses for AUX are currently stored and processed by M105/M408
			if (auxGCodeReply != nullptr || OutputBuffer::Allocate(auxGCodeReply))
			{
				auxSeq++;
				auxGCodeReply->cat(msg);
			}
		}
	}
}

void Platform::AppendAuxReply(OutputBuffer *reply)
{
	// Discard this response if either no aux device is attached or if the response is empty
	if (reply == nullptr || reply->Length() == 0 || !HaveAux())
	{
		OutputBuffer::ReleaseAll(reply);
	}
	else if ((*reply)[0] == '{')
	{
		// JSON responses are always sent directly to the AUX device
		// For big responses it makes sense to write big chunks of data in portions. Store this data here
		auxOutput->Push(reply);
	}
	else
	{
		// Other responses are stored for M105/M408
		auxSeq++;
		if (auxGCodeReply == nullptr)
		{
			auxGCodeReply = reply;
		}
		else
		{
			auxGCodeReply->Append(reply);
		}
	}
}

void Platform::Message(MessageType type, const char *message)
{
	switch (type)
	{
		case AUX_MESSAGE:
			AppendAuxReply(message);
			break;

		case AUX2_MESSAGE:
#ifdef SERIAL_AUX2_DEVICE
			// Message that is to be sent to the second auxiliary device (blocking)
			if (!aux2Output->IsEmpty())
			{
				// If we're still busy sending a response to the USART device, append this message to the output buffer
				aux2Output->GetLastItem()->cat(message);
			}
			else
			{
				// Send short strings immediately through the aux channel. There is no flow control on this port, so it can't block for long
				SERIAL_AUX2_DEVICE.write(message);
				SERIAL_AUX2_DEVICE.flush();
			}
#endif
			break;

		case DEBUG_MESSAGE:
			// Debug messages in blocking mode - potentially DANGEROUS, use with care!
			SERIAL_MAIN_DEVICE.write(message);
			SERIAL_MAIN_DEVICE.flush();
			break;

		case HOST_MESSAGE:
			// Message that is to be sent via the USB line (non-blocking)
			{
				// Ensure we have a valid buffer to write to that isn't referenced for other destinations
				OutputBuffer *usbOutputBuffer = usbOutput->GetLastItem();
				if (usbOutputBuffer == nullptr || usbOutputBuffer->IsReferenced())
				{
					if (!OutputBuffer::Allocate(usbOutputBuffer))
					{
						// Should never happen
						return;
					}
					usbOutput->Push(usbOutputBuffer);
				}

				// Append the message string
				usbOutputBuffer->cat(message);
			}
			break;

		case HTTP_MESSAGE:
		case TELNET_MESSAGE:
			// Message that is to be sent to the web
			{
				const WebSource source = (type == HTTP_MESSAGE) ? WebSource::HTTP : WebSource::Telnet;
				reprap.GetWebserver()->HandleGCodeReply(source, message);
			}
			break;

		case FIRMWARE_UPDATE_MESSAGE:
			Message(HOST_MESSAGE, message);			// send message to USB
			SendMessage(message);					// send message to aux
			break;

		case GENERIC_MESSAGE:
			// Message that is to be sent to the web & host. Make this the default one, too.
		default:
			Message(HTTP_MESSAGE, message);
			Message(TELNET_MESSAGE, message);
			Message(HOST_MESSAGE, message);
			Message(AUX_MESSAGE, message);
			break;
	}
}

void Platform::Message(const MessageType type, OutputBuffer *buffer)
{
	switch (type)
	{
		case AUX_MESSAGE:
			AppendAuxReply(buffer);
			break;

		case AUX2_MESSAGE:
			// Send this message to the second UART device
			aux2Output->Push(buffer);
			break;

		case DEBUG_MESSAGE:
			// Probably rarely used, but supported.
			while (buffer != nullptr)
			{
				SERIAL_MAIN_DEVICE.write(buffer->Data(), buffer->DataLength());
				SERIAL_MAIN_DEVICE.flush();

				buffer = OutputBuffer::Release(buffer);
			}
			break;

		case HOST_MESSAGE:
			if (!SERIAL_MAIN_DEVICE)
			{
				// If the serial USB line is not open, discard the message right away
				OutputBuffer::ReleaseAll(buffer);
			}
			else
			{
				// Else append incoming data to the stack
				usbOutput->Push(buffer);
			}
			break;

		case HTTP_MESSAGE:
		case TELNET_MESSAGE:
			// Message that is to be sent to the web
			{
				const WebSource source = (type == HTTP_MESSAGE) ? WebSource::HTTP : WebSource::Telnet;
				reprap.GetWebserver()->HandleGCodeReply(source, buffer);
			}
			break;

		case GENERIC_MESSAGE:
			// Message that is to be sent to the web & host.
			buffer->IncreaseReferences(3);		// This one is handled by three additional destinations
			Message(HTTP_MESSAGE, buffer);
			Message(TELNET_MESSAGE, buffer);
			Message(HOST_MESSAGE, buffer);
			Message(AUX_MESSAGE, buffer);
			break;

		case FIRMWARE_UPDATE_MESSAGE:
			// We don't generate any of these with an OutputBuffer argument, but if do we get one, just send it to USB
			Message(HOST_MESSAGE, buffer);
			break;

		default:
			// Everything else is unsupported (and probably not used)
			OutputBuffer::ReleaseAll(buffer);
			MessageF(HOST_MESSAGE, "Error: Unsupported Message call for type %u!\n", type);
			break;
	}
}

void Platform::MessageF(MessageType type, const char *fmt, va_list vargs)
{
	char formatBuffer[FORMAT_STRING_LENGTH];
	StringRef formatString(formatBuffer, ARRAY_SIZE(formatBuffer));
	formatString.vprintf(fmt, vargs);

	Message(type, formatBuffer);
}

void Platform::MessageF(MessageType type, const char *fmt, ...)
{
	char formatBuffer[FORMAT_STRING_LENGTH];
	StringRef formatString(formatBuffer, ARRAY_SIZE(formatBuffer));

	va_list vargs;
	va_start(vargs, fmt);
	formatString.vprintf(fmt, vargs);
	va_end(vargs);

	Message(type, formatBuffer);
}

bool Platform::AtxPower() const
{
	return ReadPin(ATX_POWER_PIN);
}

void Platform::SetAtxPower(bool on)
{
	WriteDigital(ATX_POWER_PIN, on);
}


void Platform::SetPressureAdvance(size_t extruder, float factor)
{
	if (extruder < MaxExtruders)
	{
		pressureAdvance[extruder] = factor;
	}
}

float Platform::ActualInstantDv(size_t drive) const
{
	const float idv = instantDvs[drive];
	const size_t numAxes = reprap.GetGCodes()->GetNumAxes();
	if (drive >= numAxes)
	{
		const float eComp = pressureAdvance[drive - numAxes];
		// If we are using pressure advance then we need to limit the extruder instantDv to avoid velocity mismatches.
		// Assume that we want the extruder motor position to be accurate to within 0.01mm of extrusion.
		// TODO remove this limit and add/remove steps to the previous and/or next move instead
		return (eComp <= 0.0) ? idv : min<float>(idv, 0.01/eComp);
	}
	else
	{
		return idv;
	}
}

void Platform::SetBaudRate(size_t chan, uint32_t br)
{
	if (chan < NUM_SERIAL_CHANNELS)
	{
		baudRates[chan] = br;
		ResetChannel(chan);
	}
}

uint32_t Platform::GetBaudRate(size_t chan) const
{
	return (chan < NUM_SERIAL_CHANNELS) ? baudRates[chan] : 0;
}

void Platform::SetCommsProperties(size_t chan, uint32_t cp)
{
	if (chan < NUM_SERIAL_CHANNELS)
	{
		commsParams[chan] = cp;
		ResetChannel(chan);
	}
}

uint32_t Platform::GetCommsProperties(size_t chan) const
{
	return (chan < NUM_SERIAL_CHANNELS) ? commsParams[chan] : 0;
}

// Re-initialise a serial channel.
// Ideally, this would be part of the Line class. However, the Arduino core inexplicably fails to make the serial I/O begin() and end() members
// virtual functions of a base class, which makes that difficult to do.
void Platform::ResetChannel(size_t chan)
{
	switch(chan)
	{
	case 0:
		SERIAL_MAIN_DEVICE.end();
		SERIAL_MAIN_DEVICE.begin(baudRates[0]);
		break;
	case 1:
		SERIAL_AUX_DEVICE.end();
		SERIAL_AUX_DEVICE.begin(baudRates[1]);
		break;
#ifdef SERIAL_AUX2_DEVICE
	case 2:
		SERIAL_AUX2_DEVICE.end();
		SERIAL_AUX2_DEVICE.begin(baudRates[2]);
		break;
#endif
	default:
		break;
	}
}

void Platform::SetBoardType(BoardType bt)
{
	if (bt == BoardType::Auto)
	{
#ifdef DUET_NG
		board = BoardType::DuetWiFi_10;
#elif defined(__RADDS__)
		board = BoardType::RADDS_15;
#else
		// Determine whether this is a Duet 0.6 or a Duet 0.8.5 board.
		// If it is a 0.85 board then DAC0 (AKA digital pin 67) is connected to ground via a diode and a 2.15K resistor.
		// So we enable the pullup (value 100K-150K) on pin 67 and read it, expecting a LOW on a 0.8.5 board and a HIGH on a 0.6 board.
		// This may fail if anyone connects a load to the DAC0 pin on a Duet 0.6, hence we implement board selection in M115 as well.
		pinMode(Dac0DigitalPin, INPUT_PULLUP);
		board = (digitalRead(Dac0DigitalPin)) ? BoardType::Duet_06 : BoardType::Duet_085;
		pinMode(Dac0DigitalPin, INPUT);			// turn pullup off
#endif
	}
	else
	{
		board = bt;
	}

	if (active)
	{
		InitZProbe();							// select and initialise the Z probe modulation pin
		InitFans();								// select whether cooling is inverted or not
	}
}

// Get a string describing the electronics
const char* Platform::GetElectronicsString() const
{
	switch (board)
	{
#ifdef DUET_NG
	case BoardType::DuetWiFi_10:			return "Duet WiFi 1.0";
#elif defined(__RADDS__)
	case BoardType::RADDS_15:				return "RADDS 1.5";
#else
	case BoardType::Duet_06:				return "Duet 0.6";
	case BoardType::Duet_07:				return "Duet 0.7";
	case BoardType::Duet_085:				return "Duet 0.85";
#endif
	default:								return "Unidentified";
	}
}

// Get the board string
const char* Platform::GetBoardString() const
{
	switch (board)
	{
#ifdef DUET_NG
	case BoardType::DuetWiFi_10:			return "duetwifi10";
#elif defined(__RADDS__)
	case BoardType::RADDS_15:				return "radds15";
#else
	case BoardType::Duet_06:				return "duet06";
	case BoardType::Duet_07:				return "duet07";
	case BoardType::Duet_085:				return "duet085";
#endif
	default:								return "unknown";
	}
}

// User I/O and servo support
bool Platform::GetFirmwarePin(int logicalPin, PinAccess access, Pin& firmwarePin, bool& invert)
{
	firmwarePin = NoPin;										// assume failure
	invert = false;												// this is the common case
	if (logicalPin < 0 || logicalPin > HighestLogicalPin)
	{
		// Pin number out of range, so nothing to do here
	}
	else if (logicalPin >= Heater0LogicalPin && logicalPin < Heater0LogicalPin + HEATERS)		// pins 0-9 correspond to heater channels
	{
		// For safety, we don't allow a heater channel to be used for servos until the heater has been disabled
		if (!reprap.GetHeat()->IsHeaterEnabled(logicalPin - Heater0LogicalPin))
		{
			firmwarePin = heatOnPins[logicalPin - Heater0LogicalPin];
			invert = !HEAT_ON;
		}
	}
	else if (logicalPin >= Fan0LogicalPin && logicalPin < Fan0LogicalPin + (int)NUM_FANS)		// pins 20- correspond to fan channels
	{
		// Don't allow a fan channel to be used unless the fan has been disabled
		if (!fans[logicalPin - Fan0LogicalPin].IsEnabled()
#ifdef DUET_NG
			// Fan pins on the DueX2/DueX5 cannot be used to control servos because the frequency is not well-defined.
			&& (logicalPin <= Fan0LogicalPin + 2 || access != PinAccess::servo)
#endif
		   )
		{
			firmwarePin = COOLING_FAN_PINS[logicalPin - Fan0LogicalPin];
#if !defined(DUET_NG) && !defined(__RADDS__)
			invert = (board == BoardType::Duet_06 || board == BoardType::Duet_07);
#endif
		}
	}
	else if (logicalPin >= EndstopXLogicalPin && logicalPin < EndstopXLogicalPin + (int)ARRAY_SIZE(endStopPins))	// pins 40-49 correspond to endstop pins
	{
		if (access == PinAccess::read
#ifdef DUET_NG
			// Endstop pins on the DueX2/DueX5 can be used as digital outputs too
			|| (access == PinAccess::write && logicalPin >= EndstopXLogicalPin + 5)
#endif
		   )
		{
			firmwarePin = endStopPins[logicalPin - EndstopXLogicalPin];
		}
	}
	else if (logicalPin >= Special0LogicalPin && logicalPin < Special0LogicalPin + (int)ARRAY_SIZE(SpecialPinMap))
	{
		firmwarePin = SpecialPinMap[logicalPin - Special0LogicalPin];
	}
#ifdef DUET_NG
	else if (logicalPin >= DueX5Gpio0LogicalPin && logicalPin < DueX5Gpio0LogicalPin + (int)ARRAY_SIZE(DueX5GpioPinMap))	// Pins 100-103 are the GPIO pins on the DueX2/X5
	{
		if (access != PinAccess::servo)
		{
			firmwarePin = DueX5GpioPinMap[logicalPin - DueX5Gpio0LogicalPin];
		}
	}
#endif

	if (firmwarePin != NoPin)
	{
		// Check that the pin mode has been defined suitably
		PinMode desiredMode;
		if (access == PinAccess::write)
		{
			desiredMode = (invert) ? OUTPUT_HIGH : OUTPUT_LOW;
		}
		else if (access == PinAccess::pwm || access == PinAccess::servo)
		{
			desiredMode = (invert) ? OUTPUT_PWM_HIGH : OUTPUT_PWM_LOW;
		}
		else
		{
			desiredMode = INPUT_PULLUP;
		}
		if (logicalPinModes[logicalPin] != (int8_t)desiredMode)
		{
			SetPinMode(firmwarePin, desiredMode);
			logicalPinModes[logicalPin] = (int8_t)desiredMode;
		}
		return true;
	}
	return false;
}

bool Platform::SetExtrusionAncilliaryPwmPin(int logicalPin)
{
	return GetFirmwarePin(logicalPin, PinAccess::pwm, extrusionAncilliaryPwmFirmwarePin, extrusionAncilliaryPwmInvert);
}

#if SUPPORT_INKJET

// Fire the inkjet (if any) in the given pattern
// If there is no inkjet, false is returned; if there is one this returns true
// So you can test for inkjet presence with if(platform->Inkjet(0))
bool Platform::Inkjet(int bitPattern)
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

bool Platform::GCodeAvailable(const SerialSource source) const
{
	switch (source)
	{
		case SerialSource::USB:
			return SERIAL_MAIN_DEVICE.available() > 0;

		case SerialSource::AUX:
			return SERIAL_AUX_DEVICE.available() > 0;

		case SerialSource::AUX2:
#ifdef SERIAL_AUX2_DEVICE
			return SERIAL_AUX2_DEVICE.available() > 0;
#else
			return false;
#endif
	}

	return false;
}

char Platform::ReadFromSource(const SerialSource source)
{
	switch (source)
	{
		case SerialSource::USB:
			return static_cast<char>(SERIAL_MAIN_DEVICE.read());

		case SerialSource::AUX:
			return static_cast<char>(SERIAL_AUX_DEVICE.read());

		case SerialSource::AUX2:
#ifdef SERIAL_AUX2_DEVICE
			return static_cast<char>(SERIAL_AUX2_DEVICE.read());
#else
			return 0;
#endif
	}

	return 0;
}

#ifndef __RADDS__
// CPU temperature
void Platform::GetMcuTemperatures(float& minT, float& currT, float& maxT) const
{
	minT = AdcReadingToCpuTemperature(lowestMcuTemperature);
	currT = AdcReadingToCpuTemperature(currentMcuTemperature);
	maxT = AdcReadingToCpuTemperature(highestMcuTemperature);
}
#endif

#ifdef DUET_NG
// Power in voltage
void Platform::GetPowerVoltages(float& minV, float& currV, float& maxV) const
{
	minV = AdcReadingToPowerVoltage(lowestVin);
	currV = AdcReadingToPowerVoltage(currentVin);
	maxV = AdcReadingToPowerVoltage(highestVin);
}
#endif

// Real-time clock

bool Platform::IsDateTimeSet() const
{
	return realTime != 0;
}

time_t Platform::GetDateTime() const
{
	return realTime;
}

bool Platform::SetDateTime(time_t time)
{
	struct tm brokenDateTime;
	const bool ok = (gmtime_r(&time, &brokenDateTime) != nullptr);
	if (ok)
	{
		realTime = time;
		timeLastUpdatedMillis = millis();
	}
	return ok;
}

bool Platform::SetDate(time_t date)
{
	// Check the validity of the date passed in
	struct tm brokenNewDate;
	const bool ok = (gmtime_r(&date, &brokenNewDate) != nullptr);
	if (ok)
	{
		struct tm brokenTimeNow;
		if (realTime == 0 || gmtime_r(&realTime, &brokenTimeNow) == nullptr)
		{
			// We didn't have a valid date/time set, so set the date and time to the value passed in
			realTime = date;
			timeLastUpdatedMillis = millis();
		}
		else
		{
			// Merge the existing time into the date passed in
			brokenNewDate.tm_hour = brokenTimeNow.tm_hour;
			brokenNewDate.tm_min = brokenTimeNow.tm_min;
			brokenNewDate.tm_sec = brokenTimeNow.tm_sec;
			realTime = mktime(&brokenNewDate);
		}
	}
	return ok;
}

bool Platform::SetTime(time_t time)
{
	// Check the validity of the date passed in
	struct tm brokenNewTime;
	const bool ok = (gmtime_r(&time, &brokenNewTime) != nullptr);
	if (ok)
	{
		struct tm brokenTimeNow;
		if (realTime == 0 || gmtime_r(&realTime, &brokenTimeNow) == nullptr)
		{
			// We didn't have a valid date/time set, so set the date and time to the value passed in
			realTime = time;
		}
		else
		{
			// Merge the new time into the current date/time
			brokenTimeNow.tm_hour = brokenNewTime.tm_hour;
			brokenTimeNow.tm_min = brokenNewTime.tm_min;
			brokenTimeNow.tm_sec = brokenNewTime.tm_sec;
			realTime = mktime(&brokenTimeNow);
		}
		timeLastUpdatedMillis = millis();
	}
	return ok;
}

// Pragma pop_options is not supported on this platform, so we put this time-critical code right at the end of the file
//#pragma GCC push_options
#pragma GCC optimize ("O3")

// Step pulse timer interrupt
void STEP_TC_HANDLER()
{
	STEP_TC->TC_CHANNEL[STEP_TC_CHAN].TC_IDR = TC_IER_CPAS;	// disable the interrupt
#ifdef MOVE_DEBUG
	++numInterruptsExecuted;
	lastInterruptTime = Platform::GetInterruptClocks();
#endif
	reprap.GetMove()->Interrupt();
}

// Schedule an interrupt at the specified clock count, or return true if that time is imminent or has passed already.
// Must be called with interrupts disabled,
/*static*/ bool Platform::ScheduleInterrupt(uint32_t tim)
{
	tc_write_ra(STEP_TC, STEP_TC_CHAN, tim);				// set up the compare register
	tc_get_status(STEP_TC, STEP_TC_CHAN);					// clear any pending interrupt
	int32_t diff = (int32_t)(tim - GetInterruptClocks());	// see how long we have to go
	if (diff < (int32_t)DDA::minInterruptInterval)			// if less than about 2us or already passed
	{
		return true;										// tell the caller to simulate an interrupt instead
	}

	STEP_TC->TC_CHANNEL[STEP_TC_CHAN].TC_IER = TC_IER_CPAS;	// enable the interrupt
#ifdef MOVE_DEBUG
		++numInterruptsScheduled;
		nextInterruptTime = tim;
		nextInterruptScheduledAt = Platform::GetInterruptClocks();
#endif
	return false;
}

// Make sure we get no step interrupts
/*static*/ void Platform::DisableStepInterrupt()
{
	STEP_TC->TC_CHANNEL[STEP_TC_CHAN].TC_IDR = TC_IER_CPAS;
}

// Process a 1ms tick interrupt
// This function must be kept fast so as not to disturb the stepper timing, so don't do any floating point maths in here.
// This is what we need to do:
// 1.  Kick off a new ADC conversion.
// 2.  Fetch and process the result of the last ADC conversion.
// 3a. If the last ADC conversion was for the Z probe, toggle the modulation output if using a modulated IR sensor.
// 3b. If the last ADC reading was a thermistor reading, check for an over-temperature situation and turn off the heater if necessary.
//     We do this here because the usual polling loop sometimes gets stuck trying to send data to the USB port.

//#define TIME_TICK_ISR	1		// define this to store the tick ISR time in errorCodeBits

void Platform::Tick()
{
#ifdef TIME_TICK_ISR
	uint32_t now = micros();
#endif

	if (tickState != 0)
	{
#ifdef DUET_NG
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
		if (driversPowered && currentVin > driverOverVoltageAdcReading)
		{
			TMC2660::SetDriversPowered(false);
			// We deliberately do not clear driversPowered here or increase the over voltage event count - we let the spin loop handle that
		}
#endif
	}

	switch (tickState)
	{
	case 1:			// last conversion started was a thermistor
	case 3:
		if (IsThermistorChannel(currentHeater))
		{
			// Because we are in the tick ISR and no other ISR reads the averaging filter, we can cast away 'volatile' here
			ThermistorAveragingFilter& currentFilter = const_cast<ThermistorAveragingFilter&>(thermistorFilters[currentHeater]);
			currentFilter.ProcessReading(AnalogInReadChannel(thermistorAdcChannels[heaterTempChannels[currentHeater]]));
		}

		// Guard against overly long delays between successive calls of PID::Spin().
		// Do not call Time() here, it isn't safe. We use millis() instead.
		if ((configuredHeaters & (1 << currentHeater)) != 0 && (millis() - reprap.GetHeat()->GetLastSampleTime(currentHeater)) > maxPidSpinDelay)
		{
			SetHeater(currentHeater, 0.0);
			LogError(ErrorCode::BadTemp);
		}

		++currentHeater;
		if (currentHeater == HEATERS)
		{
			currentHeater = 0;
		}
		++tickState;
		break;

	case 2:			// last conversion started was the Z probe, with IR LED on
		const_cast<ZProbeAveragingFilter&>(zProbeOnFilter).ProcessReading(GetRawZProbeReading());
		if (zProbeType == 2)									// if using a modulated IR sensor
		{
			digitalWrite(zProbeModulationPin, LOW);				// turn off the IR emitter
		}

		// Read the MCU temperature as well (no need to do it in every state)
#ifndef __RADDS__
		currentMcuTemperature = AnalogInReadChannel(temperatureAdcChannel);
#endif

		++tickState;
		break;

	case 4:			// last conversion started was the Z probe, with IR LED off if modulation is enabled
		const_cast<ZProbeAveragingFilter&>(zProbeOffFilter).ProcessReading(GetRawZProbeReading());
		// no break
	case 0:			// this is the state after initialisation, no conversion has been started
	default:
		if (zProbeType == 2)									// if using a modulated IR sensor
		{
			digitalWrite(zProbeModulationPin, HIGH);			// turn on the IR emitter
		}
		tickState = 1;
		break;
	}

	AnalogInStartConversion();

#ifdef TIME_TICK_ISR
	uint32_t now2 = micros();
	if (now2 - now > errorCodeBits)
	{
		errorCodeBits = now2 - now;
	}
#endif
}

// Pragma pop_options is not supported on this platform
//#pragma GCC pop_options

// End
