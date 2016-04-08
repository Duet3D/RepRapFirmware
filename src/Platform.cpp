/****************************************************************************************************

 RepRapFirmware - Platform: RepRapPro Ormerod with Arduino Due controller

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
#include "DueFlashStorage.h"

#include "sam/drivers/tc/tc.h"

#ifdef EXTERNAL_DRIVERS
# include "ExternalDrivers.h"
#endif

extern char _end;
extern "C" char *sbrk(int i);

const uint8_t memPattern = 0xA5;

static uint32_t fanInterruptCount = 0;				// accessed only in ISR, so no need to declare it volatile
const uint32_t fanMaxInterruptCount = 32;			// number of fan interrupts that we average over
static volatile uint32_t fanLastResetTime = 0;		// time (microseconds) at which we last reset the interrupt count, accessed inside and outside ISR
static volatile uint32_t fanInterval = 0;			// written by ISR, read outside the ISR

//#define MOVE_DEBUG

#ifdef MOVE_DEBUG
unsigned int numInterruptsScheduled = 0;
unsigned int numInterruptsExecuted = 0;
uint32_t nextInterruptTime = 0;
uint32_t nextInterruptScheduledAt = 0;
uint32_t lastInterruptTime = 0;
#endif

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
}

//*************************************************************************************************
// PidParameters class

bool PidParameters::UsePID() const
{
	return kP >= 0;
}

float PidParameters::GetThermistorR25() const
{
	return thermistorInfR * exp(thermistorBeta / (25.0 - ABS_ZERO));
}

void PidParameters::SetThermistorR25AndBeta(float r25, float beta)
{
	thermistorInfR = r25 * exp(-beta / (25.0 - ABS_ZERO));
	thermistorBeta = beta;
}

bool PidParameters::operator==(const PidParameters& other) const
{
	return kI == other.kI && kD == other.kD && kP == other.kP && kT == other.kT && kS == other.kS
			&& fullBand == other.fullBand && pidMin == other.pidMin
			&& pidMax == other.pidMax && thermistorBeta == other.thermistorBeta && thermistorInfR == other.thermistorInfR
			&& thermistorSeriesR == other.thermistorSeriesR && adcLowOffset == other.adcLowOffset
			&& adcHighOffset == other.adcHighOffset;
}

//*************************************************************************************************
// Platform class

/*static*/ const uint8_t Platform::pinAccessAllowed[NUM_PINS_ALLOWED/8] = PINS_ALLOWED;

Platform::Platform() :
		autoSaveEnabled(false), board(DEFAULT_BOARD_TYPE), active(false), errorCodeBits(0),
		fileStructureInitialised(false), tickState(0), debugCode(0)
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
	digitalWrite(ATX_POWER_PIN, LOW);		// ensure ATX power is off by default
	pinMode(ATX_POWER_PIN, OUTPUT);

	SetBoardType(BoardType::Auto);

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

	SERIAL_MAIN_DEVICE.begin(baudRates[0]);
	SERIAL_AUX_DEVICE.begin(baudRates[1]);		// this can't be done in the constructor because the Arduino port initialisation isn't complete at that point
#ifdef SERIAL_AUX2_DEVICE
	SERIAL_AUX2_DEVICE.begin(baudRates[2]);
#endif

	static_assert(sizeof(FlashData) + sizeof(SoftwareResetData) <= FLASH_DATA_LENGTH, "NVData too large");

	ResetNvData();

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

	mcpDuet.begin();							// only call begin once in the entire execution, this begins the I2C comms on that channel for all objects
	mcpExpansion.setMCP4461Address(0x2E);		// not required for mcpDuet, as this uses the default address

	// Directories

	sysDir = SYS_DIR;
	macroDir = MACRO_DIR;
	webDir = WEB_DIR;
	gcodeDir = GCODE_DIR;
	configFile = CONFIG_FILE;
	defaultFile = DEFAULT_FILE;

	// DRIVES

	ARRAY_INIT(stepPins, STEP_PINS);
	ARRAY_INIT(directionPins, DIRECTION_PINS);
	ARRAY_INIT(directions, DIRECTIONS);
	ARRAY_INIT(enableValues, ENABLE_VALUES);
	ARRAY_INIT(enablePins, ENABLE_PINS);
	ARRAY_INIT(endStopPins, END_STOP_PINS);
	ARRAY_INIT(maxFeedrates, MAX_FEEDRATES);
	ARRAY_INIT(accelerations, ACCELERATIONS);
	ARRAY_INIT(driveStepsPerUnit, DRIVE_STEPS_PER_UNIT);
	ARRAY_INIT(instantDvs, INSTANT_DVS);
	ARRAY_INIT(potWipes, POT_WIPES);
	senseResistor = SENSE_RESISTOR;
	maxStepperDigipotVoltage = MAX_STEPPER_DIGIPOT_VOLTAGE;
	maxStepperDACVoltage = MAX_STEPPER_DAC_VOLTAGE;

	// Z PROBE

	zProbePin = Z_PROBE_PIN;
	zProbeAdcChannel = PinToAdcChannel(zProbePin);
	InitZProbe();		// this also sets up zProbeModulationPin

	// AXES

	ARRAY_INIT(axisMaxima, AXIS_MAXIMA);
	ARRAY_INIT(axisMinima, AXIS_MINIMA);

	idleCurrentFactor = DEFAULT_IDLE_CURRENT_FACTOR;
	SetSlowestDrive();

	// HEATERS - Bed is assumed to be the first

	ARRAY_INIT(tempSensePins, TEMP_SENSE_PINS);
	ARRAY_INIT(heatOnPins, HEAT_ON_PINS);
	ARRAY_INIT(max31855CsPins, MAX31855_CS_PINS);

	configuredHeaters = (BED_HEATER >= 0) ? (1 << BED_HEATER) : 0;
	heatSampleTime = HEAT_SAMPLE_TIME;
	timeToHot = TIME_TO_HOT;

	// Motors

	for (size_t drive = 0; drive < DRIVES; drive++)
	{
		SetPhysicalDrive(drive, drive);					// map drivers directly to axes and extruders
		if (stepPins[drive] >= 0)
		{
			pinMode(stepPins[drive], OUTPUT);
		}
		if (directionPins[drive] >= 0)
		{
			pinMode(directionPins[drive], OUTPUT);
		}
#ifdef EXTERNAL_DRIVERS
		if (drive < FIRST_EXTERNAL_DRIVE && enablePins[drive] >= 0)
#else
		if (enablePins[drive] >= 0)
#endif
		{
			pinMode(enablePins[drive], OUTPUT);
		}
		if (endStopPins[drive] >= 0)
		{
			pinMode(endStopPins[drive], INPUT_PULLUP);
		}
		motorCurrents[drive] = 0.0;
		DisableDrive(drive);
		driveState[drive] = DriveStatus::disabled;
		SetElasticComp(drive, 0.0);
		if (drive <= AXES)
		{
			endStopType[drive] = (drive == Y_AXIS)
									? EndStopType::lowEndStop	// for Ormerod 2/Huxley/Mendel compatibility
									: EndStopType::noEndStop;	// for Ormerod/Huxley/Mendel compatibility
			endStopLogicLevel[drive] = true;					// assume all endstops use active high logic e.g. normally-closed switch to ground
		}
	}

#ifdef EXTERNAL_DRIVERS
	ExternalDrivers::Init();
#endif

	extrusionAncilliaryPWM = 0.0;

	// HEATERS - Bed is assumed to be index 0
	for (size_t heater = 0; heater < HEATERS; heater++)
	{
		if (heatOnPins[heater] >= 0)
		{
			digitalWrite(heatOnPins[heater], (HEAT_ON) ? LOW : HIGH);		// turn the heater off
			pinMode(heatOnPins[heater], OUTPUT);
		}
		AnalogChannelNumber chan = PinToAdcChannel(tempSensePins[heater]);	// translate the Arduino Due Analog pin number to the SAM ADC channel number
		thermistorAdcChannels[heater] = chan;
		AnalogInEnableChannel(chan, true);

		SetThermistorNumber(heater, heater);				// map the thermistor straight through
		thermistorFilters[heater].Init(0);
	}
	SetTemperatureLimit(DEFAULT_TEMPERATURE_LIMIT);

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
		pinMode(inkjetSerialOut, OUTPUT);
		digitalWrite(inkjetSerialOut, 0);

		inkjetShiftClock = INKJET_SHIFT_CLOCK;
		pinMode(inkjetShiftClock, OUTPUT);
		digitalWrite(inkjetShiftClock, LOW);

		inkjetStorageClock = INKJET_STORAGE_CLOCK;
		pinMode(inkjetStorageClock, OUTPUT);
		digitalWrite(inkjetStorageClock, LOW);

		inkjetOutputEnable = INKJET_OUTPUT_ENABLE;
		pinMode(inkjetOutputEnable, OUTPUT);
		digitalWrite(inkjetOutputEnable, HIGH);

		inkjetClear = INKJET_CLEAR;
		pinMode(inkjetClear, OUTPUT);
		digitalWrite(inkjetClear, HIGH);
	}
#endif

	// Clear the spare pin configuration
	memset(pinInitialised, 0, sizeof(pinInitialised));

	// Kick everything off
	lastTime = Time();
	longWait = lastTime;
	InitialiseInterrupts();		// also sets 'active' to true
}

void Platform::InvalidateFiles()
{
	for (size_t i = 0; i < MAX_FILES; i++)
	{
		files[i]->Init();
	}
}

void Platform::SetTemperatureLimit(float t)
{
	temperatureLimit = t;
	for (size_t heater = 0; heater < HEATERS; heater++)
	{
		// Calculate and store the ADC average sum that corresponds to an overheat condition, so that we can check it quickly in the tick ISR
		float thermistorOverheatResistance = nvData.pidParams[heater].GetRInf()
				* exp(-nvData.pidParams[heater].GetBeta() / (temperatureLimit - ABS_ZERO));
		float thermistorOverheatAdcValue = (AD_RANGE_REAL + 1) * thermistorOverheatResistance
				/ (thermistorOverheatResistance + nvData.pidParams[heater].thermistorSeriesR);
		thermistorOverheatSums[heater] = (uint32_t) (thermistorOverheatAdcValue + 0.9) * THERMISTOR_AVERAGE_READINGS;
	}
}

// Specify which thermistor channel a particular heater uses
void Platform::SetThermistorNumber(size_t heater, size_t thermistor)
//pre(heater < HEATERS && thermistor < HEATERS)
{
	heaterTempChannels[heater] = thermistor;

	// Initialize the associated MAX31855?
	if (thermistor >= MAX31855_START_CHANNEL)
	{
		Max31855Devices[thermistor - MAX31855_START_CHANNEL].Init(max31855CsPins[thermistor - MAX31855_START_CHANNEL]);
	}
}

int Platform::GetThermistorNumber(size_t heater) const
{
	return heaterTempChannels[heater];
}

void Platform::SetSlowestDrive()
{
	slowestDrive = 0;
	for (size_t drive = 1; drive < DRIVES; drive++)
	{
		if (ConfiguredInstantDv(drive) < ConfiguredInstantDv(slowestDrive))
		{
			slowestDrive = drive;
		}
	}
}

void Platform::InitZProbe()
{
	zProbeOnFilter.Init(0);
	zProbeOffFilter.Init(0);

#ifdef DUET_NG
	zProbeModulationPin = Z_PROBE_MOD_PIN;
#else
	zProbeModulationPin = (board == BoardType::Duet_07 || board == BoardType::Duet_085) ? Z_PROBE_MOD_PIN07 : Z_PROBE_MOD_PIN;
#endif

	switch (nvData.zProbeType)
	{
	case 1:
	case 2:
		AnalogInEnableChannel(zProbeAdcChannel, true);
		pinMode(zProbeModulationPin, OUTPUT);
		digitalWrite(zProbeModulationPin, HIGH);	// enable the IR LED
		break;

	case 3:
		AnalogInEnableChannel(zProbeAdcChannel, true);
		pinMode(zProbeModulationPin, OUTPUT);
		digitalWrite(zProbeModulationPin, LOW);		// enable the alternate sensor
		break;

	case 4:
		AnalogInEnableChannel(zProbeAdcChannel, false);
		pinMode(endStopPins[E0_AXIS], INPUT_PULLUP);
		break;

	case 5:
		AnalogInEnableChannel(zProbeAdcChannel, false);
		break;	//TODO (DeltaProbe)

	default:
		AnalogInEnableChannel(zProbeAdcChannel, false);
		break;
	}
}

// Return the Z probe data.
// The ADC readings are 12 bits, so we convert them to 10-bit readings for compatibility with the old firmware.
int Platform::ZProbe() const
{
	if (zProbeOnFilter.IsValid() && zProbeOffFilter.IsValid())
	{
		switch (nvData.zProbeType)
		{
		case 1:		// Simple or intelligent IR sensor
		case 3:		// Alternate sensor
		case 4:		// Mechanical Z probe
			return (int) ((zProbeOnFilter.GetSum() + zProbeOffFilter.GetSum()) / (8 * Z_PROBE_AVERAGE_READINGS));

		case 2:		// Modulated IR sensor.
			// We assume that zProbeOnFilter and zProbeOffFilter average the same number of readings.
			// Because of noise, it is possible to get a negative reading, so allow for this.
			return (int) (((int32_t) zProbeOnFilter.GetSum() - (int32_t) zProbeOffFilter.GetSum())
					/ (int)(4 * Z_PROBE_AVERAGE_READINGS));
		case 5:
			return (int) ((zProbeOnFilter.GetSum() + zProbeOffFilter.GetSum()) / (8 * Z_PROBE_AVERAGE_READINGS));	//TODO this is temporary

		default:
			break;
		}
	}
	return 0;	// Z probe not turned on or not initialised yet
}

// Return the Z probe secondary values.
int Platform::GetZProbeSecondaryValues(int& v1, int& v2)
{
	if (zProbeOnFilter.IsValid() && zProbeOffFilter.IsValid())
	{
		switch (nvData.zProbeType)
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

int Platform::GetZProbeType() const
{
	return nvData.zProbeType;
}

void Platform::SetZProbeAxes(const bool axes[AXES])
{
	for (size_t axis=0; axis<AXES; axis++)
	{
		nvData.zProbeAxes[axis] = axes[axis];
	}
	if (autoSaveEnabled)
	{
		WriteNvData();
	}
}

void Platform::GetZProbeAxes(bool (&axes)[AXES])
{
	for (size_t axis=0; axis<AXES; axis++)
	{
		axes[axis] = nvData.zProbeAxes[axis];
	}
}

float Platform::ZProbeStopHeight() const
{
	switch (nvData.zProbeType)
	{
	case 0:
	case 4:
		return nvData.switchZProbeParameters.GetStopHeight(GetTemperature(0));
	case 1:
	case 2:
		return nvData.irZProbeParameters.GetStopHeight(GetTemperature(0));
	case 3:
	case 5:
		return nvData.alternateZProbeParameters.GetStopHeight(GetTemperature(0));
	default:
		return 0;
	}
}

float Platform::GetZProbeDiveHeight() const
{
	switch (nvData.zProbeType)
	{
	case 1:
	case 2:
		return nvData.irZProbeParameters.diveHeight;
	case 3:
	case 5:
		return nvData.alternateZProbeParameters.diveHeight;
	case 4:
		return nvData.switchZProbeParameters.diveHeight;
	default:
		return DEFAULT_Z_DIVE;
	}
}

float Platform::GetZProbeTravelSpeed() const
{
	switch (nvData.zProbeType)
	{
	case 1:
	case 2:
		return nvData.irZProbeParameters.travelSpeed;
	case 3:
	case 5:
		return nvData.alternateZProbeParameters.travelSpeed;
	case 4:
		return nvData.switchZProbeParameters.travelSpeed;
	default:
		return DEFAULT_TRAVEL_SPEED;
	}
}

void Platform::SetZProbeType(int pt)
{
	int newZProbeType = (pt >= 0 && pt <= 5) ? pt : 0;
	if (newZProbeType != nvData.zProbeType)
	{
		nvData.zProbeType = newZProbeType;
		if (autoSaveEnabled)
		{
			WriteNvData();
		}
	}
	InitZProbe();
}

const ZProbeParameters& Platform::GetZProbeParameters() const
{
	switch (nvData.zProbeType)
	{
	case 0:
	case 4:
	default:
		return nvData.switchZProbeParameters;
	case 1:
	case 2:
		return nvData.irZProbeParameters;
	case 3:
	case 5:
		return nvData.alternateZProbeParameters;
	}
}

bool Platform::SetZProbeParameters(const struct ZProbeParameters& params)
{
	switch (nvData.zProbeType)
	{
	case 0:
	case 4:
		if (nvData.switchZProbeParameters != params)
		{
			nvData.switchZProbeParameters = params;
			if (autoSaveEnabled)
			{
				WriteNvData();
			}
		}
		return true;
	case 1:
	case 2:
		if (nvData.irZProbeParameters != params)
		{
			nvData.irZProbeParameters = params;
			if (autoSaveEnabled)
			{
				WriteNvData();
			}
		}
		return true;
	case 3:
	case 5:
		if (nvData.alternateZProbeParameters != params)
		{
			nvData.alternateZProbeParameters = params;
			if (autoSaveEnabled)
			{
				WriteNvData();
			}
		}
		return true;
	default:
		return false;
	}
}

// Return true if we must home X and Y before we home Z (i.e. we are using a bed probe)
bool Platform::MustHomeXYBeforeZ() const
{
	return nvData.zProbeType != 0 && nvData.zProbeAxes[Z_AXIS];
}

void Platform::ResetNvData()
{
	nvData.compatibility = marlin;				// default to Marlin because the common host programs expect the "OK" response to commands
	ARRAY_INIT(nvData.ipAddress, IP_ADDRESS);
	ARRAY_INIT(nvData.netMask, NET_MASK);
	ARRAY_INIT(nvData.gateWay, GATE_WAY);

#ifdef DUET_NG
	memset(nvData.macAddress, 0xFF, sizeof(nvData.macAddress));
#else
	ARRAY_INIT(nvData.macAddress, MAC_ADDRESS);
#endif

	nvData.zProbeType = 0;	// Default is to use no Z probe switch
	ARRAY_INIT(nvData.zProbeAxes, Z_PROBE_AXES);
	nvData.switchZProbeParameters.Init(0.0);
	nvData.irZProbeParameters.Init(Z_PROBE_STOP_HEIGHT);
	nvData.alternateZProbeParameters.Init(Z_PROBE_STOP_HEIGHT);

	for (size_t i = 0; i < HEATERS; ++i)
	{
		PidParameters& pp = nvData.pidParams[i];
		pp.thermistorSeriesR = defaultThermistorSeriesRs[i];
		pp.SetThermistorR25AndBeta(defaultThermistor25RS[i], defaultThermistorBetas[i]);
		pp.kI = defaultPidKis[i];
		pp.kD = defaultPidKds[i];
		pp.kP = defaultPidKps[i];
		pp.kT = defaultPidKts[i];
		pp.kS = defaultPidKss[i];
		pp.fullBand = defaultFullBands[i];
		pp.pidMin = defaultPidMins[i];
		pp.pidMax = defaultPidMaxes[i];
		pp.adcLowOffset = pp.adcHighOffset = 0.0;
	}

#if FLASH_SAVE_ENABLED
	nvData.magic = FlashData::magicValue;
	nvData.version = FlashData::versionValue;
#endif
}

void Platform::ReadNvData()
{
#if FLASH_SAVE_ENABLED
	DueFlashStorage::read(FlashData::nvAddress, &nvData, sizeof(nvData));
	if (nvData.magic != FlashData::magicValue || nvData.version != FlashData::versionValue)
	{
		// Nonvolatile data has not been initialized since the firmware was last written, so set up default values
		ResetNvData();
		// No point in writing it back here
	}
#else
	Message(BOTH_ERROR_MESSAGE, "Cannot load non-volatile data, because Flash support has been disabled!\n");
#endif
}

void Platform::WriteNvData()
{
#if FLASH_SAVE_ENABLED
	DueFlashStorage::write(FlashData::nvAddress, &nvData, sizeof(nvData));
#else
	Message(BOTH_ERROR_MESSAGE, "Cannot write non-volatile data, because Flash support has been disabled!\n");
#endif
}

void Platform::SetAutoSave(bool enabled)
{
#if FLASH_SAVE_ENABLED
	autoSaveEnabled = enabled;
#else
	Message(BOTH_ERROR_MESSAGE, "Cannot enable auto-save, because Flash support has been disabled!\n");
#endif
}

void Platform::UpdateFirmware()
{
	FileStore *iapFile = GetFileStore(GetSysDir(), IAP_UPDATE_FILE, false);
	if (iapFile == nullptr)
	{
		MessageF(GENERIC_MESSAGE, "Error: Could not open IAP programmer binary \"%s\"\n", IAP_UPDATE_FILE);
		return;
	}

	// The machine will be unresponsive for a few seconds, don't risk damaging the heaters...
	reprap.EmergencyStop();

	// Step 1 - Write update binary to Flash and overwrite the remaining space with zeros
	// Leave the last 1KB of Flash memory untouched, so we can reuse the NvData after this update
#if !defined(IFLASH_PAGE_SIZE) && defined(IFLASH0_PAGE_SIZE)
# define IFLASH_PAGE_SIZE	IFLASH0_PAGE_SIZE
#endif

	char data[IFLASH_PAGE_SIZE];
	int bytesRead;
	for(uint32_t flashAddr = IAP_FLASH_START; flashAddr < IAP_FLASH_END; flashAddr += IFLASH_PAGE_SIZE)
	{
		bytesRead = iapFile->Read(data, IFLASH_PAGE_SIZE);

		if (bytesRead > 0)
		{
			// Do we have to fill up the remaining buffer with zeros?
			if (bytesRead != IFLASH_PAGE_SIZE)
			{
				memset(data + bytesRead, 0, sizeof(data[0]) * (IFLASH_PAGE_SIZE - bytesRead));
			}

			// Write one page at a time
			cpu_irq_disable();
			flash_unlock(flashAddr, flashAddr + IFLASH_PAGE_SIZE - 1, nullptr, nullptr);
			flash_write(flashAddr, data, IFLASH_PAGE_SIZE, 1);
			flash_lock(flashAddr, flashAddr + IFLASH_PAGE_SIZE - 1, nullptr, nullptr);
			cpu_irq_enable();

			// Verify written data
			if (memcmp(reinterpret_cast<void *>(flashAddr), data, bytesRead) != 0)
			{
				Message(GENERIC_MESSAGE, "Error: Verify during Flash write failed!\n");
				return;
			}
		}
		else
		{
			// Empty write buffer so we can use it to fill up the remaining space
			if (bytesRead != IFLASH_PAGE_SIZE)
			{
				memset(data, 0, sizeof(data[0]) * sizeof(data));
				bytesRead = IFLASH_PAGE_SIZE;
			}

			// Fill up the remaining space
			cpu_irq_disable();
			flash_unlock(flashAddr, flashAddr + IFLASH_PAGE_SIZE - 1, nullptr, nullptr);
			flash_write(flashAddr, data, IFLASH_PAGE_SIZE, 1);
			flash_lock(flashAddr, flashAddr + IFLASH_PAGE_SIZE - 1, nullptr, nullptr);
			cpu_irq_enable();
		}
	}
	iapFile->Close();

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

	// Stop processing data
	Message(GENERIC_MESSAGE, "Platform class exited.\n");
	active = false;
}

Compatibility Platform::Emulating() const
{
	if (nvData.compatibility == reprapFirmware)
		return me;
	return nvData.compatibility;
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
	if (c != nvData.compatibility)
	{
		nvData.compatibility = c;
		if (autoSaveEnabled)
		{
			WriteNvData();
		}
	}
}

void Platform::UpdateNetworkAddress(byte dst[4], const byte src[4])
{
	bool changed = false;
	for (uint8_t i = 0; i < 4; i++)
	{
		if (dst[i] != src[i])
		{
			dst[i] = src[i];
			changed = true;
		}
	}
	if (changed && autoSaveEnabled)
	{
		WriteNvData();
	}
}

void Platform::SetIPAddress(byte ip[])
{
	UpdateNetworkAddress(nvData.ipAddress, ip);
}

void Platform::SetGateWay(byte gw[])
{
	UpdateNetworkAddress(nvData.gateWay, gw);
}

void Platform::SetNetMask(byte nm[])
{
	UpdateNetworkAddress(nvData.netMask, nm);
}

void Platform::Spin()
{
	if (!active)
		return;

	// Check if any files are supposed to be closed
	for(size_t i = 0; i < MAX_FILES; i++)
	{
		if (files[i]->closeRequested)
		{
			// We cannot do this in ISRs, so do it here
			files[i]->Close();
		}
	}

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

	// Thermostatically-controlled fans
	for (size_t fan = 0; fan < NUM_FANS; ++fan)
	{
		fans[fan].Check();
	}

	// Diagnostics test
	if (debugCode == (int)DiagnosticTestType::TestSpinLockup)
	{
		for (;;) {}
	}

	ClassReport(longWait);
}

void Platform::SoftwareReset(uint16_t reason)
{
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
		SoftwareResetData temp;
		temp.magic = SoftwareResetData::magicValue;
		temp.version = SoftwareResetData::versionValue;
		temp.resetReason = reason;
		GetStackUsage(NULL, NULL, &temp.neverUsedRam);

		// Save diagnostics data to Flash and reset the software
		DueFlashStorage::write(SoftwareResetData::nvAddress, &temp, sizeof(SoftwareResetData));
	}
	Reset();
	for(;;) {}
}


//*****************************************************************************************************************

// Interrupts

void STEP_TC_HANDLER()
{
	STEP_TC->TC_CHANNEL[STEP_TC_CHAN].TC_IDR = TC_IER_CPAS;	// disable the interrupt
#ifdef MOVE_DEBUG
	++numInterruptsExecuted;
	lastInterruptTime = Platform::GetInterruptClocks();
#endif
	reprap.GetMove()->Interrupt();
}

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
	STEP_TC->TC_CHANNEL[STEP_TC_CHAN].TC_IDR = ~(uint32_t)0;	// interrupts disabled for now
	tc_start(STEP_TC, 0);
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
	if (coolingFanRpmPin >= 0)
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
void Platform::Diagnostics()
{
	Message(GENERIC_MESSAGE, "Platform Diagnostics:\n");

	// Print memory stats and error codes to USB and copy them to the current webserver reply
	const char *ramstart = (char *) 0x20070000;
	const struct mallinfo mi = mallinfo();
	Message(GENERIC_MESSAGE, "Memory usage:\n");
	MessageF(GENERIC_MESSAGE, "Program static ram used: %d\n", &_end - ramstart);
	MessageF(GENERIC_MESSAGE, "Dynamic ram used: %d\n", mi.uordblks);
	MessageF(GENERIC_MESSAGE, "Recycled dynamic ram: %d\n", mi.fordblks);
	size_t currentStack, maxStack, neverUsed;
	GetStackUsage(&currentStack, &maxStack, &neverUsed);
	MessageF(GENERIC_MESSAGE, "Current stack ram used: %d\n", currentStack);
	MessageF(GENERIC_MESSAGE, "Maximum stack ram used: %d\n", maxStack);
	MessageF(GENERIC_MESSAGE, "Never used ram: %d\n", neverUsed);

	// Show the up time and reason for the last reset
	const uint32_t now = (uint32_t)Time();		// get up time in seconds
	const char* resetReasons[8] = { "power up", "backup", "watchdog", "software", "external", "?", "?", "?" };
	MessageF(GENERIC_MESSAGE, "Last reset %02d:%02d:%02d ago, cause: %s\n",
			(unsigned int)(now/3600), (unsigned int)((now % 3600)/60), (unsigned int)(now % 60),
			resetReasons[(REG_RSTC_SR & RSTC_SR_RSTTYP_Msk) >> RSTC_SR_RSTTYP_Pos]);

	// Show the error code stored at the last software reset
	{
		SoftwareResetData temp;
		temp.magic = 0;
		DueFlashStorage::read(SoftwareResetData::nvAddress, &temp, sizeof(SoftwareResetData));
		if (temp.magic == SoftwareResetData::magicValue && temp.version == SoftwareResetData::versionValue)
		{
			MessageF(GENERIC_MESSAGE, "Last software reset code & available RAM: 0x%04x, %u\n", temp.resetReason, temp.neverUsedRam);
			MessageF(GENERIC_MESSAGE, "Spinning module during software reset: %s\n", moduleName[temp.resetReason & 0x0F]);
		}
	}

	// Show the current error codes
	MessageF(GENERIC_MESSAGE, "Error status: %u\n", errorCodeBits);

	// Show the current probe position heights
	Message(GENERIC_MESSAGE, "Bed probe heights:");
	for (size_t i = 0; i < MAX_PROBE_POINTS; ++i)
	{
		MessageF(GENERIC_MESSAGE, " %.3f", reprap.GetMove()->ZBedProbePoint(i));
	}
	Message(GENERIC_MESSAGE, "\n");

	// Show the number of free entries in the file table
	unsigned int numFreeFiles = 0;
	for (size_t i = 0; i < MAX_FILES; i++)
	{
		if (!files[i]->inUse)
		{
			++numFreeFiles;
		}
	}
	MessageF(GENERIC_MESSAGE, "Free file entries: %u\n", numFreeFiles);

	// Show the longest write time
	MessageF(GENERIC_MESSAGE, "Longest block write time: %.1fms\n", FileStore::GetAndClearLongestWriteTime());

// Debug
//MessageF(GENERIC_MESSAGE, "Shortest/longest times read %.1f/%.1f write %.1f/%.1f ms, %u/%u\n",
//		(float)shortestReadWaitTime/1000, (float)longestReadWaitTime/1000, (float)shortestWriteWaitTime/1000, (float)longestWriteWaitTime/1000,
//		maxRead, maxWrite);
//longestWriteWaitTime = longestReadWaitTime = 0; shortestReadWaitTime = shortestWriteWaitTime = 1000000;

	reprap.Timing();

#ifdef MOVE_DEBUG
	MessageF(GENERIC_MESSAGE, "Interrupts scheduled %u, done %u, last %u, next %u sched at %u, now %u\n",
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

	case (int)DiagnosticTestType::TestSerialBlock:				// write an arbitrary message via debugPrintf()
		debugPrintf("Diagnostic Test\n");
		break;

	default:
		break;
	}
}

// Return the stack usage and amount of memory that has never been used, in bytes
void Platform::GetStackUsage(size_t* currentStack, size_t* maxStack, size_t* neverUsed) const
{
	const char *ramend = (const char *) 0x20088000;
	register const char * stack_ptr asm ("sp");
	const char *heapend = sbrk(0);
	const char* stack_lwm = heapend;
	while (stack_lwm < stack_ptr && *stack_lwm == memPattern)
	{
		++stack_lwm;
	}
	if (currentStack) { *currentStack = ramend - stack_ptr; }
	if (maxStack) { *maxStack = ramend - stack_lwm; }
	if (neverUsed) { *neverUsed = stack_lwm - heapend; }
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
// Normally T0 is 298.15K (25 C).  If you write that expression in brackets in the #define the compiler 
// should compute it for you (i.e. it won't need to be calculated at run time).

// If the A->D converter has a range of 0..1023 and the measured voltage is V (between 0 and 1023)
// then the thermistor resistance, R = V.RS/(1024 - V)
// and the temperature, T = BETA/ln(R/R_INF)
// To get degrees celsius (instead of kelvin) add -273.15 to T

// Result is in degrees celsius

float Platform::GetTemperature(size_t heater, TempError* err) const
{
	// Note that at this point we're actually getting an averaged ADC read, not a "raw" temp.  For thermistors,
	// we're getting an averaged voltage reading which we'll convert to a temperature.
	if (DoThermistorAdc(heater))
	{
		int rawTemp = GetRawTemperature(heater);

		// If the ADC reading is N then for an ideal ADC, the input voltage is at least N/(AD_RANGE + 1) and less than (N + 1)/(AD_RANGE + 1), times the analog reference.
		// So we add 0.5 to to the reading to get a better estimate of the input.
		float reading = (float) rawTemp + 0.5;

		// Recognise the special case of thermistor disconnected.
		// For some ADCs, the high-end offset is negative, meaning that the ADC never returns a high enough value. We need to allow for this here.
		const PidParameters& p = nvData.pidParams[heater];
		if (p.adcHighOffset < 0.0)
		{
			rawTemp -= (int) p.adcHighOffset;
		}
		if (rawTemp >= (int)AD_DISCONNECTED_VIRTUAL)
		{
			// thermistor is disconnected
			if (err)
			{
				*err = TempError::errOpen;
			}
			return ABS_ZERO;
		}

		// Correct for the low and high ADC offsets
		reading -= p.adcLowOffset;
		reading *= (AD_RANGE_VIRTUAL + 1) / (AD_RANGE_VIRTUAL + 1 + p.adcHighOffset - p.adcLowOffset);

		float resistance = reading * p.thermistorSeriesR / ((AD_RANGE_VIRTUAL + 1) - reading);
		if (resistance > p.GetRInf())
		{
			return ABS_ZERO + p.GetBeta() / log(resistance / p.GetRInf());
		}
		else
		{
			// thermistor short circuit, return a high temperature
			if (err)
			{
				*err = TempError::errShort;
			}
			return BAD_ERROR_TEMPERATURE;
		}
	}
	else
	{
		// MAX31855 thermocouple chip
		float temp;
		MAX31855_error res = Max31855Devices[heaterTempChannels[heater] - MAX31855_START_CHANNEL].getTemperature(&temp);
		if (res == MAX31855_OK)
		{
			return temp;
		}
		if (err)
		{
			switch(res) {
			case MAX31855_OK      : *err = TempError::errOk; break;			// Success
			case MAX31855_ERR_SCV : *err = TempError::errShortVcc; break;	// Short to Vcc
			case MAX31855_ERR_SCG : *err = TempError::errShortGnd; break;	// Short to GND
			case MAX31855_ERR_OC  : *err = TempError::errOpen; break;		// Open connection
			case MAX31855_ERR_TMO : *err = TempError::errTimeout; break;	// SPI comms timeout
			case MAX31855_ERR_IO  : *err = TempError::errIO; break;			// SPI comms not functioning
			}
		}
		return BAD_ERROR_TEMPERATURE;
	}
}

/*static*/ const char* Platform::TempErrorStr(TempError err)
{
	switch(err)
	{
	default : return "Unknown temperature read error";
	case TempError::errOk : return "successful temperature read";
	case TempError::errShort : return "sensor circuit is shorted";
	case TempError::errShortVcc : return "sensor circuit is shorted to the voltage rail";
	case TempError::errShortGnd : return "sensor circuit is shorted to ground";
	case TempError::errOpen : return "sensor circuit is open/disconnected";
	case TempError::errTooHigh: return "temperature above safety limit";
	case TempError::errTimeout : return "communication error whilst reading sensor; read took too long";
	case TempError::errIO: return "communication error whilst reading sensor; check sensor connections";
	}
}

// See if we need to turn on the hot end fan
bool Platform::AnyHeaterHot(uint16_t heaters, float t) const
{
	// Check tool heaters first
	for (size_t h = 0; h < reprap.GetToolHeatersInUse(); ++h)
	{
		if (((1 << h) & heaters) != 0)
		{
			const float ht = GetTemperature(h);
			if (ht >= t || ht < BAD_LOW_TEMPERATURE)
			{
				return true;
			}
		}
	}

	// Check the bed heater too, in case the user wants a fan on when the bed is hot
	int8_t bedHeater = reprap.GetHeat()->GetBedHeater();
	if (bedHeater >= 0 && ((1 << (unsigned int)bedHeater) & heaters) != 0)
	{
		const float ht = GetTemperature(bedHeater);
		if (ht >= t || ht < BAD_LOW_TEMPERATURE)
		{
			return true;
		}
	}

	// Check the chamber heater as well
	int8_t chamberHeater = reprap.GetHeat()->GetChamberHeater();
	if (chamberHeater >= 0 && ((1 << (unsigned int)chamberHeater) & heaters) != 0)
	{
		const float ht = GetTemperature(chamberHeater);
		if (ht >= t || ht < BAD_LOW_TEMPERATURE)
		{
			return true;
		}
	}
	return false;
}

// Update the heater PID parameters or thermistor resistance etc.
void Platform::SetPidParameters(size_t heater, const PidParameters& params)
{
	if (heater < HEATERS && params != nvData.pidParams[heater])
	{
		nvData.pidParams[heater] = params;
		if (autoSaveEnabled)
		{
			WriteNvData();
		}
		SetTemperatureLimit(temperatureLimit);		// recalculate the thermistor resistance at max allowed temperature for the tick ISR
	}
}
const PidParameters& Platform::GetPidParameters(size_t heater) const
{
	return nvData.pidParams[heater];
}

// power is a fraction in [0,1]

void Platform::SetHeater(size_t heater, float power)
{
	SetHeaterPwm(heater, (uint8_t)(255.0 * min<float>(1.0, max<float>(0.0, power))));
}

void Platform::SetHeaterPwm(size_t heater, uint8_t power)
{
	if (heatOnPins[heater] >= 0)
	{
		uint16_t freq = (reprap.GetHeat()->UseSlowPwm(heater)) ? SlowHeaterPwmFreq : NormalHeaterPwmFreq;
		AnalogWrite(heatOnPins[heater], (HEAT_ON) ? power : 255 - power, freq);
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
	if (endStopType[drive] == EndStopType::noEndStop)
	{
		// No homing switch is configured for this axis, so see if we should use the Z probe
		if (nvData.zProbeType > 0 && drive < AXES && nvData.zProbeAxes[drive])
		{
			return GetZProbeResult();			// using the Z probe as a low homing stop for this axis, so just get its result
		}
	}
	else if (endStopPins[drive] >= 0)
	{
		if (digitalRead(endStopPins[drive]) == ((endStopLogicLevel[drive]) ? 1 : 0))
		{
			return (endStopType[drive] == EndStopType::highEndStop) ? EndStopHit::highHit : EndStopHit::lowHit;
		}
	}
	return EndStopHit::noStop;
}

// Return the Z probe result. We assume that if the Z probe is used as an endstop, it is used as the low stop.
EndStopHit Platform::GetZProbeResult() const
{
	const int zProbeVal = ZProbe();
	const int zProbeADValue =
			(nvData.zProbeType == 4) ? nvData.switchZProbeParameters.adcValue
			: (nvData.zProbeType == 3) ? nvData.alternateZProbeParameters.adcValue
				: nvData.irZProbeParameters.adcValue;
	return (zProbeVal >= zProbeADValue) ? EndStopHit::lowHit
			: (zProbeVal * 10 >= zProbeADValue * 9) ? EndStopHit::lowNear	// if we are at/above 90% of the target value
				: EndStopHit::noStop;
}

// This is called from the step ISR as well as other places, so keep it fast, especially in the case where the motor is already enabled
void Platform::SetDirection(size_t drive, bool direction)
{
	const int driver = driverNumbers[drive];
	if (driver >= 0)
	{
		const int pin = directionPins[driver];
		if (pin >= 0)
		{
			bool d = (direction == FORWARDS) ? directions[driver] : !directions[driver];
			digitalWrite(pin, d);
		}
	}
}

// Enable a drive. Must not be called from an ISR, or with interrupts disabled.
void Platform::EnableDrive(size_t drive)
{
	if (drive < DRIVES && driveState[drive] != DriveStatus::enabled)
	{
		driveState[drive] = DriveStatus::enabled;
		const size_t driver = driverNumbers[drive];
		if (driver >= 0)
		{
			UpdateMotorCurrent(driver);						// the current may have been reduced by the idle timeout

#ifdef EXTERNAL_DRIVERS
			if (drive >= FIRST_EXTERNAL_DRIVE)
			{
				ExternalDrivers::EnableDrive(driver - FIRST_EXTERNAL_DRIVE, true);
			}
			else
			{
#endif
				const int pin = enablePins[driver];
				if (pin >= 0)
				{
					digitalWrite(pin, enableValues[driver]);
				}
#ifdef EXTERNAL_DRIVERS
			}
#endif
		}
	}
}

// Disable a drive, if it has a disable pin
void Platform::DisableDrive(size_t drive)
{
	if (drive < DRIVES)
	{
		const size_t driver = driverNumbers[drive];
#ifdef EXTERNAL_DRIVERS
		if (drive >= FIRST_EXTERNAL_DRIVE)
		{
			ExternalDrivers::EnableDrive(driver - FIRST_EXTERNAL_DRIVE, false);
		}
		else
		{
#endif
			const int pin = enablePins[driver];
			if (pin >= 0)
			{
				digitalWrite(pin, !enableValues[driver]);
			}
			driveState[drive] = DriveStatus::disabled;
#ifdef EXTERNAL_DRIVERS
		}
#endif
	}
}

// Set drives to idle hold if they are enabled. If a drive is disabled, leave it alone.
// Must not be called from an ISR, or with interrupts disabled.
void Platform::SetDrivesIdle()
{
	for (size_t drive = 0; drive < DRIVES; ++drive)
	{
		if (driveState[drive] == DriveStatus::enabled)
		{
			driveState[drive] = DriveStatus::idle;
			UpdateMotorCurrent(drive);
		}
	}
}

// Set the current for a motor. Current is in mA.
void Platform::SetMotorCurrent(size_t drive, float current)
{
	if (drive < DRIVES)
	{
		motorCurrents[drive] = current;
		UpdateMotorCurrent(drive);
	}
}

// This must not be called from an ISR, or with interrupts disabled.
void Platform::UpdateMotorCurrent(size_t drive)
{
	if (drive < DRIVES)
	{
		float current = motorCurrents[drive];
		if (driveState[drive] == DriveStatus::idle)
		{
			current *= idleCurrentFactor;
		}
		const size_t driver = driverNumbers[drive];
#ifdef EXTERNAL_DRIVERS
		if (driver >= FIRST_EXTERNAL_DRIVE)
		{
			ExternalDrivers::SetCurrent(driver - FIRST_EXTERNAL_DRIVE, current);
		}
		else
		{
#endif
			unsigned short pot = (unsigned short)((0.256*current*8.0*senseResistor + maxStepperDigipotVoltage/2)/maxStepperDigipotVoltage);
			unsigned short dac = (unsigned short)((0.256*current*8.0*senseResistor + maxStepperDACVoltage/2)/maxStepperDACVoltage);
			if (driver < 4)
			{
				mcpDuet.setNonVolatileWiper(potWipes[driver], pot);
				mcpDuet.setVolatileWiper(potWipes[driver], pot);
			}
			else
			{
#ifndef DUET_NG
				if (board == BoardType::Duet_085)
				{
#endif
					// Extruder 0 is on DAC channel 0
					if (driver == 4)
					{
#ifdef DUET_NG
						AnalogWrite(DAC1, dac);
#else
						AnalogWrite(DAC0, dac);
#endif
					}
					else
					{
						mcpExpansion.setNonVolatileWiper(potWipes[driver-1], pot);
						mcpExpansion.setVolatileWiper(potWipes[driver-1], pot);
					}
#ifndef DUET_NG
				}
				else
				{
					mcpExpansion.setNonVolatileWiper(potWipes[driver], pot);
					mcpExpansion.setVolatileWiper(potWipes[driver], pot);
				}
#endif
			}
#ifdef EXTERNAL_DRIVERS
		}
#endif
	}
}

float Platform::MotorCurrent(size_t drive) const
{
	return (drive < DRIVES) ? motorCurrents[drive] : 0.0;
}

// Set the motor idle current factor
void Platform::SetIdleCurrentFactor(float f)
{
	idleCurrentFactor = f;
	for (size_t drive = 0; drive < DRIVES; ++drive)
	{
		if (driveState[drive] == DriveStatus::idle)
		{
			UpdateMotorCurrent(drive);
		}
	}
}

// Set the microstepping, returning true if successful
bool Platform::SetMicrostepping(size_t drive, int microsteps, int mode)
{
	if (drive < DRIVES)
	{
#ifdef EXTERNAL_DRIVERS
		const size_t driver = driverNumbers[drive];
		if (driver >= FIRST_EXTERNAL_DRIVE)
		{
			return ExternalDrivers::SetMicrostepping(driver - FIRST_EXTERNAL_DRIVE, microsteps, mode);
		}
		else
		{
#endif
			// On-board drivers only support x16 microstepping.
			// We ignore the interpolation on/off parameter so that e.g. M350 I1 E16:128 won't give an error if E1 supports interpolation but E0 doesn't.
			return microsteps == 16;
#ifdef EXTERNAL_DRIVERS
		}
#endif
	}
	return false;
}

unsigned int Platform::GetMicrostepping(size_t drive, bool& interpolation) const
{
#ifdef EXTERNAL_DRIVERS
	if (drive < DRIVES)
	{
		const size_t driver = driverNumbers[drive];
		if (driver >= FIRST_EXTERNAL_DRIVE)
		{
			return ExternalDrivers::GetMicrostepping(driver - FIRST_EXTERNAL_DRIVE, interpolation);
		}
	}
#endif

	// On-board drivers only support x16 microstepping without interpolation
	interpolation = false;
	return 16;
}

// Set the physical drive (i.e. axis or extruder) number used by this driver
void Platform::SetPhysicalDrive(size_t driverNumber, int8_t physicalDrive)
{
	int oldDrive = GetPhysicalDrive(driverNumber);
	if (oldDrive >= 0)
	{
		driverNumbers[oldDrive] = -1;
		stepPinDescriptors[oldDrive] = OutputPin();
	}
	driverNumbers[physicalDrive] = driverNumber;
	stepPinDescriptors[physicalDrive] = OutputPin(stepPins[driverNumber]);
}

// Return the physical drive used by this driver, or -1 if not found
int Platform::GetPhysicalDrive(size_t driverNumber) const
{
	for (size_t drive = 0; drive < DRIVES; ++drive)
	{
		if (driverNumbers[drive] == (int8_t)driverNumber)
		{
			return drive;
		}
	}
	return -1;
}

// Get current cooling fan speed on a scale between 0 and 1
float Platform::GetFanValue(size_t fan) const
{
	return (fan < NUM_FANS) ? fans[fan].val : -1;
}

bool Platform::GetCoolingInverted(size_t fan) const
{
	return (fan < NUM_FANS) ? fans[fan].inverted : -1;

}

void Platform::SetCoolingInverted(size_t fan, bool inv)
{
	if (fan < NUM_FANS)
	{
		fans[fan].inverted = inv;
	}
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

void Platform::InitFans()
{
	for (size_t i = 0; i < NUM_FANS; ++i)
	{
		fans[i].Init(COOLING_FAN_PINS[i],
				!HEAT_ON
#ifndef DUET_NG
					// The cooling fan 0 output pin gets inverted if HEAT_ON == 0 on a Duet 0.4, 0.6 or 0.7
					&& (board == BoardType::Duet_06 || board == BoardType::Duet_07)
#endif
				);
	}

	if (NUM_FANS > 1)
	{
		// Set fan 1 to be thermostatic by default, monitoring all heaters except the default bed heater
		fans[1].SetHeatersMonitored(0xFFFF & ~(1 << BED_HEATER));
	}

	coolingFanRpmPin = COOLING_FAN_RPM_PIN;
	lastRpmResetTime = 0.0;
	if (coolingFanRpmPin >= 0)
	{
		pinModeDuet(coolingFanRpmPin, INPUT_PULLUP, 1500);	// enable pullup and 1500Hz debounce filter (500Hz only worked up to 7000RPM)
	}
}

float Platform::GetFanPwmFrequency(size_t fan) const
{
	if (fan < NUM_FANS)
	{
		return (float)fans[fan].freq;
	}
	return 0.0;
}

void Platform::SetFanPwmFrequency(size_t fan, float freq)
{
	if (fan < NUM_FANS)
	{
		fans[fan].SetPwmFrequency(freq);
	}
}

float Platform::GetTriggerTemperature(size_t fan) const
{
	if (fan < NUM_FANS)
	{
		return fans[fan].triggerTemperature;
	}
	return ABS_ZERO;

}

void Platform::SetTriggerTemperature(size_t fan, float t)
{
	if (fan < NUM_FANS)
	{
		fans[fan].SetTriggerTemperature(t);
	}
}

uint16_t Platform::GetHeatersMonitored(size_t fan) const
{
	if (fan < NUM_FANS)
	{
		return fans[fan].heatersMonitored;
	}
	return 0;
}

void Platform::SetHeatersMonitored(size_t fan, uint16_t h)
{
	if (fan < NUM_FANS)
	{
		fans[fan].SetHeatersMonitored(h);
	}
}

void Platform::Fan::Init(Pin p_pin, bool hwInverted)
{
	val = 0.0;
	freq = DefaultFanPwmFreq;
	pin = p_pin;
	hardwareInverted = hwInverted;
	inverted = false;
	heatersMonitored = 0;
	triggerTemperature = HOT_END_FAN_TEMPERATURE;
	Refresh();
}

void Platform::Fan::SetValue(float speed)
{
	if (heatersMonitored == 0)
	{
		if (speed > 1.0)
		{
			speed /= 255.0;
		}
		val = constrain<float>(speed, 0.0, 1.0);
		Refresh();
	}
}

void Platform::Fan::Refresh()
{
	if (pin >= 0)
	{
		uint32_t p = (uint32_t)(255.0 * val);
		bool invert = hardwareInverted;
		if (inverted)
		{
			invert = !invert;
		}
		AnalogWrite(pin, (invert) ? (255 - p) : p, freq);
	}
}

void Platform::Fan::SetPwmFrequency(float p_freq)
{
	freq = (uint16_t)constrain<float>(p_freq, 1.0, 65535.0);
	Refresh();
}

void Platform::Fan::Check()
{
	if (heatersMonitored != 0)
	{
		val = (reprap.GetPlatform()->AnyHeaterHot(heatersMonitored, triggerTemperature)) ? 1.0 : 0.0;
		Refresh();
	}
}

void Platform::SetMACAddress(uint8_t mac[])
{
	bool changed = false;
	for (size_t i = 0; i < 6; i++)
	{
		if (nvData.macAddress[i] != mac[i])
		{
			nvData.macAddress[i] = mac[i];
			changed = true;
		}
	}
	if (changed && autoSaveEnabled)
	{
		WriteNvData();
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

void Platform::Message(MessageType type, const char *message)
{
	switch (type)
	{
		case FLASH_LED:
			// Message that is to flash an LED; the next two bytes define
			// the frequency and M/S ratio.
			// (not implemented yet)
			break;

		case AUX_MESSAGE:
			// Message that is to be sent to the first auxiliary device
			if (!auxOutput->IsEmpty())
			{
				// If we're still busy sending a response to the UART device, append this message to the output buffer
				auxOutput->GetLastItem()->cat(message);
			}
			else
			{
				// Send short strings immediately through the aux channel. There is no flow control on this port, so it can't block for long
				SERIAL_AUX_DEVICE.write(message);
				SERIAL_AUX_DEVICE.flush();
			}
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

		case DISPLAY_MESSAGE:
			// Message that is to appear on a local display;  \f and \n should be supported.
			reprap.SetMessage(message);
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

				// Check if we need to write the indentation chars first
				const size_t stackPointer = reprap.GetGCodes()->GetStackPointer();
				if (stackPointer > 0)
				{
					// First, make sure we get the indentation right
					char indentation[StackSize * 2 + 1];
					for(size_t i = 0; i < stackPointer * 2; i++)
					{
						indentation[i] = ' ';
					}
					indentation[stackPointer * 2] = 0;

					// Append the indentation string
					usbOutputBuffer->cat(indentation);
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

		case GENERIC_MESSAGE:
			// Message that is to be sent to the web & host. Make this the default one, too.
		default:
			Message(HTTP_MESSAGE, message);
			Message(TELNET_MESSAGE, message);
			Message(HOST_MESSAGE, message);
			break;
	}
}

void Platform::Message(const MessageType type, const StringRef &message)
{
	Message(type, message.Pointer());
}

void Platform::Message(const MessageType type, OutputBuffer *buffer)
{
	switch (type)
	{
		case AUX_MESSAGE:
			// If no AUX device is attached, don't queue this buffer
			if (!reprap.GetGCodes()->HaveAux())
			{
				OutputBuffer::ReleaseAll(buffer);
				break;
			}

			// For big responses it makes sense to write big chunks of data in portions. Store this data here
			auxOutput->Push(buffer);
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
			buffer->IncreaseReferences(2);		// This one is handled by two additional destinations
			Message(HTTP_MESSAGE, buffer);
			Message(TELNET_MESSAGE, buffer);
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
	return (digitalRead(ATX_POWER_PIN) == HIGH);
}

void Platform::SetAtxPower(bool on)
{
	digitalWrite(ATX_POWER_PIN, (on) ? HIGH : LOW);
}


void Platform::SetElasticComp(size_t extruder, float factor)
{
	if (extruder < DRIVES - AXES)
	{
		elasticComp[extruder] = factor;
	}
}

float Platform::ActualInstantDv(size_t drive) const
{
	float idv = instantDvs[drive];
	if (drive >= AXES)
	{
		float eComp = elasticComp[drive - AXES];
		// If we are using elastic compensation then we need to limit the instantDv to avoid velocity mismatches
		return (eComp <= 0.0) ? idv : min<float>(idv, 1.0/(eComp * driveStepsPerUnit[drive]));
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
		board = BoardType::DuetNG_08;
#else
		// Determine whether this is a Duet 0.6 or a Duet 0.8.5 board.
		// If it is a 0.85 board then DAC0 (AKA digital pin 67) is connected to ground via a diode and a 2.15K resistor.
		// So we enable the pullup (value 150K-150K) on pin 67 and read it, expecting a LOW on a 0.8.5 board and a HIGH on a 0.6 board.
		// This may fail if anyone connects a load to the DAC0 pin on a Duet 0.6, hence we implement board selection in M115 as well.
		pinMode(Dac0DigitalPin, INPUT_PULLUP);
		board = (digitalRead(Dac0DigitalPin)) ? BoardType::Duet_06 : BoardType::Duet_085;
		pinMode(Dac0DigitalPin, INPUT);	// turn pullup off
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
	case BoardType::DuetNG_08:				return "DuetNG 0.8";
#else
	case BoardType::Duet_06:				return "Duet 0.6";
	case BoardType::Duet_07:				return "Duet 0.7";
	case BoardType::Duet_085:				return "Duet 0.85";
#endif
	default:								return "Unidentified";
	}
}

// Direct pin operations
// Set the specified pin to the specified output level. Return true if success, false if not allowed.
bool Platform::SetPin(int pin, int level)
{
	if (pin >= 0 && (unsigned int)pin < NUM_PINS_ALLOWED && (level == 0 || level == 1))
	{
		const size_t index = (unsigned int)pin/8;
		const uint8_t mask = 1 << ((unsigned int)pin & 7);
		if ((pinAccessAllowed[index] & mask) != 0)
		{
			if ((pinInitialised[index] & mask) == 0)
			{
				pinMode(pin, OUTPUT);
				pinInitialised[index] |= mask;
			}
			digitalWrite(pin, level);
			return true;
		}
	}
	return false;
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

// Pragma pop_options is not supported on this platform, so we put this time-critical code right at the end of the file
//#pragma GCC push_options
#pragma GCC optimize ("O3")

// Schedule an interrupt at the specified clock count, or return true if that time is imminent or has passed already.
// Must be called with interrupts disabled,
/*static*/ bool Platform::ScheduleInterrupt(uint32_t tim)
{
	tc_write_ra(TC1, 0, tim);								// set up the compare register
	tc_get_status(TC1, 0);									// clear any pending interrupt
	int32_t diff = (int32_t)(tim - tc_read_cv(TC1, 0));		// see how long we have to go
	if (diff < (int32_t)DDA::minInterruptInterval)			// if less than about 2us or already passed
	{
		return true;										// tell the caller to simulate an interrupt instead
	}

	TC1 ->TC_CHANNEL[0].TC_IER = TC_IER_CPAS;				// enable the interrupt
#ifdef MOVE_DEBUG
		++numInterruptsScheduled;
		nextInterruptTime = tim;
		nextInterruptScheduledAt = Platform::GetInterruptClocks();
#endif
	return false;
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
	switch (tickState)
	{
	case 1:			// last conversion started was a thermistor
	case 3:
		{
			if (DoThermistorAdc(currentHeater))
			{
				ThermistorAveragingFilter& currentFilter = const_cast<ThermistorAveragingFilter&>(thermistorFilters[currentHeater]);
				currentFilter.ProcessReading(AnalogInReadChannel(thermistorAdcChannels[heaterTempChannels[currentHeater]]));
				if (currentFilter.IsValid() && (configuredHeaters & (1 << currentHeater)) != 0)
				{
					uint32_t sum = currentFilter.GetSum();
					if (sum < thermistorOverheatSums[currentHeater] || sum >= AD_DISCONNECTED_REAL * THERMISTOR_AVERAGE_READINGS)
					{
						// We have an over-temperature or disconnected reading from this thermistor, so turn off the heater
						SetHeaterPwm(currentHeater, 0);
						LogError(ErrorCode::BadTemp);
					}
				}
			}
			else
			{
				// Thermocouple case: oversampling is not necessary as the MAX31855 is itself continuously sampling and
				// averaging.  As such, the temperature reading is taken directly by Platform::GetTemperature() and
				// periodically called by PID::Spin() where temperature fault handling is taken care of.  However, we
				// must guard against overly long delays between successive calls of PID::Spin().
				// Do not call Time() here, it isn't safe. We use millis() instead.
				if ((millis() - reprap.GetHeat()->GetLastSampleTime(currentHeater)) > maxPidSpinDelay)
				{
					SetHeaterPwm(currentHeater, 0);
					LogError(ErrorCode::BadTemp);
				}
			}

			++currentHeater;
			if (currentHeater == HEATERS)
			{
				currentHeater = 0;
			}
		}
		++tickState;
		break;

	case 2:			// last conversion started was the Z probe, with IR LED on
		const_cast<ZProbeAveragingFilter&>(zProbeOnFilter).ProcessReading(GetRawZProbeReading());
		if (nvData.zProbeType == 2)								// if using a modulated IR sensor
		{
			digitalWrite(zProbeModulationPin, LOW);				// turn off the IR emitter
		}
		++tickState;
		break;

	case 4:			// last conversion started was the Z probe, with IR LED off if modulation is enabled
		const_cast<ZProbeAveragingFilter&>(zProbeOffFilter).ProcessReading(GetRawZProbeReading());
		// no break
	case 0:			// this is the state after initialisation, no conversion has been started
	default:
		if (nvData.zProbeType == 2)								// if using a modulated IR sensor
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
