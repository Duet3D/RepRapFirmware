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

Platform::Platform() :
		tickState(0), fileStructureInitialised(false), active(false), errorCodeBits(0), debugCode(0),
		messageString(messageStringBuffer, ARRAY_SIZE(messageStringBuffer)), autoSaveEnabled(false)
{
	line = new Line(SerialUSB);
	aux = new Line(Serial);

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
	digitalWriteNonDue(atxPowerPin, LOW);		// ensure ATX power is off by default
	pinModeNonDue(atxPowerPin, OUTPUT);

	idleCurrentFactor = defaultIdleCurrentFactor;

	baudRates[0] = MainBaudRate;
	baudRates[1] = AuxBaudRate;
	commsParams[0] = 0;
	commsParams[1] = 1;							// by default we require a checksum on data from the aux port, to guard against overrun errors

	SerialUSB.begin(baudRates[0]);
	Serial.begin(baudRates[1]);					// this can't be done in the constructor because the Arduino port initialisation isn't complete at that point

	static_assert(sizeof(FlashData) + sizeof(SoftwareResetData) <= FLASH_DATA_LENGTH, "NVData too large");

	ResetNvData();

	line->Init();
	aux->Init();
	messageIndent = 0;

	// We need to initialize at least some of the time stuff before we call MassStorage::Init()
	addToTime = 0.0;
	lastTimeCall = 0;
	lastTime = Time();
	longWait = lastTime;

	massStorage->Init();

	for (size_t file = 0; file < MAX_FILES; file++)
	{
		files[file]->Init();
	}

	fileStructureInitialised = true;

	mcpDuet.begin();							// only call begin once in the entire execution, this begins the I2C comms on that channel for all objects
	mcpExpansion.setMCP4461Address(0x2E);		// not required for mcpDuet, as this uses the default address
	sysDir = SYS_DIR;
	configFile = CONFIG_FILE;
	defaultFile = DEFAULT_FILE;

	// DRIVES

	ARRAY_INIT(stepPins, STEP_PINS);
	ARRAY_INIT(directionPins, DIRECTION_PINS);
	ARRAY_INIT(directions, DIRECTIONS);
	ARRAY_INIT(enablePins, ENABLE_PINS);
	ARRAY_INIT(endStopPins, END_STOP_PINS);
	ARRAY_INIT(maxFeedrates, MAX_FEEDRATES);
	ARRAY_INIT(accelerations, ACCELERATIONS);
	ARRAY_INIT(driveStepsPerUnit, DRIVE_STEPS_PER_UNIT);
	ARRAY_INIT(instantDvs, INSTANT_DVS);
	ARRAY_INIT(potWipes, POT_WIPES);
	senseResistor = SENSE_RESISTOR;
	maxStepperDigipotVoltage = MAX_STEPPER_DIGIPOT_VOLTAGE;
	//numMixingDrives = NUM_MIXING_DRIVES;

	// Z PROBE

	zProbePin = Z_PROBE_PIN;
	zProbeAdcChannel = PinToAdcChannel(zProbePin);
	InitZProbe();

	// AXES

	ARRAY_INIT(axisMaxima, AXIS_MAXIMA);
	ARRAY_INIT(axisMinima, AXIS_MINIMA);
	ARRAY_INIT(homeFeedrates, HOME_FEEDRATES);

	SetSlowestDrive();

	// HEATERS - Bed is assumed to be the first

	ARRAY_INIT(tempSensePins, TEMP_SENSE_PINS);
	ARRAY_INIT(heatOnPins, HEAT_ON_PINS);
	ARRAY_INIT(standbyTemperatures, STANDBY_TEMPERATURES);
	ARRAY_INIT(activeTemperatures, ACTIVE_TEMPERATURES);

	heatSampleTime = HEAT_SAMPLE_TIME;
	coolingFanValue = 0.0;
	coolingFanPin = COOLING_FAN_PIN;
	coolingFanRpmPin = COOLING_FAN_RPM_PIN;
	timeToHot = TIME_TO_HOT;
	lastRpmResetTime = 0.0;

	webDir = WEB_DIR;
	gcodeDir = GCODE_DIR;
	tempDir = TEMP_DIR;

	for (size_t drive = 0; drive < DRIVES; drive++)
	{
		if (stepPins[drive] >= 0)
		{
			pinModeNonDue(stepPins[drive], OUTPUT);
		}
		if (directionPins[drive] >= 0)
		{
			pinModeNonDue(directionPins[drive], OUTPUT);
		}
		if (enablePins[drive] >= 0)
		{
			pinModeNonDue(enablePins[drive], OUTPUT);
		}
		if (endStopPins[drive] >= 0)
		{
			pinModeNonDue(endStopPins[drive], INPUT_PULLUP);
		}
		motorCurrents[drive] = 0.0;
		DisableDrive(drive);
		driveState[drive] = DriveStatus::disabled;
		SetElasticComp(drive, 0.0);
		if (drive <= AXES)
		{
			endStopType[drive] = EndStopType::lowEndStop;	// assume all endstops are low endstops
			endStopLogicLevel[drive] = true;				// assume all endstops use active high logic e.g. normally-closed switch to ground
		}
	}

	extrusionAncilliaryPWM = 0.0;

	// HEATERS - Bed is assumed to be index 0
	for (size_t heater = 0; heater < HEATERS; heater++)
	{
		if (heatOnPins[heater] >= 0)
		{
			digitalWriteNonDue(heatOnPins[heater], HIGH);	// turn the heater off
			pinModeNonDue(heatOnPins[heater], OUTPUT);
		}
		analogReadResolution(12);
		SetThermistorNumber(heater, heater);				// map the thermistor straight through
		thermistorFilters[heater].Init(analogRead(tempSensePins[heater]));

		// Calculate and store the ADC average sum that corresponds to an overheat condition, so that we can check is quickly in the tick ISR
		float thermistorOverheatResistance = nvData.pidParams[heater].GetRInf()
				* exp(-nvData.pidParams[heater].GetBeta() / (BAD_HIGH_TEMPERATURE - ABS_ZERO));
		float thermistorOverheatAdcValue = (adRangeReal + 1) * thermistorOverheatResistance
				/ (thermistorOverheatResistance + nvData.pidParams[heater].thermistorSeriesR);
		thermistorOverheatSums[heater] = (uint32_t) (thermistorOverheatAdcValue + 0.9) * numThermistorReadingsAveraged;
	}

	if (coolingFanPin >= 0)
	{
		// Inverse logic for Duet v0.6 and later; this turns it off
		analogWriteNonDue(coolingFanPin, (HEAT_ON == 0) ? 255 : 0, true);
	}

	if (coolingFanRpmPin >= 0)
	{
		pinModeNonDue(coolingFanRpmPin, INPUT_PULLUP, 1500);	// enable pullup and 1500Hz debounce filter (500Hz only worked up to 7000RPM)
	}

	// Hotend configuration
	nozzleDiameter = DefaultNozzleDiameter;
	filamentWidth = DefaultFilamentWidth;
	InitialiseInterrupts();

	lastTime = Time();
	longWait = lastTime;
}

// Specify which thermistor channel a particular heater uses
void Platform::SetThermistorNumber(size_t heater, size_t thermistor)
//pre(heater < HEATERS && thermistor < HEATERS)
{
	heaterAdcChannels[heater] = PinToAdcChannel(tempSensePins[thermistor]);
}

int Platform::GetThermistorNumber(size_t heater) const
{
	for (size_t thermistor = 0; thermistor < HEATERS; ++thermistor)
	{
		if (heaterAdcChannels[heater] == PinToAdcChannel(tempSensePins[thermistor]))
		{
			return thermistor;
		}
	}
	return -1;
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

	switch (nvData.zProbeType)
	{
	case 1:
	case 2:
		pinModeNonDue(nvData.zProbeModulationPin, OUTPUT);
		digitalWriteNonDue(nvData.zProbeModulationPin, HIGH);	// enable the IR LED
		break;

	case 3:
		pinModeNonDue(nvData.zProbeModulationPin, OUTPUT);
		digitalWriteNonDue(nvData.zProbeModulationPin, LOW);	// enable the alternate sensor
		break;

	case 4:
		pinModeNonDue(endStopPins[E0_AXIS], INPUT_PULLUP);
		break;

	case 5:
		break;	//TODO

	default:
		break;
	}
}

int Platform::GetZProbeChannel() const
{
	return (nvData.zProbeModulationPin == Z_PROBE_MOD_PIN07) ? 1 : 0;
}

void Platform::SetZProbeChannel(int chan)
{
	int temp = nvData.zProbeModulationPin;
	nvData.zProbeModulationPin = (chan == 1) ? Z_PROBE_MOD_PIN07 : Z_PROBE_MOD_PIN;
	if (autoSaveEnabled && temp != nvData.zProbeModulationPin)
	{
		WriteNvData();
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
			return (int) ((zProbeOnFilter.GetSum() + zProbeOffFilter.GetSum()) / (8 * numZProbeReadingsAveraged));

		case 2:		// Modulated IR sensor.
			// We assume that zProbeOnFilter and zProbeOffFilter average the same number of readings.
			// Because of noise, it is possible to get a negative reading, so allow for this.
			return (int) (((int32_t) zProbeOnFilter.GetSum() - (int32_t) zProbeOffFilter.GetSum())
					/ (int)(4 * numZProbeReadingsAveraged));
		case 5:
			return (int) ((zProbeOnFilter.GetSum() + zProbeOffFilter.GetSum()) / (8 * numZProbeReadingsAveraged));	//TODO this is temporary

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
			v1 = (int) (zProbeOnFilter.GetSum() / (4 * numZProbeReadingsAveraged));	// pass back the reading with IR turned on
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
		return DefaultZDive;
	}
}

void Platform::SetZProbeDiveHeight(float h)
{
	switch (nvData.zProbeType)
	{
	case 1:
	case 2:
		nvData.irZProbeParameters.diveHeight = h;
		break;
	case 3:
	case 5:
		nvData.alternateZProbeParameters.diveHeight = h;
		break;
	case 4:
		nvData.switchZProbeParameters.diveHeight = h;
		break;
	default:
		break;
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
	nvData.compatibility = me;
	ARRAY_INIT(nvData.ipAddress, IP_ADDRESS);
	ARRAY_INIT(nvData.netMask, NET_MASK);
	ARRAY_INIT(nvData.gateWay, GATE_WAY);
	ARRAY_INIT(nvData.macAddress, MAC_ADDRESS);

	nvData.zProbeType = 0;	// Default is to use no Z probe switch
	ARRAY_INIT(nvData.zProbeAxes, Z_PROBE_AXES);
	nvData.switchZProbeParameters.Init(0.0);
	nvData.irZProbeParameters.Init(Z_PROBE_STOP_HEIGHT);
	nvData.alternateZProbeParameters.Init(Z_PROBE_STOP_HEIGHT);
	nvData.zProbeModulationPin = Z_PROBE_MOD_PIN;

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
#endif
}

void Platform::ReadNvData()
{
#if FLASH_SAVE_ENABLED
	DueFlashStorage::read(FlashData::nvAddress, &nvData, sizeof(nvData));
	if (nvData.magic != FlashData::magicValue)
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

// AUX device
void Platform::Beep(int freq, int ms)
{
	// Send the beep command to the aux channel. There is no flow control on this port, so it can't block for long.
	scratchString.printf("{\"beep_freq\":%d,\"beep_length\":%d}\n", freq, ms);
	aux->Write(scratchString.Pointer(), true);
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
	Message(BOTH_MESSAGE, "Platform class exited.\n");
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
		Message(BOTH_ERROR_MESSAGE, "Attempt to emulate unsupported firmware.\n");
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

	if (debugCode == DiagnosticTest::TestSpinLockup)
	{
		for (;;) {}
	}

	line->Spin();
	aux->Spin();

	ClassReport(longWait);
}

void Platform::SoftwareReset(uint16_t reason)
{
	if (reason == SoftwareResetReason::erase)
	{
		cpu_irq_disable();
		flash_unlock(0x00080000, 0x000FFFFF, nullptr, nullptr);
		flash_clear_gpnvm(1);			// tell the system to boot from flash next time
 	}
	else
	{
		if (reason != SoftwareResetReason::user)
		{
			if (line->inWrite)
			{
				reason |= SoftwareResetReason::inUsbOutput;	// if we are resetting because we are stuck in a Spin function, record whether we are trying to send to USB
			}
			if (reprap.GetNetwork()->InLwip())
			{
				reason |= SoftwareResetReason::inLwipSpin;
			}
			if (aux->inWrite)
			{
				reason |= SoftwareResetReason::inAuxOutput;	// if we are resetting because we are stuck in a Spin function, record whether we are trying to send to aux
			}
		}
		reason |= reprap.GetSpinningModule();

		// Record the reason for the software reset
		SoftwareResetData temp;
		temp.magic = SoftwareResetData::magicValue;
		temp.resetReason = reason;
		GetStackUsage(NULL, NULL, &temp.neverUsedRam);
		if (reason != SoftwareResetReason::user)
		{
			strncpy(temp.lastMessage, messageString.Pointer(), sizeof(temp.lastMessage) - 1);
			temp.lastMessage[sizeof(temp.lastMessage) - 1] = 0;
		}
		else
		{
			temp.lastMessage[0] = 0;
		}

		// Save diagnostics data to Flash and reset the software
		DueFlashStorage::write(SoftwareResetData::nvAddress, &temp, sizeof(SoftwareResetData));
	}
	rstc_start_software_reset(RSTC);
	for(;;) {}
}

//*****************************************************************************************************************

// Interrupts

void TC3_Handler()
{
	TC1->TC_CHANNEL[0].TC_IDR = TC_IER_CPAS;	// disable the interrupt
#ifdef MOVE_DEBUG
	++numInterruptsExecuted;
	lastInterruptTime = Platform::GetInterruptClocks();
#endif
	reprap.Interrupt();
}

void TC4_Handler()
{
	TC_GetStatus(TC1, 1);
	reprap.GetNetwork()->Interrupt();
}

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
	// Timer interrupt for stepper motors
	// The clock rate we use is a compromise. Too fast and the 64-bit square roots take a long time to execute. Too slow and we lose resolution.
	// We choose a clock divisor of 32, which gives us 0.38us resolution. The next option is 128 which would give 1.524us resolution.
	pmc_set_writeprotect(false);
	pmc_enable_periph_clk((uint32_t) TC3_IRQn);
	TC_Configure(TC1, 0, TC_CMR_WAVE | TC_CMR_WAVSEL_UP | TC_CMR_TCCLKS_TIMER_CLOCK3);
	TC1 ->TC_CHANNEL[0].TC_IDR = ~(uint32_t)0;				// interrupts disabled for now
	TC_Start(TC1, 0);
	TC_GetStatus(TC1, 0);									// clear any pending interrupt
	NVIC_EnableIRQ(TC3_IRQn);

	// Timer interrupt to keep the networking timers running (called at 16Hz)
	pmc_enable_periph_clk((uint32_t) TC4_IRQn);
	TC_Configure(TC1, 1, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK2);
	uint32_t rc = (VARIANT_MCK/8)/16;						// 8 because we selected TIMER_CLOCK2 above
	TC_SetRA(TC1, 1, rc/2);									// 50% high, 50% low
	TC_SetRC(TC1, 1, rc);
	TC_Start(TC1, 1);
	TC1 ->TC_CHANNEL[1].TC_IER = TC_IER_CPCS;
	TC1 ->TC_CHANNEL[1].TC_IDR = ~TC_IER_CPCS;
	NVIC_EnableIRQ(TC4_IRQn);

	// Interrupt for 4-pin PWM fan sense line
	attachInterrupt(coolingFanRpmPin, FanInterrupt, FALLING);

	// Tick interrupt for ADC conversions
	tickState = 0;
	currentHeater = 0;

	active = true;							// this enables the tick interrupt, which keeps the watchdog happy
}

#pragma GCC push_options
#pragma GCC optimize ("O3")

// Schedule an interrupt at the specified clock count, or return true if that time is imminent or has passed already
/*static*/ bool Platform::ScheduleInterrupt(uint32_t tim)
{
	irqflags_t flags = cpu_irq_save();						// disable interrupts
	TC_SetRA(TC1, 0, tim);									// set up the compare register
	TC_GetStatus(TC1, 0);									// clear any pending interrupt
	int32_t diff = (int32_t)(tim - TC_ReadCV(TC1, 0));		// see how long we have to go
	bool ret;
	if (diff < 2)											// if less than 0.5us or already passed
	{
		ret = true;											// tell the caller to simulate an interrupt instead
	}
	else
	{
		ret = false;
		TC1 ->TC_CHANNEL[0].TC_IER = TC_IER_CPAS;			// enable the interrupt
#ifdef MOVE_DEBUG
		++numInterruptsScheduled;
		nextInterruptTime = tim;
		nextInterruptScheduledAt = Platform::GetInterruptClocks();
#endif
	}
	cpu_irq_restore(flags);									// restore interrupt enable status
	return ret;
}

#pragma GCC pop_options

#if 0	// not used
void Platform::DisableInterrupts()
{
	NVIC_DisableIRQ(TC3_IRQn);
	NVIC_DisableIRQ(TC4_IRQn);
}
#endif

// Process a 1ms tick interrupt
// This function must be kept fast so as not to disturb the stepper timing, so don't do any floating point maths in here.
// This is what we need to do:
// 0.  Kick the watchdog.
// 1.  Kick off a new ADC conversion.
// 2.  Fetch and process the result of the last ADC conversion.
// 3a. If the last ADC conversion was for the Z probe, toggle the modulation output if using a modulated IR sensor.
// 3b. If the last ADC reading was a thermistor reading, check for an over-temperature situation and turn off the heater if necessary.
//     We do this here because the usual polling loop sometimes gets stuck trying to send data to the USB port.

//#define TIME_TICK_ISR	1		// define this to store the tick ISR time in errorCodeBits

#pragma GCC push_options
#pragma GCC optimize ("O3")

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
		ThermistorAveragingFilter& currentFilter = const_cast<ThermistorAveragingFilter&>(thermistorFilters[currentHeater]);
		currentFilter.ProcessReading(GetAdcReading(heaterAdcChannels[currentHeater]));
		StartAdcConversion(zProbeAdcChannel);
		if (currentFilter.IsValid())
		{
			uint32_t sum = currentFilter.GetSum();
			if (sum < thermistorOverheatSums[currentHeater] || sum >= adDisconnectedReal * numThermistorReadingsAveraged)
			{
				// We have an over-temperature or bad reading from this thermistor, so turn off the heater
				// NB - the SetHeater function we call does floating point maths, but this is an exceptional situation so we allow it
				SetHeater(currentHeater, 0.0);
				errorCodeBits |= ErrorBadTemp;
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
		StartAdcConversion(heaterAdcChannels[currentHeater]);		// read a thermistor
		if (nvData.zProbeType == 2)									// if using a modulated IR sensor
		{
			digitalWriteNonDue(nvData.zProbeModulationPin, LOW);	// turn off the IR emitter
		}
		++tickState;
		break;

	case 4:			// last conversion started was the Z probe, with IR LED off if modulation is enabled
		const_cast<ZProbeAveragingFilter&>(zProbeOffFilter).ProcessReading(GetRawZProbeReading());
		// no break
	case 0:			// this is the state after initialisation, no conversion has been started
	default:
		StartAdcConversion(heaterAdcChannels[currentHeater]);		// read a thermistor
		if (nvData.zProbeType == 2 || nvData.zProbeType == 3)		// if using a modulated IR sensor
		{
			digitalWriteNonDue(nvData.zProbeModulationPin, HIGH);	// turn on the IR emitter
		}
		tickState = 1;
		break;
	}
#ifdef TIME_TICK_ISR
	uint32_t now2 = micros();
	if (now2 - now > errorCodeBits)
	{
		errorCodeBits = now2 - now;
	}
#endif
}

/*static*/ uint16_t Platform::GetAdcReading(adc_channel_num_t chan)
{
	uint16_t rslt = (uint16_t) adc_get_channel_value(ADC, chan);
	adc_disable_channel(ADC, chan);
	return rslt;
}

/*static*/ void Platform::StartAdcConversion(adc_channel_num_t chan)
{
	adc_enable_channel(ADC, chan);
	adc_start(ADC );
}

// Convert an Arduino Due pin number to the corresponding ADC channel number
/*static*/ adc_channel_num_t Platform::PinToAdcChannel(int pin)
{
	if (pin < A0)
	{
		pin += A0;
	}
	return (adc_channel_num_t) (int) g_APinDescription[pin].ulADCChannelNumber;
}

#pragma GCC pop_options

//*************************************************************************************************

// This diagnostics function is the first to be called, so it calls Message to start with.
// All other messages generated by this and other diagnostics functions must call AppendMessage.
void Platform::Diagnostics()
{
	Message(BOTH_MESSAGE, "Platform Diagnostics:\n");

	// Print memory stats and error codes to USB and copy them to the current webserver reply
	const char *ramstart = (char *) 0x20070000;
	const struct mallinfo mi = mallinfo();
	AppendMessage(BOTH_MESSAGE, "Memory usage:\n");
	AppendMessage(BOTH_MESSAGE, "Program static ram used: %d\n", &_end - ramstart);
	AppendMessage(BOTH_MESSAGE, "Dynamic ram used: %d\n", mi.uordblks);
	AppendMessage(BOTH_MESSAGE, "Recycled dynamic ram: %d\n", mi.fordblks);
	size_t currentStack, maxStack, neverUsed;
	GetStackUsage(&currentStack, &maxStack, &neverUsed);
	AppendMessage(BOTH_MESSAGE, "Current stack ram used: %d\n", currentStack);
	AppendMessage(BOTH_MESSAGE, "Maximum stack ram used: %d\n", maxStack);
	AppendMessage(BOTH_MESSAGE, "Never used ram: %d\n", neverUsed);

	// Show the up time and reason for the last reset
	const uint32_t now = (uint32_t)Time();		// get up time in seconds
	const char* resetReasons[8] = { "power up", "backup", "watchdog", "software", "external", "?", "?", "?" };
	AppendMessage(BOTH_MESSAGE, "Last reset %02d:%02d:%02d ago, cause: %s\n",
			(unsigned int)(now/3600), (unsigned int)((now % 3600)/60), (unsigned int)(now % 60),
			resetReasons[(REG_RSTC_SR & RSTC_SR_RSTTYP_Msk) >> RSTC_SR_RSTTYP_Pos]);

	// Show the error code stored at the last software reset
	{
		SoftwareResetData temp;
		temp.magic = 0;
		DueFlashStorage::read(SoftwareResetData::nvAddress, &temp, sizeof(SoftwareResetData));
		if (temp.magic == SoftwareResetData::magicValue)
		{
			AppendMessage(BOTH_MESSAGE, "Last software reset code & available RAM: 0x%04x, %u\n", temp.resetReason, temp.neverUsedRam);
			AppendMessage(BOTH_MESSAGE, "Spinning module during software reset: %s\n", moduleName[temp.resetReason & 0x0F]);
			if (temp.lastMessage[0])
			{
				AppendMessage(BOTH_MESSAGE, "Last message before reset: %s", temp.lastMessage); // usually ends with NL
			}
		}
	}

	// Show the current error codes
	AppendMessage(BOTH_MESSAGE, "Error status: %u\n", errorCodeBits);

	// Show the current probe position heights
	AppendMessage(BOTH_MESSAGE, "Bed probe heights:");
	for (size_t i = 0; i < MaxProbePoints; ++i)
	{
		AppendMessage(BOTH_MESSAGE, " %.3f", reprap.GetMove()->ZBedProbePoint(i));
	}
	AppendMessage(BOTH_MESSAGE, "\n");

	// Show the number of free entries in the file table
	unsigned int numFreeFiles = 0;
	for (size_t i = 0; i < MAX_FILES; i++)
	{
		if (!files[i]->inUse)
		{
			++numFreeFiles;
		}
	}
	AppendMessage(BOTH_MESSAGE, "Free file entries: %u\n", numFreeFiles);

	// Show the longest write time
	AppendMessage(BOTH_MESSAGE, "Longest block write time: %.1fms\n", FileStore::GetAndClearLongestWriteTime());

	reprap.Timing();

#ifdef MOVE_DEBUG
	AppendMessage(BOTH_MESSAGE, "Interrupts scheduled %u, done %u, last %u, next %u sched at %u, now %u\n",
			numInterruptsScheduled, numInterruptsExecuted, lastInterruptTime, nextInterruptTime, nextInterruptScheduledAt, GetInterruptClocks());
#endif
}

void Platform::DiagnosticTest(int d)
{
	switch (d)
	{
	case DiagnosticTest::TestWatchdog:
		SysTick ->CTRL &= ~(SysTick_CTRL_TICKINT_Msk);	// disable the system tick interrupt so that we get a watchdog timeout reset
		break;

	case DiagnosticTest::TestSpinLockup:
		debugCode = d;									// tell the Spin function to loop
		break;

	case DiagnosticTest::TestSerialBlock:				// write an arbitrary message via debugPrintf()
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
			Message(HOST_MESSAGE, "Class %s spinning.\n", moduleName[spinningModule]);
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

float Platform::GetTemperature(size_t heater) const
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
	if (rawTemp >= adDisconnectedVirtual)
	{
		return ABS_ZERO;		// thermistor is disconnected
	}

	// Correct for the low and high ADC offsets
	reading -= p.adcLowOffset;
	reading *= (adRangeVirtual + 1) / (adRangeVirtual + 1 + p.adcHighOffset - p.adcLowOffset);

	float resistance = reading * p.thermistorSeriesR / ((adRangeVirtual + 1) - reading);
	return (resistance <= p.GetRInf()) ? 2000.0			// thermistor short circuit, return a high temperature
			: ABS_ZERO + p.GetBeta() / log(resistance / p.GetRInf());
}

void Platform::SetPidParameters(size_t heater, const PidParameters& params)
{
	if (heater < HEATERS && params != nvData.pidParams[heater])
	{
		nvData.pidParams[heater] = params;
		if (autoSaveEnabled)
		{
			WriteNvData();
		}
	}
}
const PidParameters& Platform::GetPidParameters(size_t heater) const
{
	return nvData.pidParams[heater];
}

// power is a fraction in [0,1]

void Platform::SetHeater(size_t heater, float power)
{
	if (heatOnPins[heater] < 0)
		return;

	byte p = (byte) (255.0 * min<float>(1.0, max<float>(0.0, power)));
	analogWriteNonDue(heatOnPins[heater], (HEAT_ON == 0) ? 255 - p : p);
}

EndStopHit Platform::Stopped(size_t drive) const
{
	if (nvData.zProbeType > 0 && drive < AXES && nvData.zProbeAxes[drive])
	{
		return GetZProbeResult();			// using the Z probe as am endstop for this axis, so just get its result
	}

	if (endStopPins[drive] >= 0 && endStopType[drive] != EndStopType::noEndStop)
	{
		if (digitalReadNonDue(endStopPins[drive]) == ((endStopLogicLevel[drive]) ? 1 : 0))
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
	const int pin = directionPins[drive];
	if (pin >= 0)
	{
		bool d = (direction == FORWARDS) ? directions[drive] : !directions[drive];
		digitalWriteNonDue(pin, d);
	}
}

// Enable a drive. Must not be called from an ISR, or with interrupts disabled.
void Platform::EnableDrive(size_t drive)
{
	if (drive < DRIVES && driveState[drive] != DriveStatus::enabled)
	{
		driveState[drive] = DriveStatus::enabled;
		UpdateMotorCurrent(drive);						// the current may have been reduced by the idle timeout

		const int pin = enablePins[drive];
		if (pin >= 0)
		{
			digitalWriteNonDue(pin, ENABLE_DRIVE);
		}
	}
}

// Disable a drive, if it has a disable pin
void Platform::DisableDrive(size_t drive)
{
	if (drive < DRIVES)
	{
		const int pin = enablePins[drive];
		if (pin >= 0)
		{
			digitalWriteNonDue(pin, DISABLE_DRIVE);
			driveState[drive] = DriveStatus::disabled;
		}
	}
}

// Set a drive to idle hold if it is enabled. If it is disabled, leave it alone.
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
		unsigned short pot = (unsigned short)((0.256*current*8.0*senseResistor + maxStepperDigipotVoltage/2)/maxStepperDigipotVoltage);
		if (drive < 4)
		{
			mcpDuet.setNonVolatileWiper(potWipes[drive], pot);
			mcpDuet.setVolatileWiper(potWipes[drive], pot);
		}
		else
		{
			mcpExpansion.setNonVolatileWiper(potWipes[drive], pot);
			mcpExpansion.setVolatileWiper(potWipes[drive], pot);
		}
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

// Get current cooling fan speed on a scale between 0 and 1
float Platform::GetFanValue() const
{
	return coolingFanValue;
}

// This is a bit of a compromise - old RepRaps used fan speeds in the range
// [0, 255], which is very hardware dependent.  It makes much more sense
// to specify speeds in [0.0, 1.0].  This looks at the value supplied (which
// the G Code reader will get right for a float or an int) and attempts to
// do the right thing whichever the user has done.  This will only not work
// for an old-style fan speed of 1/255...
void Platform::SetFanValue(float speed)
{
	if (coolingFanPin >= 0)
	{
		byte p;
		if (speed <= 1.0)
		{
			p = (byte)(255.0 * max<float>(0.0, speed));
			coolingFanValue = speed;
		}
		else
		{
			p = (byte)speed;
			coolingFanValue = speed / 255.0;
		}

		// The cooling fan output pin gets inverted if HEAT_ON == 0
		analogWriteNonDue(coolingFanPin, (HEAT_ON == 0) ? (255 - p) : p, true);
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

//-----------------------------------------------------------------------------------------------------

FileStore* Platform::GetFileStore(const char* directory, const char* fileName, bool write)
{
	if (!fileStructureInitialised)
		return NULL;

	for (size_t i = 0; i < MAX_FILES; i++)
	{
		if (!files[i]->inUse)
		{
			files[i]->inUse = true;
			if (files[i]->Open(directory, fileName, write))
			{
				return files[i];
			}
			else
			{
				files[i]->inUse = false;
				return NULL;
			}
		}
	}
	Message(HOST_MESSAGE, "Max open file count exceeded.\n");
	return NULL;
}

MassStorage* Platform::GetMassStorage()
{
	return massStorage;
}

void Platform::Message(char type, const char* message, ...)
{
	va_list vargs;
	va_start(vargs, message);
	Message(type, message, vargs);
	va_end(vargs);
}

void Platform::Message(char type, const char* fmt, va_list vargs)
{
	messageString.vprintf(fmt, vargs);
	Message(type, messageString);
}

void Platform::Message(char type, const StringRef& message)
{
	if (message.Pointer() != messageString.Pointer())
	{
		// We might need to save the last message before a software reset is triggered
		messageString.copy(message.Pointer());
	}

	switch(type)
	{
	case FLASH_LED:
		// Message that is to flash an LED; the next two bytes define
		// the frequency and M/S ratio.

		break;

	case DISPLAY_MESSAGE:
		// Message that is to appear on a local display;  \f and \n should be supported.
		break;

	case HOST_MESSAGE:
	case DEBUG_MESSAGE:
		// Message that is to be sent to the host via USB; the H is not sent.
		if (line->GetOutputColumn() == 0)
		{
			for(uint8_t i = 0; i < messageIndent; i++)
			{
				line->Write(' ', type == DEBUG_MESSAGE);
			}
		}
		line->Write(message.Pointer(), type == DEBUG_MESSAGE);
		break;

	case WEB_MESSAGE:
		// Message that is to be sent to the web
		reprap.GetWebserver()->ResponseToWebInterface(message.Pointer(), false);
		break;

	case WEB_ERROR_MESSAGE:
		// Message that is to be sent to the web - flags an error
		reprap.GetWebserver()->ResponseToWebInterface(message.Pointer(), true);
		break;

	case BOTH_MESSAGE:
		// Message that is to be sent to the web & host
		if (line->GetOutputColumn() == 0)
		{
			for(uint8_t i = 0; i < messageIndent; i++)
			{
				line->Write(' ');
			}
		}
		line->Write(message.Pointer());
		reprap.GetWebserver()->ResponseToWebInterface(message.Pointer(), false);
		break;

	case BOTH_ERROR_MESSAGE:
		// Message that is to be sent to the web & host - flags an error
		// Make this the default behaviour too.

	default:
		if (line->GetOutputColumn() == 0)
		{
			for(uint8_t i = 0; i < messageIndent; i++)
			{
				line->Write(' ');
			}
		}
		line->Write(message.Pointer());
		reprap.GetWebserver()->ResponseToWebInterface(message.Pointer(), true);
		break;
	}
}

void Platform::AppendMessage(char type, const char* message, ...)
{
	va_list vargs;
	va_start(vargs, message);
	messageString.vprintf(message, vargs);
	va_end(vargs);
	AppendMessage(type, messageString);
}

void Platform::AppendMessage(char type, const StringRef& message)
{
	if (message.Pointer() != messageString.Pointer())
	{
		// We might need to save the last message before a software reset is triggered
		messageString.cat(message.Pointer());
	}

	switch(type)
	{
	case FLASH_LED:
		// Message that is to flash an LED; the next two bytes define
		// the frequency and M/S ratio.

		break;

	case DISPLAY_MESSAGE:
		// Message that is to appear on a local display;  \f and \n should be supported.

		break;

	case HOST_MESSAGE:
	case DEBUG_MESSAGE:
		// Message that is to be sent to the host via USB; the H is not sent.
		if (line->GetOutputColumn() == 0)
		{
			for(uint8_t i = 0; i < messageIndent; i++)
			{
				line->Write(' ', type == DEBUG_MESSAGE);
			}
		}
		line->Write(message.Pointer(), type == DEBUG_MESSAGE);
		break;

	case WEB_MESSAGE:
		// Message that is to be sent to the web
	case WEB_ERROR_MESSAGE:
		// Message that is to be sent to the web - flags an error
		reprap.GetWebserver()->AppendResponseToWebInterface(message.Pointer());
		break;

	case BOTH_MESSAGE:
		// Message that is to be sent to the web & host
		if (line->GetOutputColumn() == 0)
		{
			for(uint8_t i = 0; i < messageIndent; i++)
			{
				line->Write(' ');
			}
		}
		line->Write(message.Pointer());
		reprap.GetWebserver()->AppendResponseToWebInterface(message.Pointer());
		break;

	case BOTH_ERROR_MESSAGE:
		// Message that is to be sent to the web & host - flags an error
		// Make this the default behaviour too.

	default:
		if (line->GetOutputColumn() == 0)
		{
			for(uint8_t i = 0; i < messageIndent; i++)
			{
				line->Write(' ');
			}
		}
		line->Write(message.Pointer());
		reprap.GetWebserver()->AppendResponseToWebInterface(message.Pointer());
		break;
	}
}

bool Platform::AtxPower() const
{
	return (digitalReadNonDue(atxPowerPin) == HIGH);
}

void Platform::SetAtxPower(bool on)
{
	digitalWriteNonDue(atxPowerPin, (on) ? HIGH : LOW);
}


void Platform::SetElasticComp(size_t drive, float factor)
{
	if (drive < DRIVES)
	{
		elasticComp[drive] = factor;
	}
}

float Platform::ActualInstantDv(size_t drive) const
{
	float idv = instantDvs[drive];
	float eComp = elasticComp[drive];
	// If we are using elastic compensation then we need to limit the instantDv to avoid velocity mismatches
	return (eComp <= 0.0) ? idv : min<float>(idv, 1.0/(eComp * driveStepsPerUnit[drive]));
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
		SerialUSB.end();
		SerialUSB.begin(baudRates[0]);
		break;
	case 1:
		Serial.end();
		Serial.begin(baudRates[1]);
		break;
	default:
		break;
	}
}

/*********************************************************************************

 Files & Communication

 */

MassStorage::MassStorage(Platform* p) : platform(p)
{
	memset(&fileSystem, 0, sizeof(fileSystem));
}

void MassStorage::Init()
{
	// Initialize SD MMC stack
	hsmciPinsinit();
	sd_mmc_init();
	delay(20);

	bool abort = false;
	sd_mmc_err_t err;
	do {
		err = sd_mmc_check(0);
		if (err > SD_MMC_ERR_NO_CARD)
		{
			abort = true;
			delay(3000);	// Wait a few seconds, so users have a chance to see the following error message
		}
		else
		{
			abort = (err == SD_MMC_ERR_NO_CARD && platform->Time() > 5.0);
		}

		if (abort)
		{
			platform->Message(HOST_MESSAGE, "Cannot initialize the SD card: ");
			switch (err)
			{
				case SD_MMC_ERR_NO_CARD:
					platform->AppendMessage(HOST_MESSAGE, "Card not found\n");
					break;
				case SD_MMC_ERR_UNUSABLE:
					platform->AppendMessage(HOST_MESSAGE, "Card is unusable, try another one\n");
					break;
				case SD_MMC_ERR_SLOT:
					platform->AppendMessage(HOST_MESSAGE, "Slot unknown\n");
					break;
				case SD_MMC_ERR_COMM:
					platform->AppendMessage(HOST_MESSAGE, "General communication error\n");
					break;
				case SD_MMC_ERR_PARAM:
					platform->AppendMessage(HOST_MESSAGE, "Illegal input parameter\n");
					break;
				case SD_MMC_ERR_WP:
					platform->AppendMessage(HOST_MESSAGE, "Card write protected\n");
					break;
				default:
					platform->AppendMessage(HOST_MESSAGE, "Unknown (code %d)\m", err);
					break;
			}
			return;
		}
	} while (err != SD_MMC_OK);

	// Print some card details (optional)

	/*platform->Message(HOST_MESSAGE, "SD card detected!\nCapacity: %d\n", sd_mmc_get_capacity(0));
	platform->AppendMessage(HOST_MESSAGE, "Bus clock: %d\n", sd_mmc_get_bus_clock(0));
	platform->AppendMessage(HOST_MESSAGE, "Bus width: %d\nCard type: ", sd_mmc_get_bus_width(0));
	switch (sd_mmc_get_type(0))
	{
		case CARD_TYPE_SD | CARD_TYPE_HC:
			platform->AppendMessage(HOST_MESSAGE, "SDHC\n");
			break;
		case CARD_TYPE_SD:
			platform->AppendMessage(HOST_MESSAGE, "SD\n");
			break;
		case CARD_TYPE_MMC | CARD_TYPE_HC:
			platform->AppendMessage(HOST_MESSAGE, "MMC High Density\n");
			break;
		case CARD_TYPE_MMC:
			platform->AppendMessage(HOST_MESSAGE, "MMC\n");
			break;
		case CARD_TYPE_SDIO:
			platform->AppendMessage(HOST_MESSAGE, "SDIO\n");
			return;
		case CARD_TYPE_SD_COMBO:
			platform->AppendMessage(HOST_MESSAGE, "SD COMBO\n");
			break;
		case CARD_TYPE_UNKNOWN:
		default:
			platform->AppendMessage(HOST_MESSAGE, "Unknown\n");
			return;
	}*/

	// Mount the file system

	int mounted = f_mount(0, &fileSystem);
	if (mounted != FR_OK)
	{
		platform->Message(HOST_MESSAGE, "Can't mount filesystem 0: code %d\n", mounted);
	}
}

const char* MassStorage::CombineName(const char* directory, const char* fileName)
{
	int out = 0;
	int in = 0;

	if (directory != NULL)
	{
		while (directory[in] != 0 && directory[in] != '\n')
		{
			combinedName[out] = directory[in];
			in++;
			out++;
			if (out >= ARRAY_SIZE(combinedName))
			{
				platform->Message(BOTH_ERROR_MESSAGE, "CombineName() buffer overflow.");
				out = 0;
			}
		}
	}

	if (in > 0 && directory[in -1] != '/' && out < ARRAY_UPB(combinedName))
	{
		combinedName[out] = '/';
		out++;
	}

	in = 0;
	while (fileName[in] != 0 && fileName[in] != '\n')
	{
		combinedName[out] = fileName[in];
		in++;
		out++;
		if (out >= ARRAY_SIZE(combinedName))
		{
			platform->Message(BOTH_ERROR_MESSAGE, "CombineName() buffer overflow.");
			out = 0;
		}
	}
	combinedName[out] = 0;

	return combinedName;
}

// Open a directory to read a file list. Returns true if it contains any files, false otherwise.
bool MassStorage::FindFirst(const char *directory, FileInfo &file_info)
{
	TCHAR loc[MaxFilenameLength + 1];

	// Remove the trailing '/' from the directory name
	size_t len = strnlen(directory, ARRAY_UPB(loc));
	if (len == 0)
	{
		loc[0] = 0;
	}
	else if (directory[len - 1] == '/')
	{
		strncpy(loc, directory, len - 1);
		loc[len - 1] = 0;
	}
	else
	{
		strncpy(loc, directory, len);
		loc[len] = 0;
	}

	findDir.lfn = nullptr;
	FRESULT res = f_opendir(&findDir, loc);
	if (res == FR_OK)
	{
		FILINFO entry;
		entry.lfname = file_info.fileName;
		entry.lfsize = ARRAY_SIZE(file_info.fileName);

		for(;;)
		{
			res = f_readdir(&findDir, &entry);
			if (res != FR_OK || entry.fname[0] == 0) break;
			if (StringEquals(entry.fname, ".") || StringEquals(entry.fname, "..")) continue;

			file_info.isDirectory = (entry.fattrib & AM_DIR);
			file_info.size = entry.fsize;
			uint16_t day = entry.fdate & 0x1F;
			if (day == 0)
			{
				// This can happen if a transfer hasn't been processed completely.
				day = 1;
			}
			file_info.day = day;
			file_info.month = (entry.fdate & 0x01E0) >> 5;
			file_info.year = (entry.fdate >> 9) + 1980;
			if (file_info.fileName[0] == 0)
			{
				strncpy(file_info.fileName, entry.fname, ARRAY_SIZE(file_info.fileName));
			}

			return true;
		}
	}

	return false;
}

// Find the next file in a directory. Returns true if another file has been read.
bool MassStorage::FindNext(FileInfo &file_info)
{
	FILINFO entry;
	entry.lfname = file_info.fileName;
	entry.lfsize = ARRAY_SIZE(file_info.fileName);

	findDir.lfn = nullptr;
	if (f_readdir(&findDir, &entry) != FR_OK || entry.fname[0] == 0)
	{
		//f_closedir(findDir);
		return false;
	}

	file_info.isDirectory = (entry.fattrib & AM_DIR);
	file_info.size = entry.fsize;
	uint16_t day = entry.fdate & 0x1F;
	if (day == 0)
	{
		// This can happen if a transfer hasn't been processed completely.
		day = 1;
	}
	file_info.day = day;
	file_info.month = (entry.fdate & 0x01E0) >> 5;
	file_info.year = (entry.fdate >> 9) + 1980;
	if (file_info.fileName[0] == 0)
	{
		strncpy(file_info.fileName, entry.fname, ARRAY_SIZE(file_info.fileName));
	}

	return true;
}

// Month names. The first entry is used for invalid month numbers.
static const char *monthNames[13] = { "???", "Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec" };

// Returns the name of the specified month or '???' if the specified value is invalid.
const char* MassStorage::GetMonthName(const uint8_t month)
{
	return (month <= 12) ? monthNames[month] : monthNames[0];
}

// Delete a file or directory
bool MassStorage::Delete(const char* directory, const char* fileName)
{
	const char* location = (directory != NULL)
							? platform->GetMassStorage()->CombineName(directory, fileName)
								: fileName;
	if (f_unlink(location) != FR_OK)
	{
		platform->Message(BOTH_ERROR_MESSAGE, "Can't delete file %s\n", location);
		return false;
	}
	return true;
}

// Create a new directory
bool MassStorage::MakeDirectory(const char *parentDir, const char *dirName)
{
	const char* location = platform->GetMassStorage()->CombineName(parentDir, dirName);
	if (f_mkdir(location) != FR_OK)
	{
		platform->Message(BOTH_ERROR_MESSAGE, "Can't create directory %s\n", location);
		return false;
	}
	return true;
}

bool MassStorage::MakeDirectory(const char *directory)
{
	if (f_mkdir(directory) != FR_OK)
	{
		platform->Message(BOTH_ERROR_MESSAGE, "Can't create directory %s\n", directory);
		return false;
	}
	return true;
}

// Rename a file or directory
bool MassStorage::Rename(const char *oldFilename, const char *newFilename)
{
	if (f_rename(oldFilename, newFilename) != FR_OK)
	{
		platform->Message(BOTH_ERROR_MESSAGE, "Can't rename file or directory %s to %s\n", oldFilename, newFilename);
		return false;
	}
	return true;
}

// Check if the specified file exists
bool MassStorage::FileExists(const char *file) const
{
	FILINFO fil;
	fil.lfname = nullptr;
	return (f_stat(file, &fil) == FR_OK);
}

// Check if the specified directory exists
bool MassStorage::PathExists(const char *path) const
{
	DIR dir;
	dir.lfn = nullptr;
	return (f_opendir(&dir, path) == FR_OK);
}

bool MassStorage::PathExists(const char* directory, const char* subDirectory)
{
	const char* location = (directory != NULL)
							? platform->GetMassStorage()->CombineName(directory, subDirectory)
								: subDirectory;
	return PathExists(location);
}

//------------------------------------------------------------------------------------------------

FileStore::FileStore(Platform* p) : platform(p)
{
}

void FileStore::Init()
{
	bufferPointer = 0;
	inUse = false;
	writing = false;
	lastBufferEntry = 0;
	openCount = 0;
}

// Open a local file (for example on an SD card).
// This is protected - only Platform can access it.
bool FileStore::Open(const char* directory, const char* fileName, bool write)
{
	const char* location = (directory != NULL)
							? platform->GetMassStorage()->CombineName(directory, fileName)
								: fileName;
	writing = write;
	lastBufferEntry = FILE_BUF_LEN;

	FRESULT openReturn = f_open(&file, location, (writing) ?  FA_CREATE_ALWAYS | FA_WRITE : FA_OPEN_EXISTING | FA_READ);
	if (openReturn != FR_OK)
	{
		// We no longer report an error if opening a file in read mode fails unless debugging is enabled, because sometimes that is quite normal.
		// It is up to the caller to report an error if necessary.
		if (reprap.Debug(modulePlatform))
		{
			platform->Message(BOTH_ERROR_MESSAGE, "Can't open %s to %s, error code %d\n", location, (writing) ? "write" : "read", openReturn);
		}
		return false;
	}

	bufferPointer = (writing) ? 0 : FILE_BUF_LEN;
	inUse = true;
	openCount = 1;
	return true;
}

void FileStore::Duplicate()
{
	if (!inUse)
	{
		platform->Message(BOTH_ERROR_MESSAGE, "Attempt to dup a non-open file.\n");
		return;
	}
	++openCount;
}

bool FileStore::Close()
{
	if (!inUse)
	{
		platform->Message(BOTH_ERROR_MESSAGE, "Attempt to close a non-open file.\n");
		return false;
	}
	--openCount;
	if (openCount != 0)
	{
		return true;
	}
	bool ok = true;
	if (writing)
	{
		ok = Flush();
	}
	FRESULT fr = f_close(&file);
	inUse = false;
	writing = false;
	lastBufferEntry = 0;
	return ok && fr == FR_OK;
}

bool FileStore::Seek(FilePosition pos)
{
	if (!inUse)
	{
		platform->Message(BOTH_ERROR_MESSAGE, "Attempt to seek on a non-open file.\n");
		return false;
	}
	if (writing)
	{
		WriteBuffer();
	}
	FRESULT fr = f_lseek(&file, pos);
	bufferPointer = (writing) ? 0 : FILE_BUF_LEN;
	return fr == FR_OK;
}

FilePosition FileStore::GetPosition() const
{
	FilePosition pos = file.fptr;
	if (writing)
	{
		pos += bufferPointer;
	}
	else if (bufferPointer < lastBufferEntry)
	{
		pos -= (lastBufferEntry - bufferPointer);
	}
	return pos;
}

#if 0	// not currently used
bool FileStore::GoToEnd()
{
	return Seek(Length());
}
#endif

FilePosition FileStore::Length() const
{
	if (!inUse)
	{
		platform->Message(BOTH_ERROR_MESSAGE, "Attempt to size non-open file.\n");
		return 0;
	}
	return file.fsize;
}

float FileStore::FractionRead() const
{
	FilePosition len = Length();
	if (len == 0)
	{
		return 0.0;
	}

	return (float)GetPosition() / (float)len;
}

uint8_t FileStore::Status()
{
	if (!inUse)
		return (uint8_t)IOStatus::nothing;

	if (lastBufferEntry == FILE_BUF_LEN)
		return (uint8_t)IOStatus::byteAvailable;

	if (bufferPointer < lastBufferEntry)
		return (uint8_t)IOStatus::byteAvailable;

	return (uint8_t)IOStatus::nothing;
}

bool FileStore::ReadBuffer()
{
	FRESULT readStatus = f_read(&file, buf, FILE_BUF_LEN, &lastBufferEntry);	// Read a chunk of file
	if (readStatus)
	{
		platform->Message(BOTH_ERROR_MESSAGE, "Error reading file.\n");
		return false;
	}
	bufferPointer = 0;
	return true;
}

// Single character read via the buffer
bool FileStore::Read(char& b)
{
	if (!inUse)
	{
		platform->Message(BOTH_ERROR_MESSAGE, "Attempt to read from a non-open file.\n");
		return false;
	}

	if (bufferPointer >= FILE_BUF_LEN)
	{
		bool ok = ReadBuffer();
		if (!ok)
		{
			return false;
		}
	}

	if (bufferPointer >= lastBufferEntry)
	{
		b = 0;  // Good idea?
		return false;
	}

	b = (char) buf[bufferPointer];
	bufferPointer++;

	return true;
}

// Block read, doesn't use the buffer
int FileStore::Read(char* extBuf, unsigned int nBytes)
{
	if (!inUse)
	{
		platform->Message(BOTH_ERROR_MESSAGE, "Attempt to read from a non-open file.\n");
		return -1;
	}
	bufferPointer = FILE_BUF_LEN;	// invalidate the buffer
	UINT bytes_read;
	FRESULT readStatus = f_read(&file, extBuf, nBytes, &bytes_read);
	if (readStatus)
	{
		platform->Message(BOTH_ERROR_MESSAGE, "Error reading file.\n");
		return -1;
	}
	return (int)bytes_read;
}

bool FileStore::WriteBuffer()
{
	if (bufferPointer != 0)
	{
		bool ok = InternalWriteBlock((const char*)buf, bufferPointer);
		if (!ok)
		{
			platform->Message(BOTH_ERROR_MESSAGE, "Cannot write to file. Disc may be full.\n");
			return false;
		}
		bufferPointer = 0;
	}
	return true;
}

bool FileStore::Write(char b)
{
	if (!inUse)
	{
		platform->Message(BOTH_ERROR_MESSAGE, "Attempt to write byte to a non-open file.\n");
		return false;
	}
	buf[bufferPointer] = b;
	bufferPointer++;
	if (bufferPointer >= FILE_BUF_LEN)
	{
		return WriteBuffer();
	}
	return true;
}

bool FileStore::Write(const char* b)
{
	if (!inUse)
	{
		platform->Message(BOTH_ERROR_MESSAGE, "Attempt to write string to a non-open file.\n");
		return false;
	}
	int i = 0;
	while (b[i])
	{
		if (!Write(b[i++]))
		{
			return false;
		}
	}
	return true;
}

// Direct block write that bypasses the buffer. Used when uploading files.
bool FileStore::Write(const char *s, unsigned int len)
{
	if (!inUse)
	{
		platform->Message(BOTH_ERROR_MESSAGE, "Attempt to write block to a non-open file.\n");
		return false;
	}
	if (!WriteBuffer())
	{
		return false;
	}
	return InternalWriteBlock(s, len);
}

bool FileStore::InternalWriteBlock(const char *s, unsigned int len)
{
	unsigned int bytesWritten;
	uint32_t time = micros();
	FRESULT writeStatus = f_write(&file, s, len, &bytesWritten);
	time = micros() - time;
	if (time > longestWriteTime)
	{
		longestWriteTime = time;
	}
	if ((writeStatus != FR_OK) || (bytesWritten != len))
	{
		platform->Message(BOTH_ERROR_MESSAGE, "Cannot write to file. Disc may be full.\n");
		return false;
	}
	return true;
}

bool FileStore::Flush()
{
	if (!inUse)
	{
		platform->Message(BOTH_ERROR_MESSAGE, "Attempt to flush a non-open file.\n");
		return false;
	}
	if (!WriteBuffer())
	{
		return false;
	}
	return f_sync(&file) == FR_OK;
}

float FileStore::GetAndClearLongestWriteTime()
{
	float ret = (float)longestWriteTime/1000.0;
	longestWriteTime = 0;
	return ret;
}

uint32_t FileStore::longestWriteTime = 0;

//***************************************************************************************************

// Serial/USB class

Line::Line(Stream& p_iface) : iface(p_iface)
{
}

uint8_t Line::Status() const
{
	return inputNumChars == 0 ? (uint8_t)IOStatus::nothing : (uint8_t)IOStatus::byteAvailable;
}

// This is only ever called on initialisation, so we
// know the buffer won't overflow

void Line::InjectString(char* string)
{
	int i = 0;
	while(string[i])
	{
		inBuffer[(inputGetIndex + inputNumChars) % lineInBufsize] = string[i];
		inputNumChars++;
		i++;
	}
}

int Line::Read(char& b)
{
	if (inputNumChars == 0)
		return 0;
	b = inBuffer[inputGetIndex];
	inputGetIndex = (inputGetIndex + 1) % lineInBufsize;
	--inputNumChars;
	return 1;
}

void Line::Init()
{
	inputGetIndex = 0;
	inputNumChars = 0;
	outputGetIndex = 0;
	outputNumChars = 0;
	ignoringOutputLine = false;
	inWrite = 0;
	outputColumn = 0;
}

void Line::Spin()
{
	// Read the serial data in blocks to avoid excessive flow control
	if (inputNumChars <= lineInBufsize / 2)
	{
		int16_t target = iface.available() + (int16_t) inputNumChars;
		if (target > lineInBufsize)
		{
			target = lineInBufsize;
		}
		while ((int16_t) inputNumChars < target)
		{
			int incomingByte = iface.read();
			if (incomingByte < 0)
				break;
			inBuffer[(inputGetIndex + inputNumChars) % lineInBufsize] = (char) incomingByte;
			++inputNumChars;
		}
	}

	TryFlushOutput();
}

// Write a character to USB.
// If 'block' is true then we don't return until we have either written it to the USB port or put it in the buffer.
// Otherwise, if the buffer is full then we append ".\n" to the end of it, return immediately and ignore the rest
// of the data we are asked to print until we get a new line.
void Line::Write(char b, bool block)
{
	if (b == '\n')
	{
		outputColumn = 0;
	}
	else
	{
		++outputColumn;
	}

	if (block)
	{
		// We failed to print an unimportant message that (unusually) didn't finish in a newline
		ignoringOutputLine = false;
	}

	if (ignoringOutputLine)
	{
		// We have already failed to write some characters of this message line, so don't write any of it.
		// But try to start sending again after this line finishes.
		if (b == '\n')
		{
			ignoringOutputLine = false;
		}
		TryFlushOutput();		// this may help free things up
	}
	else
	{
		for(;;)
		{
			TryFlushOutput();
			if (block)
			{
				iface.flush();
			}

			if (outputNumChars == 0 && iface.canWrite() != 0)
			{
				// We can write the character directly into the USB output buffer
				++inWrite;
				iface.write(b);
				--inWrite;
				break;
			}
			else if (   outputNumChars + 2 < lineOutBufSize							// save 2 spaces in the output buffer
					 || (outputNumChars < lineOutBufSize && (block || b == '\n'))	//...unless doing blocking output or writing newline
					)
			{
				outBuffer[(outputGetIndex + outputNumChars) % lineOutBufSize] = b;
				++outputNumChars;
				break;
			}
			else if (!block)
			{
				if (outputNumChars + 2 == lineOutBufSize)
				{
					// We still have our 2 free characters, so append ".\n" to the line to indicate it was incomplete
					outBuffer[(outputGetIndex + outputNumChars) % lineOutBufSize] = '.';
					++outputNumChars;
					outBuffer[(outputGetIndex + outputNumChars) % lineOutBufSize] = '\n';
					++outputNumChars;
				}
				else
				{
					// As we don't have 2 spare characters in the buffer, we can't have written any of the current line.
					// So ignore the whole line.
				}
				ignoringOutputLine = true;
				break;
			}
		}

		TryFlushOutput();
		if (block)
		{
			iface.flush();
		}
	}
	// else discard the character
}

void Line::Write(const char* b, bool block)
{
	while (*b)
	{
		Write(*b++, block);
	}
}

void Line::TryFlushOutput()
{
	//debug
	//while (SerialUSB.canWrite() == 0) {}
	//end debug

	while (outputNumChars != 0 && iface.canWrite() != 0)
	{
		++inWrite;
		iface.write(outBuffer[outputGetIndex]);
		--inWrite;
		outputGetIndex = (outputGetIndex + 1) % lineOutBufSize;
		--outputNumChars;
	}
}

void Line::Flush()
{
	while (outputNumChars != 0)
	{
		TryFlushOutput();
	}
}

// End
