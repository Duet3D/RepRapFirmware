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

#define WINDOWED_SEND_PACKETS	(2)

extern char _end;
extern "C" char *sbrk(int i);

const uint8_t memPattern = 0xA5;

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
	//reprap.GetMove()->InterruptTime();  // Uncomment this line to time the interrupt routine on startup
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
	return kI == other.kI && kD == other.kD && kP == other.kP && fullBand == other.fullBand && pidMin == other.pidMin
			&& pidMax == other.pidMax && thermistorBeta == other.thermistorBeta && thermistorInfR == other.thermistorInfR
			&& thermistorSeriesR == other.thermistorSeriesR && adcLowOffset == other.adcLowOffset
			&& adcHighOffset == other.adcHighOffset;
}

//*************************************************************************************************
// Platform class

Platform::Platform() :
		tickState(0), fileStructureInitialised(false), active(false), errorCodeBits(0), debugCode(0)
{
	line = new Line();

	// Files

	massStorage = new MassStorage(this);

	for (int8_t i = 0; i < MAX_FILES; i++)
	{
		files[i] = new FileStore(this);
	}
}

//*******************************************************************************************************************

void Platform::Init()
{
	digitalWrite(atxPowerPin, LOW);		// ensure ATX power is off by default
	pinMode(atxPowerPin, OUTPUT);

	DueFlashStorage::init();
	DueFlashStorage::read(nvAddress, &nvData, sizeof(nvData));
	if (nvData.magic != FlashData::magicValue)
	{
		// Nonvolatile data has not been initialized since the firmware was last written, so set up default values
		nvData.compatibility = me;
		nvData.ipAddress = IP_ADDRESS;
		nvData.netMask = NET_MASK;
		nvData.gateWay = GATE_WAY;

		nvData.zProbeType = 0;	// Default is to use the switch
		nvData.switchZProbeParameters.Init();
		nvData.switchZProbeParameters.height = 0.0; // Assume the nozzle is at Z=0 when the switch is triggered
		nvData.irZProbeParameters.Init();
		nvData.ultrasonicZProbeParameters.Init();

		for (size_t i = 0; i < HEATERS; ++i)
		{
			PidParameters& pp = nvData.pidParams[i];
			pp.thermistorSeriesR = defaultThermistorSeriesRs[i];
			pp.SetThermistorR25AndBeta(defaultThermistor25RS[i], defaultThermistorBetas[i]);
			pp.kI = defaultPidKis[i];
			pp.kD = defaultPidKds[i];
			pp.kP = defaultPidKps[i];
			pp.fullBand = defaultFullBand[i];
			pp.pidMin = defaultPidMin[i];
			pp.pidMax = defaultPidMax[i];
			pp.adcLowOffset = pp.adcHighOffset = 0.0;
		}

		nvData.resetReason = 0;
		GetStackUsage(NULL, NULL, &nvData.neverUsedRam);
		nvData.magic = FlashData::magicValue;
		WriteNvData();
	}

	line->Init();
	messageIndent = 0;

	massStorage->Init();

	for (size_t i = 0; i < MAX_FILES; i++)
	{
		files[i]->Init();
	}

	fileStructureInitialised = true;

	mcp.begin();

	sysDir = SYS_DIR;
	configFile = CONFIG_FILE;

	// DRIVES

	stepPins = STEP_PINS;
	directionPins = DIRECTION_PINS;
	enablePins = ENABLE_PINS;
	disableDrives = DISABLE_DRIVES;
	lowStopPins = LOW_STOP_PINS;
	highStopPins = HIGH_STOP_PINS;
	maxFeedrates = MAX_FEEDRATES;
	accelerations = ACCELERATIONS;
	driveStepsPerUnit = DRIVE_STEPS_PER_UNIT;
	instantDvs = INSTANT_DVS;
	potWipes = POT_WIPES;
	senseResistor = SENSE_RESISTOR;
	maxStepperDigipotVoltage = MAX_STEPPER_DIGIPOT_VOLTAGE;

	// Z PROBE

	zProbePin = Z_PROBE_PIN;
	zProbeModulationPin = Z_PROBE_MOD_PIN;
	zProbeAdcChannel = PinToAdcChannel(zProbePin);
	InitZProbe();

	// AXES

	axisLengths = AXIS_LENGTHS;
	homeFeedrates = HOME_FEEDRATES;
	headOffsets = HEAD_OFFSETS;

	// HEATERS - Bed is assumed to be the first

	tempSensePins = TEMP_SENSE_PINS;
	heatOnPins = HEAT_ON_PINS;
	heatSampleTime = HEAT_SAMPLE_TIME;
	standbyTemperatures = STANDBY_TEMPERATURES;
	activeTemperatures = ACTIVE_TEMPERATURES;
	coolingFanPin = COOLING_FAN_PIN;

	webDir = WEB_DIR;
	gcodeDir = GCODE_DIR;
	tempDir = TEMP_DIR;

	for (size_t i = 0; i < DRIVES; i++)
	{
		if (stepPins[i] >= 0)
		{
			if (i > Z_AXIS)
				pinModeNonDue(stepPins[i], OUTPUT);
			else
				pinMode(stepPins[i], OUTPUT);
		}
		if (directionPins[i] >= 0)
		{
			if (i > Z_AXIS)
				pinModeNonDue(directionPins[i], OUTPUT);
			else
				pinMode(directionPins[i], OUTPUT);
		}
		if (enablePins[i] >= 0)
		{
			if (i >= Z_AXIS)
				pinModeNonDue(enablePins[i], OUTPUT);
			else
				pinMode(enablePins[i], OUTPUT);
		}
		Disable(i);
		driveEnabled[i] = false;
	}

	for (size_t i = 0; i < AXES; i++)
	{
		if (lowStopPins[i] >= 0)
		{
			pinMode(lowStopPins[i], INPUT);
			digitalWrite(lowStopPins[i], HIGH); // Turn on pullup
		}
		if (highStopPins[i] >= 0)
		{
			pinMode(highStopPins[i], INPUT);
			digitalWrite(highStopPins[i], HIGH); // Turn on pullup
		}
	}

	for (size_t i = 0; i < HEATERS; i++)
	{
		if (heatOnPins[i] >= 0)
		{
			if (i == 0)		// heater 0 (bed heater) is a standard Arduino PWM pin
			{
				pinMode(heatOnPins[i], OUTPUT);
			}
			else
			{
				pinModeNonDue(heatOnPins[i], OUTPUT);
			}
		}

		thermistorFilters[i].Init();
		heaterAdcChannels[i] = PinToAdcChannel(tempSensePins[i]);

		// Calculate and store the ADC average sum that corresponds to an overheat condition, so that we can check is quickly in the tick ISR
		float thermistorOverheatResistance = nvData.pidParams[i].GetRInf()
				* exp(-nvData.pidParams[i].GetBeta() / (BAD_HIGH_TEMPERATURE - ABS_ZERO));
		float thermistorOverheatAdcValue = (adRangeReal + 1) * thermistorOverheatResistance
				/ (thermistorOverheatResistance + nvData.pidParams[i].thermistorSeriesR);
		thermistorOverheatSums[i] = (uint32_t) (thermistorOverheatAdcValue + 0.9) * numThermistorReadingsAveraged;
	}

	if (coolingFanPin >= 0)
	{
		pinMode(coolingFanPin, OUTPUT);
		analogWrite(coolingFanPin, (HEAT_ON == 0) ? 255 : 0);		// turn auxiliary cooling fan off
	}

	InitialiseInterrupts();

	addToTime = 0.0;
	lastTimeCall = 0;
	lastTime = Time();
	longWait = lastTime;
}

void Platform::InitZProbe()
{
	zProbeOnFilter.Init();
	zProbeOffFilter.Init();

	if (nvData.zProbeType == 1 || nvData.zProbeType == 2)
	{
		pinMode(zProbeModulationPin, OUTPUT);
		digitalWrite(zProbeModulationPin, HIGH);	// enable the IR LED
		SetZProbing(false);
	}
	else if (nvData.zProbeType == 3)
	{
		pinMode(zProbeModulationPin, OUTPUT);
		digitalWrite(zProbeModulationPin, LOW);	// enable the ultrasonic sensor
		SetZProbing(false);
	}
}

int Platform::GetRawZHeight() const
{
	return (nvData.zProbeType != 0) ? analogRead(zProbePin) : 0;
}

// Return the Z probe data.
// The ADC readings are 12 bits, so we convert them to 10-bit readings for compatibility with the old firmware.
int Platform::ZProbe()
{
	if (zProbeOnFilter.IsValid() && zProbeOffFilter.IsValid())
	{
		switch (nvData.zProbeType)
		{
		case 1:
		case 3:
			// Simple IR sensor, or direct-mode ultrasonic sensor
			return (int) ((zProbeOnFilter.GetSum() + zProbeOffFilter.GetSum()) / (8 * numZProbeReadingsAveraged));

		case 2:
			// Modulated IR sensor. We assume that zProbeOnFilter and zprobeOffFilter average the same number of readings.
			// Because of noise, it is possible to get a negative reading, so allow for this.
			return (int) (((int32_t) zProbeOnFilter.GetSum() - (int32_t) zProbeOffFilter.GetSum())
					/ (4 * numZProbeReadingsAveraged));

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

float Platform::ZProbeStopHeight() const
{

	switch (nvData.zProbeType)
	{
	case 0:
		return nvData.switchZProbeParameters.GetStopHeight(GetTemperature(0));
	case 1:
	case 2:
		return nvData.irZProbeParameters.GetStopHeight(GetTemperature(0));
	case 3:
		return nvData.ultrasonicZProbeParameters.GetStopHeight(GetTemperature(0));
	default:
		return 0;
	}
}

void Platform::SetZProbeType(int pt)
{
	int newZProbeType = (pt >= 0 && pt <= 3) ? pt : 0;
	if (newZProbeType != nvData.zProbeType)
	{
		nvData.zProbeType = newZProbeType;
		WriteNvData();
	}
	InitZProbe();
}

bool Platform::GetZProbeParameters(struct ZProbeParameters& params) const
{
	switch (nvData.zProbeType)
	{
	case 0:
		params = nvData.switchZProbeParameters;
		return true;
	case 1:
	case 2:
		params = nvData.irZProbeParameters;
		return true;
	case 3:
		params = nvData.ultrasonicZProbeParameters;
		return true;
	default:
		return false;
	}
}

bool Platform::SetZProbeParameters(const struct ZProbeParameters& params)
{
	switch (nvData.zProbeType)
	{
	case 0:
		if (nvData.switchZProbeParameters != params)
		{
			nvData.switchZProbeParameters = params;
			WriteNvData();
		}
	case 1:
	case 2:
		if (nvData.irZProbeParameters != params)
		{
			nvData.irZProbeParameters = params;
			WriteNvData();
		}
		return true;
	case 3:
		if (nvData.ultrasonicZProbeParameters != params)
		{
			nvData.ultrasonicZProbeParameters = params;
			WriteNvData();
		}
		return true;
	default:
		return false;
	}
}

// Return true if we must home X and Y before we home Z (i.e. we are using a bed probe)
bool Platform::MustHomeXYBeforeZ() const
{
	return nvData.zProbeType != 0;
}

void Platform::WriteNvData()
{
	DueFlashStorage::write(nvAddress, &nvData, sizeof(nvData));
}

void Platform::SetZProbing(bool starting)
{
}

// Note: the use of floating point time will cause the resolution to degrade over time.
// For example, 1ms time resolution will only be available for about half an hour from startup.
// Personally, I (dc42) would rather just maintain and provide the time in milliseconds in a uint32_t.
// This would wrap round after about 49 days, but that isn't difficult to handle.
float Platform::Time()
{
	unsigned long now = micros();
	if (now < lastTimeCall) // Has timer overflowed?
		addToTime += ((float) ULONG_MAX) * TIME_FROM_REPRAP;
	lastTimeCall = now;
	return addToTime + TIME_FROM_REPRAP * (float) now;
}

void Platform::Exit()
{
	Message(HOST_MESSAGE, "Platform class exited.\n");
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
		Message(HOST_MESSAGE, "Attempt to emulate unsupported firmware.\n");
		return;
	}
	if (c == reprapFirmware)
	{
		c = me;
	}
	if (nvData.compatibility != c)
	{
		nvData.compatibility = c;
		WriteNvData();
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
	if (changed)
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

	if (Time() - lastTime < 0.006)
		return;
	lastTime = Time();
	ClassReport("Platform", longWait);

}

void Platform::SoftwareReset(uint16_t reason)
{
	if (reason != 0)
	{
		if (line->inUsbWrite)
		{
			reason |= SoftwareResetReason::inUsbOutput;	// if we are resetting because we are stuck in a Spin function, record whether we are trying to send to USB
		}
	}

	if (reason != 0 || reason != nvData.resetReason)
	{
		nvData.resetReason = reason;
		GetStackUsage(NULL, NULL, &nvData.neverUsedRam);
		WriteNvData();
	}

	rstc_start_software_reset(RSTC);
	for(;;) {}
}

//*****************************************************************************************************************

// Interrupts

void TC3_Handler()
{
	TC_GetStatus(TC1, 0);
	reprap.Interrupt();
}

void Platform::InitialiseInterrupts()
{
	// Timer interrupt for stepper motors
	pmc_set_writeprotect(false);
	pmc_enable_periph_clk((uint32_t) TC3_IRQn);
	TC_Configure(TC1, 0, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK4);
	TC1 ->TC_CHANNEL[0].TC_IER = TC_IER_CPCS;
	TC1 ->TC_CHANNEL[0].TC_IDR = ~TC_IER_CPCS;
	SetInterrupt(STANDBY_INTERRUPT_RATE);

	// Tick interrupt for ADC conversions
	tickState = 0;
	currentHeater = 0;

	active = true;							// this enables the tick interrupt, which keeps the watchdog happy
}

void Platform::DisableInterrupts()
{
	NVIC_DisableIRQ(TC3_IRQn);
}

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
		const_cast<ZProbeAveragingFilter&>(zProbeOnFilter).ProcessReading(GetAdcReading(zProbeAdcChannel));
		StartAdcConversion(heaterAdcChannels[currentHeater]);	// read a thermistor
		if (nvData.zProbeType == 2)								// if using a modulated IR sensor
		{
			digitalWrite(Z_PROBE_MOD_PIN, LOW);					// turn off the IR emitter
		}
		++tickState;
		break;

	case 4:			// last conversion started was the Z probe, with IR LED off if modulation is enabled
		const_cast<ZProbeAveragingFilter&>(zProbeOffFilter).ProcessReading(GetAdcReading(zProbeAdcChannel));
		// no break
	case 0:			// this is the state after initialisation, no conversion has been started
	default:
		StartAdcConversion(heaterAdcChannels[currentHeater]);	// read a thermistor
		if (nvData.zProbeType == 2)								// if using a modulated IR sensor
		{
			digitalWrite(Z_PROBE_MOD_PIN, HIGH);				// turn on the IR emitter
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

/*static*/uint16_t Platform::GetAdcReading(adc_channel_num_t chan)
{
	uint16_t rslt = (uint16_t) adc_get_channel_value(ADC, chan);
	adc_disable_channel(ADC, chan);
	return rslt;
}

/*static*/void Platform::StartAdcConversion(adc_channel_num_t chan)
{
	adc_enable_channel(ADC, chan);
	adc_start(ADC );
}

// Convert an Arduino Due pin number to the corresponding ADC channel number
/*static*/adc_channel_num_t Platform::PinToAdcChannel(int pin)
{
	if (pin < A0)
	{
		pin += A0;
	}
	return (adc_channel_num_t) (int) g_APinDescription[pin].ulADCChannelNumber;
}

//*************************************************************************************************

void Platform::Diagnostics()
{
	Message(HOST_MESSAGE, "Platform Diagnostics:\n");
}

void Platform::SetDebug(int d)
{
	switch (d)
	{
	case DiagnosticTest::TestWatchdog:
		SysTick ->CTRL &= ~(SysTick_CTRL_TICKINT_Msk);	// disable the system tick interrupt so that we get a watchdog timeout reset
		break;

	case DiagnosticTest::TestSpinLockup:
		debugCode = d;									// tell the Spin function to loop
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

// Print memory stats and error codes to USB and copy them to the current webserver reply
void Platform::PrintMemoryUsage()
{
	const char *ramstart = (char *) 0x20070000;
	const struct mallinfo mi = mallinfo();
	Message(HOST_MESSAGE, "\n");
	Message(HOST_MESSAGE, "Memory usage:\n\n");
	snprintf(scratchString, STRING_LENGTH, "Program static ram used: %d\n", &_end - ramstart);
	reprap.GetWebserver()->HandleReply(scratchString, false);
	Message(HOST_MESSAGE, scratchString);
	snprintf(scratchString, STRING_LENGTH, "Dynamic ram used: %d\n", mi.uordblks);
	reprap.GetWebserver()->AppendReply(scratchString);
	Message(HOST_MESSAGE, scratchString);
	snprintf(scratchString, STRING_LENGTH, "Recycled dynamic ram: %d\n", mi.fordblks);
	reprap.GetWebserver()->AppendReply(scratchString);
	Message(HOST_MESSAGE, scratchString);
	size_t currentStack, maxStack, neverUsed;
	GetStackUsage(&currentStack, &maxStack, &neverUsed);
	snprintf(scratchString, STRING_LENGTH, "Current stack ram used: %d\n", currentStack);
	reprap.GetWebserver()->AppendReply(scratchString);
	Message(HOST_MESSAGE, scratchString);
	snprintf(scratchString, STRING_LENGTH, "Maximum stack ram used: %d\n", maxStack);
	reprap.GetWebserver()->AppendReply(scratchString);
	Message(HOST_MESSAGE, scratchString);
	snprintf(scratchString, STRING_LENGTH, "Never used ram: %d\n", neverUsed);
	reprap.GetWebserver()->AppendReply(scratchString);
	Message(HOST_MESSAGE, scratchString);

	// Show the up time and reason for the last reset
	const uint32_t now = (uint32_t)Time();		// get up time in seconds
	const char* resetReasons[8] = { "power up", "backup", "watchdog", "software", "external", "?", "?", "?" };
	snprintf(scratchString, STRING_LENGTH, "Last reset %02d:%02d:%02d ago, cause: %s\n",
			(unsigned int)(now/3600), (unsigned int)((now % 3600)/60), (unsigned int)(now % 60),
			resetReasons[(REG_RSTC_SR & RSTC_SR_RSTTYP_Msk) >> RSTC_SR_RSTTYP_Pos]);
	reprap.GetWebserver()->AppendReply(scratchString);
	Message(HOST_MESSAGE, scratchString);

	// Show the error code stored at the last software reset
	snprintf(scratchString, STRING_LENGTH, "Last software reset code & available RAM: 0x%04x, %u\n", nvData.resetReason, nvData.neverUsedRam);
	reprap.GetWebserver()->AppendReply(scratchString);
	Message(HOST_MESSAGE, scratchString);

	// Show the current error codes
	snprintf(scratchString, STRING_LENGTH, "Error status: %u\n", errorCodeBits);
	reprap.GetWebserver()->AppendReply(scratchString);
	Message(HOST_MESSAGE, scratchString);

	// Show the current probe position heights
	strncpy(scratchString, "Bed probe heights:", STRING_LENGTH);
	for (size_t i = 0; i < NUMBER_OF_PROBE_POINTS; ++i)
	{
		sncatf(scratchString, STRING_LENGTH, " %.3f", reprap.GetMove()->zBedProbePoint(i));
	}
	strncat(scratchString, "\n", STRING_LENGTH);
	reprap.GetWebserver()->AppendReply(scratchString);
	Message(HOST_MESSAGE, scratchString);
}

void Platform::ClassReport(char* className, float &lastTime)
{
	if (!reprap.Debug())
		return;
	if (Time() - lastTime < LONG_TIME)
		return;
	lastTime = Time();
	snprintf(scratchString, STRING_LENGTH, "Class %s spinning.\n", className);
	Message(HOST_MESSAGE, scratchString);
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
		WriteNvData();
	}
}
const PidParameters& Platform::GetPidParameters(size_t heater)
{
	return nvData.pidParams[heater];
}

// power is a fraction in [0,1]

void Platform::SetHeater(size_t heater, const float& power)
{
	if (heatOnPins[heater] < 0)
		return;

	byte p = (byte) (255.0 * fmin(1.0, fmax(0.0, power)));
	if (HEAT_ON == 0)
		p = 255 - p;
	if (heater == 0)
		analogWrite(heatOnPins[heater], p);
	else
		analogWriteNonDue(heatOnPins[heater], p);
}

EndStopHit Platform::Stopped(int8_t drive)
{
	if (nvData.zProbeType > 0)
	{  // Z probe is used for both X and Z.
		if (drive != Y_AXIS)
		{
			int zProbeVal = ZProbe();
			int zProbeADValue =
					(nvData.zProbeType == 3) ?
							nvData.ultrasonicZProbeParameters.adcValue : nvData.irZProbeParameters.adcValue;
			if (zProbeVal >= zProbeADValue)
				return lowHit;
			else if (zProbeVal * 10 >= zProbeADValue * 9)	// if we are at/above 90% of the target value
				return lowNear;
			else
				return noStop;
		}
	}

	if (lowStopPins[drive] >= 0)
	{
		if (digitalRead(lowStopPins[drive]) == ENDSTOP_HIT)
			return lowHit;
	}
	if (highStopPins[drive] >= 0)
	{
		if (digitalRead(highStopPins[drive]) == ENDSTOP_HIT)
			return highHit;
	}
	return noStop;
}

void Platform::SetDirection(byte drive, bool direction)
{
	if(directionPins[drive] < 0)
		return;
	if(drive == AXES)
		digitalWriteNonDue(directionPins[drive], direction);
	else
		digitalWrite(directionPins[drive], direction);
}

void Platform::Disable(byte drive)
{
	if(enablePins[drive] < 0)
		  return;
	if(drive >= Z_AXIS)
		digitalWriteNonDue(enablePins[drive], DISABLE);
	else
		digitalWrite(enablePins[drive], DISABLE);
	driveEnabled[drive] = false;
}

void Platform::Step(byte drive)
{
	if(stepPins[drive] < 0)
		return;
	if(!driveEnabled[drive] && enablePins[drive] >= 0)
	{
		if(drive >= Z_AXIS)
			digitalWriteNonDue(enablePins[drive], ENABLE);
		else
			digitalWrite(enablePins[drive], ENABLE);
		driveEnabled[drive] = true;
	}
	if(drive == AXES)
	{
		digitalWriteNonDue(stepPins[drive], 0);
		digitalWriteNonDue(stepPins[drive], 1);
	} else
	{
		digitalWrite(stepPins[drive], 0);
		digitalWrite(stepPins[drive], 1);
	}
}

// current is in mA
void Platform::SetMotorCurrent(byte drive, float current)
{
	unsigned short pot = (unsigned short)(0.256*current*8.0*senseResistor/maxStepperDigipotVoltage);
//	Message(HOST_MESSAGE, "Set pot to: ");
//	snprintf(scratchString, STRING_LENGTH, "%d", pot);
//	Message(HOST_MESSAGE, scratchString);
//	Message(HOST_MESSAGE, "\n");
	mcp.setNonVolatileWiper(potWipes[drive], pot);
	mcp.setVolatileWiper(potWipes[drive], pot);
}

void Platform::CoolingFan(float speed)
{
	if(coolingFanPin > 0)
	{
		// The cooling fan output pin gets inverted if HEAT_ON == 0
		analogWriteNonDue(coolingFanPin, (uint32_t)( ((HEAT_ON == 0) ? (1.0 - speed) : speed) * 255.0));
	}
}

// Interrupts

void Platform::SetInterrupt(float s) // Seconds
{
  if (s <= 0.0)
  {
    //NVIC_DisableIRQ(TC3_IRQn);
    Message(HOST_MESSAGE, "Negative interrupt!\n");
    s = STANDBY_INTERRUPT_RATE;
  }
  uint32_t rc = (uint32_t)( (((long)(TIME_TO_REPRAP*s))*84l)/128l );
  TC_SetRA(TC1, 0, rc/2); //50% high, 50% low
  TC_SetRC(TC1, 0, rc);
  TC_Start(TC1, 0);
  NVIC_EnableIRQ(TC3_IRQn);
}

int8_t Line::Status() const
{
//	if(alternateInput != NULL)
//		return alternateInput->Status();
	return inputNumChars == 0 ? nothing : byteAvailable;
}

int Line::Read(char& b)
{
//  if(alternateInput != NULL)
//	return alternateInput->Read(b);

	  if (inputNumChars == 0) return 0;
	  b = inBuffer[inputGetIndex];
	  inputGetIndex = (inputGetIndex + 1) % lineInBufsize;
	  --inputNumChars;
	  return 1;
}

//-----------------------------------------------------------------------------------------------------

FileStore* Platform::GetFileStore(const char* directory, const char* fileName, bool write)
{
	if (!fileStructureInitialised)
		return NULL;

	for (int i = 0; i < MAX_FILES; i++)
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

void Platform::Message(char type, const char* message)
{
	switch (type)
	{
	case FLASH_LED:
		// Message that is to flash an LED; the next two bytes define
		// the frequency and M/S ratio.

		break;

	case DISPLAY_MESSAGE:
		// Message that is to appear on a local display;  \f and \n should be supported.
	case HOST_MESSAGE:
	default:

//    FileStore* m = GetFileStore(GetWebDir(), MESSAGE_FILE, true);
//    if(m != NULL)
//    {
//    	m->GoToEnd();
//    	m->Write(message);
//    	m->Close();
//    } else
//    	line->Write("Can't open message file.\n");
		for (uint8_t i = 0; i < messageIndent; i++)
		{
			line->Write(' ');
		}
		line->Write(message);
	}
}

void Platform::SetAtxPower(bool on)
{
	digitalWrite(atxPowerPin, (on) ? HIGH : LOW);
}


/*********************************************************************************

 Files & Communication

 */

MassStorage::MassStorage(Platform* p)
{
	platform = p;
}

void MassStorage::Init()
{
	hsmciPinsinit();
	// Initialize SD MMC stack
	sd_mmc_init();
	delay(20);
	int sdPresentCount = 0;
	while ((CTRL_NO_PRESENT == sd_mmc_check(0)) && (sdPresentCount < 5))
	{
		//platform->Message(HOST_MESSAGE, "Please plug in the SD card.\n");
		//delay(1000);
		sdPresentCount++;
	}

	if (sdPresentCount >= 5)
	{
		platform->Message(HOST_MESSAGE, "Can't find the SD card.\n");
		return;
	}

	//print card info

//	SerialUSB.print("sd_mmc_card->capacity: ");
//	SerialUSB.print(sd_mmc_get_capacity(0));
//	SerialUSB.print(" bytes\n");
//	SerialUSB.print("sd_mmc_card->clock: ");
//	SerialUSB.print(sd_mmc_get_bus_clock(0));
//	SerialUSB.print(" Hz\n");
//	SerialUSB.print("sd_mmc_card->bus_width: ");
//	SerialUSB.println(sd_mmc_get_bus_width(0));

	memset(&fileSystem, 0, sizeof(FATFS));
	//f_mount (LUN_ID_SD_MMC_0_MEM, NULL);
	//int mounted = f_mount(LUN_ID_SD_MMC_0_MEM, &fileSystem);
	int mounted = f_mount(0, &fileSystem);
	if (mounted != FR_OK)
	{
		platform->Message(HOST_MESSAGE, "Can't mount filesystem 0: code ");
		snprintf(scratchString, STRING_LENGTH, "%d", mounted);
		platform->Message(HOST_MESSAGE, scratchString);
		platform->Message(HOST_MESSAGE, "\n");
	}
}

const char* MassStorage::CombineName(const char* directory, const char* fileName)
{
	int out = 0;
	int in = 0;

//  scratchString[out] = '/';
//  out++;

	if (directory != NULL)
	{
		//if(directory[in] == '/')
		//  in++;
		while (directory[in] != 0 && directory[in] != '\n')    // && directory[in] != '/')
		{
			scratchString[out] = directory[in];
			in++;
			out++;
			if (out >= STRING_LENGTH)
			{
				platform->Message(HOST_MESSAGE, "CombineName() buffer overflow.");
				out = 0;
			}
		}
	}

	//scratchString[out] = '/';
	// out++;

	in = 0;
	while (fileName[in] != 0 && fileName[in] != '\n') // && fileName[in] != '/')
	{
		scratchString[out] = fileName[in];
		in++;
		out++;
		if (out >= STRING_LENGTH)
		{
			platform->Message(HOST_MESSAGE, "CombineName() buffer overflow.");
			out = 0;
		}
	}
	scratchString[out] = 0;

	return scratchString;
}

// List the flat files in a directory.  No sub-directories or recursion.

const char* MassStorage::FileList(const char* directory, bool fromLine)
{
	char fileListBracket = FILE_LIST_BRACKET;
	char fileListSeparator = FILE_LIST_SEPARATOR;

	if (fromLine)
	{
		if (platform->Emulating() == marlin)
		{
			fileListBracket = 0;
			fileListSeparator = '\n';
		}
	}

	TCHAR loc[64];

	// Remove the trailing '/' from the directory name
	size_t len = strnlen(directory, ARRAY_SIZE(loc));
	if (len == 0)
	{
		loc[0] = 0;
	}
	else
	{
		strncpy(loc, directory, len - 1);
		loc[len - 1] = 0;
	}

//  if(reprap.Debug()) {
//	  platform->Message(HOST_MESSAGE, "Opening: ");
//	  platform->Message(HOST_MESSAGE, loc);
//	  platform->Message(HOST_MESSAGE, "\n");
//  }

	DIR dir;
	FRESULT res = f_opendir(&dir, loc);
	if (res == FR_OK)
	{

//	  if(reprap.Debug()) {
//		  platform->Message(HOST_MESSAGE, "Directory open\n");
//	  }

		size_t p = 0;
		unsigned int foundFiles = 0;

		f_readdir(&dir, 0);

		FILINFO entry;
		TCHAR loclfname[255];		// this buffer is used to hold the directory name, and later to hold the long filename
		entry.lfname = loclfname;
		entry.lfsize = ARRAY_SIZE(loclfname);

		// When we reach, the end of the directory, the function we are about to call suppresses the "end of directory" error code and goes on returning FR_OK.
		// So we need to check the sector number before the call. What idiot wrote that function???
		while (dir.sect != 0 && f_readdir(&dir, &entry) == FR_OK)
		{
			const TCHAR *fp = (loclfname[0] == 0) ? entry.fname : loclfname;
			if (*fp != 0)
			{
				size_t lastFileStart = p;
				if (fileListBracket)
				{
					fileList[p++] = fileListBracket;
				}
				while (*fp != 0 && p <= FILE_LIST_LENGTH - 4)	// leave space for this character, bracket, separator, bracket
				{
					fileList[p++] = *fp++;
				}
				if (*fp != 0)
				{
					// Not enough space to store this filename
					p = lastFileStart;
					break;
				}
				foundFiles++;
				if (fileListBracket)
				{
					fileList[p++] = fileListBracket;
				}
				fileList[p++] = fileListSeparator;
			}
		}

		if (foundFiles == 0)
			return "NONE";

		fileList[--p] = 0; // Get rid of the last separator
		return fileList;
	}

	return "";
}

// Delete a file
bool MassStorage::Delete(const char* directory, const char* fileName)
{
	const char* location = platform->GetMassStorage()->CombineName(directory, fileName);
	if (f_unlink(location) != FR_OK)
	{
		platform->Message(HOST_MESSAGE, "Can't delete file ");
		platform->Message(HOST_MESSAGE, location);
		platform->Message(HOST_MESSAGE, "\n");
		return false;
	}
	return true;
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
	const char* location = platform->GetMassStorage()->CombineName(directory, fileName);

	writing = write;
	lastBufferEntry = FILE_BUF_LEN - 1;
	FRESULT openReturn;

	if (writing)
	{
		openReturn = f_open(&file, location, FA_CREATE_ALWAYS | FA_WRITE);
		if (openReturn != FR_OK)
		{
			platform->Message(HOST_MESSAGE, "Can't open ");
			platform->Message(HOST_MESSAGE, location);
			platform->Message(HOST_MESSAGE, " to write to.  Error code: ");
			snprintf(scratchString, STRING_LENGTH, "%d", openReturn);
			platform->Message(HOST_MESSAGE, scratchString);
			platform->Message(HOST_MESSAGE, "\n");
			return false;
		}
		bufferPointer = 0;
	}
	else
	{
		openReturn = f_open(&file, location, FA_OPEN_EXISTING | FA_READ);
		if (openReturn != FR_OK)
		{
			platform->Message(HOST_MESSAGE, "Can't open ");
			platform->Message(HOST_MESSAGE, location);
			platform->Message(HOST_MESSAGE, " to read from.  Error code: ");
			snprintf(scratchString, STRING_LENGTH, "%d", openReturn);
			platform->Message(HOST_MESSAGE, scratchString);
			platform->Message(HOST_MESSAGE, "\n");
			return false;
		}
		bufferPointer = FILE_BUF_LEN;
	}

	inUse = true;
	openCount = 1;
	return true;
}

void FileStore::Duplicate()
{
	if (!inUse)
	{
		platform->Message(HOST_MESSAGE, "Attempt to dup a non-open file.\n");
		return;
	}
	++openCount;
}

void FileStore::Close()
{
	if (!inUse)
	{
		platform->Message(HOST_MESSAGE, "Attempt to close a non-open file.\n");
		return;
	}
	--openCount;
	if (openCount == 0)
	{
		if (writing)
		{
			WriteBuffer();
		}
		f_close(&file);
		inUse = false;
		writing = false;
		lastBufferEntry = 0;
	}
}

void FileStore::Seek(unsigned long pos)
{
	if (!inUse)
	{
		platform->Message(HOST_MESSAGE, "Attempt to seek on a non-open file.\n");
		return;
	}
	if (writing)
	{
		WriteBuffer();
	}
	f_lseek(&file, pos);
	bufferPointer = (writing) ? 0 : FILE_BUF_LEN;
}

void FileStore::GoToEnd()
{
	Seek(Length());
}

unsigned long FileStore::Length()
{
	if (!inUse)
	{
		platform->Message(HOST_MESSAGE, "Attempt to size non-open file.\n");
		return 0;
	}
	return file.fsize;
}

int8_t FileStore::Status()
{
	if (!inUse)
		return nothing;

	if (lastBufferEntry == FILE_BUF_LEN)
		return byteAvailable;

	if (bufferPointer < lastBufferEntry)
		return byteAvailable;

	return nothing;
}

void FileStore::ReadBuffer()
{
	FRESULT readStatus = f_read(&file, buf, FILE_BUF_LEN, &lastBufferEntry);	// Read a chunk of file
	if (readStatus)
	{
		platform->Message(HOST_MESSAGE, "Error reading file.\n");
	}
	bufferPointer = 0;
}

// Single character read via the buffer
bool FileStore::Read(char& b)
{
	if (!inUse)
	{
		platform->Message(HOST_MESSAGE, "Attempt to read from a non-open file.\n");
		return false;
	}

	if (bufferPointer >= FILE_BUF_LEN)
	{
		ReadBuffer();
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
		platform->Message(HOST_MESSAGE, "Attempt to read from a non-open file.\n");
		return -1;
	}
	bufferPointer = FILE_BUF_LEN;	// invalidate the buffer
	UINT bytesRead;
	FRESULT readStatus = f_read(&file, extBuf, nBytes, &bytesRead);
	if (readStatus)
	{
		platform->Message(HOST_MESSAGE, "Error reading file.\n");
		return -1;
	}
	return (int)bytesRead;
}

void FileStore::WriteBuffer()
{
	FRESULT writeStatus = f_write(&file, buf, bufferPointer, &lastBufferEntry);
	if ((writeStatus != FR_OK) || (lastBufferEntry != bufferPointer))
	{
		platform->Message(HOST_MESSAGE, "Error writing file.  Disc may be full.\n");
	}
	bufferPointer = 0;
}

void FileStore::Write(char b)
{
	if (!inUse)
	{
		platform->Message(HOST_MESSAGE, "Attempt to write byte to a non-open file.\n");
		return;
	}
	buf[bufferPointer] = b;
	bufferPointer++;
	if (bufferPointer >= FILE_BUF_LEN)
		WriteBuffer();
}

void FileStore::Write(const char* b)
{
	if (!inUse)
	{
		platform->Message(HOST_MESSAGE, "Attempt to write string to a non-open file.\n");
		return;
	}
	int i = 0;
	while (b[i])
		Write(b[i++]);
}

//***************************************************************************************************

// Serial/USB class

Line::Line()
{
}

void Line::Init()
{
	inputGetIndex = 0;
	inputNumChars = 0;
	outputGetIndex = 0;
	outputNumChars = 0;
	SerialUSB.begin(BAUD_RATE);
	inUsbWrite = false;
}

void Line::Spin()
{
	// Read the serial data in blocks to avoid excessive flow control
	if (inputNumChars <= lineInBufsize / 2)
	{
		int16_t target = SerialUSB.available() + (int16_t) inputNumChars;
		if (target > lineInBufsize)
		{
			target = lineInBufsize;
		}
		while ((int16_t) inputNumChars < target)
		{
			int incomingByte = SerialUSB.read();
			if (incomingByte < 0)
				break;
			inBuffer[(inputGetIndex + inputNumChars) % lineInBufsize] = (char) incomingByte;
			++inputNumChars;
		}
	}

	TryFlushOutput();
}

void Line::Write(char b)
{
	TryFlushOutput();
	if (outputNumChars == 0 && SerialUSB.canWrite() != 0)
	{
		++inUsbWrite;
		SerialUSB.write(b);
		--inUsbWrite;
	}
	else if (outputNumChars < lineOutBufSize)
	{
		outBuffer[(outputGetIndex + outputNumChars) % lineOutBufSize] = b;
		++outputNumChars;
	}
	// else discard the character
}

void Line::Write(const char* b)
{
	while (*b)
	{
		Write(*b++);
	}
}

void Line::TryFlushOutput()
{
	while (outputNumChars != 0 && SerialUSB.canWrite() != 0)
	{
		++inUsbWrite;
		SerialUSB.write(outBuffer[outputGetIndex]);
		--inUsbWrite;
		outputGetIndex = (outputGetIndex + 1) % lineOutBufSize;
		--outputNumChars;
	}
}

// End
