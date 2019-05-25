/****************************************************************************************************

RepRapFirmware - Platform

Platform contains all the code and definitions to deal with machine-dependent things such as control
pins, bed area, number of extruders, tolerable accelerations and speeds and so on.

No definitions that are system-independent should go in here.  Put them in Configuration.h.

-----------------------------------------------------------------------------------------------------

Version 0.3

28 August 2013

Adrian Bowyer
RepRap Professional Ltd
http://reprappro.com

Licence: GPL

****************************************************************************************************/

#ifndef PLATFORM_H
#define PLATFORM_H

// Language-specific includes

// Platform-specific includes

#include "RepRapFirmware.h"
#include "Hardware/IoPorts.h"
#include "DueFlashStorage.h"
#include "Fans/Fan.h"
#include "Fans/Tacho.h"
#include "Heating/TemperatureError.h"
#include "OutputMemory.h"
#include "Storage/FileStore.h"
#include "Storage/FileData.h"
#include "Storage/MassStorage.h"	// must be after Pins.h because it needs NumSdCards defined
#include "MessageType.h"
#include "Spindle.h"
#include "ZProbe.h"
#include "ZProbeProgrammer.h"
#include <General/IPAddress.h>

#if defined(DUET_NG)
# include "DueXn.h"
#elif defined(DUET_06_085)
# include "MCP4461/MCP4461.h"
#elif defined(__ALLIGATOR__)
# include "DAC/DAC084S085.h"       // SPI DAC for motor current vref
# include "EUI48/EUI48EEPROM.h"    // SPI EUI48 mac address EEPROM
# include "Microstepping.h"
#endif

constexpr bool FORWARDS = true;
constexpr bool BACKWARDS = !FORWARDS;

// Define the number of ADC filters and the indices of the extra ones
#if HAS_VREF_MONITOR
constexpr size_t VrefFilterIndex = NumThermistorInputs;
constexpr size_t VssaFilterIndex = NumThermistorInputs + 1;
# if HAS_CPU_TEMP_SENSOR
constexpr size_t CpuTempFilterIndex = NumThermistorInputs + 2;
constexpr size_t NumAdcFilters = NumThermistorInputs + 3;
# else
constexpr size_t NumAdcFilters = NumThermistorInputs + 2;
# endif
#elif HAS_CPU_TEMP_SENSOR
constexpr size_t CpuTempFilterIndex = NumThermistorInputs;
constexpr size_t NumAdcFilters = NumThermistorInputs + 1;
#else
constexpr size_t NumAdcFilters = NumThermistorInputs;
#endif

/**************************************************************************************************/

#if SUPPORT_INKJET

// Inkjet (if any - no inkjet is flagged by INKJET_BITS negative)

const int8_t INKJET_BITS = 12;							// How many nozzles? Set to -1 to disable this feature
const int INKJET_FIRE_MICROSECONDS = 5;					// How long to fire a nozzle
const int INKJET_DELAY_MICROSECONDS = 800;				// How long to wait before the next bit

#endif

// Z PROBE

constexpr float Z_PROBE_STOP_HEIGHT = 0.7;						// Millimetres
constexpr unsigned int Z_PROBE_AVERAGE_READINGS = 8;			// We average this number of readings with IR on, and the same number with IR off

// HEATERS - The bed is assumed to be the at index 0

// Define the number of temperature readings we average for each thermistor. This should be a power of 2 and at least 4 ^ AD_OVERSAMPLE_BITS.
// Keep THERMISTOR_AVERAGE_READINGS * NUM_HEATERS * 2ms no greater than HEAT_SAMPLE_TIME or the PIDs won't work well.
constexpr unsigned int ThermistorAverageReadings = 32;

constexpr uint32_t maxPidSpinDelay = 5000;			// Maximum elapsed time in milliseconds between successive temp samples by Pid::Spin() permitted for a temp sensor

/****************************************************************************************************/

enum class BoardType : uint8_t
{
	Auto = 0,
#if defined(DUET3_V03)
	Duet3_03 = 1
#elif defined(DUET3_V05)
	Duet3_05 = 1
#elif defined(SAME70XPLD)
	SAME70XPLD_0 = 1
#elif defined(DUET_NG)
	DuetWiFi_10 = 1,
	DuetWiFi_102 = 2,
	DuetEthernet_10 = 3,
	DuetEthernet_102 = 4
#elif defined(DUET_M)
	DuetM_10 = 1,
#elif defined(DUET_06_085)
	Duet_06 = 1,
	Duet_07 = 2,
	Duet_085 = 3
#elif defined(__RADDS__)
	RADDS_15 = 1
#elif defined(__ALLIGATOR__)
	Alligator_2 = 1
#elif defined(PCCB_10)
	PCCB_v10 = 1
#elif defined(PCCB_08) || defined(PCCB_08_X5)
	PCCB_v08 = 1
#else
# error Unknown board
#endif
};

enum class EndStopHit
{
  noStop = 0,		// no endstop hit
  lowHit = 1,		// low switch hit, or Z-probe in use and above threshold
  highHit = 2,		// high stop hit
  nearStop = 3		// approaching Z-probe threshold
};

// The values of the following enumeration must tally with the X,Y,... parameters for the M574 command
enum class EndStopPosition
{
	noEndStop = 0,
	lowEndStop = 1,
	highEndStop = 2
};

// Type of an endstop input - values must tally with the M574 command S parameter
enum class EndStopInputType
{
	activeLow = 0,
	activeHigh = 1,
	zProbe = 2,
	motorStall = 3
};

// Other firmware that we might switch to be compatible with.
enum class Compatibility : uint8_t
{
	me = 0,
	reprapFirmware = 1,
	marlin = 2,
	teacup = 3,
	sprinter = 4,
	repetier = 5,
	nanoDLP = 6
};

/***************************************************************************************************/

// Enumeration describing the reasons for a software reset.
// The spin state gets or'ed into this, so keep the lower 4 bits unused.
enum class SoftwareResetReason : uint16_t
{
	user = 0,						// M999 command
	erase = 0x10,					// special M999 command to erase firmware and reset
	NMI = 0x20,
	hardFault = 0x30,				// most exceptions get escalated to a hard fault
	stuckInSpin = 0x40,				// we got stuck in a Spin() function in the Main task for too long
	wdtFault = 0x50,				// secondary watchdog
	usageFault = 0x60,
	otherFault = 0x70,
	stackOverflow = 0x80,			// FreeRTOS detected stack overflow
	assertCalled = 0x90,			// FreeRTOS assertion failure
	heaterWatchdog = 0xA0,			// the Heat task didn't kick the watchdog often enough

	// Bits that are or'ed in
	inAuxOutput = 0x0800,			// this bit is or'ed in if we were in aux output at the time
	inLwipSpin = 0x2000,			// we got stuck in a call to LWIP for too long
	inUsbOutput = 0x4000,			// this bit is or'ed in if we were in USB output at the time
	deliberate = 0x8000				// this but it or'ed in if we deliberately caused a fault
};

// Enumeration to describe various tests we do in response to the M122 command
enum class DiagnosticTestType : int
{
	PrintTestReport = 1,			// run some tests and report the processor ID

	PrintMoves = 100,				// print summary of recent moves (only if recording moves was enabled in firmware)
#ifdef DUET_NG
	PrintExpanderStatus = 101,		// print DueXn expander status
#endif
	TimeSquareRoot = 102,			// do a timing test on the square root function
	TimeSinCos = 103,				// do a timing test on the trig functions
	TimeSDWrite = 104,				// do a write timing test on the SD card
	PrintObjectSizes = 105,			// print the sizes of various objects

	TestWatchdog = 1001,			// test that we get a watchdog reset if the tick interrupt stops
	TestSpinLockup = 1002,			// test that we get a software reset if a Spin() function takes too long
	TestSerialBlock = 1003,			// test what happens when we write a blocking message via debugPrintf()
	DivideByZero = 1004,			// do an integer divide by zero to test exception handling
	UnalignedMemoryAccess = 1005,	// do an unaligned memory access to test exception handling
	BusFault = 1006					// generate a bus fault
};

/***************************************************************************************************************/

// Class to perform averaging of values read from the ADC
// numAveraged should be a power of 2 for best efficiency
template<size_t numAveraged> class AveragingFilter
{
public:
	AveragingFilter()
	{
		Init(0);
	}

	void Init(uint16_t val) volatile
	{
		const irqflags_t flags = cpu_irq_save();
		sum = (uint32_t)val * (uint32_t)numAveraged;
		index = 0;
		isValid = false;
		for (size_t i = 0; i < numAveraged; ++i)
		{
			readings[i] = val;
		}
		cpu_irq_restore(flags);
	}

	// Call this to put a new reading into the filter
	// This is only called by the ISR, so it not declared volatile to make it faster
	void ProcessReading(uint16_t r)
	{
		sum = sum - readings[index] + r;
		readings[index] = r;
		++index;
		if (index == numAveraged)
		{
			index = 0;
			isValid = true;
		}
	}

	// Return the raw sum
	uint32_t GetSum() const volatile
	{
		return sum;
	}

	// Return true if we have a valid average
	bool IsValid() const volatile
	{
		return isValid;
	}

private:
	uint16_t readings[numAveraged];
	size_t index;
	uint32_t sum;
	bool isValid;
	//invariant(sum == + over readings)
	//invariant(index < numAveraged)
};

typedef AveragingFilter<ThermistorAverageReadings> ThermistorAveragingFilter;
typedef AveragingFilter<Z_PROBE_AVERAGE_READINGS> ZProbeAveragingFilter;

// Enumeration of error condition bits
enum class ErrorCode : uint32_t
{
	BadTemp = 1u << 0,
	BadMove = 1u << 1,
	OutputStarvation = 1u << 2,
	OutputStackOverflow = 1u << 3,
	HsmciTimeout = 1u << 4
};

struct AxisDriversConfig
{
	uint8_t numDrivers;								// Number of drivers assigned to each axis
	uint8_t driverNumbers[MaxDriversPerAxis];		// The driver numbers assigned - only the first numDrivers are meaningful
};

// The main class that defines the RepRap machine for the benefit of the other classes
class Platform
{
public:
	// Enumeration to describe the status of a drive
	enum class DriverStatus : uint8_t { disabled, idle, enabled };
  
	Platform();
  
//-------------------------------------------------------------------------------------------------------------

	// These are the functions that form the interface between Platform and the rest of the firmware.

	void Init();									// Set the machine up after a restart.  If called subsequently this should set the machine up as if
													// it has just been restarted; it can do this by executing an actual restart if you like, but beware the loop of death...
	void Spin();									// This gets called in the main loop and should do any housekeeping needed
	void Exit();									// Shut down tidily. Calling Init after calling this should reset to the beginning

	Compatibility Emulating() const;
	void SetEmulating(Compatibility c);
	bool EmulatingMarlin() const;

	void Diagnostics(MessageType mtype);
	GCodeResult DiagnosticTest(GCodeBuffer& gb, const StringRef& reply, int d);
	void LogError(ErrorCode e) { errorCodeBits |= (uint32_t)e; }

	void SoftwareReset(uint16_t reason, const uint32_t *stk = nullptr) __attribute((noreturn));
	bool AtxPower() const;
	void AtxPowerOn();
	void AtxPowerOff(bool defer);
	void SetBoardType(BoardType bt);
	const char* GetElectronicsString() const;
	const char* GetBoardString() const;

#ifdef DUET_NG
	bool IsDuetWiFi() const;
#endif

	const uint8_t *GetDefaultMacAddress() const { return defaultMacAddress; }

	// Timing
	void Tick() __attribute__((hot));						// Process a systick interrupt

	// Real-time clock
	bool IsDateTimeSet() const { return realTime != 0; }	// Has the RTC been set yet?
	time_t GetDateTime() const { return realTime; }			// Retrieves the current RTC datetime and returns true if it's valid
	bool SetDateTime(time_t time);							// Sets the current RTC date and time or returns false on error

  	// Communications and data storage
	OutputBuffer *GetAuxGCodeReply();						// Returns cached G-Code reply for AUX devices and clears its reference
	void AppendAuxReply(OutputBuffer *buf, bool rawMessage);
	void AppendAuxReply(const char *msg, bool rawMessage);
    uint32_t GetAuxSeq() { return auxSeq; }
    bool HaveAux() const { return auxDetected; }			// Any device on the AUX line?
    void SetAuxDetected() { auxDetected = true; }

	void SetIPAddress(IPAddress ip);
	IPAddress GetIPAddress() const;
	void SetNetMask(IPAddress nm);
	IPAddress NetMask() const;
	void SetGateWay(IPAddress gw);
	IPAddress GateWay() const;
	void SetBaudRate(size_t chan, uint32_t br);
	uint32_t GetBaudRate(size_t chan) const;
	void SetCommsProperties(size_t chan, uint32_t cp);
	uint32_t GetCommsProperties(size_t chan) const;

#if defined(__ALLIGATOR__)
	// Mac address from EUI48 EEPROM
	EUI48EEPROM eui48MacAddress;
#endif

	// File functions
	MassStorage* GetMassStorage() const;
	FileStore* OpenFile(const char* folder, const char* fileName, OpenMode mode, uint32_t preAllocSize = 0) const;
	bool Delete(const char* folder, const char *filename) const;
	bool FileExists(const char* folder, const char *filename) const;
	bool DirectoryExists(const char *folder, const char *dir) const;

	const char* GetWebDir() const; 					// Where the html etc files are
	const char* GetGCodeDir() const; 				// Where the gcodes are
	const char* GetMacroDir() const;				// Where the user-defined macros are
	const char* GetConfigFile() const; 				// Where the configuration is stored (in the system dir).
	const char* GetDefaultFile() const;				// Where the default configuration is stored (in the system dir).

	// Function to work with the system files folder
	void SetSysDir(const char* dir);				// Set the system files path
	bool SysFileExists(const char *filename) const;
	FileStore* OpenSysFile(const char *filename, OpenMode mode) const;
	bool DeleteSysFile(const char *filename) const;
	void MakeSysFileName(const StringRef& result, const char *filename) const;
	void GetSysDir(const StringRef & path) const;

	// Message output (see MessageType for further details)
	void Message(MessageType type, const char *message);
	void Message(MessageType type, OutputBuffer *buffer);
	void MessageF(MessageType type, const char *fmt, ...) __attribute__ ((format (printf, 3, 4)));
	void MessageF(MessageType type, const char *fmt, va_list vargs);
	bool FlushAuxMessages();
	bool FlushMessages();							// Flush messages to USB and aux, returning true if there is more to send
	void SendAlert(MessageType mt, const char *message, const char *title, int sParam, float tParam, AxesBitmap controls);
	void StopLogging();

	// Movement
	void EmergencyStop();
	void SetDirection(size_t axisOrExtruder, bool direction);
	void SetDirectionValue(size_t driver, bool dVal);
	bool GetDirectionValue(size_t driver) const;
	void SetEnableValue(size_t driver, int8_t eVal);
	bool GetEnableValue(size_t driver) const;
	void EnableDriver(size_t driver);
	void DisableDriver(size_t driver);
	void EnableDrive(size_t axisOrExtruder);
	void DisableDrive(size_t axisOrExtruder);
	void DisableAllDrives();
	void SetDriversIdle();
	void SetMotorCurrent(size_t axisOrExtruder, float current, int code);
	float GetMotorCurrent(size_t axisOrExtruder, int code) const;
	void SetIdleCurrentFactor(float f);
	float GetIdleCurrentFactor() const
		{ return idleCurrentFactor; }
	bool SetDriverMicrostepping(size_t driver, unsigned int microsteps, int mode);
	unsigned int GetDriverMicrostepping(size_t drive, bool& interpolate) const;
	bool SetMicrostepping(size_t axisOrExtruder, int microsteps, bool mode);
	unsigned int GetMicrostepping(size_t axisOrExtruder, bool& interpolation) const;
	void SetDriverStepTiming(size_t driver, const float microseconds[4]);
	bool GetDriverStepTiming(size_t driver, float microseconds[4]) const;
	float DriveStepsPerUnit(size_t axisOrExtruder) const;
	const float *GetDriveStepsPerUnit() const
		{ return driveStepsPerUnit; }
	void SetDriveStepsPerUnit(size_t axisOrExtruder, float value, uint32_t microstepping);
	float Acceleration(size_t axisOrExtruder) const;
	const float* Accelerations() const;
	void SetAcceleration(size_t axisOrExtruder, float value);
	float MaxFeedrate(size_t axisOrExtruder) const;
	const float* MaxFeedrates() const { return maxFeedrates; }
	void SetMaxFeedrate(size_t axisOrExtruder, float value);
	float MinMovementSpeed() const { return minimumMovementSpeed; }
	void SetMinMovementSpeed(float value) { minimumMovementSpeed = max<float>(value, 0.01); }
	float GetInstantDv(size_t axis) const;
	void SetInstantDv(size_t axis, float value);
	EndStopHit Stopped(size_t axisOrExtruder) const;
	bool EndStopInputState(size_t axis) const;
	float AxisMaximum(size_t axis) const;
	void SetAxisMaximum(size_t axis, float value, bool byProbing);
	float AxisMinimum(size_t axis) const;
	void SetAxisMinimum(size_t axis, float value, bool byProbing);
	float AxisTotalLength(size_t axis) const;
	float GetPressureAdvance(size_t extruder) const;
	void SetPressureAdvance(size_t extruder, float factor);

	void SetEndStopConfiguration(size_t axis, EndStopPosition endstopPos, EndStopInputType inputType)
		pre(axis < MaxAxes);

	void GetEndStopConfiguration(size_t axis, EndStopPosition& endstopPos, EndStopInputType& inputType) const
		pre(axis < MaxAxes);

	uint32_t GetAllEndstopStates() const;

	const AxisDriversConfig& GetAxisDriversConfig(size_t axis) const
		pre(axis < MaxAxes)
		{ return axisDrivers[axis]; }
	void SetAxisDriversConfig(size_t axis, size_t numValues, const uint32_t driverNumbers[])
		pre(axis < MaxAxes);
	uint8_t GetExtruderDriver(size_t extruder) const
		pre(extruder < MaxExtruders)
		{ return extruderDrivers[extruder]; }
	void SetExtruderDriver(size_t extruder, uint8_t driver)
		pre(extruder < MaxExtruders);
	uint32_t GetDriversBitmap(size_t axisOrExtruder) const	// get the bitmap of driver step bits for this axis or extruder
		pre(axisOrExtruder < 2 * MaxTotalDrivers)
		{ return driveDriverBits[axisOrExtruder]; }
	static void StepDriversLow();							// set all step pins low
	static void StepDriversHigh(uint32_t driverMap);		// set the specified step pins high
	uint32_t GetSlowDriversBitmap() const { return slowDriversBitmap; }
	uint32_t GetSlowDriverStepHighClocks() const { return slowDriverStepTimingClocks[0]; }
	uint32_t GetSlowDriverStepLowClocks() const { return slowDriverStepTimingClocks[1]; }
	uint32_t GetSlowDriverDirSetupClocks() const { return slowDriverStepTimingClocks[2]; }
	uint32_t GetSlowDriverDirHoldClocks() const { return slowDriverStepTimingClocks[3]; }

#if SUPPORT_NONLINEAR_EXTRUSION
	bool GetExtrusionCoefficients(size_t extruder, float& a, float& b, float& limit) const;
	void SetNonlinearExtrusion(size_t extruder, float a, float b, float limit);
#endif

	// Z probe
	void SetZProbeDefaults();
	float GetZProbeStopHeight() const;
	float GetZProbeDiveHeight() const;
	float GetZProbeStartingHeight();
	float GetZProbeTravelSpeed() const;
	int GetZProbeReading() const;
	EndStopHit GetZProbeResult() const;
	int GetZProbeSecondaryValues(int& v1, int& v2);
	void SetZProbeType(unsigned int iZ);
	ZProbeType GetZProbeType() const { return zProbeType; }
	const ZProbe& GetZProbeParameters(ZProbeType probeType) const;
	const ZProbe& GetCurrentZProbeParameters() const { return GetZProbeParameters(zProbeType); }
	void SetZProbeParameters(ZProbeType probeType, const struct ZProbe& params);
	bool HomingZWithProbe() const;
	bool WritePlatformParameters(FileStore *f, bool includingG31) const;
	void SetProbing(bool isProbing);
	GCodeResult ProgramZProbe(GCodeBuffer& gb, const StringRef& reply);
	void SetZProbeModState(bool b) const;

	// Heat and temperature
	float GetZProbeTemperature() const;						// Get our best estimate of the Z probe temperature

	volatile ThermistorAveragingFilter& GetAdcFilter(size_t channel)
	pre(channel < ARRAY_SIZE(adcFilters))
	{
		return adcFilters[channel];
	}

	void SetHeater(size_t heater, float power, PwmFrequency freq = 0)	// power is a fraction in [0,1]
	pre(heater < Heaters);

	void UpdateConfiguredHeaters();

	// Fans
	bool ConfigureFan(unsigned int mcode, int fanNumber, GCodeBuffer& gb, const StringRef& reply, bool& error);

	float GetFanValue(size_t fan) const;					// Result is returned in range 0..1
	void SetFanValue(size_t fan, float speed);				// Accepts values between 0..1
#if defined(DUET_06_085)
	void EnableSharedFan(bool enable);						// enable/disable the fan that shares its PWM pin with the last heater
#endif
	bool IsFanControllable(size_t fan) const;
	const char *GetFanName(size_t fan) const;

	bool WriteFanSettings(FileStore *f) const;		// Save some resume information
	uint32_t GetFanRPM(size_t tachoIndex) const;

	// Flash operations
	void UpdateFirmware();
	bool CheckFirmwareUpdatePrerequisites(const StringRef& reply);

	// AUX device
	void Beep(int freq, int ms);
	void SendAuxMessage(const char* msg);

	// Hotend configuration
	float GetFilamentWidth() const;
	void SetFilamentWidth(float width);
	float GetNozzleDiameter() const;
	void SetNozzleDiameter(float diameter);

	// Fire the inkjet (if any) in the given pattern
	// If there is no inkjet false is returned; if there is one this returns true
	// So you can test for inkjet presence with if(platform->Inkjet(0))
	bool Inkjet(int bitPattern);

	// MCU temperature
#if HAS_CPU_TEMP_SENSOR
	void GetMcuTemperatures(float& minT, float& currT, float& maxT) const;
	void SetMcuTemperatureAdjust(float v) { mcuTemperatureAdjust = v; }
	float GetMcuTemperatureAdjust() const { return mcuTemperatureAdjust; }
#endif

#if HAS_VOLTAGE_MONITOR
	// Power in voltage
	void GetPowerVoltages(float& minV, float& currV, float& maxV) const;
	float GetCurrentPowerVoltage() const;
	bool IsPowerOk() const;
	bool HasVinPower() const;
	void DisableAutoSave();
	void EnableAutoSave(float saveVoltage, float resumeVoltage);
	bool GetAutoSaveSettings(float& saveVoltage, float&resumeVoltage);
#endif

#if HAS_SMART_DRIVERS
	float GetTmcDriversTemperature(unsigned int board) const;
	void DriverCoolingFansOnOff(uint32_t driverChannelsMonitored, bool on);
	unsigned int GetNumSmartDrivers() const { return numSmartDrivers; }
#endif

#if HAS_STALL_DETECT
	GCodeResult ConfigureStallDetection(GCodeBuffer& gb, const StringRef& reply, OutputBuffer *& buf);
#endif

	// User I/O and servo support
	bool GetFirmwarePin(LogicalPin logicalPin, PinAccess access, Pin& firmwarePin, bool& invert);

	// For fan pin mapping
	bool TranslateFanPin(LogicalPin logicalPin, Pin& firmwarePin, bool& invert) const;
	void AppendPinName(LogicalPin lp, const StringRef& str) const;

	// For filament sensor support
	Pin GetEndstopPin(int endstop) const;			// Get the firmware pin number for an endstop

	// Logging support
	GCodeResult ConfigureLogging(GCodeBuffer& gb, const StringRef& reply);

	// Ancillary PWM
	void SetExtrusionAncilliaryPwmValue(float v);
	float GetExtrusionAncilliaryPwmValue() const;
	void SetExtrusionAncilliaryPwmFrequency(float f);
	float GetExtrusionAncilliaryPwmFrequency() const;
	bool SetExtrusionAncilliaryPwmPin(LogicalPin logicalPin, bool invert);
	LogicalPin GetExtrusionAncilliaryPwmPin(bool& invert) const { return extrusionAncilliaryPwmPort.GetLogicalPin(invert); }
	void ExtrudeOn();
	void ExtrudeOff();

	// CNC and laser support
	Spindle& AccessSpindle(size_t slot) { return spindles[slot]; }

	void SetLaserPwm(Pwm_t pwm);
	bool SetLaserPin(LogicalPin lp, bool invert);
	LogicalPin GetLaserPin(bool& invert) const { return laserPort.GetLogicalPin(invert); }
	void SetLaserPwmFrequency(float freq);
	float GetLaserPwmFrequency() const { return laserPort.GetFrequency(); }

	// Misc

#if SAM4E || SAM4S || SAME70
	uint32_t Random();
	void PrintUniqueId(MessageType mtype);
#endif

	static uint8_t softwareResetDebugInfo;				// extra info for debugging

	//-------------------------------------------------------------------------------------------------------
  
private:
	Platform(const Platform&);						// private copy constructor to make sure we don't try to copy a Platform

	const char* InternalGetSysDir() const;  		// where the system files are - not thread-safe!

	void RawMessage(MessageType type, const char *message);	// called by Message after handling error/warning flags

	void ResetChannel(size_t chan);					// re-initialise a serial channel
	float AdcReadingToCpuTemperature(uint32_t reading) const;

#if HAS_SMART_DRIVERS
	void ReportDrivers(MessageType mt, DriversBitmap& whichDrivers, const char* text, bool& reported);
#endif
#if HAS_STALL_DETECT
	bool AnyAxisMotorStalled(size_t drive) const pre(drive < DRIVES);
	bool ExtruderMotorStalled(size_t extruder) const pre(extruder < MaxExtruders);
#endif

	// These are the structures used to hold our non-volatile data.
	// The SAM3X and SAM4E don't have EEPROM so we save the data to flash. This unfortunately means that it gets cleared
	// every time we reprogram the firmware via bossa, but it can be retained when firmware updates are performed
	// via the web interface. That's why it's a good idea to implement versioning here - increase these values
	// whenever the fields of the following structs have changed.
	//
	// The SAM4E has a large page erase size (8K). For this reason we store the software reset data in the 512-byte user signature area
	// instead, which doesn't get cleared when the Erase button is pressed. The SoftareResetData struct must have at least one 32-bit
	// field to guarantee that values of this type will be 32-bit aligned. It must have no virtual members because it is read/written
	// directly from/to flash memory.
	struct SoftwareResetData
	{
		static const uint16_t versionValue = 8;		// increment this whenever this struct changes
		static const uint16_t magicValue = 0x7D00 | versionValue;	// value we use to recognise that all the flash data has been written
#if SAM3XA
		static const uint32_t nvAddress = 0;		// must be 4-byte aligned
#endif
		static const size_t numberOfSlots = 4;		// number of storage slots used to implement wear levelling - must fit in 512 bytes

		uint16_t magic;								// the magic number, including the version
		uint16_t resetReason;						// this records why we did a software reset, for diagnostic purposes
		uint32_t neverUsedRam;						// the amount of never used RAM at the last abnormal software reset
		uint32_t hfsr;								// hard fault status register
		uint32_t cfsr;								// configurable fault status register
		uint32_t icsr;								// interrupt control and state register
		uint32_t bfar;								// bus fault address register
		uint32_t sp;								// stack pointer
		uint32_t when;								// value of the RTC when the software reset occurred
#ifdef RTOS
		uint32_t taskName;							// first 4 bytes of the task name
		uint32_t stack[23];							// stack when the exception occurred, with the program counter at the bottom
#else
		uint32_t stack[24];							// stack when the exception occurred, with the program counter at the bottom
#endif

		bool isVacant() const						// return true if this struct can be written without erasing it first
		{
			const uint32_t *p = reinterpret_cast<const uint32_t*>(this);
			for (size_t i = 0; i < sizeof(*this)/sizeof(uint32_t); ++i)
			{
				if (*p != 0xFFFFFFFF)
				{
					return false;
				}
				++p;
			}
			return true;
		}
	};

#if SAM4E || SAM4S || SAME70
	static_assert(SoftwareResetData::numberOfSlots * sizeof(SoftwareResetData) <= 512, "Can't fit software reset data in user signature area");
#else
	static_assert(SoftwareResetData::numberOfSlots * sizeof(SoftwareResetData) <= FLASH_DATA_LENGTH, "NVData too large");
#endif

	// Logging
	Logger *logger;

	// Z probes
	ZProbe switchZProbeParameters;			// Z probe values for the switch Z-probe
	ZProbe irZProbeParameters;				// Z probe values for the IR sensor
	ZProbe alternateZProbeParameters;		// Z probe values for the alternate sensor
	ZProbeType zProbeType;					// the type of Z probe we are currently using

	// Network
	IPAddress ipAddress;
	IPAddress netMask;
	IPAddress gateWay;
	uint8_t defaultMacAddress[6];

	// Board and processor
#if SAM4E || SAM4S || SAME70
	uint32_t uniqueId[5];
#endif
	BoardType board;

#ifdef DUET_NG
	ExpansionBoardType expansionBoard;
#endif

	bool active;
	Compatibility compatibility;
	uint32_t errorCodeBits;

	void InitialiseInterrupts();

	// Drives
	void SetDriverCurrent(size_t driver, float current, int code);
	void UpdateMotorCurrent(size_t driver);
	void SetDriverDirection(uint8_t driver, bool direction)
	pre(driver < DRIVES);

	static uint32_t CalcDriverBitmap(size_t driver);	// calculate the step bit(s) for this driver

	volatile DriverStatus driverState[MaxTotalDrivers];
	bool directions[MaxTotalDrivers];
	int8_t enableValues[MaxTotalDrivers];
	Pin endStopPins[NumEndstops];
	float maxFeedrates[MaxTotalDrivers];
	float minimumMovementSpeed;
	float accelerations[MaxTotalDrivers];
	float driveStepsPerUnit[MaxTotalDrivers];
	float instantDvs[MaxTotalDrivers];
	float pressureAdvance[MaxExtruders];
#if SUPPORT_NONLINEAR_EXTRUSION
	float nonlinearExtrusionA[MaxExtruders], nonlinearExtrusionB[MaxExtruders], nonlinearExtrusionLimit[MaxExtruders];
#endif
	float motorCurrents[MaxTotalDrivers];				// the normal motor current for each stepper driver
	float motorCurrentFraction[MaxTotalDrivers];		// the percentages of normal motor current that each driver is set to
	AxisDriversConfig axisDrivers[MaxAxes];				// the driver numbers assigned to each axis
	uint8_t extruderDrivers[MaxExtruders];				// the driver number assigned to each extruder
	uint32_t driveDriverBits[2 * MaxTotalDrivers];		// the bitmap of driver port bits for each axis or extruder, followed by the raw versions
	uint32_t slowDriverStepTimingClocks[4];				// minimum step high, step low, dir setup and dir hold timing for slow drivers
	uint32_t slowDriversBitmap;							// bitmap of driver port bits that need extended step pulse timing
	float idleCurrentFactor;

#if HAS_SMART_DRIVERS
	size_t numSmartDrivers;								// the number of TMC2660 drivers we have, the remaining are simple enable/step/dir drivers
	DriversBitmap temperatureShutdownDrivers, temperatureWarningDrivers, shortToGroundDrivers;
	DriversBitmap openLoadADrivers, openLoadBDrivers, notOpenLoadADrivers, notOpenLoadBDrivers;
	MillisTimer openLoadATimer, openLoadBTimer;
	MillisTimer driversFanTimers[NumTmcDriversSenseChannels];		// driver cooling fan timers
	uint8_t nextDriveToPoll;
#endif

	bool driversPowered;

#if HAS_SMART_DRIVERS && HAS_VOLTAGE_MONITOR
	bool warnDriversNotPowered;
#endif

#if HAS_STALL_DETECT
	DriversBitmap logOnStallDrivers, pauseOnStallDrivers, rehomeOnStallDrivers;
	DriversBitmap stalledDrivers, stalledDriversToLog, stalledDriversToPause, stalledDriversToRehome;
#endif

#if defined(DUET_06_085)
	// Digipots
	MCP4461 mcpDuet;
	MCP4461 mcpExpansion;
	uint8_t potWipes[8];											// we have only 8 digipots, on the Duet 0.8.5 we use the DAC for the 9th
	float senseResistor;
	float maxStepperDigipotVoltage;
	float stepperDacVoltageRange, stepperDacVoltageOffset;
#elif defined(__ALLIGATOR__)
	Pin spiDacCS[MaxSpiDac];
	DAC084S085 dacAlligator;
	DAC084S085 dacPiggy;
#endif

	// Z probe
	Pin zProbePin;
	Pin zProbeModulationPin;
	ZProbeProgrammer zProbeProg;
	volatile ZProbeAveragingFilter zProbeOnFilter;					// Z probe readings we took with the IR turned on
	volatile ZProbeAveragingFilter zProbeOffFilter;					// Z probe readings we took with the IR turned off

	// Thermistors and temperature monitoring
	volatile ThermistorAveragingFilter adcFilters[NumAdcFilters];	// ADC reading averaging filters

#if HAS_CPU_TEMP_SENSOR
	uint32_t highestMcuTemperature, lowestMcuTemperature;
	float mcuTemperatureAdjust;
#endif

	void InitZProbe();
	uint16_t GetRawZProbeReading() const;

	// Axes and endstops
	float axisMaxima[MaxAxes];
	float axisMinima[MaxAxes];
	AxesBitmap axisMinimaProbed, axisMaximaProbed;
	EndStopPosition endStopPos[MaxAxes];
	EndStopInputType endStopInputType[MaxAxes];

	static bool WriteAxisLimits(FileStore *f, AxesBitmap axesProbed, const float limits[MaxAxes], int sParam);

	// Heaters
	uint32_t configuredHeaters;										// bitmask of all real heaters in use

	// Fans
	Fan fans[NUM_FANS];
	uint32_t lastFanCheckTime;
	void InitFans();
	bool FansHardwareInverted(size_t fanNumber) const;

	// Fan tachos
	Tacho tachos[NumTachos];

  	// Serial/USB
	uint32_t baudRates[NUM_SERIAL_CHANNELS];
	uint8_t commsParams[NUM_SERIAL_CHANNELS];

#ifdef SERIAL_AUX_DEVICE
	volatile OutputStack auxOutput;
	Mutex auxMutex;
#endif

	OutputBuffer *auxGCodeReply;				// G-Code reply for AUX devices (special one because it is actually encapsulated before sending)
	uint32_t auxSeq;							// Sequence number for AUX devices
	bool auxDetected;							// Have we processed at least one G-Code from an AUX device?

#ifdef SERIAL_AUX2_DEVICE
    volatile OutputStack aux2Output;
	Mutex aux2Mutex;
#endif

	volatile OutputStack usbOutput;
	Mutex usbMutex;

	// Files
	MassStorage* massStorage;
	const char *sysDir;
  
	// Data used by the tick interrupt handler

	// Heater #n, 0 <= n < HEATERS, uses "temperature channel" tc given by
	//
	//     tc = heaterTempChannels[n]
	//
	// Temperature channels follow a convention of
	//
	//     if (0 <= tc < HEATERS) then
	//        The temperature channel is a thermistor read using ADC.
	//        The actual ADC to read for tc is
	//
	//            thermistorAdcChannel[tc]
	//
	//        which, is equivalent to
	//
	//            PinToAdcChannel(tempSensePins[tc])
	//
	//     if (100 <= tc < 100 + (MaxSpiTempSensors - 1)) then
	//        The temperature channel is a thermocouple attached to a MAX31855 chip
	//        The MAX31855 object corresponding to the specific MAX31855 chip is
	//
	//            Max31855Devices[tc - 100]
	//
	//       Note that the MAX31855 objects, although statically declared, are not
	//       initialized until configured via a "M305 Pn X10m" command with 0 <= n < HEATERS
	//       and 0 <= m < MaxSpiTempSensors.
	//
	// NOTE BENE: When a M305 command is processed, the onus is on the gcode processor,
	// GCodes.cpp, to range check the value of the X parameter.  Code consuming the results
	// of the M305 command (e.g., SetThermistorNumber() and array lookups assume range
	// checking has already been performed.

	AnalogChannelNumber filteredAdcChannels[NumAdcFilters];
	AnalogChannelNumber zProbeAdcChannel;
	uint8_t tickState;
	size_t currentFilterNumber;
	int debugCode;

	// Hotend configuration
	float filamentWidth;
	float nozzleDiameter;

	// Power monitoring
#if HAS_VOLTAGE_MONITOR
	AnalogChannelNumber vInMonitorAdcChannel;
	volatile uint16_t currentVin, highestVin, lowestVin;
	uint16_t lastUnderVoltageValue, lastOverVoltageValue;
	uint16_t autoPauseReading, autoResumeReading;
	uint32_t numUnderVoltageEvents, previousUnderVoltageEvents;
	volatile uint32_t numOverVoltageEvents, previousOverVoltageEvents;
	bool autoSaveEnabled;

	enum class AutoSaveState : uint8_t
	{
		starting = 0,
		normal,
		autoPaused
	};
	AutoSaveState autoSaveState;
#endif

	uint32_t lastWarningMillis;							// When we last sent a warning message

	// RTC
	time_t realTime;									// the current date/time, or zero if never set
	uint32_t timeLastUpdatedMillis;						// the milliseconds counter when we last incremented the time

	// CNC and laser support
	Spindle spindles[MaxSpindles];
	float extrusionAncilliaryPwmValue;
	PwmPort extrusionAncilliaryPwmPort;
	PwmPort laserPort;

	// Power on/off
	bool deferredPowerDown;

	// Direct pin manipulation
	int8_t logicalPinModes[HighestLogicalPin + 1];		// what mode each logical pin is set to - would ideally be class PinMode not int8_t

	// Misc
	bool deliberateError;								// true if we deliberately caused an exception for testing purposes
};

// Where the htm etc files are
inline const char* Platform::GetWebDir() const
{
	return WEB_DIR;
}

// Where the gcodes are
inline const char* Platform::GetGCodeDir() const
{
	return GCODE_DIR;
}

inline const char* Platform::GetMacroDir() const
{
	return MACRO_DIR;
}

inline const char* Platform::GetConfigFile() const
{
	return CONFIG_FILE;
}

inline const char* Platform::GetDefaultFile() const
{
	return CONFIG_BACKUP_FILE;
}

//*****************************************************************************************************************

// Drive the RepRap machine - Movement

inline float Platform::DriveStepsPerUnit(size_t drive) const
{
	return driveStepsPerUnit[drive];
}

inline float Platform::Acceleration(size_t drive) const
{
	return accelerations[drive];
}

inline const float* Platform::Accelerations() const
{
	return accelerations;
}

inline void Platform::SetAcceleration(size_t drive, float value)
{
	accelerations[drive] = max<float>(value, 1.0);		// don't allow zero or negative
}

inline float Platform::MaxFeedrate(size_t drive) const
{
	return maxFeedrates[drive];
}

inline void Platform::SetMaxFeedrate(size_t drive, float value)
{
	maxFeedrates[drive] = max<float>(value, minimumMovementSpeed);	// don't allow zero or negative, but do allow small values
}

inline float Platform::GetInstantDv(size_t drive) const
{
	return instantDvs[drive];
}

inline void Platform::SetInstantDv(size_t drive, float value)
{
	instantDvs[drive] = max<float>(value, 0.1);			// don't allow zero or negative values, they causes Move to loop indefinitely
}

inline void Platform::SetDirectionValue(size_t drive, bool dVal)
{
	directions[drive] = dVal;
}

inline bool Platform::GetDirectionValue(size_t drive) const
{
	return directions[drive];
}

inline void Platform::SetDriverDirection(uint8_t driver, bool direction)
{
	if (driver < NumDirectDrivers)
	{
		const bool d = (direction == FORWARDS) ? directions[driver] : !directions[driver];
		digitalWrite(DIRECTION_PINS[driver], d);
	}
}

inline bool Platform::GetEnableValue(size_t driver) const
{
	return enableValues[driver];
}

inline float Platform::AxisMaximum(size_t axis) const
{
	return axisMaxima[axis];
}

inline float Platform::AxisMinimum(size_t axis) const
{
	return axisMinima[axis];
}

inline float Platform::AxisTotalLength(size_t axis) const
{
	return axisMaxima[axis] - axisMinima[axis];
}

inline void Platform::SetExtrusionAncilliaryPwmValue(float v)
{
	extrusionAncilliaryPwmValue = min<float>(v, 1.0);			// negative values are OK, they mean don't set the output
}

inline float Platform::GetExtrusionAncilliaryPwmValue() const
{
	return extrusionAncilliaryPwmValue;
}

inline void Platform::SetExtrusionAncilliaryPwmFrequency(float f)
{
	extrusionAncilliaryPwmPort.SetFrequency(f);
}

inline float Platform::GetExtrusionAncilliaryPwmFrequency() const
{
	return extrusionAncilliaryPwmPort.GetFrequency();
}

// For the Duet we use the fan output for this
// DC 2015-03-21: To allow users to control the cooling fan via gcodes generated by slic3r etc.,
// only turn the fan on/off if the extruder ancilliary PWM has been set nonzero.
// Caution: this is often called from an ISR, or with interrupts disabled!
inline void Platform::ExtrudeOn()
{
	if (extrusionAncilliaryPwmValue > 0.0)
	{
		extrusionAncilliaryPwmPort.WriteAnalog(extrusionAncilliaryPwmValue);
	}
}

// DC 2015-03-21: To allow users to control the cooling fan via gcodes generated by slic3r etc.,
// only turn the fan on/off if the extruder ancilliary PWM has been set nonzero.
// Caution: this is often called from an ISR, or with interrupts disabled!
inline void Platform::ExtrudeOff()
{
	if (extrusionAncilliaryPwmValue > 0.0)
	{
		extrusionAncilliaryPwmPort.WriteAnalog(0.0);
	}
}

//********************************************************************************************************

// Drive the RepRap machine - Heat and temperature

inline IPAddress Platform::GetIPAddress() const
{
	return ipAddress;
}

inline IPAddress Platform::NetMask() const
{
	return netMask;
}

inline IPAddress Platform::GateWay() const
{
	return gateWay;
}

inline float Platform::GetPressureAdvance(size_t extruder) const
{
	return (extruder < MaxExtruders) ? pressureAdvance[extruder] : 0.0;
}

// This is called by the tick ISR to get the raw Z probe reading to feed to the filter
inline uint16_t Platform::GetRawZProbeReading() const
{
	switch (zProbeType)
	{
	case ZProbeType::analog:
	case ZProbeType::dumbModulated:
	case ZProbeType::alternateAnalog:
		return min<uint16_t>(AnalogInReadChannel(zProbeAdcChannel), 4000);

	case ZProbeType::endstopSwitch:
		{
			const bool b = IoPort::ReadPin(GetEndstopPin(GetCurrentZProbeParameters().inputChannel));
			return (b) ? 4000 : 0;
		}

	case ZProbeType::digital:
	case ZProbeType::unfilteredDigital:
	case ZProbeType::blTouch:
		return (IoPort::ReadPin(zProbePin)) ? 4000 : 0;

	case ZProbeType::zMotorStall:
	default:
		return 4000;
	}
}

inline float Platform::GetFilamentWidth() const
{
	return filamentWidth;
}

inline void Platform::SetFilamentWidth(float width)
{
	filamentWidth = width;
}

inline float Platform::GetNozzleDiameter() const
{
	return nozzleDiameter;
}

inline void Platform::SetNozzleDiameter(float diameter)
{
	nozzleDiameter = diameter;
}

inline MassStorage* Platform::GetMassStorage() const
{
	return massStorage;
}

inline OutputBuffer *Platform::GetAuxGCodeReply()
{
	OutputBuffer *temp = auxGCodeReply;
	auxGCodeReply = nullptr;
	return temp;
}

// *** These next three functions must use the same bit assignments in the drivers bitmap ***
// Each stepper driver must be assigned one bit in a 32-bit word, in such a way that multiple drivers can be stepped efficiently
// and more or less simultaneously by doing parallel writes to several bits in one or more output ports.
// The bitmaps for various controller electronics are organised like this:
// Duet WiFi:
//	All step pins are on port D, so the bitmap is just the map of step bits in port D.
// Duet Maestro, PCCB and Duet 3:
//	All step pins are on port C, so the bitmap is just the map of step bits in port C.
// Duet 0.6 and 0.8.5:
//	Step pins are PA0, PC7,9,11,14,25,29 and PD0,3.
//	The PC and PD bit numbers don't overlap, so we use their actual positions.
//	PA0 clashes with PD0, so we use bit 1 to represent PA0.
// RADDS:
//  Step pins are PA2,9,12,15 PB16,19 PC3,12 PD6
//	PC12 clashes with PA12 so we shift PC3,12 left one bit
// Alligator:
//  Pins on ports B,C,D are used but the bit numbers are all different, so we use their actual positions

// Calculate the step bit for a driver. This doesn't need to be fast. It must return 0 if the driver is remote.
/*static*/ inline uint32_t Platform::CalcDriverBitmap(size_t driver)
{
	if (driver >= NumDirectDrivers)
	{
		return 0;
	}

#ifndef __LPC17xx__		//LPC doesn't need pinDesc
	const PinDescription& pinDesc = g_APinDescription[STEP_PINS[driver]];
#endif

#if defined(DUET_NG) || defined(DUET_M) || defined(PCCB) || defined(DUET3_V03) || defined(DUET3_V05) || defined(SAME70XPLD)
	return pinDesc.ulPin;
#elif defined(DUET_06_085)
	return (pinDesc.pPort == PIOA) ? pinDesc.ulPin << 1 : pinDesc.ulPin;
#elif defined(__RADDS__)
	return (pinDesc.pPort == PIOC) ? pinDesc.ulPin << 1 : pinDesc.ulPin;
#elif defined(__ALLIGATOR__)
	return pinDesc.ulPin;
# elif defined(__LPC17xx__)
	return 1u << STEP_PIN_PORT2_POS[driver];
#else
# error Unknown board
#endif
}

// Set the specified step pins high
// This needs to be as fast as possible, so we do a parallel write to the port(s).
// We rely on only those port bits that are step pins being set in the PIO_OWSR register of each port
/*static*/ inline void Platform::StepDriversHigh(uint32_t driverMap)
{
#if defined(DUET_NG)
	PIOD->PIO_ODSR = driverMap;				// on Duet WiFi all step pins are on port D
#elif defined(DUET_M) || defined(PCCB) || defined(DUET3_V03) || defined(DUET3_V05) || defined(SAME70XPLD)
	PIOC->PIO_ODSR = driverMap;				// on Duet Maestro all step pins are on port C
#elif defined(DUET_06_085)
	PIOD->PIO_ODSR = driverMap;
	PIOC->PIO_ODSR = driverMap;
	PIOA->PIO_ODSR = driverMap >> 1;		// do this last, it means the processor doesn't need to preserve the register containing driverMap
#elif defined(__RADDS__)
	PIOA->PIO_ODSR = driverMap;
	PIOB->PIO_ODSR = driverMap;
	PIOD->PIO_ODSR = driverMap;
	PIOC->PIO_ODSR = driverMap >> 1;		// do this last, it means the processor doesn't need to preserve the register containing driverMap
#elif defined(__ALLIGATOR__)
	PIOB->PIO_ODSR = driverMap;
	PIOD->PIO_ODSR = driverMap;
	PIOC->PIO_ODSR = driverMap;
#elif defined(__LPC17xx__)
	//On Azteeg X5 Mini all step pins are on Port 2
	//On Smoothieboard all step pins are on Port 2
	//On ReArm all step pins are on Port 2
	LPC_GPIO2->FIOSET = driverMap;
#else
# error Unknown board
#endif
}

// Set all step pins low
// This needs to be as fast as possible, so we do a parallel write to the port(s).
// We rely on only those port bits that are step pins being set in the PIO_OWSR register of each port
/*static*/ inline void Platform::StepDriversLow()
{
#if defined(DUET_NG)
	PIOD->PIO_ODSR = 0;						// on Duet WiFi all step pins are on port D
#elif defined(DUET_M) || defined(PCCB) || defined(DUET3_V03) || defined(DUET3_V05) || defined(SAME70XPLD)
	PIOC->PIO_ODSR = 0;						// on Duet Maestro, Duet 3 and PCCB all step pins are on port C
#elif defined(DUET_06_085)
	PIOD->PIO_ODSR = 0;
	PIOC->PIO_ODSR = 0;
	PIOA->PIO_ODSR = 0;
#elif defined(__RADDS__)
	PIOD->PIO_ODSR = 0;
	PIOC->PIO_ODSR = 0;
	PIOB->PIO_ODSR = 0;
	PIOA->PIO_ODSR = 0;
#elif defined(__ALLIGATOR__)
	PIOD->PIO_ODSR = 0;
	PIOC->PIO_ODSR = 0;
	PIOB->PIO_ODSR = 0;
#elif defined(__LPC17xx__)
	LPC_GPIO2->FIOCLR = STEP_DRIVER_MASK;
#else
# error Unknown board
#endif
}

//***************************************************************************************

#endif
