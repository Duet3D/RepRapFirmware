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

#include "RepRapFirmware.h"
#include "Hardware/IoPorts.h"
#include "DueFlashStorage.h"
#include "Fans/FansManager.h"
#include "Heating/TemperatureError.h"
#include "OutputMemory.h"
#include "Storage/FileStore.h"
#include "Storage/FileData.h"
#include "Storage/MassStorage.h"	// must be after Pins.h because it needs NumSdCards defined
#include "MessageType.h"
#include "Tools/Spindle.h"
#include "Endstops/EndstopsManager.h"
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

#include <functional>

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
constexpr unsigned int ZProbeAverageReadings = 8;		// We average this number of readings with IR on, and the same number with IR off

// HEATERS - The bed is assumed to be the at index 0

// Define the number of temperature readings we average for each thermistor. This should be a power of 2 and at least 4 ^ AD_OVERSAMPLE_BITS.
#ifdef SAME70
// On the SAME70 we read a thermistor on every tick so that we can average a higher number of readings
// Keep THERMISTOR_AVERAGE_READINGS * NUM_HEATERS * 1ms no greater than HEAT_SAMPLE_TIME or the PIDs won't work well.
constexpr unsigned int ThermistorAverageReadings = 16;
#else
// We read a thermistor on alternate ticks
// Keep THERMISTOR_AVERAGE_READINGS * NUM_HEATERS * 2ms no greater than HEAT_SAMPLE_TIME or the PIDs won't work well.
constexpr unsigned int ThermistorAverageReadings = 16;
#endif

constexpr uint32_t maxPidSpinDelay = 5000;			// Maximum elapsed time in milliseconds between successive temp samples by Pid::Spin() permitted for a temp sensor

/****************************************************************************************************/

enum class BoardType : uint8_t
{
	Auto = 0,
#if defined(DUET3_V03)
	Duet3_03 = 1
#elif defined(DUET3_V05)
	Duet3_05 = 1
#elif defined(DUET3_V06)
	Duet3_06 = 1
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

	// Get the latest reading
	uint16_t GetLatestReading() const volatile
	{
		size_t indexOfLastReading = index;			// capture volatile variable
		indexOfLastReading =  (indexOfLastReading == 0) ? numAveraged - 1 : indexOfLastReading - 1;
		return readings[indexOfLastReading];
	}

	static constexpr size_t NumAveraged() { return numAveraged; }

private:
	uint16_t readings[numAveraged];
	size_t index;
	uint32_t sum;
	bool isValid;
	//invariant(sum == + over readings)
	//invariant(index < numAveraged)
};

typedef AveragingFilter<ThermistorAverageReadings> ThermistorAveragingFilter;
typedef AveragingFilter<ZProbeAverageReadings> ZProbeAveragingFilter;

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
	AxisDriversConfig() { numDrivers = 0; }
	DriversBitmap GetDriversBitmap() const;

	uint8_t numDrivers;								// Number of drivers assigned to each axis
	DriverId driverNumbers[MaxDriversPerAxis];		// The driver numbers assigned - only the first numDrivers are meaningful
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

	void Diagnostics(MessageType mtype);
	GCodeResult DiagnosticTest(GCodeBuffer& gb, const StringRef& reply, int d);
	void LogError(ErrorCode e) { errorCodeBits |= (uint32_t)e; }

	[[noreturn]] void SoftwareReset(uint16_t reason, const uint32_t *stk = nullptr);
	bool AtxPower() const;
	void AtxPowerOn();
	void AtxPowerOff(bool defer);
	void SetBoardType(BoardType bt);
	const char* GetElectronicsString() const;
	const char* GetBoardString() const;

#ifdef DUET_NG
	bool IsDuetWiFi() const;
	bool IsDueXPresent() const { return expansionBoard != ExpansionBoardType::none; }
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
	const char* GetConfigFile() const; 				// Where the configuration is stored (in the system dir).

#if HAS_MASS_STORAGE
	MassStorage* GetMassStorage() const;
	FileStore* OpenFile(const char* folder, const char* fileName, OpenMode mode, uint32_t preAllocSize = 0) const;
	bool Delete(const char* folder, const char *filename) const;
	bool FileExists(const char* folder, const char *filename) const;
	bool DirectoryExists(const char *folder, const char *dir) const;

	const char* GetWebDir() const; 					// Where the html etc files are
	const char* GetGCodeDir() const; 				// Where the gcodes are
	const char* GetMacroDir() const;				// Where the user-defined macros are
	const char* GetDefaultFile() const;				// Where the default configuration is stored (in the system dir).

	// Functions to work with the system files folder
	GCodeResult SetSysDir(const char* dir, const StringRef& reply);				// Set the system files path
	bool SysFileExists(const char *filename) const;
	FileStore* OpenSysFile(const char *filename, OpenMode mode) const;
	bool DeleteSysFile(const char *filename) const;
	bool MakeSysFileName(const StringRef& result, const char *filename) const;
	void GetSysDir(const StringRef & path) const;
#endif

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
	int8_t GetEnableValue(size_t driver) const;
	void EnableLocalDrivers(size_t axisOrExtruder);
	void EnableOneLocalDriver(size_t driver, float requiredCurrent);
	void DisableAllDrivers();
	void DisableDrivers(size_t axisOrExtruder);
	void DisableOneLocalDriver(size_t driver);
	void EmergencyDisableDrivers();
	void SetDriversIdle();
	bool SetMotorCurrent(size_t axisOrExtruder, float current, int code, const StringRef& reply);
	float GetMotorCurrent(size_t axisOrExtruder, int code) const;
	void SetIdleCurrentFactor(float f);
	float GetIdleCurrentFactor() const
		{ return idleCurrentFactor; }
	bool SetDriverMicrostepping(size_t driver, unsigned int microsteps, int mode);
	bool SetMicrostepping(size_t axisOrExtruder, int microsteps, bool mode, const StringRef& reply);
	unsigned int GetMicrostepping(size_t axisOrExtruder, bool& interpolation) const;
	void SetDriverStepTiming(size_t driver, const float microseconds[4]);
	bool GetDriverStepTiming(size_t driver, float microseconds[4]) const;
	float DriveStepsPerUnit(size_t axisOrExtruder) const;
	const float *GetDriveStepsPerUnit() const
		{ return driveStepsPerUnit; }
	void SetDriveStepsPerUnit(size_t axisOrExtruder, float value, uint32_t requestedMicrostepping);
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
	float AxisMaximum(size_t axis) const;
	void SetAxisMaximum(size_t axis, float value, bool byProbing);
	float AxisMinimum(size_t axis) const;
	void SetAxisMinimum(size_t axis, float value, bool byProbing);
	float AxisTotalLength(size_t axis) const;
	float GetPressureAdvance(size_t extruder) const;
	GCodeResult SetPressureAdvance(float advance, GCodeBuffer& gb, const StringRef& reply);

	const AxisDriversConfig& GetAxisDriversConfig(size_t axis) const
		pre(axis < MaxAxes)
		{ return axisDrivers[axis]; }
	void SetAxisDriversConfig(size_t axis, size_t numValues, const DriverId driverNumbers[])
		pre(axis < MaxAxes);
	DriverId GetExtruderDriver(size_t extruder) const
		pre(extruder < MaxExtruders)
		{ return extruderDrivers[extruder]; }
	void SetExtruderDriver(size_t extruder, DriverId driver)
		pre(extruder < MaxExtruders);
	uint32_t GetDriversBitmap(size_t axisOrExtruder) const	// get the bitmap of driver step bits for this axis or extruder
		pre(axisOrExtruder < MaxAxesPlusExtruders + NumLocalDrivers)
		{ return driveDriverBits[axisOrExtruder]; }
	uint32_t GetSlowDriversBitmap() const { return slowDriversBitmap; }
	uint32_t GetSlowDriverStepHighClocks() const { return slowDriverStepTimingClocks[0]; }
	uint32_t GetSlowDriverStepLowClocks() const { return slowDriverStepTimingClocks[1]; }
	uint32_t GetSlowDriverDirSetupClocks() const { return slowDriverStepTimingClocks[2]; }
	uint32_t GetSlowDriverDirHoldClocks() const { return slowDriverStepTimingClocks[3]; }
	uint32_t GetSteppingEnabledDrivers() const { return steppingEnabledDriversBitmap; }
	void DisableSteppingDriver(uint8_t driver) { steppingEnabledDriversBitmap &= ~StepPins::CalcDriverBitmap(driver); }
	void EnableAllSteppingDrivers() { steppingEnabledDriversBitmap = 0xFFFFFFFF; }

#if SUPPORT_NONLINEAR_EXTRUSION
	bool GetExtrusionCoefficients(size_t extruder, float& a, float& b, float& limit) const;
	void SetNonlinearExtrusion(size_t extruder, float a, float b, float limit);
#endif

	// Endstops and Z probe
	EndstopsManager& GetEndstops() { return endstops; }
	ZProbe& GetCurrentZProbe() { return endstops.GetCurrentZProbe(); }
	ZProbeType GetCurrentZProbeType() const;
	void InitZProbeFilters();
	const volatile ZProbeAveragingFilter& GetZProbeOnFilter() const { return zProbeOnFilter; }
	const volatile ZProbeAveragingFilter& GetZProbeOffFilter() const { return zProbeOffFilter; }

#if HAS_MASS_STORAGE
	bool WritePlatformParameters(FileStore *f, bool includingG31) const;
#endif

	// Heat and temperature
	volatile ThermistorAveragingFilter& GetAdcFilter(size_t channel)
	pre(channel < ARRAY_SIZE(adcFilters))
	{
		return adcFilters[channel];
	}

	int GetAveragingFilterIndex(const IoPort&) const;

	void UpdateConfiguredHeaters();

	// Flash operations
	void UpdateFirmware();
	void StartIap();
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
	void DisableAutoSave();
	void EnableAutoSave(float saveVoltage, float resumeVoltage);
	bool GetAutoSaveSettings(float& saveVoltage, float&resumeVoltage);
#endif

#if HAS_12V_MONITOR
	// 12V rail voltage
	void GetV12Voltages(float& minV, float& currV, float& maxV) const;
#endif

#if HAS_SMART_DRIVERS
	float GetTmcDriversTemperature(unsigned int board) const;
	void DriverCoolingFansOnOff(uint32_t driverChannelsMonitored, bool on);
	unsigned int GetNumSmartDrivers() const { return numSmartDrivers; }
#endif

#if HAS_VOLTAGE_MONITOR || HAS_12V_MONITOR
	bool HasVinPower() const;
#else
	bool HasVinPower() const { return true; }
#endif

#if HAS_STALL_DETECT
	GCodeResult ConfigureStallDetection(GCodeBuffer& gb, const StringRef& reply, OutputBuffer *& buf);
#endif

#if HAS_MASS_STORAGE
	// Logging support
	GCodeResult ConfigureLogging(GCodeBuffer& gb, const StringRef& reply);
#endif

	// Ancillary PWM
	GCodeResult GetSetAncillaryPwm(GCodeBuffer& gb, const StringRef& reply);
	void ExtrudeOn();
	void ExtrudeOff();

	// CNC and laser support
	Spindle& AccessSpindle(size_t slot) { return spindles[slot]; }

	void SetLaserPwm(Pwm_t pwm);
	float GetLaserPwm() const;							// return laser PWM in 0..1
	bool AssignLaserPin(GCodeBuffer& gb, const StringRef& reply);
	void SetLaserPwmFrequency(PwmFrequency freq);

	// Misc
	GCodeResult ConfigurePort(GCodeBuffer& gb, const StringRef& reply);
	const PwmPort& GetGpioPort(size_t gpioPortNumber) const pre(gpioPortNumber > MaxGpioPorts) { return gpioPorts[gpioPortNumber]; }

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

	GCodeResult ConfigureGpioOrServo(uint32_t gpioNumber, bool isServo, GCodeBuffer& gb, const StringRef& reply);

#if SUPPORT_CAN_EXPANSION
	void IterateDrivers(size_t axisOrExtruder, std::function<void(uint8_t)> localFunc, std::function<void(DriverId)> remoteFunc);
	void IterateLocalDrivers(size_t axisOrExtruder, std::function<void(uint8_t)> func) { IterateDrivers(axisOrExtruder, func, [](DriverId){}); }
#else
	void IterateDrivers(size_t axisOrExtruder, std::function<void(uint8_t)> localFunc);
	void IterateLocalDrivers(size_t axisOrExtruder, std::function<void(uint8_t)> func) { IterateDrivers(axisOrExtruder, func); }
#endif

#if HAS_SMART_DRIVERS
	void ReportDrivers(MessageType mt, DriversBitmap& whichDrivers, const char* text, bool& reported);
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
		uint32_t taskName;							// first 4 bytes of the task name
		uint32_t stack[23];							// stack when the exception occurred, with the program counter at the bottom

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

#if HAS_MASS_STORAGE
	// Logging
	Logger *logger;
#endif

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
	uint32_t errorCodeBits;

	void InitialiseInterrupts();

	// Drives
	void UpdateMotorCurrent(size_t driver, float current);
	void SetDriverDirection(uint8_t driver, bool direction)
	pre(driver < DRIVES);

	bool directions[NumDirectDrivers];
	int8_t enableValues[NumDirectDrivers];

	float motorCurrents[MaxAxesPlusExtruders];				// the normal motor current for each stepper driver
	float motorCurrentFraction[MaxAxesPlusExtruders];		// the percentages of normal motor current that each driver is set to
	float standstillCurrentFraction[MaxAxesPlusExtruders];	// the percentages of normal motor current that each driver uses when in standstill
	uint16_t microstepping[MaxAxesPlusExtruders];			// the microstepping used for each axis or extruder, top bit is set if interpolation enabled

	volatile DriverStatus driverState[MaxAxesPlusExtruders];
	float maxFeedrates[MaxAxesPlusExtruders];
	float accelerations[MaxAxesPlusExtruders];
	float driveStepsPerUnit[MaxAxesPlusExtruders];
	float instantDvs[MaxAxesPlusExtruders];
	uint32_t driveDriverBits[MaxAxesPlusExtruders + NumDirectDrivers];
															// the bitmap of local driver port bits for each axis or extruder, followed by the bitmaps for the individual Z motors
	AxisDriversConfig axisDrivers[MaxAxes];					// the driver numbers assigned to each axis

	float pressureAdvance[MaxExtruders];
#if SUPPORT_NONLINEAR_EXTRUSION
	float nonlinearExtrusionA[MaxExtruders], nonlinearExtrusionB[MaxExtruders], nonlinearExtrusionLimit[MaxExtruders];
#endif

	DriverId extruderDrivers[MaxExtruders];					// the driver number assigned to each extruder
	uint32_t slowDriverStepTimingClocks[4];					// minimum step high, step low, dir setup and dir hold timing for slow drivers
	uint32_t slowDriversBitmap;								// bitmap of driver port bits that need extended step pulse timing
	uint32_t steppingEnabledDriversBitmap;					// mask of driver bits that we haven't disabled temporarily
	float idleCurrentFactor;
	float minimumMovementSpeed;

#if HAS_SMART_DRIVERS
	size_t numSmartDrivers;									// the number of TMC drivers we have, the remaining are simple enable/step/dir drivers
	DriversBitmap temperatureShutdownDrivers, temperatureWarningDrivers, shortToGroundDrivers;
	DriversBitmap openLoadADrivers, openLoadBDrivers, notOpenLoadADrivers, notOpenLoadBDrivers;
	MillisTimer openLoadATimer, openLoadBTimer;
	MillisTimer driversFanTimers[NumTmcDriversSenseChannels];		// driver cooling fan timers
	uint8_t nextDriveToPoll;
#endif

	bool driversPowered;

#if HAS_SMART_DRIVERS && (HAS_VOLTAGE_MONITOR || HAS_12V_MONITOR)
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

	// Endstops
	EndstopsManager endstops;

	// Z probe
	volatile ZProbeAveragingFilter zProbeOnFilter;					// Z probe readings we took with the IR turned on
	volatile ZProbeAveragingFilter zProbeOffFilter;					// Z probe readings we took with the IR turned off

	// GPIO pins
	PwmPort gpioPorts[MaxGpioPorts];

	// Thermistors and temperature monitoring
	volatile ThermistorAveragingFilter adcFilters[NumAdcFilters];	// ADC reading averaging filters

#if HAS_CPU_TEMP_SENSOR
	uint32_t highestMcuTemperature, lowestMcuTemperature;
	float mcuTemperatureAdjust;
#endif

	// Axes and endstops
	float axisMaxima[MaxAxes];
	float axisMinima[MaxAxes];
	AxesBitmap axisMinimaProbed, axisMaximaProbed;

#if HAS_MASS_STORAGE
	static bool WriteAxisLimits(FileStore *f, AxesBitmap axesProbed, const float limits[MaxAxes], int sParam);
#endif

	// Heaters
	HeatersBitmap configuredHeaters;										// bitmask of all real heaters in use

	// Fans
	uint32_t lastFanCheckTime;

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
#if HAS_MASS_STORAGE
	MassStorage* massStorage;
	const char *sysDir;
#endif

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
	uint16_t lastVinUnderVoltageValue, lastVinOverVoltageValue;
	uint16_t autoPauseReading, autoResumeReading;
	uint32_t numVinUnderVoltageEvents, previousVinUnderVoltageEvents;
	volatile uint32_t numVinOverVoltageEvents, previousVinOverVoltageEvents;
	bool autoSaveEnabled;

	enum class AutoSaveState : uint8_t
	{
		starting = 0,
		normal,
		autoPaused
	};
	AutoSaveState autoSaveState;
#endif

#if HAS_12V_MONITOR
	AnalogChannelNumber v12MonitorAdcChannel;
	volatile uint16_t currentV12, highestV12, lowestV12;
	uint16_t lastV12UnderVoltageValue;
	uint32_t numV12UnderVoltageEvents, previousV12UnderVoltageEvents;
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
	float lastLaserPwm;

	// Power on/off
	bool deferredPowerDown;

	// Misc
	bool deliberateError;								// true if we deliberately caused an exception for testing purposes
};

inline const char* Platform::GetConfigFile() const
{
	return CONFIG_FILE;
}

#if HAS_MASS_STORAGE

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

inline const char* Platform::GetDefaultFile() const
{
	return CONFIG_BACKUP_FILE;
}

#endif

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

inline int8_t Platform::GetEnableValue(size_t driver) const
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

#if HAS_MASS_STORAGE

inline MassStorage* Platform::GetMassStorage() const
{
	return massStorage;
}

#endif

inline OutputBuffer *Platform::GetAuxGCodeReply()
{
	OutputBuffer *temp = auxGCodeReply;
	auxGCodeReply = nullptr;
	return temp;
}

#endif
