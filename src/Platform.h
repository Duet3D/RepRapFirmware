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
#include "ObjectModel/ObjectModel.h"
#include "Hardware/IoPorts.h"
#include "Fans/FansManager.h"
#include "Heating/TemperatureError.h"
#include "OutputMemory.h"
#include "Storage/FileStore.h"
#include "Storage/FileData.h"
#include "Storage/MassStorage.h"	// must be after Pins.h because it needs NumSdCards defined
#include "MessageType.h"
#include "Tools/Spindle.h"
#include "Endstops/EndstopsManager.h"
#include <GPIO/GpInPort.h>
#include <GPIO/GpOutPort.h>
#include <General/IPAddress.h>

#if defined(DUET_NG)
# include "DueXn.h"
#elif defined(DUET_06_085)
# include "MCP4461/MCP4461.h"
#elif defined(__ALLIGATOR__)
# include "DAC/DAC084S085.h"       // SPI DAC for motor current vref
# include "EUI48/EUI48EEPROM.h"    // SPI EUI48 mac address EEPROM
# include "Microstepping.h"
#elif defined(__LPC17xx__)
# include "MCP4461/MCP4461.h"
#endif

#if SUPPORT_CAN_EXPANSION
# include <RemoteInputHandle.h>
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
#if defined(DUET3)
	Duet3_v06_100 = 1,
	Duet3_v101 = 2,
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
#elif defined(PCCB_10)
	PCCB_v10 = 1
#elif defined(PCCB_08) || defined(PCCB_08_X5)
	PCCB_v08 = 1
#elif defined(__LPC17xx__)
	Lpc = 1
#else
# error Unknown board
#endif
};

/***************************************************************************************************/

// Enumeration to describe various tests we do in response to the M122 command
enum class DiagnosticTestType : unsigned int
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
	PrintObjectAddresses = 106,		// print the addresses and sizes of various objects

#ifdef __LPC17xx__
    PrintBoardConfiguration = 200,    //Prints out all pin/values loaded from SDCard to configure board
#endif

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
	AveragingFilter() noexcept
	{
		Init(0);
	}

	void Init(uint16_t val) volatile noexcept
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
	void ProcessReading(uint16_t r) noexcept
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
	uint32_t GetSum() const volatile noexcept
	{
		return sum;
	}

	// Return true if we have a valid average
	bool IsValid() const volatile noexcept
	{
		return isValid;
	}

	// Get the latest reading
	uint16_t GetLatestReading() const volatile noexcept
	{
		size_t indexOfLastReading = index;			// capture volatile variable
		indexOfLastReading =  (indexOfLastReading == 0) ? numAveraged - 1 : indexOfLastReading - 1;
		return readings[indexOfLastReading];
	}

	static constexpr size_t NumAveraged() noexcept { return numAveraged; }

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
	AxisDriversConfig() noexcept { numDrivers = 0; }
	DriversBitmap GetDriversBitmap() const noexcept;

	uint8_t numDrivers;								// Number of drivers assigned to each axis
	DriverId driverNumbers[MaxDriversPerAxis];		// The driver numbers assigned - only the first numDrivers are meaningful
};

// The main class that defines the RepRap machine for the benefit of the other classes
class Platform INHERIT_OBJECT_MODEL
{
public:
	// Enumeration to describe the status of a drive
	enum class DriverStatus : uint8_t { disabled, idle, enabled };

	Platform() noexcept;
	Platform(const Platform&) = delete;

//-------------------------------------------------------------------------------------------------------------

	// These are the functions that form the interface between Platform and the rest of the firmware.

	void Init() noexcept;									// Set the machine up after a restart.  If called subsequently this should set the machine up as if
															// it has just been restarted; it can do this by executing an actual restart if you like, but beware the loop of death...
	void Spin() noexcept;									// This gets called in the main loop and should do any housekeeping needed
	void Exit() noexcept;									// Shut down tidily. Calling Init after calling this should reset to the beginning

	void Diagnostics(MessageType mtype) noexcept;
	GCodeResult DiagnosticTest(GCodeBuffer& gb, const StringRef& reply, OutputBuffer*& buf, unsigned int d) THROWS(GCodeException);
	bool WasDeliberateError() const noexcept { return deliberateError; }
	void LogError(ErrorCode e) noexcept { errorCodeBits |= (uint32_t)e; }

	bool AtxPower() const noexcept;
	void AtxPowerOn() noexcept;
	void AtxPowerOff(bool defer) noexcept;

	BoardType GetBoardType() const noexcept { return board; }
	void SetBoardType(BoardType bt) noexcept;
	const char* GetElectronicsString() const noexcept;
	const char* GetBoardString() const noexcept;

#if SUPPORT_OBJECT_MODEL
	size_t GetNumGpInputsToReport() const noexcept;
	size_t GetNumGpOutputsToReport() const noexcept;
#endif

#ifdef DUET_NG
	bool IsDuetWiFi() const noexcept;
	bool IsDueXPresent() const noexcept { return expansionBoard != ExpansionBoardType::none; }
	const char *GetBoardName() const;
	const char *GetBoardShortName() const;
#endif

	const MacAddress& GetDefaultMacAddress() const noexcept { return defaultMacAddress; }

	// Timing
	void Tick() noexcept __attribute__((hot));						// Process a systick interrupt

	// Real-time clock
	bool IsDateTimeSet() const noexcept { return realTime != 0; }	// Has the RTC been set yet?
	time_t GetDateTime() const noexcept { return realTime; }		// Retrieves the current RTC datetime
	bool GetDateTime(tm& rslt) const noexcept { return gmtime_r(&realTime, &rslt) != nullptr && realTime != 0; }
																	// Retrieves the broken-down current RTC datetime and returns true if it's valid
	bool SetDateTime(time_t time) noexcept;							// Sets the current RTC date and time or returns false on error

  	// Communications and data storage
	void AppendUsbReply(OutputBuffer *buffer) noexcept;
	void AppendAuxReply(OutputBuffer *buf, bool rawMessage) noexcept;
	void AppendAuxReply(const char *msg, bool rawMessage) noexcept;

    bool IsAuxEnabled() const noexcept { return auxEnabled; }		// Any device on the AUX line?
    void EnableAux() noexcept;
    bool IsAuxRaw() const noexcept { return auxRaw; }
	void SetAuxRaw(bool raw) noexcept { auxRaw = raw; }
	void ResetChannel(size_t chan) noexcept;						// Re-initialise a serial channel

	void SetIPAddress(IPAddress ip) noexcept;
	IPAddress GetIPAddress() const noexcept;
	void SetNetMask(IPAddress nm) noexcept;
	IPAddress NetMask() const noexcept;
	void SetGateWay(IPAddress gw) noexcept;
	IPAddress GateWay() const noexcept;
	void SetBaudRate(size_t chan, uint32_t br) noexcept;
	uint32_t GetBaudRate(size_t chan) const noexcept;
	void SetCommsProperties(size_t chan, uint32_t cp) noexcept;
	uint32_t GetCommsProperties(size_t chan) const noexcept;

#if defined(__ALLIGATOR__)
	// Mac address from EUI48 EEPROM
	EUI48EEPROM eui48MacAddress;
#endif

	// File functions
#if HAS_MASS_STORAGE
	FileStore* OpenFile(const char* folder, const char* fileName, OpenMode mode, uint32_t preAllocSize = 0) const noexcept;
	bool Delete(const char* folder, const char *filename) const noexcept;
	bool FileExists(const char* folder, const char *filename) const noexcept;
	bool DirectoryExists(const char *folder, const char *dir) const noexcept;

	const char* GetWebDir() const noexcept; 					// Where the html etc files are
	const char* GetGCodeDir() const noexcept; 					// Where the gcodes are
	const char* GetMacroDir() const noexcept;					// Where the user-defined macros are

	// Functions to work with the system files folder
	GCodeResult SetSysDir(const char* dir, const StringRef& reply) noexcept;				// Set the system files path
	bool SysFileExists(const char *filename) const noexcept;
	FileStore* OpenSysFile(const char *filename, OpenMode mode) const noexcept;
	bool DeleteSysFile(const char *filename) const noexcept;
	bool MakeSysFileName(const StringRef& result, const char *filename) const noexcept;
	void AppendSysDir(const StringRef & path) const noexcept;
	void EncodeSysDir(OutputBuffer *buf) const noexcept;
#endif

	// Message output (see MessageType for further details)
	void Message(MessageType type, const char *message) noexcept;
	void Message(MessageType type, OutputBuffer *buffer) noexcept;
	void MessageF(MessageType type, const char *fmt, ...) noexcept __attribute__ ((format (printf, 3, 4)));
	void MessageF(MessageType type, const char *fmt, va_list vargs) noexcept;
	bool FlushAuxMessages() noexcept;
	bool FlushMessages() noexcept;							// Flush messages to USB and aux, returning true if there is more to send
	void SendAlert(MessageType mt, const char *message, const char *title, int sParam, float tParam, AxesBitmap controls) noexcept;
	void StopLogging() noexcept;

	// Movement
	void EmergencyStop() noexcept;
	void SetDirection(size_t axisOrExtruder, bool direction) noexcept;
	void SetDirectionValue(size_t driver, bool dVal) noexcept;
	bool GetDirectionValue(size_t driver) const noexcept;
	void SetDriverAbsoluteDirection(size_t driver, bool dVal) noexcept;
	void SetEnableValue(size_t driver, int8_t eVal) noexcept;
	int8_t GetEnableValue(size_t driver) const noexcept;
	void EnableLocalDrivers(size_t axisOrExtruder) noexcept;
	void EnableOneLocalDriver(size_t driver, float requiredCurrent) noexcept;
	void DisableAllDrivers() noexcept;
	void DisableDrivers(size_t axisOrExtruder) noexcept;
	void DisableOneLocalDriver(size_t driver) noexcept;
	void EmergencyDisableDrivers() noexcept;
	void SetDriversIdle() noexcept;
	bool SetMotorCurrent(size_t axisOrExtruder, float current, int code, const StringRef& reply) noexcept;
	float GetMotorCurrent(size_t axisOrExtruder, int code) const noexcept;
	void SetIdleCurrentFactor(float f) noexcept;
	float GetIdleCurrentFactor() const noexcept
		{ return idleCurrentFactor; }
	bool SetDriverMicrostepping(size_t driver, unsigned int microsteps, int mode) noexcept;
	bool SetMicrostepping(size_t axisOrExtruder, int microsteps, bool mode, const StringRef& reply) noexcept;
	unsigned int GetMicrostepping(size_t axisOrExtruder, bool& interpolation) const noexcept;
	void SetDriverStepTiming(size_t driver, const float microseconds[4]) noexcept;
	bool GetDriverStepTiming(size_t driver, float microseconds[4]) const noexcept;
	float DriveStepsPerUnit(size_t axisOrExtruder) const noexcept;
	const float *GetDriveStepsPerUnit() const noexcept
		{ return driveStepsPerUnit; }
	void SetDriveStepsPerUnit(size_t axisOrExtruder, float value, uint32_t requestedMicrostepping) noexcept;
	float Acceleration(size_t axisOrExtruder) const noexcept;
	const float* Accelerations() const noexcept;
	void SetAcceleration(size_t axisOrExtruder, float value) noexcept;
	float MaxFeedrate(size_t axisOrExtruder) const noexcept;
	const float* MaxFeedrates() const noexcept { return maxFeedrates; }
	void SetMaxFeedrate(size_t axisOrExtruder, float value) noexcept;
	float MinMovementSpeed() const noexcept { return minimumMovementSpeed; }
	void SetMinMovementSpeed(float value) noexcept { minimumMovementSpeed = max<float>(value, 0.01); }
	float GetInstantDv(size_t axis) const noexcept;
	void SetInstantDv(size_t axis, float value) noexcept;
	float AxisMaximum(size_t axis) const noexcept;
	void SetAxisMaximum(size_t axis, float value, bool byProbing) noexcept;
	float AxisMinimum(size_t axis) const noexcept;
	void SetAxisMinimum(size_t axis, float value, bool byProbing) noexcept;
	float AxisTotalLength(size_t axis) const noexcept;
	float GetPressureAdvance(size_t extruder) const noexcept;
	GCodeResult SetPressureAdvance(float advance, GCodeBuffer& gb, const StringRef& reply);

	const AxisDriversConfig& GetAxisDriversConfig(size_t axis) const noexcept
		pre(axis < MaxAxes)
		{ return axisDrivers[axis]; }
	void SetAxisDriversConfig(size_t axis, size_t numValues, const DriverId driverNumbers[]) noexcept
		pre(axis < MaxAxes);
	DriverId GetExtruderDriver(size_t extruder) const noexcept
		pre(extruder < MaxExtruders)
		{ return extruderDrivers[extruder]; }
	void SetExtruderDriver(size_t extruder, DriverId driver) noexcept
		pre(extruder < MaxExtruders);
	uint32_t GetDriversBitmap(size_t axisOrExtruder) const noexcept	// get the bitmap of driver step bits for this axis or extruder
		pre(axisOrExtruder < MaxAxesPlusExtruders + NumLocalDrivers)
		{ return driveDriverBits[axisOrExtruder]; }
	uint32_t GetSlowDriversBitmap() const noexcept { return slowDriversBitmap; }
	uint32_t GetSlowDriverStepHighClocks() const noexcept { return slowDriverStepTimingClocks[0]; }
	uint32_t GetSlowDriverStepLowClocks() const noexcept { return slowDriverStepTimingClocks[1]; }
	uint32_t GetSlowDriverDirSetupClocks() const noexcept { return slowDriverStepTimingClocks[2]; }
	uint32_t GetSlowDriverDirHoldClocks() const noexcept { return slowDriverStepTimingClocks[3]; }
	uint32_t GetSteppingEnabledDrivers() const noexcept { return steppingEnabledDriversBitmap; }
	void DisableSteppingDriver(uint8_t driver) noexcept { steppingEnabledDriversBitmap &= ~StepPins::CalcDriverBitmap(driver); }
	void EnableAllSteppingDrivers() noexcept { steppingEnabledDriversBitmap = 0xFFFFFFFF; }

#if SUPPORT_NONLINEAR_EXTRUSION
	bool GetExtrusionCoefficients(size_t extruder, float& a, float& b, float& limit) const noexcept;
	void SetNonlinearExtrusion(size_t extruder, float a, float b, float limit) noexcept;
#endif

	// Endstops and Z probe
	EndstopsManager& GetEndstops() noexcept { return endstops; }
	ReadLockedPointer<ZProbe> GetZProbeOrDefault(size_t probeNumber) noexcept { return endstops.GetZProbeOrDefault(probeNumber); }
	void InitZProbeFilters() noexcept;
	const volatile ZProbeAveragingFilter& GetZProbeOnFilter() const noexcept { return zProbeOnFilter; }
	const volatile ZProbeAveragingFilter& GetZProbeOffFilter() const  noexcept{ return zProbeOffFilter; }

#if HAS_MASS_STORAGE
	bool WritePlatformParameters(FileStore *f, bool includingG31) const noexcept;
#endif

	// Heat and temperature
	volatile ThermistorAveragingFilter& GetAdcFilter(size_t channel) noexcept
	pre(channel < ARRAY_SIZE(adcFilters))
	{
		return adcFilters[channel];
	}

	int GetAveragingFilterIndex(const IoPort&) const noexcept;

	void UpdateConfiguredHeaters() noexcept;

	// AUX device
	void Beep(int freq, int ms) noexcept;
	void SendAuxMessage(const char* msg) noexcept;

	// Hotend configuration
	float GetFilamentWidth() const noexcept;
	void SetFilamentWidth(float width) noexcept;
	float GetNozzleDiameter() const noexcept;
	void SetNozzleDiameter(float diameter) noexcept;

	// Fire the inkjet (if any) in the given pattern
	// If there is no inkjet false is returned; if there is one this returns true
	// So you can test for inkjet presence with if(platform->Inkjet(0))
	bool Inkjet(int bitPattern) noexcept;

	// MCU temperature
#if HAS_CPU_TEMP_SENSOR
	MinMaxCurrent GetMcuTemperatures() const noexcept;
	void SetMcuTemperatureAdjust(float v) noexcept { mcuTemperatureAdjust = v; }
	float GetMcuTemperatureAdjust() const noexcept { return mcuTemperatureAdjust; }
#endif

#if HAS_VOLTAGE_MONITOR
	// Power in voltage
	MinMaxCurrent GetPowerVoltages() const noexcept;
	float GetCurrentPowerVoltage() const noexcept;
	bool IsPowerOk() const noexcept;
	void DisableAutoSave() noexcept;
	void EnableAutoSave(float saveVoltage, float resumeVoltage) noexcept;
	bool GetAutoSaveSettings(float& saveVoltage, float&resumeVoltage) noexcept;
#endif

#if HAS_12V_MONITOR
	// 12V rail voltage
	MinMaxCurrent GetV12Voltages() const noexcept;
#endif

#if HAS_SMART_DRIVERS
	float GetTmcDriversTemperature(unsigned int board) const noexcept;
	void DriverCoolingFansOnOff(DriverChannelsBitmap driverChannelsMonitored, bool on) noexcept;
	unsigned int GetNumSmartDrivers() const noexcept { return numSmartDrivers; }
#endif

#if HAS_VOLTAGE_MONITOR || HAS_12V_MONITOR
	bool HasVinPower() const noexcept;
#else
	bool HasVinPower() const noexcept { return true; }
#endif

#if HAS_STALL_DETECT
	GCodeResult ConfigureStallDetection(GCodeBuffer& gb, const StringRef& reply, OutputBuffer *& buf) THROWS(GCodeException);
#endif

#if HAS_MASS_STORAGE
	// Logging support
	GCodeResult ConfigureLogging(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);
	const char *GetLogFileName() const noexcept;
#endif

	// Ancillary PWM
	GCodeResult GetSetAncillaryPwm(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);
	void ExtrudeOn() noexcept;
	void ExtrudeOff() noexcept;

	// CNC and laser support
	Spindle& AccessSpindle(size_t slot) noexcept { return spindles[slot]; }

#if SUPPORT_LASER
	void SetLaserPwm(Pwm_t pwm) noexcept;
	float GetLaserPwm() const noexcept;							// return laser PWM in 0..1
	bool AssignLaserPin(GCodeBuffer& gb, const StringRef& reply);
	void SetLaserPwmFrequency(PwmFrequency freq) noexcept;
#endif

	// Misc
	GCodeResult ConfigurePort(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);

	GpOutputPort& GetGpOutPort(size_t gpoutPortNumber) noexcept
		pre(gpioPortNumber < MaxGpOutPorts)	{ return gpoutPorts[gpoutPortNumber]; }
	const GpInputPort& GetGpInPort(size_t gpinPortNumber) const noexcept
		pre(gpinPortNumber < MaxGpInPorts) 	{ return gpinPorts[gpinPortNumber]; }

#if SUPPORTS_UNIQUE_ID
	uint32_t Random() noexcept;
	const char *GetUniqueIdString() const noexcept { return uniqueIdChars; }
#endif

#if SUPPORT_CAN_EXPANSION
	void HandleRemoteGpInChange(CanAddress src, uint8_t handleMajor, uint8_t handleMinor, bool state) noexcept;
#endif

protected:
	DECLARE_OBJECT_MODEL
	OBJECT_MODEL_ARRAY(axisDrivers)
	OBJECT_MODEL_ARRAY(workplaceOffsets)

private:
	const char* InternalGetSysDir() const noexcept;  					// where the system files are - not thread-safe!

	void RawMessage(MessageType type, const char *message) noexcept;	// called by Message after handling error/warning flags

	float AdcReadingToCpuTemperature(uint32_t reading) const noexcept;

#if SUPPORT_CAN_EXPANSION
	void IterateDrivers(size_t axisOrExtruder, std::function<void(uint8_t) /*noexcept*/ > localFunc, std::function<void(DriverId) /*noexcept*/ > remoteFunc) noexcept;
	void IterateLocalDrivers(size_t axisOrExtruder, std::function<void(uint8_t) /*noexcept*/ > func) noexcept { IterateDrivers(axisOrExtruder, func, [](DriverId){}); }
#else
	void IterateDrivers(size_t axisOrExtruder, std::function<void(uint8_t) /*noexcept*/ > localFunc) noexcept;
	void IterateLocalDrivers(size_t axisOrExtruder, std::function<void(uint8_t) /*noexcept*/ > func) noexcept { IterateDrivers(axisOrExtruder, func); }
#endif

#if HAS_SMART_DRIVERS
	void ReportDrivers(MessageType mt, DriversBitmap& whichDrivers, const char* text, bool& reported) noexcept;
#endif

#if HAS_MASS_STORAGE
	// Logging
	Logger *logger;
#endif

	// Network
	IPAddress ipAddress;
	IPAddress netMask;
	IPAddress gateWay;
	MacAddress defaultMacAddress;

	// Board and processor
#if SUPPORTS_UNIQUE_ID
	uint32_t uniqueId[5];
	char uniqueIdChars[30 + 5 + 1];			// 30 characters, 5 separators, 1 null terminator
#endif

	BoardType board;

#ifdef DUET_NG
	ExpansionBoardType expansionBoard;
#endif

	bool active;
	uint32_t errorCodeBits;

	void InitialiseInterrupts() noexcept;

	// Drives
	void UpdateMotorCurrent(size_t driver, float current) noexcept;
	void SetDriverDirection(uint8_t driver, bool direction) noexcept
	pre(driver < DRIVES);

	bool directions[NumDirectDrivers];
	int8_t enableValues[NumDirectDrivers];

	float motorCurrents[MaxAxesPlusExtruders];				// the normal motor current for each stepper driver
	float motorCurrentFraction[MaxAxesPlusExtruders];		// the percentages of normal motor current that each driver is set to
	float standstillCurrentPercent[MaxAxesPlusExtruders];	// the percentages of normal motor current that each driver uses when in standstill
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
#elif defined(__LPC17xx__)
	MCP4461 mcp4451;// works for 5561 (only volatile setting commands)
#endif

	// Endstops
	EndstopsManager endstops;

	// Z probe
	volatile ZProbeAveragingFilter zProbeOnFilter;					// Z probe readings we took with the IR turned on
	volatile ZProbeAveragingFilter zProbeOffFilter;					// Z probe readings we took with the IR turned off

	// GPIO pins
	GpOutputPort gpoutPorts[MaxGpOutPorts];
	GpInputPort gpinPorts[MaxGpInPorts];

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
	static bool WriteAxisLimits(FileStore *f, AxesBitmap axesProbed, const float limits[MaxAxes], int sParam) noexcept;
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

	uint32_t auxSeq;							// Sequence number for AUX devices
	bool auxEnabled;							// Do we have an AUX device?
	bool auxRaw;								// true if aux device is in raw mode

#ifdef SERIAL_AUX2_DEVICE
    volatile OutputStack aux2Output;
	Mutex aux2Mutex;
#endif

	volatile OutputStack usbOutput;
	Mutex usbMutex;

	// Files
#if HAS_MASS_STORAGE
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

#if SUPPORT_LASER
	PwmPort laserPort;
	float lastLaserPwm;
#endif

	// Power on/off
	bool deferredPowerDown;

	// Misc
	bool deliberateError;								// true if we deliberately caused an exception for testing purposes
};

#if HAS_MASS_STORAGE

// Where the htm etc files are
inline const char* Platform::GetWebDir() const noexcept
{
	return WEB_DIR;
}

// Where the gcodes are
inline const char* Platform::GetGCodeDir() const noexcept
{
	return GCODE_DIR;
}

inline const char* Platform::GetMacroDir() const noexcept
{
	return MACRO_DIR;
}

#endif

//*****************************************************************************************************************

// Drive the RepRap machine - Movement

inline float Platform::DriveStepsPerUnit(size_t drive) const noexcept
{
	return driveStepsPerUnit[drive];
}

inline float Platform::Acceleration(size_t drive) const noexcept
{
	return accelerations[drive];
}

inline const float* Platform::Accelerations() const noexcept
{
	return accelerations;
}

inline void Platform::SetAcceleration(size_t drive, float value) noexcept
{
	accelerations[drive] = max<float>(value, 1.0);		// don't allow zero or negative
}

inline float Platform::MaxFeedrate(size_t drive) const noexcept
{
	return maxFeedrates[drive];
}

inline void Platform::SetMaxFeedrate(size_t drive, float value) noexcept
{
	maxFeedrates[drive] = max<float>(value, minimumMovementSpeed);	// don't allow zero or negative, but do allow small values
}

inline float Platform::GetInstantDv(size_t drive) const noexcept
{
	return instantDvs[drive];
}

inline void Platform::SetInstantDv(size_t drive, float value) noexcept
{
	instantDvs[drive] = max<float>(value, 0.1);			// don't allow zero or negative values, they causes Move to loop indefinitely
}

inline void Platform::SetDirectionValue(size_t drive, bool dVal) noexcept
{
	directions[drive] = dVal;
}

inline bool Platform::GetDirectionValue(size_t drive) const noexcept
{
	return directions[drive];
}

inline void Platform::SetDriverDirection(uint8_t driver, bool direction) noexcept
{
	if (driver < NumDirectDrivers)
	{
		const bool d = (direction == FORWARDS) ? directions[driver] : !directions[driver];
		digitalWrite(DIRECTION_PINS[driver], d);
	}
}

inline void Platform::SetDriverAbsoluteDirection(size_t driver, bool direction) noexcept
{
	if (driver < NumDirectDrivers)
	{
		digitalWrite(DIRECTION_PINS[driver], direction);
	}
}

inline int8_t Platform::GetEnableValue(size_t driver) const noexcept
{
	return enableValues[driver];
}

inline float Platform::AxisMaximum(size_t axis) const noexcept
{
	return axisMaxima[axis];
}

inline float Platform::AxisMinimum(size_t axis) const noexcept
{
	return axisMinima[axis];
}

inline float Platform::AxisTotalLength(size_t axis) const noexcept
{
	return axisMaxima[axis] - axisMinima[axis];
}

// For the Duet we use the fan output for this
// DC 2015-03-21: To allow users to control the cooling fan via gcodes generated by slic3r etc.,
// only turn the fan on/off if the extruder ancilliary PWM has been set nonzero.
// Caution: this is often called from an ISR, or with interrupts disabled!
inline void Platform::ExtrudeOn() noexcept
{
	if (extrusionAncilliaryPwmValue > 0.0)
	{
		extrusionAncilliaryPwmPort.WriteAnalog(extrusionAncilliaryPwmValue);
	}
}

// DC 2015-03-21: To allow users to control the cooling fan via gcodes generated by slic3r etc.,
// only turn the fan on/off if the extruder ancilliary PWM has been set nonzero.
// Caution: this is often called from an ISR, or with interrupts disabled!
inline void Platform::ExtrudeOff() noexcept
{
	if (extrusionAncilliaryPwmValue > 0.0)
	{
		extrusionAncilliaryPwmPort.WriteAnalog(0.0);
	}
}

//********************************************************************************************************

// Drive the RepRap machine - Heat and temperature

inline IPAddress Platform::GetIPAddress() const noexcept
{
	return ipAddress;
}

inline IPAddress Platform::NetMask() const noexcept
{
	return netMask;
}

inline IPAddress Platform::GateWay() const noexcept
{
	return gateWay;
}

inline float Platform::GetPressureAdvance(size_t extruder) const noexcept
{
	return (extruder < MaxExtruders) ? pressureAdvance[extruder] : 0.0;
}

inline float Platform::GetFilamentWidth() const noexcept
{
	return filamentWidth;
}

inline void Platform::SetFilamentWidth(float width) noexcept
{
	filamentWidth = width;
}

inline float Platform::GetNozzleDiameter() const noexcept
{
	return nozzleDiameter;
}

inline void Platform::SetNozzleDiameter(float diameter) noexcept
{
	nozzleDiameter = diameter;
}

#endif
