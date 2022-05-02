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

#include <RepRapFirmware.h>
#include <ObjectModel/ObjectModel.h>
#include <Hardware/IoPorts.h>
#include <Fans/FansManager.h>
#include <Heating/TemperatureError.h>
#include "OutputMemory.h"
#include "UniqueId.h"
#include <Storage/FileStore.h>
#include <Storage/FileData.h>
#include <Storage/MassStorage.h>	// must be after Pins.h because it needs NumSdCards defined
#include <Tools/Spindle.h>
#include <Endstops/EndstopsManager.h>
#include <GPIO/GpInPort.h>
#include <GPIO/GpOutPort.h>
#include <Comms/AuxDevice.h>
#include <Comms/PanelDueUpdater.h>
#include <General/IPAddress.h>
#include <General/function_ref.h>

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

constexpr bool FORWARDS = true;
constexpr bool BACKWARDS = !FORWARDS;

// Define the number of ADC filters and the indices of the extra ones
// Note, the thermistor code assumes that the first N filters are used by the TEMP0 to TEMP(N-1) thermistor inputs, where N = NumThermistorInputs
#if HAS_VREF_MONITOR
constexpr size_t VrefFilterIndex = NumThermistorInputs;
constexpr size_t VssaFilterIndex = NumThermistorInputs + 1;
# if HAS_CPU_TEMP_SENSOR && !SAME5x
constexpr size_t CpuTempFilterIndex = NumThermistorInputs + 2;
constexpr size_t NumAdcFilters = NumThermistorInputs + 3;
# else
constexpr size_t NumAdcFilters = NumThermistorInputs + 2;
# endif
#elif HAS_CPU_TEMP_SENSOR && !SAME5x
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
#if SAME70
// On the SAME70 we read a thermistor on every tick so that we can average a higher number of readings
// Keep THERMISTOR_AVERAGE_READINGS * NUM_HEATERS * 1ms no greater than HEAT_SAMPLE_TIME or the PIDs won't work well.
constexpr unsigned int ThermistorAverageReadings = 16;
#else
// We read a thermistor on alternate ticks
// Keep THERMISTOR_AVERAGE_READINGS * NUM_HEATERS * 2ms no greater than HEAT_SAMPLE_TIME or the PIDs won't work well.
constexpr unsigned int ThermistorAverageReadings = 16;
#endif

#if SAME5x
constexpr unsigned int TempSenseAverageReadings = 16;
#endif

constexpr uint32_t maxPidSpinDelay = 5000;			// Maximum elapsed time in milliseconds between successive temp samples by Pid::Spin() permitted for a temp sensor

/****************************************************************************************************/

enum class BoardType : uint8_t
{
	Auto = 0,
#if defined(DUET3MINI_V04)			// we use the same values for both v0.2 and v0.4
	Duet3Mini_Unknown,
	Duet3Mini_WiFi,
	Duet3Mini_Ethernet,
#elif defined(DUET3_MB6HC)
	Duet3_6HC_v06_100 = 1,
	Duet3_6HC_v101 = 2,
#elif defined(DUET3_MB6XD)
	Duet3_6XD = 1,
#elif defined(FMDC_V02)
	FMDC,
#elif defined(SAME70XPLD)
	SAME70XPLD_0 = 1
#elif defined(DUET_NG)
	DuetWiFi_10 = 1,
	DuetWiFi_102 = 2,
	DuetEthernet_10 = 3,
	DuetEthernet_102 = 4,
	Duet2SBC_10 = 5,
	Duet2SBC_102 = 6,
#elif defined(DUET_M)
	DuetM_10 = 1,
#elif defined(PCCB_10)
	PCCB_v10 = 1
#elif defined(__LPC17xx__)
	Lpc = 1
#else
# error Unknown board
#endif
};

// Type of an axis. The values must correspond to values of the R parameter in the M584 command.
enum class AxisWrapType : uint8_t
{
	noWrap = 0,						// axis does not wrap
	wrapAt360,						// axis wraps, actual position are modulo 360deg
#if 0	// shortcut axes not implemented yet
	wrapWithShortcut,				// axis wraps, G0 moves are allowed to take the shortest direction
#endif
	undefined						// this one must be last
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
	TimeCalculations = 102,			// do a timing test on the square root function and sine/cosine
	unused1 = 103,					// was TimeSinCos
	TimeSDWrite = 104,				// do a write timing test on the SD card
	PrintObjectSizes = 105,			// print the sizes of various objects
	PrintObjectAddresses = 106,		// print the addresses and sizes of various objects
	TimeCRC32 = 107,				// time how long it takes to calculate CRC32
	TimeGetTimerTicks = 108,		// time now long it takes to read the step clock
	UndervoltageEvent = 109,		// pretend an undervoltage condition has occurred

#ifdef __LPC17xx__
	PrintBoardConfiguration = 200,	// Prints out all pin/values loaded from SDCard to configure board
#endif

	SetWriteBuffer = 500,			// enable/disable the write buffer

	OutputBufferStarvation = 900,	// Allocate almost all output buffers to emulate starvation

	TestWatchdog = 1001,			// test that we get a watchdog reset if the tick interrupt stops
	TestSpinLockup = 1002,			// test that we get a software reset if a Spin() function takes too long
	TestSerialBlock = 1003,			// test what happens when we write a blocking message via debugPrintf()
	DivideByZero = 1004,			// do an integer divide by zero to test exception handling
	UnalignedMemoryAccess = 1005,	// do an unaligned memory access to test exception handling
	BusFault = 1006,				// generate a bus fault
	AccessMemory = 1007				// read or write  memory
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
		AtomicCriticalSectionLocker lock;

		sum = (uint32_t)val * (uint32_t)numAveraged;
		index = 0;
		isValid = false;
		for (size_t i = 0; i < numAveraged; ++i)
		{
			readings[i] = val;
		}
	}

	// Call this to put a new reading into the filter
	// This is called by the ISR and by the ADC callback function
	void ProcessReading(uint16_t r) volatile noexcept
	{
		size_t locIndex = index;				// avoid repeatedly reloading volatile variable
		sum = sum - readings[locIndex] + r;
		readings[locIndex] = r;
		++locIndex;
		if (locIndex == numAveraged)
		{
			locIndex = 0;
			isValid = true;
		}
		index = locIndex;
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

	static constexpr size_t NumAveraged() noexcept { return numAveraged; }

	// Function used as an ADC callback to feed a result into an averaging filter
	static void CallbackFeedIntoFilter(CallbackParameter cp, uint16_t val) noexcept;

private:
	uint16_t readings[numAveraged];
	size_t index;
	uint32_t sum;
	bool isValid;
	//invariant(sum == + over readings)
	//invariant(index < numAveraged)
};

template<size_t numAveraged> void AveragingFilter<numAveraged>::CallbackFeedIntoFilter(CallbackParameter cp, uint16_t val) noexcept
{
	static_cast<AveragingFilter<numAveraged>*>(cp.vp)->ProcessReading(val);
}

typedef AveragingFilter<ThermistorAverageReadings> ThermistorAveragingFilter;
typedef AveragingFilter<ZProbeAverageReadings> ZProbeAveragingFilter;

#if SAME5x
typedef AveragingFilter<TempSenseAverageReadings> TempSenseAveragingFilter;
#endif

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

#if SUPPORT_NONLINEAR_EXTRUSION

struct NonlinearExtrusion
{
	float A;
	float B;
	float limit;
};

#endif

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
	static bool WasDeliberateError() noexcept { return deliberateError; }
	void LogError(ErrorCode e) noexcept { errorCodeBits |= (uint32_t)e; }

	bool GetAtxPowerState() const noexcept;
	GCodeResult HandleM80(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);
	GCodeResult HandleM81(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);
	void AtxPowerOff() noexcept;
	bool IsAtxPowerControlled() const noexcept { return PsOnPort.IsValid(); }
	bool IsDeferredPowerDown() const noexcept { return deferredPowerDown; }
	const IoPort& GetAtxPowerPort() const noexcept { return PsOnPort; }

	BoardType GetBoardType() const noexcept { return board; }
	void SetBoardType(BoardType bt) noexcept;
	const char *_ecv_array GetElectronicsString() const noexcept;
	const char *_ecv_array GetBoardString() const noexcept;

#if SUPPORT_OBJECT_MODEL
	size_t GetNumGpInputsToReport() const noexcept;
	size_t GetNumGpOutputsToReport() const noexcept;
#endif

#if defined(DUET_NG) || defined(DUET3MINI)
	bool IsDuetWiFi() const noexcept;
#endif

#ifdef DUET_NG
	const char *_ecv_array GetBoardName() const noexcept;
	const char *_ecv_array GetBoardShortName() const noexcept;

	const float GetDefaultThermistorSeriesR(size_t inputNumber) const noexcept
	{
		// This is only called from one place so we may as well inline it
		return (inputNumber >= 3 && (expansionBoard == ExpansionBoardType::DueX5_v0_11 || expansionBoard == ExpansionBoardType::DueX2_v0_11))
			? DefaultThermistorSeriesR_DueX_v0_11
				: DefaultThermistorSeriesR;
	}
#endif

	const MacAddress& GetDefaultMacAddress() const noexcept { return defaultMacAddress; }

	// Timing
	void Tick() noexcept SPEED_CRITICAL;			// Process a systick interrupt

	// Real-time clock
	bool IsDateTimeSet() const noexcept { return realTime != 0; }	// Has the RTC been set yet?
	time_t GetDateTime() const noexcept { return realTime; }		// Retrieves the current RTC datetime
	bool GetDateTime(tm& rslt) const noexcept { return gmtime_r(&realTime, &rslt) != nullptr && realTime != 0; }
																	// Retrieves the broken-down current RTC datetime and returns true if it's valid
	bool SetDateTime(time_t t) noexcept;							// Sets the current RTC date and time or returns false on error

  	// Communications and data storage
	void AppendUsbReply(OutputBuffer *buffer) noexcept;
	void AppendAuxReply(size_t auxNumber, OutputBuffer *buf, bool rawMessage) noexcept;
	void AppendAuxReply(size_t auxNumber, const char *_ecv_array msg, bool rawMessage) noexcept;

	void ResetChannel(size_t chan) noexcept;						// Re-initialise a serial channel
    bool IsAuxEnabled(size_t auxNumber) const noexcept;				// Any device on the AUX line?
    void EnableAux(size_t auxNumber) noexcept;
    bool IsAuxRaw(size_t auxNumber) const noexcept;
	void SetAuxRaw(size_t auxNumber, bool raw) noexcept;
#if SUPPORT_PANELDUE_FLASH
	PanelDueUpdater* GetPanelDueUpdater() noexcept { return panelDueUpdater; }
	void InitPanelDueUpdater() noexcept;
#endif

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
#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE || HAS_EMBEDDED_FILES
	FileStore* OpenFile(const char *_ecv_array folder, const char *_ecv_array fileName, OpenMode mode, uint32_t preAllocSize = 0) const noexcept;
	bool FileExists(const char *_ecv_array folder, const char *_ecv_array filename) const noexcept;
# if HAS_MASS_STORAGE || HAS_SBC_INTERFACE
	bool Delete(const char *_ecv_array folder, const char *_ecv_array filename) const noexcept;
# endif

	static const char *_ecv_array GetWebDir() noexcept; 		// Where the html etc files are
	static const char *_ecv_array GetGCodeDir() noexcept; 		// Where the gcodes are
	static const char *_ecv_array GetMacroDir() noexcept;		// Where the user-defined macros are

	// Functions to work with the system files folder
	GCodeResult SetSysDir(const char *_ecv_array dir, const StringRef& reply) noexcept;				// Set the system files path
	bool SysFileExists(const char *_ecv_array filename) const noexcept;
	FileStore* OpenSysFile(const char *_ecv_array filename, OpenMode mode) const noexcept;
# if HAS_MASS_STORAGE || HAS_SBC_INTERFACE
	bool DeleteSysFile(const char *_ecv_array filename) const noexcept;
# endif
	bool MakeSysFileName(const StringRef& rslt, const char *_ecv_array filename) const noexcept;
	void AppendSysDir(const StringRef & path) const noexcept;
	ReadLockedPointer<const char> GetSysDir() const noexcept;	// where the system files are
#endif

	// Message output (see MessageType for further details)
	void Message(MessageType type, const char *_ecv_array message) noexcept;
	void Message(MessageType type, OutputBuffer *buffer) noexcept;
	void MessageF(MessageType type, const char *_ecv_array fmt, ...) noexcept __attribute__ ((format (printf, 3, 4)));
	void MessageV(MessageType type, const char *_ecv_array fmt, va_list vargs) noexcept;
	void DebugMessage(const char *_ecv_array fmt, va_list vargs) noexcept;
	bool FlushMessages() noexcept;								// Flush messages to USB and aux, returning true if there is more to send
	void SendAlert(MessageType mt, const char *_ecv_array message, const char *_ecv_array title, int sParam, float tParam, AxesBitmap controls) noexcept;
	void StopLogging() noexcept;

	// Movement
	void EmergencyStop() noexcept;
	size_t GetNumActualDirectDrivers() const noexcept;
	void SetDirection(size_t axisOrExtruder, bool direction) noexcept;
	void SetDirectionValue(size_t driver, bool dVal) noexcept;
	bool GetDirectionValue(size_t driver) const noexcept;
	void SetDriverAbsoluteDirection(size_t driver, bool dVal) noexcept;
	void SetEnableValue(size_t driver, int8_t eVal) noexcept;
	int8_t GetEnableValue(size_t driver) const noexcept;
	void EnableDrivers(size_t axisOrExtruder, bool unconditional) noexcept;
	void EnableOneLocalDriver(size_t driver, float requiredCurrent) noexcept;
	void DisableAllDrivers() noexcept;
	void DisableDrivers(size_t axisOrExtruder) noexcept;
	void DisableOneLocalDriver(size_t driver) noexcept;
	void EmergencyDisableDrivers() noexcept;
	void SetDriversIdle() noexcept;
	GCodeResult ConfigureDriverBrakePort(GCodeBuffer& gb, const StringRef& reply, size_t driver) noexcept
		pre(driver < GetNumActualDirectDrivers());
	GCodeResult SetMotorCurrent(size_t axisOrExtruder, float current, int code, const StringRef& reply) noexcept;
	int GetMotorCurrent(size_t axisOrExtruder, int code) const noexcept;
	void SetIdleCurrentFactor(float f) noexcept;
	float GetIdleCurrentFactor() const noexcept { return idleCurrentFactor; }
	bool SetDriverMicrostepping(size_t driver, unsigned int microsteps, int mode) noexcept;
	bool SetMicrostepping(size_t axisOrExtruder, int microsteps, bool mode, const StringRef& reply) noexcept;
	unsigned int GetMicrostepping(size_t axisOrExtruder, bool& interpolation) const noexcept;
	void SetDriverStepTiming(size_t driver, const float microseconds[4]) noexcept;
	bool GetDriverStepTiming(size_t driver, float microseconds[4]) const noexcept;

#ifdef DUET3_MB6XD
	void GetActualDriverTimings(float timings[4]) noexcept;
#endif

	float DriveStepsPerUnit(size_t axisOrExtruder) const noexcept;
	const float *_ecv_array GetDriveStepsPerUnit() const noexcept
		{ return driveStepsPerUnit; }
	void SetDriveStepsPerUnit(size_t axisOrExtruder, float value, uint32_t requestedMicrostepping) noexcept;
	float Acceleration(size_t axisOrExtruder) const noexcept;
	const float *_ecv_array Accelerations(bool useReduced) const noexcept;
	void SetAcceleration(size_t axisOrExtruder, float value, bool reduced) noexcept;
	float MaxFeedrate(size_t axisOrExtruder) const noexcept;
	const float *_ecv_array MaxFeedrates() const noexcept { return maxFeedrates; }
	void SetMaxFeedrate(size_t axisOrExtruder, float value) noexcept;
	float MinMovementSpeed() const noexcept { return minimumMovementSpeed; }
	void SetMinMovementSpeed(float value) noexcept;
	float GetInstantDv(size_t axis) const noexcept;
	void SetInstantDv(size_t axis, float value) noexcept;
	float AxisMaximum(size_t axis) const noexcept;
	void SetAxisMaximum(size_t axis, float value, bool byProbing) noexcept;
	float AxisMinimum(size_t axis) const noexcept;
	void SetAxisMinimum(size_t axis, float value, bool byProbing) noexcept;
	float AxisTotalLength(size_t axis) const noexcept;

	inline AxesBitmap GetLinearAxes() const noexcept { return linearAxes; }
	inline AxesBitmap GetRotationalAxes() const noexcept { return rotationalAxes; }
	inline bool IsAxisRotational(size_t axis) const noexcept { return rotationalAxes.IsBitSet(axis); }
	inline bool IsAxisContinuous(size_t axis) const noexcept { return continuousAxes.IsBitSet(axis); }
#if 0	// shortcut axes not implemented yet
	inline bool IsAxisShortcutAllowed(size_t axis) const noexcept { return shortcutAxes.IsBitSet(axis); }
#endif

	void SetAxisType(size_t axis, AxisWrapType wrapType, bool isNistRotational) noexcept;

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
		pre(axisOrExtruder < MaxAxesPlusExtruders + NumDirectDrivers)
		{ return driveDriverBits[axisOrExtruder]; }

#ifdef DUET3_MB6XD		// the first element has a special meaning when we use a TC to generate the steps
	uint32_t GetSlowDriverStepPeriodClocks() { return stepPulseMinimumPeriodClocks; }
	uint32_t GetSlowDriverDirHoldClocksFromLeadingEdge() { return directionHoldClocksFromLeadingEdge; }
	uint32_t GetSlowDriverDirSetupClocks() const noexcept { return directionSetupClocks; }
#else
	uint32_t GetSlowDriversBitmap() const noexcept { return slowDriversBitmap; }
	uint32_t GetSlowDriverStepHighClocks() const noexcept { return slowDriverStepTimingClocks[0]; }
	uint32_t GetSlowDriverStepLowClocks() const noexcept { return slowDriverStepTimingClocks[1]; }
	uint32_t GetSlowDriverDirHoldClocksFromTrailingEdge() const noexcept { return slowDriverStepTimingClocks[3]; }
	uint32_t GetSlowDriverDirSetupClocks() const noexcept { return slowDriverStepTimingClocks[2]; }
#endif

	uint32_t GetSteppingEnabledDrivers() const noexcept { return steppingEnabledDriversBitmap; }
	void DisableSteppingDriver(uint8_t driver) noexcept { steppingEnabledDriversBitmap &= ~StepPins::CalcDriverBitmap(driver); }
	void EnableAllSteppingDrivers() noexcept { steppingEnabledDriversBitmap = 0xFFFFFFFFu; }

#ifdef DUET3_MB6XD
	bool HasDriverError(size_t driver) const noexcept;
#endif

#if SUPPORT_NONLINEAR_EXTRUSION
	const NonlinearExtrusion& GetExtrusionCoefficients(size_t extruder) const noexcept pre(extruder < MaxExtruders) { return nonlinearExtrusion[extruder]; }
	void SetNonlinearExtrusion(size_t extruder, float a, float b, float limit) noexcept;
#endif

	// Endstops and Z probe
	EndstopsManager& GetEndstops() noexcept { return endstops; }
	ReadLockedPointer<ZProbe> GetZProbeOrDefault(size_t probeNumber) noexcept { return endstops.GetZProbeOrDefault(probeNumber); }
	void InitZProbeFilters() noexcept;
	const volatile ZProbeAveragingFilter& GetZProbeOnFilter() const noexcept { return zProbeOnFilter; }
	const volatile ZProbeAveragingFilter& GetZProbeOffFilter() const noexcept{ return zProbeOffFilter; }

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE
	bool WritePlatformParameters(FileStore *f, bool includingG31) const noexcept;
#endif

	// Heat and temperature
	volatile ThermistorAveragingFilter& GetAdcFilter(size_t channel) noexcept
	pre(channel < ARRAY_SIZE(adcFilters))
	{
		return adcFilters[channel];
	}

	int GetAveragingFilterIndex(const IoPort&) const noexcept;

	// AUX device
	void PanelDueBeep(int freq, int ms) noexcept;
	void SendPanelDueMessage(size_t auxNumber, const char *_ecv_array msg) noexcept;

	// Hotend configuration
	float GetFilamentWidth() const noexcept;
	void SetFilamentWidth(float width) noexcept;

	// Fire the inkjet (if any) in the given pattern
	// If there is no inkjet false is returned; if there is one this returns true
	// So you can test for inkjet presence with if(platform->Inkjet(0))
	bool Inkjet(int bitPattern) noexcept;

	// MCU temperature
#if HAS_CPU_TEMP_SENSOR
	MinCurMax GetMcuTemperatures() const noexcept;
	void SetMcuTemperatureAdjust(float v) noexcept { mcuTemperatureAdjust = v; }
	float GetMcuTemperatureAdjust() const noexcept { return mcuTemperatureAdjust; }
#endif

#if HAS_VOLTAGE_MONITOR
	// Power in voltage
	MinCurMax GetPowerVoltages() const noexcept;
	float GetCurrentPowerVoltage() const noexcept;
	bool IsPowerOk() const noexcept;
	void DisableAutoSave() noexcept;
	void EnableAutoSave(float saveVoltage, float resumeVoltage) noexcept;
	bool GetAutoSaveSettings(float& saveVoltage, float&resumeVoltage) noexcept;
#endif

#if HAS_12V_MONITOR
	// 12V rail voltage
	MinCurMax GetV12Voltages() const noexcept;
	float GetCurrentV12Voltage() const noexcept;
#endif

#if HAS_SMART_DRIVERS
	float GetTmcDriversTemperature(unsigned int boardNumber) const noexcept;
	unsigned int GetNumSmartDrivers() const noexcept { return numSmartDrivers; }
#endif

#if HAS_VOLTAGE_MONITOR || HAS_12V_MONITOR
	void ResetVoltageMonitors() noexcept;
	bool HasVinPower() const noexcept;
#else
	void ResetVoltageMonitors() noexcept { }
	bool HasVinPower() const noexcept { return true; }
#endif

#if HAS_STALL_DETECT
	GCodeResult ConfigureStallDetection(GCodeBuffer& gb, const StringRef& reply, OutputBuffer *& buf) THROWS(GCodeException);
#endif

	// Logging support
	const char *_ecv_array GetLogLevel() const noexcept;
#if HAS_MASS_STORAGE
	GCodeResult ConfigureLogging(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);
	const char *_ecv_array null GetLogFileName() const noexcept;
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
	void ReleaseLaserPin() noexcept;
	void SetLaserPwmFrequency(PwmFrequency freq) noexcept;
#endif

	// Misc
	GCodeResult ConfigurePort(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);

	GpOutputPort& GetGpOutPort(size_t gpoutPortNumber) noexcept
		pre(gpoutPortNumber < MaxGpOutPorts)	{ return gpoutPorts[gpoutPortNumber]; }
	const GpInputPort& GetGpInPort(size_t gpinPortNumber) const noexcept
		pre(gpinPortNumber < MaxGpInPorts) 	{ return gpinPorts[gpinPortNumber]; }

#if MCU_HAS_UNIQUE_ID
	const UniqueId& GetUniqueId() const noexcept { return uniqueId; }
	uint32_t Random() noexcept;
#endif

#if SUPPORT_CAN_EXPANSION
	void HandleRemoteGpInChange(CanAddress src, uint8_t handleMajor, uint8_t handleMinor, bool state) noexcept;
	GCodeResult UpdateRemoteStepsPerMmAndMicrostepping(AxesBitmap axesAndExtruders, const StringRef& reply) noexcept;
#endif

#if SUPPORT_REMOTE_COMMANDS
	GCodeResult EutHandleM950Gpio(const CanMessageGeneric& msg, const StringRef& reply) noexcept;
	GCodeResult EutHandleGpioWrite(const CanMessageWriteGpio& msg, const StringRef& reply) noexcept;
	GCodeResult EutSetMotorCurrents(const CanMessageMultipleDrivesRequest<float>& msg, size_t dataLength, const StringRef& reply) noexcept;
	GCodeResult EutSetStepsPerMmAndMicrostepping(const CanMessageMultipleDrivesRequest<StepsPerUnitAndMicrostepping>& msg, size_t dataLength, const StringRef& reply) noexcept;
	GCodeResult EutHandleSetDriverStates(const CanMessageMultipleDrivesRequest<DriverStateControl>& msg, const StringRef& reply) noexcept;
	GCodeResult EutProcessM569(const CanMessageGeneric& msg, const StringRef& reply) noexcept;
	GCodeResult EutProcessM569Point2(const CanMessageGeneric& msg, const StringRef& reply) noexcept;
	GCodeResult EutProcessM569Point7(const CanMessageGeneric& msg, const StringRef& reply) noexcept;
	GCodeResult EutProcessM915(const CanMessageGeneric& msg, const StringRef& reply) noexcept;
	void SendDriversStatus(CanMessageBuffer& buf) noexcept;
#endif

#if VARIABLE_NUM_DRIVERS
	void AdjustNumDrivers(size_t numDriversNotAvailable) noexcept;
#endif

#if SUPPORT_CAN_EXPANSION
	void OnProcessingCanMessage() noexcept;								// called when we start processing any CAN message except for regular messages e.g. time sync
#endif

protected:
	DECLARE_OBJECT_MODEL
	OBJECT_MODEL_ARRAY(axisDrivers)
	OBJECT_MODEL_ARRAY(workplaceOffsets)

private:
	const char *_ecv_array InternalGetSysDir() const noexcept;  				// where the system files are - not thread-safe!

	void RawMessage(MessageType type, const char *_ecv_array message) noexcept;	// called by Message after handling error/warning flags

	float GetCpuTemperature() const noexcept;

#if SUPPORT_CAN_EXPANSION
	void IterateDrivers(size_t axisOrExtruder, function_ref<void(uint8_t) /*noexcept*/ > localFunc, function_ref<void(DriverId) /*noexcept*/ > remoteFunc) noexcept;
	void IterateLocalDrivers(size_t axisOrExtruder, function_ref<void(uint8_t) /*noexcept*/ > func) noexcept { IterateDrivers(axisOrExtruder, func, [](DriverId) noexcept {}); }
	void IterateRemoteDrivers(size_t axisOrExtruder, function_ref<void(DriverId) /*noexcept*/ > func) noexcept { IterateDrivers(axisOrExtruder, [](uint8_t) noexcept {}, func); }
#else
	void IterateDrivers(size_t axisOrExtruder, function_ref<void(uint8_t) /*noexcept*/ > localFunc) noexcept;
	void IterateLocalDrivers(size_t axisOrExtruder, function_ref<void(uint8_t) /*noexcept*/ > func) noexcept { IterateDrivers(axisOrExtruder, func); }
#endif

#if HAS_SMART_DRIVERS
	void ReportDrivers(MessageType mt, DriversBitmap& whichDrivers, const char *_ecv_array text, bool& reported) noexcept;
#endif

	// Convert microseconds to step clocks, rounding up to the next step clock
	static constexpr uint32_t MicrosecondsToStepClocks(float us) noexcept
	{
		return (uint32_t)ceilf((float)StepClockRate * 0.000001 * us);
	}

#ifdef DUET3_MB6XD
	void UpdateDriverTimings() noexcept;
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
#if MCU_HAS_UNIQUE_ID
	UniqueId uniqueId;
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
	pre(driver < NumDirectDrivers);

#if VARIABLE_NUM_DRIVERS && SUPPORT_12864_LCD
	size_t numActualDirectDrivers;
#endif

	bool directions[NumDirectDrivers];
	int8_t enableValues[NumDirectDrivers];

#ifdef DUET3_MB6XD
	bool driverErrPinsActiveLow;
#endif

	IoPort brakePorts[NumDirectDrivers];

	float motorCurrents[MaxAxesPlusExtruders];				// the normal motor current for each stepper driver
	float motorCurrentFraction[MaxAxesPlusExtruders];		// the percentages of normal motor current that each driver is set to
	float standstillCurrentPercent[MaxAxesPlusExtruders];	// the percentages of normal motor current that each driver uses when in standstill
	uint16_t microstepping[MaxAxesPlusExtruders];			// the microstepping used for each axis or extruder, top bit is set if interpolation enabled

	volatile DriverStatus driverState[MaxAxesPlusExtruders];
	float maxFeedrates[MaxAxesPlusExtruders];				// max feed rates in mm per step clock
	float normalAccelerations[MaxAxesPlusExtruders];		// max accelerations in mm per step clock squared for normal moves
	float reducedAccelerations[MaxAxesPlusExtruders];		// max accelerations in mm per step clock squared for probing and stall detection moves
	float driveStepsPerUnit[MaxAxesPlusExtruders];
	float instantDvs[MaxAxesPlusExtruders];					// max jerk in mm per step clock
	uint32_t driveDriverBits[MaxAxesPlusExtruders + NumDirectDrivers];
															// the bitmap of local driver port bits for each axis or extruder, followed by the bitmaps for the individual Z motors
	AxisDriversConfig axisDrivers[MaxAxes];					// the driver numbers assigned to each axis
	AxesBitmap linearAxes;									// axes that behave like linear axes w.r.t. feedrate handling
	AxesBitmap rotationalAxes;								// axes that behave like rotational axes w.r.t. feedrate handling
	AxesBitmap continuousAxes;								// axes that wrap modulo 360
#if 0	// shortcut axes not implemented yet
	AxesBitmap shortcutAxes;								// axes that wrap modulo 360 and for which G0 may choose the shortest direction
#endif

#if SUPPORT_NONLINEAR_EXTRUSION
	NonlinearExtrusion nonlinearExtrusion[MaxExtruders];	// nonlinear extrusion coefficients
#endif

	DriverId extruderDrivers[MaxExtruders];					// the driver number assigned to each extruder
#ifdef DUET3_MB6XD
	float driverTimingMicroseconds[NumDirectDrivers][4];	// step high time, step low time, direction setup time to step high, direction hold time from step low (1 set per driver)
	uint32_t stepPulseMinimumPeriodClocks;					// minimum period between leading edges of step pulses, in step clocks
	uint32_t directionSetupClocks;							// minimum direction change to step high time, in step clocks
	uint32_t directionHoldClocksFromLeadingEdge;			// minimum step high to direction low step clocks, calculated from the step low to direction change hold time
#else
	uint32_t slowDriversBitmap;								// bitmap of driver port bits that need extended step pulse timing
	uint32_t slowDriverStepTimingClocks[4];					// minimum step high, step low, dir setup and dir hold timing for slow drivers
#endif
	uint32_t steppingEnabledDriversBitmap;					// mask of driver bits that we haven't disabled temporarily
	float idleCurrentFactor;
	float minimumMovementSpeed;								// minimum allowed movement speed in mm per step clock

#if HAS_SMART_DRIVERS
	size_t numSmartDrivers;											// the number of TMC drivers we have, the remaining are simple enable/step/dir drivers
	DriversBitmap temperatureShutdownDrivers, temperatureWarningDrivers, shortToGroundDrivers;
	MillisTimer openLoadTimers[MaxSmartDrivers];
#endif

	StandardDriverStatus lastEventStatus[NumDirectDrivers];
	uint8_t nextDriveToPoll;

	bool driversPowered;

#if HAS_SMART_DRIVERS && (HAS_VOLTAGE_MONITOR || HAS_12V_MONITOR)
	bool warnDriversNotPowered;
#endif

#if HAS_STALL_DETECT
	DriversBitmap logOnStallDrivers, eventOnStallDrivers;
#endif

#if defined(__LPC17xx__)
	MCP4461 mcp4451;	// works for 5561 (only volatile setting commands)
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
	float highestMcuTemperature, lowestMcuTemperature;
	float mcuTemperatureAdjust;
# if SAME5x
	TempSenseAveragingFilter tpFilter, tcFilter;
	int32_t tempCalF1, tempCalF2, tempCalF3, tempCalF4;				// temperature calibration factors
	void TemperatureCalibrationInit() noexcept;
# endif
#endif

	// Axes and endstops
	float axisMaxima[MaxAxes];
	float axisMinima[MaxAxes];
	AxesBitmap axisMinimaProbed, axisMaximaProbed;

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE
	static bool WriteAxisLimits(FileStore *f, AxesBitmap axesProbed, const float limits[MaxAxes], int sParam) noexcept;
#endif

	// Fans
	uint32_t lastFanCheckTime;

  	// Serial/USB
	uint32_t baudRates[NumSerialChannels];
	uint8_t commsParams[NumSerialChannels];

	volatile OutputStack usbOutput;
	Mutex usbMutex;

#if HAS_AUX_DEVICES
	AuxDevice auxDevices[NumSerialChannels - 1];
#endif
#if SUPPORT_PANELDUE_FLASH
	PanelDueUpdater* panelDueUpdater;
#endif

	// Files
#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE || HAS_EMBEDDED_FILES
	const char *_ecv_array sysDir;
	mutable ReadWriteLock sysDirLock;
#endif

	// Data used by the tick interrupt handler
	AnalogChannelNumber filteredAdcChannels[NumAdcFilters];
	AnalogChannelNumber zProbeAdcChannel;
	uint8_t tickState;
	size_t currentFilterNumber;
	int debugCode;

	// Hotend configuration
	float filamentWidth;

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

	// Event handling
	uint32_t lastDriverPollMillis;						// when we last checked the drivers and voltage monitoring

#ifdef DUET3MINI
	uint32_t whenLastCanMessageProcessed;
#endif

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
	IoPort PsOnPort;
	bool deferredPowerDown;

	// Misc
	static bool deliberateError;						// true if we deliberately caused an exception for testing purposes. Must be static in case of exception during startup.
};

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE || HAS_EMBEDDED_FILES

// Where the htm etc files are
inline const char *_ecv_array Platform::GetWebDir() noexcept
{
	return WEB_DIR;
}

// Where the gcodes are
inline const char *_ecv_array Platform::GetGCodeDir() noexcept
{
	return GCODE_DIR;
}

inline const char *_ecv_array Platform::GetMacroDir() noexcept
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
	return normalAccelerations[drive];
}

inline const float *_ecv_array Platform::Accelerations(bool useReduced) const noexcept
{
	return (useReduced) ? reducedAccelerations : normalAccelerations;
}

inline void Platform::SetAcceleration(size_t drive, float value, bool reduced) noexcept
{
	((reduced) ? reducedAccelerations : normalAccelerations)[drive] = max<float>(value, ConvertAcceleration(MinimumAcceleration));	// don't allow zero or negative
}

inline float Platform::MaxFeedrate(size_t drive) const noexcept
{
	return maxFeedrates[drive];
}

inline void Platform::SetMaxFeedrate(size_t drive, float value) noexcept
{
	maxFeedrates[drive] = max<float>(value, minimumMovementSpeed);						// don't allow zero or negative, but do allow small values
}

inline void Platform::SetInstantDv(size_t drive, float value) noexcept
{
	instantDvs[drive] = max<float>(value, ConvertSpeedFromMmPerSec(MinimumJerk));		// don't allow zero or negative values, they causes Move to loop indefinitely
}

inline void Platform::SetMinMovementSpeed(float value) noexcept
{
	minimumMovementSpeed = max<float>(value, ConvertSpeedFromMmPerSec(AbsoluteMinFeedrate));
}

inline float Platform::GetInstantDv(size_t drive) const noexcept
{
	return instantDvs[drive];
}

inline size_t Platform::GetNumActualDirectDrivers() const noexcept
{
#if VARIABLE_NUM_DRIVERS
	return numActualDirectDrivers;
#else
	return NumDirectDrivers;
#endif
}

#if VARIABLE_NUM_DRIVERS

inline void Platform::AdjustNumDrivers(size_t numDriversNotAvailable) noexcept
{
	numActualDirectDrivers = NumDirectDrivers - numDriversNotAvailable;
}

#endif

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
	if (driver < GetNumActualDirectDrivers())
	{
		const bool d = (direction == FORWARDS) ? directions[driver] : !directions[driver];
#if SAME5x
		IoPort::WriteDigital(DIRECTION_PINS[driver], d);
#else
		digitalWrite(DIRECTION_PINS[driver], d);
#endif
	}
}

inline void Platform::SetDriverAbsoluteDirection(size_t driver, bool direction) noexcept
{
	if (driver < GetNumActualDirectDrivers())
	{
#if SAME5x
		IoPort::WriteDigital(DIRECTION_PINS[driver], direction);
#else
		digitalWrite(DIRECTION_PINS[driver], direction);
#endif
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
// only turn the fan on/off if the extruder ancillary PWM has been set nonzero.
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

inline float Platform::GetFilamentWidth() const noexcept
{
	return filamentWidth;
}

inline void Platform::SetFilamentWidth(float width) noexcept
{
	filamentWidth = width;
}

#endif
