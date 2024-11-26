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

#include <TemperatureError.h>
#include "OutputMemory.h"
#include "UniqueId.h"
#include "AveragingFilter.h"
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

#if SUPPORT_LED_STRIPS
# include <LedStrips/LedStripManager.h>
#endif

#if defined(DUET_NG)
# include "DueXn.h"
#endif

#if SUPPORT_CAN_EXPANSION
# include <CanMessageFormats.h>
# include <RemoteInputHandle.h>
#endif

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
	Auto = 0,						// this value is no longer used
#if defined(DUET3MINI_V04)			// we use the same values for both v0.2 and v0.4
	Duet3Mini_Unknown,
	Duet3Mini_WiFi,
	Duet3Mini_Ethernet,
#elif defined(DUET3_MB6HC)
	Duet3_6HC_v06_100 = 1,
	Duet3_6HC_v101 = 2,
	Duet3_6HC_v102 = 3,
	Duet3_6HC_v102b = 4,
#elif defined(DUET3_MB6XD)
	Duet3_6XD_v01 = 1,
	Duet3_6XD_v100 = 2,
	Duet3_6XD_v101 = 3,
	Duet3_6XD_v102 = 4,
#elif defined(FMDC_V02) || defined(FMDC_V03)
	FMDC,
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
	TimeCalculations = 102,			// do a timing test on the square root function and sine/cosine
	unused1 = 103,					// was TimeSinCos
	TimeSDWrite = 104,				// do a write timing test on the SD card
	PrintObjectSizes = 105,			// print the sizes of various objects
	PrintObjectAddresses = 106,		// print the addresses and sizes of various objects
	TimeCRC32 = 107,				// time how long it takes to calculate CRC32
	TimeGetTimerTicks = 108,		// time now long it takes to read the step clock
	UndervoltageEvent = 109,		// pretend an undervoltage condition has occurred

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

// The main class that defines the RepRap machine for the benefit of the other classes
class Platform INHERIT_OBJECT_MODEL
{
public:
	Platform() noexcept;
	Platform(const Platform&) = delete;

//-------------------------------------------------------------------------------------------------------------

	// These are the functions that form the interface between Platform and the rest of the firmware.

	void Init() noexcept;									// Set the machine up after a restart.  If called subsequently this should set the machine up as if
															// it has just been restarted; it can do this by executing an actual restart if you like, but beware the loop of death...
	void Spin() noexcept;									// This gets called in the main loop and should do any housekeeping needed
	void Exit() noexcept;									// Shut down tidily. Calling Init after calling this should reset to the beginning

	void Diagnostics(MessageType mtype) noexcept;
	static const char *_ecv_array GetResetReasonText() noexcept;
	GCodeResult DiagnosticTest(GCodeBuffer& gb, const StringRef& reply, OutputBuffer *_ecv_null & buf, unsigned int d) THROWS(GCodeException);
	static bool WasDeliberateError() noexcept { return deliberateError; }
	void LogError(ErrorCode e) noexcept { errorCodeBits |= (uint32_t)e; }

	bool GetAtxPowerState() const noexcept;
	GCodeResult HandleM80(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);
	GCodeResult HandleM81(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);
	GCodeResult HandleM575(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);
	GCodeResult SendI2cOrModbus(GCodeBuffer& gb, const StringRef &reply) THROWS(GCodeException);			// Handle M260
	GCodeResult ReceiveI2cOrModbus(GCodeBuffer& gb, const StringRef &reply) THROWS(GCodeException);			// Handle M261

	void AtxPowerOff() noexcept;
	bool IsAtxPowerControlled() const noexcept { return PsOnPort.IsValid(); }
	bool IsDeferredPowerDown() const noexcept { return powerDownWhenFansStop || delayedPowerDown; }
	const IoPort& GetAtxPowerPort() const noexcept { return PsOnPort; }

	BoardType GetBoardType() const noexcept { return board; }
	void SetBoardType() noexcept;
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
    bool IsAuxRaw(size_t auxNumber) const noexcept;
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

	// File functions
#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE || HAS_EMBEDDED_FILES
	FileStore *_ecv_null OpenFile(const char *_ecv_array folder, const char *_ecv_array fileName, OpenMode mode, uint32_t preAllocSize = 0) const noexcept;
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
	FileStore *_ecv_null OpenSysFile(const char *_ecv_array filename, OpenMode mode) const noexcept;
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
	void StopLogging() noexcept;

	// Buzzer via configured PWM port
	GCodeResult SetBuzzerPort(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);
	bool Beep(unsigned int freq, unsigned int ms) noexcept;

	// Movement
	void EmergencyStop() noexcept;
#if SUPPORT_LED_STRIPS
	LedStripManager& GetLedStripManager() noexcept { return ledStripManager; }
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

#if HAS_VOLTAGE_MONITOR || HAS_12V_MONITOR
	void ResetVoltageMonitors() noexcept;
	bool HasDriverPower() const noexcept;
	float GetVinVoltage() const noexcept;
# if HAS_SMART_DRIVERS
	void WarnDriverNotPowered() noexcept { warnDriversNotPowered = true; }
# endif
#else
	void ResetVoltageMonitors() noexcept { }
	bool HasDriverPower() const noexcept { return true; }
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

#if defined(DUET_NG) && HAS_SBC_INTERFACE
	void EnablePanelDuePort() noexcept;							// enable the PanelDue port so that the ATE can test the board
#endif

#if SUPPORT_CAN_EXPANSION
	void HandleRemoteGpInChange(CanAddress src, uint8_t handleMajor, uint8_t handleMinor, bool state) noexcept;
#endif

#if SUPPORT_REMOTE_COMMANDS
	GCodeResult EutHandleM950Gpio(const CanMessageGeneric& msg, const StringRef& reply) noexcept;
	GCodeResult EutHandleGpioWrite(const CanMessageWriteGpio& msg, const StringRef& reply) noexcept;
#endif

#if SUPPORT_CAN_EXPANSION
	void OnProcessingCanMessage() noexcept;										// called when we start processing any CAN message except for regular messages e.g. time sync
#endif

#if defined(DUET3_MB6HC)
	static BoardType GetMB6HCBoardType() noexcept;								// this is safe to call before Platform has been created
#endif
#if defined(DUET3_MB6XD)
	static BoardType GetMB6XDBoardType() noexcept;								// this is safe to call before Platform has been created
#endif

	void SetDiagLed(bool on) const noexcept;

#if SUPPORT_MULTICAST_DISCOVERY
	void InvertDiagLed() const noexcept;
#endif

#if defined(DUET3MINI) && SUPPORT_TMC2240 != 0
	bool HasTmc2240Expansion() const noexcept { return hasTmc2240Expansion; }
	const char *_ecv_array null GetExpansionBoardName() const noexcept { return (hasTmc2240Expansion) ? "Duet3 Mini 2+ (TMC2240)" : nullptr; }
#endif

	static bool HasDebugBuffer() noexcept;
	static bool IsrDebugPutc(char c) noexcept;
	static bool SetDebugBufferSize(uint32_t size) noexcept;

protected:
	DECLARE_OBJECT_MODEL_WITH_ARRAYS

private:
	const char *_ecv_array InternalGetSysDir() const noexcept;  				// where the system files are - not thread-safe!
	void RawMessage(MessageType type, const char *_ecv_array message) noexcept;	// called by Message after handling error/warning flags
	float GetCpuTemperature() const noexcept;
	GCodeResult PrintTestReport(GCodeBuffer& gb, const StringRef& reply, OutputBuffer *_ecv_null & buf) const THROWS(GCodeException);

#if HAS_SMART_DRIVERS
	void ReportDrivers(MessageType mt, DriversBitmap& whichDrivers, const char *_ecv_array text, bool& reported) noexcept;
#endif

#if defined(DUET3_MB6HC)
	float AdcReadingToPowerVoltage(uint16_t adcVal) const noexcept;
	uint16_t PowerVoltageToAdcReading(float voltage) const noexcept;
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

#if SUPPORT_LED_STRIPS
	// LED strips
	LedStripManager ledStripManager;
#endif

	bool active;
	uint32_t errorCodeBits;

	void InitialiseInterrupts() noexcept;

	uint8_t nextDriveToPoll;
	bool driversPowered;

#if defined(DUET3MINI) && SUPPORT_TMC2240 != 0
	bool hasTmc2240Expansion;
#endif

#if HAS_SMART_DRIVERS && (HAS_VOLTAGE_MONITOR || HAS_12V_MONITOR)
	bool warnDriversNotPowered;
#endif

	// Endstops
	EndstopsManager endstops;

	// Z probe
	volatile ZProbeAveragingFilter zProbeOnFilter;					// Z probe readings we took with the IR turned on
	volatile ZProbeAveragingFilter zProbeOffFilter;					// Z probe readings we took with the IR turned off

	// GPIO pins
	GpOutputPort gpoutPorts[MaxGpOutPorts];
	GpInputPort gpinPorts[MaxGpInPorts];

	// Buzzer
	PwmPort buzzerPort;
	volatile unsigned int beepTicksToGo;
	void StopBeep() noexcept;

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

	// Fans
	uint32_t lastFanCheckTime;

  	// Serial/USB
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

#ifdef DUET3_MB6HC
	float powerMonitorVoltageRange;
	uint16_t driverPowerOnAdcReading;
	uint16_t driverPowerOffAdcReading;
	Pin DiagPin;
	Pin ActLedPin;
	bool DiagOnPolarity;
#endif

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

#if SUPPORT_CAN_EXPANSION
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
	uint32_t whenToPowerDown;							// power down delay in milliseconds
	bool powerDownWhenFansStop;							// true if power down scheduled when all thermostatic fans stop
	bool delayedPowerDown;								// true if power down scheduled after the delay time

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
