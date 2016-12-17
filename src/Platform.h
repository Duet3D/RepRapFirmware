/****************************************************************************************************

RepRapFirmware - Platform: RepRapPro Ormerod with Duet controller

Platform contains all the code and definitions to deal with machine-dependent things such as control
pins, bed area, number of extruders, tolerable accelerations and speeds and so on.

No definitions that are system-independent should go in here.  Put them in Configuration.h.  Note that
the lengths of arrays such as DRIVES (see below) are defined here, so any array initialiser that depends on those
lengths, for example:

#define DRIVES 4
.
.
.
#define DRIVE_RELATIVE_MODES {false, false, false, true}

also needs to go here.

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

#include <cctype>
#include <cstring>
#include <malloc.h>
#include <cstdlib>
#include <climits>
#include <ctime>

// Platform-specific includes

#include "Core.h"
#include "DueFlashStorage.h"
#include "Heating/TemperatureSensor.h"
#include "Heating/Thermistor.h"
#include "Heating/TemperatureError.h"
#include "OutputMemory.h"
#include "Libraries/Fatfs/ff.h"

#if defined(DUET_NG)
# include "DueXn.h"
#elif !defined(__RADDS__)
# include "MCP4461/MCP4461.h"
#endif

#include "Storage/FileStore.h"
#include "Storage/FileData.h"
#include "MessageType.h"

// Definitions needed by Fan.h
const float SecondsToMillis = 1000.0;
const float MillisToSeconds = 0.001;

#include "Fan.h"

// Definitions needed by Pins.h
const bool FORWARDS = true;
const bool BACKWARDS = !FORWARDS;

#include "Pins.h"
#include "Storage/MassStorage.h"	// must be after Pins.h because it needs NumSdCards defined

/**************************************************************************************************/

// Some constants
#define TIME_TO_REPRAP 1.0e6 		// Convert seconds to the units used by the machine (usually microseconds)
#define TIME_FROM_REPRAP 1.0e-6 	// Convert the units used by the machine (usually microseconds) to seconds

#define DEGREE_SYMBOL	"\xC2\xB0"	// Unicode degree-symbol as UTF8

#if SUPPORT_INKJET

// Inkjet (if any - no inkjet is flagged by INKJET_BITS negative)

const int8_t INKJET_BITS = 12;							// How many nozzles? Set to -1 to disable this feature
const int INKJET_FIRE_MICROSECONDS = 5;					// How long to fire a nozzle
const int INKJET_DELAY_MICROSECONDS = 800;				// How long to wait before the next bit

#endif

const float MAX_FEEDRATES[DRIVES] = DRIVES_(100.0, 100.0, 3.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0);						// mm/sec
const float ACCELERATIONS[DRIVES] = DRIVES_(500.0, 500.0, 20.0, 250.0, 250.0, 250.0, 250.0, 250.0, 250.0, 250.0);				// mm/sec^2
const float DRIVE_STEPS_PER_UNIT[DRIVES] = DRIVES_(87.4890, 87.4890, 4000.0, 420.0, 420.0, 420.0, 420.0, 420.0, 420.0, 420.0);	// steps/mm
const float INSTANT_DVS[DRIVES] = DRIVES_(15.0, 15.0, 0.2, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0);								// mm/sec

// AXES

const size_t X_AXIS = 0, Y_AXIS = 1, Z_AXIS = 2, E0_AXIS = 3;	// The indices of the Cartesian axes in drive arrays
const size_t A_AXIS = 0, B_AXIS = 1, C_AXIS = 2;				// The indices of the 3 tower motors of a delta printer in drive arrays

const float AXIS_MINIMA[MAX_AXES] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };			// mm
const float AXIS_MAXIMA[MAX_AXES] = { 230.0, 210.0, 200.0, 0.0, 0.0, 0.0 };		// mm

const float defaultPrintRadius = 50;							// mm
const float defaultDeltaHomedHeight = 200;						// mm

// Z PROBE

const float Z_PROBE_STOP_HEIGHT = 0.7;							// Millimetres
const unsigned int Z_PROBE_AVERAGE_READINGS = 8;				// We average this number of readings with IR on, and the same number with IR off
const int ZProbeTypeDelta = 7;									// Z probe type for experimental delta probe

#ifdef DUET_NG
const int Z_PROBE_AD_VALUE = 500;								// Default for the Z probe - should be overwritten by experiment
const uint32_t Z_PROBE_AXES = (1 << Z_AXIS);					// Axes for which the Z-probe is normally used
#else
const int Z_PROBE_AD_VALUE = 400;								// Default for the Z probe - should be overwritten by experiment
const uint32_t Z_PROBE_AXES = (1 << X_AXIS) | (1 << Z_AXIS);	// Axes for which the Z-probe is normally used
#endif

// HEATERS - The bed is assumed to be the at index 0

// Define the number of temperature readings we average for each thermistor. This should be a power of 2 and at least 4 ** AD_OVERSAMPLE_BITS.
// Keep THERMISTOR_AVERAGE_READINGS * NUM_HEATERS * 2ms no greater than HEAT_SAMPLE_TIME or the PIDs won't work well.
const unsigned int ThermistorAverageReadings = 32;

const uint32_t maxPidSpinDelay = 5000;			// Maximum elapsed time in milliseconds between successive temp samples by Pid::Spin() permitted for a temp sensor

const size_t DefaultBedHeater = 0;				// Index of the default bed heater
const size_t DefaultE0Heater = 1;				// Index of the default first extruder heater

/****************************************************************************************************/

// File handling

const size_t MAX_FILES = 10;					// Must be large enough to handle the max number of simultaneous web requests + files being printed
const size_t FILE_BUFFER_SIZE = 256;

/****************************************************************************************************/

enum class BoardType : uint8_t
{
	Auto = 0,
#ifdef DUET_NG
	DuetWiFi_10 = 1
#elif defined(__RADDS__)
	RADDS_15 = 1
#else
	Duet_06 = 1,
	Duet_07 = 2,
	Duet_085 = 3
#endif
};

enum class EndStopHit
{
  noStop = 0,		// no endstop hit
  lowHit = 1,		// low switch hit, or Z-probe in use and above threshold
  highHit = 2,		// high stop hit
  lowNear = 3		// approaching Z-probe threshold
};

// The values of the following enumeration must tally with the definitions for the M574 command
enum class EndStopType
{
	noEndStop = 0,
	lowEndStop = 1,
	highEndStop = 2
};

/***************************************************************************************************/

// Enumeration describing the reasons for a software reset.
// The spin state gets or'ed into this, so keep the lower 4 bits unused.
enum class SoftwareResetReason : uint16_t
{
	user = 0,						// M999 command
	erase = 0x10,					// special M999 command to erase firmware and reset
	NMI = 0x20,
	hardFault = 0x30,
	memManage = 0x40,
	busFault = 0x50,
	usageFault = 0x60,
	otherFault = 0x70,
	inAuxOutput = 0x0800,			// this bit is or'ed in if we were in aux output at the time
	stuckInSpin = 0x1000,			// we got stuck in a Spin() function for too long
	inLwipSpin = 0x2000,			// we got stuck in a call to LWIP for too long
	inUsbOutput = 0x4000			// this bit is or'ed in if we were in USB output at the time
};

// Enumeration to describe various tests we do in response to the M111 command
enum class DiagnosticTestType : int
{
	TestWatchdog = 1001,			// test that we get a watchdog reset if the tick interrupt stops
	TestSpinLockup = 1002,			// test that we get a software reset if a Spin() function takes too long
	TestSerialBlock = 1003,			// test what happens when we write a blocking message via debugPrintf()
	PrintMoves = 100,				// print summary of recent moves
#ifdef DUET_NG
	PrintExpanderStatus = 101,		// print DueXn expander status
#endif
};

// Enumeration to describe what we want to do with a logical pin
enum class PinAccess : int
{
	read,
	write,
	pwm,
	servo
};

/***************************************************************************************************************/

// Struct for holding Z probe parameters

struct ZProbeParameters
{
	int32_t adcValue;				// the target ADC value, after inversion if enabled
	float xOffset, yOffset;			// the offset of the probe relative to the print head
	float height;					// the nozzle height at which the target ADC value is returned
	float calibTemperature;			// the temperature at which we did the calibration
	float temperatureCoefficient;	// the variation of height with bed temperature
	float diveHeight;				// the dive height we use when probing
	float probeSpeed;				// the initial speed of probing
	float travelSpeed;				// the speed at which we travel to the probe point
	float recoveryTime;				// Z probe recovery time
	float extraParam;				// extra parameters used by some types of probe e.g. Delta probe
	bool invertReading;				// true if we need to invert the reading

	void Init(float h);
	float GetStopHeight(float temperature) const;
	bool WriteParameters(FileStore *f, unsigned int probeType) const;
};

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
		irqflags_t flags = cpu_irq_save();
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
	BadTemp = 1 << 0,
	BadMove = 1 << 1,
	OutputStarvation = 1 << 2,
	OutputStackOverflow = 1 << 3
};

// Different types of hardware-related input-output
enum class SerialSource
{
	USB,
	AUX,
	AUX2
};

struct AxisDriversConfig
{
	size_t numDrivers;								// Number of drivers assigned to each axis
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

	static void EnableWatchdog();
	static void KickWatchdog();						// kick the watchdog

	Compatibility Emulating() const;
	void SetEmulating(Compatibility c);
	void Diagnostics(MessageType mtype);
	void DiagnosticTest(int d);
	void ClassReport(float &lastTime);  			// Called on Spin() return to check everything's live.
	void LogError(ErrorCode e) { errorCodeBits |= (uint32_t)e; }

	void SoftwareReset(uint16_t reason);
	bool AtxPower() const;
	void SetAtxPower(bool on);
	void SetBoardType(BoardType bt);
	const char* GetElectronicsString() const;
	const char* GetBoardString() const;

	// Timing
  
	float Time();									// Returns elapsed seconds since some arbitrary time
	static uint32_t GetInterruptClocks();			// Get the interrupt clock count
	static bool ScheduleInterrupt(uint32_t tim);	// Schedule an interrupt at the specified clock count, or return true if it has passed already
	static void DisableStepInterrupt();				// Make sure we get no step interrupts
	void Tick();

	// Real-time clock

	bool IsDateTimeSet() const;						// Has the RTC been set yet?
	time_t GetDateTime() const;						// Retrieves the current RTC datetime and returns true if it's valid
	bool SetDateTime(time_t time);					// Sets the current RTC date and time or returns false on error
	bool SetDate(time_t date);						// Sets the current RTC date or returns false on error
	bool SetTime(time_t time);						// Sets the current RTC time or returns false on error

  	// Communications and data storage
  
	bool GCodeAvailable(const SerialSource source) const;
	char ReadFromSource(const SerialSource source);
	OutputBuffer *GetAuxGCodeReply();				// Returns cached G-Code reply for AUX devices and clears its reference
	void AppendAuxReply(OutputBuffer *buf);
	void AppendAuxReply(const char *msg);
    uint32_t GetAuxSeq() { return auxSeq; }
    bool HaveAux() const { return auxDetected; }	// Any device on the AUX line?
    void SetAuxDetected() { auxDetected = true; }

	void SetIPAddress(uint8_t ip[]);
	const uint8_t* GetIPAddress() const;
	void SetNetMask(uint8_t nm[]);
	const uint8_t* NetMask() const;
	void SetGateWay(uint8_t gw[]);
	const uint8_t* GateWay() const;
	void SetMACAddress(uint8_t mac[]);
	const uint8_t* MACAddress() const;
	void SetBaudRate(size_t chan, uint32_t br);
	uint32_t GetBaudRate(size_t chan) const;
	void SetCommsProperties(size_t chan, uint32_t cp);
	uint32_t GetCommsProperties(size_t chan) const;

	friend class FileStore;

	MassStorage* GetMassStorage() const;
	FileStore* GetFileStore(const char* directory, const char* fileName, bool write);
	const char* GetWebDir() const; 					// Where the html etc files are
	const char* GetGCodeDir() const; 				// Where the gcodes are
	const char* GetSysDir() const;  				// Where the system files are
	const char* GetMacroDir() const;				// Where the user-defined macros are
	const char* GetConfigFile() const; 				// Where the configuration is stored (in the system dir).
	const char* GetDefaultFile() const;				// Where the default configuration is stored (in the system dir).
	void InvalidateFiles(const FATFS *fs);			// Called to invalidate files when the SD card is removed
	bool AnyFileOpen(const FATFS *fs) const;		// Returns true if any files are open on the SD card

	// Message output (see MessageType for further details)

	void Message(const MessageType type, const char *message);
	void Message(const MessageType type, OutputBuffer *buffer);
	void MessageF(const MessageType type, const char *fmt, ...);
	void MessageF(const MessageType type, const char *fmt, va_list vargs);
	bool FlushMessages();							// Flush messages to USB and aux, returning true if there is more to send

	// Movement

	void EmergencyStop();
	void SetPhysicalDrives(size_t drive, uint32_t physicalDrives);
	uint32_t GetPhysicalDrives(size_t drive) const;
	void SetDirection(size_t drive, bool direction);
	void SetDirectionValue(size_t driver, bool dVal);
	bool GetDirectionValue(size_t driver) const;
	void SetEnableValue(size_t driver, bool eVal);
	bool GetEnableValue(size_t driver) const;
	void EnableDriver(size_t driver);
	void DisableDriver(size_t driver);
	void EnableDrive(size_t drive);
	void DisableDrive(size_t drive);
	void SetDriversIdle();
	void SetMotorCurrent(size_t drive, float current, bool isPercent);
	float GetMotorCurrent(size_t drive, bool isPercent) const;
	void SetIdleCurrentFactor(float f);
	float GetIdleCurrentFactor() const
		{ return idleCurrentFactor; }
	bool SetDriverMicrostepping(size_t driver, int microsteps, int mode);
	unsigned int GetDriverMicrostepping(size_t drive, bool& interpolation) const;
	bool SetMicrostepping(size_t drive, int microsteps, int mode);
	unsigned int GetMicrostepping(size_t drive, bool& interpolation) const;
	void SetDriverStepTiming(size_t driver, float microseconds);
	float GetDriverStepTiming(size_t driver) const;
	float DriveStepsPerUnit(size_t drive) const;
	const float *GetDriveStepsPerUnit() const
		{ return driveStepsPerUnit; }
	void SetDriveStepsPerUnit(size_t drive, float value);
	float Acceleration(size_t drive) const;
	const float* Accelerations() const;
	void SetAcceleration(size_t drive, float value);
	float MaxFeedrate(size_t drive) const;
	const float* MaxFeedrates() const;
	void SetMaxFeedrate(size_t drive, float value);
	float ConfiguredInstantDv(size_t drive) const;
	float ActualInstantDv(size_t drive) const;
	void SetInstantDv(size_t drive, float value);
	EndStopHit Stopped(size_t drive) const;
	float AxisMaximum(size_t axis) const;
	void SetAxisMaximum(size_t axis, float value);
	float AxisMinimum(size_t axis) const;
	void SetAxisMinimum(size_t axis, float value);
	float AxisTotalLength(size_t axis) const;
	bool IsAccessibleProbePoint(float x, float y) const;
	float GetPressureAdvance(size_t drive) const;
	void SetPressureAdvance(size_t extruder, float factor);

	void SetEndStopConfiguration(size_t axis, EndStopType endstopType, bool logicLevel)
	pre(axis < MAX_AXES);

	void GetEndStopConfiguration(size_t axis, EndStopType& endstopType, bool& logicLevel) const
	pre(axis < MAX_AXES);

	uint32_t GetAllEndstopStates() const;
	void SetAxisDriversConfig(size_t drive, const AxisDriversConfig& config);
	const AxisDriversConfig& GetAxisDriversConfig(size_t drive) const
		{ return axisDrivers[drive]; }
	void SetExtruderDriver(size_t extruder, uint8_t driver);
	uint8_t GetExtruderDriver(size_t extruder) const
		{ return extruderDrivers[extruder]; }
	uint32_t GetDriversBitmap(size_t drive) const			// get the bitmap of driver step bits for this axis or extruder
		{ return driveDriverBits[drive]; }
	static void StepDriversLow();							// set all step pins low
	static void StepDriversHigh(uint32_t driverMap);		// set the specified step pins high
	uint32_t GetSlowDrivers() const { return slowDrivers; }
	uint32_t GetSlowDriverClocks() const { return slowDriverStepPulseClocks; }

	// Z probe

	void SetZProbeDefaults();
	float ZProbeStopHeight();
	float GetZProbeDiveHeight() const;
	float GetZProbeStartingHeight();
	float GetZProbeTravelSpeed() const;
	int GetZProbeReading() const;
	EndStopHit GetZProbeResult() const;
	int GetZProbeSecondaryValues(int& v1, int& v2);
	void SetZProbeType(int iZ);
	int GetZProbeType() const { return zProbeType; }
	void SetZProbeAxes(uint32_t axes);
	uint32_t GetZProbeAxes() const { return zProbeAxes; }
	const ZProbeParameters& GetZProbeParameters(int32_t probeType) const;
	const ZProbeParameters& GetCurrentZProbeParameters() const { return GetZProbeParameters(zProbeType); }
	void SetZProbeParameters(int32_t probeType, const struct ZProbeParameters& params);
	bool MustHomeXYBeforeZ() const;
	bool WriteZProbeParameters(FileStore *f) const;

	// Ancilliary PWM

	void SetExtrusionAncilliaryPwmValue(float v);
	float GetExtrusionAncilliaryPwmValue() const;
	void SetExtrusionAncilliaryPwmFrequency(float f);
	float GetExtrusionAncilliaryPwmFrequency() const;
	bool SetExtrusionAncilliaryPwmPin(int logicalPin);
	int GetExtrusionAncilliaryPwmPin() const { return extrusionAncilliaryPwmLogicalPin; }
	void ExtrudeOn();
	void ExtrudeOff();

	// Heat and temperature

	float GetTemperature(size_t heater, TemperatureError& err) // Result is in degrees Celsius
	pre(heater < HEATERS);

	float GetZProbeTemperature();							// Get our best estimate of the Z probe temperature

	void SetHeater(size_t heater, float power)				// power is a fraction in [0,1]
	pre(heater < HEATERS);

	uint32_t HeatSampleInterval() const;
	void SetHeatSampleTime(float st);
	float GetHeatSampleTime() const;

	Thermistor& GetThermistor(size_t heater)
	pre(heater < HEATERS)
	{
		return thermistors[heater];
	}

	void SetThermistorNumber(size_t heater, size_t thermistor)
	pre(heater < HEATERS; thermistor < HEATERS);

	int GetThermistorNumber(size_t heater) const
	pre(heater < HEATERS);

	bool IsThermistorChannel(uint8_t heater) const
	pre(heater < HEATERS);

	bool IsThermocoupleChannel(uint8_t heater) const
	pre(heater < HEATERS);

	bool IsRtdChannel(uint8_t heater) const
	pre(heater < HEATERS);

	void UpdateConfiguredHeaters();
	bool AnyHeaterHot(uint16_t heaters, float t);			// called to see if we need to turn on the hot end fan

	// Fans
	Fan& GetFan(size_t fanNumber)							// Get access to the fan control object
	pre(fanNumber < NUM_FANS)
	{
		return fans[fanNumber];
	}

	float GetFanValue(size_t fan) const;					// Result is returned in percent
	void SetFanValue(size_t fan, float speed);				// Accepts values between 0..1 and 1..255
#ifndef DUET_NG
	void EnableSharedFan(bool enable);						// enable/disable the fan that shares its PWM pin with the last heater
#endif
	float GetFanRPM();

	// Flash operations
	void UpdateFirmware();
	bool CheckFirmwareUpdatePrerequisites();

	// AUX device
	void Beep(int freq, int ms);
	void SendMessage(const char* msg);

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
#ifndef __RADDS
	void GetMcuTemperatures(float& minT, float& currT, float& maxT) const;
#endif
	void SetMcuTemperatureAdjust(float v) { mcuTemperatureAdjust = v; }
	float GetMcuTemperatureAdjust() const { return mcuTemperatureAdjust; }

	// Low level port access
	static void SetPinMode(Pin p, PinMode mode);
	static bool ReadPin(Pin p);
	static void WriteDigital(Pin p, bool high);
	static void WriteAnalog(Pin p, float pwm, uint16_t frequency);

#ifdef DUET_NG
	// Power in voltage
	void GetPowerVoltages(float& minV, float& currV, float& maxV) const;
#endif

	// User I/O and servo support
	bool GetFirmwarePin(int logicalPin, PinAccess access, Pin& firmwarePin, bool& invert);

//-------------------------------------------------------------------------------------------------------
  
private:
	void ResetChannel(size_t chan);					// re-initialise a serial channel
	float AdcReadingToCpuTemperature(uint16_t reading) const;

#ifdef DUET_NG
	static float AdcReadingToPowerVoltage(uint16_t reading);
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
	// directly form/to flash memory.
	struct SoftwareResetData
	{
		static const uint16_t versionValue = 2;		// increment this whenever this struct changes
		static const uint16_t magicValue = 0x7D00 | versionValue;	// value we use to recognise that all the flash data has been written
		static const uint32_t nvAddress = 0;		// must be 4-byte aligned
		static const size_t numberOfSlots = 8;		// number of storage slots used to implement wear levelling

		uint16_t magic;								// the magic number, including the version
		uint16_t resetReason;						// this records why we did a software reset, for diagnostic purposes
		uint32_t neverUsedRam;						// the amount of never used RAM at the last abnormal software reset

		bool isVacant() const						// return true if this struct can be written without erasing it first
		{
			return magic == 0xFFFF && resetReason == 0xFFFF && neverUsedRam == 0xFFFFFFFF;
		}
	};

#ifdef DUET_NG
	static_assert(SoftwareResetData::numberOfSlots * sizeof(SoftwareResetData) <= 512, "Can't fit software reset data in SAM4E user signature area");
#else
	static_assert(SoftwareResetData::numberOfSlots * sizeof(SoftwareResetData) <= FLASH_DATA_LENGTH, "NVData too large");
#endif

	ZProbeParameters switchZProbeParameters;		// Z probe values for the switch Z-probe
	ZProbeParameters irZProbeParameters;			// Z probe values for the IR sensor
	ZProbeParameters alternateZProbeParameters;		// Z probe values for the alternate sensor
	int zProbeType;									// the type of Z probe we are currently using
	uint32_t zProbeAxes;							// Z probe is used for these axes (bitmap)
	byte ipAddress[4];
	byte netMask[4];
	byte gateWay[4];
	uint8_t macAddress[6];
	Compatibility compatibility;

	BoardType board;
#ifdef DUET_NG
	ExpansionBoardType expansionBoard;
#endif

	float lastTime;
	float longWait;
	float addToTime;
	unsigned long lastTimeCall;

	bool active;
	uint32_t errorCodeBits;

	void InitialiseInterrupts();
	void GetStackUsage(uint32_t* currentStack, uint32_t* maxStack, uint32_t* neverUsed) const;

	// DRIVES

	void SetDriverCurrent(size_t driver, float current, bool isPercent);
	void UpdateMotorCurrent(size_t driver);
	void SetDriverDirection(uint8_t driver, bool direction)
	pre(driver < DRIVES);

	static uint32_t CalcDriverBitmap(size_t driver);			// calculate the step bit for this driver

	volatile DriverStatus driverState[DRIVES];
	bool directions[DRIVES];
	bool enableValues[DRIVES];
	Pin endStopPins[DRIVES];
	float maxFeedrates[DRIVES];
	float accelerations[DRIVES];
	float driveStepsPerUnit[DRIVES];
	float instantDvs[DRIVES];
	float pressureAdvance[MaxExtruders];
	float motorCurrents[DRIVES];					// the normal motor current for each stepper driver
	float motorCurrentFraction[DRIVES];				// the percentages of normal motor current that each driver is set to
	AxisDriversConfig axisDrivers[MAX_AXES];		// the driver numbers assigned to each axis
	uint8_t extruderDrivers[MaxExtruders];			// the driver number assigned to each extruder
	uint32_t driveDriverBits[DRIVES];				// the bitmap of driver port bits for each axis or extruder
	uint32_t slowDriverStepPulseClocks;				// minimum high and low step pulse widths, in processor clocks
	uint32_t slowDrivers;							// bitmap of driver port bits that need extended step pulse timing
	float idleCurrentFactor;

#if defined(DUET_NG)
	size_t numTMC2660Drivers;						// the number of TMC2660 drivers we have, the remaining are simple enable/step/dir drivers
#elif !defined(__RADDS__)
	// Digipots
	MCP4461 mcpDuet;
	MCP4461 mcpExpansion;
	Pin potWipes[8];								// we have only 8 digipots, on the Duet 0.8.5 we use the DAC for the 9th
	float senseResistor;
	float maxStepperDigipotVoltage;
	float stepperDacVoltageRange, stepperDacVoltageOffset;
#endif

	// Z probe

	Pin zProbePin;
	Pin zProbeModulationPin;
	volatile ZProbeAveragingFilter zProbeOnFilter;					// Z probe readings we took with the IR turned on
	volatile ZProbeAveragingFilter zProbeOffFilter;					// Z probe readings we took with the IR turned off
	volatile ThermistorAveragingFilter thermistorFilters[HEATERS];	// bed and extruder thermistor readings

	float extrusionAncilliaryPwmValue;
	float extrusionAncilliaryPwmFrequency;
	int extrusionAncilliaryPwmLogicalPin;
	Pin extrusionAncilliaryPwmFirmwarePin;
	bool extrusionAncilliaryPwmInvert;

	void InitZProbe();
	uint16_t GetRawZProbeReading() const;
	void UpdateNetworkAddress(byte dst[4], const byte src[4]);

	// Axes and endstops

	float axisMaxima[MAX_AXES];
	float axisMinima[MAX_AXES];
	EndStopType endStopType[MAX_AXES];
	bool endStopLogicLevel[MAX_AXES];
  
	// Heaters - bed is assumed to be the first

	Pin tempSensePins[HEATERS];
	Pin heatOnPins[HEATERS];
	Thermistor thermistors[HEATERS];
	TemperatureSensor SpiTempSensors[MaxSpiTempSensors];
	Pin spiTempSenseCsPins[MaxSpiTempSensors];
	uint32_t configuredHeaters;										// bitmask of all heaters in use
	uint32_t heatSampleTicks;

	// Fans

	Fan fans[NUM_FANS];
	Pin coolingFanRpmPin;											// we currently support only one fan RPM input
	float lastRpmResetTime;
	void InitFans();
	bool FansHardwareInverted() const;

  	// Serial/USB

	uint32_t baudRates[NUM_SERIAL_CHANNELS];
	uint8_t commsParams[NUM_SERIAL_CHANNELS];
	OutputStack *auxOutput;
	OutputStack *aux2Output;
	OutputStack *usbOutput;
    bool auxDetected;							// Have we processed at least one G-Code from an AUX device?
	OutputBuffer *auxGCodeReply;				// G-Code reply for AUX devices (special one because it is actually encapsulated before sending)
	uint32_t auxSeq;							// Sequence number for AUX devices

	// Files

	MassStorage* massStorage;
	FileStore* files[MAX_FILES];
	bool fileStructureInitialised;
  
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

	unsigned int heaterTempChannels[HEATERS];
	AnalogChannelNumber thermistorAdcChannels[HEATERS];
	AnalogChannelNumber zProbeAdcChannel;
	uint8_t tickState;
	size_t currentHeater;
	int debugCode;

	// Hotend configuration
	float filamentWidth;
	float nozzleDiameter;

	// Temperature and power monitoring
#ifndef __RADDS__		// reading temperature on the RADDS messes up one of the heater pins, so don't do it
	AnalogChannelNumber temperatureAdcChannel;
	uint16_t currentMcuTemperature, highestMcuTemperature, lowestMcuTemperature;
	uint16_t mcuAlarmTemperature;
#endif
	float mcuTemperatureAdjust;

#ifdef DUET_NG
	AnalogChannelNumber vInMonitorAdcChannel;
	volatile uint16_t currentVin, highestVin, lowestVin;
	uint32_t numUnderVoltageEvents;
	volatile uint32_t numOverVoltageEvents;
	bool driversPowered;
#endif

	// RTC
	time_t realTime;									// the current date/time, or zero if never set
	uint32_t timeLastUpdatedMillis;						// the milliseconds counter when we last incremented the time

	// Direct pin manipulation
	int8_t logicalPinModes[HighestLogicalPin + 1];		// what mode each logical pin is set to - would ideally be class PinMode not int8_t
};

/*static*/ inline void Platform::SetPinMode(Pin pin, PinMode mode)
{
#ifdef DUET_NG
	if (pin >= ExpansionStart)
	{
		DuetExpansion::SetPinMode(pin - ExpansionStart, mode);
	}
	else
	{
		pinMode(pin, mode);
	}
#else
	pinMode(pin, mode);
#endif
}

/*static*/ inline bool Platform::ReadPin(Pin pin)
{
#ifdef DUET_NG
	if (pin >= ExpansionStart)
	{
		return DuetExpansion::DigitalRead(pin - ExpansionStart);
	}
	else
	{
		return digitalRead(pin);
	}
#else
	return digitalRead(pin);
#endif
}

/*static*/ inline void Platform::WriteDigital(Pin pin, bool high)
{
#ifdef DUET_NG
	if (pin >= ExpansionStart)
	{
		DuetExpansion::DigitalWrite(pin - ExpansionStart, high);
	}
	else
	{
		digitalWrite(pin, high);
	}
#else
	digitalWrite(pin, high);
#endif
}

/*static*/ inline void Platform::WriteAnalog(Pin pin, float pwm, uint16_t freq)
{
#ifdef DUET_NG
	if (pin >= ExpansionStart)
	{
		DuetExpansion::AnalogOut(pin - ExpansionStart, pwm);
	}
	else
	{
		AnalogOut(pin, pwm, freq);
	}
#else
	AnalogOut(pin, pwm, freq);
#endif
}

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

// Where the system files are

inline const char* Platform::GetSysDir() const
{
	return SYS_DIR;
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
	return DEFAULT_FILE;
}

//*****************************************************************************************************************

// Drive the RepRap machine - Movement

inline float Platform::DriveStepsPerUnit(size_t drive) const
{
	return driveStepsPerUnit[drive];
}

inline void Platform::SetDriveStepsPerUnit(size_t drive, float value)
{
	driveStepsPerUnit[drive] = value;
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
	accelerations[drive] = value;
}

inline float Platform::MaxFeedrate(size_t drive) const
{
	return maxFeedrates[drive];
}

inline const float* Platform::MaxFeedrates() const
{
	return maxFeedrates;
}

inline void Platform::SetMaxFeedrate(size_t drive, float value)
{
	maxFeedrates[drive] = value;
}

inline float Platform::ConfiguredInstantDv(size_t drive) const
{
	return instantDvs[drive];
}

inline void Platform::SetInstantDv(size_t drive, float value)
{
	instantDvs[drive] = value;
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
	const bool d = (direction == FORWARDS) ? directions[driver] : !directions[driver];
	digitalWrite(DIRECTION_PINS[driver], d);
}

inline void Platform::SetEnableValue(size_t driver, bool eVal)
{
	enableValues[driver] = eVal;
	DisableDriver(driver);				// disable the drive, because the enable polarity may have been wrong before
}

inline bool Platform::GetEnableValue(size_t driver) const
{
	return enableValues[driver];
}

inline float Platform::AxisMaximum(size_t axis) const
{
	return axisMaxima[axis];
}

inline void Platform::SetAxisMaximum(size_t axis, float value)
{
	axisMaxima[axis] = value;
}

inline float Platform::AxisMinimum(size_t axis) const
{
	return axisMinima[axis];
}

inline void Platform::SetAxisMinimum(size_t axis, float value)
{
	axisMinima[axis] = value;
}

inline float Platform::AxisTotalLength(size_t axis) const
{
	return axisMaxima[axis] - axisMinima[axis];
}

inline void Platform::SetExtrusionAncilliaryPwmValue(float v)
{
	extrusionAncilliaryPwmValue = v;
}

inline float Platform::GetExtrusionAncilliaryPwmValue() const
{
	return extrusionAncilliaryPwmValue;
}

inline void Platform::SetExtrusionAncilliaryPwmFrequency(float f)
{
	extrusionAncilliaryPwmFrequency = f;
}

inline float Platform::GetExtrusionAncilliaryPwmFrequency() const
{
	return extrusionAncilliaryPwmFrequency;
}

// For the Duet we use the fan output for this
// DC 2015-03-21: To allow users to control the cooling fan via gcodes generated by slic3r etc.,
// only turn the fan on/off if the extruder ancilliary PWM has been set nonzero.
// Caution: this is often called from an ISR, or with interrupts disabled!
inline void Platform::ExtrudeOn()
{
	if (extrusionAncilliaryPwmValue > 0.0)
	{
		WriteAnalog(extrusionAncilliaryPwmFirmwarePin,
					(extrusionAncilliaryPwmInvert) ? 1.0 - extrusionAncilliaryPwmValue : extrusionAncilliaryPwmValue, extrusionAncilliaryPwmFrequency);
	}
}

// DC 2015-03-21: To allow users to control the cooling fan via gcodes generated by slic3r etc.,
// only turn the fan on/off if the extruder ancilliary PWM has been set nonzero.
// Caution: this is often called from an ISR, or with interrupts disabled!
inline void Platform::ExtrudeOff()
{
	if (extrusionAncilliaryPwmValue > 0.0)
	{
		WriteAnalog(extrusionAncilliaryPwmFirmwarePin,
					(extrusionAncilliaryPwmInvert) ? 1.0 : 0.0, extrusionAncilliaryPwmFrequency);
	}
}

//********************************************************************************************************

// Drive the RepRap machine - Heat and temperature

inline uint32_t Platform::HeatSampleInterval() const
{
	return heatSampleTicks;
}

inline float Platform::GetHeatSampleTime() const
{
	return (float)heatSampleTicks/1000.0;
}
inline void Platform::SetHeatSampleTime(float st)
{
	if (st > 0)
	{
		heatSampleTicks = (uint32_t)(st * 1000.0);
	}
}

inline bool Platform::IsThermistorChannel(uint8_t heater) const
{
	return heaterTempChannels[heater] < HEATERS;
}

inline bool Platform::IsThermocoupleChannel(uint8_t heater) const
{
	return heaterTempChannels[heater] >= FirstThermocoupleChannel
			&& heaterTempChannels[heater] - FirstThermocoupleChannel < MaxSpiTempSensors;
}

inline bool Platform::IsRtdChannel(uint8_t heater) const
{
	return heaterTempChannels[heater] >= FirstRtdChannel
			&& heaterTempChannels[heater] - FirstRtdChannel < MaxSpiTempSensors;
}

inline const uint8_t* Platform::GetIPAddress() const
{
	return ipAddress;
}

inline const uint8_t* Platform::NetMask() const
{
	return netMask;
}

inline const uint8_t* Platform::GateWay() const
{
	return gateWay;
}

inline const uint8_t* Platform::MACAddress() const
{
	return macAddress;
}

inline float Platform::GetPressureAdvance(size_t extruder) const
{
	return (extruder < MaxExtruders) ? pressureAdvance[extruder] : 0.0;
}

inline void Platform::SetEndStopConfiguration(size_t axis, EndStopType esType, bool logicLevel)
{
	endStopType[axis] = esType;
	endStopLogicLevel[axis] = logicLevel;
}

inline void Platform::GetEndStopConfiguration(size_t axis, EndStopType& esType, bool& logicLevel) const
{
	esType = endStopType[axis];
	logicLevel = endStopLogicLevel[axis];
}

// Get the interrupt clock count
/*static*/ inline uint32_t Platform::GetInterruptClocks()
{
	return STEP_TC->TC_CHANNEL[STEP_TC_CHAN].TC_CV;
}

// This is called by the tick ISR to get the raw Z probe reading to feed to the filter
inline uint16_t Platform::GetRawZProbeReading() const
{
	switch (zProbeType)
	{
	case 4:
		{
			const bool b = ReadPin(endStopPins[E0_AXIS]);
			return (b) ? 4000 : 0;
		}

	case 5:
		return (ReadPin(zProbePin)) ? 4000 : 0;

	case 6:
		{
			const bool b = ReadPin(endStopPins[E0_AXIS + 1]);
			return (b) ? 4000 : 0;
		}

	default:
		return min<uint16_t>(AnalogInReadChannel(zProbeAdcChannel), 4000);
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

/*static*/ inline void Platform::EnableWatchdog()
{
	watchdogEnable(1000);
}

/*static*/ inline void Platform::KickWatchdog()
{
	watchdogReset();
}

inline float Platform::AdcReadingToCpuTemperature(uint16_t adcVal) const
{
	float voltage = (float)adcVal * (3.3/4096.0);
#ifdef DUET_NG
	return (voltage - 1.44) * (1000.0/4.7) + 27.0 + mcuTemperatureAdjust;			// accuracy at 27C is +/-13C
#else
	return (voltage - 0.8) * (1000.0/2.65) + 27.0 + mcuTemperatureAdjust;			// accuracy at 27C is +/-45C
#endif
}

#ifdef DUET_NG
inline float Platform::AdcReadingToPowerVoltage(uint16_t adcVal)
{
	return adcVal * (PowerFailVoltageRange/4096.0);
}
#endif

// *** These next two functions must use the same bit assignments in the drivers bitmap ***
// The bitmaps are organised like this:
// Duet WiFi:
//	All step pins are on port D, so the bitmap is just the map of bits in port D.
// Duet 0.6 and 0.8.5:
//	Step pins are PA0, PC7,9,11,14,25,29 and PD0,3.
//	The PC and PD bit numbers don't overlap, so we use their actual positions.
//	PA0 clashes with PD0, so we use bit 1 to represent PA0.
// RADDS:
//	To be done

// Calculate the step bit for a driver. This doesn't need to be fast.
/*static*/ inline uint32_t Platform::CalcDriverBitmap(size_t driver)
{
	const PinDescription& pinDesc = g_APinDescription[STEP_PINS[driver]];
#if defined(DUET_NG)
	return pinDesc.ulPin;
#elif defined(__RADDS__)
	return (pinDesc.pPort == PIOC) ? pinDesc.ulPin << 1 : pinDesc.ulPin;
#else
	return (pinDesc.pPort == PIOA) ? pinDesc.ulPin << 1 : pinDesc.ulPin;
#endif
}

// Set the specified step pins high and all other step pins low
// This needs to be as fast as possible, so we do a parallel write to the port(s).
// We rely on only those port bits that are step pins being set in the PIO_OWSR register of each port
/*static*/ inline void Platform::StepDriversHigh(uint32_t driverMap)
{
#if defined(DUET_NG)
	PIOD->PIO_ODSR = driverMap;				// on Duet WiFi all step pins are on port D
#elif defined(__RADDS__)
	PIOA->PIO_ODSR = driverMap;
	PIOB->PIO_ODSR = driverMap;
	PIOD->PIO_ODSR = driverMap;
	PIOC->PIO_ODSR = driverMap >> 1;		// do this last, it means the processor doesn't need to preserve the register containing driverMap
#else	// Duet
	PIOD->PIO_ODSR = driverMap;
	PIOC->PIO_ODSR = driverMap;
	PIOA->PIO_ODSR = driverMap >> 1;		// do this last, it means the processor doesn't need to preserve the register containing driverMap
#endif
}

// Set all step pins low
// This needs to be as fast as possible, so we do a parallel write to the port(s).
// We rely on only those port bits that are step pins being set in the PIO_OWSR register of each port
/*static*/ inline void Platform::StepDriversLow()
{
#if defined(DUET_NG)
	PIOD->PIO_ODSR = 0;						// on Duet WiFi all step pins are on port D
#elif defined(__RADDS__)
	PIOD->PIO_ODSR = 0;
	PIOC->PIO_ODSR = 0;
	PIOB->PIO_ODSR = 0;
	PIOA->PIO_ODSR = 0;
#else	// Duet
	PIOD->PIO_ODSR = 0;
	PIOC->PIO_ODSR = 0;
	PIOA->PIO_ODSR = 0;
#endif
}

//***************************************************************************************

#endif
