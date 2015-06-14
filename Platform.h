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

// What are we supposed to be running on

#define ELECTRONICS "Duet (+ Extension)"

// Language-specific includes

#include <cctype>
#include <cstring>
#include <malloc.h>
#include <cstdlib>
#include <climits>

// Platform-specific includes

#include "Arduino.h"
#include "SamNonDuePin.h"
#include "SD_HSMCI.h"
#include "MCP4461.h"

/**************************************************************************************************/

// Some numbers...

#define TIME_TO_REPRAP 1.0e6 	// Convert seconds to the units used by the machine (usually microseconds)
#define TIME_FROM_REPRAP 1.0e-6 // Convert the units used by the machine (usually microseconds) to seconds

/**************************************************************************************************/

// The physical capabilities of the machine

#define DRIVES 8 // The number of drives in the machine, including X, Y, and Z plus extruder drives
#define AXES 3 // The number of movement axes in the machine, usually just X, Y and Z. <= DRIVES
#define HEATERS 6 // The number of heaters in the machine; 0 is the heated bed even if there isn't one.

// The numbers of entries in each array must correspond with the values of DRIVES,
// AXES, or HEATERS. Set values to -1 to flag unavailability.

#define NUM_SERIAL_CHANNELS		(2)

// DRIVES

#define STEP_PINS {14, 25, 5, X2, 41, 39, X4, 49}
#define DIRECTION_PINS {15, 26, 4, X3, 35, 53, 51, 48}
#define FORWARDS true // What to send to go...
#define BACKWARDS (!FORWARDS) // ...in each direction
#define DIRECTIONS {BACKWARDS, FORWARDS, FORWARDS, FORWARDS, FORWARDS, FORWARDS, FORWARDS, FORWARDS} // What each axis needs to make it go forwards - defaults
#define ENABLE_PINS {29, 27, X1, X0, 37, X8, 50, 47}
#define ENABLE_DRIVE false // What to send to enable...
#define DISABLE_DRIVE true // ...and disable a drive
#define DISABLE_DRIVES {false, false, true, false, false, false, false, false} // Set true to disable a drive when it becomes idle
#define END_STOP_PINS {11, 28, 60, 31, 24, 46, 45, 44} //E Stops not currently used
// Indices for motor current digipots (if any)
// first 4 are for digipot 1,(on duet)
// second 4 for digipot 2(on expansion board)
// Full order is {1, 3, 2, 0, 1, 3, 2, 0}, only include as many as you have DRIVES defined
#define POT_WIPES {1, 3, 2, 0, 1, 3, 2, 0}
#define SENSE_RESISTOR 0.1 // Stepper motor current sense resistor
#define MAX_STEPPER_DIGIPOT_VOLTAGE ( 3.3*2.5/(2.7+2.5) ) // Stepper motor current reference voltage

#define Z_PROBE_AD_VALUE (400) // Default for the Z probe - should be overwritten by experiment
#define Z_PROBE_STOP_HEIGHT (0.7) // mm
#define Z_PROBE_PIN (10) 						// Analogue pin number
#define Z_PROBE_MOD_PIN (52)					// Digital pin number to turn the IR LED on (high) or off (low)
#define Z_PROBE_MOD_PIN07 (X25)					// Digital pin number to turn the IR LED on (high) or off (low) Duet V0.7 onwards
#define Z_PROBE_AXES {true, false, true}		// Axes for which the Z-probe is normally used
const unsigned int numZProbeReadingsAveraged = 8;	// we average this number of readings with IR on, and the same number with IR off

#define MAX_FEEDRATES {100.0, 100.0, 3.0, 20.0, 20.0, 20.0, 20.0, 20.0} // mm/sec
#define ACCELERATIONS {500.0, 500.0, 20.0, 250.0, 250.0, 250.0, 250.0, 250.0} // mm/sec^2
#define DRIVE_STEPS_PER_UNIT {87.4890, 87.4890, 4000.0, 420.0, 420.0, 420.0, 420.0, 420.0}
#define INSTANT_DVS {30.0, 30.0, 0.2, 2.0, 2.0, 2.0, 2.0, 2.0} // (mm/sec)

// AXES

#define AXIS_MAXIMA {230, 200, 200} 			// mm
#define AXIS_MINIMA {0, 0, -0.5}				// mm
const float defaultPrintRadius = 50;			// mm
const float defaultDeltaHomedHeight = 200;		// mm

#define HOME_FEEDRATES {50.0, 50.0, 100.0/60.0}	// mm/sec (dc42 increased Z because we slow down z-homing when approaching the target height)

const size_t X_AXIS = 0, Y_AXIS = 1, Z_AXIS = 2, E0_AXIS = 3;	// The indices of the Cartesian axes in drive arrays
const size_t A_AXIS = 0, B_AXIS = 1, C_AXIS = 2;	// The indices of the 3 tower motors of a delta printer in drive arrays

// HEATERS - The bed is assumed to be the at index 0

#define TEMP_SENSE_PINS {5, 4, 0, 7, 8, 9} // Analogue pin numbers
#define HEAT_ON_PINS {6, X5, X7, 7, 8, 9} //pin D38 is PWM capable but not an Arduino PWM pin
// Bed thermistor: http://uk.farnell.com/epcos/b57863s103f040/sensor-miniature-ntc-10k/dp/1299930?Ntt=129-9930
// Hot end thermistor: http://www.digikey.co.uk/product-search/en?x=20&y=11&KeyWords=480-3137-ND
const float defaultThermistorBetas[HEATERS] = {3988.0, 4138.0, 4138.0, 4138.0, 4138.0, 4138.0}; // Bed thermistor: B57861S104F40; Extruder thermistor: RS 198-961
const float defaultThermistorSeriesRs[HEATERS] = {1000, 1000, 1000, 1000, 1000, 1000}; // Ohms in series with the thermistors
const float defaultThermistor25RS[HEATERS] = {10000.0, 100000.0, 100000.0, 100000.0, 100000.0, 100000.0}; // Thermistor ohms at 25 C = 298.15 K

// Note on hot end PID parameters:
// The system is highly nonlinear because the heater power is limited to a maximum value and cannot go negative.
// If we try to run a traditional PID when there are large temperature errors, this causes the I-accumulator to go out of control,
// which causes a large amount of overshoot at lower temperatures. There are at least two ways of avoiding this:
// 1. Allow the PID to operate even with very large errors, but choose a very small I-term, just the right amount so that when heating up
//    from cold, the I-accumulator is approximately the value needed to maintain the correct power when the target temperature is reached.
//    This works well most of the time. However if the Duet board is reset when the extruder is hot and is then
//    commanded to heat up again before the extruder has cooled, the I-accumulator doesn't grow large enough, so the
//    temperature undershoots. The small value of the I-term then causes it to take a long time to reach the correct temperature.
// 2. Only allow the PID to operate when the temperature error is small enough for the PID to operate in the linear region.
//    So we set FULL_PID_BAND to a small value. It needs to be at least 15C because that is how much the temperature overshoots by
//    when we turn the heater off from full power at about 180C. We set the I-accumulator to zero when the PID is off, and use a
//    much larger I-term. So the I-accumulator grows from zero to the value needed to maintain the required temperature
//    much faster, but not so fast as to cause too much overshoot. This works well most of the time, except when we reduce
//    the temperature by more than FULL_PID_BAND. In this case we turn off the PID and the heater, clear the
//    I-accumulator, and wait for the temperature to drop before we turn the PID on again. The temperature has to undershoot by 10C
//    or more in order for the I-accumulator to build up again. However, dropping the temperature by more then 20C is not a normal
//    operation for a 3D printer, so we don't worry about this case.
// An improvement on method (2) would be to preset the I-accumulator to an estimate of the value needed to maintain the
// target temperature when we start using the PID (instead of clearing it to zero), and then reduce the I-term a little.
//
// Note: a negative P, I or D value means do not use PID for this heater, use bang-bang control instead.
// This allows us to switch between PID and bang-bang using the M301 and M304 commands.

// We use method 2 (see above)
const float defaultPidKis[HEATERS] = {5.0, 0.1, 0.1, 0.1, 0.1, 0.1}; 			// Integral PID constants
const float defaultPidKds[HEATERS] = {500.0, 100.0, 100.0, 100.0, 100.0, 100.0}; // Derivative PID constants
const float defaultPidKps[HEATERS] = {-1.0, 10.0, 10.0, 10.0, 10.0, 10.0};		// Proportional PID constants, negative values indicate use bang-bang instead of PID
const float defaultPidKts[HEATERS] = {2.7, 0.4, 0.4, 0.4, 0.4, 0.4};			// approximate PWM value needed to maintain temperature, per degC above room temperature
const float defaultPidKss[HEATERS] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};			// PWM scaling factor, to allow for variation in heater power and supply voltage
const float defaultFullBands[HEATERS] = {5.0, 30.0, 30.0, 30.0, 30.0, 30.0};	// errors larger than this cause heater to be on or off
const float defaultPidMins[HEATERS] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};			// minimum value of I-term
const float defaultPidMaxes[HEATERS] = {255, 180, 180, 180, 180, 180};			// maximum value of I-term, must be high enough to reach 245C for ABS printing

#define STANDBY_TEMPERATURES {ABS_ZERO, ABS_ZERO, ABS_ZERO, ABS_ZERO, ABS_ZERO, ABS_ZERO} // We specify one for the bed, though it's not needed
#define ACTIVE_TEMPERATURES {ABS_ZERO, ABS_ZERO, ABS_ZERO, ABS_ZERO, ABS_ZERO, ABS_ZERO}
#define COOLING_FAN_PIN X6 														// pin D34 is PWM capable but not an Arduino PWM pin - use X6 instead
#define COOLING_FAN_RPM_PIN 36													// pin PC4
#define COOLING_FAN_RPM_SAMPLE_TIME	2.0											// Time to wait before resetting the internal fan RPM stats
#define HEAT_ON 0 																// 0 for inverted heater (e.g. Duet v0.6) 1 for not (e.g. Duet v0.4)

// For the theory behind ADC oversampling, see http://www.atmel.com/Images/doc8003.pdf
const unsigned int adOversampleBits = 1;					// number of bits we oversample when reading temperatures

// Define the number of temperature readings we average for each thermistor. This should be a power of 2 and at least 4 ** adOversampleBits.
// Keep numThermistorReadingsAveraged * NUM_HEATERS * 2ms no greater than HEAT_SAMPLE_TIME or the PIDs won't work well.
const unsigned int numThermistorReadingsAveraged = (HEATERS > 3) ? 32 : 64;
const unsigned int adRangeReal = 4095;						// the ADC that measures temperatures gives an int this big as its max value
const unsigned int adRangeVirtual = ((adRangeReal + 1) << adOversampleBits) - 1;	// the max value we can get using oversampling
const unsigned int adDisconnectedReal = adRangeReal - 3;	// we consider an ADC reading at/above this value to indicate that the thermistor is disconnected
const unsigned int adDisconnectedVirtual = adDisconnectedReal << adOversampleBits;

#define HOT_BED 0 	// The index of the heated bed; set to -1 if there is no heated bed
#define E0_HEATER 1 //the index of the first extruder heater
#define E1_HEATER 2 //the index of the second extruder heater
#define E2_HEATER 3 //the index of the third extruder heater
#define E3_HEATER 4 //the index of the fourth extruder heater
#define E4_HEATER 5 //the index of the fifth extruder heater

/****************************************************************************************************/

// File handling

#define MAX_FILES (10)		// must be large enough to handle the max number of simultaneous web requests + file being printed
#define FILE_BUF_LEN (256)
#define WEB_DIR "0:/www/" 						// Place to find web files on the SD card
#define GCODE_DIR "0:/gcodes/" 					// Ditto - g-codes
#define SYS_DIR "0:/sys/" 						// Ditto - system files
#define TEMP_DIR "0:/tmp/" 						// Ditto - temporary files

#define MAC_ADDRESS {0xBE, 0xEF, 0xDE, 0xAD, 0xFE, 0xED}


/****************************************************************************************************/

// Miscellaneous...

typedef int8_t Pin;								// type used to represent a pin number, negative means no pin

const int atxPowerPin = 12;						// Arduino Due pin number that controls the ATX power on/off

const uint16_t lineInBufsize = 256;				// use a power of 2 for good performance
const uint16_t lineOutBufSize = 2048;			// ideally this should be large enough to hold the results of an M503 command,
												// but could be reduced if we ever need the memory
const size_t messageStringLength = 256;			// max length of a message chunk sent via Message or AppendMessage

/****************************************************************************************************/

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

// Input and output - these are ORed into a uint8_t
// By the Status() functions of the IO classes.

enum IOStatus
{
  nothing = 0,
  byteAvailable = 1,
  atEoF = 2,
  clientLive = 4,
  clientConnected = 8
};

// Enumeration describing the reasons for a software reset.
// The spin state gets or'ed into this, so keep the lower ~4 bits unused.
namespace SoftwareResetReason
{
	enum
	{
		user = 0,					// M999 command
		erase = 55,					// special M999 command to erase firmware and reset
		inAuxOutput = 0x0800,		// this bit is or'ed in if we were in aux output at the time
		stuckInSpin = 0x1000,		// we got stuck in a Spin() function for too long
		inLwipSpin = 0x2000,		// we got stuck in a call to LWIP for too long
		inUsbOutput = 0x4000		// this bit is or'ed in if we were in USB output at the time
	};
}

// Enumeration to describe various tests we do in response to the M111 command
namespace DiagnosticTest
{
	enum
	{
		TestWatchdog = 1001,			// test that we get a watchdog reset if the tick interrupt stops
		TestSpinLockup = 1002,			// test that we get a software reset if a Spin() function takes too long
		TestSerialBlock = 1003			// test what happens when we write a blocking message via debugPrintf()
	};
}

// This class handles serial I/O - typically via USB

class Line
{
public:

	uint8_t Status() const;				// Returns OR of IOStatus
	int Read(char& b);
	void Write(char b, bool block = false);
	void Write(const char* s, bool block = false);
	void Flush();

friend class Platform;
friend class RepRap;

protected:

	Line(Stream& p_iface);
	void Init();
	void Spin();
	void InjectString(char* string);
	unsigned int GetOutputColumn() const { return outputColumn; }

private:
	void TryFlushOutput();

	// Although the sam3x usb interface code already has a 512-byte buffer, adding this extra 256-byte buffer
	// increases the speed of uploading to the SD card by 10%
	char inBuffer[lineInBufsize];
	char outBuffer[lineOutBufSize];
	uint16_t inputGetIndex;
	uint16_t inputNumChars;
	uint16_t outputGetIndex;
	uint16_t outputNumChars;

	uint8_t inWrite;
	bool ignoringOutputLine;
	unsigned int outputColumn;
	Stream& iface;
};

// Info returned by FindFirst/FindNext calls
class FileInfo
{
public:

	bool isDirectory;
	unsigned long size;
	uint8_t day;
	uint8_t month;
	uint16_t year;
	char fileName[MaxFilenameLength];
};

class MassStorage
{
public:

  bool FindFirst(const char *directory, FileInfo &file_info);
  bool FindNext(FileInfo &file_info);
  const char* GetMonthName(const uint8_t month);
  const char* CombineName(const char* directory, const char* fileName);
  bool Delete(const char* directory, const char* fileName);
  bool MakeDirectory(const char *parentDir, const char *dirName);
  bool MakeDirectory(const char *directory);
  bool Rename(const char *oldFilename, const char *newFilename);
  bool FileExists(const char *file) const;
  bool PathExists(const char *path) const;
  bool PathExists(const char* directory, const char* subDirectory);

friend class Platform;

protected:

  MassStorage(Platform* p);
  void Init();

private:

  Platform* platform;
  FATFS fileSystem;
  DIR findDir;
  char combinedName[MaxFilenameLength + 1];
};

// This class handles input from, and output to, files.

typedef uint32_t FilePosition;
const FilePosition noFilePosition = 0xFFFFFFFF;

class FileStore
{
public:

	uint8_t Status();								// Returns OR of IOStatus
	bool Read(char& b);								// Read 1 byte
	int Read(char* buf, unsigned int nBytes);		// Read a block of nBytes length
	bool Write(char b);								// Write 1 byte
	bool Write(const char *s, unsigned int len);	// Write a block of len bytes
	bool Write(const char* s);						// Write a string
	bool Close();									// Shut the file and tidy up
	bool Seek(FilePosition pos);					// Jump to pos in the file
	FilePosition GetPosition() const;				// Return the current position in the file, assuming we are reading the file
#if 0	// not currently used
	bool GoToEnd();									// Position the file at the end (so you can write on the end).
#endif
	FilePosition Length() const;					// File size in bytes
	float FractionRead() const;						// How far in we are
	void Duplicate();								// Create a second reference to this file
	bool Flush();									// Write remaining buffer data
	static float GetAndClearLongestWriteTime();		// Return the longest time it took to write a block to a file, in milliseconds

friend class Platform;

protected:

	FileStore(Platform* p);
	void Init();
    bool Open(const char* directory, const char* fileName, bool write);
        
private:

    bool inUse;
    byte buf[FILE_BUF_LEN];
    unsigned int bufferPointer;
  
	bool ReadBuffer();
	bool WriteBuffer();
	bool InternalWriteBlock(const char *s, unsigned int len);

	FIL file;
	Platform* platform;
	bool writing;
	unsigned int lastBufferEntry;
	unsigned int openCount;

	static uint32_t longestWriteTime;
};


/***************************************************************************************************************/

// Struct for holding Z probe parameters

struct ZProbeParameters
{
	int adcValue;					// the target ADC value
	float xOffset, yOffset;			// the offset of the probe relative to the print head
	float height;					// the nozzle height at which the target ADC value is returned
	float calibTemperature;			// the temperature at which we did the calibration
	float temperatureCoefficient;	// the variation of height with bed temperature
	float diveHeight;				// the dive height we use when probing
	float param1, param2;			// extra parameters used by some types of probe e.g. Delta probe

	void Init(float h)
	{
		adcValue = Z_PROBE_AD_VALUE;
		xOffset = yOffset = 0.0;
		height = h;
		calibTemperature = 20.0;
		temperatureCoefficient = 0.0;	// no default temperature correction
		diveHeight = DefaultZDive;
		param1 = param2 = 0.0;
	}

	float GetStopHeight(float temperature) const
	{
		return ((temperature - calibTemperature) * temperatureCoefficient) + height;
	}

	bool operator==(const ZProbeParameters& other) const
	{
		return adcValue == other.adcValue
				&& height == other.height
				&& xOffset == other.xOffset
				&& yOffset == other.yOffset
				&& calibTemperature == other.calibTemperature
				&& temperatureCoefficient == other.temperatureCoefficient
				&& diveHeight == other.diveHeight
				&& param1 == other.param1
				&& param2 == other.param2;
	}

	bool operator!=(const ZProbeParameters& other) const
	{
		return !operator==(other);
	}
};

class PidParameters
{
	// If you add any more variables to this class, don't forget to change the definition of operator== in Platform.cpp!
private:
	float thermistorBeta, thermistorInfR;				// private because these must be changed together

public:
	float kI, kD, kP, kT, kS;
	float fullBand, pidMin, pidMax;
	float thermistorSeriesR;
	float adcLowOffset, adcHighOffset;

	float GetBeta() const { return thermistorBeta; }
	float GetRInf() const { return thermistorInfR; }

	bool UsePID() const;
	float GetThermistorR25() const;
	void SetThermistorR25AndBeta(float r25, float beta);

	bool operator==(const PidParameters& other) const;
	bool operator!=(const PidParameters& other) const
	{
		return !operator==(other);
	}
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

typedef AveragingFilter<numThermistorReadingsAveraged> ThermistorAveragingFilter;
typedef AveragingFilter<numZProbeReadingsAveraged> ZProbeAveragingFilter;

// Enumeration of error condition bits
enum ErrorCode
{
	ErrorBadTemp = 1 << 0
};

// The main class that defines the RepRap machine for the benefit of the other classes

class Platform
{   
public:
	// Enumeration to describe the status of a drive
	enum class DriveStatus : uint8_t { disabled, idle, enabled };
  
  Platform();
  
//-------------------------------------------------------------------------------------------------------------

// These are the functions that form the interface between Platform and the rest of the firmware.

  void Init(); // Set the machine up after a restart.  If called subsequently this should set the machine up as if
               // it has just been restarted; it can do this by executing an actual restart if you like, but beware the 
               // loop of death...
  void Spin(); // This gets called in the main loop and should do any housekeeping needed
  void Exit(); // Shut down tidily. Calling Init after calling this should reset to the beginning
  Compatibility Emulating() const;
  void SetEmulating(Compatibility c);
  void Diagnostics();
  void DiagnosticTest(int d);
  void ClassReport(float &lastTime);  			// Called on Spin() return to check everything's live.
  void RecordError(ErrorCode ec) { errorCodeBits |= ec; }
  void SoftwareReset(uint16_t reason);
  bool AtxPower() const;
  void SetAtxPower(bool on);

  // Timing
  
  float Time();									// Returns elapsed seconds since some arbitrary time
  static uint32_t GetInterruptClocks();			// Get the interrupt clock count
  static bool ScheduleInterrupt(uint32_t tim);	// Schedule an interrupt at the specified clock count, or return true if it has passed already
  void Tick();
  
  // Communications and data storage
  
  Line* GetLine() const;
  Line* GetAux() const;
  void SetIPAddress(uint8_t ip[]);
  const uint8_t* IPAddress() const;
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
  
  MassStorage* GetMassStorage();
  FileStore* GetFileStore(const char* directory, const char* fileName, bool write);
  const char* GetWebDir() const; 	// Where the htm etc files are
  const char* GetGCodeDir() const; 	// Where the gcodes are
  const char* GetSysDir() const;  	// Where the system files are
  const char* GetTempDir() const;	// Where temporary files are
  const char* GetConfigFile() const; // Where the configuration is stored (in the system dir).
  const char* GetDefaultFile() const;	// Where the default configuration is stored (in the system dir).
  
  void Message(char type, const char* message, ...);		// Send a message.  Messages may simply flash an LED, or,
  	  	  	  	  	  	  	  	  	  // say, display the messages on an LCD. This may also transmit the messages to the host.
  void Message(char type, const char* fmt, va_list vargs);
  void Message(char type, const StringRef& message);
  void AppendMessage(char type, const char* message, ...);	// Send a message.  Messages may simply flash an LED, or,
  	  	  	  	  	  	  	  	  	  // say, display the messages on an LCD. This may also transmit the messages to the host.
  void AppendMessage(char type, const StringRef& message);
  void PushMessageIndent();
  void PopMessageIndent();
  
  // Movement
  
  void EmergencyStop();
  void SetDirection(size_t drive, bool direction);
  void SetDirectionValue(size_t drive, bool dVal);
  bool GetDirectionValue(size_t drive) const;
  void StepHigh(size_t drive);
  void StepLow(size_t drive);
  void EnableDrive(size_t drive);
  void DisableDrive(size_t drive);
  void SetDrivesIdle();
  void SetMotorCurrent(size_t drive, float current);
  float MotorCurrent(size_t drive) const;
  void SetIdleCurrentFactor(float f);
  float GetIdleCurrentFactor() const { return idleCurrentFactor; }
  float DriveStepsPerUnit(size_t drive) const;
  const float *GetDriveStepsPerUnit() const { return driveStepsPerUnit; }
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
  float HomeFeedRate(size_t axis) const;
  void SetHomeFeedRate(size_t axis, float value);
  EndStopHit Stopped(size_t drive) const;
  float AxisMaximum(size_t axis) const;
  void SetAxisMaximum(size_t axis, float value);
  float AxisMinimum(size_t axis) const;
  void SetAxisMinimum(size_t axis, float value);
  float AxisTotalLength(size_t axis) const;
  float GetElasticComp(size_t drive) const;
  void SetElasticComp(size_t drive, float factor);
  void SetEndStopConfiguration(size_t axis, EndStopType endstopType, bool logicLevel);
  void GetEndStopConfiguration(size_t axis, EndStopType& endstopType, bool& logicLevel) const;

  // Z probe

  float ZProbeStopHeight() const;
  float GetZProbeDiveHeight() const;
  void SetZProbeDiveHeight(float h);
  int ZProbe() const;
  EndStopHit GetZProbeResult() const;
  int GetZProbeSecondaryValues(int& v1, int& v2);
  void SetZProbeType(int iZ);
  int GetZProbeChannel() const;
  void SetZProbeChannel(int chan);
  int GetZProbeType() const;
  void SetZProbeAxes(const bool axes[AXES]);
  void GetZProbeAxes(bool (&axes)[AXES]);
  const ZProbeParameters& GetZProbeParameters() const;
  bool SetZProbeParameters(const struct ZProbeParameters& params);
  bool MustHomeXYBeforeZ() const;
  
  void SetExtrusionAncilliaryPWM(float v);
  float GetExtrusionAncilliaryPWM() const;
  void ExtrudeOn();
  void ExtrudeOff();

  // Mixing support

//  void SetMixingDrives(int);
//  int GetMixingDrives();

  size_t SlowestDrive() const;

  // Heat and temperature
  
  float GetTemperature(size_t heater) const; // Result is in degrees Celsius
  void SetHeater(size_t heater, float power); // power is a fraction in [0,1]
  float HeatSampleTime() const;
  void SetHeatSampleTime(float st);
  float GetFanValue() const;						// Result is returned in per cent
  void SetFanValue(float speed);					// Accepts values between 0..1 and 1..255
  float GetFanRPM();
  void SetPidParameters(size_t heater, const PidParameters& params);
  const PidParameters& GetPidParameters(size_t heater) const;
  float TimeToHot() const;
  void SetTimeToHot(float t);
  void SetThermistorNumber(size_t heater, size_t thermistor);
  int GetThermistorNumber(size_t heater) const;

  // Flash operations
  void ResetNvData();
  void ReadNvData();
  void WriteNvData();

  void SetAutoSave(bool enabled);

  // AUX device
  void Beep(int freq, int ms);

  // Hotend configuration
  float GetFilamentWidth() const;
  void SetFilamentWidth(float width);
  float GetNozzleDiameter() const;
  void SetNozzleDiameter(float diameter);

//-------------------------------------------------------------------------------------------------------
  
private:
  void ResetChannel(size_t chan);					// re-initialise a serial channel
  
  // These are the structures used to hold out non-volatile data.
  // The SAM3X doesn't have EEPROM so we save the data to flash. This unfortunately means that it gets cleared
  // every time we reprogram the firmware. So there is no need to cater for writing one version of this
  // struct and reading back another.

  struct SoftwareResetData
  {
	  static const uint16_t magicValue = 0x59B2;	// value we use to recognise that all the flash data has been written
	  static const uint32_t nvAddress = 0;			// address in flash where we store the nonvolatile data

	  uint16_t magic;
	  uint16_t resetReason;							// this records why we did a software reset, for diagnostic purposes
	  size_t neverUsedRam;							// the amount of never used RAM at the last abnormal software reset
	  char lastMessage[256];						// the last known message before a software reset occurred
  };

  struct FlashData
  {
	  static const uint16_t magicValue = 0xA436;	// value we use to recognise that the flash data has been written
	  static const uint32_t nvAddress = SoftwareResetData::nvAddress + sizeof(struct SoftwareResetData);

	  uint16_t magic;

	  // The remaining data could alternatively be saved to SD card.
	  // Note however that if we save them as G codes, we need to provide a way of saving IR and ultrasonic G31 parameters separately.
	  ZProbeParameters switchZProbeParameters;		// Z probe values for the endstop switch
	  ZProbeParameters irZProbeParameters;			// Z probe values for the IR sensor
	  ZProbeParameters alternateZProbeParameters;	// Z probe values for the alternate sensor
	  int zProbeType;								// the type of Z probe we are currently using
	  Pin zProbeModulationPin;						// which pin is used for Z probe modulation
	  bool zProbeAxes[AXES];						// Z probe is used for these axes
	  PidParameters pidParams[HEATERS];
	  byte ipAddress[4];
	  byte netMask[4];
	  byte gateWay[4];
	  uint8_t macAddress[6];
	  Compatibility compatibility;
  };

  FlashData nvData;
  bool autoSaveEnabled;

  float lastTime;
  float longWait;
  float addToTime;
  unsigned long lastTimeCall;
  
  bool active;
  uint32_t errorCodeBits;
  
  void InitialiseInterrupts();
  void GetStackUsage(size_t* currentStack, size_t* maxStack, size_t* neverUsed) const;

// DRIVES

  void SetSlowestDrive();
  void UpdateMotorCurrent(size_t drive);

  Pin stepPins[DRIVES];
  Pin directionPins[DRIVES];
  Pin enablePins[DRIVES];
//  bool disableDrives[DRIVES];			// not currently used
  volatile DriveStatus driveState[DRIVES];
  bool directions[DRIVES];
  Pin endStopPins[DRIVES];
  float maxFeedrates[DRIVES];  
  float accelerations[DRIVES];
  float driveStepsPerUnit[DRIVES];
  float instantDvs[DRIVES];
  float elasticComp[DRIVES];
  float motorCurrents[DRIVES];
  float idleCurrentFactor;

  MCP4461 mcpDuet;
  MCP4461 mcpExpansion;
  size_t slowestDrive;

  Pin potWipes[DRIVES];
  float senseResistor;
  float maxStepperDigipotVoltage;

// Z probe

  Pin zProbePin;

  volatile ZProbeAveragingFilter zProbeOnFilter;					// Z probe readings we took with the IR turned on
  volatile ZProbeAveragingFilter zProbeOffFilter;					// Z probe readings we took with the IR turned off
  volatile ThermistorAveragingFilter thermistorFilters[HEATERS];	// bed and extruder thermistor readings

  float extrusionAncilliaryPWM;

  void InitZProbe();
  uint16_t GetRawZProbeReading() const;
  void UpdateNetworkAddress(byte dst[4], const byte src[4]);

  // AXES

  float axisMaxima[AXES];
  float axisMinima[AXES];
  float homeFeedrates[AXES];
  EndStopType endStopType[AXES+1];
  bool endStopLogicLevel[AXES+1];
  
// HEATERS - Bed is assumed to be the first

  int GetRawTemperature(byte heater) const;

  Pin tempSensePins[HEATERS];
  Pin heatOnPins[HEATERS];
  float heatSampleTime;
  float standbyTemperatures[HEATERS];
  float activeTemperatures[HEATERS];
  float coolingFanValue;
  Pin coolingFanPin;
  Pin coolingFanRpmPin;
  float timeToHot;
  float lastRpmResetTime;

// Serial/USB

  Line* line;
  Line* aux;
  uint32_t baudRates[NUM_SERIAL_CHANNELS];
  uint8_t commsParams[NUM_SERIAL_CHANNELS];
  uint8_t messageIndent;

// Files

  MassStorage* massStorage;
  FileStore* files[MAX_FILES];
  bool fileStructureInitialised;
  const char* webDir;
  const char* gcodeDir;
  const char* sysDir;
  const char* tempDir;
  const char* configFile;
  const char* defaultFile;
  
// Data used by the tick interrupt handler

  adc_channel_num_t heaterAdcChannels[HEATERS];
  adc_channel_num_t zProbeAdcChannel;
  uint32_t thermistorOverheatSums[HEATERS];
  uint8_t tickState;
  uint8_t currentHeater;
  int debugCode;

  static uint16_t GetAdcReading(adc_channel_num_t chan);
  static void StartAdcConversion(adc_channel_num_t chan);
  static adc_channel_num_t PinToAdcChannel(int pin);

  char messageStringBuffer[messageStringLength];
  StringRef messageString;

  // Hotend configuration
  float filamentWidth;
  float nozzleDiameter;
};

// Small class to hold an open file and data relating to it.
// This is designed so that files are never left open and we never duplicate a file reference.
class FileData
{
public:
	FileData() : f(NULL) {}

	// Set this to refer to a newly-opened file
	void Set(FileStore* pfile)
	{
		Close();	// close any existing file
		f = pfile;
	}

	bool IsLive() const { return f != NULL; }

	bool Close()
	{
		if (f != NULL)
		{
			bool ok = f->Close();
			f = NULL;
			return ok;
		}
		return false;
	}

	bool Read(char& b)
	{
		return f->Read(b);
	}

	bool Write(char b)
	{
		return f->Write(b);
	}

	bool Write(const char *s, unsigned int len)
	{
		return f->Write(s, len);
	}

	bool Flush()
	{
		return f->Flush();
	}

	FilePosition GetPosition() const
	{
		return f->GetPosition();
	}

	bool Seek(FilePosition position)
	{
		return f->Seek(position);
	}

	float FractionRead() const
	{
		return (f == NULL ? -1.0 : f->FractionRead());
	}

	FilePosition Length() const
	{
		return f->Length();
	}

	// Assignment operator
	void CopyFrom(const FileData& other)
	{
		Close();
		f = other.f;
		if (f != NULL)
		{
			f->Duplicate();
		}
	}

	// Move operator
	void MoveFrom(FileData& other)
	{
		Close();
		f = other.f;
		other.Init();
	}

private:
	FileStore *f;

	void Init()
	{
		f = NULL;
	}

	// Private assignment operator to prevent us assigning these objects
	FileData& operator=(const FileData&);

	// Private copy constructor to prevent us copying these objects
	FileData(const FileData&);
};

// Where the htm etc files are

inline const char* Platform::GetWebDir() const
{
  return webDir;
}

// Where the gcodes are

inline const char* Platform::GetGCodeDir() const
{
  return gcodeDir;
}

// Where the system files are

inline const char* Platform::GetSysDir() const
{
  return sysDir;
}

// Where the temporary files are

inline const char* Platform::GetTempDir() const
{
  return tempDir;
}


inline const char* Platform::GetConfigFile() const
{
  return configFile;
}

inline const char* Platform::GetDefaultFile() const
{
  return defaultFile;
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
	SetSlowestDrive();
}

inline size_t Platform::SlowestDrive() const
{
	return slowestDrive;
}

#if 0	// not used
inline const float* Platform::InstantDvs() const
{
	return instantDvs;
}
#endif

inline void Platform::SetDirectionValue(size_t drive, bool dVal)
{
	directions[drive] = dVal;
}

inline bool Platform::GetDirectionValue(size_t drive) const
{
	return directions[drive];
}

inline float Platform::HomeFeedRate(size_t axis) const
{
  return homeFeedrates[axis];
}

inline void Platform::SetHomeFeedRate(size_t axis, float value)
{
   homeFeedrates[axis] = value;
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

// The A4988 requires 1us minimum pulse width, so we make separate StepHigh and StepLow calls so that we don't waste this time
inline void Platform::StepHigh(size_t drive)
{
	const int pin = stepPins[drive];
	if (pin >= 0)
	{
		digitalWriteNonDue(pin, 1);
	}
}

inline void Platform::StepLow(size_t drive)
{
	const int pin = stepPins[drive];
	if (pin >= 0)
	{
		digitalWriteNonDue(pin, 0);
	}
}

inline void Platform::SetExtrusionAncilliaryPWM(float v)
{
	extrusionAncilliaryPWM = v;
}

inline float Platform::GetExtrusionAncilliaryPWM() const
{
	return extrusionAncilliaryPWM;
}

// For the Duet we use the fan output for this
// DC 2015-03-21: To allow users to control the cooling fan via gcodes generated by slic3r etc.,
// only turn the fan on/off if the extruder ancilliary PWM has been set nonzero.
// Caution: this is often called from an ISR, or with interrupts disabled!
inline void Platform::ExtrudeOn()
{
	if (extrusionAncilliaryPWM > 0.0)
	{
		SetFanValue(extrusionAncilliaryPWM);
	}
}

// DC 2015-03-21: To allow users to control the cooling fan via gcodes generated by slic3r etc.,
// only turn the fan on/off if the extruder ancilliary PWM has been set nonzero.
// Caution: this is often called from an ISR, or with interrupts disabled!
inline void Platform::ExtrudeOff()
{
	if (extrusionAncilliaryPWM > 0.0)
	{
		SetFanValue(0.0);
	}
}

//********************************************************************************************************

// Drive the RepRap machine - Heat and temperature

inline int Platform::GetRawTemperature(byte heater) const
{
  return (heater < HEATERS)
		  ? thermistorFilters[heater].GetSum()/(numThermistorReadingsAveraged >> adOversampleBits)
		  : 0;
}

inline float Platform::HeatSampleTime() const
{
  return heatSampleTime;
}

inline void Platform::SetHeatSampleTime(float st)
{
	heatSampleTime = st;
}

inline float Platform::TimeToHot() const
{
	return timeToHot;
}

inline void Platform::SetTimeToHot(float t)
{
	timeToHot = t;
}

inline const uint8_t* Platform::IPAddress() const
{
	return nvData.ipAddress;
}

inline const uint8_t* Platform::NetMask() const
{
	return nvData.netMask;
}

inline const uint8_t* Platform::GateWay() const
{
	return nvData.gateWay;
}

inline void Platform::SetMACAddress(uint8_t mac[])
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

inline const uint8_t* Platform::MACAddress() const
{
	return nvData.macAddress;
}

inline Line* Platform::GetLine() const
{
	return line;
}

inline Line* Platform::GetAux() const
{
	return aux;
}

inline void Platform::PushMessageIndent()
{
	messageIndent += 2;
}

inline void Platform::PopMessageIndent()
{
	messageIndent -= 2;
}

inline float Platform::GetElasticComp(size_t drive) const
{
	return (drive < DRIVES) ? elasticComp[drive] : 0.0;
}

inline void Platform::SetEndStopConfiguration(size_t axis, EndStopType esType, bool logicLevel)
//pre(axis < AXES)
{
	endStopType[axis] = esType;
	endStopLogicLevel[axis] = logicLevel;
}

inline void Platform::GetEndStopConfiguration(size_t axis, EndStopType& esType, bool& logicLevel) const
//pre(axis < AXES)
{
	esType = endStopType[axis];
	logicLevel = endStopLogicLevel[axis];
}

// Get the interrupt clock count
/*static*/ inline uint32_t Platform::GetInterruptClocks()
{
	//return TC_ReadCV(TC1, 0);
	// sadly, the Arduino IDE does not provide the inlined version of TC_ReadCV, so use the following instead...
	return TC1 ->TC_CHANNEL[0].TC_CV;
}

// This is called by the tick ISR to get the raw Z probe reading to feed to the filter
inline uint16_t Platform::GetRawZProbeReading() const
{
	if (nvData.zProbeType >= 4)
	{
		bool b = (bool)digitalRead(endStopPins[E0_AXIS]);
		if (!endStopLogicLevel[AXES])
		{
			b = !b;
		}
		return (b) ? 4000 : 0;
	}
	else
	{
		return GetAdcReading(zProbeAdcChannel);
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

//***************************************************************************************

#endif
