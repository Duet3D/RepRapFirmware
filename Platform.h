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

#define ELECTRONICS "Duet + Extension"

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

#define STRING_LENGTH 1029
#define SHORT_STRING_LENGTH 40
#define TIME_TO_REPRAP 1.0e6 	// Convert seconds to the units used by the machine (usually microseconds)
#define TIME_FROM_REPRAP 1.0e-6 // Convert the units used by the machine (usually microseconds) to seconds

/**************************************************************************************************/

// The physical capabilities of the machine

#define DRIVES 8 // The number of drives in the machine, including X, Y, and Z plus extruder drives
#define AXES 3 // The number of movement axes in the machine, usually just X, Y and Z. <= DRIVES
#define HEATERS 6 // The number of heaters in the machine; 0 is the heated bed even if there isn't one.

// The numbers of entries in each array must correspond with the values of DRIVES,
// AXES, or HEATERS. Set values to -1 to flag unavailability.

// DRIVES

#define STEP_PINS {14, 25, 5, X2, 41, 39, X4, 49}
#define DIRECTION_PINS {15, 26, 4, X3, 35, 53, 51, 48}
#define FORWARDS true // What to send to go...
#define BACKWARDS false // ...in each direction
#define ENABLE_PINS {29, 27, X1, X0, 37, X8, 50, 47}
#define ENABLE false // What to send to enable...
#define DISABLE true // ...and disable a drive
#define DISABLE_DRIVES {false, false, true, false, false, false, false, false} // Set true to disable a drive when it becomes idle
#define LOW_STOP_PINS {11, -1, 60, 31, 24, 46, 45, 44} //E Stops not currently used
#define HIGH_STOP_PINS {-1, 28, -1, -1, -1, -1, -1, -1}
//#define HOME_DIRECTION {1, 1, 1, -1, -1, -1, -1, -1} // 1 for Max/High, -1 for Min/ Low
#define ENDSTOP_HIT 1 // when a stop == this it is hit
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
#define Z_PROBE_AXES {true, false, true}		// Axes for which the Z-probe is normally used
const unsigned int numZProbeReadingsAveraged = 8;	// we average this number of readings with IR on, and the same number with IR off

#define MAX_FEEDRATES {100.0, 100.0, 3.0, 20.0, 20.0, 20.0, 20.0, 20.0} // mm/sec
#define ACCELERATIONS {500.0, 500.0, 20.0, 250.0, 250.0, 250.0, 250.0, 250.0} // mm/sec^2
#define DRIVE_STEPS_PER_UNIT {87.4890, 87.4890, 4000.0, 420.0, 420.0, 420.0, 420.0, 420.0}
#define INSTANT_DVS {15.0, 15.0, 0.2, 2.0, 2.0, 2.0, 2.0, 2.0} // (mm/sec)
#define NUM_MIXING_DRIVES 1; //number of mixing drives

#define E0_DRIVE 3 //the index of the first Extruder drive
#define E1_DRIVE 4 //the index of the second Extruder drive
#define E2_DRIVE 5 //the index of the third Extruder drive
#define E3_DRIVE 6 //the index of the fourth Extruder drive
#define E4_DRIVE 7 //the index of the fifth Extruder drive

// AXES

#define AXIS_MAXIMA {220, 200, 200} 			// mm
#define AXIS_MINIMA {0, 0, 0}					// mm
#define HOME_FEEDRATES {50.0, 50.0, 100.0/60.0}	// mm/sec (dc42 increased Z because we slow down z-homing when approaching the target height)
#define HEAD_OFFSETS {0.0, 0.0, 0.0}			// mm

#define X_AXIS 0  								// The index of the X axis in the arrays
#define Y_AXIS 1  								// The index of the Y axis
#define Z_AXIS 2  								// The index of the Z axis

// HEATERS - The bed is assumed to be the at index 0

#define TEMP_SENSE_PINS {5, 4, 0, 7, 8, 9} // Analogue pin numbers
#define HEAT_ON_PINS {6, X5, X7, 7, 8, 9} //pin D38 is PWM capable but not an Arduino PWM pin - //FIXME TEST if E1 PWM works as D38
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
const float defaultPidKis[HEATERS] = {5.0 / HEAT_SAMPLE_TIME, 0.1 / HEAT_SAMPLE_TIME, 0.1 / HEAT_SAMPLE_TIME, 0.1 / HEAT_SAMPLE_TIME, 0.1 / HEAT_SAMPLE_TIME, 0.1 / HEAT_SAMPLE_TIME}; // Integral PID constants
const float defaultPidKds[HEATERS] = {500.0 * HEAT_SAMPLE_TIME, 100 * HEAT_SAMPLE_TIME, 100 * HEAT_SAMPLE_TIME, 100 * HEAT_SAMPLE_TIME, 100 * HEAT_SAMPLE_TIME, 100 * HEAT_SAMPLE_TIME};	// Derivative PID constants
const float defaultPidKps[HEATERS] = {-1, 10.0, 10.0, 10.0, 10.0, 10.0};		// Proportional PID constants, negative values indicate use bang-bang instead of PID
const float defaultPidKts[HEATERS] = {2.7, 0.25, 0.25, 0.25, 0.25, 0.25};		// approximate PWM value needed to maintain temperature, per degC above room temperature
const float defaultPidKss[HEATERS] = {1.0, 0.9, 0.9, 0.9, 0.9, 0.9};			// PWM scaling factor, to allow for variation in heater power and supply voltage
const float defaultFullBand[HEATERS] = {5.0, 30.0, 30.0, 30.0, 30.0, 30.0};		// errors larger than this cause heater to be on or off
const float defaultPidMin[HEATERS] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};			// minimum value of I-term
const float defaultPidMax[HEATERS] = {255, 180, 180, 180, 180, 180};			// maximum value of I-term, must be high enough to reach 245C for ABS printing

#define STANDBY_TEMPERATURES {ABS_ZERO, ABS_ZERO, ABS_ZERO, ABS_ZERO, ABS_ZERO, ABS_ZERO} // We specify one for the bed, though it's not needed
#define ACTIVE_TEMPERATURES {ABS_ZERO, ABS_ZERO, ABS_ZERO, ABS_ZERO, ABS_ZERO, ABS_ZERO}
#define COOLING_FAN_PIN X6 														//pin D34 is PWM capable but not an Arduino PWM pin - use X6 instead
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
#define SD_SPI (4) //Pin
#define WEB_DIR "0:/www/" 						// Place to find web files on the SD card
#define GCODE_DIR "0:/gcodes/" 					// Ditto - g-codes
#define SYS_DIR "0:/sys/" 						// Ditto - system files
#define TEMP_DIR "0:/tmp/" 						// Ditto - temporary files

#define MAC_ADDRESS {0xBE, 0xEF, 0xDE, 0xAD, 0xFE, 0xED}


/****************************************************************************************************/

// Miscellaneous...

#define BAUD_RATE 115200 						// Communication speed of the USB if needed.

const int atxPowerPin = 12;						// Arduino Due pin number that controls the ATX power on/off

const uint16_t lineInBufsize = 256;				// use a power of 2 for good performance
const uint16_t lineOutBufSize = 2048;			// ideally this should be large enough to hold the results of an M503 command,
												// but could be reduced if we ever need the memory
const uint16_t fileListLength = 2000;			// increased to allow for the larger size of the Unix-compatible list when using FTP

/****************************************************************************************************/

enum EndStopHit
{
  noStop = 0,		// no endstop hit
  lowHit = 1,									// low switch hit, or Z-probe in use and above threshold
  highHit = 2,		// high stop hit
  lowNear = 3		// approaching Z-probe threshold
};

/***************************************************************************************************/

// Input and output - these are ORed into an int8_t
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
		stuckInSpin = 0x1000,		// we got stuck in a Spin() function for too long
		inLwipSpin = 0x2000,		// we got stuck in a call to lwip for too long
		inUsbOutput = 0x4000		// this bit is or'ed in if we were in USB otuput at the time
	};
}

// Enumeration to describe various tests we do in response to the M111 command
namespace DiagnosticTest
{
	enum
	{
		TestWatchdog = 1001,			// test that we get a watchdog reset if the tick interrupt stops
		TestSpinLockup = 1002			// test that we get a software reset if a Spin() function takes too long

	};
}

// This class handles serial I/O - typically via USB

class Line
{
public:

	int8_t Status() const; // Returns OR of IOStatus
	int Read(char& b);
	void Write(char b, bool block = false);
	void Write(const char* s, bool block = false);

friend class Platform;
friend class RepRap;

protected:

	Line();
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

	uint8_t inUsbWrite;
	bool ignoringOutputLine;
	unsigned int outputColumn;
};

class MassStorage
{
public:

  const char* FileList(const char* directory, bool fromLine); // Returns a list of all the files in the named directory
  const char* UnixFileList(const char* directory); // Returns a UNIX-compatible file list for the specified directory
  const char* CombineName(const char* directory, const char* fileName);
  bool Delete(const char* directory, const char* fileName);
  bool MakeDirectory(const char *parentDir, const char *dirName);
  bool MakeDirectory(const char *directory);
  bool Rename(const char *oldFilename, const char *newFilename);
  bool PathExists(const char *path) const;

friend class Platform;

protected:

  MassStorage(Platform* p);
  void Init();

private:

  char fileList[fileListLength];
  char scratchString[STRING_LENGTH];
  Platform* platform;
  FATFS fileSystem;
};

// This class handles input from, and output to, files.

class FileStore //: public InputOutput
{
public:

	int8_t Status(); // Returns OR of IOStatus
	bool Read(char& b);								// Read 1 byte
	int Read(char* buf, unsigned int nBytes);		// Read a block of nBytes length
	bool Write(char b);								// Write 1 byte
	bool Write(const char *s, unsigned int len);	// Write a block of len bytes
	bool Write(const char* s);						// Write a string
	bool Close();									// Shut the file and tidy up
	bool Seek(unsigned long pos);					// Jump to pos in the file
	bool GoToEnd();									// Position the file at the end (so you can write on the end).
	unsigned long Length();							// File size in bytes
	void Duplicate();								// Create a second reference to this file
	bool Flush();									// Write remaining buffer data

friend class Platform;

protected:

	FileStore(Platform* p);
	void Init();
    bool Open(const char* directory, const char* fileName, bool write);
        
private:

  bool inUse;
  byte buf[FILE_BUF_LEN];
  int bufferPointer;
  
  bool ReadBuffer();
  bool WriteBuffer();

  FIL file;
  Platform* platform;
  bool writing;
  unsigned int lastBufferEntry;
  unsigned int openCount;
};


/***************************************************************************************************************/

// Struct for holding Z probe parameters

struct ZProbeParameters
{
	int adcValue;					// the target ADC value
	float height;					// the nozzle height at which the target ADC value is returned
	float calibTemperature;			// the temperature at which we did the calibration
	float temperatureCoefficient;	// the variation of height with bed temperature

	void Init(float h)
	{
		adcValue = Z_PROBE_AD_VALUE;
		height = h;
		calibTemperature = 20.0;
		temperatureCoefficient = 0.0;	// no default temperature correction
	}

	float GetStopHeight(float temperature) const
	{
		return ((temperature - calibTemperature) * temperatureCoefficient) + height;
	}

	bool operator==(const ZProbeParameters& other) const
	{
		return adcValue == other.adcValue
				&& height == other.height
				&& calibTemperature == other.calibTemperature
				&& temperatureCoefficient == other.temperatureCoefficient;
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
		sum = (uint32_t)val * (uint32_t)numAveraged;
		index = 0;
		isValid = false;
		for(size_t i = 0; i < numAveraged; ++i)
		{
			readings[i] = val;
		}
	}

	// Call this to put a new reading into the filter
	// This is only called by the ISR, so it not declared volatile to make it faster
	void ProcessReading(uint16_t r)
	{
		sum = sum - readings[index] + r;
		readings[index] = r;
		++index;
		if(index == numAveraged)
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
  
  Platform();
  
//-------------------------------------------------------------------------------------------------------------

// These are the functions that form the interface between Platform and the rest of the firmware.

  void Init(); // Set the machine up after a restart.  If called subsequently this should set the machine up as if
               // it has just been restarted; it can do this by executing an actual restart if you like, but beware the 
               // loop of death...
  void Spin(); // This gets called in the main loop and should do any housekeeping needed
  void Exit(); // Shut down tidily.  Calling Init after calling this should reset to the beginning
  Compatibility Emulating() const;
  void SetEmulating(Compatibility c);
  void Diagnostics();
  void DiagnosticTest(int d);
  void ClassReport(char* className, float &lastTime);  // Called on return to check everything's live.
  void RecordError(ErrorCode ec) { errorCodeBits |= ec; }
  void SoftwareReset(uint16_t reason);
  void SetAtxPower(bool on);

  // Timing
  
  float Time(); // Returns elapsed seconds since some arbitrary time
  void SetInterrupt(float s); // Set a regular interrupt going every s seconds; if s is -ve turn interrupt off
  //void DisableInterrupts();
  void Tick();
  
  // Communications and data storage
  
  Line* GetLine() const;
  void SetIPAddress(byte ip[]);
  const byte* IPAddress() const;
  void SetNetMask(byte nm[]);
  const byte* NetMask() const;
  void SetGateWay(byte gw[]);
  const byte* GateWay() const;
  void SetMACAddress(uint8_t mac[]);
  const uint8_t* MACAddress() const;
  
  friend class FileStore;
  
  MassStorage* GetMassStorage();
  FileStore* GetFileStore(const char* directory, const char* fileName, bool write);
  const char* GetWebDir() const; // Where the htm etc files are
  const char* GetGCodeDir() const; // Where the gcodes are
  const char* GetSysDir() const;  // Where the system files are
  const char* GetTempDir() const; // Where temporary files are
  const char* GetConfigFile() const; // Where the configuration is stored (in the system dir).
  
  void Message(char type, const char* message);        // Send a message.  Messages may simply flash an LED, or,
                            // say, display the messages on an LCD. This may also transmit the messages to the host.
  void AppendMessage(char type, const char* message);        // Send a message.  Messages may simply flash an LED, or,
                              // say, display the messages on an LCD. This may also transmit the messages to the host.
  void PushMessageIndent();
  void PopMessageIndent();
  
  // Movement
  
  void EmergencyStop();
  void SetDirection(byte drive, bool direction);
  void Step(byte drive);
  void Disable(byte drive); // There is no drive enable; drives get enabled automatically the first time they are used.
  void SetMotorCurrent(byte drive, float current);
  float MotorCurrent(byte drive);
  float DriveStepsPerUnit(int8_t drive) const;
  void SetDriveStepsPerUnit(int8_t drive, float value);
  float Acceleration(int8_t drive) const;
  const float* Accelerations() const;
  void SetAcceleration(int8_t drive, float value);
  float MaxFeedrate(int8_t drive) const;
  const float* MaxFeedrates() const;
  void SetMaxFeedrate(int8_t drive, float value);
  float InstantDv(int8_t drive) const;
  void SetInstantDv(int8_t drive, float value);
  const float* InstantDvs() const;
  float HomeFeedRate(int8_t axis) const;
  void SetHomeFeedRate(int8_t axis, float value);
  EndStopHit Stopped(int8_t drive);
  float AxisMaximum(int8_t axis) const;
  void SetAxisMaximum(int8_t axis, float value);
  float AxisMinimum(int8_t axis) const;
  void SetAxisMinimum(int8_t axis, float value);
  float AxisTotalLength(int8_t axis) const;

  // Z probe

  float ZProbeStopHeight() const;
  int ZProbe() const;
  int GetZProbeSecondaryValues(int& v1, int& v2);
  void SetZProbeType(int iZ);
  int GetZProbeType() const;
  void SetZProbeAxes(const bool axes[AXES]);
  void GetZProbeAxes(bool (&axes)[AXES]);
  void SetZProbing(bool starting);
  bool GetZProbeParameters(struct ZProbeParameters& params) const;
  bool SetZProbeParameters(const struct ZProbeParameters& params);
  bool MustHomeXYBeforeZ() const;
  
  // Mixing support

//  void SetMixingDrives(int);
//  int GetMixingDrives();

  int8_t SlowestDrive() const;

  // Heat and temperature
  
  float GetTemperature(size_t heater) const; // Result is in degrees Celsius
  void SetHeater(size_t heater, const float& power); // power is a fraction in [0,1]
  float HeatSampleTime() const;
  void SetHeatSampleTime(float st);
  void CoolingFan(float speed);
  void SetPidParameters(size_t heater, const PidParameters& params);
  const PidParameters& GetPidParameters(size_t heater);

//-------------------------------------------------------------------------------------------------------
  
private:
  
  // This is the structure used to hold out non-volatile data.
  // The SAM3X doesn't have EEPROM so we save the data to flash. This unfortunately means that it gets cleared
  // every time we reprogram the firmware. So there is no need to cater for writing one version of this
  // struct and reading back another.

  struct FlashData
  {
	  static const uint16_t magicValue = 0x59B2;	// value we use to recognise that the flash data has been written

	  uint16_t magic;
	  uint16_t resetReason;							// this records why we did a software reset, for diagnostic purposes
	  size_t neverUsedRam;							// the amount of never used RAM at the last abnormal software reset

	  // The remaining data could alternatively be saved to SD card.
	  // Note however that if we save them as G codes, we need to provide a way of saving IR and ultrasonic G31 parameters separately.
	  ZProbeParameters switchZProbeParameters;		// Z probe values for the endstop switch
	  ZProbeParameters irZProbeParameters;			// Z probe values for the IR sensor
	  ZProbeParameters alternateZProbeParameters;	// Z probe values for the alternate sensor
	  int zProbeType;								// the type of Z probe we are currently using
	  bool zProbeAxes[AXES];						// Z probe is used for these axes
	  PidParameters pidParams[HEATERS];
	  byte ipAddress[4];
	  byte netMask[4];
	  byte gateWay[4];
	  uint8_t macAddress[6];
	  Compatibility compatibility;
  };

  static const uint32_t nvAddress = 0;				// address in flash where we store the nonvolatile data
  FlashData nvData;

  float lastTime;
  float longWait;
  float addToTime;
  unsigned long lastTimeCall;
  
  bool active;
  Compatibility compatibility;
  uint32_t errorCodeBits;
  
  void InitialiseInterrupts();
  int GetRawZHeight() const;
  void GetStackUsage(size_t* currentStack, size_t* maxStack, size_t* neverUsed) const;

// DRIVES

  void SetSlowestDrive();

  int8_t stepPins[DRIVES];
  int8_t directionPins[DRIVES];
  int8_t enablePins[DRIVES];
  bool disableDrives[DRIVES];
  bool driveEnabled[DRIVES];
  int8_t lowStopPins[DRIVES];
  int8_t highStopPins[DRIVES];
  float maxFeedrates[DRIVES];  
  float accelerations[DRIVES];
  float driveStepsPerUnit[DRIVES];
  float instantDvs[DRIVES];
  MCP4461 mcpDuet;
  MCP4461 mcpExpansion;
  int8_t slowestDrive;


  int8_t potWipes[DRIVES];
  float senseResistor;
  float maxStepperDigipotVoltage;
  int8_t zProbePin;
  int8_t zProbeModulationPin;

  volatile ZProbeAveragingFilter zProbeOnFilter;					// Z probe readings we took with the IR turned on
  volatile ZProbeAveragingFilter zProbeOffFilter;					// Z probe readings we took with the IR turned off
  volatile ThermistorAveragingFilter thermistorFilters[HEATERS];	// bed and extruder thermistor readings
  int8_t numMixingDrives;

// AXES

  void InitZProbe();
  void UpdateNetworkAddress(byte dst[4], const byte src[4]);
  void WriteNvData();

  float axisMaxima[AXES];
  float axisMinima[AXES];
  float homeFeedrates[AXES];
  float headOffsets[AXES]; // FIXME - needs a 2D array
  
// HEATERS - Bed is assumed to be the first

  int GetRawTemperature(byte heater) const;

  int8_t tempSensePins[HEATERS];
  int8_t heatOnPins[HEATERS];
  float heatSampleTime;
  float standbyTemperatures[HEATERS];
  float activeTemperatures[HEATERS];
  int8_t coolingFanPin;

// Serial/USB

  Line* line;
  uint8_t messageIndent;

// Files

  MassStorage* massStorage;
  FileStore* files[MAX_FILES];
  bool fileStructureInitialised;
  char* webDir;
  char* gcodeDir;
  char* sysDir;
  char* tempDir;
  char* configFile;
  
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

	unsigned long Length()
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



//*****************************************************************************************************************

// Drive the RepRap machine - Movement

inline float Platform::DriveStepsPerUnit(int8_t drive) const
{
  return driveStepsPerUnit[drive]; 
}

inline void Platform::SetDriveStepsPerUnit(int8_t drive, float value)
{
  driveStepsPerUnit[drive] = value;
}

inline float Platform::Acceleration(int8_t drive) const
{
	return accelerations[drive];
}

inline const float* Platform::Accelerations() const
{
	return accelerations;
}

inline void Platform::SetAcceleration(int8_t drive, float value)
{
	accelerations[drive] = value;
}

inline float Platform::MaxFeedrate(int8_t drive) const
{
  return maxFeedrates[drive];
}

inline const float* Platform::MaxFeedrates() const
{
  return maxFeedrates;
}

inline void Platform::SetMaxFeedrate(int8_t drive, float value)
{
	maxFeedrates[drive] = value;
}

inline float Platform::InstantDv(int8_t drive) const
{
  return instantDvs[drive]; 
}

inline void Platform::SetInstantDv(int8_t drive, float value)
{
	instantDvs[drive] = value;
	SetSlowestDrive();
}

inline int8_t Platform::SlowestDrive() const
{
	return slowestDrive;
}

inline const float* Platform::InstantDvs() const
{
  return instantDvs;
}

#if 0	// not used
inline bool Platform::HighStopButNotLow(int8_t axis) const
{
	return (lowStopPins[axis] < 0) && (highStopPins[axis] >= 0);
}
#endif

inline float Platform::HomeFeedRate(int8_t axis) const
{
  return homeFeedrates[axis];
}

inline void Platform::SetHomeFeedRate(int8_t axis, float value)
{
   homeFeedrates[axis] = value;
}

inline float Platform::AxisMaximum(int8_t axis) const
{
  return axisMaxima[axis];
}

inline void Platform::SetAxisMaximum(int8_t axis, float value)
{
  axisMaxima[axis] = value;
}

inline float Platform::AxisMinimum(int8_t axis) const
{
  return axisMinima[axis];
}

inline void Platform::SetAxisMinimum(int8_t axis, float value)
{
  axisMinima[axis] = value;
}

inline float Platform::AxisTotalLength(int8_t axis) const
{
	return axisMaxima[axis] - axisMinima[axis];
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

inline const byte* Platform::IPAddress() const
{
	return nvData.ipAddress;
}

inline const byte* Platform::NetMask() const
{
	return nvData.netMask;
}

inline const byte* Platform::GateWay() const
{
	return nvData.gateWay;
}

inline void Platform::SetMACAddress(uint8_t mac[])
{
	bool changed = false;
	for(int8_t i = 0; i < 6; i++)
	{
		if (nvData.macAddress[i] != mac[i])
		{
			nvData.macAddress[i] = mac[i];
			changed = true;
		}
	}
	if (changed)
	{
		WriteNvData();
	}
}

inline const byte* Platform::MACAddress() const
{
	return nvData.macAddress;
}

inline Line* Platform::GetLine() const
{
	return line;
}

inline void Platform::PushMessageIndent()
{
	messageIndent += 2;
}

inline void Platform::PopMessageIndent()
{
	messageIndent -= 2;
}


//***************************************************************************************


#endif
