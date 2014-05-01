/****************************************************************************************************

RepRapFirmware - Platform: RepRapPro Mendel with Duet controller

Platform contains all the code and definitons to deal with machine-dependent things such as control 
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

#define ELECTRONICS "Duet"

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

#define STRING_LENGTH 1029	// needs to be long enough to receive web data
#define SHORT_STRING_LENGTH 40
#define TIME_TO_REPRAP 1.0e6 // Convert seconds to the units used by the machine (usually microseconds)
#define TIME_FROM_REPRAP 1.0e-6 // Convert the units used by the machine (usually microseconds) to seconds

/**************************************************************************************************/

// The physical capabilities of the machine

#define DRIVES 4  // The number of drives in the machine, including X, Y, and Z plus extruder drives
#define AXES 3    // The number of movement axes in the machine, usually just X, Y and Z. <= DRIVES
#define HEATERS 2 // The number of heaters in the machine; 0 is the heated bed even if there isn't one.

// The numbers of entries in each array must correspond with the values of DRIVES,
// AXES, or HEATERS.  Set values to -1 to flag unavailability.

// DRIVES

#define STEP_PINS {14, 25, 5, X2}
#define DIRECTION_PINS {15, 26, 4, X3}
#define FORWARDS true     // What to send to go... 
#define BACKWARDS false    // ...in each direction
#define ENABLE_PINS {29, 27, X1, X0}
#define ENABLE false      // What to send to enable... 
#define DISABLE true     // ...and disable a drive
#define DISABLE_DRIVES {false, false, true, false} // Set true to disable a drive when it becomes idle
#define LOW_STOP_PINS {11, -1, 60, 31}
#define HIGH_STOP_PINS {-1, 28, -1, -1}
#define ENDSTOP_HIT 1 // when a stop == this it is hit
#define POT_WIPES {1, 3, 2, 0} // Indices for motor current digipots (if any)
#define SENSE_RESISTOR 0.1   // Stepper motor current sense resistor
#define MAX_STEPPER_DIGIPOT_VOLTAGE ( 3.3*2.5/(2.7+2.5) ) // Stepper motor current reference voltage

#define Z_PROBE_AD_VALUE (400)
#define Z_PROBE_STOP_HEIGHT (0.7) // mm
#define Z_PROBE_PIN (0) 		// Analogue pin number
#define Z_PROBE_MOD_PIN (61)	// Digital pin number to turn the IR LED on (high) or off (low)
const unsigned int numZProbeReadingsAveraged = 8;	// we average this number of readings with IR on, and the same number with IR off

#define MAX_FEEDRATES {50.0, 50.0, 3.0, 16.0}    // mm/sec
#define ACCELERATIONS {800.0, 800.0, 10.0, 250.0}    // mm/sec^2
#define DRIVE_STEPS_PER_UNIT {87.4890, 87.4890, 4000.0, 420.0}
#define INSTANT_DVS {10.0, 10.0, 0.2, 2.0}    // (mm/sec)

// AXES

#define AXIS_LENGTHS {220, 200, 200} // mm
#define HOME_FEEDRATES {50.0, 50.0, 1.0}  // mm/sec
#define HEAD_OFFSETS {0.0, 0.0, 0.0}

#define X_AXIS 0  // The index of the X axis
#define Y_AXIS 1  // The index of the Y axis
#define Z_AXIS 2  // The index of the Z axis


// HEATERS - The bed is assumed to be the first

#define TEMP_SENSE_PINS {5, 4}   // Analogue pin numbers
#define HEAT_ON_PINS {6, X5}

// Bed thermistor: http://uk.farnell.com/epcos/b57863s103f040/sensor-miniature-ntc-10k/dp/1299930?Ntt=129-9930
// Hot end thermistor: http://www.digikey.co.uk/product-search/en?x=20&y=11&KeyWords=480-3137-ND
const float defaultThermistorBetas[HEATERS] = {3988.0, 4138.0};
const float defaultThermistorSeriesRs[HEATERS] = {1000, 1000}; 		// Ohms in series with the thermistors
const float defaultThermistor25RS[HEATERS] = {10000.0, 100000.0};	// Thermistor ohms at 25 C = 298.15 K

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

#if 1 	// if using method 2 above
const float defaultPidKis[HEATERS] = {5.0 / HEAT_SAMPLE_TIME, 0.2 / HEAT_SAMPLE_TIME};
const float defaultPidKds[HEATERS] = {500.0 * HEAT_SAMPLE_TIME, 50.0 * HEAT_SAMPLE_TIME};
const float defaultPidKps[HEATERS] = {-1, 9.0};
const float defaultFullBand[HEATERS] = {5.0, 20.0};		// errors larger than this cause heater to be on or off and I-term set to zero
#else	// using method 1 above
const float defaultPidKis[HEATERS] = {5.0 / HEAT_SAMPLE_TIME, 0.027 / HEAT_SAMPLE_TIME};
const float defaultPidKds[HEATERS] = {500.0 * HEAT_SAMPLE_TIME, 50.0 * HEAT_SAMPLE_TIME};
const float defaultPidKps[HEATERS] = {-1, 20.0};
const float defaultFullBand[HEATERS] = {5.0, 150.0};	// errors larger than this cause heater to be on or off and I-term set to zero
#endif

const float defaultPidMin[HEATERS] = {0.0, 0.0};	// minimum value of I-term
const float defaultPidMax[HEATERS] = {255, 180};	// maximum value of I-term, must be high enough to reach 245C for ABS printing

#define STANDBY_TEMPERATURES {ABS_ZERO, ABS_ZERO} // We specify one for the bed, though it's not needed
#define ACTIVE_TEMPERATURES {ABS_ZERO, ABS_ZERO}
#define COOLING_FAN_PIN X6
#define HEAT_ON 0 // 0 for inverted heater (e.g. Duet v0.6) 1 for not (e.g. Duet v0.4)

// For the theory behind ADC oversampling, see http://www.atmel.com/Images/doc8003.pdf
const unsigned int adOversampleBits = 1;					// number of bits we oversample when reading temperatures

// Define the number of temperature readings we average for each thermistor. This should be a power of 2 and at least 4 ** adOversampleBits.
// Keep numThermistorReadingsAveraged * NUM_HEATERS * 2ms no greater than HEAT_SAMPLE_TIME or the PIDs won't work well.
const unsigned int numThermistorReadingsAveraged = (HEATERS > 3) ? 32 : 64;
const unsigned int adRangeReal = 4095;						// the ADC that measures temperatures gives an int this big as its max value
const unsigned int adRangeVirtual = ((adRangeReal + 1) << adOversampleBits) - 1;	// the max value we can get using oversampling
const unsigned int adDisconnectedReal = adRangeReal - 3;	// we consider an ADC reading at/above this value to indicate that the thermistor is disconnected
const unsigned int adDisconnectedVirtual = adDisconnectedReal << adOversampleBits;

#define HOT_BED 0 // The index of the heated bed; set to -1 if there is no heated bed

/****************************************************************************************************/

// File handling

#define MAX_FILES (10)		// must be large enough to handle the max number of simultaneous web requests + file being printed
#define FILE_BUF_LEN (256)
#define SD_SPI (4) //Pin
#define WEB_DIR "0:/www/" // Place to find web files on the server
#define GCODE_DIR "0:/gcodes/" // Ditto - g-codes
#define SYS_DIR "0:/sys/" // Ditto - system files
#define TEMP_DIR "0:/tmp/" // Ditto - temporary files
#define FILE_LIST_SEPARATOR ','
#define FILE_LIST_BRACKET '"'
#define FILE_LIST_LENGTH (1000) // Maximum length of file list - can't make it much longer unless we also make jsonResponse longer

#define FLASH_LED 'F' 		// Type byte of a message that is to flash an LED; the next two bytes define
                      	  	  // the frequency and M/S ratio.
#define DISPLAY_MESSAGE 'L'	// Type byte of a message that is to appear on a local display; the L is
                            // not displayed; \f and \n should be supported.
#define HOST_MESSAGE 'H'	// Type byte of a message that is to be sent to the host; the H is not sent.
#define DEBUG_MESSAGE 'D'	// Type byte of a message that is to be sent for debugging; the D is not sent.

/****************************************************************************************************/

// Miscellaneous...

//#define LED_PIN 13 // Indicator LED

#define BAUD_RATE 115200 // Communication speed of the USB if needed.

const int atxPowerPin = 12;						// Arduino Due pin number that controls the ATX power on/off

const uint16_t lineInBufsize = 256;				// use a power of 2 for good performance
const uint16_t lineOutBufSize = 2048;			// ideally this should be large enough to hold the results of an M503 command,
												// but could be reduced if we ever need the memory
const uint16_t NumZProbeReadingsAveraged = 8;	// must be an even number, preferably a power of 2 for performance, and no greater than 64

/****************************************************************************************************/

enum EndStopHit
{
  noStop = 0,		// no endstop hit
  lowHit = 1,		// low switch hit, or Z-probe in use and above threshold
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

class Line //: public InputOutput
{
public:

	int8_t Status() const; // Returns OR of IOStatus
	int Read(char& b);
	void Write(char b, bool block = false);
	void Write(const char* s, bool block = false);

friend class Platform;

protected:

	Line();
	void Init();
	void Spin();

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
};

class MassStorage
{
public:

  const char* FileList(const char* directory, bool fromLine); // Returns a list of all the files in the named directory
  const char* CombineName(const char* directory, const char* fileName);
  bool Delete(const char* directory, const char* fileName);

friend class Platform;

protected:

  MassStorage(Platform* p);
  void Init();

private:

  char fileList[FILE_LIST_LENGTH];
  char scratchString[STRING_LENGTH];
  Platform* platform;
  FATFS fileSystem;
};

// This class handles input from, and output to, files.

class FileStore //: public InputOutput
{
public:

	int8_t Status(); // Returns OR of IOStatus
	bool Read(char& b);
	int Read(char* buf, unsigned int nBytes);
	void Write(char b);
	void Write(const char* s);
	bool Close();
	bool Seek(unsigned long pos);
	bool GoToEnd(); // Position the file at the end (so you can write on the end).
	unsigned long Length(); // File size in bytes
	void Duplicate();

friend class Platform;

protected:

	FileStore(Platform* p);
	void Init();
    bool Open(const char* directory, const char* fileName, bool write);
        
  bool inUse;
  byte buf[FILE_BUF_LEN];
  int bufferPointer;
  
private:

  void ReadBuffer();
  void WriteBuffer();

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
	float kI, kD, kP;
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
		Init();
	}

	void Init() volatile
	{
		sum = 0;
		index = 0;
		isValid = false;
		for(size_t i = 0; i < numAveraged; ++i)
		{
			readings[i] = 0;
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
	bool IsValid()  const volatile
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
  void PrintMemoryUsage();  // Print memory stats for debugging
  void ClassReport(char* className, float &lastTime);  // Called on return to check everything's live.
  void RecordError(ErrorCode ec) { errorCodeBits |= ec; }
  void SetDebug(int d);
  void SoftwareReset(uint16_t reason);
  void SetAtxPower(bool on);

  // Timing
  
  float Time(); // Returns elapsed seconds since some arbitrary time
  void SetInterrupt(float s); // Set a regular interrupt going every s seconds; if s is -ve turn interrupt off
  void DisableInterrupts();
  void Tick();

  // Communications and data storage
  
  Line* GetLine() const;
  void SetIPAddress(byte ip[]);
  const byte* IPAddress() const;
  void SetNetMask(byte nm[]);
  const byte* NetMask() const;
  void SetGateWay(byte gw[]);
  const byte* GateWay() const;
  
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
  void PushMessageIndent();
  void PopMessageIndent();
  
  // Movement
  
  void EmergencyStop();
  void SetDirection(byte drive, bool direction);
  void Step(byte drive);
  void Disable(byte drive); // There is no drive enable; drives get enabled automatically the first time they are used.
  void SetMotorCurrent(byte drive, float current);
  float DriveStepsPerUnit(int8_t drive) const;
  void SetDriveStepsPerUnit(int8_t drive, float value);
  float Acceleration(int8_t drive) const;
  void SetAcceleration(int8_t drive, float value);
  float MaxFeedrate(int8_t drive) const;
  void SetMaxFeedrate(int8_t drive, float value);
  float InstantDv(int8_t drive) const;
  float HomeFeedRate(int8_t axis) const;
  void SetHomeFeedRate(int8_t axis, float value);
  EndStopHit Stopped(int8_t drive);
  float AxisLength(int8_t axis) const;
  void SetAxisLength(int8_t axis, float value);
#if 0	// not used
  bool HighStopButNotLow(int8_t axis) const;
#endif

  // Z probe

  float ZProbeStopHeight() const;
  int ZProbe();
  int GetZProbeSecondaryValues(int& v1, int& v2);
  void SetZProbeType(int iZ);
  int GetZProbeType() const;
  void SetZProbing(bool starting);
  bool GetZProbeParameters(struct ZProbeParameters& params) const;
  bool SetZProbeParameters(const struct ZProbeParameters& params);
  bool MustHomeXYBeforeZ() const;

  // Heat and temperature
  
  float GetTemperature(size_t heater) const; // Result is in degrees Celsius
  void SetHeater(size_t heater, const float& power); // power is a fraction in [0,1]
  float HeatSampleTime() const;
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
	  ZProbeParameters ultrasonicZProbeParameters;	// Z probe values for the ultrasonic sensor
	  int zProbeType;								// the type of Z probe we are currently using
	  PidParameters pidParams[HEATERS];
	  byte ipAddress[4];
	  byte netMask[4];
	  byte gateWay[4];
	  Compatibility compatibility;
  };

  static const uint32_t nvAddress = 0;				// address in flash where we store the nonvolatile data
  FlashData nvData;

  float lastTime;
  float longWait;
  float addToTime;
  unsigned long lastTimeCall;
  
  bool active;
  uint32_t errorCodeBits;
  
  void InitialiseInterrupts();
  int GetRawZHeight() const;
  void GetStackUsage(size_t* currentStack, size_t* maxStack, size_t* neverUsed) const;

// DRIVES

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
  MCP4461 mcp;
  int8_t potWipes[DRIVES];
  float senseResistor;
  float maxStepperDigipotVoltage;
  int8_t zProbePin;
  int8_t zProbeModulationPin;

  volatile ZProbeAveragingFilter zProbeOnFilter;					// Z probe readings we took with the IR turned on
  volatile ZProbeAveragingFilter zProbeOffFilter;					// Z probe readings we took with the IR turned off
  volatile ThermistorAveragingFilter thermistorFilters[HEATERS];	// bed and extruder thermistor readings

// AXES

  void InitZProbe();
  void UpdateNetworkAddress(byte dst[4], const byte src[4]);
  void WriteNvData();

  float axisLengths[AXES];
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

// Seconds

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

inline void Platform::SetAcceleration(int8_t drive, float value)
{
	accelerations[drive] = value;
}

inline float Platform::InstantDv(int8_t drive) const
{
  return instantDvs[drive]; 
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

inline float Platform::AxisLength(int8_t axis) const
{
  return axisLengths[axis];
}

inline void Platform::SetAxisLength(int8_t axis, float value)
{
  axisLengths[axis] = value;
}

inline float Platform::MaxFeedrate(int8_t drive) const
{
  return maxFeedrates[drive];
}

inline void Platform::SetMaxFeedrate(int8_t drive, float value)
{
	maxFeedrates[drive] = value;
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

//****************************************************************************************************************

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
