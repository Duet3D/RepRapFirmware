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

#include <stdio.h>
#include <ctype.h>
#include <string.h>
#include <malloc.h>
#include <stdlib.h>
#include <limits.h>

// Platform-specific includes

#include "Arduino.h"
#include "SamNonDuePin.h"
#include "SD_HSMCI.h"
#include "MCP4461.h"
#include "ethernet_sam.h"

/**************************************************************************************************/

// Some numbers...

#define STRING_LENGTH 1029
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
#define Z_PROBE_AD_VALUE 400
#define Z_PROBE_STOP_HEIGHT 0.7 // mm
#define Z_PROBE_PIN 0 // Analogue pin number
#define MAX_FEEDRATES {50.0, 50.0, 3.0, 16.0}    // mm/sec
#define ACCELERATIONS {800.0, 800.0, 10.0, 250.0}    // mm/sec^2
#define DRIVE_STEPS_PER_UNIT {87.4890, 87.4890, 4000.0, 420.0}
#define INSTANT_DVS {15.0, 15.0, 0.2, 2.0}    // (mm/sec)

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
#define THERMISTOR_BETAS {3988.0, 4138.0}
#define THERMISTOR_SERIES_RS {1000, 1000} // Ohms in series with the thermistors
#define THERMISTOR_25_RS {10000.0, 100000.0} // Thermistor ohms at 25 C = 298.15 K

#define USE_PID {false, true} // PID or bang-bang for this heater?
#define PID_KIS {-1, 2.2} // PID constants...
#define PID_KDS {-1, 80}
#define PID_KPS {-1, 12}
#define FULL_PID_BAND {-1, 150.0}
#define PID_MIN {-1, 0.0}
#define PID_MAX {-1, 125.0}
#define D_MIX {-1, 0.95}
#define TEMP_INTERVAL 0.122 // secs - check and control temperatures this often
#define STANDBY_TEMPERATURES {ABS_ZERO, ABS_ZERO} // We specify one for the bed, though it's not needed
#define ACTIVE_TEMPERATURES {ABS_ZERO, ABS_ZERO}
#define COOLING_FAN_PIN 34
#define HEAT_ON 0 // 0 for inverted heater (eg Duet v0.6) 1 for not (e.g. Duet v0.4)

#define AD_RANGE 1023.0//16383 // The A->D converter that measures temperatures gives an int this big as its max value

#define HOT_BED 0 // The index of the heated bed; set to -1 if there is no heated bed

/****************************************************************************************************/

// File handling

#define MAX_FILES 7
#define FILE_BUF_LEN 256
#define SD_SPI 4 //Pin
#define WEB_DIR "0:/www/" // Place to find web files on the server
#define GCODE_DIR "0:/gcodes/" // Ditto - g-codes
#define SYS_DIR "0:/sys/" // Ditto - system files
#define TEMP_DIR "0:/tmp/" // Ditto - temporary files
#define FILE_LIST_SEPARATOR ','
#define FILE_LIST_BRACKET '"'
#define FILE_LIST_LENGTH 1000 // Maximum length of file list

#define FLASH_LED 'F' // Type byte of a message that is to flash an LED; the next two bytes define 
                      // the frequency and M/S ratio.
#define DISPLAY_MESSAGE 'L'  // Type byte of a message that is to appear on a local display; the L is 
                             // not displayed; \f and \n should be supported.
#define HOST_MESSAGE 'H' // Type byte of a message that is to be sent to the host; the H is not sent.

/****************************************************************************************************/

// Networking

// Seconds to wait after serving a page
 
#define CLIENT_CLOSE_DELAY 0.002

#define HTTP_STATE_SIZE 5

#define IP_ADDRESS {192, 168, 1, 10} // Need some sort of default...
#define NET_MASK {255, 255, 255, 0}
#define GATE_WAY {192, 168, 1, 1}


/****************************************************************************************************/

// Miscellaneous...

//#define LED_PIN 13 // Indicator LED

#define BAUD_RATE 115200 // Communication speed of the USB if needed.

const uint16_t lineBufsize = 256;	// use a power of 2 for good performance

/****************************************************************************************************/

enum EndStopHit
{
  noStop = 0,		// no enstop hit
  lowHit = 1,		// low switch hit, or Z-probe in use and above threshold
  highHit = 2		// high stop hit
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

//// All IO is done by classes derived from this class.
//
//class InputOutput
//{
//public:
//	void TakeInputFrom(InputOutput* altIp);
//	void SendOutputTo(InputOutput* altOp);
//
//protected:
//	InputOutput* alternateInput;
//	InputOutput* alternateOutput;
//};

// This class handles the network - typically an ethernet.

// Start with a ring buffer to hold input from the network
// that needs to be responded to.

class NetRing
{
public:
	friend class Network;

protected:
	NetRing(NetRing* n);
	NetRing* Next();
	bool Set(char* d, int l, void* pb, void* pc, void* h);
	char* Data();
	int Length();
	bool ReadFinished();
	void SetReadFinished();
	void* Pbuf();
	void* Pcb();
	void* Hs();
	bool Active();
	void Free();
	void SetNext(NetRing* n);
	void ReleasePbuf();
	void ReleaseHs();

private:
	void Reset();
	void* pbuf;
	void* pcb;
	void* hs;
	char* data;
	int length;
	bool read;
	bool active;
	NetRing* next;
};

// The main network class that drives the network.

class Network //: public InputOutput
{
public:

	int8_t Status() const; // Returns OR of IOStatus
	bool Read(char& b);
	bool CanWrite() const;
	void SetWriteEnable(bool enable);
	void Write(char b);
	void Write(char* s);
	void Close();
	void ReceiveInput(char* data, int length, void* pb, void* pc, void* h);
	void InputBufferReleased(void* pb);
	void ConnectionError(void* h);
	bool Active() const;
	bool LinkIsUp();

friend class Platform;

protected:

	Network();
	void Init();
	void Spin();

private:

	void Reset();
	void CleanRing();
	char* inputBuffer;
	char outputBuffer[STRING_LENGTH];
	int inputPointer;
	int inputLength;
	int outputPointer;
	bool writeEnabled;
	bool closePending;
	int8_t status;
	NetRing* netRingGetPointer;
	NetRing* netRingAddPointer;
	bool active;
};

// This class handles serial I/O - typically via USB

class Line //: public InputOutput
{
public:

	int8_t Status() const; // Returns OR of IOStatus
	int Read(char& b);
	void Write(char b);
	void Write(char* s);
	void Write(float f);
	void Write(long l);

friend class Platform;

protected:

	Line();
	void Init();
	void Spin();

private:
	char buffer[lineBufsize];
	uint16_t getIndex;
	uint16_t numChars;
};

class MassStorage
{
public:

  char* FileList(char* directory, bool fromLine); // Returns a list of all the files in the named directory
  char* CombineName(char* directory, char* fileName);
  bool Delete(char* directory, char* fileName);

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
	void Write(char b);
	void Write(char* s);
	void Close();
	void GoToEnd(); // Position the file at the end (so you can write on the end).
	unsigned long Length(); // File size in bytes

friend class Platform;

protected:

	FileStore(Platform* p);
	void Init();
        bool Open(char* directory, char* fileName, bool write);
        
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
};


/***************************************************************************************************************/

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
  
  Compatibility Emulating();

  void SetEmulating(Compatibility c);

  void Diagnostics();
  
  void PrintMemoryUsage();  // Print memory stats for debugging

  void ClassReport(char* className, float &lastTime);  // Called on return to check everything's live.

  // Timing
  
  float Time(); // Returns elapsed seconds since some arbitrary time
  
  void SetInterrupt(float s); // Set a regular interrupt going every s seconds; if s is -ve turn interrupt off
  
  void DisableInterrupts();

  // Communications and data storage
  
  Network* GetNetwork();
  Line* GetLine();
  void SetIPAddress(byte ip[]);
  byte* IPAddress();
  void SetNetMask(byte nm[]);
  byte* NetMask();
  void SetGateWay(byte gw[]);
  byte* GateWay();
  
  friend class FileStore;
  
  MassStorage* GetMassStorage();
  FileStore* GetFileStore(char* directory, char* fileName, bool write);
  void StartNetwork();
  char* GetWebDir(); // Where the htm etc files are
  char* GetGCodeDir(); // Where the gcodes are
  char* GetSysDir();  // Where the system files are
  char* GetTempDir(); // Where temporary files are
  char* GetConfigFile(); // Where the configuration is stored (in the system dir).
  
  void Message(char type, char* message);        // Send a message.  Messages may simply flash an LED, or, 
                            // say, display the messages on an LCD. This may also transmit the messages to the host.
  void PushMessageIndent();
  void PopMessageIndent();
  
  // Movement
  
  void EmergencyStop();
  void SetDirection(byte drive, bool direction);
  void Step(byte drive);
  void Disable(byte drive); // There is no drive enable; drives get enabled automatically the first time they are used.
  void SetMotorCurrent(byte drive, float current);
  float DriveStepsPerUnit(int8_t drive);
  void SetDriveStepsPerUnit(int8_t drive, float value);
  float Acceleration(int8_t drive);
  void SetAcceleration(int8_t drive, float value);
  float MaxFeedrate(int8_t drive);
  void SetMaxFeedrate(int8_t drive, float value);
  float InstantDv(int8_t drive);
  float HomeFeedRate(int8_t axis);
  void SetHomeFeedRate(int8_t axis, float value);
  EndStopHit Stopped(int8_t drive);
  float AxisLength(int8_t axis);
  void SetAxisLength(int8_t axis, float value);
  bool HighStopButNotLow(int8_t axis);
  
  float ZProbeStopHeight();
  void SetZProbeStopHeight(float z);
  long ZProbe();
  void SetZProbe(int iZ);
  void SetZProbeType(int iZ);
  
  // Heat and temperature
  
  float GetTemperature(int8_t heater); // Result is in degrees celsius
  void SetHeater(int8_t heater, const float& power); // power is a fraction in [0,1]
  float PidKp(int8_t heater);
  float PidKi(int8_t heater);
  float PidKd(int8_t heater);
  float FullPidBand(int8_t heater);
  float PidMin(int8_t heater);
  float PidMax(int8_t heater);
  float DMix(int8_t heater);
  bool UsePID(int8_t heater);
  float HeatSampleTime();
  void CoolingFan(float speed);
  //void SetHeatOn(int8_t ho); //TEMPORARY - this will go away...

//-------------------------------------------------------------------------------------------------------
  protected:
  
  void ReturnFileStore(FileStore* f);  
  
  private:
  
  float lastTime;
  float longWait;
  float addToTime;
  unsigned long lastTimeCall;
  
  bool active;
  
  Compatibility compatibility;

  void InitialiseInterrupts();
  int GetRawZHeight();
  
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
//  float zProbeGradient;
//  float zProbeConstant;
  long zProbeValue;
  int8_t zProbePin;
  int8_t zProbeCount;
  long zProbeSum;
  int zProbeADValue;
  float zProbeStopHeight;

// AXES

  void PollZHeight();

  float axisLengths[AXES];
  float homeFeedrates[AXES];
  float headOffsets[AXES]; // FIXME - needs a 2D array
//  bool zProbeStarting;
//  float zProbeHigh;
//  float zProbeLow;
  
// HEATERS - Bed is assumed to be the first

  int GetRawTemperature(byte heater);

  int8_t tempSensePins[HEATERS];
  int8_t heatOnPins[HEATERS];
  float thermistorBetas[HEATERS];
  float thermistorSeriesRs[HEATERS];
  float thermistorInfRs[HEATERS];
  bool usePID[HEATERS];
  float pidKis[HEATERS];
  float pidKds[HEATERS];
  float pidKps[HEATERS];
  float fullPidBand[HEATERS];
  float pidMin[HEATERS];
  float pidMax[HEATERS];
  float dMix[HEATERS];
  float heatSampleTime;
  float standbyTemperatures[HEATERS];
  float activeTemperatures[HEATERS];
  int8_t coolingFanPin;
  //int8_t turnHeatOn;

// Serial/USB

  Line* line;
  uint8_t messageIndent;

// Files

  MassStorage* massStorage;
  FileStore* files[MAX_FILES];
  bool fileStructureInitialised;
  //bool* inUse;
  char* webDir;
  char* gcodeDir;
  char* sysDir;
  char* tempDir;
  char* configFile;
  //byte* buf[MAX_FILES];
  //int bPointer[MAX_FILES];
  //char fileList[FILE_LIST_LENGTH];
  //char scratchString[STRING_LENGTH];
  
// Network connection

  Network* network;
  byte ipAddress[4];
  byte netMask[4];
  byte gateWay[4];
};

// Seconds

inline float Platform::Time()
{
  unsigned long now = micros();
  if(now < lastTimeCall) // Has timer overflowed?
	  addToTime += ((float)ULONG_MAX)*TIME_FROM_REPRAP;
  lastTimeCall = now;
  return addToTime + TIME_FROM_REPRAP*(float)now;
}

inline void Platform::Exit()
{
  Message(HOST_MESSAGE, "Platform class exited.\n");
  active = false;
}

inline Compatibility Platform::Emulating()
{
	if(compatibility == reprapFirmware)
		return me;
	return compatibility;
}

inline void Platform::SetEmulating(Compatibility c)
{
	if(c != me && c != reprapFirmware && c != marlin)
	{
		Message(HOST_MESSAGE, "Attempt to emulate unsupported firmware.\n");
		return;
	}
	if(c == reprapFirmware)
		c = me;
	compatibility = c;
}

// Where the htm etc files are

inline char* Platform::GetWebDir()
{
  return webDir;
}

// Where the gcodes are

inline char* Platform::GetGCodeDir()
{
  return gcodeDir;
}

// Where the system files are

inline char* Platform::GetSysDir()
{
  return sysDir;
}

// Where the temporary files are

inline char* Platform::GetTempDir()
{
  return tempDir;
}


inline char* Platform::GetConfigFile()
{
  return configFile;
}



//*****************************************************************************************************************

// Drive the RepRap machine - Movement

inline float Platform::DriveStepsPerUnit(int8_t drive)
{
  return driveStepsPerUnit[drive]; 
}

inline void Platform::SetDriveStepsPerUnit(int8_t drive, float value)
{
  driveStepsPerUnit[drive] = value;
}

inline float Platform::Acceleration(int8_t drive)
{
	return accelerations[drive];
}

inline void Platform::SetAcceleration(int8_t drive, float value)
{
	accelerations[drive] = value;
}

inline float Platform::InstantDv(int8_t drive)
{
  return instantDvs[drive]; 
}

inline bool Platform::HighStopButNotLow(int8_t axis)
{
	return (lowStopPins[axis] < 0)  && (highStopPins[axis] >= 0);
}

inline void Platform::SetDirection(byte drive, bool direction)
{
	if(directionPins[drive] < 0)
		return;
	if(drive == AXES)
		digitalWriteNonDue(directionPins[drive], direction);
	else
		digitalWrite(directionPins[drive], direction);
}

inline void Platform::Disable(byte drive)
{
	if(enablePins[drive] < 0)
		  return;
	if(drive >= Z_AXIS)
		digitalWriteNonDue(enablePins[drive], DISABLE);
	else
		digitalWrite(enablePins[drive], DISABLE);
	driveEnabled[drive] = false;
}

inline void Platform::Step(byte drive)
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

inline void Platform::SetMotorCurrent(byte drive, float current)
{
	unsigned short pot = (unsigned short)(0.256*current*8.0*senseResistor/maxStepperDigipotVoltage);
//	Message(HOST_MESSAGE, "Set pot to: ");
//	snprintf(scratchString, STRING_LENGTH, "%d", pot);
//	Message(HOST_MESSAGE, scratchString);
//	Message(HOST_MESSAGE, "\n");
	mcp.setNonVolatileWiper(potWipes[drive], pot);
	mcp.setVolatileWiper(potWipes[drive], pot);
}

inline float Platform::HomeFeedRate(int8_t axis)
{
  return homeFeedrates[axis];
}

inline void Platform::SetHomeFeedRate(int8_t axis, float value)
{
   homeFeedrates[axis] = value;
}

inline float Platform::AxisLength(int8_t axis)
{
  return axisLengths[axis];
}

inline void Platform::SetAxisLength(int8_t axis, float value)
{
  axisLengths[axis] = value;
}

inline float Platform::MaxFeedrate(int8_t drive)
{
  return maxFeedrates[drive];
}

inline void Platform::SetMaxFeedrate(int8_t drive, float value)
{
	maxFeedrates[drive] = value;
}

inline int Platform::GetRawZHeight()
{
  if(zProbePin >= 0)
    return analogRead(zProbePin);
  return 0;
}

inline long Platform::ZProbe()
{
	return zProbeValue;
}

inline float Platform::ZProbeStopHeight()
{
	return zProbeStopHeight;
}

inline void Platform::SetZProbeStopHeight(float z)
{
	zProbeStopHeight = z;
}

inline void Platform::SetZProbe(int iZ)
{
	zProbeADValue = iZ;
}

inline void Platform::SetZProbeType(int pt)
{
	if(pt != 0)
		zProbePin = Z_PROBE_PIN;
	else
		zProbePin = -1;
}

inline void Platform::PollZHeight()
{
	if(zProbeCount >= 5)
	{
		zProbeValue = zProbeSum/5;
		zProbeSum = 0;
		zProbeCount = 0;
	}
	zProbeSum += GetRawZHeight();
	zProbeCount++;
}


//********************************************************************************************************

// Drive the RepRap machine - Heat and temperature

inline int Platform::GetRawTemperature(byte heater)
{
  if(tempSensePins[heater] >= 0)
    return analogRead(tempSensePins[heater]);
  return 0;
}

inline float Platform::HeatSampleTime()
{
  return heatSampleTime; 
}

inline bool Platform::UsePID(int8_t heater)
{
  return usePID[heater];
}


inline float Platform::PidKi(int8_t heater)
{
  return pidKis[heater]*heatSampleTime;
}

inline float Platform::PidKd(int8_t heater)
{
  return pidKds[heater]/heatSampleTime;
}

inline float Platform::PidKp(int8_t heater)
{
  return pidKps[heater];
}

inline float Platform::FullPidBand(int8_t heater)
{
  return fullPidBand[heater];
}

inline float Platform::PidMin(int8_t heater)
{
  return pidMin[heater];  
}

inline float Platform::PidMax(int8_t heater)
{
  return pidMax[heater]/PidKi(heater);
}

inline float Platform::DMix(int8_t heater)
{
  return dMix[heater];  
}

inline void Platform::CoolingFan(float speed)
{
	if(coolingFanPin < 0)
		return;
	analogWrite(coolingFanPin, (uint8_t)(speed*255.0));
}

//inline void Platform::SetHeatOn(int8_t ho)
//{
//	turnHeatOn = ho;
//}


//*********************************************************************************************************

// Interrupts

inline void Platform::SetInterrupt(float s) // Seconds
{
  if(s <= 0.0)
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

//****************************************************************************************************************

inline Network* Platform::GetNetwork()
{
	return network;
}

inline void Platform::SetIPAddress(byte ip[])
{
	for(uint8_t i = 0; i < 4; i++)
		ipAddress[i] = ip[i];
}

inline byte* Platform::IPAddress()
{
	return ipAddress;
}

inline void Platform::SetNetMask(byte nm[])
{
	for(uint8_t i = 0; i < 4; i++)
		netMask[i] = nm[i];
}

inline byte* Platform::NetMask()
{
	return netMask;
}

inline void Platform::SetGateWay(byte gw[])
{
	for(uint8_t i = 0; i < 4; i++)
		gateWay[i] = gw[i];
}

inline byte* Platform::GateWay()
{
	return gateWay;
}

inline Line* Platform::GetLine()
{
	return line;
}

inline int8_t Line::Status() const
{
//	if(alternateInput != NULL)
//		return alternateInput->Status();
	return numChars == 0 ? nothing : byteAvailable;
}

inline int Line::Read(char& b)
{
//  if(alternateInput != NULL)
//	return alternateInput->Read(b);

  if (numChars == 0) return 0;
  b = buffer[getIndex];
  getIndex = (getIndex + 1) % lineBufsize;
  --numChars;
  return 1;
}

inline void Line::Write(char b)
{
	SerialUSB.print(b);
}

inline void Line::Write(char* b)
{
	SerialUSB.print(b);
}

inline void Line::Write(float f)
{
	snprintf(scratchString, STRING_LENGTH, "%f", f);
	SerialUSB.print(scratchString);
}

inline void Line::Write(long l)
{
	snprintf(scratchString, STRING_LENGTH, "%d", l);
	SerialUSB.print(scratchString);
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

//queries the PHY for link status, true = link is up, false, link is down or there is some other error
inline bool Network::LinkIsUp()
{
	return status_link_up();
}

inline bool Network::Active() const
{
	return active;
}



#endif
