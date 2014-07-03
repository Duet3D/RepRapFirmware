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

#define STRING_LENGTH 1029		// needs to be long enough to receive web data
#define SHORT_STRING_LENGTH 40
#define TIME_TO_REPRAP 1.0e6 	// Convert seconds to the units used by the machine (usually microseconds)
#define TIME_FROM_REPRAP 1.0e-6 // Convert the units used by the machine (usually microseconds) to seconds

/**************************************************************************************************/

//// The physical capabilities of the machine
//
//#define DRIVES 4  // The number of drives in the machine, including X, Y, and Z plus extruder drives
//#define AXES 3    // The number of movement axes in the machine, usually just X, Y and Z. <= DRIVES
//#define HEATERS 2 // The number of heaters in the machine; 0 is the heated bed even if there isn't one.
//
//// The numbers of entries in each {} array definition must correspond with the values of DRIVES,
//// AXES, or HEATERS.  Set values to -1 to flag unavailability.  Pins are the microcontroller pin numbers.
//
//// DRIVES
//
//#define STEP_PINS {14, 25, 5, X2}				// Full array for Duet + Duex4 is {14, 25, 5, X2, 41, 39, X4, 49}
//#define DIRECTION_PINS {15, 26, 4, X3}			// Full array for Duet + Duex4 is {15, 26, 4, X3, 35, 53, 51, 48}
//#define FORWARDS true     						// What to send to go...
//#define BACKWARDS false    						// ...in each direction
//#define ENABLE_PINS {29, 27, X1, X0}            // Full array for Duet + Duex4 is {29, 27, X1, X0, 37, X8, 50, 47}
//#define ENABLE false      						// What to send to enable...
//#define DISABLE true     						// ...and disable a drive
//#define DISABLE_DRIVES {false, false, true, false} // Set true to disable a drive when it becomes idle
//#define LOW_STOP_PINS {11, -1, 60, 31}				// Full array endstop pins for Duet + Duex4 is {11, 28, 60, 31, 24, 46, 45, 44}
//#define HIGH_STOP_PINS {-1, 28, -1, -1}
//#define ENDSTOP_HIT 1 							// when a stop == this it is hit
//// Indices for motor current digipots (if any)
////  first 4 are for digipot 1,(on duet)
////  second 4 for digipot 2(on expansion board)
////  Full order is {1, 3, 2, 0, 1, 3, 2, 0}, only include as many as you have DRIVES defined
//#define POT_WIPES {1, 3, 2, 0} 					// Indices for motor current digipots (if any)
//#define SENSE_RESISTOR 0.1   					// Stepper motor current sense resistor (ohms)
//#define MAX_STEPPER_DIGIPOT_VOLTAGE ( 3.3*2.5/(2.7+2.5) ) // Stepper motor current reference voltage
//#define Z_PROBE_AD_VALUE (400)					// Default for the Z probe - should be overwritten by experiment
//#define Z_PROBE_STOP_HEIGHT (0.7) 				// mm
////#define Z_PROBE_PIN (0) 						// Analogue pin number
////#define Z_PROBE_MOD_PIN (61)					// Digital pin number to turn the IR LED on (high) or off (low)
//#define Z_PROBE_PIN (10) 						// Analogue pin number
//#define Z_PROBE_MOD_PIN (52)					// Digital pin number to turn the IR LED on (high) or off (low)
//#define MAX_FEEDRATES {50.0, 50.0, 3.0, 16.0}   // mm/sec
//#define ACCELERATIONS {800.0, 800.0, 10.0, 250.0}    // mm/sec^2
//#define DRIVE_STEPS_PER_UNIT {87.4890, 87.4890, 4000.0, 420.0}
//#define INSTANT_DVS {15.0, 15.0, 0.2, 2.0}    	// (mm/sec)
//#define NUM_MIXING_DRIVES 1; //number of mixing drives

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
#define MAX_FEEDRATES {100.0, 100.0, 3.0, 20.0, 20.0, 20.0, 20.0, 20.0} // mm/sec
#define ACCELERATIONS {500.0, 500.0, 20.0, 250.0, 250.0, 250.0, 250.0, 250.0} // mm/sec^2
#define DRIVE_STEPS_PER_UNIT {87.4890, 87.4890, 4000.0, 420.0, 420.0, 420.0, 420.0, 420.0}
#define INSTANT_DVS {15.0, 15.0, 0.2, 2.0, 2.0, 2.0, 2.0, 2.0} // (mm/sec)
#define NUM_MIXING_DRIVES 1; //number of mixing drives

// AXES

#define AXIS_LENGTHS {220, 200, 200} 			// mm
#define HOME_FEEDRATES {50.0, 50.0, 1.0} 		// mm/sec
#define HEAD_OFFSETS {0.0, 0.0, 0.0}			// mm

#define X_AXIS 0  								// The index of the X axis in the arrays
#define Y_AXIS 1  								// The index of the Y axis
#define Z_AXIS 2  								// The index of the Z axis

#define E0_DRIVE 3 //the index of the first Extruder drive
#define E1_DRIVE 4 //the index of the second Extruder drive
#define E2_DRIVE 5 //the index of the third Extruder drive
#define E3_DRIVE 6 //the index of the fourth Extruder drive
#define E4_DRIVE 7 //the index of the fifth Extruder drive

// HEATERS - The bed is assumed to be the at index 0

//#define TEMP_SENSE_PINS {5, 4}  				// Analogue pin numbers (full array for Duet+Duex4 = {5, 4, 0, 7, 8, 9} )
//#define HEAT_ON_PINS {6, X5}					// PWM pins (full array for Duet+Duex4 = {6, X5, X7, 7, 8, 9} )
//
//// Bed thermistor: http://uk.farnell.com/epcos/b57863s103f040/sensor-miniature-ntc-10k/dp/1299930?Ntt=129-9930
//// Hot end thermistor: http://www.digikey.co.uk/product-search/en?x=20&y=11&KeyWords=480-3137-ND
//#define THERMISTOR_BETAS {3988.0, 4138.0}		// See http://en.wikipedia.org/wiki/Thermistor
//#define THERMISTOR_SERIES_RS {1000, 1000} 		// Ohms in series with the thermistors
//#define THERMISTOR_25_RS {10000.0, 100000.0} 	// Thermistor ohms at 25 C = 298.15 K
//#define USE_PID {false, true} 					// PID or bang-bang for this heater?
//#define PID_KIS {-1, 0.027 / HEAT_SAMPLE_TIME} 	// Integral PID constants, adjusted by dc42 for Ormerod hot end
//#define PID_KDS {-1, 100 * HEAT_SAMPLE_TIME}	// Derivative PID constants
//#define PID_KPS {-1, 20}						// Proportional PID constants
//#define FULL_PID_BAND {-1, 150.0}				// errors larger than this cause heater to be on or off and I-term set to zero
//#define PID_MIN {-1, 0.0}						// minimum value of I-term
//#define PID_MAX {-1, 180}						// maximum value of I-term, must be high enough to reach 245C for ABS printing
//#define D_MIX {-1, 0.5}							// higher values make the PID controller less sensitive to noise in the temperature reading, but too high makes it unstable
//#define TEMP_INTERVAL 0.122 					// secs - check and control temperatures this often
//#define STANDBY_TEMPERATURES {ABS_ZERO, ABS_ZERO} // We specify one for the bed, though it's not needed
//#define ACTIVE_TEMPERATURES {ABS_ZERO, ABS_ZERO}
//#define COOLING_FAN_PIN X6 										//pin D34 is PWM capable but not an Arduino PWM pin - use X6 instead
//#define HEAT_ON 0 								// 0 for inverted heater (eg Duet v0.6) 1 for not (e.g. Duet v0.4)

#define TEMP_SENSE_PINS {5, 4, 0, 7, 8, 9} // Analogue pin numbers
#define HEAT_ON_PINS {6, X5, X7, 7, 8, 9} //pin D38 is PWM capable but not an Arduino PWM pin - //FIXME TEST if E1 PWM works as D38
// Bed thermistor: http://uk.farnell.com/epcos/b57863s103f040/sensor-miniature-ntc-10k/dp/1299930?Ntt=129-9930
// Hot end thermistor: http://www.digikey.co.uk/product-search/en?x=20&y=11&KeyWords=480-3137-ND
#define THERMISTOR_BETAS {3988.0, 4138.0, 4138.0, 4138.0, 4138.0, 4138.0} // Bed thermistor: B57861S104F40; Extruder thermistor: RS 198-961
#define THERMISTOR_SERIES_RS {1000, 1000, 1000, 1000, 1000, 1000} // Ohms in series with the thermistors
#define THERMISTOR_25_RS {10000.0, 100000.0, 100000.0, 100000.0, 100000.0, 100000.0} // Thermistor ohms at 25 C = 298.15 K
#define USE_PID {false, true, true, true, true, true} // PID or bang-bang for this heater?
//#define PID_KIS { 2.2, 0.027 / HEAT_SAMPLE_TIME, 0.027 / HEAT_SAMPLE_TIME, 0.027 / HEAT_SAMPLE_TIME, 0.027 / HEAT_SAMPLE_TIME, 0.027 / HEAT_SAMPLE_TIME} // Integral PID constants, adjusted by dc42 for Ormerod hot end
#define PID_KIS { 2.2, 0.5 / HEAT_SAMPLE_TIME, 0.5 / HEAT_SAMPLE_TIME, 0.5 / HEAT_SAMPLE_TIME, 0.5 / HEAT_SAMPLE_TIME, 0.5 / HEAT_SAMPLE_TIME} // Integral PID constants, adjusted by dc42 for Ormerod hot end
#define PID_KDS {80, 100 * HEAT_SAMPLE_TIME, 100 * HEAT_SAMPLE_TIME, 100 * HEAT_SAMPLE_TIME, 100 * HEAT_SAMPLE_TIME, 100 * HEAT_SAMPLE_TIME}// Derivative PID constants
#define PID_KPS {12, 20, 20, 20, 20, 20} // Proportional PID constants
#define FULL_PID_BAND {150, 150.0, 150.0, 150.0, 150.0, 150.0} // errors larger than this cause heater to be on or off and I-term set to zero
#define PID_MIN {0.0, 0.0, 0.0, 0.0, 0.0, 0.0} // minimum value of I-term
#define PID_MAX {180, 180, 180, 180, 180, 180} // maximum value of I-term, must be high enough to reach 245C for ABS printing
#define D_MIX {0.5, 0.5, 0.5, 0.5, 0.5, 0.5} // higher values make the PID controller less sensitive to noise in the temperature reading, but too high makes it unstable
#define STANDBY_TEMPERATURES {ABS_ZERO, ABS_ZERO, ABS_ZERO, ABS_ZERO, ABS_ZERO, ABS_ZERO} // We specify one for the bed, though it's not needed
#define ACTIVE_TEMPERATURES {ABS_ZERO, ABS_ZERO, ABS_ZERO, ABS_ZERO, ABS_ZERO, ABS_ZERO}
#define COOLING_FAN_PIN X6 //pin D34 is PWM capable but not an Arduino PWM pin - use X6 instead
#define HEAT_ON 0 // 0 for inverted heater (eg Duet v0.6) 1 for not (e.g. Duet v0.4)

#define AD_RANGE 1023.0							//16383 // The A->D converter that measures temperatures gives an int this big as its max value

#define NUMBER_OF_A_TO_D_READINGS_AVERAGED 4	// must be an even number, preferably a power of 2 for performance, and no greater than 64
												// We hope that the compiler is clever enough to spot that division by this is a >> operation, but it doesn't really matter

#define POLL_TIME 0.006                         // Poll the A to D converters this often (seconds)

#define HOT_BED 0 	// The index of the heated bed; set to -1 if there is no heated bed
#define E0_HEATER 1 //the index of the first extruder heater
#define E1_HEATER 2 //the index of the first extruder heater
#define E2_HEATER 3 //the index of the first extruder heater
#define E3_HEATER 4 //the index of the first extruder heater
#define E4_HEATER 5 //the index of the first extruder heater

/****************************************************************************************************/

// File handling

#define MAX_FILES 7								// Maximum number of simultaneously open files
#define FILE_BUF_LEN 256						// File write buffer size
#define SD_SPI 4 								// Pin for the SD card (if any)
#define WEB_DIR "0:/www/" 						// Place to find web files on the SD card
#define GCODE_DIR "0:/gcodes/" 					// Ditto - g-codes
#define SYS_DIR "0:/sys/" 						// Ditto - system files
#define TEMP_DIR "0:/tmp/" 						// Ditto - temporary files
#define FILE_LIST_LENGTH (1000) 				// Maximum length of file list
#define MAX_FILES (42)							// Maximum number of files displayed

/****************************************************************************************************/

// Networking

#define CLIENT_CLOSE_DELAY 0.002				// Seconds to wait after serving a page

#define HTTP_STATE_SIZE 5						// Size of ring buffer used for HTTP requests

#define IP_ADDRESS {192, 168, 1, 10} 			// Need some sort of default...
#define NET_MASK {255, 255, 255, 0}
#define GATE_WAY {192, 168, 1, 1}
#define MAC_ADDRESS {0xBE, 0xEF, 0xDE, 0xAD, 0xFE, 0xED}

// The size of the http output buffer is critical to getting fast load times in the browser.
// If this value is less than the TCP MSS, then Chrome under Windows will delay ack messages by about 120ms,
// which results in very slow page loading. Any value higher than that will cause the TCP packet to be split
// into multiple transmissions, which avoids this behaviour. Using a value of twice the MSS is most efficient because
// each TCP packet will be full.
// Currently we set the MSS (in file network/lwipopts.h) to 1432 which matches the value used by most versions of Windows
// and therefore avoids additional memory use and fragmentation.
const unsigned int httpOutputBufferSize = 2 * 1432;


/****************************************************************************************************/

// Miscellaneous...

#define BAUD_RATE 115200 						// Communication speed of the USB if needed.

const uint16_t lineBufsize = 256;				// use a power of 2 for good performance

/****************************************************************************************************/

enum EndStopHit
{
  noStop = 0,									// no endstop hit
  lowHit = 1,									// low switch hit, or Z-probe in use and above threshold
  highHit = 2									// high stop hit
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

// This class handles the network - typically an ethernet.

// Start with a ring buffer to hold input from the network
// that needs to be responded to.

class NetRing
{
	friend class Network;

protected:

	NetRing(NetRing* n);
	NetRing* Next();						// Next ring entry
	bool Init(char* d, int l,				// Set up a ring entry
			void* pb, void* pc, void* h);
	char* Data();							// Pointer to the data
	int Length();							// How much data
	bool ReadFinished();					// Have we read the data?
	void SetReadFinished();					// Set if we've read the data
	void* Pbuf();							// Ethernet structure pointer that needs to be preserved
	void* Pcb();							// Ethernet structure pointer that needs to be preserved
	void* Hs();								// Ethernet structure pointer that needs to be preserved
	bool Active();							// Is this ring entry live?
	void Free();							// Is this ring entry in use?
	void SetNext(NetRing* n);				// Set the next ring entry - only used at the start
	void ReleasePbuf();						// Set the ethernet structure pointer null
	void ReleaseHs();						// Set the ethernet structure pointer null

private:

	//void Reset();
	void* pbuf;								// Ethernet structure pointer that needs to be preserved
	void* pcb;								// Ethernet structure pointer that needs to be preserved
	void* hs;								// Ethernet structure pointer that needs to be preserved
	char* data;								// Pointer to the data
	int length;								// How much data
	bool read;								// Have we read the data?
	bool active;							// Is this ring entry live?
	NetRing* next;							// Next ring entry
};

// The main network class that drives the network.

class Network
{
public:

	int8_t Status() const;					 // Returns OR of IOStatus
	bool Read(char& b);						 // Called to read a byte from the network
	bool CanWrite() const;					 // Can we send data?
	void SetWriteEnable(bool enable);		 // Set the enabling of data writing
	void SentPacketAcknowledged();			 // Called to tell us a packet has gone
	void Write(char b);						 // Send a byte to the network
	void Write(const char* s);				 // Send a string to the network
	void Close();							 // Close the connection represented by this ring entry
	void ReceiveInput(char* data, int length,// Called to give us some input
			void* pb, void* pc, void* h);
	void InputBufferReleased(void* pb);		 // Called to release the input buffer
	void ConnectionError(void* h);			 // Called when a network error has occured
	bool Active() const;					 // Is the network connection live?
	bool LinkIsUp();						 // Is the network link up?

friend class Platform;

protected:

	Network();
	void Init();
	void Spin();

private:

	void Reset();
	void CleanRing();
	char* inputBuffer;
	char outputBuffer[httpOutputBufferSize];
	int inputPointer;
	int inputLength;
	int outputPointer;
	bool writeEnabled;
	bool closePending;
	int8_t status;
	NetRing* netRingGetPointer;
	NetRing* netRingAddPointer;
	bool active;
	uint8_t sentPacketsOutstanding;		// count of TCP packets we have sent that have not been acknowledged
	uint8_t windowedSendPackets;
};

// This class handles serial I/O - typically via USB

class Line
{
public:

	int8_t Status() const; // Returns OR of IOStatus
	int Read(char& b);
	void Write(char b);
	void Write(const char* s);
	void Write(float f);
	void Write(long l);

friend class Platform;
friend class RepRap;

protected:

	Line();
	void Init();
	void Spin();
	void InjectString(char* string);

private:
	// Although the sam3x usb interface code already has a 512-byte buffer, adding this extra 256-byte buffer
	// increases the speed of uploading to the SD card by 10%
	char buffer[lineBufsize];
	uint16_t getIndex;
	uint16_t numChars;
};

class MassStorage
{
public:

  char* FileList(const char* directory, bool fromLine); // Returns a list of all the files in the named directory
  char* CombineName(const char* directory, const char* fileName);
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
	void Write(char b);
	void Write(const char* s);
	void Close();
	void GoToEnd(); // Position the file at the end (so you can write on the end).
	unsigned long Length(); // File size in bytes

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
  
  Compatibility Emulating() const;

  void SetEmulating(Compatibility c);

  void Diagnostics();
  
  void PrintMemoryUsage();  // Print memory stats for debugging

  void ClassReport(char* className, float &lastTime);  // Called on return to check everything's live.

  // Timing
  
  float Time(); // Returns elapsed seconds since some arbitrary time
  
  void SetInterrupt(float s); // Set a regular interrupt going every s seconds; if s is -ve turn interrupt off
  
  //void DisableInterrupts();

  // Communications and data storage
  
  Network* GetNetwork();
  Line* GetLine() const;
  void SetIPAddress(byte ip[]);
  const byte* IPAddress() const;
  void SetNetMask(byte nm[]);
  const byte* NetMask() const;
  void SetGateWay(byte gw[]);
  const byte* GateWay() const;
  void SetMACAddress(u8_t mac[]);
  const u8_t* MACAddress();
  
  friend class FileStore;
  
  MassStorage* GetMassStorage();
  FileStore* GetFileStore(const char* directory, const char* fileName, bool write);
  void StartNetwork();
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
  float AxisLength(int8_t axis) const;
  void SetAxisLength(int8_t axis, float value);
  bool HighStopButNotLow(int8_t axis) const;
  
  float ZProbeStopHeight() const;
  void SetZProbeStopHeight(float z);
  int ZProbe() const;
  int ZProbeOnVal() const;
  void SetZProbe(int iZ);
  void SetZProbeType(int iZ);
  int GetZProbeType() const;
  //Mixing support
  void SetMixingDrives(int);
  int GetMixingDrives();

  int8_t SlowestDrive();

  // Heat and temperature
  
  float GetTemperature(int8_t heater); // Result is in degrees celsius
  void SetHeater(int8_t heater, const float& power); // power is a fraction in [0,1]
  float PidKp(int8_t heater) const;
  float PidKi(int8_t heater) const;
  float PidKd(int8_t heater) const;
  float FullPidBand(int8_t heater) const;
  float PidMin(int8_t heater) const;
  float PidMax(int8_t heater) const;
  float DMix(int8_t heater) const;
  bool UsePID(int8_t heater) const;
  float HeatSampleTime() const;
  void CoolingFan(float speed);
  void SetPidValues(size_t heater, float pVal, float iVal, float dVal);

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
  int GetRawZHeight() const;
  
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
  int8_t zProbeType;
  bool zModOnThisTime;
  long zProbeOnSum;		// sum of readings taken when IR led is on
  long zProbeOffSum;	// sum of readings taken when IR led is on
  int zProbeADValue;
  float zProbeStopHeight;
  bool zProbeEnable;
  int8_t numMixingDrives;

// AXES

  void InitZProbe();
  void PollZHeight();

  float axisLengths[AXES];
  float homeFeedrates[AXES];
  float headOffsets[AXES]; // FIXME - needs a 2D array
  
// HEATERS - Bed is assumed to be the first

  int GetRawTemperature(byte heater) const;
  void PollTemperatures();

  long tempSum[HEATERS];
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
  u8_t macAddress[6];
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

inline Compatibility Platform::Emulating() const
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

inline int8_t Platform::SlowestDrive()
{
	return slowestDrive;
}

inline const float* Platform::InstantDvs() const
{
  return instantDvs;
}

inline bool Platform::HighStopButNotLow(int8_t axis) const
{
	return (lowStopPins[axis] < 0) && (highStopPins[axis] >= 0);
}

inline void Platform::SetDirection(byte drive, bool direction)
{
	if(directionPins[drive] < 0)
		return;
	if(drive == E0_DRIVE) //DIRECTION_PINS {15, 26, 4, X3, 35, 53, 51, 48}
		digitalWriteNonDue(directionPins[drive], direction);
	else
		digitalWrite(directionPins[drive], direction);
}

inline void Platform::Disable(byte drive)
{
	if(enablePins[drive] < 0)
		  return;
	if(drive == Z_AXIS || drive==E0_DRIVE || drive==E2_DRIVE) //ENABLE_PINS {29, 27, X1, X0, 37, X8, 50, 47}
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
		if(drive == Z_AXIS || drive==E0_DRIVE || drive==E2_DRIVE) //ENABLE_PINS {29, 27, X1, X0, 37, X8, 50, 47}
			digitalWriteNonDue(enablePins[drive], ENABLE);
		else
			digitalWrite(enablePins[drive], ENABLE);
		driveEnabled[drive] = true;
	}
	if(drive == E0_DRIVE || drive == E3_DRIVE) //STEP_PINS {14, 25, 5, X2, 41, 39, X4, 49}
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
	if(drive < 4)
	{
		mcpDuet.setNonVolatileWiper(potWipes[drive], pot);
		mcpDuet.setVolatileWiper(potWipes[drive], pot);
	}
	else
	{
		mcpExpansion.setNonVolatileWiper(potWipes[drive], pot);
		mcpExpansion.setVolatileWiper(potWipes[drive], pot);
	}
}

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

inline int Platform::GetRawZHeight() const
{
  return (zProbeType != 0) ? analogRead(zProbePin) : 0;
}

inline void Platform::PollZHeight()
{
	uint16_t currentReading = GetRawZHeight();

	// We do a moving average of the probe's A to D readings to smooth out noise

	if (zModOnThisTime)
		zProbeOnSum = zProbeOnSum + currentReading - zProbeOnSum/NUMBER_OF_A_TO_D_READINGS_AVERAGED;
	else
		zProbeOffSum = zProbeOffSum + currentReading - zProbeOffSum/NUMBER_OF_A_TO_D_READINGS_AVERAGED;

	if (zProbeType == 2)
	{
		zModOnThisTime = !zModOnThisTime;
		// Reverse the modulation, ready for next time
		digitalWrite(zProbeModulationPin, zModOnThisTime ? HIGH : LOW);
	} else
		zModOnThisTime = true; // Defensive...
}

inline int Platform::ZProbe() const
{
	return (zProbeType == 1)
			? zProbeOnSum/NUMBER_OF_A_TO_D_READINGS_AVERAGED		// non-modulated mode
			: (zProbeType == 2)
			  ? (zProbeOnSum - zProbeOffSum)/NUMBER_OF_A_TO_D_READINGS_AVERAGED	// modulated mode
			    : 0;														// z-probe disabled
}

inline int Platform::ZProbeOnVal() const
{
	return (zProbeType == 1)
			? zProbeOnSum/NUMBER_OF_A_TO_D_READINGS_AVERAGED
			: (zProbeType == 2)
			  ? zProbeOnSum/NUMBER_OF_A_TO_D_READINGS_AVERAGED
				: 0;
}

inline float Platform::ZProbeStopHeight() const
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
	zProbeType = (pt >= 0 && pt <= 2) ? pt : 0;
	InitZProbe();
}

inline int Platform::GetZProbeType() const
{
	return zProbeType;
}

inline void Platform::SetMixingDrives(int num_drives)
{
	if(num_drives>(DRIVES-AXES))
	{
		Message(HOST_MESSAGE, "More mixing extruder drives set with M160 than exist in firmware configuration\n");
		return;
	}
	numMixingDrives = num_drives;
}

inline int Platform::GetMixingDrives()
{
	return numMixingDrives;
}

//********************************************************************************************************

// Drive the RepRap machine - Heat and temperature

inline int Platform::GetRawTemperature(byte heater) const
{
  if(tempSensePins[heater] >= 0)
    return analogRead(tempSensePins[heater]);
  return 0;
}

inline void Platform::PollTemperatures()
{
	// We do a moving average of each thermometer's A to D readings to smooth out noise

	for(int8_t heater = 0; heater < HEATERS; heater++)
		tempSum[heater] = tempSum[heater] + GetRawTemperature(heater) - tempSum[heater]/NUMBER_OF_A_TO_D_READINGS_AVERAGED;
}

inline float Platform::HeatSampleTime() const
{
  return heatSampleTime; 
}

inline bool Platform::UsePID(int8_t heater) const
{
  return usePID[heater];
}


inline float Platform::PidKi(int8_t heater) const
{
  return pidKis[heater]*heatSampleTime;
}

inline float Platform::PidKd(int8_t heater) const
{
  return pidKds[heater]/heatSampleTime;
}

inline float Platform::PidKp(int8_t heater) const
{
  return pidKps[heater];
}

inline float Platform::FullPidBand(int8_t heater) const
{
  return fullPidBand[heater];
}

inline float Platform::PidMin(int8_t heater) const
{
  return pidMin[heater];  
}

inline float Platform::PidMax(int8_t heater) const
{
  return pidMax[heater];
}

inline float Platform::DMix(int8_t heater) const
{
  return dMix[heater];  
}

//Changed to be compatible with existing gcode norms
// M106 S0 = fully off M106 S255 = fully on
inline void Platform::CoolingFan(float speed)
{
	//byte p = (byte)(255.0*fmin(1.0, fmax(0.0, speed))); //this reverts to 0= off, 1 = on if uncommented
	byte p = (byte)speed;
	p = 255 - p; //duet v0.6
	if(coolingFanPin < 0)
		return;
	analogWriteNonDue(coolingFanPin, p);
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

inline const byte* Platform::IPAddress() const
{
	return ipAddress;
}

inline void Platform::SetNetMask(byte nm[])
{
	for(uint8_t i = 0; i < 4; i++)
		netMask[i] = nm[i];
}

inline const byte* Platform::NetMask() const
{
	return netMask;
}

inline void Platform::SetGateWay(byte gw[])
{
	for(uint8_t i = 0; i < 4; i++)
		gateWay[i] = gw[i];
}

inline const byte* Platform::GateWay() const
{
	return gateWay;
}

inline void Platform::SetMACAddress(u8_t mac[])
{
	for(int8_t i = 0; i < 6; i++)
		macAddress[i] = mac[i];
}

inline const byte* Platform::MACAddress()
{
	return macAddress;
}

inline Line* Platform::GetLine() const
{
	return line;
}

inline int8_t Line::Status() const
{
	return numChars == 0 ? nothing : byteAvailable;
}

inline int Line::Read(char& b)
{
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

inline void Line::Write(const char* b)
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
