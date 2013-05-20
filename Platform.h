/****************************************************************************************************

RepRapFirmware - Platform: RepRapPro Mendel with Prototype Arduino Due controller

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

Version 0.1

18 November 2012

Adrian Bowyer
RepRap Professional Ltd
http://reprappro.com

Licence: GPL

****************************************************************************************************/

#ifndef PLATFORM_H
#define PLATFORM_H

// Language-specific includes

#include <stdio.h>
#include <ctype.h>

// Platform-specific includes

#include <Arduino.h>


/**************************************************************************************************/

// The physical capabilities of the machine

#define DRIVES 4  // The number of drives in the machine, including X, Y, and Z plus extruder drives
#define AXES 3    // The number of movement axes in the machine, usually just X, Y and Z. <= DRIVES
#define HEATERS 2 // The number of heaters in the machine, including the heated bed if any.

// The numbers of entries in each array must correspond with the values of DRIVES,
// AXES, or HEATERS.  Set values to -1 to flag unavailability.

// DRIVES

#define STEP_PINS {54, 60, 46, 26}
#define DIRECTION_PINS {55, 61, 48, 28}
#define FORWARDS 1     // What to send to go... 
#define BACKWARDS 0    // ...in each direction
#define ENABLE_PINS {38, -1, 62, -1}
#define ENABLE 0      // What to send to enable... 
#define DISABLE 1     // ...and disable a drive
#define DISABLE_DRIVES {false, false, true, false} // Set true to disable a drive when it becomes idle
#define LOW_STOP_PINS {3, 14, 17, -1}
#define HIGH_STOP_PINS {-1, -1, -1, -1}
#define ENDSTOP_HIT 1 // when a stop == this it is hit
#define MAX_FEEDRATES {300, 300, 3, 45}    // mm/sec   
#define MAX_ACCELERATIONS {800, 800, 30, 250}    // mm/sec^2?? Maximum start speed for accelerated moves.
#define DRIVE_STEPS_PER_UNIT {91.4286, 91.4286, 4000, 929}
#define JERKS {15.0, 15.0, 0.4, 15.0}    // (mm/sec)

#define GCODE_LETTERS {'X', 'Y', 'Z', 'E', 'F' }

// AXES


#define AXIS_LENGTHS {210, 210, 120} // mm
#define FAST_HOME_FEEDRATES {50*60, 50*60, 1*60}  // mm/min

#define X_AXIS 0  // The index of the X axis
#define Y_AXIS 1  // The index of the Y axis
#define Z_AXIS 2  // The index of the Z axis

// HEATERS - Bed is assumed to be the first

#define TEMP_SENSE_PINS {10, 9}  // Analogue pin numbers
#define HEAT_ON_PINS {8, 9}
#define THERMISTOR_BETAS {3480.0, 3960.0} // Bed thermistor: RS 484-0149; EPCOS B57550G103J; Extruder thermistor: RS 198-961
#define THERMISTOR_SERIES_RS {4700, 4700} // Ohms in series with the thermistors
#define THERMISTOR_25_RS {10000.0, 100000.0} // Thermistor ohms at 25 C = 298.15 K
#define USE_PID {false, true} // PID or bang-bang for this heater?
#define PID_KIS {-1, 100} // PID constants...
#define PID_KDS {-1, 100}
#define PID_KPS {-1, 100}
#define PID_I_LIMITS {-1, 100} // ... to here
#define TEMP_INTERVAL 0.5 // secs - check and control temperatures this often

#define AD_RANGE 1023.0//16383 // The A->D converter that measures temperatures gives an int this big as its max value

#define HOT_BED 0 // The index of the heated bed; set to -1 if there is no heated bed

/****************************************************************************************************/

// File handling

#define MAX_FILES 7
#define FILE_BUF_LEN 256
#define SD_SPI 4 //Pin
#define WEB_DIR "www/" // Place to find web files on the server
#define GCODE_DIR "gcodes/" // Ditto - g-codes
#define SYS_DIR "sys/" // Ditto - system files
#define TEMP_DIR "tmp/" // Ditto - temporary files
#define FILE_LIST_SEPARATOR ','
#define FILE_LIST_BRACKET '"'
#define FILE_LIST_LENGTH 1000 // Maximum lenght of file list

/****************************************************************************************************/

// Networking

// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network:
#define MAC { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
#define MAC_BYTES 6

#define IP0 192
#define IP1 168
#define IP2 1
#define IP3 14

#define ETH_B_PIN 10

// port 80 is default for HTTP
  
#define HTTP_PORT 80

// Connection statuses - ORed

#define CLIENT 1
#define CONNECTED 2
#define AVAILABLE 4


/****************************************************************************************************/

// Miscellaneous...

#define LED_PIN 13 // Indicator LED

#define BAUD_RATE 115200 // Communication speed of the USB if needed.

#define GCODE_LENGTH 100 // Maximum lenght of internally-generated G Code string


/****************************************************************************************************/

class RepRap;

class Platform
{   
  public:
  
  friend class Move;
  
  Platform(RepRap* r);
  
  RepRap* GetRepRap();
  
//-------------------------------------------------------------------------------------------------------------

// These are the functions that form the interface between Platform and the rest of the firmware.

  void Init(); // Set the machine up after a restart.  If called subsequently this should set the machine up as if
               // it has just been restarted; it can do this by executing an actual restart if you like, but beware the 
               // loop of death...
  void Spin(); // This gets called in the main loop and should do any housekeeping needed
  
  void Exit(); // Shut down tidily.  Calling Init after calling this should reset to the beginning
  
  // Timing
  
  unsigned long Time(); // Returns elapsed microseconds since some arbitrary time
  
  void SetInterrupt(long t); // Set a regular interrupt going every t microseconds; if t is -ve turn interrupt off
  
  void Interrupt(); // The function that the interrupt calls
  
  // Communications and data storage; opening something unsupported returns -1.
  
  char* FileList(char* directory); // Returns a ;-separated list of all the files in the named directory (for example on an SD card).
  int OpenFile(char* fileName, boolean write); // Open a local file (for example on an SD card).
  void GoToEnd(int file); // Position the file at the end (so you can write on the end).
  boolean Read(int file, unsigned char& b);     // Read a single byte from a file into b, 
                                             // returned value is false for EoF, true otherwise
  void WriteString(int file, char* s);  // Write the string to a file.
  void Write(int file, char b);  // Write the byte b to a file.
  unsigned long Length(int file); // File size in bytes
  char* GetWebDir(); // Where the php/htm etc files are
  char* GetGcodeDir(); // Where the gcodes are
  char* GetSysDir();  // Where the system files are
  char* GetTempDir(); // Where temporary files are
  void Close(int file); // Close a file or device, writing any unwritten buffer contents first.
  boolean DeleteFile(char* fileName); // Delete a file
  char* PrependRoot(char* result, char* root, char* fileName);
  
  unsigned char ClientRead(); // Read a byte from the client
  void SendToClient(char* message); // Send string to the host
  void SendToClient(unsigned char b); // Send byte to the host
  int ClientStatus(); // Check client's status
  void DisconnectClient(); //Disconnect the client  
  
  void Message(char type, char* message);        // Send a message.  Messages may simply flash an LED, or, 
                            // say, display the messages on an LCD. This may also transmit the messages to the host. 
  
  // Movement
  
  void SetDirection(byte drive, bool direction);
  void Step(byte drive);
  void Disable(byte drive); // There is no drive enable; drives get enabled automatically the first time they are used.
  void Home(byte axis);
  float DriveStepsPerUnit(char drive);
  
  float ZProbe();  // Return the height above the bed.  Returned value is negative if probing isn't implemented
  void ZProbe(float h); // Move to height h above the bed using the probe (if there is one).  h should be non-negative.
  
  // Heat and temperature
  
  float GetTemperature(char heater); // Result is in degrees celsius
  void SetHeater(char heater, const float& power); // power is a fraction in [0,1]

//-------------------------------------------------------------------------------------------------------
  
  private:
  
  unsigned long lastTime;
  
  boolean active;
  
  // Load settings from local storage
  
  bool LoadFromStore();
  
  int GetRawTemperature(byte heater);
  
  void InitialiseInterrupts();
  
  RepRap* reprap;
  
// DRIVES

  char stepPins[DRIVES];
  char directionPins[DRIVES];
  char enablePins[DRIVES];
  boolean disableDrives[DRIVES];
  char lowStopPins[DRIVES];
  char highStopPins[DRIVES];
  float maxFeedrates[DRIVES];  
  float maxAccelerations[DRIVES];
  float driveStepsPerUnit[DRIVES];
  float jerks[DRIVES];

// AXES

  float axisLengths[AXES];
  float fastHomeFeedrates[AXES];

// HEATERS - Bed is assumed to be the first

  char tempSensePins[HEATERS];
  char heatOnPins[HEATERS];
  float thermistorBetas[HEATERS];
  float thermistorSeriesRs[HEATERS];
  float thermistorInfRs[HEATERS];
  boolean usePid[HEATERS];
  float pidKis[HEATERS];
  float pidKds[HEATERS];
  float pidKps[HEATERS];
  float pidILimits[HEATERS];

// Files

  File* files;
  boolean* inUse;
  char* webDir;
  char* gcodeDir;
  char* sysDir;
  char* tempDir;
  byte* buf[MAX_FILES];
  int bPointer[MAX_FILES];
  char fileList[FILE_LIST_LENGTH];
  
// Network connection

  void ClientMonitor();
  
  byte mac[MAC_BYTES];
  EthernetServer* server;
  EthernetClient client;
  int clientStatus;
};

inline unsigned long Platform::Time()
{
  return micros();
}

inline float Platform::DriveStepsPerUnit(char drive)
{
  return driveStepsPerUnit[drive]; 
}

inline void Platform::Exit()
{
  active = false;
}

inline RepRap* Platform::GetRepRap()
{
  return reprap;
}

// Where the htm etc files are

inline char* Platform::GetWebDir()
{
  return webDir;
}

// Where the gcodes are

inline char* Platform::GetGcodeDir()
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

//*****************************************************************************************************************

// Drive the RepRap machine

inline void Platform::SetDirection(byte drive, bool direction)
{
  digitalWrite(directionPins[drive], direction);  
}

inline void Platform::Step(byte drive)
{
  //digitalWrite(stepPins[drive], !digitalRead(stepPins[drive]));
  digitalWrite(stepPins[drive], 0);
  digitalWrite(stepPins[drive], 1);
}

inline int Platform::GetRawTemperature(byte heater)
{
  return analogRead(tempSensePins[heater]);
}

#endif
