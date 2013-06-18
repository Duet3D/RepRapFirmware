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

// Some numbers...

#define STRING_LENGTH 1000
#define TIME_TO_REPRAP 1.0e6 // Convert seconds to the units used by the machine (usually microseconds)
#define TIME_FROM_REPRAP 1.0e-6 // Convert the units used by the machine (usually microseconds) to seconds

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
#define FORWARDS true     // What to send to go... 
#define BACKWARDS false    // ...in each direction
#define ENABLE_PINS {38, -1, 62, -1}
#define ENABLE false      // What to send to enable... 
#define DISABLE true     // ...and disable a drive
#define DISABLE_DRIVES {false, false, true, false} // Set true to disable a drive when it becomes idle
#define LOW_STOP_PINS {3, 14, 17, -1}
#define HIGH_STOP_PINS {-1, -1, -1, -1}
#define ENDSTOP_HIT 1 // when a stop == this it is hit
#define MAX_FEEDRATES {300.0, 300.0, 3.0, 45.0}    // mm/sec   
#define ACCELERATIONS {800.0, 800.0, 30.0, 250.0}    // mm/sec^2??
//#define ACCELERATIONS {80, 80, 3, 25} 
#define DRIVE_STEPS_PER_UNIT {91.4286, 91.4286, 4000.0, 929.0}
#define INSTANT_DVS {15.0, 15.0, 0.4, 15.0}    // (mm/sec)
#define GCODE_LETTERS { 'X', 'Y', 'Z', 'E', 'F' } // The drives and feedrate in a GCode

// AXES

#define START_FEED_RATE 200.0

#define AXIS_LENGTHS {210, 200, 120} // mm
#define HOME_FEEDRATES {50.0*60.0, 50.0*60.0, 1.0*60.0}  // mm/min
#define HEAD_OFFSETS {0.0, 0.0, 0.0}

#define X_AXIS 0  // The index of the X axis
#define Y_AXIS 1  // The index of the Y axis
#define Z_AXIS 2  // The index of the Z axis


// HEATERS - Bed is assumed to be the first

#define TEMP_SENSE_PINS {10, 9}  // Analogue pin numbers
#define HEAT_ON_PINS {8, 9}
#define THERMISTOR_BETAS {3480.0, 3960.0} // Bed thermistor: RS 484-0149; EPCOS B57550G103J; Extruder thermistor: RS 198-961
#define THERMISTOR_SERIES_RS {4700, 4700} // Ohms in series with the thermistors
#define THERMISTOR_25_RS {10000.0, 100000.0} // Thermistor ohms at 25 C = 298.15 K
#define USE_PID {false, false} // PID or bang-bang for this heater?
#define PID_KIS {-1, 100} // PID constants...
#define PID_KDS {-1, 100}
#define PID_KPS {-1, 100}
#define PID_I_LIMITS {-1, 100} // ... to here
#define TEMP_INTERVAL 0.5 // secs - check and control temperatures this often
#define STANDBY_TEMPERATURES {0.0, 0.0} // We specify one for the bed, though it's not needed
#define ACTIVE_TEMPERATURES {0.0, 0.0}

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

#define FLASH_LED 'F' // Type byte of a message that is to flash an LED; the next two bytes define 
                      // the frequency and M/S ratio.
#define DISPLAY_MESSAGE 'L'  // Type byte of a message that is to appear on a local display; the L is 
                             // not displayed; \f and \n should be supported.
#define HOST_MESSAGE 'H' // Type byte of a message that is to be sent to the host; the H is not sent.

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

#define IP_BYTES 4

#define ETH_B_PIN 10

// port 80 is default for HTTP
  
#define HTTP_PORT 80

// Connection statuses - ORed

#define CLIENT 1
#define CONNECTED 2
#define AVAILABLE 4

// Seconds to wait after serving a page
 
#define CLIENT_CLOSE_DELAY 0.001


/****************************************************************************************************/

// Miscellaneous...

#define LED_PIN 13 // Indicator LED

#define BAUD_RATE 115200 // Communication speed of the USB if needed.

/****************************************************************************************************/

enum EndStopHit
{
  noStop = 0,
  lowHit = 1,
  highHit = 2
};

class Platform
{   
  public:
  
  Platform(RepRap* r);
  
//-------------------------------------------------------------------------------------------------------------

// These are the functions that form the interface between Platform and the rest of the firmware.

  void Init(); // Set the machine up after a restart.  If called subsequently this should set the machine up as if
               // it has just been restarted; it can do this by executing an actual restart if you like, but beware the 
               // loop of death...
  void Spin(); // This gets called in the main loop and should do any housekeeping needed
  
  void Exit(); // Shut down tidily.  Calling Init after calling this should reset to the beginning
  
  // Timing
  
  float Time(); // Returns elapsed seconds since some arbitrary time
  
  void SetInterrupt(float s); // Set a regular interrupt going every s seconds; if s is -ve turn interrupt off
  
  // Communications and data storage; opening something unsupported returns -1.
  
  char* FileList(char* directory); // Returns a ,-separated list of all the files in the named directory (for example on an SD card).
  //int OpenFile(char* fileName, boolean write); // Open a local file (for example on an SD card).
  int OpenFile(char* directory, char* fileName, boolean write); // Open a local file (for example on an SD card).
  void GoToEnd(int file); // Position the file at the end (so you can write on the end).
  boolean Read(int file, char& b);     // Read a single byte from a file into b, 
                                             // returned value is false for EoF, true otherwise
  void WriteString(int file, char* s);  // Write the string to a file.
  void Write(int file, char b);  // Write the byte b to a file.
  unsigned long Length(int file); // File size in bytes
  char* GetWebDir(); // Where the htm etc files are
  char* GetGCodeDir(); // Where the gcodes are
  char* GetSysDir();  // Where the system files are
  char* GetTempDir(); // Where temporary files are
  void Close(int file); // Close a file or device, writing any unwritten buffer contents first.
  boolean DeleteFile(char* directory, char* fileName); // Delete a file
  
  char ClientRead(); // Read a byte from the client
  void SendToClient(char* message); // Send string to the host
  void SendToClient(char b); // Send byte to the host
  int ClientStatus(); // Check client's status
  void DisconnectClient(); //Disconnect the client  
  
  void Message(char type, char* message);        // Send a message.  Messages may simply flash an LED, or, 
                            // say, display the messages on an LCD. This may also transmit the messages to the host. 
  
  // Movement
  
  void SetDirection(byte drive, bool direction);
  void Step(byte drive);
  void Disable(byte drive); // There is no drive enable; drives get enabled automatically the first time they are used.
  float DriveStepsPerUnit(int8_t drive);
  float Acceleration(int8_t drive);
  float InstantDv(int8_t drive);
  float HomeFeedRate(int8_t drive);
  EndStopHit Stopped(int8_t drive);
  float AxisLength(int8_t drive);
  
  float ZProbe();  // Return the height above the bed.  Returned value is negative if probing isn't implemented
  void ZProbe(float h); // Move to height h above the bed using the probe (if there is one).  h should be non-negative.
  
  // Heat and temperature
  
  float GetTemperature(int8_t heater); // Result is in degrees celsius
  void SetHeater(int8_t heater, const float& power); // power is a fraction in [0,1]
  void SetStandbyTemperature(int8_t heater, const float& t);
  void SetActiveTemperature(int8_t heater, const float& t);
  float StandbyTemperature(int8_t heater);
  float ActiveTemperature(int8_t heater);  
  float pidKp(int8_t heater);
  float pidKi(int8_t heater);
  float pidKd(int8_t heater);
  float pidKw(int8_t heater);
  boolean UsePID(int8_t heater);
  float HeatSampleTime();

//-------------------------------------------------------------------------------------------------------
  
  private:
  
  float lastTime;
  
  boolean active;
  
  // Load settings from local storage
  
  bool LoadFromStore();
  
  int GetRawTemperature(byte heater);
  
  void InitialiseInterrupts();
  
  char* CombineName(char* result, char* directory, char* fileName);
  
  RepRap* reprap;
  
// DRIVES

  int8_t stepPins[DRIVES];
  int8_t directionPins[DRIVES];
  int8_t enablePins[DRIVES];
  boolean disableDrives[DRIVES];
  int8_t lowStopPins[DRIVES];
  int8_t highStopPins[DRIVES];
  float maxFeedrates[DRIVES];  
  float accelerations[DRIVES];
  float driveStepsPerUnit[DRIVES];
  float instantDvs[DRIVES];

// AXES

  float axisLengths[AXES];
  float homeFeedrates[AXES];
  float headOffsets[AXES]; // FIXME - needs a 2D array
  
// HEATERS - Bed is assumed to be the first

  int8_t tempSensePins[HEATERS];
  int8_t heatOnPins[HEATERS];
  float thermistorBetas[HEATERS];
  float thermistorSeriesRs[HEATERS];
  float thermistorInfRs[HEATERS];
  boolean usePID[HEATERS];
  float pidKis[HEATERS];
  float pidKds[HEATERS];
  float pidKps[HEATERS];
  float pidILimits[HEATERS];
  float heatSampleTime;
  float standbyTemperatures[HEATERS];
  float activeTemperatures[HEATERS];

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
  char scratchString[STRING_LENGTH];
  
// Network connection

  void ClientMonitor();
  
  byte mac[MAC_BYTES];
  byte ipAddress[IP_BYTES];
  EthernetServer* server;
  EthernetClient client;
  int clientStatus;
};

inline float Platform::Time()
{
  return TIME_FROM_REPRAP*(float)micros();
}

inline void Platform::Exit()
{
  active = false;
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

//*****************************************************************************************************************

// Drive the RepRap machine - Movement

inline float Platform::DriveStepsPerUnit(int8_t drive)
{
  return driveStepsPerUnit[drive]; 
}

inline float Platform::Acceleration(int8_t drive)
{
  return accelerations[drive]; 
}

inline float Platform::InstantDv(int8_t drive)
{
  return instantDvs[drive]; 
}

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

inline float Platform::HomeFeedRate(int8_t drive)
{
  return homeFeedrates[drive];
}

inline EndStopHit Platform::Stopped(int8_t drive)
{
  if(lowStopPins[drive] >= 0)
  {
    if(digitalRead(lowStopPins[drive]) == ENDSTOP_HIT)
      return lowHit;
  }
  if(highStopPins[drive] >= 0)
  {
    if(digitalRead(highStopPins[drive]) == ENDSTOP_HIT)
      return highHit;
  }
  return noStop; 
}

inline float Platform::AxisLength(int8_t drive)
{
  return axisLengths[drive];
}

//********************************************************************************************************

// Drive the RepRap machine - Heat and temperature

inline int Platform::GetRawTemperature(byte heater)
{
  return analogRead(tempSensePins[heater]);
}

inline float Platform::HeatSampleTime()
{
  return heatSampleTime; 
}

inline boolean Platform::UsePID(int8_t heater)
{
  return usePID[heater];
}

inline void Platform::SetStandbyTemperature(int8_t heater, const float& t)
{
  standbyTemperatures[heater] = t;  
}

inline void Platform::SetActiveTemperature(int8_t heater, const float& t)
{
  activeTemperatures[heater] = t;  
}

inline float Platform::StandbyTemperature(int8_t heater)
{
  return standbyTemperatures[heater];
}

inline float Platform::ActiveTemperature(int8_t heater)
{
  return activeTemperatures[heater];
}

//*********************************************************************************************************

// Interrupts


void Platform::InitialiseInterrupts()
{
  pmc_set_writeprotect(false);
  pmc_enable_periph_clk((uint32_t)TC3_IRQn);
  TC_Configure(TC1, 0, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK4);
  TC1->TC_CHANNEL[0].TC_IER=TC_IER_CPCS;
  TC1->TC_CHANNEL[0].TC_IDR=~TC_IER_CPCS;
  SetInterrupt(STANDBY_INTERRUPT_RATE); 
}

inline void Platform::SetInterrupt(float s) // Seconds
{
  if(s <= 0.0)
  {
    NVIC_DisableIRQ(TC3_IRQn);
    return;
  }
  uint32_t rc = (uint32_t)( (((long)(TIME_TO_REPRAP*s))*84l)/128l );
  TC_SetRA(TC1, 0, rc/2); //50% high, 50% low
  TC_SetRC(TC1, 0, rc);
  TC_Start(TC1, 0);
  NVIC_EnableIRQ(TC3_IRQn);
}

//***************************************************************************************

// Network connection

inline int Platform::ClientStatus()
{
  return clientStatus;
}

inline void Platform::SendToClient(char b)
{
  if(client)
  {
    client.write(b);
  } else
    Message(HOST_MESSAGE, "Attempt to send byte to disconnected client.");
}

inline char Platform::ClientRead()
{
  if(client)
    return client.read();
    
  Message(HOST_MESSAGE, "Attempt to read from disconnected client.");
  return '\n'; // good idea?? 
}

inline void Platform::ClientMonitor()
{
  clientStatus = 0;
  
  if(!client)
  {
    client = server->available();
    if(!client)
      return;
    //else
      //Serial.println("new client");
  }
    
  clientStatus |= CLIENT;
    
  if(!client.connected())
    return;
    
  clientStatus |= CONNECTED;
    
  if (!client.available())
    return;
    
  clientStatus |= AVAILABLE;
}

inline void Platform::DisconnectClient()
{
  if (client)
  {
    client.stop();
    //Serial.println("client disconnected");
  } else
      Message(HOST_MESSAGE, "Attempt to disconnect non-existent client.");
}



#endif
