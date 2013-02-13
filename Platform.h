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

// System #includes

#include <stdio.h>

// Platform specific includes

#include <Arduino.h>
#include <SPI.h>
#include <Ethernet.h>
#include <SD.h>

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
#define ENABLE 0       // What to send to enable... 
#define DISABELE 1     // ...and disable a drive
#define DISABLE_DRIVES {false, false, true, false} // Set true to disable a drive when it becomes idle
#define MAX_FEEDRATES {300, 300, 3, 45}    // mm/sec   
#define MAX_ACCELERATIONS {800, 800, 30, 250}    // mm/sec^2?? Maximum start speed for accelerated moves.
#define DRIVE_STEPS_PER_UNIT {91.4286, 91.4286, 4000, 929}
#define JERKS {15.0, 15.0, 0.4, 15.0}    // (mm/sec)
#define DRIVE_RELATIVE_MODES {false, false, false, true} // false for default absolute movement, true for relative to last position

// AXES

#define LOW_STOP_PINS {3, 14, 17}
#define HIGH_STOP_PINS {-1, -1, -1}
#define ENDSTOPS_INVERTING false // set to true to invert the logic of the endstops (i.e. 0 = hit)
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

// Miscellaneous...

#define LED_PIN 13 // Indicator LED

#define BAUD_RATE 115200 // Communication speed of the USB if needed.

/****************************************************************************************************/

class RepRap;

class Platform
{   
  public:
  
  friend class Heat;
  
  Platform(RepRap* r);
  
//-------------------------------------------------------------------------------------------------------------

// These are the functions that form the interface between Platform and the rest of the firmware.

  void init(); // Set the machine up after a restart.  If called subsequently this should set the machine up as if
               // it has just been restarted; it can do this by executing an actual restart if you like, but beware the 
               // loop of death...
  void spin(); // This gets called in the main loop and should do any housekeeping needed
  
  // Timing
  
  unsigned long time(); // Returns elapsed microseconds since some arbitrary time
  
  void setInterrupt(long t); // Set a regular interrupt going every t microseconds; if t is -ve turn interrupt off
  
  void interrupt(); // The function that the interrupt calls
  
  // Communications and data storage; opening something unsupported returns -1.
  
  char* FileList(); // Returns a comma-separated?? list of all the files on local storage (for example on an SD card).
  int OpenFile(char* fileName, bool write); // Open a local file (for example on an SD card).
  int OpenHost();           // Open a pseudofile that gives read/write communications to the host computer.
  int OpenMessage();        // Open a pseudofile that writes to the message system.  Messages may simply flash an LED, or, 
                            // say, display the messages on an LCD. This may also transmit the messages to the host. 
  int OpenStore(bool write); // Non-volatile non-removable storage such as EEPROM.
  bool Read(int file, unsigned char& b);     // Read a single byte from a file into b, 
                                             // returned value is false for EoF, true otherwise
  void Write(int file, char b);  // Write the byte b to a file.
  void Close(int file); // Close a file or device, writing any unwritten buffer contents first.
  
  // Movement
  
  void setDirection(uint8_t drive, bool direction);
  void step(uint8_t drive);
  void disable(uint8_t drive); // There is no drive enable; drives get enabled automatically the first time they are used.
  void home(uint8_t axis);
  
  // Heat and temperature
  
  float getTemperature(uint8_t heater); // Result is in degrees celsius
  void setHeater(uint8_t heater, const float& power); // power is a fraction in [0,1]

//-------------------------------------------------------------------------------------------------------
  
  private:
  
  // Load settings from local storage
  
  bool loadFromStore();
  
  int getRawTemperature(uint8_t heater);
  
  RepRap* reprap;
  
// DIRIVES

  int8_t stepPins[DRIVES];
  int8_t directionPins[DRIVES];
  int8_t enablePins[DRIVES];
  bool disableDrives[DRIVES];
  float maxFeedrates[DRIVES];  
  float maxAccelerations[DRIVES];
  float driveStepsPerUnit[DRIVES];
  float jerks[DRIVES];
  bool driveRelativeModes[DRIVES];

// AXES

  int8_t lowStopPins[AXES];
  int8_t highStopPins[AXES];
  float axisLengths[AXES];
  float fastHomeFeedrates[AXES];

// HEATERS - Bed is assumed to be the first

  int8_t tempSensePins[HEATERS];
  int8_t heatOnPins[HEATERS];
  float thermistorBetas[HEATERS];
  float thermistorSeriesRs[HEATERS];
  float thermistorInfRs[HEATERS];
  bool usePid[HEATERS];
  float pidKis[HEATERS];
  float pidKds[HEATERS];
  float pidKps[HEATERS];
  float pidILimits[HEATERS];
  
};

inline unsigned long Platform::time()
{
  return micros();
}

inline void Platform::spin()
{
  
}

inline void Platform::setInterrupt(long t)
{
  
}

inline void Platform::interrupt()
{
  reprap->interrupt();  // Put nothing else in here
}


inline void Platform::setDirection(uint8_t drive, bool direction)
{
  digitalWrite(directionPins[drive], direction);  
}

inline void Platform::step(uint8_t drive)
{
  digitalWrite(stepPins[drive], !digitalRead(stepPins[drive]));
}

inline int Platform::getRawTemperature(uint8_t heater)
{
  return analogRead(tempSensePins[heater]);
}

#endif
