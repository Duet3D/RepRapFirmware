/****************************************************************************************************

RepRapFirmware - Platform: RepRapPro Mendel with Melzi controller; Arduino/Sanguino toolchain

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

/**************************************************************************************************/

// The physical capabilities of the machine

#define DRIVES 4  // The number of drives in the machine, including X, Y, and Z plus extruder drives
#define AXES 3    // The number of movement axes in the machine, usually just X, Y and Z. <= DRIVES
#define HEATERS 2 // The number of heaters in the machine, including the heated bed if any.

// The numbers of entries in each array must correspond with the values of DRIVES,
// AXES, or HEATERS.  Set values to -1 to flag unavailability.

// DRIVES

#define STEP_PINS {1, 2, 3, 4}
#define DIRECTION_PINS {5, 6, 7, 8}
#define ENABLE_PINS {9, 10, 11, 12}
#define ENABLE_ON {0, 0, 0, 0} // For inverting stepper enable pins (active low) use 0, non inverting (active high) use 1.
#define DISABLE_DRIVES {false, false, true, false} // Set true to disable a drive when it becomes idle
#define MAX_FEEDRATES {300, 300, 3, 45}    // mm/sec   
#define MAX_ACCELERATIONS {800, 800, 30, 250}    // mm/sec^2?? Maximum start speed for accelerated moves.
#define DRIVE_STEPS_PER_UNIT {91.4286, 91.4286, 4000, 929}
#define JERKS {15.0, 15.0, 0.4, 15.0}    // (mm/sec)
#define DRIVE_RELATIVE_MODES {false, false, false, true} // false for default absolute movement, true for relative to last position

// AXES

#define LOW_STOP_PINS {13, 14, 15}
#define HIGH_STOP_PINS {16, 17, 18}
#define ENDSTOPS_INVERTING {false, false, false} // set to true to invert the logic of the endstops; assumes LOW and HIGH behave the same 
#define AXIS_LENGTHS {210, 210, 140} // mm
#define FAST_HOME_FEEDRATES {50*60, 50*60, 1*60}  // mm/min

#define X_AXIS 0  // The index of the X axis
#define Y_AXIS 1  // The index of the Y axis
#define Z_AXIS 2  // The index of the Z axis

// HEATERS

#define TEMP_SENSE_PINS {21, 22}
#define HEAT_ON_PINS {23, 24}
#define THERMISTOR_BETAS {3480.0, 3960.0} // Bed thermistor: RS 484-0149; EPCOS B57550G103J; Extruder thermistor: RS 198-961
#define THERMISTOR_SERIES_RS {4700, 4700} // Ohms in series with the thermistors
#define THERMISTOR_25_RS {10000.0, 100000.0} // Thermistor ohms at 25 C = 298.15 K
#define USE_PID {false, true} // PID or bang-bang for this heater?
#define PID_KIS {-1, 100} // PID constants...
#define PID_KDS {-1, 100}
#define PID_KPS {-1, 100}
#define PID_I_LIMITS {-1, 100} // ... to here
#define TEMP_INTERVAL 0.5 // secs - check and control temperatures this often

#define AD_RANGE 16383 // The A->D converter that measures temperatures gives an int this big as its max value

#define HOT_BED 0 // The index of the heated bed; set to -1 if there is no heated bed


/****************************************************************************************************/

// Miscellaneous...

#define LED_PIN 0 // Indicator LED

#define BAUD_RATE 250000 // Communication speed of the USB if needed.

/****************************************************************************************************/

class Platform
{   
  public:
  
  Platform();
  
//-------------------------------------------------------------------------------------------------------------

// These are the functions that form the interface between Platform and the rest of the firmware.

  void init(); // Set the machine up after a restart.  If called subsequently this should set the machine up as if
               // it has just been restarted; it can do this by executing an actual restart if you like, but beware the 
               // loop of death...
  
  // Communications and data storage; opening something unsupported returns -1.
  
  char* FileList(); // Returns a comma-separated?? list of all the files on local storage (for example on an SD card).
  int OpenFile(char* fileName, bool write); // Open a local file (for example on an SD card).
  int OpenHost();           // Open a pseudofile that gives read/write communications to the host computer.
  int OpenMessage();        // Open a pseudofile that writes to the message system.  Messages may simply flash an LED, or, 
                            // say, display the messages on an LCD. This may also transmit the messages to the host. 
  int OpenStore(bool write); // Non-volatile non-removable storage such as EEPROM.
  void Read(int file, char* string);     // Read printable characters from a file into the string up to the next \n or \r, 
                                         // which are not returned.  Return string[0] == 0 if there's nothing there.
  void Write(int file, char* string);  // Write the 0-terminated string to a file.  End the string with \n or \r if you want them.
  void Close(int file); // Close a file or device, writing any unwritten buffer contents first.
  
  // Movement
  
  void setDirection(int drive, bool forwards);
  void step(int drive);
  void disable(int drive); // There is no drive enable; drives get enabled automatically the first time they are used.
  void home(int axis);
  
  // Heat and temperature
  
  float getTemperature(int heater);
  void setTemperature(int heater, float temperature);

//-------------------------------------------------------------------------------------------------------
  
  private:
  
/*
  
  int8_t stepPins[DRIVES] = STEP_PINS;
  int8_t directionPins[DRIVES] = DIRECTION_PINS;
#define ENABLE_PINS {9, 10,11,12}
#define LOW_STOP_PINS {13, 14, 15, 16}
#define HIGH_STOP_PINS {17, 18, 19, 20}
 */ 
};

extern "C" 
{
  void setup(void);
  void loop(void);
}

#endif
