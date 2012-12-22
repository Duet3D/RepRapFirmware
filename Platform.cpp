/****************************************************************************************************

RepRapFirmware - Platform: RepRapPro Mendel with Prototype Arduino Due controller

Platform contains all the code and definitons to deal with machine-dependent things such as control 
pins, bed area, number of extruders, tolerable accelerations and speeds and so on.

-----------------------------------------------------------------------------------------------------

Version 0.1

18 November 2012

Adrian Bowyer
RepRap Professional Ltd
http://reprappro.com

Licence: GPL

****************************************************************************************************/

#include "RepRapFirmware.h"

// Arduino initialise and loop functions
// Put nothing in these other than calls to the RepRap equivalents

void setup()
{
  reprap.init();  
}
  
void loop()
{
  reprap.spin();
}

//*************************************************************************************************

Platform::Platform()
{
  Serial.begin(9600);
  Serial.println("Platform constructor");
  
  if(loadFromStore())
    return;
  
  uint8_t i;
 
// DRIVES

  stepPins = STEP_PINS;
  directionPins = DIRECTION_PINS;
  enablePins = ENABLE_PINS;
  enableOn = ENABLE_ON;
  disableDrives = DISABLE_DRIVES;
  maxFeedrates = MAX_FEEDRATES;
  maxAccelerations = MAX_ACCELERATIONS;
  driveStepsPerUnit = DRIVE_STEPS_PER_UNIT;
  jerks = JERKS;
  driveRelativeModes = DRIVE_RELATIVE_MODES;
  
  for(i = 0; i < DRIVES; i++)
  {
    if(stepPins[i] >= 0)
      pinMode(stepPins[i], OUTPUT);
    if(directionPins[i] >= 0)  
      pinMode(directionPins[i], OUTPUT);
    if(enablePins[i] >= 0)  
      pinMode(enablePins[i], OUTPUT);
  }

// AXES

  lowStopPins = LOW_STOP_PINS;
  highStopPins = HIGH_STOP_PINS;
  endstopsInverting = ENDSTOPS_INVERTING;
  axisLengths = AXIS_LENGTHS;
  fastHomeFeedrates = FAST_HOME_FEEDRATES;

  for(i = 0; i < AXES; i++)
  {
    if(lowStopPins[i] >= 0)
    {
      pinMode(lowStopPins[i], INPUT);
      digitalWrite(lowStopPins[i], HIGH); // Turn on pullup
    }
    if(highStopPins[i] >= 0)
    {
      pinMode(highStopPins[i], INPUT);
      digitalWrite(highStopPins[i], HIGH); // Turn on pullup
    }
  }  
  
// HEATERS - Bed is assumed to be the first

  tempSensePins = TEMP_SENSE_PINS;
  heatOnPins = HEAT_ON_PINS;
  thermistorBetas = THERMISTOR_BETAS;
  thermistorSeriesRs = THERMISTOR_SERIES_RS;
  thermistor25Rs = THERMISTOR_25_RS;
  usePid = USE_PID;
  pidKis = PID_KIS;
  pidKds = PID_KDS;
  pidKps = PID_KPS;
  pidILimits = PID_I_LIMITS;
  
  for(i = 0; i < HEATERS; i++)
  {
    if(tempSensePins[i] >= 0)
      pinMode(tempSensePins[i], INPUT);
    if(heatOnPins[i] >= 0)
      pinMode(heatOnPins[i], OUTPUT);
  }    

}

// Load settings from local storage; return true if successful, false otherwise

bool Platform::loadFromStore()
{
  return false;
}

void Platform::spin()
{
  
}

unsigned long Platform::time()
{
  return micros();
}

void Platform::setDirection(uint8_t drive, bool direction)
{
  digitalWrite(directionPins[drive], direction);  
}

void Platform::step(uint8_t drive)
{
  digitalWrite(stepPins[drive], !digitalRead(stepPins[drive]));
}


//===========================================================================
//=============================Thermal Settings  ============================
//===========================================================================

// See http://en.wikipedia.org/wiki/Thermistor#B_or_.CE.B2_parameter_equation

// BETA is the B value
// RS is the value of the series resistor in ohms
// R_INF is R0.exp(-BETA/T0), where R0 is the thermistor resistance at T0 (T0 is in kelvin)
// Normally T0 is 298.15K (25 C).  If you write that expression in brackets in the #define the compiler 
// should compute it for you (i.e. it won't need to be calculated at run time).

// If the A->D converter has a range of 0..1023 and the measured voltage is V (between 0 and 1023)
// then the thermistor resistance, R = V.RS/(1023 - V)
// and the temperature, T = BETA/ln(R/R_INF)
// To get degrees celsius (instead of kelvin) add -273.15 to T
//#define THERMISTOR_R_INFS ( THERMISTOR_25_RS*exp(-THERMISTOR_BETAS/298.15) ) // Compute in Platform constructor
