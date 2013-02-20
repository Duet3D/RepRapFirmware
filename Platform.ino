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

Platform::Platform(RepRap* r)
{
  reprap = r;
  init();
}

RepRap* Platform::getRepRap()
{
  return reprap;
}

void Platform::init()
{ 
  byte i;
  
  Serial.begin(BAUD_RATE);
  Serial.println("\n\n\nPlatform constructor");
  
  lastTime = time();
  
  if(!loadFromStore())
  {     
  // DRIVES
  
    stepPins = STEP_PINS;
    directionPins = DIRECTION_PINS;
    enablePins = ENABLE_PINS;
    disableDrives = DISABLE_DRIVES;
    maxFeedrates = MAX_FEEDRATES;
    maxAccelerations = MAX_ACCELERATIONS;
    driveStepsPerUnit = DRIVE_STEPS_PER_UNIT;
    jerks = JERKS;
    driveRelativeModes = DRIVE_RELATIVE_MODES;
    
  // AXES
  
    lowStopPins = LOW_STOP_PINS;
    highStopPins = HIGH_STOP_PINS;
    axisLengths = AXIS_LENGTHS;
    fastHomeFeedrates = FAST_HOME_FEEDRATES;
   
  // HEATERS - Bed is assumed to be the first
  
    tempSensePins = TEMP_SENSE_PINS;
    heatOnPins = HEAT_ON_PINS;
    thermistorBetas = THERMISTOR_BETAS;
    thermistorSeriesRs = THERMISTOR_SERIES_RS;
    thermistorInfRs = THERMISTOR_25_RS;
    usePid = USE_PID;
    pidKis = PID_KIS;
    pidKds = PID_KDS;
    pidKps = PID_KPS;
    pidILimits = PID_I_LIMITS;
  }
  
  for(i = 0; i < DRIVES; i++)
  {
    if(stepPins[i] >= 0)
      pinMode(stepPins[i], OUTPUT);
    if(directionPins[i] >= 0)  
      pinMode(directionPins[i], OUTPUT);
    if(enablePins[i] >= 0)
    {  
      pinMode(enablePins[i], OUTPUT);
      digitalWrite(enablePins[i], ENABLE);
    }
  }
  
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
  
  
  for(i = 0; i < HEATERS; i++)
  {
    if(heatOnPins[i] >= 0)
      pinMode(heatOnPins[i], OUTPUT);
    Serial.println(thermistorInfRs[i]);
    thermistorInfRs[i] = ( thermistorInfRs[i]*exp(-thermistorBetas[i]/(25.0 - ABS_ZERO)) );
    Serial.println(thermistorInfRs[i]);
  }  

  // Files
 
  files = new File[MAX_FILES];
  inUse = new boolean[MAX_FILES];
  for(i=0; i < MAX_FILES; i++)
    inUse[i] = false;
  inUse[EEPROM] = true;

  // Network

  mac = MAC;
  server = new EthernetServer(HTTP_PORT);
  
  // disable SD SPI while starting w5100
  // or you will have trouble
  pinMode(SD_SPI, OUTPUT);
  digitalWrite(SD_SPI,HIGH);   

  Ethernet.begin(mac, *(new IPAddress(IP0, IP1, IP2, IP3)));
  server->begin();
  
  //Serial.print("server is at ");
  //Serial.println(Ethernet.localIP());
  
  // this corrects a bug in the Ethernet.begin() function
  // even tho the call to Ethernet.localIP() does the same thing
  digitalWrite(ETH_B_PIN, HIGH);
  
  clientStatus = 0;
  client = 0;
 
  if (!SD.begin(SD_SPI)) 
     Serial.println("SD initialization failed.");
  // SD.begin() returns with the SPI disabled, so you need not disable it here
  

}


// Load settings from local storage; return true if successful, false otherwise

bool Platform::loadFromStore()
{
  return false;
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

// Result is in degrees celsius

float Platform::getTemperature(byte heater)
{
  float r = (float)getRawTemperature(heater);
  //Serial.println(r);
  return ABS_ZERO + thermistorBetas[heater]/log( (r*thermistorSeriesRs[heater]/(AD_RANGE - r))/thermistorInfRs[heater] );
}


// power is a fraction in [0,1]

void Platform::setHeater(byte heater, const float& power)
{
  if(power <= 0)
  {
     digitalWrite(heatOnPins[heater], 0);
     return;
  }
  
  if(power >= 1.0)
  {
     digitalWrite(heatOnPins[heater], 1);
     return;
  }
  
  byte p = (byte)(255.0*power);
  analogWrite(heatOnPins[heater], p);
}


/*********************************************************************************

  Files & Communication
  
*/

// Open a local file (for example on an SD card).

int Platform::OpenFile(char* fileName, boolean write)
{
  int result = -1;
  for(int i=0; i < MAX_FILES; i++)
    if(!inUse[i])
    {
      result = i;
      break;
    }
  if(result < 0)
  {
      Message(HOST_MESSAGE, "Max open file count exceeded.");
      return -1;    
  }
  
  if(!SD.exists(fileName))
  {
    if(!write)
    {
      Message(HOST_MESSAGE, "File not found for reading");
      return -1;
    }
    files[result] = SD.open(fileName, FILE_WRITE);
  } else
  {
    if(write)
      files[result] = SD.open(fileName, FILE_WRITE);
    else
      files[result] = SD.open(fileName, FILE_READ);
  }

  inUse[result] = true;
  return result;
}

void Platform::Close(int file)
{ 
  if(file == EEPROM)
  {
    // ? inUse[file] = false;
    return;
  }
    
  files[file].close();
  inUse[file] = false;
}



boolean Platform::EepromRead(unsigned char& b)
{
  return false;
}

boolean Platform::Read(int file, unsigned char& b)
{
  if(!inUse[file])
  {
    Message(HOST_MESSAGE, "Attempt to read from a non-open file.");
    return false;
  }

  if(file == EEPROM)
    return EepromRead(b);
    
  if(!files[file].available())
    return false;
  b = (unsigned char) files[file].read();
  return true;
}

void Platform::Write(int file, char b)
{
  if(!inUse[file])
  {
    Message(HOST_MESSAGE, "Attempt to write byte to a non-open file.");
    return;
  }
  
  if(file == EEPROM)
  {
    // Do something here
    return;
  }
    
  files[file].write(b);
}

void Platform::WriteString(int file, char* b)
{
  if(!inUse[file])
  {
    Message(HOST_MESSAGE, "Attempt to write string to a non-open file.");
    return;
  }
  
  if(file == EEPROM)
  {
    // Do something here
    return;
  }
  
  files[file].print(b);
}


void Platform::Message(char type, char* message)
{
  switch(type)
  {
  case FLASH_LED:
  // Message that is to flash an LED; the next two bytes define 
  // the frequency and M/S ratio.
  
    break;
  
  case DISPLAY_MESSAGE:  
  // Message that is to appear on a local display;  \f and \n should be supported.
  case HOST_MESSAGE:
  default:
  
    Serial.println(message);
    
  }
}

// Send something to the network client

void Platform::SendToClient(char* message)
{
  if(client)
  {
    client.print(message);
    //Serial.print("Sent: ");
    //Serial.print(message);
  } else
    Message(HOST_MESSAGE, "Attempt to send string to disconnected client.");
}

//***************************************************************************************************




void Platform::spin()
{
   ClientMonitor();
   if(time() - lastTime < 2000000)
     return;
   lastTime = time();
   //Serial.print("Client status: ");
   //Serial.println(clientStatus);
}


















