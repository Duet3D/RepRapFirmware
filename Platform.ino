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
  reprap.Init();  
}
  
void loop()
{
  reprap.Spin();
}

//*************************************************************************************************

Platform::Platform(RepRap* r)
{
  reprap = r;
  active = false;
}

//*****************************************************************************************************************

// Interrupts

inline void Platform::SetInterrupt(long t)
{
  
}

inline void Platform::Interrupt()
{
  reprap->Interrupt();  // Put nothing else in this function
}

//***************************************************************************************

// Network connection

inline int Platform::ClientStatus()
{
  return clientStatus;
}

inline void Platform::SendToClient(unsigned char b)
{
  if(client)
  {
    client.write(b);
    if(reprap->GetWebserver()->EchoOutput())
    {
      Serial.write(b);
      //Message(HOST_MESSAGE, message);
      //if(StringEndsWith(message, "\n"))
      //  Message(HOST_MESSAGE, "o: ");
    }
    //Serial.write(b);
  } else
    Message(HOST_MESSAGE, "Attempt to send byte to disconnected client.");
}

inline unsigned char Platform::ClientRead()
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



//*****************************************************************************************************************

// Drive the RepRap machine

inline void Platform::SetDirection(byte drive, bool direction)
{
  digitalWrite(directionPins[drive], direction);  
}

inline void Platform::Step(byte drive)
{
  digitalWrite(stepPins[drive], !digitalRead(stepPins[drive]));
}

inline int Platform::GetRawTemperature(byte heater)
{
  return analogRead(tempSensePins[heater]);
}

//*******************************************************************************************************************

void Platform::Init()
{ 
  byte i;
  
  Serial.begin(BAUD_RATE);
  
  lastTime = Time();
  
  if(!LoadFromStore())
  {     
  // DRIVES
  
    stepPins = STEP_PINS;
    directionPins = DIRECTION_PINS;
    enablePins = ENABLE_PINS;
    disableDrives = DISABLE_DRIVES;
    lowStopPins = LOW_STOP_PINS;
    highStopPins = HIGH_STOP_PINS;
    maxFeedrates = MAX_FEEDRATES;
    maxAccelerations = MAX_ACCELERATIONS;
    driveStepsPerUnit = DRIVE_STEPS_PER_UNIT;
    jerks = JERKS;
    driveRelativeModes = DRIVE_RELATIVE_MODES;
    
  // AXES
  
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
    webDir = WEB_DIR;
    gcodeDir = GCODE_DIR;
    sysDir = SYS_DIR;
    tempDir = TEMP_DIR;
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
    //Serial.println(thermistorInfRs[i]);
    thermistorInfRs[i] = ( thermistorInfRs[i]*exp(-thermistorBetas[i]/(25.0 - ABS_ZERO)) );
    //Serial.println(thermistorInfRs[i]);
  }  

  // Files
 
  files = new File[MAX_FILES];
  inUse = new boolean[MAX_FILES];
  for(i=0; i < MAX_FILES; i++)
  {
    buf[i] = new byte[FILE_BUF_LEN];
    bPointer[i] = 0;
    inUse[i] = false;
  }
  
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
  
    // Reinitialise the message file
  
  DeleteFile(PrependRoot(GetWebDir(), MESSAGE_FILE));
  int m = OpenFile(PrependRoot(GetWebDir(), MESSAGE_TEMPLATE), false);
  int n = OpenFile(PrependRoot(GetWebDir(), MESSAGE_FILE), true);
  byte b;
  while (Read(m, b))
    Write(n,b);
  Close(m);  
  Close(n);
  
  active = true;
}

void Platform::Exit()
{
  active = false;
}

RepRap* Platform::GetRepRap()
{
  return reprap;
}


char* Platform::PrependRoot(char* root, char* fileName)
{
  strcpy(scratchString, root);
  return strcat(scratchString, fileName);
}


// Load settings from local storage; return true if successful, false otherwise

bool Platform::LoadFromStore()
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

float Platform::GetTemperature(byte heater)
{
  float r = (float)GetRawTemperature(heater);
  //Serial.println(r);
  return ABS_ZERO + thermistorBetas[heater]/log( (r*thermistorSeriesRs[heater]/(AD_RANGE - r))/thermistorInfRs[heater] );
}


// power is a fraction in [0,1]

void Platform::SetHeater(byte heater, const float& power)
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

// List the flat files in a directory.  No sub-directories or recursion.

char* Platform::FileList(char* directory)
{
  File dir, entry;
  dir = SD.open(directory);
  int p = 0;
  int q;
  int count = 0;
  while(entry = dir.openNextFile())
  {
    q = 0;
    count++;
    fileList[p++] = FILE_LIST_BRACKET;
    while(entry.name()[q])
    {
      fileList[p++] = entry.name()[q];
      q++;
      if(p >= FILE_LIST_LENGTH - 10) // Caution...
      {
        Message(HOST_MESSAGE, "FileList - directory: ");
        Message(HOST_MESSAGE, directory);
        Message(HOST_MESSAGE, " has too many files!<br>\n");
        return "";
      }
    }
    fileList[p++] = FILE_LIST_BRACKET;
    fileList[p++] = FILE_LIST_SEPARATOR;
    entry.close();
  }
  dir.close();
  
  if(count <= 0)
    return "";
  
  fileList[--p] = 0; // Get rid of the last separator
  return fileList;
}

// Delete a file
boolean Platform::DeleteFile(char* fileName)
{
  return SD.remove(fileName);
}

// Open a local file (for example on an SD card).

int Platform::OpenFile(char* fileName, boolean write)
{
  int result = -1;
  for(int i = 0; i < MAX_FILES; i++)
    if(!inUse[i])
    {
      result = i;
      break;
    }
  if(result < 0)
  {
      Message(HOST_MESSAGE, "Max open file count exceeded.<br>\n");
      return -1;    
  }
  
  if(!SD.exists(fileName))
  {
    if(!write)
    {
      Message(HOST_MESSAGE, "File: ");
      Message(HOST_MESSAGE, fileName);
      Message(HOST_MESSAGE, " not found for reading.<br>\n");
      return -1;
    }
    files[result] = SD.open(fileName, FILE_WRITE);
    bPointer[result] = 0;
  } else
  {
    if(write)
    {
      files[result] = SD.open(fileName, FILE_WRITE);
      bPointer[result] = 0;
    } else
      files[result] = SD.open(fileName, FILE_READ);
  }

  inUse[result] = true;
  return result;
}

void Platform::GoToEnd(int file)
{
  if(!inUse[file])
  {
    Message(HOST_MESSAGE, "Attempt to seek on a non-open file.<br>\n");
    return;
  }
  unsigned long e = files[file].size();
  files[file].seek(e);
}

unsigned long Platform::Length(int file)
{
  if(!inUse[file])
  {
    Message(HOST_MESSAGE, "Attempt to size non-open file.<br>\n");
    return 0;
  }
  return files[file].size();  
}

void Platform::Close(int file)
{ 
  if(bPointer[file] != 0)
    files[file].write(buf[file], bPointer[file]);
  bPointer[file] = 0;
  files[file].close();
  inUse[file] = false;
}


boolean Platform::Read(int file, unsigned char& b)
{
  if(!inUse[file])
  {
    Message(HOST_MESSAGE, "Attempt to read from a non-open file.<br>\n");
    return false;
  }
    
  if(!files[file].available())
    return false;
  b = (unsigned char) files[file].read();
  return true;
}

void Platform::Write(int file, char b)
{
  if(!inUse[file])
  {
    Message(HOST_MESSAGE, "Attempt to write byte to a non-open file.<br>\n");
    return;
  }
  (buf[file])[bPointer[file]] = b;
  bPointer[file]++;
  if(bPointer[file] >= FILE_BUF_LEN)
  {
    files[file].write(buf[file], FILE_BUF_LEN);
    bPointer[file] = 0;
  } 
  //files[file].write(b);
}

void Platform::WriteString(int file, char* b)
{
  if(!inUse[file])
  {
    Message(HOST_MESSAGE, "Attempt to write string to a non-open file.<br>\n");
    return;
  }
  int i = 0;
  while(b[i])
    Write(file, b[i++]); 
  //files[file].print(b);
}

// Send something to the network client

void Platform::SendToClient(char* message)
{
  if(client)
  {
    client.print(message);
    if(reprap->GetWebserver()->EchoOutput())
    {
      Serial.print(message);
      //Message(HOST_MESSAGE, message);
      //if(StringEndsWith(message, "\n"))
      //  Message(HOST_MESSAGE, "o: ");
    }
  } else
    Message(HOST_MESSAGE, "Attempt to send string to disconnected client.<br>\n");
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
  
  
    int m = OpenFile(PrependRoot(GetWebDir(), MESSAGE_FILE), true);
    GoToEnd(m);
    WriteString(m, message);
    Serial.print(message);
    Close(m);
    
  }
}

// Where the php/htm etc files are

char* Platform::GetWebDir()
{
  return webDir;
}

// Where the gcodes are

char* Platform::GetGcodeDir()
{
  return gcodeDir;
}

// Where the system files are

char* Platform::GetSysDir()
{
  return sysDir;
}

// Where the temporary files are

char* Platform::GetTempDir()
{
  return tempDir;
}


//***************************************************************************************************




void Platform::Spin()
{
  if(!active)
    return;
    
   ClientMonitor();
   if(Time() - lastTime < 2000000)
     return;
   lastTime = Time();
}


















