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
  
  // Files
 
  FIL * files = new FIL[MAX_FILES];
  inUse = new boolean[MAX_FILES];
  for(int8_t i=0; i < MAX_FILES; i++)
    buf[i] = new byte[FILE_BUF_LEN];
  
  server = new EthernetServer(HTTP_PORT);
  
  active = false;
}

//*****************************************************************************************************************

// Interrupts

void TC3_Handler()
{
  TC_GetStatus(TC1, 0);
  reprap.Interrupt();
}

//*******************************************************************************************************************

void Platform::Init()
{ 
  byte i;
  
  SerialUSB.begin(BAUD_RATE);
  
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
    accelerations = ACCELERATIONS;
    driveStepsPerUnit = DRIVE_STEPS_PER_UNIT;
    instantDvs = INSTANT_DVS;
    
  // AXES
  
    axisLengths = AXIS_LENGTHS;
    homeFeedrates = HOME_FEEDRATES;
    headOffsets = HEAD_OFFSETS;
   
  // HEATERS - Bed is assumed to be the first
  
    tempSensePins = TEMP_SENSE_PINS;
    heatOnPins = HEAT_ON_PINS;
    thermistorBetas = THERMISTOR_BETAS;
    thermistorSeriesRs = THERMISTOR_SERIES_RS;
    thermistorInfRs = THERMISTOR_25_RS;
    usePID = USE_PID;
    pidKis = PID_KIS;
    pidKds = PID_KDS;
    pidKps = PID_KPS;
    fullPidBand = FULL_PID_BAND;
    pidMin = PID_MIN;
    pidMax = PID_MAX;
    dMix = D_MIX;
    heatSampleTime = HEAT_SAMPLE_TIME;
    standbyTemperatures = STANDBY_TEMPERATURES;
    activeTemperatures = ACTIVE_TEMPERATURES;   
    
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
    thermistorInfRs[i] = ( thermistorInfRs[i]*exp(-thermistorBetas[i]/(25.0 - ABS_ZERO)) );
  }  

  // Files
 
  for(i=0; i < MAX_FILES; i++)
  {
    bPointer[i] = 0;
    inUse[i] = false;
  }
  
  // Network

  mac = MAC;
//  server = new EthernetServer(HTTP_PORT);
  
  // disable SD SPI while starting w5100
  // or you will have trouble
  pinMode(SD_SPI, OUTPUT);
  digitalWrite(SD_SPI,HIGH);   

  ipAddress = { IP0, IP1, IP2, IP3 };
  //Ethernet.begin(mac, *(new IPAddress(IP0, IP1, IP2, IP3)));
  Ethernet.begin(mac, ipAddress);
  server->begin();
  
  //Serial.print("server is at ");
  //Serial.println(Ethernet.localIP());
  
  // this corrects a bug in the Ethernet.begin() function
  // even tho the call to Ethernet.localIP() does the same thing
  digitalWrite(ETH_B_PIN, HIGH);
  
  clientStatus = 0;
  client = 0;
 
  /* Configure HSMCI pins */
  hsmciPinsinit();
  
  // Initialize SD MMC stack
  sd_mmc_init();
  
  FATFS fs;
  
  memset(&fs, 0, sizeof(FATFS));
  //u_mount SD card
  f_mount (LUN_ID_SD_MMC_0_MEM, NULL);
  //mount SD card

  if (f_mount(LUN_ID_SD_MMC_0_MEM, &fs) != FR_INVALID_DRIVE) {
     SerialUSB.println("SD initialisation failed.");
}
  InitialiseInterrupts();

  lastTime = Time();
  
  active = true;
}

void Platform::Diagnostics() 
{
  Message(HOST_MESSAGE, "Platform Diagnostics:\n"); 
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

float Platform::GetTemperature(int8_t heater)
{
  float r = (float)GetRawTemperature(heater);
  return ABS_ZERO + thermistorBetas[heater]/log( (r*thermistorSeriesRs[heater]/(AD_RANGE - r))/thermistorInfRs[heater] );
}


// power is a fraction in [0,1]

void Platform::SetHeater(int8_t heater, const float& power)
{
  if(heatOnPins[heater] < 0)
    return;
    
  if(power <= 0.00)
  {
     analogWrite(heatOnPins[heater], 0);
     return;
  }
  
  if(power >= 1.0)
  {
     analogWrite(heatOnPins[heater], 255);
     return;
  }
  
  byte p = (byte)(255.0*power);
  analogWrite(heatOnPins[heater], p);
}


/*********************************************************************************

  Files & Communication
  
*/

char* Platform::CombineName(char* result, char* directory, char* fileName)
{
  int out = 0;
  int in = 0;
  
  result[out] = '/';
  out++;
  
  if(directory != NULL)
  {
    if(directory[in] == '/')
      in++;
    while(directory[in] != 0 && directory[in] != '\n' && directory[in] != '/')
    {
      result[out] = directory[in];
      out++;
      in++;
    }
  }
  
  result[out] = '/';
  out++;
  
  in = 0;
  while(fileName[in] != 0 && fileName[in] != '\n' && fileName[in] != '/')
  {
    result[out] = fileName[in];
    out++;
    in++;
  }
  result[out] = 0;
  
  return result;
}

// List the flat files in a directory.  No sub-directories or recursion.

char* Platform::FileList(char* directory)
{
  FILINFO entry;
  DIR dir;
  f_opendir(&dir, directory);
  int p = 0;
  int q;
  int count = 0;
  while(f_readdir(&dir, &entry)==FR_OK)
  {
    q = 0;
    count++;
    fileList[p++] = FILE_LIST_BRACKET;
	fileList[p++] = *entry.fname;
      q++;
      if(p >= FILE_LIST_LENGTH - 10) // Caution...
      {
        Message(HOST_MESSAGE, "FileList - directory: ");
        Message(HOST_MESSAGE, directory);
		Message(HOST_MESSAGE, " has too many files!\n");
        return "";
    }
    fileList[p++] = FILE_LIST_BRACKET;
    fileList[p++] = FILE_LIST_SEPARATOR;
  }
  
  if(count <= 0)
    return "";
  
  fileList[--p] = 0; // Get rid of the last separator
  return fileList;
}

// Delete a file
boolean Platform::DeleteFile(char* directory, char* fileName)
{
  CombineName(scratchString, directory, fileName);
  return f_unlink(scratchString);
}

// Open a local file (for example on an SD card).

int Platform::OpenFile(char* directory, char* fileName, boolean write)
{
  CombineName(scratchString, directory, fileName);
  int result = -1;
  for(int i = 0; i < MAX_FILES; i++)
    if(!inUse[i])
    {
      result = i;
      break;
    }
  if(result < 0)
  {
      Message(HOST_MESSAGE, "Max open file count exceeded.\n");
      return -1;    
  }
  
  int res = f_open(&files[result], scratchString, FILE_READ);
  f_close(&files[result]);
  if(res != FR_OK)
  {
    if(!write)
    {
      Message(HOST_MESSAGE, "File: ");
      Message(HOST_MESSAGE, fileName);
      Message(HOST_MESSAGE, " not found for reading.\n");
      return -1;
    }
    f_open(&files[result], scratchString, FILE_WRITE);
    bPointer[result] = 0;
  } else
  {
    if(write)
    {
    	f_open(&files[result], scratchString, FILE_WRITE);
      bPointer[result] = 0;
    } else
    	f_open(&files[result], scratchString, FILE_WRITE);
  }

  inUse[result] = true;
  return result;
}

void Platform::GoToEnd(int file)
{
  if(!inUse[file])
  {
    Message(HOST_MESSAGE, "Attempt to seek on a non-open file.\n");
    return;
  }
  unsigned long e = files[file].fsize;
  f_lseek(&files[file],e);
}

unsigned long Platform::Length(int file)
{
  if(!inUse[file])
  {
    Message(HOST_MESSAGE, "Attempt to size non-open file.\n");
    return 0;
  }
  return files[file].fsize;
}

void Platform::Close(int file)
{ 
  UINT* bWrit;
  if(bPointer[file] != 0)
    f_write(&files[file],&buf[file],bPointer[file],bWrit);
  bPointer[file] = 0;
  f_close(&files[file]);
  inUse[file] = false;
}


boolean Platform::Read(int file, char& b)
{
  if(!inUse[file])
  {
    Message(HOST_MESSAGE, "Attempt to read from a non-open file.\n");
    return false;
  }
  UINT* rBuf;
  if(f_read(&files[file],&b,1,rBuf) != FR_OK)
//  if(!files[file].available())
    return false;
//  b = (char) files[file].read();
  return true;
}

void Platform::Write(int file, char b)
{
  if(!inUse[file])
  {
    Message(HOST_MESSAGE, "Attempt to write byte to a non-open file.\n");
    return;
  }
  (buf[file])[bPointer[file]] = b;
  bPointer[file]++;
  if(bPointer[file] >= FILE_BUF_LEN)
  {
	  UINT* wBuf;
    f_write(&files[file],&b,1,wBuf);
//  files[file].write(buf[file], FILE_BUF_LEN);
    bPointer[file] = 0;
  } 
  //files[file].write(b);
}

void Platform::WriteString(int file, char* b)
{
  if(!inUse[file])
  {
    Message(HOST_MESSAGE, "Attempt to write string to a non-open file.\n");
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
  } else
    Message(HOST_MESSAGE, "Attempt to send string to disconnected client.\n");
}



void Platform::Message(char type, char* message)
{
  char scratchString[STRING_LENGTH];
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
  
	SerialUSB.println(message);
//    int m = OpenFile(GetWebDir(), MESSAGE_FILE, true);
//	int m = 0;
//    GoToEnd(m);
//    WriteString(m, message);
//    Close(m);
  }
}




//***************************************************************************************************




void Platform::Spin()
{
  if(!active)
    return;
    
   ClientMonitor();
   if(Time() - lastTime < 2.0)
     return;
   lastTime = Time();
}


















