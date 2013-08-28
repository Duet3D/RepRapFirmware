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
  //reprap.GetMove()->InterruptTime();  // Uncomment this line to time the interrupt routine on startup
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
 
  //files = new File[MAX_FILES];
  //inUse = new bool[MAX_FILES];
  
  massStorage = new MassStorage(this);
  
  for(int8_t i=0; i < MAX_FILES; i++)
    files[i] = new FileStore(this);
    
    //buf[i] = new byte[FILE_BUF_LEN];
  
  line = new Line();
  network = new Network();
  
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
    configFile = CONFIG_FILE;
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
  
  for(i=0; i < MAX_FILES; i++)
    files[i]->Init();
 
//  for(i=0; i < MAX_FILES; i++)
//  {
//    bPointer[i] = 0;
//    inUse[i] = false;
//  }
  
  line->Init();

  network->Init();

  massStorage->Init();
  
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

MassStorage::MassStorage(Platform* p)
{
   platform = p;
}

void MassStorage::Init()
{
  if (!SD.begin(SD_SPI)) 
     platform->Message(HOST_MESSAGE, "SD initialization failed.\n");
  // SD.begin() returns with the SPI disabled, so you need not disable it here  
}

char* MassStorage::CombineName(char* directory, char* fileName)
{
  int out = 0;
  int in = 0;
  
  scratchString[out] = '/';
  out++;
  
  if(directory != NULL)
  {
    if(directory[in] == '/')
      in++;
    while(directory[in] != 0 && directory[in] != '\n' && directory[in] != '/')
    {
      scratchString[out] = directory[in];
      in++;
      out++;
      if(out >= STRING_LENGTH)
      {
         platform->Message(HOST_MESSAGE, "CombineName() buffer overflow.");
         out = 0;
      }
    }
  }
  
  scratchString[out] = '/';
  out++;
  
  in = 0;
  while(fileName[in] != 0 && fileName[in] != '\n' && fileName[in] != '/')
  {
    scratchString[out] = fileName[in];
    in++;
    out++;
    if(out >= STRING_LENGTH)
    {
       platform->Message(HOST_MESSAGE, "CombineName() buffer overflow.");
       out = 0;
    }
  }
  scratchString[out] = 0;
  
  return scratchString;
}

// List the flat files in a directory.  No sub-directories or recursion.

char* MassStorage::FileList(char* directory)
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
        platform->Message(HOST_MESSAGE, "FileList - directory: ");
        platform->Message(HOST_MESSAGE, directory);
        platform->Message(HOST_MESSAGE, " has too many files!\n");
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
bool MassStorage::Delete(char* directory, char* fileName)
{
  return SD.remove(CombineName(directory, fileName));
}

//------------------------------------------------------------------------------------------------


FileStore::FileStore(Platform* p)
{
   platform = p;  
}


void FileStore::Init()
{
  bPointer = 0;
  inUse = false;  
}


void FileStore::Close()
{
  if(bPointer != 0)
    file.write(buf, bPointer);
  bPointer = 0;
  file.close();
  platform->ReturnFileStore(this);
  inUse = false;
}

// Open a local file (for example on an SD card).
// This is protected - only Platform can access it.

bool FileStore::Open(char* directory, char* fileName, bool write)
{
  char* location = platform->GetMassStorage()->CombineName(directory, fileName);
  
  if(!SD.exists(location))
  {
    if(!write)
    {
      platform->Message(HOST_MESSAGE, "File: ");
      platform->Message(HOST_MESSAGE, fileName);
      platform->Message(HOST_MESSAGE, " not found for reading.\n");
      return false;
    }
    file = SD.open(location, FILE_WRITE);
    bPointer = 0;
  } else
  {
    if(write)
    {
      file = SD.open(location, FILE_WRITE);
      bPointer = 0;
    } else
      file = SD.open(location, FILE_READ);
  }

  inUse = true;
  return true;
}

void FileStore::GoToEnd()
{
  if(!inUse)
  {
    platform->Message(HOST_MESSAGE, "Attempt to seek on a non-open file.\n");
    return;
  }
  unsigned long e = file.size();
  file.seek(e);
}

unsigned long FileStore::Length()
{
  if(!inUse)
  {
    platform->Message(HOST_MESSAGE, "Attempt to size non-open file.\n");
    return 0;
  }
  return file.size();  
}

int8_t FileStore::Status()
{
  if(!inUse)
    return nothing;
    
  if(file.available())
    return byteAvailable;
    
  return nothing;
}

bool FileStore::Read(char& b)
{
  if(!inUse)
  {
    platform->Message(HOST_MESSAGE, "Attempt to read from a non-open file.\n");
    return false;
  }
    
  if(!(Status() & byteAvailable))
    return false;
  int c = file.read();
  if(c < 0)
    return false;
    
  b = (char) c;
  return true;
}

void FileStore::Write(char b)
{
  if(!inUse)
  {
    platform->Message(HOST_MESSAGE, "Attempt to write byte to a non-open file.\n");
    return;
  }
  buf[bPointer] = b;
  bPointer++;
  if(bPointer >= FILE_BUF_LEN)
  {
    file.write(buf, FILE_BUF_LEN);
    bPointer = 0;
  } 
}

void FileStore::Write(char* b)
{
  if(!inUse)
  {
    platform->Message(HOST_MESSAGE, "Attempt to write string to a non-open file.\n");
    return;
  }
  int i = 0;
  while(b[i])
    Write(b[i++]); 
  //files[file].print(b);
}


//-----------------------------------------------------------------------------------------------------


FileStore* Platform::GetFileStore(char* directory, char* fileName, bool write)
{
  FileStore* result = NULL;
  for(int i = 0; i < MAX_FILES; i++)
    if(!files[i]->inUse)
    {
      files[i]->inUse = true;
      if(files[i]->Open(directory, fileName, write))
        return files[i];
      else
      {
        files[i]->inUse = false;
        return NULL;
      }
    }
  Message(HOST_MESSAGE, "Max open file count exceeded.\n");
  return NULL;
}


MassStorage* Platform::GetMassStorage()
{
  return massStorage;
}

void Platform::ReturnFileStore(FileStore* fs)
{
  for(int i = 0; i < MAX_FILES; i++)
      if(files[i] = fs)
        {
          files[i]->inUse = false;
          return;
        }
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
  
    FileStore* m = GetFileStore(GetWebDir(), MESSAGE_FILE, true);
    m->GoToEnd();
    m->Write(message);
    m->Close();
    line->Write(message);
  }
}




//***************************************************************************************************


void Platform::Spin()
{
   if(!active)
     return;
    
   network->Spin();
   line->Spin();

   if(Time() - lastTime < 2.0)
     return;
   lastTime = Time();
}

Line::Line()
{
}

void Line::Init()
{
	alternateInput = NULL;
	alternateOutput = NULL;
	Serial.begin(BAUD_RATE);
}

Network::Network()
{
	server = new EthernetServer(HTTP_PORT);
}

void Network::Init()
{
	alternateInput = NULL;
	alternateOutput = NULL;

	mac = MAC;

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
}

void Network::Write(char b)
{
  if(client)
  {
    client.write(b);
  } else
    reprap.GetPlatform()->Message(HOST_MESSAGE, "Attempt to send byte to disconnected client.");
}

void Network::Write(char* s)
{
  if(client)
  {
    client.print(s);
  } else
	  reprap.GetPlatform()->Message(HOST_MESSAGE, "Attempt to send string to disconnected client.\n");
}

int Network::Read(char& b)
{
  if(client)
  {
    b = client.read();
    return true;
  }

  reprap.GetPlatform()->Message(HOST_MESSAGE, "Attempt to read from disconnected client.");
  b = '\n'; // good idea??
  return 0;
}


void Network::Close()
{
  if (client)
  {
    client.stop();
    //Serial.println("client disconnected");
  } else
	  reprap.GetPlatform()->Message(HOST_MESSAGE, "Attempt to disconnect non-existent client.");
}



















