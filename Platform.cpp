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
  fileStructureInitialised = false;
  
  line = new Line();

  // Files
  
  massStorage = new MassStorage(this);
  
  for(int8_t i=0; i < MAX_FILES; i++)
    files[i] = new FileStore(this);
  
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

  line->Init();

  network->Init();

  massStorage->Init();

  for(i=0; i < MAX_FILES; i++)
    files[i]->Init();

  fileStructureInitialised = true;

  mcp.begin();

  sysDir = SYS_DIR;
  configFile = CONFIG_FILE;

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
  potWipes = POT_WIPES;
  senseResistor = SENSE_RESISTOR;
  maxAtoDVoltage = MAX_A_TO_D_VOLTAGE;

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
  tempDir = TEMP_DIR;

  for(i = 0; i < DRIVES; i++)
  {

	  if(stepPins[i] >= 0)
	  {
		  if(i > Z_AXIS)
			  pinModeNonDue(stepPins[i], OUTPUT);
		  else
			  pinMode(stepPins[i], OUTPUT);
	  }
	  if(directionPins[i] >= 0)
	  {
		  if(i > Z_AXIS)
			  pinModeNonDue(directionPins[i], OUTPUT);
		  else
			  pinMode(directionPins[i], OUTPUT);
	  }
	  if(enablePins[i] >= 0)
	  {
		  if(i >= Z_AXIS)
			  pinModeNonDue(enablePins[i], OUTPUT);
		  else
			  pinMode(enablePins[i], OUTPUT);
	  }
	  Disable(i);
	  driveEnabled[i] = false;
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
      pinModeNonDue(heatOnPins[i], OUTPUT);
    thermistorInfRs[i] = ( thermistorInfRs[i]*exp(-thermistorBetas[i]/(25.0 - ABS_ZERO)) );
  }  
  
  InitialiseInterrupts();
  
  lastTime = Time();
  
  active = true;
}

void Platform::Diagnostics() 
{
  Message(HOST_MESSAGE, "Platform Diagnostics:\n"); 
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
    
  if(power <= 0.0)
  {
     analogWriteNonDue(heatOnPins[heater], 0);
     return;
  }
  
  if(power >= 1.0)
  {
     analogWriteNonDue(heatOnPins[heater], 255);
     return;
  }
  
  byte p = (byte)(255.0*power);
  analogWriteNonDue(heatOnPins[heater], p);
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
	hsmciPinsinit();
	// Initialize SD MMC stack
	sd_mmc_init();

	int sdPresentCount = 0;
	while ((CTRL_NO_PRESENT == sd_mmc_check(0)) && (sdPresentCount < 5))
	{
		//platform->Message(HOST_MESSAGE, "Please plug in the SD card.\n");
		//delay(1000);
	}

	if(sdPresentCount >= 5)
	{
		platform->Message(HOST_MESSAGE, "Can't find the SD card.\n");
		return;
	}

	//print card info

//	SerialUSB.print("sd_mmc_card->capacity: ");
//	SerialUSB.print(sd_mmc_get_capacity(0));
//	SerialUSB.print(" bytes\n");
//	SerialUSB.print("sd_mmc_card->clock: ");
//	SerialUSB.print(sd_mmc_get_bus_clock(0));
//	SerialUSB.print(" Hz\n");
//	SerialUSB.print("sd_mmc_card->bus_width: ");
//	SerialUSB.println(sd_mmc_get_bus_width(0));

	memset(&fileSystem, 0, sizeof(FATFS));
	//f_mount (LUN_ID_SD_MMC_0_MEM, NULL);
	//int mounted = f_mount(LUN_ID_SD_MMC_0_MEM, &fileSystem);
	int mounted = f_mount(0, &fileSystem);
	if (mounted != FR_OK)
	{
		platform->Message(HOST_MESSAGE, "Can't mount filesystem 0: code ");
		sprintf(scratchString, "%d", mounted);
		platform->Message(HOST_MESSAGE, scratchString);
		platform->Message(HOST_MESSAGE, "\n");
	}
}

char* MassStorage::CombineName(char* directory, char* fileName)
{
  int out = 0;
  int in = 0;
  
//  scratchString[out] = '/';
//  out++;
  
  if(directory != NULL)
  {
    //if(directory[in] == '/')
    //  in++;
    while(directory[in] != 0 && directory[in] != '\n')// && directory[in] != '/')
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
  
  //scratchString[out] = '/';
 // out++;
  
  in = 0;
  while(fileName[in] != 0 && fileName[in] != '\n')// && fileName[in] != '/')
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
//  File dir, entry;
//  dir = SD.open(directory);
//  int p = 0;
//  int q;
//  int count = 0;
//  while(entry = dir.openNextFile())
//  {
//    q = 0;
//    count++;
//    fileList[p++] = FILE_LIST_BRACKET;
//    while(entry.name()[q])
//    {
//      fileList[p++] = entry.name()[q];
//      q++;
//      if(p >= FILE_LIST_LENGTH - 10) // Caution...
//      {
//        platform->Message(HOST_MESSAGE, "FileList - directory: ");
//        platform->Message(HOST_MESSAGE, directory);
//        platform->Message(HOST_MESSAGE, " has too many files!\n");
//        return "";
//      }
//    }
//    fileList[p++] = FILE_LIST_BRACKET;
//    fileList[p++] = FILE_LIST_SEPARATOR;
//    entry.close();
//  }
//  dir.close();
//
//  if(count <= 0)
//    return "";
//
//  fileList[--p] = 0; // Get rid of the last separator
//  return fileList;
	return "";
}

// Delete a file
bool MassStorage::Delete(char* directory, char* fileName)
{
	char* location = platform->GetMassStorage()->CombineName(directory, fileName);
	if( f_unlink (location) != FR_OK)
	{
		platform->Message(HOST_MESSAGE, "Can't delete file ");
		platform->Message(HOST_MESSAGE, location);
		platform->Message(HOST_MESSAGE, "\n");
		return false;
	}
	return true;
}

//------------------------------------------------------------------------------------------------


FileStore::FileStore(Platform* p)
{
   platform = p;  
}


void FileStore::Init()
{
  bufferPointer = 0;
  inUse = false;
  writing = false;
  lastBufferEntry = 0;
}


// Open a local file (for example on an SD card).
// This is protected - only Platform can access it.

bool FileStore::Open(char* directory, char* fileName, bool write)
{
  char* location = platform->GetMassStorage()->CombineName(directory, fileName);
//  SerialUSB.print("Opening: ");
//  SerialUSB.println(location);
  writing = write;
  lastBufferEntry = FILE_BUF_LEN - 1;
  FRESULT openReturn;

  if(writing)
  {
	  openReturn = f_open(&file, location, FA_CREATE_ALWAYS | FA_WRITE);
	  if (openReturn != FR_OK)
	  {
		  platform->Message(HOST_MESSAGE, "Can't open ");
		  platform->Message(HOST_MESSAGE, location);
		  platform->Message(HOST_MESSAGE, " to write to.  Error code: ");
		  sprintf(scratchString, "%d", openReturn);
		  platform->Message(HOST_MESSAGE, scratchString);
		  platform->Message(HOST_MESSAGE, "\n");
		  return false;
	  }
	  bufferPointer = 0;
  } else
  {
	  openReturn = f_open(&file, location, FA_OPEN_EXISTING | FA_READ);
	  if (openReturn != FR_OK)
	  {
		  platform->Message(HOST_MESSAGE, "Can't open ");
		  platform->Message(HOST_MESSAGE, location);
		  platform->Message(HOST_MESSAGE, " to read from.  Error code: ");
		  sprintf(scratchString, "%d", openReturn);
		  platform->Message(HOST_MESSAGE, scratchString);
		  platform->Message(HOST_MESSAGE, "\n");
		  return false;
	  }
	  bufferPointer = FILE_BUF_LEN;
  }

  inUse = true;
  return true;
}

void FileStore::Close()
{
  if(writing)
	  WriteBuffer();
  f_close(&file);
  platform->ReturnFileStore(this);
  inUse = false;
  writing = false;
  lastBufferEntry = 0;
}

void FileStore::GoToEnd()
{
  if(!inUse)
  {
    platform->Message(HOST_MESSAGE, "Attempt to seek on a non-open file.\n");
    return;
  }
  unsigned long e = Length();
  f_lseek(&file, e);
}

unsigned long FileStore::Length()
{
  if(!inUse)
  {
    platform->Message(HOST_MESSAGE, "Attempt to size non-open file.\n");
    return 0;
  }
  return file.fsize;
	return 0;
}

int8_t FileStore::Status()
{
  if(!inUse)
    return nothing;

  if(lastBufferEntry == FILE_BUF_LEN)
	return byteAvailable;

  if(bufferPointer < lastBufferEntry)
    return byteAvailable;
    
  return nothing;
}

void FileStore::ReadBuffer()
{
	FRESULT readStatus;
	readStatus = f_read(&file, buf, FILE_BUF_LEN, &lastBufferEntry);	// Read a chunk of file
	if (readStatus)
	{
		platform->Message(HOST_MESSAGE, "Error reading file.\n");
	}
	bufferPointer = 0;
}

bool FileStore::Read(char& b)
{
  if(!inUse)
  {
    platform->Message(HOST_MESSAGE, "Attempt to read from a non-open file.\n");
    return false;
  }

  if(bufferPointer >= FILE_BUF_LEN)
	  ReadBuffer();

  if(bufferPointer >= lastBufferEntry)
  {
	  b = 0;  // Good idea?
	  return false;
  }

  b = (char)buf[bufferPointer];
  bufferPointer++;

  return true;
}

void FileStore::WriteBuffer()
{
	FRESULT writeStatus;
	writeStatus = f_write(&file, buf, bufferPointer, &lastBufferEntry);
	if((writeStatus != FR_OK) || (lastBufferEntry != bufferPointer))
	{
		platform->Message(HOST_MESSAGE, "Error writing file.  Disc may be full.\n");
	}
	bufferPointer = 0;
}


void FileStore::Write(char b)
{
  if(!inUse)
  {
    platform->Message(HOST_MESSAGE, "Attempt to write byte to a non-open file.\n");
    return;
  }
  buf[bufferPointer] = b;
  bufferPointer++;
  if(bufferPointer >= FILE_BUF_LEN)
	  WriteBuffer();
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
}


//-----------------------------------------------------------------------------------------------------

FileStore* Platform::GetFileStore(char* directory, char* fileName, bool write)
{
  FileStore* result = NULL;

  if(!fileStructureInitialised)
	  return NULL;

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
      if(files[i] == fs)
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
  
//    FileStore* m = GetFileStore(GetWebDir(), MESSAGE_FILE, true);
//    if(m != NULL)
//    {
//    	m->GoToEnd();
//    	m->Write(message);
//    	m->Close();
//    } else
//    	line->Write("Can't open message file.\n");
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
	SerialUSB.begin(BAUD_RATE);
	while (!SerialUSB.available());
}

Network::Network()
{
//	server = new EthernetServer(HTTP_PORT);
}

void Network::Init()
{
	alternateInput = NULL;
	alternateOutput = NULL;

	mac = MAC;

//	// disable SD SPI while starting w5100
//	// or you will have trouble
//	pinMode(SD_SPI, OUTPUT);
//	digitalWrite(SD_SPI,HIGH);

	ipAddress = { IP0, IP1, IP2, IP3 };
	//Ethernet.begin(mac, *(new IPAddress(IP0, IP1, IP2, IP3)));
//	Ethernet.begin(mac, ipAddress);
//	server->begin();
//
//	//Serial.print("server is at ");
//	//Serial.println(Ethernet.localIP());
//
//	// this corrects a bug in the Ethernet.begin() function
//	// even tho the call to Ethernet.localIP() does the same thing
//	digitalWrite(ETH_B_PIN, HIGH);

	clientStatus = 0;
//	client = 0;
}

void Network::Write(char b)
{
//  if(client)
//  {
//    client.write(b);
//  } else
    reprap.GetPlatform()->Message(HOST_MESSAGE, "Attempt to send byte to disconnected client.");
}

void Network::Write(char* s)
{
//  if(client)
//  {
//    client.print(s);
//  } else
	  reprap.GetPlatform()->Message(HOST_MESSAGE, "Attempt to send string to disconnected client.\n");
}

int Network::Read(char& b)
{
//  if(client)
//  {
//    b = client.read();
//    return true;
//  }
//
//  reprap.GetPlatform()->Message(HOST_MESSAGE, "Attempt to read from disconnected client.");
//  b = '\n'; // good idea??
  return 0;
}


void Network::Close()
{
//  if (client)
//  {
//    client.stop();
//    //Serial.println("client disconnected");
//  } else
	  reprap.GetPlatform()->Message(HOST_MESSAGE, "Attempt to disconnect non-existent client.");
}



















