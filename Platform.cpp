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
  homeDirection = HOME_DIRECTION;
  maxFeedrates = MAX_FEEDRATES;
  accelerations = ACCELERATIONS;
  driveStepsPerUnit = DRIVE_STEPS_PER_UNIT;
  instantDvs = INSTANT_DVS;
  potWipes = POT_WIPES;
  senseResistor = SENSE_RESISTOR;
  maxStepperDigipotVoltage = MAX_STEPPER_DIGIPOT_VOLTAGE;
//  zProbeGradient = Z_PROBE_GRADIENT;
//  zProbeConstant = Z_PROBE_CONSTANT;
  zProbeEnable = Z_PROBE_ENABLE;
  zProbePin = Z_PROBE_PIN;
  zProbeCount = 0;
  zProbeSum = 0;
  zProbeValue = 0;
//  zProbeStarting = false;
//  zProbeHigh = Z_PROBE_HIGH;
//  zProbeLow = Z_PROBE_LOW;
  zProbeADValue = Z_PROBE_AD_VALUE;
  zProbeStopHeight = Z_PROBE_STOP_HEIGHT;

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
  
  if(heatOnPins[0] >= 0)
        pinMode(heatOnPins[0], OUTPUT);
      thermistorInfRs[0] = ( thermistorInfRs[0]*exp(-thermistorBetas[0]/(25.0 - ABS_ZERO)) );
  
  for(i = 1; i < HEATERS; i++)
  {
    if(heatOnPins[i] >= 0)
      pinModeNonDue(heatOnPins[i], OUTPUT);
    thermistorInfRs[i] = ( thermistorInfRs[i]*exp(-thermistorBetas[i]/(25.0 - ABS_ZERO)) );
  }

  if(zProbePin >= 0)
	  pinMode(zProbePin, INPUT);
  
  InitialiseInterrupts();
  
  lastTime = Time();
  
  active = true;
}


//int zcount; // NASTY - FIX ME

void Platform::Spin()
{
  if(!active)
    return;

  network->Spin();
  line->Spin();

  if(Time() - lastTime < 0.006)
    return;
  PollZHeight();
  lastTime = Time();

//  zcount++;
//  if(zcount > 30)
//  {
//	   zcount = 0;
//	   SerialUSB.println(GetRawZHeight());
//  }
}

//*****************************************************************************************************************

// Interrupts

void TC3_Handler()
{
  TC_GetStatus(TC1, 0);
  reprap.Interrupt();
}

void Platform::InitialiseInterrupts()
{
  pmc_set_writeprotect(false);
  pmc_enable_periph_clk((uint32_t)TC3_IRQn);
  TC_Configure(TC1, 0, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK4);
  TC1->TC_CHANNEL[0].TC_IER=TC_IER_CPCS;
  TC1->TC_CHANNEL[0].TC_IDR=~TC_IER_CPCS;
  SetInterrupt(STANDBY_INTERRUPT_RATE);
}


//*************************************************************************************************

void Platform::Diagnostics() 
{
  Message(HOST_MESSAGE, "Platform Diagnostics:\n"); 
}

//extern int __bss_end; // void? long?
//extern void *__brkval;
//
//long Platform::GetFreeMemory()
//{
//	long free_memory;
//
//  if((long)__brkval == 0)
//    free_memory = ((long)&free_memory) - ((long)&__bss_end);
//  else
//    free_memory = ((long)&free_memory) - ((long)__brkval);
//
//  return free_memory;
//}


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
  
  
  byte p = (byte)(255.0*fmin(1.0, fmax(0.0, power)));
  if(heater == 0)
	  analogWrite(heatOnPins[heater], p);
  else
	  analogWriteNonDue(heatOnPins[heater], p);
}

inline void Platform::PollZHeight()
{
	if(zProbeCount >= 5)
	{
		zProbeValue = zProbeSum/5;
		zProbeSum = 0;
		zProbeCount = 0;
	}
	zProbeSum += GetRawZHeight();
	zProbeCount++;
}


EndStopHit Platform::Stopped(int8_t drive)
{
	if(zProbeEnable && drive == Z_AXIS)
	{
		if(ZProbe() > zProbeADValue)
			return lowHit;
		else
			return noStop;
	}

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
		sdPresentCount++;
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
  DIR dir;
  FILINFO entry;
  FRESULT res;
  char loc[64];
  int len = 0;

  len = strlen(directory);
  strncpy(loc,directory,len-1);
  loc[len - 1 ] = 0;

//  if(reprap.debug()) {
//	  platform->Message(HOST_MESSAGE, "Opening: ");
//	  platform->Message(HOST_MESSAGE, loc);
//	  platform->Message(HOST_MESSAGE, "\n");
//  }

  res = f_opendir(&dir,loc);
  if(res == FR_OK)
  {

//	  if(reprap.debug()) {
//		  platform->Message(HOST_MESSAGE, "Directory open\n");
//	  }

	  int p = 0;
//  int q;
	  int foundFiles = 0;

	  f_readdir(&dir,0);

	  while((f_readdir(&dir,&entry) == FR_OK) && (foundFiles < 24))
	  {
		  foundFiles++;

		  if(strlen(entry.fname) > 0)
		  {
			int q = 0;
			fileList[p++] = FILE_LIST_BRACKET;
			while(entry.fname[q])
			{
			  fileList[p++] = entry.fname[q];
			  //SerialUSB.print(entry.fname[q]);
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
		  }
	  }

	  if(foundFiles <= 0)
		return "NONE";

	  fileList[--p] = 0; // Get rid of the last separator
	  return fileList;
  }

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

// Serial/USB class

Line::Line()
{
}

void Line::Init()
{
//	alternateInput = NULL;
//	alternateOutput = NULL;
	SerialUSB.begin(BAUD_RATE);
	//while (!SerialUSB.available());
}

//***************************************************************************************************

// Network/Ethernet class

// C calls to interface with LWIP (http://savannah.nongnu.org/projects/lwip/)
// These are implemented in, and called from, a modified version of httpd.c
// in the network directory.

extern "C"
{

// Transmit data to the Network

void RepRapNetworkSendOutput(char* data, int length, void* pbuf, void* pcb, void* hs);

// When lwip releases storage, set the local copy of the pointer to 0 to stop
// it being used again.

void RepRapNetworkInputBufferReleased(void* pb)
{
	reprap.GetPlatform()->GetNetwork()->InputBufferReleased(pb);
}

void RepRapNetworkHttpStateReleased(void* h)
{
	reprap.GetPlatform()->GetNetwork()->HttpStateReleased(h);
}

// Called to put out a message via the RepRap firmware.

void RepRapNetworkMessage(char* s)
{
	reprap.GetPlatform()->Message(HOST_MESSAGE, s);
}

// Called to push data into the RepRap firmware.

void RepRapNetworkReceiveInput(char* data, int length, void* pbuf, void* pcb, void* hs)
{
	reprap.GetPlatform()->GetNetwork()->ReceiveInput(data, length, pbuf, pcb, hs);
}

// Called when transmission of outgoing data is complete to allow
// the RepRap firmware to write more.

void RepRapNetworkAllowWriting()
{
	reprap.GetPlatform()->GetNetwork()->SetWriteEnable(true);
}

bool RepRapNetworkHasALiveClient()
{
	return reprap.GetPlatform()->GetNetwork()->Status() & clientLive;
}

}



Network::Network()
{
	ethPinsInit();

	// Construct the ring buffer

	netRingAddPointer = new NetRing(NULL);
	netRingGetPointer = netRingAddPointer;
	for(int8_t i = 1; i < HTTP_STATE_SIZE; i++)
		netRingGetPointer = new NetRing(netRingGetPointer);
	netRingAddPointer->SetNext(netRingGetPointer);
}

// Reset the network to its disconnected and ready state.

void Network::Reset()
{
	//reprap.GetPlatform()->Message(HOST_MESSAGE, "Reset.\n");
	inputPointer = 0;
	inputLength = -1;
	outputPointer = 0;
	writeEnabled = false;
	closePending = false;
	status = nothing;
}

void Network::CleanRing()
{
	for(int8_t i = 0; i <= HTTP_STATE_SIZE; i++)
	{
		netRingGetPointer->Free();
		netRingGetPointer = netRingGetPointer->Next();
	}
	netRingAddPointer = netRingGetPointer;
}

void Network::Init()
{
//	alternateInput = NULL;
//	alternateOutput = NULL;
	init_ethernet();
	CleanRing();
	Reset();
}

void Network::Spin()
{
	// Keep the Ethernet running

	ethernet_task();

	// Anything come in from the network to act on?

	if(!netRingGetPointer->Active())
		return;

	// Finished reading the active ring element?

	if(!netRingGetPointer->ReadFinished())
	{
		// No - Finish reading any data that's been received.

		if(inputPointer < inputLength)
			return;

		// Haven't started reading it yet - set that up.

		inputPointer = 0;
		inputLength = netRingGetPointer->Length();
		inputBuffer = netRingGetPointer->Data();
	}
}

// Webserver calls this to read bytes that have come in from the network

bool Network::Read(char& b)
{
	if(inputPointer >= inputLength)
	{
		inputLength = -1;
		inputPointer = 0;
		netRingGetPointer->SetReadFinished(); // Past tense...
		SetWriteEnable(true);
		//reprap.GetPlatform()->Message(HOST_MESSAGE, "Network - data read.\n");
		return false;
	}
	b = inputBuffer[inputPointer];
	inputPointer++;
	return true;
}

// Webserver calls this to write bytes that need to go out to the network

void Network::Write(char b)
{
	// Check for horrible things...

	if(!writeEnabled)
	{
		reprap.GetPlatform()->Message(HOST_MESSAGE, "Network::Write(char b) - Attempt to write when disabled.\n");
		return;
	}

	if(outputPointer >= STRING_LENGTH)
	{
		reprap.GetPlatform()->Message(HOST_MESSAGE, "Network::Write(char b) - Output buffer overflow! \n");
		return;
	}

	// Add the byte to the buffer

	outputBuffer[outputPointer] = b;
	outputPointer++;

	// Buffer full?  If so, send it.

	if(outputPointer >= STRING_LENGTH - 5) // 5 is for safety
	{
		SetWriteEnable(false);  // Stop further writing from Webserver until the network tells us that this has gone
		RepRapNetworkSendOutput(outputBuffer, outputPointer, netRingGetPointer->Pbuf(), netRingGetPointer->Pcb(), netRingGetPointer->Hs());
		outputPointer = 0;
	}
}




void Network::InputBufferReleased(void* pb)
{
	if(netRingGetPointer->Pbuf() != pb)
	{
		reprap.GetPlatform()->Message(HOST_MESSAGE, "Network::InputBufferReleased() - Pointers don't match!\n");
		return;
	}
	netRingGetPointer->ReleasePbuf();
}

void Network::HttpStateReleased(void* h)
{
	if(netRingGetPointer->Hs() != h)
	{
		reprap.GetPlatform()->Message(HOST_MESSAGE, "Network::HttpStateReleased() - Pointers don't match!\n");
		return;
	}
	netRingGetPointer->ReleaseHs();
}


void Network::ReceiveInput(char* data, int length, void* pbuf, void* pcb, void* hs)
{
	status = clientLive;
	if(netRingAddPointer->Active())
	{
		reprap.GetPlatform()->Message(HOST_MESSAGE, "Network::ReceiveInput() - Ring buffer full!\n");
		return;
	}
	netRingAddPointer->Set(data, length, pbuf, pcb, hs);
	netRingAddPointer = netRingAddPointer->Next();
	//reprap.GetPlatform()->Message(HOST_MESSAGE, "Network - input received.\n");
}



bool Network::CanWrite()
{
	return writeEnabled;
}

void Network::SetWriteEnable(bool enable)
{
	writeEnabled = enable;
	if(!writeEnabled)
		return;
	if(closePending)
		Close();
}

// This is not called for data, only for internally-
// generated short strings at the start of a transmission,
// so it should never overflow the buffer (which is checked
// anyway).

void Network::Write(char* s)
{
	int i = 0;
	while(s[i])
		Write(s[i++]);
}



void Network::Close()
{
	if(Status() && clientLive)
	{
		if(outputPointer > 0)
		{
			SetWriteEnable(false);
			RepRapNetworkSendOutput(outputBuffer, outputPointer, netRingGetPointer->Pbuf(), netRingGetPointer->Pcb(), netRingGetPointer->Hs());
			outputPointer = 0;
			closePending = true;
			return;
		}
		RepRapNetworkSendOutput((char*)NULL, 0, netRingGetPointer->Pbuf(), netRingGetPointer->Pcb(), netRingGetPointer->Hs());
		netRingGetPointer->Free();
		netRingGetPointer = netRingGetPointer->Next();
		//reprap.GetPlatform()->Message(HOST_MESSAGE, "Network - output sent and closed.\n");
	} else
		reprap.GetPlatform()->Message(HOST_MESSAGE, "Network::Close() - Attempt to close a closed connection!\n");
	closePending = false;
	status = nothing;
	//Reset();
}

int8_t Network::Status()
{
	if(inputPointer >= inputLength)
		return status;
	return status | clientConnected | byteAvailable;
}


NetRing::NetRing(NetRing* n)
{
	next = n;
	Free();
}

void NetRing::Free()
{
	pbuf = 0;
	pcb = 0;
	hs = 0;
	data = "";
	length = 0;
	read = false;
	active = false;
}

bool NetRing::Set(char* d, int l, void* pb, void* pc, void* h)
{
	if(active)
		return false;
	pbuf = pb;
	pcb = pc;
	hs = h;
	data = d;
	length = l;
	read = false;
	active = true;
	return true;
}

NetRing* NetRing::Next()
{
	return next;
}

char* NetRing::Data()
{
	return data;
}

int NetRing::Length()
{
	return length;
}

bool NetRing::ReadFinished()
{
	return read;
}

void NetRing::SetReadFinished()
{
	read = true;
}

bool NetRing::Active()
{
	return active;
}

void NetRing::SetNext(NetRing* n)
{
	next = n;
}

void* NetRing::Pbuf()
{
	return pbuf;
}

void NetRing::ReleasePbuf()
{
	pbuf = 0;
}

void* NetRing::Pcb()
{
	return pcb;
}

void* NetRing::Hs()
{
	return hs;
}

void NetRing::ReleaseHs()
{
	hs = 0;
}



















