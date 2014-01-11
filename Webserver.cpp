/****************************************************************************************************

RepRapFirmware - Webserver

This class serves a single-page web applications to the attached network.  This page forms the user's 
interface with the RepRap machine.  This software interprests returned values from the page and uses it
to Generate G Codes, which it sends to the RepRap.  It also collects values from the RepRap like 
temperature and uses those to construct the web page.

The page itself - reprap.htm - uses Knockout.js and Jquery.js.  See:

http://knockoutjs.com/

http://jquery.com/

-----------------------------------------------------------------------------------------------------

Version 0.2

10 May 2013

Adrian Bowyer
RepRap Professional Ltd
http://reprappro.com

Licence: GPL

****************************************************************************************************/

#include "RepRapFirmware.h"

//***************************************************************************************************


bool Webserver::MatchBoundary(char c)
{
  if(!postBoundary[0])
    return false;
    
  if(c == postBoundary[boundaryCount])
  {
    boundaryCount++;
    if(!postBoundary[boundaryCount])
    {
      boundaryCount = 0;
      return true;
    }
  } else
  {
    for(int i = 0; i < boundaryCount; i++)
      postFile->Write(postBoundary[i]);
    postFile->Write(c);
    boundaryCount = 0;
  }
  return false;  
}


//****************************************************************************************************

// Feeding G Codes to the GCodes class

bool Webserver::GCodeAvailable()
{
  return gcodeAvailable;
}

byte Webserver::ReadGCode()
{
  byte c = gcodeBuffer[gcodePointer];
  if(!c)
  {
     gcodeAvailable = false;
     gcodePointer = 0;
     gcodeBuffer[gcodePointer] = 0;
  } else
    gcodePointer++;
  return c;
}


bool Webserver::LoadGcodeBuffer(char* gc, bool convertWeb)
{
  char scratchString[STRING_LENGTH];
  if(gcodeAvailable)
    return false;
  
  if(strlen(gc) > GCODE_LENGTH-1)
  {
    platform->Message(HOST_MESSAGE, "Webserver: GCode buffer overflow.\n");
    return false;  
  }
  
  int gcp = 0;
  gcodePointer = 0;
  gcodeBuffer[gcodePointer] = 0;
  
  char c;
  
  while(c = gc[gcp++])
  {
    if(c == '+' && convertWeb)
      c = ' ';
    if(c == '%'&& convertWeb)
    {
      c = 0;
      if(isalpha(gc[gcp]))
        c += 16*(gc[gcp] - 'A' + 10);
      else
        c += 16*(gc[gcp] - '0');
      gcp++;
      if(isalpha(gc[gcp]))
        c += gc[gcp] - 'A' + 10;
      else
        c += gc[gcp] - '0';
      gcp++;
    }
    gcodeBuffer[gcodePointer++] = c;
  }
  while(isspace(gcodeBuffer[gcodePointer - 1]) && gcodePointer > 0)  // Kill any trailing space
    gcodePointer--;
  gcodeBuffer[gcodePointer] = 0;
  gcodePointer = 0;  
  
// We intercept three G/M Codes so we can deal with file manipulation and emergencies.  That
// way things don't get out of sync, and - as a file name can contain
// a valid G code (!) - confusion is avoided.
  
  int8_t specialAction = 0;
  if(StringStartsWith(gcodeBuffer, "M30 ")) specialAction = 1;
  if(StringStartsWith(gcodeBuffer, "M23 ")) specialAction = 2;
  if(StringStartsWith(gcodeBuffer, "M112")) specialAction = 3;  // FIXME - suppose we ever have an M1121 ??
  
  if(specialAction) // Delete or print a file?
  { 
    if(specialAction == 1) // Delete?
    {
      if(!platform->GetMassStorage()->Delete(platform->GetGCodeDir(), &gcodeBuffer[4]))
      {
        platform->Message(HOST_MESSAGE, "Unsuccessful attempt to delete: ");
        platform->Message(HOST_MESSAGE, &gcodeBuffer[4]);
        platform->Message(HOST_MESSAGE, "\n");
      } 
    } else if (specialAction == 2)
    {
      reprap.GetGCodes()->QueueFileToPrint(&gcodeBuffer[4]);
    } else
    {
    	reprap.EmergencyStop();
    }
    
    // Check for further G Codes in the string
    
    while(gcodeBuffer[gcodePointer])
    {
       if(gcodeBuffer[gcodePointer] == '\n')
       {
         gcodeAvailable = true;
         return true;
       }
       gcodePointer++;
    }
    gcodePointer = 0;
    gcodeBuffer[gcodePointer] = 0;
    gcodeAvailable = false;
    return true;
  }

// Otherwise, send them to the G Code interpreter

  gcodeAvailable = true;
  return true;
}

//********************************************************************************************

// Communications with the client

//--------------------------------------------------------------------------------------------

// Output to the client

void Webserver::CloseClient()
{
  writing = false;
  //inPHPFile = false;
  //InitialisePHP();
  clientCloseTime = platform->Time();
  needToCloseClient = true;   
}


void Webserver::SendFile(char* nameOfFileToSend)
{
  char scratchString[STRING_LENGTH];
  char sLen[POST_LENGTH];
  bool zip = false;
    
  if(StringStartsWith(nameOfFileToSend, KO_START))
    GetJsonResponse(&nameOfFileToSend[KO_FIRST]);
    
  if(jsonPointer < 0)
  {
    fileBeingSent = platform->GetFileStore(platform->GetWebDir(), nameOfFileToSend, false);
    if(fileBeingSent == NULL)
    {
      nameOfFileToSend = FOUR04_FILE;
      fileBeingSent = platform->GetFileStore(platform->GetWebDir(), nameOfFileToSend, false);
    }
    writing = fileBeingSent != NULL;
  } 
  
  platform->GetNetwork()->Write("HTTP/1.1 200 OK\n");
  
  platform->GetNetwork()->Write("Content-Type: ");
  
  if(StringEndsWith(nameOfFileToSend, ".png"))
	  platform->GetNetwork()->Write("image/png\n");
  else if(StringEndsWith(nameOfFileToSend, ".ico"))
	  platform->GetNetwork()->Write("image/x-icon\n");
  else if (jsonPointer >=0)
	  platform->GetNetwork()->Write("application/json\n");
  else if(StringEndsWith(nameOfFileToSend, ".js"))
	  platform->GetNetwork()->Write("application/javascript\n");
  else if(StringEndsWith(nameOfFileToSend, ".zip"))
  {
	  platform->GetNetwork()->Write("application/zip\n");
    zip = true;
  } else
	  platform->GetNetwork()->Write("text/html\n");
    
  if (jsonPointer >=0)
  {
	  platform->GetNetwork()->Write("Content-Length: ");
    snprintf(sLen, POST_LENGTH, "%d", strlen(jsonResponse));
    platform->GetNetwork()->Write(sLen);
    platform->GetNetwork()->Write("\n");
  }
    
  if(zip)
  {
	platform->GetNetwork()->Write("Content-Encoding: gzip\n");
	platform->GetNetwork()->Write("Content-Length: ");
    snprintf(sLen, POST_LENGTH, "%llu", fileBeingSent->Length());
    platform->GetNetwork()->Write(sLen);
    platform->GetNetwork()->Write("\n");
  }
    
  platform->GetNetwork()->Write("Connection: close\n");

  platform->GetNetwork()->Write('\n');
}

void Webserver::WriteByte()
{
    char b;
    
    if(jsonPointer >= 0)
    {
      if(jsonResponse[jsonPointer])
    	  platform->GetNetwork()->Write(jsonResponse[jsonPointer++]);
      else
      {
        jsonPointer = -1;
        jsonResponse[0] = 0;
        CloseClient();
      }
    } else
    {
      if(fileBeingSent->Read(b))
    	  platform->GetNetwork()->Write(b);
      else
      {
        fileBeingSent->Close();
        CloseClient();
      }
    }
}

//----------------------------------------------------------------------------------------------------

// Input from the client

void Webserver::CheckPassword()
{
  gotPassword = StringEndsWith(clientQualifier, password);
}

void Webserver::JsonReport(bool ok, char* request)
{
  if(ok)
  {
    if(reprap.Debug())
    {
      platform->Message(HOST_MESSAGE, "JSON response: ");
      platform->Message(HOST_MESSAGE, jsonResponse);
      platform->Message(HOST_MESSAGE, " queued\n");
    }
  } else
  { 
    platform->Message(HOST_MESSAGE, "KnockOut request: ");
    platform->Message(HOST_MESSAGE, request);
    platform->Message(HOST_MESSAGE, " not recognised\n");
    clientRequest[0] = 0;
  } 
}

void Webserver::GetJsonResponse(char* request)
{
  jsonPointer = 0;
  writing = true;
  
  if(StringStartsWith(request, "poll"))
  {
    strncpy(jsonResponse, "{\"poll\":[", STRING_LENGTH);
    if(reprap.GetGCodes()->PrintingAFile())
    	strncat(jsonResponse, "\"P\",", STRING_LENGTH); // Printing
    else
    	strncat(jsonResponse, "\"I\",", STRING_LENGTH); // Idle
    for(int8_t heater = 0; heater < HEATERS; heater++)
    {
      strncat(jsonResponse, "\"", STRING_LENGTH);
      strncat(jsonResponse, ftoa(0, reprap.GetHeat()->GetTemperature(heater), 1), STRING_LENGTH);
      strncat(jsonResponse, "\",", STRING_LENGTH);
    }
    float liveCoordinates[DRIVES+1];
    reprap.GetMove()->LiveCoordinates(liveCoordinates);
    for(int8_t drive = 0; drive < AXES; drive++)
    {
    	strncat(jsonResponse, "\"", STRING_LENGTH);
    	strncat(jsonResponse, ftoa(0, liveCoordinates[drive], 2), STRING_LENGTH);
    	strncat(jsonResponse, "\",", STRING_LENGTH);
    }

    // FIXME: should loop through all Es

    strncat(jsonResponse, "\"", STRING_LENGTH);
    strncat(jsonResponse, ftoa(0, liveCoordinates[AXES], 4), STRING_LENGTH);
    strncat(jsonResponse, "\"", STRING_LENGTH);
    strncat(jsonResponse, "]}", STRING_LENGTH);
    JsonReport(true, request);
    return;
  }
  
  if(StringStartsWith(request, "gcode"))
  {
    if(!LoadGcodeBuffer(&clientQualifier[6], true))
      platform->Message(HOST_MESSAGE, "Webserver: buffer not free!\n");
    strncpy(jsonResponse, "{}", STRING_LENGTH);
    JsonReport(true, request);
    return;
  }
  
  if(StringStartsWith(request, "files"))
  {
    char* fileList = platform->GetMassStorage()->FileList(platform->GetGCodeDir(), false);
    strncpy(jsonResponse, "{\"files\":[", STRING_LENGTH);
    strncat(jsonResponse, fileList, STRING_LENGTH);
    strncat(jsonResponse, "]}", STRING_LENGTH);
    JsonReport(true, request);
    return;
  }
  
  if(StringStartsWith(request, "name"))
  {
    strncpy(jsonResponse, "{\"myName\":\"", STRING_LENGTH);
    strncat(jsonResponse, myName, STRING_LENGTH);
    strncat(jsonResponse, "\"}", STRING_LENGTH);
    JsonReport(true, request);
    return;
  }
  
  if(StringStartsWith(request, "password"))
  {
    CheckPassword();
    strncpy(jsonResponse, "{\"password\":\"", STRING_LENGTH);
    if(gotPassword)
      strncat(jsonResponse, "right", STRING_LENGTH);
    else
      strncat(jsonResponse, "wrong", STRING_LENGTH);
    strncat(jsonResponse, "\"}", STRING_LENGTH);
    JsonReport(true, request);
    return;
  }
  
  if(StringStartsWith(request, "axes"))
  {
    strncpy(jsonResponse, "{\"axes\":[", STRING_LENGTH);
    for(int8_t drive = 0; drive < AXES; drive++)
    {
      strncat(jsonResponse, "\"", STRING_LENGTH);
      strncat(jsonResponse, ftoa(0, platform->AxisLength(drive), 1), STRING_LENGTH);
      if(drive < AXES-1)
        strncat(jsonResponse, "\",", STRING_LENGTH);
      else
        strncat(jsonResponse, "\"", STRING_LENGTH);
    }
    strncat(jsonResponse, "]}", STRING_LENGTH);
    JsonReport(true, request);
    return;
  }
  
  JsonReport(false, request);
}

/*

Parse a string in clientLine[] from the user's web browser

Simple requests have the form:

GET /page2.htm HTTP/1.1
     ^  Start clientRequest[] at clientLine[5]; stop at the blank or...

...fancier ones with arguments after a '?' go:

GET /gather.asp?pwd=my_pwd HTTP/1.1
     ^ Start clientRequest[]
                ^ Start clientQualifier[]
*/

void Webserver::ParseGetPost()
{
    if(reprap.Debug())
    {
      platform->Message(HOST_MESSAGE, "HTTP request: ");
      platform->Message(HOST_MESSAGE, clientLine);
      platform->Message(HOST_MESSAGE, "\n");
    }
    
    int i = 5;
    int j = 0;
    clientRequest[j] = 0;
    clientQualifier[0] = 0;
    while(clientLine[i] != ' ' && clientLine[i] != '?')
    {
      clientRequest[j] = clientLine[i];
      j++;
      i++;
    }
    clientRequest[j] = 0;
    if(clientLine[i] == '?')
    {
      i++;
      j = 0;
      while(clientLine[i] != ' ')
      {
        clientQualifier[j] = clientLine[i];
        j++;
        i++;      
      }
      clientQualifier[j] = 0;
    } 
}

void Webserver::InitialisePost()
{
  postSeen = false;
  receivingPost = false;
  boundaryCount = 0; 
  postBoundary[0] = 0;
  postFileName[0] = 0;
  postFile = NULL;
}

void Webserver::ParseClientLine()
{ 
  if(StringStartsWith(clientLine, "GET"))
  {
    ParseGetPost();
    postSeen = false;
    getSeen = true;
    if(!clientRequest[0])
      strncpy(clientRequest, INDEX_PAGE, STRING_LENGTH);
    return;
  }
  
  if(StringStartsWith(clientLine, "POST"))
  {
    ParseGetPost();
    InitialisePost();
    postSeen = true;
    getSeen = false;
    if(!clientRequest[0])
      strncpy(clientRequest, INDEX_PAGE, STRING_LENGTH);
    return;
  }
  
  int bnd;
  
  if(postSeen && ( (bnd = StringContains(clientLine, "boundary=")) >= 0) )
  {
    if(strlen(&clientLine[bnd]) >= POST_LENGTH - 4)
    {
      platform->Message(HOST_MESSAGE, "Post boundary buffer overflow.\n");
      return;
    }
    postBoundary[0] = '-';
    postBoundary[1] = '-';
    strncpy(&postBoundary[2], &clientLine[bnd], POST_LENGTH - 3);
    strncat(postBoundary, "--", POST_LENGTH);
    //Serial.print("Got boundary: ");
    //Serial.println(postBoundary);
    return;
  }
  
  if(receivingPost && StringStartsWith(clientLine, "Content-Disposition:"))
  {
    bnd = StringContains(clientLine, "filename=\"");
    if(bnd < 0)
    {
      platform->Message(HOST_MESSAGE, "Post disposition gives no filename.\n");
      return;
    }
    int i = 0;
    while(clientLine[bnd] && clientLine[bnd] != '"')
    {
      postFileName[i++] = clientLine[bnd++];
      if(i >= POST_LENGTH)
      {
        i = 0;
        platform->Message(HOST_MESSAGE, "Post filename buffer overflow.\n");
      }
    }
    postFileName[i] = 0;
    //Serial.print("Got file name: ");
    //Serial.println(postFileName);    
    return;
  }  
}

// if you've gotten to the end of the line (received a newline
// character) and the line is blank, the http request has ended,
// so you can send a reply
void Webserver::BlankLineFromClient()
{
  char scratchString[STRING_LENGTH];
  clientLine[clientLinePointer] = 0;
  clientLinePointer = 0;
  //ParseQualifier();
  
  //Serial.println("End of header.");
  
  // Soak up any rubbish on the end.

  char c;
  while(platform->GetNetwork()->Read(c));


  if(getSeen)
  {
    SendFile(clientRequest);
    clientRequest[0] = 0;
    return;
  }
  
  if(postSeen)
  {
    receivingPost = true;
    postSeen = false;
    return;
  }
  
  if(receivingPost)
  {
    postFile = platform->GetFileStore(platform->GetGCodeDir(), postFileName, true);
    if(postFile == NULL  || !postBoundary[0])
    {
      platform->Message(HOST_MESSAGE, "Can't open file for write or no post boundary: ");
      platform->Message(HOST_MESSAGE, postFileName);
      platform->Message(HOST_MESSAGE, "\n");
      InitialisePost();
      if(postFile != NULL)
        postFile->Close();
    }
  }  

  
}

void Webserver::CharFromClient(char c)
{
  if(c == '\n' && clientLineIsBlank) 
  {
    BlankLineFromClient();
    return;
  }
  
  if(c == '\n') 
  {
    clientLine[clientLinePointer] = 0;
    ParseClientLine();
    // you're starting a new line
    clientLineIsBlank = true;
    clientLinePointer = 0;
  } else if(c != '\r') 
  {
    // you've gotten a character on the current line
    clientLineIsBlank = false;
    clientLine[clientLinePointer] = c;
    clientLinePointer++;
    if(clientLinePointer >= STRING_LENGTH)
    {
      platform->Message(HOST_MESSAGE,"Client read buffer overflow.\n");
      clientLinePointer = 0;
      clientLine[clientLinePointer] = 0; 
    }
  }  
}

// Deal with input/output from/to the client (if any) one byte at a time.

void Webserver::Spin()
{
  //char sw[2];
  if(!active)
    return;
    
  if(writing)
  {
 //   if(inPHPFile)
 //     WritePHPByte();
 //   else
	if(platform->GetNetwork()->CanWrite())
      WriteByte();
	platform->ClassReport("Webserver", longWait);
    return;
  }
  
  char c;

  if(platform->GetNetwork()->Active())
  {
	  if(platform->GetNetwork()->Status() & clientConnected)
	  {
		  if(platform->GetNetwork()->Status() & byteAvailable)
		  {
			  platform->GetNetwork()->Read(c);
			  //SerialUSB.print(c);

			  if(receivingPost && postFile != NULL)
			  {
				  if(MatchBoundary(c))
				  {
					  //Serial.println("Got to end of file.");
					  postFile->Close();
					  SendFile(clientRequest);
					  clientRequest[0] = 0;
					  InitialisePost();
				  }
				  platform->ClassReport("Webserver", longWait);
				  return;
			  }

			  CharFromClient(c);
		  }
	  }
  }
   
  if (platform->GetNetwork()->Status() & clientLive)
  {
    if(needToCloseClient)
    {
      if(platform->Time() - clientCloseTime < CLIENT_CLOSE_DELAY)
      {
    	platform->ClassReport("Webserver", longWait);
        return;
      }
      needToCloseClient = false;  
      platform->GetNetwork()->Close();
    }   
  }
  platform->ClassReport("Webserver", longWait);
}

//******************************************************************************************

// Constructor and initialisation

Webserver::Webserver(Platform* p)
{ 
  platform = p;
  active = false;
  gotPassword = false;
}

void Webserver::Init()
{
  writing = false;
  receivingPost = false;
  postSeen = false;
  getSeen = false;
  jsonPointer = -1;
  clientLineIsBlank = true;
  needToCloseClient = false;
  clientLinePointer = 0;
  clientLine[0] = 0;
  clientRequest[0] = 0;
  SetPassword(DEFAULT_PASSWORD);
  SetName(DEFAULT_NAME);
  //gotPassword = false;
  gcodeAvailable = false;
  gcodePointer = 0;
  InitialisePost();
  lastTime = platform->Time();
  longWait = lastTime;
  active = true; 
  
  // Reinitialise the message file
  
  //platform->GetMassStorage()->Delete(platform->GetWebDir(), MESSAGE_FILE);
}

// This is called when the connection has been lost.
// In particular, we must cancel any pending writes.
void Webserver::ConnectionError()
{
	  writing = false;
	  receivingPost = false;
	  postSeen = false;
	  getSeen = false;
	  jsonPointer = -1;
	  clientLineIsBlank = true;
	  needToCloseClient = false;
	  clientLinePointer = 0;
	  clientLine[0] = 0;
	  clientRequest[0] = 0;
	  gotPassword = false;
	  gcodeAvailable = false;
	  gcodePointer = 0;
	  InitialisePost();
	  lastTime = platform->Time();
	  longWait = lastTime;
	  active = true;

}

void Webserver::Exit()
{
  platform->Message(HOST_MESSAGE, "Webserver class exited.\n");
  active = false;
}

void Webserver::Diagnostics() 
{
  platform->Message(HOST_MESSAGE, "Webserver Diagnostics:\n"); 
}

void Webserver::SetPassword(char* pw)
{
	strncpy(password, pw, SHORT_STRING_LENGTH);
	password[SHORT_STRING_LENGTH] = 0; // NB array is dimensioned to SHORT_STRING_LENGTH+1
}

void Webserver::SetName(char* nm)
{
	strncpy(myName, nm, SHORT_STRING_LENGTH);
	myName[SHORT_STRING_LENGTH] = 0; // NB array is dimensioned to SHORT_STRING_LENGTH+1
}




