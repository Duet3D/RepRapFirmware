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
  
// We intercept two G Codes so we can deal with file manipulation.  That
// way things don't get out of sync, and - as a file name can contain
// a valid G code (!) - confusion is avoided.
  
  int8_t fileAct = 0;
  if(StringStartsWith(gcodeBuffer, "M30 ")) fileAct |= 1;
  if(StringStartsWith(gcodeBuffer, "M23 ")) fileAct |= 2;
  
  if(fileAct) // Delete or print a file?
  { 
    if(fileAct == 1) // Delete?
    {
      if(!platform->GetMassStorage()->Delete(platform->GetGCodeDir(), &gcodeBuffer[4]))
      {
        platform->Message(HOST_MESSAGE, "Unsuccsessful attempt to delete: ");
        platform->Message(HOST_MESSAGE, &gcodeBuffer[4]);
        platform->Message(HOST_MESSAGE, "\n");
      } 
    } else // Print it
    {
      reprap.GetGCodes()->QueueFileToPrint(&gcodeBuffer[4]);
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
    sprintf(sLen, "%d", strlen(jsonResponse));
    platform->GetNetwork()->Write(sLen);
    platform->GetNetwork()->Write("\n");
  }
    
  if(zip)
  {
	platform->GetNetwork()->Write("Content-Encoding: gzip\n");
	platform->GetNetwork()->Write("Content-Length: ");
    sprintf(sLen, "%llu", fileBeingSent->Length());
    platform->GetNetwork()->Write(sLen);
    platform->GetNetwork()->Write("\n");
  }
    
  platform->GetNetwork()->Write("Connnection: close\n");

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
    if(reprap.debug())
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
  
  if(StringStartsWith(request, "temps"))
  {
    strcpy(jsonResponse, "{\"temps\":[");
    for(int8_t heater = 0; heater < HEATERS; heater++)
    {
      strcat(jsonResponse, "\"");
      strcat(jsonResponse, ftoa(NULL, reprap.GetHeat()->GetTemperature(heater), 1));
      //sprintf(scratchString, "%d", (int)reprap.GetHeat()->GetTemperature(heater));
      //strcat(jsonResponse, scratchString);
      if(heater < HEATERS-1)
        strcat(jsonResponse, "\",");
      else
        strcat(jsonResponse, "\"");
    }
    strcat(jsonResponse, "]}");    
    JsonReport(true, request);
    return;
  }
  
  if(StringStartsWith(request, "gcode"))
  {
    if(!LoadGcodeBuffer(&clientQualifier[6], true))
      platform->Message(HOST_MESSAGE, "Webserver: buffer not free!\n");
    strcpy(jsonResponse, "{}");
    JsonReport(true, request);
    return;
  }
  
  if(StringStartsWith(request, "files"))
  {
    char* fileList = platform->GetMassStorage()->FileList(platform->GetGCodeDir());
    strcpy(jsonResponse, "{\"files\":[");
    strcat(jsonResponse, fileList);
    strcat(jsonResponse, "]}");    
    JsonReport(true, request);
    return;
  }
  
  if(StringStartsWith(request, "name"))
  {
    strcpy(jsonResponse, "{\"myName\":\"");
    strcat(jsonResponse, myName);
    strcat(jsonResponse, "\"}");
    JsonReport(true, request);
    return;
  }
  
  if(StringStartsWith(request, "password"))
  {
    CheckPassword();
    strcpy(jsonResponse, "{\"password\":\"");
    if(gotPassword)
      strcat(jsonResponse, "right");
    else
      strcat(jsonResponse, "wrong");
    strcat(jsonResponse, "\"}");   
    JsonReport(true, request);
    return;
  }
  
  if(StringStartsWith(request, "axes"))
  {
    strcpy(jsonResponse, "{\"axes\":[");
    for(int8_t drive = 0; drive < AXES; drive++)
    {
      strcat(jsonResponse, "\"");
      strcat(jsonResponse, ftoa(NULL, platform->AxisLength(drive), 1));
      if(drive < AXES-1)
        strcat(jsonResponse, "\",");
      else
        strcat(jsonResponse, "\"");
    }
    strcat(jsonResponse, "]}");    
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
    if(reprap.debug())
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
      strcpy(clientRequest, INDEX_PAGE);
    return;
  }
  
  if(StringStartsWith(clientLine, "POST"))
  {
    ParseGetPost();
    InitialisePost();
    postSeen = true;
    getSeen = false;
    if(!clientRequest[0])
      strcpy(clientRequest, INDEX_PAGE);
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
    strcpy(&postBoundary[2], &clientLine[bnd]);
    strcat(postBoundary, "--");
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
      WriteByte();
    return;         
  }
  
  char c;

  if(platform->GetNetwork()->Status() & clientConnected)
  {
    if(platform->GetNetwork()->Status() & byteAvailable)
    {
    	platform->GetNetwork()->Read(c);
//        Serial.print(c);

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
        return;
      }  
      
      CharFromClient(c);
    }
  }  
   
  if (platform->GetNetwork()->Status() & clientLive)
  {
    if(needToCloseClient)
    {
      if(platform->Time() - clientCloseTime < CLIENT_CLOSE_DELAY)
        return;
      needToCloseClient = false;  
      platform->GetNetwork()->Close();
    }   
  }
}

//******************************************************************************************

// Constructor and initialisation

Webserver::Webserver(Platform* p)
{ 
  platform = p;
  active = false;
}

void Webserver::Init()
{
  char scratchString[STRING_LENGTH];
  lastTime = platform->Time();
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
  password = DEFAULT_PASSWORD;
  myName = DEFAULT_NAME;
  gotPassword = false;
  gcodeAvailable = false;
  gcodePointer = 0;
  InitialisePost();
  active = true; 
  
  // Reinitialise the message file
  
  platform->GetMassStorage()->Delete(platform->GetWebDir(), MESSAGE_FILE);
}

void Webserver::Exit()
{
  active = false;
}

void Webserver::Diagnostics() 
{
  platform->Message(HOST_MESSAGE, "Webserver Diagnostics:\n"); 
}



