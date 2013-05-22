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



boolean Webserver::MatchBoundary(char c)
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
      platform->Write(postFile, postBoundary[i]);
    platform->Write(postFile, c);
    boundaryCount = 0;
  }
  return false;  
}


//****************************************************************************************************

// Feeding G Codes to the GCodes class

boolean Webserver::GCodeAvailable()
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


boolean Webserver::LoadGcodeBuffer(char* gc, boolean convertWeb)
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
  
  char fileAct = 0;
  if(StringStartsWith(gcodeBuffer, "M30 ")) fileAct |= 1;
  if(StringStartsWith(gcodeBuffer, "M23 ")) fileAct |= 2;
  
  if(fileAct) // Delete or print a file?
  {
    if(fileAct == 1) // Delete?
    {
      if(!platform->DeleteFile(platform->PrependRoot(scratchString, platform->GetGcodeDir(), &gcodeBuffer[4])))
      {
        platform->Message(HOST_MESSAGE, "Unsuccsessful attempt to delete: ");
        platform->Message(HOST_MESSAGE, &gcodeBuffer[4]);
        platform->Message(HOST_MESSAGE, "\n");
      } 
    } else // Print it
    {
      reprap.GetGCodes()->QueueFileToPrint(platform->PrependRoot(scratchString, platform->GetGcodeDir(), &gcodeBuffer[4])); 
    }
    
    // Check for further G Codes in the string
    
    gcodePointer = 0;
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
  boolean zip = false;
    
  if(StringStartsWith(nameOfFileToSend, KO_START))
    GetJsonResponse(&nameOfFileToSend[KO_FIRST]);
    
  if(jsonPointer < 0)
  {
    fileBeingSent = platform->OpenFile(platform->PrependRoot(scratchString, platform->GetWebDir(), nameOfFileToSend), false);
    if(fileBeingSent < 0)
    {
      nameOfFileToSend = FOUR04_FILE;
      fileBeingSent = platform->OpenFile(platform->PrependRoot(scratchString, platform->GetWebDir(), nameOfFileToSend), false);
    }
    writing = true;
  } 
  
  platform->SendToClient("HTTP/1.1 200 OK\n");
  
  platform->SendToClient("Content-Type: ");
  
  if(StringEndsWith(nameOfFileToSend, ".png"))
    platform->SendToClient("image/png\n");
  else if(StringEndsWith(nameOfFileToSend, ".ico"))
    platform->SendToClient("image/x-icon\n");
  else if (jsonPointer >=0)
    platform->SendToClient("application/json\n");
  else if(StringEndsWith(nameOfFileToSend, ".js"))
    platform->SendToClient("application/javascript\n");
  else if(StringEndsWith(nameOfFileToSend, ".zip"))
  {
    platform->SendToClient("application/zip\n");
    zip = true;
  } else
    platform->SendToClient("text/html\n");
    
  if (jsonPointer >=0)
  {
    platform->SendToClient("Content-Length: ");
    sprintf(sLen, "%d", strlen(jsonResponse));
    platform->SendToClient(sLen);
    platform->SendToClient("\n");
  }
    
  if(zip)
  {
    platform->SendToClient("Content-Encoding: gzip\n");
    platform->SendToClient("Content-Length: ");
    sprintf(sLen, "%llu", platform->Length(fileBeingSent));
    platform->SendToClient(sLen);
    platform->SendToClient("\n");    
  }
    
  platform->SendToClient("Connnection: close\n");

  platform->SendToClient('\n');
}

void Webserver::WriteByte()
{
    unsigned char b;
    
    if(jsonPointer >= 0)
    {
      if(jsonResponse[jsonPointer])
        platform->SendToClient(jsonResponse[jsonPointer++]);
      else
      {
        jsonPointer = -1;
        jsonResponse[0] = 0;
        CloseClient();
      }
    } else
    {
      if(platform->Read(fileBeingSent, b))
        platform->SendToClient(b);
      else
      { 
        platform->Close(fileBeingSent);    
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


void Webserver::GetJsonResponse(char* request)
{
  jsonPointer = 0;
  writing = true;
  boolean ok = false;
  
  if(StringStartsWith(request, "name"))
  {
    strcpy(jsonResponse, "{\"myName\":\"");
    strcat(jsonResponse, myName);
    strcat(jsonResponse, "\"}");
    ok = true;
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
    ok = true;
  }
  
  if(StringStartsWith(request, "gcode"))
  {
    if(!LoadGcodeBuffer(&clientQualifier[6], true))
      platform->Message(HOST_MESSAGE, "Webserver: buffer not free!\n");
    strcpy(jsonResponse, "{}");
    ok = true;
  }
  
  if(StringStartsWith(request, "files"))
  {
    char* fileList = platform->FileList(platform->GetGcodeDir());
    strcpy(jsonResponse, "{\"files\":[");
    strcat(jsonResponse, fileList);
    strcat(jsonResponse, "]}");    
    ok = true;
  }
  
  if(ok)
  {
    platform->Message(HOST_MESSAGE, "JSON response: ");
    platform->Message(HOST_MESSAGE, jsonResponse);
    platform->Message(HOST_MESSAGE, " queued\n");
  } else
  { 
    platform->Message(HOST_MESSAGE, "KnockOut request: ");
    platform->Message(HOST_MESSAGE, request);
    platform->Message(HOST_MESSAGE, " not recognised\n");
    clientRequest[0] = 0;
  }
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
  postFile = -1;
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
    postFile = platform->OpenFile(platform->PrependRoot(scratchString, platform->GetGcodeDir(), postFileName), true);
    if(postFile < 0  || !postBoundary[0])
    {
      platform->Message(HOST_MESSAGE, "Can't open file for write or no post boundary: ");
      platform->Message(HOST_MESSAGE, platform->PrependRoot(scratchString, platform->GetGcodeDir(), postFileName));
      platform->Message(HOST_MESSAGE, "\n");
      InitialisePost();
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
  char sw[2];
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
  
  if(platform->ClientStatus() & CONNECTED)
  {
    if(platform->ClientStatus() & AVAILABLE) 
    {
      char c = platform->ClientRead();
//        Serial.print(c);

      if(receivingPost && postFile >= 0)
      {
        if(MatchBoundary(c))
        {
          //Serial.println("Got to end of file.");
          platform->Close(postFile);
          SendFile(clientRequest);
          clientRequest[0] = 0;
          InitialisePost();       
        }
        return;
      }  
      
      CharFromClient(c);
    }
  }  
   
  if (platform->ClientStatus() & CLIENT) 
  {
    if(needToCloseClient)
    {
      if(platform->Time() - clientCloseTime < CLIENT_CLOSE_DELAY)
        return;
      needToCloseClient = false;  
      platform->DisconnectClient();
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
  
  platform->DeleteFile(platform->PrependRoot(scratchString, platform->GetWebDir(), MESSAGE_FILE));
}

void Webserver::Exit()
{
  active = false;
}



