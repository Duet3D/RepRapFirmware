/****************************************************************************************************

RepRapFirmware - Webserver

This class serves web pages to the attached network.  These pages form the user's interface with the
RepRap machine.  It interprests returned values from those pages and uses them to Generate G Codes,
which it sends to the RepRap.  It also collects values from the RepRap like temperature and uses
those to construct the web pages.

It implements very very restricted PHP.  It can do:

   <?php print(myStringFunction()); ?>
   <?php if(myBooleanFunction()) print(myOtherStringFunction()); ?>
   <?php if(myOtherBooleanFunction()) echo 'Some arbitrarily long string of HTML including newlines up to this quote:'; ?>

Note that by printing a function that returns "" you can just call 
that function in this C++ code with no effect on the loaded web page.


-----------------------------------------------------------------------------------------------------

Version 0.1

13 February 2013

Adrian Bowyer
RepRap Professional Ltd
http://reprappro.com

Licence: GPL

****************************************************************************************************/

#include "RepRapFirmware.h"

//***************************************************************************************************

// String manipulation

boolean Webserver::StringEndsWith(char* string, char* ending)
{
  int j = strlen(string);
  int k = strlen(ending);
  if(k > j)
    return false;
  
  return(StringEquals(&string[j - k], ending));
}

boolean Webserver::StringEquals(char* s1, char* s2)
{
  int i = 0;
  while(s1[i] && s2[i])
  {
     if(tolower(s1[i]) != tolower(s2[i]))
       return false;
     i++;
  }
  
  return !(s1[i] || s2[i]);
}

boolean Webserver::StringStartsWith(char* string, char* starting)
{ 
  int j = strlen(string);
  int k = strlen(starting);
  if(k > j)
    return false;
  
  for(int i = 0; i < k; i++)
    if(string[i] != starting[i])
      return false;
      
  return true;
}

int Webserver::StringContains(char* string, char* match)
{ 
  int i = 0;
  int count = 0;
  
  while(string[i])
  {
    if(string[i++] == match[count])
    {
      count++;
      if(!match[count])
        return i;
    } else
      count = 0;
  }
      
  return -1;
}

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

boolean Webserver::Available()
{
  return gcodeAvailable;
}

byte Webserver::Read()
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
  if(gcodeAvailable)
    return false;
  
  if(strlen(gc) > GCODE_LENGTH-1)
  {
    platform->Message(HOST_MESSAGE, "Webserver: GCode buffer overflow.<br>\n");
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
  
// We intercept a G Code so we can deal with file manipulation.  That
// way things don't get out of sync.
  
  if(StringStartsWith(gcodeBuffer, "M30 ")) // Delete file?
  {
    if(!platform->DeleteFile(&gcodeBuffer[4]))
    {
      platform->Message(HOST_MESSAGE, "Unsuccsessful attempt to delete: ");
      platform->Message(HOST_MESSAGE, &gcodeBuffer[4]);
      platform->Message(HOST_MESSAGE, "<br>\n");
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
  inPHPFile = false;
  InitialisePHP();
  clientCloseTime = platform->Time();
  needToCloseClient = true;   
}


void Webserver::SendFile(char* nameOfFileToSend)
{  
  if(!gotPassword)
  {
    sendTable = false;
    nameOfFileToSend = PASSWORD_PAGE;
  } else
    sendTable = true;
    
  if(StringStartsWith(nameOfFileToSend, KO_START))
  {
    GetKOString(&nameOfFileToSend[KO_FIRST]);
    return;
  }
    
  platform->SendToClient("HTTP/1.1 200 OK\n");
  
  if(StringEndsWith(nameOfFileToSend, ".png"))
    platform->SendToClient("Content-Type: image/png\n");
  else
    platform->SendToClient("Content-Type: text/html\n");
    
  platform->SendToClient("Connnection: close\n");

  platform->SendToClient('\n');
  
  fileBeingSent = platform->OpenFile(platform->PrependRoot(platform->GetWebDir(), nameOfFileToSend), false);
  if(fileBeingSent < 0)
  {
    sendTable = false;
    nameOfFileToSend = FOUR04_FILE;
    fileBeingSent = platform->OpenFile(platform->PrependRoot(platform->GetWebDir(), nameOfFileToSend), false);
  }
  
  inPHPFile = StringEndsWith(nameOfFileToSend, ".php");
  if(inPHPFile)
    InitialisePHP();
  writing = true; 
}

void Webserver::WriteByte()
{
    unsigned char b;
    
    if(koPointer >= 0)
    {
      if(clientRequest[koPointer])
        platform->SendToClient(clientRequest[koPointer++]);
      else
      {
        koPointer = -1;
        clientRequest[0] = 0;
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
  if(!StringEndsWith(clientQualifier, password))
    return;
    
  gotPassword = true;
  strcpy(clientRequest, INDEX_PAGE);
}




void Webserver::GetKOString(char* request)
{
  koPointer = 0;
  writing = true;
  
  if(StringStartsWith(request, "name"))
  {
    strcpy(clientRequest, "{\"myName\":\"");
    strcat(clientRequest, myName);
    strcat(clientRequest, "\"}");
    platform->Message(HOST_MESSAGE, "KnockOut response: ");
    platform->Message(HOST_MESSAGE, clientRequest);
    platform->Message(HOST_MESSAGE, " queued<br>\n");
    return;
  }
  
  if(StringStartsWith(request, "page"))
  {
    strcpy(clientRequest, "{\"page\":\"");
    strcat(clientRequest, myName);  //FIXME
    strcat(clientRequest, "\"}");
    platform->Message(HOST_MESSAGE, "KnockOut response: ");
    platform->Message(HOST_MESSAGE, clientRequest);
    platform->Message(HOST_MESSAGE, " queued<br>\n");
    return;
  }
    
    
  platform->Message(HOST_MESSAGE, "KnockOut request: ");
  platform->Message(HOST_MESSAGE, request);
  platform->Message(HOST_MESSAGE, " not recognised<br>\n");
  koPointer = -1;
  writing = false; 
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
//    Serial.print("HTTP request: ");
//    Serial.println(clientLine);
  
    platform->Message(HOST_MESSAGE, "HTTP request: ");
    platform->Message(HOST_MESSAGE, clientLine);
    platform->Message(HOST_MESSAGE, "<br>\n");
    
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
      strcpy(clientRequest, PRINT_PAGE);
    return;
  }
  
  int bnd;
  
  if(postSeen && ( (bnd = StringContains(clientLine, "boundary=")) >= 0) )
  {
    if(strlen(&clientLine[bnd]) >= POST_LENGTH - 4)
    {
      platform->Message(HOST_MESSAGE, "Post boundary buffer overflow.<br>\n");
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
      platform->Message(HOST_MESSAGE, "Post disposition gives no filename.<br>\n");
      return;
    }
    int i = 0;
    while(clientLine[bnd] && clientLine[bnd] != '"')
    {
      postFileName[i++] = clientLine[bnd++];
      if(i >= POST_LENGTH)
      {
        i = 0;
        platform->Message(HOST_MESSAGE, "Post filename buffer overflow.<br>\n");
      }
    }
    postFileName[i] = 0;
    //Serial.print("Got file name: ");
    //Serial.println(postFileName);    
    return;
  }  
}
  

void Webserver::ParseQualifier()
{
  if(!clientQualifier[0])
    return;
    
  if(StringStartsWith(clientQualifier, "pwd="))
    CheckPassword();
  if(!gotPassword) //Doan work fur nuffink
    return;
    
  if(StringStartsWith(clientQualifier, "gcode="))
  {
    if(!LoadGcodeBuffer(&clientQualifier[6], true))
      platform->Message(HOST_MESSAGE, "Webserver: buffer not free!<br>\n");
    //strcpy(clientRequest, INDEX_PAGE);
  } 
}

// if you've gotten to the end of the line (received a newline
// character) and the line is blank, the http request has ended,
// so you can send a reply
void Webserver::BlankLineFromClient()
{
  clientLine[clientLinePointer] = 0;
  clientLinePointer = 0;
  ParseQualifier();
  
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
    postFile = platform->OpenFile(platform->PrependRoot(platform->GetGcodeDir(), postFileName), true);
    if(postFile < 0  || !postBoundary[0])
    {
      platform->Message(HOST_MESSAGE, "Can't open file for write or no post boundary: ");
      platform->Message(HOST_MESSAGE, platform->PrependRoot(platform->GetGcodeDir(), postFileName));
      platform->Message(HOST_MESSAGE, "<br>\n");
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
      platform->Message(HOST_MESSAGE,"Client read buffer overflow.<br>\n");
      clientLinePointer = 0;
      clientLine[clientLinePointer] = 0; 
    }
  }  
}

// Deal with input/output from/to the client (if any) one byte at a time.

void Webserver::Spin()
{
  if(!active)
    return;
    
  if(writing)
  {
    if(inPHPFile)
      WritePHPByte();
    else
      WriteByte();
    return;         
  }
  
  if(platform->ClientStatus() & CONNECTED)
  {
    if(platform->ClientStatus() & AVAILABLE) 
    {
      char c = platform->ClientRead();
      //Serial.write(c); 
 
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

//**********************************************************************************************

// PHP interpreter

void Webserver::InitialisePHP()
{
  phpTag[0] = 0;
  inPHPString = 0;
  phpPointer = 0;
  phpEchoing = false;
  phpIfing = false;
  phpPrinting = false;
  eatInput = false;
  recordInput = false;
  phpRecordPointer = 0;
  phpRecord[phpRecordPointer] = 0;
  ifWasTrue = true;
}

char Webserver::PHPParse(char* phpString)
{
  if(StringEquals(phpString, "if("))
    return PHP_IF;
    
  if(StringEquals(phpString, "echo"))
    return PHP_ECHO;   
  
  if(StringEquals(phpString, "print("))
    return PHP_PRINT;
  
  return NO_PHP;
}


boolean Webserver::PrintLinkTable() { boolean r = sendTable; sendTable = true; return r; }

boolean Webserver::CallPHPBoolean(char* phpRecord)
{ 
  if(StringEquals(phpRecord, "gotPassword("))
    return gotPassword;
    
  if(StringEquals(phpRecord, "printLinkTable("))
    return PrintLinkTable();
    
  platform->Message(HOST_MESSAGE, "callPHPBoolean(): non-existent function - ");
  platform->Message(HOST_MESSAGE, phpRecord);
  platform->Message(HOST_MESSAGE, "<br>\n");
  
  return true; // Best default
}

void Webserver::GetGCodeList()
{
  platform->SendToClient(platform->FileList(platform->GetGcodeDir()));
}

void Webserver::CallPHPString(char* phpRecord)
{
  if(StringEquals(phpRecord, "getMyName("))
  {
    platform->SendToClient(myName);
    return;
  }
    
  if(StringEquals(phpRecord, "getGCodeList("))
  {
    GetGCodeList();
    return;
  }  
    
  if(StringEquals(phpRecord, "logout("))
  {
    gotPassword = false;
    platform->SendToClient("<meta http-equiv=\"REFRESH\" content=\"0;url=");
    platform->SendToClient(PASSWORD_PAGE);
    platform->SendToClient("\"></HEAD>");
    return;
  }
    
  platform->Message(HOST_MESSAGE, "callPHPString(): non-existent function - ");
  platform->Message(HOST_MESSAGE, phpRecord);
  platform->Message(HOST_MESSAGE, "<br>\n"); 
}

void Webserver::ProcessPHPByte(char b)
{
  if(eatInput)
  {
    if(b == eatInputChar)  
      eatInput = false;
    return;
  }
  
  if(recordInput)
  {
    if(b == eatInputChar)
    {
      recordInput = false;
      phpRecordPointer = 0;
      return;
    }
    
    phpRecord[phpRecordPointer++] = b;
    if(phpRecordPointer >= PHP_TAG_LENGTH)
    {
      platform->Message(HOST_MESSAGE, "ProcessPHPByte: PHP record buffer overflow.<br>\n");
      InitialisePHP();
    }
    phpRecord[phpRecordPointer] = 0;
    return;
  }
  
  if(phpEchoing)
  {
     if(b != '\'')
     {
       if(ifWasTrue)
         platform->SendToClient(b);
     } else
     {
       InitialisePHP();
       eatInput = true;
       eatInputChar = '>';
     }
     return;
  }
  
  if(phpIfing)
  {
    boolean ifWas = CallPHPBoolean(phpRecord);
    InitialisePHP();
    ifWasTrue = ifWas;
    inPHPString = 5;
    if(b != ')')
    {
      eatInput = true;
      eatInputChar = ')';
    }
    return;
  }
  
  if(phpPrinting)
  {
    CallPHPString(phpRecord);
    InitialisePHP();
    eatInput = true;
    eatInputChar = '>';    
    return;
  }  
  
  if(inPHPString >= 5)
  {
  // We are in a PHP expression
  
    if(isspace(b))
      return;    
    phpTag[phpPointer++] = b;
    phpTag[phpPointer] = 0;
    if(phpPointer >= PHP_TAG_LENGTH)
    {
      platform->Message(HOST_MESSAGE, "ProcessPHPByte: PHP buffer overflow: ");
      platform->Message(HOST_MESSAGE, phpTag);
      platform->Message(HOST_MESSAGE, "<br>\n");
      InitialisePHP();
      return;
    }
    
    switch(PHPParse(phpTag))
    {
    case PHP_ECHO:
      phpEchoing = true;
      eatInput = true;
      eatInputChar = '\'';
      break;
      
    case PHP_IF:
      phpIfing = true;
      recordInput = true;
      phpRecordPointer = 0;
      phpRecord[phpRecordPointer] = 0;
      eatInputChar = ')';
      break;
    
    case PHP_PRINT:
      phpPrinting = true;
      recordInput = true;
      phpRecordPointer = 0;
      phpRecord[phpRecordPointer] = 0;
      eatInputChar = ')';
      break;     
     
    default:
      break;
    }

    return; 
  }
  
  // We are not in a PHP expression
  
  phpTag[inPHPString] = b;
  
  switch(inPHPString)
  {
  case 0:
    if(b == '<')
    {
      inPHPString = 1;
    } else
    {
      phpTag[1] = 0;
      inPHPString = 0;
      platform->SendToClient(phpTag); 
    }       
    return;
    
  case 1:
    if(b == '?')
    {
      inPHPString = 2;
    } else
    {
      phpTag[2] = 0;
      inPHPString = 0;
      platform->SendToClient(phpTag); 
    }
    return;
    
  case 2:
    if(b == 'p')
    {
      inPHPString = 3;
    } else
    {
      phpTag[3] = 0;
      inPHPString = 0;
      platform->SendToClient(phpTag); 
    }
    return;  
  
  case 3:
    if(b == 'h')
    {
      inPHPString = 4;
    } else
    {
      phpTag[4] = 0;
      inPHPString = 0;
      platform->SendToClient(phpTag); 
    }
    return;

  case 4:
    if(b == 'p')
    {
      inPHPString = 5;
      phpTag[0] = 0;
      phpPointer = 0;
    } else
    {
      phpTag[5] = 0;
      inPHPString = 0;
      platform->SendToClient(phpTag); 
    }
    return;
  
  // Should never get here...
  
  default:
     platform->Message(HOST_MESSAGE, "ProcessPHPByte: PHP tag runout.<br>\n");
     platform->SendToClient(b);
     InitialisePHP();
  }   
}

void Webserver::WritePHPByte()
{
    unsigned char b;
    if(platform->Read(fileBeingSent, b))
      ProcessPHPByte(b);
    else
    {     
      platform->Close(fileBeingSent);
      InitialisePHP();    
      CloseClient(); 
    }  
}

//******************************************************************************************

// Constructor and initialisation

Webserver::Webserver(Platform* p)
{
  //Serial.println("Webserver constructor"); 
  platform = p;
  active = false;
}

void Webserver::Init()
{
  lastTime = platform->Time();
  writing = false;
  receivingPost = false;
  postSeen = false;
  getSeen = false;
  koPointer = -1;
  //postLength = 0L;
  inPHPFile = false;
  InitialisePHP();
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
  sendTable = true;
  phpRecordPointer = 0;
  InitialisePost();
  active = true; 
}

void Webserver::Exit()
{
  active = false;
}



