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
  
  return(!strcmp(&string[j - k], ending));
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

char* Webserver::prependRoot(char* root, char* fileName)
{
  strcpy(scratchString, root);
  return strcat(scratchString, fileName);
}


//****************************************************************************************************

// Handling G Codes for the rest of the software

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
  gcodeBuffer[gcodePointer] = 0;
  gcodePointer = 0;  
  
// We intercept two G Codes so we can deal with file manipulation.  That
// way things don't get out of sync.
  
/*  if(StringStartsWith(gc, "M28")) // Upload file?
  {
    uploadFile(&gc[4]);
    return;
  }*/
  
  if(StringStartsWith(gcodeBuffer, "M30")) // Delete file?
  {
    if(!platform->deleteFile(&gcodeBuffer[4]))
    {
      platform->Message(HOST_MESSAGE, "Unsuccsessful attempt to delete: ");
      platform->Message(HOST_MESSAGE, &gcodeBuffer[4]);
      platform->Message(HOST_MESSAGE, "\n");
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
  initialisePHP();
  clientCloseTime = platform->time();
  needToCloseClient = true;   
}


void Webserver::SendFile(char* nameOfFileToSend)
{
//  Serial.print("Sending: ");
//  Serial.println(nameOfFileToSend);
  
  if(!gotPassword)
  {
    sendTable = false;
    nameOfFileToSend = PASSWORD_PAGE;
  } else
    sendTable = true;
    
  platform->SendToClient("HTTP/1.1 200 OK\n");
  
  if(StringEndsWith(nameOfFileToSend, ".png"))
    platform->SendToClient("Content-Type: image/png\n");
  else
    platform->SendToClient("Content-Type: text/html\n");
    
  platform->SendToClient("Connnection: close\n");
  
//          if(loadingImage)
//          {
//           platform->SendToHost("Cache-Control: max-age=3600\n");
//           Serial.println("Image requested");
//          }

  platform->SendToClient('\n');
  
//  if(InternalFile(nameOfFileToSend))
//    return;
  
  //Serial.print("File requested: ");
  //Serial.println(nameOfFileToSend);
  
  fileBeingSent = platform->OpenFile(prependRoot(platform->getWebDir(), nameOfFileToSend), false);
  if(fileBeingSent < 0)
  {
    sendTable = false;
    nameOfFileToSend = "html404.htm";
    fileBeingSent = platform->OpenFile(prependRoot(platform->getWebDir(), nameOfFileToSend), false);
  }
  
  inPHPFile = StringEndsWith(nameOfFileToSend, ".php");
  if(inPHPFile)
    initialisePHP();
  writing = true; 
}

void Webserver::WriteByte()
{
    unsigned char b;
    if(platform->Read(fileBeingSent, b))
      platform->SendToClient(b);
    else
    {     
      platform->Close(fileBeingSent);    
      CloseClient(); 
    }  
}

//----------------------------------------------------------------------------------------------------

// Input from the client

void Webserver::CheckPassword()
{
  if(!StringEndsWith(clientQualifier, password))
    return;
    
  gotPassword = true;
  strcpy(clientRequest, "control.php");
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
      strcpy(clientRequest, "control.php");
    return;
  }
  
  if(StringStartsWith(clientLine, "POST"))
  {
    ParseGetPost();
    InitialisePost();
    postSeen = true;
    getSeen = false;
    if(!clientRequest[0])
      strcpy(clientRequest, "print.php");
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
      platform->Message(HOST_MESSAGE, "Webserver: buffer not free!\n");
    //strcpy(clientRequest, "control.php");
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
    postFile = platform->OpenFile(prependRoot(platform->getGcodeDir(), postFileName), true);
    if(postFile < 0  || !postBoundary[0])
    {
      platform->Message(HOST_MESSAGE, "Can't open file for write or no post boundary: ");
      platform->Message(HOST_MESSAGE, prependRoot(platform->getGcodeDir(), postFileName));
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

void Webserver::spin()
{ 
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
      Serial.write(c); 
 
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
      if(platform->time() - clientCloseTime < CLIENT_CLOSE_DELAY)
        return;
      needToCloseClient = false;  
      platform->DisconnectClient();
    }   
  }
}

//**********************************************************************************************

// PHP interpreter

void Webserver::initialisePHP()
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
  if(!strcmp(phpString, "if("))
    return PHP_IF;
    
  if(!strcmp(phpString, "echo"))
    return PHP_ECHO;   
  
  if(!strcmp(phpString, "print("))
    return PHP_PRINT;
  
  return NO_PHP;
}


boolean Webserver::printLinkTable() { boolean r = sendTable; sendTable = true; return r; }

boolean Webserver::callPHPBoolean(char* phpRecord)
{ 
  if(!strcmp(phpRecord, "gotPassword("))
    return gotPassword;
    
  if(!strcmp(phpRecord, "printLinkTable("))
    return printLinkTable();
    
  platform->Message(HOST_MESSAGE, "callPHPBoolean(): non-existent function - ");
  platform->Message(HOST_MESSAGE, phpRecord);
  platform->Message(HOST_MESSAGE, "\n");
  
  return true; // Best default
}

void Webserver::getGCodeList()
{
  platform->SendToClient(platform->FileList(platform->getGcodeDir()));
}

void Webserver::callPHPString(char* phpRecord)
{
  if(!strcmp(phpRecord, "getMyName("))
  {
    platform->SendToClient(myName);
    return;
  }
    
  if(!strcmp(phpRecord, "getGCodeList("))
  {
    getGCodeList();
    return;
  }  
    
  if(!strcmp(phpRecord, "logout("))
  {
    gotPassword = false;
    platform->SendToClient("<meta http-equiv=\"REFRESH\" content=\"0;url=passwd.php\"></HEAD>");
    return;
  }
    
  platform->Message(HOST_MESSAGE, "callPHPString(): non-existent function - ");
  platform->Message(HOST_MESSAGE, phpRecord);
  platform->Message(HOST_MESSAGE, "\n"); 
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
      platform->Message(HOST_MESSAGE, "ProcessPHPByte: PHP record buffer overflow.\n");
      initialisePHP();
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
       initialisePHP();
       eatInput = true;
       eatInputChar = '>';
     }
     return;
  }
  
  if(phpIfing)
  {
    boolean ifWas = callPHPBoolean(phpRecord);
    initialisePHP();
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
    callPHPString(phpRecord);
    initialisePHP();
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
      platform->Message(HOST_MESSAGE, "\n");
      initialisePHP();
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
     platform->Message(HOST_MESSAGE, "ProcessPHPByte: PHP tag runout.\n");
     platform->SendToClient(b);
     initialisePHP();
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
      initialisePHP();    
      CloseClient(); 
    }  
}

//******************************************************************************************

// Constructor and initialisation

Webserver::Webserver(Platform* p)
{
  //Serial.println("Webserver constructor"); 
  platform = p;
  lastTime = platform->time();
  writing = false;
  receivingPost = false;
  postSeen = false;
  getSeen = false;
  postLength = 0L;
  inPHPFile = false;
  initialisePHP();
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
}




