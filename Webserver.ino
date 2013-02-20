/****************************************************************************************************

RepRapFirmware - Webserver

This class serves web pages to the attached network.  These pages form the user's interface with the
RepRap machine.  It interprests returned values from those pages and uses them to Generate G Codes,
which it sends to the RepRap.  It also collects values from the RepRap like temperature and uses
those to construct the web pages.

-----------------------------------------------------------------------------------------------------

Version 0.1

13 February 2013

Adrian Bowyer
RepRap Professional Ltd
http://reprappro.com

Licence: GPL

****************************************************************************************************/

#include "RepRapFirmware.h"

Webserver::Webserver(Platform* p)
{
  Serial.println("Webserver constructor"); 
  platform = p;
  lastTime = platform->time();
  writing = false;
  clientLineIsBlank = true;
  needToCloseClient = false;
  clientLinePointer = 0;
  clientLine[0] = 0;
  clientRequest[0] = 0;
  password = DEFAULT_PASSWORD;
  gotPassword = false;
}

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

void Webserver::SendFile(char* nameOfFileToSend)
{
  if(!gotPassword)
    nameOfFileToSend = PASSWORD_PAGE;
    
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
  
  //Serial.print("File requested: ");
  //Serial.println(nameOfFileToSend);
  
  fileBeingSent = platform->OpenFile(nameOfFileToSend, false);
  if(fileBeingSent < 0)
      fileBeingSent = platform->OpenFile("html404.htm", false);
      
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
      writing = false;
      clientCloseTime = platform->time();
      needToCloseClient = true; 
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

void Webserver::ParseClientLine()
{ 
  if(!StringStartsWith(clientLine, "GET"))
    return;
  
  int i = 5;
  int j = 0;
  clientRequest[j] = 0;
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
    ParseQualifier();
  }
  
  if(!clientRequest[0])
    strcpy(clientRequest, "index.htm");
}

void Webserver::CheckPassword()
{
  if(!StringEndsWith(clientQualifier, password))
    return;
    
  gotPassword = true;
  strcpy(clientRequest, "index.htm");
}
  

void Webserver::ParseQualifier()
{
  if(StringStartsWith(clientQualifier, "pwd="))
    CheckPassword();
  
}



void Webserver::spin()
{
    
  if(writing)
  {
    WriteByte();
    return;         
  }
  
  if(platform->ClientStatus() & CONNECTED)
  {
    if (platform->ClientStatus() & AVAILABLE) 
    {
      char c = platform->ClientRead();
      Serial.write(c);
      // if you've gotten to the end of the line (received a newline
      // character) and the line is blank, the http request has ended,
      // so you can send a reply
      if (c == '\n' && clientLineIsBlank) 
      {
        clientLine[clientLinePointer] = 0;
        clientLinePointer = 0;
        SendFile(clientRequest);
        clientRequest[0] = 0;
        return;
      }
      if (c == '\n') 
      {
        clientLine[clientLinePointer] = 0;
        ParseClientLine();
        // you're starting a new line
        clientLineIsBlank = true;
        clientLinePointer = 0;
      } else if (c != '\r') 
      {
        // you've gotten a character on the current line
        clientLineIsBlank = false;
        clientLine[clientLinePointer] = c;
        clientLinePointer++;
      }
    }
    //return;
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






